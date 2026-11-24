// 仿真推进（M2-06）与 MuJoCo 后端集成。PRD F-M2-02 的核心判据：
//   同一控制序列在不同显示帧率下产生容差内一致的物理轨迹——这里要求逐位相同：
//   小车场景（两辆车 + 地面，按步序号安排的变化控制）与下落立方体（接触）各在 30 / 60 / 144 / 240 Hz 以及带抖动的
//   帧间隔下运行到同一步数，逐步比较全部实体位姿。
// 另覆盖：暂停时物理时间不增长、单步、reset 屏障（命令队列清空、显示不插值到旧世界）、迟到命令、被拒绝的命令、
// 插值显示位姿位于两份快照之间、步进错误后不再推进。本文件不包含 MuJoCo 头文件。

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/scene/scene.hpp"
#include "terra/simulation/runner.hpp"

#ifndef TERRA_TEST_ASSETS_DIR
#error "需要 TERRA_TEST_ASSETS_DIR"
#endif

namespace {

namespace fs = std::filesystem;
namespace sim = terra::simulation;
namespace sc = terra::scene;
namespace tp = terra::physics;
namespace td = terra::data;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

const std::string kDir = (fs::path{TERRA_TEST_ASSETS_DIR} / "scene").lexically_normal().string();

std::unique_ptr<tp::PhysicsWorld> cartsWorld() {
    auto loaded = sc::loadScene(kDir + "/two_carts.terra-scene.json", tp::mujoco::composeWorld);
    check(loaded.ok(), "加载两辆小车场景");
    if (!loaded.ok()) return nullptr;
    return std::move(loaded.scene->world);
}

std::unique_ptr<tp::PhysicsWorld> cubeWorld() {
    auto r = tp::mujoco::loadWorld(td::ModelSource{(fs::path{TERRA_TEST_ASSETS_DIR} / "falling_cube.xml").string()});
    check(r.world != nullptr, "加载 falling_cube.xml");
    return std::move(r.world);
}

// 按步序号安排的控制：每 100 步改变两辆车的推力（与帧率无关）
void scheduleDrives(sim::SimulationRunner& runner, const td::ModelDescription& d, std::uint64_t steps) {
    const auto* a = d.findActuator("cart_a/drive");
    const auto* b = d.findActuator("cart_b/drive");
    if (a == nullptr || b == nullptr) return;
    for (std::uint64_t k = 0; k < steps; k += 100) {
        const double u = 4.0 * std::sin(0.37 * static_cast<double>(k / 100) + 0.3);
        runner.schedule(k, {{a->id, {u}}, {b->id, {-0.5 * u}}});
    }
}

using Trajectory = std::vector<std::vector<td::Pose>>;  // 每步所有实体的位姿

// 以给定的帧间隔序列驱动 runner，直到世界走完 steps 步；记录每一步的快照
Trajectory run(tp::PhysicsWorld& world, bool drives, std::uint64_t steps, const std::vector<double>& frameIntervals,
               std::uint64_t* frames) {
    sim::SimulationRunner runner(world);
    if (drives) scheduleDrives(runner, world.description(), steps);
    Trajectory trajectory;
    runner.setStepObserver([&](const td::PhysicsSnapshot& s) {
        std::vector<td::Pose> poses;
        for (const auto& e : s.entities) poses.push_back(e.pose);
        trajectory.push_back(std::move(poses));
    });
    std::uint64_t n = 0;
    while (world.stepIndex() < steps && n < 1000000) {
        const auto r = runner.update(frameIntervals[n % frameIntervals.size()]);
        check(!r.error && r.controlErrors.empty(), "推进无错误");
        if (r.error) break;
        ++n;
    }
    if (frames != nullptr) *frames = n;
    trajectory.resize(std::min<std::size_t>(trajectory.size(), steps));
    return trajectory;
}

bool identical(const Trajectory& a, const Trajectory& b, std::size_t* firstDiff) {
    if (a.size() != b.size()) {
        *firstDiff = std::min(a.size(), b.size());
        return false;
    }
    for (std::size_t s = 0; s < a.size(); ++s) {
        for (std::size_t e = 0; e < a[s].size(); ++e) {
            const td::Pose& p = a[s][e];
            const td::Pose& q = b[s][e];
            if (p.position.x != q.position.x || p.position.y != q.position.y || p.position.z != q.position.z ||
                p.orientation.w != q.orientation.w || p.orientation.x != q.orientation.x ||
                p.orientation.y != q.orientation.y || p.orientation.z != q.orientation.z) {
                *firstDiff = s;
                return false;
            }
        }
    }
    return true;
}

void testFrameRateIndependence(const char* label, std::unique_ptr<tp::PhysicsWorld> (*make)(), bool drives,
                               std::uint64_t steps) {
    std::vector<std::vector<double>> schedules;
    for (const int hz : {30, 60, 144, 240}) schedules.push_back({1.0 / hz});
    // 带抖动的帧间隔（确定的伪随机序列，4–40 ms，含偶发的 150 ms 卡顿，会触发追赶上限）
    std::vector<double> jitter;
    std::uint32_t seed = 12345;
    for (int i = 0; i < 97; ++i) {
        seed = seed * 1664525u + 1013904223u;
        jitter.push_back(0.004 + 0.036 * static_cast<double>(seed >> 8) / static_cast<double>(1u << 24));
    }
    jitter[50] = 0.15;
    schedules.push_back(jitter);
    const char* names[] = {"30 Hz", "60 Hz", "144 Hz", "240 Hz", "抖动"};

    Trajectory reference;
    for (std::size_t i = 0; i < schedules.size(); ++i) {
        auto world = make();
        if (!world) return;
        std::uint64_t frames = 0;
        const Trajectory t = run(*world, drives, steps, schedules[i], &frames);
        check(t.size() == steps, std::string{label} + " " + names[i] + "：记录 " + std::to_string(t.size()) + " 步");
        std::printf("INFO %s %s：%llu 帧推进 %zu 步\n", label, names[i], static_cast<unsigned long long>(frames), t.size());
        if (i == 0) {
            reference = t;
            continue;
        }
        std::size_t diff = 0;
        check(identical(reference, t, &diff), std::string{label} + "：" + names[i] + " 与 30 Hz 的轨迹逐位相同" +
                                                  (identical(reference, t, &diff) ? "" : "（第 " + std::to_string(diff) + " 步起不同）"));
    }
}

void testPauseStepReset() {
    auto world = cartsWorld();
    if (!world) return;
    sim::SimulationRunner runner(*world);
    const auto& d = world->description();
    const auto* a = d.findActuator("cart_a/drive");
    if (a == nullptr) return;
    runner.scheduleNext({{a->id, {3.0}}});
    for (int i = 0; i < 30; ++i) runner.update(1.0 / 60);
    const std::uint64_t before = world->stepIndex();
    check(before > 200, "运行 0.5 秒后约 250 步，实际 " + std::to_string(before));

    // 显示位姿位于两份快照之间（小车在加速前进，x 单调增加）
    runner.update(0.001);  // 留下余量
    const std::uint64_t mid = world->stepIndex();
    const auto shown = runner.displayPoses();
    const auto* chassis = d.findEntity("cart_a/chassis");
    if (chassis != nullptr && runner.previous() && runner.latest()) {
        const double x0 = runner.previous()->entities[chassis->id.value - 1].pose.position.x;
        const double x1 = runner.latest()->entities[chassis->id.value - 1].pose.position.x;
        const double xs = shown[chassis->id.value - 1].position.x;
        check(x0 < x1 && xs >= x0 && xs <= x1, "显示位姿在上一份与最新快照之间（晚约一步）");
    }

    // 暂停：物理时间不变，显示最新快照
    runner.pause();
    const double t0 = world->simulationTime();
    for (int i = 0; i < 20; ++i) check(runner.update(1.0 / 60).steps == 0, "暂停中不推进");
    check(world->simulationTime() == t0, "暂停时物理时间不增长");
    if (chassis != nullptr) {
        check(runner.displayPoses()[chassis->id.value - 1].position.x == runner.latest()->entities[chassis->id.value - 1].pose.position.x,
              "暂停时显示最新快照");
    }
    runner.stepOnce();
    check(runner.update(0.0).steps == 1 && world->stepIndex() == mid + 1, "暂停中单步恰好 1 步");
    runner.resume();

    // 迟到命令与被拒绝的命令
    runner.schedule(0, {{a->id, {1.0}}});  // 第 0 步早已过去
    const auto late = runner.update(0.002);
    check(runner.lateCommands() == 1 && late.controlErrors.empty(), "安排在过去的命令在下一步之前生效，计为迟到");
    runner.scheduleNext({{a->id, {99.0}}});  // 超出 ctrlrange [−5, 5]
    const auto rejected = runner.update(0.002);
    check(rejected.controlErrors.size() == 1 && rejected.controlErrors[0].kind == tp::ControlErrorKind::OutOfRange && !rejected.error,
          "超出范围的命令被拒绝并报告，仿真继续");

    // reset 屏障
    runner.schedule(world->stepIndex() + 5, {{a->id, {2.0}}});
    check(runner.pendingCommands() == 1, "队列中有 1 条待执行命令");
    runner.reset();
    check(world->stepIndex() == 0 && world->runEpoch() == 1, "世界已 reset");
    check(runner.pendingCommands() == 0, "reset 清空命令队列");
    check(!runner.previous() && runner.latest() && runner.latest()->runEpoch == 1, "reset 后快照历史只剩新代次的初始快照");
    runner.update(0.003);  // 1 步
    check(runner.previous() && runner.previous()->runEpoch == 1, "reset 后的第一步只与新代次的快照插值");
    if (chassis != nullptr) {
        check(std::fabs(runner.displayPoses()[chassis->id.value - 1].position.x) < 1e-3, "reset 后显示回到初始位置附近");
    }
}

void testFault() {
    // 夹具 fault_runaway（步长 1 s）：几步之内发散，step 返回错误；之后 runner 不再推进。每帧 1 s 墙钟 = 1 步
    auto r = tp::mujoco::loadWorld(td::ModelSource{(fs::path{TERRA_TEST_ASSETS_DIR} / "fault_runaway.xml").string()});
    check(r.world != nullptr, "加载 fault_runaway.xml");
    if (!r.world) return;
    sim::SimulationRunner runner(*r.world);
    bool sawError = false;
    for (int i = 0; i < 200 && !sawError; ++i) sawError = runner.update(1.0).error.has_value();
    check(sawError && runner.faulted(), "步进错误被报告，世界进入故障状态");
    const std::uint64_t at = r.world->stepIndex();
    check(runner.update(1.0).steps == 0 && r.world->stepIndex() == at, "故障后不再推进");
    runner.reset();
    check(!runner.faulted() && runner.update(1.0).steps == 1, "reset 后恢复推进");
}

// 起始关键帧（M3-02）：设置后 reset 回到关键帧；resetToKeyframe 同样形成屏障；越界下标不做任何事
void testHomeKeyframe() {
    auto r = tp::mujoco::loadWorld(td::ModelSource{(fs::path{TERRA_TEST_ASSETS_DIR} / "robot_description.xml").string()});
    check(r.world != nullptr, "加载 robot_description.xml");
    if (!r.world) return;
    tp::PhysicsWorld& world = *r.world;
    const auto& d = world.description();
    const auto* base = d.findEntity("base");
    const int raised = d.findKeyframe("raised");
    if (base == nullptr || raised < 0) {
        check(false, "夹具中有实体 base 与关键帧 raised");
        return;
    }
    sim::SimulationRunner runner(world);
    check(!runner.homeKeyframe(), "默认没有起始关键帧");
    runner.setHomeKeyframe(static_cast<std::size_t>(raised));
    for (int i = 0; i < 10; ++i) runner.update(1.0 / 60);
    const auto* motor = d.findActuator("extend_motor");
    if (motor != nullptr) runner.schedule(world.stepIndex() + 3, {{motor->id, {0.5}}});
    runner.reset();
    const auto& pose = runner.latest()->entities[base->id.value - 1].pose;
    check(std::fabs(pose.position.z - 0.5) < 1e-9 && std::fabs(pose.position.x - 0.2) < 1e-9,
          "设置起始关键帧后 reset 回到关键帧（基座 z = 0.5），实际 z = " + std::to_string(pose.position.z));
    check(world.stepIndex() == 0 && runner.pendingCommands() == 0 && !runner.previous(), "按关键帧 reset 同样形成屏障");

    for (int i = 0; i < 10; ++i) runner.update(1.0 / 60);
    runner.scheduleNext({});
    check(runner.resetToKeyframe(0), "按关键帧 rest 重置");
    check(std::fabs(runner.latest()->entities[base->id.value - 1].pose.position.z - 0.4) < 1e-9 && runner.pendingCommands() == 0 &&
              !runner.previous() && world.stepIndex() == 0,
          "resetToKeyframe：回到关键帧 rest（z = 0.4），清空队列与快照历史");
    const std::uint64_t epoch = world.runEpoch();
    check(!runner.resetToKeyframe(99) && world.runEpoch() == epoch, "越界的关键帧下标：返回 false，不重置");

    runner.setHomeKeyframe(std::nullopt);
    runner.resetToKeyframe(static_cast<std::size_t>(raised));
    runner.reset();
    check(std::fabs(runner.latest()->entities[base->id.value - 1].pose.position.z - 0.4) < 1e-9,
          "取消起始关键帧后 reset 回到模型初值");
}

}  // namespace

int main() {
    testFrameRateIndependence("两辆小车", cartsWorld, true, 1500);
    testFrameRateIndependence("下落立方体", cubeWorld, false, 2000);
    testPauseStepReset();
    testFault();
    testHomeKeyframe();
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
