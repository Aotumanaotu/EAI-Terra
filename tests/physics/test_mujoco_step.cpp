// MujocoWorld 固定步进与 reset 测试（M0-05）。
// 只通过 Terra 公共头访问后端，本文件不包含 MuJoCo 头文件。
//
// 数值门槛来自开发计划 4.2 节的候选容差（尚未冻结，M0-08 正式评审）。
// 速度、接触相关的判定需要 M0-06 的状态读取，不在本测试中。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "terra/core/run_schedule.hpp"
#include "terra/physics/mujoco/mujoco_world.hpp"

#ifndef TERRA_TEST_ASSETS_DIR
#error "需要 TERRA_TEST_ASSETS_DIR"
#endif

namespace {

using terra::data::EntityId;
using terra::data::ModelSource;
using terra::data::Pose;
using terra::physics::mujoco::MujocoWorld;
using terra::physics::mujoco::SimulationErrorKind;

constexpr double kGravity = 9.81;
const std::string kAssets = TERRA_TEST_ASSETS_DIR;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

std::unique_ptr<MujocoWorld> loadWorld(const std::string& file) {
    auto r = MujocoWorld::load(ModelSource{kAssets + "/" + file});
    if (!r.world) {
        check(false, "加载 " + file + " 失败：" + (r.error ? r.error->message : ""));
    }
    return std::move(r.world);
}

Pose cubePose(const MujocoWorld& world) {
    const auto pose = world.entityPose(EntityId{1});
    check(pose.has_value(), "cube 有位姿");
    return pose.value_or(Pose{});
}

// 连续推进 n 步；出错时记录失败并返回 false。
bool stepN(MujocoWorld& world, std::uint64_t n) {
    for (std::uint64_t i = 0; i < n; ++i) {
        if (const auto error = world.step()) {
            check(false, "第 " + std::to_string(error->step) + " 步出错：" + error->message);
            return false;
        }
    }
    return true;
}

void testTimeAndDerivedState() {
    auto world = loadWorld("falling_cube.xml");
    if (!world) return;
    const double dt = world->description().timestep;
    check(world->stepIndex() == 0 && world->runEpoch() == 0 && !world->faulted(), "加载后：第 0 步、代次 0、无故障");

    // 第 1 步：半隐式 Euler 先更新速度再更新位置，z1 = 1 - g·dt²。
    // 若步后没有刷新派生量，位姿会停留在 1.0（积分前的值）。
    check(!world->step().has_value(), "第 1 步成功");
    check(world->stepIndex() == 1, "stepIndex 为 1");
    const double z1 = cubePose(*world).position.z;
    check(std::fabs(z1 - (1.0 - kGravity * dt * dt)) <= 1e-15,
          "第 1 步后位姿已刷新：z = 1 - g·dt²，实际 " + std::to_string(z1));

    // 时间按步长增长，严格单调；N 步后 |t - N·dt| <= 1e-8（开发计划 4.2）
    double previous = world->simulationTime();
    bool monotonic = true;
    for (std::uint64_t n = 2; n <= 5000; ++n) {
        if (world->step()) {
            check(false, "第 " + std::to_string(n) + " 步出错");
            return;
        }
        const double t = world->simulationTime();
        monotonic = monotonic && t > previous;
        previous = t;
        if (std::fabs(t - static_cast<double>(n) * dt) > 1e-8) {
            check(false, "第 " + std::to_string(n) + " 步时间误差超过 1e-8：" + std::to_string(t));
            return;
        }
    }
    check(monotonic, "时间严格单调");
    check(world->stepIndex() == 5000, "stepIndex 为 5000");
}

void testFreeFallAndRest() {
    auto world = loadWorld("falling_cube.xml");
    if (!world) return;
    const double dt = world->description().timestep;

    // 碰撞前 t ∈ [0.05, 0.20]：与解析解 z = 1 - g·t²/2 的误差 <= 0.005 m，水平方向无漂移
    double maxError = 0.0;
    double maxDrift = 0.0;
    double previousZ = 1.0;
    bool descending = true;
    for (std::uint64_t n = 1; n <= 200; ++n) {
        if (!stepN(*world, 1)) return;
        const Pose p = cubePose(*world);
        descending = descending && p.position.z < previousZ;
        previousZ = p.position.z;
        maxDrift = std::max(maxDrift, std::max(std::fabs(p.position.x), std::fabs(p.position.y)));
        if (n >= 50) {
            const double t = static_cast<double>(n) * dt;
            maxError = std::max(maxError, std::fabs(p.position.z - (1.0 - 0.5 * kGravity * t * t)));
        }
    }
    check(descending, "碰撞前持续下落");
    check(maxError <= 0.005, "自由落体误差 <= 0.005 m，实际 " + std::to_string(maxError));
    check(maxDrift <= 1e-12, "水平方向无漂移，实际 " + std::to_string(maxDrift));

    // 运行到 5 s。最后 0.5 s 内中心高度距 0.1 m 不超过 0.005 m，姿态保持水平。
    // 线/角速度与有效接触的判定在 M0-06/08。
    double maxHeightError = 0.0;
    double minW = 1.0;
    for (std::uint64_t n = 201; n <= 5000; ++n) {
        if (!stepN(*world, 1)) return;
        if (n > 4500) {
            const Pose p = cubePose(*world);
            maxHeightError = std::max(maxHeightError, std::fabs(p.position.z - 0.1));
            minW = std::min(minW, std::fabs(p.orientation.w));
        }
    }
    check(maxHeightError <= 0.005, "稳定窗口内高度误差 <= 0.005 m，实际 " + std::to_string(maxHeightError));
    check(minW >= 1.0 - 1e-6, "稳定窗口内姿态接近初始（|w| >= 1 - 1e-6），实际 " + std::to_string(minW));
}

void testReset() {
    auto world = loadWorld("falling_cube.xml");
    if (!world) return;

    if (!stepN(*world, 5000)) return;
    const Pose reference = cubePose(*world);
    const double referenceTime = world->simulationTime();

    for (std::uint64_t cycle = 1; cycle <= 20; ++cycle) {
        world->reset();
        check(world->runEpoch() == cycle, "reset 后代次加 1（第 " + std::to_string(cycle) + " 次）");
        check(world->stepIndex() == 0 && world->simulationTime() == 0.0, "reset 后步序号与时间归零");
        const Pose initial = cubePose(*world);
        check(initial.position.x == 0.0 && initial.position.y == 0.0 && initial.position.z == 1.0 &&
                  initial.orientation.w == 1.0,
              "reset 后回到初始位姿");
        if (!stepN(*world, 5000)) return;
        const Pose p = cubePose(*world);
        // 同一进程、同一环境下重复运行：逐位一致（开发计划候选容差为 1e-9，这里要求更严）
        check(p.position.x == reference.position.x && p.position.y == reference.position.y &&
                  p.position.z == reference.position.z && p.orientation.w == reference.orientation.w &&
                  p.orientation.x == reference.orientation.x && p.orientation.y == reference.orientation.y &&
                  p.orientation.z == reference.orientation.z && world->simulationTime() == referenceTime,
              "第 " + std::to_string(cycle) + " 次 reset 后的轨迹与首次运行逐位一致");
    }

    // 运行中途 reset
    world->reset();
    if (!stepN(*world, 123)) return;
    world->reset();
    check(world->stepIndex() == 0 && cubePose(*world).position.z == 1.0, "中途 reset 回到初始状态");
}

void testObservationDoesNotAffectSimulation() {
    // 一个世界每步都读取状态，另一个只在结束时读取；结果必须逐位相同。
    auto observed = loadWorld("falling_cube.xml");
    auto silent = loadWorld("falling_cube.xml");
    if (!observed || !silent) return;
    for (int i = 0; i < 1000; ++i) {
        if (!stepN(*observed, 1)) return;
        (void)cubePose(*observed);
        (void)observed->simulationTime();
    }
    if (!stepN(*silent, 1000)) return;
    const Pose a = cubePose(*observed);
    const Pose b = cubePose(*silent);
    check(a.position.z == b.position.z && a.orientation.w == b.orientation.w &&
              observed->simulationTime() == silent->simulationTime(),
          "读取状态不影响仿真结果");
}

void testFaults() {
    // 状态发散：第 5 步 |qpos| > 1e10，由 Terra 检测（MuJoCo 要到下一步开始时才会发现）
    if (auto world = loadWorld("fault_runaway.xml")) {
        std::optional<terra::physics::mujoco::SimulationError> error;
        for (int i = 0; i < 20 && !error; ++i) {
            error = world->step();
        }
        check(error.has_value(), "runaway：应报错");
        if (error) {
            check(error->kind == SimulationErrorKind::NonFinite, "runaway：错误类型为 NonFinite");
            check(error->step == 5, "runaway：在第 5 步报错，实际 " + std::to_string(error->step));
        }
        check(world->faulted() && world->stepIndex() == 4, "runaway：进入故障状态，stepIndex 停在最后正常的第 4 步");

        const auto again = world->step();
        check(again && again->kind == SimulationErrorKind::Faulted, "故障后继续 step 返回 Faulted");
        check(world->stepIndex() == 4, "故障后 step 不推进");

        world->reset();
        check(!world->faulted() && world->stepIndex() == 0 && world->runEpoch() == 1, "reset 清除故障");
        check(!world->step().has_value(), "reset 后可以继续推进");
    }

    // 加速度坏值：MuJoCo 在第 1 步内自动重置状态；Terra 必须报告，而不是悄悄从 t=0 继续
    if (auto world = loadWorld("fault_bad_acceleration.xml")) {
        const auto error = world->step();
        check(error.has_value(), "bad acceleration：应报错");
        if (error) {
            check(error->kind == SimulationErrorKind::AutoReset, "bad acceleration：错误类型为 AutoReset");
            check(error->step == 1, "bad acceleration：在第 1 步报错");
        }
        check(world->faulted() && world->stepIndex() == 0, "bad acceleration：进入故障状态");
    }
}

void testRunSchedule() {
    const terra::RunSchedule s{10, 4};
    std::vector<std::uint64_t> sampled;
    for (std::uint64_t i = 0; i <= 10; ++i) {
        if (s.shouldSample(i)) sampled.push_back(i);
    }
    check(sampled == std::vector<std::uint64_t>{0, 4, 8, 10}, "采样：第 0 步、每 4 步、最后一步");
    const terra::RunSchedule sparse{3, 100};
    check(sparse.shouldSample(0) && !sparse.shouldSample(1) && sparse.shouldSample(3), "采样间隔大于总步数");
    const terra::RunSchedule every{5, 1};
    bool all = true;
    for (std::uint64_t i = 0; i <= 5; ++i) all = all && every.shouldSample(i);
    check(all, "间隔为 1 时每步都采样");
}

}  // namespace

int main() {
    testTimeAndDerivedState();
    testFreeFallAndRest();
    testReset();
    testObservationDoesNotAffectSimulation();
    testFaults();
    testRunSchedule();
    if (failures == 0) {
        std::printf("all passed\n");
    }
    return failures == 0 ? 0 : 1;
}
