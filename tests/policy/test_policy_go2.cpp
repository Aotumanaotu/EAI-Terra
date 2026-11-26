// 用户的 Go2 策略在 Terra 中运行（M4-10a，policy.go2_walk；设计稿第 3 节的验收）。策略文件只读，不在仓库中；
// 策略文件或 ONNX Runtime 组件不在时跳过（返回 77）。场景 assets/scenes/robots/go2_policy.terra-scene.json（陀螺带噪声）。
//   1. 自动配置（ONNX 元数据 + task_config.json）：打印自动完成的项与警告；
//   2. 站立 10 s：机身高度、倾角、水平漂移；
//   3. 前进指令 0.5 m/s：先走 2 s 进入步态，再量 10 s 的平均速度（沿机身朝向）与侧向速度；
//   4. 转向指令 1.0 rad/s：5 s 的平均偏航角速度；
//   阈值按第一次基线运行（2026-11-25）冻结并留余量，见各检查的说明与 M4-10a 报告；
//   5. 同一机器上两次运行逐位相同（3 s，含推理与噪声）。
// 结果另写一行 JSON 到指定文件，供验证报告使用。
// 用法：test_policy_go2 <组件目录> <policy.onnx> <结果文件>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <string>

#include "terra/library/components.hpp"
#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/policy/autoconfig.hpp"
#include "terra/policy/backend.hpp"
#include "terra/policy/controller.hpp"
#include "terra/scene/scene.hpp"

#ifndef TERRA_ASSETS_DIR
#error "需要 TERRA_ASSETS_DIR"
#endif

namespace {

namespace fs = std::filesystem;
namespace pol = terra::policy;
namespace sc = terra::scene;
namespace td = terra::data;

int failures = 0;

void check(bool condition, const std::string& what) {
    std::printf("%s %s\n", condition ? "  ok  " : "FAILED", what.c_str());
    if (!condition) ++failures;
}

double yawOf(const td::Quaternion& q) { return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)); }
// 机身 z 轴与竖直方向的夹角（度）
double tiltOf(const td::Quaternion& q) {
    const double zz = 1 - 2 * (q.x * q.x + q.y * q.y);
    return std::acos(std::clamp(zz, -1.0, 1.0)) * 180.0 / 3.14159265358979323846;
}

struct Run {
    sc::SceneInstance scene;
    std::unique_ptr<pol::PolicyController> controller;
    std::size_t base = 0;
};

const std::string kScene = TERRA_ASSETS_DIR "/scenes/robots/go2_policy.terra-scene.json";

bool makeRun(Run& run, const pol::PolicyDescription& policy, const pol::BackendContext& context, std::string& error) {
    auto loaded = sc::loadScene(kScene, terra::physics::mujoco::composeWorld);
    if (!loaded.scene) {
        error = "场景无法加载";
        return false;
    }
    run.scene = std::move(*loaded.scene);
    auto model = pol::loadModel(policy, context);
    if (!model.model) {
        error = model.error;
        return false;
    }
    run.controller = pol::PolicyController::create(policy, std::move(model.model), run.scene, true, error);
    const auto* base = run.scene.world->description().findEntity("go2/base");
    if (base == nullptr) error = "没有 go2/base";
    else run.base = base->id.value - 1;
    return run.controller != nullptr && base != nullptr;
}

// 推进 n 步；返回是否全程没有出错
bool advance(Run& run, int steps, const std::function<void(const td::PhysicsSnapshot&)>& each = {}) {
    auto& world = *run.scene.world;
    for (int i = 0; i < steps; ++i) {
        const auto commands = run.controller->commands(*world.snapshot());
        if (run.controller->failed()) {
            std::printf("策略停止：%s\n", run.controller->error().c_str());
            return false;
        }
        if (world.setControl(commands) || world.step()) return false;
        if (each) each(*world.snapshot());
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 4) {
        std::fprintf(stderr, "用法：test_policy_go2 <组件目录> <policy.onnx> <结果文件>\n");
        return 2;
    }
    pol::BackendContext context;
    context.componentsDirectory = argv[1];
    context.catalogPath = TERRA_ASSETS_DIR "/components/components.json";
    const std::string onnx = argv[2];
    std::error_code ec;
    if (!fs::is_regular_file(onnx, ec)) {
        std::printf("SKIP: 没有用户的 Go2 策略（%s）\n", onnx.c_str());
        return 77;
    }
    const auto catalog = terra::library::readComponentCatalog(context.catalogPath);
    const auto* info = catalog.find("onnxruntime-cpu");
    if (!info || terra::library::componentStatus(*info, context.componentsDirectory).state != terra::library::ComponentState::Installed) {
        std::printf("SKIP: 没有安装 ONNX Runtime 组件（%s）\n", argv[1]);
        return 77;
    }

    // 1. 自动配置
    Run probe;
    {
        auto loaded = sc::loadScene(kScene, terra::physics::mujoco::composeWorld);
        if (!loaded.scene) return 1;
        probe.scene = std::move(*loaded.scene);
    }
    const auto ac = pol::autoconfigure({onnx, kScene, ""}, probe.scene.world->description());
    std::printf("格式：%s\n", ac.format.c_str());
    for (const auto& n : ac.notes) std::printf("  自动配置：%s\n", n.c_str());
    for (const auto& w : ac.warnings) std::printf("  警告：%s\n", w.c_str());
    for (const auto& e : ac.errors) std::printf("  错误：%s\n", e.c_str());
    check(ac.policy && ac.policy->inputWidth() == 235 && ac.policy->actionWidth() == 12, "自动配置：235 维输入（47 × 5）、12 个动作");
    if (!ac.policy) return 1;
    const pol::PolicyDescription& policy = *ac.policy;

    std::string error;
    Run run;
    check(makeRun(run, policy, context, error), "控制器：" + error);
    if (!run.controller) return 1;
    std::printf("后端：%s；降频 %d\n", run.controller->model().describe().c_str(), run.controller->decimation());
    const double dt = run.scene.world->description().timestep;
    const auto stepsFor = [&](double seconds) { return static_cast<int>(std::lround(seconds / dt)); };

    // 2. 站立 10 s
    double minHeight = 1e9, maxTilt = 0.0;
    const auto start = run.scene.world->snapshot()->entities[run.base].pose.position;
    bool ok = advance(run, stepsFor(10.0), [&](const td::PhysicsSnapshot& s) {
        const auto& e = s.entities[run.base];
        minHeight = std::min(minHeight, e.pose.position.z);
        maxTilt = std::max(maxTilt, tiltOf(e.pose.orientation));
    });
    const auto stood = run.scene.world->snapshot()->entities[run.base].pose.position;
    const double drift = std::hypot(stood.x - start.x, stood.y - start.y);
    std::printf("站立 10 s：最低高度 %.3f m，最大倾角 %.1f°，水平漂移 %.3f m\n", minHeight, maxTilt, drift);
    check(ok && minHeight > 0.25 && maxTilt < 10.0 && drift < 0.05, "站立 10 s：高度 > 0.25 m、倾角 < 10°、漂移 < 0.05 m（2026-11-25 冻结；首次 0.270 m、2.5°、0.009 m）");

    // 3. 前进 0.5 m/s
    run.controller->setCommand({0.5, 0.0, 0.0});
    ok = advance(run, stepsFor(2.0));
    auto s0 = *run.scene.world->snapshot();
    const double heading = yawOf(s0.entities[run.base].pose.orientation);
    double walkMinHeight = 1e9, walkMaxTilt = 0.0;
    ok = ok && advance(run, stepsFor(10.0), [&](const td::PhysicsSnapshot& s) {
        walkMinHeight = std::min(walkMinHeight, s.entities[run.base].pose.position.z);
        walkMaxTilt = std::max(walkMaxTilt, tiltOf(s.entities[run.base].pose.orientation));
    });
    auto s1 = *run.scene.world->snapshot();
    const auto& p0 = s0.entities[run.base].pose.position;
    const auto& p1 = s1.entities[run.base].pose.position;
    const double forward = ((p1.x - p0.x) * std::cos(heading) + (p1.y - p0.y) * std::sin(heading)) / 10.0;
    const double lateral = (-(p1.x - p0.x) * std::sin(heading) + (p1.y - p0.y) * std::cos(heading)) / 10.0;
    std::printf("前进指令 0.5 m/s：平均前进速度 %.3f m/s，侧向 %.3f m/s，最低高度 %.3f m，最大倾角 %.1f°\n", forward, lateral, walkMinHeight, walkMaxTilt);
    check(ok && walkMinHeight > 0.2 && walkMaxTilt < 15.0, "行走 10 s：高度 > 0.2 m、倾角 < 15°（首次 0.286 m、4.9°）");
    check(forward > 0.3 && forward < 0.6 && std::fabs(lateral) < 0.05, "前进 0.3～0.6 m/s、侧向 < 0.05 m/s（冻结；首次 0.382、−0.001）");

    // 4. 转向 1.0 rad/s
    run.controller->setCommand({0.0, 0.0, 1.0});
    ok = advance(run, stepsFor(1.0));
    double yawRate = 0.0, lastYaw = yawOf(run.scene.world->snapshot()->entities[run.base].pose.orientation), turned = 0.0;
    ok = ok && advance(run, stepsFor(5.0), [&](const td::PhysicsSnapshot& s) {
        const double y = yawOf(s.entities[run.base].pose.orientation);
        double dy = y - lastYaw;
        if (dy > 3.14159265358979323846) dy -= 2 * 3.14159265358979323846;
        if (dy < -3.14159265358979323846) dy += 2 * 3.14159265358979323846;
        turned += dy;
        lastYaw = y;
    });
    yawRate = turned / 5.0;
    std::printf("转向指令 1.0 rad/s：平均偏航角速度 %.3f rad/s\n", yawRate);
    check(ok && yawRate > 0.4 && yawRate < 1.2, "转向 0.4～1.2 rad/s，方向正确（冻结；首次 0.657）");

    // 5. 两次运行逐位相同（3 s：站立 1 s + 前进 2 s）
    Run a, b;
    bool same = makeRun(a, policy, context, error) && makeRun(b, policy, context, error);
    for (Run* r : {&a, &b}) {
        if (!same) break;
        same = advance(*r, stepsFor(1.0));
        r->controller->setCommand({0.5, 0.0, 0.0});
        same = same && advance(*r, stepsFor(2.0));
    }
    if (same) {
        const auto sa = a.scene.world->snapshot();
        const auto sb = b.scene.world->snapshot();
        for (std::size_t i = 0; same && i < sa->joints.size(); ++i) same = sa->joints[i].position == sb->joints[i].position;
        same = same && a.controller->lastOutput() == b.controller->lastOutput();
    }
    check(same, "两次运行逐位相同（3 s，含推理与 IMU 噪声）");

    std::ofstream out(argv[3]);
    out << "{\"stand_min_height\":" << minHeight << ",\"stand_max_tilt_deg\":" << maxTilt << ",\"stand_drift\":" << drift
        << ",\"walk_forward\":" << forward << ",\"walk_lateral\":" << lateral << ",\"walk_min_height\":" << walkMinHeight
        << ",\"walk_max_tilt_deg\":" << walkMaxTilt << ",\"turn_yaw_rate\":" << yawRate << ",\"bitwise\":" << (same ? "true" : "false")
        << ",\"inference_us\":" << run.controller->lastInferenceSeconds() * 1e6 << "}\n";

    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
