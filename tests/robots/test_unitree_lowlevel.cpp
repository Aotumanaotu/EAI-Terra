// 宇树低层控制场景（M4-06b，robots.unitree_lowlevel）。不需要 ROS2；Menagerie 的 Go2、G1 缺失时返回 77 表示跳过。
//   - g1_lowlevel：torqueActuators 把 29 个位置执行器改为力矩执行器（增益 1、无偏置），控制范围等于原来的力限
//     （左髋俯仰 ±88、左膝 ±139、左踝 ±50 N·m）；悬挂（没有自由关节）；关节限位不变；
//     对照 g1_suspended（不改执行器）：仍是位置执行器；
//     控制量 1 N·m 作用在左膝上，执行器输出为 1（力矩执行器的语义）；零控制推进 1 s 状态有限；
//   - go2_lowlevel：场景文件在安装点 go2/imu 添加 IMU；12 个力矩执行器；没有 PD 层。
// 本文件不包含 MuJoCo 头文件。

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>

#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/scene/scene.hpp"

#ifndef TERRA_ASSETS_DIR
#error "需要 TERRA_ASSETS_DIR"
#endif

namespace {

namespace sc = terra::scene;
namespace td = terra::data;

constexpr int kSkipped = 77;
int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

sc::SceneLoadResult load(const char* scene) {
    auto loaded = sc::loadScene(std::string{TERRA_ASSETS_DIR} + "/scenes/robots/" + scene + ".terra-scene.json",
                                terra::physics::mujoco::composeWorld);
    for (const auto& issue : loaded.issues) {
        std::printf("INFO [%s] %s %s：%s\n", scene, issue.severity == sc::SceneIssue::Severity::Error ? "错误" : "警告", issue.where.c_str(),
                    issue.message.c_str());
    }
    check(loaded.ok(), std::string{scene} + "：加载成功");
    return loaded;
}

void testG1() {
    auto loaded = load("g1_lowlevel");
    if (!loaded.ok()) return;
    auto& world = *loaded.scene->world;
    const auto& d = world.description();
    std::size_t motors = 0;
    std::size_t positions = 0;
    std::size_t free = 0;
    for (const auto& a : d.actuators) {
        motors += a.kind == td::ActuatorKind::Motor && a.gain == 1.0 ? 1 : 0;
        positions += a.kind == td::ActuatorKind::Position ? 1 : 0;
    }
    for (const auto& j : d.joints) free += j.type == td::JointType::Free ? 1 : 0;
    std::printf("INFO g1_lowlevel：执行器 %zu（力矩 %zu、位置 %zu），关节 %zu（自由 %zu）\n", d.actuators.size(), motors, positions, d.joints.size(), free);
    check(d.actuators.size() == 29 && motors == 29 && positions == 0, "g1_lowlevel：29 个执行器全部改为力矩执行器（增益 1）");
    check(free == 0 && d.joints.size() == 29, "g1_lowlevel：悬挂（没有自由关节），29 个关节");

    struct Range {
        const char* actuator;
        double limit;
    };
    for (const Range& r : {Range{"g1/left_hip_pitch_joint", 88.0}, Range{"g1/left_knee_joint", 139.0}, Range{"g1/left_ankle_pitch_joint", 50.0}}) {
        const auto* a = d.findActuator(r.actuator);
        check(a != nullptr && a->inputs.size() == 1 && a->inputs[0].limited && a->inputs[0].min == -r.limit && a->inputs[0].max == r.limit,
              std::string{r.actuator} + "：控制范围为原来的力限 ±" + std::to_string(r.limit));
    }
    const auto* knee = d.findJoint("g1/left_knee_joint");
    check(knee != nullptr && knee->limited && knee->rangeMax > 2.0, "g1_lowlevel：关节限位不变");

    // 控制量 1 → 执行器输出 1（力矩），推进 1 s 状态有限
    const auto* kneeMotor = d.findActuator("g1/left_knee_joint");
    if (kneeMotor != nullptr) {
        check(!world.setControl({{kneeMotor->id, {1.0}}}).has_value(), "g1_lowlevel：左膝的控制量可以设置");
        check(!world.step().has_value(), "g1_lowlevel：推进一步");
        const auto snap = world.snapshot();
        const auto index = static_cast<std::size_t>(kneeMotor - d.actuators.data());
        check(index < snap->actuators.size() && std::fabs(snap->actuators[index].force - 1.0) < 1e-12,
              "g1_lowlevel：控制量 1 时执行器输出 1 N·m（力矩执行器）");
        check(!world.setControl({{kneeMotor->id, {0.0}}}).has_value(), "g1_lowlevel：清零");
    }
    for (int i = 0; i < 500; ++i) {
        if (world.step().has_value()) {
            check(false, "g1_lowlevel：零控制推进出错");
            break;
        }
    }
    check(world.snapshot()->diagnostics.finite, "g1_lowlevel：零控制推进 1 s，状态有限");

    // 对照：不加 torqueActuators 的悬挂场景仍是位置执行器
    auto plain = load("g1_suspended");
    if (plain.ok()) {
        std::size_t p = 0;
        for (const auto& a : plain.scene->world->description().actuators) p += a.kind == td::ActuatorKind::Position ? 1 : 0;
        check(p == 29, "g1_suspended（对照）：29 个位置执行器不变");
    }
}

void testGo2() {
    auto loaded = load("go2_lowlevel");
    if (!loaded.ok()) return;
    const auto& d = loaded.scene->world->description();
    std::size_t motors = 0;
    for (const auto& a : d.actuators) motors += a.kind == td::ActuatorKind::Motor ? 1 : 0;
    const auto* imu = d.findSensor("go2/imu");
    check(motors == 12 && d.actuators.size() == 12, "go2_lowlevel：12 个力矩执行器");
    check(imu != nullptr && imu->added, "go2_lowlevel：场景文件在 go2/imu 添加了 IMU");
    check(loaded.scene->pd.empty(), "go2_lowlevel：没有 PD 层（LowCmd 直接给力矩）");
}

}  // namespace

int main() {
    for (const char* f : {"unitree_g1/g1.xml", "unitree_go2/go2.xml"}) {
        if (!std::filesystem::exists(std::string{TERRA_ASSETS_DIR} + "/robots/" + f)) {
            std::printf("Menagerie 模型未准备（assets/robots/%s），跳过；请运行 scripts/assets/fetch_robots.sh\n", f);
            return kSkipped;
        }
    }
    testG1();
    testGo2();
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
