// 执行器描述与控制输入测试（M0-07）。
// 只通过 Terra 公共头访问后端，本文件不包含 MuJoCo 头文件。
//
// 夹具 hinge_motor.xml：铰链 shoulder（I = 0.1 kg·m²）+ motor drive（gear 1，ctrlrange [-1, 1]），
// 球关节 neck + orientation 执行器 aim（四元数输入，4 个控制量），以及排在 aim 之后、
// 同样作用于 shoulder 的 motor trim（ctrlrange [-0.2, 0.3]，控制地址 5 ≠ 执行器下标 2）。无重力、无接触。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/physics/physics_world.hpp"

#ifndef TERRA_TEST_ASSETS_DIR
#error "需要 TERRA_TEST_ASSETS_DIR"
#endif
#ifndef TERRA_TEST_WORK_DIR
#error "需要 TERRA_TEST_WORK_DIR"
#endif

namespace {

namespace fs = std::filesystem;
using terra::data::ActuatorId;
using terra::data::ActuatorTransmission;
using terra::data::JointId;
using terra::data::ModelSource;
using terra::physics::ActuatorCommand;
using terra::physics::ControlError;
using terra::physics::ControlErrorKind;
using terra::physics::LoadErrorKind;
using terra::physics::PhysicsWorld;

constexpr double kInertia = 0.1;  // hinge_motor.xml 中 arm 绕 Y 轴的转动惯量
constexpr ActuatorId kDrive{1};
constexpr ActuatorId kAim{2};
constexpr ActuatorId kTrim{3};

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

std::unique_ptr<PhysicsWorld> loadPath(const fs::path& path) {
    auto r = terra::physics::mujoco::loadWorld(ModelSource{path.string()});
    if (!r.world) {
        check(false, "加载 " + path.string() + " 失败：" + (r.error ? r.error->message : ""));
    }
    return std::move(r.world);
}

std::unique_ptr<PhysicsWorld> loadAsset(const std::string& file) {
    return loadPath(fs::path{TERRA_TEST_ASSETS_DIR} / file);
}

fs::path writeModel(const std::string& name, const std::string& xml) {
    const fs::path dir{TERRA_TEST_WORK_DIR};
    fs::create_directories(dir);
    const fs::path path = dir / name;
    std::ofstream{path} << xml;
    return path;
}

bool stepN(PhysicsWorld& world, int n) {
    for (int i = 0; i < n; ++i) {
        if (const auto error = world.step()) {
            check(false, "第 " + std::to_string(error->step) + " 步出错：" + error->message);
            return false;
        }
    }
    return true;
}

double hingeAngle(const PhysicsWorld& world) {
    return world.snapshot()->joints[0].position[0];
}

// 期望特定类型的控制错误，并检查错误中的执行器与输入下标。
void expectControlError(const std::optional<ControlError>& error, ControlErrorKind kind, ActuatorId actuator,
                        int input, const std::string& what) {
    check(error.has_value(), what + "：应返回错误");
    if (!error) return;
    check(error->kind == kind, what + "：错误类型不符，消息：" + error->message);
    check(error->actuator == actuator, what + "：错误中的执行器 ID 不符");
    check(error->input == input, what + "：错误中的输入下标不符（实际 " + std::to_string(error->input) + "）");
    check(!error->message.empty(), what + "：错误消息不应为空");
}

void testDescription() {
    auto world = loadAsset("hinge_motor.xml");
    if (!world) return;
    const auto& desc = world->description();
    check(desc.actuators.size() == 3, "三个执行器");
    if (desc.actuators.size() != 3) return;

    const auto* drive = desc.findActuator("drive");
    check(drive != nullptr && drive->id == kDrive, "drive 的 ID 为 1");
    check(drive && drive->transmission == ActuatorTransmission::Joint && drive->joint == JointId{1},
          "drive 作用于关节 shoulder");
    check(drive && drive->inputs.size() == 1 && drive->inputs[0].limited && drive->inputs[0].min == -1.0 &&
              drive->inputs[0].max == 1.0,
          "drive：1 个输入，范围 [-1, 1]");

    // 多输入执行器：控制量数不等于执行器数（M0-07 报告第 2 节）
    const auto* aim = desc.actuator(kAim);
    check(aim != nullptr && aim->name == "aim", "按 ID 查到 aim");
    check(aim && aim->transmission == ActuatorTransmission::Other && !aim->joint.valid(),
          "aim 的 SO3 传动归为 Other，不给出目标关节");
    check(aim && aim->inputs.size() == 4, "aim：4 个输入（四元数）");
    if (aim && aim->inputs.size() == 4) {
        check(aim->inputs[0].name == "qw" && aim->inputs[3].name == "qz", "aim 输入名称为 qw..qz");
    }
    // trim 的范围按它自己的控制地址读取（不是按执行器下标）
    const auto* trim = desc.findActuator("trim");
    check(trim && trim->id == kTrim && trim->joint == JointId{1} && trim->inputs.size() == 1 &&
              trim->inputs[0].min == -0.2 && trim->inputs[0].max == 0.3,
          "trim：作用于 shoulder，范围 [-0.2, 0.3]");
    check(desc.actuator(ActuatorId{}) == nullptr && desc.actuator(ActuatorId{4}) == nullptr, "无效 ID 查不到");

    auto cube = loadAsset("falling_cube.xml");
    check(cube && cube->description().actuators.empty(), "立方体没有执行器");
}

void testEmptyControl() {
    // 无执行器模型：空控制合法，且不改变任何结果（与从不调用逐位相同）。
    auto controlled = loadAsset("falling_cube.xml");
    auto plain = loadAsset("falling_cube.xml");
    if (!controlled || !plain) return;
    for (int i = 0; i < 1000; ++i) {
        check(!controlled->setControl({}).has_value(), "空控制成功");
        if (!stepN(*controlled, 1)) return;
    }
    if (!stepN(*plain, 1000)) return;
    const auto a = controlled->snapshot()->entities[0];
    const auto b = plain->snapshot()->entities[0];
    check(a.pose.position.x == b.pose.position.x && a.pose.position.z == b.pose.position.z &&
              a.pose.orientation.w == b.pose.orientation.w && a.linearVelocity.z == b.linearVelocity.z,
          "空控制与不设置控制的轨迹逐位相同");
}

void testHingeResponse() {
    // 恒定力矩下的离散解析解（半隐式 Euler）：a = u/I，ωₖ = a·dt·k，θₖ = a·dt²·k(k+1)/2。
    for (const double u : {0.5, -0.5, 0.0}) {
        auto world = loadAsset("hinge_motor.xml");
        if (!world) return;
        const double dt = world->description().timestep;
        const double a = u / kInertia;
        // 只设置一次：控制应持续生效
        check(!world->setControl({{kDrive, {u}}}).has_value(), "设置 drive = " + std::to_string(u));
        double maxAngleError = 0.0;
        double maxVelocityError = 0.0;
        for (int k = 1; k <= 1000; ++k) {
            if (!stepN(*world, 1)) return;
            const auto snap = world->snapshot();
            const double kk = static_cast<double>(k);
            maxAngleError = std::max(maxAngleError, std::fabs(snap->joints[0].position[0] - a * dt * dt * kk * (kk + 1) / 2));
            maxVelocityError = std::max(maxVelocityError, std::fabs(snap->joints[0].velocity[0] - a * dt * kk));
        }
        const std::string tag = "u = " + std::to_string(u);
        check(maxAngleError <= 1e-9, tag + "：角度与解析解误差 <= 1e-9，实际 " + std::to_string(maxAngleError));
        check(maxVelocityError <= 1e-9, tag + "：角速度与解析解误差 <= 1e-9，实际 " + std::to_string(maxVelocityError));

        // 方向：正力矩绕 +Y 正转；实体角速度（世界坐标）与关节角速度一致
        const auto snap = world->snapshot();
        const double omega = snap->joints[0].velocity[0];
        if (u > 0) check(omega > 0 && hingeAngle(*world) > 0, tag + "：正向转动");
        if (u < 0) check(omega < 0 && hingeAngle(*world) < 0, tag + "：反向转动");
        if (u == 0) check(omega == 0 && hingeAngle(*world) == 0, tag + "：保持静止");
        check(std::fabs(snap->entities[0].angularVelocity.y - omega) <= 1e-12 &&
                  snap->entities[0].angularVelocity.x == 0 && snap->entities[0].angularVelocity.z == 0,
              tag + "：实体角速度 = (0, ω, 0)");
        // 球关节与 arm 无耦合，aim 保持中性值时不动
        check(snap->joints[1].velocity[0] == 0 && snap->joints[1].velocity[1] == 0 &&
                  snap->joints[1].velocity[2] == 0,
              tag + "：球关节不受影响");
    }
}

void testMultiInputAndPartialUpdate() {
    auto world = loadAsset("hinge_motor.xml");
    if (!world) return;
    check(!world->setControl({{kDrive, {0.5}}}).has_value(), "设置 drive");
    // 绕 +Z 转 90° 的目标姿态；只更新 aim，drive 应保持原值
    const double h = std::sqrt(0.5);
    check(!world->setControl({{kAim, {h, 0, 0, h}}}).has_value(), "设置 aim（四元数输入）");
    if (!stepN(*world, 100)) return;
    const auto snap = world->snapshot();
    check(snap->entities[1].angularVelocity.z > 0, "aim：头部开始绕 +Z 转向目标");
    check(std::fabs(snap->entities[1].angularVelocity.x) <= 1e-12 && std::fabs(snap->entities[1].angularVelocity.y) <= 1e-12,
          "aim：只绕 Z 轴转动");
    const double a = 0.5 / kInertia;
    const double dt = world->description().timestep;
    check(std::fabs(snap->joints[0].velocity[0] - a * dt * 100) <= 1e-9, "部分更新：drive 保持 0.5");

    // trim 写入自己的控制地址：铰链加速度变为 (0.5 + 0.25) / I，aim 的目标不受影响
    const double omegaBefore = snap->joints[0].velocity[0];
    const double headBefore = snap->entities[1].angularVelocity.z;
    check(!world->setControl({{kTrim, {0.25}}}).has_value(), "设置 trim");
    if (!stepN(*world, 1)) return;
    const auto after = world->snapshot();
    check(std::fabs(after->joints[0].velocity[0] - omegaBefore - (0.75 / kInertia) * dt) <= 1e-12,
          "trim 与 drive 叠加作用于铰链");
    check(after->entities[1].angularVelocity.z > headBefore && std::fabs(after->entities[1].angularVelocity.x) <= 1e-12,
          "设置 trim 不改变 aim 的目标");
}

void testLastWriteWins() {
    auto world = loadAsset("hinge_motor.xml");
    if (!world) return;
    check(!world->setControl({{kDrive, {0.9}}}).has_value() && !world->setControl({{kDrive, {-0.3}}}).has_value(),
          "同一步之前两次设置");
    if (!stepN(*world, 1)) return;
    const double dt = world->description().timestep;
    check(std::fabs(world->snapshot()->joints[0].velocity[0] - (-0.3 / kInertia) * dt) <= 1e-12, "后写覆盖先写");
}

void testResetRestoresNeutral() {
    auto world = loadAsset("hinge_motor.xml");
    if (!world) return;
    const double h = std::sqrt(0.5);
    check(!world->setControl({{kDrive, {0.8}}, {kAim, {h, h, 0, 0}}}).has_value(), "设置两个执行器");
    if (!stepN(*world, 200)) return;
    check(hingeAngle(*world) > 0, "reset 前已转动");

    world->reset();
    // 中性值：drive 为 0，aim 为单位四元数（等于初始姿态），因此两个关节都应保持静止。
    // 若 aim 被清成全零，目标姿态无意义，球关节会被驱动或出现坏值。
    if (!stepN(*world, 500)) return;
    const auto snap = world->snapshot();
    check(snap->joints[0].position[0] == 0 && snap->joints[0].velocity[0] == 0, "reset 后 drive 为 0：铰链静止");
    const auto& q = snap->joints[1].position;
    check(std::fabs(q[0] - 1) <= 1e-12 && std::fabs(q[1]) <= 1e-12 && std::fabs(q[2]) <= 1e-12 &&
              std::fabs(q[3]) <= 1e-12,
          "reset 后 aim 为单位四元数：球关节保持初始姿态");
}

void testSnapshotUnaffected() {
    auto world = loadAsset("hinge_motor.xml");
    if (!world) return;
    if (!stepN(*world, 10)) return;
    const auto before = world->snapshot();
    check(!world->setControl({{kDrive, {1.0}}}).has_value(), "设置控制");
    check(world->snapshot() == before, "设置控制不使当前快照失效（同一对象）");
    check(world->stepIndex() == 10, "设置控制不推进仿真");
    if (!stepN(*world, 1)) return;
    check(world->snapshot() != before && before->stepIndex == 10, "下一步后才是新快照，旧快照不变");
}

void testValidation() {
    auto world = loadAsset("hinge_motor.xml");
    if (!world) return;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    expectControlError(world->setControl({{ActuatorId{}, {0.1}}}), ControlErrorKind::UnknownActuator, ActuatorId{}, -1,
                       "无效 ID 0");
    expectControlError(world->setControl({{ActuatorId{4}, {0.1}}}), ControlErrorKind::UnknownActuator, ActuatorId{4},
                       -1, "不存在的 ID 4");
    expectControlError(world->setControl({{kDrive, {0.1}}, {kDrive, {0.2}}}), ControlErrorKind::DuplicateActuator,
                       kDrive, -1, "同一次调用重复");
    expectControlError(world->setControl({{kDrive, {}}}), ControlErrorKind::SizeMismatch, kDrive, -1, "drive 给 0 个值");
    expectControlError(world->setControl({{kDrive, {0.1, 0.2}}}), ControlErrorKind::SizeMismatch, kDrive, -1,
                       "drive 给 2 个值");
    expectControlError(world->setControl({{kAim, {1, 0, 0}}}), ControlErrorKind::SizeMismatch, kAim, -1,
                       "aim 给 3 个值（需要 4 个）");
    expectControlError(world->setControl({{kDrive, {nan}}}), ControlErrorKind::NonFinite, kDrive, 0, "NaN");
    expectControlError(world->setControl({{kAim, {1, 0, -inf, 0}}}), ControlErrorKind::NonFinite, kAim, 2, "-Inf");
    expectControlError(world->setControl({{kDrive, {1.5}}}), ControlErrorKind::OutOfRange, kDrive, 0, "超出上限");
    expectControlError(world->setControl({{kAim, {1, 0, 0, -1.01}}}), ControlErrorKind::OutOfRange, kAim, 3,
                       "超出下限");
    check(!world->setControl({{kDrive, {1.0}}}).has_value() && !world->setControl({{kDrive, {-1.0}}}).has_value(),
          "边界值 ±1 合法（闭区间）");
    check(!world->setControl({{kTrim, {0.3}}}).has_value() && !world->setControl({{kTrim, {-0.2}}}).has_value(),
          "trim 非对称边界值合法");
    expectControlError(world->setControl({{kTrim, {0.5}}}), ControlErrorKind::OutOfRange, kTrim, 0,
                       "trim 按自己的范围判定（0.5 在 aim 的范围内，但超出 trim 的上限）");

    // 原子性：一批中后一项非法时，前一项也不写入
    auto fresh = loadAsset("hinge_motor.xml");
    if (!fresh) return;
    expectControlError(fresh->setControl({{kDrive, {0.5}}, {kAim, {2, 0, 0, 0}}}), ControlErrorKind::OutOfRange, kAim,
                       0, "批量中有一项越界");
    if (!stepN(*fresh, 100)) return;
    check(hingeAngle(*fresh) == 0, "原子性：整批被拒绝，drive 未被写入");

    // 未限制范围的执行器：任意有限值都接受，但超出后端有效数值范围（1e10）时拒绝
    auto unlimited = loadPath(writeModel("unlimited_motor.xml", R"(
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody><body><joint name="j" type="hinge" axis="0 1 0"/><geom type="sphere" size="0.1"/></body></worldbody>
  <actuator><motor name="free_motor" joint="j"/></actuator>
</mujoco>)"));
    if (!unlimited) return;
    const auto& in = unlimited->description().actuators.at(0).inputs.at(0);
    check(!in.limited, "未设置 ctrlrange 的执行器为不限范围");
    check(!unlimited->setControl({{ActuatorId{1}, {1e6}}}).has_value(), "不限范围：1e6 合法");
    expectControlError(unlimited->setControl({{ActuatorId{1}, {2e10}}}), ControlErrorKind::OutOfRange, ActuatorId{1}, 0,
                       "不限范围：超出 1e10");
    check(stepN(*unlimited, 10), "合法的大控制量不会触发后端故障");
}

void testFaultedWorld() {
    auto world = loadAsset("fault_runaway.xml");
    if (!world) return;
    for (int i = 0; i < 20 && !world->faulted(); ++i) {
        (void)world->step();
    }
    check(world->faulted(), "runaway 进入故障状态");
    expectControlError(world->setControl({}), ControlErrorKind::Faulted, ActuatorId{}, -1, "故障状态下设置控制");
    world->reset();
    check(!world->setControl({}).has_value(), "reset 后可以设置控制");
}

void testUnsupportedActuators() {
    const auto r = terra::physics::mujoco::loadWorld(ModelSource{writeModel("delayed_motor.xml", R"(
<mujoco>
  <worldbody><body><joint name="j" type="hinge"/><geom type="sphere" size="0.1"/></body></worldbody>
  <actuator><motor name="lagging" joint="j" nsample="4" delay="0.002"/></actuator>
</mujoco>)").string()});
    check(!r.world && r.error && r.error->kind == LoadErrorKind::Unsupported, "带控制延迟的执行器：加载返回 Unsupported");
    check(r.error && r.error->message.find("lagging") != std::string::npos &&
              r.error->message.find("延迟") != std::string::npos,
          "错误消息指明执行器和原因：" + (r.error ? r.error->message : std::string{}));
}

}  // namespace

int main() {
    testDescription();
    testEmptyControl();
    testHingeResponse();
    testMultiInputAndPartialUpdate();
    testLastWriteWins();
    testResetRestoresNeutral();
    testSnapshotUnaffected();
    testValidation();
    testFaultedWorld();
    testUnsupportedActuators();
    if (failures == 0) {
        std::printf("all passed\n");
    }
    return failures == 0 ? 0 : 1;
}
