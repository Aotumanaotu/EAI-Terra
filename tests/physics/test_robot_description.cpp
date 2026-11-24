// 机器人描述（M3-02，模型描述修订 3）。只通过 Terra 公共头访问后端，本文件不包含 MuJoCo 头文件。
// 夹具 assets/tests/robot_description.xml：
//   - 关节：限位、轴向、位置（所属实体坐标系）、阻尼、电枢惯量、摩擦损失；
//   - 执行器类别：力矩（Motor）、位置（Position，kp / kv）、速度（Velocity，kv）、带动力学的（Other）；传动比与力限制；
//   - 关键帧：名字与时间；按关键帧重置后的位置、速度、控制量；仿真时间与步序号从 0 起算；越界下标不改变状态；
//   - 组合（两个实例，第二个带平移与偏航）：关键帧名带实例前缀；自由关节位姿相对实例放置框架；
//     实例自己的关键帧没有给出基座位姿时，保持放置后的初值（不被重复变换）；其他实例的关键帧不影响它。

#include <cmath>
#include <cstdio>
#include <string>

#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/physics/physics_world.hpp"
#include "terra/physics/world_composition.hpp"

#ifndef TERRA_TEST_ASSETS_DIR
#error "需要 TERRA_TEST_ASSETS_DIR"
#endif

namespace {

namespace td = terra::data;
namespace tp = terra::physics;
using td::Quaternion;
using td::Vec3;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

std::string str(const Vec3& v) {
    char buf[96];
    std::snprintf(buf, sizeof buf, "(%.9g, %.9g, %.9g)", v.x, v.y, v.z);
    return buf;
}

bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) <= tol; }
bool near(const Vec3& a, const Vec3& b, double tol = 1e-9) {
    return near(a.x, b.x, tol) && near(a.y, b.y, tol) && near(a.z, b.z, tol);
}
bool sameRotation(const Quaternion& a, const Quaternion& b, double tol = 1e-9) {
    const double dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    return std::fabs(std::fabs(dot) - 1.0) <= tol;
}
Quaternion yawDeg(double deg) {
    const double h = deg * 3.14159265358979323846 / 360.0;
    return {std::cos(h), 0.0, 0.0, std::sin(h)};
}

const std::string kFixture = std::string{TERRA_TEST_ASSETS_DIR} + "/robot_description.xml";

const td::JointState* jointState(const tp::PhysicsWorld& world, const char* name) {
    const auto* j = world.description().findJoint(name);
    check(j != nullptr, std::string{"找到关节 "} + name);
    return j ? &world.snapshot()->joints[j->id.value - 1] : nullptr;
}

const td::EntityState* entityState(const tp::PhysicsWorld& world, const char* name) {
    const auto* e = world.description().findEntity(name);
    check(e != nullptr, std::string{"找到实体 "} + name);
    return e ? &world.snapshot()->entities[e->id.value - 1] : nullptr;
}

void testJoints(const td::ModelDescription& d) {
    const auto* root = d.findJoint("root");
    const auto* shoulder = d.findJoint("shoulder");
    const auto* extend = d.findJoint("extend");
    const auto* spin = d.findJoint("spin");
    check(root && shoulder && extend && spin, "四个关节都在");
    if (!(root && shoulder && extend && spin)) return;
    check(root->type == td::JointType::Free && !root->limited, "自由关节没有限位");
    check(near(root->axis, {0, 0, 0}), "自由关节的轴为 0");

    check(shoulder->limited, "肩关节有限位");
    check(near(shoulder->rangeMin, -1.0) && near(shoulder->rangeMax, 1.0),
          "肩关节限位 [-1, 1] rad（实际 [" + std::to_string(shoulder->rangeMin) + ", " + std::to_string(shoulder->rangeMax) + "]）");
    check(near(shoulder->axis, {0, 1, 0}), "肩关节轴 (0, 1, 0)，实际 " + str(shoulder->axis));
    check(near(shoulder->anchor, {0.05, 0, 0}), "肩关节位置 (0.05, 0, 0)，实际 " + str(shoulder->anchor));
    check(near(shoulder->damping, 0.3) && near(shoulder->armature, 0.01) && near(shoulder->frictionLoss, 0.05),
          "肩关节阻尼 0.3、电枢惯量 0.01、摩擦损失 0.05");

    check(!extend->limited, "滑动关节没有限位");
    check(near(extend->axis, {1, 0, 0}) && near(extend->damping, 2.0), "滑动关节轴 (1, 0, 0)、阻尼 2");
    check(!spin->limited && near(spin->damping, 0.0), "轮子没有限位、没有阻尼");
}

void testActuators(const td::ModelDescription& d) {
    const auto* motor = d.findActuator("extend_motor");
    const auto* position = d.findActuator("shoulder_pos");
    const auto* velocity = d.findActuator("spin_vel");
    const auto* filtered = d.findActuator("filtered");
    check(motor && position && velocity && filtered, "四个执行器都在");
    if (!(motor && position && velocity && filtered)) return;
    check(motor->kind == td::ActuatorKind::Motor && near(motor->gain, 1.0), "extend_motor 为力矩执行器，增益 1");
    check(near(motor->gear, 2.0), "extend_motor 传动比 2，实际 " + std::to_string(motor->gear));
    check(motor->forceLimited && near(motor->forceMin, -5.0) && near(motor->forceMax, 5.0), "extend_motor 力限制 [-5, 5]");
    check(position->kind == td::ActuatorKind::Position, "shoulder_pos 为位置执行器");
    check(near(position->gain, 50.0) && near(position->damping, 5.0),
          "shoulder_pos kp 50、kv 5，实际 " + std::to_string(position->gain) + "、" + std::to_string(position->damping));
    check(!position->forceLimited, "shoulder_pos 没有力限制");
    check(velocity->kind == td::ActuatorKind::Velocity && near(velocity->gain, 3.0), "spin_vel 为速度执行器，kv 3");
    check(filtered->kind == td::ActuatorKind::Other, "带滤波动力学的执行器归为 Other");
}

void testKeyframes(tp::PhysicsWorld& world) {
    const td::ModelDescription& d = world.description();
    check(d.keyframes.size() == 2, "两个关键帧，实际 " + std::to_string(d.keyframes.size()));
    if (d.keyframes.size() != 2) return;
    check(d.keyframes[0].name == "rest" && d.keyframes[1].name == "raised", "关键帧名字与顺序");
    check(near(d.keyframes[1].time, 1.5), "关键帧 raised 的时间 1.5 s（只作记录）");
    check(d.findKeyframe("raised") == 1 && d.findKeyframe("missing") == -1, "按名字查找关键帧");

    for (int k = 0; k < 10; ++k) check(!world.step(), "推进");
    const std::uint64_t epoch = world.runEpoch();
    check(world.resetToKeyframe(1), "按关键帧 raised 重置");
    check(world.stepIndex() == 0 && world.simulationTime() == 0.0, "按关键帧重置后第 0 步、仿真时间 0（不取关键帧的 1.5 s）");
    check(world.runEpoch() == epoch + 1, "按关键帧重置开始新的运行段");
    if (const auto* base = entityState(world, "base")) {
        check(near(base->pose.position, {0.2, 0, 0.5}), "基座位置 (0.2, 0, 0.5)，实际 " + str(base->pose.position));
        check(sameRotation(base->pose.orientation, yawDeg(90)), "基座偏航 90°");
    }
    if (const auto* root = jointState(world, "root")) {
        check(near(Vec3{root->velocity[0], root->velocity[1], root->velocity[2]}, {1, 0, 0}), "基座线速度 (1, 0, 0)");
    }
    if (const auto* shoulder = jointState(world, "shoulder")) check(near(shoulder->position[0], 0.5), "肩关节 0.5 rad");
    if (const auto* extend = jointState(world, "extend")) check(near(extend->position[0], 0.05), "滑动关节 0.05 m");

    // 控制量来自关键帧：位置执行器的目标为 0.8 rad，肩关节应朝 0.8 运动（无重力，其他力只有阻尼与摩擦损失）
    for (int k = 0; k < 250; ++k) check(!world.step(), "按关键帧重置后推进");
    if (const auto* shoulder = jointState(world, "shoulder")) {
        check(shoulder->position[0] > 0.6, "关键帧的控制量生效：0.5 s 后肩关节转向目标 0.8 rad（实际 " +
                                               std::to_string(shoulder->position[0]) + "）");
    }

    const std::uint64_t before = world.stepIndex();
    check(!world.resetToKeyframe(2), "越界的关键帧下标返回 false");
    check(world.stepIndex() == before && world.runEpoch() == epoch + 1, "越界下标不改变状态");

    world.reset();
    if (const auto* base = entityState(world, "base")) {
        check(near(base->pose.position, {0, 0, 0.4}), "普通重置回到模型初值 (0, 0, 0.4)，实际 " + str(base->pose.position));
    }
    check(world.resetToKeyframe(0), "按关键帧 rest（没有给出任何值）重置");
    if (const auto* base = entityState(world, "base")) {
        check(near(base->pose.position, {0, 0, 0.4}), "空关键帧等于模型初值");
    }
}

void testComposed() {
    tp::WorldComposition composition;
    composition.sourceLabel = "robot_description（两个实例）";
    td::Pose placed;
    placed.position = {1, 2, 0};
    placed.orientation = yawDeg(90);
    composition.models = {{"a", "a/", kFixture, td::Pose{}, false}, {"b", "b/", kFixture, placed, false}};
    auto composed = terra::physics::mujoco::composeWorld(composition);
    for (const auto& issue : composed.issues) std::printf("INFO 组合：%s\n", issue.message.c_str());
    check(composed.world != nullptr, "组合两个实例");
    if (!composed.world) return;
    tp::PhysicsWorld& world = *composed.world;
    const td::ModelDescription& d = world.description();
    std::string names;
    for (const auto& k : d.keyframes) names += (names.empty() ? "" : "、") + k.name;
    std::printf("INFO 组合后的关键帧：%s\n", names.c_str());
    check(d.keyframes.size() == 4, "组合后 4 个关键帧，实际 " + std::to_string(d.keyframes.size()));
    check(d.findKeyframe("a/raised") >= 0 && d.findKeyframe("b/raised") >= 0 && d.findKeyframe("raised") < 0,
          "关键帧名带实例前缀");
    if (const auto* b = entityState(world, "b/base")) {
        check(near(b->pose.position, {1, 2, 0.4}), "实例 b 放置后的初值 (1, 2, 0.4)，实际 " + str(b->pose.position));
    }

    const int bRaised = d.findKeyframe("b/raised");
    if (bRaised >= 0 && world.resetToKeyframe(static_cast<std::size_t>(bRaised))) {
        if (const auto* b = entityState(world, "b/base")) {
            check(near(b->pose.position, {1, 2.2, 0.5}),
                  "实例 b 按自己的关键帧：基座位于放置框架中的 (0.2, 0, 0.5)，即世界 (1, 2.2, 0.5)，实际 " + str(b->pose.position));
            check(sameRotation(b->pose.orientation, yawDeg(180)), "实例 b 基座偏航 = 放置 90° + 关键帧 90°");
        }
        if (const auto* root = jointState(world, "b/root")) {
            check(near(Vec3{root->velocity[0], root->velocity[1], root->velocity[2]}, {0, 1, 0}),
                  "实例 b 基座线速度随放置旋转为 (0, 1, 0)");
        }
        if (const auto* a = entityState(world, "a/base")) {
            check(near(a->pose.position, {0, 0, 0.4}), "实例 a 不受 b 的关键帧影响，实际 " + str(a->pose.position));
        }
    } else {
        check(false, "按关键帧 b/raised 重置");
    }

    // 实例自己的关键帧没有给出基座位姿（rest 为空）：MuJoCo 以已含放置位姿的初值补齐，不能再乘一次放置位姿。
    // 参考机器人中 Stretch 的 home / stow 就是这种情况（只给控制量）。
    const int bRest = d.findKeyframe("b/rest");
    if (bRest >= 0 && world.resetToKeyframe(static_cast<std::size_t>(bRest))) {
        if (const auto* b = entityState(world, "b/base")) {
            check(near(b->pose.position, {1, 2, 0.4}),
                  "实例 b 的空关键帧 rest：基座保持放置后的初值 (1, 2, 0.4)，没有被重复变换，实际 " + str(b->pose.position));
            check(sameRotation(b->pose.orientation, yawDeg(90)), "实例 b 的空关键帧 rest：保持放置偏航 90°");
        }
    } else {
        check(false, "按关键帧 b/rest 重置");
    }

    const int aRaised = d.findKeyframe("a/raised");
    if (aRaised >= 0 && world.resetToKeyframe(static_cast<std::size_t>(aRaised))) {
        if (const auto* a = entityState(world, "a/base")) {
            check(near(a->pose.position, {0.2, 0, 0.5}), "实例 a（单位放置）按自己的关键帧，实际 " + str(a->pose.position));
        }
        if (const auto* b = entityState(world, "b/base")) {
            check(near(b->pose.position, {1, 2, 0.4}),
                  "其他实例的关键帧不改变实例 b（保持放置后的初值），实际 " + str(b->pose.position));
            check(sameRotation(b->pose.orientation, yawDeg(90)), "实例 b 保持放置偏航 90°");
        }
    } else {
        check(false, "按关键帧 a/raised 重置");
    }
}

}  // namespace

int main() {
    auto loaded = terra::physics::mujoco::loadWorld(td::ModelSource{kFixture});
    check(loaded.world != nullptr, "加载 robot_description.xml");
    if (loaded.world) {
        testJoints(loaded.world->description());
        testActuators(loaded.world->description());
        testKeyframes(*loaded.world);
    }
    testComposed();
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
