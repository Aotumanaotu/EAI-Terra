// 状态读取与不可变快照测试（M0-06）。
// 只通过 Terra 公共头访问后端，本文件不包含 MuJoCo 头文件。
//
// 立方体主基准的判定门槛来自开发计划 4.2 节的候选容差（尚未冻结，M0-08 正式评审）。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

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
using terra::data::EntityId;
using terra::data::JointType;
using terra::data::PhysicsSnapshot;
using terra::data::Quaternion;
using terra::data::Vec3;
using terra::physics::PhysicsWorld;

constexpr double kGravity = 9.81;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

std::string num(double v) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.6g", v);
    return buf;
}

double norm(const Vec3& v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

double quatNorm(const Quaternion& q) {
    return std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
}

// 用单位四元数旋转向量：v' = q v q*
Vec3 rotate(const Quaternion& q, const Vec3& v) {
    const double tx = 2 * (q.y * v.z - q.z * v.y);
    const double ty = 2 * (q.z * v.x - q.x * v.z);
    const double tz = 2 * (q.x * v.y - q.y * v.x);
    return {v.x + q.w * tx + (q.y * tz - q.z * ty), v.y + q.w * ty + (q.z * tx - q.x * tz),
            v.z + q.w * tz + (q.x * ty - q.y * tx)};
}

std::unique_ptr<PhysicsWorld> loadPath(const fs::path& path) {
    auto r = terra::physics::mujoco::loadWorld(terra::data::ModelSource{path.string()});
    if (!r.world) {
        check(false, "加载 " + path.string() + " 失败：" + (r.error ? r.error->message : ""));
    }
    return std::move(r.world);
}

std::unique_ptr<PhysicsWorld> loadAsset(const std::string& file) {
    return loadPath(fs::path{TERRA_TEST_ASSETS_DIR} / file);
}

std::unique_ptr<PhysicsWorld> loadInline(const std::string& name, const std::string& xml) {
    static const fs::path dir = [] {
        fs::path d{TERRA_TEST_WORK_DIR};
        fs::remove_all(d);
        fs::create_directories(d);
        return d;
    }();
    const fs::path path = dir / name;
    std::ofstream{path} << xml;
    return loadPath(path);
}

bool step(PhysicsWorld& world) {
    if (const auto error = world.step()) {
        check(false, "第 " + std::to_string(error->step) + " 步出错：" + error->message);
        return false;
    }
    return true;
}

void testInitialSnapshot() {
    auto world = loadAsset("falling_cube.xml");
    if (!world) return;
    const auto s = world->snapshot();
    check(s->runEpoch == 0 && s->stepIndex == 0 && s->simulationTime == 0.0, "初始快照：代次 0、第 0 步、t=0");
    check(s->fixedTimestep == 0.001, "快照记录步长");
    check(s->entities.size() == 1 && s->joints.size() == 1, "1 个实体、1 个关节");
    if (s->entities.size() != 1 || s->joints.size() != 1) return;

    const auto& e = s->entities[0];
    check(e.pose.position.z == 1.0 && e.pose.orientation.w == 1.0, "初始位姿");
    check(norm(e.linearVelocity) == 0.0 && norm(e.angularVelocity) == 0.0, "初始速度为零");

    const auto& j = s->joints[0];
    check(j.type == JointType::Free && j.positionSize == 7 && j.velocitySize == 6,
          "自由关节：配置 7 维、速度 6 维，两者不同");
    check(j.position[0] == 0 && j.position[1] == 0 && j.position[2] == 1 && j.position[3] == 1,
          "自由关节配置 = 位置 + 姿态四元数");

    check(s->contacts.activeCount == 0 && s->contacts.pairs.empty(), "初始时没有有效接触");
    check(s->diagnostics.finite, "初始快照全部有限");
}

void testSnapshotImmutability() {
    auto world = loadAsset("falling_cube.xml");
    if (!world) return;
    const auto s0 = world->snapshot();
    check(world->snapshot() == s0, "同一观测时刻返回同一快照对象");

    if (!step(*world)) return;
    const auto s1 = world->snapshot();
    check(s1 != s0, "step 后返回新快照");
    check(s1->stepIndex == 1, "新快照为第 1 步");

    for (int i = 0; i < 499; ++i) {
        if (!step(*world)) return;
    }
    world->reset();
    for (int i = 0; i < 10; ++i) {
        if (!step(*world)) return;
    }
    // 旧快照在之后的 step/reset 中保持不变
    check(s0->stepIndex == 0 && s0->runEpoch == 0 && s0->simulationTime == 0.0 &&
              s0->entities[0].pose.position.z == 1.0 && s0->entities[0].linearVelocity.z == 0.0,
          "旧快照（第 0 步）未被后续 step/reset 修改");
    check(s1->stepIndex == 1 && s1->entities[0].pose.position.z == 1.0 - kGravity * 1e-6,
          "旧快照（第 1 步）未被后续 step/reset 修改");
    const auto now = world->snapshot();
    check(now->runEpoch == 1 && now->stepIndex == 10, "reset 后的快照带新代次");
}

void testFreeFallVelocity() {
    auto world = loadAsset("falling_cube.xml");
    if (!world) return;
    const double dt = world->description().timestep;
    auto previous = world->snapshot();
    double maxVelocityError = 0.0;
    double maxDifferenceError = 0.0;
    double maxJointMismatch = 0.0;
    for (std::uint64_t k = 1; k <= 300; ++k) {
        if (!step(*world)) return;
        const auto s = world->snapshot();
        const auto& e = s->entities[0];
        const auto& j = s->joints[0];
        // 半隐式 Euler：v_k = -g·k·dt；x_k = x_{k-1} + dt·v_k
        maxVelocityError = std::max(maxVelocityError, std::fabs(e.linearVelocity.z + kGravity * dt * static_cast<double>(k)));
        maxDifferenceError = std::max(
            maxDifferenceError,
            std::fabs((e.pose.position.z - previous->entities[0].pose.position.z) / dt - e.linearVelocity.z));
        check(norm(e.angularVelocity) == 0.0 && e.linearVelocity.x == 0.0 && e.linearVelocity.y == 0.0,
              "自由落体只有竖直速度");
        // 自由关节位于 body 原点：关节位置/线速度与实体一致
        maxJointMismatch = std::max({maxJointMismatch, std::fabs(j.position[2] - e.pose.position.z),
                                     std::fabs(j.velocity[2] - e.linearVelocity.z)});
        previous = s;
    }
    check(maxVelocityError <= 1e-12, "自由落体速度 v = -g·t，误差 " + num(maxVelocityError));
    check(maxDifferenceError <= 1e-9, "位置差分与速度一致，误差 " + num(maxDifferenceError));
    check(maxJointMismatch == 0.0, "自由关节状态与实体状态一致");
}

// 开发计划 4.2 节：立方体主基准中依赖速度与接触的判定（时间、自由落体、高度见 physics.mujoco_step）。
void testCubeBenchmark() {
    auto world = loadAsset("falling_cube.xml");
    if (!world) return;
    double firstContact = -1.0;
    bool contactBeforeWindow = false;
    double maxQuatError = 0.0;
    bool allFinite = true;
    double maxRestLinear = 0.0;
    double maxRestAngular = 0.0;
    double maxRestHeight = 0.0;
    double minRestForce = 1e9;
    double maxRestForce = 0.0;
    bool restContactEveryStep = true;
    std::uint32_t restPoints = 0;
    for (std::uint64_t k = 1; k <= 5000; ++k) {
        if (!step(*world)) return;
        const auto s = world->snapshot();
        const auto& e = s->entities[0];
        const double t = s->simulationTime;
        allFinite = allFinite && s->diagnostics.finite;
        maxQuatError = std::max(maxQuatError, std::fabs(quatNorm(e.pose.orientation) - 1.0));

        const bool active = s->contacts.activeCount > 0;
        if (active && firstContact < 0) {
            firstContact = t;
            contactBeforeWindow = t < 0.35;
        }
        if (k > 4500) {
            maxRestLinear = std::max(maxRestLinear, norm(e.linearVelocity));
            maxRestAngular = std::max(maxRestAngular, norm(e.angularVelocity));
            maxRestHeight = std::max(maxRestHeight, std::fabs(e.pose.position.z - 0.1));
            const bool groundPair = s->contacts.pairs.size() == 1 && !s->contacts.pairs[0].entityA.valid() &&
                                    s->contacts.pairs[0].entityB == EntityId{1};
            restContactEveryStep = restContactEveryStep && groundPair;
            if (groundPair) {
                minRestForce = std::min(minRestForce, s->contacts.pairs[0].normalForce);
                maxRestForce = std::max(maxRestForce, s->contacts.pairs[0].normalForce);
                restPoints = s->contacts.pairs[0].pointCount;
            }
        }
    }
    std::printf("立方体基准：首次有效接触 t=%.3f s；稳定窗口 |v|max=%.3g m/s，|ω|max=%.3g rad/s，"
                "高度误差 %.3g m；法向力 %.6g–%.6g N（%u 个接触点）；四元数范数误差 %.3g\n",
                firstContact, maxRestLinear, maxRestAngular, maxRestHeight, minRestForce, maxRestForce,
                restPoints, maxQuatError);

    check(firstContact >= 0.35 && firstContact <= 0.55 && !contactBeforeWindow,
          "首次有效接触在 0.35–0.55 s 内，实际 " + num(firstContact));
    check(maxRestLinear <= 0.01, "稳定窗口线速度 <= 0.01 m/s，实际 " + num(maxRestLinear));
    check(maxRestAngular <= 0.05, "稳定窗口角速度 <= 0.05 rad/s，实际 " + num(maxRestAngular));
    check(maxRestHeight <= 0.005, "稳定窗口高度误差 <= 0.005 m，实际 " + num(maxRestHeight));
    check(restContactEveryStep, "稳定窗口内每一步都存在立方体与地面的有效接触");
    // 静止时法向力之和应平衡重力 m·g = 9.81 N
    check(std::fabs(minRestForce - kGravity) <= 0.01 * kGravity && std::fabs(maxRestForce - kGravity) <= 0.01 * kGravity,
          "静止时法向力 ≈ m·g（1% 以内），实际 " + num(minRestForce) + "–" + num(maxRestForce));
    check(maxQuatError <= 1e-6, "四元数范数误差 <= 1e-6，实际 " + num(maxQuatError));
    check(allFinite, "全程所有字段有限");
}

void testVelocityReferencePoint() {
    // 摆：body 原点在铰链枢轴上，质心在 x = 0.25 m 处。枢轴固定，所以原点的线速度必须为 0；
    // 若错误地使用质心作为参考点，线速度不为 0。
    auto world = loadInline("pendulum.xml", R"(
<mujoco>
  <option timestep="0.001" gravity="0 0 -9.81"/>
  <worldbody>
    <body name="pendulum" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="capsule" fromto="0 0 0 0.5 0 0" size="0.02" mass="1" contype="0" conaffinity="0"/>
    </body>
  </worldbody>
</mujoco>)");
    if (!world) return;
    double maxOriginSpeed = 0.0;
    double maxAxisMismatch = 0.0;
    double maxJointSpeed = 0.0;
    for (int k = 0; k < 500; ++k) {
        if (!step(*world)) return;
        const auto s = world->snapshot();
        const auto& e = s->entities[0];
        const auto& j = s->joints[0];
        maxOriginSpeed = std::max(maxOriginSpeed, norm(e.linearVelocity));
        maxJointSpeed = std::max(maxJointSpeed, std::fabs(j.velocity[0]));
        // 转轴为世界 y：实体角速度 = (0, q̇, 0)
        maxAxisMismatch = std::max({maxAxisMismatch, std::fabs(e.angularVelocity.x),
                                    std::fabs(e.angularVelocity.y - j.velocity[0]), std::fabs(e.angularVelocity.z)});
    }
    check(maxJointSpeed > 1.0, "摆确实在转动，最大角速度 " + num(maxJointSpeed));
    check(maxOriginSpeed <= 1e-12, "实体原点（枢轴）的线速度为 0，实际 " + num(maxOriginSpeed));
    check(maxAxisMismatch <= 1e-12, "实体角速度 = 转轴 × 关节角速度，误差 " + num(maxAxisMismatch));
    const auto s = world->snapshot();
    check(s->joints[0].type == JointType::Hinge && s->joints[0].positionSize == 1 && s->joints[0].velocitySize == 1,
          "铰链关节维度 1/1");
}

void testFreeJointFrames() {
    // 倾斜的立方体落地后会翻转：检查自由关节角速度（body 局部坐标）旋转到世界坐标后，等于实体角速度。
    auto world = loadInline("tilted_cube.xml", R"(
<mujoco>
  <option timestep="0.001"/>
  <worldbody>
    <geom type="plane" size="0 0 1"/>
    <body name="cube" pos="0 0 0.5" quat="0.9659258262890683 0.25881904510252074 0 0">
      <freejoint/>
      <geom type="box" size="0.1 0.2 0.05" mass="1"/>
    </body>
  </worldbody>
</mujoco>)");
    if (!world) return;
    double maxSpin = 0.0;
    double maxAngularMismatch = 0.0;
    double maxLinearMismatch = 0.0;
    double maxOrientationMismatch = 0.0;
    for (int k = 0; k < 800; ++k) {
        if (!step(*world)) return;
        const auto s = world->snapshot();
        const auto& e = s->entities[0];
        const auto& j = s->joints[0];
        const Quaternion q{j.position[3], j.position[4], j.position[5], j.position[6]};
        const Vec3 wWorld = rotate(q, Vec3{j.velocity[3], j.velocity[4], j.velocity[5]});
        maxSpin = std::max(maxSpin, norm(e.angularVelocity));
        maxAngularMismatch = std::max(maxAngularMismatch, norm(Vec3{wWorld.x - e.angularVelocity.x,
                                                                    wWorld.y - e.angularVelocity.y,
                                                                    wWorld.z - e.angularVelocity.z}));
        maxLinearMismatch = std::max(maxLinearMismatch, norm(Vec3{j.velocity[0] - e.linearVelocity.x,
                                                                  j.velocity[1] - e.linearVelocity.y,
                                                                  j.velocity[2] - e.linearVelocity.z}));
        maxOrientationMismatch = std::max({maxOrientationMismatch, std::fabs(q.w - e.pose.orientation.w),
                                           std::fabs(q.x - e.pose.orientation.x)});
    }
    check(maxSpin > 0.5, "立方体确实发生了转动，最大角速度 " + num(maxSpin));
    check(maxAngularMismatch <= 1e-9,
          "自由关节角速度（body 局部）旋转后 = 实体角速度（世界），误差 " + num(maxAngularMismatch));
    check(maxLinearMismatch <= 1e-12, "自由关节线速度 = 实体原点线速度（世界），误差 " + num(maxLinearMismatch));
    check(maxOrientationMismatch <= 1e-15, "自由关节姿态 = 实体姿态");
}

void testContactPairs() {
    // 两个方块叠放在地面上：接触对应为 (world, lower) 与 (lower, upper)。
    auto world = loadInline("stack.xml", R"(
<mujoco>
  <option timestep="0.001"/>
  <worldbody>
    <geom type="plane" size="0 0 1"/>
    <body name="lower" pos="0 0 0.1">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="1"/>
    </body>
    <body name="upper" pos="0 0 0.35">
      <freejoint/>
      <geom type="box" size="0.05 0.05 0.05" mass="0.5"/>
    </body>
  </worldbody>
</mujoco>)");
    if (!world) return;
    for (int k = 0; k < 2000; ++k) {
        if (!step(*world)) return;
    }
    const auto s = world->snapshot();
    const auto* lower = world->description().findEntity("lower");
    const auto* upper = world->description().findEntity("upper");
    if (!lower || !upper) {
        check(false, "找到 lower/upper");
        return;
    }
    check(s->contacts.pairs.size() == 2, "两对有效接触，实际 " + std::to_string(s->contacts.pairs.size()));
    if (s->contacts.pairs.size() == 2) {
        const auto& ground = s->contacts.pairs[0];
        const auto& stacked = s->contacts.pairs[1];
        check(!ground.entityA.valid() && ground.entityB == lower->id, "第一对为 (world, lower)");
        check(stacked.entityA == lower->id && stacked.entityB == upper->id, "第二对为 (lower, upper)");
        check(std::fabs(ground.normalForce - 1.5 * kGravity) <= 0.015 * kGravity,
              "地面承受两个方块的重量 1.5·g，实际 " + num(ground.normalForce));
        check(std::fabs(stacked.normalForce - 0.5 * kGravity) <= 0.005 * kGravity,
              "下方方块承受上方方块的重量 0.5·g，实际 " + num(stacked.normalForce));
        std::uint32_t points = 0;
        for (const auto& p : s->contacts.pairs) points += p.pointCount;
        check(points == s->contacts.activeCount, "各接触对点数之和 = 有效接触点数");
    }
}

void testCandidateVersusActive() {
    // gap > 0：几何距离小于 margin 的接触会被检测到，但距离在 gap 内的不参与求解、不受力。
    // 这类候选接触不能被计为有效接触。
    auto world = loadInline("gap.xml", R"(
<mujoco>
  <option timestep="0.001"/>
  <default><geom margin="0.05" gap="0.05"/></default>
  <worldbody>
    <geom type="plane" size="0 0 1"/>
    <body name="cube" pos="0 0 0.3">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="1"/>
    </body>
  </worldbody>
</mujoco>)");
    if (!world) return;
    bool sawCandidateOnly = false;
    for (int k = 0; k < 400 && !sawCandidateOnly; ++k) {
        if (!step(*world)) return;
        const auto s = world->snapshot();
        sawCandidateOnly = s->contacts.candidateCount > 0 && s->contacts.activeCount == 0;
    }
    check(sawCandidateOnly, "出现过“有候选接触、无有效接触”的时刻");
}

}  // namespace

int main() {
    testInitialSnapshot();
    testSnapshotImmutability();
    testFreeFallVelocity();
    testCubeBenchmark();
    testVelocityReferencePoint();
    testFreeJointFrames();
    testContactPairs();
    testCandidateVersusActive();
    if (failures == 0) {
        std::printf("all passed\n");
    }
    return failures == 0 ? 0 : 1;
}
