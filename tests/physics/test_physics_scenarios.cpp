// M0-08 物理场景验证：主场景正式判定（开发计划 4.2 节）与辅助用例（4.3 节）。
// 只通过 Terra 公共头访问后端，本文件不包含 MuJoCo 头文件。门槛全部来自 m0_tolerances.hpp。
//
// 输出约定：
//   - 每项判定打印一行 “METRIC <夹具> <判定> 实测 <值> 门槛 <值> <PASS|FAIL>”，供验证报告引用；
//   - 失败时另打印 “FAILED: …”，包含夹具、步数与状态字段，便于定位。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "m0_tolerances.hpp"
#include "terra/physics/mujoco/backend_info.hpp"
#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/physics/physics_world.hpp"

#ifndef TERRA_TEST_ASSETS_DIR
#error "需要 TERRA_TEST_ASSETS_DIR"
#endif

namespace {

namespace m0 = terra::test::m0;
using terra::data::ActuatorId;
using terra::data::EntityId;
using terra::data::ModelSource;
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

// 记录一项“实测 ≤ 门槛”类判定。
void metric(const char* fixture, const char* name, double measured, double limit, const std::string& context = {}) {
    const bool ok = measured <= limit;
    std::printf("METRIC %s %s 实测 %.6g 门槛 %.6g %s\n", fixture, name, measured, limit, ok ? "PASS" : "FAIL");
    check(ok, std::string{fixture} + "：" + name + " 实测 " + std::to_string(measured) + " 超过门槛 " +
                  std::to_string(limit) + (context.empty() ? "" : "（" + context + "）"));
}

// 记录一项“实测落在区间内”类判定。
void metricRange(const char* fixture, const char* name, double measured, double lo, double hi) {
    const bool ok = measured >= lo && measured <= hi;
    std::printf("METRIC %s %s 实测 %.6g 门槛 [%.6g, %.6g] %s\n", fixture, name, measured, lo, hi, ok ? "PASS" : "FAIL");
    check(ok, std::string{fixture} + "：" + name + " 实测 " + std::to_string(measured) + " 不在 [" +
                  std::to_string(lo) + ", " + std::to_string(hi) + "] 内");
}

std::unique_ptr<PhysicsWorld> loadAsset(const std::string& file) {
    auto r = terra::physics::mujoco::loadWorld(ModelSource{std::string{TERRA_TEST_ASSETS_DIR} + "/" + file});
    check(r.world != nullptr, "加载 " + file + " 失败：" + (r.error ? r.error->message : ""));
    return std::move(r.world);
}

double norm(const Vec3& v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

double quatNorm(const Quaternion& q) { return std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z); }

// 两个单位四元数之间的旋转角（rad），按 q 与 -q 等价处理。
double rotationAngle(const Quaternion& a, const Quaternion& b) {
    const double dot = std::fabs(a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z);
    return 2.0 * std::acos(std::min(1.0, dot));
}

std::string at(std::uint64_t step, const char* field) {
    return "第 " + std::to_string(step) + " 步，字段 " + field;
}

// ---- 主场景：falling_cube.xml，5000 步，每步判定 ----
void testMainScenario() {
    const char* fx = "falling_cube";
    auto world = loadAsset("falling_cube.xml");
    if (!world) return;
    const double dt = world->description().timestep;

    double maxTimeError = 0.0;
    bool monotonic = true;
    double maxFreeFallError = 0.0;
    std::uint64_t freeFallWorstStep = 0;
    double maxDrift = 0.0;
    bool descendingBeforeContact = true;
    std::uint64_t firstContactStep = 0;
    double firstContactTime = -1.0;
    double maxRestHeight = 0.0, maxRestLinear = 0.0, maxRestAngular = 0.0, maxRestForceRel = 0.0;
    bool restContactEveryStep = true;
    std::uint64_t restContactMissing = 0;
    double maxQuatError = 0.0;
    bool allFinite = true;
    std::uint64_t firstNonFinite = 0;

    const std::uint64_t restBegin = static_cast<std::uint64_t>(m0::kMainSteps - std::lround(m0::kRestWindow / dt));
    std::shared_ptr<const PhysicsSnapshot> held;  // 观测一致：持有第 100 步快照
    double heldZ = 0.0;
    double previousTime = 0.0;
    double previousZ = 1.0;

    for (std::uint64_t k = 1; k <= static_cast<std::uint64_t>(m0::kMainSteps); ++k) {
        if (const auto error = world->step()) {
            check(false, std::string{fx} + "：第 " + std::to_string(error->step) + " 步仿真错误：" + error->message);
            return;
        }
        const auto s = world->snapshot();
        const auto& e = s->entities[0];
        const double t = s->simulationTime;

        maxTimeError = std::max(maxTimeError, std::fabs(t - static_cast<double>(k) * dt));
        monotonic = monotonic && t > previousTime;
        previousTime = t;

        if (!s->diagnostics.finite && allFinite) {
            allFinite = false;
            firstNonFinite = k;
        }
        maxQuatError = std::max(maxQuatError, std::fabs(quatNorm(e.pose.orientation) - 1.0));
        maxDrift = std::max({maxDrift, std::fabs(e.pose.position.x), std::fabs(e.pose.position.y)});

        const bool contact = s->contacts.activeCount > 0;
        if (contact && firstContactTime < 0) {
            firstContactTime = t;
            firstContactStep = k;
        }
        if (firstContactTime < 0) {
            // 碰撞前：每一步都在向下运动
            descendingBeforeContact = descendingBeforeContact && e.linearVelocity.z < 0 && e.pose.position.z < previousZ;
        }
        previousZ = e.pose.position.z;

        if (t >= m0::kFreeFallWindowBegin - 1e-12 && t <= m0::kFreeFallWindowEnd + 1e-12) {
            const double err = std::fabs(e.pose.position.z - (1.0 - 0.5 * kGravity * t * t));
            if (err > maxFreeFallError) {
                maxFreeFallError = err;
                freeFallWorstStep = k;
            }
        }

        if (k > restBegin) {
            maxRestHeight = std::max(maxRestHeight, std::fabs(e.pose.position.z - m0::kRestHeight));
            maxRestLinear = std::max(maxRestLinear, norm(e.linearVelocity));
            maxRestAngular = std::max(maxRestAngular, norm(e.angularVelocity));
            const auto& pairs = s->contacts.pairs;
            const bool ground = pairs.size() == 1 && !pairs[0].entityA.valid() && pairs[0].entityB == EntityId{1};
            if (ground) {
                maxRestForceRel = std::max(maxRestForceRel, std::fabs(pairs[0].normalForce - kGravity) / kGravity);
            } else if (restContactEveryStep) {
                restContactEveryStep = false;
                restContactMissing = k;
            }
        }

        if (k == 100) {
            held = s;
            heldZ = s->entities[0].pose.position.z;
        }
    }
    const auto last = world->snapshot();

    metric(fx, "时间误差(s)", maxTimeError, m0::kTimeError);
    check(monotonic, std::string{fx} + "：时间严格单调");
    metric(fx, "自由落体误差(m)", maxFreeFallError, m0::kFreeFallError, at(freeFallWorstStep, "entities[0].position.z"));
    metric(fx, "平面漂移(m)", maxDrift, m0::kPlanarDrift, "entities[0].position.x/y");
    check(descendingBeforeContact, std::string{fx} + "：碰撞前每一步都向下运动（linear_velocity.z < 0，z 递减）");
    metricRange(fx, "首次有效接触(s)", firstContactTime, m0::kFirstContactBegin, m0::kFirstContactEnd);
    std::printf("INFO %s 首次有效接触在第 %llu 步\n", fx, static_cast<unsigned long long>(firstContactStep));
    metric(fx, "稳定窗口高度误差(m)", maxRestHeight, m0::kRestHeightError, "entities[0].position.z");
    metric(fx, "稳定窗口线速度(m/s)", maxRestLinear, m0::kRestLinearSpeed, "entities[0].linear_velocity");
    metric(fx, "稳定窗口角速度(rad/s)", maxRestAngular, m0::kRestAngularSpeed, "entities[0].angular_velocity");
    metric(fx, "静止法向力相对偏差", maxRestForceRel, m0::kRestNormalForceRelative, "contacts.pairs[0].normal_force");
    check(restContactEveryStep, std::string{fx} + "：稳定窗口每步都有地面有效接触，" + at(restContactMissing, "contacts.pairs") + " 缺失");
    metric(fx, "四元数范数误差", maxQuatError, m0::kQuaternionNormError);
    metric(fx, "最终姿态旋转角(rad)", rotationAngle(last->entities[0].pose.orientation, Quaternion{}),
           m0::kFinalOrientationAngle);
    check(allFinite, std::string{fx} + "：全程有限，" + at(firstNonFinite, "diagnostics.finite") + " 为 false");
    check(held && held->stepIndex == 100 && held->entities[0].pose.position.z == heldZ,
          std::string{fx} + "：持有的第 100 步快照未被后续 step 修改");
}

// ---- 辅助：无重力、无初速度（floating_cube.xml）----
void testFloating() {
    const char* fx = "floating_cube";
    auto world = loadAsset("floating_cube.xml");
    if (!world) return;
    const auto initial = world->snapshot();
    const auto& p0 = initial->entities[0].pose;
    // 夹具中写明的初始位姿：坐标轴与四元数顺序若有错误，这里就对不上
    const Quaternion expectedQ{0.96592582628906831, 0, 0.25881904510252074, 0};
    check(std::fabs(p0.position.x - 0.3) <= 1e-15 && std::fabs(p0.position.y + 0.2) <= 1e-15 &&
              std::fabs(p0.position.z - 1.5) <= 1e-15,
          std::string{fx} + "：初始位置 = (0.3, -0.2, 1.5)");
    check(rotationAngle(p0.orientation, expectedQ) <= 1e-12 && p0.orientation.y > 0,
          std::string{fx} + "：初始姿态 = 绕 +Y 30°（wxyz）");
    const auto& j = initial->joints[0];
    check(j.position[0] == p0.position.x && j.position[3] == p0.orientation.w && j.position[5] == p0.orientation.y,
          std::string{fx} + "：自由关节配置与实体位姿一致");

    double maxPose = 0.0, maxSpeed = 0.0;
    for (int k = 1; k <= m0::kFloatingSteps; ++k) {
        if (const auto error = world->step()) {
            check(false, std::string{fx} + "：第 " + std::to_string(error->step) + " 步仿真错误：" + error->message);
            return;
        }
        const auto s = world->snapshot();
        const auto& e = s->entities[0];
        maxPose = std::max({maxPose, std::fabs(e.pose.position.x - p0.position.x),
                            std::fabs(e.pose.position.y - p0.position.y), std::fabs(e.pose.position.z - p0.position.z),
                            rotationAngle(e.pose.orientation, p0.orientation)});
        maxSpeed = std::max({maxSpeed, norm(e.linearVelocity), norm(e.angularVelocity)});
        if (s->contacts.candidateCount != 0) {
            check(false, std::string{fx} + "：不应有接触，" + at(static_cast<std::uint64_t>(k), "contacts.candidates"));
        }
    }
    metric(fx, "位姿变化", maxPose, m0::kFloatingPoseError, "entities[0].position/orientation");
    metric(fx, "速度", maxSpeed, m0::kFloatingSpeed, "entities[0].linear/angular_velocity");
}

// ---- 辅助：reset / 再运行 20 个周期 ----
struct Trajectory {
    std::vector<double> values;
};

double maxDifference(const Trajectory& a, const Trajectory& b) {
    if (a.values.size() != b.values.size()) return std::numeric_limits<double>::infinity();
    double d = 0.0;
    for (std::size_t i = 0; i < a.values.size(); ++i) d = std::max(d, std::fabs(a.values[i] - b.values[i]));
    return d;
}

void checkResetState(const PhysicsWorld& world, const PhysicsSnapshot& reference, std::uint64_t cycle,
                     const char* fx) {
    const auto s = world.snapshot();
    const std::string tag = std::string{fx} + " 第 " + std::to_string(cycle) + " 周期：";
    check(s->runEpoch == cycle && world.runEpoch() == cycle, tag + "运行代次 = 周期序号");
    check(s->stepIndex == 0 && s->simulationTime == 0.0, tag + "第 0 步、t = 0");
    check(s->worldInstance == reference.worldInstance, tag + "世界实例号不变");
    bool same = s->entities.size() == reference.entities.size() && s->joints.size() == reference.joints.size();
    for (std::size_t i = 0; same && i < s->joints.size(); ++i) {
        for (int k = 0; k < s->joints[i].positionSize; ++k) {
            same = same && s->joints[i].position[static_cast<std::size_t>(k)] ==
                               reference.joints[i].position[static_cast<std::size_t>(k)];
        }
        for (int k = 0; k < s->joints[i].velocitySize; ++k) {
            same = same && s->joints[i].velocity[static_cast<std::size_t>(k)] == 0.0;
        }
    }
    check(same, tag + "初始状态与首次加载相同");
}

void testResetCycles() {
    // 立方体：每周期 1000 步，跨过触地（约第 428 步），比较位置、姿态和速度。
    {
        const char* fx = "falling_cube";
        auto world = loadAsset("falling_cube.xml");
        if (!world) return;
        const auto reference = world->snapshot();
        Trajectory first;
        double worst = 0.0;
        for (std::uint64_t cycle = 0; cycle < static_cast<std::uint64_t>(m0::kResetCycles); ++cycle) {
            if (cycle > 0) world->reset();
            checkResetState(*world, *reference, cycle, fx);
            Trajectory t;
            for (int k = 1; k <= 1000; ++k) {
                if (world->step()) {
                    check(false, std::string{fx} + "：reset 周期 " + std::to_string(cycle) + " 第 " + std::to_string(k) + " 步出错");
                    return;
                }
                const auto snap = world->snapshot();
                const auto& e = snap->entities[0];
                t.values.insert(t.values.end(), {e.pose.position.x, e.pose.position.y, e.pose.position.z,
                                                 e.pose.orientation.w, e.pose.orientation.x, e.pose.orientation.y,
                                                 e.pose.orientation.z, e.linearVelocity.z});
            }
            if (cycle == 0) first = t;
            else worst = std::max(worst, maxDifference(first, t));
        }
        metric(fx, "reset 再运行轨迹差", worst, m0::kRerunTrajectoryError, "20 周期 × 1000 步");
    }

    // 铰链电机：每周期先 100 步不设控制（验证 reset 已恢复中性控制，关节应静止），
    // 再执行固定控制序列 600 步，比较关节轨迹。
    {
        const char* fx = "hinge_motor";
        auto world = loadAsset("hinge_motor.xml");
        if (!world) return;
        const auto reference = world->snapshot();
        const ActuatorId drive{1}, aim{2}, trim{3};
        const double h = std::sqrt(0.5);
        Trajectory first;
        double worst = 0.0;
        for (std::uint64_t cycle = 0; cycle < static_cast<std::uint64_t>(m0::kResetCycles); ++cycle) {
            if (cycle > 0) world->reset();
            checkResetState(*world, *reference, cycle, fx);
            Trajectory t;
            for (int k = 1; k <= 700; ++k) {
                if (k == 101) check(!world->setControl({{drive, {0.3}}, {aim, {h, 0, 0, h}}}).has_value(), "设置控制 1");
                if (k == 401) check(!world->setControl({{drive, {-0.2}}, {trim, {0.1}}}).has_value(), "设置控制 2");
                if (world->step()) {
                    check(false, std::string{fx} + "：reset 周期 " + std::to_string(cycle) + " 第 " + std::to_string(k) + " 步出错");
                    return;
                }
                const auto s = world->snapshot();
                if (k <= 100) {
                    const auto& w = s->joints[1].velocity;
                    check(s->joints[0].velocity[0] == 0 && w[0] == 0 && w[1] == 0 && w[2] == 0,
                          std::string{fx} + " 第 " + std::to_string(cycle) + " 周期：reset 后未设控制时关节静止（" +
                              at(static_cast<std::uint64_t>(k), "joints[*].velocity") + "）");
                }
                t.values.push_back(s->joints[0].position[0]);
                t.values.push_back(s->joints[0].velocity[0]);
                for (std::size_t i = 0; i < 4; ++i) t.values.push_back(s->joints[1].position[i]);
            }
            if (cycle == 0) first = t;
            else worst = std::max(worst, maxDifference(first, t));
        }
        metric(fx, "reset 再运行轨迹差", worst, m0::kRerunTrajectoryError, "20 周期 × 700 步，含控制序列");
    }
}

// ---- 辅助：生命周期与快照 ----
void testLifecycle() {
    const char* fx = "lifecycle";
    // 重复加载与释放；实例号严格递增、互不相同
    std::uint64_t previous = 0;
    bool increasing = true;
    for (int i = 0; i < 100; ++i) {
        auto world = loadAsset(i % 2 ? "falling_cube.xml" : "hinge_motor.xml");
        if (!world) return;
        const std::uint64_t instance = world->description().worldInstance;
        increasing = increasing && instance > previous && world->snapshot()->worldInstance == instance;
        previous = instance;
        if (!world->step().has_value()) (void)world->snapshot();
    }
    check(increasing, std::string{fx} + "：100 次加载/释放，世界实例号严格递增且快照与描述一致");

    // 重新加载后旧 ID 的数值相同，但世界实例号不同：消费者据此判定旧 ID/快照不属于新世界
    std::shared_ptr<const PhysicsSnapshot> oldSnapshot;
    std::uint64_t oldInstance = 0;
    {
        auto a = loadAsset("falling_cube.xml");
        if (!a) return;
        for (int k = 0; k < 200; ++k) (void)a->step();
        oldSnapshot = a->snapshot();
        oldInstance = a->description().worldInstance;
    }  // 世界 a 已释放
    const auto failed = terra::physics::mujoco::loadWorld(ModelSource{std::string{TERRA_TEST_ASSETS_DIR} + "/no_such_model.xml"});
    check(!failed.world, std::string{fx} + "：加载不存在的模型失败");
    auto b = loadAsset("falling_cube.xml");
    if (!b) return;
    check(b->description().entities[0].id == oldSnapshot->entities[0].id,
          std::string{fx} + "：重新加载后实体 ID 数值相同（ID 本身不携带世界信息）");
    check(b->description().worldInstance != oldSnapshot->worldInstance,
          std::string{fx} + "：重新加载后世界实例号不同，旧快照可被识别为旧世界");
    check(b->description().worldInstance == oldInstance + 1, std::string{fx} + "：失败的加载不消耗实例号");
    // 世界释放后，持有的旧快照仍可安全读取
    check(oldSnapshot->stepIndex == 200 && std::isfinite(oldSnapshot->entities[0].pose.position.z) &&
              oldSnapshot->entities[0].pose.position.z < 1.0,
          std::string{fx} + "：世界释放后旧快照仍可读取");
}

}  // namespace

int main() {
    const auto backend = terra::physics::mujoco::backendInfo();
    std::printf("INFO 后端 %.*s %.*s\n", static_cast<int>(backend.name.size()), backend.name.data(),
                static_cast<int>(backend.libraryVersionString.size()), backend.libraryVersionString.data());
    testMainScenario();
    testFloating();
    testResetCycles();
    testLifecycle();
    if (failures == 0) {
        std::printf("all passed\n");
    }
    return failures == 0 ? 0 : 1;
}
