// 关节与接触验证（M3-06）：每台参考机器人一组数值判据。判据的取值与依据见 M3-06 报告，冻结后修改须重新评审。
//
// 场景（全部经“动作 → PD 层 → setControl → step”推进，与查看器、Headless 相同）：
//   g1_stand   G1 地面，动作 stand_hold（11 s）
//   g1_wave    G1 悬挂，动作 wave；g1_squat  G1 悬挂，动作 squat
//   go2_hold   Go2 地面，不给动作，PD 层保持 home（3 s）
//   go2_motion Go2 地面，动作 lie_stand_sit
//   stretch    Stretch 3 地面，动作 tour
// 每个场景统计：
//   - 关节限位：所有有限位的转动 / 滑动关节，超出限位的最大量（MuJoCo 的限位是软约束，允许极小的穿透）；
//   - 基座：高度范围、最大倾斜角（基座 z 轴与世界 z 轴的夹角）、水平漂移；
//   - 与地面的接触：最后 1 s 中接触地面的实体；接触力合计 / 总重量（静止时应约为 1）；
//   - 状态有限、没有仿真错误。
// 另做模型核查：步长 0.002 s、重力 (0, 0, −9.81)、转动 / 滑动关节的轴为单位向量、限位区间 min < max、实体质量非负。
// 参考机器人未准备时以返回码 77 表示跳过。本文件不包含 MuJoCo 头文件。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <initializer_list>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/scene/scene.hpp"
#include "terra/simulation/motion.hpp"
#include "terra/simulation/pd_layer.hpp"

#ifndef TERRA_ASSETS_DIR
#error "需要 TERRA_ASSETS_DIR"
#endif

namespace {

namespace fs = std::filesystem;
namespace sc = terra::scene;
namespace sim = terra::simulation;
namespace td = terra::data;

constexpr int kSkipped = 77;
constexpr double kSettleSeconds = 0.5;  // 初始穿透释放期
constexpr double kPi = 3.14159265358979323846;
int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

struct Metrics {
    bool ok = true;
    double seconds = 0.0;
    double limitViolation = 0.0;  // rad 或 m
    std::string worstJoint;
    double settlingViolation = 0.0;  // 开始后 kSettleSeconds 内的最大超出（只记录）
    double worstTime = 0.0;       // 出现最大超出量的时刻（s）
    double worstValue = 0.0;      // 该时刻的关节位置
    std::string worstContacts;    // 该时刻机器人自身部件之间、以及与其他物体的接触（不含地面）
    double zMin = 1e9, zMax = -1e9;
    double tiltMaxDeg = 0.0;
    double drift = 0.0;            // 水平漂移（m）
    std::set<std::string> ground;  // 最后 1 s 中接触地面的实体
    double weightRatio = 0.0;      // 最后 1 s 的平均：地面接触力合计 / 总重量
    std::map<std::string, double> probes;  // 场景特有的观测值
    std::map<std::string, double> firstGround;  // 各实体第一次接触地面的时刻（s）
};

double tiltDeg(const td::Quaternion& q) {
    // 基座 z 轴在世界中的 z 分量 = 1 − 2(x² + y²)
    const double cz = std::clamp(1.0 - 2.0 * (q.x * q.x + q.y * q.y), -1.0, 1.0);
    return std::acos(cz) * 180.0 / kPi;
}

// probe(时间, 世界, 指标) 在每一步之后调用，用于场景特有的观测
using Probe = void (*)(double, const terra::physics::PhysicsWorld&, Metrics&);

Metrics runScenario(const char* label, const std::string& scenePath, const std::string& motionPath, double seconds, const char* root,
                    Probe probe = nullptr) {
    Metrics m;
    auto loaded = sc::loadScene(scenePath, terra::physics::mujoco::composeWorld);
    check(loaded.ok(), std::string{label} + "：加载场景 " + scenePath);
    if (!loaded.ok()) {
        m.ok = false;
        return m;
    }
    sc::SceneInstance& s = *loaded.scene;
    terra::physics::PhysicsWorld& world = *s.world;
    const td::ModelDescription& d = world.description();
    sim::PdLayer pd(d, s.pd);
    std::optional<sim::MotionPlayer> player;
    if (!motionPath.empty()) {
        const auto parsed = sim::readMotionFile(motionPath);
        const auto bound = parsed.document ? sim::bindMotion(*parsed.document, d, pd.targetRangeFunction()) : sim::MotionBindResult{};
        check(bound.motion.has_value(), std::string{label} + "：动作可以绑定 " + motionPath);
        if (!bound.motion) {
            m.ok = false;
            return m;
        }
        if (bound.motion->startKeyframe) world.resetToKeyframe(*bound.motion->startKeyframe);
        player.emplace(*bound.motion);
        player->start(world.stepIndex());
        if (seconds <= 0) seconds = bound.motion->duration + 1.0;
    }
    double totalMass = 0.0;
    for (const auto& e : d.entities) totalMass += e.mass;
    const auto* base = d.findEntity(root);
    check(base != nullptr, std::string{label} + "：找到基座 " + root);
    const td::Vec3 start = base ? world.snapshot()->entities[base->id.value - 1].pose.position : td::Vec3{};
    const double dt = d.timestep;
    const auto steps = static_cast<std::uint64_t>(std::llround(seconds / dt));
    const auto lastSecond = static_cast<std::uint64_t>(std::llround(1.0 / dt));
    double forceSum = 0.0;
    std::uint64_t forceSamples = 0;
    for (std::uint64_t k = 0; k < steps; ++k) {
        auto commands = player ? player->commands(world.stepIndex(), dt) : std::vector<terra::physics::ActuatorCommand>{};
        if (!pd.empty()) commands = pd.process(std::move(commands), *world.snapshot());
        if (const auto error = world.setControl(commands)) {
            check(false, std::string{label} + "：控制被拒绝：" + error->message);
            m.ok = false;
            break;
        }
        if (const auto error = world.step()) {
            check(false, std::string{label} + "：第 " + std::to_string(error->step) + " 步出错：" + error->message);
            m.ok = false;
            break;
        }
        const auto snap = world.snapshot();
        // 开始后 0.5 s 内的超出单独记录、不计入判据：Menagerie 的 Stretch 在初始姿态下夹爪与手臂连杆互相穿透，
        // 第一次运行中 0.08 s 时腕部俯仰被接触力（约 1.5 kN）推出限位 0.07 rad（M3-06 报告第 4 节）
        const bool settling = static_cast<double>(k + 1) * dt <= kSettleSeconds;
        for (const auto& j : d.joints) {
            if (!j.limited) continue;
            if (settling) {
                const double q0 = snap->joints[j.id.value - 1].position[0];
                m.settlingViolation = std::max(m.settlingViolation, std::max(j.rangeMin - q0, q0 - j.rangeMax));
                continue;
            }
            const double q = snap->joints[j.id.value - 1].position[0];
            const double v = std::max(j.rangeMin - q, q - j.rangeMax);
            if (v > m.limitViolation) {
                m.limitViolation = v;
                m.worstJoint = j.name + "，限位 [" + std::to_string(j.rangeMin) + ", " + std::to_string(j.rangeMax) + "]";
                m.worstTime = static_cast<double>(k + 1) * dt;
                m.worstValue = q;
                m.worstContacts.clear();
                for (const auto& pair : snap->contacts.pairs) {
                    if (!pair.entityA.valid()) continue;
                    const auto* a = d.entity(pair.entityA);
                    const auto* b = d.entity(pair.entityB);
                    char force[32];
                    std::snprintf(force, sizeof force, "%.1f N", pair.normalForce);
                    m.worstContacts += (m.worstContacts.empty() ? "" : "；") + (a ? a->name : std::string{"?"}) + " – " +
                                       (b ? b->name : std::string{"?"}) + " " + force;
                }
            }
        }
        if (base) {
            const auto& pose = snap->entities[base->id.value - 1].pose;
            m.zMin = std::min(m.zMin, pose.position.z);
            m.zMax = std::max(m.zMax, pose.position.z);
            m.tiltMaxDeg = std::max(m.tiltMaxDeg, tiltDeg(pose.orientation));
            m.drift = std::max(m.drift, std::hypot(pose.position.x - start.x, pose.position.y - start.y));
        }
        for (const auto& pair : snap->contacts.pairs) {
            if (pair.entityA.valid()) continue;
            if (const auto* e = d.entity(pair.entityB)) m.firstGround.emplace(e->name, static_cast<double>(k + 1) * dt);
        }
        if (k + lastSecond >= steps) {
            double groundForce = 0.0;
            for (const auto& pair : snap->contacts.pairs) {
                if (pair.entityA.valid()) continue;  // 只看与世界（地面）的接触
                if (const auto* e = d.entity(pair.entityB)) m.ground.insert(e->name);
                groundForce += pair.normalForce;
            }
            forceSum += groundForce;
            ++forceSamples;
        }
        if (probe) probe(static_cast<double>(k + 1) * dt, world, m);
    }
    m.seconds = static_cast<double>(world.stepIndex()) * dt;
    if (forceSamples > 0 && totalMass > 0) m.weightRatio = forceSum / static_cast<double>(forceSamples) / (totalMass * 9.81);
    check(world.snapshot()->diagnostics.finite, std::string{label} + "：状态有限");
    std::string ground;
    for (const auto& g : m.ground) ground += (ground.empty() ? "" : "、") + g;
    std::printf("INFO [%s] %.2f s：限位超出最大 %.5f（%s）；基座高度 %.4f–%.4f m，倾斜最大 %.2f°，水平漂移 %.4f m；"
                "最后 1 s 接触地面：%s；接触力 / 重量 %.4f（总质量 %.3f kg）\n",
                label, m.seconds, m.limitViolation, m.worstJoint.empty() ? "-" : m.worstJoint.c_str(), m.zMin, m.zMax, m.tiltMaxDeg,
                m.drift, ground.empty() ? "无" : ground.c_str(), m.weightRatio, totalMass);
    for (const auto& [name, value] : m.probes) std::printf("INFO [%s] %s = %.5f\n", label, name.c_str(), value);
    std::string first;
    for (const auto& [name, t] : m.firstGround) {
        char buf[32];
        std::snprintf(buf, sizeof buf, " %.3f s", t);
        first += (first.empty() ? "" : "、") + name + buf;
    }
    std::printf("INFO [%s] 各实体第一次接触地面：%s\n", label, first.empty() ? "无" : first.c_str());
    if (m.settlingViolation > 0) std::printf("INFO [%s] 开始后 %.1f s 内（初始穿透释放期，不计入判据）限位超出最大 %.5f\n", label, kSettleSeconds, m.settlingViolation);
    if (m.limitViolation > 0) {
        std::printf("INFO [%s] 最大超出出现在 %.3f s：%s 位置 %.5f；此时的非地面接触：%s\n", label, m.worstTime, m.worstJoint.c_str(),
                    m.worstValue, m.worstContacts.empty() ? "无" : m.worstContacts.c_str());
    }
    return m;
}

// ---- 冻结的判据（M3-06 报告第 3 节） ----
// 关节超出限位的上限：一般 0.01；动作中把关节压向限位且有接触力的场景（Go2 趴下时小腿目标 −2.6、限位 −2.7227）
// 实测 0.0077，取 0.02
constexpr double kLimitViolationMax = 0.01;  // rad 或 m

void checkCommon(const char* label, const Metrics& m, double limit = kLimitViolationMax) {
    check(m.ok, std::string{label} + "：运行完成");
    check(m.limitViolation <= limit, std::string{label} + "：关节始终在限位之内（超出最大 " + std::to_string(m.limitViolation) +
                                         "，判据 ≤ " + std::to_string(limit) + "，" + m.worstJoint + "）");
}

bool contains(const std::set<std::string>& set, std::initializer_list<const char*> names) {
    for (const char* n : names) {
        if (set.count(n) == 0) return false;
    }
    return true;
}

void modelChecks(const char* label, const std::string& scenePath) {
    auto loaded = sc::loadScene(scenePath, terra::physics::mujoco::composeWorld);
    if (!loaded.ok()) return;
    const td::ModelDescription& d = loaded.scene->world->description();
    check(d.timestep == 0.002, std::string{label} + "：步长 0.002 s");
    check(d.settings.gravity.x == 0 && d.settings.gravity.y == 0 && std::fabs(d.settings.gravity.z + 9.81) < 1e-9,
          std::string{label} + "：重力 (0, 0, −9.81)");
    std::size_t axes = 0, ranges = 0, bad = 0;
    for (const auto& j : d.joints) {
        if (j.type == td::JointType::Hinge || j.type == td::JointType::Slide) {
            ++axes;
            const double n = std::sqrt(j.axis.x * j.axis.x + j.axis.y * j.axis.y + j.axis.z * j.axis.z);
            if (std::fabs(n - 1.0) > 1e-9) ++bad;
            if (j.limited) {
                ++ranges;
                if (!(j.rangeMin < j.rangeMax)) ++bad;
            }
        }
    }
    double mass = 0.0;
    for (const auto& e : d.entities) {
        if (e.mass < 0) ++bad;
        mass += e.mass;
    }
    std::printf("INFO [%s] 模型核查：%zu 个转动 / 滑动关节，%zu 个有限位，总质量 %.3f kg\n", label, axes, ranges, mass);
    check(bad == 0, std::string{label} + "：关节轴为单位向量、限位区间 min < max、质量非负（问题 " + std::to_string(bad) + " 处）");
}

std::string asset(const std::string& rel) { return std::string{TERRA_ASSETS_DIR} + "/" + rel; }

void stretchProbe(double t, const terra::physics::PhysicsWorld& world, Metrics& m) {
    const auto& d = world.description();
    const auto snap = world.snapshot();
    // 动作 tour（2026-11-24 修订）：先把升降柱由 0.6 升到 0.95 m（0–1.5 s），再两轮同速前进（2.5–5.5 s）
    if (std::fabs(t - 6.0) < 1e-9) {
        if (const auto* b = d.findEntity("stretch3/base_link")) m.probes["底盘 6 s 的 x（m）"] = snap->entities[b->id.value - 1].pose.position.x;
    }
    if (const auto* j = d.findJoint("stretch3/joint_lift")) {
        if (std::fabs(t - 0.5) < 1e-9) m.probes["升降柱 0.5 s（m）"] = snap->joints[j->id.value - 1].position[0];
        if (std::fabs(t - 3.0) < 1e-9) m.probes["升降柱 3 s（m）"] = snap->joints[j->id.value - 1].position[0];
    }
}

// G1 悬挂下蹲的诊断（2026-11-24：第一次运行中双脚在 4.35 s 触地，原因待查）：左脚连杆的最低高度，以及此时的腿部关节角
void squatProbe(double t, const terra::physics::PhysicsWorld& world, Metrics& m) {
    const auto& d = world.description();
    const auto snap = world.snapshot();
    const auto* foot = d.findEntity("g1/left_ankle_roll_link");
    if (foot == nullptr) return;
    const double z = snap->entities[foot->id.value - 1].pose.position.z;
    const auto it = m.probes.find("左脚最低高度（m）");
    if (it == m.probes.end() || z < it->second) {
        m.probes["左脚最低高度（m）"] = z;
        m.probes["左脚最低时刻（s）"] = t;
        for (const char* j : {"g1/left_hip_pitch_joint", "g1/left_knee_joint", "g1/left_ankle_pitch_joint"}) {
            if (const auto* joint = d.findJoint(j)) m.probes[std::string{"此时 "} + j] = snap->joints[joint->id.value - 1].position[0];
        }
    }
    if (std::fabs(t - 1.0) < 1e-9) m.probes["左脚 1 s 高度（m）"] = z;
    if (std::fabs(t - 3.0) < 1e-9) m.probes["左脚 3 s 高度（m，下蹲保持中）"] = z;
}

}  // namespace

int main() {
    for (const char* folder : {"unitree_g1", "unitree_go2", "hello_robot_stretch_3"}) {
        if (!fs::exists(asset(std::string{"robots/"} + folder + "/scene.xml"))) {
            std::printf("INFO 参考机器人未准备（运行 scripts/assets/fetch_robots.sh），跳过\n");
            return kSkipped;
        }
    }
    modelChecks("g1", asset("scenes/robots/g1.terra-scene.json"));
    modelChecks("go2", asset("scenes/robots/go2.terra-scene.json"));
    modelChecks("stretch3", asset("scenes/robots/stretch3.terra-scene.json"));

    // G1 自由站立：骨盆高度 0.75–0.82 m，倾斜 ≤ 5°，漂移 ≤ 0.05 m；双脚着地；接触力 / 重量 0.95–1.05
    {
        const Metrics m = runScenario("g1_stand", asset("scenes/robots/g1.terra-scene.json"), asset("motions/g1/stand_hold.terra-motion.json"),
                                      0, "g1/pelvis");
        checkCommon("g1_stand", m);
        // 冻结（2026-11-24 实测：骨盆 0.7900–0.7916 m，倾斜 0.07°，漂移 0.0002 m，接触力 / 重量 1.0000）
        check(m.zMin >= 0.78 && m.zMax <= 0.80, "g1_stand：骨盆高度在 0.78–0.80 m 之内");
        check(m.tiltMaxDeg <= 1.0, "g1_stand：倾斜 ≤ 1°");
        check(m.drift <= 0.01, "g1_stand：水平漂移 ≤ 0.01 m");
        check(contains(m.ground, {"g1/left_ankle_roll_link", "g1/right_ankle_roll_link"}), "g1_stand：双脚（踝关节滚转连杆）着地");
        check(m.weightRatio >= 0.98 && m.weightRatio <= 1.02, "g1_stand：地面接触力与重量平衡（0.98–1.02）");
    }
    // G1 悬挂动作：限位；悬挂时任何部位都不接触地面（2026-11-24：此前悬挂场景把 Menagerie 的 scene.xml 整体抬高 0.3 m，
    // 地面随之抬高，双脚离地只有约 1 mm，下蹲后站起时踩到地面；现改为机器人本体 + 单独的地面，见 M3-06 报告第 6 节）
    for (const auto& [label, motion] : std::vector<std::pair<const char*, const char*>>{{"g1_wave", "motions/g1/wave.terra-motion.json"},
                                                                                        {"g1_squat", "motions/g1/squat.terra-motion.json"}}) {
        const Metrics m = runScenario(label, asset("scenes/robots/g1_suspended.terra-scene.json"), asset(motion), 0, "g1/pelvis",
                                      std::string{label} == "g1_squat" ? squatProbe : nullptr);
        checkCommon(label, m);
        check(m.firstGround.empty(), std::string{label} + "：悬挂时不接触地面");
    }
    // Go2 保持 home：机身高度 0.22–0.30 m，倾斜 ≤ 5°，漂移 ≤ 0.05 m；四足着地；接触力 / 重量 0.95–1.05
    {
        const Metrics m = runScenario("go2_hold", asset("scenes/robots/go2.terra-scene.json"), "", 3.0, "go2/base");
        checkCommon("go2_hold", m);
        // 冻结（2026-11-24 实测：机身 0.2507–0.2700 m，倾斜 0.91°，漂移 0.0294 m，接触力 / 重量 1.0000）
        check(m.zMin >= 0.24 && m.zMax <= 0.28, "go2_hold：机身高度在 0.24–0.28 m 之内");
        check(m.tiltMaxDeg <= 2.0, "go2_hold：倾斜 ≤ 2°");
        check(m.drift <= 0.05, "go2_hold：水平漂移 ≤ 0.05 m");
        check(contains(m.ground, {"go2/FL_calf", "go2/FR_calf", "go2/RL_calf", "go2/RR_calf"}), "go2_hold：四足着地");
        check(m.weightRatio >= 0.98 && m.weightRatio <= 1.02, "go2_hold：地面接触力与重量平衡（0.98–1.02）");
    }
    // Go2 动作：限位；结束时回到 home 高度附近
    {
        const Metrics m = runScenario("go2_motion", asset("scenes/robots/go2.terra-scene.json"),
                                      asset("motions/go2/lie_stand_sit.terra-motion.json"), 0, "go2/base");
        checkCommon("go2_motion", m, 0.02);
        check(contains(m.ground, {"go2/FL_calf", "go2/FR_calf", "go2/RL_calf", "go2/RR_calf"}), "go2_motion：结束时四足着地");
    }
    // Stretch：限位；两轮同速时向 +x 前进；升降柱随目标升高；结束时两轮着地
    {
        const Metrics m = runScenario("stretch", asset("scenes/robots/stretch3.terra-scene.json"), asset("motions/stretch3/tour.terra-motion.json"),
                                      0, "stretch3/base_link", stretchProbe);
        checkCommon("stretch", m);
        const auto x = m.probes.find("底盘 6 s 的 x（m）");
        check(x != m.probes.end() && x->second > 0.1, "stretch：两轮正向同速时底盘沿 +x 前进 > 0.1 m（轮速正方向即前进方向）");
        const auto l0 = m.probes.find("升降柱 0.5 s（m）");
        const auto l3 = m.probes.find("升降柱 3 s（m）");
        check(l0 != m.probes.end() && l3 != m.probes.end() && l3->second > l0->second + 0.15,
              "stretch：升降柱随目标升高（3 s 比 0.5 s 高 0.15 m 以上；单位为 m）");
        check(contains(m.ground, {"stretch3/link_left_wheel", "stretch3/link_right_wheel"}), "stretch：两轮着地");
    }

    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
