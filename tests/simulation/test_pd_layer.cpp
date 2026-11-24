// PD 控制层（M3-04b）与快照中的执行器状态。
//   - 场景校验：只接受作用于单个转动 / 滑动关节的力矩执行器；"<实例>/*" 展开；未知执行器、重复接管报错；
//   - 换算：τ = kp (q* − q) − kd q̇，u = τ / (增益 × 传动比)，截断到控制范围；目标范围取关节限位（没有限位时不限制）；
//   - process：属于本层的命令更新目标、不直接下发；其余命令原样保留；没有目标时以当前关节位置为目标（保持姿态）；
//   - 闭环：悬挂的机械臂，滑动关节（力矩执行器，传动比 2）由 PD 层驱动到 0.1 m；动作文件给出的目标经 PD 层执行，
//     越界检查改用关节范围；与帧率无关（逐位相同）；
//   - 快照的执行器状态：控制量即这一步所用的值，输出力与控制量一致（力矩执行器：力 = 增益 × 控制量）。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>

#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/scene/scene.hpp"
#include "terra/simulation/motion.hpp"
#include "terra/simulation/pd_layer.hpp"
#include "terra/simulation/runner.hpp"

#ifndef TERRA_TEST_ASSETS_DIR
#error "需要 TERRA_TEST_ASSETS_DIR"
#endif

namespace {

namespace fs = std::filesystem;
namespace sc = terra::scene;
namespace sim = terra::simulation;
namespace td = terra::data;
namespace tp = terra::physics;
using Severity = sc::SceneIssue::Severity;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

std::string dump(const std::vector<sc::SceneIssue>& issues) {
    std::string text;
    for (const auto& i : issues) text += std::string{"\n  "} + (i.severity == Severity::Error ? "错误 " : "警告 ") + i.where + "：" + i.message;
    return text;
}

bool has(const std::vector<sc::SceneIssue>& issues, const std::string& where, const std::string& part) {
    for (const auto& i : issues) {
        if (i.severity == Severity::Error && i.where == where && i.message.find(part) != std::string::npos) return true;
    }
    return false;
}

// 悬挂的机械臂；controllers 由参数给出
sc::SceneLoadResult load(const std::string& controllers) {
    const std::string text = R"({"format": "terra.scene", "version": 1,
        "models": [{"name": "arm", "mjcf": "../robot_description.xml", "fixedBase": true}],
        "controllers": )" + controllers + "}";
    auto parsed = sc::parseScene(text, (fs::path{TERRA_TEST_ASSETS_DIR} / "scene" / "inline.terra-scene.json").string());
    check(parsed.document.has_value(), "内联场景解析" + dump(parsed.issues));
    if (!parsed.document) return {};
    return sc::instantiate(std::move(*parsed.document), tp::mujoco::composeWorld);
}

constexpr const char* kPd = R"([{"type": "pd", "actuators": ["arm/extend_motor"], "kp": 20, "kd": 1}])";

void testSceneValidation() {
    auto ok = load(kPd);
    check(ok.ok() && ok.scene->pd.size() == 1 && ok.scene->pd[0].kp == 20 && ok.scene->pd[0].kd == 1, "PD 层接管 arm/extend_motor" + dump(ok.issues));
    auto position = load(R"([{"type": "pd", "actuators": ["arm/shoulder_pos"], "kp": 1}])");
    check(!position.ok() && has(position.issues, "/controllers/0/actuators/0", "只接受作用于单个转动或滑动关节的力矩执行器"),
          "位置执行器不能由 PD 层接管" + dump(position.issues));
    auto wildcard = load(R"([{"type": "pd", "actuators": ["arm/*"], "kp": 1}])");
    check(!wildcard.ok() && has(wildcard.issues, "/controllers/0/actuators/0", "arm/shoulder_pos"),
          "“arm/*” 展开后包含非力矩执行器：逐个报错" + dump(wildcard.issues));
    auto unknown = load(R"([{"type": "pd", "actuators": ["arm/nope"], "kp": 1}])");
    check(!unknown.ok() && has(unknown.issues, "/controllers/0/actuators/0", "找不到执行器"), "未知执行器" + dump(unknown.issues));
    auto noPrefix = load(R"([{"type": "pd", "actuators": ["ghost/*"], "kp": 1}])");
    check(!noPrefix.ok() && has(noPrefix.issues, "/controllers/0/actuators/0", "ghost/"), "通配的前缀不存在" + dump(noPrefix.issues));
    auto twice = load(R"([{"type": "pd", "actuators": ["arm/extend_motor"], "kp": 1}, {"type": "pd", "actuators": ["arm/extend_motor"], "kp": 2}])");
    check(!twice.ok() && has(twice.issues, "/controllers/1/actuators/0", "已由前面的 PD 层接管"), "重复接管" + dump(twice.issues));
}

void testFormula(const sc::SceneInstance& s) {
    const auto& d = s.world->description();
    sim::PdLayer pd(d, s.pd);
    const auto* motor = d.findActuator("arm/extend_motor");
    const auto* shoulder = d.findActuator("arm/shoulder_pos");
    if (!motor || !shoulder) return;
    check(pd.size() == 1 && pd.manages(motor->id) && !pd.manages(shoulder->id), "PD 层只接管 extend_motor");
    // 传动比 2、增益 1：u = (20 × (0.1 − 0.05) − 1 × 0.02) / 2 = 0.49
    check(std::fabs(pd.control(motor->id, 0.1, 0.05, 0.02) - 0.49) < 1e-12, "u = (kp (q* − q) − kd q̇) / (增益 × 传动比)");
    check(pd.control(motor->id, 5.0, 0.0, 0.0) == 1.0 && pd.control(motor->id, -5.0, 0.0, 0.0) == -1.0, "截断到控制范围 [−1, 1]");
    check(!pd.targetRange(*motor).has_value(), "滑动关节没有限位：目标不限制");
    const auto r = pd.targetRange(*shoulder);
    check(r && r->first == -1.0 && r->second == 1.0, "不由 PD 层接管的执行器：目标范围为控制范围");

    // process：没有目标时以当前位置为目标；其余命令原样保留；本层的命令只更新目标
    // （先按关键帧 raised 重置，使滑动关节不在 0：0.05 m）
    if (const int k = d.findKeyframe("arm/raised"); k >= 0) s.world->resetToKeyframe(static_cast<std::size_t>(k));
    const auto snap = s.world->snapshot();
    auto out = pd.process({{shoulder->id, {0.3}}}, *snap);
    check(out.size() == 2 && out[0].actuator == shoulder->id && out[0].values[0] == 0.3, "其余命令原样保留");
    const auto* extend = d.findJoint("arm/extend");
    const double q = snap->joints[extend->id.value - 1].position[0];
    check(q != 0.0 && pd.target(motor->id) && *pd.target(motor->id) == q, "没有目标时以当前关节位置为目标（保持姿态，0.05 m）");
    out = pd.process({{motor->id, {0.1}}}, *snap);
    check(out.size() == 1 && *pd.target(motor->id) == 0.1, "本层的命令只更新目标，不直接下发");
    pd.clearTargets();
    check(!pd.target(motor->id), "clearTargets");
}

struct Result {
    double extend = 0.0;
    std::vector<td::Pose> poses;
};

Result drive(sc::SceneInstance& s, const std::vector<double>& frames, std::uint64_t steps, const sim::BoundMotion* motion) {
    const auto& d = s.world->description();
    s.world->reset();
    sim::PdLayer pd(d, s.pd);
    sim::MotionPlayer player;
    if (motion) {
        player = sim::MotionPlayer(*motion);
        player.start(0);
    } else if (const auto* motor = d.findActuator("arm/extend_motor")) {
        pd.setTarget(motor->id, 0.1);
    }
    sim::SimulationRunner runner(*s.world);
    const double dt = d.timestep;
    runner.setController([&](std::uint64_t step) { return pd.process(player.commands(step, dt), *s.world->snapshot()); });
    std::size_t f = 0;
    while (s.world->stepIndex() < steps) {
        const double remaining = static_cast<double>(steps - s.world->stepIndex());
        runner.update(std::min(frames[f++ % frames.size()], (remaining - 0.5) * dt));
    }
    Result r;
    if (const auto* j = d.findJoint("arm/extend")) r.extend = s.world->snapshot()->joints[j->id.value - 1].position[0];
    for (const auto& e : s.world->snapshot()->entities) r.poses.push_back(e.pose);
    return r;
}

bool same(const Result& a, const Result& b) {
    if (a.poses.size() != b.poses.size()) return false;
    for (std::size_t i = 0; i < a.poses.size(); ++i) {
        const auto& p = a.poses[i].position;
        const auto& q = b.poses[i].position;
        if (p.x != q.x || p.y != q.y || p.z != q.z) return false;
    }
    return a.extend == b.extend;
}

void testClosedLoop(sc::SceneInstance& s) {
    const Result held = drive(s, {1.0 / 60}, 1500, nullptr);
    check(std::fabs(held.extend - 0.1) < 0.005, "PD 层把滑动关节驱动到 0.1 m（3 s 后 " + std::to_string(held.extend) + "）");

    // 动作文件经 PD 层执行：目标为关节位置，越界检查改用关节范围（滑动关节没有限位，2.0 m 也允许）
    const auto& d = s.world->description();
    sim::PdLayer pd(d, s.pd);
    const auto parsed = sim::parseMotion(R"({"format": "terra.motion", "version": 1, "interpolation": "linear",
        "poses": [{"time": 0, "targets": {"arm/extend_motor": 0.0}}, {"time": 1, "targets": {"arm/extend_motor": 0.2}}]})",
                                         std::string{TERRA_TEST_ASSETS_DIR} + "/motion/inline.terra-motion.json");
    check(parsed.document.has_value(), "内联动作解析");
    if (!parsed.document) return;
    const auto bound = sim::bindMotion(*parsed.document, d, pd.targetRangeFunction());
    check(bound.motion.has_value(), "PD 层接管的执行器：目标按关节范围校验（0.2 m 超出控制范围 [−1, 1] 的含义不同，但不越关节范围）");
    const auto plain = sim::bindMotion(*parsed.document, d);
    check(plain.motion.has_value(), "（对照）按控制范围也不越界");
    const auto far = sim::parseMotion(R"({"format": "terra.motion", "version": 1, "poses": [{"time": 0, "targets": {"arm/extend_motor": 2.0}}]})",
                                      std::string{TERRA_TEST_ASSETS_DIR} + "/motion/inline.terra-motion.json");
    if (far.document) {
        check(sim::bindMotion(*far.document, d, pd.targetRangeFunction()).motion.has_value() &&
                  !sim::bindMotion(*far.document, d).motion.has_value(),
              "目标 2.0 m：PD 层下合法（关节没有限位），按控制范围则越界");
    }
    if (!bound.motion) return;
    const Result a = drive(s, {1.0 / 60}, 1500, &*bound.motion);
    const Result b = drive(s, {1.0 / 144, 0.031, 0.004}, 1500, &*bound.motion);
    check(std::fabs(a.extend - 0.2) < 0.01, "动作结束后滑动关节到达 0.2 m（实际 " + std::to_string(a.extend) + "）");
    check(same(a, b), "PD 层 + 动作：不同帧间隔逐位相同");
}

void testActuatorState(sc::SceneInstance& s) {
    const auto& d = s.world->description();
    s.world->reset();
    const auto* motor = d.findActuator("arm/extend_motor");
    const auto* shoulder = d.findActuator("arm/shoulder_pos");
    if (!motor || !shoulder) return;
    check(!s.world->setControl({{motor->id, {0.25}}, {shoulder->id, {0.4}}}), "设置控制量");
    check(!s.world->step(), "推进一步");
    const auto snap = s.world->snapshot();
    check(snap->actuators.size() == d.actuators.size(), "快照中每个执行器一项");
    const auto& m = snap->actuators[motor->id.value - 1];
    const auto& p = snap->actuators[shoulder->id.value - 1];
    check(m.id == motor->id && m.controls.size() == 1 && m.controls[0] == 0.25, "力矩执行器：控制量 0.25");
    check(std::fabs(m.force - 0.25) < 1e-12, "力矩执行器：输出力 = 增益 × 控制量 = 0.25（传动比之前）");
    check(p.controls.size() == 1 && p.controls[0] == 0.4 && p.force > 0, "位置执行器：控制量 0.4，输出力朝目标（为正）");
    check(td::PhysicsSnapshot::kContractRevision >= 2, "快照修订号 ≥ 2（M4-02 起为 3）");
}

}  // namespace

int main() {
    testSceneValidation();
    auto loaded = load(kPd);
    if (loaded.ok()) {
        testFormula(*loaded.scene);
        testClosedLoop(*loaded.scene);
        testActuatorState(*loaded.scene);
    }
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
