// 动作的绑定与播放（M3-04），使用 MuJoCo 与场景文件 assets/tests/scene/keyframe_arm.terra-scene.json：
//   - 绑定：夹具 arm_swing 通过；未知执行器（列出该实例的执行器、没带前缀的提示）、越界（控制范围）、多输入、
//     未知起始关键帧报错；Other 类执行器给出警告；轨道按执行器分开、只含提到它的点；
//   - 播放：经 SimulationRunner 的每步控制器执行；不同的帧间隔（60 Hz、144 Hz、抖动、单步）得到逐位相同的轨迹；
//     与“每步手动排队同样的命令”逐位相同；动作结束后保持最后的目标（肩关节停在 0.2 rad 附近）；
//     按起始关键帧重置后重播，与第一次逐位相同。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/scene/scene.hpp"
#include "terra/simulation/motion.hpp"
#include "terra/simulation/runner.hpp"

#ifndef TERRA_TEST_ASSETS_DIR
#error "需要 TERRA_TEST_ASSETS_DIR"
#endif

namespace {

namespace fs = std::filesystem;
namespace sim = terra::simulation;
namespace td = terra::data;
using Severity = sim::MotionIssue::Severity;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

const std::string kScene = (fs::path{TERRA_TEST_ASSETS_DIR} / "scene" / "keyframe_arm.terra-scene.json").lexically_normal().string();
const std::string kMotion = (fs::path{TERRA_TEST_ASSETS_DIR} / "motion" / "arm_swing.terra-motion.json").lexically_normal().string();

std::string dump(const std::vector<sim::MotionIssue>& issues) {
    std::string text;
    for (const auto& i : issues) text += std::string{"\n  "} + (i.severity == Severity::Error ? "错误 " : "警告 ") + i.where + "：" + i.message;
    return text;
}

bool has(const std::vector<sim::MotionIssue>& issues, Severity severity, const std::string& where, const std::string& part) {
    for (const auto& i : issues) {
        if (i.severity == severity && i.where == where && i.message.find(part) != std::string::npos) return true;
    }
    return false;
}

std::optional<terra::scene::SceneInstance> loadArm() {
    auto loaded = terra::scene::loadScene(kScene, terra::physics::mujoco::composeWorld);
    check(loaded.ok(), "加载 keyframe_arm 场景");
    if (!loaded.ok()) return std::nullopt;
    return std::move(*loaded.scene);
}

sim::MotionBindResult bindText(const std::string& poses, const td::ModelDescription& d, const std::string& extra = "") {
    const auto parsed = sim::parseMotion(R"({"format": "terra.motion", "version": 1, "poses": )" + poses + extra + "}",
                                         std::string{TERRA_TEST_ASSETS_DIR} + "/motion/inline.terra-motion.json");
    check(parsed.document.has_value(), "内联动作解析" + dump(parsed.issues));
    if (!parsed.document) return {};
    return sim::bindMotion(*parsed.document, d);
}

void testBind(const td::ModelDescription& d) {
    const auto parsed = sim::readMotionFile(kMotion);
    check(parsed.document.has_value(), "读取 arm_swing" + dump(parsed.issues));
    if (!parsed.document) return;
    const auto bound = sim::bindMotion(*parsed.document, d);
    check(bound.motion.has_value() && bound.issues.empty(), "arm_swing 绑定通过" + dump(bound.issues));
    if (bound.motion) {
        const auto& m = *bound.motion;
        check(m.tracks.size() == 3 && m.duration == 1.5, "3 条轨道（肩、轮、滑动），时长 1.5 s");
        check(m.startKeyframe && d.keyframes[*m.startKeyframe].name == "arm/rest", "起始关键帧 arm/rest");
        for (const auto& t : m.tracks) {
            if (t.name == "arm/shoulder_pos") check(t.keys.size() == 4, "肩：4 个点");
            if (t.name == "arm/spin_vel") check(t.keys.size() == 2 && t.keys[1].first == 1.0, "轮：只含提到它的 2 个点");
            if (t.name == "arm/extend_motor") check(t.keys.size() == 1 && t.keys[0].first == 1.5, "滑动：只在 1.5 s 出现");
        }
    }

    const auto unknown = bindText(R"([{"time": 0, "targets": {"arm/elbow": 0.1}}])", d);
    check(!unknown.motion && has(unknown.issues, Severity::Error, "/poses/0/targets/arm~1elbow", "arm/shoulder_pos"),
          "未知执行器：列出该实例的执行器" + dump(unknown.issues));
    const auto noPrefix = bindText(R"([{"time": 0, "targets": {"shoulder_pos": 0.1}}])", d);
    check(!noPrefix.motion && has(noPrefix.issues, Severity::Error, "/poses/0/targets/shoulder_pos", "需要带实例前缀"),
          "没带实例前缀" + dump(noPrefix.issues));
    const auto outOfRange = bindText(R"([{"time": 0, "targets": {"arm/shoulder_pos": 1.5}}])", d);
    check(!outOfRange.motion && has(outOfRange.issues, Severity::Error, "/poses/0/targets/arm~1shoulder_pos", "超出执行器"),
          "越界：拒绝" + dump(outOfRange.issues));
    const auto badKey = bindText(R"([{"time": 0, "targets": {"arm/shoulder_pos": 0.1}}])", d, R"(, "startKeyframe": "arm/nope")");
    check(!badKey.motion && has(badKey.issues, Severity::Error, "/startKeyframe", "arm/raised"), "未知起始关键帧：列出关键帧" + dump(badKey.issues));
    const auto other = bindText(R"([{"time": 0, "targets": {"arm/filtered": 0.3}}, {"time": 1, "targets": {"arm/filtered": 0.1}}])", d);
    check(other.motion && other.issues.size() == 1 && has(other.issues, Severity::Warning, "/poses/0/targets/arm~1filtered", "原样下发"),
          "Other 类执行器：只警告一次" + dump(other.issues));
    const auto many = bindText(R"([{"time": 0, "targets": {"arm/x": 0, "arm/shoulder_pos": 9}}])", d, R"(, "startKeyframe": "x")");
    check(!many.motion && many.issues.size() == 3, "一次报出全部 3 个错误" + dump(many.issues));
}

struct Run {
    std::vector<td::Pose> poses;
    std::uint64_t steps = 0;
    double shoulder = 0.0;
};

// 用 runner + 每步控制器播放：先按起始关键帧重置，再按给定的帧间隔推进到 steps 步
Run play(terra::scene::SceneInstance& scene, const sim::BoundMotion& motion, const std::vector<double>& frames, std::uint64_t steps) {
    sim::SimulationRunner runner(*scene.world);
    if (motion.startKeyframe) runner.resetToKeyframe(*motion.startKeyframe);
    sim::MotionPlayer player(motion);
    player.start(scene.world->stepIndex());
    const double dt = scene.world->description().timestep;
    runner.setController([&](std::uint64_t step) { return player.commands(step, dt); });
    std::size_t f = 0;
    while (scene.world->stepIndex() < steps) {
        const double wall = frames.empty() ? 0.0 : frames[f++ % frames.size()];
        if (wall <= 0.0) {  // 单步：暂停中请求一步
            runner.pause();
            runner.stepOnce();
            runner.update(0.0);
        } else {
            // 最后几帧缩短，保证不越过目标步数（时钟余量 < 1 步，再减半步即不会多走一步）
            runner.resume();
            const double remaining = static_cast<double>(steps - scene.world->stepIndex());
            runner.update(std::min(wall, (remaining - 0.5) * dt));
        }
    }
    Run out;
    out.steps = scene.world->stepIndex();
    for (const auto& e : scene.world->snapshot()->entities) out.poses.push_back(e.pose);
    if (const auto* j = scene.world->description().findJoint("arm/shoulder")) {
        out.shoulder = scene.world->snapshot()->joints[j->id.value - 1].position[0];
    }
    return out;
}

bool identical(const Run& a, const Run& b) {
    if (a.steps != b.steps || a.poses.size() != b.poses.size()) return false;
    for (std::size_t i = 0; i < a.poses.size(); ++i) {
        const auto& p = a.poses[i];
        const auto& q = b.poses[i];
        if (p.position.x != q.position.x || p.position.y != q.position.y || p.position.z != q.position.z ||
            p.orientation.w != q.orientation.w || p.orientation.x != q.orientation.x || p.orientation.y != q.orientation.y ||
            p.orientation.z != q.orientation.z) {
            return false;
        }
    }
    return true;
}

void testPlayback() {
    auto scene = loadArm();
    if (!scene) return;
    const auto parsed = sim::readMotionFile(kMotion);
    if (!parsed.document) return;
    const auto bound = sim::bindMotion(*parsed.document, scene->world->description());
    if (!bound.motion) return;
    const sim::BoundMotion& motion = *bound.motion;
    constexpr std::uint64_t kSteps = 1500;  // 3 s：动作 1.5 s，之后保持

    // 帧间隔（墙钟，秒）；0 表示单步
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> jitter(0.001, 0.03);
    std::vector<double> random;
    for (int i = 0; i < 400; ++i) random.push_back(jitter(rng));
    const Run reference = play(*scene, motion, {1.0 / 60}, kSteps);
    const Run fast = play(*scene, motion, {1.0 / 144}, kSteps);
    const Run jittered = play(*scene, motion, random, kSteps);
    const Run stepped = play(*scene, motion, {0.0}, 300);
    check(identical(reference, fast) && identical(reference, jittered), "60 Hz、144 Hz、抖动帧间隔：轨迹逐位相同");
    const Run reference300 = play(*scene, motion, {1.0 / 60}, 300);
    check(identical(reference300, stepped), "单步推进与连续推进逐位相同（300 步）");
    check(std::fabs(reference.shoulder - 0.2) < 0.05, "动作结束后保持最后的目标：肩关节 " + std::to_string(reference.shoulder) + " rad ≈ 0.2");

    // 与“每步手动排队同样的命令”逐位相同
    {
        sim::SimulationRunner runner(*scene->world);
        runner.resetToKeyframe(*motion.startKeyframe);
        sim::MotionPlayer player(motion);
        player.start(0);
        const double dt = scene->world->description().timestep;
        for (std::uint64_t k = 0; k < kSteps; ++k) {
            auto commands = player.commands(k, dt);
            if (!commands.empty()) runner.schedule(k, std::move(commands));
        }
        runner.pause();
        while (scene->world->stepIndex() < kSteps) {
            runner.stepOnce();
            runner.update(0.0);
        }
        Run manual;
        manual.steps = scene->world->stepIndex();
        for (const auto& e : scene->world->snapshot()->entities) manual.poses.push_back(e.pose);
        check(identical(reference, manual), "每步控制器与预先排队的命令逐位相同");
    }

    // 控制器拒绝的命令进入 FrameResult::controlErrors（越界由绑定拦住，这里用不存在的执行器 ID）
    {
        sim::SimulationRunner runner(*scene->world);
        runner.setController([](std::uint64_t) {
            return std::vector<terra::physics::ActuatorCommand>{{td::ActuatorId{999}, {0.0}}};
        });
        runner.pause();
        runner.stepOnce();
        const auto frame = runner.update(0.0);
        check(frame.steps == 1 && frame.controlErrors.size() == 1, "控制器给出的无效命令被报告，仿真继续");
        check(runner.hasController(), "hasController");
    }

    // 世界中的初始关键帧与动作的起始关键帧不同：重播仍逐位相同
    const Run again = play(*scene, motion, {1.0 / 60}, kSteps);
    check(identical(reference, again), "按起始关键帧重置后重播，逐位相同");
}

}  // namespace

int main() {
    if (auto scene = loadArm()) testBind(scene->world->description());
    testPlayback();
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
