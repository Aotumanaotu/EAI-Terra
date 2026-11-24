// 动作文件与插值（M3-04）：纯逻辑，不需要物理后端。
//   - 夹具 assets/tests/motion/arm_swing.terra-motion.json 的全部字段与路径展开；
//   - 错误用例：JSON 指针与原因（执行器名中的 “/” 写作 “~1”）；一次报出全部错误（夹具 invalid.terra-motion.json）；
//   - 插值：step / linear / smooth 的数值；第一个点之前不下发；最后一个点之后保持；
//   - 播放器：开始步、动作时间、结束、循环、未播放时不下发。
// 另带 --diagnose <文件> 演示模式（验证脚本打印诊断）。

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>

#include "terra/simulation/motion.hpp"

#ifndef TERRA_TEST_ASSETS_DIR
#error "需要 TERRA_TEST_ASSETS_DIR"
#endif

namespace {

namespace fs = std::filesystem;
namespace sim = terra::simulation;
using Severity = sim::MotionIssue::Severity;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

const std::string kDir = (fs::path{TERRA_TEST_ASSETS_DIR} / "motion").lexically_normal().string();
const std::string kPath = kDir + "/inline.terra-motion.json";

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

std::string motion(const std::string& extra, const std::string& poses = R"([{"time": 0, "targets": {"a/x": 1}}])") {
    return R"({"format": "terra.motion", "version": 1, "poses": )" + poses + extra + "}";
}

void expectError(const std::string& text, const std::string& where, const std::string& part, const std::string& what) {
    const auto r = sim::parseMotion(text, kPath);
    check(!r.document && has(r.issues, Severity::Error, where, part), what + "（应在 " + where + " 报“" + part + "”）" + dump(r.issues));
}

void testFixture() {
    const auto r = sim::readMotionFile(kDir + "/arm_swing.terra-motion.json");
    check(r.document.has_value() && r.issues.empty(), "读取夹具 arm_swing.terra-motion.json" + dump(r.issues));
    if (!r.document) return;
    const auto& d = *r.document;
    check(d.name == "机械臂摆动（M3-04 测试夹具）", "动作名");
    check(d.scene == (fs::path{TERRA_TEST_ASSETS_DIR} / "scene" / "keyframe_arm.terra-scene.json").lexically_normal().string(),
          "推荐场景按动作文件目录展开：" + d.scene);
    check(d.startKeyframe == "arm/rest" && d.interpolation == sim::MotionInterpolation::Smooth, "起始关键帧与插值方式");
    check(d.poses.size() == 4 && d.poses[1].time == 0.5 && d.poses[1].targets.size() == 1 && d.poses[3].where == "/poses/3",
          "4 个时间点；第 2 个点只有一个执行器");

    const auto minimal = sim::parseMotion(motion(""), kPath);
    check(minimal.document && minimal.document->interpolation == sim::MotionInterpolation::Smooth &&
              minimal.document->scene.empty() && minimal.document->startKeyframe.empty(),
          "默认：smooth 插值、没有推荐场景、从当前状态开始" + dump(minimal.issues));
    const auto missingScene = sim::parseMotion(motion(R"(, "scene": "no_such.terra-scene.json")"), kPath);
    check(missingScene.document && has(missingScene.issues, Severity::Warning, "/scene", "不存在"), "推荐场景不存在：只警告" + dump(missingScene.issues));
}

void testCatalog() {
    const std::string scene = (fs::path{TERRA_TEST_ASSETS_DIR} / "scene" / "keyframe_arm.terra-scene.json").lexically_normal().string();
    const auto found = sim::findMotionsForScene({kDir}, scene);
    check(found.size() == 1 && found[0].name == "机械臂摆动（M3-04 测试夹具）" &&
              found[0].path == kDir + "/arm_swing.terra-motion.json",
          "动作目录：keyframe_arm 场景找到 arm_swing（有错误的 invalid 被跳过）");
    const auto other = sim::findMotionsForScene({kDir}, (fs::path{TERRA_TEST_ASSETS_DIR} / "scene" / "two_carts.terra-scene.json").string());
    check(other.empty(), "动作目录：其他场景没有动作");
    check(sim::findMotionsForScene({kDir + "/no_such_dir"}, scene).empty(), "动作目录：目录不存在时为空");
}

void testErrors() {
    expectError("{ nope", "", "JSON 语法错误", "JSON 语法错误");
    expectError(R"({"version": 1, "poses": []})", "", "缺少 format", "缺少 format");
    expectError(R"({"format": "terra.scene", "version": 1, "poses": [{"time": 0, "targets": {"a/x": 1}}]})", "/format",
                "不是 Terra 动作文件", "format 不对");
    expectError(R"({"format": "terra.motion", "version": 2, "poses": [{"time": 0, "targets": {"a/x": 1}}]})", "/version",
                "高于本程序支持的版本 1", "更高的版本");
    expectError(motion(R"(, "extra": 1)"), "/extra", "未知字段“extra”", "根上的未知字段");
    expectError(motion(R"(, "interpolation": "cubic")"), "/interpolation", "step、linear、smooth", "未知插值方式");
    expectError(motion(R"(, "startKeyframe": "")"), "/startKeyframe", "关键帧名为空", "空的起始关键帧");
    expectError(motion("", "[]"), "/poses", "至少要有一个", "poses 为空");
    expectError(motion("", "{}"), "/poses", "应为数组", "poses 不是数组");
    expectError(motion("", R"([{"time": -1, "targets": {"a/x": 1}}])"), "/poses/0/time", "不能为负", "负时间");
    expectError(motion("", R"([{"time": 1, "targets": {"a/x": 1}}, {"time": 1, "targets": {"a/x": 2}}])"), "/poses/1/time", "严格递增",
                "时间相同");
    expectError(motion("", R"([{"targets": {"a/x": 1}}])"), "/poses/0", "缺少 time", "缺少时间");
    expectError(motion("", R"([{"time": 0, "targets": []}])"), "/poses/0/targets", "应为对象", "targets 不是对象");
    expectError(motion("", R"([{"time": 0, "targets": {}}])"), "/poses/0/targets", "至少要有一个执行器", "targets 为空");
    expectError(motion("", R"([{"time": 0, "targets": {"g1/left_knee": "up"}}])"), "/poses/0/targets/g1~1left_knee", "应为数值",
                "目标不是数值；指针中 “/” 写作 “~1”");
    expectError(motion("", R"([{"time": 0, "targets": {"a/x": 1}, "speed": 2}])"), "/poses/0/speed", "未知字段", "时间点中的未知字段");

    const auto many = sim::readMotionFile(kDir + "/invalid.terra-motion.json");
    check(!many.document && many.issues.size() == 5, "夹具 invalid.terra-motion.json：一次报出全部 5 个错误" + dump(many.issues));
    const auto missing = sim::readMotionFile(kDir + "/no_such.terra-motion.json");
    check(!missing.document && !missing.issues.empty() && missing.issues[0].message.find("动作文件不存在") != std::string::npos,
          "动作文件不存在");
    check(sim::jsonPointerToken("a~b/c") == "a~0b~1c", "JSON 指针转义：~ → ~0，/ → ~1");
}

bool near(double a, double b, double tol = 1e-12) { return std::fabs(a - b) <= tol; }

void testInterpolation() {
    sim::MotionTrack track;
    track.keys = {{1.0, 0.0}, {2.0, 10.0}, {4.0, 2.0}};
    using I = sim::MotionInterpolation;
    check(!sim::sampleTrack(track, I::Linear, 0.999), "第一个点之前不下发");
    check(near(*sim::sampleTrack(track, I::Linear, 1.0), 0.0) && near(*sim::sampleTrack(track, I::Linear, 2.0), 10.0),
          "在点上取点的值");
    check(near(*sim::sampleTrack(track, I::Linear, 1.25), 2.5), "linear：1.25 s → 2.5");
    check(near(*sim::sampleTrack(track, I::Step, 1.99), 0.0) && near(*sim::sampleTrack(track, I::Step, 3.0), 10.0),
          "step：保持到下一个点");
    // smooth：s = 0.25 → 0.25² × (3 − 0.5) = 0.15625
    check(near(*sim::sampleTrack(track, I::Smooth, 1.25), 1.5625), "smooth：1.25 s → 10 × 0.15625");
    check(near(*sim::sampleTrack(track, I::Smooth, 3.0), 6.0), "smooth：区间中点取两端平均");
    check(near(*sim::sampleTrack(track, I::Smooth, 9.0), 2.0) && near(*sim::sampleTrack(track, I::Linear, 4.0), 2.0),
          "最后一个点之后保持最后的值");
    // smooth 在点附近的斜率接近 0（点上速度为 0）
    const double eps = 1e-6;
    const double slope = (*sim::sampleTrack(track, I::Smooth, 2.0 + eps) - *sim::sampleTrack(track, I::Smooth, 2.0)) / eps;
    check(std::fabs(slope) < 1e-3, "smooth：在点上速度为 0（斜率 " + std::to_string(slope) + "）");
}

void testPlayer() {
    sim::BoundMotion m;
    m.duration = 1.0;
    sim::MotionTrack a;
    a.actuator = terra::data::ActuatorId{3};
    a.keys = {{0.0, 0.0}, {1.0, 1.0}};
    sim::MotionTrack late;
    late.actuator = terra::data::ActuatorId{5};
    late.keys = {{0.5, 7.0}};
    m.tracks = {a, late};
    m.interpolation = sim::MotionInterpolation::Linear;
    sim::MotionPlayer player(m);
    const double dt = 0.002;
    check(player.commands(0, dt).empty(), "没有播放时不下发");
    player.start(100);
    check(player.playing() && player.commands(99, dt).empty(), "开始步之前不下发");
    const auto first = player.commands(100, dt);
    check(first.size() == 1 && first[0].actuator.value == 3 && first[0].values == std::vector<double>{0.0},
          "开始步：动作时间 0；还没轮到的执行器不下发");
    const auto mid = player.commands(350, dt);
    check(mid.size() == 2 && near(mid[0].values[0], 0.5, 1e-12) && near(mid[1].values[0], 7.0), "第 350 步：动作时间 0.5 s");
    check(near(player.timeAt(350, dt), 0.5, 1e-12), "动作时间 = (步 − 开始步) × 步长");
    check(!player.finished(599, dt) && player.finished(600, dt), "第 600 步（1.0 s）结束");
    const auto after = player.commands(5000, dt);
    check(after.size() == 2 && near(after[0].values[0], 1.0), "结束后保持最后的值");
    player.setLoop(true);
    check(!player.finished(5000, dt) && near(player.timeAt(850, dt), 0.5, 1e-9), "循环：动作时间对时长取模，不结束");
    player.stop();
    check(!player.playing() && player.commands(350, dt).empty(), "停止后不下发");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc == 3 && std::string{argv[1]} == "--diagnose") {
        const auto r = sim::readMotionFile(argv[2]);
        std::printf("%s%s\n", r.document ? "解析成功" : "解析失败", dump(r.issues).c_str());
        return 0;
    }
    testFixture();
    testErrors();
    testCatalog();
    testInterpolation();
    testPlayer();
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
