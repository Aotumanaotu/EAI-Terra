// 实验记录（M3-07a）。
//   file  （纯逻辑）写入 → 读回：记录头、控制量（只在变化时写）、状态（逐位相同，含非有限值写成 null）、事件、摘要；
//         格式错误的诊断带行号；缺少 summary 只警告；版本过高、第一行不是记录头、步序号不递增都报错
//   rerun （MuJoCo）机械臂场景 + 动作 arm_swing 记录 900 步 → 复现逐位相同；改动一处控制量后报告第一次超出容差的步；
//         场景摘要不同时警告；场景中的模型与记录不同时报错
// 用法：test_record file <临时目录> | test_record rerun <临时目录>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>

#include "terra/core/sha256.hpp"
#include "terra/simulation/motion.hpp"
#include "terra/simulation/pd_layer.hpp"
#include "terra/simulation/record.hpp"

#ifdef TERRA_RECORD_RERUN
#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/scene/scene.hpp"
#endif

#ifndef TERRA_TEST_ASSETS_DIR
#error "需要 TERRA_TEST_ASSETS_DIR"
#endif

namespace {

namespace fs = std::filesystem;
namespace sim = terra::simulation;
namespace td = terra::data;
namespace tp = terra::physics;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

std::string joined(const std::vector<std::string>& lines) {
    std::string text;
    for (const auto& l : lines) text += "\n  " + l;
    return text;
}

void writeText(const std::string& path, const std::string& text) { std::ofstream(path, std::ios::binary) << text; }

std::string readText(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

std::size_t countLines(const std::string& text, const std::string& part) {
    std::size_t n = 0;
    for (std::size_t pos = 0; (pos = text.find(part, pos)) != std::string::npos; ++pos) ++n;
    return n;
}

td::PhysicsSnapshot snapshot(std::uint64_t step, double x) {
    td::PhysicsSnapshot s;
    s.stepIndex = step;
    s.simulationTime = static_cast<double>(step) * 0.002;
    s.runEpoch = 1;
    td::EntityState e;
    e.id = td::EntityId{1};
    e.pose.position = {x, 0.1 + 1e-17, -2.5e-300};  // 有效数字很多的数值，检查往返逐位相同
    e.pose.orientation = {std::sqrt(0.5), 0.0, 0.0, std::sqrt(0.5)};
    e.linearVelocity = {1.0 / 3.0, 0.0, std::numeric_limits<double>::quiet_NaN()};
    s.entities.push_back(e);
    td::JointState j;
    j.id = td::JointId{1};
    j.positionSize = 1;
    j.velocitySize = 1;
    j.position[0] = 0.123456789012345678;
    j.velocity[0] = -7.0;
    s.joints.push_back(j);
    s.contacts.candidateCount = 3;
    s.contacts.activeCount = 2;
    s.contacts.pairs.push_back({td::EntityId{}, td::EntityId{1}, 2, 9.81});
    s.diagnostics.finite = false;
    s.diagnostics.solverIterations = 4;
    return s;
}

void testFile(const std::string& dir) {
    sim::RecordHeader h;
    h.terraVersion = "0.0.0";
    h.compiler = "GNU 11";
    h.physicsBackend = "MuJoCo 4.0.1";
    h.platform = "linux x86_64";
    h.created = "2026-11-24T00:00:00Z";
    h.scenePath = "/s.terra-scene.json";
    h.sceneSha256 = std::string(64, 'a');
    h.startKeyframe = "arm/rest";
    h.timestep = 0.002;
    h.integrator = "implicitfast";
    h.solver = "Newton";
    h.iterations = 100;
    h.tolerance = 1e-8;
    h.gravity = {0, 0, -9.81};
    h.sampleEvery = 10;
    h.entities = {"arm/base"};
    h.joints = {"arm/shoulder"};
    h.actuators = {"arm/a", "arm/b"};
    const std::string path = dir + "/file.terra-record.jsonl";
    std::string error;
    auto w = sim::RecordWriter::create(path, h, error);
    check(w != nullptr, "创建记录：" + error);
    if (!w) return;
    const td::ActuatorId a{1}, b{2};
    w->state(snapshot(0, 0.5));
    w->control(0, {{a, {1.0}}, {b, {2.0}}});
    w->control(1, {{a, {1.0}}, {b, {2.0}}});  // 没有变化：不写
    w->control(2, {{a, {1.0}}, {b, {2.5}}});  // 只写 b
    w->state(snapshot(10, 0.6));
    w->event(10, "error", "示例");
    // 同一步内两次下发（查看器：排队的命令、再是每步控制器）：合并为一行，后下发的覆盖先下发的，没有变化的执行器不写
    w->control(20, {{a, {3.0}}});
    w->control(20, {{a, {4.0}}, {b, {2.5}}});
    check(w->finish(20, 0.04, "completed"), "写完");
    const std::string text = readText(path);
    check(countLines(text, "\"type\":\"control\"") == 3, "控制量只在变化时写，同一步合并（5 次调用写 3 行）");
    check(text.find("\"step\":20,\"actuators\":[[1,[4.0]]]") != std::string::npos, "同一步内后下发的覆盖先下发的；b 没有变化不写");
    check(text.find("\"step\":2,\"actuators\":[[2,[2.5]]]") != std::string::npos, "第 2 步只写变化的执行器 b");
    check(text.rfind("{\"type\":\"header\",\"format\":\"terra.record\",\"version\":1", 0) == 0, "第一行是记录头");

    const auto r = sim::readRecord(path);
    check(r.record.has_value() && r.errors.empty() && r.warnings.empty(), "读回" + joined(r.errors) + joined(r.warnings));
    if (!r.record) return;
    const auto& rec = *r.record;
    check(rec.header.sceneSha256 == h.sceneSha256 && rec.header.startKeyframe == "arm/rest" && rec.header.actuators == h.actuators &&
              rec.header.gravity.z == -9.81 && rec.header.sampleEvery == 10 && rec.header.created == h.created,
          "记录头往返");
    check(rec.controls.size() == 3 && rec.controls[1].step == 2 && rec.controls[1].commands.size() == 1 &&
              rec.controls[1].commands[0].actuator == b && rec.controls[1].commands[0].values == std::vector<double>{2.5},
          "控制量往返");
    check(rec.states.size() == 2, "两份状态");
    if (rec.states.size() == 2) {
        const auto& s = rec.states[0];
        const auto expected = snapshot(0, 0.5);
        const auto& e = s.entities.at(0);
        const auto& x = expected.entities[0];
        check(e.pose.position.y == x.pose.position.y && e.pose.position.z == x.pose.position.z &&
                  e.pose.orientation.w == x.pose.orientation.w && e.linearVelocity.x == x.linearVelocity.x,
              "数值往返逐位相同（含 1/3、√0.5、极小值）");
        check(std::isnan(e.linearVelocity.z), "非有限值写成 null，读回为 NaN");
        check(s.joints.at(0).position[0] == expected.joints[0].position[0] && s.joints[0].positionSize == 1, "关节状态往返");
        check(s.contacts.pairs.size() == 1 && !s.contacts.pairs[0].entityA.valid() && s.contacts.pairs[0].normalForce == 9.81 &&
                  !s.diagnostics.finite && s.diagnostics.solverIterations == 4,
              "接触与诊断往返");
    }
    check(rec.events.size() == 1 && rec.events[0].kind == "error" && rec.events[0].detail == "示例", "事件往返");
    check(rec.summary && rec.summary->steps == 20 && rec.summary->reason == "completed", "摘要往返");

    // 格式错误
    const auto expectError = [&](const std::string& content, const std::string& part, const std::string& what) {
        const std::string bad = dir + "/bad.terra-record.jsonl";
        writeText(bad, content);
        const auto rr = sim::readRecord(bad);
        bool found = false;
        for (const auto& e : rr.errors) found = found || e.find(part) != std::string::npos;
        check(!rr.record && found, what + "（应报“" + part + "”）" + joined(rr.errors));
    };
    const std::string headerLine = text.substr(0, text.find('\n') + 1);
    expectError("{\"type\":\"state\"}\n", "第一行应为记录头", "第一行不是记录头");
    expectError("{\"type\":\"header\",\"format\":\"terra.record\",\"version\":2}\n", "高于本程序支持的版本", "版本过高");
    expectError("{\"type\":\"header\",\"format\":\"other\",\"version\":1}\n", "不是 Terra 实验记录", "format 不对");
    expectError(headerLine + "{ not json\n", "第 2 行", "JSON 语法错误带行号");
    expectError(headerLine + "{\"type\":\"bogus\"}\n", "未知的记录类型", "未知类型");
    expectError(headerLine + "{\"type\":\"control\",\"step\":5,\"actuators\":[]}\n{\"type\":\"control\",\"step\":5,\"actuators\":[]}\n",
                "严格递增", "控制量的步序号不递增");
    expectError("", "记录文件为空", "空文件");
    // 没有 summary：只警告
    writeText(dir + "/partial.terra-record.jsonl", headerLine);
    const auto partial = sim::readRecord(dir + "/partial.terra-record.jsonl");
    check(partial.record.has_value() && !partial.warnings.empty(), "缺少 summary：可读，给出警告");
    check(!sim::readRecord(dir + "/no_such.terra-record.jsonl").record, "文件不存在");
}

#ifdef TERRA_RECORD_RERUN
// 与 Headless 相同的路径：动作 → PD 层 → 记录 → setControl → step
std::string recordArm(const std::string& path, std::uint64_t steps, sim::Record* full) {
    const std::string scenePath = std::string{TERRA_TEST_ASSETS_DIR} + "/scene/keyframe_arm.terra-scene.json";
    const std::string motionPath = std::string{TERRA_TEST_ASSETS_DIR} + "/motion/arm_swing.terra-motion.json";
    auto loaded = terra::scene::loadScene(scenePath, tp::mujoco::composeWorld);
    const auto parsed = sim::readMotionFile(motionPath);
    if (!loaded.ok() || !parsed.document) return "加载场景或动作失败";
    auto& world = *loaded.scene->world;
    sim::PdLayer pd(world.description(), loaded.scene->pd);
    const auto bound = sim::bindMotion(*parsed.document, world.description(), pd.targetRangeFunction());
    if (!bound.motion) return "绑定失败";
    world.resetToKeyframe(*bound.motion->startKeyframe);
    sim::MotionPlayer player(*bound.motion);
    player.start(world.stepIndex());
    auto header = sim::makeRecordHeader(world.description(), 10);
    header.scenePath = fs::path{scenePath}.lexically_normal().string();
    header.sceneSha256 = terra::sha256File(header.scenePath).value_or("");
    header.motionPath = parsed.document->path;
    header.motionSha256 = terra::sha256File(header.motionPath).value_or("");
    header.startKeyframe = parsed.document->startKeyframe;
    std::string error;
    auto w = sim::RecordWriter::create(path, header, error);
    if (!w) return error;
    w->state(*world.snapshot());
    if (full) {
        full->header = header;
        full->states.push_back(*world.snapshot());
    }
    const double dt = world.description().timestep;
    for (std::uint64_t k = 0; k < steps; ++k) {
        auto commands = player.commands(world.stepIndex(), dt);
        if (!pd.empty()) commands = pd.process(std::move(commands), *world.snapshot());
        if (!commands.empty()) {
            if (full) full->controls.push_back({world.stepIndex(), commands});  // 对照：每步完整的命令，不去重
            w->control(world.stepIndex(), commands);
            if (world.setControl(commands)) return "控制被拒绝";
        }
        if (world.step()) return "仿真出错";
        if (world.stepIndex() % 10 == 0 || world.stepIndex() == steps) {
            w->state(*world.snapshot());
            if (full) full->states.push_back(*world.snapshot());
        }
    }
    return w->finish(world.stepIndex(), world.simulationTime(), "completed") ? std::string{} : "写入失败";
}

void testRerun(const std::string& dir) {
    const std::string path = dir + "/arm.terra-record.jsonl";
    sim::Record full;
    const std::string error = recordArm(path, 900, &full);
    check(error.empty(), "记录机械臂动作：" + error);
    if (!error.empty()) return;
    const auto read = sim::readRecord(path);
    check(read.record.has_value() && read.errors.empty() && read.warnings.empty(), "读回记录" + joined(read.errors));
    if (!read.record) return;
    std::printf("INFO 记录：%zu 行控制量，%zu 份状态，%llu 字节\n", read.record->controls.size(), read.record->states.size(),
                static_cast<unsigned long long>(fs::file_size(path)));
    check(read.record->states.size() == 91, "状态：第 0 步与每 10 步共 91 份");
    const auto r = sim::rerunRecord(*read.record, tp::mujoco::composeWorld);
    check(r.ran && r.warnings.empty(), "复现跑完：" + r.error + joined(r.warnings));
    if (r.firstDifferenceStep) {
        std::printf("INFO 复现诊断：第一个不逐位相同的采样点在第 %llu 步：%s\n", static_cast<unsigned long long>(*r.firstDifferenceStep),
                    r.firstDifference.c_str());
    }
    // 对照：同一次运行在内存中保留每步完整的命令（不去重、不经过文件）后复现，区分“去重 / 文件往返”与“物理本身”的问题
    full.summary = sim::RecordSummary{900, 0.0, "completed"};
    const auto f = sim::rerunRecord(full, tp::mujoco::composeWorld);
    std::printf("INFO 对照（内存中、每步完整命令）：逐位相同 %s，最大位置误差 %.3g m%s%s\n", f.bitwise ? "是" : "否", f.maxPositionError,
                f.firstDifferenceStep ? "，第一个不同的采样步 " : "",
                f.firstDifferenceStep ? std::to_string(*f.firstDifferenceStep).c_str() : "");
    check(f.ran && f.bitwise, "对照：每步完整命令的复现逐位相同");
    check(r.compared == 91 && r.bitwise && r.maxPositionError == 0.0 && r.maxRotationError == 0.0 && r.withinTolerance(),
          "同一机器复现：91 个采样点逐位相同（最大位置误差 " + std::to_string(r.maxPositionError) + "）");

    // 改动第 300 步附近的一处控制量（肩关节目标）：复现应在其后的采样点超出容差
    std::string text = readText(path);
    // 注意要带逗号：不带时会先匹配到第 30 步（2026-11-24 第二次运行中改动落在了第 30 步，因此在第 40 步超出容差）
    const std::size_t at = text.find("{\"type\":\"control\",\"step\":300,");
    check(at != std::string::npos, "找到第 300 步的控制行");
    if (at != std::string::npos) {
        const std::size_t value = text.find("[[", at) + 2;  // 第一个执行器的 ID
        const std::size_t open = text.find('[', value) + 1;
        const std::size_t close = text.find(']', open);
        text.replace(open, close - open, "0.9");
        const std::string tampered = dir + "/tampered.terra-record.jsonl";
        writeText(tampered, text);
        const auto t = sim::readRecord(tampered);
        if (t.record) {
            const auto tr = sim::rerunRecord(*t.record, tp::mujoco::composeWorld);
            check(tr.ran && !tr.bitwise && tr.firstExceedStep && *tr.firstExceedStep >= 300 && *tr.firstExceedStep <= 320 &&
                      !tr.withinTolerance(),
                  "改动控制量后，在其后的第一个采样点（300–320 步）超出容差（实际 " +
                      (tr.firstExceedStep ? std::to_string(*tr.firstExceedStep) : std::string{"无"}) + "）");
            std::printf("INFO 改动一处控制量后：第一次超出容差在第 %llu 步，最大位置误差 %.3g m\n",
                        static_cast<unsigned long long>(tr.firstExceedStep.value_or(0)), tr.maxPositionError);
        }
    }

    // 场景摘要不同：警告；场景中的模型与记录不同：报错
    sim::Record changed = *read.record;
    changed.header.sceneSha256 = std::string(64, '0');
    const auto warned = sim::rerunRecord(changed, tp::mujoco::composeWorld);
    bool shaWarning = false;
    for (const auto& w : warned.warnings) shaWarning = shaWarning || w.find("sha256") != std::string::npos;
    check(warned.ran && shaWarning && warned.bitwise, "场景摘要不同：警告，仍可复现" + joined(warned.warnings));
    sim::Record other = *read.record;
    other.header.scenePath = std::string{TERRA_TEST_ASSETS_DIR} + "/scene/two_carts.terra-scene.json";
    const auto mismatch = sim::rerunRecord(other, tp::mujoco::composeWorld);
    check(!mismatch.ran && mismatch.error.find("与记录时不同") != std::string::npos, "场景中的模型与记录不同：报错：" + mismatch.error);
}
#endif

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "用法：test_record file|rerun <临时目录>\n");
        return 2;
    }
    const std::string mode = argv[1];
    const std::string dir = argv[2];
    std::error_code ec;
    fs::create_directories(dir, ec);
    if (mode == "file") testFile(dir);
#ifdef TERRA_RECORD_RERUN
    else if (mode == "rerun") testRerun(dir);
#endif
    else {
        std::fprintf(stderr, "未知模式：%s\n", mode.c_str());
        return 2;
    }
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
