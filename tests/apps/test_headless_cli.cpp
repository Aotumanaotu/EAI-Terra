// terra_headless 参数解析与 JSON 输出的单元测试（M0-03）。

#include <cstdio>
#include <initializer_list>
#include <limits>
#include <string>
#include <vector>

#include "cli.hpp"

namespace {

using namespace terra::headless;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

ParseResult parse(std::initializer_list<const char*> args) {
    std::vector<const char*> argv{"terra_headless"};
    argv.insert(argv.end(), args.begin(), args.end());
    return parseArguments(static_cast<int>(argv.size()), argv.data());
}

// 期望解析失败，且错误信息包含指定片段。
void expectError(std::initializer_list<const char*> args, const std::string& fragment) {
    std::string joined;
    for (const char* a : args) {
        joined += std::string{" "} + a;
    }
    const ParseResult r = parse(args);
    check(!r.ok(), "应失败:" + joined);
    check(r.error.find(fragment) != std::string::npos,
          "错误信息应包含 '" + fragment + "'，实际 '" + r.error + "':" + joined);
}

void testHelpAndVersion() {
    check(parse({"--help"}).command == Command::Help, "--help");
    check(parse({"-h"}).command == Command::Help, "-h");
    check(parse({"--version"}).command == Command::Version, "--version");
    // --help 优先于其余参数，即使其余参数非法
    const ParseResult r = parse({"--steps", "-1", "--help"});
    check(r.ok() && r.command == Command::Help, "--help 优先");
}

void testValidRuns() {
    {
        const ParseResult r = parse({"--model", "a.xml", "--steps", "5000"});
        check(r.ok(), "基本 --steps: " + r.error);
        check(r.command == Command::Run, "命令为 Run");
        check(r.options.modelPath == "a.xml", "模型路径");
        check(r.options.steps && *r.options.steps == 5000, "步数");
        check(!r.options.durationSeconds, "无 duration");
        check(r.options.sampleEvery == 1, "默认 sample-every 为 1");
        check(!r.options.dryRun, "默认非 dry-run");
    }
    {
        const ParseResult r =
            parse({"--duration=2.5", "--model=dir/b.xml", "--sample-every=10", "--dry-run"});
        check(r.ok(), "--opt=value 写法: " + r.error);
        check(r.options.durationSeconds && *r.options.durationSeconds == 2.5, "时长 2.5");
        check(r.options.modelPath == "dir/b.xml", "模型路径（= 写法）");
        check(r.options.sampleEvery == 10, "sample-every 10");
        check(r.options.dryRun, "dry-run");
    }
    {
        const ParseResult r = parse({"--model", "a.xml", "--duration", "1e-3"});
        check(r.ok() && r.options.durationSeconds && *r.options.durationSeconds == 1e-3, "科学计数法");
    }
    {
        const ParseResult r = parse({"--model", "a.xml", "--steps", "18446744073709551615"});
        check(r.ok() && r.options.steps && *r.options.steps == 18446744073709551615ULL, "uint64 最大值");
    }
}

// M2-08：--scene（与 --model 二选一）
void testScene() {
    const ParseResult r = parse({"--scene", "scenes/two.terra-scene.json", "--steps", "100"});
    check(r.ok() && r.options.scenePath == "scenes/two.terra-scene.json" && r.options.modelPath.empty(), "--scene: " + r.error);
    expectError({"--scene", "a.json", "--model", "a.xml", "--steps", "1"}, "只能二选一");
    expectError({"--scene", "a.json", "--scene", "b.json", "--steps", "1"}, "重复的参数 --scene");
    expectError({"--scene=", "--steps", "1"}, "不能为空");
    expectError({"--steps", "10"}, "或 --scene");
    check(usageText().find("--scene") != std::string::npos, "帮助中有 --scene");
}

// M3-05：--motion（可不带 --scene 与步数；不能与 --model 同用）
void testMotion() {
    {
        const ParseResult r = parse({"--motion", "m/wave.terra-motion.json"});
        check(r.ok() && r.options.motionPath == "m/wave.terra-motion.json" && r.options.scenePath.empty() && !r.options.steps &&
                  !r.options.durationSeconds,
              "--motion 单独使用：场景取推荐场景，步数取动作时长 + 1 s：" + r.error);
    }
    {
        const ParseResult r = parse({"--scene", "s.json", "--motion=m.json", "--steps", "50"});
        check(r.ok() && r.options.scenePath == "s.json" && r.options.motionPath == "m.json" && r.options.steps && *r.options.steps == 50,
              "--scene + --motion + --steps：" + r.error);
    }
    expectError({"--model", "a.xml", "--motion", "m.json"}, "不能与 --model 同用");
    expectError({"--motion", "a.json", "--motion", "b.json"}, "重复的参数 --motion");
    expectError({"--motion="}, "不能为空");
    expectError({"--motion", "m.json", "--steps", "1", "--duration", "1"}, "不能同时使用");
    check(usageText().find("--motion") != std::string::npos, "帮助中有 --motion");
}

// M3-07a：--record / --rerun / --tolerance
void testRecord() {
    {
        const ParseResult r = parse({"--motion", "m.json", "--record", "out/r.jsonl"});
        check(r.ok() && r.options.recordPath == "out/r.jsonl", "--motion + --record：" + r.error);
    }
    {
        const ParseResult r = parse({"--rerun", "r.jsonl"});
        check(r.ok() && r.options.rerunPath == "r.jsonl" && r.options.positionTolerance == 1e-6 && r.options.rotationTolerance == 1e-6,
              "--rerun 单独使用，默认容差 1e-6：" + r.error);
    }
    {
        const ParseResult r = parse({"--rerun", "r.jsonl", "--tolerance", "1e-3,2e-3"});
        check(r.ok() && r.options.positionTolerance == 1e-3 && r.options.rotationTolerance == 2e-3, "--tolerance 位置,姿态：" + r.error);
    }
    expectError({"--rerun", "r.jsonl", "--scene", "s.json"}, "--rerun 不能与");
    expectError({"--rerun", "r.jsonl", "--steps", "10"}, "--rerun 不能与");
    expectError({"--rerun", "r.jsonl", "--tolerance", "1e-3"}, "--tolerance 需要");
    expectError({"--rerun", "r.jsonl", "--tolerance", "0,1"}, "--tolerance 需要");
    expectError({"--scene", "s.json", "--steps", "1", "--tolerance", "1,1"}, "只用于 --rerun");
    expectError({"--model", "a.xml", "--steps", "1", "--record", "r.jsonl"}, "不能与 --model 同用");
    expectError({"--scene", "s.json", "--steps", "1", "--record", "r.jsonl", "--dry-run"}, "不能与 --dry-run 同用");
    expectError({"--rerun", "a", "--rerun", "b"}, "重复的参数 --rerun");
    expectError({"--record="}, "不能为空");
    check(usageText().find("--rerun") != std::string::npos && usageText().find("6 复现超出容差") != std::string::npos,
          "帮助中有 --rerun 与退出码 6");
    // M3-07b：--library / --list-records
    {
        const ParseResult r = parse({"--list-records", "--library", "lib.sqlite3", "--scene", "s.json"});
        check(r.ok() && r.options.listRecords && r.options.libraryPath == "lib.sqlite3" && r.options.scenePath == "s.json",
              "--list-records + --library + --scene（筛选）：" + r.error);
    }
    {
        const ParseResult r = parse({"--scene", "s.json", "--steps", "1", "--record", "r.jsonl", "--library", "lib.sqlite3"});
        check(r.ok() && r.options.libraryPath == "lib.sqlite3", "--record + --library：" + r.error);
    }
    expectError({"--list-records", "--steps", "1"}, "--list-records 只能与");
    expectError({"--scene", "s.json", "--steps", "1", "--library", "lib.sqlite3"}, "--library 只用于");
    expectError({"--list-records", "--list-records"}, "重复的参数 --list-records");
}

void testUsageErrors() {
    expectError({}, "缺少参数");
    expectError({"--bogus"}, "未知参数");
    expectError({"positional"}, "未知参数");
    expectError({"-s", "10"}, "未知参数");
    expectError({"--steps", "10"}, "缺少 --model");
    expectError({"--model", "a.xml"}, "需要 --steps 或 --duration");
    expectError({"--model", "a.xml", "--steps", "10", "--duration", "1"}, "不能同时使用");

    // 缺值
    expectError({"--model"}, "缺少值");
    expectError({"--model", "--steps", "10"}, "缺少值");
    expectError({"--model", "a.xml", "--steps"}, "缺少值");
    expectError({"--model="}, "不能为空");

    // 重复
    expectError({"--model", "a", "--model", "b", "--steps", "1"}, "重复");
    expectError({"--model", "a", "--steps", "1", "--steps", "2"}, "重复");
    expectError({"--model", "a", "--duration", "1", "--duration", "2"}, "重复");
    expectError({"--model", "a", "--steps", "1", "--sample-every", "1", "--sample-every", "2"}, "重复");
    expectError({"--model", "a", "--steps", "1", "--dry-run", "--dry-run"}, "重复");

    // 非法步数
    for (const char* bad : {"0", "-5", "+5", "1.5", "10x", " 10", "", "1e3", "18446744073709551616"}) {
        expectError({"--model", "a", "--steps", bad}, "--steps 需要正整数");
    }
    // 非法时长
    for (const char* bad : {"0", "-1", "+1", "nan", "inf", "1e400", "1e-400", "abc", "1.0s", " 1", "",
                            "0x10", "."}) {
        expectError({"--model", "a", "--duration", bad}, "--duration 需要正的有限秒数");
    }
    // 非法采样间隔
    for (const char* bad : {"0", "-1", "2.0"}) {
        expectError({"--model", "a", "--steps", "1", "--sample-every", bad}, "--sample-every 需要正整数");
    }
}

void testJson() {
    check(jsonEscape("plain") == "plain", "普通字符串");
    check(jsonEscape("a\"b\\c") == "a\\\"b\\\\c", "引号与反斜杠");
    check(jsonEscape("x\ny\tz") == "x\\ny\\tz", "换行与制表符");
    check(jsonEscape(std::string{"\x01", 1}) == "\\u0001", "控制字符");
    check(jsonEscape("模型") == "模型", "UTF-8 原样保留");

    RunOptions options;
    options.modelPath = "m.xml";
    options.steps = 100;
    options.sampleEvery = 5;
    options.dryRun = true;
    const ResolvedRun resolved{"/abs/m.xml", 0.001, 100};
    const std::string json = configToJson(options, resolved);
    check(json ==
              "{\"type\":\"config\",\"model\":\"m.xml\",\"model_resolved\":\"/abs/m.xml\","
              "\"steps\":100,\"timestep\":0.001,\"total_steps\":100,\"sample_every\":5,\"dry_run\":true}",
          "config JSON（steps）: " + json);

    RunOptions byDuration;
    byDuration.modelPath = "m.xml";
    byDuration.durationSeconds = 0.1;
    const std::string json2 = configToJson(byDuration, ResolvedRun{"m.xml", 0.001, 100});
    check(json2.find("\"duration\":0.10000000000000001") != std::string::npos,
          "duration 以 17 位有效数字输出，可无损往返: " + json2);
    check(json2.find("\"steps\"") == std::string::npos, "未指定 steps 时不输出");
}

void testRecords() {
    terra::data::ModelDescription model;
    model.timestep = 0.002;
    model.settings.gravity = {0, 0, -9.5};
    model.settings.integrator = "Euler";
    model.settings.solver = "Newton";
    model.settings.iterations = 50;
    model.settings.tolerance = 1e-8;
    model.entities.push_back({terra::data::EntityId{1}, "base", terra::data::EntityId{}, terra::data::ModelInstanceId{1}, 1.0});
    model.entities.push_back({terra::data::EntityId{2}, "arm\"x", terra::data::EntityId{1}, terra::data::ModelInstanceId{1}, 0.5});
    terra::data::JointDescription joint;
    joint.id = terra::data::JointId{1};
    joint.name = "shoulder";
    joint.type = terra::data::JointType::Hinge;
    joint.entity = terra::data::EntityId{2};
    joint.configurationSize = 1;
    joint.velocitySize = 1;
    model.joints.push_back(joint);
    terra::data::ActuatorDescription motor;
    motor.id = terra::data::ActuatorId{1};
    motor.name = "drive";
    motor.transmission = terra::data::ActuatorTransmission::Joint;
    motor.joint = terra::data::JointId{1};
    motor.inputs.push_back({"", true, -1.5, 2});
    motor.inputs.push_back({"qw", false, 0, 0});
    model.actuators.push_back(motor);
    terra::data::ActuatorDescription tendon;
    tendon.id = terra::data::ActuatorId{2};
    model.actuators.push_back(tendon);
    const std::string m = modelToJson(model);
    check(m ==
              "{\"type\":\"model\",\"contract_revision\":1,\"conventions\":{\"units\":\"SI\",\"up_axis\":\"+z\","
              "\"quaternion\":\"wxyz\",\"entity_velocity\":\"entity_origin_world\","
              "\"free_joint_angular_velocity\":\"body_local\",\"ball_joint_angular_velocity\":\"child_local\","
              "\"contact_entity_0\":\"world_static\"},\"timestep\":0.002,\"gravity\":[0,0,-9.5],\"integrator\":\"Euler\","
              "\"solver\":\"Newton\",\"iterations\":50,\"tolerance\":1e-08,\"entities\":["
              "{\"id\":1,\"name\":\"base\",\"parent\":0},{\"id\":2,\"name\":\"arm\\\"x\",\"parent\":1}],"
              "\"joints\":[{\"id\":1,\"name\":\"shoulder\",\"type\":\"hinge\",\"entity\":2,\"nq\":1,\"nv\":1}],"
              "\"actuators\":[{\"id\":1,\"name\":\"drive\",\"transmission\":\"joint\",\"joint\":1,\"inputs\":["
              "{\"name\":\"\",\"limited\":true,\"range\":[-1.5,2]},{\"name\":\"qw\",\"limited\":false}]},"
              "{\"id\":2,\"name\":\"\",\"transmission\":\"other\",\"joint\":0,\"inputs\":[]}]}",
          "model 记录: " + m);

    terra::data::PhysicsSnapshot snap;
    snap.runEpoch = 3;
    snap.stepIndex = 42;
    snap.simulationTime = 0.25;
    terra::data::EntityState e;
    e.id = terra::data::EntityId{1};
    e.pose.position = {1, 2, 0.5};
    e.pose.orientation = {0.5, 0.5, 0.5, 0.5};
    e.linearVelocity = {0, 0, -1.5};
    e.angularVelocity = {0.1, 0, 0};
    snap.entities.push_back(e);
    terra::data::JointState hinge;
    hinge.id = terra::data::JointId{1};
    hinge.positionSize = 1;
    hinge.velocitySize = 1;
    hinge.position[0] = 0.75;
    hinge.velocity[0] = -2;
    snap.joints.push_back(hinge);
    snap.contacts.candidateCount = 5;
    snap.contacts.activeCount = 4;
    snap.contacts.pairs.push_back({terra::data::EntityId{}, terra::data::EntityId{1}, 4, 9.75});
    snap.diagnostics.finite = true;
    snap.diagnostics.solverIterations = 7;
    const std::string st = stateToJson(snap);
    check(st ==
              "{\"type\":\"state\",\"run_epoch\":3,\"step\":42,\"time\":0.25,\"entities\":["
              "{\"id\":1,\"position\":[1,2,0.5],\"orientation\":[0.5,0.5,0.5,0.5],"
              "\"linear_velocity\":[0,0,-1.5],\"angular_velocity\":[0.10000000000000001,0,0]}],"
              "\"joints\":[{\"id\":1,\"position\":[0.75],\"velocity\":[-2]}],"
              "\"contacts\":{\"candidates\":5,\"active\":4,\"pairs\":[{\"entities\":[0,1],\"points\":4,"
              "\"normal_force\":9.75}]},\"diagnostics\":{\"finite\":true,\"solver_iterations\":7}}",
          "state 记录: " + st);
    const std::string empty = stateToJson(terra::data::PhysicsSnapshot{});
    check(empty ==
              "{\"type\":\"state\",\"run_epoch\":0,\"step\":0,\"time\":0,\"entities\":[],\"joints\":[],"
              "\"contacts\":{\"candidates\":0,\"active\":0,\"pairs\":[]},"
              "\"diagnostics\":{\"finite\":true,\"solver_iterations\":0}}",
          "空快照的 state 记录: " + empty);

    check(summaryToJson(1, 5000, 5) ==
              "{\"type\":\"summary\",\"run_epoch\":1,\"steps\":5000,\"time\":5,\"status\":\"ok\"}",
          "summary 记录");
    check(errorToJson("non_finite", 0, 5, 5, "qpos[2] \"bad\"") ==
              "{\"type\":\"error\",\"kind\":\"non_finite\",\"run_epoch\":0,\"step\":5,\"time\":5,"
              "\"message\":\"qpos[2] \\\"bad\\\"\"}",
          "error 记录（消息转义）");
}

void testStepsForDuration() {
    auto steps = [](double duration, double dt) { return stepsForDuration(duration, dt); };
    check(steps(5.0, 0.001) == 5000u, "5 s / 1 ms = 5000（浮点误差不应得到 5001）");
    check(steps(0.3, 0.1) == 3u, "0.3 / 0.1 = 3（0.3/0.1 的浮点结果为 2.9999999999999996）");
    check(steps(1.0, 0.003) == 334u, "不能整除时向上取整：1 / 0.003 -> 334");
    check(steps(0.0001, 0.001) == 1u, "时长短于一步时至少运行 1 步");
    check(steps(60.0, 0.002) == 30000u, "60 s / 2 ms");
    check(!steps(0.0, 0.001), "时长为 0");
    check(!steps(1.0, 0.0), "步长为 0");
    check(!steps(1.0, -0.001), "步长为负");
    check(!steps(1e300, 1e-300), "溢出");
    check(!steps(1.0, std::numeric_limits<double>::quiet_NaN()), "步长为 NaN");
}

}  // namespace

int main() {
    testHelpAndVersion();
    testValidRuns();
    testUsageErrors();
    testScene();
    testMotion();
    testRecord();
    testJson();
    testStepsForDuration();
    testRecords();
    if (failures == 0) {
        std::printf("all passed\n");
    }
    return failures == 0 ? 0 : 1;
}
