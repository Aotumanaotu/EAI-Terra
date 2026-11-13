// terra_headless 参数解析与 JSON 输出的单元测试（M0-03）。

#include <cstdio>
#include <initializer_list>
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
    const std::string json = configToJson(options, "/abs/m.xml");
    check(json ==
              "{\"type\":\"config\",\"model\":\"m.xml\",\"model_resolved\":\"/abs/m.xml\","
              "\"steps\":100,\"sample_every\":5,\"dry_run\":true}",
          "config JSON（steps）: " + json);

    RunOptions byDuration;
    byDuration.modelPath = "m.xml";
    byDuration.durationSeconds = 0.1;
    const std::string json2 = configToJson(byDuration, "m.xml");
    check(json2.find("\"duration\":0.10000000000000001") != std::string::npos,
          "duration 以 17 位有效数字输出，可无损往返: " + json2);
    check(json2.find("\"steps\"") == std::string::npos, "未指定 steps 时不输出");
}

}  // namespace

int main() {
    testHelpAndVersion();
    testValidRuns();
    testUsageErrors();
    testJson();
    if (failures == 0) {
        std::printf("all passed\n");
    }
    return failures == 0 ? 0 : 1;
}
