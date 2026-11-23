// 致命错误报告与紧急保存（M2-02）。纯逻辑，不退出进程；真正的退出路径由 physics.backend_fatal 子进程用例验证。
// 覆盖：报告正文；崩溃目录的优先顺序；文件名；紧急保存按登记顺序调用、失败与异常被记录且不影响后续项、注销；
// 作用域登记；上下文的设置与删除。

#include <cstdio>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "terra/core/fatal.hpp"

namespace {

namespace core = terra::core;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

bool contains(const std::string& text, const std::string& part) { return text.find(part) != std::string::npos; }

void testReport() {
    core::FatalReport report;
    report.source = "MuJoCo";
    report.message = "mj_stackAlloc: out of memory";
    report.details = {{"世界实例", "3"}, {"步序号（出错前已完成）", "120"}};
    const std::string text = core::formatFatalReport(report, {{"设置文件", "/tmp/s.json"}}, "2026-11-23 10:00:00");
    check(contains(text, "时间：2026-11-23 10:00:00"), "报告含时间");
    check(contains(text, "来源：MuJoCo"), "报告含来源");
    check(contains(text, "错误：mj_stackAlloc: out of memory"), "报告含错误原文");
    check(contains(text, "  世界实例：3\n") && contains(text, "  步序号（出错前已完成）：120\n"), "报告含出错时的状态");
    check(contains(text, "上下文：\n  设置文件：/tmp/s.json\n"), "报告含上下文");
    check(contains(text, "退出码 70"), "报告说明退出码 70\n" + text);
    const std::string bare = core::formatFatalReport(core::FatalReport{}, {}, "t");
    check(contains(bare, "来源：未知") && !contains(bare, "出错时的状态") && !contains(bare, "上下文"), "空报告只有必需行");
    check(core::kFatalExitCode == 70, "致命错误退出码为 70");
}

void testCrashDirectory() {
    std::map<std::string, std::string> env;
    const auto lookup = [&](const char* name) -> const char* {
        const auto it = env.find(name);
        return it == env.end() ? nullptr : it->second.c_str();
    };
    check(core::crashDirectory(lookup).empty(), "没有任何环境变量：目录为空");
    env["HOME"] = "/home/u";
    check(core::crashDirectory(lookup) == "/home/u/.local/state/terra", "只有 HOME：~/.local/state/terra");
    env["XDG_STATE_HOME"] = "/state";
    check(core::crashDirectory(lookup) == "/state/terra", "XDG_STATE_HOME 优先于 HOME");
    env["TERRA_CRASH_DIR"] = "/crash";
    check(core::crashDirectory(lookup) == "/crash", "TERRA_CRASH_DIR 优先于其他");
    env["TERRA_CRASH_DIR"] = "";
    check(core::crashDirectory(lookup) == "/state/terra", "空的 TERRA_CRASH_DIR 视为未设置");
    check(core::crashFileName("20261123-100000", 4242) == "crash-20261123-100000-4242.txt", "报告文件名");
}

void testEmergencySaves() {
    std::vector<std::string> order;
    const int a = core::registerEmergencySave("设置", [&] {
        order.push_back("设置");
        return std::optional<std::string>{};
    });
    const int b = core::registerEmergencySave("布局", [&]() -> std::optional<std::string> {
        order.push_back("布局");
        return std::string{"磁盘已满"};
    });
    const int c = core::registerEmergencySave("场景", [&]() -> std::optional<std::string> {
        order.push_back("场景");
        throw std::runtime_error("写入被拒绝");
    });
    const int d = core::registerEmergencySave("最后", [&] {
        order.push_back("最后");
        return std::optional<std::string>{};
    });
    check(a > 0 && b > a && c > b && d > c, "句柄为正且递增");

    auto results = core::runEmergencySaves();
    check(order == std::vector<std::string>{"设置", "布局", "场景", "最后"}, "按登记顺序全部调用（异常不影响后续项）");
    check(results.size() == 4, "每项一个结果");
    if (results.size() == 4) {
        check(results[0].ok && results[0].error.empty(), "成功项");
        check(!results[1].ok && results[1].error == "磁盘已满", "返回的失败原因被记录");
        check(!results[2].ok && results[2].error == "异常：写入被拒绝", "异常被捕获并记录");
        check(results[3].ok, "异常之后的项仍然成功");
        check(results[0].seconds >= 0.0, "记录用时");
    }
    const std::string text = core::formatEmergencyResults(results);
    check(contains(text, "  设置：成功") && contains(text, "  布局：失败（磁盘已满）"), "紧急保存结果的文字\n" + text);

    core::unregisterEmergencySave(b);
    core::unregisterEmergencySave(c);
    order.clear();
    results = core::runEmergencySaves();
    check(order == std::vector<std::string>{"设置", "最后"} && results.size() == 2, "注销后不再调用");
    core::unregisterEmergencySave(a);
    core::unregisterEmergencySave(d);
    core::unregisterEmergencySave(9999);  // 未知句柄：无事发生
    check(core::runEmergencySaves().empty(), "全部注销后没有项");
    check(core::formatEmergencyResults({}) == "紧急保存：没有登记的项目\n", "没有登记项时的文字");

    {
        const core::ScopedEmergencySave scoped("作用域", [] { return std::optional<std::string>{}; });
        check(core::runEmergencySaves().size() == 1, "作用域内已登记");
    }
    check(core::runEmergencySaves().empty(), "离开作用域后已注销");
}

void testContext() {
    core::setFatalContext("场景文件", "/a/scene.json");
    core::setFatalContext("设置文件", "/a/settings.json");
    auto ctx = core::fatalContext();
    check(ctx.size() == 2, "两项上下文");
    core::setFatalContext("场景文件", "/b/scene.json");
    ctx = core::fatalContext();
    bool updated = false;
    for (const auto& [k, v] : ctx) updated = updated || (k == "场景文件" && v == "/b/scene.json");
    check(ctx.size() == 2 && updated, "同名键覆盖");
    core::setFatalContext("场景文件", "");
    core::setFatalContext("设置文件", "");
    check(core::fatalContext().empty(), "空值删除该项");
}

}  // namespace

int main() {
    testReport();
    testCrashDirectory();
    testEmergencySaves();
    testContext();
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
