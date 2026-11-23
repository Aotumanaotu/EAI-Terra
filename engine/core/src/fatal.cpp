#include "terra/core/fatal.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>

#include <unistd.h>

namespace terra::core {

namespace {

struct Registry {
    std::mutex mutex;
    int nextHandle = 1;
    std::vector<std::pair<int, std::pair<std::string, EmergencySave>>> saves;
    std::map<std::string, std::string> context;
};

Registry& registry() {
    static Registry* r = new Registry;  // 故意不释放：fatalExit 可能在静态析构期间之后仍被调用
    return *r;
}

std::string localTime(const char* format) {
    const std::time_t now = std::time(nullptr);
    std::tm tm{};
    localtime_r(&now, &tm);
    char buf[64];
    std::strftime(buf, sizeof buf, format, &tm);
    return buf;
}

}  // namespace

int registerEmergencySave(std::string name, EmergencySave save) {
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    const int handle = r.nextHandle++;
    r.saves.push_back({handle, {std::move(name), std::move(save)}});
    return handle;
}

void unregisterEmergencySave(int handle) {
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    for (auto it = r.saves.begin(); it != r.saves.end(); ++it) {
        if (it->first == handle) {
            r.saves.erase(it);
            return;
        }
    }
}

ScopedEmergencySave::ScopedEmergencySave(std::string name, EmergencySave save)
    : handle_(registerEmergencySave(std::move(name), std::move(save))) {}

ScopedEmergencySave::~ScopedEmergencySave() { unregisterEmergencySave(handle_); }

void setFatalContext(const std::string& key, std::string value) {
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    if (value.empty()) {
        r.context.erase(key);
    } else {
        r.context[key] = std::move(value);
    }
}

std::vector<std::pair<std::string, std::string>> fatalContext() {
    Registry& r = registry();
    std::lock_guard<std::mutex> lock(r.mutex);
    return {r.context.begin(), r.context.end()};
}

std::vector<EmergencyResult> runEmergencySaves() {
    std::vector<std::pair<std::string, EmergencySave>> saves;
    {
        Registry& r = registry();
        std::lock_guard<std::mutex> lock(r.mutex);
        for (const auto& entry : r.saves) saves.push_back(entry.second);
    }
    std::vector<EmergencyResult> results;
    for (const auto& [name, save] : saves) {
        EmergencyResult result;
        result.name = name;
        const auto start = std::chrono::steady_clock::now();
        try {
            const std::optional<std::string> error = save ? save() : std::optional<std::string>{"未提供保存函数"};
            result.ok = !error.has_value();
            if (error) result.error = *error;
        } catch (const std::exception& e) {
            result.error = std::string{"异常："} + e.what();
        } catch (...) {
            result.error = "未知异常";
        }
        result.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        results.push_back(std::move(result));
    }
    return results;
}

std::string formatFatalReport(const FatalReport& report, const std::vector<std::pair<std::string, std::string>>& context,
                              const std::string& timestamp) {
    std::string text = "EAI Terra 致命错误报告\n";
    text += "时间：" + timestamp + "\n";
    text += "来源：" + (report.source.empty() ? std::string{"未知"} : report.source) + "\n";
    text += "错误：" + report.message + "\n";
    if (!report.details.empty()) {
        text += "出错时的状态：\n";
        for (const auto& [key, value] : report.details) text += "  " + key + "：" + value + "\n";
    }
    if (!context.empty()) {
        text += "上下文：\n";
        for (const auto& [key, value] : context) text += "  " + key + "：" + value + "\n";
    }
    text += "进程将以退出码 " + std::to_string(kFatalExitCode) + " 结束。\n";
    return text;
}

std::string formatEmergencyResults(const std::vector<EmergencyResult>& results) {
    if (results.empty()) return "紧急保存：没有登记的项目\n";
    std::string text = "紧急保存：\n";
    for (const auto& r : results) {
        char seconds[32];
        std::snprintf(seconds, sizeof seconds, "%.3f", r.seconds);
        text += "  " + r.name + "：" + (r.ok ? std::string{"成功"} : "失败（" + r.error + "）") + "，用时 " + seconds +
                " 秒\n";
    }
    return text;
}

std::string crashDirectory(const std::function<const char*(const char*)>& lookup) {
    const auto value = [&](const char* name) -> std::string {
        const char* v = lookup ? lookup(name) : nullptr;
        return v != nullptr ? std::string{v} : std::string{};
    };
    if (std::string dir = value("TERRA_CRASH_DIR"); !dir.empty()) return dir;
    if (std::string state = value("XDG_STATE_HOME"); !state.empty()) return state + "/terra";
    if (std::string home = value("HOME"); !home.empty()) return home + "/.local/state/terra";
    return {};
}

std::string crashDirectory() {
    return crashDirectory([](const char* name) -> const char* { return std::getenv(name); });
}

std::string crashFileName(const std::string& compactTime, std::uint64_t pid) {
    return "crash-" + compactTime + "-" + std::to_string(pid) + ".txt";
}

void fatalExit(const FatalReport& report) {
    static std::atomic<bool> entered{false};
    if (entered.exchange(true)) {
        // 处理过程中再次出错（例如紧急保存触发了同一错误）：不再重复，直接退出
        std::_Exit(kFatalExitCode);
    }
    const std::string text = formatFatalReport(report, fatalContext(), localTime("%Y-%m-%d %H:%M:%S"));
    std::fputs(text.c_str(), stderr);
    std::fflush(stderr);

    // 先写报告，再做紧急保存：即使某个保存函数卡住，诊断也已落盘
    std::string reportPath;
    if (const std::string dir = crashDirectory(); !dir.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        reportPath = dir + "/" + crashFileName(localTime("%Y%m%d-%H%M%S"), static_cast<std::uint64_t>(::getpid()));
        std::ofstream out(reportPath, std::ios::binary | std::ios::trunc);
        if (out) {
            out << text;
            out.flush();
        } else {
            reportPath.clear();
        }
    }

    const std::string saves = formatEmergencyResults(runEmergencySaves());
    std::fputs(saves.c_str(), stderr);
    if (!reportPath.empty()) {
        std::ofstream out(reportPath, std::ios::binary | std::ios::app);
        out << saves;
        std::fprintf(stderr, "崩溃报告：%s\n", reportPath.c_str());
    } else {
        std::fputs("崩溃报告：未写入文件（没有可用的目录）\n", stderr);
    }
    std::fflush(stderr);
    std::exit(kFatalExitCode);
}

}  // namespace terra::core
