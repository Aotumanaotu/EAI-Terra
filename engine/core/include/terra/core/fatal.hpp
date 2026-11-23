#pragma once

// 不可恢复错误的报告与用户数据保护（M2-02，M2-01 设计第 8 节；用户 2026-11-23：有限保证）。
//
// 物理后端（MuJoCo）运行期的致命错误不能恢复：其处理函数不得返回。Terra 的保证只有两条：
//   1. 写出诊断（stderr 与崩溃报告文件），说明错误、来源与上下文；
//   2. 退出前调用已登记的“紧急保存”，使用户数据（设置、布局等）不丢失。
// 随后以 kFatalExitCode 结束进程。加载期的错误不走这里，由各加载接口作为普通错误返回。
//
// 紧急保存函数的要求：不调用物理后端与图形 API，只写文件，尽快返回（建议 1 秒内）；抛出的异常会被捕获并记录。
// 线程：登记与注销可在任意线程调用；fatalExit 在出错的线程上运行，第二次进入（例如紧急保存中又出错）时立即退出。

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace terra::core {

// 致命错误的退出码（BSD sysexits 的 EX_SOFTWARE）。Headless 的其他退出码见其 --help。
constexpr int kFatalExitCode = 70;

// 返回空表示成功，否则为失败原因。
using EmergencySave = std::function<std::optional<std::string>()>;

// 登记紧急保存，返回句柄（> 0）。按登记顺序调用。
int registerEmergencySave(std::string name, EmergencySave save);
void unregisterEmergencySave(int handle);

// 作用域内登记，析构时注销。
class ScopedEmergencySave {
public:
    ScopedEmergencySave(std::string name, EmergencySave save);
    ~ScopedEmergencySave();
    ScopedEmergencySave(const ScopedEmergencySave&) = delete;
    ScopedEmergencySave& operator=(const ScopedEmergencySave&) = delete;

private:
    int handle_ = 0;
};

// 进程级诊断上下文（如“设置文件”“场景文件”），写入崩溃报告。value 为空时删除该项。
void setFatalContext(const std::string& key, std::string value);
std::vector<std::pair<std::string, std::string>> fatalContext();

struct EmergencyResult {
    std::string name;
    bool ok = false;
    std::string error;
    double seconds = 0.0;
};

// 依次调用全部紧急保存（每个都捕获异常），返回结果。fatalExit 内部使用；也供测试直接调用。
std::vector<EmergencyResult> runEmergencySaves();

struct FatalReport {
    std::string source;   // 如 "MuJoCo"
    std::string message;  // 错误原文
    std::vector<std::pair<std::string, std::string>> details;  // 出错时的状态（世界、步序号、仿真时间等）
};

// 报告正文（纯函数）：时间、来源、错误、细节、上下文。
std::string formatFatalReport(const FatalReport& report, const std::vector<std::pair<std::string, std::string>>& context,
                              const std::string& timestamp);

// 紧急保存结果的文字（纯函数），附在报告之后。
std::string formatEmergencyResults(const std::vector<EmergencyResult>& results);

// 崩溃报告目录：$TERRA_CRASH_DIR；否则 $XDG_STATE_HOME/terra；否则 $HOME/.local/state/terra；都没有时为空（只写 stderr）。
// 环境变量查询可注入，便于测试。
std::string crashDirectory(const std::function<const char*(const char*)>& lookup);
std::string crashDirectory();

// 报告文件名：crash-<YYYYmmdd-HHMMSS>-<进程号>.txt
std::string crashFileName(const std::string& compactTime, std::uint64_t pid);

// 写 stderr 与崩溃报告文件，调用紧急保存，再以 kFatalExitCode 退出（会执行 atexit 处理）。
[[noreturn]] void fatalExit(const FatalReport& report);

}  // namespace terra::core
