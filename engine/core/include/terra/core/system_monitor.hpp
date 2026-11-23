#pragma once

// 进程与系统资源占用（M1-15，性能图表）。Linux 上读取 /proc；其他平台各项为空（显示“不可用”，而不是 0）。
//
// CPU 占用率 = 两次采样之间的增量之比：
//   整机     (Δ总时间 − Δ空闲时间) / Δ总时间          （/proc/stat 第一行 cpu，空闲 = idle + iowait）
//   本进程   Δ(utime + stime) / Δ总时间              （/proc/self/stat 第 14、15 字段；占整机的百分比，满载 = 100%）
// 内存：本进程常驻内存（/proc/self/statm 第 2 字段 × 页大小）。解析函数是纯函数，可单元测试。

#include <cstdint>
#include <optional>
#include <string_view>

namespace terra::core {

struct CpuTimes {
    std::uint64_t total = 0;  // 所有 CPU 的累计时钟滴答
    std::uint64_t idle = 0;   // 其中空闲（idle + iowait）
};

// /proc/stat 的文本 → 第一行 "cpu ..." 的汇总；格式不符时为空。
std::optional<CpuTimes> parseProcStat(std::string_view text);
// /proc/self/stat 的文本 → utime + stime（时钟滴答）。进程名可能含空格与括号，按最后一个 ')' 之后计字段。
std::optional<std::uint64_t> parseProcessCpuTicks(std::string_view text);
// /proc/self/statm 的文本 → 常驻页数。
std::optional<std::uint64_t> parseResidentPages(std::string_view text);
// 两次采样之间的占用率（0–100）；增量为 0 或计数回退时为空。
std::optional<double> systemCpuPercent(const CpuTimes& before, const CpuTimes& after);
std::optional<double> processCpuPercent(std::uint64_t ticksBefore, std::uint64_t ticksAfter, const CpuTimes& before,
                                        const CpuTimes& after);

class SystemMonitor {
public:
    struct Sample {
        std::optional<double> processCpuPercent;  // 第一次采样时为空（需要两次采样的增量）
        std::optional<double> systemCpuPercent;
        std::optional<double> residentMb;
        int logicalCores = 0;
    };

    // 采样一次；建议间隔不小于 0.2 秒（/proc 的计数精度为 10 ms）。
    Sample sample();

private:
    std::optional<CpuTimes> lastSystem_;
    std::optional<std::uint64_t> lastProcess_;
};

}  // namespace terra::core
