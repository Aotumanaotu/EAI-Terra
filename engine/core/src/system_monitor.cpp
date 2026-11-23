#include "terra/core/system_monitor.hpp"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>

#if defined(__linux__)
#include <unistd.h>
#endif

namespace terra::core {

namespace {

// 从 text 的 pos 处跳过空白后读取一个无符号整数；失败返回空并保持 pos。
std::optional<std::uint64_t> readNumber(std::string_view text, std::size_t& pos) {
    while (pos < text.size() && (text[pos] == ' ' || text[pos] == '\t')) ++pos;
    const std::size_t begin = pos;
    std::uint64_t value = 0;
    while (pos < text.size() && text[pos] >= '0' && text[pos] <= '9') {
        value = value * 10u + static_cast<std::uint64_t>(text[pos] - '0');
        ++pos;
    }
    if (pos == begin) return std::nullopt;
    return value;
}

std::string readFile(const char* path) {
    std::ifstream in(path);
    std::ostringstream s;
    s << in.rdbuf();
    return s.str();
}

}  // namespace

std::optional<CpuTimes> parseProcStat(std::string_view text) {
    if (text.substr(0, 4) != "cpu ") return std::nullopt;
    std::size_t pos = 4;
    std::uint64_t fields[8] = {};
    int count = 0;
    for (; count < 8; ++count) {
        const auto v = readNumber(text, pos);
        if (!v) break;
        fields[count] = *v;
    }
    if (count < 4) return std::nullopt;  // 至少 user nice system idle
    CpuTimes t;
    for (int i = 0; i < count; ++i) t.total += fields[i];
    t.idle = fields[3] + (count > 4 ? fields[4] : 0u);
    return t;
}

std::optional<std::uint64_t> parseProcessCpuTicks(std::string_view text) {
    const std::size_t close = text.rfind(')');
    if (close == std::string_view::npos) return std::nullopt;
    // ')' 之后依次为字段 3（state）、4、5 …；utime、stime 为字段 14、15
    std::size_t pos = close + 1;
    int field = 2;
    std::optional<std::uint64_t> utime;
    while (pos < text.size()) {
        while (pos < text.size() && text[pos] == ' ') ++pos;
        if (pos >= text.size()) break;
        ++field;
        const std::size_t begin = pos;
        while (pos < text.size() && text[pos] != ' ' && text[pos] != '\n') ++pos;
        if (field == 14 || field == 15) {
            std::size_t p = begin;
            const auto v = readNumber(text.substr(0, pos), p);
            if (!v) return std::nullopt;
            if (field == 14) {
                utime = v;
            } else {
                return *utime + *v;
            }
        }
        if (pos < text.size() && text[pos] == '\n') break;
    }
    return std::nullopt;
}

std::optional<std::uint64_t> parseResidentPages(std::string_view text) {
    std::size_t pos = 0;
    if (!readNumber(text, pos)) return std::nullopt;  // 第 1 字段：总页数
    return readNumber(text, pos);
}

std::optional<double> systemCpuPercent(const CpuTimes& before, const CpuTimes& after) {
    if (after.total <= before.total || after.idle < before.idle) return std::nullopt;
    const double total = static_cast<double>(after.total - before.total);
    const double idle = static_cast<double>(after.idle - before.idle);
    return std::max(0.0, std::min(100.0, (total - idle) / total * 100.0));
}

std::optional<double> processCpuPercent(std::uint64_t ticksBefore, std::uint64_t ticksAfter, const CpuTimes& before,
                                        const CpuTimes& after) {
    if (after.total <= before.total || ticksAfter < ticksBefore) return std::nullopt;
    const double total = static_cast<double>(after.total - before.total);
    return std::max(0.0, std::min(100.0, static_cast<double>(ticksAfter - ticksBefore) / total * 100.0));
}

SystemMonitor::Sample SystemMonitor::sample() {
    Sample s;
    s.logicalCores = static_cast<int>(std::thread::hardware_concurrency());
#if defined(__linux__)
    const auto system = parseProcStat(readFile("/proc/stat"));
    const auto process = parseProcessCpuTicks(readFile("/proc/self/stat"));
    if (system && lastSystem_) {
        s.systemCpuPercent = systemCpuPercent(*lastSystem_, *system);
        if (process && lastProcess_) s.processCpuPercent = processCpuPercent(*lastProcess_, *process, *lastSystem_, *system);
    }
    lastSystem_ = system;
    lastProcess_ = process;
    if (const auto pages = parseResidentPages(readFile("/proc/self/statm"))) {
        const long pageSize = sysconf(_SC_PAGESIZE);
        if (pageSize > 0) s.residentMb = static_cast<double>(*pages) * static_cast<double>(pageSize) / (1024.0 * 1024.0);
    }
#endif
    return s;
}

}  // namespace terra::core
