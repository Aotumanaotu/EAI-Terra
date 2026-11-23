// 性能监视数据层测试（M1-15）。不需要图形环境，渲染关闭时也运行。
// 覆盖：滚动时间序列（容量、顺序、统计）；/proc/stat、/proc/self/stat（进程名含空格与括号）、/proc/self/statm 的解析；
// 占用率公式与异常增量；Linux 上实际采样（第一次没有占用率，第二次有，内存为正）。

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>

#include "terra/core/system_monitor.hpp"
#include "terra/core/time_series.hpp"

namespace {

namespace c = terra::core;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) <= tol; }

void testTimeSeries() {
    c::TimeSeries s(3);
    check(s.empty() && s.capacity() == 3 && s.latest() == 0.0, "新序列为空");
    for (const double v : {1.0, 2.0, 3.0, 4.0}) s.push(v);
    check(s.size() == 3 && s.at(0) == 2.0 && s.at(1) == 3.0 && s.at(2) == 4.0 && s.latest() == 4.0, "超出容量时丢弃最旧的值，按从旧到新访问");
    check(s.minimum() == 2.0 && s.maximum() == 4.0 && near(s.average(), 3.0), "最小、最大、平均值");
    check(s.at(7) == 0.0, "越界访问返回 0");
    s.clear();
    check(s.empty() && s.size() == 0, "清空");
    c::TimeSeries one(0);
    one.push(5.0);
    check(one.capacity() == 1 && one.latest() == 5.0, "容量至少为 1");
}

void testParsing() {
    const auto t = c::parseProcStat("cpu  10 20 30 400 50 6 7 8 0 0\ncpu0 1 2 3 4 5 6 7 8 0 0\n");
    check(t && t->total == 531 && t->idle == 450, "/proc/stat：总时间为前 8 项之和，空闲 = idle + iowait");
    check(!c::parseProcStat("cpux 1 2 3 4") && !c::parseProcStat("cpu 1 2"), "/proc/stat 格式不符：为空");
    const auto ticks = c::parseProcessCpuTicks(
        "1234 (my (weird) prog) S 1 1234 1234 0 -1 4194560 100 0 0 0 250 75 0 0 20 0 1 0 5000 1000000 300 18446744073709551615\n");
    check(ticks && *ticks == 325, "/proc/self/stat：进程名含空格与括号时，utime + stime = 250 + 75");
    check(!c::parseProcessCpuTicks("1234 (short) S 1 2"), "/proc/self/stat 字段不足：为空");
    const auto pages = c::parseResidentPages("12345 678 90 1 0 200 0\n");
    check(pages && *pages == 678, "/proc/self/statm：第 2 字段为常驻页数");
    check(!c::parseResidentPages(""), "/proc/self/statm 为空：为空");
}

void testPercent() {
    const c::CpuTimes a{1000, 800};
    const c::CpuTimes b{2000, 1300};
    const auto sys = c::systemCpuPercent(a, b);
    check(sys && near(*sys, 50.0), "整机：(Δ总 − Δ空闲) / Δ总 = 50%");
    const auto proc = c::processCpuPercent(100, 350, a, b);
    check(proc && near(*proc, 25.0), "本进程：Δ进程 / Δ总 = 25%（占整机）");
    check(!c::systemCpuPercent(b, a) && !c::processCpuPercent(350, 100, a, b) && !c::systemCpuPercent(a, a),
          "计数回退或没有增量：为空（不显示为 0）");
}

void testLiveSample() {
#if defined(__linux__)
    c::SystemMonitor monitor;
    const auto first = monitor.sample();
    check(!first.processCpuPercent && !first.systemCpuPercent, "第一次采样没有占用率（需要增量）");
    check(first.residentMb && *first.residentMb > 0.0 && first.logicalCores > 0, "常驻内存为正，逻辑核数为正");
    // 忙等约 0.3 秒，让计数产生增量
    const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(300);
    volatile double sink = 0.0;
    while (std::chrono::steady_clock::now() < until) sink = sink + 1.0;
    const auto second = monitor.sample();
    check(second.processCpuPercent && *second.processCpuPercent >= 0.0 && *second.processCpuPercent <= 100.0 &&
              second.systemCpuPercent && *second.systemCpuPercent >= 0.0 && *second.systemCpuPercent <= 100.0,
          "第二次采样：本进程与整机占用率都在 0–100%");
    std::printf("INFO 实测：本进程 %.1f%%，整机 %.1f%%，常驻内存 %.1f MB，逻辑核 %d\n", second.processCpuPercent.value_or(-1.0),
                second.systemCpuPercent.value_or(-1.0), second.residentMb.value_or(-1.0), second.logicalCores);
#endif
}

}  // namespace

int main() {
    testTimeSeries();
    testParsing();
    testPercent();
    testLiveSample();
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
