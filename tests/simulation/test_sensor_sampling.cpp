// 传感器采样时刻（M4-02，按时间累计）：纯逻辑。
//   - 第 0 步总是采样；
//   - 物理 500 Hz：30 Hz 在 1 s（第 1–500 步）内正好 30 次，间隔只有 16 与 17 步；10 Hz 每 50 步；250 Hz 每 2 步；
//   - 频率等于或高于物理频率（含 (1/Δt)·Δt 的舍入）时每步采样；步长 0.001 时 1000 Hz 也每步采样；
//   - 10 s 内 30 Hz 正好 300 次（长时间不漂移）；参数无效时每步采样（不丢数据）。

#include <cstdio>
#include <set>
#include <string>

#include "terra/simulation/sensor_sampling.hpp"

namespace {

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

using terra::simulation::sensorSampleDue;

int count(double dt, double rate, std::uint64_t from, std::uint64_t to, std::set<std::uint64_t>* gaps = nullptr) {
    int n = 0;
    std::uint64_t last = 0;
    for (std::uint64_t k = from; k <= to; ++k) {
        if (!sensorSampleDue(k, dt, rate)) continue;
        if (gaps != nullptr && n > 0) gaps->insert(k - last);
        last = k;
        ++n;
    }
    return n;
}

}  // namespace

int main() {
    check(sensorSampleDue(0, 0.002, 30.0) && sensorSampleDue(0, 0.002, 0.5), "第 0 步总是采样");
    std::set<std::uint64_t> gaps;
    const int n30 = count(0.002, 30.0, 1, 500, &gaps);
    check(n30 == 30, "500 Hz 物理、30 Hz：1 s 内 30 次，实际 " + std::to_string(n30));
    check(gaps == std::set<std::uint64_t>{16, 17}, "30 Hz 的间隔只有 16 与 17 步");
    check(count(0.002, 30.0, 1, 5000) == 300, "10 s 内正好 300 次，不漂移");
    gaps.clear();
    check(count(0.002, 10.0, 1, 500, &gaps) == 10 && gaps == std::set<std::uint64_t>{50}, "10 Hz：每 50 步");
    gaps.clear();
    check(count(0.002, 250.0, 1, 500, &gaps) == 250 && gaps == std::set<std::uint64_t>{2}, "250 Hz：每 2 步");
    check(count(0.002, 500.0, 1, 500) == 500, "等于物理频率：每步");
    check(count(0.002, 1.0 / 0.002, 1, 1000) == 1000, "1/Δt 的舍入不丢步");
    check(count(0.001, 1000.0, 1, 1000) == 1000 && count(0.001, 1.0 / 0.001, 1, 1000) == 1000, "步长 0.001、1000 Hz：每步");
    check(count(0.002, 2000.0, 1, 100) == 100, "高于物理频率：每步");
    check(count(0.002, 0.0, 1, 10) == 10 && count(0.0, 30.0, 1, 10) == 10, "参数无效时每步采样");
    check(count(0.002, 30.0, 1, 3000000) == 180000, "100 分钟后仍正好 30 Hz");
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
