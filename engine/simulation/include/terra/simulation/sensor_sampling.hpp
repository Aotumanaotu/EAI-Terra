#pragma once

// 传感器采样时刻（M4-02，M4-01 设计稿第 8 节问题 1：按时间累计）。
//
// 物理步长 Δt、传感器频率 f：第 k 步（k ≥ 1）满足 ⌊k·Δt·f⌋ > ⌊(k−1)·Δt·f⌋ 时采样；第 0 步（加载或重置后的初始状态）
// 总是采样。这样平均频率正好是 f（500 Hz 物理、30 Hz 相机时，间隔在 16 与 17 步之间交替），与真机的标称频率一致；
// 只由步序号决定，可复现。f 不低于物理频率时每步采样。为避免 0.002 × 500 这类乘积的舍入误差，取整前加 1e-9。

#include <cstdint>

namespace terra::simulation {

bool sensorSampleDue(std::uint64_t step, double timestep, double rateHz) noexcept;

}  // namespace terra::simulation
