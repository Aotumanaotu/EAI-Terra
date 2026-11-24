#include "terra/simulation/sensor_sampling.hpp"

#include <cmath>

namespace terra::simulation {

bool sensorSampleDue(std::uint64_t step, double timestep, double rateHz) noexcept {
    if (step == 0) return true;
    if (!(timestep > 0) || !(rateHz > 0) || rateHz * timestep >= 1.0 - 1e-9) return true;  // 不低于物理频率（含 (1/Δt)·Δt 的舍入）
    const double perStep = timestep * rateHz;
    const auto count = [perStep](std::uint64_t k) { return std::floor(static_cast<double>(k) * perStep + 1e-9); };
    return count(step) > count(step - 1);
}

}  // namespace terra::simulation
