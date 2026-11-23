#include "terra/simulation/clock.hpp"

#include <algorithm>
#include <cmath>

namespace terra::simulation {

FixedStepClock::FixedStepClock(double timestep, ClockSettings settings)
    : timestep_(std::isfinite(timestep) && timestep > 0 ? timestep : 0.001), settings_(settings) {
    if (!(std::isfinite(settings_.speed) && settings_.speed > 0)) settings_.speed = 1.0;
    if (!(std::isfinite(settings_.maxCatchUp) && settings_.maxCatchUp > 0)) settings_.maxCatchUp = 0.1;
    maxSteps_ = std::max<std::uint64_t>(1, static_cast<std::uint64_t>(std::floor(settings_.maxCatchUp / timestep_ + 1e-9)));
}

std::uint64_t FixedStepClock::advance(double wallSeconds) {
    lastOverloaded_ = false;
    if (paused_) {
        accumulator_ = 0.0;
        if (singleStep_) {
            singleStep_ = false;
            return 1;
        }
        return 0;
    }
    if (std::isfinite(wallSeconds) && wallSeconds > 0) accumulator_ += wallSeconds * settings_.speed;
    // 加一个极小的余量，避免 0.002 × 5 这类累加因舍入差一点点而少算一步
    auto steps = static_cast<std::uint64_t>(std::floor(accumulator_ / timestep_ + 1e-9));
    accumulator_ = std::max(0.0, accumulator_ - static_cast<double>(steps) * timestep_);
    if (singleStep_) {
        singleStep_ = false;
        steps = std::max<std::uint64_t>(steps, 1);
    }
    if (steps > maxSteps_) {
        lastOverloaded_ = true;
        ++overloads_;
        dropped_ += static_cast<double>(steps - maxSteps_) * timestep_;
        steps = maxSteps_;
    }
    return steps;
}

double FixedStepClock::alpha() const noexcept {
    if (paused_) return 0.0;
    return std::clamp(accumulator_ / timestep_, 0.0, 1.0 - 1e-12);
}

void FixedStepClock::pause() noexcept {
    paused_ = true;
    accumulator_ = 0.0;
}

void FixedStepClock::resume() noexcept { paused_ = false; }

void FixedStepClock::reset() noexcept {
    accumulator_ = 0.0;
    singleStep_ = false;
}

void FixedStepClock::setSpeed(double speed) noexcept {
    if (std::isfinite(speed) && speed > 0) settings_.speed = speed;
}

}  // namespace terra::simulation
