#pragma once

#include <cstdint>

namespace terra {

// 固定步数运行的输出采样计划。
// 第 0 步（初始状态）与最后一步总是输出；中间每 sampleEvery 步输出一次。
// 采样只决定输出哪些步，不影响仿真本身。
struct RunSchedule {
    std::uint64_t totalSteps = 0;
    std::uint64_t sampleEvery = 1;  // 必须 >= 1

    bool shouldSample(std::uint64_t step) const noexcept {
        return step == 0 || step == totalSteps || (sampleEvery != 0 && step % sampleEvery == 0);
    }
};

}  // namespace terra
