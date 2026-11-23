#pragma once

// 固定步长时钟（M2-06，M2-01 设计第 7 节）。
//
// 每帧把墙钟增量 × 速度倍率累加到“待推进时间”，按固定步长折算为整数步。单帧最多推进 maxCatchUp 秒的仿真时间
// （至少 1 步）：超出的部分丢弃——仿真比墙钟慢，计入过载次数与丢弃时间。物理步永远不跳过，也不改变步长。
// 暂停时累加器清零、不推进；单步请求使下一次 advance 恰好推进 1 步（暂停中也有效）。
// 时钟只决定“这一帧推进几步”；物理轨迹只取决于步数与按步生效的命令，因此与帧率无关。

#include <cstdint>

namespace terra::simulation {

struct ClockSettings {
    double maxCatchUp = 0.1;  // 秒，单帧最多推进的仿真时间
    double speed = 1.0;       // 仿真时间 / 墙钟时间
};

class FixedStepClock {
public:
    explicit FixedStepClock(double timestep, ClockSettings settings = {});

    // 加入墙钟增量（秒），返回本帧应推进的步数。负数、非有限值按 0 处理。
    std::uint64_t advance(double wallSeconds);

    // 累加器余量占一步的比例，[0, 1)；用于快照插值。暂停时为 0。
    double alpha() const noexcept;

    void pause() noexcept;
    void resume() noexcept;
    bool paused() const noexcept { return paused_; }
    void requestSingleStep() noexcept { singleStep_ = true; }

    // 清空累加器与单步请求（reset 世界时调用）；过载统计保留。
    void reset() noexcept;

    void setSpeed(double speed) noexcept;
    double speed() const noexcept { return settings_.speed; }
    double timestep() const noexcept { return timestep_; }
    std::uint64_t maxStepsPerFrame() const noexcept { return maxSteps_; }

    std::uint64_t overloadCount() const noexcept { return overloads_; }
    bool lastAdvanceOverloaded() const noexcept { return lastOverloaded_; }
    double droppedSeconds() const noexcept { return dropped_; }

private:
    double timestep_;
    ClockSettings settings_;
    std::uint64_t maxSteps_;
    double accumulator_ = 0.0;
    bool paused_ = false;
    bool singleStep_ = false;
    std::uint64_t overloads_ = 0;
    double dropped_ = 0.0;
    bool lastOverloaded_ = false;
};

}  // namespace terra::simulation
