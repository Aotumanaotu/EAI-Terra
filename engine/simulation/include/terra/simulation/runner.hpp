#pragma once

// 仿真推进（M2-06，M2-01 设计第 7 节；D-106：M2 单线程，接口可迁移到物理线程）。
//
// 主循环每帧调用 update(墙钟增量)：时钟决定推进几步；每步之前先把“在该步生效”的命令交给 setControl，再 step，
// 然后把新快照放入历史（保留最近两份）。显示用 displayPoses()：在两份快照之间按时钟余量插值，画面比物理晚约一步。
//
// 命令按步序号排队：schedule(k, 命令) 表示在世界从第 k 步推进到第 k+1 步之前生效（即 stepIndex() == k 时）。
// 物理轨迹因此只取决于命令与步数，与帧率、帧间隔抖动无关。安排在已经过去的步上的命令在下一步之前生效，并计为“迟到”。
// reset 形成屏障：清空命令队列、时钟累加器与快照历史，之后的第一帧不与 reset 前的快照插值。
// 步进出错（SimulationError）后不再推进，直到 reset。

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <vector>

#include "terra/data/geometry.hpp"
#include "terra/data/state.hpp"
#include "terra/physics/physics_world.hpp"
#include "terra/simulation/clock.hpp"

namespace terra::simulation {

struct FrameResult {
    std::uint64_t steps = 0;                             // 本帧实际推进的步数
    bool overloaded = false;                             // 本帧触到了单帧追赶上限
    std::optional<terra::physics::SimulationError> error;  // 本帧出现的步进错误（之后不再推进，直到 reset）
    std::vector<terra::physics::ControlError> controlErrors;  // 被 setControl 拒绝的命令
};

class SimulationRunner {
public:
    explicit SimulationRunner(terra::physics::PhysicsWorld& world, ClockSettings settings = {});

    FrameResult update(double wallSeconds);

    // 在第 step 步之前生效（stepIndex() == step 时交给 setControl）
    void schedule(std::uint64_t step, std::vector<terra::physics::ActuatorCommand> commands);
    // 在下一步之前生效
    void scheduleNext(std::vector<terra::physics::ActuatorCommand> commands);

    void pause() noexcept { clock_.pause(); }
    void resume() noexcept { clock_.resume(); }
    bool paused() const noexcept { return clock_.paused(); }
    void stepOnce() noexcept { clock_.requestSingleStep(); }
    void setSpeed(double speed) noexcept { clock_.setSpeed(speed); }

    // reset 世界并形成屏障（见文件头）
    void reset();

    // 每完成一步调用一次（记录、Headless 输出等）；在推进的线程上调用
    void setStepObserver(std::function<void(const terra::data::PhysicsSnapshot&)> observer) {
        observer_ = std::move(observer);
    }

    // 显示用的实体位姿：暂停时为最新快照；否则在最近两份快照之间按时钟余量插值
    std::vector<terra::data::Pose> displayPoses() const;
    std::shared_ptr<const terra::data::PhysicsSnapshot> latest() const noexcept { return current_; }
    std::shared_ptr<const terra::data::PhysicsSnapshot> previous() const noexcept { return previous_; }

    const FixedStepClock& clock() const noexcept { return clock_; }
    std::uint64_t lateCommands() const noexcept { return late_; }
    std::size_t pendingCommands() const noexcept { return queue_.size(); }
    bool faulted() const noexcept { return world_.faulted(); }

private:
    terra::physics::PhysicsWorld& world_;
    FixedStepClock clock_;
    std::multimap<std::uint64_t, std::vector<terra::physics::ActuatorCommand>> queue_;
    std::shared_ptr<const terra::data::PhysicsSnapshot> previous_;
    std::shared_ptr<const terra::data::PhysicsSnapshot> current_;
    std::function<void(const terra::data::PhysicsSnapshot&)> observer_;
    std::uint64_t late_ = 0;
};

}  // namespace terra::simulation
