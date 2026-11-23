#include "terra/simulation/runner.hpp"

#include <utility>

#include "terra/simulation/interpolation.hpp"

namespace terra::simulation {

namespace tp = terra::physics;
namespace td = terra::data;

SimulationRunner::SimulationRunner(tp::PhysicsWorld& world, ClockSettings settings)
    : world_(world), clock_(world.description().timestep, settings), current_(world.snapshot()) {}

void SimulationRunner::schedule(std::uint64_t step, std::vector<tp::ActuatorCommand> commands) {
    queue_.emplace(step, std::move(commands));
}

void SimulationRunner::scheduleNext(std::vector<tp::ActuatorCommand> commands) {
    schedule(world_.stepIndex(), std::move(commands));
}

FrameResult SimulationRunner::update(double wallSeconds) {
    FrameResult result;
    const std::uint64_t steps = clock_.advance(wallSeconds);
    result.overloaded = clock_.lastAdvanceOverloaded();
    if (world_.faulted()) return result;
    for (std::uint64_t i = 0; i < steps; ++i) {
        const std::uint64_t index = world_.stepIndex();
        // 先交出所有在这一步（或更早）生效的命令，按安排的先后顺序
        for (auto it = queue_.begin(); it != queue_.end() && it->first <= index;) {
            if (it->first < index) ++late_;
            if (auto error = world_.setControl(it->second)) result.controlErrors.push_back(std::move(*error));
            it = queue_.erase(it);
        }
        if (auto error = world_.step()) {
            result.error = std::move(error);
            previous_ = current_;
            current_ = world_.snapshot();
            break;
        }
        ++result.steps;
        previous_ = current_;
        current_ = world_.snapshot();
        if (observer_) observer_(*current_);
    }
    return result;
}

void SimulationRunner::reset() {
    world_.reset();
    queue_.clear();
    clock_.reset();
    previous_.reset();
    current_ = world_.snapshot();
}

std::vector<td::Pose> SimulationRunner::displayPoses() const {
    if (!current_) return {};
    if (!previous_ || clock_.paused()) {
        std::vector<td::Pose> poses;
        poses.reserve(current_->entities.size());
        for (const auto& e : current_->entities) poses.push_back(e.pose);
        return poses;
    }
    return interpolatePoses(*previous_, *current_, clock_.alpha());
}

}  // namespace terra::simulation
