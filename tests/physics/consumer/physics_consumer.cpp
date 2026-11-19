#include "physics_consumer.hpp"

#include <algorithm>
#include <cmath>

namespace terra::test {

std::optional<std::vector<terra::physics::ActuatorCommand>> commandsByName(
    const terra::data::ModelDescription& model,
    const std::vector<std::pair<std::string, std::vector<double>>>& named, std::string& error) {
    std::vector<terra::physics::ActuatorCommand> commands;
    for (const auto& [name, values] : named) {
        const terra::data::ActuatorDescription* actuator = model.findActuator(name);
        if (actuator == nullptr) {
            error = "模型中没有执行器 '" + name + "'";
            return std::nullopt;
        }
        commands.push_back({actuator->id, values});
    }
    return commands;
}

std::optional<RunSummary> runWithControl(terra::physics::PhysicsWorld& world,
                                         const std::vector<terra::physics::ActuatorCommand>& commands, int steps,
                                         std::string& error) {
    if (const auto controlError = world.setControl(commands)) {
        error = controlError->message;
        return std::nullopt;
    }
    RunSummary summary;
    for (int i = 0; i < steps; ++i) {
        if (const auto stepError = world.step()) {
            error = stepError->message;
            return std::nullopt;
        }
        const auto snap = world.snapshot();
        summary.allFinite = summary.allFinite && snap->diagnostics.finite;
        for (const auto& e : snap->entities) {
            const auto& q = e.pose.orientation;
            const double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
            summary.maxQuaternionNormError = std::max(summary.maxQuaternionNormError, std::fabs(norm - 1.0));
        }
    }
    const auto last = world.snapshot();
    summary.steps = last->stepIndex;
    summary.time = last->simulationTime;
    summary.entities = last->entities;
    summary.joints = last->joints;
    return summary;
}

}  // namespace terra::test
