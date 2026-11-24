#include "terra/simulation/pd_layer.hpp"

#include <algorithm>
#include <cmath>

namespace terra::simulation {

namespace td = terra::data;
namespace tp = terra::physics;

PdLayer::PdLayer(const td::ModelDescription& model, const std::vector<terra::scene::PdAssignment>& assignments) {
    for (const auto& a : assignments) {
        const td::ActuatorDescription* act = model.actuator(a.actuator);
        if (act == nullptr || !act->joint.valid() || act->inputs.size() != 1) continue;
        const td::JointDescription* joint = model.joint(act->joint);
        if (joint == nullptr) continue;
        Channel c;
        c.actuator = a.actuator;
        c.joint = act->joint.value - 1;
        c.kp = a.kp;
        c.kd = a.kd;
        const double scale = act->gain * act->gear;
        c.scale = std::fabs(scale) > 0 ? scale : 1.0;
        c.limited = act->inputs[0].limited;
        c.min = act->inputs[0].min;
        c.max = act->inputs[0].max;
        if (joint->limited) c.jointRange = std::make_pair(joint->rangeMin, joint->rangeMax);
        channels_[a.actuator.value] = c;
    }
}

std::optional<std::pair<double, double>> PdLayer::targetRange(const td::ActuatorDescription& actuator) const {
    const auto it = channels_.find(actuator.id.value);
    if (it == channels_.end()) return controlRange(actuator);
    return it->second.jointRange;
}

TargetRange PdLayer::targetRangeFunction() const {
    return [this](const td::ActuatorDescription& actuator) { return targetRange(actuator); };
}

void PdLayer::setTarget(td::ActuatorId actuator, double target) {
    const auto it = channels_.find(actuator.value);
    if (it != channels_.end()) it->second.target = target;
}

std::optional<double> PdLayer::target(td::ActuatorId actuator) const {
    const auto it = channels_.find(actuator.value);
    return it == channels_.end() ? std::nullopt : it->second.target;
}

void PdLayer::clearTargets() noexcept {
    for (auto& entry : channels_) entry.second.target.reset();
}

double PdLayer::control(td::ActuatorId actuator, double target, double q, double qd) const {
    const auto it = channels_.find(actuator.value);
    if (it == channels_.end()) return 0.0;
    const Channel& c = it->second;
    const double torque = c.kp * (target - q) - c.kd * qd;
    double u = torque / c.scale;
    if (c.limited) u = std::clamp(u, c.min, c.max);
    return u;
}

std::vector<tp::ActuatorCommand> PdLayer::process(std::vector<tp::ActuatorCommand> commands, const td::PhysicsSnapshot& state) {
    if (channels_.empty()) return commands;
    std::vector<tp::ActuatorCommand> out;
    out.reserve(commands.size() + channels_.size());
    for (auto& command : commands) {
        const auto it = channels_.find(command.actuator.value);
        if (it == channels_.end()) {
            out.push_back(std::move(command));
        } else if (!command.values.empty()) {
            it->second.target = command.values[0];
        }
    }
    for (auto& entry : channels_) {
        Channel& c = entry.second;
        if (c.joint >= state.joints.size()) continue;
        const td::JointState& js = state.joints[c.joint];
        if (!c.target) c.target = js.position[0];
        out.push_back({c.actuator, {control(c.actuator, *c.target, js.position[0], js.velocity[0])}});
    }
    return out;
}

}  // namespace terra::simulation
