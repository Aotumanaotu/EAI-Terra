// 策略控制器（M4-10a），说明见 controller.hpp。

#include "terra/policy/controller.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <utility>

#include "terra/simulation/sensor_sampling.hpp"

namespace terra::policy {

namespace {

namespace td = terra::data;
namespace tp = terra::physics;

constexpr double kPi = 3.14159265358979323846;

double at(const std::vector<double>& v, std::size_t i, double fallback) {
    if (v.empty()) return fallback;
    return v.size() == 1 ? v[0] : (i < v.size() ? v[i] : fallback);
}

// 世界坐标中的向量 v 转到四元数 q（局部 → 世界）的局部坐标：q⁻¹ v q
td::Vec3 toLocal(const td::Quaternion& q, const td::Vec3& v) {
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    const double w = n > 0 ? q.w / n : 1.0;
    const double ux = n > 0 ? -q.x / n : 0.0, uy = n > 0 ? -q.y / n : 0.0, uz = n > 0 ? -q.z / n : 0.0;
    // t = 2 u × v；v' = v + w t + u × t
    const double tx = 2 * (uy * v.z - uz * v.y), ty = 2 * (uz * v.x - ux * v.z), tz = 2 * (ux * v.y - uy * v.x);
    return {v.x + w * tx + (uy * tz - uz * ty), v.y + w * ty + (uz * tx - ux * tz), v.z + w * tz + (ux * ty - uy * tx)};
}

}  // namespace

std::unique_ptr<PolicyController> PolicyController::create(PolicyDescription policy, std::unique_ptr<PolicyModel> model,
                                                           const terra::scene::SceneInstance& scene, bool noise, std::string& error) {
    if (!model || !scene.world) {
        error = "没有模型或场景";
        return nullptr;
    }
    const td::ModelDescription& d = scene.world->description();
    std::unique_ptr<PolicyController> c(new PolicyController());
    c->timestep_ = d.timestep;
    if (!(d.timestep > 0)) {
        error = "场景的物理步长无效";
        return nullptr;
    }
    const double ratio = policy.controlPeriod / d.timestep;
    c->decimation_ = static_cast<int>(std::lround(ratio));
    if (c->decimation_ < 1 || std::fabs(ratio - c->decimation_) > 1e-6 * ratio) {
        error = "控制周期 " + std::to_string(policy.controlPeriod) + " s 不是物理步长 " + std::to_string(d.timestep) + " s 的整数倍";
        return nullptr;
    }

    // 动作：关节 → 执行器
    std::string problems;
    const auto problem = [&](const std::string& text) { problems += (problems.empty() ? "" : "；") + text; };
    for (std::size_t i = 0; i < policy.action.joints.size(); ++i) {
        const std::string& name = policy.action.joints[i];
        const td::JointDescription* joint = d.findJoint(name);
        if (joint == nullptr) {
            problem("场景中没有关节“" + name + "”");
            continue;
        }
        td::ActuatorKind wanted = td::ActuatorKind::Motor;
        if (policy.action.type == ActionType::Position) wanted = td::ActuatorKind::Position;
        if (policy.action.type == ActionType::Velocity) wanted = td::ActuatorKind::Velocity;
        const td::ActuatorDescription* actuator = nullptr;
        for (const auto& a : d.actuators) {
            if (a.transmission == td::ActuatorTransmission::Joint && a.joint.value == joint->id.value && a.inputs.size() == 1 && a.kind == wanted) {
                actuator = &a;
                break;
            }
        }
        if (actuator == nullptr) {
            const char* kind = wanted == td::ActuatorKind::Motor ? "力矩（motor）" : wanted == td::ActuatorKind::Position ? "位置（position）" : "速度（velocity）";
            problem("关节“" + name + "”没有" + kind + "执行器" +
                    (policy.action.type == ActionType::PositionPd ? "（positionPd 需要力矩执行器，请用低层控制的场景）" : ""));
            continue;
        }
        Channel ch;
        ch.joint = joint->id.value - 1;
        ch.actuator = actuator->id;
        ch.scale = at(policy.action.scale, i, 1.0);
        ch.offset = at(policy.action.offset, i, 0.0);
        ch.kp = at(policy.action.kp, i, 0.0);
        ch.kd = at(policy.action.kd, i, 0.0);
        ch.torqueLimit = at(policy.action.torqueLimit, i, 0.0);
        const double s = actuator->gain * actuator->gear;
        ch.actuatorScale = std::fabs(s) > 0 ? s : 1.0;
        ch.limited = actuator->inputs[0].limited;
        ch.min = actuator->inputs[0].min;
        ch.max = actuator->inputs[0].max;
        c->channels_.push_back(ch);
    }

    // 观测项
    for (std::size_t i = 0; i < policy.observation.size(); ++i) {
        Term t;
        t.spec = policy.observation[i];
        const std::string where = std::string{termName(t.spec.kind)} + "：";
        if (!t.spec.sensor.empty()) {
            const td::SensorDescription* s = d.findSensor(t.spec.sensor);
            if (s == nullptr || s->kind != td::SensorKind::Imu) problem(where + "场景中没有 IMU“" + t.spec.sensor + "”");
            else t.sensor = static_cast<int>(s->id.value - 1);
        }
        if (!t.spec.entity.empty()) {
            const td::EntityDescription* e = d.findEntity(t.spec.entity);
            if (e == nullptr) problem(where + "场景中没有实体“" + t.spec.entity + "”");
            else t.entity = static_cast<int>(e->id.value - 1);
        }
        if (t.spec.kind == TermKind::JointPositions || t.spec.kind == TermKind::JointVelocities) {
            for (const auto& name : t.spec.joints) {
                const td::JointDescription* joint = d.findJoint(name);
                if (joint == nullptr || (joint->type != td::JointType::Hinge && joint->type != td::JointType::Slide)) {
                    problem(where + "场景中没有转动或滑动关节“" + name + "”");
                    continue;
                }
                t.joints.push_back(joint->id.value - 1);
            }
        }
        if (t.spec.kind == TermKind::Phase) {
            c->phaseSteps_ = static_cast<int>(std::lround(t.spec.period / policy.controlPeriod));
            if (c->phaseSteps_ < 1) problem(where + "周期小于控制周期");
        }
        c->terms_.push_back(std::move(t));
    }
    if (!problems.empty()) {
        error = "策略与场景不匹配：" + problems;
        return nullptr;
    }

    // 传感器的采样频率与噪声
    c->sensorRates_.assign(d.sensors.size(), 1.0 / d.timestep);
    for (const auto& cfg : scene.sensors) {
        if (cfg.sensor.valid() && cfg.sensor.value <= c->sensorRates_.size()) c->sensorRates_[cfg.sensor.value - 1] = cfg.rateHz;
    }
    c->held_.assign(d.sensors.size(), {});
    c->heldValid_.assign(d.sensors.size(), false);
    c->noise_ = terra::simulation::SensorNoise(scene, noise);

    c->policy_ = std::move(policy);
    c->model_ = std::move(model);
    c->setCommand(c->policy_.defaultCommand);
    c->output_.assign(static_cast<std::size_t>(c->policy_.actionWidth()), 0.0f);
    c->targets_.assign(c->channels_.size(), 0.0);
    for (std::size_t i = 0; i < c->channels_.size(); ++i) c->targets_[i] = c->channels_[i].offset;
    return c;
}

void PolicyController::setCommand(const std::array<double, 3>& command) {
    for (std::size_t i = 0; i < 3; ++i) command_[i] = std::clamp(command[i], policy_.commandRange[i][0], policy_.commandRange[i][1]);
}

std::vector<td::ActuatorId> PolicyController::actuators() const {
    std::vector<td::ActuatorId> out;
    for (const auto& c : channels_) out.push_back(c.actuator);
    return out;
}

void PolicyController::readSensors(const td::PhysicsSnapshot& snapshot) {
    for (const auto& t : terms_) {
        if (t.sensor < 0) continue;
        const auto k = static_cast<std::size_t>(t.sensor);
        if (k >= snapshot.sensors.size()) continue;
        if (heldValid_[k] && !terra::simulation::sensorSampleDue(snapshot.stepIndex, timestep_, sensorRates_[k])) continue;
        held_[k] = snapshot.sensors[k].values;
        noise_.imu(k, snapshot.stepIndex, held_[k]);  // M4-09：与其他出口逐位相同的带噪声读数
        heldValid_[k] = true;
    }
}

std::vector<float> PolicyController::observe(const td::PhysicsSnapshot& snapshot) {
    readSensors(snapshot);
    std::vector<double> frame;
    frame.reserve(static_cast<std::size_t>(policy_.frameWidth()));
    for (const auto& t : terms_) {
        std::vector<double> v;
        const auto entityPose = [&]() -> const td::EntityState* {
            return t.entity >= 0 && static_cast<std::size_t>(t.entity) < snapshot.entities.size() ? &snapshot.entities[static_cast<std::size_t>(t.entity)]
                                                                                                    : nullptr;
        };
        const std::vector<double>* imu = t.sensor >= 0 && heldValid_[static_cast<std::size_t>(t.sensor)] ? &held_[static_cast<std::size_t>(t.sensor)] : nullptr;
        switch (t.spec.kind) {
            case TermKind::BaseAngularVelocity:
                if (imu && imu->size() >= 6) {
                    v = {(*imu)[3], (*imu)[4], (*imu)[5]};
                } else if (const auto* e = entityPose()) {
                    const auto w = toLocal(e->pose.orientation, e->angularVelocity);
                    v = {w.x, w.y, w.z};
                }
                break;
            case TermKind::ProjectedGravity: {
                td::Quaternion q;
                bool have = false;
                if (imu && imu->size() >= 10) {
                    q = {(*imu)[6], (*imu)[7], (*imu)[8], (*imu)[9]};
                    have = true;
                } else if (const auto* e = entityPose()) {
                    q = e->pose.orientation;
                    have = true;
                }
                if (have) {
                    const auto g = toLocal(q, {0.0, 0.0, -1.0});
                    v = {g.x, g.y, g.z};
                }
                break;
            }
            case TermKind::BaseLinearVelocity:
                if (const auto* e = entityPose()) {
                    const auto l = toLocal(e->pose.orientation, e->linearVelocity);
                    v = {l.x, l.y, l.z};
                }
                break;
            case TermKind::JointPositions:
                for (std::size_t i = 0; i < t.joints.size(); ++i) {
                    const double q = t.joints[i] < snapshot.joints.size() ? snapshot.joints[t.joints[i]].position[0] : 0.0;
                    v.push_back(q - at(t.spec.offset, i, 0.0));
                }
                break;
            case TermKind::JointVelocities:
                for (std::size_t j : t.joints) v.push_back(j < snapshot.joints.size() ? snapshot.joints[j].velocity[0] : 0.0);
                break;
            case TermKind::LastAction:
                for (float a : output_) v.push_back(static_cast<double>(a));
                break;
            case TermKind::Command: v = {command_[0], command_[1], command_[2]}; break;
            case TermKind::Phase: {
                const double magnitude = std::hypot(command_[0], command_[1]) + std::fabs(command_[2]);
                if (magnitude > t.spec.standingThreshold && phaseSteps_ > 0) {
                    const auto n = static_cast<std::uint64_t>(phaseSteps_);
                    const double phase = static_cast<double>(episodeInferences_ % n) / static_cast<double>(n);
                    v = {std::sin(2 * kPi * phase), std::cos(2 * kPi * phase)};
                } else {
                    v = {0.0, 0.0};
                }
                break;
            }
        }
        v.resize(static_cast<std::size_t>(policy_.termWidth(t.spec)), 0.0);
        for (std::size_t i = 0; i < v.size(); ++i) {
            double x = v[i] * at(t.spec.scale, i, 1.0);
            if (t.spec.clip > 0) x = std::clamp(x, -t.spec.clip, t.spec.clip);
            frame.push_back(x);
        }
    }
    std::vector<float> out(frame.size());
    for (std::size_t i = 0; i < frame.size(); ++i) out[i] = static_cast<float>(frame[i]);
    return out;
}

double PolicyController::control(const Channel& c, double target, double q, double qd) const {
    double u = target;
    if (policy_.action.type == ActionType::PositionPd || policy_.action.type == ActionType::Torque) {
        double tau = policy_.action.type == ActionType::PositionPd ? c.kp * (target - q) - c.kd * qd : target;
        if (c.torqueLimit > 0) tau = std::clamp(tau, -c.torqueLimit, c.torqueLimit);
        u = tau / c.actuatorScale;
    }
    if (c.limited) u = std::clamp(u, c.min, c.max);
    return u;
}

std::vector<tp::ActuatorCommand> PolicyController::commands(const td::PhysicsSnapshot& snapshot) {
    if (failed_) return {};
    const std::uint64_t step = snapshot.stepIndex;
    if (!started_ || step < lastStep_) {
        started_ = true;
        start_ = step;
        episodeInferences_ = 0;
        history_.clear();
        std::fill(heldValid_.begin(), heldValid_.end(), false);
        std::fill(output_.begin(), output_.end(), 0.0f);
    }
    lastStep_ = step;
    readSensors(snapshot);
    if ((step - start_) % static_cast<std::uint64_t>(decimation_) == 0) {
        frame_ = observe(snapshot);
        const std::size_t width = frame_.size();
        const std::size_t total = width * static_cast<std::size_t>(policy_.historyLength);
        if (history_.size() != total) {
            history_.clear();
            for (int h = 0; h < policy_.historyLength; ++h) history_.insert(history_.end(), frame_.begin(), frame_.end());
        } else {
            std::move(history_.begin() + static_cast<std::ptrdiff_t>(width), history_.end(), history_.begin());
            std::copy(frame_.begin(), frame_.end(), history_.end() - static_cast<std::ptrdiff_t>(width));
        }
        std::vector<float> out;
        const auto begin = std::chrono::steady_clock::now();
        std::string why;
        const bool ok = model_->run(history_, out, why);
        lastInferenceSeconds_ = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
        if (!ok || out.size() != output_.size() || !std::all_of(out.begin(), out.end(), [](float x) { return std::isfinite(x); })) {
            failed_ = true;
            error_ = !ok ? "推理失败：" + why : out.size() != output_.size() ? "推理输出的宽度不对" : "策略输出了非有限值（NaN 或无穷大）";
            return {};
        }
        if (policy_.action.clip > 0) {
            const auto clip = static_cast<float>(policy_.action.clip);
            for (float& a : out) a = std::clamp(a, -clip, clip);
        }
        output_ = std::move(out);
        for (std::size_t i = 0; i < channels_.size(); ++i) {
            targets_[i] = channels_[i].offset + channels_[i].scale * static_cast<double>(output_[i]);
        }
        ++episodeInferences_;
        ++inferences_;
    }
    std::vector<tp::ActuatorCommand> out;
    out.reserve(channels_.size());
    for (std::size_t i = 0; i < channels_.size(); ++i) {
        const Channel& c = channels_[i];
        const double q = c.joint < snapshot.joints.size() ? snapshot.joints[c.joint].position[0] : 0.0;
        const double qd = c.joint < snapshot.joints.size() ? snapshot.joints[c.joint].velocity[0] : 0.0;
        out.push_back({c.actuator, {control(c, targets_[i], q, qd)}});
    }
    return out;
}

std::vector<tp::ActuatorCommand> overrideCommands(std::vector<tp::ActuatorCommand> base, const std::vector<tp::ActuatorCommand>& overrides) {
    std::set<std::uint32_t> taken;
    for (const auto& o : overrides) taken.insert(o.actuator.value);
    base.erase(std::remove_if(base.begin(), base.end(), [&](const tp::ActuatorCommand& c) { return taken.count(c.actuator.value) != 0; }), base.end());
    base.insert(base.end(), overrides.begin(), overrides.end());
    return base;
}

}  // namespace terra::policy
