// 动作的绑定、插值与播放（M3-04）。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>

#include "terra/simulation/motion.hpp"

namespace terra::simulation {

namespace td = terra::data;
namespace tp = terra::physics;
using Severity = MotionIssue::Severity;

namespace {

std::string number(double v) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.6g", v);
    return buf;
}

// 找不到执行器时的提示：没带前缀时说明写法；带了前缀时列出该实例的执行器（最多 8 个）
std::string actuatorHint(const td::ModelDescription& model, const std::string& name) {
    const auto slash = name.find('/');
    if (slash == std::string::npos) return "（执行器名需要带实例前缀，如 “<实例名>/" + name + "”）";
    const std::string prefix = name.substr(0, slash + 1);
    std::string list;
    std::size_t count = 0;
    for (const auto& a : model.actuators) {
        if (a.name.rfind(prefix, 0) != 0) continue;
        if (count < 8) list += (list.empty() ? "" : "、") + a.name;
        ++count;
    }
    if (count == 0) return "（场景中没有实例前缀 “" + prefix + "”）";
    if (count > 8) list += " 等 " + std::to_string(count) + " 个";
    return "（该实例的执行器：" + list + "）";
}

}  // namespace

std::optional<std::pair<double, double>> controlRange(const td::ActuatorDescription& actuator) {
    if (actuator.inputs.empty() || !actuator.inputs[0].limited) return std::nullopt;
    return std::make_pair(actuator.inputs[0].min, actuator.inputs[0].max);
}

MotionBindResult bindMotion(const MotionDocument& document, const td::ModelDescription& model, const TargetRange& range) {
    MotionBindResult out;
    BoundMotion motion;
    motion.name = document.name;
    motion.interpolation = document.interpolation;
    bool failed = false;
    const auto error = [&](const std::string& where, std::string message) {
        out.issues.push_back({Severity::Error, where, std::move(message)});
        failed = true;
    };

    if (!document.startKeyframe.empty()) {
        const int k = model.findKeyframe(document.startKeyframe);
        if (k < 0) {
            std::string names;
            for (const auto& key : model.keyframes) names += (names.empty() ? "" : "、") + key.name;
            error("/startKeyframe", "找不到关键帧“" + document.startKeyframe + "”" +
                                        (names.empty() ? "；场景中没有关键帧" : "；场景中的关键帧：" + names));
        } else {
            motion.startKeyframe = static_cast<std::size_t>(k);
        }
    }

    std::map<std::uint32_t, MotionTrack> tracks;
    std::map<std::string, bool> warnedOther;
    for (const MotionPose& pose : document.poses) {
        for (const auto& [name, value] : pose.targets) {
            const std::string where = pose.where + "/targets/" + jsonPointerToken(name);
            const td::ActuatorDescription* a = model.findActuator(name);
            if (a == nullptr) {
                error(where, "找不到执行器“" + name + "”" + actuatorHint(model, name));
                continue;
            }
            if (a->inputs.size() != 1) {
                error(where, "执行器“" + name + "”有 " + std::to_string(a->inputs.size()) + " 个控制输入，动作文件只支持单输入执行器");
                continue;
            }
            if (const auto r = range(*a); r && (value < r->first || value > r->second)) {
                error(where, "目标 " + number(value) + " 超出执行器“" + name + "”的范围 [" + number(r->first) + ", " +
                                 number(r->second) + "]");
                continue;
            }
            if (a->kind == td::ActuatorKind::Other && !warnedOther[name]) {
                warnedOther[name] = true;
                out.issues.push_back({Severity::Warning, where,
                                      "执行器“" + name + "”带动力学或不是力矩 / 位置 / 速度执行器，目标值按控制量原样下发"});
            }
            MotionTrack& track = tracks[a->id.value];
            track.actuator = a->id;
            track.name = name;
            track.keys.emplace_back(pose.time, value);
        }
        motion.duration = std::max(motion.duration, pose.time);
    }
    if (failed) return out;
    for (auto& entry : tracks) motion.tracks.push_back(std::move(entry.second));
    out.motion = std::move(motion);
    return out;
}

std::optional<double> sampleTrack(const MotionTrack& track, MotionInterpolation interpolation, double t) {
    if (track.keys.empty() || t < track.keys.front().first) return std::nullopt;
    if (t >= track.keys.back().first) return track.keys.back().second;
    // 第一个时间大于 t 的点
    const auto next = std::upper_bound(track.keys.begin(), track.keys.end(), t,
                                       [](double at, const std::pair<double, double>& key) { return at < key.first; });
    const auto& [t1, v1] = *next;
    const auto& [t0, v0] = *(next - 1);
    const double s = (t - t0) / (t1 - t0);
    switch (interpolation) {
        case MotionInterpolation::Step: return v0;
        case MotionInterpolation::Linear: return v0 + (v1 - v0) * s;
        case MotionInterpolation::Smooth: return v0 + (v1 - v0) * (s * s * (3.0 - 2.0 * s));
    }
    return v0;
}

double MotionPlayer::timeAt(std::uint64_t step, double timestep) const noexcept {
    if (step < start_) return 0.0;
    const double t = static_cast<double>(step - start_) * timestep;
    if (loop_ && motion_.duration > 0) return std::fmod(t, motion_.duration);
    return t;
}

bool MotionPlayer::finished(std::uint64_t step, double timestep) const noexcept {
    return !loop_ && step >= start_ && static_cast<double>(step - start_) * timestep >= motion_.duration;
}

std::vector<tp::ActuatorCommand> MotionPlayer::commands(std::uint64_t step, double timestep) const {
    std::vector<tp::ActuatorCommand> out;
    if (!playing_ || step < start_) return out;
    const double t = timeAt(step, timestep);
    for (const MotionTrack& track : motion_.tracks) {
        if (const auto v = sampleTrack(track, motion_.interpolation, t)) out.push_back({track.actuator, {*v}});
    }
    return out;
}

}  // namespace terra::simulation
