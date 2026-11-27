// 算法接入档案中与 ROS 无关的部分（M5-13a），说明见 integration.hpp。

#include "terra/ros2/integration.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

namespace terra::ros2 {

bool globMatch(std::string_view pattern, std::string_view name) {
    // 回溯到最近一个 “*”；“*” 不跨 “/”
    std::size_t p = 0, n = 0;
    std::size_t starP = std::string_view::npos, starN = 0;
    while (n < name.size()) {
        if (p < pattern.size() && pattern[p] == '*') {
            starP = p++;
            starN = n;
        } else if (p < pattern.size() && pattern[p] == name[n]) {
            ++p;
            ++n;
        } else if (starP != std::string_view::npos && name[starN] != '/') {
            p = starP + 1;
            n = ++starN;
        } else {
            return false;
        }
    }
    while (p < pattern.size() && pattern[p] == '*') ++p;
    return p == pattern.size();
}

bool hasGlob(std::string_view pattern) { return pattern.find('*') != std::string_view::npos; }

std::optional<PointLayout> pointLayoutFromName(std::string_view name) {
    if (name.empty() || name == "livox") return PointLayout::Livox;
    if (name == "xyzi") return PointLayout::Xyzi;
    if (name == "velodyne") return PointLayout::Velodyne;
    if (name == "ouster") return PointLayout::Ouster;
    return std::nullopt;
}

namespace {

constexpr std::uint8_t kUint8 = 2, kUint16 = 4, kUint32 = 6, kFloat32 = 7, kFloat64 = 8;

template <class T>
void put(std::uint8_t* at, T value) {
    std::memcpy(at, &value, sizeof value);
}

}  // namespace

PackedCloud packPoints(PointLayout layout, const terra::simulation::Lidar3dScan& scan, double headerTime) {
    PackedCloud c;
    const auto field = [&](const char* name, std::uint32_t offset, std::uint8_t type) { c.fields.push_back({name, offset, type}); };
    field("x", 0, kFloat32);
    field("y", 4, kFloat32);
    field("z", 8, kFloat32);
    switch (layout) {
        case PointLayout::Livox:
            field("intensity", 12, kFloat32);
            field("tag", 16, kUint8);
            field("line", 17, kUint8);
            field("timestamp", 18, kFloat64);
            c.pointStep = 26;
            break;
        case PointLayout::Xyzi:
            field("intensity", 12, kFloat32);
            c.pointStep = 16;
            break;
        case PointLayout::Velodyne:
            field("intensity", 12, kFloat32);
            field("ring", 16, kUint16);
            field("time", 18, kFloat32);
            c.pointStep = 22;
            break;
        case PointLayout::Ouster:
            field("intensity", 16, kFloat32);
            field("t", 20, kUint32);
            field("reflectivity", 24, kUint16);
            field("ring", 26, kUint16);
            field("ambient", 28, kUint16);
            field("range", 32, kUint32);
            c.pointStep = 48;
            break;
    }
    c.data.assign(static_cast<std::size_t>(c.pointStep) * scan.points.size(), 0);
    const float intensity = 100.0f;
    std::uint8_t* out = c.data.data();
    for (const auto& p : scan.points) {
        put(out, p.x);
        put(out + 4, p.y);
        put(out + 8, p.z);
        switch (layout) {
            case PointLayout::Livox:
                put(out + 12, intensity);
                out[16] = 0;
                out[17] = p.line;
                put(out + 18, (headerTime + static_cast<double>(p.time)) * 1e9);
                break;
            case PointLayout::Xyzi:
                put(out + 12, intensity);
                break;
            case PointLayout::Velodyne:
                put(out + 12, intensity);
                put(out + 16, static_cast<std::uint16_t>(p.line));
                put(out + 18, p.time);
                break;
            case PointLayout::Ouster: {
                put(out + 16, intensity);
                put(out + 20, static_cast<std::uint32_t>(std::llround(static_cast<double>(p.time) * 1e9)));
                put(out + 26, static_cast<std::uint16_t>(p.line));
                const double r = std::sqrt(static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y + static_cast<double>(p.z) * p.z);
                put(out + 32, static_cast<std::uint32_t>(std::llround(r * 1000.0)));
                break;
            }
        }
        out += c.pointStep;
    }
    return c;
}

void WallStamps::sync(double simTime, std::uint64_t runEpoch, double wallNow) {
    const bool drift = anchored_ && std::fabs(wallNow - (offset_ + simTime)) > tolerance_;
    if (!anchored_ || runEpoch != epoch_ || simTime < lastSim_ || drift) {
        offset_ = wallNow - simTime;
        anchored_ = true;
        epoch_ = runEpoch;
        ++reanchors_;
    }
    lastSim_ = simTime;
}

void CommandArbiter::configure(std::vector<CommandInput> inputs) {
    inputs_ = std::move(inputs);
    slots_.assign(inputs_.size(), Slot{});
}

void CommandArbiter::receive(std::size_t input, const std::array<double, 3>& command, double now) {
    if (input >= slots_.size()) return;
    slots_[input].command = command;
    slots_[input].at = now;
    slots_[input].received = true;
}

std::optional<std::size_t> CommandArbiter::active(double now) const {
    std::optional<std::size_t> best;
    for (std::size_t i = 0; i < slots_.size(); ++i) {
        const Slot& s = slots_[i];
        if (!s.received || now - s.at > inputs_[i].timeout) continue;
        if (!best || inputs_[i].priority > inputs_[*best].priority ||
            (inputs_[i].priority == inputs_[*best].priority && s.at > slots_[*best].at)) {
            best = i;
        }
    }
    return best;
}

std::optional<std::array<double, 3>> CommandArbiter::current(double now) const {
    if (const auto i = active(now)) return slots_[*i].command;
    const bool any = std::any_of(slots_.begin(), slots_.end(), [](const Slot& s) { return s.received; });
    if (any) return std::array<double, 3>{0.0, 0.0, 0.0};
    return std::nullopt;
}

std::array<double, 3> planarJointVelocities(const std::array<double, 3>& command, double yaw, const std::array<double, 3>& limits) {
    std::array<double, 3> v = command;
    for (std::size_t i = 0; i < 3; ++i) {
        if (!std::isfinite(v[i])) v[i] = 0.0;
        if (limits[i] > 0.0) v[i] = std::clamp(v[i], -limits[i], limits[i]);
    }
    const double c = std::cos(yaw), s = std::sin(yaw);
    return {c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]};
}

}  // namespace terra::ros2
