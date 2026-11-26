#include "terra/simulation/lidar3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <cstring>

#include "terra/core/sha256.hpp"
#include "terra/scene/pose_math.hpp"

namespace terra::simulation {

namespace td = terra::data;

std::uint64_t lidar3dFrame(std::uint64_t step, double timestep, double rateHz) noexcept {
    return static_cast<std::uint64_t>(std::floor(static_cast<double>(step) * timestep * rateHz + 1e-9));
}

td::Pose lidar3dWorldPose(const td::PhysicsSnapshot& snapshot, const terra::scene::Lidar3dSensor& lidar) {
    if (!lidar.entity.valid() || lidar.entity.value > snapshot.entities.size()) return lidar.mount;
    return terra::scene::multiply(snapshot.entities[lidar.entity.value - 1].pose, lidar.mount);
}

Lidar3dScan scanLidar3d(const terra::physics::PhysicsWorld& world, const terra::scene::Lidar3dSensor& lidar) {
    const auto snapshot = world.snapshot();
    Lidar3dScan scan;
    scan.sensor = lidar.name;
    scan.step = snapshot->stepIndex;
    scan.time = snapshot->simulationTime;
    scan.runEpoch = snapshot->runEpoch;
    scan.pose = lidar3dWorldPose(*snapshot, lidar);
    scan.rangeMin = lidar.rangeMin;
    scan.rangeMax = lidar.rangeMax;
    if (!lidar.pattern || lidar.pattern->size() == 0) return scan;
    const auto& pattern = *lidar.pattern;
    const std::size_t n = lidar.pointsPerFrame();
    scan.frame = lidar3dFrame(scan.step, snapshot->fixedTimestep, lidar.rateHz);
    scan.patternStart = static_cast<std::size_t>((scan.frame * n) % pattern.size());
    scan.rays = n;

    constexpr double kDeg = 3.14159265358979323846 / 180.0;
    std::vector<td::Vec3> local(n);
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t j = (scan.patternStart + i) % pattern.size();
        const double az = static_cast<double>(pattern.azimuth[j]) * kDeg;
        const double zen = static_cast<double>(pattern.zenith[j]) * kDeg;
        local[i] = {std::sin(zen) * std::cos(az), std::sin(zen) * std::sin(az), std::cos(zen)};
    }
    // 运动畸变：第 i 点在帧内的 τ_i = i ÷ 点频 时刻测得，帧的最后一点即当前时刻。每 1 ms 一组，按该组中点时刻的安装位姿发出：
    // 位置 = 当前位置 − v·Δt，姿态 = exp(−ω·Δt) · 当前姿态（v、ω 为安装点在世界中的线速度与角速度，由安装实体的速度算出）
    td::Vec3 v{0, 0, 0}, w{0, 0, 0};
    if (lidar.motionDistortion && lidar.entity.valid() && lidar.entity.value <= snapshot->entities.size()) {
        const auto& e = snapshot->entities[lidar.entity.value - 1];
        w = e.angularVelocity;
        const td::Vec3 r = terra::scene::rotate(e.pose.orientation, lidar.mount.position);
        v = {e.linearVelocity.x + (w.y * r.z - w.z * r.y), e.linearVelocity.y + (w.z * r.x - w.x * r.z), e.linearVelocity.z + (w.x * r.y - w.y * r.x)};
    }
    const bool moving = v.x != 0 || v.y != 0 || v.z != 0 || w.x != 0 || w.y != 0 || w.z != 0;
    const double lastTime = static_cast<double>(n - 1) / lidar.pointsPerSecond;  // 最后一点在帧内的时刻 = 当前时刻
    scan.frameStart = scan.time - lastTime;
    const std::size_t chunk = moving ? std::max<std::size_t>(1, static_cast<std::size_t>(std::llround(lidar.pointsPerSecond * 0.001))) : n;
    std::vector<double> distances(n, std::numeric_limits<double>::infinity());
    for (std::size_t begin = 0; begin < n; begin += chunk) {
        const std::size_t end = std::min(n, begin + chunk);
        const double tau = (static_cast<double>(begin + end - 1) * 0.5) / lidar.pointsPerSecond;
        const double back = moving ? std::max(0.0, lastTime - tau) : 0.0;  // 这一组比当前时刻早多少秒
        td::Pose pose = scan.pose;
        if (moving && back > 0) {
            pose.position = {pose.position.x - v.x * back, pose.position.y - v.y * back, pose.position.z - v.z * back};
            const double angle = std::sqrt(w.x * w.x + w.y * w.y + w.z * w.z) * back;
            if (angle > 0) {
                const double norm = angle / back;
                const double sh = std::sin(-angle * 0.5) / norm;
                const td::Quaternion dq{std::cos(-angle * 0.5), w.x * sh, w.y * sh, w.z * sh};
                pose.orientation = terra::scene::multiply(td::Pose{{0, 0, 0}, dq}, td::Pose{{0, 0, 0}, pose.orientation}).orientation;
            }
        }
        terra::physics::RayQuery query;
        query.origin = pose.position;
        query.maxRange = lidar.rangeMax;
        query.exclude = lidar.entity;
        query.hits = lidar.hitVisual ? terra::physics::RayTargets::Visual : terra::physics::RayTargets::Collision;
        query.directions.reserve(end - begin);
        for (std::size_t i = begin; i < end; ++i) query.directions.push_back(terra::scene::rotate(pose.orientation, local[i]));
        const std::vector<double> d = world.castRays(query);
        for (std::size_t i = begin; i < end && i - begin < d.size(); ++i) distances[i] = d[i - begin];
    }
    scan.points.reserve(n);
    const int lines = std::max(1, lidar.lines);
    for (std::size_t i = 0; i < n; ++i) {
        const double d = distances[i];
        if (!std::isfinite(d) || d < lidar.rangeMin || d > lidar.rangeMax) continue;
        Lidar3dPoint p;
        p.x = static_cast<float>(local[i].x * d);
        p.y = static_cast<float>(local[i].y * d);
        p.z = static_cast<float>(local[i].z * d);
        p.time = static_cast<float>(static_cast<double>(i) / lidar.pointsPerSecond);
        p.ray = static_cast<std::uint32_t>(i);
        p.line = static_cast<std::uint8_t>(((scan.patternStart + i) % pattern.size()) % static_cast<std::size_t>(lines));
        scan.points.push_back(p);
    }
    return scan;
}

std::string lidar3dDigest(const Lidar3dScan& scan) {
    terra::Sha256 h;
    for (const auto& p : scan.points) {
        unsigned char b[17];
        std::memcpy(b, &p.x, 4);
        std::memcpy(b + 4, &p.y, 4);
        std::memcpy(b + 8, &p.z, 4);
        std::memcpy(b + 12, &p.time, 4);
        b[16] = p.line;
        h.update(b, sizeof b);
    }
    return h.finishHex();
}

}  // namespace terra::simulation
