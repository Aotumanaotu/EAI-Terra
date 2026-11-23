#include "terra/simulation/interpolation.hpp"

#include <algorithm>
#include <cmath>

namespace terra::simulation {

namespace td = terra::data;

bool interpolatable(const td::PhysicsSnapshot& older, const td::PhysicsSnapshot& newer) noexcept {
    return older.worldInstance == newer.worldInstance && older.runEpoch == newer.runEpoch &&
           older.entities.size() == newer.entities.size();
}

td::Pose interpolate(const td::Pose& a, const td::Pose& b, double alpha) noexcept {
    const double t = std::clamp(alpha, 0.0, 1.0);
    const double s = 1.0 - t;
    td::Pose p;
    p.position = {s * a.position.x + t * b.position.x, s * a.position.y + t * b.position.y,
                  s * a.position.z + t * b.position.z};
    const td::Quaternion& qa = a.orientation;
    td::Quaternion qb = b.orientation;
    if (qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z < 0) qb = {-qb.w, -qb.x, -qb.y, -qb.z};  // 最短弧
    const td::Quaternion q{s * qa.w + t * qb.w, s * qa.x + t * qb.x, s * qa.y + t * qb.y, s * qa.z + t * qb.z};
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    p.orientation = n > 0 ? td::Quaternion{q.w / n, q.x / n, q.y / n, q.z / n} : b.orientation;
    return p;
}

std::vector<td::Pose> interpolatePoses(const td::PhysicsSnapshot& older, const td::PhysicsSnapshot& newer, double alpha) {
    std::vector<td::Pose> poses;
    poses.reserve(newer.entities.size());
    if (!interpolatable(older, newer)) {
        for (const auto& e : newer.entities) poses.push_back(e.pose);
        return poses;
    }
    for (std::size_t i = 0; i < newer.entities.size(); ++i) {
        poses.push_back(interpolate(older.entities[i].pose, newer.entities[i].pose, alpha));
    }
    return poses;
}

}  // namespace terra::simulation
