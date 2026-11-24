#include "terra/simulation/lidar.hpp"

#include <cmath>
#include <limits>

#include "terra/scene/pose_math.hpp"

namespace terra::simulation {

namespace td = terra::data;

td::Pose lidarWorldPose(const td::PhysicsSnapshot& snapshot, const terra::scene::LidarSensor& lidar) {
    if (!lidar.entity.valid() || lidar.entity.value > snapshot.entities.size()) return lidar.mount;
    return terra::scene::multiply(snapshot.entities[lidar.entity.value - 1].pose, lidar.mount);
}

double lidarBeamAngle(const terra::scene::LidarSensor& lidar, int beam) {
    if (lidar.samples <= 1) return lidar.angleMin;
    const double increment = (lidar.angleMax - lidar.angleMin) / static_cast<double>(lidar.samples - 1);
    return lidar.angleMin + static_cast<double>(beam) * increment;
}

LidarScan scanLidar(const terra::physics::PhysicsWorld& world, const terra::scene::LidarSensor& lidar) {
    const auto snapshot = world.snapshot();
    LidarScan scan;
    scan.sensor = lidar.name;
    scan.step = snapshot->stepIndex;
    scan.time = snapshot->simulationTime;
    scan.runEpoch = snapshot->runEpoch;
    scan.pose = lidarWorldPose(*snapshot, lidar);
    scan.angleMin = lidar.angleMin;
    scan.angleIncrement = lidar.samples > 1 ? (lidar.angleMax - lidar.angleMin) / static_cast<double>(lidar.samples - 1) : 0.0;
    scan.rangeMin = lidar.rangeMin;
    scan.rangeMax = lidar.rangeMax;

    terra::physics::RayQuery query;
    query.origin = scan.pose.position;
    query.maxRange = lidar.rangeMax;
    query.exclude = lidar.entity;
    query.hits = lidar.hitVisual ? terra::physics::RayTargets::Visual : terra::physics::RayTargets::Collision;
    query.directions.reserve(static_cast<std::size_t>(lidar.samples));
    for (int i = 0; i < lidar.samples; ++i) {
        const double a = lidarBeamAngle(lidar, i);
        query.directions.push_back(terra::scene::rotate(scan.pose.orientation, {std::cos(a), std::sin(a), 0.0}));
    }
    const std::vector<double> distances = world.castRays(query);
    scan.ranges.reserve(distances.size());
    for (const double d : distances) {
        if (!std::isfinite(d)) scan.ranges.push_back(std::numeric_limits<float>::infinity());
        else if (d < lidar.rangeMin) scan.ranges.push_back(-std::numeric_limits<float>::infinity());
        else scan.ranges.push_back(static_cast<float>(d));
    }
    return scan;
}

}  // namespace terra::simulation
