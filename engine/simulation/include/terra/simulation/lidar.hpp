#pragma once

// 2D 激光雷达（M4-03b，M4-01 设计稿第 3、4 节）：在当前观测时刻，从雷达安装点沿扫描平面发出 samples 束射线，
// 用物理世界的射线检测（PhysicsWorld::castRays）求距离。只依赖物理接口，Headless 与查看器共用；不需要图形。
//
// 约定与 sensor_msgs/LaserScan 一致：
//   - 扫描平面为安装坐标系的 XY 平面，角度从 +X 起逆时针为正；第 i 束的角度 = angleMin + i × angleIncrement，
//     angleIncrement = (angleMax − angleMin) / (samples − 1)（只有一束时为 0）；
//   - 距离单位米，单精度；超出最大量程（没有命中）为 +inf，近于最小量程为 −inf（ROS REP-117）。
// 读数只是几何距离，没有噪声与强度（P1）。

#include <cstdint>
#include <string>
#include <vector>

#include "terra/data/geometry.hpp"
#include "terra/data/state.hpp"
#include "terra/physics/physics_world.hpp"
#include "terra/scene/scene.hpp"

namespace terra::simulation {

struct LidarScan {
    std::string sensor;
    std::uint64_t step = 0;
    double time = 0.0;
    std::uint64_t runEpoch = 0;
    terra::data::Pose pose;  // 雷达安装坐标系在世界中的位姿
    double angleMin = 0.0;
    double angleIncrement = 0.0;
    double rangeMin = 0.0;
    double rangeMax = 0.0;
    std::vector<float> ranges;
};

// 雷达在世界中的位姿：安装实体的位姿 × 安装位姿；装在世界上时即安装位姿
terra::data::Pose lidarWorldPose(const terra::data::PhysicsSnapshot& snapshot, const terra::scene::LidarSensor& lidar);

// 第 i 束在扫描平面中的角度（rad）
double lidarBeamAngle(const terra::scene::LidarSensor& lidar, int beam);

LidarScan scanLidar(const terra::physics::PhysicsWorld& world, const terra::scene::LidarSensor& lidar);

}  // namespace terra::simulation
