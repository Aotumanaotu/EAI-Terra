#pragma once

// 3D 激光雷达（M5-11，M5-01 设计稿第 4 节）：在当前观测时刻，按扫描图案从安装点发出一帧的射线，用物理世界的射线检测
// （PhysicsWorld::castRays）求距离。只依赖物理接口，Headless 与查看器共用；不需要图形。
//
// 帧与图案：第 f 帧（f = ⌊步序号 × 步长 × 帧率⌋，与采样时刻 sensorSampleDue 一致）取图案中第 f × 每帧点数 起的
// 每帧点数个方向，取到末尾时从头循环——与真机的非重复扫描相同（Mid-360：每帧 20,000 点，4 s 一个循环）。
// 每点的测量时刻 = frameStart + 序号 ÷ 点频，最后一点为当前时刻。motionDistortion 开启时（默认）射线按各自时刻的安装位姿发出
// （由当前速度回推，每 1 ms 一组），点在各自时刻的安装坐标系中，与真机相同；关闭时一帧在当前时刻发出。
// 世界中其他物体在这 0.1 s 内的运动不回推（只回推雷达自身的运动）。
// 输出只含有回波的点（与 livox_ros_driver2 相同）；近于 rangeMin（盲区）或超出 rangeMax 的方向没有点。
// 坐标：安装坐标系（米）。line = 图案中的序号对线数取余。

#include <cstdint>
#include <string>
#include <vector>

#include "terra/data/geometry.hpp"
#include "terra/data/state.hpp"
#include "terra/physics/physics_world.hpp"
#include "terra/scene/scene.hpp"

namespace terra::simulation {

struct Lidar3dPoint {
    float x = 0.0f, y = 0.0f, z = 0.0f;  // 安装坐标系，米
    float time = 0.0f;                   // 相对帧起点的扫描时间（秒）
    std::uint32_t ray = 0;               // 本帧中的射线序号（噪声的分量编号）
    std::uint8_t line = 0;
};

struct Lidar3dScan {
    std::string sensor;
    std::uint64_t step = 0;
    double time = 0.0;
    std::uint64_t runEpoch = 0;
    terra::data::Pose pose;      // 雷达安装坐标系在世界中的位姿
    std::uint64_t frame = 0;     // 帧序号
    double frameStart = 0.0;     // 第一点的测量时刻 = time − (每帧点数 − 1) ÷ 点频；点的 time 相对它
    std::size_t patternStart = 0;
    std::size_t rays = 0;        // 本帧发出的射线数
    double rangeMin = 0.0;
    double rangeMax = 0.0;
    std::vector<Lidar3dPoint> points;
};

// 采样步上的帧序号
std::uint64_t lidar3dFrame(std::uint64_t step, double timestep, double rateHz) noexcept;

terra::data::Pose lidar3dWorldPose(const terra::data::PhysicsSnapshot& snapshot, const terra::scene::Lidar3dSensor& lidar);

Lidar3dScan scanLidar3d(const terra::physics::PhysicsWorld& world, const terra::scene::Lidar3dSensor& lidar);

// 点的内容摘要（sha256，按 x、y、z、time、line 的小端字节）：Headless 输出与实验记录用来逐位比较
std::string lidar3dDigest(const Lidar3dScan& scan);

}  // namespace terra::simulation
