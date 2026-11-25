#pragma once

// 传感器噪声（M4-09，设计稿第 2 节）。真值（物理与渲染给出的读数）之上按场景文件的配置加噪声，带种子、可复现。
//
// 随机数是计数器式的（设计稿问题 3）：每个随机数由 (种子, 传感器全名, 步序号, 分量) 唯一决定，与调用顺序、程序无关。
// 只有 IMU 的零偏随机游走带状态（逐个采样步累加）；它也只取决于步序号，所以各出口（Headless、查看器、terra_capture、
// ROS2 桥接、实验记录与复现）各自持有一个 SensorNoise 也得到逐位相同的读数——同一时刻只有一份带噪声的读数（设计稿 2.3 节）。
// 步序号变小（重置、按关键帧重置）时随机游走从初始状态重新累加。
//
// 关闭（用户评审时的要求）：传感器的 noise.enabled、场景的 noise.enabled、运行时的开关（--no-noise、查看器面板）任一处关闭即为真值。

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "terra/data/state.hpp"
#include "terra/scene/scene.hpp"
#include "terra/simulation/lidar.hpp"

namespace terra::simulation {

// 计数器式随机数：SplitMix64 混合 (种子, 流, 步序号, 分量)
std::uint64_t noiseBits(std::uint64_t seed, std::uint64_t stream, std::uint64_t step, std::uint64_t index) noexcept;
double noiseUniform(std::uint64_t seed, std::uint64_t stream, std::uint64_t step, std::uint64_t index) noexcept;  // [0, 1)
double noiseGaussian(std::uint64_t seed, std::uint64_t stream, std::uint64_t step, std::uint64_t index) noexcept; // 标准正态
std::uint64_t noiseStream(const std::string& sensorName) noexcept;  // 传感器名 → 流编号（FNV-1a）

class SensorNoise {
public:
    SensorNoise() = default;
    // 按场景实例的配置；enabled 为运行时开关（--no-noise、查看器面板）
    explicit SensorNoise(const terra::scene::SceneInstance& scene, bool enabled = true);

    // 是否有任何传感器会加噪声（场景与运行时都开着，且至少一个传感器配置了噪声）
    bool active() const noexcept;
    bool enabled() const noexcept { return runtimeEnabled_; }
    void setEnabled(bool on) noexcept { runtimeEnabled_ = on; }
    std::uint64_t seed() const noexcept { return seed_; }

    // IMU：k 为模型描述中的传感器下标；values 为 6 个分量（比力 xyz、角速度 xyz），原地加噪声
    void imu(std::size_t k, std::uint64_t step, std::vector<double>& values);
    // 快照中全部 IMU 读数加噪声（记录、发布时用）；返回副本
    terra::data::PhysicsSnapshot withNoise(const terra::data::PhysicsSnapshot& snapshot);
    // 2D 激光雷达：k 为 SceneInstance::lidars 的下标；scan.step 为采样的步序号
    void lidar(std::size_t k, terra::simulation::LidarScan& scan) const;
    // 相机：k 为 SceneInstance::cameras 的下标；深度（米，inf 为没有命中）与彩色（每像素 channels 个字节）
    void depth(std::size_t k, std::uint64_t step, std::vector<float>& depth) const;
    void color(std::size_t k, std::uint64_t step, std::vector<std::uint8_t>& pixels, int channels) const;

    // 每个 IMU 的采样频率与步长（随机游走只在采样步上累加）
private:
    struct ImuState {
        terra::scene::ImuNoise config;
        std::uint64_t stream = 0;
        double rateHz = 0.0;
        bool hasLast = false;
        std::uint64_t last = 0;            // 已累加到的步序号
        std::array<double, 6> walk{};      // 随机游走的零偏（比力 xyz、角速度 xyz）
        std::array<double, 6> initial{};   // 初始零偏
    };
    bool imuActive(const ImuState& s) const noexcept;
    void advance(ImuState& s, std::uint64_t step);

    std::uint64_t seed_ = 0;
    bool sceneEnabled_ = true;
    bool runtimeEnabled_ = true;
    double timestep_ = 0.0;
    std::vector<ImuState> imus_;
    std::vector<std::pair<terra::scene::LidarNoise, std::uint64_t>> lidars_;   // 配置、流编号
    std::vector<std::pair<terra::scene::CameraNoise, std::uint64_t>> cameras_;
};

}  // namespace terra::simulation
