#pragma once

// 策略控制器（M4-10a，设计稿第 3 节）。每一步物理推进之前调用 commands(快照)，返回这一步的执行器命令：
//   - 降频：控制周期 ÷ 物理步长（必须接近整数）；从开始（或重置）起每隔这么多步推理一次，其余步保持上一次的目标；
//   - 观测（用户的决定：从带噪声的读数取）：IMU 读数先经 M4-09 的 SensorNoise（与 Headless、查看器、ROS2 等出口逐位相同），
//     并按 IMU 的采样频率保持（采样步之间用上一次的读数）；关节位置、速度取快照（M4-09 没有编码器噪声）；
//   - 历史帧：最旧的在前；开始或重置后的第一次推理把这一帧重复填满历史；
//   - 相位：第 i 次推理（从 0 起）的相位 = (i mod n) / n，n = round(周期 ÷ 控制周期)；指令幅值不大于阈值时 sin、cos 都为 0；
//   - 动作 positionPd：目标 = offset + scale × 输出；每一步 τ = kp (目标 − q) − kd q̇，截断到 ±torqueLimit，
//     再换算为执行器的控制量（÷ 增益 × 传动比），截断到控制范围（与 M3-04b 的 PD 层相同）；
//     position、velocity：控制量 = offset + scale × 输出；torque：力矩 = offset + scale × 输出（同样截断）；
//   - lastAction 观测项是上一次推理的原始输出（缩放前）。
// 世界重置（步序号变小）时自动从头开始；也可以调用 reset()。
// 推理失败时不再输出命令（failed() 为真，error() 给出原因），由界面停止策略。

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "terra/data/model_description.hpp"
#include "terra/data/state.hpp"
#include "terra/physics/physics_world.hpp"
#include "terra/policy/backend.hpp"
#include "terra/policy/policy.hpp"
#include "terra/scene/scene.hpp"
#include "terra/simulation/sensor_noise.hpp"

namespace terra::policy {

class PolicyController {
public:
    // 按场景绑定关节、执行器、传感器与实体；noise 为运行时的噪声开关（--no-noise、查看器面板）。失败时返回空并写 error
    static std::unique_ptr<PolicyController> create(PolicyDescription policy, std::unique_ptr<PolicyModel> model,
                                                    const terra::scene::SceneInstance& scene, bool noise, std::string& error);

    std::vector<terra::physics::ActuatorCommand> commands(const terra::data::PhysicsSnapshot& snapshot);
    void reset() noexcept { started_ = false; }
    // 运行时的噪声开关（查看器的传感器面板）；与 SensorNoise::setEnabled 相同
    void setNoise(bool on) noexcept { noise_.setEnabled(on); }

    // 速度指令（vx、vy 机身坐标系 m/s，偏航角速度 rad/s），截断到说明文件的范围
    void setCommand(const std::array<double, 3>& command);
    const std::array<double, 3>& command() const noexcept { return command_; }

    const PolicyDescription& policy() const noexcept { return policy_; }
    const PolicyModel& model() const noexcept { return *model_; }
    int decimation() const noexcept { return decimation_; }
    std::vector<terra::data::ActuatorId> actuators() const;  // 由策略接管的执行器

    // 运行信息（界面、测试）
    std::uint64_t inferences() const noexcept { return inferences_; }
    double lastInferenceSeconds() const noexcept { return lastInferenceSeconds_; }
    const std::vector<float>& lastFrame() const noexcept { return frame_; }     // 最近一帧观测
    const std::vector<float>& lastOutput() const noexcept { return output_; }  // 最近一次推理的原始输出
    const std::vector<double>& targets() const noexcept { return targets_; }   // 当前的动作目标（offset + scale × 输出）
    bool failed() const noexcept { return failed_; }
    const std::string& error() const noexcept { return error_; }

    // 由快照算一帧观测（不推进历史，测试用）；stepIndex 用于噪声
    std::vector<float> observe(const terra::data::PhysicsSnapshot& snapshot);

private:
    PolicyController() = default;

    struct Channel {
        std::size_t joint = 0;  // JointId − 1
        terra::data::ActuatorId actuator;
        double scale = 1.0, offset = 0.0, kp = 0.0, kd = 0.0, torqueLimit = 0.0;
        double actuatorScale = 1.0;  // 增益 × 传动比
        bool limited = false;
        double min = 0.0, max = 0.0;
    };
    struct Term {
        ObservationTerm spec;
        std::vector<std::size_t> joints;  // JointId − 1
        int sensor = -1;                  // ModelDescription::sensors 的下标
        int entity = -1;                  // ModelDescription::entities 的下标
    };

    void readSensors(const terra::data::PhysicsSnapshot& snapshot);
    double control(const Channel& c, double target, double q, double qd) const;

    PolicyDescription policy_;
    std::unique_ptr<PolicyModel> model_;
    terra::simulation::SensorNoise noise_;
    std::vector<double> sensorRates_;
    std::vector<std::vector<double>> held_;  // 每个 IMU 最近一次采样的带噪声读数
    std::vector<bool> heldValid_;
    double timestep_ = 0.0;
    int decimation_ = 1;
    int phaseSteps_ = 0;
    std::vector<Channel> channels_;
    std::vector<Term> terms_;

    std::array<double, 3> command_{0.0, 0.0, 0.0};
    bool started_ = false;
    std::uint64_t start_ = 0;
    std::uint64_t lastStep_ = 0;
    std::uint64_t episodeInferences_ = 0;
    std::vector<float> history_;  // historyLength × frameWidth，最旧的在前
    std::vector<float> frame_, output_;
    std::vector<double> targets_;
    std::uint64_t inferences_ = 0;
    double lastInferenceSeconds_ = 0.0;
    bool failed_ = false;
    std::string error_;
};

// 用 overrides 中的命令替换 base 中同一执行器的命令，其余保留（策略在 PD 层、动作之后，同一执行器以策略为准）
std::vector<terra::physics::ActuatorCommand> overrideCommands(std::vector<terra::physics::ActuatorCommand> base,
                                                              const std::vector<terra::physics::ActuatorCommand>& overrides);

}  // namespace terra::policy
