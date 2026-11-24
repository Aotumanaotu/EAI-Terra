#pragma once

// PD 控制层（M3-04b，M3-04 设计稿第 3 节，2026-11-24 评审通过：方案 A）。
//
// 场景文件 controllers 中列出的力矩执行器由本层接管：动作与面板给出的是关节的**目标位置**（rad 或 m，范围为关节限位），
// 本层每一步按当前关节状态算出力矩，再换算成执行器的控制量：
//   τ = kp (q* − q) − kd q̇            （关节空间的力或力矩）
//   u = τ / (增益 × 传动比)，截断到执行器的控制范围
// 还没有目标的执行器（场景刚打开、重置之后）以当时的关节位置为目标，即保持当前姿态。
// 只依赖快照中的关节状态与模型描述，与物理后端无关；查看器与 Headless 共用。

#include <cstdint>
#include <map>
#include <optional>
#include <utility>
#include <vector>

#include "terra/data/model_description.hpp"
#include "terra/data/state.hpp"
#include "terra/physics/physics_world.hpp"
#include "terra/scene/scene.hpp"
#include "terra/simulation/motion.hpp"

namespace terra::simulation {

class PdLayer {
public:
    PdLayer() = default;
    // assignments 来自 SceneInstance::pd（已校验：作用于单个转动 / 滑动关节的单输入力矩执行器）
    PdLayer(const terra::data::ModelDescription& model, const std::vector<terra::scene::PdAssignment>& assignments);

    bool empty() const noexcept { return channels_.empty(); }
    bool manages(terra::data::ActuatorId actuator) const noexcept { return channels_.count(actuator.value) != 0; }
    std::size_t size() const noexcept { return channels_.size(); }

    // 目标位置的范围：关节有限位时为限位，否则不限制。不由本层接管的执行器返回控制范围。
    std::optional<std::pair<double, double>> targetRange(const terra::data::ActuatorDescription& actuator) const;
    TargetRange targetRangeFunction() const;

    void setTarget(terra::data::ActuatorId actuator, double target);
    std::optional<double> target(terra::data::ActuatorId actuator) const;
    // 重置之后调用：清除全部目标（下一步起以当时的关节位置为目标）
    void clearTargets() noexcept;

    // 处理一步的命令：commands 中属于本层的执行器视为目标位置（更新目标、不直接下发），其余原样保留；
    // 再按 state 为本层全部执行器追加力矩控制量。
    std::vector<terra::physics::ActuatorCommand> process(std::vector<terra::physics::ActuatorCommand> commands,
                                                         const terra::data::PhysicsSnapshot& state);

    // 单个通道的控制量（供测试与面板显示）：目标 target、关节位置 q、速度 qd
    double control(terra::data::ActuatorId actuator, double target, double q, double qd) const;

private:
    struct Channel {
        terra::data::ActuatorId actuator;
        std::size_t joint = 0;  // JointId − 1
        double kp = 0.0;
        double kd = 0.0;
        double scale = 1.0;  // 增益 × 传动比
        bool limited = false;
        double min = 0.0;
        double max = 0.0;
        std::optional<std::pair<double, double>> jointRange;
        std::optional<double> target;
    };
    std::map<std::uint32_t, Channel> channels_;  // 按 ActuatorId
};

}  // namespace terra::simulation
