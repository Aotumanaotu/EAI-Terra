#pragma once

// 只依赖 Terra Physics 公共接口的调用方（M0-07 ⑥）。
// 本库只链接 terra::physics（仅头文件），构建时没有 MuJoCo 头文件路径；能编译即证明调用方不需要 MuJoCo。

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "terra/physics/physics_world.hpp"

namespace terra::test {

struct RunSummary {
    std::uint64_t steps = 0;
    double time = 0.0;
    std::vector<terra::data::EntityState> entities;  // 最后一个观测时刻
    std::vector<terra::data::JointState> joints;
    double maxQuaternionNormError = 0.0;  // 全程各实体姿态四元数范数与 1 的最大偏差
    bool allFinite = true;
};

// 按执行器名称构造控制命令；名称不存在时返回空并写入 error。
std::optional<std::vector<terra::physics::ActuatorCommand>> commandsByName(
    const terra::data::ModelDescription& model,
    const std::vector<std::pair<std::string, std::vector<double>>>& named, std::string& error);

// 设置一次控制，然后推进 steps 步并汇总快照；任何错误返回空并写入 error。
std::optional<RunSummary> runWithControl(terra::physics::PhysicsWorld& world,
                                         const std::vector<terra::physics::ActuatorCommand>& commands, int steps,
                                         std::string& error);

}  // namespace terra::test
