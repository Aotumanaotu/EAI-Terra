#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "terra/data/geometry.hpp"
#include "terra/data/ids.hpp"

namespace terra::data {

// 模型来源。M0 只支持 MJCF 文件；文件内的相对资源路径以模型文件所在目录为基准。
struct ModelSource {
    std::string mjcfPath;
};

enum class JointType { Free, Ball, Slide, Hinge };

struct EntityDescription {
    EntityId id;
    std::string name;  // 模型中未命名时为空
    EntityId parent;   // 父实体；直接挂在世界上时为无效 ID
};

struct JointDescription {
    JointId id;
    std::string name;  // 模型中未命名时为空
    JointType type = JointType::Hinge;
    EntityId entity;  // 关节所属实体
    // 配置量与速度量的维度不一定相同：自由关节 7/6，球关节 4/3，滑动与转动关节 1/1。
    int configurationSize = 0;
    int velocitySize = 0;
};

// 模型中固定的仿真配置，用于记录和复现实验（开发计划 M0-05 ①）。
// 名称为后端无关的文字描述；未知取值记为 "unknown(<编号>)"。
struct SimulationSettings {
    Vec3 gravity;             // m/s²
    std::string integrator;   // 如 "Euler"、"RK4"、"implicit"、"implicitfast"、"discrete"
    std::string solver;       // 如 "PGS"、"CG"、"Newton"
    int iterations = 0;       // 主求解器最大迭代次数
    double tolerance = 0.0;   // 主求解器收敛容差
};

// 已加载模型的只读描述。entities[i].id.value == i + 1，joints 同理。
struct ModelDescription {
    std::string sourcePath;  // 解析后的模型文件绝对路径
    double timestep = 0.0;   // 固定物理步长（秒）
    SimulationSettings settings;
    std::vector<EntityDescription> entities;
    std::vector<JointDescription> joints;

    const EntityDescription* entity(EntityId id) const noexcept {
        return id.valid() && id.value <= entities.size() ? &entities[id.value - 1] : nullptr;
    }
    const JointDescription* joint(JointId id) const noexcept {
        return id.valid() && id.value <= joints.size() ? &joints[id.value - 1] : nullptr;
    }
    // 按名称查找；空名称不参与查找。
    const EntityDescription* findEntity(std::string_view name) const noexcept {
        if (name.empty()) return nullptr;
        for (const auto& e : entities) {
            if (e.name == name) return &e;
        }
        return nullptr;
    }
    const JointDescription* findJoint(std::string_view name) const noexcept {
        if (name.empty()) return nullptr;
        for (const auto& j : joints) {
            if (j.name == name) return &j;
        }
        return nullptr;
    }
};

}  // namespace terra::data
