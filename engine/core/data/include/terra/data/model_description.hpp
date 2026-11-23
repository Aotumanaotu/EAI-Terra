#pragma once

#include <array>
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
    std::string name;  // 模型中未命名时为空；场景组合的世界中为带实例前缀的全名（M2-04）
    EntityId parent;   // 父实体；直接挂在世界上时为无效 ID
    ModelInstanceId modelInstance;  // 所属模型实例（M2-03 起）
};

// 模型实例（M2-03）。直接加载单个 MJCF 时只有一个实例：名字为空（不加前缀），放置位姿为单位变换。
struct ModelInstanceDescription {
    ModelInstanceId id;
    std::string name;
    std::string sourcePath;  // MJCF 文件绝对路径
    Pose placement;          // 实例在世界中的放置位姿
};

// 几何体形状（M2-03）。Other：高度场、SDF 等暂不显示的形状。
enum class GeomShape { Plane, Sphere, Capsule, Ellipsoid, Cylinder, Box, Mesh, Other };

// 几何体描述（M2-03，M2-01 设计第 5 节）。size 的含义（米）：
//   Plane：x、y 为半边长（0 表示无限大），z 为网格线间距
//   Sphere：x 为半径
//   Capsule、Cylinder：x 为半径，y 为沿局部 Z 轴的半长
//   Ellipsoid：三个半轴；Box：三个半边长
//   Mesh：不使用（网格顶点已含尺寸）
// local 为相对所属实体原点（body 坐标系，不是质心）的位姿；静态几何（entity 无效）相对世界。
// 世界位姿 = 实体位姿 × local；几何体不进入快照，随实体移动。
struct GeomDescription {
    GeomId id;
    std::string name;  // 模型中未命名时为空
    EntityId entity;   // 无效 ID 表示世界上的静态几何（如地面）
    ModelInstanceId modelInstance;
    GeomShape shape = GeomShape::Other;
    Vec3 size;
    Pose local;
    // 颜色：几何体自己的颜色；用了材质且几何体颜色为 MJCF 默认值（0.5, 0.5, 0.5, 1）时取材质颜色
    // （与 MuJoCo 自带查看器的规则相同）。
    std::array<float, 4> rgba{0.5f, 0.5f, 0.5f, 1.0f};
    int group = 0;          // MJCF group；默认显示 0、1、2 组
    bool collides = false;  // 参与碰撞（contype 或 conaffinity 非 0）
    int mesh = -1;          // Mesh 形状时为 ModelDescription::meshes 下标，否则 -1
};

// 网格（M2-03）：编译后的顶点（网格自身坐标，已含缩放）与三角形，供显示。
// MuJoCo 编译时可能把网格平移、旋转到其惯性主轴，GeomDescription::local 已相应补偿，二者相乘即为原始摆放。
struct MeshDescription {
    std::string name;
    std::vector<Vec3> vertices;
    std::vector<std::array<std::uint32_t, 3>> triangles;
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

// 执行器的传动方式。Joint：直接作用于一个关节；Other：肌腱、site、body 等，M0 只描述、不解释。
enum class ActuatorTransmission { Joint, Other };

// 执行器的一个控制输入。limited 为真时，合法取值为闭区间 [min, max]。
struct ControlInput {
    std::string name;  // 输入名称（如四元数输入的 "qw"）；普通标量输入通常为空
    bool limited = false;
    double min = 0.0;
    double max = 0.0;
};

// 执行器描述。控制值的物理含义（力、力矩、目标位置、目标速度等）由模型中的执行器定义，
// Terra 不分类、不标单位（M0-07 报告 3.3 节）。一个执行器可以有多个控制输入。
struct ActuatorDescription {
    ActuatorId id;
    std::string name;  // 模型中未命名时为空
    ActuatorTransmission transmission = ActuatorTransmission::Other;
    JointId joint;  // transmission == Joint 时为目标关节，否则为无效 ID
    std::vector<ControlInput> inputs;
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

// 已加载模型的只读描述。entities[i].id.value == i + 1，joints、actuators、modelInstances、geoms 同理。
struct ModelDescription {
    // 描述的修订号：1 为 M0；2 为 M2-03（增加模型实例、几何体与网格，已有字段不变）。
    static constexpr std::uint32_t kRevision = 2;

    // 世界实例号：每次成功加载分配一个新值，进程内唯一、从 1 递增；0 无效。
    // 实体、关节、执行器 ID 只在同一个世界实例内有意义；重新加载后旧 ID 的数值可能相同，
    // 消费者应比较 worldInstance 来判断 ID 或快照是否属于当前世界（M0-08）。
    std::uint64_t worldInstance = 0;
    std::string sourcePath;  // 解析后的模型文件绝对路径
    double timestep = 0.0;   // 固定物理步长（秒）
    SimulationSettings settings;
    std::vector<EntityDescription> entities;
    std::vector<JointDescription> joints;
    std::vector<ActuatorDescription> actuators;
    std::vector<ModelInstanceDescription> modelInstances;
    std::vector<GeomDescription> geoms;  // 按后端编译顺序；含静态几何
    std::vector<MeshDescription> meshes;

    const EntityDescription* entity(EntityId id) const noexcept {
        return id.valid() && id.value <= entities.size() ? &entities[id.value - 1] : nullptr;
    }
    const JointDescription* joint(JointId id) const noexcept {
        return id.valid() && id.value <= joints.size() ? &joints[id.value - 1] : nullptr;
    }
    const ActuatorDescription* actuator(ActuatorId id) const noexcept {
        return id.valid() && id.value <= actuators.size() ? &actuators[id.value - 1] : nullptr;
    }
    const ModelInstanceDescription* modelInstance(ModelInstanceId id) const noexcept {
        return id.valid() && id.value <= modelInstances.size() ? &modelInstances[id.value - 1] : nullptr;
    }
    const GeomDescription* geom(GeomId id) const noexcept {
        return id.valid() && id.value <= geoms.size() ? &geoms[id.value - 1] : nullptr;
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
    const ActuatorDescription* findActuator(std::string_view name) const noexcept {
        if (name.empty()) return nullptr;
        for (const auto& a : actuators) {
            if (a.name == name) return &a;
        }
        return nullptr;
    }
    const GeomDescription* findGeom(std::string_view name) const noexcept {
        if (name.empty()) return nullptr;
        for (const auto& g : geoms) {
            if (g.name == name) return &g;
        }
        return nullptr;
    }
};

}  // namespace terra::data
