#pragma once

#include <array>
#include <cstddef>
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
    int material = -1;      // M3-03：ModelDescription::materials 下标，没有材质时为 -1（颜色已按上面的规则合成到 rgba）
};

// 网格（M2-03）：编译后的顶点（网格自身坐标，已含缩放）与三角形，供显示。
// MuJoCo 编译时可能把网格平移、旋转到其惯性主轴，GeomDescription::local 已相应补偿，二者相乘即为原始摆放。
// M3-03（描述修订 4）：显示用的法线与纹理坐标，按“三角形角”索引（与 triangles 一一对应、各自独立编号）：
//   第 i 个三角形第 k 个角的法线为 normals[triangleNormals[i][k]]，纹理坐标为 texcoords[triangleTexcoords[i][k]]。
// 法线为 MuJoCo 编译后的值（文件给出的法线，或按面积加权平均、去掉大折角面的贡献），已随网格一起变换到网格坐标。
// 纹理坐标沿用 MuJoCo 的约定：v 已翻转，(0, 0) 对应贴图第一行（图像顶部）。网格没有纹理坐标时两者为空。
struct MeshDescription {
    std::string name;
    std::vector<Vec3> vertices;
    std::vector<std::array<std::uint32_t, 3>> triangles;
    std::vector<Vec3> normals;
    std::vector<std::array<std::uint32_t, 3>> triangleNormals;
    std::vector<std::array<float, 2>> texcoords;
    std::vector<std::array<std::uint32_t, 3>> triangleTexcoords;
};

// 贴图（M3-03）。只有二维贴图带像素（RGBA8，行优先，第一行 = 纹理坐标 v = 0）；立方体与天空盒只记录尺寸，
// Terra 的天空与环境光来自查看器的环境设置，不使用模型中的天空盒。
enum class TextureKind { TwoD, Cube, Skybox };

struct TextureDescription {
    std::string name;
    TextureKind kind = TextureKind::TwoD;
    int width = 0;
    int height = 0;
    std::vector<std::uint8_t> rgba;  // TwoD：width × height × 4；其他为空
};

// 材质（M3-03），数值为 MJCF 原值（含默认值）。显示侧的换算见 M3-03 报告：
//   rgba 已合成到 GeomDescription::rgba；specular、shininess 为 MuJoCo 经典着色参数（0–1）；
//   metallic、roughness 为 −1 时表示模型没有给出；reflectance 是 MuJoCo 的镜面反射（平面倒影），Terra 不使用。
struct MaterialDescription {
    std::string name;
    std::array<float, 4> rgba{1.0f, 1.0f, 1.0f, 1.0f};
    float emission = 0.0f;  // 自发光 = emission × 颜色
    float specular = 0.5f;
    float shininess = 0.5f;
    float reflectance = 0.0f;
    float metallic = -1.0f;
    float roughness = -1.0f;
    int texture = -1;  // 基础色贴图（MuJoCo 的 RGB 角色），TextureDescription 下标
    std::array<float, 2> texRepeat{1.0f, 1.0f};
    bool texUniform = false;  // 重复次数按空间长度（每米）而不是按物体
};

struct JointDescription {
    JointId id;
    std::string name;  // 模型中未命名时为空
    JointType type = JointType::Hinge;
    EntityId entity;  // 关节所属实体
    // 配置量与速度量的维度不一定相同：自由关节 7/6，球关节 4/3，滑动与转动关节 1/1。
    int configurationSize = 0;
    int velocitySize = 0;
    // M3-02（描述修订 3）。单位：转动关节为弧度，滑动关节为米（D-012）。
    bool limited = false;  // 有限位时 rangeMin ≤ position[0] ≤ rangeMax（转动、滑动关节）
    double rangeMin = 0.0;
    double rangeMax = 0.0;
    Vec3 axis;    // 转动轴或滑动方向，所属实体坐标系中的单位向量（自由、球关节为 0）
    Vec3 anchor;  // 关节位置，所属实体坐标系（body 原点为参考）
    // 第一个自由度的被动参数：阻尼（N·m·s/rad 或 N·s/m）、电枢惯量、摩擦损失
    double damping = 0.0;
    double armature = 0.0;
    double frictionLoss = 0.0;
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
// 执行器类别（M3-02），由 MuJoCo 的增益 / 偏置类型判定：
//   Motor：力 = gain × 控制（控制量即力或力矩，经传动比 gear 作用到关节）
//   Position：力 = kp × (控制 − 位置) − kv × 速度（控制量是目标位置，单位同关节）
//   Velocity：力 = kv × (控制 − 速度)（控制量是目标速度）
//   Other：带动力学（滤波、积分等）或其他增益形式，只描述、不解释
enum class ActuatorKind { Motor, Position, Velocity, Other };

struct ActuatorDescription {
    ActuatorId id;
    std::string name;  // 模型中未命名时为空
    ActuatorTransmission transmission = ActuatorTransmission::Other;
    JointId joint;  // transmission == Joint 时为目标关节，否则为无效 ID
    std::vector<ControlInput> inputs;
    // M3-02（描述修订 3）
    ActuatorKind kind = ActuatorKind::Other;
    double gain = 0.0;    // Motor：gain；Position：kp；Velocity：kv
    double damping = 0.0; // Position：kv（速度阻尼）；其他为 0
    double gear = 1.0;    // 传动比（第一个分量）
    bool forceLimited = false;
    double forceMin = 0.0;  // 力或力矩限制（执行器输出，乘传动比之前）
    double forceMax = 0.0;
};

// 关键帧（M3-02）：模型中预设的状态（位置、速度、控制）。组合场景中名字带实例前缀，如 "g1/stand"。
// 按关键帧重置时恢复位置、速度、控制量与执行器内部状态；仿真时间仍从 0 起算，time 只作记录。
// 组合场景中，自由关节（浮动基座）的位姿按实例的放置位姿变换：关键帧里写的是相对放置框架的位姿。
struct KeyframeDescription {
    std::string name;
    double time = 0.0;  // 模型中写的关键帧时间（s），只作记录
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
    // 3 为 M3-02（关节限位、轴向与被动参数；执行器类别、增益、传动比与力限；关键帧）。
    static constexpr std::uint32_t kRevision = 4;

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
    std::vector<MaterialDescription> materials;  // M3-03
    std::vector<TextureDescription> textures;    // M3-03
    std::vector<KeyframeDescription> keyframes;  // 按模型中的顺序；下标即 PhysicsWorld::resetToKeyframe 的参数

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
    // 关键帧下标；没有时为 -1
    int findKeyframe(std::string_view name) const noexcept {
        for (std::size_t i = 0; i < keyframes.size(); ++i) {
            if (keyframes[i].name == name) return static_cast<int>(i);
        }
        return -1;
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
