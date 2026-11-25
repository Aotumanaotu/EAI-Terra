#pragma once

// Terra 场景（M2-05，M2-01 设计第 3、4、6 节）：场景文件、场景实例（物理世界 + 视觉部件映射）、视觉部件的世界位姿。
//
// 场景文件是 JSON（D-102）：format "terra.scene"、version 1；models 为模型实例，visuals 为 glTF 绑定，physics 与 view 可选。
// 未知字段一律报错；诊断位置用 JSON 指针（如 "/visuals/2/entity"）。路径相对场景文件所在目录。
//
// 本模块不解析 glTF（Headless 不需要 GL 与 terra_asset）：只记录绑定、检查文件存在；节点是否存在、网格与材质加载由
// 查看器的场景适配层（M2-07）负责。视觉部件的世界位姿在这里用双精度计算：
//   MJCF 几何体：T_世界_几何 = T_世界_实体 × T_实体_几何
//   glTF 绑定： T_世界_视觉 = T_世界_实体 × O × T_局部 × S(scale)
//     O 为对齐：frame = "node" 时为单位变换；frame = "assembly" 时为 T_世界_实体(初始)⁻¹，使整体装配在初始姿态下
//     与 glTF 中的摆放重合。渲染侧再乘 glTF 的 Y→Z 换算与节点变换（node：节点子树相对节点；assembly：节点世界变换）。
// 静态部件（属于世界）的 T_世界_实体 为单位变换。

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "terra/data/geometry.hpp"
#include "terra/data/ids.hpp"
#include "terra/data/state.hpp"
#include "terra/physics/physics_world.hpp"
#include "terra/scene/composition.hpp"

namespace terra::scene {

enum class VisualFrame { Node, Assembly };

struct VisualBindingSpec {
    std::string entity;    // 实体全名；"world" 表示静态装饰
    std::string gltf;      // 解析后的路径（相对场景文件目录已展开）
    std::string node;      // 空表示整个文件；"#下标" 表示按下标
    terra::data::Pose local;
    double scale = 1.0;
    VisualFrame frame = VisualFrame::Node;
    bool showGeoms = false;
    std::string where;  // JSON 指针，如 "/visuals/0"
};

struct ViewSpec {
    std::optional<std::string> environment;
    std::optional<terra::data::Vec3> cameraPosition;
    std::optional<terra::data::Vec3> cameraTarget;
};

// PD 控制层（M3-04，设计稿第 3 节）：把动作与面板给出的目标位置换算成力矩执行器的控制量
//   τ = kp (q* − q) − kd q̇，控制量 = τ / (增益 × 传动比)，再截断到控制范围
// actuators 为执行器全名；"<实例名>/*" 表示该实例的全部执行器。只接受作用于单个转动 / 滑动关节的力矩执行器。
struct PdControllerSpec {
    std::vector<std::string> actuators;
    double kp = 0.0;
    double kd = 0.0;
    std::string where;  // "/controllers/<i>"
};

// 解析后的 PD 通道（instantiate 中按模型描述校验并展开通配）
struct PdAssignment {
    terra::data::ActuatorId actuator;
    double kp = 0.0;
    double kd = 0.0;
};

// 场景文件中的传感器（M4-02，M4-01 设计稿第 2 节）。type 为 "imu"、"camera"（M4-03）或 "lidar2d"（M4-03b）。
// IMU：
//   - 装在模型中已有的安装点上：写 site（安装点全名）；name 可省略，写了必须与 site 相同（IMU 的名字就是安装点名）；
//   - 装在实体上：写 entity（实体全名）与可选的 position / rotation（实体坐标系）；name 必填，带同一实例的前缀。
//   rateHz 省略时等于物理频率（每步采样）；高于物理频率时按物理频率并给出警告。
// 相机（M4-03）：
//   - 用 MJCF 中的相机：写 camera（相机全名）；name 可省略（取相机名）；视场角默认取 MJCF 中的值；
//   - 装在实体上：写 entity 与可选的 position / rotation；相机看向自身 −Z、+Y 向上（与 MJCF 相同）；name 必填。
//   width、height（默认 640 × 480，MJCF 写了分辨率时取它）、fovyDeg（竖直视场角，默认 60 或 MJCF 中的值）、
//   near、far（默认 0.05、50 m）、rateHz（默认 30）、rgb、depth（默认都输出）。
//   displayRotationDeg（M4-04，默认 0）：只影响显示，图像逆时针转 0、90、180 或 270° 后显示；数据（图像、深度、内参）不变，
//   与真机一致（如 Stretch 3 的头部相机是横着装的，真机图像同样需要转 90° 才正）。
// 2D 激光雷达（M4-03b）：装在实体上（entity、position、rotation），name 必填；在安装坐标系的 XY 平面内扫描，角度从 +X 起
//   逆时针为正（与 sensor_msgs/LaserScan 一致）。angleMinDeg、angleMaxDeg（默认 −180、180）、samples（默认 360，
//   第一束在 angleMin，最后一束在 angleMax）、rangeMin、rangeMax（默认 0.15、12 m）、rateHz（默认 10）、
//   hits（"collision" 默认，只打参与碰撞的几何体；"visual" 只打默认显示的几何体）。
struct SensorSpec {
    std::string type;
    std::string name;
    std::string site;
    std::string entity;
    terra::data::Pose pose;
    std::optional<double> rateHz;
    std::string where;  // "/sensors/<i>"
    // 相机（M4-03）
    std::string camera;
    std::optional<int> width;
    std::optional<int> height;
    std::optional<double> fovyDegrees;
    double nearClip = 0.05;
    double farClip = 50.0;
    bool rgb = true;
    bool depth = true;
    int displayRotationDegrees = 0;  // M4-04：只影响显示
    // 2D 激光雷达（M4-03b）
    double angleMinDegrees = -180.0;
    double angleMaxDegrees = 180.0;
    int samples = 360;
    double rangeMin = 0.15;
    double rangeMax = 12.0;
    bool hitVisual = false;
};

// 启用的 2D 激光雷达（M4-03b）。读数由物理侧的射线检测给出（不需要图形，Headless 也可用）。
struct LidarSensor {
    std::string name;
    terra::data::EntityId entity;  // 无效 ID 表示装在世界上
    terra::data::Pose mount;       // 相对实体原点；在安装坐标系的 XY 平面内扫描
    double angleMin = 0.0;         // rad
    double angleMax = 0.0;         // rad
    int samples = 360;
    double rangeMin = 0.15;        // m
    double rangeMax = 12.0;        // m
    double rateHz = 10.0;          // 不高于物理频率
    bool hitVisual = false;
    std::string where;
};

// 启用的相机传感器（M4-03）。图像由有图形上下文的程序渲染（查看器、terra_capture）；Headless 不渲染。
struct CameraSensor {
    std::string name;
    terra::data::EntityId entity;  // 无效 ID 表示装在世界上
    terra::data::Pose mount;       // 相对实体原点；相机看向自身 −Z，+Y 向上
    int width = 640;
    int height = 480;
    double fovyDegrees = 60.0;
    double nearClip = 0.05;
    double farClip = 50.0;
    double rateHz = 30.0;  // 不高于物理频率
    bool rgb = true;
    bool depth = true;
    int displayRotation = 0;  // M4-04：显示时逆时针旋转的角度（0、90、180、270），数据不变
    std::string where;
};

// 传感器的采样配置（与 ModelDescription::sensors 一一对应）
struct SensorConfig {
    terra::data::SensorId sensor;
    double rateHz = 0.0;  // 实际采用的频率（Hz），不高于物理频率
};

struct SceneDocument {
    std::string path;       // 场景文件（绝对路径；从文本解析时为调用方给出的路径）
    std::string directory;  // 相对路径的基准目录
    std::string name;
    std::string title;  // M4-08c：资源库卡片上的短标题（可选；空时显示 name）
    PhysicsOverrides physics;
    std::vector<SceneModel> models;  // mjcf 已展开为相对场景目录的路径
    std::vector<VisualBindingSpec> visuals;
    ViewSpec view;
    // M3-02：载入后从这个关键帧开始（全名，如 "g1/stand"），重置时也回到它；空表示模型初值
    std::string initialKeyframe;
    std::vector<PdControllerSpec> controllers;  // M3-04
    std::vector<SensorSpec> sensors;            // M4-02
};

struct ParseResult {
    std::optional<SceneDocument> document;  // 有错误时为空
    std::vector<SceneIssue> issues;
};

// 解析场景文件文本。scenePath 用于展开相对路径与诊断（不读取文件）。
ParseResult parseScene(const std::string& text, const std::string& scenePath);
// 读取并解析场景文件。
ParseResult readSceneFile(const std::string& path);

// 视觉部件（M2-01 设计第 4 节）：先排 MJCF 几何体（按 GeomId），再排 glTF 绑定（按场景文件顺序）。
struct VisualPart {
    enum class Kind { Geom, Gltf };
    terra::data::VisualId id;
    Kind kind = Kind::Geom;
    terra::data::EntityId entity;  // 无效表示静态（属于世界）
    terra::data::ModelInstanceId modelInstance;  // glTF 绑定到 "world" 时无效
    terra::data::GeomId geom;      // Kind::Geom
    int binding = -1;              // Kind::Gltf：SceneInstance::bindings 下标
    bool hiddenByBinding = false;  // 所属实体绑定了 glTF 且没有要求 showGeoms 时，MJCF 几何体默认隐藏
};

struct BoundVisual {
    VisualBindingSpec spec;
    terra::data::EntityId entity;  // 无效表示 "world"
    terra::data::Pose alignment;   // O：node 为单位变换；assembly 为 T_世界_实体(初始)⁻¹
};

struct SceneInstance {
    SceneDocument document;
    std::unique_ptr<terra::physics::PhysicsWorld> world;
    std::vector<BoundVisual> bindings;
    std::vector<VisualPart> visuals;
    std::optional<std::size_t> homeKeyframe;  // 场景的初始关键帧（已应用）；重置时回到它
    std::vector<PdAssignment> pd;             // M3-04：PD 层接管的执行器（按 ActuatorId 排序）
    std::vector<SensorConfig> sensors;        // M4-02：MJCF 中原有的与场景文件添加的传感器，与模型描述的顺序相同
    std::vector<CameraSensor> cameras;        // M4-03：场景文件中启用的相机，按场景文件的顺序
    std::vector<LidarSensor> lidars;          // M4-03b：场景文件中的 2D 激光雷达，按场景文件的顺序

    const VisualPart* visual(terra::data::VisualId id) const noexcept {
        return id.valid() && id.value <= visuals.size() ? &visuals[id.value - 1] : nullptr;
    }
};

struct SceneLoadResult {
    std::optional<SceneInstance> scene;  // 有错误时为空
    std::vector<SceneIssue> issues;      // 错误与警告
    bool ok() const noexcept { return scene.has_value(); }
};

// 由已解析的场景文件创建场景实例：组合物理世界（composeWorld）并建立视觉部件映射。
SceneLoadResult instantiate(SceneDocument document, const WorldFactory& factory);
// readSceneFile + instantiate
SceneLoadResult loadScene(const std::string& path, const WorldFactory& factory);

struct VisualPose {
    terra::data::VisualId id;
    terra::data::Pose pose;
    double scale = 1.0;
};

// 快照中的实体位姿，按 EntityId 下标（value − 1）
std::vector<terra::data::Pose> entityPoses(const terra::data::PhysicsSnapshot& snapshot);

// 每个视觉部件的世界位姿（含被隐藏的部件；显示与否由调用方按 hiddenByBinding 与几何体分组决定）。
// entityPoses 可以是快照中的位姿，也可以是插值后的位姿（M2-06）。
std::vector<VisualPose> visualPoses(const SceneInstance& scene, const std::vector<terra::data::Pose>& entityPoses);

}  // namespace terra::scene
