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

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "terra/data/geometry.hpp"
#include "terra/data/ids.hpp"
#include "terra/data/state.hpp"
#include "terra/physics/physics_world.hpp"
#include "terra/scene/composition.hpp"
#include "terra/scene/element.hpp"
#include "terra/scene/thermal_spec.hpp"

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
    // M5-11：传感器的外观模型（由预设给出，如 Mid-360 的外壳）；不影响所属实体的 MJCF 几何体是否隐藏
    bool sensorModel = false;
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
//   thermal（M5-06）：热像仪，见 ThermalCameraSpec；预设给出分辨率、视场角与帧率（相机显式写的优先），rgb 与 depth 可以都关闭。
//   displayRotationDeg（M4-04，默认 0）：只影响显示，图像逆时针转 0、90、180 或 270° 后显示；数据（图像、深度、内参）不变，
//   与真机一致（如 Stretch 3 的头部相机是横着装的，真机图像同样需要转 90° 才正）。
// 2D 激光雷达（M4-03b）：装在实体上（entity、position、rotation），name 必填；在安装坐标系的 XY 平面内扫描，角度从 +X 起
//   逆时针为正（与 sensor_msgs/LaserScan 一致）。angleMinDeg、angleMaxDeg（默认 −180、180）、samples（默认 360，
//   第一束在 angleMin，最后一束在 angleMax）、rangeMin、rangeMax（默认 0.15、12 m）、rateHz（默认 10）、
//   hits（"collision" 默认，只打参与碰撞的几何体；"visual" 只打默认显示的几何体）。
// 3D 激光雷达（M5-11）：type 为 "lidar3d"；装在实体上（entity、position、rotation），name 必填；preset 选
//   assets/sensors/lidar3d_presets.json 中的型号（如 "livox_mid360"），给出扫描图案、点频、帧率、量程与噪声；
//   rateHz、rangeMin、rangeMax、pointsPerSecond、hits、pattern（自定义扫描图案，相对场景文件）可覆盖预设；noise 写 {"enabled": false} 关闭，或覆盖各项；
//   motionDistortion（默认 true）：按每点的测量时刻发出射线（运动畸变）；
//   imu: true 时在预设给出的位置另装一个 IMU（名字为 <name>_imu，频率与噪声取预设，如 Mid-360 内置的 ICM40609）；
//   model（默认 true）：预设给出外观模型时，在安装位置显示（glTF，由 scripts/assets/fetch_sensor_models.sh 下载；没有时只提示）。
// ---- 传感器噪声（M4-09，设计稿第 2 节）----
// 不写即为 0（不加该项噪声）；enabled = false 时保留参数但不加（用户评审时的要求：噪声可以关闭）。
struct ImuNoiseAxis {
    double noiseDensity = 0.0;  // 白噪声密度（陀螺 rad/s/√Hz，加速度计 m/s²/√Hz）；单个样本 σ = 密度 × √频率
    double randomWalk = 0.0;    // 零偏随机游走（陀螺 rad/s²/√Hz，加速度计 m/s³/√Hz）；每个样本零偏走 σ × √Δt
    std::array<double, 3> bias{0.0, 0.0, 0.0};  // 固定的初始零偏
    double biasStd = 0.0;                       // 初始零偏另按此标准差随机取（与种子有关）
};

struct ImuNoise {
    bool configured = false;
    bool enabled = true;
    ImuNoiseAxis gyro;
    ImuNoiseAxis accel;
};

struct LidarNoise {
    bool configured = false;
    bool enabled = true;
    std::string preset;              // 来自 assets/sensors/lidar_presets.json（显式字段覆盖预设）
    double rangeStd = 0.0;           // 米
    double rangeStdRelative = 0.0;   // 与距离成正比的部分：σ = rangeStd + rangeStdRelative × 距离
    double dropout = 0.0;            // 每束丢失的概率（丢失为 inf）
    double resolution = 0.0;         // 距离量化（米）；0 为不量化
};

struct CameraNoise {
    bool configured = false;
    bool enabled = true;
    double depthStd = 0.0;           // 米
    double depthStdQuadratic = 0.0;  // σ = depthStd + depthStdQuadratic × z²（1/米）
    double dropout = 0.0;            // 深度像素丢失的概率（丢失为 inf）
    double pixelStd = 0.0;           // RGB 每个通道的高斯噪声（0–255）
};

// ---- 热成像（M5-06，M5-01 设计稿第 3 节）----
// TemperatureCurve、HotspotSpec 见 thermal_spec.hpp；thermal 段的说明见那里
struct ThermalSceneSpec {
    bool configured = false;
    double ambient = 20.0;
    double sky = -20.0;
    std::vector<std::pair<std::string, TemperatureCurve>> temperatures;  // 匹配写法 → 温度
    std::vector<HotspotSpec> hotspots;
};

enum class ThermalPalette { Ironbow, WhiteHot, BlackHot, Rainbow };
const char* thermalPaletteName(ThermalPalette palette) noexcept;

// 热像仪的噪声：每帧每像素 σ = netdK 的高斯（时间噪声）；每台相机一个固定偏差（测温精度），
//   偏差(T) = g × max(accuracyAbsolute, accuracyRelative × |T|) ÷ 2，g 为按种子取的标准正态数（精度按 2σ 理解）
struct ThermalNoise {
    bool configured = false;
    bool enabled = true;
    double netdK = 0.0;
    double accuracyAbsolute = 0.0;  // °C
    double accuracyRelative = 0.0;  // 读数（°C）的比例
};

// 相机的 thermal 块：preset（assets/sensors/thermal_presets.json）、gain（预设的增益档，决定量程与精度）、
//   palette（ironbow 默认、white-hot、black-hot、rainbow）、display（"auto" 按每帧的最低、最高温度，或 [下限, 上限] °C）、
//   noise（{"enabled": false} 关闭，或覆盖 netdK、accuracyAbsolute、accuracyRelative）
struct ThermalCameraSpec {
    bool enabled = false;
    std::string preset;
    std::string gain;
    double rangeMin = -273.15;  // 量程（°C）：辐射测温图超出时截断
    double rangeMax = 1000.0;
    ThermalPalette palette = ThermalPalette::Ironbow;
    bool autoDisplay = true;
    double displayMin = 0.0;  // autoDisplay 为 false 时的伪彩色区间（°C）
    double displayMax = 100.0;
    ThermalNoise noise;
};

// M5-11：3D 激光雷达的噪声。距离 σ 在 rangeNear（σ = rangeStdNear）与 rangeFar（σ = rangeStdFar）之间按距离线性插值，
// 两端之外取端点的值；方向另加 σ = angleStdDeg 的方位与俯仰扰动
struct Lidar3dNoise {
    bool configured = false;
    bool enabled = true;
    std::string preset;
    double rangeStdNear = 0.0;  // 米
    double rangeNear = 0.2;     // 米
    double rangeStdFar = 0.0;   // 米
    double rangeFar = 10.0;     // 米
    double angleStdDeg = 0.0;
};

// 场景级的噪声设置
struct NoiseSettings {
    std::uint64_t seed = 0;
    bool enabled = true;
};

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
    bool segmentation = false;       // M4-09：输出分割图（每个像素的实体 ID）
    // 2D 激光雷达（M4-03b）
    double angleMinDegrees = -180.0;
    double angleMaxDegrees = 180.0;
    int samples = 360;
    double rangeMin = 0.15;
    double rangeMax = 12.0;
    bool hitVisual = false;
    // M4-09：噪声（按类型取其一）
    ImuNoise imuNoise;
    LidarNoise lidarNoise;
    // 3D 激光雷达（M5-11）：rangeMin、rangeMax、hitVisual 与 2D 共用（预设的值先填入）
    std::string lidar3dPreset;
    std::string patternFile;         // 扫描图案（绝对路径）
    double pointsPerSecond = 0.0;
    int lines = 1;
    bool motionDistortion = true;
    Lidar3dNoise lidar3dNoise;
    ThermalCameraSpec thermal;      // M5-06：相机的热成像输出
    std::string modelFile;          // 外观模型（glTF，绝对路径；空表示不显示）
    terra::data::Pose modelPose;    // 外观模型在安装坐标系中的位姿（Mid-360：底面中心在雷达原点下方 47 mm）
    CameraNoise cameraNoise;
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
    LidarNoise noise;  // M4-09
    std::string where;
};

// 3D 激光雷达的扫描图案（M5-11）：依次发出的射线方向（安装坐标系，度）。天顶角从 +Z 起，方位角从 +X 起逆时针；
// 方向 = (sin 天顶 · cos 方位, sin 天顶 · sin 方位, cos 天顶)。按步依次取用，用完后从头循环（非重复扫描的实测图案）
struct Lidar3dPattern {
    std::string path;
    std::vector<float> azimuth;
    std::vector<float> zenith;
    std::size_t size() const noexcept { return azimuth.size(); }
};

// 读入扫描图案（CSV：表头一行，之后每行 “序号, 方位角（度）, 天顶角（度）”，即 Livox 官方仿真插件的格式）。
// 同一路径只读一次（进程内缓存）；失败时返回空并写 error
std::shared_ptr<const Lidar3dPattern> loadLidarPattern(const std::string& path, std::string& error);

// 启用的 3D 激光雷达（M5-11）：按扫描图案做物理侧的射线检测（不需要图形，Headless 也可用）
struct Lidar3dSensor {
    std::string name;
    terra::data::EntityId entity;  // 无效 ID 表示装在世界上
    terra::data::Pose mount;       // 相对实体原点；扫描图案在安装坐标系中
    std::string preset;
    std::shared_ptr<const Lidar3dPattern> pattern;
    double pointsPerSecond = 200000.0;
    double rateHz = 10.0;
    double rangeMin = 0.1;   // m：近于它的没有回波（盲区）
    double rangeMax = 40.0;  // m
    int lines = 4;
    bool hitVisual = false;
    // 运动畸变：真机一帧的点在 1 / 帧率 秒内依次测得。开启时每 1 ms 的射线按该时刻的安装位姿发出（由当前的线速度与角速度
    // 向前回推），点在各自时刻的安装坐标系中——与真机相同，供 LIO 等算法按每点时间去畸变。关闭时一帧在同一时刻发出
    bool motionDistortion = true;
    Lidar3dNoise noise;
    std::string where;
    // 每帧的点数 = 点频 ÷ 帧率（四舍五入）
    std::size_t pointsPerFrame() const noexcept;
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
    bool segmentation = false;  // M4-09：输出分割图
    CameraNoise noise;          // M4-09
    ThermalCameraSpec thermal;  // M5-06：输出温度图（与彩色、深度、分割像素对齐）
    std::string where;
};

// M5-06：实例化后的温度表。实体按 EntityId 下标（value − 1）；没有写温度的实体为空（取 ambient）
struct ThermalModel {
    struct Hotspot {
        terra::data::EntityId entity;  // 无效表示世界
        terra::data::Vec3 position{0, 0, 0};
        double radius = 0.1;
        TemperatureCurve temperature;
    };
    double ambient = 20.0;
    double sky = -20.0;
    std::vector<std::optional<TemperatureCurve>> entities;
    std::optional<TemperatureCurve> world;  // 不属于任何实体的静态几何体
    std::vector<Hotspot> hotspots;
    // 实体（无效 ID 为世界）在 time 时刻的底温（不含热点）
    double base(terra::data::EntityId entity, double time) const noexcept;
};

// 传感器的采样配置（与 ModelDescription::sensors 一一对应）
struct SensorConfig {
    terra::data::SensorId sensor;
    double rateHz = 0.0;  // 实际采用的频率（Hz），不高于物理频率
    ImuNoise noise;       // M4-09：场景文件中同名的 IMU 给出的噪声
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
    NoiseSettings noise;                        // M4-09：场景级的噪声设置（种子、总开关）
    // M4-09：分割图的语义类别：实体全名或模型实例名的通配（“go2/*”、“floor”）→ 类别名
    std::vector<std::pair<std::string, std::string>> labels;
    ThermalSceneSpec thermal;  // M5-06：温度
    std::vector<SceneElementSpec> elements;  // M5-02a：场景元素（组合时排在 models 之后）
};

struct ParseResult {
    std::optional<SceneDocument> document;  // 有错误时为空
    std::vector<SceneIssue> issues;
};

// 解析场景文件文本。scenePath 用于展开相对路径与诊断（不读取文件）。
ParseResult parseScene(const std::string& text, const std::string& scenePath);
// 读取并解析场景文件。
ParseResult readSceneFile(const std::string& path);

// M4-09：实体的语义类别（分割图的对照表）：实体全名完全匹配优先，其次模型实例名，其次以 * 结尾的前缀（最长的优先）；
// 都不匹配时为空
std::string sceneLabel(const SceneDocument& document, const std::string& entityName, const std::string& instanceName);
// 匹配写法的等级（labels 与 M5-06 的 thermal.temperatures 共用）：3 实体全名，2 实例名，1 以 * 结尾的前缀（length 为前缀长度），0 不匹配
int scenePatternRank(const std::string& pattern, const std::string& entityName, const std::string& instanceName, std::size_t& length);

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
    std::vector<Lidar3dSensor> lidars3d;      // M5-11：场景文件中的 3D 激光雷达，按场景文件的顺序
    ThermalModel thermal;                     // M5-06：场景的温度（热像仪用）

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

// M4-09：分割图的对照表（JSON 文本，一行）：{"format": "terra.segmentation-labels", "version": 1, "background": 0,
//   "labels": [{"id": 实体编号, "entity": 全名, "instance": 模型实例名, "label": 语义类别（场景文件 labels；没有时为空）}]}
std::string segmentationLabelsJson(const SceneInstance& scene);

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
