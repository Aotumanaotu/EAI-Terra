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

struct SceneDocument {
    std::string path;       // 场景文件（绝对路径；从文本解析时为调用方给出的路径）
    std::string directory;  // 相对路径的基准目录
    std::string name;
    PhysicsOverrides physics;
    std::vector<SceneModel> models;  // mjcf 已展开为相对场景目录的路径
    std::vector<VisualBindingSpec> visuals;
    ViewSpec view;
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
