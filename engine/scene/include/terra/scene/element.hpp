#pragma once

// 场景元素（M5-02a，M5-01 设计稿 2.1、2.2 节，2026-11-26 评审通过）：场景中可以被引用、移动、赋温度的一件东西，
// 如变压器、桌子。元素文件 *.terra-element.json：
//
//   {"format": "terra.element", "version": 1, "name": "主变压器 110kV", "category": "substation/transformer",
//    "visual": {"gltf": "transformer.glb", "node": "", "scale": 1.0, "position": [...], "rotation": {...}},
//    "collision": {"mode": "boxes", "boxes": [{"position": [x, y, z], "rotation": {...}, "halfSize": [hx, hy, hz]}, …],
//                  "generatedBy": "auto-boxes"},
//    "physics": {"movable": "layout" | "dynamic" | "fixed", "mass": 0, "friction": 1.0},
//    "thermal": {"default": 35.0, "hotspots": [{"position": [...], "radius": 0.1, "temperature": 60}]},
//    "inspection": [{"name": "油位计", "position": [...], "distance": 2.0, "heightDeg": 10}],
//    "license": {"source": "https://…", "license": "CC0-1.0", "author": "…"}}
//
// 坐标：元素坐标系 Z 向上、米；原点一般在底面中心（贴地时即放置位姿）。visual 的位姿与缩放只作用于显示。
// collision.mode：
//   - boxes：盒体列表（元素坐标系），由 M5-02b 的导入按 glTF 节点的包围盒生成（generatedBy 记来源），也可以手写或人工替换；
//   - mesh：一个碰撞网格文件（OBJ 或 STL，相对元素文件），MuJoCo 按凸包碰撞；可多个：meshes: [{file, position, rotation, scale}]；
//   - none：只显示，不碰撞（如地面贴花、天花板灯）。
// physics.movable：
//   - layout：只在布局编辑（M5-03）中移动，仿真时静止；
//   - fixed：不可移动（布局编辑中也锁定）；
//   - dynamic：自由刚体，机器人可以推动，需要 mass（kg，按各碰撞体的体积分配）。
// 运行时不解析 glTF（Headless 不需要图形）：碰撞体都写在元素文件中。
//
// 场景文件的 elements（2.2 节）：
//   "elements": [{"id": "T1", "element": "../elements/transformer.terra-element.json",
//                 "position": [12, 8.5, 0], "rotation": {"rpyDeg": [0, 0, 90]}, "thermal": {"default": 42.0}}]
//   - id 为实例名（同模型实例名的规则，不能与模型实例重名）；元素成为模型实例 id，刚体为 “<id>/body”；
//   - 实例的 thermal.default 覆盖元素的默认温度；场景 thermal.temperatures 中匹配到的写法优先于元素的默认温度；
//   - 分割图的类别：场景 labels 没有写时取元素的 category。

#include <optional>
#include <string>
#include <vector>

#include "terra/data/geometry.hpp"
#include "terra/scene/composition.hpp"
#include "terra/scene/thermal_spec.hpp"

namespace terra::scene {

struct ElementBox {
    terra::data::Pose pose;                  // 元素坐标系
    terra::data::Vec3 halfSize{0.1, 0.1, 0.1};
};

struct ElementMesh {
    std::string file;  // 绝对路径
    terra::data::Pose pose;
    double scale = 1.0;
};

enum class ElementCollision { Boxes, Mesh, None };
enum class ElementMovable { Layout, Fixed, Dynamic };

struct ElementInspection {
    std::string name;
    terra::data::Vec3 position{0, 0, 0};  // 元素坐标系
    double distance = 2.0;                // 推荐的拍摄距离（米）
    double heightDeg = 0.0;               // 推荐的俯仰（度）
};

struct ElementDocument {
    std::string path;       // 元素文件（绝对路径）
    std::string directory;  // 相对路径的基准
    std::string name;
    std::string category;
    // 显示
    std::string gltf;  // 绝对路径；空表示没有外观（显示碰撞体）
    std::string node;
    double visualScale = 1.0;
    terra::data::Pose visualPose;
    // 碰撞与物理
    ElementCollision collision = ElementCollision::Boxes;
    std::vector<ElementBox> boxes;
    std::vector<ElementMesh> meshes;
    std::string generatedBy;  // 碰撞体的来源（如 "auto-boxes"），只用于显示
    ElementMovable movable = ElementMovable::Layout;
    double mass = 0.0;
    double friction = 1.0;
    // 温度（M5-06）
    std::optional<TemperatureCurve> thermalDefault;
    std::vector<HotspotSpec> hotspots;  // entity 为空，位置在元素坐标系
    // 检查点（M5-07 使用）
    std::vector<ElementInspection> inspection;
    // 许可
    std::string licenseSource;
    std::string licenseName;
    std::string licenseAuthor;
};

struct ElementParseResult {
    std::optional<ElementDocument> document;  // 有错误时为空
    std::vector<SceneIssue> issues;           // where 为元素文件中的 JSON 指针
};

ElementParseResult parseElement(const std::string& text, const std::string& path);
ElementParseResult readElementFile(const std::string& path);

// 元素的刚体名（实例内）；实体全名为 “<id>/body”
inline constexpr const char* kElementBody = "body";

// 元素生成的 MJCF 文本：一个刚体（dynamic 时带自由关节），碰撞体为盒体或网格（凸包）；不含 option（全局设置随其他实例）。
// 碰撞体的颜色为半透明的灰（只在显示碰撞体时可见；有 glTF 外观时默认隐藏）
std::string elementMjcf(const ElementDocument& element);

// 场景中的一个元素实例
struct SceneElementSpec {
    std::string id;
    std::string file;  // 元素文件（绝对路径）
    terra::data::Pose placement;
    std::optional<TemperatureCurve> thermal;  // 实例覆盖的默认温度
    std::string where;                        // "/elements/<i>"
    ElementDocument element;                  // 解析场景时读入
};

}  // namespace terra::scene
