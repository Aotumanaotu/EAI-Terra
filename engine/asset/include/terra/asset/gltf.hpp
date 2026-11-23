#pragma once

// Terra 资产：glTF 2.0 加载（M1-09，D-013）。纯 CPU：解析文件、读取缓冲区与贴图图像，产出 Terra 类型的网格、材质与实例。
//
// 坐标：glTF 为 Y 向上、米；Terra 为 Z 向上（D-012）。实例变换 = 绕 X 轴 +90°（(x, y, z) → (x, −z, y)）× 节点世界矩阵；
// 顶点保持在网格自身坐标中，由实例变换换算（法线、切线在着色器中随之变换）。
// 支持：.gltf（外部 .bin 与图像）与 .glb（内嵌缓冲区与图像）；节点层级；三角形图元；位置、法线（缺失时按面积加权生成）、
//       UV0、切线（缺失时生成）；8/16/32 位索引；pbrMetallicRoughness、法线、遮蔽、自发光贴图；双面；PNG 与 JPEG。
// 不支持的特性给出诊断：extensionsRequired 中的扩展为错误（不加载），extensionsUsed 中的为警告（忽略该特性继续加载）；
// 动画、蒙皮、非三角形图元、透明混合等同为警告。贴图缺失为警告，对应槽位不使用贴图。
// 公共接口只用 Terra 类型；cgltf 只在实现中使用。

#include <array>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "terra/render/image.hpp"
#include "terra/render/mesh.hpp"
#include "terra/render/pbr.hpp"
#include "terra/render/scene_renderer.hpp"

namespace terra::asset {

struct Diagnostic {
    enum class Severity { Info, Warning, Error };  // Info：按约定做了推断（如 ARM 遮蔽），不是问题
    Severity severity = Severity::Warning;
    std::string message;
};

struct AssetImage {
    std::string uri;   // glTF 中的写法（内嵌图像为空）
    std::string path;  // 解析后的完整路径（内嵌图像为 "<文件>#images[i]"）
    bool srgb = false;  // 用作基础色或自发光贴图时按 sRGB 解码
    bool missing = false;
    terra::render::ImageRgba8 pixels;
};

struct AssetMaterial {
    std::string name;
    terra::render::Material factors;  // 因子与开关；贴图字段为 0，由 GpuAsset 填入纹理名称
    int baseColorImage = -1;          // AssetImage 下标，-1 表示无
    int metallicRoughnessImage = -1;
    int normalImage = -1;
    int occlusionImage = -1;
    int emissiveImage = -1;
    // M1-10：材质没有声明遮蔽贴图、但金属度-粗糙度贴图按 ARM 约定打包（文件名含 "_arm"，R = 环境遮蔽）时，
    // occlusionImage 指向同一图像，并标记为来自 ARM 约定（渲染时可关闭）。
    bool occlusionFromArm = false;
};

struct AssetPrimitive {
    terra::render::MeshData mesh;  // glTF 网格坐标（Y 向上）
    int material = -1;             // AssetMaterial 下标，-1 表示默认材质
};

struct AssetMesh {
    std::string name;
    std::vector<AssetPrimitive> primitives;
};

struct AssetInstance {
    int mesh = -1;
    std::string node;
    terra::render::Matrix4 transform{};  // 网格坐标 → Terra 世界坐标（含 Y→Z 换算）
    int nodeIndex = -1;                  // 所在节点（Asset::nodes 下标，M2-07）
    terra::render::Matrix4 nodeWorld{};  // 节点在 glTF 中的世界矩阵（Y 向上，不含换算）
};

// glTF 节点（M2-07：按节点绑定到物理实体）。下标与 glTF 文件中的 nodes 数组相同。
struct AssetNode {
    std::string name;
    int parent = -1;
    terra::render::Matrix4 world{};  // glTF 中的世界矩阵（Y 向上）
};

// 绑定节点时的对齐方式（M2-01 设计第 6 节）：Node——以节点自身坐标为准（去掉节点的世界变换）；
// Assembly——保留节点在 glTF 中的世界变换（整体装配位置）。
enum class NodeFrame { Node, Assembly };

struct Asset {
    std::string source;
    std::vector<AssetImage> images;
    std::vector<AssetMaterial> materials;
    std::vector<AssetMesh> meshes;
    std::vector<AssetInstance> instances;
    std::vector<AssetNode> nodes;
    std::array<float, 3> boundsMin{0.0f, 0.0f, 0.0f};  // Terra 世界坐标（Z 向上）
    std::array<float, 3> boundsMax{0.0f, 0.0f, 0.0f};
    std::size_t triangleCount() const noexcept;
};

struct LoadResult {
    std::optional<Asset> asset;  // 有错误时为空
    std::vector<Diagnostic> diagnostics;
    bool hasErrors() const noexcept;
    std::string summary() const;  // 所有诊断，每条一行
};

struct LoadOptions {
    // Poly Haven 等资产库的 ARM 打包约定（R = AO、G = 粗糙度、B = 金属度）；glTF 规范本身不要求，默认启用推断。
    bool armOcclusion = true;
};

LoadResult loadGltf(const std::string& path, const LoadOptions& options = {});
// 按名字或 "#下标" 查找节点（M2-07）。找不到或名字重复时返回 -1 并在 error 中说明原因。
int findNode(const Asset& asset, const std::string& nameOrIndex, std::string& error);

// node 是否在 root 的子树中（含 root 本身）
bool inSubtree(const std::vector<AssetNode>& nodes, int node, int root);

// 绑定框架的基准矩阵：Node 对齐时为 C × 节点世界⁻¹，Assembly 对齐（或 root 为 -1）时为 C
terra::render::Matrix4 bindingBase(const std::vector<AssetNode>& nodes, int root, NodeFrame frame);

// 节点子树中每个实例相对绑定框架的变换（M2-07）：返回 (实例下标, 网格坐标 → 绑定框架)。
// 绑定框架为 Terra 坐标（Z 向上）：Node 对齐时为 C × 节点世界⁻¹ × 实例世界；Assembly 对齐时为 C × 实例世界（C 为 Y→Z 换算）。
// root 为 -1 时取全部实例（整个文件）。
std::vector<std::pair<int, terra::render::Matrix4>> subtreeTransforms(const Asset& asset, int root, NodeFrame frame);

// 仿射矩阵求逆（线性部分可含缩放与剪切，不可奇异；奇异时返回单位矩阵）
terra::render::Matrix4 invertAffine(const terra::render::Matrix4& m);

// 文件名（不含目录）是否符合 ARM 约定：含 "_arm"（不区分大小写），如 "metal_toolbox_arm_2k.jpg"。
bool looksLikeArmTexture(const std::string& uriOrPath);

// glTF（Y 向上）→ Terra（Z 向上）：绕 X 轴 +90°。
terra::render::Matrix4 yUpToZUp();
terra::render::Matrix4 multiply(const terra::render::Matrix4& a, const terra::render::Matrix4& b);

}  // namespace terra::asset
