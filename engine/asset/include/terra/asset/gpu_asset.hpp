#pragma once

// Terra 资产：上传到 GPU 的 glTF 资产（M1-09）。贴图（含 mip）与网格各上传一次，可按不同摆放生成多组渲染项。
// 需要当前 OpenGL 上下文；只通过 terra_render 的接口创建 GL 资源。

#include <vector>

#include "terra/asset/gltf.hpp"
#include "terra/render/mesh.hpp"
#include "terra/render/resources.hpp"
#include "terra/render/scene_renderer.hpp"

namespace terra::asset {

class GpuAsset {
public:
    GpuAsset() = default;
    static GpuAsset create(const Asset& asset);

    // placement：整个资产在 Terra 世界中的摆放（先应用资产自身的实例变换）。
    // armOcclusion 为假时，不使用按 ARM 约定推断出的遮蔽贴图（显式声明的遮蔽贴图不受影响）。
    std::vector<terra::render::RenderItem> items(const terra::render::Matrix4& placement, bool armOcclusion = true) const;
    // 节点子树（M2-07）：root 为 -1 时取整个文件。模型矩阵 = placement × bindingBase(root, frame) × 实例的 glTF 世界矩阵。
    // exclude（M5-02b）：不画这些节点的子树（节点下标）
    std::vector<terra::render::RenderItem> subtreeItems(const terra::render::Matrix4& placement, int root, NodeFrame frame,
                                                        bool armOcclusion = true, const std::vector<int>& exclude = {}) const;
    // 性能（2026-11-26）：先算一次子树中的图元下标（subtreeParts），每帧只按下标生成渲染项（partItems）；结果与 subtreeItems 相同
    std::vector<int> subtreeParts(int root, const std::vector<int>& exclude = {}) const;
    std::vector<terra::render::RenderItem> partItems(const terra::render::Matrix4& placement, int root, NodeFrame frame, bool armOcclusion,
                                                     const std::vector<int>& parts) const;

    int textureCount() const noexcept { return static_cast<int>(textures_.size()); }
    int meshCount() const noexcept { return static_cast<int>(meshes_.size()); }

private:
    struct Part {
        int mesh = -1;  // meshes_ 下标
        terra::render::Material material;
        terra::render::Matrix4 transform{};
        bool occlusionFromArm = false;
        int node = -1;                        // 所在节点（M2-07）
        terra::render::Matrix4 nodeWorld{};   // 节点的 glTF 世界矩阵
    };
    std::vector<AssetNode> nodes_;
    std::vector<terra::render::Texture2D> textures_;
    std::vector<terra::render::GpuMesh> meshes_;
    std::vector<Part> parts_;
};

}  // namespace terra::asset
