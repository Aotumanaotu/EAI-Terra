#include "terra/asset/gpu_asset.hpp"

#include <cstddef>

namespace terra::asset {

namespace r = terra::render;

GpuAsset GpuAsset::create(const Asset& asset) {
    GpuAsset g;
    for (const AssetImage& img : asset.images) {
        g.textures_.push_back(img.missing || img.pixels.empty()
                                  ? r::Texture2D{}
                                  : r::Texture2D::create(img.pixels.width, img.pixels.height, img.pixels.pixels.data(), img.srgb));
    }
    const auto texture = [&](int image) -> std::uint32_t {
        return image >= 0 && static_cast<std::size_t>(image) < g.textures_.size() ? g.textures_[static_cast<std::size_t>(image)].name() : 0u;
    };
    // 每个网格的每个图元上传为一个 GpuMesh；first[m] 为网格 m 的第一个图元在 meshes_ 中的下标
    std::vector<int> first;
    for (const AssetMesh& mesh : asset.meshes) {
        first.push_back(static_cast<int>(g.meshes_.size()));
        for (const AssetPrimitive& p : mesh.primitives) g.meshes_.push_back(r::GpuMesh::create(p.mesh));
    }
    for (const AssetInstance& inst : asset.instances) {
        if (inst.mesh < 0 || static_cast<std::size_t>(inst.mesh) >= asset.meshes.size()) continue;
        const AssetMesh& mesh = asset.meshes[static_cast<std::size_t>(inst.mesh)];
        for (std::size_t p = 0; p < mesh.primitives.size(); ++p) {
            const AssetPrimitive& prim = mesh.primitives[p];
            Part part;
            part.mesh = first[static_cast<std::size_t>(inst.mesh)] + static_cast<int>(p);
            part.transform = inst.transform;
            part.node = inst.nodeIndex;
            part.nodeWorld = inst.nodeWorld;
            if (prim.material >= 0 && static_cast<std::size_t>(prim.material) < asset.materials.size()) {
                const AssetMaterial& m = asset.materials[static_cast<std::size_t>(prim.material)];
                part.material = m.factors;
                part.material.baseColorTexture = texture(m.baseColorImage);
                part.material.metallicRoughnessTexture = texture(m.metallicRoughnessImage);
                part.material.normalTexture = texture(m.normalImage);
                part.material.occlusionTexture = texture(m.occlusionImage);
                part.material.emissiveTexture = texture(m.emissiveImage);
                part.occlusionFromArm = m.occlusionFromArm;
            } else {
                // glTF 默认材质：白色、金属度 1、粗糙度 1
                part.material.baseColor = {1.0f, 1.0f, 1.0f};
                part.material.metallic = 1.0f;
                part.material.roughness = 1.0f;
            }
            g.parts_.push_back(part);
        }
    }
    g.nodes_ = asset.nodes;
    return g;
}

std::vector<r::RenderItem> GpuAsset::items(const r::Matrix4& placement, bool armOcclusion) const {
    std::vector<r::RenderItem> out;
    out.reserve(parts_.size());
    for (const Part& part : parts_) {
        r::RenderItem item;
        item.mesh = &meshes_[static_cast<std::size_t>(part.mesh)];
        item.model = multiply(placement, part.transform);
        item.material = part.material;
        if (part.occlusionFromArm && !armOcclusion) item.material.occlusionTexture = 0;
        item.castsShadow = true;
        out.push_back(item);
    }
    return out;
}

std::vector<int> GpuAsset::subtreeParts(int root, const std::vector<int>& exclude) const {
    std::vector<int> out;
    for (std::size_t k = 0; k < parts_.size(); ++k) {
        const Part& part = parts_[k];
        if (root >= 0 && !inSubtree(nodes_, part.node, root)) continue;
        // M5-02b：排除的节点子树（元素的部件另有绑定）
        bool excluded = false;
        for (const int x : exclude) excluded = excluded || inSubtree(nodes_, part.node, x);
        if (!excluded) out.push_back(static_cast<int>(k));
    }
    return out;
}

std::vector<r::RenderItem> GpuAsset::subtreeItems(const r::Matrix4& placement, int root, NodeFrame frame,
                                                   bool armOcclusion, const std::vector<int>& exclude) const {
    return partItems(placement, root, frame, armOcclusion, subtreeParts(root, exclude));
}

std::vector<r::RenderItem> GpuAsset::partItems(const r::Matrix4& placement, int root, NodeFrame frame, bool armOcclusion,
                                               const std::vector<int>& parts) const {
    const r::Matrix4 base = multiply(placement, bindingBase(nodes_, root, frame));
    std::vector<r::RenderItem> out;
    out.reserve(parts.size());
    for (const int k : parts) {
        if (k < 0 || static_cast<std::size_t>(k) >= parts_.size()) continue;
        const Part& part = parts_[static_cast<std::size_t>(k)];
        r::RenderItem item;
        item.mesh = &meshes_[static_cast<std::size_t>(part.mesh)];
        item.model = multiply(base, part.nodeWorld);
        item.material = part.material;
        if (part.occlusionFromArm && !armOcclusion) item.material.occlusionTexture = 0;
        item.castsShadow = true;
        out.push_back(item);
    }
    return out;
}

}  // namespace terra::asset
