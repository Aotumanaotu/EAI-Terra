#pragma once

// 场景的渲染适配（M2-07，M2-01 设计第 5、6 节）：把 terra_scene 的视觉部件与其世界位姿转换为 terra_render 的渲染项。
//   - MJCF 几何体：按形状生成程序网格（同形状同尺寸共用一个网格）；颜色取几何体颜色（按 sRGB 换算为线性），
//     透明度近 0 的几何体不显示（MJCF 常用 rgba 的 alpha = 0 表示隐藏）；平面只接收阴影。
//   - glTF 绑定：每个文件加载一次；按节点（或 "#下标"）取子树，node / assembly 两种对齐（terra::asset::subtreeItems）。
//     文件读不出或节点不存在时给出提示并放弃该绑定，所属实体改为显示 MJCF 几何体。
//   - 显示规则：默认显示 group 0、1、2（与 MuJoCo 自带查看器一致）；实体有可用的 glTF 绑定且都没有要求 showGeoms 时，
//     隐藏其 MJCF 几何体；可叠加显示碰撞几何体（不论分组）。
// 需要当前 OpenGL 上下文。不依赖界面库；查看器、参考图测试共用，保证画面一致。

#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "demo_scene.hpp"
#include "terra/asset/gltf.hpp"
#include "terra/asset/gpu_asset.hpp"
#include "terra/render/mesh.hpp"
#include "terra/render/scene_renderer.hpp"
#include "terra/scene/scene.hpp"

namespace terra::viewer {

struct SceneViewOptions {
    std::array<bool, 6> groups{true, true, true, false, false, false};  // MJCF group 0–5 是否显示
    bool collision = false;     // 叠加显示所有参与碰撞的几何体
    bool armOcclusion = true;   // glTF 资产的 ARM 遮蔽约定（M1-10）
};

// 位姿与缩放 → 模型矩阵（列主序，单精度）
terra::render::Matrix4 poseMatrix(const terra::data::Pose& pose, double scale = 1.0);
// MJCF 颜色（sRGB 0–1）→ 线性
float srgbToLinear(float c);

class SceneView {
public:
    // 为场景准备 GPU 资源。问题以 SceneMessage（AssetWarning / AssetInfo）返回。
    static std::unique_ptr<SceneView> create(const terra::scene::SceneInstance& scene, std::vector<SceneMessage>& messages);

    std::vector<terra::render::RenderItem> items(const terra::scene::SceneInstance& scene,
                                                 const std::vector<terra::scene::VisualPose>& poses,
                                                 const SceneViewOptions& options = {}) const;

    std::size_t meshCount() const noexcept { return meshes_.size(); }
    std::size_t liveBindings() const noexcept;

private:
    struct GeomDraw {
        const terra::render::GpuMesh* mesh = nullptr;  // 为空：不显示（透明或不支持的形状）
        terra::render::Matrix4 shape{};                // 形状缩放（如椭球的三个半轴），在几何体位姿之后
        terra::render::Material material;
        bool castsShadow = true;
        int group = 0;
        bool collides = false;
    };
    struct BindingDraw {
        const terra::asset::GpuAsset* asset = nullptr;  // 为空：绑定已放弃
        int node = -1;
        terra::asset::NodeFrame frame = terra::asset::NodeFrame::Node;
    };

    const terra::render::GpuMesh* mesh(const std::string& key, const terra::render::MeshData& data);

    std::map<std::string, std::unique_ptr<terra::render::GpuMesh>> meshes_;
    std::map<std::string, std::unique_ptr<terra::asset::GpuAsset>> assets_;
    std::vector<GeomDraw> geoms_;        // 按 GeomId − 1
    std::vector<BindingDraw> bindings_;  // 按 SceneInstance::bindings 下标
    std::vector<bool> hideGeomsOf_;      // 按 EntityId − 1：有可用绑定且都不要求 showGeoms
};

}  // namespace terra::viewer
