#pragma once

// 相机传感器的渲染（M4-03，M4-01 设计稿第 3～5 节）：按某一步的快照渲染相机看到的彩色图与线性深度图。
//
// - 彩色：与视口相同的 SceneRenderer（PBR、阴影、多重采样、色调映射），但不画地面网格；8 位 RGB，sRGB 编码；
// - 深度：DepthRenderer，沿光轴的距离（米），没有命中为 +inf；只含不透明物体（不含碰撞几何体的半透明叠加）；
// - 位姿与时间：取自给定的快照（采样那一步），不使用显示帧的插值姿态；位姿为 Terra 相机坐标系（看向 −Z，+Y 向上）；
// - 图像行 0 为顶部，像素按行存放。
// 需要当前 OpenGL 上下文。只在启用了相机时创建（不启用相机时不创建任何离屏目标）。

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "demo_scene.hpp"
#include "scene_view.hpp"
#include "terra/render/context.hpp"
#include "terra/render/depth_renderer.hpp"
#include "terra/render/image.hpp"
#include "terra/render/scene_renderer.hpp"
#include "terra/scene/camera_model.hpp"
#include "terra/scene/scene.hpp"

namespace terra::viewer {

struct CameraFrame {
    std::string sensor;
    std::uint64_t step = 0;
    double time = 0.0;
    std::uint64_t runEpoch = 0;
    terra::data::Pose pose;                // 相机在世界中的位姿（Terra 相机坐标系）
    terra::scene::CameraIntrinsics intrinsics;
    std::vector<std::uint8_t> rgb;         // width × height × 3；相机不输出彩色时为空
    std::vector<float> depth;              // width × height；相机不输出深度时为空
};

// 相机的观察与投影矩阵（列主序，单精度）
terra::render::ViewParams cameraViewParams(const terra::data::Pose& cameraWorld, const terra::scene::CameraSensor& camera);

class CameraRenderer {
public:
    // 着色器创建失败时返回空，error 为原因
    static std::unique_ptr<CameraRenderer> create(terra::render::RenderContext& context, std::string& error);

    // 渲染 camera 在 snapshot 那一步看到的画面。lighting 与视口相同（太阳、环境光、环境贴图、画质）；网格总是关闭。
    CameraFrame capture(const terra::scene::SceneInstance& scene, const SceneView& view, const terra::scene::CameraSensor& camera,
                        const terra::data::PhysicsSnapshot& snapshot, const FrameLighting& lighting);

    int depthTargets() const noexcept;

private:
    CameraRenderer() = default;
    std::unique_ptr<terra::render::SceneRenderer> color_;
    std::unique_ptr<terra::render::DepthRenderer> depth_;
};

// 深度的伪彩色预览：近处红、远处蓝（线性映射到 [near, maxDepth]），没有命中为黑色
terra::render::ImageRgba8 depthPreview(const std::vector<float>& depth, int width, int height, float nearDepth, float maxDepth);
// 彩色图转为 RGBA8 图像（便于写 PNG 与界面显示）
terra::render::ImageRgba8 colorImage(const CameraFrame& frame);

}  // namespace terra::viewer
