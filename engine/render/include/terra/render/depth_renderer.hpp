#pragma once

// Terra Render：线性深度（M4-03，M4-01 设计稿第 3 节）。相机传感器的深度图：每个像素为命中点沿光轴的距离（米），
// 即相机坐标系中的 −z（不是到光心的直线距离），单精度浮点，不经过深度缓冲的非线性映射。
//
// 只画不透明的渲染项（opacity < 1 的半透明叠加不进入深度）；不做多重采样（每个像素只取像素中心）。没有命中、或命中点
// 在近、远平面之外时为 +inf（ROS REP-117 的约定）。行序为图像约定：行 0 为图像顶部。需要当前 OpenGL 上下文。

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "terra/render/scene_renderer.hpp"

namespace terra::render {

struct DepthRendererResult;

class DepthRenderer {
public:
    static DepthRendererResult create(RenderContext& context);
    ~DepthRenderer();
    DepthRenderer(const DepthRenderer&) = delete;
    DepthRenderer& operator=(const DepthRenderer&) = delete;

    // 渲染并读回 width × height 个深度值（米）。尺寸变化时重建内部目标；结束时恢复默认帧缓冲。
    std::vector<float> render(int width, int height, const ViewParams& view, const std::vector<RenderItem>& items);

    // 已创建的离屏目标数（0 或 1）；用于检查“不启用相机时不创建任何离屏目标”
    int targetCount() const noexcept;

private:
    struct Impl;
    explicit DepthRenderer(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

struct DepthRendererResult {
    std::unique_ptr<DepthRenderer> renderer;
    std::optional<std::string> error;
};

}  // namespace terra::render
