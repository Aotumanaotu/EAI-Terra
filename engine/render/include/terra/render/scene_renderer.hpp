#pragma once

// Terra Render：场景渲染器（M1-08a，D-027）。
//
// 每帧：太阳光阴影图（单张，PCF 3×3）→ 不透明物体 PBR 着色（RGBA16F，可多重采样）→ 地面网格（着色器绘制，
// 1 m / 10 m 两级、随距离淡出，X 轴红、Y 轴绿）→ 多重采样解析 → 曝光与色调映射 → sRGB 编码写入 RGBA8 输出纹理。
// 输出纹理可直接交给界面显示或回读。公共接口只用 Terra 类型；矩阵为列主序 float[16]（与 terra::view 相同）。

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "terra/render/mesh.hpp"
#include "terra/render/pbr.hpp"

namespace terra::render {

class RenderContext;

using Matrix4 = std::array<float, 16>;  // 列主序

Matrix4 identityMatrix();
// 先缩放、再平移（不含旋转）。
Matrix4 translationScale(Vec3f translation, Vec3f scale = {1.0f, 1.0f, 1.0f});

struct RenderItem {
    const GpuMesh* mesh = nullptr;
    Matrix4 model = identityMatrix();
    Material material;
    bool castsShadow = true;  // 地面等只接收阴影的物体设为 false，阴影图只覆盖投影物体，精度更高
};

struct ViewParams {
    Matrix4 view{};
    Matrix4 projection{};
    Vec3f cameraPosition;
};

struct RenderSettings {
    ToneMapper toneMapper = ToneMapper::AgX;
    float exposureEv = 0.0f;
    int msaaSamples = 4;  // 1、2、4、8；超过驱动上限时降到上限
    bool shadows = true;
    int shadowMapSize = 2048;
    bool grid = true;
    Vec3f background{0.02f, 0.022f, 0.026f};  // 线性辐射
};

struct FrameStats {
    int samples = 1;          // 实际使用的采样数
    int shadowCasters = 0;    // 本帧投射阴影的物体数
    bool shadowsDrawn = false;
};

struct SceneRendererResult;

class SceneRenderer {
public:
    static SceneRendererResult create(RenderContext& context);
    ~SceneRenderer();
    SceneRenderer(const SceneRenderer&) = delete;
    SceneRenderer& operator=(const SceneRenderer&) = delete;

    // 渲染到内部目标（尺寸变化时自动重建）；结束时恢复默认帧缓冲，关闭面剔除、混合与多边形偏移。
    void render(int width, int height, const ViewParams& view, const std::vector<RenderItem>& items, const SunLight& sun,
                const HemisphereLight& environment, const RenderSettings& settings);

    std::uint32_t outputTexture() const noexcept;  // RGBA8，内容为 sRGB 编码；尚未渲染时为 0
    int width() const noexcept;
    int height() const noexcept;
    const FrameStats& lastFrame() const noexcept;
    // 回读输出纹理（行序自下而上）。
    std::vector<std::uint8_t> readOutput(int x, int y, int width, int height) const;

private:
    struct Impl;
    explicit SceneRenderer(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

struct SceneRendererResult {
    std::unique_ptr<SceneRenderer> renderer;
    std::optional<std::string> error;  // 着色器编译或链接失败的诊断
};

}  // namespace terra::render
