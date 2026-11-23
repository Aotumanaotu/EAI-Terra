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

#include "terra/render/environment.hpp"
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

// 基于图像的光照（M1-08b）。environment 为空时使用半球环境光；不为空时环境光完全来自贴图（半球光不再参与）。
struct EnvironmentLighting {
    const Environment* environment = nullptr;
    float intensity = 1.0f;
    float rotationRadians = 0.0f;  // 绕世界 Z 轴旋转环境
    bool background = true;        // 视口背景显示环境贴图
    float backgroundBlur = 0.0f;   // 0–1：背景取环境立方体贴图的 mip（0 = 原图）
};

struct FrameStats {
    int samples = 1;          // 实际使用的采样数
    int shadowCasters = 0;    // 本帧投射阴影的物体数
    bool shadowsDrawn = false;
    bool imageLighting = false;  // 本帧使用了环境贴图光照
    bool backgroundDrawn = false;
};

// GPU 分阶段计时（M1-10）：每帧在各阶段之间写 OpenGL 时间戳。
enum class RenderPass { Shadow, Opaque, Background, Grid, Resolve, ToneMap };
constexpr int kRenderPassCount = 6;
const char* renderPassName(RenderPass pass) noexcept;  // "shadow" "opaque" "background" "grid" "resolve" "tonemap"

struct PassTimings {
    std::array<double, kRenderPassCount> milliseconds{};
    bool valid = false;
    double total() const noexcept;
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
    // 带基于图像的光照（M1-08b）：背景在不透明物体之后、网格之前绘制。
    void render(int width, int height, const ViewParams& view, const std::vector<RenderItem>& items, const SunLight& sun,
                const HemisphereLight& hemisphere, const RenderSettings& settings, const EnvironmentLighting& imageLighting);

    std::uint32_t outputTexture() const noexcept;  // RGBA8，内容为 sRGB 编码；尚未渲染时为 0
    int width() const noexcept;
    int height() const noexcept;
    const FrameStats& lastFrame() const noexcept;

    // ---- GPU 计时（M1-10）----
    // 开启后每帧写 7 个时间戳（3 帧轮换，不阻塞）；lastTimings 为最近一个结果已可用的帧（通常落后 1–2 帧）。
    void setTimingEnabled(bool enabled);
    bool timingEnabled() const noexcept;
    const PassTimings& lastTimings() const noexcept;
    // 等待本帧（最近一次 render）的计时结果（会阻塞直到 GPU 完成该帧），供基准程序逐帧记录。
    PassTimings waitForTimings();
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
