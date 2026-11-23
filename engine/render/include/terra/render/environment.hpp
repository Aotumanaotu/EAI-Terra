#pragma once

// Terra Render：基于图像的光照环境（M1-08b，D-027）。
//
// 由一张等距柱状 HDR 图像在 GPU 上预计算（创建时一次完成）：
//   环境立方体贴图   cubeSize²，带完整 mip 链（背景显示与预过滤的源）
//   辐照度立方体贴图 irradianceSize²：余弦加权半球积分（均匀角度网格），漫反射用
//   预过滤立方体贴图 prefilterSize²，prefilterMips 级：第 m 级对应粗糙度 m / (级数 − 1)，GGX 重要性采样（按概率密度选源 mip）
//   DFG 查找表       dfgSize²：在 CPU 上用 pbr::dfgLut 计算后上传，测试可用同一份数据算参考值
// 立方体贴图的方向即世界方向（Z 向上），按 OpenGL 立方体贴图的面约定生成，查找时只需旋转（环境旋转角）。

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "terra/render/hdr_image.hpp"

namespace terra::render {

class RenderContext;

struct EnvironmentOptions {
    int cubeSize = 512;
    int irradianceSize = 16;
    int prefilterSize = 128;
    int prefilterMips = 6;
    int prefilterSamples = 64;
    int dfgSize = 64;
    int dfgSamples = 256;
};

struct EnvironmentResult;

class Environment {
public:
    static EnvironmentResult create(RenderContext& context, const HdrImage& image, const EnvironmentOptions& options = {});
    ~Environment();
    Environment(const Environment&) = delete;
    Environment& operator=(const Environment&) = delete;

    std::uint32_t cubeTexture() const noexcept;
    std::uint32_t irradianceTexture() const noexcept;
    std::uint32_t prefilteredTexture() const noexcept;
    std::uint32_t dfgTexture() const noexcept;
    int cubeMipCount() const noexcept;
    int prefilteredMipCount() const noexcept;
    const std::vector<float>& dfgLut() const noexcept;
    int dfgSize() const noexcept;
    double precomputeSeconds() const noexcept;  // 预计算耗时（含等待 GPU 完成）

private:
    struct Impl;
    explicit Environment(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

struct EnvironmentResult {
    std::unique_ptr<Environment> environment;
    std::optional<std::string> error;
};

}  // namespace terra::render
