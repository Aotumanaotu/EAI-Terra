#pragma once

// Terra Render：HDR 环境贴图（M1-08b，D-027）。
//
// 等距柱状投影（equirectangular），坐标约定沿用 D-012（Z 向上）：
//   列 u ∈ [0, 1)：方位角 φ = atan2(y, x)，u = 0.5 + φ / 2π（u = 0.5 为 +X，u = 0.75 为 +Y）
//   行 v ∈ [0, 1]：v = acos(z) / π，第 0 行为天顶（+Z），最后一行为天底
// 像素为线性 RGB 辐射（Radiance .hdr 本身即线性，不做 gamma 处理）。纯 CPU，不需要 OpenGL。

#include <optional>
#include <string>
#include <vector>

#include "terra/render/pbr.hpp"

namespace terra::render {

struct HdrImage {
    int width = 0;
    int height = 0;
    std::vector<float> rgb;  // width × height × 3，行 0 为顶部（天顶）

    Vec3f at(int x, int y) const;  // 越界时按边缘取值
};

struct HdrResult {
    HdrImage image;     // 失败时为空（width = 0）
    std::string error;  // 失败原因（含路径）
    bool ok() const noexcept { return image.width > 0; }
};

// 读取 Radiance .hdr（stb_image，只编译 HDR 解码）。
HdrResult loadHdr(const std::string& path);

// 测试与占位用的合成环境：整幅同一颜色；上半球 / 下半球两种颜色；+Y / −Y 两侧两种颜色。
HdrImage solidHdr(int width, int height, Vec3f color);
HdrImage upperLowerHdr(int width, int height, Vec3f upper, Vec3f lower);
HdrImage leftRightHdr(int width, int height, Vec3f positiveY, Vec3f negativeY);

// ---- 环境贴图中的太阳（M1-09，让方向光与天空一致）----
// 判定：存在亮度不低于整幅平均亮度 50 倍的像素，且高于阈值 T = max(20 × 平均, 1% × 峰值) 的像素所占立体角
// 小于全球面的 0.5%（摄影棚的大面积灯箱不算太阳）。这些像素的亮度被压到 T（保持色度），被压掉的辐射积分为太阳：
//   辐照度 E = Σ (L − L_压后) dω（正入射，RGB），方向 = 按 (亮度 − T) dω 加权的平均方向。
struct SunEstimate {
    Vec3f direction;          // 指向太阳的单位向量（贴图坐标，未旋转）
    Vec3f irradiance;         // 线性 RGB，可直接作为方向光的颜色 × 强度
    float solidAngle = 0.0f;  // 太阳像素所占立体角，sr
    float peakLuminance = 0.0f;
};
// remove 为真时把太阳像素从图像中去掉（压到阈值），避免环境光照与方向光重复计算太阳。
std::optional<SunEstimate> extractSun(HdrImage& image, bool remove);
float luminance(Vec3f c);

// 贴图坐标 (u, v) 对应的世界方向（单位向量），约定见文件开头。
Vec3f equirectDirection(float u, float v);

}  // namespace terra::render
