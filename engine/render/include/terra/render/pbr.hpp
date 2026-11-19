#pragma once

// Terra Render：基于物理的着色（PBR）与色调映射（M1-08，D-027）。
//
// 参考 Filament 文档中的标准材质模型：
//   镜面  Cook-Torrance：GGX 法线分布 D、高度相关 Smith-GGX 可见性 V、Schlick 菲涅耳 F；α = roughness²
//   漫反射  Lambert：diffuseColor / π，diffuseColor = baseColor · (1 − metallic)
//   F0 = mix(0.16 · reflectance², baseColor, metallic)（非金属 reflectance 0.5 → 4%）
//   太阳光：(Fd + Fr) · 辐照度 · N·L · 阴影；环境光（M1-08a）：半球光，漫反射按法线取色，镜面按反射方向取色
// 参数与 glTF 2.0 金属度-粗糙度材质一致。颜色为线性值。
//
// 本头文件中的 C++ 函数是着色器的参考实现：GLSL 由 shaderLibrary() 提供同一组公式，
// 测试用 C++ 结果检查 GPU 像素（D-022，llvmpipe）。修改公式时两处必须同步。

#include <array>
#include <cstdint>
#include <string>

namespace terra::render {

struct Vec3f {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Material {
    Vec3f baseColor{0.8f, 0.8f, 0.8f};  // 线性 RGB
    float metallic = 0.0f;
    float roughness = 0.5f;             // 感知粗糙度；着色时下限 kMinRoughness
    float reflectance = 0.5f;           // 非金属的镜面反射率参数
    Vec3f emissive{0.0f, 0.0f, 0.0f};   // 线性辐射，直接相加
};

// 太阳光（方向光）。direction 指向光源（单位向量）。
struct SunLight {
    Vec3f direction{0.0f, 0.0f, 1.0f};
    Vec3f color{1.0f, 1.0f, 1.0f};
    float intensity = 3.0f;  // 相对辐照度（与曝光一起决定亮度）
};

// 半球环境光：方向 d 的入射光 = mix(ground, sky, 0.5·d.z + 0.5) · intensity。
struct HemisphereLight {
    Vec3f sky{0.60f, 0.70f, 0.90f};
    Vec3f ground{0.30f, 0.27f, 0.24f};
    float intensity = 0.6f;
};

enum class ToneMapper {
    AgX,         // Blender 4 默认风格（AgX 最小近似）
    PbrNeutral,  // Khronos PBR Neutral：基础色还原最准确
    Aces,        // ACES 拟合（Narkowicz），电影感、对比较强
    None,        // 线性截断到 [0, 1]，仅调试
};

namespace pbr {

constexpr float kPi = 3.14159265358979f;
constexpr float kMinRoughness = 0.045f;

float distributionGgx(float NoH, float alpha);
float visibilitySmithGgxCorrelated(float NoV, float NoL, float alpha);
Vec3f fresnelSchlick(Vec3f f0, float VoH);
Vec3f hemisphere(const HemisphereLight& env, Vec3f direction);

// 表面一点的出射辐射（线性）。n、v 为单位向量（v 指向相机）；shadow ∈ [0, 1]，1 表示完全受光。
Vec3f shade(const Material& material, Vec3f n, Vec3f v, const SunLight& sun, const HemisphereLight& env, float shadow);

// 乘以 2^exposureEv 后色调映射，返回线性显示值 ∈ [0, 1]。
Vec3f toneMap(Vec3f radiance, ToneMapper mapper, float exposureEv);
float srgbEncode(float linear);
// 最终的 8 位显示像素：色调映射 → sRGB 编码 → 四舍五入（与 GPU 写入 RGBA8 相同）。
std::array<std::uint8_t, 3> displayPixel(Vec3f radiance, ToneMapper mapper, float exposureEv);

// GLSL 函数库（不含 #version）：D_GGX、V_SmithGGXCorrelated、F_Schlick、hemisphere、shadeSurface、toneMap、srgbEncode。
// 使用方需声明 uniform：uSky、uGround、uAmbient（vec3、vec3、float）。
const char* shaderLibrary();

}  // namespace pbr

}  // namespace terra::render
