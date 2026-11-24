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
#include <vector>

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

    // M1-09：贴图（GL 纹理名称，0 = 无）。按 glTF 约定与上面的因子相乘。
    std::uint32_t baseColorTexture = 0;          // sRGB：RGB 乘以 baseColor
    std::uint32_t metallicRoughnessTexture = 0;  // 线性：G 乘以粗糙度，B 乘以金属度
    std::uint32_t normalTexture = 0;             // 线性：切线空间法线（OpenGL 约定，+Y 向上），xy 乘以 normalScale
    std::uint32_t occlusionTexture = 0;          // 线性：R 为环境遮蔽，按 occlusionStrength 插值
    std::uint32_t emissiveTexture = 0;           // sRGB：RGB 乘以 emissive
    float normalScale = 1.0f;
    float occlusionStrength = 1.0f;
    bool doubleSided = false;                    // 双面：不剔除背面，背面法线取反
    // M3-06：不透明度。小于 1 时在不透明物体、背景与网格之后按距离从远到近混合绘制（深度测试、不写深度、不剔除背面）；
    // 半透明物体应设 castsShadow = false（阴影图只画不透明物体）。
    float opacity = 1.0f;
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
// occlusion：环境遮蔽（M1-09），只作用于环境光——漫反射乘以 occlusion，镜面乘以 specularOcclusion。
Vec3f shade(const Material& material, Vec3f n, Vec3f v, const SunLight& sun, const HemisphereLight& env, float shadow,
            float occlusion = 1.0f);
// 镜面遮蔽近似（Lagarde）：saturate((N·V + ao)^(2^(−16α − 1)) − 1 + ao)，α = 粗糙度²。ao = 1 时为 1。
float specularOcclusion(float NoV, float occlusion, float alpha);

// 乘以 2^exposureEv 后色调映射，返回线性显示值 ∈ [0, 1]。
Vec3f toneMap(Vec3f radiance, ToneMapper mapper, float exposureEv);
float srgbEncode(float linear);
float srgbDecode(float encoded);  // sRGB → 线性（与 GL 的 SRGB8_ALPHA8 解码相同的公式）
// 最终的 8 位显示像素：色调映射 → sRGB 编码 → 四舍五入（与 GPU 写入 RGBA8 相同）。
std::array<std::uint8_t, 3> displayPixel(Vec3f radiance, ToneMapper mapper, float exposureEv);

// ---- 基于图像的光照（M1-08b，分离求和近似）----
// 环境光 = 辐照度(n) · diffuseColor · (1 − E) + 预过滤辐射(r, 粗糙度) · E，其中 E = F0 · DFG.x + DFG.y，
// r = reflect(−v, n)，DFG 为预积分的 BRDF 查找表（行 = 粗糙度，列 = N·V，取纹素中心，双线性采样）。

struct Vec2f {
    float x = 0.0f;
    float y = 0.0f;
};

// 生成 size × size 的 DFG 查找表（每项两个值，行优先，第 0 行为最小粗糙度）：
// GGX 重要性采样（Hammersley 序列，samples 个样本），与着色器中的高度相关 Smith 可见性一致。
std::vector<float> dfgLut(int size, int samples);
// 与 GL_LINEAR + CLAMP_TO_EDGE 相同的双线性采样。
Vec2f sampleDfg(const std::vector<float>& lut, int size, float NoV, float roughness);
// 环境光部分（不含太阳光与自发光）；irradiance、prefiltered 由调用方提供（测试中来自解析环境）。
Vec3f ambientIbl(const Material& material, Vec3f n, Vec3f v, Vec3f irradiance, Vec3f prefiltered,
                 const std::vector<float>& lut, int lutSize, float occlusion = 1.0f);

// GLSL 函数库（不含 #version）：D_GGX、V_SmithGGXCorrelated、F_Schlick、hemisphere、shadeSurface、toneMap、srgbEncode。
// 使用方需声明 uniform：uSky、uGround、uAmbient（vec3、vec3、float）。
const char* shaderLibrary();

}  // namespace pbr

}  // namespace terra::render
