#include "terra/render/pbr.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace terra::render::pbr {

namespace {

Vec3f add(Vec3f a, Vec3f b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3f sub(Vec3f a, Vec3f b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3f mul(Vec3f a, Vec3f b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }
Vec3f scale(Vec3f a, float k) { return {a.x * k, a.y * k, a.z * k}; }
float dot(Vec3f a, Vec3f b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3f normalize(Vec3f a) {
    const float len = std::sqrt(dot(a, a));
    return len > 0.0f ? scale(a, 1.0f / len) : a;
}
Vec3f mix(Vec3f a, Vec3f b, float t) { return add(scale(a, 1.0f - t), scale(b, t)); }
float clamp01(float x) { return std::min(std::max(x, 0.0f), 1.0f); }
Vec3f clamp01(Vec3f a) { return {clamp01(a.x), clamp01(a.y), clamp01(a.z)}; }
Vec3f reflect(Vec3f i, Vec3f n) { return sub(i, scale(n, 2.0f * dot(n, i))); }

// GLSL mat3(c0, c1, c2) 按列存储：m · v = c0·v.x + c1·v.y + c2·v.z。
Vec3f mulColumns(const float m[9], Vec3f v) {
    return {m[0] * v.x + m[3] * v.y + m[6] * v.z, m[1] * v.x + m[4] * v.y + m[7] * v.z, m[2] * v.x + m[5] * v.y + m[8] * v.z};
}

// AgX 最小近似（Benjamin Wrensch，iolite；Blender 4 的 AgX 视图变换的常用实时近似）。
constexpr float kAgxIn[9] = {0.842479062253094f, 0.0423282422610123f, 0.0423756549057051f,
                             0.0784335999999992f, 0.878468636469772f, 0.0784336f,
                             0.0792237451477643f, 0.0791661274605434f, 0.879142973793104f};
constexpr float kAgxOut[9] = {1.19687900512017f, -0.0528968517574562f, -0.0529716355144438f,
                              -0.0980208811401368f, 1.15190312990417f, -0.0980434501171241f,
                              -0.0990297440797205f, -0.0989611768448433f, 1.15107367264116f};
constexpr float kAgxMinEv = -12.47393f;
constexpr float kAgxMaxEv = 4.026069f;

float agxContrast(float x) {
    const float x2 = x * x;
    const float x4 = x2 * x2;
    return 15.5f * x4 * x2 - 40.14f * x4 * x + 31.96f * x4 - 6.868f * x2 * x + 0.4298f * x2 + 0.1191f * x - 0.00232f;
}

Vec3f agx(Vec3f c) {
    Vec3f v = mulColumns(kAgxIn, c);
    const auto encode = [](float x) {
        const float ev = std::min(std::max(std::log2(std::max(x, 1e-10f)), kAgxMinEv), kAgxMaxEv);
        return agxContrast((ev - kAgxMinEv) / (kAgxMaxEv - kAgxMinEv));
    };
    v = {encode(v.x), encode(v.y), encode(v.z)};
    v = mulColumns(kAgxOut, v);
    return {std::pow(std::max(v.x, 0.0f), 2.2f), std::pow(std::max(v.y, 0.0f), 2.2f), std::pow(std::max(v.z, 0.0f), 2.2f)};
}

// Khronos PBR Neutral（2024）。
Vec3f pbrNeutral(Vec3f c) {
    constexpr float kStart = 0.8f - 0.04f;
    constexpr float kDesaturation = 0.15f;
    const float x = std::min(c.x, std::min(c.y, c.z));
    const float offset = x < 0.08f ? x - 6.25f * x * x : 0.04f;
    c = sub(c, Vec3f{offset, offset, offset});
    const float peak = std::max(c.x, std::max(c.y, c.z));
    if (peak < kStart) return c;
    constexpr float d = 1.0f - kStart;
    const float newPeak = 1.0f - d * d / (peak + d - kStart);
    c = scale(c, newPeak / peak);
    const float g = 1.0f - 1.0f / (kDesaturation * (peak - newPeak) + 1.0f);
    return mix(c, Vec3f{newPeak, newPeak, newPeak}, g);
}

// ACES 拟合（Krzysztof Narkowicz，输入先乘 0.6）。
float acesChannel(float x) {
    x *= 0.6f;
    return (x * (2.51f * x + 0.03f)) / (x * (2.43f * x + 0.59f) + 0.14f);
}

}  // namespace

float distributionGgx(float NoH, float alpha) {
    const float a2 = alpha * alpha;
    const float f = (NoH * a2 - NoH) * NoH + 1.0f;
    return a2 / (kPi * f * f);
}

float visibilitySmithGgxCorrelated(float NoV, float NoL, float alpha) {
    const float a2 = alpha * alpha;
    const float ggxl = NoV * std::sqrt((-NoL * a2 + NoL) * NoL + a2);
    const float ggxv = NoL * std::sqrt((-NoV * a2 + NoV) * NoV + a2);
    return 0.5f / (ggxv + ggxl);
}

Vec3f fresnelSchlick(Vec3f f0, float VoH) {
    const float f = std::pow(1.0f - VoH, 5.0f);
    return add(Vec3f{f, f, f}, scale(f0, 1.0f - f));
}

Vec3f hemisphere(const HemisphereLight& env, Vec3f direction) {
    return scale(mix(env.ground, env.sky, 0.5f * direction.z + 0.5f), env.intensity);
}

float specularOcclusion(float NoV, float occlusion, float alpha) {
    return clamp01(std::pow(NoV + occlusion, std::exp2(-16.0f * alpha - 1.0f)) - 1.0f + occlusion);
}

Vec3f shade(const Material& m, Vec3f n, Vec3f v, const SunLight& sun, const HemisphereLight& env, float shadow,
            float occlusion) {
    const float rough = std::max(m.roughness, kMinRoughness);
    const float alpha = rough * rough;
    const Vec3f diffuseColor = scale(m.baseColor, 1.0f - m.metallic);
    const float r0 = 0.16f * m.reflectance * m.reflectance;
    const Vec3f f0 = mix(Vec3f{r0, r0, r0}, m.baseColor, m.metallic);
    const float NoV = std::max(dot(n, v), 1e-4f);
    Vec3f color = m.emissive;

    const Vec3f l = sun.direction;
    const float NoL = clamp01(dot(n, l));
    if (NoL > 0.0f) {
        const Vec3f h = normalize(add(v, l));
        const float NoH = clamp01(dot(n, h));
        const float LoH = clamp01(dot(l, h));
        const Vec3f F = fresnelSchlick(f0, LoH);
        const float D = distributionGgx(NoH, alpha);
        const float V = visibilitySmithGgxCorrelated(NoV, NoL, alpha);
        const Vec3f Fr = scale(F, D * V);
        const Vec3f Fd = scale(diffuseColor, 1.0f / kPi);
        const Vec3f radiance = scale(sun.color, sun.intensity * NoL * shadow);
        color = add(color, mul(add(Fd, Fr), radiance));
    }

    const float fresnel = std::pow(1.0f - NoV, 5.0f);
    const Vec3f rough3{1.0f - rough, 1.0f - rough, 1.0f - rough};
    const Vec3f Fa = add(f0, scale(sub(Vec3f{std::max(rough3.x, f0.x), std::max(rough3.y, f0.y), std::max(rough3.z, f0.z)}, f0),
                                   fresnel));
    const Vec3f r = normalize(mix(reflect(scale(v, -1.0f), n), n, alpha));
    const Vec3f one{1.0f, 1.0f, 1.0f};
    const float so = specularOcclusion(NoV, occlusion, alpha);
    color = add(color, add(scale(mul(mul(hemisphere(env, n), diffuseColor), sub(one, Fa)), occlusion),
                           scale(mul(hemisphere(env, r), Fa), so)));
    return color;
}

Vec3f toneMap(Vec3f c, ToneMapper mapper, float exposureEv) {
    c = scale(c, std::exp2(exposureEv));
    c = {std::max(c.x, 0.0f), std::max(c.y, 0.0f), std::max(c.z, 0.0f)};
    switch (mapper) {
        case ToneMapper::AgX: c = agx(c); break;
        case ToneMapper::PbrNeutral: c = pbrNeutral(c); break;
        case ToneMapper::Aces: c = {acesChannel(c.x), acesChannel(c.y), acesChannel(c.z)}; break;
        case ToneMapper::None: break;
    }
    return clamp01(c);
}

// ---- IBL ----

namespace {

float radicalInverse(std::uint32_t bits) {
    bits = (bits << 16u) | (bits >> 16u);
    bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
    bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
    bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
    bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
    return static_cast<float>(bits) * 2.3283064365386963e-10f;
}

}  // namespace

std::vector<float> dfgLut(int size, int samples) {
    size = std::max(size, 1);
    samples = std::max(samples, 1);
    std::vector<float> lut(static_cast<std::size_t>(size) * static_cast<std::size_t>(size) * 2u, 0.0f);
    for (int j = 0; j < size; ++j) {
        const float rough = std::max((static_cast<float>(j) + 0.5f) / static_cast<float>(size), kMinRoughness);
        const float alpha = rough * rough;
        for (int i = 0; i < size; ++i) {
            const float NoV = (static_cast<float>(i) + 0.5f) / static_cast<float>(size);
            const Vec3f v{std::sqrt(1.0f - NoV * NoV), 0.0f, NoV};
            float a = 0.0f;
            float b = 0.0f;
            for (int k = 0; k < samples; ++k) {
                const float xi0 = static_cast<float>(k) / static_cast<float>(samples);
                const float xi1 = radicalInverse(static_cast<std::uint32_t>(k));
                const float phi = 2.0f * kPi * xi0;
                const float cosTheta = std::sqrt((1.0f - xi1) / (1.0f + (alpha * alpha - 1.0f) * xi1));
                const float sinTheta = std::sqrt(std::max(0.0f, 1.0f - cosTheta * cosTheta));
                const Vec3f h{sinTheta * std::cos(phi), sinTheta * std::sin(phi), cosTheta};
                const float VoH = dot(v, h);
                const Vec3f l = sub(scale(h, 2.0f * VoH), v);
                const float NoL = l.z;
                const float NoH = h.z;
                if (NoL <= 0.0f || NoH <= 0.0f || VoH <= 0.0f) continue;
                const float vis = visibilitySmithGgxCorrelated(NoV, NoL, alpha);
                const float gv = vis * 4.0f * NoL * VoH / NoH;
                const float fc = std::pow(1.0f - VoH, 5.0f);
                a += (1.0f - fc) * gv;
                b += fc * gv;
            }
            const std::size_t o = (static_cast<std::size_t>(j) * static_cast<std::size_t>(size) + static_cast<std::size_t>(i)) * 2u;
            lut[o] = a / static_cast<float>(samples);
            lut[o + 1] = b / static_cast<float>(samples);
        }
    }
    return lut;
}

Vec2f sampleDfg(const std::vector<float>& lut, int size, float NoV, float roughness) {
    if (size <= 0 || lut.size() < static_cast<std::size_t>(size) * static_cast<std::size_t>(size) * 2u) return {};
    const auto texel = [&](int i, int j) {
        i = std::min(std::max(i, 0), size - 1);
        j = std::min(std::max(j, 0), size - 1);
        const std::size_t o = (static_cast<std::size_t>(j) * static_cast<std::size_t>(size) + static_cast<std::size_t>(i)) * 2u;
        return Vec2f{lut[o], lut[o + 1]};
    };
    const float x = clamp01(NoV) * static_cast<float>(size) - 0.5f;
    const float y = clamp01(roughness) * static_cast<float>(size) - 0.5f;
    const float fx = std::floor(x);
    const float fy = std::floor(y);
    const int i0 = static_cast<int>(fx);
    const int j0 = static_cast<int>(fy);
    const float tx = x - fx;
    const float ty = y - fy;
    const Vec2f a = texel(i0, j0);
    const Vec2f b = texel(i0 + 1, j0);
    const Vec2f c = texel(i0, j0 + 1);
    const Vec2f d = texel(i0 + 1, j0 + 1);
    const auto lerp = [](float p, float q, float t) { return p + (q - p) * t; };
    return {lerp(lerp(a.x, b.x, tx), lerp(c.x, d.x, tx), ty), lerp(lerp(a.y, b.y, tx), lerp(c.y, d.y, tx), ty)};
}

Vec3f ambientIbl(const Material& m, Vec3f n, Vec3f v, Vec3f irradiance, Vec3f prefiltered, const std::vector<float>& lut,
                 int lutSize, float occlusion) {
    const float rough = std::max(m.roughness, kMinRoughness);
    const Vec3f diffuseColor = scale(m.baseColor, 1.0f - m.metallic);
    const float r0 = 0.16f * m.reflectance * m.reflectance;
    const Vec3f f0 = mix(Vec3f{r0, r0, r0}, m.baseColor, m.metallic);
    const float NoV = std::max(dot(n, v), 1e-4f);
    const Vec2f dfg = sampleDfg(lut, lutSize, NoV, rough);
    const Vec3f e = add(scale(f0, dfg.x), Vec3f{dfg.y, dfg.y, dfg.y});
    const Vec3f one{1.0f, 1.0f, 1.0f};
    const float so = specularOcclusion(NoV, occlusion, rough * rough);
    return add(scale(mul(mul(irradiance, diffuseColor), sub(one, e)), occlusion), scale(mul(prefiltered, e), so));
}

float srgbDecode(float x) {
    x = clamp01(x);
    return x <= 0.04045f ? x / 12.92f : std::pow((x + 0.055f) / 1.055f, 2.4f);
}

float srgbEncode(float x) {
    x = clamp01(x);
    return x <= 0.0031308f ? 12.92f * x : 1.055f * std::pow(x, 1.0f / 2.4f) - 0.055f;
}

std::array<std::uint8_t, 3> displayPixel(Vec3f radiance, ToneMapper mapper, float exposureEv) {
    const Vec3f t = toneMap(radiance, mapper, exposureEv);
    const auto to8 = [](float x) { return static_cast<std::uint8_t>(std::lround(srgbEncode(x) * 255.0f)); };
    return {to8(t.x), to8(t.y), to8(t.z)};
}

const char* shaderLibrary() {
    // 与上面的 C++ 实现逐项对应。
    return R"GLSL(
const float PI = 3.14159265358979;
const float MIN_ROUGHNESS = 0.045;

uniform vec3 uSky;
uniform vec3 uGround;
uniform float uAmbient;

float D_GGX(float NoH, float a) {
    float a2 = a * a;
    float f = (NoH * a2 - NoH) * NoH + 1.0;
    return a2 / (PI * f * f);
}

float V_SmithGGXCorrelated(float NoV, float NoL, float a) {
    float a2 = a * a;
    float ggxl = NoV * sqrt((-NoL * a2 + NoL) * NoL + a2);
    float ggxv = NoL * sqrt((-NoV * a2 + NoV) * NoV + a2);
    return 0.5 / (ggxv + ggxl);
}

vec3 F_Schlick(vec3 f0, float VoH) {
    float f = pow(1.0 - VoH, 5.0);
    return vec3(f) + f0 * (1.0 - f);
}

vec3 hemisphere(vec3 d) {
    return mix(uGround, uSky, 0.5 * d.z + 0.5) * uAmbient;
}

float specularOcclusion(float NoV, float occlusion, float a) {
    return clamp(pow(NoV + occlusion, exp2(-16.0 * a - 1.0)) - 1.0 + occlusion, 0.0, 1.0);
}

vec3 shadeSurface(vec3 baseColor, float metallic, float roughness, float reflectance, vec3 emissive,
                  vec3 n, vec3 v, vec3 l, vec3 sunRadiance, float shadow, float occlusion) {
    float rough = max(roughness, MIN_ROUGHNESS);
    float a = rough * rough;
    vec3 diffuseColor = baseColor * (1.0 - metallic);
    float r0 = 0.16 * reflectance * reflectance;
    vec3 f0 = mix(vec3(r0), baseColor, metallic);
    float NoV = max(dot(n, v), 1e-4);
    vec3 color = emissive;
    float NoL = clamp(dot(n, l), 0.0, 1.0);
    if (NoL > 0.0) {
        vec3 h = normalize(v + l);
        float NoH = clamp(dot(n, h), 0.0, 1.0);
        float LoH = clamp(dot(l, h), 0.0, 1.0);
        vec3 F = F_Schlick(f0, LoH);
        float D = D_GGX(NoH, a);
        float V = V_SmithGGXCorrelated(NoV, NoL, a);
        vec3 Fr = F * (D * V);
        vec3 Fd = diffuseColor * (1.0 / PI);
        color += (Fd + Fr) * (sunRadiance * (NoL * shadow));
    }
    float fresnel = pow(1.0 - NoV, 5.0);
    vec3 Fa = f0 + (max(vec3(1.0 - rough), f0) - f0) * fresnel;
    vec3 r = normalize(mix(reflect(-v, n), n, a));
    color += hemisphere(n) * diffuseColor * (vec3(1.0) - Fa) * occlusion + hemisphere(r) * Fa * specularOcclusion(NoV, occlusion, a);
    return color;
}

vec3 agxContrast(vec3 x) {
    vec3 x2 = x * x;
    vec3 x4 = x2 * x2;
    return 15.5 * x4 * x2 - 40.14 * x4 * x + 31.96 * x4 - 6.868 * x2 * x + 0.4298 * x2 + 0.1191 * x - 0.00232;
}

vec3 agx(vec3 c) {
    const mat3 agxIn = mat3(0.842479062253094, 0.0423282422610123, 0.0423756549057051,
                            0.0784335999999992, 0.878468636469772, 0.0784336,
                            0.0792237451477643, 0.0791661274605434, 0.879142973793104);
    const mat3 agxOut = mat3(1.19687900512017, -0.0528968517574562, -0.0529716355144438,
                             -0.0980208811401368, 1.15190312990417, -0.0980434501171241,
                             -0.0990297440797205, -0.0989611768448433, 1.15107367264116);
    const float minEv = -12.47393;
    const float maxEv = 4.026069;
    vec3 v = agxIn * c;
    v = clamp(log2(max(v, vec3(1e-10))), minEv, maxEv);
    v = agxContrast((v - minEv) / (maxEv - minEv));
    v = agxOut * v;
    return pow(max(v, vec3(0.0)), vec3(2.2));
}

vec3 pbrNeutral(vec3 c) {
    const float startCompression = 0.8 - 0.04;
    const float desaturation = 0.15;
    float x = min(c.r, min(c.g, c.b));
    float offset = x < 0.08 ? x - 6.25 * x * x : 0.04;
    c -= offset;
    float peak = max(c.r, max(c.g, c.b));
    if (peak < startCompression) return c;
    const float d = 1.0 - startCompression;
    float newPeak = 1.0 - d * d / (peak + d - startCompression);
    c *= newPeak / peak;
    float g = 1.0 - 1.0 / (desaturation * (peak - newPeak) + 1.0);
    return mix(c, vec3(newPeak), g);
}

vec3 aces(vec3 x) {
    x *= 0.6;
    return (x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14);
}

// mode：0 AgX、1 PBR Neutral、2 ACES、3 无
vec3 toneMap(vec3 c, int mode, float exposureEv) {
    c = max(c * exp2(exposureEv), vec3(0.0));
    if (mode == 0) c = agx(c);
    else if (mode == 1) c = pbrNeutral(c);
    else if (mode == 2) c = aces(c);
    return clamp(c, 0.0, 1.0);
}

float srgbEncode(float x) {
    x = clamp(x, 0.0, 1.0);
    return x <= 0.0031308 ? 12.92 * x : 1.055 * pow(x, 1.0 / 2.4) - 0.055;
}

// ---- IBL（M1-08b）：与 C++ ambientIbl 对应；环境贴图按 uEnvRotation（cos, sin）绕 Z 旋转 ----
uniform samplerCube uIrradiance;
uniform samplerCube uPrefiltered;
uniform sampler2D uDfg;
uniform float uPrefilteredMaxLod;
uniform vec2 uEnvRotation;

vec3 envDirection(vec3 d) {
    return vec3(uEnvRotation.x * d.x + uEnvRotation.y * d.y, -uEnvRotation.y * d.x + uEnvRotation.x * d.y, d.z);
}

vec3 ambientIbl(vec3 baseColor, float metallic, float roughness, float reflectance, vec3 n, vec3 v, float occlusion) {
    float rough = max(roughness, MIN_ROUGHNESS);
    vec3 diffuseColor = baseColor * (1.0 - metallic);
    float r0 = 0.16 * reflectance * reflectance;
    vec3 f0 = mix(vec3(r0), baseColor, metallic);
    float NoV = max(dot(n, v), 1e-4);
    vec2 dfg = texture(uDfg, vec2(NoV, rough)).rg;
    vec3 e = f0 * dfg.x + vec3(dfg.y);
    vec3 irradiance = texture(uIrradiance, envDirection(n)).rgb;
    vec3 prefiltered = textureLod(uPrefiltered, envDirection(reflect(-v, n)), rough * uPrefilteredMaxLod).rgb;
    return irradiance * diffuseColor * (vec3(1.0) - e) * occlusion + prefiltered * e * specularOcclusion(NoV, occlusion, rough * rough);
}
)GLSL";
}

}  // namespace terra::render::pbr
