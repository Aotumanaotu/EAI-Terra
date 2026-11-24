#include "terra/render/scene_renderer.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>
#include <vector>

#include <glad/gl.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "terra/render/context.hpp"
#include "terra/render/resources.hpp"

namespace terra::render {

namespace {

// ---- 着色器 ----

const char* kDepthVertex = R"(#version 450 core
layout(location = 0) in vec3 aPosition;
uniform mat4 uLightViewProj;
uniform mat4 uModel;
void main() { gl_Position = uLightViewProj * uModel * vec4(aPosition, 1.0); }
)";

const char* kDepthFragment = R"(#version 450 core
void main() {}
)";

const char* kPbrVertex = R"(#version 450 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aUv;
uniform mat4 uModel;
uniform mat4 uNormalMatrix;
layout(location = 3) in vec4 aTangent;
uniform mat4 uViewProj;
out vec3 vWorld;
out vec3 vNormal;
out vec2 vUv;
out vec4 vTangent;
void main() {
    vec4 world = uModel * vec4(aPosition, 1.0);
    vWorld = world.xyz;
    vNormal = mat3(uNormalMatrix) * aNormal;
    vUv = aUv;
    vTangent = vec4(mat3(uModel) * aTangent.xyz, aTangent.w);
    gl_Position = uViewProj * world;
}
)";

const char* kPbrFragmentMain = R"(
in vec3 vWorld;
in vec3 vNormal;
in vec2 vUv;
in vec4 vTangent;
out vec4 fragColor;
// 贴图（M1-09）：uTextures 按位表示有哪些贴图——1 基础色、2 金属度-粗糙度、4 法线、8 环境遮蔽、16 自发光
uniform int uTextures;
uniform sampler2D uBaseColorMap;
uniform sampler2D uMetallicRoughnessMap;
uniform sampler2D uNormalMap;
uniform sampler2D uOcclusionMap;
uniform sampler2D uEmissiveMap;
uniform float uNormalScale;
uniform float uOcclusionStrength;
uniform int uDoubleSided;
uniform vec3 uBaseColor;
uniform float uMetallic;
uniform float uRoughness;
uniform float uReflectance;
uniform vec3 uEmissive;
uniform vec3 uCamera;
uniform vec3 uSunDir;
uniform vec3 uSunRadiance;
uniform int uShadows;
uniform mat4 uLightViewProj;
uniform sampler2DShadow uShadowMap;
uniform float uNormalBias;
uniform float uTexel;

float shadowFactor(vec3 p, vec3 n) {
    if (uShadows == 0) return 1.0;
    vec4 clip = uLightViewProj * vec4(p + n * uNormalBias, 1.0);
    vec3 s = clip.xyz / clip.w * 0.5 + 0.5;
    if (s.z >= 1.0) return 1.0;
    float sum = 0.0;
    for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) sum += texture(uShadowMap, vec3(s.xy + vec2(x, y) * uTexel, s.z));
    }
    return sum / 9.0;
}

uniform int uIbl;
uniform float uIblIntensity;
uniform float uOpacity;

void main() {
    vec3 geometric = normalize(vNormal);
    if (uDoubleSided == 1 && !gl_FrontFacing) geometric = -geometric;
    vec3 baseColor = uBaseColor;
    float metallic = uMetallic;
    float roughness = uRoughness;
    vec3 emissive = uEmissive;
    float occlusion = 1.0;
    vec3 n = geometric;
    if ((uTextures & 1) != 0) baseColor *= texture(uBaseColorMap, vUv).rgb;
    if ((uTextures & 2) != 0) {
        vec4 mr = texture(uMetallicRoughnessMap, vUv);
        roughness *= mr.g;
        metallic *= mr.b;
    }
    if ((uTextures & 4) != 0) {
        vec3 t = texture(uNormalMap, vUv).xyz * 2.0 - 1.0;
        t.xy *= uNormalScale;
        vec3 tangent = normalize(vTangent.xyz - geometric * dot(geometric, vTangent.xyz));
        vec3 bitangent = cross(geometric, tangent) * vTangent.w;
        n = normalize(tangent * t.x + bitangent * t.y + geometric * t.z);
    }
    if ((uTextures & 8) != 0) occlusion = 1.0 + uOcclusionStrength * (texture(uOcclusionMap, vUv).r - 1.0);
    if ((uTextures & 16) != 0) emissive *= texture(uEmissiveMap, vUv).rgb;
    vec3 v = normalize(uCamera - vWorld);
    float shadow = shadowFactor(vWorld, geometric);
    vec3 color = shadeSurface(baseColor, metallic, roughness, uReflectance, emissive, n, v, uSunDir, uSunRadiance, shadow, occlusion);
    if (uIbl == 1) color += ambientIbl(baseColor, metallic, roughness, uReflectance, n, v, occlusion) * uIblIntensity;
    fragColor = vec4(color, uOpacity);
}
)";

const char* kGridVertex = R"(#version 450 core
uniform mat4 uViewProj;
uniform vec3 uCenter;
uniform float uExtent;
out vec2 vWorld;
const vec2 kCorners[6] = vec2[](vec2(-1, -1), vec2(1, -1), vec2(1, 1), vec2(-1, -1), vec2(1, 1), vec2(-1, 1));
void main() {
    vec2 p = uCenter.xy + kCorners[gl_VertexID] * uExtent;
    vWorld = p;
    gl_Position = uViewProj * vec4(p, 0.0, 1.0);
}
)";

const char* kGridFragment = R"(#version 450 core
in vec2 vWorld;
out vec4 fragColor;
uniform vec3 uCamera;
uniform float uExtent;
uniform vec3 uAxisX;
uniform vec3 uAxisY;

float gridLine(vec2 coord, float width) {
    vec2 d = fwidth(coord);
    vec2 g = abs(fract(coord - 0.5) - 0.5) / (d * width);
    return 1.0 - min(min(g.x, g.y), 1.0);
}

void main() {
    float dist = length(vWorld - uCamera.xy);
    float fade = 1.0 - smoothstep(uExtent * 0.3, uExtent * 0.9, dist);
    vec2 d = fwidth(vWorld);
    float minorFade = 1.0 - smoothstep(0.15, 0.5, max(d.x, d.y));  // 1 m 线过密时淡出，避免摩尔纹
    float minor = gridLine(vWorld, 1.0) * minorFade;
    float major = gridLine(vWorld / 10.0, 1.2);
    vec3 color = vec3(0.2);
    float alpha = max(minor * 0.3, major * 0.5);
    float ax = 1.0 - min(abs(vWorld.y) / (d.y * 1.5), 1.0);  // X 轴（y = 0）
    float ay = 1.0 - min(abs(vWorld.x) / (d.x * 1.5), 1.0);  // Y 轴（x = 0）
    color = mix(color, uAxisX, ax);
    alpha = max(alpha, ax * 0.9);
    color = mix(color, uAxisY, ay);
    alpha = max(alpha, ay * 0.9);
    alpha *= fade;
    if (alpha <= 0.001) discard;
    fragColor = vec4(color, alpha);
}
)";

const char* kFullscreenVertex = R"(#version 450 core
void main() {
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
)";

// 环境背景：全屏三角形放在远平面（深度 1），只填充没有物体的像素。
const char* kBackgroundVertex = R"(#version 450 core
void main() {
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    gl_Position = vec4(p * 2.0 - 1.0, 1.0, 1.0);
}
)";

const char* kBackgroundFragmentMain = R"(
out vec4 fragColor;
uniform mat4 uInvViewProj;
uniform vec3 uCamera;
uniform vec2 uViewport;
uniform samplerCube uEnvironmentCube;
uniform float uLod;
uniform float uIntensity;
void main() {
    vec2 ndc = gl_FragCoord.xy / uViewport * 2.0 - 1.0;
    vec4 world = uInvViewProj * vec4(ndc, 1.0, 1.0);
    vec3 dir = normalize(world.xyz / world.w - uCamera);
    fragColor = vec4(textureLod(uEnvironmentCube, envDirection(dir), uLod).rgb * uIntensity, 1.0);
}
)";

const char* kToneMapFragmentMain = R"(
out vec4 fragColor;
uniform sampler2D uHdr;
uniform int uToneMapper;
uniform float uExposure;
void main() {
    vec3 c = texelFetch(uHdr, ivec2(gl_FragCoord.xy), 0).rgb;
    vec3 t = toneMap(c, uToneMapper, uExposure);
    fragColor = vec4(srgbEncode(t.r), srgbEncode(t.g), srgbEncode(t.b), 1.0);
}
)";

std::string withLibrary(const char* main) { return std::string{"#version 450 core\n"} + pbr::shaderLibrary() + main; }

glm::mat4 toGlm(const Matrix4& m) {
    glm::mat4 out;
    std::memcpy(&out[0][0], m.data(), sizeof(float) * 16);
    return out;
}

Matrix4 fromGlm(const glm::mat4& m) {
    Matrix4 out;
    std::memcpy(out.data(), &m[0][0], sizeof(float) * 16);
    return out;
}

glm::vec3 toGlm(Vec3f v) { return {v.x, v.y, v.z}; }

// 环境旋转：着色器中的 envDirection 用 (cos, sin) 把世界方向绕 Z 旋转 −θ 后查找贴图。
void setRotation(ShaderProgram& program, float radians) {
    const int loc = program.uniformLocation("uEnvRotation");
    if (loc >= 0) program.setUniformVec2(loc, std::cos(radians), std::sin(radians));
}

int toneMapperIndex(ToneMapper t) {
    switch (t) {
        case ToneMapper::AgX: return 0;
        case ToneMapper::PbrNeutral: return 1;
        case ToneMapper::Aces: return 2;
        case ToneMapper::None: return 3;
    }
    return 0;
}

}  // namespace

Matrix4 identityMatrix() { return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}; }

Matrix4 translationScale(Vec3f t, Vec3f s) { return {s.x, 0, 0, 0, 0, s.y, 0, 0, 0, 0, s.z, 0, t.x, t.y, t.z, 1}; }

struct SceneRenderer::Impl {
    explicit Impl(RenderContext& c) : ctx(c) {}

    RenderContext& ctx;
    ShaderProgram depth;
    ShaderProgram pbr;
    ShaderProgram grid;
    ShaderProgram toneMap;
    ShaderProgram background;
    VertexArray empty;  // 全屏三角形与网格四边形由 gl_VertexID 生成，核心模式仍需绑定一个顶点数组
    std::optional<Framebuffer> hdr;       // 场景（可多重采样）
    std::optional<Framebuffer> resolved;  // 多重采样解析结果
    std::optional<Framebuffer> output;    // RGBA8 显示结果
    std::optional<Framebuffer> shadow;
    int width = 0;
    int height = 0;
    int samples = 1;
    FrameStats stats;

    // ---- GPU 计时：kRing 帧轮换，每帧 kStamps 个时间戳 ----
    static constexpr int kRing = 3;
    static constexpr int kStamps = kRenderPassCount + 1;
    GLuint queries[kRing][kStamps] = {};
    bool pending[kRing] = {};
    bool timing = false;
    bool queriesCreated = false;
    int frame = 0;
    int current = -1;  // 本帧使用的轮换槽；未计时为 -1
    PassTimings last;

    ~Impl() {
        if (queriesCreated) glDeleteQueries(kRing * kStamps, &queries[0][0]);
    }

    void stamp(int index) {
        if (current >= 0) glQueryCounter(queries[current][index], GL_TIMESTAMP);
    }

    // 读取一个槽的结果；wait 为假且结果尚不可用时返回假。
    bool collect(int slot, bool wait) {
        if (!pending[slot]) return false;
        if (!wait) {
            GLint available = 0;
            glGetQueryObjectiv(queries[slot][kStamps - 1], GL_QUERY_RESULT_AVAILABLE, &available);
            if (available == 0) return false;
        }
        GLuint64 t[kStamps] = {};
        for (int i = 0; i < kStamps; ++i) glGetQueryObjectui64v(queries[slot][i], GL_QUERY_RESULT, &t[i]);
        PassTimings out;
        for (int i = 0; i < kRenderPassCount; ++i) {
            out.milliseconds[static_cast<std::size_t>(i)] = t[i + 1] >= t[i] ? static_cast<double>(t[i + 1] - t[i]) * 1e-6 : 0.0;
        }
        out.valid = true;
        last = out;
        pending[slot] = false;
        return true;
    }

    void ensureTargets(int w, int h, int requestedSamples) {
        int s = std::clamp(requestedSamples, 1, std::max(1, ctx.info().maxSamples));
        if (hdr && w == width && h == height && s == samples) return;
        width = w;
        height = h;
        FramebufferDesc d;
        d.width = w;
        d.height = h;
        d.color = ColorFormat::Rgba16F;
        d.depth = DepthFormat::Renderbuffer;
        d.samples = s;
        auto made = Framebuffer::create(d);
        if (!made.framebuffer && s > 1) {  // 驱动不支持该采样数时退回单采样
            s = 1;
            d.samples = 1;
            made = Framebuffer::create(d);
        }
        samples = s;
        hdr = std::move(made.framebuffer);
        resolved.reset();
        if (s > 1) {
            FramebufferDesc r{w, h, ColorFormat::Rgba16F, DepthFormat::None, 1};
            resolved = Framebuffer::create(r).framebuffer;
        }
        FramebufferDesc o{w, h, ColorFormat::Rgba8, DepthFormat::None, 1};
        output = Framebuffer::create(o).framebuffer;
    }

    void ensureShadow(int size) {
        size = std::clamp(size, 256, 8192);
        if (shadow && shadow->width() == size) return;
        FramebufferDesc d{size, size, ColorFormat::None, DepthFormat::ShadowTexture, 1};
        shadow = Framebuffer::create(d).framebuffer;
    }
};

SceneRenderer::SceneRenderer(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
SceneRenderer::~SceneRenderer() = default;

SceneRendererResult SceneRenderer::create(RenderContext& context) {
    SceneRendererResult result;
    auto impl = std::make_unique<Impl>(context);
    const auto build = [&](ShaderProgram& target, const char* vsName, const std::string& vs, const char* fsName,
                           const std::string& fs) {
        auto made = ShaderProgram::create({{ShaderStage::Vertex, vsName, vs}, {ShaderStage::Fragment, fsName, fs}});
        if (!made.program) {
            if (!result.error) result.error = made.error ? made.error->summary() : std::string{"着色器创建失败"};
            return;
        }
        target = std::move(*made.program);
    };
    build(impl->depth, "shadow.vert", kDepthVertex, "shadow.frag", kDepthFragment);
    build(impl->pbr, "pbr.vert", kPbrVertex, "pbr.frag", withLibrary(kPbrFragmentMain));
    build(impl->grid, "grid.vert", kGridVertex, "grid.frag", kGridFragment);
    build(impl->toneMap, "fullscreen.vert", kFullscreenVertex, "tonemap.frag", withLibrary(kToneMapFragmentMain));
    build(impl->background, "background.vert", kBackgroundVertex, "background.frag", withLibrary(kBackgroundFragmentMain));
    if (result.error) return result;
    impl->empty = VertexArray::create();
    result.renderer.reset(new SceneRenderer(std::move(impl)));
    return result;
}

void SceneRenderer::render(int width, int height, const ViewParams& view, const std::vector<RenderItem>& items,
                           const SunLight& sun, const HemisphereLight& environment, const RenderSettings& settings) {
    render(width, height, view, items, sun, environment, settings, EnvironmentLighting{});
}

void SceneRenderer::render(int width, int height, const ViewParams& view, const std::vector<RenderItem>& items,
                           const SunLight& sun, const HemisphereLight& environment, const RenderSettings& settings,
                           const EnvironmentLighting& imageLighting) {
    Impl& s = *impl_;
    const Environment* env = imageLighting.environment;
    RenderContext& ctx = s.ctx;
    s.stats = FrameStats{};
    if (width <= 0 || height <= 0) return;
    s.ensureTargets(width, height, settings.msaaSamples);
    if (!s.hdr || !s.output || (s.samples > 1 && !s.resolved)) return;
    s.stats.samples = s.samples;
    s.current = -1;
    if (s.timing) {
        if (!s.queriesCreated) {
            glCreateQueries(GL_TIMESTAMP, Impl::kRing * Impl::kStamps, &s.queries[0][0]);
            s.queriesCreated = true;
        }
        // 先不阻塞地收取较早的帧（由旧到新，最新可用的结果留在 last），再占用本帧的槽
        for (int k = Impl::kRing - 1; k >= 1; --k) (void)s.collect((s.frame - k + Impl::kRing * 1024) % Impl::kRing, false);
        const int slot = s.frame % Impl::kRing;
        if (s.pending[slot]) (void)s.collect(slot, true);
        s.current = slot;
        s.stamp(0);
    }

    const glm::mat4 viewProj = toGlm(view.projection) * toGlm(view.view);
    const glm::vec3 l = glm::normalize(toGlm(sun.direction));

    // ---- 阴影图：只覆盖投影物体的包围球 ----
    glm::vec3 lo(std::numeric_limits<float>::max());
    glm::vec3 hi(std::numeric_limits<float>::lowest());
    for (const RenderItem& item : items) {
        if (item.mesh == nullptr || !item.castsShadow) continue;
        ++s.stats.shadowCasters;
        const glm::mat4 model = toGlm(item.model);
        const auto& a = item.mesh->boundsMin();
        const auto& b = item.mesh->boundsMax();
        for (int corner = 0; corner < 8; ++corner) {
            const glm::vec4 p = model * glm::vec4((corner & 1) ? b[0] : a[0], (corner & 2) ? b[1] : a[1], (corner & 4) ? b[2] : a[2], 1.0f);
            lo = glm::min(lo, glm::vec3(p));
            hi = glm::max(hi, glm::vec3(p));
        }
    }
    glm::mat4 lightViewProj(1.0f);
    float normalBias = 0.0f;
    // 阴影图总是存在：着色器的阴影采样器始终绑定一张深度比较纹理（关闭阴影时不采样）。
    s.ensureShadow(settings.shadowMapSize);
    const bool drawShadows = settings.shadows && s.stats.shadowCasters > 0 && l.z > 0.0f && s.shadow.has_value();
    if (drawShadows) {
        const glm::vec3 center = 0.5f * (lo + hi);
        const float radius = std::max(0.5f * glm::length(hi - lo), 0.01f);
        const float back = radius + 50.0f;
        const glm::vec3 up = std::abs(l.z) > 0.99f ? glm::vec3(0, 1, 0) : glm::vec3(0, 0, 1);
        const glm::mat4 lightView = glm::lookAt(center + l * back, center, up);
        const glm::mat4 lightProj = glm::ortho(-radius, radius, -radius, radius, 0.1f, back + radius + 200.0f);
        lightViewProj = lightProj * lightView;
        normalBias = 1.5f * 2.0f * radius / static_cast<float>(s.shadow->width());
        const Matrix4 lvp = fromGlm(lightViewProj);

        s.shadow->bind();
        ctx.setDepthTest(true);
        ctx.setCullBackFaces(false);
        ctx.clear({1.0f, 1.0f, 1.0f, 1.0f});
        ctx.setPolygonOffset(true, 2.0f, 4.0f);
        s.depth.use();
        s.depth.setUniformMatrix4(s.depth.uniformLocation("uLightViewProj"), lvp.data());
        const int modelLoc = s.depth.uniformLocation("uModel");
        for (const RenderItem& item : items) {
            if (item.mesh == nullptr || !item.castsShadow) continue;
            s.depth.setUniformMatrix4(modelLoc, item.model.data());
            item.mesh->draw(ctx);
        }
        ctx.setPolygonOffset(false);
        s.stats.shadowsDrawn = true;
    }

    s.stamp(1);

    // ---- 场景（HDR）----
    s.hdr->bind();
    ctx.setDepthTest(true);
    ctx.setCullBackFaces(true);
    ctx.clear({settings.background.x, settings.background.y, settings.background.z, 1.0f});
    ShaderProgram& p = s.pbr;
    p.use();
    const Matrix4 vp = fromGlm(viewProj);
    const Matrix4 lvp = fromGlm(lightViewProj);
    p.setUniformMatrix4(p.uniformLocation("uViewProj"), vp.data());
    p.setUniformVec3(p.uniformLocation("uCamera"), view.cameraPosition.x, view.cameraPosition.y, view.cameraPosition.z);
    p.setUniformVec3(p.uniformLocation("uSunDir"), l.x, l.y, l.z);
    p.setUniformVec3(p.uniformLocation("uSunRadiance"), sun.color.x * sun.intensity, sun.color.y * sun.intensity,
                     sun.color.z * sun.intensity);
    p.setUniformVec3(p.uniformLocation("uSky"), environment.sky.x, environment.sky.y, environment.sky.z);
    p.setUniformVec3(p.uniformLocation("uGround"), environment.ground.x, environment.ground.y, environment.ground.z);
    // 有环境贴图时环境光完全来自贴图，半球光不再参与
    p.setUniformFloat(p.uniformLocation("uAmbient"), env != nullptr ? 0.0f : environment.intensity);
    p.setUniformInt(p.uniformLocation("uIbl"), env != nullptr ? 1 : 0);
    p.setUniformFloat(p.uniformLocation("uIblIntensity"), imageLighting.intensity);
    setRotation(p, imageLighting.rotationRadians);
    p.setUniformInt(p.uniformLocation("uIrradiance"), 1);
    p.setUniformInt(p.uniformLocation("uPrefiltered"), 2);
    p.setUniformInt(p.uniformLocation("uDfg"), 3);
    if (env != nullptr) {
        p.setUniformFloat(p.uniformLocation("uPrefilteredMaxLod"), static_cast<float>(env->prefilteredMipCount() - 1));
        ctx.bindTexture(1, env->irradianceTexture());
        ctx.bindTexture(2, env->prefilteredTexture());
        ctx.bindTexture(3, env->dfgTexture());
        s.stats.imageLighting = true;
    }
    p.setUniformInt(p.uniformLocation("uShadows"), drawShadows ? 1 : 0);
    p.setUniformMatrix4(p.uniformLocation("uLightViewProj"), lvp.data());
    p.setUniformInt(p.uniformLocation("uShadowMap"), 0);
    p.setUniformFloat(p.uniformLocation("uNormalBias"), normalBias);
    p.setUniformFloat(p.uniformLocation("uTexel"), s.shadow ? 1.0f / static_cast<float>(s.shadow->width()) : 0.0f);
    if (s.shadow) ctx.bindTexture(0, s.shadow->depthTexture());
    const int modelLoc = p.uniformLocation("uModel");
    const int normalLoc = p.uniformLocation("uNormalMatrix");
    const int baseLoc = p.uniformLocation("uBaseColor");
    const int metallicLoc = p.uniformLocation("uMetallic");
    const int roughnessLoc = p.uniformLocation("uRoughness");
    const int reflectanceLoc = p.uniformLocation("uReflectance");
    const int emissiveLoc = p.uniformLocation("uEmissive");
    const int texturesLoc = p.uniformLocation("uTextures");
    const int normalScaleLoc = p.uniformLocation("uNormalScale");
    const int occlusionStrengthLoc = p.uniformLocation("uOcclusionStrength");
    const int doubleSidedLoc = p.uniformLocation("uDoubleSided");
    const int opacityLoc = p.uniformLocation("uOpacity");
    // 贴图单元：0 阴影、1–3 环境光照、4 环境立方体（背景）、5–9 材质贴图
    p.setUniformInt(p.uniformLocation("uBaseColorMap"), 5);
    p.setUniformInt(p.uniformLocation("uMetallicRoughnessMap"), 6);
    p.setUniformInt(p.uniformLocation("uNormalMap"), 7);
    p.setUniformInt(p.uniformLocation("uOcclusionMap"), 8);
    p.setUniformInt(p.uniformLocation("uEmissiveMap"), 9);
    std::vector<const RenderItem*> translucent;  // 半透明物体（M3-06）：在网格之后绘制
    const auto drawItem = [&](const RenderItem& item) {
        const glm::mat4 model = toGlm(item.model);
        const Matrix4 normalMatrix = fromGlm(glm::mat4(glm::transpose(glm::inverse(glm::mat3(model)))));
        const Material& m = item.material;
        p.setUniformMatrix4(modelLoc, item.model.data());
        p.setUniformMatrix4(normalLoc, normalMatrix.data());
        p.setUniformVec3(baseLoc, m.baseColor.x, m.baseColor.y, m.baseColor.z);
        p.setUniformFloat(metallicLoc, m.metallic);
        p.setUniformFloat(roughnessLoc, m.roughness);
        p.setUniformFloat(reflectanceLoc, m.reflectance);
        p.setUniformVec3(emissiveLoc, m.emissive.x, m.emissive.y, m.emissive.z);
        int mask = 0;
        const std::uint32_t maps[5] = {m.baseColorTexture, m.metallicRoughnessTexture, m.normalTexture, m.occlusionTexture,
                                       m.emissiveTexture};
        for (int k = 0; k < 5; ++k) {
            if (maps[k] == 0) continue;
            mask |= 1 << k;
            ctx.bindTexture(static_cast<std::uint32_t>(5 + k), maps[k]);
        }
        p.setUniformInt(texturesLoc, mask);
        p.setUniformFloat(normalScaleLoc, m.normalScale);
        p.setUniformFloat(occlusionStrengthLoc, m.occlusionStrength);
        p.setUniformInt(doubleSidedLoc, m.doubleSided ? 1 : 0);
        p.setUniformFloat(opacityLoc, std::clamp(m.opacity, 0.0f, 1.0f));
        ctx.setCullBackFaces(!m.doubleSided && m.opacity >= 1.0f);
        item.mesh->draw(ctx);
    };
    for (const RenderItem& item : items) {
        if (item.mesh == nullptr) continue;
        if (item.material.opacity < 1.0f) {
            translucent.push_back(&item);
            continue;
        }
        drawItem(item);
    }

    s.stamp(2);

    // ---- 环境背景：只填充没有物体的像素 ----
    if (env != nullptr && imageLighting.background) {
        ShaderProgram& b = s.background;
        b.use();
        const Matrix4 inv = fromGlm(glm::inverse(viewProj));
        b.setUniformMatrix4(b.uniformLocation("uInvViewProj"), inv.data());
        b.setUniformVec3(b.uniformLocation("uCamera"), view.cameraPosition.x, view.cameraPosition.y, view.cameraPosition.z);
        b.setUniformVec2(b.uniformLocation("uViewport"), static_cast<float>(s.width), static_cast<float>(s.height));
        b.setUniformInt(b.uniformLocation("uEnvironmentCube"), 4);
        b.setUniformFloat(b.uniformLocation("uLod"),
                          std::clamp(imageLighting.backgroundBlur, 0.0f, 1.0f) * static_cast<float>(env->cubeMipCount() - 1));
        b.setUniformFloat(b.uniformLocation("uIntensity"), imageLighting.intensity);
        setRotation(b, imageLighting.rotationRadians);
        ctx.bindTexture(4, env->cubeTexture());
        ctx.setCullBackFaces(false);
        ctx.setDepthLessEqual(true);
        ctx.setDepthWrite(false);
        s.empty.bind();
        ctx.drawTriangles(0, 3);
        ctx.setDepthWrite(true);
        ctx.setDepthLessEqual(false);
        s.stats.backgroundDrawn = true;
    }

    s.stamp(3);

    // ---- 地面网格：叠加在 z = 0，深度测试但不写深度 ----
    if (settings.grid) {
        constexpr float kExtent = 100.0f;
        ShaderProgram& g = s.grid;
        g.use();
        g.setUniformMatrix4(g.uniformLocation("uViewProj"), vp.data());
        g.setUniformVec3(g.uniformLocation("uCamera"), view.cameraPosition.x, view.cameraPosition.y, view.cameraPosition.z);
        // 以相机下方为中心，取整到 10 m，避免网格随相机平移闪烁
        const float cx = 10.0f * std::floor(view.cameraPosition.x / 10.0f);
        const float cy = 10.0f * std::floor(view.cameraPosition.y / 10.0f);
        g.setUniformVec3(g.uniformLocation("uCenter"), cx, cy, 0.0f);
        g.setUniformFloat(g.uniformLocation("uExtent"), kExtent);
        g.setUniformVec3(g.uniformLocation("uAxisX"), 0.75f, 0.12f, 0.12f);
        g.setUniformVec3(g.uniformLocation("uAxisY"), 0.25f, 0.60f, 0.12f);
        ctx.setCullBackFaces(false);
        ctx.setDepthLessEqual(true);
        ctx.setDepthWrite(false);
        ctx.setAlphaBlend(true);
        ctx.setPolygonOffset(true, -1.0f, -1.0f);
        s.empty.bind();
        ctx.drawTriangles(0, 6);
        ctx.setPolygonOffset(false);
        ctx.setAlphaBlend(false);
        ctx.setDepthWrite(true);
        ctx.setDepthLessEqual(false);
    }

    // ---- 半透明物体（M3-06，如叠加显示的碰撞几何体）：从远到近混合，不写深度；与同位置的不透明表面重合时仍可见 ----
    if (!translucent.empty()) {
        const glm::vec3 eye = toGlm(view.cameraPosition);
        std::stable_sort(translucent.begin(), translucent.end(), [&](const RenderItem* a, const RenderItem* b) {
            const glm::vec3 pa(a->model[12], a->model[13], a->model[14]);
            const glm::vec3 pb(b->model[12], b->model[13], b->model[14]);
            return glm::length(pa - eye) > glm::length(pb - eye);
        });
        p.use();
        ctx.setDepthLessEqual(true);
        ctx.setDepthWrite(false);
        ctx.setAlphaBlend(true);
        for (const RenderItem* item : translucent) drawItem(*item);
        ctx.setAlphaBlend(false);
        ctx.setDepthWrite(true);
        ctx.setDepthLessEqual(false);
        s.stats.translucentDrawn = static_cast<int>(translucent.size());
    }

    s.stamp(4);

    // ---- 解析与色调映射 ----
    const Framebuffer* source = &*s.hdr;
    if (s.samples > 1) {
        s.hdr->resolveTo(*s.resolved);
        source = &*s.resolved;
    }
    s.stamp(5);
    s.output->bind();
    ctx.setDepthTest(false);
    ctx.setCullBackFaces(false);
    ShaderProgram& t = s.toneMap;
    t.use();
    t.setUniformInt(t.uniformLocation("uHdr"), 0);
    t.setUniformInt(t.uniformLocation("uToneMapper"), toneMapperIndex(settings.toneMapper));
    t.setUniformFloat(t.uniformLocation("uExposure"), settings.exposureEv);
    ctx.bindTexture(0, source->colorTexture());
    s.empty.bind();
    ctx.drawTriangles(0, 3);
    s.stamp(6);
    if (s.current >= 0) {
        s.pending[s.current] = true;
        ++s.frame;
    }
    Framebuffer::bindDefault();
}

const char* renderPassName(RenderPass pass) noexcept {
    switch (pass) {
        case RenderPass::Shadow: return "shadow";
        case RenderPass::Opaque: return "opaque";
        case RenderPass::Background: return "background";
        case RenderPass::Grid: return "grid";
        case RenderPass::Resolve: return "resolve";
        case RenderPass::ToneMap: return "tonemap";
    }
    return "";
}

double PassTimings::total() const noexcept {
    double sum = 0.0;
    for (const double ms : milliseconds) sum += ms;
    return sum;
}

void SceneRenderer::setTimingEnabled(bool enabled) { impl_->timing = enabled; }
bool SceneRenderer::timingEnabled() const noexcept { return impl_->timing; }
const PassTimings& SceneRenderer::lastTimings() const noexcept { return impl_->last; }

PassTimings SceneRenderer::waitForTimings() {
    Impl& s = *impl_;
    const int slot = (s.frame - 1 + Impl::kRing * 1024) % Impl::kRing;
    if (s.queriesCreated && s.pending[slot]) (void)s.collect(slot, true);
    return s.last;
}

std::uint32_t SceneRenderer::outputTexture() const noexcept { return impl_->output ? impl_->output->colorTexture() : 0u; }
int SceneRenderer::width() const noexcept { return impl_->width; }
int SceneRenderer::height() const noexcept { return impl_->height; }
const FrameStats& SceneRenderer::lastFrame() const noexcept { return impl_->stats; }

std::vector<std::uint8_t> SceneRenderer::readOutput(int x, int y, int width, int height) const {
    return impl_->output ? impl_->output->readPixels(x, y, width, height) : std::vector<std::uint8_t>{};
}

}  // namespace terra::render
