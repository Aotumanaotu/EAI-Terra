#include "terra/render/scene_renderer.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

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
uniform mat4 uViewProj;
out vec3 vWorld;
out vec3 vNormal;
void main() {
    vec4 world = uModel * vec4(aPosition, 1.0);
    vWorld = world.xyz;
    vNormal = mat3(uNormalMatrix) * aNormal;
    gl_Position = uViewProj * world;
}
)";

const char* kPbrFragmentMain = R"(
in vec3 vWorld;
in vec3 vNormal;
out vec4 fragColor;
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

void main() {
    vec3 n = normalize(vNormal);
    vec3 v = normalize(uCamera - vWorld);
    float shadow = shadowFactor(vWorld, n);
    vec3 color = shadeSurface(uBaseColor, uMetallic, uRoughness, uReflectance, uEmissive, n, v, uSunDir, uSunRadiance, shadow);
    fragColor = vec4(color, 1.0);
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
    VertexArray empty;  // 全屏三角形与网格四边形由 gl_VertexID 生成，核心模式仍需绑定一个顶点数组
    std::optional<Framebuffer> hdr;       // 场景（可多重采样）
    std::optional<Framebuffer> resolved;  // 多重采样解析结果
    std::optional<Framebuffer> output;    // RGBA8 显示结果
    std::optional<Framebuffer> shadow;
    int width = 0;
    int height = 0;
    int samples = 1;
    FrameStats stats;

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
    if (result.error) return result;
    impl->empty = VertexArray::create();
    result.renderer.reset(new SceneRenderer(std::move(impl)));
    return result;
}

void SceneRenderer::render(int width, int height, const ViewParams& view, const std::vector<RenderItem>& items,
                           const SunLight& sun, const HemisphereLight& environment, const RenderSettings& settings) {
    Impl& s = *impl_;
    RenderContext& ctx = s.ctx;
    s.stats = FrameStats{};
    if (width <= 0 || height <= 0) return;
    s.ensureTargets(width, height, settings.msaaSamples);
    if (!s.hdr || !s.output || (s.samples > 1 && !s.resolved)) return;
    s.stats.samples = s.samples;

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
    p.setUniformFloat(p.uniformLocation("uAmbient"), environment.intensity);
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
    for (const RenderItem& item : items) {
        if (item.mesh == nullptr) continue;
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
        item.mesh->draw(ctx);
    }

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

    // ---- 解析与色调映射 ----
    const Framebuffer* source = &*s.hdr;
    if (s.samples > 1) {
        s.hdr->resolveTo(*s.resolved);
        source = &*s.resolved;
    }
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
    Framebuffer::bindDefault();
}

std::uint32_t SceneRenderer::outputTexture() const noexcept { return impl_->output ? impl_->output->colorTexture() : 0u; }
int SceneRenderer::width() const noexcept { return impl_->width; }
int SceneRenderer::height() const noexcept { return impl_->height; }
const FrameStats& SceneRenderer::lastFrame() const noexcept { return impl_->stats; }

std::vector<std::uint8_t> SceneRenderer::readOutput(int x, int y, int width, int height) const {
    return impl_->output ? impl_->output->readPixels(x, y, width, height) : std::vector<std::uint8_t>{};
}

}  // namespace terra::render
