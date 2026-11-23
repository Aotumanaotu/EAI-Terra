#include "terra/render/environment.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <utility>

#include <glad/gl.h>

#include "terra/render/context.hpp"
#include "terra/render/pbr.hpp"
#include "terra/render/resources.hpp"

namespace terra::render {

namespace {

const char* kFullscreenVertex = R"(#version 450 core
void main() {
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
)";

// 立方体面上一个片元对应的世界方向（OpenGL 立方体贴图的面约定：s、t 由主轴方向分量求得）。
const char* kFaceDirection = R"(
uniform int uFace;
uniform float uSize;
vec3 faceDirection(vec2 fragCoord) {
    vec2 c = fragCoord / uSize * 2.0 - 1.0;
    vec3 d;
    if (uFace == 0) d = vec3(1.0, -c.y, -c.x);
    else if (uFace == 1) d = vec3(-1.0, -c.y, c.x);
    else if (uFace == 2) d = vec3(c.x, 1.0, c.y);
    else if (uFace == 3) d = vec3(c.x, -1.0, -c.y);
    else if (uFace == 4) d = vec3(c.x, -c.y, 1.0);
    else d = vec3(-c.x, -c.y, -1.0);
    return normalize(d);
}
out vec4 fragColor;
)";

// 等距柱状 → 立方体：u = 0.5 + atan2(y, x) / 2π，v = acos(z) / π（第 0 行为天顶）。
const char* kEquirectMain = R"(
uniform sampler2D uEquirect;
void main() {
    vec3 d = faceDirection(gl_FragCoord.xy);
    vec2 uv = vec2(0.5 + atan(d.y, d.x) / (2.0 * PI), acos(clamp(d.z, -1.0, 1.0)) / PI);
    fragColor = vec4(textureLod(uEquirect, uv, 0.0).rgb, 1.0);
}
)";

// 辐照度：余弦加权半球积分，中点法均匀角度网格（32 × 128）；结果为 (1/π)∫L cosθ dω，即常数环境 L 得到 L。
const char* kIrradianceMain = R"(
uniform samplerCube uEnvironment;
uniform float uSourceLod;
void main() {
    vec3 n = faceDirection(gl_FragCoord.xy);
    vec3 up = abs(n.z) < 0.999 ? vec3(0.0, 0.0, 1.0) : vec3(1.0, 0.0, 0.0);
    vec3 right = normalize(cross(up, n));
    up = cross(n, right);
    const int thetaSteps = 32;
    const int phiSteps = 128;
    const float dTheta = 0.5 * PI / float(thetaSteps);
    const float dPhi = 2.0 * PI / float(phiSteps);
    vec3 sum = vec3(0.0);
    for (int i = 0; i < thetaSteps; ++i) {
        float theta = (float(i) + 0.5) * dTheta;
        float weight = cos(theta) * sin(theta);
        for (int j = 0; j < phiSteps; ++j) {
            float phi = (float(j) + 0.5) * dPhi;
            vec3 t = vec3(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
            vec3 s = t.x * right + t.y * up + t.z * n;
            sum += textureLod(uEnvironment, s, uSourceLod).rgb * weight;
        }
    }
    fragColor = vec4(sum * dTheta * dPhi / PI, 1.0);
}
)";

// 预过滤：N = V = R 近似，GGX 重要性采样；按样本概率密度选择源 mip，减少亮点噪声。
const char* kPrefilterMain = R"(
uniform samplerCube uEnvironment;
uniform float uRoughness;
uniform float uSourceSize;
uniform int uSamples;

float radicalInverse(uint bits) { return float(bitfieldReverse(bits)) * 2.3283064365386963e-10; }

void main() {
    vec3 n = faceDirection(gl_FragCoord.xy);
    if (uRoughness <= 0.0) {
        fragColor = vec4(textureLod(uEnvironment, n, 0.0).rgb, 1.0);
        return;
    }
    vec3 up = abs(n.z) < 0.999 ? vec3(0.0, 0.0, 1.0) : vec3(1.0, 0.0, 0.0);
    vec3 tangent = normalize(cross(up, n));
    vec3 bitangent = cross(n, tangent);
    float a = uRoughness * uRoughness;
    float texelSolidAngle = 4.0 * PI / (6.0 * uSourceSize * uSourceSize);
    vec3 sum = vec3(0.0);
    float weight = 0.0;
    for (int i = 0; i < uSamples; ++i) {
        vec2 xi = vec2(float(i) / float(uSamples), radicalInverse(uint(i)));
        float phi = 2.0 * PI * xi.x;
        float cosTheta = sqrt((1.0 - xi.y) / (1.0 + (a * a - 1.0) * xi.y));
        float sinTheta = sqrt(max(0.0, 1.0 - cosTheta * cosTheta));
        vec3 h = normalize(tangent * (sinTheta * cos(phi)) + bitangent * (sinTheta * sin(phi)) + n * cosTheta);
        vec3 l = normalize(2.0 * dot(n, h) * h - n);
        float NoL = dot(n, l);
        if (NoL > 0.0) {
            float NoH = max(dot(n, h), 0.0);
            float pdf = D_GGX(NoH, a) * 0.25 + 1e-4;  // N = V 时 NoH / (4 VoH) = 1/4
            float sampleSolidAngle = 1.0 / (float(uSamples) * pdf);
            float lod = max(0.5 * log2(sampleSolidAngle / texelSolidAngle) + 1.0, 0.0);
            sum += textureLod(uEnvironment, l, lod).rgb * NoL;
            weight += NoL;
        }
    }
    fragColor = vec4(sum / max(weight, 1e-4), 1.0);
}
)";

int mipCount(int size) {
    int levels = 1;
    while ((size >> levels) > 0) ++levels;
    return levels;
}

GLuint makeCube(int size, int levels, GLenum minFilter) {
    GLuint tex = 0;
    glCreateTextures(GL_TEXTURE_CUBE_MAP, 1, &tex);
    glTextureStorage2D(tex, levels, GL_RGBA16F, size, size);
    glTextureParameteri(tex, GL_TEXTURE_MIN_FILTER, static_cast<GLint>(minFilter));
    glTextureParameteri(tex, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTextureParameteri(tex, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTextureParameteri(tex, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTextureParameteri(tex, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glTextureParameteri(tex, GL_TEXTURE_BASE_LEVEL, 0);
    glTextureParameteri(tex, GL_TEXTURE_MAX_LEVEL, levels - 1);
    return tex;
}

}  // namespace

struct Environment::Impl {
    GLuint cube = 0;
    GLuint irradiance = 0;
    GLuint prefiltered = 0;
    GLuint dfg = 0;
    int cubeMips = 1;
    int prefilteredMips = 1;
    std::vector<float> lut;
    int lutSize = 0;
    double seconds = 0.0;

    ~Impl() {
        for (GLuint* t : {&cube, &irradiance, &prefiltered, &dfg}) {
            if (*t != 0) glDeleteTextures(1, t);
        }
    }
};

Environment::Environment(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
Environment::~Environment() = default;

EnvironmentResult Environment::create(RenderContext& context, const HdrImage& image, const EnvironmentOptions& o) {
    (void)context;  // 只要求调用方的上下文为当前上下文；渲染状态直接设置并在结束时恢复默认帧缓冲
    EnvironmentResult result;
    if (image.width <= 0 || image.height <= 0 ||
        image.rgb.size() < static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height) * 3u) {
        result.error = "环境贴图为空或数据不完整";
        return result;
    }
    const auto start = std::chrono::steady_clock::now();
    auto impl = std::make_unique<Impl>();

    // ---- 着色器 ----
    const std::string prefix = std::string{"#version 450 core\n"} + pbr::shaderLibrary() + kFaceDirection;
    ShaderProgram equirect;
    ShaderProgram irradiance;
    ShaderProgram prefilter;
    const auto build = [&](ShaderProgram& target, const char* name, const char* main) {
        auto made = ShaderProgram::create({{ShaderStage::Vertex, "fullscreen.vert", kFullscreenVertex},
                                           {ShaderStage::Fragment, name, prefix + main}});
        if (!made.program) {
            if (!result.error) result.error = made.error ? made.error->summary() : std::string{"着色器创建失败"};
            return;
        }
        target = std::move(*made.program);
    };
    build(equirect, "env_equirect.frag", kEquirectMain);
    build(irradiance, "env_irradiance.frag", kIrradianceMain);
    build(prefilter, "env_prefilter.frag", kPrefilterMain);
    if (result.error) return result;

    glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glDisable(GL_BLEND);
    VertexArray empty = VertexArray::create();
    empty.bind();
    GLuint fbo = 0;
    glCreateFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    const auto renderFaces = [&](ShaderProgram& program, GLuint target, int level, int size) {
        program.setUniformFloat(program.uniformLocation("uSize"), static_cast<float>(size));
        const int faceLoc = program.uniformLocation("uFace");
        glViewport(0, 0, size, size);
        for (int face = 0; face < 6; ++face) {
            glNamedFramebufferTextureLayer(fbo, GL_COLOR_ATTACHMENT0, target, level, face);
            program.setUniformInt(faceLoc, face);
            glDrawArrays(GL_TRIANGLES, 0, 3);
        }
    };

    // ---- 1. 等距柱状 → 环境立方体（带 mip）----
    GLuint source = 0;
    glCreateTextures(GL_TEXTURE_2D, 1, &source);
    glTextureStorage2D(source, 1, GL_RGB32F, image.width, image.height);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    glTextureSubImage2D(source, 0, 0, 0, image.width, image.height, GL_RGB, GL_FLOAT, image.rgb.data());
    glTextureParameteri(source, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTextureParameteri(source, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTextureParameteri(source, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTextureParameteri(source, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    const int cubeSize = std::max(o.cubeSize, 8);
    impl->cubeMips = mipCount(cubeSize);
    impl->cube = makeCube(cubeSize, impl->cubeMips, GL_LINEAR_MIPMAP_LINEAR);
    equirect.use();
    equirect.setUniformInt(equirect.uniformLocation("uEquirect"), 0);
    glBindTextureUnit(0, source);
    renderFaces(equirect, impl->cube, 0, cubeSize);
    glGenerateTextureMipmap(impl->cube);
    glDeleteTextures(1, &source);

    // ---- 2. 辐照度 ----
    const int irrSize = std::max(o.irradianceSize, 4);
    impl->irradiance = makeCube(irrSize, 1, GL_LINEAR);
    irradiance.use();
    irradiance.setUniformInt(irradiance.uniformLocation("uEnvironment"), 0);
    // 源 mip：面约 32 像素，足够平滑又不丢失大尺度明暗
    const float sourceLod = std::max(0.0f, std::log2(static_cast<float>(cubeSize) / 32.0f));
    irradiance.setUniformFloat(irradiance.uniformLocation("uSourceLod"), sourceLod);
    glBindTextureUnit(0, impl->cube);
    renderFaces(irradiance, impl->irradiance, 0, irrSize);

    // ---- 3. 预过滤镜面 ----
    const int preSize = std::max(o.prefilterSize, 4);
    impl->prefilteredMips = std::clamp(o.prefilterMips, 1, mipCount(preSize));
    impl->prefiltered = makeCube(preSize, impl->prefilteredMips, GL_LINEAR_MIPMAP_LINEAR);
    prefilter.use();
    prefilter.setUniformInt(prefilter.uniformLocation("uEnvironment"), 0);
    prefilter.setUniformFloat(prefilter.uniformLocation("uSourceSize"), static_cast<float>(cubeSize));
    prefilter.setUniformInt(prefilter.uniformLocation("uSamples"), std::max(o.prefilterSamples, 1));
    for (int level = 0; level < impl->prefilteredMips; ++level) {
        const float roughness = impl->prefilteredMips > 1 ? static_cast<float>(level) / static_cast<float>(impl->prefilteredMips - 1) : 0.0f;
        prefilter.setUniformFloat(prefilter.uniformLocation("uRoughness"), roughness);
        renderFaces(prefilter, impl->prefiltered, level, std::max(preSize >> level, 1));
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteFramebuffers(1, &fbo);

    // ---- 4. DFG 查找表（CPU 计算后上传）----
    impl->lutSize = std::max(o.dfgSize, 2);
    impl->lut = pbr::dfgLut(impl->lutSize, o.dfgSamples);
    glCreateTextures(GL_TEXTURE_2D, 1, &impl->dfg);
    glTextureStorage2D(impl->dfg, 1, GL_RG32F, impl->lutSize, impl->lutSize);
    glTextureSubImage2D(impl->dfg, 0, 0, 0, impl->lutSize, impl->lutSize, GL_RG, GL_FLOAT, impl->lut.data());
    glTextureParameteri(impl->dfg, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTextureParameteri(impl->dfg, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTextureParameteri(impl->dfg, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTextureParameteri(impl->dfg, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glFinish();
    impl->seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    result.environment.reset(new Environment(std::move(impl)));
    return result;
}

std::uint32_t Environment::cubeTexture() const noexcept { return impl_->cube; }
std::uint32_t Environment::irradianceTexture() const noexcept { return impl_->irradiance; }
std::uint32_t Environment::prefilteredTexture() const noexcept { return impl_->prefiltered; }
std::uint32_t Environment::dfgTexture() const noexcept { return impl_->dfg; }
int Environment::cubeMipCount() const noexcept { return impl_->cubeMips; }
int Environment::prefilteredMipCount() const noexcept { return impl_->prefilteredMips; }
const std::vector<float>& Environment::dfgLut() const noexcept { return impl_->lut; }
int Environment::dfgSize() const noexcept { return impl_->lutSize; }
double Environment::precomputeSeconds() const noexcept { return impl_->seconds; }

}  // namespace terra::render
