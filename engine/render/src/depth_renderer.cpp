#include "terra/render/depth_renderer.hpp"

#include <cmath>
#include <limits>
#include <utility>

#include "terra/render/context.hpp"
#include "terra/render/resources.hpp"

namespace terra::render {

namespace {

// 片元写出相机坐标系中的 −z（插值是透视正确的，命中平面时为常数）
const char* kLinearDepthVertex = R"(#version 450 core
layout(location = 0) in vec3 aPosition;
uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;
out float vDepth;
void main() {
    vec4 viewPosition = uView * uModel * vec4(aPosition, 1.0);
    vDepth = -viewPosition.z;
    gl_Position = uProjection * viewPosition;
}
)";

const char* kLinearDepthFragment = R"(#version 450 core
in float vDepth;
layout(location = 0) out float oDepth;
void main() { oDepth = vDepth; }
)";

// M4-09：分割图（平涂写出渲染项的 id）
const char* kIdVertex = R"(#version 450 core
layout(location = 0) in vec3 aPosition;
uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;
void main() { gl_Position = uProjection * uView * uModel * vec4(aPosition, 1.0); }
)";

const char* kIdFragment = R"(#version 450 core
uniform float uId;
layout(location = 0) out float oId;
void main() { oId = uId; }
)";

}  // namespace

struct DepthRenderer::Impl {
    explicit Impl(RenderContext& c) : ctx(c) {}
    RenderContext& ctx;
    ShaderProgram program;
    int model = -1;
    int view = -1;
    int projection = -1;
    ShaderProgram idProgram;  // M4-09
    int idModel = -1;
    int idView = -1;
    int idProjection = -1;
    int idValue = -1;
    std::optional<Framebuffer> target;  // R32F 颜色 + 深度缓冲

    bool ensureTarget(int width, int height) {
        if (target && target->width() == width && target->height() == height) return true;
        target.reset();
        FramebufferDesc desc;
        desc.width = width;
        desc.height = height;
        desc.color = ColorFormat::R32F;
        desc.depth = DepthFormat::Renderbuffer;
        auto made = Framebuffer::create(desc);
        if (!made.framebuffer) return false;
        target = std::move(*made.framebuffer);
        return true;
    }
};

DepthRenderer::DepthRenderer(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
DepthRenderer::~DepthRenderer() = default;

DepthRendererResult DepthRenderer::create(RenderContext& context) {
    DepthRendererResult result;
    auto made = ShaderProgram::create({{ShaderStage::Vertex, "linear_depth.vert", kLinearDepthVertex},
                                       {ShaderStage::Fragment, "linear_depth.frag", kLinearDepthFragment}});
    if (!made.program) {
        result.error = made.error ? made.error->summary() : std::string{"着色器创建失败"};
        return result;
    }
    auto ids = ShaderProgram::create({{ShaderStage::Vertex, "segmentation.vert", kIdVertex}, {ShaderStage::Fragment, "segmentation.frag", kIdFragment}});
    if (!ids.program) {
        result.error = ids.error ? ids.error->summary() : std::string{"着色器创建失败"};
        return result;
    }
    auto impl = std::make_unique<Impl>(context);
    impl->program = std::move(*made.program);
    impl->idProgram = std::move(*ids.program);
    impl->idModel = impl->idProgram.uniformLocation("uModel");
    impl->idView = impl->idProgram.uniformLocation("uView");
    impl->idProjection = impl->idProgram.uniformLocation("uProjection");
    impl->idValue = impl->idProgram.uniformLocation("uId");
    impl->model = impl->program.uniformLocation("uModel");
    impl->view = impl->program.uniformLocation("uView");
    impl->projection = impl->program.uniformLocation("uProjection");
    result.renderer.reset(new DepthRenderer(std::move(impl)));
    return result;
}

int DepthRenderer::targetCount() const noexcept { return impl_->target ? 1 : 0; }

std::vector<float> DepthRenderer::render(int width, int height, const ViewParams& view, const std::vector<RenderItem>& items) {
    Impl& d = *impl_;
    if (width <= 0 || height <= 0) return {};
    if (!d.ensureTarget(width, height)) return {};
    d.target->bind();
    // 清为 0：近平面大于 0，任何命中都大于 0；读回时把 0 换成 +inf
    d.ctx.setDepthWrite(true);
    d.ctx.setDepthTest(true);
    d.ctx.clear(Color{0.0f, 0.0f, 0.0f, 0.0f}, true);
    d.ctx.setCullBackFaces(false);
    d.ctx.setAlphaBlend(false);
    d.program.use();
    d.program.setUniformMatrix4(d.view, view.view.data());
    d.program.setUniformMatrix4(d.projection, view.projection.data());
    for (const RenderItem& item : items) {
        if (item.mesh == nullptr || item.material.opacity < 1.0f) continue;
        d.program.setUniformMatrix4(d.model, item.model.data());
        item.mesh->draw(d.ctx);
    }
    std::vector<float> bottomUp = d.target->readFloats(0, 0, width, height);
    Framebuffer::bindDefault();
    // 行序转为图像约定（行 0 为顶部），没有命中的像素为 +inf
    std::vector<float> depth(bottomUp.size());
    const auto w = static_cast<std::size_t>(width);
    const auto h = static_cast<std::size_t>(height);
    for (std::size_t row = 0; row < h && bottomUp.size() == w * h; ++row) {
        for (std::size_t col = 0; col < w; ++col) {
            const float v = bottomUp[(h - 1 - row) * w + col];
            depth[row * w + col] = v > 0.0f ? v : std::numeric_limits<float>::infinity();
        }
    }
    return depth;
}

std::vector<std::uint32_t> DepthRenderer::renderIds(int width, int height, const ViewParams& view, const std::vector<RenderItem>& items) {
    Impl& d = *impl_;
    if (width <= 0 || height <= 0 || !d.ensureTarget(width, height)) return {};
    d.target->bind();
    d.ctx.setDepthWrite(true);
    d.ctx.setDepthTest(true);
    d.ctx.clear(Color{0.0f, 0.0f, 0.0f, 0.0f}, true);  // 0 为背景
    d.ctx.setCullBackFaces(false);
    d.ctx.setAlphaBlend(false);
    d.idProgram.use();
    d.idProgram.setUniformMatrix4(d.idView, view.view.data());
    d.idProgram.setUniformMatrix4(d.idProjection, view.projection.data());
    for (const RenderItem& item : items) {
        if (item.mesh == nullptr || item.material.opacity < 1.0f) continue;
        d.idProgram.setUniformMatrix4(d.idModel, item.model.data());
        d.idProgram.setUniformFloat(d.idValue, static_cast<float>(item.id));
        item.mesh->draw(d.ctx);
    }
    const std::vector<float> bottomUp = d.target->readFloats(0, 0, width, height);
    Framebuffer::bindDefault();
    const auto w = static_cast<std::size_t>(width);
    const auto h = static_cast<std::size_t>(height);
    std::vector<std::uint32_t> ids(w * h, 0);
    for (std::size_t row = 0; row < h && bottomUp.size() == w * h; ++row) {
        for (std::size_t col = 0; col < w; ++col) {
            const float v = bottomUp[(h - 1 - row) * w + col];
            ids[row * w + col] = v > 0.5f ? static_cast<std::uint32_t>(std::lround(v)) : 0u;
        }
    }
    return ids;
}

}  // namespace terra::render
