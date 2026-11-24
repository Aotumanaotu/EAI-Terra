#include "terra/render/depth_renderer.hpp"

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

}  // namespace

struct DepthRenderer::Impl {
    explicit Impl(RenderContext& c) : ctx(c) {}
    RenderContext& ctx;
    ShaderProgram program;
    int model = -1;
    int view = -1;
    int projection = -1;
    std::optional<Framebuffer> target;  // R32F 颜色 + 深度缓冲
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
    auto impl = std::make_unique<Impl>(context);
    impl->program = std::move(*made.program);
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
    if (!d.target || d.target->width() != width || d.target->height() != height) {
        d.target.reset();
        FramebufferDesc desc;
        desc.width = width;
        desc.height = height;
        desc.color = ColorFormat::R32F;
        desc.depth = DepthFormat::Renderbuffer;
        auto made = Framebuffer::create(desc);
        if (!made.framebuffer) return {};
        d.target = std::move(*made.framebuffer);
    }
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

}  // namespace terra::render
