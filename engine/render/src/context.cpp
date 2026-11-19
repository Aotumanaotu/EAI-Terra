#include "terra/render/context.hpp"

#include <utility>

#include <glad/gl.h>

namespace terra::render {

namespace {

// 调试回调写入的位置；地址在上下文生命周期内保持不变（随 Impl 分配在堆上）。
struct DebugSink {
    std::vector<DebugMessage> messages;
    std::size_t errorCount = 0;
};

}  // namespace

struct RenderContext::Impl {
    GlInfo info;
    DebugSink debug;
};

namespace {

std::string glString(GLenum name) {
    const GLubyte* value = glGetString(name);
    return value != nullptr ? std::string{reinterpret_cast<const char*>(value)} : std::string{};
}

DebugSeverity toSeverity(GLenum severity) {
    switch (severity) {
        case GL_DEBUG_SEVERITY_HIGH: return DebugSeverity::High;
        case GL_DEBUG_SEVERITY_MEDIUM: return DebugSeverity::Medium;
        case GL_DEBUG_SEVERITY_LOW: return DebugSeverity::Low;
        default: return DebugSeverity::Notification;
    }
}

// 同步调试输出：回调在触发它的 GL 调用返回前执行，消息可以对应到具体调用。
void GLAD_API_PTR onDebugMessage(GLenum /*source*/, GLenum type, GLuint id, GLenum severity, GLsizei length,
                                 const GLchar* message, const void* userParam) {
    auto* sink = static_cast<DebugSink*>(const_cast<void*>(userParam));
    if (sink == nullptr || message == nullptr) return;
    DebugMessage m;
    m.severity = toSeverity(severity);
    m.isError = type == GL_DEBUG_TYPE_ERROR;
    m.id = id;
    m.text = length >= 0 ? std::string{message, static_cast<std::size_t>(length)} : std::string{message};
    if (m.isError) ++sink->errorCount;
    sink->messages.push_back(std::move(m));
}

}  // namespace

ContextResult RenderContext::create(GlLoader loader) {
    ContextResult result;
    if (loader == nullptr) {
        result.error = "OpenGL 函数加载器为空";
        return result;
    }
    const int version = gladLoadGL(loader);
    if (version == 0) {
        result.error = "加载 OpenGL 函数失败（当前线程是否有当前上下文？）";
        return result;
    }
    auto impl = std::make_unique<Impl>();
    GlInfo& gl = impl->info;
    gl.major = GLAD_VERSION_MAJOR(version);
    gl.minor = GLAD_VERSION_MINOR(version);
    gl.vendor = glString(GL_VENDOR);
    gl.renderer = glString(GL_RENDERER);
    gl.version = glString(GL_VERSION);
    gl.glslVersion = glString(GL_SHADING_LANGUAGE_VERSION);
    {
        GLint samples = 1;
        glGetIntegerv(GL_MAX_SAMPLES, &samples);
        gl.maxSamples = samples > 0 ? samples : 1;
    }
    if (gl.major < 4 || (gl.major == 4 && gl.minor < 5)) {
        result.error = "OpenGL 版本 " + std::to_string(gl.major) + "." + std::to_string(gl.minor) +
                       " 低于 Terra 要求的 4.5（D-021）；渲染器：" + gl.renderer;
        return result;
    }
    GLint profile = 0;
    glGetIntegerv(GL_CONTEXT_PROFILE_MASK, &profile);
    if ((profile & GL_CONTEXT_CORE_PROFILE_BIT) == 0) {
        result.error = "当前上下文不是 core profile";
        return result;
    }

    GLint flags = 0;
    glGetIntegerv(GL_CONTEXT_FLAGS, &flags);
    gl.debugContext = (flags & GL_CONTEXT_FLAG_DEBUG_BIT) != 0;
    if (gl.debugContext) {
        glEnable(GL_DEBUG_OUTPUT);
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
        glDebugMessageCallback(&onDebugMessage, &impl->debug);
        // 通知级消息（如缓冲放在哪块内存）数量大且无诊断价值，丢弃。
        glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DEBUG_SEVERITY_NOTIFICATION, 0, nullptr, GL_FALSE);
    }
    result.context.reset(new RenderContext(std::move(impl)));
    return result;
}

RenderContext::RenderContext(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

RenderContext::~RenderContext() {
    if (impl_ && impl_->info.debugContext) {
        glDebugMessageCallback(nullptr, nullptr);
        glDisable(GL_DEBUG_OUTPUT);
    }
}

const GlInfo& RenderContext::info() const noexcept { return impl_->info; }

std::vector<DebugMessage> RenderContext::takeDebugMessages() {
    std::vector<DebugMessage> out;
    out.swap(impl_->debug.messages);
    return out;
}

std::size_t RenderContext::errorMessageCount() const noexcept { return impl_->debug.errorCount; }

void RenderContext::setViewport(int x, int y, int width, int height) { glViewport(x, y, width, height); }

void RenderContext::setDepthTest(bool enabled) {
    if (enabled) {
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glDepthMask(GL_TRUE);
    } else {
        glDisable(GL_DEPTH_TEST);
    }
}

void RenderContext::clear(Color color, bool clearDepth) {
    GLbitfield mask = GL_COLOR_BUFFER_BIT;
    if (clearDepth) {
        glDepthMask(GL_TRUE);  // 深度写入关闭时 glClear 不会清除深度
        glClearDepth(1.0);
        mask |= GL_DEPTH_BUFFER_BIT;
    }
    glClearColor(color.r, color.g, color.b, color.a);
    glClear(mask);
}

void RenderContext::drawTriangles(int firstVertex, int vertexCount) {
    glDrawArrays(GL_TRIANGLES, firstVertex, vertexCount);
}

void RenderContext::drawLines(int firstVertex, int vertexCount) {
    glDrawArrays(GL_LINES, firstVertex, vertexCount);
}

void RenderContext::drawIndexed(int indexCount) {
    glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, nullptr);
}

void RenderContext::setCullBackFaces(bool enabled) {
    if (enabled) {
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glFrontFace(GL_CCW);
    } else {
        glDisable(GL_CULL_FACE);
    }
}

void RenderContext::setDepthWrite(bool enabled) { glDepthMask(enabled ? GL_TRUE : GL_FALSE); }

void RenderContext::setDepthLessEqual(bool enabled) { glDepthFunc(enabled ? GL_LEQUAL : GL_LESS); }

void RenderContext::setAlphaBlend(bool enabled) {
    if (enabled) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    } else {
        glDisable(GL_BLEND);
    }
}

void RenderContext::setPolygonOffset(bool enabled, float factor, float units) {
    if (enabled) {
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(factor, units);
    } else {
        glDisable(GL_POLYGON_OFFSET_FILL);
    }
}

void RenderContext::bindTexture(std::uint32_t unit, std::uint32_t texture) { glBindTextureUnit(unit, texture); }

std::vector<std::uint8_t> RenderContext::readPixels(int x, int y, int width, int height) const {
    std::vector<std::uint8_t> pixels;
    if (width <= 0 || height <= 0) return pixels;
    pixels.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4u);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadnPixels(x, y, width, height, GL_RGBA, GL_UNSIGNED_BYTE, static_cast<GLsizei>(pixels.size()), pixels.data());
    return pixels;
}

}  // namespace terra::render
