#include "terra/render/resources.hpp"

#include <cstdio>
#include <regex>
#include <utility>

#include <glad/gl.h>

namespace terra::render {

// ---- Buffer ----

Buffer Buffer::create(std::size_t bytes, const void* data, bool dynamic) {
    Buffer b;
    if (bytes == 0) return b;  // GL 不允许零大小的存储；返回空缓冲
    glCreateBuffers(1, &b.name_);
    const GLbitfield flags = dynamic ? GL_DYNAMIC_STORAGE_BIT : 0u;
    glNamedBufferStorage(b.name_, static_cast<GLsizeiptr>(bytes), data, flags);
    b.size_ = bytes;
    b.dynamic_ = dynamic;
    return b;
}

Buffer::~Buffer() {
    if (name_ != 0) glDeleteBuffers(1, &name_);
}

Buffer::Buffer(Buffer&& other) noexcept
    : name_(std::exchange(other.name_, 0u)), size_(std::exchange(other.size_, 0u)),
      dynamic_(std::exchange(other.dynamic_, false)) {}

Buffer& Buffer::operator=(Buffer&& other) noexcept {
    if (this != &other) {
        if (name_ != 0) glDeleteBuffers(1, &name_);
        name_ = std::exchange(other.name_, 0u);
        size_ = std::exchange(other.size_, 0u);
        dynamic_ = std::exchange(other.dynamic_, false);
    }
    return *this;
}

bool Buffer::update(std::size_t offset, std::size_t bytes, const void* data) {
    if (name_ == 0 || !dynamic_ || offset > size_ || bytes > size_ - offset) return false;
    glNamedBufferSubData(name_, static_cast<GLintptr>(offset), static_cast<GLsizeiptr>(bytes), data);
    return true;
}

// ---- Texture2D ----

Texture2D Texture2D::create(int width, int height, const std::uint8_t* pixels, bool srgb) {
    Texture2D t;
    if (width <= 0 || height <= 0 || pixels == nullptr) return t;
    int levels = 1;
    while (((width > height ? width : height) >> levels) > 0) ++levels;
    glCreateTextures(GL_TEXTURE_2D, 1, &t.name_);
    glTextureStorage2D(t.name_, levels, srgb ? GL_SRGB8_ALPHA8 : GL_RGBA8, width, height);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTextureSubImage2D(t.name_, 0, 0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
    glGenerateTextureMipmap(t.name_);
    glTextureParameteri(t.name_, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTextureParameteri(t.name_, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTextureParameteri(t.name_, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTextureParameteri(t.name_, GL_TEXTURE_WRAP_T, GL_REPEAT);
    t.width_ = width;
    t.height_ = height;
    return t;
}

Texture2D::~Texture2D() {
    if (name_ != 0) glDeleteTextures(1, &name_);
}

Texture2D::Texture2D(Texture2D&& other) noexcept
    : name_(std::exchange(other.name_, 0u)), width_(std::exchange(other.width_, 0)), height_(std::exchange(other.height_, 0)) {}

Texture2D& Texture2D::operator=(Texture2D&& other) noexcept {
    if (this != &other) {
        if (name_ != 0) glDeleteTextures(1, &name_);
        name_ = std::exchange(other.name_, 0u);
        width_ = std::exchange(other.width_, 0);
        height_ = std::exchange(other.height_, 0);
    }
    return *this;
}

// ---- VertexArray ----

VertexArray VertexArray::create() {
    VertexArray v;
    glCreateVertexArrays(1, &v.name_);
    return v;
}

VertexArray::~VertexArray() {
    if (name_ != 0) glDeleteVertexArrays(1, &name_);
}

VertexArray::VertexArray(VertexArray&& other) noexcept : name_(std::exchange(other.name_, 0u)) {}

VertexArray& VertexArray::operator=(VertexArray&& other) noexcept {
    if (this != &other) {
        if (name_ != 0) glDeleteVertexArrays(1, &name_);
        name_ = std::exchange(other.name_, 0u);
    }
    return *this;
}

void VertexArray::setVertexBuffer(std::uint32_t binding, const Buffer& buffer, std::size_t offset, std::size_t stride) {
    glVertexArrayVertexBuffer(name_, binding, buffer.name(), static_cast<GLintptr>(offset), static_cast<GLsizei>(stride));
}

void VertexArray::setFloatAttribute(std::uint32_t location, std::uint32_t binding, int components, std::size_t offset) {
    glEnableVertexArrayAttrib(name_, location);
    glVertexArrayAttribFormat(name_, location, components, GL_FLOAT, GL_FALSE, static_cast<GLuint>(offset));
    glVertexArrayAttribBinding(name_, location, binding);
}

void VertexArray::setIndexBuffer(const Buffer& buffer) { glVertexArrayElementBuffer(name_, buffer.name()); }

void VertexArray::bind() const { glBindVertexArray(name_); }

// ---- ShaderProgram ----

std::optional<int> parseShaderErrorLine(const std::string& log) {
    // Mesa:   "0:4(12): error: ..."      NVIDIA: "0(4) : error C0000: ..."
    static const std::regex mesa{R"(^\s*\d+:(\d+)\(\d+\):\s*error)"};
    static const std::regex nvidia{R"(^\s*\d+\((\d+)\)\s*:\s*error)"};
    std::size_t begin = 0;
    while (begin < log.size()) {
        std::size_t end = log.find('\n', begin);
        if (end == std::string::npos) end = log.size();
        const std::string line = log.substr(begin, end - begin);
        std::smatch m;
        if (std::regex_search(line, m, mesa) || std::regex_search(line, m, nvidia)) {
            return std::stoi(m[1].str());
        }
        begin = end + 1;
    }
    return std::nullopt;
}

std::string ShaderError::summary() const {
    std::string first = log.substr(0, log.find('\n'));
    const char* what = kind == Kind::Compile ? "编译失败" : kind == Kind::Link ? "链接失败" : "输入无效";
    return name + (line ? ":" + std::to_string(*line) : std::string{}) + ": " + what + "：" + first;
}

namespace {

std::string shaderLog(GLuint shader) {
    GLint length = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
    std::string log(static_cast<std::size_t>(length > 0 ? length : 0), '\0');
    if (length > 0) glGetShaderInfoLog(shader, length, nullptr, log.data());
    while (!log.empty() && (log.back() == '\0' || log.back() == '\n')) log.pop_back();
    return log;
}

std::string programLog(GLuint program) {
    GLint length = 0;
    glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
    std::string log(static_cast<std::size_t>(length > 0 ? length : 0), '\0');
    if (length > 0) glGetProgramInfoLog(program, length, nullptr, log.data());
    while (!log.empty() && (log.back() == '\0' || log.back() == '\n')) log.pop_back();
    return log;
}

// 编译成功返回着色器名称，失败返回 0 并填写 error。
GLuint compile(const ShaderSource& s, std::optional<ShaderError>& error) {
    const GLuint shader = glCreateShader(s.stage == ShaderStage::Vertex ? GL_VERTEX_SHADER : GL_FRAGMENT_SHADER);
    const GLchar* text = s.source.c_str();
    const auto length = static_cast<GLint>(s.source.size());
    glShaderSource(shader, 1, &text, &length);
    glCompileShader(shader);
    GLint ok = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (ok != GL_TRUE) {
        ShaderError e;
        e.kind = ShaderError::Kind::Compile;
        e.name = s.name;
        e.log = shaderLog(shader);
        e.line = parseShaderErrorLine(e.log);
        error = std::move(e);
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

}  // namespace

ShaderResult ShaderProgram::create(const std::vector<ShaderSource>& sources) {
    ShaderResult result;
    const ShaderSource* vertex = nullptr;
    const ShaderSource* fragment = nullptr;
    int vertexCount = 0;
    int fragmentCount = 0;
    for (const ShaderSource& s : sources) {
        if (s.stage == ShaderStage::Vertex) {
            vertex = &s;
            ++vertexCount;
        } else {
            fragment = &s;
            ++fragmentCount;
        }
    }
    if (vertexCount != 1 || fragmentCount != 1) {
        ShaderError e;
        e.kind = ShaderError::Kind::InvalidInput;
        e.name = "<program>";
        e.log = "需要恰好一个顶点着色器与一个片元着色器（实际 " + std::to_string(vertexCount) + " 与 " +
                std::to_string(fragmentCount) + " 个）";
        result.error = std::move(e);
        return result;
    }

    const GLuint vs = compile(*vertex, result.error);
    if (vs == 0) return result;
    const GLuint fs = compile(*fragment, result.error);
    if (fs == 0) {
        glDeleteShader(vs);
        return result;
    }

    const GLuint program = glCreateProgram();
    glAttachShader(program, vs);
    glAttachShader(program, fs);
    glLinkProgram(program);
    glDetachShader(program, vs);
    glDetachShader(program, fs);
    glDeleteShader(vs);
    glDeleteShader(fs);
    GLint ok = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);
    if (ok != GL_TRUE) {
        ShaderError e;
        e.kind = ShaderError::Kind::Link;
        e.name = vertex->name + " + " + fragment->name;
        e.log = programLog(program);
        e.line = parseShaderErrorLine(e.log);
        result.error = std::move(e);
        glDeleteProgram(program);
        return result;
    }
    result.program = ShaderProgram{program};
    return result;
}

ShaderProgram::~ShaderProgram() {
    if (name_ != 0) glDeleteProgram(name_);
}

ShaderProgram::ShaderProgram(ShaderProgram&& other) noexcept : name_(std::exchange(other.name_, 0u)) {}

ShaderProgram& ShaderProgram::operator=(ShaderProgram&& other) noexcept {
    if (this != &other) {
        if (name_ != 0) glDeleteProgram(name_);
        name_ = std::exchange(other.name_, 0u);
    }
    return *this;
}

void ShaderProgram::use() const { glUseProgram(name_); }

int ShaderProgram::uniformLocation(const std::string& uniform) const {
    return glGetUniformLocation(name_, uniform.c_str());
}

void ShaderProgram::setUniform(int location, float x, float y, float z, float w) {
    if (location >= 0) glProgramUniform4f(name_, location, x, y, z, w);
}

void ShaderProgram::setUniformMatrix4(int location, const float* columnMajor16) {
    if (location >= 0) glProgramUniformMatrix4fv(name_, location, 1, GL_FALSE, columnMajor16);
}

void ShaderProgram::setUniformInt(int location, int value) {
    if (location >= 0) glProgramUniform1i(name_, location, value);
}

void ShaderProgram::setUniformFloat(int location, float value) {
    if (location >= 0) glProgramUniform1f(name_, location, value);
}

void ShaderProgram::setUniformVec2(int location, float x, float y) {
    if (location >= 0) glProgramUniform2f(name_, location, x, y);
}

void ShaderProgram::setUniformVec3(int location, float x, float y, float z) {
    if (location >= 0) glProgramUniform3f(name_, location, x, y, z);
}

// ---- Framebuffer ----

FramebufferResult Framebuffer::create(int width, int height, bool withDepth) {
    FramebufferDesc desc;
    desc.width = width;
    desc.height = height;
    desc.depth = withDepth ? DepthFormat::Renderbuffer : DepthFormat::None;
    return create(desc);
}

FramebufferResult Framebuffer::create(const FramebufferDesc& desc) {
    FramebufferResult result;
    const int width = desc.width;
    const int height = desc.height;
    if (width <= 0 || height <= 0) {
        result.error = "帧缓冲尺寸无效：" + std::to_string(width) + "x" + std::to_string(height);
        return result;
    }
    if (desc.samples < 1) {
        result.error = "采样数无效：" + std::to_string(desc.samples);
        return result;
    }
    if (desc.color == ColorFormat::None && desc.depth == DepthFormat::None) {
        result.error = "帧缓冲既没有颜色也没有深度";
        return result;
    }
    if (desc.samples > 1 && desc.depth == DepthFormat::ShadowTexture) {
        result.error = "阴影深度纹理不支持多重采样";
        return result;
    }
    Framebuffer f;
    f.width_ = width;
    f.height_ = height;
    f.samples_ = desc.samples;
    const bool multisample = desc.samples > 1;
    glCreateFramebuffers(1, &f.name_);
    if (desc.color != ColorFormat::None) {
        const GLenum format = desc.color == ColorFormat::Rgba16F ? GL_RGBA16F : desc.color == ColorFormat::R32F ? GL_R32F : GL_RGBA8;
        if (multisample) {
            glCreateRenderbuffers(1, &f.color_);
            glNamedRenderbufferStorageMultisample(f.color_, desc.samples, format, width, height);
            glNamedFramebufferRenderbuffer(f.name_, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, f.color_);
        } else {
            glCreateTextures(GL_TEXTURE_2D, 1, &f.color_);
            glTextureStorage2D(f.color_, 1, format, width, height);
            glTextureParameteri(f.color_, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTextureParameteri(f.color_, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTextureParameteri(f.color_, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTextureParameteri(f.color_, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glNamedFramebufferTexture(f.name_, GL_COLOR_ATTACHMENT0, f.color_, 0);
            f.colorIsTexture_ = true;
        }
    } else {
        glNamedFramebufferDrawBuffer(f.name_, GL_NONE);
        glNamedFramebufferReadBuffer(f.name_, GL_NONE);
    }
    if (desc.depth == DepthFormat::Renderbuffer) {
        glCreateRenderbuffers(1, &f.depth_);
        if (multisample) {
            glNamedRenderbufferStorageMultisample(f.depth_, desc.samples, GL_DEPTH24_STENCIL8, width, height);
        } else {
            glNamedRenderbufferStorage(f.depth_, GL_DEPTH24_STENCIL8, width, height);
        }
        glNamedFramebufferRenderbuffer(f.name_, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, f.depth_);
    } else if (desc.depth == DepthFormat::ShadowTexture) {
        glCreateTextures(GL_TEXTURE_2D, 1, &f.depth_);
        glTextureStorage2D(f.depth_, 1, GL_DEPTH_COMPONENT32F, width, height);
        glTextureParameteri(f.depth_, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTextureParameteri(f.depth_, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTextureParameteri(f.depth_, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTextureParameteri(f.depth_, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
        const float border[4] = {1.0f, 1.0f, 1.0f, 1.0f};  // 阴影图范围外：深度最远，即不在阴影中
        glTextureParameterfv(f.depth_, GL_TEXTURE_BORDER_COLOR, border);
        glTextureParameteri(f.depth_, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
        glTextureParameteri(f.depth_, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
        glNamedFramebufferTexture(f.name_, GL_DEPTH_ATTACHMENT, f.depth_, 0);
        f.depthIsTexture_ = true;
    }
    const GLenum status = glCheckNamedFramebufferStatus(f.name_, GL_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        char code[16];
        std::snprintf(code, sizeof(code), "0x%04X", static_cast<unsigned>(status));
        result.error = std::string{"帧缓冲不完整（状态 "} + code + "）";
        return result;  // f 析构时释放已创建的对象
    }
    result.framebuffer = std::move(f);
    return result;
}

void Framebuffer::release() noexcept {
    if (name_ != 0) glDeleteFramebuffers(1, &name_);
    if (color_ != 0) {
        if (colorIsTexture_) {
            glDeleteTextures(1, &color_);
        } else {
            glDeleteRenderbuffers(1, &color_);
        }
    }
    if (depth_ != 0) {
        if (depthIsTexture_) {
            glDeleteTextures(1, &depth_);
        } else {
            glDeleteRenderbuffers(1, &depth_);
        }
    }
    name_ = color_ = depth_ = 0;
    colorIsTexture_ = depthIsTexture_ = false;
}

Framebuffer::~Framebuffer() { release(); }

Framebuffer::Framebuffer(Framebuffer&& other) noexcept
    : name_(std::exchange(other.name_, 0u)), color_(std::exchange(other.color_, 0u)),
      depth_(std::exchange(other.depth_, 0u)), colorIsTexture_(std::exchange(other.colorIsTexture_, false)),
      depthIsTexture_(std::exchange(other.depthIsTexture_, false)), width_(std::exchange(other.width_, 0)),
      height_(std::exchange(other.height_, 0)), samples_(std::exchange(other.samples_, 1)) {}

Framebuffer& Framebuffer::operator=(Framebuffer&& other) noexcept {
    if (this != &other) {
        release();
        name_ = std::exchange(other.name_, 0u);
        color_ = std::exchange(other.color_, 0u);
        depth_ = std::exchange(other.depth_, 0u);
        colorIsTexture_ = std::exchange(other.colorIsTexture_, false);
        depthIsTexture_ = std::exchange(other.depthIsTexture_, false);
        width_ = std::exchange(other.width_, 0);
        height_ = std::exchange(other.height_, 0);
        samples_ = std::exchange(other.samples_, 1);
    }
    return *this;
}

void Framebuffer::resolveTo(const Framebuffer& target) const {
    if (name_ == 0 || target.name_ == 0) return;
    glBlitNamedFramebuffer(name_, target.name_, 0, 0, width_, height_, 0, 0, target.width_, target.height_,
                           GL_COLOR_BUFFER_BIT, GL_NEAREST);
}

void Framebuffer::bind() const {
    glBindFramebuffer(GL_FRAMEBUFFER, name_);
    glViewport(0, 0, width_, height_);
}

void Framebuffer::bindDefault() { glBindFramebuffer(GL_FRAMEBUFFER, 0); }

std::vector<std::uint8_t> Framebuffer::readPixels(int x, int y, int width, int height) const {
    std::vector<std::uint8_t> pixels;
    if (name_ == 0 || width <= 0 || height <= 0) return pixels;
    pixels.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4u);
    GLint previous = 0;
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &previous);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, name_);
    glNamedFramebufferReadBuffer(name_, GL_COLOR_ATTACHMENT0);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadnPixels(x, y, width, height, GL_RGBA, GL_UNSIGNED_BYTE, static_cast<GLsizei>(pixels.size()), pixels.data());
    glBindFramebuffer(GL_READ_FRAMEBUFFER, static_cast<GLuint>(previous));
    return pixels;
}

std::vector<float> Framebuffer::readFloats(int x, int y, int width, int height) const {
    std::vector<float> values;
    if (name_ == 0 || width <= 0 || height <= 0) return values;
    values.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));
    GLint previous = 0;
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &previous);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, name_);
    glNamedFramebufferReadBuffer(name_, GL_COLOR_ATTACHMENT0);
    glPixelStorei(GL_PACK_ALIGNMENT, 4);
    glReadnPixels(x, y, width, height, GL_RED, GL_FLOAT, static_cast<GLsizei>(values.size() * sizeof(float)), values.data());
    glBindFramebuffer(GL_READ_FRAMEBUFFER, static_cast<GLuint>(previous));
    return values;
}

}  // namespace terra::render
