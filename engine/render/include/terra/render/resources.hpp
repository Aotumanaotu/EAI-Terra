#pragma once

// Terra Render：GL 资源的 RAII 封装（M1-06）。全部使用 OpenGL 4.5 的直接状态访问（DSA）创建与修改。
//
// 每个类型独占一个 GL 对象：只能移动，不能复制；析构时删除对象；被移走的对象名称为 0、析构时不做任何事。
// 必须在 RenderContext 创建之后、其上下文仍然有效时使用与销毁。

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace terra::render {

class Buffer {
public:
    Buffer() = default;
    // 创建并分配 bytes 字节；data 可为空（未初始化内容）。dynamic 为真时允许之后 update。
    static Buffer create(std::size_t bytes, const void* data, bool dynamic = false);

    ~Buffer();
    Buffer(Buffer&& other) noexcept;
    Buffer& operator=(Buffer&& other) noexcept;
    Buffer(const Buffer&) = delete;
    Buffer& operator=(const Buffer&) = delete;

    // 仅 dynamic 缓冲；越界时不写入并返回 false。
    bool update(std::size_t offset, std::size_t bytes, const void* data);

    std::uint32_t name() const noexcept { return name_; }
    std::size_t size() const noexcept { return size_; }
    bool dynamic() const noexcept { return dynamic_; }

private:
    std::uint32_t name_ = 0;
    std::size_t size_ = 0;
    bool dynamic_ = false;
};

class VertexArray {
public:
    VertexArray() = default;
    static VertexArray create();

    ~VertexArray();
    VertexArray(VertexArray&& other) noexcept;
    VertexArray& operator=(VertexArray&& other) noexcept;
    VertexArray(const VertexArray&) = delete;
    VertexArray& operator=(const VertexArray&) = delete;

    // 把缓冲挂到绑定点 binding，stride 为每个顶点的字节数。
    void setVertexBuffer(std::uint32_t binding, const Buffer& buffer, std::size_t offset, std::size_t stride);
    // 顶点属性 location：components 个 float，从顶点起始处 offset 字节开始，读自绑定点 binding。
    void setFloatAttribute(std::uint32_t location, std::uint32_t binding, int components, std::size_t offset);
    void bind() const;

    std::uint32_t name() const noexcept { return name_; }

private:
    std::uint32_t name_ = 0;
};

enum class ShaderStage { Vertex, Fragment };

struct ShaderSource {
    ShaderStage stage = ShaderStage::Vertex;
    std::string name;    // 用于诊断，如 "basic.vert"
    std::string source;  // GLSL 源码，首行应为 "#version 450 core"
};

struct ShaderError {
    enum class Kind { Compile, Link, InvalidInput };
    Kind kind = Kind::Compile;
    std::string name;         // 出错的着色器名称；链接错误时为参与链接的全部名称
    std::optional<int> line;  // 从驱动日志中解析出的首个错误行号（1 起）；无法解析时为空
    std::string log;          // 驱动原始日志

    // “basic.frag:4: 编译失败：<日志首行>” 形式的单行诊断。
    std::string summary() const;
};

class ShaderProgram;

struct ShaderResult;

class ShaderProgram {
public:
    ShaderProgram() = default;
    // 编译全部阶段并链接；要求恰好一个顶点着色器与一个片元着色器。
    static ShaderResult create(const std::vector<ShaderSource>& sources);

    ~ShaderProgram();
    ShaderProgram(ShaderProgram&& other) noexcept;
    ShaderProgram& operator=(ShaderProgram&& other) noexcept;
    ShaderProgram(const ShaderProgram&) = delete;
    ShaderProgram& operator=(const ShaderProgram&) = delete;

    void use() const;
    // uniform 不存在（或被驱动优化掉）时返回 -1；对 -1 设置值无效果。
    int uniformLocation(const std::string& uniform) const;
    void setUniform(int location, float x, float y, float z, float w);
    void setUniformMatrix4(int location, const float* columnMajor16);

    std::uint32_t name() const noexcept { return name_; }

private:
    explicit ShaderProgram(std::uint32_t name) : name_(name) {}
    std::uint32_t name_ = 0;
};

struct ShaderResult {
    std::optional<ShaderProgram> program;
    std::optional<ShaderError> error;
};

class Framebuffer;

struct FramebufferResult;

// 离屏帧缓冲：RGBA8 颜色纹理 + 可选 DEPTH24_STENCIL8 深度。
class Framebuffer {
public:
    Framebuffer() = default;
    static FramebufferResult create(int width, int height, bool withDepth);

    ~Framebuffer();
    Framebuffer(Framebuffer&& other) noexcept;
    Framebuffer& operator=(Framebuffer&& other) noexcept;
    Framebuffer(const Framebuffer&) = delete;
    Framebuffer& operator=(const Framebuffer&) = delete;

    // 绑定为绘制目标，并把视口设为整个帧缓冲。
    void bind() const;
    // 恢复默认帧缓冲（窗口）；视口由调用方设置。
    static void bindDefault();

    // 读回 RGBA8 像素，行序自下而上（OpenGL 约定），每行 width*4 字节。
    std::vector<std::uint8_t> readPixels(int x, int y, int width, int height) const;

    int width() const noexcept { return width_; }
    int height() const noexcept { return height_; }
    std::uint32_t name() const noexcept { return name_; }
    std::uint32_t colorTexture() const noexcept { return color_; }

private:
    void release() noexcept;
    std::uint32_t name_ = 0;
    std::uint32_t color_ = 0;
    std::uint32_t depth_ = 0;
    int width_ = 0;
    int height_ = 0;
};

struct FramebufferResult {
    std::optional<Framebuffer> framebuffer;
    std::optional<std::string> error;
};

// 从驱动日志中解析首个错误行号。支持 Mesa（"0:4(12): error: …"）与 NVIDIA（"0(4) : error C0000: …"）格式。
std::optional<int> parseShaderErrorLine(const std::string& log);

}  // namespace terra::render
