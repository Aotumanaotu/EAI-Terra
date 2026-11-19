#pragma once

// Terra Render：OpenGL 上下文初始化、调试输出与基本绘制状态（M1-06，D-004、D-021）。
//
// 公共接口不包含 OpenGL 头文件，也不暴露窗口库类型；GL 对象以 std::uint32_t 名称表示。
// 前提：调用方已经在当前线程上让一个 OpenGL 4.5 core 上下文成为当前上下文（例如 terra_platform 创建的窗口）。
// 线程：所有调用必须在拥有该上下文的线程上进行。

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace terra::render {

// 与 terra::platform::GlProcLoader 相同的函数类型；Render 不依赖平台层，只接受这个函数指针。
using GlProc = void (*)();
using GlLoader = GlProc (*)(const char* name);

struct Color {
    float r = 0.0f;
    float g = 0.0f;
    float b = 0.0f;
    float a = 1.0f;
};

struct GlInfo {
    std::string vendor;
    std::string renderer;
    std::string version;
    std::string glslVersion;
    int major = 0;
    int minor = 0;
    bool debugContext = false;  // 上下文带调试标志时，驱动消息经 KHR_debug 同步回调收集
};

enum class DebugSeverity { Notification, Low, Medium, High };

struct DebugMessage {
    DebugSeverity severity = DebugSeverity::Notification;
    bool isError = false;  // 驱动把它归为 GL 错误（GL_DEBUG_TYPE_ERROR）
    std::uint32_t id = 0;
    std::string text;
};

struct ContextResult;

class RenderContext {
public:
    // 在当前上下文上初始化：加载 OpenGL 函数、确认版本 ≥ 4.5 core；
    // 若为调试上下文，开启同步调试输出并收集驱动消息（通知级消息丢弃）。
    static ContextResult create(GlLoader loader);

    ~RenderContext();
    RenderContext(const RenderContext&) = delete;
    RenderContext& operator=(const RenderContext&) = delete;

    const GlInfo& info() const noexcept;

    // 取出并清空目前收集到的调试消息。
    std::vector<DebugMessage> takeDebugMessages();
    // 自创建以来收到的 GL 错误类消息总数（不随 takeDebugMessages 清零）。
    std::size_t errorMessageCount() const noexcept;

    // ---- 基本绘制状态 ----
    void setViewport(int x, int y, int width, int height);
    void setDepthTest(bool enabled);  // 开启时比较函数为 LESS，并写入深度
    void clear(Color color, bool clearDepth = true);  // 清除当前绑定的帧缓冲
    void drawTriangles(int firstVertex, int vertexCount);  // 使用当前绑定的程序与顶点数组
    void drawLines(int firstVertex, int vertexCount);      // 每两个顶点一条线段（M1-07 起，网格与坐标轴）

private:
    struct Impl;
    explicit RenderContext(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

struct ContextResult {
    std::unique_ptr<RenderContext> context;
    std::optional<std::string> error;
};

}  // namespace terra::render
