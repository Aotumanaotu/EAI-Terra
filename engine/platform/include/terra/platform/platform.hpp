#pragma once

// 平台层接口：窗口与 OpenGL 上下文（M1-03，D-017、D-021）。
//
// 只使用 Terra 类型；具体实现（首个为 GLFW）在 backends/ 下，其窗口库类型不出现在本接口中。
// 平台在 pollEvents() 中把键盘、鼠标等事件翻译后写入创建时绑定的 InputSink（D-018）。
// 线程：所有调用都必须在创建平台的线程（主线程）上进行。

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace terra::platform {

struct Extent {
    int width = 0;
    int height = 0;
};

struct WindowDescription {
    std::string title = "EAI Terra";
    int width = 1280;  // 窗口尺寸（屏幕坐标）；帧缓冲尺寸可能不同（高 DPI）
    int height = 720;
    bool visible = true;
    bool vsync = true;
    bool glDebugContext = false;  // 请求调试上下文，配合 KHR_debug（4.5 core 内置）
    // 请求的 OpenGL core 版本。Terra 要求至少 4.5（D-021），低于 4.5 的请求被拒绝。
    int glMajor = 4;
    int glMinor = 5;
};

// 显示模式（M1-12，D-025）。
//   Windowed              普通窗口（默认）
//   BorderlessFullscreen  无边框全屏：占满显示器，沿用桌面分辨率，不改变显示模式；切换快（F11）
//   ExclusiveFullscreen   独占全屏：可指定分辨率与刷新率（必须是显示器支持的模式）；失去焦点时自动最小化
enum class DisplayMode { Windowed, BorderlessFullscreen, ExclusiveFullscreen };

inline const char* displayModeName(DisplayMode mode) noexcept {
    switch (mode) {
        case DisplayMode::Windowed: return "窗口";
        case DisplayMode::BorderlessFullscreen: return "无边框全屏";
        case DisplayMode::ExclusiveFullscreen: return "独占全屏";
    }
    return "未知";
}

struct VideoMode {
    int width = 0;
    int height = 0;
    int refreshRate = 0;  // Hz

    friend bool operator==(const VideoMode& a, const VideoMode& b) noexcept {
        return a.width == b.width && a.height == b.height && a.refreshRate == b.refreshRate;
    }
};

struct MonitorInfo {
    int index = 0;           // 在 Platform::monitors() 中的下标，用于 setDisplayMode
    std::string name;
    bool primary = false;
    int x = 0;               // 在虚拟桌面中的位置（屏幕坐标）
    int y = 0;
    float contentScale = 1.0f;
    VideoMode current;       // 当前桌面模式
    std::vector<VideoMode> modes;  // 支持的全部模式（按显示器报告的顺序）
};

enum class PlatformErrorKind {
    NoDisplay,           // 没有可用的显示服务（DISPLAY / WAYLAND_DISPLAY 均未设置，或无法连接）
    ContextUnavailable,  // 无法创建所请求版本的 OpenGL core 上下文
    WindowCreation,      // 窗口创建失败或参数无效
    AlreadyInitialized,  // 同一进程已有一个平台实例
    InvalidRequest,      // 参数无效：不存在的显示器、显示器不支持的视频模式等（M1-12）
    Internal,
};

struct PlatformError {
    PlatformErrorKind kind;
    std::string message;
};

// 交给 OpenGL 加载器（glad）的函数类型；与窗口库无关。
using GlProc = void (*)();
using GlProcLoader = GlProc (*)(const char* name);

class Window {
public:
    virtual ~Window() = default;
    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;

    virtual bool closeRequested() const = 0;
    virtual void requestClose() = 0;
    virtual void setTitle(const std::string& title) = 0;  // UTF-8
    virtual Extent windowSize() const = 0;       // 屏幕坐标
    virtual Extent framebufferSize() const = 0;  // 像素，用于 glViewport
    virtual float contentScale() const = 0;      // 显示缩放（高 DPI 时 > 1）
    virtual void makeContextCurrent() = 0;
    virtual void swapBuffers() = 0;
    virtual GlProcLoader glProcLoader() const = 0;
    // ---- 显示模式（M1-12，D-025）----
    virtual DisplayMode displayMode() const = 0;
    // 切换显示模式。monitorIndex < 0 表示窗口当前所在的显示器；videoMode 只对独占全屏有效，
    // 为空时使用显示器当前模式，否则必须是该显示器支持的模式。切回窗口时恢复进入全屏前的位置与大小。
    // 尺寸变化可能在下一次 pollEvents 之后才体现在 windowSize/framebufferSize 中。
    virtual std::optional<PlatformError> setDisplayMode(DisplayMode mode, int monitorIndex = -1,
                                                        std::optional<VideoMode> videoMode = std::nullopt) = 0;
    // F11 的语义：窗口 ↔ 无边框全屏；从独占全屏回到窗口。
    virtual std::optional<PlatformError> toggleBorderlessFullscreen() = 0;
    // 垂直同步；作用于本窗口的上下文（会把它设为当前上下文）。
    virtual void setVsync(bool enabled) = 0;

    // 实际创建的 OpenGL 上下文版本（≥ 请求版本）。
    virtual int glMajor() const = 0;
    virtual int glMinor() const = 0;

    // M4-08a：取出自上次调用以来拖进窗口的文件（绝对路径，UTF-8）；在 pollEvents 之后调用
    virtual std::vector<std::string> takeDroppedPaths() { return {}; }

protected:
    Window() = default;
};

struct WindowResult {
    std::unique_ptr<Window> window;
    std::optional<PlatformError> error;
};

class Platform {
public:
    virtual ~Platform() = default;
    Platform(const Platform&) = delete;
    Platform& operator=(const Platform&) = delete;

    // 创建窗口及其 OpenGL 上下文；成功后该上下文为当前上下文。
    // 窗口可以比平台对象活得更久（实现保证底层库在最后一个窗口销毁后才释放）。
    virtual WindowResult createWindow(const WindowDescription& description) = 0;

    // 处理窗口系统事件，并把输入事件写入绑定的 InputSink。每帧调用一次，在 InputSystem::update() 之前。
    virtual void pollEvents() = 0;

    // 单调时钟（秒），也是写入 InputSink 的事件时间戳。
    virtual double time() const = 0;

    // 已连接的显示器；第一个为主显示器（M1-12）。
    virtual std::vector<MonitorInfo> monitors() const = 0;

protected:
    Platform() = default;
};

struct PlatformResult {
    std::unique_ptr<Platform> platform;
    std::optional<PlatformError> error;
};

}  // namespace terra::platform
