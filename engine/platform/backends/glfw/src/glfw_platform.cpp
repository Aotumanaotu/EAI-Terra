// GLFW 平台实现（M1-03）与 GLFW 键鼠适配器（M1-04）。GLFW 只在本文件与翻译表中出现（D-017）。

#include "terra/platform/glfw/glfw_platform.hpp"

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <utility>

#include <GLFW/glfw3.h>

namespace terra::platform::glfw {

namespace {

namespace in = terra::input;

// GLFW 的错误回调是进程级的；记录最近一次错误，供创建失败时生成诊断。
struct LastError {
    int code = 0;
    std::string description;
};

LastError& lastError() {
    static LastError error;
    return error;
}

void onGlfwError(int code, const char* description) {
    lastError() = LastError{code, description != nullptr ? description : ""};
}

void clearError() { lastError() = LastError{}; }

std::string describeLastError() {
    const LastError& e = lastError();
    if (e.code == 0) return "GLFW 未报告错误";
    char code[16];
    std::snprintf(code, sizeof(code), "0x%05X", static_cast<unsigned>(e.code));
    return std::string{"GLFW 错误 "} + code + "：" + e.description;
}

std::atomic<bool> g_instanceExists{false};

// GLFW 的进程级生命周期：平台与所有窗口共同持有，最后一个持有者销毁时才调用 glfwTerminate。
struct Runtime {
    ~Runtime() {
        glfwTerminate();
        g_instanceExists.store(false);
    }
};

// 窗口回调与输入系统之间的桥：只通过 InputSink 写入（D-018）。
struct InputBridge {
    in::InputSink* sink = nullptr;
    in::DeviceId keyboard;
    in::DeviceId mouse;
    bool haveCursor = false;
    double lastX = 0.0;
    double lastY = 0.0;

    void releaseAll() {
        sink->resetDevice(keyboard);
        sink->resetDevice(mouse);
        haveCursor = false;
    }
};

class GlfwWindow final : public Window {
public:
    GlfwWindow(std::shared_ptr<Runtime> runtime, std::shared_ptr<InputBridge> bridge, GLFWwindow* handle, int major,
               int minor)
        : runtime_(std::move(runtime)), bridge_(std::move(bridge)), handle_(handle), major_(major), minor_(minor) {
        glfwSetWindowUserPointer(handle_, this);
        glfwSetKeyCallback(handle_, &GlfwWindow::onKey);
        glfwSetMouseButtonCallback(handle_, &GlfwWindow::onMouseButton);
        glfwSetCursorPosCallback(handle_, &GlfwWindow::onCursorPos);
        glfwSetCursorEnterCallback(handle_, &GlfwWindow::onCursorEnter);
        glfwSetScrollCallback(handle_, &GlfwWindow::onScroll);
        glfwSetWindowFocusCallback(handle_, &GlfwWindow::onFocus);
    }

    ~GlfwWindow() override {
        glfwDestroyWindow(handle_);
        // runtime_ 在成员析构时释放；若这是最后一个持有者，GLFW 随之终止。
    }

    bool closeRequested() const override { return glfwWindowShouldClose(handle_) == GLFW_TRUE; }
    void requestClose() override { glfwSetWindowShouldClose(handle_, GLFW_TRUE); }
    void setTitle(const std::string& title) override { glfwSetWindowTitle(handle_, title.c_str()); }

    Extent windowSize() const override {
        Extent e;
        glfwGetWindowSize(handle_, &e.width, &e.height);
        return e;
    }

    Extent framebufferSize() const override {
        Extent e;
        glfwGetFramebufferSize(handle_, &e.width, &e.height);
        return e;
    }

    float contentScale() const override {
        float x = 1.0f;
        float y = 1.0f;
        glfwGetWindowContentScale(handle_, &x, &y);
        return x;
    }

    void makeContextCurrent() override { glfwMakeContextCurrent(handle_); }
    void swapBuffers() override { glfwSwapBuffers(handle_); }
    GlProcLoader glProcLoader() const override { return &glfwGetProcAddress; }
    int glMajor() const override { return major_; }
    int glMinor() const override { return minor_; }

private:
    static InputBridge* bridgeOf(GLFWwindow* handle) {
        auto* self = static_cast<GlfwWindow*>(glfwGetWindowUserPointer(handle));
        return self != nullptr ? self->bridge_.get() : nullptr;
    }

    static void onKey(GLFWwindow* handle, int glfwKey, int /*scancode*/, int action, int /*mods*/) {
        InputBridge* b = bridgeOf(handle);
        if (b == nullptr || action == GLFW_REPEAT) return;  // 按住期间的重复事件不改变控件值
        const in::Key k = translateKey(glfwKey);
        if (k == in::Key::Unknown) return;
        b->sink->setControl(b->keyboard, in::key(k), action == GLFW_PRESS ? 1.0f : 0.0f, glfwGetTime());
    }

    static void onMouseButton(GLFWwindow* handle, int button, int action, int /*mods*/) {
        InputBridge* b = bridgeOf(handle);
        if (b == nullptr) return;
        const auto control = translateMouseButton(button);
        if (!control) return;
        b->sink->setControl(b->mouse, in::mouse(*control), action == GLFW_PRESS ? 1.0f : 0.0f, glfwGetTime());
    }

    static void onCursorPos(GLFWwindow* handle, double x, double y) {
        InputBridge* b = bridgeOf(handle);
        if (b == nullptr) return;
        const double t = glfwGetTime();
        b->sink->setControl(b->mouse, in::mouse(in::MouseControl::PositionX), static_cast<float>(x), t);
        b->sink->setControl(b->mouse, in::mouse(in::MouseControl::PositionY), static_cast<float>(y), t);
        // 第一次（或指针重新进入窗口后）只记录位置，不产生增量，避免视角跳变。
        if (b->haveCursor) {
            b->sink->addDelta(b->mouse, in::mouse(in::MouseControl::DeltaX), static_cast<float>(x - b->lastX), t);
            b->sink->addDelta(b->mouse, in::mouse(in::MouseControl::DeltaY), static_cast<float>(y - b->lastY), t);
        }
        b->haveCursor = true;
        b->lastX = x;
        b->lastY = y;
    }

    static void onCursorEnter(GLFWwindow* handle, int /*entered*/) {
        if (InputBridge* b = bridgeOf(handle)) b->haveCursor = false;
    }

    static void onScroll(GLFWwindow* handle, double dx, double dy) {
        InputBridge* b = bridgeOf(handle);
        if (b == nullptr) return;
        const double t = glfwGetTime();
        b->sink->addDelta(b->mouse, in::mouse(in::MouseControl::ScrollX), static_cast<float>(dx), t);
        b->sink->addDelta(b->mouse, in::mouse(in::MouseControl::ScrollY), static_cast<float>(dy), t);
    }

    static void onFocus(GLFWwindow* handle, int focused) {
        // 失去焦点时释放所有按键，避免切换窗口后按键“卡住”（与设备断开同一套释放语义）。
        if (focused == GLFW_FALSE) {
            if (InputBridge* b = bridgeOf(handle)) b->releaseAll();
        }
    }

    std::shared_ptr<Runtime> runtime_;
    std::shared_ptr<InputBridge> bridge_;
    GLFWwindow* handle_;
    int major_;
    int minor_;
};

class GlfwPlatform final : public Platform {
public:
    GlfwPlatform(std::shared_ptr<Runtime> runtime, std::shared_ptr<InputBridge> bridge)
        : runtime_(std::move(runtime)), bridge_(std::move(bridge)) {}

    ~GlfwPlatform() override {
        bridge_->sink->disconnectDevice(bridge_->keyboard);
        bridge_->sink->disconnectDevice(bridge_->mouse);
    }

    WindowResult createWindow(const WindowDescription& d) override {
        WindowResult result;
        if (d.width <= 0 || d.height <= 0) {
            result.error = PlatformError{PlatformErrorKind::WindowCreation,
                                         "窗口尺寸无效：" + std::to_string(d.width) + "x" + std::to_string(d.height)};
            return result;
        }
        if (d.glMajor < 4 || (d.glMajor == 4 && d.glMinor < 5)) {
            result.error = PlatformError{PlatformErrorKind::WindowCreation,
                                         "请求的 OpenGL " + std::to_string(d.glMajor) + "." + std::to_string(d.glMinor) +
                                             " 低于 Terra 要求的 4.5（D-021）"};
            return result;
        }

        glfwDefaultWindowHints();
        glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, d.glMajor);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, d.glMinor);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
        glfwWindowHint(GLFW_CONTEXT_DEBUG, d.glDebugContext ? GLFW_TRUE : GLFW_FALSE);
        glfwWindowHint(GLFW_VISIBLE, d.visible ? GLFW_TRUE : GLFW_FALSE);

        clearError();
        GLFWwindow* handle = glfwCreateWindow(d.width, d.height, d.title.c_str(), nullptr, nullptr);
        if (handle == nullptr) {
            const int code = lastError().code;
            const bool context =
                code == GLFW_VERSION_UNAVAILABLE || code == GLFW_API_UNAVAILABLE || code == GLFW_FORMAT_UNAVAILABLE;
            result.error = PlatformError{
                context ? PlatformErrorKind::ContextUnavailable : PlatformErrorKind::WindowCreation,
                "无法创建 OpenGL " + std::to_string(d.glMajor) + "." + std::to_string(d.glMinor) +
                    " core 窗口（" + describeLastError() + "）"};
            return result;
        }

        glfwMakeContextCurrent(handle);
        glfwSwapInterval(d.vsync ? 1 : 0);
        const int major = glfwGetWindowAttrib(handle, GLFW_CONTEXT_VERSION_MAJOR);
        const int minor = glfwGetWindowAttrib(handle, GLFW_CONTEXT_VERSION_MINOR);
        if (major < d.glMajor || (major == d.glMajor && minor < d.glMinor)) {
            glfwDestroyWindow(handle);
            result.error = PlatformError{PlatformErrorKind::ContextUnavailable,
                                         "请求 OpenGL " + std::to_string(d.glMajor) + "." + std::to_string(d.glMinor) +
                                             "，实际只得到 " + std::to_string(major) + "." + std::to_string(minor)};
            return result;
        }
        result.window = std::make_unique<GlfwWindow>(runtime_, bridge_, handle, major, minor);
        return result;
    }

    void pollEvents() override { glfwPollEvents(); }
    double time() const override { return glfwGetTime(); }

private:
    std::shared_ptr<Runtime> runtime_;
    std::shared_ptr<InputBridge> bridge_;
};

bool envSet(const char* name) {
    const char* value = std::getenv(name);
    return value != nullptr && value[0] != '\0';
}

}  // namespace

PlatformResult createPlatform(terra::input::InputSink& input) {
    PlatformResult result;
    if (g_instanceExists.exchange(true)) {
        result.error = PlatformError{PlatformErrorKind::AlreadyInitialized, "同一进程只能有一个 GLFW 平台实例"};
        return result;
    }
    if (!envSet("DISPLAY") && !envSet("WAYLAND_DISPLAY")) {
        g_instanceExists.store(false);
        result.error = PlatformError{PlatformErrorKind::NoDisplay, "没有显示服务（DISPLAY 与 WAYLAND_DISPLAY 均未设置）"};
        return result;
    }

    glfwSetErrorCallback(&onGlfwError);
    clearError();
    if (glfwInit() != GLFW_TRUE) {
        const bool noDisplay = lastError().code == GLFW_PLATFORM_UNAVAILABLE;
        g_instanceExists.store(false);
        result.error = PlatformError{noDisplay ? PlatformErrorKind::NoDisplay : PlatformErrorKind::Internal,
                                     "GLFW 初始化失败（" + describeLastError() + "）"};
        return result;
    }

    auto runtime = std::make_shared<Runtime>();
    auto bridge = std::make_shared<InputBridge>();
    bridge->sink = &input;
    bridge->keyboard = input.connectDevice({in::DeviceKind::Keyboard, "GLFW keyboard", "glfw", ""});
    bridge->mouse = input.connectDevice({in::DeviceKind::Mouse, "GLFW mouse", "glfw", ""});
    result.platform = std::make_unique<GlfwPlatform>(std::move(runtime), std::move(bridge));
    return result;
}

}  // namespace terra::platform::glfw
