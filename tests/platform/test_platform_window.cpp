// 平台窗口测试（M1-03）：只通过 terra_platform 接口与 GLFW 实现入口访问，本文件不包含 GLFW 头文件。
//
// 模式（第一个参数）：
//   window      在 Xvfb + llvmpipe 中运行（D-022）：创建窗口与 4.5 core 上下文、glad 加载、尺寸、
//               请求 4.6 时返回 ContextUnavailable、重复创建平台被拒绝、低于 4.5 的请求被拒绝、
//               窗口比平台活得更久、平台连接与断开键鼠设备
//   no-display  清除 DISPLAY/WAYLAND_DISPLAY 后运行：创建平台返回 NoDisplay，不崩溃
//   display     在 Xvfb 中运行（M1-12，D-025）：显示器查询；窗口 ↔ 无边框全屏 ↔ 独占全屏切换后的尺寸；
//               F11 语义的切换；切回窗口恢复原尺寸；不存在的显示器与不支持的视频模式被拒绝

#include <cstdio>
#include <cstring>
#include <memory>
#include <string>

#include <glad/gl.h>

#include "terra/input/input_system.hpp"
#include "terra/platform/glfw/glfw_platform.hpp"
#include "terra/platform/platform.hpp"

namespace {

using terra::input::DeviceKind;
using terra::input::InputSystem;
using terra::platform::DisplayMode;
using terra::platform::PlatformErrorKind;
using terra::platform::WindowDescription;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

const char* kindName(PlatformErrorKind k) {
    switch (k) {
        case PlatformErrorKind::NoDisplay: return "NoDisplay";
        case PlatformErrorKind::ContextUnavailable: return "ContextUnavailable";
        case PlatformErrorKind::WindowCreation: return "WindowCreation";
        case PlatformErrorKind::AlreadyInitialized: return "AlreadyInitialized";
        case PlatformErrorKind::InvalidRequest: return "InvalidRequest";
        case PlatformErrorKind::Internal: return "Internal";
    }
    return "?";
}

int runNoDisplay() {
    InputSystem input;
    auto result = terra::platform::glfw::createPlatform(input);
    check(!result.platform && result.error && result.error->kind == PlatformErrorKind::NoDisplay,
          "无显示：返回 NoDisplay");
    if (result.error) std::printf("INFO %s：%s\n", kindName(result.error->kind), result.error->message.c_str());
    check(input.devices().empty(), "无显示：没有连接任何输入设备");
    return failures == 0 ? 0 : 1;
}

int runWindow() {
    InputSystem input;
    std::unique_ptr<terra::platform::Window> survivor;
    {
        auto created = terra::platform::glfw::createPlatform(input);
        if (!created.platform) {
            std::fprintf(stderr, "FAILED: 创建平台：%s\n", created.error ? created.error->message.c_str() : "");
            return 1;
        }
        auto& platform = *created.platform;

        // 平台连接 GLFW 键盘与鼠标两个设备
        const auto devices = input.devices();
        check(devices.size() == 2 && devices[0].description.kind == DeviceKind::Keyboard &&
                  devices[1].description.kind == DeviceKind::Mouse && devices[0].description.source == "glfw",
              "平台连接 GLFW 键盘与鼠标");

        // 同一进程第二个平台被拒绝
        auto second = terra::platform::glfw::createPlatform(input);
        check(!second.platform && second.error && second.error->kind == PlatformErrorKind::AlreadyInitialized,
              "重复创建平台返回 AlreadyInitialized");

        // 低于 4.5 的请求被拒绝（不尝试创建）
        WindowDescription old;
        old.visible = false;
        old.glMajor = 3;
        old.glMinor = 3;
        auto oldResult = platform.createWindow(old);
        check(!oldResult.window && oldResult.error && oldResult.error->kind == PlatformErrorKind::WindowCreation,
              "请求 OpenGL 3.3 被拒绝（D-021 要求 ≥ 4.5）");

        // llvmpipe 只支持到 4.5：请求 4.6 必须明确失败（确定性负向用例）
        WindowDescription tooNew;
        tooNew.visible = false;
        tooNew.glMinor = 6;
        auto tooNewResult = platform.createWindow(tooNew);
        check(!tooNewResult.window && tooNewResult.error &&
                  tooNewResult.error->kind == PlatformErrorKind::ContextUnavailable,
              "llvmpipe 上请求 OpenGL 4.6 返回 ContextUnavailable");
        if (tooNewResult.error) {
            std::printf("INFO 请求 4.6：%s：%s\n", kindName(tooNewResult.error->kind),
                        tooNewResult.error->message.c_str());
        }

        // 正常窗口：4.5 core
        WindowDescription desc;
        desc.title = "terra platform test";
        desc.width = 320;
        desc.height = 240;
        desc.visible = false;
        desc.glDebugContext = true;
        auto made = platform.createWindow(desc);
        if (!made.window) {
            std::fprintf(stderr, "FAILED: 创建 4.5 窗口：%s\n", made.error ? made.error->message.c_str() : "");
            return 1;
        }
        auto& window = *made.window;
        std::printf("INFO 上下文 %d.%d\n", window.glMajor(), window.glMinor());
        check(window.glMajor() == 4 && window.glMinor() >= 5, "上下文版本 ≥ 4.5");

        const int version = gladLoadGL(window.glProcLoader());
        check(GLAD_VERSION_MAJOR(version) == 4 && GLAD_VERSION_MINOR(version) >= 5, "经 glProcLoader 加载 glad");
        if (version != 0) {
            const auto* renderer = reinterpret_cast<const char*>(glGetString(GL_RENDERER));
            std::printf("INFO GL_RENDERER=%s\n", renderer != nullptr ? renderer : "<null>");
            check(renderer != nullptr && std::strstr(renderer, "llvmpipe") != nullptr, "运行在 llvmpipe 上（D-022）");
            GLint flags = 0;
            glGetIntegerv(GL_CONTEXT_FLAGS, &flags);
            check((flags & GL_CONTEXT_FLAG_DEBUG_BIT) != 0, "请求调试上下文后带 DEBUG 标志");
        }

        const auto size = window.windowSize();
        const auto fb = window.framebufferSize();
        std::printf("INFO 窗口 %dx%d，帧缓冲 %dx%d，缩放 %.2f\n", size.width, size.height, fb.width, fb.height,
                    static_cast<double>(window.contentScale()));
        check(size.width == 320 && size.height == 240, "窗口尺寸与请求一致");
        check(fb.width > 0 && fb.height > 0, "帧缓冲尺寸有效");

        check(!window.closeRequested(), "初始未请求关闭");
        window.requestClose();
        check(window.closeRequested(), "requestClose 后 closeRequested 为真");

        platform.pollEvents();
        const double t0 = platform.time();
        const double t1 = platform.time();
        check(t1 >= t0 && t0 >= 0.0, "单调时钟");
        window.swapBuffers();

        survivor = std::move(made.window);
    }  // 平台先销毁：键鼠设备断开，但窗口仍然有效（底层库在最后一个窗口销毁后才终止）

    check(input.devices().empty(), "平台销毁后键鼠设备已断开");
    check(survivor->framebufferSize().width > 0, "平台销毁后窗口仍可使用");
    survivor.reset();

    // 所有对象销毁后可以重新创建平台
    auto again = terra::platform::glfw::createPlatform(input);
    check(again.platform != nullptr, "全部销毁后可以重新创建平台");
    return failures == 0 ? 0 : 1;
}

// 全屏切换在 X11 下是异步的：轮询事件直到条件成立或超时。
template <typename Predicate>
bool waitFor(terra::platform::Platform& platform, Predicate condition, double timeoutSeconds = 2.0) {
    const double deadline = platform.time() + timeoutSeconds;
    while (platform.time() < deadline) {
        platform.pollEvents();
        if (condition()) return true;
    }
    platform.pollEvents();
    return condition();
}

std::string extentText(terra::platform::Extent e) { return std::to_string(e.width) + "x" + std::to_string(e.height); }

int runDisplay() {
    InputSystem input;
    auto created = terra::platform::glfw::createPlatform(input);
    if (!created.platform) {
        std::fprintf(stderr, "FAILED: 创建平台：%s\n", created.error ? created.error->message.c_str() : "");
        return 1;
    }
    auto& platform = *created.platform;

    const auto monitors = platform.monitors();
    check(!monitors.empty(), "至少一个显示器");
    if (monitors.empty()) return 1;
    for (const auto& m : monitors) {
        std::printf("INFO 显示器 %d：%s%s，位置 (%d, %d)，当前 %dx%d@%dHz，支持 %zu 种模式，缩放 %.2f\n", m.index,
                    m.name.c_str(), m.primary ? "（主）" : "", m.x, m.y, m.current.width, m.current.height,
                    m.current.refreshRate, m.modes.size(), static_cast<double>(m.contentScale));
    }
    const auto& primary = monitors.front();
    check(primary.primary && primary.index == 0, "第一个显示器为主显示器");
    check(primary.current.width > 0 && primary.current.height > 0, "当前模式尺寸有效");
    bool listed = false;
    for (const auto& vm : primary.modes) listed = listed || vm == primary.current;
    check(listed, "当前模式在支持列表中");

    WindowDescription desc;
    desc.title = "terra display test";
    desc.width = 640;
    desc.height = 480;
    desc.visible = true;  // 全屏切换需要可见窗口（Xvfb 中没有真实显示器）
    auto made = platform.createWindow(desc);
    if (!made.window) {
        std::fprintf(stderr, "FAILED: 创建窗口：%s\n", made.error ? made.error->message.c_str() : "");
        return 1;
    }
    auto& window = *made.window;
    const int fullW = primary.current.width;
    const int fullH = primary.current.height;
    const auto isWindowed = [&] { return window.windowSize().width == 640 && window.windowSize().height == 480; };
    const auto isFull = [&] { return window.framebufferSize().width == fullW && window.framebufferSize().height == fullH; };

    check(window.displayMode() == DisplayMode::Windowed && waitFor(platform, isWindowed), "初始为 640x480 窗口");

    check(!window.setDisplayMode(DisplayMode::BorderlessFullscreen).has_value(), "切换到无边框全屏");
    check(window.displayMode() == DisplayMode::BorderlessFullscreen, "模式为无边框全屏");
    check(waitFor(platform, isFull), "无边框全屏：帧缓冲为显示器尺寸，实际 " + extentText(window.framebufferSize()));

    check(!window.setDisplayMode(DisplayMode::Windowed).has_value(), "切回窗口");
    check(waitFor(platform, isWindowed), "切回窗口：恢复 640x480，实际 " + extentText(window.windowSize()));

    // F11 语义：窗口 → 无边框 → 窗口
    check(!window.toggleBorderlessFullscreen().has_value() && window.displayMode() == DisplayMode::BorderlessFullscreen,
          "F11：窗口 → 无边框全屏");
    check(waitFor(platform, isFull), "F11 后帧缓冲为显示器尺寸");
    check(!window.toggleBorderlessFullscreen().has_value() && window.displayMode() == DisplayMode::Windowed,
          "F11：无边框全屏 → 窗口");
    check(waitFor(platform, isWindowed), "F11 切回后恢复 640x480");

    // 独占全屏：使用显示器支持的模式
    check(!window.setDisplayMode(DisplayMode::ExclusiveFullscreen, 0, primary.current).has_value(), "独占全屏（当前模式）");
    check(window.displayMode() == DisplayMode::ExclusiveFullscreen && waitFor(platform, isFull), "独占全屏：帧缓冲为所选模式尺寸");
    check(!window.toggleBorderlessFullscreen().has_value() && window.displayMode() == DisplayMode::Windowed,
          "F11 从独占全屏回到窗口");
    check(waitFor(platform, isWindowed), "从独占全屏回到窗口：恢复 640x480");

    // 无效请求：不改变当前模式
    const auto badMonitor = window.setDisplayMode(DisplayMode::BorderlessFullscreen, 99);
    check(badMonitor && badMonitor->kind == PlatformErrorKind::InvalidRequest, "不存在的显示器被拒绝");
    const auto badMode = window.setDisplayMode(DisplayMode::ExclusiveFullscreen, 0, terra::platform::VideoMode{12345, 77, 1});
    check(badMode && badMode->kind == PlatformErrorKind::InvalidRequest, "不支持的视频模式被拒绝（不悄悄改用最接近的模式）");
    if (badMode) std::printf("INFO %s\n", badMode->message.c_str());
    check(window.displayMode() == DisplayMode::Windowed, "无效请求不改变当前模式");

    window.setVsync(false);
    window.setVsync(true);
    return failures == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
    const char* mode = argc > 1 ? argv[1] : "window";
    int rc = 1;
    if (std::strcmp(mode, "no-display") == 0) {
        rc = runNoDisplay();
    } else if (std::strcmp(mode, "window") == 0) {
        rc = runWindow();
    } else if (std::strcmp(mode, "display") == 0) {
        rc = runDisplay();
    } else {
        std::fprintf(stderr, "用法：test_platform_window [window|no-display|display]\n");
        return 2;
    }
    if (rc == 0) std::printf("all passed\n");
    return rc;
}
