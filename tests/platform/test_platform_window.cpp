// 平台窗口测试（M1-03）：只通过 terra_platform 接口与 GLFW 实现入口访问，本文件不包含 GLFW 头文件。
//
// 模式（第一个参数）：
//   window      在 Xvfb + llvmpipe 中运行（D-022）：创建窗口与 4.5 core 上下文、glad 加载、尺寸、
//               请求 4.6 时返回 ContextUnavailable、重复创建平台被拒绝、低于 4.5 的请求被拒绝、
//               窗口比平台活得更久、平台连接与断开键鼠设备
//   no-display  清除 DISPLAY/WAYLAND_DISPLAY 后运行：创建平台返回 NoDisplay，不崩溃

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

}  // namespace

int main(int argc, char** argv) {
    const char* mode = argc > 1 ? argv[1] : "window";
    int rc = 1;
    if (std::strcmp(mode, "no-display") == 0) {
        rc = runNoDisplay();
    } else if (std::strcmp(mode, "window") == 0) {
        rc = runWindow();
    } else {
        std::fprintf(stderr, "用法：test_platform_window [window|no-display]\n");
        return 2;
    }
    if (rc == 0) std::printf("all passed\n");
    return rc;
}
