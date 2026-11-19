#pragma once

// GLFW 平台实现的入口（M1-03，D-017）。本头文件不包含 GLFW 头文件，也不暴露任何 GLFW 类型。

#include <optional>

#include "terra/input/input_sink.hpp"
#include "terra/input/types.hpp"
#include "terra/platform/platform.hpp"

namespace terra::platform::glfw {

// 初始化 GLFW 并创建平台。同一进程只能有一个实例（GLFW 是进程级单例），重复创建返回 AlreadyInitialized。
// 平台连接两个输入设备（GLFW 键盘、GLFW 鼠标），窗口事件经翻译后写入 input；input 必须比平台及其窗口活得更久。
// 没有显示服务时返回 NoDisplay，不会崩溃。
PlatformResult createPlatform(terra::input::InputSink& input);

// GLFW 编码到 Terra 控件的翻译（纯函数，便于测试；参数为 GLFW 的整数常量）。
// 无法识别的键返回 Key::Unknown；无法识别的鼠标键返回空。
terra::input::Key translateKey(int glfwKey) noexcept;
std::optional<terra::input::MouseControl> translateMouseButton(int glfwButton) noexcept;

}  // namespace terra::platform::glfw
