#pragma once

// terra_viewer 的设置换算与“偏好设置”窗口（M1-14，D-026）。
//
// 设置模块（terra_settings）不依赖平台、视图与界面；这里把设置换算成各模块的参数，并提供编辑设置的窗口。
// 窗口直接修改传入的 Settings；查看器每帧比较前后差异并让改动立即生效（见 main.cpp 的 applySettings）。

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "terra/input/action_map.hpp"
#include "terra/platform/platform.hpp"
#include "demo_scene.hpp"  // 渲染设置换算（toSunLight 等）与 kRadiansPerDegree
#include "terra/settings/settings.hpp"
#include "terra/view/camera_controller.hpp"

namespace terra::viewer {

terra::view::CameraKeys toCameraKeys(const terra::settings::KeyMap& keys);
terra::view::CameraControlSettings toCameraControl(const terra::settings::CameraSettings& camera);
terra::platform::DisplayMode toDisplayMode(terra::settings::WindowMode mode) noexcept;
terra::settings::WindowMode toWindowMode(terra::platform::DisplayMode mode) noexcept;
// 一个按钮动作的绑定：每个非空槽位一个单键绑定。
std::vector<terra::input::Binding> buttonBindings(const terra::settings::KeySlots& slots);

struct SettingsWindow {
    enum Page { Interface, Display, Camera, Keys, PageCount };

    bool open = false;
    int page = Interface;
    // 按键录制：点击按键格后打开模态弹窗，下一帧起读取按下的键；Esc 取消。
    std::optional<terra::settings::Command> capture;
    std::size_t captureSlot = 0;
    int captureFrames = 0;
    bool openCapturePopup = false;
    std::string captureMessage;  // 例如“Ctrl 是保留键”
    // 拖动界面缩放滑块时的临时值：松开后才写入设置（避免拖动中字号变化导致滑块跳动）。
    bool editingScale = false;
    float scaleEdit = 100.0f;

    bool capturingKey() const noexcept { return capture.has_value(); }
};

// 绘制偏好设置窗口（open 为假时只清理录制状态）。currentUiScale：当前生效的界面缩放（关闭“自动”时作为初值）；
// path：设置文件路径，为空表示本次运行不保存；log 收到“按键已从某操作移走”等提示。
void drawSettingsWindow(SettingsWindow& window, terra::settings::Settings& settings,
                        const std::vector<terra::platform::MonitorInfo>& monitors, float currentUiScale,
                        const std::string& path, std::vector<std::string>& log);

}  // namespace terra::viewer
