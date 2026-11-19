#pragma once

// 界面输入（M1-13，D-018、D-024）。
//
// UiInputBridge：分流写入端。平台把事件写给它，它原样转发给目标（InputSystem），同时记录一份，
// 在每帧开始时按顺序交给 ImGui。ImGui 因此不接触任何窗口库。
// UiInputCapture：界面与视口抢输入。注册最高优先级、默认停用的两张动作表，绑定全部鼠标控件或全部按键并消费；
// 界面需要输入时启用，低优先级的相机等动作表就拿不到这些控件（复用 M1-04 的优先级与消费机制）。

#include <cstddef>
#include <cstdint>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "imgui.h"
#include "terra/input/input_sink.hpp"
#include "terra/input/input_system.hpp"

namespace terra::ui {

// Terra 物理键位 → ImGui 键；Key::Unknown 与越界值返回 ImGuiKey_None。
ImGuiKey toImGuiKey(terra::input::Key key) noexcept;

class UiInputBridge final : public terra::input::InputSink {
public:
    struct Event {
        enum class Kind { Key, MouseButton, MousePos, Wheel, Text };
        Kind kind = Kind::Key;
        ImGuiKey key = ImGuiKey_None;  // Key（含 ImGuiMod_* 修饰键）
        int button = 0;                // MouseButton：0 左、1 右、2 中、3、4
        bool down = false;             // Key / MouseButton
        float x = 0.0f;                // MousePos：窗口坐标；Wheel：水平
        float y = 0.0f;                // MousePos：窗口坐标；Wheel：竖直（正值向上）
        char32_t text = 0;             // Text
    };

    explicit UiInputBridge(terra::input::InputSink& target);

    terra::input::DeviceId connectDevice(const terra::input::DeviceDescription& description) override;
    void disconnectDevice(terra::input::DeviceId id) override;
    void resetDevice(terra::input::DeviceId id) override;
    void setControl(terra::input::DeviceId id, terra::input::Control control, float value, double timestamp) override;
    void addDelta(terra::input::DeviceId id, terra::input::Control control, float delta, double timestamp) override;
    void addText(char32_t codepoint) override;

    // 把积压的事件按顺序交给 ImGui（每帧 ImGui::NewFrame() 之前调用），然后清空。
    void flushTo(ImGuiIO& io);
    const std::vector<Event>& pending() const noexcept { return events_; }

private:
    void keyEvent(terra::input::Key key, bool down);
    void modifierEvents();
    void releaseHeld(terra::input::DeviceId id);

    terra::input::InputSink& target_;
    std::unordered_map<std::uint32_t, terra::input::DeviceKind> kinds_;
    std::set<std::pair<std::uint32_t, std::uint16_t>> held_;  // (设备, 控件编码)：按下中的键与鼠标键
    bool modifiers_[4] = {false, false, false, false};          // Ctrl、Shift、Alt、Super
    float mouseX_ = 0.0f;
    float mouseY_ = 0.0f;
    std::vector<Event> events_;
};

class UiInputCapture {
public:
    static constexpr const char* kMouseMap = "ui_capture_mouse";
    static constexpr const char* kKeyboardMap = "ui_capture_keyboard";
    static constexpr int kPriority = 1000;

    // 同一个 InputSystem 上只能有一个（动作表重名时抛出 std::logic_error）。
    explicit UiInputCapture(terra::input::InputSystem& input);

    // 每帧调用：mouse 为真时界面占用全部鼠标控件；keyboard 为真时占用全部按键（例如输入框获得焦点）。
    void update(bool mouse, bool keyboard);
    bool capturingMouse() const noexcept { return mouse_; }
    bool capturingKeyboard() const noexcept { return keyboard_; }

private:
    terra::input::InputSystem& input_;
    bool mouse_ = false;
    bool keyboard_ = false;
};

}  // namespace terra::ui
