// 界面输入测试（M1-13）：键位映射、UiInputBridge 分流、UiInputCapture 与相机控制器的抢占。不需要显示。

#include <cmath>
#include <cstdio>
#include <set>
#include <string>
#include <vector>

#include "imgui.h"
#include "imgui_internal.h"
#include "terra/input/input_system.hpp"
#include "terra/ui/ui_input.hpp"
#include "terra/view/camera.hpp"
#include "terra/view/camera_controller.hpp"

namespace {

namespace in = terra::input;
namespace ui = terra::ui;
using Event = ui::UiInputBridge::Event;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

int count(const std::vector<Event>& events, Event::Kind kind) {
    int n = 0;
    for (const auto& e : events) n += e.kind == kind ? 1 : 0;
    return n;
}

bool hasKey(const std::vector<Event>& events, ImGuiKey key, bool down) {
    for (const auto& e : events) {
        if (e.kind == Event::Kind::Key && e.key == key && e.down == down) return true;
    }
    return false;
}

void testKeyMapping() {
    std::set<int> seen;
    int mapped = 0;
    bool collision = false;
    for (int k = 1; k < static_cast<int>(in::Key::Count); ++k) {
        const ImGuiKey key = ui::toImGuiKey(static_cast<in::Key>(k));
        if (key == ImGuiKey_None) {
            check(false, "Terra 键 " + std::to_string(k) + " 没有对应的 ImGui 键");
            continue;
        }
        ++mapped;
        collision = collision || !seen.insert(static_cast<int>(key)).second;
    }
    std::printf("INFO 映射了 %d 个键\n", mapped);
    check(mapped == static_cast<int>(in::Key::Count) - 1, "Terra 的每个键都映射到 ImGui");
    check(!collision, "映射无冲突");
    check(ui::toImGuiKey(in::Key::Unknown) == ImGuiKey_None, "Unknown → None");
    check(ui::toImGuiKey(in::Key::W) == ImGuiKey_W && ui::toImGuiKey(in::Key::Up) == ImGuiKey_UpArrow &&
              ui::toImGuiKey(in::Key::F11) == ImGuiKey_F11 && ui::toImGuiKey(in::Key::LeftControl) == ImGuiKey_LeftCtrl,
          "抽查：W、↑、F11、左 Ctrl");
}

void testBridge() {
    in::InputSystem system;
    ui::UiInputBridge bridge(system);
    const in::DeviceId kb = bridge.connectDevice({in::DeviceKind::Keyboard, "kb", "virtual", ""});
    const in::DeviceId ms = bridge.connectDevice({in::DeviceKind::Mouse, "mouse", "virtual", ""});
    check(system.devices().size() == 2, "设备连接转发到输入系统");

    // 键：同时到达输入系统与 ImGui 队列；重复的同一状态不产生新事件
    bridge.setControl(kb, in::key(in::Key::W), 1.0f, 1.0);
    bridge.setControl(kb, in::key(in::Key::W), 1.0f, 1.1);
    check(system.controlValue(kb, in::key(in::Key::W)) == 1.0f, "按键转发到输入系统");
    check(count(bridge.pending(), Event::Kind::Key) == 1 && hasKey(bridge.pending(), ImGuiKey_W, true), "ImGui 收到一次 W 按下");

    // 修饰键：左 Ctrl 产生键事件与 ImGuiMod_Ctrl
    bridge.setControl(kb, in::key(in::Key::LeftControl), 1.0f, 1.2);
    check(hasKey(bridge.pending(), ImGuiKey_LeftCtrl, true) && hasKey(bridge.pending(), ImGuiMod_Ctrl, true),
          "左 Ctrl：键事件与 Ctrl 修饰键");
    bridge.setControl(kb, in::key(in::Key::LeftControl), 0.0f, 1.3);
    check(hasKey(bridge.pending(), ImGuiMod_Ctrl, false), "松开 Ctrl：修饰键抬起");

    // 鼠标：位置 X、Y 合并为一个事件；按键；滚轮合并
    bridge.setControl(ms, in::mouse(in::MouseControl::PositionX), 100.0f, 2.0);
    bridge.setControl(ms, in::mouse(in::MouseControl::PositionY), 50.0f, 2.0);
    const auto& events = bridge.pending();
    check(count(events, Event::Kind::MousePos) == 1 && events.back().x == 100.0f && events.back().y == 50.0f,
          "位置 X、Y 合并为一个事件 (100, 50)");
    bridge.setControl(ms, in::mouse(in::MouseControl::ButtonRight), 1.0f, 2.1);
    check(count(bridge.pending(), Event::Kind::MouseButton) == 1 && bridge.pending().back().button == 1 &&
              bridge.pending().back().down,
          "右键 → ImGui 按键 1 按下");
    bridge.addDelta(ms, in::mouse(in::MouseControl::ScrollY), 1.0f, 2.2);
    bridge.addDelta(ms, in::mouse(in::MouseControl::ScrollY), 2.0f, 2.2);
    check(count(bridge.pending(), Event::Kind::Wheel) == 1 && bridge.pending().back().y == 3.0f, "滚轮合并为一个事件（3 格）");
    bridge.addDelta(ms, in::mouse(in::MouseControl::DeltaX), 5.0f, 2.3);
    check(count(bridge.pending(), Event::Kind::Wheel) == 1 && system.controlValue(ms, in::mouse(in::MouseControl::DeltaX)) == 5.0f,
          "指针增量只转发到输入系统，不进入 ImGui 队列");

    // 文本
    bridge.addText(U'设');
    check(bridge.pending().back().kind == Event::Kind::Text && bridge.pending().back().text == U'设', "文本字符进入队列");

    // 焦点丢失（resetDevice）：按下中的 W 抬起；断开鼠标：右键抬起
    bridge.resetDevice(kb);
    check(hasKey(bridge.pending(), ImGuiKey_W, false) && system.controlValue(kb, in::key(in::Key::W)) == 0.0f,
          "resetDevice：W 在 ImGui 与输入系统中都抬起");
    bridge.disconnectDevice(ms);
    check(bridge.pending().back().kind == Event::Kind::MouseButton && !bridge.pending().back().down &&
              system.devices().size() == 1,
          "断开鼠标：右键抬起，输入系统中设备移除");

    // 交给 ImGui：队列清空，ImGui 收到事件
    ImGui::CreateContext();
    const std::size_t pending = bridge.pending().size();
    bridge.flushTo(ImGui::GetIO());
    check(bridge.pending().empty(), "flushTo 后队列清空");
    check(ImGui::GetCurrentContext()->InputEventsQueue.Size > 0, "ImGui 输入队列收到事件（" + std::to_string(pending) + " 个）");
    ImGui::DestroyContext();
}

void testCapture() {
    in::InputSystem system;
    const in::DeviceId kb = system.connectDevice({in::DeviceKind::Keyboard, "kb", "virtual", ""});
    const in::DeviceId ms = system.connectDevice({in::DeviceKind::Mouse, "mouse", "virtual", ""});
    ui::UiInputCapture capture(system);
    terra::view::CameraController controller(system);
    terra::view::Camera camera;
    terra::view::CameraController::reset(camera);

    const auto dragLook = [&](bool uiMouse) {
        capture.update(uiMouse, false);
        system.setControl(ms, in::mouse(in::MouseControl::ButtonMiddle), 1.0f, 0.0);
        system.addDelta(ms, in::mouse(in::MouseControl::DeltaX), 20.0f, 0.0);
        const double before = camera.yaw();
        system.update();
        controller.update(camera, 0.016);
        system.setControl(ms, in::mouse(in::MouseControl::ButtonMiddle), 0.0f, 0.0);
        system.update();
        return camera.yaw() - before;
    };
    check(dragLook(true) == 0.0, "鼠标在面板上（界面占用鼠标）：中键拖动不转相机");
    check(dragLook(false) != 0.0, "鼠标在视口里（界面不占用）：中键拖动转相机");

    const auto holdW = [&](bool uiKeyboard) {
        capture.update(false, uiKeyboard);
        const auto before = camera.position;
        system.setControl(kb, in::key(in::Key::W), 1.0f, 0.0);
        system.update();
        controller.update(camera, 1.0);
        system.setControl(kb, in::key(in::Key::W), 0.0f, 0.0);
        system.update();
        return std::fabs(camera.position.x - before.x) + std::fabs(camera.position.y - before.y) +
               std::fabs(camera.position.z - before.z);
    };
    check(holdW(true) == 0.0, "输入框获得焦点（界面占用键盘）：W 不移动相机");
    check(holdW(false) > 0.0, "界面不占用键盘：W 移动相机");
    check(!capture.capturingKeyboard(), "状态查询");
}

}  // namespace

int main() {
    testKeyMapping();
    testBridge();
    testCapture();
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
