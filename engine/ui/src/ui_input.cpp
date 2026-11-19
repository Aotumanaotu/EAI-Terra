#include "terra/ui/ui_input.hpp"

#include <stdexcept>
#include <string>

namespace terra::ui {

namespace in = terra::input;

ImGuiKey toImGuiKey(in::Key key) noexcept {
    using K = in::Key;
    const auto offset = [](K k, K first, ImGuiKey imguiFirst) {
        return static_cast<ImGuiKey>(static_cast<int>(imguiFirst) + (static_cast<int>(k) - static_cast<int>(first)));
    };
    if (key >= K::A && key <= K::Z) return offset(key, K::A, ImGuiKey_A);
    if (key >= K::Digit0 && key <= K::Digit9) return offset(key, K::Digit0, ImGuiKey_0);
    if (key >= K::F1 && key <= K::F12) return offset(key, K::F1, ImGuiKey_F1);
    if (key >= K::Keypad0 && key <= K::Keypad9) return offset(key, K::Keypad0, ImGuiKey_Keypad0);
    switch (key) {
        case K::Space: return ImGuiKey_Space;
        case K::Enter: return ImGuiKey_Enter;
        case K::Escape: return ImGuiKey_Escape;
        case K::Tab: return ImGuiKey_Tab;
        case K::Backspace: return ImGuiKey_Backspace;
        case K::Insert: return ImGuiKey_Insert;
        case K::Delete: return ImGuiKey_Delete;
        case K::Right: return ImGuiKey_RightArrow;
        case K::Left: return ImGuiKey_LeftArrow;
        case K::Down: return ImGuiKey_DownArrow;
        case K::Up: return ImGuiKey_UpArrow;
        case K::PageUp: return ImGuiKey_PageUp;
        case K::PageDown: return ImGuiKey_PageDown;
        case K::Home: return ImGuiKey_Home;
        case K::End: return ImGuiKey_End;
        case K::CapsLock: return ImGuiKey_CapsLock;
        case K::ScrollLock: return ImGuiKey_ScrollLock;
        case K::NumLock: return ImGuiKey_NumLock;
        case K::PrintScreen: return ImGuiKey_PrintScreen;
        case K::Pause: return ImGuiKey_Pause;
        case K::LeftShift: return ImGuiKey_LeftShift;
        case K::LeftControl: return ImGuiKey_LeftCtrl;
        case K::LeftAlt: return ImGuiKey_LeftAlt;
        case K::LeftSuper: return ImGuiKey_LeftSuper;
        case K::RightShift: return ImGuiKey_RightShift;
        case K::RightControl: return ImGuiKey_RightCtrl;
        case K::RightAlt: return ImGuiKey_RightAlt;
        case K::RightSuper: return ImGuiKey_RightSuper;
        case K::Menu: return ImGuiKey_Menu;
        case K::Minus: return ImGuiKey_Minus;
        case K::Equal: return ImGuiKey_Equal;
        case K::LeftBracket: return ImGuiKey_LeftBracket;
        case K::RightBracket: return ImGuiKey_RightBracket;
        case K::Backslash: return ImGuiKey_Backslash;
        case K::Semicolon: return ImGuiKey_Semicolon;
        case K::Apostrophe: return ImGuiKey_Apostrophe;
        case K::GraveAccent: return ImGuiKey_GraveAccent;
        case K::Comma: return ImGuiKey_Comma;
        case K::Period: return ImGuiKey_Period;
        case K::Slash: return ImGuiKey_Slash;
        case K::KeypadDecimal: return ImGuiKey_KeypadDecimal;
        case K::KeypadDivide: return ImGuiKey_KeypadDivide;
        case K::KeypadMultiply: return ImGuiKey_KeypadMultiply;
        case K::KeypadSubtract: return ImGuiKey_KeypadSubtract;
        case K::KeypadAdd: return ImGuiKey_KeypadAdd;
        case K::KeypadEnter: return ImGuiKey_KeypadEnter;
        case K::KeypadEqual: return ImGuiKey_KeypadEqual;
        default: return ImGuiKey_None;
    }
}

namespace {

bool isMouseButton(std::uint16_t code) noexcept {
    return code <= static_cast<std::uint16_t>(in::MouseControl::Button5);
}

}  // namespace

UiInputBridge::UiInputBridge(in::InputSink& target) : target_(target) {}

in::DeviceId UiInputBridge::connectDevice(const in::DeviceDescription& description) {
    const in::DeviceId id = target_.connectDevice(description);
    kinds_[id.value] = description.kind;
    return id;
}

void UiInputBridge::disconnectDevice(in::DeviceId id) {
    target_.disconnectDevice(id);
    releaseHeld(id);
    kinds_.erase(id.value);
}

void UiInputBridge::resetDevice(in::DeviceId id) {
    target_.resetDevice(id);
    releaseHeld(id);
}

void UiInputBridge::setControl(in::DeviceId id, in::Control control, float value, double timestamp) {
    target_.setControl(id, control, value, timestamp);
    const std::pair<std::uint32_t, std::uint16_t> slot{id.value, control.code};
    const bool down = value >= 0.5f;
    if (control.kind == in::DeviceKind::Keyboard) {
        const bool was = held_.count(slot) != 0;
        if (down == was) return;
        if (down) held_.insert(slot); else held_.erase(slot);
        keyEvent(static_cast<in::Key>(control.code), down);
        modifierEvents();
    } else if (control.kind == in::DeviceKind::Mouse) {
        if (isMouseButton(control.code)) {
            const bool was = held_.count(slot) != 0;
            if (down == was) return;
            if (down) held_.insert(slot); else held_.erase(slot);
            Event e;
            e.kind = Event::Kind::MouseButton;
            e.button = static_cast<int>(control.code);
            e.down = down;
            events_.push_back(e);
        } else if (control.code == static_cast<std::uint16_t>(in::MouseControl::PositionX) ||
                   control.code == static_cast<std::uint16_t>(in::MouseControl::PositionY)) {
            (control.code == static_cast<std::uint16_t>(in::MouseControl::PositionX) ? mouseX_ : mouseY_) = value;
            // 连续的位置事件合并为一个
            if (!events_.empty() && events_.back().kind == Event::Kind::MousePos) {
                events_.back().x = mouseX_;
                events_.back().y = mouseY_;
            } else {
                Event e;
                e.kind = Event::Kind::MousePos;
                e.x = mouseX_;
                e.y = mouseY_;
                events_.push_back(e);
            }
        }
    }
}

void UiInputBridge::addDelta(in::DeviceId id, in::Control control, float delta, double timestamp) {
    target_.addDelta(id, control, delta, timestamp);
    if (control.kind != in::DeviceKind::Mouse) return;
    const bool horizontal = control.code == static_cast<std::uint16_t>(in::MouseControl::ScrollX);
    const bool vertical = control.code == static_cast<std::uint16_t>(in::MouseControl::ScrollY);
    if (!horizontal && !vertical) return;
    if (!events_.empty() && events_.back().kind == Event::Kind::Wheel) {
        (horizontal ? events_.back().x : events_.back().y) += delta;
        return;
    }
    Event e;
    e.kind = Event::Kind::Wheel;
    (horizontal ? e.x : e.y) = delta;
    events_.push_back(e);
}

void UiInputBridge::addText(char32_t codepoint) {
    target_.addText(codepoint);
    Event e;
    e.kind = Event::Kind::Text;
    e.text = codepoint;
    events_.push_back(e);
}

void UiInputBridge::keyEvent(in::Key key, bool down) {
    const ImGuiKey k = toImGuiKey(key);
    if (k == ImGuiKey_None) return;
    Event e;
    e.kind = Event::Kind::Key;
    e.key = k;
    e.down = down;
    events_.push_back(e);
}

void UiInputBridge::modifierEvents() {
    const auto heldKey = [&](in::Key k) {
        for (const auto& [device, code] : held_) {
            const auto it = kinds_.find(device);
            if (it != kinds_.end() && it->second == in::DeviceKind::Keyboard && code == static_cast<std::uint16_t>(k)) {
                return true;
            }
        }
        return false;
    };
    const bool now[4] = {heldKey(in::Key::LeftControl) || heldKey(in::Key::RightControl),
                         heldKey(in::Key::LeftShift) || heldKey(in::Key::RightShift),
                         heldKey(in::Key::LeftAlt) || heldKey(in::Key::RightAlt),
                         heldKey(in::Key::LeftSuper) || heldKey(in::Key::RightSuper)};
    const ImGuiKey mods[4] = {ImGuiMod_Ctrl, ImGuiMod_Shift, ImGuiMod_Alt, ImGuiMod_Super};
    for (int i = 0; i < 4; ++i) {
        if (now[i] == modifiers_[i]) continue;
        modifiers_[i] = now[i];
        Event e;
        e.kind = Event::Kind::Key;
        e.key = mods[i];
        e.down = now[i];
        events_.push_back(e);
    }
}

void UiInputBridge::releaseHeld(in::DeviceId id) {
    const auto kind = kinds_.find(id.value);
    for (auto it = held_.begin(); it != held_.end();) {
        if (it->first != id.value) {
            ++it;
            continue;
        }
        if (kind != kinds_.end() && kind->second == in::DeviceKind::Keyboard) {
            keyEvent(static_cast<in::Key>(it->second), false);
        } else if (kind != kinds_.end() && kind->second == in::DeviceKind::Mouse) {
            Event e;
            e.kind = Event::Kind::MouseButton;
            e.button = static_cast<int>(it->second);
            e.down = false;
            events_.push_back(e);
        }
        it = held_.erase(it);
    }
    modifierEvents();
}

void UiInputBridge::flushTo(ImGuiIO& io) {
    for (const Event& e : events_) {
        switch (e.kind) {
            case Event::Kind::Key: io.AddKeyEvent(e.key, e.down); break;
            case Event::Kind::MouseButton: io.AddMouseButtonEvent(e.button, e.down); break;
            case Event::Kind::MousePos: io.AddMousePosEvent(e.x, e.y); break;
            case Event::Kind::Wheel: io.AddMouseWheelEvent(e.x, e.y); break;
            case Event::Kind::Text: io.AddInputCharacter(static_cast<unsigned int>(e.text)); break;
        }
    }
    events_.clear();
}

// ---- UiInputCapture ----

UiInputCapture::UiInputCapture(in::InputSystem& input) : input_(input) {
    const auto ctl = [](in::MouseControl m) { return in::mouse(m); };

    in::ActionMapDefinition mouseMap{kMouseMap, kPriority, false, {}};
    {
        in::ActionDefinition pointer;
        pointer.name = "pointer";
        pointer.type = in::ActionType::Delta2D;
        pointer.bindings = {in::Binding::pair(ctl(in::MouseControl::DeltaX), ctl(in::MouseControl::DeltaY))};
        in::ActionDefinition buttons;
        buttons.name = "buttons";
        buttons.type = in::ActionType::Button;
        for (const auto b : {in::MouseControl::ButtonLeft, in::MouseControl::ButtonRight, in::MouseControl::ButtonMiddle,
                             in::MouseControl::Button4, in::MouseControl::Button5}) {
            buttons.bindings.push_back(in::Binding::single(ctl(b)));
        }
        in::ActionDefinition scroll;
        scroll.name = "scroll";
        scroll.type = in::ActionType::Axis2D;
        scroll.bindings = {in::Binding::pair(ctl(in::MouseControl::ScrollX), ctl(in::MouseControl::ScrollY))};
        mouseMap.actions = {pointer, buttons, scroll};
    }
    in::ActionMapDefinition keyboardMap{kKeyboardMap, kPriority, false, {}};
    {
        in::ActionDefinition keys;
        keys.name = "keys";
        keys.type = in::ActionType::Button;
        for (int k = 1; k < static_cast<int>(in::Key::Count); ++k) {
            keys.bindings.push_back(in::Binding::single(in::key(static_cast<in::Key>(k))));
        }
        keyboardMap.actions = {keys};
    }
    for (auto* map : {&mouseMap, &keyboardMap}) {
        const std::string name = map->name;
        if (auto error = input_.addActionMap(std::move(*map))) {
            throw std::logic_error("注册界面输入动作表 " + name + " 失败：" + *error);
        }
    }
}

void UiInputCapture::update(bool mouse, bool keyboard) {
    mouse_ = mouse;
    keyboard_ = keyboard;
    input_.setActionMapEnabled(kMouseMap, mouse);
    input_.setActionMapEnabled(kKeyboardMap, keyboard);
}

}  // namespace terra::ui
