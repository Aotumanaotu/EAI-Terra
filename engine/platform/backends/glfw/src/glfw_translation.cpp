// GLFW 编码 → Terra 控件（M1-04）。GLFW 键码按物理键位（美式布局）命名，与 Terra Key 语义一致。

#include "terra/platform/glfw/glfw_platform.hpp"

#include <GLFW/glfw3.h>

namespace terra::platform::glfw {

using terra::input::Key;
using terra::input::MouseControl;

namespace {

// 区间 [first, last] 在 GLFW 与 Terra 中都连续时按偏移换算。
Key offset(int glfwKey, int first, Key terraFirst) noexcept {
    return static_cast<Key>(static_cast<int>(terraFirst) + (glfwKey - first));
}

}  // namespace

Key translateKey(int k) noexcept {
    if (k >= GLFW_KEY_A && k <= GLFW_KEY_Z) return offset(k, GLFW_KEY_A, Key::A);
    if (k >= GLFW_KEY_0 && k <= GLFW_KEY_9) return offset(k, GLFW_KEY_0, Key::Digit0);
    if (k >= GLFW_KEY_F1 && k <= GLFW_KEY_F12) return offset(k, GLFW_KEY_F1, Key::F1);
    if (k >= GLFW_KEY_KP_0 && k <= GLFW_KEY_KP_9) return offset(k, GLFW_KEY_KP_0, Key::Keypad0);
    switch (k) {
        case GLFW_KEY_SPACE: return Key::Space;
        case GLFW_KEY_ENTER: return Key::Enter;
        case GLFW_KEY_ESCAPE: return Key::Escape;
        case GLFW_KEY_TAB: return Key::Tab;
        case GLFW_KEY_BACKSPACE: return Key::Backspace;
        case GLFW_KEY_INSERT: return Key::Insert;
        case GLFW_KEY_DELETE: return Key::Delete;
        case GLFW_KEY_RIGHT: return Key::Right;
        case GLFW_KEY_LEFT: return Key::Left;
        case GLFW_KEY_DOWN: return Key::Down;
        case GLFW_KEY_UP: return Key::Up;
        case GLFW_KEY_PAGE_UP: return Key::PageUp;
        case GLFW_KEY_PAGE_DOWN: return Key::PageDown;
        case GLFW_KEY_HOME: return Key::Home;
        case GLFW_KEY_END: return Key::End;
        case GLFW_KEY_CAPS_LOCK: return Key::CapsLock;
        case GLFW_KEY_SCROLL_LOCK: return Key::ScrollLock;
        case GLFW_KEY_NUM_LOCK: return Key::NumLock;
        case GLFW_KEY_PRINT_SCREEN: return Key::PrintScreen;
        case GLFW_KEY_PAUSE: return Key::Pause;
        case GLFW_KEY_LEFT_SHIFT: return Key::LeftShift;
        case GLFW_KEY_LEFT_CONTROL: return Key::LeftControl;
        case GLFW_KEY_LEFT_ALT: return Key::LeftAlt;
        case GLFW_KEY_LEFT_SUPER: return Key::LeftSuper;
        case GLFW_KEY_RIGHT_SHIFT: return Key::RightShift;
        case GLFW_KEY_RIGHT_CONTROL: return Key::RightControl;
        case GLFW_KEY_RIGHT_ALT: return Key::RightAlt;
        case GLFW_KEY_RIGHT_SUPER: return Key::RightSuper;
        case GLFW_KEY_MENU: return Key::Menu;
        case GLFW_KEY_MINUS: return Key::Minus;
        case GLFW_KEY_EQUAL: return Key::Equal;
        case GLFW_KEY_LEFT_BRACKET: return Key::LeftBracket;
        case GLFW_KEY_RIGHT_BRACKET: return Key::RightBracket;
        case GLFW_KEY_BACKSLASH: return Key::Backslash;
        case GLFW_KEY_SEMICOLON: return Key::Semicolon;
        case GLFW_KEY_APOSTROPHE: return Key::Apostrophe;
        case GLFW_KEY_GRAVE_ACCENT: return Key::GraveAccent;
        case GLFW_KEY_COMMA: return Key::Comma;
        case GLFW_KEY_PERIOD: return Key::Period;
        case GLFW_KEY_SLASH: return Key::Slash;
        case GLFW_KEY_KP_DECIMAL: return Key::KeypadDecimal;
        case GLFW_KEY_KP_DIVIDE: return Key::KeypadDivide;
        case GLFW_KEY_KP_MULTIPLY: return Key::KeypadMultiply;
        case GLFW_KEY_KP_SUBTRACT: return Key::KeypadSubtract;
        case GLFW_KEY_KP_ADD: return Key::KeypadAdd;
        case GLFW_KEY_KP_ENTER: return Key::KeypadEnter;
        case GLFW_KEY_KP_EQUAL: return Key::KeypadEqual;
        default: return Key::Unknown;  // 包括 GLFW_KEY_UNKNOWN 与 WORLD_1/2 等非美式键位
    }
}

std::optional<MouseControl> translateMouseButton(int b) noexcept {
    switch (b) {
        case GLFW_MOUSE_BUTTON_LEFT: return MouseControl::ButtonLeft;
        case GLFW_MOUSE_BUTTON_RIGHT: return MouseControl::ButtonRight;
        case GLFW_MOUSE_BUTTON_MIDDLE: return MouseControl::ButtonMiddle;
        case GLFW_MOUSE_BUTTON_4: return MouseControl::Button4;
        case GLFW_MOUSE_BUTTON_5: return MouseControl::Button5;
        default: return std::nullopt;
    }
}

}  // namespace terra::platform::glfw
