#include "terra/input/types.hpp"

#include <array>
#include <cstddef>

namespace terra::input {

namespace {

// 下标 = Key 的数值；第 0 项对应 Key::Unknown（空名称）。顺序与 Key 枚举一致（105 个键位）。
constexpr std::array<const char*, static_cast<std::size_t>(Key::Count)> kKeyNames = {
    "",
    "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T",
    "U", "V", "W", "X", "Y", "Z", "Digit0", "Digit1", "Digit2", "Digit3", "Digit4", "Digit5", "Digit6",
    "Digit7", "Digit8", "Digit9", "Space", "Enter", "Escape", "Tab", "Backspace", "Insert", "Delete",
    "Right", "Left", "Down", "Up", "PageUp", "PageDown", "Home", "End", "CapsLock", "ScrollLock",
    "NumLock", "PrintScreen", "Pause", "F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10",
    "F11", "F12", "LeftShift", "LeftControl", "LeftAlt", "LeftSuper", "RightShift", "RightControl",
    "RightAlt", "RightSuper", "Menu", "Minus", "Equal", "LeftBracket", "RightBracket", "Backslash",
    "Semicolon", "Apostrophe", "GraveAccent", "Comma", "Period", "Slash", "Keypad0", "Keypad1",
    "Keypad2", "Keypad3", "Keypad4", "Keypad5", "Keypad6", "Keypad7", "Keypad8", "Keypad9",
    "KeypadDecimal", "KeypadDivide", "KeypadMultiply", "KeypadSubtract", "KeypadAdd", "KeypadEnter",
    "KeypadEqual",
};

constexpr bool allNamed() {
    for (const char* name : kKeyNames) {
        if (name == nullptr) return false;
    }
    return true;
}
static_assert(allNamed(), "kKeyNames 必须覆盖全部键位");

}  // namespace

const char* keyName(Key key) noexcept {
    const auto index = static_cast<std::size_t>(key);
    return index < kKeyNames.size() ? kKeyNames[index] : "";
}

Key keyFromName(std::string_view name) noexcept {
    if (name.empty()) return Key::Unknown;
    for (std::size_t i = 1; i < kKeyNames.size(); ++i) {
        if (name == kKeyNames[i]) return static_cast<Key>(i);
    }
    return Key::Unknown;
}

}  // namespace terra::input
