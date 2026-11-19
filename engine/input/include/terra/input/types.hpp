#pragma once

// Terra 输入：设备与控件（M1-04，D-018）。
//
// 本模块不依赖任何窗口库或输入库。各输入来源（GLFW、虚拟设备，后续 SDL、OpenXR、ROS2）由适配器把
// 自己的编码翻译成这里的控件，再通过 InputSink 写入输入系统。
//
// 控件值统一用 float：按键 0/1；摇杆轴 -1..1（+X 向右，+Y 向上/向前）；扳机 0..1；
// 增量类控件（鼠标移动、滚轮）为本帧累计值，单位为像素或滚轮格数。

#include <cstdint>
#include <string>
#include <string_view>

namespace terra::input {

enum class DeviceKind : std::uint8_t {
    Keyboard,
    Mouse,
    Gamepad,
    XrController,  // 预留：OpenXR（位姿类输入需扩展值类型，届时评审）
    External,      // 外部指令来源，如 ROS2（M4）
};

const char* deviceKindName(DeviceKind kind) noexcept;

// 设备连接实例的标识，由输入系统分配，只增不复用；设备断开后旧 ID 失效。0 无效。
struct DeviceId {
    std::uint32_t value = 0;

    constexpr bool valid() const noexcept { return value != 0; }
    friend constexpr bool operator==(DeviceId a, DeviceId b) noexcept { return a.value == b.value; }
    friend constexpr bool operator!=(DeviceId a, DeviceId b) noexcept { return a.value != b.value; }
    friend constexpr bool operator<(DeviceId a, DeviceId b) noexcept { return a.value < b.value; }
};

// 一个控件：设备种类 + 种类内的编码。
struct Control {
    DeviceKind kind = DeviceKind::Keyboard;
    std::uint16_t code = 0;

    friend constexpr bool operator==(Control a, Control b) noexcept { return a.kind == b.kind && a.code == b.code; }
    friend constexpr bool operator!=(Control a, Control b) noexcept { return !(a == b); }
};

// 键盘按物理键位命名（美式布局名称，与 USB HID、GLFW、SDL 的键位语义一致），不随键盘布局变化。
// 文字输入另走文本通道（M1 不做）。
enum class Key : std::uint16_t {
    Unknown = 0,
    A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U, V, W, X, Y, Z,
    Digit0, Digit1, Digit2, Digit3, Digit4, Digit5, Digit6, Digit7, Digit8, Digit9,
    Space, Enter, Escape, Tab, Backspace, Insert, Delete,
    Right, Left, Down, Up, PageUp, PageDown, Home, End,
    CapsLock, ScrollLock, NumLock, PrintScreen, Pause,
    F1, F2, F3, F4, F5, F6, F7, F8, F9, F10, F11, F12,
    LeftShift, LeftControl, LeftAlt, LeftSuper, RightShift, RightControl, RightAlt, RightSuper, Menu,
    Minus, Equal, LeftBracket, RightBracket, Backslash, Semicolon, Apostrophe, GraveAccent, Comma, Period, Slash,
    Keypad0, Keypad1, Keypad2, Keypad3, Keypad4, Keypad5, Keypad6, Keypad7, Keypad8, Keypad9,
    KeypadDecimal, KeypadDivide, KeypadMultiply, KeypadSubtract, KeypadAdd, KeypadEnter, KeypadEqual,
    Count,
};

enum class MouseControl : std::uint16_t {
    ButtonLeft,
    ButtonRight,
    ButtonMiddle,
    Button4,
    Button5,
    PositionX,  // 绝对值：窗口内容区坐标，像素，原点左上
    PositionY,
    DeltaX,     // 增量：本帧指针移动，像素，+X 向右
    DeltaY,     // 增量：+Y 向下（与窗口坐标一致）
    ScrollX,    // 增量：滚轮格数
    ScrollY,    // 增量：+Y 为向上/远离用户
    Count,
};

// 手柄按标准布局命名（与 GLFW/SDL 的 gamepad mapping 一致）。
enum class GamepadControl : std::uint16_t {
    A, B, X, Y,
    LeftBumper, RightBumper, Back, Start, Guide, LeftThumb, RightThumb,
    DpadUp, DpadRight, DpadDown, DpadLeft,
    LeftX, LeftY, RightX, RightY,  // -1..1，+Y 向上（推杆向前）
    LeftTrigger, RightTrigger,     // 0..1
    Count,
};

// 键位的稳定名称（与枚举名相同，如 "W"、"Up"、"LeftShift"、"Digit1"、"F11"），用于配置文件与界面显示（M1-14，D-026）。
// Key::Unknown 与越界值返回空字符串；keyFromName 区分大小写，未知名称返回 Key::Unknown。
const char* keyName(Key key) noexcept;
Key keyFromName(std::string_view name) noexcept;

constexpr Control key(Key k) noexcept { return Control{DeviceKind::Keyboard, static_cast<std::uint16_t>(k)}; }
constexpr Control mouse(MouseControl m) noexcept { return Control{DeviceKind::Mouse, static_cast<std::uint16_t>(m)}; }
constexpr Control gamepad(GamepadControl g) noexcept {
    return Control{DeviceKind::Gamepad, static_cast<std::uint16_t>(g)};
}

struct DeviceDescription {
    DeviceKind kind = DeviceKind::Keyboard;
    std::string name;       // 可读名称，如 "Xbox Controller"
    std::string source;     // 适配器名，如 "glfw"、"virtual"
    std::string stableKey;  // 跨重连识别同一物理设备（如手柄 GUID），可为空
};

struct DeviceInfo {
    DeviceId id;
    DeviceDescription description;
};

}  // namespace terra::input
