// GLFW 编码 → Terra 控件翻译表测试（M1-04）。纯函数，不初始化 GLFW，不需要显示。
// 测试直接引用 GLFW 常量作为输入，这是边界检查允许的唯一测试用法。

#include <cstdio>
#include <set>
#include <string>

#include <GLFW/glfw3.h>

#include "terra/platform/glfw/glfw_platform.hpp"

namespace {

using terra::input::Key;
using terra::input::MouseControl;
namespace g = terra::platform::glfw;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

}  // namespace

int main() {
    check(g::translateKey(GLFW_KEY_A) == Key::A && g::translateKey(GLFW_KEY_Z) == Key::Z, "字母 A..Z");
    check(g::translateKey(GLFW_KEY_W) == Key::W, "W（区间换算）");
    check(g::translateKey(GLFW_KEY_0) == Key::Digit0 && g::translateKey(GLFW_KEY_9) == Key::Digit9, "数字 0..9");
    check(g::translateKey(GLFW_KEY_F1) == Key::F1 && g::translateKey(GLFW_KEY_F12) == Key::F12, "F1..F12");
    check(g::translateKey(GLFW_KEY_KP_0) == Key::Keypad0 && g::translateKey(GLFW_KEY_KP_9) == Key::Keypad9, "小键盘 0..9");
    check(g::translateKey(GLFW_KEY_LEFT_SHIFT) == Key::LeftShift && g::translateKey(GLFW_KEY_RIGHT_SUPER) == Key::RightSuper,
          "修饰键");
    check(g::translateKey(GLFW_KEY_ESCAPE) == Key::Escape && g::translateKey(GLFW_KEY_SPACE) == Key::Space, "Esc、空格");
    check(g::translateKey(GLFW_KEY_UP) == Key::Up && g::translateKey(GLFW_KEY_LEFT) == Key::Left, "方向键");
    check(g::translateKey(GLFW_KEY_UNKNOWN) == Key::Unknown, "GLFW_KEY_UNKNOWN → Unknown");
    check(g::translateKey(GLFW_KEY_WORLD_1) == Key::Unknown, "非美式键位 WORLD_1 → Unknown");
    check(g::translateKey(GLFW_KEY_F13) == Key::Unknown, "F13（Terra 未定义）→ Unknown");

    // 所有 GLFW 键码翻译后互不冲突：不同物理键不能映射到同一个 Terra Key
    std::set<Key> seen;
    int mapped = 0;
    bool collision = false;
    for (int k = GLFW_KEY_SPACE; k <= GLFW_KEY_LAST; ++k) {
        const Key t = g::translateKey(k);
        if (t == Key::Unknown) continue;
        ++mapped;
        collision = collision || !seen.insert(t).second;
    }
    check(!collision, "翻译表无冲突");
    const int terraKeys = static_cast<int>(Key::Count) - 1;  // 去掉 Unknown
    std::printf("INFO 翻译了 %d 个 GLFW 键码，Terra 定义 %d 个键\n", mapped, terraKeys);
    check(mapped == terraKeys, "Terra 定义的每个键都有对应的 GLFW 键码");

    check(g::translateMouseButton(GLFW_MOUSE_BUTTON_LEFT) == MouseControl::ButtonLeft, "鼠标左键");
    check(g::translateMouseButton(GLFW_MOUSE_BUTTON_RIGHT) == MouseControl::ButtonRight, "鼠标右键");
    check(g::translateMouseButton(GLFW_MOUSE_BUTTON_MIDDLE) == MouseControl::ButtonMiddle, "鼠标中键");
    check(g::translateMouseButton(GLFW_MOUSE_BUTTON_5) == MouseControl::Button5, "鼠标键 5");
    check(!g::translateMouseButton(GLFW_MOUSE_BUTTON_8).has_value(), "鼠标键 8 未定义 → 空");

    if (failures == 0) {
        std::printf("all passed\n");
    }
    return failures == 0 ? 0 : 1;
}
