#pragma once

// 视图层：视口相机控制器（M1-07，D-018、D-023）。
//
// 单一的组合式控制（用户决定，2026-11-19：不再区分环绕/飞行两种模式；转视角改用中键，与 Blender 一致，
// 左右键留给界面与以后的选择操作）：
//   按住中键拖动   转视角（向右拖 → 右转；向下拖 → 低头）
//   W/↑ S/↓       沿视线方向前进/后退（含俯仰，即飞行式）
//   A/← D/→       水平向左/向右平移
//   E/空格 Q      沿世界 Z 上升/下降（Ctrl 留给界面快捷键，如 Ctrl+加减号缩放界面）
//   滚轮           沿视线方向拉近/拉远（每格固定距离）
//   Shift + 中键拖动  在视平面内平移（场景跟随鼠标）
//   Shift          以上移动加速
//   F              恢复默认视角
// 控制器只消费 Terra 输入动作，不知道输入来自哪个设备或窗口库。构造时向 InputSystem 注册动作表 "camera" 与
// "camera_pan"（Shift+中键平移，优先级更高，生效时消费鼠标增量，于是中键转视角不会同时触发）；
// 键盘按键可由设置覆盖（M1-14，D-026，setKeys）；鼠标操作固定。

#include <array>

#include "terra/input/input_system.hpp"
#include "terra/view/camera.hpp"

namespace terra::view {

struct CameraControlSettings {
    double lookDegreesPerPixel = 0.15;  // 视角灵敏度
    double speed = 3.0;                 // 键盘移动速度，m/s
    double boost = 3.0;                 // 按住加速键时的倍率
    double wheelStep = 0.5;             // 每格滚轮沿视线移动的距离，m（加速时同样加速）
    double panMetersPerPixel = 0.01;    // Shift+中键平移，m/像素
};

// 键盘按键：每个操作两个槽位（主键与备用键），Key::Unknown 表示空。
// 平移的修饰键与加速键相同（默认 Shift）。
struct CameraKeys {
    using Slots = std::array<terra::input::Key, 2>;
    Slots forward{terra::input::Key::W, terra::input::Key::Up};
    Slots back{terra::input::Key::S, terra::input::Key::Down};
    Slots left{terra::input::Key::A, terra::input::Key::Left};
    Slots right{terra::input::Key::D, terra::input::Key::Right};
    Slots up{terra::input::Key::E, terra::input::Key::Space};
    Slots down{terra::input::Key::Q, terra::input::Key::Unknown};
    Slots boost{terra::input::Key::LeftShift, terra::input::Key::RightShift};
    Slots reset{terra::input::Key::F, terra::input::Key::Unknown};

    friend bool operator==(const CameraKeys& a, const CameraKeys& b) noexcept {
        return a.forward == b.forward && a.back == b.back && a.left == b.left && a.right == b.right && a.up == b.up &&
               a.down == b.down && a.boost == b.boost && a.reset == b.reset;
    }
    friend bool operator!=(const CameraKeys& a, const CameraKeys& b) noexcept { return !(a == b); }
};

class CameraController {
public:
    static constexpr const char* kMapName = "camera";
    static constexpr const char* kPanMapName = "camera_pan";

    // 同一个 InputSystem 上只能有一个控制器（动作表重名时抛出 std::logic_error）。
    explicit CameraController(terra::input::InputSystem& input, CameraControlSettings config = {},
                              const CameraKeys& keys = {});

    // 每帧在 InputSystem::update() 之后调用；dt 为本帧时长（秒）。
    void update(Camera& camera, double dt);
    // 恢复默认视角：从 (-5, 0, 2) 看向原点。
    static void reset(Camera& camera);
    void setEnabled(bool enabled);

    CameraControlSettings& settings() noexcept { return settings_; }
    // 替换键盘按键（立即生效；按下中的键若不再绑定，下一帧产生“抬起”）。
    void setKeys(const CameraKeys& keys);
    const CameraKeys& keys() const noexcept { return keys_; }

private:
    terra::input::InputSystem& input_;
    CameraControlSettings settings_;
    CameraKeys keys_;
    terra::input::ActionHandle look_, move_, lift_, dolly_, pan_, boost_, reset_;
};

}  // namespace terra::view
