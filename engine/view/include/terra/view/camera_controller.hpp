#pragma once

// 视图层：视口相机控制器（M1-07，D-018、D-023）。
//
// 单一的组合式控制（用户决定，2026-11-19：不再区分环绕/飞行两种模式），类似 UE5 视口与游戏：
//   按住右键拖动   转视角（向右拖 → 右转；向下拖 → 低头）
//   W/↑ S/↓       沿视线方向前进/后退（含俯仰，即飞行式）
//   A/← D/→       水平向左/向右平移
//   E/空格 Q/Ctrl  沿世界 Z 上升/下降
//   滚轮           沿视线方向拉近/拉远（每格固定距离）
//   中键拖动       在视平面内平移（场景跟随鼠标）
//   Shift          以上移动加速
//   F              恢复默认视角
// 控制器只消费 Terra 输入动作，不知道输入来自哪个设备或窗口库。构造时向 InputSystem 注册动作表 "camera"；
// 默认绑定只是动作映射的初始值，之后可由设置覆盖（D-026）。

#include "terra/input/input_system.hpp"
#include "terra/view/camera.hpp"

namespace terra::view {

struct CameraControlSettings {
    double lookDegreesPerPixel = 0.15;  // 视角灵敏度
    double speed = 3.0;                 // 键盘移动速度，m/s
    double boost = 3.0;                 // 按住 Shift 时的倍率
    double wheelStep = 0.5;             // 每格滚轮沿视线移动的距离，m（Shift 时同样加速）
    double panMetersPerPixel = 0.01;    // 中键平移，m/像素
};

class CameraController {
public:
    static constexpr const char* kMapName = "camera";

    // 同一个 InputSystem 上只能有一个控制器（动作表重名时抛出 std::logic_error）。
    explicit CameraController(terra::input::InputSystem& input, CameraControlSettings config = {});

    // 每帧在 InputSystem::update() 之后调用；dt 为本帧时长（秒）。
    void update(Camera& camera, double dt);
    // 恢复默认视角：从 (-5, 0, 2) 看向原点。
    static void reset(Camera& camera);
    void setEnabled(bool enabled);

    CameraControlSettings& settings() noexcept { return settings_; }

private:
    terra::input::InputSystem& input_;
    CameraControlSettings settings_;
    terra::input::ActionHandle look_, move_, lift_, dolly_, pan_, boost_, reset_;
};

}  // namespace terra::view
