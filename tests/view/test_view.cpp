// 视图层测试（M1-07）：相机数学与视口相机控制器（单一组合式控制，D-023）。
//
// 模式：
//   test_view               自动化（不需要窗口）：相机基向量、矩阵、俯仰限制；视口相机控制器的每种操作；
//                           重复注册被拒绝；按键改绑（M1-14）
//   test_view --show <秒>   展示：桌面会话中打开可交互窗口（地面网格、坐标轴、彩色立方体），Esc 退出；到时自动关闭
// 控制器只通过虚拟设备注入的输入动作驱动，本文件的自动化部分不涉及窗口库或 OpenGL。

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "terra/input/input_system.hpp"
#include "terra/platform/glfw/glfw_platform.hpp"
#include "terra/render/context.hpp"
#include "terra/render/resources.hpp"
#include "terra/view/camera.hpp"
#include "terra/view/camera_controller.hpp"

namespace {

namespace in = terra::input;
namespace v = terra::view;
using terra::data::Vec3;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) <= tol; }
bool near(const Vec3& a, const Vec3& b, double tol = 1e-9) {
    return near(a.x, b.x, tol) && near(a.y, b.y, tol) && near(a.z, b.z, tol);
}
double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
double length(const Vec3& a) { return std::sqrt(dot(a, a)); }
Vec3 sub(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 add(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 mul(const Vec3& a, double k) { return {a.x * k, a.y * k, a.z * k}; }
std::string str(const Vec3& a) {
    return "(" + std::to_string(a.x) + ", " + std::to_string(a.y) + ", " + std::to_string(a.z) + ")";
}

// 列主序矩阵乘向量
std::array<double, 4> applyMatrix(const v::Matrix4& m, const std::array<double, 4>& p) {
    std::array<double, 4> out{};
    for (std::size_t r = 0; r < 4; ++r) {
        for (std::size_t c = 0; c < 4; ++c) out[r] += static_cast<double>(m[c * 4 + r]) * p[c];
    }
    return out;
}

std::array<double, 3> ndc(const v::Camera& cam, const Vec3& p, double aspect) {
    const auto view = applyMatrix(cam.viewMatrix(), {p.x, p.y, p.z, 1.0});
    const auto clip = applyMatrix(cam.projectionMatrix(aspect), view);
    return {clip[0] / clip[3], clip[1] / clip[3], clip[2] / clip[3]};
}

void testCameraMath() {
    v::Camera cam;
    check(near(cam.position, {-5, 0, 2}), "默认位置 (-5, 0, 2)");
    const Vec3 toOrigin = mul(sub(Vec3{}, cam.position), 1.0 / length(cam.position));
    check(near(cam.forward(), toOrigin, 1e-12), "默认朝向原点");

    for (const double yaw : {0.0, 0.7, -2.0, 3.0}) {
        for (const double pitch : {0.0, 0.5, -1.2}) {
            cam.setOrientation(yaw, pitch);
            const Vec3 f = cam.forward(), r = cam.right(), u = cam.up();
            const bool ortho = near(length(f), 1, 1e-12) && near(length(r), 1, 1e-12) && near(length(u), 1, 1e-12) &&
                               near(dot(f, r), 0, 1e-12) && near(dot(f, u), 0, 1e-12) && near(dot(r, u), 0, 1e-12);
            check(ortho && r.z == 0.0 && u.z > 0.0,
                  "基向量正交归一、right 水平、up 朝上（yaw " + std::to_string(yaw) + "，pitch " + std::to_string(pitch) + "）");
        }
    }
    cam.setOrientation(0.0, 0.0);
    check(near(cam.forward(), {1, 0, 0}) && near(cam.right(), {0, -1, 0}) && near(cam.up(), {0, 0, 1}),
          "yaw 0 pitch 0：前 +X、右 -Y、上 +Z（Terra 约定 +Y 向左）");
    cam.setOrientation(0.0, 2.0);
    check(near(cam.pitch(), v::Camera::kMaxPitch), "俯仰限制在 +89°");
    cam.setOrientation(3.0 * kPi, -2.0);
    check(near(cam.yaw(), kPi, 1e-12) && near(cam.pitch(), -v::Camera::kMaxPitch), "偏航归一化到 (-π, π]，俯仰限制在 -89°");

    cam.position = {1, 2, 3};
    cam.lookAt({1, 2, 3});
    check(std::isfinite(cam.yaw()) && std::isfinite(cam.pitch()), "看向自身位置时朝向不变、不产生 NaN");

    // 视图矩阵：正前方 5 m 处 → 视图空间 (0, 0, -5)；右侧 1 m → x = +1
    cam = v::Camera{};
    const Vec3 ahead = add(cam.position, mul(cam.forward(), 5.0));
    const auto va = applyMatrix(cam.viewMatrix(), {ahead.x, ahead.y, ahead.z, 1.0});
    check(near(va[0], 0, 1e-5) && near(va[1], 0, 1e-5) && near(va[2], -5, 1e-5), "视图矩阵：正前方点 → (0, 0, -5)");
    const Vec3 side = add(cam.position, cam.right());
    const auto vs = applyMatrix(cam.viewMatrix(), {side.x, side.y, side.z, 1.0});
    check(near(vs[0], 1, 1e-5) && near(vs[1], 0, 1e-5), "视图矩阵：右侧点 → x = +1");

    // 投影：注视点在中心；+Y（左）在屏幕左侧；+Z 在上方；近/远平面映射到 z = -1 / +1
    cam.setOrientation(0.0, 0.0);
    cam.position = {-5, 0, 0};
    const auto c = ndc(cam, {0, 0, 0}, 16.0 / 9.0);
    const auto l = ndc(cam, {0, 1, 0}, 16.0 / 9.0);
    const auto u = ndc(cam, {0, 0, 1}, 16.0 / 9.0);
    check(near(c[0], 0, 1e-6) && near(c[1], 0, 1e-6), "投影：注视点在中心");
    check(l[0] < 0 && u[1] > 0, "投影：+Y 在左、+Z 在上");
    const auto nearPoint = ndc(cam, {-5 + cam.nearPlane, 0, 0}, 1.0);
    const auto farPoint = ndc(cam, {-5 + cam.farPlane, 0, 0}, 1.0);
    check(near(nearPoint[2], -1, 1e-4) && near(farPoint[2], 1, 1e-4), "投影：近/远平面 → z = -1 / +1");
}

struct Devices {
    in::DeviceId keyboard;
    in::DeviceId mouse;
};

Devices connect(in::InputSystem& input) {
    return {input.connectDevice({in::DeviceKind::Keyboard, "kb", "virtual", ""}),
            input.connectDevice({in::DeviceKind::Mouse, "mouse", "virtual", ""})};
}

// 一帧：先注入，再 update，然后控制器更新。
void frame(in::InputSystem& input, v::CameraController& controller, v::Camera& cam, double dt) {
    input.update();
    controller.update(cam, dt);
}

void drag(in::InputSystem& input, const Devices& d, float dx, float dy) {
    input.addDelta(d.mouse, in::mouse(in::MouseControl::DeltaX), dx, 0.0);
    input.addDelta(d.mouse, in::mouse(in::MouseControl::DeltaY), dy, 0.0);
}

void setButton(in::InputSystem& input, in::DeviceId dev, in::Control c, bool down) {
    input.setControl(dev, c, down ? 1.0f : 0.0f, 0.0);
}

// 按住一个键 dt 秒（一帧），返回相机位移。
Vec3 hold(in::InputSystem& input, v::CameraController& controller, v::Camera& cam, in::DeviceId kb, in::Key k,
          double dt) {
    const Vec3 before = cam.position;
    setButton(input, kb, in::key(k), true);
    frame(input, controller, cam, dt);
    setButton(input, kb, in::key(k), false);
    frame(input, controller, cam, 0.0);
    return sub(cam.position, before);
}

void testController() {
    in::InputSystem input;
    const Devices d = connect(input);
    v::CameraController controller(input);
    v::Camera cam;
    v::CameraController::reset(cam);
    check(near(cam.position, {-5, 0, 2}, 1e-12) && near(cam.forward(), mul(sub(Vec3{}, cam.position), 1.0 / length(cam.position)), 1e-12),
          "默认视角：从 (-5, 0, 2) 看向原点，实际位置 " + str(cam.position));

    // 不按键拖动：不转视角（松开时鼠标留给界面）
    const double yaw0 = cam.yaw(), pitch0 = cam.pitch();
    drag(input, d, 40, 30);
    frame(input, controller, cam, 0.016);
    check(near(cam.yaw(), yaw0, 1e-12) && near(cam.pitch(), pitch0, 1e-12), "未按中键：拖动不改变视角");

    // 左键拖动也不转视角（左键留给以后的选择/拖动物体）
    setButton(input, d.mouse, in::mouse(in::MouseControl::ButtonLeft), true);
    drag(input, d, 40, 30);
    frame(input, controller, cam, 0.016);
    setButton(input, d.mouse, in::mouse(in::MouseControl::ButtonLeft), false);
    check(near(cam.yaw(), yaw0, 1e-12), "左键拖动：不改变视角");

    // 右键拖动也不转视角（2026-11-19 起转视角改用中键，右键留给界面）
    setButton(input, d.mouse, in::mouse(in::MouseControl::ButtonRight), true);
    drag(input, d, 40, 30);
    frame(input, controller, cam, 0.016);
    setButton(input, d.mouse, in::mouse(in::MouseControl::ButtonRight), false);
    check(near(cam.yaw(), yaw0, 1e-12), "右键拖动：不改变视角");

    // 中键拖动转视角：向右 20 像素 → 偏航 -3°；向下 20 像素 → 俯仰 -3°；位置不变
    const Vec3 p0 = cam.position;
    setButton(input, d.mouse, in::mouse(in::MouseControl::ButtonMiddle), true);
    drag(input, d, 20, 20);
    frame(input, controller, cam, 0.016);
    setButton(input, d.mouse, in::mouse(in::MouseControl::ButtonMiddle), false);
    check(near(cam.yaw(), yaw0 - 3 * kDeg, 1e-9) && near(cam.pitch(), pitch0 - 3 * kDeg, 1e-9),
          "中键向右下拖 20 像素：偏航 -3°、俯仰 -3°");
    check(near(cam.position, p0, 1e-12), "转视角不改变位置");

    // 前进：W 与 ↑ 都沿视线方向（含俯仰）前进 速度 × 时间
    const Vec3 f = cam.forward();
    check(near(hold(input, controller, cam, d.keyboard, in::Key::W, 1.0), mul(f, 3.0), 1e-9), "W 1 秒：沿视线前进 3 m（含俯仰，飞行式）");
    check(near(hold(input, controller, cam, d.keyboard, in::Key::Up, 0.5), mul(f, 1.5), 1e-9), "↑ 0.5 秒：沿视线前进 1.5 m");
    check(near(hold(input, controller, cam, d.keyboard, in::Key::Down, 0.5), mul(f, -1.5), 1e-9), "↓ 0.5 秒：后退 1.5 m");
    const Vec3 r = cam.right();
    check(near(hold(input, controller, cam, d.keyboard, in::Key::D, 1.0), mul(r, 3.0), 1e-9) && r.z == 0.0, "D 1 秒：水平右移 3 m");
    check(near(hold(input, controller, cam, d.keyboard, in::Key::Left, 1.0), mul(r, -3.0), 1e-9), "← 1 秒：水平左移 3 m");
    check(near(hold(input, controller, cam, d.keyboard, in::Key::E, 1.0), {0, 0, 3.0}, 1e-9), "E 1 秒：沿世界 Z 上升 3 m");
    check(near(hold(input, controller, cam, d.keyboard, in::Key::Space, 1.0), {0, 0, 3.0}, 1e-9), "空格 1 秒：上升 3 m");
    check(near(hold(input, controller, cam, d.keyboard, in::Key::LeftControl, 1.0), {0, 0, 0}, 1e-12), "Ctrl 不再移动相机（留给界面快捷键）");
    check(near(hold(input, controller, cam, d.keyboard, in::Key::Q, 0.5), {0, 0, -1.5}, 1e-9), "Q 0.5 秒：下降 1.5 m");

    // W + D 同时按：两方向叠加
    Vec3 before = cam.position;
    setButton(input, d.keyboard, in::key(in::Key::W), true);
    setButton(input, d.keyboard, in::key(in::Key::D), true);
    frame(input, controller, cam, 1.0);
    setButton(input, d.keyboard, in::key(in::Key::W), false);
    setButton(input, d.keyboard, in::key(in::Key::D), false);
    frame(input, controller, cam, 0.0);
    check(near(sub(cam.position, before), add(mul(f, 3.0), mul(r, 3.0)), 1e-9), "W + D：前进与右移叠加");

    // Shift 加速 3 倍
    setButton(input, d.keyboard, in::key(in::Key::LeftShift), true);
    check(near(hold(input, controller, cam, d.keyboard, in::Key::W, 1.0), mul(f, 9.0), 1e-9), "Shift + W 1 秒：前进 9 m");
    setButton(input, d.keyboard, in::key(in::Key::LeftShift), false);

    // 滚轮：向上 1 格沿视线前进 0.5 m，向下 2 格后退 1 m
    before = cam.position;
    input.addDelta(d.mouse, in::mouse(in::MouseControl::ScrollY), 1.0f, 0.0);
    frame(input, controller, cam, 0.016);
    check(near(sub(cam.position, before), mul(f, 0.5), 1e-9), "滚轮向上 1 格：拉近 0.5 m");
    before = cam.position;
    input.addDelta(d.mouse, in::mouse(in::MouseControl::ScrollY), -2.0f, 0.0);
    frame(input, controller, cam, 0.016);
    check(near(sub(cam.position, before), mul(f, -1.0), 1e-9), "滚轮向下 2 格：拉远 1 m");

    // Shift + 中键平移：向右拖 → 相机向左；向下拖 → 相机向上；平移期间不转视角（高优先级动作表消费鼠标增量）
    before = cam.position;
    const double yawBeforePan = cam.yaw();
    setButton(input, d.keyboard, in::key(in::Key::LeftShift), true);
    setButton(input, d.mouse, in::mouse(in::MouseControl::ButtonMiddle), true);
    drag(input, d, 100, 0);
    frame(input, controller, cam, 0.016);
    check(near(sub(cam.position, before), mul(r, -1.0), 1e-9), "Shift + 中键向右拖 100 像素：相机向左 1 m");
    check(near(cam.yaw(), yawBeforePan, 1e-12), "Shift + 中键平移时不转视角");
    before = cam.position;
    drag(input, d, 0, 100);
    frame(input, controller, cam, 0.016);
    setButton(input, d.mouse, in::mouse(in::MouseControl::ButtonMiddle), false);
    setButton(input, d.keyboard, in::key(in::Key::LeftShift), false);
    check(near(sub(cam.position, before), cam.up(), 1e-9), "Shift + 中键向下拖 100 像素：相机沿视平面向上 1 m");

    // F 复位
    setButton(input, d.keyboard, in::key(in::Key::F), true);
    frame(input, controller, cam, 0.016);
    setButton(input, d.keyboard, in::key(in::Key::F), false);
    check(near(cam.position, {-5, 0, 2}, 1e-12) && near(cam.yaw(), 0.0, 1e-12), "F：恢复默认视角，实际位置 " + str(cam.position));

    // 停用后不响应
    controller.setEnabled(false);
    check(near(hold(input, controller, cam, d.keyboard, in::Key::W, 1.0), {0, 0, 0}, 1e-12), "停用后按键不移动相机");
    controller.setEnabled(true);

    // 同一个输入系统上第二个控制器：动作表重名，明确失败
    bool threw = false;
    try {
        v::CameraController second(input);
    } catch (const std::logic_error& e) {
        threw = true;
        std::printf("INFO 重复注册：%s\n", e.what());
    }
    check(threw, "同一个输入系统上创建第二个相机控制器被拒绝");
}

// 按键设置（M1-14）：改绑后新键生效、旧键失效；平移修饰键跟随加速键；清空槽位；按下中改绑产生抬起。
void testControllerKeys() {
    in::InputSystem input;
    const Devices d = connect(input);
    v::CameraController controller(input);
    v::Camera cam;
    v::CameraController::reset(cam);
    check(controller.keys() == v::CameraKeys{}, "默认按键与 CameraKeys{} 相同");

    v::CameraKeys keys;
    keys.forward = {in::Key::I, in::Key::Unknown};
    keys.boost = {in::Key::LeftAlt, in::Key::Unknown};
    keys.reset = {in::Key::Home, in::Key::Unknown};
    keys.up = {in::Key::PageUp, in::Key::Unknown};
    controller.setKeys(keys);
    check(controller.keys() == keys, "setKeys 后 keys() 返回新按键");

    const Vec3 f = cam.forward();
    check(near(hold(input, controller, cam, d.keyboard, in::Key::I, 1.0), mul(f, 3.0), 1e-9), "改绑后 I 前进 3 m");
    check(near(hold(input, controller, cam, d.keyboard, in::Key::W, 1.0), {0, 0, 0}, 1e-12), "改绑后 W 不再前进");
    check(near(hold(input, controller, cam, d.keyboard, in::Key::Up, 1.0), {0, 0, 0}, 1e-12), "清空的备用槽位（↑）不再前进");
    check(near(hold(input, controller, cam, d.keyboard, in::Key::S, 1.0), mul(f, -3.0), 1e-9), "未改的后退键 S 仍有效");
    check(near(hold(input, controller, cam, d.keyboard, in::Key::Space, 1.0), {0, 0, 0}, 1e-12), "上升备用键清空后空格无效");
    check(near(hold(input, controller, cam, d.keyboard, in::Key::PageUp, 1.0), {0, 0, 3.0}, 1e-9), "改绑后 PageUp 上升");
    check(near(hold(input, controller, cam, d.keyboard, in::Key::Q, 1.0), {0, 0, -3.0}, 1e-9), "下降键 Q 仍有效");

    // 加速键改为 Alt：Alt + I 加速；Shift 不再加速
    setButton(input, d.keyboard, in::key(in::Key::LeftAlt), true);
    check(near(hold(input, controller, cam, d.keyboard, in::Key::I, 1.0), mul(f, 9.0), 1e-9), "Alt + I：加速 3 倍");
    setButton(input, d.keyboard, in::key(in::Key::LeftAlt), false);
    setButton(input, d.keyboard, in::key(in::Key::LeftShift), true);
    check(near(hold(input, controller, cam, d.keyboard, in::Key::I, 1.0), mul(f, 3.0), 1e-9), "Shift 不再加速");

    // 平移修饰键跟随加速键：Shift + 中键 现在是转视角；Alt + 中键 平移
    const Vec3 r = cam.right();
    Vec3 before = cam.position;
    const double yaw0 = cam.yaw();
    setButton(input, d.mouse, in::mouse(in::MouseControl::ButtonMiddle), true);
    drag(input, d, 100, 0);
    frame(input, controller, cam, 0.016);
    check(near(cam.position, before, 1e-12) && !near(cam.yaw(), yaw0, 1e-6), "Shift + 中键：改绑后为转视角而非平移");
    setButton(input, d.keyboard, in::key(in::Key::LeftShift), false);
    cam.setOrientation(yaw0, cam.pitch());
    setButton(input, d.keyboard, in::key(in::Key::LeftAlt), true);
    drag(input, d, 100, 0);
    frame(input, controller, cam, 0.016);
    setButton(input, d.keyboard, in::key(in::Key::LeftAlt), false);
    setButton(input, d.mouse, in::mouse(in::MouseControl::ButtonMiddle), false);
    frame(input, controller, cam, 0.0);
    check(near(sub(cam.position, before), mul(r, -1.0), 1e-9) && near(cam.yaw(), yaw0, 1e-12), "Alt + 中键：平移 1 m，不转视角");

    // 复位键改为 Home
    cam.position = {1, 2, 3};
    (void)hold(input, controller, cam, d.keyboard, in::Key::F, 0.016);
    check(near(cam.position, {1, 2, 3}, 1e-12), "F 不再复位");
    (void)hold(input, controller, cam, d.keyboard, in::Key::Home, 0.016);
    check(near(cam.position, {-5, 0, 2}, 1e-12), "Home 复位");

    // 按住 I 时改回默认：下一帧 I 不再前进
    setButton(input, d.keyboard, in::key(in::Key::I), true);
    frame(input, controller, cam, 0.0);
    controller.setKeys(v::CameraKeys{});
    before = cam.position;
    frame(input, controller, cam, 1.0);
    setButton(input, d.keyboard, in::key(in::Key::I), false);
    check(near(cam.position, before, 1e-12), "按住中改绑：旧键立即失效");
    check(near(hold(input, controller, cam, d.keyboard, in::Key::W, 1.0), mul(cam.forward(), 3.0), 1e-9), "恢复默认后 W 前进");

    // 全部清空：键盘不再驱动相机，鼠标转视角仍可用
    v::CameraKeys none;
    for (auto* slots : {&none.forward, &none.back, &none.left, &none.right, &none.up, &none.down, &none.boost, &none.reset}) {
        *slots = {in::Key::Unknown, in::Key::Unknown};
    }
    controller.setKeys(none);
    check(near(hold(input, controller, cam, d.keyboard, in::Key::W, 1.0), {0, 0, 0}, 1e-12), "清空全部按键后 W 无效");
    const double yaw1 = cam.yaw();
    setButton(input, d.mouse, in::mouse(in::MouseControl::ButtonMiddle), true);
    drag(input, d, 20, 0);
    frame(input, controller, cam, 0.016);
    setButton(input, d.mouse, in::mouse(in::MouseControl::ButtonMiddle), false);
    check(near(cam.yaw(), yaw1 - 3 * kDeg, 1e-9), "清空按键后中键转视角仍有效");
}

// ---- 展示 ----

struct Vertex {
    float position[3];
    float color[3];
};

std::vector<Vertex> buildScene(int& gridCount, int& axesFirst, int& cubeFirst) {
    std::vector<Vertex> verts;
    // 地面网格：XY 平面，-10..10 m，每 1 m 一条线；每 5 m 稍亮；跳过坐标轴所在的线
    for (int i = -10; i <= 10; ++i) {
        if (i == 0) continue;
        const float c = i % 5 == 0 ? 0.36f : 0.24f;
        const auto fi = static_cast<float>(i);
        verts.push_back({{fi, -10, 0}, {c, c, c}});
        verts.push_back({{fi, 10, 0}, {c, c, c}});
        verts.push_back({{-10, fi, 0}, {c, c, c}});
        verts.push_back({{10, fi, 0}, {c, c, c}});
    }
    // 负半轴的暗线
    verts.push_back({{-10, 0, 0}, {0.30f, 0.30f, 0.30f}});
    verts.push_back({{0, 0, 0}, {0.30f, 0.30f, 0.30f}});
    verts.push_back({{0, -10, 0}, {0.30f, 0.30f, 0.30f}});
    verts.push_back({{0, 0, 0}, {0.30f, 0.30f, 0.30f}});
    gridCount = static_cast<int>(verts.size());
    // 坐标轴：X 红、Y 绿、Z 蓝，长 10 / 10 / 2 m
    axesFirst = gridCount;
    verts.push_back({{0, 0, 0.001f}, {0.90f, 0.25f, 0.25f}});
    verts.push_back({{10, 0, 0.001f}, {0.90f, 0.25f, 0.25f}});
    verts.push_back({{0, 0, 0.001f}, {0.45f, 0.80f, 0.30f}});
    verts.push_back({{0, 10, 0.001f}, {0.45f, 0.80f, 0.30f}});
    verts.push_back({{0, 0, 0}, {0.30f, 0.50f, 0.95f}});
    verts.push_back({{0, 0, 2}, {0.30f, 0.50f, 0.95f}});
    // 立方体：边长 1 m，底面贴地，六个面不同颜色
    cubeFirst = static_cast<int>(verts.size());
    const float h = 0.5f;
    const float corners[8][3] = {{-h, -h, 0}, {h, -h, 0}, {h, h, 0}, {-h, h, 0},
                                 {-h, -h, 1}, {h, -h, 1}, {h, h, 1}, {-h, h, 1}};
    const int faces[6][4] = {{0, 1, 2, 3}, {4, 5, 6, 7}, {0, 1, 5, 4}, {2, 3, 7, 6}, {1, 2, 6, 5}, {3, 0, 4, 7}};
    const float colors[6][3] = {{0.35f, 0.35f, 0.40f}, {0.85f, 0.85f, 0.88f}, {0.80f, 0.45f, 0.30f},
                                {0.30f, 0.60f, 0.85f}, {0.85f, 0.70f, 0.30f}, {0.45f, 0.75f, 0.50f}};
    for (int f = 0; f < 6; ++f) {
        const int* q = faces[f];
        const float* col = colors[f];
        for (const int idx : {q[0], q[1], q[2], q[0], q[2], q[3]}) {
            const float* p = corners[idx];
            verts.push_back({{p[0], p[1], p[2]}, {col[0], col[1], col[2]}});
        }
    }
    return verts;
}

v::Matrix4 multiply(const v::Matrix4& a, const v::Matrix4& b) {
    v::Matrix4 out{};
    for (std::size_t c = 0; c < 4; ++c) {
        for (std::size_t r = 0; r < 4; ++r) {
            float s = 0.0f;
            for (std::size_t k = 0; k < 4; ++k) s += a[k * 4 + r] * b[c * 4 + k];
            out[c * 4 + r] = s;
        }
    }
    return out;
}

int runShow(double seconds) {
    in::InputSystem input;
    auto platform = terra::platform::glfw::createPlatform(input);
    if (!platform.platform) {
        std::fprintf(stderr, "展示：无法创建平台：%s\n", platform.error ? platform.error->message.c_str() : "");
        return 1;
    }
    in::ActionMapDefinition app{"demo", 100, true, {}};
    using KeyAction = std::pair<const char*, in::Key>;
    for (const auto& [name, k] : {KeyAction{"quit", in::Key::Escape}, KeyAction{"fullscreen", in::Key::F11}}) {
        in::ActionDefinition a;
        a.name = name;
        a.type = in::ActionType::Button;
        a.bindings = {in::Binding::single(in::key(k))};
        app.actions.push_back(a);
    }
    (void)input.addActionMap(app);
    const in::ActionHandle quit = input.findAction("demo", "quit");
    const in::ActionHandle fullscreen = input.findAction("demo", "fullscreen");
    v::CameraController controller(input);
    v::Camera camera;
    v::CameraController::reset(camera);

    terra::platform::WindowDescription desc;
    desc.width = 1280;
    desc.height = 720;
    auto window = platform.platform->createWindow(desc);
    if (!window.window) {
        std::fprintf(stderr, "展示：无法创建窗口：%s\n", window.error ? window.error->message.c_str() : "");
        return 1;
    }
    auto created = terra::render::RenderContext::create(window.window->glProcLoader());
    if (!created.context) {
        std::fprintf(stderr, "展示：初始化渲染失败：%s\n", created.error.value_or("").c_str());
        return 1;
    }
    auto& ctx = *created.context;

    const char* vs = R"(#version 450 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aColor;
uniform mat4 uViewProjection;
out vec3 vColor;
void main() { vColor = aColor; gl_Position = uViewProjection * vec4(aPosition, 1.0); }
)";
    const char* fs = R"(#version 450 core
in vec3 vColor;
out vec4 fragColor;
void main() { fragColor = vec4(vColor, 1.0); }
)";
    auto shader = terra::render::ShaderProgram::create(
        {{terra::render::ShaderStage::Vertex, "scene.vert", vs}, {terra::render::ShaderStage::Fragment, "scene.frag", fs}});
    if (!shader.program) {
        std::fprintf(stderr, "展示：着色器：%s\n", shader.error ? shader.error->summary().c_str() : "");
        return 1;
    }
    int gridCount = 0, axesFirst = 0, cubeFirst = 0;
    const std::vector<Vertex> verts = buildScene(gridCount, axesFirst, cubeFirst);
    auto buffer = terra::render::Buffer::create(verts.size() * sizeof(Vertex), verts.data());
    auto vao = terra::render::VertexArray::create();
    vao.setVertexBuffer(0, buffer, 0, sizeof(Vertex));
    vao.setFloatAttribute(0, 0, 3, offsetof(Vertex, position));
    vao.setFloatAttribute(1, 0, 3, offsetof(Vertex, color));
    const int location = shader.program->uniformLocation("uViewProjection");

    window.window->setTitle("EAI Terra · 视口（中键拖动转视角，WASD/方向键移动，滚轮拉近，F11 全屏，Esc 退出）");
    for (const auto& m : platform.platform->monitors()) {
        std::printf("INFO 显示器 %d：%s%s，%dx%d@%dHz，支持 %zu 种模式\n", m.index, m.name.c_str(), m.primary ? "（主）" : "",
                    m.current.width, m.current.height, m.current.refreshRate, m.modes.size());
    }
    std::printf("INFO 展示：%s\n", ctx.info().renderer.c_str());
    std::printf("INFO 中键拖动转视角；W/↑ S/↓ 沿视线前后，A/← D/→ 左右；E/空格 上升，Q 下降；\n");
    std::printf("INFO 滚轮拉近/拉远；Shift+中键拖动平移；Shift 加速；F 复位；F11 全屏/窗口；Esc 退出\n");

    const double start = platform.platform->time();
    double last = start;
    while (!window.window->closeRequested()) {
        platform.platform->pollEvents();
        input.update();
        if (input.state(quit).pressed) break;
        if (input.state(fullscreen).pressed) {
            if (const auto error = window.window->toggleBorderlessFullscreen()) {
                std::printf("INFO 切换全屏失败：%s\n", error->message.c_str());
            } else {
                std::printf("INFO 显示模式：%s\n", terra::platform::displayModeName(window.window->displayMode()));
            }
        }
        const double now = platform.platform->time();
        if (now - start >= seconds) break;
        controller.update(camera, now - last);
        last = now;

        const auto fb = window.window->framebufferSize();
        ctx.setViewport(0, 0, fb.width, fb.height);
        ctx.setDepthTest(true);
        ctx.clear({0.11f, 0.11f, 0.12f, 1.0f});
        const double aspect = fb.height > 0 ? static_cast<double>(fb.width) / fb.height : 1.0;
        const v::Matrix4 vp = multiply(camera.projectionMatrix(aspect), camera.viewMatrix());
        shader.program->use();
        shader.program->setUniformMatrix4(location, vp.data());
        vao.bind();
        ctx.drawLines(0, gridCount);
        ctx.drawLines(axesFirst, 6);
        ctx.drawTriangles(cubeFirst, 36);
        window.window->swapBuffers();
    }
    std::printf("INFO 展示结束：%.1f 秒\n", platform.platform->time() - start);
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "--show") == 0) {
        const double seconds = argc > 2 ? std::atof(argv[2]) : 60.0;
        return runShow(seconds > 0 ? seconds : 60.0);
    }
    testCameraMath();
    testController();
    testControllerKeys();
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
