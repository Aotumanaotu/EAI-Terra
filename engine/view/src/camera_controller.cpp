#include "terra/view/camera_controller.hpp"

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace terra::view {

namespace {

namespace in = terra::input;
using terra::data::Vec3;

constexpr double kDegree = 3.14159265358979323846 / 180.0;

Vec3 add(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 scale(Vec3 a, double k) { return {a.x * k, a.y * k, a.z * k}; }

in::ActionDefinition action(const char* name, in::ActionType type, std::vector<in::Binding> bindings) {
    in::ActionDefinition a;
    a.name = name;
    a.type = type;
    a.bindings = std::move(bindings);
    return a;
}

}  // namespace

CameraController::CameraController(in::InputSystem& input, CameraControlSettings config)
    : input_(input), settings_(config) {
    const in::Control dx = in::mouse(in::MouseControl::DeltaX);
    const in::Control dy = in::mouse(in::MouseControl::DeltaY);
    in::ActionMapDefinition map{
        kMapName,
        0,
        true,
        {action("look", in::ActionType::Delta2D,
                {in::Binding::pair(dx, dy).withModifier(in::mouse(in::MouseControl::ButtonRight))}),
         action("move", in::ActionType::Axis2D,
                {in::Binding::composite2D(in::key(in::Key::W), in::key(in::Key::S), in::key(in::Key::A), in::key(in::Key::D)),
                 in::Binding::composite2D(in::key(in::Key::Up), in::key(in::Key::Down), in::key(in::Key::Left), in::key(in::Key::Right))}),
         action("lift", in::ActionType::Axis1D,
                {in::Binding::composite1D(in::key(in::Key::Q), in::key(in::Key::E)),
                 in::Binding::composite1D(in::key(in::Key::LeftControl), in::key(in::Key::Space))}),
         action("dolly", in::ActionType::Axis1D, {in::Binding::single(in::mouse(in::MouseControl::ScrollY))}),
         action("pan", in::ActionType::Delta2D,
                {in::Binding::pair(dx, dy).withModifier(in::mouse(in::MouseControl::ButtonMiddle))}),
         action("boost", in::ActionType::Button,
                {in::Binding::single(in::key(in::Key::LeftShift)), in::Binding::single(in::key(in::Key::RightShift))}),
         action("reset", in::ActionType::Button, {in::Binding::single(in::key(in::Key::F))})}};
    if (auto error = input_.addActionMap(std::move(map))) {
        // 只可能是编程错误（例如同一个 InputSystem 上创建了两个控制器）
        throw std::logic_error(std::string{"注册相机动作表失败："} + *error);
    }
    look_ = input_.findAction(kMapName, "look");
    move_ = input_.findAction(kMapName, "move");
    lift_ = input_.findAction(kMapName, "lift");
    dolly_ = input_.findAction(kMapName, "dolly");
    pan_ = input_.findAction(kMapName, "pan");
    boost_ = input_.findAction(kMapName, "boost");
    reset_ = input_.findAction(kMapName, "reset");
}

void CameraController::setEnabled(bool enabled) { input_.setActionMapEnabled(kMapName, enabled); }

void CameraController::reset(Camera& camera) {
    camera.position = {-5.0, 0.0, 2.0};
    camera.lookAt({0.0, 0.0, 0.0});
}

void CameraController::update(Camera& camera, double dt) {
    if (input_.state(reset_).pressed) {
        reset(camera);
        return;
    }

    const in::ActionState& look = input_.state(look_);
    if (look.down) {
        const double k = settings_.lookDegreesPerPixel * kDegree;
        camera.setOrientation(camera.yaw() - static_cast<double>(look.x) * k,
                              camera.pitch() - static_cast<double>(look.y) * k);
    }

    const double factor = input_.state(boost_).down ? settings_.boost : 1.0;
    const Vec3 forward = camera.forward();
    const Vec3 right = camera.right();

    // 键盘：前后沿视线（飞行式），左右水平，升降沿世界 Z。
    const double v = settings_.speed * factor * dt;
    const in::ActionState& m = input_.state(move_);
    camera.position = add(camera.position, add(scale(forward, static_cast<double>(m.y) * v),
                                               scale(right, static_cast<double>(m.x) * v)));
    camera.position.z += static_cast<double>(input_.state(lift_).x) * v;

    // 滚轮：向上（正）拉近 = 沿视线前进。
    const float wheel = input_.state(dolly_).x;
    if (wheel != 0.0f) {
        camera.position = add(camera.position, scale(forward, static_cast<double>(wheel) * settings_.wheelStep * factor));
    }

    // 中键平移：场景跟随鼠标（向右拖 → 相机向左移；向下拖 → 相机向上移）。
    const in::ActionState& pan = input_.state(pan_);
    if (pan.down) {
        const double k = settings_.panMetersPerPixel * factor;
        camera.position = add(camera.position, add(scale(right, -static_cast<double>(pan.x) * k),
                                                   scale(camera.up(), static_cast<double>(pan.y) * k)));
    }
}

}  // namespace terra::view
