#include "terra/view/camera_controller.hpp"

#include <cstddef>
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

// 由按键槽位生成绑定：每个槽位一组；整组为空的槽位不生成绑定。
std::vector<in::Binding> moveBindings(const CameraKeys& k) {
    std::vector<in::Binding> out;
    for (std::size_t i = 0; i < 2; ++i) {
        if (k.forward[i] == in::Key::Unknown && k.back[i] == in::Key::Unknown && k.left[i] == in::Key::Unknown &&
            k.right[i] == in::Key::Unknown) {
            continue;
        }
        out.push_back(in::Binding::composite2D(in::key(k.forward[i]), in::key(k.back[i]), in::key(k.left[i]),
                                               in::key(k.right[i])));
    }
    return out;
}

std::vector<in::Binding> liftBindings(const CameraKeys& k) {
    std::vector<in::Binding> out;
    for (std::size_t i = 0; i < 2; ++i) {
        if (k.down[i] == in::Key::Unknown && k.up[i] == in::Key::Unknown) continue;
        out.push_back(in::Binding::composite1D(in::key(k.down[i]), in::key(k.up[i])));
    }
    return out;
}

std::vector<in::Binding> buttonBindings(const CameraKeys::Slots& slots) {
    std::vector<in::Binding> out;
    for (const in::Key slot : slots) {
        if (slot != in::Key::Unknown) out.push_back(in::Binding::single(in::key(slot)));
    }
    return out;
}

// Shift + 中键 = 平移：修饰键与加速键相同。
std::vector<in::Binding> panBindings(const CameraKeys& k) {
    const in::Control dx = in::mouse(in::MouseControl::DeltaX);
    const in::Control dy = in::mouse(in::MouseControl::DeltaY);
    std::vector<in::Binding> out;
    for (const in::Key modifier : k.boost) {
        if (modifier == in::Key::Unknown) continue;
        out.push_back(in::Binding::pair(dx, dy)
                          .withModifier(in::mouse(in::MouseControl::ButtonMiddle))
                          .withModifier(in::key(modifier)));
    }
    return out;
}

}  // namespace

CameraController::CameraController(in::InputSystem& input, CameraControlSettings config, const CameraKeys& keys)
    : input_(input), settings_(config), keys_(keys) {
    const in::Control dx = in::mouse(in::MouseControl::DeltaX);
    const in::Control dy = in::mouse(in::MouseControl::DeltaY);
    in::ActionMapDefinition map{
        kMapName,
        0,
        true,
        {action("look", in::ActionType::Delta2D,
                {in::Binding::pair(dx, dy).withModifier(in::mouse(in::MouseControl::ButtonMiddle))}),
         action("move", in::ActionType::Axis2D, moveBindings(keys_)),
         action("lift", in::ActionType::Axis1D, liftBindings(keys_)),
         action("dolly", in::ActionType::Axis1D, {in::Binding::single(in::mouse(in::MouseControl::ScrollY))}),
         action("boost", in::ActionType::Button, buttonBindings(keys_.boost)),
         action("reset", in::ActionType::Button, buttonBindings(keys_.reset))}};
    // 平移放在更高优先级的表中，生效时消费鼠标增量，低优先级的“中键转视角”拿不到。
    in::ActionMapDefinition panMap{kPanMapName, 1, true, {action("pan", in::ActionType::Delta2D, panBindings(keys_))}};
    for (auto* m : {&map, &panMap}) {
        if (auto error = input_.addActionMap(std::move(*m))) {
            // 只可能是编程错误（例如同一个 InputSystem 上创建了两个控制器）
            throw std::logic_error(std::string{"注册相机动作表失败："} + *error);
        }
    }
    look_ = input_.findAction(kMapName, "look");
    move_ = input_.findAction(kMapName, "move");
    lift_ = input_.findAction(kMapName, "lift");
    dolly_ = input_.findAction(kMapName, "dolly");
    pan_ = input_.findAction(kPanMapName, "pan");
    boost_ = input_.findAction(kMapName, "boost");
    reset_ = input_.findAction(kMapName, "reset");
}

void CameraController::setKeys(const CameraKeys& keys) {
    keys_ = keys;
    const std::pair<const char*, std::vector<in::Binding>> updates[] = {
        {"move", moveBindings(keys_)},
        {"lift", liftBindings(keys_)},
        {"boost", buttonBindings(keys_.boost)},
        {"reset", buttonBindings(keys_.reset)},
    };
    for (const auto& u : updates) {
        if (auto error = input_.setBindings(kMapName, u.first, u.second)) {
            throw std::logic_error(std::string{"更新相机按键失败："} + *error);
        }
    }
    if (auto error = input_.setBindings(kPanMapName, "pan", panBindings(keys_))) {
        throw std::logic_error(std::string{"更新相机按键失败："} + *error);
    }
}

void CameraController::setEnabled(bool enabled) {
    input_.setActionMapEnabled(kMapName, enabled);
    input_.setActionMapEnabled(kPanMapName, enabled);
}

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

    // Shift+中键平移：场景跟随鼠标（向右拖 → 相机向左移；向下拖 → 相机向上移）。平移时不加速（Shift 已用作修饰键）。
    const in::ActionState& pan = input_.state(pan_);
    if (pan.down) {
        const double k = settings_.panMetersPerPixel;
        camera.position = add(camera.position, add(scale(right, -static_cast<double>(pan.x) * k),
                                                   scale(camera.up(), static_cast<double>(pan.y) * k)));
    }
}

}  // namespace terra::view
