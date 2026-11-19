#include "terra/view/camera.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace terra::view {

namespace {

constexpr double kPi = 3.14159265358979323846;

glm::dvec3 toGlm(const terra::data::Vec3& v) { return {v.x, v.y, v.z}; }

Matrix4 toMatrix(const glm::dmat4& m) {
    Matrix4 out{};
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            out[static_cast<std::size_t>(c * 4 + r)] = static_cast<float>(m[c][r]);
        }
    }
    return out;
}

}  // namespace

void Camera::setOrientation(double newYaw, double newPitch) noexcept {
    if (std::isfinite(newYaw)) {
        double y = std::remainder(newYaw, 2.0 * kPi);  // [-π, π]
        if (y <= -kPi) y += 2.0 * kPi;
        yaw_ = y;
    }
    if (std::isfinite(newPitch)) pitch_ = std::clamp(newPitch, -kMaxPitch, kMaxPitch);
}

void Camera::lookAt(const terra::data::Vec3& target) noexcept {
    const double dx = target.x - position.x;
    const double dy = target.y - position.y;
    const double dz = target.z - position.z;
    const double horizontal = std::sqrt(dx * dx + dy * dy);
    if (horizontal == 0.0 && dz == 0.0) return;
    setOrientation(horizontal > 0.0 ? std::atan2(dy, dx) : yaw_, std::atan2(dz, horizontal));
}

terra::data::Vec3 Camera::forward() const noexcept {
    const double cp = std::cos(pitch_);
    return {cp * std::cos(yaw_), cp * std::sin(yaw_), std::sin(pitch_)};
}

terra::data::Vec3 Camera::right() const noexcept {
    return {std::sin(yaw_), -std::cos(yaw_), 0.0};
}

terra::data::Vec3 Camera::up() const noexcept {
    const terra::data::Vec3 f = forward();
    const terra::data::Vec3 r = right();
    return {r.y * f.z - r.z * f.y, r.z * f.x - r.x * f.z, r.x * f.y - r.y * f.x};  // right × forward
}

Matrix4 Camera::viewMatrix() const {
    const glm::dvec3 eye = toGlm(position);
    return toMatrix(glm::lookAt(eye, eye + toGlm(forward()), toGlm(up())));
}

Matrix4 Camera::projectionMatrix(double aspect) const {
    const double a = aspect > 0.0 && std::isfinite(aspect) ? aspect : 1.0;
    return toMatrix(glm::perspective(verticalFov, a, nearPlane, farPlane));
}

}  // namespace terra::view
