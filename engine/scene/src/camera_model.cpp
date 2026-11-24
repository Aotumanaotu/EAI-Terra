#include "terra/scene/camera_model.hpp"

#include <cmath>

#include "terra/scene/pose_math.hpp"

namespace terra::scene {

namespace td = terra::data;

CameraIntrinsics cameraIntrinsics(int width, int height, double fovyDegrees) {
    CameraIntrinsics k;
    k.width = width;
    k.height = height;
    const double half = fovyDegrees * 3.14159265358979323846 / 360.0;
    k.fy = 0.5 * static_cast<double>(height) / std::tan(half);
    k.fx = k.fy;
    k.cx = 0.5 * static_cast<double>(width - 1);
    k.cy = 0.5 * static_cast<double>(height - 1);
    return k;
}

td::Quaternion opticalRotation() { return td::Quaternion{0.0, 1.0, 0.0, 0.0}; }

td::Pose cameraWorldPose(const td::PhysicsSnapshot& snapshot, const CameraSensor& camera) {
    if (!camera.entity.valid() || camera.entity.value > snapshot.entities.size()) return camera.mount;
    return multiply(snapshot.entities[camera.entity.value - 1].pose, camera.mount);
}

std::optional<std::array<double, 2>> projectPoint(const CameraIntrinsics& k, const td::Pose& cameraWorld, const td::Vec3& worldPoint) {
    // 世界 → Terra 相机坐标系 → 光学坐标系（x 不变，y、z 取反）
    const td::Vec3 p = rotate(conjugate(cameraWorld.orientation),
                              {worldPoint.x - cameraWorld.position.x, worldPoint.y - cameraWorld.position.y,
                               worldPoint.z - cameraWorld.position.z});
    const double x = p.x;
    const double y = -p.y;
    const double z = -p.z;
    if (!(z > 0.0)) return std::nullopt;
    return std::array<double, 2>{k.fx * x / z + k.cx, k.fy * y / z + k.cy};
}

}  // namespace terra::scene
