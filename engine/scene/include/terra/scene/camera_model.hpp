#pragma once

// 相机模型（M4-03，M4-01 设计稿第 3、4 节）：内参、相机在世界中的位姿、投影。纯数学，双精度，不需要图形上下文。
//
// 坐标系：
//   Terra 相机坐标系与 MuJoCo、OpenGL 相同：看向 −Z，+X 向右，+Y 向上；
//   光学坐标系（ROS 约定，sensor_msgs/CameraInfo）：看向 +Z，+X 向右，+Y 向下；由 Terra 相机坐标系绕 X 轴转 180° 得到。
// 内参（针孔模型、无畸变、方形像素）：fy = (h / 2) / tan(fovy / 2)，fx = fy；主点 cx = (w − 1) / 2，cy = (h − 1) / 2。
// 像素坐标：u 向右、v 向下，像素中心为整数（OpenCV / ROS 约定），与对称视锥渲染的图像一致。

#include <array>
#include <optional>

#include "terra/data/geometry.hpp"
#include "terra/data/state.hpp"
#include "terra/scene/scene.hpp"

namespace terra::scene {

struct CameraIntrinsics {
    int width = 0;
    int height = 0;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
};

CameraIntrinsics cameraIntrinsics(int width, int height, double fovyDegrees);

// 光学坐标系相对 Terra 相机坐标系的旋转：绕 X 轴 180°，即 (w, x, y, z) = (0, 1, 0, 0)
terra::data::Quaternion opticalRotation();

// 相机（Terra 相机坐标系）在世界中的位姿：安装实体的位姿 × 安装位姿；装在世界上时即安装位姿
terra::data::Pose cameraWorldPose(const terra::data::PhysicsSnapshot& snapshot, const CameraSensor& camera);

// 世界中的点投影到像素 (u, v)；在相机后方或光轴方向距离不大于 0 时为空。不检查是否在图像范围内。
std::optional<std::array<double, 2>> projectPoint(const CameraIntrinsics& intrinsics, const terra::data::Pose& cameraWorld,
                                                  const terra::data::Vec3& worldPoint);

// 显示旋转（M4-04，只影响显示）：图像逆时针转 rotationDegrees（0、90、180、270）后显示。
// 显示区域中的归一化坐标 (dx, dy)（0～1，原点在左上）→ 原图的归一化坐标 (u, v)（0～1，原点在左上）。
// 显示四个角的纹理坐标、悬停时取深度都用它，保证二者一致。其他角度按 0 处理。
std::array<double, 2> displayToImage(int rotationDegrees, double dx, double dy);
// 旋转后显示的宽高比（宽 / 高）：90、270 时宽高互换
double displayAspect(int rotationDegrees, int width, int height);

}  // namespace terra::scene
