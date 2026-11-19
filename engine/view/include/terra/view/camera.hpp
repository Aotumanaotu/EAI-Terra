#pragma once

// 视图层：透视相机（M1-07，D-023）。
//
// 坐标约定沿用 D-012：右手系、Z 向上、SI 单位、双精度。相机姿态用“位置 + 偏航 + 俯仰”表示，不含滚转：
//   偏航 yaw：绕 +Z，0 时朝向 +X，逆时针（从上往下看）为正；
//   俯仰 pitch：相对水平面，抬头为正，限制在 ±89°，避免与 Z 轴重合时方向不确定。
// 输出矩阵为列主序 float[16]，裁剪空间为 OpenGL 约定（z ∈ [-1, 1]），可直接交给着色器。
// 公共接口只使用 Terra 类型与标准库；GLM 只在实现内部使用（D-020）。

#include <array>

#include "terra/data/geometry.hpp"

namespace terra::view {

using Matrix4 = std::array<float, 16>;  // 列主序

class Camera {
public:
    static constexpr double kMaxPitch = 1.5533430342749532;  // 89°

    terra::data::Vec3 position{-5.0, 0.0, 2.0};
    double verticalFov = 1.0471975511965976;  // 60°，弧度
    double nearPlane = 0.05;                   // m
    double farPlane = 500.0;                   // m

    double yaw() const noexcept { return yaw_; }
    double pitch() const noexcept { return pitch_; }
    // 设置朝向；俯仰被限制在 ±89°，偏航归一化到 (-π, π]。
    void setOrientation(double newYaw, double newPitch) noexcept;
    // 朝向目标点；目标与位置重合时不改变朝向。
    void lookAt(const terra::data::Vec3& target) noexcept;

    terra::data::Vec3 forward() const noexcept;  // 单位向量
    terra::data::Vec3 right() const noexcept;    // 单位向量，水平
    terra::data::Vec3 up() const noexcept;       // 单位向量，与 forward、right 正交

    Matrix4 viewMatrix() const;
    // aspect = 宽 / 高；非正值按 1 处理。
    Matrix4 projectionMatrix(double aspect) const;

private:
    double yaw_ = 0.0;
    double pitch_ = -0.3805063771123649;  // 约 -21.8°：从 (-5, 0, 2) 看向原点
};

}  // namespace terra::view
