#pragma once

// 位姿运算（M2-05）：双精度，四元数 wxyz、表示局部 → 父坐标的旋转（D-012）。只供场景计算使用，不是通用数学库。

#include <cmath>

#include "terra/data/geometry.hpp"

namespace terra::scene {

inline terra::data::Quaternion multiply(const terra::data::Quaternion& a, const terra::data::Quaternion& b) {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

inline terra::data::Quaternion conjugate(const terra::data::Quaternion& q) { return {q.w, -q.x, -q.y, -q.z}; }

inline double norm(const terra::data::Quaternion& q) { return std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z); }

inline terra::data::Quaternion normalized(const terra::data::Quaternion& q) {
    const double n = norm(q);
    return n > 0.0 ? terra::data::Quaternion{q.w / n, q.x / n, q.y / n, q.z / n} : terra::data::Quaternion{};
}

// v' = q v q*（q 为单位四元数）
inline terra::data::Vec3 rotate(const terra::data::Quaternion& q, const terra::data::Vec3& v) {
    const double tx = 2 * (q.y * v.z - q.z * v.y);
    const double ty = 2 * (q.z * v.x - q.x * v.z);
    const double tz = 2 * (q.x * v.y - q.y * v.x);
    return {v.x + q.w * tx + (q.y * tz - q.z * ty), v.y + q.w * ty + (q.z * tx - q.x * tz),
            v.z + q.w * tz + (q.x * ty - q.y * tx)};
}

// a × b：先 b 后 a（b 在 a 的坐标系中表达）
inline terra::data::Pose multiply(const terra::data::Pose& a, const terra::data::Pose& b) {
    const terra::data::Vec3 r = rotate(a.orientation, b.position);
    return {{a.position.x + r.x, a.position.y + r.y, a.position.z + r.z}, multiply(a.orientation, b.orientation)};
}

inline terra::data::Pose inverse(const terra::data::Pose& p) {
    const terra::data::Quaternion qi = conjugate(p.orientation);
    const terra::data::Vec3 t = rotate(qi, p.position);
    return {{-t.x, -t.y, -t.z}, qi};
}

// 滚转、俯仰、偏航（度）：依次绕世界 X、Y、Z 轴旋转，R = Rz·Ry·Rx（M2-01 设计 3.2 节）
inline terra::data::Quaternion fromRpyDegrees(double roll, double pitch, double yaw) {
    constexpr double kHalfDegree = 3.14159265358979323846 / 360.0;
    const terra::data::Quaternion qx{std::cos(roll * kHalfDegree), std::sin(roll * kHalfDegree), 0.0, 0.0};
    const terra::data::Quaternion qy{std::cos(pitch * kHalfDegree), 0.0, std::sin(pitch * kHalfDegree), 0.0};
    const terra::data::Quaternion qz{std::cos(yaw * kHalfDegree), 0.0, 0.0, std::sin(yaw * kHalfDegree)};
    return multiply(qz, multiply(qy, qx));
}

}  // namespace terra::scene
