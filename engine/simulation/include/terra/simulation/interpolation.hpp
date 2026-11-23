#pragma once

// 快照插值（M2-06，M2-01 设计第 7 节；技术设计 4.1 节）。插值只用于显示，不写回物理。
//   - 位置线性插值；姿态取最短弧（两四元数点积为负时取反）后线性插值再归一化（nlerp）。
//   - 两份快照属于不同世界实例或不同运行代次（reset）时不插值，直接使用较新的一份——reset 后的第一帧不会插值到旧世界。
//   - 实体数不同（不应发生，防御性处理）时同样不插值。

#include <vector>

#include "terra/data/geometry.hpp"
#include "terra/data/state.hpp"

namespace terra::simulation {

// 能否在两份快照之间插值（同一世界实例、同一运行代次、实体数相同）。
bool interpolatable(const terra::data::PhysicsSnapshot& older, const terra::data::PhysicsSnapshot& newer) noexcept;

terra::data::Pose interpolate(const terra::data::Pose& a, const terra::data::Pose& b, double alpha) noexcept;

// 实体位姿，按 EntityId 下标（value − 1）。alpha = 0 为 older，1 为 newer；不能插值时返回 newer 的位姿。
std::vector<terra::data::Pose> interpolatePoses(const terra::data::PhysicsSnapshot& older,
                                                const terra::data::PhysicsSnapshot& newer, double alpha);

}  // namespace terra::simulation
