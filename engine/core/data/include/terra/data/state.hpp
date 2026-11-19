#pragma once

// Terra 物理状态与不可变快照（M0-06）。
//
// 约定（D-012，M0-06 验证）：
//   - SI 单位：米、秒、弧度、牛顿；双精度。
//   - 世界坐标：右手系，Z 向上。
//   - 四元数顺序 w, x, y, z，表示局部坐标到父坐标（实体为世界）的旋转。
//   - 实体速度：实体原点（body 坐标系原点，不是质心）的线速度与角速度，在世界坐标中表达。
//
// 快照是独立的数据副本：不含后端指针，发布后不再改变；持有者可以在之后的 step/reset 后继续读取。
// 它是观测数据，不是可用于精确恢复仿真的检查点。

#include <array>
#include <cstdint>
#include <vector>

#include "terra/data/geometry.hpp"
#include "terra/data/ids.hpp"
#include "terra/data/model_description.hpp"

namespace terra::data {

struct EntityState {
    EntityId id;
    Pose pose;
    Vec3 linearVelocity;   // m/s，实体原点，世界坐标
    Vec3 angularVelocity;  // rad/s，世界坐标
};

// 关节状态。position/velocity 只有前 positionSize/velocitySize 个元素有效，含义按类型：
//   Hinge：position[0] 角度 (rad)；velocity[0] 角速度 (rad/s)
//   Slide：position[0] 位移 (m)；velocity[0] 速度 (m/s)
//   Ball： position[0..3] 相对父坐标的姿态四元数 wxyz；velocity[0..2] 角速度 (rad/s)，子 body 坐标
//   Free： position[0..2] 位置 (m)，世界坐标；position[3..6] 姿态四元数 wxyz，世界坐标；
//          velocity[0..2] 线速度 (m/s)，世界坐标；velocity[3..5] 角速度 (rad/s)，body 局部坐标
// 注意：自由关节的角速度在 body 局部坐标中，与 EntityState::angularVelocity（世界坐标）不同。
struct JointState {
    JointId id;
    JointType type = JointType::Hinge;
    int positionSize = 0;
    int velocitySize = 0;
    std::array<double, 7> position{};
    std::array<double, 6> velocity{};
};

// 一对实体之间的有效接触汇总。entity 为无效 ID（0）表示世界上的静态几何（如地面）。
// 有效接触：参与约束求解且法向力 > 0 的接触点；仅几何上检测到、但不受力的候选接触不计入。
struct ContactPair {
    EntityId entityA;
    EntityId entityB;
    std::uint32_t pointCount = 0;
    double normalForce = 0.0;  // N，各接触点法向力之和（>= 0），按当前观测时刻的状态计算
};

struct ContactSummary {
    std::uint32_t candidateCount = 0;  // 碰撞检测得到的候选接触点数（含不受力的）
    std::uint32_t activeCount = 0;     // 有效接触点数
    std::vector<ContactPair> pairs;    // 按 (entityA, entityB) 排序，entityA <= entityB
};

struct Diagnostics {
    bool finite = true;          // 位姿、速度、关节状态均为有限值
    int solverIterations = 0;    // 本观测时刻约束求解的最大迭代次数（各约束岛取最大）
};

struct PhysicsSnapshot {
    static constexpr std::uint32_t kContractRevision = 1;  // 快照格式修订号

    std::uint64_t worldInstance = 0;  // 与 ModelDescription::worldInstance 相同，区分不同的加载
    std::uint64_t runEpoch = 0;
    std::uint64_t stepIndex = 0;
    double simulationTime = 0.0;  // s
    double fixedTimestep = 0.0;   // s

    std::vector<EntityState> entities;  // 与 ModelDescription::entities 顺序相同
    std::vector<JointState> joints;     // 与 ModelDescription::joints 顺序相同
    ContactSummary contacts;
    Diagnostics diagnostics;
};

}  // namespace terra::data
