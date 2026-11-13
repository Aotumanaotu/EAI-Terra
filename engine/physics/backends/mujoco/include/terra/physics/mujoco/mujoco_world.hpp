#pragma once

// MuJoCo 后端的物理世界（M0-04 加载与初始化；M0-05 固定步进与 reset）。
//
// 一个 MujocoWorld 对应一个已编译的 MJCF 模型及其运行数据。公共接口只使用 Terra 类型；
// MuJoCo 的模型、数据和对象索引都封装在实现内部。
//
// 观测一致性：初始化、每次 step 和 reset 之后，都会刷新派生量（位姿等），
// 因此任意时刻读到的时间、步序号与位姿都对应同一个仿真时刻。
// 线程：M0 单线程使用，不支持并发调用。

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "terra/data/geometry.hpp"
#include "terra/data/ids.hpp"
#include "terra/data/model_description.hpp"

namespace terra::physics::mujoco {

enum class LoadErrorKind {
    FileUnavailable,  // 文件不存在、不是常规文件或不可读
    InvalidModel,     // MJCF 解析或编译失败（含引用的资源缺失），或初始状态无效
    Internal,         // 运行数据分配等内部失败
};

struct LoadError {
    LoadErrorKind kind;
    std::string message;
};

class MujocoWorld;

enum class SimulationErrorKind {
    NonFinite,      // 状态中出现 NaN/Inf，或超出 MuJoCo 的有效范围（|x| > 1e10）
    AutoReset,      // MuJoCo 检测到坏值并自行重置了状态（意外重置）
    SolverWarning,  // MuJoCo 报告严重诊断：接触/约束缓冲区溢出、惯量奇异、控制量非法等
    Faulted,        // 世界已处于故障状态，需要 reset 后才能继续
};

struct SimulationError {
    SimulationErrorKind kind;
    std::uint64_t step;  // 出错的步序号（尝试推进到的那一步）
    std::string message;
};

// 加载结果：成功时 world 非空、error 为空；失败时 world 为空，不会留下半初始化的世界。
struct LoadResult {
    std::unique_ptr<MujocoWorld> world;
    std::optional<LoadError> error;
    std::vector<std::string> warnings;  // 编译成功但 MuJoCo 给出的警告
};

class MujocoWorld {
public:
    // 解析并编译 MJCF，分配运行数据，计算初始派生状态（位姿等）。
    static LoadResult load(const terra::data::ModelSource& source);

    ~MujocoWorld();
    MujocoWorld(const MujocoWorld&) = delete;
    MujocoWorld& operator=(const MujocoWorld&) = delete;
    MujocoWorld(MujocoWorld&&) = delete;
    MujocoWorld& operator=(MujocoWorld&&) = delete;

    const terra::data::ModelDescription& description() const noexcept;

    // 推进一个固定步长。成功返回空；失败返回错误，世界进入故障状态，状态保持出错时的值，
    // 之后的 step 均返回 Faulted，直到 reset。
    std::optional<SimulationError> step();

    // 回到模型初始状态：时间与步序号归零、控制量清零、清除故障、刷新初始观测，运行代次加 1。
    // 拓扑与 ID 保持不变。
    void reset();

    // 当前仿真时间（秒）。
    double simulationTime() const noexcept;

    // 本运行代次内已完成的步数；reset 后归零。
    std::uint64_t stepIndex() const noexcept;

    // 运行代次：加载后为 0，每次 reset 加 1。用于区分 reset 前后的数据。
    std::uint64_t runEpoch() const noexcept;

    bool faulted() const noexcept;

    // 实体原点的世界位姿；未知 ID 返回空。
    // M0-04 用于核对初始状态；M0-06 起由不可变状态快照统一提供。
    std::optional<terra::data::Pose> entityPose(terra::data::EntityId id) const;

private:
    struct Impl;
    explicit MujocoWorld(std::unique_ptr<Impl> impl);

    std::unique_ptr<Impl> impl_;
};

}  // namespace terra::physics::mujoco
