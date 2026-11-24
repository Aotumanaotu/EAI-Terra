#pragma once

// Terra Physics 后端无关接口（M0-07）。
//
// 一个 PhysicsWorld 对应一个已加载并初始化的模型及其运行数据。本头文件只使用 Terra 类型；
// 具体后端通过各自的 loadWorld 入口创建世界（如 terra/physics/mujoco/mujoco_backend.hpp）。
// 抽象接口只是把唯一的实现放在边界之后，不表示已承诺多后端语义一致（D-107）。
//
// 生命周期（M0-07 报告 3.1 节）：
//   load 成功 → 就绪；step 出错 → 故障；reset 从两种状态回到就绪。
//   故障状态下 step 与 setControl 返回 Faulted；snapshot 与各查询仍可用。
// 观测一致性：加载、每次 step 和 reset 之后都会刷新派生量，任意时刻读到的时间、步序号与状态对应同一仿真时刻。
// 线程：M0 单线程使用，不支持并发调用。

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "terra/data/ids.hpp"
#include "terra/data/model_description.hpp"
#include "terra/data/state.hpp"

namespace terra::physics {

enum class LoadErrorKind {
    FileUnavailable,  // 文件不存在、不是常规文件或不可读
    InvalidModel,     // 模型解析或编译失败（含引用的资源缺失），或初始状态无效
    Unsupported,      // 模型合法，但用到了 M0 不支持的特性（如带控制延迟的执行器）
    Internal,         // 运行数据分配等内部失败
};

struct LoadError {
    LoadErrorKind kind;
    std::string message;
};

enum class SimulationErrorKind {
    NonFinite,      // 状态中出现 NaN/Inf，或超出后端的有效数值范围
    AutoReset,      // 后端检测到坏值并自行重置了状态（意外重置）
    SolverWarning,  // 后端报告严重诊断：接触/约束缓冲区溢出、惯量奇异、控制量非法等
    Faulted,        // 世界已处于故障状态，需要 reset 后才能继续
};

struct SimulationError {
    SimulationErrorKind kind;
    std::uint64_t step;  // 出错的步序号（尝试推进到的那一步）
    std::string message;
};

// 一个执行器的控制命令。values 的长度必须等于该执行器的输入数（ActuatorDescription::inputs）。
struct ActuatorCommand {
    terra::data::ActuatorId actuator;
    std::vector<double> values;
};

enum class ControlErrorKind {
    Faulted,            // 世界处于故障状态
    UnknownActuator,    // 执行器 ID 无效或不存在
    DuplicateActuator,  // 同一次调用中重复出现同一执行器
    SizeMismatch,       // values 长度与执行器输入数不符
    NonFinite,          // 含 NaN/Inf
    OutOfRange,         // 超出输入的 [min, max]，或超出后端的有效数值范围
};

struct ControlError {
    ControlErrorKind kind;
    terra::data::ActuatorId actuator;  // 出错的执行器；Faulted 时为无效 ID
    int input = -1;                    // 出错的输入下标；不适用时为 -1
    std::string message;
};

// 射线检测（M4-03b，激光雷达用）。在当前观测时刻的几何上求每条射线到第一个命中面的距离。
//   origin、directions 为世界坐标，directions 应为单位向量；距离不超过 maxRange 才算命中；
//   exclude：忽略该实体自身的几何体（雷达装在机器人上时不打到安装它的实体）；
//   hits：Collision 只打参与碰撞的几何体（与物理一致），Visual 只打默认显示的几何体（MJCF 第 0～2 组）。
//   完全透明（alpha = 0）的几何体不会被打到（MuJoCo 的规则）。
enum class RayTargets { Collision, Visual };

struct RayQuery {
    terra::data::Vec3 origin;
    std::vector<terra::data::Vec3> directions;
    double maxRange = 0.0;
    terra::data::EntityId exclude;
    RayTargets hits = RayTargets::Collision;
};

class PhysicsWorld {
public:
    virtual ~PhysicsWorld() = default;
    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    virtual const terra::data::ModelDescription& description() const noexcept = 0;

    // 推进一个固定步长。成功返回空；失败返回错误，世界进入故障状态，状态保持出错时的值，
    // 之后的 step 均返回 Faulted，直到 reset。
    virtual std::optional<SimulationError> step() = 0;

    // 回到模型初始状态：时间与步序号归零、控制恢复为模型定义的中性值、清除故障、刷新初始观测，
    // 运行代次加 1。拓扑与 ID 保持不变。
    virtual void reset() = 0;

    // 按关键帧重置（M3-02）：与 reset 相同（时间与步序号归零、清除故障、运行代次加 1），但位置、速度与控制取自
    // ModelDescription::keyframes[index]。下标越界时不做任何事并返回 false。
    virtual bool resetToKeyframe(std::size_t index) = 0;

    // 设置执行器控制（M0-07 报告 3.4 节）：
    //   - 按执行器寻址，部分更新：未出现的执行器保持原值；空列表合法，不做任何改变。
    //   - 从下一次 step 起生效并一直保持，直到再次设置或 reset。不影响当前观测时刻与已缓存的快照。
    //   - 原子：全部校验通过才写入；任一项失败则一项也不写，返回第一个错误。
    //   - 越界一律拒绝，不裁剪。
    virtual std::optional<ControlError> setControl(const std::vector<ActuatorCommand>& commands) = 0;

    // 当前仿真时间（秒）。
    virtual double simulationTime() const noexcept = 0;

    // 本运行代次内已完成的步数；reset 后归零。
    virtual std::uint64_t stepIndex() const noexcept = 0;

    // 运行代次：加载后为 0，每次 reset 加 1。用于区分 reset 前后的数据。
    virtual std::uint64_t runEpoch() const noexcept = 0;

    virtual bool faulted() const noexcept = 0;

    // 当前观测时刻的不可变快照（实体位姿与速度、关节状态、接触摘要、诊断）。
    // 同一观测时刻多次调用返回同一对象；step/reset 之后返回新对象，已取得的旧快照不受影响。
    // 只读取已计算好的量，不推进或重算物理。
    virtual std::shared_ptr<const terra::data::PhysicsSnapshot> snapshot() const = 0;

    // 射线检测（M4-03b）：返回与 directions 一一对应的距离（m）；maxRange 内没有命中为 +inf。只读取当前状态，不推进物理。
    virtual std::vector<double> castRays(const RayQuery& query) const = 0;

protected:
    PhysicsWorld() = default;
};

// 加载结果：成功时 world 非空、error 为空；失败时 world 为空，不会留下半初始化的世界。
struct LoadResult {
    std::unique_ptr<PhysicsWorld> world;
    std::optional<LoadError> error;
    std::vector<std::string> warnings;  // 加载成功但后端给出的警告
};

}  // namespace terra::physics
