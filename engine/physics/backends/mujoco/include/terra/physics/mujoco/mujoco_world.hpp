#pragma once

// MuJoCo 后端的物理世界（M0-04：加载与初始化）。
//
// 一个 MujocoWorld 对应一个已编译的 MJCF 模型及其运行数据。公共接口只使用 Terra 类型；
// MuJoCo 的模型、数据和对象索引都封装在实现内部。

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

    // 当前仿真时间（秒）。
    double simulationTime() const noexcept;

    // 实体原点的世界位姿；未知 ID 返回空。
    // M0-04 用于核对初始状态；M0-06 起由不可变状态快照统一提供。
    std::optional<terra::data::Pose> entityPose(terra::data::EntityId id) const;

private:
    struct Impl;
    explicit MujocoWorld(std::unique_ptr<Impl> impl);

    std::unique_ptr<Impl> impl_;
};

}  // namespace terra::physics::mujoco
