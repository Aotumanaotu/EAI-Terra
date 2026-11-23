#pragma once

// MuJoCo 后端的物理世界（M0-04 加载与初始化；M0-05 固定步进与 reset；M0-06 状态快照；M0-07 控制）。
// 私有头文件：外部通过 terra/physics/mujoco/mujoco_backend.hpp 的 loadWorld 取得 PhysicsWorld。
// 语义见 terra/physics/physics_world.hpp。

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "terra/physics/physics_world.hpp"
#include "terra/physics/world_composition.hpp"

namespace terra::physics::mujoco {

// 由已编译模型建立世界所需的描述信息（M2-04）：单模型加载时一个无前缀的实例；组合时每个实例一个前缀。
struct BuildInput {
    std::string sourcePath;
    std::vector<terra::data::ModelInstanceDescription> instances;
    std::vector<std::string> prefixes;  // 与 instances 顺序相同
};

class MujocoWorld final : public PhysicsWorld {
public:
    static LoadResult load(const terra::data::ModelSource& source);
    static ComposeResult compose(const WorldComposition& composition);

    ~MujocoWorld() override;

    const terra::data::ModelDescription& description() const noexcept override;
    std::optional<SimulationError> step() override;
    void reset() override;
    std::optional<ControlError> setControl(const std::vector<ActuatorCommand>& commands) override;
    double simulationTime() const noexcept override;
    std::uint64_t stepIndex() const noexcept override;
    std::uint64_t runEpoch() const noexcept override;
    bool faulted() const noexcept override;
    std::shared_ptr<const terra::data::PhysicsSnapshot> snapshot() const override;

private:
    struct Impl;
    explicit MujocoWorld(std::unique_ptr<Impl> impl);
    // 接管 model：校验、分配运行数据、建立描述；result 中已有的警告保留。失败时释放 model。
    static LoadResult build(mjModel* model, const BuildInput& input, LoadResult result);

    std::unique_ptr<Impl> impl_;
};

}  // namespace terra::physics::mujoco
