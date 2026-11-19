#pragma once

// MuJoCo 后端的物理世界（M0-04 加载与初始化；M0-05 固定步进与 reset；M0-06 状态快照；M0-07 控制）。
// 私有头文件：外部通过 terra/physics/mujoco/mujoco_backend.hpp 的 loadWorld 取得 PhysicsWorld。
// 语义见 terra/physics/physics_world.hpp。

#include <memory>
#include <optional>
#include <vector>

#include "terra/physics/physics_world.hpp"

namespace terra::physics::mujoco {

class MujocoWorld final : public PhysicsWorld {
public:
    static LoadResult load(const terra::data::ModelSource& source);

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

    std::unique_ptr<Impl> impl_;
};

}  // namespace terra::physics::mujoco
