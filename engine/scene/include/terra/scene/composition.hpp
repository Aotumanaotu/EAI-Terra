#pragma once

// 模型实例组合（M2-04，M2-01 设计第 2、3 节；用户 2026-11-23：由 terra_scene 统一负责创建世界、管理实例前缀、
// 处理全局配置和编译错误）。
//
// 职责划分：本模块决定实例名是否合法、前缀（"实例名/"）、全局物理配置的取值规则，以及每个问题归属到哪个实例；
// 物理后端（通过注入的 WorldFactory，如 terra::physics::mujoco::composeWorld）只做机械的组合与编译。
// 本模块不含 MuJoCo 类型，也不链接任何物理后端。场景文件（JSON）的读取在 M2-05 加入，这里的输入是已解析的实例列表。

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "terra/data/geometry.hpp"
#include "terra/physics/physics_world.hpp"
#include "terra/physics/world_composition.hpp"

namespace terra::scene {

struct SceneModel {
    std::string name;   // 实例名
    std::string mjcf;   // MJCF 路径
    terra::data::Pose placement;
    std::string where;  // 诊断中的位置（M2-05 起为场景文件中的 JSON 指针，如 "/models/1"）
    bool fixedBase = false;  // M3-04：悬挂模式，组合时删掉该实例根部的自由关节，基座固定在放置位姿上
    bool torqueActuators = false;  // M4-06b：位置执行器改为力矩执行器（宇树低层控制）
    // M5-02：场景元素——MJCF 由场景模块生成（mjcf 为元素文件路径，只用于诊断）；全局物理设置随其他实例，不参与比较
    std::string mjcfText{};  // 带默认初始化：旧的聚合初始化可以省略本字段（不触发 -Wmissing-field-initializers）
    bool element = false;
};

// 场景文件中写了的全局物理配置；没写的项取第一个实例的值（M2-01 设计 3.4 节）。
struct PhysicsOverrides {
    std::optional<double> timestep;
    std::optional<terra::data::Vec3> gravity;
    std::optional<std::string> integrator;
    std::optional<std::string> solver;
    std::optional<int> iterations;
    std::optional<double> tolerance;
};

struct SceneIssue {
    enum class Severity { Warning, Error };
    Severity severity = Severity::Error;
    std::string instance;  // 归属的实例名；空表示整个场景
    std::string where;     // 诊断位置（实例的 where）
    std::string message;
};

using WorldFactory = std::function<terra::physics::ComposeResult(const terra::physics::WorldComposition&)>;

// 实例名：[A-Za-z_][A-Za-z0-9_-]*
bool isValidInstanceName(std::string_view name);

// 实例前缀："实例名/"
std::string instancePrefix(const std::string& name);

// 决定 6 项全局配置：场景写了的以场景为准；没写的取第一个实例的值，其他实例与之不同时追加警告（场景写了的项不警告）。
terra::physics::PhysicsOptions resolvePhysicsOptions(const PhysicsOverrides& overrides,
                                                     const std::vector<terra::physics::PhysicsOptions>& models,
                                                     const std::vector<SceneModel>& instances,
                                                     std::vector<SceneIssue>& warnings);

// 合并后的编译错误不能确定实例时（后端给出 model = -1），按错误文字中最先出现的实例前缀归属；找不到时返回 -1。
int attributeByPrefix(const std::string& message, const std::vector<SceneModel>& instances);

struct ComposedWorld {
    std::unique_ptr<terra::physics::PhysicsWorld> world;
    std::vector<SceneIssue> issues;  // 错误与警告
    bool ok() const noexcept { return world != nullptr; }
};

// 校验实例，生成组合请求并调用 factory。校验失败时不调用 factory。label 写入模型描述的 sourcePath。
// imus（M4-02）：按实例下标给出场景文件添加的 IMU（名字不带前缀）；为空或较短时其余实例没有。
ComposedWorld composeWorld(const std::vector<SceneModel>& instances, const PhysicsOverrides& overrides,
                           const WorldFactory& factory, const std::string& label,
                           const std::vector<std::vector<terra::physics::ImuMount>>& imus = {});

}  // namespace terra::scene
