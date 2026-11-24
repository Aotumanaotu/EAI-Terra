#pragma once

// 世界组合请求（M2-04，M2-01 设计第 2、3 节）。只用 Terra 类型。
//
// 场景模块（terra_scene）决定：有哪些模型实例、各自的名字前缀与放置位姿、全局物理配置的取值，以及错误归属到哪个实例。
// 物理后端只做机械的组合：逐个解析 MJCF，按请求的前缀与位姿组合成一个世界并编译，把每一步的问题按实例返回。
//
// 组合规则（后端保证）：
//   - 每个实例中所有有名字的元素都加上 prefix；
//   - 全局物理配置中，PhysicsOptions 列出的 6 项取 resolveOptions 的返回值；其余全局设置（接触锥、阻抗比、
//     内存大小等）一律取第一个实例的值，其他实例与之不同时给出警告；
//   - 单个实例、单位放置位姿时，世界的动力学与直接加载该 MJCF 相同。

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "terra/data/geometry.hpp"
#include "terra/physics/physics_world.hpp"

namespace terra::physics {

// 全局物理配置中由场景决定的 6 项。integrator：Euler、RK4、implicit、implicitfast；solver：PGS、CG、Newton。
struct PhysicsOptions {
    double timestep = 0.0;
    terra::data::Vec3 gravity;
    std::string integrator;
    std::string solver;
    int iterations = 0;
    double tolerance = 0.0;
};

struct ModelPlacement {
    std::string instanceName;  // 写入 ModelDescription::modelInstances
    std::string prefix;        // 加在该实例所有元素名字前，如 "cart_a/"
    std::string mjcfPath;      // 绝对路径或相对当前目录
    terra::data::Pose placement;
    // M3-04：删掉该实例根部 body 的自由关节（悬挂模式）。没有自由关节时给出警告。
    bool fixedBase = false;
};

struct WorldComposition {
    std::string sourceLabel;  // 写入 ModelDescription::sourcePath，如场景文件路径
    std::vector<ModelPlacement> models;
    // 由场景模块决定 6 项全局配置：参数为各实例 MJCF 中的值（与 models 顺序相同），返回最终值。
    // 为空时取第一个实例的值。
    std::function<PhysicsOptions(const std::vector<PhysicsOptions>&)> resolveOptions;
};

struct ComposeIssue {
    enum class Severity { Warning, Error };
    Severity severity = Severity::Error;
    int model = -1;  // models 下标；-1 表示不能确定是哪个实例（如合并后的编译错误，由场景模块再按前缀归属）
    LoadErrorKind kind = LoadErrorKind::InvalidModel;  // 仅错误有意义
    std::string message;
};

// 成功时 world 非空（可能仍有警告）；失败时 world 为空，issues 中至少有一个错误。
struct ComposeResult {
    std::unique_ptr<PhysicsWorld> world;
    std::vector<ComposeIssue> issues;
};

}  // namespace terra::physics
