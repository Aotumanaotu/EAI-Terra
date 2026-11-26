#pragma once

// 策略的自动配置（M4-10a 的核心部分；M4-10b 的导入向导在此之上）。用户的要求：只需选择策略与对应的机器人。
//
// 输入：模型文件（.onnx）、机器人所在场景的模型描述与场景文件路径、可选的模型实例名（场景中有多个机器人时）。
// 来源（按顺序）：
//   1. ONNX 的元数据——按“格式”交给注册的解析器（本步：supermimic_moe_velocity_policy_v1）；以后的格式注册新的解析器；
//   2. 模型所在目录及上两级目录中的训练配置（本步：task_config.json 的 control.stiffness / damping / effort_limit、
//      commands.initial_ranges、sim.timestep）；
// 自动完成：关节名对应（FL_hip_joint ↔ go2/FL_hip_joint）、默认姿态、动作缩放、PD 增益与力矩上限（按关节组）、控制周期、
// 观测项与历史帧；角速度与重力方向取机器人身上的 IMU（带噪声），没有 IMU 时退回机身的真值并警告。
// 无法确定的项列在 errors 中（M4-10b 的向导让用户填写）。

#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "terra/data/model_description.hpp"
#include "terra/policy/policy.hpp"

namespace terra::policy {

struct AutoconfigResult {
    std::optional<PolicyDescription> policy;
    std::string format;               // 识别出的格式
    std::vector<std::string> errors;  // 无法确定的项
    std::vector<std::string> warnings;
    std::vector<std::string> notes;   // 自动完成了什么（界面列出，供用户核对）
};

struct AutoconfigInput {
    std::string model;     // 模型文件（绝对路径）
    std::string scene;     // 场景文件（写入说明文件的推荐场景）
    std::string instance;  // 模型实例名；为空时自动选择（只有一个匹配的机器人时）
    std::string configDirectory;  // M4-10b：从哪个目录起找训练配置（空为模型所在目录；转换得到的 ONNX 用原文件的目录）
};

AutoconfigResult autoconfigure(const AutoconfigInput& input, const terra::data::ModelDescription& robot);

// ---- 机器人匹配（M4-10b：导入向导按关节名推荐场景；机器人与训练时不同时警告）----
// 策略控制的关节名（不带实例前缀）：ONNX 的元数据（已知格式）或说明文件的动作关节；读不出时为空
std::vector<std::string> policyJointNames(const std::string& path);

struct RobotMatch {
    std::string instance;              // 匹配的模型实例（没有前缀时为空）
    std::size_t matched = 0;           // 找到的策略关节数（取最多的实例）
    std::size_t wanted = 0;            // 策略关节数
    std::vector<std::string> extra;    // 该实例中不由策略控制的转动 / 滑动关节（全名）：多出时机器人可能与训练时不同
    std::size_t instances = 0;         // 包含全部策略关节的实例数（> 1 时需要指定）
    bool complete() const noexcept { return wanted > 0 && matched == wanted; }
};
RobotMatch matchRobotJoints(const std::vector<std::string>& names, const terra::data::ModelDescription& robot);

// 格式解析器（与导出器相同的注册方式）：按 ONNX 元数据识别并填写说明文件
struct FormatContext {
    const OnnxInfo& onnx;
    const terra::data::ModelDescription& robot;
    const AutoconfigInput& input;
    std::string trainingConfig;  // 找到的 task_config.json 的路径（没有为空）
    std::string trainingConfigText;
};
struct PolicyFormat {
    std::string id;
    std::function<bool(const OnnxInfo&)> matches;
    std::function<void(const FormatContext&, AutoconfigResult&)> configure;
};
void registerPolicyFormat(PolicyFormat format);
std::vector<std::string> policyFormats();

}  // namespace terra::policy
