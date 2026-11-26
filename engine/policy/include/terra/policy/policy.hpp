#pragma once

// 策略的说明文件（M4-10a，设计稿第 3 节）：*.terra-policy.json。说明模型文件、推理后端、控制周期、观测怎么拼、动作怎么用。
// 一般由自动配置（autoconfig.hpp）生成，用户不需要手写。
//
//   {"format": "terra.policy", "version": 1, "name": "…", "scene": "<推荐场景，相对本文件>",
//    "model": {"file": "policy.onnx", "backend": "onnxruntime-cpu", "input": "obs_history", "output": "actions", "sha256": "…"},
//    "controlPeriod": 0.02,
//    "history": {"length": 5, "order": "oldestFirst", "reset": "repeatInitial"},
//    "command": {"default": [0, 0, 0], "range": [[-1, 1], [-1, 1], [-2, 2]]},
//    "observation": [{"term": "baseAngularVelocity", "sensor": "go2/imu", "scale": 1}, …],
//    "action": {"type": "positionPd", "joints": […], "scale": […], "offset": […], "kp": […], "kd": […], "torqueLimit": […]},
//    "source": {…}}
// 不认识的顶层字段给出警告并原样保留（以后的版本可以增加字段，旧版本打开、保存不丢内容）。
//
// 观测项（每帧按列表顺序拼接；有历史时每次推理的输入为 历史长度 × 每帧宽度，最旧的在前）：
//   baseAngularVelocity  机身角速度（机身坐标系，3）：sensor（IMU 的陀螺，带 M4-09 噪声）或 entity（真值）
//   projectedGravity     重力方向在机身坐标系中的单位向量（3）：sensor（IMU 的姿态）或 entity
//   baseLinearVelocity   机身线速度（机身坐标系，3）：entity（真值）
//   jointPositions       关节位置减去 offset（每个关节 1）
//   jointVelocities      关节速度（每个关节 1）
//   lastAction           上一次推理的输出（缩放前；动作的维数）
//   command              速度指令 vx、vy（机身）与偏航角速度（3）
//   phase                步态相位 sin、cos（2）：period（s）；指令幅值（|vx, vy| + |偏航|）不大于 standingThreshold 时为 0
// 每项可写 scale（一个数或逐项）、clip（绝对值上限，0 为不截断）。以后增加新项不改已有项的含义。
//
// 动作类型：positionPd（目标位置 = offset + scale × 输出，按 kp、kd 每步算力矩并截断到 torqueLimit，送力矩执行器）、
// position、velocity、torque（直接送对应的执行器）。

#include <array>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace terra::policy {

inline constexpr const char* kPolicySuffix = ".terra-policy.json";

enum class TermKind { BaseAngularVelocity, ProjectedGravity, BaseLinearVelocity, JointPositions, JointVelocities, LastAction, Command, Phase };
const char* termName(TermKind kind);
std::optional<TermKind> termFromName(const std::string& name);

struct ObservationTerm {
    TermKind kind = TermKind::BaseAngularVelocity;
    std::string sensor;               // IMU 全名（带噪声的读数）
    std::string entity;               // 实体全名（真值）
    std::vector<std::string> joints;  // jointPositions、jointVelocities
    std::vector<double> scale;        // 空为 1；一个数为全部；否则逐项
    std::vector<double> offset;       // jointPositions：减去的偏置（空为 0）
    double clip = 0.0;
    double period = 0.0;              // phase
    double standingThreshold = 0.0;   // phase
};

enum class ActionType { PositionPd, Position, Velocity, Torque };
const char* actionTypeName(ActionType type);

struct ActionSpec {
    ActionType type = ActionType::PositionPd;
    std::vector<std::string> joints;  // 动作的顺序（全名）；执行器按关节查找
    std::vector<double> scale, offset, kp, kd, torqueLimit;
    double clip = 0.0;  // 输出的绝对值上限（0 为不截断）
};

struct PolicyDescription {
    std::string path;  // 说明文件（绝对路径）
    std::string name;
    std::string scene;      // 推荐场景（绝对路径）
    std::string model;      // 模型文件（绝对路径）
    std::string backend = "onnxruntime-cpu";
    std::string inputName, outputName;
    std::string modelSha256;
    double controlPeriod = 0.02;
    int historyLength = 1;
    std::array<double, 3> defaultCommand{0.0, 0.0, 0.0};
    std::array<std::array<double, 2>, 3> commandRange{{{-1.0, 1.0}, {-1.0, 1.0}, {-2.0, 2.0}}};
    std::vector<ObservationTerm> observation;
    ActionSpec action;
    std::map<std::string, std::string> source;  // 来源说明（格式、训练配置文件、spec_hash 等）
    std::string extra;  // 不认识的顶层字段（更新的版本写的），JSON 对象文本，保存时原样写回

    int termWidth(const ObservationTerm& term) const;
    int frameWidth() const;
    int inputWidth() const { return frameWidth() * historyLength; }
    int actionWidth() const { return static_cast<int>(action.joints.size()); }
};

struct PolicyFileResult {
    std::optional<PolicyDescription> policy;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;
};

PolicyFileResult parsePolicy(const std::string& text, const std::string& path);
PolicyFileResult readPolicyFile(const std::string& path);
std::string policyToJson(const PolicyDescription& policy, const std::string& path);  // 相对路径按 path 所在目录写
std::optional<std::string> savePolicy(const PolicyDescription& policy, const std::string& path);

// ---- ONNX 文件的信息（不需要推理库：直接读 protobuf）----
struct TensorShape {
    std::string name;
    std::vector<long long> dims;  // 符号维（如 batch）为 -1
};
struct OnnxInfo {
    std::string producer;
    long long opset = 0;
    std::vector<TensorShape> inputs, outputs;
    std::map<std::string, std::string> metadata;  // metadata_props
    std::map<std::string, int> operators;         // 算子 → 出现次数
    long long parameters = 0;                     // 初始化张量的元素总数
};
std::optional<OnnxInfo> readOnnxInfo(const std::string& path, std::string& error);

}  // namespace terra::policy
