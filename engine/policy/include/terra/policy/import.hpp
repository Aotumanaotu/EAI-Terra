#pragma once

// 策略的导入（M4-10b，设计稿第 4 节）。用户只做两件事：选策略文件，选机器人（场景）。
//
// 支持的文件：
//   .onnx                 直接使用；按 ONNX 元数据与训练配置自动配置（autoconfig.hpp）。
//   .pt（TorchScript）    用 Python 与 PyTorch 转为 ONNX（scripts/policy/torchscript_to_onnx.py；Python 由调用方给出，
//                         默认 python3，可用环境变量 TERRA_PYTHON 指定）。TorchScript 没有观测布局等元数据，
//                         自动配置通常无法完成：生成一份说明文件草稿（*.terra-policy-draft.json）列出要填写的项。
//   .pt（训练检查点）     只含参数，不能单独运行：在同目录与上两级目录中找 spec_hash 与迭代次数都相同的 ONNX。
//                         2026-11-25 核查：用户的 model_10000.pt 与 model_15000.pt 的 spec_hash 相同（同一网络结构），
//                         只有迭代次数不同，所以只比 spec_hash 会把 10000 的检查点错配到 15000 的 ONNX。
// 导入结果放在 <目标根目录>/<名字>/：模型的副本、训练配置的副本与说明文件 <名字>.terra-policy.json
// （资源库扫描库目录下的 policies/，登记为“策略”类资产）。原文件不改动。

#include <optional>
#include <string>
#include <vector>

#include "terra/data/model_description.hpp"
#include "terra/policy/autoconfig.hpp"

namespace terra::policy {

inline constexpr const char* kPolicyDraftSuffix = ".terra-policy-draft.json";

enum class ModelFileKind { Onnx, TorchScript, Checkpoint, PolicyFile, Unknown };
const char* modelFileKindName(ModelFileKind kind);  // "onnx"、"torchscript"、"checkpoint"、"policy"、"unknown"

struct ModelFileInfo {
    ModelFileKind kind = ModelFileKind::Unknown;
    std::string format;                 // ONNX 元数据或检查点中的 format
    std::string specHash;               // 网络结构的摘要（ONNX 元数据、检查点）
    std::optional<long long> iteration;  // 训练迭代次数（ONNX 元数据、检查点）
    std::string error;
};
ModelFileInfo inspectModelFile(const std::string& path);

struct CheckpointLink {
    std::string onnx;                  // 找到的 ONNX（空为没有）
    std::vector<std::string> notes;    // 找到了什么、依据
    std::string error;                 // 没有找到时的原因与导出方法
};
CheckpointLink findOnnxForCheckpoint(const std::string& checkpoint);

struct ConvertResult {
    bool ok = false;
    bool missingTorch = false;  // Python 中没有 PyTorch（或没有 Python）
    std::string log;            // 转换脚本的输出
    std::string error;
};
// TorchScript → ONNX：python <script> <pt> <onnx>；阻塞（界面在后台线程中调用）
ConvertResult convertTorchScript(const std::string& pt, const std::string& onnx, const std::string& python, const std::string& script);
// 默认的 Python：环境变量 TERRA_PYTHON，否则 "python3"
std::string defaultPython();

struct ImportOptions {
    std::string model;            // 用户选的文件（.onnx / .pt）
    std::string scene;            // 机器人所在的场景文件
    std::string instance;         // 模型实例（空为自动）
    std::string name;             // 名字（空为自动：自动配置给出的名字或文件名）
    std::string destination;      // 目标根目录（资源库目录下的 policies/）
    std::string python;           // TorchScript 转换用（空为 defaultPython()）
    std::string converter;        // 转换脚本的路径
};

struct ImportResult {
    std::optional<std::string> policyFile;  // 成功时的说明文件
    std::string directory;                  // 导入目录
    std::string model;                      // 实际使用的 ONNX（导入目录中的副本）
    ModelFileInfo source;
    AutoconfigResult config;                // 自动配置的结果（说明、警告、无法确定的项）
    std::vector<std::string> log;           // 依次做了什么（界面与 Headless 显示）
    std::string error;
    std::string draftFile;                  // 自动配置无法完成时生成的说明文件草稿
    bool missingTorch = false;
};
ImportResult importPolicy(const ImportOptions& options, const terra::data::ModelDescription& robot);

// 目录名：字母、数字、“-”、“_”与非 ASCII 字符保留，其余换成“_”
std::string policySlug(const std::string& name);

}  // namespace terra::policy
