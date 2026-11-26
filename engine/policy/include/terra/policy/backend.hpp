#pragma once

// 推理后端（M4-10a，设计稿第 3 节）。后端按标识注册（与 M4-08d 的导出器相同的方式），说明文件的 model.backend 选择后端。
// 本步注册两个：
//   onnxruntime-cpu  ONNX Runtime（CPU，单线程、确定性计算，同一机器上逐位可复现）。库文件是可下载的组件
//                    （terra::library 的组件清单），运行时 dlopen；组件不在时返回 missingComponent，界面据此提示下载。
//   linear           y = W x + b，模型文件为 JSON（{"format": "terra.linear-policy", "weights": [[…]], "bias": […]}）；测试与调试用。
// 以后的 GPU、其他厂商的后端作为新的注册项与新的组件加入，不改控制器。

#include <memory>
#include <string>
#include <vector>

#include "terra/policy/policy.hpp"

namespace terra::policy {

// 一个已加载、可推理的模型。输入、输出都是一维 float（batch 为 1）。
class PolicyModel {
public:
    virtual ~PolicyModel() = default;
    virtual int inputWidth() const = 0;
    virtual int outputWidth() const = 0;
    // 失败时返回假并写 error
    virtual bool run(const std::vector<float>& input, std::vector<float>& output, std::string& error) = 0;
    virtual std::string describe() const = 0;  // 如 “ONNX Runtime 1.32.0（CPU，单线程）”
};

struct BackendContext {
    std::string componentsDirectory;  // 资源库目录下的 components/
    std::string catalogPath;          // 组件清单（assets/components/components.json）
};

struct LoadModelResult {
    std::unique_ptr<PolicyModel> model;
    std::string error;
    std::string missingComponent;  // 需要而没有安装（或损坏、版本不对）的组件标识；界面提示下载
    std::vector<std::string> warnings;
};

class PolicyBackend {
public:
    virtual ~PolicyBackend() = default;
    virtual std::string id() const = 0;
    virtual std::string displayName() const = 0;
    virtual std::string component() const { return {}; }  // 需要的组件标识（没有则为空）
    virtual LoadModelResult load(const PolicyDescription& policy, const BackendContext& context) const = 0;
};

// 注册表：内置的两个后端在第一次使用时注册
void registerBackend(std::unique_ptr<PolicyBackend> backend);
const PolicyBackend* findBackend(const std::string& id);
std::vector<std::string> backendIds();

// 按说明文件加载：找后端、校验模型文件的 sha256（不一致时警告）、检查输入输出宽度与说明文件一致
LoadModelResult loadModel(const PolicyDescription& policy, const BackendContext& context);

}  // namespace terra::policy
