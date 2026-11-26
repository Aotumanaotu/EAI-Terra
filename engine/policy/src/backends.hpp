#pragma once

// 内置后端的工厂（只在本模块内使用）
#include <memory>

#include "terra/policy/backend.hpp"

namespace terra::policy {

inline constexpr const char* kOnnxRuntimeComponent = "onnxruntime-cpu";
std::unique_ptr<PolicyBackend> makeOnnxRuntimeBackend();

}  // namespace terra::policy
