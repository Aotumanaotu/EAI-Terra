#pragma once

#include <string_view>

namespace terra {

// 构建时确定的版本与工具链信息，用于诊断输出和环境追溯。
struct BuildInfo {
    int versionMajor;
    int versionMinor;
    int versionPatch;
    std::string_view version;
    std::string_view compilerId;
    std::string_view compilerVersion;
    long cxxStandard;  // 编译 terra_core 时的 __cplusplus 值
};

const BuildInfo& buildInfo() noexcept;

}  // namespace terra
