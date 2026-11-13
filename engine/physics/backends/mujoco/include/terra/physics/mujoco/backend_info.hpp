#pragma once

#include <string_view>

namespace terra::physics::mujoco {

// MuJoCo 后端的版本信息。版本号编码为 major*1000000 + minor*1000 + patch。
struct BackendInfo {
    std::string_view name;
    int headerVersion;                       // 编译时头文件版本
    int libraryVersion;                      // 运行时实际装载的库版本
    std::string_view libraryVersionString;   // 运行时库的版本字符串，如 "4.0.1"

    // 头文件与运行库版本一致时才可以安全使用后端。
    bool versionsMatch() const noexcept { return headerVersion == libraryVersion; }
};

BackendInfo backendInfo() noexcept;

}  // namespace terra::physics::mujoco
