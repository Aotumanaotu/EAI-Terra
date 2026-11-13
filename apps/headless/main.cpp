// EAI Terra Headless 入口（M0-03）。
//
// 生命周期：解析参数 → 检查物理后端 → 检查模型文件 → 运行 → 释放。
// 物理运行（加载 MJCF、固定步进、状态输出）由 M0-04–06 接入；在此之前只有 --dry-run
// 能完整走通，实际运行以退出码 3 如实报告“尚未接入”。
//
// 输出约定、退出码见 cli.hpp。本文件只使用 Terra 类型，不包含 MuJoCo 头文件。

#include <cstdio>
#include <exception>
#include <filesystem>
#include <string>
#include <system_error>

#include "cli.hpp"
#include "terra/core/build_info.hpp"
#include "terra/physics/mujoco/backend_info.hpp"

namespace {

using namespace terra::headless;

void diagnose(const std::string& message) {
    std::fprintf(stderr, "terra_headless: %s\n", message.c_str());
}

// 物理后端可用性检查：头文件与运行库版本必须一致。
bool backendUsable() {
    const terra::physics::mujoco::BackendInfo backend = terra::physics::mujoco::backendInfo();
    if (!backend.versionsMatch()) {
        diagnose("MuJoCo 头文件版本 " + std::to_string(backend.headerVersion) + " 与运行库版本 " +
                 std::to_string(backend.libraryVersion) + " 不一致");
        return false;
    }
    return true;
}

int printVersion() {
    const terra::BuildInfo& info = terra::buildInfo();
    std::printf("terra_headless %.*s (%.*s %.*s, __cplusplus=%ld)\n",
                static_cast<int>(info.version.size()), info.version.data(),
                static_cast<int>(info.compilerId.size()), info.compilerId.data(),
                static_cast<int>(info.compilerVersion.size()), info.compilerVersion.data(),
                info.cxxStandard);

    const terra::physics::mujoco::BackendInfo backend = terra::physics::mujoco::backendInfo();
    std::printf("physics backend: %.*s %.*s\n",
                static_cast<int>(backend.name.size()), backend.name.data(),
                static_cast<int>(backend.libraryVersionString.size()),
                backend.libraryVersionString.data());
    return backendUsable() ? kExitOk : kExitUnavailable;
}

std::string resolvePath(const std::string& path) {
    std::error_code ec;
    const std::filesystem::path resolved = std::filesystem::weakly_canonical(path, ec);
    return ec ? path : resolved.string();
}

int run(const RunOptions& options) {
    if (!backendUsable()) {
        return kExitUnavailable;
    }
    if (const std::string error = checkModelFile(options.modelPath); !error.empty()) {
        diagnose(error);
        return kExitModel;
    }

    if (options.dryRun) {
        std::printf("%s\n", configToJson(options, resolvePath(options.modelPath)).c_str());
        return std::fflush(stdout) == 0 ? kExitOk : kExitInternal;
    }

    diagnose("物理运行尚未接入（M0-04–06）；可使用 --dry-run 校验参数");
    return kExitUnavailable;
}

int mainImpl(int argc, char** argv) {
    const ParseResult parsed = parseArguments(argc, argv);
    if (!parsed.ok()) {
        diagnose(parsed.error);
        std::fprintf(stderr, "\n%s", usageText().c_str());
        return kExitUsage;
    }

    switch (parsed.command) {
        case Command::Help:
            std::fputs(usageText().c_str(), stdout);
            return kExitOk;
        case Command::Version:
            return printVersion();
        case Command::Run:
            return run(parsed.options);
    }
    return kExitInternal;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        return mainImpl(argc, argv);
    } catch (const std::exception& e) {
        diagnose(std::string{"内部错误："} + e.what());
    } catch (...) {
        diagnose("内部错误：未知异常");
    }
    return kExitInternal;
}
