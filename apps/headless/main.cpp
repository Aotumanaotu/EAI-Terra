// EAI Terra Headless 入口。
//
// 生命周期：解析参数 → 检查物理后端 → 加载并初始化模型（M0-04）→ 运行 → 释放。
// 固定步进与状态输出由 M0-05/06 接入；在此之前 --dry-run 可以完整走通，
// 实际运行在加载成功后以退出码 3 如实报告“尚未接入”。
//
// 输出约定、退出码见 cli.hpp。本文件只使用 Terra 类型，不包含 MuJoCo 头文件。

#include <cstdio>
#include <exception>
#include <string>

#include "cli.hpp"
#include "terra/core/build_info.hpp"
#include "terra/physics/mujoco/backend_info.hpp"
#include "terra/physics/mujoco/mujoco_world.hpp"

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

int run(const RunOptions& options) {
    namespace mj = terra::physics::mujoco;

    if (!backendUsable()) {
        return kExitUnavailable;
    }

    mj::LoadResult loaded = mj::MujocoWorld::load(terra::data::ModelSource{options.modelPath});
    for (const std::string& warning : loaded.warnings) {
        diagnose("模型警告：" + warning);
    }
    if (loaded.error) {
        diagnose(loaded.error->message);
        return loaded.error->kind == mj::LoadErrorKind::Internal ? kExitInternal : kExitModel;
    }
    const mj::MujocoWorld& world = *loaded.world;

    ResolvedRun resolved;
    resolved.modelPath = world.description().sourcePath;
    resolved.timestep = world.description().timestep;
    if (options.steps) {
        resolved.totalSteps = *options.steps;
    } else {
        const auto steps = stepsForDuration(*options.durationSeconds, resolved.timestep);
        if (!steps) {
            diagnose("--duration 换算为步数时超出范围（步长 " + std::to_string(resolved.timestep) + " 秒）");
            return kExitUsage;
        }
        resolved.totalSteps = *steps;
    }

    if (options.dryRun) {
        std::printf("%s\n", configToJson(options, resolved).c_str());
        return std::fflush(stdout) == 0 ? kExitOk : kExitInternal;
    }

    diagnose("模型已加载，但物理步进尚未接入（M0-05）；可使用 --dry-run 校验参数与模型");
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
