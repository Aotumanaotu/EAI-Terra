// EAI Terra Headless 入口。
//
// 生命周期：解析参数 → 检查物理后端 → 加载并初始化模型（M0-04）→ 固定步进运行（M0-05）→ 释放。
// 按步数离线运行，不按墙钟 sleep 或追赶实时；输出采样不影响仿真。
// 状态记录格式为初版，正式格式在 M0-06 确定。
//
// 输出约定、退出码见 cli.hpp。本文件只使用 Terra 类型，不包含 MuJoCo 头文件。

#include <cstdio>
#include <exception>
#include <string>
#include <vector>

#include "cli.hpp"
#include "terra/core/build_info.hpp"
#include "terra/core/run_schedule.hpp"
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

namespace mj = terra::physics::mujoco;

const char* errorKindName(mj::SimulationErrorKind kind) {
    switch (kind) {
        case mj::SimulationErrorKind::NonFinite: return "non_finite";
        case mj::SimulationErrorKind::AutoReset: return "auto_reset";
        case mj::SimulationErrorKind::SolverWarning: return "solver_warning";
        case mj::SimulationErrorKind::Faulted: return "faulted";
    }
    return "unknown";
}

// 写一行 JSON 到 stdout；写入失败返回 false。
bool emit(const std::string& line) {
    return std::fputs(line.c_str(), stdout) >= 0 && std::fputc('\n', stdout) != EOF;
}

std::string sampleState(const mj::MujocoWorld& world) {
    std::vector<EntitySample> samples;
    samples.reserve(world.description().entities.size());
    for (const auto& entity : world.description().entities) {
        if (const auto pose = world.entityPose(entity.id)) {
            samples.push_back({entity.id, *pose});
        }
    }
    return stateToJson(world.runEpoch(), world.stepIndex(), world.simulationTime(), samples);
}

int run(const RunOptions& options) {
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
    mj::MujocoWorld& world = *loaded.world;

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

    bool ok = emit(configToJson(options, resolved));
    if (options.dryRun) {
        return ok && std::fflush(stdout) == 0 ? kExitOk : kExitInternal;
    }

    ok = ok && emit(modelToJson(world.description()));
    const terra::RunSchedule schedule{resolved.totalSteps, options.sampleEvery};
    ok = ok && emit(sampleState(world));  // 第 0 步：初始状态
    for (std::uint64_t i = 0; ok && i < resolved.totalSteps; ++i) {
        if (const auto error = world.step()) {
            emit(errorToJson(errorKindName(error->kind), world.runEpoch(), error->step, world.simulationTime(),
                             error->message));
            std::fflush(stdout);
            diagnose("仿真在第 " + std::to_string(error->step) + " 步出错：" + error->message);
            return kExitSimulation;
        }
        if (schedule.shouldSample(world.stepIndex())) {
            ok = emit(sampleState(world));
        }
    }
    ok = ok && emit(summaryToJson(world.runEpoch(), world.stepIndex(), world.simulationTime()));

    if (!ok || std::fflush(stdout) != 0) {
        diagnose("写入 stdout 失败");
        return kExitInternal;
    }
    return kExitOk;
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
