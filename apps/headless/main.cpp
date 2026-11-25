// EAI Terra Headless 入口。
//
// 生命周期：解析参数 → 检查物理后端 → 加载并初始化模型（M0-04）→ 固定步进运行（M0-05）→ 释放。
// 世界通过后端无关的 PhysicsWorld 接口使用（M0-07）。不给动作时不设置控制，执行器保持模型的中性值（场景中的 PD 层除外）。
// M3-05：--motion 时每一步之前按“动作 → PD 层 → setControl”下发控制，与查看器的每步控制器是同一条路径，结果逐位相同。
// 按步数离线运行，不按墙钟 sleep 或追赶实时；输出采样不影响仿真。
// 输出格式见 cli.hpp 与 M0-06 报告。
//
// 输出约定、退出码见 cli.hpp。本文件只使用 Terra 类型，不包含 MuJoCo 头文件。

#include <cstdio>
#include <exception>
#include <filesystem>
#include <memory>
#include <ctime>
#include <chrono>
#include <thread>
#include <optional>
#include <string>
#include <vector>

#include "cli.hpp"
#include "terra/core/build_info.hpp"
#include "terra/core/run_schedule.hpp"
#include "terra/physics/mujoco/backend_info.hpp"
#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/physics/physics_world.hpp"
#include "terra/scene/scene.hpp"
#include "terra/simulation/motion.hpp"
#include "terra/simulation/pd_layer.hpp"
#include "terra/simulation/record.hpp"
#include "terra/simulation/sensor_sampling.hpp"
#include "terra/simulation/lidar.hpp"
#ifdef TERRA_WITH_ROS2
#include "terra/ros2/bridge.hpp"
#endif
#include "terra/library/library.hpp"
#include "terra/core/sha256.hpp"

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

namespace physics = terra::physics;

const char* errorKindName(physics::SimulationErrorKind kind) {
    switch (kind) {
        case physics::SimulationErrorKind::NonFinite: return "non_finite";
        case physics::SimulationErrorKind::AutoReset: return "auto_reset";
        case physics::SimulationErrorKind::SolverWarning: return "solver_warning";
        case physics::SimulationErrorKind::Faulted: return "faulted";
    }
    return "unknown";
}

// 写一行 JSON 到 stdout；写入失败返回 false。
bool emit(const std::string& line) {
    return std::fputs(line.c_str(), stdout) >= 0 && std::fputc('\n', stdout) != EOF;
}

std::string sampleState(const physics::PhysicsWorld& world) {
    return stateToJson(*world.snapshot());
}

// 实验记录头中的运行环境（M3-07）
void fillEnvironment(terra::simulation::RecordHeader& h) {
    const terra::BuildInfo& info = terra::buildInfo();
    h.terraVersion = std::string{info.version};
    h.compiler = std::string{info.compilerId} + " " + std::string{info.compilerVersion};
    const terra::physics::mujoco::BackendInfo backend = terra::physics::mujoco::backendInfo();
    h.physicsBackend = std::string{backend.name} + " " + std::string{backend.libraryVersionString};
#if defined(__linux__)
    h.platform = "linux";
#elif defined(_WIN32)
    h.platform = "windows";
#elif defined(__APPLE__)
    h.platform = "macos";
#else
    h.platform = "unknown";
#endif
#if defined(__x86_64__) || defined(_M_X64)
    h.platform += " x86_64";
#elif defined(__aarch64__) || defined(_M_ARM64)
    h.platform += " aarch64";
#endif
    const std::time_t now = std::time(nullptr);
    std::tm utc{};
    gmtime_r(&now, &utc);
    char buf[32];
    std::strftime(buf, sizeof buf, "%Y-%m-%dT%H:%M:%SZ", &utc);
    h.created = buf;
}

// --rerun（M3-07）：按记录重新仿真并比较，输出一行 rerun
int rerun(const RunOptions& options) {
    if (!backendUsable()) return kExitUnavailable;
    const auto read = terra::simulation::readRecord(options.rerunPath);
    for (const auto& w : read.warnings) diagnose("记录警告：" + w);
    for (const auto& e : read.errors) diagnose("记录错误：" + e);
    if (!read.record) return kExitModel;
    terra::simulation::RerunOptions ro;
    ro.positionTolerance = options.positionTolerance;
    ro.rotationTolerance = options.rotationTolerance;
    const auto result = terra::simulation::rerunRecord(*read.record, terra::physics::mujoco::composeWorld, ro);
    for (const auto& w : result.warnings) diagnose("复现警告：" + w);
    if (!result.ran) {
        diagnose("复现失败：" + result.error);
        return kExitModel;
    }
    std::string json = "{\"type\":\"rerun\",\"record\":\"" + jsonEscape(options.rerunPath) + "\",\"steps\":" +
                       std::to_string(result.steps) + ",\"compared\":" + std::to_string(result.compared) +
                       ",\"bitwise\":" + (result.bitwise ? "true" : "false") + ",\"max_position_error\":" +
                       formatNumber(result.maxPositionError) + ",\"max_rotation_error\":" + formatNumber(result.maxRotationError) +
                       ",\"position_tolerance\":" + formatNumber(ro.positionTolerance) + ",\"rotation_tolerance\":" +
                       formatNumber(ro.rotationTolerance) + ",\"first_exceed_step\":" +
                       (result.firstExceedStep ? std::to_string(*result.firstExceedStep) : std::string{"null"}) +
                       ",\"within_tolerance\":" + (result.withinTolerance() ? "true" : "false") + ",\"sensor_samples\":" +
                       std::to_string(result.sensorSamples) + ",\"sensors_bitwise\":" + (result.sensorsBitwise ? "true" : "false") + "}";
    if (!result.sensorsBitwise) diagnose("复现诊断：传感器 " + result.firstSensorDifference);
    if (result.firstDifferenceStep) {
        diagnose("复现诊断：第一个不逐位相同的采样点在第 " + std::to_string(*result.firstDifferenceStep) + " 步：" + result.firstDifference);
    }
    if (!emit(json) || std::fflush(stdout) != 0) return kExitInternal;
    return result.withinTolerance() ? kExitOk : kExitRerunMismatch;
}

// --list-records（M3-07b）：每行一个 record
int listRecords(const RunOptions& options) {
    std::string error, warning;
    const std::string path = options.libraryPath.empty() ? terra::library::defaultLibraryPath() : options.libraryPath;
    auto library = terra::library::Library::open(path, error, &warning);
    if (!warning.empty()) diagnose("资源库警告：" + warning);
    if (!library) {
        diagnose("资源库无法打开：" + error);
        return kExitInternal;
    }
    std::string scene;
    if (!options.scenePath.empty()) {
        std::error_code ec;
        scene = std::filesystem::absolute(options.scenePath, ec).lexically_normal().string();
    }
    bool ok = true;
    for (const auto& r : library->records(scene)) {
        std::string json = "{\"type\":\"record\",\"path\":\"" + jsonEscape(r.path) + "\",\"created\":\"" + jsonEscape(r.created) +
                           "\",\"scene\":\"" + jsonEscape(r.scene) + "\",\"motion\":\"" + jsonEscape(r.motion) +
                           "\",\"steps\":" + (r.steps ? std::to_string(*r.steps) : std::string{"null"}) +
                           ",\"time\":" + (r.simulationTime ? formatNumber(*r.simulationTime) : std::string{"null"}) +
                           ",\"reason\":\"" + jsonEscape(r.reason) + "\",\"bytes\":" + std::to_string(r.bytes) +
                           ",\"sha256\":\"" + r.fileSha256 + "\"}";
        ok = ok && emit(json);
    }
    return ok && std::fflush(stdout) == 0 ? kExitOk : kExitInternal;
}

int run(const RunOptions& requested) {
    if (requested.listRecords) return listRecords(requested);
    if (!requested.rerunPath.empty()) return rerun(requested);
    if (!backendUsable()) {
        return kExitUnavailable;
    }
    RunOptions options = requested;

    // 动作文件（M3-05）：先解析；没给 --scene 时使用其中的推荐场景
    std::optional<terra::simulation::MotionDocument> motionDocument;
    if (!options.motionPath.empty()) {
        const auto parsed = terra::simulation::readMotionFile(options.motionPath);
        for (const auto& issue : parsed.issues) {
            const bool error = issue.severity == terra::simulation::MotionIssue::Severity::Error;
            diagnose(std::string{error ? "动作错误 " : "动作警告 "} + issue.where + (issue.where.empty() ? "" : "：") + issue.message);
        }
        if (!parsed.document) return kExitModel;
        motionDocument = *parsed.document;
        if (options.scenePath.empty()) {
            if (motionDocument->scene.empty()) {
                diagnose("动作文件没有推荐场景，请用 --scene 指定场景文件");
                return kExitUsage;
            }
            options.scenePath = motionDocument->scene;
        }
    }

    // 唯一与具体后端相关的调用；之后只通过 PhysicsWorld 接口使用世界。
    std::unique_ptr<physics::PhysicsWorld> owned;
    std::optional<terra::scene::SceneInstance> sceneInstance;  // M4-06a：ROS2 桥接需要场景（传感器、相机、雷达）
    std::vector<terra::scene::PdAssignment> pdAssignments;
    std::vector<terra::scene::SensorConfig> sensorConfigs;  // M4-02：场景中各传感器的采样频率
    std::vector<terra::scene::LidarSensor> lidars;          // M4-03b：场景中的 2D 激光雷达
    if (!options.scenePath.empty()) {
        // 场景文件（M2-08）：由 terra_scene 组合模型实例，只跑物理；视觉绑定只检查文件是否存在
        auto scene = terra::scene::loadScene(options.scenePath, terra::physics::mujoco::composeWorld);
        for (const auto& issue : scene.issues) {
            const bool error = issue.severity == terra::scene::SceneIssue::Severity::Error;
            diagnose(std::string{error ? "场景错误 " : "场景警告 "} + issue.where + (issue.where.empty() ? "" : "：") +
                     (issue.instance.empty() ? std::string{} : "[" + issue.instance + "] ") + issue.message);
        }
        if (!scene.ok()) return kExitModel;
        pdAssignments = scene.scene->pd;
        sensorConfigs = scene.scene->sensors;
        lidars = scene.scene->lidars;
        // 相机需要图形上下文（M4-03）：Headless 不链接图形库，只提示、不渲染
        if (!scene.scene->cameras.empty()) {
            diagnose("场景中有 " + std::to_string(scene.scene->cameras.size()) +
                     " 个相机：相机需要图形上下文，在 Headless 中不可用（请用查看器或 terra_capture）；其余照常运行");
        }
        sceneInstance = std::move(*scene.scene);
    } else {
        physics::LoadResult loaded = terra::physics::mujoco::loadWorld(terra::data::ModelSource{options.modelPath});
        for (const std::string& warning : loaded.warnings) {
            diagnose("模型警告：" + warning);
        }
        if (loaded.error) {
            diagnose(loaded.error->message);
            return loaded.error->kind == physics::LoadErrorKind::Internal ? kExitInternal : kExitModel;
        }
        owned = std::move(loaded.world);
    }
    physics::PhysicsWorld& world = sceneInstance ? *sceneInstance->world : *owned;

    // PD 层（M3-04b）与动作（M3-05）
    terra::simulation::PdLayer pd(world.description(), pdAssignments);
    std::optional<terra::simulation::MotionPlayer> player;
    if (motionDocument) {
        const auto bound = terra::simulation::bindMotion(*motionDocument, world.description(), pd.targetRangeFunction());
        for (const auto& issue : bound.issues) {
            const bool error = issue.severity == terra::simulation::MotionIssue::Severity::Error;
            diagnose(std::string{error ? "动作错误 " : "动作警告 "} + issue.where + (issue.where.empty() ? "" : "：") + issue.message);
        }
        if (!bound.motion) return kExitModel;
        if (const auto key = bound.motion->startKeyframe) world.resetToKeyframe(*key);
        player.emplace(*bound.motion);
        player->start(world.stepIndex());
    }

    ResolvedRun resolved;
    resolved.modelPath = world.description().sourcePath;
    resolved.timestep = world.description().timestep;
    if (options.steps) {
        resolved.totalSteps = *options.steps;
    } else {
        // 没给步数与时长时（只有 --motion 才允许）：运行到动作结束后 1 秒
        const double duration = options.durationSeconds ? *options.durationSeconds : player->motion().duration + 1.0;
        const auto steps = stepsForDuration(duration, resolved.timestep);
        if (!steps) {
            diagnose("运行时长换算为步数时超出范围（步长 " + std::to_string(resolved.timestep) + " 秒）");
            return kExitUsage;
        }
        resolved.totalSteps = *steps;
    }

    bool ok = emit(configToJson(options, resolved));
    if (options.dryRun) {
        return ok && std::fflush(stdout) == 0 ? kExitOk : kExitInternal;
    }

    // 实验记录（M3-07）：记录头在起始关键帧之后写，与复现时的起始状态一致
    std::unique_ptr<terra::simulation::RecordWriter> recorder;
    if (!options.recordPath.empty()) {
        auto header = terra::simulation::makeRecordHeader(world.description(), options.sampleEvery);
        if (sceneInstance) terra::simulation::describeSensors(header, *sceneInstance);  // M4-07
        fillEnvironment(header);
        std::error_code ec;
        header.scenePath = std::filesystem::absolute(options.scenePath, ec).lexically_normal().string();
        header.sceneSha256 = terra::sha256File(header.scenePath).value_or("");
        if (motionDocument) {
            header.motionPath = motionDocument->path;
            header.motionSha256 = terra::sha256File(header.motionPath).value_or("");
            header.startKeyframe = motionDocument->startKeyframe;
        }
        std::string error;
        recorder = terra::simulation::RecordWriter::create(options.recordPath, header, error);
        if (!recorder) {
            diagnose(error);
            return kExitInternal;
        }
    }

    ok = ok && emit(modelToJson(world.description()));
    const terra::RunSchedule schedule{resolved.totalSteps, options.sampleEvery};
    const double dt = resolved.timestep;

    // 传感器（M4-02）：频率来自场景文件，直接加载 MJCF 时等于物理频率；按 sensorSampleDue 采样
    const auto& sensors = world.description().sensors;
    std::vector<double> sensorRates(sensors.size(), dt > 0 ? 1.0 / dt : 0.0);
    for (const auto& c : sensorConfigs) {
        if (c.sensor.valid() && c.sensor.value <= sensorRates.size()) sensorRates[c.sensor.value - 1] = c.rateHz;
    }
    const auto emitSensors = [&]() {
        if (!options.sensors) return true;
        const auto snap = world.snapshot();
        bool written = true;
        for (std::size_t k = 0; k < sensors.size() && k < snap->sensors.size(); ++k) {
            if (!terra::simulation::sensorSampleDue(snap->stepIndex, dt, sensorRates[k])) continue;
            written = written && emit(sensorToJson(sensors[k], snap->sensors[k], snap->stepIndex, snap->simulationTime));
        }
        // 2D 激光雷达（M4-03b）：只在采样步上做射线检测
        for (const auto& lidar : lidars) {
            if (!terra::simulation::sensorSampleDue(snap->stepIndex, dt, lidar.rateHz)) continue;
            const auto scan = terra::simulation::scanLidar(world, lidar);
            LidarJson meta;
            meta.name = scan.sensor;
            meta.step = scan.step;
            meta.time = scan.time;
            meta.angleMin = scan.angleMin;
            meta.angleIncrement = scan.angleIncrement;
            meta.rangeMin = scan.rangeMin;
            meta.rangeMax = scan.rangeMax;
            meta.rateHz = lidar.rateHz;
            written = written && emit(lidarToJson(meta, scan.ranges));
        }
        return written;
    };
    if (options.sensors) {
        if (sensors.empty() && lidars.empty()) diagnose("--sensors：模型中没有传感器（MJCF 中的 IMU，或场景文件 sensors 中添加的）");
        ok = ok && emit(sensorListToJson(world.description(), sensorRates));
    }

    // ROS2 桥接（M4-06a）：只有构建时打开 TERRA_BUILD_ROS2 才可用；接上后按墙钟节奏推进，并处理重置、暂停、单步请求
#ifdef TERRA_WITH_ROS2
    std::unique_ptr<terra::ros2::Bridge> bridge;
    if (!options.ros2Profile.empty()) {
        const auto profile = terra::ros2::readProfile(options.ros2Profile);
        for (const auto& e : profile.errors) diagnose("通信配置错误 " + e);
        if (!profile.profile) return kExitUsage;
        std::string error;
        bridge = terra::ros2::Bridge::create(*sceneInstance, pd, *profile.profile, error);
        if (!bridge) {
            diagnose(error);
            return kExitUnavailable;
        }
        diagnose("ROS2 桥接已开启：" + profile.profile->name + "（" + options.ros2Profile + "）");
    }
#else
    if (!options.ros2Profile.empty()) {
        diagnose("本程序构建时没有 ROS2 桥接。请安装 ROS2 Humble（ros-humble-ros-base，说明："
                 "https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html），source /opt/ros/humble/setup.bash 后"
                 "以 -DTERRA_BUILD_ROS2=ON 重新构建");
        return kExitUnavailable;
    }
#endif
    const bool bridged = !options.ros2Profile.empty();
    [[maybe_unused]] bool paused = false;  // ROS2 的 /terra/pause
    // 墙钟节奏：从锚点起，第 n 步不早于锚点 + 经过的仿真时间；暂停、重置后重新取锚点
    using Clock = std::chrono::steady_clock;
    auto anchorWall = Clock::now();
    double anchorSim = world.simulationTime();
    [[maybe_unused]] const auto reanchor = [&] {
        anchorWall = Clock::now();
        anchorSim = world.simulationTime();
    };
    const auto publishBridge = [&] {
#ifdef TERRA_WITH_ROS2
        if (bridge) bridge->publish(*world.snapshot());
#endif
    };

    ok = ok && emit(sampleState(world));  // 第 0 步：初始状态
    ok = ok && emitSensors();
    publishBridge();
    if (recorder) recorder->state(*world.snapshot());
    // M4-07：记录中的雷达扫描与相机元数据（各自的频率；IMU 随 state 记录）
    const auto recordSensors = [&] {
        if (recorder && sceneInstance) terra::simulation::recordSceneSensors(*recorder, world, *sceneInstance, *world.snapshot());
    };
    recordSensors();
    for (std::uint64_t i = 0; ok && i < resolved.totalSteps; ++i) {
#ifdef TERRA_WITH_ROS2
        // ROS2 的服务请求：暂停期间等待（继续或单步）；重置回到场景的初始状态
        if (bridge) {
            bool stepOnce = false;
            for (;;) {
                for (const auto r : bridge->takeRequests()) {
                    if (r == terra::ros2::Request::Pause) {
                        paused = true;
                    } else if (r == terra::ros2::Request::Resume) {
                        paused = false;
                        reanchor();
                    } else if (r == terra::ros2::Request::Step) {
                        stepOnce = true;
                    } else {
                        if (sceneInstance && sceneInstance->homeKeyframe) world.resetToKeyframe(*sceneInstance->homeKeyframe);
                        else world.reset();
                        pd.clearTargets();
                        if (player) player->stop();
                        publishBridge();
                        reanchor();
                    }
                }
                if (!paused || stepOnce) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }
#endif
        if (options.realtime) {
            const double ahead = world.simulationTime() + dt - anchorSim;
            std::this_thread::sleep_until(anchorWall + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(ahead)));
        }
        // 每步控制：动作给出的命令 → PD 层（与查看器的每步控制器相同）；ROS2 的命令并在动作之后（M4-06a）
        if (player || !pd.empty() || bridged) {
            auto commands = player ? player->commands(world.stepIndex(), dt) : std::vector<physics::ActuatorCommand>{};
#ifdef TERRA_WITH_ROS2
            if (bridge) commands = terra::ros2::mergeCommands(std::move(commands), bridge->takeCommands());
#endif
            if (!pd.empty()) commands = pd.process(std::move(commands), *world.snapshot());
#ifdef TERRA_WITH_ROS2
            // M4-06b：宇树 LowCmd 换算的力矩在 PD 层之后，同一执行器以它为准（PD 层让位）
            if (bridge) commands = terra::ros2::mergeCommands(std::move(commands), bridge->directCommands());
#endif
            if (!commands.empty()) {
                if (recorder) recorder->control(world.stepIndex(), commands);
                if (const auto error = world.setControl(commands)) {
                    emit(errorToJson("control_rejected", world.runEpoch(), world.stepIndex(), world.simulationTime(), error->message));
                    std::fflush(stdout);
                    diagnose("第 " + std::to_string(world.stepIndex()) + " 步的控制被拒绝：" + error->message);
                    if (recorder) {
                        recorder->event(world.stepIndex(), "error", error->message);
                        recorder->finish(world.stepIndex(), world.simulationTime(), "error");
                    }
                    return kExitSimulation;
                }
            }
        }
        if (const auto error = world.step()) {
            emit(errorToJson(errorKindName(error->kind), world.runEpoch(), error->step, world.simulationTime(),
                             error->message));
            std::fflush(stdout);
            diagnose("仿真在第 " + std::to_string(error->step) + " 步出错：" + error->message);
            if (recorder) {
                recorder->event(error->step, "error", error->message);
                recorder->finish(world.stepIndex(), world.simulationTime(), "error");
            }
            return kExitSimulation;
        }
        if (schedule.shouldSample(world.stepIndex())) {
            ok = emit(sampleState(world));
            if (recorder) recorder->state(*world.snapshot());
        }
        recordSensors();
        ok = ok && emitSensors();
        publishBridge();
    }
#ifdef TERRA_WITH_ROS2
    if (bridge) {
        const auto st = bridge->stats();
        diagnose("ROS2 桥接：发布 " + std::to_string(st.published) + " 条消息；命令接受 " + std::to_string(st.commandsAccepted) + " 条、拒绝 " +
                 std::to_string(st.commandsRejected) + " 条" + (st.lastRejection.empty() ? "" : "（最近一次：" + st.lastRejection + "）"));
    }
#endif
    if (recorder && !recorder->finish(world.stepIndex(), world.simulationTime(), "completed")) {
        diagnose("写入实验记录失败：" + options.recordPath);
        return kExitInternal;
    }
    // 加入资源库索引（M3-07b）：失败只警告，记录文件本身已完整写出
    if (recorder) {
        std::string error, warning;
        const std::string path = options.libraryPath.empty() ? terra::library::defaultLibraryPath() : options.libraryPath;
        auto library = terra::library::Library::open(path, error, &warning);
        if (!warning.empty()) diagnose("资源库警告：" + warning);
        if (!library) {
            diagnose("记录未加入资源库（" + error + "）");
        } else if (const auto failed = library->addRecord(options.recordPath)) {
            diagnose("记录未加入资源库（" + *failed + "）");
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
