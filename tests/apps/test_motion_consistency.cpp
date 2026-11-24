// Headless 与查看器执行同一动作的一致性（M3-05）。
// 用法：test_motion_consistency <terra_headless> <场景文件> <动作文件> <步数> [--optional]
//   1. 运行 terra_headless --scene … --motion … --steps N --sample-every N，取第 N 步的 state 行；
//   2. 在本进程中按查看器的路径执行：SimulationRunner + 每步控制器（动作 → PD 层），帧间隔不规则（60 Hz、144 Hz、抖动混合）；
//   3. 用 Headless 的同一个函数（stateToJson）把第 N 步的快照写成 JSON，与第 1 步的行逐字节比较
//      （数值以 %.17g 输出，逐字节相同即逐位相同）。
// --optional：场景或动作文件不存在时以返回码 77 表示跳过（参考机器人未准备时）。

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <random>
#include <string>
#include <vector>

#include "cli.hpp"
#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/scene/scene.hpp"
#include "terra/simulation/motion.hpp"
#include "terra/simulation/pd_layer.hpp"
#include "terra/simulation/runner.hpp"

namespace {

namespace fs = std::filesystem;
namespace sim = terra::simulation;

constexpr int kSkipped = 77;

std::string headlessState(const std::string& exe, const std::string& scene, const std::string& motion, std::uint64_t steps) {
    const std::string n = std::to_string(steps);
    const std::string command = "\"" + exe + "\" --scene \"" + scene + "\" --motion \"" + motion + "\" --steps " + n +
                                " --sample-every " + n + " 2>&1";
    std::FILE* pipe = popen(command.c_str(), "r");
    if (pipe == nullptr) return {};
    std::string output;
    char buffer[65536];
    std::size_t got = 0;
    while ((got = std::fread(buffer, 1, sizeof buffer, pipe)) > 0) output.append(buffer, got);
    const int status = pclose(pipe);
    if (status != 0) {
        std::fprintf(stderr, "terra_headless 退出状态 %d：\n%s\n", status, output.substr(0, 2000).c_str());
        return {};
    }
    // 最后一行 state（第 N 步）
    const std::string key = "{\"type\":\"state\",";
    const std::string stepKey = ",\"step\":" + n + ",";
    std::size_t pos = 0;
    std::string found;
    while ((pos = output.find(key, pos)) != std::string::npos) {
        const std::size_t end = output.find('\n', pos);
        const std::string line = output.substr(pos, end == std::string::npos ? std::string::npos : end - pos);
        if (line.find(stepKey) != std::string::npos) found = line;
        pos = end == std::string::npos ? output.size() : end;
    }
    return found;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 5) {
        std::fprintf(stderr, "用法：test_motion_consistency <terra_headless> <场景文件> <动作文件> <步数> [--optional]\n");
        return 2;
    }
    const std::string exe = argv[1];
    const std::string scenePath = argv[2];
    const std::string motionPath = argv[3];
    const auto steps = static_cast<std::uint64_t>(std::stoull(argv[4]));
    const bool optional = argc > 5 && std::string{argv[5]} == "--optional";
    if (optional && (!fs::exists(scenePath) || !fs::exists(motionPath))) {
        std::printf("INFO 场景或动作文件不存在（参考机器人未准备），跳过\n");
        return kSkipped;
    }
    // 参考机器人场景引用的模型不存在时同样跳过
    if (optional && !fs::exists(fs::path{scenePath}.parent_path() / "../../robots")) {
        std::printf("INFO 参考机器人未准备，跳过\n");
        return kSkipped;
    }

    const std::string expected = headlessState(exe, scenePath, motionPath, steps);
    if (expected.empty()) {
        std::fprintf(stderr, "FAILED: 没有得到 Headless 第 %llu 步的 state 行\n", static_cast<unsigned long long>(steps));
        return 1;
    }

    // 查看器的路径：runner + 每步控制器，帧间隔不规则
    auto loaded = terra::scene::loadScene(scenePath, terra::physics::mujoco::composeWorld);
    if (!loaded.ok()) {
        std::fprintf(stderr, "FAILED: 加载场景 %s\n", scenePath.c_str());
        return 1;
    }
    auto& world = *loaded.scene->world;
    const auto& d = world.description();
    sim::PdLayer pd(d, loaded.scene->pd);
    const auto parsed = sim::readMotionFile(motionPath);
    if (!parsed.document) {
        std::fprintf(stderr, "FAILED: 读取动作 %s\n", motionPath.c_str());
        return 1;
    }
    const auto bound = sim::bindMotion(*parsed.document, d, pd.targetRangeFunction());
    if (!bound.motion) {
        std::fprintf(stderr, "FAILED: 绑定动作\n");
        return 1;
    }
    sim::SimulationRunner runner(world);
    if (bound.motion->startKeyframe) runner.resetToKeyframe(*bound.motion->startKeyframe);
    sim::MotionPlayer player(*bound.motion);
    player.start(world.stepIndex());
    const double dt = d.timestep;
    runner.setController([&](std::uint64_t step) {
        auto commands = player.commands(step, dt);
        if (!pd.empty()) commands = pd.process(std::move(commands), *world.snapshot());
        return commands;
    });
    std::mt19937 rng(11);
    std::uniform_real_distribution<double> jitter(0.0005, 0.035);
    const double fixed[] = {1.0 / 60, 1.0 / 144};
    std::size_t frame = 0;
    while (world.stepIndex() < steps) {
        const double wall = frame % 3 == 2 ? jitter(rng) : fixed[frame % 2];
        ++frame;
        const double remaining = static_cast<double>(steps - world.stepIndex());
        runner.update(std::min(wall, (remaining - 0.5) * dt));
    }
    const std::string actual = terra::headless::stateToJson(*world.snapshot());
    std::printf("INFO %s + %s：%llu 步，%zu 帧，PD 层 %zu 个执行器\n", fs::path{scenePath}.filename().c_str(),
                fs::path{motionPath}.filename().c_str(), static_cast<unsigned long long>(steps), frame, pd.size());
    if (actual != expected) {
        std::fprintf(stderr, "FAILED: Headless 与查看器路径第 %llu 步的状态不同\n  Headless: %.300s…\n  查看器:   %.300s…\n",
                     static_cast<unsigned long long>(steps), expected.c_str(), actual.c_str());
        return 1;
    }
    std::printf("INFO 第 %llu 步的完整状态（%zu 字节 JSON）逐字节相同\nall passed\n", static_cast<unsigned long long>(steps),
                actual.size());
    return 0;
}
