// terra_bench：M0 物理步进性能基线（开发计划 4.4 节，NFR-03）。只记录，不设阻塞门槛。
//
// 三种模式分别计时（每步单独计时，steady_clock）：
//   step       只推进物理
//   snapshot   推进后每步取快照（等同 Headless 每步观测，但不序列化）
//   jsonl      推进、取快照，并按 --sample-every 把状态序列化为 JSON Lines 写入 /dev/null
// 每种模式先预热 --warmup 轮，再正式运行 --repeat 轮；每轮从 reset 开始运行 --steps 步。
// 输出：BENCH 开头的单行记录（便于脚本提取），以及运行条件（模型摘要、构建配置、版本）。
//
// 用法：terra_bench --model <MJCF> [--steps N] [--repeat R] [--warmup W] [--sample-every K]

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "cli.hpp"
#include "terra/core/build_info.hpp"
#include "terra/physics/mujoco/backend_info.hpp"
#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/physics/physics_world.hpp"

#ifndef TERRA_BENCH_BUILD_TYPE
#define TERRA_BENCH_BUILD_TYPE "unknown"
#endif

namespace {

using Clock = std::chrono::steady_clock;

struct Options {
    std::string model;
    long steps = 5000;
    long repeat = 20;
    long warmup = 2;
    long sampleEvery = 1;
};

bool parsePositive(const char* text, long& out) {
    char* end = nullptr;
    const long v = std::strtol(text, &end, 10);
    if (end == text || *end != '\0' || v <= 0) return false;
    out = v;
    return true;
}

bool parse(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (i + 1 >= argc) return false;
        const char* value = argv[++i];
        if (arg == "--model") o.model = value;
        else if (arg == "--steps") { if (!parsePositive(value, o.steps)) return false; }
        else if (arg == "--repeat") { if (!parsePositive(value, o.repeat)) return false; }
        else if (arg == "--warmup") { if (!parsePositive(value, o.warmup)) return false; }
        else if (arg == "--sample-every") { if (!parsePositive(value, o.sampleEvery)) return false; }
        else return false;
    }
    return !o.model.empty();
}

enum class Mode { Step, Snapshot, Jsonl };

const char* modeName(Mode m) {
    switch (m) {
        case Mode::Step: return "step";
        case Mode::Snapshot: return "snapshot";
        case Mode::Jsonl: return "jsonl";
    }
    return "?";
}

double percentile(std::vector<double>& sorted, double p) {
    if (sorted.empty()) return 0.0;
    const auto index = static_cast<std::size_t>(p * static_cast<double>(sorted.size() - 1) + 0.5);
    return sorted[std::min(index, sorted.size() - 1)];
}

// 运行一轮；把每步耗时（微秒）追加到 stepMicros，返回本轮墙钟时间（秒）。失败返回负值。
double runOnce(terra::physics::PhysicsWorld& world, const Options& o, Mode mode, std::FILE* sink,
               std::vector<double>* stepMicros) {
    world.reset();
    const auto begin = Clock::now();
    for (long k = 1; k <= o.steps; ++k) {
        const auto t0 = Clock::now();
        if (world.step()) return -1.0;
        if (mode != Mode::Step) {
            const auto snap = world.snapshot();
            if (mode == Mode::Jsonl && k % o.sampleEvery == 0) {
                const std::string line = terra::headless::stateToJson(*snap);
                std::fputs(line.c_str(), sink);
                std::fputc('\n', sink);
            }
        }
        const auto t1 = Clock::now();
        if (stepMicros) stepMicros->push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
    }
    return std::chrono::duration<double>(Clock::now() - begin).count();
}

}  // namespace

int main(int argc, char** argv) {
    Options o;
    if (!parse(argc, argv, o)) {
        std::fprintf(stderr,
                     "用法：terra_bench --model <MJCF> [--steps N] [--repeat R] [--warmup W] [--sample-every K]\n");
        return 2;
    }
    auto loaded = terra::physics::mujoco::loadWorld(terra::data::ModelSource{o.model});
    if (!loaded.world) {
        std::fprintf(stderr, "加载失败：%s\n", loaded.error ? loaded.error->message.c_str() : "");
        return 4;
    }
    auto& world = *loaded.world;
    const auto& d = world.description();
    std::FILE* sink = std::fopen("/dev/null", "w");
    if (!sink) {
        std::fprintf(stderr, "无法打开 /dev/null\n");
        return 1;
    }

    const auto& build = terra::buildInfo();
    const auto backend = terra::physics::mujoco::backendInfo();
    std::printf("CONDITION build=%s compiler=%.*s-%.*s terra=%.*s backend=%.*s-%.*s\n", TERRA_BENCH_BUILD_TYPE,
                static_cast<int>(build.compilerId.size()), build.compilerId.data(),
                static_cast<int>(build.compilerVersion.size()), build.compilerVersion.data(),
                static_cast<int>(build.version.size()), build.version.data(), static_cast<int>(backend.name.size()),
                backend.name.data(), static_cast<int>(backend.libraryVersionString.size()),
                backend.libraryVersionString.data());
    std::printf("CONDITION model=%s dt=%g integrator=%s solver=%s iterations=%d entities=%zu joints=%zu actuators=%zu "
                "steps=%ld repeat=%ld warmup=%ld sample_every=%ld\n",
                d.sourcePath.c_str(), d.timestep, d.settings.integrator.c_str(), d.settings.solver.c_str(),
                d.settings.iterations, d.entities.size(), d.joints.size(), d.actuators.size(), o.steps, o.repeat,
                o.warmup, o.sampleEvery);

    const double simSeconds = static_cast<double>(o.steps) * d.timestep;
    for (const Mode mode : {Mode::Step, Mode::Snapshot, Mode::Jsonl}) {
        for (long w = 0; w < o.warmup; ++w) {
            if (runOnce(world, o, mode, sink, nullptr) < 0) {
                std::fprintf(stderr, "预热时仿真出错（模式 %s）\n", modeName(mode));
                return 5;
            }
        }
        std::vector<double> micros;
        micros.reserve(static_cast<std::size_t>(o.steps * o.repeat));
        std::vector<double> factors;
        for (long r = 0; r < o.repeat; ++r) {
            const double wall = runOnce(world, o, mode, sink, &micros);
            if (wall < 0) {
                std::fprintf(stderr, "仿真出错（模式 %s，第 %ld 轮）\n", modeName(mode), r);
                return 5;
            }
            factors.push_back(simSeconds / wall);
        }
        double sum = 0.0;
        for (const double m : micros) sum += m;
        std::sort(micros.begin(), micros.end());
        std::sort(factors.begin(), factors.end());
        std::printf("BENCH mode=%s mean_us=%.3f p50_us=%.3f p95_us=%.3f p99_us=%.3f max_us=%.3f "
                    "rtf_median=%.1f rtf_min=%.1f\n",
                    modeName(mode), sum / static_cast<double>(micros.size()), percentile(micros, 0.50),
                    percentile(micros, 0.95), percentile(micros, 0.99), micros.back(), percentile(factors, 0.5),
                    factors.front());
    }
    std::fclose(sink);
    return 0;
}
