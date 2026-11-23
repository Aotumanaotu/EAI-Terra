// 仿真时钟与快照插值（M2-06）的纯逻辑测试，不需要物理后端。
// 覆盖：各帧率下的步数累计与余量；单帧追赶上限、过载计数与丢弃时间；暂停与单步；速度倍率；异常的墙钟增量；
// 位置线性插值、四元数最短弧插值与归一化；跨世界实例、跨运行代次、实体数不同时不插值。

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "terra/simulation/clock.hpp"
#include "terra/simulation/interpolation.hpp"

namespace {

namespace sim = terra::simulation;
namespace td = terra::data;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

void testRates() {
    // 步长 0.002 s：1 秒墙钟在任意帧率下都应推进 500 步（允许最后一步落在下一帧，即 499 或 500）
    for (const int hz : {30, 60, 144, 240, 1000}) {
        sim::FixedStepClock clock(0.002);
        std::uint64_t total = 0;
        for (int f = 0; f < hz; ++f) {
            total += clock.advance(1.0 / hz);
            check(clock.alpha() >= 0.0 && clock.alpha() < 1.0, "余量比例在 [0, 1) 内");
        }
        check(total == 500 || total == 499, std::to_string(hz) + " Hz：1 秒推进 " + std::to_string(total) + " 步（应为 500）");
        total += clock.advance(0.001);
        check(total == 500, std::to_string(hz) + " Hz：再过 1 ms 后累计恰好 500 步");
        check(clock.overloadCount() == 0, std::to_string(hz) + " Hz：没有过载");
    }
    // 恰好 5 步：0.002 × 5 的累加舍入不应少算一步
    sim::FixedStepClock exact(0.002);
    check(exact.advance(0.01) == 5, "0.01 s 恰好 5 步（不因舍入少一步）");
    check(exact.alpha() < 1e-6, "整步时余量为 0");
}

void testCatchUp() {
    sim::FixedStepClock clock(0.002);  // 默认单帧最多 0.1 s = 50 步
    check(clock.maxStepsPerFrame() == 50, "单帧最多 50 步");
    check(clock.advance(1.0) == 50 && clock.lastAdvanceOverloaded(), "卡顿 1 秒：只推进 50 步并标记过载");
    check(clock.overloadCount() == 1 && std::fabs(clock.droppedSeconds() - 0.9) < 1e-9, "过载 1 次，丢弃 0.9 秒");
    check(clock.advance(0.002) == 1 && !clock.lastAdvanceOverloaded(), "之后正常推进，不补回丢弃的时间");

    sim::FixedStepClock slow(0.01, {0.005, 1.0});  // 上限小于一步：至少 1 步
    check(slow.maxStepsPerFrame() == 1 && slow.advance(0.05) == 1, "上限小于一步时每帧至少 1 步");
}

void testPauseAndStep() {
    sim::FixedStepClock clock(0.002);
    check(clock.advance(0.003) == 1, "运行中 3 ms → 1 步，余 1 ms");
    clock.pause();
    check(clock.paused() && clock.alpha() == 0.0, "暂停后余量清零");
    check(clock.advance(1.0) == 0 && clock.advance(0.5) == 0, "暂停中不推进");
    clock.requestSingleStep();
    check(clock.advance(0.0) == 1 && clock.advance(0.1) == 0, "暂停中单步：恰好 1 步，之后仍暂停");
    clock.resume();
    check(clock.advance(0.002) == 1, "继续后正常推进（暂停前的余量不保留）");
    clock.requestSingleStep();
    check(clock.advance(0.0) == 1, "运行中单步：本帧至少 1 步");

    sim::FixedStepClock fast(0.002, {0.1, 2.0});
    check(fast.advance(0.01) == 10, "2 倍速：10 ms 墙钟 → 10 步");
    fast.setSpeed(0.5);
    check(fast.advance(0.01) == 2 && fast.speed() == 0.5, "0.5 倍速：10 ms 墙钟 → 2 步余 1 ms");
    fast.setSpeed(-1);
    check(fast.speed() == 0.5, "非法速度被忽略");

    sim::FixedStepClock odd(0.002);
    check(odd.advance(-1.0) == 0 && odd.advance(std::nan("")) == 0 && odd.advance(INFINITY) == 0, "负数、NaN、无穷大按 0 处理");
    odd.advance(0.001);
    odd.reset();
    check(odd.alpha() == 0.0 && odd.advance(0.001) == 0, "reset 清空余量");
}

td::PhysicsSnapshot snapshotWith(std::uint64_t world, std::uint64_t epoch, td::Pose pose) {
    td::PhysicsSnapshot s;
    s.worldInstance = world;
    s.runEpoch = epoch;
    td::EntityState e;
    e.id = td::EntityId{1};
    e.pose = pose;
    s.entities.push_back(e);
    return s;
}

void testInterpolation() {
    const double h = std::sqrt(0.5);
    td::Pose a;
    td::Pose b;
    b.position = {4, -2, 1};
    b.orientation = {h, 0, 0, h};  // 绕 Z 90°
    const td::Pose m = sim::interpolate(a, b, 0.25);
    check(std::fabs(m.position.x - 1) < 1e-12 && std::fabs(m.position.y + 0.5) < 1e-12 && std::fabs(m.position.z - 0.25) < 1e-12,
          "位置线性插值");
    const td::Pose half = sim::interpolate(a, b, 0.5);
    const double angle = 2 * std::atan2(half.orientation.z, half.orientation.w) * 180 / 3.14159265358979323846;
    check(std::fabs(angle - 45) < 1e-9, "姿态中点为 45°，实际 " + std::to_string(angle));
    check(std::fabs(std::sqrt(half.orientation.w * half.orientation.w + half.orientation.z * half.orientation.z) - 1) < 1e-12,
          "插值结果归一化");
    td::Pose neg = b;
    neg.orientation = {-h, 0, 0, -h};  // 与 b 同一旋转
    const td::Pose viaNeg = sim::interpolate(a, neg, 0.5);
    check(std::fabs(std::fabs(viaNeg.orientation.z) - std::fabs(half.orientation.z)) < 1e-12 &&
              std::fabs(std::fabs(viaNeg.orientation.w) - std::fabs(half.orientation.w)) < 1e-12,
          "q 与 −q 走同一条最短弧");
    const td::Pose end0 = sim::interpolate(a, b, 0.0);
    const td::Pose end1 = sim::interpolate(a, b, 1.0);
    check(end0.position.x == 0 && end1.position.x == 4, "alpha 0 / 1 为两端");
    check(sim::interpolate(a, b, 7.0).position.x == 4, "alpha 超出范围时截断");

    const auto older = snapshotWith(1, 0, a);
    const auto newer = snapshotWith(1, 0, b);
    check(sim::interpolatable(older, newer), "同一世界、同一代次可以插值");
    check(std::fabs(sim::interpolatePoses(older, newer, 0.5)[0].position.x - 2) < 1e-12, "按实体插值");
    const auto afterReset = snapshotWith(1, 1, b);
    check(!sim::interpolatable(older, afterReset) && sim::interpolatePoses(older, afterReset, 0.5)[0].position.x == 4,
          "reset 之后（代次不同）不插值，直接用新快照");
    const auto otherWorld = snapshotWith(2, 0, b);
    check(!sim::interpolatable(older, otherWorld) && sim::interpolatePoses(older, otherWorld, 0.1)[0].position.x == 4,
          "不同世界实例不插值");
    auto bigger = newer;
    bigger.entities.push_back(bigger.entities[0]);
    check(!sim::interpolatable(older, bigger) && sim::interpolatePoses(older, bigger, 0.5).size() == 2, "实体数不同不插值");
}

}  // namespace

int main() {
    testRates();
    testCatchUp();
    testPauseAndStep();
    testInterpolation();
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
