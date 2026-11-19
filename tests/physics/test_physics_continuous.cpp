// M0-08 连续运行（开发计划 4.3 节）：主夹具运行 60 s 仿真时间。
// 判定：无仿真错误、全程有限、四元数范数误差与时间误差在门槛内、始终保持静止接触后的稳定。
// 内存：每 10 s 仿真时间记录一次常驻内存（Linux /proc/self/statm），只记录走势，不作判定；
// 不能凭一次观察宣称没有泄漏（泄漏由 ASan/LeakSanitizer 配置检查）。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <unistd.h>

#include "m0_tolerances.hpp"
#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/physics/physics_world.hpp"

#ifndef TERRA_TEST_ASSETS_DIR
#error "需要 TERRA_TEST_ASSETS_DIR"
#endif

namespace {

namespace m0 = terra::test::m0;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

// 常驻内存（KiB）；读取失败返回 -1。
long residentKiB() {
    std::ifstream statm{"/proc/self/statm"};
    long size = 0;
    long resident = 0;
    if (!(statm >> size >> resident)) return -1;
    return resident * (sysconf(_SC_PAGESIZE) / 1024);
}

}  // namespace

int main() {
    auto r = terra::physics::mujoco::loadWorld(
        terra::data::ModelSource{std::string{TERRA_TEST_ASSETS_DIR} + "/falling_cube.xml"});
    if (!r.world) {
        std::fprintf(stderr, "FAILED: 加载 falling_cube.xml：%s\n", r.error ? r.error->message.c_str() : "");
        return 1;
    }
    auto& world = *r.world;
    const double dt = world.description().timestep;
    const long rssStart = residentKiB();
    std::printf("MEMORY falling_cube t=0s rss=%ldKiB\n", rssStart);

    double maxQuatError = 0.0;
    double maxTimeError = 0.0;
    bool allFinite = true;
    for (int k = 1; k <= m0::kContinuousSteps; ++k) {
        if (const auto error = world.step()) {
            check(false, "falling_cube：第 " + std::to_string(error->step) + " 步仿真错误：" + error->message);
            break;
        }
        const auto s = world.snapshot();
        const auto& q = s->entities[0].pose.orientation;
        maxQuatError = std::max(maxQuatError, std::fabs(std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z) - 1.0));
        maxTimeError = std::max(maxTimeError, std::fabs(s->simulationTime - static_cast<double>(k) * dt));
        if (!s->diagnostics.finite && allFinite) {
            allFinite = false;
            check(false, "falling_cube：第 " + std::to_string(k) + " 步出现非有限值（diagnostics.finite）");
        }
        if (k % 10000 == 0) {
            std::printf("MEMORY falling_cube t=%ds rss=%ldKiB\n", k / 1000, residentKiB());
        }
    }
    const auto last = world.snapshot();
    std::printf("METRIC continuous 步数 实测 %llu 门槛 %d %s\n", static_cast<unsigned long long>(last->stepIndex),
                m0::kContinuousSteps, last->stepIndex == static_cast<std::uint64_t>(m0::kContinuousSteps) ? "PASS" : "FAIL");
    check(last->stepIndex == static_cast<std::uint64_t>(m0::kContinuousSteps), "falling_cube：完成 60000 步");
    std::printf("METRIC continuous 时间误差(s) 实测 %.6g 门槛 %.6g %s\n", maxTimeError, m0::kTimeError,
                maxTimeError <= m0::kTimeError ? "PASS" : "FAIL");
    check(maxTimeError <= m0::kTimeError, "falling_cube：60 s 时间误差");
    std::printf("METRIC continuous 四元数范数误差 实测 %.6g 门槛 %.6g %s\n", maxQuatError, m0::kQuaternionNormError,
                maxQuatError <= m0::kQuaternionNormError ? "PASS" : "FAIL");
    check(maxQuatError <= m0::kQuaternionNormError, "falling_cube：60 s 四元数范数误差");
    const double z = last->entities[0].pose.position.z;
    std::printf("METRIC continuous 最终高度误差(m) 实测 %.6g 门槛 %.6g %s\n", std::fabs(z - m0::kRestHeight),
                m0::kRestHeightError, std::fabs(z - m0::kRestHeight) <= m0::kRestHeightError ? "PASS" : "FAIL");
    check(std::fabs(z - m0::kRestHeight) <= m0::kRestHeightError, "falling_cube：60 s 后仍静止在地面上");
    std::printf("MEMORY falling_cube 增长 %ldKiB（仅记录，不判定）\n", residentKiB() - rssStart);

    if (failures == 0) {
        std::printf("all passed\n");
    }
    return failures == 0 ? 0 : 1;
}
