// 2D 激光雷达（M4-03b）：夹具 assets/tests/sensors/lidar_room.*（见 MJCF 注释），与解析解比较。
//   - 安装位姿：雷达在 (0.5, 0, 0.3)，偏航 90°（雷达的 0° 指向世界 +Y）；第 i 束角度 = −180° + i°；
//   - collision 模式：每一束的距离与三面墙的解析距离一致（< 1e-5 m）；朝敞开一侧的射线为 +inf；
//     穿过只用于显示的柱子；不打到安装它的实体（外壳把雷达原点包在里面）；
//   - visual 模式：朝柱子的那一束打到柱面（解析的射线—圆求交），其余与 collision 模式相同；
//   - 最小量程 2.1 m：比它近的读数为 −inf，其余不变；最大量程 1 m：全部为 +inf；
//   - 扫描的步序号、时间与位姿取自当前快照；同一状态扫描两次逐位相同；
//   - 场景文件的雷达：默认值、频率上限、解析错误与组合错误。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/scene/pose_math.hpp"
#include "terra/scene/scene.hpp"
#include "terra/simulation/lidar.hpp"

#ifndef TERRA_TEST_ASSETS_DIR
#error "需要 TERRA_TEST_ASSETS_DIR"
#endif

namespace {

namespace sc = terra::scene;
namespace sim = terra::simulation;
namespace td = terra::data;
using Severity = sc::SceneIssue::Severity;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

std::string dump(const std::vector<sc::SceneIssue>& issues) {
    std::string text;
    for (const auto& i : issues) text += std::string{"\n  "} + (i.severity == Severity::Error ? "错误 " : "警告 ") + i.where + "：" + i.message;
    return text;
}

bool hasIssue(const std::vector<sc::SceneIssue>& issues, Severity severity, const std::string& where, const std::string& words) {
    for (const auto& i : issues) {
        if (i.severity == severity && (where.empty() || i.where == where) && i.message.find(words) != std::string::npos) return true;
    }
    return false;
}

const std::string kDir = std::string{TERRA_TEST_ASSETS_DIR} + "/sensors";
constexpr double kPi = 3.14159265358979323846;
constexpr double kInf = std::numeric_limits<double>::infinity();

// 从 (ox, oy) 沿世界角 phi 到三面墙内表面的距离；没有命中为 +inf
double wallDistance(double ox, double oy, double phi) {
    const double dx = std::cos(phi);
    const double dy = std::sin(phi);
    double best = kInf;
    const auto consider = [&](double t, double along, double limit) {
        if (t > 0 && std::fabs(along) <= limit && t < best) best = t;
    };
    if (dx > 1e-12) consider((2.0 - ox) / dx, oy + (2.0 - ox) / dx * dy, 2.1);
    if (dx < -1e-12) consider((-2.0 - ox) / dx, oy + (-2.0 - ox) / dx * dy, 2.1);
    if (dy < -1e-12) consider((-2.0 - oy) / dy, ox + (-2.0 - oy) / dy * dx, 2.1);
    return best;
}

// 射线与圆（柱子）求交；不相交为 +inf
double circleDistance(double ox, double oy, double phi, double cx, double cy, double r) {
    const double dx = std::cos(phi);
    const double dy = std::sin(phi);
    const double along = (cx - ox) * dx + (cy - oy) * dy;
    const double px = ox + along * dx - cx;
    const double py = oy + along * dy - cy;
    const double h2 = r * r - (px * px + py * py);
    if (along <= 0 || h2 < 0) return kInf;
    return along - std::sqrt(h2);
}

// 夹具中第 i 束的角度：−180° + i°（独立于被测的 lidarBeamAngle 计算）
double beamAngle(int i) { return (-180.0 + static_cast<double>(i)) * kPi / 180.0; }

bool same(float actual, double expected, double tol) {
    if (std::isinf(expected)) return std::isinf(actual) && (actual > 0) == (expected > 0);
    return std::isfinite(actual) && std::fabs(static_cast<double>(actual) - expected) <= tol;
}

void room() {
    auto loaded = sc::loadScene(kDir + "/lidar_room.terra-scene.json", terra::physics::mujoco::composeWorld);
    check(loaded.ok() && loaded.scene->lidars.size() == 2, "加载雷达夹具" + dump(loaded.issues));
    if (!loaded.ok() || loaded.scene->lidars.size() != 2) return;
    auto& scene = *loaded.scene;
    const sc::LidarSensor& lidar = scene.lidars[0];
    check(lidar.samples == 360 && lidar.rateHz == 10.0 && !lidar.hitVisual && std::fabs(lidar.angleMin + kPi) < 1e-12 &&
              std::fabs(lidar.angleMax - 179.0 * kPi / 180.0) < 1e-12,
          "bench/scan：360 束，−180°～179°，10 Hz，打参与碰撞的几何体");
    check(scene.lidars[1].hitVisual && scene.lidars[1].rateHz == 10.0 && scene.lidars[1].rangeMin == 0.15 && scene.lidars[1].rangeMax == 12.0,
          "bench/scan_visual：visual 模式；默认 10 Hz、0.15～12 m");
    for (int i = 0; i < 7; ++i) scene.world->step();

    const sim::LidarScan scan = sim::scanLidar(*scene.world, lidar);
    const auto snap = scene.world->snapshot();
    const td::Pose expectedPose = sim::lidarWorldPose(*snap, lidar);
    check(scan.step == 7 && scan.time == snap->simulationTime && scan.sensor == "bench/scan", "扫描的步序号（7）、时间、名字");
    check(std::fabs(scan.pose.position.x - 0.5) < 1e-12 && std::fabs(scan.pose.position.y) < 1e-12 && std::fabs(scan.pose.position.z - 0.3) < 1e-12 &&
              scan.pose.position.x == expectedPose.position.x && scan.pose.orientation.w == expectedPose.orientation.w,
          "雷达在 (0.5, 0, 0.3)，与 lidarWorldPose 一致");
    check(scan.ranges.size() == 360 && std::fabs(scan.angleIncrement - kPi / 180.0) < 1e-12 && scan.rangeMin == 0.15 && scan.rangeMax == 12.0,
          "360 个读数，角度增量 1°");
    check(std::fabs(sim::lidarBeamAngle(lidar, 0) + kPi) < 1e-12 && std::fabs(sim::lidarBeamAngle(lidar, 359) - 179.0 * kPi / 180.0) < 1e-12,
          "第一束在 −180°，最后一束在 179°");
    if (scan.ranges.size() != 360) return;

    int bad = 0;
    int openBeams = 0;
    std::string firstBad;
    for (int i = 0; i < 360; ++i) {
        const double phi = beamAngle(i) + kPi / 2.0;  // 偏航 90°
        const double expected = wallDistance(0.5, 0.0, phi);
        if (std::isinf(expected)) ++openBeams;
        if (!same(scan.ranges[static_cast<std::size_t>(i)], expected, 1e-5)) {
            if (bad++ == 0) firstBad = "第 " + std::to_string(i) + " 束：" + std::to_string(scan.ranges[static_cast<std::size_t>(i)]) + "，应为 " +
                                       std::to_string(expected);
        }
    }
    std::printf("INFO 360 束中 %d 束朝敞开一侧（+inf）\n", openBeams);
    check(bad == 0, "collision 模式：每一束与三面墙的解析距离一致（< 1e-5 m），朝敞开一侧为 +inf；穿过柱子；不打到外壳" +
                        (bad > 0 ? "（" + std::to_string(bad) + " 束不符，" + firstBad + "）" : std::string{}));
    check(openBeams > 20 && openBeams < 180, "有一部分射线朝敞开一侧");

    // visual 模式：柱子
    const sim::LidarScan visual = sim::scanLidar(*scene.world, scene.lidars[1]);
    int pillarBeams = 0;
    int visualBad = 0;
    for (int i = 0; i < 360 && visual.ranges.size() == 360; ++i) {
        const double phi = beamAngle(i) + kPi / 2.0;
        const double toPillar = circleDistance(0.5, 0.0, phi, -1.0, -1.0, 0.1);
        const double expected = std::min(toPillar, wallDistance(0.5, 0.0, phi));
        if (std::isfinite(toPillar)) ++pillarBeams;
        visualBad += same(visual.ranges[static_cast<std::size_t>(i)], expected, 1e-5) ? 0 : 1;
    }
    check(pillarBeams >= 3 && visualBad == 0, "visual 模式：" + std::to_string(pillarBeams) + " 束打到柱子（解析的射线—圆求交），其余与墙一致" +
                                                  (visualBad > 0 ? "（" + std::to_string(visualBad) + " 束不符）" : std::string{}));

    // 最小量程
    sc::LidarSensor nearLimit = lidar;
    nearLimit.rangeMin = 2.1;
    const sim::LidarScan clipped = sim::scanLidar(*scene.world, nearLimit);
    int clippedBad = 0;
    int tooClose = 0;
    for (int i = 0; i < 360 && clipped.ranges.size() == 360; ++i) {
        const double expected = wallDistance(0.5, 0.0, beamAngle(i) + kPi / 2.0);
        const double want = expected < 2.1 ? -kInf : expected;
        tooClose += want < 0 ? 1 : 0;
        clippedBad += same(clipped.ranges[static_cast<std::size_t>(i)], want, 1e-5) ? 0 : 1;
    }
    check(tooClose > 0 && clippedBad == 0, "最小量程 2.1 m：比它近的 " + std::to_string(tooClose) + " 束为 −inf，其余不变");

    // 最大量程：1 m 内没有任何东西（墙至少 1.5 m 远，外壳被忽略），全部为 +inf
    sc::LidarSensor shortRange = lidar;
    shortRange.rangeMax = 1.0;
    const sim::LidarScan empty = sim::scanLidar(*scene.world, shortRange);
    bool allInf = empty.ranges.size() == 360;
    for (const float r : empty.ranges) allInf = allInf && std::isinf(r) && r > 0;
    check(allInf, "最大量程 1 m：全部为 +inf");

    const sim::LidarScan again = sim::scanLidar(*scene.world, lidar);
    check(again.ranges == scan.ranges, "同一状态扫描两次逐位相同");
}

sc::SceneLoadResult load(const std::string& sensors) {
    const std::string text =
        R"({"format": "terra.scene", "version": 1, "models": [{"name": "bench", "mjcf": "lidar_room.xml"}], "sensors": )" + sensors + "}";
    auto parsed = sc::parseScene(text, kDir + "/inline.terra-scene.json");
    if (!parsed.document) {
        sc::SceneLoadResult out;
        out.issues = parsed.issues;
        return out;
    }
    return sc::instantiate(std::move(*parsed.document), terra::physics::mujoco::composeWorld);
}

void sceneFile() {
    auto defaults = load(R"([{"type": "lidar2d", "name": "bench/a", "entity": "bench/base"},
                             {"type": "lidar2d", "name": "fixed", "entity": "world", "position": [0, 0, 0.3], "rateHz": 900}])");
    check(defaults.ok() && defaults.scene->lidars.size() == 2, "默认值场景可以加载" + dump(defaults.issues));
    if (defaults.ok() && defaults.scene->lidars.size() == 2) {
        const auto& a = defaults.scene->lidars[0];
        check(a.samples == 360 && std::fabs(a.angleMin + kPi) < 1e-12 && std::fabs(a.angleMax - kPi) < 1e-12 && a.rangeMin == 0.15 &&
                  a.rangeMax == 12.0 && a.rateHz == 10.0,
              "默认：360 束、−180°～180°、0.15～12 m、10 Hz");
        const auto& w = defaults.scene->lidars[1];
        check(!w.entity.valid() && std::fabs(w.rateHz - 500.0) < 1e-9 && hasIssue(defaults.issues, Severity::Warning, "/sensors/1/rateHz", "高于物理频率"),
              "装在世界上（名字不需要前缀）；900 Hz 按物理频率并警告");
    }
    struct Case {
        const char* sensors;
        const char* where;
        const char* words;
    };
    const Case cases[] = {
        {R"([{"type": "lidar2d", "name": "bench/a"}])", "/sensors/0", "需要 entity"},
        {R"([{"type": "lidar2d", "entity": "bench/base"}])", "/sensors/0", "需要 name"},
        {R"([{"type": "lidar2d", "name": "bench/a", "entity": "bench/base", "angleMinDeg": 10, "angleMaxDeg": 10}])", "/sensors/0/angleMaxDeg", "角度范围"},
        {R"([{"type": "lidar2d", "name": "bench/a", "entity": "bench/base", "rangeMin": 5, "rangeMax": 4}])", "/sensors/0/rangeMax", "大于最小量程"},
        {R"([{"type": "lidar2d", "name": "bench/a", "entity": "bench/base", "samples": 0}])", "/sensors/0/samples", "1 到 100000"},
        {R"([{"type": "lidar2d", "name": "bench/a", "entity": "bench/base", "hits": "all"}])", "/sensors/0/hits", "collision"},
        {R"([{"type": "lidar2d", "name": "bench/a", "entity": "bench/nobody"}])", "/sensors/0/entity", "找不到实体"},
        {R"([{"type": "lidar2d", "name": "other/a", "entity": "bench/base"}])", "/sensors/0/name", "前缀"},
    };
    for (const Case& c : cases) {
        const auto loaded = load(c.sensors);
        check(!loaded.ok() && hasIssue(loaded.issues, Severity::Error, c.where, c.words),
              std::string{"错误 "} + c.sensors + " → " + c.where + "“" + c.words + "”" + dump(loaded.issues));
    }
}

}  // namespace

int main() {
    room();
    sceneFile();
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
