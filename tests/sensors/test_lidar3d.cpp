// 3D 激光雷达（M5-11，simulation.lidar3d）。夹具 assets/tests/sensors/lidar3d_room.*：封闭房间、测试用的网格扫描图案。
//   - 场景文件：预设（livox_mid360）、自定义图案、每帧点数、量程、线数；imu: true 在预设的位置另装 IMU（200 Hz、ICM40609 的噪声）；
//     错误：没有预设、图案不存在、每帧点数超过图案长度；
//   - 几何：每个点都在房间的墙面上，距离与解析解一致；点的时间偏移、第一点时刻、线号；两次扫描逐位相同；
//   - 噪声：距离噪声的标准差与预设（0.2 m 处 3 cm、10 m 处 2 cm，线性插值）一致；方向噪声的标准差与设定一致；关闭时为真值；
//   - 运动畸变：匀速移动的雷达，朝运动方向的射线距离按每点时刻回推（与关闭时之差 = 速度 × 回推时间）；侧向不变；
//   - 官方 Mid-360 图案（已下载时）：每帧 20,000 点，按帧推进，4 s 一个循环，仰角在 −7.2°～52.2° 之内；记录单帧耗时；
//     地面圆环（夹具 lidar3d_ground*）：正装与倒置时盲区内边界与图案的解析值相同、前后左右对称；外观模型的位置（已下载时）。
// 统计容差按样本数的 4σ 左右取，冻结在本文件中。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/scene/pose_math.hpp"
#include "terra/scene/scene.hpp"
#include "terra/simulation/lidar3d.hpp"
#include "terra/simulation/sensor_noise.hpp"

#ifndef TERRA_ASSETS_DIR
#error "需要 TERRA_ASSETS_DIR"
#endif

namespace {

namespace sc = terra::scene;
namespace sim = terra::simulation;
namespace td = terra::data;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

const std::string kDir = std::string{TERRA_ASSETS_DIR} + "/tests/sensors";
constexpr double kPi = 3.14159265358979323846;

std::size_t lidarIndex(const sc::SceneInstance& s, const std::string& name) {
    for (std::size_t k = 0; k < s.lidars3d.size(); ++k) {
        if (s.lidars3d[k].name == name) return k;
    }
    return s.lidars3d.size();
}

// 房间内表面 x = ±3、y = ±2、z = 0、z = 3：从 o 沿 u 的射线到墙面的距离
double roomDistance(const td::Vec3& o, const td::Vec3& u) {
    double t = 1e9;
    const auto hit = [&](double origin, double dir, double plane) {
        if (std::fabs(dir) < 1e-12) return;
        const double s = (plane - origin) / dir;
        if (s > 0) t = std::min(t, s);
    };
    hit(o.x, u.x, 3.0);
    hit(o.x, u.x, -3.0);
    hit(o.y, u.y, 2.0);
    hit(o.y, u.y, -2.0);
    hit(o.z, u.z, 0.0);
    hit(o.z, u.z, 3.0);
    return t;
}

double norm(const sim::Lidar3dPoint& p) {
    return std::sqrt(static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y + static_cast<double>(p.z) * p.z);
}

struct Stats {
    double mean = 0.0, stddev = 0.0;
    std::size_t n = 0;
};
Stats stats(const std::vector<double>& v) {
    Stats s;
    s.n = v.size();
    if (v.size() < 2) return s;
    for (double x : v) s.mean += x;
    s.mean /= static_cast<double>(v.size());
    for (double x : v) s.stddev += (x - s.mean) * (x - s.mean);
    s.stddev = std::sqrt(s.stddev / static_cast<double>(v.size() - 1));
    return s;
}

}  // namespace

int main() {
    auto loaded = sc::loadScene(kDir + "/lidar3d_room.terra-scene.json", terra::physics::mujoco::composeWorld);
    for (const auto& i : loaded.issues) std::fprintf(stderr, "场景：%s %s\n", i.where.c_str(), i.message.c_str());
    if (!loaded.scene) return 1;
    auto& scene = *loaded.scene;
    auto& world = *scene.world;
    const auto& d = world.description();

    // ---- 场景文件 ----
    check(scene.lidars3d.size() == 6, "6 个 3D 雷达");
    const std::size_t k0 = lidarIndex(scene, "room/scan3d");
    if (k0 >= scene.lidars3d.size()) return 1;
    const auto& lidar = scene.lidars3d[k0];
    check(lidar.preset == "livox_mid360" && lidar.pointsPerFrame() == 540 && lidar.pattern && lidar.pattern->size() == 540 && lidar.rangeMin == 0.1 &&
              lidar.rangeMax == 40.0 && lidar.lines == 4 && lidar.rateHz == 10.0 && lidar.motionDistortion,
          "预设 + 自定义图案：每帧 540 点，量程 0.1～40 m，4 线，10 Hz");
    const td::SensorDescription* imu = d.findSensor("room/scan3d_imu");
    check(imu != nullptr, "imu: true：另装 IMU room/scan3d_imu");
    if (imu) {
        check(std::fabs(imu->mount.position.x - 0.011) < 1e-12 && std::fabs(imu->mount.position.y - 0.02329) < 1e-12 &&
                  std::fabs(imu->mount.position.z + 0.04412) < 1e-12,
              "IMU 的安装位置取预设（Mid-360 用户手册：11.0、23.29、−44.12 mm）");
        bool rate = false, density = false;
        for (const auto& c : scene.sensors) {
            if (c.sensor == imu->id) {
                rate = c.rateHz == 200.0;
                density = std::fabs(c.noise.gyro.noiseDensity - 7.854e-5) < 1e-12 && std::fabs(c.noise.accel.noiseDensity - 9.807e-4) < 1e-12;
            }
        }
        check(rate && density, "IMU 200 Hz，噪声取 ICM40609 的数据手册");
    }
    {
        const auto bad = [&](const std::string& sensor) {
            const std::string text = R"({"format": "terra.scene", "version": 1, "models": [{"name": "room", "mjcf": ")" + kDir +
                                     R"(/lidar3d_room.xml"}], "sensors": [)" + sensor + "]}";
            const auto parsed = sc::parseScene(text, kDir + "/x.terra-scene.json");
            if (!parsed.document) return std::string{parsed.issues.empty() ? "" : parsed.issues.front().message};
            const auto r = sc::instantiate(*parsed.document, terra::physics::mujoco::composeWorld);
            for (const auto& i : r.issues) {
                if (i.severity == sc::SceneIssue::Severity::Error) return i.message;
            }
            return std::string{};
        };
        check(bad(R"({"type": "lidar3d", "name": "room/a", "entity": "room/rig"})").find("preset") != std::string::npos, "没有 preset：报错");
        check(bad(R"({"type": "lidar3d", "name": "room/a", "entity": "room/rig", "preset": "velodyne"})").find("livox_mid360") != std::string::npos,
              "未知预设：列出可用的");
        check(bad(R"({"type": "lidar3d", "name": "room/a", "entity": "room/rig", "preset": "livox_mid360", "pattern": "none.csv"})")
                      .find("fetch_lidar_patterns") != std::string::npos,
              "图案不存在：给出下载方法");
        check(bad(R"({"type": "lidar3d", "name": "room/a", "entity": "room/rig", "preset": "livox_mid360", "pattern": "lidar3d_grid.csv", "pointsPerSecond": 100000})")
                      .find("每帧点数") != std::string::npos,
              "每帧点数超过图案长度：报错");
    }

    // ---- 几何 ----
    const auto scan = sim::scanLidar3d(world, lidar);
    check(scan.rays == 540 && scan.points.size() == 540, "封闭房间：540 条射线全部有回波（" + std::to_string(scan.points.size()) + "）");
    double worst = 0.0;
    bool timesOk = true, linesOk = true;
    for (const auto& p : scan.points) {
        const double r = norm(p);
        const td::Vec3 u{p.x / r, p.y / r, p.z / r};
        const td::Vec3 world_u = sc::rotate(scan.pose.orientation, u);
        worst = std::max(worst, std::fabs(r - roomDistance(scan.pose.position, world_u)));
        timesOk = timesOk && p.time == static_cast<float>(static_cast<double>(p.ray) / 5400.0);
        linesOk = linesOk && p.line == p.ray % 4;
    }
    check(worst < 1e-5, "距离与解析解一致（最大差 " + std::to_string(worst) + " m）");
    check(timesOk && std::fabs(scan.frameStart - (scan.time - 539.0 / 5400.0)) < 1e-12, "每点时间 = 序号 ÷ 点频；第一点时刻 = 当前 − 539 ÷ 点频");
    check(linesOk, "线号 = 图案序号对 4 取余");
    check(sim::lidar3dDigest(sim::scanLidar3d(world, lidar)) == sim::lidar3dDigest(scan), "两次扫描逐位相同");
    {
        // 每帧半个图案（270 个方向）：第 1 帧从第 270 个起，第 2 帧回到开头
        const auto& half = scene.lidars3d[lidarIndex(scene, "room/half")];
        const auto h0 = sim::scanLidar3d(world, half);
        for (int i = 0; i < 50; ++i) world.step();
        const auto h1 = sim::scanLidar3d(world, half);
        for (int i = 0; i < 50; ++i) world.step();
        const auto h2 = sim::scanLidar3d(world, half);
        check(h0.rays == 270 && h0.patternStart == 0 && h1.frame == 1 && h1.patternStart == 270 && h2.patternStart == 0 &&
                  sim::lidar3dDigest(h0) != sim::lidar3dDigest(h1) && sim::lidar3dDigest(h0) == sim::lidar3dDigest(h2),
              "按帧推进图案：每帧 270 个方向，第 1 帧从 270 起，第 2 帧循环回开头");
    }

    // ---- 噪声 ----
    sim::SensorNoise noise(scene, true);
    sim::SensorNoise off(scene, false);
    const std::size_t kr = lidarIndex(scene, "room/range_noise"), ka = lidarIndex(scene, "room/angle_noise");
    std::vector<double> normalized, dAz, dEl;
    bool offIsTruth = true;
    for (int frame = 0; frame < 60; ++frame) {
        for (int i = 0; i < 50; ++i) world.step();
        auto truth = sim::scanLidar3d(world, scene.lidars3d[kr]);
        auto noisy = truth;
        noise.lidar3d(kr, noisy);
        auto untouched = truth;
        off.lidar3d(kr, untouched);
        offIsTruth = offIsTruth && sim::lidar3dDigest(untouched) == sim::lidar3dDigest(truth);
        for (std::size_t i = 0; i < truth.points.size() && i < noisy.points.size(); ++i) {
            const double r = norm(truth.points[i]);
            const double t = std::clamp((r - 0.2) / (10.0 - 0.2), 0.0, 1.0);
            normalized.push_back((norm(noisy.points[i]) - r) / (0.03 + (0.02 - 0.03) * t));
        }
        auto angTruth = sim::scanLidar3d(world, scene.lidars3d[ka]);
        auto angNoisy = angTruth;
        noise.lidar3d(ka, angNoisy);
        for (std::size_t i = 0; i < angTruth.points.size() && i < angNoisy.points.size(); ++i) {
            const auto& a = angTruth.points[i];
            const auto& b = angNoisy.points[i];
            const double elA = std::asin(a.z / norm(a)), elB = std::asin(b.z / norm(b));
            if (std::fabs(elA) > 60.0 * kPi / 180.0) continue;  // 靠近天顶时方位角不稳定
            double daz = std::atan2(b.y, b.x) - std::atan2(a.y, a.x);
            if (daz > kPi) daz -= 2 * kPi;
            if (daz < -kPi) daz += 2 * kPi;
            dAz.push_back(daz * 180.0 / kPi);
            dEl.push_back((elB - elA) * 180.0 / kPi);
        }
    }
    const Stats rs = stats(normalized), as = stats(dAz), es = stats(dEl);
    std::printf("INFO 距离噪声：%zu 个样本，标准化后均值 %.4f、标准差 %.4f；方向噪声：方位 %.4f°、俯仰 %.4f°（设定 0.5°）\n", rs.n, rs.mean, rs.stddev,
                as.stddev, es.stddev);
    check(rs.n > 30000 && std::fabs(rs.mean) < 0.03 && std::fabs(rs.stddev - 1.0) < 0.03, "距离噪声：σ 按 0.2 m 处 3 cm、10 m 处 2 cm 插值");
    check(as.n > 10000 && std::fabs(as.stddev - 0.5) < 0.025 && std::fabs(es.stddev - 0.5) < 0.025, "方向噪声：方位与俯仰各 σ = 0.5°");
    check(offIsTruth, "噪声关闭：真值");

    // ---- 运动畸变 ----
    world.resetToKeyframe(static_cast<std::size_t>(d.findKeyframe("room/moving")));
    const auto& moving = scene.lidars3d[lidarIndex(scene, "room/moving")];
    const auto& still = scene.lidars3d[lidarIndex(scene, "room/still")];
    const auto a = sim::scanLidar3d(world, moving);
    const auto b = sim::scanLidar3d(world, still);
    // 天顶 90°、方位 0°（+X，运动方向）的射线：第 7 行第 0 个，序号 252；方位 90°（+Y，侧向）：序号 261
    const auto find = [](const sim::Lidar3dScan& s, std::uint32_t ray) -> const sim::Lidar3dPoint* {
        for (const auto& p : s.points) {
            if (p.ray == ray) return &p;
        }
        return nullptr;
    };
    const auto* fa = find(a, 252);
    const auto* fb = find(b, 252);
    const auto* sa = find(a, 261);
    const auto* sb = find(b, 261);
    if (fa && fb && sa && sb) {
        const std::size_t begin = 250;  // 每组 5 条（1 ms × 5400 点/秒）：射线 252 在第 250～254 组
        const double tau = static_cast<double>(begin + begin + 4) * 0.5 / 5400.0;
        const double back = 539.0 / 5400.0 - tau;
        const double diff = norm(*fa) - norm(*fb);
        std::printf("INFO 运动畸变：朝运动方向的射线距离多 %.6f m（速度 1 m/s × 回推 %.6f s）\n", diff, back);
        check(std::fabs(diff - back) < 1e-5, "运动畸变：朝运动方向的距离 = 当前距离 + 速度 × 回推时间");
        check(std::fabs(norm(*sa) - norm(*sb)) < 1e-5, "运动畸变：侧向的射线距离不变");
    } else {
        check(false, "运动畸变：找不到射线 252 / 261");
    }
    check(sim::lidar3dDigest(a) != sim::lidar3dDigest(b), "开启与关闭运动畸变的点云不同");

    // ---- 官方 Mid-360 图案（已下载时）----
    const std::string pattern = std::string{TERRA_ASSETS_DIR} + "/sensors/livox/mid360.csv";
    if (!std::filesystem::exists(pattern)) {
        std::printf("SKIP Mid-360 官方图案：没有 %s（运行 scripts/assets/fetch_lidar_patterns.sh）\n", pattern.c_str());
    } else {
        auto m = sc::loadScene(kDir + "/lidar3d_mid360.terra-scene.json", terra::physics::mujoco::composeWorld);
        if (!m.scene) {
            check(false, "Mid-360 场景无法加载");
        } else {
            auto& w = *m.scene->world;
            const auto& l = m.scene->lidars3d[0];
            check(l.pattern->size() == 800000 && l.pointsPerFrame() == 20000, "Mid-360：图案 800,000 个方向（4 s），每帧 20,000 点");
            double elMin = 90.0, elMax = -90.0;
            for (std::size_t i = 0; i < l.pattern->size(); ++i) {
                const double el = 90.0 - static_cast<double>(l.pattern->zenith[i]);
                elMin = std::min(elMin, el);
                elMax = std::max(elMax, el);
            }
            check(elMin > -7.3 && elMax < 52.3 && elMin < -7.0 && elMax > 52.0, "仰角 " + std::to_string(elMin) + "°～" + std::to_string(elMax) + "°，与规格 −7°～52° 一致");
            const auto begin = std::chrono::steady_clock::now();
            const auto f0 = sim::scanLidar3d(w, l);
            const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
            std::printf("INFO Mid-360 一帧：%zu 条射线、%zu 个点，耗时 %.2f ms\n", f0.rays, f0.points.size(), ms);
            check(f0.rays == 20000 && f0.points.size() == 20000 && f0.patternStart == 0, "第 0 帧：从图案开头取 20,000 条，封闭房间全部有回波");
            for (int i = 0; i < 50; ++i) w.step();
            const auto f1 = sim::scanLidar3d(w, l);
            check(f1.frame == 1 && f1.patternStart == 20000, "0.1 s 后第 1 帧：从第 20,000 个方向开始");
            for (int i = 0; i < 1950; ++i) w.step();
            const auto f40 = sim::scanLidar3d(w, l);
            check(f40.frame == 40 && f40.patternStart == 0 && sim::lidar3dDigest(f40) == sim::lidar3dDigest(f0), "4 s 后第 40 帧：图案循环，静止场景中点云与第 0 帧相同");
        }

        // 地面圆环：装在 0.4 m 高处，4 s（40 帧，整个图案）中每 30° 方位扇区地面点的最小水平距离（盲区内边界），
        // 应等于该扇区图案边缘仰角的解析值（0.4 ÷ tan）。官方图案的边缘随方位变化：下边缘左右（±90°）−7.2°、前后（0°、180°）−4.9°，
        // 上边缘 52.2° 与 48.3°。
        //   - 正装：地面在安装坐标系 z = −0.4 处，由下边缘决定，盲区前后长、左右短（约 3.94 m 与 3.16 m），不是正圆；
        //   - 倒置（绕 X 轴转 180°）：地面在 z = +0.4 处，由上边缘决定，盲区只有 0.31～0.34 m，大部分射线打到地面，中间近似圆形。
        // 检查 Terra 与图案一致、相对的扇区相同（不偏心）
        struct GroundCase {
            const char* file;
            const char* label;
            double side;  // 地面在安装坐标系中的 z 符号：正装 −1，倒置 +1
            double innerMin, innerMax, outerMin;
        };
        for (const GroundCase& gc : {GroundCase{"lidar3d_ground.terra-scene.json", "正装", -1.0, 3.1, 4.0, 39.0},
                                     GroundCase{"lidar3d_ground_inverted.terra-scene.json", "倒置", 1.0, 0.30, 0.35, 39.0}}) {
            auto g = sc::loadScene(kDir + "/" + gc.file, terra::physics::mujoco::composeWorld);
            const std::string tag = std::string{"地面圆环（"} + gc.label + "）";
            if (!g.scene) {
                check(false, tag + "：场景无法加载");
                continue;
            }
            auto& w = *g.scene->world;
            const auto& l = g.scene->lidars3d[0];
            constexpr double kHeight = 0.4;
            std::vector<double> measured(12, 1e9), expected(12, 1e9);
            double outer = 0.0;
            std::size_t ground = 0, total = 0;
            for (std::size_t i = 0; i < l.pattern->size(); ++i) {
                const double el = (90.0 - static_cast<double>(l.pattern->zenith[i])) * kPi / 180.0;
                const auto sector = static_cast<std::size_t>(std::fmod(static_cast<double>(l.pattern->azimuth[i]), 360.0) / 30.0) % 12;
                if (el * gc.side > 0) expected[sector] = std::min(expected[sector], kHeight / std::tan(std::fabs(el)));
            }
            for (int frame = 0; frame < 40; ++frame) {
                if (frame > 0) {
                    for (int i = 0; i < 50; ++i) w.step();
                }
                const auto frameScan = sim::scanLidar3d(w, l);
                total += frameScan.rays;
                for (const auto& p : frameScan.points) {
                    if (std::fabs(static_cast<double>(p.z) - gc.side * kHeight) > 1e-3) continue;
                    ++ground;
                    const double r = std::hypot(static_cast<double>(p.x), static_cast<double>(p.y));
                    double az = std::atan2(static_cast<double>(p.y), static_cast<double>(p.x)) * 180.0 / kPi;
                    if (az < 0) az += 360.0;
                    const auto sector = static_cast<std::size_t>(az / 30.0) % 12;
                    measured[sector] = std::min(measured[sector], r);
                    outer = std::max(outer, r);
                }
            }
            const auto [lo, hi] = std::minmax_element(measured.begin(), measured.end());
            bool exact = true;
            std::string text;
            for (std::size_t k = 0; k < 12; ++k) {
                exact = exact && std::fabs(measured[k] - expected[k]) < 2e-3 * expected[k];
                char buf[32];
                std::snprintf(buf, sizeof buf, "%s%.2f", k == 0 ? "" : " ", measured[k]);
                text += buf;
            }
            std::printf("INFO %s（0.4 m 高，40 帧）：%zu / %zu 条射线打到地面；每 30° 扇区的内半径 %s m（最大 / 最小 = %.4f），外半径 %.2f m\n",
                        tag.c_str(), ground, total, text.c_str(), *hi / *lo, outer);
            check(exact, tag + "：每个扇区的内半径与图案边缘仰角的解析值一致（0.2% 以内）");
            check(*lo > gc.innerMin && *hi < gc.innerMax, tag + "：内半径在预期范围内");
            bool symmetric = true;
            for (std::size_t k = 0; k < 6; ++k) symmetric = symmetric && std::fabs(measured[k] - measured[k + 6]) < 0.01 * measured[k];
            check(symmetric, tag + "：相对的扇区内半径相同（不偏心）");
            check(outer > gc.outerMin && outer < 40.0, tag + "：外半径到量程 40 m");
            if (gc.side > 0) check(ground > total * 8 / 10, tag + "：八成以上的射线打到地面（视场朝下）");
            // 外观模型（已下载时）：作为 glTF 绑定跟随安装实体，位于雷达原点下方 47 mm（倒置时在上方）
            const auto* binding = g.scene->bindings.empty() ? nullptr : &g.scene->bindings.back();
            if (binding != nullptr && binding->spec.sensorModel) {
                const auto local = binding->spec.local;
                check(std::fabs(local.position.x) < 1e-9 && std::fabs(local.position.y) < 1e-9 && std::fabs(local.position.z - gc.side * 0.047) < 1e-9,
                      tag + "：外观模型的底面在雷达原点下方 47 mm（相对支架：正装 z = −0.047，倒置 z = +0.047）");
            }
        }
    }

    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
