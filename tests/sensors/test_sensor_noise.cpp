// 传感器噪声（M4-09，simulation.sensor_noise）：夹具 assets/tests/sensors/noise_suite.terra-scene.json。
//   - 计数器式随机数：同输入相同、换种子或步序号不同；标准正态的均值与标准差；
//   - 场景文件：噪声参数、种子、雷达预设（rplidar_a1m8 按数据手册换算）、显式字段覆盖预设、解析错误；
//   - IMU：白噪声标准差 = 密度 × √频率；随机游走的方差随时间线性增长；初始零偏（固定值与随机）；
//     各自持有的 SensorNoise 逐位相同、与调用顺序无关；步序号变小后从初始状态重来；没有配置噪声的 IMU 不变；
//   - 关闭：运行时开关、场景的 noise.enabled、传感器的 noise.enabled 三个层次；
//   - 2D 激光雷达：丢失率、量化、距离噪声的标准差；没有命中的束保持 inf；
//   - 深度：σ = a + b·z²、丢失；彩色：每通道 σ，截断到 0～255，不改透明度。
// 统计容差按样本数的 4σ 左右取，冻结在本文件中。

#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/scene/scene.hpp"
#include "terra/simulation/lidar.hpp"
#include "terra/simulation/sensor_noise.hpp"
#include "terra/simulation/sensor_sampling.hpp"

#ifndef TERRA_TEST_ASSETS_DIR
#error "需要 TERRA_TEST_ASSETS_DIR"
#endif

namespace {

namespace sc = terra::scene;
namespace sim = terra::simulation;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

struct Stats {
    double mean = 0.0;
    double stddev = 0.0;
};

Stats stats(const std::vector<double>& v) {
    Stats s;
    for (const double x : v) s.mean += x;
    s.mean /= static_cast<double>(v.size());
    for (const double x : v) s.stddev += (x - s.mean) * (x - s.mean);
    s.stddev = std::sqrt(s.stddev / static_cast<double>(v.size() - 1));
    return s;
}

std::size_t sensorIndex(const sc::SceneInstance& scene, const std::string& name) {
    const auto& sensors = scene.world->description().sensors;
    for (std::size_t k = 0; k < sensors.size(); ++k) {
        if (sensors[k].name == name) return k;
    }
    return sensors.size();
}

// 采样步：给定频率下第 n 个采样所在的步序号
std::vector<std::uint64_t> sampleSteps(double timestep, double rate, std::size_t count) {
    std::vector<std::uint64_t> out;
    for (std::uint64_t t = 0; out.size() < count; ++t) {
        if (sim::sensorSampleDue(t, timestep, rate)) out.push_back(t);
    }
    return out;
}

}  // namespace

int main() {
    const std::string dir = TERRA_TEST_ASSETS_DIR "/sensors";

    // 随机数
    check(sim::noiseBits(1, 2, 3, 4) == sim::noiseBits(1, 2, 3, 4) && sim::noiseBits(1, 2, 3, 4) != sim::noiseBits(2, 2, 3, 4) &&
              sim::noiseBits(1, 2, 3, 4) != sim::noiseBits(1, 2, 4, 4) && sim::noiseBits(1, 2, 3, 4) != sim::noiseBits(1, 2, 3, 5),
          "计数器式随机数：同输入相同，种子、步序号、分量不同则不同");
    {
        std::vector<double> g;
        for (std::uint64_t i = 0; i < 1000000; ++i) g.push_back(sim::noiseGaussian(7, 11, i, 0));
        const auto s = stats(g);
        std::printf("INFO 标准正态 10⁶ 个：均值 %.5f，标准差 %.5f\n", s.mean, s.stddev);
        check(std::fabs(s.mean) < 0.005 && std::fabs(s.stddev - 1.0) < 0.005, "标准正态：均值 ≈ 0、标准差 ≈ 1");
    }

    auto loaded = sc::loadScene(dir + "/noise_suite.terra-scene.json", terra::physics::mujoco::composeWorld);
    for (const auto& i : loaded.issues) std::printf("INFO 场景：%s %s\n", i.where.c_str(), i.message.c_str());
    check(loaded.ok(), "夹具可以加载");
    if (!loaded.ok()) return 1;
    auto& scene = *loaded.scene;
    const double dt = scene.world->description().timestep;
    const std::size_t level = sensorIndex(scene, "bench/level_imu");
    const std::size_t tilt = sensorIndex(scene, "bench/tilt_imu");
    check(level < scene.sensors.size() && tilt < scene.sensors.size(), "IMU 存在");
    if (level >= scene.sensors.size() || tilt >= scene.sensors.size()) return 1;

    // 场景文件
    const auto& ln = scene.sensors[level].noise;
    check(scene.document.noise.seed == 42 && scene.document.noise.enabled, "场景级：种子 42，开启");
    check(ln.configured && ln.gyro.noiseDensity == 0.01 && ln.gyro.randomWalk == 0.001 && ln.gyro.bias[0] == 0.1 && ln.accel.biasStd == 0.05,
          "IMU 噪声参数");
    check(!scene.sensors[tilt].noise.configured, "没有写噪声的 IMU");
    check(scene.lidars.size() == 2 && scene.lidars[0].noise.rangeStd == 0.01 && scene.lidars[0].noise.dropout == 0.05 &&
              scene.lidars[1].noise.preset == "rplidar_a1m8" && scene.lidars[1].noise.resolution == 0.0005 &&
              std::fabs(scene.lidars[1].noise.rangeStdRelative - 0.003333) < 1e-9,
          "雷达噪声；预设 rplidar_a1m8（分辨率 0.5 mm，相对 σ = 1% / 3）");
    check(scene.cameras.size() == 1 && scene.cameras[0].segmentation && scene.cameras[0].noise.pixelStd == 3, "相机噪声与分割图开关");
    check(scene.document.labels.size() == 2, "语义类别");
    {
        const auto overridden = sc::parseScene(
            R"({"format": "terra.scene", "version": 1, "models": [{"name": "room", "mjcf": "x.xml"}], "sensors": [
                {"type": "lidar2d", "name": "room/scan", "entity": "room/base", "noise": {"preset": "hokuyo_ust10lx", "rangeStd": 0.02}}]})",
            dir + "/x.terra-scene.json");
        check(overridden.document && overridden.document->sensors[0].lidarNoise.rangeStd == 0.02, "显式字段覆盖预设");
        const auto expectError = [&](const std::string& sensor, const std::string& part, const std::string& what) {
            const auto r = sc::parseScene(R"({"format": "terra.scene", "version": 1, "models": [{"name": "room", "mjcf": "x.xml"}], "sensors": [)" + sensor +
                                              "]}",
                                          dir + "/x.terra-scene.json");
            bool found = false;
            for (const auto& i : r.issues) found = found || (i.severity == sc::SceneIssue::Severity::Error && i.message.find(part) != std::string::npos);
            check(!r.document && found, what);
        };
        expectError(R"({"type": "lidar2d", "name": "room/s", "entity": "room/base", "noise": {"preset": "nope"}})", "没有雷达预设", "未知预设：列出可用的");
        expectError(R"({"type": "lidar2d", "name": "room/s", "entity": "room/base", "noise": {"dropout": 1.5}})", "应在 0 到", "丢失率超过 1");
        expectError(R"({"type": "imu", "site": "room/x", "noise": {"gyro": {"noiseDensity": -1}}})", "不能为负", "负的噪声密度");
        expectError(R"({"type": "imu", "site": "room/x", "noise": {"gyroscope": {}}})", "未知字段", "未知字段");
        const auto seed = sc::parseScene(R"({"format": "terra.scene", "version": 1, "models": [], "noise": {"seed": -3}})", dir + "/x.terra-scene.json");
        check(!seed.document, "负的种子：错误");
    }

    // IMU：白噪声（只开白噪声）
    const double rate = scene.sensors[level].rateHz;
    const auto steps = sampleSteps(dt, rate, 100000);
    {
        auto copy = scene.sensors[level].noise;
        scene.sensors[level].noise.gyro = {};
        scene.sensors[level].noise.accel = {};
        scene.sensors[level].noise.gyro.noiseDensity = 0.01;
        sim::SensorNoise noise(scene);
        std::vector<double> gz;
        for (const auto t : steps) {
            std::vector<double> v(6, 0.0);
            noise.imu(level, t, v);
            gz.push_back(v[5]);
        }
        const auto s = stats(gz);
        const double expected = 0.01 * std::sqrt(rate);
        std::printf("INFO 陀螺白噪声：σ %.5f（理论 %.5f），均值 %.6f\n", s.stddev, expected, s.mean);
        check(std::fabs(s.stddev / expected - 1.0) < 0.02 && std::fabs(s.mean) < 4.0 * expected / std::sqrt(static_cast<double>(gz.size())),
              "白噪声：σ = 密度 × √频率");
        scene.sensors[level].noise = copy;
    }
    // IMU：随机游走（只开随机游走）：多个种子在 T 秒时的零偏方差 ≈ rw² T
    {
        auto copy = scene.sensors[level].noise;
        const auto seedCopy = scene.document.noise.seed;
        scene.sensors[level].noise.gyro = {};
        scene.sensors[level].noise.accel = {};
        scene.sensors[level].noise.gyro.randomWalk = 0.001;
        const std::size_t samples = 400;  // 2 s
        std::vector<double> finals;
        for (std::uint64_t seed = 1; seed <= 2000; ++seed) {
            scene.document.noise.seed = seed;
            sim::SensorNoise noise(scene);
            std::vector<double> v(6, 0.0);
            noise.imu(level, steps[samples - 1], v);
            finals.push_back(v[3]);
        }
        const auto s = stats(finals);
        const double expected = 0.001 * std::sqrt(static_cast<double>(samples) / rate);
        std::printf("INFO 陀螺随机游走：%zu 个样本后 σ %.6f（理论 %.6f）\n", samples, s.stddev, expected);
        check(std::fabs(s.stddev / expected - 1.0) < 0.07, "随机游走：方差随时间线性增长（σ = rw × √T）");
        scene.sensors[level].noise = copy;
        scene.document.noise.seed = seedCopy;
    }
    // IMU：初始零偏（固定 0.1 与随机 σ 0.05）
    {
        auto copy = scene.sensors[level].noise;
        const auto seedCopy = scene.document.noise.seed;
        scene.sensors[level].noise.gyro.noiseDensity = 0;
        scene.sensors[level].noise.gyro.randomWalk = 0;
        scene.sensors[level].noise.accel.noiseDensity = 0;
        std::vector<double> gx, ax;
        for (std::uint64_t seed = 1; seed <= 4000; ++seed) {
            scene.document.noise.seed = seed;
            sim::SensorNoise noise(scene);
            std::vector<double> v(6, 0.0);
            noise.imu(level, 0, v);
            ax.push_back(v[0]);
            gx.push_back(v[3]);
        }
        const auto a = stats(ax);
        bool fixed = true;
        for (const double v : gx) fixed = fixed && v == 0.1;  // 不经过均值与方差（求和的舍入会产生 1e-17 量级的差）
        check(fixed, "固定零偏：陀螺 x 恒为 0.1");
        check(std::fabs(a.stddev / 0.05 - 1.0) < 0.05 && std::fabs(a.mean) < 0.004, "随机零偏：加速度计 σ ≈ 0.05");
        scene.sensors[level].noise = copy;
        scene.document.noise.seed = seedCopy;
    }
    // 可复现、与调用顺序无关、重置
    {
        sim::SensorNoise a(scene), b(scene), c(scene);
        std::vector<double> va, vb(6, 1.0), vc(6, 1.0);
        for (std::size_t i = 0; i < 1000; ++i) {
            va.assign(6, 1.0);
            a.imu(level, steps[i], va);
        }
        b.imu(level, steps[999], vb);  // 直接跳到第 1000 个采样
        check(va == vb, "各自的 SensorNoise 逐位相同，与调用顺序无关（随机游走只取决于步序号）");
        std::vector<double> again(6, 1.0), fresh(6, 1.0);
        a.imu(level, steps[10], again);  // 步序号变小：从初始状态重来
        c.imu(level, steps[10], fresh);
        check(again == fresh, "步序号变小后从初始状态重新累加（重置）");
        std::vector<double> t(6, 1.0);
        a.imu(tilt, steps[10], t);
        check(t == std::vector<double>(6, 1.0), "没有配置噪声的 IMU 不变");
        const auto seedCopy = scene.document.noise.seed;
        scene.document.noise.seed = 43;
        sim::SensorNoise d(scene);
        std::vector<double> vd(6, 1.0);
        d.imu(level, steps[999], vd);
        check(vd != vb, "换种子：读数不同");
        scene.document.noise.seed = seedCopy;
    }
    // 关闭的三个层次
    {
        const auto truth = std::vector<double>(6, 1.0);
        sim::SensorNoise off(scene, false);
        auto v = truth;
        off.imu(level, steps[5], v);
        check(v == truth && !off.active(), "运行时关闭（--no-noise、查看器面板）：真值");
        scene.document.noise.enabled = false;
        sim::SensorNoise sceneOff(scene);
        v = truth;
        sceneOff.imu(level, steps[5], v);
        check(v == truth && !sceneOff.active(), "场景的 noise.enabled = false：真值");
        scene.document.noise.enabled = true;
        scene.sensors[level].noise.enabled = false;
        sim::SensorNoise sensorOff(scene);
        v = truth;
        sensorOff.imu(level, steps[5], v);
        check(v == truth, "传感器的 noise.enabled = false：真值");
        scene.sensors[level].noise.enabled = true;
        sim::SensorNoise on(scene);
        v = truth;
        on.imu(level, steps[5], v);
        check(v != truth && on.active(), "开启时加噪声");
    }
    // 快照：withNoise 只改配置了噪声的 IMU
    {
        sim::SensorNoise noise(scene);
        const auto snap = scene.world->snapshot();
        const auto noisy = noise.withNoise(*snap);
        check(noisy.sensors[level].values != snap->sensors[level].values && noisy.sensors[tilt].values == snap->sensors[tilt].values,
              "快照加噪声：只改配置了噪声的 IMU");
    }

    // 2D 激光雷达
    {
        sim::SensorNoise noise(scene);
        std::size_t hits = 0, dropped = 0, infKept = 0, infTotal = 0;
        bool quantized = true;
        std::vector<double> errors;
        for (std::uint64_t step = 0; step < 200; ++step) {
            auto truth = sim::scanLidar(*scene.world, scene.lidars[0]);
            truth.step = step;
            auto scan = truth;
            noise.lidar(0, scan);
            for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
                if (!std::isfinite(truth.ranges[i])) {
                    ++infTotal;
                    infKept += scan.ranges[i] == truth.ranges[i] ? 1u : 0u;
                    continue;
                }
                ++hits;
                if (std::isinf(scan.ranges[i]) && scan.ranges[i] > 0) {
                    ++dropped;
                    continue;
                }
                const double q = static_cast<double>(scan.ranges[i]) / 0.001;
                quantized = quantized && std::fabs(q - std::round(q)) < 1e-3;
                errors.push_back(static_cast<double>(scan.ranges[i]) - static_cast<double>(truth.ranges[i]));
            }
        }
        const double rateDropped = static_cast<double>(dropped) / static_cast<double>(hits);
        const auto e = stats(errors);
        std::printf("INFO 雷达：%zu 束命中，丢失 %.4f（配置 0.05），误差 σ %.5f（配置 0.01，量化 1 mm）\n", hits, rateDropped, e.stddev);
        check(std::fabs(rateDropped - 0.05) < 0.005, "丢失率");
        check(quantized, "距离按 1 mm 量化");
        check(std::fabs(e.stddev / 0.01 - 1.0) < 0.05 && std::fabs(e.mean) < 0.0005, "距离噪声 σ ≈ 0.01 m");
        check(infTotal == 0 || infKept == infTotal, "没有命中的束保持 inf");
    }

    // 深度与彩色
    {
        sim::SensorNoise noise(scene);
        std::vector<float> depth(200000, 2.0f);
        depth[0] = std::numeric_limits<float>::infinity();
        noise.depth(0, 30, depth);
        std::vector<double> d;
        std::size_t lost = 0;
        for (std::size_t i = 1; i < depth.size(); ++i) {
            if (std::isinf(depth[i])) ++lost;
            else d.push_back(static_cast<double>(depth[i]) - 2.0);
        }
        const auto s = stats(d);
        const double expected = 0.001 + 0.002 * 4.0;
        std::printf("INFO 深度 2 m：σ %.5f（理论 %.5f），丢失 %.4f\n", s.stddev, expected, static_cast<double>(lost) / 199999.0);
        check(std::isinf(depth[0]) && std::fabs(s.stddev / expected - 1.0) < 0.02 && std::fabs(static_cast<double>(lost) / 199999.0 - 0.01) < 0.002,
              "深度：σ = a + b·z²，丢失 1%，没有命中保持 inf");
        std::vector<std::uint8_t> rgba(4 * 100000, 128);
        for (std::size_t i = 3; i < rgba.size(); i += 4) rgba[i] = 200;
        noise.color(0, 30, rgba, 4);
        std::vector<double> c;
        bool alphaKept = true;
        for (std::size_t i = 0; i < rgba.size(); ++i) {
            if (i % 4 == 3) alphaKept = alphaKept && rgba[i] == 200;
            else c.push_back(rgba[i]);
        }
        const auto cs = stats(c);
        const double expectedColor = std::sqrt(9.0 + 1.0 / 12.0);  // 取整的方差 1/12
        check(alphaKept && std::fabs(cs.stddev / expectedColor - 1.0) < 0.02 && std::fabs(cs.mean - 128.0) < 0.05, "彩色：每通道 σ ≈ 3，不改透明度");
        std::vector<std::uint8_t> white(30000, 254);
        noise.color(0, 31, white, 3);
        bool saturated = false;
        for (const auto v : white) saturated = saturated || v == 255;
        check(saturated, "彩色：超出的截断到 255");
    }

    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
