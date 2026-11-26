// 热像仪（M5-06，simulation.thermal）。不需要图形：温度图由分割图与深度算出，这里直接给出。
//   - 场景文件：预设（Lepton 3.5 的分辨率、由水平视场角算出的竖直视场角、帧率、量程、精度、NETD）；色板、显示区间、噪声开关；
//     thermal 段的匹配（实体全名 > 实例名 > 前缀，"world"）、随时间变化的曲线、热点；错误：未知预设、增益档、色板、曲线、热点实体；
//   - 温度图：天空、世界、实体、环境温度；热点的中心与权重（按反投影出的表面位置）；
//   - 辐射测温（0.01 K，量程外截断）、色板的端点、显示区间；
//   - 噪声：时间噪声的标准差 = NETD；固定偏差每台相机一个、各帧相同；同一步逐位相同；关闭时为真值。
// 夹具 assets/tests/sensors/thermal_bench.*。统计容差按样本数取，冻结在本文件中。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/scene/scene.hpp"
#include "terra/simulation/sensor_noise.hpp"
#include "terra/simulation/thermal.hpp"

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

std::size_t cameraIndex(const sc::SceneInstance& s, const std::string& name) {
    for (std::size_t k = 0; k < s.cameras.size(); ++k) {
        if (s.cameras[k].name == name) return k;
    }
    return s.cameras.size();
}

// 场景文本中的错误（解析或实例化），没有错误时为空
std::string errorOf(const std::string& thermal, const std::string& sensor) {
    const std::string text = R"({"format": "terra.scene", "version": 1, "models": [{"name": "bench", "mjcf": ")" + kDir +
                             R"(/thermal_bench.xml"}])" + (thermal.empty() ? std::string{} : ", \"thermal\": " + thermal) +
                             (sensor.empty() ? std::string{} : ", \"sensors\": [" + sensor + "]") + "}";
    const auto parsed = sc::parseScene(text, kDir + "/x.terra-scene.json");
    for (const auto& i : parsed.issues) {
        if (i.severity == sc::SceneIssue::Severity::Error) return i.message;
    }
    if (!parsed.document) return "?";
    const auto r = sc::instantiate(*parsed.document, terra::physics::mujoco::composeWorld);
    for (const auto& i : r.issues) {
        if (i.severity == sc::SceneIssue::Severity::Error) return i.message;
    }
    return {};
}

}  // namespace

int main() {
    auto loaded = sc::loadScene(kDir + "/thermal_bench.terra-scene.json", terra::physics::mujoco::composeWorld);
    for (const auto& i : loaded.issues) std::fprintf(stderr, "场景：%s %s\n", i.where.c_str(), i.message.c_str());
    if (!loaded.scene) return 1;
    auto& scene = *loaded.scene;
    const auto& d = scene.world->description();

    // ---- 场景文件：预设 ----
    const std::size_t ir = cameraIndex(scene, "bench/ir");
    const std::size_t noisy = cameraIndex(scene, "bench/ir_noisy");
    const std::size_t skyCam = cameraIndex(scene, "bench/ir_sky");
    if (ir >= scene.cameras.size() || noisy >= scene.cameras.size() || skyCam >= scene.cameras.size()) {
        std::fprintf(stderr, "FAILED: 夹具中应有 bench/ir、bench/ir_noisy、bench/ir_sky\n");
        return 1;
    }
    const auto& c = scene.cameras[ir];
    const double fovy = 2.0 * std::atan(std::tan(57.0 * 3.14159265358979323846 / 360.0) * 120.0 / 160.0) * 360.0 / 3.14159265358979323846;
    check(c.thermal.enabled && c.thermal.preset == "flir_lepton_3_5" && c.width == 160 && c.height == 120 && std::fabs(c.fovyDegrees - fovy) < 1e-9 &&
              c.rateHz == 8.7,
          "Lepton 3.5：160 × 120，竖直视场角由水平 57° 算出（" + std::to_string(c.fovyDegrees) + "°），8.7 Hz");
    check(c.thermal.gain == "high" && c.thermal.rangeMin == -10.0 && c.thermal.rangeMax == 140.0, "默认高增益：量程 −10～140 °C");
    check(c.thermal.noise.configured && !c.thermal.noise.enabled && c.thermal.palette == sc::ThermalPalette::Ironbow && c.thermal.autoDisplay,
          "bench/ir：噪声关闭，默认色板 ironbow，显示区间 auto");
    const auto& n = scene.cameras[noisy].thermal;
    check(n.noise.enabled && n.noise.netdK == 0.05 && n.noise.accuracyAbsolute == 5.0 && n.noise.accuracyRelative == 0.05,
          "bench/ir_noisy：NETD 0.05 K，精度 ±5 °C 或 5%（预设）");
    check(n.palette == sc::ThermalPalette::WhiteHot && !n.autoDisplay && n.displayMin == 0.0 && n.displayMax == 130.0 && !scene.cameras[noisy].rgb &&
              !scene.cameras[noisy].depth,
          "bench/ir_noisy：white-hot，固定区间 0～130 °C；只输出温度（rgb、depth 都关闭）");

    check(scene.cameras[skyCam].thermal.enabled && !scene.cameras[skyCam].rgb, "bench/ir_sky：只输出温度");

    // ---- 场景文件：温度表 ----
    const auto entity = [&](const char* name) {
        const auto* e = d.findEntity(name);
        return e ? e->id : td::EntityId{};
    };
    const auto& t = scene.thermal;
    check(t.ambient == 22.0 && t.sky == -20.0 && t.world && t.world->at(0) == 15.0, "环境 22 °C，天空 −20 °C，世界（墙）15 °C");
    check(t.base(entity("bench/plate"), 3.0) == 80.0, "实体全名优先于前缀：板 80 °C");
    check(t.base(entity("bench/rig"), 0.0) == 30.0, "前缀 bench/*：支架 30 °C");
    check(t.base(entity("bench/block"), 0.0) == 20.0 && std::fabs(t.base(entity("bench/block"), 0.25) - 30.0) < 1e-12 && t.base(entity("bench/block"), 5.0) == 60.0,
          "曲线：方块 0 s 20 °C、0.25 s 30 °C，1 s 之后保持 60 °C");
    check(t.hotspots.size() == 1 && t.hotspots[0].entity == entity("bench/heater") && t.hotspots[0].radius == 0.08 && t.hotspots[0].temperature.at(0) == 120.0,
          "热点：加热器正面中心，半径 0.08 m，120 °C");

    // ---- 场景文件：错误 ----
    const std::string cam = R"({"type": "camera", "name": "bench/c", "entity": "bench/rig", )";
    check(errorOf("", cam + R"("thermal": {"preset": "flir_x"}})").find("flir_lepton_3_5") != std::string::npos, "未知预设：报错并列出可用的预设");
    check(errorOf("", cam + R"("thermal": {"preset": "flir_lepton_3_5", "gain": "mid"}})").find("high") != std::string::npos, "未知增益档：报错并列出可用的档");
    check(!errorOf("", cam + R"("thermal": {"preset": "flir_lepton_3_5", "palette": "jet"}})").empty(), "未知色板：报错");
    check(!errorOf("", cam + R"("thermal": {"preset": "flir_lepton_3_5", "display": [50, 20]}})").empty(), "显示区间上限不大于下限：报错");
    check(errorOf("", cam + R"("thermal": {"preset": "flir_lepton_3_5"}, "rgb": false, "depth": false})").empty(), "热像仪可以关闭 rgb 与 depth");
    check(!errorOf("", cam + R"("rgb": false, "depth": false})").empty(), "普通相机 rgb 与 depth 都关闭：报错");
    check(!errorOf(R"({"temperatures": {"bench/block": [[1, 20], [0, 30]]}})", "").empty(), "曲线时间不递增：报错");
    check(!errorOf(R"({"temperatures": {"bench/*x": 20}})", "").empty(), "* 不在末尾：报错");
    check(!errorOf(R"({"hotspots": [{"entity": "bench/nothing", "temperature": 50}]})", "").empty(), "热点的实体不存在：报错");
    check(!errorOf(R"({"ambient": -300})", "").empty(), "低于绝对零度：报错");

    // ---- 温度图：天空、世界、实体、环境 ----
    {
        sc::ThermalModel m;
        m.ambient = 21.0;
        m.sky = -30.0;
        m.world = sc::TemperatureCurve{{{0.0, 10.0}}};
        m.entities = {sc::TemperatureCurve{{{0.0, 50.0}}}, std::nullopt};
        const std::vector<std::uint32_t> ids = {0, 0, 1, 2};
        const float inf = std::numeric_limits<float>::infinity();
        const std::vector<float> depth = {inf, 1.0f, 1.0f, 1.0f};
        sim::ThermalFrameInput in;
        in.width = 2;
        in.height = 2;
        in.ids = &ids;
        in.depth = &depth;
        const auto out = sim::thermalTruth(m, in);
        check(out.size() == 4 && out[0] == -30.0f && out[1] == 10.0f && out[2] == 50.0f && out[3] == 21.0f, "没有命中 → 天空；ID 0 → 世界；实体 → 它的温度；没写 → 环境");
        sim::ThermalFrameInput wrong = in;
        wrong.width = 3;
        check(sim::thermalTruth(m, wrong).empty(), "尺寸不符：返回空");
    }
    // ---- 温度图：热点（相机在原点看向 −Z，实体在 (0, 0, −2)，深度处处 2 m，竖直视场角 90°）----
    {
        sc::ThermalModel m;
        m.entities = {sc::TemperatureCurve{{{0.0, 20.0}}}};
        sc::ThermalModel::Hotspot spot;
        spot.entity = td::EntityId{1};
        spot.position = {0, 0, 0};
        spot.radius = 1.0;
        spot.temperature = sc::TemperatureCurve{{{0.0, 100.0}}};
        m.hotspots = {spot};
        const std::vector<std::uint32_t> ids(25, 1);
        const std::vector<float> depth(25, 2.0f);
        td::Pose entityPose;
        entityPose.position = {0, 0, -2};
        const std::vector<td::Pose> poses = {entityPose};
        sim::ThermalFrameInput in;
        in.width = 5;
        in.height = 5;
        in.fovyDegrees = 90.0;
        in.ids = &ids;
        in.depth = &depth;
        in.entityPoses = &poses;
        const auto out = sim::thermalTruth(m, in);
        // fy = 2.5，主点 (2, 2)：像素 (3, 2) 在相机坐标 x = 0.8 m 处，距热点中心 0.8 m，权重 (1 − 0.64)² = 0.1296
        check(out.size() == 25 && std::fabs(out[12] - 100.0f) < 1e-4f, "热点中心的像素为热点温度 100 °C（" + std::to_string(out.empty() ? 0.f : out[12]) + "）");
        check(out.size() == 25 && std::fabs(static_cast<double>(out[13]) - (20.0 + 80.0 * 0.1296)) < 1e-4, "距中心 0.8 m：20 + 80 × 0.1296 °C");
        check(out.size() == 25 && out[0] == 20.0f, "角上的像素距中心 > 1 m：底温 20 °C");
        check(sim::hotspotWeight(0.0, 1.0) == 1.0 && sim::hotspotWeight(1.0, 1.0) == 0.0 && sim::hotspotWeight(2.0, 1.0) == 0.0, "热点权重：中心 1，边缘与外面 0");
    }

    // ---- 辐射测温、色板、显示区间 ----
    check(sim::radiometricValue(80.0, -10.0, 140.0) == 35315 && sim::radiometricValue(-273.15, -300.0, 100.0) == 0, "80 °C → 35315（0.01 K）；0 K → 0");
    check(sim::radiometricValue(200.0, -10.0, 140.0) == 41315 && sim::radiometricValue(-50.0, -10.0, 140.0) == 26315, "量程外截断：200 → 140 °C，−50 → −10 °C");
    check(sim::radiometricValue(std::nan(""), -10.0, 140.0) == 26315, "NaN 记为量程下限");
    std::uint8_t rgb[3];
    sim::thermalPaletteColor(sc::ThermalPalette::Ironbow, 0.0, rgb);
    bool ok = rgb[0] == 0 && rgb[1] == 0 && rgb[2] == 0;
    sim::thermalPaletteColor(sc::ThermalPalette::Ironbow, 1.0, rgb);
    ok = ok && rgb[0] == 255 && rgb[1] == 255 && rgb[2] == 230;
    check(ok, "ironbow：最低为黑，最高为近白");
    sim::thermalPaletteColor(sc::ThermalPalette::WhiteHot, 0.5, rgb);
    ok = rgb[0] == 128 && rgb[1] == 128 && rgb[2] == 128;
    sim::thermalPaletteColor(sc::ThermalPalette::BlackHot, 0.0, rgb);
    ok = ok && rgb[0] == 255;
    sim::thermalPaletteColor(sc::ThermalPalette::Rainbow, 0.0, rgb);
    ok = ok && rgb[0] == 0 && rgb[1] == 0 && rgb[2] == 255;
    sim::thermalPaletteColor(sc::ThermalPalette::Rainbow, 2.0, rgb);
    ok = ok && rgb[0] == 255 && rgb[1] == 0 && rgb[2] == 0;
    check(ok, "white-hot 中间为灰，black-hot 最低为白，rainbow 从蓝到红，区间外截断");
    {
        sc::ThermalCameraSpec spec;
        const auto [lo, hi] = sim::thermalDisplayRange(spec, {20.0f, 20.2f});
        const auto [lo2, hi2] = sim::thermalDisplayRange(spec, {10.0f, 40.0f});
        spec.autoDisplay = false;
        spec.displayMin = 0;
        spec.displayMax = 130;
        const auto [lo3, hi3] = sim::thermalDisplayRange(spec, {10.0f, 40.0f});
        check(std::fabs(lo - 19.6) < 1e-6 && std::fabs(hi - 20.6) < 1e-6 && lo2 == 10.0 && hi2 == 40.0 && lo3 == 0.0 && hi3 == 130.0,
              "显示区间：auto 取本帧最低、最高（不足 1 °C 时扩到 1 °C），固定区间不变");
        const auto colors = sim::thermalColor({10.0f, 40.0f}, sc::ThermalPalette::WhiteHot, 10.0, 40.0);
        check(colors.size() == 6 && colors[0] == 0 && colors[3] == 255, "伪彩色图：每像素 3 个字节");
    }

    // ---- 噪声 ----
    {
        sim::SensorNoise noise(scene);
        check(noise.active(), "有噪声的热像仪：噪声开启");
        const std::size_t pixels = 160 * 120;
        const std::vector<float> truth(pixels, 30.0f);
        auto clean = truth;
        noise.thermal(ir, 100, clean);
        check(clean == truth, "bench/ir 关闭了噪声：真值");
        auto a = truth, b = truth, a2 = truth;
        noise.thermal(noisy, 100, a);
        noise.thermal(noisy, 200, b);
        noise.thermal(noisy, 100, a2);
        check(a == a2 && a != b, "同一步逐位相同，不同步不同");
        const auto meanStd = [&](const std::vector<float>& v) {
            double mean = 0;
            for (float x : v) mean += static_cast<double>(x) - 30.0;
            mean /= static_cast<double>(v.size());
            double var = 0;
            for (float x : v) var += (static_cast<double>(x) - 30.0 - mean) * (static_cast<double>(x) - 30.0 - mean);
            return std::pair<double, double>{mean, std::sqrt(var / static_cast<double>(v.size() - 1))};
        };
        const auto [meanA, stdA] = meanStd(a);
        const auto [meanB, stdB] = meanStd(b);
        std::printf("INFO 热像噪声：固定偏差 %.4f °C（两帧 %.4f / %.4f），时间噪声 σ %.4f / %.4f K（NETD 0.05 K）\n", meanA, meanA, meanB, stdA, stdB);
        // 19200 个样本：标准差的相对误差约 1 / √(2n) ≈ 0.5%，取 3%；均值之差约 √2 × 0.05 / √n ≈ 0.0005，取 0.003
        check(std::fabs(stdA - 0.05) < 0.0015 && std::fabs(stdB - 0.05) < 0.0015, "时间噪声的标准差 = NETD 0.05 K（3% 以内）");
        check(std::fabs(meanA - meanB) < 0.003 && std::fabs(meanA) < 4.0 * 2.5, "固定偏差各帧相同，在精度（±5 °C，按 2σ）范围内");
        // 种子 7 下的标准正态数 g 不为 0（|g| < 0.0004 的概率约 0.03%，种子固定后结果确定）
        check(std::fabs(meanA) > 1e-3, "有固定偏差（均值不为 0）");
        sim::SensorNoise off(scene, false);
        auto c2 = truth;
        off.thermal(noisy, 100, c2);
        check(c2 == truth && !off.active(), "运行时关闭噪声：真值");
    }

    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
