// 热成像（M5-06），说明见 thermal.hpp。

#include "terra/simulation/thermal.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>

#include "terra/scene/camera_model.hpp"
#include "terra/scene/pose_math.hpp"

namespace terra::simulation {

namespace sc = terra::scene;
namespace td = terra::data;

double hotspotWeight(double distance, double radius) noexcept {
    if (!(radius > 0) || !(distance < radius)) return 0.0;
    const double q = distance / radius;
    const double a = 1.0 - q * q;
    return a * a;
}

std::vector<float> thermalTruth(const sc::ThermalModel& model, const ThermalFrameInput& input) {
    const auto w = static_cast<std::size_t>(std::max(0, input.width));
    const auto h = static_cast<std::size_t>(std::max(0, input.height));
    if (w == 0 || h == 0 || input.ids == nullptr || input.depth == nullptr || input.ids->size() != w * h || input.depth->size() != w * h) return {};
    const auto& ids = *input.ids;
    const auto& depth = *input.depth;
    // 各实体（及世界，下标 0）在本时刻的底温：每帧只插值一次
    std::vector<double> base(model.entities.size() + 1);
    base[0] = model.base(td::EntityId{}, input.time);
    for (std::size_t e = 0; e < model.entities.size(); ++e) base[e + 1] = model.base(td::EntityId{static_cast<std::uint32_t>(e + 1)}, input.time);
    std::vector<double> hot(model.hotspots.size());
    for (std::size_t k = 0; k < model.hotspots.size(); ++k) hot[k] = model.hotspots[k].temperature.at(input.time);
    // 热点所属实体的位姿之逆（世界坐标 → 实体坐标）
    std::vector<td::Pose> toLocal(model.hotspots.size());
    for (std::size_t k = 0; k < model.hotspots.size(); ++k) {
        const auto e = model.hotspots[k].entity;
        if (e.valid() && input.entityPoses != nullptr && e.value <= input.entityPoses->size()) toLocal[k] = sc::inverse((*input.entityPoses)[e.value - 1]);
    }
    const auto in = sc::cameraIntrinsics(input.width, input.height, input.fovyDegrees);
    std::vector<float> out(w * h);
    for (std::size_t row = 0; row < h; ++row) {
        for (std::size_t col = 0; col < w; ++col) {
            const std::size_t i = row * w + col;
            const float z = depth[i];
            const std::uint32_t id = ids[i];
            if (!std::isfinite(z)) {
                out[i] = static_cast<float>(id == 0 ? model.sky : (id < base.size() ? base[id] : model.ambient));
                continue;
            }
            const double t = id < base.size() ? base[id] : model.ambient;
            double result = t;
            if (!model.hotspots.empty()) {
                // 像素中心反投影到相机坐标系，再到世界
                const double d = static_cast<double>(z);
                const td::Vec3 local{(static_cast<double>(col) - in.cx) / in.fx * d, -(static_cast<double>(row) - in.cy) / in.fy * d, -d};
                const td::Vec3 r = sc::rotate(input.cameraWorld.orientation, local);
                const td::Vec3 world{input.cameraWorld.position.x + r.x, input.cameraWorld.position.y + r.y, input.cameraWorld.position.z + r.z};
                for (std::size_t k = 0; k < model.hotspots.size(); ++k) {
                    const auto& spot = model.hotspots[k];
                    if ((spot.entity.valid() ? spot.entity.value : 0u) != id) continue;
                    td::Vec3 p = world;
                    if (spot.entity.valid()) {
                        const td::Vec3 q = sc::rotate(toLocal[k].orientation, world);
                        p = {toLocal[k].position.x + q.x, toLocal[k].position.y + q.y, toLocal[k].position.z + q.z};
                    }
                    const double dx = p.x - spot.position.x, dy = p.y - spot.position.y, dz = p.z - spot.position.z;
                    const double weight = hotspotWeight(std::sqrt(dx * dx + dy * dy + dz * dz), spot.radius);
                    if (weight > 0) result = std::max(result, t + (hot[k] - t) * weight);
                }
            }
            out[i] = static_cast<float>(result);
        }
    }
    return out;
}

std::uint16_t radiometricValue(double celsius, double rangeMin, double rangeMax) noexcept {
    double t = celsius;
    if (std::isnan(t)) t = rangeMin;
    t = std::clamp(t, rangeMin, rangeMax);
    const double v = std::round((t + 273.15) * 100.0);
    return static_cast<std::uint16_t>(std::clamp(v, 0.0, 65535.0));
}

std::vector<std::uint16_t> thermalRadiometric(const std::vector<float>& celsius, double rangeMin, double rangeMax) {
    std::vector<std::uint16_t> out(celsius.size());
    for (std::size_t i = 0; i < celsius.size(); ++i) out[i] = radiometricValue(static_cast<double>(celsius[i]), rangeMin, rangeMax);
    return out;
}

std::pair<double, double> thermalDisplayRange(const sc::ThermalCameraSpec& spec, const std::vector<float>& celsius) {
    if (!spec.autoDisplay) return {spec.displayMin, spec.displayMax};
    double lo = std::numeric_limits<double>::infinity();
    double hi = -std::numeric_limits<double>::infinity();
    for (const float v : celsius) {
        if (!std::isfinite(v)) continue;
        lo = std::min(lo, static_cast<double>(v));
        hi = std::max(hi, static_cast<double>(v));
    }
    if (!(hi >= lo)) return {0.0, 1.0};
    if (hi - lo < 1.0) {
        const double mid = 0.5 * (lo + hi);
        return {mid - 0.5, mid + 0.5};
    }
    return {lo, hi};
}

namespace {

struct Stop {
    double t;
    double r, g, b;
};

// 分段线性的色板（控制点 0～1）
void lookup(const Stop* stops, std::size_t count, double t, std::uint8_t rgb[3]) noexcept {
    t = std::clamp(std::isnan(t) ? 0.0 : t, 0.0, 1.0);
    std::size_t k = 1;
    while (k + 1 < count && t > stops[k].t) ++k;
    const Stop& a = stops[k - 1];
    const Stop& b = stops[k];
    const double f = b.t > a.t ? std::clamp((t - a.t) / (b.t - a.t), 0.0, 1.0) : 0.0;
    const double c[3] = {a.r + (b.r - a.r) * f, a.g + (b.g - a.g) * f, a.b + (b.b - a.b) * f};
    for (int i = 0; i < 3; ++i) rgb[i] = static_cast<std::uint8_t>(std::lround(std::clamp(c[i], 0.0, 255.0)));
}

// ironbow：黑 → 深蓝紫 → 品红 → 红 → 橙 → 黄 → 近白（热像仪常用的“铁红”）
constexpr Stop kIronbow[] = {{0.00, 0, 0, 0},     {0.10, 30, 0, 90},    {0.25, 110, 0, 150}, {0.40, 190, 20, 110},
                             {0.55, 230, 70, 40}, {0.70, 250, 130, 0}, {0.85, 255, 200, 40}, {1.00, 255, 255, 230}};
// rainbow：蓝 → 青 → 绿 → 黄 → 红
constexpr Stop kRainbow[] = {{0.00, 0, 0, 255}, {0.25, 0, 255, 255}, {0.50, 0, 255, 0}, {0.75, 255, 255, 0}, {1.00, 255, 0, 0}};
constexpr Stop kWhiteHot[] = {{0.0, 0, 0, 0}, {1.0, 255, 255, 255}};
constexpr Stop kBlackHot[] = {{0.0, 255, 255, 255}, {1.0, 0, 0, 0}};

}  // namespace

void thermalPaletteColor(sc::ThermalPalette palette, double t, std::uint8_t rgb[3]) noexcept {
    switch (palette) {
        case sc::ThermalPalette::Ironbow: lookup(kIronbow, std::size(kIronbow), t, rgb); return;
        case sc::ThermalPalette::WhiteHot: lookup(kWhiteHot, std::size(kWhiteHot), t, rgb); return;
        case sc::ThermalPalette::BlackHot: lookup(kBlackHot, std::size(kBlackHot), t, rgb); return;
        case sc::ThermalPalette::Rainbow: lookup(kRainbow, std::size(kRainbow), t, rgb); return;
    }
    lookup(kIronbow, std::size(kIronbow), t, rgb);
}

std::vector<std::uint8_t> thermalColor(const std::vector<float>& celsius, sc::ThermalPalette palette, double low, double high) {
    std::vector<std::uint8_t> out(celsius.size() * 3);
    const double span = high > low ? high - low : 1.0;
    for (std::size_t i = 0; i < celsius.size(); ++i) thermalPaletteColor(palette, (static_cast<double>(celsius[i]) - low) / span, out.data() + i * 3);
    return out;
}

}  // namespace terra::simulation
