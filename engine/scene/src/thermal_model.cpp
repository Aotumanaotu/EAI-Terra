// 场景的温度表（M5-06），说明见 scene.hpp。

#include "terra/scene/scene.hpp"

namespace terra::scene {

double TemperatureCurve::at(double time) const noexcept {
    if (points.empty()) return 0.0;
    if (time <= points.front().first) return points.front().second;
    if (time >= points.back().first) return points.back().second;
    for (std::size_t i = 1; i < points.size(); ++i) {
        if (time > points[i].first) continue;
        const auto& [t0, v0] = points[i - 1];
        const auto& [t1, v1] = points[i];
        return v0 + (v1 - v0) * (time - t0) / (t1 - t0);
    }
    return points.back().second;
}

double ThermalModel::base(terra::data::EntityId entity, double time) const noexcept {
    if (!entity.valid()) return world ? world->at(time) : ambient;
    const std::size_t i = entity.value - 1;
    if (i < entities.size() && entities[i]) return entities[i]->at(time);
    return ambient;
}

const char* thermalPaletteName(ThermalPalette palette) noexcept {
    switch (palette) {
        case ThermalPalette::Ironbow: return "ironbow";
        case ThermalPalette::WhiteHot: return "white-hot";
        case ThermalPalette::BlackHot: return "black-hot";
        case ThermalPalette::Rainbow: return "rainbow";
    }
    return "ironbow";
}

}  // namespace terra::scene
