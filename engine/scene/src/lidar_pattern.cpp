// 3D 激光雷达的扫描图案（M5-11），说明见 scene.hpp。

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <mutex>

#include "terra/scene/scene.hpp"

namespace terra::scene {

std::size_t Lidar3dSensor::pointsPerFrame() const noexcept {
    if (!(rateHz > 0.0) || !(pointsPerSecond > 0.0)) return 0;
    return static_cast<std::size_t>(std::llround(pointsPerSecond / rateHz));
}

std::shared_ptr<const Lidar3dPattern> loadLidarPattern(const std::string& path, std::string& error) {
    static std::mutex mutex;
    static std::map<std::string, std::shared_ptr<const Lidar3dPattern>> cache;
    std::lock_guard<std::mutex> lock(mutex);
    if (const auto it = cache.find(path); it != cache.end()) return it->second;
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        error = "扫描图案不存在：" + path + "（运行 scripts/assets/fetch_lidar_patterns.sh 下载）";
        return nullptr;
    }
    auto p = std::make_shared<Lidar3dPattern>();
    p->path = path;
    std::string line;
    std::size_t number = 0;
    while (std::getline(in, line)) {
        ++number;
        if (number == 1) continue;  // 表头
        if (line.empty() || line == "\r") continue;
        const char* c = line.c_str();
        char* end = nullptr;
        std::strtod(c, &end);  // 序号
        if (end == c || *end != ',') {
            error = path + " 第 " + std::to_string(number) + " 行格式不对（应为 序号, 方位角, 天顶角）";
            return nullptr;
        }
        c = end + 1;
        const double azimuth = std::strtod(c, &end);
        if (end == c || *end != ',') {
            error = path + " 第 " + std::to_string(number) + " 行的方位角不对";
            return nullptr;
        }
        c = end + 1;
        const double zenith = std::strtod(c, &end);
        if (end == c || !std::isfinite(azimuth) || !std::isfinite(zenith) || zenith < 0.0 || zenith > 180.0) {
            error = path + " 第 " + std::to_string(number) + " 行的天顶角不对";
            return nullptr;
        }
        p->azimuth.push_back(static_cast<float>(azimuth));
        p->zenith.push_back(static_cast<float>(zenith));
    }
    if (p->azimuth.empty()) {
        error = "扫描图案为空：" + path;
        return nullptr;
    }
    cache[path] = p;
    return p;
}

}  // namespace terra::scene
