#pragma once

// 温度的描述（M5-06）：场景文件的 thermal 段与元素文件（M5-02a）共用。

#include <string>
#include <utility>
#include <vector>

#include "terra/data/geometry.hpp"

namespace terra::scene {

// 温度曲线 [[t, T], …]：仿真时间（秒）→ 温度（°C），按时间线性插值，第一点之前与最后一点之后取端点的值；
// 场景文件中也可以只写一个数（常数）
struct TemperatureCurve {
    std::vector<std::pair<double, double>> points;  // 按时间升序
    double at(double time) const noexcept;
};

// 场景文件的 thermal 段：
//   ambient（默认 20 °C）：没有指定温度的物体；sky（默认 −20 °C）：没有命中的像素；
//   temperatures：{"实体全名" | "实例名" | "前缀*" | "world": 温度或曲线}，匹配规则与 labels 相同（实体全名 > 实例名 > 最长的前缀），
//     "world" 为不属于任何实体的静态几何体（如 MJCF worldbody 中的地面、墙）；
//   hotspots：[{entity, position（实体坐标系，米）, radius（米）, temperature}]：以 position 为中心、radius 为半径的区域，
//     温度 = 底温 + (热点温度 − 底温) × (1 − (d / radius)²)²（中心为热点温度，边缘平滑地回到底温）；多个热点取最高
struct HotspotSpec {
    std::string entity;
    terra::data::Vec3 position{0, 0, 0};
    double radius = 0.1;
    TemperatureCurve temperature;
    std::string where;
};

}  // namespace terra::scene
