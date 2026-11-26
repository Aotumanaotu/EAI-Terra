#pragma once

// 热成像（M5-06，M5-01 设计稿第 3 节）：由相机同一帧的分割图（实体 ID）与线性深度算出每个像素的温度（°C），
// 与彩色、深度、分割像素对齐。纯 CPU，不需要图形上下文（测试可直接给出分割图与深度）。
//
// 温度来源（优先级从高到低）：热点 → 实体（场景 thermal.temperatures 中匹配到的写法）→ 环境温度；
//   分割为 0（不属于任何实体）且有命中的像素为静态几何体（"world"，没写时为环境温度）；没有命中的像素为天空温度。
// 热点需要每个像素的表面位置：由深度与内参反投影到相机坐标系（Terra 相机：看向 −Z，+Y 向上，像素中心为整数，
//   主点 ((w − 1) / 2, (h − 1) / 2)），再变换到所属实体的坐标系。
// 不模拟的物理（写进报告）：发射率与反射、大气衰减、热传导与热辐射、镜头渐晕、非均匀校正（NUC）的快门冻结。
//
// 输出：
//   - 辐射测温图：16 位无符号，单位 0.01 K（与 FLIR Lepton 的 TLinear 0.01 K 相同）：读数 = round((T + 273.15) × 100)，
//     T 先截断到相机的量程；
//   - 伪彩色图：RGB 8 位，色板 ironbow（默认）、white-hot、black-hot、rainbow；区间为每帧的最低、最高温度（auto），
//     或场景文件中写的固定区间；区间外截断。

#include <cstdint>
#include <utility>
#include <vector>

#include "terra/data/geometry.hpp"
#include "terra/scene/scene.hpp"

namespace terra::simulation {

struct ThermalFrameInput {
    int width = 0;
    int height = 0;
    double fovyDegrees = 60.0;
    terra::data::Pose cameraWorld;  // Terra 相机坐标系在世界中的位姿
    double time = 0.0;              // 仿真时间（温度曲线按它插值）
    const std::vector<std::uint32_t>* ids = nullptr;        // width × height，实体 ID（0 为不属于实体）
    const std::vector<float>* depth = nullptr;              // width × height，沿光轴的距离（米），没有命中为 +inf
    const std::vector<terra::data::Pose>* entityPoses = nullptr;  // 按 EntityId 下标（value − 1）；没有热点时可为空
};

// 真值温度图（°C，行 0 为顶部）。ids 或 depth 的大小不对时返回空
std::vector<float> thermalTruth(const terra::scene::ThermalModel& model, const ThermalFrameInput& input);

// 热点的权重：d 为到中心的距离，r 为半径；(1 − (d / r)²)²，d ≥ r 时为 0
double hotspotWeight(double distance, double radius) noexcept;

// 辐射测温图（0.01 K）：T 截断到 [rangeMin, rangeMax] 后换算，再截断到 0～65535
std::vector<std::uint16_t> thermalRadiometric(const std::vector<float>& celsius, double rangeMin, double rangeMax);
std::uint16_t radiometricValue(double celsius, double rangeMin, double rangeMax) noexcept;

// 伪彩色的区间（°C）：autoDisplay 时为本帧的最低、最高温度（区间不足 1 °C 时以中点为中心扩到 1 °C）；否则为固定区间
std::pair<double, double> thermalDisplayRange(const terra::scene::ThermalCameraSpec& spec, const std::vector<float>& celsius);

// 色板：t ∈ [0, 1]（低 → 高）→ RGB
void thermalPaletteColor(terra::scene::ThermalPalette palette, double t, std::uint8_t rgb[3]) noexcept;
// 伪彩色图（width × height × 3，RGB 8 位）
std::vector<std::uint8_t> thermalColor(const std::vector<float>& celsius, terra::scene::ThermalPalette palette, double low, double high);

}  // namespace terra::simulation
