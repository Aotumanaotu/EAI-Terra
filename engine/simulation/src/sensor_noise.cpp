// 传感器噪声（M4-09）。

#include "terra/simulation/sensor_noise.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "terra/simulation/sensor_sampling.hpp"

namespace terra::simulation {

namespace {

constexpr double kTwoPi = 6.283185307179586476925286766559;
// 各类随机数的分量编号区段（同一步、同一传感器内互不重叠）
constexpr std::uint64_t kImuWhite = 0;          // 6 个分量 × 2（高斯用两个均匀数）
constexpr std::uint64_t kImuWalk = 64;          // 6 个分量 × 2
constexpr std::uint64_t kImuBias = 128;         // 6 个分量 × 2
constexpr std::uint64_t kDropout = 1ull << 40;  // 雷达束与深度像素的丢失（之下为高斯的分量）
constexpr std::uint64_t kInitialStep = std::numeric_limits<std::uint64_t>::max();  // 初始零偏用的“步序号”

std::uint64_t splitMix(std::uint64_t x) noexcept {
    x += 0x9E3779B97F4A7C15ull;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
    return x ^ (x >> 31);
}

bool anyNoise(const terra::scene::ImuNoiseAxis& a) noexcept {
    return a.noiseDensity > 0 || a.randomWalk > 0 || a.biasStd > 0 || a.bias[0] != 0 || a.bias[1] != 0 || a.bias[2] != 0;
}

}  // namespace

std::uint64_t noiseBits(std::uint64_t seed, std::uint64_t stream, std::uint64_t step, std::uint64_t index) noexcept {
    return splitMix(seed ^ splitMix(stream ^ splitMix(step ^ splitMix(index))));
}

double noiseUniform(std::uint64_t seed, std::uint64_t stream, std::uint64_t step, std::uint64_t index) noexcept {
    return static_cast<double>(noiseBits(seed, stream, step, index) >> 11) * (1.0 / 9007199254740992.0);
}

double noiseGaussian(std::uint64_t seed, std::uint64_t stream, std::uint64_t step, std::uint64_t index) noexcept {
    // Box–Muller：两个均匀数取自分量 2·index 与 2·index + 1
    const double u1 = 1.0 - noiseUniform(seed, stream, step, 2 * index);  // (0, 1]
    const double u2 = noiseUniform(seed, stream, step, 2 * index + 1);
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(kTwoPi * u2);
}

std::uint64_t noiseStream(const std::string& sensorName) noexcept {
    std::uint64_t h = 1469598103934665603ull;
    for (const char c : sensorName) {
        h ^= static_cast<std::uint8_t>(c);
        h *= 1099511628211ull;
    }
    return h;
}

SensorNoise::SensorNoise(const terra::scene::SceneInstance& scene, bool enabled)
    : seed_(scene.document.noise.seed), sceneEnabled_(scene.document.noise.enabled), runtimeEnabled_(enabled) {
    const auto& d = scene.world->description();
    timestep_ = d.timestep;
    for (std::size_t k = 0; k < d.sensors.size(); ++k) {
        ImuState s;
        if (k < scene.sensors.size()) {
            s.config = scene.sensors[k].noise;
            s.rateHz = scene.sensors[k].rateHz;
        }
        s.stream = noiseStream(d.sensors[k].name);
        const terra::scene::ImuNoiseAxis* axes[2] = {&s.config.accel, &s.config.gyro};
        for (std::size_t c = 0; c < 6; ++c) {
            const auto& axis = *axes[c / 3];
            s.initial[c] = axis.bias[c % 3] + axis.biasStd * noiseGaussian(seed_, s.stream, kInitialStep, kImuBias + c);
        }
        imus_.push_back(s);
    }
    for (const auto& l : scene.lidars) lidars_.emplace_back(l.noise, noiseStream(l.name));
    for (const auto& c : scene.cameras) cameras_.emplace_back(c.noise, noiseStream(c.name));
}

bool SensorNoise::imuActive(const ImuState& s) const noexcept {
    return sceneEnabled_ && runtimeEnabled_ && s.config.configured && s.config.enabled && (anyNoise(s.config.gyro) || anyNoise(s.config.accel));
}

bool SensorNoise::active() const noexcept {
    if (!sceneEnabled_ || !runtimeEnabled_) return false;
    for (const auto& s : imus_) {
        if (imuActive(s)) return true;
    }
    for (const auto& [n, stream] : lidars_) {
        (void)stream;
        if (n.configured && n.enabled && (n.rangeStd > 0 || n.rangeStdRelative > 0 || n.dropout > 0 || n.resolution > 0)) return true;
    }
    for (const auto& [n, stream] : cameras_) {
        (void)stream;
        if (n.configured && n.enabled && (n.depthStd > 0 || n.depthStdQuadratic > 0 || n.dropout > 0 || n.pixelStd > 0)) return true;
    }
    return false;
}

// 随机游走累加到 step（含）：只在该 IMU 的采样步上走一步；步序号变小时从初始状态重新累加
void SensorNoise::advance(ImuState& s, std::uint64_t step) {
    std::uint64_t from = 0;
    if (s.hasLast && step >= s.last) {
        from = s.last + 1;
    } else {
        s.walk.fill(0.0);
    }
    if (s.hasLast && step == s.last) return;
    const double rate = s.rateHz > 0 ? s.rateHz : (timestep_ > 0 ? 1.0 / timestep_ : 0.0);
    const double sqrtDt = rate > 0 ? std::sqrt(1.0 / rate) : 0.0;
    const double walk[6] = {s.config.accel.randomWalk, s.config.accel.randomWalk, s.config.accel.randomWalk,
                            s.config.gyro.randomWalk,  s.config.gyro.randomWalk,  s.config.gyro.randomWalk};
    const bool walks = walk[0] > 0 || walk[3] > 0;
    for (std::uint64_t t = from; walks && t <= step; ++t) {
        if (!sensorSampleDue(t, timestep_, rate)) continue;
        for (std::size_t c = 0; c < 6; ++c) {
            if (walk[c] > 0) s.walk[c] += walk[c] * sqrtDt * noiseGaussian(seed_, s.stream, t, kImuWalk + c);
        }
    }
    s.last = step;
    s.hasLast = true;
}

void SensorNoise::imu(std::size_t k, std::uint64_t step, std::vector<double>& values) {
    if (k >= imus_.size() || values.size() < 6) return;
    ImuState& s = imus_[k];
    if (!imuActive(s)) return;
    advance(s, step);
    const double rate = s.rateHz > 0 ? s.rateHz : (timestep_ > 0 ? 1.0 / timestep_ : 0.0);
    const double sqrtRate = std::sqrt(std::max(rate, 0.0));
    for (std::size_t c = 0; c < 6; ++c) {
        const auto& axis = c < 3 ? s.config.accel : s.config.gyro;
        double v = values[c] + s.initial[c] + s.walk[c];
        if (axis.noiseDensity > 0) v += axis.noiseDensity * sqrtRate * noiseGaussian(seed_, s.stream, step, kImuWhite + c);
        values[c] = v;
    }
}

terra::data::PhysicsSnapshot SensorNoise::withNoise(const terra::data::PhysicsSnapshot& snapshot) {
    terra::data::PhysicsSnapshot out = snapshot;
    for (std::size_t k = 0; k < out.sensors.size(); ++k) imu(k, out.stepIndex, out.sensors[k].values);
    return out;
}

void SensorNoise::lidar(std::size_t k, terra::simulation::LidarScan& scan) const {
    if (k >= lidars_.size() || !sceneEnabled_ || !runtimeEnabled_) return;
    const auto& [n, stream] = lidars_[k];
    if (!n.configured || !n.enabled) return;
    const float inf = std::numeric_limits<float>::infinity();
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
        float& r = scan.ranges[i];
        if (!std::isfinite(r)) continue;  // 没有命中的束保持 inf
        if (n.dropout > 0 && noiseUniform(seed_, stream, scan.step, kDropout + i) < n.dropout) {
            r = inf;
            continue;
        }
        double v = r;
        const double sigma = n.rangeStd + n.rangeStdRelative * v;
        if (sigma > 0) v += sigma * noiseGaussian(seed_, stream, scan.step, i);
        if (n.resolution > 0) v = std::round(v / n.resolution) * n.resolution;
        if (v > scan.rangeMax) r = inf;
        else if (v < scan.rangeMin) r = -inf;
        else r = static_cast<float>(v);
    }
}

void SensorNoise::depth(std::size_t k, std::uint64_t step, std::vector<float>& depth) const {
    if (k >= cameras_.size() || !sceneEnabled_ || !runtimeEnabled_) return;
    const auto& [n, stream] = cameras_[k];
    if (!n.configured || !n.enabled || (n.depthStd <= 0 && n.depthStdQuadratic <= 0 && n.dropout <= 0)) return;
    const float inf = std::numeric_limits<float>::infinity();
    for (std::size_t i = 0; i < depth.size(); ++i) {
        float& z = depth[i];
        if (!std::isfinite(z)) continue;
        if (n.dropout > 0 && noiseUniform(seed_, stream, step, kDropout + i) < n.dropout) {
            z = inf;
            continue;
        }
        const double sigma = n.depthStd + n.depthStdQuadratic * static_cast<double>(z) * static_cast<double>(z);
        if (sigma > 0) z = static_cast<float>(std::max(0.0, static_cast<double>(z) + sigma * noiseGaussian(seed_, stream, step, i)));
    }
}

void SensorNoise::color(std::size_t k, std::uint64_t step, std::vector<std::uint8_t>& pixels, int channels) const {
    if (k >= cameras_.size() || !sceneEnabled_ || !runtimeEnabled_ || channels <= 0) return;
    const auto& [n, stream] = cameras_[k];
    if (!n.configured || !n.enabled || n.pixelStd <= 0) return;
    const auto per = static_cast<std::size_t>(channels);
    for (std::size_t i = 0; i < pixels.size(); ++i) {
        if (per == 4 && i % 4 == 3) continue;  // 不改透明度
        // 彩色的分量编号与深度错开（同一相机两者都可能加噪声）
        const double v = pixels[i] + n.pixelStd * noiseGaussian(seed_, stream ^ 0x5A5A5A5A5A5A5A5Aull, step, i);
        pixels[i] = static_cast<std::uint8_t>(std::clamp(std::lround(v), 0L, 255L));
    }
}

}  // namespace terra::simulation
