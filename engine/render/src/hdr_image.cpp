#include "terra/render/hdr_image.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "stb_image.h"  // 实现在 stb_image_impl.cpp

namespace terra::render {

namespace {

constexpr float kPi = 3.14159265358979323846f;

HdrImage filled(int width, int height) {
    HdrImage img;
    img.width = std::max(width, 1);
    img.height = std::max(height, 1);
    img.rgb.assign(static_cast<std::size_t>(img.width) * static_cast<std::size_t>(img.height) * 3u, 0.0f);
    return img;
}

void set(HdrImage& img, int x, int y, Vec3f c) {
    const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(x)) * 3u;
    img.rgb[i] = c.x;
    img.rgb[i + 1] = c.y;
    img.rgb[i + 2] = c.z;
}

}  // namespace

Vec3f HdrImage::at(int x, int y) const {
    if (width <= 0 || height <= 0) return {};
    x = std::clamp(x, 0, width - 1);
    y = std::clamp(y, 0, height - 1);
    const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x)) * 3u;
    return {rgb[i], rgb[i + 1], rgb[i + 2]};
}

HdrResult loadHdr(const std::string& path) {
    HdrResult result;
    int w = 0;
    int h = 0;
    int channels = 0;
    float* data = stbi_loadf(path.c_str(), &w, &h, &channels, 3);
    if (data == nullptr) {
        const char* reason = stbi_failure_reason();
        result.error = "无法读取 HDR 图像 " + path + "：" + (reason != nullptr ? reason : "未知原因");
        return result;
    }
    result.image.width = w;
    result.image.height = h;
    result.image.rgb.assign(data, data + static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 3u);
    stbi_image_free(data);
    return result;
}

HdrImage solidHdr(int width, int height, Vec3f color) {
    HdrImage img = filled(width, height);
    for (int y = 0; y < img.height; ++y) {
        for (int x = 0; x < img.width; ++x) set(img, x, y, color);
    }
    return img;
}

HdrImage upperLowerHdr(int width, int height, Vec3f upper, Vec3f lower) {
    HdrImage img = filled(width, height);
    for (int y = 0; y < img.height; ++y) {
        for (int x = 0; x < img.width; ++x) set(img, x, y, 2 * y < img.height ? upper : lower);
    }
    return img;
}

HdrImage leftRightHdr(int width, int height, Vec3f positiveY, Vec3f negativeY) {
    HdrImage img = filled(width, height);
    for (int y = 0; y < img.height; ++y) {
        for (int x = 0; x < img.width; ++x) set(img, x, y, 2 * x >= img.width ? positiveY : negativeY);  // u ≥ 0.5 ⇔ y ≥ 0
    }
    return img;
}

float luminance(Vec3f c) { return 0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z; }

std::optional<SunEstimate> extractSun(HdrImage& image, bool remove) {
    if (image.width <= 0 || image.height <= 0) return std::nullopt;
    const double dPhi = 2.0 * static_cast<double>(kPi) / image.width;
    const double dTheta = static_cast<double>(kPi) / image.height;
    double mean = 0.0;
    double total = 0.0;
    float peak = 0.0f;
    for (int y = 0; y < image.height; ++y) {
        const double w = dPhi * dTheta * std::sin((y + 0.5) * dTheta);
        for (int x = 0; x < image.width; ++x) {
            const float lum = luminance(image.at(x, y));
            mean += static_cast<double>(lum) * w;
            total += w;
            peak = std::max(peak, lum);
        }
    }
    mean /= total;
    if (mean <= 0.0 || peak < 50.0 * mean) return std::nullopt;
    const double threshold = std::max(20.0 * mean, 0.01 * static_cast<double>(peak));
    double omega = 0.0;
    double ex = 0.0, ey = 0.0, ez = 0.0;
    double dx = 0.0, dy = 0.0, dz = 0.0;
    for (int y = 0; y < image.height; ++y) {
        const double w = dPhi * dTheta * std::sin((y + 0.5) * dTheta);
        for (int x = 0; x < image.width; ++x) {
            const Vec3f c = image.at(x, y);
            const double lum = luminance(c);
            if (lum <= threshold) continue;
            const double keep = threshold / lum;  // 压到阈值，保持色度
            ex += c.x * (1.0 - keep) * w;
            ey += c.y * (1.0 - keep) * w;
            ez += c.z * (1.0 - keep) * w;
            const Vec3f d = equirectDirection((static_cast<float>(x) + 0.5f) / static_cast<float>(image.width),
                                              (static_cast<float>(y) + 0.5f) / static_cast<float>(image.height));
            const double weight = (lum - threshold) * w;
            dx += d.x * weight;
            dy += d.y * weight;
            dz += d.z * weight;
            omega += w;
        }
    }
    if (omega <= 0.0 || omega > 0.005 * 4.0 * static_cast<double>(kPi)) return std::nullopt;
    const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (len <= 0.0) return std::nullopt;
    SunEstimate sun;
    sun.direction = {static_cast<float>(dx / len), static_cast<float>(dy / len), static_cast<float>(dz / len)};
    sun.irradiance = {static_cast<float>(ex), static_cast<float>(ey), static_cast<float>(ez)};
    sun.solidAngle = static_cast<float>(omega);
    sun.peakLuminance = peak;
    if (remove) {
        for (int y = 0; y < image.height; ++y) {
            for (int x = 0; x < image.width; ++x) {
                const Vec3f c = image.at(x, y);
                const double lum = luminance(c);
                if (lum <= threshold) continue;
                const float keep = static_cast<float>(threshold / lum);
                set(image, x, y, {c.x * keep, c.y * keep, c.z * keep});
            }
        }
    }
    return sun;
}

Vec3f equirectDirection(float u, float v) {
    const float phi = (u - 0.5f) * 2.0f * kPi;
    const float theta = v * kPi;
    return {std::sin(theta) * std::cos(phi), std::sin(theta) * std::sin(phi), std::cos(theta)};
}

}  // namespace terra::render
