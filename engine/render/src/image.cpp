#include "terra/render/image.hpp"

#include <algorithm>
#include <climits>
#include <cstddef>
#include <cstdio>
#include <cstdlib>

#include "stb_image.h"        // 实现在 stb_image_impl.cpp
#include "stb_image_write.h"  // 同上

namespace terra::render {

namespace {

ImageResult take(unsigned char* data, int w, int h, const std::string& name) {
    ImageResult result;
    if (data == nullptr) {
        const char* reason = stbi_failure_reason();
        result.error = "无法解码图像 " + name + "：" + (reason != nullptr ? reason : "未知原因");
        return result;
    }
    result.image.width = w;
    result.image.height = h;
    result.image.pixels.assign(data, data + static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 4u);
    stbi_image_free(data);
    return result;
}

}  // namespace

ImageResult loadImage(const std::string& path) {
    int w = 0;
    int h = 0;
    int channels = 0;
    unsigned char* data = stbi_load(path.c_str(), &w, &h, &channels, 4);
    return take(data, w, h, path);
}

ImageResult decodeImage(const std::uint8_t* data, std::size_t size, const std::string& name) {
    if (data == nullptr || size == 0 || size > static_cast<std::size_t>(INT_MAX)) {
        ImageResult result;
        result.error = "无法解码图像 " + name + "：数据为空或过大";
        return result;
    }
    int w = 0;
    int h = 0;
    int channels = 0;
    unsigned char* out = stbi_load_from_memory(data, static_cast<int>(size), &w, &h, &channels, 4);
    return take(out, w, h, name);
}

ImageRgba8 solidImage(int width, int height, std::uint8_t r, std::uint8_t g, std::uint8_t b, std::uint8_t a) {
    ImageRgba8 img;
    img.width = std::max(width, 1);
    img.height = std::max(height, 1);
    img.pixels.resize(static_cast<std::size_t>(img.width) * static_cast<std::size_t>(img.height) * 4u);
    for (std::size_t i = 0; i < img.pixels.size(); i += 4) {
        img.pixels[i] = r;
        img.pixels[i + 1] = g;
        img.pixels[i + 2] = b;
        img.pixels[i + 3] = a;
    }
    return img;
}

std::optional<std::string> savePng(const std::string& path, const ImageRgba8& image) {
    if (image.empty() || image.pixels.size() < static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height) * 4u) {
        return "图像为空，无法写入 " + path;
    }
    if (stbi_write_png(path.c_str(), image.width, image.height, 4, image.pixels.data(), image.width * 4) == 0) {
        return "无法写入 PNG：" + path;
    }
    return std::nullopt;
}

ImageRgba8 fromBottomUp(int width, int height, const std::vector<std::uint8_t>& pixels) {
    ImageRgba8 img;
    if (width <= 0 || height <= 0 || pixels.size() < static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4u) return img;
    img.width = width;
    img.height = height;
    img.pixels.resize(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4u);
    const std::size_t row = static_cast<std::size_t>(width) * 4u;
    for (int y = 0; y < height; ++y) {
        const std::size_t src = static_cast<std::size_t>(height - 1 - y) * row;
        std::copy(pixels.begin() + static_cast<std::ptrdiff_t>(src), pixels.begin() + static_cast<std::ptrdiff_t>(src + row),
                  img.pixels.begin() + static_cast<std::ptrdiff_t>(static_cast<std::size_t>(y) * row));
    }
    return img;
}

namespace {

int channelDifference(const ImageRgba8& a, const ImageRgba8& b, std::size_t pixel) {
    int worst = 0;
    for (std::size_t c = 0; c < 3; ++c) {
        const int d = std::abs(static_cast<int>(a.pixels[pixel * 4 + c]) - static_cast<int>(b.pixels[pixel * 4 + c]));
        worst = std::max(worst, d);
    }
    return worst;
}

}  // namespace

std::string CompareResult::summary() const {
    if (!sameSize) return "尺寸不同";
    char buf[128];
    std::snprintf(buf, sizeof(buf), "不同像素 %zu（%.3f%%），最大差 %d/255", differingPixels, differingFraction * 100.0, maxDifference);
    return buf;
}

CompareResult compareImages(const ImageRgba8& actual, const ImageRgba8& reference, const CompareOptions& options) {
    CompareResult r;
    r.sameSize = !actual.empty() && actual.width == reference.width && actual.height == reference.height &&
                 actual.pixels.size() == reference.pixels.size();
    if (!r.sameSize) return r;
    const std::size_t count = static_cast<std::size_t>(actual.width) * static_cast<std::size_t>(actual.height);
    for (std::size_t i = 0; i < count; ++i) {
        const int d = channelDifference(actual, reference, i);
        r.maxDifference = std::max(r.maxDifference, d);
        if (d > options.channelTolerance) ++r.differingPixels;
    }
    r.differingFraction = static_cast<double>(r.differingPixels) / static_cast<double>(count);
    r.passed = r.differingFraction <= options.maxDifferingFraction && r.maxDifference <= options.maxChannelDifference;
    return r;
}

ImageRgba8 differenceImage(const ImageRgba8& actual, const ImageRgba8& reference, int gain, int channelTolerance) {
    ImageRgba8 out;
    if (actual.empty() || actual.width != reference.width || actual.height != reference.height ||
        actual.pixels.size() != reference.pixels.size()) {
        return out;
    }
    out.width = actual.width;
    out.height = actual.height;
    out.pixels.resize(actual.pixels.size());
    const std::size_t count = static_cast<std::size_t>(actual.width) * static_cast<std::size_t>(actual.height);
    for (std::size_t i = 0; i < count; ++i) {
        const int d = channelDifference(actual, reference, i);
        const auto v = static_cast<std::uint8_t>(std::min(255, d * gain));
        const bool over = d > channelTolerance;
        out.pixels[i * 4] = over ? std::uint8_t{255} : v;
        out.pixels[i * 4 + 1] = over ? std::uint8_t{0} : v;
        out.pixels[i * 4 + 2] = over ? std::uint8_t{0} : v;
        out.pixels[i * 4 + 3] = 255;
    }
    return out;
}

}  // namespace terra::render
