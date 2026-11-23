#pragma once

// Terra Render：8 位图像（M1-09，glTF 贴图）。PNG 与 JPEG 由 stb_image 解码，统一转为 RGBA8，行 0 为图像顶部。
// 纯 CPU，不需要 OpenGL。

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace terra::render {

struct ImageRgba8 {
    int width = 0;
    int height = 0;
    std::vector<std::uint8_t> pixels;  // width × height × 4

    bool empty() const noexcept { return width <= 0 || height <= 0; }
};

struct ImageResult {
    ImageRgba8 image;
    std::string error;  // 失败原因（含来源说明）
    bool ok() const noexcept { return !image.empty(); }
};

ImageResult loadImage(const std::string& path);
// 从内存解码（glb 内嵌或 data URI 的图像）；name 只用于诊断。
ImageResult decodeImage(const std::uint8_t* data, std::size_t size, const std::string& name);
// 整幅同一颜色（测试与缺失贴图时的占位）。
ImageRgba8 solidImage(int width, int height, std::uint8_t r, std::uint8_t g, std::uint8_t b, std::uint8_t a = 255);

// ---- 参考图（M1-10）----
// 写 PNG（stb_image_write）；失败时返回原因。
std::optional<std::string> savePng(const std::string& path, const ImageRgba8& image);
// OpenGL 回读的像素行序自下而上，转换为图像约定（行 0 为顶部）。pixels 为 width × height × 4 字节。
ImageRgba8 fromBottomUp(int width, int height, const std::vector<std::uint8_t>& pixels);

// 比较规则：某像素任一颜色通道（不含透明度）之差 > channelTolerance 即为“不同像素”；
// 通过条件：尺寸相同，不同像素占比 ≤ maxDifferingFraction，且所有通道最大差 ≤ maxChannelDifference。
struct CompareOptions {
    int channelTolerance = 3;
    double maxDifferingFraction = 0.001;
    int maxChannelDifference = 16;
};

struct CompareResult {
    bool sameSize = false;
    std::size_t differingPixels = 0;
    double differingFraction = 0.0;
    int maxDifference = 0;
    bool passed = false;
    std::string summary() const;  // 例如“不同像素 12（0.005%），最大差 5/255”
};

CompareResult compareImages(const ImageRgba8& actual, const ImageRgba8& reference, const CompareOptions& options = {});
// 差异图：灰度为三通道最大差 × gain（截断到 255）；不同像素（超过容差）标为红色。尺寸不同时返回空图。
ImageRgba8 differenceImage(const ImageRgba8& actual, const ImageRgba8& reference, int gain = 8, int channelTolerance = 3);

}  // namespace terra::render
