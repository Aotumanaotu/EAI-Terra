// 参考图比较规则测试（M1-10）。纯 CPU，不需要 OpenGL。
// 覆盖：相同图通过；容差内的差异不计；少量超差像素通过、超差比例过大失败；单个像素差异过大失败；尺寸不同失败；
// 差异图（超差像素为红、其余为放大的灰度）；自下而上 → 自上而下的行序转换；PNG 写入后读回逐字节相同。
//
// 用法：test_image_compare <临时目录>

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <string>

#include "terra/render/image.hpp"

namespace {

namespace r = terra::render;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

void setPixel(r::ImageRgba8& img, int x, int y, std::uint8_t red, std::uint8_t green, std::uint8_t blue) {
    const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(x)) * 4u;
    img.pixels[i] = red;
    img.pixels[i + 1] = green;
    img.pixels[i + 2] = blue;
}

void testRules() {
    const r::ImageRgba8 reference = r::solidImage(100, 100, 100, 120, 140);  // 10000 个像素
    r::CompareResult same = r::compareImages(reference, reference);
    check(same.sameSize && same.passed && same.differingPixels == 0 && same.maxDifference == 0, "相同图像：通过，差异为 0");

    r::ImageRgba8 tolerated = reference;
    for (int x = 0; x < 100; ++x) setPixel(tolerated, x, 5, 103, 117, 140);  // 差 3：容差内
    const auto t = r::compareImages(tolerated, reference);
    check(t.passed && t.differingPixels == 0 && t.maxDifference == 3, "通道差 3（容差内）：不计为不同像素");

    r::ImageRgba8 few = reference;
    for (int x = 0; x < 10; ++x) setPixel(few, x, 0, 110, 120, 140);  // 10 个像素差 10：0.1%
    const auto f = r::compareImages(few, reference);
    check(f.passed && f.differingPixels == 10 && f.maxDifference == 10, "10 个像素差 10（0.1%）：通过（等于上限）\n" + f.summary());

    r::ImageRgba8 many = reference;
    for (int x = 0; x < 11; ++x) setPixel(many, x, 0, 110, 120, 140);  // 11 个像素：0.11%
    const auto m = r::compareImages(many, reference);
    check(!m.passed && m.differingPixels == 11, "11 个像素差 10（0.11%）：超过比例上限，失败");

    r::ImageRgba8 spike = reference;
    setPixel(spike, 50, 50, 100, 120, 157);  // 单像素差 17
    const auto s = r::compareImages(spike, reference);
    check(!s.passed && s.differingPixels == 1 && s.maxDifference == 17, "单个像素差 17（超过最大差 16）：失败");

    r::ImageRgba8 alphaOnly = reference;
    alphaOnly.pixels[3] = 0;
    check(r::compareImages(alphaOnly, reference).passed, "透明度不参与比较");

    const r::ImageRgba8 other = r::solidImage(100, 50, 100, 120, 140);
    const auto size = r::compareImages(other, reference);
    check(!size.sameSize && !size.passed && size.summary() == "尺寸不同", "尺寸不同：失败");

    // 差异图
    r::ImageRgba8 mixed = reference;
    setPixel(mixed, 1, 0, 102, 120, 140);  // 差 2：灰度 2 × 8 = 16
    setPixel(mixed, 2, 0, 130, 120, 140);  // 差 30：超差，红色
    const r::ImageRgba8 diff = r::differenceImage(mixed, reference);
    check(diff.width == 100 && diff.height == 100, "差异图尺寸与原图相同");
    if (!diff.empty()) {
        check(diff.pixels[0] == 0 && diff.pixels[1] == 0 && diff.pixels[2] == 0, "差异图：相同像素为黑");
        check(diff.pixels[4] == 16 && diff.pixels[5] == 16 && diff.pixels[6] == 16, "差异图：容差内的差异放大 8 倍为灰度");
        check(diff.pixels[8] == 255 && diff.pixels[9] == 0 && diff.pixels[10] == 0, "差异图：超差像素为红色");
    }
    check(r::differenceImage(other, reference).empty(), "尺寸不同：差异图为空");
}

void testRowOrderAndPng(const std::filesystem::path& dir) {
    // 2 × 2：OpenGL 回读的第 0 行是底部
    const std::vector<std::uint8_t> bottomUp = {1, 1, 1, 255, 2, 2, 2, 255,   // 底行
                                                3, 3, 3, 255, 4, 4, 4, 255};  // 顶行
    const r::ImageRgba8 img = r::fromBottomUp(2, 2, bottomUp);
    check(img.width == 2 && img.pixels[0] == 3 && img.pixels[4] == 4 && img.pixels[8] == 1 && img.pixels[12] == 2,
          "自下而上 → 自上而下：顶行在前");
    r::ImageRgba8 pattern = r::solidImage(7, 5, 10, 20, 30);
    setPixel(pattern, 6, 4, 200, 100, 50);
    const std::string path = (dir / "pattern.png").string();
    const auto saved = r::savePng(path, pattern);
    check(!saved, "写入 PNG" + (saved ? "：" + *saved : std::string{}));
    const auto loaded = r::loadImage(path);
    check(loaded.ok() && loaded.image.pixels == pattern.pixels, "PNG 读回与写入的像素逐字节相同");
    check(r::savePng((dir / "no_such_dir" / "x.png").string(), pattern).has_value(), "目录不存在时写入失败并给出原因");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "用法：test_image_compare <临时目录>\n");
        return 2;
    }
    const std::filesystem::path dir{argv[1]};
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
    std::filesystem::create_directories(dir);
    testRules();
    testRowOrderAndPng(dir);
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
