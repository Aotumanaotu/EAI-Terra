// 界面文本与字体测试（M1-13，D-028、D-029）。不需要显示。
//   - 中英两张表覆盖全部 TextId，没有空字符串；语言切换即时生效
//   - 字体文件存在、大小与生成记录一致
//   - 界面用到的每一个字符（两种语言全部文本）在字体中都有字形：用 stb_truetype 查 cmap

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <unistd.h>

#define STB_TRUETYPE_IMPLEMENTATION
#define STBTT_STATIC
#include "imstb_truetype.h"

#include "terra/ui/file_browser.hpp"
#include "terra/ui/text.hpp"
#include "terra/ui/widgets.hpp"

#ifndef TERRA_ASSETS_DIR
#error "需要 TERRA_ASSETS_DIR"
#endif

namespace {

namespace ui = terra::ui;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

std::vector<unsigned char> readFile(const std::string& path) {
    std::ifstream in{path, std::ios::binary};
    return std::vector<unsigned char>{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

// UTF-8 → 码点（输入来自本项目的文本目录，假定合法）。
std::vector<char32_t> decode(const char* s) {
    std::vector<char32_t> out;
    const auto* p = reinterpret_cast<const unsigned char*>(s);
    while (*p != 0) {
        char32_t c = *p;
        int extra = 0;
        if (c >= 0xF0) { c &= 0x07; extra = 3; }
        else if (c >= 0xE0) { c &= 0x0F; extra = 2; }
        else if (c >= 0xC0) { c &= 0x1F; extra = 1; }
        ++p;
        for (int i = 0; i < extra && *p != 0; ++i, ++p) c = (c << 6) | static_cast<char32_t>(*p & 0x3Fu);
        out.push_back(c);
    }
    return out;
}

void testCatalog() {
    for (const ui::Language lang : {ui::Language::Chinese, ui::Language::English}) {
        for (int i = 0; i < ui::kTextCount; ++i) {
            const char* t = ui::text(static_cast<ui::TextId>(i), lang);
            check(t != nullptr && t[0] != '\0', std::string{ui::languageName(lang)} + " 文本 " + std::to_string(i) + " 为空");
        }
    }
    check(std::string{ui::text(ui::TextId::Count, ui::Language::Chinese)}.empty(), "越界 id 返回空字符串");
    ui::setLanguage(ui::Language::English);
    check(std::string{ui::tr(ui::TextId::MenuSettings)} == "Settings" && ui::language() == ui::Language::English, "切换为英文");
    ui::setLanguage(ui::Language::Chinese);
    check(std::string{ui::tr(ui::TextId::MenuSettings)} == "设置", "切换回中文");
    check(std::string{ui::languageName(ui::Language::Chinese)} == "中文" &&
              std::string{ui::languageName(ui::Language::English)} == "English",
          "语言名称");
}

void testFont(const char* file, std::size_t expectedBytes) {
    const std::string path = std::string{TERRA_ASSETS_DIR} + "/fonts/" + file;
    std::vector<unsigned char> data = readFile(path);
    check(data.size() == expectedBytes,
          std::string{file} + " 大小 " + std::to_string(data.size()) + " 与生成记录 " + std::to_string(expectedBytes) + " 一致");
    if (data.empty()) return;
    stbtt_fontinfo info;
    if (!stbtt_InitFont(&info, data.data(), stbtt_GetFontOffsetForIndex(data.data(), 0))) {
        check(false, std::string{file} + " 能被 stb_truetype 解析");
        return;
    }
    int checked = 0;
    std::string missing;
    for (const ui::Language lang : {ui::Language::Chinese, ui::Language::English}) {
        for (int i = 0; i < ui::kTextCount; ++i) {
            for (const char32_t c : decode(ui::text(static_cast<ui::TextId>(i), lang))) {
                if (c == U'\n') continue;
                ++checked;
                if (stbtt_FindGlyphIndex(&info, static_cast<int>(c)) == 0) {
                    char buf[16];
                    std::snprintf(buf, sizeof(buf), " U+%04X", static_cast<unsigned>(c));
                    missing += buf;
                }
            }
        }
    }
    std::printf("INFO %s：%zu 字节，检查了 %d 个字符\n", file, data.size(), checked);
    check(missing.empty(), std::string{file} + " 缺少字形：" + missing);
}

}  // namespace

// 图表纵轴的自动上限（M1-15）
void testNiceCeiling() {
    namespace ui = terra::ui;
    const auto eq = [](double a, double b) { return std::fabs(a - b) < 1e-9; };
    check(eq(ui::niceCeiling(3.0), 5.0) && eq(ui::niceCeiling(16.7), 20.0) && eq(ui::niceCeiling(2.2), 2.5) && eq(ui::niceCeiling(0.7), 1.0) &&
              eq(ui::niceCeiling(1.0), 1.0) && eq(ui::niceCeiling(95.0), 100.0) && eq(ui::niceCeiling(0.0), 1.0),
          "图表上限取 1、2、2.5、5 × 10ⁿ 中不小于数值的最小者");
}

// 文件浏览窗口的目录读取（M2-08，纯函数，不需要界面）
void testFileBrowserListing() {
    namespace fs = std::filesystem;
    namespace ui = terra::ui;
    const fs::path root = fs::temp_directory_path() / ("terra_file_browser_" + std::to_string(::getpid()));
    std::error_code ec;
    fs::remove_all(root, ec);
    fs::create_directories(root / "b_dir");
    fs::create_directories(root / "a_dir");
    fs::create_directories(root / ".hidden_dir");
    for (const char* name : {"z.terra-scene.json", "m.terra-scene.json", "other.json", "notes.txt", ".hidden.terra-scene.json"}) {
        std::ofstream(root / name) << "{}";
    }
    const auto scenes = ui::listDirectory(root.string(), ".terra-scene.json");
    std::vector<std::string> names;
    for (const auto& e : scenes.entries) names.push_back(e.name);
    check(scenes.error.empty() && names == std::vector<std::string>{"a_dir", "b_dir", "m.terra-scene.json", "z.terra-scene.json"},
          "只列出目录与场景文件，目录在前、按名称排序、不含隐藏项");
    check(scenes.entries.size() == 4 && scenes.entries[0].directory && !scenes.entries[2].directory && scenes.entries[2].size == 2,
          "目录标记与文件大小");
    const auto json = ui::listDirectory(root.string(), ".json");
    check(json.entries.size() == 5, "过滤 .json：两个目录与三个 JSON 文件");
    check(ui::listDirectory(root.string(), "").entries.size() == 6, "不过滤：全部非隐藏项");
    check(!ui::listDirectory((root / "no_such").string(), "").error.empty(), "目录不存在时给出错误");
    check(ui::normalizeDirectory((root / "m.terra-scene.json").string()) == root.string(), "文件路径 → 所在目录");
    check(ui::normalizeDirectory((root / "a_dir" / "x" / "y").string()) == (root / "a_dir").string(), "不存在的路径 → 最近的存在的上级");
    check(ui::normalizeDirectory((root / "b_dir").string() + "/").back() != '/', "去掉末尾斜杠");
    fs::remove_all(root, ec);
}

int main() {
    testFileBrowserListing();
    testNiceCeiling();
    testCatalog();
    testFont("NotoSansSC-Regular-Subset.otf", 1956920);
    testFont("NotoSansSC-Bold-Subset.otf", 1993812);
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
