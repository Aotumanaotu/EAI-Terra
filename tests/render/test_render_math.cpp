// 渲染数学测试（M1-08a）：网格生成与 PBR / 色调映射的 C++ 参考实现。纯 CPU，不需要 OpenGL 或显示。
//
// 网格：顶点/索引数、索引范围、单位法线、每个三角形的绕序与其顶点法线一致、凸体法线朝外、包围盒、
//       用散度定理求出的有向体积（为正且接近解析体积 → 表面封闭且整体朝外）。
// 着色：GGX、Smith 可见性、Schlick 的已知值；白色 Lambert 能量、金属无漫反射、背光为零、纯环境光；
//       曝光、色调映射的范围与单调性、PBR Neutral 保持中间调、sRGB 编码与 8 位量化。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include "terra/render/hdr_image.hpp"
#include "terra/render/mesh.hpp"
#include "terra/render/pbr.hpp"

namespace {

namespace r = terra::render;
namespace pbr = terra::render::pbr;
using r::Vec3f;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }
bool near(Vec3f a, Vec3f b, float tol) { return near(a.x, b.x, tol) && near(a.y, b.y, tol) && near(a.z, b.z, tol); }
std::string str(double v) { return std::to_string(v); }

struct MeshReport {
    bool indicesValid = true;
    bool normalsUnit = true;
    bool windingMatchesNormals = true;
    bool outward = true;
    double volume = 0.0;
};

MeshReport inspect(const r::MeshData& m, bool checkOutward) {
    MeshReport rep;
    if (m.indices.size() % 3 != 0) rep.indicesValid = false;
    for (const auto i : m.indices) {
        if (i >= m.vertices.size()) rep.indicesValid = false;
    }
    if (!rep.indicesValid) return rep;
    for (const auto& v : m.vertices) {
        const double len = std::sqrt(v.normal[0] * v.normal[0] + v.normal[1] * v.normal[1] + v.normal[2] * v.normal[2]);
        if (!near(len, 1.0, 1e-5)) rep.normalsUnit = false;
        const double d = v.normal[0] * v.position[0] + v.normal[1] * v.position[1] + v.normal[2] * v.position[2];
        if (checkOutward && d <= 0.0) rep.outward = false;
    }
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const auto& a = m.vertices[m.indices[t]];
        const auto& b = m.vertices[m.indices[t + 1]];
        const auto& c = m.vertices[m.indices[t + 2]];
        const double e1[3] = {double(b.position[0]) - a.position[0], double(b.position[1]) - a.position[1],
                              double(b.position[2]) - a.position[2]};
        const double e2[3] = {double(c.position[0]) - a.position[0], double(c.position[1]) - a.position[1],
                              double(c.position[2]) - a.position[2]};
        const double cr[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]};
        double nsum[3] = {0, 0, 0};
        for (const auto* v : {&a, &b, &c}) {
            for (int k = 0; k < 3; ++k) nsum[k] += v->normal[k];
        }
        const double area2 = std::sqrt(cr[0] * cr[0] + cr[1] * cr[1] + cr[2] * cr[2]);
        if (area2 < 1e-12 || cr[0] * nsum[0] + cr[1] * nsum[1] + cr[2] * nsum[2] <= 0.0) rep.windingMatchesNormals = false;
        // 有向体积：Σ a · (b × c) / 6
        const double bc[3] = {double(b.position[1]) * c.position[2] - double(b.position[2]) * c.position[1],
                              double(b.position[2]) * c.position[0] - double(b.position[0]) * c.position[2],
                              double(b.position[0]) * c.position[1] - double(b.position[1]) * c.position[0]};
        rep.volume += (a.position[0] * bc[0] + a.position[1] * bc[1] + a.position[2] * bc[2]) / 6.0;
    }
    return rep;
}

bool boundsAre(const r::MeshData& m, float x, float y, float z, float tol = 1e-5f) {
    return near(m.boundsMin[0], -x, tol) && near(m.boundsMin[1], -y, tol) && near(m.boundsMin[2], -z, tol) &&
           near(m.boundsMax[0], x, tol) && near(m.boundsMax[1], y, tol) && near(m.boundsMax[2], z, tol);
}

void checkSolid(const char* name, const r::MeshData& m, double expectedVolume, double relTol) {
    const MeshReport rep = inspect(m, true);
    const std::string n = name;
    check(rep.indicesValid, n + "：索引数为 3 的倍数且都在范围内");
    check(rep.normalsUnit, n + "：法线为单位向量");
    check(rep.windingMatchesNormals, n + "：每个三角形逆时针（与顶点法线同向）且无退化三角形");
    check(rep.outward, n + "：法线朝外");
    check(std::fabs(rep.volume - expectedVolume) <= relTol * expectedVolume,
          n + "：有向体积 " + str(rep.volume) + " 接近 " + str(expectedVolume) + "（表面封闭、整体朝外）");
}

void testMeshes() {
    constexpr double kPi = 3.14159265358979323846;
    const auto plane = r::meshes::plane(2.0f, 3.0f);
    const MeshReport pr = inspect(plane, false);
    check(plane.vertices.size() == 4 && plane.indices.size() == 6 && pr.windingMatchesNormals && pr.normalsUnit,
          "平面：4 个顶点、2 个三角形、法线 +Z 且逆时针");
    check(boundsAre(plane, 2.0f, 3.0f, 0.0f), "平面包围盒 ±2 × ±3");

    const auto box = r::meshes::box(1.0f, 2.0f, 3.0f);
    check(box.vertices.size() == 24 && box.indices.size() == 36, "立方体：24 个顶点、12 个三角形");
    check(boundsAre(box, 1.0f, 2.0f, 3.0f), "立方体包围盒为半边长");
    checkSolid("立方体", box, 8.0 * 1 * 2 * 3, 1e-6);

    const auto sphere8 = r::meshes::sphere(1.0f, 8, 4);
    check(sphere8.vertices.size() == 45 && sphere8.indices.size() == 144, "球（8×4）：45 个顶点、48 个三角形（两极无退化三角形）");
    const auto sphere = r::meshes::sphere(0.5f);
    check(boundsAre(sphere, 0.5f, 0.5f, 0.5f, 1e-4f), "球包围盒为半径");
    checkSolid("球", sphere, 4.0 / 3.0 * kPi * 0.125, 0.02);

    const auto cyl8 = r::meshes::cylinder(1.0f, 1.0f, 8);
    check(cyl8.vertices.size() == 38 && cyl8.indices.size() == 96, "圆柱（8 段）：38 个顶点、32 个三角形");
    const auto cyl = r::meshes::cylinder(0.3f, 0.8f);
    check(boundsAre(cyl, 0.3f, 0.3f, 0.8f, 1e-4f), "圆柱沿 Z：半径 0.3、半长 0.8");
    const double prism = 48.0 / 2.0 * 0.09 * std::sin(2.0 * kPi / 48.0) * 1.6;  // 正 48 边形棱柱的精确体积
    checkSolid("圆柱", cyl, prism, 1e-4);

    const auto cap8 = r::meshes::capsule(1.0f, 1.0f, 8, 2);
    check(cap8.vertices.size() == 54 && cap8.indices.size() == 192, "胶囊（8 段、每端 2 圈）：54 个顶点、64 个三角形");
    const auto cap = r::meshes::capsule(0.3f, 0.5f);
    check(boundsAre(cap, 0.3f, 0.3f, 0.8f, 1e-4f), "胶囊沿 Z：总半长 = 半长 0.5 + 半径 0.3");
    checkSolid("胶囊", cap, kPi * 0.09 * 1.0 + 4.0 / 3.0 * kPi * 0.027, 0.02);
}

void testShading() {
    // 已知值
    check(near(pbr::distributionGgx(1.0f, 0.25f), 1.0 / (3.14159265358979 * 0.0625), 1e-4), "GGX：N·H = 1 时为 1/(π α²)");
    check(near(pbr::visibilitySmithGgxCorrelated(1.0f, 1.0f, 0.3f), 0.25, 1e-6) &&
              near(pbr::visibilitySmithGgxCorrelated(1.0f, 1.0f, 1.0f), 0.25, 1e-6),
          "Smith 可见性：正对时为 1/4");
    check(near(pbr::fresnelSchlick({0.04f, 0.5f, 1.0f}, 1.0f), {0.04f, 0.5f, 1.0f}, 1e-6f) &&
              near(pbr::fresnelSchlick({0.04f, 0.04f, 0.04f}, 0.0f), {1.0f, 1.0f, 1.0f}, 1e-6f),
          "Schlick：正入射为 F0，掠射为 1");

    const Vec3f up{0.0f, 0.0f, 1.0f};
    r::SunLight sun;
    sun.direction = up;
    sun.color = {1.0f, 1.0f, 1.0f};
    sun.intensity = 3.14159265358979f;
    r::HemisphereLight dark;
    dark.intensity = 0.0f;

    r::Material white;
    white.baseColor = {1.0f, 1.0f, 1.0f};
    white.metallic = 0.0f;
    white.roughness = 1.0f;
    white.reflectance = 0.0f;
    check(near(pbr::shade(white, up, up, sun, dark, 1.0f), {1.0f, 1.0f, 1.0f}, 1e-5f),
          "白色 Lambert、F0 = 0、辐照度 π、正对光源：出射辐射为 1（能量守恒）");
    check(near(pbr::shade(white, up, up, sun, dark, 0.25f), {0.25f, 0.25f, 0.25f}, 1e-5f), "阴影系数按比例减弱直接光");

    r::Material redMetal = white;
    redMetal.baseColor = {1.0f, 0.0f, 0.0f};
    redMetal.metallic = 1.0f;
    check(near(pbr::shade(redMetal, up, up, sun, dark, 1.0f), {0.25f, 0.0f, 0.0f}, 1e-5f),
          "金属：无漫反射，镜面 = D·V·F·E = (1/π)·(1/4)·F0·π");

    r::SunLight behind = sun;
    behind.direction = {0.0f, 0.0f, -1.0f};
    check(near(pbr::shade(white, up, up, behind, dark, 1.0f), {0.0f, 0.0f, 0.0f}, 1e-7f), "背光且无环境光：黑色");

    r::HemisphereLight uniform;
    uniform.sky = {1.0f, 1.0f, 1.0f};
    uniform.ground = {1.0f, 1.0f, 1.0f};
    uniform.intensity = 1.0f;
    check(near(pbr::shade(white, up, up, behind, uniform, 1.0f), {1.0f, 1.0f, 1.0f}, 1e-5f),
          "均匀环境光 1、白色漫反射：出射辐射为 1（白炉）");
    r::Material glow = white;
    glow.baseColor = {0.0f, 0.0f, 0.0f};
    glow.emissive = {0.2f, 0.4f, 0.6f};
    check(near(pbr::shade(glow, up, up, behind, dark, 1.0f), {0.2f, 0.4f, 0.6f}, 1e-6f), "自发光直接相加");

    // 色调映射与编码
    check(near(pbr::toneMap({0.25f, 0.25f, 0.25f}, r::ToneMapper::None, 1.0f), {0.5f, 0.5f, 0.5f}, 1e-6f), "曝光 +1 EV 亮度加倍");
    check(near(pbr::toneMap({0.5f, 0.5f, 0.5f}, r::ToneMapper::PbrNeutral, 0.0f), {0.46f, 0.46f, 0.46f}, 1e-5f),
          "PBR Neutral：中间调只减去 0.04 的偏移");
    for (const auto mapper : {r::ToneMapper::AgX, r::ToneMapper::PbrNeutral, r::ToneMapper::Aces, r::ToneMapper::None}) {
        float previous = -1.0f;
        bool monotonic = true;
        bool inRange = true;
        for (int i = 0; i <= 200; ++i) {
            const float x = 64.0f * static_cast<float>(i * i) / 40000.0f;
            const float y = pbr::toneMap({x, x, x}, mapper, 0.0f).x;
            if (y < previous - 1e-6f) monotonic = false;
            if (y < 0.0f || y > 1.0f) inRange = false;
            previous = y;
        }
        const float zero = pbr::toneMap({0.0f, 0.0f, 0.0f}, mapper, 0.0f).x;
        const std::string name = std::to_string(static_cast<int>(mapper));
        check(monotonic && inRange, "色调映射 " + name + "：灰阶单调不减且在 [0, 1] 内");
        check(zero <= 1e-4f, "色调映射 " + name + "：黑色保持黑色");
    }
    check(pbr::toneMap({16.0f, 16.0f, 16.0f}, r::ToneMapper::AgX, 0.0f).x > 0.98f &&
              near(pbr::toneMap({0.18f, 0.18f, 0.18f}, r::ToneMapper::AgX, 0.0f).x, 0.2145, 0.002),
          "AgX：中灰 0.18 → 约 0.214，高光 16 接近 1");
    check(near(pbr::srgbEncode(0.5f), 0.735357, 1e-5) && near(pbr::srgbEncode(1.0f), 1.0, 1e-6) &&
              near(pbr::srgbEncode(0.002f), 0.02584, 1e-6),
          "sRGB 编码的已知值");
    const auto px = pbr::displayPixel({0.18f, 0.18f, 0.18f}, r::ToneMapper::None, 0.0f);
    check(px[0] == 118 && px[1] == 118 && px[2] == 118, "8 位量化：线性 0.18 → 118");
}

// ---- M1-08b：IBL 参考实现与 HDR 读取 ----

void testIblMath() {
    const int size = 32;
    const auto lut = pbr::dfgLut(size, 512);
    check(lut.size() == static_cast<std::size_t>(size * size * 2), "DFG 表大小为 32 × 32 × 2");
    bool inRange = true;
    bool energyBounded = true;
    for (std::size_t i = 0; i + 1 < lut.size(); i += 2) {
        if (lut[i] < 0.0f || lut[i + 1] < 0.0f || lut[i] > 1.05f || lut[i + 1] > 1.05f) inRange = false;
        if (lut[i] + lut[i + 1] > 1.02f) energyBounded = false;
    }
    check(inRange && energyBounded, "DFG：两项都在 [0, 1] 附近，A + B ≤ 1（不产生能量）");
    const auto smooth = pbr::sampleDfg(lut, size, 0.98f, 0.05f);
    check(near(smooth.x + smooth.y, 1.0, 0.03) && smooth.y < 0.05f,
          "光滑表面正对：A + B ≈ 1、B ≈ 0（实际 " + str(smooth.x) + " + " + str(smooth.y) + "）");
    const auto rough = pbr::sampleDfg(lut, size, 0.1f, 1.0f);
    check(rough.x + rough.y < 0.9f, "粗糙表面掠射：多次散射能量损失使 A + B < 0.9");
    // 双线性采样：纹素中心取原值，相邻两个纹素中心之间取平均
    const float c0 = (0.5f + 3.0f) / 32.0f;
    const float c1 = (0.5f + 4.0f) / 32.0f;
    const float r0 = (0.5f + 10.0f) / 32.0f;
    const auto t0 = pbr::sampleDfg(lut, size, c0, r0);
    const auto t1 = pbr::sampleDfg(lut, size, c1, r0);
    const auto mid = pbr::sampleDfg(lut, size, 0.5f * (c0 + c1), r0);
    check(near(t0.x, lut[static_cast<std::size_t>((10 * 32 + 3) * 2)], 1e-6) && near(mid.x, 0.5 * (t0.x + t1.x), 1e-5), "DFG 双线性采样与 GL_LINEAR 一致");

    // 白炉：白色金属、近镜面、常数环境 L → 出射约为 L
    r::Material chrome;
    chrome.baseColor = {1.0f, 1.0f, 1.0f};
    chrome.metallic = 1.0f;
    chrome.roughness = 0.0f;
    const Vec3f up{0.0f, 0.0f, 1.0f};
    const Vec3f L{0.5f, 0.5f, 0.5f};
    const Vec3f out = pbr::ambientIbl(chrome, up, up, L, L, lut, size);
    check(near(out, L, 0.02f), "白色金属镜面 + 常数环境：出射 ≈ 环境辐射（白炉）");
    r::Material white;
    white.baseColor = {1.0f, 1.0f, 1.0f};
    white.roughness = 1.0f;
    white.reflectance = 0.0f;
    const Vec3f diffuse = pbr::ambientIbl(white, up, up, L, Vec3f{}, lut, size);
    check(diffuse.x > 0.4f && diffuse.x <= 0.5f + 1e-5f, "白色漫反射 + 常数辐照度：出射略小于辐照度（F0 = 0 时镜面占比很小）");
}

void testHdr(const std::filesystem::path& dir) {
    // 手写一个 3 × 2 的 Radiance 文件（宽度 < 8 时为不压缩扫描线）。
    // 解码按 Radiance 参考实现：值 = (尾数 + 0.5) · 2^(指数 − 136)，指数为 0 时为 0
    // （stb_image 2.32 采用此规则；M1-08b 第一次基线运行时测试误用了不加 0.5 的旧规则）。
    const std::filesystem::path file = dir / "tiny.hdr";
    const unsigned char pixels[6][4] = {{128, 64, 32, 129}, {128, 0, 0, 130}, {0, 0, 0, 0},
                                        {0, 128, 0, 129},   {0, 0, 128, 129}, {64, 64, 64, 129}};
    {
        std::ofstream out(file, std::ios::binary);
        out << "#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n\n-Y 2 +X 3\n";
        for (const auto& px : pixels) out.write(reinterpret_cast<const char*>(px), 4);
    }
    const auto decode = [&](int index) {
        const unsigned char* px = pixels[index];
        if (px[3] == 0) return Vec3f{0.0f, 0.0f, 0.0f};
        const float scale = std::ldexp(1.0f, static_cast<int>(px[3]) - 136);
        return Vec3f{(static_cast<float>(px[0]) + 0.5f) * scale, (static_cast<float>(px[1]) + 0.5f) * scale,
                     (static_cast<float>(px[2]) + 0.5f) * scale};
    };
    const auto loaded = r::loadHdr(file.string());
    check(loaded.ok() && loaded.image.width == 3 && loaded.image.height == 2, "读取手写的 3 × 2 .hdr" + (loaded.ok() ? std::string{} : "：" + loaded.error));
    if (loaded.ok()) {
        check(near(loaded.image.at(0, 0), decode(0), 1e-6f) && near(loaded.image.at(1, 0), decode(1), 1e-6f) &&
                  near(loaded.image.at(2, 0), {0.0f, 0.0f, 0.0f}, 0.0f),
              "第 0 行（顶部）三个像素按 (尾数 + 0.5) · 2^(指数 − 136) 解码，指数 0 为 0");
        check(near(loaded.image.at(0, 1), decode(3), 1e-6f) && near(loaded.image.at(1, 1), decode(4), 1e-6f) &&
                  near(loaded.image.at(2, 1), decode(5), 1e-6f),
              "第 1 行像素正确（行序自上而下）");
        check(near(loaded.image.at(0, 0), {1.0f, 0.5f, 0.25f}, 0.005f), "解码值与名义值 (1, 0.5, 0.25) 相差不到半个量化步长");
    }
    const auto missing = r::loadHdr((dir / "no_such.hdr").string());
    check(!missing.ok() && missing.error.find("no_such.hdr") != std::string::npos, "文件不存在：失败并在原因中给出路径");

    // 等距柱状方向约定与合成环境
    check(near(r::equirectDirection(0.5f, 0.5f), {1.0f, 0.0f, 0.0f}, 1e-6f) && near(r::equirectDirection(0.75f, 0.5f), {0.0f, 1.0f, 0.0f}, 1e-6f) &&
              near(r::equirectDirection(0.3f, 0.0f), {0.0f, 0.0f, 1.0f}, 1e-6f),
          "等距柱状：u = 0.5 为 +X、u = 0.75 为 +Y，第 0 行为天顶");
    const auto ul = r::upperLowerHdr(8, 4, {1, 0, 0}, {0, 0, 1});
    const auto lr = r::leftRightHdr(8, 4, {1, 0, 0}, {0, 0, 1});
    check(near(ul.at(3, 0), {1, 0, 0}, 0) && near(ul.at(3, 3), {0, 0, 1}, 0), "上下两色环境：上半为天空色");
    check(near(lr.at(6, 2), {1, 0, 0}, 0) && near(lr.at(1, 2), {0, 0, 1}, 0), "左右两色环境：u ≥ 0.5（+Y 一侧）为第一种颜色");
    check(near(r::solidHdr(4, 2, {0.1f, 0.2f, 0.3f}).at(3, 1), {0.1f, 0.2f, 0.3f}, 0), "纯色环境");

    // 实际环境贴图（存在时）
    for (const char* name : {"rural_asphalt_road_2k.hdr", "studio_small_09_2k.hdr"}) {
        const std::string path = std::string{TERRA_ASSETS_DIR} + "/hdri/" + name;
        if (!std::filesystem::exists(path)) {
            std::printf("INFO 未找到 %s（未运行 scripts/assets/fetch_hdri.sh），跳过\n", path.c_str());
            continue;
        }
        const auto real = r::loadHdr(path);
        check(real.ok() && real.image.width == 2048 && real.image.height == 1024, std::string{name} + "：2048 × 1024");
        if (real.ok()) std::printf("INFO %s：%d × %d，天顶像素 (%.3f, %.3f, %.3f)\n", name, real.image.width, real.image.height,
                                   static_cast<double>(real.image.at(1024, 0).x), static_cast<double>(real.image.at(1024, 0).y),
                                   static_cast<double>(real.image.at(1024, 0).z));
    }
}

// ---- M1-09：切线、环境遮蔽、环境贴图中的太阳 ----

void testTangents() {
    const auto plane = r::meshes::plane(1.0f, 1.0f);
    bool planeOk = true;
    for (const auto& v : plane.vertices) {
        planeOk = planeOk && near(v.tangent[0], 1.0, 1e-6) && near(v.tangent[1], 0.0, 1e-6) && near(v.tangent[2], 0.0, 1e-6) &&
                  v.tangent[3] == 1.0f;
    }
    check(planeOk, "平面（UV = xy）：切线 = +X，手性 +1（副切线 = 法线 × 切线 = +Y，与 v 方向一致）");
    for (const auto& [name, mesh] : {std::pair<const char*, r::MeshData>{"立方体", r::meshes::box(0.5f, 0.5f, 0.5f)},
                                     std::pair<const char*, r::MeshData>{"球", r::meshes::sphere(1.0f, 16, 8)},
                                     std::pair<const char*, r::MeshData>{"圆柱", r::meshes::cylinder(0.5f, 1.0f, 16)},
                                     std::pair<const char*, r::MeshData>{"胶囊", r::meshes::capsule(0.5f, 0.5f, 16, 4)}}) {
        bool ok = true;
        for (const auto& v : mesh.vertices) {
            const double len = std::sqrt(v.tangent[0] * v.tangent[0] + v.tangent[1] * v.tangent[1] + v.tangent[2] * v.tangent[2]);
            const double d = v.tangent[0] * v.normal[0] + v.tangent[1] * v.normal[1] + v.tangent[2] * v.normal[2];
            ok = ok && near(len, 1.0, 1e-4) && near(d, 0.0, 1e-4) && (v.tangent[3] == 1.0f || v.tangent[3] == -1.0f);
        }
        check(ok, std::string{name} + "：切线为单位向量、与法线正交、手性为 ±1");
    }
}

void testOcclusion() {
    check(near(pbr::specularOcclusion(0.7f, 1.0f, 0.25f), 1.0, 1e-6) && near(pbr::specularOcclusion(0.7f, 0.0f, 0.25f), 0.0, 1e-6),
          "镜面遮蔽：ao = 1 时为 1，ao = 0 时为 0");
    const float mid = pbr::specularOcclusion(1.0f, 0.5f, 0.25f);
    check(mid > 0.4f && mid < 1.0f, "镜面遮蔽：正对时介于 ao 与 1 之间（实际 " + str(mid) + "）");
    const Vec3f up{0.0f, 0.0f, 1.0f};
    r::SunLight off;
    off.intensity = 0.0f;
    r::HemisphereLight white;
    white.sky = {1.0f, 1.0f, 1.0f};
    white.ground = {1.0f, 1.0f, 1.0f};
    white.intensity = 1.0f;
    r::Material m;
    m.baseColor = {1.0f, 1.0f, 1.0f};
    m.roughness = 1.0f;
    m.reflectance = 0.0f;
    const Vec3f full = pbr::shade(m, up, up, off, white, 1.0f, 1.0f);
    const Vec3f half = pbr::shade(m, up, up, off, white, 1.0f, 0.5f);
    check(near(half.x, 0.5 * full.x, 1e-5), "环境遮蔽 0.5：漫反射环境光减半");
    r::SunLight sun;
    sun.direction = up;
    r::HemisphereLight dark;
    dark.intensity = 0.0f;
    check(near(pbr::shade(m, up, up, sun, dark, 1.0f, 0.2f), pbr::shade(m, up, up, sun, dark, 1.0f, 1.0f), 1e-6f),
          "环境遮蔽不影响太阳直射光");
    check(near(pbr::srgbDecode(pbr::srgbEncode(0.3f)), 0.3, 1e-5) && near(pbr::srgbDecode(1.0f), 1.0, 1e-6), "sRGB 解码为编码的逆");
}

void testSunExtraction() {
    // 天空 0.5 + 半径约 2.5° 的太阳（方位 30°、高度 40°），亮度 5000
    r::HdrImage img = r::solidHdr(512, 256, {0.5f, 0.5f, 0.5f});
    const double az = 30.0 * 3.14159265358979 / 180.0;
    const double el = 40.0 * 3.14159265358979 / 180.0;
    const Vec3f sunDir{static_cast<float>(std::cos(el) * std::cos(az)), static_cast<float>(std::cos(el) * std::sin(az)),
                       static_cast<float>(std::sin(el))};
    const Vec3f sunColor{5000.0f, 4500.0f, 4000.0f};
    double expectedLum = 0.0;
    for (int y = 0; y < img.height; ++y) {
        const double w = (2.0 * 3.14159265358979 / img.width) * (3.14159265358979 / img.height) * std::sin((y + 0.5) * 3.14159265358979 / img.height);
        for (int x = 0; x < img.width; ++x) {
            const Vec3f d = r::equirectDirection((static_cast<float>(x) + 0.5f) / static_cast<float>(img.width),
                                                 (static_cast<float>(y) + 0.5f) / static_cast<float>(img.height));
            if (d.x * sunDir.x + d.y * sunDir.y + d.z * sunDir.z > std::cos(2.5 * 3.14159265358979 / 180.0)) {
                const std::size_t i = (static_cast<std::size_t>(y) * static_cast<std::size_t>(img.width) + static_cast<std::size_t>(x)) * 3u;
                img.rgb[i] = sunColor.x;
                img.rgb[i + 1] = sunColor.y;
                img.rgb[i + 2] = sunColor.z;
                expectedLum += static_cast<double>(r::luminance(sunColor)) * w;
            }
        }
    }
    const auto sun = r::extractSun(img, true);
    check(sun.has_value(), "合成环境：找到太阳");
    if (sun) {
        const double cosAngle = sun->direction.x * sunDir.x + sun->direction.y * sunDir.y + sun->direction.z * sunDir.z;
        check(cosAngle > std::cos(1.0 * 3.14159265358979 / 180.0), "太阳方向误差小于 1°");
        const double lum = r::luminance(sun->irradiance);
        check(lum > 0.97 * expectedLum && lum <= 1.0 * expectedLum,
              "太阳辐照度约等于太阳像素的亮度积分（实际 " + str(lum) + "，积分 " + str(expectedLum) + "）");
        check(sun->irradiance.x > sun->irradiance.z, "太阳颜色（偏暖）保留");
        float peak = 0.0f;
        for (int y = 0; y < img.height; ++y) {
            for (int x = 0; x < img.width; ++x) peak = std::max(peak, r::luminance(img.at(x, y)));
        }
        check(peak < 60.0f, "去掉太阳后环境中不再有极亮像素（实际峰值 " + str(peak) + "）");
    }
    // 没有太阳：纯色；大面积明亮区域（上半球 100）不算太阳
    r::HdrImage plain = r::solidHdr(64, 32, {1.0f, 1.0f, 1.0f});
    check(!r::extractSun(plain, false), "纯色环境：没有太阳");
    r::HdrImage bright = r::upperLowerHdr(64, 32, {100.0f, 100.0f, 100.0f}, {0.5f, 0.5f, 0.5f});
    check(!r::extractSun(bright, false), "大面积明亮区域（如摄影棚灯箱）不算太阳");
}

}  // namespace

int main(int argc, char** argv) {
    const std::filesystem::path dir = argc > 1 ? std::filesystem::path{argv[1]} : std::filesystem::temp_directory_path() / "terra_render_math";
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
    std::filesystem::create_directories(dir);
    testMeshes();
    testShading();
    testIblMath();
    testHdr(dir);
    testTangents();
    testOcclusion();
    testSunExtraction();
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
