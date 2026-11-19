// 渲染数学测试（M1-08a）：网格生成与 PBR / 色调映射的 C++ 参考实现。纯 CPU，不需要 OpenGL 或显示。
//
// 网格：顶点/索引数、索引范围、单位法线、每个三角形的绕序与其顶点法线一致、凸体法线朝外、包围盒、
//       用散度定理求出的有向体积（为正且接近解析体积 → 表面封闭且整体朝外）。
// 着色：GGX、Smith 可见性、Schlick 的已知值；白色 Lambert 能量、金属无漫反射、背光为零、纯环境光；
//       曝光、色调映射的范围与单调性、PBR Neutral 保持中间调、sRGB 编码与 8 位量化。

#include <cmath>
#include <cstdio>
#include <string>

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

}  // namespace

int main() {
    testMeshes();
    testShading();
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
