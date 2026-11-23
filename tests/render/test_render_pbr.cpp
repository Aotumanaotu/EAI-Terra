// PBR 场景渲染测试（M1-08a）。Xvfb + llvmpipe（D-022）中离屏渲染并回读像素，与 C++ 参考实现（terra/render/pbr.hpp）比较。
//
// 覆盖：正对光源的非金属与金属球、带环境光、光源斜 45°；四种色调映射与曝光；背光为黑；地面阴影（阴影区 = 只有环境光、受光区 = 完整光照）；
// 阴影统计；4× MSAA 在边缘产生中间值、单采样没有；地面网格的 X 轴为红、Y 轴为绿；尺寸变化；
// M1-08b：合成环境（常数、上下两色、左右两色）下的环境光照、镜面反射方向、环境旋转与背景；全程无 GL 错误。
//
// 用法：test_render_pbr [期望的渲染器子串]

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <tuple>
#include <vector>

#include "terra/input/input_system.hpp"
#include "terra/platform/glfw/glfw_platform.hpp"
#include "terra/render/context.hpp"
#include "terra/render/environment.hpp"
#include "terra/render/hdr_image.hpp"
#include "terra/render/mesh.hpp"
#include "terra/render/pbr.hpp"
#include "terra/render/resources.hpp"
#include "terra/render/scene_renderer.hpp"
#include "terra/view/camera.hpp"

namespace {

namespace r = terra::render;
namespace pbr = terra::render::pbr;
namespace in = terra::input;
using r::Vec3f;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

struct Rgb {
    int r = 0;
    int g = 0;
    int b = 0;
};

std::string str(Rgb c) {
    return "(" + std::to_string(c.r) + ", " + std::to_string(c.g) + ", " + std::to_string(c.b) + ")";
}

bool near(Rgb a, Rgb b, int tol) { return std::abs(a.r - b.r) <= tol && std::abs(a.g - b.g) <= tol && std::abs(a.b - b.b) <= tol; }

Rgb expected(Vec3f radiance, r::ToneMapper mapper, float ev) {
    const auto p = pbr::displayPixel(radiance, mapper, ev);
    return {p[0], p[1], p[2]};
}

Vec3f normalize(Vec3f v) {
    const float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    return {v.x / len, v.y / len, v.z / len};
}

struct View {
    terra::view::Camera camera;
    r::ViewParams params;
    int width = 128;
    int height = 128;

    View(Vec3f eye, Vec3f target, int w, int h) : width(w), height(h) {
        camera.position = {eye.x, eye.y, eye.z};
        camera.lookAt({target.x, target.y, target.z});
        params.view = camera.viewMatrix();
        params.projection = camera.projectionMatrix(static_cast<double>(w) / static_cast<double>(h));
        params.cameraPosition = eye;
    }

    // 世界坐标 → 像素（原点左下，与回读一致）；在视野外时返回 false。
    bool project(Vec3f p, int& x, int& y) const {
        const auto& v = params.view;
        const auto& pr = params.projection;
        float e[4];
        for (int row = 0; row < 4; ++row) {
            e[row] = v[static_cast<std::size_t>(row)] * p.x + v[static_cast<std::size_t>(4 + row)] * p.y +
                     v[static_cast<std::size_t>(8 + row)] * p.z + v[static_cast<std::size_t>(12 + row)];
        }
        float c[4];
        for (int row = 0; row < 4; ++row) {
            c[row] = 0.0f;
            for (int k = 0; k < 4; ++k) c[row] += pr[static_cast<std::size_t>(k * 4 + row)] * e[k];
        }
        if (c[3] <= 0.0f) return false;
        x = static_cast<int>(std::floor((c[0] / c[3] * 0.5f + 0.5f) * static_cast<float>(width)));
        y = static_cast<int>(std::floor((c[1] / c[3] * 0.5f + 0.5f) * static_cast<float>(height)));
        return x >= 0 && y >= 0 && x < width && y < height;
    }
};

Rgb pixel(const r::SceneRenderer& renderer, int x, int y) {
    const auto p = renderer.readOutput(x, y, 1, 1);
    return p.size() >= 3 ? Rgb{p[0], p[1], p[2]} : Rgb{-1, -1, -1};
}

r::RenderSettings plainSettings() {
    r::RenderSettings s;
    s.grid = false;
    s.background = {0.0f, 0.0f, 0.0f};
    return s;
}

void testSphere(r::SceneRenderer& renderer, const r::GpuMesh& sphere) {
    const View view({-4.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, 128, 128);
    r::SunLight sun;
    sun.direction = {-1.0f, 0.0f, 0.0f};  // 从相机方向照射
    sun.color = {1.0f, 0.96f, 0.9f};
    sun.intensity = 3.0f;
    r::HemisphereLight dark;
    dark.intensity = 0.0f;
    r::HemisphereLight env;  // 默认天空与地面色，强度 0.6
    const Vec3f n{-1.0f, 0.0f, 0.0f};
    const Vec3f v{-1.0f, 0.0f, 0.0f};

    r::Material plastic;
    plastic.baseColor = {0.8f, 0.3f, 0.2f};
    plastic.roughness = 0.5f;
    r::Material gold;
    gold.baseColor = {1.0f, 0.78f, 0.34f};
    gold.metallic = 1.0f;
    // 粗糙度 0.7：正对时镜面约为 F0 量级，不会过曝到 255（粗糙度 0.3 时高光饱和，金属是否带漫反射看不出来——
    // M1-08a 第一次基线运行中“金属也有漫反射”的变异因此未被发现）
    gold.roughness = 0.7f;

    struct Case {
        const char* name;
        r::Material material;
        r::HemisphereLight env;
        r::ToneMapper mapper;
        float ev;
    };
    const Case cases[] = {
        {"非金属，正对光源，AgX", plastic, dark, r::ToneMapper::AgX, 0.0f},
        {"金属（金，粗糙 0.7），正对光源，AgX", gold, dark, r::ToneMapper::AgX, 0.0f},
        {"金属（金）+ 半球环境光，PBR Neutral", gold, env, r::ToneMapper::PbrNeutral, 0.0f},
        {"非金属 + 半球环境光，AgX", plastic, env, r::ToneMapper::AgX, 0.0f},
        {"非金属，PBR Neutral", plastic, dark, r::ToneMapper::PbrNeutral, 0.0f},
        {"非金属，ACES", plastic, dark, r::ToneMapper::Aces, 0.0f},
        {"非金属，无色调映射，曝光 -1 EV", plastic, dark, r::ToneMapper::None, -1.0f},
    };
    for (const Case& c : cases) {
        r::RenderSettings rs = plainSettings();
        rs.toneMapper = c.mapper;
        rs.exposureEv = c.ev;
        const std::vector<r::RenderItem> items{{&sphere, r::identityMatrix(), c.material, true}};
        renderer.render(view.width, view.height, view.params, items, sun, c.env, rs);
        const Rgb got = pixel(renderer, 64, 64);
        const Rgb want = expected(pbr::shade(c.material, n, v, sun, c.env, 1.0f), c.mapper, c.ev);
        std::printf("INFO %s：中心像素 %s，参考 %s\n", c.name, str(got).c_str(), str(want).c_str());
        check(near(got, want, 3), std::string{c.name} + "：中心像素 " + str(got) + " 与参考 " + str(want) + " 相差不超过 3");
    }
    // 光源斜 45°（从上前方照射）：N·L ≈ 0.707，检查直接光按 N·L 衰减；此时太阳在地平线以上，阴影图参与绘制
    {
        r::SunLight oblique = sun;
        oblique.direction = normalize({-1.0f, 0.0f, 1.0f});
        const std::vector<r::RenderItem> one{{&sphere, r::identityMatrix(), plastic, true}};
        renderer.render(view.width, view.height, view.params, one, oblique, dark, plainSettings());
        const Rgb got = pixel(renderer, 64, 64);
        const Rgb want = expected(pbr::shade(plastic, n, v, oblique, dark, 1.0f), r::ToneMapper::AgX, 0.0f);
        std::printf("INFO 光源斜 45°：中心像素 %s，参考 %s\n", str(got).c_str(), str(want).c_str());
        check(near(got, want, 3), "光源斜 45°：中心像素 " + str(got) + " 与参考 " + str(want) + " 相差不超过 3");
        check(renderer.lastFrame().shadowsDrawn, "太阳在地平线以上：绘制阴影图");
    }
    // 背光、无环境光 → 黑
    r::SunLight behind = sun;
    behind.direction = {1.0f, 0.0f, 0.0f};
    const std::vector<r::RenderItem> items{{&sphere, r::identityMatrix(), plastic, true}};
    renderer.render(view.width, view.height, view.params, items, behind, dark, plainSettings());
    const Rgb back = pixel(renderer, 64, 64);
    check(near(back, Rgb{0, 0, 0}, 1), "背光且无环境光：中心为黑色，实际 " + str(back));
    // 背景：线性 0 → 黑；角落像素
    check(near(pixel(renderer, 2, 2), Rgb{0, 0, 0}, 1), "背景为清屏颜色（黑）");
    check(!renderer.lastFrame().shadowsDrawn, "太阳在地平线以下（方向 z ≤ 0）时不绘制阴影");
}

void testShadow(r::SceneRenderer& renderer, const r::GpuMesh& plane, const r::GpuMesh& box) {
    const Vec3f eye{-6.0f, 0.0f, 3.0f};
    const View view(eye, {0.0f, 0.0f, 0.0f}, 160, 120);
    r::SunLight sun;
    sun.direction = {0.0f, 0.0f, 1.0f};  // 正上方
    sun.intensity = 3.0f;
    r::HemisphereLight env;
    env.intensity = 0.5f;
    r::Material ground;
    ground.baseColor = {0.5f, 0.5f, 0.5f};
    ground.roughness = 0.9f;
    r::Material boxMaterial;
    boxMaterial.baseColor = {0.2f, 0.4f, 0.8f};

    std::vector<r::RenderItem> items{{&plane, r::identityMatrix(), ground, false},
                                     {&box, r::translationScale({0.0f, 0.0f, 1.5f}), boxMaterial, true}};
    r::RenderSettings rs = plainSettings();
    renderer.render(view.width, view.height, view.params, items, sun, env, rs);
    check(renderer.lastFrame().shadowsDrawn && renderer.lastFrame().shadowCasters == 1, "绘制了阴影图，投影物体 1 个（地面不投影）");

    const Vec3f up{0.0f, 0.0f, 1.0f};
    const auto shadeAt = [&](Vec3f p, float shadow) {
        const Vec3f v = normalize({eye.x - p.x, eye.y - p.y, eye.z - p.z});
        return expected(pbr::shade(ground, up, v, sun, env, shadow), rs.toneMapper, rs.exposureEv);
    };
    int x = 0;
    int y = 0;
    const Vec3f underBox{0.0f, 0.0f, 0.0f};
    const Vec3f open{0.0f, 2.5f, 0.0f};
    if (view.project(underBox, x, y)) {
        const Rgb got = pixel(renderer, x, y);
        const Rgb want = shadeAt(underBox, 0.0f);
        std::printf("INFO 阴影区（箱子正下方）像素 %s，参考（只有环境光）%s\n", str(got).c_str(), str(want).c_str());
        check(near(got, want, 3), "阴影区只剩环境光：" + str(got) + " ≈ " + str(want));
    } else {
        check(false, "阴影区在视野内");
    }
    Rgb lit;
    if (view.project(open, x, y)) {
        lit = pixel(renderer, x, y);
        const Rgb want = shadeAt(open, 1.0f);
        std::printf("INFO 受光区像素 %s，参考 %s\n", str(lit).c_str(), str(want).c_str());
        check(near(lit, want, 3), "受光区完整光照：" + str(lit) + " ≈ " + str(want));
    } else {
        check(false, "受光区在视野内");
    }

    // 关闭阴影：箱子下方恢复完整光照
    rs.shadows = false;
    renderer.render(view.width, view.height, view.params, items, sun, env, rs);
    check(!renderer.lastFrame().shadowsDrawn, "关闭阴影后不绘制阴影图");
    if (view.project(underBox, x, y)) {
        const Rgb got = pixel(renderer, x, y);
        check(near(got, shadeAt(underBox, 1.0f), 3), "关闭阴影：箱子下方为完整光照，实际 " + str(got));
    }
}

int countIntermediate(const r::SceneRenderer& renderer, int row, int width, Rgb full) {
    const auto p = renderer.readOutput(0, row, width, 1);
    int count = 0;
    for (int i = 0; i < width; ++i) {
        const int v = p[static_cast<std::size_t>(i) * 4];
        if (v > 2 && v < full.r - 2) ++count;
    }
    return count;
}

void testMsaa(r::SceneRenderer& renderer, const r::GpuMesh& sphere, int maxSamples) {
    const View view({-4.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, 96, 96);
    r::SunLight off;
    off.intensity = 0.0f;
    r::HemisphereLight dark;
    dark.intensity = 0.0f;
    r::Material glow;
    glow.baseColor = {0.0f, 0.0f, 0.0f};
    glow.emissive = {0.5f, 0.5f, 0.5f};
    const std::vector<r::RenderItem> items{{&sphere, r::identityMatrix(), glow, true}};
    r::RenderSettings rs = plainSettings();
    rs.toneMapper = r::ToneMapper::None;
    rs.shadows = false;

    rs.msaaSamples = 1;
    renderer.render(view.width, view.height, view.params, items, off, dark, rs);
    const Rgb full = pixel(renderer, 48, 48);
    check(near(full, expected(glow.emissive, r::ToneMapper::None, 0.0f), 1), "自发光球中心为自发光颜色，实际 " + str(full));
    const int single = countIntermediate(renderer, 48, view.width, full);
    check(renderer.lastFrame().samples == 1 && single == 0, "单采样：边缘没有中间值（实际 " + std::to_string(single) + " 个）");

    rs.msaaSamples = 4;
    renderer.render(view.width, view.height, view.params, items, off, dark, rs);
    const int used = renderer.lastFrame().samples;
    const int multi = countIntermediate(renderer, 48, view.width, full);
    std::printf("INFO MSAA：请求 4×，驱动上限 %d×，实际 %d×；中心行边缘中间值 %d 个\n", maxSamples, used, multi);
    check(used == std::min(4, maxSamples), "实际采样数为 min(4, 驱动上限)");
    if (used > 1) check(multi >= 2, "多重采样：球的左右边缘出现中间值");
}

void testGrid(r::SceneRenderer& renderer) {
    const View view({-3.0f, -3.0f, 5.0f}, {1.5f, 1.5f, 0.0f}, 160, 160);
    r::SunLight sun;
    r::HemisphereLight env;
    r::RenderSettings rs;
    rs.background = {0.0f, 0.0f, 0.0f};
    renderer.render(view.width, view.height, view.params, {}, sun, env, rs);
    int x = 0;
    int y = 0;
    if (view.project({3.0f, 0.0f, 0.0f}, x, y)) {
        const Rgb c = pixel(renderer, x, y);
        check(c.r > c.g + 20 && c.r > c.b + 20, "网格：X 轴（y = 0）为红色，实际 " + str(c));
    } else {
        check(false, "X 轴上的点在视野内");
    }
    if (view.project({0.0f, 3.0f, 0.0f}, x, y)) {
        const Rgb c = pixel(renderer, x, y);
        check(c.g > c.r + 20 && c.g > c.b + 20, "网格：Y 轴（x = 0）为绿色，实际 " + str(c));
    } else {
        check(false, "Y 轴上的点在视野内");
    }
    if (view.project({2.5f, 2.5f, 0.0f}, x, y)) {
        const Rgb c = pixel(renderer, x, y);
        check(c.r < 40 && c.g < 40 && c.b < 40, "网格：格子中间接近背景，实际 " + str(c));
    }
    rs.grid = false;
    renderer.render(view.width, view.height, view.params, {}, sun, env, rs);
    if (view.project({3.0f, 0.0f, 0.0f}, x, y)) {
        check(near(pixel(renderer, x, y), Rgb{0, 0, 0}, 1), "关闭网格后 X 轴位置为背景");
    }
}

// ---- M1-08b：基于图像的光照（合成环境，结果可解析）----

Vec3f reflectDir(Vec3f i, Vec3f n) {
    const float d = 2.0f * (n.x * i.x + n.y * i.y + n.z * i.z);
    return {i.x - d * n.x, i.y - d * n.y, i.z - d * n.z};
}

void testIbl(r::RenderContext& ctx, r::SceneRenderer& renderer, const r::GpuMesh& sphere) {
    r::EnvironmentOptions o;  // 测试用较小尺寸，llvmpipe 上预计算更快
    o.cubeSize = 128;
    o.irradianceSize = 16;
    o.prefilterSize = 64;
    o.prefilterMips = 5;
    o.dfgSize = 32;
    o.dfgSamples = 256;
    const Vec3f L{0.5f, 0.4f, 0.3f};
    const Vec3f A{0.9f, 0.6f, 0.3f};  // 上半球（天空）
    const Vec3f B{0.1f, 0.2f, 0.4f};  // 下半球
    const Vec3f C{0.9f, 0.2f, 0.2f};  // +Y 一侧
    const Vec3f D{0.2f, 0.2f, 0.9f};  // −Y 一侧
    auto solid = r::Environment::create(ctx, r::solidHdr(64, 32, L), o);
    auto upperLower = r::Environment::create(ctx, r::upperLowerHdr(64, 32, A, B), o);
    auto leftRight = r::Environment::create(ctx, r::leftRightHdr(64, 32, C, D), o);
    if (!solid.environment || !upperLower.environment || !leftRight.environment) {
        check(false, "创建环境：" + solid.error.value_or("") + upperLower.error.value_or("") + leftRight.error.value_or(""));
        return;
    }
    std::printf("INFO 环境预计算（立方体 128、预过滤 64 × 5 级）：%.3f 秒\n", solid.environment->precomputeSeconds());
    const auto& lut = solid.environment->dfgLut();
    const int lutSize = solid.environment->dfgSize();

    const Vec3f eye{-4.0f, 0.0f, 0.0f};
    const View view(eye, {0.0f, 0.0f, 0.0f}, 128, 128);
    r::SunLight off;
    off.intensity = 0.0f;
    r::HemisphereLight dark;
    dark.intensity = 0.0f;
    r::RenderSettings rs = plainSettings();
    rs.shadows = false;
    const Vec3f n{-1.0f, 0.0f, 0.0f};
    const Vec3f v{-1.0f, 0.0f, 0.0f};

    r::Material plastic;
    plastic.baseColor = {0.8f, 0.3f, 0.2f};
    plastic.roughness = 0.5f;
    r::Material gold;
    gold.baseColor = {1.0f, 0.78f, 0.34f};
    gold.metallic = 1.0f;
    gold.roughness = 0.7f;
    r::Material matte;
    matte.baseColor = {0.8f, 0.8f, 0.8f};
    matte.roughness = 1.0f;
    r::Material chrome;
    chrome.baseColor = {0.95f, 0.95f, 0.95f};
    chrome.metallic = 1.0f;
    chrome.roughness = 0.0f;

    const auto renderSphere = [&](const r::Material& m, const r::Environment& env, float intensity, float rotation, bool background) {
        r::EnvironmentLighting lighting;
        lighting.environment = &env;
        lighting.intensity = intensity;
        lighting.rotationRadians = rotation;
        lighting.background = background;
        const std::vector<r::RenderItem> items{{&sphere, r::identityMatrix(), m, true}};
        renderer.render(view.width, view.height, view.params, items, off, dark, rs, lighting);
    };
    const auto compare = [&](const char* name, Rgb got, Vec3f radiance, int tol) {
        const Rgb want = expected(radiance, rs.toneMapper, rs.exposureEv);
        std::printf("INFO %s：像素 %s，参考 %s\n", name, str(got).c_str(), str(want).c_str());
        check(near(got, want, tol), std::string{name} + "：" + str(got) + " 与参考 " + str(want) + " 相差不超过 " + std::to_string(tol));
    };
    const auto scaled = [](Vec3f c, float k) { return Vec3f{c.x * k, c.y * k, c.z * k}; };

    // 常数环境：辐照度与预过滤都等于 L
    renderSphere(plastic, *solid.environment, 1.0f, 0.0f, false);
    check(renderer.lastFrame().imageLighting && !renderer.lastFrame().backgroundDrawn, "使用了环境光照，未要求背景时不画背景");
    compare("常数环境，非金属", pixel(renderer, 64, 64), pbr::ambientIbl(plastic, n, v, L, L, lut, lutSize), 3);
    renderSphere(gold, *solid.environment, 0.5f, 0.0f, false);
    compare("常数环境，金属（金），强度 0.5", pixel(renderer, 64, 64), scaled(pbr::ambientIbl(gold, n, v, L, L, lut, lutSize), 0.5f), 3);

    // 上下两色：水平法线的辐照度与粗糙反射都约为两色平均；镜面在朝上 45° 处反射天空
    const Vec3f average{0.5f * (A.x + B.x), 0.5f * (A.y + B.y), 0.5f * (A.z + B.z)};
    renderSphere(matte, *upperLower.environment, 1.0f, 0.0f, false);
    compare("上下两色，粗糙非金属（水平法线：两色平均）", pixel(renderer, 64, 64), pbr::ambientIbl(matte, n, v, average, average, lut, lutSize), 4);
    renderSphere(chrome, *upperLower.environment, 1.0f, 0.0f, false);
    {
        const Vec3f p{-0.70710678f, 0.0f, 0.70710678f};  // 法线朝相机上方 45°：反射方向接近 +Z
        int x = 0;
        int y = 0;
        if (view.project(p, x, y)) {
            const Vec3f pv = normalize({eye.x - p.x, eye.y - p.y, eye.z - p.z});
            const Vec3f mirror = reflectDir({-pv.x, -pv.y, -pv.z}, p);
            check(mirror.z > 0.9f, "镜面反射方向朝上");
            compare("上下两色，镜面金属朝上 45°（反射天空）", pixel(renderer, x, y), pbr::ambientIbl(chrome, p, pv, A, A, lut, lutSize), 4);
        } else {
            check(false, "朝上 45° 的球面点在视野内");
        }
    }

    // 左右两色 + 旋转：+Y 一侧的镜面点反射 +Y（红）；环境旋转 180° 后反射 −Y（蓝）
    {
        const Vec3f p{-0.70710678f, 0.70710678f, 0.0f};
        int x = 0;
        int y = 0;
        if (view.project(p, x, y)) {
            const Vec3f pv = normalize({eye.x - p.x, eye.y - p.y, eye.z - p.z});
            renderSphere(chrome, *leftRight.environment, 1.0f, 0.0f, false);
            compare("左右两色，镜面金属 +Y 一侧（反射 +Y：红）", pixel(renderer, x, y), pbr::ambientIbl(chrome, p, pv, C, C, lut, lutSize), 4);
            renderSphere(chrome, *leftRight.environment, 1.0f, 3.14159265f, false);
            compare("环境旋转 180°（反射 −Y：蓝）", pixel(renderer, x, y), pbr::ambientIbl(chrome, p, pv, D, D, lut, lutSize), 4);
            // 旋转 90°（逆时针）：世界方向按 −90° 转回贴图坐标，反射方向 (0.21, 0.98) → (0.98, −0.21)，落在 −Y 一侧（蓝）；
            // 若旋转方向写反会落在 +Y 一侧（红）
            renderSphere(chrome, *leftRight.environment, 1.0f, 0.5f * 3.14159265f, false);
            compare("环境旋转 +90°（反射落到 −Y 一侧：蓝）", pixel(renderer, x, y), pbr::ambientIbl(chrome, p, pv, D, D, lut, lutSize), 4);
        } else {
            check(false, "+Y 一侧的球面点在视野内");
        }
    }

    // 背景与物体同时存在：背景只填充空白处，不遮挡物体
    renderSphere(plastic, *solid.environment, 1.0f, 0.0f, true);
    check(renderer.lastFrame().backgroundDrawn, "绘制了背景");
    compare("背景开启时球心仍为物体着色", pixel(renderer, 64, 64), pbr::ambientIbl(plastic, n, v, L, L, lut, lutSize), 3);
    compare("背景开启时角落为环境色", pixel(renderer, 2, 2), L, 3);

    // 背景：仰视看到天空色，俯视看到地面色（避开立方体面的交界）；关闭背景时为清屏颜色
    for (const auto& [name, target, color] : {std::tuple<const char*, Vec3f, Vec3f>{"背景：仰角约 63° 为天空色", {1.0f, 0.0f, 2.0f}, A},
                                               std::tuple<const char*, Vec3f, Vec3f>{"背景：俯角约 63° 为地面色", {1.0f, 0.0f, -2.0f}, B}}) {
        const View look({0.0f, 0.0f, 0.0f}, target, 64, 64);
        r::EnvironmentLighting lighting;
        lighting.environment = upperLower.environment.get();
        lighting.background = true;
        renderer.render(look.width, look.height, look.params, {}, off, dark, rs, lighting);
        check(renderer.lastFrame().backgroundDrawn, std::string{name} + "：绘制了背景");
        compare(name, pixel(renderer, 32, 32), color, 3);
    }
    {
        const View look({0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 1.0f}, 64, 64);
        r::EnvironmentLighting lighting;
        lighting.environment = upperLower.environment.get();
        lighting.background = false;
        renderer.render(look.width, look.height, look.params, {}, off, dark, rs, lighting);
        check(near(pixel(renderer, 32, 32), Rgb{0, 0, 0}, 1), "关闭背景：为清屏颜色");
    }
}

// ---- M1-09：贴图材质（2 × 2 纯色贴图，结果可与 C++ 参考精确比较）----

r::Texture2D solidTexture(std::uint8_t red, std::uint8_t green, std::uint8_t blue, bool srgb) {
    const std::uint8_t px[16] = {red, green, blue, 255, red, green, blue, 255, red, green, blue, 255, red, green, blue, 255};
    return r::Texture2D::create(2, 2, px, srgb);
}

void testTextures(r::SceneRenderer& renderer) {
    const r::GpuMesh plane = r::GpuMesh::create(r::meshes::plane(3.0f, 3.0f));
    const Vec3f eye{-2.0f, 0.0f, 2.0f};
    const View view(eye, {0.0f, 0.0f, 0.0f}, 96, 96);  // 中心像素对应地面原点
    const Vec3f up{0.0f, 0.0f, 1.0f};
    const Vec3f v = normalize(eye);
    r::SunLight sun;
    sun.direction = normalize({0.0f, 0.5f, 1.0f});  // 含 y 分量：法线贴图的绿色通道方向会影响结果
    sun.intensity = 3.0f;
    r::SunLight off;
    off.intensity = 0.0f;
    r::HemisphereLight dark;
    dark.intensity = 0.0f;
    r::HemisphereLight env;
    env.intensity = 1.0f;
    r::RenderSettings rs = plainSettings();
    rs.shadows = false;
    const auto renderPlane = [&](const r::Material& m, const r::SunLight& light, const r::HemisphereLight& hemi, const View& at) {
        const std::vector<r::RenderItem> items{{&plane, r::identityMatrix(), m, false}};
        renderer.render(at.width, at.height, at.params, items, light, hemi, rs);
        return pixel(renderer, at.width / 2, at.height / 2);
    };
    const auto compare = [&](const char* name, Rgb got, Vec3f radiance) {
        const Rgb want = expected(radiance, rs.toneMapper, rs.exposureEv);
        std::printf("INFO %s：像素 %s，参考 %s\n", name, str(got).c_str(), str(want).c_str());
        check(near(got, want, 3), std::string{name} + "：" + str(got) + " 与参考 " + str(want) + " 相差不超过 3");
    };
    const auto byte = [](int b) { return static_cast<float>(b) / 255.0f; };

    // 基础色贴图（sRGB）× 因子
    const r::Texture2D base = solidTexture(200, 100, 50, true);
    r::Material textured;
    textured.baseColor = {0.9f, 1.0f, 0.8f};
    textured.roughness = 0.5f;
    textured.baseColorTexture = base.name();
    r::Material reference = textured;
    reference.baseColorTexture = 0;
    reference.baseColor = {0.9f * pbr::srgbDecode(byte(200)), pbr::srgbDecode(byte(100)), 0.8f * pbr::srgbDecode(byte(50))};
    compare("基础色贴图（sRGB 解码后乘以因子）", renderPlane(textured, sun, dark, view), pbr::shade(reference, up, v, sun, dark, 1.0f));

    // 金属度-粗糙度贴图（线性）：G → 粗糙度，B → 金属度
    const r::Texture2D mr = solidTexture(0, 128, 255, false);
    r::Material metal;
    metal.baseColor = {0.9f, 0.7f, 0.4f};
    metal.metallic = 1.0f;
    metal.roughness = 1.0f;
    metal.metallicRoughnessTexture = mr.name();
    r::Material metalRef = metal;
    metalRef.metallicRoughnessTexture = 0;
    metalRef.roughness = byte(128);
    metalRef.metallic = 1.0f;
    compare("金属度-粗糙度贴图（G 粗糙度、B 金属度）", renderPlane(metal, sun, dark, view), pbr::shade(metalRef, up, v, sun, dark, 1.0f));

    // 法线贴图（切线空间，OpenGL 约定）：平面的切线 = +X、副切线 = +Y
    const r::Texture2D normal = solidTexture(191, 64, 221, false);
    r::Material bumped;
    bumped.baseColor = {0.7f, 0.7f, 0.7f};
    bumped.roughness = 0.6f;
    bumped.normalTexture = normal.name();
    r::Material bumpedRef = bumped;
    bumpedRef.normalTexture = 0;
    const Vec3f nMap = normalize({byte(191) * 2.0f - 1.0f, byte(64) * 2.0f - 1.0f, byte(221) * 2.0f - 1.0f});
    compare("法线贴图（偏向 +X、−Y）", renderPlane(bumped, sun, dark, view), pbr::shade(bumpedRef, nMap, v, sun, dark, 1.0f));

    // 环境遮蔽贴图：只压暗环境光，不影响太阳直射光
    const r::Texture2D ao = solidTexture(128, 255, 255, false);
    r::Material occluded;
    occluded.baseColor = {0.8f, 0.8f, 0.8f};
    occluded.roughness = 0.7f;
    occluded.occlusionTexture = ao.name();
    r::Material occludedRef = occluded;
    occludedRef.occlusionTexture = 0;
    compare("遮蔽贴图 0.5：环境光变暗", renderPlane(occluded, off, env, view), pbr::shade(occludedRef, up, v, off, env, 1.0f, byte(128)));
    compare("遮蔽贴图不影响太阳直射光", renderPlane(occluded, sun, dark, view), pbr::shade(occludedRef, up, v, sun, dark, 1.0f, 1.0f));

    // 双面：从下方看平面（背面）；双面材质按翻转的法线着色，单面材质被剔除
    const Vec3f below{-2.0f, 0.0f, -2.0f};
    const View under(below, {0.0f, 0.0f, 0.0f}, 96, 96);
    r::Material twoSided;
    twoSided.baseColor = {0.6f, 0.8f, 0.6f};
    twoSided.roughness = 0.8f;
    twoSided.doubleSided = true;
    compare("双面材质的背面（法线取反）", renderPlane(twoSided, off, env, under),
            pbr::shade(twoSided, {0.0f, 0.0f, -1.0f}, normalize(below), off, env, 1.0f));
    twoSided.doubleSided = false;
    check(near(renderPlane(twoSided, off, env, under), Rgb{0, 0, 0}, 1), "单面材质的背面被剔除（背景）");
}

// GPU 计时（M1-10）：开启后第一帧等待即可得到有效的分阶段耗时；各阶段非负、总和为正。
void testTiming(r::SceneRenderer& renderer, const r::GpuMesh& sphere) {
    const View view({-4.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, 64, 64);
    r::SunLight sun;
    sun.direction = normalize({-1.0f, 0.0f, 1.0f});
    renderer.setTimingEnabled(true);
    const std::vector<r::RenderItem> items{{&sphere, r::identityMatrix(), r::Material{}, true}};
    renderer.render(view.width, view.height, view.params, items, sun, r::HemisphereLight{}, plainSettings());
    const r::PassTimings t = renderer.waitForTimings();
    bool nonNegative = true;
    for (const double ms : t.milliseconds) nonNegative = nonNegative && ms >= 0.0;
    std::printf("INFO GPU 计时：总计 %.3f ms（阴影 %.3f、物体 %.3f、色调映射 %.3f）\n", t.total(), t.milliseconds[0], t.milliseconds[1],
                t.milliseconds[5]);
    check(t.valid && nonNegative && t.total() > 0.0, "开启计时后，等待即可得到本帧有效的分阶段耗时");
    renderer.setTimingEnabled(false);
}

}  // namespace

int main(int argc, char** argv) {
    in::InputSystem input;
    auto platform = terra::platform::glfw::createPlatform(input);
    if (!platform.platform) {
        std::fprintf(stderr, "FAILED: 创建平台：%s\n", platform.error ? platform.error->message.c_str() : "");
        return 1;
    }
    terra::platform::WindowDescription desc;
    desc.title = "terra pbr test";
    desc.width = 64;
    desc.height = 64;
    desc.visible = false;
    desc.glDebugContext = true;
    auto window = platform.platform->createWindow(desc);
    if (!window.window) {
        std::fprintf(stderr, "FAILED: 创建窗口：%s\n", window.error ? window.error->message.c_str() : "");
        return 1;
    }
    auto created = r::RenderContext::create(window.window->glProcLoader());
    if (!created.context) {
        std::fprintf(stderr, "FAILED: 初始化渲染上下文：%s\n", created.error.value_or("").c_str());
        return 1;
    }
    r::RenderContext& ctx = *created.context;
    std::printf("INFO %s，OpenGL %s，最大采样数 %d\n", ctx.info().renderer.c_str(), ctx.info().version.c_str(), ctx.info().maxSamples);
    if (argc > 1) {
        check(ctx.info().renderer.find(argv[1]) != std::string::npos, std::string{"渲染器包含 '"} + argv[1] + "'（D-022）");
    }

    // 显存查询（M1-15）：llvmpipe 声明了 NVX 扩展，但报告的不是显存（实测总量 526344 MB，LOG-069）；
    // 软件渲染器应为空且不产生 GL 错误；其他驱动只记录
    {
        check(r::isSoftwareRenderer("llvmpipe (LLVM 15.0.7, 256 bits)") && r::isSoftwareRenderer("softpipe") &&
                  !r::isSoftwareRenderer("NVIDIA GeForce RTX 5060/PCIe/SSE2") && !r::isSoftwareRenderer("AMD Radeon RX 7600 (radeonsi)"),
              "软件渲染器识别：llvmpipe、softpipe 是，NVIDIA、AMD 不是");
        {
            // 驱动原始值 → 显存（纯函数，输入固定，不依赖 llvmpipe 每次返回什么）
            const std::string lp = "llvmpipe (LLVM 15.0.7, 256 bits)";
            const std::string nv = "NVIDIA GeForce RTX 5060/PCIe/SSE2";
            const std::string nvx = "GL_NVX_gpu_memory_info";
            check(!r::gpuMemoryFromReport(lp, nvx, 538976256LL, 538976256LL), "llvmpipe 报告约 514 GB（实测原始值）：为空");
            check(!r::gpuMemoryFromReport(lp, nvx, 0LL, 0LL), "llvmpipe 报告 0：为空");
            const auto real = r::gpuMemoryFromReport(nv, nvx, 8346624LL, 5222400LL);
            check(real && real->totalMb && *real->totalMb == 8151.0 && real->availableMb == 5100.0 && real->source == nvx,
                  "NVIDIA 报告 8151 / 5100 MB：原样换算");
            check(!r::gpuMemoryFromReport(nv, nvx, 0LL, 0LL), "硬件驱动报告总量 0：为空");
            check(!r::gpuMemoryFromReport(nv, nvx, 1024LL, 2048LL), "可用量大于总量：为空");
            const auto ati = r::gpuMemoryFromReport("AMD Radeon RX 7600 (radeonsi)", "GL_ATI_meminfo", std::nullopt, 4096LL);
            check(ati && !ati->totalMb && ati->availableMb == 4.0, "ATI 只报告可用量：总量为空、可用 4 MB");
        }
        const auto vram = ctx.gpuMemory();
        if (ctx.info().renderer.find("llvmpipe") != std::string::npos) {
            check(!vram.has_value(), "llvmpipe 没有专用显存：结果为空（界面显示“不可用”，不显示 0 MB）");
        }
        if (vram) {
            std::printf("INFO 显存（%s）：总量 %.0f MB，可用 %.0f MB\n", vram->source.c_str(), vram->totalMb.value_or(-1.0), vram->availableMb);
        }
        check(ctx.errorMessageCount() == 0, "显存查询不产生 GL 错误");
    }
    auto made = r::SceneRenderer::create(ctx);
    if (!made.renderer) {
        std::fprintf(stderr, "FAILED: 创建场景渲染器：%s\n", made.error.value_or("").c_str());
        return 1;
    }
    r::SceneRenderer& renderer = *made.renderer;
    {
        const r::GpuMesh sphere = r::GpuMesh::create(r::meshes::sphere(1.0f));
        const r::GpuMesh plane = r::GpuMesh::create(r::meshes::plane(10.0f, 10.0f));
        const r::GpuMesh box = r::GpuMesh::create(r::meshes::box(0.5f, 0.5f, 0.5f));
        testSphere(renderer, sphere);
        testShadow(renderer, plane, box);
        testMsaa(renderer, sphere, ctx.info().maxSamples);
        testGrid(renderer);
        testIbl(ctx, renderer, sphere);
        testTextures(renderer);
        testTiming(renderer, sphere);
        renderer.render(40, 30, r::ViewParams{}, {}, r::SunLight{}, r::HemisphereLight{}, r::RenderSettings{});
        check(renderer.width() == 40 && renderer.height() == 30 && renderer.outputTexture() != 0, "尺寸变化后重建渲染目标");
    }
    for (const auto& m : ctx.takeDebugMessages()) std::printf("INFO GL 消息：%s\n", m.text.c_str());
    check(ctx.errorMessageCount() == 0, "全程没有 GL 错误（调试上下文）");
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
