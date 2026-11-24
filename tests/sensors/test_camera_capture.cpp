// 相机传感器的渲染（M4-03）：camera_bench 夹具中按快照渲染彩色与深度，与解析解比较。
//   - 深度：墙的正面在 x = 2.0，凡是打到墙的像素深度为 2.0 m（沿光轴的距离，容差 1e-5）；视野里没有空白（没有 +inf）；
//   - 投影：方块正面（深度 0.95 m）像素的质心与按内参算出的投影一致（±0.6 像素）；
//   - 远平面：far = 1.5 时，墙在远平面之外，那些像素为 +inf，方块仍在；
//   - 彩色：方块正面中心的像素以红色为主，墙为灰色；
//   - 时间与位姿：帧的步序号、时间与给定快照相同（推进 10 步后），位姿与 cameraWorldPose 逐位相同；
//   - 两种写法（MJCF 相机 eye、装在实体上的 side）的深度图一致（1e-5）；side 只输出深度，彩色为空；
//   - 确定性：同一快照渲染两次，彩色逐字节、深度逐位相同；
//   - 离屏目标只在第一次渲染深度时创建；
//   - 参考机器人（有时）：Stretch 3 的头部相机 d435i_camera_rgb，深度有有限值、彩色不是单一颜色。
// 输出目录中写出 PNG（彩色 | 深度伪彩色），供查看。
//
// 用法：test_camera_capture <输出目录> [--expect-renderer 子串]

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include "camera_sensor.hpp"
#include "demo_scene.hpp"
#include "scene_view.hpp"
#include "terra/input/input_system.hpp"
#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/platform/glfw/glfw_platform.hpp"
#include "terra/render/context.hpp"
#include "terra/render/image.hpp"
#include "terra/scene/camera_model.hpp"
#include "terra/scene/scene.hpp"

#ifndef TERRA_ASSETS_DIR
#error "需要 TERRA_ASSETS_DIR"
#endif

namespace {

namespace fs = std::filesystem;
namespace r = terra::render;
namespace sc = terra::scene;
namespace td = terra::data;
namespace vw = terra::viewer;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

// 彩色与深度并排写成一张 PNG
void saveMontage(const vw::CameraFrame& frame, float maxDepth, const fs::path& path) {
    const int w = frame.intrinsics.width;
    const int h = frame.intrinsics.height;
    r::ImageRgba8 out;
    out.width = 2 * w;
    out.height = h;
    out.pixels.assign(static_cast<std::size_t>(out.width) * static_cast<std::size_t>(h) * 4, 0);
    const r::ImageRgba8 color = vw::colorImage(frame);
    const r::ImageRgba8 depth = vw::depthPreview(frame.depth, w, h, 0.0f, maxDepth);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const auto src = (static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 4;
            const auto left = (static_cast<std::size_t>(y) * static_cast<std::size_t>(out.width) + static_cast<std::size_t>(x)) * 4;
            const auto right = left + static_cast<std::size_t>(w) * 4;
            for (std::size_t c = 0; c < 4; ++c) {
                out.pixels[left + c] = frame.rgb.empty() ? std::uint8_t{0} : color.pixels[src + c];
                out.pixels[right + c] = frame.depth.empty() ? std::uint8_t{0} : depth.pixels[src + c];
            }
        }
    }
    if (const auto error = r::savePng(path.string(), out)) {
        check(false, "写 PNG：" + *error);
    } else {
        std::printf("INFO 已写出 %s\n", path.string().c_str());
    }
}

void bench(r::RenderContext& ctx, vw::CameraRenderer& renderer, vw::EnvironmentLibrary& environments, const fs::path& outDir) {
    auto loaded = sc::loadScene(std::string{TERRA_ASSETS_DIR} + "/tests/sensors/camera_bench.terra-scene.json",
                                terra::physics::mujoco::composeWorld);
    check(loaded.ok() && loaded.scene->cameras.size() == 2, "加载相机夹具");
    if (!loaded.ok() || loaded.scene->cameras.size() != 2) return;
    auto& scene = *loaded.scene;
    for (int i = 0; i < 10; ++i) scene.world->step();
    std::vector<vw::SceneMessage> messages;
    const auto view = vw::SceneView::create(scene, messages);
    terra::settings::RenderSettings rs;
    rs.environment = terra::settings::EnvironmentSetting::Hemisphere;
    const vw::FrameLighting lit = vw::frameLighting(rs, environments, ctx, messages);
    const auto snap = scene.world->snapshot();
    const sc::CameraSensor& eye = scene.cameras[0];

    check(renderer.depthTargets() == 0, "第一次渲染之前没有离屏目标");
    const vw::CameraFrame frame = renderer.capture(scene, *view, eye, *snap, lit);
    check(renderer.depthTargets() == 1, "渲染深度时创建了离屏目标");
    const vw::CameraFrame again = renderer.capture(scene, *view, eye, *snap, lit);
    const std::size_t w = 160;
    const std::size_t h = 120;
    check(frame.rgb.size() == w * h * 3 && frame.depth.size() == w * h, "彩色 160 × 120 × 3，深度 160 × 120");
    if (frame.depth.size() != w * h || frame.rgb.size() != w * h * 3) return;

    // 时间与位姿
    const td::Pose expected = sc::cameraWorldPose(*snap, eye);
    check(frame.step == 10 && frame.time == snap->simulationTime && frame.runEpoch == snap->runEpoch, "帧的步序号（10）、时间与快照相同");
    check(frame.pose.position.x == expected.position.x && frame.pose.position.y == expected.position.y &&
              frame.pose.position.z == expected.position.z && frame.pose.orientation.w == expected.orientation.w &&
              frame.pose.orientation.x == expected.orientation.x && frame.pose.orientation.y == expected.orientation.y &&
              frame.pose.orientation.z == expected.orientation.z,
          "帧的位姿与 cameraWorldPose 逐位相同");

    // 深度：墙 2.0 m；方块正面 0.95 m；没有空白
    std::size_t wall = 0;
    std::size_t faceCount = 0;
    std::size_t infinite = 0;
    double maxWallError = 0.0;
    double sumU = 0.0;
    double sumV = 0.0;
    for (std::size_t y = 0; y < h; ++y) {
        for (std::size_t x = 0; x < w; ++x) {
            const float d = frame.depth[y * w + x];
            if (!std::isfinite(d)) {
                ++infinite;
            } else if (d > 1.5f) {
                ++wall;
                maxWallError = std::max(maxWallError, std::fabs(static_cast<double>(d) - 2.0));
            } else if (std::fabs(static_cast<double>(d) - 0.95) < 1e-4) {
                ++faceCount;
                sumU += static_cast<double>(x);
                sumV += static_cast<double>(y);
            }
        }
    }
    std::printf("INFO 墙 %zu 像素，最大深度误差 %.3g m；方块正面 %zu 像素；空白 %zu 像素\n", wall, maxWallError, faceCount, infinite);
    check(infinite == 0, "视野里没有空白（墙充满视野）");
    check(wall > w * h * 9 / 10 && maxWallError < 1e-5, "打到墙的像素深度为 2.0 m（误差 < 1e-5）");
    const auto k = frame.intrinsics;
    const auto projected = sc::projectPoint(k, frame.pose, {0.95, 0.3, 1.2});
    check(faceCount > 50 && projected.has_value(), "方块正面可见（深度 0.95 m）");
    if (faceCount > 0 && projected) {
        const double u = sumU / static_cast<double>(faceCount);
        const double v = sumV / static_cast<double>(faceCount);
        std::printf("INFO 方块正面质心 (%.3f, %.3f)，按内参的投影 (%.3f, %.3f)\n", u, v, (*projected)[0], (*projected)[1]);
        check(std::fabs(u - (*projected)[0]) < 0.6 && std::fabs(v - (*projected)[1]) < 0.6, "方块正面的质心与内参算出的投影一致（±0.6 像素）");
        const auto cu = static_cast<std::size_t>(std::lround((*projected)[0]));
        const auto cv = static_cast<std::size_t>(std::lround((*projected)[1]));
        const std::uint8_t* px = frame.rgb.data() + (cv * w + cu) * 3;
        check(px[0] > px[1] + 40 && px[0] > px[2] + 40, "方块中心的像素以红色为主（" + std::to_string(px[0]) + ", " + std::to_string(px[1]) + ", " +
                                                           std::to_string(px[2]) + "）");
        const std::uint8_t* wallPx = frame.rgb.data() + (5 * w + 150) * 3;
        check(std::abs(int{wallPx[0]} - int{wallPx[1]}) < 12 && std::abs(int{wallPx[1]} - int{wallPx[2]}) < 12, "墙为灰色");
    }
    check(again.rgb == frame.rgb && again.depth == frame.depth, "同一快照渲染两次：彩色逐字节、深度逐位相同");

    // 远平面之外为 +inf
    sc::CameraSensor shortRange = eye;
    shortRange.farClip = 1.5;
    const vw::CameraFrame clipped = renderer.capture(scene, *view, shortRange, *snap, lit);
    std::size_t clippedInf = 0;
    std::size_t clippedFace = 0;
    for (const float d : clipped.depth) {
        clippedInf += std::isfinite(d) ? 0u : 1u;
        clippedFace += std::isfinite(d) && std::fabs(static_cast<double>(d) - 0.95) < 1e-4 ? 1u : 0u;
    }
    check(clippedInf + clippedFace >= w * h * 9 / 10 && clippedFace == faceCount, "far = 1.5：墙在远平面之外为 +inf，方块不变");

    // 装在实体上的相机：只输出深度，与 eye 一致。两种写法的姿态只差舍入，方块边缘个别像素可能落到墙上，允许至多 4 个这样的像素
    const vw::CameraFrame side = renderer.capture(scene, *view, scene.cameras[1], *snap, lit);
    std::size_t differing = 0;
    for (std::size_t i = 0; i < frame.depth.size() && side.depth.size() == frame.depth.size(); ++i) {
        const double a = static_cast<double>(frame.depth[i]);
        const double b = static_cast<double>(side.depth[i]);
        differing += (std::isfinite(a) != std::isfinite(b) || (std::isfinite(a) && std::fabs(a - b) > 1e-5)) ? 1u : 0u;
    }
    check(side.rgb.empty() && side.depth.size() == frame.depth.size() && differing <= 4,
          "装在实体上的 side：只输出深度，与 eye 的深度一致（相差超过 1e-5 m 的像素 " + std::to_string(differing) + " 个）");
    check(ctx.errorMessageCount() == 0, "没有 GL 错误");
    saveMontage(frame, 2.5f, outDir / "camera_bench.png");
}

void stretch(r::RenderContext& ctx, vw::CameraRenderer& renderer, vw::EnvironmentLibrary& environments, const fs::path& outDir) {
    if (!fs::exists(std::string{TERRA_ASSETS_DIR} + "/robots/hello_robot_stretch_3/stretch.xml")) {
        std::printf("参考机器人未准备，跳过 Stretch 3 的头部相机\n");
        return;
    }
    auto parsed = sc::readSceneFile(std::string{TERRA_ASSETS_DIR} + "/scenes/robots/stretch3.terra-scene.json");
    check(parsed.document.has_value(), "读取 Stretch 3 场景");
    if (!parsed.document) return;
    sc::SensorSpec head;
    head.type = "camera";
    head.camera = "stretch3/d435i_camera_rgb";
    head.name = head.camera;
    head.where = "/sensors/0";
    parsed.document->sensors.push_back(head);
    auto loaded = sc::instantiate(std::move(*parsed.document), terra::physics::mujoco::composeWorld);
    check(loaded.ok() && loaded.scene->cameras.size() == 1, "Stretch 3 场景启用头部相机");
    if (!loaded.ok() || loaded.scene->cameras.size() != 1) return;
    auto& scene = *loaded.scene;
    std::vector<vw::SceneMessage> messages;
    const auto view = vw::SceneView::create(scene, messages);
    terra::settings::RenderSettings rs;
    rs.environment = terra::settings::EnvironmentSetting::Hemisphere;
    const vw::FrameLighting lit = vw::frameLighting(rs, environments, ctx, messages);
    const auto snap = scene.world->snapshot();
    const vw::CameraFrame frame = renderer.capture(scene, *view, scene.cameras[0], *snap, lit);
    std::size_t finite = 0;
    float nearest = std::numeric_limits<float>::infinity();
    for (const float d : frame.depth) {
        if (std::isfinite(d)) {
            ++finite;
            nearest = std::min(nearest, d);
        }
    }
    bool varied = false;
    for (std::size_t i = 3; i < frame.rgb.size() && !varied; ++i) varied = frame.rgb[i] != frame.rgb[i % 3];
    std::printf("INFO Stretch 3 头部相机 %d × %d，视场角 %.1f°：%zu 个像素有深度，最近 %.3f m\n", frame.intrinsics.width,
                frame.intrinsics.height, scene.cameras[0].fovyDegrees, finite, static_cast<double>(nearest));
    check(finite > 0 && varied, "Stretch 3 头部相机：深度有有限值，彩色不是单一颜色");
    saveMontage(frame, 4.0f, outDir / "stretch3_d435i.png");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "用法：test_camera_capture <输出目录> [--expect-renderer 子串]\n");
        return 2;
    }
    const fs::path outDir = argv[1];
    std::string expectRenderer;
    for (int i = 2; i + 1 < argc; ++i) {
        if (std::string{argv[i]} == "--expect-renderer") expectRenderer = argv[i + 1];
    }
    std::error_code ec;
    fs::create_directories(outDir, ec);

    terra::input::InputSystem input;
    auto platform = terra::platform::glfw::createPlatform(input);
    if (!platform.platform) {
        std::fprintf(stderr, "FAILED: 创建平台：%s\n", platform.error ? platform.error->message.c_str() : "");
        return 1;
    }
    terra::platform::WindowDescription desc;
    desc.title = "terra camera test";
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
    std::printf("INFO %s，OpenGL %s\n", ctx.info().renderer.c_str(), ctx.info().version.c_str());
    if (!expectRenderer.empty()) check(ctx.info().renderer.find(expectRenderer) != std::string::npos, "渲染器包含 " + expectRenderer);
    {
        std::string error;
        auto renderer = vw::CameraRenderer::create(ctx, error);
        check(renderer != nullptr, "创建相机渲染器：" + error);
        vw::EnvironmentLibrary environments(TERRA_ASSETS_DIR);
        if (renderer) {
            bench(ctx, *renderer, environments, outDir);
            stretch(ctx, *renderer, environments, outDir);
        }
    }
    check(ctx.errorMessageCount() == 0, "整个运行期间没有 GL 错误");
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
