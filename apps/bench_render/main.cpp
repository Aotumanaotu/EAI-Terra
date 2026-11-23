// terra_render_bench：渲染帧耗时基线（M1-10，F-M1-03、NFR-06）。
//
// 离屏渲染查看器的演示场景（与 terra_viewer、参考图测试共用 terra_viewer_scene），不经过窗口交换与垂直同步。
// 预热后逐帧记录：提交耗时（CPU 调用 render 的时间）、整帧耗时（含等待 GPU 完成）、GPU 各阶段耗时（OpenGL 时间戳）。
// 输出：--output 指定的 JSON Lines（config、每帧 frame、summary），以及标准输出的一行汇总（以 "summary " 开头）。
//
// 用法：terra_render_bench [--width W] [--height H] [--frames N] [--warmup N] [--scene procedural|studio|outdoor]
//                          [--msaa 1|2|4|8] [--no-shadows] [--output 文件] [--expect-renderer 子串]

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "demo_scene.hpp"
#include "terra/input/input_system.hpp"
#include "terra/platform/glfw/glfw_platform.hpp"
#include "terra/render/context.hpp"
#include "terra/render/scene_renderer.hpp"
#include "terra/view/camera.hpp"

namespace {

namespace r = terra::render;
namespace st = terra::settings;
namespace vw = terra::viewer;
using Clock = std::chrono::steady_clock;

struct Options {
    int width = 1920;
    int height = 1080;
    int frames = 600;
    int warmup = 60;
    std::string scene = "outdoor";
    int msaa = 4;
    bool shadows = true;
    std::string output;
    std::string expectRenderer;
};

bool parse(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const bool hasValue = i + 1 < argc;
        if (a == "--width" && hasValue) o.width = std::atoi(argv[++i]);
        else if (a == "--height" && hasValue) o.height = std::atoi(argv[++i]);
        else if (a == "--frames" && hasValue) o.frames = std::atoi(argv[++i]);
        else if (a == "--warmup" && hasValue) o.warmup = std::atoi(argv[++i]);
        else if (a == "--scene" && hasValue) o.scene = argv[++i];
        else if (a == "--msaa" && hasValue) o.msaa = std::atoi(argv[++i]);
        else if (a == "--no-shadows") o.shadows = false;
        else if (a == "--output" && hasValue) o.output = argv[++i];
        else if (a == "--expect-renderer" && hasValue) o.expectRenderer = argv[++i];
        else return false;
    }
    return o.width > 0 && o.height > 0 && o.frames > 0 && o.warmup >= 0 &&
           (o.scene == "procedural" || o.scene == "studio" || o.scene == "outdoor") &&
           (o.msaa == 1 || o.msaa == 2 || o.msaa == 4 || o.msaa == 8);
}

struct Stats {
    double mean = 0, p50 = 0, p95 = 0, p99 = 0, max = 0;
};

Stats stats(std::vector<double> v) {
    Stats s;
    if (v.empty()) return s;
    std::sort(v.begin(), v.end());
    double sum = 0;
    for (const double x : v) sum += x;
    const auto at = [&](double q) {
        const auto i = static_cast<std::size_t>(std::ceil(q * static_cast<double>(v.size()))) - 1;
        return v[std::min(i, v.size() - 1)];
    };
    s.mean = sum / static_cast<double>(v.size());
    s.p50 = at(0.50);
    s.p95 = at(0.95);
    s.p99 = at(0.99);
    s.max = v.back();
    return s;
}

std::string json(const Stats& s) {
    char buf[200];
    std::snprintf(buf, sizeof(buf), "{\"mean\":%.4f,\"p50\":%.4f,\"p95\":%.4f,\"p99\":%.4f,\"max\":%.4f}", s.mean, s.p50, s.p95, s.p99, s.max);
    return buf;
}

std::string escape(const std::string& s) {
    std::string out;
    for (const char c : s) {
        if (c == '"' || c == '\\') out += '\\';
        out += c;
    }
    return out;
}

}  // namespace

int main(int argc, char** argv) {
    Options o;
    if (!parse(argc, argv, o)) {
        std::fprintf(stderr,
                     "用法：terra_render_bench [--width W] [--height H] [--frames N] [--warmup N] [--scene procedural|studio|outdoor] "
                     "[--msaa 1|2|4|8] [--no-shadows] [--output 文件] [--expect-renderer 子串]\n");
        return 2;
    }
    terra::input::InputSystem input;
    auto platform = terra::platform::glfw::createPlatform(input);
    if (!platform.platform) {
        std::fprintf(stderr, "无法创建平台：%s\n", platform.error ? platform.error->message.c_str() : "");
        return 1;
    }
    terra::platform::WindowDescription desc;
    desc.title = "terra render bench";
    desc.width = 64;
    desc.height = 64;
    desc.visible = false;
    desc.vsync = false;
    auto window = platform.platform->createWindow(desc);
    if (!window.window) {
        std::fprintf(stderr, "无法创建窗口：%s\n", window.error ? window.error->message.c_str() : "");
        return 1;
    }
    auto created = r::RenderContext::create(window.window->glProcLoader());
    if (!created.context) {
        std::fprintf(stderr, "无法初始化渲染：%s\n", created.error.value_or("").c_str());
        return 1;
    }
    r::RenderContext& ctx = *created.context;
    if (!o.expectRenderer.empty() && ctx.info().renderer.find(o.expectRenderer) == std::string::npos) {
        std::fprintf(stderr, "渲染器 %s 不含 %s\n", ctx.info().renderer.c_str(), o.expectRenderer.c_str());
        return 1;
    }
    auto made = r::SceneRenderer::create(ctx);
    if (!made.renderer) {
        std::fprintf(stderr, "无法创建场景渲染器：%s\n", made.error.value_or("").c_str());
        return 1;
    }
    r::SceneRenderer& renderer = *made.renderer;
    renderer.setTimingEnabled(true);

    int result = 0;
    {
        vw::EnvironmentLibrary environments(TERRA_ASSETS_DIR);
        vw::DemoScene scene;
        scene.create();
        std::vector<vw::SceneMessage> messages;
        st::RenderSettings rs;
        rs.msaaSamples = o.msaa;
        rs.shadows = o.shadows;
        r::Vec3f eye{-5.0f, 0.0f, 2.0f};
        r::Vec3f target{0.0f, 0.0f, 0.0f};
        if (o.scene != "procedural") {
            scene.loadAssets(TERRA_ASSETS_DIR, messages);
            rs.environment = o.scene == "studio" ? st::EnvironmentSetting::Studio : st::EnvironmentSetting::Outdoor;
            eye = {-4.2f, 0.0f, 0.95f};
            target = {-2.55f, 0.0f, 0.25f};
            if (scene.assets().size() != 3 || !environments.available(rs.environment)) {
                std::fprintf(stderr, "场景 %s 需要 HDRI 与 glTF 资产（scripts/assets/fetch_hdri.sh、fetch_models.sh）\n", o.scene.c_str());
                return 3;
            }
        } else {
            rs.environment = st::EnvironmentSetting::Hemisphere;
        }
        const vw::FrameLighting lit = vw::frameLighting(rs, environments, ctx, messages);
        terra::view::Camera camera;
        camera.position = {eye.x, eye.y, eye.z};
        camera.lookAt({target.x, target.y, target.z});
        r::ViewParams view;
        view.view = camera.viewMatrix();
        view.projection = camera.projectionMatrix(static_cast<double>(o.width) / static_cast<double>(o.height));
        view.cameraPosition = eye;
        const auto& items = scene.items(rs.assetArmOcclusion);

        std::FILE* out = o.output.empty() ? nullptr : std::fopen(o.output.c_str(), "w");
        if (!o.output.empty() && out == nullptr) {
            std::fprintf(stderr, "无法写入 %s\n", o.output.c_str());
            return 1;
        }
        char line[512];
        std::snprintf(line, sizeof(line),
                      "{\"type\":\"config\",\"renderer\":\"%s\",\"gl\":\"%s\",\"width\":%d,\"height\":%d,\"scene\":\"%s\",\"msaa\":%d,"
                      "\"shadows\":%s,\"frames\":%d,\"warmup\":%d,\"items\":%zu}",
                      escape(ctx.info().renderer).c_str(), escape(ctx.info().version).c_str(), o.width, o.height, o.scene.c_str(), o.msaa,
                      o.shadows ? "true" : "false", o.frames, o.warmup, items.size());
        if (out != nullptr) std::fprintf(out, "%s\n", line);

        std::vector<double> wall;
        std::vector<double> submit;
        std::vector<double> gpu;
        std::vector<std::vector<double>> passes(r::kRenderPassCount);
        for (int i = 0; i < o.warmup + o.frames; ++i) {
            const auto t0 = Clock::now();
            renderer.render(o.width, o.height, view, items, lit.sun, lit.hemisphere, lit.settings, lit.image);
            const auto t1 = Clock::now();
            const r::PassTimings timings = renderer.waitForTimings();  // 等待 GPU 完成本帧
            const auto t2 = Clock::now();
            if (i < o.warmup) continue;
            const double wallMs = std::chrono::duration<double, std::milli>(t2 - t0).count();
            const double submitMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
            wall.push_back(wallMs);
            submit.push_back(submitMs);
            gpu.push_back(timings.total());
            std::string passJson;
            for (int k = 0; k < r::kRenderPassCount; ++k) {
                const double ms = timings.milliseconds[static_cast<std::size_t>(k)];
                passes[static_cast<std::size_t>(k)].push_back(ms);
                char p[64];
                std::snprintf(p, sizeof(p), "%s\"%s\":%.4f", k ? "," : "", r::renderPassName(static_cast<r::RenderPass>(k)), ms);
                passJson += p;
            }
            if (out != nullptr) {
                std::fprintf(out, "{\"type\":\"frame\",\"index\":%d,\"wall_ms\":%.4f,\"submit_ms\":%.4f,\"gpu_ms\":%.4f,\"passes\":{%s}}\n",
                             i - o.warmup, wallMs, submitMs, timings.total(), passJson.c_str());
            }
            if (!timings.valid) result = 1;
        }
        std::string passSummary;
        for (int k = 0; k < r::kRenderPassCount; ++k) {
            passSummary += std::string{k ? "," : ""} + "\"" + r::renderPassName(static_cast<r::RenderPass>(k)) + "\":" +
                           json(stats(passes[static_cast<std::size_t>(k)]));
        }
        const Stats w = stats(wall);
        const Stats g = stats(gpu);
        const std::string summary = "{\"type\":\"summary\",\"renderer\":\"" + escape(ctx.info().renderer) + "\",\"width\":" +
                                    std::to_string(o.width) + ",\"height\":" + std::to_string(o.height) + ",\"scene\":\"" + o.scene +
                                    "\",\"msaa\":" + std::to_string(renderer.lastFrame().samples) + ",\"shadows\":" +
                                    (o.shadows ? "true" : "false") + ",\"frames\":" + std::to_string(o.frames) +
                                    ",\"wall_ms\":" + json(w) + ",\"submit_ms\":" + json(stats(submit)) + ",\"gpu_ms\":" + json(g) +
                                    ",\"passes\":{" + passSummary + "}}";
        if (out != nullptr) {
            std::fprintf(out, "%s\n", summary.c_str());
            std::fclose(out);
        }
        std::printf("summary %s\n", summary.c_str());
        std::printf("INFO %s %d×%d，%s，MSAA %d×，阴影%s：整帧中位数 %.2f ms（p95 %.2f），GPU 中位数 %.2f ms（p95 %.2f）\n",
                    ctx.info().renderer.c_str(), o.width, o.height, o.scene.c_str(), renderer.lastFrame().samples,
                    o.shadows ? "开" : "关", w.p50, w.p95, g.p50, g.p95);
    }
    if (ctx.errorMessageCount() != 0) {
        std::fprintf(stderr, "运行期间有 GL 错误\n");
        result = 1;
    }
    return result;
}
