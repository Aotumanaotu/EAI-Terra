// terra_capture（M4-04，M4-01 设计稿第 5、6 节）：离线采集传感器数据。按场景（与可选的动作）运行仿真，把每个传感器在
// 各自采样步上的读数写入目录。不开界面；场景中有相机时创建一个隐藏窗口作为图形上下文（需要 X 或 Xvfb），
// 没有相机或给了 --no-cameras 时不创建任何图形资源。同一输入、同一机器上两次运行的输出逐字节相同。
//
// 输出目录：
//   capture.json                       场景、动作、步长、步数、各传感器的清单与参数（相机含内参）
//   sensors.jsonl                      IMU 与 2D 激光雷达的读数（与 Headless --sensors 的 sensor / lidar 行相同）
//   cameras/<相机名>/frames.jsonl       每帧的步序号、时间、相机位姿（Terra 相机坐标系）与文件名
//   cameras/<相机名>/rgb/<步序号>.png   彩色（8 位，sRGB）
//   cameras/<相机名>/depth/<步序号>.npy 深度（float32，米，沿光轴；无命中为 +inf；NumPy 格式，行 0 为图像顶部）
//   cameras/<相机名>/seg/<步序号>.npy   分割图（M4-09，uint16 实体 ID，0 为背景；相机写了 "segmentation": true 时）
//   cameras/<相机名>/seg_labels.json    分割图的对照表（ID → 实体全名、模型实例、语义类别）
//   lidars3d/<雷达名>/frames.jsonl      M5-11：3D 激光雷达每帧的步序号、时间、帧序号、第一点时刻、雷达位姿、点数与摘要
//   lidars3d/<雷达名>/<步序号>.npy      M5-11：一帧的点（float32，形状 (点数, 5)：x、y、z（米，安装坐标系）、time（相对第一点，秒）、line）
// 传感器噪声（M4-09）按场景文件加在 IMU、雷达、彩色与深度上；分割图不加噪声。
// 相机名中的 “/” 换成 “_”。
//
// 用法：terra_capture (--scene <场景> | --motion <动作> [--scene <场景>]) (--steps N | --duration 秒) --output <目录>
//                     [--no-cameras] [--no-noise] [--force] [--expect-renderer 子串]
//   --no-noise  不加传感器噪声（M4-09）：输出真值；capture.json 中写明是否加了噪声
//   --force  输出目录中已有采集结果时先删除（只删除本程序写的 capture.json、sensors.jsonl 与 cameras/）
// 退出码：0 成功；2 参数错误；4 场景或动作错误；5 仿真错误；1 其他错误（写文件、图形上下文）。

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "camera_sensor.hpp"
#include "cli.hpp"
#include "demo_scene.hpp"
#include "scene_view.hpp"
#include "terra/core/build_info.hpp"
#include "terra/input/input_system.hpp"
#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/platform/glfw/glfw_platform.hpp"
#include "terra/render/context.hpp"
#include "terra/render/image.hpp"
#include "terra/scene/camera_model.hpp"
#include "terra/scene/scene.hpp"
#include "terra/simulation/lidar.hpp"
#include "terra/simulation/lidar3d.hpp"
#include "terra/simulation/motion.hpp"
#include "terra/simulation/pd_layer.hpp"
#include "terra/simulation/sensor_noise.hpp"
#include "terra/simulation/sensor_sampling.hpp"

namespace {

namespace fs = std::filesystem;
namespace hl = terra::headless;
namespace sc = terra::scene;
namespace sim = terra::simulation;
namespace td = terra::data;
namespace vw = terra::viewer;

struct Options {
    std::string scene;
    std::string motion;
    std::optional<std::uint64_t> steps;
    std::optional<double> duration;
    std::string output;
    bool noCameras = false;
    bool noNoise = false;  // M4-09
    bool force = false;
    std::string expectRenderer;
};

void diagnose(const std::string& text) { std::fprintf(stderr, "%s\n", text.c_str()); }

const char* kUsage =
    "用法：terra_capture (--scene <场景> | --motion <动作> [--scene <场景>]) (--steps N | --duration 秒) --output <目录>\n"
    "                    [--no-cameras] [--no-noise] [--force] [--expect-renderer 子串]\n";

std::optional<Options> parse(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const bool hasValue = i + 1 < argc;
        if (a == "--scene" && hasValue) {
            o.scene = argv[++i];
        } else if (a == "--motion" && hasValue) {
            o.motion = argv[++i];
        } else if (a == "--steps" && hasValue) {
            const std::string v = argv[++i];
            char* end = nullptr;
            const unsigned long long n = std::strtoull(v.c_str(), &end, 10);
            if (v.empty() || *end != '\0' || n == 0) {
                diagnose("--steps 需要正整数");
                return std::nullopt;
            }
            o.steps = n;
        } else if (a == "--duration" && hasValue) {
            const std::string v = argv[++i];
            char* end = nullptr;
            const double d = std::strtod(v.c_str(), &end);
            if (v.empty() || *end != '\0' || !(d > 0) || !std::isfinite(d)) {
                diagnose("--duration 需要正的秒数");
                return std::nullopt;
            }
            o.duration = d;
        } else if (a == "--output" && hasValue) {
            o.output = argv[++i];
        } else if (a == "--no-cameras") {
            o.noCameras = true;
        } else if (a == "--no-noise") {
            o.noNoise = true;
        } else if (a == "--force") {
            o.force = true;
        } else if (a == "--expect-renderer" && hasValue) {
            o.expectRenderer = argv[++i];
        } else {
            diagnose("未知参数或缺少值：" + a);
            return std::nullopt;
        }
    }
    if (o.scene.empty() && o.motion.empty()) diagnose("需要 --scene 或 --motion");
    else if (o.output.empty()) diagnose("需要 --output（输出目录）");
    else if (o.steps && o.duration) diagnose("--steps 与 --duration 只能二选一");
    else if (!o.steps && !o.duration && o.motion.empty()) diagnose("需要 --steps 或 --duration（给了 --motion 时可省略：运行到动作结束后 1 秒）");
    else return o;
    return std::nullopt;
}

std::string safeName(std::string name) {
    for (char& c : name) {
        if (c == '/' || c == '\\') c = '_';
    }
    return name;
}

std::string stepName(std::uint64_t step) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%06llu", static_cast<unsigned long long>(step));
    return buf;
}

// NumPy .npy（格式 1.0）：小端 float32，形状 (height, width)，行优先
bool writeNpy(const fs::path& path, const std::vector<float>& values, int width, int height) {
    std::string header = "{'descr': '<f4', 'fortran_order': False, 'shape': (" + std::to_string(height) + ", " + std::to_string(width) + "), }";
    const std::size_t prefix = 10;  // 魔数 6 + 版本 2 + 头长度 2
    while ((prefix + header.size() + 1) % 64 != 0) header += ' ';
    header += '\n';
    std::ofstream out(path, std::ios::binary);
    if (!out) return false;
    const unsigned char magic[8] = {0x93, 'N', 'U', 'M', 'P', 'Y', 1, 0};
    out.write(reinterpret_cast<const char*>(magic), 8);
    const auto length = static_cast<std::uint16_t>(header.size());
    const char lengthBytes[2] = {static_cast<char>(length & 0xFFu), static_cast<char>((length >> 8) & 0xFFu)};
    out.write(lengthBytes, 2);
    out.write(header.data(), static_cast<std::streamsize>(header.size()));
    // 本程序只在小端机器上运行（x86-64），float 的内存布局即 '<f4'
    out.write(reinterpret_cast<const char*>(values.data()), static_cast<std::streamsize>(values.size() * sizeof(float)));
    return static_cast<bool>(out);
}

// M5-11：3D 雷达一帧的点，形状 (点数, 5)
bool writePointsNpy(const fs::path& path, const sim::Lidar3dScan& scan) {
    std::vector<float> values;
    values.reserve(scan.points.size() * 5);
    for (const auto& p : scan.points) {
        values.insert(values.end(), {p.x, p.y, p.z, p.time, static_cast<float>(p.line)});
    }
    return writeNpy(path, values, 5, static_cast<int>(scan.points.size()));
}

std::string jsonString(const std::string& text) { return "\"" + hl::jsonEscape(text) + "\""; }

std::string vec3(const td::Vec3& v) {
    return "[" + hl::formatNumber(v.x) + "," + hl::formatNumber(v.y) + "," + hl::formatNumber(v.z) + "]";
}

// 隐藏窗口与渲染上下文（只在需要渲染相机时创建）
struct Graphics {
    terra::input::InputSystem input;
    terra::platform::PlatformResult platform;
    std::unique_ptr<terra::platform::Window> window;
    std::unique_ptr<terra::render::RenderContext> context;
};

int run(const Options& options) {
    // 动作（可选）：没给 --scene 时用动作文件中的推荐场景
    std::optional<sim::MotionDocument> motionDocument;
    std::string scenePath = options.scene;
    if (!options.motion.empty()) {
        const auto parsed = sim::readMotionFile(options.motion);
        for (const auto& issue : parsed.issues) diagnose("动作：" + issue.where + " " + issue.message);
        if (!parsed.document) return 4;
        motionDocument = *parsed.document;
        if (scenePath.empty()) scenePath = motionDocument->scene;
        if (scenePath.empty()) {
            diagnose("动作文件没有推荐场景，请用 --scene 指定");
            return 2;
        }
    }
    auto loaded = sc::loadScene(scenePath, terra::physics::mujoco::composeWorld);
    for (const auto& issue : loaded.issues) {
        diagnose(std::string{issue.severity == sc::SceneIssue::Severity::Error ? "场景错误 " : "场景警告 "} + issue.where + "：" + issue.message);
    }
    if (!loaded.ok()) return 4;
    sc::SceneInstance& scene = *loaded.scene;
    sim::SensorNoise noise(scene, !options.noNoise);  // M4-09：传感器噪声（各出口读数逐位相同）
    if (noise.active()) std::printf("INFO 传感器噪声：开启（种子 %llu；--no-noise 可关闭）\n", static_cast<unsigned long long>(noise.seed()));
    auto& world = *scene.world;
    const auto& d = world.description();
    sim::PdLayer pd(d, scene.pd);
    std::optional<sim::MotionPlayer> player;
    if (motionDocument) {
        const auto bound = sim::bindMotion(*motionDocument, d, pd.targetRangeFunction());
        for (const auto& issue : bound.issues) diagnose("动作：" + issue.where + " " + issue.message);
        if (!bound.motion) return 4;
        if (const auto key = bound.motion->startKeyframe) world.resetToKeyframe(*key);
        player.emplace(*bound.motion);
        player->start(world.stepIndex());
    }
    const double dt = d.timestep;
    std::uint64_t total = 0;
    if (options.steps) {
        total = *options.steps;
    } else {
        const double duration = options.duration ? *options.duration : player->motion().duration + 1.0;
        const auto steps = hl::stepsForDuration(duration, dt);
        if (!steps) {
            diagnose("时长换算为步数时超出范围");
            return 2;
        }
        total = *steps;
    }

    // 输出目录：不覆盖已有的采集结果
    std::error_code ec;
    const fs::path out = options.output;
    if (fs::exists(out / "capture.json", ec)) {
        if (!options.force) {
            diagnose("输出目录中已有采集结果：" + (out / "capture.json").string() + "；请换一个目录，或加 --force 覆盖");
            return 2;
        }
        fs::remove(out / "capture.json", ec);
        fs::remove(out / "sensors.jsonl", ec);
        fs::remove_all(out / "cameras", ec);
        fs::remove_all(out / "lidars3d", ec);  // M5-11
    }
    fs::create_directories(out, ec);
    if (ec) {
        diagnose("无法创建输出目录：" + out.string());
        return 1;
    }

    // 相机：需要图形上下文
    const bool cameras = !scene.cameras.empty() && !options.noCameras;
    Graphics g;
    std::unique_ptr<vw::SceneView> view;
    std::unique_ptr<vw::CameraRenderer> renderer;
    std::unique_ptr<vw::EnvironmentLibrary> environments;
    vw::FrameLighting lighting;
    std::string rendererName;
    if (cameras) {
        g.platform = terra::platform::glfw::createPlatform(g.input);
        if (!g.platform.platform) {
            diagnose("无法创建图形平台：" + (g.platform.error ? g.platform.error->message : std::string{}));
            return 1;
        }
        terra::platform::WindowDescription desc;
        desc.title = "terra_capture";
        desc.width = 64;
        desc.height = 64;
        desc.visible = false;
        auto window = g.platform.platform->createWindow(desc);
        if (!window.window) {
            diagnose("无法创建隐藏窗口（需要 X 或 Xvfb）：" + (window.error ? window.error->message : std::string{}));
            return 1;
        }
        g.window = std::move(window.window);
        auto created = terra::render::RenderContext::create(g.window->glProcLoader());
        if (!created.context) {
            diagnose("无法初始化渲染上下文：" + created.error.value_or(""));
            return 1;
        }
        g.context = std::move(created.context);
        rendererName = g.context->info().renderer;
        if (!options.expectRenderer.empty() && rendererName.find(options.expectRenderer) == std::string::npos) {
            diagnose("渲染器 “" + rendererName + "” 不包含 “" + options.expectRenderer + "”");
            return 1;
        }
        std::string error;
        renderer = vw::CameraRenderer::create(*g.context, error);
        if (!renderer) {
            diagnose("无法创建相机渲染器：" + error);
            return 1;
        }
        std::vector<vw::SceneMessage> messages;
        view = vw::SceneView::create(scene, messages);
        environments = std::make_unique<vw::EnvironmentLibrary>(TERRA_ASSETS_DIR);
        terra::settings::RenderSettings rs;  // 默认画质与半球光（与查看器的默认设置相同）
        lighting = vw::frameLighting(rs, *environments, *g.context, messages);
        for (const auto& m : messages) diagnose("场景提示：" + m.text);
        std::printf("INFO 渲染器 %s；%zu 个相机\n", rendererName.c_str(), scene.cameras.size());
    } else {
        std::printf("INFO 不创建图形上下文（%s）\n", scene.cameras.empty() ? "场景中没有相机" : "--no-cameras");
    }

    // capture.json
    {
        std::string json = "{\"format\":\"terra.capture\",\"version\":1,\"terra_version\":" + jsonString(std::string{terra::buildInfo().version}) +
                           ",\"scene\":" + jsonString(fs::absolute(scenePath, ec).lexically_normal().string()) +
                           ",\"motion\":" + (motionDocument ? jsonString(motionDocument->path) : std::string{"null"}) +
                           ",\"timestep\":" + hl::formatNumber(dt) + ",\"steps\":" + std::to_string(total) +
                           ",\"renderer\":" + (cameras ? jsonString(rendererName) : std::string{"null"}) +
                           ",\"noise\":" + (noise.active() ? "true" : "false") + ",\"noise_seed\":" + std::to_string(noise.seed()) + ",\"imus\":[";
        for (std::size_t k = 0; k < d.sensors.size(); ++k) {
            const double rate = k < scene.sensors.size() ? scene.sensors[k].rateHz : 1.0 / dt;
            json += std::string{k > 0 ? "," : ""} + "{\"name\":" + jsonString(d.sensors[k].name) + ",\"rate_hz\":" + hl::formatNumber(rate) + "}";
        }
        json += "],\"lidars\":[";
        for (std::size_t k = 0; k < scene.lidars.size(); ++k) {
            const auto& l = scene.lidars[k];
            json += std::string{k > 0 ? "," : ""} + "{\"name\":" + jsonString(l.name) + ",\"rate_hz\":" + hl::formatNumber(l.rateHz) +
                    ",\"samples\":" + std::to_string(l.samples) + ",\"angle_min\":" + hl::formatNumber(l.angleMin) +
                    ",\"angle_max\":" + hl::formatNumber(l.angleMax) + ",\"range_min\":" + hl::formatNumber(l.rangeMin) +
                    ",\"range_max\":" + hl::formatNumber(l.rangeMax) + "}";
        }
        json += "],\"lidars3d\":[";  // M5-11
        for (std::size_t k = 0; k < scene.lidars3d.size(); ++k) {
            const auto& l = scene.lidars3d[k];
            json += std::string{k > 0 ? "," : ""} + "{\"name\":" + jsonString(l.name) + ",\"directory\":" + jsonString("lidars3d/" + safeName(l.name)) +
                    ",\"preset\":" + jsonString(l.preset) + ",\"rate_hz\":" + hl::formatNumber(l.rateHz) + ",\"points_per_second\":" +
                    hl::formatNumber(l.pointsPerSecond) + ",\"points_per_frame\":" + std::to_string(l.pointsPerFrame()) + ",\"range_min\":" +
                    hl::formatNumber(l.rangeMin) + ",\"range_max\":" + hl::formatNumber(l.rangeMax) + ",\"motion_distortion\":" +
                    (l.motionDistortion ? "true" : "false") + "}";
        }
        json += "],\"cameras\":[";
        for (std::size_t k = 0; k < scene.cameras.size() && cameras; ++k) {
            const auto& c = scene.cameras[k];
            const auto in = sc::cameraIntrinsics(c.width, c.height, c.fovyDegrees);
            json += std::string{k > 0 ? "," : ""} + "{\"name\":" + jsonString(c.name) + ",\"directory\":" + jsonString("cameras/" + safeName(c.name)) +
                    ",\"width\":" + std::to_string(c.width) + ",\"height\":" + std::to_string(c.height) + ",\"fx\":" + hl::formatNumber(in.fx) +
                    ",\"fy\":" + hl::formatNumber(in.fy) + ",\"cx\":" + hl::formatNumber(in.cx) + ",\"cy\":" + hl::formatNumber(in.cy) +
                    ",\"near\":" + hl::formatNumber(c.nearClip) + ",\"far\":" + hl::formatNumber(c.farClip) + ",\"rate_hz\":" +
                    hl::formatNumber(c.rateHz) + ",\"rgb\":" + (c.rgb ? "true" : "false") + ",\"depth\":" + (c.depth ? "true" : "false") +
                    ",\"display_rotation_deg\":" + std::to_string(c.displayRotation) + ",\"segmentation\":" + (c.segmentation ? "true" : "false") + "}";
        }
        json += "]}\n";
        std::ofstream file(out / "capture.json", std::ios::binary);
        file << json;
        if (!file) {
            diagnose("无法写入 capture.json");
            return 1;
        }
    }

    std::ofstream sensors(out / "sensors.jsonl", std::ios::binary);
    std::vector<std::ofstream> frameLogs;
    std::vector<fs::path> cameraDirs;
    if (cameras) {
        for (const auto& c : scene.cameras) {
            const fs::path dir = out / "cameras" / safeName(c.name);
            fs::create_directories(dir / "rgb", ec);
            fs::create_directories(dir / "depth", ec);
            if (c.segmentation) {  // M4-09：分割图与对照表（对照表只写一次）
                fs::create_directories(dir / "seg", ec);
                std::ofstream(dir / "seg_labels.json", std::ios::binary) << vw::segmentationLabelsJson(scene);
            }
            cameraDirs.push_back(dir);
            frameLogs.emplace_back(dir / "frames.jsonl", std::ios::binary);
        }
    }
    // M5-11：3D 雷达的输出目录
    std::vector<fs::path> lidar3dDirs;
    std::vector<std::ofstream> lidar3dLogs;
    for (const auto& l : scene.lidars3d) {
        const fs::path dir = out / "lidars3d" / safeName(l.name);
        fs::create_directories(dir, ec);
        lidar3dDirs.push_back(dir);
        lidar3dLogs.emplace_back(dir / "frames.jsonl", std::ios::binary);
    }
    std::uint64_t clouds = 0;
    std::uint64_t imuLines = 0;
    std::uint64_t scans = 0;
    std::uint64_t frames = 0;
    bool written = static_cast<bool>(sensors);

    // 第 s 步的状态：按各传感器的频率采样
    const auto sample = [&]() {
        const auto snap = world.snapshot();
        const std::uint64_t step = snap->stepIndex;
        for (std::size_t k = 0; k < d.sensors.size() && k < snap->sensors.size(); ++k) {
            const double rate = k < scene.sensors.size() ? scene.sensors[k].rateHz : 1.0 / dt;
            if (!sim::sensorSampleDue(step, dt, rate)) continue;
            auto reading = snap->sensors[k];
            noise.imu(k, step, reading.values);  // M4-09
            sensors << hl::sensorToJson(d.sensors[k], reading, step, snap->simulationTime) << '\n';
            ++imuLines;
        }
        for (std::size_t k = 0; k < scene.lidars.size(); ++k) {
            const auto& lidar = scene.lidars[k];
            if (!sim::sensorSampleDue(step, dt, lidar.rateHz)) continue;
            auto scan = sim::scanLidar(world, lidar);
            noise.lidar(k, scan);  // M4-09
            hl::LidarJson meta;
            meta.name = scan.sensor;
            meta.step = scan.step;
            meta.time = scan.time;
            meta.angleMin = scan.angleMin;
            meta.angleIncrement = scan.angleIncrement;
            meta.rangeMin = scan.rangeMin;
            meta.rangeMax = scan.rangeMax;
            meta.rateHz = lidar.rateHz;
            sensors << hl::lidarToJson(meta, scan.ranges) << '\n';
            ++scans;
        }
        for (std::size_t k = 0; k < scene.lidars3d.size(); ++k) {  // M5-11
            const auto& lidar = scene.lidars3d[k];
            if (!sim::sensorSampleDue(step, dt, lidar.rateHz)) continue;
            auto scan = sim::scanLidar3d(world, lidar);
            noise.lidar3d(k, scan);
            const std::string name = stepName(step);
            written = written && writePointsNpy(lidar3dDirs[k] / (name + ".npy"), scan);
            lidar3dLogs[k] << "{\"step\":" << step << ",\"time\":" << hl::formatNumber(scan.time) << ",\"frame\":" << scan.frame
                           << ",\"frame_start\":" << hl::formatNumber(scan.frameStart) << ",\"position\":" << vec3(scan.pose.position)
                           << ",\"orientation\":[" << hl::formatNumber(scan.pose.orientation.w) << "," << hl::formatNumber(scan.pose.orientation.x) << ","
                           << hl::formatNumber(scan.pose.orientation.y) << "," << hl::formatNumber(scan.pose.orientation.z) << "],\"points\":"
                           << scan.points.size() << ",\"sha256\":\"" << sim::lidar3dDigest(scan) << "\",\"file\":\"" << name << ".npy\"}\n";
            ++clouds;
        }
        for (std::size_t k = 0; k < cameraDirs.size(); ++k) {
            const auto& c = scene.cameras[k];
            if (!sim::sensorSampleDue(step, dt, c.rateHz)) continue;
            vw::CameraFrame frame = renderer->capture(scene, *view, c, *snap, lighting);
            noise.color(k, step, frame.rgb, 3);  // M4-09
            noise.depth(k, step, frame.depth);
            const std::string name = stepName(step);
            std::string line = "{\"step\":" + std::to_string(step) + ",\"time\":" + hl::formatNumber(frame.time) + ",\"position\":" +
                               vec3(frame.pose.position) + ",\"orientation\":[" + hl::formatNumber(frame.pose.orientation.w) + "," +
                               hl::formatNumber(frame.pose.orientation.x) + "," + hl::formatNumber(frame.pose.orientation.y) + "," +
                               hl::formatNumber(frame.pose.orientation.z) + "]";
            if (!frame.rgb.empty()) {
                if (const auto error = terra::render::savePng((cameraDirs[k] / "rgb" / (name + ".png")).string(), vw::colorImage(frame))) {
                    diagnose("写 PNG 失败：" + *error);
                    written = false;
                }
                line += ",\"rgb\":\"rgb/" + name + ".png\"";
            }
            if (!frame.depth.empty()) {
                written = written && writeNpy(cameraDirs[k] / "depth" / (name + ".npy"), frame.depth, c.width, c.height);
                line += ",\"depth\":\"depth/" + name + ".npy\"";
            }
            if (!frame.segmentation.empty()) {  // M4-09
                if (!vw::writeSegmentationNpy((cameraDirs[k] / "seg" / (name + ".npy")).string(), frame.segmentation, c.width, c.height)) {
                    diagnose("写分割图失败（实体 ID 超过 65535，或无法写文件）：" + c.name);
                    written = false;
                }
                line += ",\"segmentation\":\"seg/" + name + ".npy\"";
            }
            frameLogs[k] << line << "}\n";
            ++frames;
        }
    };

    sample();
    for (std::uint64_t i = 0; i < total && written; ++i) {
        if (player || !pd.empty()) {
            auto commands = player ? player->commands(world.stepIndex(), dt) : std::vector<terra::physics::ActuatorCommand>{};
            if (!pd.empty()) commands = pd.process(std::move(commands), *world.snapshot());
            if (!commands.empty()) {
                if (const auto error = world.setControl(commands)) {
                    diagnose("第 " + std::to_string(world.stepIndex()) + " 步的控制被拒绝：" + error->message);
                    return 5;
                }
            }
        }
        if (const auto error = world.step()) {
            diagnose("仿真在第 " + std::to_string(error->step) + " 步出错：" + error->message);
            return 5;
        }
        sample();
    }
    sensors.flush();
    for (auto& log : frameLogs) {
        log.flush();
        written = written && static_cast<bool>(log);
    }
    for (auto& log : lidar3dLogs) {
        log.flush();
        written = written && static_cast<bool>(log);
    }
    if (!written || !sensors) {
        diagnose("写入输出失败：" + out.string());
        return 1;
    }
    std::printf("INFO 采集完成：%llu 步，IMU %llu 行，雷达 %llu 次扫描，相机 %llu 帧 → %s\n", static_cast<unsigned long long>(total),
                static_cast<unsigned long long>(imuLines), static_cast<unsigned long long>(scans), static_cast<unsigned long long>(frames),
                out.string().c_str());
    if (!scene.lidars3d.empty()) std::printf("INFO 3D 激光雷达 %llu 帧\n", static_cast<unsigned long long>(clouds));  // M5-11
    // 图形资源早于上下文释放
    renderer.reset();
    view.reset();
    environments.reset();
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "-h" || a == "--help") {
            std::fputs(kUsage, stdout);
            return 0;
        }
    }
    const auto options = parse(argc, argv);
    if (!options) {
        std::fputs(kUsage, stderr);
        return 2;
    }
    try {
        return run(*options);
    } catch (const std::exception& e) {
        diagnose(std::string{"内部错误："} + e.what());
    }
    return 1;
}
