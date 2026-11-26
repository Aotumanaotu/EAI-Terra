#include "camera_sensor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <string>
#include <utility>

#include "terra/scene/pose_math.hpp"
#include "terra/simulation/thermal.hpp"

namespace terra::viewer {

namespace r = terra::render;
namespace sc = terra::scene;
namespace td = terra::data;

r::ViewParams cameraViewParams(const td::Pose& cameraWorld, const sc::CameraSensor& camera) {
    r::ViewParams params;
    // Terra 相机坐标系与 OpenGL 的观察坐标系相同（看向 −Z，+Y 向上），观察矩阵即相机位姿的逆
    params.view = poseMatrix(sc::inverse(cameraWorld));
    const double f = 1.0 / std::tan(camera.fovyDegrees * 3.14159265358979323846 / 360.0);
    const double aspect = static_cast<double>(camera.width) / static_cast<double>(camera.height);
    const double n = camera.nearClip;
    const double far = camera.farClip;
    r::Matrix4 p{};
    p[0] = static_cast<float>(f / aspect);
    p[5] = static_cast<float>(f);
    p[10] = static_cast<float>((far + n) / (n - far));
    p[11] = -1.0f;
    p[14] = static_cast<float>(2.0 * far * n / (n - far));
    params.projection = p;
    params.cameraPosition = {static_cast<float>(cameraWorld.position.x), static_cast<float>(cameraWorld.position.y),
                             static_cast<float>(cameraWorld.position.z)};
    return params;
}

std::unique_ptr<CameraRenderer> CameraRenderer::create(r::RenderContext& context, std::string& error) {
    auto color = r::SceneRenderer::create(context);
    if (!color.renderer) {
        error = color.error.value_or("彩色渲染器创建失败");
        return nullptr;
    }
    auto depth = r::DepthRenderer::create(context);
    if (!depth.renderer) {
        error = depth.error.value_or("深度渲染器创建失败");
        return nullptr;
    }
    std::unique_ptr<CameraRenderer> out{new CameraRenderer()};
    out->color_ = std::move(color.renderer);
    out->depth_ = std::move(depth.renderer);
    return out;
}

int CameraRenderer::depthTargets() const noexcept { return depth_ ? depth_->targetCount() : 0; }

CameraFrame CameraRenderer::capture(const sc::SceneInstance& scene, const SceneView& view, const sc::CameraSensor& camera,
                                    const td::PhysicsSnapshot& snapshot, const FrameLighting& lighting) {
    CameraFrame frame;
    frame.sensor = camera.name;
    frame.step = snapshot.stepIndex;
    frame.time = snapshot.simulationTime;
    frame.runEpoch = snapshot.runEpoch;
    frame.pose = sc::cameraWorldPose(snapshot, camera);
    frame.intrinsics = sc::cameraIntrinsics(camera.width, camera.height, camera.fovyDegrees);
    const r::ViewParams params = cameraViewParams(frame.pose, camera);
    // 传感器看到的是物体本身：默认分组，不高亮，不叠加碰撞几何体
    const auto items = view.items(scene, sc::visualPoses(scene, sc::entityPoses(snapshot)), SceneViewOptions{});
    if (camera.rgb) {
        r::RenderSettings settings = lighting.settings;
        settings.grid = false;
        color_->render(camera.width, camera.height, params, items, lighting.sun, lighting.hemisphere, settings, lighting.image);
        const std::vector<std::uint8_t> bottomUp = color_->readOutput(0, 0, camera.width, camera.height);
        const auto w = static_cast<std::size_t>(camera.width);
        const auto h = static_cast<std::size_t>(camera.height);
        frame.rgb.resize(w * h * 3);
        for (std::size_t row = 0; row < h && bottomUp.size() == w * h * 4; ++row) {
            for (std::size_t col = 0; col < w; ++col) {
                const std::uint8_t* src = bottomUp.data() + ((h - 1 - row) * w + col) * 4;
                std::uint8_t* dst = frame.rgb.data() + (row * w + col) * 3;
                dst[0] = src[0];
                dst[1] = src[1];
                dst[2] = src[2];
            }
        }
    }
    if (camera.depth) frame.depth = depth_->render(camera.width, camera.height, params, items);
    if (camera.segmentation) frame.segmentation = depth_->renderIds(camera.width, camera.height, params, items);  // M4-09
    if (camera.thermal.enabled) {
        // M5-06：温度由同一帧的分割图与深度算出（相机不输出它们时也在这里渲染，只用于温度）
        const std::vector<float> depth = camera.depth ? frame.depth : depth_->render(camera.width, camera.height, params, items);
        const std::vector<std::uint32_t> ids = camera.segmentation ? frame.segmentation : depth_->renderIds(camera.width, camera.height, params, items);
        const std::vector<td::Pose> poses = sc::entityPoses(snapshot);
        terra::simulation::ThermalFrameInput in;
        in.width = camera.width;
        in.height = camera.height;
        in.fovyDegrees = camera.fovyDegrees;
        in.cameraWorld = frame.pose;
        in.time = snapshot.simulationTime;
        in.ids = &ids;
        in.depth = &depth;
        in.entityPoses = &poses;
        frame.thermal = terra::simulation::thermalTruth(scene.thermal, in);
    }
    return frame;
}

// ---- 热成像（M5-06）----

r::ImageRgba8 thermalImage(const CameraFrame& frame, const sc::ThermalCameraSpec& spec) {
    r::ImageRgba8 image;
    image.width = frame.intrinsics.width;
    image.height = frame.intrinsics.height;
    const auto [lo, hi] = terra::simulation::thermalDisplayRange(spec, frame.thermal);
    const std::vector<std::uint8_t> rgb = terra::simulation::thermalColor(frame.thermal, spec.palette, lo, hi);
    image.pixels.assign(frame.thermal.size() * 4, 255);
    for (std::size_t i = 0; i < frame.thermal.size(); ++i) {
        for (std::size_t c = 0; c < 3; ++c) image.pixels[i * 4 + c] = rgb[i * 3 + c];
    }
    return image;
}

bool writeThermalNpy(const std::string& path, const std::vector<float>& celsius, int width, int height) { return writeDepthNpy(path, celsius, width, height); }

// ---- 分割图（M4-09）----

r::ImageRgba8 segmentationPreview(const std::vector<std::uint32_t>& ids, int width, int height) {
    r::ImageRgba8 image;
    image.width = width;
    image.height = height;
    image.pixels.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4, 0);
    for (std::size_t i = 0; i < ids.size() && i * 4 < image.pixels.size(); ++i) {
        std::uint8_t* px = image.pixels.data() + i * 4;
        px[3] = 255;
        if (ids[i] == 0) continue;
        // 固定的配色：按 ID 散列取色相（黄金角），饱和、明亮，相邻 ID 颜色差别大
        const double hue = std::fmod(static_cast<double>(ids[i]) * 137.508, 360.0) / 60.0;
        const double c = 0.85, x = c * (1.0 - std::fabs(std::fmod(hue, 2.0) - 1.0)), m = 0.12;
        double rgb[3] = {0, 0, 0};
        const int sector = static_cast<int>(hue);
        const double table[6][3] = {{c, x, 0}, {x, c, 0}, {0, c, x}, {0, x, c}, {x, 0, c}, {c, 0, x}};
        for (int k = 0; k < 3; ++k) rgb[k] = table[std::clamp(sector, 0, 5)][k] + m;
        for (int k = 0; k < 3; ++k) px[k] = static_cast<std::uint8_t>(std::lround(255.0 * std::clamp(rgb[k], 0.0, 1.0)));
    }
    return image;
}

std::string segmentationLabelsJson(const sc::SceneInstance& scene) { return sc::segmentationLabelsJson(scene); }

bool writeSegmentationNpy(const std::string& path, const std::vector<std::uint32_t>& ids, int width, int height) {
    std::vector<std::uint16_t> narrow(ids.size());
    for (std::size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] > 0xFFFFu) return false;
        narrow[i] = static_cast<std::uint16_t>(ids[i]);
    }
    std::string header = "{'descr': '<u2', 'fortran_order': False, 'shape': (" + std::to_string(height) + ", " + std::to_string(width) + "), }";
    const std::size_t prefix = 10;
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
    out.write(reinterpret_cast<const char*>(narrow.data()), static_cast<std::streamsize>(narrow.size() * sizeof(std::uint16_t)));
    return static_cast<bool>(out);
}

r::ImageRgba8 depthPreview(const std::vector<float>& depth, int width, int height, float nearDepth, float maxDepth) {
    r::ImageRgba8 image;
    image.width = width;
    image.height = height;
    image.pixels.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 4, 0);
    const float span = std::max(maxDepth - nearDepth, 1e-6f);
    for (std::size_t i = 0; i < depth.size() && i * 4 < image.pixels.size(); ++i) {
        std::uint8_t* px = image.pixels.data() + i * 4;
        px[3] = 255;
        if (!std::isfinite(depth[i])) continue;
        const float t = std::clamp((depth[i] - nearDepth) / span, 0.0f, 1.0f);
        // 分段线性配色（M4-04 起）：近处红 → 橙 → 绿 → 青 → 蓝 → 远处深蓝；相邻颜色之间线性过渡，不会叠成白色或黄色
        static constexpr float kStops[6][3] = {{0.95f, 0.15f, 0.10f}, {1.00f, 0.55f, 0.05f}, {0.25f, 0.80f, 0.30f},
                                               {0.10f, 0.70f, 0.90f}, {0.15f, 0.30f, 0.95f}, {0.10f, 0.10f, 0.45f}};
        const float position = t * 5.0f;
        const auto lower = static_cast<std::size_t>(std::min(position, 4.999f));
        const float fraction = position - static_cast<float>(lower);
        for (std::size_t c = 0; c < 3; ++c) {
            const float v = kStops[lower][c] + (kStops[lower + 1][c] - kStops[lower][c]) * fraction;
            px[c] = static_cast<std::uint8_t>(std::lround(255.0f * std::clamp(v, 0.0f, 1.0f)));
        }
    }
    return image;
}

r::ImageRgba8 colorImage(const CameraFrame& frame) {
    r::ImageRgba8 image;
    image.width = frame.intrinsics.width;
    image.height = frame.intrinsics.height;
    image.pixels.assign(static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height) * 4, 255);
    for (std::size_t i = 0; i * 3 + 2 < frame.rgb.size() && i * 4 + 3 < image.pixels.size(); ++i) {
        for (std::size_t c = 0; c < 3; ++c) image.pixels[i * 4 + c] = frame.rgb[i * 3 + c];
    }
    return image;
}

bool writeDepthNpy(const std::string& path, const std::vector<float>& depth, int width, int height) {
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
    // 只在小端机器上运行（x86-64），float 的内存布局即 '<f4'
    out.write(reinterpret_cast<const char*>(depth.data()), static_cast<std::streamsize>(depth.size() * sizeof(float)));
    return static_cast<bool>(out);
}

std::vector<float> readDepthNpy(const std::string& path, int width, int height) {
    std::ifstream in(path, std::ios::binary);
    if (!in || width <= 0 || height <= 0) return {};
    unsigned char head[10];
    in.read(reinterpret_cast<char*>(head), 10);
    if (!in || head[0] != 0x93 || head[1] != 'N' || head[6] != 1) return {};
    const std::size_t length = static_cast<std::size_t>(head[8]) | (static_cast<std::size_t>(head[9]) << 8);
    std::string header(length, '\0');
    in.read(header.data(), static_cast<std::streamsize>(length));
    const std::string shape = "(" + std::to_string(height) + ", " + std::to_string(width) + ")";
    if (!in || header.find("'<f4'") == std::string::npos || header.find(shape) == std::string::npos) return {};
    std::vector<float> depth(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));
    in.read(reinterpret_cast<char*>(depth.data()), static_cast<std::streamsize>(depth.size() * sizeof(float)));
    if (!in) return {};
    return depth;
}

}  // namespace terra::viewer
