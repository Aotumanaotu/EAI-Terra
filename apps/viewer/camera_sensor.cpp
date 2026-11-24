#include "camera_sensor.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "terra/scene/pose_math.hpp"

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
    return frame;
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
        // 红 → 黄 → 青 → 蓝
        const float rC = std::clamp(1.5f - 3.0f * std::fabs(t - 0.0f), 0.0f, 1.0f) + std::clamp(1.5f - 3.0f * std::fabs(t - 0.33f), 0.0f, 1.0f);
        const float gC = std::clamp(1.5f - 3.0f * std::fabs(t - 0.33f), 0.0f, 1.0f) + std::clamp(1.5f - 3.0f * std::fabs(t - 0.66f), 0.0f, 1.0f);
        const float bC = std::clamp(1.5f - 3.0f * std::fabs(t - 0.66f), 0.0f, 1.0f) + std::clamp(1.5f - 3.0f * std::fabs(t - 1.0f), 0.0f, 1.0f);
        px[0] = static_cast<std::uint8_t>(255.0f * std::min(rC, 1.0f));
        px[1] = static_cast<std::uint8_t>(255.0f * std::min(gC, 1.0f));
        px[2] = static_cast<std::uint8_t>(255.0f * std::min(bC, 1.0f));
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

}  // namespace terra::viewer
