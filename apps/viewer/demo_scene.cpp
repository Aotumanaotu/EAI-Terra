#include "demo_scene.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

#include "terra/asset/gltf.hpp"
#include "terra/render/mesh.hpp"

namespace terra::viewer {

namespace r = terra::render;
namespace st = terra::settings;

// ---- 设置 → 渲染参数 ----

r::ToneMapper toToneMapper(st::ToneMapperSetting mapper) noexcept {
    switch (mapper) {
        case st::ToneMapperSetting::AgX: return r::ToneMapper::AgX;
        case st::ToneMapperSetting::PbrNeutral: return r::ToneMapper::PbrNeutral;
        case st::ToneMapperSetting::Aces: return r::ToneMapper::Aces;
        case st::ToneMapperSetting::None: return r::ToneMapper::None;
    }
    return r::ToneMapper::AgX;
}

r::SunLight toSunLight(const st::RenderSettings& render) {
    const double az = render.sunAzimuthDegrees * kRadiansPerDegree;
    const double el = render.sunElevationDegrees * kRadiansPerDegree;
    r::SunLight sun;
    sun.direction = {static_cast<float>(std::cos(el) * std::cos(az)), static_cast<float>(std::cos(el) * std::sin(az)),
                     static_cast<float>(std::sin(el))};
    sun.color = {1.0f, 0.95f, 0.88f};  // 略暖的白光
    sun.intensity = static_cast<float>(render.sunIntensity);
    return sun;
}

r::HemisphereLight toEnvironment(const st::RenderSettings& render) {
    r::HemisphereLight env;  // 天空色与地面色用渲染模块的默认值
    env.intensity = static_cast<float>(render.ambientIntensity);
    return env;
}

r::RenderSettings toRenderSettings(const st::RenderSettings& render) {
    r::RenderSettings out;
    out.toneMapper = toToneMapper(render.toneMapper);
    out.exposureEv = static_cast<float>(render.exposureEv);
    out.msaaSamples = render.msaaSamples;
    out.shadows = render.shadows;
    out.grid = render.grid;
    return out;
}

// ---- 演示场景 ----

r::Material DemoScene::cubeMaterial() {
    r::Material m;
    m.baseColor = {0.80f, 0.42f, 0.22f};
    m.roughness = 0.45f;
    return m;
}

void DemoScene::create() {
    ground_ = r::GpuMesh::create(r::meshes::plane(20.0f, 20.0f));
    cube_ = r::GpuMesh::create(r::meshes::box(0.5f, 0.5f, 0.5f));
    sphere_ = r::GpuMesh::create(r::meshes::sphere(0.45f));
    cylinder_ = r::GpuMesh::create(r::meshes::cylinder(0.35f, 0.5f));
    capsule_ = r::GpuMesh::create(r::meshes::capsule(0.3f, 0.35f));

    procedural_.clear();
    r::Material ground;
    ground.baseColor = {0.42f, 0.42f, 0.43f};
    ground.roughness = 0.85f;
    procedural_.push_back({&ground_, r::identityMatrix(), ground, false});
    procedural_.push_back({&cube_, r::translationScale({0.0f, 0.0f, 0.5f}), cubeMaterial(), true});
    for (int i = 0; i < 5; ++i) {
        const float t = static_cast<float>(i) / 4.0f;
        const float y = -3.2f + 1.6f * static_cast<float>(i);
        r::Material metal;  // 金属度 0 → 1
        metal.baseColor = {0.95f, 0.72f, 0.30f};
        metal.roughness = 0.3f;
        metal.metallic = t;
        procedural_.push_back({&sphere_, r::translationScale({2.2f, y, 0.45f}), metal, true});
        r::Material rough;  // 粗糙度 0 → 1
        rough.baseColor = {0.15f, 0.38f, 0.80f};
        rough.roughness = t;
        procedural_.push_back({&sphere_, r::translationScale({3.4f, y, 0.45f}), rough, true});
    }
    r::Material green;
    green.baseColor = {0.30f, 0.65f, 0.38f};
    green.roughness = 0.6f;
    procedural_.push_back({&cylinder_, r::translationScale({0.5f, -2.2f, 0.5f}), green, true});
    r::Material steel;
    steel.baseColor = {0.86f, 0.86f, 0.88f};
    steel.metallic = 1.0f;
    steel.roughness = 0.25f;
    procedural_.push_back({&capsule_, r::translationScale({0.5f, 2.2f, 0.65f}), steel, true});
    rebuild();
}

void DemoScene::loadAssets(const std::string& assetsDir, std::vector<SceneMessage>& messages) {
    struct Spec {
        const char* name;
        r::Vec3f position;
    };
    const Spec specs[] = {{"metal_toolbox", {-2.4f, 1.1f, 0.0f}}, {"Drill_01", {-2.9f, -0.45f, 0.0f}},
                          {"brass_vase_01", {-2.4f, -1.25f, 0.0f}}};
    for (const Spec& spec : specs) {
        const std::string path = assetsDir + "/models/" + spec.name + "/" + spec.name + "_2k.gltf";
        std::FILE* probe = std::fopen(path.c_str(), "rb");
        if (probe == nullptr) {
            std::printf("INFO 未找到资产 %s（未运行 scripts/assets/fetch_models.sh），跳过\n", path.c_str());
            continue;
        }
        std::fclose(probe);
        const auto loaded = terra::asset::loadGltf(path);
        for (const auto& d : loaded.diagnostics) {
            using S = terra::asset::Diagnostic::Severity;
            const SceneMessage::Kind kind = d.severity == S::Error     ? SceneMessage::Kind::AssetError
                                            : d.severity == S::Warning ? SceneMessage::Kind::AssetWarning
                                                                       : SceneMessage::Kind::AssetInfo;
            messages.push_back({kind, d.message});
            std::printf("INFO 资产 %s %s：%s\n", spec.name, d.severity == S::Error ? "错误" : d.severity == S::Warning ? "警告" : "信息",
                        d.message.c_str());
        }
        if (!loaded.asset) continue;
        LoadedAsset a;
        a.name = spec.name;
        a.gpu = terra::asset::GpuAsset::create(*loaded.asset);
        a.placement = r::translationScale(spec.position);
        a.triangles = loaded.asset->triangleCount();
        a.textures = a.gpu.textureCount();
        char buf[200];
        std::snprintf(buf, sizeof(buf), "%s（%zu 个三角形，%d 张贴图，高 %.2f m）", spec.name, a.triangles, a.textures,
                      static_cast<double>(loaded.asset->boundsMax[2] - loaded.asset->boundsMin[2]));
        messages.push_back({SceneMessage::Kind::AssetLoaded, buf});
        std::printf("INFO 已加载资产 %s\n", buf);
        assets_.push_back(std::move(a));
    }
    rebuild();
}

void DemoScene::rebuild() {
    // 资产全部加载后再生成渲染项（渲染项指向 GpuAsset 内部的网格，GpuAsset 移动时网格存储不变）
    itemsArm_ = procedural_;
    itemsPlain_ = procedural_;
    for (const LoadedAsset& a : assets_) {
        const auto withArm = a.gpu.items(a.placement, true);
        const auto withoutArm = a.gpu.items(a.placement, false);
        itemsArm_.insert(itemsArm_.end(), withArm.begin(), withArm.end());
        itemsPlain_.insert(itemsPlain_.end(), withoutArm.begin(), withoutArm.end());
    }
}

// ---- 环境贴图 ----

const char* EnvironmentLibrary::fileName(st::EnvironmentSetting which) {
    return which == st::EnvironmentSetting::Studio ? "studio_small_09_2k.hdr" : "rural_asphalt_road_2k.hdr";
}

bool EnvironmentLibrary::available(st::EnvironmentSetting which) const {
    if (which == st::EnvironmentSetting::Hemisphere) return false;
    std::FILE* f = std::fopen((assetsDir_ + "/hdri/" + fileName(which)).c_str(), "rb");
    if (f == nullptr) return false;
    std::fclose(f);
    return true;
}

std::optional<r::SunEstimate> EnvironmentLibrary::sun(st::EnvironmentSetting which) const {
    if (which == st::EnvironmentSetting::Hemisphere) return std::nullopt;
    const Slot& slot = which == st::EnvironmentSetting::Outdoor ? outdoor_ : studio_;
    return slot.environment ? slot.sun : std::nullopt;
}

const r::Environment* EnvironmentLibrary::get(st::EnvironmentSetting which, r::RenderContext& ctx,
                                              std::vector<SceneMessage>& messages) {
    if (which == st::EnvironmentSetting::Hemisphere) return nullptr;
    Slot& slot = which == st::EnvironmentSetting::Outdoor ? outdoor_ : studio_;
    if (!slot.environment && !slot.failed) {
        const std::string path = assetsDir_ + "/hdri/" + fileName(which);
        auto image = r::loadHdr(path);
        if (!image.ok()) {
            slot.failed = true;
            messages.push_back({SceneMessage::Kind::EnvironmentFailed, image.error});
            std::printf("INFO 环境贴图不可用：%s\n", image.error.c_str());
            return nullptr;
        }
        // 环境贴图中的太阳（M1-09）：提取方向与辐照度并从贴图中去掉，改由方向光（带阴影）表现
        slot.sun = r::extractSun(image.image, true);
        if (slot.sun) {
            const r::Vec3f d = slot.sun->direction;
            std::printf("INFO 环境贴图中的太阳：方向 (%.3f, %.3f, %.3f)，高度角 %.1f°，辐照度亮度 %.2f，立体角 %.2e sr\n",
                        static_cast<double>(d.x), static_cast<double>(d.y), static_cast<double>(d.z),
                        std::asin(static_cast<double>(d.z)) / kRadiansPerDegree,
                        static_cast<double>(r::luminance(slot.sun->irradiance)), static_cast<double>(slot.sun->solidAngle));
        }
        auto made = r::Environment::create(ctx, image.image);
        if (!made.environment) {
            slot.failed = true;
            messages.push_back({SceneMessage::Kind::EnvironmentFailed, made.error.value_or("")});
            return nullptr;
        }
        slot.environment = std::move(made.environment);
        char buf[160];
        std::snprintf(buf, sizeof(buf), "%s（%d×%d，预计算 %.2f 秒）", fileName(which), image.image.width, image.image.height,
                      slot.environment->precomputeSeconds());
        messages.push_back({SceneMessage::Kind::EnvironmentLoaded, buf});
        std::printf("INFO 已加载环境贴图 %s\n", buf);
    }
    return slot.environment.get();
}

FrameLighting frameLighting(const st::RenderSettings& rs, EnvironmentLibrary& environments, r::RenderContext& ctx,
                            std::vector<SceneMessage>& messages) {
    FrameLighting out;
    out.settings = toRenderSettings(rs);
    out.hemisphere = toEnvironment(rs);
    out.sun = toSunLight(rs);
    out.image.environment = environments.get(rs.environment, ctx, messages);
    out.image.intensity = static_cast<float>(rs.environmentIntensity);
    out.image.rotationRadians = static_cast<float>(rs.environmentRotationDegrees * kRadiansPerDegree);
    out.image.background = rs.environmentBackground;
    out.image.backgroundBlur = static_cast<float>(rs.backgroundBlur);
    const auto estimate = environments.sun(rs.environment);
    if (out.image.environment != nullptr && rs.sunFromEnvironment && estimate) {
        // 方向光跟随贴图中的太阳：方向随环境旋转，亮度随环境强度
        const float c = std::cos(out.image.rotationRadians);
        const float s = std::sin(out.image.rotationRadians);
        const r::Vec3f d = estimate->direction;
        out.sun.direction = {c * d.x - s * d.y, s * d.x + c * d.y, d.z};
        const float lum = std::max(r::luminance(estimate->irradiance), 1e-6f);
        out.sun.color = {estimate->irradiance.x / lum, estimate->irradiance.y / lum, estimate->irradiance.z / lum};
        out.sun.intensity = lum * out.image.intensity;
    }
    return out;
}

}  // namespace terra::viewer
