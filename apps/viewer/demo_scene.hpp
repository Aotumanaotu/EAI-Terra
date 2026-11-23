#pragma once

// 查看器的演示场景、环境贴图库与渲染设置换算（M1-08 起；M1-10 从 main.cpp 移出为 terra_viewer_scene 库），
// 供 terra_viewer、参考图测试（graphics.render_reference）与渲染基准（terra_render_bench）共用，三者画面完全一致。
// 不依赖界面库；日志以 SceneMessage 返回，由调用方翻译显示。

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "terra/asset/gpu_asset.hpp"
#include "terra/render/environment.hpp"
#include "terra/render/hdr_image.hpp"
#include "terra/render/pbr.hpp"
#include "terra/render/scene_renderer.hpp"
#include "terra/settings/settings.hpp"

namespace terra::render {
class RenderContext;
}

namespace terra::viewer {

constexpr double kRadiansPerDegree = 3.14159265358979323846 / 180.0;

// ---- 设置 → 渲染参数 ----
terra::render::ToneMapper toToneMapper(terra::settings::ToneMapperSetting mapper) noexcept;
// 太阳方向由方位角与高度角换算（指向光源）
terra::render::SunLight toSunLight(const terra::settings::RenderSettings& render);
// 半球光：天空色与地面色固定，强度来自设置
terra::render::HemisphereLight toEnvironment(const terra::settings::RenderSettings& render);
terra::render::RenderSettings toRenderSettings(const terra::settings::RenderSettings& render);

struct SceneMessage {
    enum class Kind { AssetLoaded, AssetInfo, AssetWarning, AssetError, EnvironmentLoaded, EnvironmentFailed };
    Kind kind;
    std::string text;
};

// ---- 演示场景：地面、立方体、两排材质球（金属度 0→1、粗糙度 0→1）、圆柱、胶囊；可选三个 glTF 资产 ----
class DemoScene {
public:
    static terra::render::Material cubeMaterial();

    // 程序几何体。网格存放在本对象中，渲染项指向它们：创建后不能移动。
    void create();
    // glTF 资产（M1-09）：assetsDir/models/ 中存在时加载，放在相机前方的地面上；不存在的跳过。
    void loadAssets(const std::string& assetsDir, std::vector<SceneMessage>& messages);

    struct LoadedAsset {
        std::string name;
        terra::asset::GpuAsset gpu;
        terra::render::Matrix4 placement{};
        std::size_t triangles = 0;
        int textures = 0;
    };

    // armOcclusion：资产是否使用按 ARM 约定推断的遮蔽贴图（M1-10）。
    const std::vector<terra::render::RenderItem>& items(bool armOcclusion = true) const noexcept {
        return armOcclusion ? itemsArm_ : itemsPlain_;
    }
    const std::vector<LoadedAsset>& assets() const noexcept { return assets_; }

private:
    void rebuild();

    terra::render::GpuMesh ground_;
    terra::render::GpuMesh cube_;
    terra::render::GpuMesh sphere_;
    terra::render::GpuMesh cylinder_;
    terra::render::GpuMesh capsule_;
    std::vector<terra::render::RenderItem> procedural_;
    std::vector<LoadedAsset> assets_;
    std::vector<terra::render::RenderItem> itemsArm_;
    std::vector<terra::render::RenderItem> itemsPlain_;
};

// ---- 环境贴图：首次选用时读取 assetsDir/hdri/ 中的 .hdr，提取并去掉其中的太阳，在 GPU 上预计算；失败只报告一次 ----
class EnvironmentLibrary {
public:
    explicit EnvironmentLibrary(std::string assetsDir) : assetsDir_(std::move(assetsDir)) {}

    const terra::render::Environment* get(terra::settings::EnvironmentSetting which, terra::render::RenderContext& ctx,
                                          std::vector<SceneMessage>& messages);
    // 已加载的环境贴图中提取到的太阳；未加载或没有明显太阳时为空。
    std::optional<terra::render::SunEstimate> sun(terra::settings::EnvironmentSetting which) const;
    // 贴图文件是否存在（未运行 scripts/assets/fetch_hdri.sh 时不存在）。
    bool available(terra::settings::EnvironmentSetting which) const;
    static const char* fileName(terra::settings::EnvironmentSetting which);

private:
    struct Slot {
        std::unique_ptr<terra::render::Environment> environment;
        std::optional<terra::render::SunEstimate> sun;
        bool failed = false;
    };
    std::string assetsDir_;
    Slot outdoor_;
    Slot studio_;
};

// 一帧的光照：环境贴图光照参数，以及（开启跟随且贴图中有太阳时）跟随贴图的方向光。
struct FrameLighting {
    terra::render::SunLight sun;
    terra::render::HemisphereLight hemisphere;
    terra::render::EnvironmentLighting image;
    terra::render::RenderSettings settings;
};
FrameLighting frameLighting(const terra::settings::RenderSettings& rs, EnvironmentLibrary& environments,
                            terra::render::RenderContext& ctx, std::vector<SceneMessage>& messages);

}  // namespace terra::viewer
