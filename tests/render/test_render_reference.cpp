// 参考图测试（M1-10）。Xvfb + llvmpipe（D-022）中渲染固定场景（与 terra_viewer 共用 terra_viewer_scene），
// 与 tests/render/reference/ 中提交的 PNG 比较（规则见 terra/render/image.hpp：每通道容差 3、不同像素 ≤ 0.1%、最大差 ≤ 16）。
//
// 场景：
//   procedural  程序几何体、半球光、手动太阳与阴影、地面网格；不依赖下载素材
//   studio      摄影棚环境贴图 + 三个 glTF 资产（需要 HDRI 与资产，缺失时以返回码 77 表示跳过）
//   outdoor     室外环境贴图（太阳由贴图提取）+ 三个 glTF 资产（同上）
//   carts       （M2-07）场景文件 assets/tests/scene/two_carts.terra-scene.json：物理世界推进 300 步（两车各施加恒定推力）后，
//               由快照驱动渲染 MJCF 几何体与 glTF 节点绑定（node / assembly 对齐、showGeoms、静态装饰）；半球光；不需要下载素材
//   mjcf_materials（M3-03）场景文件 assets/tests/scene/materials.terra-scene.json：MJCF 材质、二维贴图、平滑法线；半球光
//   robot_g1 / robot_go2 / robot_stretch3（M3-03）参考机器人场景，处于初始关键帧；摄影棚环境
//               （需要 Menagerie 模型与 HDRI，缺失时以返回码 77 表示跳过）
// 每个场景渲染两次，先检查两次逐字节相同（确定性），再与参考图比较；失败时在输出目录写出实际图与差异图。
//
// 用法：test_render_reference <参考图目录> <输出目录> --scene <名称> [--update] [--expect-renderer 子串]
//   --update  把渲染结果写为新的参考图（只在基线机器上由 scripts/verify/update_references.sh 调用）

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "demo_scene.hpp"
#include "scene_view.hpp"
#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/scene/scene.hpp"
#include "terra/input/input_system.hpp"
#include "terra/platform/glfw/glfw_platform.hpp"
#include "terra/render/context.hpp"
#include "terra/render/image.hpp"
#include "terra/render/scene_renderer.hpp"
#include "terra/view/camera.hpp"

namespace {

namespace fs = std::filesystem;
namespace r = terra::render;
namespace st = terra::settings;
namespace vw = terra::viewer;

constexpr int kWidth = 640;
constexpr int kHeight = 360;
constexpr int kSkipped = 77;

struct SceneSpec {
    std::string name;
    bool assets = false;
    bool sceneFile = false;  // M2-07：由场景文件与物理快照驱动
    st::EnvironmentSetting environment = st::EnvironmentSetting::Hemisphere;
    r::Vec3f eye;
    r::Vec3f target;
    std::string scenePath;   // M3-03：场景文件（相对 assets/）；空表示 carts 的 tests/scene/two_carts.terra-scene.json
    std::string robotCheck;  // M3-03：需要的参考机器人文件（相对 assets/），缺失时跳过；空表示不需要
};

bool spec(const std::string& name, SceneSpec& out) {
    if (name == "procedural") {
        out = {name, false, false, st::EnvironmentSetting::Hemisphere, {-5.0f, 0.0f, 2.0f}, {0.0f, 0.0f, 0.0f}, {}, {}};
    } else if (name == "studio") {
        out = {name, true, false, st::EnvironmentSetting::Studio, {-4.2f, 0.0f, 0.95f}, {-2.55f, 0.0f, 0.25f}, {}, {}};
    } else if (name == "outdoor") {
        out = {name, true, false, st::EnvironmentSetting::Outdoor, {-4.2f, 0.0f, 0.95f}, {-2.55f, 0.0f, 0.25f}, {}, {}};
    } else if (name == "carts") {
        out = {name, false, true, st::EnvironmentSetting::Hemisphere, {-2.2f, -2.4f, 1.5f}, {0.25f, 0.0f, 0.1f}, {}, {}};
    } else if (name == "mjcf_materials") {
        // M3-03：材质、二维贴图（平面 texuniform、长方体、带纹理坐标的网格）、平滑法线与平直着色、不支持的立方体贴图
        out = {name, false, true, st::EnvironmentSetting::Hemisphere, {1.05f, -1.25f, 0.75f}, {0.05f, 0.15f, 0.05f},
               "tests/scene/materials.terra-scene.json", {}};
    } else if (name == "robot_g1") {
        // M3-03：三台参考机器人，处于场景的初始关键帧；摄影棚环境（需要 HDRI 与 Menagerie 模型）
        out = {name, true, true, st::EnvironmentSetting::Studio, {2.2f, -1.6f, 1.3f}, {0.0f, 0.0f, 0.75f},
               "scenes/robots/g1.terra-scene.json", "robots/unitree_g1/g1.xml"};
    } else if (name == "robot_go2") {
        out = {name, true, true, st::EnvironmentSetting::Studio, {1.4f, -1.1f, 0.7f}, {0.0f, 0.0f, 0.25f},
               "scenes/robots/go2.terra-scene.json", "robots/unitree_go2/go2.xml"};
    } else if (name == "robot_stretch3") {
        out = {name, true, true, st::EnvironmentSetting::Studio, {1.8f, 1.4f, 1.4f}, {0.0f, -0.3f, 0.6f},
               "scenes/robots/stretch3.terra-scene.json", "robots/hello_robot_stretch_3/stretch.xml"};
    } else {
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr, "用法：test_render_reference <参考图目录> <输出目录> --scene <procedural|studio|outdoor|carts|mjcf_materials|robot_g1|robot_go2|robot_stretch3> [--update]\n");
        return 2;
    }
    const fs::path referenceDir{argv[1]};
    const fs::path outputDir{argv[2]};
    std::string sceneName;
    std::string expectRenderer;
    bool update = false;
    for (int i = 3; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--scene" && i + 1 < argc) {
            sceneName = argv[++i];
        } else if (a == "--expect-renderer" && i + 1 < argc) {
            expectRenderer = argv[++i];
        } else if (a == "--update") {
            update = true;
        } else {
            std::fprintf(stderr, "未知参数：%s\n", a.c_str());
            return 2;
        }
    }
    SceneSpec s;
    if (!spec(sceneName, s)) {
        std::fprintf(stderr, "未知场景：%s\n", sceneName.c_str());
        return 2;
    }

    if (!s.robotCheck.empty() && !fs::exists(std::string{TERRA_ASSETS_DIR} + "/" + s.robotCheck)) {
        std::printf("INFO 场景 %s 需要参考机器人（scripts/assets/fetch_robots.sh），跳过\n", s.name.c_str());
        return kSkipped;
    }
    if (s.assets) {
        const std::string toolbox = std::string{TERRA_ASSETS_DIR} + "/models/metal_toolbox/metal_toolbox_2k.gltf";
        if (!fs::exists(toolbox) || !vw::EnvironmentLibrary(TERRA_ASSETS_DIR).available(s.environment)) {
            std::printf("INFO 场景 %s 需要 HDRI 与 glTF 资产（scripts/assets/fetch_hdri.sh、fetch_models.sh），跳过\n", s.name.c_str());
            return kSkipped;
        }
    }

    terra::input::InputSystem input;
    auto platform = terra::platform::glfw::createPlatform(input);
    if (!platform.platform) {
        std::fprintf(stderr, "FAILED: 创建平台：%s\n", platform.error ? platform.error->message.c_str() : "");
        return 1;
    }
    terra::platform::WindowDescription desc;
    desc.title = "terra reference";
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
    std::printf("INFO 场景 %s，%s，%d × %d\n", s.name.c_str(), ctx.info().renderer.c_str(), kWidth, kHeight);
    int failures = 0;
    const auto check = [&](bool ok, const std::string& what) {
        if (!ok) {
            std::fprintf(stderr, "FAILED: %s\n", what.c_str());
            ++failures;
        }
    };
    if (!expectRenderer.empty()) {
        check(ctx.info().renderer.find(expectRenderer) != std::string::npos, "渲染器包含 " + expectRenderer + "（D-022）");
    }
    auto made = r::SceneRenderer::create(ctx);
    if (!made.renderer) {
        std::fprintf(stderr, "FAILED: 创建场景渲染器：%s\n", made.error.value_or("").c_str());
        return 1;
    }

    r::ImageRgba8 first;
    r::ImageRgba8 second;
    {
        // GPU 资源（场景、环境贴图）在本作用域内创建与销毁，早于 OpenGL 上下文
        vw::EnvironmentLibrary environments(TERRA_ASSETS_DIR);
        vw::DemoScene scene;
        scene.create();
        std::vector<vw::SceneMessage> messages;
        if (s.assets && !s.sceneFile) scene.loadAssets(TERRA_ASSETS_DIR, messages);
        // M2-07：场景文件 → 物理世界推进固定步数 → 视觉部件位姿 → 渲染项
        std::optional<terra::scene::SceneInstance> simulated;
        std::unique_ptr<vw::SceneView> sceneView;
        std::vector<r::RenderItem> sceneItems;
        if (s.sceneFile) {
            const std::string scenePath = s.scenePath.empty() ? "tests/scene/two_carts.terra-scene.json" : s.scenePath;
            auto loaded = terra::scene::loadScene(std::string{TERRA_ASSETS_DIR} + "/" + scenePath, terra::physics::mujoco::composeWorld);
            for (const auto& i : loaded.issues) check(false, "场景：" + i.where + " " + i.message);
            if (loaded.ok()) {
                simulated = std::move(*loaded.scene);
                if (s.name == "carts") {
                    const auto& d = simulated->world->description();
                    const auto* driveA = d.findActuator("cart_a/drive");
                    const auto* driveB = d.findActuator("cart_b/drive");
                    check(driveA != nullptr && driveB != nullptr, "执行器 cart_a/drive、cart_b/drive");
                    if (driveA != nullptr && driveB != nullptr) {
                        (void)simulated->world->setControl({{driveA->id, {3.0}}, {driveB->id, {-2.0}}});
                    }
                    for (int k = 0; k < 300; ++k) check(!simulated->world->step(), "物理推进");
                }
                // 其他场景文件：不推进，渲染打开时的姿态（机器人处于场景的初始关键帧）
                sceneView = vw::SceneView::create(*simulated, messages);
                const auto poses = terra::scene::visualPoses(*simulated, terra::scene::entityPoses(*simulated->world->snapshot()));
                sceneItems = sceneView->items(*simulated, poses);
                if (s.name == "carts") check(sceneView->liveBindings() == 4, "4 个 glTF 绑定全部可用（节点 Chassis、Mast 存在）");
                for (const auto& m : messages) std::printf("INFO 场景提示：%s\n", m.text.c_str());
                std::printf("INFO 场景：%zu 个渲染项，%zu 个网格，仿真时间 %.3f 秒\n", sceneItems.size(), sceneView->meshCount(),
                            simulated->world->simulationTime());
            }
        }
        st::RenderSettings rs;  // 默认画质：AgX、4× MSAA、阴影、网格
        rs.environment = s.environment;
        terra::view::Camera camera;
        camera.position = {s.eye.x, s.eye.y, s.eye.z};
        camera.lookAt({s.target.x, s.target.y, s.target.z});
        r::ViewParams view;
        view.view = camera.viewMatrix();
        view.projection = camera.projectionMatrix(static_cast<double>(kWidth) / static_cast<double>(kHeight));
        view.cameraPosition = s.eye;
        const vw::FrameLighting lit = vw::frameLighting(rs, environments, ctx, messages);
        for (const auto& m : messages) {
            if (m.kind == vw::SceneMessage::Kind::AssetError || m.kind == vw::SceneMessage::Kind::EnvironmentFailed) {
                check(false, "场景准备：" + m.text);
            }
        }
        if (s.assets && !s.sceneFile) check(scene.assets().size() == 3, "三个 glTF 资产已加载");
        if (s.environment != st::EnvironmentSetting::Hemisphere) check(lit.image.environment != nullptr, "环境贴图已加载");
        for (r::ImageRgba8* out : {&first, &second}) {
            made.renderer->render(kWidth, kHeight, view, s.sceneFile ? sceneItems : scene.items(rs.assetArmOcclusion), lit.sun,
                                  lit.hemisphere, lit.settings, lit.image);
            *out = r::fromBottomUp(kWidth, kHeight, made.renderer->readOutput(0, 0, kWidth, kHeight));
        }
        std::printf("INFO 阴影投射物 %d 个，MSAA %d×，环境贴图光照 %s\n", made.renderer->lastFrame().shadowCasters,
                    made.renderer->lastFrame().samples, made.renderer->lastFrame().imageLighting ? "是" : "否");
    }
    check(!first.empty() && first.pixels == second.pixels, "同一场景渲染两次逐字节相同（确定性）");

    std::error_code ec;
    fs::create_directories(outputDir, ec);
    const fs::path referencePath = referenceDir / (s.name + ".png");
    if (update) {
        fs::create_directories(referenceDir, ec);
        if (const auto error = r::savePng(referencePath.string(), first)) {
            check(false, *error);
        } else {
            std::printf("INFO 已写入参考图 %s\n", referencePath.string().c_str());
        }
    } else {
        const auto reference = r::loadImage(referencePath.string());
        if (!reference.ok()) {
            check(false, "参考图不存在或无法读取：" + referencePath.string() + "（在基线机器上运行 scripts/verify/update_references.sh 生成并提交）");
        } else {
            const r::CompareResult result = r::compareImages(first, reference.image);
            std::printf("INFO 与参考图比较：%s\n", result.summary().c_str());
            check(result.passed, "与参考图一致（" + result.summary() + "）");
            if (!result.passed) {
                const fs::path actual = outputDir / (s.name + "_actual.png");
                const fs::path diff = outputDir / (s.name + "_diff.png");
                (void)r::savePng(actual.string(), first);
                (void)r::savePng(diff.string(), r::differenceImage(first, reference.image));
                std::printf("INFO 实际图 %s，差异图 %s\n", actual.string().c_str(), diff.string().c_str());
            }
        }
    }
    for (const auto& m : ctx.takeDebugMessages()) std::printf("INFO GL 消息：%s\n", m.text.c_str());
    check(ctx.errorMessageCount() == 0, "没有 GL 错误");
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
