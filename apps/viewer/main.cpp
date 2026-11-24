// terra_viewer：EAI Terra 的第一个应用——编辑器式界面外壳（M1-13，D-024、D-025、D-028、D-029）。
//
// 布局参考 Blender / UE5：顶部菜单栏；中间视口（场景渲染到离屏帧缓冲后作为图像显示）；左侧“大纲”、右侧“属性”、
// 底部“日志 / 统计”面板，均可拖动停靠；最下方状态栏。F11 全屏；菜单“设置 → 语言”切换中英文。
// 输入：平台 → UiInputBridge（分流）→ InputSystem 与 ImGui；鼠标在面板上时界面占用输入，在视口里时相机控制器工作。
// 设置（M1-14，D-026）：启动时读取 settings.json（默认 ~/.config/eai-terra/），“设置 → 偏好设置…”中修改立即生效，
// 改动 1 秒后自动保存，退出时再保存一次（含窗口大小与停靠布局）。
//
// 用法：terra_viewer [--seconds N] [--frames N] [--language zh|en] [--settings 文件] [--self-test] [--expect-renderer 子串]
//   --settings   设置文件路径（默认按 XDG 规则）
//   --language   本次使用的界面语言（会写入设置）
//   --self-test  从默认设置开始（不读取设置文件），运行 --frames 帧后检查：字体未回退、中文字形可用、无 GL 错误、
//                视口与窗口像素、语言切换、设置换算与生效；给出 --settings 时另把设置保存到该文件再读回比较。失败返回 1
//   --scene      （M2-07）打开场景文件（JSON）：组合物理世界、运行仿真，视口显示由快照驱动的场景（代替演示场景）；
//                场景有错误时打印诊断并返回 4。场景文件中的 view.camera 作为初始相机
//   --demo-drive （M2-07）演示：给场景中的所有执行器施加正弦控制（幅度为控制范围的 60%），用于观看运动
//   --motion     （M3-04c）打开动作文件并从头播放；没有 --scene 时打开动作文件中的推荐场景。动作有错误时返回 4。
//                自检时在第 3 帧打开（代替按播放），并与“动作 → PD 层 → setControl → step”的纯物理对照逐位比较

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iterator>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "imgui.h"
#include "imgui_internal.h"
#include "terra/asset/gltf.hpp"
#include "terra/asset/gpu_asset.hpp"
#include "terra/core/fatal.hpp"
#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/scene/scene.hpp"
#include "terra/simulation/motion.hpp"
#include "terra/simulation/pd_layer.hpp"
#include "terra/simulation/runner.hpp"
#include "terra/core/system_monitor.hpp"
#include "terra/core/time_series.hpp"
#include "terra/input/input_system.hpp"
#include "terra/platform/glfw/glfw_platform.hpp"
#include "terra/render/context.hpp"
#include "terra/render/environment.hpp"
#include "terra/render/hdr_image.hpp"
#include "terra/render/mesh.hpp"
#include "terra/render/pbr.hpp"
#include "terra/render/resources.hpp"
#include "terra/render/scene_renderer.hpp"
#include "terra/settings/settings.hpp"
#include "terra/ui/file_browser.hpp"
#include "terra/ui/text.hpp"
#include "terra/ui/ui_context.hpp"
#include "terra/ui/ui_input.hpp"
#include "terra/ui/widgets.hpp"
#include "terra/view/camera.hpp"
#include "terra/view/camera_controller.hpp"
#include "scene_view.hpp"
#include "viewer_settings.hpp"

#ifndef TERRA_ASSETS_DIR
#error "需要 TERRA_ASSETS_DIR"
#endif

namespace {

namespace in = terra::input;
namespace r = terra::render;
namespace st = terra::settings;
namespace vw = terra::viewer;
namespace ui = terra::ui;
namespace v = terra::view;
using ui::TextId;
using ui::tr;

struct Options {
    double seconds = 0.0;  // 0：不限时
    int frames = 0;        // 0：不限帧
    bool selfTest = false;
    std::string expectRenderer;
    std::string settingsPath;
    std::optional<ui::Language> language;
    std::string scenePath;
    std::string motionPath;  // M3-04c：动作文件（需要同时给出场景，或动作文件自带推荐场景）
    bool demoDrive = false;
};

bool parseOptions(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const bool hasValue = i + 1 < argc;
        if (a == "--seconds" && hasValue) {
            o.seconds = std::atof(argv[++i]);
        } else if (a == "--frames" && hasValue) {
            o.frames = std::atoi(argv[++i]);
        } else if (a == "--language" && hasValue) {
            const std::string lang = argv[++i];
            if (lang == "zh") o.language = ui::Language::Chinese;
            else if (lang == "en") o.language = ui::Language::English;
            else return false;
        } else if (a == "--settings" && hasValue) {
            o.settingsPath = argv[++i];
        } else if (a == "--self-test") {
            o.selfTest = true;
        } else if (a == "--scene" && hasValue) {
            o.scenePath = argv[++i];
        } else if (a == "--motion" && hasValue) {
            o.motionPath = argv[++i];
        } else if (a == "--demo-drive") {
            o.demoDrive = true;
        } else if (a == "--expect-renderer" && hasValue) {
            o.expectRenderer = argv[++i];
        } else {
            return false;
        }
    }
    return true;
}

using vw::DemoScene;
using vw::EnvironmentLibrary;

r::ViewParams viewParams(const v::Camera& camera, double aspect) {
    r::ViewParams p;
    p.view = camera.viewMatrix();
    p.projection = camera.projectionMatrix(aspect);
    p.cameraPosition = {static_cast<float>(camera.position.x), static_cast<float>(camera.position.y),
                        static_cast<float>(camera.position.z)};
    return p;
}

// ---- 应用状态 ----

// ---- 视口导航坐标系（仿 Blender，用户 2026-11-19 要求）----
// 视口左下角：X 红、Y 绿、Z 蓝，正方向为带字母的实心圆，负方向为半透明圆；随相机转动；点击圆点把视角对齐到该轴。

constexpr double kAxisDirs[6][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {-1, 0, 0}, {0, -1, 0}, {0, 0, -1}};
constexpr unsigned kAxisColors[3][3] = {{230, 64, 64}, {115, 204, 77}, {77, 128, 242}};  // 与场景坐标轴同色
constexpr const char* kAxisNames[3] = {"X", "Y", "Z"};

// 与 ImGui 颜色宏相同的打包方式（R 在低位），避免在本文件展开含 C 风格转换的宏。
constexpr ImU32 color32(unsigned r, unsigned g, unsigned b, unsigned a) {
    return static_cast<ImU32>((a << 24) | (b << 16) | (g << 8) | r);
}

double dot3(const double a[3], const terra::data::Vec3& b) { return a[0] * b.x + a[1] * b.y + a[2] * b.z; }

// 把视角对齐到轴 axis（0..5：+X +Y +Z -X -Y -Z）：从该轴正方向一侧看向前方 5 m 处的点。
void snapView(v::Camera& camera, int axis) {
    const double d = 5.0;
    const auto f = camera.forward();
    const terra::data::Vec3 target{camera.position.x + f.x * d, camera.position.y + f.y * d, camera.position.z + f.z * d};
    const double* a = kAxisDirs[axis];
    camera.position = {target.x + a[0] * d, target.y + a[1] * d, target.z + a[2] * d};
    camera.lookAt(target);
}

struct GizmoLayout {
    ImVec2 center{0, 0};
    ImVec2 bubble[6];
    float radius = 0.0f;
    float bubbleRadius = 0.0f;
};

// 绘制在当前窗口（视口）的绘制列表上；返回被点击的轴，没有点击返回 -1。
int navigationGizmo(const v::Camera& camera, ImVec2 imageMin, ImVec2 imageMax, float scale, bool hovered,
                    GizmoLayout& layout) {
    const float R = 46.0f * scale;
    const float r = 10.0f * scale;
    const float margin = 14.0f * scale;
    if (imageMax.x - imageMin.x < 2.0f * (R + r + margin) || imageMax.y - imageMin.y < 2.0f * (R + r + margin)) return -1;
    const ImVec2 c(imageMin.x + margin + R + r, imageMax.y - margin - R - r);
    layout.center = c;
    layout.radius = R;
    layout.bubbleRadius = r;
    const auto right = camera.right();
    const auto up = camera.up();
    const auto forward = camera.forward();
    int order[6];
    double depth[6];
    for (int i = 0; i < 6; ++i) {
        const double* a = kAxisDirs[i];
        layout.bubble[i] = ImVec2(c.x + static_cast<float>(dot3(a, right)) * R, c.y - static_cast<float>(dot3(a, up)) * R);
        depth[i] = dot3(a, forward);  // 越大越朝屏幕里，先画
        order[i] = i;
    }
    std::sort(order, order + 6, [&](int x, int y) { return depth[x] > depth[y]; });

    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 mouse = ImGui::GetIO().MousePos;
    const auto dist2 = [](ImVec2 p, ImVec2 q) { return (p.x - q.x) * (p.x - q.x) + (p.y - q.y) * (p.y - q.y); };
    if (hovered && dist2(mouse, c) < (R + r) * (R + r)) dl->AddCircleFilled(c, R + r, color32(255, 255, 255, 28));
    int clicked = -1;
    ImFont* font = ImGui::GetFont();
    const float fontSize = ImGui::GetFontSize() * 0.8f;
    for (const int i : order) {
        const unsigned* rgb = kAxisColors[i % 3];
        const ImU32 solid = color32(rgb[0], rgb[1], rgb[2], 255);
        const ImVec2 p = layout.bubble[i];
        const bool over = hovered && dist2(mouse, p) < r * r;
        if (i < 3) {
            dl->AddLine(c, p, solid, 2.5f * scale);
            dl->AddCircleFilled(p, r, solid);
            const ImVec2 ts = font->CalcTextSizeA(fontSize, FLT_MAX, 0.0f, kAxisNames[i]);
            dl->AddText(font, fontSize, ImVec2(p.x - ts.x * 0.5f, p.y - ts.y * 0.5f), color32(20, 20, 22, 255), kAxisNames[i]);
        } else {
            dl->AddCircleFilled(p, r * 0.85f, color32(rgb[0], rgb[1], rgb[2], 90));
            dl->AddCircle(p, r * 0.85f, color32(rgb[0], rgb[1], rgb[2], 200), 0, 1.5f * scale);
        }
        if (over) {
            dl->AddCircle(p, r + 1.5f * scale, color32(255, 255, 255, 230), 0, 2.0f * scale);
            if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) clicked = i;
        }
    }
    return clicked;
}

// 性能监视（M1-15）：每帧记录帧时间与 GPU 耗时；每 0.25 秒采样 CPU、显存与进程内存。
struct PerfState {
    terra::core::TimeSeries frameMs{600};
    terra::core::TimeSeries gpuMs{600};
    terra::core::TimeSeries processCpu{240};
    terra::core::TimeSeries systemCpu{240};
    terra::core::TimeSeries vramUsedMb{240};
    terra::core::TimeSeries residentMb{240};
    terra::core::SystemMonitor monitor;
    terra::core::SystemMonitor::Sample sample;
    std::optional<r::GpuMemory> vram;
    double nextSample = 0.0;

    void record(double now, double frameMilliseconds, const r::PassTimings& timings, const r::RenderContext& ctx) {
        frameMs.push(frameMilliseconds);
        if (timings.valid) gpuMs.push(timings.total());
        if (now < nextSample) return;
        nextSample = now + 0.25;
        sample = monitor.sample();
        if (sample.processCpuPercent) processCpu.push(*sample.processCpuPercent);
        if (sample.systemCpuPercent) systemCpu.push(*sample.systemCpuPercent);
        if (sample.residentMb) residentMb.push(*sample.residentMb);
        vram = ctx.gpuMemory();
        if (vram && vram->totalMb) vramUsedMb.push(*vram->totalMb - vram->availableMb);
    }
};

struct AppState {
    bool showControls = false;
    bool showAbout = false;
    bool layoutBuilt = false;
    bool quit = false;
    bool viewportHovered = false;
    ImVec2 viewportImageMin{0, 0};  // 视口图像在窗口中的位置（屏幕坐标），供自检
    ImVec2 viewportImageMax{0, 0};
    GizmoLayout gizmo;
    bool imageLightingSeen = false;
    std::array<double, r::kRenderPassCount> gpuMs{};  // 统计面板中平滑后的 GPU 分阶段耗时
    PerfState perf;  // 自检：某一帧使用了环境贴图光照并画出了背景
    float autoUiScale = 1.0f;
    vw::SettingsWindow settingsWindow;
    std::vector<terra::platform::MonitorInfo> monitors;
    std::vector<std::string> log;
    double fps = 0.0;
    double frameMs = 0.0;
    // M2-08：场景文件与仿真界面
    ui::FileBrowser fileBrowser;
    terra::data::EntityId selectedEntity;    // 大纲中选中的实体（属于当前场景）
    std::vector<std::string> sceneErrors;    // 最近一次打开失败的诊断（弹窗显示）
    bool showSceneErrors = false;
    std::optional<std::string> openRequest;  // 菜单或文件窗口请求打开的场景（在帧末处理，避免在绘制界面时替换 GPU 资源）
    // M3-04c：动作文件与执行器面板
    ui::FileBrowser motionBrowser;
    std::optional<std::string> motionRequest;  // 请求打开的动作文件（帧末处理：可能需要先打开推荐场景）
    bool showActuators = true;
    std::string errorTitle;  // 错误弹窗的标题（场景或动作）
    bool closeRequest = false;
    bool demoAssetsLoaded = false;
    std::optional<st::EnvironmentSetting> sceneEnvironment;  // 场景文件 view.environment：只影响本次会话，不写入设置
};

const char* displayModeText(terra::platform::DisplayMode mode) {
    switch (mode) {
        case terra::platform::DisplayMode::Windowed: return tr(TextId::DisplayWindowed);
        case terra::platform::DisplayMode::BorderlessFullscreen: return tr(TextId::DisplayBorderless);
        case terra::platform::DisplayMode::ExclusiveFullscreen: return tr(TextId::DisplayExclusive);
    }
    return "";
}

// 面板标题随语言变化，但 “###” 之后的 ID 不变，停靠位置因此保持。
std::string label(TextId id, const char* stableId) { return std::string{tr(id)} + "###" + stableId; }

void buildDefaultLayout(ImGuiID dockspace) {
    ImGui::DockBuilderRemoveNode(dockspace);
    ImGui::DockBuilderAddNode(dockspace, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockspace, ImGui::GetMainViewport()->WorkSize);
    ImGuiID center = dockspace;
    const ImGuiID left = ImGui::DockBuilderSplitNode(center, ImGuiDir_Left, 0.17f, nullptr, &center);
    const ImGuiID right = ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, 0.24f, nullptr, &center);
    const ImGuiID bottom = ImGui::DockBuilderSplitNode(center, ImGuiDir_Down, 0.24f, nullptr, &center);
    ImGui::DockBuilderDockWindow("###Outliner", left);
    ImGui::DockBuilderDockWindow("###Details", right);
    ImGui::DockBuilderDockWindow("###Actuators", right);
    ImGui::DockBuilderDockWindow("###Log", bottom);
    ImGui::DockBuilderDockWindow("###Stats", bottom);
    ImGui::DockBuilderDockWindow("###Viewport", center);
    ImGui::DockBuilderFinish(dockspace);
}

// 设置的三份状态：界面直接修改 current；applySettings 把与 applied 的差异生效；与 saved 不同时延迟保存。
struct SettingsRuntime {
    st::Settings current;
    st::Settings applied;
    std::optional<st::Settings> saved;  // 文件中的内容；空 = 需要写入（首次运行、文件有问题时）
    std::string path;                   // 空 = 本次运行不保存
    bool autosave = true;               // 自检时关闭（自检结束时单独检查保存）
    double changedAt = -1.0;            // current 开始与 saved 不同的时刻
    bool saveFailureReported = false;
};

// 由场景文件创建的仿真场景（M2-07）：场景实例、仿真推进与渲染适配。GPU 资源在 OpenGL 上下文之前销毁。
struct SimulatedScene {
    terra::scene::SceneInstance instance;
    std::unique_ptr<terra::simulation::SimulationRunner> runner;
    terra::simulation::PdLayer pd;  // M3-04b：场景文件 controllers 中的 PD 层（为空表示没有）
    // M3-04c：动作播放与执行器面板
    std::optional<terra::simulation::MotionPlayer> motion;
    std::string motionPath;
    std::map<std::uint32_t, double> panelTargets;  // 面板设置过、但还没有出现在快照中的目标（暂停时也能立即显示）
    std::vector<terra::simulation::MotionCatalogEntry> motionCatalog;  // assets/motions/ 中推荐场景为本场景的动作
    std::unique_ptr<vw::SceneView> view;
    bool demoDrive = false;
    std::string path;               // 场景文件（绝对路径）
    double realtimeFactor = 0.0;    // 仿真时间 / 墙钟时间（平滑）
    int speedIndex = 2;             // kSimSpeeds 下标
    // M3-03：显示哪些 MJCF 几何体分组（默认 0–2，Menagerie 的可见几何在 2、碰撞几何在 3），是否叠加碰撞几何体
    std::array<bool, 6> groups{true, true, true, false, false, false};
    bool collision = false;

    static constexpr double kSimSpeeds[] = {0.25, 0.5, 1.0, 2.0, 4.0};

    void setSpeed(int index) {
        speedIndex = std::clamp(index, 0, static_cast<int>(sizeof(kSimSpeeds) / sizeof(kSimSpeeds[0])) - 1);
        runner->setSpeed(kSimSpeeds[speedIndex]);
    }
    void togglePlay() {
        if (runner->paused()) runner->resume();
        else runner->pause();
    }

    // 演示控制：所有执行器正弦驱动，相位错开；在下一步之前生效
    void drive() {
        if (!demoDrive) return;
        const auto& d = instance.world->description();
        const double t = instance.world->simulationTime();
        std::vector<terra::physics::ActuatorCommand> commands;
        for (std::size_t k = 0; k < d.actuators.size(); ++k) {
            const auto& a = d.actuators[k];
            terra::physics::ActuatorCommand c{a.id, {}};
            for (const auto& input : a.inputs) {
                const double amplitude = input.limited ? 0.6 * std::min(std::fabs(input.min), std::fabs(input.max)) : 1.0;
                c.values.push_back(amplitude * std::sin(2.0 * 3.14159265358979323846 * 0.25 * t + 1.3 * static_cast<double>(k)));
            }
            commands.push_back(std::move(c));
        }
        if (!commands.empty()) runner->scheduleNext(std::move(commands));
    }

    // 每步控制器：动作给出的命令（播放中）→ PD 层（有时）。查看器与 Headless、自检中的纯物理对照走同一条路径。
    std::vector<terra::physics::ActuatorCommand> control(std::uint64_t step) {
        std::vector<terra::physics::ActuatorCommand> commands;
        if (motion && motion->playing()) commands = motion->commands(step, instance.world->description().timestep);
        if (pd.empty()) return commands;
        return pd.process(std::move(commands), *instance.world->snapshot());
    }

    std::vector<r::RenderItem> items(const st::RenderSettings& rs, terra::data::EntityId highlight) const {
        vw::SceneViewOptions options;
        options.armOcclusion = rs.assetArmOcclusion;
        options.highlight = highlight;
        options.groups = groups;
        options.collision = collision;
        return view->items(instance, terra::scene::visualPoses(instance, runner->displayPoses()), options);
    }
};

struct Frame {
    terra::platform::Platform& platform;
    terra::platform::Window& window;
    in::InputSystem& input;
    ui::UiContext& uiCtx;
    r::RenderContext& ctx;
    v::Camera& camera;
    v::CameraController& controller;
    DemoScene& scene;
    r::SceneRenderer& renderer;
    EnvironmentLibrary& environments;
    AppState& state;
    SettingsRuntime& settings;
    std::optional<SimulatedScene>& simulated;  // 未打开场景时为空，视口显示演示场景
};

void appendMessages(AppState& state, const std::vector<vw::SceneMessage>& messages);

// ---- 场景文件与仿真（M2-08）----

constexpr const char* kSceneSuffix = ".terra-scene.json";

std::optional<st::EnvironmentSetting> environmentFromName(const std::string& name) {
    if (name == "outdoor") return st::EnvironmentSetting::Outdoor;
    if (name == "studio") return st::EnvironmentSetting::Studio;
    if (name == "hemisphere") return st::EnvironmentSetting::Hemisphere;
    return std::nullopt;
}

std::string issueText(const terra::scene::SceneIssue& issue) {
    std::string text = std::string{tr(issue.severity == terra::scene::SceneIssue::Severity::Error ? TextId::IssueError
                                                                                                  : TextId::IssueWarning)} + " ";
    if (!issue.where.empty()) text += issue.where + "：";
    if (!issue.instance.empty()) text += "[" + issue.instance + "] ";
    return text + issue.message;
}

// 演示场景的 glTF 资产只在需要显示演示场景时加载（打开场景文件时不加载，节省时间与显存）
void ensureDemoAssets(Frame& f) {
    if (f.state.demoAssetsLoaded) return;
    std::vector<vw::SceneMessage> messages;
    f.scene.loadAssets(TERRA_ASSETS_DIR, messages);
    appendMessages(f.state, messages);
    f.state.demoAssetsLoaded = true;
}

void closeScene(Frame& f) {
    if (!f.simulated) return;
    f.simulated.reset();
    f.state.selectedEntity = {};
    f.state.sceneEnvironment.reset();
    terra::core::setFatalContext("场景文件", "");
    f.state.log.push_back(tr(TextId::LogSceneClosed));
    ensureDemoAssets(f);
}

// 打开场景：成功时替换当前场景（打开后暂停，按播放开始）；失败时保留当前场景，弹窗列出诊断。
bool openScene(Frame& f, const std::string& requested) {
    std::error_code ec;
    const std::string path = std::filesystem::absolute(requested, ec).lexically_normal().string();
    auto result = terra::scene::loadScene(path, terra::physics::mujoco::composeWorld);
    std::vector<std::string> lines;
    for (const auto& issue : result.issues) lines.push_back(issueText(issue));
    for (const auto& line : lines) {
        std::printf("INFO 场景诊断：%s\n", line.c_str());
        f.state.log.push_back(line);
    }
    if (!result.ok()) {
        f.state.log.push_back(std::string{tr(TextId::LogSceneFailed)} + path);
        f.state.sceneErrors = lines;
        if (f.state.sceneErrors.empty()) f.state.sceneErrors.push_back(path);
        f.state.errorTitle = "scene";
        f.state.showSceneErrors = true;
        return false;
    }
    f.simulated.reset();  // 先释放旧场景的 GPU 资源
    f.simulated.emplace();
    SimulatedScene& sim = *f.simulated;
    sim.instance = std::move(*result.scene);
    sim.path = path;
    sim.runner = std::make_unique<terra::simulation::SimulationRunner>(*sim.instance.world);
    sim.runner->setHomeKeyframe(sim.instance.homeKeyframe);
    sim.pd = terra::simulation::PdLayer(sim.instance.world->description(), sim.instance.pd);
    sim.motionCatalog = terra::simulation::findMotionsForScene({std::string{TERRA_ASSETS_DIR} + "/motions"}, path);
    if (!sim.pd.empty()) std::printf("INFO 场景：PD 层接管 %zu 个执行器\n", sim.pd.size());
    if (!sim.motionCatalog.empty()) std::printf("INFO 场景：%zu 个预定义动作（菜单 动作 → 本场景的预定义动作）\n", sim.motionCatalog.size());
    {
        // 每步控制器：动作与 PD 层（没有时返回空，等同于没有控制器）
        SimulatedScene* self = &sim;
        sim.runner->setController([self](std::uint64_t step) { return self->control(step); });
    }
    sim.runner->pause();
    std::vector<vw::SceneMessage> messages;
    sim.view = vw::SceneView::create(sim.instance, messages);
    for (const auto& m : messages) std::printf("INFO 场景：%s\n", m.text.c_str());
    appendMessages(f.state, messages);

    const auto& view = sim.instance.document.view;
    if (view.cameraPosition) {
        f.camera.position = *view.cameraPosition;
        if (view.cameraTarget) f.camera.lookAt(*view.cameraTarget);
    }
    f.state.sceneEnvironment.reset();
    if (view.environment) {
        f.state.sceneEnvironment = environmentFromName(*view.environment);
        if (!f.state.sceneEnvironment) f.state.log.push_back(std::string{tr(TextId::SceneUnknownEnvironment)} + *view.environment);
    }
    f.state.selectedEntity = {};
    st::rememberScene(f.settings.current.ui, path);
    terra::core::setFatalContext("场景文件", path);
    f.state.log.push_back(std::string{tr(TextId::LogSceneOpened)} + path);
    const auto& d = sim.instance.world->description();
    std::printf("INFO 场景 %s：%zu 个模型实例，%zu 个实体，%zu 个几何体，%zu 个 glTF 绑定（可用 %zu），步长 %.4g 秒\n",
                sim.instance.document.name.c_str(), d.modelInstances.size(), d.entities.size(), d.geoms.size(),
                sim.instance.bindings.size(), sim.view->liveBindings(), d.timestep);
    return true;
}

void simReset(Frame& f) {
    if (!f.simulated) return;
    f.simulated->runner->reset();
    f.simulated->pd.clearTargets();
    f.simulated->panelTargets.clear();
    if (f.simulated->motion) f.simulated->motion->stop();
    f.simulated->realtimeFactor = 0.0;
}

void simResetToKeyframe(Frame& f, std::size_t index) {
    if (!f.simulated || !f.simulated->runner->resetToKeyframe(index)) return;
    f.simulated->pd.clearTargets();
    f.simulated->panelTargets.clear();
    if (f.simulated->motion) f.simulated->motion->stop();
    f.simulated->realtimeFactor = 0.0;
    f.state.log.push_back(std::string{tr(TextId::LogKeyframeReset)} + f.simulated->instance.world->description().keyframes[index].name);
}

// ---- 动作（M3-04c）----

constexpr const char* kMotionSuffix = ".terra-motion.json";

std::string motionIssueText(const terra::simulation::MotionIssue& issue) {
    std::string text = std::string{tr(issue.severity == terra::simulation::MotionIssue::Severity::Error ? TextId::IssueError
                                                                                                        : TextId::IssueWarning)} + " ";
    if (!issue.where.empty()) text += issue.where + "：";
    return text + issue.message;
}

// 从头播放：按起始关键帧重置（没有时从当前状态开始），清除 PD 与面板目标，从当前步开始并运行
void startMotion(Frame& f) {
    if (!f.simulated || !f.simulated->motion) return;
    SimulatedScene& sim = *f.simulated;
    if (const auto key = sim.motion->motion().startKeyframe) sim.runner->resetToKeyframe(*key);
    sim.pd.clearTargets();
    sim.panelTargets.clear();
    sim.motion->start(sim.instance.world->stepIndex());
    sim.realtimeFactor = 0.0;
    sim.runner->resume();
}

// 打开动作文件：先解析；推荐场景与当前场景不同时切换到推荐场景；再按当前场景绑定（PD 层接管的执行器按关节范围校验）。
// 成功时从头播放；失败时保留当前状态，弹窗列出诊断。
bool openMotion(Frame& f, const std::string& requested) {
    std::error_code ec;
    const std::string path = std::filesystem::absolute(requested, ec).lexically_normal().string();
    std::vector<std::string> lines;
    const auto fail = [&](const std::string& extra) {
        if (!extra.empty()) lines.push_back(extra);
        for (const auto& line : lines) std::printf("INFO 动作诊断：%s\n", line.c_str());
        f.state.log.push_back(std::string{tr(TextId::LogMotionFailed)} + path);
        f.state.sceneErrors = lines;
        if (f.state.sceneErrors.empty()) f.state.sceneErrors.push_back(path);
        f.state.errorTitle = "motion";
        f.state.showSceneErrors = true;
        return false;
    };
    const auto parsed = terra::simulation::readMotionFile(path);
    for (const auto& issue : parsed.issues) lines.push_back(motionIssueText(issue));
    if (!parsed.document) return fail({});
    // 动作文件写了推荐场景、且与当前场景不同（或当前没有场景）时，先打开推荐场景；没写时绑定到当前场景
    const std::string& wanted = parsed.document->scene;
    if (!f.simulated && wanted.empty()) return fail(tr(TextId::MotionNeedsScene));
    if (!wanted.empty() && (!f.simulated || f.simulated->path != wanted)) {
        if (!openScene(f, wanted)) return false;
    }
    SimulatedScene& sim = *f.simulated;
    const auto bound = terra::simulation::bindMotion(*parsed.document, sim.instance.world->description(), sim.pd.targetRangeFunction());
    for (const auto& issue : bound.issues) lines.push_back(motionIssueText(issue));
    if (!bound.motion) return fail({});
    for (const auto& line : lines) {
        std::printf("INFO 动作诊断：%s\n", line.c_str());
        f.state.log.push_back(line);
    }
    const bool loop = sim.motion && sim.motion->loop();
    sim.motion = terra::simulation::MotionPlayer(*bound.motion);
    sim.motion->setLoop(loop);
    sim.motionPath = path;
    f.state.log.push_back(std::string{tr(TextId::LogMotionOpened)} + path);
    std::printf("INFO 动作 %s：%zu 条轨道，时长 %.3f 秒\n", bound.motion->name.c_str(), bound.motion->tracks.size(),
                bound.motion->duration);
    startMotion(f);
    return true;
}

const char* shapeText(terra::data::GeomShape shape) {
    using S = terra::data::GeomShape;
    switch (shape) {
        case S::Plane: return tr(TextId::ShapePlane);
        case S::Sphere: return tr(TextId::ShapeSphere);
        case S::Capsule: return tr(TextId::ShapeCapsule);
        case S::Ellipsoid: return tr(TextId::ShapeEllipsoid);
        case S::Cylinder: return tr(TextId::ShapeCylinder);
        case S::Box: return tr(TextId::ShapeBox);
        case S::Mesh: return tr(TextId::ShapeMesh);
        case S::Other: return tr(TextId::ShapeOther);
    }
    return "";
}

// 视口上方的仿真工具栏：播放 / 暂停、单步、重置、速度
void simulationToolbar(Frame& f) {
    if (!f.simulated) return;
    SimulatedScene& sim = *f.simulated;
    const bool paused = sim.runner->paused();
    ImGui::SetCursorPos(ImVec2(ImGui::GetCursorPosX() + 4.0f, ImGui::GetCursorPosY() + 3.0f));
    if (ImGui::Button(paused ? tr(TextId::SimPlay) : tr(TextId::SimPause))) sim.togglePlay();
    ImGui::SameLine();
    ImGui::BeginDisabled(!paused);
    if (ImGui::Button(tr(TextId::SimStep))) sim.runner->stepOnce();
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button(tr(TextId::SimReset))) simReset(f);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ImGui::GetFontSize() * 5.0f);
    char current[16];
    std::snprintf(current, sizeof current, "%g×", SimulatedScene::kSimSpeeds[sim.speedIndex]);
    if (ImGui::BeginCombo((std::string{"##"} + tr(TextId::SimSpeed)).c_str(), current)) {
        for (int i = 0; i < static_cast<int>(sizeof(SimulatedScene::kSimSpeeds) / sizeof(SimulatedScene::kSimSpeeds[0])); ++i) {
            char option[16];
            std::snprintf(option, sizeof option, "%g×", SimulatedScene::kSimSpeeds[i]);
            if (ImGui::Selectable(option, i == sim.speedIndex)) sim.setSpeed(i);
        }
        ImGui::EndCombo();
    }
    ImGui::SameLine();
    const char* status = sim.runner->faulted() ? tr(TextId::StatusFaulted) : paused ? tr(TextId::StatusPaused) : tr(TextId::StatusRunning);
    ImGui::AlignTextToFramePadding();
    ImGui::Text("%s  %.3f s", status, sim.instance.world->simulationTime());
}

void sceneErrorWindow(Frame& f) {
    // 场景与动作共用一个弹窗；标题按最近一次失败的来源
    const std::string title = f.state.errorTitle == "motion" ? label(TextId::MotionErrorTitle, "SceneErrors")
                                                              : label(TextId::SceneErrorTitle, "SceneErrors");
    if (f.state.showSceneErrors) {
        ImGui::OpenPopup(title.c_str());
        f.state.showSceneErrors = false;
    }
    if (ImGui::BeginPopupModal(title.c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
        for (const auto& line : f.state.sceneErrors) ImGui::TextUnformatted(line.c_str());
        ImGui::Spacing();
        if (ImGui::Button(tr(TextId::Close)) || ImGui::IsKeyPressed(ImGuiKey_Escape)) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
}

constexpr const char* kAppMap = "app";

ui::Language toLanguage(const std::string& code) { return code == "en" ? ui::Language::English : ui::Language::Chinese; }

int monitorIndex(const st::DisplaySettings& d, const std::vector<terra::platform::MonitorInfo>& monitors) {
    return d.monitor >= 0 && static_cast<std::size_t>(d.monitor) < monitors.size() ? d.monitor : 0;  // 不存在时用主显示器
}

std::optional<terra::platform::VideoMode> toVideoMode(const std::optional<st::VideoModeSetting>& m) {
    if (!m) return std::nullopt;
    return terra::platform::VideoMode{m->width, m->height, m->refreshRate};
}

// 显示模式由窗口本身改变（F11）时，同步到设置，且不再重复生效。
void syncDisplayModeFromWindow(Frame& f) {
    const st::WindowMode mode = vw::toWindowMode(f.window.displayMode());
    f.settings.current.display.mode = mode;
    f.settings.applied.display.mode = mode;
}

void toggleFullscreen(Frame& f) {
    if (const auto error = f.window.toggleBorderlessFullscreen()) {
        f.state.log.push_back(std::string{tr(TextId::LogDisplayModeFailed)} + error->message);
    } else {
        f.state.log.push_back(std::string{tr(TextId::LogDisplayModeChanged)} + displayModeText(f.window.displayMode()));
    }
    syncDisplayModeFromWindow(f);
}

// 把 current 与 applied 的差异生效（每帧开始时调用）；显示模式切换失败时把 current 改回实际状态。
void applySettings(Frame& f) {
    SettingsRuntime& rt = f.settings;
    st::Settings& next = rt.current;
    const st::Settings& prev = rt.applied;
    if (next == prev) return;
    if (next.ui.language != prev.ui.language) {
        ui::setLanguage(toLanguage(next.ui.language));
        f.window.setTitle(tr(TextId::AppName));
        f.state.log.push_back(tr(TextId::LogLanguageChanged));
    }
    if (next.ui.uiScale != prev.ui.uiScale) {
        f.uiCtx.requestScale(static_cast<float>(next.ui.uiScale.value_or(static_cast<double>(f.state.autoUiScale))));
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.0f%%", static_cast<double>(f.uiCtx.scale()) * 100.0);
        f.state.log.push_back(std::string{tr(TextId::LogUiScale)} + buf);
    }
    if (next.ui.layout.empty() && !prev.ui.layout.empty()) f.state.layoutBuilt = false;  // 恢复默认布局
    const st::DisplaySettings& nd = next.display;
    const st::DisplaySettings& pd = prev.display;
    const bool targetChanged = nd.mode != st::WindowMode::Windowed && (nd.monitor != pd.monitor || nd.videoMode != pd.videoMode);
    if (nd.mode != pd.mode || targetChanged) {
        f.state.monitors = f.platform.monitors();
        const auto error = f.window.setDisplayMode(vw::toDisplayMode(nd.mode), monitorIndex(nd, f.state.monitors),
                                                   nd.mode == st::WindowMode::ExclusiveFullscreen ? toVideoMode(nd.videoMode)
                                                                                                  : std::nullopt);
        if (error) {
            f.state.log.push_back(std::string{tr(TextId::LogDisplayModeFailed)} + error->message);
            next.display.mode = vw::toWindowMode(f.window.displayMode());
            next.display.monitor = pd.monitor;
            next.display.videoMode = pd.videoMode;
        } else {
            f.state.log.push_back(std::string{tr(TextId::LogDisplayModeChanged)} + displayModeText(f.window.displayMode()));
        }
    }
    if (nd.vsync != pd.vsync) f.window.setVsync(nd.vsync);
    if (next.render.environment != prev.render.environment) f.state.sceneEnvironment.reset();  // 手动选择环境后不再沿用场景文件的设置
    if (next.camera != prev.camera) {
        f.controller.settings() = vw::toCameraControl(next.camera);
        f.camera.verticalFov = next.camera.fovDegrees * vw::kRadiansPerDegree;
    }
    if (next.keys != prev.keys) {
        f.controller.setKeys(vw::toCameraKeys(next.keys));
        if (auto error = f.input.setBindings(kAppMap, "fullscreen", vw::buttonBindings(next.keys[st::Command::AppFullscreen]))) {
            f.state.log.push_back(*error);
        }
        using CommandAction = std::pair<st::Command, const char*>;
        for (const auto& [command, action] : {CommandAction{st::Command::SimPlayPause, "sim_play"}, CommandAction{st::Command::SimStep, "sim_step"},
                                              CommandAction{st::Command::SimReset, "sim_reset"}}) {
            if (auto error = f.input.setBindings(kAppMap, action, vw::buttonBindings(next.keys[command]))) f.state.log.push_back(*error);
        }
    }
    rt.applied = next;
}

// current 与 saved 不同且已持续 1 秒（或 force）时保存；失败只报告一次，之后每秒重试。
void persistSettings(Frame& f, double now, bool force) {
    SettingsRuntime& rt = f.settings;
    if (!rt.autosave || rt.path.empty()) return;
    if (rt.saved && *rt.saved == rt.current) {
        rt.changedAt = -1.0;
        return;
    }
    if (rt.changedAt < 0.0) rt.changedAt = now;
    if (!force && now - rt.changedAt < 1.0) return;
    if (const auto error = st::save(rt.current, rt.path)) {
        if (!rt.saveFailureReported) f.state.log.push_back(std::string{tr(TextId::LogSettingsSaveFailed)} + *error);
        rt.saveFailureReported = true;
        rt.changedAt = now;
    } else {
        rt.saved = rt.current;
        rt.changedAt = -1.0;
        rt.saveFailureReported = false;
    }
}

// 界面缩放写入设置（下一帧生效）；取 0.01 的整数倍，避免浮点累加误差写进文件。
void setUiScale(Frame& f, double value) {
    const double clamped = std::clamp(value, st::InterfaceSettings::kMinScale, st::InterfaceSettings::kMaxScale);
    f.settings.current.ui.uiScale = std::round(clamped * 100.0) / 100.0;
}

void openSettingsWindow(Frame& f, bool open) {
    if (open && !f.state.settingsWindow.open) f.state.monitors = f.platform.monitors();
    f.state.settingsWindow.open = open;
}

// Ctrl + 加号 / 减号 / 0：界面放大、缩小、恢复自动；Ctrl + 逗号：偏好设置。输入框获得焦点或正在录制按键时不处理。
void shortcuts(Frame& f) {
    if (ImGui::GetIO().WantTextInput || f.state.settingsWindow.capturingKey()) return;
    const auto chord = [](ImGuiKey key) { return ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | key); };
    const double current = static_cast<double>(f.uiCtx.scale());
    if (chord(ImGuiKey_Equal) || chord(ImGuiKey_KeypadAdd)) setUiScale(f, current + 0.1);
    if (chord(ImGuiKey_Minus) || chord(ImGuiKey_KeypadSubtract)) setUiScale(f, current - 0.1);
    if (chord(ImGuiKey_0) || chord(ImGuiKey_Keypad0)) f.settings.current.ui.uiScale.reset();
    if (chord(ImGuiKey_Comma)) openSettingsWindow(f, !f.state.settingsWindow.open);
    if (chord(ImGuiKey_O)) f.state.fileBrowser.open(f.settings.current.ui.recentScenes.empty() ? std::string{}
                                                                                               : f.settings.current.ui.recentScenes.front(),
                                                    kSceneSuffix);
    if (chord(ImGuiKey_R)) simReset(f);  // 固定快捷键；另可在偏好设置中给“仿真：重置”分配单键
}

void menuBar(Frame& f) {
    if (!ImGui::BeginMainMenuBar()) return;
    if (ImGui::BeginMenu(tr(TextId::MenuFile))) {
        auto& recent = f.settings.current.ui.recentScenes;
        if (ImGui::MenuItem(tr(TextId::MenuFileOpenScene), "Ctrl+O")) {
            f.state.fileBrowser.open(f.simulated ? f.simulated->path : recent.empty() ? std::string{} : recent.front(), kSceneSuffix);
        }
        if (ImGui::BeginMenu(tr(TextId::MenuFileRecent))) {
            if (recent.empty()) ImGui::TextDisabled("%s", tr(TextId::MenuFileRecentEmpty));
            for (const auto& path : recent) {
                if (ImGui::MenuItem(path.c_str())) f.state.openRequest = path;
            }
            ImGui::EndMenu();
        }
        if (ImGui::MenuItem(tr(TextId::MenuFileCloseScene), nullptr, false, f.simulated.has_value())) f.state.closeRequest = true;
        ImGui::Separator();
        if (ImGui::MenuItem(tr(TextId::MenuFileQuit), "Alt+F4")) f.state.quit = true;
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu(tr(TextId::MenuSimulation), f.simulated.has_value())) {
        SimulatedScene& sim = *f.simulated;
        const auto key = [&](st::Command c) { return in::keyName(f.settings.current.keys[c][0]); };
        if (ImGui::MenuItem(sim.runner->paused() ? tr(TextId::SimPlay) : tr(TextId::SimPause), key(st::Command::SimPlayPause))) {
            sim.togglePlay();
        }
        if (ImGui::MenuItem(tr(TextId::SimStep), key(st::Command::SimStep), false, sim.runner->paused())) sim.runner->stepOnce();
        if (ImGui::MenuItem(tr(TextId::SimReset), "Ctrl+R")) simReset(f);
        const auto& keyframes = sim.instance.world->description().keyframes;
        if (ImGui::BeginMenu(tr(TextId::SimResetToKeyframe), !keyframes.empty())) {
            for (std::size_t k = 0; k < keyframes.size(); ++k) {
                const bool home = sim.runner->homeKeyframe() == k;
                const std::string label = (keyframes[k].name.empty() ? "#" + std::to_string(k) : keyframes[k].name) +
                                          (home ? tr(TextId::SimKeyframeInitial) : "") + "##key" + std::to_string(k);
                if (ImGui::MenuItem(label.c_str())) simResetToKeyframe(f, k);
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu(tr(TextId::SimSpeed))) {
            for (int i = 0; i < static_cast<int>(sizeof(SimulatedScene::kSimSpeeds) / sizeof(SimulatedScene::kSimSpeeds[0])); ++i) {
                char option[16];
                std::snprintf(option, sizeof option, "%g×", SimulatedScene::kSimSpeeds[i]);
                if (ImGui::MenuItem(option, nullptr, i == sim.speedIndex)) sim.setSpeed(i);
            }
            ImGui::EndMenu();
        }
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu(tr(TextId::MenuMotion))) {
        if (ImGui::MenuItem(tr(TextId::MenuMotionOpen))) {
            const std::string start = f.simulated && !f.simulated->motionPath.empty() ? f.simulated->motionPath
                                      : f.simulated                                    ? f.simulated->path
                                                                                       : std::string{};
            f.state.motionBrowser.open(start, kMotionSuffix);
        }
        const bool hasMotion = f.simulated && f.simulated->motion;
        if (ImGui::MenuItem(tr(TextId::MenuMotionPlay), nullptr, false, hasMotion)) startMotion(f);
        if (ImGui::MenuItem(tr(TextId::MenuMotionStop), nullptr, false, hasMotion && f.simulated->motion->playing())) {
            f.simulated->motion->stop();
        }
        bool loop = hasMotion && f.simulated->motion->loop();
        if (ImGui::MenuItem(tr(TextId::MenuMotionLoop), nullptr, &loop, hasMotion)) f.simulated->motion->setLoop(loop);
        // 本场景的预定义动作（assets/motions/ 中推荐场景为当前场景的动作文件）
        ImGui::SeparatorText(tr(TextId::MenuMotionForScene));
        if (!f.simulated || f.simulated->motionCatalog.empty()) ImGui::TextDisabled("%s", tr(TextId::MenuMotionNone));
        if (f.simulated) {
            for (const auto& entry : f.simulated->motionCatalog) {
                const bool current = entry.path == f.simulated->motionPath;
                if (ImGui::MenuItem((entry.name + "###" + entry.path).c_str(), nullptr, current)) f.state.motionRequest = entry.path;
            }
        }
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu(tr(TextId::MenuView))) {
        const bool full = f.window.displayMode() != terra::platform::DisplayMode::Windowed;
        if (ImGui::MenuItem(tr(TextId::MenuViewFullscreen), in::keyName(f.settings.current.keys[st::Command::AppFullscreen][0]),
                            full)) {
            toggleFullscreen(f);
        }
        if (ImGui::MenuItem(tr(TextId::MenuViewResetCamera), "F")) v::CameraController::reset(f.camera);
        if (ImGui::BeginMenu(tr(TextId::MenuViewUiScale))) {
            ImGui::TextDisabled("%.0f%%", static_cast<double>(f.uiCtx.scale()) * 100.0);
            const double current = static_cast<double>(f.uiCtx.scale());
            if (ImGui::MenuItem(tr(TextId::MenuViewZoomIn), "Ctrl+=")) setUiScale(f, current + 0.1);
            if (ImGui::MenuItem(tr(TextId::MenuViewZoomOut), "Ctrl+-")) setUiScale(f, current - 0.1);
            if (ImGui::MenuItem(tr(TextId::MenuViewZoomReset), "Ctrl+0")) f.settings.current.ui.uiScale.reset();
            ImGui::EndMenu();
        }
        // M3-03：几何体分组与碰撞几何体（只作用于当前场景，不保存）
        ImGui::Separator();
        if (ImGui::BeginMenu(tr(TextId::MenuViewGeomGroups), f.simulated.has_value())) {
            for (std::size_t g = 0; g < f.simulated->groups.size(); ++g) {
                const std::string label = std::string{tr(TextId::MenuViewGeomGroup)} + std::to_string(g);
                ImGui::MenuItem(label.c_str(), nullptr, &f.simulated->groups[g]);
            }
            ImGui::EndMenu();
        }
        if (f.simulated) ImGui::MenuItem(tr(TextId::MenuViewCollision), nullptr, &f.simulated->collision);
        else ImGui::MenuItem(tr(TextId::MenuViewCollision), nullptr, false, false);
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu(tr(TextId::MenuWindow))) {
        st::InterfaceSettings& panels = f.settings.current.ui;
        ImGui::MenuItem(tr(TextId::PanelOutliner), nullptr, &panels.showOutliner);
        ImGui::MenuItem(tr(TextId::PanelDetails), nullptr, &panels.showDetails);
        ImGui::MenuItem(tr(TextId::PanelLog), nullptr, &panels.showLog);
        ImGui::MenuItem(tr(TextId::PanelStats), nullptr, &panels.showStats);
        ImGui::MenuItem(tr(TextId::PanelActuators), nullptr, &f.state.showActuators);
        ImGui::Separator();
        if (ImGui::MenuItem(tr(TextId::MenuWindowResetLayout))) {
            f.settings.current.ui.layout.clear();
            f.state.layoutBuilt = false;
        }
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu(tr(TextId::MenuSettings))) {
        if (ImGui::MenuItem(tr(TextId::MenuSettingsPreferences), "Ctrl+,", f.state.settingsWindow.open)) {
            openSettingsWindow(f, !f.state.settingsWindow.open);
        }
        if (ImGui::BeginMenu(tr(TextId::MenuSettingsLanguage))) {
            using Choice = std::pair<const char*, ui::Language>;
            for (const auto& [code, lang] : {Choice{"zh", ui::Language::Chinese}, Choice{"en", ui::Language::English}}) {
                if (ImGui::MenuItem(ui::languageName(lang), nullptr, f.settings.current.ui.language == code)) {
                    f.settings.current.ui.language = code;
                }
            }
            ImGui::EndMenu();
        }
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu(tr(TextId::MenuHelp))) {
        ImGui::MenuItem(tr(TextId::MenuHelpControls), nullptr, &f.state.showControls);
        ImGui::MenuItem(tr(TextId::MenuHelpAbout), nullptr, &f.state.showAbout);
        ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
}

void statusBar(Frame& f) {
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_MenuBar;
    if (ImGui::BeginViewportSideBar("##StatusBar", ImGui::GetMainViewport(), ImGuiDir_Down, ImGui::GetFrameHeight(), flags)) {
        if (ImGui::BeginMenuBar()) {
            const auto& p = f.camera.position;
            ImGui::Text("%.0f FPS  |  %s  |  %s (%.2f, %.2f, %.2f)", f.state.fps, displayModeText(f.window.displayMode()),
                        tr(TextId::StatusCamera), p.x, p.y, p.z);
            if (f.simulated) {
                const SimulatedScene& sim = *f.simulated;
                ImGui::SameLine();
                ImGui::Text("  |  %s  %s %.3f s  %s %llu  %s %.2f  %s %llu",
                            sim.runner->faulted() ? tr(TextId::StatusFaulted)
                            : sim.runner->paused() ? tr(TextId::StatusPaused)
                                                   : tr(TextId::StatusRunning),
                            tr(TextId::StatusSimTime), sim.instance.world->simulationTime(), tr(TextId::StatusStep),
                            static_cast<unsigned long long>(sim.instance.world->stepIndex()), tr(TextId::StatusRealtime),
                            sim.realtimeFactor, tr(TextId::StatusOverload),
                            static_cast<unsigned long long>(sim.runner->clock().overloadCount()));
                if (sim.motion) {
                    const auto step = sim.instance.world->stepIndex();
                    const double dt = sim.instance.world->description().timestep;
                    const auto& m = sim.motion->motion();
                    ImGui::SameLine();
                    ImGui::Text("  |  %s %s  %.2f / %.2f s%s", tr(TextId::StatusMotion), m.name.c_str(),
                                sim.motion->playing() ? std::min(sim.motion->timeAt(step, dt), m.duration) : 0.0, m.duration,
                                sim.motion->playing() ? (sim.motion->finished(step, dt) ? tr(TextId::StatusMotionEnded) : "")
                                                      : tr(TextId::StatusMotionStopped));
                }
            }
            ImGui::EndMenuBar();
        }
    }
    ImGui::End();
}

// 场景库返回的消息 → 界面日志（按当前语言）。
void appendMessages(AppState& state, const std::vector<vw::SceneMessage>& messages) {
    for (const auto& m : messages) {
        TextId id = TextId::LogAssetLoaded;
        switch (m.kind) {
            case vw::SceneMessage::Kind::AssetLoaded: id = TextId::LogAssetLoaded; break;
            case vw::SceneMessage::Kind::AssetInfo: id = TextId::LogAssetInfo; break;
            case vw::SceneMessage::Kind::AssetWarning: id = TextId::LogAssetWarning; break;
            case vw::SceneMessage::Kind::AssetError: id = TextId::LogAssetError; break;
            case vw::SceneMessage::Kind::EnvironmentLoaded: id = TextId::LogEnvironmentLoaded; break;
            case vw::SceneMessage::Kind::EnvironmentFailed: id = TextId::LogEnvironmentFailed; break;
        }
        state.log.push_back(std::string{tr(id)} + m.text);
    }
}

void viewportPanel(Frame& f) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    const bool open = ImGui::Begin(label(TextId::PanelViewport, "Viewport").c_str(), nullptr,
                                   ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::PopStyleVar();
    f.state.viewportHovered = false;
    if (open) {
        simulationToolbar(f);
        const ImVec2 avail = ImGui::GetContentRegionAvail();
        const ImVec2 scale = ImGui::GetIO().DisplayFramebufferScale;
        const int w = std::max(1, static_cast<int>(avail.x * scale.x));
        const int h = std::max(1, static_cast<int>(avail.y * scale.y));
        if (avail.x >= 1.0f && avail.y >= 1.0f) {
            st::RenderSettings rs = f.settings.current.render;
            if (f.simulated && f.state.sceneEnvironment) rs.environment = *f.state.sceneEnvironment;  // 场景文件的 view.environment
            std::vector<vw::SceneMessage> messages;
            const vw::FrameLighting lit = vw::frameLighting(rs, f.environments, f.ctx, messages);
            appendMessages(f.state, messages);
            f.renderer.render(w, h, viewParams(f.camera, static_cast<double>(w) / static_cast<double>(h)),
                              f.simulated ? f.simulated->items(rs, f.state.selectedEntity) : f.scene.items(rs.assetArmOcclusion), lit.sun,
                              lit.hemisphere,
                              lit.settings, lit.image);
            if (f.renderer.lastFrame().imageLighting && f.renderer.lastFrame().backgroundDrawn) f.state.imageLightingSeen = true;
            ImGui::Image(static_cast<ImTextureID>(f.renderer.outputTexture()), avail, ImVec2(0, 1), ImVec2(1, 0));
            f.state.viewportHovered = ImGui::IsItemHovered();
            f.state.viewportImageMin = ImGui::GetItemRectMin();
            f.state.viewportImageMax = ImGui::GetItemRectMax();
            const int axis = navigationGizmo(f.camera, f.state.viewportImageMin, f.state.viewportImageMax, f.uiCtx.scale(),
                                             f.state.viewportHovered, f.state.gizmo);
            if (axis >= 0) snapView(f.camera, axis);
        }
    }
    ImGui::End();
}

// 场景大纲（M2-08）：模型实例 → 实体（按父子层级）→ 视觉部件；点选实体后属性面板显示其位姿，视口中高亮
void entityNode(Frame& f, const terra::data::ModelDescription& d, const std::vector<std::vector<std::size_t>>& children,
                const std::vector<std::vector<const terra::scene::VisualPart*>>& partsOf, std::size_t e) {
    const terra::scene::SceneInstance& scene = f.simulated->instance;
    const terra::data::EntityDescription& entity = d.entities[e];
    const std::string name = entity.name.empty() ? std::string{tr(TextId::OutlinerUnnamed)} + " #" + std::to_string(entity.id.value)
                                                 : entity.name;
    ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_SpanAvailWidth;
    if (f.state.selectedEntity == entity.id) flags |= ImGuiTreeNodeFlags_Selected;
    if (children[e].empty() && partsOf[e].empty()) flags |= ImGuiTreeNodeFlags_Leaf;
    const bool open = ImGui::TreeNodeEx(reinterpret_cast<void*>(static_cast<std::uintptr_t>(entity.id.value)), flags, "%s", name.c_str());
    if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen()) f.state.selectedEntity = entity.id;
    if (!open) return;
    for (const terra::scene::VisualPart* part : partsOf[e]) {
        if (part->kind == terra::scene::VisualPart::Kind::Geom) {
            const auto* g = d.geom(part->geom);
            if (g == nullptr) continue;
            ImGui::BulletText("%s %s%s", shapeText(g->shape), g->name.empty() ? tr(TextId::OutlinerUnnamed) : g->name.c_str(),
                              part->hiddenByBinding ? "  -" : "");
        } else {
            const auto& b = scene.bindings[static_cast<std::size_t>(part->binding)];
            ImGui::BulletText("%s %s%s%s", tr(TextId::OutlinerGltf), std::filesystem::path{b.spec.gltf}.filename().string().c_str(),
                              b.spec.node.empty() ? "" : " : ", b.spec.node.c_str());
        }
    }
    for (const std::size_t child : children[e]) entityNode(f, d, children, partsOf, child);
    ImGui::TreePop();
}

void sceneOutliner(Frame& f) {
    const terra::scene::SceneInstance& scene = f.simulated->instance;
    const terra::data::ModelDescription& d = scene.world->description();
    // 父子关系与每个实体的视觉部件
    std::vector<std::vector<std::size_t>> children(d.entities.size());
    std::vector<std::vector<std::size_t>> roots(d.modelInstances.size() + 1);  // 下标 0：不属于任何实例
    for (std::size_t e = 0; e < d.entities.size(); ++e) {
        const auto& entity = d.entities[e];
        if (entity.parent.valid() && entity.parent.value <= d.entities.size()) children[entity.parent.value - 1].push_back(e);
        else roots[std::min<std::size_t>(entity.modelInstance.value, d.modelInstances.size())].push_back(e);
    }
    std::vector<std::vector<const terra::scene::VisualPart*>> partsOf(d.entities.size());
    std::vector<std::vector<const terra::scene::VisualPart*>> staticOf(d.modelInstances.size() + 1);
    for (const auto& part : scene.visuals) {
        if (part.entity.valid() && part.entity.value <= d.entities.size()) {
            partsOf[part.entity.value - 1].push_back(&part);
        } else {
            staticOf[std::min<std::size_t>(part.modelInstance.value, d.modelInstances.size())].push_back(&part);
        }
    }
    ImGui::SetNextItemOpen(true, ImGuiCond_Once);
    const std::string title = scene.document.name.empty() ? std::filesystem::path{f.simulated->path}.filename().string()
                                                           : scene.document.name;
    if (!ImGui::TreeNode("##scene", "%s", title.c_str())) return;
    for (std::size_t i = 0; i <= d.modelInstances.size(); ++i) {
        if (i == 0 && roots[0].empty() && staticOf[0].empty()) continue;
        const std::string name = i == 0 ? std::string{"world"} : d.modelInstances[i - 1].name;
        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (!ImGui::TreeNode(reinterpret_cast<void*>(static_cast<std::uintptr_t>(i + 1000000)), "%s  (%zu %s)", name.c_str(), roots[i].size(),
                             tr(TextId::OutlinerInstanceCount))) {
            continue;
        }
        for (const std::size_t e : roots[i]) entityNode(f, d, children, partsOf, e);
        if (!staticOf[i].empty()) {
            if (ImGui::TreeNode(reinterpret_cast<void*>(static_cast<std::uintptr_t>(i + 2000000)), "%s", tr(TextId::OutlinerStatic))) {
                for (const terra::scene::VisualPart* part : staticOf[i]) {
                    if (part->kind == terra::scene::VisualPart::Kind::Geom) {
                        if (const auto* g = d.geom(part->geom)) {
                            ImGui::BulletText("%s %s", shapeText(g->shape), g->name.empty() ? tr(TextId::OutlinerUnnamed) : g->name.c_str());
                        }
                    } else {
                        const auto& b = scene.bindings[static_cast<std::size_t>(part->binding)];
                        ImGui::BulletText("%s %s", tr(TextId::OutlinerGltf), std::filesystem::path{b.spec.gltf}.filename().string().c_str());
                    }
                }
                ImGui::TreePop();
            }
        }
        ImGui::TreePop();
    }
    ImGui::TreePop();
}

void outlinerPanel(Frame& f) {
    bool& visible = f.settings.current.ui.showOutliner;
    if (!visible) return;
    if (ImGui::Begin(label(TextId::PanelOutliner, "Outliner").c_str(), &visible)) {
        if (f.simulated) {
            sceneOutliner(f);
            ImGui::End();
            return;
        }
        ImGui::TextDisabled("%s", tr(TextId::OutlinerNoScene));
        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::TreeNode(tr(TextId::OutlinerScene))) {
            for (const TextId item : {TextId::OutlinerGround, TextId::OutlinerAxes, TextId::OutlinerCube, TextId::OutlinerSpheres,
                                      TextId::OutlinerCylinder, TextId::OutlinerCapsule, TextId::OutlinerSun}) {
                ImGui::BulletText("%s", tr(item));
            }
            for (const auto& a : f.scene.assets()) {
                ImGui::BulletText("%s  (%zu △, %d %s)", a.name.c_str(), a.triangles, a.textures, tr(TextId::OutlinerTextures));
            }
            ImGui::TreePop();
        }
        ImGui::Spacing();
        ImGui::TextDisabled("%s", tr(TextId::OutlinerHint));
    }
    ImGui::End();
}

// 属性面板的“渲染”分组（M1-08）：直接修改设置，下一帧生效并自动保存。
void renderControls(st::RenderSettings& rs, bool environmentHasSun) {
    ImGui::SeparatorText(tr(TextId::DetailsRender));
    const auto slider = [](TextId name, const char* id, double& value, st::Range range, const char* format) {
        ImGui::SetNextItemWidth(-FLT_MIN);
        ui::propertySlider(id, tr(name), &value, range.min, range.max, format);
    };
    // 环境光来源（M1-08b）
    const std::pair<st::EnvironmentSetting, TextId> environments[] = {
        {st::EnvironmentSetting::Outdoor, TextId::EnvironmentOutdoor},
        {st::EnvironmentSetting::Studio, TextId::EnvironmentStudio},
        {st::EnvironmentSetting::Hemisphere, TextId::EnvironmentHemisphere}};
    const char* currentEnvironment = "";
    for (const auto& [e, text] : environments) {
        if (e == rs.environment) currentEnvironment = tr(text);
    }
    ImGui::TextUnformatted(tr(TextId::RenderEnvironment));
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::BeginCombo("##environment", currentEnvironment)) {
        for (const auto& [e, text] : environments) {
            if (ImGui::Selectable(tr(text), e == rs.environment)) rs.environment = e;
        }
        ImGui::EndCombo();
    }
    const bool imageBased = rs.environment != st::EnvironmentSetting::Hemisphere;
    ImGui::BeginDisabled(!imageBased);
    slider(TextId::RenderEnvironmentIntensity, "envIntensity", rs.environmentIntensity, st::ranges::kEnvironmentIntensity, "%.2f");
    slider(TextId::RenderEnvironmentRotation, "envRotation", rs.environmentRotationDegrees, st::ranges::kEnvironmentRotation, "%.0f°");
    ImGui::Checkbox(tr(TextId::RenderBackground), &rs.environmentBackground);
    ImGui::BeginDisabled(!rs.environmentBackground);
    slider(TextId::RenderBackgroundBlur, "backgroundBlur", rs.backgroundBlur, st::ranges::kBackgroundBlur, "%.2f");
    ImGui::EndDisabled();
    ImGui::EndDisabled();
    ImGui::Spacing();

    ImGui::BeginDisabled(!imageBased);
    ImGui::Checkbox(tr(TextId::RenderSunFromEnvironment), &rs.sunFromEnvironment);
    ImGui::EndDisabled();
    const bool following = imageBased && rs.sunFromEnvironment && environmentHasSun;
    if (imageBased && rs.sunFromEnvironment && !environmentHasSun) ImGui::TextDisabled("%s", tr(TextId::RenderNoSunInEnvironment));
    ImGui::BeginDisabled(following);  // 跟随贴图中的太阳时，手动的太阳方向与强度不起作用
    slider(TextId::RenderSunAzimuth, "sunAzimuth", rs.sunAzimuthDegrees, st::ranges::kAzimuth, "%.0f°");
    slider(TextId::RenderSunElevation, "sunElevation", rs.sunElevationDegrees, st::ranges::kElevation, "%.0f°");
    slider(TextId::RenderSunIntensity, "sunIntensity", rs.sunIntensity, st::ranges::kSunIntensity, "%.2f");
    ImGui::EndDisabled();
    ImGui::BeginDisabled(imageBased);  // 半球光只在不用环境贴图时生效
    slider(TextId::RenderAmbient, "ambient", rs.ambientIntensity, st::ranges::kAmbient, "%.2f");
    ImGui::EndDisabled();
    slider(TextId::RenderExposure, "exposure", rs.exposureEv, st::ranges::kExposure, "%+.1f EV");

    const std::pair<st::ToneMapperSetting, TextId> mappers[] = {{st::ToneMapperSetting::AgX, TextId::ToneMapperAgx},
                                                                {st::ToneMapperSetting::PbrNeutral, TextId::ToneMapperNeutral},
                                                                {st::ToneMapperSetting::Aces, TextId::ToneMapperAces},
                                                                {st::ToneMapperSetting::None, TextId::ToneMapperNone}};
    const char* currentMapper = "";
    for (const auto& [m, text] : mappers) {
        if (m == rs.toneMapper) currentMapper = tr(text);
    }
    ImGui::TextUnformatted(tr(TextId::RenderToneMapper));
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::BeginCombo("##toneMapper", currentMapper)) {
        for (const auto& [m, text] : mappers) {
            if (ImGui::Selectable(tr(text), m == rs.toneMapper)) rs.toneMapper = m;
        }
        ImGui::EndCombo();
    }

    ImGui::TextUnformatted(tr(TextId::RenderMsaa));
    ImGui::SetNextItemWidth(-FLT_MIN);
    const auto msaaText = [](int samples) {
        return samples <= 1 ? std::string{tr(TextId::RenderMsaaOff)} : std::to_string(samples) + "×";
    };
    if (ImGui::BeginCombo("##msaa", msaaText(rs.msaaSamples).c_str())) {
        for (const int samples : {1, 2, 4, 8}) {
            if (ImGui::Selectable(msaaText(samples).c_str(), samples == rs.msaaSamples)) rs.msaaSamples = samples;
        }
        ImGui::EndCombo();
    }
    ImGui::Checkbox(tr(TextId::RenderAssetArmOcclusion), &rs.assetArmOcclusion);
    ImGui::Checkbox(tr(TextId::RenderShadows), &rs.shadows);
    ImGui::SameLine();
    ImGui::Checkbox(tr(TextId::RenderGrid), &rs.grid);
    if (ImGui::Button(tr(TextId::RenderRestore))) rs = st::RenderSettings{};
}

// 选中实体的位姿与速度（M2-08，只读）。姿态以滚转 / 俯仰 / 偏航（度，R = Rz·Ry·Rx）显示。
void entityDetails(Frame& f) {
    const terra::scene::SceneInstance& scene = f.simulated->instance;
    const terra::data::ModelDescription& d = scene.world->description();
    ImGui::SeparatorText(tr(TextId::DetailsEntity));
    const terra::data::EntityDescription* e = d.entity(f.state.selectedEntity);
    if (e == nullptr) {
        ImGui::TextDisabled("%s", tr(TextId::DetailsSelectHint));
        return;
    }
    const auto snapshot = f.simulated->runner->latest();
    if (!snapshot || e->id.value > snapshot->entities.size()) return;
    const terra::data::EntityState& s = snapshot->entities[e->id.value - 1];
    const auto& q = s.pose.orientation;
    const double rad = 180.0 / 3.14159265358979323846;
    const double roll = std::atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)) * rad;
    const double pitch = std::asin(std::clamp(2 * (q.w * q.y - q.z * q.x), -1.0, 1.0)) * rad;
    const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) * rad;
    if (ImGui::BeginTable("entity", 2, ImGuiTableFlags_SizingStretchProp)) {
        const auto row = [](const char* name, const std::string& value) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextDisabled("%s", name);
            ImGui::TableSetColumnIndex(1);
            ImGui::TextUnformatted(value.c_str());
        };
        const auto vec = [](double x, double y, double z, const char* unit) {
            char buf[128];
            std::snprintf(buf, sizeof buf, "%.4f, %.4f, %.4f %s", x, y, z, unit);
            return std::string{buf};
        };
        row(tr(TextId::DetailsName), e->name.empty() ? std::string{tr(TextId::OutlinerUnnamed)} : e->name);
        const auto* inst = d.modelInstance(e->modelInstance);
        row(tr(TextId::DetailsInstance), inst != nullptr ? inst->name : std::string{tr(TextId::DetailsNone)});
        const auto* parent = d.entity(e->parent);
        row(tr(TextId::DetailsParent), parent != nullptr ? (parent->name.empty() ? std::string{tr(TextId::OutlinerUnnamed)} : parent->name)
                                                         : std::string{tr(TextId::DetailsNone)});
        row(tr(TextId::DetailsPosition), vec(s.pose.position.x, s.pose.position.y, s.pose.position.z, "m"));
        row(tr(TextId::DetailsOrientation), vec(roll, pitch, yaw, "°"));
        row(tr(TextId::DetailsLinearVelocity), vec(s.linearVelocity.x, s.linearVelocity.y, s.linearVelocity.z, "m/s"));
        row(tr(TextId::DetailsAngularVelocity), vec(s.angularVelocity.x, s.angularVelocity.y, s.angularVelocity.z, "rad/s"));
        ImGui::EndTable();
    }
}

void detailsPanel(Frame& f) {
    bool& visible = f.settings.current.ui.showDetails;
    if (!visible) return;
    if (ImGui::Begin(label(TextId::PanelDetails, "Details").c_str(), &visible)) {
        if (f.simulated) entityDetails(f);
        ImGui::SeparatorText(tr(TextId::DetailsCamera));
        const auto& p = f.camera.position;
        const double rad = 180.0 / 3.14159265358979323846;
        if (ImGui::BeginTable("camera", 2, ImGuiTableFlags_SizingStretchProp)) {
            const auto row = [](const char* name, const std::string& value) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextDisabled("%s", name);
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(value.c_str());
            };
            char buf[96];
            std::snprintf(buf, sizeof(buf), "%.2f, %.2f, %.2f", p.x, p.y, p.z);
            row(tr(TextId::DetailsPosition), buf);
            std::snprintf(buf, sizeof(buf), "%.1f", f.camera.yaw() * rad);
            row(tr(TextId::DetailsYaw), buf);
            std::snprintf(buf, sizeof(buf), "%.1f", f.camera.pitch() * rad);
            row(tr(TextId::DetailsPitch), buf);
            std::snprintf(buf, sizeof(buf), "%.0f", f.camera.verticalFov * rad);
            row(tr(TextId::DetailsFov), buf);
            ImGui::EndTable();
        }
        renderControls(f.settings.current.render, f.environments.sun(f.settings.current.render.environment).has_value());
        ImGui::Spacing();
        ImGui::TextDisabled("%s", tr(TextId::DetailsHint));
    }
    ImGui::End();
}

// 执行器面板（M3-04c）：每个执行器一行——名字、类别与单位、滑块（范围为控制范围；PD 层接管的为关节限位）、输出。
// 动作播放时只读。拖动即修改目标：PD 层接管的交给 PD 层，其余在下一步之前下发。
const char* actuatorUnit(const terra::data::ActuatorDescription& a, const terra::data::ModelDescription& d, bool pd) {
    const terra::data::JointDescription* j = a.joint.valid() ? d.joint(a.joint) : nullptr;
    const bool slide = j != nullptr && j->type == terra::data::JointType::Slide;
    if (j == nullptr && !pd) return "";
    if (pd || a.kind == terra::data::ActuatorKind::Position) return slide ? "m" : "rad";
    if (a.kind == terra::data::ActuatorKind::Velocity) return slide ? "m/s" : "rad/s";
    if (a.kind == terra::data::ActuatorKind::Motor) return slide ? "N" : "N·m";
    return "";
}

const char* actuatorKindText(terra::data::ActuatorKind kind, bool pd) {
    if (pd) return tr(TextId::ActuatorKindPd);
    switch (kind) {
        case terra::data::ActuatorKind::Motor: return tr(TextId::ActuatorKindMotor);
        case terra::data::ActuatorKind::Position: return tr(TextId::ActuatorKindPosition);
        case terra::data::ActuatorKind::Velocity: return tr(TextId::ActuatorKindVelocity);
        case terra::data::ActuatorKind::Other: return tr(TextId::ActuatorKindOther);
    }
    return "";
}

void actuatorPanel(Frame& f) {
    if (!f.state.showActuators) return;
    bool visible = true;
    if (ImGui::Begin(label(TextId::PanelActuators, "Actuators").c_str(), &visible)) {
        if (!f.simulated) {
            ImGui::TextDisabled("%s", tr(TextId::OutlinerNoScene));
        } else {
            SimulatedScene& sim = *f.simulated;
            const auto& d = sim.instance.world->description();
            const auto snap = sim.instance.world->snapshot();
            const bool locked = sim.motion && sim.motion->playing();
            if (locked) ImGui::TextWrapped("%s", tr(TextId::ActuatorsLocked));
            ImGui::BeginDisabled(locked);
            std::string instance;
            for (std::size_t i = 0; i < d.actuators.size(); ++i) {
                const auto& a = d.actuators[i];
                const auto slash = a.name.find('/');
                const std::string prefix = slash == std::string::npos ? std::string{} : a.name.substr(0, slash);
                if (i == 0 || prefix != instance) {
                    instance = prefix;
                    ImGui::SeparatorText(instance.empty() ? "-" : instance.c_str());
                }
                const std::string shortName = slash == std::string::npos ? a.name : a.name.substr(slash + 1);
                ImGui::PushID(static_cast<int>(a.id.value));
                if (a.inputs.size() != 1) {
                    ImGui::TextDisabled("%s  %s", shortName.c_str(), tr(TextId::ActuatorsMultiInput));
                    ImGui::PopID();
                    continue;
                }
                const bool pd = sim.pd.manages(a.id);
                double value = 0.0;
                if (const auto pending = sim.panelTargets.find(a.id.value); pending != sim.panelTargets.end()) {
                    value = pending->second;
                } else if (pd) {
                    const auto* j = d.joint(a.joint);
                    value = sim.pd.target(a.id).value_or(j ? snap->joints[a.joint.value - 1].position[0] : 0.0);
                } else if (i < snap->actuators.size() && !snap->actuators[i].controls.empty()) {
                    value = snap->actuators[i].controls[0];
                }
                ImGui::TextUnformatted(shortName.c_str());
                ImGui::SameLine();
                ImGui::TextDisabled("%s %s", actuatorKindText(a.kind, pd), actuatorUnit(a, d, pd));
                const auto range = pd ? sim.pd.targetRange(a) : terra::simulation::controlRange(a);
                ImGui::SetNextItemWidth(-ImGui::GetFontSize() * 5.0f);
                bool changed = false;
                if (range) {
                    double lo = range->first;
                    double hi = range->second;
                    changed = ImGui::SliderScalar("##target", ImGuiDataType_Double, &value, &lo, &hi, "%.3f",
                                                  ImGuiSliderFlags_AlwaysClamp);
                } else {
                    changed = ImGui::DragScalar("##target", ImGuiDataType_Double, &value, 0.01f, nullptr, nullptr, "%.3f");
                }
                ImGui::SameLine();
                ImGui::TextDisabled("%s %.2f", tr(TextId::ActuatorForce), i < snap->actuators.size() ? snap->actuators[i].force : 0.0);
                if (changed && !locked) {
                    sim.panelTargets[a.id.value] = value;
                    if (pd) {
                        sim.pd.setTarget(a.id, value);
                    } else {
                        sim.runner->scheduleNext({{a.id, {value}}});
                    }
                }
                ImGui::PopID();
            }
            ImGui::EndDisabled();
            // 已经出现在快照中的面板目标不再单独保存（PD 层的目标始终由 PD 层保存）
            for (auto it = sim.panelTargets.begin(); it != sim.panelTargets.end();) {
                const auto index = static_cast<std::size_t>(it->first - 1);
                const bool pd = sim.pd.manages(terra::data::ActuatorId{it->first});
                const bool applied = pd || (index < snap->actuators.size() && !snap->actuators[index].controls.empty() &&
                                            snap->actuators[index].controls[0] == it->second);
                it = applied ? sim.panelTargets.erase(it) : std::next(it);
            }
        }
    }
    ImGui::End();
    if (!visible) f.state.showActuators = false;
}

void logPanel(Frame& f) {
    bool& visible = f.settings.current.ui.showLog;
    if (!visible) return;
    if (ImGui::Begin(label(TextId::PanelLog, "Log").c_str(), &visible)) {
        for (const auto& line : f.state.log) ImGui::TextUnformatted(line.c_str());
        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) ImGui::SetScrollHereY(1.0f);
    }
    ImGui::End();
}

// 性能图表（M1-15，用户建议）：帧时间、CPU 占用、显存、进程内存，随时间向左滚动。
void performanceCharts(const PerfState& p) {
    if (!ImGui::CollapsingHeader(tr(TextId::StatsCharts), ImGuiTreeNodeFlags_DefaultOpen)) return;
    ImGui::SetNextItemWidth(-FLT_MIN);
    const ui::ChartSeries frame[] = {{&p.frameMs, color32(240, 160, 70, 255), tr(TextId::ChartFrame)},
                                     {&p.gpuMs, color32(90, 160, 245, 255), tr(TextId::ChartGpu)}};
    ui::lineChart("chartFrame", tr(TextId::ChartFrameTime), frame, 2, 0.0, "%.1f ms", 1000.0 / 60.0);
    ImGui::SetNextItemWidth(-FLT_MIN);
    const ui::ChartSeries cpu[] = {{&p.processCpu, color32(110, 200, 120, 255), tr(TextId::ChartProcess)},
                                   {&p.systemCpu, color32(160, 170, 200, 255), tr(TextId::ChartSystem)}};
    ui::lineChart("chartCpu", tr(TextId::ChartCpu), cpu, 2, 100.0, "%.0f%%");
    if (p.vram && p.vram->totalMb) {
        ImGui::SetNextItemWidth(-FLT_MIN);
        const ui::ChartSeries vram[] = {{&p.vramUsedMb, color32(200, 120, 230, 255), tr(TextId::ChartVramUsed)}};
        char title[96];
        std::snprintf(title, sizeof(title), "%s（%.0f MB）", tr(TextId::ChartVram), *p.vram->totalMb);
        ui::lineChart("chartVram", title, vram, 1, *p.vram->totalMb, "%.0f MB");
    } else {
        ImGui::TextDisabled("%s", tr(TextId::ChartVramUnavailable));
    }
    ImGui::SetNextItemWidth(-FLT_MIN);
    const ui::ChartSeries memory[] = {{&p.residentMb, color32(230, 200, 90, 255), tr(TextId::ChartResident)}};
    ui::lineChart("chartMemory", tr(TextId::ChartMemory), memory, 1, 0.0, "%.0f MB");
}

void statsPanel(Frame& f) {
    bool& visible = f.settings.current.ui.showStats;
    if (!visible) return;
    if (ImGui::Begin(label(TextId::PanelStats, "Stats").c_str(), &visible)) {
        const auto& info = f.ctx.info();
        if (ImGui::BeginTable("stats", 2, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg)) {
            const auto row = [](TextId name, const std::string& value) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextDisabled("%s", tr(name));
                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(value.c_str());
            };
            char buf[128];
            std::snprintf(buf, sizeof(buf), "%.1f", f.state.fps);
            row(TextId::StatsFrameRate, buf);
            std::snprintf(buf, sizeof(buf), "%.2f ms", f.state.frameMs);
            row(TextId::StatsFrameTime, buf);
            row(TextId::StatsRenderer, info.renderer);
            row(TextId::StatsOpenGl, info.version);
            row(TextId::StatsDisplayMode, displayModeText(f.window.displayMode()));
            if (f.renderer.width() > 0) {
                std::snprintf(buf, sizeof(buf), "%d × %d", f.renderer.width(), f.renderer.height());
                row(TextId::StatsViewportSize, buf);
                const int samples = f.renderer.lastFrame().samples;
                row(TextId::StatsMsaa, samples > 1 ? std::to_string(samples) + "×" : std::string{tr(TextId::RenderMsaaOff)});
                // GPU 分阶段耗时（M1-10，时间戳查询，结果落后 1–2 帧；按约 0.5 秒平滑）
                const r::PassTimings& t = f.renderer.lastTimings();
                if (t.valid) {
                    for (int i = 0; i < r::kRenderPassCount; ++i) {
                        const auto k = static_cast<std::size_t>(i);
                        f.state.gpuMs[k] = f.state.gpuMs[k] * 0.97 + t.milliseconds[k] * 0.03;
                    }
                    double total = 0.0;
                    for (const double ms : f.state.gpuMs) total += ms;
                    std::snprintf(buf, sizeof(buf), "%.2f ms", total);
                    row(TextId::StatsGpuTime, buf);
                    const TextId names[r::kRenderPassCount] = {TextId::StatsGpuShadow, TextId::StatsGpuOpaque, TextId::StatsGpuBackground,
                                                               TextId::StatsGpuGrid, TextId::StatsGpuResolve, TextId::StatsGpuToneMap};
                    for (int i = 0; i < r::kRenderPassCount; ++i) {
                        std::snprintf(buf, sizeof(buf), "%.2f ms", f.state.gpuMs[static_cast<std::size_t>(i)]);
                        row(names[i], buf);
                    }
                }
            }
            std::snprintf(buf, sizeof(buf), "%.2f", static_cast<double>(f.window.contentScale()));
            row(TextId::StatsContentScale, buf);
            std::snprintf(buf, sizeof(buf), "%.0f%%", static_cast<double>(f.uiCtx.scale()) * 100.0);
            row(TextId::StatsUiScale, buf);
            std::snprintf(buf, sizeof(buf), "%d", f.state.perf.sample.logicalCores);
            row(TextId::StatsCpuCores, buf);
            row(TextId::StatsVramSource, f.state.perf.vram ? f.state.perf.vram->source : std::string{tr(TextId::StatsUnavailable)});
            ImGui::EndTable();
        }
        performanceCharts(f.state.perf);
    }
    ImGui::End();
}

void helpWindows(Frame& f) {
    if (f.state.showControls) {
        ImGui::SetNextWindowSize(ImVec2(0, 0), ImGuiCond_Always);
        if (ImGui::Begin(label(TextId::ControlsTitle, "Controls").c_str(), &f.state.showControls,
                         ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::TextUnformatted(tr(TextId::ControlsBody));
        }
        ImGui::End();
    }
    if (f.state.showAbout) {
        if (ImGui::Begin(label(TextId::AboutTitle, "About").c_str(), &f.state.showAbout,
                         ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::TextUnformatted(tr(TextId::AboutBody));
        }
        ImGui::End();
    }
}

void buildUi(Frame& f) {
    shortcuts(f);
    menuBar(f);
    statusBar(f);
    const ImGuiID dockspace = ImGui::DockSpaceOverViewport(0, nullptr, ImGuiDockNodeFlags_None);
    if (!f.state.layoutBuilt) {
        // 读入的布局中有停靠分割时沿用，否则（首次运行、恢复默认布局）建立默认布局。
        const ImGuiDockNode* node = ImGui::DockBuilderGetNode(dockspace);
        if (f.settings.current.ui.layout.empty() || node == nullptr || node->IsLeafNode()) buildDefaultLayout(dockspace);
        f.state.layoutBuilt = true;
    }
    viewportPanel(f);
    outlinerPanel(f);
    detailsPanel(f);
    actuatorPanel(f);
    logPanel(f);
    statsPanel(f);
    helpWindows(f);
    vw::drawSettingsWindow(f.state.settingsWindow, f.settings.current, f.state.monitors, f.uiCtx.scale(), f.settings.path,
                           f.state.log);
    if (auto chosen = f.state.fileBrowser.draw(label(TextId::FileBrowserTitle, "OpenScene").c_str())) f.state.openRequest = *chosen;
    if (auto chosen = f.state.motionBrowser.draw(label(TextId::MotionBrowserTitle, "OpenMotion").c_str())) f.state.motionRequest = *chosen;
    sceneErrorWindow(f);
}

// ImGui 的布局有变化时（它自带约 5 秒的节流）把布局文本记入设置；布局只在启动时读入，因此同时记为已生效。
void captureLayout(Frame& f, bool now) {
    ImGuiIO& io = ImGui::GetIO();
    if (!io.WantSaveIniSettings && !now) return;
    f.settings.current.ui.layout = ImGui::SaveIniSettingsToMemory();
    f.settings.applied.ui.layout = f.settings.current.ui.layout;
    io.WantSaveIniSettings = false;
}

// ---- 自检 ----

struct Rgb {
    unsigned r, g, b;
};

Rgb pixelAt(const std::vector<std::uint8_t>& p) {
    return p.size() >= 4 ? Rgb{p[0], p[1], p[2]} : Rgb{0, 0, 0};
}

bool near(Rgb a, Rgb b, unsigned tol) {
    const auto d = [](unsigned x, unsigned y) { return x > y ? x - y : y - x; };
    return d(a.r, b.r) <= tol && d(a.g, b.g) <= tol && d(a.b, b.b) <= tol;
}

std::string str(Rgb c) {
    return "(" + std::to_string(c.r) + ", " + std::to_string(c.g) + ", " + std::to_string(c.b) + ")";
}

Rgb themeRgb(ImGuiCol col) {
    const ImVec4 c = ui::themeColor(col);
    const auto to8 = [](float x) { return static_cast<unsigned>(std::lround(static_cast<double>(x) * 255.0)); };
    return {to8(c.x), to8(c.y), to8(c.z)};
}

int selfTest(Frame& f, const ui::UiContext& uiContext, const Options& options) {
    int failures = 0;
    const auto check = [&](bool ok, const std::string& what) {
        std::printf("%s %s\n", ok ? "PASS" : "FAILED:", what.c_str());
        if (!ok) ++failures;
    };
    const auto& info = f.ctx.info();
    std::printf("INFO 渲染器 %s，OpenGL %s\n", info.renderer.c_str(), info.version.c_str());
    if (!options.expectRenderer.empty()) {
        check(info.renderer.find(options.expectRenderer) != std::string::npos, "渲染器包含 " + options.expectRenderer);
    }
    check(!uiContext.usingFallbackFont(), "界面字体为 Noto Sans SC 子集（未回退）");
    check(uiContext.regularFont() != nullptr && uiContext.regularFont()->IsGlyphInFont(0x8BBE) &&
              uiContext.regularFont()->IsGlyphInFont(0x2191),
          "字体包含“设”与“↑”");
    check(f.ctx.errorMessageCount() == 0, "整个运行期间没有 GL 错误（含 ImGui 渲染后端）");
    check(std::string{tr(TextId::MenuFile)} == "File", "语言已切换为英文");

    // 视口：画面中心是立方体朝向相机的 -X 面；期望颜色由 C++ 参考着色（与着色器同一公式）按当前渲染设置算出（M1-08a）
    Rgb cubeFace{0, 0, 0};
    {
        const auto& cam = f.camera.position;
        const auto fwd = f.camera.forward();
        const double t = (-0.5 - cam.x) / fwd.x;  // 与 x = -0.5 平面相交
        const r::Vec3f hit{static_cast<float>(cam.x + t * fwd.x), static_cast<float>(cam.y + t * fwd.y),
                           static_cast<float>(cam.z + t * fwd.z)};
        check(t > 0.0 && std::fabs(hit.y) < 0.5f && hit.z > 0.0f && hit.z < 1.0f, "视口中心射线落在立方体的 -X 面上");
        const r::Vec3f toEye{static_cast<float>(cam.x) - hit.x, static_cast<float>(cam.y) - hit.y, static_cast<float>(cam.z) - hit.z};
        const float len = std::sqrt(toEye.x * toEye.x + toEye.y * toEye.y + toEye.z * toEye.z);
        const st::RenderSettings& rs = f.settings.current.render;
        const r::Vec3f radiance = r::pbr::shade(DemoScene::cubeMaterial(), {-1.0f, 0.0f, 0.0f},
                                                {toEye.x / len, toEye.y / len, toEye.z / len}, vw::toSunLight(rs),
                                                vw::toEnvironment(rs), 1.0f);
        const auto px = r::pbr::displayPixel(radiance, vw::toToneMapper(rs.toneMapper), static_cast<float>(rs.exposureEv));
        cubeFace = {px[0], px[1], px[2]};
    }
    if (f.renderer.width() > 0) {
        const Rgb center = pixelAt(f.renderer.readOutput(f.renderer.width() / 2, f.renderer.height() / 2, 1, 1));
        std::printf("INFO 视口 %d×%d（%d× MSAA，阴影投射物 %d 个），中心像素 %s，参考着色 %s\n", f.renderer.width(),
                    f.renderer.height(), f.renderer.lastFrame().samples, f.renderer.lastFrame().shadowCasters,
                    str(center).c_str(), str(cubeFace).c_str());
        check(near(center, cubeFace, 3), "视口中心与 C++ 参考着色一致（立方体受光面，AgX）");
        check(f.renderer.lastFrame().shadowsDrawn, "绘制了太阳光阴影");
        {
            const std::string toolbox = std::string{TERRA_ASSETS_DIR} + "/models/metal_toolbox/metal_toolbox_2k.gltf";
            if (std::FILE* probe = std::fopen(toolbox.c_str(), "rb")) {
                std::fclose(probe);
                const auto& assets = f.scene.assets();
                bool texturesOk = !assets.empty();
                for (const auto& asset : assets) texturesOk = texturesOk && asset.textures == 3 && asset.triangles > 0;
                check(assets.size() == 3 && texturesOk, "三个 glTF 资产已加载，每个 3 张贴图（实际 " + std::to_string(assets.size()) + " 个）");
            } else {
                std::printf("INFO 未找到 glTF 资产（未运行 scripts/assets/fetch_models.sh），跳过资产检查\n");
            }
        }
        {
            const PerfState& p = f.state.perf;
            check(p.frameMs.size() >= 30 && p.residentMb.size() >= 1 && p.residentMb.latest() > 0.0,
                  "性能图表数据：帧时间已记录 " + std::to_string(p.frameMs.size()) + " 帧，进程内存已采样");
            if (p.vram && p.vram->totalMb) {
                std::printf("INFO 显存（%s）：总量 %.0f MB，可用 %.0f MB\n", p.vram->source.c_str(), *p.vram->totalMb, p.vram->availableMb);
            } else {
                std::printf("INFO 显存：当前驱动不提供查询\n");
            }
        }
        if (f.environments.available(st::EnvironmentSetting::Outdoor)) {
            check(f.state.imageLightingSeen, "室外环境贴图已加载，环境光照与背景已绘制（前 3/4 的帧）");
        } else {
            std::printf("INFO 未找到 assets/hdri/%s（未运行 scripts/assets/fetch_hdri.sh），跳过环境贴图检查\n",
                        EnvironmentLibrary::fileName(st::EnvironmentSetting::Outdoor));
        }
    } else {
        check(false, "视口已渲染");
    }
    {
        const r::RenderSettings a = vw::toRenderSettings(st::RenderSettings{});
        const r::RenderSettings b;
        check(a.toneMapper == b.toneMapper && a.exposureEv == b.exposureEv && a.msaaSamples == b.msaaSamples &&
                  a.shadows == b.shadows && a.grid == b.grid &&
                  vw::toEnvironment(st::RenderSettings{}).intensity == r::HemisphereLight{}.intensity &&
                  vw::toSunLight(st::RenderSettings{}).intensity == r::SunLight{}.intensity,
              "设置的默认渲染参数与渲染器默认值一致");
    }

    // 窗口画面（尚未交换的后缓冲）：视口图像中心显示同一颜色；菜单栏右端为菜单栏背景色
    const auto fb = f.window.framebufferSize();
    const ImVec2 scale = ImGui::GetIO().DisplayFramebufferScale;
    const float cx = (f.state.viewportImageMin.x + f.state.viewportImageMax.x) * 0.5f * scale.x;
    const float cy = (f.state.viewportImageMin.y + f.state.viewportImageMax.y) * 0.5f * scale.y;
    const Rgb shown = pixelAt(f.ctx.readPixels(static_cast<int>(cx), fb.height - 1 - static_cast<int>(cy), 1, 1));
    std::printf("INFO 窗口中视口图像中心像素 %s\n", str(shown).c_str());
    check(near(shown, cubeFace, 3), "视口图像正确显示在窗口中（方向与位置）");
    const int menuY = fb.height - 1 - static_cast<int>(ImGui::GetFrameHeight() * scale.y * 0.5f);
    const Rgb menu = pixelAt(f.ctx.readPixels(fb.width - 4, menuY, 1, 1));
    std::printf("INFO 菜单栏右端像素 %s，主题菜单栏背景 %s\n", str(menu).c_str(), str(themeRgb(ImGuiCol_MenuBarBg)).c_str());
    check(near(menu, themeRgb(ImGuiCol_MenuBarBg), 3), "菜单栏使用 Terra 主题颜色");

    // 导航坐标系：+Y 圆点内、字母下方为纯绿色（默认视角下 +Y 指向左侧，不与其他轴重叠）
    const GizmoLayout& g = f.state.gizmo;
    if (g.radius > 0.0f) {
        const float gx = g.bubble[1].x * scale.x;
        const float gy = (g.bubble[1].y + 0.7f * g.bubbleRadius) * scale.y;
        const Rgb yBubble = pixelAt(f.ctx.readPixels(static_cast<int>(gx), fb.height - 1 - static_cast<int>(gy), 1, 1));
        std::printf("INFO 导航坐标系 +Y 圆点像素 %s\n", str(yBubble).c_str());
        check(near(yBubble, Rgb{115, 204, 77}, 6), "视口左下角显示导航坐标系（+Y 圆点为绿色）");
        check(g.bubble[1].x < g.center.x - g.radius * 0.9f, "默认视角下 +Y 轴指向左侧（Terra 约定 +Y 向左）");
    } else {
        check(false, "导航坐标系已绘制");
    }
    // 点击坐标轴对齐视角（直接调用对齐逻辑）
    v::Camera snapped = f.camera;
    snapView(snapped, 2);
    check(snapped.forward().z < -0.99 && snapped.position.z > f.camera.position.z, "对齐 +Z：从上方俯视");
    snapView(snapped, 0);
    check(snapped.forward().x < -0.99, "对齐 +X：从 +X 一侧看向 -X");
    // 界面缩放推算
    check(std::fabs(ui::UiContext::autoScale(1.25f, 2160) - 1.75f) < 1e-4f && std::fabs(ui::UiContext::autoScale(1.0f, 1080) - 1.0f) < 1e-4f &&
              std::fabs(ui::UiContext::autoScale(1.25f, 1440) - 1.25f) < 1e-4f,
          "界面缩放推算：4K×1.25 → 1.75，1080p → 1.0，1440p×1.25 → 1.25");

    // ---- 设置（M1-14）----
    SettingsRuntime& rt = f.settings;
    check(vw::toCameraKeys(st::KeyMap{}) == v::CameraKeys{}, "设置的默认按键与相机控制器的默认按键一致");
    {
        const v::CameraControlSettings a = vw::toCameraControl(st::CameraSettings{});
        const v::CameraControlSettings b;
        check(a.lookDegreesPerPixel == b.lookDegreesPerPixel && a.speed == b.speed && a.boost == b.boost &&
                  a.wheelStep == b.wheelStep && a.panMetersPerPixel == b.panMetersPerPixel &&
                  std::fabs(st::CameraSettings{}.fovDegrees * vw::kRadiansPerDegree - v::Camera{}.verticalFov) < 1e-12,
              "设置的默认相机参数与相机控制器、相机的默认值一致");
    }
    check(rt.current.ui.language == "en" && rt.applied == rt.current, "语言切换经由设置生效（current = applied）");

    // 修改设置 → 立即生效：相机速度、视场角、界面缩放、按键（含应用的全屏键）
    st::Settings next = rt.current;
    next.camera.speed = 7.0;
    next.camera.fovDegrees = 75.0;
    next.ui.uiScale = 1.5;
    (void)next.keys.assign(st::Command::CameraForward, 0, in::Key::I);
    (void)next.keys.assign(st::Command::AppFullscreen, 0, in::Key::F10);
    rt.current = next;
    applySettings(f);
    check(f.controller.settings().speed == 7.0 && std::fabs(f.camera.verticalFov - 75.0 * vw::kRadiansPerDegree) < 1e-12,
          "相机速度 7 m/s、视场角 75° 立即生效");
    check(std::fabs(f.uiCtx.scale() - 1.5f) < 1e-6f, "界面缩放 150% 已请求（下一帧生效）");
    check(f.controller.keys().forward[0] == in::Key::I && rt.applied == rt.current, "按键已交给相机控制器");
    {
        // 端到端：虚拟键盘按 I 1 秒 → 沿视线前进 7 m；W 已不再绑定
        const in::DeviceId kb = f.input.connectDevice({in::DeviceKind::Keyboard, "selftest", "virtual", ""});
        const auto press = [&](in::Key key, double dt) {
            const auto before = f.camera.position;
            f.input.setControl(kb, in::key(key), 1.0f, 0.0);
            f.input.update();
            f.controller.update(f.camera, dt);
            f.input.setControl(kb, in::key(key), 0.0f, 0.0);
            f.input.update();
            f.controller.update(f.camera, 0.0);
            const auto after = f.camera.position;
            return terra::data::Vec3{after.x - before.x, after.y - before.y, after.z - before.z};
        };
        const auto fwd = f.camera.forward();
        const auto movedI = press(in::Key::I, 1.0);
        const auto movedW = press(in::Key::W, 1.0);
        check(std::fabs(movedI.x - 7.0 * fwd.x) < 1e-9 && std::fabs(movedI.y - 7.0 * fwd.y) < 1e-9 &&
                  std::fabs(movedI.z - 7.0 * fwd.z) < 1e-9,
              "改绑后按 I 1 秒：沿视线前进 7 m");
        check(movedW.x == 0.0 && movedW.y == 0.0 && movedW.z == 0.0, "改绑后 W 不再移动相机");
        const in::ActionHandle fullscreen = f.input.findAction(kAppMap, "fullscreen");
        f.input.setControl(kb, in::key(in::Key::F10), 1.0f, 0.0);
        f.input.update();
        const bool f10 = f.input.state(fullscreen).pressed;
        f.input.setControl(kb, in::key(in::Key::F10), 0.0f, 0.0);
        f.input.update();
        f.input.setControl(kb, in::key(in::Key::F11), 1.0f, 0.0);
        f.input.update();
        const bool f11 = f.input.state(fullscreen).pressed;
        f.input.disconnectDevice(kb);
        f.input.update();
        check(f10 && !f11, "全屏键改为 F10：F10 触发，F11 不再触发");
    }

    // 保存到 --settings 指定的文件并读回（含停靠布局）
    if (!rt.path.empty()) {
        captureLayout(f, true);
        const auto error = st::save(rt.current, rt.path);
        check(!error, "保存设置到 " + rt.path + (error ? "：" + *error : std::string{}));
        const auto loaded = st::load(rt.path);
        check(loaded.status == st::LoadStatus::Loaded && loaded.warnings.empty() && loaded.settings == rt.current,
              "读回的设置与保存的相同");
        check(loaded.settings.ui.layout.find("[Docking]") != std::string::npos, "设置中保存了停靠布局");
    }
    return failures == 0 ? 0 : 1;
}

// ---- 场景模式的自检（M2-08）：terra_viewer --scene <文件> --self-test --frames N ----
// 第 2 帧记录步数（打开后应暂停，为 0）；第 3 帧按“播放”；N/2 帧在大纲中选中 cart_a/chassis（夹具中存在时）；
// 3N/4 帧暂停并记录；3N/4 + 2 帧单步。结束时检查暂停、单步、选中与高亮，并与一个不经过界面、直接按同样步数推进的
// 同一场景逐位比较全部实体位姿（查看器的仿真与纯物理一致）；最后检查重置。
struct SceneSelfTestRecord {
    std::uint64_t stepsAtStart = 1;
    std::shared_ptr<const terra::data::PhysicsSnapshot> initial;  // 第 2 帧（尚未推进）的快照
    std::string motionPath;  // M3-04c：给出时第 3 帧打开动作文件（代替按播放）
    std::string openedScene;  // 第 2 帧时打开的场景（检查打开动作时是否切换到推荐场景）
    std::uint64_t stepAtPause = 0;
    double timeAtPause = -1.0;
    bool stepRequested = false;
};
SceneSelfTestRecord sceneRecord;

void sceneSelfTestScript(Frame& f, int frame, int total) {
    SimulatedScene& sim = *f.simulated;
    if (frame == 2) {
        sceneRecord.stepsAtStart = sim.instance.world->stepIndex();
        sceneRecord.initial = sim.instance.world->snapshot();
        sceneRecord.openedScene = sim.path;
    }
    if (frame == 3) {
        if (sceneRecord.motionPath.empty()) sim.togglePlay();
        else openMotion(f, sceneRecord.motionPath);
    }
    if (frame == total / 2) {
        if (const auto* e = sim.instance.world->description().findEntity("cart_a/chassis")) f.state.selectedEntity = e->id;
    }
    if (frame == total * 3 / 4) {
        sim.runner->pause();
        sceneRecord.stepAtPause = sim.instance.world->stepIndex();
        sceneRecord.timeAtPause = sim.instance.world->simulationTime();
    }
    if (frame == total * 3 / 4 + 2) {
        sim.runner->stepOnce();
        sceneRecord.stepRequested = true;
    }
}

int sceneSelfTest(Frame& f, const Options& options) {
    int failures = 0;
    const auto check = [&](bool ok, const std::string& what) {
        std::printf("%s %s\n", ok ? "PASS" : "FAILED:", what.c_str());
        if (!ok) ++failures;
    };
    SimulatedScene& sim = *f.simulated;
    terra::physics::PhysicsWorld& world = *sim.instance.world;
    const auto& info = f.ctx.info();
    std::printf("INFO 渲染器 %s；场景 %s\n", info.renderer.c_str(), sim.path.c_str());
    if (!options.expectRenderer.empty()) {
        check(info.renderer.find(options.expectRenderer) != std::string::npos, "渲染器包含 " + options.expectRenderer);
    }
    check(sceneRecord.stepsAtStart == 0, "打开场景后处于暂停（第 2 帧仍为第 0 步）");
    check(sceneRecord.stepAtPause > 0, "按播放后仿真推进（暂停时第 " + std::to_string(sceneRecord.stepAtPause) + " 步）");
    check(sceneRecord.stepRequested && world.stepIndex() == sceneRecord.stepAtPause + 1,
          "暂停后物理时间不增长，只有单步推进了 1 步（实际第 " + std::to_string(world.stepIndex()) + " 步）");
    check(std::fabs(world.simulationTime() - (sceneRecord.timeAtPause + world.description().timestep)) < 1e-12,
          "暂停期间仿真时间不变，单步后恰好多一个步长");
    check(sim.view->liveBindings() == sim.instance.bindings.size(), "全部 glTF 绑定可用（" + std::to_string(sim.view->liveBindings()) + " 个）");

    // 选中与高亮
    const auto* chassis = world.description().findEntity("cart_a/chassis");
    if (chassis != nullptr) {
        check(f.state.selectedEntity == chassis->id, "大纲中选中了 cart_a/chassis");
        const auto items = sim.items(f.settings.current.render, f.state.selectedEntity);
        std::size_t glowing = 0;
        for (const auto& item : items) glowing += item.material.emissive.x > 0.0f ? 1 : 0;
        const auto plain = sim.items(f.settings.current.render, {});
        std::size_t plainGlowing = 0;
        for (const auto& item : plain) plainGlowing += item.material.emissive.x > 0.0f ? 1 : 0;
        check(glowing > plainGlowing, "选中实体的视觉部件高亮（" + std::to_string(glowing - plainGlowing) + " 个渲染项）");
    }

    // 与纯物理一致：同一场景不经过界面，推进同样的步数，全部实体位姿逐位相同。
    // M3-04c：有动作或 PD 层时，对照也走同一条控制路径（动作的命令 → PD 层 → setControl → step）。
    if (sim.motion) {
        check(sim.motion->playing(), "动作已开始播放（" + sim.motion->motion().name + "）");
        std::printf("INFO 动作自检：动作时间 %.3f s / %.3f s\n", sim.motion->timeAt(world.stepIndex(), world.description().timestep),
                    sim.motion->motion().duration);
    }
    {
        auto fresh = terra::scene::loadScene(sim.path, terra::physics::mujoco::composeWorld);
        check(fresh.ok(), "独立加载同一场景");
        if (fresh.ok()) {
            auto& other = *fresh.scene->world;
            terra::simulation::PdLayer pd(other.description(), fresh.scene->pd);
            terra::simulation::MotionPlayer player;
            std::uint64_t from = 0;
            if (sim.motion) {
                player = *sim.motion;
                if (const auto key = player.motion().startKeyframe) other.resetToKeyframe(*key);
                // 动作从重置后的当前步开始（查看器在第 3 帧打开动作时世界处于第 0 步）
                player.start(other.stepIndex());
                from = other.stepIndex();
            }
            const double dt = other.description().timestep;
            bool stepped = true;
            for (std::uint64_t k = 0; k < world.stepIndex() && stepped; ++k) {
                auto commands = k >= from ? player.commands(k, dt) : std::vector<terra::physics::ActuatorCommand>{};
                if (!pd.empty()) commands = pd.process(std::move(commands), *other.snapshot());
                if (!commands.empty() && other.setControl(commands)) stepped = false;
                stepped = stepped && !other.step();
            }
            const auto a = world.snapshot();
            const auto b = other.snapshot();
            bool same = stepped && a->entities.size() == b->entities.size();
            for (std::size_t e = 0; same && e < a->entities.size(); ++e) {
                const auto& p = a->entities[e].pose;
                const auto& q = b->entities[e].pose;
                same = p.position.x == q.position.x && p.position.y == q.position.y && p.position.z == q.position.z &&
                       p.orientation.w == q.orientation.w && p.orientation.x == q.orientation.x && p.orientation.y == q.orientation.y &&
                       p.orientation.z == q.orientation.z;
            }
            check(same, std::string{"查看器中的仿真与直接推进的纯物理逐位相同（"} + std::to_string(world.stepIndex()) + " 步" +
                            (sim.motion ? "，含动作" : "") + (pd.empty() ? "" : "，含 PD 层") + "）");
        }
    }

    simReset(f);
    check(world.stepIndex() == 0 && world.runEpoch() >= 1 && world.simulationTime() == 0.0, "重置后回到第 0 步");
    // 打开动作时切换了场景：重置后的对照改为重新加载的同一场景的初值（场景的初始关键帧）
    std::shared_ptr<const terra::data::PhysicsSnapshot> reference = sceneRecord.initial;
    if (sim.path != sceneRecord.openedScene) {
        auto again = terra::scene::loadScene(sim.path, terra::physics::mujoco::composeWorld);
        reference = again.ok() ? again.scene->world->snapshot() : nullptr;
        check(reference != nullptr, "重新加载切换后的场景");
    }
    if (!sim.motionPath.empty()) {
        const auto doc = terra::simulation::readMotionFile(sim.motionPath);
        if (doc.document && !doc.document->scene.empty()) {
            check(sim.path == doc.document->scene, "打开动作文件时切换到了它的推荐场景：" + doc.document->scene);
        }
    }
    if (reference) {
        const auto now = world.snapshot();
        bool same = now->entities.size() == reference->entities.size();
        for (std::size_t e = 0; same && e < now->entities.size(); ++e) {
            const auto& p = now->entities[e].pose;
            const auto& q = reference->entities[e].pose;
            same = p.position.x == q.position.x && p.position.y == q.position.y && p.position.z == q.position.z &&
                   p.orientation.w == q.orientation.w && p.orientation.x == q.orientation.x &&
                   p.orientation.y == q.orientation.y && p.orientation.z == q.orientation.z;
        }
        const auto& keys = world.description().keyframes;
        const std::string home = sim.instance.homeKeyframe ? "关键帧 " + keys[*sim.instance.homeKeyframe].name : "模型初值";
        check(same, "重置后回到打开时的姿态（" + home + "），全部实体位姿逐位相同");
    }
    check(f.ctx.errorMessageCount() == 0, "整个运行期间没有 GL 错误");
    std::printf("INFO 场景自检：暂停于第 %llu 步，渲染项 %zu 个\n", static_cast<unsigned long long>(sceneRecord.stepAtPause),
                sim.items(f.settings.current.render, {}).size());
    return failures == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parseOptions(argc, argv, options)) {
        std::fprintf(stderr,
                     "用法：terra_viewer [--seconds N] [--frames N] [--language zh|en] [--settings 文件] [--self-test] "
                     "[--expect-renderer 子串] [--scene 场景文件] [--motion 动作文件] [--demo-drive]\n");
        return 2;
    }

    // ---- 设置：自检从默认值开始；否则读取文件（默认 XDG 路径）----
    SettingsRuntime settings;
    settings.path = !options.settingsPath.empty() ? options.settingsPath : options.selfTest ? std::string{} : st::defaultPath();
    settings.autosave = !options.selfTest;
    std::optional<st::LoadResult> loaded;
    std::string backupPath;
    std::string backupError;
    if (!options.selfTest && !settings.path.empty()) {
        loaded = st::load(settings.path);
        settings.current = loaded->settings;
        if (loaded->status == st::LoadStatus::Loaded && loaded->warnings.empty()) settings.saved = loaded->settings;
        if (loaded->status == st::LoadStatus::Corrupt) {
            // 先备份坏文件再写新文件；备份失败时本次不保存，避免覆盖用户的内容。
            if (auto backup = st::backupCorrupt(settings.path, &backupError)) {
                backupPath = *backup;
            } else {
                settings.autosave = false;
            }
        }
        if (loaded->status == st::LoadStatus::Unreadable) settings.autosave = false;
    }
    if (options.language) settings.current.ui.language = *options.language == ui::Language::English ? "en" : "zh";
    ui::setLanguage(toLanguage(settings.current.ui.language));

    in::InputSystem input;
    ui::UiInputBridge bridge(input);
    auto created = terra::platform::glfw::createPlatform(bridge);
    if (!created.platform) {
        std::fprintf(stderr, "无法创建平台：%s\n", created.error ? created.error->message.c_str() : "");
        return 1;
    }
    auto& platform = *created.platform;

    terra::platform::WindowDescription desc;
    desc.title = tr(TextId::AppName);
    // 自检时用较小的窗口，保证在 1280×720 的 Xvfb 屏幕内（窗口超出屏幕部分的像素不保证被绘制）。
    desc.width = options.selfTest ? 1200 : settings.current.display.windowWidth;
    desc.height = options.selfTest ? 700 : settings.current.display.windowHeight;
    desc.vsync = settings.current.display.vsync;
    desc.glDebugContext = options.selfTest;
    auto made = platform.createWindow(desc);
    if (!made.window) {
        std::fprintf(stderr, "无法创建窗口：%s\n", made.error ? made.error->message.c_str() : "");
        return 1;
    }
    auto& window = *made.window;
    auto rendered = r::RenderContext::create(window.glProcLoader());
    if (!rendered.context) {
        std::fprintf(stderr, "无法初始化渲染：%s\n", rendered.error.value_or("").c_str());
        return 1;
    }
    auto& ctx = *rendered.context;

    // 界面缩放默认按显示器推算（用户 2026-11-19：4K 全屏时字太小）；设置中固定了缩放时用设置值。
    const auto monitors = platform.monitors();
    const int monitorHeight = monitors.empty() ? 1080 : monitors.front().current.height;
    const float autoUiScale = ui::UiContext::autoScale(window.contentScale(), monitorHeight);
    ui::UiOptions uiOptions;
    uiOptions.fontDirectory = std::string{TERRA_ASSETS_DIR} + "/fonts";
    uiOptions.contentScale = settings.current.ui.uiScale ? static_cast<float>(*settings.current.ui.uiScale) : autoUiScale;
    auto uiMade = ui::UiContext::create(uiOptions);
    if (!uiMade.context) {
        std::fprintf(stderr, "无法初始化界面：%s\n", uiMade.error.c_str());
        return 1;
    }
    if (!uiMade.diagnostics.empty()) std::fprintf(stderr, "%s\n", uiMade.diagnostics.c_str());
    auto& uiContext = *uiMade.context;
    // 停靠布局：必须在第一次 NewFrame 之前读入。
    if (!settings.current.ui.layout.empty()) {
        ImGui::LoadIniSettingsFromMemory(settings.current.ui.layout.c_str(), settings.current.ui.layout.size());
    }

    ui::UiInputCapture capture(input);
    v::CameraController controller(input, vw::toCameraControl(settings.current.camera), vw::toCameraKeys(settings.current.keys));
    v::Camera camera;
    v::CameraController::reset(camera);
    camera.verticalFov = settings.current.camera.fovDegrees * vw::kRadiansPerDegree;

    in::ActionMapDefinition app{kAppMap, 0, true, {}};
    in::ActionDefinition fullscreen;
    fullscreen.name = "fullscreen";
    fullscreen.type = in::ActionType::Button;
    fullscreen.bindings = vw::buttonBindings(settings.current.keys[st::Command::AppFullscreen]);
    app.actions.push_back(fullscreen);
    // 仿真操作（M2-08）：播放 / 暂停、单步、重置，按键来自设置
    using CommandAction = std::pair<st::Command, const char*>;
    for (const auto& [command, actionName] : {CommandAction{st::Command::SimPlayPause, "sim_play"},
                                              CommandAction{st::Command::SimStep, "sim_step"},
                                              CommandAction{st::Command::SimReset, "sim_reset"}}) {
        in::ActionDefinition a;
        a.name = actionName;
        a.type = in::ActionType::Button;
        a.bindings = vw::buttonBindings(settings.current.keys[command]);
        app.actions.push_back(a);
    }
    (void)input.addActionMap(app);
    const in::ActionHandle fullscreenAction = input.findAction(kAppMap, "fullscreen");
    const in::ActionHandle simPlayAction = input.findAction(kAppMap, "sim_play");
    const in::ActionHandle simStepAction = input.findAction(kAppMap, "sim_step");
    const in::ActionHandle simResetAction = input.findAction(kAppMap, "sim_reset");

    auto rendererMade = r::SceneRenderer::create(ctx);
    if (!rendererMade.renderer) {
        std::fprintf(stderr, "无法创建场景渲染器：%s\n", rendererMade.error.value_or("").c_str());
        return 1;
    }
    r::SceneRenderer& renderer = *rendererMade.renderer;
    renderer.setTimingEnabled(true);
    DemoScene scene;
    scene.create();
    EnvironmentLibrary environments(TERRA_ASSETS_DIR);

    AppState state;
    state.autoUiScale = autoUiScale;
    state.monitors = monitors;
    state.log.push_back(tr(TextId::LogStarted));
    state.log.push_back(std::string{tr(TextId::StatsRenderer)} + "：" + ctx.info().renderer);
    std::optional<SimulatedScene> simulated;
    if (loaded) {
        switch (loaded->status) {
            case st::LoadStatus::Loaded: state.log.push_back(tr(TextId::LogSettingsLoaded) + settings.path); break;
            case st::LoadStatus::Missing: state.log.push_back(tr(TextId::LogSettingsMissing) + settings.path); break;
            case st::LoadStatus::Corrupt:
                state.log.push_back(backupPath.empty() ? backupError : tr(TextId::LogSettingsCorrupt) + backupPath);
                state.log.push_back(loaded->error);
                break;
            case st::LoadStatus::Unreadable: state.log.push_back(tr(TextId::LogSettingsUnreadable) + loaded->error); break;
        }
        for (const auto& w : loaded->warnings) state.log.push_back(tr(TextId::LogSettingsWarning) + w);
    }
    settings.applied = settings.current;
    Frame frame{platform, window, input, uiContext, ctx, camera, controller, scene, renderer, environments, state, settings, simulated};

    // 场景文件（M2-07 起；M2-08 起打开后暂停，--demo-drive 时直接运行）。没有场景时才加载演示场景的资产。
    if (!options.scenePath.empty()) {
        if (!openScene(frame, options.scenePath)) {
            for (const auto& line : state.sceneErrors) std::fprintf(stderr, "%s\n", line.c_str());
            std::fprintf(stderr, "错误：场景文件不能使用：%s\n", options.scenePath.c_str());
            return 4;
        }
        simulated->demoDrive = options.demoDrive;
        if (options.demoDrive) simulated->runner->resume();
    } else if (options.motionPath.empty()) {
        ensureDemoAssets(frame);
    }
    // 动作文件（M3-04c）：自检时在第 3 帧打开（代替按播放），否则启动时打开并开始播放
    if (options.selfTest && simulated) sceneRecord.motionPath = options.motionPath;
    if (!options.motionPath.empty() && !(options.selfTest && simulated)) {
        if (!openMotion(frame, options.motionPath)) {
            for (const auto& line : state.sceneErrors) std::fprintf(stderr, "%s\n", line.c_str());
            std::fprintf(stderr, "错误：动作文件不能使用：%s\n", options.motionPath.c_str());
            return 4;
        }
    }

    // 不可恢复错误（M2-02）：退出前把当前设置（含布局）写入设置文件；诊断报告中写出设置文件路径。
    // 只写文件，不调用 GL 与物理后端（terra/core/fatal.hpp 的要求）。
    terra::core::setFatalContext("设置文件", settings.path);
    const terra::core::ScopedEmergencySave emergencySettings("设置", [&settings]() -> std::optional<std::string> {
        if (!settings.autosave || settings.path.empty()) return std::nullopt;
        return st::save(settings.current, settings.path);
    });

    // 设置中的全屏模式在窗口创建后进入；失败时退回窗口模式。
    if (settings.current.display.mode != st::WindowMode::Windowed && !options.selfTest) {
        const st::DisplaySettings& d = settings.current.display;
        if (const auto error = window.setDisplayMode(vw::toDisplayMode(d.mode), monitorIndex(d, monitors),
                                                     d.mode == st::WindowMode::ExclusiveFullscreen ? toVideoMode(d.videoMode)
                                                                                                   : std::nullopt)) {
            state.log.push_back(std::string{tr(TextId::LogDisplayModeFailed)} + error->message);
        }
        syncDisplayModeFromWindow(frame);
    }

    std::printf("INFO %s，系统缩放 %.2f，显示器高度 %d，界面缩放 %.2f，字体%s\n", ctx.info().renderer.c_str(),
                static_cast<double>(window.contentScale()), monitorHeight, static_cast<double>(uiContext.scale()),
                uiContext.usingFallbackFont() ? "回退到内置（无中文）" : " Noto Sans SC 子集");
    std::printf("INFO 设置文件：%s（%s%s）\n", settings.path.empty() ? "无" : settings.path.c_str(),
                loaded ? st::loadStatusName(loaded->status) : "未读取，使用默认设置",
                settings.autosave && !settings.path.empty() ? "，自动保存" : "，不自动保存");
    if (loaded) {
        for (const auto& w : loaded->warnings) std::printf("INFO 设置警告：%s\n", w.c_str());
        if (!loaded->error.empty()) std::printf("INFO 设置错误：%s\n", loaded->error.c_str());
    }

    const double start = platform.time();
    double last = start;
    int frames = 0;
    int result = 0;
    while (!window.closeRequested() && !state.quit) {
        platform.pollEvents();
        const double now = platform.time();
        const double dt = now - last;
        last = now;
        state.frameMs = dt * 1000.0;
        state.fps = state.fps == 0.0 ? (dt > 0 ? 1.0 / dt : 0.0) : state.fps * 0.95 + (dt > 0 ? 0.05 / dt : 0.0);

        applySettings(frame);  // 上一帧界面中的改动在这里生效（界面缩放在 beginFrame 中生效）
        uiContext.beginFrame(bridge, dt, window.windowSize(), window.framebufferSize());
        // 鼠标在面板上（且不在视口里）时界面占用鼠标；输入框获得焦点或正在录制按键时界面占用键盘。
        capture.update(uiContext.wantsMouse() && !state.viewportHovered,
                       uiContext.wantsTextInput() || state.settingsWindow.capturingKey());
        input.update();
        if (input.state(fullscreenAction).pressed) toggleFullscreen(frame);
        controller.update(camera, dt);
        if (simulated) {
            const bool keyboardFree = !uiContext.wantsTextInput() && !state.settingsWindow.capturingKey();
            if (keyboardFree && input.state(simPlayAction).pressed) simulated->togglePlay();
            if (keyboardFree && input.state(simStepAction).pressed && simulated->runner->paused()) simulated->runner->stepOnce();
            if (keyboardFree && input.state(simResetAction).pressed) simReset(frame);
            if (options.selfTest) sceneSelfTestScript(frame, frames, options.frames);
            simulated->drive();
            const double before = simulated->instance.world->simulationTime();
            const auto stepped = simulated->runner->update(dt);
            const double rate = dt > 0 ? (simulated->instance.world->simulationTime() - before) / dt : 0.0;
            simulated->realtimeFactor = simulated->runner->paused() ? 0.0 : simulated->realtimeFactor * 0.9 + rate * 0.1;
            if (stepped.error) state.log.push_back(std::string{tr(TextId::StatusFaulted)} + "：" + stepped.error->message);
            for (const auto& e : stepped.controlErrors) state.log.push_back(e.message);
        }

        if (options.selfTest && options.frames > 0) {
            // 自检脚本：第 4–7 帧依次打开偏好设置的四个页面，第 7 帧打开按键录制弹窗，第 8 帧取消录制、第 9 帧关闭窗口；
            // 中间帧切换为英文（经由设置）。
            if (frames >= 4 && frames <= 7) {
                openSettingsWindow(frame, true);
                state.settingsWindow.page = frames - 4;
            }
            if (frames == 7) {
                state.settingsWindow.capture = st::Command::CameraReset;
                state.settingsWindow.captureSlot = 1;
                state.settingsWindow.captureFrames = 0;
                state.settingsWindow.openCapturePopup = true;
            }
            if (frames == 8) state.settingsWindow.capture.reset();
            if (frames == 9) openSettingsWindow(frame, false);
            if (frames == options.frames / 2) settings.current.ui.language = "en";
            // 前 3/4 的帧使用室外环境贴图（检查 IBL 管线；2026-11-24 起默认是半球光，这里显式切换），
            // 之后切到半球光，使结束时的视口像素可与 C++ 参考精确比较
            if (frames == 0 && !simulated) settings.current.render.environment = st::EnvironmentSetting::Outdoor;
            if (frames == options.frames * 3 / 4) settings.current.render.environment = st::EnvironmentSetting::Hemisphere;
        }
        buildUi(frame);
        captureLayout(frame, false);
        state.perf.record(now, dt * 1000.0, renderer.lastTimings(), ctx);

        r::Framebuffer::bindDefault();
        const auto fbSize = window.framebufferSize();
        ctx.setViewport(0, 0, fbSize.width, fbSize.height);
        ctx.setDepthTest(false);
        const ImVec4 bg = ui::themeColor(ImGuiCol_DockingEmptyBg);
        ctx.clear({bg.x, bg.y, bg.z, 1.0f});
        uiContext.render();

        // 打开 / 关闭场景在帧末处理：本帧的界面已经提交，替换 GPU 资源是安全的
        if (state.openRequest) {
            const std::string path = *state.openRequest;
            state.openRequest.reset();
            openScene(frame, path);
        }
        if (state.closeRequest) {
            state.closeRequest = false;
            closeScene(frame);
        }
        if (state.motionRequest) {
            const std::string path = *state.motionRequest;
            state.motionRequest.reset();
            openMotion(frame, path);
        }

        ++frames;
        const bool lastFrame = (options.frames > 0 && frames >= options.frames) ||
                               (options.seconds > 0.0 && now - start >= options.seconds);
        if (lastFrame && options.selfTest) result = simulated ? sceneSelfTest(frame, options) : selfTest(frame, uiContext, options);
        window.swapBuffers();
        persistSettings(frame, now, false);
        if (lastFrame) break;
    }

    // 退出时保存：窗口模式下的窗口大小与当前停靠布局。
    if (settings.autosave && !settings.path.empty()) {
        if (window.displayMode() == terra::platform::DisplayMode::Windowed) {
            const auto size = window.windowSize();
            const auto fit = [](int v) { return std::clamp(v, st::DisplaySettings::kMinWindowSize, st::DisplaySettings::kMaxWindowSize); };
            settings.current.display.windowWidth = fit(size.width);
            settings.current.display.windowHeight = fit(size.height);
        }
        captureLayout(frame, true);
        persistSettings(frame, platform.time(), true);
        std::printf("INFO 设置%s：%s\n", settings.saved && *settings.saved == settings.current ? "已保存" : "保存失败",
                    settings.path.c_str());
    }
    if (simulated) {
        const auto& clock = simulated->runner->clock();
        std::printf("INFO 仿真：%llu 步，仿真时间 %.3f 秒，过载 %llu 次（丢弃 %.3f 秒）%s\n",
                    static_cast<unsigned long long>(simulated->instance.world->stepIndex()),
                    simulated->instance.world->simulationTime(), static_cast<unsigned long long>(clock.overloadCount()),
                    clock.droppedSeconds(), simulated->runner->faulted() ? "，世界处于故障状态" : "");
    }
    std::printf("INFO 结束：%d 帧，%.1f 秒\n", frames, platform.time() - start);
    return result;
}
