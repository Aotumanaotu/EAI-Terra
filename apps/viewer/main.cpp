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
//   --library    （M3-07c）资源库文件（实验记录的索引）；默认 $TERRA_LIBRARY 或 ~/.local/share/eai-terra/library.sqlite3
//   --record-self-test （M3-07c）自检时测试记录与回放（与 --self-test、--motion 同用）
//   --sensor-self-test （M4-04）自检时检查传感器采样与预览（与 --self-test、--scene 同用；场景需含 IMU、雷达与相机）
//   --import-self-test <文件> （M4-08a）自检时经导入向导导入该模型（第 12 帧分析、第 14 帧导入并打开），检查打开的场景
//                与导入目录（与 --self-test、--library 同用；导入目录在资源库目录下的 assets/ 中）
//   --sensor-record-self-test （M4-07）自检时检查传感器的记录（含相机图像）、复现与回放（与 --self-test、--scene、--library 同用）
//   --viz-self-test （M4-08d）自检显示配置：仿真曲线；有 --ros2 时另检查 ROS2 面板（消息、图像、曲线、TF 树）；保存、重开场景后恢复、
//                导出 .terviz 与 .rviz（与 --self-test、--scene、--library 同用）
//   --library-self-test （M4-08c）自检资源库面板与欢迎页：列出内置场景；单击卡片打开场景、当前场景高亮；收藏写入资源库；
//                快速打开按名字筛选并打开；动作卡片打开推荐场景并播放；隐藏（与 --self-test、--library 同用，不给 --scene）
//   --ros2       （M4-06a）打开场景后开启 ROS2 桥接，使用给出的通信配置（*.terra-ros2.json）；构建时没有 ROS2 时提示安装并返回 3
//   --motion     （M3-04c）打开动作文件并从头播放；没有 --scene 时打开动作文件中的推荐场景。动作有错误时返回 4。
//                自检时在第 3 帧打开（代替按播放），并与“动作 → PD 层 → setControl → step”的纯物理对照逐位比较

#include <algorithm>
#include <array>
#include <cctype>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
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
#include "terra/simulation/interpolation.hpp"
#include "terra/simulation/motion.hpp"
#include "terra/simulation/record.hpp"
#include "terra/simulation/lidar.hpp"
#include "terra/simulation/sensor_noise.hpp"
#include "terra/simulation/sensor_sampling.hpp"
#include "terra/library/library.hpp"
#include "terra/core/build_info.hpp"
#include "terra/core/sha256.hpp"
#include "terra/physics/mujoco/backend_info.hpp"
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
#include "camera_sensor.hpp"
#include "import_wizard.hpp"
#include "library_panel.hpp"
#include "terra/library/pack.hpp"
#include "terra/viz/exporter.hpp"
#include "terra/viz/viz_config.hpp"
#include "viz_panel.hpp"
#include "policy_panel.hpp"
#include "terra/policy/controller.hpp"
#include "terra/simulation/lidar3d.hpp"
#include "terra/scene/pose_math.hpp"
#include "scene_view.hpp"
#include "viewer_settings.hpp"
#ifdef TERRA_WITH_ROS2
#include "terra/ros2/bridge.hpp"
#endif

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
    std::string libraryPath;  // M3-07c：资源库文件；空表示默认位置（$TERRA_LIBRARY 或 ~/.local/share/eai-terra/）
    bool recordSelfTest = false;  // M3-07c：自检时测试记录与回放（第 5 帧开始记录，中间停止并回放）
    bool sensorSelfTest = false;  // M4-04：自检时检查传感器采样与预览
    std::string importSelfTestPath;     // M4-08a：自检时导入的模型
    bool sensorRecordSelfTest = false;  // M4-07：自检时检查传感器的记录、复现与回放（与 --self-test、--scene、--library 同用）
    bool librarySelfTest = false;       // M4-08c：自检资源库面板、欢迎页与快速打开
    bool vizSelfTest = false;           // M4-08d：自检显示配置与 ROS2 面板
    std::string ros2Profile;      // M4-06a：开启 ROS2 桥接的通信配置
    bool demoDrive = false;
    std::string policyPath;               // M4-10a：打开场景后运行的策略（.terra-policy.json 或 .onnx）
    std::optional<std::array<float, 3>> policyCommand;  // M4-10a：策略的速度指令
    bool policySelfTest = false;          // M4-10a：自检策略的运行、速度指令面板与停止
    std::string policyImportSelfTest;     // M4-10b：自检导入向导（给出要导入的文件）
    std::string importPolicy;             // M4-10b：启动后打开导入向导并选好这个文件
    bool lidar3dSelfTest = false;         // M5-11：3D 雷达自检
    bool thermalSelfTest = false;         // M5-06：热像仪自检
    bool elementsSelfTest = false;        // M5-02a：场景元素与元素卡片的预览
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
        } else if (a == "--library" && hasValue) {
            o.libraryPath = argv[++i];
        } else if (a == "--record-self-test") {
            o.recordSelfTest = true;
        } else if (a == "--sensor-self-test") {
            o.sensorSelfTest = true;
        } else if (a == "--import-self-test" && hasValue) {
            o.importSelfTestPath = argv[++i];
        } else if (a == "--sensor-record-self-test") {
            o.sensorRecordSelfTest = true;
        } else if (a == "--library-self-test") {
            o.librarySelfTest = true;
        } else if (a == "--viz-self-test") {
            o.vizSelfTest = true;
        } else if (a == "--ros2" && hasValue) {
            o.ros2Profile = argv[++i];
        } else if (a == "--policy" && hasValue) {
            o.policyPath = argv[++i];
        } else if (a == "--policy-command" && hasValue) {
            std::array<float, 3> c{};
            if (std::sscanf(argv[++i], "%f,%f,%f", &c[0], &c[1], &c[2]) != 3) return false;
            o.policyCommand = c;
        } else if (a == "--policy-self-test") {
            o.policySelfTest = true;
        } else if (a == "--lidar3d-self-test") {
            o.lidar3dSelfTest = true;
        } else if (a == "--thermal-self-test") {
            o.thermalSelfTest = true;
        } else if (a == "--elements-self-test") {
            o.elementsSelfTest = true;
        } else if (a == "--import-policy" && hasValue) {
            o.importPolicy = argv[++i];
        } else if (a == "--policy-import-self-test" && hasValue) {
            o.policyImportSelfTest = argv[++i];
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
    std::string errorTitle;  // 错误弹窗的标题（场景、动作或记录）
    // M3-07c：资源库与实验记录
    std::unique_ptr<terra::library::Library> library;
    std::string libraryPath;
    // M3-07d：底部“时间轴”标签页（与日志、统计并列），记录列表合并在其中
    bool showTimeline = true;
    bool focusTimeline = false;    // 下一帧把“时间轴”标签页切到前面（菜单 文件 → 实验记录…）
    bool recordsOnlyScene = true;
    bool recordListStale = true;   // 切换场景、保存记录后在下一次绘制时刷新
    std::array<char, 64> actuatorFilter{};  // 执行器面板的名字过滤
    bool showSensors = true;  // M4-04：传感器面板
    bool sensorNoise = true;  // M4-09：传感器噪声的运行时开关（只影响本次运行；场景文件中没有配置噪声时无效）
    int sensorImu = 0;        // 传感器面板中显示曲线的 IMU
    // 执行器面板本帧显示的实测与目标（自检用：回放时应等于记录中的值）
    std::map<std::uint32_t, std::pair<double, std::optional<double>>> actuatorShown;
    std::uint64_t actuatorShownStep = 0;  // 回放时面板对应的步序号
    bool selfTestPressRecord = false;     // 自检：本帧在工具栏上按一下录制按钮
    std::vector<terra::library::RecordEntry> recordList;
    std::optional<std::string> replayRequest;            // 请求回放的记录（帧末处理：可能需要切换场景）
    std::string ros2Profile;  // M4-06a：非空表示开启 ROS2 桥接（打开其他场景时以同一配置重新开启）
    vw::ImportWizardState importWizard;  // M4-08a：导入向导
    bool importWizardSeen = false;       // M4-08a 自检：向导窗口出现过
    bool recordCameraImages = false;  // M4-07：记录时同时保存相机图像（实际渲染的帧）到记录旁的 .frames 目录
    std::string lastRecordPath;                          // 最近一次保存的记录（自检用）
    // M4-08c：资源库面板、欢迎页与快速打开。资源库文件还不存在时（或自检没有给 --library 时）资产列在内存中的临时库里
    vw::LibraryPanelState libraryPanel;
    bool showLibrary = true;
    bool showWelcome = true;  // 没有打开场景时视口中显示欢迎页（本次会话可关闭）
    bool libraryMemoryOnly = false;  // 自检且没有给 --library：只用内存中的库，不读写用户目录
    std::unique_ptr<terra::library::Library> memoryLibrary;
    std::vector<terra::library::AssetEntry> assetList;
    bool assetsStale = true;
    std::size_t assetScans = 0;
    std::string activeLibraryFile;  // 当前使用的库文件（位置改变时重新打开）
    std::vector<terra::library::ScanRoot> extraScanRoots;  // 自检：另扫描测试夹具
    ui::FileBrowser packBrowser;
    // M4-08d：显示配置（ROS2 面板、消息、曲线、图像、TF 树窗口）
    vw::VizPanelState viz;
    vw::PolicyPanelState policy;  // M4-10a：运行策略、组件、速度指令面板
    struct PolicyTest {
        bool started = false, panel = false, commanded = false, stopped = false;
        std::uint64_t inferences = 0;
        double forward = 0.0;
    } policyTest;
    bool pointCloudOverlay = true;      // M5-11：在视口中叠加 3D 雷达的点云
    std::size_t overlayPointsDrawn = 0;  // 自检：本帧在视口中画出的点数
    std::uint64_t thermalFramesShown = 0;  // M5-06 自检：创建过的热像预览次数
    std::optional<std::string> pendingPolicy;  // M4-10b：打开场景后运行的策略（资源库中的策略卡片、导入并运行）
    struct PolicyImportTest {
        bool opened = false, scanned = false, bestFull = false, extraWarned = false, imported = false, registered = false, componentAsked = false;
        bool sceneOpened = false;
        int stage = 0;
        int imports = 0, importsBeforeEmpty = -1;  // 导入完成的次数；菜单重开（未选文件）前的次数
        std::string policyFile;
    } policyImportTest;
    ui::FileBrowser vizBrowser;
    std::optional<std::pair<terra::viz::VizConfig, std::string>> pendingDisplay;  // 打开场景后应用：配置（导入）或文件（以某一份打开）
    struct VizTest {
        int phase = 0;
        double phaseStart = 0.0;
        bool ros2 = false;          // 本次自检含 ROS2（--ros2 开启了桥接）
        std::string imuTopic, imageTopic;
        bool simPlot = false, message = false, image = false, plot = false, tf = false, saved = false, registered = false;
        bool restored = false, exported = false, rvizHasImage = false;
        std::size_t savedWindows = 0;
        std::string savedPath;
    } vizTest;
    struct LibraryTest {
        std::size_t welcomeCards = 0, panelCards = 0, builtinScenes = 0, quickMatches = 0;
        bool cardOpened = false, highlighted = false, favoriteSaved = false, quickOpened = false, motionPlayed = false;
        bool hidden = false, welcomeAgain = false;
    } libraryTest;
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
    // M4-08c：左侧加宽（原 17%），上方为资源库（场景卡片），下方为大纲
    ImGuiID left = ImGui::DockBuilderSplitNode(center, ImGuiDir_Left, 0.20f, nullptr, &center);
    const ImGuiID leftTop = ImGui::DockBuilderSplitNode(left, ImGuiDir_Up, 0.58f, nullptr, &left);
    // M4-08a（用户 2026-11-25）：右侧与底部稍宽（原 24%、24%）
    const ImGuiID right = ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, 0.28f, nullptr, &center);
    const ImGuiID bottom = ImGui::DockBuilderSplitNode(center, ImGuiDir_Down, 0.26f, nullptr, &center);
    ImGui::DockBuilderDockWindow("###Library", leftTop);
    ImGui::DockBuilderDockWindow("###Outliner", left);
    ImGui::DockBuilderDockWindow("###Details", right);
    ImGui::DockBuilderDockWindow("###Actuators", right);
    ImGui::DockBuilderDockWindow("###Sensors", right);
    ImGui::DockBuilderDockWindow("###Log", bottom);
    ImGui::DockBuilderDockWindow("###Stats", bottom);
    ImGui::DockBuilderDockWindow("###Timeline", bottom);
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
    // M3-07c：实验记录（正在记录时非空）与回放（回放时非空；回放期间不推进物理）
    std::unique_ptr<terra::simulation::RecordWriter> recorder;
    std::uint64_t recordSampleEvery = 10;
    double recordStartTime = 0.0;  // 开始记录时的仿真时间（工具栏的录制时长、时间轴上的录制区段）
    struct Replay {
        terra::simulation::Record record;
        std::string path;
        double time = 0.0;      // 回放时间（s，相对记录开始）
        double duration = 0.0;  // 最后一份状态的时间
        bool playing = false;
        double speed = 1.0;
    };
    std::optional<Replay> replay;

    // M4-04：传感器预览。每完成一步，按各传感器的频率在对应的那一步采样（与 Headless、terra_capture 相同的采样时刻）：
    // IMU 记入曲线；雷达在该步做射线检测；相机只保存该步的快照，本帧结束前渲染（预览只渲染最新的一份）。
    // 回放期间不推进物理，传感器也不更新（传感器的记录在 M4-07）。
    struct ImuSample {
        double time = 0.0;
        std::array<float, 6> values{};  // 比力 xyz、角速度 xyz
    };
    struct CameraPreview {
        std::shared_ptr<const terra::data::PhysicsSnapshot> pending;
        std::optional<vw::CameraFrame> frame;
        terra::render::Texture2D color;
        terra::render::Texture2D depth;
        terra::render::Texture2D segmentation;  // M4-09
        terra::render::Texture2D thermal;       // M5-06：伪彩色
        std::uint64_t rendered = 0;
        bool pendingRecorded = false;  // M4-07：pending 这一步的元数据已写入记录（图像可以随后写入）
        std::string shownImage;        // M4-07：回放时显示的图像文件（避免每帧重读）
    };
    static constexpr std::size_t kImuHistory = 2500;
    std::vector<std::deque<ImuSample>> imuHistory;  // 与模型描述的 sensors 一一对应
    std::vector<std::uint64_t> imuSamples;          // 累计采样数（自检用）
    std::vector<std::optional<terra::simulation::LidarScan>> lidarScans;
    std::vector<std::uint64_t> lidarSamples;
    std::vector<std::optional<terra::simulation::Lidar3dScan>> lidar3dScans;  // M5-11
    std::vector<std::uint64_t> lidar3dSamples;
    std::vector<CameraPreview> cameraPreviews;
    std::unique_ptr<vw::CameraRenderer> cameraRenderer;  // 场景中有相机时才创建
    terra::simulation::SensorNoise noise;                // M4-09：传感器噪声（按场景文件；面板中的开关可关闭）
    // M4-07：回放时传感器面板显示记录中的读数（回放时间变化时更新）
    double replaySensorTime = std::numeric_limits<double>::quiet_NaN();
    bool replaySensorsShown = false;

    void resetSensors() {
        const std::size_t imus = instance.world->description().sensors.size();
        imuHistory.assign(imus, {});
        imuSamples.assign(imus, 0);
        lidarScans.assign(instance.lidars.size(), std::nullopt);
        lidarSamples.assign(instance.lidars.size(), 0);
        lidar3dScans.assign(instance.lidars3d.size(), std::nullopt);
        lidar3dSamples.assign(instance.lidars3d.size(), 0);
        cameraPreviews.resize(instance.cameras.size());
        for (auto& c : cameraPreviews) c.pending.reset();
    }
    double imuRate(std::size_t k) const {
        for (const auto& c : instance.sensors) {
            if (c.sensor.value == k + 1) return c.rateHz;
        }
        return 0.0;
    }
    // 每完成一步（以及打开、重置之后的初始状态）：记录与传感器采样
    void observe(const terra::data::PhysicsSnapshot& snapshot) {
        if (recorder && snapshot.stepIndex % recordSampleEvery == 0) recorder->state(noise.withNoise(snapshot));  // M4-09：带噪声的读数
        const double dt = snapshot.fixedTimestep;
        for (std::size_t k = 0; k < imuHistory.size() && k < snapshot.sensors.size(); ++k) {
            if (!terra::simulation::sensorSampleDue(snapshot.stepIndex, dt, imuRate(k))) continue;
            ImuSample sample;
            sample.time = snapshot.simulationTime;
            auto values = snapshot.sensors[k].values;
            noise.imu(k, snapshot.stepIndex, values);  // M4-09
            for (std::size_t c = 0; c < 6 && c < values.size(); ++c) sample.values[c] = static_cast<float>(values[c]);
            imuHistory[k].push_back(sample);
            if (imuHistory[k].size() > kImuHistory) imuHistory[k].pop_front();
            ++imuSamples[k];
        }
        for (std::size_t k = 0; k < lidarScans.size(); ++k) {
            if (!terra::simulation::sensorSampleDue(snapshot.stepIndex, dt, instance.lidars[k].rateHz)) continue;
            lidarScans[k] = terra::simulation::scanLidar(*instance.world, instance.lidars[k]);
            noise.lidar(k, *lidarScans[k]);  // M4-09
            ++lidarSamples[k];
            if (recorder) recorder->lidar(*lidarScans[k]);  // M4-07
        }
        for (std::size_t k = 0; k < lidar3dScans.size(); ++k) {  // M5-11
            if (!terra::simulation::sensorSampleDue(snapshot.stepIndex, dt, instance.lidars3d[k].rateHz)) continue;
            lidar3dScans[k] = terra::simulation::scanLidar3d(*instance.world, instance.lidars3d[k]);
            noise.lidar3d(k, *lidar3dScans[k]);
            ++lidar3dSamples[k];
            if (recorder) recorder->lidar3d(*lidar3dScans[k]);
        }
        for (std::size_t k = 0; k < cameraPreviews.size(); ++k) {
            if (!terra::simulation::sensorSampleDue(snapshot.stepIndex, dt, instance.cameras[k].rateHz)) continue;
            cameraPreviews[k].pending = std::make_shared<terra::data::PhysicsSnapshot>(snapshot);
            cameraPreviews[k].pendingRecorded = recorder != nullptr;
            if (recorder) {  // M4-07：相机元数据（步序号、时间、位姿）；图像在渲染后按需写入
                terra::simulation::RecordCameraFrame meta;
                meta.step = snapshot.stepIndex;
                meta.time = snapshot.simulationTime;
                meta.name = instance.cameras[k].name;
                meta.pose = terra::scene::cameraWorldPose(snapshot, instance.cameras[k]);
                recorder->camera(meta);
            }
        }
#ifdef TERRA_WITH_ROS2
        if (ros2) ros2->publish(snapshot);  // M4-06a：与 Headless 相同，每步之后与初始状态
#endif
    }
    // 重置、打开之后：从初始状态重新开始
    void restartSensors() {
        resetSensors();
        observe(*instance.world->snapshot());
    }

    // 回放时刻 t 的实体位姿：落在采样点上时直接取该份状态（逐位相同），否则在相邻两份之间插值
    std::vector<terra::data::Pose> replayPoses() const {
        const auto& states = replay->record.states;
        std::vector<terra::data::Pose> poses;
        if (states.empty()) return poses;
        const double t = replay->time;
        auto next = std::lower_bound(states.begin(), states.end(), t,
                                     [](const terra::data::PhysicsSnapshot& s, double at) { return s.simulationTime < at; });
        if (next == states.end()) next = states.end() - 1;
        if (next->simulationTime == t || next == states.begin()) {
            for (const auto& e : next->entities) poses.push_back(e.pose);
            return poses;
        }
        const auto& older = *(next - 1);
        const double span = next->simulationTime - older.simulationTime;
        const double alpha = span > 0 ? (t - older.simulationTime) / span : 1.0;
        return terra::simulation::interpolatePoses(older, *next, std::clamp(alpha, 0.0, 1.0));
    }
    // 回放时刻 t 的关节状态（M3-07d，执行器面板用；记录中的状态不含执行器输出）：落在采样点上时就是该份状态，
    // 否则在相邻两份之间对关节位置与速度逐分量线性插值（面板只用到铰链与滑动关节的第一个分量）
    terra::data::PhysicsSnapshot replaySnapshot() const {
        const auto& states = replay->record.states;
        const double t = replay->time;
        auto next = std::lower_bound(states.begin(), states.end(), t,
                                     [](const terra::data::PhysicsSnapshot& s, double at) { return s.simulationTime < at; });
        if (next == states.end()) next = states.end() - 1;
        if (next->simulationTime == t || next == states.begin()) return *next;
        const auto& older = *(next - 1);
        const double span = next->simulationTime - older.simulationTime;
        const double alpha = std::clamp(span > 0 ? (t - older.simulationTime) / span : 1.0, 0.0, 1.0);
        terra::data::PhysicsSnapshot s = older;
        s.simulationTime = t;
        const auto lerp = [alpha](double a, double b) { return a + (b - a) * alpha; };
        for (std::size_t j = 0; j < s.joints.size() && j < next->joints.size(); ++j) {
            auto& js = s.joints[j];
            for (std::size_t k = 0; k < js.position.size(); ++k) js.position[k] = lerp(older.joints[j].position[k], next->joints[j].position[k]);
            for (std::size_t k = 0; k < js.velocity.size(); ++k) js.velocity[k] = lerp(older.joints[j].velocity[k], next->joints[j].velocity[k]);
        }
        return s;
    }
    // 回放时刻 t 对应的步序号：落在采样点上时就是该份状态的步序号，否则为前一份的步序号加上经过的整步数
    std::uint64_t replayStep() const {
        const auto& states = replay->record.states;
        const double t = replay->time;
        auto next = std::lower_bound(states.begin(), states.end(), t,
                                     [](const terra::data::PhysicsSnapshot& s, double at) { return s.simulationTime < at; });
        if (next == states.end()) return states.back().stepIndex;
        if (next->simulationTime == t || next == states.begin()) return next->stepIndex;
        const auto& older = *(next - 1);
        const double dt = replay->record.header.timestep;
        if (dt <= 0.0) return older.stepIndex;
        const auto passed = static_cast<std::uint64_t>(std::floor((t - older.simulationTime) / dt + 1e-9));
        return std::min(older.stepIndex + passed, next->stepIndex);
    }
    std::unique_ptr<vw::SceneView> view;
    bool demoDrive = false;
    std::string path;               // 场景文件（绝对路径）
    double realtimeFactor = 0.0;    // 仿真时间 / 墙钟时间（平滑）
    int speedIndex = 2;             // kSimSpeeds 下标
    // M3-03：显示哪些 MJCF 几何体分组（默认 0–2，Menagerie 的可见几何在 2、碰撞几何在 3），是否叠加碰撞几何体
    std::array<bool, 6> groups{true, true, true, false, false, false};
    bool collision = false;
    // M4-10a：正在运行的策略（接管对应的执行器；同一执行器以策略为准）
    std::unique_ptr<terra::policy::PolicyController> policy;
    bool velocitySubscribed = false;  // M4-10b：已向桥接订阅 /cmd_vel（策略的速度指令）
#ifdef TERRA_WITH_ROS2
    // M4-06a：ROS2 桥接（开启时非空）。引用 instance 与 pd，放在最后，最先析构
    std::unique_ptr<terra::ros2::Bridge> ros2;
#endif

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
#ifdef TERRA_WITH_ROS2
        if (ros2) commands = terra::ros2::mergeCommands(std::move(commands), ros2->takeCommands());  // M4-06a：桥接的命令在后
#endif
        if (!pd.empty()) commands = pd.process(std::move(commands), *instance.world->snapshot());
#ifdef TERRA_WITH_ROS2
        // M4-06b：宇树 LowCmd 换算的力矩在 PD 层之后，同一执行器以它为准（PD 层让位）
        if (ros2) commands = terra::ros2::mergeCommands(std::move(commands), ros2->directCommands());
#endif
        if (policy && !policy->failed()) commands = terra::policy::overrideCommands(std::move(commands), policy->commands(*instance.world->snapshot()));
        return commands;
    }

    std::vector<r::RenderItem> items(const st::RenderSettings& rs, terra::data::EntityId highlight) const {
        vw::SceneViewOptions options;
        options.armOcclusion = rs.assetArmOcclusion;
        options.highlight = highlight;
        options.groups = groups;
        options.collision = collision;
        return view->items(instance, terra::scene::visualPoses(instance, replay ? replayPoses() : runner->displayPoses()), options);
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

void stopRecording(Frame& f, const char* reason);

void closeScene(Frame& f) {
    if (!f.simulated) return;
    stopRecording(f, "stopped");
    f.simulated.reset();
    f.state.selectedEntity = {};
    f.state.sceneEnvironment.reset();
    terra::core::setFatalContext("场景文件", "");
    f.state.log.push_back(tr(TextId::LogSceneClosed));
    ensureDemoAssets(f);
}

// 打开场景：成功时替换当前场景（打开后暂停，按播放开始）；失败时保留当前场景，弹窗列出诊断。
void startRos2(Frame& f);
void restoreDisplay(Frame& f);  // M4-08d
void stopPolicy(Frame& f);      // M4-10a
void runPolicyFile(Frame& f, const std::string& policyFile, const std::string& scene);  // M4-10b

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
    stopRecording(f, "stopped");
    f.simulated.reset();  // 先释放旧场景的 GPU 资源
    f.simulated.emplace();
    SimulatedScene& sim = *f.simulated;
    sim.instance = std::move(*result.scene);
    sim.path = path;
    f.state.recordListStale = true;  // 时间轴默认只列当前场景的记录
    sim.noise = terra::simulation::SensorNoise(sim.instance, f.state.sensorNoise);  // M4-09
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
        // 每步之后：实验记录与传感器采样（M4-04 起常驻，开始 / 停止记录时不再替换）
        sim.runner->setStepObserver([self](const terra::data::PhysicsSnapshot& snapshot) { self->observe(snapshot); });
    }
    sim.restartSensors();
    if (!sim.instance.cameras.empty() || !sim.instance.lidars.empty() || !sim.instance.world->description().sensors.empty()) {
        std::printf("INFO 场景：传感器 IMU %zu 个、相机 %zu 个、2D 激光雷达 %zu 个（窗口 → 传感器）\n",
                    sim.instance.world->description().sensors.size(), sim.instance.cameras.size(), sim.instance.lidars.size());
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
    startRos2(f);  // M4-06a：桥接开着时，新场景以同一配置重新开启
    restoreDisplay(f);  // M4-08d：恢复该场景上次使用的显示配置
    return true;
}

// ---- 实验记录（M3-07c）----

constexpr const char* kRecordSuffix = ".terra-record.jsonl";

// 库文件：命令行 --library 优先，其次设置中的资源库位置（M4-08c），否则默认位置
std::string libraryFilePath(const Frame& f) {
    if (!f.state.libraryPath.empty()) return f.state.libraryPath;
    if (!f.settings.current.ui.libraryLocation.empty()) {
        return (std::filesystem::path{f.settings.current.ui.libraryLocation} / "library.sqlite3").string();
    }
    return terra::library::defaultLibraryPath();
}

terra::library::Library* recordLibrary(Frame& f) {
    if (!f.state.library) {
        std::string error, warning;
        const std::string path = libraryFilePath(f);
        f.state.library = terra::library::Library::open(path, error, &warning);
        if (!warning.empty()) f.state.log.push_back(warning);
        if (!f.state.library) f.state.log.push_back(std::string{tr(TextId::LogRecordIndexFailed)} + error);
    }
    return f.state.library.get();
}

std::string recordsDirectory(Frame& f) {
    if (auto* lib = recordLibrary(f)) return (std::filesystem::path{lib->path()}.parent_path() / "records").string();
    return terra::library::defaultRecordsDirectory();
}

// M4-08a：导入目录的上级：资源库目录下的 assets/（与记录 records/ 并列）
std::string importDirectory(const Frame& f) {
    return (std::filesystem::path{libraryFilePath(f)}.parent_path() / "assets").string();
}

void refreshRecordList(Frame& f) {
    f.state.recordList.clear();
    // 只是列出时不新建资源库（M3-08）：还没有库文件时列表为空。时间轴一打开就会刷新列表，
    // 否则每次启动查看器（包括自检）都会在用户目录中建库；保存记录或“重新扫描”时才建立
    std::error_code ec;
    if (!f.state.library && !std::filesystem::exists(libraryFilePath(f), ec)) return;
    if (auto* lib = recordLibrary(f)) {
        f.state.recordList = lib->records(f.state.recordsOnlyScene && f.simulated ? f.simulated->path : std::string{});
    }
}

// ---- 资源库中的资产（M4-08c）----

// 列出资产用的库：资源库文件已存在（或命令行给了 --library）时用它；否则用内存中的临时库，不在用户目录中建库
terra::library::Library* assetLibrary(Frame& f) {
    std::error_code ec;
    if (!f.state.libraryMemoryOnly && (f.state.library || !f.state.libraryPath.empty() || std::filesystem::exists(libraryFilePath(f), ec))) {
        if (auto* lib = recordLibrary(f)) return lib;
    }
    if (!f.state.memoryLibrary) {
        std::string error;
        f.state.memoryLibrary = terra::library::Library::open(":memory:", error);
        if (!f.state.memoryLibrary) f.state.log.push_back(std::string{tr(TextId::LogRecordIndexFailed)} + error);
    }
    return f.state.memoryLibrary.get();
}

std::vector<terra::library::ScanRoot> assetScanRoots(const Frame& f) {
    auto roots = terra::library::defaultScanRoots(TERRA_ASSETS_DIR, f.state.libraryMemoryOnly ? std::string{":memory:"} : libraryFilePath(f));
    roots.insert(roots.end(), f.state.extraScanRoots.begin(), f.state.extraScanRoots.end());
    return roots;
}

// 扫描（增量）并刷新卡片列表
void refreshAssets(Frame& f) {
    f.state.assetsStale = false;
    auto* lib = assetLibrary(f);
    if (lib == nullptr) {
        f.state.assetList.clear();
        return;
    }
    const auto t0 = std::chrono::steady_clock::now();
    const auto scan = lib->scanAssets(assetScanRoots(f));
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    f.state.assetList = lib->assets();
    ++f.state.assetScans;
    char time[32];
    std::snprintf(time, sizeof time, "%.1f ms", ms);
    const std::string text = std::to_string(scan.files) + " 个文件（解析 " + std::to_string(scan.updated) + "、未变 " + std::to_string(scan.unchanged) +
                             "）、" + std::to_string(scan.robots) + " 个机器人、" + std::to_string(scan.errors.size()) + " 个错误，" + time + "（" +
                             (lib->inMemory() ? std::string{"内存中的临时库"} : lib->path()) + "）";
    std::printf("INFO 资源库扫描：%s\n", text.c_str());
    f.state.log.push_back(std::string{tr(TextId::LogLibraryScanned)} + text);
    for (const auto& e : scan.errors) std::printf("INFO 资源库扫描：%s\n", e.c_str());
}

// 需要写入（收藏、隐藏、导入登记）时用的库：需要时建立资源库文件，并把内存中的临时库换成它（自检没有给 --library 时仍在内存中）
terra::library::Library* writableLibrary(Frame& f) {
    if (f.state.libraryMemoryOnly) return assetLibrary(f);
    auto* lib = recordLibrary(f);
    if (lib && f.state.memoryLibrary) {
        f.state.memoryLibrary.reset();
        lib->scanAssets(assetScanRoots(f));
    }
    return lib ? lib : assetLibrary(f);
}

// 资源库位置改变（偏好设置）时重新打开
void syncLibraryLocation(Frame& f) {
    std::error_code ec;
    const std::string wanted = std::filesystem::absolute(libraryFilePath(f), ec).lexically_normal().string();
    f.state.settingsWindow.activeLibrary = wanted;
    f.state.settingsWindow.libraryFromCommandLine = !f.state.libraryPath.empty();
    if (wanted == f.state.activeLibraryFile) return;
    if (!f.state.activeLibraryFile.empty()) {
        f.state.library.reset();
        f.state.memoryLibrary.reset();
        f.state.recordListStale = true;
        f.state.log.push_back(std::string{tr(TextId::LogLibraryLocation)} + wanted);
        std::printf("INFO 资源库位置：%s\n", wanted.c_str());
    }
    f.state.activeLibraryFile = wanted;
    f.state.assetsStale = true;
}

// 偏好设置中的“复制到新位置”：复制库文件、records/ 与 assets/，成功后改用新位置
void copyLibraryIfRequested(Frame& f) {
    auto& request = f.state.settingsWindow.copyLibraryRequest;
    if (!request) return;
    const std::string targetDir = *request;
    request.reset();
    const std::string target = (std::filesystem::path{targetDir} / "library.sqlite3").string();
    if (!recordLibrary(f)) return;  // 原位置还没有库文件时先建立
    f.state.library.reset();        // 关闭后再复制
    if (const auto error = terra::library::copyLibrary(f.state.activeLibraryFile, target)) {
        f.state.log.push_back(std::string{tr(TextId::LogLibraryCopyFailed)} + *error);
        std::printf("INFO 复制资源库失败：%s\n", error->c_str());
        return;
    }
    f.state.log.push_back(std::string{tr(TextId::LogLibraryCopied)} + target);
    std::printf("INFO 资源库已复制到：%s\n", target.c_str());
    f.settings.current.ui.libraryLocation = targetDir;
}

vw::LibraryContext libraryContext(const Frame& f) {
    vw::LibraryContext c;
    c.assets = &f.state.assetList;
    if (f.simulated) c.currentScene = f.simulated->path;
    c.recent = f.settings.current.ui.recentScenes;
    return c;
}

// 目录中的第一个场景文件（按名字）
std::string firstSceneIn(const std::string& dir) {
    std::vector<std::string> scenes;
    std::error_code ec;
    for (auto it = std::filesystem::recursive_directory_iterator(dir, ec); !ec && it != std::filesystem::recursive_directory_iterator(); it.increment(ec)) {
        const std::string name = it->path().filename().string();
        if (name.size() > std::strlen(kSceneSuffix) && name.compare(name.size() - std::strlen(kSceneSuffix), std::string::npos, kSceneSuffix) == 0) {
            scenes.push_back(it->path().string());
        }
    }
    std::sort(scenes.begin(), scenes.end());
    return scenes.empty() ? std::string{} : scenes.front();
}

// 解包资产包到资源库目录下的 assets/，登记并打开其中的场景
void importPackFile(Frame& f, const std::string& pack) {
    const auto r = terra::library::importPack(pack, importDirectory(f));
    if (!r.ok) {
        f.state.log.push_back(std::string{tr(TextId::LogPackFailed)} + r.error);
        f.state.sceneErrors = {r.error};
        f.state.errorTitle = "pack";
        f.state.showSceneErrors = true;
        std::printf("INFO 资产包失败：%s\n", r.error.c_str());
        return;
    }
    f.state.log.push_back(std::string{tr(TextId::LogPackImported)} + r.directory);
    std::printf("INFO 资产包已导入：%s（%zu 个文件）\n", r.directory.c_str(), r.files);
    writableLibrary(f);
    f.state.assetsStale = true;
    if (const std::string scene = firstSceneIn(r.directory); !scene.empty()) f.state.openRequest = scene;
}

// ---- 显示配置（M4-08d）----

// 显示配置的目录：资源库目录下的 displays/<场景名>/
std::string displaysDirectory(const Frame& f) { return (std::filesystem::path{libraryFilePath(f)}.parent_path() / "displays").string(); }

std::string sceneStem(const std::string& scene) {
    std::string name = std::filesystem::path{scene}.filename().string();
    const std::string suffix = kSceneSuffix;
    if (name.size() > suffix.size() && name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0) name.resize(name.size() - suffix.size());
    return name.empty() ? std::string{"scene"} : name;
}

// 当前场景的显示配置（资源库中关联到该场景的）
std::vector<std::pair<std::string, std::string>> sceneDisplays(const Frame& f) {
    std::vector<std::pair<std::string, std::string>> out;
    if (!f.simulated) return out;
    for (const auto& a : f.state.assetList) {
        if (a.kind == terra::library::AssetKind::Display && !a.hidden && !a.related.empty() && a.related[0] == f.simulated->path) {
            out.emplace_back(a.title, a.path);
        }
    }
    return out;
}

bool loadDisplay(Frame& f, const std::string& path, bool remember) {
    const auto r = terra::viz::readViz(path);
    for (const auto& w : r.warnings) f.state.log.push_back(std::string{tr(TextId::LogVizLoaded)} + w);
    if (!r.config) {
        for (const auto& e : r.errors) f.state.log.push_back(std::string{tr(TextId::LogVizFailed)} + e);
        return false;
    }
    vw::applyConfig(f.state.viz, *r.config, path);
    if (remember && f.simulated) f.settings.current.ui.sceneDisplays[f.simulated->path] = path;
    f.state.log.push_back(std::string{tr(TextId::LogVizLoaded)} + path);
    std::printf("INFO 显示配置已恢复：%s（%zu 个窗口）\n", path.c_str(), r.config->windows.size());
    return true;
}

// 打开场景后：有待应用的配置（导入、以某一份打开）时用它，否则恢复该场景上次使用的那一份；都没有时为空配置
void restoreDisplay(Frame& f) {
    auto& pending = f.state.pendingDisplay;
    if (pending) {
        const auto p = std::move(*pending);
        pending.reset();
        if (p.second.empty()) {
            vw::applyConfig(f.state.viz, p.first, {});
            f.state.viz.dirty = true;
        } else {
            loadDisplay(f, p.second, true);
        }
        return;
    }
    const auto& map = f.settings.current.ui.sceneDisplays;
    std::error_code ec;
    if (const auto it = map.find(f.simulated->path); it != map.end() && std::filesystem::exists(it->second, ec) && loadDisplay(f, it->second, false)) return;
    terra::viz::VizConfig empty;
    empty.scene = f.simulated->path;
    vw::applyConfig(f.state.viz, std::move(empty), {});
}

// 保存：写入资源库目录下的 displays/<场景名>/<名字>.terviz，登记并记为该场景上次使用的配置
void saveDisplay(Frame& f, std::string name) {
    auto& viz = f.state.viz;
    if (f.state.libraryMemoryOnly) {
        f.state.log.push_back(std::string{tr(TextId::LogVizFailed)} + "自检没有给出 --library，不写用户目录");
        return;
    }
    if (name.empty()) name = viz.config.name.empty() ? std::string{"默认"} : viz.config.name;
    for (char& c : name) {
        if (c == '/' || c == '\\' || c == ':') c = '_';
    }
    std::string path = viz.path;
    if (path.empty() || name != viz.config.name) {
        const std::string scene = f.simulated ? sceneStem(f.simulated->path) : std::string{"_"};
        path = (std::filesystem::path{displaysDirectory(f)} / scene / (name + terra::viz::kVizSuffix)).string();
    }
    viz.config.name = name;
    if (f.simulated) viz.config.scene = f.simulated->path;
    vw::capturePlacement(viz);
    if (const auto error = terra::viz::saveViz(viz.config, path)) {
        f.state.log.push_back(std::string{tr(TextId::LogVizFailed)} + *error);
        return;
    }
    viz.path = path;
    viz.dirty = false;
    if (f.simulated) f.settings.current.ui.sceneDisplays[f.simulated->path] = path;
    writableLibrary(f);
    f.state.assetsStale = true;
    f.state.log.push_back(std::string{tr(TextId::LogVizSaved)} + path);
    std::printf("INFO 显示配置已保存：%s（%zu 个窗口）\n", path.c_str(), viz.config.windows.size());
}

// 导入 .terviz：关联的场景存在且不是当前场景时先打开它；否则关联到当前场景（标为未保存）
void importDisplay(Frame& f, const std::string& file) {
    const auto r = terra::viz::readViz(file);
    if (!r.config) {
        for (const auto& e : r.errors) f.state.log.push_back(std::string{tr(TextId::LogVizFailed)} + e);
        return;
    }
    auto config = *r.config;
    std::error_code ec;
    if (!config.scene.empty() && std::filesystem::exists(config.scene, ec) && (!f.simulated || f.simulated->path != config.scene)) {
        f.state.pendingDisplay = std::make_pair(config, std::string{});
        f.state.openRequest = config.scene;
        return;
    }
    if (f.simulated) config.scene = f.simulated->path;
    vw::applyConfig(f.state.viz, std::move(config), {});
    f.state.viz.dirty = true;
    f.state.log.push_back(std::string{tr(TextId::LogVizLoaded)} + file);
}

// 以资源库中的某一份显示配置打开（场景卡片右键）
void openDisplayAsset(Frame& f, const std::string& file) {
    const auto r = terra::viz::readViz(file);
    if (!r.config || r.config->scene.empty()) {
        importDisplay(f, file);
        return;
    }
    if (f.simulated && f.simulated->path == r.config->scene) {
        loadDisplay(f, file, true);
        return;
    }
    f.state.pendingDisplay = std::make_pair(terra::viz::VizConfig{}, file);
    f.state.openRequest = r.config->scene;
}

// 仿真来源的曲线（IMU 的比力与角速度）
vw::SimPlotSource simPlotSource(Frame& f) {
    vw::SimPlotSource source;
    Frame* frame = &f;
    source.sensors = [frame] {
        std::vector<std::string> names;
        if (frame->simulated) {
            for (const auto& s : frame->simulated->instance.world->description().sensors) names.push_back(s.name);
        }
        return names;
    };
    source.samples = [frame](const std::string& sensor, const std::string& field) {
        std::vector<std::pair<double, double>> out;
        if (!frame->simulated) return out;
        const auto& sensors = frame->simulated->instance.world->description().sensors;
        static const char* const kFields[6] = {"acc.x", "acc.y", "acc.z", "gyro.x", "gyro.y", "gyro.z"};
        std::size_t component = 6;
        for (std::size_t c = 0; c < 6; ++c) {
            if (field == kFields[c]) component = c;
        }
        for (std::size_t k = 0; k < sensors.size() && k < frame->simulated->imuHistory.size() && component < 6; ++k) {
            if (sensors[k].name != sensor) continue;
            for (const auto& s : frame->simulated->imuHistory[k]) out.emplace_back(s.time, static_cast<double>(s.values[component]));
        }
        return out;
    };
    return source;
}

// 传感器面板：把某个 IMU 的角速度弹出为曲线窗口（随显示配置保存）
void popOutImuPlot(Frame& f, const std::string& imu) {
    for (const char* field : {"gyro.x", "gyro.y", "gyro.z"}) {
        terra::viz::PlotSeries series;
        series.source = "sim";
        series.sensor = imu;
        series.field = field;
        vw::addSeries(f.state.viz, series, imu);
    }
}

void handleVizActions(Frame& f, const std::vector<vw::VizAction>& actions) {
    for (const auto& a : actions) {
        switch (a.type) {
            case vw::VizAction::Type::Save: saveDisplay(f, f.state.viz.config.name); break;
            case vw::VizAction::Type::SaveAs: saveDisplay(f, a.text); break;
            case vw::VizAction::Type::Switch: loadDisplay(f, a.text, true); break;
            case vw::VizAction::Type::Import: f.state.vizBrowser.open(displaysDirectory(f), terra::viz::kVizSuffix, ""); break;
            case vw::VizAction::Type::CloseAll: {
                f.state.viz.config.windows.clear();
                f.state.viz.dirty = true;
                break;
            }
        }
    }
}

void vizPanels(Frame& f) {
    const vw::SimPlotSource sim = simPlotSource(f);
    vw::VizContext ctx;
    if (f.simulated) ctx.currentScene = f.simulated->path;
    ctx.displays = sceneDisplays(f);
    ctx.defaultExportDir = (std::filesystem::path{libraryFilePath(f)}.parent_path() / "exports").string();
    ctx.sim = &sim;
    handleVizActions(f, vw::drawVizWindows(f.state.viz, ctx));
    if (auto chosen = f.state.vizBrowser.draw(label(TextId::VizImportBrowserTitle, "VizBrowser").c_str())) importDisplay(f, *chosen);
}

bool ros2PanelOpen(const Frame& f) {
    for (const auto& w : f.state.viz.config.windows) {
        if (w.type == terra::viz::WindowType::Topics) return true;
    }
    return false;
}

void toggleRos2Panel(Frame& f, bool open) {
    auto& windows = f.state.viz.config.windows;
    if (open) {
        terra::viz::VizWindow w;
        w.type = terra::viz::WindowType::Topics;
        vw::openWindow(f.state.viz, w);
    } else {
        windows.erase(std::remove_if(windows.begin(), windows.end(), [](const terra::viz::VizWindow& w) { return w.type == terra::viz::WindowType::Topics; }),
                      windows.end());
        f.state.viz.dirty = true;
    }
}

// M5-02a：元素卡片的预览——生成一个只有地面与这个元素的场景（放在资源库目录的 previews/ 下；内存中的库放在系统临时目录），
// 相机按碰撞体的大小取景。返回场景文件路径；失败时为空，error 为原因
std::string elementPreviewScene(const Frame& f, const std::string& elementPath, std::string& error) {
    namespace fs = std::filesystem;
    const auto parsed = terra::scene::readElementFile(elementPath);
    if (!parsed.document) {
        error = parsed.issues.empty() ? elementPath : parsed.issues.front().where + " " + parsed.issues.front().message;
        return {};
    }
    const auto& e = *parsed.document;
    double radius = 0.5, top = 0.5;
    for (const auto& b : e.boxes) {
        const double r = std::sqrt(b.pose.position.x * b.pose.position.x + b.pose.position.y * b.pose.position.y) +
                         std::max({b.halfSize.x, b.halfSize.y, b.halfSize.z});
        radius = std::max(radius, r);
        top = std::max(top, b.pose.position.z + b.halfSize.z);
    }
    std::error_code ec;
    const fs::path dir = f.state.libraryMemoryOnly ? fs::temp_directory_path(ec) / "terra_element_previews" : fs::path{libraryFilePath(f)}.parent_path() / "previews";
    fs::create_directories(dir, ec);
    std::string stem = fs::path{elementPath}.filename().string();
    if (const auto at = stem.find(".terra-element.json"); at != std::string::npos) stem = stem.substr(0, at);
    const fs::path scene = dir / (stem + ".preview.terra-scene.json");
    const auto quote = [](const std::string& text) {
        std::string q = "\"";
        for (const char c : text) {
            if (c == '"' || c == '\\') q += '\\';
            q += c;
        }
        return q + "\"";
    };
    const auto number = [](double v) {
        char buf[32];
        std::snprintf(buf, sizeof buf, "%.4f", v);
        return std::string{buf};
    };
    const double d = 2.2 * radius + 0.8;
    const std::string json = "{\n  \"format\": \"terra.scene\",\n  \"version\": 1,\n  \"name\": " + quote("元素预览：" + e.name) + ",\n  \"title\": " +
                             quote(e.name) + ",\n  \"models\": [{\"name\": \"floor\", \"mjcf\": " +
                             quote((fs::path{TERRA_ASSETS_DIR} / "scenes/common/floor.xml").string()) + "}],\n  \"elements\": [{\"id\": \"preview\", \"element\": " +
                             quote(fs::absolute(elementPath, ec).lexically_normal().string()) + "}],\n  \"view\": {\"camera\": {\"position\": [" + number(d) +
                             ", " + number(-d) + ", " + number(top + 0.6 * d) + "], \"target\": [0, 0, " + number(0.5 * top) + "]}}\n}\n";
    std::ofstream out(scene, std::ios::binary);
    out << json;
    if (!out) {
        error = "无法写入 " + scene.string();
        return {};
    }
    return scene.string();
}

void handleLibraryAction(Frame& f, const vw::LibraryAction& a) {
    using T = vw::LibraryAction::Type;
    namespace lib = terra::library;
    switch (a.type) {
        case T::Open:
            if (a.kind == lib::AssetKind::Motion) {
                f.state.motionRequest = a.path;
            } else if (a.kind == lib::AssetKind::Policy) {  // M4-10b：打开推荐场景并运行
                std::string scene;
                for (const auto& asset : f.state.assetList) {
                    if (asset.kind == lib::AssetKind::Policy && asset.path == a.path && !asset.related.empty()) scene = asset.related[0];
                }
                runPolicyFile(f, a.path, scene);
            } else if (a.kind == lib::AssetKind::Element) {  // M5-02a：在空场景中预览
                std::string error;
                const std::string scene = elementPreviewScene(f, a.path, error);
                if (scene.empty()) f.state.log.push_back(std::string{tr(TextId::ElementPreviewFailed)} + error);
                else f.state.openRequest = scene;
            } else {
                f.state.openRequest = a.path;
            }
            break;
        case T::ToggleFavorite:
        case T::Hide:
        case T::UnhideAll: {
            auto* store = writableLibrary(f);
            if (store == nullptr) break;
            if (a.type == T::ToggleFavorite) store->setFavorite(a.kind, a.path, a.favorite);
            else if (a.type == T::Hide) store->setHidden(a.kind, a.path, true);
            else {
                for (const auto& e : f.state.assetList) {
                    if (e.kind == a.kind && e.hidden) store->setHidden(e.kind, e.path, false);
                }
            }
            f.state.assetList = store->assets();
            break;
        }
        case T::CopyPath: ImGui::SetClipboardText(a.path.c_str()); break;
        case T::Reveal: vw::revealInFileManager(a.path); break;
        case T::Reimport: {
            // 按导入目录中的配方重新导入到原处（覆盖）
            const std::string recipe = (std::filesystem::path{a.importDirectory} / "import.terra-import.json").string();
            vw::beginImport(f.state.importWizard, recipe, std::filesystem::path{a.importDirectory}.parent_path().string());
            f.state.importWizard.options.overwrite = true;
            break;
        }
        case T::ExportPack: {
            const std::string name = std::filesystem::path{a.importDirectory}.filename().string();
            const std::string out = (std::filesystem::path{libraryFilePath(f)}.parent_path() / "packs" / (name + terra::library::kPackSuffix)).string();
            const auto r = terra::library::exportPack(a.importDirectory, out, true);
            if (r.ok) {
                f.state.log.push_back(std::string{tr(TextId::LogPackExported)} + out);
                std::printf("INFO 资产包已导出：%s（%zu 个文件）\n", out.c_str(), r.files);
                vw::revealInFileManager(out);
            } else {
                f.state.log.push_back(std::string{tr(TextId::LogPackFailed)} + r.error);
            }
            break;
        }
        case T::Rescan: f.state.assetsStale = true; break;
        case T::OpenDisplay: openDisplayAsset(f, a.path); break;  // M4-08d
    }
}

// 结束记录：写摘要、加入资源库。重置、切换场景、开始回放、出错时都会自动结束（一份记录对应一段连续运行）
void stopRecording(Frame& f, const char* reason) {
    if (!f.simulated || !f.simulated->recorder) return;
    SimulatedScene& sim = *f.simulated;
    const std::string path = sim.recorder->path();
    const bool ok = sim.recorder->finish(sim.instance.world->stepIndex(), sim.instance.world->simulationTime(), reason);
    sim.recorder.reset();
    sim.runner->setControlObserver(nullptr);
    if (!ok) {
        f.state.log.push_back(std::string{tr(TextId::LogRecordIndexFailed)} + path);
        return;
    }
    f.state.lastRecordPath = path;
    f.state.log.push_back(std::string{tr(TextId::LogRecordSaved)} + path);
    std::printf("INFO 记录已保存：%s\n", path.c_str());
    if (auto* lib = recordLibrary(f)) {
        if (const auto error = lib->addRecord(path)) f.state.log.push_back(std::string{tr(TextId::LogRecordIndexFailed)} + *error);
    }
    f.state.recordListStale = true;
}

void simReset(Frame& f) {
    if (!f.simulated) return;
    stopRecording(f, "stopped");
    f.simulated->runner->reset();
    f.simulated->restartSensors();
    f.simulated->pd.clearTargets();
    f.simulated->panelTargets.clear();
    if (f.simulated->motion) f.simulated->motion->stop();
    f.simulated->realtimeFactor = 0.0;
}

void simResetToKeyframe(Frame& f, std::size_t index) {
    if (!f.simulated) return;
    stopRecording(f, "stopped");
    if (!f.simulated->runner->resetToKeyframe(index)) return;
    f.simulated->restartSensors();
    f.simulated->pd.clearTargets();
    f.simulated->panelTargets.clear();
    if (f.simulated->motion) f.simulated->motion->stop();
    f.simulated->realtimeFactor = 0.0;
    f.state.log.push_back(std::string{tr(TextId::LogKeyframeReset)} + f.simulated->instance.world->description().keyframes[index].name);
}

// ---- ROS2 桥接（M4-06a）----

constexpr const char* kDefaultRos2Profile = TERRA_ASSETS_DIR "/ros2/profiles/generic.terra-ros2.json";

bool ros2Available() {
#ifdef TERRA_WITH_ROS2
    return true;
#else
    return false;
#endif
}

bool ros2Running(const Frame& f) {
#ifdef TERRA_WITH_ROS2
    return f.simulated && f.simulated->ros2 != nullptr;
#else
    (void)f;
    return false;
#endif
}

// 按 f.state.ros2Profile 为当前场景开启桥接（打开场景后、菜单开启时）；失败时关闭开关并记入日志
void startRos2(Frame& f) {
#ifdef TERRA_WITH_ROS2
    if (!f.simulated || f.state.ros2Profile.empty()) return;
    SimulatedScene& sim = *f.simulated;
    sim.ros2.reset();
    const auto profile = terra::ros2::readProfile(f.state.ros2Profile);
    std::string error;
    for (const auto& e : profile.errors) error += (error.empty() ? "" : "；") + e;
    if (profile.profile) sim.ros2 = terra::ros2::Bridge::create(sim.instance, sim.pd, *profile.profile, error);
    if (sim.ros2) sim.ros2->setSensorNoise(f.state.sensorNoise);  // M4-09
    if (!sim.ros2) {
        std::printf("INFO ROS2 桥接无法开启：%s\n", error.c_str());
        f.state.log.push_back(std::string{tr(TextId::LogRos2Failed)} + error);
        f.state.ros2Profile.clear();
        return;
    }
    std::printf("INFO ROS2 桥接已开启：%s（%s）\n", profile.profile->name.c_str(), f.state.ros2Profile.c_str());
    f.state.log.push_back(std::string{tr(TextId::LogRos2Started)} + profile.profile->name + "（" + f.state.ros2Profile + "）");
    sim.ros2->publish(*sim.instance.world->snapshot());  // 当前状态（含 /tf 与传感器的初始值）
#else
    (void)f;
#endif
}

void stopRos2(Frame& f) {
#ifdef TERRA_WITH_ROS2
    if (f.simulated && f.simulated->ros2) {
        const auto stats = f.simulated->ros2->stats();
        std::printf("INFO ROS2 桥接：发布 %llu 条消息；命令接受 %llu 条、拒绝 %llu 条\n", static_cast<unsigned long long>(stats.published),
                    static_cast<unsigned long long>(stats.commandsAccepted), static_cast<unsigned long long>(stats.commandsRejected));
        f.simulated->ros2.reset();
        f.state.log.push_back(tr(TextId::LogRos2Stopped));
    }
#endif
    f.state.ros2Profile.clear();
}

// 主线程每帧推进之前：执行服务请求（重置与菜单的“重置”相同；暂停 / 继续 / 单步与工具栏相同）。回放期间忽略
void handleRos2Requests(Frame& f) {
#ifdef TERRA_WITH_ROS2
    if (!f.simulated || !f.simulated->ros2) return;
    for (const auto r : f.simulated->ros2->takeRequests()) {
        const char* name = r == terra::ros2::Request::Reset   ? "reset"
                           : r == terra::ros2::Request::Pause ? "pause"
                           : r == terra::ros2::Request::Resume ? "resume"
                                                               : "step";
        if (f.simulated->replay) {
            f.state.log.push_back(std::string{tr(TextId::LogRos2RequestIgnored)} + name);
            continue;
        }
        f.state.log.push_back(std::string{tr(TextId::LogRos2Request)} + name);
        SimulatedScene& sim = *f.simulated;
        if (r == terra::ros2::Request::Reset) simReset(f);
        else if (r == terra::ros2::Request::Pause) sim.runner->pause();
        else if (r == terra::ros2::Request::Resume) sim.runner->resume();
        else if (sim.runner->paused()) sim.runner->stepOnce();
    }
#else
    (void)f;
#endif
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
    stopRecording(f, "stopped");
    SimulatedScene& sim = *f.simulated;
    if (const auto key = sim.motion->motion().startKeyframe) sim.runner->resetToKeyframe(*key);
    if (sim.motion->motion().startKeyframe) sim.restartSensors();  // M4-04：按关键帧重置后传感器从初始状态重新开始
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

// 开始记录：先回到可复现的起点——有动作时从头播放动作（按其起始关键帧重置，没有起始关键帧时先回到场景的初始状态），
// 否则重置到场景的初始状态；然后写记录头与第 0 步的状态。记录文件在资源库旁的 records/ 中。
void startRecording(Frame& f) {
    if (!f.simulated || f.simulated->recorder || f.simulated->replay) return;
    SimulatedScene& sim = *f.simulated;
    std::string startKeyframe;
    if (sim.motion) {
        if (const auto key = sim.motion->motion().startKeyframe) {
            startKeyframe = sim.instance.world->description().keyframes[*key].name;
        } else {
            simReset(f);
        }
        startMotion(f);
    } else {
        simReset(f);
    }
    const auto& d = sim.instance.world->description();
    const double hz = f.settings.current.ui.recordSampleHz;
    sim.recordSampleEvery = std::max<std::uint64_t>(1, static_cast<std::uint64_t>(std::llround(1.0 / (hz * d.timestep))));
    auto header = terra::simulation::makeRecordHeader(d, sim.recordSampleEvery);
    terra::simulation::describeSensors(header, sim.instance);  // M4-07
    header.sensorNoise = sim.noise.active();                    // M4-09
    if (sim.policy) {                                           // M4-10a
        const auto& p = sim.policy->policy();
        header.policyPath = p.path.empty() ? p.model : p.path;
        header.policySha256 = terra::sha256File(header.policyPath).value_or("");
        header.policyModelSha256 = terra::sha256File(p.model).value_or("");
        header.policyBackend = sim.policy->model().describe();
    }
    const terra::BuildInfo& info = terra::buildInfo();
    header.terraVersion = std::string{info.version};
    header.compiler = std::string{info.compilerId} + " " + std::string{info.compilerVersion};
    const auto backend = terra::physics::mujoco::backendInfo();
    header.physicsBackend = std::string{backend.name} + " " + std::string{backend.libraryVersionString};
    header.platform = "linux x86_64";
    {
        const std::time_t now = std::time(nullptr);
        std::tm utc{};
        gmtime_r(&now, &utc);
        char buf[32];
        std::strftime(buf, sizeof buf, "%Y-%m-%dT%H:%M:%SZ", &utc);
        header.created = buf;
        std::strftime(buf, sizeof buf, "%Y%m%d-%H%M%S", &utc);
        const std::string stem = std::filesystem::path{sim.path}.filename().string();
        const std::string base = stem.substr(0, stem.find('.'));
        f.state.lastRecordPath = recordsDirectory(f) + "/" + base + "-" + buf + kRecordSuffix;
    }
    header.scenePath = sim.path;
    header.sceneSha256 = terra::sha256File(sim.path).value_or("");
    if (sim.motion) {
        header.motionPath = sim.motionPath;
        header.motionSha256 = terra::sha256File(sim.motionPath).value_or("");
    }
    header.startKeyframe = startKeyframe;
    std::string error;
    sim.recorder = terra::simulation::RecordWriter::create(f.state.lastRecordPath, header, error);
    if (!sim.recorder) {
        f.state.log.push_back(std::string{tr(TextId::LogRecordIndexFailed)} + error);
        return;
    }
    sim.recorder->state(sim.noise.withNoise(*sim.instance.world->snapshot()));  // M4-09
    // M4-07：起始状态的雷达与相机（restartSensors 在记录开始之前已采过这一步）
    {
        const auto start = sim.instance.world->snapshot();
        terra::simulation::recordSceneSensors(*sim.recorder, *sim.instance.world, sim.instance, *start, &sim.noise);
        for (auto& c : sim.cameraPreviews) {
            if (c.pending && c.pending->stepIndex == start->stepIndex) c.pendingRecorded = true;
        }
    }
    sim.recordStartTime = sim.instance.world->simulationTime();
    SimulatedScene* self = &sim;
    sim.runner->setControlObserver([self](std::uint64_t step, const std::vector<terra::physics::ActuatorCommand>& commands) {
        if (self->recorder) self->recorder->control(step, commands);
    });
    f.state.log.push_back(std::string{tr(TextId::LogRecordStarted)} + f.state.lastRecordPath);
    std::printf("INFO 开始记录：%s（每 %llu 步采样一次）\n", f.state.lastRecordPath.c_str(),
                static_cast<unsigned long long>(sim.recordSampleEvery));
}

// 开始 / 停止记录（M3-07d）：工具栏按钮、时间轴、菜单、Ctrl+Shift+R 与可设置的按键共用；回放期间不可用
void toggleRecording(Frame& f) {
    if (!f.simulated || f.simulated->replay) return;
    if (f.simulated->recorder) stopRecording(f, "stopped");
    else startRecording(f);
}

// 回放：读入记录；场景与当前不同时先打开记录中的场景；回放期间不推进物理，画面由记录的状态给出
bool startReplay(Frame& f, const std::string& path) {
    const auto read = terra::simulation::readRecord(path);
    std::vector<std::string> lines;
    for (const auto& e : read.errors) lines.push_back(std::string{tr(TextId::IssueError)} + " " + e);
    if (!read.record || read.record->states.empty()) {
        if (lines.empty()) lines.push_back(path);
        f.state.sceneErrors = lines;
        f.state.errorTitle = "record";
        f.state.showSceneErrors = true;
        return false;
    }
    for (const auto& w : read.warnings) f.state.log.push_back(w);
    const auto& h = read.record->header;
    if (!f.simulated || f.simulated->path != h.scenePath) {
        if (!openScene(f, h.scenePath)) return false;
    }
    SimulatedScene& sim = *f.simulated;
    stopRecording(f, "stopped");
    if (terra::sha256File(h.scenePath).value_or("") != h.sceneSha256) {
        f.state.log.push_back(std::string{tr(TextId::LogReplaySceneChanged)} + h.scenePath);
    }
    sim.runner->pause();
    if (sim.motion) sim.motion->stop();
    SimulatedScene::Replay replay;
    replay.record = *read.record;
    replay.path = path;
    replay.duration = replay.record.states.back().simulationTime;
    replay.playing = true;
    sim.replay = std::move(replay);
    f.state.log.push_back(std::string{tr(TextId::LogReplayStarted)} + path);
    std::printf("INFO 开始回放：%s（%zu 份状态，%.3f 秒）\n", path.c_str(), sim.replay->record.states.size(), sim.replay->duration);
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

// ---- 图形按钮与播放控制（M3-07d）----
// 按钮上的图形由绘制接口画出，不依赖图标字体；颜色取当前主题，录制与超限用红色。

constexpr ImU32 kRecordRed = IM_COL32(222, 72, 60, 255);

enum class Icon { Play, Pause, Step, Reset, Record, Stop };

bool iconButton(const char* id, Icon icon, bool enabled, const std::string& tooltip) {
    const float size = ImGui::GetFrameHeight();
    ImGui::BeginDisabled(!enabled);
    const bool pressed = ImGui::Button(id, ImVec2(size, size));
    ImGui::EndDisabled();
    const ImVec2 lo = ImGui::GetItemRectMin();
    const ImVec2 hi = ImGui::GetItemRectMax();
    const ImVec2 c((lo.x + hi.x) * 0.5f, (lo.y + hi.y) * 0.5f);
    const float r = size * 0.22f;
    const ImU32 col = ImGui::GetColorU32(enabled ? ImGuiCol_Text : ImGuiCol_TextDisabled);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    switch (icon) {
        case Icon::Play:
            dl->AddTriangleFilled(ImVec2(c.x - r * 0.7f, c.y - r), ImVec2(c.x - r * 0.7f, c.y + r), ImVec2(c.x + r, c.y), col);
            break;
        case Icon::Pause:
            dl->AddRectFilled(ImVec2(c.x - r * 0.8f, c.y - r), ImVec2(c.x - r * 0.25f, c.y + r), col);
            dl->AddRectFilled(ImVec2(c.x + r * 0.25f, c.y - r), ImVec2(c.x + r * 0.8f, c.y + r), col);
            break;
        case Icon::Step:  // ▶|
            dl->AddTriangleFilled(ImVec2(c.x - r, c.y - r), ImVec2(c.x - r, c.y + r), ImVec2(c.x + r * 0.45f, c.y), col);
            dl->AddRectFilled(ImVec2(c.x + r * 0.6f, c.y - r), ImVec2(c.x + r, c.y + r), col);
            break;
        case Icon::Reset:  // |◀
            dl->AddRectFilled(ImVec2(c.x - r, c.y - r), ImVec2(c.x - r * 0.6f, c.y + r), col);
            dl->AddTriangleFilled(ImVec2(c.x + r, c.y - r), ImVec2(c.x + r, c.y + r), ImVec2(c.x - r * 0.45f, c.y), col);
            break;
        case Icon::Record: dl->AddCircleFilled(c, r, enabled ? kRecordRed : col); break;
        case Icon::Stop: dl->AddRectFilled(ImVec2(c.x - r * 0.8f, c.y - r * 0.8f), ImVec2(c.x + r * 0.8f, c.y + r * 0.8f), kRecordRed, 1.5f); break;
    }
    if (!tooltip.empty() && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) ImGui::SetTooltip("%s", tooltip.c_str());
    return pressed && enabled;
}

// 当前窗口内容区右边界（窗口坐标；ImGui 1.93 已去掉 GetWindowContentRegionMax）
float contentRight() { return ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x; }

// 在同一行靠右放宽度为 width 的内容（放不下时紧跟在前一项之后）
void sameLineRight(float width) {
    ImGui::SameLine();
    ImGui::SetCursorPosX(std::max(ImGui::GetCursorPosX(), contentRight() - width));
}

// mm:ss.ss
std::string clockText(double seconds) {
    const double t = std::max(0.0, seconds);
    const int minutes = static_cast<int>(t / 60.0);
    char buf[32];
    std::snprintf(buf, sizeof buf, "%02d:%05.2f", minutes, t - 60.0 * minutes);
    return buf;
}

std::string withKey(TextId id, const char* key) { return std::string{tr(id)} + "    " + key; }

template <std::size_t N>
bool speedCombo(const char* id, double current, const double (&speeds)[N], double& chosen) {
    char text[16];
    std::snprintf(text, sizeof text, "%g×", current);
    ImGui::SetNextItemWidth(ImGui::CalcTextSize("0.25×").x + ImGui::GetFrameHeight() + ImGui::GetStyle().FramePadding.x * 2.0f);
    bool changed = false;
    if (ImGui::BeginCombo(id, text, ImGuiComboFlags_HeightLarge)) {
        for (const double speed : speeds) {
            std::snprintf(text, sizeof text, "%g×", speed);
            if (ImGui::Selectable(text, speed == current)) {
                chosen = speed;
                changed = true;
            }
        }
        ImGui::EndCombo();
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tr(TextId::SimSpeed));
    return changed;
}

constexpr double kReplaySpeeds[] = {0.1, 0.25, 0.5, 1.0, 2.0, 4.0};

// 播放 / 暂停、单步、重置、速度与录制：视口工具栏与时间轴标签页共用（M3-07d 评审问题 2：两处都放）。
// 回放时控制回放；录制按钮在回放时不可用。pressRecord 为真时等同于按下录制按钮（自检用，走与点击完全相同的路径）。
void transportControls(Frame& f, bool pressRecord = false) {
    SimulatedScene& sim = *f.simulated;
    const float gap = ImGui::GetStyle().ItemSpacing.x * 0.35f;
    if (sim.replay) {
        auto& replay = *sim.replay;
        if (iconButton("##play", replay.playing ? Icon::Pause : Icon::Play, true, tr(replay.playing ? TextId::ReplayPause : TextId::ReplayPlay))) {
            if (!replay.playing && replay.time >= replay.duration) replay.time = replay.record.states.front().simulationTime;
            replay.playing = !replay.playing;
        }
        ImGui::SameLine(0, gap);
        iconButton("##step", Icon::Step, false, tr(TextId::SimStep));
        ImGui::SameLine(0, gap);
        if (iconButton("##reset", Icon::Reset, true, tr(TextId::TimelineToStart))) replay.time = replay.record.states.front().simulationTime;
        ImGui::SameLine();
        double speed = replay.speed;
        if (speedCombo("##speed", replay.speed, kReplaySpeeds, speed)) replay.speed = speed;
    } else {
        const bool paused = sim.runner->paused();
        if (iconButton("##play", paused ? Icon::Play : Icon::Pause, true, withKey(paused ? TextId::SimPlay : TextId::SimPause,
                                                                               in::keyName(f.settings.current.keys[st::Command::SimPlayPause][0])))) {
            sim.togglePlay();
        }
        ImGui::SameLine(0, gap);
        if (iconButton("##step", Icon::Step, paused, tr(TextId::SimStep))) sim.runner->stepOnce();
        ImGui::SameLine(0, gap);
        if (iconButton("##reset", Icon::Reset, true, withKey(TextId::SimReset, "Ctrl+R"))) simReset(f);
        ImGui::SameLine();
        double speed = SimulatedScene::kSimSpeeds[sim.speedIndex];
        if (speedCombo("##speed", speed, SimulatedScene::kSimSpeeds, speed)) {
            for (int i = 0; i < static_cast<int>(std::size(SimulatedScene::kSimSpeeds)); ++i) {
                if (SimulatedScene::kSimSpeeds[i] == speed) sim.setSpeed(i);
            }
        }
    }
    ImGui::SameLine();
    const bool recording = sim.recorder != nullptr;
    const bool clicked = iconButton("##record", recording ? Icon::Stop : Icon::Record, !sim.replay,
                                    withKey(recording ? TextId::SimRecordStop : TextId::SimRecordStart, "Ctrl+Shift+R"));
    if (clicked || pressRecord) toggleRecording(f);
    // 按下后记录可能已经结束（recorder 已释放）：按当前状态显示，不能沿用按下前的 recording
    if (sim.recorder) {
        ImGui::SameLine();
        ImGui::AlignTextToFramePadding();
        ImGui::PushStyleColor(ImGuiCol_Text, kRecordRed);
        ImGui::Text("%s  %.1f MB", clockText(sim.instance.world->simulationTime() - sim.recordStartTime).c_str(),
                    static_cast<double>(sim.recorder->bytes()) / (1024.0 * 1024.0));
        ImGui::PopStyleColor();
    }
}

// 视口上方的仿真工具栏：播放控制与录制在左，状态与时间在右
void simulationToolbar(Frame& f) {
    if (!f.simulated) return;
    SimulatedScene& sim = *f.simulated;
    ImGui::SetCursorPos(ImVec2(ImGui::GetCursorPosX() + 4.0f, ImGui::GetCursorPosY() + 3.0f));
    ImGui::PushID("toolbar");
    transportControls(f, std::exchange(f.state.selfTestPressRecord, false));
    ImGui::PopID();
    const char* status = sim.replay                ? tr(TextId::StatusReplay)
                         : sim.runner->faulted()   ? tr(TextId::StatusFaulted)
                         : sim.runner->paused()    ? tr(TextId::StatusPaused)
                                                   : tr(TextId::StatusRunning);
    char text[96];
    std::snprintf(text, sizeof text, "%s   %s", status,
                  clockText(sim.replay ? sim.replay->time : sim.instance.world->simulationTime()).c_str());
    sameLineRight(ImGui::CalcTextSize(text).x + 8.0f);
    ImGui::AlignTextToFramePadding();
    if (sim.replay) ImGui::TextColored(ImGui::GetStyleColorVec4(ImGuiCol_CheckMark), "%s", text);
    else ImGui::TextUnformatted(text);
}

void sceneErrorWindow(Frame& f) {
    // 场景与动作共用一个弹窗；标题按最近一次失败的来源
    const std::string title = f.state.errorTitle == "motion"   ? label(TextId::MotionErrorTitle, "SceneErrors")
                              : f.state.errorTitle == "record" ? label(TextId::RecordErrorTitle, "SceneErrors")
                              : f.state.errorTitle == "pack"   ? label(TextId::LogPackFailed, "SceneErrors")
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
                                              CommandAction{st::Command::SimReset, "sim_reset"}, CommandAction{st::Command::SimRecord, "sim_record"}}) {
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
void openQuickOpen(Frame& f) {
    auto& p = f.state.libraryPanel;
    p.quickOpen = true;
    p.quickFocus = true;
    p.quickFrames = 0;
    p.quickSelected = 0;
    p.quickSearch.fill('\0');
    f.state.assetsStale = f.state.assetsStale || f.state.assetScans == 0;
}

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
    if (chord(ImGuiKey_P)) openQuickOpen(f);  // M4-08c：快速打开（辅助；主要方式是单击资源库中的卡片）
    if (chord(ImGuiKey_R)) simReset(f);  // 固定快捷键；另可在偏好设置中给“仿真：重置”分配单键
    // M3-07d：Ctrl+Shift+R 开始 / 停止记录（组合键要求修饰键完全一致，与 Ctrl+R 不会同时触发）；另可分配单键
    if (ImGui::IsKeyChordPressed(ImGuiMod_Ctrl | ImGuiMod_Shift | ImGuiKey_R)) toggleRecording(f);
}

void menuBar(Frame& f) {
    if (!ImGui::BeginMainMenuBar()) return;
    if (ImGui::BeginMenu(tr(TextId::MenuFile))) {
        auto& recent = f.settings.current.ui.recentScenes;
        if (ImGui::MenuItem(tr(TextId::MenuFileOpenScene), "Ctrl+O")) {
            f.state.fileBrowser.open(f.simulated ? f.simulated->path : recent.empty() ? std::string{} : recent.front(), kSceneSuffix);
        }
        // M4-08a：导入 URDF / xacro / MJCF（也可以把文件拖进窗口）
        if (ImGui::MenuItem(tr(TextId::MenuFileImport))) {
            f.state.importWizard.browser.open(f.state.importWizard.options.input, vw::kImportFilter, "");
        }
        // M4-08c：资产包、快速打开、资源库中的场景（按来源分组，点一下打开）
        if (ImGui::MenuItem(tr(TextId::MenuFileImportPack))) f.state.packBrowser.open(importDirectory(f), terra::library::kPackSuffix, "");
        if (ImGui::MenuItem(tr(TextId::MenuFileQuickOpen), "Ctrl+P")) openQuickOpen(f);
        if (ImGui::BeginMenu(tr(TextId::MenuFileLibraryScenes))) {
            using Group = std::pair<terra::library::AssetSource, TextId>;
            bool any = false;
            for (const auto& [source, text] : {Group{terra::library::AssetSource::Builtin, TextId::LibrarySourceBuiltin},
                                               Group{terra::library::AssetSource::Imported, TextId::LibrarySourceImported},
                                               Group{terra::library::AssetSource::User, TextId::LibrarySourceUser}}) {
                bool header = false;
                for (const auto& a : f.state.assetList) {
                    if (a.kind != terra::library::AssetKind::Scene || a.hidden || a.source != source) continue;
                    if (!header) {
                        if (any) ImGui::Separator();
                        ImGui::TextDisabled("%s", tr(text));
                        header = any = true;
                    }
                    ImGui::PushID(a.path.c_str());
                    if (ImGui::MenuItem(a.title.c_str(), nullptr, f.simulated && f.simulated->path == a.path)) f.state.openRequest = a.path;
                    ImGui::PopID();
                }
            }
            if (!any) ImGui::TextDisabled("%s", tr(TextId::LibraryEmpty));
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu(tr(TextId::MenuFileRecent))) {
            if (recent.empty()) ImGui::TextDisabled("%s", tr(TextId::MenuFileRecentEmpty));
            for (const auto& path : recent) {
                if (ImGui::MenuItem(path.c_str())) f.state.openRequest = path;
            }
            ImGui::EndMenu();
        }
        if (ImGui::MenuItem(tr(TextId::MenuFileCloseScene), nullptr, false, f.simulated.has_value())) f.state.closeRequest = true;
        if (ImGui::MenuItem(tr(TextId::MenuFileRecords))) {
            f.state.showTimeline = true;
            f.state.focusTimeline = true;
            f.state.recordListStale = true;
        }
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
        // M4-06a：ROS2 桥接（默认通用配置）；构建时没有 ROS2 时禁用，悬停提示安装
        ImGui::Separator();
        if (ImGui::MenuItem(tr(TextId::SimRos2), nullptr, ros2Running(f), ros2Available())) {
            if (ros2Running(f)) {
                stopRos2(f);
            } else {
                f.state.ros2Profile = kDefaultRos2Profile;
                startRos2(f);
            }
        }
        if (!ros2Available() && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) ImGui::SetTooltip("%s", tr(TextId::SimRos2Unavailable));
        // M3-07c：实验记录与采样率
        ImGui::Separator();
        if (ImGui::MenuItem(sim.recorder ? tr(TextId::SimRecordStop) : tr(TextId::SimRecordStart), "Ctrl+Shift+R", false, !sim.replay)) {
            toggleRecording(f);
        }
        // M4-07：记录时同时保存相机图像（只在场景中有相机时有意义）
        if (ImGui::MenuItem(tr(TextId::SimRecordImages), nullptr, f.state.recordCameraImages, !sim.recorder && !sim.instance.cameras.empty())) {
            f.state.recordCameraImages = !f.state.recordCameraImages;
        }
        if (ImGui::BeginMenu(tr(TextId::SimRecordRate), !sim.recorder)) {
            for (const double hz : {10.0, 25.0, 50.0, 100.0, 250.0, 500.0}) {
                char option[16];
                std::snprintf(option, sizeof option, "%g Hz", hz);
                if (ImGui::MenuItem(option, nullptr, f.settings.current.ui.recordSampleHz == hz)) f.settings.current.ui.recordSampleHz = hz;
            }
            ImGui::EndMenu();
        }
        ImGui::Separator();
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
    // M4-10b：策略菜单（导入、运行、停止、组件）
    if (ImGui::BeginMenu(tr(TextId::MenuPolicy))) {
        if (ImGui::MenuItem(tr(TextId::MenuPolicyImport))) vw::requestPolicyImport(f.state.policy, {});
        if (ImGui::MenuItem(tr(TextId::SimRunPolicy), nullptr, false, f.simulated.has_value())) {
            f.state.policy.dialogOpen = true;
            f.state.policy.preparedKey.clear();
        }
        if (ImGui::MenuItem(tr(TextId::SimStopPolicy), nullptr, false, f.simulated && f.simulated->policy)) stopPolicy(f);
        ImGui::Separator();
        if (ImGui::MenuItem(tr(TextId::MenuSettingsComponents), nullptr, f.state.policy.componentsOpen)) {
            f.state.policy.componentsOpen = !f.state.policy.componentsOpen;
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
        ImGui::MenuItem(tr(TextId::PanelLibrary), nullptr, &f.state.showLibrary);
        ImGui::MenuItem(tr(TextId::PanelOutliner), nullptr, &panels.showOutliner);
        ImGui::MenuItem(tr(TextId::PanelDetails), nullptr, &panels.showDetails);
        ImGui::MenuItem(tr(TextId::PanelLog), nullptr, &panels.showLog);
        ImGui::MenuItem(tr(TextId::PanelStats), nullptr, &panels.showStats);
        ImGui::MenuItem(tr(TextId::PanelActuators), nullptr, &f.state.showActuators);
        ImGui::MenuItem(tr(TextId::PanelTimeline), nullptr, &f.state.showTimeline);
        ImGui::MenuItem(tr(TextId::PanelSensors), nullptr, &f.state.showSensors);
        // M4-08d：ROS2 面板（没有 ROS2 时禁用，悬停给出安装说明）与显示配置
        {
            bool ros2 = ros2PanelOpen(f);
            if (ImGui::MenuItem(tr(TextId::PanelRos2), nullptr, &ros2, vw::ros2PanelAvailable())) toggleRos2Panel(f, ros2);
            if (!vw::ros2PanelAvailable() && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                ImGui::SetTooltip("%s", tr(TextId::Ros2Unavailable));
            }
        }
        if (ImGui::BeginMenu(tr(TextId::MenuWindowDisplay))) {
            auto& viz = f.state.viz;
            for (const auto& [name, file] : sceneDisplays(f)) {
                if (ImGui::MenuItem(name.c_str(), nullptr, file == viz.path)) loadDisplay(f, file, true);
            }
            ImGui::Separator();
            if (ImGui::MenuItem(tr(TextId::VizSave), nullptr, false, viz.dirty || viz.path.empty())) saveDisplay(f, viz.config.name);
            if (ImGui::MenuItem(tr(TextId::VizSaveAs))) {
                viz.saveAsOpen = true;
                std::snprintf(viz.saveName.data(), viz.saveName.size(), "%s", viz.config.name.c_str());
            }
            if (ImGui::MenuItem(tr(TextId::VizExport))) viz.exportOpen = true;
            if (ImGui::MenuItem(tr(TextId::VizImport))) f.state.vizBrowser.open(displaysDirectory(f), terra::viz::kVizSuffix, "");
            if (ImGui::MenuItem(tr(TextId::VizCloseAll), nullptr, false, !viz.config.windows.empty())) {
                viz.config.windows.clear();
                viz.dirty = true;
            }
            ImGui::EndMenu();
        }
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
        if (ImGui::MenuItem(tr(TextId::MenuSettingsComponents), nullptr, f.state.policy.componentsOpen)) {  // M4-10a
            f.state.policy.componentsOpen = !f.state.policy.componentsOpen;
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
                if (sim.recorder) {
                    ImGui::SameLine();
                    ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.3f, 1.0f), "  |  %s %.1f MB", tr(TextId::StatusRecording),
                                       static_cast<double>(sim.recorder->bytes()) / (1024.0 * 1024.0));
                }
                if (sim.replay) {
                    ImGui::SameLine();
                    ImGui::Text("  |  %s %.2f / %.2f s", tr(TextId::StatusReplay), sim.replay->time, sim.replay->duration);
                }
#ifdef TERRA_WITH_ROS2
                if (sim.ros2) {
                    ImGui::SameLine();
                    ImGui::TextColored(ImVec4(0.45f, 0.75f, 1.0f, 1.0f), "  |  %s %zu", tr(TextId::StatusRos2), sim.ros2->stats().subscribers);
                }
#endif
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
            // M4-08c：没有打开场景时显示欢迎页（场景卡片，单击即打开）
            if (!f.simulated && f.state.showWelcome) {
                if (auto action = vw::drawWelcome(f.state.libraryPanel, libraryContext(f), f.state.viewportImageMin, f.state.viewportImageMax)) {
                    handleLibraryAction(f, *action);
                }
                if (f.state.libraryPanel.welcomeCards > 0) {
                    const float k = ImGui::GetFontSize();
                    ImGui::SetCursorScreenPos(ImVec2(f.state.viewportImageMax.x - k * 2.2f, f.state.viewportImageMin.y + k * 0.6f));
                    if (ImGui::Button("##closeWelcome", ImVec2(k * 1.6f, k * 1.6f))) f.state.showWelcome = false;
                    const ImVec2 b0 = ImGui::GetItemRectMin(), b1 = ImGui::GetItemRectMax();
                    const float m = k * 0.45f;
                    ImDrawList* dl = ImGui::GetWindowDrawList();
                    dl->AddLine(ImVec2(b0.x + m, b0.y + m), ImVec2(b1.x - m, b1.y - m), ImGui::GetColorU32(ImGuiCol_Text), k * 0.12f);
                    dl->AddLine(ImVec2(b0.x + m, b1.y - m), ImVec2(b1.x - m, b0.y + m), ImGui::GetColorU32(ImGuiCol_Text), k * 0.12f);
                }
            }
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

// ---- 执行器面板（M3-07d）：每个执行器一个“实测 + 目标”控件，回放时与时间轴同步 ----

struct ActuatorReading {
    double measured = 0.0;
    std::optional<double> target;  // 回放中 PD 层接管的执行器没有记录目标（记录的是 PD 层的输出）
    std::optional<std::pair<double, double>> range;
    bool outOfRange = false;
};

// 实测量按执行器类型（评审问题 3）：位置执行器与 PD 层取关节位置，速度执行器取关节速度，力矩执行器取输出
// （除以增益，与控制量同一单位），不作用在铰链或滑动关节上的执行器取目标本身。
// recorded 非空时为回放：snap 是记录中该时刻的状态，目标取记录中该时刻生效的控制量。
ActuatorReading actuatorReading(const SimulatedScene& sim, std::size_t i, const terra::data::PhysicsSnapshot& snap,
                                const std::map<std::uint32_t, std::vector<double>>* recorded) {
    using terra::data::ActuatorKind;
    using terra::data::JointType;
    const auto& d = sim.instance.world->description();
    const auto& a = d.actuators[i];
    const bool pd = sim.pd.manages(a.id);
    const terra::data::JointState* joint = nullptr;
    if (d.joint(a.joint) != nullptr && a.joint.value >= 1 && a.joint.value <= snap.joints.size()) {
        const auto& js = snap.joints[a.joint.value - 1];
        if (js.type == JointType::Hinge || js.type == JointType::Slide) joint = &js;
    }
    ActuatorReading r;
    if (recorded != nullptr) {
        const auto it = recorded->find(a.id.value);
        if (!pd && it != recorded->end() && !it->second.empty()) r.target = it->second[0];
    } else if (const auto pending = sim.panelTargets.find(a.id.value); pending != sim.panelTargets.end()) {
        r.target = pending->second;
    } else if (pd) {
        r.target = sim.pd.target(a.id).value_or(joint != nullptr ? joint->position[0] : 0.0);
    } else if (i < snap.actuators.size() && !snap.actuators[i].controls.empty()) {
        r.target = snap.actuators[i].controls[0];
    }
    if (joint != nullptr && (pd || a.kind == ActuatorKind::Position)) {
        r.measured = joint->position[0];
    } else if (joint != nullptr && a.kind == ActuatorKind::Velocity) {
        r.measured = joint->velocity[0];
    } else if (a.kind == ActuatorKind::Motor && i < snap.actuators.size()) {
        r.measured = a.gain != 0.0 ? snap.actuators[i].force / a.gain : snap.actuators[i].force;
    } else {
        r.measured = r.target.value_or(0.0);
    }
    r.range = pd ? sim.pd.targetRange(a) : terra::simulation::controlRange(a);
    if (r.range) r.outOfRange = r.measured < r.range->first - 1e-9 || r.measured > r.range->second + 1e-9;
    return r;
}

// 填充条表示实测量（从 0 或范围下限起），竖线表示目标；超出范围时填充条为红色。
// 可编辑时在条上按下或拖动即把目标设为该处的值（返回新目标）；双击可输入精确值。
std::optional<double> actuatorBar(const ActuatorReading& r, bool editable, const std::string& tooltip) {
    double lo = -1.0;
    double hi = 1.0;
    if (r.range && r.range->second > r.range->first) {
        lo = r.range->first;
        hi = r.range->second;
    } else {
        const double m = 1.25 * std::max({1.0, std::fabs(r.measured), std::fabs(r.target.value_or(0.0))});
        lo = -m;
        hi = m;
    }
    const float height = std::round(ImGui::GetFontSize() * 0.55f);
    ImGui::InvisibleButton("##bar", ImVec2(std::max(ImGui::GetContentRegionAvail().x, 1.0f), height + 4.0f));
    const bool hovered = ImGui::IsItemHovered();
    const bool active = ImGui::IsItemActive();
    const ImVec2 lo2 = ImGui::GetItemRectMin();
    const ImVec2 hi2 = ImGui::GetItemRectMax();
    const ImVec2 p0(lo2.x, lo2.y + 2.0f);
    const ImVec2 p1(hi2.x, hi2.y - 2.0f);
    const auto xOf = [&](double v) {
        return p0.x + static_cast<float>((std::clamp(v, lo, hi) - lo) / (hi - lo)) * (p1.x - p0.x);
    };
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const float rounding = height * 0.5f;
    dl->AddRectFilled(p0, p1, ImGui::GetColorU32(editable && (hovered || active) ? ImGuiCol_FrameBgHovered : ImGuiCol_FrameBg), rounding);
    const double origin = lo < 0.0 && hi > 0.0 ? 0.0 : lo;
    const float x0 = xOf(origin);
    const float x1 = xOf(r.measured);
    const ImU32 fill = r.outOfRange ? kRecordRed : ImGui::GetColorU32(ImGuiCol_SliderGrab);
    if (std::fabs(x1 - x0) >= 1.0f) dl->AddRectFilled(ImVec2(std::min(x0, x1), p0.y), ImVec2(std::max(x0, x1), p1.y), fill, rounding);
    if (origin == 0.0 && lo < 0.0) dl->AddLine(ImVec2(x0, p0.y), ImVec2(x0, p1.y), ImGui::GetColorU32(ImGuiCol_Border));
    if (r.target) {
        const float xt = xOf(*r.target);
        const ImU32 col = ImGui::GetColorU32(editable ? ImGuiCol_Text : ImGuiCol_TextDisabled);
        dl->AddRectFilled(ImVec2(xt - 1.5f, lo2.y), ImVec2(xt + 1.5f, hi2.y), col, 1.0f);
    }
    if (hovered && !tooltip.empty()) ImGui::SetTooltip("%s", tooltip.c_str());
    if (!editable) return std::nullopt;
    if (hovered) ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
    std::optional<double> chosen;
    if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) ImGui::OpenPopup("##exact");
    if (active && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        const double alpha = std::clamp(static_cast<double>((ImGui::GetIO().MousePos.x - p0.x) / std::max(p1.x - p0.x, 1.0f)), 0.0, 1.0);
        chosen = lo + alpha * (hi - lo);
    }
    if (ImGui::BeginPopup("##exact")) {
        static double exact = 0.0;
        if (ImGui::IsWindowAppearing()) {
            exact = r.target.value_or(r.measured);
            ImGui::SetKeyboardFocusHere();
        }
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 8.0f);
        if (ImGui::InputDouble("##value", &exact, 0.0, 0.0, "%.4f", ImGuiInputTextFlags_EnterReturnsTrue)) {
            chosen = r.range ? std::clamp(exact, r.range->first, r.range->second) : exact;
            ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
    }
    return chosen;
}

bool containsNoCase(const std::string& text, const char* pattern) {
    if (pattern == nullptr || *pattern == '\0') return true;
    const auto lower = [](unsigned char c) { return static_cast<char>(std::tolower(c)); };
    std::string a = text;
    std::string b = pattern;
    std::transform(a.begin(), a.end(), a.begin(), lower);
    std::transform(b.begin(), b.end(), b.begin(), lower);
    return a.find(b) != std::string::npos;
}

void actuatorPanel(Frame& f) {
    if (!f.state.showActuators) return;
    bool visible = true;
    f.state.actuatorShown.clear();
    if (ImGui::Begin(label(TextId::PanelActuators, "Actuators").c_str(), &visible)) {
        if (!f.simulated) {
            ImGui::TextDisabled("%s", tr(TextId::OutlinerNoScene));
        } else {
            SimulatedScene& sim = *f.simulated;
            const auto& d = sim.instance.world->description();
            const auto live = sim.instance.world->snapshot();
            // 回放时实测取记录中的关节状态，目标取记录中该时刻生效的控制量（第 s 步的状态反映第 s-1 步之前下发的控制量）
            std::optional<terra::data::PhysicsSnapshot> replayed;
            std::map<std::uint32_t, std::vector<double>> recorded;
            if (sim.replay) {
                replayed = sim.replaySnapshot();
                const std::uint64_t step = sim.replayStep();
                f.state.actuatorShownStep = step;
                recorded = terra::simulation::controlsAt(sim.replay->record, step > 0 ? step - 1 : 0);
            }
            const terra::data::PhysicsSnapshot& snap = replayed ? *replayed : *live;
            const bool locked = sim.replay || (sim.motion && sim.motion->playing());
            if (sim.replay) ImGui::TextDisabled("%s", tr(TextId::ActuatorsReplay));
            else if (locked) ImGui::TextDisabled("%s", tr(TextId::ActuatorsLocked));
            ImGui::SetNextItemWidth(-FLT_MIN);
            ImGui::InputTextWithHint("##filter", tr(TextId::ActuatorsFilter), f.state.actuatorFilter.data(), f.state.actuatorFilter.size());
            std::string instance;
            bool open = true;
            for (std::size_t i = 0; i < d.actuators.size(); ++i) {
                const auto& a = d.actuators[i];
                const auto slash = a.name.find('/');
                const std::string prefix = slash == std::string::npos ? std::string{} : a.name.substr(0, slash);
                if (i == 0 || prefix != instance) {
                    instance = prefix;
                    ImGui::PushID(instance.c_str());
                    open = ImGui::CollapsingHeader(instance.empty() ? "-" : instance.c_str(), ImGuiTreeNodeFlags_DefaultOpen);
                    ImGui::PopID();
                }
                if (!open || !containsNoCase(a.name, f.state.actuatorFilter.data())) continue;
                const std::string shortName = slash == std::string::npos ? a.name : a.name.substr(slash + 1);
                ImGui::PushID(static_cast<int>(a.id.value));
                if (a.inputs.size() != 1) {
                    ImGui::TextDisabled("%s  %s", shortName.c_str(), tr(TextId::ActuatorsMultiInput));
                    ImGui::PopID();
                    continue;
                }
                const bool pd = sim.pd.manages(a.id);
                const ActuatorReading r = actuatorReading(sim, i, snap, sim.replay ? &recorded : nullptr);
                f.state.actuatorShown[a.id.value] = {r.measured, r.target};
                // 第一行：名字、类型与单位，右侧“实测 → 目标”
                const char* unit = actuatorUnit(a, d, pd);
                ImGui::TextUnformatted(shortName.c_str());
                ImGui::SameLine();
                ImGui::TextDisabled("%s%s%s", actuatorKindText(a.kind, pd), *unit ? " · " : "", unit);
                char numbers[64];
                if (r.target) std::snprintf(numbers, sizeof numbers, "%.3f → %.3f", r.measured, *r.target);
                else std::snprintf(numbers, sizeof numbers, "%.3f", r.measured);
                const float w = ImGui::CalcTextSize(numbers).x;
                sameLineRight(w);
                if (r.outOfRange) ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(kRecordRed), "%s", numbers);
                else ImGui::TextUnformatted(numbers);
                // 第二行：控件；悬停时显示细节
                std::string tip = a.name + "\n" + tr(TextId::ActuatorMeasured) + "  " + std::to_string(r.measured);
                if (r.target) tip += std::string{"\n"} + tr(TextId::ActuatorTarget) + "  " + std::to_string(*r.target);
                if (r.range) tip += std::string{"\n"} + tr(TextId::ActuatorRange) + "  [" + std::to_string(r.range->first) + ", " + std::to_string(r.range->second) + "]";
                if (!sim.replay && i < live->actuators.size()) tip += std::string{"\n"} + tr(TextId::ActuatorForce) + "  " + std::to_string(live->actuators[i].force);
                tip += std::string{"\n"} + tr(TextId::ActuatorGear) + "  " + std::to_string(a.gear);
                if (r.outOfRange) tip += std::string{"\n"} + tr(TextId::ActuatorOutOfRange);
                if (!locked) tip += std::string{"\n"} + tr(TextId::ActuatorDragHint);
                if (const auto value = actuatorBar(r, !locked, tip); value && (!r.target || *value != *r.target)) {
                    sim.panelTargets[a.id.value] = *value;
                    if (pd) {
                        sim.pd.setTarget(a.id, *value);
                    } else {
                        sim.runner->scheduleNext({{a.id, {*value}}});
                    }
                }
                ImGui::PopID();
            }
            // 已经出现在快照中的面板目标不再单独保存（PD 层的目标始终由 PD 层保存）
            for (auto it = sim.panelTargets.begin(); it != sim.panelTargets.end();) {
                const auto index = static_cast<std::size_t>(it->first - 1);
                const bool pd = sim.pd.manages(terra::data::ActuatorId{it->first});
                const bool applied = pd || (index < live->actuators.size() && !live->actuators[index].controls.empty() &&
                                            live->actuators[index].controls[0] == it->second);
                it = applied ? sim.panelTargets.erase(it) : std::next(it);
            }
        }
    }
    ImGui::End();
    if (!visible) f.state.showActuators = false;
}

// ---- 时间轴标签页（M3-07d）：播放控制、实时 / 回放、时间刻度与记录列表 ----

struct TimelineMarker {
    double time = 0.0;
    bool error = false;
    std::string text;
};

// 主刻度的间隔（s）与其中的次刻度数：按每秒像素数选第一个能放下时间标签的间隔
std::pair<double, int> rulerStep(double pixelsPerSecond, float labelWidth) {
    static constexpr std::pair<double, int> kSteps[] = {{0.01, 5}, {0.02, 4}, {0.05, 5}, {0.1, 5},  {0.2, 4},  {0.5, 5},
                                                        {1.0, 5},  {2.0, 4},  {5.0, 5},  {10.0, 5}, {15.0, 3}, {30.0, 6},
                                                        {60.0, 6}, {120.0, 4}, {300.0, 5}, {600.0, 5}, {1800.0, 6}, {3600.0, 6}};
    for (const auto& s : kSteps) {
        if (s.first * pixelsPerSecond >= labelWidth) return s;
    }
    return kSteps[std::size(kSteps) - 1];
}

std::string rulerLabel(double t, double step) {
    char buf[32];
    if (step >= 60.0) {
        const auto total = static_cast<long long>(std::llround(t));
        std::snprintf(buf, sizeof buf, "%lld:%02lld", total / 60, total % 60);
    } else {
        std::snprintf(buf, sizeof buf, step >= 1.0 ? "%.0fs" : step >= 0.1 ? "%.1fs" : "%.2fs", t);
    }
    return buf;
}

// 时间刻度：上面是主刻度的时间标签与刻度线，下面是轨道（band 为高亮的区段，markers 为事件）；播放头贯穿两者。
// draggable 时可以在刻度上按下或拖动播放头，返回新的时间。
std::optional<double> timeRuler(double t0, double t1, double playhead, std::optional<std::pair<double, double>> band, ImU32 bandColor,
                                const std::vector<TimelineMarker>& markers, bool draggable) {
    if (!(t1 > t0)) t1 = t0 + 1.0;
    const float font = ImGui::GetFontSize();
    const float width = std::max(ImGui::GetContentRegionAvail().x, 1.0f);
    const float labelH = font + 2.0f;
    const float tickH = font * 0.45f;
    const float trackH = font * 1.1f;
    ImGui::InvisibleButton("##ruler", ImVec2(width, labelH + tickH + trackH));
    const bool hovered = ImGui::IsItemHovered();
    const bool active = ImGui::IsItemActive();
    const ImVec2 a = ImGui::GetItemRectMin();
    const ImVec2 b = ImGui::GetItemRectMax();
    const float inset = 6.0f;  // 两端留出播放头的位置
    const float x0 = a.x + inset;
    const float x1 = b.x - inset;
    const auto xOf = [&](double t) { return x0 + static_cast<float>((std::clamp(t, t0, t1) - t0) / (t1 - t0)) * (x1 - x0); };
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImU32 text = ImGui::GetColorU32(ImGuiCol_Text);
    const ImU32 faint = ImGui::GetColorU32(ImGuiCol_TextDisabled);
    const ImU32 grid = ImGui::GetColorU32(ImGuiCol_Border);
    const float tickTop = a.y + labelH;
    const float trackTop = tickTop + tickH;
    // 轨道
    dl->AddRectFilled(ImVec2(x0, trackTop), ImVec2(x1, b.y), ImGui::GetColorU32(ImGuiCol_FrameBg), 3.0f);
    if (band) dl->AddRectFilled(ImVec2(xOf(band->first), trackTop + 3.0f), ImVec2(xOf(band->second), b.y - 3.0f), bandColor, 2.0f);
    // 刻度
    const auto [step, sub] = rulerStep((x1 - x0) / (t1 - t0), ImGui::CalcTextSize("00:00").x + font * 1.5f);
    const double minor = step / sub;
    for (auto k = static_cast<long long>(std::ceil(t0 / minor - 1e-9)); static_cast<double>(k) * minor <= t1 + 1e-9; ++k) {
        const double t = static_cast<double>(k) * minor;
        const float x = xOf(t);
        const bool major = k % sub == 0;
        dl->AddLine(ImVec2(x, major ? tickTop - 2.0f : tickTop + tickH * 0.45f), ImVec2(x, trackTop), major ? faint : grid);
        if (major) {
            const std::string s = rulerLabel(t, step);
            const float tw = ImGui::CalcTextSize(s.c_str()).x;
            const float lx = std::max(a.x, std::min(x - tw * 0.5f, b.x - tw));
            dl->AddText(ImVec2(lx, a.y), faint, s.c_str());
            dl->AddLine(ImVec2(x, trackTop), ImVec2(x, b.y), (grid & 0x00FFFFFFu) | 0x30000000u);
        }
    }
    // 事件：轨道上方的小三角，悬停时显示内容
    std::string hoveredMarker;
    for (const auto& m : markers) {
        const float x = xOf(m.time);
        const ImU32 col = m.error ? kRecordRed : text;
        dl->AddTriangleFilled(ImVec2(x - 4.0f, trackTop), ImVec2(x + 4.0f, trackTop), ImVec2(x, trackTop + 6.0f), col);
        if (hovered && std::fabs(ImGui::GetIO().MousePos.x - x) <= 5.0f) hoveredMarker = m.text;
    }
    // 播放头
    const ImU32 head = ImGui::GetColorU32(ImGuiCol_CheckMark);
    const float xp = xOf(playhead);
    dl->AddLine(ImVec2(xp, a.y + labelH * 0.5f), ImVec2(xp, b.y), head, 2.0f);
    dl->AddTriangleFilled(ImVec2(xp - 5.0f, tickTop - 5.0f), ImVec2(xp + 5.0f, tickTop - 5.0f), ImVec2(xp, tickTop + 2.0f), head);
    if (!hoveredMarker.empty()) ImGui::SetTooltip("%s", hoveredMarker.c_str());
    if (!draggable) return std::nullopt;
    if (hovered || active) ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
    if (active && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        const double alpha = std::clamp(static_cast<double>((ImGui::GetIO().MousePos.x - x0) / std::max(x1 - x0, 1.0f)), 0.0, 1.0);
        return t0 + alpha * (t1 - t0);
    }
    return std::nullopt;
}

// 两段式切换按钮（实时 | 回放）中的一段
bool segment(const char* text, bool selected, bool enabled) {
    if (selected) ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
    ImGui::BeginDisabled(!enabled);
    const bool pressed = ImGui::Button(text);
    ImGui::EndDisabled();
    if (selected) ImGui::PopStyleColor();
    return pressed && enabled && !selected;
}

// 记录的创建时间（UTC，ISO 8601）换成本地时间“MM-DD HH:MM:SS”；读不出时原样返回
std::string localTimeText(const std::string& iso) {
    std::tm utc{};
    if (std::sscanf(iso.c_str(), "%d-%d-%dT%d:%d:%d", &utc.tm_year, &utc.tm_mon, &utc.tm_mday, &utc.tm_hour, &utc.tm_min, &utc.tm_sec) != 6) {
        return iso;
    }
    utc.tm_year -= 1900;
    utc.tm_mon -= 1;
    const std::time_t t = timegm(&utc);
    std::tm local{};
    localtime_r(&t, &local);
    char buf[32];
    std::strftime(buf, sizeof buf, "%m-%d %H:%M:%S", &local);
    return buf;
}

const char* recordReasonText(const std::string& reason) {
    if (reason == "completed") return tr(TextId::RecordReasonCompleted);
    if (reason == "stopped") return tr(TextId::RecordReasonStopped);
    if (reason == "error") return tr(TextId::RecordReasonError);
    if (reason == "incomplete") return tr(TextId::RecordReasonIncomplete);
    return reason.c_str();
}

void recordTable(Frame& f) {
    const bool scene = f.simulated.has_value();
    ImGui::AlignTextToFramePadding();
    ImGui::TextDisabled("%s  (%zu)", tr(f.state.recordsOnlyScene && scene ? TextId::TimelineRecordsScene : TextId::TimelineRecordsAll),
                        f.state.recordList.size());
    bool all = !f.state.recordsOnlyScene;
    const char* allText = tr(TextId::TimelineAllScenes);
    const char* rescanText = tr(TextId::RecordsRescan);
    const ImGuiStyle& style = ImGui::GetStyle();
    const float right = ImGui::CalcTextSize(allText).x + ImGui::GetFrameHeight() + style.ItemInnerSpacing.x + style.ItemSpacing.x +
                        ImGui::CalcTextSize(rescanText).x + style.FramePadding.x * 2.0f;
    sameLineRight(right);
    if (ImGui::Checkbox(allText, &all)) {
        f.state.recordsOnlyScene = !all;
        f.state.recordListStale = true;
    }
    ImGui::SameLine();
    if (ImGui::Button(rescanText)) {
        if (auto* lib = recordLibrary(f)) {
            const auto result = lib->rebuild({recordsDirectory(f)});
            for (const auto& e : result.errors) f.state.log.push_back(e);
        }
        f.state.recordListStale = true;
    }
    if (f.state.recordList.empty()) {
        ImGui::TextDisabled("%s", tr(TextId::RecordsEmpty));
        return;
    }
    const ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY | ImGuiTableFlags_BordersInnerH |
                                  ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchProp;
    if (!ImGui::BeginTable("records", 8, flags, ImVec2(0.0f, 0.0f))) return;
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn(tr(TextId::RecordsColTime), ImGuiTableColumnFlags_None, 1.3f);
    ImGui::TableSetupColumn(tr(TextId::RecordsColScene), ImGuiTableColumnFlags_None, 1.6f);
    ImGui::TableSetupColumn(tr(TextId::RecordsColMotion), ImGuiTableColumnFlags_None, 1.4f);
    ImGui::TableSetupColumn(tr(TextId::RecordsColDuration), ImGuiTableColumnFlags_None, 0.8f);
    ImGui::TableSetupColumn(tr(TextId::RecordsColSteps), ImGuiTableColumnFlags_None, 0.8f);
    ImGui::TableSetupColumn(tr(TextId::RecordsColSize), ImGuiTableColumnFlags_None, 0.8f);
    ImGui::TableSetupColumn(tr(TextId::RecordsColResult), ImGuiTableColumnFlags_None, 0.9f);
    ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_NoResize, ImGui::GetFrameHeight());
    ImGui::TableHeadersRow();
    const std::string replaying = f.simulated && f.simulated->replay ? f.simulated->replay->path : std::string{};
    const auto stem = [](const std::string& path) {
        const std::string name = std::filesystem::path{path}.filename().string();
        return name.substr(0, name.find('.'));
    };
    for (const auto& r : f.state.recordList) {
        ImGui::PushID(static_cast<int>(r.id));
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::AlignTextToFramePadding();
        if (ImGui::Selectable(localTimeText(r.created).c_str(), r.path == replaying,
                              ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowDoubleClick | ImGuiSelectableFlags_AllowOverlap) &&
            ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
            f.state.replayRequest = r.path;
        }
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayNormal)) ImGui::SetTooltip("%s", r.path.c_str());
        ImGui::TableSetColumnIndex(1);
        ImGui::TextUnformatted(stem(r.scene).c_str());
        ImGui::TableSetColumnIndex(2);
        if (r.motion.empty()) ImGui::TextDisabled("-");
        else ImGui::TextUnformatted(stem(r.motion).c_str());
        ImGui::TableSetColumnIndex(3);
        if (r.simulationTime) ImGui::Text("%.1f s", *r.simulationTime);
        else ImGui::TextDisabled("-");
        ImGui::TableSetColumnIndex(4);
        if (r.steps) ImGui::Text("%llu", static_cast<unsigned long long>(*r.steps));
        else ImGui::TextDisabled("-");
        ImGui::TableSetColumnIndex(5);
        ImGui::Text("%.1f MB", static_cast<double>(r.bytes) / (1024.0 * 1024.0));
        ImGui::TableSetColumnIndex(6);
        if (r.reason == "error" || r.reason == "incomplete") ImGui::TextColored(ImGui::ColorConvertU32ToFloat4(kRecordRed), "%s", recordReasonText(r.reason));
        else ImGui::TextUnformatted(recordReasonText(r.reason));
        ImGui::TableSetColumnIndex(7);
        if (iconButton("##replay", Icon::Play, true, tr(TextId::RecordsReplay))) f.state.replayRequest = r.path;
        ImGui::PopID();
    }
    ImGui::EndTable();
}

void timelinePanel(Frame& f) {
    if (!f.state.showTimeline) return;
    // 已保存的布局中没有这个标签页时，首次显示并入日志（或统计）所在的停靠组；默认布局中已经安排好
    for (const char* sibling : {"###Log", "###Stats"}) {
        const ImGuiWindow* w = ImGui::FindWindowByName(sibling);
        if (w != nullptr && w->DockId != 0) {
            ImGui::SetNextWindowDockID(w->DockId, ImGuiCond_FirstUseEver);
            break;
        }
    }
    if (f.state.focusTimeline) {
        ImGui::SetNextWindowFocus();
        f.state.focusTimeline = false;
    }
    if (f.state.recordListStale) {
        refreshRecordList(f);
        f.state.recordListStale = false;
    }
    bool visible = true;
    if (ImGui::Begin(label(TextId::PanelTimeline, "Timeline").c_str(), &visible)) {
        if (!f.simulated) {
            ImGui::TextDisabled("%s", tr(TextId::TimelineNoScene));
        } else {
            SimulatedScene& sim = *f.simulated;
            // 第一行：播放控制与录制 │ 实时 | 回放 │ 右侧时间
            ImGui::PushID("timeline");
            transportControls(f);
            ImGui::PopID();
            ImGui::SameLine(0.0f, ImGui::GetFontSize() * 1.5f);
            ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(1.0f, ImGui::GetStyle().ItemSpacing.y));
            if (segment(tr(TextId::TimelineLive), !sim.replay, true)) sim.replay.reset();
            ImGui::SameLine();
            if (segment(tr(TextId::TimelineReplay), sim.replay.has_value(), !f.state.recordList.empty() && !sim.recorder)) {
                f.state.replayRequest = f.state.recordList.front().path;
            }
            ImGui::PopStyleVar();
            char clock[96];
            if (sim.replay) {
                std::snprintf(clock, sizeof clock, "%s / %s", clockText(sim.replay->time).c_str(), clockText(sim.replay->duration).c_str());
            } else {
                std::snprintf(clock, sizeof clock, "%s   %s %llu", clockText(sim.instance.world->simulationTime()).c_str(), tr(TextId::StatusStep),
                              static_cast<unsigned long long>(sim.instance.world->stepIndex()));
            }
            const float cw = ImGui::CalcTextSize(clock).x;
            sameLineRight(cw);
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(clock);
            if (sim.replay) {
                ImGui::SameLine();
                ImGui::TextDisabled("%s", std::filesystem::path{sim.replay->path}.filename().string().c_str());
            }
            // 第二行：时间刻度。回放：记录的范围，可拖动；实时：仿真时间，录制区段为红色
            ImGui::Spacing();
            if (sim.replay) {
                auto& replay = *sim.replay;
                const auto& first = replay.record.states.front();
                const double dt = replay.record.header.timestep;
                std::vector<TimelineMarker> markers;
                for (const auto& e : replay.record.events) {
                    TimelineMarker m;
                    m.time = first.simulationTime + static_cast<double>(e.step - std::min(e.step, first.stepIndex)) * dt;
                    m.error = e.kind == "error";
                    m.text = e.kind + (e.detail.empty() ? "" : "：" + e.detail);
                    markers.push_back(std::move(m));
                }
                const ImU32 band = (ImGui::GetColorU32(ImGuiCol_CheckMark) & 0x00FFFFFFu) | 0x50000000u;
                if (const auto t = timeRuler(first.simulationTime, replay.duration, replay.time,
                                             std::make_pair(first.simulationTime, replay.duration), band, markers, true)) {
                    replay.time = *t;
                    replay.playing = false;
                }
            } else {
                const double now = sim.instance.world->simulationTime();
                const double end = std::max(10.0, 10.0 * std::ceil(now * 1.1 / 10.0));
                std::optional<std::pair<double, double>> band;
                if (sim.recorder) band = std::make_pair(sim.recordStartTime, now);
                timeRuler(0.0, end, now, band, (kRecordRed & 0x00FFFFFFu) | 0x90000000u, {}, false);
            }
            ImGui::Spacing();
        }
        ImGui::Separator();
        recordTable(f);
    }
    ImGui::End();
    if (!visible) f.state.showTimeline = false;
}

// ---- 传感器（M4-04）：相机预览的渲染与传感器面板 ----

// M4-07：回放时传感器面板显示记录中的读数——IMU 取每份状态中的读数（时间不晚于回放时间的最近 kImuHistory 份），
// 雷达取不晚于回放时间的最后一次扫描，相机取最后一帧的元数据，保存了图像时读入图像。只在回放时间变化时更新
void updateReplaySensors(Frame& f) {
    SimulatedScene& sim = *f.simulated;
    const auto& rec = sim.replay->record;
    const double t = sim.replay->time;
    if (sim.replaySensorsShown && sim.replaySensorTime == t) return;
    sim.replaySensorsShown = true;
    sim.replaySensorTime = t;
    for (std::size_t k = 0; k < sim.imuHistory.size(); ++k) {
        auto& history = sim.imuHistory[k];
        history.clear();
        std::uint64_t count = 0;
        for (const auto& s : rec.states) {
            if (s.simulationTime > t) break;
            if (k >= s.sensors.size()) continue;
            SimulatedScene::ImuSample sample;
            sample.time = s.simulationTime;
            for (std::size_t c = 0; c < 6 && c < s.sensors[k].values.size(); ++c) sample.values[c] = static_cast<float>(s.sensors[k].values[c]);
            history.push_back(sample);
            if (history.size() > SimulatedScene::kImuHistory) history.pop_front();
            ++count;
        }
        sim.imuSamples[k] = count;
    }
    for (std::size_t k = 0; k < sim.lidarScans.size(); ++k) {
        const auto& lidar = sim.instance.lidars[k];
        const terra::simulation::RecordLidar* last = nullptr;
        for (const auto& l : rec.lidars) {
            if (l.time > t) break;
            if (l.name == lidar.name) last = &l;
        }
        sim.lidarScans[k].reset();
        if (last == nullptr) continue;
        terra::simulation::LidarScan scan;
        scan.sensor = last->name;
        scan.step = last->step;
        scan.time = last->time;
        scan.angleMin = lidar.angleMin;
        scan.angleIncrement = lidar.samples > 1 ? (lidar.angleMax - lidar.angleMin) / static_cast<double>(lidar.samples - 1) : 0.0;
        scan.rangeMin = lidar.rangeMin;
        scan.rangeMax = lidar.rangeMax;
        scan.ranges = last->ranges;
        sim.lidarScans[k] = std::move(scan);
    }
    for (auto& s3 : sim.lidar3dScans) s3.reset();  // M5-11：记录不保存点
    const std::filesystem::path base = std::filesystem::path{sim.replay->path}.parent_path();
    for (std::size_t k = 0; k < sim.cameraPreviews.size(); ++k) {
        const auto& camera = sim.instance.cameras[k];
        auto& preview = sim.cameraPreviews[k];
        const terra::simulation::RecordCameraFrame* last = nullptr;
        for (const auto& c : rec.cameraFrames) {
            if (c.time > t) break;
            if (c.name == camera.name) last = &c;
        }
        if (last == nullptr) {
            preview.frame.reset();
            preview.color = {};
            preview.depth = {};
            preview.thermal = {};  // M5-06：记录中不保存热像
            preview.shownImage.clear();
            continue;
        }
        if (preview.frame && preview.frame->step == last->step && preview.shownImage == last->rgb + "|" + last->depth) continue;
        vw::CameraFrame frame;
        frame.sensor = camera.name;
        frame.step = last->step;
        frame.time = last->time;
        frame.pose = last->pose;
        frame.intrinsics = terra::scene::cameraIntrinsics(camera.width, camera.height, camera.fovyDegrees);
        preview.color = {};
        preview.depth = {};
        preview.thermal = {};  // M5-06：记录中不保存热像
        if (!last->rgb.empty()) {
            const auto loaded = terra::render::loadImage((base / last->rgb).string());
            if (loaded.ok() && loaded.image.width == camera.width && loaded.image.height == camera.height) {
                preview.color = terra::render::Texture2D::create(loaded.image.width, loaded.image.height, loaded.image.pixels.data(), false);
            }
        }
        if (!last->depth.empty()) {
            frame.depth = vw::readDepthNpy((base / last->depth).string(), camera.width, camera.height);
            if (!frame.depth.empty()) {
                float farthest = 0.0f;
                for (const float d : frame.depth) {
                    if (std::isfinite(d)) farthest = std::max(farthest, d);
                }
                const auto shown = vw::depthPreview(frame.depth, camera.width, camera.height, 0.0f, std::max(farthest, 0.1f));
                preview.depth = terra::render::Texture2D::create(shown.width, shown.height, shown.pixels.data(), false);
            }
        }
        preview.shownImage = last->rgb + "|" + last->depth;
        preview.frame = std::move(frame);
    }
}

// M4-07：记录时保存实际渲染的相机图像（彩色 PNG、深度 .npy），写在记录旁的 .frames 目录中，并在记录中登记
void saveRecordedImage(Frame& f, const terra::scene::CameraSensor& camera, const vw::CameraFrame& frame) {
    SimulatedScene& sim = *f.simulated;
    namespace fs = std::filesystem;
    const fs::path recordPath{sim.recorder->path()};
    const fs::path frames{terra::simulation::recordFramesDirectory(recordPath.string())};
    std::string cameraDir = camera.name;
    std::replace(cameraDir.begin(), cameraDir.end(), '/', '_');
    char name[32];
    std::snprintf(name, sizeof name, "%08llu", static_cast<unsigned long long>(frame.step));
    const std::string relative = frames.filename().string() + "/" + cameraDir;
    std::error_code ec;
    std::string rgb, depth;
    if (!frame.rgb.empty()) {
        fs::create_directories(frames / cameraDir / "rgb", ec);
        rgb = relative + "/rgb/" + name + ".png";
        if (terra::render::savePng((recordPath.parent_path() / rgb).string(), vw::colorImage(frame))) rgb.clear();
    }
    if (!frame.depth.empty()) {
        fs::create_directories(frames / cameraDir / "depth", ec);
        depth = relative + "/depth/" + name + ".npy";
        if (!vw::writeDepthNpy((recordPath.parent_path() / depth).string(), frame.depth, camera.width, camera.height)) depth.clear();
    }
    if (rgb.empty() && depth.empty()) {
        f.state.log.push_back(std::string{tr(TextId::LogRecordImageFailed)} + (frames / cameraDir).string());
        return;
    }
    sim.recorder->cameraImage(frame.step, camera.name, rgb, depth);
}

// 本帧结束前渲染各相机最新一份该采样的快照（预览只渲染最新的一份；不丢帧的完整采集用 terra_capture）
void updateCameraPreviews(Frame& f) {
    if (!f.simulated) return;
    SimulatedScene& sim = *f.simulated;
    // M4-07：回放时显示记录中的传感器读数；回到实时后从当前状态重新开始
    if (sim.replay) {
        updateReplaySensors(f);
        return;
    }
    if (sim.replaySensorsShown) {
        sim.replaySensorsShown = false;
        sim.restartSensors();
        for (auto& c : sim.cameraPreviews) {
            c.frame.reset();
            c.shownImage.clear();
        }
    }
    bool pending = false;
    for (const auto& c : sim.cameraPreviews) pending = pending || c.pending != nullptr;
    if (!pending) return;
    if (!sim.cameraRenderer) {
        std::string error;
        sim.cameraRenderer = vw::CameraRenderer::create(f.ctx, error);
        if (!sim.cameraRenderer) {
            f.state.log.push_back(std::string{tr(TextId::SensorsCameraFailed)} + error);
            for (auto& c : sim.cameraPreviews) c.pending.reset();
            return;
        }
    }
    st::RenderSettings rs = f.settings.current.render;
    if (f.state.sceneEnvironment) rs.environment = *f.state.sceneEnvironment;
    std::vector<vw::SceneMessage> messages;
    const vw::FrameLighting lit = vw::frameLighting(rs, f.environments, f.ctx, messages);
    appendMessages(f.state, messages);
    for (std::size_t k = 0; k < sim.cameraPreviews.size(); ++k) {
        auto& preview = sim.cameraPreviews[k];
        if (!preview.pending) continue;
        const auto& camera = sim.instance.cameras[k];
        preview.frame = sim.cameraRenderer->capture(sim.instance, *sim.view, camera, *preview.pending, lit);
        // M4-09：渲染后加一次噪声，之后的预览、记录与 ROS2 都用这一份
        sim.noise.color(k, preview.frame->step, preview.frame->rgb, 3);
        sim.noise.depth(k, preview.frame->step, preview.frame->depth);
        sim.noise.thermal(k, preview.frame->step, preview.frame->thermal);  // M5-06
        preview.pending.reset();
        if (preview.pendingRecorded && sim.recorder && f.state.recordCameraImages) saveRecordedImage(f, camera, *preview.frame);  // M4-07
        ++preview.rendered;
        const vw::CameraFrame& frame = *preview.frame;
#ifdef TERRA_WITH_ROS2
        if (sim.ros2) {
            // M4-06a：同一份画面发给桥接（行 0 为图像顶部；图像不按 displayRotationDeg 旋转，与真机一致）
            terra::ros2::CameraImage image;
            image.step = frame.step;
            image.time = frame.time;
            image.width = frame.intrinsics.width;
            image.height = frame.intrinsics.height;
            image.fx = frame.intrinsics.fx;
            image.fy = frame.intrinsics.fy;
            image.cx = frame.intrinsics.cx;
            image.cy = frame.intrinsics.cy;
            image.rgb = frame.rgb;
            image.depth = frame.depth;
            image.segmentation = frame.segmentation;  // M4-09
            image.thermal = frame.thermal;            // M5-06
            sim.ros2->publishCamera(k, image);
        }
#endif
        if (!frame.rgb.empty()) {
            const auto image = vw::colorImage(frame);
            preview.color = terra::render::Texture2D::create(image.width, image.height, image.pixels.data(), false);
        }
        if (!frame.depth.empty()) {
            float farthest = 0.0f;
            for (const float d : frame.depth) {
                if (std::isfinite(d)) farthest = std::max(farthest, d);
            }
            const auto image = vw::depthPreview(frame.depth, camera.width, camera.height, 0.0f, std::max(farthest, 0.1f));
            preview.depth = terra::render::Texture2D::create(image.width, image.height, image.pixels.data(), false);
        }
        if (!frame.segmentation.empty()) {  // M4-09：分割图按 ID 着色
            const auto image = vw::segmentationPreview(frame.segmentation, camera.width, camera.height);
            preview.segmentation = terra::render::Texture2D::create(image.width, image.height, image.pixels.data(), false);
        }
        if (!frame.thermal.empty()) {  // M5-06：按相机的色板与显示区间
            const auto image = vw::thermalImage(frame, camera.thermal);
            preview.thermal = terra::render::Texture2D::create(image.width, image.height, image.pixels.data(), false);
            ++f.state.thermalFramesShown;
        }
    }
}

// x、y、z 三条曲线（红、绿、蓝）画在同一坐标中，纵轴按数据自动缩放；左上角与左下角标出范围
void tripleCurve(const char* id, const std::deque<SimulatedScene::ImuSample>& samples, std::size_t offset, const char* unit, float height) {
    const float width = std::max(ImGui::GetContentRegionAvail().x, 1.0f);
    ImGui::InvisibleButton(id, ImVec2(width, height));
    const ImVec2 a = ImGui::GetItemRectMin();
    const ImVec2 b = ImGui::GetItemRectMax();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(a, b, ImGui::GetColorU32(ImGuiCol_FrameBg), 3.0f);
    if (samples.size() < 2) return;
    float lo = std::numeric_limits<float>::infinity();
    float hi = -std::numeric_limits<float>::infinity();
    for (const auto& s : samples) {
        for (std::size_t c = 0; c < 3; ++c) {
            lo = std::min(lo, s.values[offset + c]);
            hi = std::max(hi, s.values[offset + c]);
        }
    }
    if (hi - lo < 1e-3f) {
        lo -= 0.5f;
        hi += 0.5f;
    }
    const float pad = 0.08f * (hi - lo);
    lo -= pad;
    hi += pad;
    const auto yOf = [&](float v) { return b.y - (v - lo) / (hi - lo) * (b.y - a.y); };
    if (lo < 0.0f && hi > 0.0f) dl->AddLine(ImVec2(a.x, yOf(0.0f)), ImVec2(b.x, yOf(0.0f)), ImGui::GetColorU32(ImGuiCol_Border));
    const ImU32 colors[3] = {IM_COL32(230, 80, 70, 255), IM_COL32(90, 190, 90, 255), IM_COL32(80, 140, 235, 255)};
    std::vector<ImVec2> points(samples.size());
    const float step = (b.x - a.x) / static_cast<float>(samples.size() - 1);
    for (std::size_t c = 0; c < 3; ++c) {
        for (std::size_t i = 0; i < samples.size(); ++i) points[i] = ImVec2(a.x + step * static_cast<float>(i), yOf(samples[i].values[offset + c]));
        dl->AddPolyline(points.data(), static_cast<int>(points.size()), colors[c], 1.5f);  // ImGui 1.93：粗细在前、标志在后
    }
    char text[48];
    std::snprintf(text, sizeof text, "%.2f %s", static_cast<double>(hi), unit);
    dl->AddText(ImVec2(a.x + 4.0f, a.y + 2.0f), ImGui::GetColorU32(ImGuiCol_TextDisabled), text);
    std::snprintf(text, sizeof text, "%.2f %s", static_cast<double>(lo), unit);
    dl->AddText(ImVec2(a.x + 4.0f, b.y - ImGui::GetFontSize() - 2.0f), ImGui::GetColorU32(ImGuiCol_TextDisabled), text);
}

// 雷达俯视图：雷达在中心，朝上为雷达的 0°（+X），逆时针为正；红点命中，灰线没有命中，蓝点近于最小量程；每 1 m 一圈
void lidarView(const terra::simulation::LidarScan& scan, float size) {
    ImGui::InvisibleButton("##lidar", ImVec2(size, size));
    const ImVec2 a = ImGui::GetItemRectMin();
    const ImVec2 b = ImGui::GetItemRectMax();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(a, b, ImGui::GetColorU32(ImGuiCol_FrameBg), 3.0f);
    const ImVec2 c((a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f);
    double farthest = 0.0;
    for (const float r : scan.ranges) {
        if (std::isfinite(r)) farthest = std::max(farthest, static_cast<double>(r));
    }
    const double range = farthest > 0.0 ? std::min(farthest * 1.1, scan.rangeMax) : scan.rangeMax;
    const float scale = (size * 0.5f - 6.0f) / static_cast<float>(std::max(range, 0.1));
    const ImU32 grid = (ImGui::GetColorU32(ImGuiCol_Border) & 0x00FFFFFFu) | 0x60000000u;
    for (int m = 1; m <= static_cast<int>(range) && m <= 30; ++m) dl->AddCircle(c, scale * static_cast<float>(m), grid, 48);
    const ImU32 miss = (ImGui::GetColorU32(ImGuiCol_TextDisabled) & 0x00FFFFFFu) | 0x30000000u;
    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
        const double angle = scan.angleMin + static_cast<double>(i) * scan.angleIncrement;
        const ImVec2 dir(static_cast<float>(-std::sin(angle)), static_cast<float>(-std::cos(angle)));
        const float r = scan.ranges[i];
        if (std::isfinite(r)) {
            dl->AddCircleFilled(ImVec2(c.x + dir.x * r * scale, c.y + dir.y * r * scale), 1.8f, IM_COL32(230, 70, 60, 255), 6);
        } else if (r > 0) {
            dl->AddLine(c, ImVec2(c.x + dir.x * (size * 0.5f - 4.0f), c.y + dir.y * (size * 0.5f - 4.0f)), miss);
        } else {
            dl->AddCircleFilled(ImVec2(c.x + dir.x * 6.0f, c.y + dir.y * 6.0f), 1.8f, IM_COL32(70, 120, 230, 255), 6);
        }
    }
    const ImU32 head = ImGui::GetColorU32(ImGuiCol_Text);
    dl->AddTriangleFilled(ImVec2(c.x, c.y - 7.0f), ImVec2(c.x - 5.0f, c.y + 5.0f), ImVec2(c.x + 5.0f, c.y + 5.0f), head);
}

// M5-11：3D 雷达的俯视（XY）与侧视（XZ）散点图，按高度着色；点多时均匀抽稀到 6000 个
ImU32 heightColor(float z, float zMin, float zMax) {
    const float t = zMax > zMin ? std::clamp((z - zMin) / (zMax - zMin), 0.0f, 1.0f) : 0.5f;
    const auto lerp = [](float a, float b, float u) { return static_cast<int>(a + (b - a) * u); };
    // 蓝 → 青 → 黄 → 红
    if (t < 0.33f) return IM_COL32(lerp(40, 40, t / 0.33f), lerp(80, 200, t / 0.33f), lerp(220, 220, t / 0.33f), 255);
    if (t < 0.66f) return IM_COL32(lerp(40, 240, (t - 0.33f) / 0.33f), lerp(200, 220, (t - 0.33f) / 0.33f), lerp(220, 60, (t - 0.33f) / 0.33f), 255);
    return IM_COL32(lerp(240, 230, (t - 0.66f) / 0.34f), lerp(220, 60, (t - 0.66f) / 0.34f), lerp(60, 50, (t - 0.66f) / 0.34f), 255);
}

void pointCloudView(const terra::simulation::Lidar3dScan& scan, float size, bool side) {
    ImGui::InvisibleButton(side ? "##side" : "##top", ImVec2(size, side ? size * 0.5f : size));
    const ImVec2 a = ImGui::GetItemRectMin();
    const ImVec2 b = ImGui::GetItemRectMax();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(a, b, ImGui::GetColorU32(ImGuiCol_FrameBg), 3.0f);
    float extent = 0.5f, zMin = 0.0f, zMax = 0.0f;
    for (std::size_t i = 0; i < scan.points.size(); ++i) {
        const auto& p = scan.points[i];
        extent = std::max({extent, std::fabs(p.x), std::fabs(p.y)});
        zMin = i == 0 ? p.z : std::min(zMin, p.z);
        zMax = i == 0 ? p.z : std::max(zMax, p.z);
    }
    extent *= 1.05f;
    const ImVec2 c((a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f);
    const float scale = (size * 0.5f - 4.0f) / extent;
    const std::size_t stride = std::max<std::size_t>(1, scan.points.size() / 6000);
    for (std::size_t i = 0; i < scan.points.size(); i += stride) {
        const auto& p = scan.points[i];
        // 俯视：+X 向上、+Y 向左；侧视：+X 向右、+Z 向上（雷达在左右居中、上下居中）
        const ImVec2 q = side ? ImVec2(c.x + p.x * scale, c.y - p.z * scale) : ImVec2(c.x - p.y * scale, c.y - p.x * scale);
        if (q.x < a.x || q.x > b.x || q.y < a.y || q.y > b.y) continue;
        dl->AddRectFilled(q, ImVec2(q.x + 1.5f, q.y + 1.5f), heightColor(p.z, zMin, zMax));
    }
    const ImU32 head = ImGui::GetColorU32(ImGuiCol_Text);
    if (side) dl->AddCircleFilled(c, 3.0f, head);
    else dl->AddTriangleFilled(ImVec2(c.x, c.y - 7.0f), ImVec2(c.x - 5.0f, c.y + 5.0f), ImVec2(c.x + 5.0f, c.y + 5.0f), head);
}

// M5-11：把 3D 雷达的点按查看器相机投影到视口图像上（叠加显示，不做遮挡；类似 RViz 的点云显示）
void pointCloudOverlay(Frame& f) {
    f.state.overlayPointsDrawn = 0;
    if (!f.state.pointCloudOverlay || !f.simulated) return;
    ImGuiWindow* viewport = ImGui::FindWindowByName("###Viewport");
    const ImVec2 a = f.state.viewportImageMin, b = f.state.viewportImageMax;
    if (viewport == nullptr || viewport->DrawList == nullptr || b.x <= a.x || b.y <= a.y) return;
    const double aspect = static_cast<double>(b.x - a.x) / static_cast<double>(b.y - a.y);
    const auto view = f.camera.viewMatrix();
    const auto proj = f.camera.projectionMatrix(aspect);
    const auto mul = [](const std::array<float, 16>& m, const std::array<float, 4>& v) {
        std::array<float, 4> o{};
        for (int r = 0; r < 4; ++r) {
            o[static_cast<std::size_t>(r)] = m[static_cast<std::size_t>(r)] * v[0] + m[static_cast<std::size_t>(4 + r)] * v[1] +
                                             m[static_cast<std::size_t>(8 + r)] * v[2] + m[static_cast<std::size_t>(12 + r)] * v[3];
        }
        return o;
    };
    ImDrawList* dl = viewport->DrawList;
    dl->PushClipRect(a, b, true);
    for (const auto& scan : f.simulated->lidar3dScans) {
        if (!scan) continue;
        float zMin = 0.0f, zMax = 0.0f;
        std::vector<terra::data::Vec3> world;
        world.reserve(scan->points.size());
        for (std::size_t i = 0; i < scan->points.size(); ++i) {
            const auto& p = scan->points[i];
            const terra::data::Vec3 local{p.x, p.y, p.z};
            const terra::data::Vec3 r = terra::scene::rotate(scan->pose.orientation, local);
            const terra::data::Vec3 w{scan->pose.position.x + r.x, scan->pose.position.y + r.y, scan->pose.position.z + r.z};
            world.push_back(w);
            const auto z = static_cast<float>(w.z);
            zMin = i == 0 ? z : std::min(zMin, z);
            zMax = i == 0 ? z : std::max(zMax, z);
        }
        for (const auto& w : world) {
            const auto clip = mul(proj, mul(view, {static_cast<float>(w.x), static_cast<float>(w.y), static_cast<float>(w.z), 1.0f}));
            if (clip[3] <= 1e-4f) continue;
            const float nx = clip[0] / clip[3], ny = clip[1] / clip[3];
            if (nx < -1.0f || nx > 1.0f || ny < -1.0f || ny > 1.0f) continue;
            const ImVec2 q(a.x + (nx * 0.5f + 0.5f) * (b.x - a.x), a.y + (0.5f - ny * 0.5f) * (b.y - a.y));
            dl->AddRectFilled(ImVec2(q.x - 1.0f, q.y - 1.0f), ImVec2(q.x + 1.0f, q.y + 1.0f), heightColor(static_cast<float>(w.z), zMin, zMax));
            ++f.state.overlayPointsDrawn;
        }
    }
    dl->PopClipRect();
}

void sensorsPanel(Frame& f) {
    if (!f.state.showSensors) return;
    // 已保存的布局中没有这个面板时，首次显示并入执行器面板所在的停靠组
    if (const ImGuiWindow* sibling = ImGui::FindWindowByName("###Actuators"); sibling != nullptr && sibling->DockId != 0) {
        ImGui::SetNextWindowDockID(sibling->DockId, ImGuiCond_FirstUseEver);
    }
    bool visible = true;
    if (ImGui::Begin(label(TextId::PanelSensors, "Sensors").c_str(), &visible)) {
        if (!f.simulated) {
            ImGui::TextDisabled("%s", tr(TextId::OutlinerNoScene));
        } else {
            SimulatedScene& sim = *f.simulated;
            const auto& d = sim.instance.world->description();
            if (sim.replay) ImGui::TextWrapped("%s", tr(TextId::SensorsReplay));  // M4-07：显示记录中的读数
            if (d.sensors.empty() && sim.instance.cameras.empty() && sim.instance.lidars.empty()) {
                ImGui::TextWrapped("%s", tr(TextId::SensorsNone));
            }
            const float font = ImGui::GetFontSize();
            // M4-09：噪声开关（只影响本次运行；记录中途不能切换）
            {
                terra::simulation::SensorNoise probe(sim.instance, true);
                const bool configured = probe.active();
                ImGui::BeginDisabled(!configured || sim.recorder != nullptr);
                bool on = f.state.sensorNoise;
                if (ImGui::Checkbox(tr(TextId::SensorsNoise), &on)) {
                    f.state.sensorNoise = on;
                    sim.noise.setEnabled(on);
                    if (sim.policy) sim.policy->setNoise(on);  // M4-10a
#ifdef TERRA_WITH_ROS2
                    if (sim.ros2) sim.ros2->setSensorNoise(on);
#endif
                    f.state.log.push_back(std::string{tr(TextId::SensorsNoise)} + (on ? "：开" : "：关"));
                }
                ImGui::EndDisabled();
                if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
                    ImGui::SetTooltip("%s", tr(configured ? TextId::SensorsNoiseHint : TextId::SensorsNoiseNone));
                }
                if (configured) {
                    ImGui::SameLine();
                    ImGui::TextDisabled("%s %llu", tr(TextId::SensorsNoiseSeed), static_cast<unsigned long long>(probe.seed()));
                }
            }
            if (!d.sensors.empty()) {
                const std::string header = std::string{tr(TextId::SensorsImu)} + "  (" + std::to_string(d.sensors.size()) + ")###imu";
                if (ImGui::CollapsingHeader(header.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
                    int& selected = f.state.sensorImu;
                    selected = std::clamp(selected, 0, static_cast<int>(d.sensors.size()) - 1);
                    const auto k = static_cast<std::size_t>(selected);
                    ImGui::SetNextItemWidth(-FLT_MIN);
                    if (ImGui::BeginCombo("##imu", d.sensors[k].name.c_str())) {
                        for (std::size_t i = 0; i < d.sensors.size(); ++i) {
                            if (ImGui::Selectable(d.sensors[i].name.c_str(), i == k)) selected = static_cast<int>(i);
                        }
                        ImGui::EndCombo();
                    }
                    const auto& history = sim.imuHistory[k];
                    if (history.empty()) {
                        ImGui::TextDisabled("%s", tr(TextId::SensorsWaiting));
                    } else {
                        const auto& v = history.back().values;
                        ImGui::Text("%s  % .3f  % .3f  % .3f  m/s²", tr(TextId::SensorsAccel), static_cast<double>(v[0]), static_cast<double>(v[1]),
                                    static_cast<double>(v[2]));
                        ImGui::Text("%s  % .3f  % .3f  % .3f  rad/s", tr(TextId::SensorsGyro), static_cast<double>(v[3]), static_cast<double>(v[4]),
                                    static_cast<double>(v[5]));
                        ImGui::TextDisabled("%.0f Hz  ·  %s %llu  ·  t = %.3f s", sim.imuRate(k), tr(TextId::SensorsSamples),
                                            static_cast<unsigned long long>(sim.imuSamples[k]), history.back().time);
                        tripleCurve("##acc", history, 0, "m/s²", font * 5.0f);
                        tripleCurve("##gyro", history, 3, "rad/s", font * 5.0f);
                        if (ImGui::SmallButton(tr(TextId::VizPopPlot))) popOutImuPlot(f, d.sensors[k].name);  // M4-08d
                    }
                }
            }
            if (!sim.instance.lidars.empty()) {
                const std::string header = std::string{tr(TextId::SensorsLidar)} + "  (" + std::to_string(sim.instance.lidars.size()) + ")###lidar";
                if (ImGui::CollapsingHeader(header.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
                    for (std::size_t k = 0; k < sim.instance.lidars.size(); ++k) {
                        ImGui::PushID(static_cast<int>(k));
                        const auto& lidar = sim.instance.lidars[k];
                        ImGui::TextUnformatted(lidar.name.c_str());
                        if (const auto& scan = sim.lidarScans[k]) {
                            std::size_t hits = 0;
                            for (const float r : scan->ranges) hits += std::isfinite(r) ? 1u : 0u;
                            ImGui::SameLine();
                            ImGui::TextDisabled("%.0f Hz  ·  %s %zu / %zu  ·  t = %.3f s", lidar.rateHz, tr(TextId::SensorsHits), hits,
                                                scan->ranges.size(), scan->time);
                            lidarView(*scan, std::min(ImGui::GetContentRegionAvail().x, font * 18.0f));
                        } else {
                            ImGui::TextDisabled("%s", tr(TextId::SensorsWaiting));
                        }
                        ImGui::PopID();
                    }
                }
            }
            if (!sim.instance.lidars3d.empty()) {  // M5-11
                const std::string header = std::string{tr(TextId::SensorsLidar3d)} + "  (" + std::to_string(sim.instance.lidars3d.size()) + ")###lidar3d";
                if (ImGui::CollapsingHeader(header.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
                    ImGui::Checkbox(tr(TextId::SensorsPointOverlay), &f.state.pointCloudOverlay);
                    for (std::size_t k = 0; k < sim.instance.lidars3d.size(); ++k) {
                        ImGui::PushID(static_cast<int>(k));
                        const auto& lidar = sim.instance.lidars3d[k];
                        ImGui::TextUnformatted(lidar.name.c_str());
                        if (const auto& scan = sim.lidar3dScans[k]) {
                            ImGui::SameLine();
                            ImGui::TextDisabled("%.0f Hz  ·  %zu / %zu %s  ·  t = %.3f s", lidar.rateHz, scan->points.size(), scan->rays, tr(TextId::SensorsPoints),
                                                scan->time);
                            const float w = std::min(ImGui::GetContentRegionAvail().x, font * 18.0f);
                            ImGui::TextDisabled("%s", tr(TextId::SensorsTopView));
                            pointCloudView(*scan, w, false);
                            ImGui::TextDisabled("%s", tr(TextId::SensorsSideView));
                            pointCloudView(*scan, w, true);
                        } else {
                            ImGui::TextDisabled("%s", sim.replay ? tr(TextId::SensorsLidar3dReplay) : tr(TextId::SensorsWaiting));
                        }
                        ImGui::PopID();
                    }
                }
            }
            if (!sim.instance.cameras.empty()) {
                const std::string header = std::string{tr(TextId::SensorsCamera)} + "  (" + std::to_string(sim.instance.cameras.size()) + ")###camera";
                if (ImGui::CollapsingHeader(header.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
                    for (std::size_t k = 0; k < sim.instance.cameras.size(); ++k) {
                        ImGui::PushID(static_cast<int>(k));
                        const auto& camera = sim.instance.cameras[k];
                        const auto& preview = sim.cameraPreviews[k];
                        ImGui::TextUnformatted(camera.name.c_str());
                        ImGui::SameLine();
                        ImGui::TextDisabled("%d × %d  ·  %.0f°  ·  %.0f Hz", camera.width, camera.height, camera.fovyDegrees, camera.rateHz);
                        if (!preview.frame) {
                            ImGui::TextDisabled("%s", tr(TextId::SensorsWaiting));
                        } else if (sim.replay && preview.color.name() == 0 && preview.depth.name() == 0) {
                            // M4-07：回放的这一帧在记录中只有元数据（没有保存图像，或这一帧没有渲染）
                            ImGui::TextDisabled("t = %.3f s  ·  %s %llu", preview.frame->time, tr(TextId::SensorsStep),
                                                static_cast<unsigned long long>(preview.frame->step));
                            ImGui::TextDisabled("%s", tr(TextId::SensorsNoImage));
                        } else {
                            const vw::CameraFrame& frame = *preview.frame;
                            ImGui::TextDisabled("t = %.3f s  ·  %s %llu", frame.time, tr(TextId::SensorsStep),
                                                static_cast<unsigned long long>(frame.step));
                            const int shown = (preview.color.name() != 0 ? 1 : 0) + (preview.depth.name() != 0 ? 1 : 0) +
                                              (preview.segmentation.name() != 0 ? 1 : 0) + (preview.thermal.name() != 0 ? 1 : 0);
                            if (!frame.thermal.empty()) {  // M5-06：预设、量程、色板与本帧的温度
                                const std::size_t center = static_cast<std::size_t>(camera.height / 2) * static_cast<std::size_t>(camera.width) +
                                                           static_cast<std::size_t>(camera.width / 2);
                                const auto [lo, hi] = std::minmax_element(frame.thermal.begin(), frame.thermal.end());
                                ImGui::TextDisabled("%s  %s · %s · %.0f～%.0f °C · %s", tr(TextId::SensorsThermal), camera.thermal.preset.c_str(),
                                                    terra::scene::thermalPaletteName(camera.thermal.palette), camera.thermal.rangeMin, camera.thermal.rangeMax,
                                                    camera.thermal.gain.c_str());
                                ImGui::TextDisabled("%s  %.2f / %.2f / %.2f °C", tr(TextId::SensorsThermalSummary), static_cast<double>(*lo),
                                                    static_cast<double>(frame.thermal[std::min(center, frame.thermal.size() - 1)]), static_cast<double>(*hi));
                            }
                            const float gap = ImGui::GetStyle().ItemSpacing.x;
                            const float w = (ImGui::GetContentRegionAvail().x - gap * static_cast<float>(std::max(shown - 1, 0))) /
                                            static_cast<float>(std::max(shown, 1));
                            // 显示旋转（M4-04）：只影响显示，按旋转后的四个角贴图；悬停取深度用同一个换算
                            const int rotation = camera.displayRotation;
                            const auto aspect = static_cast<float>(terra::scene::displayAspect(rotation, camera.width, camera.height));
                            const ImVec2 size(w, w / std::max(aspect, 0.01f));
                            const auto uvAt = [rotation](double dx, double dy) {
                                const auto t = terra::scene::displayToImage(rotation, dx, dy);
                                return ImVec2(static_cast<float>(t[0]), static_cast<float>(t[1]));
                            };
                            int index = 0;
                            for (const auto* texture : {&preview.color, &preview.depth, &preview.segmentation, &preview.thermal}) {
                                if (texture->name() == 0) continue;
                                if (index > 0) ImGui::SameLine();
                                ImGui::PushID(index++);
                                ImGui::InvisibleButton("##image", size);
                                const ImVec2 a = ImGui::GetItemRectMin();
                                const ImVec2 b = ImGui::GetItemRectMax();
                                ImGui::GetWindowDrawList()->AddImageQuad(static_cast<ImTextureID>(texture->name()), a, ImVec2(b.x, a.y), b, ImVec2(a.x, b.y),
                                                                         uvAt(0, 0), uvAt(1, 0), uvAt(1, 1), uvAt(0, 1));
                                // M4-09：分割图上悬停显示该像素的实体
                                if (texture == &preview.thermal && ImGui::IsItemHovered() && !frame.thermal.empty()) {  // M5-06：悬停显示温度
                                    const ImVec2 at = ImGui::GetIO().MousePos;
                                    const auto t = terra::scene::displayToImage(rotation, static_cast<double>((at.x - a.x) / size.x),
                                                                                static_cast<double>((at.y - a.y) / size.y));
                                    const int px = std::clamp(static_cast<int>(t[0] * camera.width), 0, camera.width - 1);
                                    const int py = std::clamp(static_cast<int>(t[1] * camera.height), 0, camera.height - 1);
                                    const float celsius = frame.thermal[static_cast<std::size_t>(py) * static_cast<std::size_t>(camera.width) + static_cast<std::size_t>(px)];
                                    ImGui::SetTooltip("(%d, %d)  %s %.2f °C", px, py, tr(TextId::SensorsTemperatureAt), static_cast<double>(celsius));
                                } else if (texture == &preview.segmentation && ImGui::IsItemHovered() && !frame.segmentation.empty()) {
                                    const ImVec2 at = ImGui::GetIO().MousePos;
                                    const auto t = terra::scene::displayToImage(rotation, static_cast<double>((at.x - a.x) / size.x),
                                                                                static_cast<double>((at.y - a.y) / size.y));
                                    const int px = std::clamp(static_cast<int>(t[0] * camera.width), 0, camera.width - 1);
                                    const int py = std::clamp(static_cast<int>(t[1] * camera.height), 0, camera.height - 1);
                                    const std::uint32_t id = frame.segmentation[static_cast<std::size_t>(py) * static_cast<std::size_t>(camera.width) + static_cast<std::size_t>(px)];
                                    const auto* entity = id > 0 ? sim.instance.world->description().entity(terra::data::EntityId{id}) : nullptr;
                                    ImGui::SetTooltip("(%d, %d)  %s %u  %s", px, py, tr(TextId::SensorsSegmentation), id, entity ? entity->name.c_str() : "—");
                                } else if (ImGui::IsItemHovered() && !frame.depth.empty()) {  // 悬停时显示该像素的深度
                                    const ImVec2 at = ImGui::GetIO().MousePos;
                                    const auto t = terra::scene::displayToImage(rotation, static_cast<double>((at.x - a.x) / size.x),
                                                                                static_cast<double>((at.y - a.y) / size.y));
                                    const int px = std::clamp(static_cast<int>(t[0] * camera.width), 0, camera.width - 1);
                                    const int py = std::clamp(static_cast<int>(t[1] * camera.height), 0, camera.height - 1);
                                    const float depth = frame.depth[static_cast<std::size_t>(py) * static_cast<std::size_t>(camera.width) + static_cast<std::size_t>(px)];
                                    if (std::isfinite(depth)) ImGui::SetTooltip("(%d, %d)  %s %.3f m", px, py, tr(TextId::SensorsDepthAt), static_cast<double>(depth));
                                    else ImGui::SetTooltip("(%d, %d)  %s ∞", px, py, tr(TextId::SensorsDepthAt));
                                }
                                ImGui::PopID();
                            }
                            if (rotation != 0) ImGui::TextDisabled("%s %d°", tr(TextId::SensorsRotated), rotation);
                        }
                        ImGui::PopID();
                    }
                }
            }
        }
    }
    ImGui::End();
    if (!visible) f.state.showSensors = false;
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

// 资源库面板（M4-08c）：场景、动作、机器人的卡片，单击即打开
void libraryPanel(Frame& f) {
    if (!f.state.showLibrary) return;
    // 已保存的布局中没有这个面板时，首次显示并入大纲所在的停靠组
    if (const ImGuiWindow* sibling = ImGui::FindWindowByName("###Outliner"); sibling != nullptr && sibling->DockId != 0) {
        ImGui::SetNextWindowDockID(sibling->DockId, ImGuiCond_FirstUseEver);
    }
    bool visible = true;
    std::optional<vw::LibraryAction> action;
    if (ImGui::Begin(label(TextId::PanelLibrary, "Library").c_str(), &visible)) action = vw::drawLibraryPanel(f.state.libraryPanel, libraryContext(f));
    ImGui::End();
    if (!visible) f.state.showLibrary = false;
    if (action) handleLibraryAction(f, *action);
}

// ---- 策略（M4-10a）----

constexpr const char* kComponentCatalog = TERRA_ASSETS_DIR "/components/components.json";

void stopPolicy(Frame& f) {
    if (!f.simulated || !f.simulated->policy) return;
    f.state.log.push_back(std::string{tr(TextId::LogPolicyStopped)} + f.simulated->policy->policy().name);
    std::printf("INFO 策略已停止：%s\n", f.simulated->policy->policy().name.c_str());
    f.simulated->policy.reset();
    f.simulated->pd.clearTargets();  // PD 层（若有）以当前姿态为目标接回
}

vw::PolicyContext policyContext(const Frame& f) {
    vw::PolicyContext ctx;
    if (f.simulated) {
        ctx.scene = &f.simulated->instance;
        ctx.scenePath = f.simulated->path;
        ctx.running = f.simulated->policy.get();
    }
    ctx.componentsDirectory = terra::library::componentsDirectory(libraryFilePath(f));
    ctx.catalogPath = kComponentCatalog;
    ctx.noise = f.state.sensorNoise;
    ctx.viewportMin = f.state.viewportImageMin;
    ctx.viewportMax = f.state.viewportImageMax;
    ctx.viewportHovered = f.state.viewportHovered;
    ctx.keyboardFree = !f.uiCtx.wantsTextInput() && !f.state.settingsWindow.capturingKey();
    // M4-10b：导入向导
    for (const auto& a : f.state.assetList) {
        const auto has = [&](const char* t) { return std::find(a.tags.begin(), a.tags.end(), t) != a.tags.end(); };
        if (a.kind == terra::library::AssetKind::Scene && !a.hidden && !a.missing && !has("invalid") && !has("models-missing")) {
            ctx.scenes.emplace_back(a.path, a.title);
        }
    }
    ctx.policiesDirectory = (std::filesystem::path{libraryFilePath(f)}.parent_path() / "policies").string();
    ctx.converter = TERRA_SOURCE_DIR "/scripts/policy/torchscript_to_onnx.py";
    return ctx;
}

// M4-10b：运行说明文件；场景不是当前场景时先打开它（帧末），打开后再运行
void runPolicyFile(Frame& f, const std::string& policyFile, const std::string& scene) {
    if (!scene.empty() && (!f.simulated || f.simulated->path != scene)) {
        f.state.openRequest = scene;
        f.state.pendingPolicy = policyFile;
        return;
    }
    if (!f.simulated) return;
    vw::requestPolicy(f.state.policy, policyFile);
    f.state.policy.dialogOpen = false;
    f.state.policy.startRequested = true;
    f.simulated->runner->resume();
}

void policyPanels(Frame& f) {
    auto ctx = policyContext(f);
#ifdef TERRA_WITH_ROS2
    // M4-10b：开启 ROS2 时，策略的速度指令也可以来自 /cmd_vel（最近一条；0.5 s 没有消息时归零）
    if (f.simulated && f.simulated->policy && f.simulated->ros2) {
        auto& sim = *f.simulated;
        if (!sim.velocitySubscribed) {
            sim.velocitySubscribed = true;
            if (!sim.ros2->enableVelocityCommand()) f.state.log.push_back("/cmd_vel 已用于差速底盘，策略的速度指令只取面板");
        }
        if (const auto v = sim.ros2->velocityCommand()) {
            for (std::size_t i = 0; i < 3; ++i) f.state.policy.command[i] = static_cast<float>((*v)[i]);
            ctx.velocityFromRos2 = true;
        }
    }
#endif
    auto action = vw::drawPolicyUi(f.state.policy, ctx);
    for (auto& line : f.state.policy.log) {
        std::printf("INFO 策略：%s\n", line.c_str());
        f.state.log.push_back(std::move(line));
    }
    f.state.policy.log.clear();
    if (action.type == vw::PolicyAction::Type::Start && f.simulated) {
        f.simulated->policy = std::move(action.controller);
        const auto& c = *f.simulated->policy;
        f.state.log.push_back(std::string{tr(TextId::LogPolicyStarted)} + action.name);
        std::printf("INFO 策略开始运行：%s（%s；降频 %d；接管 %zu 个执行器）\n", action.name.c_str(), c.model().describe().c_str(), c.decimation(),
                    c.actuators().size());
    }
    if (action.type == vw::PolicyAction::Type::Imported || action.type == vw::PolicyAction::Type::OpenAndRun) {
        f.state.assetsStale = true;  // 下一帧重新扫描，策略卡片出现在资源库中
        f.state.policyImportTest.imported = true;
        ++f.state.policyImportTest.imports;
        f.state.policyImportTest.policyFile = action.policyFile;
        if (action.type == vw::PolicyAction::Type::OpenAndRun) runPolicyFile(f, action.policyFile, action.scene);
    }
    if (f.simulated && f.simulated->policy && f.simulated->policy->failed()) {
        f.state.log.push_back(std::string{tr(TextId::LogPolicyStopped)} + f.simulated->policy->error());
        std::printf("INFO 策略出错：%s\n", f.simulated->policy->error().c_str());
        stopPolicy(f);
    }
}

// M4-10a 自检：命令行 --policy 打开后运行；第 20 帧设速度指令；倒数第 5 帧停止
void policySelfTestScript(Frame& f, int frames, int total) {
    auto& t = f.state.policyTest;
    if (!f.simulated) return;
    auto& sim = *f.simulated;
    if (sim.policy) {
        t.started = true;
        t.inferences = sim.policy->inferences();
    }
    t.panel = f.state.policy.commandPanelFrames > 0;
    if (frames == 20) f.state.policy.command = {0.4f, 0.0f, 0.0f};
    if (frames == 22 && sim.policy) t.commanded = std::fabs(sim.policy->command()[0] - 0.4) < 1e-6;
    if (frames == total - 5) stopPolicy(f);
    if (frames == total - 3) t.stopped = sim.policy == nullptr && t.started;
}

// M4-10b 自检：导入向导。第 3 帧打开向导（夹具 bench_supermimic.onnx）；检查完场景后按“导入并运行”；
// 导入后场景打开、资源库登记了策略；运行时需要的组件不在（自检的资源库没有装 ONNX Runtime）→ 弹出“需要下载组件”
void policyImportSelfTestScript(Frame& f, int frames, const std::string& file) {
    auto& t = f.state.policyImportTest;
    auto& w = f.state.policy.importer;
    if (frames == 3) {
        vw::requestPolicyImport(f.state.policy, file);
        t.opened = true;
    }
    if (t.stage == 0 && t.opened && w.open && !w.file.empty() && w.file == w.inspected && w.scanned() && !w.candidates.empty()) {
        t.scanned = true;
        if (w.selected >= 0) {
            const auto& best = w.candidates[static_cast<std::size_t>(w.selected)];
            t.bestFull = best.scene.find("policy_bench.terra-scene.json") != std::string::npos && best.match.complete();
            t.extraWarned = best.match.extra == std::vector<std::string>{"bench/j3"};
        }
        w.selfTestImport = true;  // 导入并运行
        t.stage = 1;
    }
    if (t.stage == 1 && t.imported) t.stage = 2;
    if (t.stage >= 2) {
        if (f.simulated && f.simulated->path.find("policy_bench.terra-scene.json") != std::string::npos) t.sceneOpened = true;
        if (f.state.policy.neededComponent == "onnxruntime-cpu") t.componentAsked = true;
        for (const auto& a : f.state.assetList) {
            if (a.kind == terra::library::AssetKind::Policy && a.path == t.policyFile) t.registered = true;
        }
    }
    // 2026-11-25 展示中发现：菜单重开向导（未选文件）后沿用了上一个文件的结果，按“导入”得到“文件不存在：”。重开后按导入应无效
    if (t.stage == 2 && t.sceneOpened && t.componentAsked) {
        f.state.policy.neededComponent.clear();
        vw::requestPolicyImport(f.state.policy, {});
        w.selfTestImport = true;
        t.importsBeforeEmpty = t.imports;
        t.stage = 3;
    }
}

int policyImportSelfTest(Frame& f) {
    int failures = 0;
    const auto check = [&](bool ok, const std::string& what) {
        std::printf("%s %s\n", ok ? "PASS" : "FAILED:", what.c_str());
        if (!ok) ++failures;
    };
    const auto& t = f.state.policyImportTest;
    std::printf("INFO 导入向导自检：导入 %s\n", t.policyFile.c_str());
    check(t.opened && t.scanned, "导入向导：打开并检查资源库中的场景");
    check(t.bestFull, "按关节名推荐：policy_bench 完全包含策略的关节");
    check(t.extraWarned, "列出机器人多出的关节（bench/j3）");
    check(t.imported && !t.policyFile.empty(), "导入并运行：生成说明文件");
    check(t.registered, "资源库登记为策略（策略分页、场景卡片的标签）");
    check(t.sceneOpened, "打开推荐场景");
    check(t.componentAsked, "需要的组件不在：弹出“需要下载组件”");
    check(t.stage == 3 && t.imports == t.importsBeforeEmpty, "菜单重开向导、还没选文件时不能导入（不沿用上一次的结果）");
    check(f.ctx.errorMessageCount() == 0, "整个运行期间没有 GL 错误");
    std::printf(failures == 0 ? "导入向导自检通过\n" : "导入向导自检失败：%d 项\n", failures);
    return failures == 0 ? 0 : 1;
}

int policySelfTest(Frame& f) {
    int failures = 0;
    const auto check = [&](bool ok, const std::string& what) {
        std::printf("%s %s\n", ok ? "PASS" : "FAILED:", what.c_str());
        if (!ok) ++failures;
    };
    const auto& t = f.state.policyTest;
    std::printf("INFO 策略自检：推理 %llu 次；速度指令面板绘制 %d 帧\n", static_cast<unsigned long long>(t.inferences), f.state.policy.commandPanelFrames);
    check(t.started, "命令行 --policy：打开场景后自动配置并运行");
    check(t.inferences >= 5, "运行中按控制周期推理");
    check(t.panel, "视口上方的速度指令面板");
    check(t.commanded, "面板中的速度指令传给策略");
    check(t.stopped, "停止策略");
    check(f.ctx.errorMessageCount() == 0, "整个运行期间没有 GL 错误");
    std::printf(failures == 0 ? "策略自检通过\n" : "策略自检失败：%d 项\n", failures);
    return failures == 0 ? 0 : 1;
}

void buildUi(Frame& f) {
    syncLibraryLocation(f);  // M4-08c
    copyLibraryIfRequested(f);
    if (f.state.assetsStale) refreshAssets(f);
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
    libraryPanel(f);  // 在大纲之前：首次显示时并入大纲所在的停靠组
    outlinerPanel(f);
    detailsPanel(f);
    actuatorPanel(f);
    sensorsPanel(f);  // 在执行器面板之后：首次显示时并入它所在的停靠组
    pointCloudOverlay(f);  // M5-11：在视口上叠加 3D 雷达的点云
    logPanel(f);
    statsPanel(f);
    timelinePanel(f);  // 在日志之后：首次显示时并入日志所在的停靠组
    vizPanels(f);      // M4-08d：显示配置中的窗口（ROS2 面板、消息、曲线、图像、TF 树）
    policyPanels(f);   // M4-10a：运行策略、组件、速度指令面板
    helpWindows(f);
    vw::drawSettingsWindow(f.state.settingsWindow, f.settings.current, f.state.monitors, f.uiCtx.scale(), f.settings.path,
                           f.state.log);
    if (auto chosen = f.state.fileBrowser.draw(label(TextId::FileBrowserTitle, "OpenScene").c_str())) f.state.openRequest = *chosen;
    if (auto chosen = f.state.motionBrowser.draw(label(TextId::MotionBrowserTitle, "OpenMotion").c_str())) f.state.motionRequest = *chosen;
    // M4-08a：导入向导
    auto& wizard = f.state.importWizard;
    if (auto chosen = wizard.browser.draw(label(TextId::ImportBrowserTitle, "ImportBrowser").c_str())) vw::beginImport(wizard, *chosen, importDirectory(f));
    if (auto imported = vw::drawImportWizard(wizard, terra::physics::mujoco::composeWorld)) {
        f.state.openRequest = *imported;
        writableLibrary(f);  // M4-08c：导入后登记（下一帧重新扫描）
        f.state.assetsStale = true;
    }
    if (auto chosen = f.state.packBrowser.draw(label(TextId::PackBrowserTitle, "PackBrowser").c_str())) importPackFile(f, *chosen);
    if (auto action = vw::drawQuickOpen(f.state.libraryPanel, libraryContext(f))) handleLibraryAction(f, *action);
    if (wizard.open && ImGui::FindWindowByName("###ImportWizard") != nullptr) f.state.importWizardSeen = true;
    for (auto& line : wizard.log) f.state.log.push_back(std::move(line));
    wizard.log.clear();
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
    // M3-07c：记录与回放自检
    bool recordTest = false;
    std::size_t replayIndex = 0;  // 回放跳到的状态下标
    bool sensorTest = false;      // M4-04：传感器自检（播放后不暂停、不单步）
    bool lidar3dTest = false;     // M5-11：3D 雷达自检（同上）
    bool thermalTest = false;     // M5-06：热像仪自检（同上）
    bool elementsTest = false;    // M5-02a：元素自检（播放；中途单击元素卡片打开预览）
    double crateHeightAtSwitch = -1.0;
    std::string firstScene;
    bool previewRequested = false;
    bool vizTest = false;         // M4-08d：显示配置自检（播放后一直运行）
    bool sensorRecordTest = false;  // M4-07：传感器记录自检（第 6 帧开始记录并保存图像，中间停止并回放到一帧有图像的时刻）
    std::uint64_t replayCameraStep = 0;
    double replayCameraTime = -1.0;
    bool toggleStarted = false;   // M3-07d：开始 / 停止切换（工具栏、快捷键共用的路径）确实开始与停止了记录
    bool toggleStopped = false;
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
    if (sceneRecord.sensorRecordTest) {
        if (frame == 6) {
            f.state.recordCameraImages = true;
            startRecording(f);
        }
        if (frame == total / 2) {
            stopRecording(f, "stopped");
            if (startReplay(f, f.state.lastRecordPath) && f.simulated->replay) {
                auto& replay = *f.simulated->replay;
                replay.playing = false;
                // 回放到后半段中一帧保存了图像的相机帧
                const auto& frames = replay.record.cameraFrames;
                for (std::size_t i = frames.size() / 2; i < frames.size(); ++i) {
                    if (frames[i].rgb.empty() || frames[i].depth.empty()) continue;
                    sceneRecord.replayCameraStep = frames[i].step;
                    sceneRecord.replayCameraTime = frames[i].time;
                    replay.time = frames[i].time;
                    break;
                }
            }
        }
        return;
    }
    if (sceneRecord.elementsTest && frame == total / 2) {  // M5-02a：记下木箱的高度，再像单击卡片一样打开元素的预览
        sceneRecord.firstScene = sim.path;
        if (const auto* e = sim.instance.world->description().findEntity("crate_a/body")) {
            sceneRecord.crateHeightAtSwitch = sim.instance.world->snapshot()->entities[e->id.value - 1].pose.position.z;
        }
        for (const auto& a : f.state.assetList) {
            if (a.kind != terra::library::AssetKind::Element || a.path.find("simple_cabinet.terra-element.json") == std::string::npos) continue;
            handleLibraryAction(f, vw::LibraryAction{vw::LibraryAction::Type::Open, terra::library::AssetKind::Element, a.path, {}, false});
            sceneRecord.previewRequested = true;
        }
    }
    if (sceneRecord.sensorTest || sceneRecord.vizTest || sceneRecord.lidar3dTest || sceneRecord.thermalTest || sceneRecord.elementsTest) return;  // M4-04、M4-08d、M5-11、M5-06、M5-02a：第 3 帧按播放后一直运行
    if (sceneRecord.recordTest) {
        // 第 6 帧开始记录（动作从头播放），中间停止并回放，再跳到记录中间的一份状态；
        // 开始与停止都按工具栏上的录制按钮（M3-07d：与点击走同一条路径，包括按下后同一帧内的绘制）
        if (frame == 6) f.state.selfTestPressRecord = true;
        if (frame == 7) sceneRecord.toggleStarted = f.simulated->recorder != nullptr;
        if (frame == total / 2) f.state.selfTestPressRecord = true;
        if (frame == total / 2 + 1) {
            sceneRecord.toggleStopped = sceneRecord.toggleStarted && f.simulated->recorder == nullptr;
            startReplay(f, f.state.lastRecordPath);
        }
        if (frame == total / 2 + 3 && f.simulated->replay) {
            auto& replay = *f.simulated->replay;
            sceneRecord.replayIndex = replay.record.states.size() / 2;
            replay.time = replay.record.states[sceneRecord.replayIndex].simulationTime;
            replay.playing = false;
            ImGui::SetWindowFocus("###Actuators");  // 执行器面板与详情同在右侧一组：切到前面才会绘制
        }
        return;
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

// ---- 显示配置与 ROS2 面板自检（M4-08d）----
// 仿真来源的曲线（传感器面板“弹出为曲线窗口”）；有 --ros2 时另开 ROS2 面板，打开 IMU 的消息、图像、TF 树与 IMU 曲线；
// 保存到资源库 → 关闭全部窗口并重开场景 → 自动恢复；按导出弹窗的默认值导出 .terviz 与 .rviz。
void vizSelfTestScript(Frame& f, int frame) {
    auto& t = f.state.vizTest;
    auto& viz = f.state.viz;
    const double now = f.platform.time();
    const auto next = [&](int phase) {
        t.phase = phase;
        t.phaseStart = now;
    };
    if (!f.simulated || frame < 4) return;
    switch (t.phase) {
        case 0: {
            // 仿真必须在运行（桥接按步发布）：带 --ros2 启动时已经在运行，第 3 帧的“播放 / 暂停”会把它暂停
            if (f.simulated->runner->paused()) f.simulated->runner->resume();
            const auto& sensors = f.simulated->instance.world->description().sensors;
            if (!sensors.empty()) popOutImuPlot(f, sensors.front().name);
            if (t.ros2) toggleRos2Panel(f, true);
            next(1);
            break;
        }
        case 1: {
#ifdef TERRA_WITH_ROS2
            if (t.ros2 && viz.monitor) {
                for (const auto& topic : viz.monitor->topics()) {
                    if (t.imuTopic.empty() && topic.type == "sensor_msgs/msg/Imu") t.imuTopic = topic.name;
                    if (t.imageTopic.empty() && topic.type == "sensor_msgs/msg/Image" && topic.name.find("depth") == std::string::npos) t.imageTopic = topic.name;
                }
                if ((t.imuTopic.empty() || t.imageTopic.empty()) && now - t.phaseStart < 15.0) break;
                std::printf("INFO 自检：IMU 话题 %s，图像话题 %s\n", t.imuTopic.c_str(), t.imageTopic.c_str());
                terra::viz::VizWindow w;
                w.type = terra::viz::WindowType::Message;
                w.topic = t.imuTopic;
                vw::openWindow(viz, w);
                w.type = terra::viz::WindowType::Image;
                w.topic = t.imageTopic;
                vw::openWindow(viz, w);
                terra::viz::VizWindow tf;
                tf.type = terra::viz::WindowType::Tf;
                vw::openWindow(viz, tf);
                terra::viz::PlotSeries series;
                series.topic = t.imuTopic;
                series.field = "linear_acceleration.z";
                vw::addSeries(viz, series, t.imuTopic);
            }
#endif
            next(2);
            break;
        }
        case 2: {
            if (f.simulated->runner->paused()) f.simulated->runner->resume();
            if (now - t.phaseStart < 3.0) break;
            // ROS2：数据到达前最多再等 9 秒（桥接与监视器的发现需要时间）
            if (t.ros2 && (viz.messageTrees == 0 || viz.imageUploads == 0) && now - t.phaseStart < 12.0) break;
            t.simPlot = viz.plotPoints > 0;
#ifdef TERRA_WITH_ROS2
            if (t.ros2 && viz.monitor) {
                t.message = viz.messageTrees > 0;
                t.image = viz.imageUploads > 0;
                t.plot = !viz.monitor->samples(t.imuTopic, "linear_acceleration.z").empty();
                t.tf = !viz.monitor->transforms().empty();
                std::printf("INFO 自检：消息树 %zu 次，图像上传 %zu 次，ROS2 曲线点 %zu，TF %zu 个；仿真 %llu 步\n", viz.messageTrees, viz.imageUploads,
                            viz.monitor->samples(t.imuTopic, "linear_acceleration.z").size(), viz.monitor->transforms().size(),
                            static_cast<unsigned long long>(f.simulated->instance.world->stepIndex()));
                for (const std::string& topic : {t.imuTopic, t.imageTopic}) {
                    const auto st = viz.monitor->stats(topic);
                    const auto info = viz.monitor->topic(topic);
                    std::printf("INFO 自检：%s 已订阅 %s，收到 %llu 条（%.1f Hz），类型问题“%s”，QoS %s\n", topic.c_str(), viz.monitor->watching(topic) ? "是" : "否",
                                static_cast<unsigned long long>(st.messages), st.hz, viz.monitor->typeError(topic).c_str(), info ? info->qos.c_str() : "?");
                }
            }
#endif
            saveDisplay(f, "自检");
            std::error_code ec;
            t.savedPath = viz.path;
            t.saved = !viz.path.empty() && std::filesystem::exists(viz.path, ec) && !viz.dirty;
            t.savedWindows = viz.config.windows.size();
            next(3);
            break;
        }
        case 3: {
            if (f.state.assetsStale) break;  // 等资源库重新扫描
            for (const auto& a : f.state.assetList) {
                if (a.kind == terra::library::AssetKind::Display && a.path == t.savedPath && !a.related.empty() && a.related[0] == f.simulated->path) t.registered = true;
            }
            vw::applyConfig(viz, terra::viz::VizConfig{}, {});
            f.state.openRequest = f.simulated->path;  // 重开场景：应自动恢复
            next(4);
            break;
        }
        case 4: {
            t.restored = viz.path == t.savedPath && viz.config.windows.size() == t.savedWindows && t.savedWindows > 0;
            std::printf("INFO 自检：重开场景后显示配置 %s，%zu 个窗口（保存时 %zu 个）\n", viz.path.c_str(), viz.config.windows.size(), t.savedWindows);
            f.simulated->runner->resume();
            viz.exportSelfTest = true;
            next(5);
            break;
        }
        case 5: {
            const auto dir = std::filesystem::path{libraryFilePath(f)}.parent_path() / "exports";
            std::error_code ec;
            const auto terviz = terra::viz::readViz((dir / "自检.terviz").string());
            const bool rviz = std::filesystem::exists(dir / "自检.rviz", ec);
            t.exported = terviz.config.has_value() && terviz.config->windows.size() == t.savedWindows && rviz;
            if (rviz) {
                std::ifstream in(dir / "自检.rviz");
                const std::string text{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
                t.rvizHasImage = text.find("rviz_default_plugins/Image") != std::string::npos && text.find("rviz_default_plugins/TF") != std::string::npos;
            }
            next(6);
            break;
        }
        default: break;
    }
}

int vizSelfTest(Frame& f) {
    int failures = 0;
    const auto check = [&](bool ok, const std::string& what) {
        std::printf("%s %s\n", ok ? "PASS" : "FAILED:", what.c_str());
        if (!ok) ++failures;
    };
    const auto& t = f.state.vizTest;
    check(t.phase == 6, "自检的各阶段都完成（阶段 " + std::to_string(t.phase) + "）");
    check(t.simPlot, "仿真来源的曲线（传感器面板弹出）有数据");
    if (t.ros2) {
        check(!t.imuTopic.empty() && !t.imageTopic.empty(), "ROS2 面板发现桥接发布的 IMU 与图像话题");
        check(t.message, "消息窗口：运行时展开 IMU 消息");
        check(t.image, "图像窗口：收到并显示相机图像");
        check(t.plot, "ROS2 曲线：IMU 的 linear_acceleration.z 有数据");
        check(t.tf, "TF 树：收到 /tf 或 /tf_static");
    }
    check(t.saved, "显示配置保存到资源库目录：" + t.savedPath);
    check(t.registered, "显示配置登记到资源库，关联当前场景");
    check(t.restored, "关闭全部窗口、重开场景后自动恢复");
    check(t.exported, "导出 .terviz（可以再读入）与 .rviz");
    if (t.ros2) check(t.rvizHasImage, ".rviz 中有 Image 与 TF 显示");
    check(f.ctx.errorMessageCount() == 0, "整个运行期间没有 GL 错误");
    std::printf(failures == 0 ? "显示配置自检通过\n" : "显示配置自检失败：%d 项\n", failures);
    return failures == 0 ? 0 : 1;
}

// 记录与回放自检（M3-07c）：查看器中记录的一段运行能复现（逐位相同）、已加入资源库、回放的画面与记录逐位相同
int recordSelfTest(Frame& f) {
    int failures = 0;
    const auto check = [&](bool ok, const std::string& what) {
        std::printf("%s %s\n", ok ? "PASS" : "FAILED:", what.c_str());
        if (!ok) ++failures;
    };
    const std::string path = f.state.lastRecordPath;
    std::printf("INFO 记录文件：%s\n", path.c_str());
    const auto read = terra::simulation::readRecord(path);
    check(read.record.has_value() && read.errors.empty() && read.warnings.empty(), "记录可以读回");
    if (read.record) {
        const auto& rec = *read.record;
        check(rec.summary && rec.summary->reason == "stopped" && rec.states.size() > 2 && !rec.controls.empty(),
              "记录完整：" + std::to_string(rec.states.size()) + " 份状态、" + std::to_string(rec.controls.size()) + " 行控制量，结束原因 stopped");
        check(rec.header.sampleEvery == 10, "默认 50 Hz：步长 0.002 s 时每 10 步采样一次（实际 " + std::to_string(rec.header.sampleEvery) + "）");
        check(rec.header.startKeyframe == "arm/rest" && !rec.header.motionPath.empty(), "记录从动作的起始关键帧开始");
        const auto rerun = terra::simulation::rerunRecord(rec, terra::physics::mujoco::composeWorld);
        check(rerun.ran && rerun.bitwise, "查看器中记录的运行可以复现，" + std::to_string(rerun.compared) + " 个采样点逐位相同" +
                                              (rerun.firstDifferenceStep ? "（第一个不同的在第 " + std::to_string(*rerun.firstDifferenceStep) + " 步）" : ""));
        if (auto* lib = recordLibrary(f)) {
            const auto entry = lib->record(path);
            check(entry && entry->reason == "stopped" && entry->steps == rec.summary->steps, "记录已加入资源库：" + lib->path());
        } else {
            check(false, "资源库可以打开");
        }
        if (f.simulated && f.simulated->replay) {
            const auto& expected = rec.states.at(sceneRecord.replayIndex);
            const auto poses = f.simulated->replayPoses();
            bool same = poses.size() == expected.entities.size();
            for (std::size_t i = 0; same && i < poses.size(); ++i) {
                const auto& p = poses[i];
                const auto& q = expected.entities[i].pose;
                same = p.position.x == q.position.x && p.position.y == q.position.y && p.position.z == q.position.z &&
                       p.orientation.w == q.orientation.w && p.orientation.x == q.orientation.x && p.orientation.y == q.orientation.y &&
                       p.orientation.z == q.orientation.z;
            }
            check(same, "回放到第 " + std::to_string(expected.stepIndex) + " 步：画面位姿与记录逐位相同");
            check(f.simulated->instance.world->stepIndex() == 0 || f.simulated->runner->paused(), "回放期间不推进物理");
            // M3-07d：回放时执行器面板与记录同步——实测按类型取记录中的关节状态，目标为记录中该时刻生效的控制量
            const auto& d = f.simulated->instance.world->description();
            const auto controls = terra::simulation::controlsAt(rec, expected.stepIndex > 0 ? expected.stepIndex - 1 : 0);
            std::size_t compared = 0;
            bool synced = f.state.actuatorShownStep == expected.stepIndex;
            for (const auto& a : d.actuators) {
                const auto shown = f.state.actuatorShown.find(a.id.value);
                if (shown == f.state.actuatorShown.end() || a.joint.value == 0) continue;
                const auto& js = expected.joints.at(a.joint.value - 1);
                const auto ctrl = controls.find(a.id.value);
                const std::optional<double> target =
                    ctrl != controls.end() && !ctrl->second.empty() ? std::optional<double>(ctrl->second[0]) : std::nullopt;
                double measured = target.value_or(0.0);  // 记录中没有执行器输出：力矩执行器显示目标
                if (a.kind == terra::data::ActuatorKind::Position) measured = js.position[0];
                if (a.kind == terra::data::ActuatorKind::Velocity) measured = js.velocity[0];
                const bool match = shown->second.first == measured && shown->second.second == target;
                if (!match) {
                    std::printf("INFO 执行器 %s：面板 %.17g / %s，记录 %.17g / %s\n", a.name.c_str(), shown->second.first,
                                shown->second.second ? std::to_string(*shown->second.second).c_str() : "-", measured,
                                target ? std::to_string(*target).c_str() : "-");
                }
                synced = synced && match;
                ++compared;
            }
            check(synced && compared == d.actuators.size(), "回放时执行器面板跟随记录：" + std::to_string(compared) +
                                                                " 个执行器的实测与目标都等于记录中第 " + std::to_string(expected.stepIndex) + " 步的值");
        } else {
            check(false, "回放已开始");
        }
        check(sceneRecord.toggleStarted && sceneRecord.toggleStopped, "工具栏录制按钮开始并停止了记录（停止后同一帧的绘制不出错）");
        bool listed = false;
        const std::string normalized = std::filesystem::absolute(path).lexically_normal().string();
        for (const auto& r : f.state.recordList) listed = listed || r.path == normalized;
        check(listed, "时间轴的记录列表中有刚保存的记录（" + std::to_string(f.state.recordList.size()) + " 条）");
        const ImGuiWindow* timeline = ImGui::FindWindowByName("###Timeline");
        const ImGuiWindow* logWindow = ImGui::FindWindowByName("###Log");
        check(timeline != nullptr && logWindow != nullptr && timeline->DockNode != nullptr && timeline->DockNode == logWindow->DockNode,
              "“时间轴”标签页与“日志”停靠在同一组");
    }
    check(f.ctx.errorMessageCount() == 0, "整个运行期间没有 GL 错误");
    return failures == 0 ? 0 : 1;
}

// 传感器自检（M4-04）：场景 assets/tests/sensors/sensor_suite.terra-scene.json（IMU 夹具、雷达房间、相机夹具）。
// 采样次数与 Headless、terra_capture 的采样规则一致；雷达与相机的读数与 M4-03 的解析解一致；面板与执行器面板同组。
// M4-08a：导入向导自检（--import-self-test）
int importSelfTest(Frame& f) {
    int failures = 0;
    const auto check = [&](bool ok, const std::string& what) {
        std::printf("%s %s\n", ok ? "PASS" : "FAILED:", what.c_str());
        if (!ok) ++failures;
    };
    const auto& wizard = f.state.importWizard;
    check(f.state.importWizardSeen, "导入向导窗口出现过（第 12 帧分析之后）");
    check(wizard.result.has_value() && wizard.result->ok, "导入成功");
    if (wizard.result) {
        for (const auto& i : wizard.result->issues) {
            std::printf("INFO [%s] %s：%s\n", terra::importer::severityName(i.severity), i.code.c_str(), i.message.c_str());
        }
        std::printf("INFO 导入目录：%s\n", wizard.result->directory.c_str());
        check(!wizard.open, "导入后向导关闭");
        const std::string expected = std::filesystem::absolute(wizard.result->scene).lexically_normal().string();
        check(f.simulated && f.simulated->path == expected, "打开了生成的场景：" + expected);
        check(wizard.result->directory.rfind(importDirectory(f), 0) == 0, "导入目录在资源库目录下的 assets/ 中");
        // M4-08c：导入后登记到资源库，卡片来源为“导入”
        const auto registered = std::find_if(f.state.assetList.begin(), f.state.assetList.end(), [&](const terra::library::AssetEntry& a) {
            return a.kind == terra::library::AssetKind::Scene && a.path == expected;
        });
        check(registered != f.state.assetList.end() && registered->source == terra::library::AssetSource::Imported &&
                  registered->title == wizard.result->directory.substr(wizard.result->directory.find_last_of('/') + 1),
              "导入的场景已登记到资源库（来源为导入，标题为实例名）");
    }
    if (f.simulated) {
        const auto& d = f.simulated->instance.world->description();
        std::printf("INFO 场景：%zu 个实体，%zu 个执行器，雷达 %zu、相机 %zu\n", d.entities.size(), d.actuators.size(), f.simulated->instance.lidars.size(),
                    f.simulated->instance.cameras.size());
        check(d.entities.size() > 5 && d.actuators.size() == 3 && f.simulated->instance.lidars.size() == 1 && f.simulated->instance.cameras.size() == 1,
              "场景中有小车的连杆、3 个执行器、雷达与相机");
    }
    check(f.ctx.errorMessageCount() == 0, "整个运行期间没有 GL 错误");
    std::printf(failures == 0 ? "导入向导自检通过\n" : "导入向导自检失败：%d 项\n", failures);
    return failures == 0 ? 0 : 1;
}

// ---- 资源库面板自检（M4-08c）----
// 另扫描测试夹具（tests/scene、tests/motion），因为它们不需要下载参考机器人就能打开

std::string fixturePath(const char* rel) {
    return (std::filesystem::path{TERRA_ASSETS_DIR} / "tests" / rel).lexically_normal().string();
}

void librarySelfTestScript(Frame& f, int frame) {
    namespace lib = terra::library;
    auto& p = f.state.libraryPanel;
    auto& t = f.state.libraryTest;
    const std::string arm = fixturePath("scene/keyframe_arm.terra-scene.json");
    const std::string carts = fixturePath("scene/two_carts.terra-scene.json");
    const std::string drop = fixturePath("scene/drop.terra-scene.json");
    const std::string motion = fixturePath("motion/arm_swing.terra-motion.json");
    switch (frame) {
        case 3:  // 欢迎页与面板中的卡片；单击欢迎页中的卡片
            t.welcomeCards = p.welcomeCards;
            t.panelCards = p.panelCards;
            for (const auto& a : f.state.assetList) t.builtinScenes += a.kind == lib::AssetKind::Scene && a.source == lib::AssetSource::Builtin ? 1u : 0u;
            p.selfTestClick = arm;
            break;
        case 6: t.cardOpened = f.simulated && f.simulated->path == arm; break;
        case 8: t.highlighted = p.currentHighlighted; break;
        case 9: {  // 收藏：建立资源库文件并写入，排到最前
            handleLibraryAction(f, vw::LibraryAction{vw::LibraryAction::Type::ToggleFavorite, lib::AssetKind::Scene, carts, {}, true});
            std::error_code ec;
            const auto saved = f.state.library ? f.state.library->asset(lib::AssetKind::Scene, carts) : std::nullopt;
            const auto shown = vw::visibleAssets(libraryContext(f), lib::AssetKind::Scene, vw::LibraryFilter::All, {}, false);
            t.favoriteSaved = std::filesystem::exists(f.state.activeLibraryFile, ec) && saved && saved->favorite && !shown.empty() &&
                              shown.front()->path == carts;
            break;
        }
        case 11:
            openQuickOpen(f);
            std::snprintf(p.quickSearch.data(), p.quickSearch.size(), "%s", "two_carts");
            break;
        case 13:
            t.quickMatches = p.quickMatches;
            p.selfTestQuickEnter = true;
            break;
        case 16: t.quickOpened = f.simulated && f.simulated->path == carts && !p.quickOpen; break;
        case 18:  // 动作分页，单击动作卡片：打开推荐场景并播放
            p.selectTab = vw::LibraryTab::Motions;
            p.selfTestClick = motion;
            break;
        case 24:
            t.motionPlayed = f.simulated && f.simulated->path == arm && f.simulated->motion && f.simulated->motionPath == motion;
            p.selectTab = vw::LibraryTab::Scenes;
            handleLibraryAction(f, vw::LibraryAction{vw::LibraryAction::Type::Hide, lib::AssetKind::Scene, drop, {}, false});
            {
                const auto shown = vw::visibleAssets(libraryContext(f), lib::AssetKind::Scene, vw::LibraryFilter::All, {}, false);
                t.hidden = std::none_of(shown.begin(), shown.end(), [&](const lib::AssetEntry* a) { return a->path == drop; });
            }
            break;
        case 26: f.state.closeRequest = true; break;
        case 30: t.welcomeAgain = !f.simulated && p.welcomeCards > 0; break;
        default: break;
    }
}

int librarySelfTest(Frame& f) {
    int failures = 0;
    const auto check = [&](bool ok, const std::string& what) {
        std::printf("%s %s\n", ok ? "PASS" : "FAILED:", what.c_str());
        if (!ok) ++failures;
    };
    const auto& t = f.state.libraryTest;
    std::printf("INFO 资源库：%zu 个资产，内置场景 %zu 个；欢迎页 %zu 张卡片，面板 %zu 张；快速打开匹配 %zu 个；扫描 %zu 次\n", f.state.assetList.size(),
                t.builtinScenes, t.welcomeCards, t.panelCards, t.quickMatches, f.state.assetScans);
    check(t.builtinScenes >= 8, "内置场景已登记（assets/scenes/robots 中的 8 个，及导入生成的）");
    check(t.welcomeCards >= t.builtinScenes && t.panelCards == t.welcomeCards, "欢迎页与资源库面板列出全部场景卡片");
    check(t.cardOpened, "单击欢迎页中的卡片：立即打开场景");
    check(t.highlighted, "当前场景的卡片高亮");
    check(t.favoriteSaved, "收藏：建立资源库文件并写入，收藏的排在最前");
    check(t.quickMatches >= 1, "快速打开：按名字筛选");
    check(t.quickOpened, "快速打开：回车打开选中的场景，窗口关闭");
    check(t.motionPlayed, "单击动作卡片：打开推荐场景并播放");
    check(t.hidden, "隐藏的卡片不再显示");
    check(t.welcomeAgain, "关闭场景后欢迎页再次出现");
    check(f.ctx.errorMessageCount() == 0, "整个运行期间没有 GL 错误");
    std::printf(failures == 0 ? "资源库面板自检通过\n" : "资源库面板自检失败：%d 项\n", failures);
    return failures == 0 ? 0 : 1;
}

// M4-07：传感器记录自检。场景需含 IMU、雷达与相机（sensor_suite）
int sensorRecordSelfTest(Frame& f) {
    int failures = 0;
    const auto check = [&](bool ok, const std::string& what) {
        std::printf("%s %s\n", ok ? "PASS" : "FAILED:", what.c_str());
        if (!ok) ++failures;
    };
    const std::string path = f.state.lastRecordPath;
    std::printf("INFO 记录文件：%s\n", path.c_str());
    const auto read = terra::simulation::readRecord(path);
    check(read.record.has_value() && read.errors.empty() && read.warnings.empty(), "记录可以读回");
    if (!read.record || !f.simulated) return 1;
    const auto& rec = *read.record;
    const auto& h = rec.header;
    SimulatedScene& sim = *f.simulated;
    const auto& d = sim.instance.world->description();
    check(h.imus.size() == d.sensors.size() && h.lidars.size() == sim.instance.lidars.size() && h.cameras.size() == sim.instance.cameras.size(),
          "记录头中的传感器清单：IMU " + std::to_string(h.imus.size()) + "、雷达 " + std::to_string(h.lidars.size()) + "、相机 " +
              std::to_string(h.cameras.size()));
    bool imuInStates = !rec.states.empty();
    for (const auto& s : rec.states) imuInStates = imuInStates && s.sensors.size() == d.sensors.size() && !s.sensors.empty() && s.sensors[0].values.size() == 10;
    check(imuInStates, "每份状态都记了全部 IMU 的读数（" + std::to_string(rec.states.size()) + " 份）");
    // 雷达与相机按各自的频率、在记录区间内的每个采样步上都有一行
    const std::uint64_t first = rec.states.front().stepIndex;
    const std::uint64_t last = rec.summary ? rec.summary->steps : rec.states.back().stepIndex;
    const double dt = h.timestep;
    std::size_t lidarDue = 0, cameraDue = 0;
    for (std::uint64_t k = first; k <= last; ++k) {
        lidarDue += terra::simulation::sensorSampleDue(k, dt, sim.instance.lidars[0].rateHz) ? 1u : 0u;
        cameraDue += terra::simulation::sensorSampleDue(k, dt, sim.instance.cameras[0].rateHz) ? 1u : 0u;
    }
    std::size_t images = 0;
    bool filesExist = true;
    const std::filesystem::path base = std::filesystem::path{path}.parent_path();
    for (const auto& c : rec.cameraFrames) {
        if (c.rgb.empty()) continue;
        ++images;
        filesExist = filesExist && std::filesystem::exists(base / c.rgb) && std::filesystem::exists(base / c.depth);
    }
    std::printf("INFO 记录：第 %llu～%llu 步，%zu 份状态，雷达 %zu 次（应 %zu），相机 %zu 帧（应 %zu），其中 %zu 帧保存了图像\n",
                static_cast<unsigned long long>(first), static_cast<unsigned long long>(last), rec.states.size(), rec.lidars.size(), lidarDue,
                rec.cameraFrames.size(), cameraDue, images);
    check(rec.lidars.size() == lidarDue && lidarDue >= 2, "雷达按 10 Hz 记录，每个采样步一行（" + std::to_string(rec.lidars.size()) + " 行）");
    check(rec.cameraFrames.size() == cameraDue && cameraDue >= 5, "相机按 30 Hz 记录元数据，每个采样步一行（" + std::to_string(rec.cameraFrames.size()) + " 行）");
    check(images >= 3 && images <= rec.cameraFrames.size() && filesExist, "实际渲染的相机帧保存了彩色与深度图像，文件都在（" + std::to_string(images) + " 帧）");
    const auto rerun = terra::simulation::rerunRecord(rec, terra::physics::mujoco::composeWorld);
    std::printf("INFO 复现：%zu 个状态采样点，%zu 个传感器采样，位姿逐位相同 %s，传感器逐位相同 %s %s\n", rerun.compared, rerun.sensorSamples,
                rerun.bitwise ? "是" : "否", rerun.sensorsBitwise ? "是" : "否", rerun.firstSensorDifference.c_str());
    check(rerun.ran && rerun.bitwise && rerun.sensorsBitwise && rerun.sensorSamples == rec.states.size() + rec.lidars.size() + rec.cameraFrames.size(),
          "复现：IMU、雷达与相机位姿逐位相同");
    // 回放：传感器面板显示记录中的读数
    check(sim.replay.has_value() && sceneRecord.replayCameraTime >= 0.0, "回放已开始，并跳到一帧保存了图像的时刻");
    if (sim.replay) {
        const double t = sim.replay->time;
        const terra::data::PhysicsSnapshot* lastState = nullptr;
        for (const auto& s : rec.states) {
            if (s.simulationTime <= t) lastState = &s;
        }
        const auto& history = sim.imuHistory.at(0);
        check(lastState != nullptr && !history.empty() && history.back().time == lastState->simulationTime &&
                  history.back().values[2] == static_cast<float>(lastState->sensors[0].values[2]),
              "回放时 IMU 曲线取自记录（最后一点为不晚于回放时间的状态中的读数）");
        const terra::simulation::RecordLidar* lastLidar = nullptr;
        for (const auto& l : rec.lidars) {
            if (l.time <= t) lastLidar = &l;
        }
        const auto& scan = sim.lidarScans.at(0);
        check(lastLidar != nullptr && scan && scan->step == lastLidar->step && scan->ranges.size() == lastLidar->ranges.size() &&
                  std::equal(scan->ranges.begin(), scan->ranges.end(), lastLidar->ranges.begin(),
                             [](float a, float b) { return a == b || (std::isnan(a) && std::isnan(b)); }),
              "回放时雷达俯视图取自记录（第 " + std::to_string(lastLidar ? lastLidar->step : 0) + " 步）");
        const auto& preview = sim.cameraPreviews.at(0);
        const auto& camera = sim.instance.cameras[0];
        float center = std::numeric_limits<float>::quiet_NaN();
        if (preview.frame && !preview.frame->depth.empty()) {
            center = preview.frame->depth[static_cast<std::size_t>(camera.height / 2) * static_cast<std::size_t>(camera.width) +
                                          static_cast<std::size_t>(camera.width / 2)];
        }
        std::printf("INFO 回放：相机第 %llu 步，中心深度 %.4f m\n", static_cast<unsigned long long>(preview.frame ? preview.frame->step : 0),
                    static_cast<double>(center));
        check(preview.frame && preview.frame->step == sceneRecord.replayCameraStep && preview.color.name() != 0 && preview.depth.name() != 0,
              "回放时相机预览读入了记录中保存的彩色与深度图像");
        check(std::fabs(center - 2.0f) < 0.01f, "回放的深度图读回正确（中心深度 2 m）");
    }
    check(f.ctx.errorMessageCount() == 0, "整个运行期间没有 GL 错误");
    std::printf(failures == 0 ? "传感器记录自检通过\n" : "传感器记录自检失败：%d 项\n", failures);
    return failures == 0 ? 0 : 1;
}

int sensorSelfTest(Frame& f) {
    int failures = 0;
    const auto check = [&](bool ok, const std::string& what) {
        std::printf("%s %s\n", ok ? "PASS" : "FAILED:", what.c_str());
        if (!ok) ++failures;
    };
    SimulatedScene& sim = *f.simulated;
    const auto& d = sim.instance.world->description();
    const std::uint64_t steps = sim.instance.world->stepIndex();
    const double dt = d.timestep;
    std::printf("INFO 仿真推进了 %llu 步\n", static_cast<unsigned long long>(steps));
    check(steps > 10, "仿真推进了（" + std::to_string(steps) + " 步）");
    const auto due = [&](double rate) {
        std::uint64_t n = 0;
        for (std::uint64_t s = 0; s <= steps; ++s) n += terra::simulation::sensorSampleDue(s, dt, rate) ? 1u : 0u;
        return n;
    };
    // IMU：物理频率，每一步（含初始状态）采样一次；水平静止的 level_imu 为 (0, 0, 9.81)
    const auto* level = d.findSensor("imu/level_imu");
    check(level != nullptr, "找到 IMU imu/level_imu");
    if (level != nullptr) {
        const std::size_t k = level->id.value - 1;
        const auto& history = sim.imuHistory[k];
        check(sim.imuSamples[k] == steps + 1, "IMU 每步采样：" + std::to_string(sim.imuSamples[k]) + " 次 = 步数 + 1");
        check(!history.empty() && std::fabs(history.back().values[2] - 9.81f) < 1e-4f && history.back().values[0] == 0.0f,
              "IMU 曲线的最新一点为 (0, 0, 9.81)");
    }
    // 雷达：10 Hz；读数与 M4-03b 的房间一致（275 束命中）
    check(sim.instance.lidars.size() == 1 && sim.lidarSamples.size() == 1, "一个 2D 激光雷达");
    if (sim.instance.lidars.size() == 1 && sim.lidarSamples.size() == 1) {
        check(sim.lidarSamples[0] == due(10.0), "雷达按 10 Hz 采样：" + std::to_string(sim.lidarSamples[0]) + " 次，与采样规则一致");
        const auto& scan = sim.lidarScans[0];
        std::size_t hits = 0;
        if (scan) {
            for (const float r : scan->ranges) hits += std::isfinite(r) ? 1u : 0u;
        }
        check(scan && terra::simulation::sensorSampleDue(scan->step, dt, 10.0) && hits == 275,
              "最新一次扫描在采样步上，275 束命中（与 M4-03b 的解析解一致，实际 " + std::to_string(hits) + "）");
    }
    // 相机：30 Hz；最新一帧在采样步上；画面中心是 2 m 外的墙
    check(sim.cameraPreviews.size() == 1, "一个相机");
    if (sim.cameraPreviews.size() == 1) {
        const auto& preview = sim.cameraPreviews[0];
        check(preview.rendered >= 1 && preview.frame.has_value(), "相机画面已渲染（" + std::to_string(preview.rendered) + " 次）");
        if (preview.frame) {
            const auto& frame = *preview.frame;
            const auto& cam = sim.instance.cameras[0];
            const float center = frame.depth.empty() ? 0.0f
                                                     : frame.depth[static_cast<std::size_t>(cam.height / 2) * static_cast<std::size_t>(cam.width) +
                                                                   static_cast<std::size_t>(cam.width / 2)];
            check(terra::simulation::sensorSampleDue(frame.step, dt, 30.0) && frame.step <= steps, "相机画面取自 30 Hz 的采样步");
            check(std::fabs(center - 2.0f) < 1e-5f, "画面中心的深度为 2 m（墙），实际 " + std::to_string(center));
            check(preview.color.name() != 0 && preview.depth.name() != 0, "彩色与深度的预览纹理已创建");
        }
    }
    const ImGuiWindow* sensors = ImGui::FindWindowByName("###Sensors");
    const ImGuiWindow* actuators = ImGui::FindWindowByName("###Actuators");
    check(sensors != nullptr && actuators != nullptr && sensors->DockNode != nullptr && sensors->DockNode == actuators->DockNode,
          "“传感器”面板与“执行器”面板停靠在同一组");
    check(f.ctx.errorMessageCount() == 0, "整个运行期间没有 GL 错误");
    return failures == 0 ? 0 : 1;
}

// M5-11：3D 雷达自检（夹具 lidar3d_room）：各雷达按 10 Hz 采样；最新一帧 540 点；传感器面板与视口叠加画出点云
int lidar3dSelfTest(Frame& f) {
    int failures = 0;
    const auto check = [&](bool ok, const std::string& what) {
        std::printf("%s %s\n", ok ? "PASS" : "FAILED:", what.c_str());
        if (!ok) ++failures;
    };
    SimulatedScene& sim = *f.simulated;
    const double dt = sim.instance.world->description().timestep;
    const std::uint64_t steps = sim.instance.world->stepIndex();
    std::uint64_t due = 0;
    for (std::uint64_t s = 0; s <= steps; ++s) due += terra::simulation::sensorSampleDue(s, dt, 10.0) ? 1u : 0u;
    std::printf("INFO 3D 雷达自检：%llu 步；视口中画出 %zu 个点\n", static_cast<unsigned long long>(steps), f.state.overlayPointsDrawn);
    check(steps > 50, "仿真推进了（" + std::to_string(steps) + " 步）");
    check(!sim.lidar3dSamples.empty(), "场景中有 3D 雷达");
    for (std::size_t k = 0; k < sim.lidar3dSamples.size(); ++k) {
        check(sim.lidar3dSamples[k] == due, sim.instance.lidars3d[k].name + " 按 10 Hz 采样：" + std::to_string(sim.lidar3dSamples[k]) + " 次");
    }
    const auto& first = sim.lidar3dScans.empty() ? std::nullopt : sim.lidar3dScans[0];
    check(first && first->points.size() == 540 && terra::simulation::sensorSampleDue(first->step, dt, 10.0), "最新一帧在采样步上，540 个点");
    check(f.state.overlayPointsDrawn > 0, "视口中叠加了点云");
    check(ImGui::FindWindowByName("###Sensors") != nullptr, "传感器面板");
    check(f.ctx.errorMessageCount() == 0, "整个运行期间没有 GL 错误");
    std::printf(failures == 0 ? "3D 雷达自检通过\n" : "3D 雷达自检失败：%d 项\n", failures);
    return failures == 0 ? 0 : 1;
}

// M5-06：热像仪自检（夹具 thermal_bench）：三台热像仪都出了温度图；无噪声相机的图像中心在 80 °C 的板上；
// 背向墙的相机全是天空温度；预览纹理已创建；没有 GL 错误
int thermalSelfTest(Frame& f) {
    int failures = 0;
    const auto check = [&](bool ok, const std::string& what) {
        std::printf("%s %s\n", ok ? "PASS" : "FAILED:", what.c_str());
        if (!ok) ++failures;
    };
    SimulatedScene& sim = *f.simulated;
    const std::uint64_t steps = sim.instance.world->stepIndex();
    std::printf("INFO 热像仪自检：%llu 步；创建了 %llu 次热像预览\n", static_cast<unsigned long long>(steps),
                static_cast<unsigned long long>(f.state.thermalFramesShown));
    check(steps > 50, "仿真推进了（" + std::to_string(steps) + " 步）");
    std::size_t thermals = 0;
    for (std::size_t k = 0; k < sim.instance.cameras.size(); ++k) {
        const auto& camera = sim.instance.cameras[k];
        if (!camera.thermal.enabled) continue;
        ++thermals;
        const auto& preview = sim.cameraPreviews[k];
        const bool ok = preview.frame && preview.frame->thermal.size() == static_cast<std::size_t>(camera.width) * static_cast<std::size_t>(camera.height);
        check(ok && preview.thermal.name() != 0, camera.name + "：温度图 " + std::to_string(camera.width) + " × " + std::to_string(camera.height) + "，预览纹理已创建");
        if (!ok) continue;
        const auto& t = preview.frame->thermal;
        const std::size_t center = static_cast<std::size_t>(camera.height / 2) * static_cast<std::size_t>(camera.width) + static_cast<std::size_t>(camera.width / 2);
        if (camera.name == "bench/ir") {
            check(std::fabs(static_cast<double>(t[center]) - 80.0) < 1e-3, "bench/ir：图像中心在板上，80 °C（" + std::to_string(t[center]) + "）");
        }
        if (camera.name == "bench/ir_sky") {
            const bool sky = std::all_of(t.begin(), t.end(), [](float v) { return v == -20.0f; });
            check(sky, "bench/ir_sky：背向墙没有命中，全是天空温度 −20 °C");
        }
    }
    check(thermals == 3, "场景中有 3 台热像仪");
    check(f.state.thermalFramesShown >= 3, "热像预览更新了多次");
    check(ImGui::FindWindowByName("###Sensors") != nullptr, "传感器面板");
    check(f.ctx.errorMessageCount() == 0, "整个运行期间没有 GL 错误");
    std::printf(failures == 0 ? "热像仪自检通过\n" : "热像仪自检失败：%d 项\n", failures);
    return failures == 0 ? 0 : 1;
}

// M5-02a：元素自检（夹具 elements_bench）：木箱在下落；资源库有内置的元素卡片；单击卡片打开的预览场景只有地面与这个元素
int elementsSelfTest(Frame& f) {
    int failures = 0;
    const auto check = [&](bool ok, const std::string& what) {
        std::printf("%s %s\n", ok ? "PASS" : "FAILED:", what.c_str());
        if (!ok) ++failures;
    };
    std::size_t cards = 0;
    for (const auto& a : f.state.assetList) cards += a.kind == terra::library::AssetKind::Element && !a.hidden ? 1u : 0u;
    std::printf("INFO 元素自检：切换时木箱高 %.3f m；资源库中 %zu 张元素卡片；当前场景 %s\n", sceneRecord.crateHeightAtSwitch, cards,
                f.simulated ? f.simulated->path.c_str() : "（无）");
    check(sceneRecord.firstScene.find("elements_bench.terra-scene.json") != std::string::npos, "先打开元素夹具");
    // 落地后静止时接触有零点几毫米的穿透，高度可以略低于 0
    check(sceneRecord.crateHeightAtSwitch > -0.01 && sceneRecord.crateHeightAtSwitch < 0.45,
          "播放后木箱从 0.5 m 落下（切换时 " + std::to_string(sceneRecord.crateHeightAtSwitch) + " m）");
    check(cards >= 4, "资源库中有内置的元素卡片（至少 4 张）");
    check(sceneRecord.previewRequested, "单击“简易配电柜”的卡片");
    const bool preview = f.simulated && f.simulated->path.find("simple_cabinet.preview.terra-scene.json") != std::string::npos;
    check(preview, "打开了预览场景");
    if (preview) {
        const auto& d = f.simulated->instance.world->description();
        check(d.findEntity("preview/body") != nullptr && d.modelInstances.size() == 2, "预览场景只有地面与这个元素（preview/body）");
        check(f.simulated->instance.thermal.hotspots.size() == 1, "元素的热点随元素进入场景");
    }
    check(f.ctx.errorMessageCount() == 0, "整个运行期间没有 GL 错误");
    std::printf(failures == 0 ? "元素自检通过\n" : "元素自检失败：%d 项\n", failures);
    return failures == 0 ? 0 : 1;
}

int sceneSelfTest(Frame& f, const Options& options) {
    if (sceneRecord.elementsTest) return elementsSelfTest(f);
    if (sceneRecord.thermalTest) return thermalSelfTest(f);
    if (sceneRecord.lidar3dTest) return lidar3dSelfTest(f);
    if (sceneRecord.sensorRecordTest) return sensorRecordSelfTest(f);
    if (sceneRecord.sensorTest) return sensorSelfTest(f);
    if (sceneRecord.recordTest) return recordSelfTest(f);
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
                     "[--expect-renderer 子串] [--scene 场景文件] [--motion 动作文件] [--demo-drive] [--library 库文件] "
                     "[--record-self-test] [--sensor-self-test] [--ros2 通信配置]\n");
        return 2;
    }
    if (!options.ros2Profile.empty() && !ros2Available()) {
        std::fprintf(stderr, "%s\n", ui::text(TextId::SimRos2Unavailable, ui::Language::Chinese));
        return 3;
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

    // M4-08a（用户 2026-11-25）：首次启动（没有设置文件）时，窗口取主显示器的 85%；已有设置的用户保持自己的大小
    if (!options.selfTest && (!loaded || loaded->status == st::LoadStatus::Missing)) {
        const auto screens = platform.monitors();
        const auto primary = std::find_if(screens.begin(), screens.end(), [](const terra::platform::MonitorInfo& m) { return m.primary; });
        const auto& screen = primary != screens.end() ? *primary : screens.empty() ? terra::platform::MonitorInfo{} : screens.front();
        if (screen.current.width > 0 && screen.current.height > 0) {
            const auto fit = [](int v) { return std::clamp(v, st::DisplaySettings::kMinWindowSize, st::DisplaySettings::kMaxWindowSize); };
            settings.current.display.windowWidth = fit(static_cast<int>(screen.current.width * 0.85));
            settings.current.display.windowHeight = fit(static_cast<int>(screen.current.height * 0.85));
        }
    }
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
                                              CommandAction{st::Command::SimReset, "sim_reset"},
                                              CommandAction{st::Command::SimRecord, "sim_record"}}) {
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
    const in::ActionHandle simRecordAction = input.findAction(kAppMap, "sim_record");

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
    state.libraryPath = options.libraryPath;  // M3-07c：空表示默认位置
    // M4-08c：自检没有给 --library 时只用内存中的库；欢迎页会盖住视口，只在资源库自检与正常使用时显示
    state.libraryMemoryOnly = options.selfTest && options.libraryPath.empty();
    state.showWelcome = !options.selfTest || options.librarySelfTest;
    state.vizTest.ros2 = options.vizSelfTest && !options.ros2Profile.empty();
    if (options.librarySelfTest) {
        state.extraScanRoots = {{std::string{TERRA_ASSETS_DIR} + "/tests/scene", terra::library::AssetSource::User},
                                {std::string{TERRA_ASSETS_DIR} + "/tests/motion", terra::library::AssetSource::User}};
    }
    if (!options.importPolicy.empty()) vw::requestPolicyImport(state.policy, options.importPolicy);  // M4-10b
    if (!options.policyImportSelfTest.empty()) {  // M4-10b：策略夹具的场景（policy_bench）也列为可选的机器人
        state.extraScanRoots = {{std::string{TERRA_ASSETS_DIR} + "/tests/policy", terra::library::AssetSource::User}};
    }
    state.autoUiScale = autoUiScale;
    state.monitors = monitors;
    state.log.push_back(tr(TextId::LogStarted));
    if (!vw::ros2PanelAvailable()) state.log.push_back(tr(TextId::Ros2Unavailable));  // M4-08d：提醒安装 ROS2
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
    state.ros2Profile = options.ros2Profile;  // M4-06a：打开场景（或动作的推荐场景）时开启桥接
    if (!options.scenePath.empty()) {
        if (!openScene(frame, options.scenePath)) {
            for (const auto& line : state.sceneErrors) std::fprintf(stderr, "%s\n", line.c_str());
            std::fprintf(stderr, "错误：场景文件不能使用：%s\n", options.scenePath.c_str());
            return 4;
        }
        simulated->demoDrive = options.demoDrive;
        if (options.demoDrive) simulated->runner->resume();
        if (!options.ros2Profile.empty()) {
            // 命令行开启桥接时直接运行（与 Headless 相同）；/terra/pause 可以暂停
            if (!ros2Running(frame)) {
                std::fprintf(stderr, "错误：ROS2 桥接无法开启（见上面的 INFO）：%s\n", options.ros2Profile.c_str());
                return 3;
            }
            simulated->runner->resume();
        }
        // M4-10a：命令行给出策略时打开后直接运行（需要的组件不在时弹出下载窗口）
        if (!options.policyPath.empty()) {
            vw::requestPolicy(state.policy, options.policyPath);
            state.policy.dialogOpen = false;
            state.policy.startRequested = true;
            if (options.policyCommand) state.policy.command = *options.policyCommand;
            simulated->runner->resume();
        }
    } else if (options.motionPath.empty()) {
        ensureDemoAssets(frame);
    }
    // 动作文件（M3-04c）：自检时在第 3 帧打开（代替按播放），否则启动时打开并开始播放
    if (options.selfTest && simulated) {
        sceneRecord.motionPath = options.motionPath;
        sceneRecord.recordTest = options.recordSelfTest;
        sceneRecord.sensorTest = options.sensorSelfTest;
        sceneRecord.lidar3dTest = options.lidar3dSelfTest;
        sceneRecord.thermalTest = options.thermalSelfTest;
        sceneRecord.elementsTest = options.elementsSelfTest;
        sceneRecord.sensorRecordTest = options.sensorRecordSelfTest;
        sceneRecord.vizTest = options.vizSelfTest;
    }
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
        // M4-08a：拖进窗口的文件：场景直接打开，动作打开并播放，其余（URDF、xacro、MJCF）进入导入向导
        for (const auto& dropped : window.takeDroppedPaths()) {
            const auto ends = [&](const std::string& suffix) {
                return dropped.size() >= suffix.size() && dropped.compare(dropped.size() - suffix.size(), suffix.size(), suffix) == 0;
            };
            if (ends(kSceneSuffix)) state.openRequest = dropped;
            else if (ends(".terra-motion.json")) state.motionRequest = dropped;
            else if (ends(terra::library::kPackSuffix)) importPackFile(frame, dropped);  // M4-08c
            else if (ends(terra::viz::kVizSuffix)) importDisplay(frame, dropped);    // M4-08d
            else if (ends(terra::policy::kPolicySuffix)) vw::requestPolicy(state.policy, dropped);  // M4-10a
            else if (ends(".onnx") || ends(".pt") || ends(".pth")) vw::requestPolicyImport(state.policy, dropped);  // M4-10b：导入向导
            else vw::beginImport(state.importWizard, dropped, importDirectory(frame));
        }
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
            if (keyboardFree && input.state(simPlayAction).pressed) {
                if (simulated->replay) simulated->replay->playing = !simulated->replay->playing;
                else simulated->togglePlay();
            }
            if (keyboardFree && input.state(simStepAction).pressed && simulated->runner->paused()) simulated->runner->stepOnce();
            if (keyboardFree && input.state(simResetAction).pressed) simReset(frame);
            if (keyboardFree && input.state(simRecordAction).pressed) toggleRecording(frame);
            if (options.selfTest && !options.policySelfTest) sceneSelfTestScript(frame, frames, options.frames);  // M4-10a：策略自检自己运行
            handleRos2Requests(frame);  // M4-06a
            if (simulated && simulated->replay) {
                // 回放：不推进物理，只推进回放时间
                auto& replay = *simulated->replay;
                if (replay.playing) {
                    replay.time = std::min(replay.duration, replay.time + dt * replay.speed);
                    if (replay.time >= replay.duration) replay.playing = false;
                }
            } else if (simulated) {
                simulated->drive();
                const double before = simulated->instance.world->simulationTime();
                const auto stepped = simulated->runner->update(dt);
                const double rate = dt > 0 ? (simulated->instance.world->simulationTime() - before) / dt : 0.0;
                simulated->realtimeFactor = simulated->runner->paused() ? 0.0 : simulated->realtimeFactor * 0.9 + rate * 0.1;
                if (stepped.error) {
                    state.log.push_back(std::string{tr(TextId::StatusFaulted)} + "：" + stepped.error->message);
                    if (simulated->recorder) {
                        simulated->recorder->event(stepped.error->step, "error", stepped.error->message);
                        stopRecording(frame, "error");
                    }
                }
                for (const auto& e : stepped.controlErrors) state.log.push_back(e.message);
            }
        }
        updateCameraPreviews(frame);  // M4-04：渲染本帧该采样的相机画面

        if (options.selfTest && options.librarySelfTest) librarySelfTestScript(frame, frames);  // M4-08c
        if (options.selfTest && options.vizSelfTest) vizSelfTestScript(frame, frames);          // M4-08d
        if (options.selfTest && options.policySelfTest) policySelfTestScript(frame, frames, options.frames);  // M4-10a
        if (options.selfTest && !options.policyImportSelfTest.empty()) policyImportSelfTestScript(frame, frames, options.policyImportSelfTest);  // M4-10b
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
            // M4-08a：导入向导自检
            if (!options.importSelfTestPath.empty() && frames == 12) {
                vw::beginImport(state.importWizard, options.importSelfTestPath, importDirectory(frame));
                state.importWizard.options.overwrite = true;  // 重复运行自检时覆盖上一次的导入目录
            }
            if (!options.importSelfTestPath.empty() && frames == 14) {
                if (auto imported = vw::runImport(state.importWizard, terra::physics::mujoco::composeWorld)) {
                    state.openRequest = *imported;
                    writableLibrary(frame);  // M4-08c：与向导的“导入并打开”相同，导入后登记
                    state.assetsStale = true;
                }
            }
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
            const bool opened = openScene(frame, path);
            // M4-10b：打开后运行等待中的策略
            if (state.pendingPolicy) {
                const std::string policyFile = *state.pendingPolicy;
                state.pendingPolicy.reset();
                if (opened) runPolicyFile(frame, policyFile, path);
            }
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
        if (state.replayRequest) {
            const std::string path = *state.replayRequest;
            state.replayRequest.reset();
            startReplay(frame, path);
        }

        ++frames;
        // M4-08d：显示配置自检按墙钟等待话题与数据，没完成时最多延长到 4 倍帧数
        const bool vizWaiting = options.vizSelfTest && state.vizTest.phase < 6 && frames < options.frames * 4;
        const bool lastFrame = (options.frames > 0 && frames >= options.frames && !vizWaiting) ||
                               (options.seconds > 0.0 && now - start >= options.seconds);
        if (lastFrame && options.selfTest) {
            result = !options.policyImportSelfTest.empty() ? policyImportSelfTest(frame)
                     : options.policySelfTest              ? policySelfTest(frame)
                     : options.vizSelfTest                 ? vizSelfTest(frame)
                     : options.librarySelfTest             ? librarySelfTest(frame)
                     : !options.importSelfTestPath.empty() ? importSelfTest(frame)
                     : simulated                        ? sceneSelfTest(frame, options)
                                                        : selfTest(frame, uiContext, options);
        }
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
    stopRos2(frame);  // M4-06a：打印桥接的统计
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
