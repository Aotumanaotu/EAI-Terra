#include "terra/ui/text.hpp"

#include <array>
#include <atomic>

namespace terra::ui {

namespace {

struct Entry {
    TextId id;
    const char* chinese;
    const char* english;
};

// 顺序必须与 TextId 一致（编译期检查见下方）。
constexpr Entry kEntries[] = {
    {TextId::AppName, "EAI Terra", "EAI Terra"},
    {TextId::MenuFile, "文件", "File"},
    {TextId::MenuFileQuit, "退出", "Quit"},
    {TextId::MenuView, "视图", "View"},
    {TextId::MenuViewFullscreen, "全屏", "Fullscreen"},
    {TextId::MenuViewResetCamera, "复位相机", "Reset Camera"},
    {TextId::MenuWindow, "窗口", "Window"},
    {TextId::MenuSettings, "设置", "Settings"},
    {TextId::MenuSettingsLanguage, "语言", "Language"},
    {TextId::MenuHelp, "帮助", "Help"},
    {TextId::MenuHelpControls, "操作说明", "Controls"},
    {TextId::MenuHelpAbout, "关于", "About"},
    {TextId::PanelViewport, "视口", "Viewport"},
    {TextId::PanelOutliner, "大纲", "Outliner"},
    {TextId::PanelDetails, "属性", "Details"},
    {TextId::PanelLog, "日志", "Log"},
    {TextId::PanelStats, "统计", "Statistics"},
    {TextId::OutlinerScene, "场景", "Scene"},
    {TextId::OutlinerGround, "地面与网格", "Ground and Grid"},
    {TextId::OutlinerAxes, "坐标轴", "Axes"},
    {TextId::OutlinerCube, "立方体", "Cube"},
    {TextId::OutlinerHint, "场景数据将在 M2 接入", "Scene data arrives in M2"},
    {TextId::DetailsCamera, "相机", "Camera"},
    {TextId::DetailsPosition, "位置 (m)", "Position (m)"},
    {TextId::DetailsYaw, "偏航 (°)", "Yaw (°)"},
    {TextId::DetailsPitch, "俯仰 (°)", "Pitch (°)"},
    {TextId::DetailsFov, "视场 (°)", "Field of View (°)"},
    {TextId::DetailsHint, "选中物体的属性将在 M2 接入", "Selection details arrive in M2"},
    {TextId::LogStarted, "EAI Terra 已启动", "EAI Terra started"},
    {TextId::LogLanguageChanged, "界面语言已切换为中文", "Interface language switched to English"},
    {TextId::LogDisplayModeChanged, "显示模式：", "Display mode: "},
    {TextId::StatsFrameRate, "帧率", "Frame rate"},
    {TextId::StatsFrameTime, "帧时间", "Frame time"},
    {TextId::StatsRenderer, "渲染器", "Renderer"},
    {TextId::StatsOpenGl, "OpenGL", "OpenGL"},
    {TextId::StatsDisplayMode, "显示模式", "Display mode"},
    {TextId::StatsViewportSize, "视口尺寸", "Viewport size"},
    {TextId::StatsContentScale, "显示缩放", "Content scale"},
    {TextId::StatusCamera, "相机", "Camera"},
    {TextId::DisplayWindowed, "窗口", "Windowed"},
    {TextId::DisplayBorderless, "无边框全屏", "Borderless fullscreen"},
    {TextId::DisplayExclusive, "独占全屏", "Exclusive fullscreen"},
    {TextId::ControlsTitle, "操作说明", "Controls"},
    {TextId::ControlsBody,
     "鼠标在视口中时：\n"
     "  按住中键拖动    转视角\n"
     "  W/↑  S/↓        沿视线前进 / 后退\n"
     "  A/←  D/→        左右平移\n"
     "  E/空格  Q       上升 / 下降\n"
     "  滚轮            拉近 / 拉远\n"
     "  Shift+中键拖动  平移\n"
     "  Shift           加速\n"
     "  F               复位视角\n"
     "  F11             全屏 / 窗口\n"
     "  Ctrl + / -      界面放大 / 缩小（Ctrl+0 恢复自动）\n"
     "按键可在“设置 → 偏好设置…”中修改（以上为默认按键）",
     "With the mouse over the viewport:\n"
     "  Hold middle button + drag  Look around\n"
     "  W/Up  S/Down               Move forward / back along view\n"
     "  A/Left  D/Right            Strafe left / right\n"
     "  E/Space  Q                 Up / down\n"
     "  Mouse wheel                Dolly in / out\n"
     "  Shift + middle drag        Pan\n"
     "  Shift                      Faster\n"
     "  F                          Reset view\n"
     "  F11                        Fullscreen / windowed\n"
     "  Ctrl + / -                 UI zoom in / out (Ctrl+0: auto)\n"
     "Keys can be changed in Settings > Preferences… (defaults shown above)"},
    {TextId::AboutTitle, "关于 EAI Terra", "About EAI Terra"},
    {TextId::AboutBody,
     "EAI Terra —— 开源机器人仿真引擎\n物理：MuJoCo 4.0.1\n渲染：OpenGL 4.5（自研 Terra Render）\n界面：Dear ImGui 1.93（docking）\n字体：Noto Sans SC（SIL OFL 1.1）",
     "EAI Terra - An Open Robotics Simulation Engine\nPhysics: MuJoCo 4.0.1\nRendering: OpenGL 4.5 (Terra Render)\nUI: Dear ImGui 1.93 (docking)\nFont: Noto Sans SC (SIL OFL 1.1)"},
    {TextId::MenuViewUiScale, "界面缩放", "UI Scale"},
    {TextId::MenuViewZoomIn, "放大", "Zoom In"},
    {TextId::MenuViewZoomOut, "缩小", "Zoom Out"},
    {TextId::MenuViewZoomReset, "恢复自动", "Reset to Auto"},
    {TextId::StatsUiScale, "界面缩放", "UI scale"},
    {TextId::LogUiScale, "界面缩放：", "UI scale: "},
    {TextId::MenuSettingsPreferences, "偏好设置…", "Preferences…"},
    {TextId::MenuWindowResetLayout, "恢复默认布局", "Reset Layout"},
    {TextId::SettingsTitle, "偏好设置", "Preferences"},
    {TextId::SettingsInterface, "界面", "Interface"},
    {TextId::SettingsDisplay, "显示", "Display"},
    {TextId::SettingsCamera, "相机", "Camera"},
    {TextId::SettingsKeys, "按键", "Keys"},
    {TextId::SettingsLanguage, "语言", "Language"},
    {TextId::SettingsUiScale, "界面缩放", "UI scale"},
    {TextId::SettingsUiScaleAuto, "自动（按显示器推算）", "Automatic (from monitor)"},
    {TextId::SettingsDisplayMode, "显示模式", "Display mode"},
    {TextId::SettingsMonitor, "显示器", "Monitor"},
    {TextId::SettingsPrimary, "（主）", " (primary)"},
    {TextId::SettingsVideoMode, "分辨率与刷新率", "Resolution and refresh rate"},
    {TextId::SettingsDesktopMode, "桌面当前模式", "Current desktop mode"},
    {TextId::SettingsVsync, "垂直同步", "Vertical sync"},
    {TextId::SettingsExclusiveHint, "独占全屏会切换显示器的分辨率，失去焦点时自动最小化；按 F11 回到窗口。", "Exclusive fullscreen changes the monitor resolution and minimizes when focus is lost; press F11 to return to a window."},
    {TextId::SettingsLook, "转视角灵敏度（°/像素）", "Look sensitivity (°/px)"},
    {TextId::SettingsSpeed, "移动速度（m/s）", "Move speed (m/s)"},
    {TextId::SettingsBoost, "加速倍率", "Boost multiplier"},
    {TextId::SettingsWheel, "滚轮步长（m/格）", "Wheel step (m/notch)"},
    {TextId::SettingsPan, "平移速度（m/像素）", "Pan speed (m/px)"},
    {TextId::SettingsFov, "视场角（°）", "Field of view (°)"},
    {TextId::SettingsSliderHint, "按住 Ctrl 点击滑块可直接输入数值。", "Ctrl+click a slider to type a value."},
    {TextId::SettingsAction, "操作", "Action"},
    {TextId::SettingsKey, "按键", "Key"},
    {TextId::SettingsAltKey, "备用", "Alternate"},
    {TextId::SettingsNoKey, "（无）", "(none)"},
    {TextId::SettingsKeysHint, "点击后按下新键，右键点击清除。同一个键只能用于一个操作。", "Click, then press a new key; right-click to clear. Each key can be used by only one action."},
    {TextId::SettingsMouseHint, "鼠标（暂不可改）：中键拖动转视角，加速键 + 中键拖动平移，滚轮拉近 / 拉远。", "Mouse (fixed for now): middle-drag to look, boost key + middle-drag to pan, wheel to dolly."},
    {TextId::SettingsPressKey, "请按下新键…", "Press a new key…"},
    {TextId::SettingsPressKeyHint, "Esc 取消；Ctrl 保留给界面快捷键", "Esc to cancel; Ctrl is reserved for UI shortcuts"},
    {TextId::SettingsRestorePage, "恢复本页默认值", "Restore Page Defaults"},
    {TextId::SettingsRestoreAll, "全部恢复默认", "Restore All Defaults"},
    {TextId::SettingsFile, "设置文件：", "Settings file: "},
    {TextId::SettingsNotSaved, "（本次运行不保存）", "(not saved in this run)"},
    {TextId::SettingsAutoSave, "修改立即生效并自动保存。", "Changes take effect immediately and are saved automatically."},
    {TextId::CmdCameraForward, "前进", "Forward"},
    {TextId::CmdCameraBack, "后退", "Backward"},
    {TextId::CmdCameraLeft, "向左", "Left"},
    {TextId::CmdCameraRight, "向右", "Right"},
    {TextId::CmdCameraUp, "上升", "Up"},
    {TextId::CmdCameraDown, "下降", "Down"},
    {TextId::CmdCameraBoost, "加速（也是平移修饰键）", "Boost (also the pan modifier)"},
    {TextId::CmdCameraReset, "复位视角", "Reset view"},
    {TextId::CmdAppFullscreen, "全屏 / 窗口", "Fullscreen / windowed"},
    {TextId::LogSettingsLoaded, "已读取设置：", "Settings loaded: "},
    {TextId::LogSettingsMissing, "未找到设置文件，使用默认设置：", "No settings file yet, using defaults: "},
    {TextId::LogSettingsCorrupt, "设置文件无法解析，已备份为 ", "Settings file could not be parsed; backed up to "},
    {TextId::LogSettingsUnreadable, "无法读取设置文件，使用默认设置：", "Cannot read the settings file, using defaults: "},
    {TextId::LogSettingsWarning, "设置：", "Settings: "},
    {TextId::LogSettingsSaveFailed, "保存设置失败：", "Failed to save settings: "},
    {TextId::LogKeyMoved, " 已从以下操作移走：", " removed from: "},
    {TextId::LogDisplayModeFailed, "切换显示模式失败：", "Display mode change failed: "},
    {TextId::DetailsRender, "渲染", "Rendering"},
    {TextId::RenderSunAzimuth, "太阳方位角", "Sun azimuth"},
    {TextId::RenderSunElevation, "太阳高度角", "Sun elevation"},
    {TextId::RenderSunIntensity, "太阳强度", "Sun intensity"},
    {TextId::RenderAmbient, "环境光强度", "Ambient intensity"},
    {TextId::RenderExposure, "曝光", "Exposure"},
    {TextId::RenderToneMapper, "色调映射", "Tone mapping"},
    {TextId::ToneMapperAgx, "AgX（Blender 风格）", "AgX (Blender style)"},
    {TextId::ToneMapperNeutral, "PBR Neutral（还原基础色）", "PBR Neutral (faithful base colors)"},
    {TextId::ToneMapperAces, "ACES（电影感）", "ACES (filmic)"},
    {TextId::ToneMapperNone, "无（线性截断）", "None (linear clip)"},
    {TextId::RenderMsaa, "抗锯齿（MSAA）", "Anti-aliasing (MSAA)"},
    {TextId::RenderMsaaOff, "关闭", "Off"},
    {TextId::RenderShadows, "阴影", "Shadows"},
    {TextId::RenderGrid, "地面网格", "Ground grid"},
    {TextId::RenderRestore, "恢复渲染默认值", "Restore Rendering Defaults"},
    {TextId::StatsMsaa, "抗锯齿", "Anti-aliasing"},
    {TextId::OutlinerSpheres, "材质球（金属度 / 粗糙度）", "Material spheres (metallic / roughness)"},
    {TextId::OutlinerCylinder, "圆柱", "Cylinder"},
    {TextId::OutlinerCapsule, "胶囊", "Capsule"},
    {TextId::OutlinerSun, "太阳光", "Sun light"},
    {TextId::RenderEnvironment, "环境光", "Environment lighting"},
    {TextId::EnvironmentOutdoor, "室外（HDRI：rural_asphalt_road）", "Outdoor (HDRI: rural_asphalt_road)"},
    {TextId::EnvironmentStudio, "摄影棚（HDRI：studio_small_09）", "Studio (HDRI: studio_small_09)"},
    {TextId::EnvironmentHemisphere, "半球光（不用贴图）", "Hemisphere (no image)"},
    {TextId::RenderEnvironmentIntensity, "环境强度", "Environment intensity"},
    {TextId::RenderEnvironmentRotation, "环境旋转", "Environment rotation"},
    {TextId::RenderBackground, "显示环境背景", "Show environment background"},
    {TextId::RenderBackgroundBlur, "背景模糊", "Background blur"},
    {TextId::LogEnvironmentLoaded, "已加载环境贴图：", "Environment loaded: "},
    {TextId::LogEnvironmentFailed, "环境贴图不可用，改用半球光：", "Environment unavailable, using hemisphere light: "},
    {TextId::RenderSunFromEnvironment, "方向光跟随环境贴图中的太阳", "Sun follows the environment's sun"},
    {TextId::RenderNoSunInEnvironment, "（该环境贴图中没有明显的太阳，使用下面的手动设置）", "(No distinct sun in this environment; manual settings below apply)"},
    {TextId::OutlinerTextures, "张贴图", "textures"},
    {TextId::LogAssetLoaded, "已加载资产：", "Asset loaded: "},
    {TextId::LogAssetWarning, "资产警告：", "Asset warning: "},
    {TextId::LogAssetError, "资产错误：", "Asset error: "},
    {TextId::LogAssetInfo, "资产：", "Asset: "},
    {TextId::RenderAssetArmOcclusion, "资产：用 ARM 贴图的 R 通道作为环境遮蔽", "Assets: use the ARM texture's R channel as occlusion"},
    {TextId::StatsGpuTime, "显卡耗时", "GPU time"},
    {TextId::StatsGpuShadow, "  阴影", "  Shadows"},
    {TextId::StatsGpuOpaque, "  物体", "  Opaque"},
    {TextId::StatsGpuBackground, "  背景", "  Background"},
    {TextId::StatsGpuGrid, "  网格", "  Grid"},
    {TextId::StatsGpuResolve, "  抗锯齿解析", "  MSAA resolve"},
    {TextId::StatsGpuToneMap, "  色调映射", "  Tone mapping"},
    {TextId::StatsCpuCores, "CPU 逻辑核数", "CPU logical cores"},
    {TextId::StatsVramSource, "显存数据来源", "VRAM data source"},
    {TextId::StatsUnavailable, "不可用", "Unavailable"},
    {TextId::StatsCharts, "性能图表", "Performance charts"},
    {TextId::ChartFrameTime, "帧时间", "Frame time"},
    {TextId::ChartFrame, "整帧", "Frame"},
    {TextId::ChartGpu, "显卡", "GPU"},
    {TextId::ChartCpu, "CPU 占用", "CPU usage"},
    {TextId::ChartProcess, "本进程", "This process"},
    {TextId::ChartSystem, "整机", "System"},
    {TextId::ChartVram, "显存", "VRAM"},
    {TextId::ChartVramUsed, "已用", "Used"},
    {TextId::ChartVramUnavailable, "显存：当前驱动不提供查询（如 llvmpipe）", "VRAM: not reported by this driver (e.g. llvmpipe)"},
    {TextId::ChartMemory, "进程内存", "Process memory"},
    {TextId::ChartResident, "常驻", "Resident"},
    {TextId::Close, "关闭", "Close"},
};

constexpr bool entriesInOrder() {
    if (sizeof(kEntries) / sizeof(kEntries[0]) != static_cast<std::size_t>(kTextCount)) return false;
    for (int i = 0; i < kTextCount; ++i) {
        if (static_cast<int>(kEntries[i].id) != i) return false;
    }
    return true;
}
static_assert(entriesInOrder(), "kEntries 必须按 TextId 顺序覆盖全部条目");

std::atomic<Language> g_language{Language::Chinese};

}  // namespace

const char* text(TextId id, Language lang) noexcept {
    const int i = static_cast<int>(id);
    if (i < 0 || i >= kTextCount) return "";
    return lang == Language::Chinese ? kEntries[i].chinese : kEntries[i].english;
}

const char* tr(TextId id) noexcept { return text(id, g_language.load()); }

void setLanguage(Language lang) noexcept { g_language.store(lang); }

Language language() noexcept { return g_language.load(); }

const char* languageName(Language lang) noexcept { return lang == Language::Chinese ? "中文" : "English"; }

}  // namespace terra::ui
