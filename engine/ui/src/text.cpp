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
    {TextId::OutlinerGround, "地面网格", "Ground Grid"},
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
     "  按住右键拖动    转视角\n"
     "  W/↑  S/↓        沿视线前进 / 后退\n"
     "  A/←  D/→        左右平移\n"
     "  E/空格  Q/Ctrl  上升 / 下降\n"
     "  滚轮            拉近 / 拉远\n"
     "  按住中键拖动    平移\n"
     "  Shift           加速\n"
     "  F               复位视角\n"
     "  F11             全屏 / 窗口",
     "With the mouse over the viewport:\n"
     "  Hold right button + drag   Look around\n"
     "  W/Up  S/Down               Move forward / back along view\n"
     "  A/Left  D/Right            Strafe left / right\n"
     "  E/Space  Q/Ctrl            Up / down\n"
     "  Mouse wheel                Dolly in / out\n"
     "  Hold middle button + drag  Pan\n"
     "  Shift                      Faster\n"
     "  F                          Reset view\n"
     "  F11                        Fullscreen / windowed"},
    {TextId::AboutTitle, "关于 EAI Terra", "About EAI Terra"},
    {TextId::AboutBody,
     "EAI Terra —— 开源机器人仿真引擎\n物理：MuJoCo 4.0.1\n渲染：OpenGL 4.5（自研 Terra Render）\n界面：Dear ImGui 1.93（docking）\n字体：Noto Sans SC（SIL OFL 1.1）",
     "EAI Terra - An Open Robotics Simulation Engine\nPhysics: MuJoCo 4.0.1\nRendering: OpenGL 4.5 (Terra Render)\nUI: Dear ImGui 1.93 (docking)\nFont: Noto Sans SC (SIL OFL 1.1)"},
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
