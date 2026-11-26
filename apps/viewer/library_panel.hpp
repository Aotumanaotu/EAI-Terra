#pragma once

// 资源库面板、欢迎页与快速打开（M4-08c；用户 2026-11-25 评审：维护好的场景就是按钮，点一下就加载，快捷键只是辅助）。
//
// - 资源库面板（停靠在左侧，大纲之上）：搜索框、网格 / 列表切换；分页为场景、动作、机器人；筛选为全部、收藏、最近、内置、导入。
//   每个条目是一张卡片：矢量图标、短标题、一行说明、标签（来源、传感器、动作数）；右上角星标收藏；当前场景描边并标“当前”。
//   单击卡片：场景立即打开；动作打开推荐场景并播放；机器人打开用到它的第一个场景（右键菜单列出全部）。
// - 欢迎页：没有打开场景时在视口中央显示大号的场景卡片（收藏与最近打开的在前）。
// - 快速打开（Ctrl+P）：输入名字的一部分，方向键选择，回车打开（场景与动作）。
// 这里只绘制并返回用户的操作（LibraryAction），打开场景、写资源库等由查看器在帧末处理。

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "imgui.h"
#include "terra/library/library.hpp"

namespace terra::viewer {

enum class LibraryTab { Scenes, Motions, Robots, Policies };  // Policies：M4-10b
enum class LibraryFilter { All, Favorites, Recent, Builtin, Imported };

struct LibraryAction {
    enum class Type { Open, ToggleFavorite, Hide, UnhideAll, CopyPath, Reveal, Reimport, ExportPack, Rescan, OpenDisplay };  // OpenDisplay：M4-08d
    Type type = Type::Open;
    terra::library::AssetKind kind = terra::library::AssetKind::Scene;
    std::string path;             // Open：要打开的场景或动作（机器人为选中的场景）；其他：条目的路径
    std::string importDirectory;  // Reimport、ExportPack
    bool favorite = false;        // ToggleFavorite：新的值
};

struct LibraryContext {
    const std::vector<terra::library::AssetEntry>* assets = nullptr;  // 全部类型（不含缺失的）
    std::string currentScene;                                         // 当前打开的场景（绝对路径；没有时为空）
    std::vector<std::string> recent;                                  // 最近打开的场景，最新的在前
};

struct LibraryPanelState {
    std::array<char, 128> search{};
    LibraryTab tab = LibraryTab::Scenes;
    std::optional<LibraryTab> selectTab;  // 下一帧切到该分页（自检、菜单）
    LibraryFilter filter = LibraryFilter::All;
    bool grid = true;
    // 快速打开（Ctrl+P）
    bool quickOpen = false;
    bool quickFocus = false;
    std::array<char, 128> quickSearch{};
    int quickSelected = 0;
    int quickFrames = 0;  // 打开后经过的帧数（失去焦点即关闭，但第一帧焦点还没有给到）
    // 自检：本帧把该路径的卡片当作被单击（与鼠标单击走同一条路径）；快速打开中按回车
    std::string selfTestClick;
    bool selfTestQuickEnter = false;
    // 本帧绘制的情况（自检）
    std::size_t panelCards = 0;
    std::size_t welcomeCards = 0;
    std::size_t quickMatches = 0;
    bool currentHighlighted = false;
};

// 某一类中要显示的条目：去掉隐藏的，按筛选与搜索；收藏在前，recentFirst 时最近打开的随后，其余保持库中的顺序
std::vector<const terra::library::AssetEntry*> visibleAssets(const LibraryContext& context, terra::library::AssetKind kind,
                                                             LibraryFilter filter, const std::string& query, bool recentFirst);

// 面板内容（在 ImGui::Begin / End 之间调用）
std::optional<LibraryAction> drawLibraryPanel(LibraryPanelState& state, const LibraryContext& context);
// 欢迎页：画在视口图像的 [min, max] 区域中
std::optional<LibraryAction> drawWelcome(LibraryPanelState& state, const LibraryContext& context, ImVec2 min, ImVec2 max);
// 快速打开窗口（state.quickOpen 为真时）
std::optional<LibraryAction> drawQuickOpen(LibraryPanelState& state, const LibraryContext& context);

// 在文件管理器中显示（xdg-open 打开所在目录，不等待）
void revealInFileManager(const std::string& path);

}  // namespace terra::viewer
