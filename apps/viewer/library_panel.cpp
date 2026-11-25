// 资源库面板、欢迎页与快速打开（M4-08c）。

#include "library_panel.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <filesystem>
#include <map>
#include <utility>

#include <spawn.h>
#include <sys/wait.h>

#include "terra/ui/text.hpp"

extern char** environ;

namespace terra::viewer {

namespace {

namespace lib = terra::library;
namespace fs = std::filesystem;
using terra::ui::TextId;
using terra::ui::tr;

// 与 ImGui 颜色宏相同的打包方式（R 在低位），避免展开含 C 风格转换的宏
constexpr ImU32 rgba(unsigned r, unsigned g, unsigned b, unsigned a = 255) {
    return static_cast<ImU32>((a << 24) | (b << 16) | (g << 8) | r);
}

// 与文件浏览窗口的类型颜色一致
ImU32 kindColor(lib::AssetKind kind) {
    switch (kind) {
        case lib::AssetKind::Scene: return rgba(76, 175, 110);
        case lib::AssetKind::Motion: return rgba(156, 110, 220);
        case lib::AssetKind::Robot: return rgba(70, 140, 230);
        case lib::AssetKind::Environment: return rgba(220, 110, 170);
    }
    return rgba(130, 130, 140);
}

bool hasTag(const lib::AssetEntry& a, const char* tag) { return std::find(a.tags.begin(), a.tags.end(), tag) != a.tags.end(); }

bool unusable(const lib::AssetEntry& a) { return hasTag(a, "models-missing") || hasTag(a, "invalid") || hasTag(a, "scene-missing"); }

// 按宽度截断（UTF-8 按字符），超出时加省略号
std::string ellipsize(const std::string& text, float size, float width) {
    ImFont* font = ImGui::GetFont();
    const auto measure = [&](const std::string& s) { return font->CalcTextSizeA(size, FLT_MAX, 0.0f, s.c_str()).x; };
    if (measure(text) <= width) return text;
    const std::string ellipsis = "…";
    std::string out = text;
    while (!out.empty()) {
        std::size_t n = out.size() - 1;
        while (n > 0 && (static_cast<unsigned char>(out[n]) & 0xC0u) == 0x80u) --n;
        out.resize(n);
        if (measure(out + ellipsis) <= width) return out + ellipsis;
    }
    return ellipsis;
}

// ---- 矢量图标 ----

void drawAssetIcon(ImDrawList* dl, ImVec2 a, float s, lib::AssetKind kind, bool dim) {
    const ImU32 back = dim ? rgba(110, 112, 122) : kindColor(kind);
    dl->AddRectFilled(a, ImVec2(a.x + s, a.y + s), back, s * 0.2f);
    const ImU32 w = rgba(255, 255, 255, 235);
    const float u = s / 16.0f;
    const auto at = [&](float x, float y) { return ImVec2(a.x + x * u, a.y + y * u); };
    switch (kind) {
        case lib::AssetKind::Scene: {  // 地面上的方块
            dl->AddEllipseFilled(at(8.0f, 12.2f), ImVec2(6.0f * u, 1.6f * u), rgba(255, 255, 255, 120));
            const ImVec2 front[] = {at(4.5f, 6.5f), at(9.5f, 6.5f), at(9.5f, 11.5f), at(4.5f, 11.5f)};
            const ImVec2 top[] = {at(4.5f, 6.5f), at(6.8f, 4.2f), at(11.8f, 4.2f), at(9.5f, 6.5f)};
            const ImVec2 side[] = {at(9.5f, 6.5f), at(11.8f, 4.2f), at(11.8f, 9.2f), at(9.5f, 11.5f)};
            dl->AddConvexPolyFilled(front, 4, w);
            dl->AddConvexPolyFilled(top, 4, rgba(255, 255, 255, 200));
            dl->AddConvexPolyFilled(side, 4, rgba(255, 255, 255, 160));
            break;
        }
        case lib::AssetKind::Motion: {  // 播放
            dl->AddTriangleFilled(at(5.5f, 3.8f), at(5.5f, 12.2f), at(12.6f, 8.0f), w);
            break;
        }
        case lib::AssetKind::Robot: {  // 机器人头部
            dl->AddLine(at(8.0f, 5.0f), at(8.0f, 3.0f), w, 1.0f * u);
            dl->AddCircleFilled(at(8.0f, 2.6f), 1.1f * u, w);
            dl->AddRectFilled(at(3.2f, 5.0f), at(12.8f, 12.8f), w, 2.0f * u);
            dl->AddCircleFilled(at(6.2f, 8.6f), 1.2f * u, back);
            dl->AddCircleFilled(at(9.8f, 8.6f), 1.2f * u, back);
            dl->AddRectFilled(at(6.0f, 10.8f), at(10.0f, 11.6f), back, 0.4f * u);
            break;
        }
        case lib::AssetKind::Environment: {  // 太阳与地平线
            dl->AddCircleFilled(at(8.0f, 8.0f), 3.0f * u, w);
            dl->AddRectFilled(at(2.0f, 10.5f), at(14.0f, 12.0f), w, 0.5f * u);
            break;
        }
    }
    if (dim) {  // 不可用：右下角感叹号
        dl->AddCircleFilled(at(13.0f, 13.0f), 3.0f * u, rgba(230, 160, 40));
        dl->AddRectFilled(at(12.6f, 11.2f), at(13.4f, 13.6f), rgba(30, 30, 30));
        dl->AddRectFilled(at(12.6f, 14.1f), at(13.4f, 14.9f), rgba(30, 30, 30));
    }
}

void drawStar(ImDrawList* dl, ImVec2 c, float r, bool filled, ImU32 color) {
    ImVec2 points[10];
    for (int k = 0; k < 10; ++k) {
        const float radius = (k % 2 == 0) ? r : r * 0.45f;
        const float angle = -1.5707963f + static_cast<float>(k) * 0.6283185f;
        points[k] = ImVec2(c.x + std::cos(angle) * radius, c.y + std::sin(angle) * radius);
    }
    if (filled) dl->AddConcavePolyFilled(points, 10, color);
    else dl->AddPolyline(points, 10, color, std::max(1.0f, r * 0.14f), ImDrawFlags_Closed);
}

// 圆角小标签，返回宽度
float chip(ImDrawList* dl, ImVec2 pos, const std::string& text, ImU32 back, ImU32 fore, float size) {
    ImFont* font = ImGui::GetFont();
    const ImVec2 t = font->CalcTextSizeA(size, FLT_MAX, 0.0f, text.c_str());
    const float pad = size * 0.4f;
    dl->AddRectFilled(pos, ImVec2(pos.x + t.x + 2 * pad, pos.y + size * 1.35f), back, size * 0.4f);
    dl->AddText(font, size, ImVec2(pos.x + pad, pos.y + size * 0.175f), fore, text.c_str());
    return t.x + 2 * pad;
}

void gridGlyph(ImDrawList* dl, ImVec2 a, float s, ImU32 c) {
    const float g = s * 0.12f, w = (s - g) * 0.5f;
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            const ImVec2 p(a.x + static_cast<float>(i) * (w + g), a.y + static_cast<float>(j) * (w + g));
            dl->AddRectFilled(p, ImVec2(p.x + w, p.y + w), c, s * 0.08f);
        }
    }
}

void listGlyph(ImDrawList* dl, ImVec2 a, float s, ImU32 c) {
    for (int k = 0; k < 3; ++k) {
        const float y = a.y + s * (0.15f + 0.35f * static_cast<float>(k));
        dl->AddRectFilled(ImVec2(a.x, y - s * 0.08f), ImVec2(a.x + s * 0.18f, y + s * 0.08f), c, s * 0.04f);
        dl->AddRectFilled(ImVec2(a.x + s * 0.28f, y - s * 0.07f), ImVec2(a.x + s, y + s * 0.07f), c, s * 0.04f);
    }
}

bool glyphButton(const char* id, bool active, void (*glyph)(ImDrawList*, ImVec2, float, ImU32), const char* tooltip) {
    const float h = ImGui::GetFrameHeight();
    if (active) ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
    const bool pressed = ImGui::Button(id, ImVec2(h, h));
    if (active) ImGui::PopStyleColor();
    const ImVec2 a = ImGui::GetItemRectMin();
    const float pad = h * 0.24f;
    glyph(ImGui::GetWindowDrawList(), ImVec2(a.x + pad, a.y + pad), h - 2 * pad, ImGui::GetColorU32(ImGuiCol_Text));
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tooltip);
    return pressed;
}

// ---- 卡片 ----

struct CardInfo {
    std::string subtitle;
    std::vector<std::pair<std::string, ImU32>> chips;
    std::string hint;  // 悬停提示的最后一行
};

const ImU32 kChipText = rgba(240, 242, 246);

CardInfo describe(const lib::AssetEntry& a, const std::map<std::string, int>& motionCounts) {
    CardInfo info;
    info.subtitle = a.description == a.title ? std::string{} : a.description;
    if (a.kind == lib::AssetKind::Motion && !a.related.empty()) info.subtitle = fs::path{a.related[0]}.filename().string();
    info.chips.emplace_back(tr(a.source == lib::AssetSource::Builtin ? TextId::LibrarySourceBuiltin
                               : a.source == lib::AssetSource::Imported ? TextId::LibrarySourceImported
                                                                        : TextId::LibrarySourceUser),
                            a.source == lib::AssetSource::Builtin ? rgba(84, 92, 108) : rgba(196, 120, 40));
    for (const auto& t : a.tags) {
        if (t == "imu") info.chips.emplace_back("IMU", rgba(52, 110, 150));
        else if (t == "camera") info.chips.emplace_back(tr(TextId::LibraryTagCamera), rgba(52, 110, 150));
        else if (t == "lidar2d") info.chips.emplace_back(tr(TextId::LibraryTagLidar), rgba(52, 110, 150));
        else if (t == "pd") info.chips.emplace_back("PD", rgba(96, 96, 140));
        else if (t == "fixed-base") info.chips.emplace_back(tr(TextId::LibraryTagFixedBase), rgba(96, 96, 140));
    }
    if (a.kind == lib::AssetKind::Scene) {
        if (const auto it = motionCounts.find(a.path); it != motionCounts.end()) {
            info.chips.emplace_back(std::to_string(it->second) + tr(TextId::LibraryTagMotions), rgba(120, 84, 176));
        }
    }
    if (a.kind == lib::AssetKind::Robot) info.chips.emplace_back(std::to_string(a.related.size()) + tr(TextId::LibraryTagScenes), rgba(70, 104, 150));
    if (hasTag(a, "models-missing")) info.hint = tr(TextId::LibraryModelsMissing);
    else if (hasTag(a, "invalid")) info.hint = std::string{tr(TextId::LibraryInvalid)} + a.description;
    else if (hasTag(a, "scene-missing")) info.hint = tr(TextId::LibrarySceneMissing);
    else info.hint = tr(a.kind == lib::AssetKind::Motion ? TextId::LibraryClickToPlay : TextId::LibraryClickToOpen);
    return info;
}

enum class CardStyle { Grid, List, Welcome };

std::optional<LibraryAction> card(LibraryPanelState& st, const LibraryContext& ctx, const lib::AssetEntry& a, const CardInfo& info, ImVec2 size,
                                  CardStyle style) {
    std::optional<LibraryAction> action;
    ImGui::PushID(static_cast<int>(a.kind));
    ImGui::PushID(a.path.c_str());
    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const bool pressed = ImGui::InvisibleButton("##card", size);
    const bool hovered = ImGui::IsItemHovered();
    const ImVec2 p1(p0.x + size.x, p0.y + size.y);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const float font = ImGui::GetFontSize();
    const float rounding = font * 0.35f;
    const bool current = a.kind == lib::AssetKind::Scene && !ctx.currentScene.empty() && a.path == ctx.currentScene;
    if (current) st.currentHighlighted = true;
    dl->AddRectFilled(p0, p1, ImGui::GetColorU32(hovered ? ImGuiCol_FrameBgHovered : ImGuiCol_FrameBg), rounding);
    if (current) dl->AddRect(p0, p1, ImGui::GetColorU32(ImGuiCol_CheckMark), rounding, std::max(2.0f, font * 0.14f));
    else if (hovered) dl->AddRect(p0, p1, ImGui::GetColorU32(ImGuiCol_Border), rounding);

    const bool dim = unusable(a);
    const float pad = font * 0.5f;
    const float iconSize = style == CardStyle::List ? size.y - 2 * pad : style == CardStyle::Welcome ? font * 3.4f : font * 2.6f;
    const float iconY = style == CardStyle::List ? p0.y + pad : p0.y + (size.y - iconSize) * 0.5f;
    drawAssetIcon(dl, ImVec2(p0.x + pad, iconY), iconSize, a.kind, dim);

    const float starR = font * 0.5f;
    const ImVec2 starC(p1.x - pad - starR, p0.y + pad + starR);
    const ImVec2 starMin(starC.x - starR * 1.4f, starC.y - starR * 1.4f), starMax(starC.x + starR * 1.4f, starC.y + starR * 1.4f);
    const bool starHovered = ImGui::IsMouseHoveringRect(starMin, starMax);
    if (a.favorite) drawStar(dl, starC, starR, true, rgba(250, 196, 50));
    else if (hovered) drawStar(dl, starC, starR, false, starHovered ? rgba(250, 196, 50) : ImGui::GetColorU32(ImGuiCol_TextDisabled));

    const float textX = p0.x + pad + iconSize + pad;
    const float textW = std::max(1.0f, starMin.x - textX - pad * 0.5f);
    ImFont* f = ImGui::GetFont();
    const ImU32 textColor = ImGui::GetColorU32(dim ? ImGuiCol_TextDisabled : ImGuiCol_Text);
    const float titleSize = style == CardStyle::Welcome ? font * 1.15f : font;
    const float smallSize = font * 0.8f;
    if (style == CardStyle::List) {
        dl->AddText(f, titleSize, ImVec2(textX, p0.y + (size.y - titleSize) * 0.5f), textColor, ellipsize(a.title, titleSize, textW).c_str());
    } else {
        float y = p0.y + pad * 0.8f;
        dl->AddText(f, titleSize, ImVec2(textX, y), textColor, ellipsize(a.title, titleSize, textW).c_str());
        y += titleSize * 1.25f;
        if (!info.subtitle.empty()) {
            dl->AddText(f, smallSize, ImVec2(textX, y), ImGui::GetColorU32(ImGuiCol_TextDisabled),
                        ellipsize(info.subtitle, smallSize, p1.x - pad - textX).c_str());
        }
        // 标签一行：当前、来源、传感器……放不下的省略
        float x = textX;
        const float chipY = p1.y - pad * 0.8f - smallSize * 1.35f;
        std::vector<std::pair<std::string, ImU32>> chips = info.chips;
        if (current) chips.insert(chips.begin(), std::make_pair(std::string{tr(TextId::LibraryCurrent)}, ImGui::GetColorU32(ImGuiCol_CheckMark)));
        for (const auto& [text, color] : chips) {
            const float wChip = f->CalcTextSizeA(smallSize, FLT_MAX, 0.0f, text.c_str()).x + smallSize * 0.8f;
            if (x + wChip > p1.x - pad) break;
            x += chip(dl, ImVec2(x, chipY), text, color, kChipText, smallSize) + smallSize * 0.35f;
        }
    }

    bool click = pressed;
    if (!st.selfTestClick.empty() && st.selfTestClick == a.path) {
        st.selfTestClick.clear();
        click = true;
    }
    if (click) {
        if (pressed && starHovered) {
            action = LibraryAction{LibraryAction::Type::ToggleFavorite, a.kind, a.path, a.importDirectory, !a.favorite};
        } else if (a.kind == lib::AssetKind::Robot) {
            if (!a.related.empty()) action = LibraryAction{LibraryAction::Type::Open, lib::AssetKind::Scene, a.related[0], {}, false};
        } else {
            action = LibraryAction{LibraryAction::Type::Open, a.kind, a.path, {}, false};
        }
    }
    if (hovered && !ImGui::IsPopupOpen("##ctx")) {
        if (starHovered) {
            ImGui::SetTooltip("%s", tr(a.favorite ? TextId::LibraryUnfavorite : TextId::LibraryFavorite));
        } else {
            ImGui::BeginTooltip();
            ImGui::PushTextWrapPos(font * 30.0f);
            ImGui::TextUnformatted(a.title.c_str());
            if (!a.description.empty() && a.description != a.title) ImGui::TextDisabled("%s", a.description.c_str());
            ImGui::TextDisabled("%s", a.path.c_str());
            ImGui::TextUnformatted(info.hint.c_str());
            ImGui::PopTextWrapPos();
            ImGui::EndTooltip();
        }
    }
    if (ImGui::BeginPopupContextItem("##ctx")) {
        if (ImGui::MenuItem(tr(a.favorite ? TextId::LibraryUnfavorite : TextId::LibraryFavorite))) {
            action = LibraryAction{LibraryAction::Type::ToggleFavorite, a.kind, a.path, a.importDirectory, !a.favorite};
        }
        if (a.kind == lib::AssetKind::Robot) {
            ImGui::Separator();
            for (const auto& scene : a.related) {
                const std::string text = std::string{tr(TextId::LibraryOpenScene)} + fs::path{scene}.filename().string();
                if (ImGui::MenuItem(text.c_str())) action = LibraryAction{LibraryAction::Type::Open, lib::AssetKind::Scene, scene, {}, false};
            }
        }
        ImGui::Separator();
        if (ImGui::MenuItem(tr(TextId::LibraryCopyPath))) action = LibraryAction{LibraryAction::Type::CopyPath, a.kind, a.path, {}, false};
        if (ImGui::MenuItem(tr(TextId::LibraryReveal))) action = LibraryAction{LibraryAction::Type::Reveal, a.kind, a.path, {}, false};
        if (!a.importDirectory.empty()) {
            if (ImGui::MenuItem(tr(TextId::LibraryReimport))) {
                action = LibraryAction{LibraryAction::Type::Reimport, a.kind, a.path, a.importDirectory, false};
            }
            if (ImGui::MenuItem(tr(TextId::LibraryExportPack))) {
                action = LibraryAction{LibraryAction::Type::ExportPack, a.kind, a.path, a.importDirectory, false};
            }
        }
        ImGui::Separator();
        if (ImGui::MenuItem(tr(TextId::LibraryHide))) action = LibraryAction{LibraryAction::Type::Hide, a.kind, a.path, {}, false};
        ImGui::EndPopup();
    }
    ImGui::PopID();
    ImGui::PopID();
    return action;
}

std::map<std::string, int> motionCountsOf(const LibraryContext& ctx) {
    std::map<std::string, int> counts;
    for (const auto& a : *ctx.assets) {
        if (a.kind == lib::AssetKind::Motion && !a.hidden && !a.related.empty()) ++counts[a.related[0]];
    }
    return counts;
}

// 卡片网格：按宽度分列，卡片等宽
std::optional<LibraryAction> cardGrid(LibraryPanelState& st, const LibraryContext& ctx, const std::vector<const lib::AssetEntry*>& items,
                                      float minWidth, float height, CardStyle style) {
    std::optional<LibraryAction> action;
    const auto counts = motionCountsOf(ctx);
    const float avail = ImGui::GetContentRegionAvail().x;
    const float gap = ImGui::GetStyle().ItemSpacing.x;
    const int cols = style == CardStyle::List ? 1 : std::max(1, static_cast<int>((avail + gap) / (minWidth + gap)));
    const float width = (avail - gap * static_cast<float>(cols - 1)) / static_cast<float>(cols);
    for (std::size_t i = 0; i < items.size(); ++i) {
        if (i % static_cast<std::size_t>(cols) != 0) ImGui::SameLine(0.0f, gap);
        if (auto a = card(st, ctx, *items[i], describe(*items[i], counts), ImVec2(width, height), style)) action = std::move(a);
    }
    return action;
}

int recentIndex(const LibraryContext& ctx, const lib::AssetEntry& a) {
    const auto indexOf = [&](const std::string& p) {
        const auto it = std::find(ctx.recent.begin(), ctx.recent.end(), p);
        return it == ctx.recent.end() ? -1 : static_cast<int>(it - ctx.recent.begin());
    };
    if (a.kind == lib::AssetKind::Scene) return indexOf(a.path);
    int best = -1;
    for (const auto& r : a.related) {
        const int k = indexOf(r);
        if (k >= 0 && (best < 0 || k < best)) best = k;
    }
    return best;
}

lib::AssetKind kindOfTab(LibraryTab tab) {
    switch (tab) {
        case LibraryTab::Scenes: return lib::AssetKind::Scene;
        case LibraryTab::Motions: return lib::AssetKind::Motion;
        case LibraryTab::Robots: return lib::AssetKind::Robot;
    }
    return lib::AssetKind::Scene;
}

}  // namespace

std::vector<const lib::AssetEntry*> visibleAssets(const LibraryContext& ctx, lib::AssetKind kind, LibraryFilter filter, const std::string& query,
                                                  bool recentFirst) {
    std::vector<const lib::AssetEntry*> out;
    if (ctx.assets == nullptr) return out;
    for (const auto& a : *ctx.assets) {
        if (a.kind != kind || a.hidden || a.missing) continue;
        if (filter == LibraryFilter::Favorites && !a.favorite) continue;
        if (filter == LibraryFilter::Recent && recentIndex(ctx, a) < 0) continue;
        if (filter == LibraryFilter::Builtin && a.source != lib::AssetSource::Builtin) continue;
        if (filter == LibraryFilter::Imported && a.source == lib::AssetSource::Builtin) continue;
        if (!lib::assetMatches(a, query)) continue;
        out.push_back(&a);
    }
    const bool byRecent = recentFirst || filter == LibraryFilter::Recent;
    std::stable_sort(out.begin(), out.end(), [&](const lib::AssetEntry* x, const lib::AssetEntry* y) {
        if (filter != LibraryFilter::Recent && x->favorite != y->favorite) return x->favorite;
        if (!byRecent) return false;
        const int rx = recentIndex(ctx, *x), ry = recentIndex(ctx, *y);
        if ((rx >= 0) != (ry >= 0)) return rx >= 0;
        return rx >= 0 && rx < ry;
    });
    return out;
}

std::optional<LibraryAction> drawLibraryPanel(LibraryPanelState& st, const LibraryContext& ctx) {
    std::optional<LibraryAction> action;
    st.panelCards = 0;
    st.currentHighlighted = false;
    const float h = ImGui::GetFrameHeight();
    const float spacing = ImGui::GetStyle().ItemSpacing.x;
    ImGui::SetNextItemWidth(std::max(h, ImGui::GetContentRegionAvail().x - 2.0f * (h + spacing)));
    ImGui::InputTextWithHint("##librarySearch", tr(TextId::LibrarySearchHint), st.search.data(), st.search.size());
    ImGui::SameLine();
    if (glyphButton("##grid", st.grid, gridGlyph, tr(TextId::FileBrowserGrid))) st.grid = true;
    ImGui::SameLine();
    if (glyphButton("##list", !st.grid, listGlyph, tr(TextId::FileBrowserList))) st.grid = false;

    const std::string query = st.search.data();
    if (ImGui::BeginTabBar("##libraryTabs")) {
        using Tab = std::pair<LibraryTab, TextId>;
        for (const auto& [tab, text] : {Tab{LibraryTab::Scenes, TextId::LibraryTabScenes}, Tab{LibraryTab::Motions, TextId::LibraryTabMotions},
                                        Tab{LibraryTab::Robots, TextId::LibraryTabRobots}}) {
            const std::size_t n = visibleAssets(ctx, kindOfTab(tab), LibraryFilter::All, {}, false).size();
            const std::string name = std::string{tr(text)} + " " + std::to_string(n) + "###tab" + std::to_string(static_cast<int>(tab));
            const bool select = st.selectTab && *st.selectTab == tab;
            if (ImGui::BeginTabItem(name.c_str(), nullptr, select ? ImGuiTabItemFlags_SetSelected : ImGuiTabItemFlags_None)) {
                st.tab = tab;
                ImGui::EndTabItem();
            }
        }
        ImGui::EndTabBar();
    }
    st.selectTab.reset();
    // 筛选：全部、收藏、最近、内置、导入
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(ImGui::GetStyle().FramePadding.x * 0.8f, ImGui::GetStyle().FramePadding.y * 0.5f));
    using Choice = std::pair<LibraryFilter, TextId>;
    bool first = true;
    for (const auto& [filter, text] : {Choice{LibraryFilter::All, TextId::LibraryFilterAll}, Choice{LibraryFilter::Favorites, TextId::LibraryFilterFavorites},
                                       Choice{LibraryFilter::Recent, TextId::LibraryFilterRecent}, Choice{LibraryFilter::Builtin, TextId::LibraryFilterBuiltin},
                                       Choice{LibraryFilter::Imported, TextId::LibraryFilterImported}}) {
        const float w = ImGui::CalcTextSize(tr(text)).x + ImGui::GetStyle().FramePadding.x * 2.0f;
        if (!first && ImGui::GetContentRegionAvail().x > w + spacing) ImGui::SameLine(0.0f, spacing * 0.6f);
        first = false;
        const bool active = st.filter == filter;
        if (active) ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        ImGui::PushID(static_cast<int>(filter));
        if (ImGui::Button(tr(text))) st.filter = filter;
        ImGui::PopID();
        if (active) ImGui::PopStyleColor();
    }
    ImGui::PopStyleVar();

    std::size_t hidden = 0;
    if (ctx.assets) {
        for (const auto& a : *ctx.assets) hidden += a.hidden && a.kind == kindOfTab(st.tab) ? 1u : 0u;
    }
    const float footer = ImGui::GetFrameHeightWithSpacing();
    const auto items = visibleAssets(ctx, kindOfTab(st.tab), st.filter, query, false);
    st.panelCards = items.size();
    const float font = ImGui::GetFontSize();
    if (ImGui::BeginChild("##cards", ImVec2(0.0f, -footer))) {
        if (items.empty()) ImGui::TextDisabled("%s", tr(TextId::LibraryEmpty));
        if (auto a = cardGrid(st, ctx, items, font * 13.0f, st.grid ? font * 4.4f : font * 2.0f, st.grid ? CardStyle::Grid : CardStyle::List)) {
            action = std::move(a);
        }
    }
    ImGui::EndChild();
    if (ImGui::SmallButton(tr(TextId::LibraryRescan))) action = LibraryAction{LibraryAction::Type::Rescan, lib::AssetKind::Scene, {}, {}, false};
    if (hidden > 0) {
        ImGui::SameLine();
        ImGui::TextDisabled("%s%zu", tr(TextId::LibraryHiddenCount), hidden);
        ImGui::SameLine();
        if (ImGui::SmallButton(tr(TextId::LibraryUnhideAll))) {
            action = LibraryAction{LibraryAction::Type::UnhideAll, kindOfTab(st.tab), {}, {}, false};
        }
    }
    return action;
}

std::optional<LibraryAction> drawWelcome(LibraryPanelState& st, const LibraryContext& ctx, ImVec2 min, ImVec2 max) {
    st.welcomeCards = 0;
    const float font = ImGui::GetFontSize();
    const float width = max.x - min.x, height = max.y - min.y;
    if (width < font * 20.0f || height < font * 12.0f) return std::nullopt;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(min, max, rgba(14, 16, 22, 175));
    const float margin = font * 2.0f;
    const float contentW = std::min(width - 2.0f * margin, font * 72.0f);
    const float x0 = min.x + (width - contentW) * 0.5f;
    float y = min.y + margin;
    ImFont* f = ImGui::GetFont();
    dl->AddText(f, font * 1.6f, ImVec2(x0, y), ImGui::GetColorU32(ImGuiCol_Text), tr(TextId::WelcomeTitle));
    y += font * 2.2f;
    dl->AddText(f, font * 0.9f, ImVec2(x0, y), ImGui::GetColorU32(ImGuiCol_TextDisabled), tr(TextId::WelcomeHint));
    y += font * 1.8f;
    ImGui::SetCursorScreenPos(ImVec2(x0, y));
    std::optional<LibraryAction> action;
    const auto items = visibleAssets(ctx, lib::AssetKind::Scene, LibraryFilter::All, {}, true);
    st.welcomeCards = items.size();
    if (ImGui::BeginChild("##welcome", ImVec2(contentW, std::max(font * 6.0f, max.y - margin - y)), ImGuiChildFlags_None,
                          ImGuiWindowFlags_NoBackground)) {
        if (items.empty()) ImGui::TextDisabled("%s", tr(TextId::LibraryEmpty));
        action = cardGrid(st, ctx, items, font * 16.0f, font * 5.6f, CardStyle::Welcome);
    }
    ImGui::EndChild();
    return action;
}

std::optional<LibraryAction> drawQuickOpen(LibraryPanelState& st, const LibraryContext& ctx) {
    if (!st.quickOpen) return std::nullopt;
    std::optional<LibraryAction> action;
    const float font = ImGui::GetFontSize();
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const float width = std::min(font * 40.0f, vp->WorkSize.x * 0.9f);
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x * 0.5f, vp->WorkPos.y + vp->WorkSize.y * 0.12f), ImGuiCond_Always, ImVec2(0.5f, 0.0f));
    if (st.quickFrames == 0) ImGui::SetNextWindowFocus();
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                                   ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize;
    bool close = false;
    if (ImGui::Begin("###QuickOpen", nullptr, flags)) {
        ImGui::TextDisabled("%s", tr(TextId::QuickOpenTitle));
        if (st.quickFocus) {
            ImGui::SetKeyboardFocusHere();
            st.quickFocus = false;
        }
        ImGui::SetNextItemWidth(width);
        if (ImGui::InputTextWithHint("##quick", tr(TextId::LibrarySearchHint), st.quickSearch.data(), st.quickSearch.size())) st.quickSelected = 0;
        const std::string query = st.quickSearch.data();
        auto matches = visibleAssets(ctx, lib::AssetKind::Scene, LibraryFilter::All, query, true);
        for (const auto* m : visibleAssets(ctx, lib::AssetKind::Motion, LibraryFilter::All, query, true)) matches.push_back(m);
        if (matches.size() > 12) matches.resize(12);
        st.quickMatches = matches.size();
        const int count = static_cast<int>(matches.size());
        if (ImGui::IsKeyPressed(ImGuiKey_DownArrow)) st.quickSelected = std::min(st.quickSelected + 1, std::max(0, count - 1));
        if (ImGui::IsKeyPressed(ImGuiKey_UpArrow)) st.quickSelected = std::max(st.quickSelected - 1, 0);
        st.quickSelected = std::clamp(st.quickSelected, 0, std::max(0, count - 1));
        const bool enter = ImGui::IsKeyPressed(ImGuiKey_Enter) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter) || st.selfTestQuickEnter;
        st.selfTestQuickEnter = false;
        if (matches.empty()) ImGui::TextDisabled("%s", tr(TextId::LibraryEmpty));
        ImDrawList* dl = ImGui::GetWindowDrawList();
        for (int i = 0; i < count; ++i) {
            const auto& a = *matches[static_cast<std::size_t>(i)];
            ImGui::PushID(i);
            const ImVec2 at = ImGui::GetCursorScreenPos();
            const float icon = ImGui::GetTextLineHeight();
            const std::string text = "      " + ellipsize(a.title, font, width * 0.55f);
            if (ImGui::Selectable(text.c_str(), i == st.quickSelected, ImGuiSelectableFlags_None, ImVec2(width, 0.0f))) {
                action = LibraryAction{LibraryAction::Type::Open, a.kind, a.path, {}, false};
            }
            drawAssetIcon(dl, ImVec2(at.x, at.y), icon, a.kind, unusable(a));
            const std::string detail = ellipsize(a.description == a.title ? fs::path{a.path}.filename().string() : a.description, font * 0.8f, width * 0.4f);
            dl->AddText(ImGui::GetFont(), font * 0.8f, ImVec2(at.x + width * 0.58f, at.y + font * 0.1f), ImGui::GetColorU32(ImGuiCol_TextDisabled),
                        detail.c_str());
            ImGui::PopID();
        }
        if (enter && count > 0) {
            const auto& a = *matches[static_cast<std::size_t>(st.quickSelected)];
            action = LibraryAction{LibraryAction::Type::Open, a.kind, a.path, {}, false};
        }
        ImGui::TextDisabled("%s", tr(TextId::QuickOpenHint));
        if (ImGui::IsKeyPressed(ImGuiKey_Escape)) close = true;
        if (st.quickFrames > 1 && !ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows)) close = true;  // 点到别处
    }
    ImGui::End();
    ++st.quickFrames;
    if (action || close) {
        st.quickOpen = false;
        st.quickSearch.fill('\0');
        st.quickSelected = 0;
        st.quickFrames = 0;
    }
    return action;
}

void revealInFileManager(const std::string& path) {
    std::error_code ec;
    const std::string dir = fs::is_directory(path, ec) ? path : fs::path{path}.parent_path().string();
    // sh 在后台启动 xdg-open 后立即退出，这里等 sh 结束（不留僵尸进程）
    std::string sh = "sh", flag = "-c", script = "xdg-open \"$1\" >/dev/null 2>&1 &", name = "sh", arg = dir;
    char* argv[] = {sh.data(), flag.data(), script.data(), name.data(), arg.data(), nullptr};
    pid_t pid = 0;
    if (posix_spawnp(&pid, "sh", nullptr, nullptr, argv, environ) == 0) {
        int status = 0;
        waitpid(pid, &status, 0);
    }
}

}  // namespace terra::viewer
