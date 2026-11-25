#include "terra/ui/file_browser.hpp"

#include <algorithm>
#include <cfloat>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <system_error>
#include <utility>
#include <vector>

#include "imgui.h"
#include "terra/ui/text.hpp"

namespace terra::ui {

namespace fs = std::filesystem;

namespace {

bool endsWith(const std::string& s, const std::string& suffix) {
    return s.size() >= suffix.size() && s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// filter 中用 “|” 分隔的任一后缀
bool matches(const std::string& name, const std::string& filter) {
    std::size_t start = 0;
    while (start <= filter.size()) {
        const std::size_t end = filter.find('|', start);
        const std::string suffix = filter.substr(start, end == std::string::npos ? std::string::npos : end - start);
        if (!suffix.empty() && endsWith(name, suffix)) return true;
        if (end == std::string::npos) break;
        start = end + 1;
    }
    return false;
}

void copyTo(char* buffer, std::size_t size, const std::string& text) {
    std::snprintf(buffer, size, "%s", text.c_str());
}

std::string humanSize(std::uintmax_t bytes) {
    char buf[32];
    if (bytes < 1024) {
        std::snprintf(buf, sizeof buf, "%ju B", bytes);
    } else if (bytes < 1024 * 1024) {
        std::snprintf(buf, sizeof buf, "%.1f KB", static_cast<double>(bytes) / 1024.0);
    } else {
        std::snprintf(buf, sizeof buf, "%.1f MB", static_cast<double>(bytes) / (1024.0 * 1024.0));
    }
    return buf;
}

}  // namespace

DirectoryListing listDirectory(const std::string& dir, const std::string& filter) {
    DirectoryListing out;
    std::error_code ec;
    fs::directory_iterator it(dir, fs::directory_options::skip_permission_denied, ec);
    if (ec) {
        out.error = ec.message();
        return out;
    }
    for (const fs::directory_entry& e : it) {
        const std::string name = e.path().filename().string();
        if (name.empty() || name[0] == '.') continue;
        std::error_code ignored;
        DirectoryEntry entry;
        entry.name = name;
        entry.directory = e.is_directory(ignored);
        if (!entry.directory) {
            if (!e.is_regular_file(ignored)) continue;
            if (!filter.empty() && !matches(name, filter)) continue;
            entry.size = e.file_size(ignored);
        }
        out.entries.push_back(std::move(entry));
    }
    std::sort(out.entries.begin(), out.entries.end(), [](const DirectoryEntry& a, const DirectoryEntry& b) {
        if (a.directory != b.directory) return a.directory;
        return a.name < b.name;
    });
    return out;
}

std::string normalizeDirectory(const std::string& path) {
    std::error_code ec;
    fs::path p = path.empty() ? fs::current_path(ec) : fs::absolute(path, ec);
    p = p.lexically_normal();
    for (int guard = 0; guard < 64 && !p.empty(); ++guard) {
        if (fs::is_directory(p, ec)) {
            std::string s = p.string();
            if (s.size() > 1 && s.back() == '/') s.pop_back();
            return s;
        }
        const fs::path parent = p.parent_path();
        if (parent == p) break;
        p = parent;
    }
    return fs::current_path(ec).string();
}

void FileBrowser::open(const std::string& startPath, std::string filter, std::string fallback) {
    filter_ = std::move(filter);
    fallback_ = std::move(fallback);
    std::error_code ec;
    if (!startPath.empty() && fs::is_regular_file(startPath, ec)) fileName_ = fs::path{startPath}.filename().string();
    enter(normalizeDirectory(startPath));
    open_ = true;
    requestOpen_ = true;
}

void FileBrowser::enter(const std::string& dir) {
    directory_ = dir;
    pathEdit_ = dir;
    refresh();
}

void FileBrowser::refresh() {
    listing_ = listDirectory(directory_, onlyFilter_ ? filter_ : fallback_);
}

namespace {

// ---- 矢量图标（M4-08a，用户 2026-11-25：网格视图 + 矢量图标）：用 ImDrawList 画，任何缩放下都清晰 ----

enum class FileKind { Folder, Scene, Motion, Record, Urdf, Xacro, Mjcf, Json, Image, Mesh, Other };

FileKind kindOf(const DirectoryEntry& e) {
    if (e.directory) return FileKind::Folder;
    const std::string& n = e.name;
    if (endsWith(n, ".terra-scene.json")) return FileKind::Scene;
    if (endsWith(n, ".terra-motion.json")) return FileKind::Motion;
    if (endsWith(n, ".terra-record.jsonl")) return FileKind::Record;
    if (endsWith(n, ".urdf")) return FileKind::Urdf;
    if (endsWith(n, ".xacro")) return FileKind::Xacro;
    if (endsWith(n, ".xml") || endsWith(n, ".mjcf")) return FileKind::Mjcf;
    if (endsWith(n, ".json") || endsWith(n, ".jsonl")) return FileKind::Json;
    if (endsWith(n, ".png") || endsWith(n, ".jpg") || endsWith(n, ".jpeg") || endsWith(n, ".hdr")) return FileKind::Image;
    if (endsWith(n, ".stl") || endsWith(n, ".obj") || endsWith(n, ".dae") || endsWith(n, ".glb") || endsWith(n, ".gltf")) return FileKind::Mesh;
    return FileKind::Other;
}

// 文件类型的标签与颜色
struct Badge {
    const char* text;
    ImU32 color;
};

Badge badgeOf(FileKind k) {
    switch (k) {
        case FileKind::Folder: return {"", IM_COL32(232, 178, 64, 255)};
        case FileKind::Scene: return {"SCENE", IM_COL32(76, 175, 110, 255)};
        case FileKind::Motion: return {"MOTION", IM_COL32(156, 110, 220, 255)};
        case FileKind::Record: return {"REC", IM_COL32(220, 90, 80, 255)};
        case FileKind::Urdf: return {"URDF", IM_COL32(70, 140, 230, 255)};
        case FileKind::Xacro: return {"XACRO", IM_COL32(60, 110, 200, 255)};
        case FileKind::Mjcf: return {"MJCF", IM_COL32(40, 170, 180, 255)};
        case FileKind::Json: return {"JSON", IM_COL32(150, 150, 160, 255)};
        case FileKind::Image: return {"IMG", IM_COL32(220, 110, 170, 255)};
        case FileKind::Mesh: return {"MESH", IM_COL32(200, 140, 70, 255)};
        case FileKind::Other: return {"", IM_COL32(130, 130, 140, 255)};
    }
    return {"", IM_COL32(130, 130, 140, 255)};
}

// 在 [a, a + size] 的方框中画图标
void drawIcon(ImDrawList* dl, ImVec2 a, float size, FileKind kind) {
    const Badge badge = badgeOf(kind);
    const float u = size / 16.0f;
    if (kind == FileKind::Folder) {
        const ImU32 back = IM_COL32(196, 146, 46, 255);
        dl->AddRectFilled(ImVec2(a.x + 1 * u, a.y + 3 * u), ImVec2(a.x + 7 * u, a.y + 6 * u), back, 1.2f * u);  // 标签
        dl->AddRectFilled(ImVec2(a.x + 1 * u, a.y + 4.5f * u), ImVec2(a.x + 15 * u, a.y + 14 * u), back, 1.5f * u);
        dl->AddRectFilled(ImVec2(a.x + 1 * u, a.y + 6 * u), ImVec2(a.x + 15 * u, a.y + 14 * u), badge.color, 1.5f * u);
        return;
    }
    // 纸张：右上角折起
    const ImU32 paper = IM_COL32(236, 238, 242, 255);
    const ImU32 fold = IM_COL32(190, 194, 204, 255);
    const ImVec2 p0(a.x + 3 * u, a.y + 1 * u), p1(a.x + 10 * u, a.y + 1 * u), p2(a.x + 13 * u, a.y + 4 * u), p3(a.x + 13 * u, a.y + 15 * u),
        p4(a.x + 3 * u, a.y + 15 * u);
    const ImVec2 page[] = {p0, p1, p2, p3, p4};
    dl->AddConvexPolyFilled(page, 5, paper);
    const ImVec2 corner[] = {p1, ImVec2(a.x + 10 * u, a.y + 4 * u), p2};
    dl->AddConvexPolyFilled(corner, 3, fold);
    dl->AddPolyline(page, 5, IM_COL32(120, 124, 136, 255), 0.6f * u, ImDrawFlags_Closed);
    // 内容线
    for (int k = 0; k < 2; ++k) {
        const float y = a.y + (6.0f + 2.0f * static_cast<float>(k)) * u;
        dl->AddLine(ImVec2(a.x + 5 * u, y), ImVec2(a.x + 11 * u, y), fold, 0.8f * u);
    }
    // 类型标签：彩色条 + 文字
    if (badge.text[0] != '\0') {
        const ImVec2 b0(a.x + 1.5f * u, a.y + 10 * u), b1(a.x + 14.5f * u, a.y + 14 * u);
        dl->AddRectFilled(b0, b1, badge.color, 1.0f * u);
        ImFont* font = ImGui::GetFont();
        const float fontSize = std::min(3.4f * u, ImGui::GetFontSize());
        const ImVec2 t = font->CalcTextSizeA(fontSize, FLT_MAX, 0.0f, badge.text);
        const float scale = t.x > 12.0f * u ? 12.0f * u / t.x : 1.0f;
        dl->AddText(font, fontSize * scale, ImVec2((b0.x + b1.x - t.x * scale) * 0.5f, (b0.y + b1.y - t.y * scale) * 0.5f), IM_COL32_WHITE,
                    badge.text);
    } else {
        dl->AddRectFilled(ImVec2(a.x + 1.5f * u, a.y + 11.5f * u), ImVec2(a.x + 14.5f * u, a.y + 13.5f * u), badge.color, 1.0f * u);
    }
}

// “上一级”与视图切换用的小图标
void drawGridGlyph(ImDrawList* dl, ImVec2 a, float s, ImU32 c) {
    const float g = s * 0.12f, w = (s - g) * 0.5f;
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            const ImVec2 p(a.x + static_cast<float>(i) * (w + g), a.y + static_cast<float>(j) * (w + g));
            dl->AddRectFilled(p, ImVec2(p.x + w, p.y + w), c, s * 0.08f);
        }
    }
}

void drawListGlyph(ImDrawList* dl, ImVec2 a, float s, ImU32 c) {
    for (int k = 0; k < 3; ++k) {
        const float y = a.y + s * (0.15f + 0.35f * static_cast<float>(k));
        dl->AddRectFilled(ImVec2(a.x, y - s * 0.08f), ImVec2(a.x + s * 0.18f, y + s * 0.08f), c, s * 0.04f);
        dl->AddRectFilled(ImVec2(a.x + s * 0.28f, y - s * 0.07f), ImVec2(a.x + s, y + s * 0.07f), c, s * 0.04f);
    }
}

void drawUpGlyph(ImDrawList* dl, ImVec2 a, float s, ImU32 c) {
    const ImVec2 tip(a.x + s * 0.5f, a.y + s * 0.1f);
    dl->AddTriangleFilled(tip, ImVec2(a.x + s * 0.12f, a.y + s * 0.5f), ImVec2(a.x + s * 0.88f, a.y + s * 0.5f), c);
    dl->AddRectFilled(ImVec2(a.x + s * 0.38f, a.y + s * 0.45f), ImVec2(a.x + s * 0.62f, a.y + s * 0.9f), c);
}

// 图标按钮（方形，按下返回 true）
bool glyphButton(const char* id, bool active, void (*glyph)(ImDrawList*, ImVec2, float, ImU32), const char* tooltip) {
    const float h = ImGui::GetFrameHeight();
    if (active) ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
    const bool pressed = ImGui::Button(id, ImVec2(h, h));
    if (active) ImGui::PopStyleColor();
    const ImVec2 a = ImGui::GetItemRectMin();
    const float pad = h * 0.22f;
    glyph(ImGui::GetWindowDrawList(), ImVec2(a.x + pad, a.y + pad), h - 2 * pad, ImGui::GetColorU32(ImGuiCol_Text));
    if (ImGui::IsItemHovered() && tooltip != nullptr) ImGui::SetTooltip("%s", tooltip);
    return pressed;
}

// 名字按宽度截成至多两行（超出的加省略号）
std::vector<std::string> wrapName(const std::string& name, float width) {
    std::vector<std::string> lines;
    std::string line;
    std::size_t i = 0;
    while (i < name.size() && lines.size() < 2) {
        // 一个 UTF-8 字符
        std::size_t len = 1;
        const auto c = static_cast<unsigned char>(name[i]);
        if (c >= 0xF0) len = 4;
        else if (c >= 0xE0) len = 3;
        else if (c >= 0xC0) len = 2;
        const std::string ch = name.substr(i, len);
        if (ImGui::CalcTextSize((line + ch).c_str()).x > width && !line.empty()) {
            lines.push_back(line);
            line.clear();
            if (lines.size() == 2) break;
        }
        line += ch;
        i += len;
    }
    if (lines.size() < 2 && !line.empty()) lines.push_back(line);
    if (i < name.size() && !lines.empty()) {
        std::string& last = lines.back();
        while (!last.empty() && ImGui::CalcTextSize((last + "…").c_str()).x > width) {
            // 去掉最后一个 UTF-8 字符
            std::size_t cut = last.size() - 1;
            while (cut > 0 && (static_cast<unsigned char>(last[cut]) & 0xC0) == 0x80) --cut;
            last.erase(cut);
        }
        last += "…";
    }
    return lines;
}

}  // namespace

std::optional<std::string> FileBrowser::draw(const char* title) {
    if (!open_) return std::nullopt;
    if (requestOpen_) {
        ImGui::OpenPopup(title);
        requestOpen_ = false;
    }
    std::optional<std::string> chosen;
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowSize(ImVec2(vp->WorkSize.x * 0.6f, vp->WorkSize.y * 0.65f), ImGuiCond_Appearing);
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x * 0.5f, vp->WorkPos.y + vp->WorkSize.y * 0.5f), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    bool keepOpen = true;
    if (ImGui::BeginPopupModal(title, &keepOpen)) {
        const float font = ImGui::GetFontSize();
        std::optional<std::string> enterDir;
        // 工具栏：上一级、地址栏（M4-08b，与 Windows 文件管理器相同：逐级可点，点空白处直接编辑）、视图切换
        if (glyphButton("##up", false, drawUpGlyph, tr(TextId::FileBrowserUp))) {
            const fs::path parent = fs::path{directory_}.parent_path();
            if (!parent.empty() && parent.string() != directory_) enterDir = parent.string();
        }
        ImGui::SameLine();
        const float viewButtons = ImGui::GetFrameHeight() * 2.0f + ImGui::GetStyle().ItemSpacing.x * 2.0f;
        bool pathEscape = false;  // 地址栏中的 Esc 只退出编辑，不关闭窗口
        if (editPath_) {
            char pathBuf[1024];
            copyTo(pathBuf, sizeof pathBuf, pathEdit_);
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - viewButtons);
            if (focusPath_) {
                ImGui::SetKeyboardFocusHere();
                focusPath_ = false;
            }
            const bool entered = ImGui::InputText("##path", pathBuf, sizeof pathBuf, ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll);
            pathEdit_ = pathBuf;
            if (entered) {
                enterDir = normalizeDirectory(pathBuf);
                editPath_ = false;
            } else if (ImGui::IsItemDeactivated()) {
                // Esc 或点到别处：恢复为逐级按钮
                pathEscape = ImGui::IsKeyPressed(ImGuiKey_Escape);
                editPath_ = false;
            }
        } else {
            // 地址栏的底色：先在下层通道画框，最后合并
            ImDrawList* bar = ImGui::GetWindowDrawList();
            bar->ChannelsSplit(2);
            bar->ChannelsSetCurrent(1);
            const ImVec2 barStart = ImGui::GetCursorScreenPos();
            const float barWidth = ImGui::GetContentRegionAvail().x - viewButtons;
            // 面包屑：根目录与每一级目录各是一个按钮；放不下时只显示后面几级
            std::vector<std::pair<std::string, std::string>> crumbs;  // 显示名、路径
            fs::path acc;
            for (const auto& part : fs::path{directory_}) {
                acc /= part;
                crumbs.emplace_back(part.string() == "/" ? "/" : part.string(), acc.string());
            }
            const float avail = ImGui::GetContentRegionAvail().x - viewButtons - font * 3.0f;
            std::size_t first = 0;
            float width = 0.0f;
            for (std::size_t k = crumbs.size(); k-- > 0;) {
                width += ImGui::CalcTextSize(crumbs[k].first.c_str()).x + ImGui::GetStyle().FramePadding.x * 2 + font;
                if (width > avail) {
                    first = k + 1;
                    break;
                }
            }
            if (first > 0) {
                ImGui::TextDisabled("…");
                ImGui::SameLine();
            }
            for (std::size_t k = first; k < crumbs.size(); ++k) {
                ImGui::PushID(static_cast<int>(k));
                if (ImGui::SmallButton(crumbs[k].first.c_str())) enterDir = crumbs[k].second;
                ImGui::PopID();
                ImGui::SameLine(0.0f, 2.0f);
                if (k + 1 < crumbs.size()) {
                    ImGui::TextDisabled("/");
                    ImGui::SameLine(0.0f, 2.0f);
                }
            }
            // 按钮之后的空白处：点击即进入编辑（选中全部路径）
            const float rest = barStart.x + barWidth - ImGui::GetCursorScreenPos().x;
            if (rest > 1.0f) {
                ImGui::InvisibleButton("##pathblank", ImVec2(rest, ImGui::GetFrameHeight()));
                if (ImGui::IsItemHovered()) {
                    ImGui::SetMouseCursor(ImGuiMouseCursor_TextInput);
                    ImGui::SetTooltip("%s", tr(TextId::FileBrowserEditPath));
                }
                if (ImGui::IsItemClicked(ImGuiMouseButton_Left)) {
                    editPath_ = true;
                    focusPath_ = true;
                    pathEdit_ = directory_;
                }
            }
            bar->ChannelsSetCurrent(0);
            bar->AddRectFilled(barStart, ImVec2(barStart.x + barWidth, barStart.y + ImGui::GetFrameHeight()), ImGui::GetColorU32(ImGuiCol_FrameBg),
                               ImGui::GetStyle().FrameRounding);
            bar->ChannelsMerge();
        }
        // 视图切换靠右（ImGui 1.93 没有 GetWindowContentRegionMax：用当前位置加剩余宽度）
        ImGui::SameLine();
        ImGui::SetCursorPosX(std::max(ImGui::GetCursorPosX(), ImGui::GetCursorPosX() + ImGui::GetContentRegionAvail().x - viewButtons + ImGui::GetStyle().ItemSpacing.x));
        if (glyphButton("##grid", grid_, drawGridGlyph, tr(TextId::FileBrowserGrid))) grid_ = true;
        ImGui::SameLine();
        if (glyphButton("##list", !grid_, drawListGlyph, tr(TextId::FileBrowserList))) grid_ = false;

        // 文件区域
        const float footer = ImGui::GetFrameHeightWithSpacing() * 2.2f;
        if (ImGui::BeginChild("##entries", ImVec2(0, -footer), ImGuiChildFlags_Borders)) {
            if (!listing_.error.empty()) {
                ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.4f, 1.0f), "%s", listing_.error.c_str());
            } else if (listing_.entries.empty()) {
                ImGui::TextDisabled("%s", tr(TextId::FileBrowserEmpty));
            }
            const auto activate = [&](const DirectoryEntry& e, bool doubleClicked) {
                if (e.directory) {
                    if (doubleClicked) enterDir = (fs::path{directory_} / e.name).string();
                } else {
                    fileName_ = e.name;
                    if (doubleClicked) chosen = (fs::path{directory_} / e.name).string();
                }
            };
            ImDrawList* dl = ImGui::GetWindowDrawList();
            if (grid_) {
                const float tileW = font * 6.5f;
                const float iconSize = font * 3.2f;
                const float tileH = iconSize + font * 3.0f;
                const float spacing = font * 0.5f;
                const int columns = std::max(1, static_cast<int>((ImGui::GetContentRegionAvail().x + spacing) / (tileW + spacing)));
                for (std::size_t k = 0; k < listing_.entries.size(); ++k) {
                    const DirectoryEntry& e = listing_.entries[k];
                    if (k % static_cast<std::size_t>(columns) != 0) ImGui::SameLine(0.0f, spacing);
                    ImGui::PushID(static_cast<int>(k));
                    const ImVec2 a = ImGui::GetCursorScreenPos();
                    ImGui::InvisibleButton("##tile", ImVec2(tileW, tileH));
                    const bool hovered = ImGui::IsItemHovered();
                    const bool selected = !e.directory && e.name == fileName_;
                    if (ImGui::IsItemClicked(ImGuiMouseButton_Left)) activate(e, ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left));
                    if (selected || hovered) {
                        dl->AddRectFilled(a, ImVec2(a.x + tileW, a.y + tileH),
                                          ImGui::GetColorU32(selected ? ImGuiCol_HeaderActive : ImGuiCol_HeaderHovered), font * 0.3f);
                    }
                    drawIcon(dl, ImVec2(a.x + (tileW - iconSize) * 0.5f, a.y + font * 0.4f), iconSize, kindOf(e));
                    float y = a.y + font * 0.6f + iconSize;
                    for (const auto& line : wrapName(e.name, tileW - font * 0.4f)) {
                        const float w = ImGui::CalcTextSize(line.c_str()).x;
                        dl->AddText(ImVec2(a.x + (tileW - w) * 0.5f, y), ImGui::GetColorU32(ImGuiCol_Text), line.c_str());
                        y += font * 1.1f;
                    }
                    if (hovered) {
                        if (e.directory) ImGui::SetTooltip("%s", e.name.c_str());
                        else ImGui::SetTooltip("%s\n%s", e.name.c_str(), humanSize(e.size).c_str());
                    }
                    ImGui::PopID();
                }
            } else if (ImGui::BeginTable("##list", 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchProp)) {
                ImGui::TableSetupColumn(tr(TextId::FileBrowserName), ImGuiTableColumnFlags_WidthStretch, 4.0f);
                ImGui::TableSetupColumn(tr(TextId::FileBrowserType), ImGuiTableColumnFlags_WidthStretch, 1.0f);
                ImGui::TableSetupColumn(tr(TextId::FileBrowserSize), ImGuiTableColumnFlags_WidthStretch, 1.0f);
                ImGui::TableHeadersRow();
                for (std::size_t k = 0; k < listing_.entries.size(); ++k) {
                    const DirectoryEntry& e = listing_.entries[k];
                    ImGui::PushID(static_cast<int>(k));
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    const ImVec2 a = ImGui::GetCursorScreenPos();
                    const bool selected = !e.directory && e.name == fileName_;
                    // M4-08b（用户要求）：图标与文件名之间留出间隔
                    const float iconSize = font * 1.2f;
                    const float rowHeight = std::max(iconSize, ImGui::GetTextLineHeight());
                    if (ImGui::Selectable("##row", selected, ImGuiSelectableFlags_AllowDoubleClick | ImGuiSelectableFlags_SpanAllColumns,
                                          ImVec2(0.0f, rowHeight))) {
                        activate(e, ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left));
                    }
                    drawIcon(dl, ImVec2(a.x, a.y + (rowHeight - iconSize) * 0.5f), iconSize, kindOf(e));
                    ImGui::SameLine(0.0f, 0.0f);
                    ImGui::SetCursorScreenPos(ImVec2(a.x + iconSize + font * 0.6f, a.y + (rowHeight - ImGui::GetTextLineHeight()) * 0.5f));
                    ImGui::TextUnformatted(e.name.c_str());
                    ImGui::TableSetColumnIndex(1);
                    const Badge badge = badgeOf(kindOf(e));
                    ImGui::TextDisabled("%s", e.directory ? tr(TextId::FileBrowserFolder) : badge.text);
                    ImGui::TableSetColumnIndex(2);
                    if (!e.directory) ImGui::TextDisabled("%s", humanSize(e.size).c_str());
                    ImGui::PopID();
                }
                ImGui::EndTable();
            }
            // 在文件区域按退格：上一级（不在输入框中时）
            if (ImGui::IsWindowFocused() && !ImGui::GetIO().WantTextInput && ImGui::IsKeyPressed(ImGuiKey_Backspace)) {
                const fs::path parent = fs::path{directory_}.parent_path();
                if (!parent.empty() && parent.string() != directory_) enterDir = parent.string();
            }
        }
        ImGui::EndChild();
        if (enterDir) enter(normalizeDirectory(*enterDir));

        // 文件名、过滤、按钮
        char nameBuf[512];
        copyTo(nameBuf, sizeof nameBuf, fileName_);
        ImGui::TextUnformatted(tr(TextId::FileBrowserName));
        ImGui::SameLine();
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.5f);
        const bool enterPressed = ImGui::InputText("##name", nameBuf, sizeof nameBuf, ImGuiInputTextFlags_EnterReturnsTrue);
        fileName_ = nameBuf;
        ImGui::SameLine();
        // M3-04c：过滤后缀随用途变化（场景文件、动作文件）
        const std::string onlyLabel = std::string{tr(TextId::FileBrowserOnlyScenes)} + filter_ + "###onlyFilter";
        if (ImGui::Checkbox(onlyLabel.c_str(), &onlyFilter_)) refresh();
        if (ImGui::Button(tr(TextId::FileBrowserOpen)) || enterPressed) {
            if (!fileName_.empty()) {
                const fs::path p = fs::path{fileName_}.is_absolute() ? fs::path{fileName_} : fs::path{directory_} / fileName_;
                chosen = p.lexically_normal().string();
            }
        }
        ImGui::SameLine();
        if (ImGui::Button(tr(TextId::FileBrowserCancel)) || (ImGui::IsKeyPressed(ImGuiKey_Escape) && !pathEscape && !editPath_)) keepOpen = false;
        if (chosen || !keepOpen) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    } else {
        keepOpen = false;
    }
    if (chosen || !keepOpen) open_ = false;
    return chosen;
}

}  // namespace terra::ui
