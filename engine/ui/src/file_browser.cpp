#include "terra/ui/file_browser.hpp"

#include <algorithm>
#include <cfloat>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <system_error>

#include "imgui.h"
#include "terra/ui/text.hpp"

namespace terra::ui {

namespace fs = std::filesystem;

namespace {

bool endsWith(const std::string& s, const std::string& suffix) {
    return s.size() >= suffix.size() && s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
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
            if (!filter.empty() && !endsWith(name, filter)) continue;
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

void FileBrowser::open(const std::string& startPath, std::string filter) {
    filter_ = std::move(filter);
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
    listing_ = listDirectory(directory_, onlyFilter_ ? filter_ : std::string{".json"});
}

std::optional<std::string> FileBrowser::draw(const char* title) {
    if (!open_) return std::nullopt;
    if (requestOpen_) {
        ImGui::OpenPopup(title);
        requestOpen_ = false;
    }
    std::optional<std::string> chosen;
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowSize(ImVec2(vp->WorkSize.x * 0.55f, vp->WorkSize.y * 0.6f), ImGuiCond_Appearing);
    ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + vp->WorkSize.x * 0.5f, vp->WorkPos.y + vp->WorkSize.y * 0.5f), ImGuiCond_Appearing,
                            ImVec2(0.5f, 0.5f));
    bool keepOpen = true;
    if (ImGui::BeginPopupModal(title, &keepOpen)) {
        // 目录行：上一级 + 可编辑路径
        if (ImGui::Button(tr(TextId::FileBrowserUp))) {
            const fs::path parent = fs::path{directory_}.parent_path();
            if (!parent.empty() && parent.string() != directory_) enter(parent.string());
        }
        ImGui::SameLine();
        char pathBuf[1024];
        copyTo(pathBuf, sizeof pathBuf, pathEdit_);
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (ImGui::InputText("##path", pathBuf, sizeof pathBuf, ImGuiInputTextFlags_EnterReturnsTrue)) {
            enter(normalizeDirectory(pathBuf));
        } else {
            pathEdit_ = pathBuf;
        }

        // 列表
        const float footer = ImGui::GetFrameHeightWithSpacing() * 2.2f;
        if (ImGui::BeginChild("##entries", ImVec2(0, -footer), ImGuiChildFlags_Borders)) {
            if (!listing_.error.empty()) {
                ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.4f, 1.0f), "%s", listing_.error.c_str());
            } else if (listing_.entries.empty()) {
                ImGui::TextDisabled("%s", tr(TextId::FileBrowserEmpty));
            }
            std::optional<std::string> enterDir;
            for (const DirectoryEntry& e : listing_.entries) {
                const std::string text = e.directory ? "[" + std::string{tr(TextId::FileBrowserFolder)} + "] " + e.name : e.name;
                const bool selected = !e.directory && e.name == fileName_;
                if (ImGui::Selectable(text.c_str(), selected, ImGuiSelectableFlags_AllowDoubleClick)) {
                    if (e.directory) {
                        if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) enterDir = (fs::path{directory_} / e.name).string();
                    } else {
                        fileName_ = e.name;
                        if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) chosen = (fs::path{directory_} / e.name).string();
                    }
                }
                if (!e.directory) {
                    ImGui::SameLine(ImGui::GetContentRegionAvail().x * 0.8f + ImGui::GetCursorStartPos().x);
                    ImGui::TextDisabled("%s", humanSize(e.size).c_str());
                }
            }
            if (enterDir) enter(normalizeDirectory(*enterDir));
        }
        ImGui::EndChild();

        // 文件名、过滤、按钮
        char nameBuf[512];
        copyTo(nameBuf, sizeof nameBuf, fileName_);
        ImGui::TextUnformatted(tr(TextId::FileBrowserName));
        ImGui::SameLine();
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.5f);
        const bool enterPressed = ImGui::InputText("##name", nameBuf, sizeof nameBuf, ImGuiInputTextFlags_EnterReturnsTrue);
        fileName_ = nameBuf;
        ImGui::SameLine();
        if (ImGui::Checkbox(tr(TextId::FileBrowserOnlyScenes), &onlyFilter_)) refresh();
        if (ImGui::Button(tr(TextId::FileBrowserOpen)) || enterPressed) {
            if (!fileName_.empty()) {
                const fs::path p = fs::path{fileName_}.is_absolute() ? fs::path{fileName_} : fs::path{directory_} / fileName_;
                chosen = p.lexically_normal().string();
            }
        }
        ImGui::SameLine();
        if (ImGui::Button(tr(TextId::FileBrowserCancel)) || ImGui::IsKeyPressed(ImGuiKey_Escape)) keepOpen = false;
        if (chosen || !keepOpen) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    } else {
        keepOpen = false;
    }
    if (chosen || !keepOpen) open_ = false;
    return chosen;
}

}  // namespace terra::ui
