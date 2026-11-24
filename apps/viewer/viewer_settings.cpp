#include "viewer_settings.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <tuple>

#include "imgui.h"
#include "terra/ui/text.hpp"
#include "terra/ui/ui_input.hpp"
#include "terra/ui/widgets.hpp"

namespace terra::viewer {

namespace {

namespace in = terra::input;
namespace st = terra::settings;
using terra::ui::TextId;
using terra::ui::tr;

constexpr const char* kCapturePopup = "###KeyCapture";

// 操作在界面上的名称，顺序与 settings::Command 一致。
constexpr TextId kCommandText[] = {
    TextId::CmdCameraForward, TextId::CmdCameraBack, TextId::CmdCameraLeft,  TextId::CmdCameraRight,  TextId::CmdCameraUp,
    TextId::CmdCameraDown,    TextId::CmdCameraBoost, TextId::CmdCameraReset, TextId::CmdAppFullscreen,
    TextId::CmdSimPlayPause,  TextId::CmdSimStep,     TextId::CmdSimReset,    TextId::CmdSimRecord,
};
static_assert(sizeof(kCommandText) / sizeof(kCommandText[0]) == st::kCommandCount, "kCommandText 必须与 Command 一一对应");

constexpr TextId kPageText[] = {TextId::SettingsInterface, TextId::SettingsDisplay, TextId::SettingsCamera, TextId::SettingsKeys};
static_assert(sizeof(kPageText) / sizeof(kPageText[0]) == static_cast<std::size_t>(SettingsWindow::PageCount),
              "kPageText 必须与页面一一对应");

const char* commandText(st::Command c) { return tr(kCommandText[static_cast<std::size_t>(c)]); }

// 标题随语言变化，“###” 之后的 ID 不变。
std::string label(TextId id, const char* stableId) { return std::string{tr(id)} + "###" + stableId; }

const char* modeText(st::WindowMode mode) {
    switch (mode) {
        case st::WindowMode::Windowed: return tr(TextId::DisplayWindowed);
        case st::WindowMode::BorderlessFullscreen: return tr(TextId::DisplayBorderless);
        case st::WindowMode::ExclusiveFullscreen: return tr(TextId::DisplayExclusive);
    }
    return "";
}

std::string videoModeText(const st::VideoModeSetting& m) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%d × %d @ %d Hz", m.width, m.height, m.refreshRate);
    return buf;
}

// 左侧标签，右侧控件，宽度占满。
void rowLabel(TextId id) {
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(tr(id));
    ImGui::SameLine(ImGui::GetContentRegionAvail().x * 0.42f);
    ImGui::SetNextItemWidth(-1.0f);
}

// 整行宽的属性滑块，名称显示在滑块内（用户 2026-11-19 反馈：默认滑块不够精致）。
bool sliderDouble(TextId id, const char* stableId, double& value, st::Range range, const char* format, bool logarithmic) {
    ImGui::SetNextItemWidth(-FLT_MIN);
    return terra::ui::propertySlider(stableId, tr(id), &value, range.min, range.max, format, logarithmic);
}

void interfacePage(SettingsWindow& w, st::Settings& s, float currentUiScale) {
    rowLabel(TextId::SettingsLanguage);
    const char* current = s.ui.language == "en" ? terra::ui::languageName(terra::ui::Language::English)
                                                : terra::ui::languageName(terra::ui::Language::Chinese);
    if (ImGui::BeginCombo("##language", current)) {
        using Choice = std::pair<const char*, terra::ui::Language>;
        for (const auto& [code, lang] : {Choice{"zh", terra::ui::Language::Chinese}, Choice{"en", terra::ui::Language::English}}) {
            if (ImGui::Selectable(terra::ui::languageName(lang), s.ui.language == code)) s.ui.language = code;
        }
        ImGui::EndCombo();
    }

    ImGui::Spacing();
    bool automatic = !s.ui.uiScale.has_value();
    if (ImGui::Checkbox(tr(TextId::SettingsUiScaleAuto), &automatic)) {
        if (automatic) {
            s.ui.uiScale.reset();
        } else {
            s.ui.uiScale = std::round(static_cast<double>(currentUiScale) * 100.0) / 100.0;
        }
    }
    ImGui::BeginDisabled(automatic);
    double percent = w.editingScale ? static_cast<double>(w.scaleEdit) : s.ui.uiScale.value_or(currentUiScale) * 100.0;
    ImGui::SetNextItemWidth(-FLT_MIN);
    terra::ui::propertySlider("uiScale", tr(TextId::SettingsUiScale), &percent, st::InterfaceSettings::kMinScale * 100.0,
                              st::InterfaceSettings::kMaxScale * 100.0, "%.0f%%");
    w.editingScale = ImGui::IsItemActive();
    w.scaleEdit = static_cast<float>(percent);
    if (ImGui::IsItemDeactivatedAfterEdit()) s.ui.uiScale = std::round(percent) / 100.0;
    ImGui::EndDisabled();
    ImGui::TextDisabled("Ctrl + / -  ·  Ctrl+0");

    ImGui::Spacing();
    if (ImGui::Button(tr(TextId::SettingsRestorePage))) {
        const st::InterfaceSettings defaults;
        s.ui.language = defaults.language;
        s.ui.uiScale = defaults.uiScale;
    }
}

void displayPage(st::Settings& s, const std::vector<terra::platform::MonitorInfo>& monitors) {
    rowLabel(TextId::SettingsDisplayMode);
    if (ImGui::BeginCombo("##mode", modeText(s.display.mode))) {
        for (const st::WindowMode m :
             {st::WindowMode::Windowed, st::WindowMode::BorderlessFullscreen, st::WindowMode::ExclusiveFullscreen}) {
            if (ImGui::Selectable(modeText(m), s.display.mode == m)) s.display.mode = m;
        }
        ImGui::EndCombo();
    }

    const auto monitorText = [&](std::size_t i) {
        return monitors[i].name + (monitors[i].primary ? tr(TextId::SettingsPrimary) : "");
    };
    // 不存在的显示器按主显示器（下标 0）处理，与应用设置时一致。
    const std::size_t selected =
        s.display.monitor >= 0 && static_cast<std::size_t>(s.display.monitor) < monitors.size() ? static_cast<std::size_t>(s.display.monitor) : 0;
    const bool fullscreen = s.display.mode != st::WindowMode::Windowed;
    ImGui::BeginDisabled(!fullscreen || monitors.empty());
    rowLabel(TextId::SettingsMonitor);
    if (ImGui::BeginCombo("##monitor", monitors.empty() ? "-" : monitorText(selected).c_str())) {
        for (std::size_t i = 0; i < monitors.size(); ++i) {
            ImGui::PushID(static_cast<int>(i));
            if (ImGui::Selectable(monitorText(i).c_str(), i == selected)) {
                s.display.monitor = static_cast<int>(i);
                s.display.videoMode.reset();  // 换显示器后原分辨率不一定可用
            }
            ImGui::PopID();
        }
        ImGui::EndCombo();
    }
    ImGui::EndDisabled();

    // 独占全屏的分辨率与刷新率：显示器支持的模式（去重，从大到小）。
    std::vector<st::VideoModeSetting> modes;
    if (!monitors.empty()) {
        for (const auto& m : monitors[selected].modes) modes.push_back({m.width, m.height, m.refreshRate});
        std::sort(modes.begin(), modes.end(), [](const st::VideoModeSetting& a, const st::VideoModeSetting& b) {
            return std::tie(b.width, b.height, b.refreshRate) < std::tie(a.width, a.height, a.refreshRate);
        });
        modes.erase(std::unique(modes.begin(), modes.end(), [](const st::VideoModeSetting& a, const st::VideoModeSetting& b) {
                        return a == b;
                    }),
                    modes.end());
    }
    ImGui::BeginDisabled(s.display.mode != st::WindowMode::ExclusiveFullscreen || modes.empty());
    rowLabel(TextId::SettingsVideoMode);
    const std::string preview = s.display.videoMode ? videoModeText(*s.display.videoMode) : tr(TextId::SettingsDesktopMode);
    if (ImGui::BeginCombo("##videoMode", preview.c_str(), ImGuiComboFlags_HeightLarge)) {
        if (ImGui::Selectable(tr(TextId::SettingsDesktopMode), !s.display.videoMode)) s.display.videoMode.reset();
        for (std::size_t i = 0; i < modes.size(); ++i) {
            ImGui::PushID(static_cast<int>(i));
            const bool isSelected = s.display.videoMode && *s.display.videoMode == modes[i];
            if (ImGui::Selectable(videoModeText(modes[i]).c_str(), isSelected)) s.display.videoMode = modes[i];
            ImGui::PopID();
        }
        ImGui::EndCombo();
    }
    ImGui::EndDisabled();

    ImGui::Spacing();
    ImGui::Checkbox(tr(TextId::SettingsVsync), &s.display.vsync);
    ImGui::Spacing();
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    ImGui::TextWrapped("%s", tr(TextId::SettingsExclusiveHint));
    ImGui::PopStyleColor();

    ImGui::Spacing();
    if (ImGui::Button(tr(TextId::SettingsRestorePage))) {
        const st::DisplaySettings defaults;
        s.display.mode = defaults.mode;
        s.display.monitor = defaults.monitor;
        s.display.videoMode = defaults.videoMode;
        s.display.vsync = defaults.vsync;
    }
}

void cameraPage(st::Settings& s) {
    namespace rg = st::ranges;
    sliderDouble(TextId::SettingsLook, "look", s.camera.lookDegreesPerPixel, rg::kLook, "%.3f", true);
    sliderDouble(TextId::SettingsSpeed, "speed", s.camera.speed, rg::kSpeed, "%.2f", true);
    sliderDouble(TextId::SettingsBoost, "boost", s.camera.boost, rg::kBoost, "%.1f×", false);
    sliderDouble(TextId::SettingsWheel, "wheel", s.camera.wheelStep, rg::kWheelStep, "%.2f", true);
    sliderDouble(TextId::SettingsPan, "pan", s.camera.panMetersPerPixel, rg::kPan, "%.4f", true);
    sliderDouble(TextId::SettingsFov, "fov", s.camera.fovDegrees, rg::kFov, "%.0f°", false);
    ImGui::TextDisabled("%s", tr(TextId::SettingsSliderHint));
    ImGui::Spacing();
    if (ImGui::Button(tr(TextId::SettingsRestorePage))) s.camera = st::CameraSettings{};
}

void keysPage(SettingsWindow& w, st::Settings& s) {
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    ImGui::TextWrapped("%s", tr(TextId::SettingsKeysHint));
    ImGui::PopStyleColor();
    const ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp;
    if (ImGui::BeginTable("keys", 3, flags)) {
        ImGui::TableSetupColumn(tr(TextId::SettingsAction), ImGuiTableColumnFlags_WidthStretch, 1.6f);
        ImGui::TableSetupColumn(tr(TextId::SettingsKey), ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn(tr(TextId::SettingsAltKey), ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableHeadersRow();
        for (std::size_t c = 0; c < st::kCommandCount; ++c) {
            const auto command = static_cast<st::Command>(c);
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::AlignTextToFramePadding();
            ImGui::TextUnformatted(commandText(command));
            for (std::size_t slot = 0; slot < 2; ++slot) {
                ImGui::TableSetColumnIndex(static_cast<int>(slot + 1));
                ImGui::PushID(static_cast<int>(c * 2 + slot));
                const in::Key key = s.keys[command][slot];
                const bool recording = w.capture == command && w.captureSlot == slot;
                const char* name = recording ? "…" : key == in::Key::Unknown ? tr(TextId::SettingsNoKey) : in::keyName(key);
                if (ImGui::Button(name, ImVec2(-1.0f, 0.0f))) {
                    w.capture = command;
                    w.captureSlot = slot;
                    w.captureFrames = 0;
                    w.captureMessage.clear();
                    w.openCapturePopup = true;
                }
                if (ImGui::IsItemClicked(ImGuiMouseButton_Right)) s.keys.clear(command, slot);
                ImGui::PopID();
            }
        }
        ImGui::EndTable();
    }
    ImGui::Spacing();
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    ImGui::TextWrapped("%s", tr(TextId::SettingsMouseHint));
    ImGui::PopStyleColor();
    ImGui::Spacing();
    if (ImGui::Button(tr(TextId::SettingsRestorePage))) s.keys = st::KeyMap{};
}

// 按键录制弹窗：第一帧不读按键（避免把打开弹窗的那次按键当成新键），之后第一个按下的键即为新键。
void captureModal(SettingsWindow& w, st::Settings& s, std::vector<std::string>& log) {
    if (w.openCapturePopup) {
        ImGui::OpenPopup(kCapturePopup);
        w.openCapturePopup = false;
    }
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    const std::string title = label(TextId::SettingsPressKey, "KeyCapture");
    if (!ImGui::BeginPopupModal(title.c_str(), nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove)) {
        if (w.capture && w.captureFrames > 0) w.capture.reset();  // 弹窗已被关闭
        return;
    }
    if (!w.capture) {
        ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
        return;
    }
    ImGui::TextUnformatted(commandText(*w.capture));
    ImGui::Spacing();
    ImGui::TextDisabled("%s", tr(TextId::SettingsPressKeyHint));
    if (!w.captureMessage.empty()) ImGui::TextColored(ImVec4(0.95f, 0.65f, 0.35f, 1.0f), "%s", w.captureMessage.c_str());

    if (w.captureFrames++ > 0) {
        for (int i = 1; i < static_cast<int>(in::Key::Count); ++i) {
            const auto key = static_cast<in::Key>(i);
            const ImGuiKey imguiKey = terra::ui::toImGuiKey(key);
            if (imguiKey == ImGuiKey_None || !ImGui::IsKeyPressed(imguiKey, false)) continue;
            if (key == in::Key::Escape) {
                w.capture.reset();
                break;
            }
            if (st::isReservedKey(key)) {
                w.captureMessage = std::string{in::keyName(key)} + "：" + tr(TextId::SettingsPressKeyHint);
                continue;
            }
            const auto movedFrom = s.keys.assign(*w.capture, w.captureSlot, key);
            if (movedFrom && *movedFrom != *w.capture) {
                log.push_back(std::string{in::keyName(key)} + tr(TextId::LogKeyMoved) + commandText(*movedFrom));
            }
            w.capture.reset();
            break;
        }
    }
    if (!w.capture) ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
}

}  // namespace

terra::view::CameraKeys toCameraKeys(const st::KeyMap& keys) {
    terra::view::CameraKeys out;
    out.forward = keys[st::Command::CameraForward];
    out.back = keys[st::Command::CameraBack];
    out.left = keys[st::Command::CameraLeft];
    out.right = keys[st::Command::CameraRight];
    out.up = keys[st::Command::CameraUp];
    out.down = keys[st::Command::CameraDown];
    out.boost = keys[st::Command::CameraBoost];
    out.reset = keys[st::Command::CameraReset];
    return out;
}

terra::view::CameraControlSettings toCameraControl(const st::CameraSettings& camera) {
    terra::view::CameraControlSettings out;
    out.lookDegreesPerPixel = camera.lookDegreesPerPixel;
    out.speed = camera.speed;
    out.boost = camera.boost;
    out.wheelStep = camera.wheelStep;
    out.panMetersPerPixel = camera.panMetersPerPixel;
    return out;
}

terra::platform::DisplayMode toDisplayMode(st::WindowMode mode) noexcept {
    switch (mode) {
        case st::WindowMode::Windowed: return terra::platform::DisplayMode::Windowed;
        case st::WindowMode::BorderlessFullscreen: return terra::platform::DisplayMode::BorderlessFullscreen;
        case st::WindowMode::ExclusiveFullscreen: return terra::platform::DisplayMode::ExclusiveFullscreen;
    }
    return terra::platform::DisplayMode::Windowed;
}

st::WindowMode toWindowMode(terra::platform::DisplayMode mode) noexcept {
    switch (mode) {
        case terra::platform::DisplayMode::Windowed: return st::WindowMode::Windowed;
        case terra::platform::DisplayMode::BorderlessFullscreen: return st::WindowMode::BorderlessFullscreen;
        case terra::platform::DisplayMode::ExclusiveFullscreen: return st::WindowMode::ExclusiveFullscreen;
    }
    return st::WindowMode::Windowed;
}

std::vector<in::Binding> buttonBindings(const st::KeySlots& slots) {
    std::vector<in::Binding> out;
    for (const in::Key k : slots) {
        if (k != in::Key::Unknown) out.push_back(in::Binding::single(in::key(k)));
    }
    return out;
}

void drawSettingsWindow(SettingsWindow& w, st::Settings& s, const std::vector<terra::platform::MonitorInfo>& monitors,
                        float currentUiScale, const std::string& path, std::vector<std::string>& log) {
    if (!w.open) {
        w.capture.reset();
        w.editingScale = false;
        return;
    }
    const float k = ImGui::GetFontSize() / 16.0f;  // 随界面缩放
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(760.0f * k, 520.0f * k), ImGuiCond_Appearing);
    const std::string title = label(TextId::SettingsTitle, "Preferences");
    if (ImGui::Begin(title.c_str(), &w.open, ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoCollapse)) {
        const float footer = ImGui::GetFrameHeightWithSpacing() * 2.0f + ImGui::GetStyle().ItemSpacing.y * 2.0f;
        // 左侧分类列表（参考 Blender 偏好设置）
        ImGui::BeginChild("pages", ImVec2(150.0f * k, -footer), ImGuiChildFlags_Borders);
        for (int i = 0; i < SettingsWindow::PageCount; ++i) {
            if (ImGui::Selectable(tr(kPageText[i]), w.page == i)) w.page = i;
        }
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::BeginChild("page", ImVec2(0.0f, -footer));
        ImGui::SeparatorText(tr(kPageText[std::clamp(w.page, 0, SettingsWindow::PageCount - 1)]));
        switch (w.page) {
            case SettingsWindow::Interface: interfacePage(w, s, currentUiScale); break;
            case SettingsWindow::Display: displayPage(s, monitors); break;
            case SettingsWindow::Camera: cameraPage(s); break;
            case SettingsWindow::Keys: keysPage(w, s); break;
            default: w.page = SettingsWindow::Interface; break;
        }
        ImGui::EndChild();

        ImGui::Separator();
        ImGui::TextDisabled("%s%s", tr(TextId::SettingsFile), path.empty() ? tr(TextId::SettingsNotSaved) : path.c_str());
        ImGui::TextDisabled("%s", path.empty() ? "" : tr(TextId::SettingsAutoSave));
        ImGui::SameLine();
        const float buttonWidth = ImGui::CalcTextSize(tr(TextId::SettingsRestoreAll)).x + ImGui::GetStyle().FramePadding.x * 2.0f;
        const float room = ImGui::GetContentRegionAvail().x;
        if (room > buttonWidth) ImGui::SetCursorPosX(ImGui::GetCursorPosX() + room - buttonWidth);  // 靠右
        if (ImGui::Button(tr(TextId::SettingsRestoreAll))) s = st::Settings{};
        captureModal(w, s, log);
    }
    ImGui::End();
    if (!w.open) w.capture.reset();
}

}  // namespace terra::viewer
