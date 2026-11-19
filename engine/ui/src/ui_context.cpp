#include "terra/ui/ui_context.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>

#include "imgui_impl_opengl3.h"

namespace terra::ui {

namespace {

ImVec4 rgb(int r, int g, int b, float a = 1.0f) {
    return ImVec4(static_cast<float>(r) / 255.0f, static_cast<float>(g) / 255.0f, static_cast<float>(b) / 255.0f, a);
}

// 颜色表：深灰阶 + 蓝色强调（Blender/UE5 风格）。
void setColors(ImVec4* c) {
    const ImVec4 accent = rgb(66, 135, 245);
    const ImVec4 accentDim = rgb(52, 98, 170);
    c[ImGuiCol_Text] = rgb(222, 222, 226);
    c[ImGuiCol_TextDisabled] = rgb(128, 128, 134);
    c[ImGuiCol_WindowBg] = rgb(40, 40, 43);
    c[ImGuiCol_ChildBg] = rgb(40, 40, 43);
    c[ImGuiCol_PopupBg] = rgb(32, 32, 35, 0.98f);
    c[ImGuiCol_Border] = rgb(22, 22, 24);
    c[ImGuiCol_BorderShadow] = rgb(0, 0, 0, 0.0f);
    c[ImGuiCol_FrameBg] = rgb(28, 28, 31);
    c[ImGuiCol_FrameBgHovered] = rgb(52, 52, 57);
    c[ImGuiCol_FrameBgActive] = rgb(60, 60, 66);
    c[ImGuiCol_TitleBg] = rgb(30, 30, 33);
    c[ImGuiCol_TitleBgActive] = rgb(30, 30, 33);
    c[ImGuiCol_TitleBgCollapsed] = rgb(30, 30, 33);
    c[ImGuiCol_MenuBarBg] = rgb(33, 33, 36);
    c[ImGuiCol_ScrollbarBg] = rgb(30, 30, 33);
    c[ImGuiCol_ScrollbarGrab] = rgb(70, 70, 76);
    c[ImGuiCol_ScrollbarGrabHovered] = rgb(90, 90, 98);
    c[ImGuiCol_ScrollbarGrabActive] = accentDim;
    c[ImGuiCol_CheckMark] = accent;
    c[ImGuiCol_SliderGrab] = accentDim;
    c[ImGuiCol_SliderGrabActive] = accent;
    c[ImGuiCol_Button] = rgb(58, 58, 63);
    c[ImGuiCol_ButtonHovered] = rgb(74, 74, 80);
    c[ImGuiCol_ButtonActive] = accentDim;
    c[ImGuiCol_Header] = rgb(58, 58, 63);
    c[ImGuiCol_HeaderHovered] = rgb(70, 70, 76);
    c[ImGuiCol_HeaderActive] = accentDim;
    c[ImGuiCol_Separator] = rgb(24, 24, 26);
    c[ImGuiCol_SeparatorHovered] = accentDim;
    c[ImGuiCol_SeparatorActive] = accent;
    c[ImGuiCol_ResizeGrip] = rgb(0, 0, 0, 0.0f);
    c[ImGuiCol_ResizeGripHovered] = accentDim;
    c[ImGuiCol_ResizeGripActive] = accent;
    c[ImGuiCol_Tab] = rgb(33, 33, 36);
    c[ImGuiCol_TabHovered] = rgb(60, 60, 66);
    c[ImGuiCol_TabSelected] = rgb(40, 40, 43);
    c[ImGuiCol_TabSelectedOverline] = accent;
    c[ImGuiCol_TabDimmed] = rgb(30, 30, 33);
    c[ImGuiCol_TabDimmedSelected] = rgb(40, 40, 43);
    c[ImGuiCol_TabDimmedSelectedOverline] = rgb(80, 80, 86);
    c[ImGuiCol_DockingPreview] = ImVec4(accent.x, accent.y, accent.z, 0.6f);
    c[ImGuiCol_DockingEmptyBg] = rgb(22, 22, 24);
    c[ImGuiCol_TextSelectedBg] = ImVec4(accent.x, accent.y, accent.z, 0.35f);
    c[ImGuiCol_NavCursor] = accent;
    c[ImGuiCol_ModalWindowDimBg] = rgb(0, 0, 0, 0.5f);
    c[ImGuiCol_TableHeaderBg] = rgb(48, 48, 52);
    c[ImGuiCol_TableBorderStrong] = rgb(24, 24, 26);
    c[ImGuiCol_TableBorderLight] = rgb(32, 32, 35);
    c[ImGuiCol_TableRowBg] = rgb(0, 0, 0, 0.0f);
    c[ImGuiCol_TableRowBgAlt] = rgb(255, 255, 255, 0.03f);
}

}  // namespace

void applyTerraTheme(ImGuiStyle& style, float scale) {
    style = ImGuiStyle();
    setColors(style.Colors);
    style.WindowRounding = 3.0f;
    style.ChildRounding = 3.0f;
    style.FrameRounding = 3.0f;
    style.PopupRounding = 3.0f;
    style.ScrollbarRounding = 3.0f;
    style.GrabRounding = 3.0f;
    style.TabRounding = 3.0f;
    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 0.0f;
    style.PopupBorderSize = 1.0f;
    style.WindowPadding = ImVec2(8.0f, 8.0f);
    style.FramePadding = ImVec2(6.0f, 4.0f);
    style.ItemSpacing = ImVec2(8.0f, 6.0f);
    style.ItemInnerSpacing = ImVec2(6.0f, 4.0f);
    style.IndentSpacing = 16.0f;
    style.ScrollbarSize = 12.0f;
    style.WindowMenuButtonPosition = ImGuiDir_None;  // 停靠标签页不显示下拉按钮，更接近 Blender/UE5
    style.ScaleAllSizes(scale);
}

ImVec4 themeColor(ImGuiCol col) {
    static ImVec4 colors[ImGuiCol_COUNT];
    static bool ready = false;
    if (!ready) {
        setColors(colors);
        ready = true;
    }
    return col >= 0 && col < ImGuiCol_COUNT ? colors[col] : ImVec4(0, 0, 0, 1);
}

UiResult UiContext::create(const UiOptions& options) {
    UiResult result;
    if (ImGui::GetCurrentContext() != nullptr) {
        result.error = "同一进程只能有一个界面上下文";
        return result;
    }
    std::unique_ptr<UiContext> self{new UiContext()};
    self->scale_ = std::clamp(options.contentScale > 0.0f ? options.contentScale : 1.0f, kMinScale, kMaxScale);
    self->baseFontSize_ = options.baseFontSize;

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;  // 不写 imgui.ini：停靠布局由应用通过 SaveIniSettingsToMemory 存进设置文件（M1-14，D-026）
    io.LogFilename = nullptr;
    io.BackendPlatformName = "terra_platform（Terra 输入桥）";
    if (options.docking) io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigWindowsMoveFromTitleBarOnly = true;  // 只能拖动标题栏移动窗口，在视口等内容区域点击不会拖走面板

    // 字体：正文与标题。按缩放后的像素大小加载，ImGui 1.92+ 按需光栅化字形（中文无需预先指定字符范围）。
    const std::filesystem::path dir{options.fontDirectory};
    const std::filesystem::path regularPath = dir / "NotoSansSC-Regular-Subset.otf";
    const std::filesystem::path boldPath = dir / "NotoSansSC-Bold-Subset.otf";
    std::error_code ec;
    if (std::filesystem::is_regular_file(regularPath, ec) && std::filesystem::is_regular_file(boldPath, ec)) {
        self->regular_ = io.Fonts->AddFontFromFileTTF(regularPath.string().c_str(), options.baseFontSize);
        self->bold_ = io.Fonts->AddFontFromFileTTF(boldPath.string().c_str(), options.baseFontSize);
    }
    if (self->regular_ == nullptr || self->bold_ == nullptr) {
        io.Fonts->Clear();
        self->regular_ = io.Fonts->AddFontDefault();
        self->bold_ = self->regular_;
        self->fallback_ = true;
        result.diagnostics = "找不到界面字体 " + regularPath.string() + "，回退到 ImGui 内置字体（不含中文）";
    }
    io.FontDefault = self->regular_;

    ImGuiStyle& style = ImGui::GetStyle();
    applyTerraTheme(style, self->scale_);
    style.FontSizeBase = options.baseFontSize;
    style.FontScaleDpi = self->scale_;

    if (!ImGui_ImplOpenGL3_Init("#version 450 core")) {
        ImGui::DestroyContext();
        result.error = "ImGui OpenGL3 渲染后端初始化失败";
        return result;
    }
    self->initialized_ = true;
    result.context = std::move(self);
    return result;
}

UiContext::~UiContext() {
    if (initialized_) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui::DestroyContext();
    }
}

void UiContext::beginFrame(UiInputBridge& input, double dt, terra::platform::Extent windowSize,
                           terra::platform::Extent framebufferSize) {
    ImGuiIO& io = ImGui::GetIO();
    if (pendingScale_ > 0.0f) {
        ImGuiStyle& style = ImGui::GetStyle();
        applyTerraTheme(style, pendingScale_);
        style.FontSizeBase = baseFontSize_;
        style.FontScaleDpi = pendingScale_;
        scale_ = pendingScale_;
        pendingScale_ = 0.0f;
    }
    io.DisplaySize = ImVec2(static_cast<float>(windowSize.width), static_cast<float>(windowSize.height));
    if (windowSize.width > 0 && windowSize.height > 0) {
        io.DisplayFramebufferScale = ImVec2(static_cast<float>(framebufferSize.width) / static_cast<float>(windowSize.width),
                                            static_cast<float>(framebufferSize.height) / static_cast<float>(windowSize.height));
    }
    io.DeltaTime = dt > 0.0 ? static_cast<float>(dt) : 1.0f / 60.0f;
    input.flushTo(io);
    ImGui_ImplOpenGL3_NewFrame();
    ImGui::NewFrame();
}

void UiContext::render() {
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void UiContext::requestScale(float value) {
    if (!std::isfinite(value)) return;
    pendingScale_ = std::clamp(value, kMinScale, kMaxScale);
}

float UiContext::autoScale(float systemScale, int monitorHeight) {
    const float base = systemScale > 0.0f ? systemScale : 1.0f;
    const float boost = std::max(1.0f, 0.7f * static_cast<float>(monitorHeight) / 1080.0f);
    const float s = std::round(base * boost * 20.0f) / 20.0f;
    return std::clamp(s, kMinScale, kMaxScale);
}

bool UiContext::wantsMouse() const { return ImGui::GetIO().WantCaptureMouse; }

bool UiContext::wantsTextInput() const { return ImGui::GetIO().WantTextInput; }

}  // namespace terra::ui
