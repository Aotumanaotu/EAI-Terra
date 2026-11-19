// terra_viewer：EAI Terra 的第一个应用——编辑器式界面外壳（M1-13，D-024、D-025、D-028、D-029）。
//
// 布局参考 Blender / UE5：顶部菜单栏；中间视口（场景渲染到离屏帧缓冲后作为图像显示）；左侧“大纲”、右侧“属性”、
// 底部“日志 / 统计”面板，均可拖动停靠；最下方状态栏。F11 全屏；菜单“设置 → 语言”切换中英文。
// 输入：平台 → UiInputBridge（分流）→ InputSystem 与 ImGui；鼠标在面板上时界面占用输入，在视口里时相机控制器工作。
//
// 用法：terra_viewer [--seconds N] [--frames N] [--language zh|en] [--self-test] [--expect-renderer 子串]
//   --self-test  运行 --frames 帧后检查：字体未回退、中文字形可用、无 GL 错误、视口与窗口像素、语言切换；失败返回 1

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "imgui.h"
#include "imgui_internal.h"
#include "terra/input/input_system.hpp"
#include "terra/platform/glfw/glfw_platform.hpp"
#include "terra/render/context.hpp"
#include "terra/render/resources.hpp"
#include "terra/ui/text.hpp"
#include "terra/ui/ui_context.hpp"
#include "terra/ui/ui_input.hpp"
#include "terra/view/camera.hpp"
#include "terra/view/camera_controller.hpp"

#ifndef TERRA_ASSETS_DIR
#error "需要 TERRA_ASSETS_DIR"
#endif

namespace {

namespace in = terra::input;
namespace r = terra::render;
namespace ui = terra::ui;
namespace v = terra::view;
using ui::TextId;
using ui::tr;

struct Options {
    double seconds = 0.0;  // 0：不限时
    int frames = 0;        // 0：不限帧
    bool selfTest = false;
    std::string expectRenderer;
    ui::Language language = ui::Language::Chinese;
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
        } else if (a == "--self-test") {
            o.selfTest = true;
        } else if (a == "--expect-renderer" && hasValue) {
            o.expectRenderer = argv[++i];
        } else {
            return false;
        }
    }
    return true;
}

// ---- 视口中的预览场景：地面网格、坐标轴、彩色立方体（M1-08 起由正式的网格与材质替代）----

struct Vertex {
    float position[3];
    float color[3];
};

class ScenePreview {
public:
    bool create(std::string& error) {
        const char* vs = R"(#version 450 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aColor;
uniform mat4 uViewProjection;
out vec3 vColor;
void main() { vColor = aColor; gl_Position = uViewProjection * vec4(aPosition, 1.0); }
)";
        const char* fs = R"(#version 450 core
in vec3 vColor;
out vec4 fragColor;
void main() { fragColor = vec4(vColor, 1.0); }
)";
        auto shader = r::ShaderProgram::create({{r::ShaderStage::Vertex, "preview.vert", vs},
                                                {r::ShaderStage::Fragment, "preview.frag", fs}});
        if (!shader.program) {
            error = shader.error ? shader.error->summary() : std::string{"着色器创建失败"};
            return false;
        }
        program_ = std::move(*shader.program);
        location_ = program_.uniformLocation("uViewProjection");
        const std::vector<Vertex> verts = build();
        buffer_ = r::Buffer::create(verts.size() * sizeof(Vertex), verts.data());
        vao_ = r::VertexArray::create();
        vao_.setVertexBuffer(0, buffer_, 0, sizeof(Vertex));
        vao_.setFloatAttribute(0, 0, 3, offsetof(Vertex, position));
        vao_.setFloatAttribute(1, 0, 3, offsetof(Vertex, color));
        return true;
    }

    void draw(r::RenderContext& ctx, const v::Camera& camera, double aspect) {
        const v::Matrix4 vp = multiply(camera.projectionMatrix(aspect), camera.viewMatrix());
        program_.use();
        program_.setUniformMatrix4(location_, vp.data());
        vao_.bind();
        ctx.drawLines(0, gridCount_);
        ctx.drawLines(axesFirst_, 6);
        ctx.drawTriangles(cubeFirst_, 36);
    }

private:
    static v::Matrix4 multiply(const v::Matrix4& a, const v::Matrix4& b) {
        v::Matrix4 out{};
        for (std::size_t c = 0; c < 4; ++c) {
            for (std::size_t row = 0; row < 4; ++row) {
                float s = 0.0f;
                for (std::size_t k = 0; k < 4; ++k) s += a[k * 4 + row] * b[c * 4 + k];
                out[c * 4 + row] = s;
            }
        }
        return out;
    }

    std::vector<Vertex> build() {
        std::vector<Vertex> verts;
        for (int i = -10; i <= 10; ++i) {
            if (i == 0) continue;
            const float c = i % 5 == 0 ? 0.36f : 0.24f;
            const auto fi = static_cast<float>(i);
            verts.push_back({{fi, -10, 0}, {c, c, c}});
            verts.push_back({{fi, 10, 0}, {c, c, c}});
            verts.push_back({{-10, fi, 0}, {c, c, c}});
            verts.push_back({{10, fi, 0}, {c, c, c}});
        }
        verts.push_back({{-10, 0, 0}, {0.30f, 0.30f, 0.30f}});
        verts.push_back({{0, 0, 0}, {0.30f, 0.30f, 0.30f}});
        verts.push_back({{0, -10, 0}, {0.30f, 0.30f, 0.30f}});
        verts.push_back({{0, 0, 0}, {0.30f, 0.30f, 0.30f}});
        gridCount_ = static_cast<int>(verts.size());
        axesFirst_ = gridCount_;
        verts.push_back({{0, 0, 0.001f}, {0.90f, 0.25f, 0.25f}});
        verts.push_back({{10, 0, 0.001f}, {0.90f, 0.25f, 0.25f}});
        verts.push_back({{0, 0, 0.001f}, {0.45f, 0.80f, 0.30f}});
        verts.push_back({{0, 10, 0.001f}, {0.45f, 0.80f, 0.30f}});
        verts.push_back({{0, 0, 0}, {0.30f, 0.50f, 0.95f}});
        verts.push_back({{0, 0, 2}, {0.30f, 0.50f, 0.95f}});
        cubeFirst_ = static_cast<int>(verts.size());
        const float h = 0.5f;
        const float corners[8][3] = {{-h, -h, 0}, {h, -h, 0}, {h, h, 0}, {-h, h, 0},
                                     {-h, -h, 1}, {h, -h, 1}, {h, h, 1}, {-h, h, 1}};
        const int faces[6][4] = {{0, 1, 2, 3}, {4, 5, 6, 7}, {0, 1, 5, 4}, {2, 3, 7, 6}, {1, 2, 6, 5}, {3, 0, 4, 7}};
        const float colors[6][3] = {{0.35f, 0.35f, 0.40f}, {0.85f, 0.85f, 0.88f}, {0.80f, 0.45f, 0.30f},
                                    {0.30f, 0.60f, 0.85f}, {0.85f, 0.70f, 0.30f}, {0.45f, 0.75f, 0.50f}};
        for (int f = 0; f < 6; ++f) {
            const int* q = faces[f];
            const float* col = colors[f];
            for (const int idx : {q[0], q[1], q[2], q[0], q[2], q[3]}) {
                const float* p = corners[idx];
                verts.push_back({{p[0], p[1], p[2]}, {col[0], col[1], col[2]}});
            }
        }
        return verts;
    }

    r::ShaderProgram program_;
    r::Buffer buffer_;
    r::VertexArray vao_;
    int location_ = -1;
    int gridCount_ = 0;
    int axesFirst_ = 0;
    int cubeFirst_ = 0;
};

// ---- 应用状态 ----

struct AppState {
    bool showOutliner = true;
    bool showDetails = true;
    bool showLog = true;
    bool showStats = true;
    bool showControls = false;
    bool showAbout = false;
    bool layoutBuilt = false;
    bool quit = false;
    bool viewportHovered = false;
    ImVec2 viewportImageMin{0, 0};  // 视口图像在窗口中的位置（屏幕坐标），供自检
    ImVec2 viewportImageMax{0, 0};
    std::vector<std::string> log;
    double fps = 0.0;
    double frameMs = 0.0;
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
    ImGui::DockBuilderDockWindow("###Log", bottom);
    ImGui::DockBuilderDockWindow("###Stats", bottom);
    ImGui::DockBuilderDockWindow("###Viewport", center);
    ImGui::DockBuilderFinish(dockspace);
}

struct Frame {
    terra::platform::Window& window;
    r::RenderContext& ctx;
    v::Camera& camera;
    v::CameraController& controller;
    ScenePreview& scene;
    std::optional<r::Framebuffer>& viewportFb;
    AppState& state;
};

void menuBar(Frame& f) {
    if (!ImGui::BeginMainMenuBar()) return;
    if (ImGui::BeginMenu(tr(TextId::MenuFile))) {
        if (ImGui::MenuItem(tr(TextId::MenuFileQuit), "Alt+F4")) f.state.quit = true;
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu(tr(TextId::MenuView))) {
        const bool full = f.window.displayMode() != terra::platform::DisplayMode::Windowed;
        if (ImGui::MenuItem(tr(TextId::MenuViewFullscreen), "F11", full)) {
            (void)f.window.toggleBorderlessFullscreen();
            f.state.log.push_back(std::string{tr(TextId::LogDisplayModeChanged)} + displayModeText(f.window.displayMode()));
        }
        if (ImGui::MenuItem(tr(TextId::MenuViewResetCamera), "F")) v::CameraController::reset(f.camera);
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu(tr(TextId::MenuWindow))) {
        ImGui::MenuItem(tr(TextId::PanelOutliner), nullptr, &f.state.showOutliner);
        ImGui::MenuItem(tr(TextId::PanelDetails), nullptr, &f.state.showDetails);
        ImGui::MenuItem(tr(TextId::PanelLog), nullptr, &f.state.showLog);
        ImGui::MenuItem(tr(TextId::PanelStats), nullptr, &f.state.showStats);
        ImGui::EndMenu();
    }
    if (ImGui::BeginMenu(tr(TextId::MenuSettings))) {
        if (ImGui::BeginMenu(tr(TextId::MenuSettingsLanguage))) {
            for (const ui::Language lang : {ui::Language::Chinese, ui::Language::English}) {
                if (ImGui::MenuItem(ui::languageName(lang), nullptr, ui::language() == lang) && ui::language() != lang) {
                    ui::setLanguage(lang);
                    f.window.setTitle(tr(TextId::AppName));
                    f.state.log.push_back(tr(TextId::LogLanguageChanged));
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
            ImGui::EndMenuBar();
        }
    }
    ImGui::End();
}

void viewportPanel(Frame& f) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    const bool open = ImGui::Begin(label(TextId::PanelViewport, "Viewport").c_str(), nullptr,
                                   ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::PopStyleVar();
    f.state.viewportHovered = false;
    if (open) {
        const ImVec2 avail = ImGui::GetContentRegionAvail();
        const ImVec2 scale = ImGui::GetIO().DisplayFramebufferScale;
        const int w = std::max(1, static_cast<int>(avail.x * scale.x));
        const int h = std::max(1, static_cast<int>(avail.y * scale.y));
        if (!f.viewportFb || f.viewportFb->width() != w || f.viewportFb->height() != h) {
            auto made = r::Framebuffer::create(w, h, true);
            if (made.framebuffer) f.viewportFb = std::move(made.framebuffer);
        }
        if (f.viewportFb && avail.x >= 1.0f && avail.y >= 1.0f) {
            f.viewportFb->bind();
            f.ctx.setDepthTest(true);
            f.ctx.clear({0.11f, 0.11f, 0.12f, 1.0f});
            f.scene.draw(f.ctx, f.camera, static_cast<double>(w) / static_cast<double>(h));
            r::Framebuffer::bindDefault();
            ImGui::Image(static_cast<ImTextureID>(f.viewportFb->colorTexture()), avail, ImVec2(0, 1), ImVec2(1, 0));
            f.state.viewportHovered = ImGui::IsItemHovered();
            f.state.viewportImageMin = ImGui::GetItemRectMin();
            f.state.viewportImageMax = ImGui::GetItemRectMax();
        }
    }
    ImGui::End();
}

void outlinerPanel(Frame& f) {
    if (!f.state.showOutliner) return;
    if (ImGui::Begin(label(TextId::PanelOutliner, "Outliner").c_str(), &f.state.showOutliner)) {
        ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        if (ImGui::TreeNode(tr(TextId::OutlinerScene))) {
            for (const TextId item : {TextId::OutlinerGround, TextId::OutlinerAxes, TextId::OutlinerCube}) {
                ImGui::BulletText("%s", tr(item));
            }
            ImGui::TreePop();
        }
        ImGui::Spacing();
        ImGui::TextDisabled("%s", tr(TextId::OutlinerHint));
    }
    ImGui::End();
}

void detailsPanel(Frame& f) {
    if (!f.state.showDetails) return;
    if (ImGui::Begin(label(TextId::PanelDetails, "Details").c_str(), &f.state.showDetails)) {
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
        ImGui::Spacing();
        ImGui::TextDisabled("%s", tr(TextId::DetailsHint));
    }
    ImGui::End();
}

void logPanel(Frame& f) {
    if (!f.state.showLog) return;
    if (ImGui::Begin(label(TextId::PanelLog, "Log").c_str(), &f.state.showLog)) {
        for (const auto& line : f.state.log) ImGui::TextUnformatted(line.c_str());
        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) ImGui::SetScrollHereY(1.0f);
    }
    ImGui::End();
}

void statsPanel(Frame& f) {
    if (!f.state.showStats) return;
    if (ImGui::Begin(label(TextId::PanelStats, "Stats").c_str(), &f.state.showStats)) {
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
            if (f.viewportFb) {
                std::snprintf(buf, sizeof(buf), "%d × %d", f.viewportFb->width(), f.viewportFb->height());
                row(TextId::StatsViewportSize, buf);
            }
            std::snprintf(buf, sizeof(buf), "%.2f", static_cast<double>(f.window.contentScale()));
            row(TextId::StatsContentScale, buf);
            ImGui::EndTable();
        }
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
    menuBar(f);
    statusBar(f);
    const ImGuiID dockspace = ImGui::DockSpaceOverViewport(0, nullptr, ImGuiDockNodeFlags_None);
    if (!f.state.layoutBuilt) {
        buildDefaultLayout(dockspace);
        f.state.layoutBuilt = true;
    }
    viewportPanel(f);
    outlinerPanel(f);
    detailsPanel(f);
    logPanel(f);
    statsPanel(f);
    helpWindows(f);
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

    // 视口离屏帧缓冲：画面中心是立方体朝向相机的面（默认视角下为绿色面 (115, 191, 128)）
    const Rgb cubeFace{115, 191, 128};
    if (f.viewportFb) {
        const Rgb center = pixelAt(f.viewportFb->readPixels(f.viewportFb->width() / 2, f.viewportFb->height() / 2, 1, 1));
        std::printf("INFO 视口 %d×%d，中心像素 %s\n", f.viewportFb->width(), f.viewportFb->height(), str(center).c_str());
        check(near(center, cubeFace, 3), "视口中心为立方体正面颜色");
    } else {
        check(false, "视口帧缓冲已创建");
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
    return failures == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parseOptions(argc, argv, options)) {
        std::fprintf(stderr,
                     "用法：terra_viewer [--seconds N] [--frames N] [--language zh|en] [--self-test] [--expect-renderer 子串]\n");
        return 2;
    }
    ui::setLanguage(options.language);

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
    desc.width = options.selfTest ? 1200 : 1600;
    desc.height = options.selfTest ? 700 : 900;
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

    ui::UiOptions uiOptions;
    uiOptions.fontDirectory = std::string{TERRA_ASSETS_DIR} + "/fonts";
    uiOptions.contentScale = window.contentScale();
    auto uiMade = ui::UiContext::create(uiOptions);
    if (!uiMade.context) {
        std::fprintf(stderr, "无法初始化界面：%s\n", uiMade.error.c_str());
        return 1;
    }
    if (!uiMade.diagnostics.empty()) std::fprintf(stderr, "%s\n", uiMade.diagnostics.c_str());
    auto& uiContext = *uiMade.context;

    ui::UiInputCapture capture(input);
    v::CameraController controller(input);
    v::Camera camera;
    v::CameraController::reset(camera);

    in::ActionMapDefinition app{"app", 0, true, {}};
    in::ActionDefinition fullscreen;
    fullscreen.name = "fullscreen";
    fullscreen.type = in::ActionType::Button;
    fullscreen.bindings = {in::Binding::single(in::key(in::Key::F11))};
    app.actions.push_back(fullscreen);
    (void)input.addActionMap(app);
    const in::ActionHandle fullscreenAction = input.findAction("app", "fullscreen");

    ScenePreview scene;
    std::string sceneError;
    if (!scene.create(sceneError)) {
        std::fprintf(stderr, "无法创建预览场景：%s\n", sceneError.c_str());
        return 1;
    }

    std::optional<r::Framebuffer> viewportFb;
    AppState state;
    state.log.push_back(tr(TextId::LogStarted));
    state.log.push_back(std::string{tr(TextId::StatsRenderer)} + "：" + ctx.info().renderer);
    Frame frame{window, ctx, camera, controller, scene, viewportFb, state};

    std::printf("INFO %s，显示缩放 %.2f，字体%s\n", ctx.info().renderer.c_str(), static_cast<double>(window.contentScale()),
                uiContext.usingFallbackFont() ? "回退到内置（无中文）" : " Noto Sans SC 子集");

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

        uiContext.beginFrame(bridge, dt, window.windowSize(), window.framebufferSize());
        // 鼠标在面板上（且不在视口里）时界面占用鼠标；输入框获得焦点时界面占用键盘。
        capture.update(uiContext.wantsMouse() && !state.viewportHovered, uiContext.wantsTextInput());
        input.update();
        if (input.state(fullscreenAction).pressed) {
            (void)window.toggleBorderlessFullscreen();
            state.log.push_back(std::string{tr(TextId::LogDisplayModeChanged)} + displayModeText(window.displayMode()));
        }
        controller.update(camera, dt);

        if (options.selfTest && options.frames > 0 && frames == options.frames / 2) {
            ui::setLanguage(ui::Language::English);
            window.setTitle(tr(TextId::AppName));
        }
        buildUi(frame);

        r::Framebuffer::bindDefault();
        const auto fbSize = window.framebufferSize();
        ctx.setViewport(0, 0, fbSize.width, fbSize.height);
        ctx.setDepthTest(false);
        const ImVec4 bg = ui::themeColor(ImGuiCol_DockingEmptyBg);
        ctx.clear({bg.x, bg.y, bg.z, 1.0f});
        uiContext.render();

        ++frames;
        const bool lastFrame = (options.frames > 0 && frames >= options.frames) ||
                               (options.seconds > 0.0 && now - start >= options.seconds);
        if (lastFrame && options.selfTest) result = selfTest(frame, uiContext, options);
        window.swapBuffers();
        if (lastFrame) break;
    }
    std::printf("INFO 结束：%d 帧，%.1f 秒\n", frames, platform.time() - start);
    return result;
}
