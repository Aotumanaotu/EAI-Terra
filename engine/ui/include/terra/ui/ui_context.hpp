#pragma once

// 界面上下文（M1-13，D-024、D-028）：Dear ImGui 上下文、字体与缩放、主题、OpenGL 渲染后端。
//
// 前提：一个 OpenGL 4.5 core 上下文已是当前上下文。ImGui 的 OpenGL3 渲染后端使用其自带的最小 GL 加载器。
// 每帧：UiContext::beginFrame（把 UiInputBridge 积压的事件交给 ImGui 并开始新帧）→ 构建界面 → render()。
// 同一进程只能有一个 UiContext（ImGui 当前上下文是全局状态）。

#include <memory>
#include <string>

#include "imgui.h"
#include "terra/platform/platform.hpp"
#include "terra/ui/ui_input.hpp"

namespace terra::ui {

struct UiOptions {
    std::string fontDirectory;     // 含 NotoSansSC-Regular-Subset.otf 与 -Bold-Subset.otf
    float contentScale = 1.0f;     // 显示器内容缩放（字号与控件尺寸都乘以它）
    float baseFontSize = 16.0f;    // 缩放前的正文字号，像素
    bool docking = true;
};

struct UiResult;

class UiContext {
public:
    static UiResult create(const UiOptions& options);
    ~UiContext();
    UiContext(const UiContext&) = delete;
    UiContext& operator=(const UiContext&) = delete;

    void beginFrame(UiInputBridge& input, double dt, terra::platform::Extent windowSize,
                    terra::platform::Extent framebufferSize);
    // 结束本帧并把界面绘制到当前绑定的帧缓冲（通常是窗口），视口由调用方设置。
    void render();

    bool wantsMouse() const;      // 鼠标在界面上（面板、菜单、弹窗）
    bool wantsTextInput() const;  // 输入框获得焦点
    ImFont* regularFont() const noexcept { return regular_; }
    ImFont* boldFont() const noexcept { return bold_; }
    // 字体文件缺失时回退到 ImGui 内置字体（只有英文），此时为真。
    bool usingFallbackFont() const noexcept { return fallback_; }
    float contentScale() const noexcept { return scale_; }

private:
    UiContext() = default;
    ImFont* regular_ = nullptr;
    ImFont* bold_ = nullptr;
    bool fallback_ = false;
    float scale_ = 1.0f;
    bool initialized_ = false;
};

struct UiResult {
    std::unique_ptr<UiContext> context;
    std::string error;        // 失败原因（context 为空时）
    std::string diagnostics;  // 非致命诊断，如字体回退
};

// Terra 深色主题：参考 Blender 与 UE5 的灰阶与蓝色强调色；按 scale 放大控件尺寸。
void applyTerraTheme(ImGuiStyle& style, float scale);

// 主题中用到的几个颜色，供应用与测试引用（线性 0–1）。
ImVec4 themeColor(ImGuiCol col);

}  // namespace terra::ui
