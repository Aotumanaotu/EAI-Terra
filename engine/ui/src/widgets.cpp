#include "terra/ui/widgets.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "imgui.h"
#include "imgui_internal.h"

namespace terra::ui {

bool propertySlider(const char* id, const char* name, double* value, double min, double max, const char* format,
                    bool logarithmic) {
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    if (window->SkipItems) return false;
    ImGuiContext& g = *GImGui;
    const ImGuiStyle& style = g.Style;
    const ImGuiID itemId = window->GetID(id);
    const float width = ImGui::CalcItemWidth();
    const float height = ImGui::GetFrameHeight() + 2.0f * std::max(1.0f, style.FramePadding.y * 0.35f);  // 比普通输入框略高
    const ImRect frame(window->DC.CursorPos, ImVec2(window->DC.CursorPos.x + width, window->DC.CursorPos.y + height));
    ImGui::ItemSize(frame, (height - ImGui::GetFontSize()) * 0.5f);
    if (!ImGui::ItemAdd(frame, itemId, &frame, ImGuiItemFlags_Inputable)) return false;

    ImGuiSliderFlags flags = ImGuiSliderFlags_AlwaysClamp;
    if (logarithmic) flags |= ImGuiSliderFlags_Logarithmic;

    // ---- 交互（与 ImGui::SliderScalar 相同）----
    const bool hovered = ImGui::ItemHoverable(frame, itemId, g.LastItemData.ItemFlags);
    bool typing = ImGui::TempInputIsActive(itemId);
    if (!typing) {
        const bool clicked = hovered && ImGui::IsMouseClicked(0, ImGuiInputFlags_None, itemId);
        const bool activate = clicked || g.NavActivateId == itemId;
        if (activate && clicked) ImGui::SetKeyOwner(ImGuiKey_MouseLeft, itemId);
        if (activate && ((clicked && g.IO.KeyCtrl) ||
                         (g.NavActivateId == itemId && (g.NavActivateFlags & ImGuiActivateFlags_PreferInput)))) {
            typing = true;
        }
        if (activate && !typing) {
            ImGui::SetActiveID(itemId, window);
            ImGui::SetFocusID(itemId, window);
            ImGui::FocusWindow(window);
            g.ActiveIdUsingNavDirMask |= (1u << ImGuiDir_Left) | (1u << ImGuiDir_Right);
        }
    }
    if (typing) {
        return ImGui::TempInputScalar(frame, itemId, name != nullptr ? name : "", ImGuiDataType_Double, value, format, &min, &max);
    }

    ImRect grab;
    const bool changed = ImGui::SliderBehavior(frame, itemId, ImGuiDataType_Double, value, &min, &max, format, flags, &grab);
    if (changed) ImGui::MarkItemEdited(itemId);

    // ---- 绘制 ----
    const bool active = g.ActiveId == itemId;
    ImDrawList* dl = window->DrawList;
    const float rounding = std::max(style.FrameRounding, height * 0.22f);
    const ImU32 background = ImGui::GetColorU32(active ? ImGuiCol_FrameBgActive : hovered ? ImGuiCol_FrameBgHovered : ImGuiCol_FrameBg);
    dl->AddRectFilled(frame.Min, frame.Max, background, rounding);
    // 已填充部分：从左端到滑块位置（ImGui 已按线性或对数刻度算好位置）；用裁剪保证左端圆角、右端平直
    if (grab.Max.x > grab.Min.x) {
        const float fillX = std::clamp((grab.Min.x + grab.Max.x) * 0.5f, frame.Min.x, frame.Max.x);
        if (fillX > frame.Min.x + 0.5f) {
            const ImVec4 accent = style.Colors[active ? ImGuiCol_SliderGrabActive : ImGuiCol_SliderGrab];
            const float alpha = active ? 0.95f : hovered ? 0.85f : 0.70f;
            dl->PushClipRect(frame.Min, ImVec2(fillX, frame.Max.y), true);
            dl->AddRectFilled(frame.Min, frame.Max, ImGui::GetColorU32(ImVec4(accent.x, accent.y, accent.z, alpha)), rounding);
            dl->PopClipRect();
            // 填充末端一条细亮线，便于看清当前位置
            if (fillX < frame.Max.x - 1.0f) {
                dl->AddLine(ImVec2(fillX, frame.Min.y + 2.0f), ImVec2(fillX, frame.Max.y - 2.0f),
                            ImGui::GetColorU32(ImVec4(1.0f, 1.0f, 1.0f, hovered || active ? 0.55f : 0.30f)), 1.0f);
            }
        }
    }
    dl->AddRect(frame.Min, frame.Max, ImGui::GetColorU32(ImVec4(1.0f, 1.0f, 1.0f, hovered || active ? 0.16f : 0.07f)), rounding);
    ImGui::RenderNavCursor(frame, itemId);

    // 文本：数值靠右（优先完整显示），名称靠左（放不下时裁剪）
    char text[64];
    ImGui::DataTypeFormatString(text, static_cast<int>(sizeof(text)), ImGuiDataType_Double, value, format);
    const float pad = style.FramePadding.x + 2.0f;
    const ImVec2 textSize = ImGui::CalcTextSize(text);
    const float valueX = frame.Max.x - pad - textSize.x;
    ImGui::RenderTextClipped(ImVec2(std::max(valueX, frame.Min.x + pad), frame.Min.y), ImVec2(frame.Max.x - pad, frame.Max.y),
                             text, nullptr, &textSize, ImVec2(1.0f, 0.5f));
    if (name != nullptr && name[0] != '\0') {
        const ImVec2 nameMax(valueX - pad, frame.Max.y);
        if (nameMax.x > frame.Min.x + pad) {
            ImGui::RenderTextClipped(ImVec2(frame.Min.x + pad, frame.Min.y), nameMax, name, nullptr, nullptr, ImVec2(0.0f, 0.5f));
        }
    }
    return changed;
}

double niceCeiling(double value) {
    if (!(value > 0.0) || !std::isfinite(value)) return 1.0;
    const double exponent = std::floor(std::log10(value));
    const double base = std::pow(10.0, exponent);
    for (const double step : {1.0, 2.0, 2.5, 5.0, 10.0}) {
        if (value <= step * base * (1.0 + 1e-9)) return step * base;
    }
    return 10.0 * base;
}

void lineChart(const char* id, const char* title, const ChartSeries* series, int count, double maxValue, const char* valueFormat,
               double referenceLine, float height) {
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    if (window->SkipItems) return;
    const ImGuiStyle& style = ImGui::GetStyle();
    const float width = ImGui::CalcItemWidth();
    const float h = height > 0.0f ? height : ImGui::GetTextLineHeight() * 4.0f + style.FramePadding.y * 4.0f;
    const ImRect frame(window->DC.CursorPos, ImVec2(window->DC.CursorPos.x + width, window->DC.CursorPos.y + h));
    ImGui::ItemSize(frame);
    if (!ImGui::ItemAdd(frame, window->GetID(id))) return;

    double top = maxValue;
    if (top <= 0.0) {
        double m = 0.0;
        for (int i = 0; i < count; ++i) {
            if (series[i].data != nullptr) m = std::max(m, series[i].data->maximum());
        }
        top = niceCeiling(std::max(m * 1.1, referenceLine > 0.0 ? referenceLine * 1.1 : 0.0));
    }
    ImDrawList* dl = window->DrawList;
    const float rounding = std::max(style.FrameRounding, 4.0f);
    dl->AddRectFilled(frame.Min, frame.Max, ImGui::GetColorU32(ImGuiCol_FrameBg), rounding);
    dl->AddRect(frame.Min, frame.Max, ImGui::GetColorU32(ImVec4(1.0f, 1.0f, 1.0f, 0.07f)), rounding);

    // 绘图区：留出标题行
    const float pad = style.FramePadding.x;
    const float titleH = ImGui::GetTextLineHeight() + style.FramePadding.y;
    const ImRect plot(ImVec2(frame.Min.x + pad, frame.Min.y + titleH + style.FramePadding.y),
                      ImVec2(frame.Max.x - pad, frame.Max.y - style.FramePadding.y));
    const auto yOf = [&](double v) {
        const double t = std::clamp(v / top, 0.0, 1.0);
        return plot.Max.y - static_cast<float>(t) * (plot.Max.y - plot.Min.y);
    };
    const ImU32 gridColor = ImGui::GetColorU32(ImVec4(1.0f, 1.0f, 1.0f, 0.06f));
    for (int k = 0; k <= 4; ++k) {
        const float y = plot.Min.y + (plot.Max.y - plot.Min.y) * static_cast<float>(k) / 4.0f;
        dl->AddLine(ImVec2(plot.Min.x, y), ImVec2(plot.Max.x, y), gridColor, 1.0f);
    }
    // 参考线（虚线）
    if (referenceLine > 0.0 && referenceLine <= top) {
        const float y = yOf(referenceLine);
        const ImU32 c = ImGui::GetColorU32(ImVec4(1.0f, 0.85f, 0.4f, 0.45f));
        for (float x = plot.Min.x; x < plot.Max.x; x += 8.0f) dl->AddLine(ImVec2(x, y), ImVec2(std::min(x + 4.0f, plot.Max.x), y), c, 1.0f);
    }
    // 序列：最新值在最右侧，横轴按容量等分（序列未填满时从右侧开始）
    dl->PushClipRect(plot.Min, plot.Max, true);
    std::vector<ImVec2> points;
    for (int i = 0; i < count; ++i) {
        const terra::core::TimeSeries* data = series[i].data;
        if (data == nullptr || data->size() < 2) continue;
        const std::size_t n = data->size();
        const float step = (plot.Max.x - plot.Min.x) / static_cast<float>(std::max<std::size_t>(data->capacity() - 1, 1));
        points.clear();
        for (std::size_t j = 0; j < n; ++j) {
            const float x = plot.Max.x - static_cast<float>(n - 1 - j) * step;
            points.emplace_back(x, yOf(data->at(j)));
        }
        dl->AddPolyline(points.data(), static_cast<int>(points.size()), series[i].color, 1.5f, ImDrawFlags_None);  // 1.92.8 起粗细在前
    }
    dl->PopClipRect();

    // 标题与上限（左上），图例与当前值（右上，从右往左排）
    char buf[64];
    std::snprintf(buf, sizeof(buf), valueFormat, top);
    const float textY = frame.Min.y + style.FramePadding.y * 0.5f;
    dl->AddText(ImVec2(frame.Min.x + pad, textY), ImGui::GetColorU32(ImGuiCol_Text), title);
    const float titleW = ImGui::CalcTextSize(title).x;
    dl->AddText(ImVec2(frame.Min.x + pad + titleW + pad, textY), ImGui::GetColorU32(ImGuiCol_TextDisabled), buf);
    float x = frame.Max.x - pad;
    for (int i = count - 1; i >= 0; --i) {
        if (series[i].data == nullptr) continue;
        char value[64];
        std::snprintf(value, sizeof(value), valueFormat, series[i].data->latest());
        const std::string label = std::string{series[i].name} + " " + (series[i].data->empty() ? "—" : value);
        const float w = ImGui::CalcTextSize(label.c_str()).x;
        x -= w;
        dl->AddText(ImVec2(x, textY), series[i].color, label.c_str());
        x -= pad * 1.5f;
    }
}

}  // namespace terra::ui
