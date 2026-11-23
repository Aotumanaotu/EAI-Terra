#pragma once

// Terra 界面控件（用户 2026-11-19 反馈：默认的滑块“不够高级，有点粗糙”）。
//
// propertySlider：Blender 风格的属性滑块——整条圆角框，左侧已填充部分用强调色表示当前值在范围中的位置，
// 名称在框内左侧、数值在框内右侧；悬停与拖动时整条提亮。交互沿用 ImGui 滑块（点击或拖动定位、对数刻度、
// Ctrl+点击直接输入、键盘导航），因此 IsItemDeactivatedAfterEdit 等查询照常可用。宽度取当前项宽度（SetNextItemWidth）。

#include "imgui.h"
#include "terra/core/time_series.hpp"

namespace terra::ui {

// id：在当前窗口内唯一（不显示）；name：框内左侧显示的名称，可为空；format：数值格式，如 "%.2f"、"%.0f°"。
// 返回本帧数值是否改变。值总是被限制在 [min, max] 内。
bool propertySlider(const char* id, const char* name, double* value, double min, double max, const char* format,
                    bool logarithmic = false);

// ---- 滚动折线图（M1-15，性能监视）----
// 圆角框内画若干条序列（共用纵轴 0 – 上限），最新值在最右侧、随时间向左滚动；左上角为标题，右上角为图例与当前值，
// 可选一条虚线参考线（如 16.7 ms）。maxValue ≤ 0 时按数据自动取“整”上限（1、2、2.5、5 × 10ⁿ）。
// 宽度取当前项宽度；height ≤ 0 时为 4 行字高。
struct ChartSeries {
    const terra::core::TimeSeries* data = nullptr;
    ImU32 color = 0;
    const char* name = "";
};

void lineChart(const char* id, const char* title, const ChartSeries* series, int count, double maxValue, const char* valueFormat,
               double referenceLine = -1.0, float height = 0.0f);
// 自动上限：不小于 value 的最小“整”数（1、2、2.5、5 × 10ⁿ）；value ≤ 0 时为 1。
double niceCeiling(double value);

}  // namespace terra::ui
