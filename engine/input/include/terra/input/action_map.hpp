#pragma once

// 动作映射定义（M1-04，D-018）。纯数据结构：M1 用代码注册，结构本身可直接序列化为配置文件（格式待定）。
//
// 概念：
//   动作（Action）      消费者读取的逻辑输入，如 camera.orbit；与设备无关。
//   绑定（Binding）     控件 → 动作；按设备“种类”匹配，任何已连接的同类设备都可驱动（热插拔后自动生效）。
//   处理器（Processor） 绑定输出依次经过的变换：死区、缩放、裁剪。
//   动作表（ActionMap） 一组动作，带优先级与启用状态；高优先级表的绑定可“消费”控件，屏蔽低优先级表。

#include <array>
#include <string>
#include <vector>

#include "terra/input/types.hpp"

namespace terra::input {

enum class ActionType {
    Button,   // 按下/抬起；值为 0/1（绑定到轴时按 |值| ≥ 阈值判定）
    Axis1D,   // x ∈ [-1, 1] 或扳机 [0, 1]
    Axis2D,   // (x, y)，如摇杆、WASD
    Delta2D,  // 本帧累计增量，如鼠标拖动；不限范围
};

// 同一动作有多个活动来源时（多个绑定或多台同类设备）如何选出一个。
enum class Arbitration {
    LargestMagnitude,  // 默认：取幅度最大的来源
    MostRecent,        // 取最近发生变化的来源
    SourcePriority,    // 取 Binding::sourcePriority 最高的来源；相同时取幅度最大
};

struct Processor {
    enum class Kind {
        AxialDeadzone,   // 每个分量：|v| < a → 0；(a, b) 线性映射到 (0, 1)；> b 饱和为 ±1
        RadialDeadzone,  // 按向量长度：|v| < a → 0；长度 (a, b) 映射到 (0, 1)，方向不变
        Scale,           // x *= a，y *= b（取负即反向）
        Clamp,           // 每个分量裁剪到 [a, b]
    };
    Kind kind = Kind::Scale;
    float a = 1.0f;
    float b = 1.0f;

    static Processor axialDeadzone(float inner, float outer = 1.0f) { return {Kind::AxialDeadzone, inner, outer}; }
    static Processor radialDeadzone(float inner, float outer = 1.0f) { return {Kind::RadialDeadzone, inner, outer}; }
    static Processor scale(float x, float y = 1.0f) { return {Kind::Scale, x, y}; }
    static Processor clamp(float lo, float hi) { return {Kind::Clamp, lo, hi}; }
};

struct Binding {
    enum class Kind {
        Single,       // 一个控件 → x
        Pair,         // 两个控件 → (x, y)，如摇杆两轴、鼠标 DeltaX/DeltaY
        Composite1D,  // controls[0] 负向、controls[1] 正向 → x ∈ {-1, 0, 1}
        Composite2D,  // controls[0..3] = 上、下、左、右 → (x, y)，各分量 ∈ {-1, 0, 1}
    };
    Kind kind = Kind::Single;
    std::array<Control, 4> controls{};  // 同一绑定的控件必须属于同一设备种类，取自同一台设备
    std::vector<Control> modifiers;     // 全部按下（任一已连接的同类设备上值 ≥ 0.5）时绑定才生效
    std::vector<Processor> processors;
    bool consume = true;                // 生效时屏蔽低优先级动作表中使用相同控件的绑定
    int sourcePriority = 0;             // 用于 Arbitration::SourcePriority，越大越优先

    static Binding single(Control c) { return make(Kind::Single, {c, {}, {}, {}}); }
    static Binding pair(Control x, Control y) { return make(Kind::Pair, {x, y, {}, {}}); }
    static Binding composite1D(Control negative, Control positive) {
        return make(Kind::Composite1D, {negative, positive, {}, {}});
    }
    static Binding composite2D(Control up, Control down, Control left, Control right) {
        return make(Kind::Composite2D, {up, down, left, right});
    }

    Binding withModifier(Control c) const { Binding b = *this; b.modifiers.push_back(c); return b; }
    Binding withProcessor(Processor p) const { Binding b = *this; b.processors.push_back(p); return b; }
    Binding withSourcePriority(int priority) const { Binding b = *this; b.sourcePriority = priority; return b; }
    Binding nonConsuming() const { Binding b = *this; b.consume = false; return b; }

    // 该种绑定使用的控件数。
    int controlCount() const noexcept {
        switch (kind) {
            case Kind::Single: return 1;
            case Kind::Pair: return 2;
            case Kind::Composite1D: return 2;
            case Kind::Composite2D: return 4;
        }
        return 0;
    }

private:
    static Binding make(Kind kind, std::array<Control, 4> controls) {
        Binding b;
        b.kind = kind;
        b.controls = controls;
        return b;
    }
};

struct ActionDefinition {
    std::string name;
    ActionType type = ActionType::Button;
    Arbitration arbitration = Arbitration::LargestMagnitude;
    float pressThreshold = 0.5f;  // Button：|值| ≥ 阈值视为按下
    std::vector<Binding> bindings;
};

struct ActionMapDefinition {
    std::string name;
    int priority = 0;  // 越大越先处理
    bool enabled = true;
    std::vector<ActionDefinition> actions;
};

}  // namespace terra::input
