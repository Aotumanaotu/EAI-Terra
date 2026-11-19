#pragma once

// 输入写入协议（M1-04，D-018）。适配器只看到 InputSink，看不到动作映射与消费者。
// 线程：M1 单线程，所有调用都在主线程，且与 InputSystem::update 交替进行。

#include "terra/input/types.hpp"

namespace terra::input {

class InputSink {
public:
    virtual ~InputSink() = default;

    // 设备连接；返回新的连接实例 ID。
    virtual DeviceId connectDevice(const DeviceDescription& description) = 0;

    // 设备断开；该设备所有控件视为归零，按下中的按钮在下一次 update 产生“抬起”。未知 ID 忽略。
    virtual void disconnectDevice(DeviceId id) = 0;

    // 把设备所有控件置零（例如窗口失去焦点），与断开时的释放语义相同，但设备保持连接。
    virtual void resetDevice(DeviceId id) = 0;

    // 设置控件的绝对值（按键、摇杆、指针位置）。timestamp 为单调时钟秒数，用于“最近来源”仲裁。
    virtual void setControl(DeviceId id, Control control, float value, double timestamp) = 0;

    // 累加控件的本帧增量（鼠标移动、滚轮）；update 之后清零。
    virtual void addDelta(DeviceId id, Control control, float delta, double timestamp) = 0;

    // 文本输入（M1-13）：键盘布局与输入法处理之后得到的字符（Unicode 码点），供界面输入框使用。
    // 与控件无关，动作映射不使用；默认忽略。
    virtual void addText(char32_t codepoint) { (void)codepoint; }

protected:
    InputSink() = default;
    InputSink(const InputSink&) = default;
    InputSink& operator=(const InputSink&) = default;
};

}  // namespace terra::input
