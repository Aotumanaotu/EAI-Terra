#pragma once

// 输入系统（M1-04，D-018）：接收适配器写入的设备事件，按动作表计算每帧的动作状态。
//
// 每帧流程：适配器在事件轮询中调用 InputSink 接口 → 调用 update() 一次 → 消费者用 state() 读取动作。
// 消费者只依赖 ActionHandle / ActionState，不知道输入来自哪个设备或适配器。
// 线程：M1 单线程使用。

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "terra/input/action_map.hpp"
#include "terra/input/input_sink.hpp"
#include "terra/input/types.hpp"

namespace terra::input {

struct ActionHandle {
    std::uint32_t index = 0xFFFFFFFFu;
    constexpr bool valid() const noexcept { return index != 0xFFFFFFFFu; }
};

struct ActionState {
    float x = 0.0f;         // Button：0/1；Axis1D：值；Axis2D/Delta2D：x 分量
    float y = 0.0f;         // Axis2D/Delta2D：y 分量
    bool down = false;      // Button：按下中；其他类型：值非零
    bool pressed = false;   // 本帧由“未按下”变为“按下”（含帧内快速按下又抬起）
    bool released = false;  // 本帧由“按下”变为“未按下”（含帧内快速按下又抬起）
    DeviceId source;        // 生效来源设备；无活动来源时无效
};

class InputSystem final : public InputSink {
public:
    InputSystem();
    ~InputSystem() override;
    InputSystem(const InputSystem&) = delete;
    InputSystem& operator=(const InputSystem&) = delete;

    // ---- InputSink ----
    DeviceId connectDevice(const DeviceDescription& description) override;
    void disconnectDevice(DeviceId id) override;
    void resetDevice(DeviceId id) override;
    void setControl(DeviceId id, Control control, float value, double timestamp) override;
    void addDelta(DeviceId id, Control control, float delta, double timestamp) override;

    // ---- 配置 ----
    // 注册动作表。校验失败时返回错误说明，不做任何修改：表名重复、动作名重复、绑定种类与动作类型不符、
    // 同一绑定的控件种类不一致、死区参数不满足 0 ≤ inner < outer ≤ 1、裁剪 lo > hi、阈值不在 (0, 1]。
    std::optional<std::string> addActionMap(ActionMapDefinition definition);
    // 替换一个已注册动作的全部绑定（M1-14：按键设置）。校验规则与 addActionMap 相同，失败时不做修改；
    // 动作句柄保持有效。动作表或动作不存在时返回错误。
    std::optional<std::string> setBindings(std::string_view map, std::string_view action, std::vector<Binding> bindings);
    // 启用/停用动作表；停用时其动作在下一次 update 归零（按下中的按钮产生“抬起”）。返回表是否存在。
    bool setActionMapEnabled(std::string_view map, bool enabled);
    // 查找动作；不存在时返回无效句柄。
    ActionHandle findAction(std::string_view map, std::string_view action) const;

    // ---- 每帧 ----
    // 按当前控件值计算所有动作状态，然后清除本帧增量与边沿记录。
    void update();
    // 无效句柄返回全零状态。
    const ActionState& state(ActionHandle handle) const;

    // ---- 诊断 ----
    std::vector<DeviceInfo> devices() const;
    // 设备连接/断开时递增；消费者可据此发现设备列表变化（例如提示“手柄已断开”）。
    std::uint64_t deviceListRevision() const noexcept;
    // 控件当前值（绝对值 + 本帧增量）；设备不存在时为 0。
    float controlValue(DeviceId id, Control control) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace terra::input
