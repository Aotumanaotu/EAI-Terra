// 输入系统单元测试（M1-04，D-018）。用虚拟设备注入事件，不需要窗口或图形环境。
// 覆盖：按钮边沿与帧内快速点按、组合与修饰键、增量累加、轴向/径向死区数值、热插拔与断开释放、
// 焦点丢失释放、动作表优先级与消费、三种多来源仲裁、配置校验、DeviceId 不复用、非有限值。

#include <cmath>
#include <cstdio>
#include <string>

#include "terra/input/input_system.hpp"

namespace {

using namespace terra::input;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

bool near(float a, float b, float tol = 1e-5f) { return std::fabs(a - b) <= tol; }

std::string num(float v) { return std::to_string(v); }

DeviceDescription virtualDevice(DeviceKind kind, const char* name) {
    return DeviceDescription{kind, name, "virtual", ""};
}

// 注册单个动作表的简便写法；注册失败记为测试失败。
void addMap(InputSystem& sys, ActionMapDefinition map) {
    const std::string name = map.name;
    if (auto error = sys.addActionMap(std::move(map))) check(false, "注册动作表 " + name + "：" + *error);
}

ActionDefinition action(const char* name, ActionType type, std::vector<Binding> bindings,
                        Arbitration arbitration = Arbitration::LargestMagnitude) {
    ActionDefinition a;
    a.name = name;
    a.type = type;
    a.arbitration = arbitration;
    a.bindings = std::move(bindings);
    return a;
}

void testButtonEdges() {
    InputSystem sys;
    const DeviceId kb = sys.connectDevice(virtualDevice(DeviceKind::Keyboard, "kb"));
    addMap(sys, {"m", 0, true, {action("jump", ActionType::Button, {Binding::single(key(Key::Space))})}});
    const ActionHandle jump = sys.findAction("m", "jump");
    check(jump.valid(), "找到动作 m.jump");

    sys.update();
    check(!sys.state(jump).down && !sys.state(jump).pressed, "初始未按下");

    sys.setControl(kb, key(Key::Space), 1.0f, 1.0);
    sys.update();
    const ActionState& s = sys.state(jump);
    check(s.down && s.pressed && !s.released && s.x == 1.0f && s.source == kb, "按下：down + pressed，来源为键盘");

    sys.update();
    check(sys.state(jump).down && !sys.state(jump).pressed, "按住：只有 down，没有新的 pressed");

    sys.setControl(kb, key(Key::Space), 0.0f, 2.0);
    sys.update();
    check(!sys.state(jump).down && sys.state(jump).released && !sys.state(jump).source.valid(), "抬起：released，来源清空");

    // 帧内按下又抬起：不能丢失
    sys.setControl(kb, key(Key::Space), 1.0f, 3.0);
    sys.setControl(kb, key(Key::Space), 0.0f, 3.01);
    sys.update();
    check(sys.state(jump).pressed && sys.state(jump).released && !sys.state(jump).down,
          "帧内快速点按：同时报告 pressed 与 released，down 为 false");
    sys.update();
    check(!sys.state(jump).pressed && !sys.state(jump).released, "快速点按只报告一次");

    check(!sys.findAction("m", "nope").valid() && !sys.findAction("x", "jump").valid(), "不存在的动作返回无效句柄");
    const ActionState& empty = sys.state(ActionHandle{});
    check(!empty.down && empty.x == 0.0f, "无效句柄返回全零状态");
}

void testCompositeModifiersAndDeltas() {
    InputSystem sys;
    const DeviceId kb = sys.connectDevice(virtualDevice(DeviceKind::Keyboard, "kb"));
    const DeviceId ms = sys.connectDevice(virtualDevice(DeviceKind::Mouse, "mouse"));
    addMap(sys, {"camera", 0, true,
                 {action("move", ActionType::Axis2D,
                         {Binding::composite2D(key(Key::W), key(Key::S), key(Key::A), key(Key::D))}),
                  action("zoom", ActionType::Axis1D, {Binding::composite1D(key(Key::Q), key(Key::E))}),
                  action("pan", ActionType::Delta2D,
                         {Binding::pair(mouse(MouseControl::DeltaX), mouse(MouseControl::DeltaY))
                              .withModifier(key(Key::LeftShift))})}});
    const ActionHandle move = sys.findAction("camera", "move");
    const ActionHandle zoom = sys.findAction("camera", "zoom");
    const ActionHandle pan = sys.findAction("camera", "pan");

    sys.setControl(kb, key(Key::W), 1.0f, 1.0);
    sys.setControl(kb, key(Key::D), 1.0f, 1.0);
    sys.update();
    check(sys.state(move).x == 1.0f && sys.state(move).y == 1.0f, "W+D → (1, 1)");
    sys.setControl(kb, key(Key::S), 1.0f, 1.1);
    sys.update();
    check(sys.state(move).x == 1.0f && sys.state(move).y == 0.0f, "W 与 S 相互抵消 → y = 0");

    sys.setControl(kb, key(Key::Q), 1.0f, 1.2);
    sys.update();
    check(sys.state(zoom).x == -1.0f, "Q → -1");

    // 增量：同一帧多次事件累加，下一帧清零；修饰键未按下时绑定不生效
    sys.addDelta(ms, mouse(MouseControl::DeltaX), 3.0f, 2.0);
    sys.addDelta(ms, mouse(MouseControl::DeltaX), 4.0f, 2.0);
    sys.addDelta(ms, mouse(MouseControl::DeltaY), -2.0f, 2.0);
    sys.update();
    check(sys.state(pan).x == 0.0f && !sys.state(pan).down, "未按 Shift：平移不生效");

    sys.setControl(kb, key(Key::LeftShift), 1.0f, 3.0);
    sys.addDelta(ms, mouse(MouseControl::DeltaX), 3.0f, 3.0);
    sys.addDelta(ms, mouse(MouseControl::DeltaX), 4.0f, 3.0);
    sys.addDelta(ms, mouse(MouseControl::DeltaY), -2.0f, 3.0);
    sys.update();
    check(sys.state(pan).x == 7.0f && sys.state(pan).y == -2.0f && sys.state(pan).source == ms,
          "Shift + 鼠标移动：同一帧增量累加为 (7, -2)，来源为鼠标");
    sys.update();
    check(sys.state(pan).x == 0.0f && sys.state(pan).y == 0.0f, "下一帧没有新增量 → 归零");
}

void testDeadzones() {
    InputSystem sys;
    const DeviceId pad = sys.connectDevice(virtualDevice(DeviceKind::Gamepad, "pad"));
    const Control lx = gamepad(GamepadControl::LeftX);
    const Control ly = gamepad(GamepadControl::LeftY);
    addMap(sys, {"m", 0, true,
                 {action("axial1d", ActionType::Axis1D,
                         {Binding::single(lx).withProcessor(Processor::axialDeadzone(0.2f, 0.9f))}),
                  action("axial2d", ActionType::Axis2D,
                         {Binding::pair(lx, ly).withProcessor(Processor::axialDeadzone(0.2f))}),
                  action("radial", ActionType::Axis2D,
                         {Binding::pair(lx, ly).withProcessor(Processor::radialDeadzone(0.2f))}),
                  action("scaled", ActionType::Axis2D,
                         {Binding::pair(lx, ly)
                              .withProcessor(Processor::scale(2.0f, -1.0f))
                              .withProcessor(Processor::clamp(-1.0f, 1.0f))})}});
    const ActionHandle axial1d = sys.findAction("m", "axial1d");
    const ActionHandle axial2d = sys.findAction("m", "axial2d");
    const ActionHandle radial = sys.findAction("m", "radial");
    const ActionHandle scaled = sys.findAction("m", "scaled");

    // 轴向死区 inner 0.2、outer 0.9：线性重新归一化到 [0, 1]
    const struct { float in, out; } axialCases[] = {
        {0.1f, 0.0f}, {0.2f, 0.0f}, {0.55f, 0.5f}, {0.9f, 1.0f}, {1.0f, 1.0f}, {-0.55f, -0.5f}};
    for (const auto& c : axialCases) {
        sys.setControl(pad, lx, c.in, 1.0);
        sys.setControl(pad, ly, 0.0f, 1.0);
        sys.update();
        check(near(sys.state(axial1d).x, c.out),
              "轴向死区 (0.2, 0.9)：" + num(c.in) + " → " + num(c.out) + "，实际 " + num(sys.state(axial1d).x));
    }

    // 径向死区 inner 0.2：按长度判断并重新归一化，方向不变
    sys.setControl(pad, lx, 0.1f, 2.0);
    sys.setControl(pad, ly, 0.1f, 2.0);
    sys.update();
    check(sys.state(radial).x == 0.0f && sys.state(radial).y == 0.0f && !sys.state(radial).down,
          "径向：长度 0.141 < 0.2 → 0");
    sys.setControl(pad, lx, 0.3f, 2.1);
    sys.setControl(pad, ly, 0.4f, 2.1);
    sys.update();
    check(near(sys.state(radial).x, 0.225f) && near(sys.state(radial).y, 0.3f),
          "径向：(0.3, 0.4) 长度 0.5 → 长度 0.375，方向不变 → (0.225, 0.3)");
    sys.setControl(pad, lx, 0.6f, 2.2);
    sys.setControl(pad, ly, 0.8f, 2.2);
    sys.update();
    check(near(sys.state(radial).x, 0.6f) && near(sys.state(radial).y, 0.8f), "径向：长度 1 → 不变");

    // 斜向小幅推杆：径向保留方向，轴向（方形死区）把它吃掉——这正是需要径向死区的原因
    sys.setControl(pad, lx, 0.15f, 2.3);
    sys.setControl(pad, ly, 0.15f, 2.3);
    sys.update();
    check(sys.state(axial2d).x == 0.0f && sys.state(axial2d).y == 0.0f, "斜向 (0.15, 0.15)：轴向死区 → 0");
    check(sys.state(radial).x > 0.0f && near(sys.state(radial).x, sys.state(radial).y), "斜向 (0.15, 0.15)：径向死区保留 45° 方向");

    // 缩放与裁剪：x * 2 后裁剪到 1，y 取反
    sys.setControl(pad, lx, 0.7f, 2.4);
    sys.setControl(pad, ly, 0.5f, 2.4);
    sys.update();
    check(near(sys.state(scaled).x, 1.0f) && near(sys.state(scaled).y, -0.5f), "缩放 (2, -1) + 裁剪 [-1, 1] → (1, -0.5)");
}

void testTriggerAsButton() {
    InputSystem sys;
    const DeviceId pad = sys.connectDevice(virtualDevice(DeviceKind::Gamepad, "pad"));
    ActionDefinition fire = action("fire", ActionType::Button, {Binding::single(gamepad(GamepadControl::RightTrigger))});
    fire.pressThreshold = 0.5f;
    addMap(sys, {"m", 0, true, {fire}});
    const ActionHandle h = sys.findAction("m", "fire");
    sys.setControl(pad, gamepad(GamepadControl::RightTrigger), 0.4f, 1.0);
    sys.update();
    check(!sys.state(h).down, "扳机 0.4 < 阈值：未按下");
    sys.setControl(pad, gamepad(GamepadControl::RightTrigger), 0.6f, 2.0);
    sys.update();
    check(sys.state(h).down && sys.state(h).pressed && sys.state(h).x == 1.0f, "扳机 0.6 ≥ 阈值：按下，值为 1");
}

void testHotplug() {
    InputSystem sys;
    const std::uint64_t rev0 = sys.deviceListRevision();
    addMap(sys, {"m", 0, true,
                 {action("a", ActionType::Button, {Binding::single(gamepad(GamepadControl::A))}),
                  action("stick", ActionType::Axis2D,
                         {Binding::pair(gamepad(GamepadControl::LeftX), gamepad(GamepadControl::LeftY))})}});
    const ActionHandle a = sys.findAction("m", "a");
    const ActionHandle stick = sys.findAction("m", "stick");

    const DeviceId pad1 = sys.connectDevice(virtualDevice(DeviceKind::Gamepad, "pad1"));
    check(sys.deviceListRevision() == rev0 + 1 && sys.devices().size() == 1, "连接后设备列表更新");
    sys.setControl(pad1, gamepad(GamepadControl::A), 1.0f, 1.0);
    sys.setControl(pad1, gamepad(GamepadControl::LeftX), 0.8f, 1.0);
    sys.update();
    check(sys.state(a).down && sys.state(stick).x == 0.8f, "手柄 1：A 按下，摇杆 0.8");

    // 按住时断开：下一帧必须抬起、摇杆归零（避免机器人或相机一直运动）
    sys.disconnectDevice(pad1);
    check(sys.deviceListRevision() == rev0 + 2 && sys.devices().empty(), "断开后设备列表更新");
    sys.update();
    check(!sys.state(a).down && sys.state(a).released, "断开：按下中的按钮产生 released");
    check(sys.state(stick).x == 0.0f && !sys.state(stick).down && !sys.state(stick).source.valid(), "断开：摇杆归零");

    // 断开后向旧 ID 写入被忽略；新设备自动生效，ID 不复用
    sys.setControl(pad1, gamepad(GamepadControl::A), 1.0f, 2.0);
    sys.update();
    check(!sys.state(a).down, "向已断开设备写入被忽略");
    const DeviceId pad2 = sys.connectDevice(virtualDevice(DeviceKind::Gamepad, "pad2"));
    check(pad2 != pad1 && pad1 < pad2, "DeviceId 不复用");
    sys.setControl(pad2, gamepad(GamepadControl::A), 1.0f, 3.0);
    sys.update();
    check(sys.state(a).down && sys.state(a).source == pad2, "新插入的手柄无需重新绑定即可生效");
    sys.disconnectDevice(DeviceId{999});  // 未知 ID：忽略
}

void testResetDevice() {
    InputSystem sys;
    const DeviceId kb = sys.connectDevice(virtualDevice(DeviceKind::Keyboard, "kb"));
    addMap(sys, {"m", 0, true, {action("fwd", ActionType::Button, {Binding::single(key(Key::W))})}});
    const ActionHandle fwd = sys.findAction("m", "fwd");
    sys.setControl(kb, key(Key::W), 1.0f, 1.0);
    sys.update();
    check(sys.state(fwd).down, "W 按住");
    sys.resetDevice(kb);  // 窗口失去焦点
    sys.update();
    check(!sys.state(fwd).down && sys.state(fwd).released && sys.devices().size() == 1,
          "焦点丢失（resetDevice）：释放按键，设备保持连接");
}

void testPriorityAndConsume() {
    InputSystem sys;
    const DeviceId ms = sys.connectDevice(virtualDevice(DeviceKind::Mouse, "mouse"));
    const Control lmb = mouse(MouseControl::ButtonLeft);
    addMap(sys, {"camera", 0, true, {action("orbit", ActionType::Button, {Binding::single(lmb)})}});
    addMap(sys, {"tool", 10, true, {action("drag", ActionType::Button, {Binding::single(lmb)})}});
    addMap(sys, {"overlay", 5, true,
                 {action("hover", ActionType::Button, {Binding::single(lmb).nonConsuming()})}});
    const ActionHandle orbit = sys.findAction("camera", "orbit");
    const ActionHandle drag = sys.findAction("tool", "drag");
    const ActionHandle hover = sys.findAction("overlay", "hover");

    sys.setControl(ms, lmb, 1.0f, 1.0);
    sys.update();
    check(sys.state(drag).down, "高优先级工具拿到左键");
    check(!sys.state(hover).down, "工具消费了左键：中优先级表也拿不到");
    check(!sys.state(orbit).down, "工具消费了左键：相机不旋转");

    check(sys.setActionMapEnabled("tool", false), "停用工具表");
    sys.update();
    check(!sys.state(drag).down && sys.state(drag).released, "停用后工具动作抬起");
    check(sys.state(hover).down && sys.state(hover).pressed, "工具停用后，中优先级表拿到左键");
    check(sys.state(orbit).down, "overlay 的绑定不消费：相机同时拿到左键");
    check(!sys.setActionMapEnabled("nope", true), "不存在的动作表返回 false");
}

void testArbitration() {
    // 最大幅度（默认）：键盘 1 对手柄 0.5
    {
        InputSystem sys;
        const DeviceId kb = sys.connectDevice(virtualDevice(DeviceKind::Keyboard, "kb"));
        const DeviceId pad = sys.connectDevice(virtualDevice(DeviceKind::Gamepad, "pad"));
        addMap(sys, {"m", 0, true,
                     {action("turn", ActionType::Axis1D,
                             {Binding::composite1D(key(Key::A), key(Key::D)),
                              Binding::single(gamepad(GamepadControl::RightX))})}});
        const ActionHandle turn = sys.findAction("m", "turn");
        sys.setControl(pad, gamepad(GamepadControl::RightX), 0.5f, 1.0);
        sys.update();
        check(sys.state(turn).x == 0.5f && sys.state(turn).source == pad, "最大幅度：只有手柄 → 0.5，来源手柄");
        sys.setControl(kb, key(Key::A), 1.0f, 2.0);
        sys.update();
        check(sys.state(turn).x == -1.0f && sys.state(turn).source == kb, "最大幅度：键盘 |−1| > 0.5 → 键盘生效");
    }
    // 最近变化
    {
        InputSystem sys;
        const DeviceId kb = sys.connectDevice(virtualDevice(DeviceKind::Keyboard, "kb"));
        const DeviceId pad = sys.connectDevice(virtualDevice(DeviceKind::Gamepad, "pad"));
        addMap(sys, {"m", 0, true,
                     {action("turn", ActionType::Axis1D,
                             {Binding::composite1D(key(Key::A), key(Key::D)),
                              Binding::single(gamepad(GamepadControl::RightX))},
                             Arbitration::MostRecent)}});
        const ActionHandle turn = sys.findAction("m", "turn");
        sys.setControl(pad, gamepad(GamepadControl::RightX), 0.8f, 1.0);
        sys.setControl(kb, key(Key::D), 1.0f, 2.0);
        sys.update();
        check(sys.state(turn).x == 1.0f && sys.state(turn).source == kb, "最近变化：键盘（t=2）晚于手柄（t=1）");
        sys.setControl(pad, gamepad(GamepadControl::RightX), 0.3f, 3.0);
        sys.update();
        check(sys.state(turn).x == 0.3f && sys.state(turn).source == pad, "最近变化：手柄 t=3 变化 → 手柄生效，即使幅度更小");
    }
    // 来源优先级：本地手柄（优先级 10）高于外部指令（优先级 0），例如 ROS2 遥控时本地急停/接管
    {
        InputSystem sys;
        const DeviceId ext = sys.connectDevice(virtualDevice(DeviceKind::External, "ros2"));
        const DeviceId pad = sys.connectDevice(virtualDevice(DeviceKind::Gamepad, "pad"));
        const Control extSpeed{DeviceKind::External, 0};
        addMap(sys, {"robot", 0, true,
                     {action("speed", ActionType::Axis1D,
                             {Binding::single(extSpeed),
                              Binding::single(gamepad(GamepadControl::LeftY)).withSourcePriority(10)},
                             Arbitration::SourcePriority)}});
        const ActionHandle speed = sys.findAction("robot", "speed");
        sys.setControl(ext, extSpeed, 1.0f, 1.0);
        sys.update();
        check(sys.state(speed).x == 1.0f && sys.state(speed).source == ext, "来源优先级：只有外部指令 → 外部生效");
        sys.setControl(pad, gamepad(GamepadControl::LeftY), 0.2f, 2.0);
        sys.update();
        check(sys.state(speed).x == 0.2f && sys.state(speed).source == pad, "来源优先级：本地手柄接管，即使幅度更小");
    }
}

void testValidation() {
    InputSystem sys;
    addMap(sys, {"m", 0, true, {action("ok", ActionType::Button, {Binding::single(key(Key::A))})}});
    const auto expectError = [&](ActionMapDefinition map, const char* what) {
        const auto error = sys.addActionMap(std::move(map));
        check(error.has_value(), std::string{"应拒绝："} + what);
        if (error) std::printf("INFO 拒绝 %s：%s\n", what, error->c_str());
    };
    expectError({"m", 0, true, {}}, "动作表重名");
    expectError({"", 0, true, {}}, "动作表名为空");
    expectError({"n1", 0, true,
                 {action("x", ActionType::Button, {Binding::single(key(Key::A))}),
                  action("x", ActionType::Button, {Binding::single(key(Key::B))})}},
                "动作重名");
    expectError({"n2", 0, true,
                 {action("x", ActionType::Button, {Binding::pair(gamepad(GamepadControl::LeftX), gamepad(GamepadControl::LeftY))})}},
                "按钮动作使用双轴绑定");
    expectError({"n3", 0, true,
                 {action("x", ActionType::Axis2D,
                         {Binding::composite2D(key(Key::W), key(Key::S), key(Key::A), gamepad(GamepadControl::A))})}},
                "组合绑定混用设备种类");
    expectError({"n4", 0, true,
                 {action("x", ActionType::Axis1D,
                         {Binding::single(gamepad(GamepadControl::LeftX)).withProcessor(Processor::axialDeadzone(0.5f, 0.5f))})}},
                "死区 inner ≥ outer");
    expectError({"n5", 0, true,
                 {action("x", ActionType::Axis2D,
                         {Binding::pair(gamepad(GamepadControl::LeftX), gamepad(GamepadControl::LeftY))
                              .withProcessor(Processor::radialDeadzone(-0.1f))})}},
                "死区 inner < 0");
    expectError({"n6", 0, true,
                 {action("x", ActionType::Axis1D,
                         {Binding::single(gamepad(GamepadControl::LeftX)).withProcessor(Processor::clamp(1.0f, -1.0f))})}},
                "裁剪 lo > hi");
    ActionDefinition badThreshold = action("x", ActionType::Button, {Binding::single(key(Key::A))});
    badThreshold.pressThreshold = 0.0f;
    expectError({"n7", 0, true, {badThreshold}}, "按下阈值为 0");
    expectError({"n8", 0, true,
                 {action("x", ActionType::Delta2D, {Binding::composite2D(key(Key::W), key(Key::S), key(Key::A), key(Key::D))})}},
                "增量动作使用按键组合");
    check(!sys.findAction("n1", "x").valid() && !sys.findAction("n3", "x").valid(), "校验失败的动作表没有被部分注册");
    check(sys.findAction("m", "ok").valid(), "已有的动作表不受影响");
}

void testNonFinite() {
    InputSystem sys;
    const DeviceId pad = sys.connectDevice(virtualDevice(DeviceKind::Gamepad, "pad"));
    addMap(sys, {"m", 0, true, {action("x", ActionType::Axis1D, {Binding::single(gamepad(GamepadControl::LeftX))})}});
    const ActionHandle h = sys.findAction("m", "x");
    sys.setControl(pad, gamepad(GamepadControl::LeftX), 0.5f, 1.0);
    sys.setControl(pad, gamepad(GamepadControl::LeftX), std::nanf(""), 2.0);
    sys.addDelta(pad, gamepad(GamepadControl::LeftX), INFINITY, 2.0);
    sys.update();
    check(sys.state(h).x == 0.5f && std::isfinite(sys.controlValue(pad, gamepad(GamepadControl::LeftX))),
          "非有限值被忽略，保留之前的 0.5");
}

}  // namespace

int main() {
    testButtonEdges();
    testCompositeModifiersAndDeltas();
    testDeadzones();
    testTriggerAsButton();
    testHotplug();
    testResetDevice();
    testPriorityAndConsume();
    testArbitration();
    testValidation();
    testNonFinite();
    if (failures == 0) {
        std::printf("all passed\n");
    }
    return failures == 0 ? 0 : 1;
}
