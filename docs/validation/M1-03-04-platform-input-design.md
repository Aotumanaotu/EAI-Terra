# M1-03 / M1-04 平台窗口与输入抽象：接口设计

> 状态：**设计待评审，未实现**。依据：D-017（GLFW 为首个适配器）、D-018（输入独立抽象）、D-021（OpenGL 4.5 core）、开发计划第 8 节 M1-03/04/05、PRD F-M1-01/F-M1-04。评审通过后实现，实现与验证另写报告。

## 1. 目标与约束

D-018 的要求逐条落到设计上：

| 要求 | 设计中的对应 |
| --- | --- |
| 输入模块独立抽象，GLFW 只是第一种设备适配器 | `terra_input` 不依赖任何窗口库；GLFW 适配器在平台层实现，只通过 `InputSink` 向输入系统写事件（第 3、4 节） |
| 相机、机器人控制器不得直接调用 GLFW | 消费者只链接 `terra_input`，只读**动作**；构建依赖和边界检查共同保证（第 6 节） |
| 未来支持 SDL、OpenXR、ROS2 等来源 | 来源 = 新的适配器，实现同一个 `InputSink` 写入协议；消费者与动作映射不变（第 3.7 节） |
| 统一动作映射 | 动作表（ActionMap）+ 绑定 + 组合绑定 + 处理器链（第 3.4 节） |
| 设备热插拔 | 设备连接/断开事件；断开时自动释放该设备所有控件；绑定按设备种类而非实例（第 3.5 节） |
| 死区处理 | 处理器链中的轴向/径向死区，带重新归一化（第 3.4 节） |
| 输入优先级管理 | 两层：动作表之间的优先级与消费，同一动作多来源的仲裁策略（第 3.6 节） |

其他约束：

- 输入只在帧边界被采样。影响物理的输入（M3 机器人控制）必须转成 Core 命令，在确定的步边界执行（技术设计 1.3 节），输入系统本身不写物理。
- M1 单线程：事件处理、动作计算和消费都在主线程。
- 公共接口只用 Terra 类型；GLFW、GL 类型只出现在平台实现内部。

## 2. 模块与依赖

```text
terra_input（纯 C++，无窗口依赖）         ◀── 相机控制器 / 机器人控制器（只依赖它）
     ▲ InputSink
     │
terra_platform（接口：Platform、Window）
     ▲
terra_platform_glfw（GLFW 实现 + GLFW 键鼠/手柄适配器；PRIVATE 链接 GLFW）
     ▲
应用（apps/viewer，组合入口：创建平台、窗口、输入系统，并把它们连起来）
```

| 目标 | 位置 | 依赖 |
| --- | --- | --- |
| `terra_input` | `engine/input/` | 只依赖标准库与 `terra_core` |
| `terra_platform` | `engine/platform/include/` | 仅头文件；只依赖 `terra_input`（`InputSink` 类型） |
| `terra_platform_glfw` | `engine/platform/backends/glfw/` | `terra_platform`；**PRIVATE** `terra::glfw` |

`terra_input` 不受 `TERRA_BUILD_RENDER` 控制，总是构建，单元测试在无图形环境运行；平台与 GLFW 目标只在渲染开启时构建。

## 3. 输入系统（`terra_input`）

### 3.1 分层

```text
适配器（GLFW / 虚拟设备 / 以后的 SDL、OpenXR、ROS2）
   │  InputSink：设备连接/断开、控件值变化
   ▼
设备层：DeviceRegistry（设备实例、种类、稳定标识、当前控件值）
   ▼
映射层：ActionMap（动作、绑定、组合、处理器链）× 优先级与消费
   ▼
动作状态：每帧计算一次（值、按下/抬起边沿、来源设备）
   ▼
消费者：相机控制器等，只按 ActionHandle 读取
```

### 3.2 设备与控件

- `DeviceId`：设备**连接实例**的标识，由输入系统分配，只增不复用；设备断开后旧 ID 失效。
- `DeviceInfo`：`id`、`kind`（`Keyboard`、`Mouse`、`Gamepad`，预留 `XrController`、`External`）、`name`、`source`（适配器名，如 `"glfw"`）、`stableKey`（跨重连识别同一物理设备，例如手柄 GUID，可为空）。
- 控件 `Control`：`{DeviceKind kind; std::uint16_t code}`，code 在各种类内定义：
  - 键盘：Terra 自己的 `Key` 枚举，**按物理键位**命名（采用美式键盘布局的名称，与 USB HID、GLFW、SDL 的键位语义一致），不随键盘布局变化；需要文字输入时另走文本通道（M1 不做）。
  - 鼠标：按键、指针位置、移动增量 dx/dy、滚轮 x/y。
  - 手柄：标准布局（与 GLFW/SDL 的 gamepad mapping 一致）：A/B/X/Y、肩键、扳机、两根摇杆的 x/y、方向键等。
- 控件值统一用 `float`：按键 0/1；轴 −1…1；扳机 0…1；增量不限范围（单位：像素或滚轮格数）。

适配器负责把来源自己的编码翻译成 Terra 的控件；翻译表是纯函数，可以单独测试。

### 3.3 写入协议 `InputSink`

```cpp
class InputSink {
public:
    virtual DeviceId connectDevice(const DeviceDescription& desc) = 0;
    virtual void disconnectDevice(DeviceId id) = 0;
    virtual void setControl(DeviceId id, Control control, float value, double timestamp) = 0;  // 绝对值
    virtual void addDelta(DeviceId id, Control control, float delta, double timestamp) = 0;    // 本帧累加
};
```

`InputSystem` 实现 `InputSink`。适配器只能看到 `InputSink`，看不到动作与消费者。

### 3.4 动作映射

- **动作类型**：`Button`（按下/抬起边沿）、`Axis1D`、`Axis2D`、`Delta2D`（鼠标拖动、视角转动之类按帧累加的量）。
- **绑定**：`控件 → 动作`；可附加**修饰键条件**（例如 Shift + 鼠标左键拖动 = 平移）。
- **组合绑定**：多个控件合成一个值，例如 W/A/S/D → `Axis2D`，Q/E → `Axis1D`。
- **处理器链**（按绑定配置，依次作用）：
  - `AxialDeadzone{inner, outer}`：单轴，|x| < inner 归零，(inner, outer) 线性重映射到 (0, 1)，> outer 饱和；
  - `RadialDeadzone{inner, outer}`：双轴按向量长度处理，避免方形死区让斜向变“粘”；
  - `Scale{k}`、`Invert`、`Clamp`。

  死区放在处理器而不是适配器里，同一根摇杆在不同动作中可以用不同死区。
- **动作表 `ActionMap`**：一组动作及其绑定，带名称（如 `camera`、`robot_teleop`）、启用状态和优先级。

### 3.5 热插拔

- 适配器检测到设备连接/断开时调用 `connectDevice`/`disconnectDevice`。
- **断开时自动释放**：该设备所有非零控件被视为归零，按下中的按钮在下一帧产生“抬起”边沿。避免按住按钮时拔掉手柄导致相机或机器人一直运动。
- **绑定按设备种类，不按实例**：新插入的手柄自动生效，不需要重新绑定。需要“玩家 1 的手柄”这类指定实例的绑定时，按 `stableKey` 匹配（M1 只预留字段）。
- 设备列表变化通知消费者（可选回调），例如界面提示“手柄已断开”。

### 3.6 优先级

两层，解决不同问题：

1. **动作表之间（谁拿到输入）**：启用的动作表按优先级从高到低处理；某个绑定标记为**消费**时，同一控件不再传给更低优先级的表。典型场景：M2/M5 的拾取或拖动工具（高优先级）按下鼠标左键时，相机（低优先级）不应同时旋转。
2. **同一动作的多个来源（谁说了算）**：同一个动作同时有键盘和手柄在驱动时，按该动作配置的仲裁策略：
   - `LargestMagnitude`（默认）：取绝对值最大的来源，适合连续轴；
   - `MostRecent`：取最近变化的来源，适合“谁最后操作听谁的”；
   - `SourcePriority`：按来源优先级排序取第一个非零值。例如 M4 中，本地手柄的急停优先于 ROS2 远程指令。

   动作状态会报告当前生效的来源设备，便于诊断。

### 3.7 新来源怎么接入

| 来源 | 设备种类 | 说明 |
| --- | --- | --- |
| GLFW（M1） | Keyboard、Mouse、Gamepad | 窗口回调与手柄轮询 → `InputSink` |
| SDL（后续） | 同上 | 另一个适配器；同一台机器上不与 GLFW 同时提供同一类设备 |
| OpenXR（后续） | `XrController` | 位姿类控件需要扩展值类型（位姿），届时评审；现有动作类型不受影响 |
| ROS2（M4） | `External` | 外部指令作为“虚拟设备”接入，享有同样的映射、死区与优先级；时间语义按 D-104 处理 |

### 3.8 消费者接口

```cpp
ActionHandle orbit = system.findAction("camera", "orbit");   // 启动时解析一次
...
system.update(frameTime);                                     // 每帧一次：处理积压事件、计算动作状态
const ActionState& s = system.state(orbit);                   // s.value2d、s.pressed、s.released、s.source
```

相机控制器只依赖 `ActionHandle` 和 `ActionState`。它不知道输入来自键盘、手柄还是 ROS2，也就不可能调用 GLFW。

## 4. 平台层（`terra_platform`、`terra_platform_glfw`）

```cpp
namespace terra::platform {

struct WindowDescription {
    std::string title;
    int width = 1280, height = 720;
    bool visible = true;
    bool vsync = true;
    bool glDebugContext = false;   // 调试构建开启，配合 KHR_debug（4.5 core 内置）
};

enum class PlatformErrorKind { NoDisplay, ContextUnavailable, WindowCreation, Internal };
struct PlatformError { PlatformErrorKind kind; std::string message; };

using GlProcLoader = void* (*)(const char* name);   // 交给 glad；不是 GLFW 类型

class Window {
public:
    virtual ~Window() = default;
    virtual bool closeRequested() const = 0;
    virtual void requestClose() = 0;
    virtual Extent framebufferSize() const = 0;   // 像素；与窗口尺寸不同（高 DPI）
    virtual float contentScale() const = 0;
    virtual void makeContextCurrent() = 0;
    virtual void swapBuffers() = 0;
    virtual GlProcLoader glProcLoader() const = 0;
};

class Platform {
public:
    virtual ~Platform() = default;
    virtual Result<std::unique_ptr<Window>> createWindow(const WindowDescription&) = 0;
    virtual void pollEvents() = 0;          // 把本次的窗口与输入事件写入绑定的 InputSink
    virtual double time() const = 0;        // 单调时钟，秒
};

// GLFW 实现的入口（terra/platform/glfw/glfw_platform.hpp）：唯一与 GLFW 相关的公共符号
Result<std::unique_ptr<Platform>> createGlfwPlatform(InputSink& input);
}
```

- **上下文**：固定请求 OpenGL 4.5 core、forward-compatible（D-021）。版本不足时返回 `ContextUnavailable`，消息中写明请求的版本、实际可用的版本与驱动。
- **无显示**：`DISPLAY`/`WAYLAND_DISPLAY` 都没有设置时返回 `NoDisplay`，不崩溃。
- **GLFW 是进程级单例**：同一进程只允许一个 GLFW 平台实例，重复创建明确报错。
- **键鼠是窗口事件**：GLFW 键鼠适配器在窗口回调中把事件翻译后写入 `InputSink`；窗口失去焦点时释放所有按下的键（与热插拔断开同一套逻辑，避免切换窗口后按键卡住）。

## 5. 分期（相对开发计划第 8 节的调整建议）

与原草案相比，把**死区和热插拔的核心逻辑**提前到 M1-04（P0）。它们是纯逻辑，用虚拟设备就能完整测试，成本低；只有**GLFW 手柄适配器**留在 M1-05（P1）。

| 任务 | 内容 |
| --- | --- |
| M1-03（P0） | `terra_platform` 接口；GLFW 平台与窗口；OpenGL 4.5 上下文；错误路径 |
| M1-04（P0） | `terra_input` 全部核心：设备层、`InputSink`、动作类型、绑定/组合/修饰键、处理器链（含两种死区）、热插拔与断开释放、两层优先级与三种仲裁策略；GLFW 键鼠适配器；虚拟设备测试工具 |
| M1-05（P1） | GLFW 手柄适配器（连接/断开回调、标准布局映射）；实机手柄为可选人工检查 |

## 6. 测试计划

| 测试 | 环境 | 内容 |
| --- | --- | --- |
| `input.*` 单元测试 | 无图形（普通 CTest，渲染关闭时也运行） | 用虚拟设备注入事件：按钮边沿；WASD 组合；修饰键；轴向/径向死区的数值（边界、重新归一化、斜向）；热插拔断开释放（按住时断开 → 下一帧抬起）；新设备自动生效；动作表优先级与消费；三种仲裁策略；帧内多次事件的合并；`DeviceId` 不复用 |
| `input.glfw_translation` | 无图形 | GLFW 键码、鼠标键、手柄布局到 Terra 控件的翻译表（纯函数） |
| `graphics.platform_window` | Xvfb + llvmpipe | 创建/关闭窗口；帧缓冲尺寸；4.5 上下文与 glad 加载；**请求 4.6 时在 llvmpipe 上返回 `ContextUnavailable`**（确定性的负向用例，复用 D-022 的固定环境）；无显示时返回 `NoDisplay` |
| `boundary.render_isolation` | 无图形 | 扩展边界检查：`GLFW/` 头文件只允许出现在 `engine/platform/backends/glfw/src/` 与测试中；`glad/` 只允许在平台实现与 `engine/render/` 中；`engine/input/` 不得包含任何窗口或 GL 头文件 |
| 依赖检查 | 构建 | `terra_input` 与控制器目标的链接依赖中没有 GLFW（CMake 目标关系 + ldd） |

真实键鼠事件很难在 Xvfb 中可靠地合成，GLFW 回调本身只做“翻译后转发”，覆盖方式是：翻译表单元测试 + 输入系统单元测试 + M1-07 时在桌面会话做一次人工交互检查。

## 7. 待评审

1. **整体分层与 `InputSink` 写入协议**（第 3.1–3.3 节）。
2. **分期调整**：死区与热插拔核心逻辑提前到 M1-04（第 5 节）。
3. **多来源仲裁的默认策略**：建议 `LargestMagnitude`（第 3.6 节）。
4. **动作映射的配置方式**（开发计划 8.3 节待定项）：建议 M1 先用代码注册（数据结构即配置），配置文件格式在有外部配置需求时再定；数据结构从一开始就设计成可序列化。
5. **键盘按物理键位命名**（第 3.2 节），文本输入不在 M1 范围内。
