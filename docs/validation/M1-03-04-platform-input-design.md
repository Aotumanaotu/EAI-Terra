# M1-03 / M1-04 平台窗口与输入抽象：接口设计

> 状态：**待验收**。设计已评审通过（2026-11-19，第 7 节 5 条全部同意）；实现与测试见第 8–9 节。基线机器（2026-11-19）全部通过：渲染关闭 37/37，渲染开启 Debug 与 Release + `-Werror` 41/41，ASan 全部通过，变异 8/8（第 10.2 节）。依据：D-017（GLFW 为首个适配器）、D-018（输入独立抽象）、D-021（OpenGL 4.5 core）、开发计划第 8 节 M1-03/04/05、PRD F-M1-01/F-M1-04。

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

## 7. 评审结论（2026-11-19，第 1–5 条全部同意）

1. **整体分层与 `InputSink` 写入协议**（第 3.1–3.3 节）。
2. **分期调整**：死区与热插拔核心逻辑提前到 M1-04（第 5 节）。
3. **多来源仲裁的默认策略**：建议 `LargestMagnitude`（第 3.6 节）。
4. **动作映射的配置方式**（开发计划 8.3 节待定项）：建议 M1 先用代码注册（数据结构即配置），配置文件格式在有外部配置需求时再定；数据结构从一开始就设计成可序列化。
5. **键盘按物理键位命名**（第 3.2 节），文本输入不在 M1 范围内。

## 8. 实现

| 部分 | 位置 | 说明 |
| --- | --- | --- |
| 设备与控件 | `engine/input/include/terra/input/types.hpp` | `DeviceKind`、`DeviceId`、`Control`；`Key`（105 个物理键位）、`MouseControl`、`GamepadControl`；`DeviceDescription` |
| 写入协议 | `input_sink.hpp` | `connectDevice`、`disconnectDevice`、`resetDevice`（焦点丢失）、`setControl`、`addDelta` |
| 动作映射 | `action_map.hpp` | 四种动作类型；`Binding`（Single、Pair、Composite1D/2D，修饰键、处理器、消费、来源优先级）；`Processor`（轴向/径向死区、缩放、裁剪）；三种仲裁策略；纯数据结构，可序列化 |
| 输入系统 | `input_system.hpp`、`src/input_system.cpp` | `InputSystem : InputSink`；注册时校验并原子生效；`update()` 按动作表优先级计算、消费与仲裁；帧内快速点按通过上升沿记录保留；非有限值忽略 |
| 平台接口 | `engine/platform/include/terra/platform/platform.hpp` | `Platform`、`Window`、`WindowDescription`、五类 `PlatformError`；`GlProcLoader` 为纯函数指针类型 |
| GLFW 实现 | `engine/platform/backends/glfw/` | `createPlatform(InputSink&)`；4.5 core、forward-compatible，可选调试上下文；版本不足或请求 < 4.5 时明确报错；GLFW 生命周期由共享指针管理（窗口可晚于平台销毁）；连接 GLFW 键盘与鼠标两个设备；键鼠回调翻译后写入 `InputSink`；指针首次进入不产生增量；失去焦点时 `resetDevice` |
| 翻译表 | `src/glfw_translation.cpp` | `translateKey`、`translateMouseButton`，纯函数 |
| 边界检查 | `cmake/TerraCheckRenderIsolation.cmake` | GLFW 头只在 GLFW 实现与测试中；OpenGL 头只在渲染、平台实现与测试中；`engine/`、`apps/` 不得直接使用 X11、Wayland、SDL、Vulkan 头 |

`terra_input` 总是构建；`terra_platform`、`terra_platform_glfw` 只在 `TERRA_BUILD_RENDER=ON` 时构建。M1-05（GLFW 手柄适配器）未包含在本次实现中。

## 9. 测试

渲染关闭 37 项（M0 的 35 项 + 2），渲染开启 41 项（+ M1-02 探针与本次 3 项）。

| 测试 | 环境 | 内容 |
| --- | --- | --- |
| `input.system` | 无图形，总是运行 | 按钮边沿与帧内快速点按；WASD 组合与相反键抵消；修饰键；同帧增量累加与下一帧清零；轴向死区 6 个数值点；径向死区（小于内径、重新归一化、长度 1、斜向保留方向）；缩放与裁剪；扳机作为按钮；热插拔（按住时断开 → 抬起与归零、旧 ID 写入被忽略、新设备自动生效、ID 不复用）；焦点丢失释放；三张动作表的优先级、消费与非消费绑定、停用后让出；三种仲裁策略（含“本地手柄接管 ROS2 指令”场景）；10 种配置错误且失败不部分注册；非有限值 |
| `boundary.render_isolation` | 无图形，总是运行 | 见第 8 节 |
| `input.glfw_translation` | 渲染开启，不需要显示 | 区间与单个键位；未知键；Terra 的 105 个键全部有对应且无冲突；鼠标键 |
| `graphics.platform_window` | Xvfb + llvmpipe | 平台连接键鼠设备；重复创建被拒绝；请求 3.3 被拒绝；**请求 4.6 返回 `ContextUnavailable`**；4.5 窗口、glad 经 `glProcLoader` 加载、渲染器为 llvmpipe、调试标志；窗口与帧缓冲尺寸；关闭请求；时钟；平台先于窗口销毁后窗口仍可用、设备已断开；全部销毁后可重建平台 |
| `graphics.platform_no_display` | 清除 `DISPLAY`，不启动 Xvfb | 返回 `NoDisplay`，不连接任何设备 |

**变异测试（脚本内执行，8 项）：**死区不重新归一化；径向死区按分量判断；断开设备不释放控件；动作表之间不消费；帧内快速点按丢失；最近变化仲裁失效；忽略修饰键；焦点丢失不释放。

真实键鼠事件无法在 Xvfb 中可靠合成，GLFW 回调只做“翻译后转发”：由翻译表测试与输入系统测试覆盖，M1-07 时在桌面会话做人工交互检查。

## 10. 验证记录

### 10.1 开发机

未编译、未运行。静态检查：脚本 `bash -n`；8 个变异锚点在源码中唯一匹配；按项目警告选项人工复查全部新代码。

### 10.2 基线机器

- **日期：**2026-11-19；命令：`bash scripts/verify/m1_03_04.sh`。
- **结果：**

| 项目 | 结果 |
| --- | --- |
| 渲染关闭 Debug | **37/37 通过** |
| 渲染开启 Debug / Release + `-Werror` | 均 **41/41 通过** |
| ASan/UBSan（渲染开启） | 非图形 38 项（含 LeakSanitizer）**全部通过**；图形 3 项（不查泄漏）**全部通过** |
| 变异测试 | **8/8 被发现**（失败检查数：死区不归一化 4、径向按分量 1、断开不释放 5、不消费 3、点按丢失 1、最近变化失效 1、忽略修饰键 1、焦点不释放 1）；源文件已恢复 |

- **测试输出要点：**
  - 10 种配置错误均被拒绝，消息定位到动作表、动作与绑定序号。
  - **请求 4.6：`ContextUnavailable`**，GLFW 错误 `0x10007`（`GLFW_VERSION_UNAVAILABLE`）“GLX: Failed to create context: GLXBadFBConfig”——确认了实现前基于 GLFW 源码的判断。
  - 4.5 上下文；`GL_RENDERER=llvmpipe (LLVM 15.0.7, 256 bits)`；窗口与帧缓冲均 320×240，缩放 1.00。
  - 无显示：`NoDisplay`。
  - 翻译表：105 个 GLFW 键码对应 Terra 的 105 个键，无冲突。
- **之后的修改：**错误消息中“第 N 个绑定 的…”多余的空格已删除（只改字符串，不影响逻辑与变异锚点；下次运行时确认）。
