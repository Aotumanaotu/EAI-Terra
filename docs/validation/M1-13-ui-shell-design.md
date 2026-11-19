# M1-13 编辑器式界面外壳：设计

> 状态：**设计待评审，未实现**。依据：D-018（输入独立抽象）、D-024（Dear ImGui docking，不用其 GLFW 后端）、D-025（显示模式）、D-028（Noto Sans SC 子集字体）、D-029（中英文界面）、PRD F-M1-05。依赖已锁定：Dear ImGui 1.93.0-docking（`render.lock`）；字体已生成（`assets/fonts/`）。

## 1. 目标观感

参考 Blender、UE5 与游戏：深色主题；顶部菜单栏；中间是 3D 视口；左侧“大纲”、右侧“属性”、底部“日志/统计”面板，均可拖动停靠、合并为标签页；最下方状态栏（帧率、GPU、显示模式、相机位置）；F11 全屏；字号按显示器缩放（基线 1.25）。

## 2. 模块

| 目标 | 位置 | 内容 | 依赖 |
| --- | --- | --- | --- |
| `terra_ui` | `engine/ui/` | ImGui 上下文、Terra 输入桥、ImGui 的 OpenGL 渲染后端（官方 `imgui_impl_opengl3`，改用我们的 glad 加载函数）、主题、字体与缩放、文本目录（中/英） | ImGui（源码编入）、`terra_input`、`terra_platform` 接口、glad（PRIVATE） |
| `terra_viewer` | `apps/viewer/` | 第一个真正的应用：菜单栏、停靠布局、视口（场景先渲染到离屏帧缓冲，再作为图像显示在视口面板里）、面板、状态栏 | `terra_ui`、`terra_render`、`terra_view`、`terra_platform_glfw` |

ImGui 是**应用层的界面 API**（D-024）：`terra_ui` 与 `apps/viewer` 可以使用 ImGui 头文件；`engine/` 下其他模块（core、physics、input、view、render 的公共头）不得包含 ImGui——边界检查扩展覆盖。OpenGL 头文件新增允许 `engine/ui/src/`（ImGui 渲染后端）。

## 3. 输入：ImGui 由 Terra 喂事件（D-018、D-024）

```text
GLFW 回调 ──▶ UiInputBridge（实现 InputSink）──┬─▶ InputSystem（动作、相机控制器）
                                               └─▶ 事件队列 ──(每帧开始)──▶ ImGui IO
```

- `UiInputBridge` 是一个“分流”写入端：平台写给它的每个事件，原样转发给 `InputSystem`，同时记录一份，在每帧开始时按顺序交给 ImGui（键、鼠标位置、按键、滚轮、焦点）。ImGui 看不到 GLFW。
- **文本输入**：`InputSink` 新增 `addText(char32_t)`（默认空实现），GLFW 的字符回调写入；ImGui 的输入框从这里拿字符。
- **界面与视口抢输入**：注册一个最高优先级的动作表 `ui_capture`，绑定全部鼠标控件（以及键盘）并设为消费。每帧根据 ImGui 的 `WantCaptureMouse` / `WantCaptureKeyboard` 启用或停用它：鼠标在面板上时，相机控制器拿不到鼠标；**鼠标在视口面板里时不启用**，相机照常工作；输入框获得焦点时键盘被界面占用，按 W 不会让相机前进。直接复用 M1-04 的动作表优先级与消费机制。

## 4. 字体、缩放与主题

- 加载 `NotoSansSC-Regular-Subset.otf`（正文）与 `-Bold-`（标题）；基准字号 16 px × 显示器内容缩放；控件尺寸同样按缩放放大。字体文件缺失时回退到 ImGui 内置字体（只有英文）并给出诊断，不阻止启动。
- 主题：在 ImGui 深色主题基础上调整为接近 Blender/UE5 的灰阶与蓝色强调色，小圆角，统一间距；颜色集中定义，便于以后在设置里改。

## 5. 中英文（D-029）

- 文本目录：`TextId` 枚举 → 中文与英文两张表；界面代码只写 `tr(TextId::MenuFile)`，不写死字符串。
- 切换语言即时生效（M1-13 先放在菜单“设置 → 语言”，M1-14 移入设置窗口并持久化）。
- 测试保证两张表覆盖全部 `TextId` 且没有空字符串。

## 6. 视口

- 场景（网格、坐标轴、立方体）渲染到离屏帧缓冲（M1-06 的 `Framebuffer`），尺寸跟随视口面板大小与缩放；面板中以图像显示，宽高比正确。
- 视口面板获得焦点或鼠标悬停时，相机控制器生效（第 3 节）。

## 7. 测试计划

| 测试 | 环境 | 内容 |
| --- | --- | --- |
| `ui.input_bridge` | 无图形 | 分流：每个事件同时到达 InputSystem 与 ImGui 队列且顺序不变；Terra `Key` → ImGui 键的映射覆盖全部 105 个键；文本字符；`ui_capture` 启用时相机动作拿不到鼠标、停用时恢复 |
| `ui.text` | 无图形 | 中英两张表覆盖全部 `TextId`、无空字符串；字体文件存在、sha256 与记录一致、包含界面用到的全部汉字 |
| `graphics.ui_render` | Xvfb + llvmpipe | 离屏渲染一帧完整外壳：菜单栏区域、面板区域、视口区域像素符合预期；中文字形实际被光栅化（字形查找成功） |
| 可交互展示 | 桌面 | `terra_viewer` 最长 60 秒：拖动面板停靠、切换中英文、F11、在视口里操作相机、鼠标在面板上时相机不动 |

## 8. 待评审

1. 模块划分：`terra_ui` + 第一个应用 `terra_viewer`；ImGui 作为应用层界面 API，边界检查约束其使用范围（第 2 节）。
2. 输入分流 `UiInputBridge` 与 `ui_capture` 动作表（第 3 节）。
3. 默认布局：左大纲、右属性、下日志/统计、中视口、底部状态栏（第 1 节）；面板内容 M1 先放占位与统计信息，场景数据在 M2 接入。
