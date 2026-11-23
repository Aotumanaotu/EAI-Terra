# M1-14 设置与持久化

> 状态：**已验收**（M1-11，2026-11-23 用户评审通过）——基线机器验证全部通过（2026-11-19，见 6.2 节）。关联 D-023、D-025、D-026、D-029、D-030，PRD F-M1-06。

## 1. 目标

开发计划 M1-14：nlohmann/json 源码锁定；设置模型（界面、显示、相机、按键）；偏好设置窗口；JSON 读写（XDG 路径），非法内容回退默认值。验收：读写往返、缺失/损坏文件、非法字段有单元测试；设置窗口中修改相机参数与显示模式立即生效；跨次运行保留。

## 2. 依赖：nlohmann/json 3.12.0

| 项目 | 内容 |
| --- | --- |
| 版本 | 3.12.0（2025-04-11，当前最新发布） |
| 来源 | 官方发布页单头文件发布物 `json.hpp`、`json_fwd.hpp`，标签下的 `LICENSE.MIT` |
| sha256 | `json.hpp` `aaf127c0…dfc5de63`；`json_fwd.hpp` `fb6aa70c…2d028b37`；`LICENSE.MIT` `46a65cff…ae95b603`（全文见 `third_party/nlohmann_json/json.lock`） |
| 存放 | 与 glad 生成代码一样**直接提交**在 `third_party/nlohmann_json/`（约 0.95 MB），构建不联网，渲染关闭时也可用 |
| 复核 | `scripts/settings/check_json.sh`（离线校验 sha256 与版本号）；`--online` 另与官方下载比对（开发机已执行，3 个文件一致） |
| 许可 | MIT；文件内嵌片段（Hedley、Abseil 片段、Grisu2、UTF-8 解码器）SPDX 均标为 MIT，无 copyleft。分发时保留 `LICENSE.MIT`（D-109） |
| 使用约束 | CMake 目标 `terra::json`（SYSTEM 方式引入，`JSON_USE_IMPLICIT_CONVERSIONS=0`，要求显式 `get<T>()`）；只允许在 `engine/settings/src/` 与测试中包含，`boundary.render_isolation` 新增检查 |

## 3. 设计

### 3.1 模块

新增 `terra_settings`（`engine/settings/`），**总是构建**（不依赖渲染），公共头只用 Terra 类型与标准库，JSON 库只在实现中使用。它不依赖平台、视图与界面：由应用（`terra_viewer`）把设置换算成各模块参数（`apps/viewer/viewer_settings.*`）。

| 内容 | 说明 |
| --- | --- |
| `Settings` | `ui`（语言、界面缩放或自动、四个面板是否显示、停靠布局文本）、`display`（窗口/无边框/独占全屏、显示器、独占全屏的分辨率与刷新率或“桌面当前模式”、垂直同步、窗口大小）、`camera`（灵敏度、速度、加速倍率、滚轮步长、平移速度、视场角）、`keys`（9 个操作 × 主键/备用键） |
| `fromJson` / `toJson` | 带缩进、字段顺序固定、以换行结尾；同一设置序列化结果逐字节相同 |
| `load` / `save` | 见 3.3 |
| `KeyMap` | 默认按键与 M1-07/M1-13 相同；`assign` 把键分配给操作时，若该键已被其他操作占用则从那里移走（返回被移走的操作）；保留键不可分配 |
| `defaultPath` | `$XDG_CONFIG_HOME/eai-terra/settings.json`（须为绝对路径），否则 `$HOME/.config/eai-terra/settings.json` |

### 3.2 文件格式（版本 1）

```json
{
  "version": 1,
  "interface": {
    "language": "zh",
    "uiScale": "auto",
    "panels": { "outliner": true, "details": true, "log": true, "stats": true },
    "layout": "[Window][###Viewport]\n...[Docking][Data]\n..."
  },
  "display": {
    "mode": "windowed",
    "monitor": 0,
    "videoMode": null,
    "vsync": true,
    "windowSize": { "width": 1600, "height": 900 }
  },
  "camera": {
    "lookDegreesPerPixel": 0.15, "speed": 3.0, "boost": 3.0,
    "wheelStep": 0.5, "panMetersPerPixel": 0.01, "fovDegrees": 60.0
  },
  "keys": {
    "camera.forward": ["W", "Up"], "camera.back": ["S", "Down"], "camera.left": ["A", "Left"],
    "camera.right": ["D", "Right"], "camera.up": ["E", "Space"], "camera.down": ["Q", ""],
    "camera.boost": ["LeftShift", "RightShift"], "camera.reset": ["F", ""], "app.fullscreen": ["F11", ""]
  }
}
```

键名与 `terra::input::Key` 的枚举名相同（新增 `keyName` / `keyFromName`，105 个键位，名称表由枚举生成并有编译期完整性检查）。

### 3.3 读取与写入规则

| 情况 | 处理 |
| --- | --- |
| 文件不存在 | 默认设置，不算错误（首次运行）；1 秒后写出默认文件 |
| JSON 语法错误、数值溢出、顶层不是对象 | 默认设置；错误信息含行列号；**先把坏文件改名为 `settings.json.bak`** 再写新文件；改名失败时本次运行不保存，避免覆盖用户内容 |
| 存在但无法读取（是目录、无权限、超过 16 MB） | 默认设置，本次运行不保存 |
| 单个字段类型不对或超出范围 | 只有该字段用默认值，并给出警告（含字段路径、范围与默认值）；同组其他字段照常读取 |
| 整组不是对象 | 整组默认值并警告 |
| 未知字段 | 警告并忽略 |
| `version` 缺失 / 比程序新 | 警告；按版本 1 读取认识的字段 |
| 按键：未知键名、保留键（Esc、左右 Ctrl）、不是数组 | 该操作用默认按键并警告 |
| 按键：同一个键用于多个操作 | 按操作顺序保留第一次出现，后面的清空并警告 |
| 写入 | 先写 `settings.json.tmp` 再改名（原子替换）；自动创建目录；失败时原文件不变、不留临时文件 |

取值范围：界面缩放 0.5–4；灵敏度 0.01–1 °/像素；速度 0.1–100 m/s；加速倍率 1–20；滚轮步长 0.01–10 m；平移 0.0005–0.5 m/像素；视场角 20–120°；窗口 320–16384；显示器下标 0–63；刷新率 1–1000 Hz；布局文本 ≤ 1 MB。

### 3.4 查看器集成

- **三份状态**：界面直接修改 `current`；每帧开始时 `applySettings` 把 `current` 与 `applied` 的差异生效（语言、界面缩放、显示模式与垂直同步、相机参数与视场角、按键）；`current` 与 `saved` 不同且持续 1 秒后自动保存，退出时再保存一次（窗口模式下的窗口大小、当前停靠布局）。显示模式切换失败时把 `current` 改回实际状态并记日志。
- **已有入口统一走设置**：菜单语言、Ctrl+加减号/Ctrl+0 与“视图 → 界面缩放”、F11、“窗口”菜单中的面板开关都改为修改设置，因此都会被保存。新增“窗口 → 恢复默认布局”。
- **停靠布局**：ImGui 不写 `imgui.ini`；它提示布局有变化时（自带约 5 秒节流）用 `SaveIniSettingsToMemory` 把布局文本存入设置；启动时在第一次 `NewFrame` 之前读入。读入的布局没有停靠分割时建立默认布局。
- **按键改绑**：`InputSystem::setBindings`（新增）替换一个已注册动作的全部绑定，校验规则与注册时相同，失败不修改，句柄保持有效；`CameraController::setKeys`（新增，`CameraKeys` 每个操作两个槽位）。**Shift+中键平移的修饰键跟随加速键**。鼠标操作本阶段固定。
- **按键录制时**界面占用全部键盘（复用 `UiInputCapture`），相机与 F11 不响应；Ctrl+加减号等快捷键也暂停。
- **命令行**：`--settings 文件` 指定路径；`--language` 覆盖本次语言（会写入设置）；`--self-test` 从默认设置开始、不读文件、不自动保存。

### 3.5 偏好设置窗口（“设置 → 偏好设置…”，Ctrl+,）

参考 Blender 偏好设置：左侧分类列表，右侧内容，底部显示设置文件路径与“全部恢复默认”。

| 页面 | 内容 |
| --- | --- |
| 界面 | 语言；界面缩放（“自动（按显示器推算）”或 50%–400% 滑块，松开后生效，避免拖动中字号变化） |
| 显示 | 显示模式；全屏所用显示器；独占全屏的分辨率与刷新率（显示器支持的模式，去重排序）；垂直同步；独占全屏说明 |
| 相机 | 六个滑块（范围同 3.3，速度等宽范围参数用对数刻度；Ctrl+点击可直接输入） |
| 按键 | 表格：操作 / 按键 / 备用；点击后在模态弹窗中按新键（Esc 取消，Ctrl 保留），右键清除；键已被占用时从原操作移走并在日志中提示；鼠标操作说明 |

每页有“恢复本页默认值”。

## 4. 测试

| 测试 | 内容 |
| --- | --- |
| `settings.store`（新增，总是运行） | 默认值；全部字段往返且再次序列化逐字节相同；部分字段；12 种字段级错误逐一回退并给出对应警告；整组类型错误、未知字段、缺少/较新版本；语法错误（含行号）、顶层数组、空文本、数值溢出；按键（单键、null、空字符串、未知键名、保留键、非数组、未知操作、重复按键去重）；`KeyMap::assign` 抢占与换槽、保留键与非法槽位；保存（建目录、无临时文件残留）、覆盖保存、目标是目录与父路径是文件时保存失败且原文件不变；路径是目录 → Unreadable；损坏 → Corrupt → 备份为 `.bak`；XDG 路径 6 种情况 |
| `input.system`（扩充） | `setBindings`：按下中改绑产生抬起、句柄不变、清空、校验失败不修改、表或动作不存在；键位名称：全部 105 个可往返且唯一、与枚举名一致、未知/大小写不符返回 Unknown |
| `view.camera`（扩充） | `CameraKeys{}` 与控制器默认相同；改绑后新键生效、旧键与清空槽位失效；加速键改为 Alt 后 Alt+中键平移、Shift+中键变为转视角；复位键改绑；按住中改回默认；全部清空后键盘无效、中键转视角仍有效 |
| `graphics.viewer_selftest`（扩充） | 第 4–7 帧依次打开偏好设置四个页面，第 7 帧打开按键录制弹窗（全程无 GL 错误）；语言切换改为经由设置；结束时检查：设置默认按键/相机参数与控制器默认值一致；修改设置后相机速度、视场角、界面缩放、按键立即生效；虚拟键盘按 I 1 秒前进 7 m、W 不再移动；全屏键改为 F10 后 F10 触发、F11 不触发；保存到构建目录再读回相同且含停靠布局（不触碰 `~/.config`） |
| `boundary.render_isolation`（扩充） | `nlohmann/` 只允许在 `engine/settings/src/` 与 `tests/` |

测试总数：渲染关闭 37 → **38**；渲染开启 47 → **48**。

**变异测试（6 项，`scripts/verify/m1_14.sh`）**：超出范围的值没有回退默认值；语法错误被当作读取成功；重复按键没有去重；`XDG_CONFIG_HOME` 被忽略（→ `settings.store`）；替换绑定不生效（→ `input.system`）；平移修饰键不跟随加速键（→ `view.camera`）。

验证脚本另含真实 GPU 自检，以及**两次可交互展示**：第一次在全新设置文件下修改设置后关闭，脚本打印保存的要点；第二次用同一文件打开，确认改动保留。展示用 `build/verify-m1-14/show/settings.json`，不影响正常启动时使用的 `~/.config/eai-terra/settings.json`。

## 5. 已知限制

- 鼠标操作（中键转视角、滚轮）暂不可改；手柄绑定随 M1-05。
- 不保存窗口位置与相机位置；显示器热插拔后，设置中不存在的显示器按主显示器处理。
- 从设置切换到独占全屏会改变显示器分辨率（用户主动操作）；启动时按设置进入全屏，失败时退回窗口并记日志。
- 首次运行会在 1 秒后写出默认设置文件。

## 6. 验证记录

### 6.1 开发机

未编译、未运行（按约定）。已做：nlohmann/json 三个文件与官方下载 sha256 一致；新增界面文字的全部字符在两个字体子集中都有字形（fontTools 检查 307 个字符）；ImGui 1.93 接口逐一对照头文件（`BeginChild` 的 `ImGuiChildFlags_Borders`、`SliderScalar`、`BeginPopupModal`、`SaveIniSettingsToMemory`/`LoadIniSettingsFromMemory`、`DockBuilderGetNode`/`IsLeafNode`、`ImGuiKey_Comma` 等；避开已废弃的 `GetContentRegionMax`）；验证脚本 `bash -n`；M1-14 的 6 个变异锚点唯一匹配；M1-03/04、M1-07、M1-13 的旧变异锚点复查（M1-07 的“方向键未绑定”因按键改由槽位生成而改写锚点）。

### 6.2 基线机器（2026-11-19，第一次运行全部通过）

`bash scripts/verify/m1_14.sh`：

| 项目 | 结果 |
| --- | --- |
| nlohmann/json 校验 | 3/3 个文件 sha256 一致 |
| 渲染关闭 Debug | **38/38 通过** |
| 渲染开启 Debug / Release + `-Werror` | 均 **48/48 通过**，无警告 |
| ASan/UBSan | 非图形 42 项（含 LeakSanitizer）、图形 6 项全部通过 |
| 变异测试 | **6/6 被发现**（超出范围 4、语法错误 1、重复按键 3、XDG 2、替换绑定 3、平移修饰键 2 项检查失败）；源文件已恢复 |
| Xvfb 自检（llvmpipe） | 全部 PASS，含新增 12 项设置检查：默认值一致、速度/视场角/缩放/按键立即生效、改绑后 I 前进 7 m 且 W 无效、全屏键改为 F10、保存并读回相同（含 1375 字节停靠布局） |
| 真实 GPU 自检（RTX 5060） | 全部 PASS（同上），无 GL 错误 |
| ui.text | 两个字体各检查 3451 个字符，无缺失字形 |
| 可交互展示（第一次） | 全新设置文件；120 秒、7198 帧；退出时保存。本次展示中只改变了窗口大小（2365×1378）与停靠布局（1316 字节），语言、缩放、相机参数与按键保持默认 |
| 可交互展示（第二次） | 读取上次的设置（“已读取”），45.4 秒后用户关闭，退出码 0；用户确认 |

说明：语言、缩放、相机参数与按键的跨次保留由自检的“保存 → 读回相同”与 `settings.store` 往返测试覆盖；两次展示实际验证的是窗口大小与停靠布局的保留。
