# M1-06 渲染核心

> 状态：**已验收**（M1-11，2026-11-23 用户评审通过）。`terra_render`：上下文初始化与调试输出、GL 资源 RAII（DSA）、着色器错误定位、基本绘制状态；测试 `graphics.render_core` 与桌面展示模式。基线机器（2026-11-19）全部通过：渲染关闭 37/37，渲染开启 Debug 与 Release + `-Werror` 42/42，ASan 全部通过，变异 5/5，NVIDIA 上的可见展示 60 FPS（第 4.2 节）。

## 1. 范围（开发计划 M1-06）

目标：`engine/render/` 的 GL 资源 RAII（缓冲、着色器、顶点数组）、着色器编译与链接诊断、清屏与深度测试。验收要点：着色器错误带文件与行号；资源释放无泄漏（ASan）。另按用户要求（2026-11-19）提供可见窗口展示。

不在本任务：相机与控制器（M1-07）、网格/材质/光照（M1-08）、资产加载（M1-09）、图像基准与帧耗时（M1-10）。

## 2. 接口（`engine/render/include/terra/render/`）

| 头文件 | 内容 |
| --- | --- |
| `context.hpp` | `RenderContext::create(GlLoader)`：加载 OpenGL 函数，确认 ≥ 4.5 core；调试上下文时开启**同步**调试输出并收集驱动消息（丢弃通知级）；`GlInfo`；`takeDebugMessages()`、`errorMessageCount()`；`setViewport`、`setDepthTest`（LESS + 写深度）、`clear`（清深度前确保深度写入开启）、`drawTriangles` |
| `resources.hpp` | `Buffer`（不可变存储，可选动态更新，越界拒绝）、`VertexArray`（DSA 顶点格式）、`ShaderProgram`（恰好一个顶点 + 一个片元着色器；`ShaderError` 区分编译/链接/输入无效，带名称、行号、驱动原始日志与单行摘要）、`Framebuffer`（RGBA8 颜色纹理 + 可选 DEPTH24_STENCIL8；不完整时报状态码；像素回读）；`parseShaderErrorLine`（Mesa 与 NVIDIA 两种日志格式） |

约束：

- 公共头**不包含任何 OpenGL、窗口库或 MuJoCo 头文件**，GL 对象以 `std::uint32_t` 表示；glad 只 PRIVATE 链接。
- `terra_render` **不依赖平台层与物理层**：上下文由调用方创建，只通过 `GlLoader` 函数指针（与 `terra::platform::GlProcLoader` 同一类型）交接。
- 全部资源用 OpenGL 4.5 的 DSA 函数创建与修改；只能移动不能复制；析构时删除；被移走的对象名称为 0。

## 3. 测试

**`graphics.render_core`**（Xvfb + llvmpipe，`graphics` 标签；渲染开启时共 42 项）：

| 用例 | 内容 |
| --- | --- |
| 上下文 | 4.5、调试上下文、渲染器为 llvmpipe；空加载器被拒绝 |
| 离屏绘制 | 64×64 帧缓冲；尺寸 0 被拒绝；uniform 查找（不存在时 -1） |
| 深度测试 | 先画近处蓝色、再画远处红色：开启深度测试时中心为蓝色；只被红色覆盖处为红色；角落为清屏颜色 (26, 51, 77)；关闭深度测试时中心变为红色 |
| 着色器错误 | 第 4 行使用未定义变量：`Compile`、名称 `broken.frag`、**行号 4**；顶点无输出而片元需要输入：`Link`；两个顶点着色器：`InvalidInput`；Mesa 与 NVIDIA 日志格式的行号解析；警告不当作错误 |
| 调试输出 | 注入一个 `GL_INVALID_ENUM`，同步回调捕获且错误计数恰好加 1；结束时没有其他 GL 错误 |
| RAII | 1000 个缓冲在作用域结束时被删除（`glIsBuffer` 为假）；移动后原对象为空；动态/静态缓冲的更新规则；零大小返回空缓冲 |

**可见展示**（用户要求，2026-11-19）：`test_render_core --show <秒>` 在桌面会话用真实 GPU 打开 960×600 可见窗口：深色背景，一个旋转的渐变三角形，前方一个静止的蓝色三角形（深度测试），到时自动关闭；**Esc 经 Terra 输入系统的动作映射**提前关闭——这是平台、输入、渲染三层第一次一起运行。结束时打印渲染器、帧数与平均 FPS。不在 CTest 中运行。

**变异测试（脚本内执行，5 项）：**深度测试未开启；帧缓冲没有深度附件；清屏不清深度；着色器行号解析失效；调试回调未安装。

## 4. 验证记录

### 4.1 开发机

未编译、未运行。静态检查：脚本 `bash -n`；5 个变异锚点在源码中唯一匹配；按项目警告选项人工复查。

### 4.2 基线机器

- **日期：**2026-11-19；命令：`bash scripts/verify/m1_06.sh`（桌面会话）。
- **结果：**

| 项目 | 结果 |
| --- | --- |
| 渲染关闭 Debug | **37/37 通过** |
| 渲染开启 Debug / Release + `-Werror` | 均 **42/42 通过** |
| ASan/UBSan | 非图形 38 项（含 LeakSanitizer）、图形 4 项（不查泄漏）**全部通过** |
| 变异测试 | **5/5 被发现**（失败检查数：深度未开启 1、无深度附件 1、清屏不清深度 2、行号解析失效 3、回调未安装 2）；源文件已恢复 |
| 可见展示 | NVIDIA GeForce RTX 5060，OpenGL 4.5.0 NVIDIA 595.91.07；4.0 秒 242 帧，**平均 60 FPS**（垂直同步）；退出码 0 |

- **测试输出：**
  - llvmpipe，OpenGL 4.5，GLSL 4.50，调试上下文。
  - 深度开启：中心 (0, 0, 255) 蓝色，红色区域 (255, 0, 0)，角落 **(26, 51, 76)**——清屏值 0.3 对应 76.5，llvmpipe 舍入为 76（±1 容差内；与 M1-02 中 0.5 → 127/128 的舍入差异同类）。
  - 编译错误：`broken.frag:4: 编译失败：0:4(26): error: `undefinedVariable' undeclared`——行号定位正确。
  - 链接错误：`empty.vert + needs.frag: 链接失败：error: fragment shader input `vMissing' has no matching output in the previous stage`。
  - 调试输出：`id=3 GL_INVALID_ENUM in glBindBufferARB(target 0x1234)`，同步捕获。
- **之后的修改：**验证脚本中“可见展示”一行重复打印，已修正（只影响屏幕输出）。

## 5. 验收标准对照（开发计划 M1-06）

| 标准 | 当前 |
| --- | --- |
| 着色器错误带文件与行号 | `ShaderError` 名称 + 解析行号 + 驱动原文；测试断言行号 4 |
| 资源释放无泄漏（ASan） | RAII 测试；ASan 非图形测试含泄漏检测（图形测试因系统库误报关闭泄漏检测，见脚本说明） |
