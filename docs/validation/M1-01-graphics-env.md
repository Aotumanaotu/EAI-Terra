# M1-01 图形环境与 OpenGL 最低版本

> 状态：**已验收**（M1-11，2026-11-23 用户评审通过）。两次采集（2026-11-19）确认：桌面默认、NVIDIA PRIME 与 Xvfb + llvmpipe 三种情况的 OpenGL core 版本均 ≥ 4.5。按第 3 节的判定标准，**OpenGL 最低版本定为 4.5 core**（D-021），**自动化图形测试采用 Xvfb + Mesa llvmpipe**（D-022）。

## 1. 目的

开发计划 M1-01：在基线机器上确定 OpenGL 最低版本与 profile，以及自动化图形测试的方式，作为 M1-02 起所有渲染工作的前提（开发计划第 5、8 节）。

## 2. 第一次采集（2026-11-19，`scripts/gfx_inventory.sh`）

| 项目 | 结果 |
| --- | --- |
| 会话 | X11，GNOME（`XDG_SESSION_TYPE=x11`，`DISPLAY=:0`） |
| 显示器 | 1 台，DP-2，3840×2160 @ 60 Hz |
| GPU 1 | Intel 核显 RPL-S（PCI `8086:a780`），驱动 i915 |
| GPU 2 | NVIDIA GeForce RTX 5060（PCI `10de:2d05`），NVIDIA 开源内核模块 595.91.07 |
| Mesa | 23.2.1（`libgl1-mesa-dri`、`libegl-mesa0`），LLVM 15.0.7 |
| Vulkan（仅记录） | NVIDIA 1.4.329；Intel 与 llvmpipe 1.3.255 |
| OpenGL | **未采集**：`glxinfo` 未安装 |
| 已安装开发包 | `libgl-dev`、`libegl-dev`、`libopengl-dev`、`libglx-dev`、X11/XRandR/Xinerama/Xcursor/XInput、`libwayland-dev`、`libxkbcommon-dev`、`libglfw3-dev` 3.3.6、`libsdl2-dev` 2.0.20、`xvfb` |
| 未安装 | `libglm-dev`（按 D-019/D-020 走源码锁定，不需要）、`mesa-utils` |

**观察：**

- **双显卡。**桌面实际由哪块 GPU 渲染，取决于显示器接在哪块卡上以及 PRIME 配置；必须用 `glxinfo` 确认。这会影响手动运行渲染程序时的性能与行为，也是“图形问题只在某块卡上复现”时的排查前提。
- **GLFW 构建依赖已齐。**X11 与 Wayland 后端所需的开发包都已安装，源码构建 GLFW（D-019）不需要再装系统包。apt 中的 GLFW 3.3.6 不会被使用。
- **Xvfb 已安装，Mesa 带 llvmpipe。**这是无显示器自动化图形测试的现成路径。
- 第一次采集时脚本的驱动包列表把“已知但未安装”的包也列了出来（版本为空），已修正为只列已安装的包。

## 3. 初步推荐（待第二次采集确认）

| 事项 | 推荐 | 理由 |
| --- | --- | --- |
| OpenGL 最低版本 | **4.5 core profile** | 三个可能的驱动都支持 4.5 core：NVIDIA 595（4.6）、Intel/Mesa 23.2 iris（4.6）、Mesa 23.2 llvmpipe（4.5）。4.5 提供直接状态访问（DSA）、`glClipControl` 与调试输出（`KHR_debug` 进入核心），能让 RAII 封装更简单、错误更易诊断。选 4.6 会把 llvmpipe 排除在自动化测试之外 |
| 自动化图形测试 | **Xvfb + Mesa llvmpipe**（离屏帧缓冲 + 像素回读） | 不需要显示器和真实 GPU，结果在同一 Mesa 版本下稳定，可以无人值守地在基线机器运行；作为单独的 CTest 标签，不阻塞无图形测试（NFR-06） |
| 真实 GPU 检查 | 桌面会话手动运行 + 帧耗时基线（F-M1-03） | 画质与性能只能在真实 GPU 上评估；默认 GPU 与 NVIDIA PRIME 两种情况都记录 |

需要第二次采集确认的三件事：

1. 桌面默认 GPU 的 OpenGL core 版本 ≥ 4.5；
2. NVIDIA（PRIME offload）的 OpenGL core 版本 ≥ 4.5；
3. Xvfb + llvmpipe 的 OpenGL core 版本 ≥ 4.5。

任一不满足时退回 3.3 core，并在报告中说明放弃 DSA 的影响。

## 4. 第二次采集（2026-11-19，安装 `mesa-utils` 8.4.0 后）

| 情况 | 渲染器 | OpenGL core | GLSL | 说明 |
| --- | --- | --- | --- | --- |
| 1) 桌面会话默认 | NVIDIA GeForce RTX 5060 | **4.6.0**（NVIDIA 595.91.07） | 4.60 | 直接渲染。**桌面本身运行在 NVIDIA 上**，手动运行渲染程序默认使用 NVIDIA |
| 2) NVIDIA PRIME offload | NVIDIA GeForce RTX 5060 | **4.6.0** | 4.60 | 与默认相同 |
| 3) Xvfb + Mesa 软件渲染 | llvmpipe（LLVM 15.0.7，256 bits） | **4.5**（Mesa 23.2.1） | 4.50 | 非硬件加速；最高 core 版本 4.5 |

**EGL（仅记录，供 M4 离屏相机参考）：**NVIDIA EGL 1.5，支持 X11、Wayland、GBM、device 平台，并提供 `EGL_EXT_platform_device`、`EGL_MESA_platform_surfaceless` 等无窗口扩展。

**其他：**Intel 核显没有出现在任何 OpenGL 查询中（显示器接在 NVIDIA 上），M1 不以它为目标；如需覆盖，可在后续用 `DRI_PRIME` 等方式单独采集。

## 5. 结论

1. **OpenGL 最低版本：4.5 core profile**（D-021）。三种情况均满足；4.5 的直接状态访问（DSA）、`glClipControl`、核心调试输出可直接使用。不使用 4.6 独有功能（如 SPIR-V 着色器），以保持 llvmpipe 可用。上下文创建时显式请求 4.5 core、forward-compatible；版本不足时 M1-03 必须明确报错。
2. **自动化图形测试：Xvfb + Mesa llvmpipe**（D-022）。离屏帧缓冲渲染并回读像素；作为单独的 CTest 标签，不阻塞无图形测试。**必须显式固定环境**，否则 Xvfb 下的 GLVND 可能选中 NVIDIA 的 GLX 库：

   ```bash
   env -u DISPLAY -u WAYLAND_DISPLAY __GLX_VENDOR_LIBRARY_NAME=mesa LIBGL_ALWAYS_SOFTWARE=1 \
       xvfb-run -a -s "-screen 0 1280x720x24" <图形测试>
   ```

   测试运行时应断言渲染器字符串包含 `llvmpipe`，防止环境悄悄变化导致像素基准失效。
3. **真实 GPU：**NVIDIA RTX 5060（OpenGL 4.6）用于手动运行与 F-M1-03 帧耗时基线；像素级自动比对只以 llvmpipe 为准。

## 6. 验收标准对照（开发计划 M1-01）

| 标准 | 当前 |
| --- | --- |
| 报告记录 GPU、驱动、Mesa、GL 版本与选择理由 | 第 2、4、5 节 |
| 确定 OpenGL 最低版本与 profile | 4.5 core（D-021） |
| 确定自动化图形测试方式 | Xvfb + llvmpipe，固定环境变量（D-022） |
