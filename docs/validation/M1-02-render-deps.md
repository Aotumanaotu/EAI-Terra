# M1-02 渲染依赖源码锁定

> 状态：**已验收**（M1-11，2026-11-23 用户评审通过）。GLFW、GLM 与 glad 生成器按版本 + sha256 锁定；glad 生成的 OpenGL 4.5 core 加载代码已提交并经基线机器复核一致；新增 `TERRA_BUILD_RENDER` 选项（默认关闭）与渲染依赖探针。基线机器（2026-11-19）全部通过：渲染关闭 35/35，渲染开启 Debug 与 Release + `-Werror` 均 36/36，真实 GPU 探针通过（第 6.2 节）。

## 1. 锁定的依赖（`third_party/render/render.lock`）

| 依赖 | 版本 | 发布 | 来源 | sha256 | 用途 |
| --- | --- | --- | --- | --- | --- |
| GLFW | 3.5.1 | 2026-07-31 | `github.com/glfw/glfw` 标签源码包 | `5234f4f2…ff0c20` | 窗口、OpenGL 上下文、首个输入适配器（D-017）；静态库 |
| GLM | 1.0.3 | 2025-12-31 | `github.com/g-truc/glm` 标签源码包 | `6775e472…607001` | 渲染数学，仅头文件，只在 Render 内部使用（D-020） |
| glad | 2.0.8 | 2024-09-29 | `github.com/Dav1dde/glad` 标签源码包 | `44f06f91…2d0fc` | 生成 OpenGL 加载代码；只用于复核与重新生成 |

版本为 2026-11-19 查询 GitHub 时各项目的最新发布；sha256 在开发机下载后计算（只下载、计算，不构建）。完整值见锁定文件。

## 2. glad 生成代码（`third_party/glad/`，已提交）

- 参数：`--api gl:core=4.5 --extensions '' --reproducible c`（D-021；无扩展；不含内置加载器；不写时间戳）。
- 结果：`include/glad/gl.h`（3601 行）、`include/KHR/khrplatform.h`、`src/gl.c`（1606 行）。
- 只包含 4.5 core 的函数：误用更高版本或扩展中的函数会在编译期报错，而不是运行时才发现。
- 生成器在开发机运行（Python 代码生成，未编译任何东西），在单独目录中解压，以 `python3 -I` 运行。基线机器用 `scripts/render/gen_glad.sh` 以锁定版本重新生成并逐字节比较。

## 3. 许可核查

| 组件 | 许可 | 义务 |
| --- | --- | --- |
| GLFW | zlib | 不得冒称原作者；修改版须标明；源码分发保留声明 |
| GLM | MIT（或 Happy Bunny，二选一，采用 MIT） | 附带版权与许可声明 |
| glad 生成器 | MIT | 只在开发时使用，不随产物分发 |
| glad 生成代码 | `(WTFPL OR CC0-1.0) AND Apache-2.0`（Apache-2.0 部分来自 Khronos 规范） | 保留文件头声明；Apache-2.0 要求附带许可文本 |
| `KHR/khrplatform.h` | Khronos MIT 风格许可 | 附带版权与许可声明 |

结论：均为宽松许可，无 copyleft；与 D-109 的第三方核查结论一致。

## 4. 构建集成

| 部分 | 位置 | 说明 |
| --- | --- | --- |
| 获取 | `scripts/render/fetch.sh` | 下载、校验 sha256、解压到 `third_party/render/src/`；显式联网步骤 |
| 构建 | `scripts/render/build.sh` | GLFW 静态库，X11 + Wayland（缺 `wayland-scanner` 时可 `--no-wayland`），安装到 `third_party/render/install/`。GLFW 在运行时以 dlopen 加载 libGL 与 X11/Wayland 库 |
| 复核 | `scripts/render/gen_glad.sh` | 重新生成 glad 并逐字节比较；`--update` 覆盖 |
| CMake | `cmake/TerraRender.cmake` | 仅 `TERRA_BUILD_RENDER=ON` 时加载；按锁定版本精确查找 GLFW（`EXACT`）；提供 `terra::glfw`（带 `GLFW_INCLUDE_NONE`）、`terra::glad`、`terra::glm`，头文件以 SYSTEM 方式引入，不触发 Terra 的 `-Werror` |
| 选项 | 根 `CMakeLists.txt` | `TERRA_BUILD_RENDER` 默认 **OFF**：关闭时 M0 的 35 项测试、`terra_headless` 与物理目标完全不变（开发计划 8.1 节） |
| 预设 | `CMakePresets.json` | 新增 `render-debug`、`render-release`（开启渲染） |
| 忽略 | `.gitignore` | `third_party/render/{downloads,src,build,install}/`；`third_party/glad/` 已提交，不忽略 |

## 5. 测试

**新增 `graphics.render_deps`**（`tests/render/test_render_deps.cpp`，标签 `graphics`，仅渲染开启时注册）：

1. GLFW 初始化，创建隐藏窗口与 OpenGL 4.5 core、forward-compatible 上下文；
2. glad 加载，实际版本 ≥ 4.5、core profile；断言渲染器包含 `llvmpipe`（D-022）；
3. 用 4.5 的 DSA 函数创建离屏帧缓冲，清屏为 (0.25, 0.5, 0.75, 1)，回读像素应为 (64, 128, 191, 255) ±1，且无 GL 错误；
4. GLM：Z 向上的相机看向原点，注视点投影到中心，+Z 在上方，+Y（Terra 约定向左）在左侧。

CTest 以 D-022 的固定环境运行：`cmake -E env --unset=DISPLAY --unset=WAYLAND_DISPLAY __GLX_VENDOR_LIBRARY_NAME=mesa LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a -s "-screen 0 1280x720x24" test_render_deps llvmpipe`。

渲染开启时共 36 项（M0 的 35 项 + 1 项）；渲染关闭时仍为 35 项。

## 6. 验证记录

### 6.1 开发机

未编译、未运行（开发机不承担构建与测试）。只做了：下载源码包并计算 sha256；读取许可文件；运行 glad 生成器；所有脚本 `bash -n`；按项目警告选项人工复查探针代码。

### 6.2 基线机器

- **日期：**2026-11-19。
- **环境：**内核 6.8.0-106-generic；GCC 11.4.0；CMake 3.22.1；Ninja 1.10.1；`wayland-scanner` 已安装；python3-jinja2 3.1.6；桌面会话 X11（`DISPLAY=:0`）。
- **命令：**`bash scripts/verify/m1_02.sh`（桌面会话终端）。
- **结果：**

| 步骤 | 结果 |
| --- | --- |
| 1. 获取并校验源码 | 通过（三个源码包 sha256 与锁定值一致） |
| 2. 构建 GLFW | GLFW 3.5.1 安装成功，X11 与 Wayland 后端均已构建 |
| 3. glad 复核 | 以锁定版本重新生成，与仓库**逐字节一致** |
| 4. 渲染关闭 Debug（M0 回归） | **35/35 通过** |
| 5. 渲染开启 Debug / Release + `-Werror` | 均 **36/36 通过**；`graphics.render_deps` 分别用时 0.13 s、0.12 s（Xvfb + llvmpipe） |
| 6. 真实 GPU 探针 | 通过：GLFW 3.5.1（Wayland、X11、GLX、Null、EGL、OSMesa 后端），平台 X11；NVIDIA GeForce RTX 5060，`GL_VERSION=4.5.0 NVIDIA 595.91.07`；回读像素 (64, 127, 191, 255) |

**观察：**

- **NVIDIA 报告 4.5.0 而非 4.6。**探针请求 4.5 core 上下文，NVIDIA 按请求版本创建；M1-01 中 4.6 是不指定版本时的最高版本。符合预期。
- **像素舍入在不同驱动间相差 1。**清屏值 0.5 对应 127.5：NVIDIA 回读为 127，llvmpipe 为 128（llvmpipe 在 CTest 中以 ±1 容差通过，具体值见基线日志）。这印证了 D-022：像素级自动比对只以 llvmpipe 为基准；跨 GPU 比较必须带容差，M1-10 设计图像比对时按此处理。

## 7. 验收标准对照（开发计划 M1-02）

| 标准 | 当前 |
| --- | --- |
| 干净构建可复现 | 锁定版本与 sha256；脚本从删除构建目录开始 |
| 关闭选项时 M0 全量测试不变 | 第 6.2 节第 4 步 |
| 许可记录 | 第 3 节 |
