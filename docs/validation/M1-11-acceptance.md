# M1-11 阶段验收

> 状态：**已验收**。2026-11-23，基线机器在干净提交 `e5900c8` 上运行 `scripts/verify/m1_acceptance.sh`，全部必需项通过（第 11.2 节）；同日用户评审通过。按开发计划第 5 节核对的 M1 完成标准全部满足（第 7 节），PRD 的 M1 需求见第 8 节。**M1 阶段完成。**

## 1. 范围（开发计划 M1-11）

沿用 [M0-09](M0-09-acceptance.md) 的做法：

| 要求 | 交付 |
| --- | --- |
| 一条命令复现全部证据 | `scripts/verify/m1_acceptance.sh`（第 2 节） |
| 干净构建、无显示环境 | 各构建目录先删除再重建；自动化图形测试在 Xvfb + llvmpipe 中运行，并清除 `DISPLAY`（D-022） |
| 回归 | M0 统一验收原样重跑；M1 历次变异测试全部重跑（锚点先静态预检） |
| 重复运行 | 渲染开启的 Release 全量 CTest 每项至多 3 次 |
| 依赖与公共类型审查 | `ldd` 检查加源码检查（第 6 节） |
| 图像与帧耗时基线 | 参考图逐像素比较；真实 GPU 上帧耗时 5 组，只记录（第 5 节） |
| 逐条核对完成标准 | 第 7、8 节 |
| 开发记录 | LOG-072（准备）；运行结果另记 |

为本次验收新增或修改了以下内容：
- **`scripts/verify/check_mutation_anchors.py`**：不编译，检查所有验证脚本的变异锚点在当前源码中只出现一次。源码改动后，旧锚点可能已经失效；验收脚本先跑这项检查，免得几十分钟构建之后才报“注入失败”。开发机运行结果：**71/71 唯一**（M0 11 个，M1 60 个）。
- **10 个 M1 验证脚本新增 `--mutation-only`**：只做变异测试。不删除、不重建 `build/` 下的预设构建目录，并跳过 ASan 与真实 GPU 步骤，这样验收时不必为每个历史脚本再做三次全量构建。
- **订正过时的状态行**：M1-13（反馈修订后已重跑通过）、M1-10（帧耗时已补录）、D-013、D-019、D-029。

## 2. 从零复现 M1

在 Ubuntu 22.04 / x86-64 的桌面会话中运行：

```bash
sudo apt install build-essential cmake ninja-build python3 git curl xvfb \
                 libgl-dev libegl-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev \
                 libwayland-dev libwayland-bin libxkbcommon-dev   # 仅首次（图形开发包见 M1-01 报告，wayland-scanner 见 scripts/render/build.sh）
scripts/mujoco/fetch.sh && scripts/mujoco/build.sh      # M0：MuJoCo 4.0.1
scripts/render/fetch.sh && scripts/render/build.sh      # GLFW 3.5.1、GLM 1.0.3、glad 2.0.8、Dear ImGui 1.93.0-docking
scripts/assets/fetch_hdri.sh && scripts/assets/fetch_models.sh   # Poly Haven 的 2 张 HDRI 与 3 个 glTF 资产（CC0）
bash scripts/verify/m1_acceptance.sh
```

nlohmann/json、stb、cgltf 与 glad 生成代码直接提交在 `third_party/` 中，验收时按 sha256 离线核对。

验收脚本的步骤：

| 步骤 | 内容 | 判定 |
| --- | --- | --- |
| 1 | 提交号与未提交改动 | 记录 |
| 2 | 环境清单（工具链与 dpkg 来源）、图形环境清单（GPU、驱动、OpenGL） | 记录 |
| 3 | 变异锚点预检 | 必须 71/71；否则立即停止 |
| 4 | `m0_acceptance.sh`：渲染关闭的干净构建，Debug、Release、ASan 全量测试，M0 变异 11 个，重复运行，ldd | 退出码 0 |
| 5 | `m1_15.sh --skip-show`：依赖与素材 sha256；渲染关闭 Debug 39 项；渲染开启 Debug 与 Release + `-Werror` 57 项；ASan；M1-15 变异 5 个 | 退出码 0 |
| 6 | 10 个历史脚本 `--mutation-only`：M1-03/04、06、07、12、13、14、08a、08b、09、10，共 55 个变异 | 每个脚本全部发现 |
| 7 | 渲染开启 Release 全量测试，每项至多 3 次 | 全部通过 |
| 8 | 依赖边界（第 6 节） | 全部满足 |
| 9 | 真实 GPU：查看器 40 帧自检（必须通过）；帧耗时 5 组（只记录） | 自检退出码 0 |
| 10 | 可交互总览，最长 180 秒 | 用户确认 |

日志在 `build/verify-m1/logs/`，各子脚本另有自己的日志目录。实际用时约 7 分半（基线机器 24 个逻辑核；准备时估计的 1–2 小时偏保守）。

## 3. 依赖锁定与许可

| 组件 | 版本 / 锁定 | 许可 | 记录 |
| --- | --- | --- | --- |
| GLFW | 3.5.1，sha256（`third_party/render/render.lock`） | zlib | [M1-02](M1-02-render-deps.md) 第 3 节 |
| GLM | 1.0.3，同上 | MIT | 同上 |
| glad | 2.0.8 生成器；生成代码已提交，可用 `gen_glad.sh` 逐字节复核 | (WTFPL OR CC0-1.0) AND Apache-2.0 | 同上 |
| Dear ImGui | 1.93.0-docking，同上 | MIT | [M1-13](M1-13-ui-shell-design.md) |
| Noto Sans SC 子集（界面字体） | `scripts/fonts/make_ui_font.py` 生成 | SIL OFL 1.1 | M1-13、D-028 |
| nlohmann/json | 3.12.0，3 个文件 sha256 | MIT | [M1-14](M1-14-settings.md) |
| stb_image / stb_image_write | 2.32 / 1.18，提交 `9f755b60…`，3 个文件 sha256 | MIT 或公有领域 | [M1-08 设计](M1-08-materials-lighting-design.md) |
| cgltf | 1.15，提交 `360db1a9…`，2 个文件 sha256 | MIT | [M1-09 设计](M1-09-assets-design.md) |
| HDRI ×2 | Poly Haven 2K `.hdr`，`assets/hdri/hdri.lock` | CC0 | M1-08 设计第 8 节 |
| glTF 资产 ×3 | Poly Haven 2K（metal_toolbox、Drill_01、brass_vase_01），15 个文件，`assets/models/models.lock` | CC0 | M1-09 设计 |

全部是宽松许可，没有 copyleft。CMake 不会自动联网下载，获取都通过显式脚本完成（D-019）。项目自身的许可证（D-109）仍待用户决定。

## 4. 测试清单

渲染关闭时 **39 项**：M0 的 35 项，加 `input.system`、`boundary.render_isolation`、`settings.store`、`core.system_monitor`。

渲染开启时 **57 项**，在 39 项基础上新增 18 项：

| 模块 | 测试 |
| --- | --- |
| 依赖与平台 | `graphics.render_deps`、`input.glfw_translation`、`graphics.platform_window`、`graphics.platform_no_display`、`graphics.platform_display` |
| 渲染核心与 PBR | `graphics.render_core`、`render.math`、`graphics.render_pbr` |
| 资产 | `asset.gltf` |
| 参考图与基准 | `render.image_compare`、`graphics.render_reference.{procedural,studio,outdoor}`、`graphics.render_bench_smoke` |
| 相机与界面 | `view.camera`、`ui.input_bridge`、`ui.text`、`graphics.viewer_selftest` |

其中带 `graphics` 标签的 11 项需要 OpenGL 上下文，可以用 `ctest -L graphics` 单独运行，也可以用 `-LE graphics` 排除，不影响无图形测试（NFR-06）。

**变异测试共 71 个**，每个都在各自的基线运行中被测试发现：

| 来源 | 数量 | 覆盖 |
| --- | --- | --- |
| M0-07 / M0-08 | 8 + 3 | 物理接口、场景判定 |
| M1-03/04 | 8 | 死区、热插拔、动作表消费、仲裁、修饰键、焦点 |
| M1-06 | 5 | 渲染核心 |
| M1-07 | 5 | 相机控制器 |
| M1-12 | 3 | 显示模式 |
| M1-13 | 5 | 界面输入桥接、文本 |
| M1-14 | 6 | 设置、按键绑定、相机参数 |
| M1-08a / 08b | 6 + 6 | 着色公式、色调映射、阴影、MSAA；辐照度、立方体面、等距柱状映射、环境旋转、背景深度、DFG |
| M1-09 | 6 | Y→Z 换算、sRGB、法线贴图绿色通道、遮蔽范围、切线手性、太阳去除 |
| M1-10 | 5 | 比较规则、差异图、回读行序、计时等待、ARM 约定 |
| M1-15 | 5 | /proc 解析、滚动序列、软件渲染器显存、图表上限 |

## 5. 图像与帧耗时基线

**参考图**（M1-10）：`tests/render/reference/{procedural,studio,outdoor}.png`，640 × 360，默认画质（AgX、4× MSAA、阴影、网格）。参考图在基线机器的 llvmpipe（Mesa 23.2.1，LLVM 15.0.7）上生成，并经用户目视确认。

判定规则：每通道容差 3，超差像素不超过 0.1%，最大差不超过 16。M1-10、M1-15 的历次运行中，三张参考图均**逐像素一致**（不同像素 0，最大差 0）。在 NVIDIA 上渲染与参考图约有 0.4–0.5% 的像素不同，只作记录（像素基准以 llvmpipe 为准，D-022）。

**帧耗时**（RTX 5060，驱动 595.91.07；离屏渲染，不经过垂直同步；每组 600 帧，预热 60 帧；室外场景含三个资产）：

| 配置 | 整帧中位数 | GPU 中位数 |
| --- | --- | --- |
| 1080p 默认画质 | 0.47 ms | 0.38 ms |
| 4K 默认画质 | 1.58 ms | 1.30 ms |
| 4K 无 MSAA | — | 0.80 ms |
| 4K 无阴影 | — | 1.11 ms |
| 4K 程序场景 | 1.36 ms | 1.05 ms |

完整表格（含 p95 与分阶段耗时）见 [M1-10 报告](M1-10-render-baseline-design.md)。验收运行又测了一次（第 11.2 节），结果与上表一致或略好，只记录不判定。这些数值远低于 PRD NFR-03 的候选目标（基础画面 60 FPS，即 16.7 ms），但只代表这几个小场景，不构成性能承诺。

## 6. 依赖边界审查

| 检查 | 方式 | 结果 |
| --- | --- | --- |
| 公共头不含 MuJoCo 头文件 | `boundary.no_mujoco_in_public_headers`：扫描 `engine/**/include/`、`apps/` 等 | 通过 |
| 窗口库与图形 API 只出现在允许的位置 | `boundary.render_isolation`：GLFW、glad/GL、ImGui、JSON、stb、cgltf、X11/Wayland/Vulkan 各有允许目录；输入模块与平台接口层不得包含 | 通过 |
| Headless 不链接图形库 | `headless.no_graphics_libs`，并由验收脚本对 `terra_headless`、`terra_bench` 再做一次 ldd | 通过，无图形库 |
| 渲染程序不链接 MuJoCo | 验收脚本对 `terra_viewer`、`terra_render_bench` 做 ldd | 通过，不链接 libmujoco |
| 渲染侧不引用物理模块 | 在 render、asset、ui、view、platform、input、settings、viewer、bench_render 的 CMake 与源码中搜索 `terra::physics`、`terra_physics`、`terra/physics/`、`mujoco` | **0 处**（开发机与基线机器） |

渲染侧唯一使用的 M0 模块是 `terra_data` 的几何类型（`terra/view/camera.hpp` 包含 `terra/data/geometry.hpp`）。这是与后端无关的数据契约，不含 MuJoCo 类型，允许使用。

## 7. M1 完成标准核对（开发计划第 5 节）

| 完成标准 | 证据 | 状态 |
| --- | --- | --- |
| PRD 的 M1 P0 通过 | 第 8 节逐条核对 | 满足（统一运行全部通过） |
| 无 MuJoCo 类型进入渲染接口 | 第 6 节：公共头检查、ldd、源码搜索；渲染公共接口只用 Terra 类型（`terra::render::Vec3f` 等），GLM 只在渲染实现内部（D-020） | 满足（第 8 步全部通过） |
| 可独立显示测试资产 | M1-09：三个 glTF 资产在查看器中显示（真实 GPU 自检与参考图 `studio`、`outdoor`）；资产缺失时给出路径与原因（`asset.gltf`） | 满足（LOG-062） |
| Headless 仍可单独构建运行 | 渲染关闭 39/39，与 M0 的 35 项结果一致；`TERRA_BUILD_RENDER` 默认关闭；`headless.no_graphics_libs`；第 4 步 M0 回归 | 满足：M0 回归全部通过，渲染关闭 39/39 |
| 图像与基线记录齐全 | 参考图 3 张（已提交）；帧耗时 5 组；GPU 分阶段耗时；测量条件与硬件（第 5 节） | 满足（LOG-065、LOG-068） |

## 8. PRD M1 需求核对

| 需求 | 验收要点（PRD） | 证据 | 状态 |
| --- | --- | --- | --- |
| F-M1-01 / P0 OpenGL 窗口、相机、基础网格与深度测试 | 显示并移动观察立方体；遮挡正确；关闭渲染不影响 Headless | M1-03（窗口）、M1-06（深度测试：`graphics.render_core` 的遮挡检查）、M1-07（相机、用户试用）、M1-08a（程序网格）；渲染关闭 39/39 | 满足 |
| F-M1-02 / P0 基础材质、光照及资源加载最小路径 | 显示指定测试资产，资源缺失可诊断；渲染公共接口不接收 MuJoCo 类型；glTF/GLB 方案待验证 | M1-08a/b（PBR、阴影、IBL）；M1-09（cgltf，单位与轴向换算，缺失贴图与文件的诊断）；第 6 节 | 满足；D-013 中 glTF 加载已验证 |
| F-M1-03 / P1 基础画质与帧耗时基线 | 固定光照参考可复查；记录帧耗时与测量条件 | M1-10 参考图与 `terra_render_bench`；第 5 节 | 满足 |
| F-M1-04 / P0 独立输入抽象与动作映射 | 相机控制器只依赖动作接口；映射可配置；多来源优先级仲裁；死区与热插拔有接口与测试（手柄实现可为 P1） | M1-04（`input.system`，8 个变异）；`boundary.render_isolation`；M1-14 按键绑定可在设置中修改 | 满足；GLFW 手柄适配器（M1-05，P1）后置 |
| F-M1-05 / P0 编辑器式界面与显示模式 | 深色主题、可停靠面板；F11 无边框全屏；鼠标在面板上时视口不响应；界面不绕过 Terra 输入系统 | M1-12（显示模式，真实显示器检查）；M1-13（`ui.input_bridge`，不使用 ImGui 的 GLFW 后端，用户试用） | 满足 |
| F-M1-06 / P0 视口相机控制与设置 | 中键转视角、Shift+中键平移、键盘移动、滚轮拉近；按键可改；设置立即生效并跨次运行保留；损坏文件回退默认值且有诊断 | M1-07、M1-13 修订（中键操作）；M1-14（`settings.store`，两次展示确认改动保留） | 满足 |
| NFR-01 模块解耦 | 公共契约不暴露 MuJoCo 与 OpenGL 类型；纯物理目标不需要窗口 | 第 6 节 | 满足 |
| NFR-03 性能 | M1 起记录帧耗时 | 第 5 节；查看器统计面板与性能图表（M1-15） | 已记录 |
| NFR-06 可测试性 | 图形测试单独启用，不阻塞无图形测试 | `graphics` 标签；Xvfb + llvmpipe 无人值守（D-022） | 满足 |

## 9. 任务状态

| 任务 | 优先级 | 状态 | 证据 |
| --- | --- | --- | --- |
| M1-01 图形环境与 OpenGL 最低版本 | P0 | 已验收 | [M1-01](M1-01-graphics-env.md)；D-021、D-022；LOG-024、025 |
| M1-02 渲染依赖源码锁定 | P0 | 已验收 | [M1-02](M1-02-render-deps.md)；LOG-026、027 |
| M1-03 平台窗口与上下文 | P0 | 已验收 | [M1-03/04](M1-03-04-platform-input-design.md)；LOG-030、031 |
| M1-04 输入抽象（核心） | P0 | 已验收 | 同上 |
| M1-05 GLFW 手柄适配器 | P1 | **后置**（未开发；死区与热插拔核心逻辑已在 M1-04 实现） | 不阻塞 M1 |
| M1-06 渲染核心 | P0 | 已验收 | [M1-06](M1-06-render-core.md)；LOG-033、034 |
| M1-07 相机与视口相机控制器 | P0 | 已验收 | [M1-07](M1-07-camera.md)；LOG-036–039 |
| M1-12 显示模式 | P0 | 已验收 | [M1-12](M1-12-display-modes.md)；LOG-041–043 |
| M1-13 编辑器式界面外壳 | P0 | 已验收 | [M1-13](M1-13-ui-shell-design.md)；LOG-046–050 |
| M1-14 设置与持久化 | P0 | 已验收 | [M1-14](M1-14-settings.md)；LOG-051、052 |
| M1-08a 网格、PBR、阴影、色调映射、MSAA | P0 | 已验收 | [M1-08 设计](M1-08-materials-lighting-design.md)；LOG-054–056 |
| M1-08b 基于图像的光照 | P0 | 已验收 | 同上；LOG-057–059 |
| M1-09 资产加载最小路径 | P0 | 已验收 | [M1-09](M1-09-assets-design.md)；LOG-061、062 |
| M1-10 渲染测试与帧耗时基线 | P0 / P1 | 已验收 | [M1-10](M1-10-render-baseline-design.md)；LOG-064、065、068 |
| M1-15 性能监视图表 | P2 | 已验收 | [M1-15](M1-15-perf-charts.md)；LOG-066–071 |
| M1-11 阶段验收 | P0 | 已验收 | 本报告；LOG-072–074 |

## 10. 剩余风险与待决事项

**需要用户处理：**

1. ~~提交与统一验收运行~~：已完成（提交 `e5900c8`，第 11.2 节）；用户 2026-11-23 评审通过。
2. **项目许可证**（D-109）：仍待决定；发布前确定即可，不阻塞 M1。

**技术风险（不阻塞 M1，在后续阶段处理）：**

| 风险 | 影响 | 处理时机 |
| --- | --- | --- |
| 材质观感仍不够真实（用户反馈：金属等质感）；没有物体之间的遮蔽（GTAO）、反射探针等 | 画质离 UE5/Blender 有差距 | M3（M1-10 设计第 6 节已定：GTAO 等放到 M3；D-027 的后备是重新评估 Filament） |
| 像素基准只在 llvmpipe（Mesa 23.2.1）上成立；升级 Mesa 可能改变像素 | 参考图测试失败 | 升级时用 `scripts/verify/update_references.sh` 重新生成，目视确认后提交 |
| NVIDIA 与 llvmpipe 渲染约有 0.5% 像素不同 | 真实 GPU 上不能做像素判定 | 只作记录（D-022） |
| 显存只能通过 NVIDIA（NVX）与 AMD（ATI）扩展查询；软件渲染器显示“不可用” | 其他驱动看不到显存 | 需要时在 M4 的 Vulkan/图像后端评估中再考虑 |
| 手柄未实现（M1-05，P1） | 只能用键鼠 | 需要时开发；核心逻辑已有测试 |
| 只验证了 Ubuntu 22.04 / x86-64 / X11 桌面（GLFW 同时构建了 Wayland，但 Wayland 会话未试用） | 其他平台与会话不保证 | NFR-05，P2 |
| 渲染单线程；尚未接物理 | M2 快照驱动渲染时要处理发布方式与帧率独立 | M2（F-M2-02、D-106） |
| CI 未接入 | 回归依赖人工在基线机器运行脚本 | P1 |
| 物体库拖放摆放（F-M5-05） | 只记录了想法 | M5，依赖 M2 场景系统与视口拾取 |

## 11. 验证记录

### 11.1 开发机

未编译、未运行（按约定）。已做：
- `check_mutation_anchors.py` 运行结果：71/71 锚点唯一。
- 所有改动的脚本均通过 `bash -n` 语法检查。
- 第 6 节的源码搜索：渲染侧引用物理模块 0 处。

### 11.2 基线机器

- **日期：**2026-11-23。
- **提交：**`e5900c8`，未提交改动 0 个文件。
- **环境：**Ubuntu 22.04，x86-64；gcc/g++ 11.4.0（apt 包 `gcc-11`/`g++-11` 11.4.0-1ubuntu1~22.04.3）；cmake/ctest 3.22.1；ninja 1.10.1；git 2.34.1；GNU ld 2.38。GPU：NVIDIA GeForce RTX 5060（PCI 10de:2d05），驱动 595.91.07，OpenGL 4.6 core；自动化图形测试：llvmpipe（LLVM 15.0.7，Mesa 23.2.1）。
- **命令：**`bash scripts/verify/m1_acceptance.sh`，用时 7 分 19 秒。
- **结果：**

| 步骤 | 结果 |
| --- | --- |
| 1. 提交 | `e5900c8`，工作区干净 |
| 2. 环境清单 | 已采集；工具均来自 Ubuntu 官方软件包；图形清单确认桌面为 NVIDIA、Xvfb 为 llvmpipe |
| 3. 变异锚点 | **71/71** 唯一 |
| 4. M0 回归 | `m0_acceptance.sh` 全部通过：MuJoCo 源码包 sha256 一致；`m0_08.sh`、`m0_07.sh` 退出码 0；重复运行 39/39（每项至多 5 次）；运行依赖无图形库 |
| 5. M1 全量 | 依赖与素材 sha256 全部一致（json 3/3、stb 3/3、cgltf 2/2、渲染依赖、HDRI 2/2、glTF 15/15）；渲染关闭 Debug **39/39**；渲染开启 Debug、Release + `-Werror` **57/57**；ASan 非图形 46/46、图形 11/11；M1-15 变异 5/5 |
| 6. M1 历次变异 | 03/04 8/8、06 5/5、07 5/5、12 3/3、13 5/5、14 6/6、08a 6/6、08b 6/6、09 6/6、10 5/5；**合计 60/60**（加 M0 的 11 个，共 71 个变异全部被发现）；没有“以非 0 退出但没有 FAILED 行”的可疑项 |
| 7. 重复运行 | 渲染开启 57 项每项至多 3 次，**全部通过** |
| 8. 依赖边界 | `terra_headless`、`terra_bench` 无图形库；`terra_viewer`、`terra_render_bench` 不链接 libmujoco；渲染侧 9 个模块 0 处引用物理模块；三项边界测试通过 |
| 9. 真实 GPU | 查看器自检通过（视口射线、C++ 参考着色、性能图表、显存 8151 / 可用 5342 MB）；帧耗时见下表 |
| 10. 可交互展示 | 117.7 秒、7049 帧、59.9 FPS（垂直同步）；三个资产、两张环境贴图加载正常，设置自动保存；退出码 0 |

帧耗时（RTX 5060，600 帧，预热 60 帧，不经过垂直同步）：

| 配置 | 整帧中位数（p95） | GPU 中位数（p95） |
| --- | --- | --- |
| 1080p 默认 | 0.47 ms（0.56） | 0.38 ms（0.39） |
| 4K 默认 | 1.58 ms（1.73） | 1.30 ms（1.56） |
| 4K 无 MSAA | 0.84 ms（1.24） | 0.81 ms（0.81） |
| 4K 无阴影 | 1.17 ms（1.54） | 1.11 ms（1.12） |
| 4K 程序场景 | 1.30 ms（1.41） | 1.05 ms（1.06） |

GPU 中位数与 M1-10/M1-15 记录的数值相差不超过 0.01 ms，基线稳定。

**结论：全部通过。**用户于 2026-11-23 评审通过，M1 完成。
