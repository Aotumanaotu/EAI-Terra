# M0-01 环境与工程初始化验证

> 状态：**已验收**。基线机器上 Debug/Release 干净构建与测试全部通过（4.2 节）；用户于 2026-11-19 确认评审通过。

## 1. 目标边界（M0-01 评审结果）

| 目标 | 位置 | 职责 | 依赖 | 状态 |
| --- | --- | --- | --- | --- |
| `terra_core`（`terra::core`） | `engine/core/` | 最小公共类型；当前只有构建信息 `terra::buildInfo()` | 无外部库 | 已建立 |
| `terra_headless` | `apps/headless/` | 命令行入口；当前只有 `--help`、`--version` | `terra::core` | 已建立（骨架） |
| `test_core_build_info` 等 CTest 用例 | `tests/` | 工程骨架自检，见第 3 节 | `terra::core`、`terra_headless` | 已建立 |
| `terra_physics` | `engine/physics/` | 后端无关物理接口 | `terra::core` | **未创建**，M0-04–07 接入 |
| `terra_physics_mujoco` | `engine/physics/backends/mujoco/` | 私有链接官方 C API | MuJoCo 官方库 | M0-02 已建立骨架，见 [M0-02 报告](M0-02-mujoco.md) |

约束：

- 所有目标通过 `terra_configure_target()`（`cmake/TerraCompilerOptions.cmake`）设置 `cxx_std_17`，关闭 GNU 扩展，启用 `-Wall -Wextra -Wpedantic` 等警告。
- 根构建不调用 `find_package(OpenGL)` 或任何窗口/图形库；`headless.no_graphics_libs` 测试用 ldd 检查。
- 禁止在源码目录内构建。
- 公共数据契约（`PhysicsState`/`PhysicsSnapshot`、ID 类型）按技术设计放在 `engine/core/`，由 M0-04–06 实际需要时再加入，本任务不提前创建。

## 2. 构建与测试步骤

前置条件：GCC（C++17）、CMake ≥ 3.22、Ninja。本工程没有其他依赖，构建阶段不联网。

```bash
# 记录环境（只读）
scripts/env_inventory.sh > env.txt

# Debug
cmake --preset debug
cmake --build --preset debug
ctest --preset debug

# Release，同时把警告视为错误
cmake --preset release -DTERRA_WARNINGS_AS_ERRORS=ON
cmake --build --preset release
ctest --preset release
```

构建目录为 `build/<preset>/`，已在 `.gitignore` 中忽略。干净构建只需删除对应目录后重新执行。

可选项：

| 选项 | 默认值 | 说明 |
| --- | --- | --- |
| `TERRA_BUILD_TESTS` | `ON` | 构建并注册 CTest 测试 |
| `TERRA_WARNINGS_AS_ERRORS` | `OFF` | 加 `-Werror` |

## 3. 测试用例

| 用例 | 检查内容 |
| --- | --- |
| `core.build_info` | 库与调用方都以 C++17 编译；版本字段自洽 |
| `headless.help` | `--help` 的退出码精确为 0 |
| `headless.version` | `--version` 输出以 `terra_headless <版本>` 开头 |
| `headless.unknown_argument` | 未知参数的退出码精确为 2；崩溃不算通过 |
| `headless.no_display` | 清除 `DISPLAY`、`WAYLAND_DISPLAY` 后可以正常运行 |
| `headless.no_graphics_libs` | ldd 结果中没有 GL/EGL/GLES/OSMesa/X11/xcb/Wayland/GLFW/Vulkan 库（仅 Linux） |

这些用例只证明工程骨架可用，不涉及物理功能。

## 4. 验证记录

### 4.1 开发机自测（不是基线验证）

- **日期：** 2026-11-13（UTC）
- **环境：** Ubuntu 22.04.5 LTS，x86_64（KVM 虚拟机，2 核，1.6 GiB 内存），glibc 2.35，GCC 11.4.0，GNU ld 2.38。
- **工具来源：** 系统没有安装 CMake 和 Ninja。自测时临时用 pip 装在 `/tmp/terra-tools`，版本为 **CMake 4.4.4**、**Ninja 1.13.2**，比基线机器上的大概率更新，**不作为版本记录**。
- **执行内容与结果：**
  - `cmake --preset debug`、`cmake --build --preset debug`、`ctest --preset debug`：6/6 通过。
  - `cmake --preset release -DTERRA_WARNINGS_AS_ERRORS=ON` 后构建、测试：没有警告，6/6 通过。
  - `compile_commands.json` 中的标准参数只有 `-std=c++17`。
  - 反向检查：期望退出码与实际不符时，`TerraCheckExitCode.cmake` 会报错；用模拟的 ldd 输出验证了图形库正则能匹配 libGL/libEGL/libwayland-client/libxcb，不会误匹配 libglib、libstdc++；在源码目录内配置时会被拒绝。
- **未验证：** CMake 3.22 下的兼容性（Presets v3、`cmake -E env --unset` 等）。

### 4.2 基线机器验证

- **日期：** 2026-11-13T04:00:08Z（环境清单采集时间）
- **机器：** HP Pro Tower 280 G9（物理机），13th Gen Intel Core i7-13700，24 线程，31.0 GiB 内存
- **系统：** Ubuntu 22.04.5 LTS，x86_64，内核 6.8.0-106-generic，glibc 2.35（`2.35-0ubuntu3.13`）
- **工具链（M0-01 记录的精确版本）：**

| 工具 | 版本 | 来源 |
| --- | --- | --- |
| GCC / G++ | 11.4.0（`11.4.0-1ubuntu1~22.04.3`） | Ubuntu 22.04 系统包 |
| CMake / CTest | 3.22.1 | apt 包 `cmake` 3.22.1-1ubuntu1.22.04.2（M0-09 以 dpkg 归属核实） |
| Ninja | 1.10.1 | apt 包 `ninja-build` 1.10.1-1（M0-09 以 dpkg 归属核实） |
| GNU ld | 2.38 | Ubuntu 22.04 系统包 |
| Git | 2.34.1 | Ubuntu 22.04 系统包 |

- **执行命令：**

```bash
scripts/env_inventory.sh
cmake --preset debug && cmake --build --preset debug && ctest --preset debug
cmake --preset release -DTERRA_WARNINGS_AS_ERRORS=ON && cmake --build --preset release && ctest --preset release
```

- **结果：**
  - 两个构建目录都是全新配置（输出中可以看到编译器检测过程），即干净构建。
  - Debug：6/6 通过。
  - Release（`-Werror`）：构建成功，6/6 通过。
  - 在有图形环境的桌面机（`DISPLAY=:0`）上，`headless.no_display` 与 `headless.no_graphics_libs` 均通过。
  - 4.1 节中未验证的 CMake 3.22 兼容性（Presets v3、`cmake -E env --unset`），在本次运行中得到确认。
- **已知问题：** 本次运行的 `env_inventory.sh` 有两处脚本缺陷：
  - 物理机上“容器/虚拟化”多输出一行“未知”。
  - 图形库检查用空格匹配 `ldconfig -p` 的输出，而实际分隔符是制表符，因此结果一律为“未找到”。

  这两处已在 LOG-003 中修复。本节图形库一栏不可信，没有记录，但不影响 M0-01 判定：M0 只要求 Terra 自身不依赖图形库，这一点由 `headless.no_graphics_libs` 的 ldd 检查覆盖。需要了解基线机器的图形库现状时，重新运行脚本即可。

### 4.3 M0-01 验收标准对照

| 验收标准（开发计划 M0-01） | 结果 | 证据 |
| --- | --- | --- |
| 干净构建目录用 Ninja 完成基础目标构建 | 满足 | 4.2，Debug 与 Release |
| C++17 配置生效 | 满足 | `core.build_info`；开发机检查 `compile_commands.json` 为 `-std=c++17` |
| 物理目标不要求窗口系统和图形开发包 | 满足（当前目标范围） | `headless.no_graphics_libs`、`headless.no_display`；`terra_physics` 尚未创建，M0-02 起由同一测试继续约束 |
| 明确哪些目标尚待后续任务接入 | 满足 | 第 1 节 |
| 工具精确版本可追溯 | 满足 | 4.2 工具链表；CMake/Ninja 来源已核实为 Ubuntu 官方 apt 软件包（`cmake` 3.22.1-1ubuntu1.22.04.2、`ninja-build` 1.10.1-1），见 [M0-09 报告](M0-09-acceptance.md) 9.2 节 |

**验收结论：** 2026-11-19 用户评审通过；本报告随 M0-06～M0-09 交付提交归档，可通过 Git 日志关联提交。
