# M0-09 自动化测试与阶段验收

> 状态：**进行中——用户评审通过，基线机器统一验收运行未执行**。统一入口 `scripts/verify/m0_acceptance.sh`，性能基线程序 `terra_bench`，第三方许可核查与 MuJoCo 4.0.1 锁定已完成。按约定未在开发机编译或运行。第 7 节逐条核对 M0 DoD；基线运行结果补入第 5、9 节。M0-01 至 M0-08 已于 2026-11-19 经用户评审转为“已验收”。

## 1. 范围（开发计划 M0-09）

| 步骤 | 交付 |
| --- | --- |
| ① 统一测试入口 | CTest（35 项）+ 验收脚本 `scripts/verify/m0_acceptance.sh`，一条命令复现全部证据（第 2 节） |
| ② 干净构建目录与无显示环境 | 脚本删除构建目录后重建；整个脚本在清除 `DISPLAY`/`WAYLAND_DISPLAY` 的环境中运行 |
| ③ 重复运行、连续运行、快照持有 | 重复运行：Release 全量 CTest 每项至多 5 次；连续运行与快照持有：M0-08 `physics.continuous`、`physics.scenarios`、M0-06 快照测试 |
| ④ 审查运行依赖与公共类型 | `ldd` 检查（第 6 节）；公共头清单与边界检查（第 6 节） |
| ⑤ 性能基线（不阻塞） | 新增 `apps/bench`（`terra_bench`），三种模式分别计时（第 5 节） |
| ⑥ 汇总报告、核对 DoD | 本报告第 7、8 节；开发记录 LOG-020 |

另外关闭了以下遗留项：MuJoCo 锁定条件 3（第三方许可核查，第 3 节）；M0-01 的“CMake/Ninja 安装来源未核实”与“环境清单脚本需重跑”（`env_inventory.sh` 增加 dpkg 归属查询，验收脚本自动运行）。

## 2. 从零复现 M0

在 Ubuntu 22.04 / x86-64 上：

```bash
sudo apt install build-essential gcc-12 g++-12 cmake ninja-build python3 git curl   # 仅首次
scripts/mujoco/fetch.sh            # 下载 MuJoCo 4.0.1 源码，按 mujoco.lock 校验 sha256
scripts/mujoco/build.sh            # 构建 libmujoco 并安装到 third_party/mujoco/install（首次需联网拉取依赖）
bash scripts/verify/m0_acceptance.sh
```

验收脚本依次执行：提交与工作区状态 → 环境清单（含工具来源）→ MuJoCo 源码包 sha256 核对 → `m0_08.sh`（Debug、Release + `-Werror`、ASan/UBSan 全量 CTest，M0-08 变异，场景判定输出）→ `m0_07.sh --skip-build`（M0-07 变异）→ 重复运行 → `ldd` 依赖检查 → 性能基线。任一必需步骤失败时退出码非 0；日志在 `build/verify-m0/logs/`，各子脚本日志在 `build/verify-m0-07/`、`build/verify-m0-08/`。

单独查看物理输出：

```bash
build/release/apps/headless/terra_headless --model assets/tests/falling_cube.xml --duration 5 --sample-every 500
```

## 3. 第三方许可核查（MuJoCo 锁定条件 3，D-109 第三方部分）

逐个阅读 `third_party/mujoco/src/mujoco-4.0.1/LICENSE` 与 `third_party/mujoco/deps/*-src/` 中的许可文件（2026-11-18）：

| 组件 | 许可 | 再分发义务 |
| --- | --- | --- |
| MuJoCo 4.0.1 | Apache-2.0 | 附带许可文本；保留版权、专利与 NOTICE 声明；修改的文件需注明 |
| libccd v2.1 | BSD（3 条款，`BSD-LICENSE`） | 二进制分发时在文档中附带版权声明与条款；不得用作者名义背书 |
| qhull | Qhull 许可（`COPYING.txt`） | 保留版权声明；**随包含 qhull 的程序附带 COPYING.txt**；修改需注明修改人、日期、原因，并说明原始源码获取方式 |
| lodepng | zlib | 不得冒称原作者；修改版须标明；源码分发保留声明 |
| tinyxml2 | zlib | 同上 |
| tinyobjloader | MIT | 附带版权与许可声明 |
| miniz | MIT | 附带版权与许可声明 |
| MarchingCubeCpp | 公有领域或 MIT（二选一，见 README） | 选 MIT 时附带声明；公有领域无义务 |

**结论：**全部为宽松许可，没有 copyleft 条款，不限制 Terra 选择许可证。分发包含 `libmujoco.so` 的二进制时，需随附以上许可文本（qhull、libccd 有明确要求）。只在开启 MuJoCo 测试或 Python 工具时才拉取的 abseil、googletest、benchmark、Eigen 不参与当前构建，未核查。

**未关闭：**Terra 项目自身的许可证（D-109）仍需用户决定；在发布或接受外部贡献前确定即可，不阻塞 M0。测试夹具 `assets/tests/*.xml` 均为本项目编写，不涉及第三方资产。

## 4. MuJoCo 4.0.1 锁定（D-108）

[M0-02 报告](M0-02-mujoco.md)第 5 节的三个锁定条件：

1. M0-02 验收：基线机器验证通过（LOG-005）。
2. M0-04–08 物理回归：全部在 4.0.1 下通过（LOG-010、012、014、017、019）。
3. 依赖许可核查：第 3 节。

已在 `third_party/mujoco/mujoco.lock` 中标为“已锁定”，D-108 改为“已确定”。验收脚本核对源码包 sha256 与锁定值一致。升级版本须重跑验收脚本，并重新评审 M0-08 冻结的容差。

## 5. 性能基线（只记录，不判定）

`terra_bench`（`apps/bench/bench.cpp`）在同一进程内测量三种模式。每轮从 reset 开始，预热 2 轮，正式 20 轮 × 5000 步，每步单独计时：

| 模式 | 测量内容 |
| --- | --- |
| `step` | 只推进物理 |
| `snapshot` | 推进后每步取快照（Headless 的观测开销，不含序列化） |
| `jsonl` | 推进、取快照，并按采样间隔序列化为 JSON Lines 写入 `/dev/null` |

输出每步耗时的平均值、p50/p95/p99/最大值，以及每轮实时因子（仿真时间/墙钟时间）的中位数和最小值，同时记录构建配置、编译器、后端版本、模型摘要、CPU、核数、内存与频率调节策略。验收脚本运行三组：立方体每步输出、立方体每 100 步输出、铰链每步输出。

**结果：**待基线机器运行后补入。PRD NFR-03 的候选目标（简单场景实时因子 ≥ 1）只作参考，不是 M0 门槛；立方体结果不能外推到复杂机器人或工业场景。

## 6. 运行依赖与公共类型审查

**公共头文件（共 9 个）：**

| 目标 | 头文件 |
| --- | --- |
| `terra_data` | `terra/data/geometry.hpp`、`ids.hpp`、`model_description.hpp`、`state.hpp` |
| `terra_core` | `terra/core/build_info.hpp`、`run_schedule.hpp` |
| `terra_physics` | `terra/physics/physics_world.hpp` |
| `terra_physics_mujoco` | `terra/physics/mujoco/backend_info.hpp`、`mujoco_backend.hpp` |

公共头只包含标准库（`<array> <cstdint> <functional> <memory> <optional> <string> <string_view> <vector>`）和 Terra 自己的头文件；不含 MuJoCo 类型、后端指针或图形类型。由 `boundary.no_mujoco_in_public_headers` 自动检查，扫描范围包括 `engine/**/include/`、`apps/`、`tests/physics/consumer/`；`physics.consumer` 证明调用方在没有 MuJoCo 头文件路径的情况下可以编译。

**运行依赖：**`headless.no_graphics_libs` 检查 `terra_headless` 的 ldd 输出。验收脚本对 `terra_headless` 与 `terra_bench` 再查一次 GL/EGL/OSMesa/X11/Vulkan/GLFW，并记录 `libmujoco` 的实际加载路径。

## 7. M0 Definition of Done 核对（开发计划第 7 节）

| DoD 条目 | 证据 | 状态 |
| --- | --- | --- |
| Ubuntu 22.04 / x86-64 上 GCC、C++17、CMake、Ninja 的精确环境可追溯，干净构建可复现 | M0-01 工具链表；验收脚本的环境清单（含 dpkg 来源）；所有基线验证都从删除构建目录开始 | 满足；工具来源待验收运行补入 |
| MuJoCo 官方版本及来源已验证并锁定，头文件/运行库一致，运行库查找明确，未修改其内核 | 第 4 节；`physics.mujoco_backend_info`（版本一致）；RPATH 指向本地安装（M0-02）；源码包 sha256 核对；未打补丁 | 满足 |
| Headless 入口可加载 MJCF、固定步进、reset、读取状态并处理最小控制；不依赖图形上下文 | M0-03–07 报告；`headless.*`、`physics.*`；`no_display`、`no_graphics_libs` | 满足。注：Headless 程序本身不接受控制输入，最小控制入口在 `PhysicsWorld::setControl`，M0-07 已验证（PRD F-M0-03“可选控制”） |
| 立方体主基准、辅助用例和错误路径通过，容差在受控条件下验证并冻结 | [M0-08 报告](M0-08-scenarios.md) 第 3、4、7 节 | 满足 |
| 所有公共接口只使用 Terra 类型；旧快照保持有效，时间、ID、单位和坐标语义一致 | 第 6 节；M0-06 不可变性用例；M0-08 生命周期用例与 `worldInstance` | 满足 |
| 自动化测试入口能判定成功/失败，缺失/损坏输入有可复查诊断；连续运行及重复生命周期检查完成 | CTest 35 项与验收脚本（非 0 退出码）；`headless.model_*`、`invalid_mjcf`、`arena_too_small`；`physics.continuous`；20 次 reset、100 次加载/释放 | 满足；重复运行待验收运行补入 |
| 记录实际构建和测试命令、版本、模型摘要、结果、性能基线和未关闭风险 | M0-01–M0-08 各报告；本报告第 2、5、8 节 | 性能基线待验收运行补入 |
| 开发记录关联提交与报告并更新为真实状态；评审确认无未完成 P0，才进入 M1 | 开发记录 LOG-001–021；本次提交归档；用户于 2026-11-19 确认评审全部通过 | 评审通过；待基线统一运行及结果归档后关闭 M0 |

## 8. 剩余风险与待决事项

**需要用户处理：**

1. **基线统一验收运行**：用户已于 2026-11-19 确认 M0-01–M0-09 评审全部通过，M0-01–M0-08 已转为“已验收”；M0-09 仍需实际运行并补入结果，不能以材料评审替代运行证据。
2. **关联提交**：本次提交统一归档 M0-06–M0-09 交付。应在全部提交后的干净工作区运行验收脚本，使证据对应明确提交。
3. **项目许可证**（D-109）：第三方部分已核查，项目自身许可证待定；不阻塞 M0，发布前确定。

**技术风险（不阻塞 M0，进入后续阶段时处理）：**

| 风险 | 影响 | 处理时机 |
| --- | --- | --- |
| MuJoCo 致命错误直接 `exit(1)`，Terra 无法接管 | 嵌入长期运行的进程（M2 可视化、M4 ROS2）时会整体退出 | M2 前评估子进程隔离或 `mju_user_error` 处理策略 |
| 运行期内存不足（接触规模大时）的行为没有固定夹具 | 复杂场景可能报 `SolverWarning` 或进程退出 | M3 机器人场景时补用例，必要时显式设置 `<size memory>` |
| 容差只适用于 4.0.1、Euler、当前夹具 | 换积分器或升级版本时需重新评审 | 版本升级或 M3 换积分器时 |
| 球关节状态只按文档约定转换，没有数值用例；flex 接触不计入 | 带球关节或柔性体的机器人 | M3 |
| 拒绝带控制延迟或 nsample 的执行器（偏保守）；`Other` 传动只描述不解释；执行器状态不进快照 | 部分机器人模型无法加载或缺少执行器观测 | M3（与记录/复现 F-M3-03 一起设计） |
| 7 类 MuJoCo 警告一律视为致命 | 复杂场景可能过于严格 | M3 按实际场景评审 |
| 单线程；快照每步完整复制 | 渲染与物理并行时的发布方式 | M2（D-106） |
| CI 平台未接入 | 回归依赖人工在基线机器运行脚本 | P1，可在 M1 期间接入 |

## 9. 验证记录

### 9.1 开发机

未执行（开发机不承担构建与测试）。静态检查：脚本 `bash -n` 语法检查；`terra_bench` 与新增 CMake 按项目警告选项人工复查。

### 9.2 基线机器（未执行）

```bash
bash scripts/verify/m0_acceptance.sh
```

预期：三种配置均 35/35；M0-07 变异 8/8，M0-08 变异 3/3；重复运行通过；`ldd` 无图形库；源码包 sha256 一致；打印性能基线与环境清单。
