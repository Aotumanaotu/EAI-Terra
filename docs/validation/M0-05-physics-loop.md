# M0-05 物理仿真循环

> 状态：**已验收**（用户于 2026-11-19 确认评审通过）。固定步进、reset、故障检测、Headless 实际运行与 JSON Lines 输出均已完成；基线机器 Debug/Release 27/27 通过，5000 步轨迹与开发机逐位相同（5.2 节）。

## 1. 交付内容

| 部分 | 位置 | 说明 |
| --- | --- | --- |
| `MujocoWorld::step()` | `engine/physics/backends/mujoco/` | `mj_step` → `mj_forward` → 检查；成功时 `stepIndex` 加 1，失败时进入故障状态 |
| `MujocoWorld::reset()` | 同上 | `mj_resetData` → `mj_forward`；时间与步序号归零，`ctrl` 清零，清除故障，`runEpoch` 加 1 |
| 状态查询 | 同上 | `stepIndex()`、`runEpoch()`（加载后为 0）、`faulted()` |
| 求解配置记录 | `terra_data::SimulationSettings` | 重力、积分器、求解器、迭代次数、容差，随 `model` 记录输出（开发计划 M0-05 ①） |
| `RunSchedule` | `engine/core/include/terra/core/run_schedule.hpp` | 输出采样：第 0 步与最后一步总是输出，中间每 N 步输出一次（M0-03 遗留的问题在此确定） |
| Headless 运行 | `apps/headless/main.cpp` | 按步数离线运行，不 sleep、不追赶实时；输出 config → model → state… → summary/error |
| 故障夹具 | `assets/tests/fault_runaway.xml`、`fault_bad_acceleration.xml` | 分别触发 Terra 自身检测和 MuJoCo 自动重置 |

## 2. 步进语义

### 2.1 派生量刷新

`mj_step` 先算动力学再积分，积分后 `xpos/xquat` 仍对应积分前的状态。每步之后再调用一次 `mj_forward`，让发布的位姿与新时间一致（技术设计 2.3 节）。

开发机实验（官方预编译 4.0.1，立方体夹具）：

| 项目 | 结果 |
| --- | --- |
| 第 1 步后不刷新 | `qpos.z` = 0.99999019，`xpos.z` = **1.0**（旧值） |
| 第 1 步后刷新 | `xpos.z` = 0.99999019 = 1 − g·dt²（半隐式 Euler 理论值） |
| 每步额外调用 `mj_forward` 是否改变轨迹 | 在 N = 100、400、450（已触地）、1000、5000 时，`qpos` 与不调用时**逐位相同** |

这里只验证了立方体夹具和 Euler 积分器；换用其他积分器、或模型带执行器/传感器时需要重新验证。代价是每步多算一次动力学，性能影响在 M0-09 测量。

### 2.2 故障检测

MuJoCo 遇到坏值（NaN 或 |x| > `mjMAXVAL` = 1e10）时，会在**下一步开始时**调用 `mj_resetData`，只把警告计数器加 1，仿真时间回到 0 后继续运行。不检查的话，看起来像是正常运行。Terra 每步之后按下列顺序检查：

| 检查 | 错误类型 | 说明 |
| --- | --- | --- |
| MuJoCo 警告计数器有任何增加 | `AutoReset`（qpos/qvel/qacc 坏值，且未禁用自动重置）或 `SolverWarning`（接触/约束缓冲区溢出、惯量奇异、ctrl 坏值） | 7 类警告全部视为严重诊断；以后如有必要再区分 |
| 仿真时间没有前进 | `AutoReset` | 防备未通过计数器报告的重置 |
| `qpos`/`qvel` 有坏值（`mju_isBad`） | `NonFinite` | 在**当步**发现，不必等 MuJoCo 下一步再发现 |
| 时间、`xpos`/`xquat` 非有限 | `NonFinite` | |

出错后：
- 世界进入故障状态，`stepIndex` 停在最后一个正常完成的步；之后 `step()` 都返回 `Faulted`，直到 `reset()`。
- 若错误是 `AutoReset`，此时的状态已经是 MuJoCo 重置后的值。
- Headless 输出一条 `error` 记录，并以退出码 5 结束。

实测：`fault_bad_acceleration` 在 MuJoCo 自动重置后，时间仍推进到了 0.001 s，因此“时间是否前进”这项检查发现不了，只有警告计数器能捕获。两道检查都有必要。

### 2.3 输出与仿真相互独立

- 仿真时间只由步数决定：`time ≈ stepIndex × dt`，由 MuJoCo 逐步累加。
- 输出采样、读取状态不调用任何物理计算，有测试验证：每步都读取和完全不读取，结果逐位相同。

## 3. Headless 输出（初版，M0-06 确定正式格式）

```text
{"type":"config", …运行参数…}
{"type":"model","timestep":0.001,"gravity":[0,0,-9.8100000000000005],"integrator":"Euler","solver":"Newton","iterations":100,"tolerance":1e-10,"entities":[{"id":1,"name":"cube","parent":0}],"joints":[{"id":1,"name":"cube_free","type":"free","entity":1,"nq":7,"nv":6}]}
{"type":"state","run_epoch":0,"step":0,"time":0,"entities":[{"id":1,"position":[0,0,1],"orientation":[1,0,0,0]}]}
…
{"type":"summary","run_epoch":0,"steps":5000,"time":5.000000000000004,"status":"ok"}
```

- 出错时最后一行为 `{"type":"error","kind":"non_finite"|"auto_reset"|"solver_warning"|"faulted","run_epoch":…,"step":…,"time":…,"message":…}`。
- 实体名称只在 `model` 记录中出现，`state` 记录只用 ID。
- 速度、关节状态、接触摘要和诊断信息在 M0-06 加入。
- `-9.8100000000000005` 是 `%.17g` 对 9.81 的精确表示，读回后与 9.81 相等。

## 4. 测试

总数从 22 项变为 27 项：

- **新增 `physics.mujoco_step`：**
  - 时间：5000 步内每步 |t − N·dt| ≤ 1e-8，严格单调。
  - 派生量刷新：第 1 步后 z **精确等于** 1 − g·dt²（误差 ≤ 1e-15）。
  - 自由落体：0.05–0.20 s 内与解析解的误差 ≤ 0.005 m，持续下落，水平无漂移。
  - 稳定：最后 0.5 s 内高度距 0.1 m 不超过 0.005 m，姿态接近初始。
  - reset：20 次 reset，每次代次加 1、时间与位姿回到初值、重复运行的轨迹与首次**逐位一致**；运行中途 reset。
  - 读取状态不影响仿真：每步读取与完全不读取的结果逐位相同。
  - 故障：runaway 在第 5 步报 `NonFinite`，`stepIndex` 停在 4；之后 `step` 返回 `Faulted`；reset 后恢复。bad acceleration 在第 1 步报 `AutoReset`。
  - `RunSchedule` 的采样规则。
- **新增 Headless 用例：**
  - `run_cube`：检查输出各行的完整顺序与内容。
  - `run_duration`：`--duration 0.01` 运行 10 步。
  - `run_sparse_sampling`：采样间隔大于总步数时，仍输出第 0 步和最后一步。
  - `run_fault_runaway`：退出码 5，`non_finite`，第 5 步。
  - `run_fault_auto_reset`：退出码 5，`auto_reset`，第 1 步。
- **删除 `headless.run_not_available`**：步进已接入，不再有“尚未接入”的情况。
- **`headless.cli_parser`**：新增 model/state/summary/error 记录格式的精确比对，包括字符串转义。

**反向检查：** 把 `run_cube` 期望的采样步改为 51、或让 `sparse_sampling` 多输出行，正则检查都会失败。手动检查时发现，在 shell 中必须用 `$'…'` 传入真正的换行符；CTest 中 `\\n` 经 `add_test` 处理后就是真正的换行符，测试本身是有效的。

## 5. 验证记录

### 5.1 开发机（不是基线验证）

- **依赖：** 官方预编译 MuJoCo 4.0.1（同 M0-04 报告 5.1 节）。
- **结果：** Debug + `-Werror`、Release + `-Werror`、ASan/UBSan（含 LeakSanitizer）均 **27/27 通过**，第一次运行即全部通过。
- **5000 步立方体的实测数值**（Headless 输出逐步状态后计算，作为 M0-08 的参考，不是正式判定）：

| 指标 | 实测 | 候选门槛（开发计划 4.2） |
| --- | --- | --- |
| 时间误差 \|t − N·dt\| 最大值 | 3.3e-13 s | ≤ 1e-8 s |
| 自由落体误差（0.05–0.20 s）最大值 | 9.81e-4 m | ≤ 0.005 m |
| 最终中心高度 | 0.099892245 m | — |
| 稳定窗口高度误差最大值 | 1.08e-4 m | ≤ 0.005 m |

自由落体误差 9.81e-4 m 恰好等于半隐式 Euler 在 t = 0.2 s 时的理论偏差 g·t·dt/2，因此误差来自积分方法本身，与实现无关。

### 5.2 基线机器

- **日期：** 2026-11-13；机器与工具链同 [M0-02 报告](M0-02-mujoco.md) 4.2 节（GCC 11.4.0，链接 gcc-12 源码构建的 MuJoCo 4.0.1）。
- **测试：** 删除 `build/` 后，Debug 与 Release（`-Werror`）各构建 21 步，均 **27/27 通过**。`physics.mujoco_step` 用时约 0.7 s。
- **5000 步立方体**（`--steps 5000 --sample-every 1000`）：

| 步 | 时间 | z | 说明 |
| --- | --- | --- | --- |
| 0 | 0 | 1 | 初始 |
| 1000 | 1.0000000000000007 | 0.099892244542846245 | 已触地并基本稳定 |
| 5000 | 5.0000000000000044 | 0.099892244579783615 | 最终 |

  触地后 x、y 与四元数虚部为 1e-19 至 1e-15 量级的非零值，来自接触求解的数值噪声，远小于任何容差。

- **与开发机的对比：** 在开发机上用官方预编译库运行同一条命令，6 条 state 记录与基线机器**逐位相同**（时间、位置、姿态差均为 0），包括触地后的微小噪声。也就是说，在这个夹具和配置下，官方预编译库（Xeon）与 gcc-12 源码构建（i7-13700）的结果一致。这只是对单一夹具的观察，**不能据此声称跨平台逐位确定**（开发计划 4.3 节）。

## 6. 验收标准对照（开发计划 M0-05）

| 标准 | 当前 |
| --- | --- |
| 完成立方体基准 | 时间、自由落体、稳定高度已在开发机通过；速度与接触判定需要 M0-06 的状态读取，正式判定属于 M0-08 |
| 仿真时间按步长增长，不受输出或墙钟 sleep 影响 | 时间误差用例；读取与否结果逐位相同；Headless 不 sleep |
| reset 后回到初始时刻和位姿 | 20 次 reset 用例 |
| 日志能区分新旧运行代次 | 每条 state/summary/error 记录都带 `run_epoch`；reset 后代次加 1 |
| 非有限数、意外重置、严重诊断立即结束并报告 | 两个故障夹具，退出码 5 |
| 不加入追赶实时、可变步长或渲染调度 | 没有加入 |

**附带：** M0-03 的“正常完成零退出”已在基线机器上通过（`run_cube`、`run_duration`）；M0-03 与 M0-05 均于 2026-11-19 经用户评审通过。
