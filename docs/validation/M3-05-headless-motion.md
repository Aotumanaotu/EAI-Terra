# M3-05 Headless 执行动作，与查看器逐位一致

> 状态：**已实现，待基线验证**（2026-11-24）。依据：开发计划 10.2 节 M3-05；[M3-04 设计稿](M3-04-motion-design.md) 第 6 节。

## 1. 交付

**Headless 新参数 `--motion <动作文件>`：**
- **场景：**不给 `--scene` 时，使用动作文件中的推荐场景；推荐场景也没有时，报用法错误（退出码 2）。不能与 `--model` 同用，因为动作需要带实例前缀的执行器名。
- **步数：**不给 `--steps` 或 `--duration` 时，运行到动作结束后 1 秒。
- **执行：**从起始关键帧开始，每一步之前按“动作 → PD 层 → `setControl`”下发控制，与查看器的每步控制器是同一条路径。
- **错误：**动作文件或绑定有错误时，逐条打印“动作错误 /位置：原因”，退出码 4；运行中控制被拒绝时输出 `error` 记录（`kind` 为 `control_rejected`），退出码 5。
- **输出：**格式不变，`config` 记录多一个 `motion` 字段；`scene` 字段为实际使用的场景。

**顺带修正：**场景中有 PD 层（`controllers`）而不给动作时，Headless 现在也会运行 PD 层（M3-04b 只接入了查看器）。Go2 场景在 Headless 中因此不会塌下去，与查看器一致。

## 2. 一致性的判定

新增 `test_motion_consistency`：
1. 运行 `terra_headless --scene … --motion … --steps N --sample-every N`，取第 N 步的 `state` 行；
2. 在同一进程中按查看器的路径执行：`SimulationRunner` + 每步控制器，帧间隔在 60 Hz、144 Hz 与随机抖动（0.5–35 ms）之间交替；
3. 用 Headless 的同一个函数 `stateToJson` 写出第 N 步的快照，与第 1 步得到的行**逐字节**比较。

比较的内容覆盖全部实体的位姿与速度、关节、接触和诊断，数值以 `%.17g` 输出，所以逐字节相同就是逐位相同。

## 3. 测试

| 测试 | 内容 |
| --- | --- |
| `motion.consistency`（新增） | 机械臂场景 + `arm_swing`，900 步（含动作结束后的保持段）；不需要下载素材 |
| `motion.consistency_go2`（新增，需要参考机器人，否则跳过） | Go2 场景 + `lie_stand_sit`，3000 步，含 PD 层与地面接触 |
| `headless.run_motion`（新增） | 只给 `--motion`：使用推荐场景 `keyframe_arm`；运行 1250 步（1.5 s + 1 s）；运行代次为 2（场景的初始关键帧与动作的起始关键帧各重置一次）；`config` 中带 `motion` |
| `headless.motion_invalid`（新增） | 有错误的动作文件：退出码 4，打印“动作错误 /interpolation” |
| `headless.motion_wrong_scene`（新增） | 动作与指定的场景不匹配：退出码 4，打印找不到执行器的位置 |
| `headless.motion_and_model`（新增） | `--motion` 与 `--model` 同用：退出码 2 |
| `headless.cli_parser`（扩充） | `--motion` 的解析：单独使用、与 `--scene` 和 `--steps` 组合、与 `--model` 互斥、重复、空值、步数与时长同时给出 |

**测试总数：**渲染关闭 57 → **63**，渲染开启 84 → **90**。

**变异测试 5 项**（`scripts/verify/m3_05.sh`，支持 `--mutation-only`）：
- Headless 不下发动作；不按起始关键帧重置；不用 PD 层（由 Go2 的一致性测试发现）；
- 默认时长不加 1 秒；
- `--motion` 与 `--model` 不互斥。

## 4. 验证记录

开发机：未编译、未运行（按约定）。已做的静态检查：
- 变异锚点全部唯一，所有脚本合计 168/168；
- 脚本通过 `bash -n`。

**基线机器：**未执行。
