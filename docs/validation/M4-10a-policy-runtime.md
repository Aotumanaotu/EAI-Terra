# M4-10a 推理组件与策略运行：实现与验证

> 状态：**通过**（2026-11-25 第二次基线运行，`bash scripts/verify/m4_10a.sh --skip-mutation`；变异 9/9 在第一次运行中完成）。设计见 [M4-10 设计稿（修订）](M4-10-policy-interface-design.md)。
> 用户要求（2026-11-25）：
> - `.pt` 与 `.onnx` 都要支持，推理先用 ONNX（CPU）；
> - 客户端提示需要的库文件并代为下载安装，带进度条，没有网络时警告；
> - 用户只选策略与机器人，自动配置；
> - 观测取带噪声的读数。
>
> `.pt`、导入向导与资源库中的“策略”类型在 M4-10b。

## 1. 用户能做什么

| 入口 | 行为 |
| --- | --- |
| 查看器：**仿真 → 运行策略…** | 选策略文件：<br>· `.terra-policy.json`；<br>· `.onnx`：按当前场景自动配置。<br>窗口列出自动完成的每一项、警告与无法运行的原因，按“运行”即接管对应的执行器。<br>也可以把文件拖进窗口 |
| 需要组件时 | 弹出“需要下载组件”：<br>· 名称、版本、大小、许可、用途；<br>· **下载并安装**（进度条、可取消）、**从文件安装…**、稍后；<br>· 安装完成后自动开始运行；<br>· 下载失败（没有网络、地址不可达）时**警告**，给出下载地址、sha256 与“从文件安装”的方法，可复制 |
| **设置 → 组件…** | 列出组件、状态（已安装、未安装、已损坏、需要更新）；可以下载、从文件安装、重新校验、删除 |
| 速度指令面板 | 策略运行时出现在视口左上角：<br>· 前进、侧移、转向三个滑块与“归零”；<br>· **键盘驾驶**：按住 I / K、J / L、U / O，松开即停；W/A/S/D 仍用于相机，按键与 ROS 的 teleop_twist_keyboard 相同；<br>· 显示推理次数、耗时与后端 |
| 传感器面板的“噪声”开关 | 同样作用于策略的观测 |
| Headless | `--policy <说明文件或 .onnx> [--command vx,vy,yaw] [--library <库>]`：<br>· 没有组件时退出码 3，并给出安装方法；<br>· `--install-component onnxruntime-cpu [--from <nupkg>]`：进度写 stderr；<br>· `--list-components` |
| 实验记录 | 记录头新增 `policy`：路径、sha256、模型 sha256、后端。<br>复现按记录的控制量，**不需要推理库** |

## 2. 实现

| 部分 | 文件 | 说明 |
| --- | --- | --- |
| 组件管理 | `engine/library/{include/terra/library/components.hpp, src/components.cpp}`、`assets/components/components.json` | 清单、状态、curl 下载（`file://` 也可）、压缩包与逐个文件的 sha256、先解到隐藏的临时目录再改名、取消、手动安装说明。<br>装在资源库目录下的 `components/<标识>/` |
| ONNX Runtime | `third_party/onnxruntime/`（只有 C API 头文件与许可，MIT） | 1.32.0，来自 NuGet（SHA-512 与 NuGet 目录一致）。<br>运行时 `dlopen`，构建不依赖它 |
| 说明文件 | `engine/policy/{include/terra/policy/policy.hpp, src/policy_file.cpp}` | `*.terra-policy.json` 的读写；不认识的字段警告并原样保留 |
| ONNX 信息 | `src/onnx_info.cpp`、`scripts/policy/onnx_ops.py` | 直接读 protobuf：输入输出、算子、参数量、元数据（C++ 与 Python 两份） |
| 推理后端 | `include/terra/policy/backend.hpp`、`src/backend.cpp`、`src/onnxruntime_backend.cpp` | 注册表：<br>· `onnxruntime-cpu`：单线程、顺序执行、`SetDeterministicCompute`；<br>· `linear`：测试用。<br>组件不在时返回 `missingComponent` |
| 自动配置 | `include/terra/policy/autoconfig.hpp`、`src/autoconfig.cpp` | 格式解析器注册表（本步：`supermimic_moe_velocity_policy_v1`）。<br>训练配置 `task_config.json` 在模型目录及上两级查找。<br>关节名对应、机身与 IMU 自动选择 |
| 控制器 | `include/terra/policy/controller.hpp`、`src/controller.cpp` | 降频、历史帧、相位、positionPd 等；**IMU 读数经 M4-09 的 SensorNoise，并按 IMU 频率保持** |
| Headless | `apps/headless/{cli.*, main.cpp}` | 策略的命令在 PD 层与 ROS2 命令之后，同一执行器以策略为准 |
| 查看器 | `apps/viewer/{policy_panel.*, main.cpp}`、`engine/ui` 的文本表（52 条） | 见第 1 节 |
| 场景 | `assets/scenes/robots/go2_policy.terra-scene.json` | Go2 低层控制（力矩执行器）。<br>机身 IMU 500 Hz，带陀螺白噪声：由训练配置 `noise_scales.base_ang_vel = 0.2` 换算，见场景名中的说明 |

### 2.1 你的 Go2 策略：自动配置得到什么

（ONNX 元数据与 `task_config.json`，只读；以基线运行的输出为准）

- **关节：**12 个，`FL_hip_joint` → `go2/FL_hip_joint` 等；
- **默认姿态与动作缩放：**取自元数据（hip 0.25，thigh、calf 0.5）；
- **PD：**kp 为 hip 20、thigh 20、calf 40；kd 为 1、1、2；力矩上限 23.7、23.7、45.43 N·m；
- **控制周期：**0.02 s（每 10 个物理步推理一次）；
- **历史：**5 帧 × 47 维；
- **观测：**
  - 角速度、重力方向取 `go2/imu`（带噪声）；
  - 关节位置减默认姿态、关节速度；
  - 上一次的原始输出；
  - 指令 (vx, vy, 偏航角速度)；
  - 步态相位：周期 0.6 s，指令小于 0.1 时为 0；
- **指令范围：**取训练配置 `commands.initial_ranges`（vx ±1，vy ±1，偏航 ±2）。

**sim2sim 的差别（作为警告列出）：**

| 项 | 训练时 | Terra |
| --- | --- | --- |
| 物理步长 | 0.005 s（mjlab，MuJoCo 3.11） | 0.002 s（MuJoCo 4.0.1） |
| 编码器偏置 | 训练观测含偏置（域随机化） | 偏置为 0 |
| 关节阻尼与摩擦 | 训练模型的设置 | Menagerie 的 Go2（阻尼 2、摩擦损失 0.2） |
| 重力方向观测的噪声 | 有 | 没有（M4-09 的 IMU 噪声只加在比力与角速度上） |
| 关节编码器噪声 | 有 | 没有（M4-09 未提供） |

## 3. 测试

| 测试 | 内容 | 需要 |
| --- | --- | --- |
| `library.components` | 清单校验、从 `file://` 地址与本地文件安装、进度、篡改检测、版本、删除；sha256 不对、地址不可达、取消 | — |
| `policy.file` | 说明文件的读取、往返、保留字段、8 类错误；ONNX 信息；后端注册表、linear 推理、sha256 警告、宽度不一致；没有组件时的 `missingComponent`；不认识的格式与 `.pt` 的说明 | — |
| `policy.controller` | 夹具 `policy_bench`：<br>· 重力方向的解析值；<br>· IMU 50 Hz 保持；噪声开时不为 0，关时为真值；<br>· 历史帧顺序；相位；<br>· 力矩截断与 gear 换算；<br>· 重置；不匹配的三种情况；<br>· 两次运行逐位相同。<br>另有自动配置：bench_supermimic 加训练配置 | — |
| `policy.onnxruntime` | gemm_elu 的输出与手算一致、两次逐位相同、输入名不对时的提示；ONNX Runtime 与线性模型逐步一致 | 组件（没有时跳过） |
| `policy.go2_walk` | 你的策略：<br>· 站立 10 s；<br>· 前进 0.5 m/s 时测 10 s 的平均速度与侧向速度；<br>· 转向 1.0 rad/s 时的偏航角速度；<br>· 阈值按第一次结果冻结（第 4 节）；<br>· 3 s 两次运行逐位相同。<br>指标写入 `policy_go2_metrics.json` | 组件与 `others/` 中的策略 |
| `headless.policy` | 两次运行逐字节相同；`--command` 与 `--no-noise` 改变运动；记录头与复现逐位相同；组件不在时退出码 3；组件命令；装有组件时用 ONNX 运行；你的 Go2 策略运行 2 s，记录复现逐位相同 | ONNX 部分需要组件 |
| `graphics.viewer_policy_selftest` | 命令行 `--policy` 自动运行、按控制周期推理、速度指令面板、面板中的指令传给策略、停止 | — |
| `apps.headless_cli`（扩充） | `--policy`、`--command`、`--install-component`、`--from`、`--list-components` 的解析与组合限制 | — |

预期数量：

| 构建 | 预期 |
| --- | --- |
| 渲染关闭 Debug | 108 |
| 渲染 Debug 与 Release | 147 |
| 含 ROS2 | 161 |
| ASan 非图形 | 117，其中 2 个跳过：ASan 构建没有装组件 |
| ASan 图形 | 30 |

变异 9 个：历史顺序、站立相位、力矩截断、lastAction、IMU 保持、观测噪声、自动配置的 kp / kd、`--command`、组件的文件 sha256。

## 4. 基线运行结果

### 4.1 第一次运行（2026-11-25）

| 项 | 结果 |
| --- | --- |
| ONNX Runtime 组件 | 从 NuGet 下载并安装，9 行进度，sha256 一致 |
| 渲染关闭 Debug | 107/108 |
| 渲染 Debug、Release -Werror | 146/147 |
| 含 ROS2 Release -Werror | 160/161 |
| ASan 非图形 | 116/117，2 个按预期跳过 |
| ASan 图形 | 30/30 |
| ASan 含 ROS2 | 14/14 |
| 变异 | 9/9 |
| 可交互展示 | 退出码 0；你的策略在 Go2 场景中自动配置并运行，停止、重新运行正常 |

**唯一的失败：** `headless.policy` 中的“`--no-noise` 改变运动”，各构建相同。
- **原因在测试：**夹具线性模型的输出只取重力方向与指令。重力方向来自 IMU 姿态，M4-09 没有给姿态加噪声，所以噪声开关改变不了它的输出，运动当然相同。
- **控制器本身没有问题：**`policy.controller` 中“噪声开：角速度不为 0”“噪声关：为真值”都通过，变异“观测不加噪声”也被发现。
- **修正：**这项检查改用一个输出含角速度的线性模型（测试运行时生成在工作目录中）。另加一项：原夹具在 `--no-noise` 下运动不变，说明噪声只加在比力与角速度上。

### 4.2 第二次运行（2026-11-25，跳过变异）

| 项 | 结果 |
| --- | --- |
| ONNX Runtime 组件 | 已安装，沿用第一次下载的组件，没有重复下载 |
| 渲染关闭 Debug | 108/108 |
| 渲染 Debug | 147/147 |
| 渲染 Release -Werror | 147/147 |
| 含 ROS2 Release -Werror | 161/161 |
| ASan 非图形 | 117/117，2 个按预期跳过 |
| ASan 图形 | 30/30 |
| ASan 含 ROS2 | 14/14 |
| 可交互展示 | 退出码 0 |

修正后的两项检查都通过：
- `--no-noise` 改变运动（观测取带噪声的角速度）；
- 原夹具在 `--no-noise` 下运动不变。

Go2 指标与第一次逐位相同，单次推理约 0.07 ms。

### 4.3 你的 Go2 策略：指标与冻结的阈值

同一机器上两次运行逐位相同；Headless 记录复现逐位相同（复现不需要推理库）。

| 项 | 第一次结果 | 冻结的阈值 |
| --- | --- | --- |
| 站立 10 s：最低高度 | 0.270 m（关键帧初始高度 0.27） | > 0.25 m |
| 站立：最大倾角 | 2.5° | < 10° |
| 站立：水平漂移 | 0.009 m | < 0.05 m |
| 前进指令 0.5 m/s：平均前进速度 | 0.382 m/s | 0.3～0.6 m/s |
| 前进：侧向速度 | −0.001 m/s | 绝对值 < 0.05 m/s |
| 前进：最低高度 / 最大倾角 | 0.286 m / 4.9° | > 0.2 m / < 15° |
| 转向指令 1.0 rad/s：平均偏航角速度 | 0.657 rad/s | 0.4～1.2 rad/s |
| 单次推理（ONNX Runtime，单线程） | 约 0.1 ms | 只记录 |

- **跟踪不足：**前进约 76%，转向约 66%。可能的原因是第 2 节列出的 sim2sim 差别，以步长和关节阻尼的嫌疑最大；
- 训练中有航向控制（`heading_command`），Terra 直接给偏航角速度，也可能使转向偏小；
- 这些差别留给评估报告（开发计划 11.9 节第 1 层）逐项分析。

**另一个观察：**展示中你打开了 Go2W 场景，策略也运行了：Go2W 的腿部关节与 Go2 同名，自动配置把它当成了匹配的机器人。M4-10b 的向导要在机器人与训练时的不同（关节数、质量、多出的轮子）时给出明确的警告。

## 5. 未完成与后续

- **M4-10b：**
  - 导入向导（选策略与机器人卡片，按关节名推荐场景）；
  - `.pt`：TorchScript 经 PyTorch 转为 ONNX；训练检查点找同一 `spec_hash` 的 ONNX；
  - 资源库中的“策略”类型与场景卡片上的“策略”标签；
  - 需要力矩执行器时生成场景副本。
- **ROS2 的 `/cmd_vel` 作为策略指令：**未做，放在 M4-10b；桥接中已有差速底盘的 `/cmd_vel` 订阅，可复用。
- **机器人不同的警告：**策略与机器人不完全对应时（如 Go2W 多出轮子）由向导警告（M4-10b）。
- **GPU 推理：**ONNX Runtime GPU 作为新的组件加入清单（用户：以后再说）。
