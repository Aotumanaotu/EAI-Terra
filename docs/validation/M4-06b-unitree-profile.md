# M4-06b ROS2 桥接：宇树配置（与真机相同的低层话题）

> 状态：**已实现并通过基线验证**（2026-11-24，第 6.2 节）。依据：[M4-05 ROS2 接入设计稿](M4-05-ros2-design.md)（2026-11-24 评审通过）第 1、4～7 节；通用部分见 [M4-06a](M4-06a-ros2-bridge.md)。
> 资料来源：`unitreerobotics/unitree_ros2` 固定提交 `668d1ec5`（BSD-3-Clause）中的消息定义、CRC 代码（`motor_crc.cpp`、`motor_crc_hg.cpp`）与示例（`go2_stand_example.cpp`、`g1_low_level_example.cpp`）。

## 1. 交付

**通信配置 `kind: "unitree"`：**新增 `unitree` 字段（严格校验）。

| 字段 | 含义 |
| --- | --- |
| `messages` | `go`（unitree_go，20 个电机位）或 `hg`（unitree_hg，35 个电机位） |
| `instance` | 场景中的模型实例名 |
| `motors` | 电机编号 → 关节名（实例内），按真机的电机顺序；空字符串表示该编号没有电机 |
| `imu` | 机身 IMU（实例内的传感器名） |
| `feet`、`base` | go：`foot_force` 的四个足端实体（FR、FL、RR、RL）与 sportmodestate 的机身 |
| `lowstateHz`、`lfLowstateHz`、`sportModeStateHz`、`lfSportModeStateHz`、`wirelessControllerHz` | 500、50、500、50、50（0 表示不发布） |
| `checkCrc` | 默认 true；关闭只用于调试 |
| `modeMachine` | hg 的 LowState.mode_machine（待真机核对，默认 0） |

**四份配置**（`assets/ros2/profiles/`）：

| 配置 | 消息 | 场景 | 电机 | IMU |
| --- | --- | --- | --- | --- |
| `unitree_go2` | go | `go2_lowlevel`（新） | 12（FR、FL、RR、RL × 髋、大腿、小腿） | `go2/imu`（场景文件添加） |
| `unitree_go2w` | go | `go2w` | 16（12 + 4 个轮子，编号 12～15，顺序同 unitree_mujoco） | `go2w/imu` |
| `unitree_g1` | hg | `g1_lowlevel`（新） | 29（G1JointIndex 顺序） | `g1/imu_in_pelvis` |
| `unitree_as2` | hg | `as2` | 12 | `as2/imu` |

Menagerie Go2 的执行器顺序是 FL、FR、RL、RR，与真机的 FR、FL、RR、RL 不同。因此电机表按关节名对应，不按执行器顺序。

**话题**（与官方示例的默认 QoS 一致：reliable、深度 10；`/lowcmd` 用 best effort 订阅，reliable 与 best effort 的发布者都能匹配）：

| 话题 | 方向 | 消息 | 仿真中的来源 | 状态 |
| --- | --- | --- | --- | --- |
| `/lowstate` | 发布 | go / hg 的 LowState | 见下方“LowState 字段”；`tick` 为仿真毫秒；`crc` 见第 2 节 | 已提供；频率 500 Hz 待真机核对 |
| `/lf/lowstate` | 发布 | 同上 | 同上 | 50 Hz，待真机核对 |
| `/lowcmd` | 订阅 | go / hg 的 LowCmd | 每一步 τ = tau + kp (q − q̂) + kd (dq − dq̂)；mode = 0 时为 0；截断到执行器控制范围；保持到下一条 | 已提供；CRC 不对、含非有限值的拒绝 |
| `/sportmodestate`、`/lf/sportmodestate` | 发布 | `unitree_go/SportModeState`（仅 go） | 机身位置、速度、高度、偏航角速度、IMU、足端力；步态等字段为 0 | 仿真值，部分字段为 0 |
| `/wirelesscontroller` | 发布 | `unitree_go/WirelessController`（仅 go） | 全 0 | 仿真值 |
| `/api/sport/request` 等高层服务 | — | — | 不提供：背后是真机上的运动控制器 | 不提供 |
| `/utlidar/cloud`、前置相机 | — | — | M4-09 | 以后 |

另有通用部分：`/clock`、`/joint_states`、`/tf`、IMU 等，以及 `/terra/reset`、`/terra/pause`、`/terra/step`。宇树配置不订阅 `/terra/joint_commands`。

LowState 字段：
- `motor_state[i]`：q、dq 取关节状态；`tau_est` 为执行器输出 × 传动比；温度等为仿真值；
- `imu_state`：四元数（w、x、y、z）、角速度、比力，以及由四元数换算的 rpy；
- `foot_force`：接触法向力（N），与真机的原始读数单位不同；
- `mode_pr`：回传最近一条 LowCmd 的值；`mode_machine`：取配置中的值；
- 电池、温度：仿真值。

**控制的接入：**
- LowCmd 换算的力矩在 PD 层与动作的输出**之后**并入，同一执行器以它为准（查看器与 Headless 相同；`Bridge::directCommands()`）。
- 收到第一条 LowCmd 之前，各电机为 0。Go2 关键帧中的控制量不起作用，场景若有 PD 层也让位，与真机电机未使能时一致。

**G1 的力矩执行器（设计稿问题 1）：**
- 场景文件的模型实例新增 `"torqueActuators": true`。组合时把该实例的位置执行器改为力矩执行器：增益 1、无偏置、inheritrange 置 0，控制范围取原来的力限。
- 新场景 `g1_lowlevel`：悬挂（`fixedBase`）并改为力矩执行器。不用关键帧：关键帧中的控制量是目标位置，改为力矩执行器后没有意义。
- 若对位置执行器使用宇树配置，开启桥接时报错，并提示加 `torqueActuators`。

## 2. CRC

与官方代码相同：
- 把消息逐字段放进自然对齐的 C 结构体（填充字节为 0）；
- 按 32 位小端字计算 CRC-32/MPEG-2：多项式 0x04C11DB7，初值 0xFFFFFFFF，不反转；
- 最后一个字（crc 本身）不参与。

结构体大小：

| 结构体 | 大小 |
| --- | --- |
| go LowCmd | 812 字节 |
| hg LowCmd | 1004 字节 |
| go LowState | 1180 字节（同样规则） |
| hg LowState | 2092 字节（同样规则） |

**官方代码的一个细节：**hg 的 `get_crc` 只拷贝 `reserve` 的 4 个字节，即第一个元素；其余三个元素按 0 参与计算。Terra 照做，否则官方示例在 reserve 不为 0 时会被拒绝。LowState 的 CRC 按同样规则计算，真机是否相同待核对。

## 3. 测试

**不含 ROS2：**新增 `robots.unitree_lowlevel`，检查以下内容：
- g1_lowlevel 的 29 个执行器都是增益 1 的力矩执行器，控制范围为原来的力限：左髋俯仰 ±88、左膝 ±139、左踝 ±50 N·m；
- 悬挂；控制量 1 时执行器输出 1 N·m；
- 对照场景 g1_suspended 仍是位置执行器；
- go2_lowlevel 加了 IMU，没有 PD 层。

测试总数：渲染关闭 84 → **85**，渲染开启 117 → **118**。

**含 ROS2 与宇树消息包**（需要再 source `unitree_ros2/cyclonedds_ws/install/setup.bash`）：在 M4-06a 的 121 个基础上新增 8 个，共 **129** 个。

| 测试 | 内容 |
| --- | --- |
| `ros2.profile`（扩充） | 四份宇树配置可读；G1 电机顺序；`unitree` 字段的 16 种错误 |
| `ros2.unitree_crc` | CRC 已知值；随机消息各 50 组：与照抄的官方结构体逐字节相同、CRC 相同；结构体大小；hg 的 reserve 细节 |
| `ros2.unitree_go2` | 见下方说明 |
| `ros2.unitree_g1`、`ros2.unitree_as2` | 见下方说明 |
| `ros2.unitree_official_go2`、`_go2w` | 见下方说明 |
| `ros2.unitree_official_g1` | 见下方说明 |

`ros2.unitree_go2` 检查：
- 话题、类型与 QoS；
- LowState 500 Hz（tick 每 2 ms）、几乎不丢，lf 为 50 Hz；
- CRC 与独立的 Python 实现一致；
- motor_state[i].q 与 `/joint_states` 中第 i 个关节相同（电机顺序）；IMU；
- 收到命令前力矩为 0，机器人趴下；
- 本脚本按官方算法发 LowCmd，Go2 站起，四足着地；
- CRC 不对的（kp = 0）与含 NaN 的命令被拒绝，机器人保持站立；
- mode = 0 时没有力矩，机器人趴下；
- 拒绝条数与原因。

`ros2.unitree_g1`、`ros2.unitree_as2` 检查：
- hg 的话题与 CRC；
- 收到命令前力矩为 0；
- 关节跟踪；`mode_pr` 回传；
- reserve[1..3] 不为 0 的命令按官方 CRC 被接受；CRC 不对的被拒绝。

`ros2.unitree_official_go2`、`_go2w`：**原样运行**官方 `go2_stand_example`。
- 机器人按示例的时间表由趴下到站起、再趴下；
- 站立时 12 个电机跟到示例的 `target_pos_2`；
- 示例的 LowCmd 全部被接受，0 条拒绝，说明 CRC 与官方一致。

`ros2.unitree_official_g1`：**原样运行**官方 `g1_low_level_example`。
- 两侧脚踝摆动（峰峰值 > 0.3 rad）；手腕横滚摆动；其余关节在 0 附近；
- 0 条拒绝。

官方示例没有编译时，这 3 个测试跳过（返回 77），验证脚本把“跳过”记为失败项。

**变异（9 个）：**

| 变异 | 由哪个测试发现 |
| --- | --- |
| 不校验 CRC | `ros2.unitree_go2` |
| 力矩的位置项符号反了 | `ros2.unitree_g1` |
| mode = 0 时仍出力 | `ros2.unitree_go2` |
| 收到命令前不让位 | `ros2.unitree_go2` |
| hg 的 reserve 全部参与 CRC | `ros2.unitree_crc` |
| CRC 多项式错误 | `ros2.unitree_crc` |
| LowState 的 q 取错关节 | `ros2.unitree_go2` |
| Headless 不合并 LowCmd 的力矩 | `ros2.unitree_go2` |
| torqueActuators 不改执行器 | `robots.unitree_lowlevel` |

**展示：**查看器依次打开 Go2 与 G1，并原样运行对应的官方示例。

## 4. 已知的限制与待真机核对

- **待真机核对：**
  - 话题频率；
  - LowState 的 CRC 布局；
  - `mode_machine` 的取值；
  - IMU 四元数的分量顺序（按 SDK 惯例为 w、x、y、z）。
- 仿真值：
  - `foot_force` 是接触法向力（N），不是真机的原始读数；
  - 电池、温度、无线手柄为固定值；
  - sportmodestate 的步态、足端位置等为 0。
- 没有电机模型（转矩–转速曲线、摩擦、延迟），力矩只截断到力限。
- 宇树配置按实例名找关节：场景中的实例名必须与配置的 `instance` 相同。
- 查看器的“ROS2 桥接”菜单仍只开通用配置，宇树配置用 `--ros2` 指定。

## 5. 文件

- 新增：
  - `engine/ros2/src/{unitree.cpp,unitree_link.hpp,unitree_messages.hpp}`；
  - `assets/ros2/profiles/unitree_{go2,go2w,g1,as2}.terra-ros2.json`；
  - `assets/scenes/robots/{go2_lowlevel,g1_lowlevel}.terra-scene.json`；
  - `tests/ros2/{test_unitree_crc.cpp,check_unitree.py}`、`tests/robots/test_unitree_lowlevel.cpp`；
  - `scripts/verify/m4_06b.sh`。
- 修改：
  - `engine/ros2/{CMakeLists.txt,include/terra/ros2/bridge.hpp,src/bridge.cpp,src/profile.cpp}`；
  - `engine/physics/include/terra/physics/world_composition.hpp`、`engine/physics/backends/mujoco/src/mujoco_world.cpp`；
  - `engine/scene/{include/terra/scene/composition.hpp,src/composition.cpp,src/scene_file.cpp}`；
  - `apps/headless/{main.cpp,cli.cpp}`、`apps/viewer/main.cpp`；
  - `tests/CMakeLists.txt`；
  - `tests/ros2/test_ros2_profile.cpp`、`tests/scene/test_scene_file.cpp`、`tests/physics/test_robot_description.cpp`。

## 6. 验证记录

### 6.1 开发机

未编译、未运行（按约定）。已做的静态检查：
- 变异锚点合计 253/253 唯一；
- 脚本通过 `bash -n`；Python 语法检查通过；
- CRC 已知值用 Python 核对：CRC-32/MPEG-2("123456789") = 0x0376E6E7，与参考值一致。

### 6.2 基线机器

**第一次运行（2026-11-24）：含 ROS2 的构建编译失败。**
- 通过的部分：
  - 不含 ROS2：85/85、118/118、118/118；ASan 94/94 + 24/24；
  - 官方示例已找到；
  - 变异 9/9：在不开 -Werror 的含 ROS2 构建中，全部测试都能编译运行，9 个变异都被发现。
- 失败：`profile.cpp` 中 `named += m.empty() ? 0 : 1;` 触发 -Wsign-conversion（int 条件表达式加到 size_t）。
- 已修正：该处与同类的 `slots` 改为无符号常量。
- Ninja 在第一个错误处停止，其余文件是否还有警告未知。请用户先以 `-k 0` 编译一次，列出全部警告。

**第二次运行（2026-11-24）：129 项中 127 项通过；两个失败都在检查脚本中。**
- 通过：
  - 不含 ROS2：85/85、118/118，ASan 94/94 + 24/24；
  - 变异 9/9；展示两段都正常结束。
  - 官方示例原样运行，全部命令被接受（0 条拒绝）：
    - Go2：示例前 0.094 m → 站立 0.333 m（关节最大误差 0.094 rad）→ 趴下 0.077 m；
    - Go2W：0.167 → 0.413 → 0.092 m；
    - G1：4～8 s 内左踝俯仰、横滚的峰峰值各 0.494 rad，左手腕 0.989 rad，与示例的 0.5、1.0 一致。
  - `ros2.unitree_go2`：LowState 1000/1000 条、电机顺序与 `/joint_states` 相差 1.2e-7 rad；站起 0.334 m，足端力 35～40 N。
- 未通过一：`ros2.unitree_g1`、`ros2.unitree_as2` 中“terra_simulator 发布 /lowstate”等 3 项（Release 构建；ASan 构建中通过）。
  - 原因：`get_publishers_info_by_topic` 返回的端点节点名经 `ros_discovery_info` 另行传播，可能晚于端点本身被发现，刚连上时节点名还未知。
  - 已修改：QoS 与订阅者检查最多等 5 s，直到节点名出现（通用的 `check_qos` 同样修改）。
- 未通过二：G1 的“各关节跟到目标”，最大误差 0.156 rad，阈值 0.1。
  - 悬挂的 G1 手臂只有 kp 50，承受自重时稳态误差 = 重力矩 / kp，0.1 rad 的阈值对它过严；左膝 0.578（目标 0.6）正常。
  - 已修改：
    - 腿与腰（kp 100）仍要求 < 0.1；全部关节 < 0.25；左膝 < 0.05；输出误差最大的电机名；
    - 新增与重力无关的检查：各电机的 `tau_est` 符合 τ = kp (q̂ − q) + kd (dq̂ − dq)。
- 两处修改都只在 Python 检查脚本中，不需要重新编译。

**重跑（2026-11-24）：全部通过。**用户在现有构建目录中重跑，没有重新编译：
- Release：ros2.bridge_*、ros2.unitree_* 共 9 个测试各重复 3 次，全部通过；
- ASan：ros2 标签 11/11，连续两次通过。
连同第二次运行的其余结果，M4-06b 全部项目通过。

