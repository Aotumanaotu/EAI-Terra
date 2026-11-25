# M4-05 ROS2 接入设计与宇树的真机话题清单

> 状态：**评审通过**（2026-11-24，第 7 节 6 个问题全部按建议）。用户另要求把**宇树 A2W 轮足机器人**加入这套验证（第 8 节，待用户确认模型来源）。依据：开发计划 11.3 节第 3～6 条（Humble；进程内的可选模块；`JointState` 命令；坐标系命名）、11.5 节（Sim-to-Real：通信层与真机一致，先只做宇树）、11.7 节（RViz 功能，A 层在 M4 后段）。
> 实现在 M4-06。本稿只定接口、话题清单与验证方法。

## 1. 资料来源（已核对，按提交号固定）

| 来源 | 提交 | 许可 | 用到的内容 |
| --- | --- | --- | --- |
| `unitreerobotics/unitree_ros2` | `668d1ec5a05d1c38d3306bdca7d59f2ba3581a88` | BSD-3-Clause | 消息定义（`unitree_go`、`unitree_hg`、`unitree_api`）；示例中的话题名与控制循环（`go2_stand_example.cpp`、`g1_low_level_example.cpp`）；CRC 算法（`motor_crc.cpp`、`motor_crc_hg.cpp`）；README：Humble 推荐，使用 Cyclone DDS |
| `unitreerobotics/unitree_mujoco` | `1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d` | BSD-3-Clause | 宇树官方 MuJoCo 仿真桥的做法：每步按 LowCmd 计算力矩 τ = tau + kp (q − q̂) + kd (dq − dq̂)；IMU 由模型中的传感器给出 |

**已核对的要点：**
- **话题：**低层状态为 `lowstate`（高频）与 `lf/lowstate`（低频），低层命令为 `/lowcmd`。Go2 另有 `sportmodestate` / `lf/sportmodestate`（运动状态）、`/wirelesscontroller`、`/api/sport/request`（高层运动服务）、`/utlidar/cloud`（L1 雷达点云）。在 DDS 层，ROS2 话题 `/lowstate` 即 `rt/lowstate`，与 unitree_sdk2 互通。
- **消息：**
  - Go2：`unitree_go/msg/LowState`（含 `IMUState imu_state`、`MotorState[20]`、`foot_force[4]`、`crc`）与 `LowCmd`（`MotorCmd[20]`：mode、q、dq、tau、kp、kd，`crc`）；
  - G1：`unitree_hg/msg/LowState`（`mode_pr`、`mode_machine`、`MotorState[35]`、`crc`）与 `LowCmd`（`MotorCmd[35]`、`crc`）。
- **CRC：**把消息逐字段拷进一个 C 结构体（自然对齐，填充字节为 0），按 32 位字计算 CRC-32（多项式 0x04C11DB7，初值 0xFFFFFFFF，不反转），最后一个字（crc 本身）不参与计算。官方示例每次发送前都计算 CRC。
- **关节顺序：**
  - Go2：FR_0～2、FL_0～2、RR_0～2、RL_0～2（每条腿依次为髋、大腿、小腿），共 12 个；
  - G1：`G1JointIndex` 0～28，依次为左腿 6、右腿 6、腰 3、左臂 7、右臂 7。**与 Menagerie G1 模型中 29 个关节的顺序完全一致。**
- **中间件：**Cyclone DDS（宇树 README：真机为 0.10.x；Humble 自带的 `rmw_cyclonedds_cpp` 可直接用）。基线机器已装有 `rmw_cyclonedds_cpp`，往返测试通过（LOG-134）。

**未能从资料确定、需要在真机上核对的：**
- 各话题的实际频率；
- 真机发布者的 QoS；
- `imu_state.quaternion` 的分量顺序（宇树 SDK 一般为 w、x、y、z）；
- `foot_force` 的单位（原始读数）；
- G1 的 `mode_machine` 取值。

清单中把这些标为“待真机核对”，仿真先按下面第 4 节的约定给出。

## 2. 模块与运行方式

- **新模块 `terra_ros2`：**只有打开编译选项 `TERRA_BUILD_ROS2` 且找到 ROS2 时才构建。核心库、Headless 的默认构建不链接 ROS2。没找到时，CMake 配置输出提示“未检测到 ROS2 Humble”，并给出安装说明。
- **启用：**查看器与 Headless 都加 `--ros2 <通信配置文件>`。查看器的“仿真”菜单可以开关桥接，状态栏显示连接状态。没有 ROS2 时菜单项不可用，并提示安装（11.7 节用户决定）。
- **线程：**rclcpp 的执行器在单独的线程。
  - 收到的命令放入队列，在**下一步之前**生效，与面板、动作走同一条路径（`SimulationRunner` 的命令队列）；
  - 发布在仿真线程上、每步之后进行（M4-04 的常驻步观察者）；
  - 仿真暂停时不发布状态，`/clock` 停在当前时间。
- **时钟：**每一步发布 `/clock`（仿真时间）。重置时仿真时间回到 0，ROS2 的时间会向后跳变：tf2 检测到后清空缓存，使用仿真时间的节点会收到时间跳变回调。这一点写进使用说明。
- **通信配置文件：**`assets/ros2/profiles/*.terra-ros2.json`（JSON，与场景文件相同的严格校验）。它规定发布哪些话题、什么消息、多高频率、什么 QoS、字段如何对应到 Terra 的数据，以及命令从哪里来。每份配置附一份“真机话题清单”，注明来源提交。

## 3. 通用配置（标准消息）

| 话题 | 方向 | 消息 | 频率 | 说明 |
| --- | --- | --- | --- | --- |
| `/clock` | 发布 | `rosgraph_msgs/Clock` | 每步 | 仿真时间 |
| `/joint_states` | 发布 | `sensor_msgs/JointState` | 默认 100 Hz | 名字为关节全名（如 `go2/FL_hip_joint`），位置、速度，力为执行器输出 |
| `/tf` | 发布 | `tf2_msgs/TFMessage` | 默认 100 Hz | `world` → 每个实体（坐标系名为实体全名）；`/tf_static` 发布传感器安装坐标系（相机另有光学坐标系） |
| `/<传感器名>/imu` | 发布 | `sensor_msgs/Imu` | 传感器频率 | 姿态、角速度、线加速度；协方差为 0（无噪声） |
| `/<传感器名>/scan` | 发布 | `sensor_msgs/LaserScan` | 传感器频率 | 与 M4-03b 的约定相同 |
| `/<相机名>/image_raw`、`/depth/image_raw`、`/camera_info` | 发布 | `sensor_msgs/Image`、`CameraInfo` | 传感器频率 | 彩色 rgb8；深度 32FC1（米，无命中为 +inf）；内参为 M4-03a 的约定；`frame_id` 为光学坐标系 |
| `/terra/joint_commands` | 订阅 | `sensor_msgs/JointState` | — | 名字为执行器全名；`position` 给位置执行器与 PD 层、`velocity` 给速度执行器、`effort` 给力矩执行器；越界拒绝 |
| `/terra/reset`、`/terra/pause`、`/terra/step` | 服务 | `std_srvs/Trigger`、`SetBool`、`Trigger` | — | 重置、暂停 / 继续、单步 |

QoS：传感器与状态话题用 sensor data（best effort、深度 5），命令与服务用默认（reliable、深度 10），`/tf_static` 用 transient local。

## 4. 宇树配置（Go2、G1）：与真机一致的话题

| 话题 | 方向 | 消息 | 仿真中的来源 | 状态 |
| --- | --- | --- | --- | --- |
| `/lowstate` | 发布 | Go2：`unitree_go/msg/LowState`；G1：`unitree_hg/msg/LowState` | `motor_state[i]`：按关节名对应（第 1 节的顺序），q、dq 取关节状态，`tau_est` 取执行器输出；`imu_state`：机身 IMU（Go2 加在安装点 `imu`，G1 用骨盆 `imu_in_pelvis`）的四元数、角速度、比力与由四元数换算的 rpy；`tick` 为毫秒；`crc` 按第 1 节算法计算 | 频率 500 Hz（与官方示例 2 ms 的控制循环一致），待真机核对 |
| `/lf/lowstate` | 发布 | 同上 | 同上 | 频率暂定 50 Hz，待真机核对 |
| `/lowcmd` | 订阅 | Go2：`unitree_go/msg/LowCmd`；G1：`unitree_hg/msg/LowCmd` | 每个电机：`mode` = 1 时按 τ = tau + kp (q − q̂) + kd (dq − dq̂) 在**每一步**计算力矩（与 `unitree_mujoco` 相同），截断到执行器力限；`mode` = 0 时输出 0。命令保持到下一条到达 | CRC 校验：见问题 2 |
| `/sportmodestate`、`/lf/sportmodestate` | 发布 | `unitree_go/msg/SportModeState`（仅 Go2） | 位置、速度取机身实体；`foot_force`、步态相关字段为 0 | 仿真值，部分字段为 0 |
| `/wirelesscontroller` | 发布 | `unitree_go/msg/WirelessController` | 全 0（以后可接游戏手柄，如 `unitree_mujoco`） | 仿真值 |
| `/api/sport/request` | — | `unitree_api/msg/Request` | **不提供**：背后是真机上的运动控制器，仿真中没有对应的控制器 | 不提供，清单中注明 |
| `/utlidar/cloud`、前置相机 | — | `sensor_msgs/PointCloud2` 等 | L1 是 3D 雷达，在 M4-09（P1） | 以后提供 |

**做不到或不做的（逐项列出）：**
- 电池（BMS）、电机温度、错误码、风扇：给固定的合理值，清单中标为“仿真值”；
- `foot_force` 的单位与真机的原始读数不同（真机是传感器原始值）：仿真给接触法向力（N），标为“单位不同”；
- 网络延迟与丢包：默认不加（P1 可配置）。

**控制方式：**宇树的低层控制是“每个电机的 q、dq、tau、kp、kd”，需要模型的执行器是**力矩执行器**。
- Go2 的 Menagerie 模型是力矩执行器（现在由 PD 层接管），可以直接用；启用宇树配置时 PD 层让位给 `/lowcmd`。
- G1 的 Menagerie 模型是**位置执行器**（内置 kp），与 LowCmd 的语义不同：见问题 1。

**网络：**
- 仿真默认用单独的域 ID，并只在本机通信，不会被同一网络中的真机收到；
- 连接真机或在多台机器上运行时，在配置文件中改域 ID 与网卡。用户环境中的 `ROS_LOCALHOST_ONLY=1` 届时要取消（LOG-134）。

## 5. 真机话题清单与比较

**清单文件：**`assets/ros2/profiles/unitree_go2.terra-ros2.json`、`unitree_g1.terra-ros2.json`。每个话题包含：话题名、消息类型、方向、频率、QoS、坐标系、字段对应，以及状态（已提供 / 仿真值 / 不提供 / 待真机核对），并注明来源提交。

**验收时自动比较：**仿真运行后，用 `ros2 topic list -t`、`ros2 topic info -v` 与实测频率（`ros2 topic hz`）对照清单：话题名、类型、方向、QoS 一致，频率在 ±5% 以内。

## 6. 外部示例与 Sim-to-Real 验收（M4-06）

- **宇树官方示例原样运行：**用户在基线机器上编译 `unitree_ros2`（固定提交）的消息与示例。M4-06 的验收直接运行**未修改**的官方程序：
  - `go2_stand_example`：Go2 由趴下到站起；
  - `g1_low_level_example`：G1 脚踝摆动。
  它们在真机上也是这样运行的，这正是“同一个程序不改代码，在仿真与真机之间切换”。
- **通用配置的示例：**一个 Python（rclpy）节点，读 `/joint_states` 与 IMU，用 `/terra/joint_commands` 让 Go2 站起、趴下。
- **自动化测试：**只在打开 `TERRA_BUILD_ROS2` 时构建，带 `ros2` 标签。测试在单独的域 ID 上运行，只走本机。

**需要用户准备（评审通过后，M4-06 之前）：**
```
cd ~ && git clone https://github.com/unitreerobotics/unitree_ros2.git
cd unitree_ros2 && git checkout 668d1ec5a05d1c38d3306bdca7d59f2ba3581a88
source /opt/ros/humble/setup.bash
cd cyclonedds_ws && colcon build                     # Humble 不需要单独编译 Cyclone DDS（README）
cd ../example && colcon build                         # 官方示例
```
具体步骤与检查脚本在 M4-06 中给出。宇树的仓库只在用户机器上编译，不放进 Terra 仓库，Terra 用 `find_package(unitree_go)` 找到它。

## 7. 需要评审的问题

| # | 问题 | 建议 | 其他选项 |
| --- | --- | --- | --- |
| 1 | G1 的执行器与 LowCmd 不一致（Menagerie 是位置执行器） | 启用宇树配置时，组合模型的过程中把 G1 的位置执行器改为**力矩执行器**（增益 1、无偏置，力限沿用），由 LowCmd 的 τ = tau + kp (q − q̂) + kd (dq − dq̂) 驱动，与真机和 `unitree_mujoco` 一致；不启用时模型不变 | 改用 `unitree_mujoco` 自带的 G1 模型（需要再下载、锁定一套模型，外观与碰撞也不同） |
| 2 | `/lowcmd` 的 CRC 不对时 | 与真机一样**拒绝**该命令（保持上一条），计数并在日志中提示；配置文件可以关闭校验，便于调试 | 接受并警告（与真机不同，可能让在仿真中能用的程序到真机上失效） |
| 3 | 话题频率 | `lowstate` 500 Hz、`lf/lowstate` 50 Hz、`sportmodestate` 500 Hz、`lf/sportmodestate` 50 Hz；有真机后用 `ros2 topic hz` 核对并修改清单 | 全部按物理频率发布 |
| 4 | 发布在哪个线程 | 仿真线程上、每步之后直接发布（rclcpp 的发布是线程安全的）；收到的命令经队列在下一步之前生效 | 全部交给 ROS 线程（多一次复制，时序更难保证） |
| 5 | 验收用的外部示例 | 宇树官方示例原样运行（Go2 站起、G1 脚踝摆动），加一个通用配置的 Python 示例 | 只用自己写的示例（不能证明“不改代码”） |
| 6 | 宇树消息包的来源 | 用户在基线机器上按固定提交编译 `unitree_ros2`，Terra 用 `find_package` 找到；不放进 Terra 仓库 | 把消息定义复制进 Terra 仓库（许可允许，但两份定义可能不同步） |

## 8. 宇树 A2W 轮足机器人（用户 2026-11-24 要求加入）

**公开资料的核查结果（2026-11-24）：**

| 仓库 | A2（腿足） | A2W（轮足） |
| --- | --- | --- |
| `unitree_mujoco`（`1eb6642e`） | 有 `unitree_robots/a2/a2.xml`：12 个腿关节，力矩电机（髋、大腿 ±120 N·m，小腿 ±180 N·m） | **没有**（只有 `a2`、`as2` 与轮足的 `go2w`、`b2w`） |
| `unitree_ros` | 有 `robots/a2_description`（URDF 与 MJCF） | **没有** |
| `unitree_sdk2` | 有 `example/a2`（高层运动与音频示例） | 没有单独的示例 |
| `unitree_ros2`（`668d1ec5`） | 没有 A2 专门的内容 | 没有 |

- A2 的低层消息用 `unitree_go` 还是 `unitree_hg`，公开示例中看不出来（只有高层运动接口），需要拿到模型或真机后核对。
- 轮足机型（Go2W、B2W）的轮子关节接在低层电机数组中，按电机命令驱动（速度或力矩）；A2W 应同理，但关节数与顺序要以官方资料为准。

**可选做法（待用户决定）：**
1. 用户能拿到 A2W 的官方模型（宇树提供或随机器附带）：放入 `ROBOTS/`，按 M3-01 的做法锁定、导入，加入宇树配置；
2. 先用公开的 A2（腿足版）做这套验证，等 A2W 模型公开或拿到后再加入；
3. 自己在 A2 上加轮子：尺寸、质量、电机参数都没有依据，不能保证与真机一致，**不建议**。

**补充核查（2026-11-24）：**
- **AS2**（`unitree_mujoco/unitree_robots/as2`）：12 个腿关节、四个脚，**没有轮子**，是腿足四足；官方仿真桥对它自动选用 `unitree_hg` 消息。
- **开源的轮足四足**：`unitree_mujoco`（`1eb6642e`，BSD-3-Clause）中有两款。
  - **Go2W**：16 个关节（每条腿髋、大腿、小腿加一个轮子），腿部电机 ±23.7 / ±45.4 N·m，轮子 ±15 N·m；IMU 传感器 `imu_quat`、`imu_gyro`、`imu_acc`。与已接入的 Go2 同一系列。
  - **B2W**：16 个关节，腿部 ±200 / ±300 N·m，轮子 ±20 N·m；工业级，体型大。
- **建议：**以 **Go2W** 代替 A2W 作为轮足机型加入宇树配置（与 Go2 同系列，便于对照），AS2 可作为第二台用 `unitree_hg` 消息的四足（与 G1 共用消息，验证 hg 路径在四足上的用法）。等 A2W 模型公开后再加入。

## 9. Go2W 与 AS2 的锁定与导入核查（2026-11-24，实现）

**用户决定（2026-11-24）：**宇树配置中加入 Go2W（轮足，代替尚未公开模型的 A2W）与 AS2（用户：“用这个也行”）。用户已按第 8 节的命令下载到 `ROBOTS/unitree_mujoco/`。

**核对：**用仓库文件清单中每个文件的 git blob 哈希逐个比对，Go2W 28 个、AS2 20 个文件都与固定提交 `1eb6642e` 一致，没有缺少或多出的文件。

**锁定：**`assets/robots/robots.lock` 追加 46 个文件：
- `unitree_go2w`：`go2w.xml`、`scene.xml`（地面与几级台阶）、`assets/` 中的网格；
- `unitree_as2`：`as2.xml`、`plane_terrain.xml`（平地）、`meshes/`；
- 各目录复制仓库根目录的 LICENSE（BSD-3-Clause）；
- 不含高度图地形（`scene_terrain.xml`、`height_field.png` 等）。

锁定记录新增第 5 个字段（原件相对 `ROBOTS/` 的路径）；`scripts/assets/fetch_robots.sh` 优先从该路径复制，没有时从固定提交下载。

**场景文件：**`assets/scenes/robots/go2w.terra-scene.json`、`as2.terra-scene.json`。
- 两个模型都没有关键帧，机身从 0.6 m / 0.5 m、各关节为 0 的姿态开始，没有命令时落地趴下。这与宇树官方仿真相同：官方的 `go2_stand_example` 正是从趴着的状态站起来。
- 场景文件不加 PD 层，M4-06 由 `/lowcmd` 驱动。

**模型要点：**

| | Go2W | AS2 |
| --- | --- | --- |
| 实体 / 关节 / 执行器 | 17 / 17（1 个自由关节）/ 16 | 17 / 13 / 12 |
| 执行器 | 全部为力矩电机（腿 ±23.7 / ±45.4 N·m，轮子 ±15 N·m） | 全部为力矩电机（±60 / ±90 N·m） |
| 显示 / 碰撞分组 | group 2 / group 3 | group 1 / group 3 |
| IMU | 安装点 `imu` 上的 framequat、gyro、accelerometer | 同左 |
| 物理选项 | `cone="elliptic"`、`impratio="100"` | 默认 |

**测试：**`robots.unitree_import`（新）。
- 实体、关节、执行器的数量；执行器全部是作用于单个关节的力矩执行器；
- 腿部关节名齐全（宇树配置按名字对应电机编号），Go2W 另有 4 个没有限位的轮子关节；
- IMU 被发现；分组与网格；
- 零控制 2 s 后机身停在地面以上，与地面有接触。

**测试总数：**渲染关闭 82 → **83**，渲染开启 115 → **116**。素材导入不涉及引擎代码，按 M3-01 的做法不做变异测试，改为“锁定的反向检查”（改坏一个已复制的文件，获取脚本必须发现并恢复）。

**验证（2026-11-24，`bash scripts/verify/m4_05.sh` 第一次运行）：全部通过。**
- 参考机器人 245/245 个文件（从 ROBOTS/ 复制 44 个，LICENSE 2 个从固定提交下载）；83/83、116/116（Debug 与 Release -Werror）、ASan 92/92 + 24/24；锁定的反向检查：改坏的 `go2w.xml` 被发现并从 `ROBOTS/unitree_mujoco/` 恢复。
- 导入核查表：

| | 实体 | 关节 | 执行器 | 几何体（显示 / 碰撞） | 网格 | 零控制 2 s 后机身高度 | 接触点 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Go2W | 17 | 17（自由 1） | 16（力矩 16） | 61（group 2：25，group 3：23） | 17 个、28.4 万个三角形 | 0.167 m（初始 0.60） | 22 |
| AS2 | 17 | 13（自由 1） | 12（力矩 12） | 49（group 1、group 3：27） | 17 个、61.6 万个三角形 | 0.064 m（初始 0.50） | 8 |

- 两台的步长 0.002 s、Euler、Newton（100 次迭代）；查看器展示 96 秒正常。
