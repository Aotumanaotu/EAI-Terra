# M4-06a ROS2 桥接：通用配置

> 状态：**已实现并通过基线验证**（2026-11-24，第 5.2 节；含 ROS2 的 ASan 在关闭 rclcpp 上游缺陷对应的一项检查后通过）。依据：[M4-05 ROS2 接入设计稿](M4-05-ros2-design.md)（2026-11-24 评审通过）第 3 节与第 7 节的 6 个决定。宇树配置在 M4-06b。

## 1. 交付

**构建：**
- 新选项 `TERRA_BUILD_ROS2`，默认关闭。
- 打开后由 `cmake/TerraRos2.cmake` 查找 ROS2 Humble：
  - 找到时构建 `terra_ros2`（`engine/ros2/`），查看器与 Headless 定义 `TERRA_WITH_ROS2` 并链接它；
  - 找不到时给出警告与安装说明，照常构建不含 ROS2 的版本。
- 公共头 `terra/ros2/bridge.hpp` 不含任何 ROS 类型，rclcpp 与消息包只在 `engine/ros2/src/` 中使用（`boundary.render_isolation` 检查）。
- 核心库、物理、渲染都不链接 ROS2。

**通信配置文件** `*.terra-ros2.json`（format `terra.ros2`，version 1，严格校验，未知字段报错）：
- 字段：`name`、`kind`（本步只有 `generic`）、`topicPrefix`、`clock`、`jointStatesHz`、`tfHz`、`sensors`、`commands`、`services`；
- 仓库自带 `assets/ros2/profiles/generic.terra-ros2.json`。

**话题与服务（通用配置）：**

| 名称 | 类型 | QoS | 说明 |
| --- | --- | --- | --- |
| `/clock` | rosgraph_msgs/Clock | ClockQoS（best effort） | 每步 |
| `/joint_states` | sensor_msgs/JointState | 可靠 | 100 Hz；转动与滑动关节；effort 为作用在该关节上的第一个执行器的力 |
| `/tf` | tf2_msgs/TFMessage | 可靠 | 100 Hz；`world` → 各实体（全名） |
| `/tf_static` | tf2_msgs/TFMessage | 可靠 + transient local | 传感器安装坐标系；相机另有 `<相机>_optical`（绕 X 轴 180°） |
| `/<IMU>/imu` | sensor_msgs/Imu | sensor data | 按场景文件的频率；姿态取自 IMU 读数 |
| `/<雷达>/scan` | sensor_msgs/LaserScan | sensor data | 按频率；没有命中为 ±inf（REP-117） |
| `/<相机>/image_raw`、`/<相机>/depth/image_raw`、`/<相机>/camera_info` | Image（rgb8 / 32FC1）、CameraInfo | sensor data | 只有查看器发布（需要渲染）；坐标系为 `<相机>_optical`；图像不按 `displayRotationDeg` 旋转 |
| `/terra/joint_commands`（订阅） | sensor_msgs/JointState | 可靠 | 名字为执行器全名；位置执行器与 PD 层接管的取 position，速度执行器取 velocity，其余取 effort；未知名字、缺字段、越界、非有限值的拒绝并计数 |
| `/terra/reset`、`/terra/step` | std_srvs/Trigger | | 重置回到场景的初始状态（有初始关键帧时为关键帧）；单步只在暂停时有效 |
| `/terra/pause` | std_srvs/SetBool | | true 暂停，false 继续 |

- 时间戳都是仿真时间。
- 话题名中不允许的字符（如 “-”）换成 “_”。

**Headless：**
- 新参数 `--ros2 <配置>`，隐含新参数 `--realtime`（按墙钟节奏推进）。
- 需要场景文件；不能与 `--model`、`--dry-run`、`--rerun`、`--list-records` 同用。
- 暂停时等待继续或单步。
- 结束时在标准错误打印发布数、命令的接受与拒绝数，以及最近一次拒绝的原因。
- 不含 ROS2 的构建：提示安装 ROS2 Humble 并返回 3。

**查看器：**
- 新参数 `--ros2 <配置>`：打开场景后开启桥接并直接运行；失败时返回 3。
- 菜单“仿真 → ROS2 桥接”可开关，默认使用通用配置。开着时打开其他场景，以同一配置重新开启。
- 状态栏显示 ROS2 订阅者数。
- 服务请求在主线程执行，与菜单的重置、暂停、单步走同一条路径；回放期间忽略并记入日志。
- 相机画面在预览渲染后同时发给桥接。
- 不含 ROS2 的构建：菜单项禁用，悬停提示安装 ROS2 Humble 并以 `-DTERRA_BUILD_ROS2=ON` 重新构建。

**示例与 RViz：**
- `scripts/ros2/terra_generic_example.py`：只用 ROS2 自带的包。
  - 订阅 `/clock`（`use_sim_time`）、`/joint_states`、IMU 与雷达，每秒打印摘要；
  - `--wave <执行器>`：发正弦位置目标；
  - `--pause`、`--resume`、`--step N`、`--reset`：调用对应的服务。
- `assets/ros2/rviz/terra_stretch3.rviz`：Stretch 3 的 TF、底盘雷达、头部相机彩色与深度。使用时加 `use_sim_time:=true`。

## 2. 测试

**不含 ROS2 的构建（默认）：**

| 测试 | 内容 |
| --- | --- |
| `headless.ros2_unavailable`（新） | `--ros2` 提示安装并返回 3 |
| `headless.cli_parser`（扩充） | `--ros2` 隐含 `--realtime`；与 `--model`、`--dry-run`、`--rerun`、`--list-records` 互斥；重复与空值 |

测试总数：渲染关闭 83 → **84**；渲染开启 116 → **117**。

**含 ROS2 的构建（`-DTERRA_BUILD_ROS2=ON`）：**
- 去掉 `headless.ros2_unavailable`，新增 5 个测试，渲染开启时共 **121** 个。
- 端到端检查由 `tests/ros2/check_bridge.py` 完成（rclpy）。每个检查用单独的 `ROS_DOMAIN_ID`，且 `ROS_LOCALHOST_ONLY=1`。

| 测试 | 内容 |
| --- | --- |
| `ros2.profile` | 通用配置可读；严格校验 16 种错误；`topicName`；`mergeCommands` |
| `headless.ros2_bad_profile` | 频率为 0 的配置：报 `/tfHz` 并返回 2 |
| `ros2.bridge_headless` | 见下方说明 |
| `ros2.bridge_commands` | 见下方说明 |
| `graphics.ros2_bridge_viewer` | 见下方说明 |

`ros2.bridge_headless`（sensor_suite 场景）检查：
- 8 个话题的 QoS；话题与服务的类型；
- 后订阅也收到 `/tf_static`；父子坐标系正确，光学坐标系为绕 X 轴 180°；
- 3 秒窗口内的频率与采样步：
  - 各话题的时间戳都落在该频率的采样步上（与 `sensorSampleDue` 相同）；
  - `/joint_states` 与 `/tf` 不丢（≥ 95%）；
  - IMU 500 Hz 收到 ≥ 50%（best effort）；
  - 雷达 10 Hz；
- 墙钟节奏：仿真时间与墙钟之比在 0.7～1.3；
- 数值：水平 IMU 比力为 (0, 0, 9.81)；雷达 360 束、命中 275 束（与 M4-03b 相同），角度、量程与周期正确；
- 服务：
  - 暂停后 `/clock` 不动；
  - 单步 5 次正好前进 5 步；
  - 继续后恢复推进；
  - 重置后仿真时间回到起点；
- 标准错误中有统计信息。

`ros2.bridge_commands`（keyframe_arm 场景）检查：
- 一条消息里给三个执行器：位置执行器跟到 0.5（关键帧的目标是 0.8），速度执行器跟到 2.0，力矩执行器使滑块向 +x 运动；
- 再发 4 条错误命令：未知名字、越界、缺字段、NaN；
- 统计应为“接受 3 条、拒绝 4 条”，并给出最近一次拒绝的原因。

`graphics.ros2_bridge_viewer`（Xvfb 中的查看器，sensor_suite 场景）检查：
- 命令行开启桥接后直接运行；
- 相机的 QoS；
- 彩色为 rgb8 160 × 120；深度为 32FC1；内参 fx = fy、cx = 79.5、cy = 59.5；三者的坐标系都是 `cam/eye_optical`；
- 中心深度 2 m；
- 时间戳在 30 Hz 的采样步上，彩色、深度、内参来自同一步；
- IMU 与 Headless 相同；
- 服务与 Headless 相同。

**变异（10 个，在含 ROS2 的构建中）：**

| 变异 | 由哪个测试发现 |
| --- | --- |
| Headless 不合并桥接的命令 | `ros2.bridge_commands` |
| 速度执行器取错字段 | `ros2.bridge_commands` |
| 越界的命令不拒绝 | `ros2.bridge_commands` |
| `/tf_static` 不是 transient local | `ros2.bridge_headless` |
| Headless 暂停时单步无效 | `ros2.bridge_headless` |
| 雷达不按频率发布 | `ros2.bridge_headless` |
| 光学坐标系没有旋转 | `ros2.bridge_headless` |
| topicPrefix 不校验 | `ros2.profile` |
| 查看器不发布相机画面 | `graphics.ros2_bridge_viewer` |
| 查看器不执行服务请求 | `graphics.ros2_bridge_viewer` |

**展示：**
- 查看器打开 Stretch 3 并开启桥接；
- 示例程序让头部左右转动；
- RViz2 显示 TF、雷达与头部相机。

## 3. 已知的限制

- 相机图像只在查看器中发布。查看器每帧最多渲染每个相机最新的一份：仿真追赶时，中间的相机帧不发布。
- Headless 不渲染，没有相机话题（发布者存在，但不发消息）。
- 统计中的“订阅者数”只计 `/clock`、`/joint_states`、`/tf` 的订阅者。
- 查看器的“ROS2 桥接”菜单只用通用配置。其他配置用 `--ros2` 指定；配置的选择界面放在 RViz 面板（M4-08 之后）。
- 端到端测试依赖墙钟：机器很忙时，节奏与 best effort 话题的到达率可能波动。阈值已放宽（节奏 0.7～1.3，IMU ≥ 50%）。

## 4. 文件

- 新增：
  - `engine/ros2/{CMakeLists.txt,include/terra/ros2/bridge.hpp,src/bridge.cpp,src/profile.cpp}`、`cmake/TerraRos2.cmake`；
  - `assets/ros2/profiles/generic.terra-ros2.json`、`assets/ros2/rviz/terra_stretch3.rviz`、`assets/tests/ros2/bad_rate.terra-ros2.json`；
  - `tests/ros2/{test_ros2_profile.cpp,check_bridge.py}`；
  - `scripts/ros2/terra_generic_example.py`、`scripts/verify/m4_06a.sh`。
- 修改：
  - `CMakeLists.txt`、`cmake/TerraCheckRenderIsolation.cmake`；
  - `apps/headless/{CMakeLists.txt,cli.hpp,cli.cpp,main.cpp}`；
  - `apps/viewer/{CMakeLists.txt,main.cpp}`；
  - `engine/ui/{include/terra/ui/text.hpp,src/text.cpp}`；
  - `tests/CMakeLists.txt`、`tests/apps/test_headless_cli.cpp`。

## 5. 验证记录

### 5.1 开发机

未编译、未运行（按约定）。已做的静态检查：
- 变异锚点合计 244/244 唯一；
- 脚本通过 `bash -n`；
- Python 文件语法检查通过；RViz 配置是合法的 YAML。

### 5.2 基线机器

**第一次运行（2026-11-24）：除含 ROS2 的 ASan 外全部通过。**
- 不含 ROS2：84/84、117/117、117/117；ASan 93/93 + 24/24。
- 含 ROS2（Release -Werror）：121/121。变异 10/10。
- 端到端检查的结果：
  - 墙钟节奏 1.000；`/joint_states` 300/300、IMU 1500/1500、雷达命中 275 束；
  - 命令：肩关节到 0.50000、轮子角速度 2.0000、滑块力 0.5；接受 3 条、拒绝 4 条；
  - 查看器中心深度 2.0000 m。
- **未通过：含 ROS2 的 ASan 构建中，3 个端到端测试的被测程序都没有发出 `/clock`，并以退出码 1 结束**（`ros2.profile` 通过）。退出码 1 是 ASan / UBSan 报错时的默认值，需要看被测程序的标准错误才能定位。
  - 已修改：被测程序异常退出时，检查脚本打印它输出的末尾 25 行。
- 展示：查看器、RViz2 与示例程序都启动了；切换到 Go2W 场景后桥接自动重新开启。
  - 示例程序在结束时打印了 rclpy 的报错（“context is not valid”）：脚本用 SIGTERM 结束它，rclpy 先关闭了上下文，`spin` 随后报错，属于正常结束。
  - 已修改：示例把这种情况当作正常退出；脚本改用 SIGINT 结束它，并显示示例最后几行输出。

**ASan 失败的原因（用户贴出被测程序的标准错误）：**
- 三个程序都在创建 ROS2 节点时被 ASan 报 `new-delete-type-mismatch`：分配 72 字节，按 1 字节释放。
- 调用栈：`rclcpp::Node` 构造 → 参数事件发布者 → `rcl_node_resolve_name` → `rcutils_string_map_fini` → `rclcpp::allocator::retyped_deallocate<char>`。
- 这是 rclcpp Humble 头文件中的缺陷：`retyped_deallocate` 用 `std::allocator<char>::deallocate(p, 1)` 释放 rcutils 经 `retyped_allocate` 分配的任意大小的内存，sized delete 的大小不对。
  - 模板在本程序中实例化，所以算进了 Terra 的 ASan 构建；与 Terra 的代码无关；
  - glibc 的 `operator delete` 不使用这个大小，Release 下无害（121/121 的含 ROS2 构建运行正常）。
- 处理：含 ROS2 的 ASan 步骤加 `new_delete_type_mismatch=0`，只关这一项检查，其余 ASan 与 UBSan 检查照常。不含 ROS2 的 ASan 不受影响。

**重跑含 ROS2 的 ASan（2026-11-24）：通过。**用户在现有的 ASan 构建目录中以 `ASAN_OPTIONS=detect_leaks=0:new_delete_type_mismatch=0` 重跑 ros2 标签的测试：4/4 通过。
- `ros2.profile` 0.02 s；
- `ros2.bridge_headless` 16.7 s；
- `ros2.bridge_commands` 9.5 s；
- `graphics.ros2_bridge_viewer` 40.6 s。

ASan 下墙钟节奏与 best effort 话题的到达率也在阈值内。

