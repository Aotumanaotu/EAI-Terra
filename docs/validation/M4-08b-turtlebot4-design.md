# M4-08b TurtleBot 4：设计稿

> 状态：**评审通过**（2026-11-25，第 9 节 6 个问题全部按建议）。实现见 [M4-08b 报告](M4-08b-turtlebot4.md)。依据：
> - [M4-08 设计稿](M4-08-import-design.md)第 3 节（来源、传感器、通信配置），第 7 节问题 5、6 的决定：标准版、话题与真机一致；
> - [M4-08a](M4-08a-import.md)：导入流程已通过；
> - 用户在 M4-08a 验收时对文件浏览窗口提出两处修改，按用户要求放在本步（第 6 节）。

## 1. 目标与验收

| 目标 | 验收 |
| --- | --- |
| TurtleBot 4 标准版按固定提交锁定，用 M4-08a 的导入流程一步得到 MJCF 与场景 | 导入可重复（两次导入的结果逐字节相同），没有错误；落在地面上，推进 10 s 没有仿真错误 |
| 真机的主要传感器：RPLIDAR A1、OAK-D Pro（RGB 与深度）、Create 3 底盘 IMU | 传感器装在描述文件中对应的连杆上；方向与 RViz 中显示的坐标系一致 |
| 差速底盘 | 两轮速度驱动；直行、原地转向的运动与差速运动学一致（误差有阈值） |
| ROS2 通信配置：话题名、消息类型与 TurtleBot 4（Humble）一致 | 用标准 ROS2 工具（`teleop_twist_keyboard`、`ros2 topic pub /cmd_vel`），不改代码开动；`/odom`、`/scan`、`/imu`、相机、TF 正确 |

## 2. 来源与锁定

已核对，见 M4-08 设计稿第 3 节：
- `turtlebot/turtlebot4`：`humble` 分支 `a6ee13b6`，Apache-2.0；
- `iRobotEducation/create3_sim`：`humble` 分支 `7d7a4f69`，BSD-3-Clause。

**用户下载**（稀疏检出，放在同步目录 `ROBOTS/` 下）：

```
cd ~/EAI-Terra/ROBOTS
git clone --filter=blob:none --sparse https://github.com/turtlebot/turtlebot4.git
cd turtlebot4 && git sparse-checkout set turtlebot4_description && git checkout a6ee13b63cbd524500cc6a68cd20dbef69f326a9 && cd ..
git clone --filter=blob:none --sparse https://github.com/iRobotEducation/create3_sim.git
cd create3_sim && git sparse-checkout set irobot_create_common/irobot_create_description && git checkout 7d7a4f69a503d4eef22f13aeb8755c86955b8cc5 && cd ..
```

- **锁定：**两个描述包的 44 个文件逐个记入 `robots.lock`（sha256 与在 `ROBOTS/` 中的路径），由 `fetch_robots.sh` 复制并校验，与 M4-05 的 Go2W、AS2 相同。
- **导入：**新脚本 `scripts/assets/import_turtlebot4.sh` 用 `terra_import` 与导入配方（第 3 节）生成 `assets/robots/turtlebot4/`，与其他参考机器人一样不提交到仓库。导入结果的摘要（每个生成文件的 sha256）记入锁文件，用来核对“导入可重复”。

## 3. 导入配方（M4-08a 的扩充）

TurtleBot 4 有几处无法只靠自动提议处理，因此新增**导入配方**（`*.terra-import.json`），把导入时的选择写成文件，命令行与向导都可以读写：

| 字段 | TurtleBot 4 的值 | 说明 |
| --- | --- | --- |
| `input`、`name` | `turtlebot4_description/urdf/standard/turtlebot4.urdf.xacro`、`turtlebot4` | — |
| `xacroArgs` | `gazebo:=ignition`、`namespace:=` | Create 3 的 xacro 需要这两个参数；`gazebo` 只影响 Gazebo 插件段，对几何与关节没有影响 |
| `joints` | 两个轮跌落关节（`wheel_drop_left_joint`、`wheel_drop_right_joint`，prismatic，行程约 3.5 mm）→ `fixed` | 真机是弹簧压住的检测开关，不是可驱动的关节；按自动规则会被提议为位置执行器 |
| `actuators` | 只给 `wheel_left_joint`、`wheel_right_joint` 速度执行器（kv、力限按 Create 3 的电机参数），其余不加 | 保险杠等不加执行器 |
| `sensors` | 见第 4 节；参数按真机，不用自动提议的默认值 | — |
| `meshScale`、`fixedBase` | 1、浮动 | — |

**大网格：**
- OAK-D Pro 的 Collada 有 12 MB，外壳有 7 MB。转换只在导入时做一次，之后加载转换好的 OBJ。
- 碰撞几何体：外壳有专门的碰撞网格（`shell_collision.dae`）；其余用视觉网格。MuJoCo 会对碰撞网格取凸包。

**其他需要的导入改进：**
- 导入结果逐字节可重复：浮点数的写法固定，文件顺序固定；
- 进度提示：转换大网格时，向导显示正在转换的文件。

## 4. 传感器

| 传感器 | 安装连杆 | 参数 | 依据 |
| --- | --- | --- | --- |
| 2D 激光雷达（RPLIDAR A1M8） | `rplidar_link` | 360°、0.164～12 m；暂定 1080 束、8 Hz，待真机核对 | 描述文件的 Gazebo 参数为 0.164～12 m、640 束；A1M8 的标称为 0.15～12 m、5.5～10 Hz |
| RGB 与深度相机（OAK-D Pro） | `oakd_rgb_camera_optical_frame` | 预览分辨率 250 × 250（与 TurtleBot 4 默认的 `/oakd/rgb/preview` 相同）、15 Hz；深度另行配置 640 × 400 | TurtleBot 4 的 OAK-D 驱动默认参数；待真机核对 |
| IMU | Create 3 的 `imu_link` | 100 Hz | Create 3 的 `/imu` 为 100 Hz |
| 悬崖、红外障碍、碰撞、轮跌落、光学地面跟踪 | — | 本步不做 | 需要新的传感器类型（射线式的距离传感器与接触开关），列入 M4-09 的候选 |

## 5. ROS2 通信配置

不另起一个“TurtleBot 4 专用”的配置类型，而是给通用配置加两项**可复用**的能力，TurtleBot 3 与其他差速机器人也能用：

1. **话题名映射**（`topics`）：传感器与通用话题改为真机的名字。例如：
   - `rplidar` → `/scan`；
   - OAK-D 的 RGB → `/oakd/rgb/preview/image_raw` 与 `camera_info`；
   - 深度 → `/oakd/stereo/image_raw`；
   - IMU → `/imu`。
2. **差速底盘**（`diffDrive`）：
   - 订阅 `/cmd_vel`（`geometry_msgs/Twist`，Humble 版 Create 3 的类型；Jazzy 改为 `TwistStamped`，以后按版本切换）；
   - 按轮半径与轮距换算为两轮角速度，0.5 s 没有新命令时停车（与 Create 3 相同）；
   - 发布 `/odom`（`nav_msgs/Odometry`，由两轮转角积分，与真机的轮式里程计相同，打滑时与真实位姿不同）与 TF `odom → base_link`。

| 话题 | 方向 | 消息 | 说明 |
| --- | --- | --- | --- |
| `/scan` | 发布 | `sensor_msgs/LaserScan` | 坐标系 `rplidar_link` |
| `/oakd/rgb/preview/image_raw`、`/oakd/rgb/preview/camera_info` | 发布 | `Image`、`CameraInfo` | 查看器中发布 |
| `/oakd/stereo/image_raw` | 发布 | `Image`（深度，32FC1；真机为 16UC1 毫米，见问题 4） | 查看器中发布 |
| `/imu` | 发布 | `sensor_msgs/Imu` | — |
| `/odom` | 发布 | `nav_msgs/Odometry` | 轮式里程计 |
| `/tf`、`/tf_static` | 发布 | `odom → base_link → …` | 与真机的 TF 树相同（真机没有 world） |
| `/joint_states` | 发布 | 轮子关节 | — |
| `/cmd_vel` | 订阅 | `geometry_msgs/Twist` | 0.5 s 超时停车 |
| `/battery_state`、`/dock_status`、`/hazard_detection`、`/ir_intensity`、`/cliff_intensity`、`/wheel_status` | — | — | 不提供，清单中列出 |

**真机话题清单：**`assets/ros2/profiles/turtlebot4.terra-ros2.json` 附带清单，标出状态（已提供 / 仿真值 / 不提供 / 待真机核对），与 M4-05 的宇树清单相同。

## 6. 文件浏览窗口的修改（用户 2026-11-25 在 M4-08a 验收时提出）

1. **列表视图：**图标与文件名之间留出间隔（现在贴得太近）。
2. **路径栏改为 Windows 文件管理器的方式：**
   - 点击路径栏中按钮以外的空白处，路径栏直接变为可编辑的输入框（选中全部文字）；
   - 回车跳转，Esc 或点到别处恢复为逐级按钮；
   - 去掉单独的“编辑路径”按钮；
   - 点击某一级目录跳转的功能保留。

## 7. 测试（实现时细化）

| 测试 | 内容 |
| --- | --- |
| `robots.turtlebot4_import` | 见下方说明 |
| `robots.turtlebot4_drive` | 两轮速度：直行 1 m/s 的位移、原地转向的角速度与差速运动学一致（阈值按打滑估计）；推进 10 s 状态有限 |
| `import.recipe` | 配方的读取与严格校验；配方中的关节改为固定、只加列出的执行器、按配方的传感器参数 |
| `ros2.turtlebot4`（ROS2 构建） | 见下方说明 |
| `ui.file_browser`（扩充） | 路径栏的点击与编辑状态（逻辑部分） |

`robots.turtlebot4_import` 检查：
- 导入没有错误，两次导入逐字节相同；
- 连杆与关节数与描述文件一致；
- 两个轮子是速度执行器，轮跌落关节为固定；
- 雷达、相机、IMU 装在对应的连杆上。

`ros2.turtlebot4`（ROS2 构建）检查：
- 话题名、类型与 QoS 与清单一致；
- 发 `/cmd_vel` 后 `/odom` 的速度与命令一致；
- 超时停车；
- `odom → base_link` 的 TF。

**展示：**查看器打开 TurtleBot 4，RViz2 显示 `/scan` 与相机，另一个终端运行 `teleop_twist_keyboard` 开动机器人。

## 8. 需要用户准备

1. 按第 2 节下载 TurtleBot 4 与 Create 3 的描述文件。
2. 安装键盘遥控：`sudo apt install ros-humble-teleop-twist-keyboard`（用于展示，不是测试的必需项）。

## 9. 需要评审的问题

| # | 问题 | 建议 | 其他选项 |
| --- | --- | --- | --- |
| 1 | 导入时的选择怎么保存 | 新增导入配方 `*.terra-import.json`，命令行与向导都能读写，TurtleBot 4 用仓库中的配方，导入可重复 | 把 TurtleBot 4 的特殊处理写死在导入脚本里 |
| 2 | 轮跌落关节 | 导入时改为固定（真机是弹簧加检测开关，不是驱动关节） | 做成弹簧（MuJoCo 关节刚度），保留几毫米的悬挂行程 |
| 3 | 通信配置的组织 | 给通用配置加“话题名映射”和“差速底盘”两项可复用的能力，TurtleBot 4 只是一份配置文件 | 新增 TurtleBot 4 专用的配置类型 |
| 4 | 深度图的格式 | 与真机一致：`/oakd/stereo/image_raw` 用 16UC1（毫米），通用配置中的 32FC1（米）保持不变 | 都用 32FC1 |
| 5 | 雷达参数 | 暂定 1080 束、8 Hz、0.164～12 m，有真机数据后修改 | 用描述文件中的 Gazebo 参数（640 束、62 Hz，与真机差别大） |
| 6 | 生成的 TurtleBot 4 模型放在哪里 | 与其他参考机器人一样，脚本生成到 `assets/robots/turtlebot4/`，不提交；锁文件记录源文件与导入结果的 sha256 | 把生成的 MJCF 与网格提交到仓库 |
