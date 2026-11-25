# M4-08 模型快速导入、资源库与 RViz 面板：设计稿

> 状态：**评审通过**（2026-11-25：8 个问题已答复，第 8 节的修订经用户确认）。依据：
> - 开发计划 11.2 节 M4-08（F-M4-04、D-112）；
> - 11.6 节：用户选定 TurtleBot 4；
> - 11.7 节：RViz 功能 A 层、显示配置以场景为核心、`.terviz` 与 `.rviz` 导出、没有 ROS2 时提醒。
> 本稿只定范围、流程、文件格式与验证方法；评审通过后分步实现。

## 1. 范围与拆分

M4-08 有三块相对独立的内容，按依赖顺序分为四步，每步单独验证：

| 步骤 | 内容 | 依赖 |
| --- | --- | --- |
| **M4-08a 导入** | URDF / MJCF 导入流程（命令行与查看器的导入向导）：展开 xacro、解析 `package://`、转换网格、检查单位与轴向、补默认执行器、生成场景并打开 | — |
| **M4-08b TurtleBot 4** | 按固定提交锁定 TurtleBot 4 的描述文件，用 M4-08a 导入，加上真机的传感器（激光雷达、OAK-D 相机、IMU）与差速底盘；ROS2 通信配置（`/cmd_vel`、`/odom`、`/scan` 等，与真机一致） | M4-08a |
| **M4-08c 资源库面板** | 资源库（SQLite）扩展到机器人、场景、动作、环境贴图与导入的资产；查看器中的“资源库”面板 | M4-08a |
| **M4-08d ROS2 面板（RViz A 层）** | 话题列表、任意消息查看、数值曲线、图像查看、TF 树；显示配置以场景为核心存入资源库，导出 `.terviz` 与 `.rviz` | M4-08c |

## 2. M4-08a 导入流程

**入口：**
- 命令行：`terra_import <URDF / xacro / MJCF / 资产包目录> [--name <名字>] [--output <目录>] [--package <包名>=<目录> ...]`。只做导入与检查，输出报告（JSON）。
- 查看器：菜单“文件 → 导入模型…”，或把文件拖进窗口。打开导入向导，步骤同下。每一项问题都显示原因与可操作的处理办法；完成后打开生成的场景。

**流程**（每步都记入导入报告 `import-report.json`）：

| 步骤 | 做法 | 发现问题时的提示 |
| --- | --- | --- |
| 1. 识别 | 按扩展名与根元素：`<robot>` 为 URDF，`<mujoco>` 为 MJCF，`.xacro` 先展开 | 不认识的文件：说明支持的格式 |
| 2. 展开 xacro | 调用 ROS 的 `xacro` 程序（`ros-humble-xacro`，Humble desktop 自带） | 找不到 `xacro`：提示安装命令；也可以先在别处展开后导入 `.urdf` |
| 3. 解析路径 | `package://<包>/…` 与 `$(find <包>)`：依次在 `--package` 给出的目录、`ROS_PACKAGE_PATH` / `AMENT_PREFIX_PATH`、模型所在目录的上层中查找 | 找不到的包与文件逐个列出，提示用 `--package 包名=目录` 指定 |
| 4. 转换网格 | MuJoCo 只读 STL、OBJ、MSH，不能读 Collada（`.dae`，TurtleBot 4 与 Create 3 都用它）。用 assimp 转换：碰撞与物理用 OBJ；视觉用 glTF（`.glb`，保留颜色与材质，Terra 的渲染器直接使用）。转换结果放进导入目录 | 没有 assimp：提示安装命令 `sudo apt install assimp-utils`（BSD-3-Clause）。网格过大（如 OAK-D 的 12 MB）时提示，并可以选择简化 |
| 5. 单位与轴向 | Collada 的 `<unit>` 与 `<up_axis>` 由 assimp 换算为米、Z 向上；检查包围盒是否在合理范围（例如整机超过 10 m 或小于 1 cm 时提示可能是毫米或英寸） | 给出估计的比例与“按 0.001 缩放”的选项 |
| 6. 转为 MJCF | MuJoCo 直接编译 URDF（在 URDF 中加 `<mujoco><compiler …/></mujoco>` 扩展：保留视觉几何、不合并固定连杆、网格目录），再保存为 MJCF。之后按 Terra 的约定调整：可见几何体放在 group 2、碰撞几何体放在 group 3（与 Menagerie 相同） | 没有惯量或质量为 0 的连杆：说明 MuJoCo 的处理方式（固定连杆并入父连杆；活动连杆给最小惯量），列出受影响的连杆 |
| 7. 执行器 | URDF 没有执行器。按关节类型提议默认执行器：<br>- `continuous`（轮子）：速度执行器；<br>- `revolute`、`prismatic`：位置执行器，范围取关节限位；<br>- 增益按连杆质量估计。<br>用户可以在向导中逐个修改或不加 | 有 `<transmission>` 时按其中的关节与接口类型加执行器 |
| 8. 传感器 | 识别常见的传感器连杆名（`*lidar*`、`*laser*`、`*camera*`、`*imu*`），提议加入场景文件的 `sensors`；参数可修改 | 只提议，不自动加 |
| 9. 生成 | 导入目录：`<名字>/model.xml`（MJCF）、`meshes/`（OBJ）、`visual/`（glb）、`<名字>.terra-scene.json`（加地面；视觉绑定到 glb）、`import-report.json`；登记到资源库（M4-08c） | — |
| 10. 检查 | 加载生成的场景并推进 1 秒：没有仿真错误、状态有限、机器人没有穿过地面。结果写入报告 | 失败时指出最可能的原因（如碰撞几何体相互重叠） |

**导入目录的位置：**
- 用户导入的模型放在资源库目录（`$TERRA_LIBRARY` 或 `~/.local/share/eai-terra/` 下的 `assets/`），不改动仓库；
- 仓库自带的参考模型（TurtleBot 4）由脚本导入到 `assets/robots/`，与其他参考机器人相同。

**测试用的 URDF 夹具**（放在 `assets/tests/import/`，自己写的小模型）：
- 带 `package://` 路径、STL 网格、一个 `continuous` 轮子、一个 `revolute` 关节、一个没有惯量的固定连杆；
- 一份故意出错的 URDF：缺网格、包名错误、毫米单位。

验收标准（计划 11.2 节）：
- 三台参考机器人（MJCF）与一个 URDF 示例都能一步导入并运行；
- 每类问题都有可操作的提示。

## 3. M4-08b TurtleBot 4

**来源**（已用 GitHub API 核对，2026-11-25）：

| 仓库 | 分支 / 提交 | 许可 | 用到的部分 |
| --- | --- | --- | --- |
| `turtlebot/turtlebot4` | `humble`，`a6ee13b63cbd524500cc6a68cd20dbef69f326a9`（2025-06-23） | Apache-2.0 | `turtlebot4_description`：URDF（xacro）与网格（Collada），共 22 个文件、约 30 MB |
| `iRobotEducation/create3_sim` | `humble`，`7d7a4f69a503d4eef22f13aeb8755c86955b8cc5`（2026-09-25） | BSD-3-Clause | `irobot_create_common/irobot_create_description`：Create 3 底盘的 URDF 与网格，共 22 个文件、约 8 MB |

**需要用户下载**（与 M3-01 相同，放在同步目录 `ROBOTS/` 下；只用稀疏检出，避免下载整个仓库）：

```
cd ~/EAI-Terra/ROBOTS
git clone --filter=blob:none --sparse https://github.com/turtlebot/turtlebot4.git
cd turtlebot4 && git sparse-checkout set turtlebot4_description && git checkout a6ee13b63cbd524500cc6a68cd20dbef69f326a9 && cd ..
git clone --filter=blob:none --sparse https://github.com/iRobotEducation/create3_sim.git
cd create3_sim && git sparse-checkout set irobot_create_common/irobot_create_description && git checkout 7d7a4f69a503d4eef22f13aeb8755c86955b8cc5 && cd ..
```

两份源文件逐个记入 `robots.lock`（sha256），由 `fetch_robots.sh` 校验。

导入时选**标准版**（`urdf/standard/turtlebot4.urdf.xacro`，带 OAK-D Pro；另有 lite 版）。

**传感器**（与真机对应；`待核对` 的参数在有真机时修改）：

| 传感器 | 真机 | 仿真 |
| --- | --- | --- |
| 2D 激光雷达 | RPLIDAR A1M8：360°，0.15～12 m，约 5.5～10 Hz | `lidar2d` 装在 `rplidar_link`；360°、0.164～12 m（取自描述文件的 Gazebo 参数）；束数与频率待核对，暂定 640 束、10 Hz |
| 深度相机 | OAK-D Pro：RGB 与双目深度 | `camera` 装在 OAK-D 的光学坐标系；RGB 与深度，分辨率暂定 640 × 480、15 Hz |
| IMU | Create 3 底盘 IMU | `imu` 装在 `imu_link` |
| 悬崖、红外障碍、碰撞、跌落、光学地面跟踪 | 有 | 本步不做（需要新的传感器类型，列入 M4-09 的候选） |

**底盘：**两个驱动轮用速度执行器，万向轮被动。

**ROS2 通信配置 `turtlebot4.terra-ros2.json`**（Sim-to-Real，与真机一致的部分）：

| 话题 | 方向 | 消息 | 说明 |
| --- | --- | --- | --- |
| `/scan` | 发布 | `sensor_msgs/LaserScan` | 坐标系 `rplidar_link` |
| `/oakd/rgb/preview/image_raw`、`/oakd/rgb/preview/camera_info` | 发布 | `Image`、`CameraInfo` | 查看器中发布 |
| `/oakd/stereo/image_raw` | 发布 | `Image`（深度） | 查看器中发布 |
| `/imu` | 发布 | `sensor_msgs/Imu` | — |
| `/odom` | 发布 | `nav_msgs/Odometry` | 由轮子转角积分（与真机相同的轮式里程计，有打滑时与真实位姿不同） |
| `/tf` | 发布 | `odom → base_link`，以及各连杆 | — |
| `/joint_states` | 发布 | 轮子关节 | — |
| `/cmd_vel` | 订阅 | `geometry_msgs/Twist` | 差速运动学换算为两轮角速度；超时（0.5 s 没有新命令）停车，与 Create 3 相同 |

另有：
- 不提供：`/battery_state`、`/dock_status`、`/hazard_detection`、`/ir_intensity` 等（需要对应的传感器或仿真值），清单中列出；
- 话题名以 TurtleBot 4 Humble 版的默认值为准，待真机核对（真机可能带命名空间）。

验收：用 `ros2 topic pub /cmd_vel` 或 `teleop_twist_keyboard`（标准 ROS2 工具，不改代码）开动仿真中的 TurtleBot 4，`/odom` 与 `/scan` 正确。

## 4. M4-08c 资源库面板

资源库（SQLite，M3-07b 已有记录表）新增资产表，登记：
- 机器人（导入目录或仓库中的参考机器人）；
- 场景、动作、环境贴图（HDRI）；
- 导入的资产包、显示配置（M4-08d）。

每条包含：类型、名字、路径、来源（仓库 / 导入 / 用户）、sha256、许可、创建时间、缩略图（可选）。

**查看器“资源库”面板：**
- 按类型分组，可搜索；
- 双击打开场景、播放动作、切换环境贴图；
- 右键可以“在文件夹中显示”“重新导入”“删除登记”（不删文件）；
- 首次打开时扫描仓库的 `assets/` 并登记；之后“重新扫描”更新。

## 5. M4-08d ROS2 面板（RViz A 层）

只有构建时打开 ROS2 才有；没有时菜单中显示“未检测到 ROS2 Humble”与安装说明（11.7 节的决定）。

| 功能 | 做法 |
| --- | --- |
| 节点与话题 | 列出节点、话题（类型、发布者 / 订阅者数、QoS），实测频率与带宽（订阅后统计） |
| 消息查看 | 任意类型：用 rclcpp 的通用订阅收到序列化消息，按 `rosidl_typesupport_introspection_cpp` 在运行时展开字段，不需要事先编译该类型的代码；数组只显示前若干项 |
| 曲线 | 选中消息中的数值字段（如 `/imu` 的 `angular_velocity.z`）画随时间的曲线，可多条 |
| 图像 | `Image`（rgb8、bgr8、mono8、16UC1、32FC1；深度用 M4-04 的伪彩色）与 `CompressedImage`（JPEG、PNG） |
| TF 树 | 由 `/tf`、`/tf_static` 建树，显示父子关系与最新的变换 |

**与仿真自己的传感器统一**（11.7 节问题 2）：传感器面板读仿真数据，ROS2 面板读网络数据，曲线与图像用同一套控件。

**显示配置**（11.7 节问题 3）：
- 以场景为核心，存入资源库：每个场景可以有多份，记录打开了哪些面板、话题、字段、曲线与图像的布局。
- `.terviz`：JSON，`format` 为 `"terra.viz"`、`version` 为 1，可以导出、再导入。
- `.rviz`：只导出 RViz 有对应显示类型的部分（Image、LaserScan、TF、Imu 等）；曲线等 RViz 没有的部分在导出报告中列出。视口叠加（B 层）以后做，届时再补导出项。

## 6. 需要用户准备

1. 安装 assimp 命令行工具：`sudo apt install assimp-utils`（转换 Collada 网格）。
2. 确认 `xacro` 可用：`source /opt/ros/humble/setup.bash && ros2 run xacro xacro --version`。没有时：`sudo apt install ros-humble-xacro`。
3. 按第 3 节下载 TurtleBot 4 与 Create 3 的描述文件（评审通过后、M4-08b 之前）。

## 7. 需要评审的问题

| # | 问题 | 建议 | 其他选项 |
| --- | --- | --- | --- |
| 1 | 拆分与顺序 | 按第 1 节分为 a～d 四步，依次实现、各自验证 | 先做 TurtleBot 4（只写它专用的导入脚本），通用导入向导以后再做 |
| 2 | Collada 网格的转换 | 导入时调用 assimp 命令行工具（外部程序，用户安装；找不到时提示），转换结果随导入目录保存，之后不再需要 assimp | 把 assimp 作为第三方库编进 Terra（体积大、构建时间长）；或自己写只支持三角形的 Collada 读取（容易漏掉单位、层级变换等情况） |
| 3 | xacro 的展开 | 调用 ROS 的 `xacro` 程序；没有 ROS 的机器上只能导入已展开的 `.urdf` | 自己实现 xacro（宏、属性、表达式，工作量大且容易与官方行为不一致） |
| 4 | 用户导入的模型放在哪里 | 资源库目录（默认 `~/.local/share/eai-terra/assets/`），不改动仓库；仓库自带的参考模型由脚本导入到 `assets/robots/` | 都放进仓库的 `assets/` |
| 5 | TurtleBot 4 的版本 | 标准版（带 OAK-D Pro 与上层平台），Humble 分支的固定提交 | lite 版（OAK-D Lite，没有上层平台） |
| 6 | TurtleBot 4 的通信配置 | 本步做 `/cmd_vel`、`/odom`、`/scan`、`/imu`、相机与 TF，与真机话题名一致；悬崖、红外、碰撞等传感器以后做 | 只用 M4-06 的通用配置（话题名与真机不同，不能直接对接 TurtleBot 4 的 ROS2 程序） |
| 7 | 动态消息解析（ROS2 面板） | 运行时按 introspection 类型信息展开任意消息，宇树等第三方消息也能看 | 只支持编译时已知的常用类型（实现简单，但看不了第三方消息） |
| 8 | `.terviz` 与 `.rviz` | 按第 5 节；`.rviz` 只导出对应的显示类型并列出没有导出的部分 | 不导出 `.rviz` |

## 8. 评审答复与修订（2026-11-25）

| # | 用户答复 | 处理 |
| --- | --- | --- |
| 1 | 可以分步做；需要说明 M4-08a 的手动测试方法与 demo | 见 8.1 |
| 2 | 可以 | 按建议：导入时调用 assimp 命令行工具 |
| 3 | 评估集成难度；担心导入因此不能与 ROS2 解耦 | 修订为使用独立的 xacro，不需要 ROS2，见 8.2，待确认 |
| 4 | 担心放在用户目录对产品化不友好 | 说明与修订见 8.3，待确认 |
| 5 | 标准版 | 采用 |
| 6 | 这一步就与真机保持一致 | 采用（第 3 节的通信配置在 M4-08b 实现） |
| 7 | 可以；默认界面很多内容展不开，需要自适应，并略微扩大默认界面 | 见 8.4，待确认 |
| 8 | 由用户选择：导出时弹窗选择导出哪种类型 | 导出对话框中选择 `.terviz`、`.rviz` 或两者；`.rviz` 附带“未导出的部分”清单 |

### 8.1 M4-08a 的手动测试与 demo

用三个公开的示例，各覆盖一类问题。都按固定提交下载到同步目录 `ROBOTS/import_demos/`（只稀疏检出需要的目录）：

| 示例 | 仓库与提交 | 许可 | 覆盖的问题 |
| --- | --- | --- | --- |
| urdf_tutorial（R2D2） | `ros/urdf_tutorial`，`ros2` 分支 `050f1e47` | BSD-3-Clause | `07-physics.urdf`：`package://` 路径、Collada 网格带贴图（夹爪）、没有执行器；`08-macroed.urdf.xacro`：xacro 宏 |
| TurtleBot 3 Burger | `ROBOTIS-GIT/turtlebot3`，`humble` 分支 `90a68bd2` | Apache-2.0 | `turtlebot3_burger.urdf` 内含 xacro 参数、STL 网格以毫米为单位（scale 0.001）、两个 `continuous` 轮子（差速）、激光雷达连杆 |
| Menagerie 中的一台 MJCF | 已有（如 Go2） | BSD-3-Clause | MJCF 一步导入 |

**手动测试的步骤**（验证脚本最后打开查看器时按提示做）：
1. 菜单“文件 → 导入模型…”，选 `07-physics.urdf`。预期：
   - 向导依次列出“找到包 urdf_tutorial”“转换 2 个 Collada 网格”“没有执行器，提议 N 个”；
   - 完成后打开场景，R2D2 站在地面上，夹爪有贴图。
2. 导入 `turtlebot3_burger.urdf`。预期：
   - 提示“文件中有 xacro，已展开”“网格按 0.001 缩放”“两个轮子提议速度执行器”“发现激光雷达连杆 base_scan，提议加 2D 激光雷达”；
   - 打开后在执行器面板转动两个轮子，小车前进或原地转向。
3. 导入一份故意出错的 URDF（仓库自带的夹具）。预期每个问题都有原因与处理办法，例如：
   - 缺网格时列出文件名；
   - 包名错时提示用 `--package` 指定；
   - 毫米单位时提示缩放。
4. 把文件直接拖进窗口，效果与菜单相同。

自动化测试用仓库自带的小夹具（不依赖下载）。上面三个示例已下载时也会自动导入并检查，没有下载时跳过。

### 8.2 xacro：不依赖 ROS2 的方案（修订问题 3）

**评估：**
- xacro 官方实现是纯 Python 包，已发布在 PyPI（`xacro` 2.1.1，BSD-3-Clause），只依赖 PyYAML，本身不需要 ROS。
- 只有 `$(find 包名)` 依赖 ROS 的包索引（ament）。

**方案：**
- 按固定版本获取 xacro 源码包（与其他第三方依赖相同：脚本下载、sha256 校验），放在 `third_party/xacro/`；
- Terra 通过一个小的包装脚本调用它：`$(find 包名)` 改为查导入时的包目录表（与 `package://` 相同的查找规则），不经过 ROS；
- 运行只需要 `python3` 与 `python3-yaml`（Ubuntu 默认都有）。

**结果：**导入功能与 ROS2 完全解耦。
- 纯 URDF 与 MJCF 不需要 Python；
- xacro 需要 Python，但不需要 ROS；
- 与官方行为一致（用的就是官方代码）。

**集成成本：**下载脚本、包装脚本（约 50 行）与测试；不需要自己实现 xacro。

### 8.3 用户导入的模型放在哪里（说明问题 4）

放在用户数据目录，正是为了产品化：
- 安装后的程序目录通常只读（系统安装、AppImage、容器），不能也不应写入；
- 用户的数据要与程序分开，升级或重装程序时不丢失。Linux 的惯例是 XDG 用户数据目录（`~/.local/share/<应用>/`），Windows 对应 `%APPDATA%`。

修订，使它对产品更友好：
- **资源库位置可设置：**偏好设置中新增“资源库位置”（默认仍为用户数据目录；也可以放到项目目录或共享盘，团队共用）；环境变量 `TERRA_LIBRARY` 继续有效。
- **资产包的导出与导入：**导入的机器人可以打包成一个文件（模型、网格、场景、导入报告与许可信息）发给别人；别人导入资产包，不需要重新转换。
- **仓库自带的内容**（参考机器人、场景、动作、环境贴图）作为只读的内置资源，与用户资源在资源库面板中分开显示。

### 8.4 界面自适应与默认大小（问题 7）

建议：

**默认窗口：**首次启动时取显示器可用区域的 85%（现在固定为 1600 × 900），居中。已有设置文件的用户保持自己的大小。

**默认停靠比例：**
- 右侧面板 24% → 28%，底部 24% → 26%；
- 大纲保持 17%。

**ROS2 面板与消息查看：**
- 话题列表用可拖动列宽、可排序的表格；
- 消息字段用树形，按需展开（只在展开时解析子字段）；
- 长数组分页显示（每页 100 项，可翻页）；
- 消息查看、曲线、图像都可以拖出成独立的浮动窗口，或停靠到任意位置；
- 每个面板的大小与位置随显示配置保存。

**字号：**现在按显示器 DPI 自动缩放。偏好设置中已有界面缩放，这次不改默认字号。

默认窗口与停靠比例的修改在 M4-08a 中先做，你可以在第一步的展示中先看效果，再决定是否调整。

