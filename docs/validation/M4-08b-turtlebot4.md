# M4-08b TurtleBot 4 标准版

> 状态：**代码已写，待基线验证**（开发机不编译）。依据：[M4-08b 设计稿](M4-08b-turtlebot4-design.md)（2026-11-25 评审通过，6 个问题全部按建议）。

## 1. 交付

**锁定：**
- `robots.lock` 追加 46 个文件：
  - `turtlebot4_description` 22 个；
  - `irobot_create_description` 22 个；
  - 两个仓库根目录的 LICENSE。
- 原件在 `ROBOTS/turtlebot4/` 与 `ROBOTS/create3_sim/`（用户已下载）。44 个文件逐个与固定提交的 git blob 哈希比对，全部一致。
- LICENSE 不在稀疏检出中，由 `fetch_robots.sh` 从固定提交下载。参考机器人文件合计 245 → 291 个。

**导入配方**（`*.terra-import.json`，format `terra.import-recipe`、version 1，严格校验）：
- 字段：`input`（相对配方文件）、`name`、`xacroArgs`、`packages`、`placeholderPackages`、`meshScale`、`fixedBase`、`joints`、`actuators`、`sensors`、`extraModels`、`view`。
  - `joints`：关节名 → `"fixed"`；
  - `actuators`：给出时替换提议；
  - `sensors`：给出时替换提议，其余字段为场景文件中该类传感器的参数；
  - `extraModels`：如房间。
- 命令行 `terra_import --recipe <配方>`；导入向导可以直接选配方文件；每次导入都把实际使用的选择写入导入目录的 `import.terra-import.json`。
- 导入结果可重复：同一配方导入到并列的两个目录，全部文件逐字节相同（测试检查）。
  - 设计稿第 2 节原计划把导入结果的 sha256 记入锁文件，改为由测试保证：导入结果随 assimp 版本可能不同，记进锁文件反而会在别的机器上误报。

**TurtleBot 4 的配方**（`assets/robots/turtlebot4.terra-import.json`）：

| 项目 | 内容 |
| --- | --- |
| xacro | 标准版 `turtlebot4.urdf.xacro`，参数 `gazebo:=ignition`、`namespace:=` |
| 占位包 | `irobot_create_control`：只在 Create 3 的 Gazebo 插件参数中出现，不读取其中的文件 |
| 关节 | 两个轮跌落关节（prismatic，行程 3 cm，`effort` 为 0）改为固定 |
| 执行器 | 只有两个轮子：速度执行器，kv 1.0、力限 1.5 N·m |
| 传感器 | 见下表 |
| 其他模型 | `assets/scenes/common/room.xml`：6 × 6 m 的房间与障碍物（Terra 编写），让雷达与相机看得到东西 |

| 传感器 | 安装 | 参数 |
| --- | --- | --- |
| 2D 激光雷达 | `rplidar_link` | 1080 束、0.164～12 m、8 Hz（待真机核对） |
| RGB 相机 | OAK-D 的 `oakd_rgb_camera_optical_frame` | 250 × 250、55°、15 Hz（与 `/oakd/rgb/preview` 相同；视场角待核对） |
| 深度相机 | 同上 | 640 × 400、15 Hz |
| IMU | `imu_link` | 100 Hz |

导入脚本 `scripts/assets/import_turtlebot4.sh` 生成 `assets/robots/turtlebot4/`（不提交），场景为其中的 `turtlebot4.terra-scene.json`。

**ROS2 通信配置的两项可复用能力**（通用配置，TurtleBot 3 等差速机器人也能用）：
- **话题名映射** `topics`：默认话题名 → 真机的话题名。映射中的每一项都必须是本场景的默认话题，拼错时开启桥接报错并列出可映射的话题。
- **坐标系** `frames`：
  - `stripPrefix` 去掉坐标系名与关节名的实例前缀（真机的坐标系为 `base_link` 等）；
  - `sensorsUseLinkFrames` 使安装位姿与连杆重合的传感器直接用连杆坐标系（`/scan` 的 frame 为 `rplidar_link`，与真机相同），不另发静态 TF。
- **深度格式** `depthEncoding`：`16UC1`（毫米，没有命中为 0，与 OAK-D 相同）或默认的 `32FC1`（米）。
- **差速底盘** `diffDrive`：
  - 订阅 `/cmd_vel`（`geometry_msgs/Twist`），换算为两轮角速度（超过最大轮速时按比例缩小两轮）；墙钟 0.5 s 没有新命令时停车；
  - 由两轮转角积分出轮式里程计，发布 `/odom`（20 Hz）；
  - 重置后从 0 开始；
  - 有差速底盘时 TF 树与真机相同：`odom → base_link`，连杆之间为相对位姿，没有 `world`。

`assets/ros2/profiles/turtlebot4.terra-ros2.json` 的真机话题清单：

| 话题 | 状态 |
| --- | --- |
| `/scan`、`/imu`、`/odom`、`/cmd_vel`、`/joint_states`、`/tf`、`/tf_static` | 已提供 |
| `/oakd/rgb/preview/image_raw`、`/oakd/rgb/preview/camera_info`、`/oakd/stereo/image_raw`、`/oakd/stereo/camera_info` | 已提供（查看器中发布） |
| `/clock` | 仿真附加（真机没有） |
| `/battery_state`、`/dock_status`、`/hazard_detection`、`/ir_intensity`、`/cliff_intensity`、`/wheel_status` | 不提供 |
| 雷达参数、相机视场角、话题频率、是否带命名空间 | 待真机核对 |

**文件浏览窗口**（用户在 M4-08a 验收时提出）：
- 列表视图中图标与文件名之间留出间隔（行高按图标）；
- 地址栏改为 Windows 文件管理器的方式：
  - 有底框，逐级按钮之后的空白处点击即进入编辑（选中全部路径，鼠标为文字光标）；
  - 回车跳转，Esc 或点到别处恢复为逐级按钮；地址栏中的 Esc 不再关闭整个窗口；
  - 去掉“编辑路径”按钮；点击某一级目录跳转保留。

**RViz2：**`assets/ros2/rviz/terra_turtlebot4.rviz`：固定坐标系 odom；TF、`/scan`、`/odom`、OAK-D 的 RGB 与深度。

## 2. 测试

| 测试 | 内容 |
| --- | --- |
| `import.analyze`（扩充） | 见下方说明 |
| `import.execute`（扩充） | 按配方导入两次逐字节相同；导入目录中写下配方；场景按配方：arm_joint 固定、两个执行器、雷达 180 束 5 Hz、房间 |
| `robots.turtlebot4_import`（新） | 见下方说明 |
| `ros2.profile`（扩充） | turtlebot4 配置的各项；新字段的 6 种错误 |
| `ros2.turtlebot4`（新，ROS2 构建） | 见下方说明 |

`import.analyze` 新增配方的检查：
- 读取，input 与额外模型的路径解析；
- 占位包、关节改为固定、执行器与传感器替换提议、传感器参数；
- 出错的配方逐项报告；
- 引用不存在的关节与连杆时报错。

`robots.turtlebot4_import`（描述文件缺失时跳过）检查：
- 按配方导入两次，逐字节相同；
- 49 个连杆、两个活动关节；两个速度执行器，没有轮跌落关节；
- 雷达、两个相机、IMU 装在对应的连杆上；
- 落在地面上；
- 直行、原地转向与差速运动学一致；推进 10 s 状态有限；
- 雷达在房间里多于一半的束命中。

`ros2.turtlebot4`（需要先导入）检查：
- 话题名与类型；没有留下默认话题；
- `/scan` 1080 束、frame `rplidar_link`；`/imu` frame `imu_link`；`/joint_states` 没有前缀；
- TF：`odom → base_link`、没有 world；
- `/cmd_vel` 0.2 m/s 时里程计速度与位移；超时停车；1.0 rad/s 时角速度；
- 重置后里程计回到 0。

**测试总数：**不含 ROS2 时，渲染关闭 93 → **94**，渲染开启 128 → **129**；含 ROS2 与宇树消息包 → **141**。

**变异（8 个）：**

| 变异 | 由哪个测试发现 |
| --- | --- |
| 配方的关节覆盖不生效 | `import.analyze` |
| 配方的传感器参数不覆盖默认值 | `import.execute` |
| 导入不写配方 | `import.execute` |
| 差速底盘左右轮反了 | `ros2.turtlebot4` |
| 里程计不积分位置 | `ros2.turtlebot4` |
| cmd_vel 不超时 | `ros2.turtlebot4` |
| 话题名映射不生效 | `ros2.turtlebot4` |
| 坐标系不去前缀 | `ros2.turtlebot4` |

**展示：**
- 查看器打开 TurtleBot 4（房间中），RViz2 显示 TF、雷达、相机与里程计；
- 另开终端运行 `teleop_twist_keyboard` 开动；
- 试新的文件浏览窗口。

## 3. 已知的限制

- 悬崖、红外障碍、碰撞、轮跌落、光学地面跟踪等传感器不做（需要新的传感器类型，M4-09 候选）。
- 雷达会扫到 TurtleBot 4 自己的立柱（真机也会，驱动中一般不过滤）。
- 深度 16UC1 的换算没有自动化测试（Headless 不渲染），在展示中目视检查。
- 导入向导中“重新分析”按通用规则重新提议，会丢掉配方中的执行器与传感器（重新选配方文件即可）。
- 导入在界面线程中执行，TurtleBot 4 的网格转换需要几秒。

## 4. 文件

- 新增：
  - `assets/robots/turtlebot4.terra-import.json`、`assets/scenes/common/room.xml`；
  - `assets/ros2/profiles/turtlebot4.terra-ros2.json`、`assets/ros2/rviz/terra_turtlebot4.rviz`；
  - `assets/tests/import/{demo,bad,wrong_joint}.terra-import.json`；
  - `scripts/assets/import_turtlebot4.sh`、`scripts/verify/m4_08b.sh`；
  - `tests/robots/test_turtlebot4.cpp`、`tests/ros2/check_turtlebot4.py`。
- 修改：
  - `assets/robots/robots.lock`、`scripts/assets/fetch_robots.sh`；
  - `engine/import/`（配方、关节覆盖、占位包、额外模型、传感器参数）、`apps/import/main.cpp`、`apps/viewer/import_wizard.*`；
  - `engine/ros2/`（话题映射、坐标系、差速底盘、16UC1）、`cmake/TerraRos2.cmake`（nav_msgs）、`cmake/TerraCheckRenderIsolation.cmake`；
  - `engine/ui/`（文件浏览窗口）；
  - `tests/CMakeLists.txt`、`tests/import/test_import.cpp`、`tests/ros2/test_ros2_profile.cpp`。

## 5. 验证记录

### 5.1 开发机

未编译、未运行（按约定）。已做的检查：
- 44 个描述文件与固定提交的 git blob 哈希一致；
- 用包装脚本展开 TurtleBot 4 的 xacro（加占位包 `irobot_create_control`）：49 个连杆、48 个关节；活动关节为两个轮跌落（prismatic）与两个轮子（continuous）；总质量 5.39 kg；11 个 Collada 网格；
- 变异锚点合计 276/276 唯一；
- 脚本通过 `bash -n`；Python 语法检查、配方与配置 JSON、RViz 配置 YAML 都能解析。

### 5.2 基线机器

**第一次运行（2026-11-25）：141 项中 140 项通过；变异 8/8；展示正常。**
- 通过的部分：
  - 按配方导入 TurtleBot 4 成功，两次导入逐字节相同（28 个文件）；
  - 直行 5 rad/s 2 s 位移 0.299 m（理论 0.357 m），原地转向 0.493 rad（理论 0.614 rad），都在阈值内；雷达在房间中 1080 束命中 1008 束；
  - `ros2.turtlebot4` 全部通过：`/scan` 的 frame 为 `rplidar_link`；TF 中 `base_link` 的父为 `odom`；`/cmd_vel` 0.2 m/s 时里程计 0.188 m/s、位移 0.393 m；1.0 rad/s 时 0.969 rad/s；
  - 展示中用键盘遥控开动，桥接接受 1113 条 `/cmd_vel`。
- 失败：`robots.turtlebot4_import` 的“落在地面上（base_link 高度在 0～0.1 m）”，实际 −0.0046 m。
  - 原因：Create 3 的 `base_link` 原点就在地面高度（轮子着地时约为 0），接触有几毫米的穿透，是我的阈值设错了。
  - 已修改：阈值改为 −0.02～0.1 m。
- 3 个警告：Create 3 的 `body_visual.dae`、`bumper_visual.dae`、`bumper_collision.dae` 有节点变换（缩放 0.001、绕 X 转 −90°、再放大 10 倍），原始顶点无法核对，按 assimp 的默认处理（撤销 Z_UP 旋转）。
  - 这与 RViz 的处理一致；外观请用户目视确认：底盘与保险杠是否正常。
- **用户意见：**切换场景不好用，导入的机器人场景没有登记到资源库。资源库目前只登记实验记录，场景与导入的登记属于计划中的 M4-08c。已写 [M4-08c 设计稿](M4-08c-library-design.md)。
