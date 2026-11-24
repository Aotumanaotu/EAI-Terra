# M4-03b 2D 激光雷达

> 状态：**已实现并通过基线验证**（2026-11-24 第二次运行全部通过，第 6.2 节）。依据：[M4-01 传感器设计稿](M4-01-sensor-design.md)（2026-11-24 评审通过，第 8 节问题 4：默认打参与碰撞的几何体）；开发计划 11.6 节（用户选定 TurtleBot 4，2D 激光雷达由 P1 提前到 P0）。
> 3D 雷达（MID-360、L1 的非重复扫描图案）不在本步，仍在 M4-09（P1），视进度可递延到 M5（LOG-138）。

## 1. 交付

**物理接口：**`PhysicsWorld::castRays(RayQuery)`，在当前观测时刻的几何上求每条射线到第一个命中面的距离，`maxRange` 内没有命中为 +inf。
- `exclude`：忽略安装它的实体的几何体，雷达不会打到自己的外壳；
- `hits = Collision`（默认）：只打参与碰撞的几何体，与物理一致。MuJoCo 的射线不区分碰撞与显示，所以打到不参与碰撞的几何体时，从命中点稍向前继续，最多 64 次；
- `hits = Visual`：只打默认显示的分组（MJCF 第 0～2 组），更接近真实外形；
- 完全透明（alpha = 0）的几何体不会被打到，这是 MuJoCo 的规则。

射线检测是一般的几何查询，以后别的物理后端也有，因此放在后端无关的接口中。

**场景文件：**
```json
{"type": "lidar2d", "name": "turtlebot4/scan", "entity": "turtlebot4/rplidar_link", "position": [0, 0, 0.02],
 "angleMinDeg": -180, "angleMaxDeg": 179, "samples": 360, "rangeMin": 0.15, "rangeMax": 12, "rateHz": 10, "hits": "collision"}
```
- 装在实体上，或 `"entity": "world"`。`name` 必填，装在实体上时带该实例的前缀。
- 默认值：−180°～180°、360 束、0.15～12 m、10 Hz、collision。频率高于物理频率时按物理频率并警告。
- `SceneInstance::lidars`：角度已换算为弧度。

**扫描（`terra/simulation/lidar.hpp`）：**`scanLidar(world, lidar)`，只依赖物理接口，Headless 与查看器共用。约定与 `sensor_msgs/LaserScan` 一致：
- 扫描平面为安装坐标系的 XY 平面，角度从 +X 起逆时针为正；
- 第 i 束角度 = angleMin + i × (angleMax − angleMin) / (samples − 1)。360° 一圈不想让首尾两束重合时，把 angleMaxDeg 写为 180 − 360 / samples；
- 距离为单精度米；超出最大量程为 +inf，近于最小量程为 −inf（ROS REP-117）；
- 时间标识（步序号、时间、运行代次）与位姿取自当前快照。

读数只是几何距离，没有噪声与强度（P1）。

**Headless `--sensors`：**按每个雷达的频率输出 `lidar` 行，只在采样步上做射线检测。JSON 没有无穷大：+inf 写作字符串 `"inf"`，−inf 写作 `"-inf"`。

**俯视图：**`scripts/sensors/plot_lidar.py` 只用 Python 标准库，把 Headless 输出的一次扫描画成 PNG。验证脚本用它画图并打开。

## 2. 夹具

`assets/tests/sensors/lidar_room.xml` 与 `lidar_room.terra-scene.json`：
- **房间：**三面墙的内表面在 x = ±2、y = −2，+Y 一侧敞开；
- **雷达支架：**在 (0.5, 0, 0.2)，外壳是参与碰撞的方块，把雷达原点包在里面；
- **柱子：**只用于显示，不参与碰撞，中心 (−1, −1)；
- **两个雷达：**装在支架上方 0.1 m，偏航 90°，360 束，1° 间隔；一个 collision 模式，一个 visual 模式。

每一束的距离都有解析解：射线与墙面求交，visual 模式另加射线与圆求交。

## 3. 测试

| 测试 | 内容 |
| --- | --- |
| `simulation.lidar`（新） | **角度：**期望角度在测试中独立计算（−180° + i°），首尾两束的角度。**collision 模式：**360 束都与解析距离一致（< 1e-5 m）；朝敞开一侧为 +inf；穿过柱子；不打到外壳。**visual 模式：**打到柱子的几束与射线—圆求交一致。**量程：**最小量程 2.1 m 时较近的为 −inf；最大量程 1 m 时全部为 +inf。**其他：**步序号、时间、位姿；扫描两次逐位相同；默认值、装在世界上、频率上限；8 种场景文件错误 |
| `headless.sensors_lidar`（新） | 第 0 步输出 bench/scan 的扫描，角度从 −π 开始，没有命中的写作 `"inf"` |
| `scene.sensors`、`scene.camera_model`（修改） | `lidar2d` 不再报“不支持”：写 `site` 报未知字段，缺 name 报错 |

**测试总数：**渲染关闭 80 → **82**，渲染开启 110 → **112**。

**变异（9 个）：**
- 不穿过显示用的几何体；
- 不忽略安装实体；
- 打到不参与碰撞的几何体；
- 不限最大量程；
- 角度增量少除一；
- 安装位姿丢失；
- 近于最小量程不为负无穷；
- 角度没有换算为弧度；
- Headless 的 inf 不写成字符串。

## 4. 已知的限制

- 没有噪声、强度、运动畸变（扫描期间机器人在动时，真实雷达各束的时刻不同；这里整圈取同一步）。
- collision 模式下，完全透明的碰撞几何体打不到（MuJoCo 的规则）。Menagerie 的碰撞几何体都不是完全透明的。
- 查看器中的显示在 M4-04（传感器面板）。

## 5. 文件

- 新增：
  - `engine/simulation/include/terra/simulation/lidar.hpp`、`engine/simulation/src/lidar.cpp`；
  - `assets/tests/sensors/lidar_room.xml`、`lidar_room.terra-scene.json`；
  - `tests/sensors/test_lidar.cpp`；
  - `scripts/sensors/plot_lidar.py`、`scripts/verify/m4_03b.sh`。
- 修改：
  - `engine/physics/include/terra/physics/physics_world.hpp`；
  - `engine/physics/backends/mujoco/src/{mujoco_world.hpp,mujoco_world.cpp}`；
  - `engine/scene/{include/terra/scene/scene.hpp,src/scene_file.cpp,src/scene_instance.cpp}`；
  - `engine/simulation/CMakeLists.txt`；
  - `apps/headless/{cli.hpp,cli.cpp,main.cpp}`；
  - `tests/CMakeLists.txt`、`tests/sensors/{test_scene_sensors,test_camera_model}.cpp`；
  - `scripts/verify/m4_03a.sh`（变异改名，统计改用 awk）。

## 6. 验证记录

### 6.1 开发机

未编译、未运行（按约定）。已做的检查：
- 变异锚点合计 225/225 唯一；
- 脚本通过 `bash -n`；
- `plot_lidar.py` 用一组构造的数据生成了 PNG 并查看过。

### 6.2 基线机器

**第一次运行（2026-11-24）：未通过，两处测试问题，已修正（雷达本身的结果正确）。**
- `simulation.lidar` 通过：360 束中 85 束朝敞开一侧，其余与解析距离一致；Headless 的扫描与俯视图正常。
- `headless.sensors_lidar` 失败（三种构建与 ASan 都是这一项）：程序输出正确，是我写的正则用了 `[[]` 与带引号的 `\"inf\"`，在 CMake 的正则与参数传递中不可靠。改为 `[^0-9,]inf[^0-9,]`：带引号的 `"inf"` 两侧是引号，裸写的 inf（变异）两侧是逗号，仍能区分。
- 变异 8/9：“角度增量少除一”未被发现。测试计算期望角度时调用的正是被测的 `lidarBeamAngle`，两边一起错。改为在测试中独立计算（−180° + i°），并检查第一束在 −180°、最后一束在 179°。
- “Headless 的 inf 不写成字符串”显示“0 项检查失败”：未修改时该测试本身就失败，变异没有判定意义；修正正则后重新判定。

**第二次运行（2026-11-24）：全部通过。**
- 渲染关闭 82/82；渲染开启 Debug 与 Release -Werror 112/112；ASan 90/90 + 22/22；变异 9/9（每个 1～7 项检查失败）。
- 360 束中 85 束朝敞开一侧，其余与解析距离一致；Headless 两次扫描各 275 束命中；两张俯视图已打开。
