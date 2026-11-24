# M4-03a 离屏渲染与相机

> 状态：**已实现，待基线机器验证**（第 6 节）。依据：[M4-01 传感器设计稿](M4-01-sensor-design.md)（2026-11-24 评审通过）第 3～5 节。

## 1. 交付

**渲染（`terra_render`）：**
- 帧缓冲新增单通道 32 位浮点格式（`ColorFormat::R32F`）与浮点读回（`Framebuffer::readFloats`）。
- 新的 `DepthRenderer`：片元写出相机坐标系中的 −z，即**沿光轴的距离**（米），单精度，不经过深度缓冲的非线性映射。
  - 只画不透明的渲染项，碰撞几何体的半透明叠加不进入深度；
  - 不做多重采样（每个像素取像素中心）；
  - 清为 0，读回时把 0 换成 +inf：没有命中、或在近 / 远平面之外的像素为 +inf（ROS REP-117 的约定）；
  - 行序转为图像约定（行 0 为顶部）。

**数据契约：**模型描述修订 7。`CameraDescription` 列出 MJCF 中的全部相机：名字、安装实体、相对实体的位姿、竖直视场角、MJCF 写的分辨率、是否固定、是否透视。

**场景文件 `sensors` 中的相机：**
```json
{"type": "camera", "camera": "stretch3/d435i_camera_rgb", "width": 640, "height": 480, "rateHz": 30},
{"type": "camera", "name": "g1/head_camera", "entity": "g1/torso_link", "position": [0.06, 0, 0.45],
 "rotation": {"rpyDeg": [90, 0, -90]}, "fovyDeg": 58, "near": 0.1, "far": 10, "rgb": true, "depth": true}
```
- 用 MJCF 相机（`camera`）或装在实体上（`entity` + 位姿）。相机看向自身 −Z、+Y 向上，与 MJCF 相同。
- 默认值：640 × 480（MJCF 写了分辨率时取它）、视场角 60°（或 MJCF 中的值）、0.05～50 m、30 Hz、彩色与深度都输出。
- 只支持固定在实体上的透视相机：跟踪模式与正交投影的 MJCF 相机报错。
- 雷达（`lidar2d`）在 M4-03b 加入。
- `SceneInstance::cameras`：启用的相机，按场景文件的顺序。

**与设计稿的一处细化：**设计稿把相机列在 `ModelDescription::sensors` 中。实现时，相机作为场景层的配置（`SceneInstance::cameras`），不进入物理侧的传感器清单与快照。原因：相机不需要物理后端计算任何东西，由渲染侧按快照生成图像；这样物理与渲染的边界保持不变。

**相机模型（`terra/scene/camera_model.hpp`，纯数学）：**
- 内参：fy = (h / 2) / tan(fovy / 2)，fx = fy，主点 ((w − 1) / 2, (h − 1) / 2)。像素中心为整数坐标，即 OpenCV / ROS 的约定。
- 光学坐标系（看向 +Z、+Y 向下）由 Terra 相机坐标系绕 X 轴转 180° 得到。
- 相机的世界位姿 = 安装实体的位姿 × 安装位姿。
- 世界点 → 像素的投影。

**按快照渲染（`terra_viewer_scene` 的 `CameraRenderer`）：**
- 彩色：与视口相同的渲染器（PBR、阴影、多重采样、色调映射），不画地面网格；8 位 RGB。
- 深度：`DepthRenderer`。
- 位姿与时间取自给定的快照（采样那一步），不使用显示帧的插值姿态；渲染项不高亮、不叠加碰撞几何体。
- 离屏目标在第一次渲染时才创建：没有相机就不创建。
- 附带深度伪彩色预览（`depthPreview`）与彩色转图像（`colorImage`），供测试写 PNG，M4-04 的传感器面板也会用到。

**Headless：**场景中有相机时在 stderr 提示“相机需要图形上下文，在 Headless 中不可用”，其余照常运行。Headless 仍不链接任何图形库。

## 2. 夹具

`assets/tests/sensors/camera_bench.xml` 与 `camera_bench.terra-scene.json`。无重力，一切静止：

| 物体 | 位置 | 用途 |
| --- | --- | --- |
| 相机 eye | 支架 rig 上，(0, 0, 1)，看向世界 +X，60° | MJCF 相机 |
| 相机 side | 同一支架，场景文件用 rpyDeg (90, 0, −90) 装上，只输出深度 | 装在实体上的相机，与 eye 看向同一方向 |
| 墙 | 正面在 x = 2.0，充满视野 | 深度处处为 2.0 m |
| 红色方块 | 中心 (1.0, 0.3, 1.2)，半边长 0.05 | 正面在 x = 0.95；正面中心的投影可按内参算出 |
| 相机 tracker | 跟踪模式 | 引用它应报错 |

## 3. 测试

| 测试 | 内容 |
| --- | --- |
| `scene.camera_model`（新，不需要图形） | 内参与光学坐标系；投影（光轴上的点、方块正面中心、相机后方）；MJCF 相机的描述；场景文件中两种相机的安装位姿与参数、默认值、频率上限；12 种解析错误与 4 种组合错误 |
| `graphics.camera_capture`（新） | 墙的深度为 2.0 m（误差 < 1e-5）、没有空白；方块正面质心与投影一致（±0.6 像素）；far = 1.5 时墙为 +inf；方块中心为红色、墙为灰色；帧的步序号与位姿与快照一致；两种写法的深度一致；渲染两次逐字节相同；离屏目标只在第一次渲染时创建；Stretch 3 头部相机（有参考机器人时）。写出 PNG（彩色 \| 深度） |
| `headless.cameras_ignored`（新） | 有相机的场景在 Headless 中照常运行，stderr 提示 |
| `scene.sensors`（修改） | `camera` 不再报“不支持”；写 `site` 时报未知字段；雷达报 M4-03b |
| `physics.imu`（修改） | 描述修订号的检查改为“≥ 6” |

**测试总数：**渲染关闭 78 → **80**，渲染开启 107 → **110**。

**变异（10 个）：**
- 没有命中时不为 +inf；
- 深度图行序没有翻转；
- 主点取 w / 2；
- 投影的 v 方向反了；
- 观察矩阵没有取逆；
- 投影矩阵不除宽高比；
- MJCF 相机的视场角丢失；
- 彩色通道顺序错误；
- 实体上的相机丢失安装位姿；
- 深度渲染器不清除。

**展示：**验证脚本最后用系统默认的看图程序打开两张图片（夹具、Stretch 3 头部相机），左边彩色、右边深度。查看器中的传感器面板在 M4-04。

## 4. 已知的限制

- 深度不做多重采样，物体边缘是锯齿状的，真实的深度相机也是如此；彩色有多重采样。
- 相机的频率已经保存，但目前还没有按频率采样的使用者：查看器的预览与 `terra_capture` 在 M4-04，按 `sensorSampleDue` 采样。
- 没有畸变模型（P1 的噪声与误差一并考虑）。

## 5. 文件

- 新增：
  - `engine/render/include/terra/render/depth_renderer.hpp`、`engine/render/src/depth_renderer.cpp`；
  - `engine/scene/include/terra/scene/camera_model.hpp`、`engine/scene/src/camera_model.cpp`；
  - `apps/viewer/camera_sensor.hpp`、`apps/viewer/camera_sensor.cpp`；
  - `assets/tests/sensors/camera_bench.xml`、`camera_bench.terra-scene.json`；
  - `tests/sensors/test_camera_model.cpp`、`tests/sensors/test_camera_capture.cpp`；
  - `scripts/verify/m4_03a.sh`。
- 修改：
  - `engine/render/{CMakeLists.txt,include/terra/render/resources.hpp,src/resources.cpp}`；
  - `engine/core/data/include/terra/data/model_description.hpp`；
  - `engine/physics/backends/mujoco/src/mujoco_world.cpp`；
  - `engine/scene/{CMakeLists.txt,include/terra/scene/scene.hpp,src/scene_file.cpp,src/scene_instance.cpp}`；
  - `apps/viewer/CMakeLists.txt`、`apps/headless/main.cpp`；
  - `tests/CMakeLists.txt`、`tests/sensors/test_imu.cpp`、`tests/sensors/test_scene_sensors.cpp`。

## 6. 验证记录

### 6.1 开发机

未编译、未运行（按约定）。已做的静态检查：
- 变异锚点合计 216/216 唯一；
- 脚本通过 `bash -n`。

### 6.2 基线机器

（待运行 `bash scripts/verify/m4_03a.sh` 后填写。）
