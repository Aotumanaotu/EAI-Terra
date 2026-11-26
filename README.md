<div align="center">

# EAI Terra

### An Open Robotics Simulation Engine

**面向具身智能与机器人应用的开放式三维物理仿真引擎**

**MuJoCo Physics · Independent Rendering · Robotics First**

</div>

> **项目状态：Pre-Alpha。**M0～M4 已验收（最近一次：M4 传感器与 ROS2，2026-11-25）；M5 复杂场景与应用尚未启动。
> 没有发布版本，也不提供可用性保证。实际进度以[开发记录](docs/planning/DEVELOPMENT_LOG.md)为准，阶段汇总见[开发总结](docs/planning/SUMMARY.md)。

---

## 1. 简介

EAI Terra 是面向机器人研发、具身智能训练与工业数字孪生的仿真引擎：
- **物理**由 MuJoCo 计算；
- **渲染**用自研的 OpenGL 渲染器；
- **场景、仿真时钟、传感器、记录、资源库与外部接口**由 Terra 自己管理。

Terra 不依赖 Unity 或 Unreal Engine，也不自研刚体求解器。目标是一套**可控、可复现、物理与渲染解耦**的机器人仿真基础设施。

**设计原则：**
- **物理与渲染解耦：**渲染只读取物理快照，不反向修改物理。Headless 程序不链接任何图形库。
- **可复现：**固定步长、按步序号采样，噪声用带种子的计数器式随机数。同一机器上，Headless 运行、实验记录与复现逐位相同；查看器中的运行由 Headless 的复现逻辑逐位复现。
- **可选依赖：**ROS2 桥接、推理库都可以不装。核心库不链接 ROS2，推理库（ONNX Runtime）在运行时按需加载。
- **一键操作：**维护好的场景、动作、策略都是资源库中的卡片，单击即可加载，快捷键只是辅助。

## 2. 现在能做什么

| 能力 | 内容 | 阶段 |
| --- | --- | --- |
| Headless 物理 | 加载 MJCF 或场景文件，固定步进，JSON Lines 输出状态与传感器读数 | M0、M2 |
| 渲染与查看器 | 渲染：PBR、阴影、基于图像的光照、4× MSAA、glTF 资产。<br>界面：编辑器式中英文界面，停靠面板、大纲、属性、统计、设置持久化 | M1 |
| 场景系统 | 场景文件（`*.terra-scene.json`）组合多个模型实例；快照驱动渲染，帧率不影响物理轨迹 | M2 |
| 参考机器人 | 宇树 G1、Go2、Go2W、AS2，Hello Robot Stretch 3，TurtleBot 4 | M3、M4 |
| 动作与控制 | 动作文件、PD 控制层、悬挂模式；执行器面板 | M3 |
| 实验记录 | 记录、按记录重新仿真比较、查看器中的时间轴回放 | M3、M4 |
| 传感器 | IMU；RGB / 深度相机（内参、外参）；2D 激光雷达；分割图。<br>噪声可关闭，激光雷达预设来自数据手册。<br>传感器面板；离线采集程序 `terra_capture` | M4 |
| ROS2（Humble，可选） | `/clock`、`/joint_states`、`/tf`、传感器话题、关节命令、重置 / 暂停 / 单步服务。<br>宇树配置与真机相同的 `lowstate` / `lowcmd`，与官方示例程序闭环；TurtleBot 4 的话题与差速底盘。<br>查看器中的 ROS2 面板与显示配置，可导出 `.rviz` | M4 |
| 模型导入与资源库 | URDF / xacro / MJCF 导入向导；SQLite 资源库：场景、动作、机器人、环境贴图、显示配置、策略的卡片；资产包 | M4 |
| 策略 | 导入 `.onnx` / `.pt`，选机器人后自动配置（观测、动作、PD 增益、历史帧、步态相位）。<br>推理库作为组件按需下载，带进度条，离线时可从文件安装。<br>速度指令面板、键盘驾驶、ROS2 `/cmd_vel` | M4 |

**尚未提供：**
- 3D 激光雷达；
- Python 接口；
- 强化学习训练的接入：已规划，见[开发计划](docs/planning/DEVELOPMENT_PLAN.md) 11.9 节；
- 批量仿真；
- GPU 推理；
- 工业场景示例；
- 发布版本与许可证。

## 3. 快速开始

**环境：**
- 已验证的环境：Ubuntu 22.04 / x86-64 / X11 桌面；
- 编译工具：GCC 11（C++17）、CMake ≥ 3.22、Ninja；
- 从源码构建 MuJoCo 另需 gcc-12；
- 图形测试需要 `xvfb`。

依赖都按版本与 SHA-256 锁定，用显式脚本获取，CMake 不会自动联网。

```bash
# 物理（必需）
sudo apt install gcc-12 g++-12 xvfb
scripts/mujoco/fetch.sh && scripts/mujoco/build.sh       # MuJoCo 4.0.1

# 渲染与查看器（可选）
scripts/render/fetch.sh && scripts/render/build.sh       # GLFW、GLM、Dear ImGui 等
scripts/assets/fetch_hdri.sh && scripts/assets/fetch_models.sh && scripts/assets/fetch_robots.sh

# 构建与测试（预设：debug、release 只有物理；render-debug、render-release 含渲染与查看器）
cmake --preset render-release && cmake --build --preset render-release && ctest --preset render-release
```

**ROS2 桥接**（可选，需要 ROS2 Humble；宇树配置另需 `unitree_ros2`）：

```bash
source /opt/ros/humble/setup.bash
cmake -S . -B build/ros2 -G Ninja -DCMAKE_BUILD_TYPE=Release -DTERRA_BUILD_RENDER=ON -DTERRA_BUILD_ROS2=ON
cmake --build build/ros2
```

**常用程序：**

```bash
# 查看器：欢迎页与资源库中的卡片，单击即打开
build/render-release/apps/viewer/terra_viewer

# Headless：运行场景并输出传感器读数（不需要图形）
build/render-release/apps/headless/terra_headless --scene assets/tests/sensors/sensor_suite.terra-scene.json --duration 2 --sensors

# 播放预定义动作，记录并复现
terra_headless --motion assets/motions/go2/lie_stand_sit.terra-motion.json --record go2.terra-record.jsonl
terra_headless --rerun go2.terra-record.jsonl

# 策略：安装推理组件（184 MB），导入并运行
terra_headless --install-component onnxruntime-cpu
terra_headless --import-policy <policy.onnx> --scene assets/scenes/robots/go2_policy.terra-scene.json
terra_headless --policy <导入得到的 *.terra-policy.json> --command 0.5,0,0 --duration 10

# 离线采集传感器数据（需要图形上下文，可在 Xvfb 中运行）
build/render-release/apps/capture/terra_capture --scene assets/tests/sensors/sensor_suite.terra-scene.json --steps 500 --output capture/

# 导入 URDF / xacro / MJCF，生成场景（查看器中也可以直接拖入文件）
build/render-release/apps/import/terra_import <robot.urdf> --name my_robot
```

`terra_headless --help` 列出全部参数与退出码。

**阶段验收**（统一复现全部证据，需要桌面会话；M4 的验收约 80 分钟）：

```bash
bash scripts/verify/m4_acceptance.sh
```

## 4. 架构与代码结构

```text
            ┌──────────────────────────────────────────────────────────┐
            │ apps：terra_viewer · terra_headless · terra_capture       │
            │       terra_import · terra_bench · terra_render_bench     │
            └─────────────────────────────┬────────────────────────────┘
       ┌───────────────┬──────────────────┼───────────────┬────────────────┐
       │ scene         │ simulation       │ policy        │ ros2（可选）    │
       │ 场景文件与组合 │ 时钟·动作·PD·     │ 说明文件·推理  │ 桥接·宇树配置   │
       │ 传感器配置     │ 传感器·噪声·记录  │ 后端·自动配置  │ 监视器          │
       └───────┬───────┴────────┬─────────┴───────┬───────┴────────┬───────┘
               │      physics（PhysicsWorld 接口；MuJoCo 私有后端）  │
               │                │                 │                │
       render · asset · view · ui · platform · input（不引用物理）   library · import · viz · settings
```

| 目录 | 内容 |
| --- | --- |
| `engine/core` | 构建信息、SHA-256、致命错误接管；`data`：模型描述与物理快照（数据契约） |
| `engine/physics` | `PhysicsWorld` 接口与 MuJoCo 后端（MuJoCo 头文件只出现在后端实现中） |
| `engine/scene` | 场景文件、模型实例组合、传感器与噪声配置、相机模型 |
| `engine/simulation` | 仿真推进与时钟、动作、PD 层、传感器采样、激光雷达、噪声、实验记录 |
| `engine/render`、`asset`、`view`、`ui`、`platform`、`input` | 渲染器、glTF、相机、界面、窗口与输入；不引用物理 |
| `engine/library` | SQLite 资源库、资产包、可下载组件 |
| `engine/import` | URDF / xacro / MJCF 导入 |
| `engine/viz` | 显示配置（`.terviz`）与导出器（`.rviz`） |
| `engine/policy` | 策略说明文件、ONNX 信息、推理后端、自动配置、控制器、导入 |
| `engine/ros2` | ROS2 桥接与监视器（只在 `TERRA_BUILD_ROS2=ON` 时构建） |
| `apps/` | 各程序 |
| `assets/` | 场景、机器人（脚本获取）、动作、传感器预设、组件清单、测试夹具 |
| `tests/` | 单元、集成、图形（Xvfb + llvmpipe）与 ROS2 测试 |
| `scripts/` | 依赖获取与校验、资产工具、各阶段验证脚本（`scripts/verify/`） |
| `third_party/` | 锁定版本的第三方源码与头文件（附许可） |
| `docs/` | 规划文档与验证报告 |

模块边界由测试检查，例如：
- `boundary.render_isolation`：窗口库、图形 API、界面库、JSON、SQLite、miniz、ONNX Runtime、ROS2 的头文件只出现在允许的位置；
- `boundary.no_mujoco_in_public_headers`；
- `headless.no_graphics_libs`。

## 5. 路线图

- [x] **M0 · 基础工程与物理集成**（2026-11-19）：MuJoCo 接入、Headless 固定步进与状态快照、后端无关的物理接口。
- [x] **M1 · 自研基础渲染器**（2026-11-23）：OpenGL 4.5、PBR、glTF、编辑器式界面与设置、参考图与帧耗时基线。
- [x] **M2 · 状态同步与场景系统**（2026-11-23）：场景文件、多模型实例、快照驱动渲染、查看器的仿真模式。
- [x] **M3 · 机器人模型与运动仿真**（2026-11-24）：参考机器人、动作与 PD 控制、实验记录与复现、SQLite 资源库。
- [x] **M4 · 传感器与 ROS2**（2026-11-25）：IMU、相机、激光雷达、噪声与分割；ROS2 桥接与宇树真机通信层；模型导入与资源库卡片；策略的导入与运行。
- [ ] **M5 · 复杂场景与应用**：冻结一个工业任务及规模；端到端示例；性能剖析。强化学习训练的接入按三层推进：
  1. 训练结果回流与评估；
  2. 一键启动外部训练；
  3. Python 接口。

  Studio 功能另立范围。

## 6. 文档

| 文档 | 内容 |
| --- | --- |
| [开发总结](docs/planning/SUMMARY.md) | 各阶段交付与当前边界 |
| [开发记录](docs/planning/DEVELOPMENT_LOG.md) | 实际做过什么、验证证据、下一步 |
| [开发计划](docs/planning/DEVELOPMENT_PLAN.md) | 阶段任务与完成标准 |
| [产品需求](docs/planning/PRD.md) | 需求与优先级 |
| [技术设计](docs/planning/TECHNICAL_DESIGN.md) | 模块边界、数据契约、接口 |
| [技术决策](docs/planning/DECISIONS.md) | 方案选择与理由 |
| [验证报告](docs/validation/) | 每个任务的实现说明与基线机器的运行结果；阶段验收报告：[M0-09](docs/validation/M0-09-acceptance.md)、[M1-11](docs/validation/M1-11-acceptance.md)、[M2-09](docs/validation/M2-09-acceptance.md)、[M3-08](docs/validation/M3-08-acceptance.md)、[M4-11](docs/validation/M4-11-acceptance.md) |
| [文档导航与维护规范](docs/planning/README.md) | 文档如何组织与更新 |

## 7. 第三方组件与许可

| 来源 | 内容 |
| --- | --- |
| `third_party/` | 锁定版本的第三方源码与头文件，各自附许可文本，校验值见 `*.lock` |
| 参考机器人 | MuJoCo Menagerie、unitree_mujoco、TurtleBot 4 / Create 3，许可为 BSD-3-Clause、Apache-2.0 等，随模型目录附带 |
| ONNX Runtime | MIT，作为可下载组件在运行时加载，仓库只带 C API 头文件 |

**本项目自身的许可证待定（D-109）。**在确定之前，不应默认本项目已以某种开源许可证发布。

---

<div align="center">

**EAI Terra — Build a world. Teach a robot.**

*构建世界，让机器人在其中学习。*

</div>
