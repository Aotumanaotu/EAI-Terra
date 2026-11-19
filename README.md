# EAI Terra

### An Open Robotics Simulation Engine

**面向具身智能与机器人应用的开放式三维物理仿真引擎**

**MuJoCo Physics · Independent Rendering · Robotics First**

> 项目状态：M0 已完成，M1 进行中（Pre-Alpha）。OpenGL 4.5 core 图形环境与 GLFW/GLM/glad 渲染依赖已完成基线验证、待验收；实际窗口、输入和渲染器尚未实现。详见[开发总结](docs/planning/SUMMARY.md)和[开发记录](docs/planning/DEVELOPMENT_LOG.md)。

</div>

---

## 1. 项目简介

**EAI Terra** 是一个面向机器人研发、具身智能训练和工业数字孪生的仿真引擎项目。

项目以 **MuJoCo** 为首个物理计算后端，结合**自主开发的实时三维渲染器**，将机器人动力学、场景表达、传感器仿真和可视化交互组织为一套可扩展的仿真平台。**Terra 独立管理仿真世界、物理接口、状态同步与渲染系统；MuJoCo 不替代项目整体架构。**

EAI Terra **不依赖 Unity 或 Unreal Engine 作为运行时引擎**。项目首先以 **OpenGL** 建立可运行的渲染管线，并为未来支持 Vulkan 或其他图形后端保留架构空间。

项目的重点不是重新实现 MuJoCo 的动力学求解器，也不是复制完整的通用游戏引擎，而是构建一套**面向机器人应用、可控、可扩展、物理与渲染解耦**的仿真基础设施。

## 2. 项目目标

- **自主可控**：掌握场景、渲染、交互、数据接口与系统集成的核心实现，减少对大型商业游戏引擎的依赖。
- **物理可信**：基于 MuJoCo 进行刚体、关节、碰撞与接触动力学仿真；明确区分物理精度与视觉真实感。
- **视觉表现**：逐步实现 PBR 材质、实时阴影、光照、反射与后处理，支持较高质量的机器人和工业场景呈现。
- **机器人优先**：围绕机器人模型、运动控制、状态观测、传感器和 ROS2 连接设计功能。
- **训练友好**：支持无界面（Headless）仿真、批量实验、记录与回放，为强化学习、模仿学习与策略评估提供基础能力。
- **可扩展**：分离物理引擎、渲染后端和外部接口，避免将系统长期绑定到单一技术栈。

## 3. 总体架构

```text
            ┌─────────────────────────────────────────┐
            │       Applications / Tools              │
            │  仿真编辑器 · 机器人调试 · 策略评估       │
            └──────────────────┬──────────────────────┘
                               │
       ┌───────────────────────▼───────────────────────┐
       │                Terra Core                     │
       │ 场景管理 · 实体映射 · 仿真时钟 · 资源管理 · API  │
       └──────────┬────────────┬────────────┬──────────┘
                  │            │            │
         ┌────────▼─────┐ ┌────▼─────────┐ ┌▼──────────────┐
         │ Physics      │ │ Rendering    │ │ Integration    │
         │ MuJoCo       │ │ OpenGL first │ │ ROS2 / Python │
         │ 刚体/接触/关节│ │ PBR/光影/相机 │ │ 训练与外部应用  │
         └────────┬─────┘ └────▲─────────┘ └▲──────────────┘
                  │            │            │
                  └─────► Scene State ◄─────┘
                    物体姿态 · 关节状态 · 传感器状态
```

### 核心原则：物理与渲染解耦

- **Terra 管理世界并协调物理状态**：由 Core 控制生命周期和固定仿真时间步，MuJoCo 后端计算动力学，适配层发布 Terra 状态。
- **渲染器消费物理状态快照**：以独立帧率绘制画面，必要时对姿态进行插值，不反向修改物理计算结果。
- **应用接口不直接依赖图形 API**：ROS2、Python、训练程序可以在关闭渲染窗口的情况下使用仿真能力。
- **场景与资源分层**：物理场景、可视化资产和运行时状态通过稳定的映射关系关联。

## 4. 技术路线

| 层级 | 初期技术选择 | 说明 |
| --- | --- | --- |
| 核心语言 | C++17 | 仿真与渲染核心；后续可扩展 Python 接口 |
| 物理引擎 | MuJoCo | 动力学、关节、碰撞与接触求解 |
| 图形 API | OpenGL（首选） | 尽快完成独立实时渲染原型 |
| 后续图形后端 | Vulkan（候选） | 视性能需求与开发成本决定是否投入 |
| 机器人描述 | MJCF 优先；评估 URDF 导入 | 保留模型转换和兼容能力 |
| 三维美术资产 | glTF / GLB 优先 | Blender 等工具负责建模与资产制作 |
| 机器人通信 | ROS2 桥接（规划） | 与引擎核心解耦，初期面向 Humble |
| 脚本与训练 | Python API（规划） | 控制、实验、记录与批量评估 |
| 初期开发环境 | Ubuntu 22.04 / x86-64 | 后续再评估其他 Linux / ARM 平台 |

以上为初期技术决策，具体第三方依赖与接口形式在原型验证后确定。

## 5. 核心模块

### Terra Core — 仿真核心

负责仿真时钟、场景对象标识、生命周期、资源管理，以及物理状态与可视化对象之间的同步。

### Terra Physics — 物理层

基于 MuJoCo 完成模型加载、关节驱动、动力学步进、碰撞和接触状态读取。**不计划在初期自研物理求解器**。

### Terra Render — 渲染层

自研渲染器，首期重点包括：网格与材质、相机、基础光照、阴影、PBR、帧缓冲与离屏渲染。后续按需求逐步增加更复杂的反射、全局光照与后处理能力。

### Terra Scene — 场景层

管理地形、建筑、机器人、静态设备和动态物体；建立 MJCF 物理对象与 GLB 可视对象的对应关系。支持逐步扩展工业场景和大规模场景管理能力。

### Terra Sensors — 传感器层（规划）

逐步引入 RGB、深度、分割图、IMU、激光雷达等模拟输出。传感器是否可信取决于具体成像、扫描、时间模型及噪声模型，**不会仅以视觉效果代替传感器物理验证**。

### Terra Bridge — 外部接口（规划）

通过独立接口层与 ROS2、Python 训练程序和外部客户端交互，包括控制命令、机器人状态、传感器数据、录制与回放。

### Terra Studio — 可视化工具（后续）

提供场景浏览、对象属性、机器人状态、仿真启停、调试与可视化操作界面。**编辑器不是最小可运行版本的前置条件**。

## 6. 首期适用场景

1. **机器人运动与控制验证**：导入机器人模型，验证关节控制、底盘运动、碰撞与接触行为。
2. **具身智能策略测试**：为模仿学习、强化学习和策略回归测试提供可重复的物理环境。
3. **工业场景仿真**：搭建变电站、厂房、仓储等场景，用于机器人导航、操作与任务验证。
4. **数字孪生可视化**：与真实机器人状态对接，在三维场景中呈现位姿、关节状态与运行信息。

## 7. 开发路线图

- [x] **M0 · 基础工程与物理集成**（2026-11-19 验收）：验证并锁定 MuJoCo 官方库，加载 MJCF，完成 Headless 固定步进、状态输出、Terra Physics 适配及自动化验收。
- [ ] **M1 · 自研基础渲染器**：使用 OpenGL 建立窗口、相机、基础网格、材质与光照，保持渲染可独立运行。
- [ ] **M2 · 状态同步与场景系统**：通过 Terra 快照驱动渲染，建立稳定实体 ID、场景实例与物理/视觉资源映射。
- [ ] **M3 · 机器人模型及运动仿真**：加载参考机器人，验证连杆、关节、执行器控制和可重复运动。
- [ ] **M4 · 传感器与 ROS2 接口**：验证首批 IMU、RGB/深度和时间语义，提供可选 ROS2 状态与控制桥接；按需扩展其他传感器。
- [ ] **M5 · 复杂场景与仿真应用**：交付代表性工业任务，按需求推进 Python、训练/批量实验、实测性能优化与后续 Studio 工具。

> **第一个里程碑的验收标准**：立方体从空中自由下落，与地面碰撞后稳定，输出仿真时间、位置、姿态及相关物理状态；全过程不创建 OpenGL、EGL 或 OSMesa 上下文。完整条件见[开发计划](docs/planning/DEVELOPMENT_PLAN.md)。文档规划覆盖全项目，开发按阶段推进。

## 8. 暂不纳入首期范围

- 从零研发通用刚体动力学与接触求解器。
- 一开始就同时维护 OpenGL 和 Vulkan 两套完整渲染后端。
- 在首个版本复制 Unity / UE5 的完整编辑器、资产流水线和所有视觉特效。
- 未经测试就宣称达到电影级实时渲染、严格的传感器真实性或大规模仿真性能。

## 9. 计划中的目录结构

```text
eai-terra/
├── README.md
├── docs/
│   └── planning/         # 需求、开发计划、技术设计、决策与开发记录
├── engine/
│   ├── core/             # 仿真时钟、场景对象与状态同步
│   ├── physics/          # MuJoCo 适配层
│   ├── render/           # OpenGL 渲染器
│   ├── scene/            # 资源、场景与实体映射
│   └── sensors/          # 传感器仿真
├── bridges/
│   ├── ros2/             # ROS2 接口
│   └── python/           # Python 接口
├── apps/                 # 示例、调试器和后续 Studio
├── assets/               # 示例场景与模型
└── tests/                # 核心逻辑、物理与渲染测试
```

上述目录为规划草案，以实际代码演进为准。

### 构建

需要 GCC 11（C++17）、CMake ≥ 3.22、Ninja；从源码构建 MuJoCo 另需 gcc-12/g++-12：

```bash
sudo apt install gcc-12 g++-12   # 仅首次
scripts/mujoco/fetch.sh          # 下载并校验 MuJoCo 源码（版本见 third_party/mujoco/mujoco.lock）
scripts/mujoco/build.sh          # 构建并安装到 third_party/mujoco/install
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

运行立方体下落示例（无图形，stdout 为 JSON Lines：config → model → state … → summary）：

```bash
build/debug/apps/headless/terra_headless --model assets/tests/falling_cube.xml --duration 5 --sample-every 500
```

渲染（M1 起，可选；默认关闭，不影响上面的物理构建）：

```bash
scripts/render/fetch.sh          # 下载并校验 GLFW、GLM（版本见 third_party/render/render.lock）
scripts/render/build.sh          # 构建 GLFW 静态库到 third_party/render/install
cmake --preset render-debug && cmake --build --preset render-debug && ctest --preset render-debug
```

图形测试带 `graphics` 标签，固定在 Xvfb + Mesa llvmpipe 上运行（需要 `xvfb`）。

完整复现 M0 验收（三种构建配置的全量测试、变异测试、场景判定、重复运行、依赖检查与性能基线，约数分钟）：

```bash
bash scripts/verify/m0_acceptance.sh
```

说明与验证记录见 [`docs/validation/`](docs/validation/)，M0 汇总见 [M0-09 验收报告](docs/validation/M0-09-acceptance.md)。

## 10. 当前状态

**M0（基础工程与 MuJoCo Headless 物理集成）已于 2026-11-19 完成验收**，下一阶段为 M1。已实现：MuJoCo 4.0.1 源码构建接入、Headless 程序、MJCF 加载、固定步进与 reset、不可变状态快照、后端无关的 `PhysicsWorld` 接口与执行器控制；立方体主场景与辅助用例在基线机器上通过，数值容差已冻结。无图形界面，未提供发布版本或可用性保证。实际进度统一维护在[开发记录](docs/planning/DEVELOPMENT_LOG.md)。

近期第一优先级：评审 M1-01 图形环境、M1-02 渲染依赖及 M1-03/04 平台窗口与输入抽象设计；评审通过后开始实现。

### 规划文档入口

项目主 README 保留在根目录，详细规划及开发记录集中在 `docs/planning/`：

- [文档导航与维护说明](docs/planning/README.md)
- [验证报告](docs/validation/)（M0-01 至 M0-09、M1-01 至 M1-02）
- [产品需求 PRD](docs/planning/PRD.md)
- [开发计划与阶段验收](docs/planning/DEVELOPMENT_PLAN.md)
- [技术架构与设计](docs/planning/TECHNICAL_DESIGN.md)
- [技术决策记录](docs/planning/DECISIONS.md)
- [开发记录与当前进度](docs/planning/DEVELOPMENT_LOG.md)
- [开发总结与下一步](docs/planning/SUMMARY.md)

## 11. 许可证

**待定（TBD）**。在正式确定授权方式与第三方依赖约束之前，不应默认本项目已以某种开源许可证发布。

---

<div align="center">

**EAI Terra — Build a world. Teach a robot.**

*构建世界，让机器人在其中学习。*

</div>
