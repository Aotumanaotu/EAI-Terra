# M4-09 传感器噪声与分割图：实现报告

> 状态：**通过**（2026-11-25，第二次基线运行）。设计见 [M4-09 设计稿](M4-09-noise-segmentation-design.md)（评审通过；用户补充：噪声可以关闭、雷达参数按数据手册）。
> 验证脚本：`bash scripts/verify/m4_09.sh`。

## 1. 内容

| 部分 | 实现 |
| --- | --- |
| 场景文件 | 场景级 `noise`（`seed`、`enabled`）；传感器的 `noise`（IMU：`gyro` / `accel` 的 `noiseDensity`、`randomWalk`、`bias`、`biasStd`；雷达：`preset`、`rangeStd`、`rangeStdRelative`、`dropout`、`resolution`；相机：`depthStd`、`depthStdQuadratic`、`dropout`、`pixelStd`；都可写 `enabled`）；相机 `segmentation`；场景级 `labels` |
| 雷达预设 | `assets/sensors/lidar_presets.json`：RPLIDAR A1M8、Hokuyo UST-10LX、RPLIDAR S2。逐项写明数据手册原文、出处与换算假设（见设计稿 2.4 节） |
| 噪声 | `engine/simulation/sensor_noise.*`：SplitMix64 的计数器式随机数（种子、传感器名、步序号、分量）、Box–Muller；IMU 白噪声按 σ = 密度 × √频率离散化，随机游走在采样步上累加（步序号变小时重来）；雷达、深度、彩色 |
| 出口 | Headless（`--sensors`、记录、`--no-noise`）、查看器（传感器面板、记录、相机预览与 ROS2；面板中的“噪声”开关）、`terra_capture`（`--no-noise`，`capture.json` 写明）、ROS2 桥接（IMU、雷达；宇树 LowState 的 IMU 也带噪声）、实验记录（头部 `sensors.noise`）与复现（按记录中的设置） |
| 分割图 | 渲染模块的 `DepthRenderer::renderIds`（`RenderItem::id` 为实体编号，平涂写入 R32F，读回为整数）；查看器预览（按 ID 着色，悬停显示实体名）；`terra_capture` 写 uint16 `.npy` 与 `seg_labels.json`；ROS2 `/<相机>/segmentation/image_raw`（16UC1）与 `/<相机>/segmentation/labels`（JSON，transient local） |

**与设计稿的差别：**
- “一份读数”用确定性实现，没有另做共用的采样器（设计稿 2.3 节已改写）；
- 分割图写 `.npy`，不写 16 位 PNG（stb 不能写 16 位 PNG）；
- 实验记录本步不保存分割图；
- 查看器自检没有扩充（噪声开关与分割图预览在展示中看）；
- ROS2 与 Headless 同一步读数相同的端到端测试没有加；由单元测试保证（两个独立的噪声对象逐位相同）。

## 2. 测试

| 测试 | 内容 |
| --- | --- |
| `simulation.sensor_noise`（新） | 随机数；10⁶ 个标准正态；场景文件与预设（含覆盖与错误）；白噪声 σ、随机游走 √T、初始零偏；逐位可复现、与调用顺序无关、重置；三个层次的关闭；快照只改配置了噪声的 IMU；雷达丢失率、量化、σ；深度 a + b·z²、丢失；彩色 σ、截断、不改透明度 |
| `headless.sensors_noise`（新） | 两次输出逐字节相同；`--no-noise` 只改配置了噪声的传感器；记录头 `noise`；带噪声与不带噪声的记录复现都逐位相同 |
| `graphics.capture_noise`（新） | 两次采集逐字节相同；`capture.json` 写明噪声与种子；`--no-noise` 时 IMU、雷达、彩色、深度不同；分割图 uint16 `.npy`、不受噪声影响、对照表 |
| `graphics.camera_capture`（扩充） | 分割图：方块正面像素的 ID、墙为背景 0、逐位可复现、对照表、预览 |
| `headless.cli_parser`（扩充） | `--no-noise` |

**变异（8 个）：**
- 随机数与步序号无关；
- 白噪声没有按频率离散化；
- 随机游走不累积；
- 雷达丢失没有置为 inf；
- 重置后随机游走不回到初值；
- `--no-noise` 不生效；
- 记录中的 IMU 不加噪声；
- 分割图把背景写成 1。

**预期：**渲染关闭 102/102；渲染开启 140/140；含 ROS2 154/154；ASan 含 ROS2 14/14。

## 3. 基线结果

**第一次运行：**
- `SensorConfig` 新增字段后，聚合初始化少写一项，触发 `-Wmissing-field-initializers`，Release `-Werror` 构建失败；
- 测试“固定零偏恒为 0.1”用标准差恰为 0 判断，求和舍入产生 1e-17 的差，测试写法错误，改为逐个比较。

**第二次运行：全部通过。**

| 项 | 结果 |
| --- | --- |
| 渲染关闭 Debug | 102/102 |
| 渲染 Debug、Release -Werror | 140/140、140/140 |
| ASan 非图形 / 图形 | 111/111、29/29 |
| 含 ROS2 Release -Werror | 154/154 |
| ASan 含 ROS2 | 14/14 |
| 变异 | 8/8 |

**统计结果（`simulation.sensor_noise`）：**
- 标准正态 10⁶ 个：均值 0.00020，标准差 1.00110；
- 陀螺白噪声 σ 0.14135（理论 0.14142）；随机游走 400 个样本后 σ 0.001424（理论 0.001414）；
- 雷达：55000 束命中，丢失 0.0504（配置 0.05），误差 σ 0.01001（配置 0.01）；
- 深度 2 m：σ 0.00898（理论 0.00900），丢失 0.0101。

**出口：**
- Headless 两次输出逐字节相同（1163 行）；`--no-noise` 只改配置了噪声的传感器；带噪声与不带噪声的记录复现都逐位相同；
- `terra_capture` 两次采集逐字节相同；分割图 uint16、不受噪声影响、对照表正确；
- 分割图：方块 151 像素（正面 121 像素全部正确），背景 19049 像素。

**展示：**查看器打开噪声夹具运行 123 秒，正常退出。
