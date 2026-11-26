# M5-01 范围与场景设计稿

> 状态：**评审通过**（2026-11-26，第 7 节全部按建议；**许可证暂不确定**，见第 6 节）。
> 依据：开发计划第 12 节。12.3 节的 10 个问题，用户已于 2026-11-25 按建议评审通过。用户另外要求：
> - 集成 **Livox MID-360** 3D 激光雷达；
> - 允许上网查数据手册（热像仪、MID-360）。
>
> 本稿冻结 M5 的规模、数据契约与判据。数值以数据手册为准，来源列在第 8 节；不凭空编写参数。

## 1. 规模与性能目标（冻结）

| 项 | 冻结值 | 说明 |
| --- | --- | --- |
| 变电站 | 约 80 m × 60 m；50～150 台设备；至少 1 条巡检路线、10 个以上检查点 | 素材到位后，若规模差别大，在 M5-04 中重新评审 |
| 中型室内 | 约 150～400 m²；8～15 个房间或区域；至少 1 条路线、6 个以上检查点 | 同上 |
| 查看器帧率 | 1080p 不低于 60 FPS（RTX 5060，默认画质，两个场景都打开传感器预览） | 4K 只记录，不判定 |
| 仿真 | Go2 在两个场景中巡检，实时运行，过载 0 次 | 同时开启 MID-360、热像仪与彩色相机 |
| 可复现 | 同一机器上 Headless 巡检两次逐位相同；记录复现逐位相同 | 与 M4 相同 |

## 2. 场景元素（M5-02、M5-03）

### 2.1 元素文件 `*.terra-element.json`

一个元素，就是场景中可以被引用、移动、赋温度的一件东西，如变压器、桌子。

```json
{
  "format": "terra.element", "version": 1,
  "name": "主变压器 110kV", "category": "substation/transformer",
  "visual": {"gltf": "transformer_110kv.glb", "node": "", "scale": 1.0},
  "collision": {"mode": "auto-boxes", "file": ""},
  "physics": {"movable": "layout", "mass": 0},
  "thermal": {"default": 35.0, "parts": {"bushing_a": 60.0}, "hotspots": []},
  "inspection": [{"name": "油位计", "node": "oil_gauge", "view": {"distance": 2.0, "heightDeg": 10}}],
  "license": {"source": "https://…", "license": "CC-BY-4.0", "author": "…"}
}
```

**`collision.mode`：**
- `auto-boxes`：按 glTF 节点取包围盒，默认；
- `auto-hulls`：按节点取凸包，MuJoCo 的网格碰撞按凸包处理；
- `file`：用户给出简化的碰撞网格；
- `none`：只显示，不碰撞。

墙体、地面这类大网格必须拆成节点，导入时检查并警告。

**`physics.movable`：**
- `layout`：只在编辑模式中移动，仿真时静止；
- `dynamic`：自由刚体，机器人可以推动，需要 `mass`；
- `fixed`：不可移动。

**`thermal`：**
- 元素的默认温度（°C）；
- 按 glTF 节点或材质名覆盖；
- 热点：节点上的一个区域，温度更高；
- 都可以写成时间曲线 `[[t, T], …]`，按仿真时间线性插值。

**`inspection`：**检查点，即巡检任务要拍的目标；`view` 给出推荐的拍摄距离与角度。

**`license`：**来源与许可，资源库卡片上显示。

### 2.2 场景文件的扩展

场景文件新增 `elements` 列表。旧版本场景不受影响：

```json
"elements": [
  {"id": "T1", "element": "../elements/transformer_110kv.terra-element.json",
   "position": [12.0, 8.5, 0], "rotation": {"rpyDeg": [0, 0, 90]}, "thermal": {"default": 42.0}}
]
```

- 每个实例有唯一的 `id`；
- 实例可以覆盖元素的温度，例如同型号的两台变压器负载不同；
- 组合时，元素成为一个模型实例：`layout` 与 `fixed` 元素是静态几何，`dynamic` 元素带自由关节；
- 分割图与 M4-09 的类别对照表按元素的 `category` 给出类别。

### 2.3 布局编辑（M5-03）

- **进入方式：**工具栏的“编辑布局”按钮，或按 `Tab`。
  - 进入时仿真暂停，并回到初始状态；
  - 退出时询问是否保存。
- **操作：**

| 操作 | 内容 |
| --- | --- |
| 选择 | 视口单击选中（按渲染的实体 ID 拾取，与分割图相同的通道），或在大纲中选中 |
| 移动、旋转 | 用手柄移动（水平面与高度）、绕竖直轴旋转；按住 `Shift` 吸附到网格（0.1 m / 15°） |
| 贴地 | 向下做射线检测，贴到地面或其他元素的上表面 |
| 拖入 | 从资源库的“元素”分页拖到视口中，落点即射线检测的命中点 |
| 删除 | 删除选中的元素 |
| 撤销 / 重做 | `Ctrl+Z` / `Ctrl+Shift+Z`，每一步可回退 |

- **碰撞检查：**放下时如果与其他元素相交，元素标红并警告，但不阻止放下。
- **保存：**写回场景文件的 `elements`，保留文件中其他内容；也可以“另存为”新场景。保存的场景可以直接仿真，Headless 打开结果相同。
- **机器人：**机器人不在布局编辑的范围内，它的初始位姿仍由场景文件给出。另提供“把机器人放到这里”的快捷操作。

## 3. 热成像相机（M5-06）

### 3.1 模型

- **温度：**每个像素取所见表面的温度。
  - 来源按优先级：热点 → 节点或材质 → 元素实例 → 元素默认 → 场景的环境温度；
  - 不属于任何元素的几何体（如机器人）用场景的 `thermal.robot` 或环境温度；
  - 没有命中的像素（天空）用 `thermal.sky`，默认 −20 °C。
- **渲染：**与分割图相同，多一个通道输出每个像素的温度（float32，°C），与彩色、深度、分割用同一个相机，像素对齐。
- **噪声（M4-09 的方式，可关闭）：**

| 项 | 模型 |
| --- | --- |
| 时间噪声 | 每帧每像素高斯，σ = NETD |
| 固定偏差 | 每台相机一个，按 `biasStd` 从种子取，模拟测温精度 |

- **不模拟的物理（写进报告）：**发射率与反射、大气衰减、热传导与热辐射、镜头渐晕、非均匀校正（NUC）的快门冻结。

### 3.2 输出

| 输出 | 格式 | 说明 |
| --- | --- | --- |
| 辐射测温图 | 16 位无符号，单位 0.01 K | 与 FLIR Lepton 的 TLinear 默认分辨率（0.01 K）相同：读数 = (T + 273.15) × 100；超出量程截断 |
| 伪彩色图 | RGB 8 位 | 色板：`ironbow`（默认）、`white-hot`、`black-hot`、`rainbow`。<br>自动增益按每帧的最低、最高温度，或固定区间（场景文件中写）；8 位灰度另有一路 |
| ROS2 | `…/thermal/image_raw`（`mono16`）、`…/thermal/image_color`（`rgb8`）、`…/thermal/camera_info` | 时间戳与 M4 的相机相同 |
| `terra_capture` | `thermal/<步>.npy`（float32，°C）、`thermal/<步>.png`（伪彩色） | — |
| 实验记录 | 元数据（步、时间、位姿），同 M4-07 | 图像可选写到记录旁 |

### 3.3 参数预设（`assets/sensors/thermal_presets.json`）

| 预设 | 分辨率 | 水平视场角 | 帧率 | NETD | 量程 | 测温精度 | 来源 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `flir_lepton_3_5` | 160 × 120 | 57°（对角 71°） | 8.7 Hz | < 50 mK | 高增益 −10～140 °C；低增益 −10～400 °C | 高增益：取 ±5 °C 或 5% 中较大者；低增益：取 ±10 °C 或 10% 中较大者 | Lepton 产品页，Lepton Series Datasheet（2025-12） |
| `flir_boson_640_50` | 640 × 512 | 50°（9.2 mm，f/1.2） | 60 Hz（可选 30 Hz） | < 40 mK（Industrial 级） | 见下 | 见下 | Boson 产品页与经销商规格 |
| `hikmicro_td5567t_15` | 640 × 512 | 39.9° × 32.4° | 50 Hz（子码流） | ≤ 35 mK | −20～150 °C；0～550 °C | 取 ±2 °C 或 ±2% 中较大者 | 海康威视的产品页 |

**各预设的说明：**
- **Lepton 3.5：**低增益量程的上限有两种写法：
  - 2025-12 的数据手册与 GroupGets 写 400 °C；
  - 部分旧资料写 450 °C；
  - 取 400 °C。
- **Boson 640：**
  - 镜头另有 95°、32°、24°、18°、8° 可选，与 50° 采用同样的写法；
  - 原版 Boson 640 不保证测温，只有部分型号带辐射测温。所以预设的量程与精度用户应按所买的型号填写。默认值取 Lepton 高增益的量程（−10～140 °C），并在报告中写明。
- **海康 HM-TD5567T-15/W：**
  - 这是巡检机器人常用的双光云台，产品页写明可用于巡检机器人集成；
  - 视场角由焦距 15 mm 与像元 17 µm 算出，产品页的 IFOV 为 1.13 mrad，与此一致。经销商页面写 41.7° × 33.3°，可能含镜头畸变；以算出的值为准，记下差别；
  - 可见光通道（2688 × 1520，4.8～153 mm 变焦）用 M4 的彩色相机，固定在最短焦距。

## 4. Livox MID-360（新增 M5-11，P0）

### 4.1 参数（Livox 官方规格页）

| 项 | 值 |
| --- | --- |
| 视场角 | 水平 360°，竖直 −7°～52° |
| 探测距离（100 klx） | 40 m @ 10% 反射率；70 m @ 80% 反射率 |
| 近处盲区 | 0.1 m；0.1～0.2 m 的读数精度不保证 |
| 点频 | 200,000 点 / 秒（首回波） |
| 帧率 | 10 Hz（典型） |
| 距离精度（1σ） | ≤ 2 cm @ 10 m；≤ 3 cm @ 0.2 m |
| 角度精度（1σ） | < 0.15° |
| 内置 IMU | ICM40609 |
| 波长 | 905 nm，Class 1 |

### 4.2 扫描图案

- **来源：**Livox 官方仿真插件 `Livox-SDK/livox_laser_simulation` 中的 `scan_mode/mid360.csv`。
  - 许可：MIT；
  - 锁定提交 `1cce1073633a062b92e30243a4c2920e45551bb5`（2023-03-29）；
  - 文件 sha256 `aa1fc08b6a4400608dbd6ee832b7ea3a9c3c37197e734f60f58fe5abf762269a`，16.8 MB。
- **核查（2026-11-25，只读下载）：**
  - 800,000 行，正好是 200,000 点/秒 × 4 s；
  - 天顶角 37.84°～97.21°，即仰角 52.2°～−7.2°，与规格页的竖直视场角一致；
  - 方位角 0°～360°。
- **使用：**按步序号依次取图案中的方向，每帧（10 Hz）20,000 条射线，4 s 循环一次，与真机的非重复扫描相同。
  - 射线检测沿用 M4-03b 的物理侧做法，不需要图形；
  - 文件由获取脚本下载并校验，不提交进仓库，与参考机器人相同。

### 4.3 噪声与输出

- **噪声（可关闭）：**

| 项 | 模型 |
| --- | --- |
| 距离 | σ 在 0.2 m 处 3 cm、10 m 处 2 cm 之间按距离线性插值，10 m 以外取 2 cm |
| 角度 | σ 0.15° |
| 量程 | 默认最大量程 40 m（10% 反射率的值；Terra 不模拟反射率），可以改成 70 m |
| 盲区 | 0.1 m 以内没有回波 |

- **输出：**

| 出口 | 内容 |
| --- | --- |
| Headless | 点云的元数据与统计 |
| `terra_capture` | `.npy`：x、y、z、每点的时间偏移 |
| ROS2 | `sensor_msgs/PointCloud2`。字段与 `livox_ros_driver2` 的 PointCloud2 格式相同：x、y、z、intensity、tag、line、timestamp。另可选 Livox 的 `CustomMsg`（需要 `livox_ros_driver2` 的消息包，与宇树消息包的做法相同） |
| 查看器 | 传感器面板中显示点云 |

- **内置 IMU：**在 MID-360 的安装位置加一个 IMU。噪声参数取 ICM40609 的数据手册，在 M5-11 中查。

## 5. 巡检任务（M5-07）

### 5.1 任务文件 `*.terra-task.json`

```json
{
  "format": "terra.task", "version": 1, "name": "变电站日常巡检",
  "scene": "substation.terra-scene.json", "robot": "go2", "policy": "go2_walk.terra-policy.json",
  "route": [[0, 0], [20, 0], [20, 15], …], "speed": 0.5,
  "checkpoints": [
    {"at": 2, "target": "T1/油位计", "sensors": ["head_rgb", "thermal"], "dwell": 2.0}
  ],
  "limits": {"duration": 600, "maxTiltDeg": 30}
}
```

### 5.2 运行方式

- **路线跟随器（内置）：**
  - 按航点用 pure pursuit 算速度指令，交给行走策略；
  - 到检查点后停下、转向目标，停留 `dwell` 秒；
  - 在停留结束时采集指定的传感器，以检查点为名保存图像；
  - 确定、可复现，Headless 可用。
- **外部导航：**外部节点（如 Nav2）经 `/cmd_vel` 驱动。Terra 发布检查点与任务状态，并由服务触发采集。

### 5.3 成功判据（每次运行生成任务报告）

| 判据 | 内容 |
| --- | --- |
| 到达 | 所有检查点在限时内到达：距离 ≤ 0.5 m，朝向误差 ≤ 15° |
| 安全 | 不摔倒（机身倾角始终 < `maxTiltDeg`）；与设备的接触力不超过阈值（在 M5-07 中冻结） |
| 拍到目标 | 每个检查点的彩色与热成像图像中，目标元素在分割图中占画面的比例 ≥ 2%，并且没有被完全遮挡 |
| 热像读数 | 目标区域的平均温度与设定温度之差在 NETD 与偏差的范围内 |
| 可复现 | Headless 重跑两次的报告与图像逐位相同 |

## 6. 发布准备（新增 M5-12，P0）

M5 完成后，功能上就是 MVP。要正式发布并给版本号，还需要下面几项，作为 M5 的最后一个工作包。

| 项 | 内容 |
| --- | --- |
| 许可证（D-109） | **用户 2026-11-26：暂不确定。**发布 v0.1.0 之前必须定下，其余各项照常推进。第三方的核查已完成，见 D-109 |
| 第三方声明 | 汇总所有随发布分发的第三方许可：MuJoCo、GLFW、ImGui、字体等，以及组件（ONNX Runtime）的说明 |
| 素材的再分发 | 只分发许可允许再分发的素材。其余（如只能本地使用的）不进安装包，由用户自行获取 |
| 安装包 | 在一台干净的 Ubuntu 22.04 上从安装包安装并运行验收中的关键项。建议先做 `.tar.gz` 与 `.deb`，AppImage 以后再看 |
| 用户文档 | 安装、快速开始、场景与元素、传感器、ROS2、策略、巡检任务、常见问题 |
| 版本号 | 见下 |

**版本号的建议：**
- 现在 `CMakeLists.txt` 中是 `0.0.0`；
- MVP 发布为 **v0.1.0**，按语义化版本：0.x 表示公共接口与文件格式还可能改变；
- 发布前先出 `v0.1.0-rc.1`，在干净机器上验收后转为正式版；
- 文件格式（场景、动作、策略、元素、任务）各自有版本号，与软件版本独立，已经是这样。

## 7. 需要评审的问题

| # | 问题 | 建议 |
| --- | --- | --- |
| 1 | M5-11 MID-360 的优先级 | P0（用户要求）。顺序上与 M5-06 并行：只依赖 M4-03b 的射线检测 |
| 2 | MID-360 的扫描图案 | 用 Livox 官方插件的 `mid360.csv`（MIT），锁定提交与 sha256，由脚本下载 |
| 3 | 点云消息格式 | 默认 PointCloud2（与 `livox_ros_driver2` 相同的字段）；装了 `livox_ros_driver2` 的消息包时另发 `CustomMsg` |
| 4 | 热像仪的默认预设 | 巡检机器人用 `hikmicro_td5567t_15`（640 × 512，双光）；小型平台用 `flir_lepton_3_5` |
| 5 | 版本号 | v0.1.0（MVP），先出 rc |
| 6 | 安装包形式 | `.tar.gz` 与 `.deb`，只支持 Ubuntu 22.04 / x86-64 |
| 7 | 许可证（D-109） | 在 M5-12 中定。需要用户从宽松许可（Apache-2.0 / MIT）或其他中选择；建议 Apache-2.0，与 MuJoCo 相同，带专利授权。**用户 2026-11-26：暂不确定** |

## 8. 来源

**热成像：**
- [FLIR Lepton 产品页（Lepton 3.5，P/N 500-0771-01）](https://oem.flir.com/products/lepton?vertical=microcam&segment=oem)
- [GroupGets：FLIR Lepton 3.5（引自 Lepton Series Datasheet，2025-12）](https://groupgets.com/products/flir-lepton-3-5)
- [FLIR Lepton Engineering Datasheet（TLinear 分辨率 0.1 / 0.01 K）](https://www.mouser.com/pdfDocs/flir-lepton-engineering-datasheet_en.pdf)
- [Teledyne FLIR Boson 640，50° HFOV 9.2 mm](https://www.oem.flir.com/en-hk/products/boson/?model=20640AS50-6IARX&vertical=lwir&segment=oem)
- [Boson 640 镜头型号（经销商）](https://thermal.suntekglobal.com/product/teledyne-flir-boson-640-x-512-9-2mm-50-hfov-short-lens-2/)
- [海康威视 HM-TD5567T-15/W](https://hikvision.az/en/products/thermal/thermography-thermal-cameras/pt-series-thermography-thermal-cameras/hm-td5567t-15-w/)
- [海康威视全球站 HM-TD5567T-15/W](https://www.hikvision.com/en/products/Thermal-Products/Thermography-thermal-cameras/PT-series/hm-td5567t-15-w/)

**MID-360：**
- [Livox Mid-360 规格页](https://www.livoxtech.com/mid-360/specs)
- [Livox-SDK/livox_laser_simulation（MIT）](https://github.com/Livox-SDK/livox_laser_simulation)
