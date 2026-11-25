# M4-07 传感器记录

> 状态：**已实现并通过基线验证**（2026-11-25，第 5.2 节）。依据：开发计划 11.2 节 M4-07：
> - 实验记录增加传感器读数（IMU 每份状态都记）；
> - 图像只记元数据（步序号、时间、位姿），图像本身可选写到记录旁的目录；
> - 回放时传感器面板跟随；
> - 验收：记录与复现时传感器读数逐位相同。

## 1. 交付

**记录格式版本 2**（`*.terra-record.jsonl`）：

| 行 | 内容 |
| --- | --- |
| `header` | 另有 `sensors`：IMU 名字（按传感器 ID 顺序）、雷达（名字、频率、束数）、相机（名字、频率、分辨率） |
| `state` | 另有 `sensors`：全部 IMU 的读数，每个 10 个数（比力、角速度、姿态），与快照相同 |
| `lidar`（新） | 雷达在其采样步上的一次扫描；没有命中写成字符串 `"inf"` / `"-inf"`（JSON 没有无穷大），读回逐位相同 |
| `camera`（新） | 相机在其采样步上的元数据：步序号、时间、相机位姿（Terra 相机坐标系在世界中） |
| `camera_image`（新） | 该帧的图像已保存（相对记录文件所在目录的路径） |

- 采样时刻与 Headless `--sensors`、`terra_capture` 相同（`sensorSampleDue`）。
- 版本 1 的记录照常读取（没有传感器）；资源库索引不受影响。

**Headless：**`--record` 时自动记录场景中的传感器。Headless 不渲染，相机只记元数据。`--rerun` 的输出新增 `sensor_samples` 与 `sensors_bitwise`；传感器不逐位相同时在标准错误中给出第一处。退出码仍按位姿容差判定。

**复现：**以下三类都逐位比较；传感器不设容差，跨机器时只作报告。
- IMU：每份状态；
- 雷达：在记录的采样步上重新扫描；
- 相机位姿：在记录的采样步上。

**查看器：**
- 记录时自动写入 IMU、雷达与相机元数据。
- 新菜单项“仿真 → 记录相机图像”：同时把**实际渲染的**相机帧保存到记录旁的 `<记录名>.frames/<相机>/{rgb/*.png, depth/*.npy}`，并在记录中登记。
  - 查看器每帧最多渲染每个相机的最新一份，追赶时中间的帧只有元数据；不丢帧的完整图像仍用 `terra_capture`。
- **回放时传感器面板跟随记录：**
  - IMU 曲线取不晚于回放时间的状态中的读数；
  - 雷达俯视图取最后一次扫描；
  - 相机显示最后一帧：有保存的图像时读入彩色与深度，深度可悬停取值；没有保存时提示“记录中没有这一帧的图像”。
  - 回到实时后，从当前状态重新开始采样。
- 记录开始时（重置后的第 0 步）的雷达与相机也写入记录。

## 2. 测试

| 测试 | 内容 |
| --- | --- |
| `record.file`（扩充） | 见下方说明 |
| `record.sensors`（新） | 见下方说明 |
| `headless.record_sensors`、`headless.rerun_sensors`（新） | sensor_suite 200 步；复现 `sensor_samples` 为 219（201 份状态 + 雷达 5 次 + 相机 13 帧），`sensors_bitwise` 为 true |
| `graphics.viewer_sensor_record_selftest`（新） | 见下方说明 |

`record.file`（扩充）检查：
- 版本 2；
- 记录头的传感器清单、状态中的 IMU、雷达距离（含 1/3、±inf）、相机位姿与图像路径往返逐位相同；
- camera_image 没有对应的 camera 行、距离字符串不是 inf 时报错；
- 版本 1 的记录照常读取；
- 版本 3 报“高于本程序支持的版本”。

`record.sensors`（新）：sensor_suite 记录 300 步。
- 31 份状态各带 4 个 IMU；
- 雷达 7 次，每次 85 束 +inf；相机元数据 19 帧；
- 复现 57 个传感器采样逐位相同；
- 改动第 100 步雷达的一个距离后，复现报告“第 100 步雷达不同”。

`graphics.viewer_sensor_record_selftest`（新）：查看器中第 6 帧开始记录（打开“记录相机图像”），中间停止并回放到一帧保存了图像的时刻。检查：
- 传感器清单；每份状态都有 IMU；
- 雷达与相机在每个采样步上都有一行；
- 实际渲染的帧有图像文件；
- 复现时 IMU、雷达、相机位姿逐位相同；
- 回放时 IMU 曲线、雷达俯视图、相机彩色与深度都取自记录（中心深度 2 m）；
- 没有 GL 错误。

**测试总数：**渲染关闭 85 → **88**，渲染开启 118 → **122**。

**变异（7 个）：**

| 变异 | 由哪个测试发现 |
| --- | --- |
| 状态不记 IMU | `record.file` |
| 雷达的无穷大写成 null | `record.file` |
| 复现不比较雷达 | `record.sensors` |
| 回放时传感器面板不跟随 | `graphics.viewer_sensor_record_selftest` |
| 不保存相机图像 | `graphics.viewer_sensor_record_selftest` |
| 记录开始时漏记雷达与相机 | `graphics.viewer_sensor_record_selftest` |
| Headless 不记录雷达与相机 | `headless.rerun_sensors` |

**展示：**查看器播放 Stretch 3 巡游，按提示勾选“记录相机图像”、记录 10 秒并回放，看传感器面板跟随记录。

## 3. 已知的限制

- 查看器只保存实际渲染的相机帧；其余帧只有元数据。完整的图像序列用 `terra_capture`。
- IMU 只在状态采样点上记录（与计划“每份状态都记”一致）。想要更密，可以调高记录采样率（菜单“记录采样率”，最高 500 Hz）。
- 记录文件随传感器变大。例如 sensor_suite 300 步、每 10 步采样一次，见 `record.sensors` 的 INFO 行。
- 传感器的复现只在同一机器上要求逐位相同，跨机器只作报告。

## 4. 文件

- 新增：`tests/` 中的测试见上；`scripts/verify/m4_07.sh`。
- 修改：
  - `engine/simulation/{include/terra/simulation/record.hpp,src/record.cpp}`；
  - `apps/headless/main.cpp`；
  - `apps/viewer/{main.cpp,camera_sensor.hpp,camera_sensor.cpp}`；
  - `engine/ui/{include/terra/ui/text.hpp,src/text.cpp}`；
  - `tests/CMakeLists.txt`、`tests/simulation/test_record.cpp`。

## 5. 验证记录

### 5.1 开发机

未编译、未运行（按约定）。已做的静态检查：
- 变异锚点合计 260/260 唯一；
- 脚本通过 `bash -n`；
- 期望的采样次数用 Python 按 `sensorSampleDue` 计算：200 步内雷达 5 次、相机 13 帧；300 步内 7 次、19 帧。

### 5.2 基线机器

**第一次运行（2026-11-25）：全部通过。**
- 测试：88/88；渲染开启 Debug 与 Release -Werror 122/122；ASan 97/97 + 25/25；变异 7/7。
- `record.sensors`：31 份状态（各 4 个 IMU），雷达 7 次（+inf 共 595 束），相机 19 帧，96 059 字节；复现 57 个传感器采样逐位相同。
- 查看器自检：
  - 记录第 0～176 步：18 份状态、雷达 4 次、相机 11 帧，均与采样规则一致，11 帧都保存了图像；
  - 复现 33 个传感器采样逐位相同；
  - 回放到第 84 步：IMU、雷达（第 50 步）、相机图像都取自记录，中心深度 2.0000 m。
- 展示：Stretch 3 巡游，用户记录 10.46 秒（524 份状态），保存 157 张 PNG 与 157 个 .npy，并回放；查看器运行 72.6 秒，正常结束。
