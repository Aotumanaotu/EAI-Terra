# M0-04 MJCF 模型加载

> 状态：**已验收**（用户于 2026-11-19 确认评审通过）。立方体夹具、`terra_data` 数据契约、`MujocoWorld` 加载与初始化、Headless 接入和测试均已完成；基线机器 Debug/Release 22/22 通过（5.2 节）。

## 1. 交付内容

| 部分 | 位置 | 说明 |
| --- | --- | --- |
| 立方体主夹具 | `assets/tests/falling_cube.xml` | 按开发计划 4.1 节：边长 0.2 m、质量 1 kg、中心 (0,0,1)、具名自由关节 `cube_free`、具名地面 `ground`；步长 0.001 s、Euler 积分；求解器、迭代次数、容差、摩擦、solref/solimp、margin/gap 全部显式写出 |
| `terra_data` | `engine/core/data/include/terra/data/` | 仅头文件的数据契约：`EntityId`/`JointId`（0 为无效）、`Vec3`/`Quaternion`(wxyz)/`Pose`、`ModelSource`、`ModelDescription`（实体、关节、步长、来源路径） |
| `MujocoWorld` | `engine/physics/backends/mujoco/` | `load()` → 文件检查 → `mj_loadXML` → `mj_makeData` → `mj_forward` → 建立描述与私有映射；提供 `description()`、`simulationTime()`、`entityPose()` |
| Headless 接入 | `apps/headless/main.cpp` | 运行与 `--dry-run` 都会加载模型；`--dry-run` 输出 `timestep` 和 `total_steps`；`--duration` 按模型步长换算为步数 |

`terra_data` 不依赖 Core 的运行实现，Core 和 Physics 都依赖它，按技术设计 1.2 节避免 Core ↔ Physics 循环依赖。

## 2. 关键设计

- **加载原子性。** 只有全部步骤成功才返回世界；任何一步失败只返回错误，不留下可用的半成品。模型和数据用 `unique_ptr` 加自定义删除器管理；`Impl` 中 `model` 声明在 `data` 之前，因此析构时数据先于模型释放。
- **错误分类：**

  | 类型 | 触发条件 | Headless 退出码 |
  | --- | --- | --- |
  | `FileUnavailable` | 路径为空、不存在、不是常规文件、不可读 | 4 |
  | `InvalidModel` | MJCF 解析或编译失败（含 include 或资源文件缺失、未知属性），步长无效，初始状态含非有限值，不支持的关节类型 | 4 |
  | `Internal` | `mj_makeData` 失败 | 1 |

  错误信息附带模型路径和 MuJoCo 的原始错误文本（去掉末尾空白），例如 `Error opening file 'nope.png'`。
- **实体与 ID。**
  - 每个非 world body 对应一个实体，按 body 顺序分配 `EntityId` 1..N。关节同样按顺序分配 `JointId`。
  - Terra ID 到 MuJoCo 索引的映射只保存在 `Impl` 中，不对外暴露。
  - world body 上的静态 geom（如地面）不是实体，将来作为 collider 描述（M0-06 接触摘要）。
  - 按技术设计 4.2 节，M0 不保证跨进程或重新加载后数字 ID 相同。
- **关节维度。** 自由关节 7/6、球关节 4/3、滑动与转动关节 1/1（配置量/速度量），由关节类型确定，测试中逐一核对。
- **初始观测。** `mj_makeData` 已将状态置为模型初值；随后调用 `mj_forward`，让 `xpos/xquat` 与 t=0 对应，并检查 `qpos/qvel/xpos/xquat` 均为有限值。`entityPose()` 读取实体原点位姿（body 坐标系原点，不是质心），四元数为 wxyz。它是 M0-04 用于核对初始状态的临时入口，M0-06 起由不可变快照取代。
- **路径。** 模型路径先转为绝对路径；MJCF 内的 `include` 和资源文件按模型文件所在目录解析，这是 MuJoCo 的行为，已有测试覆盖。
- **MuJoCo 日志。** MuJoCo 4.0 的默认日志会在**当前工作目录**追加写入 `MUJOCO_LOG.TXT`。第一次加载时，适配层把日志配置改为只输出到控制台（进程级设置，只设置一次）。警告和错误写 stderr，不影响 stdout 的 JSON Lines；致命错误仍以 `exit(1)` 结束进程，与 Headless 的“内部错误”退出码一致。

## 3. 开发中的发现

| 发现 | 影响与处理 |
| --- | --- |
| MuJoCo 4.0 的尺寸字段（`nbody`、`nq` 等）为 `mjtSize`（`int64_t`），对象索引仍为 `int` | 适配层显式转换；ID 超出 `uint32` 时报 `InvalidModel` |
| `mj_loadXML` 会自动识别 URDF（根元素 `<robot>`） | 不主动拦截；按 D-010，M0 只承诺 MJCF，URDF 的行为不测试、不承诺 |
| 默认日志写 `MUJOCO_LOG.TXT` | 已关闭写文件；实验确认：不经 Terra 时会写出该文件，经 Terra 加载后不再写 |
| OBJ/STL 网格解码器编译在 `libmujoco` 内；actuator/sensor/sdf 插件是单独的 `.so`，当前安装不包含 | M0 不需要；M3 引入机器人模型前需要评估插件的安装与加载 |
| 夹具预览：用官方预编译库运行 5000 步，t=5.000000，中心高度 0.099892，4 个接触点 | 只是预览，与候选容差（距 0.1 m 不超过 0.005 m）一致；正式判定属于 M0-08 |

## 4. 测试

总数从 20 项变为 22 项：

- **新增 `physics.mujoco_world`**，只通过 Terra 公共头访问后端：
  - 立方体夹具：实体、关节、7/6 维度、t=0、初始位姿、无效/越界 ID。
  - 路径：相对路径解析为绝对路径；`include` 以模型目录为基准。
  - 层级：5 个实体的父子关系、未命名 body、转动/滑动/球关节的类型与维度、ID 连续分配。
  - 非单位初始姿态：绕 X 轴 90°，用来确认 wxyz 顺序，并检查四元数已归一化。
  - 9 种加载失败：文件不存在、空路径、目录、XML 不完整、根元素非法、未知属性、资源文件缺失、include 文件缺失、步长为 0。
  - 生命周期：重复加载 50 次；失败与成功交替；5 个世界同时存在并中途释放其中一个。
  - 当前工作目录下没有 `MUJOCO_LOG.TXT`。
- **新增 `headless.invalid_mjcf`**：模型文件存在但不是合法 MJCF 时退出码为 4。
- **更新的 Headless 用例：**`dry_run_*`、`run_not_available`、`no_display_dry_run` 改用立方体夹具；`--dry-run` 输出中增加 `timestep`、`total_steps`。
- **单元测试 `headless.cli_parser`**：新增 `stepsForDuration` 的边界用例，例如 5/0.001=5000（不能得到 5001）、0.3/0.1=3、不能整除时向上取整、时长短于一步时运行 1 步、溢出、NaN。

## 5. 验证记录

### 5.1 开发机（不是基线验证）

- **依赖：** 按约定，开发机不编译 MuJoCo。自测使用官方发布页的预编译包 `mujoco-4.0.1-linux-x86_64.tar.gz`，sha256 `8df1308b…607299d` 与官方 `.sha256` 文件一致，解压在 `/tmp`，并配一个手写的 `mujocoTargets.cmake`。它与基线机器的源码构建是同一版本，但不是同一个二进制，因此**不能代替基线验证**。
- **结果：**
  - Debug + `-Werror`：22/22 通过。
  - Debug + ASan/UBSan（含 LeakSanitizer）：22/22 通过。
  - 第一次运行时 `physics.mujoco_world` 有 2 项检查失败，原因是用 `<robot>` 作根元素时被当成了 URDF，属于测试本身的错误，已改用 `<scene>`（见第 3 节）。
- **人工检查：** Headless 的 `--duration 5 --dry-run` 得到 `total_steps` 5000；非 dry-run 返回 3；资源缺失和未知属性返回 4，报错信息能定位到原因；日志文件实验见第 3 节。

- **补充（基线机器第一次运行后）：** 基线机器上 Debug 22/22 通过，但 Release + `-Werror` 编译失败：GCC 11 在 `-O3` 下对测试代码中的 `e ? world->entityPose(id) : std::nullopt` 报 `-Wmaybe-uninitialized`（误报，访问前已检查 `has_value()`）。开发机此前只跑了 Debug，漏掉了这个问题。修复方法是改为先检查指针、再直接构造 `std::optional`。在开发机上按 Release 参数复现：旧写法报 3 条，新写法报 0 条；之后 Debug 与 Release（`-Werror`）均 22/22 通过。代码中没有其他同类写法。**此后开发机自测都同时运行 Debug 和 Release。**

### 5.2 基线机器

- **日期：** 2026-11-13；机器与工具链同 [M0-02 报告](M0-02-mujoco.md) 4.2 节（GCC 11.4.0，链接源码构建的 MuJoCo 4.0.1）。
- **第一次运行：** Debug 22/22 通过；Release + `-Werror` 因测试代码触发 `-Wmaybe-uninitialized` 误报而编译失败（见 5.1 节补充）。
- **修复后再次运行：** Release + `-Werror` 构建成功，22/22 通过。
- **完整复测：** 用户再次执行 Debug 与 Release（`-Werror`）的配置、构建和测试，两种配置均构建 19 步、**22/22 通过**。

## 6. 验收标准对照（开发计划 M0-04）

| 标准 | 当前 |
| --- | --- |
| 能按名称识别立方体及其自由关节 | `physics.mujoco_world`，开发机通过 |
| 初始时间、位姿符合夹具 | 同上（t=0，(0,0,1)，单位四元数；另有非单位姿态用例） |
| 加载失败不留下可用的半成品世界 | 9 种失败都检查了 `world == nullptr`；交替加载用例 |
| 数据先于模型释放 | 由成员声明顺序保证；ASan 下没有内存问题 |
| 重复加载/释放不崩溃 | 50 次重复加载、5 个世界并存、中途释放 |
| 覆盖文件缺失、XML 错误、引用资源缺失和释放路径 | 见第 4 节 |

**验收结论：** 2026-11-19 用户评审通过。
