# M0-07 Terra Physics 统一适配接口

> 状态：**已验收**（用户于 2026-11-19 确认最终评审通过）。设计（第 1–6 节）已于 2026-11-17 评审通过；实现与测试见第 7–8 节。基线机器（2026-11-18）Debug、Release + `-Werror`、ASan/UBSan 均 **30/30 通过**，变异测试 **8/8 被发现**（9.2 节）。设计依据为开发计划 M0-07、技术设计第 3 节、D-011，以及对 MuJoCo 4.0.1 头文件和变更记录的核查（第 2 节）。

## 1. 范围

| 开发计划 M0-07 步骤 | 本设计的处理 |
| --- | --- |
| ① 评审 `loadModel/initialize/step/reset/setControl/getState/getSnapshot` 语义 | 第 3 节逐项给出结论；其中 `loadModel+initialize`、`getState+getSnapshot` 两组建议合并（3.2 节） |
| ② 加载与驱动能力留在私有后端 | 新增后端无关的 `terra_physics`（只有头文件）；`MujocoWorld` 移入后端 `src/`，不再公开（第 4 节） |
| ③ 校验生命周期、ID、控制维度、单位和范围 | `setControl` 的校验规则与错误分类（3.4 节） |
| ④ 无执行器模型接受空控制 | 空命令列表合法，立方体夹具不变 |
| ⑤ 简单单关节夹具验证非空控制 | 新增 `hinge_motor.xml`，按离散解析解判定方向和大小（第 5 节） |
| ⑥ 检查公共头、链接关系和只消费 Terra 数据的调用方 | 调用方在编译期就看不到 MuJoCo 头文件（4.2 节、第 5 节） |

不在本任务范围：第二个物理后端、运行时后端选择、多线程、Headless 的控制输入参数、执行器状态进入快照（第 6 节）。

## 2. 核查发现：MuJoCo 4.0.1 的执行器模型

以下事实来自 `third_party/mujoco/install/include/mujoco/mjmodel.h`、`mjtype.h`、`engine_forward.c` 和 `doc/changelog.rst`，直接影响接口设计：

1. **控制量数不等于执行器数。**`nu`（`mjData.ctrl` 的长度）= Σ `actuator_ctrlnum`，`nactuator` 是执行器数。已有一个执行器带多个输入：`orientation` 执行器设置 `input="quat"` 时有 4 个控制量。`actuator_ctrllimited/ctrlrange` 按**控制量**（`nu`）索引，不是按执行器索引。
   → Terra 的控制维度必须**按执行器**校验，不能假设“一个执行器一个标量”。
2. **reset 恢复的是“中性值”，不是全零。**`mj_resetData` 调用 `mj_resetCtrl`：普通输入置 0，四元数输入置单位四元数。
   → 现有 `reset()` 注释里写的“控制量清零”不准确，要改为“恢复为模型定义的中性值”。
3. **控制可以有延迟。**执行器设置 `delay`/`nsample` 后，力从历史缓冲中读取，`ctrl` 不会在下一步立即生效。
   → 这与“控制在下一步生效”的约定冲突。M0 在加载时拒绝带延迟的执行器（3.4 节）。
4. **钳位与坏值处理是静默的。**`ctrllimited` 时，MuJoCo 在计算力时把控制钳到 `ctrlrange`；任一控制量为 NaN 或 |u| > 1e10 时，**全部**控制被置 0 并发出 `mjWARN_BADCTRL` 警告（按 M0-05 策略会变成步进故障）。
   → Terra 在 `setControl` 时就拒绝越界和坏值，不依赖后端的静默钳位或清零。

## 3. 语义设计

### 3.1 生命周期

```text
           load 成功
 (无对象) ──────────▶ 就绪 ──step 出错──▶ 故障
                       ▲  │                 │
                       │  └ step / setControl / snapshot（保持就绪）
                       └──────── reset ─────┘（reset 在两种状态下都可调用）
```

- 加载失败时不产生世界对象，因此“未加载就步进”这类非法顺序在类型层面就不可能发生。
- 故障状态下，`step` 与 `setControl` 都返回 `Faulted`，状态不变；只有 `reset` 能恢复。`snapshot` 和各查询仍然可用，便于诊断。
- 销毁即释放，不需要显式的 `unload`。

### 3.2 七个建议能力的结论

| 能力（D-011） | 结论 | 理由 |
| --- | --- | --- |
| `loadModel` + `initialize` | **合并为 `load`**（保持 M0-04 的原子行为） | `initialize` 的建议输入（世界配置、Core 分配的实体绑定）在 M0 不存在，步长由模型给出。拆开会引入“已加载未初始化”的半状态，却没有使用者。等 Core/Scene 真正需要外部绑定时（M1 起）再拆，届时接口可以变化。 |
| `step` | 保持 | 语义同 M0-05。 |
| `reset` | 保持，修正说明 | 控制恢复为模型定义的中性值（第 2 节第 2 条）。 |
| `setControl` | **新增** | 见 3.4 节。 |
| `getState` + `getSnapshot` | **合并为 `snapshot()`** | 技术设计 3.2 节已允许 M0 用完整状态副本作为快照；没有消费者需要第二种读取方式。查询裁剪等有真实需求时再加。 |

### 3.3 执行器描述

`ModelDescription` 增加 `actuators`，ID 规则与实体、关节相同（`actuators[i].id.value == i + 1`）：

| 字段 | 含义 |
| --- | --- |
| `id`、`name` | Terra 执行器 ID；名称（未命名为空） |
| `transmission` | `Joint`：直接作用于一个关节（MuJoCo `mjTRN_JOINT`/`mjTRN_JOINTINPARENT`）；`Other`：肌腱、site、body、曲柄、SO3 等，M0 只描述，不解释 |
| `joint` | `transmission == Joint` 时为目标关节 ID，否则为无效 ID |
| `inputs` | 每个控制输入一项：`name`（来自 `mj_actuatorInputName`，标量输入通常为空）、`limited`、`min`、`max` |

**控制值的物理含义由模型中的执行器定义**（力、力矩、目标位置、目标速度等），Terra 不加以分类，也不提供单位标签。原因：MuJoCo 的 motor/position/velocity 只是 gain/bias 参数的快捷写法，编译后无法可靠地还原成类别；硬猜一个类别比不提供更危险。这一点在第 6 节作为待讨论项保留。

### 3.4 `setControl`

```text
setControl([{actuator: ActuatorId, values: [u0, u1, …]}, …]) → 成功 | ControlError
```

- **按执行器寻址，部分更新。**未出现在命令中的执行器保持原值。空列表合法，什么也不改变（无执行器模型的“空控制”）。
- **持久生效。**写入的值在之后每一步都生效，直到再次设置或 reset；不是“只作用一步”。
- **生效时刻。**从下一次 `step` 开始生效。不影响当前观测时刻，因此**不会使已缓存的快照失效**（快照里没有控制量，也不重算物理）。
- **原子性。**整批命令先全部校验，全部通过才写入；任一项失败则一项也不写。
- **同一步之前多次调用**：按调用顺序，后写覆盖先写（技术设计 3.1 节）。同一次调用里重复出现同一执行器则视为错误，因为“哪个算数”是含糊的。

校验规则与错误分类（按检查顺序，报告第一个错误，包含执行器 ID 和输入下标）：

| `ControlErrorKind` | 条件 |
| --- | --- |
| `Faulted` | 世界处于故障状态 |
| `UnknownActuator` | ID 无效（0）或超出范围 |
| `DuplicateActuator` | 同一次调用中重复出现同一执行器 |
| `SizeMismatch` | `values` 的长度 ≠ 该执行器的输入数 |
| `NonFinite` | 存在 NaN/Inf |
| `OutOfRange` | `limited` 输入超出 `[min, max]`（闭区间）；或任一输入 \|u\| > 后端的有效数值上限（MuJoCo 为 1e10，超出会把全部控制清零） |

越界一律**拒绝，不裁剪**。即使模型关闭了 MuJoCo 的钳位（`clampctrl` disable），只要 `ctrllimited` 为真，Terra 仍按范围拒绝。

**加载期限制（新增 `LoadErrorKind::Unsupported`）：**带延迟或历史缓冲的执行器（`actuator_delay > 0` 或 `nsample > 0`）、插件执行器在 M0 加载时直接报错，因为它们的控制不满足“下一步生效”。这比静默接受、语义却对不上要安全；M3 引入机器人时再评估。

## 4. 代码组织

### 4.1 接口（评审通过，实现见 `engine/physics/include/terra/physics/physics_world.hpp`）

```cpp
// engine/physics/include/terra/physics/physics_world.hpp —— 后端无关，只用 Terra 类型
namespace terra::physics {

enum class LoadErrorKind { FileUnavailable, InvalidModel, Unsupported, Internal };
struct LoadError { LoadErrorKind kind; std::string message; };

enum class SimulationErrorKind { NonFinite, AutoReset, SolverWarning, Faulted };
struct SimulationError { SimulationErrorKind kind; std::uint64_t step; std::string message; };

struct ActuatorCommand {
    terra::data::ActuatorId actuator;
    std::vector<double> values;  // 长度 = 该执行器的输入数
};
enum class ControlErrorKind { Faulted, UnknownActuator, DuplicateActuator, SizeMismatch, NonFinite, OutOfRange };
struct ControlError {
    ControlErrorKind kind;
    terra::data::ActuatorId actuator;  // Faulted 时无效
    int input = -1;                    // 出错的输入下标；不适用时为 -1
    std::string message;
};

class PhysicsWorld {
public:
    virtual ~PhysicsWorld() = default;

    virtual const terra::data::ModelDescription& description() const noexcept = 0;
    virtual std::optional<SimulationError> step() = 0;
    virtual void reset() = 0;
    virtual std::optional<ControlError> setControl(const std::vector<ActuatorCommand>& commands) = 0;

    virtual double simulationTime() const noexcept = 0;
    virtual std::uint64_t stepIndex() const noexcept = 0;
    virtual std::uint64_t runEpoch() const noexcept = 0;
    virtual bool faulted() const noexcept = 0;
    virtual std::shared_ptr<const terra::data::PhysicsSnapshot> snapshot() const = 0;
};

struct LoadResult {
    std::unique_ptr<PhysicsWorld> world;
    std::optional<LoadError> error;
    std::vector<std::string> warnings;
};

}  // namespace terra::physics

// engine/physics/backends/mujoco/include/terra/physics/mujoco/mujoco_backend.hpp
namespace terra::physics::mujoco {
LoadResult loadWorld(const terra::data::ModelSource& source);  // 唯一的后端入口
BackendInfo backendInfo() noexcept;                            // 现有，不变
}
```

错误继续使用返回值，不用异常，与 M0-04/05 一致。

### 4.2 目标与依赖

```text
terra_data (头文件)  ◀── terra_physics (头文件：PhysicsWorld、错误类型)
                              ▲
                              │ PUBLIC
terra_physics_mujoco ─────────┘   PRIVATE mujoco::mujoco；MujocoWorld 移入 src/
        ▲
terra_headless：只在 run() 里调用一次 mujoco::loadWorld，其余代码只用 PhysicsWorld&
```

**为什么用抽象基类：**它让“调用方只依赖 Terra 类型”可以在编译期检查。消费者库只链接 `terra_physics`，根本拿不到后端的头文件路径。虚函数的开销相对一次 `mj_step` 可以忽略。这**不等于**承诺多后端（D-107 仍是待讨论），只是把唯一的实现放在接口后面。

## 5. 测试与夹具（设计时的计划；实际内容见第 8 节）

| 新增/修改 | 内容 |
| --- | --- |
| `assets/tests/hinge_motor.xml` | 一根杆绕 +Y 铰链转动，**无重力、无阻尼、无接触**；一个 `motor`（gear 1，ctrlrange [-1, 1]）；一个两输入的执行器用于维度校验（如条件允许用 `orientation`+`quat` 作用在另一个球关节上，否则在报告中说明替代方案） |
| `physics.control`（新） | ① 立方体：空列表成功，轨迹与不设置控制逐位相同；② 铰链：u = +0.5 时，离散解析解 θₖ = a·dt²·k(k+1)/2（a = u/I），误差 ≤ 1e-9，u = −0.5 时方向相反，u = 0 时保持静止；③ 持久性：设置一次，多步都生效；④ reset 后恢复为中性值；⑤ 原子性：一批里有一项非法时，其他项也没有写入；⑥ 六种错误各至少一个用例；⑦ 设置控制不会使当前快照失效 |
| `physics.consumer`（新） | 一个只链接 `terra::physics` 的静态库，提供“跑 N 步并汇总快照”的函数；测试程序再链接后端，把世界传进去。消费者库能编译，就证明它不需要 MuJoCo 头文件 |
| `physics.mujoco_load_unsupported`（新） | 带 `delay` 的执行器加载时返回 `Unsupported` |
| 现有测试 | 改为通过 `loadWorld` 和 `PhysicsWorld` 访问；断言不变。边界检查已覆盖 `engine/` 下所有 `include/`，新增的 `terra_physics` 自动纳入 |
| Headless | `model` 记录增加 `actuators` 字段（只增加字段、属于兼容修改，`contract_revision` 保持 1）；不增加控制输入参数 |

已核实：`orientation` 执行器写法为 `<orientation joint="…" input="quat" kp="…" kv="…"/>`，4 个输入名为 qw/qx/qy/qz，传动类型为 SO3（归为 `Other`）。延迟判定使用 `actuator_delay > 0` 或 `actuator_history[2i] > 0`（nsample），已在头文件中确认。

## 6. 评审结论（2026-11-17，第 1–5 条全部同意）

1. **合并 `load`/`initialize` 与 `snapshot`/`getState`**（3.2 节）——建议合并，同时更新 D-011。
2. **抽象基类 + 单一后端入口**（4.2 节）——建议采用；另一种做法是保留具体类 `MujocoWorld` 并改名，但这样就失去了编译期边界检查。
3. **执行器语义类别**：M0 不提供（3.3 节）。M3 机器人驱动时，可能需要由模型资产层显式声明类别，而不是从后端参数推断。
4. **执行器状态进入快照**（施加的控制量、执行器力）：M0 不做。原因是“控制量对应哪个时刻”需要单独约定：快照第 k 步的状态由第 k−1 步的控制产生，而 `setControl` 写入的是下一步的控制。留到 M3 与记录/复现（F-M3-03）一起设计。
5. **拒绝带延迟的执行器**（3.4 节）——M0 加载时拒绝，M3 再评估。

## 7. 实现

| 部分 | 位置 | 说明 |
| --- | --- | --- |
| 数据契约 | `engine/core/data/include/terra/data/ids.hpp`、`model_description.hpp` | 新增 `ActuatorId`、`ActuatorTransmission`、`ControlInput`、`ActuatorDescription`；`ModelDescription::actuators` 及 `actuator(id)`/`findActuator(name)` |
| 后端无关接口 | `engine/physics/include/terra/physics/physics_world.hpp`，目标 `terra_physics`（INTERFACE） | `PhysicsWorld`、`LoadResult`、三类错误（`LoadError` 新增 `Unsupported`；`SimulationError`；`ControlError`）、`ActuatorCommand` |
| MuJoCo 后端入口 | `engine/physics/backends/mujoco/include/terra/physics/mujoco/mujoco_backend.hpp` | `loadWorld()`；原公共头 `mujoco_world.hpp` 删除 |
| MuJoCo 实现 | `engine/physics/backends/mujoco/src/mujoco_world.{hpp,cpp}` | `MujocoWorld final : PhysicsWorld`，头文件改为私有；加载时生成执行器描述（按 `actuator_ctrladr + k` 读取每个输入的范围与名称），拒绝带延迟/历史缓冲和插件执行器；`setControl` 先全部校验再写入 `ctrl`，不使快照失效；`reset` 注释改为“恢复中性值” |
| Headless | `apps/headless/main.cpp`、`cli.{hpp,cpp}`、`CMakeLists.txt` | 只在 `run()` 中调用一次 `mujoco::loadWorld`，其余经 `PhysicsWorld&`；`model` 记录增加 `actuators`（`id/name/transmission/joint/inputs[{name,limited,range}]`，`range` 仅在 limited 时输出），`contract_revision` 仍为 1 |
| 边界检查 | `cmake/TerraCheckNoMuJoCoInPublicHeaders.cmake` | 扫描范围增加 `tests/physics/consumer/` |

## 8. 测试

总数从 28 项变为 30 项。

**夹具 `assets/tests/hinge_motor.xml`：**无重力、无接触。`arm`：铰链 `shoulder` 绕 +Y，I = 0.1 kg·m²；`drive`：motor，范围 [-1, 1]。`head`：球关节 `neck`；`aim`：orientation，四元数输入（4 个），范围 [-1, 1]。`trim`：第二个作用于 shoulder 的 motor，非对称范围 [-0.2, 0.3]，排在 `aim` 之后，控制地址 5 ≠ 执行器下标 2，用于发现“按执行器下标索引控制量”的错误（设计时未计划，变异测试准备阶段补充）。

**新增 `physics.control`（`tests/physics/test_physics_control.cpp`）：**

| 用例 | 内容 |
| --- | --- |
| 描述 | 3 个执行器；drive/trim 作用于关节 1，范围正确；aim 为 `Other`、4 个输入 qw..qz；无效 ID 查不到；立方体无执行器 |
| 空控制 | 立方体每步调用空控制 1000 步，与从不调用逐位相同 |
| 铰链响应 | u = +0.5 / −0.5 / 0，只设置一次：1000 步内角度和角速度与离散解析解误差 ≤ 1e-9；方向正确；实体角速度 = (0, ω, 0)；球关节不受影响 |
| 多输入与部分更新 | aim 设为绕 +Z 90°，头部只绕 Z 转动；只更新 aim 时 drive 保持；trim 与 drive 叠加，且不改变 aim |
| 后写覆盖 | 同一步之前两次设置，取后一次 |
| reset 中性值 | 设置后 reset，再走 500 步：铰链静止，球关节保持单位四元数（证明 aim 恢复为单位四元数而非全零） |
| 快照 | 设置控制后快照是同一对象、步序号不变；下一步后才是新快照 |
| 校验 | 六类错误各有用例，并检查错误中的执行器 ID 与输入下标；±1 与 trim 非对称边界合法；trim 按自身范围判定；批量中一项非法时整批不写入；不限范围执行器 1e6 合法、2e10 拒绝 |
| 故障 | 故障状态下 `setControl` 返回 `Faulted`；reset 后恢复 |
| 不支持的执行器 | `delay`+`nsample` 的执行器加载返回 `Unsupported`，消息含执行器名与原因 |

**新增 `physics.consumer`：**调用方库 `terra_test_physics_consumer`（`tests/physics/consumer/`）只链接仅头文件的 `terra::physics`，构建时没有 MuJoCo 头文件路径。它按名称构造控制、运行并汇总快照；测试程序只在创建世界时调用后端入口。用例：立方体空控制 2000 步后停在地面；铰链按名称设置 drive = 0.5，1000 步后角度与解析解一致；未知名称、越界控制经调用方返回错误且不推进。

**修改：**`physics.mujoco_world/step/state` 改为通过 `loadWorld` 与 `PhysicsWorld` 访问，断言不变；`headless.cli_parser` 的 model 记录逐字节比对增加 actuators（含多输入、不限范围、`other` 传动）。

### 8.1 变异测试

逐个向 `mujoco_world.cpp` 注入错误，`physics.control` 必须失败：

| 变异 | 开发机结果 |
| --- | --- |
| ctrl 按执行器下标写入（而不是 `actuator_ctrladr`） | 发现（trim 叠加用例） |
| 输入范围按执行器下标读取 | 发现（trim 范围用例） |
| 不检查范围，依赖后端钳位 | 发现（5 项） |
| 非原子：逐条校验逐条写入 | 发现（原子性用例） |
| 不检查同一次调用中的重复 | 发现 |
| 设置控制时清空快照缓存 | 发现 |
| 不拒绝带延迟的执行器 | 发现 |
| 故障状态下允许设置控制 | 发现 |

## 9. 验证记录

### 9.1 开发机（不是基线验证）

- **日期：**2026-11-17。
- **环境：**开发机 Ubuntu 22.04，GCC 11.4.0；CMake 4.4.4（临时安装），因开发机没有 Ninja 改用 Unix Makefiles；链接仓库内 `third_party/mujoco/install` 的 MuJoCo 4.0.1。
- **结果：**Debug + `-Werror`：改动前 28/28；改动后 **30/30 通过**；变异测试 **8/8 被发现**，源文件已恢复并逐字节校验。
- **未执行：**Release + `-Werror`、ASan/UBSan（开发机不再承担构建与测试，2026-11-18 起改由基线机器执行）。

### 9.2 基线机器

- **日期：**2026-11-18。
- **环境：**Ubuntu 22.04（内核 6.8.0-106-generic），x86-64；GCC 11.4.0；CMake 3.22.1；Ninja 1.10.1；源码构建的 MuJoCo 4.0.1。
- **命令：**`bash scripts/verify/m0_07.sh`。脚本删除 `build/debug`、`build/release` 后依次运行 Debug、Release + `-Werror`、ASan/UBSan、8 项变异测试，并输出 `hinge_motor.xml` 的 model 记录与第 1000 步状态；日志在 `build/verify-m0-07/logs/`。
- **结果：**

| 项目 | 结果 |
| --- | --- |
| Debug | **30/30 通过** |
| Release + `-Werror` | **30/30 通过**，无警告 |
| ASan/UBSan（含 LeakSanitizer） | **30/30 通过** |
| 变异测试 | **8/8 被发现**（失败检查数：下标写入 2、下标读范围 2、不检查范围 5、非原子 3、不检查重复 1、快照失效 1、不拒绝延迟 2、故障时允许 1）；源文件已恢复 |
| `hinge_motor` Headless | 退出码 0 |

  `hinge_motor.xml` 的 actuators 记录：drive（joint 1，[-1, 1]）、aim（other，4 个输入 qw/qx/qy/qz，[-1, 1]）、trim（joint 1，[-0.2, 0.3]），与 8 节描述一致。不设置控制时第 1000 步铰链角度与速度为 0，球关节保持 [1, 0, 0, 0]、角速度为 0，说明中性控制不驱动任何关节。

## 10. 验收标准对照（开发计划 M0-07）

| 标准 | 当前（基线机器已验证） |
| --- | --- |
| 上层仅依赖 Terra 类型 | Headless 与调用方库只用 `terra::physics`/`terra::data`；边界检查覆盖公共头、apps 与调用方 |
| 非法顺序调用明确失败 | 加载失败不产生对象；故障状态下 step/setControl 返回 `Faulted` |
| 未知 ID、NaN/Inf、错误控制维度明确失败 | `UnknownActuator`、`NonFinite`、`SizeMismatch`，另有 `DuplicateActuator`、`OutOfRange` |
| 合法控制改变参考关节响应 | 铰链方向与大小符合离散解析解（≤ 1e-9） |
| 无执行器立方体正常运行 | 空控制与不调用逐位相同 |
| 快照消费者不需要 MuJoCo 头文件 | `physics.consumer` 的调用方库在没有 MuJoCo 头文件路径的情况下编译 |
