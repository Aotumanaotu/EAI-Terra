# M2-01 场景与映射设计（设计稿）

> 状态：**评审通过**（用户 2026-11-23，第 10 节 7 项全部按建议同意）。依据：开发计划第 9 节，其中 9.5 节是 2026-11-23 的评审结果；决策 D-102（场景格式）、D-103（实体与标识）、D-105（模型与渲染资源）、D-106（线程）的初版。本稿还覆盖 M2-02～M2-08 需要提前约定的接口，各任务实现时只补细节，不改方向。未编写代码。

## 1. 目标

M2 结束时，下面这条链路要能跑通：

```
场景文件（JSON）
  → terra_scene：校验、组合请求、实例与映射
  → MuJoCo 后端：mjSpec 组合、编译 → PhysicsWorld
  → terra_simulation：固定步长时钟、命令队列、快照历史与插值
  → terra_scene：视觉部件的世界位姿（纯数学）
  → 查看器：glTF 节点与 MJCF 几何 → 渲染项 → terra_render
```

Headless 走同一条链路的前三步，不读取任何视觉资源。

## 2. 模块与依赖

| 模块 | 职责 | 依赖 | 不允许依赖 |
| --- | --- | --- | --- |
| `terra_data`（扩展） | 新增模型实例、几何体、网格的描述类型；新增 `ModelInstanceId`、`GeomId`、`VisualId` | 标准库 | 任何第三方库 |
| `terra_physics`（扩展） | 新增 Terra 类型的“世界组合请求”`WorldComposition` 与按实例归属的错误 | `terra_data` | MuJoCo、GL |
| MuJoCo 后端（扩展） | `composeWorld(请求)`：逐个解析 MJCF，用 `mjSpec` + `mjs_attach` 组合并编译；填写几何体描述；登记日志处理函数（第 8 节） | MuJoCo | GL |
| **`terra_scene`**（新） | 读取并校验场景文件；分配实例前缀；决定全局物理配置；生成组合请求并调用世界工厂；把编译错误归属到实例；建立实体与视觉部件映射；计算视觉部件世界位姿 | `terra_data`、`terra_physics`（只用接口）、nlohmann/json（只在实现中） | MuJoCo、GL、`terra_render`、`terra_asset` |
| **`terra_simulation`**（新） | 固定步长时钟、暂停与单步、按步生效的命令队列、快照历史与插值、reset 屏障 | `terra_data`、`terra_physics`（只用接口） | MuJoCo、GL |
| `terra_asset`（扩展） | 保留 glTF 节点层级；按节点取子树 | 不变 | 不变 |
| `terra_viewer`（扩展） | 打开场景；用 MuJoCo 工厂创建世界；把视觉位姿转成渲染项；仿真控制界面 | 以上全部 + MuJoCo 后端 | — |
| `terra_headless`（扩展） | `--scene` 运行场景文件（只跑物理） | `terra_scene`、`terra_simulation`、MuJoCo 后端 | GL |

`terra_scene` 只通过注入的世界工厂创建世界：

```cpp
using WorldFactory = std::function<physics::ComposeResult(const physics::WorldComposition&)>;
```

由应用传入 `terra::physics::mujoco::composeWorld`。这样，按评审第 2 条的要求，由 `terra_scene` 统一负责创建世界、管理实例前缀、处理全局配置和编译错误，同时自身不含 MuJoCo 类型（NFR-01）。

边界检查扩展：
- `boundary.render_isolation` 增加规则：`terra_scene`、`terra_simulation` 不得包含 GL、GLFW、ImGui、MuJoCo 头文件；`terra_render`、`terra_ui`、`terra_asset` 不得包含物理头文件。
- 新增 ldd 检查：`terra_headless` 运行场景时仍不链接图形库。

## 3. 场景文件格式（D-102）

### 3.1 示例

```json
{
  "format": "terra.scene",
  "version": 1,
  "name": "两辆小车",
  "physics": { "timestep": 0.002, "gravity": [0, 0, -9.81], "integrator": "implicitfast" },
  "models": [
    { "name": "ground", "mjcf": "models/ground.xml" },
    { "name": "cart_a", "mjcf": "models/cart.xml", "position": [0, -0.6, 0.1] },
    { "name": "cart_b", "mjcf": "models/cart.xml", "position": [0, 0.6, 0.1], "rotation": { "rpyDeg": [0, 0, 90] } }
  ],
  "visuals": [
    { "entity": "cart_a/chassis", "gltf": "assets/cart.glb", "node": "Chassis" },
    { "entity": "cart_a/wheel_left", "gltf": "assets/cart.glb", "node": "Wheel_L",
      "position": [0, 0, 0], "rotation": { "quat": [1, 0, 0, 0] }, "scale": 1.0 }
  ],
  "view": { "environment": "outdoor", "camera": { "position": [-3, 0, 1.2], "target": [0, 0, 0.2] } }
}
```

### 3.2 字段

| 字段 | 必需 | 含义 |
| --- | --- | --- |
| `format`、`version` | 是 | 固定为 `"terra.scene"`。版本 1。读到更高版本时报错，不猜测其含义 |
| `name` | 否 | 显示名 |
| `physics` | 否 | 全局物理配置：`timestep`、`gravity`、`integrator`、`solver`、`iterations`、`tolerance`。规则见 3.4 节 |
| `models[]` | 是，至少 1 个 | 模型实例：`name` 为实例名；`mjcf` 为路径；`position`（米）、`rotation` 为放置位姿，默认原点、无旋转 |
| `visuals[]` | 否 | glTF 绑定（第 6 节） |
| `view` | 否 | 环境贴图（沿用查看器设置中的名称）与初始相机。只影响显示，Headless 忽略 |

**通用规则：**
- 路径相对于场景文件所在目录，也可以写绝对路径。
- 旋转有两种写法：`{"quat": [w, x, y, z]}`；或 `{"rpyDeg": [滚转, 俯仰, 偏航]}`，即依次绕世界 X、Y、Z 轴旋转（R = Rz·Ry·Rx），单位为度。
- 坐标与单位沿用 D-012：Z 向上，米。
- **未知字段一律报错**（问题 2），以发现拼写错误。错误位置用 JSON 指针给出，例如 `/models/2/positon`。

### 3.3 实例名与前缀

- 实例名只允许 `[A-Za-z_][A-Za-z0-9_-]*`，且在同一场景中唯一。
- 组合时，模型中所有有名字的元素都加前缀 `实例名/`，例如 `cart_a/chassis`、`cart_a/drive_motor`。没有名字的元素保持无名。
- Terra 的实体名、关节名、执行器名、几何体名都是带前缀的全名。场景文件与日志中都用全名。
- 风险：MuJoCo 是否接受带 `/` 的名字，以及模型内部对名字的引用经前缀后是否仍然正确，在 M2-04 用夹具验证。不行时改用 `实例名:`，这只是内部前缀写法，场景文件格式不变。

### 3.4 全局物理配置

全局配置由 `terra_scene` 决定。规则：
- 场景文件写了的字段，以场景文件为准。
- 没写的字段，取第一个模型实例的值。
- 其他实例的同一字段与最终值不同时，给出**警告**，列出实例名、字段、它的值和采用的值。

后端先把每个子模型的这些字段改成最终值，再组合，以避开 MuJoCo 的属性冲突规则（`compiler/conflict`）。这样结果只取决于 Terra 的规则。各模型自己的编译设置（角度单位等）由 MuJoCo 按子模型分别处理，Terra 不干预。

### 3.5 组合中需要验证的行为（M2-04）

- 模型的世界体中的静态几何体（例如地面）会随实例复制。同一个带地面的模型放两次，就会有两块地面。示例因此把地面单独做成一个模型。Terra 不自动去重。
- 模型中的关键帧（keyframe）M2 不使用，初始状态为模型初值加上放置位姿。
- 单实例场景的轨迹必须与直接加载该 MJCF 逐位相同。如果因名字前缀或组合顺序做不到，记录原因并评审。

## 4. 标识与映射（D-103、D-105）

| 标识 | 含义 | 分配 | 有效范围 |
| --- | --- | --- | --- |
| `worldInstance` | 一次成功组合 | 已有：进程内从 1 递增 | 进程内 |
| `ModelInstanceId` | 场景中的一个模型实例 | 按场景文件顺序，从 1 开始 | 同一 `worldInstance` |
| `EntityId` | 一个 body（不含世界体） | 已有：按编译后的顺序 | 同上 |
| `GeomId` | 一个几何体（含静态几何） | 按编译后的顺序 | 同上 |
| `VisualId` | 一个视觉部件：一个 MJCF 可见几何体，或一个 glTF 绑定 | 先排几何体部件（按 `GeomId`），再排 glTF 绑定（按场景文件顺序） | 同上 |

**持久化只用名字**（实例名、实体全名、glTF 路径与节点名），不保存数字 ID。不用 ECS 库：场景实例就是几张按 ID 下标的表，关系显式记录：

- 实体 → 模型实例、父实体
- 几何体 → 实体（无效 ID 表示静态几何）、模型实例
- 视觉部件 → 实体（无效 ID 表示静态）、来源（几何体或 glTF 绑定）

一个实体可以有任意多个几何体与视觉部件，这满足 F-M2-01 的“多视觉部件正确关联”。

## 5. 几何体描述（M2-03，模型描述的扩展）

`ModelDescription` 新增以下内容，只增加字段，已有字段不变：

```cpp
struct ModelInstanceDescription { ModelInstanceId id; std::string name; std::string sourcePath; Pose placement; };
enum class GeomShape { Plane, Sphere, Capsule, Ellipsoid, Cylinder, Box, Mesh, Other };
struct GeomDescription {
    GeomId id; std::string name; EntityId entity; ModelInstanceId modelInstance;
    GeomShape shape; Vec3 size;      // 含义按形状，见下表
    Pose local;                      // 相对实体原点（body 坐标系，不是质心）
    std::array<float, 4> rgba; int group; bool collides;   // collides：contype 或 conaffinity 非 0
    int mesh = -1;                   // Mesh 形状时为 meshes 下标
};
struct MeshDescription { std::string name; std::vector<Vec3> vertices; std::vector<std::array<std::uint32_t, 3>> triangles; };  // P1
// ModelDescription：modelInstances、geoms、meshes；EntityDescription 增加 modelInstance
```

| 形状 | `size` 的含义（米） |
| --- | --- |
| Plane | x、y 方向半边长（0 表示无限大），z 为网格线间距 |
| Sphere | x = 半径 |
| Capsule、Cylinder | x = 半径，y = 半长（沿局部 Z 轴） |
| Ellipsoid | 三个半轴 |
| Box | 三个半边长 |
| Mesh | 网格已含尺寸；size 不使用 |

**显示规则（评审第 4 条）：**
- 默认显示 group 0、1、2 的几何体，与 MuJoCo 自带查看器的约定相同（问题 5）。查看器可按组开关，也可以叠加显示碰撞几何（`collides` 为真）。
- 绑定了 glTF 的实体默认隐藏自己的 MJCF 几何体，可用 `"showGeoms": true` 保留。
- hfield、sdf 等其他形状记为 `Other`，不显示并给出一次提示。

快照格式不变：几何体随实体移动，位姿由实体位姿与局部位姿算出。

## 6. glTF 绑定：MJCF 实体 → glTF 节点 → 局部变换（评审第 4 条）

`visuals[]` 每项：

| 字段 | 含义 |
| --- | --- |
| `entity` | 实体全名。也可以写 `"world"`，表示不随物理运动的静态装饰 |
| `gltf` | glTF 或 GLB 文件路径 |
| `node` | 可选。节点名，表示取该节点及其子树；省略时取整个文件。节点名在文件中重复时报错，并提示改用 `"#下标"` 写法 |
| `position`、`rotation`、`scale` | 局部变换 `T_实体_视觉`，默认为单位变换 |
| `frame` | `"node"`（默认）或 `"assembly"`，见下文 |
| `showGeoms` | 是否同时显示该实体的 MJCF 几何体，默认否 |

**两种对齐方式**（问题 1）：
- `"frame": "node"`：去掉该节点在 glTF 中的世界变换，以节点自身坐标系为准，再应用局部变换。适合“每个零件单独建模、原点就在关节处”的资产。
- `"frame": "assembly"`：保留节点在 glTF 中的整体装配位置。加载时用模型的初始姿态自动算出 `T_实体_视觉 = T_世界_实体(初始)⁻¹ × T_glTF_节点世界`，此时局部变换写在装配之上。适合“整机在 Blender 中按装配姿态导出、各零件分别绑到连杆”的资产，这也是 M3 机器人最常见的情况。

两种方式都先做 glTF 的 Y 向上 → Z 向上换算（沿用 M1-09），再应用 `scale`。

**职责分工：**
- `terra_scene` 只记录绑定，并检查文件存在。它不解析 glTF，所以 Headless 不需要 GL 与 `terra_asset`。
- 节点是否存在、网格与材质加载，在查看器的场景适配层（M2-07）中检查，诊断同样带场景文件与 JSON 指针。

**`terra_asset` 的改动：**
- `Asset` 增加 `nodes`（名字、父节点、局部矩阵、网格），实例记录所属节点下标。
- 新增“取某节点子树的部件及其相对节点的变换”。
- 现有的整文件接口行为不变，M1 的参考图不受影响。

**每帧的位姿：**

```
T_世界_视觉 = T_世界_实体（插值后）× T_实体_视觉
```

这由 `terra_scene` 用双精度计算，输出 `VisualPose{VisualId, Pose, scale}`，渲染侧再转成单精度矩阵。

## 7. 时钟、命令与插值（M2-06，D-106：单线程，以后可迁移）

- **时钟：**每帧取墙钟增量 × 速度倍率，累加到“待推进时间”，按固定步长推进整数步。单帧最多推进 0.1 秒仿真时间；超出部分丢弃，此时仿真比墙钟慢，并计入“过载次数”，显示在状态栏。**物理步永远不跳过。**
- **暂停与单步：**暂停时累加器清零、物理时间不变；单步推进恰好一步。
- **命令：**命令按“在第几步之前生效”排队（`stepIndex` 加控制命令），推进到该步之前调用 `setControl`。这样控制序列与帧率无关，30/60/144/240 Hz 下的轨迹逐位相同。
- **快照与插值：**保留最近两份快照，显示时间落后一个物理步。`alpha` 取累加器余量 / 步长。位置线性插值，四元数取最短弧后归一化插值。两份快照的 `worldInstance` 或 `runEpoch` 不同时不插值，直接用最新一份。
- **以后迁移到线程：**时钟与命令队列只通过 `PhysicsWorld` 接口与不可变快照工作。快照用 `shared_ptr<const PhysicsSnapshot>` 传递，读者只持有快照。把推进循环搬到物理线程时，只需要把“最近两份快照”的交换改为加锁，不改公共接口。

## 8. 加载失败报告与用户数据保护（M2-02，评审第 6 条：有限保证）

**加载期（保证不退出）：**
- 场景文件错误由 `terra_scene` 直接报告。
- MJCF 解析与编译错误：`mj_parseXML` 与 `mj_compile` 失败时返回错误文字（`mjs_getError`），不会走致命错误路径。后端按实例返回，`terra_scene` 加上场景文件与 JSON 指针。
- 查看器收到错误后显示在日志面板与弹窗中，保留上一个场景（或空场景）继续运行。

**运行期（不保证继续运行）：**
- 后端用 `mju_setLogHandler` 登记处理函数。警告转入 Terra 的诊断，不再直接打印。
- 致命错误时依次：
  1. 写出诊断文件，内容为错误文字、场景文件、实例、仿真时刻与步序号，位置为 `$XDG_STATE_HOME/terra/crash-<时间>.txt`，缺省目录为 `~/.local/state/terra/`；
  2. 调用已登记的紧急保存函数（设置、布局，以后还有场景编辑）；
  3. 以退出码 **70** 结束进程。MuJoCo 要求这个处理函数不返回，Terra 不做跳转恢复。
- 紧急保存的登记放在 `terra_core`（`registerEmergencySave`）。要求：不调用 MuJoCo、不调用 GL、只写文件，每个函数最多等待 1 秒。
- 设置仍每 1 秒自动保存（M1-14 已有）。紧急保存只覆盖这最后 1 秒内的改动。

## 9. 测试计划（概要）

| 任务 | 主要测试 |
| --- | --- |
| M2-02 | 加载错误夹具（MJCF 语法错误、引用缺失、场景文件各类错误）：错误文字、实例与位置正确，进程不退出；运行期致命错误夹具（子进程运行）：退出码 70，诊断文件存在，退出前修改的设置已写入 |
| M2-03 | 每种形状的尺寸与局部位姿与 MJCF 数值一致；group、颜色、碰撞标志；静态几何属于世界 |
| M2-04 | 两个相同实例：名字前缀正确，分别控制互不影响；单实例与直接加载逐位相同；全局配置冲突的警告；编译错误归属到实例 |
| M2-05 | 场景文件解析与校验（每个字段的正反例、未知字段、版本）；映射表；视觉部件世界位姿（含偏移、旋转、缩放、assembly 对齐）的数值测试 |
| M2-06 | 时钟（追赶上限、过载计数、暂停、单步）；命令按步生效；四种帧率下轨迹逐位相同；跨 reset 不插值 |
| M2-07 | glTF 节点子树与两种对齐方式；节点不存在、名字重复的诊断；固定仿真时刻的场景参考图（llvmpipe） |
| M2-08 | 查看器自检：加载测试场景、运行固定步数，位姿与 Headless 的结果一致；暂停后物理时间不变；可交互展示 |

所有新模块都纳入 ASan 与变异测试，沿用 M1 的做法。

## 10. 需要评审的问题

| # | 问题 | 建议 | 其他选项 |
| --- | --- | --- | --- |
| 1 | glTF 节点绑定的对齐方式 | 同时支持 `node`（节点自身坐标，默认）与 `assembly`（按装配姿态自动对齐） | 只支持一种：只有 `node` 时，整机装配导出的资产需要手工算每个零件的偏移；只有 `assembly` 时，单独建模的零件反而不方便 |
| 2 | 场景文件中的未知字段 | 报错（指出 JSON 指针） | 警告后忽略（更宽松，但拼写错误不易发现） |
| 3 | 名字前缀写法 | `实例名/原名`；MuJoCo 不接受时改用 `实例名:原名` | `实例名_原名`（容易与原名中的下划线混淆） |
| 4 | 全局物理配置 | 场景文件优先，其次第一个实例；不一致时警告 | 不一致时报错（更严格，但组合现成模型时经常需要手工统一） |
| 5 | 默认显示的几何体组 | 0、1、2（与 MuJoCo 查看器一致），可在视图菜单开关 | 全部显示（碰撞几何与可见几何会重叠） |
| 6 | 运行期致命错误的诊断位置与退出码 | `~/.local/state/terra/crash-<时间>.txt`，退出码 70 | 只打印到终端（用户从桌面启动时看不到） |
| 7 | 场景文件中的显示设置 | 允许可选的 `view`（环境贴图、初始相机），Headless 忽略 | 显示设置只放在查看器设置里（场景无法自带推荐视角） |
