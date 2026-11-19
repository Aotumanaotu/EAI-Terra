# M0-06 位置、姿态与关节状态读取

> 状态：**已验收**（用户于 2026-11-19 确认评审通过）。状态数据契约、不可变快照、Headless 正式状态格式与测试均已完成。开发机上（官方预编译 MuJoCo 4.0.1）Debug、Release + `-Werror`、ASan/UBSan 均 28/28 通过，3 项变异测试都能被测试发现；基线机器（源码构建 MuJoCo 4.0.1）Debug、Release + `-Werror` 均 **28/28 通过**，立方体基准实测值与开发机一致（6.2 节）。

## 1. 交付内容

| 部分 | 位置 | 说明 |
| --- | --- | --- |
| 状态数据契约 | `engine/core/data/include/terra/data/state.hpp` | `EntityState`（位姿、原点线速度、角速度）、`JointState`（按类型的配置量/速度量）、`ContactPair`/`ContactSummary`、`Diagnostics`、`PhysicsSnapshot`（修订号 1） |
| `MujocoWorld::snapshot()` | `engine/physics/backends/mujoco/` | 返回 `shared_ptr<const PhysicsSnapshot>`；按需构建并缓存，`step`/`reset` 时失效；取代 M0-04 的临时接口 `entityPose()` |
| 正式输出格式 | `apps/headless/cli.{hpp,cpp}` | `state` 记录包含速度、关节、接触、诊断；`model` 记录增加 `contract_revision` 与 `conventions` |

## 2. 约定（D-012 由此验证）

| 项目 | 约定 | 验证 |
| --- | --- | --- |
| 单位 | SI：m、s、rad、N；双精度 | 自由落体速度 = −g·t；静止时法向力 = m·g |
| 世界坐标 | 右手系，Z 向上 | 重力沿 −Z；铰链绕 +Y 的角速度符号 |
| 四元数 | w, x, y, z；局部到父坐标 | M0-04 绕 X 轴 90° 用例；倾斜立方体坐标变换用例 |
| 实体速度 | 实体原点（body 坐标系原点，**不是质心**），世界坐标 | 摆的用例：枢轴原点线速度为 0 |
| 自由关节 | 配置 `[x,y,z, qw,qx,qy,qz]`，世界坐标；速度 `[vx,vy,vz]` 世界坐标，`[wx,wy,wz]` **body 局部坐标** | 局部角速度旋转到世界后与实体角速度一致（误差 ≤ 1e-9） |
| 球关节 | 配置为相对父坐标的四元数；速度为子 body 局部坐标中的角速度 | 只按 MuJoCo 文档约定转换，M0 尚无专门的数值用例 |
| 铰链/滑动 | 角度 rad / 位移 m；对应速度 | 摆的用例 |
| 观测时刻 | 快照中的时间、位姿、速度、接触都对应同一时刻（步后已 `mj_forward`） | 位置差分与速度一致；第 1 步 z = 1 − g·dt² |

X 向前、Y 向左属于建模约定，需要在模型资产层面保证，M0 不做强制检查。

## 3. 关键实现

### 3.1 速度参考点

`mj_objectVelocity` 用 `mjOBJ_BODY` 时，参考点是**质心**（`xipos`）；用 `mjOBJ_XBODY` 时才是 body 坐标系原点（`xpos`）。这一点只能从源码 `engine_core_util.c` 中确认，按名字很容易用错。Terra 使用 `mjOBJ_XBODY`，世界坐标朝向，返回 `[角速度; 线速度]`。

变异测试：改用 `mjOBJ_BODY` 后，摆的枢轴报出 1.89 m/s 的线速度（正确值为 0），测试失败。

### 3.2 有效接触

接触点必须同时满足以下条件才计入：

1. `exclude == 0`：不在 gap 内、未被合并或排除；
2. `efc_address >= 0`：参与约束求解；
3. `mj_contactForce` 给出的法向力 > 0。

接触点按 geom 所属 body 映射到实体对，`entityA <= entityB`，world 为 0；每一对汇总点数和法向力之和（N）。`candidateCount` 单独记录碰撞检测得到的全部候选点。flex 接触（geom 为 −1）在 M0 中不计入。

法向力是按当前观测时刻的状态重新计算得到的量（技术设计 2.3 节），不是刚完成那一步积分时的冲量。

变异测试：只看候选、不检查以上条件时，gap 用例失败。

### 3.3 快照的不可变性与缓存

- 快照是 `shared_ptr<const>` 持有的独立副本，不含任何后端指针，关节量用定长数组保存。
- 同一观测时刻重复调用返回同一对象。
- `step`/`reset` 时清空缓存，下次请求重新构建；已经取得的旧快照不受影响。
- 构建快照只读取已经算好的量（`mj_objectVelocity` 读 `cvel`，`mj_contactForce` 读 `efc_force`），不推进、也不重算物理。

变异测试：`step` 时不清空缓存，5 项检查失败。

### 3.4 诊断

- `finite`：位姿、速度、关节量、接触力全部为有限值。
- `solverIterations`：各约束岛中求解迭代次数的最大值。
- 致命问题由 M0-05 的故障机制处理，不会进入快照。

## 4. 输出格式（contract_revision 1）

```text
{"type":"model","contract_revision":1,
 "conventions":{"units":"SI","up_axis":"+z","quaternion":"wxyz","entity_velocity":"entity_origin_world",
                "free_joint_angular_velocity":"body_local","ball_joint_angular_velocity":"child_local",
                "contact_entity_0":"world_static"},
 "timestep":…,"gravity":[…],"integrator":…,"solver":…,"iterations":…,"tolerance":…,
 "entities":[{"id","name","parent"}],"joints":[{"id","name","type","entity","nq","nv"}]}

{"type":"state","run_epoch":…,"step":…,"time":…,
 "entities":[{"id","position":[x,y,z],"orientation":[w,x,y,z],"linear_velocity":[…],"angular_velocity":[…]}],
 "joints":[{"id","position":[…nq…],"velocity":[…nv…]}],
 "contacts":{"candidates":…,"active":…,"pairs":[{"entities":[a,b],"points":…,"normal_force":…}]},
 "diagnostics":{"finite":…,"solver_iterations":…}}
```

`config`、`summary`、`error` 记录的格式与 M0-05 相同。格式若有不兼容的修改，`contract_revision` 加 1。

## 5. 测试

总数从 27 项变为 28 项。

**新增 `physics.mujoco_state`：**

| 用例 | 内容 |
| --- | --- |
| 初始快照 | 代次、步序号、时间、步长；零速度；自由关节为 7/6 维且配置 = 位置 + 四元数；没有接触；全部有限 |
| 不可变性 | 同一时刻返回同一对象；step 后返回新对象；第 0 步和第 1 步的旧快照在 500 步和一次 reset 之后不变 |
| 自由落体速度 | 300 步内 v = −g·k·dt（误差 ≤ 1e-12）；位置差分与速度一致（≤ 1e-9）；无角速度；关节与实体一致 |
| 立方体主基准（计划 4.2 节的速度与接触部分） | 首次有效接触在 0.35–0.55 s；最后 0.5 s 线速度 ≤ 0.01、角速度 ≤ 0.05、每步都有 (world, cube) 有效接触、法向力 ≈ m·g（1% 内）；全程四元数范数误差 ≤ 1e-6、全部有限 |
| 速度参考点 | 摆：枢轴原点线速度 ≤ 1e-12；角速度 = (0, q̇, 0)；铰链 1/1 维 |
| 自由关节坐标系 | 倾斜立方体落地翻转：关节局部角速度旋转后 = 实体世界角速度；线速度、姿态一致 |
| 接触对映射 | 两个方块叠放：(world, lower) 承受 1.5·g，(lower, upper) 承受 0.5·g；点数之和等于有效接触数 |
| 候选与有效 | 设置 margin/gap 后，出现“有候选接触、无有效接触”的时刻 |

**更新：**
- `physics.mujoco_world`、`physics.mujoco_step` 改为从快照读取位姿（`entityPose()` 已删除）。
- `headless.cli_parser` 按新格式逐字节比对 model/state 记录。

## 6. 验证记录

### 6.1 开发机（不是基线验证）

- **结果：** Debug + `-Werror`、Release + `-Werror`、ASan/UBSan 均 **28/28 通过**。5000 步 Headless 输出的 5004 行全部是合法 JSON。
- **变异测试：** 见 3.1–3.3 节，3 处故意引入的错误都被测试发现；恢复后 28/28。
- **立方体主基准实测：**

| 判定（开发计划 4.2） | 实测 | 候选门槛 |
| --- | --- | --- |
| 首次有效接触 | t = 0.428 s（理论触地时间 ≈ 0.428 s） | 0.35–0.55 s |
| 稳定窗口线速度 | 3.98e-14 m/s | ≤ 0.01 m/s |
| 稳定窗口角速度 | 3.76e-13 rad/s | ≤ 0.05 rad/s |
| 稳定窗口高度误差 | 1.08e-4 m | ≤ 0.005 m |
| 静止时法向力 | 9.81 N，4 个接触点 | ≈ m·g |
| 四元数范数误差 | 0 | ≤ 1e-6 |

  加上 M0-05 已验证的时间和自由落体，开发计划 4.2 节主场景的各项判定在开发机上都有了测量结果。正式的评审和冻结属于 M0-08。

### 6.2 基线机器

- **日期：**2026-11-16。
- **环境：**基线机器（同 M0-05），Ubuntu 22.04，GCC 11.4.0，链接 `third_party/mujoco/install` 中源码构建的 MuJoCo 4.0.1。
- **命令：**

```bash
rm -rf build
cmake --preset debug && cmake --build --preset debug && ctest --preset debug
cmake --preset release -DTERRA_WARNINGS_AS_ERRORS=ON && cmake --build --preset release && ctest --preset release
build/release/tests/test_mujoco_state
```

- **结果：**
  - Debug：配置、构建（23/23）成功，CTest **28/28 通过**，用时 1.12 s。
  - Release + `-Werror`：配置、构建（23/23）成功，无警告，CTest **28/28 通过**，用时 1.04 s。
  - `test_mujoco_state` 输出：

    ```text
    立方体基准：首次有效接触 t=0.428 s；稳定窗口 |v|max=3.98e-14 m/s，|ω|max=3.76e-13 rad/s，高度误差 0.000108 m；法向力 9.81–9.81 N（4 个接触点）；四元数范数误差 0
    all passed
    ```

  - 以上数值在打印精度内与开发机（6.1 节）完全一致。这只是单一夹具的观察，不代表承诺跨平台逐位一致；正式的容差评审和冻结属于 M0-08。

## 7. 验收标准对照（开发计划 M0-06）

| 标准 | 当前 |
| --- | --- |
| 至少输出时间、实体 ID、位置、姿态、速度、关节状态和有效接触摘要 | `state` 记录全部包含 |
| 字段有限、姿态归一化 | `diagnostics.finite`；四元数范数用例 |
| 初始数据与步后数据处于同一观测时刻 | 步后 `mj_forward`；位置差分与速度一致；第 1 步精确值 |
| 不存在指向后端可变数组的公开视图 | 快照为独立副本、`shared_ptr<const>`；不可变性用例；公共头不含 MuJoCo 类型（边界测试） |
| 自由关节配置与速度维度不同 | 7/6 用例 |
| 输出格式固定 | `contract_revision` 1，单元测试逐字节比对 |
