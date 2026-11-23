# M2-04 多模型实例组合

> 状态：**已实现并通过基线验证，已验收（M2-09，2026-11-23）**（2026-11-23 第二次运行全部通过，第 5 节）。依据：[M2-01 设计](M2-01-scene-design.md) 第 2、3 节；开发计划 9.2 节 M2-04、9.5 节第 2 条（用户 2026-11-23：`mjSpec` 组合由 `terra_scene` 统一负责创建世界、管理实例前缀、处理全局配置和编译错误）。

## 1. 结构

```
terra_scene（新模块，不含 MuJoCo）                   MuJoCo 后端
  composeWorld(实例列表, 场景物理配置, 工厂, 标签)
    ├─ 校验实例名（合法、唯一、有 MJCF）——失败则不调用工厂
    ├─ 前缀 = "实例名/"
    ├─ 组合请求 WorldComposition ──────────────────→ composeWorld(请求)
    │    resolveOptions 回调（全局配置规则）  ←──────    1. 逐个 mj_parseXML（问题按实例返回）
    │                                                  2. 调用 resolveOptions 决定 6 项全局配置；其余全局设置随第一个实例
    │                                                  3. 给无名 body/几何体起临时名；mjs_attach 到各自的放置框架（加前缀）
    │                                                  4. mj_compile；编译警告
    │                                                  5. 建立世界（与 loadWorld 共用 build）
    └─ 问题归属：后端给出实例下标的直接归属；合并后的编译错误按文字中最先出现的前缀归属
```

**接口：**
- `terra/physics/world_composition.hpp`：`PhysicsOptions`、`ModelPlacement`、`WorldComposition`、`ComposeIssue`、`ComposeResult`，只用 Terra 类型；
- `terra/physics/mujoco/mujoco_backend.hpp`：`composeWorld`；
- `terra/scene/composition.hpp`：`composeWorld`、`resolvePhysicsOptions`、`attributeByPrefix`、`isValidInstanceName`、`instancePrefix`。

**边界：**`boundary.render_isolation` 增加规则：`engine/` 中只有 MuJoCo 后端的实现可以包含 `mujoco/` 头文件。`terra_scene` 只链接 `terra_physics`（仅头文件）与 `terra_data`。

## 2. 规则

| 项目 | 规则 |
| --- | --- |
| 实例名 | `[A-Za-z_][A-Za-z0-9_-]*`，不能重复；重复时指出第一次出现的位置 |
| 前缀 | `实例名/`，加在该实例所有有名字的元素上（body、关节、几何体、执行器、网格、材质等），由 MuJoCo 的 `mjs_attach` 完成 |
| 没有名字的元素 | 组合前，后端给没有名字的 body 与几何体起临时名字，以便按前缀归属到实例；对外仍显示为空名字，与单模型加载一致 |
| 6 项全局配置 | `timestep`、`gravity`、`integrator`、`solver`、`iterations`、`tolerance`：场景指定的为准，否则取第一个实例的值；其他实例不同时警告（场景指定的项不警告）；非法值（未知积分器等）报错 |
| 其他全局设置 | 摩擦锥、阻抗比、各类容差、开关标志、内存大小等一律取第一个实例的值；其他实例不同时警告并列出字段名。组合前把每个子模型的这些值改成与组合模型相同，避免 MuJoCo 的属性冲突规则（`compiler/conflict`）介入，结果只取决于 Terra 的规则 |
| 编译设置 | 角度单位、惯量计算方式等由各子模型保留，MuJoCo 按元素所属的子模型应用 |
| 放置位姿 | 每个实例挂在世界体下的一个框架上，框架的位置与四元数即放置位姿 |
| 子模型的生命周期 | `mjs_attach` 时组合模型对子模型加引用计数（MuJoCo 源码 `user_objects.cc`），Terra 释放自己的引用即可；网格等资源文件仍按子模型自己的目录解析 |

## 3. 测试 `scene.composition`（新增，总是运行）

**纯逻辑（假工厂，不需要 MuJoCo）：**
- 实例名的合法与不合法示例；
- 前缀；
- 错误按最先出现的前缀归属，包括 `a/` 出现在 `ba/x` 内部的情况；
- 全局配置规则：取第一个实例、场景指定、警告数量与归属；
- 校验失败时不调用工厂；
- 后端问题的归属。

**与 MuJoCo 后端集成（夹具 `assets/tests/scene/`）：**
- **地面 + 两辆相同小车**（`ground.xml`、`cart.xml` ×2，第二辆绕 Z 转 90°）：
  - 实体与执行器全名（`cart_a/chassis`、`cart_b/drive`），不带前缀的原名不存在；
  - 6 个实体，没有名字的 body 与几何体对外仍为空名字，但归属到正确的实例、与父实体同属一个实例；
  - 地面为静态几何，属于 ground 实例；
  - 放置位姿生效；
  - 只控制 cart_a 时 cart_b 完全不动；再控制 cart_b 时它沿世界 +Y 前进（滑轨随放置旋转）。
- **单实例与直接加载逐位相同**：`cart.xml` 带正弦控制运行 1000 步；`falling_cube.xml`（自由落体加地面接触）运行 2000 步。每一步所有实体的位置与四元数都逐位相同。
- **全局配置**（`cart.xml` + `cart_fast.xml`，后者的步长、重力、摩擦锥不同）：
  - 取第一个实例的值，并对 timestep、gravity、cone 给出警告；
  - 场景指定步长与重力后，这两项生效且不再警告，cone 仍警告；
  - 未知积分器报错。
- **错误归属**：
  - 文件缺失（`no_such.xml`）→ 对应实例，并带位置；
  - XML 语法错误（`broken.xml`）→ 对应实例；
  - 合并后编译失败（`bad_compile.xml`：带自由关节的 body 质量为 0）→ 按前缀归属到对应实例，并打印错误原文。

测试总数：渲染关闭 41 → **42**，渲染开启 59 → **60**。

变异测试 8 项（`scripts/verify/m2_04.sh`）：
- `terra_scene`：前缀未加、实例名校验放宽、全局配置忽略场景指定的步长、场景指定的项仍警告、合并后的编译错误不归属；
- MuJoCo 后端：放置位姿不生效、实例都归属到第一个、无名 body 不起临时名字。

## 4. 待验证的风险

- **名字中的 `/`**：MuJoCo 源码中没有看到对名字字符的限制，测试会直接验证。如果不行，改为 `实例名:`。
- **合并后编译错误的文字**：是否包含带前缀的元素名决定能否按前缀归属。测试会打印原文；如果不包含，归属退回到“整个场景”，并记录。
- **单实例逐位相同**：如果不成立，记录第一处不同的步数，评审是否接受。

## 5. 验证记录

开发机：未编译、未运行（按约定）。已做的静态检查：
- 变异锚点全部唯一，所有脚本合计 87/87；
- 脚本通过 `bash -n`；
- 对照 MuJoCo 4.0.1 源码核实了：`mjs_attach` 整个子模型时把子模型世界体中的元素挂到一个框架上；属性冲突的处理方式；子模型的引用计数；`mjs_numWarnings` 与 `mjs_getWarning`；`mjSpec` 与 `mjOption` 的字段。

**基线机器第一次运行（2026-11-23）：**
- 测试本身全部通过：渲染关闭 **42/42**，渲染 Debug **60/60**，变异 **8/8**。
- 但 `-Wshadow` 报了一个警告：抽出的 `build` 的参数名 `input` 与函数内原有的局部变量 `ControlInput input` 同名。Debug 构建只是警告；Release + `-Werror` 与 ASan 构建因此失败，这两项未判定。

**第 4 节的三个风险已由测试回答：**
1. 名字中的 `/` 可用：全名 `cart_a/chassis`、`cart_b/drive` 都能查到。
2. 单实例与直接加载逐位相同：小车 1000 步、下落立方体 2000 步。
3. 合并后的编译错误能按前缀归属到实例。错误文字的第一行是“Error: mass and inertia of moving bodies must be larger than mjMINVAL”，带前缀的元素名在后续行中；INFO 只打印了第一行。

**修正：**参数改名为 `inputs`，没有其他改动。

**第二次运行（2026-11-23）：全部通过。**渲染关闭 42/42；渲染开启 Debug、Release + `-Werror` 60/60；ASan 非图形 49/49、图形 11/11；变异 8/8。
