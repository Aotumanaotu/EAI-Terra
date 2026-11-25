# M4-08a 模型导入

> 状态：**已实现并通过基线验证**（2026-11-25，第 5.2 节）。依据：[M4-08 设计稿](M4-08-import-design.md)（2026-11-25 评审通过）第 2 节与第 8 节的修订。

## 1. 交付

**新模块 `terra_import`**（`engine/import/`）：URDF / xacro / MJCF → Terra 场景。
- 不包含 MuJoCo 与图形头文件；加载检查用调用方给的物理后端。
- 分两步：
  - `analyze()` 只读取与检查，供导入向导显示与修改；
  - `execute()` 写出导入目录并检查。

| 步骤 | 做法 |
| --- | --- |
| 识别 | 根元素 `<robot>` 为 URDF，`<mujoco>` 为 MJCF；`.xacro` 或文本中有 xacro 语法时先展开 |
| xacro | 随 Terra 提供官方 xacro 2.1.1（`third_party/xacro/`，PyPI wheel，BSD-3-Clause，未修改）。`engine/import/tools/xacro_expand.py` 把 `$(find 包名)` 换成包目录表，**不需要 ROS**，只需要 `python3`。也支持 xacro 参数（`名字:=值`） |
| 包目录 | `package://` 与 `$(find)` 按以下顺序查找：<br>1. 命令行或向导中给出的目录；<br>2. 模型所在的包的同级与上两级目录中的 `package.xml`（可找到 `ROBOTS/` 下并列的包）；<br>3. 已 source 的 ROS 环境 |
| 网格 | STL、OBJ 直接复制。其余（Collada 等）用 assimp 命令行工具转为 OBJ，并按材质拆分（保留颜色与贴图）。<br>坐标轴约定与 RViz 相同：保持文件中的坐标轴，单位按 `<unit>` 换算为米。assimp 命令行工具不能关闭“按 up_axis 旋转”，因此转换后与文件中的原始顶点比较包围盒，识别并撤销它的旋转；识别不了时（文件中有节点变换）给出警告 |
| 单位检查 | 网格缩放后最长边超过 5 m 时，提示“可能以毫米为单位”，并给出缩放 0.001 的办法 |
| URDF → MJCF | Terra 自己转换（tinyxml2，与 MuJoCo 4.0.1 使用同一版本，zlib 许可），见下方说明 |
| 执行器 | URDF 没有执行器，按关节类型提议（有 `<transmission>` 时只给其中的关节），见下方说明 |
| 传感器 | 按连杆名提议：<br>- `*lidar*`、`*laser*`、`*scan*` → 2D 激光雷达；<br>- `*camera*` 等 → 相机，优先光学坐标系，安装方向按 ROS 的两种坐标约定换算；<br>- `*imu*` → IMU |
| 放置 | 零位正运动学求碰撞几何体的最低点，把机器人放在地面上；根连杆为 `world` 时固定基座，否则为浮动基座（可修改） |
| 检查 | 加载生成的场景并推进 1 s：没有仿真错误、状态有限、没有掉到地面以下 |

URDF → MJCF 的转换规则：
- 连杆 → body，关节 → hinge / slide，`fixed` → 不加关节；
- 惯量用 `fullinertia`；
- 可见几何体在 group 2 且不参与碰撞，碰撞几何体在 group 3；质量只取 `<inertial>`；
- 活动连杆没有惯量时给 0.01 kg 并警告；
- planar、mimic 关节给出警告。

执行器的提议规则：

| 关节 | 执行器 | 增益 | 范围与力限 |
| --- | --- | --- | --- |
| continuous | 速度执行器 | kv 按下游质量估计 | 力限取 `effort` |
| revolute、prismatic | 位置执行器（阻尼比 1） | kp 按下游质量估计 | 范围取关节限位，力限取 `effort` |

**导入目录** `<资源库目录>/assets/<名字>/`（M4-08 设计稿 8.3 节）：

| 文件 | 内容 |
| --- | --- |
| `<名字>.xml` | MJCF |
| `meshes/`、`textures/` | 网格与贴图 |
| `floor.xml` | 地面 |
| `<名字>.terra-scene.json` | 场景 |
| `import-report.json` | 每个问题的级别、代码、说明与处理办法；网格、执行器、传感器 |

已存在时不覆盖（命令行 `--overwrite`）；有错误的计划不导入。

**命令行 `terra_import`：**
```
terra_import <URDF / xacro / MJCF> [--name 名字] [--output 目录] [--package 包名=目录]... [--mesh-scale 比例]
             [--fixed-base | --floating-base] [--no-actuators] [--no-sensors] [--overwrite] [--dry-run] [名字:=值]...
```
- 标准错误每个问题一行（级别、代码、说明 → 处理办法）；
- 标准输出最后一行 `{"type":"import",...}`；
- 退出码 0 成功、2 参数错误、4 有错误。

**查看器：**
- 导入向导：“文件 → 导入模型…”（文件浏览窗口按 `.urdf|.xacro|.xml` 过滤），或把文件拖进窗口：
  - 场景文件直接打开，动作文件打开并播放，其余进入向导；
  - 向导显示：文件与规模；问题列表（错误、警告、提示，每条附处理办法）；
  - 向导中可修改：名字、网格缩放、固定基座、补充包目录（“重新分析”）、网格表、执行器表（启用、类型、增益、力限）、传感器；
  - “导入并打开”在有错误时不可用。
- 首次启动（没有设置文件）时，窗口取主显示器的 85%（原来固定为 1600 × 900）；默认停靠比例：右侧 24% → 28%，底部 24% → 26%（设计稿 8.4 节）。
- 平台层新增拖放（`Window::takeDroppedPaths`）。文件浏览窗口支持用 `|` 分隔的多个后缀。

## 2. 测试

**夹具**（`assets/tests/import/`，由 Terra 编写）：

| 夹具 | 内容 |
| --- | --- |
| `demo_robot/` | 最小的 ROS 包。差速小车：STL 车身、Collada 标记块（单位厘米、Z_UP、两种材质）、带贴图的轮子、一个 revolute 手臂、没有惯量的雷达、相机（含光学坐标系）与 IMU 连杆 |
| `demo.urdf.xacro`（+ `wheel.xacro`） | 宏、属性、表达式、参数、`$(find)` |
| `missing_pkg.urdf.xacro` | 引用不存在的包 |
| `broken.urdf` | 缺网格、未知的包、毫米单位、没有惯量、mimic |

| 测试 | 内容 |
| --- | --- |
| `import.analyze` | 见下方说明 |
| `import.execute` | 见下方说明 |
| `import.demos` | ROBOTS/import_demos 中的 R2D2（07-physics、08-macroed）与 TurtleBot 3 Burger 一步导入；TurtleBot 3 两轮驱动能开动。没有下载时跳过 |
| `import.cli_dry_run`、`import.cli_broken` | 命令行的只分析模式；出错的模型返回 4，并逐项给出处理办法 |
| `graphics.viewer_import_selftest` | 查看器中第 12 帧分析、第 14 帧导入并打开；向导出现过、导入成功、打开了生成的场景、导入目录在资源库下 |

`import.analyze`（不需要 MuJoCo）检查：
- demo.urdf：连杆、关节、包、网格、执行器、传感器（相机选光学坐标系）；
- xacro 展开（参数生效）；
- 找不到包时报 `unknown_package`；
- broken.urdf 的 5 类问题，每条都有处理办法；设置网格缩放后不再提示单位；
- MJCF；文件不存在；名字清理。

`import.execute` 检查：
- 导入目录的文件齐全；
- Collada 换算为 0.1 × 0.04 × 0.02 m、坐标轴与文件相同；按材质拆分（顶面单独一份）；
- 场景能加载：3 个执行器、雷达、相机、IMU；导入时的检查落在地面上；
- 驱动：两轮同向前进；反向原地左转；手臂到 1.0 rad；
- 已存在时不覆盖；有错误的计划不导入。

**测试总数：**渲染关闭 88 → **93**，渲染开启 122 → **128**。`import.execute` 与查看器导入自检需要 assimp。

**变异（8 个）：**

| 变异 | 由哪个测试发现 |
| --- | --- |
| Collada 不撤销 assimp 的旋转 | `import.execute` |
| OBJ 不按材质拆分 | `import.execute` |
| package:// 不解析 | `import.analyze` |
| 不提示毫米单位 | `import.analyze` |
| 轮子提议为位置执行器 | `import.analyze` |
| xacro 不传包目录 | `import.analyze` |
| 导入后不打开场景 | `graphics.viewer_import_selftest` |
| 命令行出错时退出码为 0 | `import.cli_broken` |

另：M2-08 的变异“文件浏览不过滤后缀”的锚点随文件浏览窗口的修改更新（`endsWith` → `matches`）。

**手动测试**（设计稿 8.1 节；验证脚本最后打开查看器时按提示做）：
1. 导入 R2D2：Collada 夹爪带贴图；
2. 导入 TurtleBot 3：展开 xacro、毫米 STL、两轮开动；
3. 把 broken.urdf 拖进窗口：每个问题都有原因与办法；
4. 观察首次启动的窗口大小。

## 3. 已知的限制

- 导入在界面线程中执行，转换大网格（如 TurtleBot 4 的 OAK-D 12 MB）时界面会停几秒。
- Collada 文件中有节点变换时，方向无法与原始顶点核对，只给警告，需要目视检查。
- 贴图只支持 PNG（MuJoCo 的限制），其他格式给出警告并只用颜色。
- 只提议一个雷达、一个相机、一个 IMU；更多的在场景文件中添加。
- 资源库登记与“资源库位置”设置在 M4-08c。

## 4. 文件

- 新增：
  - `engine/import/`：`CMakeLists.txt`、`include/terra/import/importer.hpp`、`src/{importer,mesh,process,urdf}.cpp`、`tools/xacro_expand.py`；
  - `apps/import/`；
  - `apps/viewer/import_wizard.{hpp,cpp}`；
  - `third_party/tinyxml2/`、`third_party/xacro/`；
  - `scripts/import/check_third_party.sh`；
  - `assets/tests/import/`、`tests/import/test_import.cpp`；
  - `scripts/verify/m4_08a.sh`。
- 修改：
  - `CMakeLists.txt`、`cmake/TerraCheckRenderIsolation.cmake`；
  - `engine/platform/`（拖放）、`engine/ui/`（文件浏览窗口的多后缀，界面文字）；
  - `apps/viewer/{main.cpp,CMakeLists.txt}`；
  - `tests/CMakeLists.txt`、`scripts/verify/m2_08.sh`。

## 5. 验证记录

### 5.1 开发机

未编译、未运行（按约定）。已做的静态检查：
- 第三方源码与锁定的 sha256 一致：tinyxml2 与 MuJoCo 4.0.1 的依赖版本相同，且与 GitHub 上的固定提交一致；xacro 与 PyPI 的 wheel 一致；
- xacro 包装脚本展开 demo.urdf.xacro：6 个连杆，参数 `wheel_effort:=7` 生效；找不到包时给出 `--package` 的提示；不在 `third_party/` 中写 `__pycache__`；
- 变异锚点合计 268/268 唯一；
- 脚本通过 `bash -n`。

### 5.2 基线机器

**第一次运行（2026-11-25）：编译失败。**
- 依赖都已就绪：assimp、python3、公开示例。
- 错误一：`importer.cpp` 中的旋转函数名为 `apply`，参数是 `std::array`，按实参查找找到了 `std::apply` 并被选中，编译失败。已改名为 `rotate`。
- 错误二：查看器中 `auto scene` 遮蔽了外层同名变量（-Wshadow）。已改名。
- 测试文件补上 `<algorithm>`、`<utility>`。
- 构建在第一个错误处停止，其余文件是否还有问题未知。请用户先以 `-k 0` 编译，列出全部错误与警告。

**第二次运行（2026-11-25）：编译通过；渲染开启 128 项中 124 项通过，变异 8/8。**
- 通过的部分：
  - TurtleBot 3 Burger 一步导入（xacro 参数、毫米 STL），两轮 6 rad/s 1 s 前进 0.181 m；
  - Collada 识别为“按 Z_UP 旋转为 Y_UP，已换算单位”，并撤销旋转；
  - 展示中用户经向导导入 R2D2（07-physics）与 TurtleBot 3，两个场景都正常打开。
- 失败一：demo.urdf（`import.execute` 与查看器导入自检）组合时 MuJoCo 报 “mesh volume is too small: demo_robot/mesh1_marker_visual0”。
  - 原因：按材质拆分后，顶面只有一个面，体积为 0，MuJoCo 按体积计算网格惯量失败。
  - 修改：网格资产一律 `inertia="shell"`。质量只取 URDF 的 `<inertial>`（几何体 density 0），网格的惯量本来就不用。
- 失败二：`import.demos` 中 R2D2 的实例名为 “07-physics”，场景要求实例名以字母或下划线开头。
  - 修改：`sanitizeName` 在这种情况下加前缀 `robot_`；测试中给三个示例起名。
- 失败三：“xacro 中找不到包”没有识别为 `unknown_package`。
  - 最可能的原因：ctest 环境的语言设置使 Python 输出中文的错误信息出错。
  - 修改：调用 Python 时加 `-X utf8`；也按异常名 `PackageNotFoundError` 识别。
- 另：地面的积分器（Euler）与机器人（implicitfast）不同时有警告。生成的场景改为在 `physics` 中统一指定 implicitfast。

**第三次运行（2026-11-25）：128 项中 127 项通过，变异 8/8；用户确认展示效果都没问题。**
- demo.urdf 导入并驱动：
  - 推进 1 s 后根连杆高度 −0.0002 m；
  - 两轮 8 rad/s 前进 0.375 m；反向 4 rad/s 转 1.215 rad；
  - Collada 换算为 0.1 × 0.04 × 0.02 m。
- R2D2 两个版本导入成功（8 个执行器，根连杆 0.470 m）；TurtleBot 3 前进 0.181 m；查看器导入自检通过。
- 失败：`import.analyze` 的“xacro 中找不到包”。原因是夹具 `missing_pkg.urdf.xacro` 的注释中写了 “--package”，XML 注释里不允许出现 “--”，xacro 报的是 XML 语法错误，而不是找不到包。
  - 已修改注释；全部导入夹具已用 Python 的 XML 解析器检查。
  - 在开发机上用包装脚本复现：报 “找不到包‘nonexistent_pkg’……用 --package 指定”。
- **用户要求：**文件浏览窗口的展示有点生硬，希望用网格加矢量图标。已改为：
  - 默认为网格视图；图标用 ImDrawList 绘制，任何缩放下都清晰；按类型着色并标出类型（文件夹、SCENE、MOTION、REC、URDF、XACRO、MJCF、JSON、MESH、IMG）；
  - 名字最多两行，超出加省略号；悬停显示全名与大小；
  - 右上角切换网格 / 列表（列表带小图标、类型与大小）；
  - 路径栏改为逐级可点的按钮，“编辑路径”切换为输入框；在文件区按退格回到上一级。
  - 场景、动作与导入的文件浏览都使用新界面。

**第四次运行（2026-11-25）：全部通过。**
- 测试：93/93；渲染开启 Debug 与 Release -Werror 128/128；ASan 102/102 + 26/26；变异 8/8。
- 展示：用户经向导导入 R2D2 与 TurtleBot 3，场景正常打开；查看器运行 189.9 秒。
- **用户对文件浏览窗口的两处意见（按用户要求放到 M4-08b）：**
  - 列表视图中图标与文件名之间应留出间隔；
  - 路径栏改为 Windows 文件管理器的方式：点击路径栏即可直接编辑，不需要单独的“编辑路径”按钮；点击某一级目录跳转的功能保留。
  - 见 [M4-08b 设计稿](M4-08b-turtlebot4-design.md)第 6 节。
