# M4-10b 策略导入向导与自动配置：实现与验证

> 状态：**通过**（2026-11-25 第二次基线运行，`bash scripts/verify/m4_10b.sh --skip-mutation`）。展示中发现的一个界面问题已修正，并加入自检，随 M4-11 复验。
> 设计见 [M4-10 设计稿（修订）](M4-10-policy-interface-design.md)第 4 节。用户 2026-11-25 同意按建议开始。
> 另含 M4-10a 第一次运行中发现的两点：机器人与训练时不同（Go2W）时的警告；ROS2 的 `/cmd_vel` 作为策略的速度指令。

## 1. 用户能做什么

| 入口 | 行为 |
| --- | --- |
| 查看器：**策略 → 导入策略…**，或把 `.onnx` / `.pt` 拖进窗口 | 导入向导：<br>1. **选策略文件**：显示类型（ONNX 及其格式 / TorchScript / 训练检查点及对应的 ONNX）；<br>2. **选机器人**：逐个检查资源库中的场景，按关节名排序——完全对应、多出关节（悬停列出多出的关节，如 Go2W 的轮子）、部分对应；也可以选“其他场景…”；<br>3. 名字（默认用自动配置给出的名字）；**导入并运行** / **只导入**。<br>导入在后台进行，日志列出自动完成的每一项与警告 |
| 资源库 | 新的“**策略**”分页：卡片单击即打开推荐场景并运行。<br>场景卡片上显示“N 个策略”，右键“运行策略：…”。<br>模型或场景缺失时卡片变灰并说明原因 |
| 菜单“策略” | 导入策略…、运行策略…、停止策略、组件…（M4-10a 的“运行 / 停止”从“仿真”菜单移到这里） |
| ROS2 | 开启桥接时，策略的速度指令也可以来自 `/cmd_vel`：<br>· `geometry_msgs/Twist` 的 linear.x、linear.y、angular.z；<br>· 0.5 s 没有新消息时归零，与真机的超时停止相同；<br>· 速度指令面板显示“指令来自 ROS2 /cmd_vel”；<br>· 差速底盘已经占用 `/cmd_vel` 时不订阅，并说明 |
| Headless | `--import-policy <.onnx/.pt> --scene <场景> [--name N] [--library L]`：结果一行 `policy_import` JSON。<br>退出码：成功 0；需要填写草稿 4；没有 PyTorch 3。<br>`--list-assets --kind policy` |

## 2. 各类文件的处理

| 文件 | 处理 |
| --- | --- |
| `.onnx` | 按元数据与训练配置自动配置（M4-10a） |
| `.pt` 训练检查点 | 只含参数，不能单独运行。<br>在检查点所在目录及上两级中找 **spec_hash 与迭代次数都相同**的 ONNX。<br>**核查（只读）：**你的 `model_15000.pt` 与 `model_10000.pt` 的 spec_hash 相同（同一网络结构），只有迭代次数不同（15000 / 10000）。只比 spec_hash 会把 10000 的检查点错配到迭代 15000 的 `policy.onnx`。<br>因此：`model_15000.pt` → 使用 `policy.onnx`；`model_10000.pt` → 说明“网络结构相同但参数来自不同的迭代，不能代用”，并给出导出方法 |
| `.pt` TorchScript | 用 Python 与 PyTorch 转为 ONNX：`scripts/policy/torchscript_to_onnx.py`。<br>· Python 默认 `python3`，可用 `TERRA_PYTHON` 指定；没有 PyTorch 时给出安装命令（CPU 版）；<br>· TorchScript 没有观测布局等元数据，自动配置通常无法完成，所以生成**说明文件草稿**（`*.terra-policy-draft.json`）：已知的项（模型、输入输出名、IMU、力矩执行器的关节）已填好，`todo` 列出要填写的项 |
| 无法识别格式的 ONNX | 同样生成草稿 |

**导入的结果：**放在资源库目录下的 `policies/<名字>/`，包括：
- 模型副本；
- 训练配置副本；
- 说明文件（记下来源：原文件、类型、检查点）。

原文件不改动；同名时依次加 `-2`、`-3`。

## 3. 实现

| 部分 | 文件 |
| --- | --- |
| 导入 | `engine/policy/{include/terra/policy/import.hpp, src/import.cpp}`：文件类型、检查点读取（miniz 读 zip；只认 BINUNICODE、BININT 等几个操作码）、检查点对应的 ONNX、TorchScript 转换（子进程）、导入、草稿 |
| 机器人匹配 | `autoconfig.hpp`：`policyJointNames`、`matchRobotJoints`；自动配置新增“多出关节”的警告 |
| 资源库 | `engine/library`：资产类型 `policy`，扫描资源库目录下的 `policies/`。直接读 JSON，不依赖策略模块 |
| ROS2 | `Bridge::enableVelocityCommand`、`velocityCommand` |
| Headless | `--import-policy`、`--name`、`--kind policy`；`--policy` 加 `--ros2` 时订阅 `/cmd_vel` |
| 查看器 | `policy_panel.*`（导入向导）、`library_panel.*`（策略分页、卡片、右键菜单）、`main.cpp`（菜单、拖放、打开场景后运行）；文本表新增 26 条 |

## 4. 测试

| 测试 | 内容 |
| --- | --- |
| `policy.import`（新） | 文件类型（含两种整数编码与记忆操作）；检查点 → ONNX 的三种情况：找到、迭代不同、没有；导入 ONNX：目录、副本、来源、再导入得 `-2`、原文件不变；导入检查点；草稿；没有 Python 时的提示；目录名 |
| `policy.controller`（扩充） | 多出关节的警告；`matchRobotJoints`（完全、部分）；`policyJointNames` |
| `library.assets`（扩充） | 策略的登记、标题、关联、缺模型的标签；默认扫描范围加入 `policies/` |
| `headless.policy`（扩充） | `--import-policy`：<br>· ONNX：登记、警告；<br>· 草稿：退出码 4；<br>· 检查点：对应与拒绝；<br>· TorchScript：有 PyTorch 时转换（输入 17、输出 2），没有时跳过；<br>· 装有组件时运行导入的说明文件 |
| `headless.cli_parser`（扩充） | 新参数的解析与组合限制 |
| `ros2.policy_cmd_vel`（新） | 订阅 `/cmd_vel`；收到 Twist 后指令改变；停发 0.5 s 归零 |
| `graphics.viewer_policy_import_selftest`（新） | 导入向导：<br>· 推荐 policy_bench（完全对应），列出多出的 `bench/j3`；<br>· 导入并运行：登记为策略、打开场景；<br>· 自检的资源库没有装组件，所以弹出“需要下载组件” |

**预期数量：**

| 构建 | 预期 |
| --- | --- |
| 渲染关闭 Debug | 109 |
| 渲染 Debug、Release | 149 |
| 含 ROS2 | 164 |
| ASan 非图形 | 118（2 个跳过） |
| ASan 图形 | 31 |
| ASan 含 ROS2 | 15 |

**变异 6 个：**检查点只比 spec_hash、pickle 记忆操作、TorchScript 当成检查点、同名覆盖、多出关节不警告、不扫描 `policies/`。

## 5. 基线运行结果

**第一次（2026-11-25）：**
- 两个 Debug 构建的测试 109/109、149/149 全部通过，变异 6/6；
- 带 `-Werror` 的构建因测试中 5 处花括号初始化少写 `configDirectory` 失败，已修正。

**第二次（2026-11-25，跳过变异）：**

| 构建 | 结果 |
| --- | --- |
| 渲染关闭 Debug | 109/109 |
| 渲染 Debug、Release -Werror | 149/149 |
| 含 ROS2 Release -Werror | 164/164 |
| ASan 非图形 | 118/118（2 个按预期跳过） |
| ASan 图形 | 31/31 |
| ASan 含 ROS2 | 15/15 |

可交互展示：退出码 0。TorchScript 的转换在基线机器上实际执行并通过（输入 17、输出 2）。

**你的策略试导入（写在构建目录中）：**

| 文件 | 结果 |
| --- | --- |
| `policy.onnx` | 导入成功 |
| `model_15000.pt` | 对应到 `policy.onnx`，导入成功 |
| `model_10000.pt` | 拒绝，说明“网络结构相同（spec_hash 一致），但参数来自不同的迭代，不能代用”，并给出导出方法 |
| `policy.onnx` 对 Go2W | 警告 4 个轮子关节不由策略控制 |

**展示中发现的问题：**
- **现象：**日志中有一行“无法运行：文件不存在：”（路径为空）。
- **原因：**从菜单重开导入向导、还没选文件时，向导沿用了上一个文件的检查结果，“导入”按钮仍可按。
- **修正：**重开时清掉上次的结果；只有选好、检查过文件后才能导入。
- **自检：**`graphics.viewer_policy_import_selftest` 新增“菜单重开、未选文件时不能导入”。

**观察：**
- 展示中策略也在 AS2 场景中运行了：AS2 的关节名与 Go2 完全相同，向导按关节名列为“完全对应”；
- 关节名无法区分同名不同尺寸的机器人。以后可以在训练配置或元数据给出机器人信息（如腿长、质量）时加以比较，见第 6 节。

## 6. 未完成与后续

- **需要力矩执行器时生成场景副本：**没有做。现在的 Go2、G1 场景本来就是力矩执行器；策略的命令在 PD 层之后，同一执行器以策略为准，所以带 PD 层的场景也能运行。只有位置执行器的机器人会说明原因，以后需要时再做。
- **只读 TorchScript、不经 PyTorch：**不做。
- **转换带状态的 LSTM：**如宇树 `unitree_rl_gym` 的 `policy_lstm`。转换可能失败；失败时给出 PyTorch 的报错，留待有具体策略时处理。
- **同名关节、不同机器人的区分：**如 AS2 与 Go2。训练配置或元数据给出机器人信息（质量、腿长、模型文件的摘要）时比较，并在向导中警告。
- **部署配置的自动读取：**如宇树 `deploy_mujoco` 的 yaml。以后按格式解析器注册（autoconfig 的注册表）。
