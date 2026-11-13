# M0-03 Headless 仿真程序

> 状态：**进行中**。命令行参数、校验、退出码、输出约定和生命周期骨架已完成，**基线机器 Debug/Release 20/20 通过**（6.2 节）。按开发计划，完整运行能力要等 M0-04–06 接入后才能验收；在此之前，实际运行会以退出码 3 报告“尚未接入”。

## 1. 命令行

```text
terra_headless --model <MJCF> (--steps N | --duration 秒) [--sample-every N] [--dry-run]
terra_headless --help | --version
```

| 参数 | 规则 |
| --- | --- |
| `--model <路径>` | 必需，不能为空。运行前检查路径存在、是常规文件且可读。相对路径按当前工作目录解析；MJCF 内部资源的相对路径由 M0-04 定义 |
| `--steps N` | 正整数（1 到 2^64−1），只允许十进制数字 |
| `--duration 秒` | 正的有限实数，拒绝 nan、inf、溢出和下溢。换算为步数需要模型步长，在 M0-04 接入后完成 |
| `--steps` / `--duration` | 必须且只能给一个 |
| `--sample-every N` | 每 N 步输出一次状态，默认 1。初始状态与最后一步是否总是输出，由 M0-05/06 确定 |
| `--dry-run` | 校验参数并加载模型（M0-04 起会编译 MJCF），输出解析后的配置（含 `timestep`、`total_steps`），不运行仿真 |
| `--help` / `--version` | 出现在任意位置时优先处理，其余参数不再校验 |

通用规则：

- 支持 `--opt value` 和 `--opt=value` 两种写法。
- 用空格分隔时，以 `--` 开头的下一个参数不会被当作值，例如 `--model --steps 10` 会报“缺少值”。
- 同一参数重复出现、未知参数、位置参数都会报错。

## 2. 退出码

| 码 | 含义 | 当前触发情况 |
| --- | --- | --- |
| 0 | 成功 | `--help`、`--version`、`--dry-run` 通过 |
| 1 | 内部错误 | 未捕获的异常、stdout 写入失败 |
| 2 | 参数错误 | 见第 1 节规则 |
| 3 | 后端不可用或功能未接入 | MuJoCo 头文件与运行库版本不一致；非 `--dry-run` 运行（M0-04–06 接入前） |
| 4 | 模型错误 | 文件不存在、不是常规文件、不可读；M0-04 起还包括 MJCF 解析/编译失败 |
| 5 | 仿真错误 | M0-05 起：非有限值、意外重置等 |

## 3. 输出约定

- **stdout**：只输出 JSON Lines，每行一个对象，带 `type` 字段。当前只有 `type: "config"`（`--dry-run` 时输出）；状态记录的格式在 M0-06 确定。
- **stderr**：诊断信息，以 `terra_headless: ` 开头；参数错误时附带用法说明。
- 字符串按 JSON 规则转义；路径中的非 UTF-8 字节原样输出（不保证是合法 JSON，已知限制）。实数以 `%.17g` 输出，可以无损还原为 double。

示例：

```text
$ terra_headless --model tests/../README.md --duration 0.5 --dry-run
{"type":"config","model":"tests/../README.md","model_resolved":"/home/…/README.md","duration":0.5,"sample_every":1,"dry_run":true}
```

## 4. 结构

| 部分 | 位置 | 说明 |
| --- | --- | --- |
| `terra_headless_cli` | `apps/headless/cli.{hpp,cpp}` | 参数解析、模型文件检查、JSON 输出；只依赖标准库，可单独测试 |
| `terra_headless` | `apps/headless/main.cpp` | 生命周期：解析 → 检查后端 → 检查模型 → 运行 → 释放；捕获所有异常，转为退出码 1 |

应用不包含 MuJoCo 头文件，由 `boundary.no_mujoco_in_public_headers` 检查；不创建任何图形上下文。

## 5. 测试

CTest 测试从 9 项增加到 20 项：

- `headless.cli_parser`：单元测试，覆盖合法输入、各类用法错误、约 30 个非法数值、JSON 转义与输出格式。
- `headless.*`：13 项端到端测试，精确比较退出码，并用正则检查 stdout/stderr。包括无参数、未知参数、缺少模型、互斥参数、非法步数、模型不存在、模型路径是目录、两种 `--dry-run`、运行未接入，以及清除显示变量后的 `--version` 与 `--dry-run`。
- 原有 `headless.version`、`headless.no_graphics_libs`、`physics.*`、`boundary.*` 保留。

`TerraCheckExitCode.cmake` 改为把脚本路径之后的参数原样传给被测程序，并新增 `STDOUT_REGEX`、`STDERR_REGEX` 检查。

## 6. 验证记录

### 6.1 开发机（不是基线验证）

- **单元测试：** `cli.cpp` 和单元测试按 Terra 的警告参数加 `-Werror`，并开启 AddressSanitizer + UBSan 编译运行，全部通过。
- **端到端：** 在 `/tmp` 中用桩库替代 `libmujoco`（头文件取自 4.0.1 源码，只实现探针和后端用到的函数），完整执行 Terra 的配置、构建（`-Werror`）和 CTest，结果 **20/20 通过**。
  - 首次运行时两个 `--dry-run` 用例失败：测试正则开头只给 `{"` 留了一个字符。修正正则后通过。
  - 另外做了反向检查：把期望值故意写错时，测试会失败。
  - 桩环境和构建目录用完即删除，没有在仓库内产生构建产物。
- **未验证：** 链接真实 `libmujoco` 的完整构建。

### 6.2 基线机器

- **日期：** 2026-11-13；机器与工具链同 [M0-02 报告](M0-02-mujoco.md) 4.2 节（GCC 11.4.0，链接真实的 MuJoCo 4.0.1 安装）。
- **命令：**

```bash
rm -rf build
cmake --preset debug && cmake --build --preset debug && ctest --preset debug
cmake --preset release -DTERRA_WARNINGS_AS_ERRORS=ON && cmake --build --preset release && ctest --preset release
```

- **结果：** 两种配置都是全新配置，各构建 16 步，没有警告，**20/20 通过**（`headless.cli_parser`、13 项端到端用例，以及原有的 6 项）。

## 7. 验收标准对照（开发计划 M0-03）

| 标准 | 当前 |
| --- | --- |
| 缺少 `DISPLAY` / `WAYLAND_DISPLAY` 时可运行 | `headless.no_display`、`headless.no_display_dry_run`，基线通过 |
| 不创建 OpenGL/EGL/OSMesa 上下文 | 代码中没有任何图形调用；`ldd` 无图形库 |
| 非法参数非零退出 | 退出码 2 / 4，已测试 |
| 正常完成零退出 | `--dry-run` 已测试；**实际仿真运行待 M0-04–06** |
| 应用不调用 `mj_*`，也不接触其数据结构 | 边界测试，以及编译命令中应用不带 MuJoCo 头文件路径 |
