# M0-02 MuJoCo 依赖集成与版本验证

> 状态：**已验收**。基线机器上 MuJoCo 4.0.1 源码构建成功，Terra Debug/Release 9/9 通过（4.2 节）；用户于 2026-11-19 确认评审通过。4.0.1 已于 2026-11-18 完成后续回归与许可核查并锁定，见第 5 节。

## 1. 版本选择

| 项目 | 内容 |
| --- | --- |
| 获取方式 | **官方源码构建**（用户决定，2026-11-13），便于后续修改；M0 期间按 D-007 不修改上游源码 |
| 候选版本 | **4.0.1**（2026-11-11 发布） |
| 备选版本 | 3.17.0（2026-10-22 发布，3.x 系列最后一版） |
| 选择理由 | 新项目直接从 4.x 起步，避免日后再做一次大版本迁移。4.0.1 修复了 4.0.0 引入的稀疏 Cholesky 分解缺陷（该缺陷会导致 Newton 求解器 Hessian 错误、发散）。风险：4.x 刚发布，若在 M0-04–08 的物理回归中出现异常，回退评估 3.17.0 |
| 来源 | `https://github.com/google-deepmind/mujoco/archive/refs/tags/4.0.1.tar.gz` |
| sha256 | `a1ccbe11674d09c4201067fca4d9954b1e4b0904cccdca991e21428e73d7db43`（2026-11-13 在开发机下载计算） |
| 许可 | MuJoCo 为 Apache-2.0（源码包 `LICENSE`）。下列依赖的许可已在 M0-09 核查，均为宽松许可，见 [M0-09 报告](M0-09-acceptance.md) 第 3 节 |

记录位置：`third_party/mujoco/mujoco.lock`。改动版本须同时更新 sha256，并重新执行本报告第 4 节的验证。

### 1.1 MuJoCo 核心库的构建依赖

MuJoCo 首次配置时通过 FetchContent 用 git 拉取以下依赖，commit 由 4.0.1 源码中的 `cmake/MujocoDependencies.cmake` 和 `cmake/third_party_deps/lodepng.cmake` 固定：

| 依赖 | commit | 仓库 |
| --- | --- | --- |
| lodepng | `22561883dd63fd1850f18e1f6adac321e4f609b0` | github.com/lvandeve/lodepng |
| tinyxml2 | `e6caeae85799003f4ca74ff26ee16a789bc2af48` | github.com/leethomason/tinyxml2 |
| tinyobjloader | `2945a967c5303b2c8c14174117c45f3302591150` | github.com/tinyobjloader/tinyobjloader |
| MarchingCubeCpp | `f03a1b3ec29b1d7d865691ca8aea4f1eb2c2873d` | github.com/aparis69/MarchingCubeCpp |
| libccd | `7931e764a19ef6b21b443376c699bbc9c6d4fba8`（v2.1） | github.com/danfis/libccd |
| qhull | `d1c2fc0caa5f644f3a0f220290d4a868c68ed4f6` | github.com/qhull/qhull |
| miniz | `77d0dce8627735138c51770d1799a1ef48f2117d` | github.com/richgel999/miniz |

abseil、googletest、benchmark 只在开启 MuJoCo 测试时拉取，Eigen 只在开启 Python 工具时拉取，这两组选项在 `build.sh` 中都已关闭。依赖下载到 `third_party/mujoco/deps/`，之后可用 `--offline` 离线重建。

## 2. 集成方式

```text
scripts/mujoco/fetch.sh   下载源码包，校验 sha256，解压到 third_party/mujoco/src/mujoco-4.0.1/
scripts/mujoco/build.sh   用 gcc-12 编译，Release，仅核心库，安装到 third_party/mujoco/install/
cmake/TerraMuJoCo.cmake   Terra 配置时读取 mujoco.lock，核对 mjVERSION_HEADER，加载官方 mujocoTargets.cmake
```

关键决定：

- **只构建核心库。** 关闭 simulate、示例、studio、MuJoCo 自身测试、Python 工具和 USD，因此不需要 GLFW 和图形开发包。
- **MuJoCo 用 GCC 12 编译。** 上游 CI 在 Ubuntu 22.04 上测试的最老编译器是 gcc-12；MuJoCo 自身用 C++20，并对自己的源码启用 `-Werror`。GCC 11 未经上游测试，有编译失败的风险。Terra 仍用系统 GCC 11 + C++17；两者只通过 MuJoCo 的 C API 交互，没有 C++ ABI 耦合。需要 `sudo apt install gcc-12 g++-12`。要尝试 GCC 11，可加 `--cc gcc --cxx g++`。
- **AVX。** MuJoCo 默认 `MUJOCO_ENABLE_AVX=ON`（编译参数 `-mavx`），产物只能在支持 AVX 的 CPU 上运行；基线机器 i7-13700 支持。
- **不使用 `find_package(mujoco)`。** 官方 `mujocoConfig.cmake` 无条件 `find_dependency(OpenGL)`，会让纯物理构建依赖 OpenGL 开发包；而 `libmujoco` 本身不链接 OpenGL（`src/render/classic` 通过 glad 在运行时 `dlopen` GL，M0 不调用任何 `mjr_*` 函数）。因此 Terra 直接加载官方导出的 `mujocoTargets.cmake`，这对应 D-015 中“隔离的导入目标”这一备选。
- **版本核对。** 配置时比较 `mujoco.lock` 与安装目录中头文件的 `mjVERSION_HEADER`，运行时比较头文件版本与 `mj_version()`。
- **运行库查找。** 使用 CMake 构建树 RPATH 指向 `third_party/mujoco/install/lib`，不依赖 `LD_LIBRARY_PATH`。安装/分发时的查找方式留到需要打包时再定。
- **构建阶段不联网。** 只有 `fetch.sh` 和首次 `build.sh` 需要联网；Terra 的 CMake 构建不联网。

### 2.1 私有后端边界

| 目标 | 可见 MuJoCo 头文件 | 说明 |
| --- | --- | --- |
| `terra_physics_mujoco` | 仅 `.cpp` 内部 | `PRIVATE` 链接 `mujoco::mujoco`；公共头 `terra/physics/mujoco/backend_info.hpp` 只用 Terra 类型 |
| `terra_headless` | 否 | 通过 `terra::physics_mujoco` 打印后端版本 |
| `test_mujoco_probe` | 是 | 隔离探针，唯一直接链接 MuJoCo 的测试 |

## 3. 新增测试

| 用例 | 检查内容 |
| --- | --- |
| `physics.mujoco_probe` | 清除显示变量后运行；`mj_version()` 等于 `mjVERSION_HEADER`；`mj_versionString()` 等于 lock 版本；`mj_loadXML/mj_makeData/mj_step/mj_forward/mj_resetData/mj_deleteData/mj_deleteModel/mj_name2id/mj_objectVelocity` 可解析。按计划不创建场景 |
| `physics.mujoco_backend_info` | 只经 Terra 公共头取得后端版本，版本一致且等于 lock 版本 |
| `boundary.no_mujoco_in_public_headers` | `engine/**/include/` 与 `apps/` 中没有 `#include <mujoco/...>` |
| `headless.version`（更新） | 输出中增加 `physics backend: MuJoCo 4.0.1` 一行 |
| `headless.no_graphics_libs`（沿用） | 现在 `terra_headless` 链接 `libmujoco.so`，ldd 结果也覆盖其传递依赖 |

## 4. 验证记录

### 4.1 开发机检查（未编译 MuJoCo，不是基线验证）

按用户要求，开发机上不编译 MuJoCo。已做以下检查：

- `fetch.sh`：下载、sha256 校验、解压成功；再次执行时识别到已有源码并跳过。`build.sh`：`--help` 与未知参数（退出码 2）正常。**未执行实际构建。**
- 以 MuJoCo 4.0.1 源码中的头文件，对新增的 4 个 C++ 文件做 `g++ -std=c++17 -fsyntax-only`，警告参数与 Terra 一致并加 `-Werror`：全部通过。`apps/headless/main.cpp` 在不提供 MuJoCo 头文件路径时也能编译。
- 以伪造的安装目录（头文件取自源码，`mujocoTargets.cmake` 为手写桩）只执行 Terra 配置阶段：
  - 版本正确时配置成功。
  - 安装目录缺失、版本号不符时均给出含修复提示的错误。
  - 生成的编译命令中，只有 `backend_info.cpp` 和 `test_mujoco_probe.cpp` 带 MuJoCo 头文件路径（以 `-isystem` 形式）。
- 边界检查脚本：当前仓库通过；对人工构造的两处违规均能检出。

### 4.2 基线机器验证

- **日期：** 2026-11-13
- **机器：** 与 [M0-01 报告](M0-01-environment.md) 4.2 节相同（HP Pro Tower 280 G9，i7-13700，Ubuntu 22.04.5 / x86_64）
- **编译器：** MuJoCo 用 gcc-12/g++-12 **12.3.0**（`12.3.0-1ubuntu1~22.04.3`，apt）编译；Terra 用 GCC 11.4.0；CMake 3.22.1，Ninja 1.10.1
- **执行命令：** 4.2 节原列出的命令，即 `fetch.sh`、`build.sh`、删除 `build/` 后 Debug 与 Release（`-Werror`）的配置、构建和测试，最后运行 `ldd`
- **说明：** 由于 Mutagen 同步中断，代码以打包文件的方式手动拷贝到基线机器（包 sha256 `471bce5368e4…`）。第一次运行时代码仍是 M0-01 版本，结果作废，不计入本记录。

**结果：**

| 项目 | 结果 |
| --- | --- |
| `fetch.sh` | 下载约 74 MB，sha256 与 `mujoco.lock` 一致，即两台机器独立下载的校验值相同 |
| MuJoCo 配置 | AVX 检测通过；FetchContent 按固定 commit 拉取 lodepng、qhull、tinyxml2、tinyobjloader、libccd、miniz（日志中各 commit 与 1.1 节一致）；qhull 与 libccd 应用了 MuJoCo 自带的补丁。MarchingCubeCpp 通过自定义下载命令获取，日志中不显示；`ls third_party/mujoco/deps` 确认 `marchingcubecpp-src` 存在，**7 个依赖均已获取** |
| MuJoCo 构建 | 131 个构建步骤全部成功；`-Werror` 未触发失败。miniz 有 `-Wunused-function` 警告（第三方依赖，不属于 `-Werror` 的作用范围）；LTO 串行编译 104 个 LTRANS 任务（只是提示） |
| 安装 | `lib/libmujoco.so.4.0.1`、`include/mujoco/*.h`、`lib/cmake/mujoco/mujocoTargets*.cmake`，以及附带的 `share/mujoco/model` |
| Terra 配置 | `-- MuJoCo 4.0.1: …/third_party/mujoco/install`，版本核对通过 |
| Debug | 构建 12 步，**9/9 通过** |
| Release（`-Werror`） | 构建 12 步，**9/9 通过** |
| `ldd terra_headless` | `libmujoco.so.4.0.1 => …/third_party/mujoco/install/lib/libmujoco.so.4.0.1`（RPATH 生效），其余只有 libstdc++、libc、libm、libgcc_s；**没有图形库** |

### 4.3 M0-02 验收标准对照

| 验收标准（开发计划 M0-02） | 结果 | 证据 |
| --- | --- | --- |
| 官方 C API 可链接，库可在无图形会话中装载 | 满足 | `physics.mujoco_probe` 在清除显示变量后运行，9 个所需符号均可解析；`ldd` 无图形库 |
| 版本一致 | 满足 | 配置时核对 lock 与 `mjVERSION_HEADER`；运行时 `mj_version()` 等于 `mjVERSION_HEADER`，`mj_versionString()` 为 `4.0.1` |
| 依赖缺失或不匹配能定位 | 满足 | 开发机上用伪造安装目录验证了缺失和版本不符两种情况的报错（4.1 节）；sha256 不匹配时 `fetch.sh` 报错并删除下载文件 |
| 没有修改上游源码 | 满足 | 使用校验过的官方源码包；只有 MuJoCo 自身构建流程对 qhull/libccd 打的补丁，属于上游行为 |
| 应用公共头不泄露 MuJoCo 类型 | 满足 | `boundary.no_mujoco_in_public_headers`；生成的编译命令中只有适配层和探针带 MuJoCo 头文件路径 |
| 交付物：依赖导入配置、版本兼容报告、锁定记录、私有后端骨架 | 满足（锁定记录为候选状态） | `cmake/TerraMuJoCo.cmake`、本报告、`mujoco.lock`、`terra_physics_mujoco` |

**验收结论：** 2026-11-19 用户评审通过；4.0.1 已按第 5 节完成锁定。

## 5. 锁定条件

满足以下全部条件后，在 `mujoco.lock` 中把状态改为“已锁定”，并更新 D-108。（2026-11-18：三项均已满足，4.0.1 已锁定，见 M0-09 报告。）

1. 4.2 节全部通过（M0-02 验收）。
2. M0-04–08 的立方体主基准与辅助用例在该版本下通过（开发计划要求“锁定仍需通过后续场景回归”）。
3. 依赖许可核查完成（D-109，在分发前完成即可，不阻塞 M0 开发）。
