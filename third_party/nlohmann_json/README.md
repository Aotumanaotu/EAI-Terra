# nlohmann/json 3.12.0（第三方，未修改）

- 用途：设置文件（JSON）的读写（M1-14，D-026）。只在 `engine/settings/src/` 与测试中使用，公共头文件不暴露其类型（`boundary.render_isolation` 检查）。
- 来源：官方发布页 <https://github.com/nlohmann/json/releases/tag/v3.12.0> 的单头文件发布物 `json.hpp`、`json_fwd.hpp`，以及该标签下的 `LICENSE.MIT`。
- 许可：MIT（`LICENSE.MIT`）。文件内嵌的第三方片段（Hedley、Abseil 片段、Grisu2、UTF-8 解码器）均以 MIT 标注（SPDX）。
- 锁定：版本与 sha256 见 `json.lock`；`scripts/settings/check_json.sh` 校验本地文件，加 `--online` 时再与官方下载比对。
- 构建：CMake 目标 `terra::json`（仅头文件，SYSTEM 方式引入，定义 `JSON_USE_IMPLICIT_CONVERSIONS=0`，要求显式 `get<T>()`）。
