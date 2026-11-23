# cgltf 1.15（第三方，未修改）

- 用途：解析 glTF 2.0（`.gltf` / `.glb`）（M1-09，D-013）。只在 `engine/asset/src/` 与测试中包含（`boundary.render_isolation` 检查）。
- 来源：<https://github.com/jkuhlmann/cgltf> 标签 `v1.15`（提交 `360db1a95480fe102ae9c69b27c5d101167ff5ba`）的 `cgltf.h` 与 `LICENSE`。
- 许可：MIT（`LICENSE`）。
- 锁定：sha256 见 `cgltf.lock`；`scripts/render/check_cgltf.sh` 校验本地文件，`--online` 时再与该标签的原始文件比对。
- 构建：CMake 目标 `terra::cgltf`（仅头文件，SYSTEM 方式引入）。
