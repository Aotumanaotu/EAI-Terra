# stb_image 2.32 与 stb_image_write 1.18（第三方，未修改）

- 用途：读取 Radiance `.hdr` 环境贴图（M1-08b，D-027）。只在 `engine/render/src/` 与测试中包含（`boundary.render_isolation` 检查）；实现只编译 HDR 解码（`STBI_ONLY_HDR`）。
- stb_image_write：写 PNG 参考图与差异图（M1-10），只在 `engine/render/src/` 中编译。
- 来源：<https://github.com/nothings/stb>，提交 `9f755b604db3492df2cbeb39bf84b81e3e3932e3`（2026-11-12，版本号 2.32）的 `stb_image.h`、`stb_image_write.h` 与 `LICENSE`。
- 许可：公有领域（Unlicense 风格）或 MIT，二选一（`LICENSE`，文件末尾亦有全文）。
- 锁定：sha256 见 `stb.lock`；`scripts/render/check_stb.sh` 校验本地文件，`--online` 时再与该提交的原始文件比对。
- 构建：CMake 目标 `terra::stb`（仅头文件，SYSTEM 方式引入）。
