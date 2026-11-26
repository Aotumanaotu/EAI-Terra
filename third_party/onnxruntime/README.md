# ONNX Runtime（C API 头文件）

- 来源：NuGet `Microsoft.ML.OnnxRuntime` 1.32.0（Microsoft 签名；包的 SHA-512 与 NuGet 目录一致，见 `onnxruntime.lock`）。
- 许可：MIT（`LICENSE`）。
- 仓库中只有编译用的 C API 头文件（`include/`，未修改）。运行库 `libonnxruntime.so` **不随仓库提供**：
  查看器在用户第一次运行 ONNX 策略时提示下载（组件管理器，带进度条与 sha256 校验；没有网络时给出手动安装的方法），
  Terra 运行时用 dlopen 加载，构建不依赖它。
- 校验：`bash scripts/import/check_third_party.sh`。
