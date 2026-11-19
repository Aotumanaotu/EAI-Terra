# glad 生成的 OpenGL 加载代码（M1-02）

本目录是**生成结果**，不要手工修改。

| 项目 | 值 |
| --- | --- |
| 生成器 | glad 2.0.8（源码包 sha256 见 `third_party/render/render.lock`） |
| API | `gl:core=4.5`（D-021） |
| 扩展 | 无 |
| 选项 | C 生成器，`--reproducible`（使用生成器自带的 Khronos 规范，不含时间戳），不含内置加载器 |
| 许可 | 生成代码 `(WTFPL OR CC0-1.0) AND Apache-2.0`（见 `include/glad/gl.h` 头部）；`KHR/khrplatform.h` 为 Khronos MIT 风格许可 |

函数指针在创建 OpenGL 上下文后由 `gladLoadGL(glfwGetProcAddress)` 加载。只包含 4.5 core 的函数，使用更高版本或扩展中的函数会在编译期报错。

复核：`scripts/render/gen_glad.sh`（用锁定版本重新生成并逐字节比较）。修改 API 或扩展：先改 `render.lock` 中的 `GLAD_API`/`GLAD_EXTENSIONS`，再运行 `scripts/render/gen_glad.sh --update`。
