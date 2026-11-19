// 后端不可恢复错误的进程行为（M0-08，开发计划 4.3 节“加载与调用失败”）。
//
// MuJoCo 4.0.1 的 mju_error 默认处理：打印到 stderr 后调用 exit(EXIT_FAILURE)，调用方无法接管。
// 合法输入经 Terra 校验后不会触发它，因此本探针先经 Terra 加载模型（使 Terra 的日志配置生效），
// 再直接调用 mju_error，在独立子进程中固定以下行为（由 CTest + TerraCheckExitCode.cmake 判定）：
//   - 退出码为 1（与 Headless 的内部错误码相同）；
//   - stderr 含 MuJoCo 的错误文字；
//   - atexit 处理仍会执行，且没有在工作目录写出 MUJOCO_LOG.TXT。
// 与 test_mujoco_probe 一样直接链接 MuJoCo，属于后端测试，不是公共接口的使用方式。

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>

#include <mujoco/mujoco.h>

#include "terra/physics/mujoco/mujoco_backend.hpp"

namespace {

void reportLogFile() {
    const bool written = std::filesystem::exists("MUJOCO_LOG.TXT");
    std::fprintf(stderr, "probe-atexit: %s\n", written ? "MUJOCO_LOG.TXT written" : "no log file");
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 3) {
        std::fprintf(stderr, "用法：test_backend_fatal <模型> <工作目录>\n");
        return 2;
    }
    std::filesystem::create_directories(argv[2]);
    std::filesystem::current_path(argv[2]);
    std::filesystem::remove("MUJOCO_LOG.TXT");

    auto loaded = terra::physics::mujoco::loadWorld(terra::data::ModelSource{argv[1]});
    if (!loaded.world) {
        std::fprintf(stderr, "加载失败：%s\n", loaded.error ? loaded.error->message.c_str() : "");
        return 3;
    }
    std::atexit(reportLogFile);
    mju_error("terra fatal probe %d", 42);
    std::fprintf(stderr, "mju_error 返回了（不符合预期）\n");
    return 4;
}
