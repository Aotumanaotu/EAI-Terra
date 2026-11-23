// 后端不可恢复错误的进程行为（M0-08 建立；M2-02 改为 Terra 的致命错误处理）。
//
// MuJoCo 运行期的致命错误（mju_error）不能恢复：其处理函数不得返回。M2-02 起 Terra 登记自己的日志处理函数，
// 致命错误时写诊断、调用紧急保存、以退出码 70 结束（terra/core/fatal.hpp）。合法输入经 Terra 校验后不会触发它，
// 因此本探针在一次真实的 step 中经 MuJoCo 的控制回调（mjcb_control）调用 mju_error，模拟运行期致命错误。
// 在独立子进程中固定以下行为（由 CTest + TerraCheckExitCode.cmake 判定）：
//   - 退出码为 70；stderr 含 MuJoCo 的错误文字与紧急保存结果；
//   - 崩溃报告写在 TERRA_CRASH_DIR 中，含错误文字、世界实例、步序号与上下文；
//   - 登记的紧急保存已执行（模拟“退出前修改的设置已写入”）；
//   - atexit 处理仍会执行，且没有在工作目录写出 MUJOCO_LOG.TXT。
// 与 test_mujoco_probe 一样直接链接 MuJoCo，属于后端测试，不是公共接口的使用方式。

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include <mujoco/mujoco.h>

#include "terra/core/fatal.hpp"
#include "terra/physics/mujoco/mujoco_backend.hpp"

namespace {

namespace fs = std::filesystem;

fs::path workDir;

std::string readFile(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

void reportAtExit() {
    const bool logWritten = fs::exists(workDir / "MUJOCO_LOG.TXT");
    std::string crash;
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(workDir / "crash", ec)) {
        if (entry.path().filename().string().rfind("crash-", 0) == 0) crash = readFile(entry.path());
    }
    const bool crashOk = crash.find("terra fatal probe 42") != std::string::npos &&
                         crash.find("世界实例") != std::string::npos &&
                         crash.find("步序号（出错前已完成）：0") != std::string::npos &&
                         crash.find("设置文件：probe-settings.json") != std::string::npos &&
                         crash.find("探针设置：成功") != std::string::npos;
    const bool saved = readFile(workDir / "probe-settings.json") == "{\"saved\": true}\n";
    std::fprintf(stderr, "probe-atexit: %s, %s, %s\n", logWritten ? "MUJOCO_LOG.TXT written" : "no log file",
                 crashOk ? "crash report ok" : "crash report missing or incomplete",
                 saved ? "emergency save ok" : "emergency save missing");
}

void failInControl(const mjModel*, mjData*) { mju_error("terra fatal probe %d", 42); }

}  // namespace

int main(int argc, char** argv) {
    if (argc != 3) {
        std::fprintf(stderr, "用法：test_backend_fatal <模型> <工作目录>（需设置 TERRA_CRASH_DIR=<工作目录>/crash）\n");
        return 2;
    }
    workDir = fs::absolute(argv[2]);
    fs::remove_all(workDir);
    fs::create_directories(workDir);
    fs::current_path(workDir);

    auto loaded = terra::physics::mujoco::loadWorld(terra::data::ModelSource{argv[1]});
    if (!loaded.world) {
        std::fprintf(stderr, "加载失败：%s\n", loaded.error ? loaded.error->message.c_str() : "");
        return 3;
    }
    terra::core::setFatalContext("设置文件", "probe-settings.json");
    const terra::core::ScopedEmergencySave save("探针设置", [] {
        std::ofstream out(workDir / "probe-settings.json", std::ios::binary | std::ios::trunc);
        out << "{\"saved\": true}\n";
        return out ? std::optional<std::string>{} : std::optional<std::string>{"写入失败"};
    });
    std::atexit(reportAtExit);
    mjcb_control = failInControl;
    (void)loaded.world->step();
    std::fprintf(stderr, "step 返回了（不符合预期：致命错误处理函数不应返回）\n");
    return 4;
}
