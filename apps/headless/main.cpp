// EAI Terra Headless 入口（M0-01 骨架）。
//
// 当前仅提供 --help / --version。模型路径、步数、输出采样等运行参数
// 由 M0-03 接入；物理运行能力由 M0-04–06 接入。
//
// 输出约定：stdout 留给机器可读结果，诊断信息写 stderr。
// 退出码：0 成功；2 命令行用法错误。

#include <cstdio>
#include <string_view>

#include "terra/core/build_info.hpp"

namespace {

constexpr int kExitOk = 0;
constexpr int kExitUsage = 2;

void printUsage(std::FILE* out) {
    std::fputs(
        "用法: terra_headless [--help] [--version]\n"
        "\n"
        "  -h, --help     显示本帮助\n"
        "      --version  显示版本与构建信息\n"
        "\n"
        "仿真运行参数尚未接入（M0-03）。\n",
        out);
}

void printVersion() {
    const terra::BuildInfo& info = terra::buildInfo();
    std::printf("terra_headless %.*s (%.*s %.*s, __cplusplus=%ld)\n",
                static_cast<int>(info.version.size()), info.version.data(),
                static_cast<int>(info.compilerId.size()), info.compilerId.data(),
                static_cast<int>(info.compilerVersion.size()), info.compilerVersion.data(),
                info.cxxStandard);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        printUsage(argc == 1 ? stdout : stderr);
        return argc == 1 ? kExitOk : kExitUsage;
    }

    const std::string_view arg{argv[1]};
    if (arg == "-h" || arg == "--help") {
        printUsage(stdout);
        return kExitOk;
    }
    if (arg == "--version") {
        printVersion();
        return kExitOk;
    }

    std::fprintf(stderr, "terra_headless: 未知参数 '%s'\n\n", argv[1]);
    printUsage(stderr);
    return kExitUsage;
}
