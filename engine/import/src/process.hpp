#pragma once

// 运行外部程序（M4-08a：python3 展开 xacro、assimp 转换网格）。只在 Linux 上实现（posix_spawnp）。

#include <string>
#include <vector>

namespace terra::importer::detail {

struct ProcessResult {
    bool started = false;   // 程序能否启动（找不到程序时为 false）
    int exitCode = -1;
    std::string output;     // 标准输出
    std::string errors;     // 标准错误
};

// args[0] 为程序名（按 PATH 查找）；可选 stdoutPath 非空时标准输出写入该文件而不是 output
ProcessResult runProcess(const std::vector<std::string>& args, const std::string& stdoutPath = {});

// 程序是否可用（在 PATH 中找得到且可执行）
bool programAvailable(const std::string& program);

}  // namespace terra::importer::detail
