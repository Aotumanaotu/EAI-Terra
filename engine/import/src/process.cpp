#include "process.hpp"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>

#include <fcntl.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;

namespace terra::importer::detail {

namespace {

std::string readAll(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    return std::string{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

}  // namespace

bool programAvailable(const std::string& program) {
    if (program.find('/') != std::string::npos) return ::access(program.c_str(), X_OK) == 0;
    const char* path = std::getenv("PATH");
    if (path == nullptr) return false;
    std::string dirs = path;
    std::size_t start = 0;
    while (start <= dirs.size()) {
        const std::size_t end = dirs.find(':', start);
        const std::string dir = dirs.substr(start, end == std::string::npos ? std::string::npos : end - start);
        if (!dir.empty() && ::access((dir + "/" + program).c_str(), X_OK) == 0) return true;
        if (end == std::string::npos) break;
        start = end + 1;
    }
    return false;
}

ProcessResult runProcess(const std::vector<std::string>& args, const std::string& stdoutPath) {
    ProcessResult result;
    if (args.empty()) return result;
    // 标准输出与标准错误先写入临时文件（避免管道读写的死锁），结束后读回
    std::error_code ec;
    const auto tmp = std::filesystem::temp_directory_path(ec);
    char outTemplate[] = "terra-import-out-XXXXXX";
    char errTemplate[] = "terra-import-err-XXXXXX";
    const std::string outPath = stdoutPath.empty() ? (tmp / outTemplate).string() : stdoutPath;
    const std::string errPattern = (tmp / errTemplate).string();
    std::vector<char> errName(errPattern.begin(), errPattern.end());
    errName.push_back('\0');
    const int errFd = ::mkstemp(errName.data());
    int outFd = -1;
    std::vector<char> outName;
    if (stdoutPath.empty()) {
        outName.assign(outPath.begin(), outPath.end());
        outName.push_back('\0');
        outFd = ::mkstemp(outName.data());
    } else {
        outFd = ::open(stdoutPath.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    }
    if (errFd < 0 || outFd < 0) {
        if (errFd >= 0) ::close(errFd);
        if (outFd >= 0) ::close(outFd);
        result.errors = "无法创建临时文件";
        return result;
    }
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_adddup2(&actions, outFd, STDOUT_FILENO);
    posix_spawn_file_actions_adddup2(&actions, errFd, STDERR_FILENO);
    posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
    std::vector<char*> argv;
    std::vector<std::string> copies(args);
    for (auto& a : copies) argv.push_back(a.data());
    argv.push_back(nullptr);
    pid_t pid = 0;
    const int rc = ::posix_spawnp(&pid, argv[0], &actions, nullptr, argv.data(), environ);
    posix_spawn_file_actions_destroy(&actions);
    ::close(outFd);
    ::close(errFd);
    if (rc == 0) {
        int status = 0;
        while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {
        }
        result.started = true;
        // 与 WIFEXITED / WEXITSTATUS 相同的换算（不用宏：其中有 C 风格的类型转换）；被信号终止时为 128 + 信号
        result.exitCode = (status & 0x7f) == 0 ? (status >> 8) & 0xff : 128 + (status & 0x7f);
    } else {
        result.errors = std::string{"无法启动 "} + args[0] + "：" + std::strerror(rc);
    }
    if (stdoutPath.empty()) {
        result.output = readAll(outName.data());
        ::unlink(outName.data());
    }
    result.errors += readAll(errName.data());
    ::unlink(errName.data());
    return result;
}

}  // namespace terra::importer::detail
