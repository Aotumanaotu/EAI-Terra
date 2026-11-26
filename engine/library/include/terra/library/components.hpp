#pragma once

// 可下载的运行组件（M4-10a；用户 2026-11-26：不同的策略需要不同的库文件，客户端提示并代为下载安装，带进度；
// 没有网络时给出警告与手动安装的方法）。
//
// 组件清单（assets/components/components.json）列出每个组件：标识、名称、版本、许可、用途、下载地址、大小、sha256，
// 以及安装时从压缩包（zip，如 NuGet 的 .nupkg）中取出哪些文件（各自的 sha256）。
// 组件装在资源库目录下的 components/<标识>/（不写系统目录，不需要 root），其中 installed.json 记录版本与文件摘要。
// 下载用 curl 命令行（file:// 地址也可以，便于测试与离线分发）；也可以从本地文件安装（离线时）。
// 安装先写到隐藏的临时目录，全部校验通过后再改名，中途失败或取消不留下半个组件。

#include <atomic>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace terra::library {

struct ComponentFile {
    std::string from;    // 压缩包中的路径
    std::string to;      // 安装后相对组件目录的路径
    std::string sha256;
};

struct ComponentInfo {
    std::string id;
    std::string nameChinese, nameEnglish;
    std::string purposeChinese, purposeEnglish;
    std::string version;
    std::string license;
    std::string url;          // 下载地址（https:// 或 file://）
    std::uint64_t size = 0;   // 压缩包的字节数
    std::string sha256;       // 压缩包的摘要
    std::string source;       // 来源说明（如 “NuGet，Microsoft 签名”）
    std::vector<ComponentFile> files;
    std::string library;      // 运行时加载的库（相对组件目录）
};

struct ComponentCatalog {
    std::vector<ComponentInfo> components;
    std::vector<std::string> errors;
    const ComponentInfo* find(const std::string& id) const;
};

ComponentCatalog readComponentCatalog(const std::string& path);

// 组件目录：库文件所在目录下的 components/
std::string componentsDirectory(const std::string& libraryFile);

enum class ComponentState { Missing, Installed, Corrupt, Outdated };
const char* componentStateName(ComponentState state);  // "missing"、"installed"、"corrupt"、"outdated"

struct ComponentStatus {
    ComponentState state = ComponentState::Missing;
    std::string directory;  // 组件目录
    std::string library;    // 库文件的绝对路径（Installed 时有效）
    std::string detail;     // Corrupt、Outdated 的原因
};

// verify 为真时逐个核对文件的 sha256（慢一些；“重新校验”用）
ComponentStatus componentStatus(const ComponentInfo& info, const std::string& componentsDir, bool verify = false);

// 安装进度（界面线程读取，安装线程写入）
struct InstallProgress {
    enum Stage { Download = 0, Verify = 1, Extract = 2, Done = 3 };
    std::atomic<int> stage{Download};
    std::atomic<std::uint64_t> done{0};   // 已下载的字节数
    std::atomic<std::uint64_t> total{0};  // 压缩包的字节数
    std::atomic<bool> cancel{false};      // 界面置为真即取消
};

struct InstallResult {
    bool ok = false;
    bool networkError = false;  // 下载失败（没有网络、地址不可达）：界面给出警告与手动安装的方法
    bool cancelled = false;
    std::string error;
    std::string library;  // 安装后的库文件
};

// 下载（localArchive 为空时）或使用本地文件，校验，解出清单中的文件。阻塞；界面在后台线程中调用。
InstallResult installComponent(const ComponentInfo& info, const std::string& componentsDir, InstallProgress& progress,
                               const std::string& localArchive = {});

// 删除已安装的组件；失败时返回原因
std::optional<std::string> removeComponent(const ComponentInfo& info, const std::string& componentsDir);

// 手动安装的说明（离线时）：下载地址、大小、sha256 与“从文件安装”的方法
std::string manualInstallHint(const ComponentInfo& info, bool english = false);

}  // namespace terra::library
