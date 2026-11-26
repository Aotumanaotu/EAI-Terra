// 可下载的运行组件（M4-10a）：清单、状态、下载（curl）、校验与安装。miniz 只在本模块的实现中使用。

#include "terra/library/components.hpp"

#include <chrono>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <system_error>
#include <thread>

#include <miniz.h>
#include <nlohmann/json.hpp>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include "terra/core/sha256.hpp"

extern char** environ;

namespace terra::library {

namespace {

namespace fs = std::filesystem;
using Json = nlohmann::json;

constexpr const char* kInstalledFile = "installed.json";

std::string text(const Json& j, const char* key) { return j.contains(key) && j[key].is_string() ? j[key].get<std::string>() : std::string{}; }

bool safeRelative(const std::string& p) {
    if (p.empty() || p.front() == '/' || p.find('\\') != std::string::npos) return false;
    for (const auto& part : fs::path{p}) {
        if (part == ".." || part == ".") return false;
    }
    return true;
}

// 用 curl 下载到 target；下载期间按文件大小更新进度，可取消
InstallResult download(const std::string& url, const fs::path& target, InstallProgress& progress) {
    InstallResult r;
    std::string curl = "curl", fail = "-fsSL", retry = "--retry", retries = "2", timeout = "--connect-timeout", seconds = "15", out = "-o",
                file = target.string(), address = url;
    char* argv[] = {curl.data(), fail.data(), retry.data(), retries.data(), timeout.data(), seconds.data(), out.data(), file.data(), address.data(), nullptr};
    pid_t pid = 0;
    if (posix_spawnp(&pid, "curl", nullptr, nullptr, argv, environ) != 0) {
        r.error = "找不到 curl 命令（sudo apt install curl），或改用“从文件安装”";
        r.networkError = true;
        return r;
    }
    int status = 0;
    for (;;) {
        const pid_t done = waitpid(pid, &status, WNOHANG);
        std::error_code ec;
        const auto size = fs::file_size(target, ec);
        if (!ec) progress.done = static_cast<std::uint64_t>(size);
        if (done == pid) break;
        if (progress.cancel) {
            kill(pid, SIGTERM);
            waitpid(pid, &status, 0);
            r.cancelled = true;
            r.error = "已取消";
            return r;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
        r.networkError = true;
        r.error = "下载失败（curl 退出码 " + std::to_string(WIFEXITED(status) ? WEXITSTATUS(status) : -1) + "）：请检查网络连接";
        return r;
    }
    r.ok = true;
    return r;
}

}  // namespace

const ComponentInfo* ComponentCatalog::find(const std::string& id) const {
    for (const auto& c : components) {
        if (c.id == id) return &c;
    }
    return nullptr;
}

ComponentCatalog readComponentCatalog(const std::string& path) {
    ComponentCatalog out;
    std::ifstream in(path, std::ios::binary);
    const Json j = in ? Json::parse(in, nullptr, false) : Json{};
    if (j.is_discarded() || !j.is_object() || text(j, "format") != "terra.components" || !j.contains("components") || !j["components"].is_array()) {
        out.errors.push_back("组件清单无法读取或格式不对：" + path);
        return out;
    }
    for (const auto& c : j["components"]) {
        ComponentInfo info;
        info.id = text(c, "id");
        if (c.contains("name") && c["name"].is_object()) {
            info.nameChinese = text(c["name"], "zh");
            info.nameEnglish = text(c["name"], "en");
        }
        if (c.contains("purpose") && c["purpose"].is_object()) {
            info.purposeChinese = text(c["purpose"], "zh");
            info.purposeEnglish = text(c["purpose"], "en");
        }
        info.version = text(c, "version");
        info.license = text(c, "license");
        info.source = text(c, "source");
        info.library = text(c, "library");
        if (c.contains("download") && c["download"].is_object()) {
            info.url = text(c["download"], "url");
            info.sha256 = text(c["download"], "sha256");
            if (c["download"].contains("size") && c["download"]["size"].is_number_unsigned()) info.size = c["download"]["size"].get<std::uint64_t>();
        }
        bool ok = !info.id.empty() && !info.version.empty() && !info.url.empty() && info.sha256.size() == 64;
        if (c.contains("files") && c["files"].is_array()) {
            for (const auto& f : c["files"]) {
                ComponentFile file{text(f, "from"), text(f, "to"), text(f, "sha256")};
                ok = ok && !file.from.empty() && safeRelative(file.to) && file.sha256.size() == 64;
                info.files.push_back(file);
            }
        }
        ok = ok && !info.files.empty() && safeRelative(info.library);
        if (!ok) {
            out.errors.push_back("组件“" + info.id + "”的清单项不完整（需要 id、version、download.url / sha256、files、library）");
            continue;
        }
        out.components.push_back(std::move(info));
    }
    return out;
}

std::string componentsDirectory(const std::string& libraryFile) {
    std::error_code ec;
    return (fs::absolute(libraryFile, ec).lexically_normal().parent_path() / "components").string();
}

const char* componentStateName(ComponentState state) {
    switch (state) {
        case ComponentState::Missing: return "missing";
        case ComponentState::Installed: return "installed";
        case ComponentState::Corrupt: return "corrupt";
        case ComponentState::Outdated: return "outdated";
    }
    return "missing";
}

ComponentStatus componentStatus(const ComponentInfo& info, const std::string& componentsDir, bool verify) {
    ComponentStatus s;
    const fs::path dir = fs::path{componentsDir} / info.id;
    s.directory = dir.string();
    std::error_code ec;
    std::ifstream in(dir / kInstalledFile, std::ios::binary);
    if (!in) return s;
    const Json j = Json::parse(in, nullptr, false);
    if (j.is_discarded() || !j.is_object()) {
        s.state = ComponentState::Corrupt;
        s.detail = "installed.json 无法读取";
        return s;
    }
    if (text(j, "version") != info.version) {
        s.state = ComponentState::Outdated;
        s.detail = "已安装的版本 " + text(j, "version") + "，需要 " + info.version;
        return s;
    }
    for (const auto& f : info.files) {
        const fs::path p = dir / f.to;
        if (!fs::is_regular_file(p, ec)) {
            s.state = ComponentState::Corrupt;
            s.detail = "缺少文件：" + f.to;
            return s;
        }
        if (verify && terra::sha256File(p.string()).value_or("") != f.sha256) {
            s.state = ComponentState::Corrupt;
            s.detail = "文件的 sha256 不一致：" + f.to;
            return s;
        }
    }
    s.state = ComponentState::Installed;
    s.library = (dir / info.library).string();
    return s;
}

InstallResult installComponent(const ComponentInfo& info, const std::string& componentsDir, InstallProgress& progress, const std::string& localArchive) {
    InstallResult r;
    std::error_code ec;
    const fs::path root{componentsDir};
    fs::create_directories(root, ec);
    if (ec) {
        r.error = "无法创建组件目录：" + root.string();
        return r;
    }
    progress.total = info.size;
    progress.done = 0;
    progress.stage = InstallProgress::Download;
    // 1. 压缩包：下载或本地文件
    fs::path archive;
    const fs::path downloaded = root / ("." + info.id + ".download");
    if (!localArchive.empty()) {
        archive = localArchive;
        if (!fs::is_regular_file(archive, ec)) {
            r.error = "文件不存在：" + localArchive;
            return r;
        }
        progress.done = static_cast<std::uint64_t>(fs::file_size(archive, ec));
    } else {
        fs::remove(downloaded, ec);
        if (progress.cancel) {
            r.cancelled = true;
            r.error = "已取消";
            return r;
        }
        auto d = download(info.url, downloaded, progress);
        if (!d.ok) {
            fs::remove(downloaded, ec);
            return d;
        }
        archive = downloaded;
    }
    // 2. 校验
    progress.stage = InstallProgress::Verify;
    const auto digest = terra::sha256File(archive.string());
    if (!digest || *digest != info.sha256) {
        if (localArchive.empty()) fs::remove(downloaded, ec);
        r.error = "压缩包的 sha256 与清单不一致（下载不完整或文件不对）：" + archive.string();
        return r;
    }
    // 3. 解出清单中的文件到临时目录，逐个校验，最后改名
    progress.stage = InstallProgress::Extract;
    const fs::path temp = root / ("." + info.id + ".partial");
    const fs::path target = root / info.id;
    fs::remove_all(temp, ec);
    fs::create_directories(temp, ec);
    mz_zip_archive zip{};
    if (!mz_zip_reader_init_file(&zip, archive.string().c_str(), 0)) {
        r.error = "压缩包无法打开：" + archive.string();
    } else {
        for (const auto& f : info.files) {
            const int index = mz_zip_reader_locate_file(&zip, f.from.c_str(), nullptr, 0);
            std::size_t size = 0;
            void* data = index >= 0 ? mz_zip_reader_extract_to_heap(&zip, static_cast<mz_uint>(index), &size, 0) : nullptr;
            if (data == nullptr) {
                r.error = "压缩包中没有：" + f.from;
                break;
            }
            const std::string content(static_cast<const char*>(data), size);
            mz_free(data);
            if (terra::sha256Hex(content) != f.sha256) {
                r.error = "文件的 sha256 与清单不一致：" + f.from;
                break;
            }
            const fs::path out = temp / f.to;
            fs::create_directories(out.parent_path(), ec);
            std::ofstream file(out, std::ios::binary);
            file.write(content.data(), static_cast<std::streamsize>(content.size()));
            if (!file) {
                r.error = "无法写入：" + out.string();
                break;
            }
            if (progress.cancel) {
                r.cancelled = true;
                r.error = "已取消";
                break;
            }
        }
        mz_zip_reader_end(&zip);
    }
    if (localArchive.empty()) fs::remove(downloaded, ec);
    if (r.error.empty()) {
        Json installed;
        installed["id"] = info.id;
        installed["version"] = info.version;
        installed["archive_sha256"] = info.sha256;
        Json files = Json::array();
        for (const auto& f : info.files) files.push_back({{"path", f.to}, {"sha256", f.sha256}});
        installed["files"] = std::move(files);
        std::ofstream(temp / kInstalledFile, std::ios::binary) << installed.dump(2) << '\n';
        fs::remove_all(target, ec);
        fs::rename(temp, target, ec);
        if (ec) r.error = "无法改名为：" + target.string() + "（" + ec.message() + "）";
    }
    if (!r.error.empty()) {
        fs::remove_all(temp, ec);
        return r;
    }
    progress.stage = InstallProgress::Done;
    r.ok = true;
    r.library = (target / info.library).string();
    return r;
}

std::optional<std::string> removeComponent(const ComponentInfo& info, const std::string& componentsDir) {
    std::error_code ec;
    fs::remove_all(fs::path{componentsDir} / info.id, ec);
    if (ec) return "无法删除：" + (fs::path{componentsDir} / info.id).string() + "（" + ec.message() + "）";
    return std::nullopt;
}

std::string manualInstallHint(const ComponentInfo& info, bool english) {
    char size[32];
    std::snprintf(size, sizeof size, "%.0f MB", static_cast<double>(info.size) / 1e6);
    if (english) {
        return "Download on a machine with network access:\n  " + info.url + "\n  (" + size + ", sha256 " + info.sha256 +
               ")\nthen install it with “Install from file…” (Preferences → Components), or run\n  terra_headless --install-component " + info.id +
               " --from <file>";
    }
    return "请在有网络的机器上下载：\n  " + info.url + "\n  （" + size + "，sha256 " + info.sha256 +
           "）\n再用“从文件安装…”（偏好设置 → 组件）选择这个文件，或运行\n  terra_headless --install-component " + info.id + " --from <文件>";
}

}  // namespace terra::library
