// 资产包（M4-08c）：miniz 只在本文件中使用。

#include "terra/library/pack.hpp"

#include <algorithm>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <miniz.h>
#include <nlohmann/json.hpp>

#include "terra/core/sha256.hpp"

namespace terra::library {

namespace {

namespace fs = std::filesystem;
using Json = nlohmann::json;

constexpr const char* kManifest = "manifest.json";
constexpr const char* kFormat = "terra.pack";
constexpr int kVersion = 1;
constexpr std::time_t kFixedTime = 946684800;  // 2000-01-01T00:00:00Z：包内时间戳固定，导出可重复

bool readFile(const fs::path& p, std::string& out) {
    std::ifstream in(p, std::ios::binary);
    if (!in) return false;
    out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    return !in.bad();
}

// 相对路径是否安全：非空，不以 “/” 开头，不含反斜杠、空段、“.” 与 “..”
bool safeRelative(const std::string& path) {
    if (path.empty() || path.front() == '/' || path.find('\\') != std::string::npos || path.find('\0') != std::string::npos) return false;
    std::size_t start = 0;
    while (true) {
        const std::size_t end = path.find('/', start);
        const std::string part = path.substr(start, end == std::string::npos ? std::string::npos : end - start);
        if (part.empty() || part == "." || part == "..") return false;
        if (end == std::string::npos) return true;
        start = end + 1;
    }
}

std::string zipError(mz_zip_archive& zip) { return mz_zip_get_error_string(mz_zip_get_last_error(&zip)); }

PackResult fail(PackResult r, std::string message) {
    r.ok = false;
    r.error = std::move(message);
    return r;
}

}  // namespace

PackResult exportPack(const std::string& directory, const std::string& packFile, bool overwrite) {
    PackResult r;
    std::error_code ec;
    const fs::path dir = fs::absolute(directory, ec).lexically_normal();
    r.directory = dir.string();
    r.name = dir.filename().string();
    if (r.name.empty()) r.name = dir.parent_path().filename().string();
    if (!fs::is_directory(dir, ec)) return fail(r, "不是目录：" + dir.string());
    if (!safeRelative(r.name)) return fail(r, "目录名不能作为包名：" + r.name);
    const fs::path out = fs::absolute(packFile, ec).lexically_normal();
    if (fs::exists(out, ec) && !overwrite) return fail(r, "文件已存在：" + out.string());
    if (out.string().rfind(dir.string() + "/", 0) == 0) return fail(r, "资产包不能放在被打包的目录中：" + out.string());

    std::vector<std::string> files;
    for (auto it = fs::recursive_directory_iterator(dir, fs::directory_options::skip_permission_denied, ec);
         !ec && it != fs::recursive_directory_iterator(); it.increment(ec)) {
        std::error_code e2;
        const std::string name = it->path().filename().string();
        if ((!name.empty() && name[0] == '.') || it->is_symlink(e2)) {  // 不打包隐藏文件与符号链接
            if (it->is_directory(e2)) it.disable_recursion_pending();
            continue;
        }
        if (it->is_regular_file(e2)) files.push_back(it->path().lexically_relative(dir).generic_string());
    }
    if (ec) return fail(r, "读取目录失败：" + ec.message());
    if (files.empty()) return fail(r, "目录中没有文件：" + dir.string());
    std::sort(files.begin(), files.end());

    if (!out.parent_path().empty()) fs::create_directories(out.parent_path(), ec);
    const std::string partial = out.string() + ".partial";
    mz_zip_archive zip{};
    if (!mz_zip_writer_init_file_v2(&zip, partial.c_str(), 0, 0)) return fail(r, "无法创建文件：" + partial + "（" + zipError(zip) + "）");
    Json manifest;
    manifest["format"] = kFormat;
    manifest["version"] = kVersion;
    manifest["name"] = r.name;
    Json list = Json::array();
    std::string error;
    std::time_t stamp = kFixedTime;
    for (const auto& rel : files) {
        std::string data;
        if (!readFile(dir / rel, data)) {
            error = "无法读取：" + (dir / rel).string();
            break;
        }
        list.push_back({{"path", rel}, {"bytes", data.size()}, {"sha256", terra::sha256Hex(data)}});
        const std::string entry = r.name + "/" + rel;
        if (!mz_zip_writer_add_mem_ex_v2(&zip, entry.c_str(), data.data(), data.size(), nullptr, 0, static_cast<mz_uint>(MZ_DEFAULT_LEVEL), 0, 0,
                                         &stamp, nullptr, 0, nullptr, 0)) {
            error = "写入失败：" + entry + "（" + zipError(zip) + "）";
            break;
        }
        ++r.files;
        r.bytes += data.size();
    }
    if (error.empty()) {
        manifest["files"] = std::move(list);
        const std::string text = manifest.dump(2) + "\n";
        if (!mz_zip_writer_add_mem_ex_v2(&zip, kManifest, text.data(), text.size(), nullptr, 0, static_cast<mz_uint>(MZ_DEFAULT_LEVEL), 0, 0,
                                         &stamp, nullptr, 0, nullptr, 0) ||
            !mz_zip_writer_finalize_archive(&zip)) {
            error = "写入失败：" + zipError(zip);
        }
    }
    mz_zip_writer_end(&zip);
    if (!error.empty()) {
        fs::remove(partial, ec);
        return fail(r, error);
    }
    fs::rename(partial, out, ec);
    if (ec) {
        fs::remove(partial, ec);
        return fail(r, "无法改名为：" + out.string());
    }
    r.ok = true;
    return r;
}

PackResult importPack(const std::string& packFile, const std::string& outputRoot, bool overwrite) {
    PackResult r;
    std::error_code ec;
    mz_zip_archive zip{};
    if (!mz_zip_reader_init_file(&zip, packFile.c_str(), 0)) return fail(r, "不是有效的资产包（zip）：" + packFile + "（" + zipError(zip) + "）");
    struct Closer {
        mz_zip_archive* z;
        ~Closer() { mz_zip_reader_end(z); }
    } closer{&zip};

    // 条目表：名字 → 下标
    std::map<std::string, mz_uint> entries;
    const mz_uint count = mz_zip_reader_get_num_files(&zip);
    for (mz_uint i = 0; i < count; ++i) {
        if (mz_zip_reader_is_file_a_directory(&zip, i)) continue;
        std::vector<char> name(mz_zip_reader_get_filename(&zip, i, nullptr, 0) + 1u, '\0');
        mz_zip_reader_get_filename(&zip, i, name.data(), static_cast<mz_uint>(name.size()));
        entries[std::string{name.data()}] = i;
    }
    const auto extract = [&](mz_uint index, std::string& out) {
        std::size_t size = 0;
        void* p = mz_zip_reader_extract_to_heap(&zip, index, &size, 0);
        if (p == nullptr) return false;
        out.assign(static_cast<const char*>(p), size);
        mz_free(p);
        return true;
    };
    const auto m = entries.find(kManifest);
    if (m == entries.end()) return fail(r, "资产包中没有 manifest.json：" + packFile);
    std::string text;
    if (!extract(m->second, text)) return fail(r, "无法读取 manifest.json（" + zipError(zip) + "）");
    const Json manifest = Json::parse(text, nullptr, false);
    if (manifest.is_discarded() || !manifest.is_object()) return fail(r, "manifest.json 不是有效的 JSON 对象");
    if (!manifest.contains("format") || manifest["format"] != kFormat) return fail(r, "不是 Terra 资产包（format 应为 “terra.pack”）");
    if (!manifest.contains("version") || !manifest["version"].is_number_integer()) return fail(r, "manifest.json 缺少 version");
    if (manifest["version"].get<long long>() > kVersion) {
        return fail(r, "资产包版本 " + std::to_string(manifest["version"].get<long long>()) + " 高于本程序支持的版本，请更新程序");
    }
    if (!manifest.contains("name") || !manifest["name"].is_string() || !safeRelative(manifest["name"].get<std::string>()) ||
        manifest["name"].get<std::string>().find('/') != std::string::npos) {
        return fail(r, "manifest.json 中的 name 无效");
    }
    r.name = manifest["name"].get<std::string>();
    if (!manifest.contains("files") || !manifest["files"].is_array() || manifest["files"].empty()) return fail(r, "manifest.json 中没有文件列表");

    struct Item {
        std::string path;
        std::uint64_t bytes = 0;
        std::string sha256;
        mz_uint index = 0;
    };
    std::vector<Item> items;
    std::map<std::string, bool> listed;
    for (const auto& f : manifest["files"]) {
        if (!f.is_object() || !f.contains("path") || !f["path"].is_string() || !f.contains("bytes") || !f["bytes"].is_number_unsigned() ||
            !f.contains("sha256") || !f["sha256"].is_string()) {
            return fail(r, "manifest.json 的文件列表格式不对");
        }
        Item item;
        item.path = f["path"].get<std::string>();
        item.bytes = f["bytes"].get<std::uint64_t>();
        item.sha256 = f["sha256"].get<std::string>();
        if (!safeRelative(item.path)) return fail(r, "资产包中有不安全的路径：" + item.path);
        const auto e = entries.find(r.name + "/" + item.path);
        if (e == entries.end()) return fail(r, "资产包中缺少文件：" + item.path);
        if (listed.count(e->first)) return fail(r, "manifest.json 中有重复的文件：" + item.path);
        listed[e->first] = true;
        item.index = e->second;
        items.push_back(std::move(item));
    }
    for (const auto& entry : entries) {
        const std::string& name = entry.first;
        if (name != kManifest && listed.count(name) == 0) return fail(r, "资产包中有 manifest.json 之外的条目：" + name);
    }

    const fs::path root = fs::absolute(outputRoot, ec).lexically_normal();
    const fs::path dest = root / r.name;
    r.directory = dest.string();
    if (fs::exists(dest, ec) && !overwrite) return fail(r, "目录已存在：" + dest.string() + "（覆盖请选择“覆盖”）");
    const fs::path temp = root / ("." + r.name + ".unpack");
    fs::remove_all(temp, ec);
    fs::create_directories(temp, ec);
    if (ec) return fail(r, "无法创建目录：" + temp.string() + "（" + ec.message() + "）");
    std::string error;
    for (const auto& item : items) {
        std::string data;
        if (item.bytes > 0 && !extract(item.index, data)) {  // 空文件不解压（miniz 对 0 字节的条目分配失败）
            error = "解压失败：" + item.path + "（" + zipError(zip) + "）";
            break;
        }
        if (data.size() != item.bytes || terra::sha256Hex(data) != item.sha256) {
            error = "文件与 manifest.json 不一致（大小或 sha256）：" + item.path;
            break;
        }
        const fs::path target = temp / item.path;
        fs::create_directories(target.parent_path(), ec);
        std::ofstream outFile(target, std::ios::binary);
        outFile.write(data.data(), static_cast<std::streamsize>(data.size()));
        if (!outFile) {
            error = "无法写入：" + target.string();
            break;
        }
        ++r.files;
        r.bytes += data.size();
    }
    if (error.empty() && fs::exists(dest, ec)) {
        fs::remove_all(dest, ec);
        if (ec) error = "无法替换已有目录：" + dest.string();
    }
    if (error.empty()) {
        fs::rename(temp, dest, ec);
        if (ec) error = "无法改名为：" + dest.string() + "（" + ec.message() + "）";
    }
    if (!error.empty()) {
        fs::remove_all(temp, ec);
        return fail(r, error);
    }
    r.ok = true;
    return r;
}

}  // namespace terra::library
