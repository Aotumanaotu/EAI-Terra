// 资产包（M4-08c，library.pack）：
//   - 导出再导入：文件逐字节相同；同一目录两次导出的包逐字节相同；不打包隐藏文件；
//   - 目标已存在：不覆盖时报错，覆盖时替换；
//   - 拒绝：不是 zip、没有 manifest、路径含 “..” 或为绝对路径、manifest 之外的条目、sha256 不一致；失败时不留下目录。
// 用法：test_pack <临时目录>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <string>
#include <vector>

#include <miniz.h>

#include "terra/core/sha256.hpp"
#include "terra/library/pack.hpp"

namespace {

namespace fs = std::filesystem;
namespace lib = terra::library;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

void write(const fs::path& p, const std::string& text) {
    fs::create_directories(p.parent_path());
    std::ofstream(p, std::ios::binary) << text;
}

std::string read(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

std::map<std::string, std::string> snapshot(const fs::path& root) {
    std::map<std::string, std::string> out;
    for (const auto& e : fs::recursive_directory_iterator(root)) {
        if (e.is_regular_file()) out[fs::relative(e.path(), root).generic_string()] = read(e.path());
    }
    return out;
}

// 手工构造的包：entries 为 名字 → 内容
void makeZip(const fs::path& file, const std::vector<std::pair<std::string, std::string>>& entries) {
    mz_zip_archive zip{};
    mz_zip_writer_init_file_v2(&zip, file.string().c_str(), 0, 0);
    for (const auto& [name, data] : entries) {
        mz_zip_writer_add_mem(&zip, name.c_str(), data.data(), data.size(), static_cast<mz_uint>(MZ_DEFAULT_LEVEL));
    }
    mz_zip_writer_finalize_archive(&zip);
    mz_zip_writer_end(&zip);
}

std::string manifest(const std::string& path, const std::string& data) {
    return R"({"format": "terra.pack", "version": 1, "name": "evil", "files": [{"path": ")" + path + R"(", "bytes": )" +
           std::to_string(data.size()) + R"(, "sha256": ")" + terra::sha256Hex(data) + R"("}]})";
}

}  // namespace

int main(int argc, char** argv) {
    const fs::path dir = fs::absolute(argc > 1 ? argv[1] : "pack_tmp").lexically_normal();
    std::error_code ec;
    fs::remove_all(dir, ec);
    const fs::path src = dir / "src" / "robot1";
    write(src / "robot1.xml", "<mujoco/>\n");
    write(src / "robot1.terra-scene.json", "{}\n");
    write(src / "meshes" / "base.obj", std::string(200000, 'v'));  // 可压缩的大文件
    write(src / "meshes" / "empty.obj", "");
    write(src / "LICENSE", "BSD\n");
    write(src / ".cache" / "tmp.txt", "x");  // 隐藏目录不打包

    const fs::path pack = dir / "out" / "robot1.terra-pack";
    auto r = lib::exportPack(src.string(), pack.string());
    check(r.ok && r.files == 5 && r.name == "robot1", "导出：5 个文件（不含隐藏目录）：" + r.error);
    check(fs::exists(pack) && fs::file_size(pack) < 50000, "导出的包已压缩");
    const auto first = read(pack);
    check(!lib::exportPack(src.string(), pack.string()).ok, "包已存在：不覆盖时报错");
    r = lib::exportPack(src.string(), pack.string(), true);
    check(r.ok && read(pack) == first, "两次导出逐字节相同");
    check(!lib::exportPack(src.string(), (src / "self.terra-pack").string()).ok, "包不能放在被打包的目录中");

    const fs::path unpack = dir / "unpack";
    r = lib::importPack(pack.string(), unpack.string());
    check(r.ok && r.files == 5 && r.directory == (unpack / "robot1").string(), "导入：" + r.error);
    auto expected = snapshot(src);
    expected.erase(".cache/tmp.txt");
    check(snapshot(unpack / "robot1") == expected, "导入后文件逐字节相同");
    check(!fs::exists(unpack / ".robot1.unpack"), "临时目录已删除");
    r = lib::importPack(pack.string(), unpack.string());
    check(!r.ok && r.error.find("已存在") != std::string::npos, "目录已存在：不覆盖时报错：" + r.error);
    write(unpack / "robot1" / "stale.txt", "old");
    r = lib::importPack(pack.string(), unpack.string(), true);
    check(r.ok && !fs::exists(unpack / "robot1" / "stale.txt"), "覆盖：替换整个目录");

    // 拒绝
    const fs::path bad = dir / "bad";
    const auto rejected = [&](const std::string& what, const fs::path& file, const std::string& part) {
        const auto res = lib::importPack(file.string(), bad.string());
        check(!res.ok && res.error.find(part) != std::string::npos, what + "：" + res.error);
        check(!fs::exists(bad / "evil") && !fs::exists(bad / ".evil.unpack") && !fs::exists(dir / "escaped.txt"), what + "：没有留下文件");
    };
    write(dir / "notzip.terra-pack", "hello");
    rejected("不是 zip", dir / "notzip.terra-pack", "不是有效的资产包");
    makeZip(dir / "nomanifest.terra-pack", {{"evil/a.txt", "a"}});
    rejected("没有 manifest", dir / "nomanifest.terra-pack", "没有 manifest.json");
    makeZip(dir / "dotdot.terra-pack", {{"manifest.json", manifest("../../escaped.txt", "x")}, {"evil/../../escaped.txt", "x"}});
    rejected("路径含 ..", dir / "dotdot.terra-pack", "不安全的路径");
    makeZip(dir / "absolute.terra-pack", {{"manifest.json", manifest("/tmp/escaped.txt", "x")}, {"evil//tmp/escaped.txt", "x"}});
    rejected("绝对路径", dir / "absolute.terra-pack", "不安全的路径");
    makeZip(dir / "extra.terra-pack", {{"manifest.json", manifest("a.txt", "a")}, {"evil/a.txt", "a"}, {"../escaped.txt", "x"}});
    rejected("manifest 之外的条目", dir / "extra.terra-pack", "之外的条目");
    makeZip(dir / "sha.terra-pack", {{"manifest.json", manifest("a.txt", "a")}, {"evil/a.txt", "b"}});
    rejected("sha256 不一致", dir / "sha.terra-pack", "不一致");
    makeZip(dir / "format.terra-pack", {{"manifest.json", R"({"format": "other", "version": 1})"}});
    rejected("格式不对", dir / "format.terra-pack", "不是 Terra 资产包");

    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
