// 可下载的运行组件（M4-10a，library.components）：用测试中生成的小 zip 代替真实的压缩包（file:// 地址，不需要网络）。
//   - 清单的读取与校验（缺字段、不安全的路径）；仓库自带的清单 assets/components/components.json 可以读取；
//   - 从地址安装（curl 读 file://）：进度达到压缩包大小，解出的文件与清单一致，状态为已安装，库文件路径正确；
//   - 从本地文件安装；
//   - 压缩包的 sha256 不对、文件的 sha256 不对：拒绝，不留下组件目录；地址不可达：标为网络错误；取消；
//   - 重新校验发现被改动的文件；版本不同为“需要更新”；删除。
// 用法：test_components <临时目录>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include <miniz.h>

#include "terra/core/sha256.hpp"
#include "terra/library/components.hpp"

#ifndef TERRA_ASSETS_DIR
#error "需要 TERRA_ASSETS_DIR"
#endif

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

std::string read(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

}  // namespace

int main(int argc, char** argv) {
    const fs::path dir = fs::absolute(argc > 1 ? argv[1] : "components_tmp").lexically_normal();
    std::error_code ec;
    fs::remove_all(dir, ec);
    fs::create_directories(dir, ec);

    // 仓库自带的清单
    const auto real = lib::readComponentCatalog(TERRA_ASSETS_DIR "/components/components.json");
    const auto* ort = real.find("onnxruntime-cpu");
    check(real.errors.empty() && ort && ort->version == "1.32.0" && ort->size == 184391411 && ort->library == "lib/libonnxruntime.so" &&
              ort->url.rfind("https://api.nuget.org/", 0) == 0,
          "仓库自带的清单：ONNX Runtime 1.32.0（NuGet）");

    // 测试用的压缩包
    const std::string libContent = "ELF dummy library";
    const std::string license = "MIT\n";
    const fs::path archive = dir / "pkg.zip";
    {
        mz_zip_archive zip{};
        mz_zip_writer_init_file_v2(&zip, archive.string().c_str(), 0, 0);
        mz_zip_writer_add_mem(&zip, "runtimes/linux-x64/native/libdummy.so", libContent.data(), libContent.size(), static_cast<mz_uint>(MZ_DEFAULT_LEVEL));
        mz_zip_writer_add_mem(&zip, "LICENSE", license.data(), license.size(), static_cast<mz_uint>(MZ_DEFAULT_LEVEL));
        mz_zip_writer_finalize_archive(&zip);
        mz_zip_writer_end(&zip);
    }
    const std::string archiveSha = terra::sha256File(archive.string()).value_or("");
    const auto archiveSize = fs::file_size(archive);
    const auto catalogText = [&](const std::string& url, const std::string& sha, const std::string& libSha, const std::string& version,
                                 const std::string& to = "lib/libdummy.so") {
        return R"({"format": "terra.components", "version": 1, "components": [{"id": "dummy", "name": {"zh": "测试", "en": "Test"},
            "version": ")" + version + R"(", "license": "MIT", "download": {"url": ")" + url + R"(", "size": )" + std::to_string(archiveSize) +
               R"(, "sha256": ")" + sha + R"("},
            "files": [{"from": "runtimes/linux-x64/native/libdummy.so", "to": ")" + to + R"(", "sha256": ")" + libSha + R"("},
                      {"from": "LICENSE", "to": "LICENSE", "sha256": ")" + terra::sha256Hex(license) + R"("}],
            "library": "lib/libdummy.so"}]})";
    };
    const auto catalog = [&](const std::string& textContent) {
        const fs::path p = dir / "catalog.json";
        std::ofstream(p, std::ios::binary) << textContent;
        return lib::readComponentCatalog(p.string());
    };
    const std::string url = "file://" + archive.string();
    const std::string components = (dir / "components").string();

    // 清单的校验
    check(catalog(catalogText(url, archiveSha, terra::sha256Hex(libContent), "1.0", "../escape.so")).components.empty(), "不安全的路径：清单项被拒绝");
    check(catalog(catalogText(url, "abc", terra::sha256Hex(libContent), "1.0")).components.empty(), "sha256 不完整：清单项被拒绝");

    // 从地址安装
    auto good = catalog(catalogText(url, archiveSha, terra::sha256Hex(libContent), "1.0"));
    check(good.errors.empty() && good.components.size() == 1, "测试清单");
    if (good.components.empty()) return 1;
    const auto& info = good.components[0];
    check(lib::componentStatus(info, components).state == lib::ComponentState::Missing, "安装前：没有");
    lib::InstallProgress progress;
    auto r = lib::installComponent(info, components, progress);
    check(r.ok && progress.done == archiveSize && progress.total == archiveSize && progress.stage == lib::InstallProgress::Done, "从地址安装：" + r.error);
    auto status = lib::componentStatus(info, components, true);
    check(status.state == lib::ComponentState::Installed && status.library == (dir / "components" / "dummy" / "lib" / "libdummy.so").string() &&
              read(status.library) == libContent,
          "已安装，库文件内容正确");
    check(!fs::exists(dir / "components" / ".dummy.partial") && !fs::exists(dir / "components" / ".dummy.download"), "没有留下临时文件");

    // 重新校验与版本
    std::ofstream(status.library, std::ios::binary) << "tampered";
    check(lib::componentStatus(info, components, true).state == lib::ComponentState::Corrupt, "文件被改动：重新校验发现");
    auto newer = catalog(catalogText(url, archiveSha, terra::sha256Hex(libContent), "2.0"));
    check(lib::componentStatus(newer.components[0], components).state == lib::ComponentState::Outdated, "版本不同：需要更新");

    // 从本地文件安装（覆盖损坏的）
    lib::InstallProgress local;
    r = lib::installComponent(info, components, local, archive.string());
    check(r.ok && lib::componentStatus(info, components, true).state == lib::ComponentState::Installed, "从本地文件安装：" + r.error);

    // 错误
    check(!lib::removeComponent(info, components) && lib::componentStatus(info, components).state == lib::ComponentState::Missing, "删除");
    auto badArchive = catalog(catalogText(url, std::string(64, 'a'), terra::sha256Hex(libContent), "1.0"));
    lib::InstallProgress p1;
    r = lib::installComponent(badArchive.components[0], components, p1);
    check(!r.ok && !r.networkError && r.error.find("sha256") != std::string::npos && !fs::exists(dir / "components" / "dummy"), "压缩包的 sha256 不对：拒绝");
    auto badFile = catalog(catalogText(url, archiveSha, std::string(64, 'b'), "1.0"));
    lib::InstallProgress p2;
    r = lib::installComponent(badFile.components[0], components, p2);
    check(!r.ok && r.error.find("sha256") != std::string::npos && !fs::exists(dir / "components" / "dummy") && !fs::exists(dir / "components" / ".dummy.partial"),
          "文件的 sha256 不对：拒绝，不留下组件目录");
    auto unreachable = catalog(catalogText("file://" + (dir / "no_such.zip").string(), archiveSha, terra::sha256Hex(libContent), "1.0"));
    lib::InstallProgress p3;
    r = lib::installComponent(unreachable.components[0], components, p3);
    check(!r.ok && r.networkError, "地址不可达：网络错误（界面给出警告与手动安装的方法）");
    check(lib::manualInstallHint(unreachable.components[0]).find("从文件安装") != std::string::npos, "手动安装的说明");
    lib::InstallProgress p4;
    p4.cancel = true;
    r = lib::installComponent(info, components, p4);
    check(!r.ok && r.cancelled, "取消");

    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
