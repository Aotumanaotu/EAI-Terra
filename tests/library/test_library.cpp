// 资源库（M3-07b）：SQLite 中的实验记录索引。
//   - 新建库：表与版本号（user_version = 1）；
//   - 加入记录：摘要字段正确；同一路径再次加入为更新（不重复）；没有 summary 的记录标为 incomplete；
//   - 列出：按创建时间从新到旧；按场景筛选；按路径查找；
//   - 文件删除后 removeMissing 删除条目；rebuild 扫描目录（含子目录），读不出的文件计入错误、不中断；
//   - 库文件损坏：改名备份并新建，给出警告；版本高于本程序：拒绝打开；
//   - 默认路径：$TERRA_LIBRARY 优先，其次 $XDG_DATA_HOME/eai-terra/。

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

#include <sqlite3.h>

#include "terra/library/library.hpp"
#include "terra/simulation/record.hpp"

namespace {

namespace fs = std::filesystem;
namespace lib = terra::library;
namespace sim = terra::simulation;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

// 写一份最小的记录：created 决定排序；finish 为假时没有 summary
void writeRecord(const std::string& path, const std::string& scene, const std::string& created, bool finish = true) {
    sim::RecordHeader h;
    h.terraVersion = "0.0.0";
    h.physicsBackend = "MuJoCo 4.0.1";
    h.platform = "linux x86_64";
    h.created = created;
    h.scenePath = scene;
    h.sceneSha256 = std::string(64, 'b');
    h.motionPath = "/m.terra-motion.json";
    h.motionSha256 = std::string(64, 'c');
    h.startKeyframe = "arm/rest";
    h.timestep = 0.002;
    std::string error;
    auto w = sim::RecordWriter::create(path, h, error);
    check(w != nullptr, "写记录：" + error);
    if (w && finish) w->finish(500, 1.0, "completed");
}

}  // namespace

int main(int argc, char** argv) {
    const std::string dir = argc > 1 ? argv[1] : "library_tmp";
    std::error_code ec;
    fs::remove_all(dir, ec);
    fs::create_directories(dir + "/records/sub", ec);
    const std::string dbPath = dir + "/library.sqlite3";

    std::string error, warning;
    auto library = lib::Library::open(dbPath, error, &warning);
    check(library != nullptr && warning.empty(), "新建库：" + error);
    if (!library) return 1;

    const std::string a = dir + "/records/a.terra-record.jsonl";
    const std::string b = dir + "/records/sub/b.terra-record.jsonl";
    const std::string c = dir + "/records/c.terra-record.jsonl";
    writeRecord(a, "/scenes/arm.terra-scene.json", "2026-11-24T01:00:00Z");
    writeRecord(b, "/scenes/go2.terra-scene.json", "2026-11-24T03:00:00Z");
    writeRecord(c, "/scenes/arm.terra-scene.json", "2026-11-24T02:00:00Z", false);
    check(!library->addRecord(a) && !library->addRecord(b) && !library->addRecord(c), "加入三份记录");
    check(library->addRecord(dir + "/no_such.terra-record.jsonl").has_value(), "不存在的记录：返回错误");

    auto all = library->records();
    check(all.size() == 3, "三条记录，实际 " + std::to_string(all.size()));
    if (all.size() == 3) {
        check(all[0].path == fs::absolute(b).lexically_normal().string() && all[1].created == "2026-11-24T02:00:00Z" &&
                  all[2].created == "2026-11-24T01:00:00Z",
              "按创建时间从新到旧");
        check(all[0].steps && *all[0].steps == 500 && all[0].simulationTime && *all[0].simulationTime == 1.0 &&
                  all[0].reason == "completed" && all[0].startKeyframe == "arm/rest" && all[0].physicsBackend == "MuJoCo 4.0.1" &&
                  all[0].fileSha256.size() == 64 && all[0].bytes > 0 && !all[0].indexedAt.empty(),
              "摘要字段");
        check(!all[1].steps && all[1].reason == "incomplete", "没有 summary 的记录：步数为空，原因 incomplete");
    }
    check(library->records("/scenes/arm.terra-scene.json").size() == 2, "按场景筛选：arm 两条");
    check(library->record(a).has_value() && !library->record(dir + "/x.terra-record.jsonl").has_value(), "按路径查找");

    // 同一路径再次加入：更新而不是重复
    writeRecord(a, "/scenes/arm.terra-scene.json", "2026-11-24T05:00:00Z");
    check(!library->addRecord(a) && library->records().size() == 3 && library->records()[0].path == fs::absolute(a).lexically_normal().string(),
          "再次加入同一路径：更新（仍为三条，a 的创建时间变新后排第一）");

    // 删除文件后清理；重建扫描目录
    fs::remove(c, ec);
    check(library->removeMissing() == 1 && library->records().size() == 2, "removeMissing 删除文件已不存在的条目");
    { std::ofstream(dir + "/records/bad.terra-record.jsonl") << "not a record\n"; }
    const std::string d = dir + "/records/sub/d.terra-record.jsonl";
    writeRecord(d, "/scenes/stretch.terra-scene.json", "2026-11-24T04:00:00Z");
    const auto rebuilt = library->rebuild({dir + "/records"});
    check(rebuilt.added == 3 && rebuilt.errors.size() == 1 && library->records().size() == 3,
          "重建：扫描到 3 份有效记录（含子目录）、1 份读不出");

    // 库文件可以随时删掉重建
    library.reset();
    fs::remove(dbPath, ec);
    library = lib::Library::open(dbPath, error, &warning);
    check(library && library->records().empty(), "删掉库文件后重新打开：空库");
    if (library) check(library->rebuild({dir + "/records"}).added == 3 && library->records().size() == 3, "重新扫描即可恢复索引");
    library.reset();

    // 版本号
    {
        sqlite3* db = nullptr;
        sqlite3_open(dbPath.c_str(), &db);
        sqlite3_stmt* s = nullptr;
        sqlite3_prepare_v2(db, "PRAGMA user_version", -1, &s, nullptr);
        const bool row = sqlite3_step(s) == SQLITE_ROW;
        const auto v = row ? sqlite3_column_int(s, 0) : -1;
        sqlite3_finalize(s);
        check(v == lib::kSchemaVersion, "库的版本号 user_version = 1，实际 " + std::to_string(v));
        sqlite3_exec(db, "PRAGMA user_version = 9", nullptr, nullptr, nullptr);
        sqlite3_close(db);
    }
    error.clear();
    check(!lib::Library::open(dbPath, error) && error.find("高于本程序支持的版本") != std::string::npos, "版本过高：拒绝打开：" + error);

    // 损坏：改名备份并新建
    fs::remove(dbPath, ec);
    { std::ofstream(dbPath, std::ios::binary) << std::string(4096, 'x'); }
    warning.clear();
    library = lib::Library::open(dbPath, error, &warning);
    check(library != nullptr && warning.find("已改名为") != std::string::npos, "库文件损坏：改名备份并新建：" + warning + error);
    bool backup = false;
    for (const auto& e : fs::directory_iterator(dir)) backup = backup || e.path().filename().string().rfind("library.sqlite3.corrupt-", 0) == 0;
    check(backup, "损坏的库文件已备份");
    library.reset();

    // 默认路径
    setenv("TERRA_LIBRARY", (dir + "/env.sqlite3").c_str(), 1);
    check(lib::defaultLibraryPath() == fs::absolute(dir + "/env.sqlite3").lexically_normal().string() &&
              lib::defaultRecordsDirectory() == (fs::absolute(dir).lexically_normal() / "records").string(),
          "默认路径：$TERRA_LIBRARY 优先；记录目录在库文件旁的 records/");
    unsetenv("TERRA_LIBRARY");
    setenv("XDG_DATA_HOME", "/xdg", 1);
    check(lib::defaultLibraryPath() == "/xdg/eai-terra/library.sqlite3", "默认路径：$XDG_DATA_HOME/eai-terra/library.sqlite3");

    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
