// 资源库的资产（M4-08c，library.assets）：
//   - 扫描：场景、动作、环境贴图登记；标题取场景的 title；标签（传感器、模型缺失、文件有错误）；隐藏目录不扫描；
//   - 由场景推出机器人：只算含关节的 MJCF（地面不算）；导入目录（有 import.terra-import.json）的来源、导入目录与许可文件；
//   - 增量：第二次扫描只重新解析依赖缺失文件的条目；修改后重新解析；删除后标为缺失（不删除登记），收藏保留；
//   - 收藏、隐藏、搜索；内存中的库；默认扫描范围；
//   - 版本 1 的库自动升级到 2，原有记录保留；
//   - 复制资源库到新位置：路径改为新位置，目标已有时拒绝。
// 用法：test_library_assets <临时目录>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <sqlite3.h>

#include "terra/library/library.hpp"
#include "terra/simulation/record.hpp"

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

// 修改文件内容并把修改时间推后 2 秒（文件系统的时间精度可能较粗）
void rewrite(const fs::path& p, const std::string& text) {
    const auto before = fs::last_write_time(p);
    write(p, text);
    fs::last_write_time(p, before + std::chrono::seconds(2));
}

const char* kRobot = R"(<mujoco><worldbody><body name="b"><joint name="j" type="hinge"/><geom size="0.1"/></body></worldbody></mujoco>)";
const char* kFloor = R"(<mujoco><worldbody><geom type="plane" size="5 5 0.1"/></worldbody></mujoco>)";

std::string sceneText(const std::string& title, const std::string& models, const std::string& extra = {}) {
    return R"({"format": "terra.scene", "version": 1, "name": "说明 )" + title + R"(", "title": ")" + title + R"(", "models": )" + models + extra + "}";
}

const lib::AssetEntry* find(const std::vector<lib::AssetEntry>& list, const fs::path& p) {
    for (const auto& a : list) {
        if (a.path == p.string()) return &a;
    }
    return nullptr;
}

bool hasTag(const lib::AssetEntry* a, const std::string& tag) {
    if (!a) return false;
    for (const auto& t : a->tags) {
        if (t == tag) return true;
    }
    return false;
}

}  // namespace

int main(int argc, char** argv) {
    const fs::path dir = fs::absolute(argc > 1 ? argv[1] : "library_assets_tmp").lexically_normal();
    std::error_code ec;
    fs::remove_all(dir, ec);
    const fs::path builtin = dir / "builtin";
    const fs::path libDir = dir / "lib";
    const fs::path scenes = builtin / "scenes";
    write(scenes / "robot.xml", kRobot);
    write(scenes / "floor.xml", kFloor);
    write(scenes / "a.terra-scene.json",
          sceneText("A 场景", R"([{"name": "bot", "mjcf": "robot.xml"}, {"name": "floor", "mjcf": "floor.xml"}])",
                    R"(, "sensors": [{"type": "imu", "name": "imu0", "entity": "bot/b"}])"));
    write(scenes / "b.terra-scene.json", sceneText("B 场景", R"([{"name": "later", "mjcf": "later.xml"}])"));
    write(scenes / "bad.terra-scene.json", R"({"format": "terra.scene", "version": 1, "models": 3})");
    write(scenes / ".hidden" / "h.terra-scene.json", sceneText("隐藏", R"([{"name": "bot", "mjcf": "../robot.xml"}])"));
    write(builtin / "motions" / "m.terra-motion.json",
          R"({"format": "terra.motion", "version": 1, "name": "摆动", "scene": "../scenes/a.terra-scene.json",
              "poses": [{"time": 0.0, "targets": {"bot/j": 0.0}}, {"time": 1.0, "targets": {"bot/j": 0.5}}]})");
    write(builtin / "hdri" / "sky.hdr", "#?RADIANCE\n");
    const fs::path imported = libDir / "assets" / "robot1";
    write(imported / "robot1.xml", kRobot);
    write(imported / "robot1.terra-scene.json", sceneText("导入的机器人", R"([{"name": "robot1", "mjcf": "robot1.xml"}])"));
    write(imported / "import.terra-import.json", "{}");
    write(imported / "LICENSE", "BSD\n");

    const std::string dbPath = (libDir / "library.sqlite3").string();
    std::string error, warning;
    auto library = lib::Library::open(dbPath, error, &warning);
    check(library != nullptr, "打开库：" + error);
    if (!library) return 1;
    const std::vector<lib::ScanRoot> roots{{(builtin / "scenes").string(), lib::AssetSource::Builtin},
                                           {(builtin / "motions").string(), lib::AssetSource::Builtin},
                                           {(builtin / "hdri").string(), lib::AssetSource::Builtin},
                                           {(libDir / "assets").string(), lib::AssetSource::Imported}};

    // 第一次扫描
    auto r = library->scanAssets(roots);
    std::printf("INFO 第一次扫描：%zu 个文件，解析 %zu，跳过 %zu，机器人 %zu，错误 %zu\n", r.files, r.updated, r.unchanged, r.robots, r.errors.size());
    check(r.files == 6 && r.updated == 6 && r.unchanged == 0, "第一次扫描：6 个文件（隐藏目录不扫描）全部解析");
    check(r.errors.size() == 1 && r.errors[0].find("bad.terra-scene.json") != std::string::npos, "有错误的场景计入错误");
    auto sceneList = library->assets(lib::AssetKind::Scene);
    check(sceneList.size() == 4, "4 个场景（含有错误的），实际 " + std::to_string(sceneList.size()));
    const auto* a = find(sceneList, scenes / "a.terra-scene.json");
    check(a && a->title == "A 场景" && a->description == "说明 A 场景" && a->source == lib::AssetSource::Builtin && a->sha256.size() == 64,
          "场景 A：标题取 title，说明取 name，来源内置，有 sha256");
    check(a && a->related.size() == 1 && a->related[0] == (scenes / "robot.xml").string(), "场景 A 关联机器人 robot.xml（地面不算）");
    check(hasTag(a, "imu") && !hasTag(a, "models-missing"), "场景 A 的标签：imu");
    check(hasTag(find(sceneList, scenes / "b.terra-scene.json"), "models-missing"), "场景 B：模型文件缺失");
    check(hasTag(find(sceneList, scenes / "bad.terra-scene.json"), "invalid"), "有错误的场景带 invalid 标签");
    const auto* imp = find(sceneList, imported / "robot1.terra-scene.json");
    check(imp && imp->source == lib::AssetSource::Imported && imp->importDirectory == imported.string() &&
              imp->license == (imported / "LICENSE").string(),
          "导入的场景：来源、导入目录与许可文件");
    const auto motions = library->assets(lib::AssetKind::Motion);
    check(motions.size() == 1 && motions[0].title == "摆动" && motions[0].related.size() == 1 &&
              motions[0].related[0] == (scenes / "a.terra-scene.json").string(),
          "动作：标题与推荐场景");
    check(library->assets(lib::AssetKind::Environment).size() == 1, "环境贴图");
    auto robots = library->assets(lib::AssetKind::Robot);
    check(robots.size() == 2 && find(robots, scenes / "robot.xml") && find(robots, imported / "robot1.xml"),
          "两个机器人（缺失的 later.xml 不列出）");
    if (const auto* rb = find(robots, imported / "robot1.xml")) {
        check(rb->source == lib::AssetSource::Imported && rb->importDirectory == imported.string() && rb->related.size() == 1, "导入的机器人");
    }
    const auto withMissing = library->assets(lib::AssetKind::Robot, true);
    const auto* later = find(withMissing, scenes / "later.xml");
    check(later && later->missing && later->related.size() == 1, "缺失的模型也登记关联，标为缺失");

    // 第二次扫描：只重新解析依赖缺失文件的场景 B
    r = library->scanAssets(roots);
    check(r.files == 6 && r.updated == 1 && r.unchanged == 5, "第二次扫描：只重新解析 1 个（实际 " + std::to_string(r.updated) + "）");

    // 收藏与排序
    check(library->setFavorite(lib::AssetKind::Scene, (scenes / "b.terra-scene.json").string(), true), "收藏场景 B");
    check(!library->setFavorite(lib::AssetKind::Scene, (scenes / "nope.terra-scene.json").string(), true), "不存在的条目：返回 false");
    sceneList = library->assets(lib::AssetKind::Scene);
    check(!sceneList.empty() && sceneList[0].path == (scenes / "b.terra-scene.json").string() && sceneList[0].favorite, "收藏的排在前面");

    // 补上缺失的模型、修改场景 A
    write(scenes / "later.xml", kRobot);
    rewrite(scenes / "a.terra-scene.json", sceneText("A 改名", R"([{"name": "bot", "mjcf": "robot.xml"}])"));
    r = library->scanAssets(roots);
    sceneList = library->assets(lib::AssetKind::Scene);
    a = find(sceneList, scenes / "a.terra-scene.json");
    check(r.updated == 2 && a && a->title == "A 改名" && !hasTag(a, "imu"), "修改后重新解析");
    check(!hasTag(find(sceneList, scenes / "b.terra-scene.json"), "models-missing"), "模型补上后不再缺失");
    robots = library->assets(lib::AssetKind::Robot);
    check(robots.size() == 3 && find(robots, scenes / "later.xml"), "补上的模型成为机器人");

    // 删除：标为缺失，不删除登记，收藏保留
    fs::remove(scenes / "b.terra-scene.json");
    r = library->scanAssets(roots);
    check(r.missing >= 1, "删除后标为缺失（" + std::to_string(r.missing) + "）");
    check(!find(library->assets(lib::AssetKind::Scene), scenes / "b.terra-scene.json"), "缺失的不列出");
    const auto old = library->asset(lib::AssetKind::Scene, (scenes / "b.terra-scene.json").string());
    check(old && old->missing && old->favorite, "缺失的条目保留（含收藏）");
    check(!find(library->assets(lib::AssetKind::Robot), scenes / "later.xml"), "不再被场景用到的机器人标为缺失");
    write(scenes / "b.terra-scene.json", sceneText("B 场景", R"([{"name": "later", "mjcf": "later.xml"}])"));
    library->scanAssets(roots);
    const auto back = library->asset(lib::AssetKind::Scene, (scenes / "b.terra-scene.json").string());
    check(back && !back->missing && back->favorite, "文件恢复后不再缺失，收藏还在");

    // 隐藏
    check(library->setHidden(lib::AssetKind::Scene, (scenes / "bad.terra-scene.json").string(), true), "隐藏");
    const auto hidden = library->asset(lib::AssetKind::Scene, (scenes / "bad.terra-scene.json").string());
    check(hidden && hidden->hidden, "隐藏标记");

    // 搜索
    if (const auto e = library->asset(lib::AssetKind::Scene, (imported / "robot1.terra-scene.json").string())) {
        check(lib::assetMatches(*e, "") && lib::assetMatches(*e, "导入") && lib::assetMatches(*e, "ROBOT1  导入") && !lib::assetMatches(*e, "robot1 zzz"),
              "搜索：每个词都要出现，不区分大小写");
    }

    // 显示配置（M4-08d）：登记为 display，标题取 name，关联场景
    write(libDir / "displays" / "a" / "nav.terviz",
          R"({"format": "terra.viz", "version": 1, "name": "导航调试", "scene": "../../../builtin/scenes/a.terra-scene.json",
              "windows": [{"type": "tf"}, {"type": "message", "topic": "/odom"}]})");
    {
        auto withDisplays = roots;
        withDisplays.push_back({(libDir / "displays").string(), lib::AssetSource::User});
        library->scanAssets(withDisplays);
        const auto displays = library->assets(lib::AssetKind::Display);
        check(displays.size() == 1 && displays[0].title == "导航调试" && displays[0].source == lib::AssetSource::User &&
                  displays[0].related.size() == 1 && displays[0].related[0] == (scenes / "a.terra-scene.json").string() &&
                  displays[0].description == "2 个窗口",
              "显示配置：登记、标题、关联场景");
    }
    // 策略（M4-10b）：登记为 policy，标题取 name，关联推荐场景与模型；模型不在时带 model-missing
    write(libDir / "policies" / "walk" / "walk.terra-policy.json",
          R"({"format": "terra.policy", "version": 1, "name": "行走策略", "scene": "../../../builtin/scenes/a.terra-scene.json",
              "model": {"file": "policy.onnx", "backend": "onnxruntime-cpu"}, "source": {"format": "supermimic_moe_velocity_policy_v1"},
              "action": {"type": "positionPd", "joints": ["a/j"], "kp": 1, "kd": 0}, "observation": [{"term": "command"}]})");
    {
        auto withPolicies = roots;
        withPolicies.push_back({(libDir / "policies").string(), lib::AssetSource::Imported});
        library->scanAssets(withPolicies);
        const auto policies = library->assets(lib::AssetKind::Policy);
        check(policies.size() == 1 && policies[0].title == "行走策略" && policies[0].related.size() == 2 &&
                  policies[0].related[0] == (scenes / "a.terra-scene.json").string() &&
                  policies[0].related[1] == (libDir / "policies" / "walk" / "policy.onnx").string() &&
                  policies[0].description == "supermimic_moe_velocity_policy_v1 · onnxruntime-cpu" &&
                  std::find(policies[0].tags.begin(), policies[0].tags.end(), "model-missing") != policies[0].tags.end(),
              "策略：登记、标题、关联场景与模型、模型不在时的标签");
        check(lib::assetKindFromName("policy") == lib::AssetKind::Policy, "assetKindFromName(policy)");
    }

    // 默认扫描范围
    const auto defaults = lib::defaultScanRoots("/a", "/x/library.sqlite3");
    check(defaults.size() == 9 && defaults[0].directory == "/a/scenes" && defaults[4].directory == "/a/elements" && defaults[5].directory == "/x/assets" &&
              defaults[5].source == lib::AssetSource::Imported && defaults[6].directory == "/x/displays" && defaults[6].source == lib::AssetSource::User &&
              defaults[7].directory == "/x/policies" && defaults[7].source == lib::AssetSource::Imported && defaults[8].directory == "/x/elements" &&
              defaults[8].source == lib::AssetSource::Imported,
          "默认扫描范围：内置 5 个目录（M5-02a 加 elements/）与资源库目录下的 assets/、displays/、policies/（M4-10b）、elements/（M5-02a）");
    check(lib::defaultScanRoots("/a", ":memory:").size() == 5, "内存中的库：只扫描内置目录");

    // 内存中的库
    {
        const auto cwdBefore = fs::exists(":memory:");
        auto memory = lib::Library::open(":memory:", error);
        check(memory && memory->inMemory() && memory->path() == ":memory:", "内存中的库");
        if (memory) check(memory->scanAssets(roots).files == 6 && memory->assets(lib::AssetKind::Scene).size() == 4, "内存中的库可以扫描");
        check(fs::exists(":memory:") == cwdBefore, "不在当前目录建文件");
    }

    // 复制到新位置
    {
        terra::simulation::RecordHeader h;
        h.created = "2026-11-25T00:00:00Z";
        h.scenePath = (scenes / "a.terra-scene.json").string();
        h.timestep = 0.002;
        auto w = terra::simulation::RecordWriter::create((libDir / "records" / "x.terra-record.jsonl").string(), h, error);
        if (w) w->finish(10, 0.02, "completed");
        check(!library->addRecord((libDir / "records" / "x.terra-record.jsonl").string()), "加入一份记录");
    }
    library.reset();
    const fs::path newDir = dir / "moved";
    const auto copied = lib::copyLibrary(dbPath, (newDir / "library.sqlite3").string());
    check(!copied, "复制资源库：" + copied.value_or(""));
    {
        auto moved = lib::Library::open((newDir / "library.sqlite3").string(), error);
        check(moved != nullptr, "打开复制后的库");
        if (moved) {
            const auto recs = moved->records();
            check(recs.size() == 1 && recs[0].path == (newDir / "records" / "x.terra-record.jsonl").string() && fs::exists(recs[0].path),
                  "记录的路径改为新位置");
            const auto e = moved->asset(lib::AssetKind::Scene, (newDir / "assets" / "robot1" / "robot1.terra-scene.json").string());
            check(e && e->importDirectory == (newDir / "assets" / "robot1").string() && fs::exists(e->path) &&
                      e->license == (newDir / "assets" / "robot1" / "LICENSE").string(),
                  "导入的资产的路径改为新位置");
            const auto builtinScene = moved->asset(lib::AssetKind::Scene, (scenes / "a.terra-scene.json").string());
            check(builtinScene.has_value(), "内置资产的路径不变");
        }
    }
    check(fs::exists(dbPath) && fs::exists(imported / "robot1.xml"), "原位置不变");
    const auto again = lib::copyLibrary(dbPath, (newDir / "library.sqlite3").string());
    check(again && again->find("已有资源库") != std::string::npos, "目标已有资源库：拒绝");

    // 版本 1 的库自动升级
    {
        const std::string v1 = (dir / "v1.sqlite3").string();
        sqlite3* db = nullptr;
        sqlite3_open(v1.c_str(), &db);
        sqlite3_exec(db,
                     "CREATE TABLE records (id INTEGER PRIMARY KEY, path TEXT NOT NULL UNIQUE, file_sha256 TEXT NOT NULL, bytes INTEGER NOT NULL, "
                     "created TEXT NOT NULL, scene TEXT NOT NULL, scene_sha256 TEXT NOT NULL, motion TEXT NOT NULL, motion_sha256 TEXT NOT NULL, "
                     "start_keyframe TEXT NOT NULL, terra_version TEXT NOT NULL, physics_backend TEXT NOT NULL, platform TEXT NOT NULL, "
                     "steps INTEGER, sim_time REAL, reason TEXT NOT NULL, indexed_at TEXT NOT NULL);"
                     "INSERT INTO records VALUES (1, '/r.terra-record.jsonl', 'x', 1, '2026-01-01T00:00:00Z', '/s', 'y', '', '', '', '0', 'm', "
                     "'p', 1, 0.1, 'completed', 'now');"
                     "PRAGMA user_version = 1;",
                     nullptr, nullptr, nullptr);
        sqlite3_close(db);
        auto upgraded = lib::Library::open(v1, error);
        check(upgraded && upgraded->records().size() == 1 && upgraded->scanAssets(roots).files == 6, "版本 1 的库：升级后记录保留，可登记资产");
        upgraded.reset();
        sqlite3_open(v1.c_str(), &db);
        sqlite3_stmt* s = nullptr;
        sqlite3_prepare_v2(db, "PRAGMA user_version", -1, &s, nullptr);
        const int v = sqlite3_step(s) == SQLITE_ROW ? sqlite3_column_int(s, 0) : -1;
        sqlite3_finalize(s);
        sqlite3_close(db);
        check(v == 2, "升级后 user_version = 2，实际 " + std::to_string(v));
    }

    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
