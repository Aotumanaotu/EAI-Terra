// 资源库（M3-07b；M4-08c 资产表）：SQLite 只在本文件中使用。

#include "terra/library/library.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <set>
#include <system_error>
#include <utility>

#include <sqlite3.h>

#include "terra/core/sha256.hpp"
#include "terra/scene/scene.hpp"
#include "terra/simulation/motion.hpp"
#include "terra/simulation/record.hpp"
#include "terra/viz/viz_config.hpp"

namespace terra::library {

namespace {

namespace fs = std::filesystem;

constexpr const char* kSchema = R"sql(
CREATE TABLE IF NOT EXISTS records (
    id              INTEGER PRIMARY KEY,
    path            TEXT NOT NULL UNIQUE,
    file_sha256     TEXT NOT NULL,
    bytes           INTEGER NOT NULL,
    created         TEXT NOT NULL,
    scene           TEXT NOT NULL,
    scene_sha256    TEXT NOT NULL,
    motion          TEXT NOT NULL,
    motion_sha256   TEXT NOT NULL,
    start_keyframe  TEXT NOT NULL,
    terra_version   TEXT NOT NULL,
    physics_backend TEXT NOT NULL,
    platform        TEXT NOT NULL,
    steps           INTEGER,
    sim_time        REAL,
    reason          TEXT NOT NULL,
    indexed_at      TEXT NOT NULL
);
CREATE INDEX IF NOT EXISTS records_scene ON records(scene);
CREATE INDEX IF NOT EXISTS records_created ON records(created);
CREATE TABLE IF NOT EXISTS assets (
    id          INTEGER PRIMARY KEY,
    kind        TEXT NOT NULL,
    path        TEXT NOT NULL,
    source      TEXT NOT NULL,
    title       TEXT NOT NULL,
    description TEXT NOT NULL,
    sha256      TEXT NOT NULL,
    bytes       INTEGER NOT NULL,
    modified_ns INTEGER NOT NULL,
    related     TEXT NOT NULL,
    tags        TEXT NOT NULL,
    import_dir  TEXT NOT NULL,
    license     TEXT NOT NULL,
    missing     INTEGER NOT NULL DEFAULT 0,
    favorite    INTEGER NOT NULL DEFAULT 0,
    hidden      INTEGER NOT NULL DEFAULT 0,
    indexed_at  TEXT NOT NULL,
    UNIQUE (kind, path)
);
CREATE INDEX IF NOT EXISTS assets_kind ON assets(kind);
)sql";

constexpr const char* kMemory = ":memory:";

std::string utcNow() {
    const std::time_t now = std::time(nullptr);
    std::tm utc{};
    gmtime_r(&now, &utc);
    char buf[32];
    std::strftime(buf, sizeof buf, "%Y-%m-%dT%H:%M:%SZ", &utc);
    return buf;
}

std::string compactNow() {
    const std::time_t now = std::time(nullptr);
    std::tm utc{};
    gmtime_r(&now, &utc);
    char buf[32];
    std::strftime(buf, sizeof buf, "%Y%m%d-%H%M%S", &utc);
    return buf;
}

std::string absolute(const std::string& path) {
    if (path == kMemory) return path;
    std::error_code ec;
    const fs::path p = fs::absolute(path, ec);
    return (ec ? fs::path{path} : p).lexically_normal().string();
}

// 预编译语句的简单封装
class Statement {
public:
    Statement(sqlite3* db, const char* sql) { ok_ = sqlite3_prepare_v2(db, sql, -1, &stmt_, nullptr) == SQLITE_OK; }
    ~Statement() { sqlite3_finalize(stmt_); }
    Statement(const Statement&) = delete;
    Statement& operator=(const Statement&) = delete;
    bool ok() const noexcept { return ok_; }
    // 绑定的文本由本对象保存（deque 中的元素地址不变），以 SQLITE_STATIC 语义（空析构函数）绑定，
    // 避免使用宏 SQLITE_TRANSIENT（其中的 C 风格转换会触发 -Wold-style-cast）
    void text(int i, const std::string& v) {
        texts_.push_back(v);
        sqlite3_bind_text(stmt_, i, texts_.back().c_str(), static_cast<int>(texts_.back().size()), nullptr);
    }
    void integer(int i, std::int64_t v) { sqlite3_bind_int64(stmt_, i, static_cast<sqlite3_int64>(v)); }
    void real(int i, double v) { sqlite3_bind_double(stmt_, i, v); }
    void null(int i) { sqlite3_bind_null(stmt_, i); }
    int step() { return sqlite3_step(stmt_); }
    std::string columnText(int i) const {
        const unsigned char* t = sqlite3_column_text(stmt_, i);
        return t ? reinterpret_cast<const char*>(t) : std::string{};
    }
    std::int64_t columnInt(int i) const { return sqlite3_column_int64(stmt_, i); }
    double columnReal(int i) const { return sqlite3_column_double(stmt_, i); }
    bool columnNull(int i) const { return sqlite3_column_type(stmt_, i) == SQLITE_NULL; }

    void reset() {
        sqlite3_reset(stmt_);
        sqlite3_clear_bindings(stmt_);
        texts_.clear();
    }

private:
    sqlite3_stmt* stmt_ = nullptr;
    bool ok_ = false;
    std::deque<std::string> texts_;
};

bool exec(sqlite3* db, const char* sql, std::string& error) {
    char* message = nullptr;
    if (sqlite3_exec(db, sql, nullptr, nullptr, &message) != SQLITE_OK) {
        error = message ? message : sqlite3_errmsg(db);
        sqlite3_free(message);
        return false;
    }
    return true;
}

constexpr const char* kColumns =
    "id, path, file_sha256, bytes, created, scene, scene_sha256, motion, motion_sha256, start_keyframe, terra_version, "
    "physics_backend, platform, steps, sim_time, reason, indexed_at";

RecordEntry readEntry(const Statement& s) {
    RecordEntry e;
    e.id = s.columnInt(0);
    e.path = s.columnText(1);
    e.fileSha256 = s.columnText(2);
    e.bytes = static_cast<std::uint64_t>(s.columnInt(3));
    e.created = s.columnText(4);
    e.scene = s.columnText(5);
    e.sceneSha256 = s.columnText(6);
    e.motion = s.columnText(7);
    e.motionSha256 = s.columnText(8);
    e.startKeyframe = s.columnText(9);
    e.terraVersion = s.columnText(10);
    e.physicsBackend = s.columnText(11);
    e.platform = s.columnText(12);
    if (!s.columnNull(13)) e.steps = static_cast<std::uint64_t>(s.columnInt(13));
    if (!s.columnNull(14)) e.simulationTime = s.columnReal(14);
    e.reason = s.columnText(15);
    e.indexedAt = s.columnText(16);
    return e;
}

// 打开并检查库；损坏（不是数据库）时返回 SQLITE_NOTADB 或 SQLITE_CORRUPT
int openChecked(const std::string& path, sqlite3** db, std::string& error) {
    if (sqlite3_open_v2(path.c_str(), db, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, nullptr) != SQLITE_OK) {
        error = "无法打开库文件：" + path + "（" + sqlite3_errmsg(*db) + "）";
        return SQLITE_CANTOPEN;
    }
    sqlite3_busy_timeout(*db, 2000);
    std::int64_t v = 0;
    {
        // 读完版本后语句随作用域结束而释放，再执行建表（有未完成的语句时不能修改表结构）
        Statement version(*db, "PRAGMA user_version");
        if (!version.ok() || version.step() != SQLITE_ROW) {
            const int code = sqlite3_errcode(*db);
            error = std::string{"库文件无法读取（"} + sqlite3_errmsg(*db) + "）";
            return code;
        }
        v = version.columnInt(0);
    }
    if (v > kSchemaVersion) {
        error = "库文件的版本 " + std::to_string(v) + " 高于本程序支持的版本 " + std::to_string(kSchemaVersion) + "，请更新程序：" + path;
        return SQLITE_MISMATCH;
    }
    if (!exec(*db, kSchema, error)) {
        error = "创建库的表失败：" + error;
        return sqlite3_errcode(*db);
    }
    // 版本 1 → 2（M4-08c）：上面的建表语句已补上资产表，原有的记录表不变
    const std::string setVersion = "PRAGMA user_version = " + std::to_string(kSchemaVersion);
    if (v < kSchemaVersion && !exec(*db, setVersion.c_str(), error)) return sqlite3_errcode(*db);
    return SQLITE_OK;
}

}  // namespace

struct Library::Impl {
    std::string path;
    sqlite3* db = nullptr;
    ~Impl() { sqlite3_close(db); }
};

Library::Library(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
Library::~Library() = default;

const std::string& Library::path() const noexcept { return impl_->path; }
bool Library::inMemory() const noexcept { return impl_->path == kMemory; }

std::unique_ptr<Library> Library::open(const std::string& requested, std::string& error, std::string* warning) {
    auto impl = std::make_unique<Impl>();
    impl->path = absolute(requested);
    std::error_code ec;
    const fs::path parent = fs::path{impl->path}.parent_path();
    if (!parent.empty() && impl->path != kMemory) fs::create_directories(parent, ec);
    int rc = openChecked(impl->path, &impl->db, error);
    if (rc == SQLITE_NOTADB || rc == SQLITE_CORRUPT) {
        // 索引可以重建：把坏文件改名备份，新建一个空库
        sqlite3_close(impl->db);
        impl->db = nullptr;
        const std::string backup = impl->path + ".corrupt-" + compactNow();
        fs::rename(impl->path, backup, ec);
        if (ec) {
            error = "库文件损坏，且无法改名备份：" + impl->path;
            return nullptr;
        }
        if (warning) *warning = "库文件损坏，已改名为 " + backup + " 并新建；可重新扫描记录目录恢复索引";
        error.clear();
        rc = openChecked(impl->path, &impl->db, error);
    }
    if (rc != SQLITE_OK) return nullptr;
    return std::unique_ptr<Library>(new Library(std::move(impl)));
}

std::optional<std::string> Library::addRecord(const std::string& recordPath) {
    const std::string path = absolute(recordPath);
    std::string error;
    const auto info = terra::simulation::readRecordInfo(path, error);
    if (!info) return error;
    const auto digest = terra::sha256File(path);
    if (!digest) return "无法读取记录文件：" + path;
    std::error_code ec;
    const auto bytes = fs::file_size(path, ec);
    Statement s(impl_->db,
                "INSERT INTO records (path, file_sha256, bytes, created, scene, scene_sha256, motion, motion_sha256, start_keyframe, "
                "terra_version, physics_backend, platform, steps, sim_time, reason, indexed_at) "
                "VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11, ?12, ?13, ?14, ?15, ?16) "
                "ON CONFLICT(path) DO UPDATE SET file_sha256 = excluded.file_sha256, bytes = excluded.bytes, "
                "created = excluded.created, scene = excluded.scene, scene_sha256 = excluded.scene_sha256, "
                "motion = excluded.motion, motion_sha256 = excluded.motion_sha256, start_keyframe = excluded.start_keyframe, "
                "terra_version = excluded.terra_version, physics_backend = excluded.physics_backend, "
                "platform = excluded.platform, steps = excluded.steps, sim_time = excluded.sim_time, "
                "reason = excluded.reason, indexed_at = excluded.indexed_at");
    if (!s.ok()) return std::string{"库语句准备失败："} + sqlite3_errmsg(impl_->db);
    const auto& h = info->header;
    s.text(1, path);
    s.text(2, *digest);
    s.integer(3, ec ? 0 : static_cast<std::int64_t>(bytes));
    s.text(4, h.created);
    s.text(5, h.scenePath);
    s.text(6, h.sceneSha256);
    s.text(7, h.motionPath);
    s.text(8, h.motionSha256);
    s.text(9, h.startKeyframe);
    s.text(10, h.terraVersion);
    s.text(11, h.physicsBackend);
    s.text(12, h.platform);
    if (info->summary) {
        s.integer(13, static_cast<std::int64_t>(info->summary->steps));
        s.real(14, info->summary->time);
        s.text(15, info->summary->reason);
    } else {
        s.null(13);
        s.null(14);
        s.text(15, "incomplete");
    }
    s.text(16, utcNow());
    if (s.step() != SQLITE_DONE) return std::string{"写入库失败："} + sqlite3_errmsg(impl_->db);
    return std::nullopt;
}

std::vector<RecordEntry> Library::records(const std::string& scene) const {
    std::vector<RecordEntry> out;
    const std::string sql = std::string{"SELECT "} + kColumns + " FROM records" + (scene.empty() ? "" : " WHERE scene = ?1") +
                            " ORDER BY created DESC, id DESC";
    Statement s(impl_->db, sql.c_str());
    if (!s.ok()) return out;
    if (!scene.empty()) s.text(1, absolute(scene));
    while (s.step() == SQLITE_ROW) out.push_back(readEntry(s));
    return out;
}

std::optional<RecordEntry> Library::record(const std::string& recordPath) const {
    const std::string sql = std::string{"SELECT "} + kColumns + " FROM records WHERE path = ?1";
    Statement s(impl_->db, sql.c_str());
    if (!s.ok()) return std::nullopt;
    s.text(1, absolute(recordPath));
    if (s.step() != SQLITE_ROW) return std::nullopt;
    return readEntry(s);
}

std::size_t Library::removeMissing() {
    std::vector<std::int64_t> missing;
    {
        Statement s(impl_->db, "SELECT id, path FROM records");
        if (!s.ok()) return 0;
        while (s.step() == SQLITE_ROW) {
            std::error_code ec;
            if (!fs::exists(s.columnText(1), ec)) missing.push_back(s.columnInt(0));
        }
    }
    Statement del(impl_->db, "DELETE FROM records WHERE id = ?1");
    if (!del.ok()) return 0;
    std::size_t removed = 0;
    for (const auto id : missing) {
        del.reset();
        del.integer(1, id);
        if (del.step() == SQLITE_DONE) ++removed;
    }
    return removed;
}

RebuildResult Library::rebuild(const std::vector<std::string>& directories) {
    RebuildResult result;
    const std::string suffix = ".terra-record.jsonl";
    std::string ignored;
    exec(impl_->db, "BEGIN", ignored);
    for (const auto& dir : directories) {
        std::error_code ec;
        if (!fs::is_directory(dir, ec)) continue;
        for (auto it = fs::recursive_directory_iterator(dir, fs::directory_options::skip_permission_denied, ec);
             !ec && it != fs::recursive_directory_iterator(); it.increment(ec)) {
            const std::string name = it->path().filename().string();
            if (!it->is_regular_file(ec) || name.size() < suffix.size() ||
                name.compare(name.size() - suffix.size(), suffix.size(), suffix) != 0) {
                continue;
            }
            if (const auto error = addRecord(it->path().string())) {
                result.errors.push_back(*error);
            } else {
                ++result.added;
            }
        }
    }
    exec(impl_->db, "COMMIT", ignored);
    result.removed = removeMissing();
    return result;
}

// ---- 资产（M4-08c）----

namespace {

constexpr const char* kSceneSuffix = ".terra-scene.json";
constexpr const char* kMotionSuffix = ".terra-motion.json";
constexpr const char* kRecipeFile = "import.terra-import.json";

bool endsWith(const std::string& s, const std::string& suffix) {
    return s.size() >= suffix.size() && s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool isUnder(const std::string& path, const std::string& dir) {
    if (dir.empty()) return false;
    const std::string prefix = dir.back() == '/' ? dir : dir + "/";
    return path.compare(0, prefix.size(), prefix) == 0;
}

std::string joinLines(const std::vector<std::string>& items) {
    std::string out;
    for (const auto& i : items) out += (out.empty() ? "" : "\n") + i;
    return out;
}

std::vector<std::string> splitLines(const std::string& text) {
    std::vector<std::string> out;
    std::size_t start = 0;
    while (start <= text.size()) {
        const std::size_t end = text.find('\n', start);
        const std::string item = text.substr(start, end == std::string::npos ? std::string::npos : end - start);
        if (!item.empty()) out.push_back(item);
        if (end == std::string::npos) break;
        start = end + 1;
    }
    return out;
}

std::int64_t modifiedNanoseconds(const fs::path& p) {
    std::error_code ec;
    const auto t = fs::last_write_time(p, ec);
    if (ec) return 0;
    return static_cast<std::int64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count());
}

std::optional<AssetKind> kindOfFile(const std::string& name) {
    if (endsWith(name, kSceneSuffix)) return AssetKind::Scene;
    if (endsWith(name, kMotionSuffix)) return AssetKind::Motion;
    if (endsWith(name, ".hdr")) return AssetKind::Environment;
    if (endsWith(name, terra::viz::kVizSuffix)) return AssetKind::Display;
    return std::nullopt;
}

std::string stemOf(const fs::path& p) {
    std::string name = p.filename().string();
    for (const char* suffix : {kSceneSuffix, kMotionSuffix, ".hdr", ".xml", terra::viz::kVizSuffix}) {
        if (endsWith(name, suffix)) return name.substr(0, name.size() - std::string{suffix}.size());
    }
    return p.stem().string();
}

// dir 起向上（不超过 stop）找许可文件
std::string findLicense(fs::path dir, const std::string& stop) {
    std::error_code ec;
    for (int depth = 0; depth < 8 && !dir.empty(); ++depth) {
        for (const char* name : {"LICENSE", "LICENSE.txt", "LICENSE.md", "COPYING"}) {
            if (fs::is_regular_file(dir / name, ec)) return (dir / name).string();
        }
        if (stop.empty() || dir.string() == stop || !isUnder(dir.string(), stop)) break;
        dir = dir.parent_path();
    }
    return {};
}

std::string importDirectoryOf(const fs::path& file) {
    std::error_code ec;
    return fs::is_regular_file(file.parent_path() / kRecipeFile, ec) ? file.parent_path().string() : std::string{};
}

// 含关节或执行器的 MJCF 才算机器人（地面、房间等静态模型不算）
bool mjcfIsRobot(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    const std::string text{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    return text.find("<joint") != std::string::npos || text.find("<freejoint") != std::string::npos ||
           text.find("<actuator") != std::string::npos;
}

void addTag(std::vector<std::string>& tags, const std::string& tag) {
    if (std::find(tags.begin(), tags.end(), tag) == tags.end()) tags.push_back(tag);
}

bool hasTag(const std::vector<std::string>& tags, const char* tag) { return std::find(tags.begin(), tags.end(), tag) != tags.end(); }

AssetSource sourceFromName(const std::string& name) {
    if (name == "imported") return AssetSource::Imported;
    if (name == "user") return AssetSource::User;
    return AssetSource::Builtin;
}

// 解析场景、动作或贴图文件，填写标题、说明、关联与标签；解析失败时带 "invalid" 标签并返回原因
std::optional<std::string> describeFile(AssetEntry& e) {
    const fs::path p{e.path};
    e.title = stemOf(p);
    std::error_code ec;
    if (e.kind == AssetKind::Scene) {
        const auto r = terra::scene::readSceneFile(e.path);
        if (!r.document) {
            addTag(e.tags, "invalid");
            for (const auto& i : r.issues) {
                if (i.severity == terra::scene::SceneIssue::Severity::Error) {
                    e.description = (i.where.empty() ? "" : i.where + "：") + i.message;
                    break;
                }
            }
            return e.path + "：" + e.description;
        }
        const auto& d = *r.document;
        e.title = !d.title.empty() ? d.title : !d.name.empty() ? d.name : e.title;
        e.description = d.name;
        for (const auto& m : d.models) {
            const std::string mjcf = fs::path{m.mjcf}.lexically_normal().string();
            if (!fs::is_regular_file(mjcf, ec)) {
                addTag(e.tags, "models-missing");
                e.related.push_back(mjcf);  // 多半是还没下载的参考机器人
            } else if (mjcfIsRobot(mjcf)) {
                e.related.push_back(mjcf);
            }
            if (m.fixedBase) addTag(e.tags, "fixed-base");
        }
        for (const auto& sensor : d.sensors) addTag(e.tags, sensor.type);
        if (!d.controllers.empty()) addTag(e.tags, "pd");
    } else if (e.kind == AssetKind::Motion) {
        const auto r = terra::simulation::readMotionFile(e.path);
        if (!r.document) {
            addTag(e.tags, "invalid");
            for (const auto& i : r.issues) {
                if (i.severity == terra::simulation::MotionIssue::Severity::Error) {
                    e.description = (i.where.empty() ? "" : i.where + "：") + i.message;
                    break;
                }
            }
            return e.path + "：" + e.description;
        }
        if (!r.document->name.empty()) e.title = r.document->name;
        if (!r.document->scene.empty()) {
            const std::string scene = fs::path{r.document->scene}.lexically_normal().string();
            e.related.push_back(scene);
            if (!fs::is_regular_file(scene, ec)) addTag(e.tags, "scene-missing");
        }
    } else if (e.kind == AssetKind::Display) {  // M4-08d：显示配置，关联到场景
        const auto r = terra::viz::readViz(e.path);
        if (!r.config) {
            addTag(e.tags, "invalid");
            e.description = r.errors.empty() ? std::string{} : r.errors.front();
            return e.path + "：" + e.description;
        }
        if (!r.config->name.empty()) e.title = r.config->name;
        std::size_t shown = 0;
        for (const auto& w : r.config->windows) shown += w.type != terra::viz::WindowType::Unknown ? 1u : 0u;
        e.description = std::to_string(shown) + " 个窗口";
        if (!r.config->scene.empty()) {
            e.related.push_back(r.config->scene);
            if (!fs::is_regular_file(r.config->scene, ec)) addTag(e.tags, "scene-missing");
        }
    }
    return std::nullopt;
}

constexpr const char* kAssetColumns =
    "id, kind, path, source, title, description, sha256, bytes, modified_ns, related, tags, import_dir, license, missing, favorite, "
    "hidden, indexed_at";

AssetEntry readAsset(const Statement& s) {
    AssetEntry e;
    e.id = s.columnInt(0);
    e.kind = assetKindFromName(s.columnText(1)).value_or(AssetKind::Scene);
    e.path = s.columnText(2);
    e.source = sourceFromName(s.columnText(3));
    e.title = s.columnText(4);
    e.description = s.columnText(5);
    e.sha256 = s.columnText(6);
    e.bytes = static_cast<std::uint64_t>(s.columnInt(7));
    e.modifiedNs = s.columnInt(8);
    e.related = splitLines(s.columnText(9));
    e.tags = splitLines(s.columnText(10));
    e.importDirectory = s.columnText(11);
    e.license = s.columnText(12);
    e.missing = s.columnInt(13) != 0;
    e.favorite = s.columnInt(14) != 0;
    e.hidden = s.columnInt(15) != 0;
    e.indexedAt = s.columnText(16);
    return e;
}

// 新增或更新（保留收藏与隐藏），并清除缺失标记
bool upsertAsset(sqlite3* db, const AssetEntry& e, const std::string& now) {
    Statement s(db,
                "INSERT INTO assets (kind, path, source, title, description, sha256, bytes, modified_ns, related, tags, import_dir, license, "
                "missing, indexed_at) VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11, ?12, 0, ?13) "
                "ON CONFLICT(kind, path) DO UPDATE SET source = excluded.source, title = excluded.title, "
                "description = excluded.description, sha256 = excluded.sha256, bytes = excluded.bytes, "
                "modified_ns = excluded.modified_ns, related = excluded.related, tags = excluded.tags, "
                "import_dir = excluded.import_dir, license = excluded.license, missing = 0, indexed_at = excluded.indexed_at");
    if (!s.ok()) return false;
    s.text(1, assetKindName(e.kind));
    s.text(2, e.path);
    s.text(3, assetSourceName(e.source));
    s.text(4, e.title);
    s.text(5, e.description);
    s.text(6, e.sha256);
    s.integer(7, static_cast<std::int64_t>(e.bytes));
    s.integer(8, e.modifiedNs);
    s.text(9, joinLines(e.related));
    s.text(10, joinLines(e.tags));
    s.text(11, e.importDirectory);
    s.text(12, e.license);
    s.text(13, now);
    return s.step() == SQLITE_DONE;
}

std::string lowerAscii(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

}  // namespace

const char* assetKindName(AssetKind kind) {
    switch (kind) {
        case AssetKind::Scene: return "scene";
        case AssetKind::Motion: return "motion";
        case AssetKind::Robot: return "robot";
        case AssetKind::Environment: return "environment";
        case AssetKind::Display: return "display";
    }
    return "scene";
}

std::optional<AssetKind> assetKindFromName(const std::string& name) {
    for (const AssetKind k : {AssetKind::Scene, AssetKind::Motion, AssetKind::Robot, AssetKind::Environment, AssetKind::Display}) {
        if (name == assetKindName(k)) return k;
    }
    return std::nullopt;
}

const char* assetSourceName(AssetSource source) {
    switch (source) {
        case AssetSource::Builtin: return "builtin";
        case AssetSource::Imported: return "imported";
        case AssetSource::User: return "user";
    }
    return "builtin";
}

std::vector<ScanRoot> defaultScanRoots(const std::string& assetsDir, const std::string& libraryFile) {
    std::vector<ScanRoot> roots;
    for (const char* sub : {"scenes", "robots", "motions", "hdri"}) roots.push_back({assetsDir + "/" + sub, AssetSource::Builtin});
    if (!libraryFile.empty() && libraryFile != kMemory) {
        roots.push_back({(fs::path{absolute(libraryFile)}.parent_path() / "assets").string(), AssetSource::Imported});
        roots.push_back({(fs::path{absolute(libraryFile)}.parent_path() / "displays").string(), AssetSource::User});
    }
    return roots;
}

bool assetMatches(const AssetEntry& asset, const std::string& query) {
    std::string haystack = asset.title + "\n" + asset.description + "\n" + fs::path{asset.path}.filename().string();
    for (const auto& t : asset.tags) haystack += "\n" + t;
    haystack = lowerAscii(haystack);
    const std::string q = lowerAscii(query);
    std::size_t i = 0;
    while (i < q.size()) {
        while (i < q.size() && std::isspace(static_cast<unsigned char>(q[i]))) ++i;
        std::size_t j = i;
        while (j < q.size() && !std::isspace(static_cast<unsigned char>(q[j]))) ++j;
        if (j > i && haystack.find(q.substr(i, j - i)) == std::string::npos) return false;
        i = j;
    }
    return true;
}

ScanResult Library::scanAssets(const std::vector<ScanRoot>& roots) {
    ScanResult result;
    sqlite3* db = impl_->db;
    const std::string now = utcNow();
    struct Existing {
        std::int64_t modifiedNs = 0;
        std::uint64_t bytes = 0;
        bool missing = false;
        bool reparse = false;  // 依赖别的文件（模型、推荐场景）而当时缺失：每次都重新解析
    };
    std::map<std::pair<std::string, std::string>, Existing> existing;
    {
        Statement s(db, "SELECT kind, path, modified_ns, bytes, missing, tags FROM assets");
        if (!s.ok()) {
            result.errors.push_back(std::string{"读取资产表失败："} + sqlite3_errmsg(db));
            return result;
        }
        while (s.step() == SQLITE_ROW) {
            const auto tags = splitLines(s.columnText(5));
            existing[{s.columnText(0), s.columnText(1)}] = {s.columnInt(2), static_cast<std::uint64_t>(s.columnInt(3)), s.columnInt(4) != 0,
                                                            hasTag(tags, "models-missing") || hasTag(tags, "scene-missing")};
        }
    }
    std::string ignored;
    exec(db, "BEGIN", ignored);
    std::vector<std::string> rootDirs;
    std::set<std::pair<std::string, std::string>> seen;
    for (const auto& root : roots) {
        const std::string rootDir = absolute(root.directory);
        rootDirs.push_back(rootDir);
        std::error_code ec;
        if (!fs::is_directory(rootDir, ec)) continue;
        for (auto it = fs::recursive_directory_iterator(rootDir, fs::directory_options::skip_permission_denied, ec);
             !ec && it != fs::recursive_directory_iterator(); it.increment(ec)) {
            std::error_code fileEc;
            const std::string name = it->path().filename().string();
            if (!name.empty() && name[0] == '.') {  // 隐藏文件与目录（含解包中的临时目录）
                if (it->is_directory(fileEc)) it.disable_recursion_pending();
                continue;
            }
            if (!it->is_regular_file(fileEc)) continue;
            const auto kind = kindOfFile(name);
            if (!kind) continue;
            AssetEntry e;
            e.kind = *kind;
            e.source = root.source;
            e.path = it->path().lexically_normal().string();
            const std::pair<std::string, std::string> key{assetKindName(e.kind), e.path};
            if (!seen.insert(key).second) continue;  // 扫描根目录重叠时只算一次
            ++result.files;
            e.bytes = static_cast<std::uint64_t>(fs::file_size(e.path, fileEc));
            e.modifiedNs = modifiedNanoseconds(e.path);
            const auto found = existing.find(key);
            if (found != existing.end() && !found->second.missing && !found->second.reparse && found->second.modifiedNs == e.modifiedNs &&
                found->second.bytes == e.bytes) {
                ++result.unchanged;
                continue;
            }
            if (const auto error = describeFile(e)) result.errors.push_back(*error);
            e.sha256 = terra::sha256File(e.path).value_or(std::string{});
            e.importDirectory = importDirectoryOf(e.path);
            e.license = findLicense(fs::path{e.path}.parent_path(), rootDir);
            if (upsertAsset(db, e, now)) ++result.updated;
            else result.errors.push_back(e.path + "：写入库失败（" + sqlite3_errmsg(db) + "）");
        }
    }
    // 扫描根目录下没有找到（或文件已不存在）的条目标为缺失
    {
        Statement mark(db, "UPDATE assets SET missing = 1 WHERE kind = ?1 AND path = ?2");
        for (const auto& entry : existing) {
            const auto& key = entry.first;
            if (key.first == assetKindName(AssetKind::Robot) || entry.second.missing || seen.count(key)) continue;
            std::error_code ec;
            const std::string& path = key.second;
            const bool underRoot = std::any_of(rootDirs.begin(), rootDirs.end(), [&path](const std::string& d) { return isUnder(path, d); });
            if (!underRoot && fs::exists(key.second, ec)) continue;
            mark.reset();
            mark.text(1, key.first);
            mark.text(2, key.second);
            if (mark.step() == SQLITE_DONE) ++result.missing;
        }
    }
    // 由场景推出机器人：MJCF → 用到它的场景（按路径排序，第一个决定来源）
    struct RobotUse {
        std::vector<std::string> scenes;
        AssetSource source = AssetSource::Builtin;
    };
    std::map<std::string, RobotUse> robots;
    {
        Statement s(db, "SELECT path, related, source FROM assets WHERE kind = 'scene' AND missing = 0 ORDER BY path");
        while (s.ok() && s.step() == SQLITE_ROW) {
            for (const auto& mjcf : splitLines(s.columnText(1))) {
                auto& use = robots[mjcf];
                if (use.scenes.empty()) use.source = sourceFromName(s.columnText(2));
                use.scenes.push_back(s.columnText(0));
            }
        }
    }
    const auto rootOf = [&](const std::string& path) {
        for (const auto& d : rootDirs) {
            if (isUnder(path, d)) return d;
        }
        return std::string{};
    };
    for (const auto& [mjcf, use] : robots) {
        AssetEntry e;
        e.kind = AssetKind::Robot;
        e.source = use.source;
        e.path = mjcf;
        e.title = stemOf(fs::path{mjcf});
        e.description = fs::path{mjcf}.parent_path().filename().string();
        e.related = use.scenes;
        std::error_code ec;
        const bool present = fs::is_regular_file(mjcf, ec);
        if (present) {
            e.bytes = static_cast<std::uint64_t>(fs::file_size(mjcf, ec));
            e.modifiedNs = modifiedNanoseconds(mjcf);
            const auto found = existing.find({assetKindName(AssetKind::Robot), mjcf});
            const bool same = found != existing.end() && found->second.modifiedNs == e.modifiedNs && found->second.bytes == e.bytes;
            if (same) {
                if (const auto old = asset(AssetKind::Robot, mjcf)) e.sha256 = old->sha256;
            } else {
                e.sha256 = terra::sha256File(mjcf).value_or(std::string{});
            }
            e.importDirectory = importDirectoryOf(mjcf);
            const std::string root = rootOf(mjcf);
            e.license = findLicense(fs::path{mjcf}.parent_path(), root);
        }
        if (upsertAsset(db, e, now)) ++result.robots;
        if (!present) {  // 模型文件不存在（参考机器人未下载）：登记关联，但标为缺失
            Statement mark(db, "UPDATE assets SET missing = 1 WHERE kind = 'robot' AND path = ?1");
            mark.text(1, mjcf);
            mark.step();
        }
    }
    {
        Statement mark(db, "UPDATE assets SET missing = 1 WHERE kind = 'robot' AND path = ?1 AND missing = 0");
        for (const auto& [key, info] : existing) {
            if (key.first != assetKindName(AssetKind::Robot) || robots.count(key.second) || info.missing) continue;
            mark.reset();
            mark.text(1, key.second);
            if (mark.step() == SQLITE_DONE && sqlite3_changes(db) > 0) ++result.missing;
        }
    }
    exec(db, "COMMIT", ignored);
    return result;
}

std::vector<AssetEntry> Library::assets(std::optional<AssetKind> kind, bool includeMissing) const {
    std::vector<AssetEntry> out;
    std::string sql = std::string{"SELECT "} + kAssetColumns + " FROM assets WHERE 1";
    if (kind) sql += " AND kind = ?1";
    if (!includeMissing) sql += " AND missing = 0";
    sql += " ORDER BY kind, favorite DESC, CASE source WHEN 'builtin' THEN 0 WHEN 'imported' THEN 1 ELSE 2 END, title, path";
    Statement s(impl_->db, sql.c_str());
    if (!s.ok()) return out;
    if (kind) s.text(1, assetKindName(*kind));
    while (s.step() == SQLITE_ROW) out.push_back(readAsset(s));
    return out;
}

std::optional<AssetEntry> Library::asset(AssetKind kind, const std::string& path) const {
    const std::string sql = std::string{"SELECT "} + kAssetColumns + " FROM assets WHERE kind = ?1 AND path = ?2";
    Statement s(impl_->db, sql.c_str());
    if (!s.ok()) return std::nullopt;
    s.text(1, assetKindName(kind));
    s.text(2, absolute(path));
    if (s.step() != SQLITE_ROW) return std::nullopt;
    return readAsset(s);
}

bool Library::setFavorite(AssetKind kind, const std::string& path, bool favorite) {
    Statement s(impl_->db, "UPDATE assets SET favorite = ?1 WHERE kind = ?2 AND path = ?3");
    if (!s.ok()) return false;
    s.integer(1, favorite ? 1 : 0);
    s.text(2, assetKindName(kind));
    s.text(3, absolute(path));
    return s.step() == SQLITE_DONE && sqlite3_changes(impl_->db) > 0;
}

bool Library::setHidden(AssetKind kind, const std::string& path, bool hidden) {
    Statement s(impl_->db, "UPDATE assets SET hidden = ?1 WHERE kind = ?2 AND path = ?3");
    if (!s.ok()) return false;
    s.integer(1, hidden ? 1 : 0);
    s.text(2, assetKindName(kind));
    s.text(3, absolute(path));
    return s.step() == SQLITE_DONE && sqlite3_changes(impl_->db) > 0;
}

std::size_t Library::relocate(const std::string& from, const std::string& to) {
    const std::string a = absolute(from);
    const std::string b = absolute(to);
    const auto moved = [&](const std::string& p) { return isUnder(p, a) ? b + p.substr(a.size()) : p; };
    const auto movedLines = [&](const std::string& text) {
        std::vector<std::string> items = splitLines(text);
        for (auto& i : items) i = moved(i);
        return joinLines(items);
    };
    std::size_t changed = 0;
    std::string ignored;
    exec(impl_->db, "BEGIN", ignored);
    {
        std::vector<std::pair<std::int64_t, std::string>> rows;
        Statement s(impl_->db, "SELECT id, path FROM records");
        while (s.ok() && s.step() == SQLITE_ROW) {
            if (isUnder(s.columnText(1), a)) rows.emplace_back(s.columnInt(0), moved(s.columnText(1)));
        }
        Statement u(impl_->db, "UPDATE records SET path = ?1 WHERE id = ?2");
        for (const auto& [id, p] : rows) {
            u.reset();
            u.text(1, p);
            u.integer(2, id);
            if (u.step() == SQLITE_DONE) ++changed;
        }
    }
    {
        struct Row {
            std::int64_t id;
            std::string path, related, importDir, license;
        };
        std::vector<Row> rows;
        Statement s(impl_->db, "SELECT id, path, related, import_dir, license FROM assets");
        while (s.ok() && s.step() == SQLITE_ROW) {
            Row r{s.columnInt(0), moved(s.columnText(1)), movedLines(s.columnText(2)), moved(s.columnText(3)), moved(s.columnText(4))};
            if (r.path != s.columnText(1) || r.related != s.columnText(2) || r.importDir != s.columnText(3) || r.license != s.columnText(4)) {
                rows.push_back(std::move(r));
            }
        }
        Statement u(impl_->db, "UPDATE assets SET path = ?1, related = ?2, import_dir = ?3, license = ?4 WHERE id = ?5");
        for (const auto& r : rows) {
            u.reset();
            u.text(1, r.path);
            u.text(2, r.related);
            u.text(3, r.importDir);
            u.text(4, r.license);
            u.integer(5, r.id);
            if (u.step() == SQLITE_DONE) ++changed;
        }
    }
    exec(impl_->db, "COMMIT", ignored);
    return changed;
}

std::optional<std::string> copyLibrary(const std::string& fromFile, const std::string& toFile) {
    const fs::path from{absolute(fromFile)};
    const fs::path to{absolute(toFile)};
    std::error_code ec;
    if (!fs::is_regular_file(from, ec)) return "原位置没有资源库：" + from.string();
    if (fs::exists(to, ec)) return "目标位置已有资源库：" + to.string();
    const fs::path fromDir = from.parent_path();
    const fs::path toDir = to.parent_path();
    if (fs::equivalent(fromDir, toDir, ec)) return "目标与原位置是同一个目录：" + toDir.string();
    for (const char* sub : {"records", "assets"}) {
        if (fs::exists(toDir / sub, ec)) return "目标目录中已有 " + std::string{sub} + "/，不覆盖：" + (toDir / sub).string();
    }
    fs::create_directories(toDir, ec);
    if (ec) return "无法创建目录：" + toDir.string() + "（" + ec.message() + "）";
    {
        // 一致的副本：VACUUM INTO（不受原库中未完成写入的影响）
        sqlite3* db = nullptr;
        if (sqlite3_open_v2(from.string().c_str(), &db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
            const std::string message = sqlite3_errmsg(db);
            sqlite3_close(db);
            return "无法打开原资源库：" + message;
        }
        sqlite3_busy_timeout(db, 2000);
        std::string message;
        {
            Statement s(db, "VACUUM INTO ?1");
            if (s.ok()) s.text(1, to.string());
            if (!s.ok() || s.step() != SQLITE_DONE) message = sqlite3_errmsg(db);
        }
        sqlite3_close(db);
        if (!message.empty()) return "复制库文件失败：" + message;
    }
    for (const char* sub : {"records", "assets"}) {
        if (!fs::exists(fromDir / sub, ec)) continue;
        fs::copy(fromDir / sub, toDir / sub, fs::copy_options::recursive, ec);
        if (ec) return "复制 " + std::string{sub} + "/ 失败：" + ec.message();
    }
    std::string error;
    auto copied = Library::open(to.string(), error);
    if (!copied) return error;
    copied->relocate(fromDir.string(), toDir.string());
    return std::nullopt;
}

std::string defaultLibraryPath() {
    if (const char* env = std::getenv("TERRA_LIBRARY"); env && *env) return absolute(env);
    if (const char* xdg = std::getenv("XDG_DATA_HOME"); xdg && *xdg) return absolute(std::string{xdg} + "/eai-terra/library.sqlite3");
    const char* home = std::getenv("HOME");
    return absolute(std::string{home ? home : "."} + "/.local/share/eai-terra/library.sqlite3");
}

std::string defaultRecordsDirectory() {
    return (fs::path{defaultLibraryPath()}.parent_path() / "records").string();
}

}  // namespace terra::library
