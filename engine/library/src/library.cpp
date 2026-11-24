// 资源库（M3-07b）：SQLite 只在本文件中使用。

#include "terra/library/library.hpp"

#include <cstdlib>
#include <deque>
#include <ctime>
#include <filesystem>
#include <system_error>

#include <sqlite3.h>

#include "terra/core/sha256.hpp"
#include "terra/simulation/record.hpp"

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
)sql";

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
    if (v < kSchemaVersion && !exec(*db, "PRAGMA user_version = 1", error)) return sqlite3_errcode(*db);
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

std::unique_ptr<Library> Library::open(const std::string& requested, std::string& error, std::string* warning) {
    auto impl = std::make_unique<Impl>();
    impl->path = absolute(requested);
    std::error_code ec;
    const fs::path parent = fs::path{impl->path}.parent_path();
    if (!parent.empty()) fs::create_directories(parent, ec);
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
