#pragma once

// 资源库（M3-07b，D-112；M3-07 评审：实验记录进入数据库）。
//
// 文件是原件，数据库只是可重建的索引：记录文件（*.terra-record.jsonl）保存完整内容，库中只存摘要——路径、文件摘要与大小、
// 创建时间、场景 / 动作文件与其摘要、起始关键帧、运行环境、步数、仿真时间、结束原因——用于列出、筛选与查找。
// 数据库为 SQLite 单文件，默认在 $TERRA_LIBRARY，否则 $XDG_DATA_HOME/eai-terra/library.sqlite3
// （XDG_DATA_HOME 未设置时为 ~/.local/share）。库文件损坏时改名备份并重建，再扫描记录目录即可恢复。
//
// 库的格式版本记在 SQLite 的 user_version 中（当前 2）；高于本程序支持的版本时拒绝打开，提示更新程序。
// 版本 2（M4-08c）：新增资产表——场景、动作、机器人、环境贴图。来源为内置目录（仓库自带）与资源库目录下的 assets/
// （导入的模型），由 scanAssets 增量扫描（只解析修改时间或大小变化的文件）；文件被删除时标为缺失，不删除登记，
// 收藏与隐藏因此保留。版本 1 的库打开时自动加表升级，原有记录不变。
// 机器人不单独扫描：由场景用到的 MJCF 推出（含关节或执行器的 MJCF 才算机器人，地面、房间等不算）。

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace terra::library {

constexpr int kSchemaVersion = 2;

// ---- 资产（M4-08c）----

enum class AssetKind { Scene, Motion, Robot, Environment, Display, Policy };  // Display：显示配置 .terviz（M4-08d）；Policy：策略说明文件（M4-10b）
enum class AssetSource { Builtin, Imported, User };

const char* assetKindName(AssetKind kind);  // "scene"、"motion"、"robot"、"environment"、"display"、"policy"
std::optional<AssetKind> assetKindFromName(const std::string& name);
const char* assetSourceName(AssetSource source);  // "builtin"、"imported"、"user"

struct AssetEntry {
    std::int64_t id = 0;
    AssetKind kind = AssetKind::Scene;
    AssetSource source = AssetSource::Builtin;
    std::string path;         // 绝对路径（机器人为 MJCF 文件）
    std::string title;        // 卡片标题：场景的 title（没有时为 name），动作的 name，机器人在场景中的实例名，贴图的文件名
    std::string description;  // 一行说明：场景的 name；机器人为所在目录
    std::string sha256;
    std::uint64_t bytes = 0;
    std::int64_t modifiedNs = 0;  // 文件修改时间（增量扫描用）
    // 关联：场景 → 用到的机器人 MJCF；动作 → 推荐场景；机器人 → 用到它的场景；显示配置 → 场景；策略 → 推荐场景、模型文件
    std::vector<std::string> related;
    // 标签：场景为 "imu"、"camera"、"lidar2d"、"pd"、"fixed-base"、"models-missing"（有 MJCF 文件不存在，参考机器人未下载）、
    // "invalid"（场景文件有错误）；动作为 "scene-missing"、"invalid"
    std::vector<std::string> tags;
    std::string importDirectory;  // 由导入生成（目录中有 import.terra-import.json）时为该目录
    std::string license;          // 所在目录（或其上级，直到扫描根目录）中的 LICENSE 文件；没有时为空
    bool missing = false;
    bool favorite = false;
    bool hidden = false;
    std::string indexedAt;
};

struct ScanRoot {
    std::string directory;
    AssetSource source = AssetSource::Builtin;
};

struct ScanResult {
    std::size_t files = 0;      // 找到的场景、动作与贴图文件
    std::size_t updated = 0;    // 新增或重新解析的
    std::size_t unchanged = 0;  // 修改时间与大小没变、跳过解析的
    std::size_t missing = 0;    // 本次新标为缺失的（含机器人）
    std::size_t robots = 0;     // 推出的机器人
    std::vector<std::string> errors;  // 解析失败的文件（仍登记，带 "invalid" 标签）
};

// 默认的扫描范围：assetsDir 下的 scenes/、robots/、motions/、hdri/（内置）与库文件所在目录下的 assets/（导入）、
// displays/（显示配置，M4-08d，来源为用户）、policies/（导入的策略，M4-10b，来源为导入）。
// 不含测试夹具（assets/tests/）。
std::vector<ScanRoot> defaultScanRoots(const std::string& assetsDir, const std::string& libraryFile);

// 搜索：query 按空白分成若干词，每个词（不区分 ASCII 大小写）都出现在标题、说明、文件名或标签中才算匹配；空查询匹配全部
bool assetMatches(const AssetEntry& asset, const std::string& query);

struct RecordEntry {
    std::int64_t id = 0;
    std::string path;        // 记录文件（绝对路径）
    std::string fileSha256;  // 记录文件本身的摘要
    std::uint64_t bytes = 0;
    std::string created;     // 记录头中的创建时间（UTC）
    std::string scene;
    std::string sceneSha256;
    std::string motion;
    std::string motionSha256;
    std::string startKeyframe;
    std::string terraVersion;
    std::string physicsBackend;
    std::string platform;
    std::optional<std::uint64_t> steps;  // 没有摘要（记录没有正常结束）时为空
    std::optional<double> simulationTime;
    std::string reason;
    std::string indexedAt;  // 加入索引的时间（UTC）
};

struct RebuildResult {
    std::size_t added = 0;    // 新增或更新的条目
    std::size_t removed = 0;  // 文件已不存在而删除的条目
    std::vector<std::string> errors;  // 读不出的记录文件（不中断扫描）
};

class Library {
public:
    // 打开（必要时创建）库文件。损坏时改名为 <文件>.corrupt-<时间> 并新建，warning 中说明；
    // 版本过高或无法创建时返回空，error 为原因。path 为 ":memory:" 时为内存中的临时库（M4-08c：还没有库文件时列出资产）。
    static std::unique_ptr<Library> open(const std::string& path, std::string& error, std::string* warning = nullptr);
    ~Library();
    Library(const Library&) = delete;
    Library& operator=(const Library&) = delete;

    const std::string& path() const noexcept;
    bool inMemory() const noexcept;

    // 加入或更新一条实验记录（按路径去重）；失败时返回原因
    std::optional<std::string> addRecord(const std::string& recordPath);
    // 按创建时间从新到旧；scene 非空时只列出该场景的记录
    std::vector<RecordEntry> records(const std::string& scene = {}) const;
    std::optional<RecordEntry> record(const std::string& recordPath) const;
    // 删除文件已不存在的条目，返回删除数
    std::size_t removeMissing();
    // 重建：扫描目录（递归）中的全部 *.terra-record.jsonl 加入索引，并删除文件已不存在的条目
    RebuildResult rebuild(const std::vector<std::string>& directories);

    // ---- 资产（M4-08c）----
    // 增量扫描（一个事务）：新文件与修改过的文件重新解析；扫描根目录下不再存在的条目标为缺失；最后重新推出机器人
    ScanResult scanAssets(const std::vector<ScanRoot>& roots);
    // kind 为空时列出全部；默认不含缺失的。排序：类型、收藏在前、内置在前、标题、路径
    std::vector<AssetEntry> assets(std::optional<AssetKind> kind = std::nullopt, bool includeMissing = false) const;
    std::optional<AssetEntry> asset(AssetKind kind, const std::string& path) const;
    // 收藏、隐藏（不删文件）；条目不存在时返回 false
    bool setFavorite(AssetKind kind, const std::string& path, bool favorite);
    bool setHidden(AssetKind kind, const std::string& path, bool hidden);
    // 把记录与资产中以 from 目录开头的路径改为以 to 开头（资源库搬到新位置后），返回改动的条目数
    std::size_t relocate(const std::string& from, const std::string& to);

private:
    struct Impl;
    explicit Library(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

// 默认库文件：$TERRA_LIBRARY；否则 $XDG_DATA_HOME/eai-terra/library.sqlite3；否则 ~/.local/share/eai-terra/library.sqlite3
std::string defaultLibraryPath();
// 默认记录目录：库文件所在目录下的 records/
std::string defaultRecordsDirectory();

// M4-08c：把资源库（库文件与它旁边的 records/、assets/）复制到 toFile 所在的目录，并把新库中的路径改为新位置。
// 目标库文件已存在、或目标目录中已有 records/、assets/ 时不覆盖，返回错误。原位置不变。
std::optional<std::string> copyLibrary(const std::string& fromFile, const std::string& toFile);

}  // namespace terra::library
