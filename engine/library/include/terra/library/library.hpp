#pragma once

// 资源库（M3-07b，D-112；M3-07 评审：实验记录进入数据库）。
//
// 文件是原件，数据库只是可重建的索引：记录文件（*.terra-record.jsonl）保存完整内容，库中只存摘要——路径、文件摘要与大小、
// 创建时间、场景 / 动作文件与其摘要、起始关键帧、运行环境、步数、仿真时间、结束原因——用于列出、筛选与查找。
// 数据库为 SQLite 单文件，默认在 $TERRA_LIBRARY，否则 $XDG_DATA_HOME/eai-terra/library.sqlite3
// （XDG_DATA_HOME 未设置时为 ~/.local/share）。库文件损坏时改名备份并重建，再扫描记录目录即可恢复。
//
// 库的格式版本记在 SQLite 的 user_version 中（当前 1）；高于本程序支持的版本时拒绝打开，提示更新程序。

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace terra::library {

constexpr int kSchemaVersion = 1;

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
    // 版本过高或无法创建时返回空，error 为原因。
    static std::unique_ptr<Library> open(const std::string& path, std::string& error, std::string* warning = nullptr);
    ~Library();
    Library(const Library&) = delete;
    Library& operator=(const Library&) = delete;

    const std::string& path() const noexcept;

    // 加入或更新一条实验记录（按路径去重）；失败时返回原因
    std::optional<std::string> addRecord(const std::string& recordPath);
    // 按创建时间从新到旧；scene 非空时只列出该场景的记录
    std::vector<RecordEntry> records(const std::string& scene = {}) const;
    std::optional<RecordEntry> record(const std::string& recordPath) const;
    // 删除文件已不存在的条目，返回删除数
    std::size_t removeMissing();
    // 重建：扫描目录（递归）中的全部 *.terra-record.jsonl 加入索引，并删除文件已不存在的条目
    RebuildResult rebuild(const std::vector<std::string>& directories);

private:
    struct Impl;
    explicit Library(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

// 默认库文件：$TERRA_LIBRARY；否则 $XDG_DATA_HOME/eai-terra/library.sqlite3；否则 ~/.local/share/eai-terra/library.sqlite3
std::string defaultLibraryPath();
// 默认记录目录：库文件所在目录下的 records/
std::string defaultRecordsDirectory();

}  // namespace terra::library
