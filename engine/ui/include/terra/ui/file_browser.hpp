#pragma once

// 程序内的文件浏览窗口（M2-08，用户 2026-11-23 选择方案 a：界面风格统一，不依赖系统对话框）。
//
// 模态弹窗：上方为可编辑的目录路径（回车跳转）与“上一级”按钮；中间为目录与文件列表（目录在前，按名称排序，
// 隐藏文件不显示），单击选中、双击进入目录或确认文件；下方为文件名、“只显示场景文件”开关、确认与取消。
// 过滤：默认只显示以 filter（如 ".terra-scene.json"）结尾的文件；关闭开关后显示所有 .json 文件。
// 目录读取与过滤是纯函数（listDirectory），可在无界面的单元测试中验证。

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace terra::ui {

struct DirectoryEntry {
    std::string name;
    bool directory = false;
    std::uintmax_t size = 0;
};

struct DirectoryListing {
    std::vector<DirectoryEntry> entries;  // 目录在前，各自按名称排序；不含 "." 开头的隐藏项
    std::string error;                    // 非空表示目录不可读
};

// 列出 dir：目录全部列出；文件只列出以 filter 结尾的（filter 为空时列出全部）。
DirectoryListing listDirectory(const std::string& dir, const std::string& filter);

// 规范化目录：转为绝对路径；不是目录时取其父目录；都不存在时退回当前目录。
std::string normalizeDirectory(const std::string& path);

class FileBrowser {
public:
    // 打开弹窗：从 startPath（文件或目录）所在目录开始。filter：只显示场景文件时使用的后缀。
    void open(const std::string& startPath, std::string filter);
    bool isOpen() const noexcept { return open_; }

    // 每帧调用（在某个窗口或主菜单之外均可）。用户确认时返回选中文件的绝对路径，并关闭弹窗。
    // title 为弹窗标题（可带 "###" 稳定 ID）。
    std::optional<std::string> draw(const char* title);

    const std::string& directory() const noexcept { return directory_; }

private:
    void refresh();
    void enter(const std::string& dir);

    bool open_ = false;
    bool requestOpen_ = false;
    bool onlyFilter_ = true;
    std::string filter_;
    std::string directory_;
    std::string pathEdit_;
    std::string fileName_;
    DirectoryListing listing_;
};

}  // namespace terra::ui
