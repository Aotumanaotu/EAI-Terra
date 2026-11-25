#pragma once

// 查看器的导入向导（M4-08a）：菜单“文件 → 导入模型…”或把文件拖进窗口时打开。
// 显示分析结果（每个问题的原因与处理办法）、网格、提议的执行器与传感器；可修改名字、网格缩放、包目录、
// 固定基座与每个执行器 / 传感器；“导入并打开”写出导入目录、加载检查，成功后打开生成的场景。

#include <optional>
#include <string>
#include <vector>

#include "terra/import/importer.hpp"
#include "terra/scene/scene.hpp"
#include "terra/ui/file_browser.hpp"

namespace terra::viewer {

struct ImportWizardState {
    bool open = false;
    bool focus = false;
    terra::importer::ImportOptions options;
    std::optional<terra::importer::ImportPlan> plan;
    std::optional<terra::importer::ImportResult> result;
    char name[128] = "";
    float meshScale = 1.0f;
    char packageName[64] = "";
    char packageDir[512] = "";
    terra::ui::FileBrowser browser;
    std::vector<std::string> log;  // 由主程序取出写入日志面板
};

// 分析 input 并打开向导；outputRoot 为导入目录的上级（资源库目录下的 assets/）
void beginImport(ImportWizardState& state, const std::string& input, const std::string& outputRoot);

// 执行导入（“导入并打开”按钮与自检共用）；成功时返回生成的场景文件
std::optional<std::string> runImport(ImportWizardState& state, const terra::scene::WorldFactory& factory);

// 每帧绘制；用户导入成功时返回要打开的场景文件
std::optional<std::string> drawImportWizard(ImportWizardState& state, const terra::scene::WorldFactory& factory);

// 导入用的文件过滤（文件浏览窗口）
inline constexpr const char* kImportFilter = ".urdf|.xacro|.xml|.terra-import.json";  // M4-08b：也可以选导入配方

}  // namespace terra::viewer
