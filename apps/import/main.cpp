// terra_import：模型导入的命令行程序（M4-08a）。分析 → 写出导入目录 → 加载并推进 1 s 检查。
//
// 用法：terra_import <URDF / xacro / MJCF> [--name 名字] [--output 目录] [--package 包名=目录]... [--mesh-scale 比例]
//                    [--fixed-base | --floating-base] [--no-actuators] [--no-sensors] [--overwrite] [--dry-run]
//                    [--assimp 程序] [--python 程序] [名字:=值]...（xacro 参数）
//   --output   导入目录的上级；默认为资源库目录下的 assets/（$TERRA_LIBRARY 或 ~/.local/share/eai-terra/）
//   --dry-run  只分析，不写文件
//   --recipe   （M4-08b）按导入配方（*.terra-import.json）导入；配方中的选择优先，命令行可另给 --output、--overwrite、--dry-run
// 标准错误：每个问题一行（级别、代码、说明、处理办法）；标准输出：最后一行 {"type":"import",...}
// 退出码：0 成功；2 参数错误；4 有错误（分析或检查）

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>

#include "terra/import/importer.hpp"
#include "terra/library/library.hpp"
#include "terra/physics/mujoco/mujoco_backend.hpp"

namespace {

namespace im = terra::importer;

int usage() {
    std::fprintf(stderr,
                 "用法：terra_import <URDF / xacro / MJCF> [--name 名字] [--output 目录] [--package 包名=目录]... [--mesh-scale 比例]\n"
                 "                   [--fixed-base | --floating-base] [--no-actuators] [--no-sensors] [--overwrite] [--dry-run]\n"
                 "                   [--assimp 程序] [--python 程序] [名字:=值]...\n"
                 "       terra_import --recipe <导入配方.terra-import.json> [--output 目录] [--overwrite] [--dry-run]\n");
    return 2;
}

std::string escape(const std::string& s) {
    std::string out;
    for (const char c : s) {
        if (c == '"' || c == '\\') out += '\\';
        out += c;
    }
    return out;
}

void printIssues(const std::vector<im::ImportIssue>& issues) {
    for (const auto& i : issues) {
        std::fprintf(stderr, "terra_import: [%s] %s：%s%s%s\n", im::severityName(i.severity), i.code.c_str(), i.message.c_str(),
                     i.hint.empty() ? "" : "  → ", i.hint.c_str());
    }
}

}  // namespace

int main(int argc, char** argv) {
    im::ImportOptions options = im::defaultOptions();
    bool dryRun = false, noActuators = false, noSensors = false;
    std::string recipePath;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const bool has = i + 1 < argc;
        if (a == "--name" && has) options.name = argv[++i];
        else if (a == "--output" && has) options.outputRoot = argv[++i];
        else if (a == "--package" && has) {
            const std::string v = argv[++i];
            const auto eq = v.find('=');
            if (eq == std::string::npos || eq == 0 || eq + 1 == v.size()) return usage();
            options.packages[v.substr(0, eq)] = v.substr(eq + 1);
        } else if (a == "--mesh-scale" && has) {
            options.meshScale = std::atof(argv[++i]);
            if (!(options.meshScale > 0)) return usage();
        } else if (a == "--fixed-base") options.fixedBase = true;
        else if (a == "--floating-base") options.fixedBase = false;
        else if (a == "--no-actuators") noActuators = true;
        else if (a == "--no-sensors") noSensors = true;
        else if (a == "--overwrite") options.overwrite = true;
        else if (a == "--dry-run") dryRun = true;
        else if (a == "--assimp" && has) options.assimp = argv[++i];
        else if (a == "--python" && has) options.python = argv[++i];
        else if (a == "--recipe" && has) recipePath = argv[++i];
        else if (a.find(":=") != std::string::npos) options.xacroArgs.push_back(a);
        else if (!a.empty() && a[0] != '-' && options.input.empty()) options.input = a;
        else return usage();
    }
    if (options.input.empty() && recipePath.empty()) return usage();
    if (options.outputRoot.empty()) {
        options.outputRoot = (std::filesystem::path{terra::library::defaultLibraryPath()}.parent_path() / "assets").string();
    }
    im::ImportPlan plan;
    if (!recipePath.empty()) {
        const auto read = im::readRecipe(recipePath, options);
        for (const auto& e : read.errors) std::fprintf(stderr, "terra_import: [error] recipe_invalid：%s\n", e.c_str());
        if (!read.recipe) return 2;
        options = read.recipe->options;
        plan = im::analyzeRecipe(*read.recipe);
    } else {
        plan = im::analyze(options);
    }
    if (noActuators) {
        for (auto& act : plan.actuators) act.enabled = false;
    }
    if (noSensors) {
        for (auto& s : plan.sensors) s.enabled = false;
    }
    std::fprintf(stderr, "terra_import: %s：%zu 个连杆、%zu 个活动关节、%zu 个网格、提议 %zu 个执行器与 %zu 个传感器\n", plan.input.c_str(),
                 plan.links, plan.joints, plan.meshes.size(), plan.actuators.size(), plan.sensors.size());
    if (dryRun) {
        printIssues(plan.issues);
        std::printf("{\"type\":\"import\",\"dry_run\":true,\"ok\":%s,\"name\":\"%s\",\"issues\":%zu}\n", plan.ok() ? "true" : "false",
                    escape(plan.name).c_str(), plan.issues.size());
        return plan.ok() ? 0 : 4;
    }
    const auto result = im::execute(plan, options, terra::physics::mujoco::composeWorld);
    printIssues(result.issues);
    std::printf("{\"type\":\"import\",\"ok\":%s,\"name\":\"%s\",\"directory\":\"%s\",\"scene\":\"%s\",\"report\":\"%s\",\"settled_height\":%.6g}\n",
                result.ok ? "true" : "false", escape(plan.name).c_str(), escape(result.directory).c_str(), escape(result.scene).c_str(),
                escape(result.report).c_str(), result.settledHeight);
    return result.ok ? 0 : 4;
}
