// terra_import：模型导入的命令行程序（M4-08a）。分析 → 写出导入目录 → 加载并推进 1 s 检查。
//
// 用法：terra_import <URDF / xacro / MJCF> [--name 名字] [--output 目录] [--package 包名=目录]... [--mesh-scale 比例]
//                    [--fixed-base | --floating-base] [--no-actuators] [--no-sensors] [--overwrite] [--dry-run]
//                    [--assimp 程序] [--python 程序] [名字:=值]...（xacro 参数）
//   --output   导入目录的上级；默认为资源库目录下的 assets/（$TERRA_LIBRARY 或 ~/.local/share/eai-terra/）
//   --dry-run  只分析，不写文件
//   --recipe   （M4-08b）按导入配方（*.terra-import.json）导入；配方中的选择优先，命令行可另给 --output、--overwrite、--dry-run
//   --library  （M4-08c）导入成功后把导入目录登记到该资源库（不给时不写库；查看器启动时会扫描默认资源库目录下的 assets/）
//   --export-pack <导入目录> <文件.terra-pack>  （M4-08c）把导入目录打包为资产包
//   --import-pack <文件.terra-pack>  （M4-08c）解包到 --output（默认同上），可加 --overwrite、--library
//   --environment <glb / gltf>  （M5-02b）环境导入：按顶层节点拆成场景元素，写到 <--output>/<名字>/（默认为资源库目录下的
//       environments/）；--manifest <清单>（默认在 glTF 所在目录及上一级找 manifest.json）、--collision boxes|hulls、
//       --max-boxes N（每个节点最多几个盒体，默认 24）、--overwrite、--library；导入后加载并推进 0.2 s 检查
// 标准错误：每个问题一行（级别、代码、说明、处理办法）；标准输出：最后一行 {"type":"import",...}
// 退出码：0 成功；2 参数错误；4 有错误（分析或检查）

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>

#include "terra/import/environment.hpp"
#include "terra/import/importer.hpp"
#include "terra/library/library.hpp"
#include "terra/library/pack.hpp"
#include "terra/physics/mujoco/mujoco_backend.hpp"

namespace {

namespace im = terra::importer;

int usage() {
    std::fprintf(stderr,
                 "用法：terra_import <URDF / xacro / MJCF> [--name 名字] [--output 目录] [--package 包名=目录]... [--mesh-scale 比例]\n"
                 "                   [--fixed-base | --floating-base] [--no-actuators] [--no-sensors] [--overwrite] [--dry-run]\n"
                 "                   [--assimp 程序] [--python 程序] [名字:=值]...\n"
                 "       terra_import --recipe <导入配方.terra-import.json> [--output 目录] [--overwrite] [--dry-run]\n"
                 "       terra_import --export-pack <导入目录> <文件.terra-pack> [--overwrite]\n"
                 "       terra_import --import-pack <文件.terra-pack> [--output 目录] [--overwrite]\n"
                 "       terra_import --environment <glb / gltf> [--name 名字] [--output 目录] [--manifest 清单] [--collision boxes|hulls]\n"
                 "                    [--max-boxes N] [--overwrite]\n"
                 "       （导入与解包可加 --library <库文件>：成功后登记到资源库）\n");
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

// 把目录登记到资源库（M4-08c）；失败只警告
void registerDirectory(const std::string& libraryPath, const std::string& directory) {
    if (libraryPath.empty()) return;
    std::string error;
    auto library = terra::library::Library::open(libraryPath, error);
    if (!library) {
        std::fprintf(stderr, "terra_import: [warning] library：资源库无法打开：%s\n", error.c_str());
        return;
    }
    const auto scan = library->scanAssets({{directory, terra::library::AssetSource::Imported}});
    for (const auto& e : scan.errors) std::fprintf(stderr, "terra_import: [warning] library：%s\n", e.c_str());
    std::fprintf(stderr, "terra_import: 已登记到资源库 %s：%zu 个文件、%zu 个机器人\n", library->path().c_str(), scan.files, scan.robots);
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
    std::string recipePath, libraryPath, exportDir, exportFile, importFile;
    std::string environment, manifest, collision = "boxes";  // M5-02b
    int maxBoxes = 24;
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
        else if (a == "--library" && has) libraryPath = argv[++i];
        else if (a == "--export-pack" && i + 2 < argc) {
            exportDir = argv[++i];
            exportFile = argv[++i];
        } else if (a == "--import-pack" && has) importFile = argv[++i];
        else if (a == "--environment" && has) environment = argv[++i];
        else if (a == "--manifest" && has) manifest = argv[++i];
        else if (a == "--collision" && has) {
            collision = argv[++i];
            if (collision != "boxes" && collision != "hulls") return usage();
        } else if (a == "--max-boxes" && has) {
            maxBoxes = std::atoi(argv[++i]);
            if (maxBoxes < 1) return usage();
        }
        else if (a.find(":=") != std::string::npos) options.xacroArgs.push_back(a);
        else if (!a.empty() && a[0] != '-' && options.input.empty()) options.input = a;
        else return usage();
    }
    if (!exportDir.empty()) {  // M4-08c：资产包
        if (!options.input.empty() || !recipePath.empty() || !importFile.empty()) return usage();
        const auto r = terra::library::exportPack(exportDir, exportFile, options.overwrite);
        if (!r.ok) std::fprintf(stderr, "terra_import: [error] pack：%s\n", r.error.c_str());
        std::printf("{\"type\":\"export_pack\",\"ok\":%s,\"name\":\"%s\",\"pack\":\"%s\",\"files\":%zu,\"bytes\":%llu}\n", r.ok ? "true" : "false",
                    escape(r.name).c_str(), escape(exportFile).c_str(), r.files, static_cast<unsigned long long>(r.bytes));
        return r.ok ? 0 : 4;
    }
    if (!environment.empty()) {  // M5-02b：环境导入
        if (!options.input.empty() || !recipePath.empty() || !importFile.empty()) return usage();
        namespace fs = std::filesystem;
        im::EnvironmentImportOptions env;
        env.input = environment;
        env.manifest = manifest;
        env.hulls = collision == "hulls";
        env.maxBoxesPerNode = maxBoxes;
        env.force = options.overwrite;
        const std::string name = options.name.empty() ? fs::path{environment}.stem().string() : options.name;
        env.name = name;
        const fs::path root = options.outputRoot.empty() ? fs::path{terra::library::defaultLibraryPath()}.parent_path() / "environments" : fs::path{options.outputRoot};
        env.output = (root / name).string();
        const auto r = im::importEnvironment(env);
        for (const auto& w : r.warnings) std::fprintf(stderr, "terra_import: [warning] environment：%s\n", w.c_str());
        for (const auto& e : r.errors) std::fprintf(stderr, "terra_import: [error] environment：%s\n", e.c_str());
        bool loaded = false;
        std::size_t instances = 0, geoms = 0;
        if (r.ok) {  // 加载检查：组合进物理世界，推进 0.2 s
            auto scene = terra::scene::loadScene(r.scene, terra::physics::mujoco::composeWorld);
            for (const auto& i : scene.issues) {
                std::fprintf(stderr, "terra_import: [%s] scene：%s %s\n", i.severity == terra::scene::SceneIssue::Severity::Error ? "error" : "warning",
                             i.where.c_str(), i.message.c_str());
            }
            if (scene.scene) {
                for (int k = 0; k < 100; ++k) {
                    if (scene.scene->world->step()) break;
                }
                instances = scene.scene->world->description().modelInstances.size();
                geoms = scene.scene->world->description().geoms.size();
                loaded = scene.scene->world->stepIndex() == 100;
            }
            if (loaded) registerDirectory(libraryPath, env.output);
        }
        std::printf("{\"type\":\"environment\",\"ok\":%s,\"scene\":\"%s\",\"manifest\":\"%s\",\"elements\":%zu,\"boxes\":%zu,\"meshes\":%zu,"
                    "\"parts\":%zu,\"inspection\":%zu,\"instances\":%zu,\"geoms\":%zu,\"warnings\":%zu}\n",
                    r.ok && loaded ? "true" : "false", escape(r.scene).c_str(), escape(r.manifest).c_str(), r.elements, r.boxes, r.meshes, r.parts,
                    r.inspection, instances, geoms, r.warnings.size());
        return r.ok && loaded ? 0 : 4;
    }
    if (options.input.empty() && recipePath.empty() && importFile.empty()) return usage();
    if (options.outputRoot.empty()) {
        options.outputRoot = (std::filesystem::path{terra::library::defaultLibraryPath()}.parent_path() / "assets").string();
    }
    if (!importFile.empty()) {
        if (!options.input.empty() || !recipePath.empty()) return usage();
        const auto r = terra::library::importPack(importFile, options.outputRoot, options.overwrite);
        if (!r.ok) std::fprintf(stderr, "terra_import: [error] pack：%s\n", r.error.c_str());
        if (r.ok) registerDirectory(libraryPath, r.directory);
        std::printf("{\"type\":\"import_pack\",\"ok\":%s,\"name\":\"%s\",\"directory\":\"%s\",\"files\":%zu,\"bytes\":%llu}\n",
                    r.ok ? "true" : "false", escape(r.name).c_str(), escape(r.directory).c_str(), r.files, static_cast<unsigned long long>(r.bytes));
        return r.ok ? 0 : 4;
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
    if (result.ok) registerDirectory(libraryPath, result.directory);
    std::printf("{\"type\":\"import\",\"ok\":%s,\"name\":\"%s\",\"directory\":\"%s\",\"scene\":\"%s\",\"report\":\"%s\",\"settled_height\":%.6g}\n",
                result.ok ? "true" : "false", escape(plan.name).c_str(), escape(result.directory).c_str(), escape(result.scene).c_str(),
                escape(result.report).c_str(), result.settledHeight);
    return result.ok ? 0 : 4;
}
