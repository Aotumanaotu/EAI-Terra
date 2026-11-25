#pragma once

// 模型导入（M4-08a，M4-08 设计稿 2026-11-25 评审通过）：URDF / xacro / MJCF → Terra 场景。
//
// 两步：analyze() 只读取与检查（展开 xacro、解析 package:// 与 $(find)、列出网格、提议执行器与传感器），
// 不写文件，供导入向导显示与修改；execute() 按（可能被用户修改过的）计划写出导入目录并加载检查。
//
// 导入目录 <outputRoot>/<名字>/：
//   <名字>.xml              MJCF（由 Terra 从 URDF 生成；可见几何体 group 2、碰撞几何体 group 3，与 Menagerie 相同）
//   meshes/、textures/      网格与贴图（Collada 等由 assimp 命令行工具转换为 OBJ，按材质拆分）
//   floor.xml               地面
//   <名字>.terra-scene.json 场景（机器人 + 地面 + 提议并选中的传感器）
//   import-report.json      导入报告（每一步与每个问题）
// MJCF 输入不转换，只生成引用原文件的场景。
//
// 不依赖 ROS：xacro 用随 Terra 提供的官方实现（third_party/xacro，经 engine/import/tools/xacro_expand.py 调用，需要 python3）；
// 网格转换调用 assimp 命令行工具（找不到时给出安装命令）。本模块不包含 MuJoCo 与图形头文件，检查时用调用方给的 WorldFactory。

#include <array>
#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "terra/scene/scene.hpp"

namespace terra::importer {

enum class SourceKind { Urdf, Xacro, Mjcf };

struct ImportIssue {
    enum class Severity { Info, Warning, Error };
    Severity severity = Severity::Info;
    std::string code;     // 稳定的标识（测试与界面用），如 missing_mesh、unknown_package
    std::string message;  // 发生了什么
    std::string hint;     // 怎么处理（可操作）
};

struct MeshSource {
    std::string uri;       // 模型中写的文件名（如 package://pkg/meshes/a.dae）
    std::string resolved;  // 解析后的绝对路径；空表示找不到
    std::string format;    // 小写扩展名：stl、obj、dae……
    bool convert = false;  // 需要 assimp 转换（MuJoCo 只读 STL、OBJ、MSH）
};

struct ProposedActuator {
    std::string joint;          // URDF 关节名
    std::string type;           // "velocity"（continuous 关节）或 "position"（revolute、prismatic）
    double gain = 0.0;          // velocity：kv；position：kp
    double forceLimit = 0.0;    // > 0 时为力限（取 URDF limit 的 effort）
    bool limited = false;       // position：控制范围取关节限位
    double rangeMin = 0.0;
    double rangeMax = 0.0;
    bool enabled = true;
    std::string reason;
};

struct ProposedSensor {
    std::string type;  // "lidar2d"、"camera"、"imu"
    std::string link;  // 安装的连杆
    std::string name;  // 传感器名（不含实例前缀）
    bool optical = false;  // camera：连杆是光学坐标系（z 向前、y 向下）；否则按 ROS 机体约定（x 向前、z 向上）
    bool enabled = true;
    std::string reason;
    std::string parameters;  // M4-08b：导入配方给出的其余字段（JSON 对象文本，覆盖默认参数）；空表示用默认值
};

// M4-08b：随机器人一起放进场景的其他模型（如房间），mjcf 为绝对路径
struct ExtraModel {
    std::string name;
    std::string mjcf;
    std::array<double, 3> position{0.0, 0.0, 0.0};
};

struct ImportOptions {
    std::string input;                             // URDF、xacro 或 MJCF 文件
    std::string name;                              // 导入后的名字；空表示取模型中的名字
    std::string title;                             // M4-08c：场景的短标题（资源库卡片）；空表示用 name
    std::string outputRoot;                        // 导入目录的上级
    std::map<std::string, std::string> packages;   // 包名 → 包目录（补充自动查找的结果）
    std::vector<std::string> xacroArgs;            // xacro 参数，名字:=值
    std::string python = "python3";
    std::string assimp = "assimp";
    std::string xacroDir;     // third_party/xacro
    std::string xacroScript;  // engine/import/tools/xacro_expand.py
    double meshScale = 1.0;   // 对没有写 scale 的网格额外缩放（如毫米 → 米为 0.001）
    std::optional<bool> fixedBase;  // 空：自动（URDF 的根连杆为 world 时固定，否则为浮动基座）
    bool overwrite = false;   // 导入目录已存在时覆盖
    // M4-08b：导入配方中的选择
    std::map<std::string, std::string> jointTypes;   // 关节名 → 新类型（目前只支持 "fixed"）
    std::vector<std::string> placeholderPackages;    // 只在 xacro 的文本中出现（如 Gazebo 插件参数）的包：给一个占位路径
    std::vector<ExtraModel> extraModels;
    std::optional<std::array<double, 3>> viewPosition;
    std::optional<std::array<double, 3>> viewTarget;
};

struct ImportPlan {
    SourceKind kind = SourceKind::Urdf;
    std::string input;      // 绝对路径
    std::string robotName;  // 模型中的名字
    std::string name;       // 导入后的名字（实例名与目录名）
    std::string urdfText;   // 展开后的 URDF（URDF、xacro）
    std::map<std::string, std::string> packages;  // 实际使用的包目录表（含自动找到的）
    std::vector<ImportIssue> issues;
    std::vector<MeshSource> meshes;
    std::vector<ProposedActuator> actuators;
    std::vector<ProposedSensor> sensors;
    bool fixedBase = false;
    std::size_t links = 0;
    std::size_t joints = 0;  // 不含 fixed（含配方改为 fixed 的关节）
    std::map<std::string, std::string> jointTypes;  // M4-08b：生效的关节类型覆盖
    bool ok() const noexcept;  // 没有错误级别的问题
};

// 默认选项：xacro 与包装脚本取 Terra 源码树中的位置（构建时写入）
ImportOptions defaultOptions();

ImportPlan analyze(const ImportOptions& options);

struct ImportResult {
    bool ok = false;
    std::string directory;  // 导入目录
    std::string model;      // MJCF
    std::string scene;      // 场景文件
    std::string report;     // 导入报告
    std::vector<ImportIssue> issues;  // 分析与执行两个阶段的全部问题
    double settledHeight = 0.0;       // 检查：推进 1 s 后根连杆的高度（m）
};

// ---- 导入配方（M4-08b，*.terra-import.json：format "terra.import-recipe"、version 1，严格校验）----
// 把导入时的选择写成文件，命令行（--recipe）与导入向导都能读；导入时把实际使用的选择写入导入目录的 import.terra-import.json。
//   input（相对配方文件）、name、xacroArgs、packages（包名 → 目录，相对配方文件）、placeholderPackages、meshScale、fixedBase、
//   joints（关节名 → "fixed"）、actuators（给出时替换提议：joint、type、gain、forceLimit、limited、rangeMin、rangeMax）、
//   sensors（给出时替换提议：type、name、link、optical 与场景文件中该类传感器的其余字段）、
//   extraModels（name、mjcf（相对配方文件）、position）、view（position、target）
struct ImportRecipe {
    std::string path;
    ImportOptions options;
    std::optional<std::vector<ProposedActuator>> actuators;
    std::optional<std::vector<ProposedSensor>> sensors;
};

struct RecipeResult {
    std::optional<ImportRecipe> recipe;
    std::vector<std::string> errors;  // “JSON 指针：原因”
};

// base 提供配方中没有的选项（输出目录、xacro 与 assimp 的位置等）
RecipeResult readRecipe(const std::string& path, const ImportOptions& base);
// 按配方分析：analyze() 之后用配方中的执行器与传感器替换提议，并检查它们引用的关节与连杆
ImportPlan analyzeRecipe(const ImportRecipe& recipe);
// 把计划与选项写成配方（路径写成绝对路径）
bool writeRecipe(const std::string& path, const ImportPlan& plan, const ImportOptions& options);

// 按计划写出导入目录并检查（factory 用于加载生成的场景并推进 1 s；为空时不检查）
ImportResult execute(const ImportPlan& plan, const ImportOptions& options, const terra::scene::WorldFactory& factory);

// 名字中只保留字母、数字、下划线与连字符（其余换成下划线），作为实例名与目录名
std::string sanitizeName(const std::string& name);

const char* severityName(ImportIssue::Severity severity) noexcept;

}  // namespace terra::importer
