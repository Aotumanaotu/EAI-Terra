// 模型导入（M4-08a）。见 terra/import/importer.hpp。

#include "terra/import/importer.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <set>
#include <sstream>

#include <nlohmann/json.hpp>

#include "mesh.hpp"
#include "process.hpp"
#include "tinyxml2.h"
#include "urdf.hpp"

namespace terra::importer {

namespace {

namespace fs = std::filesystem;
namespace dt = detail;
using Json = nlohmann::ordered_json;
using Severity = ImportIssue::Severity;
using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;  // w、x、y、z
using Mat3 = std::array<std::array<double, 3>, 3>;

void issue(std::vector<ImportIssue>& issues, Severity s, std::string code, std::string message, std::string hint = {}) {
    issues.push_back({s, std::move(code), std::move(message), std::move(hint)});
}

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

std::string readText(const fs::path& path) {
    std::ifstream in(path, std::ios::binary);
    return std::string{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

// ---- 旋转 ----

Quat quatMul(const Quat& a, const Quat& b) {
    return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3], a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1], a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

// URDF 的 rpy：绕固定轴依次转 roll（X）、pitch（Y）、yaw（Z），即 R = Rz(yaw) · Ry(pitch) · Rx(roll)
Quat rpyQuat(const Vec3& rpy) {
    const Quat qx{std::cos(rpy[0] / 2), std::sin(rpy[0] / 2), 0, 0};
    const Quat qy{std::cos(rpy[1] / 2), 0, std::sin(rpy[1] / 2), 0};
    const Quat qz{std::cos(rpy[2] / 2), 0, 0, std::sin(rpy[2] / 2)};
    return quatMul(qz, quatMul(qy, qx));
}

Mat3 quatMatrix(const Quat& q) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    return {{{1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)},
             {2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)},
             {2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)}}};
}

// 不叫 apply：参数是 std::array，按实参查找会找到 std::apply
Vec3 rotate(const Mat3& m, const Vec3& v) {
    return {m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2], m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2]};
}

struct Frame {
    Vec3 p{0, 0, 0};
    Quat q{1, 0, 0, 0};
    Frame compose(const dt::Origin& o) const {
        const Vec3 d = rotate(quatMatrix(q), o.xyz);
        return {{p[0] + d[0], p[1] + d[1], p[2] + d[2]}, quatMul(q, rpyQuat(o.rpy))};
    }
};

std::string num(double v) {
    if (v == 0.0) return "0";
    std::ostringstream s;
    s.precision(10);
    s << v;
    return s.str();
}

std::string vec(const std::array<double, 3>& v) { return num(v[0]) + " " + num(v[1]) + " " + num(v[2]); }
std::string quatText(const Quat& q) { return num(q[0]) + " " + num(q[1]) + " " + num(q[2]) + " " + num(q[3]); }

bool identity(const dt::Origin& o) { return o.rpy == Vec3{0, 0, 0}; }

// ---- 包目录 ----

// 在 root 下（深度不超过 depth）查找含 package.xml 的目录，读出包名
void discoverPackages(const fs::path& root, int depth, std::map<std::string, std::string>& out, std::size_t& budget) {
    std::error_code ec;
    if (depth < 0 || budget == 0 || !fs::is_directory(root, ec)) return;
    const fs::path manifest = root / "package.xml";
    if (fs::exists(manifest, ec)) {
        tinyxml2::XMLDocument doc;
        if (doc.LoadFile(manifest.string().c_str()) == tinyxml2::XML_SUCCESS) {
            const auto* pkg = doc.FirstChildElement("package");
            const auto* name = pkg ? pkg->FirstChildElement("name") : nullptr;
            if (name != nullptr && name->GetText() != nullptr) out.emplace(name->GetText(), root.string());
        }
    }
    for (fs::directory_iterator it(root, fs::directory_options::skip_permission_denied, ec), end; !ec && it != end; it.increment(ec)) {
        if (--budget == 0) return;
        if (!it->is_directory(ec)) continue;
        const std::string n = it->path().filename().string();
        if (n.empty() || n[0] == '.' || n == "build" || n == "install" || n == "log" || n == "node_modules") continue;
        discoverPackages(it->path(), depth - 1, out, budget);
    }
}

std::map<std::string, std::string> findPackages(const fs::path& inputDir, const std::map<std::string, std::string>& explicitPackages) {
    std::map<std::string, std::string> out;
    // 从模型所在目录往上找到所在的包（含 package.xml 的目录），先查它的上一级（同级的包），再查上两级：
    // 如 ROBOTS/turtlebot4/turtlebot4_description/urdf/standard → 包 turtlebot4_description → ROBOTS/turtlebot4 → ROBOTS/
    // （找到同在 ROBOTS/ 下的 create3_sim/irobot_create_common/irobot_create_description）。不在包里时查模型所在目录的上一级
    std::error_code ec;
    fs::path package;
    for (fs::path p = inputDir; !p.empty(); p = p.parent_path()) {
        if (fs::exists(p / "package.xml", ec)) {
            package = p;
            break;
        }
        if (p == p.parent_path()) break;
    }
    std::size_t budget = 20000;
    if (!package.empty()) {
        discoverPackages(package.parent_path(), 3, out, budget);
        if (package.parent_path().has_parent_path()) discoverPackages(package.parent_path().parent_path(), 4, out, budget);
    } else {
        discoverPackages(inputDir.parent_path(), 4, out, budget);
    }
    // ROS 环境中的包（只在已 source 时有）
    if (const char* prefixes = std::getenv("AMENT_PREFIX_PATH")) {
        std::stringstream ss(prefixes);
        std::string prefix;
        while (std::getline(ss, prefix, ':')) {
            for (fs::directory_iterator it(fs::path{prefix} / "share", ec), end; !ec && it != end; it.increment(ec)) {
                if (fs::exists(it->path() / "package.xml", ec)) out.emplace(it->path().filename().string(), it->path().string());
            }
        }
    }
    for (const auto& [name, dir] : explicitPackages) out[name] = fs::absolute(dir).lexically_normal().string();
    return out;
}

// package://、file:// 与相对路径 → 绝对路径；找不到包时 missingPackage 为包名
std::string resolveUri(const std::string& uri, const fs::path& baseDir, const std::map<std::string, std::string>& packages,
                       std::string& missingPackage) {
    const std::string pkgPrefix = "package://";
    const std::string filePrefix = "file://";
    if (uri.rfind(pkgPrefix, 0) == 0) {
        const std::string rest = uri.substr(pkgPrefix.size());
        const auto slash = rest.find('/');
        const std::string pkg = rest.substr(0, slash);
        const auto it = packages.find(pkg);
        if (it == packages.end()) {
            missingPackage = pkg;
            return {};
        }
        return (fs::path{it->second} / (slash == std::string::npos ? std::string{} : rest.substr(slash + 1))).lexically_normal().string();
    }
    if (uri.rfind(filePrefix, 0) == 0) return fs::path{uri.substr(filePrefix.size())}.lexically_normal().string();
    const fs::path p{uri};
    return (p.is_absolute() ? p : baseDir / p).lexically_normal().string();
}

// ---- 质量与提议 ----

double subtreeMass(const dt::UrdfModel& m, const std::string& link) {
    double mass = 0.0;
    if (const auto* l = m.findLink(link); l != nullptr && l->inertial) mass += std::max(0.0, l->inertial->mass);
    for (const auto& j : m.joints) {
        if (j.parent == link) mass += subtreeMass(m, j.child);
    }
    return mass;
}

std::string rootLink(const dt::UrdfModel& m) {
    for (const auto& l : m.links) {
        if (m.jointWithChild(l.name) == nullptr) return l.name;
    }
    return m.links.front().name;
}

void proposeSensors(const dt::UrdfModel& m, ImportPlan& plan) {
    const auto pick = [&](const std::vector<std::string>& keys, const std::function<int(const std::string&)>& score) -> std::string {
        std::string best;
        int bestScore = -1;
        for (const auto& l : m.links) {
            const std::string n = lower(l.name);
            bool match = false;
            for (const auto& k : keys) match = match || n.find(k) != std::string::npos;
            if (!match) continue;
            const int s = score(n);
            if (s > bestScore) {
                bestScore = s;
                best = l.name;
            }
        }
        return best;
    };
    const auto none = [](const std::string&) { return 0; };
    const std::string lidar = pick({"lidar", "laser", "scan", "lds"}, none);
    if (!lidar.empty()) {
        plan.sensors.push_back({"lidar2d", lidar, "scan", false, true, "连杆名“" + lidar + "”像激光雷达：提议 2D 激光雷达（360 束、0.12～12 m、10 Hz）", {}});
    }
    const std::string camera = pick({"camera", "oakd", "realsense", "d435", "rgb"}, [](const std::string& n) {
        return (n.find("optical") != std::string::npos ? 2 : 0) + (n.find("rgb") != std::string::npos || n.find("color") != std::string::npos ? 1 : 0);
    });
    if (!camera.empty()) {
        const bool optical = lower(camera).find("optical") != std::string::npos;
        plan.sensors.push_back({"camera", camera, "camera", optical, true,
                                "连杆名“" + camera + "”像相机：提议 RGB 与深度相机（640 × 480、60°、15 Hz）" +
                                    (optical ? "，按光学坐标系（z 向前）安装" : "，按 ROS 机体约定（x 向前）安装"),
                                {}});
    }
    const std::string imu = pick({"imu"}, none);
    if (!imu.empty()) plan.sensors.push_back({"imu", imu, "imu", false, true, "连杆名“" + imu + "”像 IMU：提议 IMU", {}});
}

}  // namespace

ImportOptions defaultOptions() {
    ImportOptions o;
    o.xacroDir = TERRA_IMPORT_XACRO_DIR;
    o.xacroScript = TERRA_IMPORT_XACRO_SCRIPT;
    return o;
}

bool ImportPlan::ok() const noexcept {
    return std::none_of(issues.begin(), issues.end(), [](const ImportIssue& i) { return i.severity == Severity::Error; });
}

const char* severityName(Severity s) noexcept {
    switch (s) {
        case Severity::Info: return "info";
        case Severity::Warning: return "warning";
        case Severity::Error: return "error";
    }
    return "info";
}

std::string sanitizeName(const std::string& name) {
    std::string out;
    for (const char c : name) {
        const auto u = static_cast<unsigned char>(c);
        out += (std::isalnum(u) != 0 && u < 128) || c == '_' || c == '-' ? c : '_';
    }
    if (out.empty()) out = "robot";
    // 场景的实例名要以字母或下划线开头（如 “07-physics” → “robot_07-physics”）
    if (std::isalpha(static_cast<unsigned char>(out[0])) == 0 && out[0] != '_') out = "robot_" + out;
    return out;
}

ImportPlan analyze(const ImportOptions& options) {
    ImportPlan plan;
    std::error_code ec;
    const fs::path input = fs::absolute(options.input, ec).lexically_normal();
    plan.input = input.string();
    if (!fs::is_regular_file(input, ec)) {
        issue(plan.issues, Severity::Error, "file_not_found", "找不到文件：" + plan.input, "检查路径；支持 .urdf、.xacro、.urdf.xacro 与 MJCF（.xml）");
        return plan;
    }
    const std::string text = readText(input);
    const fs::path baseDir = input.parent_path();
    const std::string ext = lower(input.extension().string());
    const auto mujocoAt = text.find("<mujoco");
    const auto robotAt = text.find("<robot");

    // 1. MJCF：不转换，只生成场景
    if (mujocoAt != std::string::npos && (robotAt == std::string::npos || mujocoAt < robotAt)) {
        plan.kind = SourceKind::Mjcf;
        plan.name = sanitizeName(options.name.empty() ? input.stem().string() : options.name);
        plan.robotName = input.stem().string();
        issue(plan.issues, Severity::Info, "mjcf_as_is", "MJCF 文件，直接使用（不转换）", "生成的场景引用原文件；移动原文件后需要重新导入");
        if (text.find("<actuator") == std::string::npos) {
            issue(plan.issues, Severity::Info, "no_actuators", "MJCF 中没有执行器，导入后只能被动运动",
                  "在 MJCF 中添加 <actuator>，或在场景文件的 controllers 中加 PD 层");
        }
        return plan;
    }
    if (robotAt == std::string::npos) {
        issue(plan.issues, Severity::Error, "unknown_format", "不认识的文件：既不是 URDF（<robot>）也不是 MJCF（<mujoco>）",
              "支持 .urdf、.xacro、.urdf.xacro 与 MJCF（.xml）");
        return plan;
    }

    // 2. 包目录与 xacro
    plan.packages = findPackages(baseDir, options.packages);
    // M4-08b：只出现在 xacro 文本中的包（如 Create 3 的 Gazebo 插件参数引用 irobot_create_control）：给占位路径
    for (const auto& p : options.placeholderPackages) {
        if (plan.packages.count(p) != 0) continue;
        plan.packages[p] = "/nonexistent/terra-placeholder/" + p;
        issue(plan.issues, Severity::Info, "placeholder_package", "包“" + p + "”按导入配方给占位路径（只在文本中引用，不读取其中的文件）");
    }
    std::string urdf = text;
    plan.kind = SourceKind::Urdf;
    if (ext == ".xacro" || dt::looksLikeXacro(text)) {
        plan.kind = SourceKind::Xacro;
        const std::string script = options.xacroScript;
        if (!dt::programAvailable(options.python)) {
            issue(plan.issues, Severity::Error, "python_missing", "文件中有 xacro，展开需要 " + options.python + "，但找不到",
                  "安装 Python 3（Ubuntu：sudo apt install python3），或先在别处展开为 .urdf 后导入");
            return plan;
        }
        if (script.empty() || !fs::exists(script, ec) || !fs::exists(fs::path{options.xacroDir} / "xacro" / "__init__.py", ec)) {
            issue(plan.issues, Severity::Error, "xacro_missing", "找不到随 Terra 提供的 xacro（" + options.xacroDir + "）",
                  "检查 Terra 的安装是否完整（third_party/xacro 与 engine/import/tools/xacro_expand.py）");
            return plan;
        }
        // -X utf8：不论终端的语言设置，xacro 的输出与错误信息都按 UTF-8（错误信息中有中文）
        std::vector<std::string> args{options.python, "-X", "utf8", script, "--xacro-dir", options.xacroDir};
        for (const auto& [name, dir] : plan.packages) {
            args.push_back("--package");
            args.push_back(name + "=" + dir);
        }
        args.push_back(plan.input);
        for (const auto& a : options.xacroArgs) args.push_back(a);
        const auto run = dt::runProcess(args);
        if (!run.started || run.exitCode != 0) {
            const bool package = run.errors.find("找不到包") != std::string::npos || run.errors.find("PackageNotFoundError") != std::string::npos;
            issue(plan.issues, Severity::Error, package ? "unknown_package" : "xacro_failed",
                  "xacro 展开失败：" + run.errors.substr(0, 600),
                  package ? "用 --package 包名=目录 指定包目录（含 package.xml 的目录）" : "检查 xacro 文件与参数（名字:=值）");
            return plan;
        }
        urdf = run.output;
        issue(plan.issues, Severity::Info, "xacro_expanded", "文件中有 xacro，已用随 Terra 提供的 xacro 2.1.1 展开（不需要 ROS）");
    }
    plan.urdfText = urdf;

    // 3. 解析
    std::string error;
    auto model = dt::parseUrdf(urdf, error);
    if (!model) {
        issue(plan.issues, Severity::Error, "urdf_invalid", "URDF 无法解析：" + error, "检查文件是否完整（xacro 展开后的结果也可能有错）");
        return plan;
    }
    // M4-08b：导入配方的关节类型覆盖（如 Create 3 的轮跌落关节改为固定）
    for (const auto& [joint, type] : options.jointTypes) {
        bool found = false;
        for (auto& j : model->joints) {
            if (j.name != joint) continue;
            found = true;
            if (type == "fixed") {
                j.type = "fixed";
                plan.jointTypes[joint] = type;
                issue(plan.issues, Severity::Info, "joint_fixed", "关节 " + joint + " 按导入配方改为固定");
            }
        }
        if (!found) issue(plan.issues, Severity::Warning, "recipe_unknown_joint", "导入配方中的关节 " + joint + " 在模型中不存在", "检查配方的 joints");
        else if (type != "fixed") issue(plan.issues, Severity::Error, "recipe_joint_type", "关节类型只能改为 fixed（实际为 " + type + "）", "修改配方的 joints");
    }
    plan.robotName = model->name;
    plan.name = sanitizeName(options.name.empty() ? (model->name.empty() ? input.stem().string() : model->name) : options.name);
    plan.links = model->links.size();
    for (const auto& j : model->joints) plan.joints += j.type == "fixed" ? 0u : 1u;

    // 4. 网格与贴图
    std::set<std::string> reportedPackages;
    bool assimpReported = false;
    const auto addMesh = [&](const dt::Geometry& g) {
        if (g.type != dt::Geometry::Type::Mesh) return;
        for (const auto& m : plan.meshes) {
            if (m.uri == g.filename) return;
        }
        MeshSource m;
        m.uri = g.filename;
        std::string missing;
        m.resolved = resolveUri(g.filename, baseDir, plan.packages, missing);
        m.format = lower(fs::path{g.filename}.extension().string());
        if (!m.format.empty() && m.format[0] == '.') m.format.erase(0, 1);
        m.convert = m.format != "stl" && m.format != "obj";
        if (!missing.empty()) {
            if (reportedPackages.insert(missing).second) {
                issue(plan.issues, Severity::Error, "unknown_package", "找不到包“" + missing + "”（网格 " + g.filename + " 等）",
                      "用 --package " + missing + "=<目录> 指定（含 package.xml 的目录），或把包放在模型旁边");
            }
        } else if (!fs::is_regular_file(m.resolved, ec)) {
            issue(plan.issues, Severity::Error, "missing_mesh", "网格文件不存在：" + g.filename + "（解析为 " + m.resolved + "）",
                  "确认网格已下载；package:// 路径对应的包目录可用 --package 指定");
            m.resolved.clear();
        } else if (m.convert && !dt::programAvailable(options.assimp) && !assimpReported) {
            assimpReported = true;
            issue(plan.issues, Severity::Error, "assimp_missing", "网格 " + g.filename + " 是 " + m.format + " 格式，MuJoCo 不能直接读取，转换需要 assimp 命令行工具",
                  "安装：sudo apt install assimp-utils");
        }
        if (!m.resolved.empty() && !m.convert) {
            // 尺寸检查：只看 STL 与 OBJ（Collada 有自己的单位，转换后再看）
            std::optional<dt::Bounds> b;
            if (m.format == "stl") b = dt::stlBounds(m.resolved, error);
            else if (auto obj = dt::readObj(m.resolved, error)) b = obj->bounds();
            const double scale = g.hasScale ? std::max({std::fabs(g.scale[0]), std::fabs(g.scale[1]), std::fabs(g.scale[2])}) : options.meshScale;
            if (b && b->extent() * scale > 5.0) {
                issue(plan.issues, Severity::Warning, "suspicious_scale",
                      "网格 " + g.filename + " 缩放后最长边为 " + num(b->extent() * scale) + " m，可能以毫米为单位",
                      g.hasScale ? "检查 URDF 中该网格的 scale" : "导入时设置网格缩放 0.001（命令行 --mesh-scale 0.001，向导中“网格缩放”）");
            }
        }
        plan.meshes.push_back(m);
    };
    for (const auto& l : model->links) {
        for (const auto& v : l.visuals) {
            addMesh(v.geometry);
            if (!v.material.texture.empty()) {
                std::string missing;
                const auto path = resolveUri(v.material.texture, baseDir, plan.packages, missing);
                if (path.empty() || !fs::exists(path, ec)) {
                    issue(plan.issues, Severity::Warning, "missing_texture", "连杆 " + l.name + " 的贴图找不到：" + v.material.texture,
                          "贴图缺失时只用颜色；确认贴图文件与包目录");
                }
            }
        }
        for (const auto& c : l.collisions) addMesh(c.geometry);
    }
    const std::size_t converting = static_cast<std::size_t>(std::count_if(plan.meshes.begin(), plan.meshes.end(), [](const MeshSource& m) { return m.convert; }));
    if (converting > 0 && !assimpReported) {
        issue(plan.issues, Severity::Info, "mesh_conversion", std::to_string(converting) + " 个网格不是 STL / OBJ，导入时用 assimp 转换为 OBJ（按材质拆分，保留颜色与贴图）");
    }

    // 5. 关节与惯量
    const std::string root = rootLink(*model);
    plan.fixedBase = options.fixedBase.value_or(lower(root) == "world");
    for (const auto& j : model->joints) {
        if (j.type == "planar") {
            issue(plan.issues, Severity::Warning, "unsupported_joint", "关节 " + j.name + " 是 planar，MuJoCo 没有对应的关节类型，按固定处理",
                  "需要平面运动时可改为两个 prismatic 与一个 continuous");
        } else if (j.type == "floating") {
            issue(plan.issues, Severity::Warning, "unsupported_joint", "关节 " + j.name + " 是 floating，按自由关节处理");
        }
        if (j.mimic) {
            issue(plan.issues, Severity::Warning, "mimic_joint", "关节 " + j.name + " 带 mimic（跟随另一个关节），导入后不强制跟随",
                  "需要时在 MJCF 中用 equality 约束连接两个关节");
        }
        const auto* child = model->findLink(j.child);
        if (j.type != "fixed" && child != nullptr && (!child->inertial || child->inertial->mass <= 0.0)) {
            issue(plan.issues, Severity::Warning, "no_inertia", "活动连杆 " + j.child + " 没有质量或惯量",
                  "导入时给 0.01 kg 的最小惯量；请在 URDF 中补上 <inertial> 以得到正确的动力学");
        }
    }

    // 6. 执行器（URDF 没有执行器：有 transmission 时只给其中的关节，否则给全部活动关节）
    std::set<std::string> transmitted(model->transmissionJoints.begin(), model->transmissionJoints.end());
    for (const auto& j : model->joints) {
        if (j.type != "continuous" && j.type != "revolute" && j.type != "prismatic") continue;
        if (!transmitted.empty() && transmitted.count(j.name) == 0) continue;
        const double mass = std::max(0.1, subtreeMass(*model, j.child));
        ProposedActuator a;
        a.joint = j.name;
        a.forceLimit = j.hasLimit && j.effort > 0 ? j.effort : 0.0;
        if (j.type == "continuous") {
            a.type = "velocity";
            a.gain = std::clamp(2.0 * mass, 0.5, 50.0);
            a.reason = "continuous 关节（如轮子）：速度执行器，kv 按下游质量 " + num(mass) + " kg 估计";
        } else {
            a.type = "position";
            a.gain = std::clamp(50.0 * mass, 5.0, 2000.0);
            a.limited = j.hasLimit && j.upper > j.lower;
            a.rangeMin = j.lower;
            a.rangeMax = j.upper;
            a.reason = j.type + " 关节：位置执行器，kp 按下游质量 " + num(mass) + " kg 估计" + (a.limited ? "，范围取关节限位" : "");
        }
        plan.actuators.push_back(a);
    }
    if (!plan.actuators.empty()) {
        issue(plan.issues, Severity::Info, "default_actuators",
              "URDF 没有执行器，按关节类型提议 " + std::to_string(plan.actuators.size()) + " 个（可在向导中修改或取消）");
    } else if (plan.joints == 0) {
        issue(plan.issues, Severity::Info, "no_joints", "模型没有活动关节");
    }

    // 7. 传感器
    proposeSensors(*model, plan);
    for (const auto& s : plan.sensors) issue(plan.issues, Severity::Info, "sensor_proposed", s.reason, "可在向导中取消；参数可在场景文件中修改");
    return plan;
}

// ---- 执行 ----

namespace {

struct MeshFiles {
    std::string collision;  // 相对 meshes/ 的文件名
    struct Part {
        std::string file;
        std::array<double, 4> rgba{0.7, 0.7, 0.7, 1.0};
        std::string texture;  // 相对 textures/ 的文件名
    };
    std::vector<Part> visual;  // 空：视觉用 collision 文件
    dt::Bounds bounds;         // 网格自身单位（未乘 URDF 的 scale）
};

std::string floorXml() {
    return R"(<!-- 地面（由 Terra 导入时生成，与模型的许可无关） -->
<mujoco model="terra floor">
  <asset>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
      markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>
  <worldbody>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
  </worldbody>
</mujoco>
)";
}

Json issuesJson(const std::vector<ImportIssue>& issues) {
    Json out = Json::array();
    for (const auto& i : issues) {
        out.push_back({{"severity", severityName(i.severity)}, {"code", i.code}, {"message", i.message}, {"hint", i.hint}});
    }
    return out;
}

}  // namespace

ImportResult execute(const ImportPlan& plan, const ImportOptions& options, const terra::scene::WorldFactory& factory) {
    ImportResult result;
    result.issues = plan.issues;
    if (!plan.ok()) {
        issue(result.issues, Severity::Error, "plan_has_errors", "分析中有错误，未导入", "先处理上面的错误");
        return result;
    }
    std::error_code ec;
    const fs::path dir = fs::absolute(fs::path{options.outputRoot} / plan.name, ec).lexically_normal();
    result.directory = dir.string();
    if (fs::exists(dir, ec)) {
        if (!options.overwrite) {
            issue(result.issues, Severity::Error, "output_exists", "导入目录已存在：" + dir.string(), "换一个名字，或选择覆盖（命令行 --overwrite）");
            return result;
        }
        if (!fs::exists(dir / "import-report.json", ec)) {
            issue(result.issues, Severity::Error, "output_not_ours", "目录已存在且不是导入目录，不覆盖：" + dir.string(), "换一个名字或目录");
            return result;
        }
        fs::remove_all(dir, ec);
    }
    fs::create_directories(dir / "meshes", ec);
    fs::create_directories(dir / "textures", ec);
    const std::string base = plan.name;
    result.scene = (dir / (base + ".terra-scene.json")).string();
    result.report = (dir / "import-report.json").string();
    Json sensorsJson = Json::array();
    bool hasFloor = true;
    std::string rootEntity;
    double placeZ = 0.0;

    if (plan.kind == SourceKind::Mjcf) {
        result.model = plan.input;
        const std::string text = readText(plan.input);
        hasFloor = text.find("type=\"plane\"") == std::string::npos;  // Menagerie 的 scene.xml 自带地面
    } else {
        std::string error;
        auto model = dt::parseUrdf(plan.urdfText, error);
        if (!model) {
            issue(result.issues, Severity::Error, "urdf_invalid", error);
            return result;
        }
        for (auto& j : model->joints) {
            if (plan.jointTypes.count(j.name) != 0) j.type = plan.jointTypes.at(j.name);
        }
        // 网格：复制或转换
        std::map<std::string, MeshFiles> files;
        std::set<std::string> usedNames;
        const auto uniqueName = [&](const std::string& stem, const std::string& ext) {
            std::string n = sanitizeName(stem) + ext;
            for (int k = 2; usedNames.count(n) != 0; ++k) n = sanitizeName(stem) + "_" + std::to_string(k) + ext;
            usedNames.insert(n);
            return n;
        };
        std::set<std::string> textureNames;
        const auto copyTexture = [&](const std::string& path) -> std::string {
            if (path.empty() || !fs::exists(path, ec)) return {};
            if (lower(fs::path{path}.extension().string()) != ".png") {
                issue(result.issues, Severity::Warning, "texture_format", "贴图 " + fs::path{path}.filename().string() + " 不是 PNG，MuJoCo 不能读取，只用颜色",
                      "把贴图转换为 PNG 后重新导入");
                return {};
            }
            const std::string n = sanitizeName(fs::path{path}.stem().string()) + ".png";
            if (textureNames.insert(n).second) fs::copy_file(path, dir / "textures" / n, fs::copy_options::overwrite_existing, ec);
            return n;
        };
        for (const auto& m : plan.meshes) {
            MeshFiles f;
            const std::string stem = fs::path{m.resolved}.stem().string();
            if (!m.convert) {
                f.collision = uniqueName(stem, "." + m.format);
                fs::copy_file(m.resolved, dir / "meshes" / f.collision, fs::copy_options::overwrite_existing, ec);
                if (m.format == "stl") {
                    if (auto b = dt::stlBounds(m.resolved, error)) f.bounds = *b;
                } else if (auto obj = dt::readObj(m.resolved, error)) {
                    f.bounds = obj->bounds();
                }
            } else {
                const auto converted = dt::convertWithAssimp(options.assimp, m.resolved, (dir / ".convert").string());
                if (!converted.mesh) {
                    issue(result.issues, Severity::Error, "mesh_conversion_failed", "网格 " + m.uri + " 转换失败：" + converted.error,
                          "确认文件能被 assimp 打开（assimp info <文件>）；或先转换为 STL / OBJ 后修改 URDF");
                    continue;
                }
                if (!converted.warning.empty()) issue(result.issues, Severity::Warning, "mesh_orientation_unchecked", converted.warning);
                issue(result.issues, Severity::Info, "mesh_converted",
                      "网格 " + fs::path{m.uri}.filename().string() + " 已转换：" + std::to_string(converted.mesh->parts.size()) + " 个材质，" +
                          converted.transform);
                f.bounds = converted.mesh->bounds();
                dt::MeshPart merged;
                for (const auto& part : converted.mesh->parts) merged.positions.insert(merged.positions.end(), part.positions.begin(), part.positions.end());
                f.collision = uniqueName(stem, ".obj");
                dt::writeObj((dir / "meshes" / f.collision).string(), merged, false);
                for (std::size_t k = 0; k < converted.mesh->parts.size(); ++k) {
                    const auto& part = converted.mesh->parts[k];
                    MeshFiles::Part p;
                    p.rgba = part.rgba;
                    p.texture = copyTexture(part.texture);
                    p.file = uniqueName(stem + "_visual" + std::to_string(k), ".obj");
                    dt::writeObj((dir / "meshes" / p.file).string(), part, !p.texture.empty());
                    f.visual.push_back(p);
                }
            }
            files[m.uri] = f;
        }
        fs::remove_all(dir / ".convert", ec);

        // MJCF
        tinyxml2::XMLDocument doc;
        doc.InsertEndChild(doc.NewComment((" 由 Terra 从 " + fs::path{plan.input}.filename().string() + " 导入（M4-08a）；源模型的许可见导入报告 ").c_str()));
        auto* mj = doc.NewElement("mujoco");
        doc.InsertEndChild(mj);
        mj->SetAttribute("model", plan.name.c_str());
        auto* compiler = mj->InsertNewChildElement("compiler");
        compiler->SetAttribute("angle", "radian");
        compiler->SetAttribute("meshdir", "meshes");
        compiler->SetAttribute("texturedir", "textures");
        compiler->SetAttribute("autolimits", "true");
        auto* option = mj->InsertNewChildElement("option");
        option->SetAttribute("timestep", "0.002");
        option->SetAttribute("integrator", "implicitfast");
        auto* defaults = mj->InsertNewChildElement("default");
        auto* visualClass = defaults->InsertNewChildElement("default");
        visualClass->SetAttribute("class", "visual");
        auto* vg = visualClass->InsertNewChildElement("geom");
        vg->SetAttribute("group", 2);
        vg->SetAttribute("contype", 0);
        vg->SetAttribute("conaffinity", 0);
        vg->SetAttribute("density", 0);
        auto* collisionClass = defaults->InsertNewChildElement("default");
        collisionClass->SetAttribute("class", "collision");
        auto* cg = collisionClass->InsertNewChildElement("geom");
        cg->SetAttribute("group", 3);
        cg->SetAttribute("density", 0);  // 质量只取 URDF 的 <inertial>
        auto* asset = mj->InsertNewChildElement("asset");
        std::map<std::string, std::string> meshAssets;  // 文件 + 缩放 → 资产名
        const auto meshAsset = [&](const std::string& file, const Vec3& scale) {
            const std::string key = file + "|" + vec(scale);
            auto it = meshAssets.find(key);
            if (it != meshAssets.end()) return it->second;
            const std::string name = "mesh" + std::to_string(meshAssets.size()) + "_" + fs::path{file}.stem().string();
            auto* e = asset->InsertNewChildElement("mesh");
            e->SetAttribute("name", name.c_str());
            e->SetAttribute("file", file.c_str());
            if (scale != Vec3{1, 1, 1}) e->SetAttribute("scale", vec(scale).c_str());
            // 质量只取 URDF 的 <inertial>（几何体 density 0），网格的惯量不使用；按壳计算，开放的网格（如单个面）也能编译
            e->SetAttribute("inertia", "shell");
            meshAssets.emplace(key, name);
            return name;
        };
        std::map<std::string, std::string> materials;  // rgba + 贴图 → 材质名
        const auto material = [&](const std::array<double, 4>& rgba, const std::string& texture) {
            const std::string key = num(rgba[0]) + " " + num(rgba[1]) + " " + num(rgba[2]) + " " + num(rgba[3]) + "|" + texture;
            auto it = materials.find(key);
            if (it != materials.end()) return it->second;
            const std::string name = "material" + std::to_string(materials.size());
            if (!texture.empty()) {
                auto* t = asset->InsertNewChildElement("texture");
                t->SetAttribute("name", (name + "_texture").c_str());
                t->SetAttribute("type", "2d");
                t->SetAttribute("file", texture.c_str());
            }
            auto* e = asset->InsertNewChildElement("material");
            e->SetAttribute("name", name.c_str());
            e->SetAttribute("rgba", (num(rgba[0]) + " " + num(rgba[1]) + " " + num(rgba[2]) + " " + num(rgba[3])).c_str());
            if (!texture.empty()) e->SetAttribute("texture", (name + "_texture").c_str());
            materials.emplace(key, name);
            return name;
        };
        const auto meshScale = [&](const dt::Geometry& g) {
            return g.hasScale ? g.scale : Vec3{options.meshScale, options.meshScale, options.meshScale};
        };
        const auto placeGeom = [&](tinyxml2::XMLElement* geom, const dt::Origin& o) {
            if (o.xyz != Vec3{0, 0, 0}) geom->SetAttribute("pos", vec(o.xyz).c_str());
            if (!identity(o)) geom->SetAttribute("quat", quatText(rpyQuat(o.rpy)).c_str());
        };
        const auto setShape = [&](tinyxml2::XMLElement* geom, const dt::Geometry& g, const std::string& meshFile) -> bool {
            switch (g.type) {
                case dt::Geometry::Type::Box:
                    geom->SetAttribute("type", "box");
                    geom->SetAttribute("size", vec({g.size[0] / 2, g.size[1] / 2, g.size[2] / 2}).c_str());
                    return true;
                case dt::Geometry::Type::Cylinder:
                    geom->SetAttribute("type", "cylinder");
                    geom->SetAttribute("size", (num(g.radius) + " " + num(g.length / 2)).c_str());
                    return true;
                case dt::Geometry::Type::Sphere:
                    geom->SetAttribute("type", "sphere");
                    geom->SetAttribute("size", num(g.radius).c_str());
                    return true;
                case dt::Geometry::Type::Mesh:
                    if (meshFile.empty()) return false;
                    geom->SetAttribute("type", "mesh");
                    geom->SetAttribute("mesh", meshAsset(meshFile, meshScale(g)).c_str());
                    return true;
                case dt::Geometry::Type::None: return false;
            }
            return false;
        };
        // 正运动学（零位）求碰撞几何体的最低点，用来把机器人放在地面上
        double minZ = 1e9;
        const auto lowest = [&](const Frame& link, const dt::Origin& o, const dt::Geometry& g) {
            const Frame f = link.compose(o);
            const Mat3 r = quatMatrix(f.q);
            std::vector<Vec3> corners;
            if (g.type == dt::Geometry::Type::Box) {
                for (int k = 0; k < 8; ++k) corners.push_back({(k & 1 ? 0.5 : -0.5) * g.size[0], (k & 2 ? 0.5 : -0.5) * g.size[1], (k & 4 ? 0.5 : -0.5) * g.size[2]});
            } else if (g.type == dt::Geometry::Type::Cylinder) {
                for (int k = 0; k < 8; ++k) corners.push_back({(k & 1 ? 1 : -1) * g.radius, (k & 2 ? 1 : -1) * g.radius, (k & 4 ? 0.5 : -0.5) * g.length});
            } else if (g.type == dt::Geometry::Type::Sphere) {
                minZ = std::min(minZ, f.p[2] - g.radius);
                return;
            } else if (g.type == dt::Geometry::Type::Mesh) {
                const auto it = files.find(g.filename);
                if (it == files.end() || !it->second.bounds.valid) return;
                const auto& b = it->second.bounds;
                const Vec3 s = meshScale(g);
                for (int k = 0; k < 8; ++k) {
                    corners.push_back({(k & 1 ? b.max[0] : b.min[0]) * s[0], (k & 2 ? b.max[1] : b.min[1]) * s[1], (k & 4 ? b.max[2] : b.min[2]) * s[2]});
                }
            }
            for (const auto& c : corners) minZ = std::min(minZ, f.p[2] + rotate(r, c)[2]);
        };
        const std::set<std::string> actuated = [&] {
            std::set<std::string> s;
            for (const auto& a : plan.actuators) {
                if (a.enabled) s.insert(a.joint);
            }
            return s;
        }();
        std::function<void(tinyxml2::XMLElement*, const std::string&, const Frame&, bool)> addBody;
        addBody = [&](tinyxml2::XMLElement* parent, const std::string& linkName, const Frame& world, bool isWorld) {
            const auto* link = model->findLink(linkName);
            if (link == nullptr) return;
            const auto* joint = model->jointWithChild(linkName);
            const bool moving = joint != nullptr && joint->type != "fixed" && joint->type != "planar";
            if (isWorld) {
                // world 连杆：没有惯量；几何体直接放在世界中
            } else if (link->inertial && link->inertial->mass > 0.0) {
                const auto& in = *link->inertial;
                auto* e = parent->InsertNewChildElement("inertial");
                e->SetAttribute("pos", vec(in.origin.xyz).c_str());
                if (!identity(in.origin)) e->SetAttribute("quat", quatText(rpyQuat(in.origin.rpy)).c_str());
                e->SetAttribute("mass", num(in.mass).c_str());
                e->SetAttribute("fullinertia",
                                (num(in.ixx) + " " + num(in.iyy) + " " + num(in.izz) + " " + num(in.ixy) + " " + num(in.ixz) + " " + num(in.iyz)).c_str());
            } else if (moving || (joint == nullptr && !plan.fixedBase)) {
                auto* e = parent->InsertNewChildElement("inertial");
                e->SetAttribute("pos", "0 0 0");
                e->SetAttribute("mass", "0.01");
                e->SetAttribute("diaginertia", "1e-05 1e-05 1e-05");
            }
            for (const auto& v : link->visuals) {
                const auto it = v.geometry.type == dt::Geometry::Type::Mesh ? files.find(v.geometry.filename) : files.end();
                if (it != files.end() && !it->second.visual.empty()) {
                    for (const auto& part : it->second.visual) {  // 转换后按材质拆分的部分
                        auto* geom = parent->InsertNewChildElement("geom");
                        geom->SetAttribute("class", "visual");
                        placeGeom(geom, v.origin);
                        setShape(geom, v.geometry, part.file);
                        geom->SetAttribute("material", material(part.rgba, part.texture).c_str());
                    }
                    continue;
                }
                auto* geom = parent->InsertNewChildElement("geom");
                geom->SetAttribute("class", "visual");
                placeGeom(geom, v.origin);
                if (!setShape(geom, v.geometry, it != files.end() ? it->second.collision : std::string{})) {
                    parent->DeleteChild(geom);
                    continue;
                }
                std::string missing;
                const std::string texture = v.material.texture.empty()
                                                ? std::string{}
                                                : copyTexture(resolveUri(v.material.texture, fs::path{plan.input}.parent_path(), plan.packages, missing));
                if (v.material.rgba || !texture.empty()) geom->SetAttribute("material", material(v.material.rgba.value_or(dt::Rgba{1, 1, 1, 1}), texture).c_str());
            }
            for (const auto& c : link->collisions) {
                const auto it = c.geometry.type == dt::Geometry::Type::Mesh ? files.find(c.geometry.filename) : files.end();
                auto* geom = parent->InsertNewChildElement("geom");
                geom->SetAttribute("class", "collision");
                if (!c.name.empty()) geom->SetAttribute("name", (link->name + "/" + c.name).c_str());
                placeGeom(geom, c.origin);
                if (!setShape(geom, c.geometry, it != files.end() ? it->second.collision : std::string{})) {
                    parent->DeleteChild(geom);
                    continue;
                }
                lowest(world, c.origin, c.geometry);
            }
            if (link->collisions.empty()) {
                for (const auto& v : link->visuals) lowest(world, v.origin, v.geometry);
            }
            for (const auto& j : model->joints) {
                if (j.parent != linkName) continue;
                auto* body = parent->InsertNewChildElement("body");
                body->SetAttribute("name", j.child.c_str());
                if (j.origin.xyz != Vec3{0, 0, 0}) body->SetAttribute("pos", vec(j.origin.xyz).c_str());
                if (!identity(j.origin)) body->SetAttribute("quat", quatText(rpyQuat(j.origin.rpy)).c_str());
                if (j.type == "revolute" || j.type == "continuous" || j.type == "prismatic") {
                    auto* e = body->InsertNewChildElement("joint");
                    e->SetAttribute("name", j.name.c_str());
                    e->SetAttribute("type", j.type == "prismatic" ? "slide" : "hinge");
                    e->SetAttribute("axis", vec(j.axis).c_str());
                    if (j.type != "continuous" && j.hasLimit && j.upper > j.lower) e->SetAttribute("range", (num(j.lower) + " " + num(j.upper)).c_str());
                    if (j.damping > 0) e->SetAttribute("damping", num(j.damping).c_str());
                    if (j.friction > 0) e->SetAttribute("frictionloss", num(j.friction).c_str());
                } else if (j.type == "floating") {
                    body->InsertNewChildElement("freejoint")->SetAttribute("name", j.name.c_str());
                }
                addBody(body, j.child, world.compose(j.origin), false);
            }
        };
        auto* worldbody = mj->InsertNewChildElement("worldbody");
        const std::string root = rootLink(*model);
        if (lower(root) == "world") {
            // 根连杆为 world：其子连杆直接放在世界中（固定基座）
            addBody(worldbody, root, Frame{}, true);
            rootEntity.clear();
        } else {
            auto* body = worldbody->InsertNewChildElement("body");
            body->SetAttribute("name", root.c_str());
            if (!plan.fixedBase) body->InsertNewChildElement("freejoint")->SetAttribute("name", (root + "_free").c_str());
            addBody(body, root, Frame{}, false);
            rootEntity = root;
            if (!plan.fixedBase && minZ < 1e8) {
                placeZ = -minZ + 0.002;
                body->SetAttribute("pos", ("0 0 " + num(placeZ)).c_str());
            }
        }
        if (!actuated.empty()) {
            auto* actuators = mj->InsertNewChildElement("actuator");
            for (const auto& a : plan.actuators) {
                if (!a.enabled) continue;
                auto* e = actuators->InsertNewChildElement(a.type == "velocity" ? "velocity" : "position");
                e->SetAttribute("name", a.joint.c_str());
                e->SetAttribute("joint", a.joint.c_str());
                if (a.type == "velocity") {
                    e->SetAttribute("kv", num(a.gain).c_str());
                } else {
                    e->SetAttribute("kp", num(a.gain).c_str());
                    e->SetAttribute("dampratio", "1");
                    if (a.limited) e->SetAttribute("ctrlrange", (num(a.rangeMin) + " " + num(a.rangeMax)).c_str());
                }
                if (a.forceLimit > 0) e->SetAttribute("forcerange", (num(-a.forceLimit) + " " + num(a.forceLimit)).c_str());
            }
        }
        result.model = (dir / (base + ".xml")).string();
        if (doc.SaveFile(result.model.c_str()) != tinyxml2::XML_SUCCESS) {
            issue(result.issues, Severity::Error, "write_failed", "写入失败：" + result.model);
            return result;
        }
        // 传感器
        for (const auto& s : plan.sensors) {
            if (!s.enabled) continue;
            const std::string entity = plan.name + "/" + s.link;
            const std::string name = plan.name + "/" + s.name;
            Json sensor;
            if (s.type == "lidar2d") {
                sensor = {{"type", "lidar2d"}, {"name", name}, {"entity", entity}, {"angleMinDeg", -180}, {"angleMaxDeg", 179},
                          {"samples", 360}, {"rangeMin", 0.12}, {"rangeMax", 12}, {"rateHz", 10}};
            } else if (s.type == "camera") {
                // Terra 相机看向 −Z、+Y 向上：光学坐标系（z 前、y 下）绕 X 转 180°；ROS 机体约定（x 前、z 上）见四元数
                const Json q = s.optical ? Json::array({0, 1, 0, 0}) : Json::array({0.5, 0.5, -0.5, -0.5});
                sensor = {{"type", "camera"}, {"name", name}, {"entity", entity}, {"rotation", {{"quat", q}}}, {"width", 640},
                          {"height", 480}, {"fovyDeg", 60}, {"near", 0.05}, {"far", 10}, {"rateHz", 15}};
            } else if (s.type == "imu") {
                sensor = {{"type", "imu"}, {"name", name}, {"entity", entity}};
            } else {
                continue;
            }
            // M4-08b：导入配方给出的参数覆盖默认值
            if (!s.parameters.empty()) {
                const Json extra = Json::parse(s.parameters, nullptr, false);
                if (extra.is_object()) {
                    for (auto it = extra.begin(); it != extra.end(); ++it) sensor[it.key()] = it.value();
                }
            }
            sensorsJson.push_back(std::move(sensor));
        }
    }

    // 场景
    if (hasFloor) {
        std::ofstream(dir / "floor.xml") << floorXml();
    }
    Json scene;
    scene["format"] = "terra.scene";
    scene["version"] = 1;
    scene["name"] = plan.name + "（由 " + fs::path{plan.input}.filename().string() + " 导入）";
    scene["title"] = options.title.empty() ? plan.name : options.title;  // M4-08c：资源库卡片上的短标题
    Json models = Json::array();
    models.push_back({{"name", plan.name}, {"mjcf", plan.kind == SourceKind::Mjcf ? plan.input : base + ".xml"}});
    if (hasFloor) models.push_back({{"name", "floor"}, {"mjcf", "floor.xml"}});
    for (const auto& m : options.extraModels) {  // M4-08b：配方中的其他模型（如房间），路径相对导入目录
        const fs::path rel = fs::path{m.mjcf}.lexically_relative(dir);
        Json item = {{"name", m.name}, {"mjcf", rel.empty() ? m.mjcf : rel.string()}};
        if (m.position != std::array<double, 3>{0, 0, 0}) item["position"] = Json::array({m.position[0], m.position[1], m.position[2]});
        models.push_back(std::move(item));
    }
    scene["models"] = std::move(models);
    if (!sensorsJson.empty()) scene["sensors"] = std::move(sensorsJson);
    if (plan.kind != SourceKind::Mjcf) scene["physics"] = {{"integrator", "implicitfast"}};  // 与生成的 MJCF 相同，地面不另设
    scene["view"] = {{"camera", {{"position", Json::array({1.6, -1.3, 1.0 + placeZ})}, {"target", Json::array({0, 0, placeZ})}}}};
    if (options.viewPosition) scene["view"]["camera"]["position"] = Json::array({(*options.viewPosition)[0], (*options.viewPosition)[1], (*options.viewPosition)[2]});
    if (options.viewTarget) scene["view"]["camera"]["target"] = Json::array({(*options.viewTarget)[0], (*options.viewTarget)[1], (*options.viewTarget)[2]});
    std::ofstream(result.scene) << scene.dump(2) << '\n';

    // 检查：加载并推进 1 s
    if (factory) {
        auto loaded = terra::scene::loadScene(result.scene, factory);
        for (const auto& i : loaded.issues) {
            const bool err = i.severity == terra::scene::SceneIssue::Severity::Error;
            issue(result.issues, err ? Severity::Error : Severity::Warning, err ? "scene_load_failed" : "scene_warning", i.where + " " + i.message,
                  err ? "查看生成的 MJCF 与导入报告；常见原因是网格无效或关节定义不完整" : "");
        }
        if (loaded.ok()) {
            auto& world = *loaded.scene->world;
            bool stepped = true;
            for (int k = 0; k < 500 && stepped; ++k) {
                if (const auto e = world.step()) {
                    issue(result.issues, Severity::Error, "simulation_failed", "推进第 " + std::to_string(e->step) + " 步出错：" + e->message,
                          "常见原因：碰撞几何体相互重叠、质量或惯量过小");
                    stepped = false;
                }
            }
            const auto snap = world.snapshot();
            if (stepped && !snap->diagnostics.finite) {
                issue(result.issues, Severity::Error, "simulation_failed", "推进 1 s 后状态不是有限值", "检查惯量与执行器增益");
            }
            if (!rootEntity.empty()) {
                if (const auto* e = world.description().findEntity(plan.name + "/" + rootEntity)) {
                    result.settledHeight = snap->entities[e->id.value - 1].pose.position.z;
                    if (!plan.fixedBase && result.settledHeight < -0.05) {
                        issue(result.issues, Severity::Error, "fell_through_floor", "推进 1 s 后机器人掉到地面以下（" + num(result.settledHeight) + " m）",
                              "检查碰撞几何体是否存在、是否太小");
                    }
                }
            }
            issue(result.issues, Severity::Info, "check_passed_steps", "加载并推进 1 s：" + std::string{stepped ? "没有仿真错误" : "出错"} +
                                                                          (rootEntity.empty() ? std::string{} : "，根连杆高度 " + num(result.settledHeight) + " m"));
        }
    }
    result.ok = std::none_of(result.issues.begin(), result.issues.end(), [](const ImportIssue& i) { return i.severity == Severity::Error; });

    // 报告
    Json report;
    report["format"] = "terra.import-report";
    report["version"] = 1;
    report["input"] = plan.input;
    report["kind"] = plan.kind == SourceKind::Mjcf ? "mjcf" : plan.kind == SourceKind::Xacro ? "xacro" : "urdf";
    report["robot"] = plan.robotName;
    report["name"] = plan.name;
    report["ok"] = result.ok;
    report["links"] = plan.links;
    report["joints"] = plan.joints;
    report["fixed_base"] = plan.fixedBase;
    Json packages = Json::object();
    for (const auto& [n, d] : plan.packages) packages[n] = d;
    report["packages"] = std::move(packages);
    Json meshes = Json::array();
    for (const auto& m : plan.meshes) meshes.push_back({{"uri", m.uri}, {"resolved", m.resolved}, {"format", m.format}, {"converted", m.convert}});
    report["meshes"] = std::move(meshes);
    Json actuators = Json::array();
    for (const auto& a : plan.actuators) {
        actuators.push_back({{"joint", a.joint}, {"type", a.type}, {"gain", a.gain}, {"force_limit", a.forceLimit}, {"enabled", a.enabled}});
    }
    report["actuators"] = std::move(actuators);
    Json sensors = Json::array();
    for (const auto& s : plan.sensors) sensors.push_back({{"type", s.type}, {"link", s.link}, {"enabled", s.enabled}});
    report["sensors"] = std::move(sensors);
    report["settled_height"] = result.settledHeight;
    report["issues"] = issuesJson(result.issues);
    report["license_note"] = "导入的网格与模型沿用源文件的许可；请保留源仓库的 LICENSE";
    std::ofstream(result.report) << report.dump(2) << '\n';
    writeRecipe((dir / "import.terra-import.json").string(), plan, options);  // M4-08b：这次导入实际使用的选择
    return result;
}

// ---- 导入配方（M4-08b）----

RecipeResult readRecipe(const std::string& path, const ImportOptions& base) {
    RecipeResult out;
    const auto err = [&](const std::string& where, const std::string& message) { out.errors.push_back(where + "：" + message); };
    std::error_code ec;
    const fs::path file = fs::absolute(path, ec).lexically_normal();
    std::ifstream in(file);
    if (!in) {
        err("", "无法读取导入配方：" + file.string());
        return out;
    }
    Json j = Json::parse(in, nullptr, false);
    if (j.is_discarded() || !j.is_object()) {
        err("", "不是有效的 JSON 对象：" + file.string());
        return out;
    }
    const std::set<std::string> allowed{"format", "version", "input", "name", "title", "xacroArgs", "packages", "placeholderPackages",
                                        "meshScale", "fixedBase", "joints", "actuators", "sensors", "extraModels", "view"};
    for (auto it = j.begin(); it != j.end(); ++it) {
        if (allowed.count(it.key()) == 0) err("/" + it.key(), "未知字段“" + it.key() + "”");
    }
    if (!j.contains("format") || !j["format"].is_string() || j["format"].get<std::string>() != "terra.import-recipe") err("/format", "应为 “terra.import-recipe”");
    if (!j.contains("version") || !j["version"].is_number_integer() || j["version"].get<long long>() != 1) err("/version", "应为 1");
    const fs::path dir = file.parent_path();
    const auto rel = [&](const std::string& p) { return (fs::path{p}.is_absolute() ? fs::path{p} : dir / p).lexically_normal().string(); };
    ImportRecipe r;
    r.path = file.string();
    r.options = base;
    const auto str = [&](const char* key, std::string& target) {
        if (!j.contains(key)) return;
        if (!j[key].is_string()) err(std::string{"/"} + key, "应为字符串");
        else target = j[key].get<std::string>();
    };
    const auto strings = [&](const char* key, std::vector<std::string>& target) {
        if (!j.contains(key)) return;
        if (!j[key].is_array()) {
            err(std::string{"/"} + key, "应为字符串数组");
            return;
        }
        for (std::size_t i = 0; i < j[key].size(); ++i) {
            if (!j[key][i].is_string()) err(std::string{"/"} + key + "/" + std::to_string(i), "应为字符串");
            else target.push_back(j[key][i].get<std::string>());
        }
    };
    const auto vec3 = [&](const Json& v, const std::string& where) -> std::optional<std::array<double, 3>> {
        if (!v.is_array() || v.size() != 3 || !v[0].is_number() || !v[1].is_number() || !v[2].is_number()) {
            err(where, "应为 3 个数值");
            return std::nullopt;
        }
        return std::array<double, 3>{v[0].get<double>(), v[1].get<double>(), v[2].get<double>()};
    };
    std::string input;
    str("input", input);
    if (input.empty()) err("/input", "缺少要导入的文件（相对配方文件）");
    else r.options.input = rel(input);
    str("name", r.options.name);
    str("title", r.options.title);
    r.options.xacroArgs.clear();
    strings("xacroArgs", r.options.xacroArgs);
    strings("placeholderPackages", r.options.placeholderPackages);
    if (j.contains("packages")) {
        if (!j["packages"].is_object()) err("/packages", "应为对象（包名 → 目录）");
        else {
            for (auto it = j["packages"].begin(); it != j["packages"].end(); ++it) {
                if (!it.value().is_string()) err("/packages/" + it.key(), "应为目录（字符串）");
                else r.options.packages[it.key()] = rel(it.value().get<std::string>());
            }
        }
    }
    if (j.contains("meshScale")) {
        if (!j["meshScale"].is_number() || !(j["meshScale"].get<double>() > 0)) err("/meshScale", "应为正数");
        else r.options.meshScale = j["meshScale"].get<double>();
    }
    if (j.contains("fixedBase")) {
        if (!j["fixedBase"].is_boolean()) err("/fixedBase", "应为 true 或 false");
        else r.options.fixedBase = j["fixedBase"].get<bool>();
    }
    if (j.contains("joints")) {
        if (!j["joints"].is_object()) err("/joints", "应为对象（关节名 → \"fixed\"）");
        else {
            for (auto it = j["joints"].begin(); it != j["joints"].end(); ++it) {
                if (!it.value().is_string() || it.value().get<std::string>() != "fixed") err("/joints/" + it.key(), "目前只能改为 \"fixed\"");
                else r.options.jointTypes[it.key()] = "fixed";
            }
        }
    }
    if (j.contains("actuators")) {
        std::vector<ProposedActuator> list;
        if (!j["actuators"].is_array()) err("/actuators", "应为数组");
        for (std::size_t i = 0; j["actuators"].is_array() && i < j["actuators"].size(); ++i) {
            const Json& a = j["actuators"][i];
            const std::string where = "/actuators/" + std::to_string(i);
            if (!a.is_object() || !a.contains("joint") || !a["joint"].is_string() || !a.contains("type") || !a["type"].is_string()) {
                err(where, "应为含 joint 与 type 的对象");
                continue;
            }
            ProposedActuator p;
            p.joint = a["joint"].get<std::string>();
            p.type = a["type"].get<std::string>();
            if (p.type != "velocity" && p.type != "position") err(where + "/type", "应为 velocity 或 position");
            for (auto it = a.begin(); it != a.end(); ++it) {
                const std::string k = it.key();
                if (k == "joint" || k == "type") continue;
                if (k == "limited") {
                    if (!it.value().is_boolean()) err(where + "/limited", "应为 true 或 false");
                    else p.limited = it.value().get<bool>();
                    continue;
                }
                if (k != "gain" && k != "forceLimit" && k != "rangeMin" && k != "rangeMax") {
                    err(where + "/" + k, "未知字段“" + k + "”");
                    continue;
                }
                if (!it.value().is_number()) {
                    err(where + "/" + k, "应为数值");
                    continue;
                }
                const double v = it.value().get<double>();
                if (k == "gain") p.gain = v;
                else if (k == "forceLimit") p.forceLimit = v;
                else if (k == "rangeMin") p.rangeMin = v;
                else p.rangeMax = v;
            }
            if (!(p.gain > 0)) err(where + "/gain", "应为正数");
            p.reason = "导入配方";
            list.push_back(p);
        }
        r.actuators = std::move(list);
    }
    if (j.contains("sensors")) {
        std::vector<ProposedSensor> list;
        if (!j["sensors"].is_array()) err("/sensors", "应为数组");
        for (std::size_t i = 0; j["sensors"].is_array() && i < j["sensors"].size(); ++i) {
            const Json& v = j["sensors"][i];
            const std::string where = "/sensors/" + std::to_string(i);
            if (!v.is_object() || !v.contains("type") || !v["type"].is_string() || !v.contains("name") || !v["name"].is_string() ||
                !v.contains("link") || !v["link"].is_string()) {
                err(where, "应为含 type、name、link 的对象");
                continue;
            }
            ProposedSensor p;
            p.type = v["type"].get<std::string>();
            p.name = v["name"].get<std::string>();
            p.link = v["link"].get<std::string>();
            if (p.type != "lidar2d" && p.type != "camera" && p.type != "imu") err(where + "/type", "应为 lidar2d、camera 或 imu");
            if (v.contains("optical")) {
                if (!v["optical"].is_boolean()) err(where + "/optical", "应为 true 或 false");
                else p.optical = v["optical"].get<bool>();
            }
            Json extra = Json::object();
            for (auto it = v.begin(); it != v.end(); ++it) {
                if (it.key() != "type" && it.key() != "name" && it.key() != "link" && it.key() != "optical") extra[it.key()] = it.value();
            }
            p.parameters = extra.empty() ? std::string{} : extra.dump();
            p.reason = "导入配方";
            list.push_back(p);
        }
        r.sensors = std::move(list);
    }
    if (j.contains("extraModels")) {
        if (!j["extraModels"].is_array()) err("/extraModels", "应为数组");
        for (std::size_t i = 0; j["extraModels"].is_array() && i < j["extraModels"].size(); ++i) {
            const Json& m = j["extraModels"][i];
            const std::string where = "/extraModels/" + std::to_string(i);
            if (!m.is_object() || !m.contains("name") || !m["name"].is_string() || !m.contains("mjcf") || !m["mjcf"].is_string()) {
                err(where, "应为含 name 与 mjcf 的对象");
                continue;
            }
            ExtraModel e;
            e.name = m["name"].get<std::string>();
            e.mjcf = rel(m["mjcf"].get<std::string>());
            if (m.contains("position")) {
                if (const auto p = vec3(m["position"], where + "/position")) e.position = *p;
            }
            if (!fs::exists(e.mjcf, ec)) err(where + "/mjcf", "文件不存在：" + e.mjcf);
            r.options.extraModels.push_back(e);
        }
    }
    if (j.contains("view")) {
        const Json& v = j["view"];
        if (!v.is_object()) err("/view", "应为对象");
        else {
            if (v.contains("position")) r.options.viewPosition = vec3(v["position"], "/view/position");
            if (v.contains("target")) r.options.viewTarget = vec3(v["target"], "/view/target");
        }
    }
    if (out.errors.empty()) out.recipe = std::move(r);
    return out;
}

ImportPlan analyzeRecipe(const ImportRecipe& recipe) {
    ImportPlan plan = analyze(recipe.options);
    if (!plan.ok() || plan.kind == SourceKind::Mjcf) return plan;
    std::string error;
    auto model = dt::parseUrdf(plan.urdfText, error);
    if (!model) return plan;
    for (auto& j : model->joints) {
        if (plan.jointTypes.count(j.name) != 0) j.type = plan.jointTypes.at(j.name);
    }
    if (recipe.actuators) {
        plan.actuators = *recipe.actuators;
        for (const auto& a : plan.actuators) {
            bool movable = false;
            for (const auto& j : model->joints) movable = movable || (j.name == a.joint && j.type != "fixed");
            if (!movable) {
                issue(plan.issues, Severity::Error, "recipe_actuator_joint", "导入配方中的执行器关节 " + a.joint + " 不存在或是固定关节", "检查配方的 actuators");
            }
        }
        plan.issues.erase(std::remove_if(plan.issues.begin(), plan.issues.end(), [](const ImportIssue& i) { return i.code == "default_actuators"; }),
                          plan.issues.end());
    }
    if (recipe.sensors) {
        plan.sensors = *recipe.sensors;
        plan.issues.erase(std::remove_if(plan.issues.begin(), plan.issues.end(), [](const ImportIssue& i) { return i.code == "sensor_proposed"; }),
                          plan.issues.end());
        for (const auto& s : plan.sensors) {
            if (model->findLink(s.link) == nullptr) {
                issue(plan.issues, Severity::Error, "recipe_sensor_link", "导入配方中的传感器 " + s.name + " 的连杆 " + s.link + " 不存在", "检查配方的 sensors");
            }
        }
    }
    issue(plan.issues, Severity::Info, "recipe_applied", "按导入配方：" + recipe.path);
    return plan;
}

bool writeRecipe(const std::string& path, const ImportPlan& plan, const ImportOptions& options) {
    Json j;
    j["format"] = "terra.import-recipe";
    j["version"] = 1;
    j["input"] = plan.input;
    j["name"] = plan.name;
    if (!options.title.empty()) j["title"] = options.title;
    j["xacroArgs"] = options.xacroArgs;
    if (!options.placeholderPackages.empty()) j["placeholderPackages"] = options.placeholderPackages;
    if (!options.packages.empty()) {
        Json p = Json::object();
        for (const auto& [n, d] : options.packages) p[n] = d;
        j["packages"] = std::move(p);
    }
    j["meshScale"] = options.meshScale;
    j["fixedBase"] = plan.fixedBase;
    if (!plan.jointTypes.empty()) {
        Json jt = Json::object();
        for (const auto& [n, t] : plan.jointTypes) jt[n] = t;
        j["joints"] = std::move(jt);
    }
    Json actuators = Json::array();
    for (const auto& a : plan.actuators) {
        if (!a.enabled) continue;
        Json e = {{"joint", a.joint}, {"type", a.type}, {"gain", a.gain}};
        if (a.forceLimit > 0) e["forceLimit"] = a.forceLimit;
        if (a.limited) {
            e["limited"] = true;
            e["rangeMin"] = a.rangeMin;
            e["rangeMax"] = a.rangeMax;
        }
        actuators.push_back(std::move(e));
    }
    j["actuators"] = std::move(actuators);
    Json sensors = Json::array();
    for (const auto& s : plan.sensors) {
        if (!s.enabled) continue;
        Json e = {{"type", s.type}, {"name", s.name}, {"link", s.link}};
        if (s.optical) e["optical"] = true;
        if (!s.parameters.empty()) {
            const Json extra = Json::parse(s.parameters, nullptr, false);
            if (extra.is_object()) {
                for (auto it = extra.begin(); it != extra.end(); ++it) e[it.key()] = it.value();
            }
        }
        sensors.push_back(std::move(e));
    }
    j["sensors"] = std::move(sensors);
    if (!options.extraModels.empty()) {
        Json models = Json::array();
        for (const auto& m : options.extraModels) {
            models.push_back({{"name", m.name}, {"mjcf", m.mjcf}, {"position", Json::array({m.position[0], m.position[1], m.position[2]})}});
        }
        j["extraModels"] = std::move(models);
    }
    std::ofstream out(path);
    out << j.dump(2) << '\n';
    return static_cast<bool>(out);
}

}  // namespace terra::importer
