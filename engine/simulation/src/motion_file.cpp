// 动作文件读取与校验（M3-04）。nlohmann/json 只在本文件中使用，不进入公共头。

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <set>

#include <nlohmann/json.hpp>

#include "terra/simulation/motion.hpp"

namespace terra::simulation {

namespace {

namespace fs = std::filesystem;
using Json = nlohmann::json;
using Severity = MotionIssue::Severity;

constexpr int kSupportedVersion = 1;

std::string kindOf(const Json& v) {
    if (v.is_object()) return "对象";
    if (v.is_array()) return "数组";
    if (v.is_string()) return "字符串";
    if (v.is_boolean()) return "布尔值";
    if (v.is_number()) return "数值";
    return "null";
}

struct Reader {
    std::vector<MotionIssue> issues;

    void error(const std::string& where, std::string message) { issues.push_back({Severity::Error, where, std::move(message)}); }
    void warning(const std::string& where, std::string message) {
        issues.push_back({Severity::Warning, where, std::move(message)});
    }
    bool hasErrors() const {
        for (const auto& i : issues) {
            if (i.severity == Severity::Error) return true;
        }
        return false;
    }

    bool object(const Json& v, const std::string& where, std::initializer_list<const char*> allowed) {
        if (!v.is_object()) {
            error(where, "应为对象，实际为" + kindOf(v));
            return false;
        }
        const std::set<std::string> names(allowed.begin(), allowed.end());
        for (auto it = v.begin(); it != v.end(); ++it) {
            if (names.count(it.key()) == 0) {
                std::string list;
                for (const char* n : allowed) list += (list.empty() ? "" : "、") + std::string{n};
                error(where + "/" + jsonPointerToken(it.key()), "未知字段“" + it.key() + "”（允许的字段：" + list + "）");
            }
        }
        return true;
    }

    std::optional<std::string> string(const Json& v, const std::string& where) {
        if (!v.is_string()) {
            error(where, "应为字符串，实际为" + kindOf(v));
            return std::nullopt;
        }
        return v.get<std::string>();
    }

    std::optional<double> number(const Json& v, const std::string& where) {
        if (!v.is_number()) {
            error(where, "应为数值，实际为" + kindOf(v));
            return std::nullopt;
        }
        const double d = v.get<double>();
        if (!std::isfinite(d)) {
            error(where, "数值超出范围");
            return std::nullopt;
        }
        return d;
    }
};

}  // namespace

std::string jsonPointerToken(const std::string& key) {
    std::string out;
    for (const char c : key) {
        if (c == '~') {
            out += "~0";
        } else if (c == '/') {
            out += "~1";
        } else {
            out += c;
        }
    }
    return out;
}

MotionParseResult parseMotion(const std::string& text, const std::string& motionPath) {
    std::error_code ec;
    fs::path path = fs::absolute(motionPath, ec);
    if (ec) path = motionPath;
    path = path.lexically_normal();
    Reader r;
    MotionParseResult result;

    Json root;
    try {
        root = Json::parse(text.begin(), text.end());
    } catch (const Json::exception& e) {
        r.error("", std::string{"JSON 语法错误："} + e.what());
        result.issues = std::move(r.issues);
        return result;
    }

    MotionDocument doc;
    doc.path = path.string();
    doc.directory = path.parent_path().string();
    if (r.object(root, "", {"format", "version", "name", "scene", "startKeyframe", "interpolation", "poses"})) {
        if (!root.contains("format")) {
            r.error("", "缺少 format（应为 “terra.motion”）");
        } else if (const auto s = r.string(root["format"], "/format"); s && *s != "terra.motion") {
            r.error("/format", "不是 Terra 动作文件（format 为 “" + *s + "”，应为 “terra.motion”）");
        }
        if (!root.contains("version")) {
            r.error("", "缺少 version");
        } else if (!root["version"].is_number_integer()) {
            r.error("/version", "应为整数，实际为" + kindOf(root["version"]));
        } else {
            const long long n = root["version"].get<long long>();
            if (n > kSupportedVersion) {
                r.error("/version", "动作文件版本 " + std::to_string(n) + " 高于本程序支持的版本 " +
                                        std::to_string(kSupportedVersion) + "，请更新程序");
            } else if (n < 1) {
                r.error("/version", "版本号无效：" + std::to_string(n));
            }
        }
        if (root.contains("name")) {
            if (const auto s = r.string(root["name"], "/name")) doc.name = *s;
        }
        if (root.contains("scene")) {
            if (const auto s = r.string(root["scene"], "/scene")) {
                if (s->empty()) {
                    r.error("/scene", "场景文件路径为空（没有推荐场景时省略此字段）");
                } else {
                    doc.scene = (path.parent_path() / *s).lexically_normal().string();
                    if (!fs::exists(doc.scene, ec)) r.warning("/scene", "推荐的场景文件不存在：" + doc.scene);
                }
            }
        }
        if (root.contains("startKeyframe")) {
            if (const auto s = r.string(root["startKeyframe"], "/startKeyframe")) {
                if (s->empty()) r.error("/startKeyframe", "关键帧名为空（从当前状态开始时省略此字段）");
                doc.startKeyframe = *s;
            }
        }
        if (root.contains("interpolation")) {
            if (const auto s = r.string(root["interpolation"], "/interpolation")) {
                if (*s == "step") {
                    doc.interpolation = MotionInterpolation::Step;
                } else if (*s == "linear") {
                    doc.interpolation = MotionInterpolation::Linear;
                } else if (*s == "smooth") {
                    doc.interpolation = MotionInterpolation::Smooth;
                } else {
                    r.error("/interpolation", "未知的插值方式“" + *s + "”（可用 step、linear、smooth）");
                }
            }
        }
        if (!root.contains("poses")) {
            r.error("", "缺少 poses");
        } else if (!root["poses"].is_array()) {
            r.error("/poses", "应为数组，实际为" + kindOf(root["poses"]));
        } else if (root["poses"].empty()) {
            r.error("/poses", "至少要有一个时间点");
        } else {
            const Json& poses = root["poses"];
            std::optional<double> previous;
            for (std::size_t i = 0; i < poses.size(); ++i) {
                const std::string where = "/poses/" + std::to_string(i);
                MotionPose pose;
                pose.where = where;
                if (!r.object(poses[i], where, {"time", "targets"})) continue;
                if (!poses[i].contains("time")) {
                    r.error(where, "缺少 time");
                } else if (const auto t = r.number(poses[i]["time"], where + "/time")) {
                    pose.time = *t;
                    if (*t < 0) {
                        r.error(where + "/time", "时间不能为负");
                    } else if (previous && *t <= *previous) {
                        r.error(where + "/time", "时间应严格递增（上一个点为 " + std::to_string(*previous) + " s）");
                    }
                    previous = *t;
                }
                if (!poses[i].contains("targets")) {
                    r.error(where, "缺少 targets");
                } else if (!poses[i]["targets"].is_object()) {
                    r.error(where + "/targets", "应为对象（执行器全名 → 目标值），实际为" + kindOf(poses[i]["targets"]));
                } else if (poses[i]["targets"].empty()) {
                    r.error(where + "/targets", "至少要有一个执行器");
                } else {
                    for (auto it = poses[i]["targets"].begin(); it != poses[i]["targets"].end(); ++it) {
                        const std::string at = where + "/targets/" + jsonPointerToken(it.key());
                        if (const auto v = r.number(it.value(), at)) pose.targets.emplace_back(it.key(), *v);
                    }
                }
                doc.poses.push_back(std::move(pose));
            }
        }
    }
    if (!r.hasErrors()) result.document = std::move(doc);
    result.issues = std::move(r.issues);
    return result;
}

std::vector<MotionCatalogEntry> findMotionsForScene(const std::vector<std::string>& roots, const std::string& scenePath) {
    std::vector<MotionCatalogEntry> out;
    std::error_code ec;
    const std::string wanted = fs::absolute(scenePath, ec).lexically_normal().string();
    const std::string suffix = ".terra-motion.json";
    for (const std::string& root : roots) {
        if (!fs::is_directory(root, ec)) continue;
        for (auto it = fs::recursive_directory_iterator(root, fs::directory_options::skip_permission_denied, ec);
             !ec && it != fs::recursive_directory_iterator(); it.increment(ec)) {
            const std::string name = it->path().filename().string();
            if (!it->is_regular_file(ec) || name.size() < suffix.size() ||
                name.compare(name.size() - suffix.size(), suffix.size(), suffix) != 0) {
                continue;
            }
            const auto parsed = readMotionFile(it->path().string());
            if (!parsed.document || parsed.document->scene != wanted) continue;
            out.push_back({parsed.document->name.empty() ? name : parsed.document->name, parsed.document->path});
        }
    }
    std::sort(out.begin(), out.end(), [](const MotionCatalogEntry& a, const MotionCatalogEntry& b) {
        return a.name != b.name ? a.name < b.name : a.path < b.path;
    });
    return out;
}

MotionParseResult readMotionFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        MotionParseResult result;
        result.issues.push_back({Severity::Error, {}, fs::exists(path) ? "无法读取动作文件：" + path : "动作文件不存在：" + path});
        return result;
    }
    const std::string text{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    return parseMotion(text, path);
}

}  // namespace terra::simulation
