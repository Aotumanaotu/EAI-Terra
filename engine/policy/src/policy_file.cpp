// 策略说明文件 *.terra-policy.json 的读写（M4-10a）。

#include "terra/policy/policy.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <system_error>
#include <utility>

#include <nlohmann/json.hpp>

namespace terra::policy {

namespace {

namespace fs = std::filesystem;
using Json = nlohmann::ordered_json;

struct TermEntry {
    TermKind kind;
    const char* name;
};
constexpr TermEntry kTerms[] = {
    {TermKind::BaseAngularVelocity, "baseAngularVelocity"}, {TermKind::ProjectedGravity, "projectedGravity"},
    {TermKind::BaseLinearVelocity, "baseLinearVelocity"},   {TermKind::JointPositions, "jointPositions"},
    {TermKind::JointVelocities, "jointVelocities"},         {TermKind::LastAction, "lastAction"},
    {TermKind::Command, "command"},                         {TermKind::Phase, "phase"},
};

class Reader {
public:
    std::vector<std::string> errors;
    std::vector<std::string> warnings;

    void error(const std::string& where, const std::string& message) { errors.push_back((where.empty() ? "/" : where) + "：" + message); }
    void warn(const std::string& where, const std::string& message) { warnings.push_back((where.empty() ? "/" : where) + "：" + message); }

    std::string string(const Json& o, const char* key, const std::string& where, bool required) {
        if (!o.contains(key)) {
            if (required) error(where + "/" + key, "缺少");
            return {};
        }
        if (!o[key].is_string()) {
            error(where + "/" + key, "应为字符串");
            return {};
        }
        return o[key].get<std::string>();
    }
    double number(const Json& o, const char* key, const std::string& where, double fallback) {
        if (!o.contains(key)) return fallback;
        if (!o[key].is_number() || !std::isfinite(o[key].get<double>())) {
            error(where + "/" + key, "应为数值");
            return fallback;
        }
        return o[key].get<double>();
    }
    // 一个数或数组
    std::vector<double> numbers(const Json& o, const char* key, const std::string& where) {
        if (!o.contains(key)) return {};
        const Json& j = o[key];
        if (j.is_number()) return {j.get<double>()};
        std::vector<double> out;
        if (!j.is_array()) {
            error(where + "/" + key, "应为数值或数值数组");
            return out;
        }
        for (const auto& v : j) {
            if (!v.is_number() || !std::isfinite(v.get<double>())) {
                error(where + "/" + key, "数组中应全为数值");
                return {};
            }
            out.push_back(v.get<double>());
        }
        return out;
    }
    std::vector<std::string> strings(const Json& o, const char* key, const std::string& where) {
        std::vector<std::string> out;
        if (!o.contains(key)) return out;
        if (!o[key].is_array()) {
            error(where + "/" + key, "应为字符串数组");
            return out;
        }
        for (const auto& v : o[key]) {
            if (!v.is_string()) {
                error(where + "/" + key, "数组中应全为字符串");
                return {};
            }
            out.push_back(v.get<std::string>());
        }
        return out;
    }
    void unknown(const Json& o, const std::string& where, const std::set<std::string>& known) {
        for (auto it = o.begin(); it != o.end(); ++it) {
            if (!known.count(it.key())) warn(where + "/" + it.key(), "不认识的字段，已忽略");
        }
    }
};

std::string resolve(const std::string& p, const fs::path& base) {
    if (p.empty()) return {};
    const fs::path path{p};
    return (path.is_absolute() ? path : base / path).lexically_normal().string();
}

std::string relativeTo(const std::string& target, const fs::path& baseDir) {
    if (target.empty()) return {};
    const fs::path rel = fs::path{target}.lexically_relative(baseDir);
    return rel.empty() ? target : rel.generic_string();
}

// 长度为 1 或 n 的数组
bool broadcastable(const std::vector<double>& v, std::size_t n) { return v.empty() || v.size() == 1 || v.size() == n; }

Json numbersJson(const std::vector<double>& v) {
    if (v.size() == 1) return v[0];
    Json a = Json::array();
    for (double x : v) a.push_back(x);
    return a;
}

}  // namespace

const char* termName(TermKind kind) {
    for (const auto& t : kTerms) {
        if (t.kind == kind) return t.name;
    }
    return "baseAngularVelocity";
}

std::optional<TermKind> termFromName(const std::string& name) {
    for (const auto& t : kTerms) {
        if (name == t.name) return t.kind;
    }
    return std::nullopt;
}

const char* actionTypeName(ActionType type) {
    switch (type) {
        case ActionType::PositionPd: return "positionPd";
        case ActionType::Position: return "position";
        case ActionType::Velocity: return "velocity";
        case ActionType::Torque: return "torque";
    }
    return "positionPd";
}

int PolicyDescription::termWidth(const ObservationTerm& term) const {
    switch (term.kind) {
        case TermKind::BaseAngularVelocity:
        case TermKind::ProjectedGravity:
        case TermKind::BaseLinearVelocity:
        case TermKind::Command: return 3;
        case TermKind::JointPositions:
        case TermKind::JointVelocities: return static_cast<int>(term.joints.size());
        case TermKind::LastAction: return actionWidth();
        case TermKind::Phase: return 2;
    }
    return 0;
}

int PolicyDescription::frameWidth() const {
    int w = 0;
    for (const auto& t : observation) w += termWidth(t);
    return w;
}

PolicyFileResult parsePolicy(const std::string& text, const std::string& path) {
    PolicyFileResult result;
    Reader r;
    const Json j = Json::parse(text, nullptr, false);
    if (j.is_discarded() || !j.is_object()) {
        result.errors.push_back("不是有效的 JSON 对象");
        return result;
    }
    if (!j.contains("format") || !j["format"].is_string() || j["format"].get<std::string>() != "terra.policy") {
        result.errors.push_back("/format：应为 \"terra.policy\"");
        return result;
    }
    if (!j.contains("version") || !j["version"].is_number_integer() || j["version"].get<int>() != 1) {
        result.errors.push_back("/version：只支持 1");
        return result;
    }
    std::error_code ec;
    const fs::path file = fs::absolute(path, ec).lexically_normal();
    const fs::path base = file.parent_path();
    PolicyDescription p;
    p.path = file.string();
    p.name = r.string(j, "name", "", false);
    p.scene = resolve(r.string(j, "scene", "", false), base);

    // 模型
    if (!j.contains("model") || !j["model"].is_object()) {
        r.error("/model", "缺少（模型文件与推理后端）");
    } else {
        const Json& m = j["model"];
        p.model = resolve(r.string(m, "file", "/model", true), base);
        p.backend = r.string(m, "backend", "/model", true);
        p.inputName = r.string(m, "input", "/model", false);
        p.outputName = r.string(m, "output", "/model", false);
        p.modelSha256 = r.string(m, "sha256", "/model", false);
        r.unknown(m, "/model", {"file", "backend", "input", "output", "sha256"});
    }
    p.controlPeriod = r.number(j, "controlPeriod", "", 0.02);
    if (!(p.controlPeriod > 0)) r.error("/controlPeriod", "应大于 0");

    // 历史帧
    if (j.contains("history")) {
        const Json& h = j["history"];
        if (!h.is_object()) {
            r.error("/history", "应为对象");
        } else {
            const double length = r.number(h, "length", "/history", 1);
            if (length < 1 || length > 1000 || std::floor(length) != length) r.error("/history/length", "应为 1～1000 的整数");
            else p.historyLength = static_cast<int>(length);
            const std::string order = r.string(h, "order", "/history", false);
            if (!order.empty() && order != "oldestFirst") r.error("/history/order", "目前只支持 \"oldestFirst\"");
            const std::string reset = r.string(h, "reset", "/history", false);
            if (!reset.empty() && reset != "repeatInitial") r.error("/history/reset", "目前只支持 \"repeatInitial\"");
            r.unknown(h, "/history", {"length", "order", "reset"});
        }
    }

    // 速度指令
    if (j.contains("command")) {
        const Json& c = j["command"];
        if (!c.is_object()) {
            r.error("/command", "应为对象");
        } else {
            const auto d = r.numbers(c, "default", "/command");
            if (!d.empty() && d.size() != 3) r.error("/command/default", "应为 3 个数值（vx、vy、偏航角速度）");
            else if (d.size() == 3) p.defaultCommand = {d[0], d[1], d[2]};
            if (c.contains("range")) {
                const Json& range = c["range"];
                bool ok = range.is_array() && range.size() == 3;
                for (std::size_t i = 0; ok && i < 3; ++i) {
                    ok = range[i].is_array() && range[i].size() == 2 && range[i][0].is_number() && range[i][1].is_number() &&
                         range[i][0].get<double>() <= range[i][1].get<double>();
                    if (ok) p.commandRange[i] = {range[i][0].get<double>(), range[i][1].get<double>()};
                }
                if (!ok) r.error("/command/range", "应为 3 个 [最小, 最大]");
            }
            r.unknown(c, "/command", {"default", "range"});
        }
    }

    // 动作（先读，观测中的 lastAction 宽度取决于它）
    if (!j.contains("action") || !j["action"].is_object()) {
        r.error("/action", "缺少");
    } else {
        const Json& a = j["action"];
        const std::string type = r.string(a, "type", "/action", true);
        if (type == "positionPd") p.action.type = ActionType::PositionPd;
        else if (type == "position") p.action.type = ActionType::Position;
        else if (type == "velocity") p.action.type = ActionType::Velocity;
        else if (type == "torque") p.action.type = ActionType::Torque;
        else if (!type.empty()) r.error("/action/type", "“" + type + "”不是可用的值（可用：positionPd、position、velocity、torque）");
        p.action.joints = r.strings(a, "joints", "/action");
        if (p.action.joints.empty()) r.error("/action/joints", "缺少或为空");
        p.action.scale = r.numbers(a, "scale", "/action");
        p.action.offset = r.numbers(a, "offset", "/action");
        p.action.kp = r.numbers(a, "kp", "/action");
        p.action.kd = r.numbers(a, "kd", "/action");
        p.action.torqueLimit = r.numbers(a, "torqueLimit", "/action");
        p.action.clip = r.number(a, "clip", "/action", 0.0);
        const std::size_t n = p.action.joints.size();
        const std::pair<const char*, const std::vector<double>*> lists[] = {
            {"scale", &p.action.scale}, {"offset", &p.action.offset}, {"kp", &p.action.kp}, {"kd", &p.action.kd}, {"torqueLimit", &p.action.torqueLimit}};
        for (const auto& [key, v] : lists) {
            if (!broadcastable(*v, n)) r.error(std::string{"/action/"} + key, "长度应为 1 或关节数（" + std::to_string(n) + "）");
        }
        if (p.action.type == ActionType::PositionPd && (p.action.kp.empty() || p.action.kd.empty())) {
            r.error("/action", "positionPd 需要 kp 与 kd");
        }
        r.unknown(a, "/action", {"type", "joints", "scale", "offset", "kp", "kd", "torqueLimit", "clip"});
    }

    // 观测
    if (!j.contains("observation") || !j["observation"].is_array() || j["observation"].empty()) {
        r.error("/observation", "缺少或为空");
    } else {
        for (std::size_t i = 0; i < j["observation"].size(); ++i) {
            const Json& o = j["observation"][i];
            const std::string where = "/observation/" + std::to_string(i);
            if (!o.is_object()) {
                r.error(where, "应为对象");
                continue;
            }
            ObservationTerm t;
            const std::string name = r.string(o, "term", where, true);
            const auto kind = termFromName(name);
            if (!kind) {
                if (!name.empty()) {
                    std::string list;
                    for (const auto& e : kTerms) list += (list.empty() ? "" : "、") + std::string{e.name};
                    r.error(where + "/term", "“" + name + "”不是可用的观测项（可用：" + list + "）");
                }
                continue;
            }
            t.kind = *kind;
            t.sensor = r.string(o, "sensor", where, false);
            t.entity = r.string(o, "entity", where, false);
            t.joints = r.strings(o, "joints", where);
            t.scale = r.numbers(o, "scale", where);
            t.offset = r.numbers(o, "offset", where);
            t.clip = r.number(o, "clip", where, 0.0);
            t.period = r.number(o, "period", where, 0.0);
            t.standingThreshold = r.number(o, "standingThreshold", where, 0.0);
            switch (t.kind) {
                case TermKind::BaseAngularVelocity:
                case TermKind::ProjectedGravity:
                    if (t.sensor.empty() == t.entity.empty()) r.error(where, "需要 sensor（IMU，带噪声）或 entity（真值）之一");
                    break;
                case TermKind::BaseLinearVelocity:
                    if (t.entity.empty()) r.error(where, "需要 entity");
                    break;
                case TermKind::JointPositions:
                case TermKind::JointVelocities:
                    if (t.joints.empty()) t.joints = p.action.joints;  // 省略时与动作相同
                    if (!broadcastable(t.offset, t.joints.size())) r.error(where + "/offset", "长度应为 1 或关节数");
                    break;
                case TermKind::Phase:
                    if (!(t.period > 0)) r.error(where + "/period", "应大于 0（秒）");
                    break;
                case TermKind::LastAction:
                case TermKind::Command: break;
            }
            if (!broadcastable(t.scale, static_cast<std::size_t>(std::max(p.termWidth(t), 1)))) r.error(where + "/scale", "长度应为 1 或该项的宽度");
            r.unknown(o, where, {"term", "sensor", "entity", "joints", "scale", "offset", "clip", "period", "standingThreshold"});
            p.observation.push_back(std::move(t));
        }
    }

    if (j.contains("source") && j["source"].is_object()) {
        for (auto it = j["source"].begin(); it != j["source"].end(); ++it) {
            p.source[it.key()] = it.value().is_string() ? it.value().get<std::string>() : it.value().dump();
        }
    }
    // 不认识的顶层字段：保留
    const std::set<std::string> known = {"format", "version", "name", "scene", "model", "controlPeriod", "history", "command", "observation", "action", "source"};
    Json extra = Json::object();
    for (auto it = j.begin(); it != j.end(); ++it) {
        if (known.count(it.key())) continue;
        r.warn("/" + it.key(), "不认识的字段（可能来自更新的版本），已原样保留");
        extra[it.key()] = it.value();
    }
    if (!extra.empty()) p.extra = extra.dump();

    result.errors = std::move(r.errors);
    result.warnings = std::move(r.warnings);
    if (result.errors.empty()) result.policy = std::move(p);
    return result;
}

PolicyFileResult readPolicyFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        PolicyFileResult r;
        r.errors.push_back("无法打开：" + path);
        return r;
    }
    const std::string text{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    return parsePolicy(text, path);
}

std::string policyToJson(const PolicyDescription& p, const std::string& path) {
    std::error_code ec;
    const fs::path base = fs::absolute(path, ec).lexically_normal().parent_path();
    Json j;
    j["format"] = "terra.policy";
    j["version"] = 1;
    if (!p.name.empty()) j["name"] = p.name;
    if (!p.scene.empty()) j["scene"] = relativeTo(p.scene, base);
    Json m;
    m["file"] = relativeTo(p.model, base);
    m["backend"] = p.backend;
    if (!p.inputName.empty()) m["input"] = p.inputName;
    if (!p.outputName.empty()) m["output"] = p.outputName;
    if (!p.modelSha256.empty()) m["sha256"] = p.modelSha256;
    j["model"] = std::move(m);
    j["controlPeriod"] = p.controlPeriod;
    j["history"] = {{"length", p.historyLength}, {"order", "oldestFirst"}, {"reset", "repeatInitial"}};
    Json range = Json::array();
    for (const auto& r : p.commandRange) range.push_back({r[0], r[1]});
    j["command"] = {{"default", {p.defaultCommand[0], p.defaultCommand[1], p.defaultCommand[2]}}, {"range", std::move(range)}};
    Json obs = Json::array();
    for (const auto& t : p.observation) {
        Json o;
        o["term"] = termName(t.kind);
        if (!t.sensor.empty()) o["sensor"] = t.sensor;
        if (!t.entity.empty()) o["entity"] = t.entity;
        if ((t.kind == TermKind::JointPositions || t.kind == TermKind::JointVelocities) && t.joints != p.action.joints) o["joints"] = t.joints;
        if (!t.scale.empty()) o["scale"] = numbersJson(t.scale);
        if (!t.offset.empty()) o["offset"] = numbersJson(t.offset);
        if (t.clip > 0) o["clip"] = t.clip;
        if (t.kind == TermKind::Phase) {
            o["period"] = t.period;
            o["standingThreshold"] = t.standingThreshold;
        }
        obs.push_back(std::move(o));
    }
    j["observation"] = std::move(obs);
    Json a;
    a["type"] = actionTypeName(p.action.type);
    a["joints"] = p.action.joints;
    if (!p.action.scale.empty()) a["scale"] = numbersJson(p.action.scale);
    if (!p.action.offset.empty()) a["offset"] = numbersJson(p.action.offset);
    if (!p.action.kp.empty()) a["kp"] = numbersJson(p.action.kp);
    if (!p.action.kd.empty()) a["kd"] = numbersJson(p.action.kd);
    if (!p.action.torqueLimit.empty()) a["torqueLimit"] = numbersJson(p.action.torqueLimit);
    if (p.action.clip > 0) a["clip"] = p.action.clip;
    j["action"] = std::move(a);
    if (!p.source.empty()) {
        Json s = Json::object();
        for (const auto& [k, v] : p.source) s[k] = v;
        j["source"] = std::move(s);
    }
    if (!p.extra.empty()) {
        const Json e = Json::parse(p.extra, nullptr, false);
        if (e.is_object()) {
            for (auto it = e.begin(); it != e.end(); ++it) j[it.key()] = it.value();
        }
    }
    return j.dump(2) + "\n";
}

std::optional<std::string> savePolicy(const PolicyDescription& policy, const std::string& path) {
    std::error_code ec;
    const fs::path target = fs::absolute(path, ec).lexically_normal();
    if (!target.parent_path().empty()) fs::create_directories(target.parent_path(), ec);
    const std::string temp = target.string() + ".partial";
    {
        std::ofstream out(temp, std::ios::binary);
        out << policyToJson(policy, target.string());
        if (!out) return "无法写入：" + temp;
    }
    fs::rename(temp, target, ec);
    if (ec) {
        fs::remove(temp, ec);
        return "无法写入：" + target.string();
    }
    return std::nullopt;
}

}  // namespace terra::policy
