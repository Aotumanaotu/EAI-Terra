// 通信配置文件的读取与校验（M4-06a）。nlohmann/json 只在本文件中使用，不进入公共头。

#include <cctype>
#include <cmath>
#include <fstream>
#include <iterator>
#include <set>

#include <nlohmann/json.hpp>

#include "terra/ros2/bridge.hpp"

namespace terra::ros2 {

namespace {

using Json = nlohmann::json;

std::string kindOf(const Json& v) {
    if (v.is_object()) return "对象";
    if (v.is_array()) return "数组";
    if (v.is_string()) return "字符串";
    if (v.is_boolean()) return "布尔值";
    if (v.is_number()) return "数值";
    return "null";
}

// 宇树配置（M4-06b）。字段名与含义见 bridge.hpp 的 UnitreeProfile
template <class ErrorFn>
UnitreeProfile readUnitree(const Json& u, ErrorFn&& error) {
    UnitreeProfile p;
    if (!u.is_object()) {
        error("/unitree", "应为对象，实际为" + kindOf(u));
        return p;
    }
    const std::set<std::string> allowed{"messages", "instance", "motors", "imu", "feet", "base", "lowstateHz", "lfLowstateHz",
                                        "sportModeStateHz", "lfSportModeStateHz", "wirelessControllerHz", "checkCrc", "modeMachine"};
    for (auto it = u.begin(); it != u.end(); ++it) {
        if (allowed.count(it.key()) == 0) error("/unitree/" + it.key(), "未知字段“" + it.key() + "”");
    }
    const auto at = [](const char* key) { return std::string{"/unitree/"} + key; };
    const auto readString = [&](const char* key, std::string& out, bool required) {
        if (!u.contains(key)) {
            if (required) error(at(key), "缺少字符串 " + std::string{key});
            return;
        }
        if (!u[key].is_string() || u[key].get<std::string>().empty()) error(at(key), "应为非空字符串");
        else out = u[key].get<std::string>();
    };
    const auto readNames = [&](const char* key, std::vector<std::string>& out, bool allowEmpty) {
        if (!u.contains(key)) return;
        if (!u[key].is_array()) {
            error(at(key), "应为字符串数组，实际为" + kindOf(u[key]));
            return;
        }
        for (std::size_t i = 0; i < u[key].size(); ++i) {
            const Json& v = u[key][i];
            if (!v.is_string() || (!allowEmpty && v.get<std::string>().empty())) {
                error(at(key) + "/" + std::to_string(i), allowEmpty ? "应为字符串（空字符串表示该编号没有电机）" : "应为非空字符串");
            } else {
                out.push_back(v.get<std::string>());
            }
        }
    };
    const auto readRate = [&](const char* key, double& out, bool allowZero) {
        if (!u.contains(key)) return;
        const Json& v = u[key];
        const bool ok = v.is_number() && std::isfinite(v.get<double>()) && (allowZero ? v.get<double>() >= 0 : v.get<double>() > 0);
        if (!ok) error(at(key), allowZero ? "频率应为非负数（Hz，0 表示不发布）" : "频率应为正数（Hz）");
        else out = v.get<double>();
    };
    readString("messages", p.messages, true);
    if (!p.messages.empty() && p.messages != "go" && p.messages != "hg") error(at("messages"), "应为 “go”（unitree_go）或 “hg”（unitree_hg）");
    readString("instance", p.instance, true);
    readString("imu", p.imu, true);
    readString("base", p.base, false);
    if (!u.contains("motors")) error(at("motors"), "缺少电机表 motors（电机编号 → 关节名）");
    readNames("motors", p.motors, true);
    readNames("feet", p.feet, false);
    const std::size_t slots = p.messages == "hg" ? 35u : 20u;
    if (p.motors.size() > slots) error(at("motors"), "电机数 " + std::to_string(p.motors.size()) + " 超过消息的 " + std::to_string(slots) + " 个电机位");
    std::size_t named = 0;
    for (const auto& m : p.motors) named += m.empty() ? 0u : 1u;
    if (u.contains("motors") && named == 0) error(at("motors"), "电机表中没有电机");
    if (!p.feet.empty() && p.feet.size() != 4) error(at("feet"), "应有 4 个足端实体（FR、FL、RR、RL）");
    readRate("lowstateHz", p.lowstateHz, false);
    readRate("lfLowstateHz", p.lfLowstateHz, true);
    readRate("sportModeStateHz", p.sportModeStateHz, true);
    readRate("lfSportModeStateHz", p.lfSportModeStateHz, true);
    readRate("wirelessControllerHz", p.wirelessControllerHz, true);
    if (u.contains("checkCrc")) {
        if (!u["checkCrc"].is_boolean()) error(at("checkCrc"), "应为 true 或 false");
        else p.checkCrc = u["checkCrc"].get<bool>();
    }
    if (u.contains("modeMachine")) {
        const Json& v = u["modeMachine"];
        if (!v.is_number_integer() || v.get<long long>() < 0 || v.get<long long>() > 255) error(at("modeMachine"), "应为 0～255 的整数");
        else p.modeMachine = static_cast<int>(v.get<long long>());
    }
    return p;
}

}  // namespace

std::string topicName(const std::string& name) {
    std::string out;
    out.reserve(name.size());
    bool segmentStart = true;
    for (const char c : name) {
        const auto u = static_cast<unsigned char>(c);
        if (c == '/') {
            out += c;
            segmentStart = true;
            continue;
        }
        const bool allowed = std::isalnum(u) != 0 || c == '_';
        if (segmentStart && std::isdigit(u) != 0) out += '_';
        out += allowed && u < 128 ? c : '_';
        segmentStart = false;
    }
    return out;
}

ProfileResult parseProfile(const std::string& text, const std::string& path) {
    ProfileResult result;
    const auto error = [&](const std::string& where, const std::string& message) { result.errors.push_back(where + "：" + message); };
    Json root;
    try {
        root = Json::parse(text.begin(), text.end());
    } catch (const Json::exception& e) {
        error("", std::string{"JSON 语法错误："} + e.what());
        return result;
    }
    if (!root.is_object()) {
        error("", "应为对象，实际为" + kindOf(root));
        return result;
    }
    const std::set<std::string> allowed{"format", "version", "name", "kind", "topicPrefix", "clock", "jointStatesHz", "tfHz",
                                        "sensors", "commands", "services", "unitree"};
    for (auto it = root.begin(); it != root.end(); ++it) {
        if (allowed.count(it.key()) == 0) error("/" + it.key(), "未知字段“" + it.key() + "”");
    }
    if (!root.contains("format") || !root["format"].is_string() || root["format"].get<std::string>() != "terra.ros2") {
        error("/format", "应为 “terra.ros2”");
    }
    if (!root.contains("version") || !root["version"].is_number_integer()) {
        error("/version", "缺少整数 version");
    } else if (root["version"].get<long long>() != 1) {
        error("/version", "版本 " + std::to_string(root["version"].get<long long>()) + " 不受支持（本程序支持 1）");
    }
    Profile p;
    p.path = path;
    const auto readString = [&](const char* key, std::string& out) {
        if (!root.contains(key)) return;
        if (!root[key].is_string()) error(std::string{"/"} + key, "应为字符串，实际为" + kindOf(root[key]));
        else out = root[key].get<std::string>();
    };
    const auto readBool = [&](const char* key, bool& out) {
        if (!root.contains(key)) return;
        if (!root[key].is_boolean()) error(std::string{"/"} + key, "应为 true 或 false，实际为" + kindOf(root[key]));
        else out = root[key].get<bool>();
    };
    const auto readRate = [&](const char* key, double& out) {
        if (!root.contains(key)) return;
        if (!root[key].is_number() || !(root[key].get<double>() > 0) || !std::isfinite(root[key].get<double>())) {
            error(std::string{"/"} + key, "频率应为正数（Hz）");
        } else {
            out = root[key].get<double>();
        }
    };
    readString("name", p.name);
    readString("kind", p.kind);
    if (p.kind != "generic" && p.kind != "unitree") error("/kind", "未知的配置类型“" + p.kind + "”（可用：generic、unitree）");
    readString("topicPrefix", p.topicPrefix);
    if (!p.topicPrefix.empty() && (p.topicPrefix.front() != '/' || p.topicPrefix.back() == '/' || topicName(p.topicPrefix) != p.topicPrefix)) {
        error("/topicPrefix", "应以 “/” 开头、不以 “/” 结尾，只含字母、数字、下划线（如 “/sim”）");
    }
    readBool("clock", p.clock);
    readRate("jointStatesHz", p.jointStatesHz);
    readRate("tfHz", p.tfHz);
    readBool("sensors", p.sensors);
    readBool("commands", p.commands);
    readBool("services", p.services);
    if (p.kind == "unitree" && !root.contains("unitree")) error("/unitree", "宇树配置（kind 为 unitree）需要 unitree 字段");
    if (p.kind != "unitree" && root.contains("unitree")) error("/unitree", "只有 kind 为 unitree 时才能有 unitree 字段");
    if (p.kind == "unitree" && root.contains("unitree")) p.unitree = readUnitree(root["unitree"], error);
    if (result.errors.empty()) result.profile = p;
    return result;
}

ProfileResult readProfile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        ProfileResult r;
        r.errors.push_back("：无法读取通信配置文件 " + path);
        return r;
    }
    const std::string text{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    return parseProfile(text, path);
}

std::vector<terra::physics::ActuatorCommand> mergeCommands(std::vector<terra::physics::ActuatorCommand> base,
                                                           const std::vector<terra::physics::ActuatorCommand>& later) {
    for (const auto& c : later) {
        bool replaced = false;
        for (auto& b : base) {
            if (b.actuator == c.actuator) {
                b.values = c.values;
                replaced = true;
            }
        }
        if (!replaced) base.push_back(c);
    }
    return base;
}

}  // namespace terra::ros2
