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
                                        "sensors", "commands", "services"};
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
    if (p.kind == "unitree") error("/kind", "宇树配置在 M4-06b 中加入，当前版本只支持 “generic”");
    else if (p.kind != "generic") error("/kind", "未知的配置类型“" + p.kind + "”（可用：generic）");
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
