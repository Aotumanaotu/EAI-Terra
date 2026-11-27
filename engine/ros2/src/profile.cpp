// 通信配置文件的读取与校验（M4-06a）。nlohmann/json 只在本文件中使用，不进入公共头。

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <initializer_list>
#include <fstream>
#include <iterator>
#include <set>

#include <nlohmann/json.hpp>

#include "terra/ros2/bridge.hpp"
#include "terra/ros2/integration.hpp"

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

// ---- M5-13a：接入档案的字段 ----

bool validTopic(const std::string& t) { return !t.empty() && t[0] == '/' && topicName(t) == t; }

// 话题名或含 “*” 的话题名模式（“*” 只能是一整段或一段的一部分，其余字符同话题名）
bool validTopicPattern(const std::string& t) {
    std::string probe = t;
    std::replace(probe.begin(), probe.end(), '*', 'x');
    return validTopic(probe);
}

template <class ErrorFn>
void readTopicSettings(const Json& v, std::map<std::string, TopicSettings>& out, ErrorFn&& error) {
    if (!v.is_object()) {
        error("/topicSettings", "应为对象（话题名 → 设置）");
        return;
    }
    for (auto it = v.begin(); it != v.end(); ++it) {
        const std::string at = "/topicSettings/" + it.key();
        if (!validTopicPattern(it.key())) {
            error(at, "话题名应以 “/” 开头，只含字母、数字、下划线、“/” 与 “*”");
            continue;
        }
        const Json& o = it.value();
        if (!o.is_object()) {
            error(at, "应为对象");
            continue;
        }
        TopicSettings t;
        const std::set<std::string> keys{"reliability", "durability", "depth", "rateHz", "accelUnit", "pointLayout"};
        for (auto f = o.begin(); f != o.end(); ++f) {
            if (keys.count(f.key()) == 0) error(at + "/" + f.key(), "未知字段“" + f.key() + "”");
        }
        const auto choice = [&](const char* key, std::string& field, std::initializer_list<const char*> allowed) {
            if (!o.contains(key)) return;
            std::string list;
            for (const char* a : allowed) list += (list.empty() ? "" : "、") + std::string{a};
            const Json& x = o[key];
            const bool ok = x.is_string() && std::any_of(allowed.begin(), allowed.end(), [&](const char* a) { return x.get<std::string>() == a; });
            if (!ok) error(at + "/" + key, "应为 " + list + " 之一");
            else field = x.get<std::string>();
        };
        choice("reliability", t.reliability, {"reliable", "best_effort"});
        choice("durability", t.durability, {"volatile", "transient_local"});
        choice("accelUnit", t.accelUnit, {"mps2", "g"});
        choice("pointLayout", t.pointLayout, {"livox", "xyzi", "velodyne", "ouster"});
        if (o.contains("depth")) {
            const Json& x = o["depth"];
            if (!x.is_number_integer() || x.get<long long>() < 1 || x.get<long long>() > 10000) error(at + "/depth", "应为 1～10000 的整数");
            else t.depth = static_cast<int>(x.get<long long>());
        }
        if (o.contains("rateHz")) {
            const Json& x = o["rateHz"];
            if (!x.is_number() || !std::isfinite(x.get<double>()) || !(x.get<double>() > 0)) error(at + "/rateHz", "频率应为正数（Hz）");
            else t.rateHz = x.get<double>();
        }
        out[it.key()] = t;
    }
}

template <class ErrorFn>
void readTf(const Json& v, TfSettings& out, ErrorFn&& error) {
    if (!v.is_object()) {
        error("/tf", "应为对象");
        return;
    }
    for (auto it = v.begin(); it != v.end(); ++it) {
        const std::string at = "/tf/" + it.key();
        if (it.key() == "robot") {
            const Json& x = it.value();
            const bool ok = x.is_string() && (x == "auto" || x == "world" || x == "tree" || x == "none");
            if (!ok) error(at, "应为 auto、world、tree 或 none");
            else out.robot = x.get<std::string>();
        } else if (it.key() == "sensorMounts" || it.key() == "odom") {
            if (!it.value().is_boolean()) error(at, "应为 true 或 false");
            else (it.key() == "odom" ? out.odom : out.sensorMounts) = it.value().get<bool>();
        } else {
            error(at, "未知字段“" + it.key() + "”");
        }
    }
}

template <class ErrorFn>
void readCommandInputs(const Json& v, std::vector<CommandInput>& out, ErrorFn&& error) {
    if (!v.is_array() || v.empty()) {
        error("/cmdVel", "应为非空数组（每项一路速度指令输入）");
        return;
    }
    std::set<std::string> seen;
    for (std::size_t i = 0; i < v.size(); ++i) {
        const std::string at = "/cmdVel/" + std::to_string(i);
        const Json& o = v[i];
        if (!o.is_object()) {
            error(at, "应为对象");
            continue;
        }
        const std::set<std::string> keys{"topic", "type", "priority", "timeout"};
        for (auto f = o.begin(); f != o.end(); ++f) {
            if (keys.count(f.key()) == 0) error(at + "/" + f.key(), "未知字段“" + f.key() + "”");
        }
        CommandInput c;
        if (!o.contains("topic") || !o["topic"].is_string() || !validTopic(o["topic"].get<std::string>())) {
            error(at + "/topic", "应为话题名（以 “/” 开头，只含字母、数字、下划线与 “/”）");
        } else {
            c.topic = o["topic"].get<std::string>();
            if (!seen.insert(c.topic).second) error(at + "/topic", "话题“" + c.topic + "”重复");
        }
        if (o.contains("type")) {
            if (!o["type"].is_string() || (o["type"] != "Twist" && o["type"] != "TwistStamped")) error(at + "/type", "应为 Twist 或 TwistStamped");
            else c.type = o["type"].get<std::string>();
        }
        if (o.contains("priority")) {
            if (!o["priority"].is_number_integer() || std::llabs(o["priority"].get<long long>()) > 1000) error(at + "/priority", "应为 −1000～1000 的整数");
            else c.priority = static_cast<int>(o["priority"].get<long long>());
        }
        if (o.contains("timeout")) {
            const Json& x = o["timeout"];
            if (!x.is_number() || !std::isfinite(x.get<double>()) || !(x.get<double>() > 0)) error(at + "/timeout", "应为正数（秒）");
            else c.timeout = x.get<double>();
        }
        out.push_back(c);
    }
}

template <class ErrorFn>
void readPlanarBase(const Json& v, PlanarBaseSettings& out, ErrorFn&& error) {
    if (v.is_string()) {
        if (v != "auto") error("/planarBase", "应为 “auto”（场景中有平面底盘的关节时使用）或对象");
        out.optional = true;
        return;
    }
    if (!v.is_object()) {
        error("/planarBase", "应为 “auto” 或对象");
        return;
    }
    const std::set<std::string> keys{"optional", "xJoint", "yJoint", "yawJoint", "baseEntity", "limits", "odom", "odomFrame", "odomHz"};
    for (auto it = v.begin(); it != v.end(); ++it) {
        if (keys.count(it.key()) == 0) error("/planarBase/" + it.key(), "未知字段“" + it.key() + "”");
    }
    const auto text = [&](const char* key, std::string& field, bool allowEmpty) {
        if (!v.contains(key)) return;
        if (!v[key].is_string() || (!allowEmpty && v[key].get<std::string>().empty())) error(std::string{"/planarBase/"} + key, "应为非空字符串");
        else field = v[key].get<std::string>();
    };
    text("xJoint", out.xJoint, false);
    text("yJoint", out.yJoint, false);
    text("yawJoint", out.yawJoint, false);
    text("baseEntity", out.baseEntity, false);
    text("odom", out.odom, true);
    text("odomFrame", out.odomFrame, false);
    if (!out.odom.empty() && !validTopic(out.odom)) error("/planarBase/odom", "应为话题名（以 “/” 开头）或空字符串（不发里程计）");
    if (v.contains("optional")) {
        if (!v["optional"].is_boolean()) error("/planarBase/optional", "应为 true 或 false");
        else out.optional = v["optional"].get<bool>();
    }
    if (v.contains("limits")) {
        const Json& l = v["limits"];
        bool ok = l.is_array() && l.size() == 3;
        for (std::size_t i = 0; ok && i < 3; ++i) ok = l[i].is_number() && std::isfinite(l[i].get<double>()) && l[i].get<double>() >= 0;
        if (!ok) error("/planarBase/limits", "应为 3 个非负数 [vx, vy, wz]（m/s、m/s、rad/s；0 表示不限）");
        else out.limits = {l[0].get<double>(), l[1].get<double>(), l[2].get<double>()};
    }
    if (v.contains("odomHz")) {
        const Json& x = v["odomHz"];
        if (!x.is_number() || !std::isfinite(x.get<double>()) || !(x.get<double>() > 0)) error("/planarBase/odomHz", "频率应为正数（Hz）");
        else out.odomHz = x.get<double>();
    }
}

template <class ErrorFn>
void readGroundTruth(const Json& v, GroundTruthSettings& out, ErrorFn&& error) {
    if (!v.is_object()) {
        error("/groundTruth", "应为对象（至少有 entity）");
        return;
    }
    const std::set<std::string> keys{"entity", "topic", "frame", "childFrame", "rateHz"};
    for (auto it = v.begin(); it != v.end(); ++it) {
        if (keys.count(it.key()) == 0) error("/groundTruth/" + it.key(), "未知字段“" + it.key() + "”");
    }
    const auto text = [&](const char* key, std::string& field, bool required) {
        if (!v.contains(key)) {
            if (required) error(std::string{"/groundTruth/"} + key, "缺少字段");
            return;
        }
        if (!v[key].is_string() || v[key].get<std::string>().empty()) error(std::string{"/groundTruth/"} + key, "应为非空字符串");
        else field = v[key].get<std::string>();
    };
    text("entity", out.entity, true);
    text("topic", out.topic, false);
    text("frame", out.frame, false);
    text("childFrame", out.childFrame, false);
    if (!validTopic(out.topic)) error("/groundTruth/topic", "应为话题名（以 “/” 开头）");
    if (v.contains("rateHz")) {
        const Json& x = v["rateHz"];
        if (!x.is_number() || !std::isfinite(x.get<double>()) || !(x.get<double>() > 0)) error("/groundTruth/rateHz", "频率应为正数（Hz）");
        else out.rateHz = x.get<double>();
    }
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
                                        "sensors", "commands", "services", "unitree", "topics", "frames", "depthEncoding", "diffDrive",
                                        // M5-13a
                                        "time", "topicSettings", "tf", "groundTruth", "cmdVel", "planarBase"};
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
    // M4-08b：话题名映射、坐标系、深度格式、差速底盘
    if (root.contains("topics")) {
        const Json& t = root["topics"];
        if (!t.is_object()) error("/topics", "应为对象（默认话题名 → 新的话题名）");
        else {
            for (auto it = t.begin(); it != t.end(); ++it) {
                const bool valid = it.value().is_string() && validTopic(it.value().get<std::string>());
                // M5-13a：默认话题名可含 “*”（如 “/*/mid360/points”）
                if (!validTopicPattern(it.key())) error("/topics/" + it.key(), "默认话题名应以 “/” 开头，只含字母、数字、下划线、“/” 与 “*”");
                else if (!valid) error("/topics/" + it.key(), "新的话题名应以 “/” 开头，只含字母、数字、下划线与 “/”");
                else p.topics[it.key()] = it.value().get<std::string>();
            }
        }
    }
    if (root.contains("frames")) {
        const Json& f = root["frames"];
        if (!f.is_object()) error("/frames", "应为对象");
        else {
            for (auto it = f.begin(); it != f.end(); ++it) {
                if (it.key() == "stripPrefix") {
                    if (!it.value().is_string()) error("/frames/stripPrefix", "应为字符串");
                    else p.stripFramePrefix = it.value().get<std::string>();
                } else if (it.key() == "sensorsUseLinkFrames") {
                    if (!it.value().is_boolean()) error("/frames/sensorsUseLinkFrames", "应为 true 或 false");
                    else p.sensorsUseLinkFrames = it.value().get<bool>();
                } else if (it.key() == "rename") {  // M5-13a
                    const Json& r = it.value();
                    if (!r.is_object()) {
                        error("/frames/rename", "应为对象（Terra 的坐标系全名 → 新名字）");
                    } else {
                        for (auto q = r.begin(); q != r.end(); ++q) {
                            if (q.key().empty()) error("/frames/rename/", "坐标系名为空");
                            else if (!q.value().is_string() || q.value().get<std::string>().empty()) error("/frames/rename/" + q.key(), "新名字应为非空字符串");
                            else p.frameRename[q.key()] = q.value().get<std::string>();
                        }
                    }
                } else {
                    error("/frames/" + it.key(), "未知字段“" + it.key() + "”");
                }
            }
        }
    }
    readString("depthEncoding", p.depthEncoding);
    if (p.depthEncoding != "32FC1" && p.depthEncoding != "16UC1") error("/depthEncoding", "应为 32FC1（米）或 16UC1（毫米）");
    if (root.contains("diffDrive")) {
        const Json& d = root["diffDrive"];
        if (!d.is_object()) {
            error("/diffDrive", "应为对象");
        } else {
            DiffDriveProfile dd;
            const std::set<std::string> keys{"leftJoint", "rightJoint", "leftActuator", "rightActuator", "baseEntity", "wheelRadius", "wheelSeparation",
                                             "maxWheelSpeed", "cmdVel", "odom", "odomFrame", "odomHz", "timeout"};
            for (auto it = d.begin(); it != d.end(); ++it) {
                if (keys.count(it.key()) == 0) error("/diffDrive/" + it.key(), "未知字段“" + it.key() + "”");
            }
            const auto textField = [&](const char* key, std::string& out, bool required) {
                if (!d.contains(key)) {
                    if (required) error(std::string{"/diffDrive/"} + key, "缺少字段");
                    return;
                }
                if (!d[key].is_string() || d[key].get<std::string>().empty()) error(std::string{"/diffDrive/"} + key, "应为非空字符串");
                else out = d[key].get<std::string>();
            };
            const auto positive = [&](const char* key, double& out, bool required, bool allowZero) {
                if (!d.contains(key)) {
                    if (required) error(std::string{"/diffDrive/"} + key, "缺少字段");
                    return;
                }
                const Json& v = d[key];
                if (!v.is_number() || !std::isfinite(v.get<double>()) || v.get<double>() < 0 || (!allowZero && v.get<double>() == 0)) {
                    error(std::string{"/diffDrive/"} + key, allowZero ? "应为非负数" : "应为正数");
                } else {
                    out = v.get<double>();
                }
            };
            textField("leftJoint", dd.leftJoint, true);
            textField("rightJoint", dd.rightJoint, true);
            textField("leftActuator", dd.leftActuator, true);
            textField("rightActuator", dd.rightActuator, true);
            textField("baseEntity", dd.baseEntity, true);
            textField("cmdVel", dd.cmdVel, false);
            textField("odom", dd.odom, false);
            textField("odomFrame", dd.odomFrame, false);
            positive("wheelRadius", dd.wheelRadius, true, false);
            positive("wheelSeparation", dd.wheelSeparation, true, false);
            positive("maxWheelSpeed", dd.maxWheelSpeed, false, true);
            positive("odomHz", dd.odomHz, false, false);
            positive("timeout", dd.timeout, false, false);
            p.diffDrive = dd;
        }
    }
    // ---- M5-13a：接入档案 ----
    readString("time", p.time);
    if (p.time != "sim" && p.time != "wall") error("/time", "应为 “sim”（仿真时间）或 “wall”（墙钟）");
    if (p.time == "wall") {
        // 墙钟时间戳时不发 /clock（订阅 /clock 的节点会把时间拨回仿真时间）
        if (root.contains("clock") && p.clock) error("/clock", "时间戳为墙钟（time 为 wall）时不能发布 /clock");
        p.clock = false;
    }
    if (root.contains("topicSettings")) readTopicSettings(root["topicSettings"], p.topicSettings, error);
    if (root.contains("tf")) readTf(root["tf"], p.tf, error);
    if (root.contains("groundTruth")) {
        GroundTruthSettings g;
        readGroundTruth(root["groundTruth"], g, error);
        p.groundTruth = g;
    }
    if (root.contains("cmdVel")) readCommandInputs(root["cmdVel"], p.cmdVel, error);
    if (root.contains("planarBase")) {
        PlanarBaseSettings b;
        readPlanarBase(root["planarBase"], b, error);
        p.planarBase = b;
    }
    if (p.diffDrive && p.planarBase && !p.planarBase->optional) error("/planarBase", "不能同时有差速底盘（diffDrive）与平面底盘（planarBase）");
    if (p.diffDrive && !p.cmdVel.empty() && root["diffDrive"].is_object() &&
        (root["diffDrive"].contains("cmdVel") || root["diffDrive"].contains("timeout"))) {
        error("/diffDrive/cmdVel", "有 cmdVel 时，差速底盘的速度指令由 cmdVel 给出，diffDrive 中不能再有 cmdVel、timeout");
    }
    if (p.tf.robot == "world" && p.tf.odom && (p.diffDrive || (p.planarBase && !p.planarBase->optional))) {
        error("/tf/robot", "实体 TF 为 world 时机身已有父坐标系 world，不能再发布 odom → 机身（把 tf.odom 设为 false，或 robot 设为 tree）");
    }
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
