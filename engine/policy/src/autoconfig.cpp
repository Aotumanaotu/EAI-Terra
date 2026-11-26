// 策略的自动配置（M4-10a），说明见 autoconfig.hpp。

#include "terra/policy/autoconfig.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <utility>

#include <nlohmann/json.hpp>

#include "terra/core/sha256.hpp"

namespace terra::policy {

namespace {

namespace fs = std::filesystem;
namespace td = terra::data;
using Json = nlohmann::json;

std::string number(double v) {
    std::ostringstream s;
    s << v;
    return s.str();
}

Json parseMeta(const OnnxInfo& onnx, const std::string& key) {
    const auto it = onnx.metadata.find(key);
    if (it == onnx.metadata.end()) return Json{};
    const Json j = Json::parse(it->second, nullptr, false);
    return j.is_discarded() ? Json{} : j;
}

// 元数据中的字符串：值是 JSON 编码的（带引号）时解码，否则原样
std::string metaString(const OnnxInfo& onnx, const std::string& key) {
    const auto it = onnx.metadata.find(key);
    if (it == onnx.metadata.end()) return {};
    const Json j = Json::parse(it->second, nullptr, false);
    return !j.is_discarded() && j.is_string() ? j.get<std::string>() : it->second;
}

std::vector<double> numbers(const Json& j) {
    std::vector<double> out;
    if (!j.is_array()) return out;
    for (const auto& v : j) {
        if (!v.is_number()) return {};
        out.push_back(v.get<double>());
    }
    return out;
}

std::vector<std::string> strings(const Json& j) {
    std::vector<std::string> out;
    if (!j.is_array()) return out;
    for (const auto& v : j) {
        if (!v.is_string()) return {};
        out.push_back(v.get<std::string>());
    }
    return out;
}

// 场景中的机器人：关节名对应、机身实体与 IMU
struct Robot {
    std::string instance;                 // 模型实例名（没有前缀时为空）
    std::vector<std::string> joints;      // 与策略的关节顺序对应的全名
    std::string base;                     // 机身实体全名
    std::string imu;                      // 机身上的 IMU 全名（没有为空）
};

std::optional<Robot> matchRobot(const std::vector<std::string>& names, const td::ModelDescription& d, const std::string& wanted, AutoconfigResult& r) {
    // 每个策略关节在场景中的候选（全名等于它，或以 “/名字” 结尾）
    std::set<std::string> instances;
    for (const auto& j : d.joints) {
        for (const auto& n : names) {
            if (j.name == n) instances.insert("");
            else if (j.name.size() > n.size() && j.name.compare(j.name.size() - n.size(), n.size(), n) == 0 && j.name[j.name.size() - n.size() - 1] == '/') {
                instances.insert(j.name.substr(0, j.name.size() - n.size() - 1));
            }
        }
    }
    std::vector<std::string> complete;
    for (const auto& inst : instances) {
        bool all = true;
        for (const auto& n : names) all = all && d.findJoint(inst.empty() ? n : inst + "/" + n) != nullptr;
        if (all) complete.push_back(inst);
    }
    if (!wanted.empty()) {
        if (std::find(complete.begin(), complete.end(), wanted) == complete.end()) {
            r.errors.push_back("模型实例“" + wanted + "”中没有策略需要的全部关节（" + std::to_string(names.size()) + " 个）");
            return std::nullopt;
        }
        complete = {wanted};
    }
    if (complete.empty()) {
        std::string missing;
        for (const auto& n : names) {
            bool found = false;
            for (const auto& j : d.joints) found = found || j.name == n || (j.name.size() > n.size() && j.name.compare(j.name.size() - n.size(), n.size(), n) == 0);
            if (!found) missing += (missing.empty() ? "" : "、") + n;
        }
        r.errors.push_back("场景中没有一个机器人包含策略的全部关节" + (missing.empty() ? std::string{} : "（缺少：" + missing + "）") +
                           "：请选择与策略对应的机器人");
        return std::nullopt;
    }
    if (complete.size() > 1) {
        std::string list;
        for (const auto& c : complete) list += (list.empty() ? "" : "、") + c;
        r.errors.push_back("场景中有多个机器人与策略的关节匹配（" + list + "）：请指定模型实例");
        return std::nullopt;
    }
    Robot robot;
    robot.instance = complete[0];
    const std::string prefix = robot.instance.empty() ? "" : robot.instance + "/";
    for (const auto& n : names) robot.joints.push_back(prefix + n);
    // 机身：关节所在的模型实例中自由关节的实体
    const td::JointDescription* first = d.findJoint(robot.joints[0]);
    const td::EntityDescription* firstEntity = first ? d.entity(first->entity) : nullptr;
    const td::ModelInstanceId model = firstEntity ? firstEntity->modelInstance : td::ModelInstanceId{};
    const td::EntityDescription* base = nullptr;
    for (const auto& j : d.joints) {
        const td::EntityDescription* e = d.entity(j.entity);
        if (j.type == td::JointType::Free && e && e->modelInstance.value == model.value) {
            base = e;
            break;
        }
    }
    if (base == nullptr) {
        r.errors.push_back("机器人“" + robot.instance + "”没有浮动基座（自由关节）：速度指令类的行走策略需要移动机器人");
        return std::nullopt;
    }
    robot.base = base->name;
    const td::SensorDescription* fallback = nullptr;
    for (const auto& s : d.sensors) {
        if (s.kind != td::SensorKind::Imu || s.modelInstance.value != model.value) continue;
        if (s.entity.value == base->id.value) {
            robot.imu = s.name;
            break;
        }
        if (fallback == nullptr) fallback = &s;
    }
    if (robot.imu.empty() && fallback) {
        robot.imu = fallback->name;
        r.warnings.push_back("机身上没有 IMU，使用装在其他部位的“" + fallback->name + "”（角速度与重力方向可能与训练时不同）");
    }
    if (robot.imu.empty()) {
        r.warnings.push_back("场景中机器人身上没有 IMU：角速度与重力方向改用机身的真值（没有噪声）。可在场景文件的 sensors 中添加 IMU");
    }
    // M4-10b：机器人上还有不由策略控制的关节（如 Go2W 的轮子）：可能不是训练时的机器人
    const auto match = matchRobotJoints(names, d);
    if (!match.extra.empty()) {
        std::string list;
        for (std::size_t i = 0; i < match.extra.size() && i < 4; ++i) list += (i ? "、" : "") + match.extra[i];
        if (match.extra.size() > 4) list += " 等";
        r.warnings.push_back("机器人“" + robot.instance + "”上还有 " + std::to_string(match.extra.size()) + " 个关节不由策略控制（" + list +
                             "）：可能与训练时的机器人不同，运行时这些关节不受控制");
    }
    return robot;
}

// 训练配置：模型所在目录及上两级
std::string findTrainingConfig(const std::string& model) {
    fs::path dir = fs::path{model}.parent_path();
    for (int i = 0; i < 3 && !dir.empty(); ++i) {
        std::error_code ec;
        const fs::path p = dir / "task_config.json";
        if (fs::is_regular_file(p, ec)) return p.string();
        if (dir == dir.parent_path()) break;
        dir = dir.parent_path();
    }
    return {};
}

// 关节名（不带前缀）按 “_” 分段，与关节组名（hip、thigh、calf 等）相同的一段即属于该组
std::optional<double> groupValue(const Json& groups, const std::string& joint) {
    if (!groups.is_object()) return std::nullopt;
    std::vector<std::string> parts;
    std::string part;
    for (char ch : joint) {
        if (ch == '_') {
            parts.push_back(part);
            part.clear();
        } else {
            part += ch;
        }
    }
    parts.push_back(part);
    for (auto it = groups.begin(); it != groups.end(); ++it) {
        if (!it.value().is_number()) continue;
        if (std::find(parts.begin(), parts.end(), it.key()) != parts.end()) return it.value().get<double>();
    }
    return std::nullopt;
}

// ---- supermimic_moe_velocity_policy_v1 ----
void configureSupermimic(const FormatContext& ctx, AutoconfigResult& r) {
    const OnnxInfo& onnx = ctx.onnx;
    PolicyDescription p;
    p.model = ctx.input.model;
    p.scene = ctx.input.scene;
    p.backend = "onnxruntime-cpu";
    const auto meta = [&](const char* key) { return metaString(onnx, key); };
    p.inputName = meta("input_name");
    p.outputName = meta("output_name");
    const std::vector<std::string> joints = strings(parseMeta(onnx, "action_joint_names"));
    const std::vector<std::string> obsJoints = strings(parseMeta(onnx, "joint_names"));
    const std::vector<double> defaults = numbers(parseMeta(onnx, "default_joint_positions"));
    const std::vector<double> scales = numbers(parseMeta(onnx, "action_scales"));
    if (joints.empty()) r.errors.push_back("元数据中没有 action_joint_names");
    if (defaults.size() != joints.size()) r.errors.push_back("元数据 default_joint_positions 的长度与关节数不一致");
    if (scales.size() != joints.size()) r.errors.push_back("元数据 action_scales 的长度与关节数不一致");
    if (!obsJoints.empty() && obsJoints != joints) r.errors.push_back("元数据 joint_names 与 action_joint_names 的顺序不同（本步只支持相同的顺序）");
    if (meta("action_semantics") != "unscaled_joint_position_offsets") r.errors.push_back("元数据 action_semantics 不是 unscaled_joint_position_offsets");
    if (!r.errors.empty()) return;

    const auto robot = matchRobot(joints, ctx.robot, ctx.input.instance, r);
    if (!robot) return;
    r.notes.push_back("机器人：" + (robot->instance.empty() ? std::string{"（单个模型）"} : robot->instance) + "，" + std::to_string(joints.size()) +
                      " 个关节按名字对应（" + joints.front() + " → " + robot->joints.front() + " …）");

    const Json dt = parseMeta(onnx, "control_dt_seconds");
    p.controlPeriod = dt.is_number() ? dt.get<double>() : 0.0;
    if (!(p.controlPeriod > 0)) r.errors.push_back("元数据中没有 control_dt_seconds");
    const Json history = parseMeta(onnx, "history_length");
    p.historyLength = history.is_number_integer() ? history.get<int>() : 1;
    if (const auto order = meta("history_order"); !order.empty() && order != "frame_major_oldest_first") {
        r.errors.push_back("history_order = " + order + "（本步只支持 frame_major_oldest_first）");
    }
    if (const auto reset = meta("history_reset"); !reset.empty() && reset != "repeat_initial_frame") {
        r.errors.push_back("history_reset = " + reset + "（本步只支持 repeat_initial_frame）");
    }
    r.notes.push_back("控制周期 " + number(p.controlPeriod) + " s，历史 " + std::to_string(p.historyLength) + " 帧（最旧的在前，重置时重复初始帧）");

    // 观测布局
    const Json layout = parseMeta(onnx, "observation_layout");
    const Json phase = parseMeta(onnx, "gait_phase");
    if (!layout.is_array() || layout.empty()) {
        r.errors.push_back("元数据中没有 observation_layout");
        return;
    }
    for (const auto& item : layout) {
        const std::string name = item.contains("name") && item["name"].is_string() ? item["name"].get<std::string>() : std::string{};
        const int width = item.contains("width") && item["width"].is_number_integer() ? item["width"].get<int>() : -1;
        ObservationTerm t;
        if (name.rfind("base_angular_velocity", 0) == 0) {
            t.kind = TermKind::BaseAngularVelocity;
        } else if (name.rfind("projected_gravity", 0) == 0) {
            t.kind = TermKind::ProjectedGravity;
        } else if (name.rfind("base_linear_velocity", 0) == 0) {
            t.kind = TermKind::BaseLinearVelocity;
            t.entity = robot->base;
        } else if (name.rfind("joint_position", 0) == 0) {
            t.kind = TermKind::JointPositions;
            t.joints = robot->joints;
            t.offset = defaults;
            if (name.find("encoder_bias") != std::string::npos) {
                r.notes.push_back("观测 " + name + "：训练时含编码器偏置（域随机化），Terra 中偏置为 0");
            }
        } else if (name.rfind("joint_velocit", 0) == 0) {
            t.kind = TermKind::JointVelocities;
            t.joints = robot->joints;
        } else if (name.rfind("previous", 0) == 0 && name.find("action") != std::string::npos) {
            t.kind = TermKind::LastAction;
        } else if (name.rfind("velocity_command", 0) == 0) {
            t.kind = TermKind::Command;
        } else if (name.rfind("gait_phase", 0) == 0) {
            t.kind = TermKind::Phase;
            t.period = phase.contains("period_seconds") && phase["period_seconds"].is_number() ? phase["period_seconds"].get<double>() : 0.0;
            t.standingThreshold =
                phase.contains("standing_command_threshold") && phase["standing_command_threshold"].is_number() ? phase["standing_command_threshold"].get<double>() : 0.0;
            if (!(t.period > 0)) r.errors.push_back("元数据 gait_phase 中没有 period_seconds");
        } else {
            r.errors.push_back("不认识的观测项“" + name + "”（宽度 " + std::to_string(width) + "）");
            continue;
        }
        if (t.kind == TermKind::BaseAngularVelocity || t.kind == TermKind::ProjectedGravity) {
            if (robot->imu.empty()) t.entity = robot->base;
            else t.sensor = robot->imu;
        }
        p.action.joints = robot->joints;  // termWidth 需要
        if (p.termWidth(t) != width) {
            r.errors.push_back("观测项“" + name + "”的宽度为 " + std::to_string(width) + "，按场景应为 " + std::to_string(p.termWidth(t)));
        }
        p.observation.push_back(std::move(t));
    }
    const Json dim = parseMeta(onnx, "observation_dim");
    if (dim.is_number_integer() && dim.get<int>() != p.frameWidth()) {
        r.errors.push_back("observation_dim = " + std::to_string(dim.get<int>()) + "，各观测项之和为 " + std::to_string(p.frameWidth()));
    }
    r.notes.push_back("观测（每帧 " + std::to_string(p.frameWidth()) + " 维）：角速度与重力方向取 " +
                      (robot->imu.empty() ? "机身真值" : "IMU " + robot->imu + "（带噪声）") + "；关节位置、速度、上一次动作、速度指令、步态相位");

    // 动作
    p.action.type = ActionType::PositionPd;
    p.action.joints = robot->joints;
    p.action.scale = scales;
    p.action.offset = defaults;

    // 训练配置：PD 增益、力矩上限、指令范围
    if (ctx.trainingConfig.empty()) {
        r.errors.push_back("没有找到训练配置 task_config.json（模型所在目录及上两级），无法确定 PD 增益（kp、kd）");
    } else {
        Json cfg = Json::parse(ctx.trainingConfigText, nullptr, false);
        if (cfg.is_object() && cfg.contains("config") && cfg["config"].is_object()) cfg = cfg["config"];
        const Json control = cfg.is_object() && cfg.contains("control") ? cfg["control"] : Json{};
        bool complete = true;
        for (const auto& n : joints) {
            const auto kp = groupValue(control.is_object() && control.contains("stiffness") ? control["stiffness"] : Json{}, n);
            const auto kd = groupValue(control.is_object() && control.contains("damping") ? control["damping"] : Json{}, n);
            const auto limit = groupValue(control.is_object() && control.contains("effort_limit") ? control["effort_limit"] : Json{}, n);
            if (!kp || !kd) {
                complete = false;
                r.errors.push_back("训练配置中没有关节“" + n + "”的 stiffness / damping");
                continue;
            }
            p.action.kp.push_back(*kp);
            p.action.kd.push_back(*kd);
            p.action.torqueLimit.push_back(limit.value_or(0.0));
        }
        if (complete) {
            std::string gains;
            if (control.contains("stiffness") && control["stiffness"].is_object()) {
                for (auto it = control["stiffness"].begin(); it != control["stiffness"].end(); ++it) {
                    gains += (gains.empty() ? "" : "，") + it.key() + " " + number(it.value().is_number() ? it.value().get<double>() : 0.0);
                }
            }
            r.notes.push_back("PD 增益与力矩上限来自训练配置（kp：" + gains + "）：" + ctx.trainingConfig);
        }
        if (cfg.contains("commands") && cfg["commands"].is_object() && cfg["commands"].contains("initial_ranges")) {
            const Json& ranges = cfg["commands"]["initial_ranges"];
            if (ranges.is_array() && ranges.size() == 3) {
                for (std::size_t i = 0; i < 3; ++i) {
                    const auto v = numbers(ranges[i]);
                    if (v.size() == 2 && v[0] <= v[1]) p.commandRange[i] = {v[0], v[1]};
                }
            }
        }
        if (cfg.contains("sim") && cfg["sim"].is_object() && cfg["sim"].contains("timestep") && cfg["sim"]["timestep"].is_number()) {
            const double trained = cfg["sim"]["timestep"].get<double>();
            if (std::fabs(trained - ctx.robot.timestep) > 1e-9) {
                r.warnings.push_back("训练时的物理步长为 " + number(trained) + " s，场景为 " + number(ctx.robot.timestep) +
                                     " s（sim2sim 的差别可能影响表现）");
            }
        }
    }
    if (!r.errors.empty()) return;
    const std::string iteration = meta("iteration");
    const fs::path modelPath{ctx.input.model};
    const std::string run = modelPath.parent_path().filename().string();
    const std::string experiment = modelPath.parent_path().parent_path().filename().string();
    p.name = experiment + " / " + run + (iteration.empty() ? "" : "（迭代 " + iteration + "）");
    p.source["format"] = meta("format");
    if (!meta("spec_hash").empty()) p.source["specHash"] = meta("spec_hash");
    if (!iteration.empty()) p.source["iteration"] = iteration;
    if (!meta("reference_commit").empty()) p.source["referenceCommit"] = meta("reference_commit");
    if (!ctx.trainingConfig.empty()) p.source["trainingConfig"] = ctx.trainingConfig;
    p.source["generatedBy"] = "terra autoconfig (M4-10a)";
    r.policy = std::move(p);
}

std::mutex& formatsMutex() {
    static std::mutex m;
    return m;
}

std::vector<PolicyFormat>& formats() {
    static std::vector<PolicyFormat> list = [] {
        std::vector<PolicyFormat> v;
        v.push_back({"supermimic_moe_velocity_policy_v1",
                     [](const OnnxInfo& onnx) { return metaString(onnx, "format") == "supermimic_moe_velocity_policy_v1"; },
                     configureSupermimic});
        return v;
    }();
    return list;
}

}  // namespace

void registerPolicyFormat(PolicyFormat format) {
    std::lock_guard<std::mutex> lock(formatsMutex());
    formats().push_back(std::move(format));
}

std::vector<std::string> policyFormats() {
    std::lock_guard<std::mutex> lock(formatsMutex());
    std::vector<std::string> ids;
    for (const auto& f : formats()) ids.push_back(f.id);
    return ids;
}

AutoconfigResult autoconfigure(const AutoconfigInput& input, const td::ModelDescription& robot) {
    AutoconfigResult r;
    const std::string ext = fs::path{input.model}.extension().string();
    if (ext == ".pt" || ext == ".pth") {
        r.errors.push_back("PyTorch 文件（.pt）的导入在 M4-10b 中提供（TorchScript 转为 ONNX；训练检查点找同一 spec_hash 的 ONNX）。"
                           "目前请选择 ONNX 文件");
        return r;
    }
    std::string error;
    const auto onnx = readOnnxInfo(input.model, error);
    if (!onnx) {
        r.errors.push_back(error);
        return r;
    }
    PolicyFormat format;
    {
        std::lock_guard<std::mutex> lock(formatsMutex());
        for (const auto& f : formats()) {
            if (f.matches(*onnx)) {
                format = f;
                break;
            }
        }
    }
    if (!format.configure) {
        const std::string name = metaString(*onnx, "format");
        r.errors.push_back("无法识别模型的格式" + (name.empty() ? std::string{"（ONNX 中没有 format 元数据）"} : "“" + name + "”") +
                           "：目前能自动配置的格式：supermimic_moe_velocity_policy_v1。其他格式请手写说明文件（M4-10b 的向导会列出需要填写的项）");
        return r;
    }
    r.format = format.id;
    FormatContext ctx{*onnx, robot, input,
                      findTrainingConfig(input.configDirectory.empty() ? input.model : (fs::path{input.configDirectory} / "x").string()), {}};
    if (!ctx.trainingConfig.empty()) {
        std::ifstream in(ctx.trainingConfig, std::ios::binary);
        ctx.trainingConfigText.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    }
    format.configure(ctx, r);
    if (r.policy && !onnx->inputs.empty() && !onnx->inputs[0].dims.empty() && onnx->inputs[0].dims.back() != r.policy->inputWidth()) {
        r.errors.push_back("模型输入的宽度为 " + std::to_string(onnx->inputs[0].dims.back()) + "，按元数据应为 " + std::to_string(r.policy->inputWidth()));
    }
    if (r.policy) {
        r.policy->modelSha256 = terra::sha256File(input.model).value_or("");
        if (!r.errors.empty()) r.policy.reset();
    }
    return r;
}

std::vector<std::string> policyJointNames(const std::string& path) {
    if (path.size() >= std::string{kPolicySuffix}.size() && path.compare(path.size() - std::string{kPolicySuffix}.size(), std::string::npos, kPolicySuffix) == 0) {
        const auto r = readPolicyFile(path);
        std::vector<std::string> out;
        if (!r.policy) return out;
        for (const auto& j : r.policy->action.joints) {
            const auto slash = j.rfind('/');
            out.push_back(slash == std::string::npos ? j : j.substr(slash + 1));
        }
        return out;
    }
    std::string error;
    const auto onnx = readOnnxInfo(path, error);
    if (!onnx) return {};
    return strings(parseMeta(*onnx, "action_joint_names"));
}

RobotMatch matchRobotJoints(const std::vector<std::string>& names, const td::ModelDescription& d) {
    RobotMatch m;
    m.wanted = names.size();
    if (names.empty()) return m;
    // 每个实例（全名前缀）中找到的策略关节数
    std::map<std::string, std::size_t> found;
    for (const auto& j : d.joints) {
        for (const auto& n : names) {
            if (j.name == n) ++found[""];
            else if (j.name.size() > n.size() && j.name.compare(j.name.size() - n.size(), n.size(), n) == 0 && j.name[j.name.size() - n.size() - 1] == '/') {
                ++found[j.name.substr(0, j.name.size() - n.size() - 1)];
            }
        }
    }
    for (const auto& [inst, count] : found) {
        if (count == names.size()) ++m.instances;
        if (count > m.matched) {
            m.matched = count;
            m.instance = inst;
        }
    }
    if (m.matched == 0) return m;
    // 该实例中其他的转动 / 滑动关节
    const std::string prefix = m.instance.empty() ? "" : m.instance + "/";
    std::set<std::string> mine;
    for (const auto& n : names) mine.insert(prefix + n);
    const td::JointDescription* any = d.findJoint(prefix + names.front());
    const td::EntityDescription* e = any ? d.entity(any->entity) : nullptr;
    for (const auto& j : d.joints) {
        if (j.type != td::JointType::Hinge && j.type != td::JointType::Slide) continue;
        const td::EntityDescription* je = d.entity(j.entity);
        if (!e || !je || je->modelInstance.value != e->modelInstance.value || mine.count(j.name)) continue;
        m.extra.push_back(j.name);
    }
    return m;
}

}  // namespace terra::policy
