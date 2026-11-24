#include "cli.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace terra::headless {

namespace {

// 严格解析正整数：只允许十进制数字，拒绝符号、空白、前后缀与溢出。
std::optional<std::uint64_t> parsePositiveInteger(std::string_view text) {
    if (text.empty()) {
        return std::nullopt;
    }
    std::uint64_t value = 0;
    constexpr std::uint64_t kMax = std::numeric_limits<std::uint64_t>::max();
    for (const char c : text) {
        if (c < '0' || c > '9') {
            return std::nullopt;
        }
        const auto digit = static_cast<std::uint64_t>(c - '0');
        if (value > (kMax - digit) / 10) {
            return std::nullopt;
        }
        value = value * 10 + digit;
    }
    if (value == 0) {
        return std::nullopt;
    }
    return value;
}

// 严格解析正的有限实数：整个字符串必须被消费，拒绝 nan/inf、十六进制与前导空白。
std::optional<double> parsePositiveFinite(std::string_view text) {
    if (text.empty() || text.front() == ' ' || text.front() == '\t' || text.front() == '+' ||
        text.front() == '-') {
        return std::nullopt;
    }
    for (const char c : text) {
        const bool allowed = (c >= '0' && c <= '9') || c == '.' || c == 'e' || c == 'E' ||
                             c == '+' || c == '-';
        if (!allowed) {
            return std::nullopt;
        }
    }
    const std::string buffer{text};
    char* end = nullptr;
    errno = 0;
    const double value = std::strtod(buffer.c_str(), &end);
    if (errno != 0 || end != buffer.c_str() + buffer.size() || !std::isfinite(value) ||
        value <= 0.0) {
        return std::nullopt;
    }
    return value;
}

std::string quoted(std::string_view text) {
    return "'" + std::string{text} + "'";
}

}  // namespace

ParseResult parseArguments(int argc, const char* const* argv) {
    ParseResult result;

    for (int i = 1; i < argc; ++i) {
        const std::string_view arg{argv[i]};
        if (arg == "-h" || arg == "--help") {
            result.command = Command::Help;
            return result;
        }
        if (arg == "--version") {
            result.command = Command::Version;
            return result;
        }
    }

    if (argc <= 1) {
        result.error = "缺少参数";
        return result;
    }

    bool seenModel = false;
    bool seenScene = false;
    bool seenMotion = false;
    bool seenRecord = false;
    bool seenRerun = false;
    bool seenTolerance = false;
    bool seenSampleEvery = false;
    RunOptions& options = result.options;

    for (int i = 1; i < argc; ++i) {
        std::string_view arg{argv[i]};

        if (arg == "--list-records") {
            if (options.listRecords) {
                result.error = "重复的参数 --list-records";
                return result;
            }
            options.listRecords = true;
            continue;
        }
        if (arg == "--sensors") {
            if (options.sensors) {
                result.error = "重复的参数 --sensors";
                return result;
            }
            options.sensors = true;
            continue;
        }
        if (arg == "--dry-run") {
            if (options.dryRun) {
                result.error = "重复的参数 --dry-run";
                return result;
            }
            options.dryRun = true;
            continue;
        }

        // 取出 "--name" 与其值，支持 "--name=value" 和 "--name value"。
        std::string_view name = arg;
        std::optional<std::string_view> value;
        if (const auto eq = arg.find('='); arg.substr(0, 2) == "--" && eq != std::string_view::npos) {
            name = arg.substr(0, eq);
            value = arg.substr(eq + 1);
        }

        const bool takesValue = name == "--model" || name == "--scene" || name == "--motion" || name == "--steps" ||
                                name == "--duration" || name == "--sample-every" || name == "--record" || name == "--rerun" ||
                                name == "--tolerance" || name == "--library";
        if (!takesValue) {
            result.error = "未知参数 " + quoted(arg);
            return result;
        }
        if (!value) {
            if (i + 1 >= argc) {
                result.error = "参数 " + std::string{name} + " 缺少值";
                return result;
            }
            const std::string_view next{argv[i + 1]};
            if (next.substr(0, 2) == "--") {
                result.error = "参数 " + std::string{name} + " 缺少值";
                return result;
            }
            value = next;
            ++i;
        }

        if (name == "--library") {
            if (!options.libraryPath.empty()) {
                result.error = "重复的参数 --library";
                return result;
            }
            if (value->empty()) {
                result.error = "--library 的值不能为空";
                return result;
            }
            options.libraryPath = std::string{*value};
        } else if (name == "--record" || name == "--rerun") {
            bool& seen = name == "--record" ? seenRecord : seenRerun;
            if (seen) {
                result.error = "重复的参数 " + std::string{name};
                return result;
            }
            if (value->empty()) {
                result.error = std::string{name} + " 的值不能为空";
                return result;
            }
            seen = true;
            (name == "--record" ? options.recordPath : options.rerunPath) = std::string{*value};
        } else if (name == "--tolerance") {
            if (seenTolerance) {
                result.error = "重复的参数 --tolerance";
                return result;
            }
            const auto comma = value->find(',');
            const auto position = comma == std::string_view::npos ? std::nullopt : parsePositiveFinite(value->substr(0, comma));
            const auto rotation = comma == std::string_view::npos ? std::nullopt : parsePositiveFinite(value->substr(comma + 1));
            if (!position || !rotation) {
                result.error = "--tolerance 需要“位置,姿态”两个正数（m,rad），如 1e-6,1e-6，实际为 " + quoted(*value);
                return result;
            }
            seenTolerance = true;
            options.positionTolerance = *position;
            options.rotationTolerance = *rotation;
        } else if (name == "--motion") {
            if (seenMotion) {
                result.error = "重复的参数 --motion";
                return result;
            }
            if (value->empty()) {
                result.error = "--motion 的值不能为空";
                return result;
            }
            seenMotion = true;
            options.motionPath = std::string{*value};
        } else if (name == "--scene") {
            if (seenScene) {
                result.error = "重复的参数 --scene";
                return result;
            }
            if (value->empty()) {
                result.error = "--scene 的值不能为空";
                return result;
            }
            seenScene = true;
            options.scenePath = std::string{*value};
        } else if (name == "--model") {
            if (seenModel) {
                result.error = "重复的参数 --model";
                return result;
            }
            if (value->empty()) {
                result.error = "--model 的值不能为空";
                return result;
            }
            seenModel = true;
            options.modelPath = std::string{*value};
        } else if (name == "--steps") {
            if (options.steps) {
                result.error = "重复的参数 --steps";
                return result;
            }
            options.steps = parsePositiveInteger(*value);
            if (!options.steps) {
                result.error = "--steps 需要正整数，实际为 " + quoted(*value);
                return result;
            }
        } else if (name == "--duration") {
            if (options.durationSeconds) {
                result.error = "重复的参数 --duration";
                return result;
            }
            options.durationSeconds = parsePositiveFinite(*value);
            if (!options.durationSeconds) {
                result.error = "--duration 需要正的有限秒数，实际为 " + quoted(*value);
                return result;
            }
        } else {  // --sample-every
            if (seenSampleEvery) {
                result.error = "重复的参数 --sample-every";
                return result;
            }
            const auto parsed = parsePositiveInteger(*value);
            if (!parsed) {
                result.error = "--sample-every 需要正整数，实际为 " + quoted(*value);
                return result;
            }
            seenSampleEvery = true;
            options.sampleEvery = *parsed;
        }
    }

    if (options.listRecords) {
        // 列出库中的记录：只能配合 --library 与（作为筛选条件的）--scene
        if (seenModel || seenMotion || seenRecord || seenRerun || seenTolerance || options.steps || options.durationSeconds ||
            options.dryRun || seenSampleEvery || options.sensors) {
            result.error = "--list-records 只能与 --library、--scene（按场景筛选）同用";
        }
        return result;
    }
    if (!options.libraryPath.empty() && !seenRecord) {
        result.error = "--library 只用于 --record 与 --list-records";
        return result;
    }
    if (seenRerun) {
        // 复现：一切由记录给出，不能再指定运行参数
        if (seenModel || seenScene || seenMotion || seenRecord || options.steps || options.durationSeconds || options.dryRun ||
            seenSampleEvery || options.sensors) {
            result.error = "--rerun 不能与 --model、--scene、--motion、--record、--steps、--duration、--sample-every、--dry-run 同用";
        }
        return result;
    }
    if (seenModel && seenScene) {
        result.error = "--model 与 --scene 只能二选一";
    } else if (seenTolerance) {
        result.error = "--tolerance 只用于 --rerun";
    } else if (seenRecord && seenModel) {
        result.error = "--record 需要场景文件（--scene 或 --motion），不能与 --model 同用";
    } else if (seenRecord && options.dryRun) {
        result.error = "--record 不能与 --dry-run 同用";
    } else if (seenModel && seenMotion) {
        result.error = "--motion 需要场景文件（--scene，或动作文件中的推荐场景），不能与 --model 同用";
    } else if (!seenModel && !seenScene && !seenMotion) {
        result.error = "缺少 --model（或 --scene、--motion）";
    } else if (options.steps && options.durationSeconds) {
        result.error = "--steps 与 --duration 不能同时使用";
    } else if (!options.steps && !options.durationSeconds && !seenMotion) {
        result.error = "需要 --steps 或 --duration 之一（给出 --motion 时可省略：运行到动作结束后 1 秒）";
    }
    return result;
}

std::string usageText() {
    return "用法:\n"
           "  terra_headless (--model <MJCF> | --scene <场景文件>) (--steps N | --duration 秒) [--sample-every N] [--dry-run]\n"
           "  terra_headless [--scene <场景文件>] --motion <动作文件> [--steps N | --duration 秒] [--sample-every N]\n"
           "  terra_headless ... [--record <记录文件>]          （与 --scene 或 --motion 组合：写出实验记录）\n"
           "  terra_headless --rerun <记录文件> [--tolerance 位置,姿态]\n"
           "  terra_headless --list-records [--scene <场景文件>] [--library <库文件>]\n"
           "  terra_headless --help | --version\n"
           "\n"
           "参数:\n"
           "  --model <路径>       MJCF 模型文件\n"
           "  --scene <路径>       Terra 场景文件（M2-08，*.terra-scene.json）：组合其中的模型实例后运行，只跑物理，\n"
           "                       不读取视觉资源；实体名带实例前缀（如 cart_a/chassis）。场景中的 PD 层（controllers）同样生效\n"
           "  --motion <路径>      动作文件（M3-05，*.terra-motion.json）：从起始关键帧开始按步执行，与查看器逐位相同；\n"
           "                       不给 --scene 时使用动作文件中的推荐场景；不给步数时运行到动作结束后 1 秒\n"
           "  --record <路径>      实验记录（M3-07，*.terra-record.jsonl）：记录头、每步实际下发的控制量（变化时）、\n"
           "                       按 --sample-every 采样的状态与摘要；stdout 的输出不变\n"
           "  --rerun <路径>       按实验记录重新仿真：加载记录中的场景，下发记录的控制量，在每个采样点比较位姿；\n"
           "                       输出一行 {\"type\":\"rerun\",...}；超出容差时退出码 6\n"
           "  --tolerance p,r      复现容差：位置（m）与姿态（rad），默认 1e-6,1e-6\n"
           "  --library <路径>     资源库文件（M3-07b，SQLite）：--record 完成后把记录加入索引；默认 $TERRA_LIBRARY，\n"
           "                       否则 ~/.local/share/eai-terra/library.sqlite3。索引失败只警告，不影响记录文件\n"
           "  --list-records       列出库中的实验记录（每行一个 {\"type\":\"record\",...}，按创建时间从新到旧）\n"
           "  --steps N            运行 N 个固定物理步（正整数）\n"
           "  --duration 秒        运行到指定仿真时长（正数；与 --steps 二选一）\n"
           "  --sample-every N     每 N 步输出一次状态（默认 1；第 0 步与最后一步总是输出）\n"
           "  --sensors            输出传感器读数（M4-02）：model 之后一行 sensor_list（清单与频率），之后按各传感器的频率\n"
           "                       输出 sensor 行（第 0 步总是输出）；频率来自场景文件的 sensors，默认等于物理频率\n"
           "  --dry-run            校验参数并加载模型，输出解析后的配置，不运行仿真\n"
           "  -h, --help           显示本帮助\n"
           "      --version        显示版本与构建信息\n"
           "\n"
           "输出: stdout 为 JSON Lines（每行一个对象，带 \"type\" 字段）；诊断信息写 stderr。\n"
           "退出码: 0 成功；1 内部错误；2 参数错误；3 后端不可用或功能未接入；4 模型错误；5 仿真错误；6 复现超出容差；\n"
           "        70 物理后端致命错误（不可恢复；诊断写入 stderr 与 ~/.local/state/terra/crash-*.txt）。\n"
           "\n"
           "\n"
           "输出记录: config（参数）→ model（配置与实体清单）→ state（第 0 步、每 N 步、最后一步）\n"
           "          → summary（正常结束）或 error（仿真错误，退出码 5）。--dry-run 只输出 config。\n";
}

std::string jsonEscape(std::string_view text) {
    std::string out;
    out.reserve(text.size());
    for (const char c : text) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\b': out += "\\b"; break;
            case '\f': out += "\\f"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof buf, "\\u%04x", static_cast<unsigned>(c));
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    return out;
}

std::optional<std::uint64_t> stepsForDuration(double durationSeconds, double timestep) {
    if (!std::isfinite(durationSeconds) || !std::isfinite(timestep) || durationSeconds <= 0.0 ||
        timestep <= 0.0) {
        return std::nullopt;
    }
    const double ratio = durationSeconds / timestep;
    // 2^63：可以精确转换为 uint64 的安全上界
    if (!std::isfinite(ratio) || ratio >= 9223372036854775808.0) {
        return std::nullopt;
    }
    const double nearest = std::round(ratio);
    const double steps =
        std::fabs(ratio - nearest) <= 1e-9 * std::max(1.0, ratio) ? nearest : std::ceil(ratio);
    return static_cast<std::uint64_t>(std::max(1.0, steps));
}

std::string formatNumber(double value) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.17g", value);
    return buf;
}

namespace {

std::string quotedJson(std::string_view text) {
    return "\"" + jsonEscape(text) + "\"";
}

std::string vec3Json(const terra::data::Vec3& v) {
    return "[" + formatNumber(v.x) + "," + formatNumber(v.y) + "," + formatNumber(v.z) + "]";
}

const char* jointTypeName(terra::data::JointType type) {
    switch (type) {
        case terra::data::JointType::Free: return "free";
        case terra::data::JointType::Ball: return "ball";
        case terra::data::JointType::Slide: return "slide";
        case terra::data::JointType::Hinge: return "hinge";
    }
    return "unknown";
}

}  // namespace

// Headless 输出格式的修订号。M3-04 起快照（PhysicsSnapshot::kContractRevision = 2）增加了执行器状态，
// Headless 的输出暂不变（M3-04 设计稿第 5 节），仍为 1；M3-07 定记录格式时再升。
constexpr std::uint32_t kOutputRevision = 1;

std::string modelToJson(const terra::data::ModelDescription& model) {
    const auto& st = model.settings;
    std::string json = "{\"type\":\"model\",\"contract_revision\":" +
                       std::to_string(kOutputRevision) +
                       ",\"conventions\":{\"units\":\"SI\",\"up_axis\":\"+z\",\"quaternion\":\"wxyz\","
                       "\"entity_velocity\":\"entity_origin_world\",\"free_joint_angular_velocity\":\"body_local\","
                       "\"ball_joint_angular_velocity\":\"child_local\",\"contact_entity_0\":\"world_static\"}" +
                       ",\"timestep\":" + formatNumber(model.timestep) +
                       ",\"gravity\":" + vec3Json(st.gravity) + ",\"integrator\":" + quotedJson(st.integrator) +
                       ",\"solver\":" + quotedJson(st.solver) + ",\"iterations\":" + std::to_string(st.iterations) +
                       ",\"tolerance\":" + formatNumber(st.tolerance) + ",\"entities\":[";
    for (std::size_t i = 0; i < model.entities.size(); ++i) {
        const auto& e = model.entities[i];
        json += (i ? "," : "");
        json += "{\"id\":" + std::to_string(e.id.value) + ",\"name\":" + quotedJson(e.name) +
                ",\"parent\":" + std::to_string(e.parent.value) + "}";
    }
    json += "],\"joints\":[";
    for (std::size_t i = 0; i < model.joints.size(); ++i) {
        const auto& j = model.joints[i];
        json += (i ? "," : "");
        json += "{\"id\":" + std::to_string(j.id.value) + ",\"name\":" + quotedJson(j.name) +
                ",\"type\":\"" + jointTypeName(j.type) + "\",\"entity\":" + std::to_string(j.entity.value) +
                ",\"nq\":" + std::to_string(j.configurationSize) + ",\"nv\":" + std::to_string(j.velocitySize) +
                "}";
    }
    json += "],\"actuators\":[";
    for (std::size_t i = 0; i < model.actuators.size(); ++i) {
        const auto& a = model.actuators[i];
        json += (i ? "," : "");
        json += "{\"id\":" + std::to_string(a.id.value) + ",\"name\":" + quotedJson(a.name) +
                ",\"transmission\":\"" +
                (a.transmission == terra::data::ActuatorTransmission::Joint ? "joint" : "other") +
                "\",\"joint\":" + std::to_string(a.joint.value) + ",\"inputs\":[";
        for (std::size_t k = 0; k < a.inputs.size(); ++k) {
            const auto& in = a.inputs[k];
            json += (k ? "," : "");
            json += "{\"name\":" + quotedJson(in.name) + ",\"limited\":" + (in.limited ? "true" : "false");
            if (in.limited) {
                json += ",\"range\":[" + formatNumber(in.min) + "," + formatNumber(in.max) + "]";
            }
            json += "}";
        }
        json += "]}";
    }
    json += "]}";
    return json;
}

std::string stateToJson(const terra::data::PhysicsSnapshot& snapshot) {
    std::string json = "{\"type\":\"state\",\"run_epoch\":" + std::to_string(snapshot.runEpoch) +
                       ",\"step\":" + std::to_string(snapshot.stepIndex) +
                       ",\"time\":" + formatNumber(snapshot.simulationTime) + ",\"entities\":[";
    for (std::size_t i = 0; i < snapshot.entities.size(); ++i) {
        const auto& e = snapshot.entities[i];
        const auto& q = e.pose.orientation;
        json += (i ? "," : "");
        json += "{\"id\":" + std::to_string(e.id.value) + ",\"position\":" + vec3Json(e.pose.position) +
                ",\"orientation\":[" + formatNumber(q.w) + "," + formatNumber(q.x) + "," + formatNumber(q.y) + "," +
                formatNumber(q.z) + "],\"linear_velocity\":" + vec3Json(e.linearVelocity) +
                ",\"angular_velocity\":" + vec3Json(e.angularVelocity) + "}";
    }
    json += "],\"joints\":[";
    for (std::size_t i = 0; i < snapshot.joints.size(); ++i) {
        const auto& j = snapshot.joints[i];
        json += (i ? "," : "");
        json += "{\"id\":" + std::to_string(j.id.value) + ",\"position\":[";
        for (int k = 0; k < j.positionSize; ++k) {
            json += (k ? "," : "") + formatNumber(j.position[static_cast<std::size_t>(k)]);
        }
        json += "],\"velocity\":[";
        for (int k = 0; k < j.velocitySize; ++k) {
            json += (k ? "," : "") + formatNumber(j.velocity[static_cast<std::size_t>(k)]);
        }
        json += "]}";
    }
    const auto& c = snapshot.contacts;
    json += "],\"contacts\":{\"candidates\":" + std::to_string(c.candidateCount) +
            ",\"active\":" + std::to_string(c.activeCount) + ",\"pairs\":[";
    for (std::size_t i = 0; i < c.pairs.size(); ++i) {
        const auto& pr = c.pairs[i];
        json += (i ? "," : "");
        json += "{\"entities\":[" + std::to_string(pr.entityA.value) + "," + std::to_string(pr.entityB.value) +
                "],\"points\":" + std::to_string(pr.pointCount) +
                ",\"normal_force\":" + formatNumber(pr.normalForce) + "}";
    }
    json += "]},\"diagnostics\":{\"finite\":";
    json += snapshot.diagnostics.finite ? "true" : "false";
    json += ",\"solver_iterations\":" + std::to_string(snapshot.diagnostics.solverIterations) + "}}";
    return json;
}

std::string sensorListToJson(const terra::data::ModelDescription& model, const std::vector<double>& rates) {
    std::string json = "{\"type\":\"sensor_list\",\"sensors\":[";
    for (std::size_t k = 0; k < model.sensors.size(); ++k) {
        const auto& s = model.sensors[k];
        if (k > 0) json += ",";
        json += "{\"id\":" + std::to_string(s.id.value) + ",\"name\":\"" + jsonEscape(s.name) + "\",\"kind\":\"imu\",\"entity\":" +
                std::to_string(s.entity.value) + ",\"added\":" + (s.added ? "true" : "false") + ",\"rate_hz\":" +
                formatNumber(k < rates.size() ? rates[k] : 0.0) + "}";
    }
    return json + "]}";
}

std::string sensorToJson(const terra::data::SensorDescription& sensor, const terra::data::SensorReading& reading,
                         std::uint64_t step, double time) {
    const auto array = [&](std::size_t from, std::size_t count) {
        std::string out = "[";
        for (std::size_t i = 0; i < count; ++i) {
            if (i > 0) out += ",";
            out += formatNumber(from + i < reading.values.size() ? reading.values[from + i] : 0.0);
        }
        return out + "]";
    };
    return "{\"type\":\"sensor\",\"id\":" + std::to_string(sensor.id.value) + ",\"name\":\"" + jsonEscape(sensor.name) +
           "\",\"kind\":\"imu\",\"step\":" + std::to_string(step) + ",\"time\":" + formatNumber(time) +
           ",\"linear_acceleration\":" + array(0, 3) + ",\"angular_velocity\":" + array(3, 3) + ",\"orientation\":" +
           array(6, 4) + "}";
}

std::string lidarToJson(const LidarJson& scan, const std::vector<float>& ranges) {
    std::string json = "{\"type\":\"lidar\",\"name\":\"" + jsonEscape(scan.name) + "\",\"step\":" + std::to_string(scan.step) +
                       ",\"time\":" + formatNumber(scan.time) + ",\"rate_hz\":" + formatNumber(scan.rateHz) + ",\"angle_min\":" +
                       formatNumber(scan.angleMin) + ",\"angle_increment\":" + formatNumber(scan.angleIncrement) + ",\"range_min\":" +
                       formatNumber(scan.rangeMin) + ",\"range_max\":" + formatNumber(scan.rangeMax) + ",\"ranges\":[";
    for (std::size_t i = 0; i < ranges.size(); ++i) {
        if (i > 0) json += ",";
        const float r = ranges[i];
        if (std::isinf(r)) json += r > 0 ? "\"inf\"" : "\"-inf\"";
        else json += formatNumber(static_cast<double>(r));
    }
    return json + "]}";
}

std::string summaryToJson(std::uint64_t runEpoch, std::uint64_t steps, double time) {
    return "{\"type\":\"summary\",\"run_epoch\":" + std::to_string(runEpoch) +
           ",\"steps\":" + std::to_string(steps) + ",\"time\":" + formatNumber(time) +
           ",\"status\":\"ok\"}";
}

std::string errorToJson(std::string_view kind, std::uint64_t runEpoch, std::uint64_t step, double time,
                        std::string_view message) {
    return "{\"type\":\"error\",\"kind\":" + quotedJson(kind) + ",\"run_epoch\":" + std::to_string(runEpoch) +
           ",\"step\":" + std::to_string(step) + ",\"time\":" + formatNumber(time) +
           ",\"message\":" + quotedJson(message) + "}";
}

std::string configToJson(const RunOptions& options, const ResolvedRun& resolved) {
    std::string json = "{\"type\":\"config\",\"model\":\"" + jsonEscape(options.modelPath) +
                       "\",\"model_resolved\":\"" + jsonEscape(resolved.modelPath) + "\"";
    if (!options.scenePath.empty()) json += ",\"scene\":\"" + jsonEscape(options.scenePath) + "\"";
    if (!options.motionPath.empty()) json += ",\"motion\":\"" + jsonEscape(options.motionPath) + "\"";
    if (options.steps) {
        json += ",\"steps\":" + std::to_string(*options.steps);
    }
    if (options.durationSeconds) {
        json += ",\"duration\":" + formatNumber(*options.durationSeconds);
    }
    json += ",\"timestep\":" + formatNumber(resolved.timestep);
    json += ",\"total_steps\":" + std::to_string(resolved.totalSteps);
    json += ",\"sample_every\":" + std::to_string(options.sampleEvery);
    json += options.dryRun ? ",\"dry_run\":true}" : ",\"dry_run\":false}";
    return json;
}

}  // namespace terra::headless
