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
    bool seenSampleEvery = false;
    RunOptions& options = result.options;

    for (int i = 1; i < argc; ++i) {
        std::string_view arg{argv[i]};

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

        const bool takesValue = name == "--model" || name == "--steps" || name == "--duration" ||
                                name == "--sample-every";
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

        if (name == "--model") {
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

    if (!seenModel) {
        result.error = "缺少 --model";
    } else if (options.steps && options.durationSeconds) {
        result.error = "--steps 与 --duration 不能同时使用";
    } else if (!options.steps && !options.durationSeconds) {
        result.error = "需要 --steps 或 --duration 之一";
    }
    return result;
}

std::string usageText() {
    return "用法:\n"
           "  terra_headless --model <MJCF> (--steps N | --duration 秒) [--sample-every N] [--dry-run]\n"
           "  terra_headless --help | --version\n"
           "\n"
           "参数:\n"
           "  --model <路径>       MJCF 模型文件\n"
           "  --steps N            运行 N 个固定物理步（正整数）\n"
           "  --duration 秒        运行到指定仿真时长（正数；与 --steps 二选一）\n"
           "  --sample-every N     每 N 步输出一次状态（默认 1；第 0 步与最后一步总是输出）\n"
           "  --dry-run            校验参数并加载模型，输出解析后的配置，不运行仿真\n"
           "  -h, --help           显示本帮助\n"
           "      --version        显示版本与构建信息\n"
           "\n"
           "输出: stdout 为 JSON Lines（每行一个对象，带 \"type\" 字段）；诊断信息写 stderr。\n"
           "退出码: 0 成功；1 内部错误；2 参数错误；3 后端不可用或功能未接入；4 模型错误；5 仿真错误。\n"
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

std::string modelToJson(const terra::data::ModelDescription& model) {
    const auto& st = model.settings;
    std::string json = "{\"type\":\"model\",\"timestep\":" + formatNumber(model.timestep) +
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
    json += "]}";
    return json;
}

std::string stateToJson(std::uint64_t runEpoch, std::uint64_t step, double time,
                        const std::vector<EntitySample>& entities) {
    std::string json = "{\"type\":\"state\",\"run_epoch\":" + std::to_string(runEpoch) +
                       ",\"step\":" + std::to_string(step) + ",\"time\":" + formatNumber(time) +
                       ",\"entities\":[";
    for (std::size_t i = 0; i < entities.size(); ++i) {
        const auto& e = entities[i];
        const auto& q = e.pose.orientation;
        json += (i ? "," : "");
        json += "{\"id\":" + std::to_string(e.id.value) + ",\"position\":" + vec3Json(e.pose.position) +
                ",\"orientation\":[" + formatNumber(q.w) + "," + formatNumber(q.x) + "," + formatNumber(q.y) + "," +
                formatNumber(q.z) + "]}";
    }
    json += "]}";
    return json;
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
