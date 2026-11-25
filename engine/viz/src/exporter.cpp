// 显示配置的导出器（M4-08d）：注册表、参数校验、.terviz 与 .rviz 两个导出器。

#include "terra/viz/exporter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <set>
#include <sstream>
#include <system_error>
#include <utility>

namespace terra::viz {

namespace {

namespace fs = std::filesystem;

// ---- .terviz ----

class TervizExporter final : public Exporter {
public:
    std::string id() const override { return "terviz"; }
    Text name() const override { return {"Terra 显示配置", "Terra display configuration"}; }
    std::string suffix() const override { return kVizSuffix; }
    std::vector<ParameterSpec> parameters(const VizConfig&, const ExportContext&) const override { return {}; }
    std::vector<std::string> notExported(const VizConfig&, const ExportContext&) const override { return {}; }
    ExportResult convert(const VizConfig& config, const ParameterValues&, const ExportContext&, const std::string& outputPath) const override {
        ExportResult r;
        r.ok = true;
        r.text = vizToJson(config, outputPath);
        return r;
    }
};

// ---- .rviz（RViz2 Humble 的 YAML）----

// RViz2 默认插件中有对应显示的消息类型 → 插件类名
const char* rvizDisplayFor(const std::string& type) {
    static const std::pair<const char*, const char*> table[] = {
        {"sensor_msgs/msg/LaserScan", "rviz_default_plugins/LaserScan"},
        {"sensor_msgs/msg/PointCloud2", "rviz_default_plugins/PointCloud2"},
        {"nav_msgs/msg/Odometry", "rviz_default_plugins/Odometry"},
        {"geometry_msgs/msg/PoseStamped", "rviz_default_plugins/Pose"},
        {"nav_msgs/msg/Path", "rviz_default_plugins/Path"},
        {"visualization_msgs/msg/Marker", "rviz_default_plugins/Marker"},
        {"visualization_msgs/msg/MarkerArray", "rviz_default_plugins/MarkerArray"},
        {"sensor_msgs/msg/Image", "rviz_default_plugins/Image"},
        {"sensor_msgs/msg/CompressedImage", "rviz_default_plugins/Image"},
    };
    for (const auto& [t, display] : table) {
        if (type == t) return display;
    }
    return nullptr;
}

// YAML 双引号字符串
std::string quoted(const std::string& s) {
    std::string out = "\"";
    for (const char c : s) {
        if (c == '"' || c == '\\') out += '\\';
        if (c == '\n') {
            out += "\\n";
            continue;
        }
        out += c;
    }
    return out + "\"";
}

std::string number(double v) {
    std::ostringstream o;
    o << v;
    return o.str();
}

std::string typeOf(const ExportContext& ctx, const std::string& topic) {
    const auto it = ctx.topicTypes.find(topic);
    return it == ctx.topicTypes.end() ? std::string{} : it->second;
}

struct RvizPlan {
    bool tf = false;
    std::vector<std::pair<std::string, std::string>> displays;  // 话题 → 插件类名（按出现顺序，去重）
    std::vector<std::string> notExported;
};

RvizPlan planRviz(const VizConfig& config, const ExportContext& ctx) {
    RvizPlan plan;
    std::set<std::string> topics;
    const auto addTopic = [&](const std::string& topic, const std::string& fallbackType) {
        if (topic.empty() || topics.count(topic)) return true;
        std::string type = typeOf(ctx, topic);
        if (type.empty()) type = fallbackType;
        const char* display = rvizDisplayFor(type);
        if (display == nullptr) return false;
        topics.insert(topic);
        plan.displays.emplace_back(topic, display);
        return true;
    };
    for (const auto& w : config.windows) {
        switch (w.type) {
            case WindowType::Tf: plan.tf = true; break;
            case WindowType::Image:
                if (!addTopic(w.topic, "sensor_msgs/msg/Image")) plan.notExported.push_back("图像窗口 " + w.topic + "（类型不是图像）");
                break;
            case WindowType::Message:
                if (!addTopic(w.topic, "")) {
                    const std::string type = typeOf(ctx, w.topic);
                    plan.notExported.push_back("消息窗口 " + w.topic + (type.empty() ? "" : "（" + type + "）") + "：RViz 没有消息查看");
                }
                break;
            case WindowType::Plot: {
                plan.notExported.push_back("曲线“" + (w.title.empty() ? windowId(w) : w.title) + "”：RViz 没有曲线");
                for (const auto& s : w.series) {
                    if (s.source == "ros2") addTopic(s.topic, "");
                }
                break;
            }
            case WindowType::Topics: plan.notExported.push_back("话题表：RViz 没有对应的显示"); break;
            case WindowType::Unknown: plan.notExported.push_back("不认识的窗口类型“" + w.typeName + "”"); break;
        }
        if (w.type == WindowType::Plot) {
            for (const auto& s : w.series) {
                if (s.source == "sim") plan.notExported.push_back("曲线中的仿真来源 " + s.sensor + " " + s.field + "：不经过 ROS2");
            }
        }
    }
    if (config.overlaysJson != "[]" && !config.overlaysJson.empty()) plan.notExported.push_back("视口叠加（overlays）：本版本不导出");
    return plan;
}

class RvizExporter final : public Exporter {
public:
    std::string id() const override { return "rviz"; }
    Text name() const override { return {"RViz2 配置", "RViz2 configuration"}; }
    std::string suffix() const override { return ".rviz"; }

    std::vector<ParameterSpec> parameters(const VizConfig&, const ExportContext& ctx) const override {
        std::vector<ParameterSpec> specs;
        ParameterSpec frame;
        frame.key = "fixedFrame";
        frame.label = {"固定坐标系", "Fixed frame"};
        frame.kind = ParameterSpec::Kind::Text;
        frame.choices = ctx.frames;
        frame.defaultValue = ctx.frames.empty() ? "map" : ctx.frames.front();
        frame.help = {"RViz 的 Fixed Frame；默认取 TF 树的根", "RViz Fixed Frame; defaults to the root of the TF tree"};
        specs.push_back(frame);
        ParameterSpec grid;
        grid.key = "grid";
        grid.label = {"显示网格", "Show grid"};
        grid.kind = ParameterSpec::Kind::Bool;
        grid.defaultValue = "true";
        specs.push_back(grid);
        ParameterSpec distance;
        distance.key = "viewDistance";
        distance.label = {"视距（米）", "View distance (m)"};
        distance.kind = ParameterSpec::Kind::Number;
        distance.defaultValue = "3";
        distance.min = 0.1;
        distance.max = 1000.0;
        specs.push_back(distance);
        ParameterSpec rate;
        rate.key = "frameRate";
        rate.label = {"帧率", "Frame rate"};
        rate.kind = ParameterSpec::Kind::Number;
        rate.defaultValue = "30";
        rate.min = 1.0;
        rate.max = 240.0;
        specs.push_back(rate);
        return specs;
    }

    std::vector<std::string> notExported(const VizConfig& config, const ExportContext& ctx) const override {
        return planRviz(config, ctx).notExported;
    }

    ExportResult convert(const VizConfig& config, const ParameterValues& given, const ExportContext& ctx, const std::string&) const override {
        ExportResult r;
        const auto specs = parameters(config, ctx);
        const auto errors = validateParameters(specs, given);
        if (!errors.empty()) {
            r.error = errors.front();
            return r;
        }
        const ParameterValues v = withDefaults(specs, given);
        const RvizPlan plan = planRviz(config, ctx);
        r.notExported = plan.notExported;
        std::ostringstream y;
        y << "# 由 EAI Terra 导出的 RViz2 配置（显示配置“" << config.name << "”）。用法：rviz2 -d <本文件>";
        y << "（仿真时加 --ros-args -p use_sim_time:=true）\n";
        if (!plan.notExported.empty()) {
            y << "# 没有导出的部分（RViz2 没有对应的显示）：\n";
            for (const auto& n : plan.notExported) y << "#   - " << n << "\n";
        }
        y << "Panels:\n"
             "  - Class: rviz_common/Displays\n"
             "    Name: Displays\n"
             "    Help Height: 0\n"
             "  - Class: rviz_common/Views\n"
             "    Name: Views\n"
             "Visualization Manager:\n"
             "  Class: \"\"\n"
             "  Displays:\n";
        if (v.at("grid") == "true") {
            y << "    - Class: rviz_default_plugins/Grid\n"
                 "      Name: Grid\n"
                 "      Enabled: true\n"
                 "      Reference Frame: <Fixed Frame>\n"
                 "      Cell Size: 1\n"
                 "      Plane Cell Count: 10\n"
                 "      Value: true\n";
        }
        if (plan.tf) {
            y << "    - Class: rviz_default_plugins/TF\n"
                 "      Name: TF\n"
                 "      Enabled: true\n"
                 "      Show Arrows: false\n"
                 "      Show Axes: true\n"
                 "      Show Names: true\n"
                 "      Marker Scale: 0.25\n"
                 "      Update Interval: 0\n"
                 "      Value: true\n";
        }
        for (const auto& [topic, display] : plan.displays) {
            const std::string type = typeOf(ctx, topic);
            const bool compressed = type == "sensor_msgs/msg/CompressedImage";
            std::string topicValue = topic;
            if (compressed && topicValue.size() > 11 && topicValue.compare(topicValue.size() - 11, 11, "/compressed") == 0) {
                topicValue.resize(topicValue.size() - 11);  // image_transport 用基础话题名加传输方式
            }
            const auto be = ctx.bestEffort.find(topic);
            const bool bestEffort = be != ctx.bestEffort.end() && be->second;
            y << "    - Class: " << display << "\n"
              << "      Name: " << quoted(topic) << "\n"
              << "      Enabled: true\n";
            if (compressed) y << "      Transport Hint: compressed\n";
            y << "      Topic:\n"
              << "        Value: " << quoted(topicValue) << "\n"
              << "        Depth: 5\n"
              << "        History Policy: Keep Last\n"
              << "        Reliability Policy: " << (bestEffort ? "Best Effort" : "Reliable") << "\n"
              << "        Durability Policy: Volatile\n"
              << "      Value: true\n";
        }
        y << "  Enabled: true\n"
             "  Global Options:\n"
             "    Background Color: 48; 48; 48\n"
          << "    Fixed Frame: " << quoted(v.at("fixedFrame")) << "\n"
          << "    Frame Rate: " << std::lround(std::strtod(v.at("frameRate").c_str(), nullptr)) << "\n"
          << "  Name: root\n"
             "  Tools:\n"
             "    - Class: rviz_default_plugins/MoveCamera\n"
             "    - Class: rviz_default_plugins/Interact\n"
             "  Value: true\n"
             "  Views:\n"
             "    Current:\n"
             "      Class: rviz_default_plugins/Orbit\n"
          << "      Distance: " << number(std::strtod(v.at("viewDistance").c_str(), nullptr)) << "\n"
          << "      Focal Point:\n"
             "        X: 0\n"
             "        Y: 0\n"
             "        Z: 0.2\n"
             "      Pitch: 0.45\n"
             "      Yaw: 0.7\n"
             "      Target Frame: <Fixed Frame>\n"
             "      Value: Orbit (rviz)\n"
             "    Saved: ~\n"
             "Window Geometry:\n"
             "  Height: 900\n"
             "  Width: 1400\n"
             "  X: 60\n"
             "  Y: 60\n";
        r.text = y.str();
        r.ok = true;
        return r;
    }
};

std::mutex registryMutex;

std::vector<std::unique_ptr<Exporter>>& owned() {
    static std::vector<std::unique_ptr<Exporter>> list;
    return list;
}

std::vector<const Exporter*>& registry() {
    static std::vector<const Exporter*> list = [] {
        owned().push_back(std::make_unique<TervizExporter>());
        owned().push_back(std::make_unique<RvizExporter>());
        std::vector<const Exporter*> l;
        for (const auto& e : owned()) l.push_back(e.get());
        return l;
    }();
    return list;
}

bool parseBool(const std::string& s, bool& out) {
    if (s == "true") out = true;
    else if (s == "false") out = false;
    else return false;
    return true;
}

}  // namespace

const std::vector<const Exporter*>& exporters() {
    std::lock_guard<std::mutex> lock(registryMutex);
    return registry();
}

const Exporter* findExporter(const std::string& id) {
    for (const Exporter* e : exporters()) {
        if (e->id() == id) return e;
    }
    return nullptr;
}

void registerExporter(std::unique_ptr<Exporter> exporter) {
    std::lock_guard<std::mutex> lock(registryMutex);
    auto& list = registry();
    for (auto it = list.begin(); it != list.end(); ++it) {
        if ((*it)->id() == exporter->id()) {  // 同一标识：替换
            list.erase(it);
            break;
        }
    }
    list.push_back(exporter.get());
    owned().push_back(std::move(exporter));
}

ParameterValues withDefaults(const std::vector<ParameterSpec>& specs, const ParameterValues& values) {
    ParameterValues out = values;
    for (const auto& s : specs) {
        if (out.count(s.key) == 0 || (out[s.key].empty() && s.kind != ParameterSpec::Kind::Text)) out[s.key] = s.defaultValue;
    }
    return out;
}

std::vector<std::string> validateParameters(const std::vector<ParameterSpec>& specs, const ParameterValues& values) {
    std::vector<std::string> errors;
    for (const auto& entry : values) {
        const std::string& key = entry.first;
        const std::string& value = entry.second;
        const auto it = std::find_if(specs.begin(), specs.end(), [&key](const ParameterSpec& s) { return s.key == key; });
        if (it == specs.end()) {
            errors.push_back("不认识的参数“" + key + "”");
            continue;
        }
        switch (it->kind) {
            case ParameterSpec::Kind::Bool: {
                bool b = false;
                if (!parseBool(value, b)) errors.push_back(it->label.chinese + "：应为 true 或 false");
                break;
            }
            case ParameterSpec::Kind::Number: {
                char* end = nullptr;
                const double d = std::strtod(value.c_str(), &end);
                if (value.empty() || end == nullptr || *end != '\0' || !std::isfinite(d)) errors.push_back(it->label.chinese + "：应为数值");
                else if (it->max > it->min && (d < it->min || d > it->max)) {
                    errors.push_back(it->label.chinese + "：应在 " + number(it->min) + "～" + number(it->max) + " 之间");
                }
                break;
            }
            case ParameterSpec::Kind::Choice:
                if (std::find(it->choices.begin(), it->choices.end(), value) == it->choices.end()) errors.push_back(it->label.chinese + "：不是可选的值");
                break;
            case ParameterSpec::Kind::Text:
                if (value.empty()) errors.push_back(it->label.chinese + "：不能为空");
                break;
        }
    }
    return errors;
}

ExportResult exportToFile(const Exporter& exporter, const VizConfig& config, const ParameterValues& values, const ExportContext& context,
                          const std::string& outputPath) {
    ExportResult r = exporter.convert(config, values, context, outputPath);
    if (!r.ok) return r;
    std::error_code ec;
    const fs::path target = fs::absolute(outputPath).lexically_normal();
    if (!target.parent_path().empty()) fs::create_directories(target.parent_path(), ec);
    const std::string temp = target.string() + ".partial";
    {
        std::ofstream out(temp, std::ios::binary);
        out << r.text;
        if (!out) {
            r.ok = false;
            r.error = "无法写入：" + temp;
            return r;
        }
    }
    fs::rename(temp, target, ec);
    if (ec) {
        fs::remove(temp, ec);
        r.ok = false;
        r.error = "无法写入：" + target.string();
    }
    return r;
}

}  // namespace terra::viz
