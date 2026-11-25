// 显示配置 .terviz 的读写（M4-08d）。

#include "terra/viz/viz_config.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <system_error>
#include <utility>

#include <nlohmann/json.hpp>

namespace terra::viz {

namespace {

namespace fs = std::filesystem;
using Json = nlohmann::ordered_json;  // 保持字段顺序，保留的内容原样写回

// 读取时的问题收集与类型检查
class Reader {
public:
    std::vector<std::string> errors;
    std::vector<std::string> warnings;

    void error(const std::string& where, const std::string& message) { errors.push_back((where.empty() ? "/" : where) + "：" + message); }
    void warn(const std::string& where, const std::string& message) { warnings.push_back((where.empty() ? "/" : where) + "：" + message); }

    std::optional<std::string> string(const Json& j, const std::string& where) {
        if (!j.is_string()) {
            error(where, "应为字符串");
            return std::nullopt;
        }
        return j.get<std::string>();
    }
    std::optional<double> number(const Json& j, const std::string& where) {
        if (!j.is_number() || !std::isfinite(j.get<double>())) {
            error(where, "应为数值");
            return std::nullopt;
        }
        return j.get<double>();
    }
    std::optional<bool> boolean(const Json& j, const std::string& where) {
        if (!j.is_boolean()) {
            error(where, "应为 true 或 false");
            return std::nullopt;
        }
        return j.get<bool>();
    }
    std::optional<std::array<double, 2>> pair(const Json& j, const std::string& where) {
        if (!j.is_array() || j.size() != 2 || !j[0].is_number() || !j[1].is_number()) {
            error(where, "应为 2 个数值");
            return std::nullopt;
        }
        return std::array<double, 2>{j[0].get<double>(), j[1].get<double>()};
    }
    std::optional<std::string> choice(const Json& j, const std::string& where, std::initializer_list<const char*> allowed) {
        const auto s = string(j, where);
        if (!s) return std::nullopt;
        std::string list;
        for (const char* a : allowed) {
            if (*s == a) return s;
            list += (list.empty() ? "" : "、") + std::string{a};
        }
        error(where, "“" + *s + "”不是可用的值（可用：" + list + "）");
        return std::nullopt;
    }
    // 不认识的字段：警告并收集为对象，原样保留
    std::string extras(const Json& object, const std::string& where, const std::set<std::string>& known) {
        Json extra = Json::object();
        for (auto it = object.begin(); it != object.end(); ++it) {
            if (known.count(it.key())) continue;
            warn(where + "/" + it.key(), "不认识的字段（可能来自更新的版本），已原样保留");
            extra[it.key()] = it.value();
        }
        return extra.empty() ? std::string{} : extra.dump();
    }
};

void readSeries(Reader& r, const Json& j, const std::string& where, PlotSeries& s) {
    if (!j.is_object()) {
        r.error(where, "应为对象");
        return;
    }
    s.extraJson = r.extras(j, where, {"source", "topic", "sensor", "field", "label", "color", "scale", "offset"});
    if (j.contains("source")) {
        if (const auto v = r.choice(j["source"], where + "/source", {"ros2", "sim"})) s.source = *v;
    }
    if (j.contains("topic")) {
        if (const auto v = r.string(j["topic"], where + "/topic")) s.topic = *v;
    }
    if (j.contains("sensor")) {
        if (const auto v = r.string(j["sensor"], where + "/sensor")) s.sensor = *v;
    }
    if (!j.contains("field")) r.error(where, "缺少 field（字段路径，如 “angular_velocity.z”）");
    else if (const auto v = r.string(j["field"], where + "/field")) s.field = *v;
    if (s.source == "ros2" && s.topic.empty()) r.error(where, "来源为 ros2 时需要 topic");
    if (s.source == "sim" && s.sensor.empty()) r.error(where, "来源为 sim 时需要 sensor（传感器全名）");
    if (j.contains("label")) {
        if (const auto v = r.string(j["label"], where + "/label")) s.label = *v;
    }
    if (j.contains("color")) {
        const Json& c = j["color"];
        bool ok = c.is_array() && c.size() == 4;
        for (std::size_t k = 0; ok && k < 4; ++k) ok = c[k].is_number() && c[k].get<double>() >= 0.0 && c[k].get<double>() <= 1.0;
        if (!ok) r.error(where + "/color", "应为 4 个 0～1 的数值（RGBA）");
        else s.color = std::array<float, 4>{c[0].get<float>(), c[1].get<float>(), c[2].get<float>(), c[3].get<float>()};
    }
    if (j.contains("scale")) {
        if (const auto v = r.number(j["scale"], where + "/scale")) s.scale = *v;
    }
    if (j.contains("offset")) {
        if (const auto v = r.number(j["offset"], where + "/offset")) s.offset = *v;
    }
}

void readParams(Reader& r, const Json& p, const std::string& where, VizWindow& w) {
    if (!p.is_object()) {
        r.error(where, "应为对象");
        return;
    }
    switch (w.type) {
        case WindowType::Topics:
            w.extraParams = r.extras(p, where, {"filter"});
            if (p.contains("filter")) {
                if (const auto v = r.string(p["filter"], where + "/filter")) w.filter = *v;
            }
            break;
        case WindowType::Message:
            w.extraParams = r.extras(p, where, {"pageSize"});
            if (p.contains("pageSize")) {
                if (!p["pageSize"].is_number_integer() || p["pageSize"].get<long long>() < 1 || p["pageSize"].get<long long>() > 10000) {
                    r.error(where + "/pageSize", "应为 1～10000 的整数");
                } else {
                    w.pageSize = static_cast<int>(p["pageSize"].get<long long>());
                }
            }
            break;
        case WindowType::Plot:
            w.extraParams = r.extras(p, where, {"seconds", "timeBase", "yRange", "series"});
            if (p.contains("seconds")) {
                if (const auto v = r.number(p["seconds"], where + "/seconds")) {
                    if (*v <= 0.0 || *v > 3600.0) r.error(where + "/seconds", "应在 0～3600 秒之间");
                    else w.seconds = *v;
                }
            }
            if (p.contains("timeBase")) {
                if (const auto v = r.choice(p["timeBase"], where + "/timeBase", {"receive", "header"})) w.timeBase = *v;
            }
            if (p.contains("yRange") && !(p["yRange"].is_string() && p["yRange"].get<std::string>() == "auto")) {
                if (const auto v = r.pair(p["yRange"], where + "/yRange")) {
                    if ((*v)[0] >= (*v)[1]) r.error(where + "/yRange", "下限应小于上限（或写 “auto”）");
                    else w.yRange = *v;
                }
            }
            if (p.contains("series")) {
                if (!p["series"].is_array()) {
                    r.error(where + "/series", "应为数组");
                } else {
                    for (std::size_t i = 0; i < p["series"].size(); ++i) {
                        PlotSeries s;
                        readSeries(r, p["series"][i], where + "/series/" + std::to_string(i), s);
                        w.series.push_back(std::move(s));
                    }
                }
            }
            break;
        case WindowType::Image:
            w.extraParams = r.extras(p, where, {"depthRange", "colormap", "fit"});
            if (p.contains("depthRange")) {
                if (const auto v = r.pair(p["depthRange"], where + "/depthRange")) {
                    if ((*v)[0] < 0.0 || (*v)[0] >= (*v)[1]) r.error(where + "/depthRange", "应为 0 ≤ 下限 < 上限（米）");
                    else w.depthRange = *v;
                }
            }
            if (p.contains("colormap")) {
                if (const auto v = r.choice(p["colormap"], where + "/colormap", {"turbo", "gray"})) w.colormap = *v;
            }
            if (p.contains("fit")) {
                if (const auto v = r.choice(p["fit"], where + "/fit", {"fit", "actual"})) w.fit = *v;
            }
            break;
        case WindowType::Tf:
            w.extraParams = r.extras(p, where, {"root"});
            if (p.contains("root")) {
                if (const auto v = r.string(p["root"], where + "/root")) w.tfRoot = *v;
            }
            break;
        case WindowType::Unknown: break;
    }
}

void readWindow(Reader& r, const Json& j, const std::string& where, VizWindow& w) {
    if (!j.is_object()) {
        r.error(where, "应为对象");
        return;
    }
    if (!j.contains("type")) {
        r.error(where, "缺少 type（topics、message、plot、image、tf）");
        return;
    }
    const auto typeName = r.string(j["type"], where + "/type");
    if (!typeName) return;
    w.typeName = *typeName;
    const auto type = windowTypeFromName(*typeName);
    if (!type) {
        // 更新的版本中的窗口类型：原样保留，不显示
        r.warn(where + "/type", "不认识的窗口类型“" + *typeName + "”（可能来自更新的版本），不显示，保存时原样保留");
        w.type = WindowType::Unknown;
        w.rawJson = j.dump();
        return;
    }
    w.type = *type;
    w.extraFields = r.extras(j, where, {"type", "id", "topic", "title", "params", "placement"});
    if (j.contains("id")) {
        if (const auto v = r.string(j["id"], where + "/id")) w.id = *v;
    }
    if (j.contains("topic")) {
        if (const auto v = r.string(j["topic"], where + "/topic")) w.topic = *v;
    }
    if ((w.type == WindowType::Message || w.type == WindowType::Image) && w.topic.empty()) r.error(where, "需要 topic（话题名）");
    if (j.contains("title")) {
        if (const auto v = r.string(j["title"], where + "/title")) w.title = *v;
    }
    if (j.contains("params")) readParams(r, j["params"], where + "/params", w);
    if (j.contains("placement")) {
        const Json& p = j["placement"];
        const std::string pw = where + "/placement";
        if (!p.is_object()) {
            r.error(pw, "应为对象");
        } else {
            Placement placement;
            if (p.contains("position")) {
                if (const auto v = r.pair(p["position"], pw + "/position")) placement.position = *v;
            }
            if (p.contains("size")) {
                if (const auto v = r.pair(p["size"], pw + "/size")) {
                    if ((*v)[0] <= 0.0 || (*v)[1] <= 0.0) r.error(pw + "/size", "应为正数（相对主窗口的比例）");
                    else placement.size = *v;
                }
            }
            if (p.contains("docked")) {
                if (const auto v = r.boolean(p["docked"], pw + "/docked")) placement.docked = *v;
            }
            w.placement = placement;
        }
    }
}

std::string relativeTo(const std::string& target, const fs::path& baseDir) {
    if (target.empty()) return {};
    const fs::path rel = fs::path{target}.lexically_relative(baseDir);
    return rel.empty() ? target : rel.generic_string();
}

// 把保留的 JSON 对象文本合并进 out
void mergeExtra(Json& out, const std::string& extra) {
    if (extra.empty()) return;
    const Json e = Json::parse(extra, nullptr, false);
    if (!e.is_object()) return;
    for (auto it = e.begin(); it != e.end(); ++it) {
        if (!out.contains(it.key())) out[it.key()] = it.value();
    }
}

Json pairJson(const std::array<double, 2>& v) { return Json::array({v[0], v[1]}); }

}  // namespace

const char* windowTypeName(WindowType type) {
    switch (type) {
        case WindowType::Topics: return "topics";
        case WindowType::Message: return "message";
        case WindowType::Plot: return "plot";
        case WindowType::Image: return "image";
        case WindowType::Tf: return "tf";
        case WindowType::Unknown: return "";
    }
    return "";
}

std::optional<WindowType> windowTypeFromName(const std::string& name) {
    for (const WindowType t : {WindowType::Topics, WindowType::Message, WindowType::Plot, WindowType::Image, WindowType::Tf}) {
        if (name == windowTypeName(t)) return t;
    }
    return std::nullopt;
}

std::string windowId(const VizWindow& w) {
    if (!w.id.empty()) return w.id;
    const std::string type = w.type == WindowType::Unknown ? w.typeName : std::string{windowTypeName(w.type)};
    if (w.type == WindowType::Plot) return type + ":" + w.title;
    return w.topic.empty() ? type : type + ":" + w.topic;
}

VizReadResult parseViz(const std::string& text, const std::string& path) {
    VizReadResult result;
    Reader r;
    const Json root = Json::parse(text, nullptr, false);
    if (root.is_discarded() || !root.is_object()) {
        result.errors.push_back("不是有效的 JSON 对象：" + path);
        return result;
    }
    VizConfig c;
    c.extraJson = r.extras(root, "", {"format", "version", "name", "scene", "windows", "overlays", "extensions"});
    if (!root.contains("format") || !root["format"].is_string() || root["format"].get<std::string>() != kVizFormat) {
        r.error("/format", "应为 “terra.viz”（Terra 显示配置）");
    }
    if (!root.contains("version") || !root["version"].is_number_integer()) {
        r.error("/version", "缺少版本号");
    } else if (root["version"].get<long long>() > kVizVersion) {
        r.error("/version", "显示配置的版本 " + std::to_string(root["version"].get<long long>()) + " 高于本程序支持的版本 " +
                                std::to_string(kVizVersion) + "，请更新程序");
    } else if (root["version"].get<long long>() < 1) {
        r.error("/version", "版本号无效");
    }
    if (root.contains("name")) {
        if (const auto v = r.string(root["name"], "/name")) c.name = *v;
    }
    if (root.contains("scene")) {
        if (const auto v = r.string(root["scene"], "/scene"); v && !v->empty()) {
            const fs::path p{*v};
            c.scene = (p.is_absolute() ? p : fs::path{path}.parent_path() / p).lexically_normal().string();
        }
    }
    if (root.contains("windows")) {
        if (!root["windows"].is_array()) {
            r.error("/windows", "应为数组");
        } else {
            std::set<std::string> ids;
            for (std::size_t i = 0; i < root["windows"].size(); ++i) {
                VizWindow w;
                readWindow(r, root["windows"][i], "/windows/" + std::to_string(i), w);
                if (w.type != WindowType::Unknown && !ids.insert(windowId(w)).second) {
                    r.error("/windows/" + std::to_string(i), "窗口标识“" + windowId(w) + "”重复（同一话题的同类窗口请用不同的 id）");
                }
                c.windows.push_back(std::move(w));
            }
        }
    }
    if (root.contains("overlays")) {
        if (!root["overlays"].is_array()) r.error("/overlays", "应为数组（预留：视口叠加）");
        else c.overlaysJson = root["overlays"].dump();
    }
    if (root.contains("extensions")) {
        if (!root["extensions"].is_object()) r.error("/extensions", "应为对象（预留：第三方扩展）");
        else c.extensionsJson = root["extensions"].dump();
    }
    result.errors = std::move(r.errors);
    result.warnings = std::move(r.warnings);
    if (result.errors.empty()) result.config = std::move(c);
    return result;
}

VizReadResult readViz(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        VizReadResult r;
        r.errors.push_back("无法读取：" + path);
        return r;
    }
    const std::string text{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    return parseViz(text, fs::absolute(path).lexically_normal().string());
}

std::string vizToJson(const VizConfig& c, const std::string& path) {
    Json root = Json::object();
    root["format"] = kVizFormat;
    root["version"] = kVizVersion;
    root["name"] = c.name;
    root["scene"] = relativeTo(c.scene, fs::absolute(path).lexically_normal().parent_path());
    Json windows = Json::array();
    for (const auto& w : c.windows) {
        if (w.type == WindowType::Unknown) {
            const Json raw = Json::parse(w.rawJson, nullptr, false);
            if (!raw.is_discarded()) windows.push_back(raw);
            continue;
        }
        Json j = Json::object();
        j["type"] = windowTypeName(w.type);
        if (!w.id.empty()) j["id"] = w.id;
        if (!w.topic.empty()) j["topic"] = w.topic;
        if (!w.title.empty()) j["title"] = w.title;
        Json p = Json::object();
        switch (w.type) {
            case WindowType::Topics:
                if (!w.filter.empty()) p["filter"] = w.filter;
                break;
            case WindowType::Message: p["pageSize"] = w.pageSize; break;
            case WindowType::Plot: {
                p["seconds"] = w.seconds;
                p["timeBase"] = w.timeBase;
                if (w.yRange) p["yRange"] = pairJson(*w.yRange);
                else p["yRange"] = "auto";
                Json series = Json::array();
                for (const auto& s : w.series) {
                    Json js = Json::object();
                    js["source"] = s.source;
                    if (s.source == "sim") js["sensor"] = s.sensor;
                    else js["topic"] = s.topic;
                    js["field"] = s.field;
                    if (!s.label.empty()) js["label"] = s.label;
                    if (s.color) js["color"] = Json::array({(*s.color)[0], (*s.color)[1], (*s.color)[2], (*s.color)[3]});
                    if (s.scale != 1.0) js["scale"] = s.scale;
                    if (s.offset != 0.0) js["offset"] = s.offset;
                    mergeExtra(js, s.extraJson);
                    series.push_back(std::move(js));
                }
                p["series"] = std::move(series);
                break;
            }
            case WindowType::Image:
                p["depthRange"] = pairJson(w.depthRange);
                p["colormap"] = w.colormap;
                p["fit"] = w.fit;
                break;
            case WindowType::Tf:
                if (!w.tfRoot.empty()) p["root"] = w.tfRoot;
                break;
            case WindowType::Unknown: break;
        }
        mergeExtra(p, w.extraParams);
        if (!p.empty()) j["params"] = std::move(p);
        if (w.placement) {
            j["placement"] = {{"position", pairJson(w.placement->position)}, {"size", pairJson(w.placement->size)}, {"docked", w.placement->docked}};
        }
        mergeExtra(j, w.extraFields);
        windows.push_back(std::move(j));
    }
    root["windows"] = std::move(windows);
    const Json overlays = Json::parse(c.overlaysJson, nullptr, false);
    root["overlays"] = overlays.is_array() ? overlays : Json::array();
    const Json extensions = Json::parse(c.extensionsJson, nullptr, false);
    root["extensions"] = extensions.is_object() ? extensions : Json::object();
    mergeExtra(root, c.extraJson);
    return root.dump(2) + "\n";
}

std::optional<std::string> saveViz(const VizConfig& config, const std::string& path) {
    std::error_code ec;
    const fs::path target = fs::absolute(path).lexically_normal();
    if (!target.parent_path().empty()) fs::create_directories(target.parent_path(), ec);
    const std::string temp = target.string() + ".partial";
    {
        std::ofstream out(temp, std::ios::binary);
        out << vizToJson(config, target.string());
        if (!out) return "无法写入：" + temp;
    }
    fs::rename(temp, target, ec);
    if (ec) {
        fs::remove(temp, ec);
        return "无法写入：" + target.string();
    }
    return std::nullopt;
}

}  // namespace terra::viz
