// 场景文件读取与校验（M2-05，M2-01 设计第 3 节）。nlohmann/json 只在本文件中使用，不进入公共头。

#include <cmath>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <set>

#include <nlohmann/json.hpp>

#include "terra/scene/pose_math.hpp"
#include "terra/scene/scene.hpp"

namespace terra::scene {

namespace {

namespace fs = std::filesystem;
namespace td = terra::data;
using Json = nlohmann::json;
using Severity = SceneIssue::Severity;

constexpr int kSupportedVersion = 1;

std::string kindOf(const Json& v) {
    if (v.is_object()) return "对象";
    if (v.is_array()) return "数组";
    if (v.is_string()) return "字符串";
    if (v.is_boolean()) return "布尔值";
    if (v.is_number()) return "数值";
    return "null";
}

class Reader {
public:
    explicit Reader(std::string directory) : directory_(std::move(directory)) {}

    std::vector<SceneIssue> issues;

    void error(const std::string& where, std::string message, const std::string& instance = {}) {
        issues.push_back({Severity::Error, instance, where, std::move(message)});
    }
    void warning(const std::string& where, std::string message, const std::string& instance = {}) {
        issues.push_back({Severity::Warning, instance, where, std::move(message)});
    }
    bool hasErrors() const {
        for (const auto& i : issues) {
            if (i.severity == Severity::Error) return true;
        }
        return false;
    }

    // 对象中只允许列出的字段
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
                error(where + "/" + it.key(), "未知字段“" + it.key() + "”（允许的字段：" + list + "）");
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

    std::optional<long long> integer(const Json& v, const std::string& where) {
        if (!v.is_number_integer()) {
            error(where, "应为整数，实际为" + kindOf(v));
            return std::nullopt;
        }
        return v.get<long long>();
    }

    std::optional<bool> boolean(const Json& v, const std::string& where) {
        if (!v.is_boolean()) {
            error(where, "应为 true 或 false，实际为" + kindOf(v));
            return std::nullopt;
        }
        return v.get<bool>();
    }

    bool numbers(const Json& v, const std::string& where, std::size_t count, double* out) {
        if (!v.is_array() || v.size() != count) {
            error(where, "应为 " + std::to_string(count) + " 个数值的数组");
            return false;
        }
        bool ok = true;
        for (std::size_t i = 0; i < count; ++i) {
            const auto d = number(v[i], where + "/" + std::to_string(i));
            if (d) out[i] = *d;
            ok = ok && d.has_value();
        }
        return ok;
    }

    std::optional<td::Vec3> vec3(const Json& v, const std::string& where) {
        double a[3] = {0, 0, 0};
        if (!numbers(v, where, 3, a)) return std::nullopt;
        return td::Vec3{a[0], a[1], a[2]};
    }

    // {"quat": [w, x, y, z]} 或 {"rpyDeg": [滚转, 俯仰, 偏航]}，二选一
    std::optional<td::Quaternion> rotation(const Json& v, const std::string& where) {
        if (!object(v, where, {"quat", "rpyDeg"})) return std::nullopt;
        const bool hasQuat = v.contains("quat");
        const bool hasRpy = v.contains("rpyDeg");
        if (hasQuat == hasRpy) {
            error(where, "旋转应写 quat 或 rpyDeg 其中之一");
            return std::nullopt;
        }
        if (hasRpy) {
            double a[3] = {0, 0, 0};
            if (!numbers(v["rpyDeg"], where + "/rpyDeg", 3, a)) return std::nullopt;
            return fromRpyDegrees(a[0], a[1], a[2]);
        }
        double q[4] = {1, 0, 0, 0};
        if (!numbers(v["quat"], where + "/quat", 4, q)) return std::nullopt;
        const td::Quaternion raw{q[0], q[1], q[2], q[3]};
        const double n = norm(raw);
        if (!(n > 1e-9)) {
            error(where + "/quat", "四元数长度为 0");
            return std::nullopt;
        }
        if (std::fabs(n - 1.0) > 1e-6) warning(where + "/quat", "四元数不是单位长度（" + std::to_string(n) + "），已归一化");
        return normalized(raw);
    }

    std::string path(const std::string& written) const {
        const fs::path p{written};
        return (p.is_absolute() ? p : fs::path{directory_} / p).lexically_normal().string();
    }

    void pose(const Json& item, const std::string& where, td::Pose& out) {
        if (item.contains("position")) {
            if (const auto v = vec3(item["position"], where + "/position")) out.position = *v;
        }
        if (item.contains("rotation")) {
            if (const auto q = rotation(item["rotation"], where + "/rotation")) out.orientation = *q;
        }
    }

private:
    std::string directory_;
};

void readPhysics(Reader& r, const Json& v, PhysicsOverrides& out) {
    const std::string where = "/physics";
    if (!r.object(v, where, {"timestep", "gravity", "integrator", "solver", "iterations", "tolerance"})) return;
    if (v.contains("timestep")) {
        if (const auto d = r.number(v["timestep"], where + "/timestep")) {
            if (*d > 0) out.timestep = *d;
            else r.error(where + "/timestep", "步长必须大于 0");
        }
    }
    if (v.contains("gravity")) out.gravity = r.vec3(v["gravity"], where + "/gravity");
    if (v.contains("integrator")) {
        if (const auto s = r.string(v["integrator"], where + "/integrator")) {
            if (*s == "Euler" || *s == "RK4" || *s == "implicit" || *s == "implicitfast") out.integrator = *s;
            else r.error(where + "/integrator", "未知的积分器“" + *s + "”（可用：Euler、RK4、implicit、implicitfast）");
        }
    }
    if (v.contains("solver")) {
        if (const auto s = r.string(v["solver"], where + "/solver")) {
            if (*s == "PGS" || *s == "CG" || *s == "Newton") out.solver = *s;
            else r.error(where + "/solver", "未知的求解器“" + *s + "”（可用：PGS、CG、Newton）");
        }
    }
    if (v.contains("iterations")) {
        if (const auto n = r.integer(v["iterations"], where + "/iterations")) {
            if (*n > 0 && *n <= 1000000) out.iterations = static_cast<int>(*n);
            else r.error(where + "/iterations", "迭代次数应在 1 到 1000000 之间");
        }
    }
    if (v.contains("tolerance")) {
        if (const auto d = r.number(v["tolerance"], where + "/tolerance")) {
            if (*d >= 0) out.tolerance = *d;
            else r.error(where + "/tolerance", "容差不能为负");
        }
    }
}

void readModels(Reader& r, const Json& v, std::vector<SceneModel>& out) {
    if (!v.is_array() || v.empty()) {
        r.error("/models", "models 应为至少有一个模型实例的数组");
        return;
    }
    for (std::size_t i = 0; i < v.size(); ++i) {
        const std::string where = "/models/" + std::to_string(i);
        const Json& item = v[i];
        if (!r.object(item, where, {"name", "mjcf", "position", "rotation", "fixedBase"})) continue;
        SceneModel m;
        m.where = where;
        if (!item.contains("name")) r.error(where, "缺少 name（实例名）");
        else if (const auto s = r.string(item["name"], where + "/name")) m.name = *s;
        if (!item.contains("mjcf")) r.error(where, "缺少 mjcf（MJCF 文件路径）", m.name);
        else if (const auto s = r.string(item["mjcf"], where + "/mjcf")) m.mjcf = s->empty() ? std::string{} : r.path(*s);
        r.pose(item, where, m.placement);
        if (item.contains("fixedBase")) {
            if (const auto b = r.boolean(item["fixedBase"], where + "/fixedBase")) m.fixedBase = *b;
        }
        out.push_back(std::move(m));
    }
}

void readControllers(Reader& r, const Json& v, std::vector<PdControllerSpec>& out) {
    if (!v.is_array()) {
        r.error("/controllers", "controllers 应为数组");
        return;
    }
    for (std::size_t i = 0; i < v.size(); ++i) {
        const std::string where = "/controllers/" + std::to_string(i);
        const Json& item = v[i];
        if (!r.object(item, where, {"type", "actuators", "kp", "kd"})) continue;
        PdControllerSpec c;
        c.where = where;
        if (!item.contains("type")) {
            r.error(where, "缺少 type（目前只支持 “pd”）");
        } else if (const auto s = r.string(item["type"], where + "/type"); s && *s != "pd") {
            r.error(where + "/type", "未知的控制层类型“" + *s + "”（目前只支持 “pd”）");
        }
        if (!item.contains("actuators")) {
            r.error(where, "缺少 actuators（执行器全名的数组，或 “<实例名>/*”）");
        } else if (!item["actuators"].is_array() || item["actuators"].empty()) {
            r.error(where + "/actuators", "应为至少有一个执行器名的数组");
        } else {
            for (std::size_t k = 0; k < item["actuators"].size(); ++k) {
                if (const auto s = r.string(item["actuators"][k], where + "/actuators/" + std::to_string(k))) {
                    if (s->empty()) r.error(where + "/actuators/" + std::to_string(k), "执行器名为空");
                    else c.actuators.push_back(*s);
                }
            }
        }
        if (!item.contains("kp")) {
            r.error(where, "缺少 kp（位置增益，N·m/rad 或 N/m）");
        } else if (const auto d = r.number(item["kp"], where + "/kp")) {
            if (*d <= 0) r.error(where + "/kp", "kp 应大于 0");
            c.kp = *d;
        }
        if (item.contains("kd")) {
            if (const auto d = r.number(item["kd"], where + "/kd")) {
                if (*d < 0) r.error(where + "/kd", "kd 不能为负");
                c.kd = *d;
            }
        }
        out.push_back(std::move(c));
    }
}

// 一个 IMU（M4-02）
void readImu(Reader& r, const Json& item, const std::string& where, SensorSpec& s) {
    if (!r.object(item, where, {"type", "name", "site", "entity", "position", "rotation", "rateHz"})) return;
    const bool hasSite = item.contains("site");
    const bool hasEntity = item.contains("entity");
    if (hasSite == hasEntity) {
        r.error(where, "安装位置应写 site（已有的安装点）或 entity（实体）其中之一");
        return;
    }
    if (hasSite) {
        if (const auto n = r.string(item["site"], where + "/site")) s.site = *n;
        if (item.contains("position") || item.contains("rotation")) {
            r.error(where, "装在已有的安装点上时不能再写 position / rotation（位姿由模型中的安装点决定）");
        }
        if (!s.name.empty() && s.name != s.site) {
            r.error(where + "/name", "装在已有安装点上的 IMU 名字就是安装点名“" + s.site + "”；省略 name 即可");
        }
        s.name = s.site;
    } else {
        if (const auto n = r.string(item["entity"], where + "/entity")) s.entity = *n;
        r.pose(item, where, s.pose);
        if (s.name.empty() && !item.contains("name")) r.error(where, "装在实体上的 IMU 需要 name（带实例前缀，如 “go2/imu_head”）");
    }
}

// 一个相机（M4-03）
void readCamera(Reader& r, const Json& item, const std::string& where, SensorSpec& s) {
    if (!r.object(item, where, {"type", "name", "camera", "entity", "position", "rotation", "width", "height", "fovyDeg", "near",
                                "far", "rateHz", "rgb", "depth", "displayRotationDeg"})) {
        return;
    }
    const bool hasCamera = item.contains("camera");
    const bool hasEntity = item.contains("entity");
    if (hasCamera == hasEntity) {
        r.error(where, "相机应写 camera（MJCF 中的相机）或 entity（装在实体上）其中之一");
        return;
    }
    if (hasCamera) {
        if (const auto n = r.string(item["camera"], where + "/camera")) s.camera = *n;
        if (item.contains("position") || item.contains("rotation")) {
            r.error(where, "使用 MJCF 中的相机时不能再写 position / rotation（位姿由模型中的相机决定）");
        }
        if (s.name.empty()) s.name = s.camera;
    } else {
        if (const auto n = r.string(item["entity"], where + "/entity")) s.entity = *n;
        r.pose(item, where, s.pose);
        if (s.name.empty() && !item.contains("name")) r.error(where, "装在实体上的相机需要 name（带实例前缀，如 “g1/head_camera”）");
    }
    const std::pair<const char*, std::optional<int>*> sizes[] = {{"width", &s.width}, {"height", &s.height}};
    for (const auto& [key, target] : sizes) {
        if (!item.contains(key)) continue;
        if (const auto n = r.integer(item[key], where + "/" + key)) {
            if (*n >= 1 && *n <= 8192) *target = static_cast<int>(*n);
            else r.error(where + "/" + key, "像素数应在 1 到 8192 之间");
        }
    }
    if (item.contains("fovyDeg")) {
        if (const auto d = r.number(item["fovyDeg"], where + "/fovyDeg")) {
            if (*d > 0 && *d < 180) s.fovyDegrees = *d;
            else r.error(where + "/fovyDeg", "竖直视场角应在 0° 到 180° 之间（不含端点）");
        }
    }
    if (item.contains("near")) {
        if (const auto d = r.number(item["near"], where + "/near")) {
            if (*d > 0) s.nearClip = *d;
            else r.error(where + "/near", "近平面距离必须大于 0");
        }
    }
    if (item.contains("far")) {
        if (const auto d = r.number(item["far"], where + "/far")) s.farClip = *d;
    }
    if (!(s.farClip > s.nearClip)) r.error(where + "/far", "远平面距离必须大于近平面距离");
    if (item.contains("rgb")) {
        if (const auto b = r.boolean(item["rgb"], where + "/rgb")) s.rgb = *b;
    }
    if (item.contains("depth")) {
        if (const auto b = r.boolean(item["depth"], where + "/depth")) s.depth = *b;
    }
    if (!s.rgb && !s.depth) r.error(where, "rgb 与 depth 不能都关闭");
    if (item.contains("displayRotationDeg")) {
        if (const auto n = r.integer(item["displayRotationDeg"], where + "/displayRotationDeg")) {
            if (*n == 0 || *n == 90 || *n == 180 || *n == 270) s.displayRotationDegrees = static_cast<int>(*n);
            else r.error(where + "/displayRotationDeg", "显示旋转应为 0、90、180 或 270（逆时针，度）");
        }
    }
}

// 一个 2D 激光雷达（M4-03b）
void readLidar(Reader& r, const Json& item, const std::string& where, SensorSpec& s) {
    if (!r.object(item, where, {"type", "name", "entity", "position", "rotation", "angleMinDeg", "angleMaxDeg", "samples", "rangeMin",
                                "rangeMax", "rateHz", "hits"})) {
        return;
    }
    if (!item.contains("entity")) {
        r.error(where, "2D 激光雷达需要 entity（安装的实体，“world” 表示装在世界上）");
        return;
    }
    if (const auto n = r.string(item["entity"], where + "/entity")) s.entity = *n;
    r.pose(item, where, s.pose);
    if (s.name.empty() && !item.contains("name")) r.error(where, "2D 激光雷达需要 name（带实例前缀，如 “turtlebot4/scan”）");
    for (const auto& [key, target] : {std::pair<const char*, double*>{"angleMinDeg", &s.angleMinDegrees}, std::pair<const char*, double*>{"angleMaxDeg", &s.angleMaxDegrees},
                                      std::pair<const char*, double*>{"rangeMin", &s.rangeMin}, std::pair<const char*, double*>{"rangeMax", &s.rangeMax}}) {
        if (item.contains(key)) {
            if (const auto d = r.number(item[key], where + "/" + key)) *target = *d;
        }
    }
    if (!(s.angleMinDegrees >= -360.0 && s.angleMaxDegrees <= 360.0 && s.angleMaxDegrees > s.angleMinDegrees)) {
        r.error(where + "/angleMaxDeg", "角度范围应满足 −360° ≤ angleMinDeg < angleMaxDeg ≤ 360°");
    }
    if (!(s.rangeMin >= 0.0)) r.error(where + "/rangeMin", "最小量程不能为负");
    if (!(s.rangeMax > s.rangeMin)) r.error(where + "/rangeMax", "最大量程必须大于最小量程");
    if (item.contains("samples")) {
        if (const auto n = r.integer(item["samples"], where + "/samples")) {
            if (*n >= 1 && *n <= 100000) s.samples = static_cast<int>(*n);
            else r.error(where + "/samples", "采样数应在 1 到 100000 之间");
        }
    }
    if (item.contains("hits")) {
        if (const auto h = r.string(item["hits"], where + "/hits")) {
            if (*h == "visual") s.hitVisual = true;
            else if (*h != "collision") r.error(where + "/hits", "hits 应为 “collision”（参与碰撞的几何体）或 “visual”（显示的几何体）");
        }
    }
}

void readSensors(Reader& r, const Json& v, std::vector<SensorSpec>& out) {
    if (!v.is_array()) {
        r.error("/sensors", "sensors 应为数组");
        return;
    }
    std::set<std::string> names;
    for (std::size_t i = 0; i < v.size(); ++i) {
        const std::string where = "/sensors/" + std::to_string(i);
        const Json& item = v[i];
        if (!item.is_object()) {
            r.error(where, "应为对象，实际为" + kindOf(item));
            continue;
        }
        SensorSpec s;
        s.where = where;
        if (!item.contains("type")) {
            r.error(where, "缺少 type（imu、camera 或 lidar2d）");
            continue;
        }
        const auto type = r.string(item["type"], where + "/type");
        if (!type) continue;
        if (*type != "imu" && *type != "camera" && *type != "lidar2d") {
            r.error(where + "/type", "未知的传感器类型“" + *type + "”（可用：imu、camera、lidar2d）");
            continue;
        }
        s.type = *type;
        if (item.contains("name")) {
            if (const auto n = r.string(item["name"], where + "/name")) {
                if (n->empty()) r.error(where + "/name", "名字为空");
                s.name = *n;
            }
        }
        if (s.type == "imu") readImu(r, item, where, s);
        else if (s.type == "camera") readCamera(r, item, where, s);
        else readLidar(r, item, where, s);
        if (item.contains("rateHz")) {
            if (const auto d = r.number(item["rateHz"], where + "/rateHz")) {
                if (*d > 0) s.rateHz = *d;
                else r.error(where + "/rateHz", "频率必须大于 0");
            }
        }
        if (!s.name.empty() && !names.insert(s.name).second) r.error(where + "/name", "传感器名“" + s.name + "”重复");
        out.push_back(std::move(s));
    }
}

void readVisuals(Reader& r, const Json& v, std::vector<VisualBindingSpec>& out) {
    if (!v.is_array()) {
        r.error("/visuals", "visuals 应为数组");
        return;
    }
    for (std::size_t i = 0; i < v.size(); ++i) {
        const std::string where = "/visuals/" + std::to_string(i);
        const Json& item = v[i];
        if (!r.object(item, where, {"entity", "gltf", "node", "position", "rotation", "scale", "frame", "showGeoms"})) continue;
        VisualBindingSpec b;
        b.where = where;
        if (!item.contains("entity")) r.error(where, "缺少 entity（实体全名，如 “cart_a/chassis”，或 “world”）");
        else if (const auto s = r.string(item["entity"], where + "/entity")) b.entity = *s;
        if (!item.contains("gltf")) r.error(where, "缺少 gltf（glTF 文件路径）");
        else if (const auto s = r.string(item["gltf"], where + "/gltf")) {
            if (s->empty()) r.error(where + "/gltf", "glTF 路径为空");
            else b.gltf = r.path(*s);
        }
        if (item.contains("node")) {
            if (const auto s = r.string(item["node"], where + "/node")) {
                if (s->empty()) r.error(where + "/node", "节点名为空（取整个文件时省略 node）");
                b.node = *s;
            }
        }
        r.pose(item, where, b.local);
        if (item.contains("scale")) {
            if (const auto d = r.number(item["scale"], where + "/scale")) {
                if (*d > 0) b.scale = *d;
                else r.error(where + "/scale", "缩放必须大于 0");
            }
        }
        if (item.contains("frame")) {
            if (const auto s = r.string(item["frame"], where + "/frame")) {
                if (*s == "node") b.frame = VisualFrame::Node;
                else if (*s == "assembly") b.frame = VisualFrame::Assembly;
                else r.error(where + "/frame", "frame 应为 “node” 或 “assembly”");
            }
        }
        if (item.contains("showGeoms")) {
            if (const auto f = r.boolean(item["showGeoms"], where + "/showGeoms")) b.showGeoms = *f;
        }
        out.push_back(std::move(b));
    }
}

void readView(Reader& r, const Json& v, ViewSpec& out) {
    if (!r.object(v, "/view", {"environment", "camera"})) return;
    if (v.contains("environment")) out.environment = r.string(v["environment"], "/view/environment");
    if (v.contains("camera")) {
        const Json& c = v["camera"];
        if (r.object(c, "/view/camera", {"position", "target"})) {
            if (c.contains("position")) out.cameraPosition = r.vec3(c["position"], "/view/camera/position");
            if (c.contains("target")) out.cameraTarget = r.vec3(c["target"], "/view/camera/target");
        }
    }
}

}  // namespace

ParseResult parseScene(const std::string& text, const std::string& scenePath) {
    std::error_code ec;
    fs::path path = fs::absolute(scenePath, ec);
    if (ec) path = scenePath;
    path = path.lexically_normal();
    Reader r{path.parent_path().string()};
    ParseResult result;

    Json root;
    try {
        root = Json::parse(text.begin(), text.end());
    } catch (const Json::exception& e) {
        r.error("", std::string{"JSON 语法错误："} + e.what());
        result.issues = std::move(r.issues);
        return result;
    }

    SceneDocument doc;
    doc.path = path.string();
    doc.directory = path.parent_path().string();
    if (r.object(root, "", {"format", "version", "name", "physics", "models", "visuals", "view", "initialKeyframe", "controllers",
                            "sensors"})) {
        if (!root.contains("format")) {
            r.error("", "缺少 format（应为 “terra.scene”）");
        } else if (const auto s = r.string(root["format"], "/format"); s && *s != "terra.scene") {
            r.error("/format", "不是 Terra 场景文件（format 为 “" + *s + "”，应为 “terra.scene”）");
        }
        if (!root.contains("version")) {
            r.error("", "缺少 version");
        } else if (const auto n = r.integer(root["version"], "/version")) {
            if (*n > kSupportedVersion) {
                r.error("/version", "场景文件版本 " + std::to_string(*n) + " 高于本程序支持的版本 " +
                                        std::to_string(kSupportedVersion) + "，请更新程序");
            } else if (*n < 1) {
                r.error("/version", "版本号无效：" + std::to_string(*n));
            }
        }
        if (root.contains("name")) {
            if (const auto s = r.string(root["name"], "/name")) doc.name = *s;
        }
        if (root.contains("physics")) readPhysics(r, root["physics"], doc.physics);
        if (!root.contains("models")) r.error("", "缺少 models（模型实例列表）");
        else readModels(r, root["models"], doc.models);
        if (root.contains("visuals")) readVisuals(r, root["visuals"], doc.visuals);
        if (root.contains("view")) readView(r, root["view"], doc.view);
        if (root.contains("controllers")) readControllers(r, root["controllers"], doc.controllers);
        if (root.contains("sensors")) readSensors(r, root["sensors"], doc.sensors);
        if (root.contains("initialKeyframe")) {
            if (const auto k = r.string(root["initialKeyframe"], "/initialKeyframe")) {
                if (k->empty()) r.error("/initialKeyframe", "关键帧名为空（从模型初值开始时省略此字段）");
                doc.initialKeyframe = *k;
            }
        }
    }
    if (!r.hasErrors()) result.document = std::move(doc);
    result.issues = std::move(r.issues);
    return result;
}

ParseResult readSceneFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        ParseResult result;
        result.issues.push_back({Severity::Error, {}, {},
                                 fs::exists(path) ? "无法读取场景文件：" + path : "场景文件不存在：" + path});
        return result;
    }
    const std::string text{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    return parseScene(text, path);
}

}  // namespace terra::scene
