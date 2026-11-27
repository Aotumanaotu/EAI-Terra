// 场景文件读取与校验（M2-05，M2-01 设计第 3 节）。nlohmann/json 只在本文件中使用，不进入公共头。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <optional>
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
        if (!r.object(item, where, {"name", "mjcf", "position", "rotation", "fixedBase", "torqueActuators"})) continue;
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
        if (item.contains("torqueActuators")) {
            if (const auto b = r.boolean(item["torqueActuators"], where + "/torqueActuators")) m.torqueActuators = *b;
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

// ---- 传感器噪声（M4-09）----

#ifndef TERRA_SENSOR_PRESETS
#define TERRA_SENSOR_PRESETS ""
#endif

// 雷达预设文件：环境变量 TERRA_SENSOR_PRESETS 优先，否则为构建时写入的 assets/sensors/lidar_presets.json
std::string presetFile() {
    if (const char* env = std::getenv("TERRA_SENSOR_PRESETS"); env != nullptr && *env != '\0') return env;
    return TERRA_SENSOR_PRESETS;
}

// 非负的噪声参数（可给上限）
void noiseNumber(Reader& r, const Json& j, const std::string& where, const char* key, double& target, double max = -1.0) {
    if (!j.contains(key)) return;
    const auto d = r.number(j[key], where + "/" + key);
    if (!d) return;
    if (*d < 0.0 || (max >= 0.0 && *d > max)) {
        r.error(where + "/" + key, max >= 0.0 ? "应在 0 到 " + std::to_string(max) + " 之间" : std::string{"不能为负"});
        return;
    }
    target = *d;
}

void noiseEnabled(Reader& r, const Json& j, const std::string& where, bool& target) {
    if (!j.contains("enabled")) return;
    if (const auto b = r.boolean(j["enabled"], where + "/enabled")) target = *b;
}

void readImuAxis(Reader& r, const Json& j, const std::string& where, ImuNoiseAxis& a) {
    if (!r.object(j, where, {"noiseDensity", "randomWalk", "bias", "biasStd"})) return;
    noiseNumber(r, j, where, "noiseDensity", a.noiseDensity);
    noiseNumber(r, j, where, "randomWalk", a.randomWalk);
    noiseNumber(r, j, where, "biasStd", a.biasStd);
    if (j.contains("bias")) {
        double b[3] = {0, 0, 0};
        if (r.numbers(j["bias"], where + "/bias", 3, b)) a.bias = {b[0], b[1], b[2]};
    }
}

void readImuNoise(Reader& r, const Json& j, const std::string& where, ImuNoise& n) {
    if (!r.object(j, where, {"enabled", "gyro", "accel"})) return;
    n.configured = true;
    noiseEnabled(r, j, where, n.enabled);
    if (j.contains("gyro")) readImuAxis(r, j["gyro"], where + "/gyro", n.gyro);
    if (j.contains("accel")) readImuAxis(r, j["accel"], where + "/accel", n.accel);
}

void readLidarNoise(Reader& r, const Json& j, const std::string& where, LidarNoise& n) {
    if (!r.object(j, where, {"enabled", "preset", "rangeStd", "rangeStdRelative", "dropout", "resolution"})) return;
    n.configured = true;
    noiseEnabled(r, j, where, n.enabled);
    if (j.contains("preset")) {
        if (const auto name = r.string(j["preset"], where + "/preset")) {
            n.preset = *name;
            const std::string file = presetFile();
            std::ifstream in(file, std::ios::binary);
            const Json presets = in ? Json::parse(in, nullptr, false) : Json{};
            if (presets.is_discarded() || !presets.is_object() || !presets.contains("presets") || !presets["presets"].is_object()) {
                r.error(where + "/preset", "无法读取雷达预设文件：" + (file.empty() ? std::string{"（未设置）"} : file));
            } else if (!presets["presets"].contains(*name)) {
                std::string list;
                for (auto it = presets["presets"].begin(); it != presets["presets"].end(); ++it) list += (list.empty() ? "" : "、") + it.key();
                r.error(where + "/preset", "没有雷达预设“" + *name + "”（可用：" + list + "）");
            } else {
                const Json& p = presets["presets"][*name];
                if (p.contains("noise") && p["noise"].is_object()) {
                    const Json& v = p["noise"];
                    const std::string pw = where + "/preset(" + *name + ")";
                    noiseNumber(r, v, pw, "rangeStd", n.rangeStd);
                    noiseNumber(r, v, pw, "rangeStdRelative", n.rangeStdRelative);
                    noiseNumber(r, v, pw, "dropout", n.dropout, 1.0);
                    noiseNumber(r, v, pw, "resolution", n.resolution);
                }
            }
        }
    }
    // 显式写的字段覆盖预设
    noiseNumber(r, j, where, "rangeStd", n.rangeStd);
    noiseNumber(r, j, where, "rangeStdRelative", n.rangeStdRelative);
    noiseNumber(r, j, where, "dropout", n.dropout, 1.0);
    noiseNumber(r, j, where, "resolution", n.resolution);
}

void readCameraNoise(Reader& r, const Json& j, const std::string& where, CameraNoise& n) {
    if (!r.object(j, where, {"enabled", "depthStd", "depthStdQuadratic", "dropout", "pixelStd"})) return;
    n.configured = true;
    noiseEnabled(r, j, where, n.enabled);
    noiseNumber(r, j, where, "depthStd", n.depthStd);
    noiseNumber(r, j, where, "depthStdQuadratic", n.depthStdQuadratic);
    noiseNumber(r, j, where, "dropout", n.dropout, 1.0);
    noiseNumber(r, j, where, "pixelStd", n.pixelStd, 255.0);
}

void readNoiseSettings(Reader& r, const Json& j, NoiseSettings& n) {
    if (!r.object(j, "/noise", {"seed", "enabled"})) return;
    if (j.contains("seed")) {
        if (!j["seed"].is_number_unsigned() && !(j["seed"].is_number_integer() && j["seed"].get<long long>() >= 0)) {
            r.error("/noise/seed", "种子应为非负整数");
        } else {
            n.seed = j["seed"].get<std::uint64_t>();
        }
    }
    noiseEnabled(r, j, "/noise", n.enabled);
}

void readLabels(Reader& r, const Json& j, std::vector<std::pair<std::string, std::string>>& out) {
    if (!j.is_object()) {
        r.error("/labels", "应为对象（实体全名、实例名或带 * 的前缀 → 类别名）");
        return;
    }
    for (auto it = j.begin(); it != j.end(); ++it) {
        const std::string key = it.key();
        const std::size_t star = key.find('*');
        if (key.empty() || (star != std::string::npos && star + 1 != key.size())) {
            r.error("/labels/" + key, "写法应为实体全名、实例名，或以 * 结尾的前缀（如 “go2/*”）");
            continue;
        }
        if (const auto v = r.string(it.value(), "/labels/" + key)) {
            if (v->empty()) r.error("/labels/" + key, "类别名为空");
            else out.emplace_back(key, *v);
        }
    }
}

// 一个 IMU（M4-02）
void readImu(Reader& r, const Json& item, const std::string& where, SensorSpec& s) {
    if (!r.object(item, where, {"type", "name", "site", "entity", "position", "rotation", "rateHz", "noise"})) return;
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

void readThermalCamera(Reader& r, const Json& item, const std::string& where, SensorSpec& s);  // M5-06，定义见下

// 一个相机（M4-03）
void readCamera(Reader& r, const Json& item, const std::string& where, SensorSpec& s) {
    if (!r.object(item, where, {"type", "name", "camera", "entity", "position", "rotation", "width", "height", "fovyDeg", "near",
                                "far", "rateHz", "rgb", "depth", "displayRotationDeg", "segmentation", "noise", "thermal"})) {
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
    if (item.contains("thermal")) readThermalCamera(r, item, where, s);  // M5-06
    if (!s.rgb && !s.depth && !s.thermal.enabled) r.error(where, "rgb 与 depth 不能都关闭（热像仪写 thermal 时可以都关闭）");
    if (item.contains("segmentation")) {  // M4-09
        if (const auto b = r.boolean(item["segmentation"], where + "/segmentation")) s.segmentation = *b;
    }
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
                                "rangeMax", "rateHz", "hits", "noise"})) {
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

// ---- 3D 激光雷达（M5-11）----

// 3D 雷达预设与扫描图案所在目录：与雷达预设文件（TERRA_SENSOR_PRESETS）相同的目录
fs::path sensorPresetDirectory() { return fs::path{presetFile()}.parent_path(); }

// ---- 热成像（M5-06）----

// 温度：一个数（常数），或 [[t, T], …]（时间严格递增，t ≥ 0）
std::optional<TemperatureCurve> readTemperature(Reader& r, const Json& v, const std::string& where) {
    TemperatureCurve c;
    const auto valid = [&](double t) { return std::isfinite(t) && t > -273.15 && t < 10000.0; };
    if (v.is_number()) {
        const double t = v.get<double>();
        if (!valid(t)) {
            r.error(where, "温度应在 −273.15 °C 以上、10000 °C 以下");
            return std::nullopt;
        }
        c.points.emplace_back(0.0, t);
        return c;
    }
    if (!v.is_array() || v.empty()) {
        r.error(where, "温度应为数（°C），或随时间变化的曲线 [[时间（秒）, 温度（°C）], …]");
        return std::nullopt;
    }
    for (std::size_t i = 0; i < v.size(); ++i) {
        double pair[2] = {0, 0};
        if (!r.numbers(v[i], where + "/" + std::to_string(i), 2, pair)) return std::nullopt;
        if (!(pair[0] >= 0.0) || (!c.points.empty() && !(pair[0] > c.points.back().first))) {
            r.error(where + "/" + std::to_string(i), "曲线的时间应从 0 起严格递增");
            return std::nullopt;
        }
        if (!valid(pair[1])) {
            r.error(where + "/" + std::to_string(i), "温度应在 −273.15 °C 以上、10000 °C 以下");
            return std::nullopt;
        }
        c.points.emplace_back(pair[0], pair[1]);
    }
    return c;
}

// 场景文件的 thermal 段
void readThermalScene(Reader& r, const Json& j, ThermalSceneSpec& out) {
    const std::string where = "/thermal";
    if (!r.object(j, where, {"ambient", "sky", "temperatures", "hotspots"})) return;
    out.configured = true;
    for (const auto& [key, target] : {std::pair<const char*, double*>{"ambient", &out.ambient}, std::pair<const char*, double*>{"sky", &out.sky}}) {
        if (!j.contains(key)) continue;
        if (const auto d = r.number(j[key], where + "/" + key)) {
            if (*d > -273.15 && *d < 10000.0) *target = *d;
            else r.error(where + "/" + key, "温度应在 −273.15 °C 以上、10000 °C 以下");
        }
    }
    if (j.contains("temperatures")) {
        const Json& t = j["temperatures"];
        if (!t.is_object()) {
            r.error(where + "/temperatures", "应为对象（实体全名、实例名、以 * 结尾的前缀或 “world” → 温度）");
        } else {
            for (auto it = t.begin(); it != t.end(); ++it) {
                const std::string key = it.key();
                const std::string w = where + "/temperatures/" + key;
                const std::size_t star = key.find('*');
                if (key.empty() || (star != std::string::npos && star + 1 != key.size())) {
                    r.error(w, "写法应为实体全名、实例名、“world”，或以 * 结尾的前缀（如 “go2/*”）");
                    continue;
                }
                if (auto c = readTemperature(r, it.value(), w)) out.temperatures.emplace_back(key, std::move(*c));
            }
        }
    }
    if (j.contains("hotspots")) {
        const Json& h = j["hotspots"];
        if (!h.is_array()) {
            r.error(where + "/hotspots", "应为数组");
            return;
        }
        for (std::size_t i = 0; i < h.size(); ++i) {
            const std::string w = where + "/hotspots/" + std::to_string(i);
            if (!r.object(h[i], w, {"entity", "position", "radius", "temperature"})) continue;
            HotspotSpec spot;
            spot.where = w;
            if (!h[i].contains("entity")) r.error(w, "热点需要 entity（实体全名，或 “world”）");
            else if (const auto e = r.string(h[i]["entity"], w + "/entity")) spot.entity = *e;
            if (h[i].contains("position")) {
                if (const auto v = r.vec3(h[i]["position"], w + "/position")) spot.position = *v;
            }
            if (h[i].contains("radius")) {
                if (const auto d = r.number(h[i]["radius"], w + "/radius")) {
                    if (*d > 0) spot.radius = *d;
                    else r.error(w + "/radius", "半径必须大于 0");
                }
            }
            if (!h[i].contains("temperature")) {
                r.error(w, "热点需要 temperature（°C 或曲线）");
                continue;
            }
            if (auto c = readTemperature(r, h[i]["temperature"], w + "/temperature")) {
                spot.temperature = std::move(*c);
                out.hotspots.push_back(std::move(spot));
            }
        }
    }
}

// ---- 场景元素（M5-02a）----

void readElementBody(Reader& r, const Json& root, ElementDocument& e) {
    if (!r.object(root, "", {"format", "version", "name", "category", "visual", "collision", "physics", "thermal", "inspection", "license"})) return;
    if (!root.contains("format")) {
        r.error("", "缺少 format（应为 “terra.element”）");
    } else if (const auto f = r.string(root["format"], "/format"); f && *f != "terra.element") {
        r.error("/format", "不是 Terra 元素文件（format 为 “" + *f + "”，应为 “terra.element”）");
    }
    if (!root.contains("version")) {
        r.error("", "缺少 version");
    } else if (const auto n = r.integer(root["version"], "/version"); n && *n != 1) {
        r.error("/version", "元素文件版本 " + std::to_string(*n) + " 不受支持（应为 1）");
    }
    if (root.contains("name")) {
        if (const auto v = r.string(root["name"], "/name")) e.name = *v;
    }
    if (root.contains("category")) {
        if (const auto v = r.string(root["category"], "/category")) e.category = *v;
    }
    if (root.contains("visual")) {
        const Json& v = root["visual"];
        if (r.object(v, "/visual", {"gltf", "node", "scale", "position", "rotation"})) {
            if (v.contains("gltf")) {
                if (const auto f = r.string(v["gltf"], "/visual/gltf"); f && !f->empty()) e.gltf = r.path(*f);
            }
            if (v.contains("node")) {
                if (const auto n = r.string(v["node"], "/visual/node")) e.node = *n;
            }
            if (v.contains("scale")) {
                if (const auto d = r.number(v["scale"], "/visual/scale")) {
                    if (*d > 0) e.visualScale = *d;
                    else r.error("/visual/scale", "缩放必须大于 0");
                }
            }
            r.pose(v, "/visual", e.visualPose);
        }
    }
    if (!root.contains("collision")) {
        r.error("", "缺少 collision（boxes、mesh 或 none；见 M5-01 设计稿 2.1 节）");
    } else {
        const Json& c = root["collision"];
        if (r.object(c, "/collision", {"mode", "boxes", "meshes", "generatedBy"})) {
            std::string mode;
            if (!c.contains("mode")) r.error("/collision", "缺少 mode（boxes、mesh 或 none）");
            else if (const auto m = r.string(c["mode"], "/collision/mode")) mode = *m;
            if (c.contains("generatedBy")) {
                if (const auto g = r.string(c["generatedBy"], "/collision/generatedBy")) e.generatedBy = *g;
            }
            if (mode == "boxes") {
                e.collision = ElementCollision::Boxes;
                if (!c.contains("boxes") || !c["boxes"].is_array() || c["boxes"].empty()) {
                    r.error("/collision", "mode 为 boxes 时需要 boxes（至少一个盒体：position、rotation、halfSize）");
                } else {
                    for (std::size_t i = 0; i < c["boxes"].size(); ++i) {
                        const std::string w = "/collision/boxes/" + std::to_string(i);
                        const Json& b = c["boxes"][i];
                        if (!r.object(b, w, {"position", "rotation", "halfSize", "node"})) continue;
                        ElementBox box;
                        r.pose(b, w, box.pose);
                        if (!b.contains("halfSize")) {
                            r.error(w, "缺少 halfSize（半边长，米）");
                        } else if (const auto h = r.vec3(b["halfSize"], w + "/halfSize")) {
                            if (h->x > 0 && h->y > 0 && h->z > 0) box.halfSize = *h;
                            else r.error(w + "/halfSize", "半边长必须都大于 0");
                        }
                        e.boxes.push_back(box);
                    }
                }
            } else if (mode == "mesh") {
                e.collision = ElementCollision::Mesh;
                if (!c.contains("meshes") || !c["meshes"].is_array() || c["meshes"].empty()) {
                    r.error("/collision", "mode 为 mesh 时需要 meshes（至少一个：file、position、rotation、scale）");
                } else {
                    for (std::size_t i = 0; i < c["meshes"].size(); ++i) {
                        const std::string w = "/collision/meshes/" + std::to_string(i);
                        const Json& m = c["meshes"][i];
                        if (!r.object(m, w, {"file", "position", "rotation", "scale"})) continue;
                        ElementMesh mesh;
                        if (!m.contains("file")) {
                            r.error(w, "缺少 file（OBJ 或 STL，相对元素文件）");
                            continue;
                        }
                        if (const auto f = r.string(m["file"], w + "/file")) mesh.file = r.path(*f);
                        std::error_code ec;
                        const std::string ext = fs::path{mesh.file}.extension().string();
                        if (ext != ".obj" && ext != ".stl" && ext != ".OBJ" && ext != ".STL") {
                            r.error(w + "/file", "碰撞网格应为 .obj 或 .stl：" + mesh.file);
                        } else if (!fs::is_regular_file(mesh.file, ec)) {
                            r.error(w + "/file", "碰撞网格不存在：" + mesh.file);
                        }
                        r.pose(m, w, mesh.pose);
                        if (m.contains("scale")) {
                            if (const auto d = r.number(m["scale"], w + "/scale")) {
                                if (*d > 0) mesh.scale = *d;
                                else r.error(w + "/scale", "缩放必须大于 0");
                            }
                        }
                        e.meshes.push_back(mesh);
                    }
                }
            } else if (mode == "none") {
                e.collision = ElementCollision::None;
            } else if (!mode.empty()) {
                r.error("/collision/mode", "mode 应为 boxes、mesh 或 none（auto-boxes、auto-hulls 由导入生成，见 M5-02b）");
            }
        }
    }
    if (root.contains("physics")) {
        const Json& p = root["physics"];
        if (r.object(p, "/physics", {"movable", "mass", "friction"})) {
            if (p.contains("movable")) {
                if (const auto m = r.string(p["movable"], "/physics/movable")) {
                    if (*m == "layout") e.movable = ElementMovable::Layout;
                    else if (*m == "fixed") e.movable = ElementMovable::Fixed;
                    else if (*m == "dynamic") e.movable = ElementMovable::Dynamic;
                    else r.error("/physics/movable", "movable 应为 layout（只在布局编辑中移动）、fixed 或 dynamic（自由刚体）");
                }
            }
            if (p.contains("mass")) {
                if (const auto d = r.number(p["mass"], "/physics/mass")) {
                    if (*d >= 0) e.mass = *d;
                    else r.error("/physics/mass", "质量不能为负");
                }
            }
            if (p.contains("friction")) {
                if (const auto d = r.number(p["friction"], "/physics/friction")) {
                    if (*d >= 0) e.friction = *d;
                    else r.error("/physics/friction", "摩擦系数不能为负");
                }
            }
        }
    }
    if (e.movable == ElementMovable::Dynamic && !(e.mass > 0)) r.error("/physics/mass", "dynamic 元素需要大于 0 的 mass（千克）");
    if (e.movable == ElementMovable::Dynamic && e.collision == ElementCollision::None) r.error("/collision/mode", "dynamic 元素必须有碰撞体");
    if (root.contains("thermal")) {
        const Json& t = root["thermal"];
        if (r.object(t, "/thermal", {"default", "hotspots"})) {
            if (t.contains("default")) e.thermalDefault = readTemperature(r, t["default"], "/thermal/default");
            if (t.contains("hotspots")) {
                if (!t["hotspots"].is_array()) {
                    r.error("/thermal/hotspots", "应为数组");
                } else {
                    for (std::size_t i = 0; i < t["hotspots"].size(); ++i) {
                        const std::string w = "/thermal/hotspots/" + std::to_string(i);
                        const Json& h = t["hotspots"][i];
                        if (!r.object(h, w, {"position", "radius", "temperature"})) continue;
                        HotspotSpec spot;
                        spot.where = w;
                        if (h.contains("position")) {
                            if (const auto v = r.vec3(h["position"], w + "/position")) spot.position = *v;
                        }
                        if (h.contains("radius")) {
                            if (const auto d = r.number(h["radius"], w + "/radius")) {
                                if (*d > 0) spot.radius = *d;
                                else r.error(w + "/radius", "半径必须大于 0");
                            }
                        }
                        if (!h.contains("temperature")) {
                            r.error(w, "热点需要 temperature");
                            continue;
                        }
                        if (auto c = readTemperature(r, h["temperature"], w + "/temperature")) {
                            spot.temperature = std::move(*c);
                            e.hotspots.push_back(std::move(spot));
                        }
                    }
                }
            }
        }
    }
    if (root.contains("inspection")) {
        const Json& list = root["inspection"];
        if (!list.is_array()) {
            r.error("/inspection", "应为数组");
        } else {
            for (std::size_t i = 0; i < list.size(); ++i) {
                const std::string w = "/inspection/" + std::to_string(i);
                if (!r.object(list[i], w, {"name", "position", "distance", "heightDeg", "node"})) continue;
                ElementInspection point;
                if (list[i].contains("name")) {
                    if (const auto n = r.string(list[i]["name"], w + "/name")) point.name = *n;
                }
                if (point.name.empty()) r.error(w, "检查点需要 name");
                if (list[i].contains("position")) {
                    if (const auto v = r.vec3(list[i]["position"], w + "/position")) point.position = *v;
                }
                if (list[i].contains("distance")) {
                    if (const auto d = r.number(list[i]["distance"], w + "/distance")) point.distance = *d;
                }
                if (list[i].contains("heightDeg")) {
                    if (const auto d = r.number(list[i]["heightDeg"], w + "/heightDeg")) point.heightDeg = *d;
                }
                e.inspection.push_back(point);
            }
        }
    }
    if (root.contains("license")) {
        const Json& l = root["license"];
        if (r.object(l, "/license", {"source", "license", "author"})) {
            for (const auto& [key, target] : {std::pair<const char*, std::string*>{"source", &e.licenseSource},
                                              std::pair<const char*, std::string*>{"license", &e.licenseName},
                                              std::pair<const char*, std::string*>{"author", &e.licenseAuthor}}) {
                if (!l.contains(key)) continue;
                if (const auto v = r.string(l[key], std::string{"/license/"} + key)) *target = *v;
            }
        }
    }
}

// 场景文件的 elements：读入每个元素文件；元素文件中的问题归到 “/elements/<i>” 下
void readElements(Reader& r, const Json& v, std::vector<SceneElementSpec>& out) {
    if (!v.is_array()) {
        r.error("/elements", "elements 应为数组");
        return;
    }
    for (std::size_t i = 0; i < v.size(); ++i) {
        const std::string where = "/elements/" + std::to_string(i);
        const Json& item = v[i];
        if (!r.object(item, where, {"id", "element", "position", "rotation", "thermal"})) continue;
        SceneElementSpec e;
        e.where = where;
        if (!item.contains("id")) r.error(where, "缺少 id（元素实例名，如 “T1”）");
        else if (const auto s = r.string(item["id"], where + "/id")) e.id = *s;
        if (!e.id.empty() && !isValidInstanceName(e.id)) r.error(where + "/id", "id“" + e.id + "”不合法：只能用字母、数字、下划线与连字符，且以字母或下划线开头");
        if (!item.contains("element")) {
            r.error(where, "缺少 element（元素文件 *.terra-element.json）");
            continue;
        }
        const auto file = r.string(item["element"], where + "/element");
        if (!file) continue;
        e.file = r.path(*file);
        r.pose(item, where, e.placement);
        if (item.contains("thermal")) {
            if (r.object(item["thermal"], where + "/thermal", {"default"}) && item["thermal"].contains("default")) {
                e.thermal = readTemperature(r, item["thermal"]["default"], where + "/thermal/default");
            }
        }
        ElementParseResult parsed = readElementFile(e.file);
        for (auto& problem : parsed.issues) {
            problem.where = where + "（" + fs::path{e.file}.filename().string() + "）" + problem.where;
            r.issues.push_back(problem);
        }
        if (!parsed.document) continue;
        e.element = std::move(*parsed.document);
        out.push_back(std::move(e));
    }
}

// 相机的 thermal 块：预设给出分辨率、视场角、帧率与噪声（相机显式写的字段优先）
void readThermalCamera(Reader& r, const Json& item, const std::string& where, SensorSpec& s) {
    const Json& j = item["thermal"];
    const std::string w = where + "/thermal";
    if (!r.object(j, w, {"preset", "gain", "palette", "display", "noise"})) return;
    ThermalCameraSpec& t = s.thermal;
    t.enabled = true;
    if (!j.contains("preset")) {
        r.error(w, "热像仪需要 preset（如 “flir_lepton_3_5”；见 assets/sensors/thermal_presets.json）");
        return;
    }
    const auto name = r.string(j["preset"], w + "/preset");
    if (!name) return;
    t.preset = *name;
    const fs::path file = sensorPresetDirectory() / "thermal_presets.json";
    std::ifstream in(file, std::ios::binary);
    const Json presets = in ? Json::parse(in, nullptr, false) : Json{};
    if (presets.is_discarded() || !presets.is_object() || !presets.contains("presets") || !presets["presets"].is_object()) {
        r.error(w + "/preset", "无法读取热像仪预设文件：" + file.string());
        return;
    }
    if (!presets["presets"].contains(*name)) {
        std::string list;
        for (auto it = presets["presets"].begin(); it != presets["presets"].end(); ++it) list += (list.empty() ? "" : "、") + it.key();
        r.error(w + "/preset", "没有热像仪预设“" + *name + "”（可用：" + list + "）");
        return;
    }
    const Json& p = presets["presets"][*name];
    const std::string pw = w + "/preset(" + *name + ")";
    // 分辨率、视场角与帧率：相机没有写时取预设；竖直视场角按方形像素由水平视场角算出
    if (!item.contains("width") && p.contains("width")) {
        if (const auto n = r.integer(p["width"], pw + "/width")) s.width = static_cast<int>(*n);
    }
    if (!item.contains("height") && p.contains("height")) {
        if (const auto n = r.integer(p["height"], pw + "/height")) s.height = static_cast<int>(*n);
    }
    if (!item.contains("fovyDeg") && p.contains("hfovDeg")) {
        if (const auto h = r.number(p["hfovDeg"], pw + "/hfovDeg")) {
            constexpr double kPi = 3.14159265358979323846;
            const double width = static_cast<double>(s.width.value_or(640));
            const double height = static_cast<double>(s.height.value_or(480));
            s.fovyDegrees = 2.0 * std::atan(std::tan(*h * kPi / 360.0) * height / width) * 360.0 / kPi;
        }
    }
    if (!item.contains("rateHz") && p.contains("rateHz")) {
        if (const auto d = r.number(p["rateHz"], pw + "/rateHz")) s.rateHz = *d;
    }
    if (p.contains("netdK")) {
        if (const auto d = r.number(p["netdK"], pw + "/netdK")) t.noise.netdK = *d;
    }
    t.noise.configured = true;
    // 增益档：量程与精度
    std::string gain;
    if (p.contains("defaultGain")) {
        if (const auto g = r.string(p["defaultGain"], pw + "/defaultGain")) gain = *g;
    }
    if (j.contains("gain")) {
        if (const auto g = r.string(j["gain"], w + "/gain")) gain = *g;
    }
    if (!p.contains("gains") || !p["gains"].is_object() || !p["gains"].contains(gain)) {
        std::string list;
        if (p.contains("gains") && p["gains"].is_object()) {
            for (auto it = p["gains"].begin(); it != p["gains"].end(); ++it) list += (list.empty() ? "" : "、") + it.key();
        }
        r.error(w + "/gain", "预设“" + *name + "”没有增益档“" + gain + "”（可用：" + list + "）");
    } else {
        t.gain = gain;
        const Json& g = p["gains"][gain];
        double range[2] = {t.rangeMin, t.rangeMax};
        if (g.contains("range") && r.numbers(g["range"], pw + "/gains/" + gain + "/range", 2, range)) {
            t.rangeMin = range[0];
            t.rangeMax = range[1];
        }
        if (g.contains("accuracy") && g["accuracy"].is_object()) {
            noiseNumber(r, g["accuracy"], pw + "/gains/" + gain + "/accuracy", "absolute", t.noise.accuracyAbsolute);
            noiseNumber(r, g["accuracy"], pw + "/gains/" + gain + "/accuracy", "relative", t.noise.accuracyRelative, 1.0);
        }
    }
    if (j.contains("palette")) {
        if (const auto v = r.string(j["palette"], w + "/palette")) {
            if (*v == "ironbow") t.palette = ThermalPalette::Ironbow;
            else if (*v == "white-hot") t.palette = ThermalPalette::WhiteHot;
            else if (*v == "black-hot") t.palette = ThermalPalette::BlackHot;
            else if (*v == "rainbow") t.palette = ThermalPalette::Rainbow;
            else r.error(w + "/palette", "色板应为 ironbow、white-hot、black-hot 或 rainbow");
        }
    }
    if (j.contains("display")) {
        const Json& d = j["display"];
        if (d.is_string() && d.get<std::string>() == "auto") {
            t.autoDisplay = true;
        } else {
            double range[2] = {0, 0};
            if (r.numbers(d, w + "/display", 2, range)) {
                if (range[1] > range[0]) {
                    t.autoDisplay = false;
                    t.displayMin = range[0];
                    t.displayMax = range[1];
                } else {
                    r.error(w + "/display", "伪彩色区间的上限应大于下限");
                }
            }
        }
    }
    if (j.contains("noise")) {
        const Json& n = j["noise"];
        if (r.object(n, w + "/noise", {"enabled", "netdK", "accuracyAbsolute", "accuracyRelative"})) {
            noiseEnabled(r, n, w + "/noise", t.noise.enabled);
            noiseNumber(r, n, w + "/noise", "netdK", t.noise.netdK, 10.0);
            noiseNumber(r, n, w + "/noise", "accuracyAbsolute", t.noise.accuracyAbsolute, 1000.0);
            noiseNumber(r, n, w + "/noise", "accuracyRelative", t.noise.accuracyRelative, 1.0);
        }
    }
}

void readLidar3dNoiseFields(Reader& r, const Json& j, const std::string& where, Lidar3dNoise& n) {
    noiseNumber(r, j, where, "rangeStdNear", n.rangeStdNear);
    noiseNumber(r, j, where, "rangeNear", n.rangeNear);
    noiseNumber(r, j, where, "rangeStdFar", n.rangeStdFar);
    noiseNumber(r, j, where, "rangeFar", n.rangeFar);
    noiseNumber(r, j, where, "angleStdDeg", n.angleStdDeg, 10.0);
}

// 一个 3D 激光雷达；imu: true 时另返回一个 IMU 的描述
std::optional<SensorSpec> readLidar3d(Reader& r, const Json& item, const std::string& where, SensorSpec& s) {
    std::optional<SensorSpec> imu;
    if (!r.object(item, where, {"type", "name", "entity", "position", "rotation", "preset", "rateHz", "rangeMin", "rangeMax", "pointsPerSecond",
                                "hits", "noise", "imu", "motionDistortion", "pattern", "model"})) {
        return imu;
    }
    if (!item.contains("entity")) {
        r.error(where, "3D 激光雷达需要 entity（安装的实体，“world” 表示装在世界上）");
        return imu;
    }
    if (const auto n = r.string(item["entity"], where + "/entity")) s.entity = *n;
    r.pose(item, where, s.pose);
    if (s.name.empty() && !item.contains("name")) r.error(where, "3D 激光雷达需要 name（带实例前缀，如 “go2/mid360”）");
    if (!item.contains("preset")) {
        r.error(where, "3D 激光雷达需要 preset（扫描图案与参数，如 “livox_mid360”；见 assets/sensors/lidar3d_presets.json）");
        return imu;
    }
    const auto name = r.string(item["preset"], where + "/preset");
    if (!name) return imu;
    s.lidar3dPreset = *name;
    const fs::path dir = sensorPresetDirectory();
    std::ifstream in(dir / "lidar3d_presets.json", std::ios::binary);
    const Json presets = in ? Json::parse(in, nullptr, false) : Json{};
    if (presets.is_discarded() || !presets.is_object() || !presets.contains("presets") || !presets["presets"].is_object()) {
        r.error(where + "/preset", "无法读取 3D 雷达预设文件：" + (dir / "lidar3d_presets.json").string());
        return imu;
    }
    if (!presets["presets"].contains(*name)) {
        std::string list;
        for (auto it = presets["presets"].begin(); it != presets["presets"].end(); ++it) list += (list.empty() ? "" : "、") + it.key();
        r.error(where + "/preset", "没有 3D 雷达预设“" + *name + "”（可用：" + list + "）");
        return imu;
    }
    const Json& p = presets["presets"][*name];
    const std::string pw = where + "/preset(" + *name + ")";
    if (p.contains("pattern")) {
        if (const auto f = r.string(p["pattern"], pw + "/pattern")) s.patternFile = (dir / *f).lexically_normal().string();
    }
    const auto presetNumber = [&](const char* key, double& target) {
        if (p.contains(key)) {
            if (const auto d = r.number(p[key], pw + "/" + key)) target = *d;
        }
    };
    double rate = 10.0;
    presetNumber("pointsPerSecond", s.pointsPerSecond);
    presetNumber("rateHz", rate);
    presetNumber("rangeMin", s.rangeMin);
    presetNumber("rangeMax", s.rangeMax);
    s.rateHz = rate;
    if (p.contains("lines")) {
        if (const auto n = r.integer(p["lines"], pw + "/lines")) s.lines = static_cast<int>(std::max<long long>(1, *n));
    }
    if (p.contains("noise") && p["noise"].is_object()) {
        s.lidar3dNoise.configured = true;
        s.lidar3dNoise.preset = *name;
        readLidar3dNoiseFields(r, p["noise"], pw + "/noise", s.lidar3dNoise);
    }
    // 外观模型：预设的 model = {gltf（相对预设目录）, position, rotation（模型在安装坐标系中的位姿）}；场景写 "model": false 不显示
    bool showModel = true;
    if (item.contains("model")) {
        if (const auto b = r.boolean(item["model"], where + "/model")) showModel = *b;
    }
    if (showModel && p.contains("model") && p["model"].is_object()) {
        const Json& m = p["model"];
        if (m.contains("gltf")) {
            if (const auto f = r.string(m["gltf"], pw + "/model/gltf")) {
                const fs::path file = (dir / *f).lexically_normal();
                std::error_code ec;
                if (fs::is_regular_file(file, ec)) {
                    s.modelFile = file.string();
                    r.pose(m, pw + "/model", s.modelPose);
                } else {
                    r.warning(where, "没有外观模型 " + file.string() + "（运行 scripts/assets/fetch_sensor_models.sh 下载），不显示雷达外观；读数不受影响");
                }
            }
        }
    }
    // 显式写的字段覆盖预设；pattern 为自定义的扫描图案（相对场景文件，格式同 Livox 的 CSV）
    if (item.contains("pattern")) {
        if (const auto f = r.string(item["pattern"], where + "/pattern")) s.patternFile = r.path(*f);
    }
    for (const auto& [key, target] : {std::pair<const char*, double*>{"rangeMin", &s.rangeMin}, std::pair<const char*, double*>{"rangeMax", &s.rangeMax},
                                      std::pair<const char*, double*>{"pointsPerSecond", &s.pointsPerSecond}}) {
        if (item.contains(key)) {
            if (const auto d = r.number(item[key], where + "/" + key)) *target = *d;
        }
    }
    if (!(s.rangeMin >= 0.0)) r.error(where + "/rangeMin", "最小量程不能为负");
    if (!(s.rangeMax > s.rangeMin)) r.error(where + "/rangeMax", "最大量程必须大于最小量程");
    if (!(s.pointsPerSecond > 0.0 && s.pointsPerSecond <= 5.0e6)) r.error(where + "/pointsPerSecond", "点频应在 0 到 5,000,000 点/秒之间");
    if (item.contains("hits")) {
        if (const auto h = r.string(item["hits"], where + "/hits")) {
            if (*h == "visual") s.hitVisual = true;
            else if (*h != "collision") r.error(where + "/hits", "hits 应为 “collision”（参与碰撞的几何体）或 “visual”（显示的几何体）");
        }
    }
    if (item.contains("motionDistortion")) {
        if (const auto b = r.boolean(item["motionDistortion"], where + "/motionDistortion")) s.motionDistortion = *b;
    }
    if (item.contains("noise")) {
        const Json& n = item["noise"];
        if (r.object(n, where + "/noise", {"enabled", "rangeStdNear", "rangeNear", "rangeStdFar", "rangeFar", "angleStdDeg"})) {
            s.lidar3dNoise.configured = true;
            noiseEnabled(r, n, where + "/noise", s.lidar3dNoise.enabled);
            readLidar3dNoiseFields(r, n, where + "/noise", s.lidar3dNoise);
        }
    }
    if (item.contains("imu")) {
        const auto on = r.boolean(item["imu"], where + "/imu");
        if (on && *on) {
            if (!p.contains("imu") || !p["imu"].is_object()) {
                r.error(where + "/imu", "预设“" + *name + "”没有内置 IMU");
            } else {
                const Json& q = p["imu"];
                SensorSpec m;
                m.type = "imu";
                m.name = s.name + "_imu";
                m.entity = s.entity;
                m.where = where + "/imu";
                terra::data::Pose offset;
                if (q.contains("position")) {
                    if (const auto v = r.vec3(q["position"], pw + "/imu/position")) offset.position = *v;
                }
                m.pose = multiply(s.pose, offset);
                if (q.contains("rateHz")) {
                    if (const auto d = r.number(q["rateHz"], pw + "/imu/rateHz")) m.rateHz = *d;
                }
                if (q.contains("noise")) readImuNoise(r, q["noise"], pw + "/imu/noise", m.imuNoise);
                imu = std::move(m);
            }
        }
    }
    return imu;
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
            r.error(where, "缺少 type（imu、camera、lidar2d 或 lidar3d）");
            continue;
        }
        const auto type = r.string(item["type"], where + "/type");
        if (!type) continue;
        if (*type != "imu" && *type != "camera" && *type != "lidar2d" && *type != "lidar3d") {
            r.error(where + "/type", "未知的传感器类型“" + *type + "”（可用：imu、camera、lidar2d、lidar3d）");
            continue;
        }
        s.type = *type;
        if (item.contains("name")) {
            if (const auto n = r.string(item["name"], where + "/name")) {
                if (n->empty()) r.error(where + "/name", "名字为空");
                s.name = *n;
            }
        }
        std::optional<SensorSpec> extraImu;  // M5-11：3D 雷达内置的 IMU
        if (s.type == "imu") readImu(r, item, where, s);
        else if (s.type == "camera") readCamera(r, item, where, s);
        else if (s.type == "lidar2d") readLidar(r, item, where, s);
        else extraImu = readLidar3d(r, item, where, s);
        if (item.contains("rateHz")) {
            if (const auto d = r.number(item["rateHz"], where + "/rateHz")) {
                if (*d > 0) s.rateHz = *d;
                else r.error(where + "/rateHz", "频率必须大于 0");
            }
        }
        if (item.contains("noise")) {  // M4-09
            if (s.type == "imu") readImuNoise(r, item["noise"], where + "/noise", s.imuNoise);
            else if (s.type == "camera") readCameraNoise(r, item["noise"], where + "/noise", s.cameraNoise);
            else if (s.type == "lidar2d") readLidarNoise(r, item["noise"], where + "/noise", s.lidarNoise);
            // lidar3d 的 noise 已在 readLidar3d 中读取
        }
        if (!s.name.empty() && !names.insert(s.name).second) r.error(where + "/name", "传感器名“" + s.name + "”重复");
        out.push_back(std::move(s));
        if (extraImu) {
            if (!names.insert(extraImu->name).second) r.error(where + "/imu", "传感器名“" + extraImu->name + "”重复");
            out.push_back(std::move(*extraImu));
        }
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
    if (r.object(root, "", {"format", "version", "name", "title", "physics", "models", "visuals", "view", "initialKeyframe",
                            "controllers", "sensors", "noise", "labels", "thermal", "elements"})) {
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
        if (root.contains("title")) {  // M4-08c：资源库卡片上的短标题
            if (const auto s = r.string(root["title"], "/title")) doc.title = *s;
        }
        if (root.contains("physics")) readPhysics(r, root["physics"], doc.physics);
        if (!root.contains("models")) r.error("", "缺少 models（模型实例列表）");
        else readModels(r, root["models"], doc.models);
        if (root.contains("visuals")) readVisuals(r, root["visuals"], doc.visuals);
        if (root.contains("view")) readView(r, root["view"], doc.view);
        if (root.contains("controllers")) readControllers(r, root["controllers"], doc.controllers);
        if (root.contains("sensors")) readSensors(r, root["sensors"], doc.sensors);
        if (root.contains("elements")) readElements(r, root["elements"], doc.elements);  // M5-02a
        // M5-02a：元素的外观作为 glTF 绑定（实体 “<id>/body”）；没有外观时显示碰撞体
        for (const SceneElementSpec& e : doc.elements) {
            if (e.element.gltf.empty()) continue;
            VisualBindingSpec b;
            b.entity = e.id + "/" + kElementBody;
            b.gltf = e.element.gltf;
            b.node = e.element.node;
            b.local = e.element.visualPose;
            b.scale = e.element.visualScale;
            b.frame = VisualFrame::Node;
            b.where = e.where + "/element";
            doc.visuals.push_back(std::move(b));
        }
        // M5-11：传感器的外观模型作为 glTF 绑定加到 visuals 末尾（跟随安装的实体）
        for (const SensorSpec& s : doc.sensors) {
            if (s.modelFile.empty()) continue;
            VisualBindingSpec b;
            b.entity = s.entity;
            b.gltf = s.modelFile;
            b.local = multiply(s.pose, s.modelPose);
            b.frame = VisualFrame::Node;
            b.sensorModel = true;
            b.where = s.where + "/model";
            doc.visuals.push_back(std::move(b));
        }
        if (root.contains("noise")) readNoiseSettings(r, root["noise"], doc.noise);  // M4-09
        if (root.contains("labels")) readLabels(r, root["labels"], doc.labels);    // M4-09
        if (root.contains("thermal")) readThermalScene(r, root["thermal"], doc.thermal);  // M5-06
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

int scenePatternRank(const std::string& pattern, const std::string& entityName, const std::string& instanceName, std::size_t& length) {
    length = pattern.size();
    if (pattern == entityName) return 3;
    if (pattern == instanceName) return 2;
    if (!pattern.empty() && pattern.back() == '*' && entityName.compare(0, pattern.size() - 1, pattern, 0, pattern.size() - 1) == 0) {
        length = pattern.size() - 1;
        return 1;
    }
    return 0;
}

std::string sceneLabel(const SceneDocument& document, const std::string& entityName, const std::string& instanceName) {
    std::string best;
    std::size_t bestLength = 0;
    int bestRank = 0;  // 3：实体全名，2：实例名，1：前缀
    for (const auto& [pattern, label] : document.labels) {
        std::size_t length = 0;
        const int rank = scenePatternRank(pattern, entityName, instanceName, length);
        if (rank > bestRank || (rank == bestRank && rank == 1 && length > bestLength)) {
            best = label;
            bestRank = rank;
            bestLength = length;
        }
    }
    // M5-02a：没有写 labels 的元素取元素的类别
    if (best.empty()) {
        for (const SceneElementSpec& e : document.elements) {
            if (e.id == instanceName) return e.element.category;
        }
    }
    return best;
}

std::string segmentationLabelsJson(const SceneInstance& scene) {
    const auto& d = scene.world->description();
    nlohmann::ordered_json j;
    j["format"] = "terra.segmentation-labels";
    j["version"] = 1;
    j["background"] = 0;
    nlohmann::ordered_json labels = nlohmann::ordered_json::array();
    for (const auto& e : d.entities) {
        std::string instance;
        if (e.modelInstance.valid() && e.modelInstance.value <= d.modelInstances.size()) instance = d.modelInstances[e.modelInstance.value - 1].name;
        labels.push_back({{"id", e.id.value}, {"entity", e.name}, {"instance", instance}, {"label", sceneLabel(scene.document, e.name, instance)}});
    }
    j["labels"] = std::move(labels);
    return j.dump() + "\n";
}

ElementParseResult parseElement(const std::string& text, const std::string& path) {
    std::error_code ec;
    fs::path file = fs::absolute(path, ec);
    if (ec) file = path;
    file = file.lexically_normal();
    Reader r{file.parent_path().string()};
    ElementParseResult result;
    Json root;
    try {
        root = Json::parse(text.begin(), text.end());
    } catch (const Json::exception& e) {
        r.error("", std::string{"JSON 语法错误："} + e.what());
        result.issues = std::move(r.issues);
        return result;
    }
    ElementDocument doc;
    doc.path = file.string();
    doc.directory = file.parent_path().string();
    readElementBody(r, root, doc);
    if (doc.name.empty()) doc.name = file.filename().string();
    if (!r.hasErrors()) result.document = std::move(doc);
    result.issues = std::move(r.issues);
    return result;
}

ElementParseResult readElementFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        ElementParseResult result;
        result.issues.push_back({Severity::Error, {}, {}, fs::exists(path) ? "无法读取元素文件：" + path : "元素文件不存在：" + path});
        return result;
    }
    const std::string text{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    return parseElement(text, path);
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
