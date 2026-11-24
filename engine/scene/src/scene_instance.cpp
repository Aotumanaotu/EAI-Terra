// 场景实例与视觉部件（M2-05，M2-01 设计第 4、6 节）。

#include <filesystem>
#include <map>
#include <tuple>
#include <utility>

#include "terra/scene/pose_math.hpp"
#include "terra/scene/scene.hpp"

namespace terra::scene {

namespace {

namespace fs = std::filesystem;
namespace td = terra::data;
using Severity = SceneIssue::Severity;

// 找不到实体时的提示：没带前缀时提示写法；带了前缀时列出该实例中有名字的实体（最多 8 个）
std::string entityHint(const td::ModelDescription& d, const std::string& name) {
    const std::size_t slash = name.find('/');
    if (slash == std::string::npos) return "；实体名需要带实例前缀，如 “实例名/" + name + "”";
    const std::string prefix = name.substr(0, slash + 1);
    std::string list;
    int count = 0;
    for (const auto& e : d.entities) {
        if (e.name.rfind(prefix, 0) == 0) {
            if (count < 8) list += (list.empty() ? "" : "、") + e.name;
            ++count;
        }
    }
    if (count == 0) return "；场景中没有实例前缀 “" + prefix + "”";
    return "；该实例中的实体：" + list + (count > 8 ? " 等 " + std::to_string(count) + " 个" : std::string{});
}

}  // namespace

std::vector<td::Pose> entityPoses(const td::PhysicsSnapshot& snapshot) {
    std::vector<td::Pose> poses;
    poses.reserve(snapshot.entities.size());
    for (const auto& e : snapshot.entities) poses.push_back(e.pose);
    return poses;
}

SceneLoadResult instantiate(SceneDocument document, const WorldFactory& factory) {
    SceneLoadResult out;
    // 场景文件添加的 IMU（M4-02）：按全名中的实例前缀分到各实例，名字去掉前缀后交给后端
    std::vector<std::vector<terra::physics::ImuMount>> imus(document.models.size());
    bool sensorFailed = false;
    for (const SensorSpec& spec : document.sensors) {
        if (spec.type != "imu") continue;
        const bool onSite = !spec.site.empty();
        const std::string& mount = onSite ? spec.site : spec.entity;
        const std::string field = spec.where + (onSite ? "/site" : "/entity");
        const std::size_t slash = mount.find('/');
        std::size_t instance = document.models.size();
        if (slash != std::string::npos) {
            for (std::size_t i = 0; i < document.models.size(); ++i) {
                if (document.models[i].name == mount.substr(0, slash)) instance = i;
            }
        }
        if (instance == document.models.size()) {
            std::string list;
            for (const auto& m : document.models) list += (list.empty() ? "" : "、") + m.name;
            out.issues.push_back({Severity::Error, {}, field,
                                  "“" + mount + "”没有对应的实例：名字需要带实例前缀（场景中的实例：" + list + "）"});
            sensorFailed = true;
            continue;
        }
        const std::string prefix = instancePrefix(document.models[instance].name);
        terra::physics::ImuMount imu;
        if (onSite) {
            imu.site = mount.substr(prefix.size());
            imu.name = imu.site;
        } else {
            if (spec.name.rfind(prefix, 0) != 0 || spec.name.size() == prefix.size()) {
                out.issues.push_back({Severity::Error, document.models[instance].name, spec.where + "/name",
                                      "IMU 名字应以所属实例的前缀“" + prefix + "”开头，如 “" + prefix + "imu”"});
                sensorFailed = true;
                continue;
            }
            imu.body = mount.substr(prefix.size());
            imu.name = spec.name.substr(prefix.size());
            imu.pose = spec.pose;
        }
        imus[instance].push_back(std::move(imu));
    }
    if (sensorFailed) return out;

    ComposedWorld composed = composeWorld(document.models, document.physics, factory, document.path, imus);
    out.issues = std::move(composed.issues);
    if (!composed.ok()) return out;

    const td::ModelDescription& d = composed.world->description();
    // 初始关键帧（M3-02）：先于 assembly 对齐的计算应用，使“初始姿态”即关键帧姿态
    std::optional<std::size_t> home;
    if (!document.initialKeyframe.empty()) {
        const int k = d.findKeyframe(document.initialKeyframe);
        if (k < 0) {
            std::string names;
            for (const auto& key : d.keyframes) names += (names.empty() ? "" : "、") + (key.name.empty() ? std::string{"（无名）"} : key.name);
            out.issues.push_back({Severity::Error, {}, "/initialKeyframe",
                                  "找不到关键帧“" + document.initialKeyframe + "”" +
                                      (names.empty() ? "；场景中没有关键帧" : "；场景中的关键帧：" + names)});
            return out;
        }
        home = static_cast<std::size_t>(k);
        composed.world->resetToKeyframe(*home);
    }
    // PD 控制层（M3-04）：展开 "<实例>/*"，只接受作用于单个转动 / 滑动关节的力矩执行器，同一执行器不能出现两次
    std::map<std::uint32_t, PdAssignment> pd;
    bool pdFailed = false;
    for (const PdControllerSpec& c : document.controllers) {
        for (std::size_t k = 0; k < c.actuators.size(); ++k) {
            const std::string& name = c.actuators[k];
            const std::string where = c.where + "/actuators/" + std::to_string(k);
            std::vector<const td::ActuatorDescription*> matched;
            if (name.size() >= 2 && name.compare(name.size() - 2, 2, "/*") == 0) {
                const std::string prefix = name.substr(0, name.size() - 1);
                for (const auto& a : d.actuators) {
                    if (a.name.rfind(prefix, 0) == 0) matched.push_back(&a);
                }
                if (matched.empty()) {
                    out.issues.push_back({Severity::Error, {}, where, "没有前缀为 “" + prefix + "” 的执行器"});
                    pdFailed = true;
                }
            } else if (const td::ActuatorDescription* a = d.findActuator(name)) {
                matched.push_back(a);
            } else {
                out.issues.push_back({Severity::Error, {}, where, "找不到执行器“" + name + "”（执行器名需要带实例前缀，或写 “<实例名>/*”）"});
                pdFailed = true;
            }
            for (const td::ActuatorDescription* a : matched) {
                const td::JointDescription* j = a->joint.valid() ? d.joint(a->joint) : nullptr;
                const bool usable = a->kind == td::ActuatorKind::Motor && a->transmission == td::ActuatorTransmission::Joint &&
                                    a->inputs.size() == 1 && j != nullptr &&
                                    (j->type == td::JointType::Hinge || j->type == td::JointType::Slide);
                if (!usable) {
                    out.issues.push_back({Severity::Error, {}, where,
                                          "执行器“" + a->name + "”不能由 PD 层接管：只接受作用于单个转动或滑动关节的力矩执行器"});
                    pdFailed = true;
                    continue;
                }
                if (pd.count(a->id.value) != 0) {
                    out.issues.push_back({Severity::Error, {}, where, "执行器“" + a->name + "”已由前面的 PD 层接管"});
                    pdFailed = true;
                    continue;
                }
                pd[a->id.value] = PdAssignment{a->id, c.kp, c.kd};
            }
        }
    }
    if (pdFailed) return out;

    const std::vector<td::Pose> initial = entityPoses(*composed.world->snapshot());

    SceneInstance scene;
    bool failed = false;
    std::map<std::tuple<std::string, std::string, std::string>, std::string> seen;  // (实体, 文件, 节点) → 位置
    for (VisualBindingSpec& spec : document.visuals) {
        BoundVisual bound;
        if (spec.entity != "world") {
            const td::EntityDescription* e = d.findEntity(spec.entity);
            if (e == nullptr) {
                out.issues.push_back({Severity::Error, {}, spec.where + "/entity",
                                      "找不到实体“" + spec.entity + "”" + entityHint(d, spec.entity)});
                failed = true;
                continue;
            }
            bound.entity = e->id;
        }
        const auto key = std::make_tuple(spec.entity, spec.gltf, spec.node);
        if (const auto it = seen.find(key); it != seen.end()) {
            out.issues.push_back({Severity::Error, {}, spec.where,
                                  "重复绑定：实体、glTF 文件与节点都与 " + it->second + " 相同"});
            failed = true;
            continue;
        }
        seen.emplace(key, spec.where);
        std::error_code ec;
        if (!fs::is_regular_file(spec.gltf, ec)) {
            out.issues.push_back({Severity::Warning, {}, spec.where + "/gltf",
                                  "glTF 文件不存在：" + spec.gltf + "；忽略该绑定，实体改为显示 MJCF 几何体"});
            continue;
        }
        if (spec.frame == VisualFrame::Assembly && bound.entity.valid()) {
            bound.alignment = inverse(initial[bound.entity.value - 1]);
        }
        bound.spec = spec;
        scene.bindings.push_back(std::move(bound));
    }
    if (failed) return out;

    // 所属实体有 glTF 绑定、且没有任何绑定要求 showGeoms 时，该实体的 MJCF 几何体默认隐藏
    std::map<std::uint32_t, bool> boundEntities;  // 实体 → 是否保留几何体
    for (const BoundVisual& b : scene.bindings) {
        if (!b.entity.valid()) continue;
        bool& keep = boundEntities[b.entity.value];
        keep = keep || b.spec.showGeoms;
    }
    for (const td::GeomDescription& g : d.geoms) {
        VisualPart part;
        part.id = td::VisualId{static_cast<std::uint32_t>(scene.visuals.size() + 1)};
        part.kind = VisualPart::Kind::Geom;
        part.entity = g.entity;
        part.modelInstance = g.modelInstance;
        part.geom = g.id;
        if (g.entity.valid()) {
            const auto it = boundEntities.find(g.entity.value);
            part.hiddenByBinding = it != boundEntities.end() && !it->second;
        }
        scene.visuals.push_back(part);
    }
    for (std::size_t k = 0; k < scene.bindings.size(); ++k) {
        const BoundVisual& b = scene.bindings[k];
        VisualPart part;
        part.id = td::VisualId{static_cast<std::uint32_t>(scene.visuals.size() + 1)};
        part.kind = VisualPart::Kind::Gltf;
        part.entity = b.entity;
        if (const auto* e = d.entity(b.entity)) part.modelInstance = e->modelInstance;
        part.binding = static_cast<int>(k);
        scene.visuals.push_back(part);
    }

    // 传感器的采样频率（M4-02）：场景文件写了的按场景，否则等于物理频率；不高于物理频率
    const double physicsRate = d.timestep > 0 ? 1.0 / d.timestep : 0.0;
    for (const td::SensorDescription& s : d.sensors) {
        SensorConfig config{s.id, physicsRate};
        for (const SensorSpec& spec : document.sensors) {
            if (spec.name != s.name || !spec.rateHz) continue;
            config.rateHz = *spec.rateHz;
            if (config.rateHz > physicsRate) {
                out.issues.push_back({Severity::Warning, {}, spec.where + "/rateHz",
                                      "频率 " + std::to_string(*spec.rateHz) + " Hz 高于物理频率，按物理频率 " +
                                          std::to_string(physicsRate) + " Hz 采样"});
                config.rateHz = physicsRate;
            }
        }
        scene.sensors.push_back(config);
    }

    // 相机（M4-03）：MJCF 中的相机或装在实体上；只支持固定在实体上的透视相机
    for (const SensorSpec& spec : document.sensors) {
        if (spec.type != "camera") continue;
        CameraSensor camera;
        camera.name = spec.name;
        camera.where = spec.where;
        if (!spec.camera.empty()) {
            const td::CameraDescription* c = d.findCamera(spec.camera);
            if (c == nullptr) {
                std::string list;
                for (const auto& known : d.cameras) list += (list.empty() ? "" : "、") + known.name;
                out.issues.push_back({Severity::Error, {}, spec.where + "/camera",
                                      "找不到相机“" + spec.camera + "”" + (list.empty() ? "；场景中没有 MJCF 相机" : "；场景中的相机：" + list)});
                failed = true;
                continue;
            }
            if (!c->fixed || !c->perspective) {
                out.issues.push_back({Severity::Error, {}, spec.where + "/camera",
                                      "相机“" + spec.camera + "”" + (c->fixed ? "是正交投影" : "使用跟踪模式") + "，目前只支持固定在实体上的透视相机"});
                failed = true;
                continue;
            }
            camera.entity = c->entity;
            camera.mount = c->mount;
            camera.fovyDegrees = c->fovyDegrees;
            if (c->width > 0 && c->height > 0) {
                camera.width = c->width;
                camera.height = c->height;
            }
        } else {
            const td::EntityDescription* e = d.findEntity(spec.entity);
            if (e == nullptr) {
                out.issues.push_back({Severity::Error, {}, spec.where + "/entity", "找不到实体“" + spec.entity + "”" + entityHint(d, spec.entity)});
                failed = true;
                continue;
            }
            const std::size_t slash = spec.entity.find('/');
            const std::string prefix = slash == std::string::npos ? std::string{} : spec.entity.substr(0, slash + 1);
            if (prefix.empty() || spec.name.rfind(prefix, 0) != 0 || spec.name.size() == prefix.size()) {
                out.issues.push_back({Severity::Error, {}, spec.where + "/name",
                                      "相机名字应以所属实例的前缀“" + prefix + "”开头，如 “" + prefix + "camera”"});
                failed = true;
                continue;
            }
            camera.entity = e->id;
            camera.mount = spec.pose;
        }
        if (spec.width) camera.width = *spec.width;
        if (spec.height) camera.height = *spec.height;
        if (spec.fovyDegrees) camera.fovyDegrees = *spec.fovyDegrees;
        camera.nearClip = spec.nearClip;
        camera.farClip = spec.farClip;
        camera.rgb = spec.rgb;
        camera.depth = spec.depth;
        camera.rateHz = spec.rateHz.value_or(30.0);
        if (camera.rateHz > physicsRate) {
            out.issues.push_back({Severity::Warning, {}, spec.where + "/rateHz",
                                  "频率 " + std::to_string(camera.rateHz) + " Hz 高于物理频率，按物理频率 " + std::to_string(physicsRate) +
                                      " Hz 采样"});
            camera.rateHz = physicsRate;
        }
        scene.cameras.push_back(std::move(camera));
    }

    // 2D 激光雷达（M4-03b）：装在实体上（或 “world”）；名字带安装实体所属实例的前缀
    for (const SensorSpec& spec : document.sensors) {
        if (spec.type != "lidar2d") continue;
        LidarSensor lidar;
        lidar.name = spec.name;
        lidar.where = spec.where;
        if (spec.entity != "world") {
            const td::EntityDescription* e = d.findEntity(spec.entity);
            if (e == nullptr) {
                out.issues.push_back({Severity::Error, {}, spec.where + "/entity", "找不到实体“" + spec.entity + "”" + entityHint(d, spec.entity)});
                failed = true;
                continue;
            }
            const std::size_t slash = spec.entity.find('/');
            const std::string prefix = slash == std::string::npos ? std::string{} : spec.entity.substr(0, slash + 1);
            if (prefix.empty() || spec.name.rfind(prefix, 0) != 0 || spec.name.size() == prefix.size()) {
                out.issues.push_back({Severity::Error, {}, spec.where + "/name",
                                      "激光雷达名字应以所属实例的前缀“" + prefix + "”开头，如 “" + prefix + "scan”"});
                failed = true;
                continue;
            }
            lidar.entity = e->id;
        }
        constexpr double kRadiansPerDegree = 3.14159265358979323846 / 180.0;
        lidar.mount = spec.pose;
        lidar.angleMin = spec.angleMinDegrees * kRadiansPerDegree;
        lidar.angleMax = spec.angleMaxDegrees * kRadiansPerDegree;
        lidar.samples = spec.samples;
        lidar.rangeMin = spec.rangeMin;
        lidar.rangeMax = spec.rangeMax;
        lidar.hitVisual = spec.hitVisual;
        lidar.rateHz = spec.rateHz.value_or(10.0);
        if (lidar.rateHz > physicsRate) {
            out.issues.push_back({Severity::Warning, {}, spec.where + "/rateHz",
                                  "频率 " + std::to_string(lidar.rateHz) + " Hz 高于物理频率，按物理频率 " + std::to_string(physicsRate) +
                                      " Hz 采样"});
            lidar.rateHz = physicsRate;
        }
        scene.lidars.push_back(std::move(lidar));
    }
    if (failed) return out;

    scene.document = std::move(document);
    scene.world = std::move(composed.world);
    scene.homeKeyframe = home;
    for (const auto& entry : pd) scene.pd.push_back(entry.second);
    out.scene = std::move(scene);
    return out;
}

SceneLoadResult loadScene(const std::string& path, const WorldFactory& factory) {
    ParseResult parsed = readSceneFile(path);
    if (!parsed.document) {
        SceneLoadResult out;
        out.issues = std::move(parsed.issues);
        return out;
    }
    SceneLoadResult out = instantiate(std::move(*parsed.document), factory);
    out.issues.insert(out.issues.begin(), parsed.issues.begin(), parsed.issues.end());  // 解析阶段的警告在前
    return out;
}

std::vector<VisualPose> visualPoses(const SceneInstance& scene, const std::vector<td::Pose>& current) {
    const td::ModelDescription& d = scene.world->description();
    const auto entityPose = [&](td::EntityId id) {
        return id.valid() && id.value <= current.size() ? current[id.value - 1] : td::Pose{};
    };
    std::vector<VisualPose> poses;
    poses.reserve(scene.visuals.size());
    for (const VisualPart& part : scene.visuals) {
        VisualPose p;
        p.id = part.id;
        const td::Pose e = entityPose(part.entity);
        if (part.kind == VisualPart::Kind::Geom) {
            const td::GeomDescription* g = d.geom(part.geom);
            p.pose = g != nullptr ? multiply(e, g->local) : e;
        } else {
            const BoundVisual& b = scene.bindings[static_cast<std::size_t>(part.binding)];
            p.pose = multiply(multiply(e, b.alignment), b.spec.local);
            p.scale = b.spec.scale;
        }
        poses.push_back(p);
    }
    return poses;
}

}  // namespace terra::scene
