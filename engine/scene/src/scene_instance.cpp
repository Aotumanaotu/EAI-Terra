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
    ComposedWorld composed = composeWorld(document.models, document.physics, factory, document.path);
    out.issues = std::move(composed.issues);
    if (!composed.ok()) return out;

    const td::ModelDescription& d = composed.world->description();
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

    scene.document = std::move(document);
    scene.world = std::move(composed.world);
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
