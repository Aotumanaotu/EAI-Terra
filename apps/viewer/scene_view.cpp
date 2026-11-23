#include "scene_view.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace terra::viewer {

namespace r = terra::render;
namespace td = terra::data;
namespace sc = terra::scene;
namespace as = terra::asset;

namespace {

constexpr float kInfinitePlaneHalf = 25.0f;  // MJCF 平面尺寸为 0 表示无限大，显示时取 50 m 见方
constexpr float kHiddenAlpha = 0.01f;

std::string key(const char* shape, const td::Vec3& s) {
    char buf[128];
    std::snprintf(buf, sizeof buf, "%s:%.9g:%.9g:%.9g", shape, s.x, s.y, s.z);
    return buf;
}

// MJCF 网格 → 渲染网格：每个三角形单独的顶点与面法线（平直着色，与 MuJoCo 显示一致）
r::MeshData flatMesh(const td::MeshDescription& m) {
    r::MeshData out;
    out.vertices.reserve(m.triangles.size() * 3);
    out.indices.reserve(m.triangles.size() * 3);
    bool first = true;
    for (const auto& t : m.triangles) {
        if (t[0] >= m.vertices.size() || t[1] >= m.vertices.size() || t[2] >= m.vertices.size()) continue;
        const td::Vec3& a = m.vertices[t[0]];
        const td::Vec3& b = m.vertices[t[1]];
        const td::Vec3& c = m.vertices[t[2]];
        const double e1[3] = {b.x - a.x, b.y - a.y, b.z - a.z};
        const double e2[3] = {c.x - a.x, c.y - a.y, c.z - a.z};
        double n[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]};
        const double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        if (len > 0) {
            for (double& x : n) x /= len;
        } else {
            n[0] = 0;
            n[1] = 0;
            n[2] = 1;
        }
        for (const td::Vec3* v : {&a, &b, &c}) {
            r::MeshVertex mv{};
            mv.position[0] = static_cast<float>(v->x);
            mv.position[1] = static_cast<float>(v->y);
            mv.position[2] = static_cast<float>(v->z);
            for (std::size_t k = 0; k < 3; ++k) mv.normal[k] = static_cast<float>(n[k]);
            out.indices.push_back(static_cast<std::uint32_t>(out.vertices.size()));
            out.vertices.push_back(mv);
            for (std::size_t k = 0; k < 3; ++k) {
                out.boundsMin[k] = first ? mv.position[k] : std::min(out.boundsMin[k], mv.position[k]);
                out.boundsMax[k] = first ? mv.position[k] : std::max(out.boundsMax[k], mv.position[k]);
            }
            first = false;
        }
    }
    r::computeTangents(out);
    return out;
}

}  // namespace

float srgbToLinear(float c) {
    return c <= 0.04045f ? c / 12.92f : std::pow((c + 0.055f) / 1.055f, 2.4f);
}

r::Matrix4 poseMatrix(const td::Pose& pose, double scale) {
    const td::Quaternion& q = pose.orientation;
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    const double w = n > 0 ? q.w / n : 1.0;
    const double x = n > 0 ? q.x / n : 0.0;
    const double y = n > 0 ? q.y / n : 0.0;
    const double z = n > 0 ? q.z / n : 0.0;
    // 旋转矩阵列（列主序）× 缩放，最后一列为平移
    const double m[9] = {1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y),
                         2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x),
                         2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)};
    r::Matrix4 out{};
    for (std::size_t c = 0; c < 3; ++c) {
        for (std::size_t row = 0; row < 3; ++row) out[c * 4 + row] = static_cast<float>(m[c * 3 + row] * scale);
    }
    out[12] = static_cast<float>(pose.position.x);
    out[13] = static_cast<float>(pose.position.y);
    out[14] = static_cast<float>(pose.position.z);
    out[15] = 1.0f;
    return out;
}

const r::GpuMesh* SceneView::mesh(const std::string& meshKey, const r::MeshData& data) {
    auto it = meshes_.find(meshKey);
    if (it == meshes_.end()) {
        it = meshes_.emplace(meshKey, std::make_unique<r::GpuMesh>(r::GpuMesh::create(data))).first;
    }
    return it->second.get();
}

std::unique_ptr<SceneView> SceneView::create(const sc::SceneInstance& scene, std::vector<SceneMessage>& messages) {
    auto view = std::unique_ptr<SceneView>(new SceneView());
    const td::ModelDescription& d = scene.world->description();

    // MJCF 几何体
    view->geoms_.resize(d.geoms.size());
    std::size_t unsupported = 0;
    for (const td::GeomDescription& g : d.geoms) {
        GeomDraw& draw = view->geoms_[g.id.value - 1];
        draw.group = g.group;
        draw.collides = g.collides;
        draw.shape = r::identityMatrix();
        draw.material.baseColor = {srgbToLinear(g.rgba[0]), srgbToLinear(g.rgba[1]), srgbToLinear(g.rgba[2])};
        draw.material.metallic = 0.0f;
        draw.material.roughness = 0.55f;
        if (g.rgba[3] < kHiddenAlpha) continue;  // MJCF 用 alpha = 0 隐藏
        const auto hx = static_cast<float>(g.size.x);
        const auto hy = static_cast<float>(g.size.y);
        const auto hz = static_cast<float>(g.size.z);
        switch (g.shape) {
            case td::GeomShape::Plane: {
                const float px = hx > 0 ? hx : kInfinitePlaneHalf;
                const float py = hy > 0 ? hy : kInfinitePlaneHalf;
                draw.mesh = view->mesh(key("plane", {px, py, 0}), r::meshes::plane(px, py));
                draw.castsShadow = false;
                break;
            }
            case td::GeomShape::Sphere: draw.mesh = view->mesh(key("sphere", g.size), r::meshes::sphere(hx)); break;
            case td::GeomShape::Ellipsoid:
                draw.mesh = view->mesh("sphere:unit", r::meshes::sphere(1.0f));
                draw.shape = r::translationScale({0, 0, 0}, {hx, hy, hz});
                break;
            case td::GeomShape::Box: draw.mesh = view->mesh(key("box", g.size), r::meshes::box(hx, hy, hz)); break;
            case td::GeomShape::Cylinder: draw.mesh = view->mesh(key("cylinder", g.size), r::meshes::cylinder(hx, hy)); break;
            case td::GeomShape::Capsule: draw.mesh = view->mesh(key("capsule", g.size), r::meshes::capsule(hx, hy)); break;
            case td::GeomShape::Mesh:
                if (g.mesh >= 0 && static_cast<std::size_t>(g.mesh) < d.meshes.size()) {
                    const td::MeshDescription& m = d.meshes[static_cast<std::size_t>(g.mesh)];
                    draw.mesh = view->mesh("mesh:" + std::to_string(g.mesh), flatMesh(m));
                }
                break;
            case td::GeomShape::Other: ++unsupported; break;
        }
    }
    if (unsupported > 0) {
        messages.push_back({SceneMessage::Kind::AssetInfo,
                            std::to_string(unsupported) + " 个几何体的形状（高度场、SDF 等）暂不显示"});
    }

    // glTF 绑定
    view->bindings_.resize(scene.bindings.size());
    for (std::size_t k = 0; k < scene.bindings.size(); ++k) {
        const sc::BoundVisual& b = scene.bindings[k];
        auto it = view->assets_.find(b.spec.gltf);
        if (it == view->assets_.end()) {
            const as::LoadResult loaded = as::loadGltf(b.spec.gltf);
            for (const auto& diag : loaded.diagnostics) {
                if (diag.severity != as::Diagnostic::Severity::Info) {
                    messages.push_back({SceneMessage::Kind::AssetWarning, b.spec.gltf + "：" + diag.message});
                }
            }
            std::unique_ptr<as::GpuAsset> gpu;
            if (loaded.asset) gpu = std::make_unique<as::GpuAsset>(as::GpuAsset::create(*loaded.asset));
            it = view->assets_.emplace(b.spec.gltf, std::move(gpu)).first;
            // 节点表随 GpuAsset 保存；按名字查找需要 Asset，这里先查完全部引用此文件的绑定
            for (std::size_t j = k; j < scene.bindings.size(); ++j) {
                const sc::BoundVisual& other = scene.bindings[j];
                if (other.spec.gltf != b.spec.gltf) continue;
                BindingDraw& draw = view->bindings_[j];
                draw.frame = other.spec.frame == sc::VisualFrame::Assembly ? as::NodeFrame::Assembly : as::NodeFrame::Node;
                if (!loaded.asset) {
                    messages.push_back({SceneMessage::Kind::AssetWarning,
                                        other.spec.where + "：glTF 读取失败，放弃该绑定，实体改为显示 MJCF 几何体"});
                    continue;
                }
                if (!other.spec.node.empty()) {
                    std::string error;
                    draw.node = as::findNode(*loaded.asset, other.spec.node, error);
                    if (draw.node < 0) {
                        messages.push_back({SceneMessage::Kind::AssetWarning,
                                            other.spec.where + "/node：" + error + "；放弃该绑定，实体改为显示 MJCF 几何体"});
                        continue;
                    }
                }
                draw.asset = it->second.get();
            }
        }
    }

    // 隐藏规则：只看可用的绑定
    view->hideGeomsOf_.assign(d.entities.size(), false);
    std::vector<bool> anyLive(d.entities.size(), false);
    std::vector<bool> keep(d.entities.size(), false);
    for (std::size_t k = 0; k < scene.bindings.size(); ++k) {
        const sc::BoundVisual& b = scene.bindings[k];
        if (!b.entity.valid() || view->bindings_[k].asset == nullptr) continue;
        const std::size_t e = b.entity.value - 1;
        anyLive[e] = true;
        keep[e] = keep[e] || b.spec.showGeoms;
    }
    for (std::size_t e = 0; e < d.entities.size(); ++e) view->hideGeomsOf_[e] = anyLive[e] && !keep[e];
    return view;
}

std::size_t SceneView::liveBindings() const noexcept {
    std::size_t n = 0;
    for (const auto& b : bindings_) n += b.asset != nullptr ? 1 : 0;
    return n;
}

std::vector<r::RenderItem> SceneView::items(const sc::SceneInstance& scene, const std::vector<sc::VisualPose>& poses,
                                            const SceneViewOptions& options) const {
    std::vector<r::RenderItem> out;
    const auto highlight = [&](r::RenderItem& item) {
        item.material.emissive = {item.material.emissive.x + kHighlightEmissive.x, item.material.emissive.y + kHighlightEmissive.y,
                                  item.material.emissive.z + kHighlightEmissive.z};
    };
    for (const sc::VisualPart& part : scene.visuals) {
        if (part.id.value == 0 || part.id.value > poses.size()) continue;
        const sc::VisualPose& pose = poses[part.id.value - 1];
        const bool selected = options.highlight.valid() && part.entity == options.highlight;
        if (part.kind == sc::VisualPart::Kind::Geom) {
            if (!part.geom.valid() || part.geom.value > geoms_.size()) continue;
            const GeomDraw& g = geoms_[part.geom.value - 1];
            if (g.mesh == nullptr) continue;
            const bool groupShown = g.group >= 0 && g.group < static_cast<int>(options.groups.size()) &&
                                    options.groups[static_cast<std::size_t>(g.group)];
            const bool hidden = part.entity.valid() && part.entity.value <= hideGeomsOf_.size() && hideGeomsOf_[part.entity.value - 1];
            const bool shown = (groupShown && !hidden) || (options.collision && g.collides);
            if (!shown) continue;
            r::RenderItem item;
            item.mesh = g.mesh;
            item.model = as::multiply(poseMatrix(pose.pose), g.shape);
            item.material = g.material;
            item.castsShadow = g.castsShadow;
            if (selected) highlight(item);
            out.push_back(item);
        } else {
            if (part.binding < 0 || static_cast<std::size_t>(part.binding) >= bindings_.size()) continue;
            const BindingDraw& b = bindings_[static_cast<std::size_t>(part.binding)];
            if (b.asset == nullptr) continue;
            auto parts = b.asset->subtreeItems(poseMatrix(pose.pose, pose.scale), b.node, b.frame, options.armOcclusion);
            if (selected) {
                for (auto& item : parts) highlight(item);
            }
            out.insert(out.end(), parts.begin(), parts.end());
        }
    }
    return out;
}

}  // namespace terra::viewer
