#include "terra/asset/gltf.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <map>
#include <set>
#include <utility>

#define CGLTF_IMPLEMENTATION
#include "cgltf.h"

namespace terra::asset {

namespace {

namespace fs = std::filesystem;
namespace r = terra::render;

// 本实现支持的扩展（M1-09）。
const std::set<std::string> kSupportedExtensions = {"KHR_materials_emissive_strength"};

const char* resultText(cgltf_result result) {
    switch (result) {
        case cgltf_result_success: return "成功";
        case cgltf_result_data_too_short: return "数据过短";
        case cgltf_result_unknown_format: return "未知格式";
        case cgltf_result_invalid_json: return "JSON 无效";
        case cgltf_result_invalid_gltf: return "glTF 内容无效";
        case cgltf_result_invalid_options: return "选项无效";
        case cgltf_result_file_not_found: return "文件不存在";
        case cgltf_result_io_error: return "读取错误";
        case cgltf_result_out_of_memory: return "内存不足";
        case cgltf_result_legacy_gltf: return "glTF 1.0（不支持）";
        case cgltf_result_max_enum: break;
    }
    return "未知错误";
}

std::string text(const char* s) { return s != nullptr ? std::string{s} : std::string{}; }

struct DataGuard {
    cgltf_data* data = nullptr;
    ~DataGuard() {
        if (data != nullptr) cgltf_free(data);
    }
};

class Loader {
public:
    Loader(LoadResult& result, std::string path, const LoadOptions& options)
        : result_(result), path_(std::move(path)), dir_(fs::path{path_}.parent_path()), options_(options) {}

    void info(std::string message) { result_.diagnostics.push_back({Diagnostic::Severity::Info, std::move(message)}); }

    void warn(std::string message) { result_.diagnostics.push_back({Diagnostic::Severity::Warning, std::move(message)}); }
    void error(std::string message) { result_.diagnostics.push_back({Diagnostic::Severity::Error, std::move(message)}); }

    void run() {
        std::error_code ec;
        if (!fs::is_regular_file(path_, ec)) {
            error("glTF 文件不存在：" + path_);
            return;
        }
        cgltf_options options{};
        DataGuard guard;
        cgltf_result rc = cgltf_parse_file(&options, path_.c_str(), &guard.data);
        if (rc != cgltf_result_success) {
            error(std::string{"glTF 解析失败（"} + resultText(rc) + "）：" + path_);
            return;
        }
        cgltf_data* data = guard.data;
        for (cgltf_size i = 0; i < data->extensions_required_count; ++i) {
            const std::string name = text(data->extensions_required[i]);
            if (kSupportedExtensions.count(name) == 0) error("需要不支持的扩展 " + name + "（extensionsRequired），未加载：" + path_);
        }
        for (cgltf_size i = 0; i < data->extensions_used_count; ++i) {
            const std::string name = text(data->extensions_used[i]);
            bool required = false;
            for (cgltf_size k = 0; k < data->extensions_required_count; ++k) required = required || name == text(data->extensions_required[k]);
            if (!required && kSupportedExtensions.count(name) == 0) warn("使用了不支持的扩展 " + name + "，相关特性被忽略");
        }
        // 外部缓冲区文件：先逐个检查，缺失时给出 URI 与解析后的路径
        for (cgltf_size i = 0; i < data->buffers_count; ++i) {
            const std::string uri = text(data->buffers[i].uri);
            if (uri.empty() || uri.rfind("data:", 0) == 0) continue;
            const fs::path file = dir_ / decodedUri(uri);
            if (!fs::is_regular_file(file, ec)) {
                error("缓冲区文件缺失：buffers[" + std::to_string(i) + "].uri = \"" + uri + "\"（解析为 " + file.string() + "）");
            }
        }
        if (result_.hasErrors()) return;
        rc = cgltf_load_buffers(&options, data, path_.c_str());
        if (rc != cgltf_result_success) {
            error(std::string{"读取缓冲区失败（"} + resultText(rc) + "）：" + path_);
            return;
        }
        rc = cgltf_validate(data);
        if (rc != cgltf_result_success) {
            error(std::string{"glTF 数据不一致（"} + resultText(rc) + "）：" + path_);
            return;
        }
        if (data->animations_count > 0) warn("包含 " + std::to_string(data->animations_count) + " 个动画（不支持，已忽略）");
        if (data->skins_count > 0) warn("包含 " + std::to_string(data->skins_count) + " 个蒙皮（不支持，按静态网格显示）");

        data_ = data;
        asset_.source = path_;
        for (cgltf_size i = 0; i < data->materials_count; ++i) asset_.materials.push_back(material(data->materials[i]));

        // 节点表（M2-07）：下标与文件中的 nodes 数组相同
        for (cgltf_size i = 0; i < data->nodes_count; ++i) {
            const cgltf_node& n = data->nodes[i];
            AssetNode node;
            node.name = text(n.name);
            node.parent = n.parent != nullptr ? static_cast<int>(n.parent - data->nodes) : -1;
            cgltf_node_transform_world(&n, node.world.data());
            asset_.nodes.push_back(std::move(node));
        }

        const cgltf_scene* scene = data->scene != nullptr ? data->scene : (data->scenes_count > 0 ? &data->scenes[0] : nullptr);
        if (scene != nullptr) {
            for (cgltf_size i = 0; i < scene->nodes_count; ++i) visit(scene->nodes[i]);
        } else {
            for (cgltf_size i = 0; i < data->nodes_count; ++i) {
                if (data->nodes[i].parent == nullptr) visit(&data->nodes[i]);
            }
        }
        if (asset_.instances.empty()) warn("场景中没有可显示的网格");
        computeBounds();
        if (!result_.hasErrors()) result_.asset = std::move(asset_);
    }

private:
    static std::string decodedUri(const std::string& uri) {
        std::string copy = uri;
        copy.resize(cgltf_decode_uri(copy.data()));
        return copy;
    }

    // 贴图：按引用它的槽位决定 sRGB；同一图像只读一次。返回图像下标，缺失或不支持时返回 -1。
    int image(const cgltf_texture_view& view, bool srgb, const std::string& where) {
        if (view.texture == nullptr) return -1;
        if (view.texcoord != 0) warn(where + "：使用 TEXCOORD_" + std::to_string(view.texcoord) + "（只支持 TEXCOORD_0），按 TEXCOORD_0 采样");
        if (view.has_transform) warn(where + "：纹理坐标变换（KHR_texture_transform）不支持，已忽略");
        const cgltf_image* img = view.texture->image;
        if (img == nullptr) {
            warn(where + "：贴图格式不支持（如 KTX2 / WebP），该槽位不使用贴图");
            return -1;
        }
        const auto found = images_.find(img);
        if (found != images_.end()) {
            AssetImage& existing = asset_.images[static_cast<std::size_t>(found->second)];
            if (existing.srgb != srgb) warn(where + "：同一图像既作颜色贴图又作数据贴图，按" + (existing.srgb ? "sRGB" : "线性") + "读取");
            return existing.missing ? -1 : found->second;
        }
        AssetImage out;
        out.srgb = srgb;
        const auto index = static_cast<std::size_t>(img - data_->images);
        if (img->buffer_view != nullptr) {
            out.path = path_ + "#images[" + std::to_string(index) + "]";
            const auto* bytes = cgltf_buffer_view_data(img->buffer_view);
            auto decoded = r::decodeImage(bytes, img->buffer_view->size, out.path);
            if (decoded.ok()) {
                out.pixels = std::move(decoded.image);
            } else {
                out.missing = true;
                warn(where + "：内嵌图像无法解码：" + decoded.error);
            }
        } else {
            out.uri = text(img->uri);
            if (out.uri.rfind("data:", 0) == 0) {
                out.missing = true;
                warn(where + "：data URI 形式的图像不支持，该槽位不使用贴图");
            } else {
                out.path = (dir_ / decodedUri(out.uri)).string();
                auto loaded = r::loadImage(out.path);
                if (loaded.ok()) {
                    out.pixels = std::move(loaded.image);
                } else {
                    out.missing = true;
                    warn(where + "：贴图缺失或无法解码：\"" + out.uri + "\"（解析为 " + out.path + "）");
                }
            }
        }
        const int at = static_cast<int>(asset_.images.size());
        const bool missing = out.missing;
        asset_.images.push_back(std::move(out));
        images_[img] = at;
        return missing ? -1 : at;
    }

    AssetMaterial material(const cgltf_material& m) {
        AssetMaterial out;
        out.name = text(m.name);
        const std::string where = "材质 \"" + out.name + "\"";
        r::Material& f = out.factors;
        if (m.has_pbr_metallic_roughness) {
            const auto& pbr = m.pbr_metallic_roughness;
            f.baseColor = {pbr.base_color_factor[0], pbr.base_color_factor[1], pbr.base_color_factor[2]};
            f.metallic = pbr.metallic_factor;
            f.roughness = pbr.roughness_factor;
            out.baseColorImage = image(pbr.base_color_texture, true, where + " 基础色贴图");
            out.metallicRoughnessImage = image(pbr.metallic_roughness_texture, false, where + " 金属度-粗糙度贴图");
        } else {
            // glTF 默认：白色、金属度 1、粗糙度 1
            f.baseColor = {1.0f, 1.0f, 1.0f};
            f.metallic = 1.0f;
            f.roughness = 1.0f;
            if (m.has_pbr_specular_glossiness) warn(where + "：镜面-光泽度工作流（KHR_materials_pbrSpecularGlossiness）不支持，按默认材质显示");
        }
        f.emissive = {m.emissive_factor[0], m.emissive_factor[1], m.emissive_factor[2]};
        if (m.has_emissive_strength) {
            const float k = m.emissive_strength.emissive_strength;
            f.emissive = {f.emissive.x * k, f.emissive.y * k, f.emissive.z * k};
        }
        out.normalImage = image(m.normal_texture, false, where + " 法线贴图");
        if (m.normal_texture.texture != nullptr) f.normalScale = m.normal_texture.scale;
        out.occlusionImage = image(m.occlusion_texture, false, where + " 遮蔽贴图");
        if (m.occlusion_texture.texture != nullptr) f.occlusionStrength = m.occlusion_texture.scale;
        out.emissiveImage = image(m.emissive_texture, true, where + " 自发光贴图");
        if (options_.armOcclusion && m.occlusion_texture.texture == nullptr && out.metallicRoughnessImage >= 0) {
            const AssetImage& mr = asset_.images[static_cast<std::size_t>(out.metallicRoughnessImage)];
            if (looksLikeArmTexture(mr.uri.empty() ? mr.path : mr.uri)) {
                out.occlusionImage = out.metallicRoughnessImage;
                out.occlusionFromArm = true;
                f.occlusionStrength = 1.0f;
                info(where + "：按 ARM 约定使用金属度-粗糙度贴图的 R 通道作为环境遮蔽（" + (mr.uri.empty() ? mr.path : mr.uri) + "）");
            }
        }
        f.doubleSided = m.double_sided != 0;
        if (m.alpha_mode == cgltf_alpha_mode_blend) warn(where + "：透明混合（alphaMode = BLEND）暂不支持，按不透明显示");
        if (m.alpha_mode == cgltf_alpha_mode_mask) warn(where + "：alphaMode = MASK 暂不支持，按不透明显示");
        return out;
    }

    int mesh(const cgltf_mesh* m) {
        const auto found = meshes_.find(m);
        if (found != meshes_.end()) return found->second;
        AssetMesh out;
        out.name = text(m->name);
        for (cgltf_size p = 0; p < m->primitives_count; ++p) {
            const cgltf_primitive& prim = m->primitives[p];
            const std::string where = "网格 \"" + out.name + "\" 图元 " + std::to_string(p);
            if (prim.type != cgltf_primitive_type_triangles) {
                warn(where + "：只支持三角形图元，已跳过");
                continue;
            }
            if (prim.targets_count > 0) warn(where + "：变形目标不支持，已忽略");
            const cgltf_accessor* position = nullptr;
            const cgltf_accessor* normal = nullptr;
            const cgltf_accessor* uv = nullptr;
            const cgltf_accessor* tangent = nullptr;
            for (cgltf_size a = 0; a < prim.attributes_count; ++a) {
                const cgltf_attribute& attr = prim.attributes[a];
                if (attr.type == cgltf_attribute_type_position) position = attr.data;
                if (attr.type == cgltf_attribute_type_normal) normal = attr.data;
                if (attr.type == cgltf_attribute_type_texcoord && attr.index == 0) uv = attr.data;
                if (attr.type == cgltf_attribute_type_tangent) tangent = attr.data;
            }
            if (position == nullptr) {
                warn(where + "：没有 POSITION，已跳过");
                continue;
            }
            AssetPrimitive ap;
            r::MeshData& md = ap.mesh;
            const cgltf_size count = position->count;
            md.vertices.resize(count);
            for (cgltf_size v = 0; v < count; ++v) {
                r::MeshVertex& vert = md.vertices[v];
                vert = r::MeshVertex{};
                cgltf_accessor_read_float(position, v, vert.position, 3);
                if (normal != nullptr && normal->count == count) cgltf_accessor_read_float(normal, v, vert.normal, 3);
                if (uv != nullptr && uv->count == count) cgltf_accessor_read_float(uv, v, vert.uv, 2);
                vert.tangent[3] = 1.0f;
                if (tangent != nullptr && tangent->count == count) cgltf_accessor_read_float(tangent, v, vert.tangent, 4);
            }
            if (prim.indices != nullptr) {
                md.indices.resize(prim.indices->count);
                for (cgltf_size i = 0; i < prim.indices->count; ++i) {
                    md.indices[i] = static_cast<std::uint32_t>(cgltf_accessor_read_index(prim.indices, i));
                }
            } else {
                md.indices.resize(count);
                for (cgltf_size i = 0; i < count; ++i) md.indices[i] = static_cast<std::uint32_t>(i);
            }
            md.indices.resize(md.indices.size() / 3 * 3);
            const bool indicesValid = std::all_of(md.indices.begin(), md.indices.end(), [&](std::uint32_t i) { return i < count; });
            if (!indicesValid || md.indices.empty()) {
                warn(where + "：索引越界或没有三角形，已跳过");
                continue;
            }
            if (normal == nullptr) {
                warn(where + "：没有 NORMAL，已按面积加权生成");
                generateNormals(md);
            }
            if (tangent == nullptr) r::computeTangents(md);
            bounds(md);
            ap.material = prim.material != nullptr ? static_cast<int>(prim.material - data_->materials) : -1;
            out.primitives.push_back(std::move(ap));
        }
        const int at = static_cast<int>(asset_.meshes.size());
        asset_.meshes.push_back(std::move(out));
        meshes_[m] = at;
        return at;
    }

    void visit(const cgltf_node* node) {
        if (node == nullptr) return;
        if (node->mesh != nullptr) {
            if (node->has_mesh_gpu_instancing) warn("节点 \"" + text(node->name) + "\"：GPU 实例化扩展不支持，只显示一份");
            AssetInstance instance;
            instance.mesh = mesh(node->mesh);
            instance.node = text(node->name);
            r::Matrix4 world{};
            cgltf_node_transform_world(node, world.data());
            instance.transform = multiply(yUpToZUp(), world);
            instance.nodeIndex = static_cast<int>(node - data_->nodes);
            instance.nodeWorld = world;
            asset_.instances.push_back(std::move(instance));
        }
        for (cgltf_size i = 0; i < node->children_count; ++i) visit(node->children[i]);
    }

    static void generateNormals(r::MeshData& md) {
        std::vector<std::array<double, 3>> acc(md.vertices.size(), {0.0, 0.0, 0.0});
        for (std::size_t t = 0; t + 2 < md.indices.size(); t += 3) {
            const auto& a = md.vertices[md.indices[t]].position;
            const auto& b = md.vertices[md.indices[t + 1]].position;
            const auto& c = md.vertices[md.indices[t + 2]].position;
            const double e1[3] = {double{b[0]} - a[0], double{b[1]} - a[1], double{b[2]} - a[2]};
            const double e2[3] = {double{c[0]} - a[0], double{c[1]} - a[1], double{c[2]} - a[2]};
            const double n[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]};
            for (std::size_t k = 0; k < 3; ++k) {
                for (int j = 0; j < 3; ++j) acc[md.indices[t + static_cast<std::size_t>(j)]][k] += n[k];  // 叉积长度 = 2 × 面积
            }
        }
        for (std::size_t v = 0; v < md.vertices.size(); ++v) {
            const double len = std::sqrt(acc[v][0] * acc[v][0] + acc[v][1] * acc[v][1] + acc[v][2] * acc[v][2]);
            for (std::size_t k = 0; k < 3; ++k) {
                md.vertices[v].normal[k] = len > 0.0 ? static_cast<float>(acc[v][k] / len) : (k == 2 ? 1.0f : 0.0f);
            }
        }
    }

    static void bounds(r::MeshData& md) {
        if (md.vertices.empty()) return;
        md.boundsMin = {md.vertices[0].position[0], md.vertices[0].position[1], md.vertices[0].position[2]};
        md.boundsMax = md.boundsMin;
        for (const auto& v : md.vertices) {
            for (std::size_t k = 0; k < 3; ++k) {
                md.boundsMin[k] = std::min(md.boundsMin[k], v.position[k]);
                md.boundsMax[k] = std::max(md.boundsMax[k], v.position[k]);
            }
        }
    }

    void computeBounds() {
        bool any = false;
        for (const AssetInstance& inst : asset_.instances) {
            for (const AssetPrimitive& p : asset_.meshes[static_cast<std::size_t>(inst.mesh)].primitives) {
                for (int corner = 0; corner < 8; ++corner) {
                    const float x = (corner & 1) ? p.mesh.boundsMax[0] : p.mesh.boundsMin[0];
                    const float y = (corner & 2) ? p.mesh.boundsMax[1] : p.mesh.boundsMin[1];
                    const float z = (corner & 4) ? p.mesh.boundsMax[2] : p.mesh.boundsMin[2];
                    const auto& m = inst.transform;
                    const float w[3] = {m[0] * x + m[4] * y + m[8] * z + m[12], m[1] * x + m[5] * y + m[9] * z + m[13],
                                        m[2] * x + m[6] * y + m[10] * z + m[14]};
                    for (std::size_t k = 0; k < 3; ++k) {
                        asset_.boundsMin[k] = any ? std::min(asset_.boundsMin[k], w[k]) : w[k];
                        asset_.boundsMax[k] = any ? std::max(asset_.boundsMax[k], w[k]) : w[k];
                    }
                    any = true;
                }
            }
        }
    }

    LoadResult& result_;
    std::string path_;
    fs::path dir_;
    LoadOptions options_;
    cgltf_data* data_ = nullptr;
    Asset asset_;
    std::map<const cgltf_image*, int> images_;
    std::map<const cgltf_mesh*, int> meshes_;
};

}  // namespace

std::size_t Asset::triangleCount() const noexcept {
    std::size_t n = 0;
    for (const AssetInstance& inst : instances) {
        if (inst.mesh < 0 || static_cast<std::size_t>(inst.mesh) >= meshes.size()) continue;
        for (const AssetPrimitive& p : meshes[static_cast<std::size_t>(inst.mesh)].primitives) n += p.mesh.triangleCount();
    }
    return n;
}

bool LoadResult::hasErrors() const noexcept {
    return std::any_of(diagnostics.begin(), diagnostics.end(), [](const Diagnostic& d) { return d.severity == Diagnostic::Severity::Error; });
}

std::string LoadResult::summary() const {
    std::string out;
    for (const Diagnostic& d : diagnostics) {
        out += d.severity == Diagnostic::Severity::Error ? "错误：" : d.severity == Diagnostic::Severity::Warning ? "警告：" : "信息：";
        out += d.message;
        out += "\n";
    }
    return out;
}

bool looksLikeArmTexture(const std::string& uriOrPath) {
    std::string name = fs::path{uriOrPath}.filename().string();
    std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return name.find("_arm") != std::string::npos;
}

LoadResult loadGltf(const std::string& path, const LoadOptions& options) {
    LoadResult result;
    Loader loader(result, path, options);
    loader.run();
    return result;
}

int findNode(const Asset& asset, const std::string& nameOrIndex, std::string& error) {
    error.clear();
    if (!nameOrIndex.empty() && nameOrIndex[0] == '#') {
        const std::string digits = nameOrIndex.substr(1);
        const bool numeric = !digits.empty() && std::all_of(digits.begin(), digits.end(), [](unsigned char c) { return std::isdigit(c) != 0; });
        if (!numeric || digits.size() > 9) {
            error = "节点下标写法应为 “#数字”：" + nameOrIndex;
            return -1;
        }
        const int index = std::stoi(digits);
        if (index >= static_cast<int>(asset.nodes.size())) {
            error = "节点下标 " + digits + " 超出范围（共 " + std::to_string(asset.nodes.size()) + " 个节点）";
            return -1;
        }
        return index;
    }
    int found = -1;
    int count = 0;
    for (std::size_t i = 0; i < asset.nodes.size(); ++i) {
        if (asset.nodes[i].name == nameOrIndex) {
            if (found < 0) found = static_cast<int>(i);
            ++count;
        }
    }
    if (count == 0) {
        std::string names;
        int listed = 0;
        for (const auto& n : asset.nodes) {
            if (n.name.empty() || listed >= 12) continue;
            names += (names.empty() ? "" : "、") + n.name;
            ++listed;
        }
        error = "找不到节点“" + nameOrIndex + "”" + (names.empty() ? std::string{} : "；文件中的节点：" + names);
        return -1;
    }
    if (count > 1) {
        error = "节点名“" + nameOrIndex + "”在文件中出现 " + std::to_string(count) + " 次，请改用 “#下标” 写法（第一个为 #" +
                std::to_string(found) + "）";
        return -1;
    }
    return found;
}

bool inSubtree(const std::vector<AssetNode>& nodes, int node, int root) {
    std::size_t guard = 0;
    for (int n = node; n >= 0 && guard <= nodes.size(); ++guard) {
        if (n == root) return true;
        if (static_cast<std::size_t>(n) >= nodes.size()) return false;
        n = nodes[static_cast<std::size_t>(n)].parent;
    }
    return false;
}

r::Matrix4 bindingBase(const std::vector<AssetNode>& nodes, int root, NodeFrame frame) {
    const bool strip = frame == NodeFrame::Node && root >= 0 && static_cast<std::size_t>(root) < nodes.size();
    return strip ? multiply(yUpToZUp(), invertAffine(nodes[static_cast<std::size_t>(root)].world)) : yUpToZUp();
}

std::vector<std::pair<int, r::Matrix4>> subtreeTransforms(const Asset& asset, int root, NodeFrame frame) {
    std::vector<std::pair<int, r::Matrix4>> out;
    const r::Matrix4 base = bindingBase(asset.nodes, root, frame);
    for (std::size_t i = 0; i < asset.instances.size(); ++i) {
        const AssetInstance& inst = asset.instances[i];
        if (root >= 0 && !inSubtree(asset.nodes, inst.nodeIndex, root)) continue;
        out.emplace_back(static_cast<int>(i), multiply(base, inst.nodeWorld));
    }
    return out;
}

r::Matrix4 invertAffine(const r::Matrix4& m) {
    // 列主序：线性部分 a[行][列] = m[列 * 4 + 行]，平移为第 3 列
    const double a00 = m[0], a10 = m[1], a20 = m[2];
    const double a01 = m[4], a11 = m[5], a21 = m[6];
    const double a02 = m[8], a12 = m[9], a22 = m[10];
    const double det = a00 * (a11 * a22 - a12 * a21) - a01 * (a10 * a22 - a12 * a20) + a02 * (a10 * a21 - a11 * a20);
    r::Matrix4 out{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    if (!(std::fabs(det) > 1e-20)) return out;
    const double inv[3][3] = {{(a11 * a22 - a12 * a21) / det, (a02 * a21 - a01 * a22) / det, (a01 * a12 - a02 * a11) / det},
                              {(a12 * a20 - a10 * a22) / det, (a00 * a22 - a02 * a20) / det, (a02 * a10 - a00 * a12) / det},
                              {(a10 * a21 - a11 * a20) / det, (a01 * a20 - a00 * a21) / det, (a00 * a11 - a01 * a10) / det}};
    const double t[3] = {m[12], m[13], m[14]};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t col = 0; col < 3; ++col) out[col * 4 + row] = static_cast<float>(inv[row][col]);
        out[12 + row] = static_cast<float>(-(inv[row][0] * t[0] + inv[row][1] * t[1] + inv[row][2] * t[2]));
    }
    return out;
}

r::Matrix4 yUpToZUp() {
    // 列主序：第 0 列 (1, 0, 0)，第 1 列 Y → Z (0, 0, 1)，第 2 列 Z → −Y (0, −1, 0)
    return {1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1};
}

r::Matrix4 multiply(const r::Matrix4& a, const r::Matrix4& b) {
    r::Matrix4 out{};
    for (std::size_t c = 0; c < 4; ++c) {
        for (std::size_t row = 0; row < 4; ++row) {
            float s = 0.0f;
            for (std::size_t k = 0; k < 4; ++k) s += a[k * 4 + row] * b[c * 4 + k];
            out[c * 4 + row] = s;
        }
    }
    return out;
}

}  // namespace terra::asset
