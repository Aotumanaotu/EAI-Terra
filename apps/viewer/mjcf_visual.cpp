#include "mjcf_visual.hpp"

#include <algorithm>
#include <cmath>

namespace terra::viewer {

namespace r = terra::render;
namespace td = terra::data;

namespace {

constexpr double kPi = 3.14159265358979323846;

void normalize(double v[3]) {
    const double len = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (len > 0) {
        for (int k = 0; k < 3; ++k) v[k] /= len;
    }
}

}  // namespace

float srgbToLinear(float c) {
    return c <= 0.04045f ? c / 12.92f : std::pow((c + 0.055f) / 1.055f, 2.4f);
}

r::MeshData mjcfMesh(const td::MeshDescription& m, std::array<float, 2> uvScale) {
    r::MeshData out;
    out.vertices.reserve(m.triangles.size() * 3);
    out.indices.reserve(m.triangles.size() * 3);
    const bool smooth = !m.normals.empty() && m.triangleNormals.size() == m.triangles.size();
    const bool textured = !m.texcoords.empty() && m.triangleTexcoords.size() == m.triangles.size();
    const double creaseCos = std::cos(kCreaseAngleDeg * kPi / 180.0);
    bool first = true;
    for (std::size_t i = 0; i < m.triangles.size(); ++i) {
        const auto& t = m.triangles[i];
        if (t[0] >= m.vertices.size() || t[1] >= m.vertices.size() || t[2] >= m.vertices.size()) continue;
        const td::Vec3& a = m.vertices[t[0]];
        const td::Vec3& b = m.vertices[t[1]];
        const td::Vec3& c = m.vertices[t[2]];
        const double e1[3] = {b.x - a.x, b.y - a.y, b.z - a.z};
        const double e2[3] = {c.x - a.x, c.y - a.y, c.z - a.z};
        double face[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]};
        const bool degenerate = face[0] == 0 && face[1] == 0 && face[2] == 0;
        normalize(face);
        if (degenerate) face[2] = 1;
        const td::Vec3* corners[3] = {&a, &b, &c};
        for (std::size_t k = 0; k < 3; ++k) {
            double n[3] = {face[0], face[1], face[2]};
            if (smooth && m.triangleNormals[i][k] < m.normals.size()) {
                const td::Vec3& given = m.normals[m.triangleNormals[i][k]];
                double g[3] = {given.x, given.y, given.z};
                normalize(g);
                // 折角保护：与面法线夹角过大（含退化的零法线）时用面法线
                if (degenerate || g[0] * face[0] + g[1] * face[1] + g[2] * face[2] >= creaseCos) {
                    n[0] = g[0];
                    n[1] = g[1];
                    n[2] = g[2];
                }
            }
            r::MeshVertex mv{};
            mv.position[0] = static_cast<float>(corners[k]->x);
            mv.position[1] = static_cast<float>(corners[k]->y);
            mv.position[2] = static_cast<float>(corners[k]->z);
            for (std::size_t q = 0; q < 3; ++q) mv.normal[q] = static_cast<float>(n[q]);
            if (textured && m.triangleTexcoords[i][k] < m.texcoords.size()) {
                const auto& uv = m.texcoords[m.triangleTexcoords[i][k]];
                mv.uv[0] = uv[0] * uvScale[0];
                mv.uv[1] = uv[1] * uvScale[1];
            }
            out.indices.push_back(static_cast<std::uint32_t>(out.vertices.size()));
            out.vertices.push_back(mv);
            for (std::size_t q = 0; q < 3; ++q) {
                out.boundsMin[q] = first ? mv.position[q] : std::min(out.boundsMin[q], mv.position[q]);
                out.boundsMax[q] = first ? mv.position[q] : std::max(out.boundsMax[q], mv.position[q]);
            }
            first = false;
        }
    }
    r::computeTangents(out);
    return out;
}

void scaleUv(r::MeshData& data, std::array<float, 2> scale) {
    for (auto& v : data.vertices) {
        v.uv[0] *= scale[0];
        v.uv[1] *= scale[1];
    }
    r::computeTangents(data);
}

MjcfSurface mjcfSurface(const td::GeomDescription& geom, const td::ModelDescription& model) {
    MjcfSurface out;
    r::Material& mat = out.material;
    mat.baseColor = {srgbToLinear(geom.rgba[0]), srgbToLinear(geom.rgba[1]), srgbToLinear(geom.rgba[2])};
    const td::MaterialDescription defaults;  // MuJoCo 的默认材质参数
    const bool hasMaterial = geom.material >= 0 && static_cast<std::size_t>(geom.material) < model.materials.size();
    const td::MaterialDescription& m = hasMaterial ? model.materials[static_cast<std::size_t>(geom.material)] : defaults;
    mat.roughness = m.roughness >= 0 ? std::min(m.roughness, 1.0f) : std::clamp(1.0f - 0.9f * m.shininess, 0.05f, 1.0f);
    mat.metallic = m.metallic >= 0 ? std::min(m.metallic, 1.0f) : 0.0f;
    mat.reflectance = std::clamp(m.specular, 0.0f, 1.0f);
    mat.emissive = {m.emission * mat.baseColor.x, m.emission * mat.baseColor.y, m.emission * mat.baseColor.z};
    if (!hasMaterial || m.texture < 0 || static_cast<std::size_t>(m.texture) >= model.textures.size()) return out;

    const td::TextureDescription& tex = model.textures[static_cast<std::size_t>(m.texture)];
    const bool meshWithoutUv = geom.shape == td::GeomShape::Mesh &&
                               (geom.mesh < 0 || static_cast<std::size_t>(geom.mesh) >= model.meshes.size() ||
                                model.meshes[static_cast<std::size_t>(geom.mesh)].texcoords.empty());
    if (tex.kind != td::TextureKind::TwoD || tex.rgba.empty() || meshWithoutUv || geom.shape == td::GeomShape::Other) {
        out.textureUnsupported = tex.kind != td::TextureKind::Skybox;
        return out;
    }
    out.texture = m.texture;
    const float rx = m.texRepeat[0] > 0 ? m.texRepeat[0] : 1.0f;
    const float ry = m.texRepeat[1] > 0 ? m.texRepeat[1] : 1.0f;
    out.uvScale = {rx, ry};
    if (geom.shape == td::GeomShape::Plane && !m.texUniform && geom.size.x > 0 && geom.size.y > 0) {
        // 平面生成器的纹理坐标以米为单位：按平面尺寸归一化，使整个平面重复 texrepeat 次
        out.uvScale = {static_cast<float>(rx / (2.0 * geom.size.x)), static_cast<float>(ry / (2.0 * geom.size.y))};
    }
    return out;
}

}  // namespace terra::viewer
