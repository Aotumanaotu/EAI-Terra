#include "terra/render/mesh.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "terra/render/context.hpp"

namespace terra::render {

namespace {

constexpr float kPi = 3.14159265358979323846f;

void finish(MeshData& m) {
    if (m.vertices.empty()) return;
    m.boundsMin = {m.vertices[0].position[0], m.vertices[0].position[1], m.vertices[0].position[2]};
    m.boundsMax = m.boundsMin;
    for (const MeshVertex& v : m.vertices) {
        for (std::size_t k = 0; k < 3; ++k) {
            m.boundsMin[k] = std::min(m.boundsMin[k], v.position[k]);
            m.boundsMax[k] = std::max(m.boundsMax[k], v.position[k]);
        }
    }
}

std::uint32_t idx(std::size_t i) { return static_cast<std::uint32_t>(i); }

// 一行纬线：极角 theta（自 +Z 量起）与该行的 Z 偏移（胶囊的上下半球）。
struct Row {
    float theta;
    float zOffset;
};

// 按纬线行生成旋转体表面（球、胶囊）。相邻两行之间连成四边形；两极处的退化三角形跳过。
MeshData latLong(float radius, int segments, const std::vector<Row>& rows) {
    MeshData m;
    const auto columns = static_cast<std::size_t>(segments) + 1;
    for (std::size_t i = 0; i < rows.size(); ++i) {
        const float st = std::sin(rows[i].theta);
        const float ct = std::cos(rows[i].theta);
        for (std::size_t j = 0; j < columns; ++j) {
            const float phi = 2.0f * kPi * static_cast<float>(j) / static_cast<float>(segments);
            const float nx = st * std::cos(phi);
            const float ny = st * std::sin(phi);
            const float nz = ct;
            m.vertices.push_back({{radius * nx, radius * ny, radius * nz + rows[i].zOffset},
                                  {nx, ny, nz},
                                  {static_cast<float>(j) / static_cast<float>(segments),
                                   static_cast<float>(i) / static_cast<float>(rows.size() - 1)}});
        }
    }
    const bool topPole = std::sin(rows.front().theta) < 1e-6f;
    const bool bottomPole = std::sin(rows.back().theta) < 1e-6f;
    for (std::size_t i = 0; i + 1 < rows.size(); ++i) {
        for (std::size_t j = 0; j < static_cast<std::size_t>(segments); ++j) {
            const std::size_t a = i * columns + j;  // 本行
            const std::size_t b = a + columns;      // 下一行（更靠 -Z）
            const std::size_t c = b + 1;
            const std::size_t d = a + 1;
            if (!(bottomPole && i + 2 == rows.size())) {
                m.indices.insert(m.indices.end(), {idx(a), idx(b), idx(c)});
            }
            if (!(topPole && i == 0)) {
                m.indices.insert(m.indices.end(), {idx(a), idx(c), idx(d)});
            }
        }
    }
    finish(m);
    return m;
}

}  // namespace

namespace meshes {

MeshData plane(float halfX, float halfY) {
    MeshData m;
    const float n[3] = {0.0f, 0.0f, 1.0f};
    const float xs[4] = {-halfX, halfX, halfX, -halfX};
    const float ys[4] = {-halfY, -halfY, halfY, halfY};
    for (int k = 0; k < 4; ++k) {
        m.vertices.push_back({{xs[k], ys[k], 0.0f}, {n[0], n[1], n[2]}, {xs[k], ys[k]}});
    }
    m.indices = {0, 1, 2, 0, 2, 3};
    finish(m);
    return m;
}

MeshData box(float halfX, float halfY, float halfZ) {
    MeshData m;
    // 每个面：法线 n，面内两个方向 u、v 满足 u × v = n，按 (-u-v)、(+u-v)、(+u+v)、(-u+v) 逆时针排列。
    struct Face {
        float n[3];
        float u[3];
        float v[3];
    };
    const Face faces[6] = {
        {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},   {{-1, 0, 0}, {0, -1, 0}, {0, 0, 1}},
        {{0, 1, 0}, {0, 0, 1}, {1, 0, 0}},   {{0, -1, 0}, {0, 0, -1}, {1, 0, 0}},
        {{0, 0, 1}, {1, 0, 0}, {0, 1, 0}},   {{0, 0, -1}, {-1, 0, 0}, {0, 1, 0}},
    };
    const float half[3] = {halfX, halfY, halfZ};
    const float su[4] = {-1, 1, 1, -1};
    const float sv[4] = {-1, -1, 1, 1};
    for (const Face& f : faces) {
        const auto base = idx(m.vertices.size());
        for (int k = 0; k < 4; ++k) {
            MeshVertex vert{};
            for (std::size_t a = 0; a < 3; ++a) {
                vert.position[a] = (f.n[a] + su[k] * f.u[a] + sv[k] * f.v[a]) * half[a];
                vert.normal[a] = f.n[a];
            }
            vert.uv[0] = 0.5f * (su[k] + 1.0f);
            vert.uv[1] = 0.5f * (sv[k] + 1.0f);
            m.vertices.push_back(vert);
        }
        m.indices.insert(m.indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
    }
    finish(m);
    return m;
}

MeshData sphere(float radius, int segments, int rings) {
    segments = std::max(segments, 3);
    rings = std::max(rings, 2);
    std::vector<Row> rows;
    for (int i = 0; i <= rings; ++i) rows.push_back({kPi * static_cast<float>(i) / static_cast<float>(rings), 0.0f});
    return latLong(radius, segments, rows);
}

MeshData capsule(float radius, float halfLength, int segments, int ringsPerCap) {
    segments = std::max(segments, 3);
    ringsPerCap = std::max(ringsPerCap, 1);
    std::vector<Row> rows;
    // 上半球（极角 0 → π/2，抬高 halfLength），下半球（π/2 → π，降低 halfLength）；两个赤道行之间即圆柱段。
    for (int i = 0; i <= ringsPerCap; ++i) {
        rows.push_back({0.5f * kPi * static_cast<float>(i) / static_cast<float>(ringsPerCap), halfLength});
    }
    for (int i = 0; i <= ringsPerCap; ++i) {
        rows.push_back({0.5f * kPi + 0.5f * kPi * static_cast<float>(i) / static_cast<float>(ringsPerCap), -halfLength});
    }
    return latLong(radius, segments, rows);
}

MeshData cylinder(float radius, float halfLength, int segments) {
    segments = std::max(segments, 3);
    MeshData m;
    const auto n = static_cast<std::size_t>(segments);
    // 侧面：下圈与上圈，各 n + 1 个顶点（接缝处重复，便于 UV）
    for (int ring = 0; ring < 2; ++ring) {
        const float z = ring == 0 ? -halfLength : halfLength;
        for (std::size_t j = 0; j <= n; ++j) {
            const float phi = 2.0f * kPi * static_cast<float>(j) / static_cast<float>(segments);
            const float c = std::cos(phi);
            const float s = std::sin(phi);
            m.vertices.push_back({{radius * c, radius * s, z},
                                  {c, s, 0.0f},
                                  {static_cast<float>(j) / static_cast<float>(segments), static_cast<float>(ring)}});
        }
    }
    for (std::size_t j = 0; j < n; ++j) {
        const std::size_t a = j;          // 下圈
        const std::size_t b = j + 1;
        const std::size_t c = n + 1 + j + 1;  // 上圈
        const std::size_t d = n + 1 + j;
        m.indices.insert(m.indices.end(), {idx(a), idx(b), idx(c), idx(a), idx(c), idx(d)});
    }
    // 上下底面：中心点 + 一圈顶点，法线 ±Z
    for (int cap = 0; cap < 2; ++cap) {
        const float nz = cap == 0 ? 1.0f : -1.0f;
        const float z = nz * halfLength;
        const auto center = idx(m.vertices.size());
        m.vertices.push_back({{0.0f, 0.0f, z}, {0.0f, 0.0f, nz}, {0.5f, 0.5f}});
        for (std::size_t j = 0; j <= n; ++j) {
            const float phi = 2.0f * kPi * static_cast<float>(j) / static_cast<float>(segments);
            const float c = std::cos(phi);
            const float s = std::sin(phi);
            m.vertices.push_back({{radius * c, radius * s, z}, {0.0f, 0.0f, nz}, {0.5f + 0.5f * c, 0.5f + 0.5f * s}});
        }
        for (std::size_t j = 0; j < n; ++j) {
            const auto p = center + 1 + idx(j);
            if (cap == 0) {
                m.indices.insert(m.indices.end(), {center, p, p + 1});  // 从上方看逆时针 → 法线 +Z
            } else {
                m.indices.insert(m.indices.end(), {center, p + 1, p});  // 反向 → 法线 -Z
            }
        }
    }
    finish(m);
    return m;
}

}  // namespace meshes

GpuMesh GpuMesh::create(const MeshData& data) {
    GpuMesh g;
    if (data.vertices.empty() || data.indices.empty()) return g;
    g.vertices_ = Buffer::create(data.vertices.size() * sizeof(MeshVertex), data.vertices.data());
    g.indices_ = Buffer::create(data.indices.size() * sizeof(std::uint32_t), data.indices.data());
    g.vao_ = VertexArray::create();
    g.vao_.setVertexBuffer(0, g.vertices_, 0, sizeof(MeshVertex));
    g.vao_.setFloatAttribute(0, 0, 3, offsetof(MeshVertex, position));
    g.vao_.setFloatAttribute(1, 0, 3, offsetof(MeshVertex, normal));
    g.vao_.setFloatAttribute(2, 0, 2, offsetof(MeshVertex, uv));
    g.vao_.setIndexBuffer(g.indices_);
    g.indexCount_ = static_cast<int>(data.indices.size());
    g.boundsMin_ = data.boundsMin;
    g.boundsMax_ = data.boundsMax;
    return g;
}

void GpuMesh::draw(RenderContext& context) const {
    if (indexCount_ == 0) return;
    vao_.bind();
    context.drawIndexed(indexCount_);
}

}  // namespace terra::render
