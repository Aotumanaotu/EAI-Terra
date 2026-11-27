// glTF 环境导入（M5-02b），说明见 terra/import/environment.hpp。

#include "terra/import/environment.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <set>
#include <unordered_map>

#include <nlohmann/json.hpp>

#include "gltf_reader.hpp"
#include "terra/scene/composition.hpp"

namespace terra::importer {

namespace {

namespace fs = std::filesystem;
namespace g = terra::importer::gltf;
using Json = nlohmann::ordered_json;
using V = g::Vec3d;

// 3×3 矩阵，行主序 m[r][c]
using M3 = std::array<std::array<double, 3>, 3>;

constexpr M3 kC = {{{1, 0, 0}, {0, 0, -1}, {0, 1, 0}}};  // glTF（Y 向上）→ Terra（Z 向上）

V mul(const M3& m, const V& v) {
    return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z, m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z, m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
}
M3 mul(const M3& a, const M3& b) {
    M3 out{};
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 3; ++c) out[r][c] = a[r][0] * b[0][c] + a[r][1] * b[1][c] + a[r][2] * b[2][c];
    }
    return out;
}
M3 transposed(const M3& m) {
    M3 out{};
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 3; ++c) out[r][c] = m[c][r];
    }
    return out;
}
double length(const V& v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

// 列主序 Mat4 的线性部分
M3 linear(const g::Mat4& m) { return {{{m[0], m[4], m[8]}, {m[1], m[5], m[9]}, {m[2], m[6], m[10]}}}; }

// 旋转矩阵 → 四元数 (w, x, y, z)
std::array<double, 4> quaternion(const M3& r) {
    std::array<double, 4> q{};
    const double trace = r[0][0] + r[1][1] + r[2][2];
    if (trace > 0) {
        const double s = 0.5 / std::sqrt(trace + 1.0);
        q = {0.25 / s, (r[2][1] - r[1][2]) * s, (r[0][2] - r[2][0]) * s, (r[1][0] - r[0][1]) * s};
    } else if (r[0][0] > r[1][1] && r[0][0] > r[2][2]) {
        const double s = 2.0 * std::sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]);
        q = {(r[2][1] - r[1][2]) / s, 0.25 * s, (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s};
    } else if (r[1][1] > r[2][2]) {
        const double s = 2.0 * std::sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]);
        q = {(r[0][2] - r[2][0]) / s, (r[0][1] + r[1][0]) / s, 0.25 * s, (r[1][2] + r[2][1]) / s};
    } else {
        const double s = 2.0 * std::sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]);
        q = {(r[1][0] - r[0][1]) / s, (r[0][2] + r[2][0]) / s, (r[1][2] + r[2][1]) / s, 0.25 * s};
    }
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (double& c : q) c /= n;
    if (q[0] < 0) {
        for (double& c : q) c = -c;
    }
    return q;
}

// 节点世界矩阵的分解：旋转（单位正交）、平移、各轴缩放
struct Rigid {
    M3 rotation{};
    V translation;
    V scale{1, 1, 1};
};
Rigid decompose(const g::Mat4& m) {
    Rigid out;
    const M3 l = linear(m);
    double s[3];
    for (std::size_t c = 0; c < 3; ++c) {
        s[c] = std::sqrt(l[0][c] * l[0][c] + l[1][c] * l[1][c] + l[2][c] * l[2][c]);
        if (!(s[c] > 1e-12)) s[c] = 1.0;
        for (std::size_t r = 0; r < 3; ++r) out.rotation[r][c] = l[r][c] / s[c];
    }
    // 镜像（行列式为负）：翻转 z 轴，缩放取负
    const M3& R = out.rotation;
    const double det = R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1]) - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0]) +
                       R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]);
    if (det < 0) {
        for (std::size_t r = 0; r < 3; ++r) out.rotation[r][2] = -out.rotation[r][2];
        s[2] = -s[2];
    }
    out.scale = {s[0], s[1], s[2]};
    out.translation = {m[12], m[13], m[14]};
    return out;
}
g::Mat4 rigidMatrix(const Rigid& r) {
    g::Mat4 m = g::identity();
    for (std::size_t c = 0; c < 3; ++c) {
        for (std::size_t row = 0; row < 3; ++row) m[c * 4 + row] = r.rotation[row][c];
    }
    m[12] = r.translation.x;
    m[13] = r.translation.y;
    m[14] = r.translation.z;
    return m;
}

struct Box {
    V center;
    M3 rotation{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};  // 列为盒子的轴（Terra 的部件坐标系中）
    V half;
    double volume() const { return 8.0 * half.x * half.y * half.z; }
};

constexpr double kMinHalf = 0.005;

V clampHalf(V h) { return {std::max(h.x, kMinHalf), std::max(h.y, kMinHalf), std::max(h.z, kMinHalf)}; }

// 点集在给定轴（列）下的包围盒
Box boxAlong(const std::vector<V>& points, const M3& axes) {
    V lo{1e300, 1e300, 1e300}, hi{-1e300, -1e300, -1e300};
    const M3 t = transposed(axes);
    for (const V& p : points) {
        const V q = mul(t, p);
        lo = {std::min(lo.x, q.x), std::min(lo.y, q.y), std::min(lo.z, q.z)};
        hi = {std::max(hi.x, q.x), std::max(hi.y, q.y), std::max(hi.z, q.z)};
    }
    Box b;
    b.rotation = axes;
    b.half = clampHalf({0.5 * (hi.x - lo.x), 0.5 * (hi.y - lo.y), 0.5 * (hi.z - lo.z)});
    b.center = mul(axes, V{0.5 * (hi.x + lo.x), 0.5 * (hi.y + lo.y), 0.5 * (hi.z + lo.z)});
    return b;
}

// 对称 3×3 矩阵的特征向量（Jacobi 迭代）；返回的列为特征向量，保证右手系
M3 eigenvectors(M3 a) {
    M3 v{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    for (int sweep = 0; sweep < 50; ++sweep) {
        const double off = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
        if (off < 1e-24) break;
        for (std::size_t p = 0; p < 2; ++p) {
            for (std::size_t q = p + 1; q < 3; ++q) {
                if (std::fabs(a[p][q]) < 1e-300) continue;
                const double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                const double t = (theta >= 0 ? 1.0 : -1.0) / (std::fabs(theta) + std::sqrt(theta * theta + 1.0));
                const double c = 1.0 / std::sqrt(t * t + 1.0), s = t * c;
                for (std::size_t k = 0; k < 3; ++k) {  // A ← Jᵀ A J
                    const double akp = a[k][p], akq = a[k][q];
                    a[k][p] = c * akp - s * akq;
                    a[k][q] = s * akp + c * akq;
                }
                for (std::size_t k = 0; k < 3; ++k) {
                    const double apk = a[p][k], aqk = a[q][k];
                    a[p][k] = c * apk - s * aqk;
                    a[q][k] = s * apk + c * aqk;
                }
                for (std::size_t k = 0; k < 3; ++k) {
                    const double vkp = v[k][p], vkq = v[k][q];
                    v[k][p] = c * vkp - s * vkq;
                    v[k][q] = s * vkp + c * vkq;
                }
            }
        }
    }
    // 右手系：第三列 = 第一列 × 第二列
    const V x{v[0][0], v[1][0], v[2][0]}, y{v[0][1], v[1][1], v[2][1]};
    const V z{x.y * y.z - x.z * y.y, x.z * y.x - x.x * y.z, x.x * y.y - x.y * y.x};
    v[0][2] = z.x;
    v[1][2] = z.y;
    v[2][2] = z.z;
    return v;
}

Box pcaBox(const std::vector<V>& points) {
    V mean{0, 0, 0};
    for (const V& p : points) mean = {mean.x + p.x, mean.y + p.y, mean.z + p.z};
    const double n = static_cast<double>(std::max<std::size_t>(points.size(), 1));
    mean = {mean.x / n, mean.y / n, mean.z / n};
    M3 cov{};
    for (const V& p : points) {
        const double d[3] = {p.x - mean.x, p.y - mean.y, p.z - mean.z};
        for (std::size_t r = 0; r < 3; ++r) {
            for (std::size_t c = 0; c < 3; ++c) cov[r][c] += d[r] * d[c];
        }
    }
    return boxAlong(points, eigenvectors(cov));
}

// 一个网格图元按共享顶点（索引与位置都算）拆成互不相连的部分：每部分为它的三角形（在 index 中的起始下标）
std::vector<std::vector<std::size_t>> components(const std::vector<V>& positions, const std::vector<std::uint32_t>& index) {
    std::vector<std::size_t> parent(positions.size());
    for (std::size_t i = 0; i < parent.size(); ++i) parent[i] = i;
    const auto find = [&](std::size_t x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    };
    const auto unite = [&](std::size_t a, std::size_t b) {
        a = find(a);
        b = find(b);
        if (a != b) parent[std::max(a, b)] = std::min(a, b);
    };
    // 位置相同（量化到 0.01 mm）的顶点视为同一点（不同法线、纹理坐标的拆分顶点）
    struct Key {
        std::int64_t x, y, z;
        bool operator==(const Key& o) const { return x == o.x && y == o.y && z == o.z; }
    };
    struct KeyHash {
        std::size_t operator()(const Key& k) const {
            std::uint64_t h = 1469598103934665603ull;
            for (const std::int64_t v : {k.x, k.y, k.z}) {
                h ^= static_cast<std::uint64_t>(v);
                h *= 1099511628211ull;
                h ^= h >> 29;
            }
            return static_cast<std::size_t>(h);
        }
    };
    std::unordered_map<Key, std::size_t, KeyHash> seen;
    seen.reserve(positions.size());
    for (std::size_t i = 0; i < positions.size(); ++i) {
        const Key key{std::llround(positions[i].x * 1e5), std::llround(positions[i].y * 1e5), std::llround(positions[i].z * 1e5)};
        const auto [it, inserted] = seen.emplace(key, i);
        if (!inserted) unite(i, it->second);
    }
    for (std::size_t t = 0; t + 2 < index.size(); t += 3) {
        const std::size_t a = index[t], b = index[t + 1], c = index[t + 2];
        if (a >= positions.size() || b >= positions.size() || c >= positions.size()) continue;
        unite(a, b);
        unite(a, c);
    }
    std::map<std::size_t, std::vector<std::size_t>> groups;
    for (std::size_t t = 0; t + 2 < index.size(); t += 3) {
        if (index[t] >= positions.size() || index[t + 1] >= positions.size() || index[t + 2] >= positions.size()) continue;
        groups[find(index[t])].push_back(t);
    }
    std::vector<std::vector<std::size_t>> out;
    out.reserve(groups.size());
    for (auto& [root, tris] : groups) {
        (void)root;
        out.push_back(std::move(tris));
    }
    return out;
}

// 一个部分的碰撞盒（2026-11-26）：先取三种包围盒中体积最小的一个；若三角形的总面积不到盒子表面积的一半（盒子里大半是空的，
// 如门柱与门槛连成的 П 形门框——整个门洞会被堵上），就沿最长的方向在三角形重心的中位数处切开，两半分别再算（最多 8 层）。
// 实心的块（踏步、柜子、柱子）面积与盒子的表面积接近，不切；很薄的面片（单面的墙、地面）按一半的表面积比较
void fitBoxes(const std::vector<V>& pts, const std::vector<std::uint32_t>& idx, std::vector<std::size_t> tris, const M3& meshAxes, int depth,
              int maxDepth, std::vector<Box>& out) {
    std::vector<V> p;
    p.reserve(tris.size() * 3);
    double area = 0;
    for (const std::size_t t : tris) {
        const V& a = pts[idx[t]];
        const V& b = pts[idx[t + 1]];
        const V& c = pts[idx[t + 2]];
        p.push_back(a);
        p.push_back(b);
        p.push_back(c);
        const V u{b.x - a.x, b.y - a.y, b.z - a.z}, w{c.x - a.x, c.y - a.y, c.z - a.z};
        area += 0.5 * length(V{u.y * w.z - u.z * w.y, u.z * w.x - u.x * w.z, u.x * w.y - u.y * w.x});
    }
    if (p.empty()) return;
    const M3 ownerAxes{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    Box best = boxAlong(p, ownerAxes);
    for (const Box& candidate : {boxAlong(p, meshAxes), pcaBox(p)}) {
        if (candidate.volume() < best.volume() * 0.98) best = candidate;
    }
    const double dx = 2 * best.half.x, dy = 2 * best.half.y, dz = 2 * best.half.z;
    const double surface = 2 * (dx * dy + dy * dz + dz * dx);
    const double thinnest = std::min({dx, dy, dz}), longest = std::max({dx, dy, dz});
    const double ratio = surface > 0 ? area / surface : 1.0;
    const bool full = ratio >= 0.5 || (thinnest < 0.025 && ratio >= 0.25);
    // 只剩一个三角形时不能再切；两个三角形可能分属门洞两侧，仍要切开
    if (full || tris.size() < 2 || depth >= maxDepth || longest < 0.1) {
        out.push_back(best);
        return;
    }
    // 沿轴对齐范围最长的方向，按三角形重心的中位数分成两半
    V lo{1e300, 1e300, 1e300}, hi{-1e300, -1e300, -1e300};
    for (const V& q : p) {
        lo = {std::min(lo.x, q.x), std::min(lo.y, q.y), std::min(lo.z, q.z)};
        hi = {std::max(hi.x, q.x), std::max(hi.y, q.y), std::max(hi.z, q.z)};
    }
    const int axis = (hi.x - lo.x >= hi.y - lo.y && hi.x - lo.x >= hi.z - lo.z) ? 0 : (hi.y - lo.y >= hi.z - lo.z ? 1 : 2);
    const auto centroid = [&](std::size_t t) {
        const V& a = pts[idx[t]];
        const V& b = pts[idx[t + 1]];
        const V& c = pts[idx[t + 2]];
        return axis == 0 ? a.x + b.x + c.x : axis == 1 ? a.y + b.y + c.y : a.z + b.z + c.z;
    };
    const auto mid = tris.begin() + static_cast<std::ptrdiff_t>(tris.size() / 2);
    std::nth_element(tris.begin(), mid, tris.end(), [&](std::size_t a, std::size_t b) {
        const double ca = centroid(a), cb = centroid(b);
        return ca < cb || (ca == cb && a < b);
    });
    std::vector<std::size_t> left(tris.begin(), mid), right(mid, tris.end());
    if (left.empty() || right.empty()) {
        out.push_back(best);
        return;
    }
    fitBoxes(pts, idx, std::move(left), meshAxes, depth + 1, maxDepth, out);
    fitBoxes(pts, idx, std::move(right), meshAxes, depth + 1, maxDepth, out);
}

// 轴对齐盒（部件坐标系）两两合并到不超过 limit 个：每次取合并后体积增加最少的一对
std::vector<Box> mergeBoxes(std::vector<Box> boxes, std::size_t limit) {
    struct Aabb {
        V lo, hi;
    };
    std::vector<Aabb> a;
    for (const Box& b : boxes) {
        // 盒子 8 个角的轴对齐范围
        Aabb x{{1e300, 1e300, 1e300}, {-1e300, -1e300, -1e300}};
        for (int k = 0; k < 8; ++k) {
            const V local{(k & 1 ? 1 : -1) * b.half.x, (k & 2 ? 1 : -1) * b.half.y, (k & 4 ? 1 : -1) * b.half.z};
            const V r = mul(b.rotation, local);
            const V p{b.center.x + r.x, b.center.y + r.y, b.center.z + r.z};
            x.lo = {std::min(x.lo.x, p.x), std::min(x.lo.y, p.y), std::min(x.lo.z, p.z)};
            x.hi = {std::max(x.hi.x, p.x), std::max(x.hi.y, p.y), std::max(x.hi.z, p.z)};
        }
        a.push_back(x);
    }
    const auto vol = [](const Aabb& x) { return (x.hi.x - x.lo.x) * (x.hi.y - x.lo.y) * (x.hi.z - x.lo.z); };
    const auto join = [](const Aabb& x, const Aabb& y) {
        return Aabb{{std::min(x.lo.x, y.lo.x), std::min(x.lo.y, y.lo.y), std::min(x.lo.z, y.lo.z)},
                    {std::max(x.hi.x, y.hi.x), std::max(x.hi.y, y.hi.y), std::max(x.hi.z, y.hi.z)}};
    };
    while (a.size() > limit) {
        std::size_t bi = 0, bj = 1;
        double best = 1e300;
        for (std::size_t i = 0; i < a.size(); ++i) {
            for (std::size_t j = i + 1; j < a.size(); ++j) {
                const double grow = vol(join(a[i], a[j])) - vol(a[i]) - vol(a[j]);
                if (grow < best) {
                    best = grow;
                    bi = i;
                    bj = j;
                }
            }
        }
        a[bi] = join(a[bi], a[bj]);
        a.erase(a.begin() + static_cast<std::ptrdiff_t>(bj));
    }
    std::vector<Box> out;
    for (const Aabb& x : a) {
        Box b;
        b.center = {0.5 * (x.lo.x + x.hi.x), 0.5 * (x.lo.y + x.hi.y), 0.5 * (x.lo.z + x.hi.z)};
        b.half = clampHalf({0.5 * (x.hi.x - x.lo.x), 0.5 * (x.hi.y - x.lo.y), 0.5 * (x.hi.z - x.lo.z)});
        out.push_back(b);
    }
    return out;
}

Json vec3(const V& v) { return Json::array({v.x, v.y, v.z}); }
Json quatJson(const M3& r) {
    const auto q = quaternion(r);
    return Json{{"quat", Json::array({q[0], q[1], q[2], q[3]})}};
}

bool identityRotation(const M3& r) {
    return std::fabs(r[0][0] - 1) < 1e-9 && std::fabs(r[1][1] - 1) < 1e-9 && std::fabs(r[2][2] - 1) < 1e-9;
}

Json boxJson(const Box& b) {
    Json j;
    j["position"] = vec3(b.center);
    if (!identityRotation(b.rotation)) j["rotation"] = quatJson(b.rotation);
    j["halfSize"] = vec3(b.half);
    return j;
}

std::string sanitize(const std::string& name) {
    std::string out;
    for (const char c : name) {
        const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-';
        out += ok ? c : '_';
    }
    if (out.empty() || !((out[0] >= 'A' && out[0] <= 'Z') || (out[0] >= 'a' && out[0] <= 'z') || out[0] == '_')) out = "n_" + out;
    return out;
}

bool contains(const std::string& haystack, const std::string& needle) {
    std::string a = haystack, b = needle;
    for (char& c : a) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    for (char& c : b) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return !b.empty() && a.find(b) != std::string::npos;
}

}  // namespace

EnvironmentImportResult importEnvironment(const EnvironmentImportOptions& options) {
    EnvironmentImportResult result;
    const auto fail = [&](std::string message) {
        result.errors.push_back(std::move(message));
        return result;
    };
    std::error_code ec;
    const fs::path input = fs::absolute(options.input, ec).lexically_normal();
    g::Document doc;
    std::string error;
    if (!g::read(input.string(), doc, error)) return fail(error);
    if (doc.roots.empty()) return fail("glTF 中没有节点：" + input.string());

    // 清单：给出的，或 glTF 所在目录及其上一级的 manifest.json
    std::map<std::string, Json> manifest;
    {
        std::vector<fs::path> candidates;
        if (!options.manifest.empty()) candidates.push_back(options.manifest);
        else candidates = {input.parent_path() / "manifest.json", input.parent_path().parent_path() / "manifest.json"};
        for (const fs::path& c : candidates) {
            if (!fs::is_regular_file(c, ec)) {
                if (!options.manifest.empty()) return fail("清单不存在：" + c.string());
                continue;
            }
            std::ifstream in(c, std::ios::binary);
            const Json m = Json::parse(in, nullptr, false);
            if (m.is_discarded() || !m.is_object() || !m.contains("elements") || !m["elements"].is_array()) {
                result.warnings.push_back("清单格式不对（需要 elements 数组），忽略：" + c.string());
                continue;
            }
            std::size_t matched = 0;
            for (const Json& e : m["elements"]) {
                if (!e.is_object() || !e.contains("id") || !e["id"].is_string()) continue;
                const std::string id = e["id"].get<std::string>();
                manifest[id] = e;
                matched += g::findNode(doc, id) >= 0 ? 1u : 0u;
            }
            if (matched == 0) {
                manifest.clear();
                result.warnings.push_back("清单中的元素与 glTF 的节点都对不上，忽略：" + c.string());
                continue;
            }
            result.manifest = fs::absolute(c, ec).lexically_normal().string();
            break;
        }
    }

    const fs::path out = fs::absolute(options.output, ec).lexically_normal();
    const std::string name = options.name.empty() ? sanitize(input.stem().string()) : options.name;
    const fs::path scenePath = out / (name + ".terra-scene.json");
    if (fs::exists(scenePath, ec) && !options.force) return fail("输出目录中已有场景：" + scenePath.string() + "（加 --force 覆盖）");
    const fs::path elementsDir = out / "elements";
    fs::create_directories(elementsDir / "meshes", ec);
    if (ec) return fail("无法创建输出目录：" + elementsDir.string());
    const std::string gltfRel = fs::relative(input, elementsDir, ec).string();

    Json sceneElements = Json::array();
    Json report = Json::array();
    std::set<std::string> usedIds;
    V sceneLo{1e300, 1e300, 1e300}, sceneHi{-1e300, -1e300, -1e300};

    for (const int rootIndex : doc.roots) {
        const g::Node& root = doc.nodes[static_cast<std::size_t>(rootIndex)];
        std::string id = sanitize(root.name.empty() ? "node_" + std::to_string(rootIndex) : root.name);
        while (!usedIds.insert(id).second) id += "_";
        const Json info = manifest.count(root.name) ? manifest[root.name] : Json::object();
        const auto text = [&](const Json& o, const char* key, const std::string& fallback) {
            return o.is_object() && o.contains(key) && o[key].is_string() ? o[key].get<std::string>() : fallback;
        };
        const std::string category = text(info, "category", "environment");
        std::string movable = text(info, "movable", "fixed");
        if (movable != "layout" && movable != "fixed" && movable != "dynamic") {
            result.warnings.push_back(id + "：未知的 movable “" + movable + "”，按 fixed");
            movable = "fixed";
        }
        const double mass = info.is_object() && info.contains("mass") && info["mass"].is_number() ? info["mass"].get<double>() : 0.0;
        if (movable == "dynamic" && !(mass > 0)) {
            result.warnings.push_back(id + "：dynamic 但清单没有质量，按 layout");
            movable = "layout";
        }
        const bool dynamic = movable == "dynamic";
        const bool hulls = options.hulls || std::any_of(options.hullPatterns.begin(), options.hullPatterns.end(),
                                                        [&](const std::string& p) { return contains(category, p) || contains(id, p); });

        // 元素坐标系（glTF）：根节点的旋转与平移；统一缩放进外观与碰撞体
        const Rigid rootRigid = decompose(root.world);
        const double rootScale = (std::fabs(rootRigid.scale.x) + std::fabs(rootRigid.scale.y) + std::fabs(rootRigid.scale.z)) / 3.0;
        if (std::fabs(std::fabs(rootRigid.scale.x) - rootScale) > 1e-3 * rootScale || std::fabs(std::fabs(rootRigid.scale.z) - rootScale) > 1e-3 * rootScale) {
            result.warnings.push_back(id + "：节点的缩放不是统一缩放，外观按平均缩放 " + std::to_string(rootScale));
        }
        const g::Mat4 elementFrame = rigidMatrix(rootRigid);
        const g::Mat4 elementInverse = g::inverseAffine(elementFrame);

        // 部件：清单 thermal.parts 中的节点（dynamic 元素不拆部件）
        std::map<int, std::pair<std::string, double>> partNodes;  // 节点下标 → (部件名, 温度)
        if (info.is_object() && info.contains("thermal") && info["thermal"].is_object() && info["thermal"].contains("parts") &&
            info["thermal"]["parts"].is_object()) {
            for (auto it = info["thermal"]["parts"].begin(); it != info["thermal"]["parts"].end(); ++it) {
                const int n = g::findNode(doc, it.key());
                if (n < 0 || !g::inSubtree(doc, n, rootIndex) || n == rootIndex || !it.value().is_number()) {
                    result.warnings.push_back(id + "：部件 “" + it.key() + "” 不是该节点的子节点，忽略");
                    continue;
                }
                if (dynamic) {
                    result.warnings.push_back(id + "：dynamic 元素不拆部件，部件 “" + it.key() + "” 的温度忽略");
                    continue;
                }
                partNodes[n] = {sanitize(it.key()), it.value().get<double>()};
            }
        }
        // 每个网格节点归属的“拥有者”：最近的部件祖先，没有时为元素本身（-1）
        const auto ownerOf = [&](int node) {
            for (int n = node; n >= 0 && n != rootIndex; n = doc.nodes[static_cast<std::size_t>(n)].parent) {
                if (partNodes.count(n)) return n;
            }
            return -1;
        };
        // 拥有者坐标系（glTF，不含缩放）之逆
        std::map<int, g::Mat4> ownerInverse;
        ownerInverse[-1] = elementInverse;
        for (const auto& [n, p] : partNodes) {
            (void)p;
            ownerInverse[n] = g::inverseAffine(rigidMatrix(decompose(doc.nodes[static_cast<std::size_t>(n)].world)));
        }

        std::map<int, Json> ownerBoxes, ownerMeshes;
        ownerBoxes[-1] = Json::array();
        ownerMeshes[-1] = Json::array();
        for (const auto& [n, p] : partNodes) {
            (void)p;
            ownerBoxes[n] = Json::array();
            ownerMeshes[n] = Json::array();
        }
        std::size_t boxCount = 0, meshCount = 0;
        V lo{1e300, 1e300, 1e300}, hi{-1e300, -1e300, -1e300};  // 元素的世界包围盒（Terra）
        std::map<int, std::pair<V, V>> nodeBounds;                // 节点网格在元素坐标系（Terra）中的包围盒（检查点用）

        for (std::size_t ni = 0; ni < doc.nodes.size(); ++ni) {
            const int nodeIndex = static_cast<int>(ni);
            const g::Node& node = doc.nodes[ni];
            if (node.mesh < 0 || static_cast<std::size_t>(node.mesh) >= doc.meshes.size() || !g::inSubtree(doc, nodeIndex, rootIndex)) continue;
            const int owner = ownerOf(nodeIndex);
            const g::Mat4 rel = g::multiply(ownerInverse[owner], node.world);   // 网格坐标 → 拥有者坐标（glTF）
            const g::Mat4 toElement = g::multiply(elementInverse, node.world);  // 网格坐标 → 元素坐标（glTF）
            const M3 relLinear = linear(rel);
            std::vector<Box> nodeBoxes;
            struct HullMesh {
                std::vector<V> points;
                std::vector<std::array<std::size_t, 3>> faces;
            };
            std::vector<HullMesh> nodeHulls;
            for (const g::Primitive& prim : doc.meshes[static_cast<std::size_t>(node.mesh)]) {
                if (prim.mode != 4) continue;
                std::string perr;
                const std::vector<V> pos = g::positions(doc, prim.positions, perr);
                if (pos.empty()) {
                    result.warnings.push_back(id + "/" + node.name + "：" + perr);
                    continue;
                }
                std::vector<std::uint32_t> idx;
                if (prim.indices >= 0) {
                    idx = g::indices(doc, prim.indices, perr);
                } else {
                    idx.resize(pos.size());
                    for (std::size_t k = 0; k < pos.size(); ++k) idx[k] = static_cast<std::uint32_t>(k);
                }
                // 包围盒（元素 / 世界，Terra）
                for (const V& p : pos) {
                    const V e = mul(kC, g::transformPoint(toElement, p));
                    auto& nb = nodeBounds.try_emplace(nodeIndex, V{1e300, 1e300, 1e300}, V{-1e300, -1e300, -1e300}).first->second;
                    nb.first = {std::min(nb.first.x, e.x), std::min(nb.first.y, e.y), std::min(nb.first.z, e.z)};
                    nb.second = {std::max(nb.second.x, e.x), std::max(nb.second.y, e.y), std::max(nb.second.z, e.z)};
                    const V w = mul(kC, g::transformPoint(node.world, p));
                    lo = {std::min(lo.x, w.x), std::min(lo.y, w.y), std::min(lo.z, w.z)};
                    hi = {std::max(hi.x, w.x), std::max(hi.y, w.y), std::max(hi.z, w.z)};
                }
                // 拥有者坐标系（Terra）中的顶点
                std::vector<V> tp(pos.size());
                for (std::size_t k = 0; k < pos.size(); ++k) tp[k] = mul(kC, g::transformPoint(rel, pos[k]));
                // 拆成互不相连的部分
                const auto parts = components(pos, idx);
                if (hulls) {  // 每部分一个凸包（坡道的坡面、平台与两侧路沿分开，坡面才是准确的斜面）
                    for (const auto& part : parts) {
                        std::map<std::uint32_t, std::size_t> remap;
                        HullMesh mesh;
                        for (const std::size_t t : part) {
                            std::array<std::size_t, 3> face{};
                            for (std::size_t k = 0; k < 3; ++k) {
                                const std::uint32_t v = idx[t + k];
                                const auto [it, inserted] = remap.emplace(v, mesh.points.size());
                                if (inserted) mesh.points.push_back(tp[v]);
                                face[k] = it->second;
                            }
                            mesh.faces.push_back(face);
                        }
                        nodeHulls.push_back(std::move(mesh));
                    }
                    continue;
                }
                if (parts.size() > 512) {  // 太碎（如植物叶片）：整个图元一个主方向盒
                    nodeBoxes.push_back(pcaBox(tp));
                    continue;
                }
                // 网格自身坐标系的轴在拥有者坐标系（Terra）中的方向
                M3 meshAxes{};
                for (std::size_t c = 0; c < 3; ++c) {
                    const V col = mul(kC, V{relLinear[0][c], relLinear[1][c], relLinear[2][c]});
                    const double len = length(col);
                    for (std::size_t r = 0; r < 3; ++r) meshAxes[r][c] = (r == 0 ? col.x : r == 1 ? col.y : col.z) / (len > 0 ? len : 1.0);
                }
                {  // 保证右手系
                    const V x{meshAxes[0][0], meshAxes[1][0], meshAxes[2][0]}, y{meshAxes[0][1], meshAxes[1][1], meshAxes[2][1]};
                    const V z{x.y * y.z - x.z * y.y, x.z * y.x - x.x * y.z, x.x * y.y - x.y * y.x};
                    meshAxes[0][2] = z.x;
                    meshAxes[1][2] = z.y;
                    meshAxes[2][2] = z.z;
                }
                // 空心的部分切开（最多 8 层）；一个图元切出的盒体太多（超过上限的 4 倍，是精细的外形而不是空心的框，
                // 如 Poly Haven 的空调外机、消火栓）时不切，每部分一个盒体，避免合并太慢
                std::vector<Box> primBoxes;
                for (const auto& part : parts) fitBoxes(tp, idx, part, meshAxes, 0, 8, primBoxes);
                if (primBoxes.size() > 4 * static_cast<std::size_t>(std::max(1, options.maxBoxesPerNode))) {
                    primBoxes.clear();
                    for (const auto& part : parts) fitBoxes(tp, idx, part, meshAxes, 0, 0, primBoxes);
                }
                nodeBoxes.insert(nodeBoxes.end(), primBoxes.begin(), primBoxes.end());
            }
            for (std::size_t j = 0; j < nodeHulls.size(); ++j) {
                const HullMesh& hull = nodeHulls[j];
                if (hull.points.size() < 4) continue;  // 凸包至少要 4 个点
                const std::string file = sanitize(id + "__" + node.name) + (nodeHulls.size() > 1 ? "__" + std::to_string(j) : std::string{}) + ".obj";
                std::ofstream obj(elementsDir / "meshes" / file, std::ios::binary);
                obj << "# EAI Terra 环境导入（M5-02b）：" << id << " / " << node.name << " 的碰撞网格（部件坐标系，Z 向上，米）\n";
                char line[128];
                for (const V& q : hull.points) {
                    std::snprintf(line, sizeof line, "v %.6f %.6f %.6f\n", q.x, q.y, q.z);
                    obj << line;
                }
                for (const auto& f : hull.faces) obj << "f " << f[0] + 1 << " " << f[1] + 1 << " " << f[2] + 1 << "\n";
                if (!obj) return fail("无法写入碰撞网格：" + (elementsDir / "meshes" / file).string());
                ownerMeshes[owner].push_back(Json{{"file", "meshes/" + file}, {"node", node.name}});
                ++meshCount;
            }
            if (nodeBoxes.size() > static_cast<std::size_t>(std::max(1, options.maxBoxesPerNode))) {
                nodeBoxes = mergeBoxes(nodeBoxes, static_cast<std::size_t>(std::max(1, options.maxBoxesPerNode)));
            }
            for (const Box& b : nodeBoxes) {
                Json j = boxJson(b);
                j["node"] = node.name;
                ownerBoxes[owner].push_back(j);
                ++boxCount;
            }
        }

        // 元素文件
        Json e;
        e["format"] = "terra.element";
        e["version"] = 1;
        e["name"] = text(info, "name", root.name);
        e["category"] = category;
        e["visual"] = Json{{"gltf", gltfRel}, {"node", root.name}};
        if (std::fabs(rootScale - 1.0) > 1e-9) e["visual"]["scale"] = rootScale;
        const bool any = !ownerBoxes[-1].empty() || !ownerMeshes[-1].empty() ||
                         std::any_of(partNodes.begin(), partNodes.end(), [&](const auto& kv) { return !ownerBoxes[kv.first].empty() || !ownerMeshes[kv.first].empty(); });
        if (!any) {
            e["collision"] = Json{{"mode", "none"}};
        } else if (hulls) {
            e["collision"] = Json{{"mode", "mesh"}, {"meshes", ownerMeshes[-1]}, {"generatedBy", "auto-hulls（terra_import --environment）"}};
        } else {
            e["collision"] = Json{{"mode", "boxes"}, {"boxes", ownerBoxes[-1]}, {"generatedBy", "auto-boxes（terra_import --environment）"}};
        }
        if (dynamic && !any) {
            result.warnings.push_back(id + "：dynamic 但没有网格，按 layout");
            movable = "layout";
        }
        e["physics"] = Json{{"movable", movable}};
        if (movable == "dynamic") e["physics"]["mass"] = mass;
        if (!partNodes.empty()) {
            Json parts = Json::array();
            for (const auto& [n, p] : partNodes) {
                const g::Node& pn = doc.nodes[static_cast<std::size_t>(n)];
                const Rigid pr = decompose(pn.world);
                const Rigid relRigid = decompose(g::multiply(elementInverse, rigidMatrix(pr)));
                Json part;
                part["name"] = p.first;
                part["node"] = pn.name;
                part["position"] = vec3(mul(kC, relRigid.translation));
                const M3 r = mul(mul(kC, relRigid.rotation), transposed(kC));
                if (!identityRotation(r)) part["rotation"] = quatJson(r);
                const double ps = (std::fabs(pr.scale.x) + std::fabs(pr.scale.y) + std::fabs(pr.scale.z)) / 3.0;
                if (std::fabs(ps - 1.0) > 1e-9) part["visualScale"] = ps;
                if (hulls) part["meshes"] = ownerMeshes[n];
                else part["boxes"] = ownerBoxes[n];
                part["temperature"] = p.second;
                parts.push_back(part);
            }
            e["parts"] = parts;
            result.parts += partNodes.size();
        }
        if (info.is_object() && info.contains("thermal") && info["thermal"].is_object() && info["thermal"].contains("default") &&
            info["thermal"]["default"].is_number()) {
            e["thermal"] = Json{{"default", info["thermal"]["default"]}};
        }
        if (info.is_object() && info.contains("inspection") && info["inspection"].is_array()) {
            Json points = Json::array();
            for (const Json& item : info["inspection"]) {
                if (!item.is_object()) continue;
                Json point;
                point["name"] = text(item, "name", "检查点");
                const std::string nodeName = text(item, "node", "");
                const int n = nodeName.empty() ? -1 : g::findNode(doc, nodeName);
                V at{0, 0, 0};
                if (n >= 0 && nodeBounds.count(n)) {
                    const auto& [a, b] = nodeBounds[n];
                    at = {0.5 * (a.x + b.x), 0.5 * (a.y + b.y), 0.5 * (a.z + b.z)};
                } else if (n >= 0) {
                    at = mul(kC, g::transformPoint(g::multiply(elementInverse, doc.nodes[static_cast<std::size_t>(n)].world), V{0, 0, 0}));
                } else if (!nodeName.empty()) {
                    result.warnings.push_back(id + "：检查点的节点 “" + nodeName + "” 不存在，放在元素原点");
                }
                if (!nodeName.empty()) point["node"] = nodeName;
                point["position"] = vec3(at);
                if (item.contains("view") && item["view"].is_object()) {
                    if (item["view"].contains("distance") && item["view"]["distance"].is_number()) point["distance"] = item["view"]["distance"];
                    if (item["view"].contains("heightDeg") && item["view"]["heightDeg"].is_number()) point["heightDeg"] = item["view"]["heightDeg"];
                }
                points.push_back(point);
                ++result.inspection;
            }
            if (!points.empty()) e["inspection"] = points;
        }
        if (info.is_object() && info.contains("license") && info["license"].is_object()) e["license"] = info["license"];

        const fs::path elementFile = elementsDir / (id + ".terra-element.json");
        std::ofstream file(elementFile, std::ios::binary);
        file << e.dump(1) << "\n";
        if (!file) return fail("无法写入元素文件：" + elementFile.string());

        // 场景中的实例：放置位姿 = 节点的旋转与平移（Terra）
        Json inst;
        inst["id"] = id;
        inst["element"] = "elements/" + id + ".terra-element.json";
        inst["position"] = vec3(mul(kC, rootRigid.translation));
        const M3 placement = mul(mul(kC, rootRigid.rotation), transposed(kC));
        if (!identityRotation(placement)) inst["rotation"] = quatJson(placement);
        sceneElements.push_back(inst);
        report.push_back(Json{{"id", id}, {"node", root.name}, {"category", category}, {"movable", movable}, {"boxes", boxCount},
                              {"meshes", meshCount}, {"parts", partNodes.size()}, {"collision", hulls ? "hulls" : "boxes"}});
        result.boxes += boxCount;
        result.meshes += meshCount;
        ++result.elements;
        if (lo.x <= hi.x) {
            sceneLo = {std::min(sceneLo.x, lo.x), std::min(sceneLo.y, lo.y), std::min(sceneLo.z, lo.z)};
            sceneHi = {std::max(sceneHi.x, hi.x), std::max(sceneHi.y, hi.y), std::max(sceneHi.z, hi.z)};
        }
    }

    // 场景文件
    Json scene;
    scene["format"] = "terra.scene";
    scene["version"] = 1;
    std::string description = input.filename().string();
    if (!result.manifest.empty()) {
        std::ifstream in(result.manifest, std::ios::binary);
        const Json m = Json::parse(in, nullptr, false);
        if (m.is_object() && m.contains("description") && m["description"].is_string()) description = m["description"].get<std::string>();
    }
    scene["name"] = "环境（M5-02b 导入）：" + description;
    scene["title"] = name;
    scene["models"] = Json::array();
    scene["elements"] = sceneElements;
    if (sceneLo.x <= sceneHi.x) {
        const V c{0.5 * (sceneLo.x + sceneHi.x), 0.5 * (sceneLo.y + sceneHi.y), 0.5 * (sceneLo.z + sceneHi.z)};
        const double r = 0.5 * std::max(sceneHi.x - sceneLo.x, sceneHi.y - sceneLo.y);
        scene["view"] = Json{{"camera", Json{{"position", vec3({c.x - 0.9 * r, c.y - 1.1 * r, sceneHi.z + 0.6 * r})}, {"target", vec3({c.x, c.y, sceneLo.z})}}}};
    }
    std::ofstream sceneFile(scenePath, std::ios::binary);
    sceneFile << scene.dump(1) << "\n";
    if (!sceneFile) return fail("无法写入场景文件：" + scenePath.string());
    std::ofstream reportFile(out / "import-report.json", std::ios::binary);
    reportFile << Json{{"format", "terra.environment-import-report"}, {"version", 1}, {"input", input.string()}, {"manifest", result.manifest},
                       {"elements", report}, {"warnings", result.warnings}}
                      .dump(1)
               << "\n";
    result.scene = scenePath.string();
    result.ok = true;
    return result;
}

}  // namespace terra::importer
