// 射线检测的空间索引，说明见 ray_index.hpp。

#include "ray_index.hpp"

#include <algorithm>
#include <cmath>

namespace terra::physics::mujoco {

namespace {

// 与 MuJoCo engine_ray.c 的 ray_eliminate 相同（flg_static 为真），另加“只认参与碰撞的几何体”
bool eliminated(const mjModel* m, int g, const RayIndex::Filter& f) {
    if (m->geom_bodyid[g] == f.exclude) return true;
    if (m->geom_matid[g] < 0 && m->geom_rgba[4 * g + 3] == 0) return true;
    if (m->geom_matid[g] >= 0 && m->mat_rgba[4 * m->geom_matid[g] + 3] == 0) return true;
    if (f.groups != nullptr) {
        const int group = std::min(mjNGROUP - 1, std::max(0, m->geom_group[g]));
        if (f.groups[group] == 0) return true;
    }
    if (f.collidingOnly && m->geom_contype[g] == 0 && m->geom_conaffinity[g] == 0) return true;
    return false;
}

// 与 mj_ray 相同的求交
mjtNum intersect(const mjModel* m, const mjData* d, int g, const mjtNum pnt[3], const mjtNum vec[3]) {
    switch (m->geom_type[g]) {
        case mjGEOM_MESH: return mj_rayMesh(m, d, g, pnt, vec, nullptr);
        case mjGEOM_HFIELD: return mj_rayHfield(m, d, g, pnt, vec, nullptr);
        case mjGEOM_SDF: return -1;  // 有 SDF 的模型不用本索引（mj_raySdf 不是公开接口），见 usable()
        default: return mju_rayGeom(d->geom_xpos + 3 * g, d->geom_xmat + 9 * g, m->geom_size + 3 * g, pnt, vec, m->geom_type[g], nullptr);
    }
}

}  // namespace

RayIndex::RayIndex(const mjModel* m, const mjData* d) {
    const auto n = static_cast<std::size_t>(m->ngeom);
    lo_.resize(n);
    hi_.resize(n);
    center_.resize(n);
    for (int g = 0; g < m->ngeom; ++g) {
        const bool fixed = m->body_weldid[m->geom_bodyid[g]] == 0;
        const bool bounded = m->geom_rbound[g] > 0 && m->geom_type[g] != mjGEOM_SDF;
        if (!fixed || !bounded) {
            others_.push_back(g);
            continue;
        }
        // geom_aabb：几何体坐标系中的包围盒（中心 3 + 半边长 3）；世界中按旋转取绝对值展开
        const mjtNum* aabb = m->geom_aabb + 6 * g;
        const mjtNum* x = d->geom_xpos + 3 * g;
        const mjtNum* r = d->geom_xmat + 9 * g;  // 行主序
        const auto k = static_cast<std::size_t>(g);
        for (int i = 0; i < 3; ++i) {
            mjtNum c = x[i], h = 0;
            for (int j = 0; j < 3; ++j) {
                c += r[3 * i + j] * aabb[j];
                h += std::fabs(r[3 * i + j]) * aabb[3 + j];
            }
            // 留一点余量（浮点舍入），不影响结果：只用于剔除
            h = h * (1 + 1e-9) + 1e-9;
            lo_[k][static_cast<std::size_t>(i)] = c - h;
            hi_[k][static_cast<std::size_t>(i)] = c + h;
            center_[k][static_cast<std::size_t>(i)] = c;
        }
        order_.push_back(g);
    }
    if (!order_.empty()) {
        nodes_.reserve(2 * order_.size());
        build(0, static_cast<int>(order_.size()));
    }
}

bool RayIndex::usable(const mjModel* m) {
    for (int g = 0; g < m->ngeom; ++g) {
        if (m->geom_type[g] == mjGEOM_SDF) return false;
    }
    return true;
}

int RayIndex::build(int begin, int end) {
    const int id = static_cast<int>(nodes_.size());
    nodes_.push_back(Node{});
    Node node;
    node.lo = {1e300, 1e300, 1e300};
    node.hi = {-1e300, -1e300, -1e300};
    std::array<mjtNum, 3> clo{1e300, 1e300, 1e300}, chi{-1e300, -1e300, -1e300};
    for (int i = begin; i < end; ++i) {
        const auto g = static_cast<std::size_t>(order_[static_cast<std::size_t>(i)]);
        for (std::size_t a = 0; a < 3; ++a) {
            node.lo[a] = std::min(node.lo[a], lo_[g][a]);
            node.hi[a] = std::max(node.hi[a], hi_[g][a]);
            clo[a] = std::min(clo[a], center_[g][a]);
            chi[a] = std::max(chi[a], center_[g][a]);
        }
    }
    if (end - begin <= 4) {
        node.begin = begin;
        node.end = end;
        nodes_[static_cast<std::size_t>(id)] = node;
        return id;
    }
    // 按中心的范围最大的轴，在中位数处分开（确定性：nth_element 后再按下标稳定）
    std::size_t axis = 0;
    for (std::size_t a = 1; a < 3; ++a) {
        if (chi[a] - clo[a] > chi[axis] - clo[axis]) axis = a;
    }
    const int mid = (begin + end) / 2;
    std::nth_element(order_.begin() + begin, order_.begin() + mid, order_.begin() + end, [&](int a, int b) {
        const mjtNum ca = center_[static_cast<std::size_t>(a)][axis], cb = center_[static_cast<std::size_t>(b)][axis];
        return ca < cb || (ca == cb && a < b);
    });
    node.left = build(begin, mid);
    node.right = build(mid, end);
    nodes_[static_cast<std::size_t>(id)] = node;
    return id;
}

mjtNum RayIndex::cast(const mjModel* m, const mjData* d, const mjtNum pnt[3], const mjtNum vec[3], const Filter& filter, mjtNum cutoff) const {
    mjtNum best = -1;
    const auto consider = [&](int g) {
        if (eliminated(m, g, filter)) return;
        const mjtNum t = intersect(m, d, g, pnt, vec);
        if (t >= 0 && t <= cutoff && (best < 0 || t < best)) best = t;
    };
    // 射线与轴对齐盒的参数区间（以 vec 为单位）
    const auto slab = [&](const std::array<mjtNum, 3>& lo, const std::array<mjtNum, 3>& hi, mjtNum& tnear) {
        mjtNum t0 = 0, t1 = best >= 0 ? best : cutoff;
        for (std::size_t a = 0; a < 3; ++a) {
            const mjtNum p = pnt[a], v = vec[a];
            if (std::fabs(v) < 1e-300) {
                if (p < lo[a] || p > hi[a]) return false;
                continue;
            }
            mjtNum ta = (lo[a] - p) / v, tb = (hi[a] - p) / v;
            if (ta > tb) std::swap(ta, tb);
            t0 = std::max(t0, ta);
            t1 = std::min(t1, tb);
            if (t0 > t1) return false;
        }
        tnear = t0;
        return true;
    };
    if (!nodes_.empty()) {
        // 中位数切分：树深不超过 log₂(几何体数) + 1，栈最多 2 × 深度
        int stack[128];
        int top = 0;
        mjtNum tnear = 0;
        if (slab(nodes_[0].lo, nodes_[0].hi, tnear)) stack[top++] = 0;
        while (top > 0) {
            const Node& node = nodes_[static_cast<std::size_t>(stack[--top])];
            if (node.left < 0) {
                for (int i = node.begin; i < node.end; ++i) consider(order_[static_cast<std::size_t>(i)]);
                continue;
            }
            // 先访问较近的子节点（后压栈）
            mjtNum nl = 0, nr = 0;
            const bool hl = slab(nodes_[static_cast<std::size_t>(node.left)].lo, nodes_[static_cast<std::size_t>(node.left)].hi, nl);
            const bool hr = slab(nodes_[static_cast<std::size_t>(node.right)].lo, nodes_[static_cast<std::size_t>(node.right)].hi, nr);
            if (hl && hr) {
                if (nl <= nr) {
                    stack[top++] = node.right;
                    stack[top++] = node.left;
                } else {
                    stack[top++] = node.left;
                    stack[top++] = node.right;
                }
            } else if (hl) {
                stack[top++] = node.left;
            } else if (hr) {
                stack[top++] = node.right;
            }
        }
    }
    // 运动的与无界的几何体：按包围球剔除后逐个检查
    const mjtNum len = std::sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
    for (const int g : others_) {
        const mjtNum rb = m->geom_rbound[g];
        if (rb > 0 && len > 0) {
            const mjtNum* c = d->geom_xpos + 3 * g;
            const mjtNum dx = c[0] - pnt[0], dy = c[1] - pnt[1], dz = c[2] - pnt[2];
            const mjtNum proj = (dx * vec[0] + dy * vec[1] + dz * vec[2]) / len;  // 沿射线的距离（米）
            const mjtNum perp2 = dx * dx + dy * dy + dz * dz - proj * proj;
            if (proj < -rb || perp2 > rb * rb * (1 + 1e-9) + 1e-12) continue;
            const mjtNum limit = (best >= 0 ? best : cutoff) * len;  // 米
            if (proj - rb > limit) continue;
        }
        consider(g);
    }
    return best;
}

}  // namespace terra::physics::mujoco
