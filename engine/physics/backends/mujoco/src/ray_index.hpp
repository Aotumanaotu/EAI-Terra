#pragma once

// 射线检测的空间索引（M5-02b 之后的性能修正，2026-11-26）：导入的变电站有近 5000 个碰撞盒，MuJoCo 的 mj_ray / mj_multiRay
// 每条射线都检查全部几何体（只用包围球剔除），Mid-360 一帧 20,000 条射线要约 2 s，单线程的仿真跟不上实时。
//
// 做法：静止的几何体（刚体焊在世界上，body_weldid 为 0）位置不变，按世界中的轴对齐包围盒（由 geom_aabb 与位姿算出）
// 建一棵包围盒树（BVH），每条射线只访问与它相交的节点；运动的几何体（机器人、可推动的元素）与无界的几何体（平面）
// 每条射线逐个检查。几何体的求交调用 MuJoCo 自己的 mju_rayGeom / mj_rayMesh / mj_rayHfield / mj_raySdf，
// 过滤规则与 mj_ray 的 ray_eliminate 相同（排除的刚体、透明的几何体或材质、几何体组），所以命中距离与 mj_ray 逐位相同。
// 只依赖模型与静止几何体的位姿：世界组合后建一次；静止几何体的位姿改变（如 M5-03 的布局编辑）时需要重建。

#include <array>
#include <vector>

#include <mujoco/mujoco.h>

namespace terra::physics::mujoco {

class RayIndex {
public:
    RayIndex(const mjModel* m, const mjData* d);
    // 模型中有 SDF 几何体时不能用（其求交函数不公开），调用方退回 mj_multiRay
    static bool usable(const mjModel* m);

    // 过滤：exclude 为排除的刚体（−1 为不排除）；groups 为可见的几何体组（空表示全部）；collidingOnly 只认参与碰撞的几何体
    struct Filter {
        int exclude = -1;
        const mjtByte* groups = nullptr;
        bool collidingOnly = false;
    };
    // 最近的命中距离（以 vec 的长度为单位，与 mj_ray 相同）；没有命中或超过 cutoff 时为 −1
    mjtNum cast(const mjModel* m, const mjData* d, const mjtNum pnt[3], const mjtNum vec[3], const Filter& filter, mjtNum cutoff) const;

    std::size_t staticGeoms() const noexcept { return order_.size(); }
    std::size_t otherGeoms() const noexcept { return others_.size(); }

private:
    struct Node {
        std::array<mjtNum, 3> lo{}, hi{};
        int left = -1, right = -1;  // 内部节点的子节点
        int begin = 0, end = 0;     // 叶子：order_[begin, end)
    };
    int build(int begin, int end);

    std::vector<Node> nodes_;
    std::vector<int> order_;  // 静止几何体的下标（按树的叶子排列）
    std::vector<std::array<mjtNum, 3>> lo_, hi_, center_;  // 按 order_ 原下标：每个静止几何体的世界包围盒
    std::vector<int> others_;  // 运动的与无界的几何体
};

}  // namespace terra::physics::mujoco
