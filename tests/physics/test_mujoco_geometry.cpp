// 几何体与网格描述（M2-03，模型描述修订 2）。只通过 Terra 公共头访问后端，本文件不包含 MuJoCo 头文件。
// 夹具 assets/tests/geometry_shapes.xml：每种形状的尺寸与局部位姿、fromto 写法、材质颜色规则、分组与碰撞标志、
// 网格（编译后顶点 × 局部位姿 = 原始摆放）、静态几何、子实体；单模型加载时的模型实例。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/physics/physics_world.hpp"

#ifndef TERRA_TEST_ASSETS_DIR
#error "需要 TERRA_TEST_ASSETS_DIR"
#endif

namespace {

namespace fs = std::filesystem;
namespace td = terra::data;
using td::GeomShape;
using td::Quaternion;
using td::Vec3;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

std::string str(const Vec3& v) {
    char buf[96];
    std::snprintf(buf, sizeof buf, "(%.6g, %.6g, %.6g)", v.x, v.y, v.z);
    return buf;
}

bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) <= tol; }
bool near(const Vec3& a, const Vec3& b, double tol = 1e-9) {
    return near(a.x, b.x, tol) && near(a.y, b.y, tol) && near(a.z, b.z, tol);
}
Vec3 add(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }

Vec3 rotate(const Quaternion& q, const Vec3& v) {
    const double tx = 2 * (q.y * v.z - q.z * v.y);
    const double ty = 2 * (q.z * v.x - q.x * v.z);
    const double tz = 2 * (q.x * v.y - q.y * v.x);
    return {v.x + q.w * tx + (q.y * tz - q.z * ty), v.y + q.w * ty + (q.z * tx - q.x * tz),
            v.z + q.w * tz + (q.x * ty - q.y * tx)};
}

// 两个四元数表示同一旋转（q 与 −q 等价）
bool sameRotation(const Quaternion& a, const Quaternion& b, double tol = 1e-6) {
    const double dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    return std::fabs(std::fabs(dot) - 1.0) <= tol;
}

const td::GeomDescription* geom(const td::ModelDescription& d, const char* name) {
    const auto* g = d.findGeom(name);
    check(g != nullptr, std::string{"找到几何体 "} + name);
    return g;
}

void testShapes(const td::ModelDescription& d) {
    const auto* base = d.findEntity("base");
    const auto* child = d.findEntity("child");
    check(base != nullptr && child != nullptr, "实体 base 与 child");
    if (base == nullptr || child == nullptr) return;
    check(d.geoms.size() == 10, "共 10 个几何体（含静态地面），实际 " + std::to_string(d.geoms.size()));
    for (std::size_t i = 0; i < d.geoms.size(); ++i) {
        check(d.geoms[i].id.value == i + 1 && d.geom(d.geoms[i].id) == &d.geoms[i], "几何体 ID 从 1 连续，可按 ID 查找");
        check(d.geoms[i].modelInstance.value == 1, "几何体属于唯一的模型实例");
    }
    check(d.geom(td::GeomId{}) == nullptr && d.geom(td::GeomId{99}) == nullptr && d.findGeom("") == nullptr,
          "无效 ID 与空名字查不到");

    if (const auto* g = geom(d, "floor")) {
        check(g->shape == GeomShape::Plane && !g->entity.valid(), "地面：平面、静态（不属于任何实体）");
        check(near(g->size, {2, 3, 0.1}), "地面尺寸：半边长 2×3，网格间距 0.1，实际 " + str(g->size));
        check(g->rgba[0] == 0.3f && g->rgba[3] == 1.0f && g->collides && g->group == 0, "地面颜色、参与碰撞、group 0");
    }
    if (const auto* g = geom(d, "ball")) {
        check(g->shape == GeomShape::Sphere && g->entity == base->id, "球属于 base");
        check(near(g->size.x, 0.1) && near(g->local.position, {0.1, 0.2, 0.3}), "球：半径 0.1，局部位置 " + str(g->local.position));
        check(g->group == 1, "球 group 1");
    }
    if (const auto* g = geom(d, "crate")) {
        const double h = std::sqrt(0.5);
        check(g->shape == GeomShape::Box && near(g->size, {0.1, 0.2, 0.3}), "盒：三个半边长 " + str(g->size));
        check(near(g->local.position, {0, 0, -0.5}), "盒：局部位置");
        check(sameRotation(g->local.orientation, {h, 0, 0, h}), "盒：绕 Z 轴 90°（euler 写法，单位度）");
        check(g->rgba[0] == 0.8f && g->rgba[1] == 0.1f && g->rgba[2] == 0.1f, "盒：颜色为默认值时取材质颜色");
    }
    if (const auto* g = geom(d, "tinted")) {
        check(g->rgba[0] == 0.0f && g->rgba[1] == 1.0f && g->rgba[2] == 0.0f, "几何体自己写了颜色时优先于材质");
    }
    if (const auto* g = geom(d, "rod")) {
        check(g->shape == GeomShape::Capsule && near(g->size.x, 0.02) && near(g->size.y, 0.3, 1e-9),
              "竖直胶囊（fromto）：半径 0.02，半长 0.3，实际 " + str(g->size));
        check(near(g->local.position, {0, 0, 0.3}), "竖直胶囊：中心在两端点中间");
        // 胶囊关于中点对称，轴取 +Z 或 −Z 都正确：MuJoCo 由 fromto 计算方向时可能取 from − to（与水平胶囊的检查一致）
        const Vec3 axis = rotate(g->local.orientation, {0, 0, 1});
        check(near(axis.x, 0, 1e-9) && near(axis.y, 0, 1e-9) && near(std::fabs(axis.z), 1.0, 1e-9),
              "竖直胶囊：轴沿 ±Z，实际 " + str(axis));
    }
    if (const auto* g = geom(d, "bar")) {
        check(near(g->size.y, 0.2, 1e-9) && near(g->local.position, {0.2, 0, 0}), "水平胶囊：半长 0.2，中心 (0.2, 0, 0)");
        const Vec3 axis = rotate(g->local.orientation, {0, 0, 1});
        check(near(std::fabs(axis.x), 1.0, 1e-9) && near(axis.y, 0, 1e-9) && near(axis.z, 0, 1e-9),
              "水平胶囊：局部 Z 轴沿世界 X，实际 " + str(axis));
    }
    if (const auto* g = geom(d, "can")) {
        check(g->shape == GeomShape::Cylinder && near(g->size.x, 0.05) && near(g->size.y, 0.1), "圆柱：半径 0.05，半长 0.1");
        check(near(rotate(g->local.orientation, {0, 0, 1}), {0, -1, 0}, 1e-6), "圆柱：绕 X 轴 90° 后轴沿 −Y");
    }
    if (const auto* g = geom(d, "egg")) {
        check(g->shape == GeomShape::Ellipsoid && near(g->size, {0.1, 0.2, 0.3}), "椭球：三个半轴");
        check(!g->collides && g->group == 2, "椭球：不参与碰撞（contype = conaffinity = 0），group 2");
    }
    if (const auto* g = geom(d, "child_ball")) {
        check(g->entity == child->id && near(g->local.position, {0, 0, 0.2}), "子实体上的几何体：属于 child，局部位置相对 child");
    }
}

// 网格：编译后的顶点经局部位姿变换，应与 MJCF 中原始顶点（加上几何体位置）一一对应。
void testMesh(const td::ModelDescription& d) {
    const auto* g = geom(d, "tet");
    if (g == nullptr) return;
    check(g->shape == GeomShape::Mesh && g->mesh == 0 && d.meshes.size() == 1, "网格几何体引用第 0 个网格");
    if (g->mesh != 0 || d.meshes.size() != 1) return;
    const auto& mesh = d.meshes[0];
    check(mesh.name == "tetra", "网格名称");
    check(mesh.vertices.size() == 4, "网格 4 个顶点，实际 " + std::to_string(mesh.vertices.size()));
    check(mesh.triangles.size() == 4, "四面体 4 个三角形，实际 " + std::to_string(mesh.triangles.size()));
    for (const auto& t : mesh.triangles) {
        check(t[0] < mesh.vertices.size() && t[1] < mesh.vertices.size() && t[2] < mesh.vertices.size(), "三角形下标在范围内");
    }
    const std::vector<Vec3> original = {{0.5, 0, 0}, {0.7, 0, 0}, {0.5, 0.3, 0}, {0.5, 0, 0.4}};
    std::vector<bool> matched(original.size(), false);
    for (const auto& v : mesh.vertices) {
        const Vec3 placed = add(rotate(g->local.orientation, v), g->local.position);
        bool found = false;
        for (std::size_t i = 0; i < original.size(); ++i) {
            if (!matched[i] && near(placed, original[i], 1e-5)) {
                matched[i] = found = true;
                break;
            }
        }
        check(found, "编译后的顶点经局部位姿变换后落在原始顶点上：" + str(placed));
    }
    check(std::all_of(matched.begin(), matched.end(), [](bool m) { return m; }), "每个原始顶点都有对应");
}

// 世界位姿 = 实体位姿 × 局部位姿（几何体不进入快照）
void testWorldPose(terra::physics::PhysicsWorld& world) {
    const auto& d = world.description();
    const auto* g = d.findGeom("ball");
    const auto snap = world.snapshot();
    if (g == nullptr || !g->entity.valid() || snap->entities.size() < g->entity.value) return;
    const auto& e = snap->entities[g->entity.value - 1];
    const Vec3 worldPos = add(rotate(e.pose.orientation, g->local.position), e.pose.position);
    check(near(worldPos, {0.1, 0.2, 1.3}, 1e-9), "球的世界位置 = base (0, 0, 1) + 局部 (0.1, 0.2, 0.3)，实际 " + str(worldPos));
}

void testInstances(const td::ModelDescription& d) {
    check(td::ModelDescription::kRevision >= 2, "模型描述修订号 ≥ 2（M3-02 起为 3）");
    check(d.modelInstances.size() == 1, "直接加载单个 MJCF：一个模型实例");
    if (d.modelInstances.size() != 1) return;
    const auto& inst = d.modelInstances[0];
    check(inst.id.value == 1 && inst.name.empty() && inst.sourcePath == d.sourcePath, "实例 1：名字为空，来源为模型文件");
    check(inst.placement.position.x == 0 && inst.placement.orientation.w == 1, "放置位姿为单位变换");
    check(d.modelInstance(td::ModelInstanceId{1}) == &inst && d.modelInstance(td::ModelInstanceId{2}) == nullptr,
          "按 ID 查找模型实例");
    for (const auto& e : d.entities) check(e.modelInstance.value == 1, "实体属于实例 1：" + e.name);
}

void testOtherFixtures() {
    // M0 夹具也带几何体描述：立方体与地面
    auto r = terra::physics::mujoco::loadWorld(td::ModelSource{(fs::path{TERRA_TEST_ASSETS_DIR} / "falling_cube.xml").string()});
    check(r.world != nullptr, "加载 falling_cube.xml");
    if (!r.world) return;
    const auto& d = r.world->description();
    const auto statics = std::count_if(d.geoms.begin(), d.geoms.end(), [](const auto& g) { return !g.entity.valid(); });
    const auto boxes = std::count_if(d.geoms.begin(), d.geoms.end(), [](const auto& g) { return g.shape == GeomShape::Box; });
    check(statics >= 1 && boxes == 1, "falling_cube：至少一个静态几何（地面），一个盒");
    check(d.meshes.empty(), "falling_cube：没有网格");
}

}  // namespace

int main() {
    auto r = terra::physics::mujoco::loadWorld(td::ModelSource{(fs::path{TERRA_TEST_ASSETS_DIR} / "geometry_shapes.xml").string()});
    if (!r.world) {
        std::fprintf(stderr, "FAILED: 加载 geometry_shapes.xml：%s\n", r.error ? r.error->message.c_str() : "");
        return 1;
    }
    const auto& d = r.world->description();
    std::printf("INFO 几何体 %zu 个，网格 %zu 个，实体 %zu 个\n", d.geoms.size(), d.meshes.size(), d.entities.size());
    testShapes(d);
    testMesh(d);
    testWorldPose(*r.world);
    testInstances(d);
    testOtherFixtures();
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
