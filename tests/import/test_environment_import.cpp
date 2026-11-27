// glTF 环境导入（M5-02b）。两种模式：
//   fixture <输出目录>：夹具 assets/tests/environment/env_fixture.gltf（scripts/assets/make_environment_fixture.py 生成）——
//     元素数、放置位姿（Y 向上 → Z 向上）、盒体（按网格的互不相连部分；斜扶手取主方向盒）、统一缩放、部件（位姿、盒体、温度）、
//     检查点、凸包（坡道）、dynamic、只显示、盒体上限的合并、外观绑定排除部件节点；导入的场景能组合进物理世界，箱子落到基座上；
//     错误：输入不存在、输出已存在且没有 force。
//   assets <输出目录> <blender 目录>：用户的两个场景（blender/substation_110kv、blender/indoor_office；没有时跳过，返回 77）——
//     导入整站 glb，元素数与清单相同；组合进物理世界并推进 0.2 s；打印盒体数、部件数与耗时；
//     20,000 条射线（Mid-360 一帧）在 100 ms 以内（射线检测的包围盒树，2026-11-26）。

#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>

#include "terra/import/environment.hpp"
#include "terra/physics/physics_world.hpp"
#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/scene/element.hpp"
#include "terra/scene/scene.hpp"

#ifndef TERRA_TEST_ASSETS_DIR
#error "需要 TERRA_TEST_ASSETS_DIR"
#endif

namespace {

namespace fs = std::filesystem;
namespace im = terra::importer;
namespace sc = terra::scene;
namespace td = terra::data;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

bool near(double a, double b, double tol = 1e-5) { return std::fabs(a - b) < tol; }
bool near(const td::Vec3& v, double x, double y, double z, double tol = 1e-5) { return near(v.x, x, tol) && near(v.y, y, tol) && near(v.z, z, tol); }

const sc::SceneElementSpec* findElement(const sc::SceneDocument& d, const std::string& id) {
    for (const auto& e : d.elements) {
        if (e.id == id) return &e;
    }
    return nullptr;
}

int fixture(const std::string& work) {
    const std::string input = std::string{TERRA_TEST_ASSETS_DIR} + "/environment/env_fixture.gltf";
    std::error_code ec;
    fs::remove_all(work, ec);
    im::EnvironmentImportOptions o;
    o.input = input;
    o.output = work;
    o.name = "env_fixture";
    const auto r = im::importEnvironment(o);
    for (const auto& w : r.warnings) std::printf("INFO 警告：%s\n", w.c_str());
    for (const auto& e : r.errors) std::fprintf(stderr, "导入错误：%s\n", e.c_str());
    check(r.ok, "导入成功");
    if (!r.ok) return 1;
    std::printf("INFO 夹具：%zu 个元素、%zu 个盒体、%zu 个凸包网格、%zu 个部件、%zu 个检查点\n", r.elements, r.boxes, r.meshes, r.parts, r.inspection);
    check(r.elements == 8 && r.parts == 1 && r.inspection == 1 && r.meshes == 1, "8 个元素（8 个顶层节点）、1 个部件、1 个检查点、1 个凸包网格");
    check(r.manifest == fs::absolute(fs::path{TERRA_TEST_ASSETS_DIR} / "environment/manifest.json").lexically_normal().string(),
          "自动找到 glTF 旁边的 manifest.json");

    const auto parsed = sc::readSceneFile(r.scene);
    for (const auto& i : parsed.issues) std::fprintf(stderr, "场景：%s %s\n", i.where.c_str(), i.message.c_str());
    check(parsed.document.has_value(), "导入的场景可以解析（models 为空）");
    if (!parsed.document) return 1;
    const auto& doc = *parsed.document;
    check(doc.models.empty() && doc.elements.size() == 8, "场景：没有模型实例，8 个元素");

    // 基座：glTF 平移 (2, 0, −1) → Terra (2, 1, 0)；一个盒体，中心 (0, 0, 0.25)，半边长 (0.5, 0.5, 0.25)
    if (const auto* e = findElement(doc, "Plinth")) {
        check(near(e->placement.position, 2, 1, 0), "基座：放置位置 (2, 1, 0)（Y 向上 → Z 向上）");
        const auto& b = e->element.boxes;
        check(b.size() == 1 && near(b[0].pose.position, 0, 0, 0.25) && near(b[0].halfSize, 0.5, 0.5, 0.25) && near(b[0].pose.orientation.w, 1.0),
              "基座：一个盒体，中心 (0, 0, 0.25)，半边长 (0.5, 0.5, 0.25)");
        check(e->element.category == "test/plinth" && e->element.movable == sc::ElementMovable::Fixed && e->element.licenseName == "CC0-1.0",
              "基座：类别、fixed、许可取自清单");
        check(e->element.node == "Plinth" && !e->element.gltf.empty() && fs::path{e->element.gltf}.filename() == "env_fixture.gltf",
              "基座：外观引用原 glTF 的节点 Plinth");
    } else {
        check(false, "有元素 Plinth");
    }
    // 机柜：绕 Z 转 90°、缩放 1.5；机柜本体的盒体按缩放；部件 Rack_hot：位姿 (0, −0.45, 1.2)、盒体半边长 0.075、75 °C
    if (const auto* e = findElement(doc, "Rack")) {
        const double h = std::sqrt(0.5);
        const auto& q = e->placement.orientation;
        check(near(q.w, h) && near(q.x, 0) && near(q.y, 0) && near(q.z, h), "机柜：放置姿态为绕 Z 轴 90°");
        check(near(e->element.visualScale, 1.5), "机柜：外观的缩放 1.5（节点的统一缩放）");
        const auto& b = e->element.boxes;
        check(b.size() == 1 && near(b[0].pose.position, 0, 0, 0.75) && near(b[0].halfSize, 0.3, 0.3, 0.75), "机柜：本体盒体按缩放，中心 (0, 0, 0.75)，半边长 (0.3, 0.3, 0.75)");
        check(e->element.parts.size() == 1, "机柜：一个部件");
        if (e->element.parts.size() == 1) {
            const auto& p = e->element.parts[0];
            check(p.name == "Rack_hot" && p.node == "Rack_hot" && near(p.pose.position, 0, -0.45, 1.2) && near(p.visualScale, 1.5),
                  "部件 Rack_hot：节点、位姿 (0, −0.45, 1.2)、外观缩放 1.5");
            check(p.boxes.size() == 1 && near(p.boxes[0].pose.position, 0, 0, 0) && near(p.boxes[0].halfSize, 0.075, 0.075, 0.075),
                  "部件的盒体在部件坐标系中，半边长 0.075");
            check(p.temperature && p.temperature->at(0) == 75.0, "部件温度 75 °C（清单 thermal.parts）");
        }
        check(e->element.thermalDefault && e->element.thermalDefault->at(0) == 40.0, "机柜默认温度 40 °C");
        check(e->element.inspection.size() == 1 && e->element.inspection[0].name == "仪表" && e->element.inspection[0].node == "Rack_hot" &&
                  near(e->element.inspection[0].position, 0, -0.45, 1.2) && e->element.inspection[0].distance == 1.5,
              "检查点：位置为节点网格包围盒的中心（元素坐标系）");
    } else {
        check(false, "有元素 Rack");
    }
    // 楼梯：3 个踏步各一个盒体 + 斜扶手一个细长的主方向盒
    if (const auto* e = findElement(doc, "Stairs")) {
        const auto& b = e->element.boxes;
        check(b.size() == 4, "楼梯：4 个盒体（3 个踏步 + 扶手），实际 " + std::to_string(b.size()));
        int steps = 0, rails = 0;
        for (const auto& box : b) {
            for (int k = 0; k < 3; ++k) {
                if (near(box.pose.position, 0.3 * k + 0.149, 0, 0.075 * (k + 1)) && near(box.halfSize, 0.149, 0.5, 0.075 * (k + 1))) ++steps;
            }
            const double hs[3] = {box.halfSize.x, box.halfSize.y, box.halfSize.z};
            int thin = 0;
            for (const double v : hs) thin += v < 0.03 ? 1 : 0;
            if (thin == 2) ++rails;
        }
        check(steps == 3, "楼梯：每个踏步的盒体与几何一致（深 0.298、每级高 0.15、宽 1）");
        check(rails == 1, "楼梯：斜扶手取主方向盒（两个方向很薄），没有用跨过整段楼梯的轴对齐盒");
    }
    // 坡道：类别含 ramp，按凸包；网格文件存在
    if (const auto* e = findElement(doc, "Ramp_Main")) {
        check(e->element.collision == sc::ElementCollision::Mesh && e->element.meshes.size() == 1 && fs::is_regular_file(e->element.meshes[0].file),
              "坡道：凸包（OBJ 碰撞网格）");
    }
    if (const auto* e = findElement(doc, "Crate")) {
        check(e->element.movable == sc::ElementMovable::Dynamic && e->element.mass == 3.0 && near(e->placement.position, 2, 1, 1), "箱子：dynamic、3 kg，放在 (2, 1, 1)");
    }
    if (const auto* e = findElement(doc, "Lamp")) check(e->element.collision == sc::ElementCollision::None, "没有网格的节点：只显示");
    // П 形门框：门柱与门楣连成一个网格；盒体不能堵住门洞（2026-11-26：办公楼的门框曾被一个盒体整个堵上）
    if (const auto* e = findElement(doc, "Doorway")) {
        const auto inside = [](const sc::ElementBox& b, double x, double y, double z) {
            // 门框的盒体都与坐标轴对齐（或很接近）：按轴对齐近似判断
            return std::fabs(x - b.pose.position.x) < b.halfSize.x && std::fabs(y - b.pose.position.y) < b.halfSize.y && std::fabs(z - b.pose.position.z) < b.halfSize.z;
        };
        bool blocked = false;
        for (const auto& b : e->element.boxes) {
            for (const double x : {-0.6, 0.0, 0.6}) {
                for (const double z : {0.3, 1.1, 1.9}) blocked = blocked || inside(b, x, 0.0, z);
            }
        }
        check(e->element.boxes.size() >= 3, "门框：拆成至少 3 个盒体（实际 " + std::to_string(e->element.boxes.size()) + "）");
        check(!blocked, "门框：门洞（宽 1.8 m、高 2.2 m）中间是空的");
        check(near(e->placement.position, 0, 4, 0), "门框：放在 (0, 4, 0)");
    }
    if (const auto* e = findElement(doc, "Fence")) {
        check(e->element.boxes.size() == 24, "围栏：30 根立柱合并到每个节点的上限 24 个盒体（实际 " + std::to_string(e->element.boxes.size()) + "）");
    }
    // 外观绑定：机柜的绑定排除部件节点；部件另有绑定
    bool exclude = false, partBinding = false;
    for (const auto& v : doc.visuals) {
        if (v.entity == "Rack/body" && v.node == "Rack" && v.excludeNodes.size() == 1 && v.excludeNodes[0] == "Rack_hot" && near(v.scale, 1.5)) exclude = true;
        if (v.entity == "Rack/Rack_hot" && v.node == "Rack_hot" && near(v.scale, 1.5)) partBinding = true;
    }
    check(exclude && partBinding, "外观：机柜的绑定排除 Rack_hot，Rack_hot 另有绑定");

    // 组合进物理世界：箱子落到基座上（顶面 0.5 m）
    auto loaded = sc::loadScene(r.scene, terra::physics::mujoco::composeWorld);
    for (const auto& i : loaded.issues) std::fprintf(stderr, "加载：%s %s\n", i.where.c_str(), i.message.c_str());
    check(loaded.scene.has_value(), "导入的场景能组合进物理世界");
    if (loaded.scene) {
        auto& world = *loaded.scene->world;
        const auto& d = world.description();
        const auto* crate = d.findEntity("Crate/body");
        const auto* rack = d.findEntity("Rack/body");
        const auto* hot = d.findEntity("Rack/Rack_hot");
        check(crate && rack && hot && hot->parent == rack->id, "部件是元素刚体的子刚体（Rack/Rack_hot）");
        if (hot && rack) {
            check(loaded.scene->thermal.base(hot->id, 0) == 75.0 && loaded.scene->thermal.base(rack->id, 0) == 40.0, "温度：部件 75 °C，机柜 40 °C");
            const auto pose = world.snapshot()->entities[hot->id.value - 1].pose;
            check(near(pose.position, 0.45, 0, 1.2, 1e-6), "部件的世界位置：机柜转 90° 后在 (0.45, 0, 1.2)");
        }
        for (int i = 0; i < 600; ++i) world.step();
        if (crate) {
            const double z = world.snapshot()->entities[crate->id.value - 1].pose.position.z;
            std::printf("INFO 箱子 1.2 s 后 z = %.4f m（基座顶面 0.5 m）\n", z);
            check(std::fabs(z - 0.5) < 3e-3, "箱子落到基座上");
        }
    }

    // 错误
    {
        im::EnvironmentImportOptions again = o;
        check(!im::importEnvironment(again).ok, "输出目录已有场景且没有 force：报错");
        again.force = true;
        check(im::importEnvironment(again).ok, "force：覆盖");
        im::EnvironmentImportOptions missing = o;
        missing.input = std::string{TERRA_TEST_ASSETS_DIR} + "/environment/nothing.glb";
        missing.output = work + "_missing";
        const auto m = im::importEnvironment(missing);
        check(!m.ok && !m.errors.empty(), "输入不存在：报错");
        im::EnvironmentImportOptions hulls = o;
        hulls.hulls = true;
        hulls.force = true;
        hulls.output = work + "_hulls";
        const auto h = im::importEnvironment(hulls);
        check(h.ok && h.boxes == 0 && h.meshes >= 6, "全部按凸包：没有盒体，每个网格节点一个 OBJ");
        if (h.ok) check(sc::loadScene(h.scene, terra::physics::mujoco::composeWorld).scene.has_value(), "凸包导入的场景也能组合进物理世界");
    }
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}

int assets(const std::string& work, const std::string& blender) {
    struct Case {
        const char* name;
        const char* glb;
        std::size_t elements;
        double origin[3];  // 射线检测的起点：推荐巡检路线的起点离地 0.5 m
    };
    const Case cases[] = {{"substation_110kv", "substation_110kv/exports/substation_110kv_full.glb", 233, {0.0, -24.0, 0.5}},
                          {"indoor_office", "indoor_office/exports/indoor_office_full.glb", 248, {6.6, -5.0, 0.35}}};
    bool any = false;
    for (const Case& c : cases) {
        const fs::path glb = fs::path{blender} / c.glb;
        std::error_code ec;
        if (!fs::is_regular_file(glb, ec)) {
            std::printf("SKIP %s：没有 %s\n", c.name, glb.string().c_str());
            continue;
        }
        any = true;
        const auto begin = std::chrono::steady_clock::now();
        im::EnvironmentImportOptions o;
        o.input = glb.string();
        o.output = (fs::path{work} / c.name).string();
        o.name = c.name;
        o.force = true;
        const auto r = im::importEnvironment(o);
        const double importMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
        for (const auto& w : r.warnings) std::printf("INFO %s 警告：%s\n", c.name, w.c_str());
        for (const auto& e : r.errors) std::fprintf(stderr, "%s 导入错误：%s\n", c.name, e.c_str());
        check(r.ok && r.elements == c.elements && !r.manifest.empty(), std::string{c.name} + "：导入 " + std::to_string(r.elements) + " 个元素（清单 " +
                                                                           std::to_string(c.elements) + " 个）");
        if (!r.ok) continue;
        const auto t1 = std::chrono::steady_clock::now();
        auto loaded = sc::loadScene(r.scene, terra::physics::mujoco::composeWorld);
        int errors = 0;
        for (const auto& i : loaded.issues) {
            if (i.severity == sc::SceneIssue::Severity::Error && errors++ < 10) std::fprintf(stderr, "%s 加载：%s %s\n", c.name, i.where.c_str(), i.message.c_str());
        }
        check(loaded.scene.has_value(), std::string{c.name} + "：组合进物理世界");
        if (!loaded.scene) continue;
        const double loadMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t1).count();
        auto& world = *loaded.scene->world;
        const auto t2 = std::chrono::steady_clock::now();
        for (int i = 0; i < 100; ++i) world.step();
        const double stepMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t2).count() / 100.0;
        std::printf("INFO %s：%zu 个元素、%zu 个盒体、%zu 个凸包网格、%zu 个部件、%zu 个检查点；%zu 个实体、%zu 个几何体；导入 %.0f ms，加载 %.0f ms，每步 %.3f ms\n",
                    c.name, r.elements, r.boxes, r.meshes, r.parts, r.inspection, world.description().entities.size(), world.description().geoms.size(),
                    importMs, loadMs, stepMs);
        check(r.parts > 0 && r.inspection > 0, std::string{c.name} + "：有部件与检查点（清单的 thermal.parts 与 inspection）");
        // 射线检测的耗时（2026-11-26 的性能修正）：20,000 条射线（与 Mid-360 一帧相同），方向在 −7°～52° 的仰角内均匀分布，
        // 从推荐巡检路线的起点发出。修正前（每条射线检查全部几何体）估计约 2 s；要求在 100 ms 以内
        {
            terra::physics::RayQuery q;
            q.origin = {c.origin[0], c.origin[1], c.origin[2]};
            q.maxRange = 40.0;
            const double pi = 3.14159265358979323846;
            for (int k = 0; k < 20000; ++k) {
                const double az = 2.0 * pi * std::fmod(k * 0.6180339887498949, 1.0);
                const double el = (-7.0 + 59.0 * (static_cast<double>(k) + 0.5) / 20000.0) * pi / 180.0;
                q.directions.push_back({std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el)});
            }
            const auto r0 = std::chrono::steady_clock::now();
            const auto ranges = world.castRays(q);
            const double rayMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - r0).count();
            std::size_t hits = 0;
            for (const double v : ranges) hits += std::isfinite(v) ? 1u : 0u;
            const auto again = world.castRays(q);
            std::printf("INFO %s：20,000 条射线 %.1f ms，命中 %zu 条\n", c.name, rayMs, hits);
#if defined(__SANITIZE_ADDRESS__)
            // ASan / UBSan 构建慢数倍：只核对结果，不判耗时
            std::printf("INFO %s：ASan 构建不判耗时\n", c.name);
#else
            check(rayMs < 100.0, std::string{c.name} + "：20,000 条射线在 100 ms 以内（" + std::to_string(rayMs) + " ms）");
#endif
            check(hits > 1000 && again == ranges, std::string{c.name} + "：射线命中了场景，两次结果逐位相同");
        }
        check(world.stepIndex() == 100, std::string{c.name} + "：推进 100 步没有出错");
    }
    if (!any) return 77;
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string mode = argc > 1 ? argv[1] : "";
    if (mode == "fixture" && argc > 2) return fixture(argv[2]);
    if (mode == "assets" && argc > 3) return assets(argv[2], argv[3]);
    std::fprintf(stderr, "用法：test_environment_import fixture <输出目录> | assets <输出目录> <blender 目录>\n");
    return 2;
}
