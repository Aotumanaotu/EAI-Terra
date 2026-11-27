// 场景元素（M5-02a，scene.elements）。夹具 assets/tests/elements/：
//   - 元素文件：盒体、碰撞网格（凸包）、只显示；layout / fixed / dynamic；温度与热点；检查点；许可；错误；
//   - 组合：元素成为实例，刚体 “<id>/body”；全局物理设置随模型实例（不产生警告）；dynamic 的质量与自由关节；
//   - 物理：木箱从 0.5 m 落到地面、从 1.0 m 落到碰撞网格的基座上（顶面 0.5 m）；静止的元素不动；只显示的元素不挡木箱；
//   - 温度：实例覆盖 > 场景 temperatures > 元素默认；元素文件的热点；分割类别取元素的 category（labels 优先）。

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/scene/element.hpp"
#include "terra/scene/scene.hpp"

#ifndef TERRA_TEST_ASSETS_DIR
#error "需要 TERRA_TEST_ASSETS_DIR"
#endif

namespace {

namespace sc = terra::scene;
namespace td = terra::data;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

const std::string kDir = std::string{TERRA_TEST_ASSETS_DIR} + "/elements";

std::string elementError(const std::string& body) {
    // body 带上根对象的右括号
    const auto r = sc::parseElement(R"({"format": "terra.element", "version": 1, )" + body, kDir + "/x.terra-element.json");
    for (const auto& i : r.issues) {
        if (i.severity == sc::SceneIssue::Severity::Error) return i.where + " " + i.message;
    }
    return {};
}

std::string sceneError(const std::string& elements) {
    const std::string text = R"({"format": "terra.scene", "version": 1, "models": [{"name": "ground", "mjcf": "../scene/ground.xml"}], "elements": [)" +
                             elements + "]}";
    const auto parsed = sc::parseScene(text, kDir + "/x.terra-scene.json");
    for (const auto& i : parsed.issues) {
        if (i.severity == sc::SceneIssue::Severity::Error) return i.where + " " + i.message;
    }
    if (!parsed.document) return "?";
    const auto r = sc::instantiate(*parsed.document, terra::physics::mujoco::composeWorld);
    for (const auto& i : r.issues) {
        if (i.severity == sc::SceneIssue::Severity::Error) return i.message;
    }
    return {};
}

}  // namespace

int main() {
    // ---- 元素文件 ----
    {
        const auto r = sc::readElementFile(kDir + "/cabinet.terra-element.json");
        check(r.document.has_value(), "配电柜的元素文件可以读");
        if (r.document) {
            const auto& e = *r.document;
            check(e.name == "测试配电柜（两个盒体，带热点）" && e.category == "substation/cabinet" && e.collision == sc::ElementCollision::Boxes &&
                      e.boxes.size() == 2 && e.generatedBy == "manual" && e.movable == sc::ElementMovable::Layout,
                  "名字、类别、两个盒体、layout");
            check(e.thermalDefault && e.thermalDefault->at(0) == 45.0 && e.hotspots.size() == 1 && e.hotspots[0].temperature.at(0) == 90.0,
                  "默认温度 45 °C，一个 90 °C 的热点");
            check(e.inspection.size() == 1 && e.inspection[0].name == "仪表" && e.inspection[0].distance == 1.5, "检查点");
            check(e.licenseName == "CC0-1.0" && e.licenseAuthor == "EAI Terra", "许可");
        }
        const auto mesh = sc::readElementFile(kDir + "/plinth.terra-element.json");
        check(mesh.document && mesh.document->collision == sc::ElementCollision::Mesh && mesh.document->meshes.size() == 1 &&
                  mesh.document->meshes[0].file == kDir + "/block.obj" && mesh.document->movable == sc::ElementMovable::Fixed,
              "碰撞网格：路径相对元素文件，fixed");
        const auto crate = sc::readElementFile(kDir + "/crate.terra-element.json");
        if (crate.document) {
            const std::string xml = sc::elementMjcf(*crate.document);
            check(xml.find("<freejoint") != std::string::npos && xml.find("mass=\"2\"") != std::string::npos && xml.find("<option") == std::string::npos,
                  "dynamic：生成的 MJCF 有自由关节、质量 2 kg，没有 option");
        }
        const auto cabinet = sc::readElementFile(kDir + "/cabinet.terra-element.json");
        if (cabinet.document) check(sc::elementMjcf(*cabinet.document).find("<freejoint") == std::string::npos, "layout：没有自由关节（静止）");
    }
    // ---- 元素文件的错误 ----
    check(elementError(R"("physics": {"movable": "dynamic", "mass": 1}})").find("collision") != std::string::npos, "缺少 collision：报错");
    check(!elementError(R"("collision": {"mode": "auto-boxes"}})").empty(), "mode 为 auto-boxes（只由导入生成）：报错");
    check(!elementError(R"("collision": {"mode": "boxes", "boxes": []}})").empty(), "boxes 为空：报错");
    check(!elementError(R"("collision": {"mode": "boxes", "boxes": [{"halfSize": [0.1, 0, 0.1]}]}})").empty(), "半边长不大于 0：报错");
    check(!elementError(R"("collision": {"mode": "mesh", "meshes": [{"file": "nothing.obj"}]}})").empty(), "碰撞网格不存在：报错");
    check(!elementError(R"("collision": {"mode": "boxes", "boxes": [{"halfSize": [1, 1, 1]}]}, "physics": {"movable": "dynamic"}})").empty(),
          "dynamic 没有质量：报错");
    check(!elementError(R"("collision": {"mode": "none"}, "physics": {"movable": "dynamic", "mass": 1}})").empty(), "dynamic 没有碰撞体：报错");
    check(!elementError(R"("collision": {"mode": "none"}, "physics": {"movable": "rolling"}})").empty(), "未知的 movable：报错");
    check(!elementError(R"("collision": {"mode": "none"}, "colour": 1})").empty(), "未知字段：报错");
    check(sceneError(R"({"id": "x", "element": "nothing.terra-element.json"})").find("元素文件不存在") != std::string::npos, "元素文件不存在：报错");
    check(!sceneError(R"({"id": "1x", "element": "crate.terra-element.json"})").empty(), "id 不合法：报错");
    check(!sceneError(R"({"id": "ground", "element": "crate.terra-element.json"})").empty(), "id 与模型实例重名：报错");
    check(!sceneError(R"({"id": "a", "element": "crate.terra-element.json"}, {"id": "a", "element": "crate.terra-element.json"})").empty(), "id 重复：报错");

    // ---- 场景 ----
    auto loaded = sc::loadScene(kDir + "/elements_bench.terra-scene.json", terra::physics::mujoco::composeWorld);
    for (const auto& i : loaded.issues) std::fprintf(stderr, "场景：%s %s\n", i.where.c_str(), i.message.c_str());
    if (!loaded.scene) return 1;
    auto& scene = *loaded.scene;
    auto& world = *scene.world;
    const auto& d = world.description();
    bool globalWarning = false;
    for (const auto& i : loaded.issues) globalWarning = globalWarning || i.message.find("全局物理") != std::string::npos || i.message.find("全局设置") != std::string::npos;
    check(!globalWarning, "元素的全局物理设置随模型实例，没有“与第一个实例不同”的警告");
    check(d.modelInstances.size() == 6 && d.modelInstances[1].name == "cabinet" && d.modelInstances[5].name == "crate_b", "6 个实例：地面与 5 个元素（元素在后）");
    const auto* crateA = d.findEntity("crate_a/body");
    const auto* crateB = d.findEntity("crate_b/body");
    const auto* cabinet = d.findEntity("cabinet/body");
    const auto* plinth = d.findEntity("plinth/body");
    const auto* decal = d.findEntity("decal/body");
    check(crateA && crateB && cabinet && plinth && decal, "元素的刚体 “<id>/body”");
    if (!(crateA && crateB && cabinet && plinth && decal)) return 1;
    check(std::fabs(crateA->mass - 2.0) < 1e-9, "dynamic：木箱质量 2 kg");
    std::size_t decalGeoms = 0, cabinetGeoms = 0;
    for (const auto& g : d.geoms) {
        decalGeoms += g.entity == decal->id ? 1u : 0u;
        cabinetGeoms += g.entity == cabinet->id ? 1u : 0u;
    }
    check(decalGeoms == 0 && cabinetGeoms == 2, "只显示的元素没有几何体；配电柜两个盒体");

    const auto poseOf = [&](const td::EntityDescription* e) { return world.snapshot()->entities[e->id.value - 1].pose; };
    const auto cab0 = poseOf(cabinet);
    check(std::fabs(cab0.position.y - 1.5) < 1e-12 && std::fabs(poseOf(crateA).position.z - 0.5) < 1e-12, "初始位姿 = 放置位姿");
    for (int i = 0; i < 1000; ++i) world.step();  // 2 s
    const auto a = poseOf(crateA);
    const auto b = poseOf(crateB);
    std::printf("INFO 2 s 后：木箱 A z = %.4f m（地面），木箱 B z = %.4f m（基座顶面 0.5 m）\n", a.position.z, b.position.z);
    check(std::fabs(a.position.z) < 3e-3 && std::fabs(a.position.x) < 1e-3, "木箱 A 落到地面上（底面 z = 0）");
    check(std::fabs(b.position.z - 0.5) < 3e-3 && std::fabs(b.position.x - 2.0) < 1e-3, "木箱 B 落到碰撞网格的基座上（顶面 0.5 m）");
    const auto cab1 = poseOf(cabinet);
    check(cab1.position.x == cab0.position.x && cab1.position.y == cab0.position.y && cab1.position.z == cab0.position.z, "layout 元素仿真时静止");

    // ---- 温度与类别 ----
    const auto& t = scene.thermal;
    check(t.base(cabinet->id, 0) == 45.0, "配电柜：元素的默认温度 45 °C");
    check(t.base(plinth->id, 0) == 33.0, "基座：场景 temperatures 的 33 °C（元素没有默认温度）");
    check(t.base(crateA->id, 0) == 25.0 && t.base(crateB->id, 0) == 60.0, "木箱：元素默认 25 °C；实例覆盖为 60 °C");
    check(t.hotspots.size() == 1 && t.hotspots[0].entity == cabinet->id && t.hotspots[0].temperature.at(0) == 90.0, "元素文件的热点挂在刚体上");
    const std::string labels = sc::segmentationLabelsJson(scene);
    check(labels.find("\"entity\":\"cabinet/body\",\"instance\":\"cabinet\",\"label\":\"substation/cabinet\"") != std::string::npos,
          "分割类别取元素的 category");
    check(labels.find("\"entity\":\"crate_b/body\",\"instance\":\"crate_b\",\"label\":\"crate-special\"") != std::string::npos, "场景 labels 优先");

    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
