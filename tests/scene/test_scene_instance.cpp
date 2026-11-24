// 场景实例与视觉部件（M2-05）。用 MuJoCo 后端加载夹具 two_carts.terra-scene.json：
//   - 视觉部件的编号与顺序（先几何体后 glTF 绑定）、所属实体与实例；
//   - 绑定了 glTF 的实体默认隐藏 MJCF 几何体，showGeoms 保留；静态地面不受 "world" 绑定影响；
//   - 视觉部件的世界位姿：几何体、node 对齐（带局部偏移、放置旋转）、assembly 对齐（初始时为单位变换，随实体位移）、
//     静态装饰；实体运动后随之更新；
//   - 诊断：实体名没带前缀、实体不存在（列出该实例的实体）、重复绑定、glTF 文件缺失（警告并忽略绑定）。
// 本文件不包含 MuJoCo 头文件。

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>

#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/scene/scene.hpp"

#ifndef TERRA_TEST_ASSETS_DIR
#error "需要 TERRA_TEST_ASSETS_DIR"
#endif

namespace {

namespace fs = std::filesystem;
namespace sc = terra::scene;
namespace td = terra::data;
using Severity = sc::SceneIssue::Severity;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

const std::string kDir = (fs::path{TERRA_TEST_ASSETS_DIR} / "scene").lexically_normal().string();

std::string dump(const std::vector<sc::SceneIssue>& issues) {
    std::string text;
    for (const auto& i : issues) {
        text += std::string{"\n  "} + (i.severity == Severity::Error ? "错误 " : "警告 ") + i.where + "：" + i.message;
    }
    return text;
}

bool has(const std::vector<sc::SceneIssue>& issues, Severity severity, const std::string& where, const std::string& part) {
    for (const auto& i : issues) {
        if (i.severity == severity && i.where == where && i.message.find(part) != std::string::npos) return true;
    }
    return false;
}

bool near(const td::Vec3& a, const td::Vec3& b, double tol = 1e-9) {
    return std::fabs(a.x - b.x) <= tol && std::fabs(a.y - b.y) <= tol && std::fabs(a.z - b.z) <= tol;
}

std::string str(const td::Vec3& v) {
    char buf[96];
    std::snprintf(buf, sizeof buf, "(%.6g, %.6g, %.6g)", v.x, v.y, v.z);
    return buf;
}

const sc::VisualPart* geomPart(const sc::SceneInstance& s, const char* geomName) {
    const auto* g = s.world->description().findGeom(geomName);
    check(g != nullptr, std::string{"找到几何体 "} + geomName);
    if (g == nullptr) return nullptr;
    for (const auto& v : s.visuals) {
        if (v.kind == sc::VisualPart::Kind::Geom && v.geom == g->id) return &v;
    }
    return nullptr;
}

const sc::VisualPart* bindingPart(const sc::SceneInstance& s, int binding) {
    for (const auto& v : s.visuals) {
        if (v.kind == sc::VisualPart::Kind::Gltf && v.binding == binding) return &v;
    }
    check(false, "找到绑定 " + std::to_string(binding) + " 的视觉部件");
    return nullptr;
}

void testFixture() {
    auto loaded = sc::loadScene(kDir + "/two_carts.terra-scene.json", terra::physics::mujoco::composeWorld);
    check(loaded.ok() && loaded.issues.empty(), "加载夹具场景" + dump(loaded.issues));
    if (!loaded.ok()) return;
    sc::SceneInstance& s = *loaded.scene;
    const auto& d = s.world->description();
    check(s.bindings.size() == 4, "4 个 glTF 绑定");
    check(s.visuals.size() == d.geoms.size() + 4, "视觉部件 = 几何体 " + std::to_string(d.geoms.size()) + " 个 + 绑定 4 个");
    for (std::size_t i = 0; i < s.visuals.size(); ++i) {
        check(s.visuals[i].id.value == i + 1 && s.visual(s.visuals[i].id) == &s.visuals[i], "VisualId 从 1 连续");
        const bool geomFirst = i < d.geoms.size();
        check((s.visuals[i].kind == sc::VisualPart::Kind::Geom) == geomFirst, "先几何体，后 glTF 绑定");
    }

    // 隐藏规则
    const auto* hullA = geomPart(s, "cart_a/hull");
    const auto* poleA = geomPart(s, "cart_a/pole");
    const auto* hullB = geomPart(s, "cart_b/hull");
    const auto* poleB = geomPart(s, "cart_b/pole");
    const auto* groundPart = geomPart(s, "ground/floor");
    if (hullA && poleA && hullB && poleB && groundPart) {
        check(hullA->hiddenByBinding, "cart_a/chassis 绑定了 glTF：其几何体默认隐藏");
        check(poleA->hiddenByBinding, "cart_a/mast 绑定了 glTF：其几何体默认隐藏");
        check(!hullB->hiddenByBinding, "cart_b/chassis 的绑定要求 showGeoms：几何体保留");
        check(!poleB->hiddenByBinding, "cart_b/mast 没有绑定：几何体显示");
        check(!groundPart->hiddenByBinding && !groundPart->entity.valid(), "静态地面不受 “world” 绑定影响");
        check(hullA->modelInstance == d.findEntity("cart_a/chassis")->modelInstance, "几何体部件的实例");
    }
    const auto* b0 = bindingPart(s, 0);
    const auto* b3 = bindingPart(s, 3);
    if (b0 && b3) {
        check(b0->entity == d.findEntity("cart_a/chassis")->id && b0->modelInstance.value == 2, "绑定 0 属于 cart_a/chassis，实例 2");
        check(!b3->entity.valid() && !b3->modelInstance.valid(), "绑定 3 为静态装饰");
    }

    // 初始位姿
    auto poses = sc::visualPoses(s, sc::entityPoses(*s.world->snapshot()));
    check(poses.size() == s.visuals.size(), "每个视觉部件一个位姿");
    const auto poseOf = [&](const sc::VisualPart* p) { return p != nullptr ? poses[p->id.value - 1] : sc::VisualPose{}; };
    if (hullA) check(near(poseOf(hullA).pose.position, {0, -0.6, 0.1}), "cart_a 车身几何体：实体 (0, −0.6, 0.1) × 局部 (0, 0, 0)");
    if (poleA) check(near(poseOf(poleA).pose.position, {0, -0.6, 0.2}), "cart_a 桅杆几何体：子实体 + 0.1");
    if (b0) check(near(poseOf(b0).pose.position, {0, -0.6, 0.1}) && poseOf(b0).scale == 1.0, "node 对齐：与实体重合");
    const auto* b1 = bindingPart(s, 1);
    if (b1) {
        check(near(poseOf(b1).pose.position, {0, 0, 0}) && std::fabs(poseOf(b1).pose.orientation.w - 1) < 1e-12,
              "assembly 对齐：初始姿态下为单位变换（glTF 中的装配位置原样显示），实际 " + str(poseOf(b1).pose.position));
    }
    const auto* b2 = bindingPart(s, 2);
    if (b2) check(near(poseOf(b2).pose.position, {0, 0.6, 0.11}), "cart_b 的绑定：局部 +0.01（Z 不受偏航影响）");
    if (const auto* bumperB = geomPart(s, "cart_b/bumper")) {
        check(near(poseOf(bumperB).pose.position, {0, 0.82, 0.1}),
              "cart_b 的保险杠几何体：车身 (0, 0.6, 0.1) + 偏航 90° 后的局部偏移 (0.22, 0, 0) → (0, 0.82, 0.1)，实际 " +
                  str(poseOf(bumperB).pose.position));
    }
    if (b3) check(near(poseOf(b3).pose.position, {2, 0, 0}), "静态装饰：位于 (2, 0, 0)");

    // 推动 cart_a，视觉部件随实体移动
    const auto* drive = d.findActuator("cart_a/drive");
    check(drive != nullptr, "执行器 cart_a/drive");
    if (drive == nullptr) return;
    (void)s.world->setControl({{drive->id, {3.0}}});
    for (int i = 0; i < 400; ++i) {
        if (s.world->step()) {
            check(false, "步进出错");
            return;
        }
    }
    const td::Pose chassis = s.world->snapshot()->entities[d.findEntity("cart_a/chassis")->id.value - 1].pose;
    const double dx = chassis.position.x;
    check(dx > 0.05, "cart_a 前进了 " + std::to_string(dx) + " m");
    poses = sc::visualPoses(s, sc::entityPoses(*s.world->snapshot()));
    if (b0) check(near(poseOf(b0).pose.position, chassis.position), "node 对齐的绑定随车身移动");
    if (b1) check(near(poseOf(b1).pose.position, {dx, 0, 0}, 1e-9), "assembly 对齐：整体平移 dx，实际 " + str(poseOf(b1).pose.position));
    if (b3) check(near(poseOf(b3).pose.position, {2, 0, 0}), "静态装饰不动");
}

sc::SceneLoadResult fromText(const std::string& visuals) {
    const std::string text = R"({"format": "terra.scene", "version": 1,
        "models": [{"name": "cart_a", "mjcf": "cart.xml"}, {"name": "cart_b", "mjcf": "cart.xml", "position": [0, 1, 0]}],
        "visuals": )" + visuals + "}";
    auto parsed = sc::parseScene(text, kDir + "/inline.terra-scene.json");
    check(parsed.document.has_value(), "内联场景解析" + dump(parsed.issues));
    if (!parsed.document) return {};
    return sc::instantiate(std::move(*parsed.document), terra::physics::mujoco::composeWorld);
}

void testDiagnostics() {
    auto noPrefix = fromText(R"([{"entity": "chassis", "gltf": "cart.gltf"}])");
    check(!noPrefix.ok() && has(noPrefix.issues, Severity::Error, "/visuals/0/entity", "需要带实例前缀"), "实体名没带前缀" + dump(noPrefix.issues));
    auto unknown = fromText(R"([{"entity": "cart_b/wheel", "gltf": "cart.gltf"}])");
    check(!unknown.ok() && has(unknown.issues, Severity::Error, "/visuals/0/entity", "cart_b/chassis"),
          "实体不存在时列出该实例的实体" + dump(unknown.issues));
    auto noInstance = fromText(R"([{"entity": "cart_c/chassis", "gltf": "cart.gltf"}])");
    check(!noInstance.ok() && has(noInstance.issues, Severity::Error, "/visuals/0/entity", "没有实例前缀 “cart_c/”"), "实例不存在");
    auto duplicate = fromText(R"([{"entity": "cart_a/chassis", "gltf": "cart.gltf", "node": "Chassis"},
                                  {"entity": "cart_a/chassis", "gltf": "cart.gltf", "node": "Chassis", "scale": 2}])");
    check(!duplicate.ok() && has(duplicate.issues, Severity::Error, "/visuals/1", "与 /visuals/0 相同"), "重复绑定" + dump(duplicate.issues));
    auto sameEntityOtherNode = fromText(R"([{"entity": "cart_a/chassis", "gltf": "cart.gltf", "node": "Chassis"},
                                            {"entity": "cart_a/chassis", "gltf": "cart.gltf", "node": "Mast"}])");
    check(sameEntityOtherNode.ok() && sameEntityOtherNode.scene->bindings.size() == 2, "同一实体绑定不同节点（多个视觉部件）");
    auto missing = fromText(R"([{"entity": "cart_a/chassis", "gltf": "no_such.gltf"}])");
    check(missing.ok() && has(missing.issues, Severity::Warning, "/visuals/0/gltf", "glTF 文件不存在"), "glTF 缺失：警告" + dump(missing.issues));
    if (missing.ok()) {
        check(missing.scene->bindings.empty(), "缺失的绑定被忽略");
        const auto* hull = geomPart(*missing.scene, "cart_a/hull");
        check(hull != nullptr && !hull->hiddenByBinding, "绑定被忽略后，实体改为显示 MJCF 几何体");
    }
    auto none = fromText(R"([])");
    check(none.ok() && none.scene->bindings.empty(), "没有绑定的场景");

    auto absent = sc::loadScene(kDir + "/no_such.terra-scene.json", terra::physics::mujoco::composeWorld);
    check(!absent.ok() && !absent.issues.empty(), "场景文件不存在");
}

// 场景的初始关键帧（M3-02）：夹具 ../robot_description.xml（关键帧 rest、raised）放在 (0, 1, 0)
sc::SceneLoadResult keyframeScene(const std::string& extra, const std::string& mjcf = "../robot_description.xml") {
    const std::string text = R"({"format": "terra.scene", "version": 1,
        "models": [{"name": "arm", "mjcf": ")" + mjcf + R"(", "position": [0, 1, 0]}],
        "visuals": [{"entity": "arm/base", "gltf": "cart.gltf", "node": "Chassis", "frame": "assembly"}])" + extra + "}";
    auto parsed = sc::parseScene(text, kDir + "/inline.terra-scene.json");
    check(parsed.document.has_value(), "内联场景解析" + dump(parsed.issues));
    if (!parsed.document) return {};
    return sc::instantiate(std::move(*parsed.document), terra::physics::mujoco::composeWorld);
}

void testInitialKeyframe() {
    auto plain = keyframeScene("");
    check(plain.ok() && !plain.scene->homeKeyframe, "没写初始关键帧：从模型初值开始" + dump(plain.issues));

    auto raised = keyframeScene(R"(, "initialKeyframe": "arm/raised")");
    check(raised.ok(), "初始关键帧 arm/raised" + dump(raised.issues));
    if (raised.ok()) {
        const sc::SceneInstance& s = *raised.scene;
        const auto& d = s.world->description();
        check(s.homeKeyframe && *s.homeKeyframe == static_cast<std::size_t>(d.findKeyframe("arm/raised")), "记录初始关键帧的下标");
        check(s.world->stepIndex() == 0 && s.world->simulationTime() == 0.0, "应用初始关键帧后仍是第 0 步、时间 0");
        const auto* base = d.findEntity("arm/base");
        if (base) {
            const auto& pose = s.world->snapshot()->entities[base->id.value - 1].pose;
            check(near(pose.position, {0.2, 1, 0.5}), "基座在关键帧位姿（放置框架中的 (0.2, 0, 0.5)），实际 " + str(pose.position));
        }
        // assembly 对齐按关键帧姿态计算：初始时 glTF 装配保持文件中的摆放（世界原点）
        if (const auto* part = bindingPart(s, 0)) {
            const auto poses = sc::visualPoses(s, sc::entityPoses(*s.world->snapshot()));
            const auto& p = poses[part->id.value - 1].pose;
            check(near(p.position, {0, 0, 0}) && std::fabs(std::fabs(p.orientation.w) - 1) < 1e-9,
                  "assembly 对齐以初始关键帧姿态为基准，初始视觉位姿为单位变换，实际 " + str(p.position));
        }
    }

    auto unknown = keyframeScene(R"(, "initialKeyframe": "arm/nope")");
    check(!unknown.ok() && has(unknown.issues, Severity::Error, "/initialKeyframe", "arm/rest、arm/raised"),
          "找不到初始关键帧时列出场景中的关键帧" + dump(unknown.issues));
    auto noKeys = keyframeScene(R"(, "initialKeyframe": "arm/raised")", "cart.xml");
    check(!noKeys.ok() && has(noKeys.issues, Severity::Error, "/initialKeyframe", "场景中没有关键帧"),
          "模型没有关键帧" + dump(noKeys.issues));
}

}  // namespace

int main() {
    testFixture();
    testDiagnostics();
    testInitialKeyframe();
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
