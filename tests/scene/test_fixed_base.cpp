// 悬挂模式（M3-04b，场景文件 models[].fixedBase）：组合时删掉实例根部的自由关节。
// 夹具 assets/tests/robot_description.xml（浮动基座 + 机械臂 + 轮子，无重力；关键帧 rest、raised）：
//   - 删掉自由关节后关节数 4 → 3，基座固定在放置位姿上（体位置 0.4 + 放置 1.0），轮子与机械臂推进 1000 步后基座不动；
//   - 关键帧仍在、名字带实例前缀，按关键帧 raised 重置后肩关节 0.5 rad、滑动 0.05 m，基座仍在原处；
//   - 同一场景中另一个不悬挂的实例照常有自由关节；
//   - 没有自由关节的模型（cart.xml）写 fixedBase：警告，不出错。

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

std::string dump(const std::vector<sc::SceneIssue>& issues) {
    std::string text;
    for (const auto& i : issues) text += std::string{"\n  "} + (i.severity == Severity::Error ? "错误 " : "警告 ") + i.where + "：" + i.message;
    return text;
}

sc::SceneLoadResult load(const std::string& models) {
    const std::string text = R"({"format": "terra.scene", "version": 1, "models": )" + models + "}";
    auto parsed = sc::parseScene(text, (fs::path{TERRA_TEST_ASSETS_DIR} / "scene" / "inline.terra-scene.json").string());
    check(parsed.document.has_value(), "内联场景解析" + dump(parsed.issues));
    if (!parsed.document) return {};
    return sc::instantiate(std::move(*parsed.document), terra::physics::mujoco::composeWorld);
}

td::Vec3 position(const sc::SceneInstance& s, const char* entity) {
    const auto* e = s.world->description().findEntity(entity);
    check(e != nullptr, std::string{"找到实体 "} + entity);
    return e ? s.world->snapshot()->entities[e->id.value - 1].pose.position : td::Vec3{};
}

bool near(const td::Vec3& a, const td::Vec3& b, double tol = 1e-12) {
    return std::fabs(a.x - b.x) <= tol && std::fabs(a.y - b.y) <= tol && std::fabs(a.z - b.z) <= tol;
}

}  // namespace

int main() {
    auto loaded = load(R"([{"name": "fixed", "mjcf": "../robot_description.xml", "position": [0, 0, 1], "fixedBase": true},
                           {"name": "free", "mjcf": "../robot_description.xml", "position": [2, 0, 0]}])");
    check(loaded.ok(), "组合悬挂与不悬挂的两个实例" + dump(loaded.issues));
    if (loaded.ok()) {
        sc::SceneInstance& s = *loaded.scene;
        const auto& d = s.world->description();
        std::size_t fixedJoints = 0, freeJoints = 0, fixedFree = 0;
        for (const auto& j : d.joints) {
            const bool fixed = j.name.rfind("fixed/", 0) == 0;
            fixedJoints += fixed ? 1u : 0u;
            freeJoints += fixed ? 0u : 1u;
            fixedFree += (fixed && j.type == td::JointType::Free) ? 1u : 0u;
        }
        check(fixedJoints == 3 && fixedFree == 0, "悬挂实例：自由关节被删掉，剩 3 个关节（实际 " + std::to_string(fixedJoints) + "）");
        check(freeJoints == 4 && d.findJoint("free/root") != nullptr, "不悬挂的实例：4 个关节，含自由关节");
        std::string keys;
        for (const auto& k : d.keyframes) keys += (keys.empty() ? "" : "、") + k.name;
        std::printf("INFO 关键帧：%s\n", keys.c_str());
        check(d.findKeyframe("fixed/raised") >= 0 && d.findKeyframe("fixed/rest") >= 0 && d.findKeyframe("free/raised") >= 0,
              "关键帧仍在且带实例前缀：" + keys);

        const td::Vec3 base0 = position(s, "fixed/base");
        check(near(base0, {0, 0, 1.4}), "悬挂的基座在放置位姿上 (0, 0, 1.4)，实际 (" + std::to_string(base0.x) + ", " +
                                            std::to_string(base0.y) + ", " + std::to_string(base0.z) + ")");
        const auto* spin = d.findActuator("fixed/spin_vel");
        const auto* shoulder = d.findActuator("fixed/shoulder_pos");
        if (spin && shoulder) (void)s.world->setControl({{spin->id, {8.0}}, {shoulder->id, {0.9}}});
        for (int k = 0; k < 1000; ++k) check(!s.world->step(), "推进");
        check(near(position(s, "fixed/base"), base0), "轮子转动、机械臂摆动 1000 步后，悬挂的基座不动");

        const int raised = d.findKeyframe("fixed/raised");
        if (raised >= 0) {
            check(s.world->resetToKeyframe(static_cast<std::size_t>(raised)), "按关键帧 fixed/raised 重置");
            const auto* sj = d.findJoint("fixed/shoulder");
            const auto* ej = d.findJoint("fixed/extend");
            if (sj && ej) {
                const auto snap = s.world->snapshot();
                const double q = snap->joints[sj->id.value - 1].position[0];
                const double e = snap->joints[ej->id.value - 1].position[0];
                check(std::fabs(q - 0.5) < 1e-12 && std::fabs(e - 0.05) < 1e-12,
                      "关键帧中其余关节的值保留：肩 0.5 rad（实际 " + std::to_string(q) + "）、滑动 0.05 m（实际 " + std::to_string(e) + "）");
            }
            check(near(position(s, "fixed/base"), base0), "按关键帧重置后基座仍在原处");
        }
    }

    auto noFree = load(R"([{"name": "cart", "mjcf": "cart.xml", "fixedBase": true}])");
    bool warned = false;
    for (const auto& i : noFree.issues) warned = warned || (i.severity == Severity::Warning && i.message.find("没有自由关节") != std::string::npos);
    check(noFree.ok() && warned, "没有自由关节的模型写 fixedBase：只警告" + dump(noFree.issues));

    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
