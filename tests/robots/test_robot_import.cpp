// 参考机器人导入核查（M3-01）。经 Terra 场景文件（assets/scenes/robots/）在 MuJoCo 4.0.1 下加载三台参考机器人
// （MuJoCo Menagerie，由 scripts/assets/fetch_robots.sh 准备；缺失时以返回码 77 表示跳过），检查：
//   - 加载与组合没有错误（警告打印出来，供核查表记录）；
//   - 实体、关节（含自由关节）、执行器数量与 MJCF 一致；
//   - 几何体分组：Menagerie 约定可见几何体在 group 2、碰撞几何体在 group 3；网格已读入；
//   - 执行器类别（M3-02）：G1 为 29 个位置执行器（kp 500），Go2 为 12 个力矩执行器，Stretch 为 2 个速度 + 8 个位置执行器；
//   - 关键帧（M3-02）：数量与名字；场景的初始关键帧已应用（G1 骨盆高 0.79 m、Go2 机身高 0.27 m）；
//   - 零控制推进 500 步：没有仿真错误，状态有限；
//   - M3-04b：Go2 场景的 PD 层接管 12 个力矩执行器，从 home 开始保持 1 s 后机身仍在 0.2 m 以上（不塌）；
//     G1 悬挂场景（fixedBase）没有自由关节，关键帧 stand 仍可用（左肘 1.28 rad），骨盆固定在 0.793 + 0.3 m。
// 并打印核查表（步长、积分器、求解器、实体、关节、执行器、几何体与网格），写入 M3-01 报告。本文件不包含 MuJoCo 头文件。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <map>
#include <string>

#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/scene/scene.hpp"
#include "terra/simulation/pd_layer.hpp"

#ifndef TERRA_ASSETS_DIR
#error "需要 TERRA_ASSETS_DIR"
#endif

namespace {

namespace fs = std::filesystem;
namespace sc = terra::scene;
namespace td = terra::data;

constexpr int kSkipped = 77;
int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

struct Expected {
    const char* scene;  // assets/scenes/robots/<scene>.terra-scene.json
    const char* folder; // assets/robots/<folder>/
    std::size_t entities;
    std::size_t joints;  // 含自由关节
    std::size_t freeJoints;
    std::size_t actuators;
    std::size_t motors, positions, velocities;  // 执行器类别（M3-02）
    std::size_t keyframes;
    const char* initialKeyframe;   // 场景文件中的 initialKeyframe
    const char* rootEntity;        // 浮动基座；initialZ < 0 表示不检查
    double initialZ;
};

// 数量来自 MJCF 文本（机器人文件加 Menagerie 的 scene.xml；Stretch 的场景另有桌子与两个可抓取物体，共 3 个 body、2 个自由关节）
// 关键帧：G1 stand（qpos 骨盆高 0.79）；Go2 home（机身高 0.27）；Stretch home 与 stow（只给控制量）
constexpr Expected kRobots[] = {
    {"g1", "unitree_g1", 30, 30, 1, 29, 0, 29, 0, 1, "g1/stand", "g1/pelvis", 0.79},
    {"go2", "unitree_go2", 13, 13, 1, 12, 12, 0, 0, 1, "go2/home", "go2/base", 0.27},
    {"stretch3", "hello_robot_stretch_3", 40, 23, 3, 10, 0, 8, 2, 2, "stretch3/home", "stretch3/base_link", -1.0},
};

const char* jointTypeName(td::JointType t) {
    switch (t) {
        case td::JointType::Free: return "自由";
        case td::JointType::Ball: return "球";
        case td::JointType::Slide: return "滑动";
        case td::JointType::Hinge: return "转动";
    }
    return "?";
}

void inspect(const Expected& e) {
    // 对照（只记录）：不经过组合、直接由 MuJoCo 加载 Menagerie 的 scene.xml
    {
        const auto direct = terra::physics::mujoco::loadWorld(
            td::ModelSource{std::string{TERRA_ASSETS_DIR} + "/robots/" + e.folder + "/scene.xml"});
        std::printf("INFO [%s] 直接加载 scene.xml（不经组合）：%s\n", e.scene,
                    direct.world ? "成功" : ("失败：" + (direct.error ? direct.error->message : std::string{})).c_str());
    }
    const std::string path = std::string{TERRA_ASSETS_DIR} + "/scenes/robots/" + e.scene + ".terra-scene.json";
    auto loaded = sc::loadScene(path, terra::physics::mujoco::composeWorld);
    for (const auto& issue : loaded.issues) {
        std::printf("INFO [%s] %s %s：%s\n", e.scene, issue.severity == sc::SceneIssue::Severity::Error ? "错误" : "警告",
                    issue.where.c_str(), issue.message.c_str());
    }
    check(loaded.ok(), std::string{e.scene} + "：经场景文件加载成功");
    if (!loaded.ok()) return;
    terra::physics::PhysicsWorld& world = *loaded.scene->world;
    const td::ModelDescription& d = world.description();

    std::map<std::string, int> jointTypes;
    std::size_t freeJoints = 0;
    for (const auto& j : d.joints) {
        ++jointTypes[jointTypeName(j.type)];
        freeJoints += j.type == td::JointType::Free ? 1 : 0;
    }
    std::size_t jointTransmission = 0;
    std::size_t limitedInputs = 0;
    for (const auto& a : d.actuators) {
        jointTransmission += a.transmission == td::ActuatorTransmission::Joint ? 1 : 0;
        for (const auto& in : a.inputs) limitedInputs += in.limited ? 1 : 0;
    }
    std::map<int, int> groups;
    std::size_t colliding = 0;
    for (const auto& g : d.geoms) {
        ++groups[g.group];
        colliding += g.collides ? 1 : 0;
    }
    std::size_t triangles = 0;
    for (const auto& m : d.meshes) triangles += m.triangles.size();

    std::string jointText;
    for (const auto& [name, n] : jointTypes) jointText += (jointText.empty() ? "" : "、") + name + " " + std::to_string(n);
    std::string groupText;
    for (const auto& [g, n] : groups) groupText += (groupText.empty() ? "" : "、") + std::string{"group "} + std::to_string(g) + "：" + std::to_string(n);
    std::printf("INFO [%s] 步长 %.4g s，积分器 %s，求解器 %s（迭代 %d），重力 %.2f\n", e.scene, d.timestep, d.settings.integrator.c_str(),
                d.settings.solver.c_str(), d.settings.iterations, d.settings.gravity.z);
    std::printf("INFO [%s] 实体 %zu，关节 %zu（%s），执行器 %zu（关节传动 %zu，其他传动 %zu；有范围的输入 %zu）\n", e.scene,
                d.entities.size(), d.joints.size(), jointText.c_str(), d.actuators.size(), jointTransmission,
                d.actuators.size() - jointTransmission, limitedInputs);
    std::printf("INFO [%s] 几何体 %zu（%s；参与碰撞 %zu），网格 %zu 个、%zu 个三角形\n", e.scene, d.geoms.size(), groupText.c_str(),
                colliding, d.meshes.size(), triangles);

    check(d.entities.size() == e.entities, std::string{e.scene} + "：实体 " + std::to_string(d.entities.size()) + "，应为 " +
                                               std::to_string(e.entities));
    check(d.joints.size() == e.joints, std::string{e.scene} + "：关节 " + std::to_string(d.joints.size()) + "，应为 " +
                                           std::to_string(e.joints));
    check(freeJoints == e.freeJoints, std::string{e.scene} + "：自由关节 " + std::to_string(freeJoints) + "，应为 " +
                                          std::to_string(e.freeJoints));
    check(d.actuators.size() == e.actuators, std::string{e.scene} + "：执行器 " + std::to_string(d.actuators.size()) + "，应为 " +
                                                 std::to_string(e.actuators));
    check(groups[2] > 0 && groups[3] > 0, std::string{e.scene} + "：可见几何体在 group 2、碰撞几何体在 group 3（Menagerie 约定）");
    check(!d.meshes.empty() && triangles > 0, std::string{e.scene} + "：网格已读入");
    for (const auto& a : d.actuators) {
        check(a.name.rfind(std::string{e.scene} + "/", 0) == 0, std::string{e.scene} + "：执行器名带实例前缀：" + a.name);
    }

    // M3-02：执行器类别与关键帧
    std::size_t motors = 0, positions = 0, velocities = 0;
    double minKp = 1e300, maxKp = 0;
    for (const auto& a : d.actuators) {
        motors += a.kind == td::ActuatorKind::Motor ? 1 : 0;
        velocities += a.kind == td::ActuatorKind::Velocity ? 1 : 0;
        if (a.kind == td::ActuatorKind::Position) {
            ++positions;
            minKp = std::min(minKp, a.gain);
            maxKp = std::max(maxKp, a.gain);
        }
    }
    std::size_t limitedJoints = 0;
    for (const auto& j : d.joints) limitedJoints += j.limited ? 1 : 0;
    std::string keyText;
    for (const auto& k : d.keyframes) keyText += (keyText.empty() ? "" : "、") + k.name;
    std::printf("INFO [%s] 执行器类别：力矩 %zu、位置 %zu（kp %g–%g）、速度 %zu、其他 %zu；有限位的关节 %zu；关键帧 %zu 个（%s）\n", e.scene,
                motors, positions, positions ? minKp : 0.0, maxKp, velocities, d.actuators.size() - motors - positions - velocities,
                limitedJoints, d.keyframes.size(), keyText.c_str());
    check(motors == e.motors && positions == e.positions && velocities == e.velocities,
          std::string{e.scene} + "：执行器类别（力矩 / 位置 / 速度）为 " + std::to_string(e.motors) + " / " + std::to_string(e.positions) +
              " / " + std::to_string(e.velocities));
    if (std::string{e.scene} == "g1") check(minKp == 500.0 && maxKp == 500.0, "g1：位置执行器 kp 均为 500");
    check(d.keyframes.size() == e.keyframes, std::string{e.scene} + "：关键帧 " + std::to_string(d.keyframes.size()) + " 个，应为 " +
                                                 std::to_string(e.keyframes));
    check(loaded.scene->homeKeyframe && d.keyframes[*loaded.scene->homeKeyframe].name == e.initialKeyframe,
          std::string{e.scene} + "：场景的初始关键帧为 " + e.initialKeyframe);
    if (e.initialZ >= 0) {
        const auto* root = d.findEntity(e.rootEntity);
        check(root != nullptr, std::string{e.scene} + "：找到浮动基座 " + e.rootEntity);
        if (root) {
            const double z = world.snapshot()->entities[root->id.value - 1].pose.position.z;
            std::printf("INFO [%s] 初始关键帧后 %s 高 %.4f m\n", e.scene, e.rootEntity, z);
            check(std::fabs(z - e.initialZ) < 1e-9, std::string{e.scene} + "：初始关键帧已应用（" + e.rootEntity + " 高 " +
                                                        std::to_string(e.initialZ) + " m）");
        }
    }

    bool stepped = true;
    for (int k = 0; k < 500 && stepped; ++k) {
        if (const auto error = world.step()) {
            check(false, std::string{e.scene} + "：零控制推进第 " + std::to_string(error->step) + " 步出错：" + error->message);
            stepped = false;
        }
    }
    if (stepped) check(world.snapshot()->diagnostics.finite, std::string{e.scene} + "：零控制推进 500 步后状态有限");
}

void inspectControlScenes() {
    // Go2：PD 层保持 home 姿态
    {
        auto loaded = sc::loadScene(std::string{TERRA_ASSETS_DIR} + "/scenes/robots/go2.terra-scene.json", terra::physics::mujoco::composeWorld);
        check(loaded.ok(), "go2：经场景文件加载（含 PD 层）");
        if (loaded.ok()) {
            sc::SceneInstance& s = *loaded.scene;
            check(s.pd.size() == 12, "go2：PD 层接管 12 个力矩执行器，实际 " + std::to_string(s.pd.size()));
            terra::simulation::PdLayer pd(s.world->description(), s.pd);
            const auto* base = s.world->description().findEntity("go2/base");
            bool ok = true;
            for (int k = 0; k < 500 && ok; ++k) {
                const auto commands = pd.process({}, *s.world->snapshot());
                ok = !s.world->setControl(commands) && !s.world->step();
            }
            check(ok, "go2：PD 层控制下推进 500 步没有错误");
            if (base && ok) {
                const double z = s.world->snapshot()->entities[base->id.value - 1].pose.position.z;
                std::printf("INFO [go2] PD 层（kp 50，kd 0.5）从 home 保持 1 s 后机身高 %.4f m（初始 0.27）\n", z);
                check(z > 0.2, "go2：PD 层保持 home 姿态，1 s 后机身仍高于 0.2 m（实际 " + std::to_string(z) + "）");
            }
        }
    }
    // G1 悬挂模式
    {
        auto loaded = sc::loadScene(std::string{TERRA_ASSETS_DIR} + "/scenes/robots/g1_suspended.terra-scene.json",
                                    terra::physics::mujoco::composeWorld);
        for (const auto& issue : loaded.issues) std::printf("INFO [g1_suspended] %s\n", issue.message.c_str());
        check(loaded.ok(), "g1_suspended：经场景文件加载");
        if (!loaded.ok()) return;
        sc::SceneInstance& s = *loaded.scene;
        const auto& d = s.world->description();
        std::size_t freeCount = 0;
        for (const auto& j : d.joints) freeCount += j.type == td::JointType::Free ? 1u : 0u;
        check(d.joints.size() == 29 && freeCount == 0, "g1_suspended：29 个关节，没有自由关节（实际 " + std::to_string(d.joints.size()) + "）");
        check(s.homeKeyframe.has_value(), "g1_suspended：初始关键帧 g1/stand 可用");
        if (const auto* elbow = d.findJoint("g1/left_elbow_joint")) {
            const double q = s.world->snapshot()->joints[elbow->id.value - 1].position[0];
            check(std::fabs(q - 1.28) < 1e-9, "g1_suspended：关键帧 stand 已应用（左肘 1.28 rad，实际 " + std::to_string(q) + "）");
        }
        const auto* pelvis = d.findEntity("g1/pelvis");
        bool ok = true;
        for (int k = 0; k < 500 && ok; ++k) ok = !s.world->step();
        check(ok, "g1_suspended：推进 500 步没有错误");
        if (pelvis) {
            const auto p = s.world->snapshot()->entities[pelvis->id.value - 1].pose.position;
            std::printf("INFO [g1_suspended] 推进 1 s 后骨盆位于 (%.4f, %.4f, %.4f)\n", p.x, p.y, p.z);
            check(std::fabs(p.x) < 1e-12 && std::fabs(p.y) < 1e-12 && std::fabs(p.z - 1.093) < 1e-9, "g1_suspended：骨盆固定在 (0, 0, 1.093)");
        }
    }
}

}  // namespace

int main() {
    for (const Expected& e : kRobots) {
        if (!fs::exists(std::string{TERRA_ASSETS_DIR} + "/robots/" + e.folder + "/scene.xml")) {
            std::printf("INFO 参考机器人未准备（运行 scripts/assets/fetch_robots.sh），跳过\n");
            return kSkipped;
        }
    }
    for (const Expected& e : kRobots) inspect(e);
    inspectControlScenes();
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
