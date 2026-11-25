// 宇树官方 MuJoCo 模型的导入核查（M4-05，unitree_mujoco 固定提交 1eb6642e，由 scripts/assets/fetch_robots.sh 准备；
// 缺失时以返回码 77 表示跳过）。经 Terra 场景文件（assets/scenes/robots/go2w、as2）加载，检查：
//   - 加载与组合没有错误；实体、关节（含自由关节）与执行器数量与 MJCF 一致（Go2W 17 / 17 / 16，AS2 17 / 13 / 12）；
//   - 执行器都是作用于单个关节的力矩执行器（宇树低层控制 LowCmd 需要力矩执行器，M4-05 设计稿第 4 节）；
//   - 腿部关节名齐全：FR、FL、RR、RL 各有 hip、thigh、calf（宇树配置按名字对应电机编号）；Go2W 另有四个轮子关节，没有限位；
//   - 模型中的 IMU（安装点 imu 上的加速度计与陀螺仪）被发现：go2w/imu、as2/imu；
//   - 可见几何体在默认显示的分组（Go2W 为 group 2，AS2 为 group 1）、碰撞几何体在 group 3；网格已读入；
//   - 零控制推进 2 s：没有仿真错误，状态有限；机身落下后停在地面以上（与官方仿真相同：没有命令时机器人趴下）。
// 并打印核查表，写入 M4-05 报告。本文件不包含 MuJoCo 头文件。

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <map>
#include <string>

#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/scene/scene.hpp"

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
    const char* scene;   // assets/scenes/robots/<scene>.terra-scene.json，也是实例名
    const char* folder;  // assets/robots/<folder>/
    const char* model;   // 机器人的 MJCF 文件名
    std::size_t entities;
    std::size_t joints;  // 含自由关节
    std::size_t actuators;
    bool wheels;
    double startZ;       // 模型中机身的初始高度（m）
};

constexpr Expected kRobots[] = {
    {"go2w", "unitree_go2w", "go2w.xml", 17, 17, 16, true, 0.6},
    {"as2", "unitree_as2", "as2.xml", 17, 13, 12, false, 0.5},
};

void inspect(const Expected& e) {
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
    const std::string prefix = std::string{e.scene} + "/";

    std::size_t freeJoints = 0;
    for (const auto& j : d.joints) freeJoints += j.type == td::JointType::Free ? 1 : 0;
    std::size_t motors = 0;
    for (const auto& a : d.actuators) {
        const bool motor = a.kind == td::ActuatorKind::Motor && a.transmission == td::ActuatorTransmission::Joint && a.joint.valid();
        motors += motor ? 1 : 0;
    }
    std::map<int, int> groups;
    for (const auto& g : d.geoms) ++groups[g.group];
    std::size_t triangles = 0;
    for (const auto& m : d.meshes) triangles += m.triangles.size();
    std::printf("INFO [%s] 步长 %.4g s，积分器 %s，求解器 %s（迭代 %d），重力 %.2f\n", e.scene, d.timestep, d.settings.integrator.c_str(),
                d.settings.solver.c_str(), d.settings.iterations, d.settings.gravity.z);
    std::printf("INFO [%s] 实体 %zu，关节 %zu（自由 %zu），执行器 %zu（力矩 %zu），几何体 %zu（group 2：%d，group 3：%d），网格 %zu 个、%zu 个三角形，关键帧 %zu，IMU %zu\n",
                e.scene, d.entities.size(), d.joints.size(), freeJoints, d.actuators.size(), motors, d.geoms.size(), groups[2], groups[3],
                d.meshes.size(), triangles, d.keyframes.size(), d.sensors.size());

    check(d.entities.size() == e.entities, std::string{e.scene} + "：实体 " + std::to_string(d.entities.size()) + "，应为 " + std::to_string(e.entities));
    check(d.joints.size() == e.joints && freeJoints == 1,
          std::string{e.scene} + "：关节 " + std::to_string(d.joints.size()) + "（应为 " + std::to_string(e.joints) + "），一个自由关节");
    check(d.actuators.size() == e.actuators && motors == e.actuators,
          std::string{e.scene} + "：" + std::to_string(e.actuators) + " 个执行器，全部是作用于单个关节的力矩执行器（实际 " + std::to_string(motors) + "）");
    check(groups[1] + groups[2] > 0 && groups[3] > 0 && !d.meshes.empty() && triangles > 0,
          std::string{e.scene} + "：可见几何体在默认显示的分组（Go2W 为 group 2，AS2 为 group 1）、碰撞几何体在 group 3，网格已读入");

    // 腿部关节名（宇树配置按名字对应电机编号：FR_0～2、FL_0～2、RR_0～2、RL_0～2）
    std::string missing;
    for (const char* leg : {"FR", "FL", "RR", "RL"}) {
        for (const char* part : {"hip", "thigh", "calf"}) {
            const std::string name = prefix + leg + "_" + part + "_joint";
            if (d.findJoint(name) == nullptr) missing += " " + name;
        }
        if (e.wheels) {
            const std::string wheel = prefix + leg + "_wheel_joint";
            const auto* j = d.findJoint(wheel);
            if (j == nullptr) missing += " " + wheel;
            else check(!j->limited && j->type == td::JointType::Hinge, wheel + "：转动关节，没有限位（可以连续转）");
        }
    }
    check(missing.empty(), std::string{e.scene} + "：腿部" + (e.wheels ? "与轮子" : "") + "关节名齐全" + (missing.empty() ? "" : "，缺少" + missing));

    const auto* imu = d.findSensor(prefix + "imu");
    check(imu != nullptr && !imu->added, std::string{e.scene} + "：模型中的 IMU（" + prefix + "imu）被发现");

    // 零控制推进 2 s：机身落下，停在地面以上
    const auto* base = d.findEntity(prefix + "base_link");
    check(base != nullptr, std::string{e.scene} + "：机身实体 base_link");
    bool ok = true;
    for (int i = 0; i < 1000 && ok; ++i) {
        if (const auto error = world.step()) {
            check(false, std::string{e.scene} + "：推进出错：" + error->message);
            ok = false;
        }
    }
    const auto snap = world.snapshot();
    check(snap->diagnostics.finite, std::string{e.scene} + "：状态有限");
    if (base != nullptr) {
        const double z = snap->entities[base->id.value - 1].pose.position.z;
        std::printf("INFO [%s] 零控制 2 s 后机身高度 %.3f m（初始 %.2f m），有效接触点 %u 个\n", e.scene, z, e.startZ, snap->contacts.activeCount);
        check(z > 0.03 && z < e.startZ, std::string{e.scene} + "：机身落下后停在地面以上");
        check(snap->contacts.activeCount > 0, std::string{e.scene} + "：与地面有接触");
    }
}

}  // namespace

int main() {
    for (const auto& e : kRobots) {
        if (!fs::exists(std::string{TERRA_ASSETS_DIR} + "/robots/" + e.folder + "/" + e.model)) {
            std::printf("宇树官方模型未准备（assets/robots/%s/），跳过；请运行 scripts/assets/fetch_robots.sh\n", e.folder);
            return kSkipped;
        }
    }
    for (const auto& e : kRobots) inspect(e);
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
