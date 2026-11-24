// 预定义动作（M3-04c）：assets/motions/ 中的每个动作文件
//   - 解析与绑定通过（目标都在范围内；PD 层接管的执行器按关节限位校验）；
//   - 从起始关键帧开始，经“动作 → PD 层 → setControl → step”推进到动作结束后 1 秒：没有仿真错误、控制没有被拒绝、状态有限；
//   - 打印每个动作结束时的关键数值（供 M3-06 设定判据参考），并检查少数确定的结果：
//     G1 悬挂动作中骨盆不动；Go2 动作结束回到 home 后机身高于 0.2 m；Stretch 底盘移动了至少 0.1 m。
// 参考机器人未准备时以返回码 77 表示跳过。本文件不包含 MuJoCo 头文件。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/scene/scene.hpp"
#include "terra/simulation/motion.hpp"
#include "terra/simulation/pd_layer.hpp"

#ifndef TERRA_ASSETS_DIR
#error "需要 TERRA_ASSETS_DIR"
#endif

namespace {

namespace fs = std::filesystem;
namespace sc = terra::scene;
namespace sim = terra::simulation;
namespace td = terra::data;

constexpr int kSkipped = 77;
int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

struct Expectation {
    const char* file;    // assets/motions/ 下的相对路径
    const char* root;    // 观察的实体
};

constexpr Expectation kMotions[] = {
    {"g1/wave.terra-motion.json", "g1/pelvis"},
    {"g1/squat.terra-motion.json", "g1/pelvis"},
    {"g1/stand_hold.terra-motion.json", "g1/pelvis"},
    {"go2/lie_stand_sit.terra-motion.json", "go2/base"},
    {"stretch3/tour.terra-motion.json", "stretch3/base_link"},
};

void run(const Expectation& e) {
    const std::string path = std::string{TERRA_ASSETS_DIR} + "/motions/" + e.file;
    const auto parsed = sim::readMotionFile(path);
    for (const auto& i : parsed.issues) std::printf("INFO [%s] %s：%s\n", e.file, i.where.c_str(), i.message.c_str());
    check(parsed.document.has_value(), std::string{e.file} + "：解析通过");
    if (!parsed.document) return;
    auto loaded = sc::loadScene(parsed.document->scene, terra::physics::mujoco::composeWorld);
    check(loaded.ok(), std::string{e.file} + "：推荐场景可以打开：" + parsed.document->scene);
    if (!loaded.ok()) return;
    sc::SceneInstance& s = *loaded.scene;
    terra::physics::PhysicsWorld& world = *s.world;
    const auto& d = world.description();
    sim::PdLayer pd(d, s.pd);
    const auto bound = sim::bindMotion(*parsed.document, d, pd.targetRangeFunction());
    for (const auto& i : bound.issues) std::printf("INFO [%s] %s：%s\n", e.file, i.where.c_str(), i.message.c_str());
    check(bound.motion.has_value(), std::string{e.file} + "：绑定通过（执行器存在、目标在范围内）");
    if (!bound.motion) return;
    if (const auto key = bound.motion->startKeyframe) world.resetToKeyframe(*key);
    sim::MotionPlayer player(*bound.motion);
    player.start(world.stepIndex());
    const double dt = d.timestep;
    const auto steps = static_cast<std::uint64_t>(std::ceil((bound.motion->duration + 1.0) / dt));
    const auto* root = d.findEntity(e.root);
    check(root != nullptr, std::string{e.file} + "：找到实体 " + e.root);
    const td::Vec3 start = root ? world.snapshot()->entities[root->id.value - 1].pose.position : td::Vec3{};
    double minZ = start.z;
    bool ok = true;
    for (std::uint64_t k = 0; k < steps && ok; ++k) {
        auto commands = player.commands(world.stepIndex(), dt);
        if (!pd.empty()) commands = pd.process(std::move(commands), *world.snapshot());
        if (const auto error = world.setControl(commands)) {
            check(false, std::string{e.file} + "：第 " + std::to_string(k) + " 步控制被拒绝：" + error->message);
            ok = false;
        } else if (const auto stepError = world.step()) {
            check(false, std::string{e.file} + "：第 " + std::to_string(k) + " 步出错：" + stepError->message);
            ok = false;
        }
        if (root) minZ = std::min(minZ, world.snapshot()->entities[root->id.value - 1].pose.position.z);
    }
    if (!ok) return;
    check(world.snapshot()->diagnostics.finite, std::string{e.file} + "：状态有限");
    if (!root) return;
    const td::Vec3 end = world.snapshot()->entities[root->id.value - 1].pose.position;
    const double moved = std::hypot(end.x - start.x, end.y - start.y);
    std::printf("INFO [%s] %s，%llu 步（%.2f s）：%s 从 (%.3f, %.3f, %.3f) 到 (%.3f, %.3f, %.3f)，水平移动 %.3f m，最低 %.3f m\n", e.file,
                bound.motion->name.c_str(), static_cast<unsigned long long>(steps), static_cast<double>(steps) * dt, e.root, start.x,
                start.y, start.z, end.x, end.y, end.z, moved, minZ);
    const std::string file = e.file;
    if (file.rfind("g1/wave", 0) == 0 || file.rfind("g1/squat", 0) == 0) {
        check(std::fabs(end.z - start.z) < 1e-9 && moved < 1e-9, file + "：悬挂模式下骨盆不动");
    } else if (file.rfind("go2/", 0) == 0) {
        check(end.z > 0.2, file + "：动作结束回到 home 后机身高于 0.2 m（实际 " + std::to_string(end.z) + "）");
    } else if (file.rfind("stretch3/", 0) == 0) {
        check(moved > 0.1, file + "：底盘移动了至少 0.1 m（实际 " + std::to_string(moved) + "）");
    }
}

}  // namespace

int main() {
    for (const char* folder : {"unitree_g1", "unitree_go2", "hello_robot_stretch_3"}) {
        if (!fs::exists(std::string{TERRA_ASSETS_DIR} + "/robots/" + folder + "/scene.xml")) {
            std::printf("INFO 参考机器人未准备（运行 scripts/assets/fetch_robots.sh），跳过\n");
            return kSkipped;
        }
    }
    for (const Expectation& e : kMotions) run(e);
    // 动作目录：每个机器人场景都能列出它的预定义动作（查看器菜单“动作 → 本场景的预定义动作”）
    const std::string motions = std::string{TERRA_ASSETS_DIR} + "/motions";
    for (const auto& [scene, count] : std::vector<std::pair<const char*, std::size_t>>{
             {"g1_suspended", 2}, {"g1", 1}, {"go2", 1}, {"stretch3", 1}}) {
        const auto found = sim::findMotionsForScene({motions}, std::string{TERRA_ASSETS_DIR} + "/scenes/robots/" + scene + ".terra-scene.json");
        std::string names;
        for (const auto& m : found) names += (names.empty() ? "" : "、") + m.name;
        std::printf("INFO 动作目录 [%s]：%s\n", scene, names.c_str());
        check(found.size() == count, std::string{"动作目录："} + scene + " 场景有 " + std::to_string(count) + " 个预定义动作，实际 " +
                                         std::to_string(found.size()));
    }
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
