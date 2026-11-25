// TurtleBot 4 标准版（M4-08b，robots.turtlebot4_import）：按导入配方 assets/robots/turtlebot4.terra-import.json 导入。
// 描述文件由 scripts/assets/fetch_robots.sh 准备（缺失时返回 77 表示跳过）；需要 assimp 与 python3。
//   - 导入没有错误；两次导入（并列的两个目录）逐字节相同；
//   - 49 个连杆；轮跌落关节固定，只有两个轮子是转动关节（速度执行器）；雷达、两个相机（RGB、深度）与 IMU 装在对应的连杆上；
//   - 落在地面上；推进 10 s 状态有限；
//   - 差速运动学：两轮 5 rad/s 2 s 的位移、反向 2 rad/s 的转角与 r·ω、2·r·ω/L 一致（允许打滑造成的误差）；
//   - 雷达在房间里看到墙与障碍物。
// 用法：test_turtlebot4 <临时目录>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <string>
#include <utility>

#include "terra/import/importer.hpp"
#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/scene/scene.hpp"
#include "terra/simulation/lidar.hpp"

#ifndef TERRA_ASSETS_DIR
#error "需要 TERRA_ASSETS_DIR"
#endif

namespace {

namespace fs = std::filesystem;
namespace im = terra::importer;
namespace td = terra::data;

constexpr int kSkipped = 77;
constexpr double kWheelRadius = 0.03575;
constexpr double kWheelSeparation = 0.233;
int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

std::map<std::string, std::string> snapshotDirectory(const fs::path& root) {
    std::map<std::string, std::string> out;
    for (const auto& e : fs::recursive_directory_iterator(root)) {
        if (!e.is_regular_file()) continue;
        std::ifstream in(e.path(), std::ios::binary);
        out[fs::relative(e.path(), root).string()] = std::string{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
    }
    return out;
}

double yawOf(const td::Quaternion& q) { return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)); }

double wrap(double a) {
    while (a > 3.14159265358979323846) a -= 2.0 * 3.14159265358979323846;
    while (a < -3.14159265358979323846) a += 2.0 * 3.14159265358979323846;
    return a;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "用法：test_turtlebot4 <临时目录>\n");
        return 2;
    }
    const std::string assets = TERRA_ASSETS_DIR;
    for (const char* pkg : {"turtlebot4_description", "irobot_create_description"}) {
        if (!fs::exists(assets + "/robots/" + pkg + "/package.xml")) {
            std::printf("TurtleBot 4 的描述文件未准备（assets/robots/%s），跳过；请运行 scripts/assets/fetch_robots.sh\n", pkg);
            return kSkipped;
        }
    }
    const std::string recipe = assets + "/robots/turtlebot4.terra-import.json";
    std::string scene;
    std::map<std::string, std::string> first;
    for (const char* sub : {"a", "b"}) {
        auto base = im::defaultOptions();
        base.outputRoot = (fs::path{argv[1]} / sub).string();
        base.overwrite = true;
        const auto read = im::readRecipe(recipe, base);
        for (const auto& e : read.errors) std::fprintf(stderr, "FAILED: 配方：%s\n", e.c_str());
        if (!read.recipe) return 1;
        const auto plan = im::analyzeRecipe(*read.recipe);
        const auto result = im::execute(plan, read.recipe->options, terra::physics::mujoco::composeWorld);
        for (const auto& i : result.issues) {
            if (i.severity != im::ImportIssue::Severity::Info || std::string{sub} == "a") {
                std::printf("INFO [%s] %s：%s\n", im::severityName(i.severity), i.code.c_str(), i.message.c_str());
            }
        }
        check(result.ok, std::string{"TurtleBot 4 按配方导入成功（"} + sub + "）");
        if (!result.ok) return 1;
        const auto files = snapshotDirectory(result.directory);
        if (first.empty()) {
            first = files;
            scene = result.scene;
            std::printf("INFO 导入：%zu 个连杆、%zu 个活动关节、%zu 个网格、%zu 个文件，根连杆高度 %.4f m\n", plan.links, plan.joints, plan.meshes.size(),
                        files.size(), result.settledHeight);
            check(plan.links == 49 && plan.joints == 2, "49 个连杆；轮跌落关节固定后只有两个活动关节（" + std::to_string(plan.joints) + "）");
        } else {
            std::string differ;
            for (const auto& [name, content] : files) {
                if (first.count(name) == 0 || first.at(name) != content) differ += " " + name;
            }
            check(files.size() == first.size() && differ.empty(), "两次导入逐字节相同" + differ);
        }
    }

    auto loaded = terra::scene::loadScene(scene, terra::physics::mujoco::composeWorld);
    for (const auto& i : loaded.issues) std::printf("INFO 场景：%s %s\n", i.where.c_str(), i.message.c_str());
    check(loaded.ok(), "生成的场景可以加载");
    if (!loaded.ok()) return 1;
    auto& world = *loaded.scene->world;
    const auto& d = world.description();
    std::size_t hinges = 0, slides = 0;
    for (const auto& j : d.joints) {
        hinges += j.type == td::JointType::Hinge ? 1u : 0u;
        slides += j.type == td::JointType::Slide ? 1u : 0u;
    }
    const auto* left = d.findActuator("turtlebot4/left_wheel_joint");
    const auto* right = d.findActuator("turtlebot4/right_wheel_joint");
    std::printf("INFO 场景：%zu 个实体，%zu 个转动关节、%zu 个滑动关节，%zu 个执行器\n", d.entities.size(), hinges, slides, d.actuators.size());
    check(hinges == 2 && slides == 0 && d.actuators.size() == 2 && left && right && left->kind == td::ActuatorKind::Velocity, "两个轮子速度执行器，没有轮跌落关节");
    const auto& lidars = loaded.scene->lidars;
    const auto& cameras = loaded.scene->cameras;
    const auto* lidarEntity = d.findEntity("turtlebot4/rplidar_link");
    const auto* opticalEntity = d.findEntity("turtlebot4/oakd_rgb_camera_optical_frame");
    check(lidars.size() == 1 && lidarEntity && lidars[0].entity == lidarEntity->id && lidars[0].samples == 1080 && lidars[0].rateHz == 8.0,
          "雷达：rplidar_link，1080 束、8 Hz");
    check(cameras.size() == 2 && opticalEntity && cameras[0].entity == opticalEntity->id && cameras[0].width == 250 && cameras[0].rgb && !cameras[0].depth &&
              cameras[1].width == 640 && cameras[1].height == 400 && cameras[1].depth && !cameras[1].rgb,
          "相机：OAK-D 的光学坐标系上，RGB 250 × 250 与深度 640 × 400");
    check(d.findSensor("turtlebot4/imu") != nullptr, "IMU 装在 imu_link");

    const auto* baseEntity = d.findEntity("turtlebot4/base_link");
    check(baseEntity != nullptr, "机身 base_link");
    if (baseEntity == nullptr || !left || !right) return 1;
    const std::size_t base = baseEntity->id.value - 1;
    const auto stepFor = [&](double seconds) {
        for (int k = 0; k < static_cast<int>(seconds / d.timestep); ++k) {
            if (const auto e = world.step()) {
                check(false, "推进出错：" + e->message);
                return false;
            }
        }
        return true;
    };
    stepFor(1.0);  // 落稳
    const double settled = world.snapshot()->entities[base].pose.position.z;
    std::printf("INFO 落稳后 base_link 高度 %.4f m\n", settled);
    // Create 3 的 base_link 原点在地面高度（轮子着地时约为 0）；接触有几毫米的穿透（2026-11-25 基线：−0.0046 m）
    check(settled > -0.02 && settled < 0.1, "落在地面上（base_link 高度在 −0.02～0.1 m）");

    // 直行：两轮 5 rad/s 2 s；理论位移 r·ω·t
    auto before = world.snapshot()->entities[base].pose;
    check(!world.setControl({{left->id, {5.0}}, {right->id, {5.0}}}), "设置轮速");
    stepFor(2.0);
    auto after = world.snapshot()->entities[base].pose;
    const double moved = std::hypot(after.position.x - before.position.x, after.position.y - before.position.y);
    const double expectedMove = kWheelRadius * 5.0 * 2.0;
    std::printf("INFO 两轮 5 rad/s 2 s：位移 %.3f m（理论 %.3f m）\n", moved, expectedMove);
    check(std::fabs(moved - expectedMove) < 0.25 * expectedMove, "直行位移与差速运动学一致（误差 < 25%）");
    // 原地转向：左 −2、右 +2 rad/s 1 s；理论转角 2·r·ω·t / L
    before = after;
    check(!world.setControl({{left->id, {-2.0}}, {right->id, {2.0}}}), "设置反向轮速");
    stepFor(1.0);
    after = world.snapshot()->entities[base].pose;
    const double turned = wrap(yawOf(after.orientation) - yawOf(before.orientation));
    const double expectedTurn = 2.0 * kWheelRadius * 2.0 * 1.0 / kWheelSeparation;
    std::printf("INFO 反向 2 rad/s 1 s：转角 %.3f rad（理论 %.3f rad）\n", turned, expectedTurn);
    check(std::fabs(turned - expectedTurn) < 0.3 * expectedTurn, "原地转向与差速运动学一致（误差 < 30%）");
    check(!world.setControl({{left->id, {0.0}}, {right->id, {0.0}}}), "停车");
    stepFor(7.0);
    check(world.snapshot()->diagnostics.finite, "推进 10 s 状态有限");

    // 雷达在房间里看到墙与障碍物
    const auto scan = terra::simulation::scanLidar(world, lidars[0]);
    std::size_t hits = 0;
    for (const float r : scan.ranges) hits += std::isfinite(r) ? 1u : 0u;
    std::printf("INFO 雷达：%zu 束中命中 %zu 束\n", scan.ranges.size(), hits);
    check(scan.ranges.size() == 1080 && hits > 540, "雷达在房间里看到墙与障碍物（多于一半的束命中）");

    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
