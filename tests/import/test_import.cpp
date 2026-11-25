// 模型导入（M4-08a）。
//   analyze（不需要 MuJoCo）：demo.urdf 的连杆、关节、网格、包目录、提议的执行器与传感器；xacro 版本（含参数）；
//            找不到包的 xacro；故意出错的 URDF（缺网格、未知的包、毫米单位、没有惯量、mimic）；MJCF；名字清理
//   execute（MuJoCo；demo.urdf 中的 Collada 网格需要 assimp）：写出导入目录；Collada 按材质拆分、单位换算为米、
//            保持文件中的坐标轴（与 RViz 相同）；生成的场景能加载，落在地面上；两轮驱动前进与转向；位置执行器跟随；
//            已存在时不覆盖；有错误的计划不导入
//   demos（MuJoCo，可选）：ROBOTS/import_demos 中的 urdf_tutorial（R2D2）与 TurtleBot 3 Burger；没有下载时返回 77
// 用法：test_import analyze | execute <临时目录> | demos <临时目录> <ROBOTS 目录>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>

#include "terra/import/importer.hpp"

#ifdef TERRA_IMPORT_EXECUTE
#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/scene/scene.hpp"
#endif

#ifndef TERRA_TEST_ASSETS_DIR
#error "需要 TERRA_TEST_ASSETS_DIR"
#endif

namespace {

namespace fs = std::filesystem;
namespace im = terra::importer;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

const std::string kDemo = std::string{TERRA_TEST_ASSETS_DIR} + "/import/demo_robot/urdf/demo.urdf";

bool hasIssue(const std::vector<im::ImportIssue>& issues, const std::string& code, im::ImportIssue::Severity severity) {
    for (const auto& i : issues) {
        if (i.code == code && i.severity == severity) return true;
    }
    return false;
}

std::string listIssues(const std::vector<im::ImportIssue>& issues) {
    std::string out;
    for (const auto& i : issues) out += std::string{"\n  ["} + im::severityName(i.severity) + "] " + i.code + "：" + i.message + (i.hint.empty() ? "" : " → " + i.hint);
    return out;
}

const im::ProposedActuator* actuator(const im::ImportPlan& p, const std::string& joint) {
    for (const auto& a : p.actuators) {
        if (a.joint == joint) return &a;
    }
    return nullptr;
}

const im::ProposedSensor* sensor(const im::ImportPlan& p, const std::string& type) {
    for (const auto& s : p.sensors) {
        if (s.type == type) return &s;
    }
    return nullptr;
}

bool assimpAvailable() {
    const char* path = std::getenv("PATH");
    std::stringstream ss(path ? path : "");
    std::string dir;
    while (std::getline(ss, dir, ':')) {
        if (!dir.empty() && fs::exists(fs::path{dir} / "assimp")) return true;
    }
    return false;
}

void testAnalyze() {
    const bool assimp = assimpAvailable();
    auto options = im::defaultOptions();
    options.input = kDemo;
    const auto plan = im::analyze(options);
    std::printf("INFO demo.urdf：%zu 个连杆、%zu 个活动关节、%zu 个网格、%zu 个执行器、%zu 个传感器%s\n", plan.links, plan.joints, plan.meshes.size(),
                plan.actuators.size(), plan.sensors.size(), listIssues(plan.issues).c_str());
    check(plan.kind == im::SourceKind::Urdf && plan.robotName == "demo_robot" && plan.name == "demo_robot", "demo.urdf：URDF，名字 demo_robot");
    check(plan.links == 10 && plan.joints == 3, "demo.urdf：10 个连杆、3 个活动关节");
    check(plan.packages.count("demo_robot") == 1, "自动找到包 demo_robot（模型所在的包）");
    check(plan.meshes.size() == 2 && plan.meshes[0].format == "stl" && !plan.meshes[0].convert && plan.meshes[1].format == "dae" &&
              plan.meshes[1].convert && fs::exists(plan.meshes[0].resolved),
          "网格：body.stl 直接用，marker.dae 需要转换；package:// 已解析");
    check(assimp ? hasIssue(plan.issues, "mesh_conversion", im::ImportIssue::Severity::Info)
                 : hasIssue(plan.issues, "assimp_missing", im::ImportIssue::Severity::Error),
          assimp ? "有 assimp：提示将转换 1 个网格" : "没有 assimp：报错并给出安装命令");
    check(!plan.fixedBase, "根连杆不是 world：浮动基座");
    const auto* left = actuator(plan, "left_wheel_joint");
    const auto* arm = actuator(plan, "arm_joint");
    check(plan.actuators.size() == 3 && left && left->type == "velocity" && left->forceLimit == 5.0 && actuator(plan, "right_wheel_joint"),
          "轮子：速度执行器，力限取 effort 5");
    check(arm && arm->type == "position" && arm->limited && arm->rangeMin == -1.57 && arm->rangeMax == 1.57 && arm->forceLimit == 5.0,
          "arm_joint：位置执行器，范围取关节限位");
    check(hasIssue(plan.issues, "default_actuators", im::ImportIssue::Severity::Info), "说明“URDF 没有执行器，已提议”");
    const auto* lidar = sensor(plan, "lidar2d");
    const auto* camera = sensor(plan, "camera");
    const auto* imu = sensor(plan, "imu");
    check(lidar && lidar->link == "laser_link", "提议激光雷达装在 laser_link");
    check(camera && camera->link == "camera_optical_frame" && camera->optical, "提议相机装在光学坐标系 camera_optical_frame（优先于 camera_link）");
    check(imu && imu->link == "imu_link", "提议 IMU 装在 imu_link");

    // xacro（宏、属性、表达式、参数、$(find)），不需要 ROS
    auto xo = im::defaultOptions();
    xo.input = std::string{TERRA_TEST_ASSETS_DIR} + "/import/demo_robot/urdf/demo.urdf.xacro";
    xo.xacroArgs = {"wheel_effort:=7"};
    const auto xp = im::analyze(xo);
    std::printf("INFO demo.urdf.xacro：%zu 个连杆%s\n", xp.links, listIssues(xp.issues).c_str());
    check(xp.kind == im::SourceKind::Xacro && hasIssue(xp.issues, "xacro_expanded", im::ImportIssue::Severity::Info) && xp.ok(),
          "xacro：用随 Terra 提供的 xacro 展开，没有错误");
    check(xp.links == 6 && xp.joints == 2 && xp.actuators.size() == 2, "xacro：6 个连杆、两个轮子关节（宏展开）");
    const auto* xl = actuator(xp, "left_wheel_joint");
    check(xl && xl->forceLimit == 7.0, "xacro 参数 wheel_effort:=7 生效");
    auto mo = im::defaultOptions();
    mo.input = std::string{TERRA_TEST_ASSETS_DIR} + "/import/missing_pkg.urdf.xacro";
    const auto mp = im::analyze(mo);
    check(!mp.ok() && hasIssue(mp.issues, "unknown_package", im::ImportIssue::Severity::Error), "xacro 中找不到包：报 unknown_package 并提示 --package" + listIssues(mp.issues));

    // 故意出错的 URDF
    auto bo = im::defaultOptions();
    bo.input = std::string{TERRA_TEST_ASSETS_DIR} + "/import/broken.urdf";
    const auto bp = im::analyze(bo);
    std::printf("INFO broken.urdf：%s\n", listIssues(bp.issues).c_str());
    check(!bp.ok(), "broken.urdf：有错误");
    for (const auto& [code, severity] : {std::pair{"missing_mesh", im::ImportIssue::Severity::Error},
                                         std::pair{"unknown_package", im::ImportIssue::Severity::Error},
                                         std::pair{"suspicious_scale", im::ImportIssue::Severity::Warning},
                                         std::pair{"no_inertia", im::ImportIssue::Severity::Warning},
                                         std::pair{"mimic_joint", im::ImportIssue::Severity::Warning}}) {
        check(hasIssue(bp.issues, code, severity), std::string{"broken.urdf：报 "} + code);
    }
    bool hints = true;
    for (const auto& i : bp.issues) hints = hints && (i.severity == im::ImportIssue::Severity::Info || !i.hint.empty());
    check(hints, "每个错误与警告都有处理办法");
    // 网格缩放 0.001：不再提示单位
    bo.meshScale = 0.001;
    check(!hasIssue(im::analyze(bo).issues, "suspicious_scale", im::ImportIssue::Severity::Warning), "设置网格缩放 0.001 后不再提示毫米单位");

    // MJCF 与其他
    auto jo = im::defaultOptions();
    jo.input = std::string{TERRA_TEST_ASSETS_DIR} + "/scene/cart.xml";
    const auto jp = im::analyze(jo);
    check(jp.kind == im::SourceKind::Mjcf && jp.ok() && hasIssue(jp.issues, "mjcf_as_is", im::ImportIssue::Severity::Info), "MJCF：直接使用");
    auto no = im::defaultOptions();
    no.input = std::string{TERRA_TEST_ASSETS_DIR} + "/import/no_such.urdf";
    check(hasIssue(im::analyze(no).issues, "file_not_found", im::ImportIssue::Severity::Error), "文件不存在");
    check(im::sanitizeName("my robot/v2") == "my_robot_v2" && im::sanitizeName("") == "robot" && im::sanitizeName("机器人") == "_________" &&
              im::sanitizeName("07-physics") == "robot_07-physics",
          "名字清理（实例名以字母或下划线开头）");
}

#ifdef TERRA_IMPORT_EXECUTE
// OBJ 的顶点包围盒（测试自己读，不经过被测代码）
bool objBounds(const fs::path& path, double lo[3], double hi[3]) {
    std::ifstream in(path);
    std::string line;
    bool any = false;
    while (std::getline(in, line)) {
        if (line.rfind("v ", 0) != 0) continue;
        std::istringstream s(line.substr(2));
        double p[3];
        s >> p[0] >> p[1] >> p[2];
        for (int k = 0; k < 3; ++k) {
            lo[k] = any ? std::min(lo[k], p[k]) : p[k];
            hi[k] = any ? std::max(hi[k], p[k]) : p[k];
        }
        any = true;
    }
    return any;
}

double yawOf(const terra::data::Quaternion& q) { return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)); }

struct Robot {
    terra::scene::SceneLoadResult loaded;
    terra::data::ActuatorId left, right;
    std::size_t base = 0;  // 根连杆的实体下标
};

// 两轮驱动 seconds 秒；返回根连杆的 (Δx, Δyaw)
std::pair<double, double> drive(terra::physics::PhysicsWorld& world, std::size_t base, terra::data::ActuatorId left, terra::data::ActuatorId right,
                                double wl, double wr, double seconds) {
    const auto before = world.snapshot()->entities[base].pose;
    if (world.setControl({{left, {wl}}, {right, {wr}}})) return {0.0, 0.0};
    for (int k = 0; k < static_cast<int>(seconds / world.description().timestep); ++k) {
        if (world.step()) return {0.0, 0.0};
    }
    const auto after = world.snapshot()->entities[base].pose;
    return {after.position.x - before.position.x, yawOf(after.orientation) - yawOf(before.orientation)};
}

void testExecute(const std::string& dir) {
    auto options = im::defaultOptions();
    options.input = kDemo;
    options.outputRoot = dir;
    options.overwrite = true;
    const auto plan = im::analyze(options);
    const auto result = im::execute(plan, options, terra::physics::mujoco::composeWorld);
    std::printf("INFO 导入 demo.urdf：%s，根连杆高度 %.4f m%s\n", result.ok ? "成功" : "失败", result.settledHeight, listIssues(result.issues).c_str());
    check(result.ok, "demo.urdf 一步导入成功");
    if (!result.ok) return;
    const fs::path out{result.directory};
    for (const char* f : {"demo_robot.xml", "demo_robot.terra-scene.json", "floor.xml", "import-report.json", "meshes/body.stl", "meshes/marker.obj",
                          "meshes/marker_visual0.obj", "meshes/marker_visual1.obj", "textures/checker.png"}) {
        check(fs::exists(out / f), std::string{"导入目录中有 "} + f);
    }
    // Collada：单位厘米 → 米，保持文件中的坐标轴（Z_UP 不旋转，与 RViz 相同）；顶面（红）单独一个材质
    double lo[3], hi[3];
    if (objBounds(out / "meshes/marker.obj", lo, hi)) {
        std::printf("INFO marker.obj：x %.4f～%.4f，y %.4f～%.4f，z %.4f～%.4f\n", lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]);
        const bool ok = std::fabs(lo[0]) < 1e-6 && std::fabs(hi[0] - 0.1) < 1e-6 && std::fabs(lo[1]) < 1e-6 && std::fabs(hi[1] - 0.04) < 1e-6 &&
                        std::fabs(lo[2]) < 1e-6 && std::fabs(hi[2] - 0.02) < 1e-6;
        check(ok, "Collada 网格：10 × 4 × 2 cm 换算为 0.1 × 0.04 × 0.02 m，坐标轴与文件相同");
    } else {
        check(false, "读取 marker.obj");
    }
    int topParts = 0;
    for (const char* f : {"meshes/marker_visual0.obj", "meshes/marker_visual1.obj"}) {
        if (objBounds(out / f, lo, hi) && std::fabs(lo[2] - 0.02) < 1e-6 && std::fabs(hi[2] - 0.02) < 1e-6) ++topParts;
    }
    check(topParts == 1, "按材质拆分：顶面（红色材质）单独一份网格");

    auto loaded = terra::scene::loadScene(result.scene, terra::physics::mujoco::composeWorld);
    check(loaded.ok(), "生成的场景可以加载");
    if (!loaded.ok()) return;
    auto& world = *loaded.scene->world;
    const auto& d = world.description();
    const auto* left = d.findActuator("demo_robot/left_wheel_joint");
    const auto* right = d.findActuator("demo_robot/right_wheel_joint");
    const auto* arm = d.findActuator("demo_robot/arm_joint");
    check(d.actuators.size() == 3 && left && right && arm && left->kind == terra::data::ActuatorKind::Velocity &&
              arm->kind == terra::data::ActuatorKind::Position && arm->inputs.at(0).limited && arm->inputs[0].max == 1.57,
          "执行器：两个速度、一个位置（范围 ±1.57）");
    check(loaded.scene->lidars.size() == 1 && loaded.scene->lidars[0].name == "demo_robot/scan" && loaded.scene->cameras.size() == 1 &&
              d.findSensor("demo_robot/imu") != nullptr,
          "场景中的传感器：雷达、相机与 IMU");
    check(std::fabs(result.settledHeight) < 0.02, "导入时的检查：推进 1 s 后 base_footprint 在地面上（" + std::to_string(result.settledHeight) + " m）");
    const auto* base = d.findEntity("demo_robot/base_footprint");
    check(base != nullptr, "根连杆实体 demo_robot/base_footprint");
    if (base == nullptr || !left || !right || !arm) return;
    const std::size_t b = base->id.value - 1;
    for (int k = 0; k < 250; ++k) world.step();  // 先落稳
    const auto forward = drive(world, b, left->id, right->id, 8.0, 8.0, 1.0);
    const auto turn = drive(world, b, left->id, right->id, -4.0, 4.0, 1.0);  // 约 1.7 rad/s，1 s 内不超过 π
    std::printf("INFO 两轮 8 rad/s 前进 1 s：Δx = %.3f m；反向 4 rad/s 转 1 s：Δyaw = %.3f rad\n", forward.first, turn.second);
    check(forward.first > 0.15, "两个轮子同向转：小车前进");
    check(turn.second > 0.3, "两个轮子反向转：小车原地左转");
    const auto* armJoint = d.findJoint("demo_robot/arm_joint");
    check(!world.setControl({{arm->id, {1.0}}}), "设置手臂目标");
    for (int k = 0; k < 500; ++k) world.step();
    const double q = world.snapshot()->joints[static_cast<std::size_t>(armJoint - d.joints.data())].position[0];
    check(std::fabs(q - 1.0) < 0.1, "位置执行器：手臂转到 1.0 rad（实际 " + std::to_string(q) + "）");

    // 已存在时不覆盖；有错误的计划不导入
    options.overwrite = false;
    const auto again = im::execute(plan, options, terra::physics::mujoco::composeWorld);
    check(!again.ok && hasIssue(again.issues, "output_exists", im::ImportIssue::Severity::Error), "导入目录已存在：不覆盖并提示");
    auto bo = im::defaultOptions();
    bo.input = std::string{TERRA_TEST_ASSETS_DIR} + "/import/broken.urdf";
    bo.outputRoot = dir;
    const auto bad = im::execute(im::analyze(bo), bo, terra::physics::mujoco::composeWorld);
    check(!bad.ok && hasIssue(bad.issues, "plan_has_errors", im::ImportIssue::Severity::Error) && !fs::exists(fs::path{dir} / "broken"),
          "有错误的计划不导入");
}

// 公开示例（可选）：ROBOTS/import_demos/ 中按 M4-08 设计稿第 8.1 节下载的 urdf_tutorial 与 TurtleBot 3
int testDemos(const std::string& dir, const std::string& robots) {
    const fs::path demos = fs::path{robots} / "import_demos";
    struct Demo {
        const char* path;
        const char* name;
        const char* what;
    };
    const Demo list[] = {{"urdf_tutorial/urdf/07-physics.urdf", "r2d2_physics", "R2D2（Collada 带贴图、package://）"},
                         {"urdf_tutorial/urdf/08-macroed.urdf.xacro", "r2d2_macroed", "R2D2（xacro 宏）"},
                         {"turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf", "turtlebot3_burger", "TurtleBot 3 Burger（xacro 参数、毫米 STL）"}};
    int ran = 0;
    for (const auto& demo : list) {
        const fs::path input = demos / demo.path;
        if (!fs::exists(input)) {
            std::printf("INFO 没有下载：%s（%s）\n", input.string().c_str(), demo.what);
            continue;
        }
        ++ran;
        auto options = im::defaultOptions();
        options.input = input.string();
        options.outputRoot = dir;
        options.overwrite = true;
        options.name = demo.name;
        const auto plan = im::analyze(options);
        const auto result = im::execute(plan, options, terra::physics::mujoco::composeWorld);
        std::size_t converted = 0;
        for (const auto& m : plan.meshes) converted += m.convert ? 1u : 0u;
        std::printf("INFO %s：%s；%zu 个连杆、%zu 个活动关节、%zu 个网格（转换 %zu）、%zu 个执行器、%zu 个传感器，根连杆高度 %.3f m%s\n", demo.what,
                    result.ok ? "导入成功" : "导入失败", plan.links, plan.joints, plan.meshes.size(), converted, plan.actuators.size(),
                    plan.sensors.size(), result.settledHeight, listIssues(result.issues).c_str());
        check(result.ok, std::string{demo.what} + "：一步导入成功");
        if (std::string{demo.path}.find("turtlebot3") != std::string::npos && result.ok) {
            check(plan.actuators.size() == 2 && plan.actuators[0].type == "velocity" && sensor(plan, "lidar2d") && sensor(plan, "lidar2d")->link == "base_scan" &&
                      sensor(plan, "imu"),
                  "TurtleBot 3：两个轮子速度执行器，提议雷达（base_scan）与 IMU");
            auto loaded = terra::scene::loadScene(result.scene, terra::physics::mujoco::composeWorld);
            if (loaded.ok()) {
                auto& world = *loaded.scene->world;
                const auto& d = world.description();
                const auto* l = d.findActuator(options.name + "/wheel_left_joint");
                const auto* r = d.findActuator(options.name + "/wheel_right_joint");
                const auto* base = d.findEntity(options.name + "/base_footprint");
                if (l && r && base) {
                    for (int k = 0; k < 250; ++k) world.step();
                    const auto moved = drive(world, base->id.value - 1, l->id, r->id, 6.0, 6.0, 1.0);
                    std::printf("INFO TurtleBot 3 两轮 6 rad/s 1 s：Δx = %.3f m\n", moved.first);
                    check(std::fabs(moved.first) > 0.05, "TurtleBot 3：两轮驱动能开动");
                } else {
                    check(false, "TurtleBot 3：找到轮子执行器与 base_footprint");
                }
            }
        }
    }
    return ran == 0 ? 77 : 0;
}
#endif

}  // namespace

int main(int argc, char** argv) {
    const std::string mode = argc > 1 ? argv[1] : "";
    if (mode == "analyze") {
        testAnalyze();
#ifdef TERRA_IMPORT_EXECUTE
    } else if (mode == "execute" && argc > 2) {
        testExecute(argv[2]);
    } else if (mode == "demos" && argc > 3) {
        if (testDemos(argv[2], argv[3]) == 77 && failures == 0) {
            std::printf("没有下载公开示例（ROBOTS/import_demos/），跳过\n");
            return 77;
        }
#endif
    } else {
        std::fprintf(stderr, "用法：test_import analyze | execute <临时目录> | demos <临时目录> <ROBOTS 目录>\n");
        return 2;
    }
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
