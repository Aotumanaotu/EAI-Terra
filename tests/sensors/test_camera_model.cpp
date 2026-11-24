// 相机模型与相机传感器的配置（M4-03）：纯数学与场景文件，不需要图形上下文。
//   - 内参：160 × 120、60° → fy = fx = 60 / tan30° = 103.923…，主点 (79.5, 59.5)；光学坐标系为绕 X 轴 180°；
//   - 投影：光轴上的点落在主点；夹具中方块正面中心的投影；相机后方的点为空；
//   - MJCF 相机（描述修订 7）：camera_bench.xml 的 eye（装在 rig 上、60°、固定、透视）与 tracker（跟踪模式）；
//   - 场景文件的相机：MJCF 相机与装在实体上的相机都得到正确的安装位姿与参数（两种写法的相机看向同一方向）；
//     默认值（640 × 480、MJCF 的视场角、30 Hz）；高于物理频率时按物理频率并警告；
//   - 解析错误：camera 与 entity 二选一、装在实体上缺 name、MJCF 相机带位姿、近远平面、视场角、像素数、rgb 与 depth 都关；
//   - 组合错误：找不到相机（列出场景中的相机）、跟踪模式的相机、找不到实体、名字前缀不符；雷达尚不支持。

#include <cmath>
#include <cstdio>
#include <string>

#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/scene/camera_model.hpp"
#include "terra/scene/pose_math.hpp"
#include "terra/scene/scene.hpp"

#ifndef TERRA_TEST_ASSETS_DIR
#error "需要 TERRA_TEST_ASSETS_DIR"
#endif

namespace {

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

bool hasIssue(const std::vector<sc::SceneIssue>& issues, Severity severity, const std::string& where, const std::string& words) {
    for (const auto& i : issues) {
        if (i.severity == severity && (where.empty() || i.where == where) && i.message.find(words) != std::string::npos) return true;
    }
    return false;
}

const std::string kDir = std::string{TERRA_TEST_ASSETS_DIR} + "/sensors";

sc::SceneLoadResult load(const std::string& sensors, std::vector<sc::SceneIssue>* parseIssues = nullptr) {
    const std::string text =
        R"({"format": "terra.scene", "version": 1, "models": [{"name": "bench", "mjcf": "camera_bench.xml"}], "sensors": )" + sensors + "}";
    auto parsed = sc::parseScene(text, kDir + "/inline.terra-scene.json");
    if (parseIssues != nullptr) *parseIssues = parsed.issues;
    if (!parsed.document) {
        sc::SceneLoadResult out;
        out.issues = parsed.issues;
        return out;
    }
    return sc::instantiate(std::move(*parsed.document), terra::physics::mujoco::composeWorld);
}

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

void math() {
    const auto k = sc::cameraIntrinsics(160, 120, 60.0);
    const double fy = 60.0 / std::tan(3.14159265358979323846 / 6.0);
    check(k.width == 160 && k.height == 120 && near(k.fy, fy, 1e-12) && k.fx == k.fy && k.cx == 79.5 && k.cy == 59.5,
          "内参：fy = fx = 60 / tan30°，主点 (79.5, 59.5)");
    const auto q = sc::opticalRotation();
    check(q.w == 0.0 && q.x == 1.0 && q.y == 0.0 && q.z == 0.0, "光学坐标系：绕 X 轴 180°");
    // 相机在 (0, 0, 1)，看向 +X（与夹具中的 eye 相同）
    td::Pose cam;
    cam.position = {0, 0, 1};
    cam.orientation = sc::fromRpyDegrees(90, 0, -90);
    const auto onAxis = sc::projectPoint(k, cam, {3.0, 0.0, 1.0});
    check(onAxis && near((*onAxis)[0], 79.5, 1e-9) && near((*onAxis)[1], 59.5, 1e-9), "光轴上的点落在主点");
    const auto face = sc::projectPoint(k, cam, {0.95, 0.3, 1.2});
    check(face && near((*face)[0], 79.5 - fy * 0.3 / 0.95, 1e-9) && near((*face)[1], 59.5 - fy * 0.2 / 0.95, 1e-9),
          "方块正面中心：u = cx − fy·0.3/0.95（世界 +Y 在图像左侧），v = cy − fy·0.2/0.95（世界上方在图像上方）");
    check(!sc::projectPoint(k, cam, {-1.0, 0.0, 1.0}), "相机后方的点：空");
}

void mjcfCameras() {
    auto loaded = terra::physics::mujoco::loadWorld({kDir + "/camera_bench.xml"});
    check(loaded.world != nullptr, "加载 camera_bench.xml");
    if (!loaded.world) return;
    const auto& d = loaded.world->description();
    check(td::ModelDescription::kRevision == 7, "模型描述修订号 7");
    check(d.cameras.size() == 2, "两个 MJCF 相机");
    const auto* eye = d.findCamera("eye");
    const auto* rig = d.findEntity("rig");
    check(eye != nullptr && rig != nullptr && eye->entity == rig->id && eye->fovyDegrees == 60.0 && eye->fixed && eye->perspective &&
              eye->width == 0 && eye->height == 0,
          "eye：装在 rig 上，60°，固定，透视，没写分辨率");
    const auto* tracker = d.findCamera("tracker");
    check(tracker != nullptr && !tracker->fixed && !tracker->entity.valid(), "tracker：装在世界上，跟踪模式");
}

void sceneCameras() {
    auto loaded = sc::loadScene(kDir + "/camera_bench.terra-scene.json", terra::physics::mujoco::composeWorld);
    check(loaded.ok(), "加载 camera_bench.terra-scene.json" + dump(loaded.issues));
    if (!loaded.ok()) return;
    const auto& scene = *loaded.scene;
    check(scene.cameras.size() == 2, "两个相机传感器");
    if (scene.cameras.size() != 2) return;
    const auto& eye = scene.cameras[0];
    const auto& side = scene.cameras[1];
    check(eye.name == "bench/eye" && eye.width == 160 && eye.height == 120 && eye.fovyDegrees == 60.0 && eye.rateHz == 30.0 &&
              eye.rgb && eye.depth && eye.nearClip == 0.05 && eye.farClip == 50.0,
          "MJCF 相机 bench/eye：名字取相机名，160 × 120，视场角取 MJCF 的 60°，30 Hz，彩色与深度");
    check(side.name == "bench/side" && !side.rgb && side.depth, "装在实体上的 bench/side：只输出深度");
    const auto snap = scene.world->snapshot();
    const td::Pose a = sc::cameraWorldPose(*snap, eye);
    const td::Pose b = sc::cameraWorldPose(*snap, side);
    const td::Vec3 fa = sc::rotate(a.orientation, {0, 0, -1});
    const td::Vec3 fb = sc::rotate(b.orientation, {0, 0, -1});
    const td::Vec3 ua = sc::rotate(a.orientation, {0, 1, 0});
    check(near(a.position.z, 1.0, 1e-12) && near(fa.x, 1.0, 1e-12) && near(ua.z, 1.0, 1e-12), "eye 在 (0, 0, 1)，看向世界 +X，上方为 +Z");
    check(near(fb.x, 1.0, 1e-12) && near(b.position.z, 1.0, 1e-12), "side（rpyDeg 90, 0, −90）与 eye 看向同一方向");

    // 默认值与频率上限
    auto defaults = load(R"([{"type": "camera", "camera": "bench/eye"}, {"type": "camera", "name": "bench/fast", "entity": "bench/rig", "rateHz": 1000}])");
    check(defaults.ok(), "默认值场景可以加载" + dump(defaults.issues));
    if (defaults.ok() && defaults.scene->cameras.size() == 2) {
        const auto& c = defaults.scene->cameras[0];
        check(c.width == 640 && c.height == 480 && c.fovyDegrees == 60.0 && c.rateHz == 30.0, "默认 640 × 480、30 Hz、MJCF 的视场角");
        check(defaults.scene->cameras[1].fovyDegrees == 60.0 && std::fabs(defaults.scene->cameras[1].rateHz - 500.0) < 1e-9 &&
                  hasIssue(defaults.issues, Severity::Warning, "/sensors/1/rateHz", "高于物理频率"),
              "装在实体上：视场角默认 60°；1000 Hz 按物理频率 500 Hz 并警告");
    }
}

void errors() {
    struct Case {
        const char* sensors;
        const char* where;
        const char* words;
    };
    const Case parseCases[] = {
        {R"([{"type": "camera", "camera": "bench/eye", "entity": "bench/rig"}])", "/sensors/0", "其中之一"},
        {R"([{"type": "camera"}])", "/sensors/0", "其中之一"},
        {R"([{"type": "camera", "entity": "bench/rig"}])", "/sensors/0", "需要 name"},
        {R"([{"type": "camera", "camera": "bench/eye", "position": [0, 0, 1]}])", "/sensors/0", "不能再写 position"},
        {R"([{"type": "camera", "camera": "bench/eye", "near": 0}])", "/sensors/0/near", "大于 0"},
        {R"([{"type": "camera", "camera": "bench/eye", "near": 2, "far": 1}])", "/sensors/0/far", "大于近平面"},
        {R"([{"type": "camera", "camera": "bench/eye", "fovyDeg": 180}])", "/sensors/0/fovyDeg", "0° 到 180°"},
        {R"([{"type": "camera", "camera": "bench/eye", "width": 0}])", "/sensors/0/width", "1 到 8192"},
        {R"([{"type": "camera", "camera": "bench/eye", "height": 1.5}])", "/sensors/0/height", "整数"},
        {R"([{"type": "camera", "camera": "bench/eye", "rgb": false, "depth": false}])", "/sensors/0", "不能都关闭"},
        {R"([{"type": "camera", "camera": "bench/eye", "site": "x"}])", "/sensors/0/site", "未知字段"},
        {R"([{"type": "lidar2d", "entity": "bench/rig"}])", "/sensors/0/type", "M4-03b"},
    };
    for (const Case& c : parseCases) {
        std::vector<sc::SceneIssue> issues;
        const auto loaded = load(c.sensors, &issues);
        check(!loaded.ok() && hasIssue(issues, Severity::Error, c.where, c.words),
              std::string{"解析错误 "} + c.sensors + " → " + c.where + "“" + c.words + "”" + dump(issues));
    }
    const Case composeCases[] = {
        {R"([{"type": "camera", "camera": "bench/nocam"}])", "/sensors/0/camera", "找不到相机"},
        {R"([{"type": "camera", "camera": "bench/tracker"}])", "/sensors/0/camera", "跟踪模式"},
        {R"([{"type": "camera", "name": "bench/c", "entity": "bench/nobody"}])", "/sensors/0/entity", "找不到实体"},
        {R"([{"type": "camera", "name": "other/c", "entity": "bench/rig"}])", "/sensors/0/name", "前缀"},
    };
    for (const Case& c : composeCases) {
        const auto loaded = load(c.sensors);
        check(!loaded.ok() && hasIssue(loaded.issues, Severity::Error, c.where, c.words),
              std::string{"组合错误 "} + c.sensors + " → " + c.where + "“" + c.words + "”" + dump(loaded.issues));
    }
    const auto missing = load(R"([{"type": "camera", "camera": "bench/nocam"}])");
    check(hasIssue(missing.issues, Severity::Error, "/sensors/0/camera", "bench/eye"), "找不到相机时列出场景中的相机" + dump(missing.issues));
}

}  // namespace

int main() {
    math();
    mjcfCameras();
    sceneCameras();
    errors();
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
