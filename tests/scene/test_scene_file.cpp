// 场景文件读取与校验（M2-05）。纯逻辑，不需要物理后端。
// 覆盖：夹具 two_carts.terra-scene.json 的全部字段与相对路径展开；两种旋转写法；各类错误的 JSON 指针与原因；
// 版本、format、未知字段（含嵌套）、类型错误、取值范围、非单位四元数的警告；JSON 语法错误；文件不存在。
// 用法：test_scene_file [--diagnose <场景文件>]

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>

#include "terra/scene/pose_math.hpp"
#include "terra/scene/scene.hpp"

#ifndef TERRA_TEST_ASSETS_DIR
#error "需要 TERRA_TEST_ASSETS_DIR"
#endif

namespace {

namespace fs = std::filesystem;
namespace sc = terra::scene;
using Severity = sc::SceneIssue::Severity;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

const std::string kDir = (fs::path{TERRA_TEST_ASSETS_DIR} / "scene").lexically_normal().string();
const std::string kScenePath = kDir + "/inline.terra-scene.json";

std::string dump(const sc::ParseResult& r) {
    std::string text;
    for (const auto& i : r.issues) {
        text += std::string{"\n  "} + (i.severity == Severity::Error ? "错误 " : "警告 ") + i.where + "：" + i.message;
    }
    return text;
}

bool has(const sc::ParseResult& r, Severity severity, const std::string& where, const std::string& part) {
    for (const auto& i : r.issues) {
        if (i.severity == severity && i.where == where && i.message.find(part) != std::string::npos) return true;
    }
    return false;
}

// 最小合法场景，再按需替换片段
std::string scene(const std::string& extra, const std::string& models = R"([{"name": "cart", "mjcf": "cart.xml"}])") {
    return R"({"format": "terra.scene", "version": 1, "models": )" + models + extra + "}";
}

void expectError(const std::string& text, const std::string& where, const std::string& part, const std::string& what) {
    const auto r = sc::parseScene(text, kScenePath);
    check(!r.document && has(r, Severity::Error, where, part), what + "（应在 " + where + " 报“" + part + "”）" + dump(r));
}

void testFixture() {
    const auto r = sc::readSceneFile(kDir + "/two_carts.terra-scene.json");
    check(r.document.has_value() && r.issues.empty(), "读取夹具 two_carts.terra-scene.json" + dump(r));
    if (!r.document) return;
    const sc::SceneDocument& d = *r.document;
    check(d.path == kDir + "/two_carts.terra-scene.json" && d.directory == kDir, "场景文件路径与基准目录");
    check(d.name == "两辆小车（M2-05 测试夹具）", "场景名");
    check(d.physics.timestep && *d.physics.timestep == 0.002 && !d.physics.gravity && !d.physics.integrator, "physics：只写了 timestep");
    check(d.models.size() == 3, "3 个模型实例");
    if (d.models.size() == 3) {
        check(d.models[0].name == "ground" && d.models[0].mjcf == kDir + "/ground.xml" && d.models[0].where == "/models/0",
              "实例 ground：路径相对场景目录展开，位置为 /models/0");
        check(d.models[1].placement.position.y == -0.6 && d.models[1].placement.orientation.w == 1.0, "cart_a 的位置，无旋转");
        const auto& q = d.models[2].placement.orientation;
        check(std::fabs(q.w - std::sqrt(0.5)) < 1e-12 && std::fabs(q.z - std::sqrt(0.5)) < 1e-12 && q.x == 0 && q.y == 0,
              "cart_b：rpyDeg [0, 0, 90] 为绕 Z 轴 90°");
    }
    check(d.visuals.size() == 4, "4 个 glTF 绑定");
    if (d.visuals.size() == 4) {
        const auto& v0 = d.visuals[0];
        check(v0.entity == "cart_a/chassis" && v0.gltf == kDir + "/cart.gltf" && v0.node == "Chassis" &&
                  v0.frame == sc::VisualFrame::Node && !v0.showGeoms && v0.scale == 1.0 && v0.where == "/visuals/0",
              "绑定 0 的默认值");
        check(d.visuals[1].frame == sc::VisualFrame::Assembly, "绑定 1：frame = assembly");
        check(d.visuals[2].showGeoms && d.visuals[2].local.position.z == 0.01, "绑定 2：showGeoms 与局部位置");
        check(d.visuals[3].entity == "world" && d.visuals[3].node.empty() && d.visuals[3].local.position.x == 2, "绑定 3：静态装饰，整个文件");
    }
    check(d.view.environment && *d.view.environment == "outdoor" && d.view.cameraPosition && d.view.cameraTarget &&
              d.view.cameraPosition->x == -3,
          "view：环境贴图与初始相机");
}

void testRotation() {
    // rpyDeg：R = Rz·Ry·Rx。绕 X 90° 后再绕 Z 90°：局部 Y 轴 → 世界 Z 轴，局部 X 轴 → 世界 Y 轴
    const auto q = sc::fromRpyDegrees(90, 0, 90);
    const auto y = sc::rotate(q, {0, 1, 0});
    const auto x = sc::rotate(q, {1, 0, 0});
    check(std::fabs(y.z - 1) < 1e-12 && std::fabs(x.y - 1) < 1e-12, "rpyDeg 的顺序：先绕 X，再绕 Y，最后绕 Z（世界轴）");

    const auto r = sc::parseScene(scene("", R"([{"name": "c", "mjcf": "cart.xml", "rotation": {"quat": [2, 0, 0, 0]}}])"), kScenePath);
    check(r.document && has(r, Severity::Warning, "/models/0/rotation/quat", "已归一化"), "非单位四元数：警告并归一化" + dump(r));
    if (r.document) check(r.document->models[0].placement.orientation.w == 1.0, "归一化后 w = 1");
}

void testInitialKeyframe() {
    const auto none = sc::parseScene(scene(""), kScenePath);
    check(none.document && none.document->initialKeyframe.empty(), "没写 initialKeyframe 时为空（从模型初值开始）" + dump(none));
    const auto r = sc::parseScene(scene(R"(, "initialKeyframe": "cart/offset")"), kScenePath);
    check(r.document && r.issues.empty() && r.document->initialKeyframe == "cart/offset", "读取 initialKeyframe" + dump(r));
}

void testControllersAndFixedBase() {
    const auto r = sc::parseScene(scene(R"(, "controllers": [{"type": "pd", "actuators": ["cart/*", "cart/drive"], "kp": 50, "kd": 0.5}])",
                                        R"([{"name": "cart", "mjcf": "cart.xml", "fixedBase": true, "torqueActuators": true}])"),
                                  kScenePath);
    check(r.document && r.issues.empty(), "读取 controllers 与 fixedBase" + dump(r));
    if (r.document) {
        const auto& d = *r.document;
        check(d.models[0].fixedBase, "fixedBase = true");
        check(d.models[0].torqueActuators, "torqueActuators = true（M4-06b）");
        check(d.controllers.size() == 1 && d.controllers[0].actuators.size() == 2 && d.controllers[0].kp == 50 &&
                  d.controllers[0].kd == 0.5 && d.controllers[0].where == "/controllers/0",
              "PD 层的执行器、增益与位置");
    }
    const auto noKd = sc::parseScene(scene(R"(, "controllers": [{"type": "pd", "actuators": ["a/x"], "kp": 1}])"), kScenePath);
    check(noKd.document && noKd.document->controllers[0].kd == 0.0, "kd 可省略（为 0）" + dump(noKd));
}

void testErrors() {
    expectError("{ not json", "", "JSON 语法错误", "JSON 语法错误");
    expectError("[1, 2]", "", "应为对象", "根不是对象");
    expectError(R"({"version": 1, "models": []})", "", "缺少 format", "缺少 format");
    expectError(R"({"format": "mujoco", "version": 1, "models": []})", "/format", "不是 Terra 场景文件", "format 不对");
    expectError(R"({"format": "terra.scene", "version": 2, "models": [{"name": "a", "mjcf": "a.xml"}]})", "/version",
                "高于本程序支持的版本 1", "更高的版本");
    expectError(R"({"format": "terra.scene", "version": 1.5, "models": [{"name": "a", "mjcf": "a.xml"}]})", "/version", "应为整数",
                "版本不是整数");
    expectError(R"({"format": "terra.scene", "version": 1})", "", "缺少 models", "缺少 models");
    expectError(scene("", "[]"), "/models", "至少有一个", "models 为空");
    expectError(scene(R"(, "extra": 1)"), "/extra", "未知字段“extra”", "根上的未知字段");
    expectError(scene("", R"([{"name": "c", "mjcf": "cart.xml", "positon": [0, 0, 0]}])"), "/models/0/positon", "未知字段“positon”",
                "模型实例中拼错的字段");
    expectError(scene("", R"([{"mjcf": "cart.xml"}])"), "/models/0", "缺少 name", "缺少实例名");
    expectError(scene("", R"([{"name": "c"}])"), "/models/0", "缺少 mjcf", "缺少 MJCF");
    expectError(scene("", R"([{"name": "c", "mjcf": "cart.xml", "position": [0, 0]}])"), "/models/0/position", "3 个数值",
                "位置不是 3 个数");
    expectError(scene("", R"([{"name": "c", "mjcf": "cart.xml", "position": [0, "x", 0]}])"), "/models/0/position/1", "应为数值",
                "位置中有字符串");
    expectError(scene("", R"([{"name": "c", "mjcf": "cart.xml", "rotation": {"quat": [1, 0, 0, 0], "rpyDeg": [0, 0, 0]}}])"),
                "/models/0/rotation", "其中之一", "两种旋转写法同时出现");
    expectError(scene("", R"([{"name": "c", "mjcf": "cart.xml", "rotation": {"quat": [0, 0, 0, 0]}}])"), "/models/0/rotation/quat",
                "长度为 0", "零四元数");
    expectError(scene(R"(, "physics": {"timestep": -1})"), "/physics/timestep", "大于 0", "负步长");
    expectError(scene(R"(, "physics": {"integrator": "Verlet"})"), "/physics/integrator", "未知的积分器", "未知积分器");
    expectError(scene(R"(, "physics": {"solver": "Jacobi"})"), "/physics/solver", "未知的求解器", "未知求解器");
    expectError(scene(R"(, "physics": {"iterations": 0})"), "/physics/iterations", "1 到 1000000", "迭代次数为 0");
    expectError(scene(R"(, "physics": {"tolerance": -1e-9})"), "/physics/tolerance", "不能为负", "负容差");
    expectError(scene(R"(, "physics": {"dt": 0.001})"), "/physics/dt", "未知字段", "physics 中的未知字段");
    expectError(scene(R"(, "visuals": {})"), "/visuals", "应为数组", "visuals 不是数组");
    expectError(scene(R"(, "visuals": [{"gltf": "a.gltf"}])"), "/visuals/0", "缺少 entity", "绑定缺少实体");
    expectError(scene(R"(, "visuals": [{"entity": "c/x"}])"), "/visuals/0", "缺少 gltf", "绑定缺少 glTF");
    expectError(scene(R"(, "visuals": [{"entity": "c/x", "gltf": "a.gltf", "node": ""}])"), "/visuals/0/node", "节点名为空", "空节点名");
    expectError(scene(R"(, "visuals": [{"entity": "c/x", "gltf": "a.gltf", "scale": 0}])"), "/visuals/0/scale", "大于 0", "缩放为 0");
    expectError(scene(R"(, "visuals": [{"entity": "c/x", "gltf": "a.gltf", "frame": "world"}])"), "/visuals/0/frame", "node",
                "未知的对齐方式");
    expectError(scene(R"(, "visuals": [{"entity": "c/x", "gltf": "a.gltf", "showGeoms": "yes"}])"), "/visuals/0/showGeoms",
                "true 或 false", "showGeoms 不是布尔值");
    expectError(scene(R"(, "view": {"camera": {"position": [0, 0, 1], "fov": 60}})"), "/view/camera/fov", "未知字段", "相机中的未知字段");

    expectError(scene(R"(, "initialKeyframe": 1)"), "/initialKeyframe", "应为字符串", "初始关键帧不是字符串");
    expectError(scene(R"(, "initialKeyframe": "")"), "/initialKeyframe", "关键帧名为空", "初始关键帧为空");

    expectError(scene("", R"([{"name": "c", "mjcf": "cart.xml", "fixedBase": 1}])"), "/models/0/fixedBase", "true 或 false",
                "fixedBase 不是布尔值");
    expectError(scene("", R"([{"name": "c", "mjcf": "cart.xml", "torqueActuators": "yes"}])"), "/models/0/torqueActuators", "true 或 false",
                "torqueActuators 不是布尔值");
    expectError(scene(R"(, "controllers": {})"), "/controllers", "应为数组", "controllers 不是数组");
    expectError(scene(R"(, "controllers": [{"type": "pid", "actuators": ["a/x"], "kp": 1}])"), "/controllers/0/type", "只支持 “pd”",
                "未知的控制层类型");
    expectError(scene(R"(, "controllers": [{"type": "pd", "actuators": [], "kp": 1}])"), "/controllers/0/actuators", "至少有一个",
                "执行器列表为空");
    expectError(scene(R"(, "controllers": [{"type": "pd", "actuators": ["a/x"], "kp": 0}])"), "/controllers/0/kp", "大于 0", "kp 为 0");
    expectError(scene(R"(, "controllers": [{"type": "pd", "actuators": ["a/x"], "kp": 1, "kd": -1}])"), "/controllers/0/kd", "不能为负",
                "kd 为负");
    expectError(scene(R"(, "controllers": [{"type": "pd", "actuators": ["a/x"]}])"), "/controllers/0", "缺少 kp", "缺少 kp");
    expectError(scene(R"(, "controllers": [{"type": "pd", "actuators": ["a/x"], "kp": 1, "ki": 2}])"), "/controllers/0/ki", "未知字段",
                "PD 层中的未知字段");

    // 多个错误一次报全
    const auto many = sc::parseScene(scene(R"(, "physics": {"timestep": 0, "solver": "X"}, "bogus": true)"), kScenePath);
    check(!many.document && many.issues.size() == 3, "一次报出全部 3 个错误" + dump(many));

    const auto missing = sc::readSceneFile(kDir + "/no_such.terra-scene.json");
    check(!missing.document && !missing.issues.empty() && missing.issues[0].message.find("场景文件不存在") != std::string::npos,
          "场景文件不存在");
}

}  // namespace

int main(int argc, char** argv) {
    // 演示用：打印一个场景文件的全部诊断（验证脚本 m2_05.sh 第 6 步）
    if (argc == 3 && std::string{argv[1]} == "--diagnose") {
        const auto r = sc::readSceneFile(argv[2]);
        std::printf("%s%s\n", r.document ? "解析成功" : "解析失败", dump(r).c_str());
        return 0;
    }
    testFixture();
    testRotation();
    testInitialKeyframe();
    testControllersAndFixedBase();
    testErrors();
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
