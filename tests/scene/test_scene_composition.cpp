// 模型实例组合（M2-04）。terra_scene 的纯逻辑（实例名、全局配置规则、错误归属、校验失败时不调用工厂）用假工厂测试；
// 与 MuJoCo 后端的集成用 assets/tests/scene/ 中的夹具测试：
//   - 两个相同小车 + 地面：名字前缀、实例归属（含没有名字的 body 与几何体）、放置位姿、分别控制互不串扰；
//   - 单个实例与直接加载同一 MJCF 的轨迹逐位相同（小车、带接触的下落立方体）；
//   - 全局配置：取第一个实例、冲突警告、场景指定的值、非法值；
//   - 错误归属：文件缺失、解析失败、合并后的编译失败。
// 本文件不包含 MuJoCo 头文件。

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/scene/composition.hpp"

#ifndef TERRA_TEST_ASSETS_DIR
#error "需要 TERRA_TEST_ASSETS_DIR"
#endif

namespace {

namespace fs = std::filesystem;
namespace sc = terra::scene;
namespace tp = terra::physics;
namespace td = terra::data;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

std::string asset(const char* name) { return (fs::path{TERRA_TEST_ASSETS_DIR} / name).string(); }
std::string sceneAsset(const char* name) { return (fs::path{TERRA_TEST_ASSETS_DIR} / "scene" / name).string(); }

std::string issues(const sc::ComposedWorld& w) {
    std::string text;
    for (const auto& i : w.issues) {
        text += std::string{"\n  "} + (i.severity == sc::SceneIssue::Severity::Error ? "错误" : "警告") + " [" + i.instance +
                "] " + i.message;
    }
    return text;
}

bool hasIssue(const sc::ComposedWorld& w, sc::SceneIssue::Severity severity, const std::string& instance,
              const std::string& part) {
    for (const auto& i : w.issues) {
        if (i.severity == severity && i.instance == instance && i.message.find(part) != std::string::npos) return true;
    }
    return false;
}

td::Pose at(double x, double y, double z, double yawDeg = 0.0) {
    td::Pose p;
    p.position = {x, y, z};
    constexpr double kPi = 3.14159265358979323846;
    const double half = yawDeg * kPi / 360.0;
    p.orientation = {std::cos(half), 0.0, 0.0, std::sin(half)};
    return p;
}

sc::ComposedWorld compose(const std::vector<sc::SceneModel>& models, const sc::PhysicsOverrides& overrides = {}) {
    return sc::composeWorld(models, overrides, tp::mujoco::composeWorld, "测试场景");
}

bool stepN(tp::PhysicsWorld& w, int n) {
    for (int i = 0; i < n; ++i) {
        if (const auto e = w.step()) {
            check(false, "第 " + std::to_string(e->step) + " 步出错：" + e->message);
            return false;
        }
    }
    return true;
}

const td::EntityState& stateOf(const tp::PhysicsWorld& w, const std::string& entity) {
    static const td::EntityState none{};
    const auto* e = w.description().findEntity(entity);
    check(e != nullptr, "找到实体 " + entity);
    if (e == nullptr) return none;
    return w.snapshot()->entities[e->id.value - 1];
}

// ---- 纯逻辑 ----

void testNames() {
    for (const char* ok : {"cart", "cart_a", "_x", "A-1", "z9"}) check(sc::isValidInstanceName(ok), std::string{"合法实例名 "} + ok);
    for (const char* bad : {"", "1cart", "-a", "a/b", "a b", "a.b", "车"}) {
        check(!sc::isValidInstanceName(bad), std::string{"不合法实例名 “"} + bad + "”");
    }
    check(sc::instancePrefix("cart_a") == "cart_a/", "前缀为“实例名/”");
    const std::vector<sc::SceneModel> inst = {{"a", "", {}, "", false}, {"ba", "", {}, "", false}};
    check(sc::attributeByPrefix("Error: Element name 'ba/x', id 3", inst) == 1, "按最先出现的前缀归属（“a/”出现在“ba/x”内部）");
    check(sc::attributeByPrefix("Error in 'a/y' and 'ba/z'", inst) == 0, "多个前缀时取最先出现的");
    check(sc::attributeByPrefix("Error without names", inst) == -1, "没有前缀时不归属");
}

void testResolve() {
    tp::PhysicsOptions a;
    a.timestep = 0.002;
    a.gravity = {0, 0, -9.81};
    a.integrator = "implicitfast";
    a.solver = "Newton";
    a.iterations = 100;
    a.tolerance = 1e-8;
    tp::PhysicsOptions b = a;
    b.timestep = 0.001;
    b.gravity = {0, 0, -9.7};
    const std::vector<sc::SceneModel> inst = {{"first", "", {}, "/models/0", false}, {"second", "", {}, "/models/1", false}};
    std::vector<sc::SceneIssue> warnings;
    const tp::PhysicsOptions r = sc::resolvePhysicsOptions({}, {a, b}, inst, warnings);
    check(r.timestep == 0.002 && r.gravity.z == -9.81, "场景没写：取第一个实例的值");
    check(warnings.size() == 2, "两项不同 → 两条警告，实际 " + std::to_string(warnings.size()));
    for (const auto& w : warnings) check(w.instance == "second" && w.where == "/models/1", "警告归属到第二个实例");

    sc::PhysicsOverrides o;
    o.timestep = 0.004;
    o.iterations = 50;
    warnings.clear();
    const tp::PhysicsOptions r2 = sc::resolvePhysicsOptions(o, {a, b}, inst, warnings);
    check(r2.timestep == 0.004 && r2.iterations == 50 && r2.solver == "Newton", "场景写了的项以场景为准，其余取第一个实例");
    check(warnings.size() == 1 && warnings[0].message.find("gravity") != std::string::npos, "场景写了的项不再警告，只剩重力");
}

void testValidationSkipsFactory() {
    int calls = 0;
    const sc::WorldFactory counting = [&](const tp::WorldComposition&) {
        ++calls;
        return tp::ComposeResult{};
    };
    const auto bad = sc::composeWorld({{"ok", "x.xml", {}, "/models/0", false}, {"9bad", "y.xml", {}, "/models/1", false},
                                       {"ok", "z.xml", {}, "/models/2", false}, {"empty", "", {}, "/models/3", false}},
                                      {}, counting, "l");
    check(calls == 0 && !bad.ok(), "校验失败时不调用工厂");
    check(hasIssue(bad, sc::SceneIssue::Severity::Error, "9bad", "不合法"), "不合法的实例名" + issues(bad));
    check(hasIssue(bad, sc::SceneIssue::Severity::Error, "ok", "重复（与 /models/0 相同）"), "重复的实例名指出第一次出现的位置");
    check(hasIssue(bad, sc::SceneIssue::Severity::Error, "empty", "没有指定 MJCF"), "缺少 MJCF 路径");
    check(!sc::composeWorld({}, {}, counting, "l").ok() && calls == 0, "没有实例时报错");

    // 工厂返回的问题：model = -1 的错误按前缀归属
    const sc::WorldFactory failing = [&](const tp::WorldComposition& request) {
        ++calls;
        check(request.models.size() == 2 && request.models[1].prefix == "beta/" && request.sourceLabel == "l",
              "组合请求带前缀与标签");
        tp::ComposeResult r;
        r.issues.push_back({tp::ComposeIssue::Severity::Error, -1, tp::LoadErrorKind::InvalidModel, "Element name 'beta/link'"});
        r.issues.push_back({tp::ComposeIssue::Severity::Warning, 0, tp::LoadErrorKind::InvalidModel, "注意"});
        return r;
    };
    const auto failed = sc::composeWorld({{"alpha", "a.xml", {}, "", false}, {"beta", "b.xml", {}, "", false}}, {}, failing, "l");
    check(calls == 1 && !failed.ok(), "工厂失败");
    check(hasIssue(failed, sc::SceneIssue::Severity::Error, "beta", "beta/link"), "合并后的错误按前缀归属到 beta" + issues(failed));
    check(hasIssue(failed, sc::SceneIssue::Severity::Warning, "alpha", "注意"), "后端给出的实例下标直接归属");
}

// ---- 与 MuJoCo 后端集成 ----

void testTwoCarts() {
    auto w = compose({{"ground", sceneAsset("ground.xml"), {}, "", false},
                      {"cart_a", sceneAsset("cart.xml"), at(0, -0.6, 0), "", false},
                      {"cart_b", sceneAsset("cart.xml"), at(0, 0.6, 0, 90), "", false}});
    check(w.ok(), "组合地面与两辆小车" + issues(w));
    if (!w.ok()) return;
    check(w.issues.empty(), "没有警告" + issues(w));
    const auto& d = w.world->description();
    check(d.sourcePath == "测试场景" && d.modelInstances.size() == 3, "描述：来源为标签，3 个实例");
    if (d.modelInstances.size() == 3) {
        check(d.modelInstances[1].name == "cart_a" && d.modelInstances[2].placement.position.y == 0.6 &&
                  d.modelInstances[1].sourcePath == sceneAsset("cart.xml"),
              "实例的名字、放置位姿与来源");
    }
    for (const char* name : {"cart_a/chassis", "cart_a/mast", "cart_b/chassis", "cart_b/mast"}) {
        check(d.findEntity(name) != nullptr, std::string{"实体全名 "} + name);
    }
    check(d.findEntity("chassis") == nullptr, "不带前缀的原名不存在");
    check(d.entities.size() == 6, "两辆车各 3 个实体（含没有名字的 body），实际 " + std::to_string(d.entities.size()));
    int unnamed = 0;
    for (const auto& e : d.entities) {
        const std::string expect = e.name.rfind("cart_a/", 0) == 0 ? "cart_a" : e.name.rfind("cart_b/", 0) == 0 ? "cart_b" : "";
        if (e.name.empty()) {
            ++unnamed;
            check(e.modelInstance.value == 2 || e.modelInstance.value == 3, "没有名字的 body 也归属到小车实例");
            check(d.entity(e.parent) != nullptr && d.entity(e.parent)->modelInstance == e.modelInstance, "与父实体属于同一实例");
        } else {
            check(d.modelInstance(e.modelInstance) != nullptr && d.modelInstance(e.modelInstance)->name == expect,
                  "实体 " + e.name + " 归属到 " + expect);
        }
    }
    check(unnamed == 2, "没有名字的 body 对外仍为空名字（每辆车 1 个）");
    const auto* floor = d.findGeom("ground/floor");
    check(floor != nullptr && !floor->entity.valid() && floor->modelInstance.value == 1, "地面：静态几何，属于 ground 实例");
    int hiddenGeoms = 0;
    for (const auto& g : d.geoms) {
        if (g.name.empty()) {
            ++hiddenGeoms;
            check(g.modelInstance.value >= 2, "没有名字的几何体归属到小车实例");
        }
    }
    check(hiddenGeoms == 2, "没有名字的几何体对外仍为空名字");

    // 放置位姿
    const auto a0 = stateOf(*w.world, "cart_a/chassis").pose;
    const auto b0 = stateOf(*w.world, "cart_b/chassis").pose;
    check(std::fabs(a0.position.y + 0.6) < 1e-12 && std::fabs(a0.position.z - 0.1) < 1e-12, "cart_a 放在 y = −0.6");
    check(std::fabs(b0.position.y - 0.6) < 1e-12 && std::fabs(std::fabs(b0.orientation.z) - std::sqrt(0.5)) < 1e-9,
          "cart_b 放在 y = 0.6，绕 Z 转 90°");

    // 分别控制：只推 cart_a
    const auto* driveA = d.findActuator("cart_a/drive");
    const auto* driveB = d.findActuator("cart_b/drive");
    check(driveA != nullptr && driveB != nullptr, "执行器全名 cart_a/drive、cart_b/drive");
    if (driveA == nullptr || driveB == nullptr) return;
    check(!w.world->setControl({{driveA->id, {2.0}}}), "设置 cart_a 的控制");
    if (!stepN(*w.world, 500)) return;
    const auto a1 = stateOf(*w.world, "cart_a/chassis").pose;
    const auto b1 = stateOf(*w.world, "cart_b/chassis").pose;
    check(a1.position.x > a0.position.x + 0.05, "cart_a 沿 +X 前进");
    check(b1.position.x == b0.position.x && b1.position.y == b0.position.y, "cart_b 没有动（控制互不串扰）");

    // 再推 cart_b：它的滑轨随放置转了 90°，沿世界 +Y 前进
    check(!w.world->setControl({{driveB->id, {2.0}}}), "设置 cart_b 的控制");
    if (!stepN(*w.world, 500)) return;
    const auto b2 = stateOf(*w.world, "cart_b/chassis").pose;
    check(b2.position.y > b1.position.y + 0.05 && std::fabs(b2.position.x - b1.position.x) < 1e-9,
          "cart_b 沿世界 +Y 前进（滑轨随放置位姿旋转）");
}

void compareTrajectories(const std::string& path, int steps, bool drive) {
    auto direct = tp::mujoco::loadWorld(td::ModelSource{path});
    auto composed = compose({{"solo", path, {}, "", false}});
    check(direct.world != nullptr && composed.ok(), "直接加载与单实例组合都成功：" + path + issues(composed));
    if (!direct.world || !composed.ok()) return;
    auto& a = *direct.world;
    auto& b = *composed.world;
    check(a.description().timestep == b.description().timestep && a.description().settings.integrator == b.description().settings.integrator &&
              a.description().entities.size() == b.description().entities.size(),
          "步长、积分器、实体数相同");
    bool identical = true;
    int firstDiff = -1;
    for (int s = 0; s <= steps && identical; ++s) {
        const auto sa = a.snapshot();
        const auto sb = b.snapshot();
        for (std::size_t e = 0; e < sa->entities.size() && e < sb->entities.size(); ++e) {
            const auto& pa = sa->entities[e].pose;
            const auto& pb = sb->entities[e].pose;
            if (pa.position.x != pb.position.x || pa.position.y != pb.position.y || pa.position.z != pb.position.z ||
                pa.orientation.w != pb.orientation.w || pa.orientation.x != pb.orientation.x ||
                pa.orientation.y != pb.orientation.y || pa.orientation.z != pb.orientation.z) {
                identical = false;
                firstDiff = s;
            }
        }
        if (s == steps) break;
        if (drive && !a.description().actuators.empty()) {
            const double u = 3.0 * std::sin(0.01 * s);
            (void)a.setControl({{a.description().actuators[0].id, {u}}});
            (void)b.setControl({{b.description().actuators[0].id, {u}}});
        }
        if (a.step() || b.step()) {
            check(false, "步进出错");
            return;
        }
    }
    check(identical, "单实例组合与直接加载的轨迹逐位相同（" + path + "，" + std::to_string(steps) + " 步）" +
                         (firstDiff >= 0 ? "，第 " + std::to_string(firstDiff) + " 步起不同" : std::string{}));
}

void testOptions() {
    auto w = compose({{"cart", sceneAsset("cart.xml"), {}, "/models/0", false}, {"fast", sceneAsset("cart_fast.xml"), at(0, 1, 0), "/models/1", false}});
    check(w.ok(), "组合配置不同的两个模型" + issues(w));
    if (w.ok()) {
        check(w.world->description().timestep == 0.002, "步长取第一个实例的 0.002");
        check(std::fabs(w.world->description().settings.gravity.z + 9.81) < 1e-12, "重力取第一个实例的值");
    }
    check(hasIssue(w, sc::SceneIssue::Severity::Warning, "fast", "timestep"), "步长不同的警告" + issues(w));
    check(hasIssue(w, sc::SceneIssue::Severity::Warning, "fast", "gravity"), "重力不同的警告");
    check(hasIssue(w, sc::SceneIssue::Severity::Warning, "fast", "cone"), "其他全局设置（摩擦锥）不同的警告");

    sc::PhysicsOverrides o;
    o.timestep = 0.004;
    o.gravity = td::Vec3{0, 0, -1.62};
    auto v = compose({{"cart", sceneAsset("cart.xml"), {}, "", false}, {"fast", sceneAsset("cart_fast.xml"), {}, "", false}}, o);
    check(v.ok() && v.world->description().timestep == 0.004 && std::fabs(v.world->description().settings.gravity.z + 1.62) < 1e-12,
          "场景指定的步长与重力生效" + issues(v));
    check(!hasIssue(v, sc::SceneIssue::Severity::Warning, "fast", "timestep") && hasIssue(v, sc::SceneIssue::Severity::Warning, "fast", "cone"),
          "场景指定的项不再警告，其他设置仍警告");

    sc::PhysicsOverrides bad;
    bad.integrator = "Verlet";
    auto x = compose({{"cart", sceneAsset("cart.xml"), {}, "", false}}, bad);
    check(!x.ok() && hasIssue(x, sc::SceneIssue::Severity::Error, "", "未知的积分器：Verlet"), "非法积分器名报错" + issues(x));
}

void testErrors() {
    auto missing = compose({{"cart", sceneAsset("cart.xml"), {}, "", false}, {"ghost", sceneAsset("no_such.xml"), {}, "/models/1", false}});
    check(!missing.ok() && hasIssue(missing, sc::SceneIssue::Severity::Error, "ghost", "模型文件不存在"), "文件缺失归属到实例" + issues(missing));
    for (const auto& i : missing.issues) {
        if (i.instance == "ghost") check(i.where == "/models/1", "错误带实例的位置");
    }
    auto broken = compose({{"cart", sceneAsset("cart.xml"), {}, "", false}, {"broken", sceneAsset("broken.xml"), {}, "", false}});
    check(!broken.ok() && hasIssue(broken, sc::SceneIssue::Severity::Error, "broken", "MJCF 解析失败"), "解析失败归属到实例" + issues(broken));
    auto bad = compose({{"cart", sceneAsset("cart.xml"), {}, "", false}, {"bad", sceneAsset("bad_compile.xml"), {}, "", false}});
    check(!bad.ok(), "合并后编译失败");
    check(hasIssue(bad, sc::SceneIssue::Severity::Error, "bad", "编译失败"), "合并后的编译错误按前缀归属到 bad" + issues(bad));
    std::printf("INFO 编译错误示例：%s\n", bad.issues.empty() ? "" : bad.issues.front().message.c_str());
}

}  // namespace

int main() {
    testNames();
    testResolve();
    testValidationSkipsFactory();
    testTwoCarts();
    compareTrajectories(sceneAsset("cart.xml"), 1000, true);
    compareTrajectories(asset("falling_cube.xml"), 2000, false);
    testOptions();
    testErrors();
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
