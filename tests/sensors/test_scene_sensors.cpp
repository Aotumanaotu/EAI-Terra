// 场景文件的传感器（M4-02，M4-01 设计稿第 2 节）。
//   - 夹具 assets/tests/sensors/imu_added.terra-scene.json：
//       装在实体上的 IMU（fall_imu2 与原有的 fall_imu 读数逐位相同；tilt_imu2 抵消安装角后比力为 (0, 0, 9.81)）；
//       给只有陀螺仪的 bare_site 补上加速度计；已有的 level_imu 只设频率；
//       频率：写了的按场景（100、30 Hz），没写的等于物理频率 500 Hz，高于物理频率的按 500 Hz 并警告；
//       “场景添加”标记正确；添加传感器不改变动力学：与不加传感器的同一场景推进 500 步，全部实体位姿逐位相同。
//   - 解析错误（JSON 指针 + 原因）：未知类型、相机与雷达写 site 报未知字段、site 与 entity 同时写或都不写、装在实体上缺 name、
//     name 与 site 不同、site 带位姿、频率不为正、名字重复。
//   - 组合错误：没有对应的实例、找不到安装点（列出模型中的安装点）、找不到实体、名字前缀与实例不符、与已有安装点重名。
//   - 参考机器人（有时）：G1 自带的两个 IMU 被发现；给 Go2 的安装点 imu 加 IMU，站立保持 1 s 后比力约 9.81，
//     姿态与机身实体一致。参考机器人缺失时跳过这一部分（不影响其余检查）。

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>

#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/scene/scene.hpp"
#include "terra/simulation/pd_layer.hpp"

#ifndef TERRA_TEST_ASSETS_DIR
#error "需要 TERRA_TEST_ASSETS_DIR"
#endif
#ifndef TERRA_ASSETS_DIR
#error "需要 TERRA_ASSETS_DIR"
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

bool hasIssue(const std::vector<sc::SceneIssue>& issues, Severity severity, const std::string& where, const std::string& words) {
    for (const auto& i : issues) {
        if (i.severity == severity && i.where == where && i.message.find(words) != std::string::npos) return true;
    }
    return false;
}

const std::string kSceneDir = std::string{TERRA_TEST_ASSETS_DIR} + "/sensors";

std::string sceneText(const std::string& sensors) {
    return R"({"format": "terra.scene", "version": 1, "models": [{"name": "bench", "mjcf": "imu_bench.xml"}], "sensors": )" + sensors + "}";
}

sc::ParseResult parse(const std::string& sensors) { return sc::parseScene(sceneText(sensors), kSceneDir + "/inline.terra-scene.json"); }

sc::SceneLoadResult load(const std::string& sensors) {
    auto parsed = parse(sensors);
    if (!parsed.document) {
        sc::SceneLoadResult out;
        out.issues = parsed.issues;
        return out;
    }
    return sc::instantiate(std::move(*parsed.document), terra::physics::mujoco::composeWorld);
}

double rateOf(const sc::SceneInstance& s, const char* name) {
    const auto* d = s.world->description().findSensor(name);
    if (d == nullptr) return -1;
    for (const auto& c : s.sensors) {
        if (c.sensor == d->id) return c.rateHz;
    }
    return -1;
}

void addedFixture() {
    auto loaded = sc::loadScene(kSceneDir + "/imu_added.terra-scene.json", terra::physics::mujoco::composeWorld);
    check(loaded.ok(), "加载 imu_added.terra-scene.json" + dump(loaded.issues));
    if (!loaded.ok()) return;
    sc::SceneInstance& scene = *loaded.scene;
    const auto& d = scene.world->description();
    check(d.sensors.size() == 7 && scene.sensors.size() == 7, "7 个 IMU（原有 4 个、实体上 2 个、补齐 1 个），实际 " + std::to_string(d.sensors.size()));
    const std::pair<const char*, bool> origins[] = {
        {"bench/level_imu", false}, {"bench/fall_imu", false}, {"bench/fall_imu2", true}, {"bench/tilt_imu2", true}, {"bench/bare_site", true}};
    for (const auto& [name, added] : origins) {
        const auto* s = d.findSensor(name);
        check(s != nullptr && s->added == added, std::string{name} + (added ? " 标为场景添加" : " 标为 MJCF 原有"));
    }
    if (const auto* s = d.findSensor("bench/fall_imu2")) {
        const auto* e = d.entity(s->entity);
        check(e != nullptr && e->name == "bench/faller" && s->modelInstance.value == 1, "fall_imu2 装在 bench/faller 上，属于实例 bench");
    }
    // 频率
    check(rateOf(scene, "bench/fall_imu2") == 100.0 && rateOf(scene, "bench/bare_site") == 30.0, "场景写了的频率：100 Hz、30 Hz");
    check(std::fabs(rateOf(scene, "bench/spin_imu") - 500.0) < 1e-9, "没写频率的等于物理频率 500 Hz");
    check(std::fabs(rateOf(scene, "bench/level_imu") - 500.0) < 1e-9 &&
              hasIssue(loaded.issues, Severity::Warning, "/sensors/3/rateHz", "高于物理频率"),
          "1000 Hz 高于物理频率：按 500 Hz 并警告" + dump(loaded.issues));

    // 读数：装在实体原点的 fall_imu2 与原有 fall_imu 逐位相同；tilt_imu2 抵消了 30° 安装角
    bool same = true;
    bool levelled = true;
    bool bare = true;
    for (int i = 0; i < 200; ++i) {
        const auto snap = scene.world->snapshot();
        const auto at = [&](const char* name) { return snap->sensors[d.findSensor(name)->id.value - 1].values; };
        same = same && at("bench/fall_imu") == at("bench/fall_imu2");
        const auto t = at("bench/tilt_imu2");
        levelled = levelled && std::fabs(t[0]) < 1e-9 && std::fabs(t[1]) < 1e-9 && std::fabs(t[2] - 9.81) < 1e-9;
        const auto b = at("bench/bare_site");
        bare = bare && std::fabs(b[2] - 9.81) < 1e-9 && b[3] == 0.0 && b[4] == 0.0 && b[5] == 0.0;
        scene.world->step();
    }
    check(same, "装在实体原点的 fall_imu2 与原有 fall_imu 每一步逐位相同");
    check(levelled, "tilt_imu2（安装角 −30° 抵消实体的 30°）：比力 (0, 0, 9.81)");
    check(bare, "bare_site 补上加速度计后成为 IMU：比力 (0, 0, 9.81)，原有陀螺仪读数 0");

    // 添加传感器不改变动力学
    auto plain = load("[]");
    auto withImu = sc::loadScene(kSceneDir + "/imu_added.terra-scene.json", terra::physics::mujoco::composeWorld);
    check(plain.ok() && withImu.ok(), "两个对照场景可以加载" + dump(plain.issues));
    if (plain.ok() && withImu.ok()) {
        auto& a = *plain.scene->world;
        auto& b = *withImu.scene->world;
        const int spin = a.description().findKeyframe("spin");
        a.resetToKeyframe(static_cast<std::size_t>(spin));
        b.resetToKeyframe(static_cast<std::size_t>(b.description().findKeyframe("spin")));
        bool identical = a.description().entities.size() == b.description().entities.size();
        for (int i = 0; i < 500 && identical; ++i) {
            a.step();
            b.step();
            const auto sa = a.snapshot();
            const auto sb = b.snapshot();
            for (std::size_t e = 0; e < sa->entities.size(); ++e) {
                const auto& p = sa->entities[e].pose;
                const auto& q = sb->entities[e].pose;
                identical = identical && p.position.x == q.position.x && p.position.y == q.position.y && p.position.z == q.position.z &&
                            p.orientation.w == q.orientation.w && p.orientation.x == q.orientation.x &&
                            p.orientation.y == q.orientation.y && p.orientation.z == q.orientation.z;
            }
        }
        check(identical, "添加 IMU 不改变动力学：500 步内全部实体位姿逐位相同");
        check(plain.scene->sensors.size() == 4, "不加传感器时只有 MJCF 中原有的 4 个 IMU");
    }
}

void parseErrors() {
    struct Case {
        const char* sensors;
        const char* where;
        const char* words;
    };
    const Case cases[] = {
        {R"([{"type": "sonar", "site": "bench/level_imu"}])", "/sensors/0/type", "未知的传感器类型"},
        {R"([{"type": "camera", "site": "bench/level_imu"}])", "/sensors/0/site", "未知字段"},
        {R"([{"type": "lidar2d", "site": "bench/level_imu"}])", "/sensors/0/site", "未知字段"},
        {R"([{"site": "bench/level_imu"}])", "/sensors/0", "缺少 type"},
        {R"([{"type": "imu", "site": "bench/level_imu", "entity": "bench/level"}])", "/sensors/0", "其中之一"},
        {R"([{"type": "imu"}])", "/sensors/0", "其中之一"},
        {R"([{"type": "imu", "entity": "bench/level"}])", "/sensors/0", "需要 name"},
        {R"([{"type": "imu", "name": "bench/x", "site": "bench/level_imu"}])", "/sensors/0/name", "就是安装点名"},
        {R"([{"type": "imu", "site": "bench/level_imu", "position": [0, 0, 1]}])", "/sensors/0", "不能再写 position"},
        {R"([{"type": "imu", "site": "bench/level_imu", "rateHz": 0}])", "/sensors/0/rateHz", "大于 0"},
        {R"([{"type": "imu", "site": "bench/level_imu", "speed": 1}])", "/sensors/0/speed", "未知字段"},
        {R"([{"type": "imu", "site": "bench/level_imu"}, {"type": "imu", "site": "bench/level_imu"}])", "/sensors/1/name", "重复"},
        {R"({"type": "imu"})", "/sensors", "应为数组"},
    };
    for (const Case& c : cases) {
        const auto parsed = parse(c.sensors);
        check(!parsed.document && hasIssue(parsed.issues, Severity::Error, c.where, c.words),
              std::string{"解析错误 "} + c.sensors + " → " + c.where + "“" + c.words + "”" + dump(parsed.issues));
    }
}

void composeErrors() {
    struct Case {
        const char* sensors;
        const char* where;
        const char* words;
    };
    const Case cases[] = {
        {R"([{"type": "imu", "site": "ghost/level_imu"}])", "/sensors/0/site", "没有对应的实例"},
        {R"([{"type": "imu", "site": "level_imu"}])", "/sensors/0/site", "没有对应的实例"},
        {R"([{"type": "imu", "name": "bench/a", "entity": "bench/nobody"}])", "", "找不到实体“nobody”"},
        {R"([{"type": "imu", "site": "bench/nosite"}])", "", "找不到安装点“nosite”"},
        {R"([{"type": "imu", "name": "other/a", "entity": "bench/level"}])", "/sensors/0/name", "前缀"},
        {R"([{"type": "imu", "name": "bench/level_imu", "entity": "bench/level"}])", "", "已有名为“level_imu”的安装点"},
    };
    for (const Case& c : cases) {
        const auto loaded = load(c.sensors);
        bool found = false;
        for (const auto& i : loaded.issues) {
            found = found || (i.severity == Severity::Error && (c.where[0] == '\0' || i.where == c.where) &&
                              i.message.find(c.words) != std::string::npos);
        }
        check(!loaded.ok() && found, std::string{"组合错误 "} + c.sensors + " → “" + c.words + "”" + dump(loaded.issues));
    }
    // 找不到安装点时列出模型中的安装点
    const auto loaded = load(R"([{"type": "imu", "site": "bench/nosite"}])");
    bool listed = false;
    for (const auto& i : loaded.issues) listed = listed || i.message.find("level_imu") != std::string::npos;
    check(listed, "找不到安装点时列出模型中的安装点" + dump(loaded.issues));
}

// 参考机器人：返回是否检查了（缺失时跳过）
bool robots() {
    const std::string g1 = std::string{TERRA_ASSETS_DIR} + "/scenes/robots/g1_suspended.terra-scene.json";
    const std::string go2 = std::string{TERRA_ASSETS_DIR} + "/scenes/robots/go2.terra-scene.json";
    if (!fs::exists(std::string{TERRA_ASSETS_DIR} + "/robots/unitree_g1/g1.xml") ||
        !fs::exists(std::string{TERRA_ASSETS_DIR} + "/robots/unitree_go2/go2.xml")) {
        std::printf("参考机器人未准备，跳过 G1 / Go2 部分\n");
        return false;
    }
    auto g = sc::loadScene(g1, terra::physics::mujoco::composeWorld);
    check(g.ok(), "加载 G1 悬挂场景" + dump(g.issues));
    if (g.ok()) {
        const auto& d = g.scene->world->description();
        check(d.findSensor("g1/imu_in_pelvis") != nullptr && d.findSensor("g1/imu_in_torso") != nullptr,
              "G1 自带的两个 IMU 被发现（g1/imu_in_pelvis、g1/imu_in_torso）");
    }

    auto parsed = sc::readSceneFile(go2);
    check(parsed.document.has_value(), "读取 Go2 场景" + dump(parsed.issues));
    if (!parsed.document) return true;
    sc::SensorSpec imu;
    imu.type = "imu";
    imu.site = "go2/imu";
    imu.name = "go2/imu";
    imu.where = "/sensors/0";
    parsed.document->sensors.push_back(imu);
    auto loaded = sc::instantiate(std::move(*parsed.document), terra::physics::mujoco::composeWorld);
    check(loaded.ok(), "Go2 场景加 IMU（装在安装点 go2/imu 上）" + dump(loaded.issues));
    if (!loaded.ok()) return true;
    auto& scene = *loaded.scene;
    auto& world = *scene.world;
    const auto& d = world.description();
    const auto* s = d.findSensor("go2/imu");
    check(s != nullptr && s->added, "Go2 的 IMU 标为场景添加");
    if (s == nullptr) return true;
    // 站立保持（PD 层保持关键帧姿态）1 s
    terra::simulation::PdLayer pd(d, scene.pd);
    for (int i = 0; i < 500; ++i) {
        if (!pd.empty()) world.setControl(pd.process({}, *world.snapshot()));
        world.step();
    }
    const auto snap = world.snapshot();
    const auto& v = snap->sensors[s->id.value - 1].values;
    const double norm = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    check(std::fabs(norm - 9.81) < 0.3, "Go2 站立 1 s 后比力的模约为 9.81：" + std::to_string(norm));
    const auto& base = snap->entities[s->entity.value - 1].pose.orientation;
    const double dot = std::fabs(base.w * v[6] + base.x * v[7] + base.y * v[8] + base.z * v[9]);
    check(std::fabs(dot - 1.0) < 1e-9, "Go2 IMU 的姿态与机身实体一致（安装点没有旋转）");
    return true;
}

}  // namespace

int main() {
    addedFixture();
    parseErrors();
    composeErrors();
    robots();
    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
