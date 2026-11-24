// IMU（M4-02）：MuJoCo 传感器经后端无关接口读出，与解析解比较。夹具 assets/tests/sensors/imu_bench.xml（见文件注释）：
//   - 发现：同一安装点上有加速度计与陀螺仪的才是 IMU（4 个）；只有陀螺仪的 bare_site 不是；名字为安装点名，实体与安装位姿正确；
//   - level：比力 (0, 0, 9.81)，角速度 0；tilted（绕 X 30°）：比力 (0, 9.81 sin30°, 9.81 cos30°)，姿态为绕 X 30°；
//   - spinner（关键帧 spin，ω = 2 rad/s，安装点离轴 0.5 m）：每一步角速度 (0, 0, 2)、比力 (−2, 0, 9.81)；
//   - faller（自由落体）：比力 ≈ 0，角速度 0；
//   - 第 0 步、每一步、reset 之后都有读数，读数个数为 10；快照修订号 3、描述修订号 6。

#include <cmath>
#include <cstdio>
#include <string>

#include "terra/physics/mujoco/mujoco_backend.hpp"

#ifndef TERRA_TEST_ASSETS_DIR
#error "需要 TERRA_TEST_ASSETS_DIR"
#endif

namespace {

namespace td = terra::data;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

struct Expected {
    double acc[3];
    double gyro[3];
};

std::string values(const std::vector<double>& v) {
    std::string out;
    for (const double x : v) {
        char buf[32];
        std::snprintf(buf, sizeof buf, "%s%.12g", out.empty() ? "" : ", ", x);
        out += buf;
    }
    return "(" + out + ")";
}

bool matches(const std::vector<double>& v, const Expected& e, double tol) {
    if (v.size() != 10) return false;
    for (std::size_t c = 0; c < 3; ++c) {
        if (std::fabs(v[c] - e.acc[c]) > tol || std::fabs(v[3 + c] - e.gyro[c]) > tol) return false;
    }
    return true;
}

const std::vector<double>* reading(const td::PhysicsSnapshot& snap, const td::ModelDescription& d, const char* name) {
    const auto* s = d.findSensor(name);
    if (s == nullptr || s->id.value > snap.sensors.size()) return nullptr;
    return &snap.sensors[s->id.value - 1].values;
}

}  // namespace

int main() {
    check(td::ModelDescription::kRevision >= 6, "模型描述修订号 ≥ 6（M4-03 起为 7）");
    check(td::PhysicsSnapshot::kContractRevision == 3, "快照修订号 3");

    auto loaded = terra::physics::mujoco::loadWorld({std::string{TERRA_TEST_ASSETS_DIR} + "/sensors/imu_bench.xml"});
    check(loaded.world != nullptr, "加载 imu_bench.xml" + (loaded.error ? "：" + loaded.error->message : std::string{}));
    if (!loaded.world) return 1;
    auto& world = *loaded.world;
    const auto& d = world.description();

    // 发现
    check(d.sensors.size() == 4, "4 个 IMU（只有陀螺仪的 bare_site 不算），实际 " + std::to_string(d.sensors.size()));
    for (const char* name : {"level_imu", "tilt_imu", "spin_imu", "fall_imu"}) {
        const auto* s = d.findSensor(name);
        check(s != nullptr && s->kind == td::SensorKind::Imu && !s->added, std::string{"IMU "} + name + "（MJCF 中原有）");
    }
    check(d.findSensor("bare_site") == nullptr, "bare_site 只有陀螺仪，不是 IMU");
    if (const auto* s = d.findSensor("spin_imu")) {
        const auto* e = d.entity(s->entity);
        check(e != nullptr && e->name == "spinner" && s->mount.position.x == 0.5 && s->mount.position.y == 0.0 &&
                  s->mount.orientation.w == 1.0,
              "spin_imu 装在 spinner 上，相对实体原点 (0.5, 0, 0)");
    }
    for (std::size_t k = 0; k < d.sensors.size(); ++k) check(d.sensors[k].id.value == k + 1, "传感器 ID 从 1 连续编号");

    const double g = 9.81;
    const double s30 = std::sin(30.0 * 3.14159265358979323846 / 180.0);
    const double c30 = std::cos(30.0 * 3.14159265358979323846 / 180.0);
    const Expected level{{0, 0, g}, {0, 0, 0}};
    const Expected tilted{{0, g * s30, g * c30}, {0, 0, 0}};
    const Expected spinning{{-2.0, 0, g}, {0, 0, 2.0}};
    const Expected falling{{0, 0, 0}, {0, 0, 0}};

    // 第 0 步（加载后的初始状态）
    {
        const auto snap = world.snapshot();
        check(snap->sensors.size() == d.sensors.size(), "快照中有全部传感器的读数");
        const auto* l = reading(*snap, d, "level_imu");
        check(l && matches(*l, level, 1e-9), "level：比力 (0, 0, 9.81)，角速度 0：" + (l ? values(*l) : std::string{"无读数"}));
        const auto* t = reading(*snap, d, "tilt_imu");
        check(t && matches(*t, tilted, 1e-9), "tilted：比力 (0, 9.81 sin30°, 9.81 cos30°)：" + (t ? values(*t) : std::string{"无读数"}));
        if (t && t->size() == 10) {
            const double hw = std::cos(15.0 * 3.14159265358979323846 / 180.0);
            const double hx = std::sin(15.0 * 3.14159265358979323846 / 180.0);
            const double sign = (*t)[6] < 0 ? -1.0 : 1.0;
            check(std::fabs(sign * (*t)[6] - hw) < 1e-12 && std::fabs(sign * (*t)[7] - hx) < 1e-12 && std::fabs((*t)[8]) < 1e-12 &&
                      std::fabs((*t)[9]) < 1e-12,
                  "tilted 的姿态为绕 X 轴 30°：" + values(*t));
        }
    }

    // 旋转：关键帧给 2 rad/s，没有阻尼，每一步都保持
    const int spin = d.findKeyframe("spin");
    check(spin >= 0 && world.resetToKeyframe(static_cast<std::size_t>(spin)), "按关键帧 spin 重置");
    bool spinOk = true;
    bool fallOk = true;
    bool staticOk = true;
    std::string firstBad;
    for (int i = 0; i < 500; ++i) {
        const auto snap = world.snapshot();
        const auto* sp = reading(*snap, d, "spin_imu");
        const auto* fa = reading(*snap, d, "fall_imu");
        const auto* l = reading(*snap, d, "level_imu");
        if (!(sp && matches(*sp, spinning, 1e-9)) && spinOk) firstBad += " spinner 第 " + std::to_string(i) + " 步 " + (sp ? values(*sp) : "");
        if (!(fa && matches(*fa, falling, 1e-9)) && fallOk) firstBad += " faller 第 " + std::to_string(i) + " 步 " + (fa ? values(*fa) : "");
        spinOk = spinOk && sp && matches(*sp, spinning, 1e-9);
        fallOk = fallOk && fa && matches(*fa, falling, 1e-9);
        staticOk = staticOk && l && matches(*l, level, 1e-9);
        if (const auto error = world.step()) {
            check(false, "推进出错：" + error->message);
            break;
        }
    }
    check(spinOk, "spinner 每一步：角速度 (0, 0, 2)，比力 (−2, 0, 9.81)" + firstBad);
    check(fallOk, "faller 自由落体每一步：比力 ≈ 0，角速度 0" + firstBad);
    check(staticOk, "level 在推进中保持 (0, 0, 9.81)");
    if (const auto* e = d.findEntity("faller")) {
        const double z = world.snapshot()->entities[e->id.value - 1].pose.position.z;
        check(z < 5.0 - 1.0, "faller 确实在下落（1 s 后 z = " + std::to_string(z) + "）");
    }

    // reset 之后仍有读数，且与初始一致
    world.reset();
    {
        const auto snap = world.snapshot();
        const auto* l = reading(*snap, d, "level_imu");
        check(snap->sensors.size() == 4 && l && matches(*l, level, 1e-9), "reset 之后仍有读数");
    }

    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
