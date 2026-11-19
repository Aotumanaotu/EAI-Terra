// 后端无关调用方测试（M0-07 ⑥）：terra_test_physics_consumer 只链接 terra::physics，
// 本程序只在创建世界时调用一次后端入口，其余全部经由该调用方完成。

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include "physics_consumer.hpp"
#include "terra/physics/mujoco/mujoco_backend.hpp"

#ifndef TERRA_TEST_ASSETS_DIR
#error "需要 TERRA_TEST_ASSETS_DIR"
#endif

namespace {

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

std::unique_ptr<terra::physics::PhysicsWorld> load(const std::string& file) {
    auto r = terra::physics::mujoco::loadWorld(terra::data::ModelSource{std::string{TERRA_TEST_ASSETS_DIR} + "/" + file});
    check(r.world != nullptr, "加载 " + file);
    return std::move(r.world);
}

}  // namespace

int main() {
    std::string error;

    // 无执行器的立方体：空控制，2000 步后静止在地面上（中心高度约 0.1 m）
    if (auto cube = load("falling_cube.xml")) {
        const auto summary = terra::test::runWithControl(*cube, {}, 2000, error);
        check(summary.has_value(), "立方体运行：" + error);
        if (summary) {
            check(summary->steps == 2000 && std::fabs(summary->time - 2.0) <= 1e-9, "立方体：2000 步、2 s");
            check(std::fabs(summary->entities.at(0).pose.position.z - 0.1) <= 0.005, "立方体：停在地面上");
            check(summary->allFinite && summary->maxQuaternionNormError <= 1e-6, "立方体：全程有限、姿态归一");
        }
    }

    // 铰链电机：按名称设置控制，1000 步后角度与解析解一致
    if (auto hinge = load("hinge_motor.xml")) {
        const auto commands = terra::test::commandsByName(hinge->description(), {{"drive", {0.5}}}, error);
        check(commands.has_value(), "按名称构造控制：" + error);
        if (commands) {
            const auto summary = terra::test::runWithControl(*hinge, *commands, 1000, error);
            check(summary.has_value(), "铰链运行：" + error);
            if (summary) {
                const double expected = 5.0 * 1e-6 * 1000 * 1001 / 2;  // a·dt²·k(k+1)/2，a = 0.5 / 0.1
                check(std::fabs(summary->joints.at(0).position[0] - expected) <= 1e-9, "铰链：角度与解析解一致");
            }
        }
        check(!terra::test::commandsByName(hinge->description(), {{"no_such", {0.0}}}, error).has_value(),
              "未知执行器名称被拒绝");
        // 非法控制经由调用方返回错误，且不推进仿真
        const std::uint64_t before = hinge->stepIndex();
        check(!terra::test::runWithControl(*hinge, {{terra::data::ActuatorId{1}, {2.0}}}, 10, error).has_value() &&
                  hinge->stepIndex() == before,
              "越界控制：调用方得到错误，未推进");
    }

    if (failures == 0) {
        std::printf("all passed\n");
    }
    return failures == 0 ? 0 : 1;
}
