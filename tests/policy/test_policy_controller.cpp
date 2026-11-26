// 策略控制器与自动配置（M4-10a，policy.controller）。夹具 assets/tests/policy/：policy_bench（没有重力的浮动基座，
// 初始绕 X 轴转 90°；两个力矩关节；IMU 50 Hz 带噪声）与线性模型（输出 0 = 最新一帧的重力方向 y，输出 1 = 最旧一帧的指令 vx + 0.25）。
//   - 降频（控制周期 0.01 s ÷ 步长 0.002 s = 5）、绑定的执行器；
//   - 观测：重力方向 (0, −1, 0)（取 IMU 的姿态）、关节位置减偏置、上一次动作、指令、相位（站立时为 0，行走时 i mod 4）；
//   - IMU 按 50 Hz 保持（第 5 步的读数与第 0 步相同，第 10 步换新）；噪声开时角速度不为 0，关时为真值 0；两次运行逐位相同；
//   - 历史帧（最旧的在前，第一次推理重复填满）；positionPd 的力矩（截断到 torqueLimit）与换算到控制量（÷ gear）；
//   - 世界重置后从头开始；指令截断到范围；不匹配的策略（没有的关节、只有位置执行器、控制周期不是步长的整数倍）给出原因；
//   - overrideCommands；
//   - 自动配置（bench_supermimic.onnx + task_config.json，与用户 Go2 策略相同的元数据写法）：关节对应、IMU、PD 增益、指令范围、
//     步长差别与多出关节的警告；保存后可以读回；模型实例不存在时报错；matchRobotJoints、policyJointNames（M4-10b）。
// 用法：test_policy_controller <临时目录>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>

#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/policy/autoconfig.hpp"
#include "terra/policy/backend.hpp"
#include "terra/policy/controller.hpp"
#include "terra/scene/scene.hpp"

#ifndef TERRA_ASSETS_DIR
#error "需要 TERRA_ASSETS_DIR"
#endif

namespace {

namespace fs = std::filesystem;
namespace pol = terra::policy;
namespace sc = terra::scene;
namespace tp = terra::physics;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

bool near(double a, double b, double tol = 1e-6) { return std::fabs(a - b) <= tol; }

const fs::path kDir = fs::path{TERRA_ASSETS_DIR} / "tests" / "policy";

sc::SceneInstance loadBench() {
    auto loaded = sc::loadScene((kDir / "policy_bench.terra-scene.json").string(), terra::physics::mujoco::composeWorld);
    for (const auto& issue : loaded.issues) std::fprintf(stderr, "场景：%s %s\n", issue.where.c_str(), issue.message.c_str());
    if (!loaded.scene) {
        std::fprintf(stderr, "夹具场景无法加载\n");
        std::exit(1);
    }
    return std::move(*loaded.scene);
}

pol::PolicyDescription benchPolicy() {
    const auto r = pol::readPolicyFile((kDir / "bench.terra-policy.json").string());
    if (!r.policy) {
        std::fprintf(stderr, "夹具说明文件无法读取\n");
        std::exit(1);
    }
    return *r.policy;
}

std::unique_ptr<pol::PolicyController> makeController(const pol::PolicyDescription& policy, const sc::SceneInstance& scene, bool noise, std::string& error) {
    pol::BackendContext context;
    auto loaded = pol::loadModel(policy, context);
    if (!loaded.model) {
        error = loaded.error;
        return nullptr;
    }
    return pol::PolicyController::create(policy, std::move(loaded.model), scene, noise, error);
}

// 推进一步：控制器给出命令，下发，再步进；返回这一步的命令
std::vector<tp::ActuatorCommand> step(pol::PolicyController& c, tp::PhysicsWorld& world) {
    auto commands = c.commands(*world.snapshot());
    if (!commands.empty() && world.setControl(commands)) {
        std::fprintf(stderr, "控制被拒绝\n");
        ++failures;
    }
    world.step();
    return commands;
}

double controlOf(const std::vector<tp::ActuatorCommand>& commands, std::uint32_t actuator) {
    for (const auto& c : commands) {
        if (c.actuator.value == actuator) return c.values.at(0);
    }
    return std::nan("");
}

}  // namespace

int main(int argc, char** argv) {
    const fs::path tmp = fs::absolute(argc > 1 ? argv[1] : "policy_controller_tmp").lexically_normal();
    std::error_code ec;
    fs::create_directories(tmp, ec);
    auto scene = loadBench();
    tp::PhysicsWorld& world = *scene.world;
    const auto& d = world.description();
    const auto policy = benchPolicy();
    std::string error;
    auto controller = makeController(policy, scene, true, error);
    check(controller != nullptr, "控制器：" + error);
    if (!controller) return 1;
    pol::PolicyController& c = *controller;
    const std::uint32_t m1 = d.findActuator("bench/m1")->id.value, m2 = d.findActuator("bench/m2")->id.value;
    check(c.decimation() == 5 && c.actuators().size() == 2 && c.actuators()[0].value == m1 && c.actuators()[1].value == m2, "降频 5，接管 m1、m2");

    // 第 0 步：第一次推理（站立指令）
    auto cmd = step(c, world);
    auto f0 = c.lastFrame();
    check(c.inferences() == 1 && f0.size() == 17, "第 0 步推理，每帧 17 维");
    check(near(f0[3], 0.0) && near(f0[4], -1.0) && near(f0[5], 0.0), "重力方向（IMU 姿态）为 (0, −1, 0)");
    check(f0[0] != 0.0f || f0[1] != 0.0f || f0[2] != 0.0f, "噪声开：角速度不为 0（真值为 0）");
    check(near(f0[6], -0.1) && near(f0[7], 0.1), "关节位置减偏置");
    check(f0[8] == 0.0f && f0[9] == 0.0f && f0[10] == 0.0f && f0[11] == 0.0f, "关节速度 0，上一次动作 0");
    check(f0[12] == 0.0f && f0[13] == 0.0f && f0[14] == 0.0f && f0[15] == 0.0f && f0[16] == 0.0f, "指令 0；站立时相位为 0");
    check(near(c.lastOutput()[0], -1.0) && near(c.lastOutput()[1], 0.25), "输出：(−1, 最旧一帧 vx 0 + 0.25)");
    check(near(c.targets()[0], 0.1 + 0.5 * -1.0) && near(c.targets()[1], -0.1 + 0.5 * 0.25), "目标 = offset + scale × 输出");
    check(near(controlOf(cmd, m1), -2.0) && near(controlOf(cmd, m2), 10.0 * 0.025 / 2.0), "m1：力矩 −4 截断到 −2；m2：力矩 0.25 ÷ gear 2");

    // 第 1～4 步：不推理，PD 每步按当前状态算
    c.setCommand({0.3, 0.0, 0.0});
    for (int i = 1; i <= 4; ++i) cmd = step(c, world);
    check(c.inferences() == 1, "第 1～4 步不推理");
    // 第 5 步：第二次推理
    cmd = step(c, world);
    auto f1 = c.lastFrame();
    check(c.inferences() == 2, "第 5 步推理");
    check(f1[0] == f0[0] && f1[1] == f0[1] && f1[2] == f0[2], "IMU 50 Hz：第 5 步用第 0 步的读数");
    check(near(f1[10], -1.0) && near(f1[11], 0.25), "上一次动作 = 第一次推理的原始输出");
    check(near(f1[12], 0.3) && near(f1[15], 1.0) && near(f1[16], 0.0), "指令 vx 0.3；相位 1/4：sin 1、cos 0");
    check(near(c.lastOutput()[1], 0.25), "历史：最旧一帧是第一次推理的帧（vx 0）");
    // 第 10 步：第三次推理
    c.setCommand({0.6, 0.0, 0.0});
    for (int i = 6; i <= 10; ++i) cmd = step(c, world);
    auto f2 = c.lastFrame();
    check(c.inferences() == 3 && near(c.lastOutput()[1], 0.3 + 0.25), "历史：最旧一帧是第二次推理的帧（vx 0.3）");
    check(f2[0] != f1[0] || f2[1] != f1[1] || f2[2] != f1[2], "IMU 50 Hz：第 10 步换新读数");
    check(near(f2[15], 0.0, 1e-6) && near(f2[16], -1.0), "相位 2/4：sin 0、cos −1");

    // 指令截断
    c.setCommand({5.0, -5.0, 9.0});
    check(c.command()[0] == 1.0 && c.command()[1] == -1.0 && c.command()[2] == 2.0, "指令截断到范围");
    c.setCommand({0.6, 0.0, 0.0});

    // 两次运行逐位相同；噪声关时为真值
    {
        auto scene2 = loadBench();
        auto c2 = makeController(policy, scene2, true, error);
        auto scene3 = loadBench();
        auto c3 = makeController(policy, scene3, false, error);
        bool same = c2 && c3;
        for (int i = 0; same && i <= 10; ++i) {
            if (i == 1) c2->setCommand({0.3, 0.0, 0.0});
            if (i == 6) c2->setCommand({0.6, 0.0, 0.0});
            const auto commands = step(*c2, *scene2.world);
            step(*c3, *scene3.world);
            if (i == 0) {
                same = c2->lastFrame() == f0;
                check(c3->lastFrame()[0] == 0.0f && c3->lastFrame()[1] == 0.0f && c3->lastFrame()[2] == 0.0f, "噪声关：角速度为真值 0");
            }
        }
        check(same && c2->lastFrame() == f2 && c2->lastOutput() == c.lastOutput(), "两次运行逐位相同");
    }

    // 世界重置：从头开始（历史重复填满，相位从 0 起）
    world.reset();
    cmd = step(c, world);
    check(c.inferences() == 4 && near(c.lastOutput()[1], 0.6 + 0.25), "重置后第一次推理：历史重复当前帧");
    check(near(c.lastFrame()[15], 0.0) && near(c.lastFrame()[16], 1.0) && c.lastFrame()[10] == 0.0f, "重置后：相位 0、上一次动作清零");

    // 不匹配的策略
    {
        auto bad = policy;
        bad.action.joints = {"bench/j1", "bench/j9"};
        check(!makeController(bad, scene, true, error) && error.find("bench/j9") != std::string::npos, "没有的关节：" + error);
        bad = policy;
        bad.action.joints = {"bench/j1", "bench/j3"};
        check(!makeController(bad, scene, true, error) && error.find("力矩") != std::string::npos, "positionPd 遇到位置执行器：" + error);
        bad = policy;
        bad.controlPeriod = 0.003;
        check(!makeController(bad, scene, true, error) && error.find("整数倍") != std::string::npos, "控制周期不是步长的整数倍：" + error);
    }

    // overrideCommands
    {
        std::vector<tp::ActuatorCommand> base{{terra::data::ActuatorId{1}, {1.0}}, {terra::data::ActuatorId{2}, {2.0}}};
        const auto merged = pol::overrideCommands(base, {{terra::data::ActuatorId{2}, {9.0}}});
        check(merged.size() == 2 && controlOf(merged, 1) == 1.0 && controlOf(merged, 2) == 9.0, "overrideCommands：同一执行器以策略为准");
    }

    // 自动配置
    {
        const std::string model = (kDir / "bench_supermimic.onnx").string();
        const std::string scenePath = (kDir / "policy_bench.terra-scene.json").string();
        const auto r = pol::autoconfigure({model, scenePath, "", ""}, d);
        for (const auto& e : r.errors) std::fprintf(stderr, "自动配置：%s\n", e.c_str());
        check(r.policy && r.format == "supermimic_moe_velocity_policy_v1" && !r.notes.empty(), "自动配置：识别格式并生成说明文件");
        if (r.policy) {
            const auto& p = *r.policy;
            check(p.action.joints == std::vector<std::string>{"bench/j1", "bench/j2"}, "关节名对应 j1 → bench/j1");
            check(p.action.kp == std::vector<double>{10.0, 20.0} && p.action.kd == std::vector<double>{1.0, 2.0} &&
                      p.action.torqueLimit == std::vector<double>{2.0, 3.0},
                  "PD 增益与力矩上限按关节组取自训练配置");
            check(p.action.offset == std::vector<double>{0.1, -0.1} && p.action.scale == std::vector<double>{0.5, 0.5}, "默认姿态与动作缩放");
            check(p.historyLength == 2 && p.frameWidth() == 17 && near(p.controlPeriod, 0.01) && p.inputName == "obs_history" && p.outputName == "actions",
                  "历史、每帧宽度、控制周期、输入输出名");
            check(p.observation.size() == 7 && p.observation[0].sensor == "bench/imu" && p.observation[1].sensor == "bench/imu" &&
                      p.observation[6].kind == pol::TermKind::Phase && near(p.observation[6].period, 0.04) && near(p.observation[6].standingThreshold, 0.1),
                  "观测：角速度与重力取 IMU；相位周期与阈值");
            check(p.commandRange[0][0] == -0.5 && p.commandRange[0][1] == 0.8, "指令范围取自训练配置");
            check(p.modelSha256.size() == 64 && p.scene == scenePath && p.source.at("specHash") == "e117647d1c05ee855f6b388cd5e686d052c5bd61a7e9512e128db93a32c26be4", "sha256、推荐场景、来源");
            bool warned = false;
            for (const auto& w : r.warnings) warned = warned || w.find("0.005") != std::string::npos;
            check(warned, "训练步长 0.005 s 与场景 0.002 s 不同：警告");
            bool extra = false;
            for (const auto& w : r.warnings) extra = extra || (w.find("bench/j3") != std::string::npos && w.find("1 个关节") != std::string::npos);
            check(extra, "机器人上还有不由策略控制的关节（bench/j3）：警告（M4-10b）");
            const auto m = pol::matchRobotJoints({"j1", "j2"}, d);
            check(m.complete() && m.instance == "bench" && m.instances == 1 && m.extra == std::vector<std::string>{"bench/j3"}, "matchRobotJoints");
            const auto partial = pol::matchRobotJoints({"j1", "j9"}, d);
            check(!partial.complete() && partial.matched == 1, "matchRobotJoints：部分对应");
            check(pol::policyJointNames(model) == std::vector<std::string>{"j1", "j2"} &&
                      pol::policyJointNames((kDir / "bench.terra-policy.json").string()) == std::vector<std::string>{"j1", "j2"},
                  "policyJointNames：ONNX 元数据与说明文件（去掉实例前缀）");
            const fs::path out = tmp / "autoconfig.terra-policy.json";
            check(!pol::savePolicy(p, out.string()), "保存");
            const auto back = pol::readPolicyFile(out.string());
            check(back.policy && back.policy->inputWidth() == 34 && back.policy->model == model, "读回");
            // 自动配置的结果可以直接用于控制器（后端换成线性模型以免依赖 ONNX Runtime）
            auto viaLinear = p;
            viaLinear.backend = "linear";
            viaLinear.model = (kDir / "linear_policy.json").string();
            viaLinear.modelSha256.clear();
            check(makeController(viaLinear, scene, true, error) != nullptr, "自动配置的说明文件可以创建控制器：" + error);
        }
        const auto wrong = pol::autoconfigure({model, scenePath, "nope", ""}, d);
        check(!wrong.policy && !wrong.errors.empty() && wrong.errors[0].find("nope") != std::string::npos, "模型实例不存在：报错");
    }

    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
