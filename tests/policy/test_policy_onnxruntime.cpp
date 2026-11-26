// onnxruntime-cpu 后端（M4-10a，policy.onnxruntime）。需要已安装的 ONNX Runtime 组件（验证脚本先用
// terra_headless --install-component onnxruntime-cpu 安装到构建目录中的资源库）；没有组件时跳过（返回 77）。
//   - 夹具 gemm_elu.onnx：输出与手算一致（Gemm + Elu）；同样的输入两次逐位相同；输入输出名不对时列出模型中的名字；
//   - 自动配置 bench_supermimic.onnx 后用 ONNX Runtime 运行控制器，与同一网络的线性模型逐步比较（输出、目标、命令）。
// 用法：test_policy_onnxruntime <组件目录>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>

#include "terra/library/components.hpp"
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

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

double elu(double x) { return x > 0 ? x : std::exp(x) - 1.0; }

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "用法：test_policy_onnxruntime <组件目录>\n");
        return 2;
    }
    const fs::path dir = fs::path{TERRA_ASSETS_DIR} / "tests" / "policy";
    pol::BackendContext context;
    context.componentsDirectory = argv[1];
    context.catalogPath = TERRA_ASSETS_DIR "/components/components.json";
    const auto catalog = terra::library::readComponentCatalog(context.catalogPath);
    const auto* info = catalog.find("onnxruntime-cpu");
    if (!info || terra::library::componentStatus(*info, context.componentsDirectory).state != terra::library::ComponentState::Installed) {
        std::printf("SKIP: 没有安装 ONNX Runtime 组件（%s）；先运行 terra_headless --install-component onnxruntime-cpu --library <资源库>\n", argv[1]);
        return 77;
    }

    // 夹具 gemm_elu.onnx（输入 4，输出 2）
    pol::PolicyDescription p;
    p.model = (dir / "gemm_elu.onnx").string();
    p.backend = "onnxruntime-cpu";
    p.action.joints = {"a", "b"};
    pol::ObservationTerm t;
    t.kind = pol::TermKind::JointPositions;
    t.joints = {"1", "2", "3", "4"};
    p.observation = {t};
    auto loaded = pol::loadModel(p, context);
    check(loaded.model != nullptr, "加载：" + loaded.error);
    if (!loaded.model) return 1;
    std::printf("后端：%s\n", loaded.model->describe().c_str());
    check(loaded.model->describe().find("1.32") != std::string::npos, "版本 1.32");
    const double W[2][4] = {{0.5, -1.0, 0.25, 2.0}, {1.0, 1.0, -1.0, 0.5}};
    const double B[2] = {0.1, -0.2};
    for (const std::vector<float>& x : {std::vector<float>{1, 2, 3, 4}, std::vector<float>{1, -2, 3, -4}}) {
        std::vector<float> y, y2;
        std::string why;
        const bool ok = loaded.model->run(x, y, why) && loaded.model->run(x, y2, why);
        check(ok && y.size() == 2, "推理：" + why);
        if (!ok || y.size() != 2) continue;
        for (int i = 0; i < 2; ++i) {
            double h = B[i];
            for (int k = 0; k < 4; ++k) h += W[i][k] * static_cast<double>(x[static_cast<std::size_t>(k)]);
            const double expected = elu(h);
            check(std::fabs(static_cast<double>(y[static_cast<std::size_t>(i)]) - expected) < 1e-5,
                  "输出 " + std::to_string(i) + " = " + std::to_string(y[static_cast<std::size_t>(i)]) + "，手算 " + std::to_string(expected));
        }
        check(y == y2, "同样的输入两次逐位相同");
    }
    auto wrongName = p;
    wrongName.inputName = "observation";
    loaded = pol::loadModel(wrongName, context);
    check(!loaded.model && loaded.error.find("obs") != std::string::npos, "输入名不对：列出模型中的名字（" + loaded.error + "）");

    // 自动配置 + ONNX Runtime 的控制器，与线性模型逐步比较
    auto load = [] {
        auto r = sc::loadScene((fs::path{TERRA_ASSETS_DIR} / "tests" / "policy" / "policy_bench.terra-scene.json").string(),
                               terra::physics::mujoco::composeWorld);
        return std::move(*r.scene);
    };
    auto sceneA = load();
    auto sceneB = load();
    const auto ac = pol::autoconfigure({(dir / "bench_supermimic.onnx").string(), "", ""}, sceneA.world->description());
    check(ac.policy.has_value(), "自动配置");
    if (!ac.policy) return 1;
    std::string error;
    auto ort = pol::loadModel(*ac.policy, context);
    auto controllerA = ort.model ? pol::PolicyController::create(*ac.policy, std::move(ort.model), sceneA, true, error) : nullptr;
    check(controllerA != nullptr, "ONNX Runtime 控制器：" + ort.error + error);
    auto linearPolicy = *ac.policy;
    linearPolicy.backend = "linear";
    linearPolicy.model = (dir / "linear_policy.json").string();
    linearPolicy.modelSha256.clear();
    auto lin = pol::loadModel(linearPolicy, context);
    auto controllerB = lin.model ? pol::PolicyController::create(linearPolicy, std::move(lin.model), sceneB, true, error) : nullptr;
    if (!controllerA || !controllerB) return 1;
    // 线性夹具的输出 1 = 最旧一帧的 vx + 0.25，ONNX 夹具的输出 1 = 0：只比较输出 0（最新一帧的重力方向 y）
    double maxDiff = 0.0;
    for (int i = 0; i < 50; ++i) {
        const auto a = controllerA->commands(*sceneA.world->snapshot());
        controllerB->commands(*sceneB.world->snapshot());
        maxDiff = std::max(maxDiff, std::fabs(static_cast<double>(controllerA->lastOutput()[0] - controllerB->lastOutput()[0])));
        // 两边下发同样的命令（ONNX 一边的），世界保持一致
        sceneA.world->setControl(a);
        sceneB.world->setControl(a);
        sceneA.world->step();
        sceneB.world->step();
    }
    check(controllerA->inferences() == 10 && maxDiff < 1e-6, "ONNX Runtime 与线性模型的输出 0 一致（最大差 " + std::to_string(maxDiff) + "）");
    std::printf("推理耗时（最后一次）：%.1f µs\n", controllerA->lastInferenceSeconds() * 1e6);

    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
