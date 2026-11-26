// 策略说明文件、ONNX 信息与推理后端（M4-10a，policy.file）。夹具 assets/tests/policy/。
//   - 说明文件：读取（相对路径、每帧宽度、历史）、往返（换目录保存后相对路径改写、内容相同）、不认识的字段保留、各类错误；
//   - ONNX 信息：输入输出与符号维、算子、参数量、opset、元数据；不是 ONNX 的文件被拒绝；
//   - 后端：注册表（onnxruntime-cpu、linear）、linear 推理、sha256 不一致的警告、宽度不一致、未知后端；
//     没有安装 ONNX Runtime 组件时返回 missingComponent（界面据此提示下载）；
//   - 自动配置：不认识的格式与 .pt 给出说明。
// 用法：test_policy_file <临时目录>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "terra/policy/autoconfig.hpp"
#include "terra/policy/backend.hpp"
#include "terra/policy/policy.hpp"

#ifndef TERRA_ASSETS_DIR
#error "需要 TERRA_ASSETS_DIR"
#endif

namespace {

namespace fs = std::filesystem;
namespace pol = terra::policy;

int failures = 0;

void check(bool condition, const std::string& what) {
    if (!condition) {
        std::fprintf(stderr, "FAILED: %s\n", what.c_str());
        ++failures;
    }
}

std::string read(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

bool hasError(const pol::PolicyFileResult& r, const std::string& fragment) {
    for (const auto& e : r.errors) {
        if (e.find(fragment) != std::string::npos) return true;
    }
    return false;
}

// 在夹具文本中替换一段，再解析
pol::PolicyFileResult variant(const std::string& text, const std::string& from, const std::string& to, const std::string& path) {
    std::string t = text;
    const auto pos = t.find(from);
    if (pos == std::string::npos) {
        std::fprintf(stderr, "夹具中没有：%s\n", from.c_str());
        ++failures;
        return {};
    }
    t.replace(pos, from.size(), to);
    return pol::parsePolicy(t, path);
}

}  // namespace

int main(int argc, char** argv) {
    const fs::path tmp = fs::absolute(argc > 1 ? argv[1] : "policy_file_tmp").lexically_normal();
    std::error_code ec;
    fs::remove_all(tmp, ec);
    fs::create_directories(tmp, ec);
    const fs::path dir = fs::path{TERRA_ASSETS_DIR} / "tests" / "policy";
    const std::string benchPath = (dir / "bench.terra-policy.json").string();

    // ---- 说明文件 ----
    const auto bench = pol::readPolicyFile(benchPath);
    check(bench.policy && bench.errors.empty() && bench.warnings.empty(), "夹具说明文件可以读取，没有警告");
    if (!bench.policy) return 1;
    const pol::PolicyDescription& p = *bench.policy;
    check(p.model == (dir / "linear_policy.json").lexically_normal().string() && p.scene == (dir / "policy_bench.terra-scene.json").lexically_normal().string(),
          "模型与场景的相对路径按说明文件所在目录解析");
    check(p.backend == "linear" && p.historyLength == 2 && std::fabs(p.controlPeriod - 0.01) < 1e-12, "后端、历史、控制周期");
    check(p.frameWidth() == 17 && p.inputWidth() == 34 && p.actionWidth() == 2, "每帧 17 维（3 + 3 + 2 + 2 + 2 + 3 + 2），输入 34，动作 2");
    check(p.observation.size() == 7 && p.observation[2].joints == p.action.joints, "关节项省略 joints 时与动作相同");
    check(p.action.type == pol::ActionType::PositionPd && p.action.kp == std::vector<double>{10.0} && p.action.offset == std::vector<double>{0.1, -0.1},
          "动作：positionPd、kp 一个数、offset 逐项");

    // 往返：保存到另一个目录，相对路径改写，再读回内容相同
    const fs::path saved = tmp / "sub" / "copy.terra-policy.json";
    check(!pol::savePolicy(p, saved.string()), "保存");
    const auto back = pol::readPolicyFile(saved.string());
    check(back.policy && back.policy->model == p.model && back.policy->scene == p.scene && back.policy->frameWidth() == 17 &&
              back.policy->action.kp == p.action.kp && back.policy->observation.size() == p.observation.size() &&
              back.policy->observation[6].period == p.observation[6].period,
          "往返：内容相同");
    check(read(saved).find("\"file\": \"../../") != std::string::npos || read(saved).find("\"file\": \"/") != std::string::npos,
          "往返：模型路径写成相对路径（或不同根时的绝对路径）");

    // 不认识的字段：警告并保留
    const std::string text = read(benchPath);
    auto future = variant(text, "\"controlPeriod\"", "\"futureField\": {\"x\": 1}, \"controlPeriod\"", benchPath);
    check(future.policy && !future.warnings.empty() && future.policy->extra.find("futureField") != std::string::npos, "不认识的顶层字段：警告并保留");
    if (future.policy) {
        check(pol::policyToJson(*future.policy, benchPath).find("\"futureField\"") != std::string::npos, "保存时原样写回");
    }

    // 错误
    check(hasError(variant(text, "\"terra.policy\"", "\"terra.scene\"", benchPath), "format"), "format 不对");
    check(hasError(variant(text, "\"kp\": 10, ", "", benchPath), "kp"), "positionPd 缺 kp");
    check(hasError(variant(text, "\"term\": \"lastAction\"", "\"term\": \"lastActions\"", benchPath), "不是可用的观测项"), "未知观测项");
    check(hasError(variant(text, "\"offset\": [0.1, -0.1]}", "\"offset\": [0.1, -0.1, 0]}", benchPath), "offset"), "offset 长度不对");
    check(hasError(variant(text, "\"period\": 0.04, ", "", benchPath), "period"), "相位没有周期");
    check(hasError(variant(text, "{\"term\": \"baseAngularVelocity\", \"sensor\": \"bench/imu\"}", "{\"term\": \"baseAngularVelocity\"}", benchPath),
                   "sensor"),
          "角速度没有 sensor 或 entity");
    check(hasError(variant(text, "\"oldestFirst\"", "\"newestFirst\"", benchPath), "oldestFirst"), "历史顺序只支持 oldestFirst");
    check(hasError(variant(text, "\"positionPd\"", "\"impedance\"", benchPath), "impedance"), "未知动作类型");

    // ---- ONNX 信息 ----
    std::string error;
    const auto onnx = pol::readOnnxInfo((dir / "gemm_elu.onnx").string(), error);
    check(onnx.has_value(), "夹具 ONNX 可以读取：" + error);
    if (onnx) {
        check(onnx->inputs.size() == 1 && onnx->inputs[0].name == "obs" && onnx->inputs[0].dims == std::vector<long long>{-1, 4}, "输入 obs [batch, 4]（初始化张量不算输入）");
        check(onnx->outputs.size() == 1 && onnx->outputs[0].name == "act" && onnx->outputs[0].dims == std::vector<long long>{-1, 2}, "输出 act [batch, 2]");
        check(onnx->operators.size() == 2 && onnx->operators.at("Gemm") == 1 && onnx->operators.at("Elu") == 1, "算子 Gemm、Elu");
        check(onnx->parameters == 10 && onnx->opset == 18 && onnx->producer == "terra-fixture", "参数 10、opset 18、生成者");
        check(onnx->metadata.count("format") && onnx->metadata.at("format") == "terra_fixture_v1", "元数据");
    }
    check(!pol::readOnnxInfo((dir / "bench.terra-policy.json").string(), error) && !error.empty(), "不是 ONNX 的文件被拒绝：" + error);

    // ---- 后端 ----
    const auto ids = pol::backendIds();
    check(ids.size() >= 2 && pol::findBackend("onnxruntime-cpu") && pol::findBackend("linear") &&
              pol::findBackend("onnxruntime-cpu")->component() == "onnxruntime-cpu",
          "注册表：onnxruntime-cpu（需要组件）与 linear");
    pol::BackendContext context;
    context.componentsDirectory = (tmp / "no_components").string();
    context.catalogPath = TERRA_ASSETS_DIR "/components/components.json";
    auto loaded = pol::loadModel(p, context);
    check(loaded.model && loaded.error.empty() && loaded.warnings.empty(), "linear 模型加载：" + loaded.error);
    if (loaded.model) {
        std::vector<float> in(34, 0.0f), out;
        in[21] = -0.75f;
        in[12] = 0.5f;
        std::string why;
        check(loaded.model->run(in, out, why) && out.size() == 2 && out[0] == -0.75f && out[1] == 0.75f, "linear 推理：y = W x + b");
        check(!loaded.model->run(std::vector<float>(3, 0.0f), out, why) && !why.empty(), "输入宽度不对：报错");
    }
    pol::PolicyDescription wrongSha = p;
    wrongSha.modelSha256 = std::string(64, '0');
    loaded = pol::loadModel(wrongSha, context);
    check(loaded.model && loaded.warnings.size() == 1 && loaded.warnings[0].find("sha256") != std::string::npos, "模型 sha256 与记录不一致：警告");
    pol::PolicyDescription wrongWidth = p;
    wrongWidth.historyLength = 3;
    loaded = pol::loadModel(wrongWidth, context);
    check(!loaded.model && loaded.error.find("34") != std::string::npos, "模型宽度与说明文件不一致：报错");
    pol::PolicyDescription unknown = p;
    unknown.backend = "tensorrt";
    loaded = pol::loadModel(unknown, context);
    check(!loaded.model && loaded.error.find("tensorrt") != std::string::npos && loaded.error.find("linear") != std::string::npos, "未知后端：列出可用的后端");
    pol::PolicyDescription ort = p;
    ort.backend = "onnxruntime-cpu";
    ort.model = (dir / "gemm_elu.onnx").string();
    loaded = pol::loadModel(ort, context);
    check(!loaded.model && loaded.missingComponent == "onnxruntime-cpu" && loaded.error.find("--install-component") != std::string::npos,
          "没有 ONNX Runtime 组件：missingComponent 与安装方法");

    // ---- 自动配置的说明 ----
    terra::data::ModelDescription empty;
    auto ac = pol::autoconfigure({(dir / "gemm_elu.onnx").string(), "", ""}, empty);
    check(!ac.policy && !ac.errors.empty() && ac.errors[0].find("terra_fixture_v1") != std::string::npos, "不认识的格式：说明");
    ac = pol::autoconfigure({(tmp / "model_15000.pt").string(), "", ""}, empty);
    check(!ac.policy && !ac.errors.empty() && ac.errors[0].find("M4-10b") != std::string::npos, ".pt：说明在 M4-10b 中提供");
    check(!pol::policyFormats().empty() && pol::policyFormats()[0] == "supermimic_moe_velocity_policy_v1", "格式注册表");

    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
