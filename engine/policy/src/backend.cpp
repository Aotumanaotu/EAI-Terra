// 推理后端的注册表、linear 后端与按说明文件加载（M4-10a）。

#include "terra/policy/backend.hpp"

#include <cmath>
#include <fstream>
#include <mutex>
#include <utility>

#include <nlohmann/json.hpp>

#include "backends.hpp"
#include "terra/core/sha256.hpp"

namespace terra::policy {

namespace {

using Json = nlohmann::json;

// y = W x + b
class LinearModel final : public PolicyModel {
public:
    LinearModel(std::vector<std::vector<double>> weights, std::vector<double> bias) : weights_(std::move(weights)), bias_(std::move(bias)) {}
    int inputWidth() const override { return weights_.empty() ? 0 : static_cast<int>(weights_[0].size()); }
    int outputWidth() const override { return static_cast<int>(weights_.size()); }
    bool run(const std::vector<float>& input, std::vector<float>& output, std::string& error) override {
        if (static_cast<int>(input.size()) != inputWidth()) {
            error = "输入宽度 " + std::to_string(input.size()) + "，模型需要 " + std::to_string(inputWidth());
            return false;
        }
        output.assign(weights_.size(), 0.0f);
        for (std::size_t i = 0; i < weights_.size(); ++i) {
            double y = i < bias_.size() ? bias_[i] : 0.0;
            for (std::size_t k = 0; k < input.size(); ++k) y += weights_[i][k] * static_cast<double>(input[k]);
            output[i] = static_cast<float>(y);
        }
        return true;
    }
    std::string describe() const override { return "linear（" + std::to_string(outputWidth()) + " × " + std::to_string(inputWidth()) + "）"; }

private:
    std::vector<std::vector<double>> weights_;
    std::vector<double> bias_;
};

class LinearBackend final : public PolicyBackend {
public:
    std::string id() const override { return "linear"; }
    std::string displayName() const override { return "线性模型（测试用）"; }
    LoadModelResult load(const PolicyDescription& policy, const BackendContext&) const override {
        LoadModelResult r;
        std::ifstream in(policy.model, std::ios::binary);
        const Json j = in ? Json::parse(in, nullptr, false) : Json{};
        if (j.is_discarded() || !j.is_object() || !j.contains("format") || !j["format"].is_string() ||
            j["format"].get<std::string>() != "terra.linear-policy" || !j.contains("weights") || !j["weights"].is_array()) {
            r.error = "线性模型文件无法读取或格式不对（需要 format = \"terra.linear-policy\" 与 weights）：" + policy.model;
            return r;
        }
        std::vector<std::vector<double>> weights;
        for (const auto& row : j["weights"]) {
            std::vector<double> w;
            if (row.is_array()) {
                for (const auto& v : row) w.push_back(v.is_number() ? v.get<double>() : std::nan(""));
            }
            if (w.empty() || (!weights.empty() && w.size() != weights[0].size())) {
                r.error = "线性模型的 weights 应为等长的非空数值数组：" + policy.model;
                return r;
            }
            weights.push_back(std::move(w));
        }
        std::vector<double> bias;
        if (j.contains("bias") && j["bias"].is_array()) {
            for (const auto& v : j["bias"]) bias.push_back(v.is_number() ? v.get<double>() : 0.0);
        }
        r.model = std::make_unique<LinearModel>(std::move(weights), std::move(bias));
        return r;
    }
};

std::mutex& registryMutex() {
    static std::mutex m;
    return m;
}

std::vector<std::unique_ptr<PolicyBackend>>& registry() {
    static std::vector<std::unique_ptr<PolicyBackend>> backends = [] {
        std::vector<std::unique_ptr<PolicyBackend>> v;
        v.push_back(makeOnnxRuntimeBackend());
        v.push_back(std::make_unique<LinearBackend>());
        return v;
    }();
    return backends;
}

}  // namespace

void registerBackend(std::unique_ptr<PolicyBackend> backend) {
    if (!backend) return;
    std::lock_guard<std::mutex> lock(registryMutex());
    auto& r = registry();
    for (auto& b : r) {
        if (b->id() == backend->id()) {
            b = std::move(backend);
            return;
        }
    }
    r.push_back(std::move(backend));
}

const PolicyBackend* findBackend(const std::string& id) {
    std::lock_guard<std::mutex> lock(registryMutex());
    for (const auto& b : registry()) {
        if (b->id() == id) return b.get();
    }
    return nullptr;
}

std::vector<std::string> backendIds() {
    std::lock_guard<std::mutex> lock(registryMutex());
    std::vector<std::string> ids;
    for (const auto& b : registry()) ids.push_back(b->id());
    return ids;
}

LoadModelResult loadModel(const PolicyDescription& policy, const BackendContext& context) {
    LoadModelResult r;
    const PolicyBackend* backend = findBackend(policy.backend);
    if (backend == nullptr) {
        std::string list;
        for (const auto& id : backendIds()) list += (list.empty() ? "" : "、") + id;
        r.error = "没有推理后端“" + policy.backend + "”（可用：" + list + "）";
        return r;
    }
    const auto digest = terra::sha256File(policy.model);
    if (!digest) {
        r.error = "模型文件不存在或无法读取：" + policy.model;
        return r;
    }
    std::vector<std::string> warnings;
    if (!policy.modelSha256.empty() && *digest != policy.modelSha256) {
        warnings.push_back("模型文件与说明文件记录的 sha256 不一致（模型可能已重新导出）：" + policy.model);
    }
    r = backend->load(policy, context);
    r.warnings.insert(r.warnings.begin(), warnings.begin(), warnings.end());
    if (!r.model) return r;
    if (r.model->inputWidth() != policy.inputWidth() || r.model->outputWidth() != policy.actionWidth()) {
        r.error = "模型的输入 / 输出宽度为 " + std::to_string(r.model->inputWidth()) + " / " + std::to_string(r.model->outputWidth()) +
                  "，说明文件需要 " + std::to_string(policy.inputWidth()) + "（" + std::to_string(policy.historyLength) + " 帧 × " +
                  std::to_string(policy.frameWidth()) + "）/ " + std::to_string(policy.actionWidth());
        r.model.reset();
    }
    return r;
}

}  // namespace terra::policy
