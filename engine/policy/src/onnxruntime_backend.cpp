// onnxruntime-cpu 后端（M4-10a，设计稿第 2、3 节）。
// 库文件来自可下载的组件（资源库目录下的 components/onnxruntime-cpu/），运行时 dlopen，构建只需要仓库中的 C API 头文件
// （third_party/onnxruntime，MIT）。进程中只打开一次库、只建一个 OrtEnv（ONNX Runtime 的要求），之后不再释放。
// 会话选项：单线程（算子内与算子间各 1）、顺序执行、确定性计算（SetDeterministicCompute）——同一机器上同样的输入得到逐位相同的输出。

#include <cstdint>
#include <cstring>
#include <map>
#include <mutex>

#include <dlfcn.h>
#include <onnxruntime_c_api.h>

#include "backends.hpp"
#include "terra/library/components.hpp"

namespace terra::policy {

namespace {

// 已打开的库（按路径）
struct Runtime {
    const OrtApi* api = nullptr;
    OrtEnv* env = nullptr;
    std::string version;
};

std::mutex& runtimeMutex() {
    static std::mutex m;
    return m;
}

// 打开库并取得 API；失败返回空并写 error
const Runtime* openRuntime(const std::string& library, std::string& error) {
    std::lock_guard<std::mutex> lock(runtimeMutex());
    static std::map<std::string, Runtime> opened;
    const auto it = opened.find(library);
    if (it != opened.end()) return &it->second;
    void* handle = dlopen(library.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (handle == nullptr) {
        const char* why = dlerror();
        error = "无法加载 ONNX Runtime：" + library + "（" + (why ? why : "未知原因") + "）";
        return nullptr;
    }
    void* symbol = dlsym(handle, "OrtGetApiBase");
    if (symbol == nullptr) {
        error = "库中没有 OrtGetApiBase，不是 ONNX Runtime：" + library;
        return nullptr;
    }
    using GetApiBase = const OrtApiBase* (*)();
    GetApiBase getApiBase = nullptr;
    std::memcpy(&getApiBase, &symbol, sizeof getApiBase);  // 对象指针 → 函数指针（避免 -Wpedantic 的转换警告）
    const OrtApiBase* base = getApiBase();
    Runtime rt;
    rt.version = base->GetVersionString();
    rt.api = base->GetApi(ORT_API_VERSION);
    if (rt.api == nullptr) {
        error = "ONNX Runtime " + rt.version + " 太旧：需要支持 API 版本 " + std::to_string(ORT_API_VERSION) + "（1.32 或更新）";
        return nullptr;
    }
    if (OrtStatus* s = rt.api->CreateEnv(ORT_LOGGING_LEVEL_WARNING, "terra", &rt.env)) {
        error = std::string{"ONNX Runtime 初始化失败："} + rt.api->GetErrorMessage(s);
        rt.api->ReleaseStatus(s);
        return nullptr;
    }
    return &opened.emplace(library, rt).first->second;
}

class OnnxModel final : public PolicyModel {
public:
    explicit OnnxModel(const Runtime& rt) : rt_(rt) {}
    ~OnnxModel() override {
        const OrtApi* api = rt_.api;
        if (memory_) api->ReleaseMemoryInfo(memory_);
        if (session_) api->ReleaseSession(session_);
    }
    OnnxModel(const OnnxModel&) = delete;
    OnnxModel& operator=(const OnnxModel&) = delete;

    // 打开会话、确定输入输出；失败返回假并写 error
    bool open(const PolicyDescription& policy, std::string& error) {
        const OrtApi* api = rt_.api;
        OrtSessionOptions* options = nullptr;
        bool ok = check(api->CreateSessionOptions(&options), error);
        ok = ok && check(api->SetIntraOpNumThreads(options, 1), error);
        ok = ok && check(api->SetInterOpNumThreads(options, 1), error);
        ok = ok && check(api->SetSessionExecutionMode(options, ORT_SEQUENTIAL), error);
        ok = ok && check(api->SetDeterministicCompute(options, true), error);
        ok = ok && check(api->AddSessionConfigEntry(options, "session.intra_op.allow_spinning", "0"), error);
        ok = ok && check(api->CreateSession(rt_.env, policy.model.c_str(), options, &session_), error);
        if (options) api->ReleaseSessionOptions(options);
        if (!ok) {
            error = "ONNX 模型无法加载：" + policy.model + "（" + error + "）";
            return false;
        }
        OrtAllocator* allocator = nullptr;
        if (!check(api->GetAllocatorWithDefaultOptions(&allocator), error)) return false;
        if (!pick(true, policy.inputName, allocator, inputName_, inputWidth_, error)) return false;
        if (!pick(false, policy.outputName, allocator, outputName_, outputWidth_, error)) return false;
        return check(api->CreateCpuMemoryInfo(OrtArenaAllocator, OrtMemTypeDefault, &memory_), error);
    }

    int inputWidth() const override { return inputWidth_; }
    int outputWidth() const override { return outputWidth_; }

    bool run(const std::vector<float>& input, std::vector<float>& output, std::string& error) override {
        if (static_cast<int>(input.size()) != inputWidth_) {
            error = "输入宽度 " + std::to_string(input.size()) + "，模型需要 " + std::to_string(inputWidth_);
            return false;
        }
        const OrtApi* api = rt_.api;
        buffer_ = input;  // CreateTensorWithDataAsOrtValue 需要可写的内存
        const std::int64_t shape[2] = {1, static_cast<std::int64_t>(inputWidth_)};
        OrtValue* in = nullptr;
        OrtValue* out = nullptr;
        bool ok = check(api->CreateTensorWithDataAsOrtValue(memory_, buffer_.data(), buffer_.size() * sizeof(float), shape, 2,
                                                            ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, &in),
                        error);
        const char* inNames[] = {inputName_.c_str()};
        const char* outNames[] = {outputName_.c_str()};
        ok = ok && check(api->Run(session_, nullptr, inNames, &in, 1, outNames, 1, &out), error);
        void* data = nullptr;
        ok = ok && check(api->GetTensorMutableData(out, &data), error);
        if (ok) {
            output.resize(static_cast<std::size_t>(outputWidth_));
            std::memcpy(output.data(), data, output.size() * sizeof(float));
        }
        if (out) api->ReleaseValue(out);
        if (in) api->ReleaseValue(in);
        return ok;
    }

    std::string describe() const override { return "ONNX Runtime " + rt_.version + "（CPU，单线程，确定性计算）"; }

private:
    bool check(OrtStatus* status, std::string& error) const {
        if (status == nullptr) return true;
        error = rt_.api->GetErrorMessage(status);
        rt_.api->ReleaseStatus(status);
        return false;
    }

    // 按名字（为空时取第一个）选输入或输出，要求 float 张量，宽度为除第一维（batch）外各维之积
    bool pick(bool input, const std::string& wanted, OrtAllocator* allocator, std::string& name, int& width, std::string& error) {
        const OrtApi* api = rt_.api;
        std::size_t count = 0;
        if (!check(input ? api->SessionGetInputCount(session_, &count) : api->SessionGetOutputCount(session_, &count), error)) return false;
        const char* what = input ? "输入" : "输出";
        if (input && count != 1) {
            error = "模型应只有 1 个输入，实际 " + std::to_string(count) + " 个";
            return false;
        }
        std::string names;
        for (std::size_t i = 0; i < count; ++i) {
            char* raw = nullptr;
            if (!check(input ? api->SessionGetInputName(session_, i, allocator, &raw) : api->SessionGetOutputName(session_, i, allocator, &raw), error)) {
                return false;
            }
            const std::string n = raw;
            std::string ignored;
            check(api->AllocatorFree(allocator, raw), ignored);  // 返回值带 warn_unused_result
            names += (names.empty() ? "" : "、") + n;
            if (!wanted.empty() && n != wanted) continue;
            OrtTypeInfo* type = nullptr;
            if (!check(input ? api->SessionGetInputTypeInfo(session_, i, &type) : api->SessionGetOutputTypeInfo(session_, i, &type), error)) return false;
            const OrtTensorTypeAndShapeInfo* tensor = nullptr;
            ONNXTensorElementDataType element = ONNX_TENSOR_ELEMENT_DATA_TYPE_UNDEFINED;
            std::size_t rank = 0;
            bool ok = check(api->CastTypeInfoToTensorInfo(type, &tensor), error) && tensor != nullptr;
            ok = ok && check(api->GetTensorElementType(tensor, &element), error);
            ok = ok && check(api->GetDimensionsCount(tensor, &rank), error);
            std::vector<std::int64_t> dims(rank);
            ok = ok && check(api->GetDimensions(tensor, dims.data(), rank), error);
            api->ReleaseTypeInfo(type);
            if (!ok) return false;
            if (element != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
                error = std::string{what} + "“" + n + "”不是 float32 张量";
                return false;
            }
            long long w = 1;
            for (std::size_t d = rank >= 2 ? 1 : 0; d < rank; ++d) {
                if (dims[d] <= 0) {
                    error = std::string{what} + "“" + n + "”除 batch 外有不定长的维度";
                    return false;
                }
                w *= dims[d];
            }
            name = n;
            width = static_cast<int>(w);
            return true;
        }
        error = "模型中没有" + std::string{what} + "“" + wanted + "”（有：" + names + "）";
        return false;
    }

    const Runtime& rt_;
    OrtSession* session_ = nullptr;
    OrtMemoryInfo* memory_ = nullptr;
    std::string inputName_, outputName_;
    int inputWidth_ = 0, outputWidth_ = 0;
    std::vector<float> buffer_;
};

class OnnxRuntimeBackend final : public PolicyBackend {
public:
    std::string id() const override { return kOnnxRuntimeComponent; }
    std::string displayName() const override { return "ONNX Runtime（CPU）"; }
    std::string component() const override { return kOnnxRuntimeComponent; }

    LoadModelResult load(const PolicyDescription& policy, const BackendContext& context) const override {
        namespace lib = terra::library;
        LoadModelResult r;
        const auto catalog = lib::readComponentCatalog(context.catalogPath);
        const lib::ComponentInfo* info = catalog.find(kOnnxRuntimeComponent);
        if (info == nullptr) {
            r.error = "组件清单中没有 " + std::string{kOnnxRuntimeComponent} + "：" + context.catalogPath;
            return r;
        }
        const auto status = lib::componentStatus(*info, context.componentsDirectory);
        if (status.state != lib::ComponentState::Installed) {
            r.missingComponent = info->id;
            const char* why = status.state == lib::ComponentState::Missing ? "没有安装" : status.state == lib::ComponentState::Outdated ? "版本不对" : "已损坏";
            r.error = "运行这个策略需要组件“" + info->nameChinese + " " + info->version + "”（" + why + (status.detail.empty() ? "" : "：" + status.detail) +
                      "）。请在查看器中下载安装，或运行 terra_headless --install-component " + info->id;
            return r;
        }
        const Runtime* rt = openRuntime(status.library, r.error);
        if (rt == nullptr) return r;
        auto model = std::make_unique<OnnxModel>(*rt);
        if (!model->open(policy, r.error)) return r;
        r.model = std::move(model);
        return r;
    }
};

}  // namespace

std::unique_ptr<PolicyBackend> makeOnnxRuntimeBackend() { return std::make_unique<OnnxRuntimeBackend>(); }

}  // namespace terra::policy
