// 策略的导入（M4-10b），说明见 import.hpp。miniz 只用于读 .pt（zip）中的 data.pkl 与目录。

#include "terra/policy/import.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>
#include <system_error>

#include <fcntl.h>
#include <miniz.h>
#include <nlohmann/json.hpp>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include "terra/core/sha256.hpp"

extern char** environ;

namespace terra::policy {

namespace {

namespace fs = std::filesystem;
namespace td = terra::data;
using Json = nlohmann::ordered_json;

bool endsWith(const std::string& s, const std::string& suffix) {
    return s.size() >= suffix.size() && s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// ONNX 元数据中的字符串（值是 JSON 编码时解码）
std::string metaText(const OnnxInfo& onnx, const char* key) {
    const auto it = onnx.metadata.find(key);
    if (it == onnx.metadata.end()) return {};
    const Json j = Json::parse(it->second, nullptr, false);
    return !j.is_discarded() && j.is_string() ? j.get<std::string>() : it->second;
}

std::optional<long long> metaInteger(const OnnxInfo& onnx, const char* key) {
    const auto it = onnx.metadata.find(key);
    if (it == onnx.metadata.end()) return std::nullopt;
    const Json j = Json::parse(it->second, nullptr, false);
    if (!j.is_discarded() && j.is_number_integer()) return j.get<long long>();
    return std::nullopt;
}

// ---- pickle（只认最常见的几个操作码，够读出字典中字符串与整数类型的值）----
// 键的位置：BINUNICODE（'X' + 4 字节长度）或 SHORT_BINUNICODE（0x8c + 1 字节长度）
std::size_t findKey(const std::string& data, const std::string& key) {
    std::string a = "X";
    const auto n = static_cast<std::uint32_t>(key.size());
    for (int i = 0; i < 4; ++i) a += static_cast<char>((n >> (8 * i)) & 0xff);
    a += key;
    std::size_t pos = data.find(a);
    if (pos != std::string::npos) return pos + a.size();
    std::string b = "\x8c";
    b += static_cast<char>(key.size() & 0xff);
    b += key;
    pos = data.find(b);
    return pos == std::string::npos ? std::string::npos : pos + b.size();
}

// 跳过记忆操作（BINPUT 'q'、LONG_BINPUT 'r'、MEMOIZE 0x94）
std::size_t skipMemo(const std::string& d, std::size_t p) {
    while (p < d.size()) {
        const auto op = static_cast<unsigned char>(d[p]);
        if (op == 'q') p += 2;
        else if (op == 'r') p += 5;
        else if (op == 0x94) p += 1;
        else break;
    }
    return p;
}

std::uint64_t littleEndian(const std::string& d, std::size_t p, int bytes) {
    std::uint64_t v = 0;
    for (int i = 0; i < bytes; ++i) v |= static_cast<std::uint64_t>(static_cast<unsigned char>(d[p + static_cast<std::size_t>(i)])) << (8 * i);
    return v;
}

std::optional<std::string> pickleString(const std::string& d, const std::string& key) {
    std::size_t p = findKey(d, key);
    if (p == std::string::npos) return std::nullopt;
    p = skipMemo(d, p);
    if (p + 5 <= d.size() && d[p] == 'X') {
        const auto n = static_cast<std::size_t>(littleEndian(d, p + 1, 4));
        if (p + 5 + n <= d.size()) return d.substr(p + 5, n);
    }
    if (p + 2 <= d.size() && static_cast<unsigned char>(d[p]) == 0x8c) {
        const auto n = static_cast<std::size_t>(static_cast<unsigned char>(d[p + 1]));
        if (p + 2 + n <= d.size()) return d.substr(p + 2, n);
    }
    return std::nullopt;
}

std::optional<long long> pickleInteger(const std::string& d, const std::string& key) {
    std::size_t p = findKey(d, key);
    if (p == std::string::npos) return std::nullopt;
    p = skipMemo(d, p);
    if (p >= d.size()) return std::nullopt;
    const auto op = static_cast<unsigned char>(d[p]);
    if (op == 'K' && p + 2 <= d.size()) return static_cast<long long>(littleEndian(d, p + 1, 1));
    if (op == 'M' && p + 3 <= d.size()) return static_cast<long long>(littleEndian(d, p + 1, 2));
    if (op == 'J' && p + 5 <= d.size()) return static_cast<long long>(static_cast<std::int32_t>(littleEndian(d, p + 1, 4)));
    return std::nullopt;
}

// zip 中的文件名列表与 data.pkl 的内容
struct ZipSummary {
    bool ok = false;
    bool code = false;      // 有 TorchScript 的 code/ 目录或 constants.pkl
    std::string dataPkl;
};

ZipSummary readZip(const std::string& path) {
    ZipSummary z;
    mz_zip_archive zip{};
    if (!mz_zip_reader_init_file(&zip, path.c_str(), 0)) return z;
    z.ok = true;
    const mz_uint n = mz_zip_reader_get_num_files(&zip);
    for (mz_uint i = 0; i < n; ++i) {
        char name[512];
        mz_zip_reader_get_filename(&zip, i, name, sizeof name);
        const std::string s = name;
        if (s.find("/code/") != std::string::npos || endsWith(s, "/constants.pkl")) z.code = true;
        if (z.dataPkl.empty() && (s == "data.pkl" || endsWith(s, "/data.pkl"))) {
            std::size_t size = 0;
            if (void* data = mz_zip_reader_extract_to_heap(&zip, i, &size, 0)) {
                z.dataPkl.assign(static_cast<const char*>(data), size);
                mz_free(data);
            }
        }
    }
    mz_zip_reader_end(&zip);
    return z;
}

bool copyFile(const fs::path& from, const fs::path& to, std::string& error) {
    std::error_code ec;
    fs::copy_file(from, to, fs::copy_options::overwrite_existing, ec);
    if (ec) error = "无法复制 " + from.string() + " → " + to.string() + "（" + ec.message() + "）";
    return !ec;
}

std::string readText(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

// 自动配置无法完成时的说明文件草稿：已知的项填好，未知的列在 todo 中
std::string draftJson(const std::string& name, const std::string& model, const std::string& scene, const OnnxInfo* onnx,
                      const td::ModelDescription& robot, const AutoconfigResult& config) {
    Json j;
    j["format"] = "terra.policy";
    j["version"] = 1;
    j["name"] = name;
    j["scene"] = scene;
    Json m;
    m["file"] = model;
    m["backend"] = "onnxruntime-cpu";
    long long inWidth = 0, outWidth = 0;
    if (onnx && !onnx->inputs.empty()) {
        m["input"] = onnx->inputs[0].name;
        inWidth = onnx->inputs[0].dims.empty() ? 0 : onnx->inputs[0].dims.back();
    }
    if (onnx && !onnx->outputs.empty()) {
        m["output"] = onnx->outputs[0].name;
        outWidth = onnx->outputs[0].dims.empty() ? 0 : onnx->outputs[0].dims.back();
    }
    j["model"] = std::move(m);
    j["controlPeriod"] = 0.02;
    j["history"] = {{"length", 1}};
    // 力矩执行器作用的关节（场景中的顺序）
    std::vector<std::string> joints;
    std::string imu;
    for (const auto& a : robot.actuators) {
        if (a.kind != td::ActuatorKind::Motor || a.transmission != td::ActuatorTransmission::Joint) continue;
        if (const auto* jd = robot.joint(a.joint)) joints.push_back(jd->name);
    }
    for (const auto& s : robot.sensors) {
        if (s.kind == td::SensorKind::Imu) {
            imu = s.name;
            break;
        }
    }
    Json obs = Json::array();
    obs.push_back(imu.empty() ? Json{{"term", "baseAngularVelocity"}, {"entity", "TODO"}} : Json{{"term", "baseAngularVelocity"}, {"sensor", imu}});
    obs.push_back(imu.empty() ? Json{{"term", "projectedGravity"}, {"entity", "TODO"}} : Json{{"term", "projectedGravity"}, {"sensor", imu}});
    obs.push_back({{"term", "command"}});
    obs.push_back({{"term", "jointPositions"}, {"offset", 0.0}});
    obs.push_back({{"term", "jointVelocities"}});
    obs.push_back({{"term", "lastAction"}});
    j["observation"] = std::move(obs);
    j["action"] = {{"type", "positionPd"}, {"joints", joints}, {"scale", 0.25}, {"offset", 0.0}, {"kp", 0.0}, {"kd", 0.0}};
    Json todo = Json::array();
    todo.push_back("模型输入 " + std::to_string(inWidth) + " 维、输出 " + std::to_string(outWidth) + " 维：按训练代码核对观测项的顺序、缩放与历史帧数，使每帧宽度 × 历史 = 输入宽度");
    todo.push_back("action.joints：按训练时动作的顺序排列（这里先列出场景中力矩执行器的关节，共 " + std::to_string(joints.size()) + " 个）；长度应等于输出宽度");
    todo.push_back("action.offset（默认姿态）、scale（动作缩放）、kp、kd、torqueLimit：取自训练或部署配置");
    todo.push_back("controlPeriod（控制周期，秒）");
    for (const auto& e : config.errors) todo.push_back("自动配置：" + e);
    todo.push_back("填好后把文件改名为 *.terra-policy.json（去掉 -draft），在查看器“策略 → 运行策略…”中打开");
    j["todo"] = std::move(todo);
    return j.dump(2) + "\n";
}

}  // namespace

const char* modelFileKindName(ModelFileKind kind) {
    switch (kind) {
        case ModelFileKind::Onnx: return "onnx";
        case ModelFileKind::TorchScript: return "torchscript";
        case ModelFileKind::Checkpoint: return "checkpoint";
        case ModelFileKind::PolicyFile: return "policy";
        case ModelFileKind::Unknown: return "unknown";
    }
    return "unknown";
}

ModelFileInfo inspectModelFile(const std::string& path) {
    ModelFileInfo info;
    std::error_code ec;
    if (!fs::is_regular_file(path, ec)) {
        info.error = "文件不存在：" + path;
        return info;
    }
    if (endsWith(path, kPolicySuffix)) {
        info.kind = ModelFileKind::PolicyFile;
        return info;
    }
    if (endsWith(path, ".onnx")) {
        const auto onnx = readOnnxInfo(path, info.error);
        if (!onnx) return info;
        info.kind = ModelFileKind::Onnx;
        info.format = metaText(*onnx, "format");
        info.specHash = metaText(*onnx, "spec_hash");
        info.iteration = metaInteger(*onnx, "iteration");
        return info;
    }
    if (endsWith(path, ".pt") || endsWith(path, ".pth")) {
        const auto z = readZip(path);
        if (!z.ok) {
            info.error = "不是 PyTorch 的 zip 格式（torch.save 1.6 起的默认格式）：" + path + "。旧格式请在 PyTorch 中重新保存";
            return info;
        }
        if (z.code) {
            info.kind = ModelFileKind::TorchScript;
            return info;
        }
        if (z.dataPkl.empty()) {
            info.error = "zip 中没有 data.pkl，不是 PyTorch 的文件：" + path;
            return info;
        }
        info.kind = ModelFileKind::Checkpoint;
        info.format = pickleString(z.dataPkl, "format").value_or("");
        info.specHash = pickleString(z.dataPkl, "spec_hash").value_or("");
        info.iteration = pickleInteger(z.dataPkl, "iteration");
        return info;
    }
    info.error = "不认识的文件类型（支持 .onnx、.pt、.terra-policy.json）：" + path;
    return info;
}

CheckpointLink findOnnxForCheckpoint(const std::string& checkpoint) {
    CheckpointLink link;
    const auto info = inspectModelFile(checkpoint);
    const std::string it = info.iteration ? std::to_string(*info.iteration) : std::string{"未知"};
    const std::string howTo = "训练检查点只含参数与训练状态，需要训练代码才能运行。请用训练项目的导出命令导出这次迭代的 ONNX，"
                              "放在检查点旁边后重新导入，或直接导入 ONNX";
    if (info.kind != ModelFileKind::Checkpoint) {
        link.error = info.error.empty() ? "不是训练检查点：" + checkpoint : info.error;
        return link;
    }
    if (info.specHash.size() < 16) {
        link.error = "检查点中没有 spec_hash，无法确定对应的 ONNX（格式 " + (info.format.empty() ? std::string{"未知"} : info.format) + "）。" + howTo;
        return link;
    }
    // 候选：检查点所在目录，以及上两级目录下（不超过 3 层）的 .onnx
    std::vector<fs::path> candidates;
    std::set<std::string> seen;
    const fs::path dir = fs::absolute(checkpoint).lexically_normal().parent_path();
    fs::path top = dir;
    for (int i = 0; i < 2 && top.has_parent_path() && top.parent_path() != top; ++i) top = top.parent_path();
    std::error_code ec;
    std::size_t visited = 0;
    for (auto walk = fs::recursive_directory_iterator(top, fs::directory_options::skip_permission_denied, ec);
         !ec && walk != fs::recursive_directory_iterator() && ++visited < 20000; walk.increment(ec)) {
        if (walk.depth() > 3) {
            walk.disable_recursion_pending();
            continue;
        }
        if (walk->is_regular_file(ec) && walk->path().extension() == ".onnx" && seen.insert(walk->path().string()).second) {
            candidates.push_back(walk->path());
            if (candidates.size() > 200) break;
        }
    }
    // 检查点所在目录中的优先，其余按路径
    std::stable_sort(candidates.begin(), candidates.end(), [&](const fs::path& a, const fs::path& b) {
        const bool ia = a.parent_path() == dir, ib = b.parent_path() == dir;
        return ia != ib ? ia : a.string() < b.string();
    });
    std::string sameSpec;
    for (const auto& c : candidates) {
        const auto o = inspectModelFile(c.string());
        if (o.kind != ModelFileKind::Onnx || o.specHash != info.specHash) continue;
        if (!info.iteration || !o.iteration || *o.iteration == *info.iteration) {
            link.onnx = c.string();
            link.notes.push_back("训练检查点（格式 " + info.format + "，迭代 " + it + "）对应 " + c.string() + "：spec_hash 相同" +
                                 (info.iteration && o.iteration ? "，迭代次数相同" : "（缺少迭代次数，只按 spec_hash 对应）"));
            if (!info.iteration || !o.iteration) link.notes.push_back("警告：无法核对迭代次数，ONNX 可能来自另一次迭代");
            return link;
        }
        sameSpec += (sameSpec.empty() ? "" : "、") + c.filename().string() + "（迭代 " + std::to_string(*o.iteration) + "）";
    }
    if (!sameSpec.empty()) {
        link.error = "检查点（迭代 " + it + "）与 " + sameSpec + " 的网络结构相同（spec_hash 一致），但参数来自不同的迭代，不能代用。" + howTo;
    } else {
        link.error = "在 " + top.string() + " 下没有找到 spec_hash 为 " + info.specHash.substr(0, 12) + "… 的 ONNX。" + howTo;
    }
    return link;
}

std::string defaultPython() {
    const char* env = std::getenv("TERRA_PYTHON");
    return env && *env ? env : "python3";
}

ConvertResult convertTorchScript(const std::string& pt, const std::string& onnx, const std::string& python, const std::string& script) {
    ConvertResult r;
    std::error_code ec;
    if (!fs::is_regular_file(script, ec)) {
        r.error = "找不到转换脚本：" + script;
        return r;
    }
    fs::create_directories(fs::path{onnx}.parent_path(), ec);
    const std::string logPath = onnx + ".log";
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_addopen(&actions, 1, logPath.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    posix_spawn_file_actions_adddup2(&actions, 1, 2);
    std::string py = python.empty() ? defaultPython() : python, s = script, in = pt, out = onnx;
    char* argv[] = {py.data(), s.data(), in.data(), out.data(), nullptr};
    pid_t pid = 0;
    const int spawned = posix_spawnp(&pid, py.c_str(), &actions, nullptr, argv, environ);
    posix_spawn_file_actions_destroy(&actions);
    if (spawned != 0) {
        r.missingTorch = true;
        r.error = "无法运行 Python（" + py + "）：请安装 Python 3 与 PyTorch，或用环境变量 TERRA_PYTHON 指定带 PyTorch 的 Python";
        return r;
    }
    int status = 0;
    waitpid(pid, &status, 0);
    r.log = readText(logPath);
    fs::remove(logPath, ec);
    const int code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    if (code == 3) {
        r.missingTorch = true;
        r.error = py + " 中没有 PyTorch：pip install torch --index-url https://download.pytorch.org/whl/cpu（CPU 版，约 200 MB），"
                       "或用环境变量 TERRA_PYTHON 指定带 PyTorch 的 Python";
        return r;
    }
    if (code != 0 || !fs::is_regular_file(onnx, ec)) {
        r.error = "TorchScript 转 ONNX 失败（退出码 " + std::to_string(code) + "）";
        return r;
    }
    r.ok = true;
    return r;
}

std::string policySlug(const std::string& name) {
    std::string out;
    for (const char c : name) {
        const auto u = static_cast<unsigned char>(c);
        const char kept = (u >= 0x80 || std::isalnum(u) || c == '-' || c == '_') ? c : '_';
        if (kept == '_' && (out.empty() || out.back() == '_')) continue;  // 连续的“_”只留一个，开头不留
        out += kept;
    }
    while (!out.empty() && out.back() == '_') out.pop_back();
    return out.empty() ? "policy" : out;
}

ImportResult importPolicy(const ImportOptions& o, const td::ModelDescription& robot) {
    ImportResult r;
    std::error_code ec;
    const fs::path original = fs::absolute(o.model, ec).lexically_normal();
    r.source = inspectModelFile(original.string());
    fs::path onnx;
    fs::path temp;  // 转换得到的临时文件
    switch (r.source.kind) {
        case ModelFileKind::Unknown: r.error = r.source.error; return r;
        case ModelFileKind::PolicyFile:
            r.error = "这是说明文件，不需要导入：在“策略 → 运行策略…”中直接打开";
            return r;
        case ModelFileKind::Onnx: onnx = original; break;
        case ModelFileKind::Checkpoint: {
            const auto link = findOnnxForCheckpoint(original.string());
            for (const auto& n : link.notes) r.log.push_back(n);
            if (link.onnx.empty()) {
                r.error = link.error;
                return r;
            }
            onnx = link.onnx;
            break;
        }
        case ModelFileKind::TorchScript: {
            fs::create_directories(o.destination, ec);
            temp = fs::path{o.destination} / ("." + original.stem().string() + ".converting.onnx");
            r.log.push_back("TorchScript：用 " + (o.python.empty() ? defaultPython() : o.python) + " 与 PyTorch 转为 ONNX");
            const auto c = convertTorchScript(original.string(), temp.string(), o.python, o.converter);
            if (!c.log.empty()) r.log.push_back(c.log);
            if (!c.ok) {
                r.missingTorch = c.missingTorch;
                r.error = c.error;
                fs::remove(temp, ec);
                return r;
            }
            onnx = temp;
            break;
        }
    }

    AutoconfigInput input;
    input.model = onnx.string();
    input.scene = o.scene;
    input.instance = o.instance;
    input.configDirectory = (r.source.kind == ModelFileKind::Onnx ? onnx : original).parent_path().string();
    r.config = autoconfigure(input, robot);
    for (const auto& n : r.config.notes) r.log.push_back("自动配置：" + n);
    for (const auto& w : r.config.warnings) r.log.push_back("警告：" + w);

    std::string name = o.name;
    if (name.empty() && r.config.policy) name = r.config.policy->name;
    if (name.empty()) name = original.stem().string();
    const std::string slug = policySlug(name);
    fs::path dir = fs::path{o.destination} / slug;
    for (int n = 2; fs::exists(dir, ec) && n < 1000; ++n) dir = fs::path{o.destination} / (slug + "-" + std::to_string(n));
    fs::create_directories(dir, ec);
    if (ec) {
        r.error = "无法创建导入目录：" + dir.string();
        fs::remove(temp, ec);
        return r;
    }
    r.directory = dir.string();
    const fs::path modelCopy = dir / (r.source.kind == ModelFileKind::TorchScript ? original.stem().string() + ".onnx" : onnx.filename().string());
    std::string error;
    const bool copied = temp.empty() ? copyFile(onnx, modelCopy, error) : (fs::rename(temp, modelCopy, ec), !ec);
    if (!copied) {
        r.error = error.empty() ? "无法写入：" + modelCopy.string() : error;
        fs::remove(temp, ec);
        return r;
    }
    r.model = modelCopy.string();
    r.log.push_back("模型：" + modelCopy.string());

    if (!r.config.policy) {
        std::string why;
        const auto info = readOnnxInfo(modelCopy.string(), why);
        r.draftFile = (dir / (slug + kPolicyDraftSuffix)).string();
        std::ofstream(r.draftFile, std::ios::binary) << draftJson(name, modelCopy.string(), o.scene, info ? &*info : nullptr, robot, r.config);
        r.error = "自动配置无法完成（" + (r.config.errors.empty() ? std::string{"未知原因"} : r.config.errors.front()) +
                  "）。已生成说明文件草稿，填写其中 todo 列出的项：" + r.draftFile;
        return r;
    }
    PolicyDescription p = *r.config.policy;
    p.name = name;
    p.model = modelCopy.string();
    p.modelSha256 = terra::sha256File(p.model).value_or("");
    p.source["importedFrom"] = original.string();
    p.source["sourceKind"] = modelFileKindName(r.source.kind);
    if (r.source.kind == ModelFileKind::Checkpoint) p.source["checkpoint"] = original.string();
    const auto trainingConfig = p.source.find("trainingConfig");
    if (trainingConfig != p.source.end() && fs::is_regular_file(trainingConfig->second, ec)) {
        std::string copyError;
        if (copyFile(trainingConfig->second, dir / "task_config.json", copyError)) p.source["trainingConfigCopy"] = (dir / "task_config.json").string();
    }
    const fs::path file = dir / (slug + kPolicySuffix);
    if (const auto failed = savePolicy(p, file.string())) {
        r.error = *failed;
        return r;
    }
    r.policyFile = file.string();
    r.log.push_back("说明文件：" + file.string());
    return r;
}

}  // namespace terra::policy
