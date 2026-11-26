// 策略的导入（M4-10b，policy.import）。夹具 assets/tests/policy/ 与测试中用 miniz 生成的 .pt（zip）。
//   - 文件类型：ONNX（格式、spec_hash、迭代）、说明文件、TorchScript（有 code/）、训练检查点（data.pkl 中的 format、spec_hash、
//     迭代；BININT1 与 BININT2 两种整数、BINPUT 记忆操作）、不是 zip 的 .pt、不认识的扩展名；
//   - 检查点 → ONNX：spec_hash 与迭代都相同时找到；迭代不同（与用户的 model_10000.pt 相同的情况）时说明不能代用；没有时说明导出方法；
//   - 导入 ONNX：导入目录中有模型副本、训练配置副本与说明文件，可以读回；同名再导入得到 “-2”；原文件不变；
//   - 导入检查点：用对应的 ONNX，说明文件记下检查点；
//   - 自动配置无法完成（gemm_elu.onnx）：生成说明文件草稿（todo、力矩执行器的关节）；
//   - TorchScript 转换：Python 不存在时 missingTorch 与安装提示；policySlug。
// 用法：test_policy_import <临时目录>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include <miniz.h>
#include <nlohmann/json.hpp>

#include "terra/core/sha256.hpp"
#include "terra/physics/mujoco/mujoco_backend.hpp"
#include "terra/policy/import.hpp"
#include "terra/scene/scene.hpp"

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

const fs::path kDir = fs::path{TERRA_ASSETS_DIR} / "tests" / "policy";
const std::string kSpec = "e117647d1c05ee855f6b388cd5e686d052c5bd61a7e9512e128db93a32c26be4";  // bench_supermimic.onnx

std::string binunicode(const std::string& s) {
    std::string out = "X";
    const auto n = static_cast<std::uint32_t>(s.size());
    for (int i = 0; i < 4; ++i) out += static_cast<char>((n >> (8 * i)) & 0xff);
    return out + s;
}

// 与 torch.save 的 data.pkl 相同写法的片段：键、BINPUT、值
std::string checkpointPickle(const std::string& spec, int iteration) {
    std::string d("\x80\x02}q", 4);  // PROTO 2、EMPTY_DICT、BINPUT 0、MARK
    d += '\0';
    d += '(';
    d += binunicode("format") + "q\x01" + binunicode("supermimic_moe_velocity_v1") + "q\x02";
    d += binunicode("spec_hash") + "q\x03" + binunicode(spec) + "q\x04";
    d += binunicode("iteration") + "q\x05";
    if (iteration < 256) {
        d += 'K';
        d += static_cast<char>(iteration);
    } else {
        d += 'M';
        d += static_cast<char>(iteration & 0xff);
        d += static_cast<char>((iteration >> 8) & 0xff);
    }
    return d + "u.";
}

void writeZip(const fs::path& path, const std::vector<std::pair<std::string, std::string>>& entries) {
    mz_zip_archive zip{};
    mz_zip_writer_init_file_v2(&zip, path.string().c_str(), 0, 0);
    for (const auto& [name, data] : entries) {
        mz_zip_writer_add_mem(&zip, name.c_str(), data.data(), data.size(), static_cast<mz_uint>(MZ_NO_COMPRESSION));
    }
    mz_zip_writer_finalize_archive(&zip);
    mz_zip_writer_end(&zip);
}

std::string read(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

}  // namespace

int main(int argc, char** argv) {
    const fs::path tmp = fs::absolute(argc > 1 ? argv[1] : "policy_import_tmp").lexically_normal();
    std::error_code ec;
    fs::remove_all(tmp, ec);
    fs::create_directories(tmp / "run", ec);

    // ---- 文件类型 ----
    auto info = pol::inspectModelFile((kDir / "bench_supermimic.onnx").string());
    check(info.kind == pol::ModelFileKind::Onnx && info.format == "supermimic_moe_velocity_policy_v1" && info.specHash == kSpec && info.iteration == 1LL,
          "ONNX：格式、spec_hash（解码 JSON 字符串）、迭代");
    check(pol::inspectModelFile((kDir / "bench.terra-policy.json").string()).kind == pol::ModelFileKind::PolicyFile, "说明文件");
    const fs::path run = tmp / "run";
    fs::copy_file(kDir / "bench_supermimic.onnx", run / "policy.onnx", ec);
    fs::copy_file(kDir / "task_config.json", run / "task_config.json", ec);
    writeZip(run / "model_1.pt", {{"model_1/data.pkl", checkpointPickle(kSpec, 1)}, {"model_1/data/0", std::string(64, '\0')}});
    writeZip(run / "model_10000.pt", {{"model_10000/data.pkl", checkpointPickle(kSpec, 10000)}});
    writeZip(run / "other.pt", {{"other/data.pkl", checkpointPickle(std::string(64, 'a'), 1)}});
    writeZip(run / "scripted.pt", {{"scripted/code/__torch__.py", "def forward(self, x): ..."}, {"scripted/data.pkl", "\x80\x02}."}, {"scripted/constants.pkl", "\x80\x02)."}});
    std::ofstream(run / "notzip.pt") << "not a zip";
    std::ofstream(run / "weights.bin") << "x";
    info = pol::inspectModelFile((run / "model_1.pt").string());
    check(info.kind == pol::ModelFileKind::Checkpoint && info.format == "supermimic_moe_velocity_v1" && info.specHash == kSpec && info.iteration == 1LL,
          "检查点：format、spec_hash、迭代（BININT1）");
    info = pol::inspectModelFile((run / "model_10000.pt").string());
    check(info.kind == pol::ModelFileKind::Checkpoint && info.iteration == 10000LL, "检查点：迭代（BININT2）");
    check(pol::inspectModelFile((run / "scripted.pt").string()).kind == pol::ModelFileKind::TorchScript, "TorchScript（有 code/）");
    info = pol::inspectModelFile((run / "notzip.pt").string());
    check(info.kind == pol::ModelFileKind::Unknown && info.error.find("zip") != std::string::npos, "不是 zip 的 .pt：说明");
    check(pol::inspectModelFile((run / "weights.bin").string()).kind == pol::ModelFileKind::Unknown, "不认识的扩展名");

    // ---- 检查点 → ONNX ----
    auto link = pol::findOnnxForCheckpoint((run / "model_1.pt").string());
    check(link.onnx == (run / "policy.onnx").string() && !link.notes.empty(), "检查点：找到 spec_hash 与迭代都相同的 ONNX：" + link.error);
    link = pol::findOnnxForCheckpoint((run / "model_10000.pt").string());
    check(link.onnx.empty() && link.error.find("迭代 10000") != std::string::npos && link.error.find("不能代用") != std::string::npos,
          "迭代不同（同一网络结构）：不能代用：" + link.error);
    link = pol::findOnnxForCheckpoint((run / "other.pt").string());
    check(link.onnx.empty() && link.error.find("没有找到") != std::string::npos && link.error.find("导出") != std::string::npos, "没有对应的 ONNX：说明导出方法");

    // ---- 导入 ----
    auto loaded = terra::scene::loadScene((kDir / "policy_bench.terra-scene.json").string(), terra::physics::mujoco::composeWorld);
    if (!loaded.scene) return 1;
    const auto& robot = loaded.scene->world->description();
    pol::ImportOptions o;
    o.model = (run / "policy.onnx").string();
    o.scene = (kDir / "policy_bench.terra-scene.json").string();
    o.destination = (tmp / "library" / "policies").string();
    o.name = "bench walker";
    const std::string before = terra::sha256File(o.model).value_or("");
    auto r = pol::importPolicy(o, robot);
    check(r.policyFile.has_value() && r.error.empty(), "导入 ONNX：" + r.error);
    if (r.policyFile) {
        check(fs::path{*r.policyFile}.parent_path() == tmp / "library" / "policies" / "bench_walker", "导入目录：policies/<名字>");
        const auto back = pol::readPolicyFile(*r.policyFile);
        check(back.policy && back.policy->model == (tmp / "library" / "policies" / "bench_walker" / "policy.onnx").string() &&
                  back.policy->name == "bench walker" && back.policy->source.at("sourceKind") == "onnx" &&
                  back.policy->source.at("importedFrom") == o.model && back.policy->modelSha256 == before,
              "说明文件：模型副本、名字、来源");
        check(fs::is_regular_file(tmp / "library" / "policies" / "bench_walker" / "task_config.json"), "训练配置的副本");
    }
    check(terra::sha256File(o.model).value_or("") == before, "原文件不变");
    r = pol::importPolicy(o, robot);
    check(r.policyFile && fs::path{*r.policyFile}.parent_path().filename() == "bench_walker-2", "同名再导入：bench_walker-2");

    o.model = (run / "model_1.pt").string();
    o.name.clear();
    r = pol::importPolicy(o, robot);
    check(r.policyFile.has_value(), "导入检查点：" + r.error);
    if (r.policyFile) {
        const auto back = pol::readPolicyFile(*r.policyFile);
        check(back.policy && back.policy->source.at("checkpoint") == o.model && back.policy->source.at("sourceKind") == "checkpoint", "说明文件记下检查点");
    }
    o.model = (run / "model_10000.pt").string();
    r = pol::importPolicy(o, robot);
    check(!r.policyFile && r.error.find("不能代用") != std::string::npos, "迭代不同的检查点：拒绝");

    // 自动配置无法完成：草稿
    o.model = (kDir / "gemm_elu.onnx").string();
    o.name = "unknown format";
    r = pol::importPolicy(o, robot);
    check(!r.policyFile && !r.draftFile.empty() && fs::is_regular_file(r.draftFile) && r.error.find("草稿") != std::string::npos, "不认识的格式：草稿");
    if (!r.draftFile.empty()) {
        const auto j = nlohmann::json::parse(read(r.draftFile), nullptr, false);
        check(!j.is_discarded() && j.contains("todo") && j["todo"].size() >= 4 && j["model"]["input"].get<std::string>() == "obs" &&
                  j["action"]["joints"].get<std::vector<std::string>>() == std::vector<std::string>{"bench/j1", "bench/j2"},
              "草稿：todo、输入名、力矩执行器的关节（位置执行器 p3 不列）");
    }

    // TorchScript：没有 Python
    o.model = (run / "scripted.pt").string();
    o.python = (tmp / "no_python").string();
    o.converter = TERRA_SOURCE_DIR "/scripts/policy/torchscript_to_onnx.py";
    r = pol::importPolicy(o, robot);
    check(!r.policyFile && r.missingTorch && r.error.find("TERRA_PYTHON") != std::string::npos, "没有 Python：提示安装或指定 TERRA_PYTHON");

    check(pol::policySlug("go2 / run（迭代 1）") == "go2_run（迭代_1）" && pol::policySlug("///") == "policy", "policySlug");

    if (failures == 0) std::printf("all passed\n");
    return failures == 0 ? 0 : 1;
}
