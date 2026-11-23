#pragma once

// terra_headless 命令行解析、校验与 JSON Lines 输出格式（M0-03 起）。
// 只依赖标准库与 Terra 数据契约，不涉及物理后端，便于单元测试。

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "terra/data/geometry.hpp"
#include "terra/data/ids.hpp"
#include "terra/data/model_description.hpp"
#include "terra/data/state.hpp"

namespace terra::headless {

// 退出码约定。stdout 只输出机器可读的 JSON Lines，诊断信息写 stderr。
enum ExitCode : int {
    kExitOk = 0,
    kExitInternal = 1,     // 未预期的内部错误
    kExitUsage = 2,        // 命令行参数错误
    kExitUnavailable = 3,  // 物理后端不可用，或所需功能尚未接入
    kExitModel = 4,        // 模型文件不可用或加载失败
    kExitSimulation = 5,   // 仿真过程中出现非有限值、意外重置等严重错误（M0-05 起使用）
};

enum class Command { Help, Version, Run };

struct RunOptions {
    std::string modelPath;
    std::string scenePath;                  // M2-08：场景文件（与 modelPath 二选一）
    std::optional<std::uint64_t> steps;     // 与 duration 二选一
    std::optional<double> durationSeconds;  // 仿真时长（秒）；换算为步数需要模型步长，在 M0-04 后完成
    std::uint64_t sampleEvery = 1;          // 每 N 步输出一次状态
    bool dryRun = false;                    // 只校验参数与模型文件，不运行仿真
};

struct ParseResult {
    Command command = Command::Run;
    RunOptions options;
    std::string error;  // 非空表示用法错误

    bool ok() const noexcept { return error.empty(); }
};

// 解析 argv[1..argc)。支持 "--opt value" 与 "--opt=value" 两种写法。
// --help / --version 出现在任意位置时优先处理，其余参数不再校验。
ParseResult parseArguments(int argc, const char* const* argv);

std::string usageText();

// 加载模型后才能确定的运行参数。
struct ResolvedRun {
    std::string modelPath;     // 解析后的模型文件绝对路径
    double timestep = 0.0;     // 模型固定步长（秒）
    std::uint64_t totalSteps = 0;
};

// 把仿真时长换算为步数：运行到仿真时间首次 >= duration 为止。
// duration/timestep 与整数的相对误差在 1e-9 以内时视为整数，避免 5.0/0.001 因浮点误差得到 5001 步。
// 结果超出范围或输入无效时返回空。
std::optional<std::uint64_t> stepsForDuration(double durationSeconds, double timestep);

// 运行配置的 JSON 表示（单行，不含换行符），type 为 "config"。
std::string configToJson(const RunOptions& options, const ResolvedRun& resolved);

// ---- JSON Lines 输出记录（M0-06 确定的格式，修订号见 model 记录的 contract_revision）----
// 每条记录为单行 JSON 对象，不含换行符；实数以 %.17g 输出，可无损还原为 double。

// type "model"：仿真配置、单位与坐标约定、实体、关节与执行器清单。实体名称只在这里出现，state 记录只用 ID。
// actuators 字段由 M0-07 加入，属于兼容的新增字段，contract_revision 不变。
std::string modelToJson(const terra::data::ModelDescription& model);

// type "state"：一个观测时刻的完整状态（实体位姿与速度、关节状态、接触摘要、诊断）。
std::string stateToJson(const terra::data::PhysicsSnapshot& snapshot);

// type "summary"：正常结束。
std::string summaryToJson(std::uint64_t runEpoch, std::uint64_t steps, double time);

// type "error"：仿真错误。kind 为 snake_case 标识，如 "auto_reset"。
std::string errorToJson(std::string_view kind, std::uint64_t runEpoch, std::uint64_t step, double time,
                        std::string_view message);

// 实数格式化（%.17g）。
std::string formatNumber(double value);

// JSON 字符串转义（不含两侧引号）。
std::string jsonEscape(std::string_view text);

}  // namespace terra::headless
