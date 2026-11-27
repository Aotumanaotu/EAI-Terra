#pragma once

// terra_headless 命令行解析、校验与 JSON Lines 输出格式（M0-03 起）。
// 只依赖标准库与 Terra 数据契约，不涉及物理后端，便于单元测试。

#include <array>
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
    kExitRerunMismatch = 6,  // M3-07：--rerun 复现的结果超出容差
};

enum class Command { Help, Version, Run };

struct RunOptions {
    std::string modelPath;
    std::string scenePath;                  // M2-08：场景文件（与 modelPath 二选一）
    std::string motionPath;                 // M3-05：动作文件；不给 --scene 时使用动作文件中的推荐场景；不能与 --model 同用
    std::string recordPath;                 // M3-07：把本次运行写成实验记录（*.terra-record.jsonl）；需要场景（--scene 或 --motion）
    std::string rerunPath;                  // M3-07：按实验记录重新仿真并比较；不能与其他运行参数同用
    std::string libraryPath;                // M3-07b：资源库文件（SQLite）；空表示默认位置（$TERRA_LIBRARY 或 ~/.local/share/eai-terra/）
    bool listRecords = false;               // M3-07b：列出库中的实验记录（可用 --scene 筛选）后退出
    bool noNoise = false;                   // M4-09：不加传感器噪声（输出真值）
    bool listAssets = false;                // M4-08c：扫描并列出资源库中的资产（可用 --kind 筛选）后退出
    std::string assetKind;                  // M4-08c：--kind scene|motion|robot|environment（M5-02a 加 element）；空表示全部
    double positionTolerance = 1e-6;        // M3-07：--tolerance 位置,姿态（m,rad），只用于 --rerun
    double rotationTolerance = 1e-6;
    std::optional<std::uint64_t> steps;     // 与 duration 二选一
    std::optional<double> durationSeconds;  // 仿真时长（秒）；换算为步数需要模型步长，在 M0-04 后完成
    std::uint64_t sampleEvery = 1;          // 每 N 步输出一次状态
    bool dryRun = false;                    // 只校验参数与模型文件，不运行仿真
    bool sensors = false;                   // M4-02：输出传感器读数（sensor_list 与 sensor 行），按各传感器的频率采样
    std::string ros2Profile;                // M4-06a：ROS2 通信配置文件（*.terra-ros2.json）；需要场景；隐含 --realtime
    bool realtime = false;                  // M4-06a：按墙钟节奏推进（1 秒仿真时间 = 1 秒），供外部程序实时交互
    std::string policyPath;                 // M4-10a：策略（*.terra-policy.json，或 .onnx 时按场景自动配置）
    std::optional<std::array<double, 3>> command;  // M4-10a：--command vx,vy,偏航角速度（只用于 --policy）
    std::string installComponent;           // M4-10a：安装组件（如 onnxruntime-cpu）后退出
    std::string installFrom;                // M4-10a：--from 本地压缩包（离线安装）
    bool listComponents = false;            // M4-10a：列出组件与安装状态后退出
    std::string importPolicy;               // M4-10b：导入策略（.onnx / .pt）到资源库，需要 --scene（机器人）
    std::string policyName;                 // M4-10b：--name 导入后的名字
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

// type "sensor_list"（M4-02，只在 --sensors 时输出）：传感器清单与采样频率，rates 与 model.sensors 一一对应。
std::string sensorListToJson(const terra::data::ModelDescription& model, const std::vector<double>& rates);

// type "sensor"（M4-02）：一个传感器在一个采样时刻的读数。IMU 的字段为 linear_acceleration（比力，m/s²）、
// angular_velocity（rad/s），均在安装点坐标系中；orientation 为安装点在世界中的姿态（wxyz）。
std::string sensorToJson(const terra::data::SensorDescription& sensor, const terra::data::SensorReading& reading,
                         std::uint64_t step, double time);

// type "lidar"（M4-03b，只在 --sensors 时输出）：一次 2D 激光雷达扫描。ranges 为米；JSON 没有无穷大，
// 超出最大量程（没有命中）写作字符串 "inf"，近于最小量程写作 "-inf"（与 ROS REP-117 的 ±inf 对应）。
struct LidarJson {
    std::string name;
    std::uint64_t step = 0;
    double time = 0.0;
    double angleMin = 0.0;
    double angleIncrement = 0.0;
    double rangeMin = 0.0;
    double rangeMax = 0.0;
    double rateHz = 0.0;
};
std::string lidarToJson(const LidarJson& scan, const std::vector<float>& ranges);

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
