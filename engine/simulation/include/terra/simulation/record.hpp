#pragma once

// 实验记录与复现（M3-07a，M3-07 设计稿 2026-11-24 评审通过）。
//
// 记录文件（*.terra-record.jsonl）：每行一个 JSON 对象，带 "type"：
//   header   第一行：format "terra.record"、version 1；程序与物理后端版本、平台、场景 / 动作文件路径与 sha256、
//            起始关键帧、物理设置、状态采样间隔、实体 / 关节 / 执行器的名字（按 ID 顺序）、创建时间
//   control  第 step 步之前实际下发的控制量；只在变化时写（评审问题 1），只列出变化的执行器
//   state    采样的状态（实体位姿与速度、关节、接触摘要、诊断），字段与 Headless 的 state 行相同
//   event    事件（如 "error"），带步序号
//   summary  最后一行：总步数、仿真时间、结束原因
// 数值按往返精确的方式写出，读回后逐位相同。一份记录对应一段连续运行（中途重置时由调用方结束本记录、另起一份）。
//
// 复现（重新仿真）：按记录头加载同一场景，从同一起始状态开始，每步之前下发记录的控制量（其中已包含 PD 层与动作的输出，
// 复现时不再运行它们），在每个采样点与记录比较位姿：同一机器、同一构建应逐位相同；跨机器按容差比较（评审问题 3）。

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "terra/data/geometry.hpp"
#include "terra/data/state.hpp"
#include "terra/physics/physics_world.hpp"
#include "terra/scene/composition.hpp"

namespace terra::simulation {

struct RecordHeader {
    // 运行环境（由调用方填写）
    std::string terraVersion;
    std::string compiler;
    std::string physicsBackend;  // 如 "MuJoCo 4.0.1"
    std::string platform;        // 如 "linux x86_64"
    std::string created;         // UTC，ISO 8601
    // 输入
    std::string scenePath;
    std::string sceneSha256;
    std::string motionPath;  // 没有动作时为空
    std::string motionSha256;
    std::string startKeyframe;  // 记录开始时按此关键帧重置过；空表示场景打开时的状态（场景的初始关键帧已应用）
    // 物理设置
    double timestep = 0.0;
    std::string integrator;
    std::string solver;
    int iterations = 0;
    double tolerance = 0.0;
    terra::data::Vec3 gravity;
    // 记录选项与模型摘要
    std::uint64_t sampleEvery = 10;  // 每 N 步写一次状态（第 0 步与最后一步总写）
    std::vector<std::string> entities;
    std::vector<std::string> joints;
    std::vector<std::string> actuators;
};

// 由模型描述与场景信息填写物理设置与名字清单（运行环境、文件摘要由调用方补充）
RecordHeader makeRecordHeader(const terra::data::ModelDescription& model, std::uint64_t sampleEvery);

struct RecordControl {
    std::uint64_t step = 0;
    std::vector<terra::physics::ActuatorCommand> commands;
};

struct RecordEvent {
    std::uint64_t step = 0;
    std::string kind;
    std::string detail;
};

struct RecordSummary {
    std::uint64_t steps = 0;
    double time = 0.0;
    std::string reason;  // "completed"、"error"、"stopped"
};

struct Record {
    RecordHeader header;
    std::vector<RecordControl> controls;            // 按步序号递增
    std::vector<terra::data::PhysicsSnapshot> states;  // 按步序号递增
    std::vector<RecordEvent> events;
    std::optional<RecordSummary> summary;  // 没有时表示记录没有正常结束（如程序中断）
};

class RecordWriter {
public:
    // 创建文件并写入记录头；失败时返回空，error 为原因
    static std::unique_ptr<RecordWriter> create(const std::string& path, const RecordHeader& header, std::string& error);
    ~RecordWriter();
    RecordWriter(const RecordWriter&) = delete;
    RecordWriter& operator=(const RecordWriter&) = delete;

    // 第 step 步之前实际下发的控制量；与上一次记录的值相同的执行器不写，全部相同时不写这一行
    void control(std::uint64_t step, const std::vector<terra::physics::ActuatorCommand>& commands);
    void state(const terra::data::PhysicsSnapshot& snapshot);
    void event(std::uint64_t step, const std::string& kind, const std::string& detail = {});
    // 写入 summary 并关闭文件；返回整个记录是否写入成功
    bool finish(std::uint64_t steps, double time, const std::string& reason);

    bool good() const noexcept;
    std::uint64_t bytes() const noexcept;
    const std::string& path() const noexcept;

private:
    struct Impl;
    explicit RecordWriter(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

struct RecordReadResult {
    std::optional<Record> record;      // 有错误时为空
    std::vector<std::string> errors;   // “第 N 行：原因”
    std::vector<std::string> warnings;
};

RecordReadResult readRecord(const std::string& path);

struct RerunOptions {
    double positionTolerance = 1e-6;  // m
    double rotationTolerance = 1e-6;  // rad
};

struct RerunResult {
    bool ran = false;                  // 复现跑完（场景可以加载、控制量都被接受、没有仿真错误）
    std::string error;                 // ran 为假时的原因
    std::vector<std::string> warnings; // 文件摘要或版本与记录不同等
    std::uint64_t steps = 0;
    std::size_t compared = 0;          // 比较的采样点数
    bool bitwise = true;               // 所有采样点的位姿逐位相同
    double maxPositionError = 0.0;     // m
    double maxRotationError = 0.0;     // rad
    std::optional<std::uint64_t> firstExceedStep;  // 第一次超出容差的采样步
    std::optional<std::uint64_t> firstDifferenceStep;  // 第一个不逐位相同的采样步（诊断用）
    std::string firstDifference;                       // 该采样点上第一个不同的实体与差值（诊断用）
    bool withinTolerance() const noexcept { return ran && !firstExceedStep; }
};

RerunResult rerunRecord(const Record& record, const terra::scene::WorldFactory& factory, const RerunOptions& options = {});

}  // namespace terra::simulation
