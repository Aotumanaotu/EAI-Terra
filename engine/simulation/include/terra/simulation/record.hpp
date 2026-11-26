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
// 版本 2（M4-07，传感器记录）：
//   header   另有 sensors：IMU 名字（按传感器 ID 顺序）、2D 激光雷达与相机（名字、频率、束数或分辨率）
//   state    另有 sensors：每份状态都记全部 IMU 的读数（快照的 sensors，每个 10 个数）
//   lidar    雷达在其采样步上的一次扫描（距离；没有命中写成字符串 "inf" / "-inf"）
//   camera   相机在其采样步上的元数据：步序号、时间、相机位姿（Terra 相机坐标系在世界中）
//   lidar3d  M5-11：3D 雷达在其采样步上的一帧：帧序号、点数与内容摘要（sha256）；header 的 sensors 另有 lidars3d
//   camera_image  该步的图像已保存（相对记录文件所在目录的路径；只有查看器在打开“记录相机图像”时写，且只写实际渲染的帧）
// 版本 1 的记录照常读取（没有传感器）。
// 数值按往返精确的方式写出，读回后逐位相同。一份记录对应一段连续运行（中途重置时由调用方结束本记录、另起一份）。
//
// 复现（重新仿真）：按记录头加载同一场景，从同一起始状态开始，每步之前下发记录的控制量（其中已包含 PD 层与动作的输出，
// 复现时不再运行它们），在每个采样点与记录比较位姿：同一机器、同一构建应逐位相同；跨机器按容差比较（评审问题 3）。

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "terra/data/geometry.hpp"
#include "terra/data/state.hpp"
#include "terra/physics/physics_world.hpp"
#include "terra/scene/composition.hpp"
#include "terra/scene/scene.hpp"
#include "terra/simulation/lidar.hpp"
#include "terra/simulation/lidar3d.hpp"

namespace terra::simulation {

// M4-07：记录头中的传感器清单
struct RecordLidarInfo {
    std::string name;
    double rateHz = 0.0;
    int samples = 0;
};
// M5-11：3D 激光雷达（每帧只记点数与内容摘要，点本身体积太大；复现时重算并逐位比较摘要）
struct RecordLidar3dInfo {
    std::string name;
    double rateHz = 0.0;
    std::size_t pointsPerFrame = 0;
};
struct RecordCameraInfo {
    std::string name;
    double rateHz = 0.0;
    int width = 0;
    int height = 0;
};

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
    // M4-07（版本 2）：传感器
    std::vector<std::string> imus;
    std::vector<RecordLidarInfo> lidars;
    std::vector<RecordCameraInfo> cameras;
    std::vector<RecordLidar3dInfo> lidars3d;  // M5-11
    // M4-09：记录时是否加了传感器噪声（场景文件的配置与种子随场景文件的摘要核对）；复现时按此设置。旧记录没有此项，视为没有噪声
    bool sensorNoise = false;
    // M4-10a：运行时的策略（只作记录：复现按记录的控制量，不需要推理库）。没有策略时为空
    std::string policyPath;        // 说明文件或模型文件
    std::string policySha256;      // 说明文件的摘要（直接给模型文件时与 policyModelSha256 相同）
    std::string policyModelSha256;
    std::string policyBackend;     // 如 "ONNX Runtime 1.32.0（CPU，单线程，确定性计算）"
};

// 由模型描述与场景信息填写物理设置与名字清单（运行环境、文件摘要由调用方补充）
RecordHeader makeRecordHeader(const terra::data::ModelDescription& model, std::uint64_t sampleEvery);
// M4-07：填写传感器清单（IMU 已由 makeRecordHeader 按模型描述填写；这里补雷达与相机）
void describeSensors(RecordHeader& header, const terra::scene::SceneInstance& scene);

// M4-07：雷达的一次扫描与相机的一帧元数据
struct RecordLidar {
    std::uint64_t step = 0;
    double time = 0.0;
    std::string name;
    std::vector<float> ranges;
};
struct RecordLidar3d {
    std::uint64_t step = 0;
    double time = 0.0;
    std::string name;
    std::uint64_t frame = 0;
    std::size_t points = 0;
    std::string sha256;  // lidar3dDigest
};
struct RecordCameraFrame {
    std::uint64_t step = 0;
    double time = 0.0;
    std::string name;
    terra::data::Pose pose;  // Terra 相机坐标系在世界中的位姿（terra::scene::cameraWorldPose）
    std::string rgb;         // 图像文件（相对记录文件所在目录）；空表示没有保存
    std::string depth;
};

// 相机图像保存的目录：记录文件名去掉 ".terra-record.jsonl" 再加 ".frames"（与记录文件在同一目录）
std::string recordFramesDirectory(const std::string& recordPath);

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
    std::vector<RecordLidar> lidars;              // M4-07：按写入顺序（步序号不减）
    std::vector<RecordLidar3d> lidars3d;          // M5-11：按写入顺序
    std::vector<RecordCameraFrame> cameraFrames;  // M4-07：按写入顺序；camera_image 合并到同一帧
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
    // M4-07：传感器（IMU 随 state 写，不需要单独调用）
    void lidar(const LidarScan& scan);
    void lidar3d(const Lidar3dScan& scan);  // M5-11：只写点数与摘要
    void camera(const RecordCameraFrame& frame);  // 只写元数据（rgb、depth 忽略）
    void cameraImage(std::uint64_t step, const std::string& name, const std::string& rgb, const std::string& depth);
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

// M4-07：按场景的频率记录这一步的雷达扫描与相机元数据（Headless 用；查看器自己算过雷达，直接调用 lidar / camera）
class SensorNoise;  // terra/simulation/sensor_noise.hpp
// M4-09：noise 非空时雷达扫描加噪声（与其他出口相同的读数）
void recordSceneSensors(RecordWriter& writer, const terra::physics::PhysicsWorld& world, const terra::scene::SceneInstance& scene,
                        const terra::data::PhysicsSnapshot& snapshot, const SensorNoise* noise = nullptr);

// 只读记录头与摘要（第一行与最后一个非空行），用于建立索引（M3-07b）：不解析中间的状态，大文件也很快。
// 失败时返回空，error 为原因；没有 summary 时 summary 为空（记录没有正常结束）。
struct RecordInfo {
    RecordHeader header;
    std::optional<RecordSummary> summary;
};
std::optional<RecordInfo> readRecordInfo(const std::string& path, std::string& error);

// 第 step 步时生效的控制量（M3-07d，回放时执行器面板的目标）：每个执行器取步序号 ≤ step 的最后一次记录；
// 从未记录过的执行器不在结果中。键为 ActuatorId 的值。
std::map<std::uint32_t, std::vector<double>> controlsAt(const Record& record, std::uint64_t step);

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
    // M4-07：传感器（IMU 在每个采样点、雷达与相机位姿在记录的采样步上比较；逐位比较，不设容差）
    std::size_t sensorSamples = 0;
    bool sensorsBitwise = true;
    std::string firstSensorDifference;
    bool withinTolerance() const noexcept { return ran && !firstExceedStep; }
};

RerunResult rerunRecord(const Record& record, const terra::scene::WorldFactory& factory, const RerunOptions& options = {});

}  // namespace terra::simulation
