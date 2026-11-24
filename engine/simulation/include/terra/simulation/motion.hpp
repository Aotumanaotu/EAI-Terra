#pragma once

// 动作文件与播放（M3-04，M3-04 设计稿第 2 节，2026-11-24 评审通过）。
//
// 动作文件（*.terra-motion.json）：format "terra.motion"、version 1；poses 为按时间递增的“时间 → 执行器目标”表，
// 执行器用全名（带实例前缀）。未知字段一律报错，诊断带 JSON 指针（执行器名中的 “/” 按 RFC 6901 写作 “~1”）。
//
// 语义：
//   - 目标值的单位即执行器控制量的单位（Position：rad 或 m；Velocity：rad/s 或 m/s；Motor：N·m 或 N）；
//   - 每个执行器只在提到它的点之间插值；第一个提到它的点之前不下发（保持当前控制量），最后一个点之后保持最后的值；
//   - 插值：step（保持到下一个点）、linear、smooth（三次 smoothstep，在点上速度为 0），默认 smooth；
//   - 绑定到世界时校验：执行器存在、只有一个控制输入、目标在控制范围内（越界拒绝）、起始关键帧存在；
//     执行器类别为 Other 时给出警告。
// 播放：第 k 步（从动作开始的那一步算起）的动作时间为 k × 步长，与墙钟、帧率、暂停无关。

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "terra/data/ids.hpp"
#include "terra/data/model_description.hpp"
#include "terra/physics/physics_world.hpp"

namespace terra::simulation {

struct MotionIssue {
    enum class Severity { Warning, Error };
    Severity severity = Severity::Error;
    std::string where;  // JSON 指针，如 "/poses/2/time"
    std::string message;
};

enum class MotionInterpolation { Step, Linear, Smooth };

struct MotionPose {
    double time = 0.0;
    std::vector<std::pair<std::string, double>> targets;  // 执行器全名 → 目标值
    std::string where;                                    // "/poses/<i>"
};

struct MotionDocument {
    std::string path;       // 动作文件（绝对路径）
    std::string directory;  // 相对路径的基准目录
    std::string name;
    std::string scene;           // 推荐的场景文件（已按动作文件目录展开）；空表示没有
    std::string startKeyframe;   // 空表示从当前状态开始
    MotionInterpolation interpolation = MotionInterpolation::Smooth;
    std::vector<MotionPose> poses;
};

struct MotionParseResult {
    std::optional<MotionDocument> document;  // 有错误时为空
    std::vector<MotionIssue> issues;
};

MotionParseResult parseMotion(const std::string& text, const std::string& motionPath);
MotionParseResult readMotionFile(const std::string& path);

// 动作目录（2026-11-24，用户：切换场景后要能看到可用的动作）：在 roots 下递归查找 *.terra-motion.json，
// 返回推荐场景与 scenePath 相同的动作（按名字、再按路径排序）。读不出或有错误的文件跳过。
struct MotionCatalogEntry {
    std::string name;  // 动作名；文件没写时取文件名
    std::string path;  // 绝对路径
};
std::vector<MotionCatalogEntry> findMotionsForScene(const std::vector<std::string>& roots, const std::string& scenePath);

// 一个执行器的关键点（时间递增）
struct MotionTrack {
    terra::data::ActuatorId actuator;
    std::string name;
    std::vector<std::pair<double, double>> keys;  // (时间 s, 目标值)
};

struct BoundMotion {
    std::string name;
    MotionInterpolation interpolation = MotionInterpolation::Smooth;
    std::optional<std::size_t> startKeyframe;
    std::vector<MotionTrack> tracks;  // 按 ActuatorId 排序
    double duration = 0.0;            // 最后一个点的时间
};

struct MotionBindResult {
    std::optional<BoundMotion> motion;  // 有错误时为空
    std::vector<MotionIssue> issues;
};

// 目标值的合法范围（闭区间）；返回空表示不限制。默认取执行器第一个控制输入的范围。
// M3-04b 的 PD 层接管的执行器以关节限位为范围，由调用方提供。
using TargetRange = std::function<std::optional<std::pair<double, double>>(const terra::data::ActuatorDescription&)>;
std::optional<std::pair<double, double>> controlRange(const terra::data::ActuatorDescription& actuator);

MotionBindResult bindMotion(const MotionDocument& document, const terra::data::ModelDescription& model,
                            const TargetRange& range = controlRange);

// 轨道在动作时间 t 的值；t 早于第一个点时返回空
std::optional<double> sampleTrack(const MotionTrack& track, MotionInterpolation interpolation, double t);

// RFC 6901：JSON 指针中的 “~” 写作 “~0”，“/” 写作 “~1”
std::string jsonPointerToken(const std::string& key);

class MotionPlayer {
public:
    MotionPlayer() = default;
    explicit MotionPlayer(BoundMotion bound) : motion_(std::move(bound)) {}

    const BoundMotion& motion() const noexcept { return motion_; }

    // 从第 step 步开始播放（动作时间 0 对应世界的第 step 步）
    void start(std::uint64_t step) noexcept {
        start_ = step;
        playing_ = true;
    }
    void stop() noexcept { playing_ = false; }
    bool playing() const noexcept { return playing_; }
    void setLoop(bool loop) noexcept { loop_ = loop; }
    bool loop() const noexcept { return loop_; }
    std::uint64_t startStep() const noexcept { return start_; }

    // 动作时间（s）：(step − 开始步) × timestep；循环时对时长取模
    double timeAt(std::uint64_t step, double timestep) const noexcept;
    bool finished(std::uint64_t step, double timestep) const noexcept;

    // 在第 step 步之前生效的命令（世界的 stepIndex() == step 时交给 setControl）；没有播放或尚未开始时为空
    std::vector<terra::physics::ActuatorCommand> commands(std::uint64_t step, double timestep) const;

private:
    BoundMotion motion_;
    std::uint64_t start_ = 0;
    bool playing_ = false;
    bool loop_ = false;
};

}  // namespace terra::simulation
