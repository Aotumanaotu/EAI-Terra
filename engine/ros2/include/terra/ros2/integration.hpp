#pragma once

// 算法接入档案（M5-13a）中与 ROS 无关的部分：名字匹配、点云布局、墙钟时间戳、速度指令仲裁、平面底盘的换算。
// 不含 ROS 类型，由桥接（bridge.cpp）使用，也由单元测试（ros2.profile）直接测试。

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "terra/ros2/bridge.hpp"
#include "terra/simulation/lidar3d.hpp"

namespace terra::ros2 {

// “*” 匹配一段中任意个字符（不跨 “/”）；其余字符逐个相同
bool globMatch(std::string_view pattern, std::string_view name);
bool hasGlob(std::string_view pattern);

// IMU 的加速度单位为 g 时除以它（标准重力，ISO 80000-3）
constexpr double kStandardGravity = 9.80665;

// ---- 点云布局 ----
// livox：   与 livox_ros_driver2 的 PointCloud2 相同：x、y、z、intensity（float32）、tag、line（uint8）、
//           timestamp（float64，每点测量时刻的绝对时间，纳秒）；每点 26 字节
// xyzi：    x、y、z、intensity（float32）；每点 16 字节
// velodyne：与 velodyne_pointcloud 相同：x、y、z、intensity（float32）、ring（uint16）、time（float32，相对消息头时刻的秒）；
//           每点 22 字节
// ouster：  与 ouster-ros 相同：x、y、z（float32，偏移 0、4、8）、intensity（float32，16）、t（uint32，相对消息头时刻的纳秒，20）、
//           reflectivity（uint16，24）、ring（uint16，26）、ambient（uint16，28）、range（uint32，毫米，32）；每点 48 字节
// 消息头的时刻都是一帧第一点的测量时刻；intensity 固定为 100（Terra 不模拟反射率），reflectivity、ambient 为 0
enum class PointLayout { Livox, Xyzi, Velodyne, Ouster };
std::optional<PointLayout> pointLayoutFromName(std::string_view name);

struct CloudField {
    std::string name;
    std::uint32_t offset = 0;
    std::uint8_t datatype = 0;  // sensor_msgs/PointField 的类型码：UINT8 2、UINT16 4、UINT32 6、FLOAT32 7、FLOAT64 8
};

struct PackedCloud {
    std::vector<CloudField> fields;
    std::uint32_t pointStep = 0;
    std::vector<std::uint8_t> data;  // 小端
};

// headerTime：消息头的时刻（秒，与输出的时间戳同一时基：仿真时间或墙钟），即 scan.frameStart 对应的输出时刻
PackedCloud packPoints(PointLayout layout, const terra::simulation::Lidar3dScan& scan, double headerTime);

// ---- 墙钟时间戳 ----
// 仿真时间 t 对应的墙钟时刻 = 偏移 + t。偏移在第一次、重置（运行序号变化）、仿真时间倒退，以及与墙钟相差超过 tolerance 秒
// （暂停之后、仿真慢于实时）时重新取为“现在 − t”；其余时候不变，同一帧内与相邻帧之间的时间间隔与仿真时间完全相同
class WallStamps {
public:
    explicit WallStamps(double tolerance = 0.25) : tolerance_(tolerance) {}
    void sync(double simTime, std::uint64_t runEpoch, double wallNow);
    double map(double simTime) const noexcept { return offset_ + simTime; }
    std::uint64_t reanchors() const noexcept { return reanchors_; }

private:
    double tolerance_;
    double offset_ = 0.0;
    double lastSim_ = 0.0;
    std::uint64_t epoch_ = 0;
    bool anchored_ = false;
    std::uint64_t reanchors_ = 0;
};

// ---- 速度指令的仲裁 ----
// 时间为调用方给的单调时钟（秒）。current：没有超时的输入中优先级最高的（相同优先级取最近收到的）；
// 收到过但全部超时时为 0；从未收到时为空
class CommandArbiter {
public:
    void configure(std::vector<CommandInput> inputs);
    const std::vector<CommandInput>& inputs() const noexcept { return inputs_; }
    void receive(std::size_t input, const std::array<double, 3>& command, double now);
    std::optional<std::array<double, 3>> current(double now) const;
    // 当前生效的输入（下标）；没有时为空
    std::optional<std::size_t> active(double now) const;

private:
    struct Slot {
        std::array<double, 3> command{0.0, 0.0, 0.0};
        double at = 0.0;
        bool received = false;
    };
    std::vector<CommandInput> inputs_;
    std::vector<Slot> slots_;
};

// ---- 平面运动学底盘 ----
// 机身坐标系的速度指令（vx、vy、wz）→ 三个关节的速度：滑动关节的轴在底盘初始朝向的坐标系中，yaw 为转动关节的当前角度。
// 先按 limits 截断（0 表示不限）
std::array<double, 3> planarJointVelocities(const std::array<double, 3>& command, double yaw, const std::array<double, 3>& limits);

}  // namespace terra::ros2
