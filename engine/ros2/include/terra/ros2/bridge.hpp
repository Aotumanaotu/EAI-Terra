#pragma once

// ROS2 桥接（M4-06，M4-05 设计稿 2026-11-24 评审通过）。公共头不含任何 ROS 类型：查看器与 Headless 只通过这里使用桥接，
// 只有打开 TERRA_BUILD_ROS2 并找到 ROS2 Humble 时才构建（D-104：进程内的可选模块）。
//
// 线程（设计稿问题 4）：rclcpp 的执行器在桥接自己的线程中运行（收命令、服务请求）；
//   - publish() 在仿真线程上、每步之后（以及打开、重置之后的初始状态）调用，按配置的频率发布；
//   - takeCommands() 在仿真线程上、每步之前调用，取出收到的命令，换算为执行器控制量，在这一步生效；
//   - takeRequests() 在主线程上调用，取出重置、暂停 / 继续、单步请求，由程序按自己的方式执行。
//
// 通用配置（M4-06a，设计稿第 3 节）：标准消息。
//   发布：/clock（每步）、/joint_states、/tf（按配置的频率）、/tf_static（传感器安装坐标系与相机光学坐标系）、
//         /<IMU 名>/imu、/<雷达名>/scan（各自的频率）、/<相机名>/image_raw、/<相机名>/depth/image_raw、/<相机名>/camera_info
//         （相机由查看器渲染后调用 publishCamera）；
//   订阅：/terra/joint_commands（sensor_msgs/JointState：名字为执行器全名；position 给位置执行器与 PD 层，
//         velocity 给速度执行器，effort 给力矩执行器；越界或缺字段的拒绝并计数）；
//   服务：/terra/reset（std_srvs/Trigger）、/terra/pause（std_srvs/SetBool：true 暂停，false 继续）、/terra/step（Trigger）。
// 话题名中不允许的字符（如实例名中的 “-”）换成 “_”；坐标系名用全名。时间戳为仿真时间。

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "terra/data/geometry.hpp"
#include "terra/data/state.hpp"
#include "terra/physics/physics_world.hpp"
#include "terra/scene/scene.hpp"
#include "terra/simulation/pd_layer.hpp"

namespace terra::ros2 {

// 宇树配置（M4-06b，kind "unitree"）：与真机相同的低层话题（M4-05 设计稿第 4 节）。
//   发布 /lowstate、/lf/lowstate（unitree_go 或 unitree_hg 的 LowState，含 CRC）；go 另有 /sportmodestate、/lf/sportmodestate、
//   /wirelesscontroller；订阅 /lowcmd（LowCmd，CRC 不对的拒绝）。每一步按最近一条 LowCmd 计算每个电机的力矩
//   τ = tau + kp (q − q̂) + kd (dq − dq̂)（mode = 0 时为 0），截断到执行器的控制范围；收到第一条之前力矩为 0。
struct UnitreeProfile {
    std::string messages;             // "go"（unitree_go，20 个电机位）或 "hg"（unitree_hg，35 个电机位）
    std::string instance;             // 场景中的模型实例名（如 "go2"）
    std::vector<std::string> motors;  // 电机编号 → 关节名（实例内，如 "FR_hip_joint"）；空字符串表示该编号没有电机
    std::string imu;                  // 机身 IMU（实例内的传感器名，如 "imu"）
    std::vector<std::string> feet;    // go：foot_force 的四个足端实体（实例内），顺序与真机相同（FR、FL、RR、RL）
    std::string base;                 // go：sportmodestate 的机身实体（实例内）
    double lowstateHz = 500.0;
    double lfLowstateHz = 50.0;       // 0 表示不发布
    double sportModeStateHz = 500.0;  // go；0 表示不发布
    double lfSportModeStateHz = 50.0;
    double wirelessControllerHz = 50.0;
    bool checkCrc = true;             // 关闭只用于调试（与真机不同）
    int modeMachine = 0;              // hg：LowState 的 mode_machine（机型代码，待真机核对）
};

// 通信配置文件（*.terra-ros2.json，format "terra.ros2"、version 1；严格校验，未知字段报错）
struct Profile {
    std::string path;
    std::string name;
    std::string kind = "generic";  // "generic" 或 "unitree"（M4-06b，需要 unitree 字段）
    std::string topicPrefix;       // 加在全部话题名前（如 "/sim"），默认为空
    bool clock = true;             // 发布 /clock
    double jointStatesHz = 100.0;
    double tfHz = 100.0;
    bool sensors = true;           // IMU、雷达、相机
    bool commands = true;          // 订阅 /terra/joint_commands
    bool services = true;          // /terra/reset、/terra/pause、/terra/step
    std::optional<UnitreeProfile> unitree;
};

struct ProfileResult {
    std::optional<Profile> profile;
    std::vector<std::string> errors;  // “JSON 指针：原因”
};

ProfileResult readProfile(const std::string& path);
ProfileResult parseProfile(const std::string& text, const std::string& path);

// 话题名中不允许的字符换成 “_”（ROS2 名字只允许字母、数字、下划线与 “/”，且段首不能是数字）
std::string topicName(const std::string& name);

enum class Request { Reset, Pause, Resume, Step };

struct Stats {
    std::uint64_t published = 0;         // 发布的消息总数
    std::uint64_t commandsAccepted = 0;  // 换算成控制量的命令条目数（宇树配置：接受的 LowCmd 条数）
    std::uint64_t commandsRejected = 0;  // 拒绝的命令条目数（未知执行器、缺字段、越界、非有限值；宇树配置：CRC 不对等）
    std::string lastRejection;           // 最近一次拒绝的原因
    std::size_t subscribers = 0;         // 当前订阅了任一话题的订阅者数（界面显示）
};

// 相机的一帧（查看器渲染后交给桥接；行 0 为图像顶部；内参为 M4-03a 的约定）
struct CameraImage {
    std::uint64_t step = 0;
    double time = 0.0;
    int width = 0;
    int height = 0;
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
    std::vector<std::uint8_t> rgb;  // width × height × 3；空表示不输出彩色
    std::vector<float> depth;       // width × height，米；空表示不输出深度
};

class Bridge {
public:
    // scene（含物理世界）与 pd 在桥接的生命周期内必须有效。失败时返回空，error 为原因。
    static std::unique_ptr<Bridge> create(const terra::scene::SceneInstance& scene, const terra::simulation::PdLayer& pd,
                                          const Profile& profile, std::string& error);
    ~Bridge();
    Bridge(const Bridge&) = delete;
    Bridge& operator=(const Bridge&) = delete;

    // 每步之前（仿真线程）：收到的关节命令换算成控制量（同一执行器以最后一条为准）；PD 层接管的执行器给的是目标位置，
    // 由调用方交给 PD 层处理（与动作、面板的路径相同）
    std::vector<terra::physics::ActuatorCommand> takeCommands();
    // PD 层之后（仿真线程，M4-06b）：直接给执行器的控制量，并在 PD 层与动作的输出之后（同一执行器以它为准）。
    // 宇树配置：按最近一条 LowCmd 与当前状态算出的各电机力矩；通用配置：空
    std::vector<terra::physics::ActuatorCommand> directCommands();
    // 每步之后与初始状态（仿真线程）：按频率发布；snapshot 必须是当前状态（雷达在这里做射线检测）
    void publish(const terra::data::PhysicsSnapshot& snapshot);
    // 查看器渲染出相机画面后调用（camera 为 SceneInstance::cameras 的下标）
    void publishCamera(std::size_t camera, const CameraImage& image);
    // 主线程：取出服务请求
    std::vector<Request> takeRequests();

    Stats stats() const;
    const Profile& profile() const noexcept;

private:
    struct Impl;
    explicit Bridge(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

// 把桥接的命令并入已有的命令：同一执行器以后者为准（动作、面板的命令在前，桥接的在后）
std::vector<terra::physics::ActuatorCommand> mergeCommands(std::vector<terra::physics::ActuatorCommand> base,
                                                           const std::vector<terra::physics::ActuatorCommand>& later);

}  // namespace terra::ros2
