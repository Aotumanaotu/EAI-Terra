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

#include <array>
#include <cstdint>
#include <map>
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

// 差速底盘（M4-08b，可复用：TurtleBot 4、TurtleBot 3 等）：订阅 cmdVel（geometry_msgs/Twist）换算为两轮角速度，
// timeout 秒（墙钟）没有新命令时停车；由两轮转角积分出轮式里程计，发布 odom（nav_msgs/Odometry）与 TF odomFrame → 机身。
struct DiffDriveProfile {
    std::string leftJoint, rightJoint;          // 轮子关节全名（里程计）
    std::string leftActuator, rightActuator;    // 轮子速度执行器全名（命令）
    std::string baseEntity;                     // 机身实体全名（TF 的子坐标系）
    double wheelRadius = 0.0;                   // m
    double wheelSeparation = 0.0;               // m
    double maxWheelSpeed = 0.0;                 // rad/s；0 表示不限
    std::string cmdVel = "/cmd_vel";
    std::string odom = "/odom";
    std::string odomFrame = "odom";
    double odomHz = 20.0;
    double timeout = 0.5;                       // s
};

// ---- M5-13a 算法接入档案：通用的接入设置（不针对某个算法；EAI-Nexus 等的约定写在各自的内置档案中）----
// 名字匹配：topics、frames.rename、topicSettings、planarBase、groundTruth 中的名字可以含 “*”（匹配一段中任意个字符，不跨 “/”），
// 例如 “/*/mid360/points” 匹配任一实例上名为 mid360 的雷达。这样同一份档案可以用于不同的机器人。

// 每个话题的设置（topicSettings，键为最终的话题名，可含 “*”）
struct TopicSettings {
    std::string reliability{};  // "reliable" / "best_effort"；空表示用默认
    std::string durability{};   // "volatile" / "transient_local"；空表示用默认
    int depth = 0;              // 队列深度；0 表示用默认
    double rateHz = 0.0;        // 降频（不高于传感器本身的频率才有意义）；0 表示不降
    std::string accelUnit{};    // IMU：“mps2”（默认，m/s²）或 “g”（标准重力 9.80665 m/s²，与 Livox 等设备的原生输出相同）
    std::string pointLayout{};  // 点云：“livox”（默认）、“xyzi”、“velodyne”、“ouster”，见 integration.hpp
};

// TF 的归属：哪些变换由 Terra 发布（其余由算法发布，如 SLAM 的 odom → base、robot_state_publisher 的本体变换）
struct TfSettings {
    std::string robot = "auto";  // 实体位姿：“world”（每个实体相对 world）、“tree”（父子之间的相对位姿，根实体相对 world）、
                                 // “none”（不发布）；“auto”：有底盘（diffDrive、planarBase）时为 tree，否则为 world
    bool sensorMounts = true;    // /tf_static：传感器安装位姿与相机光学坐标系
    bool odom = true;            // 底盘的 odom → 机身（diffDrive、planarBase）
};

// 真值（groundTruth）：nav_msgs/Odometry，实体在世界中的位姿，速度在实体坐标系中。只发话题，不进 TF（不与算法的 TF 冲突），
// 供评测（M5-07）与调试
struct GroundTruthSettings {
    std::string entity{};                          // 实体全名（可含 “*”，须恰好匹配一个）
    std::string topic = "/terra/ground_truth/odom";
    std::string frame = "world";
    std::string childFrame{};                      // 空表示用实体的坐标系名（按 frames 的设置换算）
    double rateHz = 50.0;
};

// 速度指令的输入（cmdVel）：可以有多路，按优先级仲裁——取没有超时的输入中优先级最高的（相同时取最近收到的）；
// 全部超时后指令归零（与真机的超时停车相同）。消息中的速度在机身坐标系中（linear.x、linear.y、angular.z）
struct CommandInput {
    std::string topic{};
    std::string type = "Twist";  // “Twist” 或 “TwistStamped”
    int priority = 0;            // 越大越优先
    double timeout = 0.5;        // s（墙钟）
};

// 平面运动学底盘（planarBase）：机身由两个滑动关节（x、y）与一个转动关节（yaw）在地面上移动，各有一个速度执行器，
// 速度指令换算成这三个关节的速度（不走步态）。用于先接通导航与定位，与具体机器人的步态策略无关。
// 关节名可含 “*”；机身实体默认为这些关节所在的刚体
struct PlanarBaseSettings {
    bool optional = false;        // true（档案中写 "planarBase": "auto"）：场景中没有这样的关节时不使用，不报错
    std::string xJoint = "*/planar_x";
    std::string yJoint = "*/planar_y";
    std::string yawJoint = "*/planar_yaw";
    std::string baseEntity{};
    std::array<double, 3> limits{0.0, 0.0, 0.0};  // |vx|、|vy|、|wz| 的上限；0 表示不限
    std::string odom{};                           // 里程计话题（nav_msgs/Odometry）；空表示不发
    std::string odomFrame = "odom";
    double odomHz = 20.0;
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
    // M4-08b：与真机一致的话题名与坐标系（可复用）
    std::map<std::string, std::string> topics;  // 默认话题名（如 /turtlebot4/rplidar/scan）→ 真机的话题名（如 /scan）
    std::string stripFramePrefix;                // 坐标系名与关节名去掉的前缀（如 "turtlebot4/"，真机的坐标系没有实例名）
    bool sensorsUseLinkFrames = false;           // 传感器安装位姿与连杆重合时，消息的 frame_id 直接用连杆（与真机相同），不另发静态 TF
    std::string depthEncoding = "32FC1";         // 深度图：32FC1（米）或 16UC1（毫米，没有命中为 0；与 OAK-D 等真机相同）
    std::optional<DiffDriveProfile> diffDrive;
    // M5-13a：接入档案
    std::string time = "sim";                           // 时间戳：“sim”（仿真时间，配 /clock）或 “wall”（墙钟，算法 use_sim_time 为 false 时）
    std::map<std::string, std::string> frameRename;     // 坐标系改名（frames.rename：Terra 的坐标系全名 → 新名字，可含 “*”），先于 stripPrefix
    std::map<std::string, TopicSettings> topicSettings;
    TfSettings tf;
    std::optional<GroundTruthSettings> groundTruth;
    std::vector<CommandInput> cmdVel;                   // 空表示没有配置（差速底盘用自己的 cmdVel；策略用 /cmd_vel）
    std::optional<PlanarBaseSettings> planarBase;
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
    std::string commandSource;           // M5-13a：当前生效的速度指令输入（话题名；没有时为空）
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
    std::vector<std::uint32_t> segmentation;  // M4-09：width × height 的实体 ID；发布为 16UC1（超过 65535 的记为 0）
    // M5-06：width × height 的温度（°C，已加噪声）；热像仪发布 <相机>/thermal/image_raw（mono16，0.01 K，量程外截断）、
    // <相机>/thermal/image_color（rgb8，相机的色板与显示区间）与 <相机>/thermal/camera_info
    std::vector<float> thermal;
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
    // M4-09：传感器噪声的运行时开关（--no-noise、查看器面板）；默认按场景文件加噪声
    void setSensorNoise(bool enabled);
    // 查看器渲染出相机画面后调用（camera 为 SceneInstance::cameras 的下标）
    void publishCamera(std::size_t camera, const CameraImage& image);
    // 主线程：取出服务请求
    std::vector<Request> takeRequests();
    // M4-10b：策略的速度指令。enableVelocityCommand 之后订阅 topic（geometry_msgs/Twist：linear.x、linear.y、angular.z，
    // 机身坐标系）；差速底盘已经订阅同一话题时返回 false。velocityCommand：收到过消息时返回最近一条，
    // 超过 timeout 秒（墙钟）没有新消息时返回 0（与真机的超时停止相同）；从未收到时返回空（保留界面或命令行的指令）。
    // M5-13a：档案中配置了 cmdVel 时用它的输入与仲裁（忽略参数）；cmdVel 已经给了底盘（diffDrive、planarBase）时返回 false
    bool enableVelocityCommand(const std::string& topic = "/cmd_vel", double timeout = 0.5);
    std::optional<std::array<double, 3>> velocityCommand();

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
