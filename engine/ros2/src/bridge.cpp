// ROS2 桥接（M4-06a 通用配置；M4-06b 宇树配置在 unitree.cpp）。rclcpp 与 ROS 消息只在 engine/ros2/src 中使用，不进入公共头。

#include "terra/ros2/bridge.hpp"
#include "runtime.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <map>
#include <mutex>
#include <set>
#include <stdexcept>
#include <thread>
#include <utility>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "terra/scene/camera_model.hpp"
#include "terra/scene/pose_math.hpp"
#include "terra/simulation/lidar.hpp"
#include "terra/simulation/lidar3d.hpp"
#include "terra/simulation/motion.hpp"
#include "terra/simulation/sensor_noise.hpp"
#include "terra/simulation/sensor_sampling.hpp"
#ifdef TERRA_ROS2_UNITREE_ENABLED
#include "unitree_link.hpp"
#endif

namespace terra::ros2 {

namespace {

namespace td = terra::data;

builtin_interfaces::msg::Time stampOf(double seconds) {
    builtin_interfaces::msg::Time t;
    const double whole = std::floor(seconds);
    auto nanoseconds = static_cast<long long>(std::llround((seconds - whole) * 1e9));
    auto sec = static_cast<long long>(whole);
    if (nanoseconds >= 1000000000LL) {
        nanoseconds -= 1000000000LL;
        ++sec;
    }
    t.sec = static_cast<std::int32_t>(sec);
    t.nanosec = static_cast<std::uint32_t>(nanoseconds);
    return t;
}

geometry_msgs::msg::Transform transformOf(const td::Pose& p) {
    geometry_msgs::msg::Transform t;
    t.translation.x = p.position.x;
    t.translation.y = p.position.y;
    t.translation.z = p.position.z;
    t.rotation.w = p.orientation.w;
    t.rotation.x = p.orientation.x;
    t.rotation.y = p.orientation.y;
    t.rotation.z = p.orientation.z;
    return t;
}

}  // namespace

struct Bridge::Impl {
    const terra::scene::SceneInstance& scene;
    const terra::simulation::PdLayer& pd;
    Profile profile;
    std::shared_ptr<rclcpp::Node> node;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    std::thread spinner;
    std::atomic<bool> stopping{false};  // 执行器线程用 spin_once 循环，看到它就退出（避免 cancel 早于 spin 时线程不退出）

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStates;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tfStatic;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr> imus;  // 与模型描述的 sensors 一一对应
    std::vector<double> imuRates;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> scans;  // 与 SceneInstance::lidars 一一对应
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> clouds;  // M5-11：与 SceneInstance::lidars3d 一一对应
    struct CameraTopics {
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr segmentation;  // M4-09：16UC1 实体 ID
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr labels;          // M4-09：对照表（JSON，transient local，只发一次）
    };
    std::vector<CameraTopics> cameras;  // 与 SceneInstance::cameras 一一对应
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr commandSub;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resetService;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pauseService;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stepService;
#ifdef TERRA_ROS2_UNITREE_ENABLED
    std::unique_ptr<detail::UnitreeLink> unitree;  // 宇树配置（M4-06b）
#endif

    // 关节：发布 /joint_states 的转动与滑动关节（关节下标），以及作用在它上面的第一个执行器（下标，没有时为 -1）
    std::vector<std::size_t> publishedJoints;
    std::vector<int> jointActuator;

    mutable std::mutex mutex;  // 保护以下：收到的命令、请求、统计
    std::map<std::uint32_t, double> pendingCommands;  // 执行器 ID → 值（同一执行器以最后一条为准）
    std::vector<Request> requests;
    Stats stats;
    // M4-10b：策略的速度指令（受 mutex 保护）
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocitySub;
    std::array<double, 3> velocity{0.0, 0.0, 0.0};
    std::chrono::steady_clock::time_point velocityAt;
    bool haveVelocity = false;
    double velocityTimeout = 0.5;
    std::atomic<std::uint64_t> published{0};

    Impl(const terra::scene::SceneInstance& s, const terra::simulation::PdLayer& p, Profile pr) : scene(s), pd(p), profile(std::move(pr)), noise(s) {}
    // M4-09：传感器噪声（与其他出口各自持有，读数逐位相同）
    terra::simulation::SensorNoise noise;

    std::string topic(const std::string& name) const { return profile.topicPrefix + "/" + topicName(name); }

    // ---- M4-08b：话题名映射、坐标系名、差速底盘 ----
    std::set<std::string> defaultTopics;  // 本场景的全部默认话题名（检查 topics 映射中的拼写）
    std::string mapped(const std::string& defaultName) {
        defaultTopics.insert(defaultName);
        const auto it = profile.topics.find(defaultName);
        return it == profile.topics.end() ? defaultName : it->second;
    }
    std::string frame(const std::string& name) const {
        const auto& p = profile.stripFramePrefix;
        return !p.empty() && name.size() > p.size() && name.compare(0, p.size(), p) == 0 ? name.substr(p.size()) : name;
    }
    std::string entityFrame(td::EntityId e) const {
        const auto* entity = scene.world->description().entity(e);
        return entity != nullptr && !entity->name.empty() ? frame(entity->name) : std::string{"world"};
    }
    static bool identity(const td::Pose& p) {
        const auto& q = p.orientation;
        return std::fabs(p.position.x) < 1e-9 && std::fabs(p.position.y) < 1e-9 && std::fabs(p.position.z) < 1e-9 && std::fabs(std::fabs(q.w) - 1.0) < 1e-9;
    }
    // 传感器消息的 frame_id：安装位姿与连杆重合且配置要求时用连杆，否则用传感器自己的坐标系
    std::string sensorFrame(const std::string& name, td::EntityId entity, const td::Pose& mount) const {
        return profile.sensorsUseLinkFrames && identity(mount) ? entityFrame(entity) : frame(name);
    }
    std::string cameraFrame(const terra::scene::CameraSensor& c) const {
        const td::Pose optical = terra::scene::multiply(c.mount, td::Pose{{0, 0, 0}, terra::scene::opticalRotation()});
        return profile.sensorsUseLinkFrames && identity(optical) ? entityFrame(c.entity) : frame(c.name + "_optical");
    }

    struct Drive {
        std::size_t leftJoint = 0, rightJoint = 0;
        td::ActuatorId left, right;
        std::size_t base = 0;  // 机身实体下标
        std::string baseFrame;
        double x = 0.0, y = 0.0, theta = 0.0;
        std::uint64_t epoch = 0;
        std::uint64_t lastStep = 0;
        bool started = false;
        double v = 0.0, w = 0.0;  // 最近一条 cmd_vel
        std::chrono::steady_clock::time_point lastCommand;
        bool haveCommand = false;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd;
    };
    std::optional<Drive> drive;

    void reject(const std::string& reason) {
        ++stats.commandsRejected;
        stats.lastRejection = reason;
    }

    // 执行器线程：收到命令。按执行器类别取字段：PD 层接管的与位置执行器取 position，速度执行器取 velocity，其余取 effort
    void onCommand(const sensor_msgs::msg::JointState& msg) {
        const auto& d = scene.world->description();
        std::lock_guard<std::mutex> lock(mutex);
        for (std::size_t i = 0; i < msg.name.size(); ++i) {
            const auto* a = d.findActuator(msg.name[i]);
            if (a == nullptr || a->inputs.size() != 1) {
                reject("未知执行器或多输入执行器：" + msg.name[i]);
                continue;
            }
            const bool managed = pd.manages(a->id);
            const std::vector<double>* field = &msg.effort;
            const char* fieldName = "effort";
            if (managed || a->kind == td::ActuatorKind::Position) {
                field = &msg.position;
                fieldName = "position";
            } else if (a->kind == td::ActuatorKind::Velocity) {
                field = &msg.velocity;
                fieldName = "velocity";
            }
            if (i >= field->size()) {
                reject(msg.name[i] + "：缺少 " + fieldName);
                continue;
            }
            const double value = (*field)[i];
            const auto range = managed ? pd.targetRange(*a) : terra::simulation::controlRange(*a);
            if (!std::isfinite(value) || (range && (value < range->first || value > range->second))) {
                reject(msg.name[i] + " 的 " + fieldName + " = " + std::to_string(value) + " 超出范围或不是有限值");
                continue;
            }
            pendingCommands[a->id.value] = value;
            ++stats.commandsAccepted;
        }
    }

    void request(Request r) {
        std::lock_guard<std::mutex> lock(mutex);
        requests.push_back(r);
    }

    void publishStatic() {
        const auto& d = scene.world->description();
        tf2_msgs::msg::TFMessage msg;
        const auto add = [&](const std::string& parent, const std::string& child, const td::Pose& pose) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = stampOf(0.0);
            t.header.frame_id = frame(parent);  // M4-08b：按配置去掉前缀
            t.child_frame_id = frame(child);
            t.transform = transformOf(pose);
            msg.transforms.push_back(t);
        };
        const auto frameOf = [&](td::EntityId e) {
            const auto* entity = d.entity(e);
            return entity != nullptr && !entity->name.empty() ? entity->name : std::string{"world"};
        };
        for (const auto& s : d.sensors) {
            if (!(profile.sensorsUseLinkFrames && identity(s.mount))) add(frameOf(s.entity), s.name, s.mount);
        }
        for (const auto& l : scene.lidars) {
            if (!(profile.sensorsUseLinkFrames && identity(l.mount))) add(frameOf(l.entity), l.name, l.mount);
        }
        for (const auto& l : scene.lidars3d) {  // M5-11
            if (!(profile.sensorsUseLinkFrames && identity(l.mount))) add(frameOf(l.entity), l.name, l.mount);
        }
        for (const auto& c : scene.cameras) {
            if (cameraFrame(c) == entityFrame(c.entity)) continue;  // M4-08b：光学坐标系就是连杆（如 oakd_rgb_camera_optical_frame）
            add(frameOf(c.entity), c.name, c.mount);
            add(c.name, c.name + "_optical", td::Pose{{0, 0, 0}, terra::scene::opticalRotation()});
        }
        if (!msg.transforms.empty()) {
            tfStatic->publish(msg);
            ++published;
        }
    }
};

Bridge::Bridge(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

Bridge::~Bridge() {
    impl_->stopping = true;
    if (impl_->spinner.joinable()) impl_->spinner.join();
    impl_->executor.reset();
#ifdef TERRA_ROS2_UNITREE_ENABLED
    impl_->unitree.reset();
#endif
    impl_->node.reset();
    releaseRclcpp();
}

std::unique_ptr<Bridge> Bridge::create(const terra::scene::SceneInstance& scene, const terra::simulation::PdLayer& pd,
                                       const Profile& profile, std::string& error) {
    if (profile.kind != "generic" && profile.kind != "unitree") {
        error = "未知的配置类型“" + profile.kind + "”";
        return nullptr;
    }
#ifndef TERRA_ROS2_UNITREE_ENABLED
    if (profile.kind == "unitree") {
        error = "宇树配置需要宇树消息包（unitree_go、unitree_hg），本程序构建时没有找到。请按固定提交编译 unitree_ros2 的 cyclonedds_ws，"
                "source 它的 install/setup.bash 后重新配置并构建（M4-05 设计稿第 6 节）";
        return nullptr;
    }
#endif
    acquireRclcpp();
    auto impl = std::make_unique<Impl>(scene, pd, profile);
    Impl& b = *impl;
    try {
        b.node = std::make_shared<rclcpp::Node>("terra_simulator");
        const auto& d = scene.world->description();
        // QoS（设计稿第 3 节）：/clock 为 ClockQoS；关节与 TF 为可靠、深度 10 / 100；传感器为 sensor data（best effort、深度 5）；
        // /tf_static 为 transient local（后连上的订阅者也能收到）
        if (profile.clock) b.clock = b.node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
        b.jointStates = b.node->create_publisher<sensor_msgs::msg::JointState>(b.mapped(profile.topicPrefix + "/joint_states"), rclcpp::QoS(10).reliable());
        b.tf = b.node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100).reliable());
        b.tfStatic = b.node->create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", rclcpp::QoS(1).reliable().transient_local());
        if (profile.sensors) {
            for (std::size_t k = 0; k < d.sensors.size(); ++k) {
                b.imus.push_back(b.node->create_publisher<sensor_msgs::msg::Imu>(b.mapped(b.topic(d.sensors[k].name) + "/imu"), rclcpp::SensorDataQoS()));
                double rate = 1.0 / d.timestep;
                for (const auto& c : scene.sensors) {
                    if (c.sensor == d.sensors[k].id) rate = c.rateHz;
                }
                b.imuRates.push_back(rate);
            }
            for (const auto& l : scene.lidars) {
                b.scans.push_back(b.node->create_publisher<sensor_msgs::msg::LaserScan>(b.mapped(b.topic(l.name) + "/scan"), rclcpp::SensorDataQoS()));
            }
            for (const auto& l : scene.lidars3d) {  // M5-11：<雷达名>/points（可用 topics 映射成真机的 /livox/lidar）
                b.clouds.push_back(b.node->create_publisher<sensor_msgs::msg::PointCloud2>(b.mapped(b.topic(l.name) + "/points"), rclcpp::SensorDataQoS()));
            }
            for (const auto& c : scene.cameras) {
                Impl::CameraTopics t;
                if (c.rgb) t.color = b.node->create_publisher<sensor_msgs::msg::Image>(b.mapped(b.topic(c.name) + "/image_raw"), rclcpp::SensorDataQoS());
                if (c.depth) t.depth = b.node->create_publisher<sensor_msgs::msg::Image>(b.mapped(b.topic(c.name) + "/depth/image_raw"), rclcpp::SensorDataQoS());
                t.info = b.node->create_publisher<sensor_msgs::msg::CameraInfo>(b.mapped(b.topic(c.name) + "/camera_info"), rclcpp::SensorDataQoS());
                if (c.segmentation) {  // M4-09
                    t.segmentation = b.node->create_publisher<sensor_msgs::msg::Image>(b.mapped(b.topic(c.name) + "/segmentation/image_raw"),
                                                                                      rclcpp::SensorDataQoS());
                    t.labels = b.node->create_publisher<std_msgs::msg::String>(b.mapped(b.topic(c.name) + "/segmentation/labels"),
                                                                               rclcpp::QoS(1).reliable().transient_local());
                    std_msgs::msg::String labels;
                    labels.data = terra::scene::segmentationLabelsJson(b.scene);
                    t.labels->publish(labels);
                }
                b.cameras.push_back(t);
            }
        }
        // /joint_states 的关节：转动与滑动关节（有名字的）
        for (std::size_t j = 0; j < d.joints.size(); ++j) {
            const auto& joint = d.joints[j];
            if (joint.name.empty() || (joint.type != td::JointType::Hinge && joint.type != td::JointType::Slide)) continue;
            b.publishedJoints.push_back(j);
            int actuator = -1;
            for (std::size_t a = 0; a < d.actuators.size() && actuator < 0; ++a) {
                if (d.actuators[a].joint == joint.id) actuator = static_cast<int>(a);
            }
            b.jointActuator.push_back(actuator);
        }
        Impl* self = impl.get();
        if (profile.commands) {
            b.commandSub = b.node->create_subscription<sensor_msgs::msg::JointState>(
                "/terra/joint_commands", rclcpp::QoS(10).reliable(),
                [self](const sensor_msgs::msg::JointState::SharedPtr msg) { self->onCommand(*msg); });
        }
#ifdef TERRA_ROS2_UNITREE_ENABLED
        if (profile.unitree) {
            std::string linkError;
            b.unitree = detail::createUnitreeLink(
                *b.node, scene, profile,
                [self](bool accepted, const std::string& reason) {
                    std::lock_guard<std::mutex> lock(self->mutex);
                    if (accepted) ++self->stats.commandsAccepted;
                    else self->reject(reason);
                },
                linkError);
            if (!b.unitree) throw std::runtime_error(linkError);
        }
#endif
        if (profile.services) {
            b.resetService = b.node->create_service<std_srvs::srv::Trigger>(
                "/terra/reset", [self](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                    self->request(Request::Reset);
                    res->success = true;
                    res->message = "已接受：在下一帧重置";
                });
            b.pauseService = b.node->create_service<std_srvs::srv::SetBool>(
                "/terra/pause", [self](const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
                    self->request(req->data ? Request::Pause : Request::Resume);
                    res->success = true;
                    res->message = req->data ? "已接受：暂停" : "已接受：继续";
                });
            b.stepService = b.node->create_service<std_srvs::srv::Trigger>(
                "/terra/step", [self](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                    self->request(Request::Step);
                    res->success = true;
                    res->message = "已接受：单步（暂停时有效）";
                });
        }
        // M4-08b：差速底盘
        if (profile.diffDrive) {
            const auto& dd = *profile.diffDrive;
            Impl::Drive drive;
            const auto jointIndex = [&](const std::string& n) -> std::optional<std::size_t> {
                const auto* j = d.findJoint(n);
                if (j == nullptr || j->type != td::JointType::Hinge) return std::nullopt;
                return static_cast<std::size_t>(j - d.joints.data());
            };
            const auto lj = jointIndex(dd.leftJoint);
            const auto rj = jointIndex(dd.rightJoint);
            const auto* la = d.findActuator(dd.leftActuator);
            const auto* ra = d.findActuator(dd.rightActuator);
            const auto* base = d.findEntity(dd.baseEntity);
            if (!lj || !rj) throw std::runtime_error("差速底盘：找不到轮子关节（转动关节）" + dd.leftJoint + " / " + dd.rightJoint);
            if (la == nullptr || ra == nullptr || la->kind != td::ActuatorKind::Velocity || ra->kind != td::ActuatorKind::Velocity) {
                throw std::runtime_error("差速底盘：找不到轮子的速度执行器 " + dd.leftActuator + " / " + dd.rightActuator);
            }
            if (base == nullptr) throw std::runtime_error("差速底盘：找不到机身实体 " + dd.baseEntity);
            drive.leftJoint = *lj;
            drive.rightJoint = *rj;
            drive.left = la->id;
            drive.right = ra->id;
            drive.base = base->id.value - 1;
            drive.baseFrame = b.frame(base->name);
            drive.odom = b.node->create_publisher<nav_msgs::msg::Odometry>(profile.topicPrefix + dd.odom, rclcpp::QoS(10).reliable());
            drive.cmd = b.node->create_subscription<geometry_msgs::msg::Twist>(
                profile.topicPrefix + dd.cmdVel, rclcpp::QoS(10), [self](const geometry_msgs::msg::Twist::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(self->mutex);
                    if (!self->drive) return;
                    self->drive->v = msg->linear.x;
                    self->drive->w = msg->angular.z;
                    self->drive->lastCommand = std::chrono::steady_clock::now();
                    self->drive->haveCommand = true;
                    ++self->stats.commandsAccepted;
                });
            b.drive = std::move(drive);
        }
        // 话题名映射中的每一项都应是本场景的默认话题（防止拼写错误使映射不生效）
        for (const auto& [from, to] : profile.topics) {
            if (b.defaultTopics.count(from) == 0) {
                std::string available;
                for (const auto& t : b.defaultTopics) available += (available.empty() ? "" : "、") + t;
                throw std::runtime_error("通信配置 topics 中的“" + from + "”不是本场景的话题（可映射的：" + available + "）");
            }
        }
        b.publishStatic();
        b.executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        b.executor->add_node(b.node);
        rclcpp::executors::SingleThreadedExecutor* executor = b.executor.get();
        std::atomic<bool>* stopping = &b.stopping;
        b.spinner = std::thread([executor, stopping] {
            while (!*stopping && rclcpp::ok()) executor->spin_once(std::chrono::milliseconds(20));
        });
    } catch (const std::exception& e) {
        error = std::string{"ROS2 桥接创建失败："} + e.what();
        b.stopping = true;
        if (b.spinner.joinable()) b.spinner.join();
        b.executor.reset();
#ifdef TERRA_ROS2_UNITREE_ENABLED
        b.unitree.reset();
#endif
        b.node.reset();
        releaseRclcpp();
        return nullptr;
    }
    return std::unique_ptr<Bridge>(new Bridge(std::move(impl)));
}

std::vector<terra::physics::ActuatorCommand> Bridge::takeCommands() {
    std::map<std::uint32_t, double> pending;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        pending.swap(impl_->pendingCommands);
    }
    std::vector<terra::physics::ActuatorCommand> out;
    out.reserve(pending.size() + 2);
    for (const auto& [id, value] : pending) out.push_back({td::ActuatorId{id}, {value}});
    // M4-08b：差速底盘——最近一条 cmd_vel 换算为两轮角速度；超时（墙钟）停车，与 Create 3 相同
    if (impl_->profile.diffDrive) {
        const auto& dd = *impl_->profile.diffDrive;
        double v = 0.0, w = 0.0;
        td::ActuatorId left, right;
        {
            std::lock_guard<std::mutex> lock(impl_->mutex);
            if (!impl_->drive) return out;
            const auto& drive = *impl_->drive;
            const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - drive.lastCommand).count();
            if (drive.haveCommand && age <= dd.timeout) {
                v = drive.v;
                w = drive.w;
            }
            left = drive.left;
            right = drive.right;
        }
        double wl = (v - w * dd.wheelSeparation / 2.0) / dd.wheelRadius;
        double wr = (v + w * dd.wheelSeparation / 2.0) / dd.wheelRadius;
        if (dd.maxWheelSpeed > 0.0) {
            // 超过最大轮速时按比例缩小两轮（保持转弯半径）
            const double peak = std::max(std::fabs(wl), std::fabs(wr));
            if (peak > dd.maxWheelSpeed) {
                wl *= dd.maxWheelSpeed / peak;
                wr *= dd.maxWheelSpeed / peak;
            }
        }
        out.push_back({left, {wl}});
        out.push_back({right, {wr}});
    }
    return out;
}

std::vector<terra::physics::ActuatorCommand> Bridge::directCommands() {
#ifdef TERRA_ROS2_UNITREE_ENABLED
    if (impl_->unitree) return impl_->unitree->commands(*impl_->scene.world->snapshot());
#endif
    return {};
}

void Bridge::setSensorNoise(bool enabled) { impl_->noise.setEnabled(enabled); }

void Bridge::publish(const td::PhysicsSnapshot& truth) {
    Impl& b = *impl_;
    // M4-09：IMU 加噪声（宇树 LowState 的 IMU 也用它）；没有噪声时不复制快照
    td::PhysicsSnapshot noisy;
    if (b.noise.active()) noisy = b.noise.withNoise(truth);
    const td::PhysicsSnapshot& snapshot = b.noise.active() ? noisy : truth;
    const auto& d = b.scene.world->description();
    const double dt = snapshot.fixedTimestep;
    const auto stamp = stampOf(snapshot.simulationTime);
    if (b.clock) {
        rosgraph_msgs::msg::Clock msg;
        msg.clock = stamp;
        b.clock->publish(msg);
        ++b.published;
    }
    if (terra::simulation::sensorSampleDue(snapshot.stepIndex, dt, b.profile.jointStatesHz)) {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = stamp;
        for (std::size_t k = 0; k < b.publishedJoints.size(); ++k) {
            const std::size_t j = b.publishedJoints[k];
            msg.name.push_back(b.frame(d.joints[j].name));  // M4-08b：按配置去掉前缀（与真机的关节名相同）
            msg.position.push_back(j < snapshot.joints.size() ? snapshot.joints[j].position[0] : 0.0);
            msg.velocity.push_back(j < snapshot.joints.size() ? snapshot.joints[j].velocity[0] : 0.0);
            const int a = b.jointActuator[k];
            msg.effort.push_back(a >= 0 && static_cast<std::size_t>(a) < snapshot.actuators.size() ? snapshot.actuators[static_cast<std::size_t>(a)].force : 0.0);
        }
        b.jointStates->publish(msg);
        ++b.published;
    }
    // M4-08b：差速底盘的轮式里程计（每步积分两轮转角；重置后从 0 开始）
    if (b.drive) {
        auto& drive = *b.drive;
        const auto& dd = *b.profile.diffDrive;
        if (!drive.started || snapshot.runEpoch != drive.epoch || snapshot.stepIndex <= drive.lastStep) {
            drive.x = drive.y = drive.theta = 0.0;
            drive.started = true;
            drive.epoch = snapshot.runEpoch;
        } else if (drive.leftJoint < snapshot.joints.size() && drive.rightJoint < snapshot.joints.size()) {
            const double h = dt * static_cast<double>(snapshot.stepIndex - drive.lastStep);
            const double wl = snapshot.joints[drive.leftJoint].velocity[0];
            const double wr = snapshot.joints[drive.rightJoint].velocity[0];
            const double v = dd.wheelRadius * (wl + wr) / 2.0;
            const double w = dd.wheelRadius * (wr - wl) / dd.wheelSeparation;
            const double mid = drive.theta + w * h / 2.0;
            drive.x += v * std::cos(mid) * h;
            drive.y += v * std::sin(mid) * h;
            drive.theta += w * h;
        }
        drive.lastStep = snapshot.stepIndex;
        if (terra::simulation::sensorSampleDue(snapshot.stepIndex, dt, dd.odomHz) && drive.leftJoint < snapshot.joints.size() &&
            drive.rightJoint < snapshot.joints.size()) {
            const double wl = snapshot.joints[drive.leftJoint].velocity[0];
            const double wr = snapshot.joints[drive.rightJoint].velocity[0];
            nav_msgs::msg::Odometry odom;
            odom.header.stamp = stamp;
            odom.header.frame_id = dd.odomFrame;
            odom.child_frame_id = drive.baseFrame;
            odom.pose.pose.position.x = drive.x;
            odom.pose.pose.position.y = drive.y;
            odom.pose.pose.orientation.w = std::cos(drive.theta / 2.0);
            odom.pose.pose.orientation.z = std::sin(drive.theta / 2.0);
            odom.twist.twist.linear.x = dd.wheelRadius * (wl + wr) / 2.0;
            odom.twist.twist.angular.z = dd.wheelRadius * (wr - wl) / dd.wheelSeparation;
            drive.odom->publish(odom);
            ++b.published;
        }
    }
    if (terra::simulation::sensorSampleDue(snapshot.stepIndex, dt, b.profile.tfHz)) {
        tf2_msgs::msg::TFMessage msg;
        if (b.drive) {
            // M4-08b：与真机相同的 TF 树：odom → 机身（轮式里程计），连杆之间为相对位姿（真机由 robot_state_publisher 发布）
            const auto& drive = *b.drive;
            geometry_msgs::msg::TransformStamped root;
            root.header.stamp = stamp;
            root.header.frame_id = b.profile.diffDrive->odomFrame;
            root.child_frame_id = drive.baseFrame;
            root.transform.translation.x = drive.x;
            root.transform.translation.y = drive.y;
            root.transform.rotation.w = std::cos(drive.theta / 2.0);
            root.transform.rotation.z = std::sin(drive.theta / 2.0);
            msg.transforms.push_back(root);
            for (std::size_t e = 0; e < d.entities.size() && e < snapshot.entities.size(); ++e) {
                const auto& entity = d.entities[e];
                if (entity.name.empty() || !entity.parent.valid() || entity.parent.value - 1 >= snapshot.entities.size()) continue;
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = stamp;
                t.header.frame_id = b.entityFrame(entity.parent);
                t.child_frame_id = b.frame(entity.name);
                t.transform = transformOf(terra::scene::multiply(terra::scene::inverse(snapshot.entities[entity.parent.value - 1].pose), snapshot.entities[e].pose));
                msg.transforms.push_back(t);
            }
        } else {
            for (std::size_t e = 0; e < d.entities.size() && e < snapshot.entities.size(); ++e) {
                if (d.entities[e].name.empty()) continue;
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = stamp;
                t.header.frame_id = "world";
                t.child_frame_id = b.frame(d.entities[e].name);
                t.transform = transformOf(snapshot.entities[e].pose);
                msg.transforms.push_back(t);
            }
        }
        b.tf->publish(msg);
        ++b.published;
    }
    for (std::size_t k = 0; k < b.imus.size() && k < snapshot.sensors.size(); ++k) {
        if (!terra::simulation::sensorSampleDue(snapshot.stepIndex, dt, b.imuRates[k])) continue;
        const auto& v = snapshot.sensors[k].values;
        if (v.size() < 10) continue;
        sensor_msgs::msg::Imu msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = b.sensorFrame(d.sensors[k].name, d.sensors[k].entity, d.sensors[k].mount);
        msg.linear_acceleration.x = v[0];
        msg.linear_acceleration.y = v[1];
        msg.linear_acceleration.z = v[2];
        msg.angular_velocity.x = v[3];
        msg.angular_velocity.y = v[4];
        msg.angular_velocity.z = v[5];
        msg.orientation.w = v[6];
        msg.orientation.x = v[7];
        msg.orientation.y = v[8];
        msg.orientation.z = v[9];
        b.imus[k]->publish(msg);
        ++b.published;
    }
    for (std::size_t k = 0; k < b.scans.size(); ++k) {
        const auto& lidar = b.scene.lidars[k];
        if (!terra::simulation::sensorSampleDue(snapshot.stepIndex, dt, lidar.rateHz)) continue;
        auto scan = terra::simulation::scanLidar(*b.scene.world, lidar);
        b.noise.lidar(k, scan);  // M4-09
        sensor_msgs::msg::LaserScan msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = b.sensorFrame(lidar.name, lidar.entity, lidar.mount);
        msg.angle_min = static_cast<float>(scan.angleMin);
        msg.angle_max = static_cast<float>(scan.angleMin + scan.angleIncrement * static_cast<double>(lidar.samples > 0 ? lidar.samples - 1 : 0));
        msg.angle_increment = static_cast<float>(scan.angleIncrement);
        msg.time_increment = 0.0f;
        msg.scan_time = static_cast<float>(1.0 / lidar.rateHz);
        msg.range_min = static_cast<float>(scan.rangeMin);
        msg.range_max = static_cast<float>(scan.rangeMax);
        msg.ranges = scan.ranges;
        b.scans[k]->publish(msg);
        ++b.published;
    }
    // M5-11：3D 激光雷达 → PointCloud2。字段与 livox_ros_driver2 的 PointCloud2 格式相同：
    // x、y、z、intensity（float32），tag、line（uint8），timestamp（float64，每点的测量时刻，纳秒）；
    // 消息头的时刻为第一点的测量时刻；intensity 固定为 100（Terra 不模拟反射率），tag 为 0
    for (std::size_t k = 0; k < b.clouds.size(); ++k) {
        const auto& lidar = b.scene.lidars3d[k];
        if (!terra::simulation::sensorSampleDue(snapshot.stepIndex, dt, lidar.rateHz)) continue;
        auto scan = terra::simulation::scanLidar3d(*b.scene.world, lidar);
        b.noise.lidar3d(k, scan);
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = stampOf(std::max(0.0, scan.frameStart));
        msg.header.frame_id = b.sensorFrame(lidar.name, lidar.entity, lidar.mount);
        const auto field = [&](const char* name, std::uint32_t offset, std::uint8_t type) {
            sensor_msgs::msg::PointField f;
            f.name = name;
            f.offset = offset;
            f.datatype = type;
            f.count = 1;
            msg.fields.push_back(f);
        };
        using PF = sensor_msgs::msg::PointField;
        field("x", 0, PF::FLOAT32);
        field("y", 4, PF::FLOAT32);
        field("z", 8, PF::FLOAT32);
        field("intensity", 12, PF::FLOAT32);
        field("tag", 16, PF::UINT8);
        field("line", 17, PF::UINT8);
        field("timestamp", 18, PF::FLOAT64);
        msg.point_step = 26;
        msg.height = 1;
        msg.width = static_cast<std::uint32_t>(scan.points.size());
        msg.row_step = msg.point_step * msg.width;
        msg.is_bigendian = false;
        msg.is_dense = true;
        msg.data.resize(static_cast<std::size_t>(msg.row_step));
        std::uint8_t* out = msg.data.data();
        const float intensity = 100.0f;
        for (const auto& p : scan.points) {
            const double t = (std::max(0.0, scan.frameStart) + static_cast<double>(p.time)) * 1e9;
            std::memcpy(out, &p.x, 4);
            std::memcpy(out + 4, &p.y, 4);
            std::memcpy(out + 8, &p.z, 4);
            std::memcpy(out + 12, &intensity, 4);
            out[16] = 0;
            out[17] = p.line;
            std::memcpy(out + 18, &t, 8);
            out += msg.point_step;
        }
        b.clouds[k]->publish(msg);
        ++b.published;
    }
#ifdef TERRA_ROS2_UNITREE_ENABLED
    if (b.unitree) b.unitree->publish(snapshot);
#endif
}

void Bridge::publishCamera(std::size_t camera, const CameraImage& image) {
    Impl& b = *impl_;
    if (camera >= b.cameras.size() || camera >= b.scene.cameras.size()) return;
    const auto& topics = b.cameras[camera];
    const std::string frame = b.cameraFrame(b.scene.cameras[camera]);
    const auto stamp = stampOf(image.time);
    const auto w = static_cast<std::uint32_t>(image.width);
    const auto h = static_cast<std::uint32_t>(image.height);
    if (topics.color && !image.rgb.empty()) {
        sensor_msgs::msg::Image msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = frame;
        msg.width = w;
        msg.height = h;
        msg.encoding = "rgb8";
        msg.step = w * 3;
        msg.data = image.rgb;
        topics.color->publish(msg);
        ++b.published;
    }
    if (topics.depth && !image.depth.empty()) {
        sensor_msgs::msg::Image msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = frame;
        msg.width = w;
        msg.height = h;
        msg.is_bigendian = 0;
        if (b.profile.depthEncoding == "16UC1") {
            // M4-08b：与 OAK-D 等真机相同，毫米、无符号 16 位，没有命中为 0
            msg.encoding = "16UC1";
            msg.step = w * 2;
            msg.data.resize(image.depth.size() * 2);
            for (std::size_t i = 0; i < image.depth.size(); ++i) {
                const float m = image.depth[i];
                const auto mm = std::isfinite(m) && m > 0.0f ? static_cast<std::uint16_t>(std::min(65535.0, std::round(static_cast<double>(m) * 1000.0))) : std::uint16_t{0};
                std::memcpy(msg.data.data() + i * 2, &mm, 2);
            }
        } else {
            msg.encoding = "32FC1";
            msg.step = w * 4;
            msg.data.resize(image.depth.size() * sizeof(float));
            std::memcpy(msg.data.data(), image.depth.data(), msg.data.size());
        }
        topics.depth->publish(msg);
        ++b.published;
    }
    if (topics.segmentation && !image.segmentation.empty()) {  // M4-09
        sensor_msgs::msg::Image msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = frame;
        msg.width = w;
        msg.height = h;
        msg.is_bigendian = 0;
        msg.encoding = "16UC1";
        msg.step = w * 2;
        msg.data.resize(image.segmentation.size() * 2);
        for (std::size_t i = 0; i < image.segmentation.size(); ++i) {
            const std::uint32_t id = image.segmentation[i] > 0xFFFFu ? 0u : image.segmentation[i];
            msg.data[2 * i] = static_cast<std::uint8_t>(id & 0xFFu);
            msg.data[2 * i + 1] = static_cast<std::uint8_t>(id >> 8);
        }
        topics.segmentation->publish(msg);
        ++b.published;
    }
    sensor_msgs::msg::CameraInfo info;
    info.header.stamp = stamp;
    info.header.frame_id = frame;
    info.width = w;
    info.height = h;
    info.distortion_model = "plumb_bob";
    info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    info.k = {image.fx, 0.0, image.cx, 0.0, image.fy, image.cy, 0.0, 0.0, 1.0};
    info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    info.p = {image.fx, 0.0, image.cx, 0.0, 0.0, image.fy, image.cy, 0.0, 0.0, 0.0, 1.0, 0.0};
    topics.info->publish(info);
    ++b.published;
}

bool Bridge::enableVelocityCommand(const std::string& topic, double timeout) {
    Impl* self = impl_.get();
    const std::string name = self->profile.topicPrefix + topic;
    if (self->profile.diffDrive && self->profile.topicPrefix + self->profile.diffDrive->cmdVel == name) return false;
    std::lock_guard<std::mutex> lock(self->mutex);
    self->velocityTimeout = timeout;
    if (self->velocitySub) return true;
    self->velocitySub = self->node->create_subscription<geometry_msgs::msg::Twist>(
        name, rclcpp::QoS(10), [self](const geometry_msgs::msg::Twist::SharedPtr msg) {
            std::lock_guard<std::mutex> guard(self->mutex);
            self->velocity = {msg->linear.x, msg->linear.y, msg->angular.z};
            self->velocityAt = std::chrono::steady_clock::now();
            self->haveVelocity = true;
            ++self->stats.commandsAccepted;
        });
    return true;
}

std::optional<std::array<double, 3>> Bridge::velocityCommand() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (!impl_->haveVelocity) return std::nullopt;
    const double age = std::chrono::duration<double>(std::chrono::steady_clock::now() - impl_->velocityAt).count();
    if (age > impl_->velocityTimeout) return std::array<double, 3>{0.0, 0.0, 0.0};
    return impl_->velocity;
}

std::vector<Request> Bridge::takeRequests() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    std::vector<Request> out;
    out.swap(impl_->requests);
    return out;
}

Stats Bridge::stats() const {
    Stats s;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        s = impl_->stats;
    }
    s.published = impl_->published.load();
    std::size_t subscribers = 0;
    if (impl_->jointStates) subscribers += impl_->jointStates->get_subscription_count();
    if (impl_->tf) subscribers += impl_->tf->get_subscription_count();
    if (impl_->clock) subscribers += impl_->clock->get_subscription_count();
    s.subscribers = subscribers;
    return s;
}

const Profile& Bridge::profile() const noexcept { return impl_->profile; }

}  // namespace terra::ros2
