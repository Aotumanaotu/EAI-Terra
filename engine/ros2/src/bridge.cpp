// ROS2 桥接（M4-06a 通用配置；M4-06b 宇树配置在 unitree.cpp）。rclcpp 与 ROS 消息只在 engine/ros2/src 中使用，不进入公共头。

#include "terra/ros2/bridge.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <map>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "terra/scene/camera_model.hpp"
#include "terra/scene/pose_math.hpp"
#include "terra/simulation/lidar.hpp"
#include "terra/simulation/motion.hpp"
#include "terra/simulation/sensor_sampling.hpp"
#ifdef TERRA_ROS2_UNITREE_ENABLED
#include "unitree_link.hpp"
#endif

namespace terra::ros2 {

namespace {

namespace td = terra::data;

// 进程内只初始化一次 rclcpp；不接管信号（查看器与 Headless 自己处理 Ctrl+C）
std::mutex initMutex;
int liveBridges = 0;

void acquireRclcpp() {
    std::lock_guard<std::mutex> lock(initMutex);
    if (liveBridges++ == 0 && !rclcpp::ok()) {
        rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    }
}

void releaseRclcpp() {
    std::lock_guard<std::mutex> lock(initMutex);
    if (--liveBridges == 0 && rclcpp::ok()) rclcpp::shutdown();
}

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
    struct CameraTopics {
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info;
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
    std::atomic<std::uint64_t> published{0};

    Impl(const terra::scene::SceneInstance& s, const terra::simulation::PdLayer& p, Profile pr) : scene(s), pd(p), profile(std::move(pr)) {}

    std::string topic(const std::string& name) const { return profile.topicPrefix + "/" + topicName(name); }

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
            t.header.frame_id = parent;
            t.child_frame_id = child;
            t.transform = transformOf(pose);
            msg.transforms.push_back(t);
        };
        const auto frameOf = [&](td::EntityId e) {
            const auto* entity = d.entity(e);
            return entity != nullptr && !entity->name.empty() ? entity->name : std::string{"world"};
        };
        for (const auto& s : d.sensors) add(frameOf(s.entity), s.name, s.mount);
        for (const auto& l : scene.lidars) add(frameOf(l.entity), l.name, l.mount);
        for (const auto& c : scene.cameras) {
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
        b.jointStates = b.node->create_publisher<sensor_msgs::msg::JointState>(profile.topicPrefix + "/joint_states", rclcpp::QoS(10).reliable());
        b.tf = b.node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(100).reliable());
        b.tfStatic = b.node->create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", rclcpp::QoS(1).reliable().transient_local());
        if (profile.sensors) {
            for (std::size_t k = 0; k < d.sensors.size(); ++k) {
                b.imus.push_back(b.node->create_publisher<sensor_msgs::msg::Imu>(b.topic(d.sensors[k].name) + "/imu", rclcpp::SensorDataQoS()));
                double rate = 1.0 / d.timestep;
                for (const auto& c : scene.sensors) {
                    if (c.sensor == d.sensors[k].id) rate = c.rateHz;
                }
                b.imuRates.push_back(rate);
            }
            for (const auto& l : scene.lidars) {
                b.scans.push_back(b.node->create_publisher<sensor_msgs::msg::LaserScan>(b.topic(l.name) + "/scan", rclcpp::SensorDataQoS()));
            }
            for (const auto& c : scene.cameras) {
                Impl::CameraTopics t;
                if (c.rgb) t.color = b.node->create_publisher<sensor_msgs::msg::Image>(b.topic(c.name) + "/image_raw", rclcpp::SensorDataQoS());
                if (c.depth) t.depth = b.node->create_publisher<sensor_msgs::msg::Image>(b.topic(c.name) + "/depth/image_raw", rclcpp::SensorDataQoS());
                t.info = b.node->create_publisher<sensor_msgs::msg::CameraInfo>(b.topic(c.name) + "/camera_info", rclcpp::SensorDataQoS());
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
    out.reserve(pending.size());
    for (const auto& [id, value] : pending) out.push_back({td::ActuatorId{id}, {value}});
    return out;
}

std::vector<terra::physics::ActuatorCommand> Bridge::directCommands() {
#ifdef TERRA_ROS2_UNITREE_ENABLED
    if (impl_->unitree) return impl_->unitree->commands(*impl_->scene.world->snapshot());
#endif
    return {};
}

void Bridge::publish(const td::PhysicsSnapshot& snapshot) {
    Impl& b = *impl_;
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
            msg.name.push_back(d.joints[j].name);
            msg.position.push_back(j < snapshot.joints.size() ? snapshot.joints[j].position[0] : 0.0);
            msg.velocity.push_back(j < snapshot.joints.size() ? snapshot.joints[j].velocity[0] : 0.0);
            const int a = b.jointActuator[k];
            msg.effort.push_back(a >= 0 && static_cast<std::size_t>(a) < snapshot.actuators.size() ? snapshot.actuators[static_cast<std::size_t>(a)].force : 0.0);
        }
        b.jointStates->publish(msg);
        ++b.published;
    }
    if (terra::simulation::sensorSampleDue(snapshot.stepIndex, dt, b.profile.tfHz)) {
        tf2_msgs::msg::TFMessage msg;
        for (std::size_t e = 0; e < d.entities.size() && e < snapshot.entities.size(); ++e) {
            if (d.entities[e].name.empty()) continue;
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = stamp;
            t.header.frame_id = "world";
            t.child_frame_id = d.entities[e].name;
            t.transform = transformOf(snapshot.entities[e].pose);
            msg.transforms.push_back(t);
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
        msg.header.frame_id = d.sensors[k].name;
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
        const auto scan = terra::simulation::scanLidar(*b.scene.world, lidar);
        sensor_msgs::msg::LaserScan msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = lidar.name;
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
#ifdef TERRA_ROS2_UNITREE_ENABLED
    if (b.unitree) b.unitree->publish(snapshot);
#endif
}

void Bridge::publishCamera(std::size_t camera, const CameraImage& image) {
    Impl& b = *impl_;
    if (camera >= b.cameras.size() || camera >= b.scene.cameras.size()) return;
    const auto& topics = b.cameras[camera];
    const std::string frame = b.scene.cameras[camera].name + "_optical";
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
        msg.encoding = "32FC1";
        msg.is_bigendian = 0;
        msg.step = w * 4;
        msg.data.resize(image.depth.size() * sizeof(float));
        std::memcpy(msg.data.data(), image.depth.data(), msg.data.size());
        topics.depth->publish(msg);
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
