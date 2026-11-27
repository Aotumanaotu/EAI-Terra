// ROS2 桥接（M4-06a 通用配置；M4-06b 宇树配置在 unitree.cpp；M5-13a 算法接入档案：时间戳、TF 归属、话题设置、
// 速度指令的多路输入、平面底盘、真值）。rclcpp 与 ROS 消息只在 engine/ros2/src 中使用，不进入公共头。

#include "terra/ros2/bridge.hpp"
#include "terra/ros2/integration.hpp"
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
#include <geometry_msgs/msg/twist_stamped.hpp>
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
#include "terra/simulation/thermal.hpp"
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

double steadyNow() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
double wallNow() { return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); }

// M5-13a：同一组名字中，名字（或模式）name 匹配的项：先找完全相同的，再找含 “*” 的模式（按键的顺序取第一个）
template <class Map>
auto findMatch(const Map& map, const std::string& name) -> decltype(map.end()) {
    const auto exact = map.find(name);
    if (exact != map.end()) return exact;
    for (auto it = map.begin(); it != map.end(); ++it) {
        if (hasGlob(it->first) && globMatch(it->first, name)) return it;
    }
    return map.end();
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
    std::vector<double> imuAccelScale;  // M5-13a：1（m/s²）或 1 / 9.80665（g）
    std::vector<PointLayout> cloudLayouts;  // M5-13a：与 clouds 一一对应
    std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr> scans;  // 与 SceneInstance::lidars 一一对应
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> clouds;  // M5-11：与 SceneInstance::lidars3d 一一对应
    struct CameraTopics {
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr segmentation;  // M4-09：16UC1 实体 ID
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr labels;          // M4-09：对照表（JSON，transient local，只发一次）
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr thermalRaw;    // M5-06：mono16，0.01 K
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr thermalColor;  // M5-06：rgb8 伪彩色
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr thermalInfo;
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
    // M4-10b / M5-13a：速度指令的输入与仲裁（受 mutex 保护）。consumer 为使用它的一方：差速底盘、平面底盘或策略
    enum class Consumer { None, Drive, Planar, Policy };
    Consumer consumer = Consumer::None;
    CommandArbiter arbiter;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> commandInputs;
    std::atomic<std::uint64_t> published{0};

    Impl(const terra::scene::SceneInstance& s, const terra::simulation::PdLayer& p, Profile pr) : scene(s), pd(p), profile(std::move(pr)), noise(s) {}
    // M4-09：传感器噪声（与其他出口各自持有，读数逐位相同）
    terra::simulation::SensorNoise noise;

    std::string topic(const std::string& name) const { return profile.topicPrefix + "/" + topicName(name); }

    // ---- M4-08b：话题名映射、坐标系名、差速底盘 ----
    std::set<std::string> defaultTopics;  // 本场景的全部默认话题名（检查 topics 映射中的拼写）
    std::map<std::string, std::vector<std::string>> patternUses;  // M5-13a：含 “*” 的映射 → 它匹配的默认话题名
    std::string mapped(const std::string& defaultName) {
        defaultTopics.insert(defaultName);
        const auto it = findMatch(profile.topics, defaultName);
        if (it == profile.topics.end()) return defaultName;
        if (hasGlob(it->first)) patternUses[it->first].push_back(defaultName);
        return it->second;
    }
    std::string frame(const std::string& name) const {
        // M5-13a：先按 frames.rename 改名，没有改名的再去掉前缀
        if (const auto it = findMatch(profile.frameRename, name); it != profile.frameRename.end()) return it->second;
        const auto& p = profile.stripFramePrefix;
        if (p == "*/") {  // M5-13a：去掉实例名（第一段），与具体的实例名无关
            const auto slash = name.find('/');
            return slash != std::string::npos && slash + 1 < name.size() ? name.substr(slash + 1) : name;
        }
        return !p.empty() && name.size() > p.size() && name.compare(0, p.size(), p) == 0 ? name.substr(p.size()) : name;
    }

    // ---- M5-13a：话题设置（QoS、降频）、时间戳 ----
    std::map<std::string, std::string> createdTopics;  // 最终的话题名 → 消息类别（imu、cloud、other），检查 topicSettings
    const TopicSettings* settings(const std::string& name) const {
        const auto it = findMatch(profile.topicSettings, name);
        return it == profile.topicSettings.end() ? nullptr : &it->second;
    }
    rclcpp::QoS qos(const std::string& name, rclcpp::QoS q, const char* kind = "other") {
        createdTopics[name] = kind;
        if (const auto* t = settings(name)) {
            if (t->depth > 0) q.keep_last(static_cast<std::size_t>(t->depth));
            if (t->reliability == "reliable") q.reliable();
            if (t->reliability == "best_effort") q.best_effort();
            if (t->durability == "volatile") q.durability_volatile();
            if (t->durability == "transient_local") q.transient_local();
        }
        return q;
    }
    // 降频：按仿真时间，平均频率为 rateHz（不高于原频率）；仿真时间倒退（重置）时从头开始
    struct Gate {
        double next = 0.0;
        double last = -1.0;
    };
    std::map<std::string, Gate> gates;
    bool due(const char* name, double t) {
        const auto* s = settings(name);
        if (s == nullptr || s->rateHz <= 0.0) return true;
        Gate& g = gates[name];
        const double period = 1.0 / s->rateHz;
        if (g.last < 0.0 || t < g.last) g.next = t;
        g.last = t;
        if (t + 1e-9 < g.next) return false;
        g.next = std::max(g.next + period, t);
        return true;
    }
    bool wallTime = false;
    WallStamps wall;
    double outTime(double simTime) const { return wallTime ? wall.map(simTime) : simTime; }
    builtin_interfaces::msg::Time stamp(double simTime) const { return stampOf(outTime(simTime)); }
    std::string entityFrame(td::EntityId e) const {
        const auto* entity = scene.world->description().entity(e);
        return entity != nullptr && !entity->name.empty() ? frame(entity->name) : frame("world");
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
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
    };
    std::optional<Drive> drive;

    // M5-13a：平面运动学底盘（关节下标、执行器、机身）；里程计相对每次运行开始时的位姿
    struct Planar {
        std::size_t x = 0, y = 0, yaw = 0;  // 关节下标
        td::ActuatorId ax, ay, ayaw;
        std::size_t base = 0;               // 机身实体下标
        std::string baseFrame;
        double x0 = 0.0, y0 = 0.0, yaw0 = 0.0;
        std::uint64_t epoch = 0;
        bool started = false;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
    };
    std::optional<Planar> planar;

    // M5-13a：真值
    struct Truth {
        std::size_t entity = 0;
        std::string childFrame;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub;
    };
    std::optional<Truth> truth;

    // 实体 TF 的方式（tf.robot 的 auto 已换算）
    std::string robotTf = "world";

    // 速度指令的一路输入收到消息（执行器线程）
    void onVelocity(std::size_t input, double vx, double vy, double wz) {
        std::lock_guard<std::mutex> lock(mutex);
        if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) {
            reject("速度指令不是有限值：" + arbiter.inputs()[input].topic);
            return;
        }
        arbiter.receive(input, {vx, vy, wz}, steadyNow());
        ++stats.commandsAccepted;
    }
    // 订阅速度指令的各路输入（Twist 或 TwistStamped）
    void subscribeInputs(std::vector<CommandInput> inputs, Consumer who) {
        arbiter.configure(std::move(inputs));
        consumer = who;
        Impl* self = this;
        for (std::size_t i = 0; i < arbiter.inputs().size(); ++i) {
            const auto& in = arbiter.inputs()[i];
            const std::string name = profile.topicPrefix + in.topic;
            if (in.type == "TwistStamped") {
                commandInputs.push_back(node->create_subscription<geometry_msgs::msg::TwistStamped>(
                    name, qos(name, rclcpp::QoS(10)), [self, i](const geometry_msgs::msg::TwistStamped::SharedPtr m) {
                        self->onVelocity(i, m->twist.linear.x, m->twist.linear.y, m->twist.angular.z);
                    }));
            } else {
                commandInputs.push_back(node->create_subscription<geometry_msgs::msg::Twist>(
                    name, qos(name, rclcpp::QoS(10)),
                    [self, i](const geometry_msgs::msg::Twist::SharedPtr m) { self->onVelocity(i, m->linear.x, m->linear.y, m->angular.z); }));
            }
        }
    }

    // 名字（或模式）匹配的唯一一项；没有或不唯一时报错（what 为说明）
    template <class Items, class NameOf>
    std::optional<std::size_t> unique(const Items& items, const std::string& pattern, NameOf&& nameOf, const std::string& what, bool optional) const {
        std::vector<std::size_t> hits;
        for (std::size_t i = 0; i < items.size(); ++i) {
            const std::string& n = nameOf(items[i]);
            if (!n.empty() && globMatch(pattern, n)) hits.push_back(i);
        }
        if (hits.size() == 1) return hits[0];
        if (hits.empty() && optional) return std::nullopt;
        if (hits.empty()) throw std::runtime_error(what + "：没有名字匹配“" + pattern + "”的");
        std::string list;
        for (const auto i : hits) list += (list.empty() ? "" : "、") + nameOf(items[i]);
        throw std::runtime_error(what + "：“" + pattern + "”匹配了多个（" + list + "）");
    }

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
        if (!profile.tf.sensorMounts) return;  // M5-13a：传感器安装位姿由算法一侧发布（如 robot_state_publisher）
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
        // /tf_static 为 transient local（后连上的订阅者也能收到）。M5-13a：topicSettings 可以按话题改 QoS
        b.wallTime = profile.time == "wall";
        if (profile.clock) b.clock = b.node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
        const std::string jointTopic = b.mapped(profile.topicPrefix + "/joint_states");
        b.jointStates = b.node->create_publisher<sensor_msgs::msg::JointState>(jointTopic, b.qos(jointTopic, rclcpp::QoS(10).reliable()));
        b.tf = b.node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", b.qos("/tf", rclcpp::QoS(100).reliable()));
        b.tfStatic = b.node->create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", b.qos("/tf_static", rclcpp::QoS(1).reliable().transient_local()));
        const auto sensorQos = rclcpp::SensorDataQoS();
        const auto image = [&](const std::string& defaultName) {
            const std::string name = b.mapped(defaultName);
            return b.node->create_publisher<sensor_msgs::msg::Image>(name, b.qos(name, sensorQos));
        };
        const auto info = [&](const std::string& defaultName) {
            const std::string name = b.mapped(defaultName);
            return b.node->create_publisher<sensor_msgs::msg::CameraInfo>(name, b.qos(name, sensorQos));
        };
        if (profile.sensors) {
            for (std::size_t k = 0; k < d.sensors.size(); ++k) {
                const std::string name = b.mapped(b.topic(d.sensors[k].name) + "/imu");
                b.imus.push_back(b.node->create_publisher<sensor_msgs::msg::Imu>(name, b.qos(name, sensorQos, "imu")));
                const auto* ts = b.settings(name);
                b.imuAccelScale.push_back(ts != nullptr && ts->accelUnit == "g" ? 1.0 / kStandardGravity : 1.0);
                double rate = 1.0 / d.timestep;
                for (const auto& c : scene.sensors) {
                    if (c.sensor == d.sensors[k].id) rate = c.rateHz;
                }
                b.imuRates.push_back(rate);
            }
            for (const auto& l : scene.lidars) {
                const std::string name = b.mapped(b.topic(l.name) + "/scan");
                b.scans.push_back(b.node->create_publisher<sensor_msgs::msg::LaserScan>(name, b.qos(name, sensorQos)));
            }
            for (const auto& l : scene.lidars3d) {  // M5-11：<雷达名>/points（可用 topics 映射成真机的 /livox/lidar）
                const std::string name = b.mapped(b.topic(l.name) + "/points");
                b.clouds.push_back(b.node->create_publisher<sensor_msgs::msg::PointCloud2>(name, b.qos(name, sensorQos, "cloud")));
                const auto* ts = b.settings(name);
                b.cloudLayouts.push_back(pointLayoutFromName(ts != nullptr ? ts->pointLayout : "").value_or(PointLayout::Livox));
            }
            for (const auto& c : scene.cameras) {
                Impl::CameraTopics t;
                if (c.rgb) t.color = image(b.topic(c.name) + "/image_raw");
                if (c.thermal.enabled) {  // M5-06
                    t.thermalRaw = image(b.topic(c.name) + "/thermal/image_raw");
                    t.thermalColor = image(b.topic(c.name) + "/thermal/image_color");
                    t.thermalInfo = info(b.topic(c.name) + "/thermal/camera_info");
                }
                if (c.depth) t.depth = image(b.topic(c.name) + "/depth/image_raw");
                t.info = info(b.topic(c.name) + "/camera_info");
                if (c.segmentation) {  // M4-09
                    t.segmentation = image(b.topic(c.name) + "/segmentation/image_raw");
                    const std::string labelsTopic = b.mapped(b.topic(c.name) + "/segmentation/labels");
                    t.labels = b.node->create_publisher<std_msgs::msg::String>(labelsTopic, b.qos(labelsTopic, rclcpp::QoS(1).reliable().transient_local()));
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
            const std::string odomTopic = profile.topicPrefix + dd.odom;
            drive.odom = b.node->create_publisher<nav_msgs::msg::Odometry>(odomTopic, b.qos(odomTopic, rclcpp::QoS(10).reliable()));
            b.drive = std::move(drive);
            // 速度指令：档案的 cmdVel（M5-13a），没有时为差速底盘自己的 cmdVel 与超时
            b.subscribeInputs(profile.cmdVel.empty() ? std::vector<CommandInput>{{dd.cmdVel, "Twist", 0, dd.timeout}} : profile.cmdVel, Impl::Consumer::Drive);
        }
        // M5-13a：平面运动学底盘
        if (profile.planarBase) {
            const auto& pb = *profile.planarBase;
            const auto jointName = [](const td::JointDescription& j) -> const std::string& { return j.name; };
            const auto xj = b.unique(d.joints, pb.xJoint, jointName, "平面底盘的 x 关节", pb.optional);
            if (xj) {
                const auto yj = b.unique(d.joints, pb.yJoint, jointName, "平面底盘的 y 关节", false);
                const auto wj = b.unique(d.joints, pb.yawJoint, jointName, "平面底盘的 yaw 关节", false);
                if (d.joints[*xj].type != td::JointType::Slide || d.joints[*yj].type != td::JointType::Slide || d.joints[*wj].type != td::JointType::Hinge) {
                    throw std::runtime_error("平面底盘：x、y 应为滑动关节，yaw 应为转动关节（" + d.joints[*xj].name + "、" + d.joints[*yj].name + "、" +
                                             d.joints[*wj].name + "）");
                }
                const auto velocityActuator = [&](std::size_t joint) {
                    for (const auto& a : d.actuators) {
                        if (a.joint == d.joints[joint].id && a.kind == td::ActuatorKind::Velocity && a.inputs.size() == 1) return a.id;
                    }
                    throw std::runtime_error("平面底盘：关节 " + d.joints[joint].name + " 上没有速度执行器（MJCF 的 <velocity joint=...>）");
                };
                Impl::Planar planar;
                planar.x = *xj;
                planar.y = *yj;
                planar.yaw = *wj;
                planar.ax = velocityActuator(*xj);
                planar.ay = velocityActuator(*yj);
                planar.ayaw = velocityActuator(*wj);
                std::size_t base = d.joints[*wj].entity.value - 1;
                if (!pb.baseEntity.empty()) {
                    base = *b.unique(d.entities, pb.baseEntity, [](const td::EntityDescription& e) -> const std::string& { return e.name; },
                                     "平面底盘的机身实体", false);
                }
                planar.base = base;
                planar.baseFrame = b.frame(d.entities[base].name);
                if (!pb.odom.empty()) {
                    const std::string odomTopic = profile.topicPrefix + pb.odom;
                    planar.odom = b.node->create_publisher<nav_msgs::msg::Odometry>(odomTopic, b.qos(odomTopic, rclcpp::QoS(10).reliable()));
                }
                b.planar = std::move(planar);
                b.subscribeInputs(profile.cmdVel.empty() ? std::vector<CommandInput>{{"/cmd_vel", "Twist", 0, 0.5}} : profile.cmdVel,
                                  Impl::Consumer::Planar);
            }
        }
        // M5-13a：档案给了速度指令、但没有底盘时，给策略用（策略开启时取它）
        if (!profile.cmdVel.empty() && b.consumer == Impl::Consumer::None) b.subscribeInputs(profile.cmdVel, Impl::Consumer::Policy);
        // M5-13a：实体 TF 的方式；有底盘且发布 odom 时，机身的父坐标系是 odom
        b.robotTf = profile.tf.robot;
        if (b.robotTf == "auto") b.robotTf = b.drive || b.planar ? "tree" : "world";
        if (b.robotTf == "world" && profile.tf.odom && (b.drive || b.planar)) {
            throw std::runtime_error("实体 TF 为 world 时机身已有父坐标系 world，不能再发布 odom → 机身（tf.odom 设为 false，或 tf.robot 设为 tree）");
        }
        // M5-13a：真值
        if (profile.groundTruth) {
            const auto& g = *profile.groundTruth;
            Impl::Truth t;
            if (g.entity == "auto") {
                // 机器人的机身：平面底盘或差速底盘的机身，否则为第一个有自由关节的实体
                std::optional<std::size_t> base;
                if (b.planar) base = b.planar->base;
                else if (b.drive) base = b.drive->base;
                for (std::size_t j = 0; !base && j < d.joints.size(); ++j) {
                    if (d.joints[j].type == td::JointType::Free && d.joints[j].entity.valid()) base = d.joints[j].entity.value - 1;
                }
                if (!base) throw std::runtime_error("真值的实体为 auto，但场景中没有底盘，也没有带自由关节的实体");
                t.entity = *base;
            } else {
                t.entity = *b.unique(d.entities, g.entity, [](const td::EntityDescription& e) -> const std::string& { return e.name; }, "真值的实体", false);
            }
            t.childFrame = g.childFrame.empty() ? b.frame(d.entities[t.entity].name) : g.childFrame;
            const std::string name = profile.topicPrefix + g.topic;
            t.pub = b.node->create_publisher<nav_msgs::msg::Odometry>(name, b.qos(name, rclcpp::QoS(10).reliable()));
            b.truth = std::move(t);
        }
        // 话题名映射中的每一项都应是本场景的默认话题（防止拼写错误使映射不生效）；含 “*” 的应恰好匹配一个（M5-13a）
        for (const auto& [from, to] : profile.topics) {
            const bool used = hasGlob(from) ? b.patternUses.count(from) > 0 : b.defaultTopics.count(from) > 0;
            if (!used) {
                std::string available;
                for (const auto& t : b.defaultTopics) available += (available.empty() ? "" : "、") + t;
                throw std::runtime_error("通信配置 topics 中的“" + from + "”不是本场景的话题（可映射的：" + available + "）");
            }
            if (hasGlob(from) && b.patternUses[from].size() > 1) {
                std::string list;
                for (const auto& t : b.patternUses[from]) list += (list.empty() ? "" : "、") + t;
                throw std::runtime_error("通信配置 topics 中的“" + from + "”匹配了多个话题（" + list + "），它们会被映射到同一个名字");
            }
        }
        // M5-13a：topicSettings 的每一项都应匹配本场景的话题；IMU 单位只用于 IMU，点云布局只用于点云
        for (const auto& [name, t] : profile.topicSettings) {
            std::vector<std::string> kinds;
            for (const auto& [topic, kind] : b.createdTopics) {
                if (topic == name || (hasGlob(name) && globMatch(name, topic))) kinds.push_back(kind);
            }
            if (kinds.empty()) {
                std::string available;
                for (const auto& [topic, kind] : b.createdTopics) available += (available.empty() ? "" : "、") + topic;
                throw std::runtime_error("通信配置 topicSettings 中的“" + name + "”不是本场景的话题（有：" + available + "）");
            }
            for (const auto& kind : kinds) {
                if (!t.accelUnit.empty() && kind != "imu") throw std::runtime_error("通信配置 topicSettings 中的“" + name + "”：accelUnit 只用于 IMU 话题");
                if (!t.pointLayout.empty() && kind != "cloud") throw std::runtime_error("通信配置 topicSettings 中的“" + name + "”：pointLayout 只用于点云话题");
            }
        }
        // M5-13a：frames.rename 的每一项都应匹配本场景的坐标系
        for (const auto& [from, to] : profile.frameRename) {
            bool found = from == "world";
            const auto matches = [&](const std::string& n) { return n == from || (hasGlob(from) && globMatch(from, n)); };
            for (const auto& e : d.entities) found = found || matches(e.name);
            for (const auto& x : d.sensors) found = found || matches(x.name);
            for (const auto& x : scene.lidars) found = found || matches(x.name);
            for (const auto& x : scene.lidars3d) found = found || matches(x.name);
            for (const auto& x : scene.cameras) found = found || matches(x.name) || matches(x.name + "_optical");
            if (!found) throw std::runtime_error("通信配置 frames.rename 中的“" + from + "”不是本场景的坐标系（实体、传感器的全名）");
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
    // 速度指令（M5-13a：多路输入按优先级仲裁；超时停车）
    std::array<double, 3> command{0.0, 0.0, 0.0};
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (const auto c = impl_->arbiter.current(steadyNow())) command = *c;
    }
    // M5-13a：平面底盘——机身坐标系的速度换算为 x、y、yaw 三个关节的速度
    if (impl_->planar) {
        const auto& pl = *impl_->planar;
        const auto snap = impl_->scene.world->snapshot();
        const double yaw = pl.yaw < snap->joints.size() ? snap->joints[pl.yaw].position[0] : 0.0;
        const auto q = planarJointVelocities(command, yaw, impl_->profile.planarBase->limits);
        out.push_back({pl.ax, {q[0]}});
        out.push_back({pl.ay, {q[1]}});
        out.push_back({pl.ayaw, {q[2]}});
    }
    // M4-08b：差速底盘——最近一条 cmd_vel 换算为两轮角速度；超时（墙钟）停车，与 Create 3 相同
    if (impl_->profile.diffDrive && impl_->drive) {
        const auto& dd = *impl_->profile.diffDrive;
        const double v = command[0], w = command[2];
        const td::ActuatorId left = impl_->drive->left, right = impl_->drive->right;
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
    if (b.wallTime) b.wall.sync(snapshot.simulationTime, snapshot.runEpoch, wallNow());  // M5-13a
    const double now = snapshot.simulationTime;
    const auto stamp = b.stamp(now);
    if (b.clock) {
        rosgraph_msgs::msg::Clock msg;
        msg.clock = stamp;
        b.clock->publish(msg);
        ++b.published;
    }
    if (terra::simulation::sensorSampleDue(snapshot.stepIndex, dt, b.profile.jointStatesHz) && b.due(b.jointStates->get_topic_name(), now)) {
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
            drive.rightJoint < snapshot.joints.size() && b.due(drive.odom->get_topic_name(), now)) {
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
    // M5-13a：平面底盘的里程计（关节位置相对每次运行开始时；odom 坐标系为开始时的机身坐标系）
    std::optional<td::Pose> planarOdom;
    std::array<double, 3> planarTwist{0.0, 0.0, 0.0};
    if (b.planar) {
        auto& pl = *b.planar;
        const bool ok = pl.x < snapshot.joints.size() && pl.y < snapshot.joints.size() && pl.yaw < snapshot.joints.size();
        if (ok) {
            const double qx = snapshot.joints[pl.x].position[0], qy = snapshot.joints[pl.y].position[0], qw = snapshot.joints[pl.yaw].position[0];
            if (!pl.started || snapshot.runEpoch != pl.epoch || snapshot.stepIndex == 0) {
                pl.x0 = qx;
                pl.y0 = qy;
                pl.yaw0 = qw;
                pl.started = true;
                pl.epoch = snapshot.runEpoch;
            }
            const double c0 = std::cos(pl.yaw0), s0 = std::sin(pl.yaw0);
            const double dx = qx - pl.x0, dy = qy - pl.y0, yaw = qw - pl.yaw0;
            td::Pose pose;
            pose.position = {c0 * dx + s0 * dy, -s0 * dx + c0 * dy, 0.0};
            pose.orientation = {std::cos(yaw / 2.0), 0.0, 0.0, std::sin(yaw / 2.0)};
            planarOdom = pose;
            const double vx = snapshot.joints[pl.x].velocity[0], vy = snapshot.joints[pl.y].velocity[0];
            const double c = std::cos(qw), sn = std::sin(qw);
            planarTwist = {c * vx + sn * vy, -sn * vx + c * vy, snapshot.joints[pl.yaw].velocity[0]};
        }
        if (planarOdom && pl.odom && terra::simulation::sensorSampleDue(snapshot.stepIndex, dt, b.profile.planarBase->odomHz) &&
            b.due(pl.odom->get_topic_name(), now)) {
            nav_msgs::msg::Odometry odom;
            odom.header.stamp = stamp;
            odom.header.frame_id = b.profile.planarBase->odomFrame;
            odom.child_frame_id = pl.baseFrame;
            odom.pose.pose.position.x = planarOdom->position.x;
            odom.pose.pose.position.y = planarOdom->position.y;
            odom.pose.pose.orientation.w = planarOdom->orientation.w;
            odom.pose.pose.orientation.z = planarOdom->orientation.z;
            odom.twist.twist.linear.x = planarTwist[0];
            odom.twist.twist.linear.y = planarTwist[1];
            odom.twist.twist.angular.z = planarTwist[2];
            pl.odom->publish(odom);
            ++b.published;
        }
    }
    // M5-13a：真值（世界中的位姿；速度在实体坐标系中）
    if (b.truth && b.truth->entity < snapshot.entities.size() &&
        terra::simulation::sensorSampleDue(snapshot.stepIndex, dt, b.profile.groundTruth->rateHz) && b.due(b.truth->pub->get_topic_name(), now)) {
        const auto& e = truth.entities[b.truth->entity];  // 真值不加噪声
        const auto inv = terra::scene::conjugate(e.pose.orientation);
        const auto v = terra::scene::rotate(inv, e.linearVelocity);
        const auto w = terra::scene::rotate(inv, e.angularVelocity);
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = b.profile.groundTruth->frame;
        odom.child_frame_id = b.truth->childFrame;
        odom.pose.pose.position.x = e.pose.position.x;
        odom.pose.pose.position.y = e.pose.position.y;
        odom.pose.pose.position.z = e.pose.position.z;
        odom.pose.pose.orientation.w = e.pose.orientation.w;
        odom.pose.pose.orientation.x = e.pose.orientation.x;
        odom.pose.pose.orientation.y = e.pose.orientation.y;
        odom.pose.pose.orientation.z = e.pose.orientation.z;
        odom.twist.twist.linear.x = v.x;
        odom.twist.twist.linear.y = v.y;
        odom.twist.twist.linear.z = v.z;
        odom.twist.twist.angular.x = w.x;
        odom.twist.twist.angular.y = w.y;
        odom.twist.twist.angular.z = w.z;
        b.truth->pub->publish(odom);
        ++b.published;
    }
    if (terra::simulation::sensorSampleDue(snapshot.stepIndex, dt, b.profile.tfHz) && b.due("/tf", now)) {
        tf2_msgs::msg::TFMessage msg;
        const auto add = [&](const std::string& parent, const std::string& child, const geometry_msgs::msg::Transform& t) {
            geometry_msgs::msg::TransformStamped ts;
            ts.header.stamp = stamp;
            ts.header.frame_id = parent;
            ts.child_frame_id = child;
            ts.transform = t;
            msg.transforms.push_back(ts);
        };
        // 底盘的 odom → 机身（M4-08b 差速底盘、M5-13a 平面底盘），tf.odom 为 false 时由算法发布（如 SLAM）
        std::optional<std::size_t> odomChild;
        if (b.profile.tf.odom && b.drive) {
            const auto& drive = *b.drive;
            geometry_msgs::msg::Transform t;
            t.translation.x = drive.x;
            t.translation.y = drive.y;
            t.rotation.w = std::cos(drive.theta / 2.0);
            t.rotation.z = std::sin(drive.theta / 2.0);
            add(b.profile.diffDrive->odomFrame, drive.baseFrame, t);
            odomChild = drive.base;
        }
        if (b.profile.tf.odom && b.planar && planarOdom) {
            add(b.profile.planarBase->odomFrame, b.planar->baseFrame, transformOf(*planarOdom));
            odomChild = b.planar->base;
        }
        if (b.robotTf == "tree") {
            // 与真机相同：连杆之间为相对位姿（真机由 robot_state_publisher 发布）；根实体相对 world（机身已由 odom 给出时除外）
            for (std::size_t e = 0; e < d.entities.size() && e < snapshot.entities.size(); ++e) {
                const auto& entity = d.entities[e];
                if (entity.name.empty()) continue;
                if (!entity.parent.valid()) {
                    if (odomChild && *odomChild == e) continue;
                    if (b.drive) continue;  // M4-08b 的行为：差速底盘的 TF 树没有 world
                    add(b.frame("world"), b.frame(entity.name), transformOf(snapshot.entities[e].pose));
                    continue;
                }
                if (entity.parent.value - 1 >= snapshot.entities.size()) continue;
                add(b.entityFrame(entity.parent), b.frame(entity.name),
                    transformOf(terra::scene::multiply(terra::scene::inverse(snapshot.entities[entity.parent.value - 1].pose), snapshot.entities[e].pose)));
            }
        } else if (b.robotTf == "world") {
            for (std::size_t e = 0; e < d.entities.size() && e < snapshot.entities.size(); ++e) {
                if (d.entities[e].name.empty()) continue;
                add(b.frame("world"), b.frame(d.entities[e].name), transformOf(snapshot.entities[e].pose));
            }
        }
        if (!msg.transforms.empty()) {
            b.tf->publish(msg);
            ++b.published;
        }
    }
    for (std::size_t k = 0; k < b.imus.size() && k < snapshot.sensors.size(); ++k) {
        if (!terra::simulation::sensorSampleDue(snapshot.stepIndex, dt, b.imuRates[k]) || !b.due(b.imus[k]->get_topic_name(), now)) continue;
        const auto& v = snapshot.sensors[k].values;
        if (v.size() < 10) continue;
        sensor_msgs::msg::Imu msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = b.sensorFrame(d.sensors[k].name, d.sensors[k].entity, d.sensors[k].mount);
        msg.linear_acceleration.x = v[0] * b.imuAccelScale[k];  // M5-13a：单位可以是 g
        msg.linear_acceleration.y = v[1] * b.imuAccelScale[k];
        msg.linear_acceleration.z = v[2] * b.imuAccelScale[k];
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
        if (!terra::simulation::sensorSampleDue(snapshot.stepIndex, dt, lidar.rateHz) || !b.due(b.scans[k]->get_topic_name(), now)) continue;
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
    // M5-11：3D 激光雷达 → PointCloud2。默认与 livox_ros_driver2 的 PointCloud2 格式相同（M5-13a：可选 xyzi、velodyne、ouster，
    // 见 integration.hpp）；消息头的时刻为第一点的测量时刻
    for (std::size_t k = 0; k < b.clouds.size(); ++k) {
        const auto& lidar = b.scene.lidars3d[k];
        if (!terra::simulation::sensorSampleDue(snapshot.stepIndex, dt, lidar.rateHz) || !b.due(b.clouds[k]->get_topic_name(), now)) continue;
        auto scan = terra::simulation::scanLidar3d(*b.scene.world, lidar);
        b.noise.lidar3d(k, scan);
        const double header = b.outTime(std::max(0.0, scan.frameStart));
        auto packed = packPoints(b.cloudLayouts[k], scan, header);
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = stampOf(header);
        msg.header.frame_id = b.sensorFrame(lidar.name, lidar.entity, lidar.mount);
        for (const auto& f : packed.fields) {
            sensor_msgs::msg::PointField pf;
            pf.name = f.name;
            pf.offset = f.offset;
            pf.datatype = f.datatype;
            pf.count = 1;
            msg.fields.push_back(pf);
        }
        msg.point_step = packed.pointStep;
        msg.height = 1;
        msg.width = static_cast<std::uint32_t>(scan.points.size());
        msg.row_step = msg.point_step * msg.width;
        msg.is_bigendian = false;
        msg.is_dense = true;
        msg.data = std::move(packed.data);
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
    const auto stamp = b.stamp(image.time);
    if (topics.info && !b.due(topics.info->get_topic_name(), image.time)) return;  // M5-13a：降频（相机的话题一起降）
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
    if (topics.thermalRaw && !image.thermal.empty()) {  // M5-06
        const auto& spec = b.scene.cameras[camera].thermal;
        sensor_msgs::msg::Image raw;
        raw.header.stamp = stamp;
        raw.header.frame_id = frame;
        raw.width = w;
        raw.height = h;
        raw.is_bigendian = 0;
        raw.encoding = "mono16";
        raw.step = w * 2;
        raw.data.resize(image.thermal.size() * 2);
        for (std::size_t i = 0; i < image.thermal.size(); ++i) {
            const std::uint16_t v = terra::simulation::radiometricValue(static_cast<double>(image.thermal[i]), spec.rangeMin, spec.rangeMax);
            raw.data[2 * i] = static_cast<std::uint8_t>(v & 0xFFu);
            raw.data[2 * i + 1] = static_cast<std::uint8_t>(v >> 8);
        }
        topics.thermalRaw->publish(raw);
        ++b.published;
        const auto [lo, hi] = terra::simulation::thermalDisplayRange(spec, image.thermal);
        sensor_msgs::msg::Image color;
        color.header = raw.header;
        color.width = w;
        color.height = h;
        color.is_bigendian = 0;
        color.encoding = "rgb8";
        color.step = w * 3;
        color.data = terra::simulation::thermalColor(image.thermal, spec.palette, lo, hi);
        topics.thermalColor->publish(color);
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
    if (topics.thermalInfo && !image.thermal.empty()) {  // M5-06：与热像同一相机，内参相同
        topics.thermalInfo->publish(info);
        ++b.published;
    }
}

bool Bridge::enableVelocityCommand(const std::string& topic, double timeout) {
    Impl* self = impl_.get();
    // 速度指令已经给了底盘（差速、平面）：策略只取面板或命令行
    if (self->consumer == Impl::Consumer::Drive || self->consumer == Impl::Consumer::Planar) return false;
    if (self->consumer == Impl::Consumer::Policy) return true;  // M5-13a：档案的 cmdVel，或已经订阅过
    self->subscribeInputs({{topic, "Twist", 0, timeout}}, Impl::Consumer::Policy);
    return true;
}

std::optional<std::array<double, 3>> Bridge::velocityCommand() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->consumer != Impl::Consumer::Policy) return std::nullopt;
    return impl_->arbiter.current(steadyNow());
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
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (const auto i = impl_->arbiter.active(steadyNow())) s.commandSource = impl_->arbiter.inputs()[*i].topic;
    }
    std::size_t subscribers = 0;
    if (impl_->jointStates) subscribers += impl_->jointStates->get_subscription_count();
    if (impl_->tf) subscribers += impl_->tf->get_subscription_count();
    if (impl_->clock) subscribers += impl_->clock->get_subscription_count();
    s.subscribers = subscribers;
    return s;
}

const Profile& Bridge::profile() const noexcept { return impl_->profile; }

}  // namespace terra::ros2
