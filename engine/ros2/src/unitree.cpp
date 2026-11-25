// 宇树配置（M4-06b，M4-05 设计稿第 4 节，2026-11-24 评审通过）：与真机相同的低层话题与消息。
// 只有找到宇树消息包（unitree_go、unitree_hg，用户按固定提交 668d1ec5 编译 unitree_ros2）时才编译本文件。
//
// 话题（名字前加配置的 topicPrefix，默认为空；QoS 为 reliable、深度 10，与官方示例的默认 QoS 相同；/lowcmd 用 best effort
// 订阅，可靠与尽力而为的发布者都能匹配）：
//   /lowstate、/lf/lowstate      LowState（go：unitree_go；hg：unitree_hg），含 CRC
//   /sportmodestate、/lf/sportmodestate、/wirelesscontroller   仅 go
//   /lowcmd（订阅）              LowCmd；CRC 不对的拒绝（保持上一条），与真机相同
// 控制：每一步对每个电机按最近一条 LowCmd 计算 τ = tau + kp (q − q̂) + kd (dq − dq̂)（mode = 0 时为 0；与 unitree_mujoco 相同），
//   控制量 = τ /（增益 × 传动比），截断到执行器的控制范围。收到第一条 LowCmd 之前各电机为 0（场景的 PD 层让位）。

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <optional>
#include <type_traits>
#include <utility>

#include <unitree_go/msg/sport_mode_state.hpp>
#include <unitree_go/msg/wireless_controller.hpp>

#include "terra/simulation/motion.hpp"
#include "terra/simulation/sensor_sampling.hpp"
#include "unitree_link.hpp"
#include "unitree_messages.hpp"

namespace terra::ros2::detail {

namespace {

namespace td = terra::data;

// ---- 官方结构体的字节布局：按字段顺序、自然对齐写入，填充字节为 0 ----

class Packer {
public:
    void align(std::size_t n) {
        while (bytes_.size() % n != 0) bytes_.push_back(0);
    }
    template <class T>
    void put(T value) {
        static_assert(std::is_arithmetic_v<T>, "只写算术类型");
        align(alignof(T));
        std::uint8_t raw[sizeof(T)];
        std::memcpy(raw, &value, sizeof(T));
        bytes_.insert(bytes_.end(), raw, raw + sizeof(T));
    }
    template <class T, std::size_t N>
    void put(const std::array<T, N>& values) {
        for (const T& v : values) put(v);
    }
    std::vector<std::uint8_t> take() { return std::move(bytes_); }

private:
    std::vector<std::uint8_t> bytes_;
};

// 结构体：开头与结尾都按结构体的对齐（最大成员的对齐）补齐
template <class Body>
void structure(Packer& p, std::size_t alignment, Body&& body) {
    p.align(alignment);
    body();
    p.align(alignment);
}

template <class Imu>
void packImu(Packer& p, const Imu& imu) {
    structure(p, 4, [&] {
        p.put(imu.quaternion);
        p.put(imu.gyroscope);
        p.put(imu.accelerometer);
        p.put(imu.rpy);
        p.put(imu.temperature);  // go：int8；hg：int16
    });
}

}  // namespace

std::uint32_t crc32Core(const std::uint32_t* words, std::size_t count) {
    std::uint32_t crc = 0xFFFFFFFFu;
    const std::uint32_t polynomial = 0x04C11DB7u;
    for (std::size_t i = 0; i < count; ++i) {
        std::uint32_t bit = 1u << 31;
        const std::uint32_t data = words[i];
        for (int b = 0; b < 32; ++b) {
            if ((crc & 0x80000000u) != 0) {
                crc <<= 1;
                crc ^= polynomial;
            } else {
                crc <<= 1;
            }
            if ((data & bit) != 0) crc ^= polynomial;
            bit >>= 1;
        }
    }
    return crc;
}

std::uint32_t crcOfPacked(const std::vector<std::uint8_t>& bytes) {
    const std::size_t count = bytes.size() / 4;
    std::vector<std::uint32_t> words(count);
    if (count > 0) std::memcpy(words.data(), bytes.data(), count * 4);
    return crc32Core(words.data(), count > 0 ? count - 1 : 0);
}

std::vector<std::uint8_t> pack(const unitree_go::msg::LowCmd& m) {
    Packer p;
    structure(p, 4, [&] {
        p.put(m.head);
        p.put(m.level_flag);
        p.put(m.frame_reserve);
        p.put(m.sn);
        p.put(m.version);
        p.put(m.bandwidth);
        for (const auto& c : m.motor_cmd) {
            structure(p, 4, [&] {
                p.put(c.mode);
                p.put(c.q);
                p.put(c.dq);
                p.put(c.tau);
                p.put(c.kp);
                p.put(c.kd);
                p.put(c.reserve);
            });
        }
        structure(p, 1, [&] {
            p.put(m.bms_cmd.off);
            p.put(m.bms_cmd.reserve);
        });
        p.put(m.wireless_remote);
        p.put(m.led);
        p.put(m.fan);
        p.put(m.gpio);
        p.put(m.reserve);
        p.put(m.crc);
    });
    return p.take();
}

std::vector<std::uint8_t> pack(const unitree_go::msg::LowState& m) {
    Packer p;
    structure(p, 4, [&] {
        p.put(m.head);
        p.put(m.level_flag);
        p.put(m.frame_reserve);
        p.put(m.sn);
        p.put(m.version);
        p.put(m.bandwidth);
        packImu(p, m.imu_state);
        for (const auto& s : m.motor_state) {
            structure(p, 4, [&] {
                p.put(s.mode);
                p.put(s.q);
                p.put(s.dq);
                p.put(s.ddq);
                p.put(s.tau_est);
                p.put(s.q_raw);
                p.put(s.dq_raw);
                p.put(s.ddq_raw);
                p.put(s.temperature);
                p.put(s.lost);
                p.put(s.reserve);
            });
        }
        structure(p, 4, [&] {
            const auto& b = m.bms_state;
            p.put(b.version_high);
            p.put(b.version_low);
            p.put(b.status);
            p.put(b.soc);
            p.put(b.current);
            p.put(b.cycle);
            p.put(b.bq_ntc);
            p.put(b.mcu_ntc);
            p.put(b.cell_vol);
        });
        p.put(m.foot_force);
        p.put(m.foot_force_est);
        p.put(m.tick);
        p.put(m.wireless_remote);
        p.put(m.bit_flag);
        p.put(m.adc_reel);
        p.put(m.temperature_ntc1);
        p.put(m.temperature_ntc2);
        p.put(m.power_v);
        p.put(m.power_a);
        p.put(m.fan_frequency);
        p.put(m.reserve);
        p.put(m.crc);
    });
    return p.take();
}

std::vector<std::uint8_t> pack(const unitree_hg::msg::LowCmd& m) {
    Packer p;
    structure(p, 4, [&] {
        p.put(m.mode_pr);
        p.put(m.mode_machine);
        for (const auto& c : m.motor_cmd) {
            structure(p, 4, [&] {
                p.put(c.mode);
                p.put(c.q);
                p.put(c.dq);
                p.put(c.tau);
                p.put(c.kp);
                p.put(c.kd);
                p.put(c.reserve);
            });
        }
        // 官方 get_crc（motor_crc_hg.cpp）只拷贝 reserve 的第一个元素（memcpy 4 字节），其余按 0
        p.put(m.reserve[0]);
        p.put(std::uint32_t{0});
        p.put(std::uint32_t{0});
        p.put(std::uint32_t{0});
        p.put(m.crc);
    });
    return p.take();
}

std::vector<std::uint8_t> pack(const unitree_hg::msg::LowState& m) {
    Packer p;
    structure(p, 4, [&] {
        p.put(m.version);
        p.put(m.mode_pr);
        p.put(m.mode_machine);
        p.put(m.tick);
        packImu(p, m.imu_state);
        for (const auto& s : m.motor_state) {
            structure(p, 4, [&] {
                p.put(s.mode);
                p.put(s.q);
                p.put(s.dq);
                p.put(s.ddq);
                p.put(s.tau_est);
                p.put(s.temperature);
                p.put(s.vol);
                p.put(s.sensor);
                p.put(s.motorstate);
                p.put(s.reserve);
            });
        }
        p.put(m.wireless_remote);
        p.put(m.reserve);
        p.put(m.crc);
    });
    return p.take();
}

namespace {

// ---- 话题 ----

struct MotorBinding {
    std::size_t slot = 0;      // 电机编号
    std::size_t joint = 0;     // 关节下标（描述与快照相同）
    std::size_t actuator = 0;  // 执行器下标
    td::ActuatorId actuatorId;
    double scale = 1.0;        // 控制量 = τ / scale（增益 × 传动比）
    std::optional<std::pair<double, double>> range;
};

struct MotorCommand {
    std::uint8_t mode = 0;
    double q = 0.0, dq = 0.0, tau = 0.0, kp = 0.0, kd = 0.0;
};

std::array<float, 3> rpyOf(double w, double x, double y, double z) {
    const double roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    const double pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
    const double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return {static_cast<float>(roll), static_cast<float>(pitch), static_cast<float>(yaw)};
}

template <class Imu>
void fillImu(Imu& imu, const std::vector<double>& v) {
    if (v.size() < 10) return;
    for (std::size_t i = 0; i < 4; ++i) imu.quaternion[i] = static_cast<float>(v[6 + i]);  // w、x、y、z
    for (std::size_t i = 0; i < 3; ++i) {
        imu.gyroscope[i] = static_cast<float>(v[3 + i]);
        imu.accelerometer[i] = static_cast<float>(v[i]);
    }
    imu.rpy = rpyOf(v[6], v[7], v[8], v[9]);
    imu.temperature = 30;  // 仿真值
}

std::int16_t clampInt16(double v) {
    if (!std::isfinite(v)) return 0;
    return static_cast<std::int16_t>(std::clamp(std::lround(v), -32768L, 32767L));
}

// 两种消息的差别
struct GoTraits {
    using LowState = unitree_go::msg::LowState;
    using LowCmd = unitree_go::msg::LowCmd;
    static constexpr std::size_t kSlots = 20;
};
struct HgTraits {
    using LowState = unitree_hg::msg::LowState;
    using LowCmd = unitree_hg::msg::LowCmd;
    static constexpr std::size_t kSlots = 35;
};

template <class Traits>
class Link final : public UnitreeLink {
public:
    Link(rclcpp::Node& node, const terra::scene::SceneInstance& scene, UnitreeProfile profile, const std::string& prefix,
         std::vector<MotorBinding> motors, std::size_t imu, std::vector<td::EntityId> feet, td::EntityId base, Account account)
        : scene_(scene), profile_(std::move(profile)), motors_(std::move(motors)), imu_(imu), feet_(std::move(feet)), base_(base),
          account_(std::move(account)) {
        const auto qos = rclcpp::QoS(10).reliable();
        lowstate_ = node.create_publisher<typename Traits::LowState>(prefix + "/lowstate", qos);
        if (profile_.lfLowstateHz > 0) lfLowstate_ = node.create_publisher<typename Traits::LowState>(prefix + "/lf/lowstate", qos);
        if constexpr (std::is_same_v<Traits, GoTraits>) {
            if (profile_.sportModeStateHz > 0) {
                sport_ = node.create_publisher<unitree_go::msg::SportModeState>(prefix + "/sportmodestate", qos);
            }
            if (profile_.lfSportModeStateHz > 0) {
                lfSport_ = node.create_publisher<unitree_go::msg::SportModeState>(prefix + "/lf/sportmodestate", qos);
            }
            if (profile_.wirelessControllerHz > 0) {
                wireless_ = node.create_publisher<unitree_go::msg::WirelessController>(prefix + "/wirelesscontroller", qos);
            }
        }
        commands_.resize(Traits::kSlots);
        lowcmd_ = node.create_subscription<typename Traits::LowCmd>(
            prefix + "/lowcmd", rclcpp::QoS(10).best_effort(), [this](const typename Traits::LowCmd::SharedPtr msg) { onLowCmd(*msg); });
    }

    void publish(const td::PhysicsSnapshot& snapshot) override {
        const double dt = snapshot.fixedTimestep;
        const auto due = [&](double hz) { return hz > 0 && terra::simulation::sensorSampleDue(snapshot.stepIndex, dt, hz); };
        const bool high = due(profile_.lowstateHz);
        const bool low = lfLowstate_ && due(profile_.lfLowstateHz);
        if (high || low) {
            const auto state = lowState(snapshot);
            if (high) lowstate_->publish(state);
            if (low) lfLowstate_->publish(state);
        }
        if constexpr (std::is_same_v<Traits, GoTraits>) {
            const bool sportHigh = sport_ && due(profile_.sportModeStateHz);
            const bool sportLow = lfSport_ && due(profile_.lfSportModeStateHz);
            if (sportHigh || sportLow) {
                const auto state = sportState(snapshot);
                if (sportHigh) sport_->publish(state);
                if (sportLow) lfSport_->publish(state);
            }
            if (wireless_ && due(profile_.wirelessControllerHz)) wireless_->publish(unitree_go::msg::WirelessController{});
        }
    }

    std::vector<terra::physics::ActuatorCommand> commands(const td::PhysicsSnapshot& snapshot) override {
        std::vector<MotorCommand> latest;
        bool received = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest = commands_;
            received = received_;
        }
        std::vector<terra::physics::ActuatorCommand> out;
        out.reserve(motors_.size());
        for (const auto& m : motors_) {
            double torque = 0.0;
            if (received && m.joint < snapshot.joints.size()) {
                const MotorCommand& c = latest[m.slot];
                const auto& js = snapshot.joints[m.joint];
                if (c.mode != 0) torque = c.tau + c.kp * (c.q - js.position[0]) + c.kd * (c.dq - js.velocity[0]);
            }
            double control = std::isfinite(torque) ? torque / m.scale : 0.0;
            if (m.range) control = std::clamp(control, m.range->first, m.range->second);
            out.push_back({m.actuatorId, {control}});
        }
        return out;
    }

private:
    typename Traits::LowState lowState(const td::PhysicsSnapshot& s) {
        typename Traits::LowState msg;
        if (imu_ < s.sensors.size()) fillImu(msg.imu_state, s.sensors[imu_].values);
        for (const auto& m : motors_) {
            auto& ms = msg.motor_state[m.slot];
            ms.mode = 0x01;
            if (m.joint < s.joints.size()) {
                ms.q = static_cast<float>(s.joints[m.joint].position[0]);
                ms.dq = static_cast<float>(s.joints[m.joint].velocity[0]);
            }
            // 关节力矩：执行器输出 × 传动比
            if (m.actuator < s.actuators.size()) {
                ms.tau_est = static_cast<float>(s.actuators[m.actuator].force * scene_.world->description().actuators[m.actuator].gear);
            }
        }
        const auto tick = static_cast<std::uint32_t>(std::llround(s.simulationTime * 1000.0) & 0xFFFFFFFFLL);
        msg.tick = tick;
        if constexpr (std::is_same_v<Traits, GoTraits>) {
            msg.head = {0xFE, 0xEF};
            msg.level_flag = 0xFF;
            for (auto& ms : msg.motor_state) ms.temperature = 30;
            const auto feet = footForces(s);
            msg.foot_force = feet;
            msg.foot_force_est = feet;
            // 电池与电源：仿真值
            msg.bms_state.version_high = 1;
            msg.bms_state.soc = 100;
            for (auto& v : msg.bms_state.cell_vol) v = 3700;
            msg.power_v = 28.8f;
            msg.temperature_ntc1 = 30;
            msg.temperature_ntc2 = 30;
        } else {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                msg.mode_pr = modePr_;
            }
            msg.mode_machine = static_cast<std::uint8_t>(profile_.modeMachine);
            for (auto& ms : msg.motor_state) {
                ms.temperature = {30, 30};
                ms.vol = 48.0f;
            }
        }
        msg.crc = crcOf(msg);
        return msg;
    }

    std::array<std::int16_t, 4> footForces(const td::PhysicsSnapshot& s) const {
        std::array<std::int16_t, 4> out{};
        for (std::size_t k = 0; k < feet_.size() && k < 4; ++k) {
            double force = 0.0;
            for (const auto& pair : s.contacts.pairs) {
                if (pair.entityA == feet_[k] || pair.entityB == feet_[k]) force += pair.normalForce;
            }
            out[k] = clampInt16(force);  // 接触法向力（N）；真机为传感器原始读数，单位不同
        }
        return out;
    }

    unitree_go::msg::SportModeState sportState(const td::PhysicsSnapshot& s) const {
        unitree_go::msg::SportModeState msg;
        const double whole = std::floor(s.simulationTime);
        msg.stamp.sec = static_cast<std::int32_t>(whole);
        msg.stamp.nanosec = static_cast<std::uint32_t>(std::min(999999999.0, std::round((s.simulationTime - whole) * 1e9)));
        if (imu_ < s.sensors.size()) fillImu(msg.imu_state, s.sensors[imu_].values);
        if (base_.valid() && base_.value - 1 < s.entities.size()) {
            const auto& e = s.entities[base_.value - 1];
            msg.position = {static_cast<float>(e.pose.position.x), static_cast<float>(e.pose.position.y),
                            static_cast<float>(e.pose.position.z)};
            msg.body_height = static_cast<float>(e.pose.position.z);
            msg.velocity = {static_cast<float>(e.linearVelocity.x), static_cast<float>(e.linearVelocity.y),
                            static_cast<float>(e.linearVelocity.z)};
            msg.yaw_speed = static_cast<float>(e.angularVelocity.z);
        }
        msg.foot_force = footForces(s);
        return msg;
    }

    // 执行器线程：校验 CRC 与数值，保存为最近一条命令
    void onLowCmd(const typename Traits::LowCmd& msg) {
        if (profile_.checkCrc) {
            const std::uint32_t expected = crcOf(msg);
            if (expected != msg.crc) {
                char text[160];
                std::snprintf(text, sizeof(text), "LowCmd 的 CRC 不对（收到 0x%08X，应为 0x%08X），已拒绝", static_cast<unsigned>(msg.crc),
                              static_cast<unsigned>(expected));
                account_(false, text);
                return;
            }
        }
        std::vector<MotorCommand> next(Traits::kSlots);
        for (const auto& m : motors_) {
            const auto& c = msg.motor_cmd[m.slot];
            MotorCommand mc{c.mode, c.q, c.dq, c.tau, c.kp, c.kd};
            const bool finite = std::isfinite(mc.tau) && std::isfinite(mc.kp) && std::isfinite(mc.kd) &&
                                (mc.kp == 0.0 || std::isfinite(mc.q)) && (mc.kd == 0.0 || std::isfinite(mc.dq));
            if (mc.mode != 0 && !finite) {
                account_(false, "LowCmd 中电机 " + std::to_string(m.slot) + " 的命令含非有限值，已拒绝");
                return;
            }
            next[m.slot] = mc;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            commands_ = std::move(next);
            received_ = true;
            if constexpr (std::is_same_v<Traits, HgTraits>) modePr_ = msg.mode_pr;
        }
        account_(true, "");
    }

    const terra::scene::SceneInstance& scene_;
    UnitreeProfile profile_;
    std::vector<MotorBinding> motors_;
    std::size_t imu_;
    std::vector<td::EntityId> feet_;
    td::EntityId base_;
    Account account_;
    typename rclcpp::Publisher<typename Traits::LowState>::SharedPtr lowstate_;
    typename rclcpp::Publisher<typename Traits::LowState>::SharedPtr lfLowstate_;
    rclcpp::Publisher<unitree_go::msg::SportModeState>::SharedPtr sport_;
    rclcpp::Publisher<unitree_go::msg::SportModeState>::SharedPtr lfSport_;
    rclcpp::Publisher<unitree_go::msg::WirelessController>::SharedPtr wireless_;
    typename rclcpp::Subscription<typename Traits::LowCmd>::SharedPtr lowcmd_;
    mutable std::mutex mutex_;  // 保护以下
    std::vector<MotorCommand> commands_;
    bool received_ = false;
    std::uint8_t modePr_ = 0;
};

}  // namespace

std::unique_ptr<UnitreeLink> createUnitreeLink(rclcpp::Node& node, const terra::scene::SceneInstance& scene, const Profile& profile,
                                               Account account, std::string& error) {
    if (!profile.unitree) {
        error = "宇树配置缺少 unitree 字段";
        return nullptr;
    }
    const UnitreeProfile& u = *profile.unitree;
    const auto& d = scene.world->description();
    const std::string prefix = u.instance + "/";
    const bool go = u.messages == "go";

    // IMU
    const auto* imu = d.findSensor(prefix + u.imu);
    if (imu == nullptr) {
        error = "宇树配置：场景中没有 IMU“" + prefix + u.imu + "”（MJCF 中的 IMU，或在场景文件 sensors 中添加）";
        return nullptr;
    }
    const auto imuIndex = static_cast<std::size_t>(imu - d.sensors.data());

    // 电机 → 关节 → 力矩执行器
    std::vector<MotorBinding> motors;
    for (std::size_t slot = 0; slot < u.motors.size(); ++slot) {
        if (u.motors[slot].empty()) continue;
        const std::string name = prefix + u.motors[slot];
        const auto* joint = d.findJoint(name);
        if (joint == nullptr || (joint->type != td::JointType::Hinge && joint->type != td::JointType::Slide)) {
            error = "宇树配置：电机 " + std::to_string(slot) + " 的关节“" + name + "”不存在，或不是转动 / 滑动关节";
            return nullptr;
        }
        const td::ActuatorDescription* motor = nullptr;
        const td::ActuatorDescription* other = nullptr;
        for (const auto& a : d.actuators) {
            if (a.transmission != td::ActuatorTransmission::Joint || a.joint != joint->id) continue;
            if (a.kind == td::ActuatorKind::Motor && motor == nullptr) motor = &a;
            else if (other == nullptr) other = &a;
        }
        if (motor == nullptr) {
            error = "宇树配置：关节“" + name + "”上没有力矩执行器" +
                    (other != nullptr && other->kind == td::ActuatorKind::Position
                         ? "（“" + other->name + "”是位置执行器；宇树低层控制需要力矩执行器，请在场景文件的模型实例中加 \"torqueActuators\": true）"
                         : std::string{});
            return nullptr;
        }
        MotorBinding b;
        b.slot = slot;
        b.joint = static_cast<std::size_t>(joint - d.joints.data());
        b.actuator = static_cast<std::size_t>(motor - d.actuators.data());
        b.actuatorId = motor->id;
        b.scale = motor->gain * motor->gear;
        b.range = terra::simulation::controlRange(*motor);
        if (!(std::fabs(b.scale) > 0.0) || !std::isfinite(b.scale)) {
            error = "宇树配置：执行器“" + motor->name + "”的增益 × 传动比为 0";
            return nullptr;
        }
        for (const auto& other2 : motors) {
            if (other2.joint == b.joint) {
                error = "宇树配置：关节“" + name + "”在电机表中出现了两次";
                return nullptr;
            }
        }
        motors.push_back(b);
    }

    // 足端与机身（go）
    std::vector<td::EntityId> feet;
    for (const auto& f : u.feet) {
        const auto* e = d.findEntity(prefix + f);
        if (e == nullptr) {
            error = "宇树配置：足端实体“" + prefix + f + "”不存在";
            return nullptr;
        }
        feet.push_back(e->id);
    }
    td::EntityId base;
    if (!u.base.empty()) {
        const auto* e = d.findEntity(prefix + u.base);
        if (e == nullptr) {
            error = "宇树配置：机身实体“" + prefix + u.base + "”不存在";
            return nullptr;
        }
        base = e->id;
    } else if (go && (u.sportModeStateHz > 0 || u.lfSportModeStateHz > 0)) {
        error = "宇树配置：发布 sportmodestate 需要机身实体 base";
        return nullptr;
    }

    if (go) {
        return std::make_unique<Link<GoTraits>>(node, scene, u, profile.topicPrefix, std::move(motors), imuIndex, std::move(feet), base,
                                                std::move(account));
    }
    return std::make_unique<Link<HgTraits>>(node, scene, u, profile.topicPrefix, std::move(motors), imuIndex, std::move(feet), base,
                                            std::move(account));
}

}  // namespace terra::ros2::detail
