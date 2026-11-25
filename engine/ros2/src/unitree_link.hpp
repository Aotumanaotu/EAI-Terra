#pragma once

// 宇树配置的话题（M4-06b，内部头文件，只在 engine/ros2/src 与测试中使用）。只依赖 rclcpp 与 Terra 类型；
// 宇树消息包的类型在 unitree.cpp 与 unitree_messages.hpp 中。

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "terra/ros2/bridge.hpp"

namespace terra::ros2::detail {

class UnitreeLink {
public:
    virtual ~UnitreeLink() = default;
    // 每步之后（仿真线程）：按频率发布 LowState 等
    virtual void publish(const terra::data::PhysicsSnapshot& snapshot) = 0;
    // 每步之前（仿真线程）：各电机的控制量（收到第一条 LowCmd 之前为 0）
    virtual std::vector<terra::physics::ActuatorCommand> commands(const terra::data::PhysicsSnapshot& snapshot) = 0;
};

// 收到的 LowCmd 计入桥接的统计：accepted 为 false 时 reason 为拒绝的原因
using Account = std::function<void(bool accepted, const std::string& reason)>;

// 按配置与场景建立话题；场景中找不到配置里的实例、关节、执行器或 IMU 时返回空，error 为原因
std::unique_ptr<UnitreeLink> createUnitreeLink(rclcpp::Node& node, const terra::scene::SceneInstance& scene, const Profile& profile,
                                               Account account, std::string& error);

}  // namespace terra::ros2::detail
