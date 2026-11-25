// 进程内共用的 rclcpp 运行时（M4-08d）。

#include "runtime.hpp"

#include <mutex>

#include <rclcpp/rclcpp.hpp>

namespace terra::ros2 {

namespace {
std::mutex initMutex;
int users = 0;
}  // namespace

void acquireRclcpp() {
    std::lock_guard<std::mutex> lock(initMutex);
    if (users++ == 0 && !rclcpp::ok()) {
        rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    }
}

void releaseRclcpp() {
    std::lock_guard<std::mutex> lock(initMutex);
    if (--users == 0 && rclcpp::ok()) rclcpp::shutdown();
}

}  // namespace terra::ros2
