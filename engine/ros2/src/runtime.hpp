#pragma once

// 进程内共用的 rclcpp 运行时（M4-08d 从 bridge.cpp 移出）：桥接与 ROS2 监视器都可能存在，
// 第一个使用者初始化 rclcpp，最后一个释放时关闭。不接管信号（查看器与 Headless 自己处理 Ctrl+C）。

namespace terra::ros2 {

void acquireRclcpp();
void releaseRclcpp();

}  // namespace terra::ros2
