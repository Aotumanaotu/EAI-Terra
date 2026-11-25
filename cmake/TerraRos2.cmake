# ROS2 桥接的依赖（M4-06，D-014 Humble、D-104 进程内的可选模块）。
# 在已 source ROS2 setup.bash 的环境中查找所需的包；缺少时给出提示并关闭 ROS2 桥接，不让配置失败（用户要求：提醒安装对应版本）。
# 结果：TERRA_ROS2_ENABLED（ON / OFF）；TERRA_ROS2_UNITREE（找到宇树消息包 unitree_go、unitree_hg 时为 ON，M4-06b 使用）。
set(_terra_ros2_packages rclcpp builtin_interfaces rosgraph_msgs std_msgs geometry_msgs sensor_msgs tf2_msgs std_srvs nav_msgs
    rosidl_typesupport_introspection_cpp)  # nav_msgs：M4-08b 差速底盘的 /odom；introspection：M4-08d 运行时展开任意消息
set(_terra_ros2_missing "")
foreach(_pkg IN LISTS _terra_ros2_packages)
    find_package(${_pkg} QUIET)
    if(NOT ${_pkg}_FOUND)
        list(APPEND _terra_ros2_missing ${_pkg})
    endif()
endforeach()
if(_terra_ros2_missing)
    message(WARNING
        "TERRA_BUILD_ROS2=ON，但没有找到 ROS2 Humble 的包：${_terra_ros2_missing}。本次不构建 ROS2 桥接。\n"
        "请安装 ROS2 Humble（ros-humble-ros-base 或 ros-humble-desktop，说明：https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html），"
        "并在配置前执行 source /opt/ros/humble/setup.bash。")
    set(TERRA_ROS2_ENABLED OFF)
else()
    set(TERRA_ROS2_ENABLED ON)
    if(NOT "$ENV{ROS_DISTRO}" STREQUAL "humble")
        message(WARNING "ROS_DISTRO 为“$ENV{ROS_DISTRO}”，Terra 按 ROS2 Humble 开发与验证（D-014）")
    endif()
    find_package(unitree_go QUIET)
    find_package(unitree_hg QUIET)
    if(unitree_go_FOUND AND unitree_hg_FOUND)
        set(TERRA_ROS2_UNITREE ON)
    else()
        set(TERRA_ROS2_UNITREE OFF)
    endif()
    message(STATUS "ROS2 桥接：开启（ROS_DISTRO=$ENV{ROS_DISTRO}；宇树消息包：${TERRA_ROS2_UNITREE}）")
endif()
