# terra_ros2：ROS2 桥接（M4-06，D-104：进程内的可选模块）。只有 TERRA_BUILD_ROS2=ON 且找到 ROS2 Humble 时构建（cmake/TerraRos2.cmake）。
# 公共头不含 ROS 类型；rclcpp 与消息包 PRIVATE 链接。查看器与 Headless 在 TERRA_ROS2_ENABLED 时链接本库并定义 TERRA_WITH_ROS2。
# M4-08d：ROS2 监视器（monitor.cpp，ROS2 面板用）与共用的 rclcpp 运行时（runtime.cpp）
add_library(terra_ros2 STATIC
    src/bridge.cpp
    src/profile.cpp
    src/runtime.cpp
    src/monitor.cpp)
add_library(terra::ros2 ALIAS terra_ros2)
target_include_directories(terra_ros2 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(terra_ros2
    PUBLIC terra::scene terra::simulation terra::data terra::physics
    PRIVATE terra::json rclcpp::rclcpp ${builtin_interfaces_TARGETS} ${rosgraph_msgs_TARGETS} ${std_msgs_TARGETS}
            ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS} ${tf2_msgs_TARGETS} ${std_srvs_TARGETS} ${nav_msgs_TARGETS})
terra_configure_target(terra_ros2)
# introspection 的头文件（运行时展开任意消息；类型库在运行时按话题类型加载）
if(TARGET rosidl_typesupport_introspection_cpp::rosidl_typesupport_introspection_cpp)
    target_link_libraries(terra_ros2 PRIVATE rosidl_typesupport_introspection_cpp::rosidl_typesupport_introspection_cpp)
else()
    target_include_directories(terra_ros2 SYSTEM PRIVATE ${rosidl_typesupport_introspection_cpp_INCLUDE_DIRS})
endif()
# M4-06b：宇树配置（找到 unitree_go、unitree_hg 时，cmake/TerraRos2.cmake 设 TERRA_ROS2_UNITREE）
if(TERRA_ROS2_UNITREE)
    target_sources(terra_ros2 PRIVATE src/unitree.cpp)
    target_link_libraries(terra_ros2 PRIVATE ${unitree_go_TARGETS} ${unitree_hg_TARGETS})
    target_compile_definitions(terra_ros2 PRIVATE TERRA_ROS2_UNITREE_ENABLED=1)
endif()
