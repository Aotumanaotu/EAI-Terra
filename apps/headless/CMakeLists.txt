# terra_headless：纯命令行入口。
# 约束：只链接 Terra 目标；不得包含 MuJoCo 头文件，不得链接图形或窗口库。

# 参数解析与校验，只依赖标准库，供入口和单元测试共用。
add_library(terra_headless_cli STATIC
    cli.cpp)
target_include_directories(terra_headless_cli PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(terra_headless_cli PUBLIC terra::data)
terra_configure_target(terra_headless_cli)

add_executable(terra_headless
    main.cpp)
target_link_libraries(terra_headless PRIVATE
    terra_headless_cli terra::core terra::physics terra::physics_mujoco terra::scene terra::simulation terra::library)
terra_configure_target(terra_headless)
# M4-06a：ROS2 桥接（只在 TERRA_BUILD_ROS2=ON 且找到 ROS2 时）。Headless 仍不链接任何图形库
if(TERRA_ROS2_ENABLED)
    target_link_libraries(terra_headless PRIVATE terra::ros2)
    target_compile_definitions(terra_headless PRIVATE TERRA_WITH_ROS2=1)
endif()
