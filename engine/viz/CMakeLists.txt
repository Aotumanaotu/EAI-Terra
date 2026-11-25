# terra_viz：显示配置（M4-08d）。.terviz 的读写（保留不认识的内容）、导出器接口与注册表、.terviz 与 .rviz 两个导出器。
# 不依赖 ROS2，也不依赖界面：没有 ROS2 的构建同样可以读写与导出（导出 .rviz 时由查看器提供话题类型等环境信息）。
add_library(terra_viz STATIC
    src/viz_config.cpp
    src/exporter.cpp)
add_library(terra::viz ALIAS terra_viz)
target_include_directories(terra_viz PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(terra_viz PRIVATE terra::json)
terra_configure_target(terra_viz)
