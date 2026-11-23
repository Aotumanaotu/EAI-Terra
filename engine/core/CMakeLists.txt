# terra_data：Terra 数据契约（ID、几何类型、模型描述），仅头文件。
# 不依赖 Core 运行实现，供 Core、Physics 及其消费者共用，避免 Core ↔ Physics 循环依赖。
add_library(terra_data INTERFACE)
add_library(terra::data ALIAS terra_data)
target_include_directories(terra_data INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/data/include")
target_compile_features(terra_data INTERFACE cxx_std_17)

# terra_core：最小公共类型与构建信息。
# 约束：不链接物理库、图形库或窗口系统，可在无显示环境中构建和运行。
configure_file(
    src/build_config.hpp.in
    "${CMAKE_CURRENT_BINARY_DIR}/generated/terra/core/build_config.hpp"
    @ONLY)

add_library(terra_core STATIC
    src/build_info.cpp
    src/fatal.cpp
    src/system_monitor.cpp)
add_library(terra::core ALIAS terra_core)

target_include_directories(terra_core
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PRIVATE
        "${CMAKE_CURRENT_BINARY_DIR}/generated")

terra_configure_target(terra_core)
