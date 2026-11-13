# terra_core：最小公共类型与构建信息。
# 约束：不链接物理库、图形库或窗口系统，可在无显示环境中构建和运行。
configure_file(
    src/build_config.hpp.in
    "${CMAKE_CURRENT_BINARY_DIR}/generated/terra/core/build_config.hpp"
    @ONLY)

add_library(terra_core STATIC
    src/build_info.cpp)
add_library(terra::core ALIAS terra_core)

target_include_directories(terra_core
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PRIVATE
        "${CMAKE_CURRENT_BINARY_DIR}/generated")

terra_configure_target(terra_core)
