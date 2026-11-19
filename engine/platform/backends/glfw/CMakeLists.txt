# terra_platform_glfw：GLFW 平台实现与 GLFW 键鼠适配器（M1-03/04，D-017）。
# 约束：GLFW 只能 PRIVATE 链接；GLFW 头文件只能出现在本目录的 src/ 中（由 boundary.render_isolation 检查）。
add_library(terra_platform_glfw STATIC
    src/glfw_platform.cpp
    src/glfw_translation.cpp)
add_library(terra::platform_glfw ALIAS terra_platform_glfw)
target_include_directories(terra_platform_glfw PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(terra_platform_glfw
    PUBLIC terra::platform
    PRIVATE terra::glfw)
terra_configure_target(terra_platform_glfw)
