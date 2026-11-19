# terra_platform：窗口与 OpenGL 上下文接口（M1-03），仅头文件，只依赖 terra_input（InputSink）。
# 仅在 TERRA_BUILD_RENDER=ON 时构建。
add_library(terra_platform INTERFACE)
add_library(terra::platform ALIAS terra_platform)
target_include_directories(terra_platform INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(terra_platform INTERFACE terra::input)

add_subdirectory(backends/glfw)
