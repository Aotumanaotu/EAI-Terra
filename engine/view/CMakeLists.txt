# terra_view：视图层——相机与视口相机控制器（M1-07，D-018、D-020、D-023）。仅 TERRA_BUILD_RENDER=ON 时构建（需要 GLM）。
# 约束：只消费 Terra 输入动作，不包含窗口库或 OpenGL 头文件（boundary.render_isolation 检查）；
# GLM 只 PRIVATE 使用，公共头只用 Terra 类型与标准库；不依赖 terra_render 与平台层。
add_library(terra_view STATIC
    src/camera.cpp
    src/camera_controller.cpp)
add_library(terra::view ALIAS terra_view)
target_include_directories(terra_view PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(terra_view
    PUBLIC terra::input terra::data
    PRIVATE terra::glm)
terra_configure_target(terra_view)
