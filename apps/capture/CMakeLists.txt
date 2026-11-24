# terra_capture（M4-04）：离线采集传感器数据（IMU、2D 激光雷达、相机），写入目录。仅 TERRA_BUILD_RENDER=ON 时构建。
# 与查看器共用 terra_viewer_scene（场景的渲染适配、相机渲染），与 Headless 共用 JSON 行格式（terra_headless_cli）。
# 只在场景中有相机时创建隐藏窗口；Headless 仍然不链接图形库。
add_executable(terra_capture main.cpp)
target_link_libraries(terra_capture PRIVATE terra::viewer_scene terra::render terra::platform_glfw terra::input terra::scene
                                            terra::simulation terra::physics_mujoco terra::core terra_headless_cli)
target_compile_definitions(terra_capture PRIVATE TERRA_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets")
terra_configure_target(terra_capture)
