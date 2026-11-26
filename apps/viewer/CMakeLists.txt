# terra_viewer：EAI Terra 的第一个应用——编辑器式界面外壳（M1-13），设置与持久化（M1-14）。仅 TERRA_BUILD_RENDER=ON 时构建。
# 只链接 Terra 目标；不包含 GLFW 或 OpenGL 头文件（boundary.render_isolation 检查），ImGui 作为界面 API 使用。

# terra_viewer_scene（M1-10）：演示场景、环境贴图库与渲染设置换算；不依赖界面库。
# 查看器、参考图测试与渲染基准共用，保证三者画面一致。
# M2-07：场景的渲染适配（scene_view.cpp）：terra_scene 的视觉部件 → 渲染项；不链接物理后端。
add_library(terra_viewer_scene STATIC demo_scene.cpp scene_view.cpp mjcf_visual.cpp camera_sensor.cpp)
add_library(terra::viewer_scene ALIAS terra_viewer_scene)
target_include_directories(terra_viewer_scene PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(terra_viewer_scene PUBLIC terra::render terra::asset terra::settings terra::scene)
terra_configure_target(terra_viewer_scene)

add_executable(terra_viewer main.cpp viewer_settings.cpp import_wizard.cpp library_panel.cpp viz_panel.cpp policy_panel.cpp)
target_link_libraries(terra_viewer PRIVATE terra::viewer_scene terra::ui terra::render terra::asset terra::view terra::settings
                                           terra::platform_glfw terra::core
                                           terra::scene terra::simulation terra::library terra::physics_mujoco
                                           terra::import terra::viz terra::policy)
# M2-07 起查看器运行物理（开发计划 9.5 节第 7 条）：查看器程序链接 MuJoCo 后端；渲染、界面、资产库仍不链接物理。
target_compile_definitions(terra_viewer PRIVATE TERRA_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets" TERRA_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
terra_configure_target(terra_viewer)
# M4-06a：ROS2 桥接（TERRA_BUILD_ROS2=ON 且找到 ROS2 Humble 时）；没有时菜单项禁用并提示安装
if(TERRA_ROS2_ENABLED)
    target_link_libraries(terra_viewer PRIVATE terra::ros2)
    target_compile_definitions(terra_viewer PRIVATE TERRA_WITH_ROS2=1)
endif()
