# terra_viewer：EAI Terra 的第一个应用——编辑器式界面外壳（M1-13），设置与持久化（M1-14）。仅 TERRA_BUILD_RENDER=ON 时构建。
# 只链接 Terra 目标；不包含 GLFW 或 OpenGL 头文件（boundary.render_isolation 检查），ImGui 作为界面 API 使用。

# terra_viewer_scene（M1-10）：演示场景、环境贴图库与渲染设置换算；不依赖界面库。
# 查看器、参考图测试与渲染基准共用，保证三者画面一致。
add_library(terra_viewer_scene STATIC demo_scene.cpp)
add_library(terra::viewer_scene ALIAS terra_viewer_scene)
target_include_directories(terra_viewer_scene PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(terra_viewer_scene PUBLIC terra::render terra::asset terra::settings)
terra_configure_target(terra_viewer_scene)

add_executable(terra_viewer main.cpp viewer_settings.cpp)
target_link_libraries(terra_viewer PRIVATE terra::viewer_scene terra::ui terra::render terra::asset terra::view terra::settings
                                           terra::platform_glfw)
target_compile_definitions(terra_viewer PRIVATE TERRA_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets")
terra_configure_target(terra_viewer)
