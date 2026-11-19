# terra_viewer：EAI Terra 的第一个应用——编辑器式界面外壳（M1-13），设置与持久化（M1-14）。仅 TERRA_BUILD_RENDER=ON 时构建。
# 只链接 Terra 目标；不包含 GLFW 或 OpenGL 头文件（boundary.render_isolation 检查），ImGui 作为界面 API 使用。
add_executable(terra_viewer main.cpp viewer_settings.cpp)
target_link_libraries(terra_viewer PRIVATE terra::ui terra::render terra::view terra::settings terra::platform_glfw)
target_compile_definitions(terra_viewer PRIVATE TERRA_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets")
terra_configure_target(terra_viewer)
