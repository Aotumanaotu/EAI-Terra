# terra_ui：编辑器式界面基础（M1-13，D-024、D-028、D-029）。仅 TERRA_BUILD_RENDER=ON 时构建。
# 约束：ImGui 是应用层界面 API，只允许在 engine/ui/、apps/viewer/ 与测试中使用（boundary.render_isolation 检查）；
# 不使用 ImGui 的 GLFW 平台后端，事件由 UiInputBridge 从 Terra 平台层与输入系统获得。
add_library(terra_ui STATIC
    src/file_browser.cpp
    src/text.cpp
    src/ui_input.cpp
    src/ui_context.cpp
    src/widgets.cpp)
add_library(terra::ui ALIAS terra_ui)
target_include_directories(terra_ui PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(terra_ui PUBLIC terra::imgui terra::input terra::platform terra::core)
terra_configure_target(terra_ui)
