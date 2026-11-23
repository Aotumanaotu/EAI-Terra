# terra_render_bench：渲染帧耗时基线（M1-10）。仅 TERRA_BUILD_RENDER=ON 时构建；与 terra_viewer 共用 terra_viewer_scene。
add_executable(terra_render_bench main.cpp)
target_link_libraries(terra_render_bench PRIVATE terra::viewer_scene terra::render terra::view terra::platform_glfw terra::input)
target_compile_definitions(terra_render_bench PRIVATE TERRA_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets")
terra_configure_target(terra_render_bench)
