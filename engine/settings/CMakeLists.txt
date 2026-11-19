# terra_settings：用户设置模型与 JSON 读写（M1-14，D-026）。总是构建（不受 TERRA_BUILD_RENDER 控制），单元测试在无图形环境运行。
# 约束：公共头只用 Terra 类型与标准库；nlohmann/json 只在实现中 PRIVATE 使用（boundary.render_isolation 检查）。
# 不依赖平台、渲染、视图与界面模块：应用负责把设置换算成各模块的参数。
add_library(terra_settings STATIC
    src/settings.cpp)
add_library(terra::settings ALIAS terra_settings)
target_include_directories(terra_settings PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(terra_settings
    PUBLIC terra::input
    PRIVATE terra::json)
terra_configure_target(terra_settings)
