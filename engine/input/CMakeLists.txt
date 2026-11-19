# terra_input：输入抽象（M1-04，D-018）。
# 约束：只依赖标准库，不链接、不包含任何窗口库、输入库或图形 API（由 boundary.render_isolation 检查）。
# 总是构建（不受 TERRA_BUILD_RENDER 控制），单元测试在无图形环境运行。
add_library(terra_input STATIC
    src/input_system.cpp)
add_library(terra::input ALIAS terra_input)
target_include_directories(terra_input PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
terra_configure_target(terra_input)
