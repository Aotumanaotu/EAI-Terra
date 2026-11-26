# terra_policy：策略的说明文件、推理后端、自动配置与控制器（M4-10a）。总是构建，无图形依赖。
# 推理库不在构建时链接：onnxruntime-cpu 后端在运行时 dlopen 可下载的组件（terra::library 的组件管理），
# 仓库只带 ONNX Runtime 的 C API 头文件（third_party/onnxruntime，MIT），只在 src/onnxruntime_backend.cpp 中使用。
# M4-10b：导入（src/import.cpp）用 miniz 读 .pt（zip）的目录与 data.pkl。
# 约束：JSON 库、miniz 与 ONNX Runtime 头文件不进入公共头（boundary.render_isolation 检查）。
add_library(terra_policy STATIC
    src/autoconfig.cpp
    src/backend.cpp
    src/controller.cpp
    src/import.cpp
    src/onnx_info.cpp
    src/onnxruntime_backend.cpp
    src/policy_file.cpp)
add_library(terra::policy ALIAS terra_policy)
target_include_directories(terra_policy PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories(terra_policy SYSTEM PRIVATE "${PROJECT_SOURCE_DIR}/third_party/onnxruntime/include")
target_link_libraries(terra_policy PUBLIC terra::simulation terra::scene terra::physics terra::data
                      PRIVATE terra::json terra::core terra::library terra_miniz ${CMAKE_DL_LIBS})
terra_configure_target(terra_policy)
