# terra_asset：资产加载（M1-09，D-013）。glTF 2.0 解析（cgltf，PRIVATE）与上传（经 terra_render 接口）。仅 TERRA_BUILD_RENDER=ON 时构建。
# 约束：公共头只用 Terra 类型与标准库；不包含 OpenGL、窗口库、cgltf 或 MuJoCo 头文件（boundary 检查）。
add_library(terra_asset STATIC
    src/gltf.cpp
    src/gpu_asset.cpp)
add_library(terra::asset ALIAS terra_asset)
target_include_directories(terra_asset PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(terra_asset
    PUBLIC terra::render
    PRIVATE terra::cgltf)
terra_configure_target(terra_asset)
