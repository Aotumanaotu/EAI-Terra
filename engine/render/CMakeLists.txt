# terra_render：渲染核心（M1-06，D-004、D-021）。仅 TERRA_BUILD_RENDER=ON 时构建。
# 约束：公共头不包含 OpenGL、窗口库或 MuJoCo 头文件；glad 只 PRIVATE 链接；不依赖平台层与物理层
# （上下文由调用方创建，只通过 GlLoader 函数指针交接）。M1-08 起 PRIVATE 使用 GLM（光源矩阵、法线矩阵，D-020）。
add_library(terra_render STATIC
    src/context.cpp
    src/environment.cpp
    src/hdr_image.cpp
    src/image.cpp
    src/mesh.cpp
    src/pbr.cpp
    src/resources.cpp
    src/scene_renderer.cpp
    src/stb_image_impl.cpp)
add_library(terra::render ALIAS terra_render)
target_include_directories(terra_render PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(terra_render PRIVATE terra::glad terra::glm terra::stb)
terra_configure_target(terra_render)
