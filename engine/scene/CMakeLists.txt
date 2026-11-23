# terra_scene：Terra 场景（M2，M2-01 设计）。总是构建，不受 TERRA_BUILD_RENDER 控制，单元测试在无图形环境运行。
# M2-04：模型实例组合——校验实例名、分配前缀、决定全局物理配置、调用注入的世界工厂、把问题归属到实例。
# M2-05：场景文件（JSON，nlohmann/json 只在实现中 PRIVATE 使用）、场景实例与视觉部件映射、视觉部件世界位姿。
# 约束：不链接 MuJoCo、GL、界面与渲染模块；只通过 terra_physics 的接口与注入的世界工厂创建世界（NFR-01）。
add_library(terra_scene STATIC
    src/composition.cpp
    src/scene_file.cpp
    src/scene_instance.cpp)
add_library(terra::scene ALIAS terra_scene)
target_include_directories(terra_scene PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(terra_scene PUBLIC terra::physics terra::data PRIVATE terra::json)
terra_configure_target(terra_scene)
