# terra_physics_mujoco：MuJoCo 私有适配层。
# 约束：mujoco::mujoco 只能 PRIVATE 链接；include/ 下的公共头只能使用 Terra 类型，
# 不得包含 MuJoCo 头文件（由 boundary.no_mujoco_in_public_headers 测试检查）。
add_library(terra_physics_mujoco STATIC
    src/backend_info.cpp)
add_library(terra::physics_mujoco ALIAS terra_physics_mujoco)

target_include_directories(terra_physics_mujoco
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(terra_physics_mujoco PRIVATE mujoco::mujoco)

terra_configure_target(terra_physics_mujoco)
