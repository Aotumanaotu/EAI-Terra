# terra_physics_mujoco：MuJoCo 私有适配层，实现 terra::physics::PhysicsWorld（M0-07）。
# 约束：mujoco::mujoco 只能 PRIVATE 链接；include/ 下的公共头只能使用 Terra 类型，
# 不得包含 MuJoCo 头文件（由 boundary.no_mujoco_in_public_headers 测试检查）。
add_library(terra_physics_mujoco STATIC
    src/backend_info.cpp
    src/mujoco_world.cpp
    src/ray_index.cpp)
add_library(terra::physics_mujoco ALIAS terra_physics_mujoco)

target_include_directories(terra_physics_mujoco
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(terra_physics_mujoco
    PUBLIC terra::physics
    PRIVATE mujoco::mujoco terra::core)

terra_configure_target(terra_physics_mujoco)
