# Terra Physics。
# terra_physics：后端无关接口（M0-07），仅头文件，只依赖 terra_data。上层代码只需链接它即可使用 PhysicsWorld。
add_library(terra_physics INTERFACE)
add_library(terra::physics ALIAS terra_physics)
target_include_directories(terra_physics INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(terra_physics INTERFACE terra::data)

add_subdirectory(backends/mujoco)
