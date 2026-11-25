# terra_import：模型导入的命令行程序（M4-08a）。不链接图形库。
add_executable(terra_import_cli main.cpp)
set_target_properties(terra_import_cli PROPERTIES OUTPUT_NAME terra_import)
target_link_libraries(terra_import_cli PRIVATE terra::import terra::library terra::physics_mujoco terra::scene)
terra_configure_target(terra_import_cli)
