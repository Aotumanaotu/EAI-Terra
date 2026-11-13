# terra_headless：纯命令行入口。
# 约束：只链接 Terra 目标；不得包含 MuJoCo 头文件，不得链接图形或窗口库。
add_executable(terra_headless
    main.cpp)

target_link_libraries(terra_headless PRIVATE terra::core terra::physics_mujoco)

terra_configure_target(terra_headless)
