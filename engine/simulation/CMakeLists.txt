# terra_simulation：仿真时钟、命令队列、快照历史与插值（M2-06，M2-01 设计第 7 节，D-106）。总是构建，无图形依赖。
# M2 单线程：主循环每帧调用 SimulationRunner::update，先推进若干固定步，再由渲染读取插值后的位姿。
# 接口不假设与渲染在同一线程（快照以 shared_ptr<const> 传递），以后可把推进循环移到物理线程。
# 约束：只通过 terra_physics 的接口驱动世界，不链接 MuJoCo、GL。
add_library(terra_simulation STATIC
    src/clock.cpp
    src/interpolation.cpp
    src/runner.cpp)
add_library(terra::simulation ALIAS terra_simulation)
target_include_directories(terra_simulation PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(terra_simulation PUBLIC terra::physics terra::data)
terra_configure_target(terra_simulation)
