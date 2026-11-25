# terra_simulation：仿真时钟、命令队列、快照历史与插值（M2-06，M2-01 设计第 7 节，D-106）。总是构建，无图形依赖。
# M2 单线程：主循环每帧调用 SimulationRunner::update，先推进若干固定步，再由渲染读取插值后的位姿。
# 接口不假设与渲染在同一线程（快照以 shared_ptr<const> 传递），以后可把推进循环移到物理线程。
# M4-02：传感器采样时刻（按时间累计）。M4-03b：2D 激光雷达（经物理接口的射线检测）。
# M4-09：传感器噪声（计数器式随机数，带种子、可复现）。
# M3-07a：实验记录的写入、读取与复现（record.cpp，使用 terra_core 的 SHA-256）。
# M3-04b：PD 控制层（场景文件 controllers），依赖 terra_scene 的 PdAssignment。
# M3-04：动作文件（JSON，nlohmann/json 只在实现中 PRIVATE 使用）、动作的绑定与播放、每步控制器接口。
# 约束：只通过 terra_physics 的接口驱动世界，不链接 MuJoCo、GL。
add_library(terra_simulation STATIC
    src/clock.cpp
    src/interpolation.cpp
    src/lidar.cpp
    src/motion.cpp
    src/motion_file.cpp
    src/pd_layer.cpp
    src/record.cpp
    src/runner.cpp
    src/sensor_noise.cpp
    src/sensor_sampling.cpp)
add_library(terra::simulation ALIAS terra_simulation)
target_include_directories(terra_simulation PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(terra_simulation PUBLIC terra::physics terra::data terra::scene PRIVATE terra::json terra::core)
terra_configure_target(terra_simulation)
