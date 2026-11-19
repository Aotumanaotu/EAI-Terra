# terra_bench：M0 物理步进性能基线（M0-09，开发计划 4.4 节）。只记录，不设阻塞门槛。
# 复用 Headless 的 JSON 序列化来测量输出开销；只链接 Terra 目标。
add_executable(terra_bench bench.cpp)
target_link_libraries(terra_bench PRIVATE terra_headless_cli terra::core terra::physics terra::physics_mujoco)
target_compile_definitions(terra_bench PRIVATE TERRA_BENCH_BUILD_TYPE="$<CONFIG>")
terra_configure_target(terra_bench)
