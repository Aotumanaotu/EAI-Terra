# terra_library：资源库与实验记录索引（M3-07b，D-112）。总是构建，无图形依赖。
# 文件是原件，SQLite 数据库只是可重建的索引：删掉数据库不丢任何东西，重新扫描即可恢复。
# M3 只索引实验记录（*.terra-record.jsonl）；M4 的资源库（模型、场景、动作、策略）沿用同一个库。
# 约束：SQLite 只在本模块的实现中使用（sqlite3.h 不进入公共头，boundary.render_isolation 检查）。
add_library(terra_library STATIC
    src/library.cpp)
add_library(terra::library ALIAS terra_library)
target_include_directories(terra_library PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(terra_library PUBLIC terra::simulation PRIVATE terra::sqlite terra::core)
terra_configure_target(terra_library)
