# terra_library：资源库与实验记录索引（M3-07b，D-112）。总是构建，无图形依赖。
# 文件是原件，SQLite 数据库只是可重建的索引：删掉数据库不丢任何东西，重新扫描即可恢复。
# M3 索引实验记录（*.terra-record.jsonl）；M4-08c 加入资产（场景、动作、机器人、环境贴图）与资产包（.terra-pack）。
# 约束：SQLite 只在本模块的实现中使用（sqlite3.h 不进入公共头，boundary.render_isolation 检查）。
# 第三方：miniz（MIT，third_party/miniz）读写资产包的 zip，只在 src/pack.cpp 中使用，不套用 Terra 的严格警告。
add_library(terra_miniz STATIC "${PROJECT_SOURCE_DIR}/third_party/miniz/miniz.c")
set_target_properties(terra_miniz PROPERTIES C_STANDARD 99 POSITION_INDEPENDENT_CODE ON)
target_include_directories(terra_miniz SYSTEM INTERFACE "${PROJECT_SOURCE_DIR}/third_party/miniz")
target_compile_options(terra_miniz PRIVATE -w)  # 第三方源码，不显示警告

add_library(terra_library STATIC
    src/library.cpp
    src/pack.cpp)
add_library(terra::library ALIAS terra_library)
target_include_directories(terra_library PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(terra_library PUBLIC terra::simulation PRIVATE terra::sqlite terra::core terra::scene terra::json terra_miniz terra::viz)
terra_configure_target(terra_library)
