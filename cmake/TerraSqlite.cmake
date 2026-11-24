# 导入 SQLite（M3-07b，D-112）。总是加载：实验记录索引与资源库不依赖渲染。
#
# 合并版源码直接提交在 third_party/sqlite/src/（版本与 sha256 见 sqlite.lock，scripts/sqlite/check_sqlite.sh 复核）；CMake 不联网。
# 输出目标：terra::sqlite（静态库，C 语言；头文件以 SYSTEM 方式引入）。不套用 Terra 的警告选项（第三方源码）。
# 编译选项：
#   SQLITE_DQS=0                     字符串只能用单引号（官方推荐）
#   SQLITE_DEFAULT_FOREIGN_KEYS=1    默认启用外键
#   SQLITE_OMIT_LOAD_EXTENSION       不加载扩展（不需要 libdl）
#   SQLITE_THREADSAFE=1              串行化模式（需要 pthread）；查看器以后可能在后台线程写记录
# 约束：只允许 engine/library/src/ 与测试包含 sqlite3.h（boundary.render_isolation 检查）。

enable_language(C)
find_package(Threads REQUIRED)
set(_terra_sqlite_dir "${PROJECT_SOURCE_DIR}/third_party/sqlite")
file(STRINGS "${_terra_sqlite_dir}/sqlite.lock" _terra_sqlite_line REGEX "^SQLITE_VERSION=")
string(REPLACE "SQLITE_VERSION=" "" TERRA_SQLITE_VERSION "${_terra_sqlite_line}")
if(TERRA_SQLITE_VERSION STREQUAL "" OR NOT EXISTS "${_terra_sqlite_dir}/src/sqlite3.c")
    message(FATAL_ERROR "找不到 SQLite：${_terra_sqlite_dir}（应随仓库提交）")
endif()
add_library(terra_sqlite STATIC "${_terra_sqlite_dir}/src/sqlite3.c")
add_library(terra::sqlite ALIAS terra_sqlite)
set_target_properties(terra_sqlite PROPERTIES C_STANDARD 99 POSITION_INDEPENDENT_CODE ON)
target_include_directories(terra_sqlite SYSTEM INTERFACE "${_terra_sqlite_dir}/src")
target_compile_definitions(terra_sqlite PRIVATE SQLITE_DQS=0 SQLITE_DEFAULT_FOREIGN_KEYS=1 SQLITE_OMIT_LOAD_EXTENSION
                                                SQLITE_THREADSAFE=1)
target_compile_options(terra_sqlite PRIVATE -w)  # 第三方源码，不显示警告
target_link_libraries(terra_sqlite PUBLIC Threads::Threads)
message(STATUS "SQLite: ${TERRA_SQLITE_VERSION}")
