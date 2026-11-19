# 导入 nlohmann/json（M1-14，D-026）。总是加载：设置模块不依赖渲染，渲染关闭时也构建与测试。
#
# 头文件直接提交在 third_party/nlohmann_json/（版本与 sha256 见 json.lock，scripts/settings/check_json.sh 复核）；CMake 不联网。
# 输出目标：terra::json（仅头文件，SYSTEM 方式引入；JSON_USE_IMPLICIT_CONVERSIONS=0 要求显式 get<T>()）。
# 约束：只允许在 engine/settings/src/ 与测试中包含（boundary.render_isolation 检查）。

set(_terra_json_dir "${PROJECT_SOURCE_DIR}/third_party/nlohmann_json")
file(STRINGS "${_terra_json_dir}/json.lock" _terra_json_line REGEX "^JSON_VERSION=")
string(REPLACE "JSON_VERSION=" "" TERRA_JSON_VERSION "${_terra_json_line}")
if(TERRA_JSON_VERSION STREQUAL "" OR NOT EXISTS "${_terra_json_dir}/include/nlohmann/json.hpp")
    message(FATAL_ERROR "找不到 nlohmann/json：${_terra_json_dir}（应随仓库提交）")
endif()
add_library(terra_json INTERFACE)
add_library(terra::json ALIAS terra_json)
target_include_directories(terra_json SYSTEM INTERFACE "${_terra_json_dir}/include")
target_compile_definitions(terra_json INTERFACE JSON_USE_IMPLICIT_CONVERSIONS=0)
message(STATUS "JSON: nlohmann/json ${TERRA_JSON_VERSION}")
