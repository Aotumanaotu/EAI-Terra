# 导入本地安装的 MuJoCo，定义 mujoco::mujoco（M0-02）。
#
# 不使用 find_package(mujoco)：官方 mujocoConfig.cmake 无条件 find_dependency(OpenGL)，
# 会让纯物理构建依赖图形开发包，而 libmujoco 本身并不链接 OpenGL（渲染代码运行时通过 dlopen 加载 GL）。
# 因此直接加载官方导出的 mujocoTargets.cmake，并按 third_party/mujoco/mujoco.lock 核对版本。
#
# 输入：MUJOCO_ROOT（缓存变量），默认 third_party/mujoco/install（scripts/mujoco/build.sh 的安装位置）
# 输出：mujoco::mujoco 导入目标；TERRA_MUJOCO_VERSION（锁定版本字符串）

set(MUJOCO_ROOT "${PROJECT_SOURCE_DIR}/third_party/mujoco/install"
    CACHE PATH "MuJoCo 安装目录（包含 include/mujoco 与 lib/cmake/mujoco）")

file(STRINGS "${PROJECT_SOURCE_DIR}/third_party/mujoco/mujoco.lock" _terra_mj_lock
     REGEX "^MUJOCO_VERSION=")
string(REPLACE "MUJOCO_VERSION=" "" TERRA_MUJOCO_VERSION "${_terra_mj_lock}")
if(NOT TERRA_MUJOCO_VERSION MATCHES "^([0-9]+)\\.([0-9]+)\\.([0-9]+)$")
    message(FATAL_ERROR "mujoco.lock 中的 MUJOCO_VERSION 格式无效：'${TERRA_MUJOCO_VERSION}'")
endif()
# mjVERSION_HEADER 编码：major*1000000 + minor*1000 + patch（例如 4.0.1 -> 4000001）
math(EXPR TERRA_MUJOCO_VERSION_NUMBER
     "${CMAKE_MATCH_1} * 1000000 + ${CMAKE_MATCH_2} * 1000 + ${CMAKE_MATCH_3}")

set(_terra_mj_hint
    "请先获取并构建 MuJoCo ${TERRA_MUJOCO_VERSION}：\n"
    "  scripts/mujoco/fetch.sh\n"
    "  scripts/mujoco/build.sh\n"
    "或用 -DMUJOCO_ROOT=<安装目录> 指定已有安装。")

set(_terra_mj_header "${MUJOCO_ROOT}/include/mujoco/mujoco.h")
if(NOT EXISTS "${_terra_mj_header}")
    message(FATAL_ERROR "在 MUJOCO_ROOT=${MUJOCO_ROOT} 下找不到 include/mujoco/mujoco.h。\n" ${_terra_mj_hint})
endif()

file(STRINGS "${_terra_mj_header}" _terra_mj_define REGEX "^#define mjVERSION_HEADER ")
string(REGEX REPLACE "^#define mjVERSION_HEADER ([0-9]+).*$" "\\1" _terra_mj_header_version "${_terra_mj_define}")
if(NOT _terra_mj_header_version STREQUAL TERRA_MUJOCO_VERSION_NUMBER)
    message(FATAL_ERROR
        "MuJoCo 版本不符：mujoco.lock 要求 ${TERRA_MUJOCO_VERSION}（${TERRA_MUJOCO_VERSION_NUMBER}），"
        "${_terra_mj_header} 为 mjVERSION_HEADER=${_terra_mj_header_version}。\n" ${_terra_mj_hint})
endif()

find_file(_terra_mj_targets mujocoTargets.cmake
    PATHS "${MUJOCO_ROOT}/lib/cmake/mujoco" "${MUJOCO_ROOT}/lib64/cmake/mujoco"
    NO_DEFAULT_PATH NO_CACHE)
if(NOT _terra_mj_targets)
    message(FATAL_ERROR "在 MUJOCO_ROOT=${MUJOCO_ROOT} 下找不到 lib/cmake/mujoco/mujocoTargets.cmake。\n" ${_terra_mj_hint})
endif()

if(NOT TARGET mujoco::mujoco)
    include("${_terra_mj_targets}")
endif()
if(NOT TARGET mujoco::mujoco)
    message(FATAL_ERROR "${_terra_mj_targets} 未定义 mujoco::mujoco")
endif()

message(STATUS "MuJoCo ${TERRA_MUJOCO_VERSION}: ${MUJOCO_ROOT}")
