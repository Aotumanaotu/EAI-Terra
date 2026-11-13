# EAI Terra 根构建入口（M0-01）
#
# 基线：Ubuntu 22.04 / x86-64、GCC、C++17、CMake + Ninja（见 D-008）。
# 最低 CMake 版本按 Ubuntu 22.04 官方源（3.22）设定，不使用更新版本的特性。
cmake_minimum_required(VERSION 3.22)

project(EAITerra
    VERSION 0.0.0
    DESCRIPTION "EAI Terra - Open Robotics Simulation Engine"
    LANGUAGES CXX)

if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
    message(FATAL_ERROR "禁止源码目录内构建，请使用独立构建目录，例如：cmake --preset debug")
endif()

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(TerraCompilerOptions)

option(TERRA_BUILD_TESTS "构建并注册 CTest 测试" ON)
option(TERRA_WARNINGS_AS_ERRORS "将编译警告视为错误" OFF)

# 单配置生成器默认 Debug，避免无优化、无调试信息的空构建类型。
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "构建类型" FORCE)
endif()

include(TerraMuJoCo)

# 目标边界（M0-01 评审结果，详见 docs/validation/M0-01-environment.md）：
#   terra_core            engine/core                     最小公共类型与构建信息，不依赖任何物理或图形库
#   terra_physics_mujoco  engine/physics/backends/mujoco  MuJoCo 私有适配层（M0-02），PRIVATE 链接 mujoco::mujoco
#   terra_headless        apps/headless                   命令行入口，只链接 Terra 目标
# 尚待后续任务接入：
#   terra_physics         engine/physics                  后端无关接口（M0-04–07）
add_subdirectory(engine/core)
add_subdirectory(engine/physics)
add_subdirectory(apps/headless)

if(TERRA_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
