# 导入渲染依赖（M1-02，D-017、D-019、D-020、D-021）。仅在 TERRA_BUILD_RENDER=ON 时加载。
#
# 依赖由 scripts/render/fetch.sh 与 build.sh 准备，版本见 third_party/render/render.lock；CMake 不联网。
# 输出目标：
#   terra::glfw  GLFW 静态库（窗口、OpenGL 上下文、首个输入适配器）
#   terra::glad  glad 生成的 OpenGL 4.5 core 加载代码（third_party/glad，已提交）
#   terra::glm   GLM 头文件（只在 Render 内部使用）
# 三者的头文件均以 SYSTEM 方式引入，避免第三方头文件触发 Terra 的 -Werror 警告。

set(TERRA_RENDER_ROOT "${PROJECT_SOURCE_DIR}/third_party/render"
    CACHE PATH "渲染依赖目录（包含 render.lock、src/、install/）")

set(_terra_render_lock "${PROJECT_SOURCE_DIR}/third_party/render/render.lock")
foreach(_name GLFW_VERSION GLM_VERSION GLAD_VERSION GLAD_API)
    file(STRINGS "${_terra_render_lock}" _line REGEX "^${_name}=")
    string(REPLACE "${_name}=" "" TERRA_${_name} "${_line}")
    if(TERRA_${_name} STREQUAL "")
        message(FATAL_ERROR "render.lock 中缺少 ${_name}")
    endif()
endforeach()

set(_terra_render_hint
    "请先获取并构建渲染依赖：\n"
    "  scripts/render/fetch.sh\n"
    "  scripts/render/build.sh\n"
    "或用 -DTERRA_RENDER_ROOT=<目录> 指定已有位置；不需要渲染时保持 TERRA_BUILD_RENDER=OFF。")

# glad 生成代码是 C 源文件。
enable_language(C)

# ---- GLFW ----
find_package(glfw3 ${TERRA_GLFW_VERSION} EXACT CONFIG QUIET
    PATHS "${TERRA_RENDER_ROOT}/install" NO_DEFAULT_PATH)
if(NOT glfw3_FOUND OR NOT TARGET glfw)
    message(FATAL_ERROR "在 ${TERRA_RENDER_ROOT}/install 下找不到 GLFW ${TERRA_GLFW_VERSION}。\n" ${_terra_render_hint})
endif()
add_library(terra_glfw INTERFACE)
add_library(terra::glfw ALIAS terra_glfw)
target_link_libraries(terra_glfw INTERFACE glfw)
# 由 Terra 自己加载 OpenGL 函数（glad），GLFW 头文件不包含系统 GL 头。
target_compile_definitions(terra_glfw INTERFACE GLFW_INCLUDE_NONE)

# ---- glad（已提交的生成代码）----
set(_terra_glad_dir "${PROJECT_SOURCE_DIR}/third_party/glad")
add_library(terra_glad STATIC "${_terra_glad_dir}/src/gl.c")
add_library(terra::glad ALIAS terra_glad)
target_include_directories(terra_glad SYSTEM PUBLIC "${_terra_glad_dir}/include")
set_target_properties(terra_glad PROPERTIES POSITION_INDEPENDENT_CODE ON C_STANDARD 99 C_EXTENSIONS OFF)

# ---- GLM（仅头文件）----
set(_terra_glm_dir "${TERRA_RENDER_ROOT}/src/glm-${TERRA_GLM_VERSION}")
if(NOT EXISTS "${_terra_glm_dir}/glm/glm.hpp")
    message(FATAL_ERROR "找不到 GLM ${TERRA_GLM_VERSION}：${_terra_glm_dir}/glm/glm.hpp\n" ${_terra_render_hint})
endif()
add_library(terra_glm INTERFACE)
add_library(terra::glm ALIAS terra_glm)
target_include_directories(terra_glm SYSTEM INTERFACE "${_terra_glm_dir}")
# 不启用 GLM 的实验性扩展与自动 SIMD；深度范围等约定在 M1-06/07 的渲染实现中统一决定。
target_compile_definitions(terra_glm INTERFACE GLM_FORCE_SILENT_WARNINGS)

message(STATUS "Render: GLFW ${TERRA_GLFW_VERSION}、GLM ${TERRA_GLM_VERSION}、glad ${TERRA_GLAD_VERSION}（${TERRA_GLAD_API}）")
