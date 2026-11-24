# 用法：cmake -DSOURCE_DIR=<仓库根目录> -P TerraCheckRenderIsolation.cmake
# 检查窗口库与图形 API 头文件只出现在允许的位置（D-017、D-018、NFR-01）：
#   GLFW/   只允许在 engine/platform/backends/glfw/src/ 与 tests/platform/、tests/render/
#   glad/、GL/、KHR/  只允许在 engine/render/、engine/platform/backends/glfw/src/ 与 tests/platform/、tests/render/
#   X11/、wayland-、SDL、vulkan/  engine/ 与 apps/ 中一律不允许（将来的适配器另行登记）
#   imgui   只允许在 engine/ui/、apps/viewer/ 与 tests/（M1-13，D-024：ImGui 是应用层界面 API）
#   nlohmann/  只允许在 engine/settings/src/、engine/scene/src/、engine/simulation/src/ 与 tests/（M1-14、M2-05、M3-04：JSON 库不进入任何公共头文件）
#   stb_    只允许在 engine/render/src/ 与 tests/（M1-08b）
#   cgltf   只允许在 engine/asset/src/ 与 tests/（M1-09）
#   sqlite3 只允许在 engine/library/src/ 与 tests/（M3-07b，D-112）
#   mujoco/ 在 engine/ 中只允许在 MuJoCo 后端 engine/physics/backends/mujoco/src/（M2-04：terra_scene 等不得直接使用 MuJoCo）
# 另外，engine/input/ 与 engine/platform/include/（接口层）不得包含任何上述头文件。
if(NOT DEFINED SOURCE_DIR)
    message(FATAL_ERROR "需要 -DSOURCE_DIR")
endif()

file(GLOB_RECURSE files
    "${SOURCE_DIR}/engine/*" "${SOURCE_DIR}/apps/*" "${SOURCE_DIR}/tests/*")

set(glfw_allowed "/engine/platform/backends/glfw/src/|/tests/platform/|/tests/render/")
set(gl_allowed "/engine/render/|/engine/platform/backends/glfw/src/|/tests/platform/|/tests/render/")
set(imgui_allowed "/engine/ui/|/apps/viewer/|/tests/")
set(json_allowed "/engine/settings/src/|/engine/scene/src/|/engine/simulation/src/|/tests/")
set(stb_allowed "/engine/render/src/|/tests/")
set(cgltf_allowed "/engine/asset/src/|/tests/")
set(mujoco_allowed "/engine/physics/backends/mujoco/src/")
set(sqlite_allowed "/engine/library/src/|/tests/")

set(violations "")
set(checked 0)
foreach(f IN LISTS files)
    if(NOT f MATCHES "\\.(h|hh|hpp|hxx|c|cc|cpp|cxx|inl|ipp)$")
        continue()
    endif()
    math(EXPR checked "${checked} + 1")
    file(RELATIVE_PATH rel "${SOURCE_DIR}" "${f}")
    set(rel "/${rel}")
    file(STRINGS "${f}" includes REGEX "^[ \t]*#[ \t]*include[ \t]*[<\"]")
    foreach(line IN LISTS includes)
        if(line MATCHES "[<\"]GLFW/" AND NOT rel MATCHES "${glfw_allowed}")
            list(APPEND violations "${rel}: ${line}（GLFW 只能出现在 GLFW 平台实现中）")
        endif()
        if(line MATCHES "[<\"](glad/|GL/|KHR/)" AND NOT rel MATCHES "${gl_allowed}")
            list(APPEND violations "${rel}: ${line}（OpenGL 头文件只能出现在渲染与平台实现中）")
        endif()
        if(line MATCHES "[<\"](imgui|imstb_)" AND NOT rel MATCHES "${imgui_allowed}")
            list(APPEND violations "${rel}: ${line}（ImGui 只能在界面模块与应用中使用）")
        endif()
        if(line MATCHES "[<\"]cgltf" AND NOT rel MATCHES "${cgltf_allowed}")
            list(APPEND violations "${rel}: ${line}（cgltf 只能在资产模块的实现中使用）")
        endif()
        if(line MATCHES "[<\"]sqlite3" AND NOT rel MATCHES "${sqlite_allowed}")
            list(APPEND violations "${rel}: ${line}（SQLite 只能在资源库模块的实现中使用，M3-07b）")
        endif()
        if(line MATCHES "[<\"]stb_" AND NOT rel MATCHES "${stb_allowed}")
            list(APPEND violations "${rel}: ${line}（stb_image 只能在渲染实现中使用）")
        endif()
        if(line MATCHES "[<\"]mujoco/" AND rel MATCHES "^/engine/" AND NOT rel MATCHES "${mujoco_allowed}")
            list(APPEND violations "${rel}: ${line}（MuJoCo 头文件只能出现在 MuJoCo 后端实现中）")
        endif()
        if(line MATCHES "[<\"]nlohmann/" AND NOT rel MATCHES "${json_allowed}")
            list(APPEND violations "${rel}: ${line}（JSON 库只能在设置模块的实现中使用）")
        endif()
        if(line MATCHES "[<\"](X11/|wayland-|SDL|vulkan/)" AND rel MATCHES "^/(engine|apps)/")
            list(APPEND violations "${rel}: ${line}（窗口系统或其他图形 API 头文件不允许直接使用）")
        endif()
    endforeach()
endforeach()

if(checked EQUAL 0)
    message(FATAL_ERROR "没有找到任何待检查的文件，SOURCE_DIR 可能有误：${SOURCE_DIR}")
endif()
if(violations)
    string(REPLACE ";" "\n" violations "${violations}")
    message(FATAL_ERROR "渲染隔离检查失败：\n${violations}")
endif()
message(STATUS "已检查 ${checked} 个文件，窗口库、图形 API、界面库与 JSON 库头文件只出现在允许的位置")
