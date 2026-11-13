# 用法：cmake -DBINARY=<可执行文件> -P TerraCheckNoGraphicsLibs.cmake
# 用 ldd 检查运行时依赖中不含 OpenGL / EGL / OSMesa / X11 / Wayland / GLFW 等图形库。
if(NOT DEFINED BINARY)
    message(FATAL_ERROR "需要 -DBINARY")
endif()

find_program(LDD_EXECUTABLE ldd)
if(NOT LDD_EXECUTABLE)
    message(FATAL_ERROR "未找到 ldd，无法检查运行时依赖")
endif()

execute_process(
    COMMAND "${LDD_EXECUTABLE}" "${BINARY}"
    RESULT_VARIABLE result
    OUTPUT_VARIABLE deps
    ERROR_VARIABLE err)
if(NOT result EQUAL 0)
    message(FATAL_ERROR "ldd 执行失败（${result}）：${err}")
endif()

string(REGEX MATCHALL
    "lib(GL|GLX|GLU|GLdispatch|OpenGL|EGL|GLES[^ ]*|OSMesa|X11|xcb|wayland-[^ ]*|glfw|vulkan)\\.so[^ ]*"
    hits "${deps}")
if(hits)
    message(FATAL_ERROR "发现图形相关运行时依赖：${hits}\n完整 ldd 输出：\n${deps}")
endif()
message(STATUS "未发现图形相关运行时依赖")
