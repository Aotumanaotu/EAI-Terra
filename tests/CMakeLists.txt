# Terra 自动化测试入口（CTest）。
# M0-01 阶段只覆盖工程骨架：C++17 生效、构建信息、Headless 命令行行为与无图形依赖。
# 测试框架是否另选待定（D-015），当前测试为返回码判定的独立可执行文件。

add_executable(test_core_build_info core/test_build_info.cpp)
target_link_libraries(test_core_build_info PRIVATE terra::core)
terra_configure_target(test_core_build_info)
add_test(NAME core.build_info COMMAND test_core_build_info)

set(_terra_check_exit "${PROJECT_SOURCE_DIR}/cmake/TerraCheckExitCode.cmake")

add_test(NAME headless.help
    COMMAND "${CMAKE_COMMAND}" -DEXPECTED=0
            -DCOMMAND=$<TARGET_FILE:terra_headless> -DARGS=--help
            -P "${_terra_check_exit}")

add_test(NAME headless.version COMMAND terra_headless --version)
set_tests_properties(headless.version PROPERTIES
    PASS_REGULAR_EXPRESSION "^terra_headless ${PROJECT_VERSION} ")

add_test(NAME headless.unknown_argument
    COMMAND "${CMAKE_COMMAND}" -DEXPECTED=2
            -DCOMMAND=$<TARGET_FILE:terra_headless> -DARGS=--no-such-option
            -P "${_terra_check_exit}")

# 无显示会话运行：清空 DISPLAY / WAYLAND_DISPLAY 后仍须正常退出。
# 这只证明入口不需要显示服务；二进制依赖由下面的 headless.no_graphics_libs 检查。
add_test(NAME headless.no_display
    COMMAND "${CMAKE_COMMAND}" -E env --unset=DISPLAY --unset=WAYLAND_DISPLAY
            "${CMAKE_COMMAND}" -DEXPECTED=0
            -DCOMMAND=$<TARGET_FILE:terra_headless> -DARGS=--version
            -P "${_terra_check_exit}")

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_test(NAME headless.no_graphics_libs
        COMMAND "${CMAKE_COMMAND}" -DBINARY=$<TARGET_FILE:terra_headless>
                -P "${PROJECT_SOURCE_DIR}/cmake/TerraCheckNoGraphicsLibs.cmake")
endif()
