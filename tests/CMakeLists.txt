# Terra 自动化测试入口（CTest）。
# M0-01：C++17 生效、构建信息、Headless 命令行行为与无图形依赖。
# M0-02：MuJoCo 依赖探针、后端版本一致性、公共头不泄露 MuJoCo。
# M0-03：Headless 参数解析、校验、退出码、--dry-run 输出。
# 测试框架是否另选待定（D-015），当前测试为返回码判定的独立可执行文件。

add_executable(test_core_build_info core/test_build_info.cpp)
target_link_libraries(test_core_build_info PRIVATE terra::core)
terra_configure_target(test_core_build_info)
add_test(NAME core.build_info COMMAND test_core_build_info)

set(_terra_check_exit "${PROJECT_SOURCE_DIR}/cmake/TerraCheckExitCode.cmake")

# terra_add_cli_test(<名称> EXPECT <退出码> [STDOUT <正则>] [STDERR <正则>] [NO_DISPLAY] ARGS <参数...>)
# 运行 terra_headless，精确比较退出码，并可用正则检查输出。NO_DISPLAY 表示清除显示变量后运行。
function(terra_add_cli_test name)
    cmake_parse_arguments(T "NO_DISPLAY" "EXPECT;STDOUT;STDERR" "ARGS" ${ARGN})
    set(cmd "${CMAKE_COMMAND}" -DEXPECTED=${T_EXPECT} -DCOMMAND=$<TARGET_FILE:terra_headless>)
    if(DEFINED T_STDOUT)
        list(APPEND cmd "-DSTDOUT_REGEX=${T_STDOUT}")
    endif()
    if(DEFINED T_STDERR)
        list(APPEND cmd "-DSTDERR_REGEX=${T_STDERR}")
    endif()
    list(APPEND cmd -P "${_terra_check_exit}" ${T_ARGS})
    if(T_NO_DISPLAY)
        set(cmd "${CMAKE_COMMAND}" -E env --unset=DISPLAY --unset=WAYLAND_DISPLAY ${cmd})
    endif()
    add_test(NAME headless.${name} COMMAND ${cmd})
endfunction()

# 参数解析单元测试
add_executable(test_headless_cli apps/test_headless_cli.cpp)
target_link_libraries(test_headless_cli PRIVATE terra_headless_cli)
terra_configure_target(test_headless_cli)
add_test(NAME headless.cli_parser COMMAND test_headless_cli)

# 占位模型文件：--dry-run 只检查文件可读，不解析 MJCF。真实测试模型由 M0-04 加入 assets/tests/。
set(_terra_placeholder_model "${CMAKE_CURRENT_BINARY_DIR}/fixtures/placeholder.xml")
file(WRITE "${_terra_placeholder_model}" "<!-- --dry-run 用占位文件，内容不会被解析 -->\n")

terra_add_cli_test(help EXPECT 0 STDOUT "用法" ARGS --help)
terra_add_cli_test(no_arguments EXPECT 2 STDERR "缺少参数" ARGS)
terra_add_cli_test(unknown_argument EXPECT 2 STDERR "未知参数" ARGS --no-such-option)
terra_add_cli_test(missing_model EXPECT 2 STDERR "缺少 --model" ARGS --steps 10)
terra_add_cli_test(steps_and_duration EXPECT 2 STDERR "不能同时使用"
    ARGS --model ${_terra_placeholder_model} --steps 10 --duration 1)
terra_add_cli_test(invalid_steps EXPECT 2 STDERR "--steps 需要正整数"
    ARGS --model ${_terra_placeholder_model} --steps -5)
terra_add_cli_test(model_not_found EXPECT 4 STDERR "模型文件不存在"
    ARGS --model ${CMAKE_CURRENT_BINARY_DIR}/fixtures/no_such_model.xml --steps 10 --dry-run)
terra_add_cli_test(model_is_directory EXPECT 4 STDERR "不是常规文件"
    ARGS --model ${CMAKE_CURRENT_BINARY_DIR}/fixtures --steps 10 --dry-run)
terra_add_cli_test(dry_run_steps EXPECT 0
    STDOUT "^[{].type.:.config.,.*,.steps.:5000,.sample_every.:10,.dry_run.:true}\n$"
    ARGS --model ${_terra_placeholder_model} --steps 5000 --sample-every=10 --dry-run)
terra_add_cli_test(dry_run_duration EXPECT 0
    STDOUT "^[{].type.:.config.,.*,.duration.:5,.sample_every.:1,.dry_run.:true}\n$"
    ARGS --dry-run --duration=5 --model=${_terra_placeholder_model})
terra_add_cli_test(run_not_available EXPECT 3 STDERR "尚未接入"
    ARGS --model ${_terra_placeholder_model} --steps 10)

add_test(NAME headless.version COMMAND terra_headless --version)
set_tests_properties(headless.version PROPERTIES
    PASS_REGULAR_EXPRESSION "^terra_headless ${PROJECT_VERSION} .*\nphysics backend: MuJoCo ${TERRA_MUJOCO_VERSION}\n")

# 无显示会话运行：清空 DISPLAY / WAYLAND_DISPLAY 后，--version 与 --dry-run 仍须正常退出。
# 这只证明入口不需要显示服务；二进制依赖由下面的 headless.no_graphics_libs 检查。
terra_add_cli_test(no_display EXPECT 0 NO_DISPLAY ARGS --version)
terra_add_cli_test(no_display_dry_run EXPECT 0 NO_DISPLAY STDOUT "dry_run.:true"
    ARGS --model ${_terra_placeholder_model} --steps 1 --dry-run)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_test(NAME headless.no_graphics_libs
        COMMAND "${CMAKE_COMMAND}" -DBINARY=$<TARGET_FILE:terra_headless>
                -P "${PROJECT_SOURCE_DIR}/cmake/TerraCheckNoGraphicsLibs.cmake")
endif()

# MuJoCo 依赖探针：唯一直接链接 mujoco::mujoco 的测试，独立定位依赖问题。
add_executable(test_mujoco_probe physics/test_mujoco_probe.cpp)
target_link_libraries(test_mujoco_probe PRIVATE mujoco::mujoco)
target_compile_definitions(test_mujoco_probe PRIVATE
    TERRA_MUJOCO_EXPECTED_VERSION="${TERRA_MUJOCO_VERSION}")
terra_configure_target(test_mujoco_probe)
add_test(NAME physics.mujoco_probe
    COMMAND "${CMAKE_COMMAND}" -E env --unset=DISPLAY --unset=WAYLAND_DISPLAY
            $<TARGET_FILE:test_mujoco_probe>)

add_executable(test_mujoco_backend_info physics/test_mujoco_backend_info.cpp)
target_link_libraries(test_mujoco_backend_info PRIVATE terra::physics_mujoco)
target_compile_definitions(test_mujoco_backend_info PRIVATE
    TERRA_MUJOCO_EXPECTED_VERSION="${TERRA_MUJOCO_VERSION}")
terra_configure_target(test_mujoco_backend_info)
add_test(NAME physics.mujoco_backend_info COMMAND test_mujoco_backend_info)

add_test(NAME boundary.no_mujoco_in_public_headers
    COMMAND "${CMAKE_COMMAND}" -DSOURCE_DIR=${PROJECT_SOURCE_DIR}
            -P "${PROJECT_SOURCE_DIR}/cmake/TerraCheckNoMuJoCoInPublicHeaders.cmake")
