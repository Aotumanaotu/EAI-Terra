# Terra 自动化测试入口（CTest）。
# M0-01：C++17 生效、构建信息、Headless 命令行行为与无图形依赖。
# M0-02：MuJoCo 依赖探针、后端版本一致性、公共头不泄露 MuJoCo。
# M0-03：Headless 参数解析、校验、退出码、--dry-run 输出。
# M0-04：MJCF 加载与初始化、错误分类、生命周期；Headless 接入模型加载。
# M0-05：固定步进、reset、故障检测；Headless 实际运行与 JSON Lines 记录。
# M0-06：状态快照（速度、关节、接触、诊断）、不可变性、立方体主基准中的速度与接触判定。
# M0-07：后端无关接口、执行器描述与控制输入、错误语义、只依赖 Terra 类型的调用方。
# M0-08：主场景正式判定与冻结容差、无重力、reset 再运行、生命周期、连续运行、后端致命错误。
# M0-09：性能基线程序冒烟测试。
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

set(_terra_cube_model "${PROJECT_SOURCE_DIR}/assets/tests/falling_cube.xml")
# 占位文件：存在且可读，但不是合法 MJCF。用于参数错误用例（不会读到模型）和模型编译失败用例。
set(_terra_placeholder_model "${CMAKE_CURRENT_BINARY_DIR}/fixtures/placeholder.xml")
file(WRITE "${_terra_placeholder_model}" "<!-- 不是合法的 MJCF -->\n")

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
terra_add_cli_test(invalid_mjcf EXPECT 4 STDERR "MJCF 加载失败"
    ARGS --model ${_terra_placeholder_model} --steps 10 --dry-run)
terra_add_cli_test(dry_run_steps EXPECT 0
    STDOUT "^[{].type.:.config.,.*,.steps.:5000,.timestep.:0.001,.total_steps.:5000,.sample_every.:10,.dry_run.:true}\n$"
    ARGS --model ${_terra_cube_model} --steps 5000 --sample-every=10 --dry-run)
terra_add_cli_test(dry_run_duration EXPECT 0
    STDOUT "^[{].type.:.config.,.*,.duration.:5,.timestep.:0.001,.total_steps.:5000,.sample_every.:1,.dry_run.:true}\n$"
    ARGS --dry-run --duration=5 --model=${_terra_cube_model})
# 实际运行（M0-05）。CMake 正则中 . 可匹配换行，用于检查多行输出的顺序。
terra_add_cli_test(run_cube EXPECT 0
    STDOUT "^[{].type.:.config.[^\n]*\n[{].type.:.model.[^\n]*\n[{].type.:.state.,.run_epoch.:0,.step.:0,.time.:0,[^\n]*\n[{].type.:.state.,.run_epoch.:0,.step.:50,[^\n]*\n[{].type.:.state.,.run_epoch.:0,.step.:100,[^\n]*\n[{].type.:.summary.,.run_epoch.:0,.steps.:100,.time.:0.1[0-9]*,.status.:.ok.}\n$"
    ARGS --model ${_terra_cube_model} --steps 100 --sample-every 50)
terra_add_cli_test(run_duration EXPECT 0
    STDOUT "[{].type.:.summary.,.run_epoch.:0,.steps.:10,"
    ARGS --model ${_terra_cube_model} --duration 0.01 --sample-every 1000)
terra_add_cli_test(run_sparse_sampling EXPECT 0
    STDOUT "^[^\n]*\n[^\n]*\n[^\n]*.step.:0,[^\n]*\n[^\n]*.step.:3,[^\n]*\n[^\n]*summary[^\n]*\n$"
    ARGS --model ${_terra_cube_model} --steps 3 --sample-every 100)
terra_add_cli_test(run_fault_runaway EXPECT 5
    STDOUT "[{].type.:.error.,.kind.:.non_finite.,.run_epoch.:0,.step.:5,"
    STDERR "第 5 步出错"
    ARGS --model ${PROJECT_SOURCE_DIR}/assets/tests/fault_runaway.xml --steps 20)
# 运行内存不足（M0-08）：MuJoCo 编译时试跑一步，栈溢出被编译器捕获，Terra 报告为模型错误。
terra_add_cli_test(arena_too_small EXPECT 4 STDERR "MJCF 加载失败.*mj_stackAlloc"
    ARGS --model ${PROJECT_SOURCE_DIR}/assets/tests/fault_arena_too_small.xml --steps 10 --dry-run)
terra_add_cli_test(run_fault_auto_reset EXPECT 5
    STDOUT "[{].type.:.error.,.kind.:.auto_reset.,.run_epoch.:0,.step.:1,"
    ARGS --model ${PROJECT_SOURCE_DIR}/assets/tests/fault_bad_acceleration.xml --steps 20)

add_test(NAME headless.version COMMAND terra_headless --version)
set_tests_properties(headless.version PROPERTIES
    PASS_REGULAR_EXPRESSION "^terra_headless ${PROJECT_VERSION} .*\nphysics backend: MuJoCo ${TERRA_MUJOCO_VERSION}\n")

# 无显示会话运行：清空 DISPLAY / WAYLAND_DISPLAY 后，--version 与 --dry-run 仍须正常退出。
# 这只证明入口不需要显示服务；二进制依赖由下面的 headless.no_graphics_libs 检查。
terra_add_cli_test(no_display EXPECT 0 NO_DISPLAY ARGS --version)
terra_add_cli_test(no_display_dry_run EXPECT 0 NO_DISPLAY STDOUT "dry_run.:true"
    ARGS --model ${_terra_cube_model} --steps 1 --dry-run)

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

# MJCF 加载与初始化（M0-04）。在工作目录中生成临时模型文件；也检查不会留下 MUJOCO_LOG.TXT。
add_executable(test_mujoco_world physics/test_mujoco_world.cpp)
target_link_libraries(test_mujoco_world PRIVATE terra::physics_mujoco)
target_compile_definitions(test_mujoco_world PRIVATE
    TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests"
    TERRA_TEST_WORK_DIR="${CMAKE_CURRENT_BINARY_DIR}/work/mujoco_world")
terra_configure_target(test_mujoco_world)
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/work")
add_test(NAME physics.mujoco_world COMMAND test_mujoco_world
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/work")

# 固定步进与 reset（M0-05）
add_executable(test_mujoco_step physics/test_mujoco_step.cpp)
target_link_libraries(test_mujoco_step PRIVATE terra::physics_mujoco terra::core)
target_compile_definitions(test_mujoco_step PRIVATE
    TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests")
terra_configure_target(test_mujoco_step)
add_test(NAME physics.mujoco_step COMMAND test_mujoco_step)

# 状态读取与快照（M0-06）
add_executable(test_mujoco_state physics/test_mujoco_state.cpp)
target_link_libraries(test_mujoco_state PRIVATE terra::physics_mujoco)
target_compile_definitions(test_mujoco_state PRIVATE
    TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests"
    TERRA_TEST_WORK_DIR="${CMAKE_CURRENT_BINARY_DIR}/work/mujoco_state")
terra_configure_target(test_mujoco_state)
add_test(NAME physics.mujoco_state COMMAND test_mujoco_state)

# 执行器描述与控制输入（M0-07）
add_executable(test_physics_control physics/test_physics_control.cpp)
target_link_libraries(test_physics_control PRIVATE terra::physics_mujoco)
target_compile_definitions(test_physics_control PRIVATE
    TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests"
    TERRA_TEST_WORK_DIR="${CMAKE_CURRENT_BINARY_DIR}/work/physics_control")
terra_configure_target(test_physics_control)
add_test(NAME physics.control COMMAND test_physics_control)

# 后端无关调用方（M0-07 ⑥）：调用方库只链接仅头文件的 terra::physics，没有 MuJoCo 头文件路径；
# 能编译即证明调用方不需要 MuJoCo。测试程序再链接后端，用于创建世界。
add_library(terra_test_physics_consumer STATIC physics/consumer/physics_consumer.cpp)
target_include_directories(terra_test_physics_consumer PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/physics/consumer")
target_link_libraries(terra_test_physics_consumer PUBLIC terra::physics)
terra_configure_target(terra_test_physics_consumer)

add_executable(test_physics_consumer physics/test_physics_consumer.cpp)
target_link_libraries(test_physics_consumer PRIVATE terra_test_physics_consumer terra::physics_mujoco)
target_compile_definitions(test_physics_consumer PRIVATE
    TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests")
terra_configure_target(test_physics_consumer)
add_test(NAME physics.consumer COMMAND test_physics_consumer)

# 物理场景验证（M0-08）：主场景正式判定、无重力、reset 再运行、生命周期。门槛见 physics/m0_tolerances.hpp。
add_executable(test_physics_scenarios physics/test_physics_scenarios.cpp)
target_link_libraries(test_physics_scenarios PRIVATE terra::physics_mujoco)
target_compile_definitions(test_physics_scenarios PRIVATE
    TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests")
terra_configure_target(test_physics_scenarios)
add_test(NAME physics.scenarios COMMAND test_physics_scenarios)

# 连续运行 60 s 仿真时间（M0-08），记录内存走势。
add_executable(test_physics_continuous physics/test_physics_continuous.cpp)
target_link_libraries(test_physics_continuous PRIVATE terra::physics_mujoco)
target_compile_definitions(test_physics_continuous PRIVATE
    TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests")
terra_configure_target(test_physics_continuous)
add_test(NAME physics.continuous COMMAND test_physics_continuous)

# 后端不可恢复错误（M0-08）：独立子进程中触发 mju_error，判定退出码、stderr 与不写日志文件。
# 与 test_mujoco_probe 一样直接链接 MuJoCo（后端测试）。
add_executable(test_backend_fatal physics/test_backend_fatal.cpp)
target_link_libraries(test_backend_fatal PRIVATE terra::physics_mujoco mujoco::mujoco)
terra_configure_target(test_backend_fatal)
add_test(NAME physics.backend_fatal
    COMMAND "${CMAKE_COMMAND}" -DEXPECTED=1 -DCOMMAND=$<TARGET_FILE:test_backend_fatal>
            "-DSTDERR_REGEX=terra fatal probe 42.*probe-atexit: no log file"
            -P "${_terra_check_exit}"
            "${PROJECT_SOURCE_DIR}/assets/tests/falling_cube.xml"
            "${CMAKE_CURRENT_BINARY_DIR}/work/backend_fatal")

# 性能基线程序冒烟测试（M0-09）：只确认能运行并输出 BENCH 记录；正式测量由验收脚本执行，不设门槛。
add_test(NAME bench.smoke
    COMMAND terra_bench --model "${PROJECT_SOURCE_DIR}/assets/tests/falling_cube.xml"
            --steps 100 --repeat 1 --warmup 1 --sample-every 10)
set_tests_properties(bench.smoke PROPERTIES
    PASS_REGULAR_EXPRESSION "BENCH mode=step .*BENCH mode=snapshot .*BENCH mode=jsonl ")
