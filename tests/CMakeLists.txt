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
# M1-02：渲染依赖探针（仅 TERRA_BUILD_RENDER=ON，graphics 标签，Xvfb + llvmpipe）。
# M1-03/04：输入系统（总是运行）、渲染隔离边界检查（总是运行）；GLFW 翻译表与平台窗口（仅渲染开启）。
# M1-06：渲染核心（仅渲染开启，graphics 标签）：离屏绘制与像素回读、深度测试、着色器错误定位、调试输出、资源 RAII。
# M1-07：相机与视口相机控制器（仅渲染开启；自动化部分不需要显示）。
# M1-12：显示模式（graphics.platform_display，仅渲染开启，Xvfb）。
# M1-13：界面输入与文本（ui.input_bridge、ui.text，不需要显示）；查看器自检（graphics.viewer_selftest，Xvfb）。
# M1-08a：网格与 PBR 数学（render.math）、PBR 场景渲染（graphics.render_pbr）。
# M1-15：性能监视数据层（core.system_monitor，总是运行）。
# M1-10：参考图比较规则（render.image_compare）、三个场景的参考图（graphics.render_reference.*）、渲染基准冒烟。
# M1-09：glTF 资产加载（asset.gltf）；贴图材质、切线、环境遮蔽、环境贴图中的太阳（render.math、graphics.render_pbr）。
# M1-14：设置模型与 JSON 读写（settings.store，总是运行）；输入替换绑定与键位名称（input.system）；相机按键改绑（view.camera）。
# M2-02：致命错误报告与紧急保存（core.fatal）；后端致命错误改为退出码 70（physics.backend_fatal）。
# M2-03：几何体与网格描述（physics.geometry）。
# M2-04：模型实例组合（scene.composition）。
# M2-05：场景文件（scene.file）；场景实例与视觉部件（scene.instance）。
# M2-06：仿真时钟与插值（simulation.logic）；仿真推进与帧率无关性（simulation.runner）。
# M2-07：场景渲染参考图（graphics.render_reference.carts）；glTF 节点表与子树变换（asset.gltf）。
# M3-01：参考机器人导入核查（robots.import）。
# M3-04a：动作文件（motion.file）、动作播放（motion.player）。
# M3-04b：PD 控制层与快照的执行器状态（simulation.pd）；悬挂模式（physics.fixed_base）。
# M4-08b：导入配方（import.analyze 与 import.execute 扩充）；TurtleBot 4（robots.turtlebot4_import；ROS2 构建：ros2.turtlebot4）。
# M4-08a：模型导入（import.analyze、import.execute、import.demos、import.cli_dry_run、import.cli_broken；
#   graphics.viewer_import_selftest：查看器的导入向导）。
# M4-07：传感器记录（record.sensors；record.file 扩充：版本 2 与读取版本 1；headless.record_sensors、headless.rerun_sensors；
#   graphics.viewer_sensor_record_selftest：查看器记录传感器与相机图像、复现、回放时传感器面板跟随）。
# M4-06a：ROS2 桥接。没有 ROS2 的构建：--ros2 提示安装并返回 3（headless.ros2_unavailable）。ROS2 构建（TERRA_ROS2_ENABLED，
#   需先 source /opt/ros/humble/setup.bash）：通信配置（ros2.profile）、端到端（ros2.bridge_headless、ros2.bridge_commands、
#   graphics.ros2_bridge_viewer，标签 ros2）、配置错误（headless.ros2_bad_profile）。
# M4-06b：宇树低层控制场景（robots.unitree_lowlevel，不需要 ROS2）。找到宇树消息包的 ROS2 构建（TERRA_ROS2_UNITREE，需再 source
#   unitree_ros2 的 cyclonedds_ws/install/setup.bash）：CRC 与官方结构体逐字节对照（ros2.unitree_crc）；端到端（ros2.unitree_go2、
#   ros2.unitree_g1、ros2.unitree_as2）；宇树官方示例原样运行（ros2.unitree_official_go2、_go2w、_g1；找不到示例程序时跳过）。
# M4-05：宇树官方模型 Go2W、AS2 的导入核查（robots.unitree_import）。
# M4-04：查看器传感器自检（graphics.viewer_sensor_selftest）、terra_capture（graphics.capture_determinism、capture.no_cameras）。
# M4-03b：2D 激光雷达（simulation.lidar）；Headless --sensors 输出雷达（headless.sensors_lidar）。
# M4-03a：相机模型（scene.camera_model）、相机传感器的渲染（graphics.camera_capture）。
# M4-02：IMU（physics.imu）、场景文件的传感器（scene.sensors）、采样时刻（simulation.sensor_sampling）；Headless --sensors（headless.sensors_imu 等）。
# M3-04c：预定义动作（robots.motions）；查看器播放动作的自检（graphics.viewer_motion_selftest）。
# M3-07c：查看器记录与回放（graphics.viewer_record_selftest）；记录采样率设置（settings.store 扩充）。
# M3-07b：资源库（library.store）；Headless --library / --list-records（headless.list_records）。
# M3-07a：SHA-256（core.sha256）；实验记录（record.file、record.rerun）；Headless --record / --rerun（headless.record_motion 等）。
# M3-06：关节与接触验证（robots.validation）；实体质量（模型描述修订 5）。
# M3-05：Headless --motion（headless.run_motion 等）；Headless 与查看器路径逐位一致（motion.consistency、motion.consistency_go2）。
# M3-03：显示用描述（physics.visual_description）；MJCF 显示换算（render.mjcf_visual）；参考图 mjcf_materials 与三台机器人。
# M3-02：关节限位与执行器类别、关键帧（physics.robot_description）；场景初始关键帧（scene.file、scene.instance、simulation.runner 扩充；graphics.viewer_keyframe_selftest）。
# M2-08：查看器场景模式自检（graphics.viewer_scene_selftest）；Headless 场景（headless.run_scene 等）；文件浏览（ui.text 扩充）。
# 测试框架是否另选待定（D-015），当前测试为返回码判定的独立可执行文件。

add_executable(test_core_build_info core/test_build_info.cpp)
target_link_libraries(test_core_build_info PRIVATE terra::core)
terra_configure_target(test_core_build_info)
add_test(NAME core.build_info COMMAND test_core_build_info)

# 性能监视数据层（M1-15）：滚动时间序列与 /proc 解析，总是运行。
add_executable(test_system_monitor core/test_system_monitor.cpp)
target_link_libraries(test_system_monitor PRIVATE terra::core)
terra_configure_target(test_system_monitor)
add_test(NAME core.system_monitor COMMAND test_system_monitor)

# 致命错误报告与紧急保存（M2-02）：报告正文、崩溃目录、紧急保存的顺序与异常隔离，总是运行。
add_executable(test_fatal core/test_fatal.cpp)
target_link_libraries(test_fatal PRIVATE terra::core)
terra_configure_target(test_fatal)
add_test(NAME core.fatal COMMAND test_fatal)

# SHA-256（M3-07）：实验记录中的文件摘要
add_executable(test_sha256 core/test_sha256.cpp)
target_link_libraries(test_sha256 PRIVATE terra::core)
terra_configure_target(test_sha256)
add_test(NAME core.sha256 COMMAND test_sha256 "${CMAKE_CURRENT_BINARY_DIR}")

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

# Headless 与查看器执行同一动作逐位相同（M3-05）：机械臂（不需要下载）；Go2（PD 层，需要参考机器人，否则跳过）
add_executable(test_motion_consistency apps/test_motion_consistency.cpp)
target_link_libraries(test_motion_consistency PRIVATE terra_headless_cli terra::scene terra::simulation terra::physics_mujoco)
terra_configure_target(test_motion_consistency)
add_dependencies(test_motion_consistency terra_headless)
add_test(NAME motion.consistency
    COMMAND test_motion_consistency $<TARGET_FILE:terra_headless>
            "${PROJECT_SOURCE_DIR}/assets/tests/scene/keyframe_arm.terra-scene.json"
            "${PROJECT_SOURCE_DIR}/assets/tests/motion/arm_swing.terra-motion.json" 900)
add_test(NAME motion.consistency_go2
    COMMAND test_motion_consistency $<TARGET_FILE:terra_headless>
            "${PROJECT_SOURCE_DIR}/assets/scenes/robots/go2.terra-scene.json"
            "${PROJECT_SOURCE_DIR}/assets/motions/go2/lie_stand_sit.terra-motion.json" 3000 --optional)
set_tests_properties(motion.consistency_go2 PROPERTIES SKIP_RETURN_CODE 77)

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
# 场景文件（M2-08）：组合模型实例后只跑物理；实体名带实例前缀；场景文件错误报告位置并以退出码 4 结束
terra_add_cli_test(run_scene EXPECT 0
    STDOUT "[{].type.:.config.[^\n]*.scene.:[^\n]*\n[{].type.:.model.[^\n]*cart_a/chassis[^\n]*cart_b/chassis.*[{].type.:.summary.,.run_epoch.:0,.steps.:200,"
    ARGS --scene ${PROJECT_SOURCE_DIR}/assets/tests/scene/two_carts.terra-scene.json --steps 200 --sample-every 100)
# M5-02a：有元素的场景在 Headless 中只跑物理（元素的实体名在模型描述中）
terra_add_cli_test(run_elements EXPECT 0
    STDOUT "[{].type.:.model.[^\n]*crate_a/body[^\n]*crate_b/body.*[{].type.:.summary.,.run_epoch.:0,.steps.:500,"
    ARGS --scene ${PROJECT_SOURCE_DIR}/assets/tests/elements/elements_bench.terra-scene.json --steps 500 --sample-every 250)
terra_add_cli_test(scene_invalid EXPECT 4 STDERR "场景错误 /models/0/positon"
    ARGS --scene ${PROJECT_SOURCE_DIR}/assets/tests/scene/invalid.terra-scene.json --steps 10)
terra_add_cli_test(scene_and_model EXPECT 2 STDERR "只能二选一"
    ARGS --scene a.json --model ${_terra_cube_model} --steps 10)
# M3-05：--motion（不给 --scene 时用推荐场景；不给步数时运行到动作结束后 1 秒：1.5 + 1 = 2.5 s = 1250 步；
# 初始关键帧与起始关键帧各重置一次，运行代次为 2）
terra_add_cli_test(run_motion EXPECT 0
    STDOUT "[{].type.:.config.[^\n]*.scene.:[^\n]*keyframe_arm[^\n]*.motion.:[^\n]*arm_swing.*[{].type.:.summary.,.run_epoch.:2,.steps.:1250,"
    ARGS --motion ${PROJECT_SOURCE_DIR}/assets/tests/motion/arm_swing.terra-motion.json --sample-every 1000)
terra_add_cli_test(motion_invalid EXPECT 4 STDERR "动作错误 /interpolation"
    ARGS --motion ${PROJECT_SOURCE_DIR}/assets/tests/motion/invalid.terra-motion.json --steps 10)
terra_add_cli_test(motion_wrong_scene EXPECT 4 STDERR "动作错误 /poses/0/targets/arm~1shoulder_pos：找不到执行器"
    ARGS --scene ${PROJECT_SOURCE_DIR}/assets/tests/scene/two_carts.terra-scene.json
         --motion ${PROJECT_SOURCE_DIR}/assets/tests/motion/arm_swing.terra-motion.json --steps 10)
terra_add_cli_test(motion_and_model EXPECT 2 STDERR "不能与 --model 同用"
    ARGS --model ${_terra_cube_model} --motion a.json)

# M3-07a：--record 写出实验记录，--rerun 复现（同一机器逐位相同）；两个测试按顺序执行（FIXTURES）
terra_add_cli_test(record_motion EXPECT 0
    STDOUT "[{].type.:.summary.,.run_epoch.:2,.steps.:1250,"
    ARGS --motion ${PROJECT_SOURCE_DIR}/assets/tests/motion/arm_swing.terra-motion.json --sample-every 10
         --record ${CMAKE_CURRENT_BINARY_DIR}/headless_record/arm.terra-record.jsonl
         --library ${CMAKE_CURRENT_BINARY_DIR}/headless_record/library.sqlite3)
set_tests_properties(headless.record_motion PROPERTIES FIXTURES_SETUP headless_record)
terra_add_cli_test(rerun_motion EXPECT 0
    STDOUT "[{].type.:.rerun.,[^\n]*.steps.:1250,.compared.:126,.bitwise.:true,.max_position_error.:0,.max_rotation_error.:0,[^\n]*.within_tolerance.:true"
    ARGS --rerun ${CMAKE_CURRENT_BINARY_DIR}/headless_record/arm.terra-record.jsonl)
set_tests_properties(headless.rerun_motion PROPERTIES FIXTURES_REQUIRED headless_record)
# M4-07：传感器记录——sensor_suite 200 步（IMU 随每份状态、雷达 10 Hz 共 5 次、相机元数据 30 Hz 共 13 帧）；
# 复现时 201 + 5 + 13 = 219 个传感器采样逐位相同
terra_add_cli_test(record_sensors EXPECT 0 STDERR "相机需要图形上下文"
    ARGS --scene ${PROJECT_SOURCE_DIR}/assets/tests/sensors/sensor_suite.terra-scene.json --steps 200
         --record ${CMAKE_CURRENT_BINARY_DIR}/headless_record_sensors/suite.terra-record.jsonl
         --library ${CMAKE_CURRENT_BINARY_DIR}/headless_record_sensors/library.sqlite3)
set_tests_properties(headless.record_sensors PROPERTIES FIXTURES_SETUP headless_record_sensors)
terra_add_cli_test(rerun_sensors EXPECT 0
    STDOUT "[{].type.:.rerun.,[^\n]*.steps.:200,.compared.:201,.bitwise.:true,[^\n]*.within_tolerance.:true,.sensor_samples.:219,.sensors_bitwise.:true"
    ARGS --rerun ${CMAKE_CURRENT_BINARY_DIR}/headless_record_sensors/suite.terra-record.jsonl)
set_tests_properties(headless.rerun_sensors PROPERTIES FIXTURES_REQUIRED headless_record_sensors)
# M3-07b：记录完成后已加入资源库；按场景筛选列出
# M4-02：--sensors 输出传感器清单（频率）与按频率采样的读数；不能与 --rerun 同用
terra_add_cli_test(sensors_imu EXPECT 0
    STDOUT "[{].type.:.sensor_list.[^\n]*.name.:.bench/bare_site.[^\n]*.rate_hz.:30.*[{].type.:.sensor.,.id.:[0-9]+,.name.:.bench/level_imu.,.kind.:.imu.,.step.:50,.time.:[^\n]*.linear_acceleration.:.-?0,-?0,9.81"
    ARGS --scene ${PROJECT_SOURCE_DIR}/assets/tests/sensors/imu_added.terra-scene.json --steps 50 --sample-every 50 --sensors)
terra_add_cli_test(cameras_ignored EXPECT 0 STDERR "2 个相机：相机需要图形上下文，在 Headless 中不可用"
    ARGS --scene ${PROJECT_SOURCE_DIR}/assets/tests/sensors/camera_bench.terra-scene.json --steps 5)
terra_add_cli_test(sensors_lidar EXPECT 0
    STDOUT "[{].type.:.lidar.,.name.:.bench/scan.,.step.:0,.time.:0,.rate_hz.:10,.angle_min.:-3.14159[^\n]*.ranges.:.[^\n]*[^0-9,]inf[^0-9,]"
    ARGS --scene ${PROJECT_SOURCE_DIR}/assets/tests/sensors/lidar_room.terra-scene.json --steps 1 --sensors)
terra_add_cli_test(sensors_with_rerun EXPECT 2 STDERR "--rerun 不能与"
    ARGS --rerun x.terra-record.jsonl --sensors)
terra_add_cli_test(list_records EXPECT 0
    STDOUT "[{].type.:.record.,.path.:[^\n]*headless_record/arm.terra-record.jsonl.,[^\n]*keyframe_arm[^\n]*.steps.:1250,[^\n]*.reason.:.completed."
    ARGS --list-records --library ${CMAKE_CURRENT_BINARY_DIR}/headless_record/library.sqlite3
         --scene ${PROJECT_SOURCE_DIR}/assets/tests/scene/keyframe_arm.terra-scene.json)
set_tests_properties(headless.list_records PROPERTIES FIXTURES_REQUIRED headless_record)
# M4-08c：--list-assets 扫描内置资产（资源库在构建目录中）：先输出 asset_scan，再每行一个资产；--kind 筛选
terra_add_cli_test(list_assets EXPECT 0
    STDOUT "[{].type.:.asset_scan.,[^\n]*.files.:[0-9]+,[^\n]*[}].*[{].type.:.asset.,.kind.:.scene.,.source.:.builtin.,.title.:.[^\n]*scenes/robots/go2.terra-scene.json"
    ARGS --list-assets --kind scene --library ${CMAKE_CURRENT_BINARY_DIR}/headless_assets/library.sqlite3)
terra_add_cli_test(list_assets_bad_kind EXPECT 2 STDERR "--kind 应为"
    ARGS --list-assets --kind mesh)
terra_add_cli_test(rerun_missing EXPECT 4 STDERR "记录文件不存在"
    ARGS --rerun ${CMAKE_CURRENT_BINARY_DIR}/no_such.terra-record.jsonl)
terra_add_cli_test(rerun_with_scene EXPECT 2 STDERR "--rerun 不能与"
    ARGS --rerun a.jsonl --scene b.json)

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

# 模型实例组合（M2-04）：terra_scene 的纯逻辑（假工厂）与 MuJoCo 后端集成（前缀、归属、互不串扰、逐位相同、配置与错误）。
add_executable(test_scene_composition scene/test_scene_composition.cpp)
target_link_libraries(test_scene_composition PRIVATE terra::scene terra::physics_mujoco)
target_compile_definitions(test_scene_composition PRIVATE
    TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests")
terra_configure_target(test_scene_composition)
add_test(NAME scene.composition COMMAND test_scene_composition)

# 场景文件读取与校验（M2-05）：纯逻辑，不需要物理后端。
add_executable(test_scene_file scene/test_scene_file.cpp)
target_link_libraries(test_scene_file PRIVATE terra::scene)
target_compile_definitions(test_scene_file PRIVATE TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests")
terra_configure_target(test_scene_file)
add_test(NAME scene.file COMMAND test_scene_file)

# 场景实例与视觉部件（M2-05）：映射、隐藏规则、视觉部件世界位姿（node / assembly 对齐）、诊断。
add_executable(test_scene_instance scene/test_scene_instance.cpp)
target_link_libraries(test_scene_instance PRIVATE terra::scene terra::physics_mujoco)
target_compile_definitions(test_scene_instance PRIVATE TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests")
terra_configure_target(test_scene_instance)
add_test(NAME scene.instance COMMAND test_scene_instance)

# M5-02a：场景元素（元素文件、组合、碰撞与掉落、温度与类别、错误）；夹具 assets/tests/elements/
add_executable(test_scene_elements scene/test_scene_elements.cpp)
target_link_libraries(test_scene_elements PRIVATE terra::scene terra::physics_mujoco)
target_compile_definitions(test_scene_elements PRIVATE TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests")
terra_configure_target(test_scene_elements)
add_test(NAME scene.elements COMMAND test_scene_elements)

# 参考机器人导入核查（M3-01）：三台 Menagerie 机器人经场景文件加载；未准备素材时跳过（返回码 77）。
add_executable(test_robot_import robots/test_robot_import.cpp)
target_link_libraries(test_robot_import PRIVATE terra::scene terra::simulation terra::physics_mujoco)
target_compile_definitions(test_robot_import PRIVATE TERRA_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets")
terra_configure_target(test_robot_import)
add_test(NAME robots.import COMMAND test_robot_import)
set_tests_properties(robots.import PROPERTIES SKIP_RETURN_CODE 77)

# 预定义动作（M3-04c）：assets/motions/ 中每个动作文件都能绑定并跑完，没有仿真错误
add_executable(test_robot_motions robots/test_robot_motions.cpp)
target_link_libraries(test_robot_motions PRIVATE terra::scene terra::simulation terra::physics_mujoco)
target_compile_definitions(test_robot_motions PRIVATE TERRA_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets")
terra_configure_target(test_robot_motions)
add_test(NAME robots.motions COMMAND test_robot_motions)
set_tests_properties(robots.motions PROPERTIES SKIP_RETURN_CODE 77)

# 宇树官方 MuJoCo 模型的导入核查（M4-05）：Go2W（轮足）与 AS2（unitree_hg 四足）；缺失时跳过
add_executable(test_unitree_import robots/test_unitree_import.cpp)
target_link_libraries(test_unitree_import PRIVATE terra::scene terra::physics_mujoco)
target_compile_definitions(test_unitree_import PRIVATE TERRA_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets")
terra_configure_target(test_unitree_import)
add_test(NAME robots.unitree_import COMMAND test_unitree_import)
set_tests_properties(robots.unitree_import PROPERTIES SKIP_RETURN_CODE 77)

# 模型导入（M4-08a）：analyze 不需要 MuJoCo；execute 需要 MuJoCo 与 assimp（demo.urdf 中有 Collada 网格）；
# demos 导入 ROBOTS/import_demos/ 中的公开示例（没有下载时跳过）；命令行程序的只分析模式与出错的模型
add_executable(test_import_analyze import/test_import.cpp)
target_link_libraries(test_import_analyze PRIVATE terra::import)
target_compile_definitions(test_import_analyze PRIVATE TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests")
terra_configure_target(test_import_analyze)
add_test(NAME import.analyze COMMAND test_import_analyze analyze)
add_executable(test_import_execute import/test_import.cpp)
target_link_libraries(test_import_execute PRIVATE terra::import terra::scene terra::physics_mujoco)
target_compile_definitions(test_import_execute PRIVATE TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests" TERRA_IMPORT_EXECUTE=1)
terra_configure_target(test_import_execute)
add_test(NAME import.execute COMMAND test_import_execute execute "${CMAKE_CURRENT_BINARY_DIR}/import_execute")
add_test(NAME import.demos COMMAND test_import_execute demos "${CMAKE_CURRENT_BINARY_DIR}/import_demos" "${PROJECT_SOURCE_DIR}/ROBOTS")
set_tests_properties(import.demos PROPERTIES SKIP_RETURN_CODE 77)
add_test(NAME import.cli_dry_run
    COMMAND "${CMAKE_COMMAND}" -DEXPECTED=0 -DCOMMAND=$<TARGET_FILE:terra_import_cli>
            "-DSTDOUT_REGEX=[{].type.:.import.,.dry_run.:true,.ok.:true,.name.:.demo_robot."
            "-DSTDERR_REGEX=10 个连杆、3 个活动关节、2 个网格、提议 3 个执行器与 3 个传感器"
            -P "${_terra_check_exit}" "${PROJECT_SOURCE_DIR}/assets/tests/import/demo_robot/urdf/demo.urdf" --dry-run)
add_test(NAME import.cli_broken
    COMMAND "${CMAKE_COMMAND}" -DEXPECTED=4 -DCOMMAND=$<TARGET_FILE:terra_import_cli>
            "-DSTDERR_REGEX=missing_mesh：[^\n]*no_such_mesh[^\n]*  → .*unknown_package：[^\n]*nonexistent_pkg[^\n]*  → "
            -P "${_terra_check_exit}" "${PROJECT_SOURCE_DIR}/assets/tests/import/broken.urdf"
            --output "${CMAKE_CURRENT_BINARY_DIR}/import_cli_broken")
# M5-02b：glTF 环境导入（不需要图形）：夹具 assets/tests/environment/（scripts/assets/make_environment_fixture.py 生成）；
# 用户的变电站与办公楼（blender/；没有时跳过，返回 77）；命令行程序 terra_import --environment
add_executable(test_environment_import import/test_environment_import.cpp)
target_link_libraries(test_environment_import PRIVATE terra::import terra::scene terra::physics_mujoco)
target_compile_definitions(test_environment_import PRIVATE TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests")
terra_configure_target(test_environment_import)
add_test(NAME import.environment COMMAND test_environment_import fixture "${CMAKE_CURRENT_BINARY_DIR}/environment_import")
add_test(NAME import.environment_assets COMMAND test_environment_import assets "${CMAKE_CURRENT_BINARY_DIR}/environment_assets" "${PROJECT_SOURCE_DIR}/blender")
set_tests_properties(import.environment_assets PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 600)
add_test(NAME import.cli_environment
    COMMAND "${CMAKE_COMMAND}" -DEXPECTED=0 -DCOMMAND=$<TARGET_FILE:terra_import_cli>
            "-DSTDOUT_REGEX=[{].type.:.environment.,.ok.:true,[^\n]*.elements.:8,[^\n]*.parts.:1,"
            -P "${_terra_check_exit}" --environment "${PROJECT_SOURCE_DIR}/assets/tests/environment/env_fixture.gltf"
            --output "${CMAKE_CURRENT_BINARY_DIR}/import_cli_environment" --overwrite)

# M4-08c：命令行导入后登记到资源库；资产包导出再解包到另一个资源库（逐字节相同）；Headless 列出解包后的机器人
add_test(NAME import.cli_pack
    COMMAND "${CMAKE_COMMAND}" -DIMPORT=$<TARGET_FILE:terra_import_cli> -DHEADLESS=$<TARGET_FILE:terra_headless>
            "-DINPUT=${PROJECT_SOURCE_DIR}/assets/tests/import/demo_robot/urdf/demo.urdf" "-DOUT=${CMAKE_CURRENT_BINARY_DIR}/import_cli_pack"
            -P "${PROJECT_SOURCE_DIR}/cmake/TerraCheckImportPack.cmake")

# TurtleBot 4 标准版（M4-08b）：按导入配方导入（两次逐字节相同）、传感器、差速运动学；描述文件缺失时跳过；需要 assimp
add_executable(test_turtlebot4 robots/test_turtlebot4.cpp)
target_link_libraries(test_turtlebot4 PRIVATE terra::import terra::scene terra::simulation terra::physics_mujoco)
target_compile_definitions(test_turtlebot4 PRIVATE TERRA_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets")
terra_configure_target(test_turtlebot4)
add_test(NAME robots.turtlebot4_import COMMAND test_turtlebot4 "${CMAKE_CURRENT_BINARY_DIR}/turtlebot4_import")
set_tests_properties(robots.turtlebot4_import PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 900)

# 宇树低层控制场景（M4-06b）：G1 的位置执行器改为力矩执行器（torqueActuators）；Go2 加机身 IMU、没有 PD 层
add_executable(test_unitree_lowlevel robots/test_unitree_lowlevel.cpp)
target_link_libraries(test_unitree_lowlevel PRIVATE terra::scene terra::physics_mujoco)
target_compile_definitions(test_unitree_lowlevel PRIVATE TERRA_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets")
terra_configure_target(test_unitree_lowlevel)
add_test(NAME robots.unitree_lowlevel COMMAND test_unitree_lowlevel)
set_tests_properties(robots.unitree_lowlevel PROPERTIES SKIP_RETURN_CODE 77)

# 关节与接触验证（M3-06）：每台参考机器人一组数值判据（冻结，见 M3-06 报告）
add_executable(test_robot_validation robots/test_robot_validation.cpp)
target_link_libraries(test_robot_validation PRIVATE terra::scene terra::simulation terra::physics_mujoco)
target_compile_definitions(test_robot_validation PRIVATE TERRA_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets")
terra_configure_target(test_robot_validation)
add_test(NAME robots.validation COMMAND test_robot_validation)
set_tests_properties(robots.validation PROPERTIES SKIP_RETURN_CODE 77)

# 仿真时钟与快照插值（M2-06）：纯逻辑，不需要物理后端。
add_executable(test_simulation_logic simulation/test_simulation_logic.cpp)
target_link_libraries(test_simulation_logic PRIVATE terra::simulation)
terra_configure_target(test_simulation_logic)
add_test(NAME simulation.logic COMMAND test_simulation_logic)

# 仿真推进（M2-06）：不同帧率与抖动下轨迹逐位相同、暂停、单步、reset 屏障、命令、故障。
add_executable(test_simulation_runner simulation/test_simulation_runner.cpp)
target_link_libraries(test_simulation_runner PRIVATE terra::simulation terra::scene terra::physics_mujoco)
target_compile_definitions(test_simulation_runner PRIVATE TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests")
terra_configure_target(test_simulation_runner)
add_test(NAME simulation.runner COMMAND test_simulation_runner)

# 动作文件与插值、播放器（M3-04a）：纯逻辑，不需要物理后端。
add_executable(test_motion_file simulation/test_motion_file.cpp)
target_link_libraries(test_motion_file PRIVATE terra::simulation)
target_compile_definitions(test_motion_file PRIVATE TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests")
terra_configure_target(test_motion_file)
add_test(NAME motion.file COMMAND test_motion_file)

# 动作的绑定与播放（M3-04a）：MuJoCo + 场景文件；与帧率无关、与预先排队的命令逐位相同。
add_executable(test_motion_player simulation/test_motion_player.cpp)
target_link_libraries(test_motion_player PRIVATE terra::simulation terra::scene terra::physics_mujoco)
target_compile_definitions(test_motion_player PRIVATE TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests")
terra_configure_target(test_motion_player)
add_test(NAME motion.player COMMAND test_motion_player)

# 实验记录（M3-07a）：写入与读取（纯逻辑）；复现（MuJoCo）
add_executable(test_record_file simulation/test_record.cpp)
target_link_libraries(test_record_file PRIVATE terra::simulation terra::core)
target_compile_definitions(test_record_file PRIVATE TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests")
terra_configure_target(test_record_file)
add_test(NAME record.file COMMAND test_record_file file "${CMAKE_CURRENT_BINARY_DIR}/record_file_tmp")
add_executable(test_record_rerun simulation/test_record.cpp)
target_link_libraries(test_record_rerun PRIVATE terra::simulation terra::core terra::scene terra::physics_mujoco)
target_compile_definitions(test_record_rerun PRIVATE TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests" TERRA_RECORD_RERUN=1)
terra_configure_target(test_record_rerun)
add_test(NAME record.rerun COMMAND test_record_rerun rerun "${CMAKE_CURRENT_BINARY_DIR}/record_rerun_tmp")
# M4-07：传感器的记录与复现（雷达距离含 ±inf 的往返、IMU、相机位姿；改动一处雷达距离后复现报告不同）
add_test(NAME record.sensors COMMAND test_record_rerun sensors "${CMAKE_CURRENT_BINARY_DIR}/record_sensors_tmp")

# 资源库（M3-07b）：SQLite 中的实验记录索引
add_executable(test_library library/test_library.cpp)
target_link_libraries(test_library PRIVATE terra::library terra::sqlite)
terra_configure_target(test_library)
add_test(NAME library.store COMMAND test_library "${CMAKE_CURRENT_BINARY_DIR}/library_tmp")
# M4-08c：资产表（扫描、增量、收藏、缺失、复制到新位置、版本升级）与资产包
add_executable(test_library_assets library/test_library_assets.cpp)
target_link_libraries(test_library_assets PRIVATE terra::library terra::sqlite)
terra_configure_target(test_library_assets)
add_test(NAME library.assets COMMAND test_library_assets "${CMAKE_CURRENT_BINARY_DIR}/library_assets_tmp")
add_executable(test_pack library/test_pack.cpp)
target_link_libraries(test_pack PRIVATE terra::library terra::core terra_miniz)
terra_configure_target(test_pack)
add_test(NAME library.pack COMMAND test_pack "${CMAKE_CURRENT_BINARY_DIR}/pack_tmp")
# M4-10a：可下载的运行组件（file:// 地址与本地文件，不需要网络）
add_executable(test_components library/test_components.cpp)
target_link_libraries(test_components PRIVATE terra::library terra::core terra_miniz)
target_compile_definitions(test_components PRIVATE TERRA_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets")
terra_configure_target(test_components)
add_test(NAME library.components COMMAND test_components "${CMAKE_CURRENT_BINARY_DIR}/components_tmp")
# M4-10a：策略（说明文件、ONNX 信息、推理后端、控制器、自动配置）；夹具 assets/tests/policy/（由 scripts/policy/make_fixture_onnx.py 生成 ONNX）
add_executable(test_policy_file policy/test_policy_file.cpp)
target_link_libraries(test_policy_file PRIVATE terra::policy)
target_compile_definitions(test_policy_file PRIVATE TERRA_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets")
terra_configure_target(test_policy_file)
add_test(NAME policy.file COMMAND test_policy_file "${CMAKE_CURRENT_BINARY_DIR}/policy_file_tmp")
add_executable(test_policy_controller policy/test_policy_controller.cpp)
target_link_libraries(test_policy_controller PRIVATE terra::policy terra::scene terra::physics_mujoco)
target_compile_definitions(test_policy_controller PRIVATE TERRA_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets")
terra_configure_target(test_policy_controller)
add_test(NAME policy.controller COMMAND test_policy_controller "${CMAKE_CURRENT_BINARY_DIR}/policy_controller_tmp")
# M4-10b：导入（文件类型、训练检查点 → ONNX、导入目录、草稿、TorchScript 缺少 Python 时的提示）
add_executable(test_policy_import policy/test_policy_import.cpp)
target_link_libraries(test_policy_import PRIVATE terra::policy terra::scene terra::physics_mujoco terra::core terra::json terra_miniz)
target_compile_definitions(test_policy_import PRIVATE TERRA_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets" TERRA_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
terra_configure_target(test_policy_import)
add_test(NAME policy.import COMMAND test_policy_import "${CMAKE_CURRENT_BINARY_DIR}/policy_import_tmp")
# ONNX Runtime 组件装在构建目录中的资源库（验证脚本用 terra_headless --install-component 安装，需要网络）；没有时跳过
set(TERRA_POLICY_LIBRARY "${CMAKE_BINARY_DIR}/policy_library/library.db" CACHE FILEPATH "策略测试用的资源库（其 components/ 中装 ONNX Runtime）")
get_filename_component(TERRA_POLICY_COMPONENTS "${TERRA_POLICY_LIBRARY}" DIRECTORY)
set(TERRA_POLICY_COMPONENTS "${TERRA_POLICY_COMPONENTS}/components")
add_executable(test_policy_onnxruntime policy/test_policy_onnxruntime.cpp)
target_link_libraries(test_policy_onnxruntime PRIVATE terra::policy terra::library terra::scene terra::physics_mujoco)
target_compile_definitions(test_policy_onnxruntime PRIVATE TERRA_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets")
terra_configure_target(test_policy_onnxruntime)
add_test(NAME policy.onnxruntime COMMAND test_policy_onnxruntime "${TERRA_POLICY_COMPONENTS}")
set_tests_properties(policy.onnxruntime PROPERTIES SKIP_RETURN_CODE 77)
# 用户的 Go2 策略（只读，不在仓库中）；没有时跳过
set(TERRA_GO2_POLICY "${PROJECT_SOURCE_DIR}/others/supermimic/go2_supermimic/2026-11-25_00-02-04_246580/policy.onnx" CACHE FILEPATH "用户的 Go2 策略（policy.go2_walk）")
add_executable(test_policy_go2 policy/test_policy_go2.cpp)
target_link_libraries(test_policy_go2 PRIVATE terra::policy terra::library terra::scene terra::physics_mujoco)
target_compile_definitions(test_policy_go2 PRIVATE TERRA_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets")
terra_configure_target(test_policy_go2)
add_test(NAME policy.go2_walk COMMAND test_policy_go2 "${TERRA_POLICY_COMPONENTS}" "${TERRA_GO2_POLICY}" "${CMAKE_CURRENT_BINARY_DIR}/policy_go2_metrics.json")
set_tests_properties(policy.go2_walk PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 600)
# Headless 的 --policy、--command、记录与复现、组件命令（线性夹具总是运行；ONNX Runtime 与用户策略的部分在组件已安装时运行）
find_program(TERRA_PYTHON3_EXE python3 REQUIRED)
add_test(NAME headless.policy
    COMMAND "${TERRA_PYTHON3_EXE}" "${CMAKE_CURRENT_SOURCE_DIR}/policy/check_policy.py" --headless $<TARGET_FILE:terra_headless>
            --assets "${PROJECT_SOURCE_DIR}/assets" --work "${CMAKE_CURRENT_BINARY_DIR}/headless_policy"
            --library "${TERRA_POLICY_LIBRARY}" --go2 "${TERRA_GO2_POLICY}")
set_tests_properties(headless.policy PROPERTIES TIMEOUT 600)
# M4-08d：显示配置（.terviz 往返、保留不认识的内容、导出器注册表、.rviz 导出）；不需要 ROS2
add_executable(test_viz viz/test_viz.cpp)
target_link_libraries(test_viz PRIVATE terra::viz)
terra_configure_target(test_viz)
add_test(NAME viz.config COMMAND test_viz "${CMAKE_CURRENT_BINARY_DIR}/viz_tmp")

# PD 控制层与快照的执行器状态（M3-04b）
add_executable(test_pd_layer simulation/test_pd_layer.cpp)
target_link_libraries(test_pd_layer PRIVATE terra::simulation terra::scene terra::physics_mujoco)
target_compile_definitions(test_pd_layer PRIVATE TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests")
terra_configure_target(test_pd_layer)
add_test(NAME simulation.pd COMMAND test_pd_layer)

# 传感器（M4-02）：IMU 真值（physics.imu）；场景文件的 sensors（scene.sensors，参考机器人缺失时跳过其中的机器人部分）；
# 采样时刻（simulation.sensor_sampling，纯逻辑）
add_executable(test_imu sensors/test_imu.cpp)
target_link_libraries(test_imu PRIVATE terra::physics_mujoco)
target_compile_definitions(test_imu PRIVATE TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests")
terra_configure_target(test_imu)
add_test(NAME physics.imu COMMAND test_imu)
add_executable(test_scene_sensors sensors/test_scene_sensors.cpp)
target_link_libraries(test_scene_sensors PRIVATE terra::scene terra::simulation terra::physics_mujoco)
target_compile_definitions(test_scene_sensors PRIVATE TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests"
                                                      TERRA_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets")
terra_configure_target(test_scene_sensors)
add_test(NAME scene.sensors COMMAND test_scene_sensors)
# M4-03：相机模型（内参、投影、MJCF 相机、场景文件的相机；纯数学，不需要图形）
add_executable(test_camera_model sensors/test_camera_model.cpp)
target_link_libraries(test_camera_model PRIVATE terra::scene terra::physics_mujoco)
target_compile_definitions(test_camera_model PRIVATE TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests")
terra_configure_target(test_camera_model)
add_test(NAME scene.camera_model COMMAND test_camera_model)
# M4-03b：2D 激光雷达（射线检测与解析解比较、visual / collision 模式、量程、场景文件）
add_executable(test_lidar sensors/test_lidar.cpp)
target_link_libraries(test_lidar PRIVATE terra::scene terra::simulation terra::physics_mujoco)
target_compile_definitions(test_lidar PRIVATE TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests")
terra_configure_target(test_lidar)
add_test(NAME simulation.lidar COMMAND test_lidar)
add_executable(test_sensor_sampling simulation/test_sensor_sampling.cpp)
target_link_libraries(test_sensor_sampling PRIVATE terra::simulation)
terra_configure_target(test_sensor_sampling)
add_test(NAME simulation.sensor_sampling COMMAND test_sensor_sampling)
# M4-09：传感器噪声（随机数、统计性质、可复现、关闭、雷达预设）；夹具 assets/tests/sensors/noise_suite.*
add_executable(test_sensor_noise sensors/test_sensor_noise.cpp)
target_link_libraries(test_sensor_noise PRIVATE terra::simulation terra::scene terra::physics_mujoco)
target_compile_definitions(test_sensor_noise PRIVATE TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests")
terra_configure_target(test_sensor_noise)
add_test(NAME simulation.sensor_noise COMMAND test_sensor_noise)
# M4-09：Headless 的噪声出口（两次逐字节相同、--no-noise、记录头与复现）
find_program(TERRA_PYTHON3_EXE python3 REQUIRED)
add_test(NAME headless.sensors_noise
    COMMAND "${TERRA_PYTHON3_EXE}" "${CMAKE_CURRENT_SOURCE_DIR}/sensors/check_noise.py" headless --headless $<TARGET_FILE:terra_headless>
            --scene "${PROJECT_SOURCE_DIR}/assets/tests/sensors/noise_suite.terra-scene.json" --work "${CMAKE_CURRENT_BINARY_DIR}/sensors_noise")
# M5-11：3D 激光雷达（场景文件与预设、几何、噪声统计、运动畸变、官方 Mid-360 图案——已下载时）；夹具 assets/tests/sensors/lidar3d_room.*
add_executable(test_lidar3d sensors/test_lidar3d.cpp)
target_link_libraries(test_lidar3d PRIVATE terra::simulation terra::scene terra::physics_mujoco)
target_compile_definitions(test_lidar3d PRIVATE TERRA_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets")
terra_configure_target(test_lidar3d)
add_test(NAME simulation.lidar3d COMMAND test_lidar3d)
# M5-11：Headless 与 terra_capture 的 3D 雷达出口、记录与复现（渲染关闭的构建只检查 Headless）
set(_terra_capture_args)
if(TERRA_BUILD_RENDER)
    set(_terra_capture_args --capture $<TARGET_FILE:terra_capture>)
endif()
add_test(NAME headless.lidar3d
    COMMAND "${TERRA_PYTHON3_EXE}" "${CMAKE_CURRENT_SOURCE_DIR}/sensors/check_lidar3d.py" --headless $<TARGET_FILE:terra_headless>
            ${_terra_capture_args} --scene "${PROJECT_SOURCE_DIR}/assets/tests/sensors/lidar3d_room.terra-scene.json"
            --work "${CMAKE_CURRENT_BINARY_DIR}/headless_lidar3d")

# M5-06：热像仪（预设、温度表与曲线、热点、辐射测温、色板、噪声统计；不需要图形）；夹具 assets/tests/sensors/thermal_bench.*
add_executable(test_thermal sensors/test_thermal.cpp)
target_link_libraries(test_thermal PRIVATE terra::simulation terra::scene terra::physics_mujoco)
target_compile_definitions(test_thermal PRIVATE TERRA_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets")
terra_configure_target(test_thermal)
add_test(NAME simulation.thermal COMMAND test_thermal)

# 悬挂模式（M3-04b）：fixedBase 删掉根部自由关节，关键帧保留
add_executable(test_fixed_base scene/test_fixed_base.cpp)
target_link_libraries(test_fixed_base PRIVATE terra::scene terra::physics_mujoco)
target_compile_definitions(test_fixed_base PRIVATE TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests")
terra_configure_target(test_fixed_base)
add_test(NAME physics.fixed_base COMMAND test_fixed_base)

# 几何体与网格描述（M2-03）：形状、尺寸、局部位姿、颜色、分组、网格、模型实例。
add_executable(test_mujoco_geometry physics/test_mujoco_geometry.cpp)
target_link_libraries(test_mujoco_geometry PRIVATE terra::physics_mujoco)
target_compile_definitions(test_mujoco_geometry PRIVATE
    TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests")
terra_configure_target(test_mujoco_geometry)
add_test(NAME physics.geometry COMMAND test_mujoco_geometry)

# 机器人描述（M3-02）：关节限位、执行器类别、关键帧与组合后的关键帧
add_executable(test_robot_description physics/test_robot_description.cpp)
target_link_libraries(test_robot_description PRIVATE terra::physics_mujoco)
target_compile_definitions(test_robot_description PRIVATE
    TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests")
terra_configure_target(test_robot_description)
add_test(NAME physics.robot_description COMMAND test_robot_description)

# 显示用描述（M3-03）：网格法线与纹理坐标、贴图、材质
add_executable(test_visual_description physics/test_visual_description.cpp)
target_link_libraries(test_visual_description PRIVATE terra::physics_mujoco)
target_compile_definitions(test_visual_description PRIVATE
    TERRA_TEST_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets/tests")
terra_configure_target(test_visual_description)
add_test(NAME physics.visual_description COMMAND test_visual_description)

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

# 后端不可恢复错误（M0-08 建立，M2-02 改为 Terra 的致命错误处理）：独立子进程中在 step 内触发 mju_error，
# 判定退出码 70、stderr、崩溃报告（写在构建目录，不写用户目录）、紧急保存与不写 MuJoCo 日志文件。
# 与 test_mujoco_probe 一样直接链接 MuJoCo（后端测试）。
add_executable(test_backend_fatal physics/test_backend_fatal.cpp)
target_link_libraries(test_backend_fatal PRIVATE terra::physics_mujoco terra::core mujoco::mujoco)
terra_configure_target(test_backend_fatal)
add_test(NAME physics.backend_fatal
    COMMAND "${CMAKE_COMMAND}" -DEXPECTED=70 -DCOMMAND=$<TARGET_FILE:test_backend_fatal>
            "-DSTDERR_REGEX=terra fatal probe 42.*探针设置：成功.*崩溃报告：.*probe-atexit: no log file, crash report ok, emergency save ok"
            -P "${_terra_check_exit}"
            "${PROJECT_SOURCE_DIR}/assets/tests/falling_cube.xml"
            "${CMAKE_CURRENT_BINARY_DIR}/work/backend_fatal")
set_tests_properties(physics.backend_fatal PROPERTIES
    ENVIRONMENT "TERRA_CRASH_DIR=${CMAKE_CURRENT_BINARY_DIR}/work/backend_fatal/crash")

# 性能基线程序冒烟测试（M0-09）：只确认能运行并输出 BENCH 记录；正式测量由验收脚本执行，不设门槛。
add_test(NAME bench.smoke
    COMMAND terra_bench --model "${PROJECT_SOURCE_DIR}/assets/tests/falling_cube.xml"
            --steps 100 --repeat 1 --warmup 1 --sample-every 10)
set_tests_properties(bench.smoke PROPERTIES
    PASS_REGULAR_EXPRESSION "BENCH mode=step .*BENCH mode=snapshot .*BENCH mode=jsonl ")

# ---- 输入（M1-04，总是构建，无图形依赖）----
add_executable(test_input_system input/test_input_system.cpp)
target_link_libraries(test_input_system PRIVATE terra::input)
terra_configure_target(test_input_system)
add_test(NAME input.system COMMAND test_input_system)

# ---- 设置（M1-14，总是构建，无图形依赖）----
add_executable(test_settings settings/test_settings.cpp)
target_link_libraries(test_settings PRIVATE terra::settings)
terra_configure_target(test_settings)
add_test(NAME settings.store COMMAND test_settings "${CMAKE_CURRENT_BINARY_DIR}/settings_tmp")

# 渲染隔离（M1-03/04）：GLFW、OpenGL 等头文件只出现在允许的位置；控制器与输入模块不得直接使用窗口库。
add_test(NAME boundary.render_isolation
    COMMAND "${CMAKE_COMMAND}" -DSOURCE_DIR=${PROJECT_SOURCE_DIR}
            -P "${PROJECT_SOURCE_DIR}/cmake/TerraCheckRenderIsolation.cmake")

# ---- 渲染（M1 起，仅 TERRA_BUILD_RENDER=ON）----
# 图形测试带 graphics 标签，可用 ctest -L graphics / -LE graphics 单独运行或排除（NFR-06）。
# 自动化图形测试固定在 Xvfb + Mesa llvmpipe 上运行（D-022）：清除 DISPLAY，强制 GLVND 选择 Mesa，
# 测试再断言渲染器为 llvmpipe，防止环境变化（例如选中 NVIDIA 的 GLX 库）导致结果不可比。
if(TERRA_BUILD_RENDER)
    find_program(TERRA_XVFB_RUN xvfb-run)
    if(NOT TERRA_XVFB_RUN)
        message(FATAL_ERROR "TERRA_BUILD_RENDER=ON 需要 xvfb-run 运行图形测试（sudo apt install xvfb）")
    endif()
    set(TERRA_GRAPHICS_TEST_ENV
        "${CMAKE_COMMAND}" -E env --unset=DISPLAY --unset=WAYLAND_DISPLAY
        __GLX_VENDOR_LIBRARY_NAME=mesa LIBGL_ALWAYS_SOFTWARE=1
        "${TERRA_XVFB_RUN}" -a -s "-screen 0 1280x720x24")

    # 渲染依赖探针（M1-02）：GLFW + glad + GLM，OpenGL 4.5 core 上下文、DSA 离屏清屏与回读。
    add_executable(test_render_deps render/test_render_deps.cpp)
    target_link_libraries(test_render_deps PRIVATE terra::glfw terra::glad terra::glm)
    terra_configure_target(test_render_deps)
    add_test(NAME graphics.render_deps COMMAND ${TERRA_GRAPHICS_TEST_ENV} $<TARGET_FILE:test_render_deps> llvmpipe)
    set_tests_properties(graphics.render_deps PROPERTIES LABELS graphics)

    # GLFW 编码翻译表（M1-04）：纯函数，不初始化 GLFW，不需要显示。
    add_executable(test_glfw_translation platform/test_glfw_translation.cpp)
    target_link_libraries(test_glfw_translation PRIVATE terra::platform_glfw terra::glfw)
    terra_configure_target(test_glfw_translation)
    add_test(NAME input.glfw_translation COMMAND test_glfw_translation)

    # 平台窗口（M1-03）：Xvfb + llvmpipe 中创建窗口与上下文；另一用例在无显示环境中确认返回 NoDisplay。
    add_executable(test_platform_window platform/test_platform_window.cpp)
    target_link_libraries(test_platform_window PRIVATE terra::platform_glfw terra::glad)
    terra_configure_target(test_platform_window)
    add_test(NAME graphics.platform_window COMMAND ${TERRA_GRAPHICS_TEST_ENV} $<TARGET_FILE:test_platform_window> window)
    add_test(NAME graphics.platform_no_display
        COMMAND "${CMAKE_COMMAND}" -E env --unset=DISPLAY --unset=WAYLAND_DISPLAY
                $<TARGET_FILE:test_platform_window> no-display)
    # 显示模式（M1-12）：窗口 / 无边框全屏 / 独占全屏切换与显示器查询，在 Xvfb 中运行。
    add_test(NAME graphics.platform_display COMMAND ${TERRA_GRAPHICS_TEST_ENV} $<TARGET_FILE:test_platform_window> display)
    set_tests_properties(graphics.platform_window graphics.platform_no_display graphics.platform_display PROPERTIES LABELS graphics)

    # 渲染核心（M1-06）。--show 模式供桌面展示，不在 CTest 中运行。
    add_executable(test_render_core render/test_render_core.cpp)
    target_link_libraries(test_render_core PRIVATE terra::render terra::platform_glfw terra::glad)
    terra_configure_target(test_render_core)
    add_test(NAME graphics.render_core COMMAND ${TERRA_GRAPHICS_TEST_ENV} $<TARGET_FILE:test_render_core> llvmpipe)
    set_tests_properties(graphics.render_core PROPERTIES LABELS graphics)

    # 网格与 PBR 数学（M1-08a）：纯 CPU，不需要显示。
    add_executable(test_render_math render/test_render_math.cpp)
    target_link_libraries(test_render_math PRIVATE terra::render)
    terra_configure_target(test_render_math)
    target_compile_definitions(test_render_math PRIVATE TERRA_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets")
    add_test(NAME render.math COMMAND test_render_math "${CMAKE_CURRENT_BINARY_DIR}/render_math_tmp")

    # PBR 场景渲染（M1-08a）：与 C++ 参考实现比较像素；阴影、MSAA、网格。
    add_executable(test_render_pbr render/test_render_pbr.cpp)
    target_link_libraries(test_render_pbr PRIVATE terra::render terra::view terra::platform_glfw)
    terra_configure_target(test_render_pbr)
    add_test(NAME graphics.render_pbr COMMAND ${TERRA_GRAPHICS_TEST_ENV} $<TARGET_FILE:test_render_pbr> llvmpipe)
    set_tests_properties(graphics.render_pbr PROPERTIES LABELS graphics)

    # glTF 资产加载（M1-09）：纯 CPU；用代码生成的小型 glTF / glb，另在真实资产存在时检查。
    add_executable(test_asset_gltf asset/test_asset_gltf.cpp)
    target_link_libraries(test_asset_gltf PRIVATE terra::asset)
    target_compile_definitions(test_asset_gltf PRIVATE TERRA_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets")
    terra_configure_target(test_asset_gltf)
    add_test(NAME asset.gltf COMMAND test_asset_gltf "${CMAKE_CURRENT_BINARY_DIR}/asset_gltf_tmp")

    # MJCF 几何体的显示换算（M3-03）：网格法线、纹理坐标、材质与贴图的换算；纯 CPU，不创建 GL 上下文。
    add_executable(test_mjcf_visual render/test_mjcf_visual.cpp)
    target_link_libraries(test_mjcf_visual PRIVATE terra::viewer_scene)
    terra_configure_target(test_mjcf_visual)
    add_test(NAME render.mjcf_visual COMMAND test_mjcf_visual)

    # 参考图比较规则（M1-10）：纯 CPU。
    add_executable(test_image_compare render/test_image_compare.cpp)
    target_link_libraries(test_image_compare PRIVATE terra::render)
    terra_configure_target(test_image_compare)
    add_test(NAME render.image_compare COMMAND test_image_compare "${CMAKE_CURRENT_BINARY_DIR}/image_compare_tmp")

    # 参考图（M1-10）：与 tests/render/reference/ 中的 PNG 比较；带资产的场景在素材缺失时跳过（返回码 77）。
    add_executable(test_render_reference render/test_render_reference.cpp)
    target_link_libraries(test_render_reference PRIVATE terra::viewer_scene terra::render terra::view terra::platform_glfw terra::input
                                                terra::scene terra::physics_mujoco)
    target_compile_definitions(test_render_reference PRIVATE TERRA_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets")
    terra_configure_target(test_render_reference)
    # M2-07：carts 由场景文件与物理快照驱动（MJCF 几何体 + glTF 节点绑定），不需要下载素材
    # M3-03：mjcf_materials（测试夹具，不需要下载）；robot_*（参考机器人 + 摄影棚 HDRI，缺失时跳过）
    foreach(_scene procedural studio outdoor carts mjcf_materials mjcf_collision robot_g1 robot_go2 robot_stretch3)
        add_test(NAME graphics.render_reference.${_scene}
            COMMAND ${TERRA_GRAPHICS_TEST_ENV} $<TARGET_FILE:test_render_reference>
                    "${PROJECT_SOURCE_DIR}/tests/render/reference" "${CMAKE_CURRENT_BINARY_DIR}/render_reference"
                    --scene ${_scene} --expect-renderer llvmpipe)
        set_tests_properties(graphics.render_reference.${_scene} PROPERTIES LABELS graphics SKIP_RETURN_CODE 77)
    endforeach()

    # 相机传感器的渲染（M4-03）：夹具中的深度、投影、远平面、彩色、时间与位姿、确定性；Stretch 3 头部相机（有参考机器人时）。
    # 输出目录中写出彩色 | 深度的 PNG。
    add_executable(test_camera_capture sensors/test_camera_capture.cpp)
    target_link_libraries(test_camera_capture PRIVATE terra::viewer_scene terra::render terra::platform_glfw terra::input
                                                      terra::scene terra::physics_mujoco)
    target_compile_definitions(test_camera_capture PRIVATE TERRA_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets")
    terra_configure_target(test_camera_capture)
    add_test(NAME graphics.camera_capture
        COMMAND ${TERRA_GRAPHICS_TEST_ENV} $<TARGET_FILE:test_camera_capture> "${CMAKE_CURRENT_BINARY_DIR}/camera_capture"
                --expect-renderer llvmpipe)
    set_tests_properties(graphics.camera_capture PROPERTIES LABELS graphics)

    # M4-04：查看器的传感器自检（IMU、雷达、相机的采样次数与读数，面板停靠）；terra_capture 的确定性与采样次数；
    # 没有相机（--no-cameras）时不创建图形上下文，在没有显示服务器的环境中也能运行
    # M5-11：查看器的 3D 雷达（采样、传感器面板、视口叠加点云）
    add_test(NAME graphics.viewer_lidar3d_selftest
        COMMAND ${TERRA_GRAPHICS_TEST_ENV} $<TARGET_FILE:terra_viewer> --frames 80 --self-test --expect-renderer llvmpipe
                --scene "${PROJECT_SOURCE_DIR}/assets/tests/sensors/lidar3d_room.terra-scene.json" --lidar3d-self-test)
    set_tests_properties(graphics.viewer_lidar3d_selftest PROPERTIES LABELS graphics)
    # M5-06：热像仪自检（夹具 thermal_bench：三台热像仪、图像中心 80 °C、背向墙全是天空温度、预览纹理）
    add_test(NAME graphics.viewer_thermal_selftest
        COMMAND ${TERRA_GRAPHICS_TEST_ENV} $<TARGET_FILE:terra_viewer> --frames 120 --self-test --expect-renderer llvmpipe
                --scene "${PROJECT_SOURCE_DIR}/assets/tests/sensors/thermal_bench.terra-scene.json" --thermal-self-test)
    set_tests_properties(graphics.viewer_thermal_selftest PROPERTIES LABELS graphics)
    # M5-02a：场景元素（夹具 elements_bench：木箱下落）；中途单击内置的“简易配电柜”卡片，打开只有地面与该元素的预览场景
    add_test(NAME graphics.viewer_elements_selftest
        COMMAND ${TERRA_GRAPHICS_TEST_ENV} $<TARGET_FILE:terra_viewer> --frames 120 --self-test --expect-renderer llvmpipe
                --scene "${PROJECT_SOURCE_DIR}/assets/tests/elements/elements_bench.terra-scene.json" --elements-self-test)
    set_tests_properties(graphics.viewer_elements_selftest PROPERTIES LABELS graphics)
    # M5-02b：环境导入（与菜单同一路径）：先打开元素夹具，启动时导入 env_fixture.gltf 到资源库目录的 environments/ 并打开；
    # 全部绑定可用（机柜的绑定排除部件节点，部件另有绑定）
    add_test(NAME graphics.viewer_environment_selftest
        COMMAND ${TERRA_GRAPHICS_TEST_ENV} $<TARGET_FILE:terra_viewer> --frames 100 --self-test --expect-renderer llvmpipe
                --scene "${PROJECT_SOURCE_DIR}/assets/tests/elements/elements_bench.terra-scene.json"
                --environment-self-test "${PROJECT_SOURCE_DIR}/assets/tests/environment/env_fixture.gltf"
                --library "${CMAKE_CURRENT_BINARY_DIR}/viewer_environment/library.sqlite3")
    set_tests_properties(graphics.viewer_environment_selftest PROPERTIES LABELS graphics)
    add_test(NAME graphics.viewer_sensor_selftest
        COMMAND ${TERRA_GRAPHICS_TEST_ENV} $<TARGET_FILE:terra_viewer> --frames 80 --self-test --expect-renderer llvmpipe
                --scene "${PROJECT_SOURCE_DIR}/assets/tests/sensors/sensor_suite.terra-scene.json" --sensor-self-test)
    set_tests_properties(graphics.viewer_sensor_selftest PROPERTIES LABELS graphics)
    # M4-08a：导入向导——第 12 帧分析 demo.urdf，第 14 帧导入并打开（需要 assimp）；资源库与导入目录在构建目录中
    add_test(NAME graphics.viewer_import_selftest
        COMMAND ${TERRA_GRAPHICS_TEST_ENV} $<TARGET_FILE:terra_viewer> --frames 40 --self-test --expect-renderer llvmpipe
                --import-self-test "${PROJECT_SOURCE_DIR}/assets/tests/import/demo_robot/urdf/demo.urdf"
                --library "${CMAKE_CURRENT_BINARY_DIR}/viewer_import/library.sqlite3")
    set_tests_properties(graphics.viewer_import_selftest PROPERTIES LABELS graphics)
    # M4-08c：资源库面板与欢迎页——单击卡片打开场景、当前场景高亮、收藏写入资源库、快速打开、动作卡片播放、隐藏；
    # 另扫描测试夹具（tests/scene、tests/motion）；资源库在构建目录中（重复运行时沿用，检查与之无关）
    add_test(NAME graphics.viewer_library_selftest
        COMMAND ${TERRA_GRAPHICS_TEST_ENV} $<TARGET_FILE:terra_viewer> --frames 40 --self-test --expect-renderer llvmpipe
                --library-self-test --library "${CMAKE_CURRENT_BINARY_DIR}/viewer_library/library.sqlite3")
    set_tests_properties(graphics.viewer_library_selftest PROPERTIES LABELS graphics)
    # M4-08d：显示配置——传感器面板弹出的仿真曲线、保存到资源库、重开场景后自动恢复、导出 .terviz 与 .rviz
    add_test(NAME graphics.viewer_viz_selftest
        COMMAND ${TERRA_GRAPHICS_TEST_ENV} $<TARGET_FILE:terra_viewer> --frames 300 --self-test --expect-renderer llvmpipe
                --scene "${PROJECT_SOURCE_DIR}/assets/tests/sensors/sensor_suite.terra-scene.json" --viz-self-test
                --library "${CMAKE_CURRENT_BINARY_DIR}/viewer_viz/library.sqlite3")
    set_tests_properties(graphics.viewer_viz_selftest PROPERTIES LABELS graphics)
    # M4-10a：策略——命令行 --policy 打开后运行（线性夹具，不需要 ONNX Runtime）、速度指令面板、面板中的指令传给策略、停止
    add_test(NAME graphics.viewer_policy_selftest
        COMMAND ${TERRA_GRAPHICS_TEST_ENV} $<TARGET_FILE:terra_viewer> --frames 80 --self-test --expect-renderer llvmpipe
                --scene "${PROJECT_SOURCE_DIR}/assets/tests/policy/policy_bench.terra-scene.json" --policy-self-test
                --policy "${PROJECT_SOURCE_DIR}/assets/tests/policy/bench.terra-policy.json"
                --library "${CMAKE_CURRENT_BINARY_DIR}/viewer_policy/library.sqlite3")
    set_tests_properties(graphics.viewer_policy_selftest PROPERTIES LABELS graphics)
    # M4-10b：导入向导——按关节名推荐场景、多出关节、导入并运行、登记到资源库、打开场景、需要组件时的弹窗（自检的资源库没有装组件）
    add_test(NAME graphics.viewer_policy_import_selftest
        COMMAND ${TERRA_GRAPHICS_TEST_ENV} $<TARGET_FILE:terra_viewer> --frames 120 --self-test --expect-renderer llvmpipe
                --policy-import-self-test "${PROJECT_SOURCE_DIR}/assets/tests/policy/bench_supermimic.onnx"
                --library "${CMAKE_CURRENT_BINARY_DIR}/viewer_policy_import/library.sqlite3")
    set_tests_properties(graphics.viewer_policy_import_selftest PROPERTIES LABELS graphics)
    # M4-07：第 6 帧开始记录（保存相机图像），中间停止并回放到一帧有图像的时刻；资源库在构建目录中
    add_test(NAME graphics.viewer_sensor_record_selftest
        COMMAND ${TERRA_GRAPHICS_TEST_ENV} $<TARGET_FILE:terra_viewer> --frames 80 --self-test --expect-renderer llvmpipe
                --scene "${PROJECT_SOURCE_DIR}/assets/tests/sensors/sensor_suite.terra-scene.json" --sensor-record-self-test
                --library "${CMAKE_CURRENT_BINARY_DIR}/viewer_sensor_record/library.sqlite3")
    set_tests_properties(graphics.viewer_sensor_record_selftest PROPERTIES LABELS graphics)
    add_test(NAME graphics.capture_determinism
        COMMAND ${TERRA_GRAPHICS_TEST_ENV} "${CMAKE_COMMAND}" -DCAPTURE=$<TARGET_FILE:terra_capture>
                "-DSCENE=${PROJECT_SOURCE_DIR}/assets/tests/sensors/sensor_suite.terra-scene.json"
                "-DOUT=${CMAKE_CURRENT_BINARY_DIR}/capture_determinism" -DEXPECT_RENDERER=llvmpipe
                -P "${PROJECT_SOURCE_DIR}/cmake/TerraCheckCapture.cmake")
    set_tests_properties(graphics.capture_determinism PROPERTIES LABELS graphics)
    # M4-09：terra_capture 的噪声（两次逐字节相同、--no-noise）与分割图（uint16 .npy 与对照表）
    add_test(NAME graphics.capture_noise
        COMMAND ${TERRA_GRAPHICS_TEST_ENV} "${TERRA_PYTHON3_EXE}" "${CMAKE_CURRENT_SOURCE_DIR}/sensors/check_noise.py" capture
                --capture $<TARGET_FILE:terra_capture> --scene "${PROJECT_SOURCE_DIR}/assets/tests/sensors/noise_suite.terra-scene.json"
                --work "${CMAKE_CURRENT_BINARY_DIR}/capture_noise")
    set_tests_properties(graphics.capture_noise PROPERTIES LABELS graphics)
    # M5-06：terra_capture 的热像输出（温度与分割、深度逐像素对齐，热点，曲线，噪声统计，两次逐字节相同，PNG）
    add_test(NAME graphics.capture_thermal
        COMMAND ${TERRA_GRAPHICS_TEST_ENV} "${TERRA_PYTHON3_EXE}" "${CMAKE_CURRENT_SOURCE_DIR}/sensors/check_thermal.py"
                --capture $<TARGET_FILE:terra_capture> --scene "${PROJECT_SOURCE_DIR}/assets/tests/sensors/thermal_bench.terra-scene.json"
                --work "${CMAKE_CURRENT_BINARY_DIR}/capture_thermal")
    set_tests_properties(graphics.capture_thermal PROPERTIES LABELS graphics TIMEOUT 240)
    add_test(NAME capture.no_cameras
        COMMAND "${CMAKE_COMMAND}" -E env --unset=DISPLAY --unset=WAYLAND_DISPLAY
                "${CMAKE_COMMAND}" -DEXPECTED=0 -DCOMMAND=$<TARGET_FILE:terra_capture>
                "-DSTDOUT_REGEX=不创建图形上下文（--no-cameras）.*采集完成：20 步，IMU 84 行，雷达 1 次扫描，相机 0 帧"
                -P "${_terra_check_exit}" --scene "${PROJECT_SOURCE_DIR}/assets/tests/sensors/sensor_suite.terra-scene.json"
                --steps 20 --no-cameras --force --output "${CMAKE_CURRENT_BINARY_DIR}/capture_no_cameras")

    # 渲染基准冒烟（M1-10）：小分辨率、少量帧，确认能跑完并输出汇总（数值不判定）。
    add_test(NAME graphics.render_bench_smoke
        COMMAND ${TERRA_GRAPHICS_TEST_ENV} "${CMAKE_COMMAND}" -DEXPECTED=0 -DCOMMAND=$<TARGET_FILE:terra_render_bench>
                "-DSTDOUT_REGEX=summary [{].type.:.summary.,.*.gpu_ms.:[{].mean.:"
                -P "${_terra_check_exit}" --width 160 --height 90 --frames 10 --warmup 2 --scene procedural
                --output "${CMAKE_CURRENT_BINARY_DIR}/render_bench_smoke.jsonl" --expect-renderer llvmpipe)
    set_tests_properties(graphics.render_bench_smoke PROPERTIES LABELS graphics)

    # 相机与相机控制器（M1-07）。自动化部分只用虚拟设备与数学检查，不需要显示；--show 模式供桌面交互展示。
    add_executable(test_view view/test_view.cpp)
    target_link_libraries(test_view PRIVATE terra::view terra::render terra::platform_glfw)
    terra_configure_target(test_view)
    add_test(NAME view.camera COMMAND test_view)

    # 界面（M1-13）：输入桥、键位映射、界面与相机抢输入；文本目录与字体字形覆盖。不需要显示。
    add_executable(test_ui_input ui/test_ui_input.cpp)
    target_link_libraries(test_ui_input PRIVATE terra::ui terra::view)
    terra_configure_target(test_ui_input)
    add_test(NAME ui.input_bridge COMMAND test_ui_input)

    add_executable(test_ui_text ui/test_ui_text.cpp)
    target_link_libraries(test_ui_text PRIVATE terra::ui)
    target_compile_definitions(test_ui_text PRIVATE TERRA_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets")
    terra_configure_target(test_ui_text)
    add_test(NAME ui.text COMMAND test_ui_text)

    # 查看器自检（M1-13）：在 Xvfb + llvmpipe 中运行 40 帧完整外壳，中途切换为英文，检查字体、GL 错误与像素。
    # M1-14：自检从默认设置开始（不读取文件），中途打开设置窗口；结束时检查设置换算与生效，并把设置保存到构建目录再读回比较
    # （不触碰用户的 ~/.config）。
    # 查看器场景模式自检（M2-08）：打开场景后暂停、播放、选中与高亮、暂停与单步、与纯物理逐位一致、重置
    add_test(NAME graphics.viewer_scene_selftest
        COMMAND ${TERRA_GRAPHICS_TEST_ENV} $<TARGET_FILE:terra_viewer> --frames 60 --self-test --expect-renderer llvmpipe
                --scene "${PROJECT_SOURCE_DIR}/assets/tests/scene/two_carts.terra-scene.json")
    set_tests_properties(graphics.viewer_scene_selftest PROPERTIES LABELS graphics)
    # M3-02：场景带初始关键帧时，打开即处于关键帧姿态，重置回到该关键帧（逐位相同）
    add_test(NAME graphics.viewer_keyframe_selftest
        COMMAND ${TERRA_GRAPHICS_TEST_ENV} $<TARGET_FILE:terra_viewer> --frames 60 --self-test --expect-renderer llvmpipe
                --scene "${PROJECT_SOURCE_DIR}/assets/tests/scene/keyframe_arm.terra-scene.json")
    set_tests_properties(graphics.viewer_keyframe_selftest PROPERTIES LABELS graphics)
    # M3-04c：先打开两辆小车，第 3 帧打开动作文件——切换到动作的推荐场景（keyframe_arm）并播放；
    # 与“动作 → setControl → step”的纯物理逐位相同；重置回到场景的初始关键帧
    add_test(NAME graphics.viewer_motion_selftest
        COMMAND ${TERRA_GRAPHICS_TEST_ENV} $<TARGET_FILE:terra_viewer> --frames 60 --self-test --expect-renderer llvmpipe
                --scene "${PROJECT_SOURCE_DIR}/assets/tests/scene/two_carts.terra-scene.json"
                --motion "${PROJECT_SOURCE_DIR}/assets/tests/motion/arm_swing.terra-motion.json")
    set_tests_properties(graphics.viewer_motion_selftest PROPERTIES LABELS graphics)
    # M3-07c：记录与回放——第 6 帧开始记录（动作从头播放），中间停止（加入构建目录中的资源库）并回放；
    # 检查记录可复现（逐位相同）、已加入资源库、回放画面与记录逐位相同
    add_test(NAME graphics.viewer_record_selftest
        COMMAND ${TERRA_GRAPHICS_TEST_ENV} $<TARGET_FILE:terra_viewer> --frames 80 --self-test --expect-renderer llvmpipe
                --scene "${PROJECT_SOURCE_DIR}/assets/tests/scene/keyframe_arm.terra-scene.json"
                --motion "${PROJECT_SOURCE_DIR}/assets/tests/motion/arm_swing.terra-motion.json"
                --record-self-test --library "${CMAKE_CURRENT_BINARY_DIR}/viewer_record/library.sqlite3")
    set_tests_properties(graphics.viewer_record_selftest PROPERTIES LABELS graphics)

    add_test(NAME graphics.viewer_selftest
        COMMAND ${TERRA_GRAPHICS_TEST_ENV} $<TARGET_FILE:terra_viewer> --frames 40 --self-test --expect-renderer llvmpipe
                --settings "${CMAKE_CURRENT_BINARY_DIR}/viewer_selftest/settings.json")
    set_tests_properties(graphics.viewer_selftest PROPERTIES LABELS graphics)
endif()

# ---- ROS2 桥接（M4-06a）----
if(NOT TERRA_ROS2_ENABLED)
    terra_add_cli_test(ros2_unavailable EXPECT 3 NO_DISPLAY STDERR "构建时没有 ROS2 桥接.*TERRA_BUILD_ROS2=ON"
        ARGS --scene "${PROJECT_SOURCE_DIR}/assets/tests/sensors/sensor_suite.terra-scene.json" --steps 1
             --ros2 "${PROJECT_SOURCE_DIR}/assets/ros2/profiles/generic.terra-ros2.json")
else()
    add_executable(test_ros2_profile ros2/test_ros2_profile.cpp)
    target_link_libraries(test_ros2_profile PRIVATE terra::ros2)
    target_compile_definitions(test_ros2_profile PRIVATE TERRA_ASSETS_DIR="${PROJECT_SOURCE_DIR}/assets")
    terra_configure_target(test_ros2_profile)
    add_test(NAME ros2.profile COMMAND test_ros2_profile)
    set_tests_properties(ros2.profile PROPERTIES LABELS ros2)

    terra_add_cli_test(ros2_bad_profile EXPECT 2 NO_DISPLAY STDERR "通信配置错误 /tfHz"
        ARGS --scene "${PROJECT_SOURCE_DIR}/assets/tests/sensors/sensor_suite.terra-scene.json" --steps 1
             --ros2 "${PROJECT_SOURCE_DIR}/assets/tests/ros2/bad_rate.terra-ros2.json")

    # 端到端：Python（rclpy）订阅、调用服务并核对。被测程序按墙钟运行十几秒，串行执行以免相互拖慢
    find_program(TERRA_PYTHON3 python3 REQUIRED)
    set(_terra_check_bridge "${CMAKE_CURRENT_SOURCE_DIR}/ros2/check_bridge.py")
    add_test(NAME ros2.bridge_headless
        COMMAND "${TERRA_PYTHON3}" "${_terra_check_bridge}" headless --headless $<TARGET_FILE:terra_headless>
                --assets "${PROJECT_SOURCE_DIR}/assets" --work "${CMAKE_CURRENT_BINARY_DIR}/ros2_bridge_headless")
    add_test(NAME ros2.bridge_commands
        COMMAND "${TERRA_PYTHON3}" "${_terra_check_bridge}" commands --headless $<TARGET_FILE:terra_headless>
                --assets "${PROJECT_SOURCE_DIR}/assets" --work "${CMAKE_CURRENT_BINARY_DIR}/ros2_bridge_commands")
    set_tests_properties(ros2.bridge_headless ros2.bridge_commands PROPERTIES LABELS ros2 RUN_SERIAL TRUE TIMEOUT 180)
    if(TERRA_BUILD_RENDER)
        add_test(NAME graphics.ros2_bridge_viewer
            COMMAND ${TERRA_GRAPHICS_TEST_ENV} "${TERRA_PYTHON3}" "${_terra_check_bridge}" viewer --viewer $<TARGET_FILE:terra_viewer>
                    --assets "${PROJECT_SOURCE_DIR}/assets" --work "${CMAKE_CURRENT_BINARY_DIR}/ros2_bridge_viewer")
        set_tests_properties(graphics.ros2_bridge_viewer PROPERTIES LABELS "graphics;ros2" RUN_SERIAL TRUE TIMEOUT 240)
    endif()

    # TurtleBot 4（M4-08b）：话题名映射、坐标系、差速底盘与里程计；需要先运行 scripts/assets/import_turtlebot4.sh（没有时跳过）
    add_test(NAME ros2.turtlebot4
        COMMAND "${TERRA_PYTHON3}" "${CMAKE_CURRENT_SOURCE_DIR}/ros2/check_turtlebot4.py" --headless $<TARGET_FILE:terra_headless>
                --assets "${PROJECT_SOURCE_DIR}/assets" --work "${CMAKE_CURRENT_BINARY_DIR}/ros2_turtlebot4")
    set_tests_properties(ros2.turtlebot4 PROPERTIES LABELS ros2 RUN_SERIAL TRUE TIMEOUT 180 SKIP_RETURN_CODE 77)
    # M4-10b：策略的速度指令来自 /cmd_vel（线性夹具，不需要 ONNX Runtime）；停发 0.5 s 后归零
    add_test(NAME ros2.policy_cmd_vel
        COMMAND "${TERRA_PYTHON3}" "${CMAKE_CURRENT_SOURCE_DIR}/ros2/check_policy_cmd_vel.py" --headless $<TARGET_FILE:terra_headless>
                --assets "${PROJECT_SOURCE_DIR}/assets" --work "${CMAKE_CURRENT_BINARY_DIR}/ros2_policy_cmd_vel")
    set_tests_properties(ros2.policy_cmd_vel PROPERTIES LABELS ros2 RUN_SERIAL TRUE TIMEOUT 120)
    # M5-11：3D 激光雷达的 PointCloud2（与 livox_ros_driver2 相同的字段）与内置 IMU
    add_test(NAME ros2.lidar3d
        COMMAND "${TERRA_PYTHON3}" "${CMAKE_CURRENT_SOURCE_DIR}/ros2/check_lidar3d_ros2.py" --headless $<TARGET_FILE:terra_headless>
                --assets "${PROJECT_SOURCE_DIR}/assets" --work "${CMAKE_CURRENT_BINARY_DIR}/ros2_lidar3d")
    set_tests_properties(ros2.lidar3d PROPERTIES LABELS ros2 RUN_SERIAL TRUE TIMEOUT 120)

    # ROS2 监视器（M4-08d）：Python 发布内容已知的消息（含宇树 LowState），逐项核对展开、曲线字段、图像、TF 与统计
    add_executable(test_ros2_monitor ros2/test_ros2_monitor.cpp)
    target_link_libraries(test_ros2_monitor PRIVATE terra::ros2)
    target_include_directories(test_ros2_monitor SYSTEM PRIVATE "${PROJECT_SOURCE_DIR}/third_party/stb")
    terra_configure_target(test_ros2_monitor)
    add_test(NAME ros2.monitor COMMAND test_ros2_monitor "${CMAKE_CURRENT_SOURCE_DIR}/ros2/publish_known.py")
    set_tests_properties(ros2.monitor PROPERTIES LABELS ros2 RUN_SERIAL TRUE TIMEOUT 120)
    # M4-08d：查看器开启桥接后，ROS2 面板看到桥接发布的话题：消息、图像、曲线、TF 树；保存、恢复、导出（.rviz 含 Image 与 TF）
    if(TERRA_BUILD_RENDER)
        add_test(NAME graphics.viewer_ros2_panel_selftest
            COMMAND ${TERRA_GRAPHICS_TEST_ENV} $<TARGET_FILE:terra_viewer> --frames 300 --self-test --expect-renderer llvmpipe
                    --scene "${PROJECT_SOURCE_DIR}/assets/tests/sensors/sensor_suite.terra-scene.json" --viz-self-test
                    --ros2 "${PROJECT_SOURCE_DIR}/assets/ros2/profiles/generic.terra-ros2.json"
                    --library "${CMAKE_CURRENT_BINARY_DIR}/viewer_ros2_panel/library.sqlite3")
        set_tests_properties(graphics.viewer_ros2_panel_selftest PROPERTIES LABELS "graphics;ros2" RUN_SERIAL TRUE TIMEOUT 240
                             ENVIRONMENT "ROS_DOMAIN_ID=163;ROS_LOCALHOST_ONLY=1")
        # M5-06：查看器开启桥接后的热像话题：mono16（0.01 K）、rgb8 伪彩色、camera_info（与热像同一相机）
        add_test(NAME ros2.thermal
            COMMAND ${TERRA_GRAPHICS_TEST_ENV} "${TERRA_PYTHON3}" "${CMAKE_CURRENT_SOURCE_DIR}/ros2/check_thermal_ros2.py"
                    --viewer $<TARGET_FILE:terra_viewer> --assets "${PROJECT_SOURCE_DIR}/assets" --work "${CMAKE_CURRENT_BINARY_DIR}/ros2_thermal")
        set_tests_properties(ros2.thermal PROPERTIES LABELS "graphics;ros2" RUN_SERIAL TRUE TIMEOUT 180)
    endif()

    # ---- 宇树配置（M4-06b）：需要宇树消息包 ----
    if(TERRA_ROS2_UNITREE)
        add_executable(test_unitree_crc ros2/test_unitree_crc.cpp)
        target_include_directories(test_unitree_crc PRIVATE "${PROJECT_SOURCE_DIR}/engine/ros2/src")
        target_link_libraries(test_unitree_crc PRIVATE terra::ros2 ${unitree_go_TARGETS} ${unitree_hg_TARGETS})
        terra_configure_target(test_unitree_crc)
        add_test(NAME ros2.unitree_crc COMMAND test_unitree_crc)
        set_tests_properties(ros2.unitree_crc PROPERTIES LABELS ros2)

        set(TERRA_UNITREE_ROS2_DIR "$ENV{HOME}/unitree_ros2" CACHE PATH "unitree_ros2 的检出目录（官方示例在其 example/ 下编译）")
        set(_terra_check_unitree "${CMAKE_CURRENT_SOURCE_DIR}/ros2/check_unitree.py")
        set(_terra_unitree_args --headless $<TARGET_FILE:terra_headless> --assets "${PROJECT_SOURCE_DIR}/assets")
        add_test(NAME ros2.unitree_go2
            COMMAND "${TERRA_PYTHON3}" "${_terra_check_unitree}" go2 ${_terra_unitree_args} --work "${CMAKE_CURRENT_BINARY_DIR}/ros2_unitree_go2")
        add_test(NAME ros2.unitree_g1
            COMMAND "${TERRA_PYTHON3}" "${_terra_check_unitree}" hg --robot g1 ${_terra_unitree_args}
                    --work "${CMAKE_CURRENT_BINARY_DIR}/ros2_unitree_g1")
        add_test(NAME ros2.unitree_as2
            COMMAND "${TERRA_PYTHON3}" "${_terra_check_unitree}" hg --robot as2 ${_terra_unitree_args}
                    --work "${CMAKE_CURRENT_BINARY_DIR}/ros2_unitree_as2")
        set_tests_properties(ros2.unitree_go2 ros2.unitree_g1 ros2.unitree_as2 PROPERTIES LABELS ros2 RUN_SERIAL TRUE TIMEOUT 180)
        foreach(_robot go2 go2w g1)
            add_test(NAME ros2.unitree_official_${_robot}
                COMMAND "${TERRA_PYTHON3}" "${_terra_check_unitree}" official --robot ${_robot} ${_terra_unitree_args}
                        --unitree-examples "${TERRA_UNITREE_ROS2_DIR}/example" --work "${CMAKE_CURRENT_BINARY_DIR}/ros2_unitree_official_${_robot}")
            set_tests_properties(ros2.unitree_official_${_robot} PROPERTIES LABELS ros2 RUN_SERIAL TRUE TIMEOUT 180 SKIP_RETURN_CODE 77)
        endforeach()
    endif()
endif()
