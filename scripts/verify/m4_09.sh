#!/usr/bin/env bash
# M4-09 基线机器验证：传感器噪声与分割图（M4-09 设计稿 2026-11-25 评审通过；用户要求：噪声可以关闭、雷达参数按数据手册）——
# 噪声：IMU（白噪声密度、零偏随机游走、初始零偏）、2D 激光雷达（距离噪声、丢失、量化、数据手册预设）、深度（a + b·z²、丢失）、
# 彩色（每通道 σ）；计数器式随机数，带种子、可复现；各出口（Headless、查看器、terra_capture、ROS2、记录与复现）读数逐位相同；
# 关闭：传感器、场景、运行时（--no-noise、查看器面板）三个层次。分割图：实体 ID（uint16 .npy、ROS2 16UC1）与对照表。
#
# 请在**桌面会话终端**运行（第 8 步打开查看器）。依赖与 M4-08d 相同。
# 依次执行（日志在 build/verify-m4-09/logs/，屏幕只打印结论）：
#   1. 依赖与素材校验；ROS2 与宇树消息包检查
#   2. 不含 ROS2 的构建：渲染关闭 Debug（预期 102/102）；渲染开启 Debug 与 Release + -Werror（预期 140/140）
#   3. ASan/UBSan（不含 ROS2）
#   4. 含 ROS2 与宇树消息包的构建（Release + -Werror）：全量 CTest（预期 154/154）
#   5. ASan/UBSan（含 ROS2）：ros2 标签的测试（预期 14/14）
#   6. 变异测试（不含 ROS2 的渲染构建）：逐个注入 8 个已知错误
#   7. 输出：噪声统计、出口检查与分割图的 INFO 行
#   8. 可交互展示（最长 300 秒）：查看器打开噪声夹具，传感器面板中开关噪声、看分割图
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m4_09.sh [--jobs N] [--skip-asan] [--skip-mutation] [--skip-show] [--mutation-only]
set -uo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
cd "$root"

jobs=$(nproc)
skip_asan=0
skip_mutation=0
mutation_only=0
skip_show=0
while [ $# -gt 0 ]; do
    case "$1" in
        --jobs) jobs=$2; shift 2 ;;
        --skip-asan) skip_asan=1; shift ;;
        --skip-mutation) skip_mutation=1; shift ;;
        --mutation-only) mutation_only=1; skip_asan=1; skip_show=1; shift ;;
        --skip-show) skip_show=1; shift ;;
        -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/verify-m4-09"
logs="$out/logs"
rm -rf "$out"
mkdir -p "$logs"

summary=()
failed=0
record() {  # record <名称> <结果文字> <是否通过 0/1>
    summary+=("$(printf '%-28s %s' "$1" "$2")")
    [ "$3" = 1 ] || failed=1
    echo "${summary[-1]}"
}
note() { summary+=("$(printf '%-28s %s' "$1" "$2")"); echo "${summary[-1]}"; }
ok() { [ "$1" = 0 ] && echo 1 || echo 0; }
ctest_result() { grep -E "tests passed|tests failed" "$1" | tail -1; }

# 与 tests/CMakeLists.txt 中 TERRA_GRAPHICS_TEST_ENV 相同的固定环境（D-022）
graphics_env() {
    env -u DISPLAY -u WAYLAND_DISPLAY __GLX_VENDOR_LIBRARY_NAME=mesa LIBGL_ALWAYS_SOFTWARE=1 \
        xvfb-run -a -s "-screen 0 1920x1080x24" "$@"
}

echo "== 1. 依赖与 ROS2"
bash scripts/settings/check_json.sh >"$logs/json.txt" 2>&1
rc=$?
record "nlohmann/json 校验" "$(grep -c '^一致' "$logs/json.txt")/3 个文件一致，退出码 $rc" "$(ok $rc)"
bash scripts/render/check_stb.sh >"$logs/stb.txt" 2>&1
rc=$?
record "stb 校验" "$(grep -c '^一致' "$logs/stb.txt")/3 个文件一致，退出码 $rc" "$(ok $rc)"
bash scripts/render/check_cgltf.sh >"$logs/cgltf.txt" 2>&1
rc=$?
record "cgltf 校验" "$(grep -c '^一致' "$logs/cgltf.txt")/2 个文件一致，退出码 $rc" "$(ok $rc)"
bash scripts/render/fetch.sh >"$logs/fetch.txt" 2>&1
rc=$?
record "获取并校验渲染依赖" "退出码 $rc" "$(ok $rc)"
bash scripts/assets/fetch_hdri.sh >"$logs/hdri.txt" 2>&1
rc=$?
record "HDRI 贴图" "$(grep -cE '^(已存在且一致|就绪)' "$logs/hdri.txt")/2 张一致，退出码 $rc" "$(ok $rc)"
bash scripts/assets/fetch_models.sh >"$logs/models.txt" 2>&1
rc=$?
record "glTF 资产" "$(tail -1 "$logs/models.txt")，退出码 $rc" "$(ok $rc)"
bash scripts/assets/fetch_robots.sh >"$logs/robots.txt" 2>&1
rc=$?
record "参考机器人" "$(tail -1 "$logs/robots.txt")，退出码 $rc" "$(ok $rc)"
bash scripts/import/check_third_party.sh >"$logs/third_party.txt" 2>&1
rc=$?
record "tinyxml2、xacro、miniz 校验" "$(grep -c '^一致' "$logs/third_party.txt")/12 个文件一致，退出码 $rc" "$(ok $rc)"
if [ ! -f third_party/render/install/lib/cmake/glfw3/glfw3Config.cmake ]; then
    echo "错误：未找到 GLFW 安装，请先运行 scripts/render/build.sh（M1-02）" >&2
    exit 1
fi

run_preset() {  # run_preset <preset> <名称> [额外配置参数...]
    [ $mutation_only = 1 ] && return 0
    local preset=$1 label=$2; shift 2
    local log="$logs/$preset.txt"
    rm -rf "build/$preset"
    if cmake --preset "$preset" "$@" >"$log" 2>&1 && cmake --build --preset "$preset" -j "$jobs" >>"$log" 2>&1; then
        if grep -q "warning:" "$log"; then
            record "$label 构建" "有编译警告（见 $log）" 0
            grep -A3 "warning:" "$log" | head -20 | sed 's/^/    /'
        fi
        ctest --preset "$preset" >>"$log" 2>&1
        local rc=$?
        record "$label 测试" "$(ctest_result "$log")" "$(ok $rc)"
        [ $rc = 0 ] || grep -E "^\s+[0-9]+ - |FAILED" "$log" | head -20 | sed 's/^/    /'
    else
        record "$label 构建" "失败（见 $log）" 0
        grep -E -A3 "error:|warning:" "$log" | head -20 | sed 's/^/    /'
    fi
}


ros_setup=/opt/ros/humble/setup.bash
unitree=${UNITREE_ROS2:-$HOME/unitree_ros2}
unitree_setup="$unitree/cyclonedds_ws/install/setup.bash"
bash scripts/ros2/check_ros2.sh --setup "$unitree_setup" >"$logs/check_ros2.txt" 2>&1
rc=$?
record "ROS2 与宇树消息包检查" "退出码 $rc（见 $logs/check_ros2.txt）" "$(ok $rc)"
examples_found=0
for e in go2_stand_example g1_low_level_example; do
    p=$(find "$unitree/example" -type f -name "$e" -perm -u+x 2>/dev/null | head -1)
    if [ -n "$p" ]; then
        examples_found=$((examples_found + 1))
        echo "  官方示例 $e：$p"
    fi
done
if [ $examples_found = 2 ]; then
    record "宇树官方示例程序" "已编译（go2_stand_example、g1_low_level_example）" 1
else
    record "宇树官方示例程序" "没有找到（$unitree/example），请先编译：cd $unitree/example && source $ros_setup && source ../cyclonedds_ws/install/setup.bash && colcon build" 0
fi

echo "== 2. 不含 ROS2 的构建"
run_preset debug "渲染关闭 Debug"
run_preset render-debug "渲染 Debug"
run_preset render-release "渲染 Release -Werror" -DTERRA_WARNINGS_AS_ERRORS=ON
if [ $mutation_only = 0 ]; then
    bash scripts/assets/import_turtlebot4.sh build/render-release/apps/import/terra_import >"$logs/import_turtlebot4.txt" 2>&1
    rc=$?
    record "重新导入 TurtleBot 4" "退出码 $rc；$(grep -c '^terra_import: \[warning\]' "$logs/import_turtlebot4.txt") 个警告（见 $logs/import_turtlebot4.txt）" "$(ok $rc)"
    [ $rc = 0 ] || grep -E '\[error\]' "$logs/import_turtlebot4.txt" | head -10 | sed 's/^/    /'
fi

san="-fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer"
if [ $skip_asan = 0 ]; then
    echo "== 3. ASan/UBSan（不含 ROS2）"
    log="$logs/asan.txt"
    b="$out/asan"
    if cmake -S . -B "$b" -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug -DTERRA_BUILD_RENDER=ON \
             -DTERRA_WARNINGS_AS_ERRORS=ON "-DCMAKE_CXX_FLAGS=$san" "-DCMAKE_EXE_LINKER_FLAGS=$san" >"$log" 2>&1 &&
       cmake --build "$b" -j "$jobs" >>"$log" 2>&1; then
        ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=print_stacktrace=1 \
            ctest --test-dir "$b" -LE graphics --output-on-failure >"$logs/asan_nongraphics.txt" 2>&1
        rc1=$?
        ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=print_stacktrace=1 \
            ctest --test-dir "$b" -L graphics --output-on-failure >"$logs/asan_graphics.txt" 2>&1
        rc2=$?
        record "ASan 非图形（含泄漏检测）" "$(ctest_result "$logs/asan_nongraphics.txt")" "$(ok $rc1)"
        record "ASan 图形（不查泄漏）" "$(ctest_result "$logs/asan_graphics.txt")" "$(ok $rc2)"
    else
        record "ASan 构建" "失败（见 $log）" 0
    fi
fi

# ---- 以下需要 ROS2 与宇树消息包 ----
if [ ! -f "$ros_setup" ] || [ ! -f "$unitree_setup" ]; then
    record "ROS2" "未找到 $ros_setup 或 $unitree_setup，第 4～8 步无法执行" 0
    echo; printf '%s\n' "${summary[@]}"; exit 1
fi
echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-（未设置）} CYCLONEDDS_URI=${CYCLONEDDS_URI:-（未设置）} → 清除，使用 Humble 默认" >"$logs/ros_env.txt"
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
set +u
# shellcheck disable=SC1090
source "$ros_setup"
# shellcheck disable=SC1090
source "$unitree_setup"
set -u
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
echo "  ROS_DISTRO=${ROS_DISTRO:-?}" >>"$logs/ros_env.txt"
export ROS_LOCALHOST_ONLY=1
ros_flags=(-DTERRA_BUILD_ROS2=ON "-DTERRA_UNITREE_ROS2_DIR=$unitree")

ros_build="$out/ros2"
if [ $mutation_only = 0 ]; then
    echo "== 4. 含 ROS2 与宇树消息包的构建"
    log="$logs/ros2_build.txt"
    if cmake -S . -B "$ros_build" -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Release -DTERRA_BUILD_RENDER=ON \
             "${ros_flags[@]}" -DTERRA_WARNINGS_AS_ERRORS=ON >"$log" 2>&1 &&
       cmake --build "$ros_build" -j "$jobs" >>"$log" 2>&1; then
        grep -E "ROS2 桥接：" "$log" | head -2 | sed 's/^-- /  /'
        if ! grep -q "宇树消息包：ON" "$log"; then
            record "含 ROS2 的构建" "CMake 没有找到宇树消息包（见 $log）" 0
        fi
        if grep -q "warning:" "$log"; then
            record "含 ROS2 构建" "有编译警告（见 $log）" 0
            grep -A3 "warning:" "$log" | head -20 | sed 's/^/    /'
        fi
        ctest --test-dir "$ros_build" --output-on-failure >"$logs/ros2_ctest.txt" 2>&1
        rc=$?
        record "含 ROS2 Release -Werror 测试" "$(ctest_result "$logs/ros2_ctest.txt")" "$(ok $rc)"
        [ $rc = 0 ] || grep -E "^\s+[0-9]+ - |FAILED" "$logs/ros2_ctest.txt" | head -30 | sed 's/^/    /'
        skipped=$(grep -cE "\*\*\*Skipped|Not Run.*Skipped" "$logs/ros2_ctest.txt")
        [ "$skipped" = 0 ] || record "跳过的测试" "$skipped 个（见 $logs/ros2_ctest.txt；官方示例未编译时跳过）" 0
    else
        record "含 ROS2 构建" "失败（见 $log）" 0
        grep -E -A3 "error:|warning:|CMake Error" "$log" | head -30 | sed 's/^/    /'
    fi
fi

if [ $skip_asan = 0 ]; then
    echo "== 5. ASan/UBSan（含 ROS2）"
    log="$logs/ros2_asan.txt"
    b="$out/ros2_asan"
    if cmake -S . -B "$b" -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug -DTERRA_BUILD_RENDER=ON "${ros_flags[@]}" \
             -DTERRA_WARNINGS_AS_ERRORS=ON "-DCMAKE_CXX_FLAGS=$san" "-DCMAKE_EXE_LINKER_FLAGS=$san" >"$log" 2>&1 &&
       cmake --build "$b" -j "$jobs" >>"$log" 2>&1; then
        # new_delete_type_mismatch=0：rclcpp Humble 头文件中 retyped_deallocate<char> 的上游缺陷（M4-06a 报告第 5.2 节）
        ASAN_OPTIONS=detect_leaks=0:new_delete_type_mismatch=0 UBSAN_OPTIONS=print_stacktrace=1 \
            ctest --test-dir "$b" -L ros2 --output-on-failure >"$logs/ros2_asan_ctest.txt" 2>&1
        rc=$?
        record "ASan 含 ROS2（ros2 标签）" "$(ctest_result "$logs/ros2_asan_ctest.txt")" "$(ok $rc)"
        [ $rc = 0 ] || grep -E "ERROR: AddressSanitizer|SUMMARY: |runtime error|FAILED" "$logs/ros2_asan_ctest.txt" | head -20 | sed 's/^/    /'
    else
        record "ASan 含 ROS2 构建" "失败（见 $log）" 0
    fi
fi

if [ $skip_mutation = 0 ]; then
    echo "== 6. 变异测试"
    log="$logs/mutation.txt"
    b="$out/mutation"
    files=(engine/simulation/src/sensor_noise.cpp engine/render/src/depth_renderer.cpp apps/headless/main.cpp)
    for f in "${files[@]}"; do cp "$f" "$out/${f//\//_}.orig"; done
    restore() { for f in "${files[@]}"; do cp "$out/${f//\//_}.orig" "$f"; done; }
    trap 'restore; echo "已恢复被变异的源文件"; exit 130' INT TERM
    # 不开 -Werror：变异只改语义，不应因警告而无法判定
    cmake -S . -B "$b" -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug -DTERRA_BUILD_RENDER=ON >"$log" 2>&1
    detected=0
    total=0
    while IFS='|' read -r name file target testname; do
        total=$((total + 1))
        restore
        if ! python3 - "$file" "$name" >>"$log" 2>&1 <<'PY'
import sys
path, name = sys.argv[1], sys.argv[2]
mutations = {
    "随机数与步序号无关": (
        "    return splitMix(seed ^ splitMix(stream ^ splitMix(step ^ splitMix(index))));",
        "    (void)step;\n    return splitMix(seed ^ splitMix(stream ^ splitMix(splitMix(index))));"),
    "白噪声没有按频率离散化": (
        "        if (axis.noiseDensity > 0) v += axis.noiseDensity * sqrtRate * noiseGaussian(",
        "        if (axis.noiseDensity > 0) v += axis.noiseDensity * noiseGaussian("),
    "随机游走不累积": (
        "            if (walk[c] > 0) s.walk[c] += walk[c] * sqrtDt",
        "            if (walk[c] > 0) s.walk[c] = walk[c] * sqrtDt"),
    "雷达丢失没有置为 inf": (
        "        if (n.dropout > 0 && noiseUniform(seed_, stream, scan.step, kDropout + i) < n.dropout) {\n            r = inf;",
        "        if (n.dropout > 0 && noiseUniform(seed_, stream, scan.step, kDropout + i) < n.dropout) {\n            (void)0;"),
    "重置后随机游走不回到初值": (
        "    } else {\n        s.walk.fill(0.0);\n    }",
        "    } else {\n    }"),
    "--no-noise 不生效": (
        "terra::simulation::SensorNoise(*sceneInstance, !options.noNoise)",
        "terra::simulation::SensorNoise(*sceneInstance, true)"),
    "记录中的 IMU 不加噪声": (
        "    if (recorder) recorder->state(noise.withNoise(*world.snapshot()));  // M4-09：记录带噪声的读数",
        "    if (recorder) recorder->state(*world.snapshot());"),
    "分割图把背景写成 1": (
        "            ids[row * w + col] = v > 0.5f ? static_cast<std::uint32_t>(std::lround(v)) : 0u;",
        "            ids[row * w + col] = v > 0.5f ? static_cast<std::uint32_t>(std::lround(v)) : 1u;"),
}
old, new = mutations[name]
text = open(path, encoding="utf-8").read()
if text.count(old) != 1:
    sys.exit(f"变异 '{name}' 的锚点在 {path} 中出现 {text.count(old)} 次，源码可能已改动")
open(path, "w", encoding="utf-8").write(text.replace(old, new))
PY
        then
            summary+=("  变异 $name：注入失败（见 $log）")
            failed=1
            continue
        fi
        echo "---- 变异：$name" >>"$log"
        if ! cmake --build "$b" -j "$jobs" --target "$target" >>"$log" 2>&1; then
            summary+=("  变异 $name：编译失败，未判定（见 $log）")
            failed=1
            continue
        fi
        if ctest --test-dir "$b" -R "^${testname//./\\.}\$" --output-on-failure >>"$log" 2>&1; then
            summary+=("  变异 $name：未被发现")
        else
            detected=$((detected + 1))
            n=$(awk -v head="---- 变异：$name" '$0 == head {found = 1} found' "$log" | grep -cE 'FAILED: |FAILED |不匹配|退出码不符|没有匹配')
            summary+=("  变异 $name：已发现（$n 项检查失败）")
        fi
    done <<'LIST'
随机数与步序号无关|engine/simulation/src/sensor_noise.cpp|test_sensor_noise|simulation.sensor_noise
白噪声没有按频率离散化|engine/simulation/src/sensor_noise.cpp|test_sensor_noise|simulation.sensor_noise
随机游走不累积|engine/simulation/src/sensor_noise.cpp|test_sensor_noise|simulation.sensor_noise
雷达丢失没有置为 inf|engine/simulation/src/sensor_noise.cpp|test_sensor_noise|simulation.sensor_noise
重置后随机游走不回到初值|engine/simulation/src/sensor_noise.cpp|test_sensor_noise|simulation.sensor_noise
--no-noise 不生效|apps/headless/main.cpp|terra_headless|headless.sensors_noise
记录中的 IMU 不加噪声|apps/headless/main.cpp|terra_headless|headless.sensors_noise
分割图把背景写成 1|engine/render/src/depth_renderer.cpp|test_camera_capture|graphics.camera_capture
LIST
    restore
    trap - INT TERM
    restored=1
    for f in "${files[@]}"; do cmp -s "$f" "$out/${f//\//_}.orig" || restored=0; done
    if [ $restored = 1 ]; then
        record "变异测试" "$detected/$total 被发现；源文件已恢复" "$([ $detected = $total ] && echo 1 || echo 0)"
    else
        record "变异测试" "源文件恢复失败！请用 $out/*.orig 手动恢复" 0
    fi
fi
[ $mutation_only = 1 ] && { echo; printf '%s\n' "${summary[@]}"; exit $failed; }

echo "== 7. 输出"
if [ -d "$ros_build" ]; then
    ctest --test-dir "$ros_build" -R '^(simulation\.sensor_noise|headless\.sensors_noise|graphics\.(capture_noise|camera_capture))$' -V 2>/dev/null |
        grep -E "INFO|PASS|FAILED|SKIP|all passed" | grep -v "启动：\|已加载资产\|资产 .* 信息\|已写出" | sed 's/^[0-9]*: /  /' | cut -c1-260
fi

viewer="build/render-release/apps/viewer/terra_viewer"
echo "== 8. 可交互展示"
if [ $skip_show = 1 ]; then
    note "可交互展示" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "可交互展示" "未执行：不在桌面会话中"
elif [ -x "$viewer" ]; then
    mkdir -p "$out/show"
    echo "（即将打开 terra_viewer，最长 300 秒；关闭窗口即结束。资源库与设置在 $out/show/，不读写你的用户目录）"
    echo "  - 场景：噪声与分割图夹具（assets/tests/sensors/noise_suite）；按播放运行"
    echo "  - 传感器面板顶部的“噪声”开关：关掉后 IMU 曲线与雷达点变为真值，再打开恢复（只影响本次运行）"
    echo "  - IMU 选 bench/level_imu（静止）：开噪声时能看到白噪声与零偏的慢漂；bench/tilt_imu 没有配置噪声，不变"
    echo "  - 雷达 room/scan 有 5% 丢失与 1 cm 噪声；room/scan_a1 用 RPLIDAR A1M8 的数据手册预设"
    echo "  - 相机 cam/eye：彩色与深度带噪声；第三张为分割图（按实体着色，悬停显示实体名；红色方块是 cam/target）"
    "$viewer" --scene assets/tests/sensors/noise_suite.terra-scene.json --seconds 300 --settings "$out/show/settings.json" \
        --library "$out/show/library.sqlite3" >"$logs/show.txt" 2>&1
    rc=$?
    grep -E "^INFO 场景|^INFO 结束" "$logs/show.txt" | head -8 | sed 's/^/  /'
    record "可交互展示" "退出码 $rc" "$(ok $rc)"
else
    record "可交互展示" "未执行（构建失败）" 0
fi

echo
echo "================ 汇总 ================"
printf '%s\n' "${summary[@]}"
echo "日志目录：$logs"
if [ $failed = 0 ]; then
    echo "结论：全部通过"
else
    echo "结论：存在失败项"
fi
exit $failed
