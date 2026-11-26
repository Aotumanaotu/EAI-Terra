#!/usr/bin/env bash
# M4-06a 基线机器验证：ROS2 桥接的通用配置（M4-05 设计稿 2026-11-24 评审通过）——
# 标准消息：/clock、/joint_states、/tf、/tf_static、IMU、2D 激光雷达、相机（彩色、深度、内参）；
# 订阅 /terra/joint_commands；服务 /terra/reset、/terra/pause、/terra/step。查看器与 Headless 都可开启（--ros2，查看器另有菜单）。
#
# 请在**桌面会话终端**运行（第 8 步打开查看器与 RViz2）。不需要先 source ROS2：脚本在第 4 步起自己 source /opt/ros/humble/setup.bash。
# 依次执行（日志在 build/verify-m4-06a/logs/，屏幕只打印结论）：
#   1. 依赖与素材校验；ROS2 环境检查（scripts/ros2/check_ros2.sh）
#   2. 不含 ROS2 的构建（默认）：渲染关闭 Debug（预期 84/84，新增 headless.ros2_unavailable：提示安装并返回 3）；
#      渲染开启 Debug 与 Release + -Werror（预期 117/117）
#   3. ASan/UBSan（不含 ROS2，渲染开启）
#   4. 含 ROS2 的构建（-DTERRA_BUILD_ROS2=ON，渲染开启，Release + -Werror）：全量 CTest（预期 121/121：去掉 headless.ros2_unavailable，
#      新增 ros2.profile、headless.ros2_bad_profile、ros2.bridge_headless、ros2.bridge_commands、graphics.ros2_bridge_viewer）
#   5. ASan/UBSan（含 ROS2）：只运行 ros2 标签的测试（不查泄漏：rclcpp 的全局对象；关闭 new_delete_type_mismatch：rclcpp 上游缺陷）
#   6. 变异测试（含 ROS2 的构建）：逐个注入 10 个已知错误
#   7. 输出：端到端检查的 INFO 行（话题频率、IMU 与雷达数值、相机、命令、服务）
#   8. 可交互展示（最长 180 秒）：查看器打开 Stretch 3（ROS2 桥接开启），RViz2 显示 TF、雷达与头部相机，
#      示例程序 scripts/ros2/terra_generic_example.py 让头部左右转动
# 测试只在本机通信（ROS_LOCALHOST_ONLY=1），每个检查用单独的 ROS_DOMAIN_ID；脚本清除 RMW_IMPLEMENTATION 与 CYCLONEDDS_URI，
# 使用 Humble 默认的 DDS（宇树的 CycloneDDS 设置在 M4-06b 使用）。
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m4_06a.sh [--jobs N] [--skip-asan] [--skip-mutation] [--skip-show] [--mutation-only]
# --mutation-only：只做变异测试（阶段验收调用；跳过 ASan 与展示）
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
        -h|--help) sed -n '2,26p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/verify-m4-06a"
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
bash scripts/ros2/check_ros2.sh >"$logs/check_ros2.txt" 2>&1
rc=$?
record "ROS2 环境检查" "退出码 $rc（见 $logs/check_ros2.txt）" "$(ok $rc)"

echo "== 2. 不含 ROS2 的构建"
run_preset debug "渲染关闭 Debug"
run_preset render-debug "渲染 Debug"
run_preset render-release "渲染 Release -Werror" -DTERRA_WARNINGS_AS_ERRORS=ON

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

# ---- 以下需要 ROS2 环境 ----
if [ ! -f "$ros_setup" ]; then
    record "ROS2" "未找到 $ros_setup，第 4～8 步无法执行" 0
    echo; printf '%s\n' "${summary[@]}"; exit 1
fi
echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-（未设置）} CYCLONEDDS_URI=${CYCLONEDDS_URI:-（未设置）} → 清除，使用 Humble 默认" >"$logs/ros_env.txt"
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
set +u
# shellcheck disable=SC1090
source "$ros_setup"
set -u
echo "  ROS_DISTRO=${ROS_DISTRO:-?}" >>"$logs/ros_env.txt"
export ROS_LOCALHOST_ONLY=1

ros_build="$out/ros2"
if [ $mutation_only = 0 ]; then
    echo "== 4. 含 ROS2 的构建"
    log="$logs/ros2_build.txt"
    if cmake -S . -B "$ros_build" -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Release -DTERRA_BUILD_RENDER=ON \
             -DTERRA_BUILD_ROS2=ON -DTERRA_WARNINGS_AS_ERRORS=ON >"$log" 2>&1 &&
       cmake --build "$ros_build" -j "$jobs" >>"$log" 2>&1; then
        grep -E "ROS2 桥接：" "$log" | head -2 | sed 's/^-- /  /'
        if ! grep -q "ROS2 桥接：开启" "$log"; then
            record "含 ROS2 的构建" "CMake 没有开启 ROS2 桥接（见 $log）" 0
        fi
        if grep -q "warning:" "$log"; then
            record "含 ROS2 构建" "有编译警告（见 $log）" 0
            grep -A3 "warning:" "$log" | head -20 | sed 's/^/    /'
        fi
        ctest --test-dir "$ros_build" --output-on-failure >"$logs/ros2_ctest.txt" 2>&1
        rc=$?
        record "含 ROS2 Release -Werror 测试" "$(ctest_result "$logs/ros2_ctest.txt")" "$(ok $rc)"
        [ $rc = 0 ] || grep -E "^\s+[0-9]+ - |FAILED" "$logs/ros2_ctest.txt" | head -30 | sed 's/^/    /'
    else
        record "含 ROS2 构建" "失败（见 $log）" 0
        grep -E -A3 "error:|warning:|CMake Error" "$log" | head -30 | sed 's/^/    /'
    fi
fi

if [ $skip_asan = 0 ]; then
    echo "== 5. ASan/UBSan（含 ROS2）"
    log="$logs/ros2_asan.txt"
    b="$out/ros2_asan"
    if cmake -S . -B "$b" -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug -DTERRA_BUILD_RENDER=ON -DTERRA_BUILD_ROS2=ON \
             -DTERRA_WARNINGS_AS_ERRORS=ON "-DCMAKE_CXX_FLAGS=$san" "-DCMAKE_EXE_LINKER_FLAGS=$san" >"$log" 2>&1 &&
       cmake --build "$b" -j "$jobs" >>"$log" 2>&1; then
        # new_delete_type_mismatch=0：rclcpp Humble 头文件中的 retyped_deallocate<char> 按 1 字节做 sized delete，
        # 释放 rcutils 按 72 字节分配的内存（上游缺陷，在本程序中实例化，与 Terra 代码无关；glibc 下无害）。只关这一项检查
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
    files=(apps/headless/main.cpp apps/viewer/main.cpp engine/ros2/src/bridge.cpp engine/ros2/src/profile.cpp)
    for f in "${files[@]}"; do cp "$f" "$out/${f//\//_}.orig"; done
    restore() { for f in "${files[@]}"; do cp "$out/${f//\//_}.orig" "$f"; done; }
    trap 'restore; echo "已恢复被变异的源文件"; exit 130' INT TERM
    # 不开 -Werror：变异只改语义，不应因警告而无法判定
    cmake -S . -B "$b" -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug -DTERRA_BUILD_RENDER=ON -DTERRA_BUILD_ROS2=ON >"$log" 2>&1
    detected=0
    total=0
    while IFS='|' read -r name file target testname; do
        total=$((total + 1))
        restore
        if ! python3 - "$file" "$name" >>"$log" 2>&1 <<'PY'
import sys
path, name = sys.argv[1], sys.argv[2]
mutations = {
    "Headless 不合并桥接的命令": (
        "            if (bridge) commands = terra::ros2::mergeCommands(std::move(commands), bridge->takeCommands());",
        "            (void)0;"),
    "速度执行器取错字段": (
        "            } else if (a->kind == td::ActuatorKind::Velocity) {",
        "            } else if (false) {"),
    "越界的命令不拒绝": (
        "            if (!std::isfinite(value) || (range && (value < range->first || value > range->second))) {",
        "            if (!std::isfinite(value)) {"),
    "tf_static 不保留给后来的订阅者": (
        # M4-11：M4-09 的分割对照表也用 transient local，锚点带上话题名以保持唯一
        "\"/tf_static\", rclcpp::QoS(1).reliable().transient_local()",
        "\"/tf_static\", rclcpp::QoS(1).reliable()"),
    "Headless 暂停时单步无效": (
        "                if (!paused || stepOnce) break;",
        "                if (!paused) break;"),
    "雷达不按频率发布": (
        # M5-11：3D 雷达的发布也有同样一行，锚点带上下一行以保持唯一
        "        if (!terra::simulation::sensorSampleDue(snapshot.stepIndex, dt, lidar.rateHz)) continue;\n        auto scan = terra::simulation::scanLidar(*b.scene.world, lidar);",
        "        if (false) continue;\n        auto scan = terra::simulation::scanLidar(*b.scene.world, lidar);"),
    "光学坐标系没有旋转": (
        "            add(c.name, c.name + \"_optical\", td::Pose{{0, 0, 0}, terra::scene::opticalRotation()});",
        "            add(c.name, c.name + \"_optical\", td::Pose{});"),
    "topicPrefix 不校验": (
        "    if (!p.topicPrefix.empty() && (p.topicPrefix.front() != '/' ||",
        "    if (false && (p.topicPrefix.front() != '/' ||"),
    "查看器不发布相机画面": (
        "            sim.ros2->publishCamera(k, image);",
        "            (void)image;"),
    "查看器不执行服务请求": (
        "            handleRos2Requests(frame);  // M4-06a",
        "            (void)0;"),
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
            n=$(awk -v head="---- 变异：$name" '$0 == head {found = 1} found' "$log" | grep -cE 'FAILED: |FAILED |不匹配|退出码不符')
            summary+=("  变异 $name：已发现（$n 项检查失败）")
        fi
    done <<'LIST'
Headless 不合并桥接的命令|apps/headless/main.cpp|terra_headless|ros2.bridge_commands
速度执行器取错字段|engine/ros2/src/bridge.cpp|terra_headless|ros2.bridge_commands
越界的命令不拒绝|engine/ros2/src/bridge.cpp|terra_headless|ros2.bridge_commands
tf_static 不保留给后来的订阅者|engine/ros2/src/bridge.cpp|terra_headless|ros2.bridge_headless
Headless 暂停时单步无效|apps/headless/main.cpp|terra_headless|ros2.bridge_headless
雷达不按频率发布|engine/ros2/src/bridge.cpp|terra_headless|ros2.bridge_headless
光学坐标系没有旋转|engine/ros2/src/bridge.cpp|terra_headless|ros2.bridge_headless
topicPrefix 不校验|engine/ros2/src/profile.cpp|test_ros2_profile|ros2.profile
查看器不发布相机画面|apps/viewer/main.cpp|terra_viewer|graphics.ros2_bridge_viewer
查看器不执行服务请求|apps/viewer/main.cpp|terra_viewer|graphics.ros2_bridge_viewer
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
if [ -f "$logs/ros2_ctest.txt" ]; then
    ctest --test-dir "$ros_build" -L ros2 -V 2>/dev/null | grep -E "INFO|FAILED|all passed" | sed 's/^[0-9]*: /  /'
fi

viewer="$ros_build/apps/viewer/terra_viewer"
echo "== 8. 可交互展示"
if [ $skip_show = 1 ]; then
    note "可交互展示" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "可交互展示" "未执行：不在桌面会话中"
elif [ -x "$viewer" ] && [ -f assets/robots/hello_robot_stretch_3/stretch.xml ]; then
    export ROS_DOMAIN_ID=77  # 展示用的域，与测试分开
    echo "（即将打开 terra_viewer 与 RViz2，最长 180 秒；关闭查看器即结束。设置文件：$out/show/settings.json）"
    echo "  - 查看器：Stretch 3，ROS2 桥接开启（菜单 仿真 → ROS2 桥接 打勾，状态栏显示 ROS2 订阅者数）"
    echo "  - 示例程序给 stretch3/head_pan 发正弦目标：头部左右转动，传感器面板与 RViz2 中的相机画面随之扫过"
    echo "  - RViz2：TF（各实体与传感器坐标系）、底盘雷达（红点）、头部相机彩色与深度；相机图像是真机的方向（横装，未旋转）"
    echo "  - 可在另一个终端试服务（先 source $ros_setup，并 export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1）："
    echo "      python3 scripts/ros2/terra_generic_example.py --pause    # 暂停；--step 10 单步；--resume 继续；--reset 重置"
    "$viewer" --scene assets/scenes/robots/stretch3.terra-scene.json --ros2 assets/ros2/profiles/generic.terra-ros2.json \
        --seconds 180 --settings "$out/show/settings.json" --library "$out/show/library.sqlite3" >"$logs/show.txt" 2>&1 &
    viewer_pid=$!
    sleep 4
    python3 scripts/ros2/terra_generic_example.py --wave stretch3/head_pan --center -1.0 --amplitude 1.2 --period 8 \
        >"$logs/show_example.txt" 2>&1 &
    example_pid=$!
    rviz_pid=
    if command -v rviz2 >/dev/null; then
        rviz2 -d assets/ros2/rviz/terra_stretch3.rviz --ros-args -p use_sim_time:=true >"$logs/show_rviz.txt" 2>&1 &
        rviz_pid=$!
    else
        echo "  （没有 rviz2：sudo apt install ros-humble-rviz2；本次只看查看器）"
    fi
    wait "$viewer_pid"
    rc=$?
    kill -INT "$example_pid" 2>/dev/null
    [ -n "$rviz_pid" ] && kill "$rviz_pid" 2>/dev/null
    wait 2>/dev/null
    grep -E "^INFO 场景|^INFO ROS2|^INFO 结束" "$logs/show.txt" | head -8 | sed 's/^/  /'
    grep -E "^话题|^给|仿真时间" "$logs/show_example.txt" | tail -3 | sed 's/^/  示例：/'
    grep -E "Traceback|Error" "$logs/show_example.txt" | head -3 | sed 's/^/  示例出错：/'
    record "可交互展示" "退出码 $rc" "$(ok $rc)"
else
    record "可交互展示" "未执行（含 ROS2 的构建失败或 Stretch 3 未准备）" 0
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
