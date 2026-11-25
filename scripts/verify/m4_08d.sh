#!/usr/bin/env bash
# M4-08d 基线机器验证：ROS2 面板（RViz A 层）与显示配置（M4-08d 设计稿 2026-11-25 评审通过；用户要求为以后的导出格式与参数留出余地）——
# ROS2 监视器（独立节点；运行时按 introspection 展开任意消息，含宇树 LowState）；话题表、消息、曲线、图像（含 PNG / JPEG 压缩图像）、TF 树；
# 显示配置 .terviz（保留不认识的内容）存入资源库并在打开场景时恢复；导出器注册表，导出 .terviz 与 .rviz（弹窗中选择）；
# 传感器面板的 IMU 可以弹出为曲线窗口（仿真来源）。
#
# 请在**桌面会话终端**运行（第 8 步打开查看器）。依赖与 M4-08c 相同。
# 依次执行（日志在 build/verify-m4-08d/logs/，屏幕只打印结论）：
#   1. 依赖与素材校验；ROS2 与宇树消息包检查
#   2. 不含 ROS2 的构建：渲染关闭 Debug（预期 100/100）；渲染开启 Debug 与 Release + -Werror（预期 137/137）
#   3. ASan/UBSan（不含 ROS2）
#   4. 含 ROS2 与宇树消息包的构建（Release + -Werror）：全量 CTest（预期 151/151，新增 ros2.monitor、graphics.viewer_ros2_panel_selftest）
#   5. ASan/UBSan（含 ROS2）：ros2 标签的测试（预期 14/14）
#   6. 变异测试（含 ROS2 的渲染构建）：逐个注入 8 个已知错误
#   7. 输出：显示配置、监视器与查看器自检的 INFO 行
#   8. 可交互展示（最长 300 秒）：查看器打开 TurtleBot 4（ROS2 桥接开启），在 ROS2 面板中查看话题、曲线、图像与 TF 树，导出 .rviz 后用 RViz2 打开
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m4_08d.sh [--jobs N] [--skip-asan] [--skip-mutation] [--skip-show] [--mutation-only]
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

out="$root/build/verify-m4-08d"
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
    files=(engine/viz/src/viz_config.cpp engine/viz/src/exporter.cpp engine/ros2/src/monitor.cpp apps/viewer/main.cpp)
    for f in "${files[@]}"; do cp "$f" "$out/${f//\//_}.orig"; done
    restore() { for f in "${files[@]}"; do cp "$out/${f//\//_}.orig" "$f"; done; }
    trap 'restore; echo "已恢复被变异的源文件"; exit 130' INT TERM
    # 不开 -Werror：变异只改语义，不应因警告而无法判定
    cmake -S . -B "$b" -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug -DTERRA_BUILD_RENDER=ON "${ros_flags[@]}" >"$log" 2>&1
    detected=0
    total=0
    while IFS='|' read -r name file target testname; do
        total=$((total + 1))
        restore
        if ! python3 - "$file" "$name" >>"$log" 2>&1 <<'PY'
import sys
path, name = sys.argv[1], sys.argv[2]
mutations = {
    "不认识的参数不保留": (
        "            extra[it.key()] = it.value();",
        "            (void)0;"),
    "导出参数不校验范围": (
        "                else if (it->max > it->min && (d < it->min || d > it->max)) {",
        "                else if (false) {"),
    ".rviz 漏掉固定坐标系": (
        "          << \"    Fixed Frame: \" << quoted(v.at(\"fixedFrame\")) << \"\\n\"",
        "          << \"    Fixed Frame: map\\n\""),
    "频率按条数计算": (
        "        s.hz = static_cast<double>(n - 1) / (lastT - first);",
        "        s.hz = static_cast<double>(n);"),
    "数组下标不检查越界": (
        "            if (i >= n) {",
        "            if (false) {"),
    "16UC1 不换算为米": (
        "static_cast<float>(v) * 0.001f;",
        "static_cast<float>(v);"),
    "/tf_static 不标为静态": (
        "[self](const tf2_msgs::msg::TFMessage& m) { self->receiveTf(m, true); });",
        "[self](const tf2_msgs::msg::TFMessage& m) { self->receiveTf(m, false); });"),
    "打开场景时不恢复显示配置": (
        "    if (const auto it = map.find(f.simulated->path); it != map.end() && std::filesystem::exists(it->second, ec) && loadDisplay(f, it->second, false)) return;",
        "    (void)map;\n    (void)ec;"),
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
不认识的参数不保留|engine/viz/src/viz_config.cpp|test_viz|viz.config
导出参数不校验范围|engine/viz/src/exporter.cpp|test_viz|viz.config
.rviz 漏掉固定坐标系|engine/viz/src/exporter.cpp|test_viz|viz.config
频率按条数计算|engine/ros2/src/monitor.cpp|test_ros2_monitor|ros2.monitor
数组下标不检查越界|engine/ros2/src/monitor.cpp|test_ros2_monitor|ros2.monitor
16UC1 不换算为米|engine/ros2/src/monitor.cpp|test_ros2_monitor|ros2.monitor
/tf_static 不标为静态|engine/ros2/src/monitor.cpp|test_ros2_monitor|ros2.monitor
打开场景时不恢复显示配置|apps/viewer/main.cpp|terra_viewer|graphics.viewer_viz_selftest
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
    ctest --test-dir "$ros_build" -R '^(viz\.config|ros2\.monitor|graphics\.viewer_(viz|ros2_panel)_selftest)$' -V 2>/dev/null |
        grep -E "INFO|PASS|FAILED|SKIP|all passed|自检" | grep -v "启动：\|已加载资产\|资产 .* 信息" | sed 's/^[0-9]*: /  /' | cut -c1-260
fi

viewer="$ros_build/apps/viewer/terra_viewer"
echo "== 8. 可交互展示"
if [ $skip_show = 1 ]; then
    note "可交互展示" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "可交互展示" "未执行：不在桌面会话中"
elif [ -x "$viewer" ] && [ -f assets/robots/turtlebot4/turtlebot4.terra-scene.json ]; then
    export ROS_DOMAIN_ID=79  # 展示用的域，与测试分开
    mkdir -p "$out/show"
    echo "（即将打开 terra_viewer，最长 300 秒；关闭窗口即结束。资源库与设置在 $out/show/，不读写你的用户目录）"
    echo "  - 查看器打开 TurtleBot 4，ROS2 桥接已开启（与真机相同的话题名）；按播放让它运行"
    echo "  - 菜单 窗口 → ROS2：话题表；行尾的按钮一键打开 消息 / 图像 / TF 树 / 曲线（/imu、/odom 有一键曲线）"
    echo "  - 消息窗口中数值字段旁的 + 可把该字段加入曲线；传感器面板的 IMU 可“弹出为曲线窗口”（仿真来源）"
    echo "  - 窗口 → 显示配置 → 保存；关闭窗口后重新单击 TurtleBot 4 的卡片，窗口会恢复"
    echo "  - ROS2 面板上的“导出…”：勾选 .terviz 与 .rviz；之后可用 RViz2 打开导出的 .rviz 对照："
    echo "      source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=79 ROS_LOCALHOST_ONLY=1 && rviz2 -d $out/show/exports/默认.rviz --ros-args -p use_sim_time:=true"
    echo "  - 另开终端开动机器人（i 前进、j / l 转向、k 停）：同上 source 后 ros2 run teleop_twist_keyboard teleop_twist_keyboard"
    "$viewer" --scene assets/robots/turtlebot4/turtlebot4.terra-scene.json --ros2 assets/ros2/profiles/turtlebot4.terra-ros2.json \
        --seconds 300 --settings "$out/show/settings.json" --library "$out/show/library.sqlite3" >"$logs/show.txt" 2>&1
    rc=$?
    grep -E "^INFO ROS2|^INFO 显示配置|^INFO 结束" "$logs/show.txt" | head -12 | sed 's/^/  /'
    record "可交互展示" "退出码 $rc" "$(ok $rc)"
else
    record "可交互展示" "未执行（构建失败或 TurtleBot 4 未导入）" 0
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
