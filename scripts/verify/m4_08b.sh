#!/usr/bin/env bash
# M4-08b 基线机器验证：TurtleBot 4 标准版（M4-08b 设计稿 2026-11-25 评审通过）——
# 导入配方（*.terra-import.json）；TurtleBot 4 与 Create 3 的描述文件锁定、按配方导入（两次逐字节相同）；
# 传感器按真机（RPLIDAR、OAK-D Pro 的 RGB 与深度、Create 3 的 IMU）；差速底盘；
# ROS2：话题名映射、真机的坐标系（去掉实例前缀，传感器用连杆坐标系）、/cmd_vel 与 /odom、深度 16UC1；
# 文件浏览窗口：列表视图的图标间隔、地址栏点击即可编辑（用户在 M4-08a 验收时提出）。
#
# 请在**桌面会话终端**运行（第 8 步打开查看器与 RViz2）。需要（设计稿第 8 节）：TurtleBot 4 与 Create 3 的描述文件在 ROBOTS/；
# assimp；ROS2 Humble 与 unitree_ros2 的 cyclonedds_ws（与 M4-06b 相同）；键盘遥控（展示用）：sudo apt install ros-humble-teleop-twist-keyboard
# 依次执行（日志在 build/verify-m4-08b/logs/，屏幕只打印结论）：
#   1. 依赖与素材校验（参考机器人 291 个文件，含 TurtleBot 4 与 Create 3 的 46 个）；ROS2 与宇树消息包检查
#   2. 不含 ROS2 的构建：渲染关闭 Debug（预期 94/94，新增 robots.turtlebot4_import）；渲染开启 Debug 与 Release + -Werror（预期 129/129）
#      之后用 Release 的 terra_import 按配方导入 TurtleBot 4（scripts/assets/import_turtlebot4.sh → assets/robots/turtlebot4/）
#   3. ASan/UBSan（不含 ROS2）
#   4. 含 ROS2 与宇树消息包的构建（Release + -Werror）：全量 CTest（预期 141/141，新增 ros2.turtlebot4）
#   5. ASan/UBSan（含 ROS2）：ros2 标签的测试
#   6. 变异测试（含 ROS2 的构建）：逐个注入 8 个已知错误
#   7. 输出：TurtleBot 4 的导入、驱动与 ROS2 检查的 INFO 行
#   8. 可交互展示（最长 300 秒）：查看器打开 TurtleBot 4（ROS2 桥接开启），RViz2 显示 TF、雷达、相机与里程计；
#      另开终端运行 teleop_twist_keyboard 开动；顺便试新的文件浏览窗口
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m4_08b.sh [--jobs N] [--skip-asan] [--skip-mutation] [--skip-show] [--mutation-only]
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
        -h|--help) sed -n '2,24p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/verify-m4-08b"
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
    record "导入 TurtleBot 4（配方）" "退出码 $rc；$(grep -c '^terra_import: \[warning\]' "$logs/import_turtlebot4.txt") 个警告（见 $logs/import_turtlebot4.txt）" "$(ok $rc)"
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
    files=(engine/import/src/importer.cpp engine/ros2/src/bridge.cpp)
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
    "配方的关节覆盖不生效": (
        "                j.type = \"fixed\";",
        "                (void)0;"),
    "配方的传感器参数不覆盖默认值": (
        "                    for (auto it = extra.begin(); it != extra.end(); ++it) sensor[it.key()] = it.value();",
        "                    (void)extra;"),
    "导入不写配方": (
        "    writeRecipe((dir / \"import.terra-import.json\").string(), plan, options);",
        "    (void)0;"),
    "差速底盘左右轮反了": (
        "        double wl = (v - w * dd.wheelSeparation / 2.0) / dd.wheelRadius;",
        "        double wl = (v + w * dd.wheelSeparation / 2.0) / dd.wheelRadius;"),
    "里程计不积分位置": (
        "            drive.x += v * std::cos(mid) * h;",
        "            (void)mid;"),
    "cmd_vel 不超时": (
        "            if (drive.haveCommand && age <= dd.timeout) {",
        "            if (drive.haveCommand) {"),
    "话题名映射不生效": (
        "        return it == profile.topics.end() ? defaultName : it->second;",
        "        return defaultName;"),
    "坐标系不去前缀": (
        "        return !p.empty() && name.size() > p.size() && name.compare(0, p.size(), p) == 0 ? name.substr(p.size()) : name;",
        "        return name;"),
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
配方的关节覆盖不生效|engine/import/src/importer.cpp|test_import_analyze|import.analyze
配方的传感器参数不覆盖默认值|engine/import/src/importer.cpp|test_import_execute|import.execute
导入不写配方|engine/import/src/importer.cpp|test_import_execute|import.execute
差速底盘左右轮反了|engine/ros2/src/bridge.cpp|terra_headless|ros2.turtlebot4
里程计不积分位置|engine/ros2/src/bridge.cpp|terra_headless|ros2.turtlebot4
cmd_vel 不超时|engine/ros2/src/bridge.cpp|terra_headless|ros2.turtlebot4
话题名映射不生效|engine/ros2/src/bridge.cpp|terra_headless|ros2.turtlebot4
坐标系不去前缀|engine/ros2/src/bridge.cpp|terra_headless|ros2.turtlebot4
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
    ctest --test-dir "$ros_build" -R '^(robots\.turtlebot4_import|ros2\.turtlebot4|import\.(analyze|execute))$' -V 2>/dev/null |
        grep -E "INFO|FAILED|SKIP|all passed" | grep -v "ROS_DOMAIN_ID\|启动：" | sed 's/^[0-9]*: /  /' | cut -c1-260
fi

viewer="$ros_build/apps/viewer/terra_viewer"
echo "== 8. 可交互展示"
if [ $skip_show = 1 ]; then
    note "可交互展示" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "可交互展示" "未执行：不在桌面会话中"
elif [ -x "$viewer" ] && [ -f assets/robots/turtlebot4/turtlebot4.terra-scene.json ]; then
    export ROS_DOMAIN_ID=79  # 展示用的域，与测试分开
    echo "（即将打开 terra_viewer 与 RViz2，最长 300 秒；关闭查看器即结束）"
    echo "  - 查看器：TurtleBot 4 在房间中，ROS2 桥接为 TurtleBot 4 配置（与真机相同的话题名）"
    echo "  - RViz2：固定坐标系 odom；TF、/scan（红点）、/odom、OAK-D 的 RGB 与深度"
    echo "  - 另开一个终端开动机器人（i 前进、j / l 转向、k 停）："
    echo "      source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=79 ROS_LOCALHOST_ONLY=1 && ros2 run teleop_twist_keyboard teleop_twist_keyboard"
    echo "  - 也可以试新的文件浏览窗口（文件 → 打开场景…）：列表视图图标与名字的间隔；点击地址栏空白处直接输入路径，Esc 恢复"
    "$viewer" --scene assets/robots/turtlebot4/turtlebot4.terra-scene.json --ros2 assets/ros2/profiles/turtlebot4.terra-ros2.json \
        --seconds 300 --settings "$out/show/settings.json" --library "$out/show/library.sqlite3" >"$logs/show.txt" 2>&1 &
    viewer_pid=$!
    rviz_pid=
    if command -v rviz2 >/dev/null; then
        sleep 3
        rviz2 -d assets/ros2/rviz/terra_turtlebot4.rviz --ros-args -p use_sim_time:=true >"$logs/show_rviz.txt" 2>&1 &
        rviz_pid=$!
    fi
    wait "$viewer_pid"
    rc=$?
    [ -n "$rviz_pid" ] && kill "$rviz_pid" 2>/dev/null
    wait 2>/dev/null
    grep -E "^INFO 场景 |^INFO ROS2|^INFO 结束" "$logs/show.txt" | head -6 | sed 's/^/  /'
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
