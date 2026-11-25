#!/usr/bin/env bash
# M4-06b 基线机器验证：ROS2 桥接的宇树配置（M4-05 设计稿第 4、6 节，2026-11-24 评审通过）——
# 与真机相同的低层话题：/lowstate、/lf/lowstate（unitree_go / unitree_hg 的 LowState，含 CRC）、Go2 系列另有 /sportmodestate、
# /wirelesscontroller；订阅 /lowcmd（CRC 不对的拒绝），每步按 τ = tau + kp (q − q̂) + kd (dq − dq̂) 给力矩。
# 覆盖 Go2、Go2W（unitree_go）与 G1、AS2（unitree_hg）；G1 的位置执行器在组合时改为力矩执行器（场景文件 torqueActuators）。
# 宇树官方示例（unitree_ros2 668d1ec5）原样运行：go2_stand_example（Go2、Go2W 由趴下到站起）、g1_low_level_example（G1 脚踝摆动）。
#
# 请在**桌面会话终端**运行（第 8 步打开查看器）。需要（M4-05 设计稿第 6 节）：
#   - ~/unitree_ros2 检出到 668d1ec5，cyclonedds_ws 已编译（check_ros2.sh --setup 已确认消息可见）；
#   - 官方示例已编译：cd ~/unitree_ros2/example && source /opt/ros/humble/setup.bash && source ../cyclonedds_ws/install/setup.bash && colcon build
#     （没有编译时 3 个官方示例测试跳过，结论为“存在失败项”）。
#   其他位置用环境变量 UNITREE_ROS2=<目录> 指定。脚本不修改该目录。
# 依次执行（日志在 build/verify-m4-06b/logs/，屏幕只打印结论）：
#   1. 依赖与素材校验；ROS2 与宇树消息包检查；官方示例程序是否已编译
#   2. 不含 ROS2 的构建：渲染关闭 Debug（预期 85/85，新增 robots.unitree_lowlevel）；渲染开启 Debug 与 Release + -Werror（预期 118/118）
#   3. ASan/UBSan（不含 ROS2）
#   4. 含 ROS2 与宇树消息包的构建（Release + -Werror）：全量 CTest（预期 129/129：新增 ros2.unitree_crc、ros2.unitree_go2、
#      ros2.unitree_g1、ros2.unitree_as2、ros2.unitree_official_go2、_go2w、_g1）
#   5. ASan/UBSan（含 ROS2）：ros2 标签的测试（不查泄漏；关闭 new_delete_type_mismatch：rclcpp 上游缺陷，见 M4-06a 报告）
#   6. 变异测试（含 ROS2 的构建）：逐个注入 9 个已知错误
#   7. 输出：端到端检查的 INFO 行
#   8. 可交互展示（各最长 40 秒）：查看器打开 Go2（宇树配置），官方 go2_stand_example 让它站起；再打开 G1，g1_low_level_example 摆动脚踝
# 测试只在本机通信（ROS_LOCALHOST_ONLY=1），每个检查用单独的 ROS_DOMAIN_ID；清除 RMW_IMPLEMENTATION 与 CYCLONEDDS_URI，使用 Humble 默认的 DDS。
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m4_06b.sh [--jobs N] [--skip-asan] [--skip-mutation] [--skip-show] [--mutation-only]
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
        -h|--help) sed -n '2,29p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/verify-m4-06b"
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
    files=(engine/ros2/src/unitree.cpp apps/headless/main.cpp engine/physics/backends/mujoco/src/mujoco_world.cpp)
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
    "LowCmd 不校验 CRC": (
        "        if (profile_.checkCrc) {",
        "        if (false) {"),
    "力矩的位置项符号反了": (
        "                if (c.mode != 0) torque = c.tau + c.kp * (c.q - js.position[0]) + c.kd * (c.dq - js.velocity[0]);",
        "                if (c.mode != 0) torque = c.tau + c.kp * (js.position[0] - c.q) + c.kd * (c.dq - js.velocity[0]);"),
    "mode 为 0 时仍出力": (
        "                if (c.mode != 0) torque = c.tau + c.kp * (c.q - js.position[0]) + c.kd * (c.dq - js.velocity[0]);",
        "                torque = c.tau + c.kp * (c.q - js.position[0]) + c.kd * (c.dq - js.velocity[0]);"),
    "收到命令前不让位": (
        "        out.reserve(motors_.size());",
        "        if (!received) return out;"),
    "hg 的 reserve 全部参与 CRC": (
        "        p.put(m.reserve[0]);\n        p.put(std::uint32_t{0});\n        p.put(std::uint32_t{0});\n        p.put(std::uint32_t{0});",
        "        p.put(m.reserve);"),
    "CRC 多项式错误": (
        "    const std::uint32_t polynomial = 0x04C11DB7u;",
        "    const std::uint32_t polynomial = 0x04C11DB6u;"),
    "LowState 的 q 取错关节": (
        "                ms.q = static_cast<float>(s.joints[m.joint].position[0]);",
        "                ms.q = static_cast<float>(s.joints[m.slot].position[0]);"),
    "Headless 不合并 LowCmd 的力矩": (
        "            if (bridge) commands = terra::ros2::mergeCommands(std::move(commands), bridge->directCommands());",
        "            (void)0;"),
    "torqueActuators 不改执行器": (
        "        if (!models[i].torqueActuators) continue;",
        "        continue;"),
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
LowCmd 不校验 CRC|engine/ros2/src/unitree.cpp|terra_headless|ros2.unitree_go2
力矩的位置项符号反了|engine/ros2/src/unitree.cpp|terra_headless|ros2.unitree_g1
mode 为 0 时仍出力|engine/ros2/src/unitree.cpp|terra_headless|ros2.unitree_go2
收到命令前不让位|engine/ros2/src/unitree.cpp|terra_headless|ros2.unitree_go2
hg 的 reserve 全部参与 CRC|engine/ros2/src/unitree.cpp|test_unitree_crc|ros2.unitree_crc
CRC 多项式错误|engine/ros2/src/unitree.cpp|test_unitree_crc|ros2.unitree_crc
LowState 的 q 取错关节|engine/ros2/src/unitree.cpp|terra_headless|ros2.unitree_go2
Headless 不合并 LowCmd 的力矩|apps/headless/main.cpp|terra_headless|ros2.unitree_go2
torqueActuators 不改执行器|engine/physics/backends/mujoco/src/mujoco_world.cpp|test_unitree_lowlevel|robots.unitree_lowlevel
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
    ctest --test-dir "$ros_build" -R '^(ros2\.unitree|robots\.unitree_lowlevel)' -V 2>/dev/null |
        grep -E "INFO|FAILED|SKIP|all passed" | grep -v "ROS_DOMAIN_ID\|启动：" | sed 's/^[0-9]*: /  /'
fi

viewer="$ros_build/apps/viewer/terra_viewer"
echo "== 8. 可交互展示"
show_robot() {  # show_robot <场景> <配置> <示例程序名> <说明>
    local scene=$1 profile=$2 name=$3 text=$4
    local exe
    exe=$(find "$unitree/example" -type f -name "$name" -perm -u+x 2>/dev/null | head -1)
    echo "  $text"
    "$viewer" --scene "$scene" --ros2 "$profile" --seconds 40 --settings "$out/show/settings.json" \
        --library "$out/show/library.sqlite3" >"$logs/show_$name.txt" 2>&1 &
    local viewer_pid=$!
    local example_pid=
    sleep 6
    if [ -n "$exe" ]; then
        "$exe" >"$logs/show_${name}_example.txt" 2>&1 &
        example_pid=$!
    else
        echo "  （没有找到官方示例 $name，只显示场景）"
    fi
    wait "$viewer_pid"
    local rc=$?
    [ -n "$example_pid" ] && kill -INT "$example_pid" 2>/dev/null
    wait 2>/dev/null
    grep -E "^INFO ROS2|^INFO 结束" "$logs/show_$name.txt" | head -4 | sed 's/^/    /'
    record "可交互展示（$name）" "退出码 $rc" "$(ok $rc)"
}
if [ $skip_show = 1 ]; then
    note "可交互展示" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "可交互展示" "未执行：不在桌面会话中"
elif [ -x "$viewer" ]; then
    export ROS_DOMAIN_ID=78  # 展示用的域，与测试分开
    echo "（即将依次打开两次 terra_viewer，各 40 秒；设置文件：$out/show/settings.json）"
    show_robot assets/scenes/robots/go2_lowlevel.terra-scene.json assets/ros2/profiles/unitree_go2.terra-ros2.json go2_stand_example \
        "Go2：没有命令时趴下；6 秒后官方 go2_stand_example 原样运行：先收腿趴好，再站起、保持，最后分开四腿趴下"
    show_robot assets/scenes/robots/g1_lowlevel.terra-scene.json assets/ros2/profiles/unitree_g1.terra-ros2.json g1_low_level_example \
        "G1（悬挂）：官方 g1_low_level_example 原样运行：3 秒内各关节回到 0，之后两侧脚踝画圈、手腕转动"
else
    record "可交互展示" "未执行（含 ROS2 的构建失败）" 0
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
