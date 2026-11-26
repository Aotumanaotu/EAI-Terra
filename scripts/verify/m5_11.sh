#!/usr/bin/env bash
# M5-11 基线机器验证：Livox Mid-360 3D 激光雷达（M5-01 设计稿第 4 节，2026-11-26 评审通过）——
# 扫描图案取自 Livox 官方仿真插件的 mid360.csv（MIT，锁定提交与 sha256，按帧推进、4 s 循环）；参数按 Livox 规格页
# （360° × −7°～52°、200,000 点/秒、10 Hz、40 m、盲区 0.1 m）；距离与方向噪声；运动畸变（每点按测量时刻的位姿发出）；
# 内置 IMU（ICM40609 的数据手册噪声，200 Hz）；Headless、terra_capture、ROS2（PointCloud2，与 livox_ros_driver2 相同的字段）、
# 实验记录与复现、查看器（传感器面板与视口叠加的点云）。射线检测改用 mj_multiRay。
#
# 请在**桌面会话终端**运行（第 9 步打开查看器）。第一次运行时下载扫描图案（16.8 MB，GitHub）。
# 依次执行（日志在 build/verify-m5-11/logs/，屏幕只打印结论）：
#   1. 依赖与素材校验（含 Livox 扫描图案）；ROS2 与宇树消息包检查
#   2. 不含 ROS2 的构建：渲染关闭 Debug（预期 111/111）；渲染开启 Debug 与 Release + -Werror（预期 152/152）
#   3. ASan/UBSan（不含 ROS2）：预期 120 + 32
#   4. 含 ROS2 与宇树消息包的构建（Release + -Werror）：全量 CTest（预期 168/168）
#   5. ASan/UBSan（含 ROS2）：ros2 标签的测试（预期 16/16）
#   6. 变异测试（不含 ROS2 的渲染构建）：逐个注入 6 个已知错误
#   7. 输出：噪声统计、运动畸变、Mid-360 单帧耗时；Headless 与 ROS2 的检查结果
#   8. 组件命令：--list-components
#   9. 可交互展示（最长 300 秒）：Go2 + Mid-360 运行你的行走策略，视口中叠加点云
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m5_11.sh [--jobs N] [--skip-asan] [--skip-mutation] [--skip-show] [--mutation-only]
# 运行中终端底行显示进度（第几部分、编译 / 测试进度条、已用时间；第二次起按上次的耗时估计剩余时间，记在 build/verify-timings/）
set -uo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
cd "$root"
# shellcheck source=lib/progress.sh
source "$root/scripts/verify/lib/progress.sh"

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
        -h|--help) sed -n '2,21p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/verify-m5-11"
# ONNX Runtime 组件装在验证目录之外，重复运行时沿用（不重复下载 184 MB）
policy_library="$root/build/policy_library/library.db"
policy_flags=("-DTERRA_POLICY_LIBRARY=$policy_library")
logs="$out/logs"
rm -rf "$out"
mkdir -p "$logs"
progress_init m5_11 9

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

section 1 "依赖与 ROS2"
run_step "check_json" "$logs/json.txt" bash scripts/settings/check_json.sh
rc=$?
record "nlohmann/json 校验" "$(grep -c '^一致' "$logs/json.txt")/3 个文件一致，退出码 $rc" "$(ok $rc)"
run_step "check_stb" "$logs/stb.txt" bash scripts/render/check_stb.sh
rc=$?
record "stb 校验" "$(grep -c '^一致' "$logs/stb.txt")/3 个文件一致，退出码 $rc" "$(ok $rc)"
run_step "check_cgltf" "$logs/cgltf.txt" bash scripts/render/check_cgltf.sh
rc=$?
record "cgltf 校验" "$(grep -c '^一致' "$logs/cgltf.txt")/2 个文件一致，退出码 $rc" "$(ok $rc)"
run_step "fetch" "$logs/fetch.txt" bash scripts/render/fetch.sh
rc=$?
record "获取并校验渲染依赖" "退出码 $rc" "$(ok $rc)"
run_step "fetch_hdri" "$logs/hdri.txt" bash scripts/assets/fetch_hdri.sh
rc=$?
record "HDRI 贴图" "$(grep -cE '^(已存在且一致|就绪)' "$logs/hdri.txt")/2 张一致，退出码 $rc" "$(ok $rc)"
run_step "fetch_models" "$logs/models.txt" bash scripts/assets/fetch_models.sh
rc=$?
record "glTF 资产" "$(tail -1 "$logs/models.txt")，退出码 $rc" "$(ok $rc)"
run_step "fetch_robots" "$logs/robots.txt" bash scripts/assets/fetch_robots.sh
rc=$?
record "参考机器人" "$(tail -1 "$logs/robots.txt")，退出码 $rc" "$(ok $rc)"
run_step "fetch_lidar_patterns" "$logs/lidar_patterns.txt" bash scripts/assets/fetch_lidar_patterns.sh
rc=$?
record "Livox 扫描图案" "$(tail -1 "$logs/lidar_patterns.txt")，退出码 $rc" "$(ok $rc)"
run_step "check_third_party" "$logs/third_party.txt" bash scripts/import/check_third_party.sh
rc=$?
record "tinyxml2、xacro、miniz、ORT 头文件" "$(grep -c '^一致' "$logs/third_party.txt")/16 个文件一致，退出码 $rc" "$(ok $rc)"
if [ ! -f third_party/render/install/lib/cmake/glfw3/glfw3Config.cmake ]; then
    echo "错误：未找到 GLFW 安装，请先运行 scripts/render/build.sh（M1-02）" >&2
    exit 1
fi

# ONNX Runtime 组件（M4-10a）：用刚构建的 terra_headless 安装到 build/policy_library（已安装且校验通过时跳过）
ort_state=""
ensure_ort() {  # ensure_ort <terra_headless>
    [ -n "$ort_state" ] && return 0
    local hl=$1
    [ -x "$hl" ] || return 0
    if "$hl" --list-components --library "$policy_library" 2>/dev/null | grep -q '"id":"onnxruntime-cpu".*"state":"installed"'; then
        ort_state=installed
        record "ONNX Runtime 组件" "已安装（$(dirname "$policy_library")/components/onnxruntime-cpu）" 1
        return 0
    fi
    local from=()
    [ -n "${TERRA_ORT_NUPKG:-}" ] && from=(--from "$TERRA_ORT_NUPKG")
    echo "  安装 ONNX Runtime 1.32.0（184 MB）……"
    run_step "安装 ONNX Runtime" "$logs/install_ort.txt" "$hl" --install-component onnxruntime-cpu --library "$policy_library" "${from[@]}"
    local rc=$?
    if [ $rc = 0 ]; then
        ort_state=installed
        record "ONNX Runtime 组件" "已下载并安装（$(grep -c '下载 ' "$logs/install_ort.txt") 行进度，sha256 一致）" 1
    else
        ort_state=failed
        record "ONNX Runtime 组件" "安装失败，退出码 $rc（见 $logs/install_ort.txt；离线时用 TERRA_ORT_NUPKG=<nupkg 文件>）" 0
        grep -E "警告|失败|错误" "$logs/install_ort.txt" | head -5 | sed 's/^/    /'
    fi
}

run_preset() {  # run_preset <preset> <名称> [额外配置参数...]
    [ $mutation_only = 1 ] && return 0
    local preset=$1 label=$2; shift 2
    local log="$logs/$preset.txt"
    rm -rf "build/$preset"
    if run_step "$label 配置" "$log" cmake --preset "$preset" "${policy_flags[@]}" "$@" &&
       run_step "$label 编译" "$log" cmake --build --preset "$preset" -j "$jobs"; then
        ensure_ort "build/$preset/apps/headless/terra_headless"
        if grep -q "warning:" "$log"; then
            record "$label 构建" "有编译警告（见 $log）" 0
            grep -A3 "warning:" "$log" | head -20 | sed 's/^/    /'
        fi
        run_step "$label 测试" "$log" ctest --preset "$preset"
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
run_step "check_ros2" "$logs/check_ros2.txt" bash scripts/ros2/check_ros2.sh --setup "$unitree_setup"
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

section 2 "不含 ROS2 的构建"
run_preset debug "渲染关闭 Debug"
run_preset render-debug "渲染 Debug"
run_preset render-release "渲染 Release -Werror" -DTERRA_WARNINGS_AS_ERRORS=ON
if [ $mutation_only = 0 ]; then
    run_step "import_turtlebot4" "$logs/import_turtlebot4.txt" bash scripts/assets/import_turtlebot4.sh build/render-release/apps/import/terra_import
    rc=$?
    record "重新导入 TurtleBot 4" "退出码 $rc；$(grep -c '^terra_import: \[warning\]' "$logs/import_turtlebot4.txt") 个警告（见 $logs/import_turtlebot4.txt）" "$(ok $rc)"
    [ $rc = 0 ] || grep -E '\[error\]' "$logs/import_turtlebot4.txt" | head -10 | sed 's/^/    /'
fi

san="-fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer"
if [ $skip_asan = 0 ]; then
    section 3 "ASan/UBSan（不含 ROS2）"
    log="$logs/asan.txt"
    b="$out/asan"
    if run_step "ASan 配置" "$log" cmake -S . -B "$b" -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug -DTERRA_BUILD_RENDER=ON \
             -DTERRA_WARNINGS_AS_ERRORS=ON "-DCMAKE_CXX_FLAGS=$san" "-DCMAKE_EXE_LINKER_FLAGS=$san" &&
       run_step "ASan 编译" "$log" cmake --build "$b" -j "$jobs"; then
        run_step "ASan 非图形测试" "$logs/asan_nongraphics.txt" env ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=print_stacktrace=1 \
            ctest --test-dir "$b" -LE graphics --output-on-failure
        rc1=$?
        run_step "ASan 图形测试" "$logs/asan_graphics.txt" env ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=print_stacktrace=1 \
            ctest --test-dir "$b" -L graphics --output-on-failure
        rc2=$?
        record "ASan 非图形（含泄漏检测）" "$(ctest_result "$logs/asan_nongraphics.txt")；跳过 $(grep -cE '\*\*\*Skipped' "$logs/asan_nongraphics.txt") 个（预期 2：policy.onnxruntime、policy.go2_walk）" "$(ok $rc1)"
        record "ASan 图形（不查泄漏）" "$(ctest_result "$logs/asan_graphics.txt")" "$(ok $rc2)"
    else
        record "ASan 构建" "失败（见 $log）" 0
    fi
fi

# ---- 以下需要 ROS2 与宇树消息包 ----
if [ ! -f "$ros_setup" ] || [ ! -f "$unitree_setup" ]; then
    record "ROS2" "未找到 $ros_setup 或 $unitree_setup，第 4～8 步无法执行" 0
    progress_finish
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
    section 4 "含 ROS2 与宇树消息包的构建"
    log="$logs/ros2_build.txt"
    if run_step "含 ROS2 配置" "$log" cmake -S . -B "$ros_build" -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Release -DTERRA_BUILD_RENDER=ON \
             "${ros_flags[@]}" "${policy_flags[@]}" -DTERRA_WARNINGS_AS_ERRORS=ON &&
       run_step "含 ROS2 编译" "$log" cmake --build "$ros_build" -j "$jobs"; then
        grep -E "ROS2 桥接：" "$log" | head -2 | sed 's/^-- /  /'
        if ! grep -q "宇树消息包：ON" "$log"; then
            record "含 ROS2 的构建" "CMake 没有找到宇树消息包（见 $log）" 0
        fi
        if grep -q "warning:" "$log"; then
            record "含 ROS2 构建" "有编译警告（见 $log）" 0
            grep -A3 "warning:" "$log" | head -20 | sed 's/^/    /'
        fi
        run_step "含 ROS2 测试" "$logs/ros2_ctest.txt" ctest --test-dir "$ros_build" --output-on-failure
        rc=$?
        record "含 ROS2 Release -Werror 测试" "$(ctest_result "$logs/ros2_ctest.txt")" "$(ok $rc)"
        [ $rc = 0 ] || grep -E "^\s+[0-9]+ - |FAILED" "$logs/ros2_ctest.txt" | head -30 | sed 's/^/    /'
        skipped=$(grep -cE "\*\*\*Skipped|Not Run.*Skipped" "$logs/ros2_ctest.txt")
        [ "$skipped" = 0 ] || record "跳过的测试" "$skipped 个（见 $logs/ros2_ctest.txt；官方示例未编译、ONNX Runtime 没有安装或没有用户策略时跳过）" 0
    else
        record "含 ROS2 构建" "失败（见 $log）" 0
        grep -E -A3 "error:|warning:|CMake Error" "$log" | head -30 | sed 's/^/    /'
    fi
fi

if [ $skip_asan = 0 ]; then
    section 5 "ASan/UBSan（含 ROS2）"
    log="$logs/ros2_asan.txt"
    b="$out/ros2_asan"
    if run_step "ASan 含 ROS2 配置" "$log" cmake -S . -B "$b" -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug -DTERRA_BUILD_RENDER=ON "${ros_flags[@]}" \
             -DTERRA_WARNINGS_AS_ERRORS=ON "-DCMAKE_CXX_FLAGS=$san" "-DCMAKE_EXE_LINKER_FLAGS=$san" &&
       run_step "ASan 含 ROS2 编译" "$log" cmake --build "$b" -j "$jobs"; then
        # new_delete_type_mismatch=0：rclcpp Humble 头文件中 retyped_deallocate<char> 的上游缺陷（M4-06a 报告第 5.2 节）
        run_step "ASan 含 ROS2 测试" "$logs/ros2_asan_ctest.txt" env ASAN_OPTIONS=detect_leaks=0:new_delete_type_mismatch=0 UBSAN_OPTIONS=print_stacktrace=1 \
            ctest --test-dir "$b" -L ros2 --output-on-failure
        rc=$?
        record "ASan 含 ROS2（ros2 标签）" "$(ctest_result "$logs/ros2_asan_ctest.txt")" "$(ok $rc)"
        [ $rc = 0 ] || grep -E "ERROR: AddressSanitizer|SUMMARY: |runtime error|FAILED" "$logs/ros2_asan_ctest.txt" | head -20 | sed 's/^/    /'
    else
        record "ASan 含 ROS2 构建" "失败（见 $log）" 0
    fi
fi

if [ $skip_mutation = 0 ]; then
    section 6 "变异测试"
    log="$logs/mutation.txt"
    b="$out/mutation"
    files=(engine/simulation/src/lidar3d.cpp engine/simulation/src/sensor_noise.cpp engine/simulation/src/record.cpp engine/scene/src/scene_file.cpp)
    for f in "${files[@]}"; do cp "$f" "$out/${f//\//_}.orig"; done
    restore() { for f in "${files[@]}"; do cp "$out/${f//\//_}.orig" "$f"; done; }
    trap 'restore; echo "已恢复被变异的源文件"; exit 130' INT TERM
    # 不开 -Werror：变异只改语义，不应因警告而无法判定
    run_step "变异 配置" "$log" cmake -S . -B "$b" -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug -DTERRA_BUILD_RENDER=ON "${policy_flags[@]}"
    detected=0
    total=0
    while IFS='|' read -r name file target testname; do
        total=$((total + 1))
        restore
        if ! python3 - "$file" "$name" >>"$log" 2>&1 <<'PY'
import sys
path, name = sys.argv[1], sys.argv[2]
mutations = {
    "图案不按帧推进": (
        "    scan.patternStart = static_cast<std::size_t>((scan.frame * n) % pattern.size());",
        "    scan.patternStart = 0;"),
    "运动畸变不回推": (
        "        const double back = moving ? std::max(0.0, lastTime - tau) : 0.0;",
        "        const double back = 0.0;"),
    "点的时间偏移为 0": (
        "        p.time = static_cast<float>(static_cast<double>(i) / lidar.pointsPerSecond);",
        "        p.time = 0.0f;"),
    "距离噪声缺失": (
        "        double rr = r + (sigma > 0 ? sigma * noiseGaussian(seed_, stream, scan.step, base) : 0.0);",
        "        double rr = r;"),
    "记录的点云摘要错": (
        "    j[\"sha256\"] = lidar3dDigest(scan);",
        "    j[\"sha256\"] = std::string(64, '0');"),
    "内置 IMU 不按预设偏移": (
        "                m.pose = multiply(s.pose, offset);",
        "                m.pose = s.pose;"),
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
        if ! run_step "变异 $total/6 编译" "$log" cmake --build "$b" -j "$jobs" --target "$target"; then
            summary+=("  变异 $name：编译失败，未判定（见 $log）")
            failed=1
            continue
        fi
        if run_step "变异 $total/6 测试" "$log" ctest --test-dir "$b" -R "^${testname//./\\.}\$" --output-on-failure; then
            summary+=("  变异 $name：未被发现")
        else
            detected=$((detected + 1))
            n=$(awk -v head="---- 变异：$name" '$0 == head {found = 1} found' "$log" | grep -cE 'FAILED: |FAILED |不匹配|退出码不符|没有匹配|CMake Error')
            summary+=("  变异 $name：已发现（$n 项检查失败）")
        fi
    done <<'LIST'
图案不按帧推进|engine/simulation/src/lidar3d.cpp|test_lidar3d|simulation.lidar3d
运动畸变不回推|engine/simulation/src/lidar3d.cpp|test_lidar3d|simulation.lidar3d
点的时间偏移为 0|engine/simulation/src/lidar3d.cpp|test_lidar3d|simulation.lidar3d
距离噪声缺失|engine/simulation/src/sensor_noise.cpp|test_lidar3d|simulation.lidar3d
记录的点云摘要错|engine/simulation/src/record.cpp|terra_headless|headless.lidar3d
内置 IMU 不按预设偏移|engine/scene/src/scene_file.cpp|test_lidar3d|simulation.lidar3d
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
[ $mutation_only = 1 ] && { progress_finish; echo; printf '%s\n' "${summary[@]}"; exit $failed; }

section 7 "输出"
probe="$ros_build"
[ -d "$probe" ] || probe="build/render-release"
if [ -d "$probe" ]; then
    ctest --test-dir "$probe" -R '^(simulation\.lidar3d|headless\.lidar3d|ros2\.lidar3d|graphics\.viewer_lidar3d_selftest)$' -V 2>/dev/null |
        grep -E "INFO|PASS|FAILED|SKIP|all passed|自检通过|自检失败" | sed 's/^[0-9]*: /  /' | cut -c1-260
fi
hl="$probe/apps/headless/terra_headless"

section 8 "组件命令"
hl="$probe/apps/headless/terra_headless"
if [ -x "$hl" ]; then
    "$hl" --list-components --library "$policy_library" | cut -c1-260 | sed 's/^/  /'
fi

viewer="build/render-release/apps/viewer/terra_viewer"
go2_policy=$(ls others/supermimic/go2_supermimic/*/policy.onnx 2>/dev/null | head -1)
section 9 "可交互展示"
if [ $skip_show = 1 ]; then
    note "可交互展示" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "可交互展示" "未执行：不在桌面会话中"
elif [ -x "$viewer" ]; then
    mkdir -p "$out/show"
    policy_args=()
    [ -n "$go2_policy" ] && policy_args=(--policy "$go2_policy")
    echo "（即将打开 terra_viewer，最长 300 秒；设置在 $out/show/，资源库与组件在 build/policy_library/，不读写你的用户目录）"
    echo "  - 场景：宇树 Go2 + Livox Mid-360$([ -n "$go2_policy" ] && echo "，你的行走策略直接运行")"
    echo "  - 视口中叠加的彩色点是 Mid-360 的点云（按高度着色）；传感器面板的“3D 激光雷达”有俯视与侧视图，可以关掉视口叠加"
    echo "  - 速度指令面板或键盘驾驶（I/K、J/L、U/O）让 Go2 走动：看点云随机身移动（每帧 20,000 点，非重复扫描，4 s 循环）"
    echo "  - 传感器面板的“噪声”开关：关掉后点云变为真值；IMU 列表中多了 go2/mid360_imu（Mid-360 内置的 IMU，200 Hz）"
    echo "  先打开 20 秒俯视：Mid-360 水平装在 0.4 m 高、只有地面（看盲区形状：官方图案的下边缘随方位变化，前后约 3.94 m、左右约 3.16 m）"
    "$viewer" --scene assets/tests/sensors/lidar3d_ground.terra-scene.json --seconds 20 \
        --settings "$out/show/settings_ground.json" --library "$policy_library" >"$logs/show_ground.txt" 2>&1
    echo "  地面圆环展示 退出码 $?"
    "$viewer" --scene assets/scenes/robots/go2_mid360.terra-scene.json "${policy_args[@]}" --seconds 300 \
        --settings "$out/show/settings.json" --library "$policy_library" >"$logs/show.txt" 2>&1
    rc=$?
    grep -E "^INFO 场景 |^INFO 策略|^INFO 场景：传感器" "$logs/show.txt" | head -8 | sed 's/^/  /'
    record "可交互展示" "退出码 $rc" "$(ok $rc)"
else
    record "可交互展示" "未执行（构建失败）" 0
fi

progress_finish
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
