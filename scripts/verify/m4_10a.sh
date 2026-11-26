#!/usr/bin/env bash
# M4-10a 基线机器验证：推理组件与策略运行（M4-10 设计稿修订版，用户 2026-11-25：.pt 与 .onnx 都要支持，推理先用 ONNX（CPU）；
# 客户端提示并代为下载需要的库文件，带进度条，没有网络时警告；用户只选策略与机器人即可自动配置；观测取带噪声的读数）——
# 组件管理（清单、下载、sha256、安装、校验、删除）；ONNX Runtime 1.32.0（NuGet，运行时 dlopen）；策略说明文件、ONNX 信息、
# 推理后端注册表、自动配置（ONNX 元数据 + 训练配置）、策略控制器（降频、历史、相位、positionPd）；Headless 的 --policy、
# --command、--install-component、--list-components；查看器的“运行策略…”、组件窗口与速度指令面板。
#
# 请在**桌面会话终端**运行（第 9 步打开查看器）。需要网络：第一次运行时下载 ONNX Runtime（184 MB，NuGet）到
# build/policy_library/components/（之后沿用，不重复下载）。离线时可先在别处下载 nupkg，用 TERRA_ORT_NUPKG=<文件> 指定。
# 用户的 Go2 策略（others/supermimic/…/policy.onnx）只读。
# 依次执行（日志在 build/verify-m4-10a/logs/，屏幕只打印结论）：
#   1. 依赖与素材校验；ROS2 与宇树消息包检查
#   2. 不含 ROS2 的构建：渲染关闭 Debug（预期 108/108）；渲染开启 Debug 与 Release + -Werror（预期 147/147）。
#      第一个构建完成后用它的 terra_headless --install-component 安装 ONNX Runtime（已安装时跳过）
#   3. ASan/UBSan（不含 ROS2；ONNX Runtime 的两个测试在 ASan 构建中跳过：组件只装在 build/policy_library）
#   4. 含 ROS2 与宇树消息包的构建（Release + -Werror）：全量 CTest（预期 161/161）
#   5. ASan/UBSan（含 ROS2）：ros2 标签的测试（预期 14/14）
#   6. 变异测试（不含 ROS2 的渲染构建）：逐个注入 9 个已知错误
#   7. 输出：自动配置的结果、Go2 策略的站立 / 行走 / 转向指标、推理耗时
#   8. 组件命令：--list-components
#   9. 可交互展示（最长 300 秒）：查看器打开 Go2 策略场景并运行你的策略，用速度指令面板或键盘驾驶
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m4_10a.sh [--jobs N] [--skip-asan] [--skip-mutation] [--skip-show] [--mutation-only]
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

out="$root/build/verify-m4-10a"
# ONNX Runtime 组件装在验证目录之外，重复运行时沿用（不重复下载 184 MB）
policy_library="$root/build/policy_library/library.db"
policy_flags=("-DTERRA_POLICY_LIBRARY=$policy_library")
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
    "$hl" --install-component onnxruntime-cpu --library "$policy_library" "${from[@]}" >"$logs/install_ort.txt" 2>&1
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
    if cmake --preset "$preset" "${policy_flags[@]}" "$@" >"$log" 2>&1 && cmake --build --preset "$preset" -j "$jobs" >>"$log" 2>&1; then
        ensure_ort "build/$preset/apps/headless/terra_headless"
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
        record "ASan 非图形（含泄漏检测）" "$(ctest_result "$logs/asan_nongraphics.txt")；跳过 $(grep -cE '\*\*\*Skipped' "$logs/asan_nongraphics.txt") 个（预期 2：policy.onnxruntime、policy.go2_walk）" "$(ok $rc1)"
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
             "${ros_flags[@]}" "${policy_flags[@]}" -DTERRA_WARNINGS_AS_ERRORS=ON >"$log" 2>&1 &&
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
        [ "$skipped" = 0 ] || record "跳过的测试" "$skipped 个（见 $logs/ros2_ctest.txt；官方示例未编译、ONNX Runtime 没有安装或没有用户策略时跳过）" 0
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
    files=(engine/policy/src/controller.cpp engine/policy/src/autoconfig.cpp apps/headless/main.cpp engine/library/src/components.cpp)
    for f in "${files[@]}"; do cp "$f" "$out/${f//\//_}.orig"; done
    restore() { for f in "${files[@]}"; do cp "$out/${f//\//_}.orig" "$f"; done; }
    trap 'restore; echo "已恢复被变异的源文件"; exit 130' INT TERM
    # 不开 -Werror：变异只改语义，不应因警告而无法判定
    cmake -S . -B "$b" -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug -DTERRA_BUILD_RENDER=ON "${policy_flags[@]}" >"$log" 2>&1
    detected=0
    total=0
    while IFS='|' read -r name file target testname; do
        total=$((total + 1))
        restore
        if ! python3 - "$file" "$name" >>"$log" 2>&1 <<'PY'
import sys
path, name = sys.argv[1], sys.argv[2]
mutations = {
    "历史帧最新的在前": (
        "            std::move(history_.begin() + static_cast<std::ptrdiff_t>(width), history_.end(), history_.begin());\n"
        "            std::copy(frame_.begin(), frame_.end(), history_.end() - static_cast<std::ptrdiff_t>(width));",
        "            std::move_backward(history_.begin(), history_.end() - static_cast<std::ptrdiff_t>(width), history_.end());\n"
        "            std::copy(frame_.begin(), frame_.end(), history_.begin());"),
    "站立时相位不为 0": (
        "                if (magnitude > t.spec.standingThreshold && phaseSteps_ > 0) {",
        "                if (phaseSteps_ > 0) {"),
    "力矩不截断": (
        "        if (c.torqueLimit > 0) tau = std::clamp(tau, -c.torqueLimit, c.torqueLimit);",
        "        (void)c.torqueLimit;"),
    "上一次动作取缩放后的目标": (
        "                for (float a : output_) v.push_back(static_cast<double>(a));",
        "                for (double a : targets_) v.push_back(a);"),
    "IMU 不按频率保持": (
        "        if (heldValid_[k] && !terra::simulation::sensorSampleDue(snapshot.stepIndex, timestep_, sensorRates_[k])) continue;",
        "        (void)sensorRates_;"),
    "观测不加噪声": (
        "        noise_.imu(k, snapshot.stepIndex, held_[k]);  // M4-09：与其他出口逐位相同的带噪声读数",
        "        (void)noise_;"),
    "自动配置 kp 与 kd 对调": (
        "            p.action.kp.push_back(*kp);\n            p.action.kd.push_back(*kd);",
        "            p.action.kp.push_back(*kd);\n            p.action.kd.push_back(*kp);"),
    "--command 不生效": (
        "        if (options.command) policy->setCommand(*options.command);",
        "        (void)options.command;"),
    "组件不校验文件的 sha256": (
        "            if (terra::sha256Hex(content) != f.sha256) {",
        "            if (content.empty()) {"),
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
历史帧最新的在前|engine/policy/src/controller.cpp|test_policy_controller|policy.controller
站立时相位不为 0|engine/policy/src/controller.cpp|test_policy_controller|policy.controller
力矩不截断|engine/policy/src/controller.cpp|test_policy_controller|policy.controller
上一次动作取缩放后的目标|engine/policy/src/controller.cpp|test_policy_controller|policy.controller
IMU 不按频率保持|engine/policy/src/controller.cpp|test_policy_controller|policy.controller
观测不加噪声|engine/policy/src/controller.cpp|test_policy_controller|policy.controller
自动配置 kp 与 kd 对调|engine/policy/src/autoconfig.cpp|test_policy_controller|policy.controller
--command 不生效|apps/headless/main.cpp|terra_headless|headless.policy
组件不校验文件的 sha256|engine/library/src/components.cpp|test_components|library.components
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
probe="$ros_build"
[ -d "$probe" ] || probe="build/render-release"
if [ -d "$probe" ]; then
    ctest --test-dir "$probe" -R '^(policy\.(onnxruntime|go2_walk)|headless\.policy)$' -V 2>/dev/null |
        grep -E "格式|自动配置|警告|后端|站立|前进|转向|推理耗时|  ok  |FAILED|SKIP|all passed|PASS" | sed 's/^[0-9]*: /  /' | cut -c1-300
    metrics=$(find "$probe" -name policy_go2_metrics.json | head -1)
    [ -n "$metrics" ] && echo "  INFO Go2 指标：$(cat "$metrics")"
fi

echo "== 8. 组件命令"
hl="$probe/apps/headless/terra_headless"
if [ -x "$hl" ]; then
    "$hl" --list-components --library "$policy_library" | cut -c1-260 | sed 's/^/  /'
fi

viewer="build/render-release/apps/viewer/terra_viewer"
go2_policy=$(ls others/supermimic/go2_supermimic/*/policy.onnx 2>/dev/null | head -1)
echo "== 9. 可交互展示"
if [ $skip_show = 1 ]; then
    note "可交互展示" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "可交互展示" "未执行：不在桌面会话中"
elif [ -z "$go2_policy" ]; then
    note "可交互展示" "未执行：没有找到 others/supermimic/go2_supermimic/*/policy.onnx"
elif [ -x "$viewer" ]; then
    mkdir -p "$out/show"
    echo "（即将打开 terra_viewer，最长 300 秒；关闭窗口即结束。设置在 $out/show/，资源库与组件在 build/policy_library/，不读写你的用户目录）"
    echo "  - 场景：宇树 Go2 策略运行；你的策略 $go2_policy 由 ONNX 元数据与 task_config.json 自动配置后直接运行"
    echo "  - 视口左上角的“速度指令”面板：拖动滑块，或勾选“键盘驾驶”后按住 I/K（前后）、J/L（平移）、U/O（转向），松开即停"
    echo "  - 菜单 仿真 → 停止策略 / 运行策略…（可以重新选择策略文件，窗口列出自动配置的每一项）"
    echo "  - 菜单 设置 → 组件…：ONNX Runtime 已安装，可以重新校验；传感器面板的“噪声”开关同样作用于策略的观测"
    "$viewer" --scene assets/scenes/robots/go2_policy.terra-scene.json --policy "$go2_policy" --seconds 300 \
        --settings "$out/show/settings.json" --library "$policy_library" >"$logs/show.txt" 2>&1
    rc=$?
    grep -E "^INFO 场景 |^INFO 策略" "$logs/show.txt" | head -8 | sed 's/^/  /'
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
