#!/usr/bin/env bash
# M4 阶段验收（M4-11）：一条命令复现 M4 的全部自动化证据，沿用 M0-09、M1-11、M2-09、M3-08 的做法。
#
# 请在**桌面会话终端**运行（第 10、11 步使用真实 GPU 并打开可见窗口）；自动化图形测试自己使用 Xvfb + llvmpipe（D-022）。
# 需要 ROS2 Humble 与宇树消息包（与 M4-06b 起的脚本相同）；ONNX Runtime 沿用 build/policy_library/ 中已装的组件。
# 依次执行（日志在 build/verify-m4/logs/，屏幕只打印结论）：
#   1. 提交与工作区状态
#   2. 环境清单：工具链与图形环境
#   3. 变异锚点预检：scripts/verify/check_mutation_anchors.py（全部阶段的脚本；锚点失效时提前失败）
#   4. M0 回归：scripts/verify/m0_acceptance.sh
#   5. M4 全量：scripts/verify/m4_10b.sh --skip-show——依赖、素材与参考机器人校验；渲染关闭 Debug（109 项）、
#      渲染开启 Debug 与 Release + -Werror（149 项）；ASan/UBSan（118 + 31）；含 ROS2 的构建（164 项）与 ASan（15 项）；
#      M4-10b 变异 6 个
#   6. 历次变异（--mutation-only）：M1 的 11 个脚本、M2 的 6 个、M3 的 11 个、M4-02～M4-10a 的 13 个
#   7. 重复运行：渲染开启 Release 全量 CTest 与含 ROS2 构建的 ros2 标签测试，每项至多 3 次
#   8. 依赖边界：Headless 不链接图形库（含 ROS2 构建）；没有任何程序在构建时链接 ONNX Runtime；渲染基准不链接 MuJoCo；
#      渲染侧不引用物理；场景、仿真、资源库、策略模块不包含 GL / 窗口 / 界面 / MuJoCo / ROS2 头文件；
#      查看器与 Headless 的策略运行不在用户目录建库
#   9. M4 完成标准：关键测试在第 7 步中通过；禁用图像与 ROS2 后纯物理可运行；图像采集需要图形上下文、Headless 不需要；
#      传感器、雷达与 Go2 策略在 Headless 中两次运行逐位相同，记录复现逐位相同
#  10. 真实 GPU：查看器的默认、传感器、策略、导入向导自检；terra_capture 两次逐像素相同；Go2 策略 10 秒的实时性（只记录）
#  11. 可交互展示（最长 600 秒）：M4 总览——传感器、噪声与分割、ROS2 面板、记录、导入、资源库卡片、策略
# 最后打印汇总；第 1–10 步任一必需项失败时退出码非 0。
# 用法：bash scripts/verify/m4_acceptance.sh [--jobs N] [--skip-m0] [--skip-show]
#   --skip-m0    跳过第 4 步（只在排查问题时使用；正式验收不跳过）
#   --skip-show  跳过第 10、11 步（不在桌面会话时自动跳过并注明）
set -uo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
cd "$root"

jobs=$(nproc)
skip_m0=0
skip_show=0
while [ $# -gt 0 ]; do
    case "$1" in
        --jobs) jobs=$2; shift 2 ;;
        --skip-m0) skip_m0=1; shift ;;
        --skip-show) skip_show=1; shift ;;
        -h|--help) sed -n '2,32p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/verify-m4"
logs="$out/logs"
rm -rf "$out"
mkdir -p "$logs"
started=$(date +%s)

summary=()
failed=0
record() {  # record <名称> <结果文字> <是否通过 0/1>
    summary+=("$(printf '%-28s %s' "$1" "$2")")
    [ "$3" = 1 ] || failed=1
    echo "${summary[-1]}"
}
note() { summary+=("$(printf '%-28s %s' "$1" "$2")"); echo "${summary[-1]}"; }
ok() { [ "$1" = 0 ] && echo 1 || echo 0; }
desktop() { [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; }

echo "== 1. 提交"
if git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    {
        echo "commit: $(git rev-parse HEAD)"
        echo "branch: $(git rev-parse --abbrev-ref HEAD)"
        echo "未提交的改动："
        git status --porcelain
    } >"$logs/git.txt"
    dirty=$(git status --porcelain | wc -l)
    record "提交" "$(git rev-parse --short HEAD)，未提交改动 $dirty 个文件" 1
    [ "$dirty" = 0 ] || echo "  注意：正式验收应在干净提交上运行（未提交的文件见 $logs/git.txt）"
else
    echo "不是 git 仓库" >"$logs/git.txt"
    record "提交" "不是 git 仓库，无法关联提交" 1
fi

echo "== 2. 环境清单"
if bash scripts/env_inventory.sh >"$logs/env_inventory.txt" 2>&1; then
    sed -n '/## 工具链/,/## 显示与图形/p' "$logs/env_inventory.txt" | grep -v "^## 显示"
    record "环境清单" "已采集（$logs/env_inventory.txt）" 1
else
    record "环境清单" "脚本执行失败" 0
fi
if desktop; then
    bash scripts/gfx_inventory.sh >"$logs/gfx_inventory.txt" 2>&1
    grep -E "OpenGL (renderer|core profile version|version) string|^NVIDIA-SMI|Driver Version|lspci|VGA" "$logs/gfx_inventory.txt" | head -8
    note "图形环境清单" "已采集（$logs/gfx_inventory.txt）"
else
    note "图形环境清单" "未采集：不在桌面会话中"
fi

echo "== 3. 变异锚点预检"
python3 -I scripts/verify/check_mutation_anchors.py >"$logs/anchors.txt" 2>&1
rc=$?
grep -E "^不唯一" "$logs/anchors.txt"
record "变异锚点" "$(tail -1 "$logs/anchors.txt")" "$(ok $rc)"
if [ $rc != 0 ]; then
    echo "锚点失效：变异测试无法注入。请把 $logs/anchors.txt 发给开发者，先修正脚本再验收。" >&2
    printf '%s\n' "${summary[@]}"
    exit 1
fi

echo "== 4. M0 回归（m0_acceptance.sh）"
if [ $skip_m0 = 1 ]; then
    note "M0 回归" "已跳过（--skip-m0，正式验收不可跳过）"
    failed=1
else
    bash scripts/verify/m0_acceptance.sh --jobs "$jobs" >"$logs/m0_acceptance.txt" 2>&1
    rc=$?
    sed -n '/^================ M0 验收汇总/,$p' "$logs/m0_acceptance.txt" | grep -vE "^====|^日志目录" | sed 's/^/  /'
    record "M0 回归" "退出码 $rc（详见 build/verify-m0/logs/）" "$(ok $rc)"
fi

echo "== 5. M4 全量（m4_10b.sh --skip-show）"
bash scripts/verify/m4_10b.sh --skip-show --jobs "$jobs" >"$logs/m4_10b.txt" 2>&1
rc=$?
sed -n '/^================ 汇总/,$p' "$logs/m4_10b.txt" | grep -vE "^====|^日志目录|已跳过" | sed 's/^/  /'
record "M4 全量" "退出码 $rc（详见 build/verify-m4-10b/logs/）" "$(ok $rc)"
ros_build="$root/build/verify-m4-10b/ros2"
policy_library="$root/build/policy_library/library.db"

echo "== 6. 历次变异（--mutation-only）"
all_mutations=0
all_detected=0
zero_failed=()
for s in m1_03_04 m1_06 m1_07 m1_12 m1_13 m1_14 m1_08a m1_08b m1_09 m1_10 m1_15 m2_02_03 m2_04 m2_05 m2_06 m2_07 m2_08 \
         m3_02 m3_03 m3_04a m3_04b m3_04c m3_05 m3_06 m3_07a m3_07b m3_07c m3_07d \
         m4_02 m4_03a m4_03b m4_04 m4_06a m4_06b m4_07 m4_08a m4_08b m4_08c m4_08d m4_09 m4_10a; do
    log="$logs/mutation_$s.txt"
    bash "scripts/verify/$s.sh" --mutation-only --jobs "$jobs" >"$log" 2>&1
    rc=$?
    line=$(grep -E "^变异测试 " "$log" | tail -1 | sed 's/^变异测试 *//')
    # “已发现（0 项检查失败）”：测试以非 0 退出但没有输出 FAILED，可能是崩溃或用法错误，需要看日志确认
    while IFS= read -r z; do zero_failed+=("$s：$z"); done < <(grep -E "已发现（0 项检查失败）" "$log" | sed 's/^ *变异 //; s/：已发现.*$//')
    pair=$(echo "$line" | sed -n 's/^\([0-9]*\)\/\([0-9]*\) 被发现.*/\1 \2/p')
    if [ -n "$pair" ]; then
        set -- $pair
        all_detected=$((all_detected + $1))
        all_mutations=$((all_mutations + $2))
    fi
    record "  变异 ${s}" "${line:-无结果（见 $log）}，退出码 $rc" "$([ $rc = 0 ] && [ -n "$pair" ] && echo 1 || echo 0)"
    [ $rc = 0 ] || grep -E "^  变异 .*(未被发现|注入失败|编译失败)" "$log" | sed 's/^/    /'
done
m10b=$(grep -E "^变异测试 " "$logs/m4_10b.txt" | tail -1 | sed -n 's/^变异测试 *\([0-9]*\)\/\([0-9]*\) 被发现.*/\1 \2/p')
if [ -n "$m10b" ]; then
    set -- $m10b
    all_detected=$((all_detected + $1))
    all_mutations=$((all_mutations + $2))
fi
note "M1～M4 变异合计" "$all_detected/$all_mutations 被发现（含第 5 步 M4-10b 的 6 个；另有第 4 步 M0 的 11 个）"
if [ ${#zero_failed[@]} -gt 0 ]; then
    note "需查看的变异" "${#zero_failed[@]} 个以非 0 退出但没有 FAILED 行：${zero_failed[*]}"
fi

# ROS2 环境（第 7、9、11 步的 ROS2 部分在子 shell 中使用）
ros_setup=/opt/ros/humble/setup.bash
unitree=${UNITREE_ROS2:-$HOME/unitree_ros2}
unitree_setup="$unitree/cyclonedds_ws/install/setup.bash"
with_ros() {  # with_ros <命令...>：在 source 过 ROS2 与宇树消息包的子 shell 中运行
    (
        unset RMW_IMPLEMENTATION CYCLONEDDS_URI
        set +u
        # shellcheck disable=SC1090
        source "$ros_setup" && source "$unitree_setup"
        set -u
        unset RMW_IMPLEMENTATION CYCLONEDDS_URI
        export ROS_LOCALHOST_ONLY=1
        "$@"
    )
}

echo "== 7. 重复运行"
ctest --test-dir build/render-release --repeat until-fail:3 >"$logs/repeat.txt" 2>&1
rc=$?
record "渲染开启（每项至多 3 次）" "$(grep -E "tests passed|tests failed" "$logs/repeat.txt" | tail -1)" "$(ok $rc)"
[ $rc = 0 ] || grep -E "^\s+[0-9]+ - " "$logs/repeat.txt" | head -10 | sed 's/^/    /'
if [ -d "$ros_build" ]; then
    with_ros ctest --test-dir "$ros_build" -L ros2 --repeat until-fail:3 >"$logs/repeat_ros2.txt" 2>&1
    rc=$?
    record "ros2 标签（每项至多 3 次）" "$(grep -E "tests passed|tests failed" "$logs/repeat_ros2.txt" | tail -1)" "$(ok $rc)"
    [ $rc = 0 ] || grep -E "^\s+[0-9]+ - " "$logs/repeat_ros2.txt" | head -10 | sed 's/^/    /'
else
    record "ros2 标签（每项至多 3 次）" "含 ROS2 的构建不存在（第 5 步失败？）" 0
fi

echo "== 8. 依赖边界"
bad=0
: >"$logs/ldd.txt"
for bin in build/release/apps/headless/terra_headless build/release/apps/bench/terra_bench "$ros_build/apps/headless/terra_headless"; do
    if [ ! -x "$bin" ]; then echo "$bin 不存在" >>"$logs/ldd.txt"; bad=1; continue; fi
    { echo "== $bin"; with_ros ldd "$bin"; } >>"$logs/ldd.txt" 2>&1
    if with_ros ldd "$bin" | grep -Eq "lib(GL|EGL|OSMesa|X11|vulkan|glfw|wayland)[.-]"; then
        echo "  $bin 链接了图形库"
        bad=1
    fi
done
record "Headless 无图形库" "$([ $bad = 0 ] && echo "terra_headless（含 ROS2 构建）、terra_bench 均无图形库" || echo "存在图形库或程序缺失（见 $logs/ldd.txt）")" \
    "$([ $bad = 0 ] && echo 1 || echo 0)"
bad=0
for bin in build/render-release/apps/headless/terra_headless build/render-release/apps/viewer/terra_viewer build/render-release/apps/capture/terra_capture \
           "$ros_build/apps/viewer/terra_viewer"; do
    [ -x "$bin" ] || { bad=1; continue; }
    if with_ros ldd "$bin" | grep -q "onnxruntime"; then echo "  $bin 链接了 ONNX Runtime"; bad=1; fi
done
record "ONNX Runtime 不在构建时链接" "$([ $bad = 0 ] && echo "Headless、查看器、terra_capture 都不链接（运行时 dlopen 组件）" || echo "有程序链接了 ONNX Runtime 或缺失")" \
    "$([ $bad = 0 ] && echo 1 || echo 0)"
bad=0
for bin in build/render-release/apps/bench_render/terra_render_bench; do
    if [ ! -x "$bin" ]; then echo "$bin 不存在" >>"$logs/ldd.txt"; bad=1; continue; fi
    if ldd "$bin" | grep -q "libmujoco"; then echo "  $bin 链接了 MuJoCo"; bad=1; fi
done
record "渲染程序无 MuJoCo" "$([ $bad = 0 ] && echo "terra_render_bench 不链接 libmujoco" || echo "存在 MuJoCo 依赖或程序缺失")" \
    "$([ $bad = 0 ] && echo 1 || echo 0)"
render_side=(engine/render engine/asset engine/ui engine/view engine/platform engine/input engine/settings apps/bench_render
             apps/viewer/demo_scene.hpp apps/viewer/demo_scene.cpp apps/viewer/scene_view.hpp apps/viewer/scene_view.cpp)
grep -rnE "terra::physics|terra_physics|mujoco|terra/physics/" "${render_side[@]}" >"$logs/render_physics_refs.txt" 2>&1
hits=$(grep -c . "$logs/render_physics_refs.txt")
record "渲染侧不引用物理" "$([ "$hits" = 0 ] && echo "0 处引用" || echo "$hits 处引用（见 $logs/render_physics_refs.txt）")" "$([ "$hits" = 0 ] && echo 1 || echo 0)"
grep -rnE "#include *[<\"](mujoco/|glad/|GL/|GLFW/|imgui|rclcpp/|sensor_msgs/)" engine/scene engine/simulation engine/library engine/policy engine/viz \
    >"$logs/core_includes.txt" 2>&1
hits=$(grep -c . "$logs/core_includes.txt")
record "核心模块边界" "$([ "$hits" = 0 ] && echo "scene、simulation、library、policy、viz 不包含 MuJoCo、GL、GLFW、ImGui、ROS2 头文件" || echo "$hits 处违规（见 $logs/core_includes.txt）")" \
    "$([ "$hits" = 0 ] && echo 1 || echo 0)"
missing=()
for t in boundary.no_mujoco_in_public_headers boundary.render_isolation headless.no_graphics_libs; do
    grep -qE "Test +#[0-9]+: ${t//./\\.} .*Passed" "$logs/repeat.txt" || missing+=("$t")
done
record "边界测试" "$([ ${#missing[@]} = 0 ] && echo "no_mujoco_in_public_headers、render_isolation、no_graphics_libs 在第 7 步中通过" || echo "未找到通过记录：${missing[*]}")" \
    "$([ ${#missing[@]} = 0 ] && echo 1 || echo 0)"
# 不在用户目录建库：数据目录指到构建目录，查看器场景自检、Headless 运行策略（夹具，不需要组件）后不应出现 eai-terra/
xdg="$out/xdg"
mkdir -p "$xdg"
env -u TERRA_LIBRARY -u DISPLAY -u WAYLAND_DISPLAY XDG_DATA_HOME="$xdg" __GLX_VENDOR_LIBRARY_NAME=mesa LIBGL_ALWAYS_SOFTWARE=1 \
    xvfb-run -a -s "-screen 0 1920x1080x24" build/render-release/apps/viewer/terra_viewer --frames 30 --self-test \
    --scene assets/tests/sensors/sensor_suite.terra-scene.json >"$logs/no_library_viewer.txt" 2>&1
rc1=$?
env -u TERRA_LIBRARY XDG_DATA_HOME="$xdg" build/render-release/apps/headless/terra_headless --policy assets/tests/policy/bench.terra-policy.json \
    --steps 100 --sample-every 100 >"$logs/no_library_headless.txt" 2>&1
rc2=$?
if [ $rc1 = 0 ] && [ $rc2 = 0 ] && [ ! -e "$xdg/eai-terra" ]; then
    record "不在用户目录建库" "查看器自检与 Headless 运行策略退出码 0，数据目录中没有 eai-terra/" 1
else
    record "不在用户目录建库" "退出码 $rc1/$rc2；$( [ -e "$xdg/eai-terra" ] && echo "出现了 $(ls "$xdg/eai-terra" | tr '\n' ' ')" || echo "没有建库")" 0
fi

echo "== 9. M4 完成标准"
# 关键测试（开发计划第 5 节 M4 完成标准与 PRD F-M4-01～04、D-111）
missing=()
for t in physics.imu scene.sensors scene.camera_model simulation.sensor_sampling simulation.lidar graphics.camera_capture \
         graphics.capture_determinism capture.no_cameras graphics.viewer_sensor_selftest record.sensors graphics.viewer_sensor_record_selftest \
         import.analyze import.execute import.demos robots.unitree_import robots.turtlebot4_import graphics.viewer_import_selftest \
         library.assets library.pack graphics.viewer_library_selftest viz.config graphics.viewer_viz_selftest \
         simulation.sensor_noise headless.sensors_noise graphics.capture_noise \
         library.components policy.file policy.controller policy.import policy.onnxruntime policy.go2_walk headless.policy \
         graphics.viewer_policy_selftest graphics.viewer_policy_import_selftest; do
    grep -qE "Test +#[0-9]+: ${t//./\\.} .*Passed" "$logs/repeat.txt" || missing+=("$t")
done
skipped=$(grep -cE "\*\*\*Skipped" "$logs/repeat.txt")
record "M4 关键测试" "$([ ${#missing[@]} = 0 ] && echo "35 项在第 7 步中通过（传感器、相机、采集、记录、导入、资源库、显示配置、噪声、组件与策略）；跳过 $skipped 项" || echo "未找到通过记录：${missing[*]}")" \
    "$([ ${#missing[@]} = 0 ] && echo 1 || echo 0)"
missing=()
for t in ros2.profile ros2.bridge_headless ros2.bridge_commands graphics.ros2_bridge_viewer ros2.turtlebot4 ros2.monitor graphics.viewer_ros2_panel_selftest \
         ros2.unitree_crc ros2.unitree_go2 ros2.unitree_g1 ros2.unitree_as2 ros2.unitree_official_go2 ros2.unitree_official_go2w ros2.unitree_official_g1 \
         ros2.policy_cmd_vel; do
    grep -qE "Test +#[0-9]+: ${t//./\\.} .*Passed" "$logs/repeat_ros2.txt" 2>/dev/null || missing+=("$t")
done
record "外部 ROS2 示例闭环" "$([ ${#missing[@]} = 0 ] && echo "15 项在第 7 步中通过（含宇树官方示例 go2_stand_example、g1_low_level_example 闭环与 /cmd_vel）" || echo "未找到通过记录：${missing[*]}")" \
    "$([ ${#missing[@]} = 0 ] && echo 1 || echo 0)"
# 禁用图像与 ROS2：第 5 步中的渲染关闭、不含 ROS2 的 Debug 构建全部通过；它的 Headless 照常运行传感器场景（相机只提示）
debug_line=$(grep -E "^渲染关闭 Debug 测试" "$logs/m4_10b.txt" | tail -1 | sed 's/^渲染关闭 Debug 测试 *//')
hl_debug=build/debug/apps/headless/terra_headless
"$hl_debug" --scene assets/tests/sensors/sensor_suite.terra-scene.json --steps 500 --sensors --sample-every 100 >"$logs/pure_physics.jsonl" 2>"$logs/pure_physics.err"
rc=$?
pass=0
[ $rc = 0 ] && grep -q '"type":"lidar"' "$logs/pure_physics.jsonl" && grep -q '"type":"sensor"' "$logs/pure_physics.jsonl" && grep -q "相机需要图形上下文" "$logs/pure_physics.err" && \
    [[ "$debug_line" == *"100% tests passed"* ]] && pass=1
record "禁用图像与 ROS2 后纯物理" "渲染关闭 Debug：${debug_line:-无结果}；其 Headless 运行传感器场景退出码 $rc（IMU、雷达照常输出，相机只提示）" "$pass"
# 图像采集需要图形上下文，Headless 不需要（不把“图像 Headless”与“无 GPU 的物理”混为一谈）
# 2026-11-25 首次验收运行：GLFW 是静态链接的，运行时才 dlopen libGL / libX11，ldd 看不到图形库，所以判据改为“没有显示时
# 无法创建图形上下文而退出”，并同时确认同一程序在 Xvfb 下可以运行（与 graphics.capture_determinism 相同的环境）
cap=build/render-release/apps/capture/terra_capture
env -u DISPLAY -u WAYLAND_DISPLAY "$cap" --scene assets/tests/sensors/sensor_suite.terra-scene.json --steps 10 --output "$out/capture_nodisplay" \
    >"$logs/capture_nodisplay.txt" 2>&1
rc=$?
env -u DISPLAY -u WAYLAND_DISPLAY __GLX_VENDOR_LIBRARY_NAME=mesa LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a -s "-screen 0 1920x1080x24" \
    "$cap" --scene assets/tests/sensors/sensor_suite.terra-scene.json --steps 10 --output "$out/capture_xvfb" >"$logs/capture_xvfb.txt" 2>&1
rc_x=$?
why=$(grep -m1 -iE "窗口|显示|display|GLFW|OpenGL|上下文" "$logs/capture_nodisplay.txt" | cut -c1-120)
record "图像采集与 Headless 区分" "terra_capture 没有显示时退出码 $rc（${why:-见 $logs/capture_nodisplay.txt}），Xvfb 下退出码 $rc_x；Headless 无图形库（第 8 步）" \
    "$([ $rc != 0 ] && [ $rc_x = 0 ] && [ -n "$why" ] && echo 1 || echo 0)"
# 可复现：传感器（IMU、雷达，带噪声）两次输出逐位相同；Go2 策略两次逐位相同、记录复现逐位相同
headless=build/render-release/apps/headless/terra_headless
for i in 1 2; do
    "$headless" --scene assets/tests/sensors/noise_suite.terra-scene.json --steps 1000 --sensors --sample-every 50 >"$logs/sensors_$i.jsonl" 2>/dev/null
done
h1=$(sha256sum <"$logs/sensors_1.jsonl" | cut -c1-16); h2=$(sha256sum <"$logs/sensors_2.jsonl" | cut -c1-16)
record "传感器可复现" "噪声夹具 1000 步两次输出 $([ "$h1" = "$h2" ] && echo 相同 || echo 不同)（$(wc -l <"$logs/sensors_1.jsonl") 行，sha256 $h1…）" "$([ "$h1" = "$h2" ] && echo 1 || echo 0)"
go2_policy=$(ls others/supermimic/go2_supermimic/*/policy.onnx 2>/dev/null | head -1)
if [ -n "$go2_policy" ]; then
    go2_scene=assets/scenes/robots/go2_policy.terra-scene.json
    for i in 1 2; do
        "$headless" --scene "$go2_scene" --policy "$go2_policy" --command 0.5,0,0 --duration 5 --sample-every 50 --library "$policy_library" \
            >"$logs/go2_$i.jsonl" 2>"$logs/go2_$i.err"
    done
    g1=$(sha256sum <"$logs/go2_1.jsonl" | cut -c1-16); g2=$(sha256sum <"$logs/go2_2.jsonl" | cut -c1-16)
    rec="$out/records/go2_policy.terra-record.jsonl"
    rm -f "$rec"
    "$headless" --scene "$go2_scene" --policy "$go2_policy" --command 0.5,0,0 --duration 5 --sample-every 50 --library "$policy_library" \
        --record "$rec" >/dev/null 2>"$logs/go2_record.err"
    "$headless" --rerun "$rec" >"$logs/go2_rerun.jsonl" 2>"$logs/go2_rerun.err"
    bitwise=$(grep -o '"bitwise":[a-z]*' "$logs/go2_rerun.jsonl" | cut -d: -f2)
    compared=$(grep -o '"compared":[0-9]*' "$logs/go2_rerun.jsonl" | cut -d: -f2)
    record "Go2 策略可复现" "5 s 前进两次输出 $([ "$g1" = "$g2" ] && echo 相同 || echo 不同)（sha256 $g1…）；复现 ${compared:-?} 个采样点 bitwise=${bitwise:-?}（复现不需要推理库）" \
        "$([ "$g1" = "$g2" ] && [ "$bitwise" = true ] && echo 1 || echo 0)"
else
    record "Go2 策略可复现" "没有找到 others/supermimic/go2_supermimic/*/policy.onnx" 0
fi

viewer=build/render-release/apps/viewer/terra_viewer
echo "== 10. 真实 GPU（桌面会话）"
if [ $skip_show = 1 ]; then
    note "真实 GPU 自检" "已跳过（--skip-show）"
elif ! desktop; then
    note "真实 GPU 自检" "未执行：不在桌面会话中"
elif [ -x "$viewer" ]; then
    "$viewer" --frames 40 --self-test --settings "$out/selftest/settings.json" >"$logs/desktop_selftest.txt" 2>&1
    rc=$?
    grep -E "^INFO NVIDIA|^INFO .*OpenGL|^FAILED" "$logs/desktop_selftest.txt" | head -4
    record "真实 GPU 自检" "退出码 $rc" "$(ok $rc)"
    "$viewer" --frames 60 --self-test --scene assets/tests/sensors/sensor_suite.terra-scene.json --sensor-self-test >"$logs/desktop_sensor.txt" 2>&1
    rc=$?
    grep -E "^FAILED" "$logs/desktop_sensor.txt"
    record "真实 GPU 传感器自检" "退出码 $rc" "$(ok $rc)"
    "$viewer" --frames 80 --self-test --scene assets/tests/policy/policy_bench.terra-scene.json --policy-self-test \
        --policy assets/tests/policy/bench.terra-policy.json --library "$out/gpu_policy/library.sqlite3" >"$logs/desktop_policy.txt" 2>&1
    rc=$?
    grep -E "^FAILED" "$logs/desktop_policy.txt"
    record "真实 GPU 策略自检" "退出码 $rc" "$(ok $rc)"
    "$viewer" --frames 120 --self-test --policy-import-self-test assets/tests/policy/bench_supermimic.onnx \
        --library "$out/gpu_import/library.sqlite3" >"$logs/desktop_policy_import.txt" 2>&1
    rc=$?
    grep -E "^FAILED|未选文件" "$logs/desktop_policy_import.txt"
    record "真实 GPU 导入向导自检" "退出码 $rc（含“菜单重开、未选文件时不能导入”）" "$(ok $rc)"
    for i in 1 2; do
        rm -rf "$out/gpu_capture_$i"
        "$cap" --scene assets/tests/sensors/sensor_suite.terra-scene.json --steps 300 --output "$out/gpu_capture_$i" >"$logs/gpu_capture_$i.txt" 2>&1
    done
    if diff -rq "$out/gpu_capture_1" "$out/gpu_capture_2" >"$logs/gpu_capture_diff.txt" 2>&1; then
        record "真实 GPU 采集可复现" "terra_capture 两次输出逐字节相同（$(find "$out/gpu_capture_1" -type f | wc -l) 个文件）" 1
    else
        record "真实 GPU 采集可复现" "两次输出不同（见 $logs/gpu_capture_diff.txt）" 0
    fi
    if [ -n "${go2_policy:-}" ]; then
        "$viewer" --scene assets/scenes/robots/go2_policy.terra-scene.json --policy "$go2_policy" --policy-command 0.5,0,0 --seconds 10 \
            --settings "$out/realtime_go2/settings.json" --library "$policy_library" >"$logs/realtime_go2.txt" 2>&1
        rc=$?
        line=$(grep -E "^INFO 仿真：" "$logs/realtime_go2.txt" | tail -1 | sed 's/^INFO //')
        note "实时性 Go2 策略（10 秒）" "${line:-无结果}，退出码 $rc"
    fi
else
    record "真实 GPU 自检" "未执行（render-release 构建失败）" 0
fi

echo "== 11. 可交互展示"
ros_viewer="$ros_build/apps/viewer/terra_viewer"
if [ $skip_show = 1 ]; then
    note "可交互展示" "已跳过（--skip-show）"
elif ! desktop; then
    note "可交互展示" "未执行：不在桌面会话中"
elif [ -x "$ros_viewer" ]; then
    echo "（即将打开含 ROS2 的 terra_viewer，最长 600 秒；设置在 $out/show/（全新），资源库、组件与导入的策略在 build/policy_library/）"
    echo "  M4 总览，请逐项看一眼："
    echo "  - 欢迎页与左侧资源库：单击 TurtleBot 4 卡片；传感器面板看 IMU 曲线、雷达、相机彩色 / 深度 / 分割图，开关“噪声”"
    echo "  - 仿真 → ROS2 桥接；窗口 → ROS2 面板：话题、消息、图像、TF 树；显示配置可保存与导出 .rviz"
    echo "  - 工具栏录制几秒再停止，在时间轴中回放：传感器面板显示记录中的读数"
    echo "  - 文件 → 导入模型…（或拖入 URDF）：导入向导"
    echo "  - 资源库的“策略”分页：单击 Go2 策略卡片即运行；速度指令面板的滑块或键盘驾驶（I/K、J/L、U/O）；策略 → 组件…"
    with_ros "$ros_viewer" --seconds 600 --settings "$out/show/settings.json" --library "$policy_library" >"$logs/show.txt" 2>&1
    rc=$?
    grep -E "^INFO 场景 |^INFO 策略|^INFO ROS2" "$logs/show.txt" | head -12 | sed 's/^/  /'
    record "可交互展示" "退出码 $rc" "$(ok $rc)"
else
    record "可交互展示" "未执行（含 ROS2 的构建失败）" 0
fi

{
    echo "### CPU"
    grep -m1 "model name" /proc/cpuinfo
    echo "cores: $(nproc)"
    grep MemTotal /proc/meminfo
    echo "governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo 未知)"
} >"$logs/machine.txt"

elapsed=$(( $(date +%s) - started ))
echo
echo "================ M4 验收汇总 ================"
printf '%s\n' "${summary[@]}"
echo "用时：$((elapsed / 60)) 分 $((elapsed % 60)) 秒"
echo "日志目录：$logs"
if [ $failed = 0 ]; then
    echo "结论：全部通过"
else
    echo "结论：存在失败项"
fi
exit $failed
