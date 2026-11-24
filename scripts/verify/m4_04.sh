#!/usr/bin/env bash
# M4-04 基线机器验证：传感器面板与离线采集（M4-01 设计稿 2026-11-24 评审通过）——
# 查看器“传感器”面板（IMU 曲线、雷达俯视图、相机彩色与深度，悬停显示深度）：按各传感器的频率在对应的那一步采样，
# 与 Headless、terra_capture 的采样规则相同；terra_capture 把读数与图像写入目录，两次运行逐字节相同；
# Stretch 3 场景加上头部相机（D435i）与底盘雷达。
#
# 请在**桌面会话终端**运行（第 7 步打开查看器）。
# 依次执行（日志在 build/verify-m4-04/logs/，屏幕只打印结论）：
#   1. 依赖与素材校验
#   2. 渲染关闭 Debug：全量 CTest（预期 82/82；scene.camera_model 扩充：显示旋转）
#   3. 渲染开启 Debug 与 Release + -Werror：全量 CTest（预期 115/115，新增 graphics.viewer_sensor_selftest、
#      graphics.capture_determinism、capture.no_cameras）
#   4. ASan/UBSan（渲染开启）
#   5. 变异测试：不采初始状态、雷达每步都扫、相机不按频率、深度预览纹理、面板停靠、采集漏第 0 步、采集的相机不按频率、
#      忽略 --no-cameras、显示旋转方向，逐个注入 9 个已知错误
#   6. 输出：terra_capture 采集 Stretch 3 巡游动作 3 秒（真实 GPU）：文件数、深度图中心、雷达俯视图
#   7. 可交互展示（最长 300 秒）：查看器播放 Stretch 3 巡游，传感器面板显示 IMU、雷达与头部相机
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m4_04.sh [--jobs N] [--skip-asan] [--skip-mutation] [--skip-show] [--mutation-only]
# --mutation-only：只做变异测试（阶段验收调用；不删除、不重建 build/ 下的预设构建目录，跳过 ASan 与展示）
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
        -h|--help) sed -n '2,21p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/verify-m4-04"
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

echo "== 1. 依赖"
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

echo "== 2. 渲染关闭"
run_preset debug "渲染关闭 Debug"

echo "== 3. 渲染开启"
run_preset render-debug "渲染 Debug"
run_preset render-release "渲染 Release -Werror" -DTERRA_WARNINGS_AS_ERRORS=ON

if [ $skip_asan = 0 ]; then
    echo "== 4. ASan/UBSan（渲染开启）"
    log="$logs/asan.txt"
    b="$out/asan"
    san="-fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer"
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

if [ $skip_mutation = 0 ]; then
    echo "== 5. 变异测试"
    log="$logs/mutation.txt"
    b="$out/mutation"
    files=(apps/viewer/main.cpp apps/capture/main.cpp engine/scene/src/camera_model.cpp)
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
    "打开与重置后不采初始状态": (
        "        observe(*instance.world->snapshot());",
        "        (void)0;"),
    "雷达每步都扫": (
        "            if (!terra::simulation::sensorSampleDue(snapshot.stepIndex, dt, instance.lidars[k].rateHz)) continue;",
        "            if (false) continue;"),
    "相机不按频率采样": (
        "            if (!terra::simulation::sensorSampleDue(snapshot.stepIndex, dt, instance.cameras[k].rateHz)) continue;",
        "            if (false) continue;"),
    "深度预览纹理没有创建": (
        "            preview.depth = terra::render::Texture2D::create(image.width, image.height, image.pixels.data(), false);",
        "            (void)image;"),
    "传感器面板不在执行器所在的组": (
        "    ImGui::DockBuilderDockWindow(\"###Sensors\", right);",
        "    ImGui::DockBuilderDockWindow(\"###Sensors\", center);"),
    "采集漏掉第 0 步": (
        "    sample();\n    for (std::uint64_t i = 0; i < total && written; ++i) {",
        "    for (std::uint64_t i = 0; i < total && written; ++i) {"),
    "采集的相机不按频率": (
        "            if (!sim::sensorSampleDue(step, dt, c.rateHz)) continue;",
        "            if (false) continue;"),
    "显示旋转方向反了": (
        "        case 90: return {1.0 - dy, dx};   // 逆时针 90°：原图的右边成为上边",
        "        case 90: return {dy, 1.0 - dx};"),
    "采集忽略 --no-cameras": (
        "    const bool cameras = !scene.cameras.empty() && !options.noCameras;",
        "    const bool cameras = !scene.cameras.empty();"),
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
打开与重置后不采初始状态|apps/viewer/main.cpp|terra_viewer|graphics.viewer_sensor_selftest
雷达每步都扫|apps/viewer/main.cpp|terra_viewer|graphics.viewer_sensor_selftest
相机不按频率采样|apps/viewer/main.cpp|terra_viewer|graphics.viewer_sensor_selftest
深度预览纹理没有创建|apps/viewer/main.cpp|terra_viewer|graphics.viewer_sensor_selftest
传感器面板不在执行器所在的组|apps/viewer/main.cpp|terra_viewer|graphics.viewer_sensor_selftest
采集漏掉第 0 步|apps/capture/main.cpp|terra_capture|graphics.capture_determinism
采集的相机不按频率|apps/capture/main.cpp|terra_capture|graphics.capture_determinism
采集忽略 --no-cameras|apps/capture/main.cpp|terra_capture|capture.no_cameras
显示旋转方向反了|engine/scene/src/camera_model.cpp|test_camera_model|scene.camera_model
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

echo "== 6. 输出"
ctest --test-dir build/render-release -R '^graphics\.(viewer_sensor_selftest|capture_determinism)$' -V 2>/dev/null |
    grep -E "INFO|PASS|FAILED|逐字节|IMU .* 行" | sed 's/^[0-9]*: /  /'
capture=build/render-release/apps/capture/terra_capture
if [ -x "$capture" ] && [ -f assets/robots/hello_robot_stretch_3/stretch.xml ]; then
    echo "  terra_capture 采集 Stretch 3 巡游动作 3 秒："
    "$capture" --motion assets/motions/stretch3/tour.terra-motion.json --duration 3 --output "$out/stretch3" --force \
        >"$logs/capture.txt" 2>&1
    rc=$?
    grep -E "^INFO" "$logs/capture.txt" | sed 's/^/    /'
    cam="$out/stretch3/cameras/stretch3_d435i_camera_rgb"
    echo "    文件：$(ls "$cam/rgb" 2>/dev/null | wc -l) 张 PNG，$(ls "$cam/depth" 2>/dev/null | wc -l) 个 .npy，$(wc -l <"$out/stretch3/sensors.jsonl" 2>/dev/null) 行读数"
    last=$(ls "$cam/depth" 2>/dev/null | tail -1)
    [ -n "$last" ] && python3 scripts/sensors/npy_center.py "$cam/depth/$last" | sed 's/^/    最后一帧深度：/'
    python3 scripts/sensors/plot_lidar.py "$out/stretch3/sensors.jsonl" stretch3/scan "$out/stretch3_scan.png" --range 2 2>&1 | sed 's/^/    /'
    record "terra_capture（Stretch 3）" "退出码 $rc，输出在 $out/stretch3/" "$(ok $rc)"
else
    note "terra_capture（Stretch 3）" "未执行（构建失败或参考机器人未准备）"
fi

viewer=build/render-release/apps/viewer/terra_viewer
echo "== 7. 可交互展示"
if [ $skip_show = 1 ]; then
    note "可交互展示" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "可交互展示" "未执行：不在桌面会话中"
elif [ -x "$viewer" ]; then
    echo "（即将打开 terra_viewer，最长 300 秒；设置文件：$out/show/settings.json；资源库：$out/show/library.sqlite3）"
    echo "  Stretch 3 播放巡游动作。请看右侧的“传感器”标签页（与详情、执行器同组；也可从 窗口 → 传感器 打开）："
    echo "    - IMU：底盘 IMU 的比力与角速度曲线（x 红、y 绿、z 蓝）；底盘转向时角速度 z 有变化"
    echo "    - 2D 激光雷达：俯视图，空旷的场景里大多没有命中（灰线），红点是机器人自己的立柱"
    echo "    - 相机：头部 D435i 的彩色与深度，鼠标悬停显示该像素的深度。这个相机在真机上就是横着装的，"
    echo "      面板按场景文件的 displayRotationDeg 逆时针转 90° 显示（数据不变）；深度改为红→橙→绿→青→蓝的配色"
    echo "    - 请留意远处的黄光：在彩色图还是深度图里？视口里看同一片地面是否也有？（帮助判断是不是太阳在地面上的高光）"
    echo "  也可以 文件 → 打开场景… 打开 assets/tests/sensors/sensor_suite.terra-scene.json（IMU、雷达房间、相机夹具）"
    "$viewer" --motion assets/motions/stretch3/tour.terra-motion.json --seconds 300 --settings "$out/show/settings.json" \
        --library "$out/show/library.sqlite3" >"$logs/show.txt" 2>&1
    rc=$?
    grep -E "^INFO 场景|^INFO 动作|^INFO 结束" "$logs/show.txt" | head -8
    record "可交互展示" "退出码 $rc" "$(ok $rc)"
else
    record "可交互展示" "未执行（render-release 构建失败）" 0
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
