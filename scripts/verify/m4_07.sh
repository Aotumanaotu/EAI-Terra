#!/usr/bin/env bash
# M4-07 基线机器验证：传感器记录（计划 11.2 节；记录格式版本 2）——
# 实验记录增加传感器：IMU 随每份状态记录；2D 激光雷达与相机元数据（步序号、时间、位姿）按各自的频率记录；
# 查看器可选同时保存实际渲染的相机图像（记录旁的 .frames 目录）；复现时 IMU、雷达与相机位姿逐位比较；
# 回放时传感器面板跟随记录。版本 1 的记录照常读取。
#
# 请在**桌面会话终端**运行（第 7 步打开查看器）。
# 依次执行（日志在 build/verify-m4-07/logs/，屏幕只打印结论）：
#   1. 依赖与素材校验
#   2. 渲染关闭 Debug：全量 CTest（预期 88/88，新增 record.sensors、headless.record_sensors、headless.rerun_sensors）
#   3. 渲染开启 Debug 与 Release + -Werror：全量 CTest（预期 122/122，另新增 graphics.viewer_sensor_record_selftest）
#   4. ASan/UBSan（渲染开启）
#   5. 变异测试：逐个注入 7 个已知错误
#   6. 输出：记录与复现的 INFO 行
#   7. 可交互展示（最长 300 秒）：查看器播放 Stretch 3 巡游，按提示记录（含相机图像）并回放
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m4_07.sh [--jobs N] [--skip-asan] [--skip-mutation] [--skip-show] [--mutation-only]
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
        -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/verify-m4-07"
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
    files=(engine/simulation/src/record.cpp apps/viewer/main.cpp apps/headless/main.cpp)
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
    "状态不记 IMU": (
        "    if (!s.sensors.empty()) {",
        "    if (false) {"),
    "雷达的无穷大写成 null": (
        "    if (std::isinf(r)) return r > 0 ? Json(\"inf\") : Json(\"-inf\");",
        "    (void)0;"),
    "复现不比较雷达": (
        "            if (!same) sensorDiffers(step, \"雷达 \" + expected.name + \" 的距离\");",
        "            (void)same;"),
    "回放时传感器面板不跟随": (
        "    if (sim.replaySensorsShown && sim.replaySensorTime == t) return;",
        "    return;"),
    "不保存相机图像": (
        "        if (preview.pendingRecorded && sim.recorder && f.state.recordCameraImages) saveRecordedImage(f, camera, *preview.frame);  // M4-07",
        "        (void)0;"),
    "记录开始时漏记雷达与相机": (
        "        terra::simulation::recordSceneSensors(*sim.recorder, *sim.instance.world, sim.instance, *start);",
        "        (void)start;"),
    "Headless 不记录雷达与相机": (
        "        if (recorder && sceneInstance) terra::simulation::recordSceneSensors(*recorder, world, *sceneInstance, *world.snapshot());",
        "        (void)0;"),
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
状态不记 IMU|engine/simulation/src/record.cpp|test_record_file|record.file
雷达的无穷大写成 null|engine/simulation/src/record.cpp|test_record_file|record.file
复现不比较雷达|engine/simulation/src/record.cpp|test_record_rerun|record.sensors
回放时传感器面板不跟随|apps/viewer/main.cpp|terra_viewer|graphics.viewer_sensor_record_selftest
不保存相机图像|apps/viewer/main.cpp|terra_viewer|graphics.viewer_sensor_record_selftest
记录开始时漏记雷达与相机|apps/viewer/main.cpp|terra_viewer|graphics.viewer_sensor_record_selftest
Headless 不记录雷达与相机|apps/headless/main.cpp|terra_headless|headless.rerun_sensors
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
ctest --test-dir build/render-release -R '^(record\.sensors|graphics\.viewer_sensor_record_selftest|headless\.rerun_sensors)$' -V 2>/dev/null |
    grep -E "INFO|PASS|FAILED|rerun" | sed 's/^[0-9]*: /  /' | cut -c1-260

viewer=build/render-release/apps/viewer/terra_viewer
echo "== 7. 可交互展示"
if [ $skip_show = 1 ]; then
    note "可交互展示" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "可交互展示" "未执行：不在桌面会话中"
elif [ -x "$viewer" ]; then
    echo "（即将打开 terra_viewer，最长 300 秒；设置文件：$out/show/settings.json；资源库与记录：$out/show/）"
    echo "  Stretch 3 播放巡游动作。建议这样看："
    echo "    1. 菜单 仿真 → 勾选“记录相机图像”，再按 Ctrl+Shift+R 开始记录（动作从头播放）；过 10 秒左右再按一次停止"
    echo "    2. 底部“时间轴”中点刚保存的记录回放；拖动时间轴，右侧“传感器”面板的 IMU 曲线、雷达俯视图与头部相机都跟着记录走"
    echo "    3. 记录文件与 .frames 图像目录在 $out/show/records/"
    "$viewer" --motion assets/motions/stretch3/tour.terra-motion.json --seconds 300 --settings "$out/show/settings.json" \
        --library "$out/show/library.sqlite3" >"$logs/show.txt" 2>&1
    rc=$?
    grep -E "^INFO 场景 |^INFO 开始记录|^INFO 记录已保存|^INFO 开始回放|^INFO 结束" "$logs/show.txt" | head -10 | sed 's/^/  /'
    shown=$(ls -d "$out"/show/records/*.frames 2>/dev/null | head -1)
    [ -n "$shown" ] && echo "  图像：$(find "$shown" -name '*.png' | wc -l) 张 PNG，$(find "$shown" -name '*.npy' | wc -l) 个 .npy（$shown）"
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
