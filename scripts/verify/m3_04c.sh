#!/usr/bin/env bash
# M3-04c 基线机器验证：查看器的执行器面板、动作菜单（打开动作文件、从头播放、停止、循环）、状态栏中的动作进度；
# 预定义动作（assets/motions/：G1 挥手与下蹲（悬挂）、G1 自由站立 10 秒、Go2 趴下站起坐下、Stretch 3 底盘与手臂）。
# 2026-11-24 补充：打开动作时切换到它的推荐场景（修正 Go2 动作打不开）；默认环境改为半球光（设置版本 3）。
#
# 请在**桌面会话终端**运行（第 7 步打开可见窗口）；自动化图形测试使用 Xvfb + llvmpipe（D-022）。
# 依次执行（日志在 build/verify-m3-04c/logs/，屏幕只打印结论）：
#   1. 依赖与素材校验（含参考机器人）
#   2. 渲染关闭 Debug：全量 CTest（预期 57/57，新增 robots.motions）
#   3. 渲染开启 Debug 与 Release + -Werror：全量 CTest（预期 84/84，另增 graphics.viewer_motion_selftest）
#   4. ASan/UBSan（渲染开启）
#   5. 变异测试：查看器播放动作（起始关键帧、下发、开始步、切换到推荐场景）、旧设置的环境迁移、预定义动作的目标越界，
#      动作目录按场景过滤，逐个注入 7 个已知错误
#   6. 输出：每个预定义动作结束时的关键数值；查看器动作自检
#   7. 可交互展示（最长 300 秒）：查看器打开 G1 悬挂场景并播放“挥手”；可用“动作”菜单打开其他动作
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m3_04c.sh [--jobs N] [--skip-asan] [--skip-mutation] [--skip-show] [--mutation-only]
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
        -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/verify-m3-04c"
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
    files=(apps/viewer/main.cpp engine/settings/src/settings.cpp engine/simulation/src/motion_file.cpp assets/motions/g1/wave.terra-motion.json)
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
    "播放前不按起始关键帧重置": (
        "    if (const auto key = sim.motion->motion().startKeyframe) sim.runner->resetToKeyframe(*key);",
        "    (void)0;"),
    "每步控制器不下发动作": (
        "        if (motion && motion->playing()) commands = motion->commands(step, instance.world->description().timestep);",
        "        if (false) commands = motion->commands(step, instance.world->description().timestep);"),
    "动作开始步错误": (
        "    sim.motion->start(sim.instance.world->stepIndex());",
        "    sim.motion->start(sim.instance.world->stepIndex() + 50);"),
    "打开动作时不切换到推荐场景": (
        "    if (!wanted.empty() && (!f.simulated || f.simulated->path != wanted)) {",
        "    if (!wanted.empty() && !f.simulated) {"),
    "旧设置的室外贴图不迁移": (
        "    if (fileVersion < 3 && s.environment == EnvironmentSetting::Outdoor) {",
        "    if (false) {"),
    "动作目录不按场景过滤": (
        "            if (!parsed.document || parsed.document->scene != wanted) continue;",
        "            if (!parsed.document) continue;"),
    "预定义动作目标越界": (
        '"g1/right_shoulder_roll_joint": -1.5',
        '"g1/right_shoulder_roll_joint": -2.5'),
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
            n=$(sed -n "/^---- 变异：$name\$/,\$p" "$log" | grep -cE 'FAILED: |FAILED |不匹配|退出码不符')
            summary+=("  变异 $name：已发现（$n 项检查失败）")
        fi
    done <<'LIST'
播放前不按起始关键帧重置|apps/viewer/main.cpp|terra_viewer|graphics.viewer_motion_selftest
每步控制器不下发动作|apps/viewer/main.cpp|terra_viewer|graphics.viewer_motion_selftest
动作开始步错误|apps/viewer/main.cpp|terra_viewer|graphics.viewer_motion_selftest
打开动作时不切换到推荐场景|apps/viewer/main.cpp|terra_viewer|graphics.viewer_motion_selftest
旧设置的室外贴图不迁移|engine/settings/src/settings.cpp|test_settings|settings.store
动作目录不按场景过滤|engine/simulation/src/motion_file.cpp|test_motion_file|motion.file
预定义动作目标越界|assets/motions/g1/wave.terra-motion.json|test_robot_motions|robots.motions
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
ctest --test-dir build/debug -R '^robots\.motions$' -V 2>/dev/null | grep -E "INFO \[|FAILED|跳过" | sed 's/^[0-9]*: /  /'
ctest --test-dir build/render-release -R '^graphics\.viewer_motion_selftest$' -V 2>/dev/null | grep -E "INFO 动作|PASS 动作|PASS 查看器|FAILED" | sed 's/^[0-9]*: /  /'

viewer=build/render-release/apps/viewer/terra_viewer
echo "== 7. 可交互展示"
if [ $skip_show = 1 ]; then
    note "可交互展示" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "可交互展示" "未执行：不在桌面会话中"
elif [ -x "$viewer" ]; then
    echo "（即将打开 terra_viewer，最长 300 秒；设置文件：$out/show/settings.json）"
    echo "  打开 G1 悬挂场景并播放“挥手”（右臂抬起、来回摆 3 次、放下，约 5 秒）。请试："
    echo "    - 右侧“执行器”面板（若没有，菜单 窗口 → 执行器）：播放时只读，显示动作给出的目标与输出；"
    echo "      动作 → 停止动作 后可拖动滑块（例如 left_elbow_joint），手臂跟着动；滑块拖不出范围"
    echo "    - 动作 → 循环、从头播放；状态栏右侧显示动作名与进度"
    echo "    - 动作 → 本场景的预定义动作：列出当前场景可用的动作，点一下即播放（切换场景后列表随之更新）"
    echo "    - 动作 → 打开动作文件…：assets/motions/ 下的其他动作（会切换到各自的场景）："
    echo "        g1/squat（悬挂下蹲）  g1/stand_hold（自由站立 10 秒，可能站不稳——M3-06 记录）"
    echo "        go2/lie_stand_sit（Go2 趴下、站起、坐下、站起）  stretch3/tour（底盘移动转向、升降、伸臂、夹爪）"
    echo "  动作的关节角是保守值；如果哪个动作看起来不自然，请告诉我，我再调整"
    echo "  （默认环境已改为半球光；机器人场景不再指定摄影棚贴图。需要贴图时：视图属性面板 → 环境）"
    "$viewer" --motion assets/motions/g1/wave.terra-motion.json --seconds 300 --settings "$out/show/settings.json" \
        >"$logs/show.txt" 2>&1
    rc=$?
    grep -E "^INFO 场景|^INFO 动作|^INFO 仿真|^INFO 结束|^错误" "$logs/show.txt"
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
