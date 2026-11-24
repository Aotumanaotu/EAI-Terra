#!/usr/bin/env bash
# M3-07d 基线机器验证：记录、回放与执行器面板的交互改进（设计稿 2026-11-24 评审通过）——
# 视口工具栏的图形按钮与录制按钮（Ctrl+Shift+R，可在偏好设置中另配单键）；底部“时间轴”标签页（与日志、统计并列：
# 播放控制、实时 / 回放、时间刻度与播放头、记录列表）；执行器面板的“实测 + 目标”控件，回放时与记录同步。
#
# 请在**桌面会话终端**运行（第 7 步打开可见窗口）；自动化图形测试使用 Xvfb + llvmpipe（D-022）。
# 依次执行（日志在 build/verify-m3-07d/logs/，屏幕只打印结论）：
#   1. 依赖与素材校验
#   2. 渲染关闭 Debug：全量 CTest（预期 73/73，record.file 扩充：某一步生效的控制量）
#   3. 渲染开启 Debug 与 Release + -Werror：全量 CTest（预期 102/102，graphics.viewer_record_selftest 扩充）
#   4. ASan/UBSan（渲染开启）
#   5. 变异测试：录制按钮不开始记录、面板不跟随回放的状态 / 控制量、回放步序号、时间轴不在底部组、controlsAt，逐个注入 6 个已知错误
#   6. 输出：查看器记录与回放自检
#   7. 可交互展示（最长 300 秒）：G1 悬挂场景播放挥手；请试工具栏录制、Ctrl+Shift+R、时间轴标签页与执行器面板
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m3_07d.sh [--jobs N] [--skip-asan] [--skip-mutation] [--skip-show] [--mutation-only]
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

out="$root/build/verify-m3-07d"
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
    files=(engine/simulation/src/record.cpp apps/viewer/main.cpp)
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
    "录制按钮不开始记录": (
        "    else startRecording(f);",
        "    else (void)0;"),
    "回放时执行器面板用实时状态": (
        "            const terra::data::PhysicsSnapshot& snap = replayed ? *replayed : *live;",
        "            const terra::data::PhysicsSnapshot& snap = *live;"),
    "回放时目标不取记录中的控制量": (
        "                recorded = terra::simulation::controlsAt(sim.replay->record, step > 0 ? step - 1 : 0);",
        "                (void)step;"),
    "回放的步序号偏一步": (
        "        if (next->simulationTime == t || next == states.begin()) return next->stepIndex;",
        "        if (next->simulationTime == t || next == states.begin()) return next->stepIndex + 1;"),
    "时间轴不在底部组": (
        '    ImGui::DockBuilderDockWindow("###Timeline", bottom);',
        '    ImGui::DockBuilderDockWindow("###Timeline", center);'),
    "controlsAt 取到之后一步的控制量": (
        "        if (c.step > step) break;",
        "        if (c.step > step + 1) break;"),
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
录制按钮不开始记录|apps/viewer/main.cpp|terra_viewer|graphics.viewer_record_selftest
回放时执行器面板用实时状态|apps/viewer/main.cpp|terra_viewer|graphics.viewer_record_selftest
回放时目标不取记录中的控制量|apps/viewer/main.cpp|terra_viewer|graphics.viewer_record_selftest
回放的步序号偏一步|apps/viewer/main.cpp|terra_viewer|graphics.viewer_record_selftest
时间轴不在底部组|apps/viewer/main.cpp|terra_viewer|graphics.viewer_record_selftest
controlsAt 取到之后一步的控制量|engine/simulation/src/record.cpp|test_record_file|record.file
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
ctest --test-dir build/render-release -R '^graphics\.viewer_record_selftest$' -V 2>/dev/null | grep -E "INFO|PASS|FAILED" | sed 's/^[0-9]*: /  /'

viewer=build/render-release/apps/viewer/terra_viewer
echo "== 7. 可交互展示"
if [ $skip_show = 1 ]; then
    note "可交互展示" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "可交互展示" "未执行：不在桌面会话中"
elif [ -x "$viewer" ]; then
    echo "（即将打开 terra_viewer，最长 300 秒；设置文件：$out/show/settings.json；资源库：$out/show/library.sqlite3）"
    echo "  打开 G1 悬挂场景并播放挥手。请试："
    echo "    - 视口工具栏：播放 / 暂停、单步、重置为图形按钮，悬停有说明与快捷键；红色圆点 = 录制"
    echo "      点一下开始录制（动作从头播放），按钮变成红色方块并显示录制时长与大小；再点一下停止"
    echo "    - Ctrl+Shift+R：同样开始 / 停止录制（Ctrl+R 仍是重置）；偏好设置 → 按键 中可给“仿真：开始 / 停止记录”另配单键"
    echo "    - 底部“时间轴”标签页（与日志、统计并列）：录制时刻度上有红色区段；下面是本场景的记录（可勾选全部场景）"
    echo "      双击记录或点行尾 ▶ 即回放：刻度变为记录的范围，可拖动播放头；点“实时”退出回放"
    echo "    - 执行器面板：填充条 = 实测（位置执行器为关节位置），竖线 = 目标；暂停或停止动作后在条上拖动设置目标，"
    echo "      双击输入精确值；回放时面板跟着记录动（只读）；顶部可按名字过滤，按实例折叠"
    echo "    - 文件 → 实验记录… 会切到时间轴标签页"
    "$viewer" --motion assets/motions/g1/wave.terra-motion.json --seconds 300 --settings "$out/show/settings.json" \
        --library "$out/show/library.sqlite3" >"$logs/show.txt" 2>&1
    rc=$?
    grep -E "^INFO 场景 |^INFO 开始记录|^INFO 记录已保存|^INFO 开始回放|^INFO 仿真|^INFO 结束|^错误" "$logs/show.txt"
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
