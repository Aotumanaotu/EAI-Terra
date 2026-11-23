#!/usr/bin/env bash
# M2-08 基线机器验证：查看器仿真模式——打开场景（程序内文件浏览、最近打开）、播放 / 暂停 / 单步 / 重置、速度、
# 状态栏仿真信息、场景大纲与实体属性、选中高亮、场景文件的环境与相机；Headless --scene；设置版本 2（仿真按键）。
#
# 请在**桌面会话终端**运行（第 7 步打开可见窗口）；自动化图形测试使用 Xvfb + llvmpipe（D-022）。
# 依次执行（日志在 build/verify-m2-08/logs/，屏幕只打印结论）：
#   1. 依赖与素材校验
#   2. 渲染关闭 Debug：全量 CTest（预期 49/49，新增 headless.run_scene、scene_invalid、scene_and_model）
#   3. 渲染开启 Debug 与 Release + -Werror：全量 CTest（预期 69/69，新增 graphics.viewer_scene_selftest）
#   4. ASan/UBSan（渲染开启）
#   5. 变异测试：打开后暂停、按键迁移、最近打开去重、--scene/--model 互斥、文件过滤、隐藏文件、选中高亮、重置，
#      逐个注入 8 个已知错误（预期 8/8）
#   6. 输出：查看器场景自检与 Headless 场景运行的结果
#   7. 可交互展示（最长 180 秒）：打开两辆小车场景（演示驱动），可用菜单打开“下落的方块”场景
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m2_08.sh [--jobs N] [--skip-asan] [--skip-mutation] [--skip-show]
set -uo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
cd "$root"

jobs=$(nproc)
skip_asan=0
skip_mutation=0
skip_show=0
while [ $# -gt 0 ]; do
    case "$1" in
        --jobs) jobs=$2; shift 2 ;;
        --skip-asan) skip_asan=1; shift ;;
        --skip-mutation) skip_mutation=1; shift ;;
        --skip-show) skip_show=1; shift ;;
        -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/verify-m2-08"
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
if [ ! -f third_party/render/install/lib/cmake/glfw3/glfw3Config.cmake ]; then
    echo "错误：未找到 GLFW 安装，请先运行 scripts/render/build.sh（M1-02）" >&2
    exit 1
fi

run_preset() {  # run_preset <preset> <名称> [额外配置参数...]
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
    files=(apps/viewer/main.cpp engine/settings/src/settings.cpp apps/headless/cli.cpp engine/ui/src/file_browser.cpp apps/viewer/scene_view.cpp)
    for f in "${files[@]}"; do cp "$f" "$out/$(basename "$f").orig"; done
    restore() { for f in "${files[@]}"; do cp "$out/$(basename "$f").orig" "$f"; done; }
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
    "打开场景后不暂停": (
        "    sim.runner->pause();\n    std::vector<vw::SceneMessage> messages;",
        "    std::vector<vw::SceneMessage> messages;"),
    "版本 1 不迁移空格键": (
        "    if (fileVersion < 2 && keys[Command::CameraUp] == KeySlots{in::Key::E, in::Key::Space}) {",
        "    if (false && keys[Command::CameraUp] == KeySlots{in::Key::E, in::Key::Space}) {"),
    "最近打开列表不去重": (
        "    list.erase(std::remove(list.begin(), list.end(), path), list.end());",
        "    (void)0;"),
    "--scene 与 --model 可同时使用": (
        "    if (seenModel && seenScene) {",
        "    if (false) {"),
    "文件浏览不过滤后缀": (
        "            if (!filter.empty() && !endsWith(name, filter)) continue;",
        "            (void)endsWith;"),
    "隐藏文件被列出": (
        "        if (name.empty() || name[0] == '.') continue;",
        "        if (name.empty()) continue;"),
    "选中实体不高亮": (
        "        const bool selected = options.highlight.valid() && part.entity == options.highlight;",
        "        const bool selected = false;"),
    "重置不回到第 0 步": (
        "    f.simulated->runner->reset();",
        "    (void)0;"),
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
        # 经 CTest 运行，与正式测试的参数和环境（如 TERRA_CRASH_DIR）完全相同
        if ctest --test-dir "$b" -R "^${testname//./\\.}\$" --output-on-failure >>"$log" 2>&1; then
            summary+=("  变异 $name：未被发现")
        else
            detected=$((detected + 1))
            n=$(sed -n "/^---- 变异：$name\$/,\$p" "$log" | grep -cE 'FAILED: |不匹配|退出码不符')
            summary+=("  变异 $name：已发现（$n 项检查失败）")
        fi
    done <<'LIST'
打开场景后不暂停|apps/viewer/main.cpp|terra_viewer|graphics.viewer_scene_selftest
版本 1 不迁移空格键|engine/settings/src/settings.cpp|test_settings|settings.store
最近打开列表不去重|engine/settings/src/settings.cpp|test_settings|settings.store
--scene 与 --model 可同时使用|apps/headless/cli.cpp|test_headless_cli|headless.cli_parser
文件浏览不过滤后缀|engine/ui/src/file_browser.cpp|test_ui_text|ui.text
隐藏文件被列出|engine/ui/src/file_browser.cpp|test_ui_text|ui.text
选中实体不高亮|apps/viewer/scene_view.cpp|terra_viewer|graphics.viewer_scene_selftest
重置不回到第 0 步|apps/viewer/main.cpp|terra_viewer|graphics.viewer_scene_selftest
LIST
    restore
    trap - INT TERM
    restored=1
    for f in "${files[@]}"; do cmp -s "$f" "$out/$(basename "$f").orig" || restored=0; done
    if [ $restored = 1 ]; then
        record "变异测试" "$detected/$total 被发现；源文件已恢复" "$([ $detected = $total ] && echo 1 || echo 0)"
    else
        record "变异测试" "源文件恢复失败！请用 $out/*.orig 手动恢复" 0
    fi
fi

echo "== 6. 输出"
ctest --test-dir build/render-release -R '^graphics\.viewer_scene_selftest$' -V 2>/dev/null | grep -E "INFO|PASS|FAILED" | sed 's/^[0-9]*: //'
build/debug/apps/headless/terra_headless --scene assets/tests/scene/drop.terra-scene.json --duration 2 --sample-every 1000 \
    >"$logs/headless_drop.jsonl" 2>"$logs/headless_drop.err"
rc=$?
echo "  Headless 运行“下落的方块”场景 2 秒：退出码 $rc，$(wc -l <"$logs/headless_drop.jsonl") 行输出；最后一行："
tail -1 "$logs/headless_drop.jsonl" | cut -c1-160 | sed 's/^/    /'
record "Headless 场景运行" "退出码 $rc" "$(ok $rc)"

viewer=build/render-release/apps/viewer/terra_viewer
echo "== 7. 可交互展示"
if [ $skip_show = 1 ]; then
    note "可交互展示" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "可交互展示" "未执行：不在桌面会话中"
elif [ -x "$viewer" ]; then
    echo "（即将打开 terra_viewer，最长 180 秒；设置文件：$out/show/settings.json）"
    echo "  打开的是两辆小车场景（演示驱动，已在运行）。请试："
    echo "    - 视口上方工具栏：暂停 / 播放、单步（暂停时）、重置、速度；状态栏右侧是仿真时间、步数、实时因子、过载次数"
    echo "    - 键盘：空格 播放 / 暂停，句号 单步，Ctrl+R 重置（可在偏好设置的“按键”页修改前两项）"
    echo "    - 大纲：模型实例 → 实体 → 视觉部件；点选 cart_a/chassis，视口中它会带暖色高亮，属性面板显示位姿与速度"
    echo "    - 文件 → 打开场景…（Ctrl+O）：进入 assets/tests/scene/，打开 drop.terra-scene.json（下落的方块）——"
    echo "      打开后处于暂停，按空格开始，方块落地翻滚；该场景自带摄影棚环境与相机位置；文件 → 最近打开 中有记录"
    "$viewer" --scene assets/tests/scene/two_carts.terra-scene.json --demo-drive --seconds 180 \
        --settings "$out/show/settings.json" >"$logs/show.txt" 2>&1
    rc=$?
    grep -E "^INFO 场景|^INFO 仿真|^INFO 结束|^错误" "$logs/show.txt"
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
