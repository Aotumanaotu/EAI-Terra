#!/usr/bin/env bash
# M1-14 基线机器验证：设置与持久化（terra_settings + 查看器偏好设置窗口）。
#
# 请在**桌面会话终端**运行（第 6–8 步打开可见窗口）；自动化图形测试自己使用 Xvfb + llvmpipe（D-022）。
# 依次执行（日志在 build/verify-m1-14/logs/，屏幕只打印结论）：
#   1. 复核已提交的 nlohmann/json 3.12.0（sha256，离线）；scripts/render/fetch.sh（已有依赖跳过）
#   2. 渲染关闭 Debug：删除 build/debug 后全量 CTest（预期 38/38，新增 settings.store）
#   3. 渲染开启 Debug 与 Release + -Werror：删除构建目录后全量 CTest（预期 48/48）
#   4. ASan/UBSan（渲染开启，Debug + -Werror）：非图形测试开启 LeakSanitizer；图形测试关闭泄漏检测
#   5. 变异测试：向设置、输入、相机控制器逐个注入 6 个已知错误，确认对应测试都能发现（预期 6/6）；源文件结束时恢复并校验
#   6. 真实 GPU 自检：桌面会话中运行查看器 40 帧自检（含偏好设置窗口与设置生效检查），窗口会短暂出现
#   7. 可交互展示（第一次，最长 120 秒）：使用 build/verify-m1-14/show/settings.json（全新文件）——
#      请在“设置 → 偏好设置…”（Ctrl+,）中改语言、界面缩放、相机速度，改一个按键，拖动面板改布局，然后关闭窗口
#   8. 可交互展示（第二次，最长 60 秒）：用同一个设置文件再次打开，确认上一次的改动都还在
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m1_14.sh [--jobs N] [--skip-asan] [--skip-mutation] [--skip-show]
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

out="$root/build/verify-m1-14"
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
bash scripts/render/fetch.sh >"$logs/fetch.txt" 2>&1
rc=$?
record "获取并校验渲染依赖" "退出码 $rc" "$(ok $rc)"
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
    files=(engine/settings/src/settings.cpp engine/input/src/input_system.cpp engine/view/src/camera_controller.cpp)
    for f in "${files[@]}"; do cp "$f" "$out/$(basename "$f").orig"; done
    restore() { for f in "${files[@]}"; do cp "$out/$(basename "$f").orig" "$f"; done; }
    trap 'restore; echo "已恢复被变异的源文件"; exit 130' INT TERM
    cmake -S . -B "$b" -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug -DTERRA_BUILD_RENDER=ON \
          -DTERRA_WARNINGS_AS_ERRORS=ON >"$log" 2>&1
    detected=0
    total=0
    while IFS='|' read -r name file target; do
        total=$((total + 1))
        restore
        if ! python3 - "$file" "$name" >>"$log" 2>&1 <<'PY'
import sys
path, name = sys.argv[1], sys.argv[2]
mutations = {
    "超出范围的值没有回退默认值": (
        "        if (!std::isfinite(v) || v < range.min || v > range.max) {",
        "        if (!std::isfinite(v)) {"),
    "语法错误被当作读取成功": (
        "        result.status = LoadStatus::Corrupt;",
        "        result.status = LoadStatus::Loaded;"),
    "重复按键没有去重": (
        "            if (first) {",
        "            if (first && false) {"),
    "XDG_CONFIG_HOME 被忽略": (
        "    if (xdgConfigHome != nullptr && xdgConfigHome[0] == '/') {",
        "    if (xdgConfigHome != nullptr && xdgConfigHome[0] == '#') {"),
    "替换绑定不生效": (
        "            definition.bindings = std::move(bindings);",
        "            (void)bindings;"),
    "平移修饰键不跟随加速键": (
        "    if (auto error = input_.setBindings(kPanMapName, \"pan\", panBindings(keys_))) {",
        "    if (auto error = input_.setBindings(kPanMapName, \"pan\", panBindings(CameraKeys{}))) {"),
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
        args=()
        [ "$target" = test_settings ] && args=("$out/mutation_settings_tmp")
        if "$b/tests/$target" "${args[@]}" >>"$log" 2>&1; then
            summary+=("  变异 $name：未被发现")
        else
            detected=$((detected + 1))
            summary+=("  变异 $name：已发现（$(sed -n "/^---- 变异：$name\$/,\$p" "$log" | grep -c '^FAILED') 项检查失败）")
        fi
    done <<'LIST'
超出范围的值没有回退默认值|engine/settings/src/settings.cpp|test_settings
语法错误被当作读取成功|engine/settings/src/settings.cpp|test_settings
重复按键没有去重|engine/settings/src/settings.cpp|test_settings
XDG_CONFIG_HOME 被忽略|engine/settings/src/settings.cpp|test_settings
替换绑定不生效|engine/input/src/input_system.cpp|test_input_system
平移修饰键不跟随加速键|engine/view/src/camera_controller.cpp|test_view
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

echo "== Xvfb 自检输出"
ctest --test-dir build/render-release -R '^graphics\.viewer_selftest$' -V 2>/dev/null | grep -E "INFO|PASS|FAILED" | sed 's/^[0-9]*: //'
ctest --test-dir build/render-release -R '^ui\.text$' -V 2>/dev/null | grep -E "INFO|FAILED" | sed 's/^[0-9]*: //'
echo "== 自检保存的设置文件（构建目录，不是 ~/.config）"
selftest_settings=build/render-release/tests/viewer_selftest/settings.json
if [ -f "$selftest_settings" ]; then
    python3 - "$selftest_settings" <<'PY'
import json, sys
d = json.load(open(sys.argv[1], encoding="utf-8"))
layout = d.get("interface", {}).pop("layout", "")
print("  语言", d["interface"]["language"], "| 界面缩放", d["interface"]["uiScale"], "| 相机速度", d["camera"]["speed"],
      "| 视场角", d["camera"]["fovDegrees"], "| 前进键", d["keys"]["camera.forward"], "| 全屏键", d["keys"]["app.fullscreen"],
      "| 布局", len(layout), "字节")
PY
else
    echo "  未找到 $selftest_settings"
fi

viewer=build/render-release/apps/viewer/terra_viewer
echo "== 6. 真实 GPU 自检（桌面会话）"
if [ $skip_show = 1 ]; then
    note "真实 GPU 自检" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "真实 GPU 自检" "未执行：不在桌面会话中"
elif [ -x "$viewer" ]; then
    "$viewer" --frames 40 --self-test --settings "$out/selftest/settings.json" >"$logs/desktop_selftest.txt" 2>&1
    rc=$?
    grep -E "^INFO|^PASS|^FAILED" "$logs/desktop_selftest.txt"
    record "真实 GPU 自检" "退出码 $rc" "$(ok $rc)"
else
    record "真实 GPU 自检" "未执行（render-release 构建失败）" 0
fi

# 打印设置文件的要点（布局只给长度）
show_settings() {
    python3 - "$1" <<'PY'
import json, sys
try:
    d = json.load(open(sys.argv[1], encoding="utf-8"))
except Exception as e:
    print("  无法读取设置文件：", e)
    sys.exit(0)
ui, disp, cam, keys = d.get("interface", {}), d.get("display", {}), d.get("camera", {}), d.get("keys", {})
defaults = {"camera.forward": ["W", "Up"], "camera.back": ["S", "Down"], "camera.left": ["A", "Left"],
            "camera.right": ["D", "Right"], "camera.up": ["E", "Space"], "camera.down": ["Q", ""],
            "camera.boost": ["LeftShift", "RightShift"], "camera.reset": ["F", ""], "app.fullscreen": ["F11", ""]}
changed = {k: v for k, v in keys.items() if defaults.get(k) != v}
print(f"  界面：语言 {ui.get('language')}，缩放 {ui.get('uiScale')}，面板 {ui.get('panels')}，布局 {len(ui.get('layout', ''))} 字节")
print(f"  显示：{disp.get('mode')}，显示器 {disp.get('monitor')}，模式 {disp.get('videoMode')}，垂直同步 {disp.get('vsync')}，"
      f"窗口 {disp.get('windowSize')}")
print(f"  相机：{cam}")
print(f"  改过的按键：{changed if changed else '无'}")
PY
}

show_settings_file="$out/show/settings.json"
echo "== 7. 可交互展示（第一次）"
if [ $skip_show = 1 ]; then
    note "可交互展示（第一次）" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "可交互展示（第一次）" "未执行：不在桌面会话中"
elif [ -x "$viewer" ]; then
    echo "（即将打开 terra_viewer，最长 120 秒；设置文件：$show_settings_file，目前不存在）"
    echo "  请试试：菜单“设置 → 偏好设置…”或 Ctrl+, 打开偏好设置——"
    echo "    界面：切换语言、取消“自动”后拖动界面缩放；显示：切换无边框全屏、垂直同步；"
    echo "    相机：调移动速度与视场角（视口立即变化）；按键：点击“前进”的按键格后按一个新键（如 I），右键清除备用键；"
    echo "  再拖动几个面板改变布局，然后关闭窗口（设置会自动保存）"
    "$viewer" --seconds 120 --settings "$show_settings_file" >"$logs/show1.txt" 2>&1
    rc=$?
    grep -E "^INFO" "$logs/show1.txt"
    record "可交互展示（第一次）" "退出码 $rc" "$(ok $rc)"
    if [ -f "$show_settings_file" ]; then
        echo "  保存的设置："
        show_settings "$show_settings_file"
    fi
else
    record "可交互展示（第一次）" "未执行（render-release 构建失败）" 0
fi

echo "== 8. 可交互展示（第二次，确认设置保留）"
if [ $skip_show = 1 ]; then
    note "可交互展示（第二次）" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "可交互展示（第二次）" "未执行：不在桌面会话中"
elif [ -x "$viewer" ] && [ -f "$show_settings_file" ]; then
    echo "（再次打开，最长 60 秒：语言、界面缩放、面板布局、相机参数和改过的按键应与上次关闭时相同）"
    "$viewer" --seconds 60 --settings "$show_settings_file" >"$logs/show2.txt" 2>&1
    rc=$?
    grep -E "^INFO" "$logs/show2.txt"
    loaded=$(grep -c "（已读取" "$logs/show2.txt")
    record "可交互展示（第二次）" "退出码 $rc，$([ "$loaded" -ge 1 ] && echo 已读取上次的设置 || echo 未读取到设置)" \
        "$([ $rc = 0 ] && [ "$loaded" -ge 1 ] && echo 1 || echo 0)"
else
    record "可交互展示（第二次）" "未执行（第一次展示没有生成设置文件）" 0
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
