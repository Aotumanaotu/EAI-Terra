#!/usr/bin/env bash
# M1-13 基线机器验证：编辑器式界面外壳（terra_ui + terra_viewer）。
#
# 请在**桌面会话终端**运行（第 6、7 步打开可见窗口）；自动化图形测试自己使用 Xvfb + llvmpipe（D-022）。
# 依次执行（日志在 build/verify-m1-13/logs/，屏幕只打印结论）：
#   1. scripts/render/fetch.sh：获取新增的 Dear ImGui 1.93.0-docking 源码（已有的依赖跳过）
#   2. 渲染关闭 Debug：删除 build/debug 后全量 CTest（预期 37/37）
#   3. 渲染开启 Debug 与 Release + -Werror：删除构建目录后全量 CTest（预期 47/47，新增 ui.input_bridge、ui.text、
#      graphics.viewer_selftest）
#   4. ASan/UBSan（渲染开启，Debug + -Werror）：非图形测试开启 LeakSanitizer；图形测试关闭泄漏检测
#   5. 变异测试：向界面模块逐个注入 5 个已知错误，确认对应测试都能发现（预期 5/5）；源文件结束时恢复并校验
#   6. 真实 GPU 自检：桌面会话中运行查看器 40 帧自检（字体、GL 错误、像素、语言切换），窗口会短暂出现
#   7. 可交互展示：打开 terra_viewer，最长 120 秒——请试试拖动面板、切换中英文、F11、Ctrl+加减号缩放界面、
#      在视口里用中键操作相机、点击左下角坐标系对齐视角；鼠标在面板上时相机不应响应
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m1_13.sh [--jobs N] [--skip-asan] [--skip-mutation] [--skip-show] [--mutation-only]
# --mutation-only：只做变异测试（M1-11 阶段验收调用；不删除、不重建 build/ 下的预设构建目录，跳过 ASan 与真实 GPU 步骤）
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
        -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/verify-m1-13"
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

echo "== 1. 获取依赖"
bash scripts/render/fetch.sh >"$logs/fetch.txt" 2>&1
rc=$?
record "获取并校验依赖" "退出码 $rc" "$(ok $rc)"
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
    files=(engine/ui/src/ui_input.cpp engine/ui/src/text.cpp)
    for f in "${files[@]}"; do cp "$f" "$out/${f//\//_}.orig"; done
    restore() { for f in "${files[@]}"; do cp "$out/${f//\//_}.orig" "$f"; done; }
    trap 'restore; echo "已恢复界面源文件"; exit 130' INT TERM
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
    "输入桥不转发到输入系统": (
        "    target_.setControl(id, control, value, timestamp);\n",
        "    (void)timestamp;\n"),
    "界面占用鼠标时不屏蔽相机": (
        "    input_.setActionMapEnabled(kMouseMap, mouse);",
        "    input_.setActionMapEnabled(kMouseMap, false);"),
    "方向键没有映射到 ImGui": (
        "        case K::Up: return ImGuiKey_UpArrow;",
        "        case K::Up: return ImGuiKey_None;"),
    "修饰键事件缺失": (
        "        keyEvent(static_cast<in::Key>(control.code), down);\n        modifierEvents();\n",
        "        keyEvent(static_cast<in::Key>(control.code), down);\n"),
    "英文文本缺失": (
        '{TextId::MenuSettings, "设置", "Settings"},',
        '{TextId::MenuSettings, "设置", ""},'),
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
        if "$b/tests/$target" >>"$log" 2>&1; then
            summary+=("  变异 $name：未被发现")
        else
            detected=$((detected + 1))
            summary+=("  变异 $name：已发现（$(sed -n "/^---- 变异：$name\$/,\$p" "$log" | grep -c '^FAILED') 项检查失败）")
        fi
    done <<'LIST'
输入桥不转发到输入系统|engine/ui/src/ui_input.cpp|test_ui_input
界面占用鼠标时不屏蔽相机|engine/ui/src/ui_input.cpp|test_ui_input
方向键没有映射到 ImGui|engine/ui/src/ui_input.cpp|test_ui_input
修饰键事件缺失|engine/ui/src/ui_input.cpp|test_ui_input
英文文本缺失|engine/ui/src/text.cpp|test_ui_text
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

echo "== Xvfb 自检输出"
ctest --test-dir build/render-release -R '^graphics\.viewer_selftest$' -V 2>/dev/null | grep -E "INFO|PASS|FAILED" | sed 's/^[0-9]*: //'
ctest --test-dir build/render-release -R '^ui\.text$' -V 2>/dev/null | grep -E "INFO|FAILED" | sed 's/^[0-9]*: //'

viewer=build/render-release/apps/viewer/terra_viewer
echo "== 6. 真实 GPU 自检（桌面会话）"
if [ $skip_show = 1 ]; then
    note "真实 GPU 自检" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "真实 GPU 自检" "未执行：不在桌面会话中"
elif [ -x "$viewer" ]; then
    "$viewer" --frames 40 --self-test >"$logs/desktop_selftest.txt" 2>&1
    rc=$?
    grep -E "^INFO|^PASS|^FAILED" "$logs/desktop_selftest.txt"
    record "真实 GPU 自检" "退出码 $rc" "$(ok $rc)"
else
    record "真实 GPU 自检" "未执行（render-release 构建失败）" 0
fi

echo "== 7. 可交互展示"
if [ $skip_show = 1 ]; then
    note "可交互展示" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "可交互展示" "未执行：不在桌面会话中"
elif [ -x "$viewer" ]; then
    echo "（即将打开 terra_viewer，最长 120 秒，关闭窗口或菜单“文件 → 退出”结束）"
    echo "  试试：拖动面板标签停靠/合并；菜单“设置 → 语言”切换中英文；F11 全屏；Ctrl + 加减号缩放界面（Ctrl+0 恢复）；"
    echo "        鼠标在视口里：中键拖动转视角、Shift+中键平移、WASD 移动、滚轮拉近；鼠标在面板上时相机不应响应；"
    echo "        视口左下角的坐标系会随相机转动，点击 X/Y/Z 圆点对齐视角"
    "$viewer" --seconds 120 >"$logs/show.txt" 2>&1
    rc=$?
    grep -E "^INFO" "$logs/show.txt"
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
