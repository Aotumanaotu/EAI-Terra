#!/usr/bin/env bash
# M1-07 基线机器验证：相机与视口相机控制器（单一组合式控制，D-023）。
#
# 需要先完成 M1-02（scripts/render/fetch.sh 与 build.sh 已运行）。请在**桌面会话终端**运行（第 6 步弹出可见窗口）；
# 自动化图形测试自己使用 Xvfb + llvmpipe（D-022）。
# 依次执行（日志在 build/verify-m1-06/logs/，屏幕只打印结论）：
#   1. 渲染关闭 Debug：删除 build/debug 后全量 CTest（预期 37/37，与 M1-04 相同）
#   2. 渲染开启 Debug 与 Release + -Werror：删除构建目录后全量 CTest（预期 43/43，新增 view.camera）
#   3. ASan/UBSan（渲染开启，Debug + -Werror）：非图形测试开启 LeakSanitizer；图形测试关闭泄漏检测
#      （Mesa/X11 等系统库在进程退出时不释放全部内存，属于已知误报）
#   4. 变异测试：向相机控制器逐个注入 5 个已知错误，确认 view.camera 都能发现（预期 5/5）；源文件结束时恢复并校验
#   5. 打印 view.camera 的 INFO 行
#   6. 可交互展示：桌面会话中打开 1280×720 窗口（地面网格、坐标轴、彩色立方体），最长 60 秒，Esc 退出——
#      请实际操作（这也是 M1-04 留下的真实键鼠人工检查）
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m1_07.sh [--jobs N] [--skip-asan] [--skip-mutation] [--skip-show] [--mutation-only]
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
        -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/verify-m1-07"
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
ok() { [ "$1" = 0 ] && echo 1 || echo 0; }
ctest_result() { grep -E "tests passed|tests failed" "$1" | tail -1; }

# 与 tests/CMakeLists.txt 中 TERRA_GRAPHICS_TEST_ENV 相同的固定环境（D-022）
graphics_env() {
    env -u DISPLAY -u WAYLAND_DISPLAY __GLX_VENDOR_LIBRARY_NAME=mesa LIBGL_ALWAYS_SOFTWARE=1 \
        xvfb-run -a -s "-screen 0 1280x720x24" "$@"
}

if [ ! -f third_party/render/install/lib/cmake/glfw3/glfw3Config.cmake ]; then
    echo "错误：未找到 GLFW 安装，请先运行 scripts/render/fetch.sh 与 scripts/render/build.sh（M1-02）" >&2
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
    else
        record "$label 构建" "失败（见 $log）" 0
        grep -E -A3 "error:|warning:" "$log" | head -20 | sed 's/^/    /'
    fi
}

echo "== 1. 渲染关闭"
run_preset debug "渲染关闭 Debug"

echo "== 2. 渲染开启"
run_preset render-debug "渲染 Debug"
run_preset render-release "渲染 Release -Werror" -DTERRA_WARNINGS_AS_ERRORS=ON

if [ $skip_asan = 0 ]; then
    echo "== 3. ASan/UBSan（渲染开启）"
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
    echo "== 4. 变异测试"
    log="$logs/mutation.txt"
    b="$out/mutation"
    files=(engine/view/src/camera_controller.cpp)
    for f in "${files[@]}"; do cp "$f" "$out/$(basename "$f").orig"; done
    restore() { for f in "${files[@]}"; do cp "$out/$(basename "$f").orig" "$f"; done; }
    trap 'restore; echo "已恢复相机源文件"; exit 130' INT TERM
    cmake -S . -B "$b" -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug -DTERRA_BUILD_RENDER=ON \
          -DTERRA_WARNINGS_AS_ERRORS=ON >"$log" 2>&1
    detected=0
    total=0
    while IFS='|' read -r name file; do
        total=$((total + 1))
        restore
        if ! python3 - "$file" "$name" >>"$log" 2>&1 <<'PY'
import sys
path, name = sys.argv[1], sys.argv[2]
mutations = {
    "转视角不需要按中键": (
        "{in::Binding::pair(dx, dy).withModifier(in::mouse(in::MouseControl::ButtonMiddle))}",
        "{in::Binding::pair(dx, dy)}"),
    "前进不含俯仰": (
        "add(scale(forward, static_cast<double>(m.y) * v),",
        "add(scale(Vec3{forward.x, forward.y, 0.0}, static_cast<double>(m.y) * v),"),
    "滚轮方向反了": (
        "scale(forward, static_cast<double>(wheel) * settings_.wheelStep * factor)",
        "scale(forward, -static_cast<double>(wheel) * settings_.wheelStep * factor)"),
    "加速无效": (
        "const double factor = input_.state(boost_).down ? settings_.boost : 1.0;",
        "const double factor = 1.0;"),
    "方向键未绑定": (
        # M1-14 起按键由 CameraKeys 的两个槽位生成：只生成第一个槽位 = 方向键（备用槽位）未绑定
        "    for (std::size_t i = 0; i < 2; ++i) {\n        if (k.forward[i] == in::Key::Unknown",
        "    for (std::size_t i = 0; i < 1; ++i) {\n        if (k.forward[i] == in::Key::Unknown"),
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
        if ! cmake --build "$b" -j "$jobs" --target test_view >>"$log" 2>&1; then
            summary+=("  变异 $name：编译失败，未判定（见 $log）")
            failed=1
            continue
        fi
        if "$b/tests/test_view" >>"$log" 2>&1; then
            summary+=("  变异 $name：未被发现")
        else
            detected=$((detected + 1))
            summary+=("  变异 $name：已发现（$(sed -n "/^---- 变异：$name\$/,\$p" "$log" | grep -c '^FAILED') 项检查失败）")
        fi
    done <<'LIST'
转视角不需要按中键|engine/view/src/camera_controller.cpp
前进不含俯仰|engine/view/src/camera_controller.cpp
滚轮方向反了|engine/view/src/camera_controller.cpp
加速无效|engine/view/src/camera_controller.cpp
方向键未绑定|engine/view/src/camera_controller.cpp
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

echo "== 5. 测试输出"
ctest --test-dir build/render-release -R '^view\.camera$' -V 2>/dev/null | grep -E "INFO|FAILED" | sed 's/^[0-9]*: //'

echo "== 6. 可交互展示（真实 GPU）"
show=build/render-release/tests/test_view
if [ $skip_show = 1 ]; then
    summary+=("$(printf '%-28s %s' "可见展示" "已跳过（--skip-show）")")
    echo "${summary[-1]}"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    summary+=("$(printf '%-28s %s' "可见展示" "未执行：不在桌面会话中")")
    echo "${summary[-1]}"
elif [ -x "$show" ]; then
    echo "（即将弹出可交互窗口，最长 60 秒，Esc 退出）"
    echo "  右键拖动转视角；W/↑ S/↓ 沿视线前后，A/← D/→ 左右；E/空格 上升，Q/Ctrl 下降"
    echo "  滚轮拉近/拉远；中键拖动平移；Shift 加速；F 复位"
    "$show" --show 60 >"$logs/show.txt" 2>&1
    rc=$?
    cat "$logs/show.txt"
    record "可见展示" "退出码 $rc" "$(ok $rc)"
else
    record "可见展示" "未执行（render-release 构建失败）" 0
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
