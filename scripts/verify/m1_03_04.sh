#!/usr/bin/env bash
# M1-03 / M1-04 基线机器验证：平台窗口与输入抽象。
#
# 需要先完成 M1-02（scripts/render/fetch.sh 与 build.sh 已运行）。可在任意终端运行；图形测试自己使用 Xvfb。
# 依次执行（日志在 build/verify-m1-03-04/logs/，屏幕只打印结论）：
#   1. 渲染关闭 Debug：删除 build/debug 后全量 CTest（预期 37/37：M0 的 35 项 + input.system + boundary.render_isolation）
#   2. 渲染开启 Debug 与 Release + -Werror：删除构建目录后全量 CTest（预期 41/41）
#   3. ASan/UBSan（渲染开启，Debug + -Werror）：非图形测试开启 LeakSanitizer；图形测试关闭泄漏检测
#      （Mesa/X11 等系统库在进程退出时不释放全部内存，属于已知误报）
#   4. 变异测试：向输入系统逐个注入 8 个已知错误，确认 input.system 都能发现（预期 8/8）；源文件结束时恢复并校验
#   5. 打印 input.system 与平台窗口测试的 INFO 行
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m1_03_04.sh [--jobs N] [--skip-asan] [--skip-mutation]
set -uo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
cd "$root"

jobs=$(nproc)
skip_asan=0
skip_mutation=0
while [ $# -gt 0 ]; do
    case "$1" in
        --jobs) jobs=$2; shift 2 ;;
        --skip-asan) skip_asan=1; shift ;;
        --skip-mutation) skip_mutation=1; shift ;;
        -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/verify-m1-03-04"
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

if [ ! -f third_party/render/install/lib/cmake/glfw3/glfw3Config.cmake ]; then
    echo "错误：未找到 GLFW 安装，请先运行 scripts/render/fetch.sh 与 scripts/render/build.sh（M1-02）" >&2
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
    src="engine/input/src/input_system.cpp"
    backup="$out/input_system.cpp.orig"
    cp "$src" "$backup"
    restore() { cp "$backup" "$src"; }
    trap 'restore; echo "已恢复 $src"; exit 130' INT TERM
    cmake -S . -B "$b" -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug -DTERRA_WARNINGS_AS_ERRORS=ON >"$log" 2>&1
    detected=0
    total=0
    while IFS= read -r name; do
        total=$((total + 1))
        restore
        if ! python3 - "$src" "$name" >>"$log" 2>&1 <<'PY'
import sys
path, name = sys.argv[1], sys.argv[2]
mutations = {
    "死区不重新归一化": (
        "const float t = std::min(1.0f, (m - inner) / (outer - inner));",
        "const float t = std::min(1.0f, m + 0.0f * inner * outer);"),
    "径向死区按分量判断": (
        "if (length < p.a || length <= 0.0f) return {};",
        "if (std::fabs(v.x) < p.a || std::fabs(v.y) < p.a || length <= 0.0f) return {};"),
    "断开设备不释放控件": (
        "    s.devices.erase(it);\n",
        "    (void)it;\n"),
    "动作表之间不消费": (
        "consumed.insert(consumes.begin(), consumes.end());",
        "(void)consumes;"),
    "帧内快速点按丢失": (
        "if (cs.value < kButtonThreshold && value >= kButtonThreshold) cs.tapped = true;",
        "if (false) cs.tapped = true;"),
    "最近变化仲裁失效": (
        "better = c.changedAt > best->changedAt ||",
        "better = false ||"),
    "忽略修饰键": (
        "if (!modifiersSatisfied(binding)) {",
        "if (false) {"),
    "焦点丢失不释放": (
        "entry.second.value = 0.0f;",
        "(void)entry;"),
}
old, new = mutations[name]
text = open(path, encoding="utf-8").read()
if text.count(old) != 1:
    sys.exit(f"变异 '{name}' 的锚点在源文件中出现 {text.count(old)} 次，源码可能已改动")
open(path, "w", encoding="utf-8").write(text.replace(old, new))
PY
        then
            summary+=("  变异 $name：注入失败（见 $log）")
            failed=1
            continue
        fi
        echo "---- 变异：$name" >>"$log"
        if ! cmake --build "$b" -j "$jobs" --target test_input_system >>"$log" 2>&1; then
            summary+=("  变异 $name：编译失败，未判定（见 $log）")
            failed=1
            continue
        fi
        if "$b/tests/test_input_system" >>"$log" 2>&1; then
            summary+=("  变异 $name：未被发现")
        else
            detected=$((detected + 1))
            summary+=("  变异 $name：已发现（$(sed -n "/^---- 变异：$name\$/,\$p" "$log" | grep -c '^FAILED') 项检查失败）")
        fi
    done <<'LIST'
死区不重新归一化
径向死区按分量判断
断开设备不释放控件
动作表之间不消费
帧内快速点按丢失
最近变化仲裁失效
忽略修饰键
焦点丢失不释放
LIST
    restore
    trap - INT TERM
    if cmp -s "$src" "$backup"; then
        record "变异测试" "$detected/$total 被发现；源文件已恢复" "$([ $detected = $total ] && echo 1 || echo 0)"
    else
        record "变异测试" "源文件恢复失败！请手动用 $backup 覆盖 $src" 0
    fi
fi

echo "== 5. 测试输出"
for t in input.system graphics.platform_window graphics.platform_no_display input.glfw_translation; do
    ctest --test-dir build/render-release -R "^${t//./\\.}\$" -V 2>/dev/null | grep -E "INFO|FAILED" | sed 's/^[0-9]*: //'
done

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
