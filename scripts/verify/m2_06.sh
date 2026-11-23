#!/usr/bin/env bash
# M2-06 基线机器验证：仿真时钟（固定步长、追赶上限、暂停与单步）、按步生效的命令队列、快照插值与 reset 屏障。
# PRD F-M2-02 的核心判据：同一控制序列在 30 / 60 / 144 / 240 Hz 与抖动帧间隔下物理轨迹逐位相同。
#
# 可在任意终端运行（本次没有新的可见窗口；M2-07 起恢复可见展示）。
# 依次执行（日志在 build/verify-m2-06/logs/，屏幕只打印结论）：
#   1. 依赖与素材校验
#   2. 渲染关闭 Debug：全量 CTest（预期 46/46，新增 simulation.logic、simulation.runner）
#   3. 渲染开启 Debug 与 Release + -Werror：全量 CTest（预期 64/64）
#   4. ASan/UBSan（渲染开启）
#   5. 变异测试：命令生效时机、暂停、追赶上限、reset 屏障、跨代次插值、最短弧、暂停中单步、暂停时的显示，
#      逐个注入 8 个已知错误（预期 8/8）
#   6. 输出：各帧率下的帧数与步数
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m2_06.sh [--jobs N] [--skip-asan] [--skip-mutation]
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
        -h|--help) sed -n '2,17p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/verify-m2-06"
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
    files=(engine/simulation/src/clock.cpp engine/simulation/src/interpolation.cpp engine/simulation/src/runner.cpp)
    for f in "${files[@]}"; do cp "$f" "$out/$(basename "$f").orig"; done
    restore() { for f in "${files[@]}"; do cp "$out/$(basename "$f").orig" "$f"; done; }
    trap 'restore; echo "已恢复被变异的源文件"; exit 130' INT TERM
    # 不开 -Werror：变异只改语义，不应因警告而无法判定
    cmake -S . -B "$b" -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug >"$log" 2>&1
    detected=0
    total=0
    while IFS='|' read -r name file target testname; do
        total=$((total + 1))
        restore
        if ! python3 - "$file" "$name" >>"$log" 2>&1 <<'PY'
import sys
path, name = sys.argv[1], sys.argv[2]
mutations = {
    "命令在帧开始时一次生效": (
        "        const std::uint64_t index = world_.stepIndex();",
        "        const std::uint64_t index = world_.stepIndex() + (steps - 1 - i);"),
    "暂停时仍推进": (
        "    if (paused_) {\n        accumulator_ = 0.0;",
        "    if (false) {\n        accumulator_ = 0.0;"),
    "超出追赶上限仍全部推进": (
        "        steps = maxSteps_;",
        "        (void)0;"),
    "reset 不清空命令队列": (
        "    queue_.clear();",
        "    (void)0;"),
    "跨运行代次仍插值": (
        "older.runEpoch == newer.runEpoch &&",
        "true &&"),
    "四元数不取最短弧": (
        "    if (qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z < 0)",
        "    if (false)"),
    "暂停中的单步被忽略": (
        "            return 1;",
        "            return 0;"),
    "暂停时显示插值而非最新快照": (
        "    if (!previous_ || clock_.paused()) {",
        "    if (!previous_) {"),
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
            n=$(sed -n "/^---- 变异：$name\$/,\$p" "$log" | grep -cE '^FAILED|不匹配|退出码不符')
            summary+=("  变异 $name：已发现（$n 项检查失败）")
        fi
    done <<'LIST'
命令在帧开始时一次生效|engine/simulation/src/runner.cpp|test_simulation_runner|simulation.runner
暂停时仍推进|engine/simulation/src/clock.cpp|test_simulation_logic|simulation.logic
超出追赶上限仍全部推进|engine/simulation/src/clock.cpp|test_simulation_logic|simulation.logic
reset 不清空命令队列|engine/simulation/src/runner.cpp|test_simulation_runner|simulation.runner
跨运行代次仍插值|engine/simulation/src/interpolation.cpp|test_simulation_logic|simulation.logic
四元数不取最短弧|engine/simulation/src/interpolation.cpp|test_simulation_logic|simulation.logic
暂停中的单步被忽略|engine/simulation/src/clock.cpp|test_simulation_logic|simulation.logic
暂停时显示插值而非最新快照|engine/simulation/src/runner.cpp|test_simulation_runner|simulation.runner
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
ctest --test-dir build/debug -R '^simulation\.runner$' -V 2>/dev/null | grep -E "INFO|FAILED" | sed 's/^[0-9]*: //'

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
