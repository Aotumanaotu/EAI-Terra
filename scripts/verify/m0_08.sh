#!/usr/bin/env bash
# M0-08 基线机器验证：物理场景、冻结容差与辅助用例。
#
# 整个脚本在清除 DISPLAY / WAYLAND_DISPLAY 的环境中运行（无显示会话构建与测试，开发计划 4.3 节）。
# 依次执行（每步的完整输出写入 build/verify-m0-08/logs/，屏幕只打印结论）：
#   1. Debug：删除 build/debug 后配置、构建、CTest（M0-08 时 34 项，M0-09 起 35 项）
#   2. Release + -Werror：删除 build/release 后配置、构建、CTest
#   3. ASan/UBSan（含 LeakSanitizer）：build/verify-m0-08/asan，Debug + -Werror
#   4. 变异测试：向 MuJoCo 后端逐个注入 3 个已知错误，确认 physics.scenarios 都能发现（预期 3/3）。
#      在独立构建目录中进行；源文件先备份，结束（含中断）时恢复并校验。
#   5. 用 Release 构建运行 physics.scenarios 与 physics.continuous，打印全部 METRIC / MEMORY 行，供冻结容差引用。
# 最后打印汇总；任一步失败时退出码非 0。
#
# 依赖：与 M0-07 相同（CMake、Ninja、GCC 11、third_party/mujoco/install、python3）。
# 用法：scripts/verify/m0_08.sh [--skip-asan] [--skip-mutation] [--jobs N]
set -uo pipefail

if [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    exec env -u DISPLAY -u WAYLAND_DISPLAY bash "$0" "$@"
fi

root=$(cd "$(dirname "$0")/../.." && pwd)
cd "$root"

skip_asan=0
skip_mutation=0
jobs=$(nproc)
while [ $# -gt 0 ]; do
    case "$1" in
        --skip-asan) skip_asan=1; shift ;;
        --skip-mutation) skip_mutation=1; shift ;;
        --jobs) jobs=$2; shift 2 ;;
        -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/verify-m0-08"
logs="$out/logs"
rm -rf "$out"
mkdir -p "$logs"

summary=()
failed=0
record() {  # record <名称> <结果文字> <是否通过 0/1>
    summary+=("$(printf '%-28s %s' "$1" "$2")")
    [ "$3" = 1 ] || failed=1
}

# 从 CTest 日志中提取 “100% tests passed, 0 tests failed out of 30” 这一行
ctest_result() { grep -E "tests passed|tests failed" "$1" | tail -1; }

echo "== 环境"
{
    uname -a
    g++ --version | head -1
    cmake --version | head -1
    ninja --version
    grep MUJOCO_VERSION= third_party/mujoco/mujoco.lock
    echo "DISPLAY='${DISPLAY:-}' WAYLAND_DISPLAY='${WAYLAND_DISPLAY:-}'（应为空）"
} | tee "$logs/env.txt"

run_preset() {  # run_preset <preset> [额外配置参数...]
    local preset=$1; shift
    local log="$logs/$preset.txt"
    echo "== $preset"
    rm -rf "build/$preset"
    if cmake --preset "$preset" "$@" >"$log" 2>&1 &&
       cmake --build --preset "$preset" -j "$jobs" >>"$log" 2>&1; then
        if grep -q "warning:" "$log"; then
            record "$preset 构建" "有编译警告（见 $log）" 0
        fi
        ctest --preset "$preset" >>"$log" 2>&1
        local rc=$?
        record "$preset 测试" "$(ctest_result "$log")" "$([ $rc = 0 ] && echo 1 || echo 0)"
    else
        record "$preset 构建" "失败（见 $log）" 0
    fi
    echo "${summary[-1]}"
}

run_preset debug
run_preset release -DTERRA_WARNINGS_AS_ERRORS=ON

if [ $skip_asan = 0 ]; then
    echo "== asan"
    log="$logs/asan.txt"
    b="$out/asan"
    san="-fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer"
    if cmake -S . -B "$b" -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug \
             -DTERRA_WARNINGS_AS_ERRORS=ON "-DCMAKE_CXX_FLAGS=$san" "-DCMAKE_EXE_LINKER_FLAGS=$san" >"$log" 2>&1 &&
       cmake --build "$b" -j "$jobs" >>"$log" 2>&1; then
        ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=print_stacktrace=1 \
            ctest --test-dir "$b" --output-on-failure >>"$log" 2>&1
        rc=$?
        record "asan/ubsan 测试" "$(ctest_result "$log")" "$([ $rc = 0 ] && echo 1 || echo 0)"
    else
        record "asan/ubsan 构建" "失败（见 $log）" 0
    fi
    echo "${summary[-1]}"
fi

if [ $skip_mutation = 0 ]; then
    echo "== 变异测试"
    log="$logs/mutation.txt"
    b="$out/mutation"
    src="engine/physics/backends/mujoco/src/mujoco_world.cpp"
    backup="$out/mujoco_world.cpp.orig"
    cp "$src" "$backup"
    restore() { cp "$backup" "$src"; }
    trap 'restore; echo "已恢复 $src"; exit 130' INT TERM
    cmake -S . -B "$b" -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug \
          -DTERRA_WARNINGS_AS_ERRORS=ON >"$log" 2>&1
    detected=0
    total=0
    while IFS= read -r name; do
        total=$((total + 1))
        restore
        if ! python3 - "$src" "$name" >>"$log" 2>&1 <<'PY'
import sys
path, name = sys.argv[1], sys.argv[2]
mutations = {
    "世界实例号不递增": (
        "nextWorldInstance.fetch_add(1, std::memory_order_relaxed);",
        "nextWorldInstance.load(std::memory_order_relaxed);"),
    "快照不记录世界实例号": (
        "    snap->worldInstance = description.worldInstance;\n",
        "\n"),
    "reset 不递增运行代次": (
        "    ++w.runEpoch;\n",
        "\n"),
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
        if ! cmake --build "$b" -j "$jobs" --target test_physics_scenarios >>"$log" 2>&1; then
            summary+=("  变异 $name：编译失败，未判定（见 $log）")
            failed=1
            continue
        fi
        if "$b/tests/test_physics_scenarios" >>"$log" 2>&1; then
            summary+=("  变异 $name：未被发现")
        else
            detected=$((detected + 1))
            summary+=("  变异 $name：已发现（$(grep -c '^FAILED' <(sed -n "/^---- 变异：$name\$/,\$p" "$log")) 项检查失败）")
        fi
    done <<'LIST'
世界实例号不递增
快照不记录世界实例号
reset 不递增运行代次
LIST
    restore
    trap - INT TERM
    if cmp -s "$src" "$backup"; then
        record "变异测试" "$detected/$total 被发现；源文件已恢复" "$([ $detected = $total ] && echo 1 || echo 0)"
    else
        record "变异测试" "源文件恢复失败！请手动用 $backup 覆盖 $src" 0
    fi
    echo "变异测试 $detected/$total"
fi

echo "== 场景判定与内存走势（release）"
if [ -x build/release/tests/test_physics_scenarios ] && [ -x build/release/tests/test_physics_continuous ]; then
    build/release/tests/test_physics_scenarios >"$logs/scenarios.txt" 2>&1
    rc1=$?
    build/release/tests/test_physics_continuous >"$logs/continuous.txt" 2>&1
    rc2=$?
    grep -hE "^(INFO|METRIC|MEMORY|FAILED)" "$logs/scenarios.txt" "$logs/continuous.txt"
    record "场景判定输出" "scenarios 退出码 $rc1，continuous 退出码 $rc2" "$([ $rc1 = 0 ] && [ $rc2 = 0 ] && echo 1 || echo 0)"
else
    record "场景判定输出" "未执行（release 构建失败）" 0
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
