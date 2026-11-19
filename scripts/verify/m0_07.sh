#!/usr/bin/env bash
# M0-07 基线机器验证：后端无关接口与执行器控制。
#
# 依次执行（每步的完整输出写入 build/verify-m0-07/logs/，屏幕只打印结论）：
#   1. Debug：删除 build/debug 后配置、构建、CTest（预期 30/30）
#   2. Release + -Werror：删除 build/release 后配置、构建、CTest（预期 30/30）
#   3. ASan/UBSan（含 LeakSanitizer）：build/verify-m0-07/asan，Debug + -Werror（预期 30/30）
#   4. 变异测试：向 MuJoCo 后端逐个注入 8 个已知错误，确认 physics.control 都能发现（预期 8/8）。
#      在独立构建目录 build/verify-m0-07/mutation 中进行；源文件先备份，结束（含中断）时恢复并校验。
#   5. 打印 hinge_motor.xml 的 model 记录与第 1000 步状态（u = 0 时应静止），供报告引用。
# 最后打印汇总；任一步失败时退出码非 0。
#
# 依赖：与 M0-06 相同（CMake、Ninja、GCC 11、third_party/mujoco/install）以及 python3（只用于变异注入）。
# 用法：scripts/verify/m0_07.sh [--skip-build] [--skip-asan] [--skip-mutation] [--jobs N]
#   --skip-build：跳过第 1–3 步（供 m0_acceptance.sh 复用变异测试与夹具输出；需已有 build/release）
set -uo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
cd "$root"

skip_build=0
skip_asan=0
skip_mutation=0
jobs=$(nproc)
while [ $# -gt 0 ]; do
    case "$1" in
        --skip-build) skip_build=1; skip_asan=1; shift ;;
        --skip-asan) skip_asan=1; shift ;;
        --skip-mutation) skip_mutation=1; shift ;;
        --jobs) jobs=$2; shift 2 ;;
        -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/verify-m0-07"
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

if [ $skip_build = 0 ]; then
    run_preset debug
    run_preset release -DTERRA_WARNINGS_AS_ERRORS=ON
fi

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
    "ctrl 按执行器下标写入": (
        "mjtNum* ctrl = w.data->ctrl + w.model->actuator_ctrladr[a];",
        "mjtNum* ctrl = w.data->ctrl + a;"),
    "范围按执行器下标读取": (
        "const auto u = static_cast<std::size_t>(ctrladr + k);",
        "const auto u = static_cast<std::size_t>(a + 0 * (ctrladr + k));"),
    "不检查范围（依赖后端钳位）": (
        "if (input.limited && (v < input.min || v > input.max)) {",
        "if (false) {"),
    "非原子（逐条校验逐条写入）": (
        "        seen[index] = true;\n",
        "        seen[index] = true;\n"
        "        for (std::size_t k = 0; k < command.values.size(); ++k) "
        "w.data->ctrl[static_cast<std::size_t>(w.model->actuator_ctrladr[index]) + k] = command.values[k];\n"),
    "不检查重复": (
        "        if (seen[index]) {",
        "        if (false) {"),
    "设置控制使快照失效": (
        "    // 写入 ctrl，下一次 mj_step 起生效。",
        "    w.cachedSnapshot.reset();\n    // 写入 ctrl，下一次 mj_step 起生效。"),
    "不拒绝带延迟的执行器": (
        "if (model->actuator_delay[a] > 0 || model->actuator_history[2 * a] > 0) {",
        "if (false) {"),
    "故障状态下允许设置控制": (
        "    if (w.faulted) {\n        return controlError(",
        "    if (false) {\n        return controlError("),
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
        if ! cmake --build "$b" -j "$jobs" --target test_physics_control >>"$log" 2>&1; then
            summary+=("  变异 $name：编译失败，未判定（见 $log）")
            failed=1
            continue
        fi
        if "$b/tests/test_physics_control" >>"$log" 2>&1; then
            summary+=("  变异 $name：未被发现")
        else
            detected=$((detected + 1))
            summary+=("  变异 $name：已发现（$(grep -c '^FAILED' <(sed -n "/^---- 变异：$name\$/,\$p" "$log")) 项检查失败）")
        fi
    done <<'LIST'
ctrl 按执行器下标写入
范围按执行器下标读取
不检查范围（依赖后端钳位）
非原子（逐条校验逐条写入）
不检查重复
设置控制使快照失效
不拒绝带延迟的执行器
故障状态下允许设置控制
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

echo "== hinge_motor 夹具输出"
headless="build/release/apps/headless/terra_headless"
if [ -x "$headless" ]; then
    "$headless" --model assets/tests/hinge_motor.xml --steps 1000 --sample-every 1000 >"$logs/hinge_motor.jsonl" 2>&1
    rc=$?
    python3 - "$logs/hinge_motor.jsonl" <<'PY'
import json, sys
records = [json.loads(line) for line in open(sys.argv[1], encoding="utf-8") if line.startswith("{")]
model = next(r for r in records if r["type"] == "model")
last = [r for r in records if r["type"] == "state"][-1]
print("actuators:", json.dumps(model["actuators"], ensure_ascii=False))
print(f"step {last['step']} joints:", json.dumps(last["joints"]))
PY
    record "hinge_motor Headless" "退出码 $rc" "$([ $rc = 0 ] && echo 1 || echo 0)"
else
    record "hinge_motor Headless" "未执行（release 构建失败）" 0
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
