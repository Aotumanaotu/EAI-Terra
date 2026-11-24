#!/usr/bin/env bash
# M3-04b 基线机器验证：PD 控制层（场景文件 controllers）、悬挂模式（models[].fixedBase）、快照的执行器状态（修订 2）；
# Go2 场景改由 PD 层驱动；新增 G1 悬挂场景。
#
# 请在**桌面会话终端**运行（第 7 步打开可见窗口）。
# 依次执行（日志在 build/verify-m3-04b/logs/，屏幕只打印结论）：
#   1. 依赖与素材校验（含参考机器人）
#   2. 渲染关闭 Debug：全量 CTest（预期 56/56，新增 simulation.pd、physics.fixed_base）
#   3. 渲染开启 Debug 与 Release + -Werror：全量 CTest（预期 82/82）
#   4. ASan/UBSan（渲染开启）
#   5. 变异测试：PD 换算（传动比、截断、阻尼符号）、保持姿态、命令分流、场景校验、悬挂模式、快照，逐个注入 9 个已知错误
#   6. 输出：Go2 在 PD 层下的机身高度、G1 悬挂场景的核查（robots.import）
#   7. 可交互展示（最长 180 秒）：查看器打开 Go2（PD 层保持 home 姿态）；可再打开 G1 悬挂场景
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m3_04b.sh [--jobs N] [--skip-asan] [--skip-mutation] [--skip-show] [--mutation-only]
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

out="$root/build/verify-m3-04b"
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
    files=(engine/simulation/src/pd_layer.cpp engine/scene/src/scene_instance.cpp engine/physics/backends/mujoco/src/mujoco_world.cpp)
    for f in "${files[@]}"; do cp "$f" "$out/${f//\//_}.orig"; done
    restore() { for f in "${files[@]}"; do cp "$out/${f//\//_}.orig" "$f"; done; }
    trap 'restore; echo "已恢复被变异的源文件"; exit 130' INT TERM
    # 不开 -Werror：变异只改语义，不应因警告而无法判定
    cmake -S . -B "$b" -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug -DTERRA_BUILD_RENDER=OFF >"$log" 2>&1
    detected=0
    total=0
    while IFS='|' read -r name file target testname; do
        total=$((total + 1))
        restore
        if ! python3 - "$file" "$name" >>"$log" 2>&1 <<'PY'
import sys
path, name = sys.argv[1], sys.argv[2]
mutations = {
    "PD 不除以传动比": (
        "        const double scale = act->gain * act->gear;",
        "        const double scale = act->gain;"),
    "PD 不截断到控制范围": (
        "    if (c.limited) u = std::clamp(u, c.min, c.max);",
        "    (void)0;"),
    "PD 阻尼项符号相反": (
        "    const double torque = c.kp * (target - q) - c.kd * qd;",
        "    const double torque = c.kp * (target - q) + c.kd * qd;"),
    "没有目标时不保持当前姿态": (
        "        if (!c.target) c.target = js.position[0];",
        "        if (!c.target) c.target = 0.0;"),
    "PD 层的目标被直接下发": (
        "        if (it == channels_.end()) {\n            out.push_back(std::move(command));",
        "        if (true) {\n            out.push_back(std::move(command));"),
    "PD 层接管非力矩执行器不报错": (
        "                if (!usable) {",
        "                if (false) {"),
    "悬挂模式不删除自由关节": (
        "        if (!models[i].fixedBase) continue;",
        "        if (true) continue;"),
    "删除前不先编译": (
        "        ModelPtr before{mj_compile(children[i].get(), nullptr)};\n        if (!before) {",
        "        if (false) {"),
    "快照不填控制量": (
        "        for (int k = 0; k < m->actuator_ctrlnum[a]; ++k) as.controls.push_back(ctrl[k]);",
        "        (void)ctrl;"),
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
PD 不除以传动比|engine/simulation/src/pd_layer.cpp|test_pd_layer|simulation.pd
PD 不截断到控制范围|engine/simulation/src/pd_layer.cpp|test_pd_layer|simulation.pd
PD 阻尼项符号相反|engine/simulation/src/pd_layer.cpp|test_pd_layer|simulation.pd
没有目标时不保持当前姿态|engine/simulation/src/pd_layer.cpp|test_pd_layer|simulation.pd
PD 层的目标被直接下发|engine/simulation/src/pd_layer.cpp|test_pd_layer|simulation.pd
PD 层接管非力矩执行器不报错|engine/scene/src/scene_instance.cpp|test_pd_layer|simulation.pd
悬挂模式不删除自由关节|engine/physics/backends/mujoco/src/mujoco_world.cpp|test_fixed_base|physics.fixed_base
删除前不先编译|engine/physics/backends/mujoco/src/mujoco_world.cpp|test_fixed_base|physics.fixed_base
快照不填控制量|engine/physics/backends/mujoco/src/mujoco_world.cpp|test_pd_layer|simulation.pd
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
ctest --test-dir build/debug -R '^robots\.import$' -V 2>/dev/null | grep -E "\[go2\] PD|g1_suspended|FAILED|跳过" | sed 's/^[0-9]*: /  /'
ctest --test-dir build/debug -R '^physics\.fixed_base$' -V 2>/dev/null | grep -E "INFO|FAILED|all passed" | sed 's/^[0-9]*: /  /'

viewer=build/render-release/apps/viewer/terra_viewer
echo "== 7. 可交互展示"
if [ $skip_show = 1 ]; then
    note "可交互展示" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "可交互展示" "未执行：不在桌面会话中"
elif [ -x "$viewer" ]; then
    echo "（即将打开 terra_viewer，最长 180 秒；设置文件：$out/show/settings.json）"
    echo "  打开的是宇树 Go2（暂停，关键帧 home）。与 M3-02 相比，请看："
    echo "    - 按空格播放：PD 层（kp 50、kd 0.5）以 home 的关节角为目标保持姿态，机器狗应稳稳站住，不再塌下去"
    echo "    - Ctrl+R 重置后再播放，结果相同"
    echo "    - 文件 → 打开场景…：assets/scenes/robots/g1_suspended.terra-scene.json —— G1 悬挂模式：基座固定在离地 0.3 m，"
    echo "      按空格播放，身体不会倒，四肢由位置执行器保持 stand 姿态（执行器面板与动作在 M3-04c）"
    "$viewer" --scene assets/scenes/robots/go2.terra-scene.json --seconds 180 --settings "$out/show/settings.json" \
        >"$logs/show.txt" 2>&1
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
