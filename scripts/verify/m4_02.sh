#!/usr/bin/env bash
# M4-02 基线机器验证：IMU（M4-01 设计稿 2026-11-24 评审通过）——MJCF 中已有的 IMU 自动发现；场景文件 sensors 给实体或
# 安装点添加 IMU；快照修订 3 带传感器读数；按时间累计的采样时刻；Headless --sensors。
#
# 本步只有数值输出，没有可视的内容（传感器面板在 M4-04），因此不打开窗口；第 6 步打印读数供查看。
# 依次执行（日志在 build/verify-m4-02/logs/，屏幕只打印结论）：
#   1. 依赖与素材校验
#   2. 渲染关闭 Debug：全量 CTest（预期 78/78，新增 physics.imu、scene.sensors、simulation.sensor_sampling、
#      headless.sensors_imu、headless.sensors_with_rerun）
#   3. 渲染开启 Debug 与 Release + -Werror：全量 CTest（预期 107/107）
#   4. ASan/UBSan（渲染开启）
#   5. 变异测试：加速度与角速度对调、添加标记、不补缺的传感器、安装角、频率上限、场景频率、采样时刻、
#      Headless 只在第 0 步输出、Headless 不用场景频率，逐个注入 9 个已知错误
#   6. 输出：夹具的 IMU 读数（Headless --sensors）；G1 悬挂场景自带的两个 IMU（有参考机器人时）
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m4_02.sh [--jobs N] [--skip-asan] [--skip-mutation] [--mutation-only]
# --mutation-only：只做变异测试（阶段验收调用；不删除、不重建 build/ 下的预设构建目录，跳过 ASan）
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

out="$root/build/verify-m4-02"
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
    files=(engine/physics/backends/mujoco/src/mujoco_world.cpp engine/scene/src/scene_instance.cpp
           engine/simulation/src/sensor_sampling.cpp apps/headless/main.cpp)
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
    "加速度与角速度对调": (
        "            r.values[c] = acc[c];",
        "            r.values[c] = gyro[c];"),
    "场景添加的标记丢失": (
        "        entry.added = entry.added || nameOf(model.get(), mjOBJ_SENSOR, k).find(kHiddenNameMark) != std::string::npos;",
        "        (void)0;"),
    "已有安装点缺的传感器不补": (
        "        if (have) continue;",
        "        if (have || true) continue;"),
    "装在实体上的安装角丢失": (
        "        added->quat[1] = p.orientation.x;",
        "        (void)0;"),
    "频率不限制在物理频率以内": (
        "                config.rateHz = physicsRate;",
        "                (void)0;"),
    "场景写的频率不生效": (
        "            config.rateHz = *spec.rateHz;",
        "            (void)spec;"),
    "采样时刻每步都采": (
        "    return count(step) > count(step - 1);",
        "    return count(step) >= count(step - 1);"),
    "Headless 只在第 0 步输出读数": (
        "        ok = ok && emitSensors();",
        "        (void)0;"),
    "Headless 不用场景的频率": (
        "        if (c.sensor.valid() && c.sensor.value <= sensorRates.size()) sensorRates[c.sensor.value - 1] = c.rateHz;",
        "        (void)c;"),
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
加速度与角速度对调|engine/physics/backends/mujoco/src/mujoco_world.cpp|test_imu|physics.imu
场景添加的标记丢失|engine/physics/backends/mujoco/src/mujoco_world.cpp|test_scene_sensors|scene.sensors
已有安装点缺的传感器不补|engine/physics/backends/mujoco/src/mujoco_world.cpp|test_scene_sensors|scene.sensors
装在实体上的安装角丢失|engine/physics/backends/mujoco/src/mujoco_world.cpp|test_scene_sensors|scene.sensors
频率不限制在物理频率以内|engine/scene/src/scene_instance.cpp|test_scene_sensors|scene.sensors
场景写的频率不生效|engine/scene/src/scene_instance.cpp|test_scene_sensors|scene.sensors
采样时刻每步都采|engine/simulation/src/sensor_sampling.cpp|test_sensor_sampling|simulation.sensor_sampling
Headless 只在第 0 步输出读数|apps/headless/main.cpp|terra_headless|headless.sensors_imu
Headless 不用场景的频率|apps/headless/main.cpp|terra_headless|headless.sensors_imu
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
[ $mutation_only = 1 ] && { echo; printf '%s\n' "${summary[@]}"; exit $failed; }

echo "== 6. 输出"
ctest --test-dir build/debug -R '^(physics\.imu|scene\.sensors|simulation\.sensor_sampling)$' -V 2>/dev/null |
    grep -E "all passed|FAILED|跳过" | sed 's/^[0-9]*: /  /'
headless=build/debug/apps/headless/terra_headless
echo "  夹具 imu_added（Headless --sensors，前 0.01 s）："
"$headless" --scene assets/tests/sensors/imu_added.terra-scene.json --steps 5 --sample-every 5 --sensors \
    >"$logs/imu_added.jsonl" 2>"$logs/imu_added.err"
rc=$?
grep -E '"type":"sensor_list"' "$logs/imu_added.jsonl" | cut -c1-400 | sed 's/^/    /'
grep -E '"type":"sensor".*"step":5,' "$logs/imu_added.jsonl" | cut -c1-260 | sed 's/^/    /'
record "Headless --sensors（夹具）" "退出码 $rc，$(grep -c '"type":"sensor"' "$logs/imu_added.jsonl") 行读数" "$(ok $rc)"
if [ -f assets/robots/unitree_g1/g1.xml ]; then
    echo "  G1 悬挂场景自带的 IMU（1 s 后，无动作）："
    "$headless" --scene assets/scenes/robots/g1_suspended.terra-scene.json --duration 1 --sample-every 100000 --sensors \
        >"$logs/g1.jsonl" 2>"$logs/g1.err"
    rc=$?
    grep -E '"type":"sensor_list"' "$logs/g1.jsonl" | cut -c1-300 | sed 's/^/    /'
    grep -E '"type":"sensor".*"step":500,' "$logs/g1.jsonl" | cut -c1-300 | sed 's/^/    /'
    record "Headless --sensors（G1）" "退出码 $rc" "$(ok $rc)"
else
    note "Headless --sensors（G1）" "参考机器人未准备，跳过"
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
