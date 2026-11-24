#!/usr/bin/env bash
# M4-03b 基线机器验证：2D 激光雷达（M4-01 设计稿 2026-11-24 评审通过；用户 2026-11-24 选定 TurtleBot 4 后由 P1 提前到 P0）——
# 物理接口的射线检测（PhysicsWorld::castRays，collision / visual 两种模式，忽略安装实体）；场景文件 sensors 中的 lidar2d；
# 扫描（terra_simulation 的 scanLidar，与 sensor_msgs/LaserScan 的约定一致，±inf 按 REP-117）；Headless --sensors 输出雷达。
# 3D 雷达（MID-360、L1 的扫描图案）不在本步，见开发计划 M4-09。
#
# 请在**桌面会话终端**运行（第 7 步打开雷达的俯视图）。
# 依次执行（日志在 build/verify-m4-03b/logs/，屏幕只打印结论）：
#   1. 依赖与素材校验
#   2. 渲染关闭 Debug：全量 CTest（预期 82/82，新增 simulation.lidar、headless.sensors_lidar；scene.sensors、scene.camera_model 修改）
#   3. 渲染开启 Debug 与 Release + -Werror：全量 CTest（预期 112/112）
#   4. ASan/UBSan（渲染开启）
#   5. 变异测试：不穿过显示用的几何体、不忽略安装实体、打到不参与碰撞的几何体、不限最大量程、角度增量、安装位姿、
#      最小量程、角度单位、Headless 的 inf 写法，逐个注入 9 个已知错误
#   6. 输出：雷达测试与 Headless 的一次扫描
#   7. 打开雷达的俯视图（红点命中，灰线没有命中，图中心为雷达，朝上为雷达的 0°）
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m4_03b.sh [--jobs N] [--skip-asan] [--skip-mutation] [--skip-show] [--mutation-only]
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
        -h|--help) sed -n '2,21p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/verify-m4-03b"
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
    files=(engine/physics/backends/mujoco/src/mujoco_world.cpp engine/simulation/src/lidar.cpp engine/scene/src/scene_instance.cpp
           apps/headless/cli.cpp)
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
    "不穿过显示用的几何体": (
        "            travelled = total + kStepPast;",
        "            break;"),
    "不忽略安装实体": (
        "        exclude = impl_->entityToBody[query.exclude.value - 1];",
        "        (void)0;"),
    "打到不参与碰撞的几何体": (
        "            if (query.hits == RayTargets::Visual || collides) {",
        "            if (true || collides) {"),
    "不限最大量程": (
        "            if (total > query.maxRange) break;",
        "            (void)0;"),
    "角度增量少除一": (
        "    const double increment = (lidar.angleMax - lidar.angleMin) / static_cast<double>(lidar.samples - 1);",
        "    const double increment = (lidar.angleMax - lidar.angleMin) / static_cast<double>(lidar.samples);"),
    "安装位姿丢失": (
        "    return terra::scene::multiply(snapshot.entities[lidar.entity.value - 1].pose, lidar.mount);",
        "    return snapshot.entities[lidar.entity.value - 1].pose;"),
    "近于最小量程不为负无穷": (
        "        else if (d < lidar.rangeMin) scan.ranges.push_back(-std::numeric_limits<float>::infinity());",
        "        else if (false) scan.ranges.push_back(-std::numeric_limits<float>::infinity());"),
    "角度没有换算为弧度": (
        "        lidar.angleMin = spec.angleMinDegrees * kRadiansPerDegree;",
        "        lidar.angleMin = spec.angleMinDegrees;"),
    "Headless 的 inf 不写成字符串": (
        "        if (std::isinf(r)) json += r > 0 ? \"\\\"inf\\\"\" : \"\\\"-inf\\\"\";",
        "        if (false) json += \"\";"),
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
            n=$(awk -v head="---- 变异：$name" '$0 == head {found = 1} found' "$log" | grep -cE 'FAILED: |FAILED |不匹配|退出码不符')
            summary+=("  变异 $name：已发现（$n 项检查失败）")
        fi
    done <<'LIST'
不穿过显示用的几何体|engine/physics/backends/mujoco/src/mujoco_world.cpp|test_lidar|simulation.lidar
不忽略安装实体|engine/physics/backends/mujoco/src/mujoco_world.cpp|test_lidar|simulation.lidar
打到不参与碰撞的几何体|engine/physics/backends/mujoco/src/mujoco_world.cpp|test_lidar|simulation.lidar
不限最大量程|engine/physics/backends/mujoco/src/mujoco_world.cpp|test_lidar|simulation.lidar
角度增量少除一|engine/simulation/src/lidar.cpp|test_lidar|simulation.lidar
安装位姿丢失|engine/simulation/src/lidar.cpp|test_lidar|simulation.lidar
近于最小量程不为负无穷|engine/simulation/src/lidar.cpp|test_lidar|simulation.lidar
角度没有换算为弧度|engine/scene/src/scene_instance.cpp|test_lidar|simulation.lidar
Headless 的 inf 不写成字符串|apps/headless/cli.cpp|terra_headless|headless.sensors_lidar
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
ctest --test-dir build/debug -R '^simulation\.lidar$' -V 2>/dev/null | grep -E "INFO|FAILED|all passed" | sed 's/^[0-9]*: /  /'
headless=build/debug/apps/headless/terra_headless
"$headless" --scene assets/tests/sensors/lidar_room.terra-scene.json --steps 1 --sensors >"$logs/lidar.jsonl" 2>"$logs/lidar.err"
rc=$?
grep -E '"type":"lidar","name":"bench/scan",' "$logs/lidar.jsonl" | head -1 | cut -c1-240 | sed 's/^/  /'
record "Headless --sensors（雷达）" "退出码 $rc，$(grep -c '"type":"lidar"' "$logs/lidar.jsonl") 次扫描" "$(ok $rc)"
python3 scripts/sensors/plot_lidar.py "$logs/lidar.jsonl" bench/scan "$out/lidar_scan.png" --range 3 2>&1 | sed 's/^/  /'
python3 scripts/sensors/plot_lidar.py "$logs/lidar.jsonl" bench/scan_visual "$out/lidar_scan_visual.png" --range 3 2>&1 | sed 's/^/  /'

echo "== 7. 雷达的俯视图"
if [ $skip_show = 1 ]; then
    note "俯视图" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "俯视图" "未执行：不在桌面会话中（图片在 $out/）"
elif ! command -v xdg-open >/dev/null 2>&1; then
    note "俯视图" "没有 xdg-open，请手动打开 $out/lidar_scan.png"
else
    echo "  两张图：collision 模式（穿过只用于显示的柱子）与 visual 模式（打到柱子）。房间的三面墙为红点，敞开一侧为灰线："
    for png in "$out/lidar_scan.png" "$out/lidar_scan_visual.png"; do
        [ -f "$png" ] && { echo "    $png"; xdg-open "$png" >/dev/null 2>&1 & }
    done
    note "俯视图" "已用默认看图程序打开"
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
