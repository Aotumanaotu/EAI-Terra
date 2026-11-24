#!/usr/bin/env bash
# M4-03a 基线机器验证：离屏渲染与相机（M4-01 设计稿 2026-11-24 评审通过）——线性深度渲染（米，沿光轴，无命中为 +inf）；
# MJCF 相机（模型描述修订 7）；场景文件 sensors 中的相机（MJCF 相机或装在实体上）；相机模型（内参、光学坐标系、投影）；
# 按快照渲染彩色与深度（terra_viewer_scene 的 CameraRenderer）；Headless 遇到相机时提示并忽略。
#
# 请在**桌面会话终端**运行（第 7 步打开相机拍到的图片）；自动化图形测试使用 Xvfb + llvmpipe（D-022）。
# 依次执行（日志在 build/verify-m4-03a/logs/，屏幕只打印结论）：
#   1. 依赖与素材校验
#   2. 渲染关闭 Debug：全量 CTest（预期 80/80，新增 scene.camera_model、headless.cameras_ignored；scene.sensors 修改）
#   3. 渲染开启 Debug 与 Release + -Werror：全量 CTest（预期 110/110，另增 graphics.camera_capture）
#   4. ASan/UBSan（渲染开启）
#   5. 变异测试：无命中不为 +inf、深度行序、主点、投影的 v 方向、观察矩阵、宽高比、MJCF 视场角、颜色通道、
#      实体上的安装位姿、深度渲染器不清除，逐个注入 10 个已知错误
#   6. 输出：相机测试的读数（深度误差、投影误差、Stretch 3 头部相机）
#   7. 打开相机拍到的图片（彩色 | 深度伪彩色）：夹具与 Stretch 3 头部相机（有参考机器人时）
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m4_03a.sh [--jobs N] [--skip-asan] [--skip-mutation] [--skip-show] [--mutation-only]
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
        -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/verify-m4-03a"
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
    files=(engine/render/src/depth_renderer.cpp engine/scene/src/camera_model.cpp apps/viewer/camera_sensor.cpp
           engine/physics/backends/mujoco/src/mujoco_world.cpp engine/scene/src/scene_instance.cpp)
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
    "没有命中时不为 +inf": (
        "            depth[row * w + col] = v > 0.0f ? v : std::numeric_limits<float>::infinity();",
        "            depth[row * w + col] = v;"),
    "深度图行序没有翻转": (
        "            const float v = bottomUp[(h - 1 - row) * w + col];",
        "            const float v = bottomUp[row * w + col];"),
    "主点取宽度的一半": (
        "    k.cx = 0.5 * static_cast<double>(width - 1);",
        "    k.cx = 0.5 * static_cast<double>(width);"),
    "投影的 v 方向反了": (
        "    const double y = -p.y;",
        "    const double y = p.y;"),
    "观察矩阵没有取逆": (
        "    params.view = poseMatrix(sc::inverse(cameraWorld));",
        "    params.view = poseMatrix(cameraWorld);"),
    "投影矩阵不除宽高比": (
        "    p[0] = static_cast<float>(f / aspect);",
        "    p[0] = static_cast<float>(f);"),
    "MJCF 相机的视场角丢失": (
        "        camera.fovyDegrees = model->cam_fovy[c];",
        "        (void)0;"),
    "彩色通道顺序错误": (
        "                dst[0] = src[0];",
        "                dst[0] = src[2];"),
    "实体上的相机丢失安装位姿": (
        "            camera.mount = spec.pose;",
        "            (void)0;"),
    "深度渲染器不清除": (
        "    d.ctx.clear(Color{0.0f, 0.0f, 0.0f, 0.0f}, true);",
        "    (void)0;"),
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
没有命中时不为 +inf|engine/render/src/depth_renderer.cpp|test_camera_capture|graphics.camera_capture
深度图行序没有翻转|engine/render/src/depth_renderer.cpp|test_camera_capture|graphics.camera_capture
主点取宽度的一半|engine/scene/src/camera_model.cpp|test_camera_model|scene.camera_model
投影的 v 方向反了|engine/scene/src/camera_model.cpp|test_camera_model|scene.camera_model
观察矩阵没有取逆|apps/viewer/camera_sensor.cpp|test_camera_capture|graphics.camera_capture
投影矩阵不除宽高比|apps/viewer/camera_sensor.cpp|test_camera_capture|graphics.camera_capture
MJCF 相机的视场角丢失|engine/physics/backends/mujoco/src/mujoco_world.cpp|test_camera_model|scene.camera_model
彩色通道顺序错误|apps/viewer/camera_sensor.cpp|test_camera_capture|graphics.camera_capture
实体上的相机丢失安装位姿|engine/scene/src/scene_instance.cpp|test_camera_model|scene.camera_model
深度渲染器不清除|engine/render/src/depth_renderer.cpp|test_camera_capture|graphics.camera_capture
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
ctest --test-dir build/render-release -R '^graphics\.camera_capture$' -V 2>/dev/null | grep -E "INFO|FAILED|all passed|跳过" |
    sed 's/^[0-9]*: /  /'
ctest --test-dir build/debug -R '^scene\.camera_model$' -V 2>/dev/null | grep -E "FAILED|all passed" | sed 's/^[0-9]*: /  /'

pictures=build/render-release/tests/camera_capture
echo "== 7. 相机拍到的图片"
if [ $skip_show = 1 ]; then
    note "图片展示" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "图片展示" "未执行：不在桌面会话中（图片在 $pictures/）"
elif ! command -v xdg-open >/dev/null 2>&1; then
    note "图片展示" "没有 xdg-open，请手动打开 $pictures/ 中的 PNG"
else
    echo "  左边为彩色，右边为深度（红近蓝远，黑色为没有命中）："
    for png in "$pictures/camera_bench.png" "$pictures/stretch3_d435i.png"; do
        if [ -f "$png" ]; then
            echo "    $png"
            xdg-open "$png" >/dev/null 2>&1 &
        fi
    done
    note "图片展示" "已用默认看图程序打开（查看器中的传感器面板在 M4-04）"
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
