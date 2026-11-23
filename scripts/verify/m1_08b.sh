#!/usr/bin/env bash
# M1-08b 基线机器验证：基于图像的光照（HDRI 环境、辐照度、预过滤镜面、DFG 表、环境背景）与属性滑块改版。
#
# 请在**桌面会话终端**运行（第 6、7 步打开可见窗口）；自动化图形测试自己使用 Xvfb + llvmpipe（D-022）。
# 依次执行（日志在 build/verify-m1-08b/logs/，屏幕只打印结论）：
#   1. 依赖：nlohmann/json 与 stb_image 校验（离线 sha256）；渲染依赖（已有跳过）；HDRI 贴图（已有且一致则跳过）
#   2. 渲染关闭 Debug：删除 build/debug 后全量 CTest（预期 38/38）
#   3. 渲染开启 Debug 与 Release + -Werror：删除构建目录后全量 CTest（预期 50/50；render.math 与 graphics.render_pbr 扩充）
#   4. ASan/UBSan（渲染开启，Debug + -Werror）：非图形测试开启 LeakSanitizer；图形测试关闭泄漏检测
#   5. 变异测试：向辐照度积分、立方体面方向、等距柱状映射、环境旋转、背景深度、DFG 表逐个注入 6 个已知错误（预期 6/6）
#   6. 真实 GPU：查看器 40 帧自检（必须通过，含室外环境贴图）；PBR 像素测试在 NVIDIA 上运行一次（仅记录）
#   7. 可交互展示（最长 120 秒）：环境贴图光照与背景；新的属性滑块
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m1_08b.sh [--jobs N] [--skip-asan] [--skip-mutation] [--skip-show] [--mutation-only]
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

out="$root/build/verify-m1-08b"
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
record "stb_image 校验" "$(grep -c '^一致' "$logs/stb.txt")/2 个文件一致，退出码 $rc" "$(ok $rc)"
bash scripts/render/fetch.sh >"$logs/fetch.txt" 2>&1
rc=$?
record "获取并校验渲染依赖" "退出码 $rc" "$(ok $rc)"
bash scripts/assets/fetch_hdri.sh >"$logs/hdri.txt" 2>&1
rc=$?
record "HDRI 贴图" "$(grep -cE '^(已存在且一致|就绪)' "$logs/hdri.txt")/2 张一致，退出码 $rc" "$(ok $rc)"
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
    files=(engine/render/src/environment.cpp engine/render/src/pbr.cpp engine/render/src/scene_renderer.cpp)
    for f in "${files[@]}"; do cp "$f" "$out/$(basename "$f").orig"; done
    restore() { for f in "${files[@]}"; do cp "$out/$(basename "$f").orig" "$f"; done; }
    trap 'restore; echo "已恢复被变异的源文件"; exit 130' INT TERM
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
    "辐照度积分漏乘 cosθ": (
        "        float weight = cos(theta) * sin(theta);",
        "        float weight = sin(theta);"),
    "立方体 +Z 面方向写反": (
        "    else if (uFace == 4) d = vec3(c.x, -c.y, 1.0);",
        "    else if (uFace == 4) d = vec3(c.x, -c.y, -1.0);"),
    "等距柱状经度偏了半圈": (
        "    vec2 uv = vec2(0.5 + atan(d.y, d.x) / (2.0 * PI), acos(clamp(d.z, -1.0, 1.0)) / PI);",
        "    vec2 uv = vec2(atan(d.y, d.x) / (2.0 * PI), acos(clamp(d.z, -1.0, 1.0)) / PI);"),
    "环境旋转方向反了": (
        "    return vec3(uEnvRotation.x * d.x + uEnvRotation.y * d.y, -uEnvRotation.y * d.x + uEnvRotation.x * d.y, d.z);",
        "    return vec3(uEnvRotation.x * d.x - uEnvRotation.y * d.y, uEnvRotation.y * d.x + uEnvRotation.x * d.y, d.z);"),
    "背景不在远平面（遮住物体）": (
        "    gl_Position = vec4(p * 2.0 - 1.0, 1.0, 1.0);",
        "    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);"),
    "DFG 表忽略菲涅耳项": (
        "                a += (1.0f - fc) * gv;",
        "                a += gv;"),
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
        runner=()
        args=()
        if [ "$target" = test_render_pbr ]; then runner=(graphics_env); args=(llvmpipe); fi
        [ "$target" = test_render_math ] && args=("$out/mutation_render_math_tmp")
        if "${runner[@]}" "$b/tests/$target" "${args[@]}" >>"$log" 2>&1; then
            summary+=("  变异 $name：未被发现")
        else
            detected=$((detected + 1))
            summary+=("  变异 $name：已发现（$(sed -n "/^---- 变异：$name\$/,\$p" "$log" | grep -c '^FAILED') 项检查失败）")
        fi
    done <<'LIST'
辐照度积分漏乘 cosθ|engine/render/src/environment.cpp|test_render_pbr
立方体 +Z 面方向写反|engine/render/src/environment.cpp|test_render_pbr
等距柱状经度偏了半圈|engine/render/src/environment.cpp|test_render_pbr
环境旋转方向反了|engine/render/src/pbr.cpp|test_render_pbr
背景不在远平面（遮住物体）|engine/render/src/scene_renderer.cpp|test_render_pbr
DFG 表忽略菲涅耳项|engine/render/src/pbr.cpp|test_render_math
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

echo "== Xvfb 输出（llvmpipe）"
ctest --test-dir build/render-release -R '^graphics\.render_pbr$' -V 2>/dev/null | grep -E "INFO|FAILED" | sed 's/^[0-9]*: //'
ctest --test-dir build/render-release -R '^graphics\.viewer_selftest$' -V 2>/dev/null | grep -E "INFO 视口|INFO 已加载|FAILED|渲染|阴影|环境" | sed 's/^[0-9]*: //'
ctest --test-dir build/render-release -R '^render\.math$' -V 2>/dev/null | grep -E "INFO|FAILED" | sed 's/^[0-9]*: //'

viewer=build/render-release/apps/viewer/terra_viewer
echo "== 6. 真实 GPU（桌面会话）"
if [ $skip_show = 1 ]; then
    note "真实 GPU 自检" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "真实 GPU 自检" "未执行：不在桌面会话中"
elif [ -x "$viewer" ]; then
    "$viewer" --frames 40 --self-test --settings "$out/selftest/settings.json" >"$logs/desktop_selftest.txt" 2>&1
    rc=$?
    grep -E "^INFO 视口|^INFO NVIDIA|^INFO 已加载|^PASS 视口|^PASS 绘制|^PASS 室外|^PASS 设置的默认渲染|^FAILED" "$logs/desktop_selftest.txt"
    record "真实 GPU 自检" "退出码 $rc" "$(ok $rc)"
    pbr_test=build/render-release/tests/test_render_pbr
    if [ -x "$pbr_test" ]; then
        "$pbr_test" >"$logs/desktop_pbr.txt" 2>&1
        rc=$?
        grep -E "^INFO|^FAILED" "$logs/desktop_pbr.txt" | head -40
        note "PBR 像素测试（真实 GPU，仅记录）" "退出码 $rc（$(grep -c '^FAILED' "$logs/desktop_pbr.txt") 项与 llvmpipe 基准不符）"
    fi
else
    record "真实 GPU 自检" "未执行（render-release 构建失败）" 0
fi

echo "== 7. 可交互展示"
if [ $skip_show = 1 ]; then
    note "可交互展示" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "可交互展示" "未执行：不在桌面会话中"
elif [ -x "$viewer" ]; then
    echo "（即将打开 terra_viewer，最长 120 秒；设置文件：$out/show/settings.json）"
    echo "  请看：默认为室外环境贴图光照（rural_asphalt_road），视口背景为模糊的环境照片；金属球反射出真实环境"
    echo "  请试：属性面板“渲染”分组——环境光切换 室外 / 摄影棚 / 半球光，拖动环境旋转（金属球的反射随之转动），"
    echo "        调环境强度、背景模糊，关闭背景；滑块已改为 Blender 风格（名称在条内左侧、数值在右侧，Ctrl+点击可输入）"
    echo "        偏好设置（Ctrl+,）的相机与界面页也已换成新滑块"
    "$viewer" --seconds 120 --settings "$out/show/settings.json" >"$logs/show.txt" 2>&1
    rc=$?
    grep -E "^INFO" "$logs/show.txt"
    frames=$(sed -n 's/^INFO 结束：\([0-9]*\) 帧，\([0-9.]*\) 秒$/\1 \2/p' "$logs/show.txt")
    if [ -n "$frames" ]; then
        set -- $frames
        note "展示平均帧率" "$(awk -v f="$1" -v t="$2" 'BEGIN { if (t > 0) printf "%.1f FPS（%d 帧 / %.1f 秒，含垂直同步）", f / t, f, t }')"
    fi
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
