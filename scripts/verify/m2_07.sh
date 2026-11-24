#!/usr/bin/env bash
# M2-07 基线机器验证：由物理快照驱动的场景渲染（MJCF 几何体、glTF 节点绑定的 node / assembly 对齐），场景参考图，
# 查看器打开场景文件（--scene）并运行仿真。
#
# 请在**桌面会话终端**运行（第 7 步打开可见窗口）；自动化图形测试使用 Xvfb + llvmpipe（D-022）。
# 依次执行（日志在 build/verify-m2-07/logs/，屏幕只打印结论）：
#   1. 依赖与素材校验
#   2. 渲染关闭 Debug：全量 CTest（预期 46/46）
#   3. 场景参考图：tests/render/reference/carts.png 不存在时先在 llvmpipe 上生成（首次，请查看后提交）；
#      然后渲染开启 Debug 与 Release + -Werror 全量 CTest（预期 65/65，新增 graphics.render_reference.carts；asset.gltf 扩充）
#   4. ASan/UBSan（渲染开启）
#   5. 变异测试：节点对齐、重复节点名、子树、颜色换算、几何体隐藏、视觉位姿旋转、透明几何体、assembly 对齐，
#      逐个注入 8 个已知错误（预期 8/8）
#   6. 输出：场景参考图的比较结果与渲染项统计
#   7. 可交互展示（最长 60 秒）：terra_viewer --scene two_carts --demo-drive，两辆小车在电机驱动下来回运动
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m2_07.sh [--jobs N] [--skip-asan] [--skip-mutation] [--skip-show] [--mutation-only]
# --mutation-only：只做变异测试（阶段验收调用；不删除、不重建 build/ 下的预设构建目录，跳过 ASan、参考图生成与真实 GPU 步骤）
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

out="$root/build/verify-m2-07"
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

echo "== 3. 场景参考图与渲染开启"
if [ -f tests/render/reference/carts.png ]; then
    note "场景参考图" "已存在：tests/render/reference/carts.png（与之比较）"
else
    echo "（首次：在 llvmpipe 上生成场景参考图 carts.png）"
    bash scripts/verify/update_references.sh carts >"$logs/update_references.txt" 2>&1
    rc=$?
    grep -E "^----|^  |\.png" "$logs/update_references.txt" | sed 's/^/  /'
    record "首次生成场景参考图" "$([ -f tests/render/reference/carts.png ] && echo "已生成" || echo "未生成")，退出码 $rc（请查看图片后提交）" "$(ok $rc)"
fi
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
    files=(engine/asset/src/gltf.cpp apps/viewer/scene_view.cpp apps/viewer/mjcf_visual.cpp)
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
    "Node 对齐不去掉节点变换": (
        "    const bool strip = frame == NodeFrame::Node && root >= 0",
        "    const bool strip = false && root >= 0"),
    "节点名重复不报错": (
        "    if (count > 1) {",
        "    if (false) {"),
    "子树判断忽略父链": (
        "        n = nodes[static_cast<std::size_t>(n)].parent;",
        "        n = -1;"),
    "MJCF 颜色未换算为线性": (
        "    return c <= 0.04045f ? c / 12.92f : std::pow((c + 0.055f) / 1.055f, 2.4f);",
        "    return c;"),
    "绑定了 glTF 的实体仍显示几何体": (
        "view->hideGeomsOf_[e] = anyLive[e] && !keep[e];",
        "view->hideGeomsOf_[e] = false;"),
    "视觉位姿忽略旋转": (
        "    const double z = n > 0 ? q.z / n : 0.0;",
        "    const double z = 0.0;"),
    "透明几何体仍显示": (
        "        if (g.rgba[3] < kHiddenAlpha) continue;",
        "        (void)kHiddenAlpha;"),
    "assembly 对齐被当作 node": (
        "                draw.frame = other.spec.frame == sc::VisualFrame::Assembly ? as::NodeFrame::Assembly : as::NodeFrame::Node;",
        "                draw.frame = as::NodeFrame::Node;"),
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
            n=$(sed -n "/^---- 变异：$name\$/,\$p" "$log" | grep -cE 'FAILED: |不匹配|退出码不符')
            summary+=("  变异 $name：已发现（$n 项检查失败）")
        fi
    done <<'LIST'
Node 对齐不去掉节点变换|engine/asset/src/gltf.cpp|test_asset_gltf|asset.gltf
节点名重复不报错|engine/asset/src/gltf.cpp|test_asset_gltf|asset.gltf
子树判断忽略父链|engine/asset/src/gltf.cpp|test_asset_gltf|asset.gltf
MJCF 颜色未换算为线性|apps/viewer/mjcf_visual.cpp|test_render_reference|graphics.render_reference.carts
绑定了 glTF 的实体仍显示几何体|apps/viewer/scene_view.cpp|test_render_reference|graphics.render_reference.carts
视觉位姿忽略旋转|apps/viewer/scene_view.cpp|test_render_reference|graphics.render_reference.carts
透明几何体仍显示|apps/viewer/scene_view.cpp|test_render_reference|graphics.render_reference.carts
assembly 对齐被当作 node|apps/viewer/scene_view.cpp|test_render_reference|graphics.render_reference.carts
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
ctest --test-dir build/render-release -R '^graphics\.render_reference\.carts$' -V 2>/dev/null | grep -E "INFO|FAILED" | sed 's/^[0-9]*: //'

viewer=build/render-release/apps/viewer/terra_viewer
echo "== 7. 可交互展示"
if [ $skip_show = 1 ]; then
    note "可交互展示" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "可交互展示" "未执行：不在桌面会话中"
elif [ -x "$viewer" ]; then
    echo "（即将打开 terra_viewer，最长 60 秒；设置文件：$out/show/settings.json）"
    echo "  请看：地面上两辆小车由物理驱动来回运动（正弦电机控制）——"
    echo "    左车 cart_a：蓝色车身与钢色桅杆来自 glTF（MJCF 几何体隐藏）；右车 cart_b：glTF 车身与 MJCF 几何体同时显示（showGeoms），"
    echo "    并且转了 90°，沿另一个方向运动；远处 x = 2 处是一套不随物理运动的静态 glTF 装饰"
    echo "  可以用中键转视角、滚轮拉近观察；画面应平滑（快照插值），没有跳动"
    "$viewer" --scene assets/tests/scene/two_carts.terra-scene.json --demo-drive --seconds 60 \
        --settings "$out/show/settings.json" >"$logs/show.txt" 2>&1
    rc=$?
    grep -E "^INFO|^错误|^警告" "$logs/show.txt"
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
