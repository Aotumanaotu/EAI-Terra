#!/usr/bin/env bash
# M3-03 基线机器验证：机器人显示——MJCF 网格的平滑法线（折角保护）、纹理坐标、材质参数换算与二维贴图；
# 模型描述修订 4（网格法线与纹理坐标、材质、贴图）；参考图 mjcf_materials 与三台机器人。
#
# 请在**桌面会话终端**运行（第 7 步打开可见窗口）；自动化图形测试使用 Xvfb + llvmpipe（D-022）。
# 依次执行（日志在 build/verify-m3-03/logs/，屏幕只打印结论）：
#   1. 依赖与素材校验（含参考机器人）
#   2. 渲染关闭 Debug：全量 CTest（预期 52/52，新增 physics.visual_description）
#   3. 参考图：缺失的 mjcf_materials、robot_g1、robot_go2、robot_stretch3 在 llvmpipe 上首次生成（请查看后提交）；
#      渲染开启 Debug 与 Release + -Werror：全量 CTest（预期 78/78，另增 render.mjcf_visual 与 4 张参考图）
#   4. ASan/UBSan（渲染开启）
#   5. 变异测试：法线与纹理坐标的读取、贴图通道、材质下标、折角保护、粗糙度、纹理重复、贴图绑定，逐个注入 10 个已知错误
#   6. 输出：显示用描述（physics.visual_description）与参考图比较结果
#   7. 可交互展示（最长 180 秒）：查看器打开 G1；可再打开 Go2、Stretch 3 与材质夹具
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m3_03.sh [--jobs N] [--skip-asan] [--skip-mutation] [--skip-show] [--mutation-only]
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

out="$root/build/verify-m3-03"
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

echo "== 3. 参考图与渲染开启"
missing=()
for scene in mjcf_materials robot_g1 robot_go2 robot_stretch3; do
    [ -f "tests/render/reference/$scene.png" ] || missing+=("$scene")
done
if [ $mutation_only = 1 ]; then
    :
elif [ ${#missing[@]} = 0 ]; then
    note "参考图" "已存在（与之比较）"
else
    echo "（首次：在 llvmpipe 上生成参考图 ${missing[*]}）"
    bash scripts/verify/update_references.sh "${missing[@]}" >"$logs/update_references.txt" 2>&1
    rc=$?
    grep -E "^----|^  |\.png" "$logs/update_references.txt" | sed 's/^/  /'
    made=0
    for scene in "${missing[@]}"; do [ -f "tests/render/reference/$scene.png" ] && made=$((made + 1)); done
    record "首次生成参考图" "生成 $made/${#missing[@]} 张，退出码 $rc（请查看图片后提交）" "$([ $rc = 0 ] && [ $made = ${#missing[@]} ] && echo 1 || echo 0)"
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
    files=(engine/physics/backends/mujoco/src/mujoco_world.cpp apps/viewer/mjcf_visual.cpp apps/viewer/scene_view.cpp)
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
    "法线索引取成顶点索引": (
        "                const int* t = m->mesh_facenormal + 3 * static_cast<std::size_t>(faceadr + f);",
        "                const int* t = m->mesh_face + 3 * static_cast<std::size_t>(faceadr + f);"),
    "不读取纹理坐标": (
        "        if (texcoordadr >= 0 && texcoordnum > 0) {",
        "        if (false) {"),
    "三通道贴图透明度未补齐": (
        "out[3] = channels == 4 ? px[3] : std::uint8_t{255};",
        "out[3] = px[0];"),
    "几何体不记录材质": (
        "        geom.material = matid;",
        "        geom.material = -1;"),
    "折角保护失效": (
        "                if (degenerate || g[0] * face[0] + g[1] * face[1] + g[2] * face[2] >= creaseCos) {",
        "                if (true) {"),
    "网格不用平滑法线": (
        "    const bool smooth = !m.normals.empty() && m.triangleNormals.size() == m.triangles.size();",
        "    const bool smooth = false;"),
    "粗糙度不随 shininess": (
        "std::clamp(1.0f - 0.9f * m.shininess, 0.05f, 1.0f)",
        "0.55f"),
    "网格纹理坐标不乘重复次数": (
        "                mv.uv[0] = uv[0] * uvScale[0];",
        "                mv.uv[0] = uv[0];"),
    "有限平面不按尺寸归一化": (
        "    if (geom.shape == td::GeomShape::Plane && !m.texUniform && geom.size.x > 0 && geom.size.y > 0) {",
        "    if (false) {"),
    "显示时不绑定贴图": (
        "        if (textured) draw.material.baseColorTexture = view->texture(d, surface.texture);",
        "        (void)0;"),
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
法线索引取成顶点索引|engine/physics/backends/mujoco/src/mujoco_world.cpp|test_visual_description|physics.visual_description
不读取纹理坐标|engine/physics/backends/mujoco/src/mujoco_world.cpp|test_visual_description|physics.visual_description
三通道贴图透明度未补齐|engine/physics/backends/mujoco/src/mujoco_world.cpp|test_visual_description|physics.visual_description
几何体不记录材质|engine/physics/backends/mujoco/src/mujoco_world.cpp|test_visual_description|physics.visual_description
折角保护失效|apps/viewer/mjcf_visual.cpp|test_mjcf_visual|render.mjcf_visual
网格不用平滑法线|apps/viewer/mjcf_visual.cpp|test_mjcf_visual|render.mjcf_visual
粗糙度不随 shininess|apps/viewer/mjcf_visual.cpp|test_mjcf_visual|render.mjcf_visual
网格纹理坐标不乘重复次数|apps/viewer/mjcf_visual.cpp|test_mjcf_visual|render.mjcf_visual
有限平面不按尺寸归一化|apps/viewer/mjcf_visual.cpp|test_mjcf_visual|render.mjcf_visual
显示时不绑定贴图|apps/viewer/scene_view.cpp|test_render_reference|graphics.render_reference.mjcf_materials
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
ctest --test-dir build/debug -R '^physics\.visual_description$' -V 2>/dev/null | grep -E "INFO|FAILED|all passed" | sed 's/^[0-9]*: //'
for scene in mjcf_materials robot_g1 robot_go2 robot_stretch3; do
    echo "  参考图 $scene："
    ctest --test-dir build/render-release -R "^graphics\.render_reference\.${scene}\$" -V 2>/dev/null |
        grep -E "INFO 场景|INFO 与参考图|FAILED|跳过" | sed 's/^[0-9]*: /    /'
done
echo "  参考图文件（请用图片查看器打开确认后提交）：tests/render/reference/{mjcf_materials,robot_g1,robot_go2,robot_stretch3}.png"

viewer=build/render-release/apps/viewer/terra_viewer
echo "== 7. 可交互展示"
if [ $skip_show = 1 ]; then
    note "可交互展示" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "可交互展示" "未执行：不在桌面会话中"
elif [ -x "$viewer" ]; then
    echo "（即将打开 terra_viewer，最长 180 秒；设置文件：$out/show/settings.json）"
    echo "  打开的是宇树 G1（暂停，关键帧 stand）。与 M3-02 相比，请看："
    echo "    - 网格现在是平滑着色：圆柱形的关节外壳、大腿与躯干的曲面不再有一块块的面片；棱角处仍然分明"
    echo "    - 地面是 MuJoCo 场景自带的蓝灰色棋盘格（贴图）"
    echo "    - 视图 → 叠加显示碰撞几何体：显示 group 3 的碰撞体（G1 的脚底、躯干等处的简化形状）；"
    echo "      视图 → 几何体分组：可关掉 group 2（可见网格），只看碰撞体"
    echo "    - 文件 → 打开场景…：assets/scenes/robots/stretch3 —— 桌面是木纹贴图，机身上的二维码贴纸也有贴图；"
    echo "      assets/tests/scene/materials.terra-scene.json —— 材质夹具：棋盘格地面、带贴图的长方体与立方体网格、"
    echo "      发光的金色平滑球、金属胶囊、没有贴图的四面体与红色方块（不支持的立方体贴图，只显示颜色）"
    "$viewer" --scene assets/scenes/robots/g1.terra-scene.json --seconds 180 --settings "$out/show/settings.json" \
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
