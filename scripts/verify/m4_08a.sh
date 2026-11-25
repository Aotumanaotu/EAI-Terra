#!/usr/bin/env bash
# M4-08a 基线机器验证：模型导入（M4-08 设计稿 2026-11-25 评审通过）——
# URDF / xacro / MJCF → Terra 场景：展开 xacro（随 Terra 提供的官方 xacro，不需要 ROS）、解析 package:// 与 $(find)、
# 转换 Collada 网格（assimp 命令行工具；按材质拆分，单位换算为米，坐标轴与 RViz 相同）、检查单位与惯量、提议执行器与传感器、
# 生成场景并加载检查。命令行 terra_import；查看器“文件 → 导入模型…”与拖放打开导入向导；首次启动窗口取显示器的 85%。
#
# 请在**桌面会话终端**运行（第 7 步打开查看器）。需要：
#   - assimp 命令行工具：sudo apt install assimp-utils
#   - 手动测试的公开示例（设计稿第 8.1 节；没有下载时 import.demos 跳过，展示中也没有这两个示例）：
#       mkdir -p ~/EAI-Terra/ROBOTS/import_demos && cd ~/EAI-Terra/ROBOTS/import_demos
#       git clone https://github.com/ros/urdf_tutorial.git && git -C urdf_tutorial checkout 050f1e47cfdb2c5f3eb0746bc15c57e6a870faef
#       git clone --filter=blob:none --sparse https://github.com/ROBOTIS-GIT/turtlebot3.git
#       cd turtlebot3 && git sparse-checkout set turtlebot3_description && git checkout 90a68bd2e3c61c12966779da89d8eeaec82730e9 && cd ..
# 依次执行（日志在 build/verify-m4-08a/logs/，屏幕只打印结论）：
#   1. 依赖与素材校验；第三方源码（tinyxml2、xacro）；assimp 与 python3
#   2. 渲染关闭 Debug：全量 CTest（预期 93/93，新增 import.analyze、import.execute、import.demos、import.cli_dry_run、import.cli_broken）
#   3. 渲染开启 Debug 与 Release + -Werror：全量 CTest（预期 128/128，另新增 graphics.viewer_import_selftest）
#   4. ASan/UBSan（渲染开启）
#   5. 变异测试：逐个注入 8 个已知错误
#   6. 输出：导入的 INFO 行（含公开示例）
#   7. 可交互展示（最长 300 秒）：查看器（首次启动，窗口取显示器的 85%），按提示导入 R2D2、TurtleBot 3 与出错的夹具
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m4_08a.sh [--jobs N] [--skip-asan] [--skip-mutation] [--skip-show] [--mutation-only]
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
        -h|--help) sed -n '2,27p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/verify-m4-08a"
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
bash scripts/import/check_third_party.sh >"$logs/import_third_party.txt" 2>&1
rc=$?
record "tinyxml2 与 xacro 校验" "$(grep -c '^一致' "$logs/import_third_party.txt")/9 个文件一致，退出码 $rc" "$(ok $rc)"
if command -v assimp >/dev/null; then
    record "assimp 命令行工具" "$(assimp version 2>/dev/null | grep -m1 -iE 'version|assimp' | cut -c1-60)" 1
else
    record "assimp 命令行工具" "没有找到：sudo apt install assimp-utils（import.execute 与查看器导入自检需要）" 0
fi
command -v python3 >/dev/null && record "python3（展开 xacro）" "$(python3 --version 2>&1)" 1 || record "python3" "没有找到" 0
if [ -d ROBOTS/import_demos/urdf_tutorial ] && [ -d ROBOTS/import_demos/turtlebot3 ]; then
    note "公开示例" "已下载（ROBOTS/import_demos/）"
else
    note "公开示例" "没有下载 ROBOTS/import_demos/（import.demos 跳过；下载命令见本脚本开头）"
fi
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
    files=(engine/import/src/mesh.cpp engine/import/src/importer.cpp apps/viewer/import_wizard.cpp apps/import/main.cpp)
    for f in "${files[@]}"; do cp "$f" "$out/${f//\//_}.orig"; done
    restore() { for f in "${files[@]}"; do cp "$out/${f//\//_}.orig" "$f"; done; }
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
    "Collada 不撤销 assimp 的旋转": (
        "            if (rotated) q = {p[0], -p[2], p[1]};",
        "            (void)rotated;"),
    "OBJ 不按材质拆分": (
        "            usePart(name);",
        "            (void)name;"),
    "package:// 不解析": (
        "    if (uri.rfind(pkgPrefix, 0) == 0) {",
        "    if (uri.rfind(pkgPrefix, 0) == 0 && false) {"),
    "不提示毫米单位": (
        "            if (b && b->extent() * scale > 5.0) {",
        "            if (false) {"),
    "轮子提议为位置执行器": (
        "        if (j.type == \"continuous\") {",
        "        if (false) {"),
    "xacro 不传包目录": (
        "            args.push_back(name + \"=\" + dir);",
        "            args.push_back(name + \"=/nonexistent\");"),
    "导入后不打开场景": (
        "    s.open = false;\n    return s.result->scene;",
        "    s.open = false;\n    return std::nullopt;"),
    "命令行出错时退出码为 0": (
        "    return result.ok ? 0 : 4;",
        "    return 0;"),
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
Collada 不撤销 assimp 的旋转|engine/import/src/mesh.cpp|test_import_execute|import.execute
OBJ 不按材质拆分|engine/import/src/mesh.cpp|test_import_execute|import.execute
package:// 不解析|engine/import/src/importer.cpp|test_import_analyze|import.analyze
不提示毫米单位|engine/import/src/importer.cpp|test_import_analyze|import.analyze
轮子提议为位置执行器|engine/import/src/importer.cpp|test_import_analyze|import.analyze
xacro 不传包目录|engine/import/src/importer.cpp|test_import_analyze|import.analyze
导入后不打开场景|apps/viewer/import_wizard.cpp|terra_viewer|graphics.viewer_import_selftest
命令行出错时退出码为 0|apps/import/main.cpp|terra_import_cli|import.cli_broken
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
ctest --test-dir build/render-release -R '^(import\.(analyze|execute|demos)|graphics\.viewer_import_selftest)$' -V 2>/dev/null |
    grep -E "INFO|FAILED|PASS|没有下载" | sed 's/^[0-9]*: /  /' | cut -c1-300

viewer=build/render-release/apps/viewer/terra_viewer
echo "== 7. 可交互展示"
if [ $skip_show = 1 ]; then
    note "可交互展示" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "可交互展示" "未执行：不在桌面会话中"
elif [ -x "$viewer" ]; then
    echo "（即将打开 terra_viewer，最长 300 秒；这是一次“首次启动”（新的设置文件），窗口应占显示器的 85% 左右）"
    echo "  请依次试："
    echo "    1. 菜单 文件 → 导入模型…，选 $root/ROBOTS/import_demos/urdf_tutorial/urdf/07-physics.urdf（R2D2）："
    echo "       向导列出“转换 Collada 网格”“提议执行器”等；按“导入并打开”，R2D2 站在地面上，夹爪有贴图"
    echo "    2. 导入 $root/ROBOTS/import_demos/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf："
    echo "       提示“已展开 xacro”“提议雷达 base_scan 与 IMU”；在执行器面板给两个轮子速度，小车开动"
    echo "    3. 把 $root/assets/tests/import/broken.urdf 直接拖进窗口：每个问题都有原因与处理办法，“导入并打开”不可用"
    echo "    4. 留意 R2D2 夹爪（Collada 网格）的朝向是否自然，与手臂对齐（坐标轴约定与 RViz 相同）"
    echo "    5. 文件浏览窗口（导入模型…、打开场景…）：默认为网格视图，矢量图标按类型着色（文件夹、场景、动作、URDF、xacro、MJCF、网格……）；"
    echo "       右上角切换网格 / 列表；路径逐级可点，“编辑路径”可直接输入；退格回到上一级"
    echo "  导入目录：$out/show/assets/"
    "$viewer" --seconds 300 --settings "$out/show/settings.json" --library "$out/show/library.sqlite3" >"$logs/show.txt" 2>&1
    rc=$?
    grep -E "^INFO 导入完成|^INFO 场景 |^INFO 结束" "$logs/show.txt" | head -10 | sed 's/^/  /'
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
