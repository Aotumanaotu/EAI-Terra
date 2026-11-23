#!/usr/bin/env bash
# M2-02 / M2-03 基线机器验证：加载失败报告与用户数据保护（致命错误报告、紧急保存、退出码 70）；
# 几何体与网格描述（模型描述修订 2）。
#
# 可在任意终端运行（本次没有新的可见窗口；自动化图形测试自己使用 Xvfb + llvmpipe，D-022）。
# 依次执行（日志在 build/verify-m2-02-03/logs/，屏幕只打印结论）：
#   1. 依赖与素材校验
#   2. 渲染关闭 Debug：全量 CTest（预期 41/41，新增 core.fatal、physics.geometry；physics.backend_fatal 改为退出码 70）
#   3. 渲染开启 Debug 与 Release + -Werror：全量 CTest（预期 59/59）
#   4. ASan/UBSan（渲染开启）
#   5. 变异测试：紧急保存、崩溃目录、致命错误处理、几何体颜色/位姿/归属/尺寸，逐个注入 8 个已知错误（预期 8/8）
#   6. 输出：几何体描述的 INFO 行；一份真实的崩溃报告（由 physics.backend_fatal 子进程生成）
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m2_02_03.sh [--jobs N] [--skip-asan] [--skip-mutation] [--mutation-only]
# --mutation-only：只做变异测试（阶段验收调用；不删除、不重建 build/ 下的预设构建目录，跳过 ASan、参考图生成与真实 GPU 步骤）
#   （--skip-show 为兼容旧习惯而接受，本脚本没有可见窗口）
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

out="$root/build/verify-m2-02-03"
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
    files=(engine/core/src/fatal.cpp engine/physics/backends/mujoco/src/mujoco_world.cpp)
    for f in "${files[@]}"; do cp "$f" "$out/$(basename "$f").orig"; done
    restore() { for f in "${files[@]}"; do cp "$out/$(basename "$f").orig" "$f"; done; }
    trap 'restore; echo "已恢复被变异的源文件"; exit 130' INT TERM
    # 不开 -Werror：变异只改语义，不应因警告而无法判定
    cmake -S . -B "$b" -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug >"$log" 2>&1
    detected=0
    total=0
    while IFS='|' read -r name file target testname; do
        total=$((total + 1))
        restore
        if ! python3 - "$file" "$name" >>"$log" 2>&1 <<'PY'
import sys
path, name = sys.argv[1], sys.argv[2]
mutations = {
    "紧急保存失败后中断后续项": (
        "        results.push_back(std::move(result));\n    }\n    return results;",
        "        results.push_back(std::move(result));\n        if (!results.back().ok) break;\n    }\n    return results;"),
    "崩溃目录忽略 TERRA_CRASH_DIR": (
        "    if (std::string dir = value(\"TERRA_CRASH_DIR\"); !dir.empty()) return dir;",
        "    (void)0;"),
    "致命错误前不做紧急保存": (
        "    const std::string saves = formatEmergencyResults(runEmergencySaves());",
        "    const std::string saves = formatEmergencyResults({});"),
    "致命错误处理函数返回": (
        "        terra::core::fatalExit(report);\n    }",
        "        (void)report;\n    }"),
    "材质颜色规则颠倒": (
        "(matid >= 0 && defaultColor)",
        "(matid >= 0 && !defaultColor)"),
    "几何体局部位姿忽略旋转": (
        "        geom.local.orientation = {quat[0], quat[1], quat[2], quat[3]};",
        "        geom.local.orientation = {1.0, 0.0, 0.0, 0.0};"),
    "静态几何算作实体": (
        "        geom.entity = bodyToEntity[static_cast<std::size_t>(m->geom_bodyid[g])];",
        "        geom.entity = bodyToEntity[static_cast<std::size_t>(m->geom_bodyid[g] > 0 ? m->geom_bodyid[g] : 1)];"),
    "半长取成全长": (
        "        geom.size = {size[0], size[1], size[2]};",
        "        geom.size = {size[0], 2 * size[1], size[2]};"),
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
            n=$(sed -n "/^---- 变异：$name\$/,\$p" "$log" | grep -cE '^FAILED|不匹配|退出码不符')
            summary+=("  变异 $name：已发现（$n 项检查失败）")
        fi
    done <<'LIST'
紧急保存失败后中断后续项|engine/core/src/fatal.cpp|test_fatal|core.fatal
崩溃目录忽略 TERRA_CRASH_DIR|engine/core/src/fatal.cpp|test_fatal|core.fatal
致命错误前不做紧急保存|engine/core/src/fatal.cpp|test_backend_fatal|physics.backend_fatal
致命错误处理函数返回|engine/physics/backends/mujoco/src/mujoco_world.cpp|test_backend_fatal|physics.backend_fatal
材质颜色规则颠倒|engine/physics/backends/mujoco/src/mujoco_world.cpp|test_mujoco_geometry|physics.geometry
几何体局部位姿忽略旋转|engine/physics/backends/mujoco/src/mujoco_world.cpp|test_mujoco_geometry|physics.geometry
静态几何算作实体|engine/physics/backends/mujoco/src/mujoco_world.cpp|test_mujoco_geometry|physics.geometry
半长取成全长|engine/physics/backends/mujoco/src/mujoco_world.cpp|test_mujoco_geometry|physics.geometry
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
ctest --test-dir build/debug -R '^physics\.geometry$' -V 2>/dev/null | grep -E "INFO|FAILED" | sed 's/^[0-9]*: //'
crash=$(ls -t build/debug/tests/work/backend_fatal/crash/crash-*.txt 2>/dev/null | head -1)
if [ -n "$crash" ]; then
    echo "  一份真实的崩溃报告（$crash）："
    sed 's/^/    | /' "$crash"
    note "崩溃报告示例" "已生成（physics.backend_fatal）"
else
    record "崩溃报告示例" "未找到 build/debug/tests/work/backend_fatal/crash/crash-*.txt" 0
fi
if ls "$HOME/.local/state/terra"/crash-*.txt >/dev/null 2>&1; then
    note "用户目录" "注意：~/.local/state/terra 中有崩溃报告（不是本次测试写的，测试只写构建目录）"
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
