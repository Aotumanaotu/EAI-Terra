#!/usr/bin/env bash
# M3-01 基线机器验证：参考机器人锁定与导入核查（MuJoCo Menagerie：宇树 G1、宇树 Go2、Hello Robot Stretch 3）。
#
# 请在**桌面会话终端**运行（第 7 步打开可见窗口）。用户下载的原件在 ROBOTS/mujoco_menagerie/（开发计划 10.4 节）。
# 依次执行（日志在 build/verify-m3-01/logs/，屏幕只打印结论）：
#   1. 依赖与素材校验；参考机器人：scripts/assets/fetch_robots.sh（优先从 ROBOTS/ 复制，逐个校验 sha256，199 个文件）
#   2. 渲染关闭 Debug：全量 CTest（预期 50/50，新增 robots.import）
#   3. 渲染开启 Debug 与 Release + -Werror：全量 CTest（预期 70/70）
#   4. ASan/UBSan（渲染开启）
#   5. 锁定的反向检查：改坏一个已复制的文件，fetch_robots.sh 必须发现并恢复（本步不涉及引擎代码，没有变异测试）
#   6. 输出：三台机器人的导入核查表（robots.import 的 INFO 行）
#   7. 可交互展示（最长 120 秒）：查看器打开 G1 场景（暂停）；可用“文件 → 打开场景…”打开 Go2、Stretch 3
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m3_01.sh [--jobs N] [--skip-asan] [--skip-show]
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
        --skip-mutation) shift ;;
        --skip-show) skip_show=1; shift ;;
        -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/verify-m3-01"
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

echo "== 5. 锁定的反向检查"
victim=assets/robots/unitree_go2/go2.xml
if [ -f "$victim" ]; then
    cp "$victim" "$out/go2.xml.orig"
    printf '\n<!-- 改坏 -->\n' >>"$victim"
    bash scripts/assets/fetch_robots.sh --offline >"$logs/robots_tamper.txt" 2>&1
    rc=$?
    if [ $rc = 0 ] && cmp -s "$victim" "$out/go2.xml.orig"; then
        record "锁定的反向检查" "改坏的 go2.xml 被发现并恢复（$(tail -1 "$logs/robots_tamper.txt" | cut -c1-40)…）" 1
    else
        cp "$out/go2.xml.orig" "$victim"
        record "锁定的反向检查" "没有恢复改坏的文件（退出码 $rc，见 $logs/robots_tamper.txt）" 0
    fi
else
    record "锁定的反向检查" "未执行：$victim 不存在" 0
fi

echo "== 6. 输出（导入核查表）"
ctest --test-dir build/debug -R '^robots\.import$' -V 2>/dev/null | grep -E "INFO|FAILED|Skipped|跳过" | sed 's/^[0-9]*: //'

viewer=build/render-release/apps/viewer/terra_viewer
echo "== 7. 可交互展示"
if [ $skip_show = 1 ]; then
    note "可交互展示" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "可交互展示" "未执行：不在桌面会话中"
elif [ -x "$viewer" ]; then
    echo "（即将打开 terra_viewer，最长 120 秒；设置文件：$out/show/settings.json）"
    echo "  打开的是宇树 G1（暂停）。这是导入核查，还没有控制与平滑法线（M3-02～M3-04）："
    echo "    - 网格按平直着色显示，颜色来自 MJCF 材质；大纲中可看到 30 个实体与各连杆的几何体"
    echo "    - 按空格播放：执行器目标全为 0，机器人会失去平衡倒下——这是预期的（还没有站立控制）；Ctrl+R 重置"
    echo "    - 文件 → 打开场景…：进入 assets/scenes/robots/，打开 go2 与 stretch3 看另外两台"
    "$viewer" --scene assets/scenes/robots/g1.terra-scene.json --seconds 120 --settings "$out/show/settings.json" \
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
