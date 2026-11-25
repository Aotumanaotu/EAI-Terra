#!/usr/bin/env bash
# M4-08c 基线机器验证：资源库与一键打开的场景卡片（M4-08c 设计稿 2026-11-25 评审通过；用户修正：场景就是按钮，点一下就加载）——
# 资源库新增资产表（场景、动作、机器人、环境贴图；增量扫描、收藏、隐藏、缺失标记；版本 1 → 2 自动升级）；
# 查看器：资源库面板（卡片、分页、筛选、搜索）、欢迎页（没有场景时）、Ctrl+P 快速打开、“文件 → 资源库中的场景”；
# 资源库位置可在偏好设置中修改（复制到新位置）；资产包 .terra-pack（miniz）；Headless --list-assets；场景文件的短标题 title。
# 同时复跑 M4-08b 的全部测试（TurtleBot 4 的高度阈值、文件浏览窗口的两处修改）。
#
# 请在**桌面会话终端**运行（第 8 步打开查看器）。依赖与 M4-08b 相同（ROS2 Humble 与 cyclonedds_ws 只用于第 4、5 步的全量测试）。
# 依次执行（日志在 build/verify-m4-08c/logs/，屏幕只打印结论）：
#   1. 依赖与素材校验（新增 miniz 3.0.2 的 sha256）；ROS2 与宇树消息包检查
#   2. 不含 ROS2 的构建：渲染关闭 Debug（预期 99/99）；渲染开启 Debug 与 Release + -Werror（预期 135/135）；
#      之后重新导入 TurtleBot 4（场景多了短标题 title）
#   3. ASan/UBSan（不含 ROS2）
#   4. 含 ROS2 与宇树消息包的构建（Release + -Werror）：全量 CTest（预期 147/147）
#   5. ASan/UBSan（含 ROS2）：ros2 标签的测试
#   6. 变异测试（不含 ROS2 的渲染构建）：逐个注入 8 个已知错误
#   7. 输出：资源库、资产包、Headless 与查看器自检的 INFO 行
#   8. 可交互展示（最长 300 秒）：查看器不带场景启动，欢迎页与资源库面板列出全部场景，单击即打开
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m4_08c.sh [--jobs N] [--skip-asan] [--skip-mutation] [--skip-show] [--mutation-only]
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
        -h|--help) sed -n '2,22p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/verify-m4-08c"
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

echo "== 1. 依赖与 ROS2"
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
bash scripts/import/check_third_party.sh >"$logs/third_party.txt" 2>&1
rc=$?
record "tinyxml2、xacro、miniz 校验" "$(grep -c '^一致' "$logs/third_party.txt")/12 个文件一致，退出码 $rc" "$(ok $rc)"
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


ros_setup=/opt/ros/humble/setup.bash
unitree=${UNITREE_ROS2:-$HOME/unitree_ros2}
unitree_setup="$unitree/cyclonedds_ws/install/setup.bash"
bash scripts/ros2/check_ros2.sh --setup "$unitree_setup" >"$logs/check_ros2.txt" 2>&1
rc=$?
record "ROS2 与宇树消息包检查" "退出码 $rc（见 $logs/check_ros2.txt）" "$(ok $rc)"
examples_found=0
for e in go2_stand_example g1_low_level_example; do
    p=$(find "$unitree/example" -type f -name "$e" -perm -u+x 2>/dev/null | head -1)
    if [ -n "$p" ]; then
        examples_found=$((examples_found + 1))
        echo "  官方示例 $e：$p"
    fi
done
if [ $examples_found = 2 ]; then
    record "宇树官方示例程序" "已编译（go2_stand_example、g1_low_level_example）" 1
else
    record "宇树官方示例程序" "没有找到（$unitree/example），请先编译：cd $unitree/example && source $ros_setup && source ../cyclonedds_ws/install/setup.bash && colcon build" 0
fi

echo "== 2. 不含 ROS2 的构建"
run_preset debug "渲染关闭 Debug"
run_preset render-debug "渲染 Debug"
run_preset render-release "渲染 Release -Werror" -DTERRA_WARNINGS_AS_ERRORS=ON
if [ $mutation_only = 0 ]; then
    bash scripts/assets/import_turtlebot4.sh build/render-release/apps/import/terra_import >"$logs/import_turtlebot4.txt" 2>&1
    rc=$?
    record "重新导入 TurtleBot 4" "退出码 $rc；$(grep -c '^terra_import: \[warning\]' "$logs/import_turtlebot4.txt") 个警告（见 $logs/import_turtlebot4.txt）" "$(ok $rc)"
    [ $rc = 0 ] || grep -E '\[error\]' "$logs/import_turtlebot4.txt" | head -10 | sed 's/^/    /'
fi

san="-fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer"
if [ $skip_asan = 0 ]; then
    echo "== 3. ASan/UBSan（不含 ROS2）"
    log="$logs/asan.txt"
    b="$out/asan"
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

# ---- 以下需要 ROS2 与宇树消息包 ----
if [ ! -f "$ros_setup" ] || [ ! -f "$unitree_setup" ]; then
    record "ROS2" "未找到 $ros_setup 或 $unitree_setup，第 4～8 步无法执行" 0
    echo; printf '%s\n' "${summary[@]}"; exit 1
fi
echo "  RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-（未设置）} CYCLONEDDS_URI=${CYCLONEDDS_URI:-（未设置）} → 清除，使用 Humble 默认" >"$logs/ros_env.txt"
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
set +u
# shellcheck disable=SC1090
source "$ros_setup"
# shellcheck disable=SC1090
source "$unitree_setup"
set -u
unset RMW_IMPLEMENTATION CYCLONEDDS_URI
echo "  ROS_DISTRO=${ROS_DISTRO:-?}" >>"$logs/ros_env.txt"
export ROS_LOCALHOST_ONLY=1
ros_flags=(-DTERRA_BUILD_ROS2=ON "-DTERRA_UNITREE_ROS2_DIR=$unitree")

ros_build="$out/ros2"
if [ $mutation_only = 0 ]; then
    echo "== 4. 含 ROS2 与宇树消息包的构建"
    log="$logs/ros2_build.txt"
    if cmake -S . -B "$ros_build" -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Release -DTERRA_BUILD_RENDER=ON \
             "${ros_flags[@]}" -DTERRA_WARNINGS_AS_ERRORS=ON >"$log" 2>&1 &&
       cmake --build "$ros_build" -j "$jobs" >>"$log" 2>&1; then
        grep -E "ROS2 桥接：" "$log" | head -2 | sed 's/^-- /  /'
        if ! grep -q "宇树消息包：ON" "$log"; then
            record "含 ROS2 的构建" "CMake 没有找到宇树消息包（见 $log）" 0
        fi
        if grep -q "warning:" "$log"; then
            record "含 ROS2 构建" "有编译警告（见 $log）" 0
            grep -A3 "warning:" "$log" | head -20 | sed 's/^/    /'
        fi
        ctest --test-dir "$ros_build" --output-on-failure >"$logs/ros2_ctest.txt" 2>&1
        rc=$?
        record "含 ROS2 Release -Werror 测试" "$(ctest_result "$logs/ros2_ctest.txt")" "$(ok $rc)"
        [ $rc = 0 ] || grep -E "^\s+[0-9]+ - |FAILED" "$logs/ros2_ctest.txt" | head -30 | sed 's/^/    /'
        skipped=$(grep -cE "\*\*\*Skipped|Not Run.*Skipped" "$logs/ros2_ctest.txt")
        [ "$skipped" = 0 ] || record "跳过的测试" "$skipped 个（见 $logs/ros2_ctest.txt；官方示例未编译时跳过）" 0
    else
        record "含 ROS2 构建" "失败（见 $log）" 0
        grep -E -A3 "error:|warning:|CMake Error" "$log" | head -30 | sed 's/^/    /'
    fi
fi

if [ $skip_asan = 0 ]; then
    echo "== 5. ASan/UBSan（含 ROS2）"
    log="$logs/ros2_asan.txt"
    b="$out/ros2_asan"
    if cmake -S . -B "$b" -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug -DTERRA_BUILD_RENDER=ON "${ros_flags[@]}" \
             -DTERRA_WARNINGS_AS_ERRORS=ON "-DCMAKE_CXX_FLAGS=$san" "-DCMAKE_EXE_LINKER_FLAGS=$san" >"$log" 2>&1 &&
       cmake --build "$b" -j "$jobs" >>"$log" 2>&1; then
        # new_delete_type_mismatch=0：rclcpp Humble 头文件中 retyped_deallocate<char> 的上游缺陷（M4-06a 报告第 5.2 节）
        ASAN_OPTIONS=detect_leaks=0:new_delete_type_mismatch=0 UBSAN_OPTIONS=print_stacktrace=1 \
            ctest --test-dir "$b" -L ros2 --output-on-failure >"$logs/ros2_asan_ctest.txt" 2>&1
        rc=$?
        record "ASan 含 ROS2（ros2 标签）" "$(ctest_result "$logs/ros2_asan_ctest.txt")" "$(ok $rc)"
        [ $rc = 0 ] || grep -E "ERROR: AddressSanitizer|SUMMARY: |runtime error|FAILED" "$logs/ros2_asan_ctest.txt" | head -20 | sed 's/^/    /'
    else
        record "ASan 含 ROS2 构建" "失败（见 $log）" 0
    fi
fi

if [ $skip_mutation = 0 ]; then
    echo "== 6. 变异测试"
    log="$logs/mutation.txt"
    b="$out/mutation"
    files=(engine/library/src/library.cpp engine/library/src/pack.cpp apps/viewer/library_panel.cpp apps/import/main.cpp)
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
    "增量扫描不跳过未改动的文件": (
        "                ++result.unchanged;\n                continue;",
        "                (void)0;"),
    "删除的文件不标为缺失": (
        "            if (!underRoot && fs::exists(key.second, ec)) continue;",
        "            continue;"),
    "重新扫描清掉收藏": (
        "\"import_dir = excluded.import_dir, license = excluded.license, missing = 0, indexed_at = excluded.indexed_at\");",
        "\"import_dir = excluded.import_dir, license = excluded.license, missing = 0, favorite = 0, indexed_at = excluded.indexed_at\");"),
    "搜索只看第一个词": (
        "        if (j > i && haystack.find(q.substr(i, j - i)) == std::string::npos) return false;",
        "        if (j > i && haystack.find(q.substr(i, j - i)) == std::string::npos) return false;\n        if (j > i) return true;"),
    "资产包接受含 .. 的路径": (
        "        if (part.empty() || part == \".\" || part == \"..\") return false;",
        "        if (part.empty()) return false;"),
    "资产包不核对 sha256": (
        "        if (data.size() != item.bytes || terra::sha256Hex(data) != item.sha256) {",
        "        if (data.size() != item.bytes) {"),
    "单击卡片不打开场景": (
        "        } else {\n            action = LibraryAction{LibraryAction::Type::Open, a.kind, a.path, {}, false};\n        }",
        "        } else {\n            (void)0;\n        }"),
    "命令行导入后不登记": (
        "    if (result.ok) registerDirectory(libraryPath, result.directory);",
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
            n=$(awk -v head="---- 变异：$name" '$0 == head {found = 1} found' "$log" | grep -cE 'FAILED: |FAILED |不匹配|退出码不符|没有匹配')
            summary+=("  变异 $name：已发现（$n 项检查失败）")
        fi
    done <<'LIST'
增量扫描不跳过未改动的文件|engine/library/src/library.cpp|test_library_assets|library.assets
删除的文件不标为缺失|engine/library/src/library.cpp|test_library_assets|library.assets
重新扫描清掉收藏|engine/library/src/library.cpp|test_library_assets|library.assets
搜索只看第一个词|engine/library/src/library.cpp|test_library_assets|library.assets
资产包接受含 .. 的路径|engine/library/src/pack.cpp|test_pack|library.pack
资产包不核对 sha256|engine/library/src/pack.cpp|test_pack|library.pack
单击卡片不打开场景|apps/viewer/library_panel.cpp|terra_viewer|graphics.viewer_library_selftest
命令行导入后不登记|apps/import/main.cpp|terra_import_cli|import.cli_pack
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

echo "== 7. 输出"
if [ -d "$ros_build" ]; then
    ctest --test-dir "$ros_build" -R '^(library\.(assets|pack)|import\.cli_pack|headless\.list_assets|graphics\.viewer_(library|import)_selftest|robots\.turtlebot4_import)$' -V 2>/dev/null |
        grep -E "INFO|PASS|FAILED|SKIP|all passed|自检|解包后|通过" | grep -v "启动：" | sed 's/^[0-9]*: /  /' | cut -c1-260
fi

viewer="build/render-release/apps/viewer/terra_viewer"
echo "== 8. 可交互展示"
if [ $skip_show = 1 ]; then
    note "可交互展示" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "可交互展示" "未执行：不在桌面会话中"
elif [ -x "$viewer" ]; then
    echo "（即将打开 terra_viewer，最长 300 秒；关闭窗口即结束。资源库与设置在 $out/show/，不读写你的用户目录）"
    echo "  - 不带场景启动：视口中是欢迎页，左上是资源库面板，全部场景都是卡片——单击任意一张即加载"
    echo "  - 当前场景的卡片描边并标“当前”；右上角星标收藏（收藏的排在最前）；悬停看说明与路径；右键有更多操作"
    echo "  - 面板分页：场景 / 动作（单击即打开推荐场景并播放）/ 机器人；筛选：全部、收藏、最近、内置、导入；搜索框"
    echo "  - Ctrl+P：快速打开（输入名字的一部分，↑↓ 选择，回车打开）；文件 → 资源库中的场景"
    echo "  - 请顺便单击 TurtleBot 4 的卡片，确认 Create 3 底盘与保险杠的外观（M4-08b 遗留的 3 个 Collada 警告）"
    mkdir -p "$out/show"
    "$viewer" --seconds 300 --settings "$out/show/settings.json" --library "$out/show/library.sqlite3" >"$logs/show.txt" 2>&1
    rc=$?
    grep -E "^INFO 资源库扫描|^INFO 场景 |^INFO 结束" "$logs/show.txt" | head -12 | sed 's/^/  /'
    record "可交互展示" "退出码 $rc" "$(ok $rc)"
else
    record "可交互展示" "未执行（构建失败）" 0
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
