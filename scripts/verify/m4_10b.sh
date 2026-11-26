#!/usr/bin/env bash
# M4-10b 基线机器验证：策略导入向导与自动配置（M4-10 设计稿第 4 节；用户 2026-11-25 同意）——
# 导入 .onnx（元数据 + 训练配置）、.pt（训练检查点按 spec_hash 与迭代找对应的 ONNX；TorchScript 用 PyTorch 转为 ONNX）；
# 无法自动配置时生成说明文件草稿；机器人与训练时不同（多出关节）时警告；资源库中的“策略”类型与场景卡片上的策略；
# 查看器“策略 → 导入策略…”（按关节名推荐机器人）；Headless --import-policy；ROS2 /cmd_vel 作为策略的速度指令。
#
# 请在**桌面会话终端**运行（第 9 步打开查看器）。ONNX Runtime 沿用 build/policy_library/ 中已装的组件（没有时下载 184 MB）。
# TorchScript 的转换需要带 PyTorch 的 Python：默认 python3，可用 TERRA_PYTHON=<python> 指定；没有时那一项跳过。
# 用户的策略（others/supermimic/…）只读；导入的结果写在构建目录中。
# 依次执行（日志在 build/verify-m4-10b/logs/，屏幕只打印结论）：
#   1. 依赖与素材校验；ROS2 与宇树消息包检查
#   2. 不含 ROS2 的构建：渲染关闭 Debug（预期 109/109）；渲染开启 Debug 与 Release + -Werror（预期 149/149）
#   3. ASan/UBSan（不含 ROS2；ONNX Runtime 的两个测试在 ASan 构建中跳过）
#   4. 含 ROS2 与宇树消息包的构建（Release + -Werror）：全量 CTest（预期 164/164）
#   5. ASan/UBSan（含 ROS2）：ros2 标签的测试（预期 15/15）
#   6. 变异测试（不含 ROS2 的渲染构建）：逐个注入 6 个已知错误
#   7. 输出：导入的检查结果；用你的两个检查点与 ONNX 试导入（结果写在构建目录）
#   8. 组件命令：--list-components
#   9. 可交互展示（最长 300 秒）：查看器打开导入向导并选好你的 policy.onnx，选机器人后“导入并运行”
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m4_10b.sh [--jobs N] [--skip-asan] [--skip-mutation] [--skip-show] [--mutation-only]
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
        -h|--help) sed -n '2,24p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/verify-m4-10b"
# ONNX Runtime 组件装在验证目录之外，重复运行时沿用（不重复下载 184 MB）
policy_library="$root/build/policy_library/library.db"
policy_flags=("-DTERRA_POLICY_LIBRARY=$policy_library")
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
record "tinyxml2、xacro、miniz、ORT 头文件" "$(grep -c '^一致' "$logs/third_party.txt")/16 个文件一致，退出码 $rc" "$(ok $rc)"
if [ ! -f third_party/render/install/lib/cmake/glfw3/glfw3Config.cmake ]; then
    echo "错误：未找到 GLFW 安装，请先运行 scripts/render/build.sh（M1-02）" >&2
    exit 1
fi

# ONNX Runtime 组件（M4-10a）：用刚构建的 terra_headless 安装到 build/policy_library（已安装且校验通过时跳过）
ort_state=""
ensure_ort() {  # ensure_ort <terra_headless>
    [ -n "$ort_state" ] && return 0
    local hl=$1
    [ -x "$hl" ] || return 0
    if "$hl" --list-components --library "$policy_library" 2>/dev/null | grep -q '"id":"onnxruntime-cpu".*"state":"installed"'; then
        ort_state=installed
        record "ONNX Runtime 组件" "已安装（$(dirname "$policy_library")/components/onnxruntime-cpu）" 1
        return 0
    fi
    local from=()
    [ -n "${TERRA_ORT_NUPKG:-}" ] && from=(--from "$TERRA_ORT_NUPKG")
    echo "  安装 ONNX Runtime 1.32.0（184 MB）……"
    "$hl" --install-component onnxruntime-cpu --library "$policy_library" "${from[@]}" >"$logs/install_ort.txt" 2>&1
    local rc=$?
    if [ $rc = 0 ]; then
        ort_state=installed
        record "ONNX Runtime 组件" "已下载并安装（$(grep -c '下载 ' "$logs/install_ort.txt") 行进度，sha256 一致）" 1
    else
        ort_state=failed
        record "ONNX Runtime 组件" "安装失败，退出码 $rc（见 $logs/install_ort.txt；离线时用 TERRA_ORT_NUPKG=<nupkg 文件>）" 0
        grep -E "警告|失败|错误" "$logs/install_ort.txt" | head -5 | sed 's/^/    /'
    fi
}

run_preset() {  # run_preset <preset> <名称> [额外配置参数...]
    [ $mutation_only = 1 ] && return 0
    local preset=$1 label=$2; shift 2
    local log="$logs/$preset.txt"
    rm -rf "build/$preset"
    if cmake --preset "$preset" "${policy_flags[@]}" "$@" >"$log" 2>&1 && cmake --build --preset "$preset" -j "$jobs" >>"$log" 2>&1; then
        ensure_ort "build/$preset/apps/headless/terra_headless"
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
        record "ASan 非图形（含泄漏检测）" "$(ctest_result "$logs/asan_nongraphics.txt")；跳过 $(grep -cE '\*\*\*Skipped' "$logs/asan_nongraphics.txt") 个（预期 2：policy.onnxruntime、policy.go2_walk）" "$(ok $rc1)"
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
             "${ros_flags[@]}" "${policy_flags[@]}" -DTERRA_WARNINGS_AS_ERRORS=ON >"$log" 2>&1 &&
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
        [ "$skipped" = 0 ] || record "跳过的测试" "$skipped 个（见 $logs/ros2_ctest.txt；官方示例未编译、ONNX Runtime 没有安装或没有用户策略时跳过）" 0
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
    files=(engine/policy/src/import.cpp engine/policy/src/autoconfig.cpp engine/library/src/library.cpp)
    for f in "${files[@]}"; do cp "$f" "$out/${f//\//_}.orig"; done
    restore() { for f in "${files[@]}"; do cp "$out/${f//\//_}.orig" "$f"; done; }
    trap 'restore; echo "已恢复被变异的源文件"; exit 130' INT TERM
    # 不开 -Werror：变异只改语义，不应因警告而无法判定
    cmake -S . -B "$b" -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug -DTERRA_BUILD_RENDER=ON "${policy_flags[@]}" >"$log" 2>&1
    detected=0
    total=0
    while IFS='|' read -r name file target testname; do
        total=$((total + 1))
        restore
        if ! python3 - "$file" "$name" >>"$log" 2>&1 <<'PY'
import sys
path, name = sys.argv[1], sys.argv[2]
mutations = {
    "检查点只比 spec_hash": (
        "        if (!info.iteration || !o.iteration || *o.iteration == *info.iteration) {",
        "        if (true) {"),
    "pickle 记忆操作跳错": (
        "        if (op == 'q') p += 2;",
        "        if (op == 'q') p += 1;"),
    "TorchScript 当成检查点": (
        "        if (s.find(\"/code/\") != std::string::npos || endsWith(s, \"/constants.pkl\")) z.code = true;",
        "        if (false) z.code = true;"),
    "同名导入覆盖旧目录": (
        "    for (int n = 2; fs::exists(dir, ec) && n < 1000; ++n)",
        "    for (int n = 2; false && n < 1000; ++n)"),
    "多出关节不警告": (
        "    if (!match.extra.empty()) {",
        "    if (false) {"),
    "资源库不扫描 policies/": (
        "        roots.push_back({(fs::path{absolute(libraryFile)}.parent_path() / \"policies\").string(), AssetSource::Imported});  // M4-10b",
        ""),
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
检查点只比 spec_hash|engine/policy/src/import.cpp|test_policy_import|policy.import
pickle 记忆操作跳错|engine/policy/src/import.cpp|test_policy_import|policy.import
TorchScript 当成检查点|engine/policy/src/import.cpp|test_policy_import|policy.import
同名导入覆盖旧目录|engine/policy/src/import.cpp|test_policy_import|policy.import
多出关节不警告|engine/policy/src/autoconfig.cpp|test_policy_controller|policy.controller
资源库不扫描 policies/|engine/library/src/library.cpp|test_library_assets|library.assets
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
probe="$ros_build"
[ -d "$probe" ] || probe="build/render-release"
if [ -d "$probe" ]; then
    ctest --test-dir "$probe" -R '^(policy\.import|headless\.policy|ros2\.policy_cmd_vel|graphics\.viewer_policy_import_selftest)$' -V 2>/dev/null |
        grep -E "PASS|FAILED|SKIP|all passed|自检通过|自检失败" | sed 's/^[0-9]*: /  /' | cut -c1-260
fi
hl="$probe/apps/headless/terra_headless"
run_dir=$(ls -d others/supermimic/go2_supermimic/*/ 2>/dev/null | head -1)
if [ -x "$hl" ] && [ -n "$run_dir" ]; then
    trial="$out/import_trial/library.db"
    mkdir -p "$(dirname "$trial")"
    echo "  试导入你的策略（写在 $(dirname "$trial")/policies/，原文件不动）："
    for f in policy.onnx model_15000.pt model_10000.pt; do
        [ -f "$run_dir$f" ] || continue
        "$hl" --import-policy "$run_dir$f" --scene assets/scenes/robots/go2_policy.terra-scene.json --library "$trial" \
            >"$logs/import_$f.txt" 2>"$logs/import_$f.err"
        rc=$?
        echo "    $f：退出码 $rc；$(tail -1 "$logs/import_$f.txt" | python3 -c 'import json,sys
try:
    r=json.loads(sys.stdin.read()); print(r["source_kind"], "→", r["policy"] or r["error"][:160])
except Exception as e: print("（无输出）")')"
    done
    "$hl" --import-policy "${run_dir}policy.onnx" --scene assets/scenes/robots/go2w.terra-scene.json --library "$trial" \
        >"$logs/import_go2w.txt" 2>&1
    echo "    policy.onnx 对 Go2W：$(grep -o '机器人“go2w”上还有[^）]*）' "$logs/import_go2w.txt" | head -1)"
fi

echo "== 8. 组件命令"
hl="$probe/apps/headless/terra_headless"
if [ -x "$hl" ]; then
    "$hl" --list-components --library "$policy_library" | cut -c1-260 | sed 's/^/  /'
fi

viewer="build/render-release/apps/viewer/terra_viewer"
go2_policy=$(ls others/supermimic/go2_supermimic/*/policy.onnx 2>/dev/null | head -1)
echo "== 9. 可交互展示"
if [ $skip_show = 1 ]; then
    note "可交互展示" "已跳过（--skip-show）"
elif [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    note "可交互展示" "未执行：不在桌面会话中"
elif [ -z "$go2_policy" ]; then
    note "可交互展示" "未执行：没有找到 others/supermimic/go2_supermimic/*/policy.onnx"
elif [ -x "$viewer" ]; then
    mkdir -p "$out/show"
    echo "（即将打开 terra_viewer，最长 300 秒；关闭窗口即结束。设置在 $out/show/；资源库、组件与导入的策略在 build/policy_library/，不读写你的用户目录）"
    echo "  - 导入向导已打开并选好 $go2_policy：等它检查完资源库中的场景"
    echo "  - 第 2 步的列表：“宇树 Go2 策略运行”等为“完全对应”，Go2W 为“多出关节”（悬停看多出的轮子关节）"
    echo "  - 选一个 Go2 场景，按“导入并运行”：打开场景并运行；左侧资源库的“策略”分页出现这张卡片，场景卡片上有“1 个策略”"
    echo "  - 也可以把 model_15000.pt 拖进窗口：向导说明它对应哪个 ONNX；model_10000.pt 会说明不能代用"
    echo "  - 停止后单击“策略”分页中的卡片，或场景卡片右键“运行策略：…”，再次运行"
    "$viewer" --import-policy "$go2_policy" --seconds 300 --settings "$out/show/settings.json" --library "$policy_library" >"$logs/show.txt" 2>&1
    rc=$?
    grep -E "^INFO 场景 |^INFO 策略" "$logs/show.txt" | head -8 | sed 's/^/  /'
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
