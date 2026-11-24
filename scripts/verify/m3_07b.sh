#!/usr/bin/env bash
# M3-07b 基线机器验证：SQLite 实验记录库（D-112；M3-07 评审：实验记录进入数据库）——SQLite 3.54.1 合并版源码（锁定）、
# terra_library（建库、版本号、加入 / 更新记录、按场景筛选、清理、按目录重建、损坏时备份重建）、
# Headless --library / --list-records（--record 完成后自动加入索引）。
#
# 本步没有新的界面，不打开窗口；查看器的记录列表与回放在 M3-07c。
# 依次执行（日志在 build/verify-m3-07b/logs/，屏幕只打印结论）：
#   1. 依赖与素材校验（新增 SQLite 源码校验）
#   2. 渲染关闭 Debug：全量 CTest（预期 73/73，新增 library.store、headless.list_records）
#   3. 渲染开启 Debug 与 Release + -Werror：全量 CTest（预期 101/101）
#   4. ASan/UBSan（渲染开启）
#   5. 变异测试：排序、清理、损坏重建、版本检查、未完成记录的标记、场景筛选、Headless 加入索引，逐个注入 7 个已知错误
#   6. 输出：Headless 记录三个动作并列出库中的记录（库文件在构建目录，不写入用户目录）
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m3_07b.sh [--jobs N] [--skip-asan] [--skip-mutation] [--mutation-only]
# --mutation-only：只做变异测试（阶段验收调用；不删除、不重建 build/ 下的预设构建目录，跳过 ASan）
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
        -h|--help) sed -n '2,16p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/verify-m3-07b"
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
bash scripts/sqlite/check_sqlite.sh >"$logs/sqlite.txt" 2>&1
rc=$?
record "SQLite 源码校验" "$(grep -c '^一致' "$logs/sqlite.txt")/2 个文件一致，退出码 $rc" "$(ok $rc)"
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
    files=(engine/library/src/library.cpp apps/headless/main.cpp)
    for f in "${files[@]}"; do cp "$f" "$out/${f//\//_}.orig"; done
    restore() { for f in "${files[@]}"; do cp "$out/${f//\//_}.orig" "$f"; done; }
    trap 'restore; echo "已恢复被变异的源文件"; exit 130' INT TERM
    # 不开 -Werror：变异只改语义，不应因警告而无法判定
    cmake -S . -B "$b" -G Ninja -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=Debug -DTERRA_BUILD_RENDER=OFF >"$log" 2>&1
    detected=0
    total=0
    while IFS='|' read -r name file target testname; do
        total=$((total + 1))
        restore
        if ! python3 - "$file" "$name" >>"$log" 2>&1 <<'PY'
import sys
path, name = sys.argv[1], sys.argv[2]
mutations = {
    "列出时不按创建时间排序": (
        '" ORDER BY created DESC, id DESC"',
        '" ORDER BY id ASC"'),
    "不清理文件已不存在的条目": (
        "            if (!fs::exists(s.columnText(1), ec)) missing.push_back(s.columnInt(0));",
        "            (void)ec;"),
    "库文件损坏时不备份重建": (
        "    if (rc == SQLITE_NOTADB || rc == SQLITE_CORRUPT) {",
        "    if (false) {"),
    "不检查库的版本": (
        "    if (v > kSchemaVersion) {",
        "    if (false) {"),
    "未完成的记录标为 completed": (
        '        s.text(15, "incomplete");',
        '        s.text(15, "completed");'),
    "场景筛选失效": (
        '(scene.empty() ? "" : " WHERE scene = ?1")',
        '""'),
    "Headless 不加入索引": (
        "        } else if (const auto failed = library->addRecord(options.recordPath)) {",
        "        } else if (const auto failed = std::optional<std::string>{}) {"),
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
列出时不按创建时间排序|engine/library/src/library.cpp|test_library|library.store
不清理文件已不存在的条目|engine/library/src/library.cpp|test_library|library.store
库文件损坏时不备份重建|engine/library/src/library.cpp|test_library|library.store
不检查库的版本|engine/library/src/library.cpp|test_library|library.store
未完成的记录标为 completed|engine/library/src/library.cpp|test_library|library.store
场景筛选失效|engine/library/src/library.cpp|test_library|library.store
Headless 不加入索引|apps/headless/main.cpp|terra_headless|headless.list_records
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
lib="$out/library.sqlite3"
for motion in g1/wave go2/lie_stand_sit stretch3/tour; do
    name=$(basename "$motion")
    build/debug/apps/headless/terra_headless --motion "assets/motions/$motion.terra-motion.json" --sample-every 10 \
        --record "$out/records/$name.terra-record.jsonl" --library "$lib" >/dev/null 2>"$logs/record_$name.err"
    echo "  记录 $motion：退出码 $?，$(du -h "$out/records/$name.terra-record.jsonl" 2>/dev/null | cut -f1)"
done
echo "  库中的记录（--list-records）："
build/debug/apps/headless/terra_headless --list-records --library "$lib" >"$logs/list.jsonl" 2>"$logs/list.err"
rc=$?
cut -c1-220 "$logs/list.jsonl" | sed 's/^/    /'
record "Headless 记录并列出" "退出码 $rc，$(wc -l <"$logs/list.jsonl") 条" "$([ $rc = 0 ] && [ "$(wc -l <"$logs/list.jsonl")" = 3 ] && echo 1 || echo 0)"

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
