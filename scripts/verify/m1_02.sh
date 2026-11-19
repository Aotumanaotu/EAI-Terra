#!/usr/bin/env bash
# M1-02 基线机器验证：渲染依赖源码锁定（GLFW、glad、GLM）。
#
# 请在**桌面会话终端**运行（第 6 步要用真实 GPU；自动化图形测试自己会清除 DISPLAY 并改用 Xvfb）。
# 依次执行（日志在 build/verify-m1-02/logs/，屏幕只打印结论）：
#   1. scripts/render/fetch.sh：下载并校验 GLFW、GLM、glad 生成器源码（需要联网）
#   2. scripts/render/build.sh：构建并安装 GLFW 静态库（X11 + Wayland）
#   3. scripts/render/gen_glad.sh：用锁定版本重新生成 glad 并与仓库逐字节比较（缺 jinja2 时跳过并注明）
#   4. 渲染关闭（M0 回归）：删除 build/debug 后 Debug 全量 CTest（预期 35/35，与 M0 相同）
#   5. 渲染开启：删除 build/render-debug、build/render-release 后构建；Release 加 -Werror（预期 36/36，含 graphics.render_deps）
#   6. 真实 GPU：在桌面会话直接运行探针，记录实际 OpenGL 版本与渲染器
# 最后打印汇总；任一步失败时退出码非 0。
# 用法：bash scripts/verify/m1_02.sh [--jobs N] [--no-wayland]
set -uo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
cd "$root"

jobs=$(nproc)
build_args=()
while [ $# -gt 0 ]; do
    case "$1" in
        --jobs) jobs=$2; shift 2 ;;
        --no-wayland) build_args+=(--no-wayland); shift ;;
        -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/verify-m1-02"
logs="$out/logs"
rm -rf "$out"
mkdir -p "$logs"

summary=()
failed=0
record() {  # record <名称> <结果文字> <是否通过 0/1>
    summary+=("$(printf '%-26s %s' "$1" "$2")")
    [ "$3" = 1 ] || failed=1
    echo "${summary[-1]}"
}
ok() { [ "$1" = 0 ] && echo 1 || echo 0; }
ctest_result() { grep -E "tests passed|tests failed" "$1" | tail -1; }

echo "== 环境"
{
    uname -r
    g++ --version | head -1
    cmake --version | head -1
    echo "ninja $(ninja --version)"
    echo "wayland-scanner: $(command -v wayland-scanner || echo 未安装)"
    echo "python3-jinja2: $(python3 -c 'import jinja2; print(jinja2.__version__)' 2>/dev/null || echo 未安装)"
    echo "DISPLAY='${DISPLAY:-}' XDG_SESSION_TYPE='${XDG_SESSION_TYPE:-}'"
} | tee "$logs/env.txt"

echo "== 1. 获取源码"
bash scripts/render/fetch.sh >"$logs/fetch.txt" 2>&1
rc=$?
record "获取并校验源码" "退出码 $rc" "$(ok $rc)"

echo "== 2. 构建 GLFW"
bash scripts/render/build.sh --jobs "$jobs" "${build_args[@]}" >"$logs/build_glfw.txt" 2>&1
rc=$?
record "构建 GLFW" "$(tail -1 "$logs/build_glfw.txt")" "$(ok $rc)"

echo "== 3. 复核 glad 生成代码"
if python3 -c "import jinja2" 2>/dev/null; then
    bash scripts/render/gen_glad.sh >"$logs/glad.txt" 2>&1
    rc=$?
    record "glad 复核" "$(tail -1 "$logs/glad.txt")" "$(ok $rc)"
else
    summary+=("$(printf '%-26s %s' "glad 复核" "未执行：缺少 python3-jinja2（sudo apt install python3-jinja2），不阻塞")")
    echo "${summary[-1]}"
fi

run_preset() {  # run_preset <preset> <名称> [额外配置参数...]
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
    else
        record "$label 构建" "失败（见 $log）" 0
        grep -E -A3 "error:|warning:" "$log" | head -20 | sed 's/^/    /'
    fi
}

echo "== 4. 渲染关闭（M0 回归）"
run_preset debug "渲染关闭 Debug"

echo "== 5. 渲染开启"
run_preset render-debug "渲染 Debug"
run_preset render-release "渲染 Release -Werror" -DTERRA_WARNINGS_AS_ERRORS=ON
grep -h "graphics.render_deps" "$logs/render-debug.txt" "$logs/render-release.txt" 2>/dev/null | sed 's/^ *//'

echo "== 6. 真实 GPU（桌面会话）"
probe=build/render-release/tests/test_render_deps
if [ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    summary+=("$(printf '%-26s %s' "真实 GPU 探针" "未执行：不在桌面会话中")")
    echo "${summary[-1]}"
elif [ -x "$probe" ]; then
    "$probe" >"$logs/gpu_probe.txt" 2>&1
    rc=$?
    grep -E "^INFO|^FAILED" "$logs/gpu_probe.txt"
    record "真实 GPU 探针" "退出码 $rc" "$(ok $rc)"
else
    record "真实 GPU 探针" "未执行（render-release 构建失败）" 0
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
