#!/usr/bin/env bash
# M0 阶段验收（M0-09）：一条命令复现 M0 的全部自动化证据。
#
# 整个脚本在清除 DISPLAY / WAYLAND_DISPLAY 的环境中运行。依次执行（日志在 build/verify-m0/logs/）：
#   1. 提交与工作区状态（若为 git 仓库）
#   2. 环境清单 scripts/env_inventory.sh（含工具 dpkg 来源，补 M0-01 遗留项）
#   3. MuJoCo 锁定核对：mujoco.lock 状态、源码包 sha256、安装目录版本
#   4. scripts/verify/m0_08.sh：删除构建目录后 Debug、Release + -Werror、ASan/UBSan 全量 CTest，
#      M0-08 变异测试，主场景与辅助用例的 METRIC / MEMORY 输出
#   5. scripts/verify/m0_07.sh --skip-build：M0-07 的 8 项变异测试与 hinge_motor 输出
#   6. 重复运行：Release 全量 CTest 每项最多重复 5 次（--repeat until-fail:5），检查偶发失败
#   7. 运行依赖：ldd terra_headless / terra_bench，确认无图形库
#   8. 性能基线（只记录，不判定）：terra_bench，立方体与铰链夹具
# 最后打印汇总；第 1–7 步任一失败时退出码非 0。
#
# 依赖：Ubuntu 22.04 / x86-64，GCC 11、CMake 3.22、Ninja、python3，third_party/mujoco/install（见 README）。
# 用法：bash scripts/verify/m0_acceptance.sh [--jobs N]
set -uo pipefail

if [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    exec env -u DISPLAY -u WAYLAND_DISPLAY bash "$0" "$@"
fi

root=$(cd "$(dirname "$0")/../.." && pwd)
cd "$root"

jobs=$(nproc)
while [ $# -gt 0 ]; do
    case "$1" in
        --jobs) jobs=$2; shift 2 ;;
        -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/verify-m0"
logs="$out/logs"
rm -rf "$out"
mkdir -p "$logs"

summary=()
failed=0
record() {  # record <名称> <结果文字> <是否通过 0/1>
    summary+=("$(printf '%-24s %s' "$1" "$2")")
    [ "$3" = 1 ] || failed=1
    echo "${summary[-1]}"
}

echo "== 1. 提交"
if git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    {
        echo "commit: $(git rev-parse HEAD)"
        echo "branch: $(git rev-parse --abbrev-ref HEAD)"
        echo "未提交的改动："
        git status --porcelain
    } >"$logs/git.txt"
    dirty=$(git status --porcelain | wc -l)
    record "提交" "$(git rev-parse --short HEAD)，未提交改动 $dirty 个文件" 1
else
    echo "不是 git 仓库" >"$logs/git.txt"
    record "提交" "不是 git 仓库，无法关联提交" 1
fi

echo "== 2. 环境清单"
if bash scripts/env_inventory.sh >"$logs/env_inventory.txt" 2>&1; then
    sed -n '/## 工具链/,/## 显示与图形/p' "$logs/env_inventory.txt" | grep -v "^## 显示"
    record "环境清单" "已采集（$logs/env_inventory.txt）" 1
else
    record "环境清单" "脚本执行失败" 0
fi

echo "== 3. MuJoCo 锁定核对"
lock=third_party/mujoco/mujoco.lock
# shellcheck source=/dev/null
. "$lock"
archive="third_party/mujoco/downloads/mujoco-$MUJOCO_VERSION.tar.gz"
header="third_party/mujoco/install/include/mujoco/mujoco.h"
{
    grep "^# 状态" "$lock"
    echo "MUJOCO_VERSION=$MUJOCO_VERSION"
    echo "MUJOCO_SHA256=$MUJOCO_SHA256"
    if [ -f "$archive" ]; then
        echo "archive_sha256=$(sha256sum "$archive" | cut -d' ' -f1)"
    else
        echo "archive_sha256=<源码包不存在：$archive>"
    fi
    grep -m1 "#define mjVERSION_HEADER" "$header"
} >"$logs/mujoco_lock.txt" 2>&1
cat "$logs/mujoco_lock.txt"
if [ -f "$archive" ] && [ "$(sha256sum "$archive" | cut -d' ' -f1)" = "$MUJOCO_SHA256" ]; then
    record "MuJoCo 源码包" "sha256 与 mujoco.lock 一致" 1
else
    record "MuJoCo 源码包" "sha256 不一致或源码包缺失" 0
fi

echo "== 4. 全量测试、M0-08 变异与场景判定（m0_08.sh）"
bash scripts/verify/m0_08.sh --jobs "$jobs" >"$logs/m0_08.txt" 2>&1
rc=$?
sed -n '/^================ 汇总/,$p' "$logs/m0_08.txt"
grep -E "^(METRIC|MEMORY|INFO)" "$logs/m0_08.txt" >"$logs/metrics.txt"
record "m0_08.sh" "退出码 $rc（详见 build/verify-m0-08/logs/）" "$([ $rc = 0 ] && echo 1 || echo 0)"

echo "== 5. M0-07 变异测试（m0_07.sh --skip-build）"
bash scripts/verify/m0_07.sh --skip-build --jobs "$jobs" >"$logs/m0_07.txt" 2>&1
rc=$?
sed -n '/^================ 汇总/,$p' "$logs/m0_07.txt"
record "m0_07.sh 变异" "退出码 $rc（详见 build/verify-m0-07/logs/）" "$([ $rc = 0 ] && echo 1 || echo 0)"

echo "== 6. 重复运行"
ctest --preset release --repeat until-fail:5 >"$logs/repeat.txt" 2>&1
rc=$?
record "重复运行（每项至多 5 次）" "$(grep -E "tests passed|tests failed" "$logs/repeat.txt" | tail -1)" \
    "$([ $rc = 0 ] && echo 1 || echo 0)"

echo "== 7. 运行依赖"
bad=0
for bin in build/release/apps/headless/terra_headless build/release/apps/bench/terra_bench; do
    if [ ! -x "$bin" ]; then
        echo "$bin 不存在" >>"$logs/ldd.txt"
        bad=1
        continue
    fi
    { echo "== $bin"; ldd "$bin"; } >>"$logs/ldd.txt" 2>&1
    if ldd "$bin" | grep -Eq "lib(GL|EGL|OSMesa|X11|vulkan|glfw)[.-]"; then
        bad=1
    fi
done
grep -E "==|mujoco|libstdc|libc\.so" "$logs/ldd.txt"
record "运行依赖" "$([ $bad = 0 ] && echo "无图形库，libmujoco 来自本地安装" || echo "存在图形库或程序缺失（见 $logs/ldd.txt）")" \
    "$([ $bad = 0 ] && echo 1 || echo 0)"

echo "== 8. 性能基线（只记录）"
bench=build/release/apps/bench/terra_bench
if [ -x "$bench" ]; then
    {
        echo "### falling_cube, 每步输出"
        "$bench" --model assets/tests/falling_cube.xml --steps 5000 --repeat 20 --warmup 2 --sample-every 1
        echo "### falling_cube, 每 100 步输出"
        "$bench" --model assets/tests/falling_cube.xml --steps 5000 --repeat 20 --warmup 2 --sample-every 100
        echo "### hinge_motor, 每步输出"
        "$bench" --model assets/tests/hinge_motor.xml --steps 5000 --repeat 20 --warmup 2 --sample-every 1
    } >"$logs/bench.txt" 2>&1
    cat "$logs/bench.txt"
    summary+=("$(printf '%-24s %s' "性能基线" "已记录（不判定，见 $logs/bench.txt）")")
else
    summary+=("$(printf '%-24s %s' "性能基线" "未执行（terra_bench 不存在）")")
fi
{
    echo "### CPU"
    grep -m1 "model name" /proc/cpuinfo
    echo "cores: $(nproc)"
    grep MemTotal /proc/meminfo
    echo "governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo 未知)"
} >>"$logs/bench.txt"

echo
echo "================ M0 验收汇总 ================"
printf '%s\n' "${summary[@]}"
echo "日志目录：$logs"
if [ $failed = 0 ]; then
    echo "结论：全部通过"
else
    echo "结论：存在失败项"
fi
exit $failed
