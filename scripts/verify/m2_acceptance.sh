#!/usr/bin/env bash
# M2 阶段验收（M2-09）：一条命令复现 M2 的全部自动化证据，沿用 M0-09、M1-11 的做法。
#
# 请在**桌面会话终端**运行（第 9、10 步使用真实 GPU 并打开可见窗口）；自动化图形测试自己使用 Xvfb + llvmpipe（D-022）。
# 依次执行（日志在 build/verify-m2/logs/，屏幕只打印结论）：
#   1. 提交与工作区状态
#   2. 环境清单：工具链与图形环境
#   3. 变异锚点预检：scripts/verify/check_mutation_anchors.py（全部阶段的脚本；锚点失效时提前失败）
#   4. M0 回归：scripts/verify/m0_acceptance.sh
#   5. M2 全量：scripts/verify/m2_08.sh --skip-show——依赖与素材校验；删除构建目录后渲染关闭 Debug（49 项）、
#      渲染开启 Debug 与 Release + -Werror（69 项）；ASan/UBSan；M2-08 变异 8 个；Headless 场景运行
#   6. 历次变异（--mutation-only）：M1 的 11 个脚本（60 个）与 M2-02/03、04、05、06、07（40 个）
#   7. 渲染开启重复运行：Release 全量 CTest 每项至多 3 次
#   8. 依赖边界：Headless 不链接图形库；渲染基准不链接 MuJoCo；渲染侧模块不引用物理；
#      terra_scene、terra_simulation 不包含 GL / 窗口 / 界面 / MuJoCo 头文件
#   9. 真实 GPU：查看器自检（演示场景与场景模式）；帧耗时基线 5 组（只记录）；场景仿真 10 秒的实时性
#  10. 可交互展示（最长 180 秒）：M2 总览——打开场景、播放 / 暂停 / 单步 / 重置、大纲与属性、Headless 对照
# 最后打印汇总；第 1–9 步任一必需项失败时退出码非 0。
# 用法：bash scripts/verify/m2_acceptance.sh [--jobs N] [--skip-m0] [--skip-show]
#   --skip-m0    跳过第 4 步（只在排查问题时使用；正式验收不跳过）
#   --skip-show  跳过第 9、10 步（不在桌面会话时自动跳过并注明）
set -uo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
cd "$root"

jobs=$(nproc)
skip_m0=0
skip_show=0
while [ $# -gt 0 ]; do
    case "$1" in
        --jobs) jobs=$2; shift 2 ;;
        --skip-m0) skip_m0=1; shift ;;
        --skip-show) skip_show=1; shift ;;
        -h|--help) sed -n '2,22p' "$0"; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

out="$root/build/verify-m2"
logs="$out/logs"
rm -rf "$out"
mkdir -p "$logs"
started=$(date +%s)

summary=()
failed=0
record() {  # record <名称> <结果文字> <是否通过 0/1>
    summary+=("$(printf '%-28s %s' "$1" "$2")")
    [ "$3" = 1 ] || failed=1
    echo "${summary[-1]}"
}
note() { summary+=("$(printf '%-28s %s' "$1" "$2")"); echo "${summary[-1]}"; }
ok() { [ "$1" = 0 ] && echo 1 || echo 0; }
desktop() { [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; }

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
    [ "$dirty" = 0 ] || echo "  注意：正式验收应在干净提交上运行（未提交的文件见 $logs/git.txt）"
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
if desktop; then
    bash scripts/gfx_inventory.sh >"$logs/gfx_inventory.txt" 2>&1
    grep -E "OpenGL (renderer|core profile version|version) string|^NVIDIA-SMI|Driver Version|lspci|VGA" "$logs/gfx_inventory.txt" | head -8
    note "图形环境清单" "已采集（$logs/gfx_inventory.txt）"
else
    note "图形环境清单" "未采集：不在桌面会话中"
fi

echo "== 3. 变异锚点预检"
python3 -I scripts/verify/check_mutation_anchors.py >"$logs/anchors.txt" 2>&1
rc=$?
grep -E "^不唯一" "$logs/anchors.txt"
record "变异锚点" "$(tail -1 "$logs/anchors.txt")" "$(ok $rc)"
if [ $rc != 0 ]; then
    echo "锚点失效：变异测试无法注入。请把 $logs/anchors.txt 发给开发者，先修正脚本再验收。" >&2
    printf '%s\n' "${summary[@]}"
    exit 1
fi

echo "== 4. M0 回归（m0_acceptance.sh）"
if [ $skip_m0 = 1 ]; then
    note "M0 回归" "已跳过（--skip-m0，正式验收不可跳过）"
    failed=1
else
    bash scripts/verify/m0_acceptance.sh --jobs "$jobs" >"$logs/m0_acceptance.txt" 2>&1
    rc=$?
    sed -n '/^================ M0 验收汇总/,$p' "$logs/m0_acceptance.txt" | grep -vE "^====|^日志目录" | sed 's/^/  /'
    record "M0 回归" "退出码 $rc（详见 build/verify-m0/logs/）" "$(ok $rc)"
fi

echo "== 5. M2 全量（m2_08.sh --skip-show）"
bash scripts/verify/m2_08.sh --skip-show --jobs "$jobs" >"$logs/m2_08.txt" 2>&1
rc=$?
sed -n '/^================ 汇总/,$p' "$logs/m2_08.txt" | grep -vE "^====|^日志目录|已跳过" | sed 's/^/  /'
record "M2 全量" "退出码 $rc（详见 build/verify-m2-08/logs/）" "$(ok $rc)"

echo "== 6. 历次变异（--mutation-only）"
m1_mutations=0
m1_detected=0
zero_failed=()
for s in m1_03_04 m1_06 m1_07 m1_12 m1_13 m1_14 m1_08a m1_08b m1_09 m1_10 m1_15 m2_02_03 m2_04 m2_05 m2_06 m2_07; do
    log="$logs/mutation_$s.txt"
    bash "scripts/verify/$s.sh" --mutation-only --jobs "$jobs" >"$log" 2>&1
    rc=$?
    line=$(grep -E "^变异测试 " "$log" | tail -1 | sed 's/^变异测试 *//')
    # “已发现（0 项检查失败）”：测试以非 0 退出但没有输出 FAILED，可能是崩溃或用法错误，需要看日志确认
    while IFS= read -r z; do zero_failed+=("$s：$z"); done < <(grep -E "已发现（0 项检查失败）" "$log" | sed 's/^ *变异 //; s/：已发现.*$//')
    pair=$(echo "$line" | sed -n 's/^\([0-9]*\)\/\([0-9]*\) 被发现.*/\1 \2/p')
    if [ -n "$pair" ]; then
        set -- $pair
        m1_detected=$((m1_detected + $1))
        m1_mutations=$((m1_mutations + $2))
    fi
    record "  变异 ${s}" "${line:-无结果（见 $log）}，退出码 $rc" "$([ $rc = 0 ] && [ -n "$pair" ] && echo 1 || echo 0)"
    [ $rc = 0 ] || grep -E "^  变异 .*(未被发现|注入失败|编译失败)" "$log" | sed 's/^/    /'
done
m15=$(grep -E "^变异测试 " "$logs/m2_08.txt" | tail -1 | sed -n 's/^变异测试 *\([0-9]*\)\/\([0-9]*\) 被发现.*/\1 \2/p')
if [ -n "$m15" ]; then
    set -- $m15
    m1_detected=$((m1_detected + $1))
    m1_mutations=$((m1_mutations + $2))
fi
note "M1、M2 变异合计" "$m1_detected/$m1_mutations 被发现（含第 5 步 M2-08 的 8 个；另有第 4 步 M0 的 11 个）"
if [ ${#zero_failed[@]} -gt 0 ]; then
    note "需查看的变异" "${#zero_failed[@]} 个以非 0 退出但没有 FAILED 行：${zero_failed[*]}"
fi

echo "== 7. 渲染开启重复运行"
ctest --test-dir build/render-release --repeat until-fail:3 >"$logs/repeat.txt" 2>&1
rc=$?
record "重复运行（每项至多 3 次）" "$(grep -E "tests passed|tests failed" "$logs/repeat.txt" | tail -1)" "$(ok $rc)"
[ $rc = 0 ] || grep -E "^\s+[0-9]+ - " "$logs/repeat.txt" | head -10 | sed 's/^/    /'

echo "== 8. 依赖边界"
bad=0
: >"$logs/ldd.txt"
for bin in build/release/apps/headless/terra_headless build/release/apps/bench/terra_bench; do
    if [ ! -x "$bin" ]; then echo "$bin 不存在" >>"$logs/ldd.txt"; bad=1; continue; fi
    { echo "== $bin"; ldd "$bin"; } >>"$logs/ldd.txt" 2>&1
    if ldd "$bin" | grep -Eq "lib(GL|EGL|OSMesa|X11|vulkan|glfw|wayland)[.-]"; then
        echo "  $bin 链接了图形库"
        bad=1
    fi
done
record "Headless 无图形库" "$([ $bad = 0 ] && echo "terra_headless、terra_bench 均无图形库" || echo "存在图形库或程序缺失（见 $logs/ldd.txt）")" \
    "$([ $bad = 0 ] && echo 1 || echo 0)"
bad=0
# M2-07 起 terra_viewer 按设计链接物理后端（开发计划 9.5 节第 7 条），这里只检查渲染基准程序
for bin in build/render-release/apps/bench_render/terra_render_bench; do
    if [ ! -x "$bin" ]; then echo "$bin 不存在" >>"$logs/ldd.txt"; bad=1; continue; fi
    { echo "== $bin"; ldd "$bin"; } >>"$logs/ldd.txt" 2>&1
    if ldd "$bin" | grep -q "libmujoco"; then
        echo "  $bin 链接了 MuJoCo"
        bad=1
    fi
done
record "渲染程序无 MuJoCo" "$([ $bad = 0 ] && echo "terra_render_bench 不链接 libmujoco（M2 起查看器按设计链接物理）" || echo "存在 MuJoCo 依赖或程序缺失（见 $logs/ldd.txt）")" \
    "$([ $bad = 0 ] && echo 1 || echo 0)"
# M2-07 起查看器按设计使用物理后端（开发计划 9.5 节第 7 条），不在此列；terra_viewer_scene 的源码（demo_scene、scene_view）仍不得引用物理
render_side=(engine/render engine/asset engine/ui engine/view engine/platform engine/input engine/settings apps/bench_render
             apps/viewer/demo_scene.hpp apps/viewer/demo_scene.cpp apps/viewer/scene_view.hpp apps/viewer/scene_view.cpp)
grep -rnE "terra::physics|terra_physics|mujoco|terra/physics/" "${render_side[@]}" >"$logs/render_physics_refs.txt" 2>&1
hits=$(grep -c . "$logs/render_physics_refs.txt")
head -5 "$logs/render_physics_refs.txt" | sed 's/^/    /'
record "渲染侧不引用物理" "$([ "$hits" = 0 ] && echo "渲染侧模块与 terra_viewer_scene 中 0 处引用" || echo "$hits 处引用（见 $logs/render_physics_refs.txt）")" \
    "$([ "$hits" = 0 ] && echo 1 || echo 0)"
grep -rnE "#include *[<\"](mujoco/|glad/|GL/|GLFW/|imgui)" engine/scene engine/simulation >"$logs/scene_sim_includes.txt" 2>&1
hits=$(grep -c . "$logs/scene_sim_includes.txt")
record "场景与仿真模块边界" "$([ "$hits" = 0 ] && echo "terra_scene、terra_simulation 不包含 MuJoCo、GL、GLFW、ImGui 头文件" || echo "$hits 处违规（见 $logs/scene_sim_includes.txt）")" \
    "$([ "$hits" = 0 ] && echo 1 || echo 0)"
missing=()
for t in boundary.no_mujoco_in_public_headers boundary.render_isolation headless.no_graphics_libs; do
    grep -qE "Test +#[0-9]+: ${t//./\\.} .*Passed" "$logs/repeat.txt" || missing+=("$t")
done
if [ ${#missing[@]} = 0 ]; then
    record "边界测试" "no_mujoco_in_public_headers、render_isolation、no_graphics_libs 在第 7 步中通过" 1
else
    record "边界测试" "未在第 7 步中找到通过记录：${missing[*]}" 0
fi

viewer=build/render-release/apps/viewer/terra_viewer
bench=build/render-release/apps/bench_render/terra_render_bench
echo "== 9. 真实 GPU（桌面会话）"
if [ $skip_show = 1 ]; then
    note "真实 GPU 自检" "已跳过（--skip-show）"
elif ! desktop; then
    note "真实 GPU 自检" "未执行：不在桌面会话中"
elif [ -x "$viewer" ]; then
    "$viewer" --frames 40 --self-test --settings "$out/selftest/settings.json" >"$logs/desktop_selftest.txt" 2>&1
    rc=$?
    grep -E "^INFO NVIDIA|^INFO .*OpenGL|^INFO 显存|^PASS 性能图表|^PASS 视口中心|^FAILED" "$logs/desktop_selftest.txt" | head -8
    record "真实 GPU 自检" "退出码 $rc" "$(ok $rc)"
    "$viewer" --frames 60 --self-test --scene assets/tests/scene/two_carts.terra-scene.json \
        >"$logs/desktop_scene_selftest.txt" 2>&1
    rc=$?
    grep -E "^PASS 查看器中的仿真|^FAILED" "$logs/desktop_scene_selftest.txt"
    record "真实 GPU 场景自检" "退出码 $rc" "$(ok $rc)"
    "$viewer" --scene assets/tests/scene/drop.terra-scene.json --demo-drive --seconds 10 \
        --settings "$out/realtime/settings.json" >"$logs/realtime.txt" 2>&1
    rc=$?
    line=$(grep -E "^INFO 仿真：" "$logs/realtime.txt" | tail -1 | sed 's/^INFO //')
    note "场景实时性（10 秒）" "${line:-无结果}，退出码 $rc"
    if [ -x "$bench" ]; then
        echo "  帧耗时基线（每项 600 帧，预热 60 帧，不经过垂直同步；只记录，不判定）："
        while read -r tag args; do
            "$bench" $args --output "$logs/bench_$tag.jsonl" >"$logs/bench_$tag.txt" 2>&1
            rc=$?
            line=$(grep -E "^INFO .*整帧中位数" "$logs/bench_$tag.txt" | tail -1 | sed 's/^INFO //')
            note "基准 $tag" "${line:-失败，退出码 $rc（见 $logs/bench_$tag.txt）}"
            [ $rc = 0 ] || failed=1
        done <<'BENCH'
1080p-默认 --width 1920 --height 1080 --scene outdoor
4K-默认 --width 3840 --height 2160 --scene outdoor
4K-无MSAA --width 3840 --height 2160 --scene outdoor --msaa 1
4K-无阴影 --width 3840 --height 2160 --scene outdoor --no-shadows
4K-程序场景 --width 3840 --height 2160 --scene procedural
BENCH
    fi
else
    record "真实 GPU 自检" "未执行（render-release 构建失败）" 0
fi

echo "== 10. 可交互展示"
if [ $skip_show = 1 ]; then
    note "可交互展示" "已跳过（--skip-show）"
elif ! desktop; then
    note "可交互展示" "未执行：不在桌面会话中"
elif [ -x "$viewer" ]; then
    echo "（即将打开 terra_viewer，最长 180 秒；设置文件：$out/show/settings.json（全新））"
    echo "  M2 总览，请逐项看一眼："
    echo "  - 打开的是“下落的方块”场景，处于暂停：按空格播放，方块落地翻滚；句号单步、Ctrl+R 重置、工具栏调速度"
    echo "  - 状态栏：仿真时间、步数、实时因子（约 1.00）、过载次数"
    echo "  - 大纲中点选 cube_2/block：视口高亮，属性面板显示位姿与速度随仿真变化"
    echo "  - 文件 → 打开场景…（Ctrl+O）打开 two_carts.terra-scene.json；文件 → 最近打开 中有两条记录"
    echo "  - 两辆小车静止（没有控制）：可验证暂停、单步、重置与环境、相机随场景切换"
    "$viewer" --scene assets/tests/scene/drop.terra-scene.json --seconds 180 --settings "$out/show/settings.json" \
        >"$logs/show.txt" 2>&1
    rc=$?
    grep -E "^INFO" "$logs/show.txt" | head -12
    frames=$(sed -n 's/^INFO 结束：\([0-9]*\) 帧，\([0-9.]*\) 秒$/\1 \2/p' "$logs/show.txt")
    if [ -n "$frames" ]; then
        set -- $frames
        note "展示平均帧率" "$(awk -v f="$1" -v t="$2" 'BEGIN { if (t > 0) printf "%.1f FPS（%d 帧 / %.1f 秒，含垂直同步）", f / t, f, t }')"
    fi
    record "可交互展示" "退出码 $rc" "$(ok $rc)"
else
    record "可交互展示" "未执行（render-release 构建失败）" 0
fi

{
    echo "### CPU"
    grep -m1 "model name" /proc/cpuinfo
    echo "cores: $(nproc)"
    grep MemTotal /proc/meminfo
    echo "governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo 未知)"
} >"$logs/machine.txt"

elapsed=$(( $(date +%s) - started ))
echo
echo "================ M2 验收汇总 ================"
printf '%s\n' "${summary[@]}"
echo "用时：$((elapsed / 60)) 分 $((elapsed % 60)) 秒"
echo "日志目录：$logs"
if [ $failed = 0 ]; then
    echo "结论：全部通过"
else
    echo "结论：存在失败项"
fi
exit $failed
