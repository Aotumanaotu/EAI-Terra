#!/usr/bin/env bash
# 准备参考机器人（M3-01）：按 assets/robots/robots.lock 把 MuJoCo Menagerie 中三台机器人的文件放到 assets/robots/<目录>/
# （不提交），逐个校验 sha256。来源优先用用户下载的原件 ROBOTS/mujoco_menagerie/（开发计划 10.4 节的命令）；
# 没有原件或原件中的文件不一致时，从锁定提交的 raw.githubusercontent.com 地址下载（显式联网步骤；CMake 构建不联网）。
# 已存在且校验一致的文件跳过。
# M4-05 起：锁定记录可以有第 5 个字段，为原件相对 ROBOTS/ 的路径（宇树 unitree_mujoco 的模型在 ROBOTS/unitree_mujoco/ 中）；
# 没有第 5 个字段时，原件在 ROBOTS/mujoco_menagerie/<目录>/<相对路径>。
# 用法：bash scripts/assets/fetch_robots.sh [--offline]   （--offline：只用 ROBOTS/ 中的原件，不联网）
set -uo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
dir="$root/assets/robots"
source_dir="$root/ROBOTS/mujoco_menagerie"
offline=0
[ "${1:-}" = "--offline" ] && offline=1

ok=0
total=0
copied=0
downloaded=0
status=0
check() { echo "$1  $2" | sha256sum -c --quiet - 2>/dev/null; }
while IFS='|' read -r robot rel url sha src; do
    case "$robot" in ''|'#'*) continue ;; esac
    total=$((total + 1))
    dest="$dir/$robot/$rel"
    mkdir -p "$(dirname "$dest")"
    if [ -f "$dest" ] && check "$sha" "$dest"; then
        ok=$((ok + 1))
        continue
    fi
    original="$source_dir/$robot/$rel"
    [ -n "${src:-}" ] && original="$root/ROBOTS/$src"
    if [ -f "$original" ] && check "$sha" "$original"; then
        cp "$original" "$dest"
        ok=$((ok + 1))
        copied=$((copied + 1))
        continue
    fi
    if [ $offline = 1 ]; then
        echo "错误：$robot/$rel 的原件（$original）不存在或 sha256 不一致（--offline 不联网）" >&2
        status=1
        continue
    fi
    if curl -fsSL --retry 3 -o "$dest.part" "$url" && check "$sha" "$dest.part"; then
        mv "$dest.part" "$dest"
        ok=$((ok + 1))
        downloaded=$((downloaded + 1))
    else
        rm -f "$dest.part"
        echo "错误：$robot/$rel 下载失败或 sha256 不一致（$url）" >&2
        status=1
    fi
done <"$dir/robots.lock"
echo "就绪：$ok/$total 个文件（从 ROBOTS/ 复制 $copied 个，下载 $downloaded 个；Menagerie、unitree_mujoco、TurtleBot 4 与 Create 3，BSD-3-Clause / Apache-2.0）"
exit $status
