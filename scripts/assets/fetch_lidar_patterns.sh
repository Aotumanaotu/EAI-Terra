#!/usr/bin/env bash
# 下载 3D 激光雷达的扫描图案（M5-11）并按 assets/sensors/lidar3d_patterns.lock 校验 sha256，放到 assets/sensors/<路径>（不提交）。
# 已存在且校验一致的文件跳过。这是显式联网步骤；CMake 构建不联网。
# 离线时：在有网络的机器上下载锁定文件中的地址，放到对应路径后再运行本脚本校验。
# 用法：scripts/assets/fetch_lidar_patterns.sh
set -euo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
dir="$root/assets/sensors"
status=0
ok=0
total=0
while IFS='|' read -r rel url sha source; do
    case "$rel" in ''|'#'*) continue ;; esac
    total=$((total + 1))
    dest="$dir/$rel"
    mkdir -p "$(dirname "$dest")"
    if [ -f "$dest" ] && echo "$sha  $dest" | sha256sum -c --quiet - 2>/dev/null; then
        ok=$((ok + 1))
        continue
    fi
    # GitHub 原始文件地址不通时依次换 jsDelivr 的镜像（同一提交的同一文件，按 sha256 校验，内容不会被替换）
    mirrors=("$url")
    if [[ "$url" =~ ^https://raw\.githubusercontent\.com/([^/]+)/([^/]+)/([^/]+)/(.+)$ ]]; then
        gh="${BASH_REMATCH[1]}/${BASH_REMATCH[2]}@${BASH_REMATCH[3]}/${BASH_REMATCH[4]}"
        mirrors+=("https://cdn.jsdelivr.net/gh/$gh" "https://fastly.jsdelivr.net/gh/$gh")
    fi
    done_one=0
    for m in "${mirrors[@]}"; do
        echo "下载 $m"
        if curl -fSL --retry 2 --connect-timeout 15 -o "$dest.part" "$m" && echo "$sha  $dest.part" | sha256sum -c --quiet - 2>/dev/null; then
            mv "$dest.part" "$dest"
            ok=$((ok + 1))
            done_one=1
            break
        fi
        rm -f "$dest.part"
        echo "  失败（下载不通或 sha256 不匹配），换下一个地址" >&2
    done
    if [ $done_one = 0 ]; then
        echo "错误：$rel 无法获取（来源：$source）。离线时请在有网络的机器上下载 $url，放到 $dest 后重新运行本脚本" >&2
        status=1
    fi
done <"$dir/lidar3d_patterns.lock"
echo "就绪：$ok/$total 个文件（Livox 扫描图案，MIT）"
exit $status
