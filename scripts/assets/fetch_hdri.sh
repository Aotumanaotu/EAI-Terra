#!/usr/bin/env bash
# 下载 HDRI 环境贴图（M1-08b）并按 assets/hdri/hdri.lock 校验 sha256，放到 assets/hdri/（.hdr 文件不提交）。
# 已存在且校验一致的文件跳过。这是显式联网步骤；CMake 构建不联网。
# 用法：scripts/assets/fetch_hdri.sh
set -euo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
dir="$root/assets/hdri"
status=0
while IFS='|' read -r name url sha author purpose; do
    case "$name" in ''|'#'*) continue ;; esac
    dest="$dir/$name"
    if [ -f "$dest" ] && echo "$sha  $dest" | sha256sum -c --quiet - 2>/dev/null; then
        echo "已存在且一致：$name（$purpose，作者 $author，CC0）"
        continue
    fi
    echo "下载 $url"
    curl -fSL --retry 3 -o "$dest.part" "$url"
    if echo "$sha  $dest.part" | sha256sum -c --quiet - 2>/dev/null; then
        mv "$dest.part" "$dest"
        echo "就绪：$name（$purpose，作者 $author，CC0）"
    else
        rm -f "$dest.part"
        echo "错误：$name 的 sha256 不匹配" >&2
        status=1
    fi
done <"$dir/hdri.lock"
exit $status
