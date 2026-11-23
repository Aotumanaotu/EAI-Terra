#!/usr/bin/env bash
# 下载 glTF 测试资产（M1-09）并按 assets/models/models.lock 校验 sha256，放到 assets/models/<模型>/（不提交）。
# 已存在且校验一致的文件跳过。这是显式联网步骤；CMake 构建不联网。
# 用法：scripts/assets/fetch_models.sh
set -euo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
dir="$root/assets/models"
status=0
ok=0
total=0
while IFS='|' read -r model rel url sha; do
    case "$model" in ''|'#'*) continue ;; esac
    total=$((total + 1))
    dest="$dir/$model/$rel"
    mkdir -p "$(dirname "$dest")"
    if [ -f "$dest" ] && echo "$sha  $dest" | sha256sum -c --quiet - 2>/dev/null; then
        ok=$((ok + 1))
        continue
    fi
    echo "下载 $url"
    curl -fSL --retry 3 -o "$dest.part" "$url"
    if echo "$sha  $dest.part" | sha256sum -c --quiet - 2>/dev/null; then
        mv "$dest.part" "$dest"
        ok=$((ok + 1))
    else
        rm -f "$dest.part"
        echo "错误：$model/$rel 的 sha256 不匹配" >&2
        status=1
    fi
done <"$dir/models.lock"
echo "就绪：$ok/$total 个文件（CC0，Poly Haven）"
exit $status
