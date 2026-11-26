#!/usr/bin/env bash
# 下载传感器的外观模型（M5-11）并按 assets/sensors/sensor_models.lock 校验 sha256，放到 assets/sensors/models/（不提交）；
# 再用 scripts/assets/dae_to_glb.py 转为 .glb（Terra 的 glTF 绑定只读 glTF）。已存在且校验一致的文件跳过。
# 这是显式联网步骤；CMake 构建不联网。没有外观模型时雷达照常工作，只是不显示外壳。
# 离线时：在有网络的机器上下载锁定文件中的地址，放到对应路径后再运行本脚本校验与转换。
# 用法：scripts/assets/fetch_sensor_models.sh
set -euo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
dir="$root/assets/sensors/models"
lock="$root/assets/sensors/sensor_models.lock"
status=0
ok=0
total=0
while IFS='|' read -r rel url sha source; do
    case "$rel" in ''|'#'*|'='*) continue ;; esac
    total=$((total + 1))
    dest="$dir/$rel"
    mkdir -p "$(dirname "$dest")"
    if [ -f "$dest" ] && echo "$sha  $dest" | sha256sum -c --quiet - 2>/dev/null; then
        ok=$((ok + 1))
        continue
    fi
    # GitHub 原始文件地址不通时依次换 jsDelivr 的镜像（同一提交的同一文件，按 sha256 校验）
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
done <"$lock"

converted=0
outputs=0
while IFS='|' read -r rel input sha; do
    case "$rel" in '='*) ;; *) continue ;; esac
    rel=${rel#=}
    outputs=$((outputs + 1))
    out="$dir/$rel"
    if [ ! -f "$dir/$input" ]; then
        echo "错误：缺少 $dir/$input，无法生成 $rel" >&2
        status=1
        continue
    fi
    if [ ! -f "$out" ] || [ "$dir/$input" -nt "$out" ] || [ "$root/scripts/assets/dae_to_glb.py" -nt "$out" ]; then
        if ! python3 -I "$root/scripts/assets/dae_to_glb.py" "$dir/$input" "$out.part" --name "${rel%.glb}"; then
            rm -f "$out.part"
            echo "错误：转换 $input 失败" >&2
            status=1
            continue
        fi
        mv "$out.part" "$out"
    fi
    converted=$((converted + 1))
    echo "$sha  $out" | sha256sum -c --quiet - 2>/dev/null || echo "警告：$rel 的 sha256 与锁定值不同（Python 版本差异？），仍然使用" >&2
done <"$lock"
echo "就绪：$ok/$total 个文件，转换 $converted/$outputs 个模型（Livox Mid-360 外观，MIT）"
exit $status
