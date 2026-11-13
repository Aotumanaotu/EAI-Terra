#!/usr/bin/env bash
# 下载 MuJoCo 官方源码包，校验 sha256 后解压到 third_party/mujoco/src/。
# 这是显式联网步骤；Terra 的 CMake 构建本身不联网。
# 用法：scripts/mujoco/fetch.sh
set -euo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
base="$root/third_party/mujoco"
# shellcheck source=/dev/null
. "$base/mujoco.lock"

archive="$base/downloads/mujoco-$MUJOCO_VERSION.tar.gz"
src="$base/src/mujoco-$MUJOCO_VERSION"

if [ -d "$src" ]; then
    echo "已存在：$src（如需重新获取，请先删除该目录）"
    exit 0
fi

mkdir -p "$base/downloads" "$base/src"
if [ ! -f "$archive" ]; then
    echo "下载 $MUJOCO_URL"
    curl -fSL --retry 3 -o "$archive.part" "$MUJOCO_URL"
    mv "$archive.part" "$archive"
fi

echo "校验 sha256"
if ! echo "$MUJOCO_SHA256  $archive" | sha256sum -c --quiet -; then
    echo "错误：sha256 不匹配，已删除下载文件：$archive" >&2
    rm -f "$archive"
    exit 1
fi

tmp=$(mktemp -d "$base/src/.extract.XXXXXX")
trap 'rm -rf "$tmp"' EXIT
tar -xzf "$archive" -C "$tmp"
mv "$tmp/mujoco-$MUJOCO_VERSION" "$src"
echo "源码就绪：$src"
