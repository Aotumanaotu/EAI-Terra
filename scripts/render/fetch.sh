#!/usr/bin/env bash
# 下载渲染依赖（GLFW、GLM、Dear ImGui，以及用于复核的 glad 生成器）源码包，校验 sha256 后解压到 third_party/render/src/。
# 版本与 sha256 见 third_party/render/render.lock。这是显式联网步骤；Terra 的 CMake 构建本身不联网。
# 用法：scripts/render/fetch.sh
set -euo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
base="$root/third_party/render"
# shellcheck source=/dev/null
. "$base/render.lock"

mkdir -p "$base/downloads" "$base/src"

fetch() {  # fetch <名称> <版本> <URL> <sha256> <解压后的目录名>
    local name=$1 version=$2 url=$3 sha=$4 dir=$5
    local archive="$base/downloads/$name-$version.tar.gz"
    local dest="$base/src/$dir"
    if [ -d "$dest" ]; then
        echo "已存在：$dest（如需重新获取，请先删除该目录）"
        return
    fi
    if [ ! -f "$archive" ]; then
        echo "下载 $url"
        curl -fSL --retry 3 -o "$archive.part" "$url"
        mv "$archive.part" "$archive"
    fi
    echo "校验 $name sha256"
    if ! echo "$sha  $archive" | sha256sum -c --quiet -; then
        echo "错误：$name 的 sha256 不匹配，已删除下载文件：$archive" >&2
        rm -f "$archive"
        exit 1
    fi
    local tmp
    tmp=$(mktemp -d "$base/src/.extract.XXXXXX")
    tar -xzf "$archive" -C "$tmp"
    mv "$tmp/$dir" "$dest"
    rmdir "$tmp"
    echo "源码就绪：$dest"
}

fetch glfw "$GLFW_VERSION" "$GLFW_URL" "$GLFW_SHA256" "glfw-$GLFW_VERSION"
fetch glm "$GLM_VERSION" "$GLM_URL" "$GLM_SHA256" "glm-$GLM_VERSION"
fetch glad "$GLAD_VERSION" "$GLAD_URL" "$GLAD_SHA256" "glad-$GLAD_VERSION"
fetch imgui "$IMGUI_VERSION" "$IMGUI_URL" "$IMGUI_SHA256" "imgui-$IMGUI_VERSION"
