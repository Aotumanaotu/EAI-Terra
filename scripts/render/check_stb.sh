#!/usr/bin/env bash
# 复核已提交的 stb_image 头文件与许可（M1-08b）：按 third_party/stb/stb.lock 校验 sha256。
# 用法：scripts/render/check_stb.sh [--online]
#   --online  另外从官方发布页下载同名文件，确认与锁定的 sha256 一致（显式联网步骤）。
set -euo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
base="$root/third_party/stb"
# shellcheck source=/dev/null
. "$base/stb.lock"

status=0
verify() {  # verify <文件> <sha256>
    if echo "$2  $1" | sha256sum -c --quiet - 2>/dev/null; then
        echo "一致：${1#"$root"/}"
    else
        echo "错误：sha256 不匹配或文件缺失：${1#"$root"/}" >&2
        status=1
    fi
}

verify "$base/stb_image.h" "$STB_IMAGE_SHA256"
verify "$base/LICENSE" "$STB_LICENSE_SHA256"
verify "$base/stb_image_write.h" "$STB_IMAGE_WRITE_SHA256"
grep -q "stb_image_write - v$STB_IMAGE_WRITE_VERSION" "$base/stb_image_write.h" || { echo "错误：stb_image_write.h 版本不是 $STB_IMAGE_WRITE_VERSION" >&2; status=1; }
grep -q "stb_image - v$STB_IMAGE_VERSION" "$base/stb_image.h" || { echo "错误：stb_image.h 版本不是 $STB_IMAGE_VERSION" >&2; status=1; }

if [ "${1:-}" = "--online" ]; then
    tmp=$(mktemp -d)
    trap 'rm -rf "$tmp"' EXIT
    for pair in "stb_image.h|$STB_IMAGE_URL|$STB_IMAGE_SHA256" "LICENSE|$STB_LICENSE_URL|$STB_LICENSE_SHA256" "stb_image_write.h|$STB_IMAGE_WRITE_URL|$STB_IMAGE_WRITE_SHA256"; do
        IFS='|' read -r name url sha <<<"$pair"
        curl -fsSL --retry 3 -o "$tmp/$name" "$url"
        if echo "$sha  $tmp/$name" | sha256sum -c --quiet - 2>/dev/null; then
            echo "官方下载一致：$name"
        else
            echo "错误：官方下载的 $name 与锁定的 sha256 不一致" >&2
            status=1
        fi
    done
fi
exit $status
