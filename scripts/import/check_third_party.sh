#!/usr/bin/env bash
# 复核模型导入与资源库用到的第三方源码：third_party/tinyxml2、third_party/xacro（M4-08a）与 third_party/miniz（M4-08c），
# 按 *.lock 校验 sha256。
# 用法：scripts/import/check_third_party.sh [--online]
#   --online  另外从官方下载（tinyxml2 的固定提交、xacro 的 wheel、miniz 的发布包），确认与锁定的 sha256 一致（显式联网步骤）。
set -euo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
# shellcheck source=/dev/null
. "$root/third_party/tinyxml2/tinyxml2.lock"
# shellcheck source=/dev/null
. "$root/third_party/xacro/xacro.lock"
# shellcheck source=/dev/null
. "$root/third_party/miniz/miniz.lock"

status=0
verify() {  # verify <文件> <sha256>
    if echo "$2  $1" | sha256sum -c --quiet - 2>/dev/null; then
        echo "一致：${1#"$root"/}"
    else
        echo "错误：sha256 不匹配或文件缺失：${1#"$root"/}" >&2
        status=1
    fi
}

t="$root/third_party/tinyxml2"
verify "$t/tinyxml2.h" "$TINYXML2_H_SHA256"
verify "$t/tinyxml2.cpp" "$TINYXML2_CPP_SHA256"
verify "$t/LICENSE.txt" "$TINYXML2_LICENSE_SHA256"
x="$root/third_party/xacro"
verify "$x/xacro/cli.py" "$XACRO_CLI_SHA256"
verify "$x/xacro/color.py" "$XACRO_COLOR_SHA256"
verify "$x/xacro/__init__.py" "$XACRO_INIT_SHA256"
verify "$x/xacro/substitution_args.py" "$XACRO_SUBSTITUTION_ARGS_SHA256"
verify "$x/xacro/xmlutils.py" "$XACRO_XMLUTILS_SHA256"
verify "$x/LICENSE" "$XACRO_LICENSE_SHA256"
m="$root/third_party/miniz"
verify "$m/miniz.c" "$MINIZ_C_SHA256"
verify "$m/miniz.h" "$MINIZ_H_SHA256"
verify "$m/LICENSE" "$MINIZ_LICENSE_SHA256"

if [ "${1:-}" = "--online" ]; then
    tmp=$(mktemp -d)
    trap 'rm -rf "$tmp"' EXIT
    for name in tinyxml2.h tinyxml2.cpp LICENSE.txt; do
        curl -fsSL --retry 3 -o "$tmp/$name" "$TINYXML2_URL_BASE/$name"
        if cmp -s "$tmp/$name" "$t/$name"; then echo "官方下载一致：tinyxml2/$name"; else echo "错误：tinyxml2/$name 与官方不同" >&2; status=1; fi
    done
    curl -fsSL --retry 3 -o "$tmp/xacro.whl" "$XACRO_WHEEL_URL"
    if echo "$XACRO_WHEEL_SHA256  $tmp/xacro.whl" | sha256sum -c --quiet - 2>/dev/null; then
        echo "官方下载一致：xacro wheel"
        python3 -I -c "import sys, zipfile; zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])" "$tmp/xacro.whl" "$tmp/whl"
        for f in cli.py color.py __init__.py substitution_args.py xmlutils.py; do
            cmp -s "$tmp/whl/xacro/$f" "$x/xacro/$f" || { echo "错误：xacro/$f 与 wheel 中的不同" >&2; status=1; }
        done
    else
        echo "错误：xacro wheel 的 sha256 与锁定的不一致" >&2
        status=1
    fi
    curl -fsSL --retry 3 -o "$tmp/miniz.zip" "$MINIZ_URL"
    if echo "$MINIZ_ZIP_SHA256  $tmp/miniz.zip" | sha256sum -c --quiet - 2>/dev/null; then
        echo "官方下载一致：miniz 发布包"
        python3 -I -c "import sys, zipfile; zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])" "$tmp/miniz.zip" "$tmp/miniz"
        for f in miniz.c miniz.h LICENSE; do
            cmp -s "$tmp/miniz/$f" "$m/$f" || { echo "错误：miniz/$f 与发布包中的不同" >&2; status=1; }
        done
    else
        echo "错误：miniz 发布包的 sha256 与锁定的不一致" >&2
        status=1
    fi
fi
exit $status
