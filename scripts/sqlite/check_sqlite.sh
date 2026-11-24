#!/usr/bin/env bash
# 复核已提交的 SQLite 合并版源码（M3-07b，D-112）：按 third_party/sqlite/sqlite.lock 校验 sha256。
# 用法：scripts/sqlite/check_sqlite.sh [--online]
#   --online  另外从官方下载 zip，确认 zip 的 SHA-256 与锁定一致，且解压出的文件与仓库中的相同（显式联网步骤）。
set -euo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
base="$root/third_party/sqlite"
# shellcheck source=/dev/null
. "$base/sqlite.lock"

status=0
verify() {  # verify <文件> <sha256>
    if echo "$2  $1" | sha256sum -c --quiet - 2>/dev/null; then
        echo "一致：${1#"$root"/}"
    else
        echo "错误：sha256 不匹配或文件缺失：${1#"$root"/}" >&2
        status=1
    fi
}

verify "$base/src/sqlite3.c" "$SQLITE_C_SHA256"
verify "$base/src/sqlite3.h" "$SQLITE_H_SHA256"
grep -q "#define SQLITE_VERSION        \"$SQLITE_VERSION\"" "$base/src/sqlite3.h" ||
    { echo "错误：sqlite3.h 的版本不是 $SQLITE_VERSION" >&2; status=1; }

if [ "${1:-}" = "--online" ]; then
    tmp=$(mktemp -d)
    trap 'rm -rf "$tmp"' EXIT
    curl -fsSL --retry 3 -o "$tmp/sqlite.zip" "$SQLITE_ZIP_URL"
    if echo "$SQLITE_ZIP_SHA256  $tmp/sqlite.zip" | sha256sum -c --quiet - 2>/dev/null; then
        echo "官方下载一致：$(basename "$SQLITE_ZIP_URL")"
        python3 -I -c "import sys,zipfile; zipfile.ZipFile(sys.argv[1]).extractall(sys.argv[2])" "$tmp/sqlite.zip" "$tmp/x"
        for f in sqlite3.c sqlite3.h; do
            cmp -s "$tmp/x/sqlite-amalgamation-3540100/$f" "$base/src/$f" && echo "与官方发布相同：$f" ||
                { echo "错误：$f 与官方发布不同" >&2; status=1; }
        done
    else
        echo "错误：官方下载的 zip 与锁定的 sha256 不一致" >&2
        status=1
    fi
fi
exit $status
