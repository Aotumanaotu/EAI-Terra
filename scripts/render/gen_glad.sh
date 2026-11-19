#!/usr/bin/env bash
# 用锁定版本的 glad 生成器重新生成 OpenGL 加载代码，并与仓库中的 third_party/glad/ 逐字节比较（M1-02）。
# 默认只比较；加 --update 时用生成结果覆盖 third_party/glad/（修改 GLAD_API/GLAD_EXTENSIONS 后使用）。
# 需要先运行 scripts/render/fetch.sh（获取 glad 源码），以及 python3 与 jinja2（apt 包 python3-jinja2）。
# 用法：scripts/render/gen_glad.sh [--update]
set -euo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
base="$root/third_party/render"
# shellcheck source=/dev/null
. "$base/render.lock"

update=0
[ "${1:-}" = "--update" ] && update=1

generator="$base/src/glad-$GLAD_VERSION"
if [ ! -d "$generator/glad" ]; then
    echo "错误：找不到 glad 生成器 $generator，请先运行 scripts/render/fetch.sh" >&2
    exit 1
fi
if ! python3 -c "import jinja2" 2>/dev/null; then
    echo "错误：缺少 jinja2（sudo apt install python3-jinja2）" >&2
    exit 1
fi

out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT
# -I：不从当前目录或环境变量加载模块；生成器路径显式加入 sys.path。
# --reproducible：使用生成器自带的 Khronos 规范，不联网、不写时间戳。
python3 -I -c '
import runpy, sys
sys.path.insert(0, sys.argv[1])
sys.argv = ["glad", "--api", sys.argv[2], "--extensions", sys.argv[3], "--out-path", sys.argv[4],
            "--reproducible", "--quiet", "c"]
runpy.run_module("glad", run_name="__main__")
' "$generator" "$GLAD_API" "$GLAD_EXTENSIONS" "$out"

target="$root/third_party/glad"
if [ "$update" = 1 ]; then
    rm -rf "$target/include" "$target/src"
    cp -r "$out/include" "$out/src" "$target/"
    echo "已更新 $target"
elif diff -r "$out/include" "$target/include" >/dev/null && diff -r "$out/src" "$target/src" >/dev/null; then
    echo "glad 生成结果与仓库一致（glad $GLAD_VERSION，$GLAD_API，扩展：${GLAD_EXTENSIONS:-无}）"
else
    echo "错误：glad 生成结果与仓库中的 third_party/glad/ 不一致" >&2
    diff -r "$out" "$target" --exclude=README.md | head -20 >&2
    exit 1
fi
