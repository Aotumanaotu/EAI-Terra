#!/usr/bin/env bash
# 构建 GLFW 静态库并安装到 third_party/render/install/（M1-02，D-017、D-019）。GLM 仅头文件，无需构建。
#
# 默认同时构建 X11 与 Wayland 后端（运行时按会话选择）。Wayland 后端需要 wayland-scanner
# （Ubuntu 包 libwayland-bin，通常随 libwayland-dev 安装）；缺少时可加 --no-wayland 只构建 X11。
# GLFW 在运行时以 dlopen 加载 libGL 与 X11/Wayland 库，Terra 不在链接时依赖它们。
#
# 用法：scripts/render/build.sh [--jobs N] [--no-wayland] [--clean]
set -euo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
base="$root/third_party/render"
# shellcheck source=/dev/null
. "$base/render.lock"

jobs=$(nproc)
wayland=ON
clean=0
while [ $# -gt 0 ]; do
    case "$1" in
        --jobs) jobs=$2; shift 2 ;;
        --no-wayland) wayland=OFF; shift ;;
        --clean) clean=1; shift ;;
        -h|--help) sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

src="$base/src/glfw-$GLFW_VERSION"
build="$base/build/glfw"
prefix="$base/install"

if [ ! -f "$src/CMakeLists.txt" ]; then
    echo "错误：找不到源码 $src，请先运行 scripts/render/fetch.sh" >&2
    exit 1
fi
for tool in gcc cmake ninja; do
    command -v "$tool" >/dev/null 2>&1 || { echo "错误：找不到 $tool" >&2; exit 1; }
done
if [ "$wayland" = ON ] && ! command -v wayland-scanner >/dev/null 2>&1; then
    echo "错误：找不到 wayland-scanner（sudo apt install libwayland-bin），或加 --no-wayland 只构建 X11 后端" >&2
    exit 1
fi
if [ "$clean" = 1 ]; then
    rm -rf "$build" "$prefix"
fi

cmake -S "$src" -B "$build" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_COMPILER=gcc \
    -DCMAKE_INSTALL_PREFIX="$prefix" \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DBUILD_SHARED_LIBS=OFF \
    -DGLFW_BUILD_X11=ON \
    -DGLFW_BUILD_WAYLAND="$wayland" \
    -DGLFW_BUILD_EXAMPLES=OFF \
    -DGLFW_BUILD_TESTS=OFF \
    -DGLFW_BUILD_DOCS=OFF \
    -DGLFW_INSTALL=ON

cmake --build "$build" --parallel "$jobs"
cmake --install "$build"

echo
echo "GLFW $GLFW_VERSION 已安装到：$prefix（X11=ON，Wayland=$wayland）"
