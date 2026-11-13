#!/usr/bin/env bash
# 从源码构建 MuJoCo 并安装到 third_party/mujoco/install/。
#
# 只构建核心库（libmujoco），关闭 simulate、示例、studio、测试和 USD，不需要 GLFW 或图形开发包。
# MuJoCo 首次配置时会通过 FetchContent 联网下载其依赖，下载结果保存在 third_party/mujoco/deps/；
# 之后重新构建可加 --offline，完全离线。
#
# 编译器默认为 gcc-12/g++-12：这是上游 CI 在 Ubuntu 22.04 上测试的最老版本，而 MuJoCo 自身启用了
# -Werror 和 C++20。Terra 仍用系统 GCC 11 + C++17，两者只通过 MuJoCo 的 C API 交互。
# 安装：sudo apt install gcc-12 g++-12
#
# 用法：scripts/mujoco/build.sh [--jobs N] [--cc CC] [--cxx CXX] [--offline] [--clean]
set -euo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
base="$root/third_party/mujoco"
# shellcheck source=/dev/null
. "$base/mujoco.lock"

jobs=$(nproc)
cc=gcc-12
cxx=g++-12
offline=OFF
clean=0
while [ $# -gt 0 ]; do
    case "$1" in
        --jobs) jobs=$2; shift 2 ;;
        --cc) cc=$2; shift 2 ;;
        --cxx) cxx=$2; shift 2 ;;
        --offline) offline=ON; shift ;;
        --clean) clean=1; shift ;;
        -h|--help) sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "未知参数：$1" >&2; exit 2 ;;
    esac
done

src="$base/src/mujoco-$MUJOCO_VERSION"
build="$base/build"
prefix="$base/install"

if [ ! -f "$src/CMakeLists.txt" ]; then
    echo "错误：找不到源码 $src，请先运行 scripts/mujoco/fetch.sh" >&2
    exit 1
fi
for tool in "$cc" "$cxx" cmake ninja; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        echo "错误：找不到 $tool（gcc-12/g++-12 可用 sudo apt install gcc-12 g++-12 安装）" >&2
        exit 1
    fi
done
if [ "$clean" = 1 ]; then
    rm -rf "$build" "$prefix"
fi

cmake -S "$src" -B "$build" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_COMPILER="$cc" \
    -DCMAKE_CXX_COMPILER="$cxx" \
    -DCMAKE_INSTALL_PREFIX="$prefix" \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_TESTING=OFF \
    -DMUJOCO_BUILD_EXAMPLES=OFF \
    -DMUJOCO_BUILD_SIMULATE=OFF \
    -DMUJOCO_BUILD_STUDIO=OFF \
    -DMUJOCO_BUILD_TESTS=OFF \
    -DMUJOCO_TEST_PYTHON_UTIL=OFF \
    -DMUJOCO_WITH_USD=OFF \
    -DFETCHCONTENT_BASE_DIR="$base/deps" \
    -DFETCHCONTENT_UPDATES_DISCONNECTED=ON \
    -DFETCHCONTENT_FULLY_DISCONNECTED="$offline"

cmake --build "$build" --parallel "$jobs"
cmake --install "$build"

echo
echo "MuJoCo $MUJOCO_VERSION 已安装到：$prefix"
echo "Terra 默认从该位置查找；也可通过 -DMUJOCO_ROOT=<路径> 指定。"
