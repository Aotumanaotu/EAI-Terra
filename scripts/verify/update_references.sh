#!/usr/bin/env bash
# 在基线机器上生成或更新参考图（M1-10，D-022：只以 Xvfb + llvmpipe 为准）。
# 使用 build/render-release（不存在时先配置），只构建 test_render_reference；对各场景先与现有参考图比较并打印差异，
# 再写入新的参考图到 tests/render/reference/。带资产的场景在素材缺失时跳过。
# 生成后请人工查看图片，确认无误再提交。验证脚本只在参考图缺失时调用本脚本（只更新缺失的场景）。
# 用法：bash scripts/verify/update_references.sh [场景 ...]（默认全部：procedural studio outdoor carts）
set -uo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
cd "$root"
b=build/render-release
refs=tests/render/reference
out=$b/tests/render_reference
if [ ! -f "$b/build.ninja" ]; then
    cmake --preset render-release >/dev/null || { echo "错误：配置 render-release 失败" >&2; exit 1; }
fi
cmake --build "$b" --target test_render_reference -j "$(nproc)" >/dev/null || { echo "错误：构建 test_render_reference 失败" >&2; exit 1; }
mkdir -p "$refs" "$out"
run() {
    env -u DISPLAY -u WAYLAND_DISPLAY __GLX_VENDOR_LIBRARY_NAME=mesa LIBGL_ALWAYS_SOFTWARE=1 \
        xvfb-run -a -s "-screen 0 1280x720x24" "$b/tests/test_render_reference" "$refs" "$out" "$@"
}
status=0
scenes=("$@")
[ ${#scenes[@]} -gt 0 ] || scenes=(procedural studio outdoor carts)
for scene in "${scenes[@]}"; do
    if [ -f "$refs/$scene.png" ]; then
        echo "---- $scene：与现有参考图比较"
        run --scene "$scene" --expect-renderer llvmpipe 2>&1 | grep -E "INFO 与参考图|FAILED|跳过" || true
    fi
    echo "---- $scene：写入参考图"
    run --scene "$scene" --expect-renderer llvmpipe --update >"$out/update_$scene.txt" 2>&1
    rc=$?
    if [ $rc = 77 ]; then
        echo "  跳过（需要 HDRI 与 glTF 资产）"
    elif [ $rc = 0 ]; then
        grep -E "INFO" "$out/update_$scene.txt" | sed 's/^/  /'
    else
        echo "  失败（退出码 $rc，见 $out/update_$scene.txt）"
        grep -E "FAILED" "$out/update_$scene.txt" | sed 's/^/  /'
        status=1
    fi
done
echo
for scene in "${scenes[@]}"; do ls -l "$refs/$scene.png" 2>/dev/null; sha256sum "$refs/$scene.png" 2>/dev/null; done
echo "请查看以上参考图（例如用图片查看器打开 $refs/），确认画面正确后提交。"
exit $status
