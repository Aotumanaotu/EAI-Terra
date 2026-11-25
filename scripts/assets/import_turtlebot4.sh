#!/usr/bin/env bash
# 导入 TurtleBot 4 标准版（M4-08b）：按导入配方 assets/robots/turtlebot4.terra-import.json，用 terra_import 生成
# assets/robots/turtlebot4/（MJCF、转换后的网格、场景 turtlebot4.terra-scene.json、导入报告；不提交）。
# 先运行 scripts/assets/fetch_robots.sh 准备描述文件（assets/robots/turtlebot4_description、irobot_create_description）。
# 需要 assimp 命令行工具（sudo apt install assimp-utils）与 python3。
# 用法：bash scripts/assets/import_turtlebot4.sh [terra_import 程序]（默认 build/render-release/apps/import/terra_import）
set -euo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
cd "$root"
importer=${1:-build/render-release/apps/import/terra_import}
if [ ! -x "$importer" ]; then
    echo "错误：找不到 terra_import：$importer（先构建，或在参数中给出路径）" >&2
    exit 2
fi
for pkg in turtlebot4_description irobot_create_description; do
    if [ ! -f "assets/robots/$pkg/package.xml" ]; then
        echo "错误：缺少 assets/robots/$pkg（先运行 bash scripts/assets/fetch_robots.sh）" >&2
        exit 2
    fi
done
"$importer" --recipe assets/robots/turtlebot4.terra-import.json --output assets/robots --overwrite
echo "场景：assets/robots/turtlebot4/turtlebot4.terra-scene.json"
