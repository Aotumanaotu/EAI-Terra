#!/usr/bin/env bash
# 导入用户的两个巡检场景（M5-02b，资产在 blender/，见 blender/README.md）：变电站与办公楼的整站 glb 按顶层节点拆成场景元素，
# 写到 assets/environments/<名字>/（不提交）；再各写一个“Go2 巡检”场景：Go2 放在推荐巡检路线的起点，机身 IMU、
# 正装在机身上方的 Livox Mid-360（含内置 IMU 与外观；用户 2026-11-26：不用倒置）与前方的 FLIR Lepton 3.5 热像仪。资源库启动时扫描 assets/environments/，
# 两个导入的场景与两个 Go2 巡检场景都出现在“场景”分页。
# 用法：scripts/assets/import_environments.sh <terra_import 程序> [--collision boxes|hulls]
set -euo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
import=${1:?用法：import_environments.sh <terra_import 程序>}
shift
extra=("$@")
out="$root/assets/environments"
status=0
done_count=0
for spec in "substation_110kv|blender/substation_110kv/exports/substation_110kv_full.glb" "indoor_office|blender/indoor_office/exports/indoor_office_full.glb"; do
    name=${spec%%|*}
    glb="$root/${spec#*|}"
    if [ ! -f "$glb" ]; then
        echo "跳过 $name：没有 $glb"
        continue
    fi
    echo "导入 $name：$glb"
    if ! "$import" --environment "$glb" --name "$name" --output "$out" --overwrite "${extra[@]}"; then
        echo "错误：$name 导入失败" >&2
        status=1
        continue
    fi
    # Go2 巡检场景：在导入的场景上加 Go2 与传感器（起点取清单的推荐巡检路线第一点）
    python3 -I - "$out/$name/$name.terra-scene.json" "$root/blender/$name/manifest.json" "$root/assets/robots/unitree_go2/go2.xml" <<'PY'
import json, os, sys
scene_path, manifest_path, go2 = sys.argv[1:4]
scene = json.load(open(scene_path, encoding="utf-8"))
manifest = json.load(open(manifest_path, encoding="utf-8")) if os.path.exists(manifest_path) else {}
route = manifest.get("recommended_inspection_route_xyz") or manifest.get("recommended_inspection_route") or [[0, 0]]
start = list(route[0]) + [0.0] * (3 - len(route[0]))
nxt = route[1] if len(route) > 1 else [start[0] + 1, start[1]]
# 起点的高度取脚下表面的顶面（2026-11-26：路线点的 z 是地坪的标高，进站道路、入口步道比它高 2～4 cm，
# 直接放在路线点上 Go2 的脚会陷进路面）：Go2 机身约 0.4 m × 0.3 m，取覆盖这一范围、顶面低于 1 m 的元素包围盒的最高顶面，再抬高 5 mm
support = None
for e in manifest.get("elements", []):
    lo, hi = e.get("world_bbox_min"), e.get("world_bbox_max")
    if not lo or not hi or hi[2] >= 1.0 or hi[2] - lo[2] > 0.6:
        continue
    if lo[0] - 0.25 <= start[0] <= hi[0] + 0.25 and lo[1] - 0.2 <= start[1] <= hi[1] + 0.2:
        support = hi[2] if support is None else max(support, hi[2])
if support is not None:
    start[2] = support + 0.005
import math
yaw = math.degrees(math.atan2(nxt[1] - start[1], nxt[0] - start[0]))
base = os.path.dirname(scene_path)
scene["name"] = scene["name"].replace("环境（M5-02b 导入）：", "Go2 巡检（M5-02b）：") + "；Go2 在推荐巡检路线的起点，带 Mid-360（正装）与 Lepton 3.5 热像仪"
scene["title"] = scene["title"] + "（Go2 巡检）"
scene["models"] = [{"name": "go2", "mjcf": os.path.relpath(go2, base), "position": [start[0], start[1], start[2]],
                    "rotation": {"rpyDeg": [0, 0, round(yaw, 3)]}}]
scene["initialKeyframe"] = "go2/home"
scene["noise"] = {"seed": 1}
scene["sensors"] = [
    {"type": "imu", "site": "go2/imu", "rateHz": 500, "noise": {"gyro": {"noiseDensity": 0.00516}}},
    {"type": "lidar3d", "name": "go2/mid360", "entity": "go2/base", "position": [0.15, 0, 0.12], "preset": "livox_mid360", "imu": True},
    {"type": "camera", "name": "go2/thermal", "entity": "go2/base", "position": [0.3, 0, 0.08], "rotation": {"rpyDeg": [90, 0, -90]},
     "segmentation": True, "thermal": {"preset": "flir_lepton_3_5"}},
]
scene["thermal"] = {"ambient": 22, "sky": -15, "temperatures": {"go2/*": 36}}
c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
scene["view"] = {"camera": {"position": [start[0] - 3.0 * c + 1.2 * s, start[1] - 3.0 * s - 1.2 * c, start[2] + 1.8],
                            "target": [start[0] + 1.5 * c, start[1] + 1.5 * s, start[2] + 0.3]}}
out = scene_path.replace(".terra-scene.json", "_go2.terra-scene.json")
with open(out, "w", encoding="utf-8") as f:
    json.dump(scene, f, ensure_ascii=False, indent=1)
    f.write("\n")
print("已写 Go2 巡检场景：%s（起点 (%.2f, %.2f, %.3f)，脚下表面 %s）" % (out, start[0], start[1], start[2], "%.3f m" % support if support is not None else "未知"))
PY
    done_count=$((done_count + 1))
done
echo "就绪：导入 $done_count 个环境（$out）"
exit $status
