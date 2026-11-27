#!/usr/bin/env python3
"""生成 M5-02b 环境导入的测试夹具：assets/tests/environment/env_fixture.gltf + env_fixture.bin + manifest.json。

用法：python3 scripts/assets/make_environment_fixture.py <输出目录>
只用标准库；输出是确定的（同一脚本得到逐字节相同的文件）。坐标为 glTF 的（Y 向上），Terra 读入时为 (x, −z, y)。

顶层节点（每个导入后成为一个元素）：
  Plinth      立方体基座：glTF 平移 (2, 0, −1) → Terra (2, 1, 0)；网格 x、z ∈ [−0.5, 0.5]，y ∈ [0, 0.5]
  Rack        机柜：绕 Y 轴（Terra 的 Z 轴）转 90°、统一缩放 1.5；子节点 Rack_body（网格 x、z ∈ [−0.2, 0.2]，y ∈ [0, 1]）与
              Rack_hot（平移 (0, 0.8, 0.3)，边长 0.1 的立方体；清单中 75 °C 的部件，也是检查点）
  Stairs      楼梯：平移 (−2, 0, 0)；一个网格中 3 个互不相连的踏步（深 0.298、间隔 0.3、每级高 0.15、宽 1）与一根斜扶手（26.57°）
  Ramp_Main   坡道：类别 outdoor/ramp（按凸包导入）
  Crate       可推动的箱子：清单 dynamic、3 kg；放在基座正上方 1 m 高处（Terra (2, 1, 1)）
  Lamp        没有网格的空节点（导入为只显示）
  Fence       30 根互不相连的立柱（超过每个节点 24 个盒体的上限，导入时合并）
  Doorway     П 形门框：两根门柱与门楣在角上相接（一个连通的网格），门洞宽 1.8 m、高 2.2 m；平移 (0, 0, −4) → Terra (0, 4, 0)。
              导入后门洞中间应是空的（2026-11-26：办公楼的门框曾被一个盒体整个堵上）
"""

import json
import math
import os
import struct
import sys


def cube(lo, hi):
    """轴对齐的立方体：8 个顶点、12 个三角形（外法线方向的绕序）"""
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return v, f


def merge(*parts):
    verts, faces = [], []
    for v, f in parts:
        base = len(verts)
        verts += v
        faces += [(a + base, b + base, c + base) for a, b, c in f]
    return verts, faces


def rotated_bar(center, length, thick, angle):
    """沿 x 轴、绕 z 轴转 angle 的细长方条"""
    v, f = cube((-length / 2, -thick / 2, -thick / 2), (length / 2, thick / 2, thick / 2))
    c, s = math.cos(angle), math.sin(angle)
    out = [(center[0] + c * x - s * y, center[1] + s * x + c * y, center[2] + z) for x, y, z in v]
    return out, f


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "assets/tests/environment"
    os.makedirs(out, exist_ok=True)
    meshes = {
        "Plinth": cube((-0.5, 0, -0.5), (0.5, 0.5, 0.5)),
        "Rack_body": cube((-0.2, 0, -0.2), (0.2, 1.0, 0.2)),
        "Rack_hot": cube((-0.05, -0.05, -0.05), (0.05, 0.05, 0.05)),
        # 踏步之间留 2 mm 的缝（相邻踏步不共用顶点：位置相同的顶点会被视为同一部分）
        "Stairs": merge(*[cube((k * 0.3, 0, -0.5), ((k + 1) * 0.3 - 0.002, (k + 1) * 0.15, 0.5)) for k in range(3)],
                        rotated_bar((0.45, 0.225 + 0.9, 0.45), 1.0, 0.04, math.atan2(0.15, 0.3))),
        "Ramp_Main": ([(0, 0, -0.6), (2, 0, -0.6), (2, 0.3, -0.6), (0, 0, 0.6), (2, 0, 0.6), (2, 0.3, 0.6)],
                      [(0, 1, 2), (3, 5, 4), (0, 3, 4), (0, 4, 1), (1, 4, 5), (1, 5, 2), (0, 2, 5), (0, 5, 3)]),
        "Crate": cube((-0.1, 0, -0.1), (0.1, 0.2, 0.1)),
        "Fence": merge(*[cube((k * 0.4 - 0.025, 0, -0.025), (k * 0.4 + 0.025, 1.2, 0.025)) for k in range(30)]),
        "Doorway": merge(cube((-1.0, 0, -0.1), (-0.9, 2.2, 0.1)), cube((0.9, 0, -0.1), (1.0, 2.2, 0.1)), cube((-1.0, 2.2, -0.1), (1.0, 2.4, 0.1))),
    }
    blob = bytearray()
    views, accessors, gl_meshes, mesh_index = [], [], [], {}

    def view(data, target):
        while len(blob) % 4:
            blob.append(0)
        views.append({"buffer": 0, "byteOffset": len(blob), "byteLength": len(data), "target": target})
        blob.extend(data)
        return len(views) - 1

    for name, (verts, faces) in meshes.items():
        pdata = b"".join(struct.pack("<3f", *p) for p in verts)
        lo = [min(p[i] for p in verts) for i in range(3)]
        hi = [max(p[i] for p in verts) for i in range(3)]
        lo = [struct.unpack("<f", struct.pack("<f", x))[0] for x in lo]
        hi = [struct.unpack("<f", struct.pack("<f", x))[0] for x in hi]
        accessors.append({"bufferView": view(pdata, 34962), "componentType": 5126, "count": len(verts), "type": "VEC3", "min": lo, "max": hi})
        pa = len(accessors) - 1
        idata = b"".join(struct.pack("<3H", *t) for t in faces)
        accessors.append({"bufferView": view(idata, 34963), "componentType": 5123, "count": len(faces) * 3, "type": "SCALAR"})
        mesh_index[name] = len(gl_meshes)
        gl_meshes.append({"name": name, "primitives": [{"attributes": {"POSITION": pa}, "indices": len(accessors) - 1, "mode": 4}]})
    while len(blob) % 4:
        blob.append(0)
    s45 = math.sin(math.pi / 4)
    nodes = [
        {"name": "Plinth", "mesh": mesh_index["Plinth"], "translation": [2, 0, -1]},
        {"name": "Rack", "rotation": [0, s45, 0, s45], "scale": [1.5, 1.5, 1.5], "children": [2, 3]},
        {"name": "Rack_body", "mesh": mesh_index["Rack_body"]},
        {"name": "Rack_hot", "mesh": mesh_index["Rack_hot"], "translation": [0, 0.8, 0.3]},
        {"name": "Stairs", "mesh": mesh_index["Stairs"], "translation": [-2, 0, 0]},
        {"name": "Ramp_Main", "mesh": mesh_index["Ramp_Main"], "translation": [0, 0, 3]},
        {"name": "Crate", "mesh": mesh_index["Crate"], "translation": [2, 1, -1]},
        {"name": "Lamp", "translation": [0, 3, 0]},
        {"name": "Fence", "mesh": mesh_index["Fence"], "translation": [-6, 0, 4]},
        {"name": "Doorway", "mesh": mesh_index["Doorway"], "translation": [0, 0, -4]},
    ]
    gltf = {
        "asset": {"version": "2.0", "generator": "EAI Terra scripts/assets/make_environment_fixture.py"},
        "scene": 0,
        "scenes": [{"nodes": [0, 1, 4, 5, 6, 7, 8, 9]}],
        "nodes": nodes,
        "meshes": gl_meshes,
        "accessors": accessors,
        "bufferViews": views,
        "buffers": [{"uri": "env_fixture.bin", "byteLength": len(blob)}],
    }
    with open(os.path.join(out, "env_fixture.bin"), "wb") as f:
        f.write(blob)
    with open(os.path.join(out, "env_fixture.gltf"), "w", encoding="utf-8") as f:
        json.dump(gltf, f, ensure_ascii=False, indent=1)
        f.write("\n")
    lic = {"source": "EAI Terra 测试夹具", "license": "CC0-1.0", "author": "EAI Terra"}
    manifest = {
        "project": "env_fixture",
        "description": "M5-02b 环境导入的测试夹具",
        "elements": [
            {"id": "Plinth", "name": "基座", "category": "test/plinth", "movable": "fixed", "license": lic},
            {"id": "Rack", "name": "机柜", "category": "test/rack", "movable": "layout", "license": lic,
             "thermal": {"default": 40.0, "parts": {"Rack_hot": 75.0}},
             "inspection": [{"name": "仪表", "node": "Rack_hot", "view": {"distance": 1.5, "heightDeg": 10}}]},
            {"id": "Stairs", "name": "楼梯", "category": "building/staircase", "movable": "fixed", "license": lic},
            {"id": "Ramp_Main", "name": "坡道", "category": "outdoor/ramp", "movable": "fixed", "license": lic},
            {"id": "Crate", "name": "箱子", "category": "test/crate", "movable": "dynamic", "mass": 3.0, "license": lic},
            {"id": "Fence", "name": "围栏", "category": "test/fence", "movable": "fixed", "license": lic},
            {"id": "Doorway", "name": "门框", "category": "building/door_frame", "movable": "fixed", "license": lic},
        ],
    }
    with open(os.path.join(out, "manifest.json"), "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=1)
        f.write("\n")
    print("已生成", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
