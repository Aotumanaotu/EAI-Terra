#!/usr/bin/env python3
"""把简单的 COLLADA（.dae）网格转换为 glTF 二进制（.glb），供 Terra 的 glTF 绑定显示（M5-11：传感器外观模型）。

用法：dae_to_glb.py <输入.dae> <输出.glb> [--name 名称]

支持的范围（够用即可，不是通用转换器）：
  - library_geometries 中的 <mesh>：<triangles>（输入 VERTEX / NORMAL，可选 offset）；没有 NORMAL 时由 Terra 生成；
  - 材质：profile_COMMON 的 phong / blinn / lambert 漫反射颜色 → baseColorFactor；光泽度 → 粗糙度 √(2 / (n + 2))，金属度 0；
  - library_visual_scenes 中节点的 <matrix>（没有时为单位矩阵）；
  - <unit meter="…"> 换算为米；<up_axis> 为 Z_UP 时转为 glTF 的 Y 向上（(x, y, z) → (x, z, −y)），Terra 读入时再转回 Z 向上。
输出是确定的：同一输入得到逐字节相同的 .glb（不写时间戳）。只用标准库。
"""

import argparse
import json
import math
import re
import struct
import sys
import xml.etree.ElementTree as ET


def strip_ns(root):
    for e in root.iter():
        if "}" in e.tag:
            e.tag = e.tag.split("}", 1)[1]
    return root


def floats(text):
    return [float(x) for x in (text or "").split()]


def ints(text):
    return [int(x) for x in (text or "").split()]


def material_colors(root):
    """材质 id → (rgba, roughness)。"""
    effects = {}
    for eff in root.iter("effect"):
        rgba, rough = [0.8, 0.8, 0.8, 1.0], 0.6
        for shading in ("phong", "blinn", "lambert"):
            node = eff.find(".//" + shading)
            if node is None:
                continue
            col = node.find("./diffuse/color")
            if col is not None:
                v = floats(col.text)
                rgba = (v + [1.0])[:4] if len(v) == 3 else v[:4]
            sh = node.find("./shininess/float")
            if sh is not None:
                n = max(0.0, floats(sh.text)[0])
                rough = math.sqrt(2.0 / (n + 2.0))
            break
        effects[eff.get("id")] = ([min(1.0, max(0.0, c)) for c in rgba], min(1.0, max(0.04, rough)))
    materials = {}
    for mat in root.iter("material"):
        inst = mat.find("./instance_effect")
        if inst is not None and inst.get("url", "").lstrip("#") in effects:
            materials[mat.get("id")] = effects[inst.get("url").lstrip("#")]
    return materials


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("output")
    ap.add_argument("--name", default="")
    a = ap.parse_args()

    root = strip_ns(ET.parse(a.input).getroot())
    unit = root.find("./asset/unit")
    scale = float(unit.get("meter", "1")) if unit is not None else 1.0
    up = (root.findtext("./asset/up_axis") or "Y_UP").strip()
    if up not in ("Y_UP", "Z_UP"):
        sys.exit(f"不支持的 up_axis：{up}")

    def to_gltf(p):
        x, y, z = p
        return (x, z, -y) if up == "Z_UP" else (x, y, z)

    materials = material_colors(root)
    # 节点：几何体 id → [(4×4 矩阵, {材质符号 → 材质 id})]
    instances = {}
    for node in root.iter("node"):
        mtx = node.find("./matrix")
        m = floats(mtx.text) if mtx is not None else [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
        for ig in node.findall("./instance_geometry"):
            binding = {im.get("symbol"): im.get("target", "").lstrip("#") for im in ig.iter("instance_material")}
            instances.setdefault(ig.get("url", "").lstrip("#"), []).append((m, binding))

    gltf_materials, material_index = [], {}
    meshes_out = []  # [(材质下标, positions[], normals[]|None, indices[])]
    for geom in root.iter("geometry"):
        mesh = geom.find("./mesh")
        if mesh is None:
            continue
        sources = {}
        for src in mesh.findall("./source"):
            arr = src.find("./float_array")
            acc = src.find("./technique_common/accessor")
            stride = int(acc.get("stride", "3")) if acc is not None else 3
            data = floats(arr.text) if arr is not None else []
            sources[src.get("id")] = [data[i:i + stride] for i in range(0, len(data), stride)]
        vert = mesh.find("./vertices")
        vert_inputs = {}
        if vert is not None:
            for inp in vert.findall("./input"):
                vert_inputs[inp.get("semantic")] = inp.get("source", "").lstrip("#")
        for inst_m, binding in instances.get(geom.get("id"), [([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1], {})]):
            for tris in mesh.findall("./triangles"):
                inputs = tris.findall("./input")
                stride = max(int(i.get("offset", "0")) for i in inputs) + 1
                p = ints(tris.findtext("./p"))
                pos_src = nrm_src = None
                pos_off = nrm_off = 0
                for inp in inputs:
                    sem, off, src = inp.get("semantic"), int(inp.get("offset", "0")), inp.get("source", "").lstrip("#")
                    if sem == "VERTEX":
                        pos_src, pos_off = vert_inputs.get("POSITION"), off
                        if "NORMAL" in vert_inputs:
                            nrm_src, nrm_off = vert_inputs["NORMAL"], off
                    elif sem == "NORMAL":
                        nrm_src, nrm_off = src, off
                if pos_src not in sources:
                    sys.exit(f"三角形缺少 POSITION：{geom.get('id')}")
                symbol = tris.get("material", "")
                mat_id = binding.get(symbol, symbol)
                rgba, rough = materials.get(mat_id, ([0.8, 0.8, 0.8, 1.0], 0.6))
                if mat_id not in material_index:
                    material_index[mat_id] = len(gltf_materials)
                    gltf_materials.append({"name": mat_id or "default", "pbrMetallicRoughness": {
                        "baseColorFactor": [round(c, 6) for c in rgba], "metallicFactor": 0.0, "roughnessFactor": round(rough, 6)}})
                # 矩阵按行优先（COLLADA），作用于列向量
                M = [inst_m[0:4], inst_m[4:8], inst_m[8:12]]

                def xform(v, w):
                    return [sum(M[r][c] * (v[c] if c < 3 else w) for c in range(4)) for r in range(3)]

                remap, positions, normals, indices = {}, [], [], []
                for k in range(0, len(p) - stride + 1, stride):
                    pi = p[k + pos_off]
                    ni = p[k + nrm_off] if nrm_src in sources else -1
                    key = (pi, ni)
                    if key not in remap:
                        remap[key] = len(positions)
                        pv = xform(sources[pos_src][pi], 1.0)
                        positions.append(to_gltf([c * scale for c in pv]))
                        if ni >= 0:
                            nv = xform(sources[nrm_src][ni], 0.0)
                            length = math.sqrt(sum(c * c for c in nv)) or 1.0
                            normals.append(to_gltf([c / length for c in nv]))
                    indices.append(remap[key])
                if indices:
                    meshes_out.append((material_index[mat_id], positions, normals if nrm_src in sources else None, indices))
    if not meshes_out:
        sys.exit("没有找到三角形网格")

    blob = bytearray()
    views, accessors, primitives = [], [], []

    def add_view(data, target):
        while len(blob) % 4:
            blob.append(0)
        views.append({"buffer": 0, "byteOffset": len(blob), "byteLength": len(data), "target": target})
        blob.extend(data)
        return len(views) - 1

    for mat, positions, normals, indices in meshes_out:
        attrs = {}
        pdata = b"".join(struct.pack("<3f", *v) for v in positions)
        lo = [min(v[i] for v in positions) for i in range(3)]
        hi = [max(v[i] for v in positions) for i in range(3)]
        # min / max 取 float32 舍入后的值（与缓冲区一致）
        lo = [struct.unpack("<f", struct.pack("<f", x))[0] for x in lo]
        hi = [struct.unpack("<f", struct.pack("<f", x))[0] for x in hi]
        accessors.append({"bufferView": add_view(pdata, 34962), "componentType": 5126, "count": len(positions), "type": "VEC3", "min": lo, "max": hi})
        attrs["POSITION"] = len(accessors) - 1
        if normals:
            ndata = b"".join(struct.pack("<3f", *v) for v in normals)
            accessors.append({"bufferView": add_view(ndata, 34962), "componentType": 5126, "count": len(normals), "type": "VEC3"})
            attrs["NORMAL"] = len(accessors) - 1
        big = len(positions) > 65535
        idata = struct.pack("<%d%s" % (len(indices), "I" if big else "H"), *indices)
        accessors.append({"bufferView": add_view(idata, 34963), "componentType": 5125 if big else 5123, "count": len(indices), "type": "SCALAR"})
        primitives.append({"attributes": attrs, "indices": len(accessors) - 1, "material": mat, "mode": 4})
    while len(blob) % 4:
        blob.append(0)

    name = a.name or re.sub(r"\.dae$", "", a.input.split("/")[-1])
    doc = {
        "asset": {"version": "2.0", "generator": "EAI Terra scripts/assets/dae_to_glb.py"},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [{"name": name, "mesh": 0}],
        "meshes": [{"name": name, "primitives": primitives}],
        "materials": gltf_materials,
        "accessors": accessors,
        "bufferViews": views,
        "buffers": [{"byteLength": len(blob)}],
    }
    js = json.dumps(doc, separators=(",", ":"), ensure_ascii=False).encode("utf-8")
    js += b" " * ((4 - len(js) % 4) % 4)
    total = 12 + 8 + len(js) + 8 + len(blob)
    with open(a.output, "wb") as f:
        f.write(struct.pack("<4sII", b"glTF", 2, total))
        f.write(struct.pack("<I4s", len(js), b"JSON"))
        f.write(js)
        f.write(struct.pack("<I4s", len(blob), b"BIN\x00"))
        f.write(blob)
    tris = sum(len(m[3]) for m in meshes_out) // 3
    print(f"已转换：{a.output}（{len(meshes_out)} 个图元，{tris} 个三角形，{len(gltf_materials)} 种材质，{total} 字节）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
