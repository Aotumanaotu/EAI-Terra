#!/usr/bin/env python3
"""把 Headless --sensors 输出的一次 2D 激光雷达扫描画成俯视图 PNG（M4-03b）。只用标准库。

用法：python3 plot_lidar.py <headless 输出.jsonl> <雷达名> <输出.png> [--size 像素] [--range 米]
雷达在图中心，朝上为雷达的 +X（0°），逆时针为正；红点为命中，灰线为没有命中的方向（+inf），蓝点为近于最小量程（−inf）。
"""
import json
import math
import struct
import sys
import zlib


def write_png(path, width, height, pixels):
    raw = b"".join(b"\x00" + bytes(pixels[y * width * 3:(y + 1) * width * 3]) for y in range(height))

    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(raw, 9)))
        f.write(chunk(b"IEND", b""))


def main():
    args = sys.argv[1:]
    if len(args) < 3:
        print(__doc__)
        return 2
    source, name, out = args[0], args[1], args[2]
    size = int(args[args.index("--size") + 1]) if "--size" in args else 480
    view = float(args[args.index("--range") + 1]) if "--range" in args else 4.0
    scan = None
    with open(source, encoding="utf-8") as f:
        for line in f:
            record = json.loads(line)
            if record.get("type") == "lidar" and record.get("name") == name:
                scan = record
                break
    if scan is None:
        print(f"没有找到雷达 {name} 的扫描", file=sys.stderr)
        return 1
    pixels = [255] * (size * size * 3)
    scale = (size / 2 - 10) / view
    cx = cy = size // 2

    def put(x, y, color, radius=1):
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                px, py = x + dx, y + dy
                if 0 <= px < size and 0 <= py < size:
                    i = (py * size + px) * 3
                    pixels[i:i + 3] = color

    # 1 m 网格
    for k in range(-int(view), int(view) + 1):
        for t in range(size):
            for x, y in ((cx + int(k * scale), t), (t, cy + int(k * scale))):
                if 0 <= x < size and 0 <= y < size:
                    i = (y * size + x) * 3
                    pixels[i:i + 3] = [232, 232, 232]
    hits = 0
    for i, r in enumerate(scan["ranges"]):
        a = scan["angle_min"] + i * scan["angle_increment"]
        # 图中朝上为雷达 +X，向左为雷达 +Y
        ux, uy = -math.sin(a), -math.cos(a)
        if r == "inf":
            for s in range(0, int(view * scale), 3):
                put(cx + int(ux * s), cy + int(uy * s), [205, 205, 205], 0)
        elif r == "-inf":
            put(cx + int(ux * 6), cy + int(uy * 6), [40, 90, 220], 1)
        else:
            hits += 1
            put(cx + int(ux * r * scale), cy + int(uy * r * scale), [215, 40, 40], 1)
    put(cx, cy, [0, 0, 0], 3)
    write_png(out, size, size, pixels)
    print(f"{name}：第 {scan['step']} 步，{len(scan['ranges'])} 束，{hits} 束命中 → {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
