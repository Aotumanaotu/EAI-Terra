#!/usr/bin/env python3
"""读取 terra_capture 写出的深度图（.npy，float32，行优先），打印尺寸、中心像素的深度与有限值的范围。只用标准库。

用法：python3 npy_center.py <文件.npy>
"""
import ast
import math
import struct
import sys


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    with open(sys.argv[1], "rb") as f:
        data = f.read()
    if data[:6] != b"\x93NUMPY":
        print("不是 .npy 文件", file=sys.stderr)
        return 1
    length = struct.unpack("<H", data[8:10])[0]
    header = ast.literal_eval(data[10:10 + length].decode("latin1"))
    height, width = header["shape"]
    values = struct.unpack("<%df" % (width * height), data[10 + length:10 + length + 4 * width * height])
    finite = [v for v in values if math.isfinite(v)]
    center = values[(height // 2) * width + width // 2]
    print(f"{width} × {height}，{header['descr']}；中心深度 {center:.3f} m；有限值 {len(finite)} 个"
          + (f"，范围 {min(finite):.3f}～{max(finite):.3f} m" if finite else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
