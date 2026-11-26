#!/usr/bin/env python3
# 列出 ONNX 文件的输入输出、算子、参数量与元数据（M4-10a，设计稿第 5 节）。不依赖任何包：直接读 protobuf 的线格式。
# 用法：python3 -I scripts/policy/onnx_ops.py <模型.onnx> [--json]
import collections
import json
import sys


def varint(b, i):
    r = 0
    s = 0
    while True:
        c = b[i]
        i += 1
        r |= (c & 0x7F) << s
        s += 7
        if c < 0x80:
            return r, i


def fields(b):
    i = 0
    out = []
    while i < len(b):
        k, i = varint(b, i)
        f, t = k >> 3, k & 7
        if t == 0:
            v, i = varint(b, i)
        elif t == 1:
            v = b[i:i + 8]
            i += 8
        elif t == 5:
            v = b[i:i + 4]
            i += 4
        elif t == 2:
            n, i = varint(b, i)
            v = b[i:i + n]
            i += n
        else:
            raise ValueError("不支持的 wire type %d" % t)
        out.append((f, t, v))
    return out


def text(v):
    return v.decode("utf-8", "replace")


def packed_ints(t, v):
    if t == 0:
        return [v]
    out, i = [], 0
    while i < len(v):
        x, i = varint(v, i)
        out.append(x)
    return out


def value_info(data):
    name, dims = "", []
    for f, _, v in fields(data):
        if f == 1:
            name = text(v)
        if f != 2:
            continue
        for f2, _, v2 in fields(v):
            if f2 != 1:
                continue
            for f3, _, v3 in fields(v2):
                if f3 != 2:
                    continue
                for f4, _, v4 in fields(v3):
                    if f4 != 1:
                        continue
                    dim = None
                    for f5, _, v5 in fields(v4):
                        if f5 == 1:
                            dim = v5
                        if f5 == 2:
                            dim = text(v5)
                    dims.append(dim)
    return {"name": name, "dims": dims}


def describe(path):
    data = open(path, "rb").read()
    info = {"ir_version": None, "producer": "", "opset": None, "metadata": {}, "inputs": [], "outputs": [],
            "operators": {}, "initializers": 0, "parameters": 0}
    graph = b""
    for f, _, v in fields(data):
        if f == 1:
            info["ir_version"] = v
        elif f == 2:
            info["producer"] = text(v)
        elif f == 7:
            graph = v
        elif f == 8:
            kv = {ff: vv for ff, _, vv in fields(v)}
            if text(kv.get(1, b"")) in ("", "ai.onnx"):
                info["opset"] = kv.get(2)
        elif f == 14:
            kv = {ff: vv for ff, _, vv in fields(v)}
            info["metadata"][text(kv.get(1, b""))] = text(kv.get(2, b""))
    ops = collections.Counter()
    initializers = set()
    inputs = []
    for f, t, v in fields(graph):
        if f == 1:
            for ff, _, vv in fields(v):
                if ff == 4:
                    ops[text(vv)] += 1
        elif f == 5:
            dims, name = [], ""
            for ff, tt, vv in fields(v):
                if ff == 1:
                    dims += packed_ints(tt, vv)
                if ff == 8:
                    name = text(vv)
            n = 1
            for d in dims:
                n *= d
            info["parameters"] += n
            info["initializers"] += 1
            initializers.add(name)
        elif f == 11:
            inputs.append(value_info(v))
        elif f == 12:
            info["outputs"].append(value_info(v))
    info["inputs"] = [i for i in inputs if i["name"] not in initializers]
    info["operators"] = dict(sorted(ops.items()))
    return info


def main():
    if len(sys.argv) < 2:
        print(__doc__ or "用法：onnx_ops.py <模型.onnx> [--json]", file=sys.stderr)
        return 2
    info = describe(sys.argv[1])
    if "--json" in sys.argv[2:]:
        print(json.dumps(info, ensure_ascii=False, indent=2))
        return 0
    print("IR %s，opset %s，生成者 %s" % (info["ir_version"], info["opset"], info["producer"]))
    for i in info["inputs"]:
        print("输入  %s %s" % (i["name"], i["dims"]))
    for o in info["outputs"]:
        print("输出  %s %s" % (o["name"], o["dims"]))
    print("算子  " + "、".join("%s × %d" % kv for kv in info["operators"].items()))
    print("参数  %d（%d 个初始化张量）" % (info["parameters"], info["initializers"]))
    for k, v in info["metadata"].items():
        print("元数据 %s = %s" % (k, v if len(v) <= 200 else v[:200] + "…"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
