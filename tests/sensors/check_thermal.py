#!/usr/bin/env python3
"""terra_capture 的热像输出（M5-06，graphics.capture_thermal）。夹具 thermal_bench（三台 FLIR Lepton 3.5）。

用法（由 ctest 在图形环境中调用）：
  check_thermal.py --capture <terra_capture> --scene <thermal_bench 场景> --work <工作目录>
检查：
  - capture.json 写明每台热像仪的预设、增益档、量程、色板、显示区间与 NETD；每帧有 thermal/<步>.npy（float32，(120, 160)）与 .png；
  - 与分割、深度逐像素对齐（bench/ir，无噪声）：没有命中 → 天空 −20 °C；分割为 0 → 墙 15 °C；板 80 °C；
    方块 20 + 40 × t °C（t 为该帧的仿真时间，1 s 后 60 °C）；加热器 40 °C，热点附近更高；
  - 热点：加热器上的最高温度在 115～120 °C，位置与热点中心 (0.9, 0.38, 1.2) 按内参的投影相差不到 2 像素；
  - bench/ir_sky（背向墙）：全是 −20 °C；
  - bench/ir_noisy（同一位姿，有噪声）：与 bench/ir 之差的标准差 ≈ NETD 0.05 K，均值（固定偏差）各帧相同；
  - 两次运行逐字节相同；--no-noise 时 bench/ir_noisy 与 bench/ir 相同。
"""

import argparse
import ast
import hashlib
import json
import math
import os
import struct
import subprocess
import sys

failures = []


def check(ok, what):
    print(("PASS " if ok else "FAILED: ") + what)
    if not ok:
        failures.append(what)
    return ok


def read_npy(path):
    """读 NumPy 1.0 格式（小端 float32 或 uint16，行优先）：返回 (形状, 值列表)"""
    with open(path, "rb") as f:
        data = f.read()
    if data[:6] != b"\x93NUMPY":
        raise ValueError("不是 npy：" + path)
    length = struct.unpack_from("<H", data, 8)[0]
    header = ast.literal_eval(data[10:10 + length].decode("latin1"))
    shape = tuple(header["shape"])
    count = 1
    for s in shape:
        count *= s
    code = {"<f4": "f", "<u2": "H"}[header["descr"]]
    values = struct.unpack_from("<%d%s" % (count, code), data, 10 + length)
    return shape, header["descr"], list(values)


def run(cmd):
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode != 0:
        print(p.stdout[-2000:])
        print(p.stderr[-2000:])
    return p


def frames(out, camera):
    path = os.path.join(out, "cameras", camera.replace("/", "_"), "frames.jsonl")
    if not os.path.exists(path):
        return []
    return [json.loads(line) for line in open(path, encoding="utf-8") if line.strip()]


def digest(out):
    h = hashlib.sha256()
    for root, _, files in sorted(os.walk(os.path.join(out, "cameras"))):
        for name in sorted(files):
            if "/thermal" in root or name == "frames.jsonl":
                with open(os.path.join(root, name), "rb") as f:
                    h.update(name.encode() + f.read())
    return h.hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--capture", required=True)
    ap.add_argument("--scene", required=True)
    ap.add_argument("--work", required=True)
    a = ap.parse_args()
    os.makedirs(a.work, exist_ok=True)
    out_a = os.path.join(a.work, "a")
    out_b = os.path.join(a.work, "b")
    out_n = os.path.join(a.work, "no_noise")
    base = [a.capture, "--scene", a.scene, "--steps", "600", "--force", "--expect-renderer", "llvmpipe"]
    ra = run(base + ["--output", out_a])
    rb = run(base + ["--output", out_b])
    rn = run(base + ["--output", out_n, "--no-noise"])
    if not check(ra.returncode == 0 and rb.returncode == 0 and rn.returncode == 0, "terra_capture 三次运行退出码 0"):
        return 1

    meta = json.load(open(os.path.join(out_a, "capture.json"), encoding="utf-8"))
    cams = {c["name"]: c for c in meta["cameras"]}
    t = cams.get("bench/ir", {}).get("thermal", {})
    check(t.get("preset") == "flir_lepton_3_5" and t.get("gain") == "high" and t.get("range") == [-10, 140] and t.get("palette") == "ironbow" and
          t.get("display") == "auto" and cams["bench/ir"]["width"] == 160 and cams["bench/ir"]["height"] == 120,
          "capture.json：bench/ir 为 Lepton 3.5，高增益 −10～140 °C，ironbow，auto，160 × 120")
    tn = cams.get("bench/ir_noisy", {}).get("thermal", {})
    check(tn.get("palette") == "white-hot" and tn.get("display") == [0, 130] and abs(tn.get("netd_k", 0) - 0.05) < 1e-12,
          "capture.json：bench/ir_noisy 为 white-hot、0～130 °C、NETD 0.05 K")

    ir = frames(out_a, "bench/ir")
    noisy = frames(out_a, "bench/ir_noisy")
    sky = frames(out_a, "bench/ir_sky")
    check(len(ir) >= 8 and len(ir) == len(noisy) == len(sky) and all("thermal" in f and "thermal_color" in f for f in ir + noisy + sky),
          "三台热像仪各 %d 帧（8.7 Hz × 1.2 s），每帧有温度图与伪彩色图" % len(ir))
    cam_dir = lambda name: os.path.join(out_a, "cameras", name.replace("/", "_"))
    labels = json.load(open(os.path.join(cam_dir("bench/ir"), "seg_labels.json"), encoding="utf-8"))
    names = {e["id"]: e["entity"] for e in labels["labels"]}

    # 内参（与 capture.json 相同）：热点中心 (0.9, 0.38, 1.2) 的投影；相机在 (0, 0, 1) 看向 +X，图像 x = 世界 −Y，图像 y = 世界 +Z
    c = cams["bench/ir"]
    hu = c["cx"] + c["fx"] * (-0.38) / 0.9
    hv = c["cy"] - c["fy"] * (0.2) / 0.9
    aligned = True
    bad = ""
    hottest = (-1e9, 0, 0)
    block_ok = True
    for f in ir:
        shape, descr, temp = read_npy(os.path.join(cam_dir("bench/ir"), f["thermal"]))
        _, _, depth = read_npy(os.path.join(cam_dir("bench/ir"), f["depth"]))
        _, _, seg = read_npy(os.path.join(cam_dir("bench/ir"), f["segmentation"]))
        if shape != (120, 160) or descr != "<f4":
            aligned = False
            bad = "形状 %s %s" % (shape, descr)
            break
        for i, v in enumerate(temp):
            name = names.get(seg[i], "")
            if not math.isfinite(depth[i]):
                expect = -20.0
            elif seg[i] == 0:
                expect = 15.0
            elif name == "bench/plate":
                expect = 80.0
            elif name == "bench/block":
                expect = 20.0 + 40.0 * min(f["time"], 1.0)
                block_ok = block_ok and abs(v - expect) < 1e-3
            elif name == "bench/heater":
                if v > hottest[0]:
                    hottest = (v, i % 160, i // 160)
                if not (40.0 - 1e-4 <= v <= 120.0 + 1e-4):
                    aligned = False
                    bad = "加热器像素 %d 为 %.3f" % (i, v)
                continue
            else:
                expect = 30.0
            if abs(v - expect) > 1e-3:
                aligned = False
                bad = "像素 %d（%s，深度 %.3f）为 %.4f，应为 %.4f" % (i, name or "世界", depth[i], v, expect)
                break
        if not aligned:
            break
    check(aligned, "bench/ir：温度与分割、深度逐像素对齐（天空、墙、板、方块、加热器）" + ("；" + bad if bad else ""))
    check(block_ok, "方块的温度按曲线随时间变化（20 + 40 × t °C）")
    print("INFO 热点：最高 %.2f °C，位于 (%d, %d)；热点中心的投影 (%.1f, %.1f)" % (hottest[0], hottest[1], hottest[2], hu, hv))
    check(115.0 <= hottest[0] <= 120.0 + 1e-4 and abs(hottest[1] - hu) < 2.0 and abs(hottest[2] - hv) < 2.0, "热点：最高 115～120 °C，位置与投影相差不到 2 像素")

    _, _, s0 = read_npy(os.path.join(cam_dir("bench/ir_sky"), sky[-1]["thermal"]))
    check(all(v == -20.0 for v in s0), "bench/ir_sky：背向墙，全是天空温度 −20 °C")

    # 噪声：同一位姿的两台相机逐帧相减（只取 |T| < 100 °C 的像素：偏差为 g × 5 / 2，与温度无关）
    means, stds = [], []
    for fa, fb in zip(ir, noisy):
        _, _, clean = read_npy(os.path.join(cam_dir("bench/ir"), fa["thermal"]))
        _, _, dirty = read_npy(os.path.join(cam_dir("bench/ir_noisy"), fb["thermal"]))
        diff = [d - c0 for d, c0 in zip(dirty, clean) if abs(c0) < 100.0]
        m = sum(diff) / len(diff)
        s = math.sqrt(sum((x - m) ** 2 for x in diff) / (len(diff) - 1))
        means.append(m)
        stds.append(s)
    print("INFO 热像噪声：固定偏差 %.4f～%.4f °C，时间噪声 σ %.4f～%.4f K（NETD 0.05 K）" % (min(means), max(means), min(stds), max(stds)))
    check(all(abs(s - 0.05) < 0.005 for s in stds), "时间噪声的标准差 ≈ NETD 0.05 K（每帧 10% 以内）")
    check(max(means) - min(means) < 0.01 and abs(means[0]) < 10.0, "固定偏差各帧相同（在 ±5 °C 精度的范围内）")

    png = os.path.join(cam_dir("bench/ir"), ir[-1]["thermal_color"])
    check(os.path.exists(png) and open(png, "rb").read(8) == b"\x89PNG\r\n\x1a\n", "伪彩色 PNG")
    check(digest(out_a) == digest(out_b), "两次运行的热像输出逐字节相同")
    n_ir = frames(out_n, "bench/ir")
    n_noisy = frames(out_n, "bench/ir_noisy")
    same = bool(n_ir) and all(
        read_npy(os.path.join(out_n, "cameras", "bench_ir", x["thermal"]))[2] == read_npy(os.path.join(out_n, "cameras", "bench_ir_noisy", y["thermal"]))[2]
        for x, y in zip(n_ir, n_noisy))
    check(same, "--no-noise：bench/ir_noisy 与 bench/ir 相同")
    if failures:
        print("%d 项检查失败" % len(failures))
        return 1
    print("all passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
