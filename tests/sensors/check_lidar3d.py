#!/usr/bin/env python3
"""3D 激光雷达的出口检查（M5-11）：夹具 assets/tests/sensors/lidar3d_room.terra-scene.json。

用法（由 ctest 调用）：
  check_lidar3d.py --headless <terra_headless> --capture <terra_capture|""> --scene <场景> --work <目录>
检查：
  - Headless --sensors：每个 3D 雷达按 10 Hz 输出 lidar3d 行（帧序号、第一点时刻、射线数、点数、距离范围、摘要）；
    两次运行逐字节相同；--no-noise 时带噪声的雷达摘要改变、没有噪声的不变；
  - 实验记录：头部有 lidars3d；每帧一行 lidar3d（点数与摘要）；复现逐位相同（sensors_bitwise）；
  - terra_capture（给出时）：lidars3d/<名>/frames.jsonl 与每帧 .npy（float32，形状 (点数, 5)），点数与 Headless 一致。
"""

import argparse
import json
import os
import shutil
import struct
import subprocess
import sys

failures = 0


def check(ok, what):
    global failures
    print(("PASS " if ok else "FAILED: ") + what)
    if not ok:
        failures += 1


def run(cmd):
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode != 0:
        print("命令失败（%d）：%s\n%s" % (p.returncode, " ".join(cmd), p.stderr[-2000:]))
    return p


def lidar3d_lines(text):
    return [json.loads(line) for line in text.splitlines() if '"type":"lidar3d"' in line]


def npy_shape(path):
    with open(path, "rb") as f:
        data = f.read(128)
    if data[:6] != b"\x93NUMPY":
        return None
    n = struct.unpack("<H", data[8:10])[0]
    header = data[10:10 + n].decode("latin1")
    shape = header.split("'shape': (")[1].split(")")[0]
    return tuple(int(x) for x in shape.split(",") if x.strip()), "'<f4'" in header


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", required=True)
    ap.add_argument("--capture", default="")
    ap.add_argument("--scene", required=True)
    ap.add_argument("--work", required=True)
    a = ap.parse_args()
    os.makedirs(a.work, exist_ok=True)
    env_lib = ["--library", os.path.join(a.work, "library.db")]
    base = [a.headless, "--scene", a.scene, "--steps", "200", "--sample-every", "100", "--sensors"]
    r1, r2 = run(base), run(base)
    lines = lidar3d_lines(r1.stdout)
    by = {}
    for l in lines:
        by.setdefault(l["name"], []).append(l)
    check(r1.returncode == 0 and r1.stdout == r2.stdout, "两次 --sensors 输出逐字节相同")
    check(len(by) == 6 and all(len(v) == 5 for v in by.values()), "6 个 3D 雷达，各 5 帧（第 0、50、100、150、200 步）")
    scan = by.get("room/scan3d", [])
    check(bool(scan) and all(x["rays"] == 540 and x["points"] == 540 for x in scan) and [x["frame"] for x in scan] == [0, 1, 2, 3, 4],
          "每帧 540 条射线、540 个点，帧序号 0～4")
    check(bool(scan) and abs(scan[1]["frame_start"] - (scan[1]["time"] - 539 / 5400)) < 1e-12, "第一点时刻 = 当前 − 539 ÷ 点频")
    check(bool(scan) and 1.0 < scan[0]["range_min"] < 1.2 and 3.9 < scan[0]["range_max"] < 4.01, "距离范围与房间一致（最近约 1.06 m，最远 4.0 m 在墙角）")
    q = run(base + ["--no-noise"])
    qb = {}
    for l in lidar3d_lines(q.stdout):
        qb.setdefault(l["name"], []).append(l)
    check([x["sha256"] for x in qb.get("room/range_noise", [])] != [x["sha256"] for x in by.get("room/range_noise", [])],
          "--no-noise：带噪声的雷达读数改变")
    check([x["sha256"] for x in qb.get("room/scan3d", [])] == [x["sha256"] for x in by.get("room/scan3d", [])], "--no-noise：没有噪声的雷达不变")
    # 记录与复现
    record = os.path.join(a.work, "lidar3d.terra-record.jsonl")
    if os.path.exists(record):
        os.remove(record)
    rr = run([a.headless, "--scene", a.scene, "--steps", "200", "--record", record] + env_lib)
    head = json.loads(open(record, encoding="utf-8").readline()) if rr.returncode == 0 else {}
    l3 = head.get("sensors", {}).get("lidars3d", [])
    frames = [json.loads(line) for line in open(record, encoding="utf-8") if '"type":"lidar3d"' in line] if rr.returncode == 0 else []
    check(len(l3) == 6 and l3[0].get("points_per_frame") == 540, "记录头有 lidars3d（6 个，第一个每帧 540 点）")
    check(len(frames) == 30 and all(len(f.get("sha256", "")) == 64 for f in frames), "记录中每帧一行 lidar3d（30 行，带摘要）")
    re = run([a.headless, "--rerun", record])
    check(re.returncode == 0 and '"bitwise":true' in re.stdout and '"sensors_bitwise":true' in re.stdout, "复现逐位相同（含 3D 雷达）：" + re.stdout.strip()[:160])
    # terra_capture
    if a.capture:
        out = os.path.join(a.work, "capture")
        shutil.rmtree(out, ignore_errors=True)
        c = run([a.capture, "--scene", a.scene, "--steps", "200", "--output", out])
        d = os.path.join(out, "lidars3d", "room_scan3d")
        flines = [json.loads(line) for line in open(os.path.join(d, "frames.jsonl"), encoding="utf-8")] if c.returncode == 0 else []
        shapes = [npy_shape(os.path.join(d, f["file"])) for f in flines]
        check(c.returncode == 0 and len(flines) == 5 and all(s == ((540, 5), True) for s in shapes), "terra_capture：5 帧 .npy，float32，形状 (540, 5)")
        check([f["sha256"] for f in flines] == [x["sha256"] for x in scan], "terra_capture 的点云摘要与 Headless 相同")
    else:
        print("SKIP terra_capture：渲染关闭的构建没有此程序")
    print("all passed" if failures == 0 else "%d 项失败" % failures)
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
