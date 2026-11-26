#!/usr/bin/env python3
"""传感器噪声的出口检查（M4-09）：夹具 assets/tests/sensors/noise_suite.terra-scene.json。

用法（由 ctest 调用）：
  check_noise.py headless --headless <terra_headless> --scene <场景> --work <目录>
  check_noise.py capture  --capture <terra_capture> --scene <场景> --work <目录>
headless：
  - 同一场景两次 --sensors 输出逐字节相同；stderr 写明“传感器噪声：开启（种子 42”；
  - --no-noise：配置了噪声的 bench/level_imu 与两个雷达不同，没有配置的 bench/tilt_imu 逐字节相同；
  - 实验记录：头部写明 noise true / false；带噪声与不带噪声的记录复现都逐位相同（sensors_bitwise）。
capture：
  - 两次采集逐字节相同；capture.json 写明 noise 与 noise_seed；
  - --no-noise 时 IMU、雷达、彩色与深度图像与带噪声时不同；
  - 分割图：每帧一个 seg/<步序号>.npy（uint16）与 seg_labels.json（ID → 实体与类别）。
"""

import argparse
import filecmp
import json
import os
import re
import shutil
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
        print("命令失败（%d）：%s\n%s\n%s" % (p.returncode, " ".join(cmd), p.stdout[-2000:], p.stderr[-2000:]))
    return p


def lines_by_name(text, kind):
    out = {}
    for line in text.splitlines():
        if '"type":"%s"' % kind not in line:
            continue
        m = re.search(r'"name":"([^"]+)"', line)
        if m:
            out.setdefault(m.group(1), []).append(line)
    return out


def headless(args):
    base = [args.headless, "--scene", args.scene, "--steps", "300", "--sensors"]
    a = run(base)
    b = run(base)
    check(a.returncode == 0 and a.stdout == b.stdout, "两次 --sensors 输出逐字节相同（%d 行）" % len(a.stdout.splitlines()))
    check("传感器噪声：开启（种子 42" in a.stderr, "stderr 写明噪声开启与种子")
    truth = run(base + ["--no-noise"])
    noisy_imu, truth_imu = lines_by_name(a.stdout, "sensor"), lines_by_name(truth.stdout, "sensor")
    noisy_lidar, truth_lidar = lines_by_name(a.stdout, "lidar"), lines_by_name(truth.stdout, "lidar")
    check(noisy_imu.get("bench/level_imu") and noisy_imu.get("bench/level_imu") != truth_imu.get("bench/level_imu"),
          "--no-noise：配置了噪声的 IMU 读数不同")
    check(noisy_imu.get("bench/tilt_imu") and noisy_imu.get("bench/tilt_imu") == truth_imu.get("bench/tilt_imu"),
          "没有配置噪声的 IMU 逐字节相同")
    check(all(noisy_lidar.get(n) and noisy_lidar.get(n) != truth_lidar.get(n) for n in ("room/scan", "room/scan_a1")),
          "--no-noise：两个雷达的读数不同（含预设 rplidar_a1m8）")
    check("传感器噪声" not in truth.stderr, "--no-noise 时不提示噪声开启")
    os.makedirs(args.work, exist_ok=True)
    for name, extra, expected in (("noisy", [], True), ("truth", ["--no-noise"], False)):
        record = os.path.join(args.work, name + ".terra-record.jsonl")
        if os.path.exists(record):
            os.remove(record)
        r = run([args.headless, "--scene", args.scene, "--steps", "300", "--record", record] + extra)
        header = json.loads(open(record, encoding="utf-8").readline()) if r.returncode == 0 else {}
        noise = header.get("header", header).get("sensors", {}).get("noise")
        check(noise is expected, "%s 记录：头部 noise = %s" % (name, str(expected).lower()))
        rr = run([args.headless, "--rerun", record])
        check(rr.returncode == 0 and '"sensors_bitwise":true' in rr.stdout and '"sensor_samples":0' not in rr.stdout,
              "%s 记录复现：传感器逐位相同" % name)


def capture(args):
    outs = {}
    for name, extra in (("a", []), ("b", []), ("truth", ["--no-noise"])):
        out = os.path.join(args.work, name)
        shutil.rmtree(out, ignore_errors=True)
        r = run([args.capture, "--scene", args.scene, "--steps", "40", "--output", out] + extra)
        check(r.returncode == 0, "采集 %s" % name)
        outs[name] = out
    cmp = filecmp.dircmp(outs["a"], outs["b"])

    def same_tree(d):
        return not d.left_only and not d.right_only and not d.diff_files and not d.funny_files and all(same_tree(s) for s in d.subdirs.values())

    def deep_same(left, right):
        for root, _, files in os.walk(left):
            for f in files:
                l = os.path.join(root, f)
                r = os.path.join(right, os.path.relpath(l, left))
                if not os.path.exists(r) or not filecmp.cmp(l, r, shallow=False):
                    return False
        return True

    check(same_tree(cmp) and deep_same(outs["a"], outs["b"]), "两次采集逐字节相同")
    meta = json.load(open(os.path.join(outs["a"], "capture.json"), encoding="utf-8"))
    truth_meta = json.load(open(os.path.join(outs["truth"], "capture.json"), encoding="utf-8"))
    check(meta.get("noise") is True and meta.get("noise_seed") == 42 and truth_meta.get("noise") is False, "capture.json 写明噪声与种子")
    sensors = open(os.path.join(outs["a"], "sensors.jsonl"), encoding="utf-8").read()
    truth_sensors = open(os.path.join(outs["truth"], "sensors.jsonl"), encoding="utf-8").read()
    check(lines_by_name(sensors, "sensor").get("bench/level_imu") != lines_by_name(truth_sensors, "sensor").get("bench/level_imu"),
          "--no-noise：IMU 不同")
    check(lines_by_name(sensors, "lidar").get("room/scan") != lines_by_name(truth_sensors, "lidar").get("room/scan"), "--no-noise：雷达不同")
    cam = os.path.join("cameras", "cam_eye")
    rgb = sorted(os.listdir(os.path.join(outs["a"], cam, "rgb")))
    depth = sorted(os.listdir(os.path.join(outs["a"], cam, "depth")))
    check(rgb and not filecmp.cmp(os.path.join(outs["a"], cam, "rgb", rgb[0]), os.path.join(outs["truth"], cam, "rgb", rgb[0]), shallow=False),
          "--no-noise：彩色图像不同")
    check(depth and not filecmp.cmp(os.path.join(outs["a"], cam, "depth", depth[0]), os.path.join(outs["truth"], cam, "depth", depth[0]), shallow=False),
          "--no-noise：深度图像不同")
    seg_dir = os.path.join(outs["a"], cam, "seg")
    segs = sorted(os.listdir(seg_dir)) if os.path.isdir(seg_dir) else []
    check(len(segs) == len(rgb) and segs, "分割图：每帧一个 .npy（%d 个）" % len(segs))
    if segs:
        data = open(os.path.join(seg_dir, segs[0]), "rb").read()
        check(data[:6] == b"\x93NUMPY" and b"'descr': '<u2'" in data[:128], "分割图为 uint16 的 NumPy 文件")
        check(filecmp.cmp(os.path.join(seg_dir, segs[0]), os.path.join(outs["truth"], cam, "seg", segs[0]), shallow=False),
              "分割图不受噪声影响")
    labels_path = os.path.join(outs["a"], cam, "seg_labels.json")
    labels = json.load(open(labels_path, encoding="utf-8")) if os.path.exists(labels_path) else {}
    entries = labels.get("labels", [])
    check(any(e.get("entity") == "cam/target" and e.get("label") == "bench" for e in entries) and labels.get("background") == 0,
          "对照表：cam/target 的类别为 bench，0 为背景")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("mode", choices=["headless", "capture"])
    p.add_argument("--headless")
    p.add_argument("--capture")
    p.add_argument("--scene", required=True)
    p.add_argument("--work", required=True)
    args = p.parse_args()
    if args.mode == "headless":
        headless(args)
    else:
        capture(args)
    print("all passed" if failures == 0 else "%d 项失败" % failures)
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
