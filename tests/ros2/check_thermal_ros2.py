#!/usr/bin/env python3
"""热像仪的 ROS2 输出（M5-06）：terra_viewer --ros2 通用配置，夹具 thermal_bench（三台 FLIR Lepton 3.5）。

用法（由 ctest 在图形环境中调用；需要先 source /opt/ros/humble/setup.bash）：
  check_thermal_ros2.py --viewer <terra_viewer> --assets <assets 目录> --work <工作目录>
检查：
  - /bench/ir/thermal/image_raw：mono16，160 × 120，step 320；图像中心在 80 °C 的板上：(80 + 273.15) × 100 = 35315（无噪声）；
  - /bench/ir/thermal/image_color：rgb8，160 × 120；/bench/ir/thermal/camera_info：内参与彩色相机相同（cx 79.5、cy 59.5），
    坐标系 bench/ir_optical；与 image_raw 的时间戳相同；
  - /bench/ir_sky/thermal/image_raw：天空 −20 °C 低于量程下限 −10 °C，截断为 (−10 + 273.15) × 100 = 26315；
  - /bench/ir_noisy/thermal/image_raw：带噪声（与无噪声的读数不同），没有彩色话题（rgb: false）。
"""

import argparse
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_bridge as cb  # noqa: E402

check = cb.check


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--viewer", required=True)
    ap.add_argument("--assets", required=True)
    ap.add_argument("--work", required=True)
    a = ap.parse_args()
    cb.setup_environment()
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    scene = os.path.join(a.assets, "tests/sensors/thermal_bench.terra-scene.json")
    profile = os.path.join(a.assets, "ros2/profiles/generic.terra-ros2.json")
    os.makedirs(a.work, exist_ok=True)
    process, _, _ = cb.start([a.viewer, "--scene", scene, "--ros2", profile, "--seconds", "40",
                              "--settings", os.path.join(a.work, "settings.json"),
                              "--library", os.path.join(a.work, "library.sqlite3")], a.work, "viewer")
    rclpy.init()
    node = rclpy.create_node("terra_check_thermal")
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    got = {k: [] for k in ("raw", "color", "info", "sky", "noisy")}
    node.create_subscription(Image, "/bench/ir/thermal/image_raw", got["raw"].append, qos_profile_sensor_data)
    node.create_subscription(Image, "/bench/ir/thermal/image_color", got["color"].append, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, "/bench/ir/thermal/camera_info", got["info"].append, qos_profile_sensor_data)
    node.create_subscription(Image, "/bench/ir_sky/thermal/image_raw", got["sky"].append, qos_profile_sensor_data)
    node.create_subscription(Image, "/bench/ir_noisy/thermal/image_raw", got["noisy"].append, qos_profile_sensor_data)
    try:
        ok = cb.spin_until(ex, lambda: all(len(v) >= 3 for v in got.values()), 60.0)
        check(ok, "60 s 内收到热像话题（%s）" % ", ".join("%s %d" % (k, len(v)) for k, v in got.items()))
        if got["raw"] and got["color"] and got["info"]:
            raw, color, ci = got["raw"][-1], got["color"][-1], got["info"][-1]
            check(raw.encoding == "mono16" and raw.width == 160 and raw.height == 120 and raw.step == 320 and len(raw.data) == 160 * 120 * 2 and
                  raw.is_bigendian == 0, "image_raw：mono16，160 × 120")
            data = bytes(raw.data)
            center = struct.unpack_from("<H", data, (60 * 160 + 80) * 2)[0]
            cb.info("图像中心的读数 %d（0.01 K）" % center)
            check(center == 35315, "图像中心在 80 °C 的板上：35315（0.01 K）")
            check(color.encoding == "rgb8" and color.width == 160 and color.height == 120 and len(color.data) == 160 * 120 * 3, "image_color：rgb8，160 × 120")
            k = list(ci.k)
            check(ci.width == 160 and ci.height == 120 and abs(k[2] - 79.5) < 1e-9 and abs(k[5] - 59.5) < 1e-9 and ci.header.frame_id == "bench/ir_optical" and
                  raw.header.frame_id == "bench/ir_optical", "camera_info：内参 cx 79.5、cy 59.5，坐标系 bench/ir_optical")
            stamps = lambda lst: set((m.header.stamp.sec, m.header.stamp.nanosec) for m in lst)
            check(len(stamps(got["raw"]) & stamps(got["info"])) >= 2, "image_raw 与 camera_info 来自同一步")
        if got["sky"]:
            values = struct.unpack("<%dH" % (160 * 120), bytes(got["sky"][-1].data))
            check(all(v == 26315 for v in values), "背向墙看天空：−20 °C 低于量程，截断为 −10 °C（26315）")
        if got["noisy"] and got["raw"]:
            a1 = bytes(got["noisy"][-1].data)
            check(a1 != bytes(got["raw"][-1].data), "有噪声的热像仪读数与无噪声的不同")
        topics = dict(node.get_topic_names_and_types())
        check("/bench/ir_noisy/image_raw" not in topics and "/bench/ir/image_raw" in topics, "rgb: false 的热像仪没有彩色话题；bench/ir 有")
    finally:
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        cb.finish(process, 60, "terra_viewer")
    if cb.failures:
        print("%d 项检查失败" % len(cb.failures))
        return 1
    print("all passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
