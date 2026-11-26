#!/usr/bin/env python3
"""3D 激光雷达的 ROS2 输出（M5-11）：terra_headless --ros2 通用配置，夹具 lidar3d_room。

用法（由 ctest 调用；需要先 source /opt/ros/humble/setup.bash）：
  check_lidar3d_ros2.py --headless <terra_headless> --assets <assets 目录> --work <工作目录>
检查：
  - /room/scan3d/points 为 sensor_msgs/PointCloud2，约 10 Hz；
  - 字段与 livox_ros_driver2 的 PointCloud2 相同：x、y、z、intensity（FLOAT32）、tag、line（UINT8）、timestamp（FLOAT64），point_step 26；
  - 每帧 540 个点；frame_id 为 room/scan3d；消息头时刻 = 第一点的测量时刻，每点 timestamp（纳秒）按序号递增 1 / 5400 s；
  - 点在房间的墙面上（最近约 1.06 m；最远 4.0 m，墙角处方位 30°、天顶角 60° 的射线）；内置 IMU：/room/scan3d_imu/imu 有消息。
"""

import argparse
import math
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_bridge as cb  # noqa: E402

check = cb.check


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", required=True)
    ap.add_argument("--assets", required=True)
    ap.add_argument("--work", required=True)
    a = ap.parse_args()
    cb.setup_environment()
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, PointCloud2, PointField

    scene = os.path.join(a.assets, "tests/sensors/lidar3d_room.terra-scene.json")
    profile = os.path.join(a.assets, "ros2/profiles/generic.terra-ros2.json")
    process, _, _ = cb.start([a.headless, "--scene", scene, "--duration", "12", "--sample-every", "100000", "--ros2", profile],
                             a.work, "lidar3d_ros2")
    rclpy.init()
    node = rclpy.create_node("terra_check_lidar3d")
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    clouds, imus = [], []
    node.create_subscription(PointCloud2, "/room/scan3d/points", clouds.append, qos_profile_sensor_data)
    node.create_subscription(Imu, "/room/scan3d_imu/imu", imus.append, qos_profile_sensor_data)
    try:
        ok = cb.spin_until(ex, lambda: len(clouds) >= 12 and len(imus) > 10, 40.0)
        check(ok, "40 s 内收到点云（%d）与内置 IMU（%d）" % (len(clouds), len(imus)))
        if clouds:
            m = clouds[-1]
            fields = [(f.name, f.offset, f.datatype) for f in m.fields]
            check(fields == [("x", 0, PointField.FLOAT32), ("y", 4, PointField.FLOAT32), ("z", 8, PointField.FLOAT32),
                             ("intensity", 12, PointField.FLOAT32), ("tag", 16, PointField.UINT8), ("line", 17, PointField.UINT8),
                             ("timestamp", 18, PointField.FLOAT64)] and m.point_step == 26, "字段与 livox_ros_driver2 的 PointCloud2 相同")
            check(m.width == 540 and m.height == 1 and len(m.data) == 540 * 26 and m.header.frame_id == "room/scan3d", "每帧 540 点，frame_id room/scan3d")
            stamp = m.header.stamp.sec * 1e9 + m.header.stamp.nanosec
            times = [struct.unpack_from("<d", bytes(m.data), i * 26 + 18)[0] for i in range(m.width)]
            rng = [math.sqrt(sum(v * v for v in struct.unpack_from("<fff", bytes(m.data), i * 26))) for i in range(m.width)]
            lines = [m.data[i * 26 + 17] for i in range(m.width)]
            check(abs(times[0] - stamp) < 2 and abs((times[1] - times[0]) - 1e9 / 5400) < 1.0, "消息头 = 第一点时刻；每点时间按 1 / 5400 s 递增")
            check(3.9 < max(rng) < 4.01 and 1.0 < min(rng) < 1.2 and set(lines) == {0, 1, 2, 3}, "点在房间的墙面上（最近约 1.06 m，最远 4.0 m 在墙角），4 条线")
            stamps = [c.header.stamp.sec + c.header.stamp.nanosec * 1e-9 for c in clouds[-10:]]
            gaps = [b - a2 for a2, b in zip(stamps, stamps[1:])]
            check(all(abs(g - 0.1) < 1e-6 for g in gaps), "相邻两帧的时间间隔 0.1 s（10 Hz，仿真时钟）")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    code = cb.finish(process, 60, "terra_headless")
    return 0 if code == 0 and not cb.failures else 1


if __name__ == "__main__":
    sys.exit(main())
