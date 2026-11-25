#!/usr/bin/env python3
"""ROS2 监视器测试用的发布者（M4-08d，ros2.monitor）：发布内容已知的消息，由 test_ros2_monitor 核对。

用法：publish_known.py [--seconds N]（ROS_DOMAIN_ID 等环境由调用者设置）
发布（节点名 terra_test_publisher）：
  /terra_test/imu         sensor_msgs/Imu          100 Hz；angular_velocity.z = sin(2π·0.5·t)，t 为消息头时间；linear_acceleration.z = 9.81
  /terra_test/joints      sensor_msgs/JointState    20 Hz；name = j0..j3，position = [0.1, 0.2, 0.3, 0.4]
  /terra_test/scan        sensor_msgs/LaserScan     10 Hz；360 束，ranges[i] = 1 + i / 1000
  /terra_test/image       sensor_msgs/Image          5 Hz；rgb8 4 × 2，像素 (x, y) = (10x, 20y, 30)
  /terra_test/depth       sensor_msgs/Image          5 Hz；16UC1 2 × 2 = [1000, 2000, 0, 500]（毫米，0 为无效）
  /terra_test/compressed  sensor_msgs/CompressedImage 5 Hz；PNG 2 × 2 RGBA（红、绿、蓝、白）
  /tf_static              odom → base_link（平移 1, 2, 0）；/tf：base_link → laser（平移 0, 0, 0.3），20 Hz
  /lowstate               unitree_hg/LowState        50 Hz（有宇树消息包时）：motor_state[i].q = i / 100
"""

import argparse
import math
import struct
import sys
import time
import zlib


def png_rgba(width, height, pixels):
    """不依赖第三方库的最小 PNG（8 位 RGBA，无滤波）。"""
    def chunk(kind, data):
        body = kind + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)
    raw = b"".join(b"\x00" + bytes(pixels[y * width * 4:(y + 1) * width * 4]) for y in range(height))
    header = struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0)
    return b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) + chunk(b"IDAT", zlib.compress(raw)) + chunk(b"IEND", b"")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=60.0)
    args = parser.parse_args()

    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import CompressedImage, Image, Imu, JointState, LaserScan
    from tf2_msgs.msg import TFMessage
    try:
        from unitree_hg.msg import LowState
    except ImportError:
        LowState = None

    rclpy.init()
    node = rclpy.create_node("terra_test_publisher")
    start = time.monotonic()

    def stamp():
        t = time.time()
        msg = Time()
        msg.sec = int(t)
        msg.nanosec = int((t - int(t)) * 1e9)
        return msg, msg.sec + msg.nanosec * 1e-9

    pub_imu = node.create_publisher(Imu, "/terra_test/imu", 50)
    pub_joints = node.create_publisher(JointState, "/terra_test/joints", 10)
    pub_scan = node.create_publisher(LaserScan, "/terra_test/scan", QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
    pub_image = node.create_publisher(Image, "/terra_test/image", 5)
    pub_depth = node.create_publisher(Image, "/terra_test/depth", 5)
    pub_compressed = node.create_publisher(CompressedImage, "/terra_test/compressed", 5)
    pub_tf = node.create_publisher(TFMessage, "/tf", 10)
    pub_static = node.create_publisher(TFMessage, "/tf_static",
                                       QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
    pub_low = node.create_publisher(LowState, "/lowstate", 10) if LowState else None

    def transform(parent, child, xyz, when):
        t = TransformStamped()
        t.header.frame_id = parent
        t.header.stamp = when
        t.child_frame_id = child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
        t.transform.rotation.w = 1.0
        return t

    when, _ = stamp()
    pub_static.publish(TFMessage(transforms=[transform("odom", "base_link", (1.0, 2.0, 0.0), when)]))

    def imu():
        msg = Imu()
        msg.header.stamp, t = stamp()
        msg.header.frame_id = "imu_link"
        msg.angular_velocity.z = math.sin(2 * math.pi * 0.5 * t)
        msg.linear_acceleration.z = 9.81
        pub_imu.publish(msg)

    def joints():
        msg = JointState()
        msg.header.stamp, _ = stamp()
        msg.name = ["j0", "j1", "j2", "j3"]
        msg.position = [0.1, 0.2, 0.3, 0.4]
        pub_joints.publish(msg)
        when, _ = stamp()
        pub_tf.publish(TFMessage(transforms=[transform("base_link", "laser", (0.0, 0.0, 0.3), when)]))

    def scan():
        msg = LaserScan()
        msg.header.stamp, _ = stamp()
        msg.header.frame_id = "laser"
        msg.angle_min, msg.angle_max = -math.pi, math.pi
        msg.angle_increment = 2 * math.pi / 360
        msg.range_min, msg.range_max = 0.1, 12.0
        msg.ranges = [1.0 + i / 1000.0 for i in range(360)]
        pub_scan.publish(msg)

    def images():
        rgb = Image()
        rgb.header.stamp, _ = stamp()
        rgb.width, rgb.height, rgb.encoding, rgb.step = 4, 2, "rgb8", 12
        rgb.data = bytes(v for y in range(2) for x in range(4) for v in (10 * x, 20 * y, 30))
        pub_image.publish(rgb)
        depth = Image()
        depth.header.stamp = rgb.header.stamp
        depth.width, depth.height, depth.encoding, depth.step = 2, 2, "16UC1", 4
        depth.is_bigendian = 0
        depth.data = struct.pack("<4H", 1000, 2000, 0, 500)
        pub_depth.publish(depth)
        comp = CompressedImage()
        comp.header.stamp = rgb.header.stamp
        comp.format = "png"
        comp.data = png_rgba(2, 2, [255, 0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255, 255, 255, 255, 255])
        pub_compressed.publish(comp)

    def lowstate():
        msg = LowState()
        for i in range(len(msg.motor_state)):
            msg.motor_state[i].q = i / 100.0
        pub_low.publish(msg)

    node.create_timer(0.01, imu)
    node.create_timer(0.05, joints)
    node.create_timer(0.1, scan)
    node.create_timer(0.2, images)
    if pub_low:
        node.create_timer(0.02, lowstate)
    print("publish_known: 开始发布%s" % ("（含 /lowstate）" if pub_low else "（没有宇树消息包，不发布 /lowstate）"), flush=True)
    try:
        while rclpy.ok() and time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
