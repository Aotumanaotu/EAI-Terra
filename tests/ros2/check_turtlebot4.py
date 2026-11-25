#!/usr/bin/env python3
"""TurtleBot 4 的 ROS2 通信配置（M4-08b）：terra_headless --ros2 turtlebot4.terra-ros2.json，与真机相同的话题名、坐标系与差速底盘。

用法（由 ctest 调用；需要先 source /opt/ros/humble/setup.bash，并已运行 scripts/assets/import_turtlebot4.sh）：
  check_turtlebot4.py --headless <terra_headless> --assets <assets 目录> --work <工作目录>
检查：
  - 话题与类型：/scan（LaserScan，frame rplidar_link，1080 束）、/imu（frame imu_link）、/odom（Odometry，odom → base_link）、
    /joint_states（left_wheel_joint、right_wheel_joint，没有实例前缀）、/cmd_vel 的订阅者；没有默认的 /turtlebot4/... 话题；
  - /tf：odom → base_link，连杆之间为相对位姿（如 base_link → rplidar_link 的父子关系），没有 world；
  - /cmd_vel：v = 0.2 m/s 时 /odom 的线速度约 0.2，2 s 后位置约 0.4 m；w = 1.0 rad/s 时角速度约 1.0；
  - 停发 /cmd_vel 0.5 s 后停车（与 Create 3 相同）；/terra/reset 后里程计回到 0。
场景未导入时返回 77（跳过）。
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_bridge as cb  # noqa: E402

check = cb.check
info = cb.info


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--headless", required=True)
    parser.add_argument("--assets", required=True)
    parser.add_argument("--work", required=True)
    args = parser.parse_args()
    scene = os.path.join(args.assets, "robots/turtlebot4/turtlebot4.terra-scene.json")
    if not os.path.exists(scene):
        print("SKIP：没有导入 TurtleBot 4（%s）；请运行 bash scripts/assets/import_turtlebot4.sh" % scene)
        return 77
    cb.setup_environment()
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import Twist, Vector3
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, JointState, LaserScan
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage

    profile = os.path.join(args.assets, "ros2/profiles/turtlebot4.terra-ros2.json")
    process, stderr_path, _ = cb.start([args.headless, "--scene", scene, "--duration", "25", "--sample-every", "100000", "--ros2", profile],
                                       args.work, "turtlebot4")
    rclpy.init()
    node = rclpy.create_node("terra_check_turtlebot4")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    reliable = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
    got = {"scan": [], "imu": [], "odom": [], "joints": [], "tf": []}
    node.create_subscription(LaserScan, "/scan", got["scan"].append, qos_profile_sensor_data)
    node.create_subscription(Imu, "/imu", got["imu"].append, qos_profile_sensor_data)
    node.create_subscription(Odometry, "/odom", got["odom"].append, reliable)
    node.create_subscription(JointState, "/joint_states", got["joints"].append, reliable)
    node.create_subscription(TFMessage, "/tf", got["tf"].append, reliable)
    cmd = node.create_publisher(Twist, "/cmd_vel", 10)
    reset = node.create_client(Trigger, "/terra/reset")

    def drive(v, w, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            cmd.publish(Twist(linear=Vector3(x=float(v)), angular=Vector3(z=float(w))))
            cb.spin_for(executor, 0.05)

    try:
        if not check(cb.spin_until(executor, lambda: bool(got["odom"]) and cmd.get_subscription_count() > 0, 30.0),
                     "turtlebot4：30 s 内收到 /odom，且桥接订阅了 /cmd_vel"):
            return 1
        cb.spin_for(executor, 2.0)  # 落稳
        names = dict(node.get_topic_names_and_types())
        expected = {"/scan": "sensor_msgs/msg/LaserScan", "/imu": "sensor_msgs/msg/Imu", "/odom": "nav_msgs/msg/Odometry",
                    "/cmd_vel": "geometry_msgs/msg/Twist", "/joint_states": "sensor_msgs/msg/JointState",
                    "/oakd/rgb/preview/image_raw": "sensor_msgs/msg/Image", "/oakd/rgb/preview/camera_info": "sensor_msgs/msg/CameraInfo",
                    "/oakd/stereo/image_raw": "sensor_msgs/msg/Image"}
        for topic, kind in expected.items():
            check(names.get(topic) == [kind], "话题 %s 的类型为 %s（实际 %s）" % (topic, kind, names.get(topic)))
        leftovers = [t for t in names if t.startswith("/turtlebot4/")]
        check(not leftovers, "没有留下默认的 /turtlebot4/... 话题（%s）" % leftovers)
        if check(bool(got["scan"]), "收到 /scan"):
            s = got["scan"][-1]
            hits = sum(1 for r in s.ranges if math.isfinite(r))
            info("/scan：%d 束，命中 %d，frame %s" % (len(s.ranges), hits, s.header.frame_id))
            check(len(s.ranges) == 1080 and s.header.frame_id == "rplidar_link", "/scan：1080 束，frame_id 为 rplidar_link（与真机相同）")
        if check(bool(got["imu"]), "收到 /imu"):
            m = got["imu"][-1]
            check(m.header.frame_id == "imu_link" and abs(m.linear_acceleration.z - 9.81) < 0.5, "/imu：frame_id 为 imu_link，静止时比力约 9.81")
        if check(bool(got["joints"]), "收到 /joint_states"):
            check(sorted(got["joints"][-1].name) == ["left_wheel_joint", "right_wheel_joint"], "/joint_states：两个轮子关节，没有实例前缀")
        frames = {}
        for m in got["tf"][-20:]:
            for t in m.transforms:
                frames[t.child_frame_id] = t.header.frame_id
        info("/tf：%d 个坐标系，base_link 的父 %s，rplidar_link 的父 %s" % (len(frames), frames.get("base_link"), frames.get("rplidar_link")))
        check(frames.get("base_link") == "odom" and "world" not in frames.values() and frames.get("rplidar_link") not in (None, "world"),
              "/tf：odom → base_link，连杆之间为相对位姿，没有 world")
        odom = got["odom"][-1]
        check(odom.header.frame_id == "odom" and odom.child_frame_id == "base_link", "/odom：odom → base_link")

        # 直行 0.2 m/s 2 s
        start = got["odom"][-1].pose.pose.position.x
        drive(0.2, 0.0, 2.0)
        o = got["odom"][-1]
        info("/cmd_vel 0.2 m/s 2 s：里程计速度 %.3f m/s，位移 %.3f m" % (o.twist.twist.linear.x, o.pose.pose.position.x - start))
        check(abs(o.twist.twist.linear.x - 0.2) < 0.04, "/odom 的线速度约 0.2 m/s")
        check(0.25 < o.pose.pose.position.x - start < 0.5, "/odom 的位移约 0.4 m（含加速）")
        # 停发后 0.5 s 超时停车
        cb.spin_for(executor, 1.2)
        o = got["odom"][-1]
        check(abs(o.twist.twist.linear.x) < 0.02, "停发 /cmd_vel 后超时停车（线速度 %.3f）" % o.twist.twist.linear.x)
        # 原地转向 1.0 rad/s
        drive(0.0, 1.0, 1.5)
        o = got["odom"][-1]
        info("/cmd_vel 1.0 rad/s：里程计角速度 %.3f rad/s" % o.twist.twist.angular.z)
        check(abs(o.twist.twist.angular.z - 1.0) < 0.15, "/odom 的角速度约 1.0 rad/s（左转为正）")
        # 重置后里程计回到 0
        cb.call(executor, reset, Trigger.Request(), "turtlebot4 重置")
        cb.spin_for(executor, 0.5)
        o = got["odom"][-1]
        check(abs(o.pose.pose.position.x) < 0.02 and abs(o.pose.pose.position.y) < 0.02, "重置后里程计回到 0")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        cb.finish(process, 60, "turtlebot4")
    if cb.failures:
        print("%d 项检查失败" % len(cb.failures))
        return 1
    print("all passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
