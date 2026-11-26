#!/usr/bin/env python3
"""策略的速度指令来自 ROS2 /cmd_vel（M4-10b）：terra_headless --policy（线性夹具，不需要 ONNX Runtime）--ros2 通用配置。

用法（由 ctest 调用；需要先 source /opt/ros/humble/setup.bash）：
  check_policy_cmd_vel.py --headless <terra_headless> --assets <assets 目录> --work <工作目录>
检查：
  - 桥接订阅 /cmd_vel（stderr 写明）；
  - 发布 Twist（linear.x 0.5、linear.y 0.1、angular.z 0.2）后策略的指令随之改变；
  - 停发 0.5 s 后指令归零（与真机的超时停止相同）。
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_bridge as cb  # noqa: E402

check = cb.check


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", required=True)
    parser.add_argument("--assets", required=True)
    parser.add_argument("--work", required=True)
    args = parser.parse_args()
    cb.setup_environment()
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from geometry_msgs.msg import Twist, Vector3

    policy = os.path.join(args.assets, "tests/policy/bench.terra-policy.json")
    profile = os.path.join(args.assets, "ros2/profiles/generic.terra-ros2.json")
    library = os.path.join(args.work, "library.db")
    process, stderr_path, _ = cb.start([args.headless, "--policy", policy, "--duration", "12", "--sample-every", "100000", "--ros2", profile,
                                        "--library", library], args.work, "policy_cmd_vel")
    rclpy.init()
    node = rclpy.create_node("terra_check_policy_cmd_vel")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    cmd = node.create_publisher(Twist, "/cmd_vel", 10)
    try:
        if not check(cb.spin_until(executor, lambda: cmd.get_subscription_count() > 0, 30.0), "桥接订阅了 /cmd_vel"):
            return 1
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            cmd.publish(Twist(linear=Vector3(x=0.5, y=0.1), angular=Vector3(z=0.2)))
            cb.spin_for(executor, 0.05)
        cb.spin_for(executor, 1.5)  # 停发：超时后归零
    finally:
        node.destroy_node()
        rclpy.shutdown()
    code = cb.finish(process, 60, "terra_headless")
    err = cb.read_text(stderr_path)
    check("订阅 /cmd_vel" in err, "stderr 写明订阅 /cmd_vel")
    check("策略指令来自 /cmd_vel：0.5, 0.1" in err, "收到 Twist 后策略的指令为 (0.5, 0.1, 0.2)")
    check("策略指令来自 /cmd_vel：0, 0, 0" in err, "停发 0.5 s 后指令归零")
    return 0 if code == 0 and not cb.failures else 1


if __name__ == "__main__":
    sys.exit(main())
