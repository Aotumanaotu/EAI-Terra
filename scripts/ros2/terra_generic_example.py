#!/usr/bin/env python3
"""EAI Terra ROS2 桥接（通用配置）的 Python 示例（M4-06a）。只用 ROS2 Humble 自带的包（rclpy、sensor_msgs、std_srvs）。

先启动仿真（任选其一）：
  terra_viewer   --scene assets/scenes/robots/stretch3.terra-scene.json --ros2 assets/ros2/profiles/generic.terra-ros2.json
  terra_headless --scene assets/scenes/robots/stretch3.terra-scene.json --duration 600 --ros2 assets/ros2/profiles/generic.terra-ros2.json
再 source /opt/ros/humble/setup.bash，运行本示例：
  python3 scripts/ros2/terra_generic_example.py                       # 每秒打印仿真时间、关节数、IMU 与雷达的摘要
  python3 scripts/ros2/terra_generic_example.py --wave stretch3/head_pan --amplitude 1.0 --period 6
                                                                     # 给位置执行器发正弦目标（头部左右转动，相机随之扫过）
  python3 scripts/ros2/terra_generic_example.py --pause | --resume | --step 10 | --reset
                                                                     # 调用 /terra/pause、/terra/step、/terra/reset 后退出

约定（与真机程序相同的写法）：
  - 时间：订阅 /clock，以仿真时间为准（rclpy 节点设 use_sim_time=true）；
  - 命令：/terra/joint_commands（sensor_msgs/JointState），name 为执行器全名（实例名/执行器名）；
    位置执行器与 PD 层接管的执行器填 position，速度执行器填 velocity，力矩执行器填 effort；
    越界、缺字段、未知名字的条目被拒绝（仿真程序退出时打印拒绝的条数与最近一次的原因）。
  - 话题名中的 “-” 等字符会换成 “_”。
"""

import argparse
import math
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--wave", metavar="执行器", help="给该位置执行器发正弦目标（例如 stretch3/head_pan）")
    parser.add_argument("--amplitude", type=float, default=0.5, help="正弦幅度（执行器单位，默认 0.5）")
    parser.add_argument("--center", type=float, default=0.0, help="正弦中心（默认 0）")
    parser.add_argument("--period", type=float, default=4.0, help="正弦周期（仿真秒，默认 4）")
    parser.add_argument("--rate", type=float, default=50.0, help="命令频率（Hz，按仿真时间，默认 50）")
    parser.add_argument("--imu", default=None, help="IMU 话题（默认自动找第一个 */imu）")
    parser.add_argument("--scan", default=None, help="雷达话题（默认自动找第一个 */scan）")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--pause", action="store_true", help="暂停仿真后退出")
    group.add_argument("--resume", action="store_true", help="继续仿真后退出")
    group.add_argument("--step", type=int, metavar="N", help="暂停时单步 N 次后退出")
    group.add_argument("--reset", action="store_true", help="重置仿真后退出")
    args = parser.parse_args()

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import Imu, JointState, LaserScan
        from std_srvs.srv import SetBool, Trigger
    except ImportError as e:
        print("无法导入 rclpy（%s）。请先 source /opt/ros/humble/setup.bash" % e, file=sys.stderr)
        return 1

    rclpy.init()
    node = Node("terra_generic_example", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])

    def call(client, request, label):
        if not client.wait_for_service(timeout_sec=5.0):
            print("%s：服务不可用（仿真程序是否以 --ros2 运行？）" % label, file=sys.stderr)
            return False
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        r = future.result()
        print("%s：%s" % (label, r.message if r else "无应答"))
        return bool(r and r.success)

    # 服务：调用后退出
    if args.pause or args.resume:
        ok = call(node.create_client(SetBool, "/terra/pause"), SetBool.Request(data=args.pause), "/terra/pause")
        rclpy.shutdown()
        return 0 if ok else 1
    if args.reset:
        ok = call(node.create_client(Trigger, "/terra/reset"), Trigger.Request(), "/terra/reset")
        rclpy.shutdown()
        return 0 if ok else 1
    if args.step is not None:
        client = node.create_client(Trigger, "/terra/step")
        ok = all(call(client, Trigger.Request(), "/terra/step %d/%d" % (i + 1, args.step)) for i in range(args.step))
        rclpy.shutdown()
        return 0 if ok else 1

    # 自动找传感器话题
    rclpy.spin_once(node, timeout_sec=1.0)
    topics = dict(node.get_topic_names_and_types())
    imu_topic = args.imu or next((t for t, ty in sorted(topics.items()) if "sensor_msgs/msg/Imu" in ty), None)
    scan_topic = args.scan or next((t for t, ty in sorted(topics.items()) if "sensor_msgs/msg/LaserScan" in ty), None)
    print("话题：IMU %s，雷达 %s" % (imu_topic or "（无）", scan_topic or "（无）"))

    state = {"joints": None, "imu": None, "scan": None}
    node.create_subscription(JointState, "/joint_states", lambda m: state.__setitem__("joints", m), 10)
    if imu_topic:
        node.create_subscription(Imu, imu_topic, lambda m: state.__setitem__("imu", m), qos_profile_sensor_data)
    if scan_topic:
        node.create_subscription(LaserScan, scan_topic, lambda m: state.__setitem__("scan", m), qos_profile_sensor_data)
    commands = node.create_publisher(JointState, "/terra/joint_commands", QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

    def report():
        now = node.get_clock().now().nanoseconds * 1e-9
        parts = ["仿真时间 %.2f s" % now]
        j = state["joints"]
        if j is not None:
            parts.append("%d 个关节" % len(j.name))
            if args.wave:
                joint = args.wave.rsplit("/", 1)[0] + "/joint_" + args.wave.rsplit("/", 1)[-1]
                if joint in j.name:
                    parts.append("%s = %.3f" % (joint, j.position[list(j.name).index(joint)]))
        if state["imu"] is not None:
            a = state["imu"].linear_acceleration
            parts.append("IMU 比力 (%.2f, %.2f, %.2f)" % (a.x, a.y, a.z))
        if state["scan"] is not None:
            hits = [r for r in state["scan"].ranges if math.isfinite(r)]
            parts.append("雷达 %d 束命中%s" % (len(hits), "，最近 %.2f m" % min(hits) if hits else ""))
        print("，".join(parts), flush=True)

    def wave():
        t = node.get_clock().now().nanoseconds * 1e-9
        target = args.center + args.amplitude * math.sin(2.0 * math.pi * t / args.period)
        commands.publish(JointState(name=[args.wave], position=[target]))

    node.create_timer(1.0, report)  # use_sim_time：按仿真时间计时，暂停时也停
    if args.wave:
        node.create_timer(1.0 / args.rate, wave)
        print("给 %s 发正弦目标：中心 %.3f，幅度 %.3f，周期 %.1f s（Ctrl+C 结束）" % (args.wave, args.center, args.amplitude, args.period))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        # Ctrl+C 或 SIGTERM：rclpy 的信号处理先关闭了上下文，spin 随后报错（ExternalShutdownException 或 RCLError）；
        # 这是正常结束。上下文仍有效时才是真正的错误
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
