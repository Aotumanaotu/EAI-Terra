#!/usr/bin/env python3
"""ROS2 桥接的端到端检查（M4-06a）：启动 terra_headless 或 terra_viewer（--ros2 通用配置），用 rclpy 订阅、调用服务并核对。

用法（由 ctest 调用；需要先 source /opt/ros/humble/setup.bash）：
  check_bridge.py headless --headless <terra_headless> --assets <assets 目录> --work <工作目录>
      sensor_suite 场景：话题与 QoS、/tf_static、各话题的时间戳落在各自频率的采样步上、IMU 与雷达的数值、墙钟节奏、
      /terra/pause、/terra/step、/terra/reset 服务
  check_bridge.py commands --headless <terra_headless> --assets <assets 目录> --work <工作目录>
      keyframe_arm 场景：/terra/joint_commands 按执行器类别取字段（位置 / 速度 / 力矩），越界、缺字段、未知名字的拒绝并计数
  check_bridge.py viewer --viewer <terra_viewer> --assets <assets 目录> --work <工作目录>
      查看器（Xvfb）打开 sensor_suite：相机的彩色、深度与内参，/clock 与服务

每个检查使用单独的 ROS_DOMAIN_ID，且只在本机通信（ROS_LOCALHOST_ONLY=1），不干扰用户正在运行的 ROS2 程序。
全部通过时打印 “all passed” 并返回 0；失败打印 “FAILED: …” 并返回 1。被测程序的输出写在工作目录中。
"""

import argparse
import math
import os
import subprocess
import sys
import time

# 话题与采样规则（与 terra::simulation::sensorSampleDue 相同：第 0 步总采样；floor(k·dt·f + 1e-9) 增加时采样）
DT = 0.002


def sample_due(step, dt, rate):
    if step == 0:
        return True
    if rate * dt >= 1.0 - 1e-9:
        return True
    return math.floor(step * dt * rate + 1e-9) > math.floor((step - 1) * dt * rate + 1e-9)


failures = []


def check(condition, what):
    if not condition:
        print("FAILED: " + what, flush=True)
        failures.append(what)
    return condition


def info(text):
    print("INFO " + text, flush=True)


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def stamp_step(stamp):
    return int(round(stamp_seconds(stamp) / DT))


def setup_environment():
    # 每个检查单独的域（101～200），只在本机通信
    os.environ["ROS_DOMAIN_ID"] = str(101 + os.getpid() % 100)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"


class Probe:
    """订阅桥接的话题，记录收到的消息（回调在 spin 时执行）。"""

    def __init__(self, node, imu_topic=None, scan_topic=None, camera=None):
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import JointState, Imu, LaserScan, Image, CameraInfo
        from tf2_msgs.msg import TFMessage

        self.node = node
        self.clock = []       # (墙钟, 仿真时间)
        self.joints = []
        self.tf = []
        self.imu = []
        self.scan = []
        self.rgb = []
        self.depth = []
        self.info = []
        best_effort = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        reliable = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        node.create_subscription(Clock, "/clock", lambda m: self.clock.append((time.monotonic(), stamp_seconds(m.clock))), best_effort)
        node.create_subscription(JointState, "/joint_states", self.joints.append, reliable)
        node.create_subscription(TFMessage, "/tf", self.tf.append, reliable)
        if imu_topic:
            node.create_subscription(Imu, imu_topic, self.imu.append, qos_profile_sensor_data)
        if scan_topic:
            node.create_subscription(LaserScan, scan_topic, self.scan.append, qos_profile_sensor_data)
        if camera:
            node.create_subscription(Image, camera + "/image_raw", self.rgb.append, qos_profile_sensor_data)
            node.create_subscription(Image, camera + "/depth/image_raw", self.depth.append, qos_profile_sensor_data)
            node.create_subscription(CameraInfo, camera + "/camera_info", self.info.append, qos_profile_sensor_data)

    def latest_clock(self):
        return self.clock[-1][1] if self.clock else None


def spin_for(executor, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.01)


def spin_until(executor, condition, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if condition():
            return True
        executor.spin_once(timeout_sec=0.01)
    return condition()


def call(executor, client, request, what, timeout=10.0):
    if not check(client.wait_for_service(timeout_sec=timeout), what + "：服务可用"):
        return None
    future = client.call_async(request)
    if not check(spin_until(executor, future.done, timeout), what + "：服务有应答"):
        return None
    response = future.result()
    check(response is not None and response.success, what + "：success = true（" + (response.message if response else "无应答") + "）")
    return response


def start(command, work, name):
    os.makedirs(work, exist_ok=True)
    out = open(os.path.join(work, name + ".stdout.txt"), "w")
    err = open(os.path.join(work, name + ".stderr.txt"), "w")
    info("启动：" + " ".join(command))
    process = subprocess.Popen(command, stdout=out, stderr=err, env=os.environ.copy())
    process.terra_logs = (os.path.join(work, name + ".stderr.txt"), os.path.join(work, name + ".stdout.txt"))
    return process, process.terra_logs[0], process.terra_logs[1]


def finish(process, timeout, what):
    try:
        code = process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()
        check(False, what + "：在 " + str(timeout) + " s 内没有退出")
        return None
    if not check(code == 0, what + "：退出码 " + str(code) + "（应为 0）"):
        # 被测程序异常退出（如 ASan / UBSan 报告）：打印它的输出末尾，便于定位
        for path in getattr(process, "terra_logs", ()):
            lines = read_text(path).splitlines()
            if lines:
                info("%s 的末尾 %d 行（%s）：" % (what, min(len(lines), 25), path))
                for line in lines[-25:]:
                    print("    " + line, flush=True)
    return code


def read_text(path):
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return f.read()
    except OSError:
        return ""


def check_qos(node, topic, reliable, transient_local, what):
    from rclpy.qos import ReliabilityPolicy, DurabilityPolicy
    infos = [i for i in node.get_publishers_info_by_topic(topic) if i.node_name == "terra_simulator"]
    if not check(len(infos) == 1, what + "：terra_simulator 发布 " + topic):
        return
    qos = infos[0].qos_profile
    want_r = ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT
    want_d = DurabilityPolicy.TRANSIENT_LOCAL if transient_local else DurabilityPolicy.VOLATILE
    check(qos.reliability == want_r and qos.durability == want_d,
          "%s：%s 的 QoS 为 %s / %s（实际 %s / %s）" % (what, topic, want_r.name, want_d.name, qos.reliability.name, qos.durability.name))


def check_grid(stamps, rate, what):
    """时间戳都落在该频率的采样步上；返回 (收到的条数, 时间跨度内应有的条数)。"""
    steps = sorted(set(stamp_step(s) for s in stamps))
    off = [k for k in steps if not sample_due(k, DT, rate)]
    check(not off, "%s：时间戳都在 %g Hz 的采样步上（不在的：%s）" % (what, rate, off[:5]))
    if len(steps) < 2:
        return len(steps), 0
    expected = sum(1 for k in range(steps[0], steps[-1] + 1) if sample_due(k, DT, rate))
    return len(steps), expected


def check_services(executor, node, probe, what):
    """暂停：/clock 停止；单步 5 次：正好前进 5 步；继续：恢复推进；重置：仿真时间回到起点附近。"""
    from std_srvs.srv import SetBool, Trigger
    pause = node.create_client(SetBool, "/terra/pause")
    step = node.create_client(Trigger, "/terra/step")
    reset = node.create_client(Trigger, "/terra/reset")

    call(executor, pause, SetBool.Request(data=True), what + " 暂停")
    spin_for(executor, 0.6)
    t1 = probe.latest_clock()
    spin_for(executor, 0.6)
    t2 = probe.latest_clock()
    check(t1 is not None and t1 == t2, "%s：暂停后 /clock 不再前进（%s → %s）" % (what, t1, t2))
    for _ in range(5):
        call(executor, step, Trigger.Request(), what + " 单步")
        spin_for(executor, 0.15)
    spin_for(executor, 0.3)
    t3 = probe.latest_clock()
    check(t2 is not None and t3 is not None and abs(t3 - (t2 + 5 * DT)) < 1e-6,
          "%s：暂停时单步 5 次，仿真时间正好前进 5 步（%s → %s）" % (what, t2, t3))
    call(executor, pause, SetBool.Request(data=False), what + " 继续")
    spin_for(executor, 1.0)
    t4 = probe.latest_clock()
    check(t3 is not None and t4 is not None and t4 > t3 + 0.3, "%s：继续后恢复推进（%s → %s）" % (what, t3, t4))
    before = len(probe.clock)
    call(executor, reset, Trigger.Request(), what + " 重置")
    spin_for(executor, 0.6)
    after = [t for _, t in probe.clock[before:]]
    check(bool(after) and min(after) < 0.3 and t4 is not None and t4 > 1.0,
          "%s：重置后仿真时间回到起点（重置前 %s，之后最小 %s）" % (what, t4, min(after) if after else None))


def check_static_tf(executor, node, expected, what):
    """/tf_static 为 transient local：桥接启动后才订阅也能收到。expected：{子坐标系: 父坐标系}。"""
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from tf2_msgs.msg import TFMessage
    received = []
    latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
    node.create_subscription(TFMessage, "/tf_static", received.append, latched)
    check(spin_until(executor, lambda: bool(received), 5.0), what + "：后订阅也收到 /tf_static")
    pairs = {}
    rotations = {}
    for m in received:
        for t in m.transforms:
            pairs[t.child_frame_id] = t.header.frame_id
            rotations[t.child_frame_id] = t.transform.rotation
    for child, parent in expected.items():
        check(pairs.get(child) == parent, "%s：/tf_static 中 %s 的父坐标系为 %s（实际 %s）" % (what, child, parent, pairs.get(child)))
    return pairs, rotations


# ---- 检查 1：Headless + sensor_suite ----

def run_headless(args):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    scene = os.path.join(args.assets, "tests/sensors/sensor_suite.terra-scene.json")
    profile = os.path.join(args.assets, "ros2/profiles/generic.terra-ros2.json")
    process, stderr_path, _ = start([args.headless, "--scene", scene, "--duration", "14", "--sample-every", "1000", "--ros2", profile],
                                    args.work, "headless")
    rclpy.init()
    node = rclpy.create_node("terra_check_bridge")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    probe = Probe(node, imu_topic="/imu/level_imu/imu", scan_topic="/room/scan/scan")
    try:
        if not check(spin_until(executor, lambda: len(probe.clock) > 0, 20.0), "headless：20 s 内收到 /clock"):
            return
        # 话题与 QoS（设计稿第 3 节）
        spin_for(executor, 0.5)
        check_qos(node, "/clock", False, False, "headless")
        check_qos(node, "/joint_states", True, False, "headless")
        check_qos(node, "/tf", True, False, "headless")
        check_qos(node, "/tf_static", True, True, "headless")
        check_qos(node, "/imu/level_imu/imu", False, False, "headless")
        check_qos(node, "/room/scan/scan", False, False, "headless")
        check_qos(node, "/cam/eye/image_raw", False, False, "headless")
        check_qos(node, "/cam/eye/camera_info", False, False, "headless")
        names = dict(node.get_topic_names_and_types())
        check(names.get("/terra/joint_commands") == ["sensor_msgs/msg/JointState"], "headless：订阅 /terra/joint_commands（JointState）")
        services = dict(node.get_service_names_and_types())
        check(services.get("/terra/reset") == ["std_srvs/srv/Trigger"] and services.get("/terra/pause") == ["std_srvs/srv/SetBool"] and
              services.get("/terra/step") == ["std_srvs/srv/Trigger"], "headless：服务 /terra/reset、/terra/pause、/terra/step")

        _, rotations = check_static_tf(executor, node, {"imu/level_imu": "imu/level", "room/scan": "room/base", "cam/eye": "cam/rig",
                                                        "cam/eye_optical": "cam/eye"}, "headless")
        q = rotations.get("cam/eye_optical")
        check(q is not None and abs(q.x - 1.0) < 1e-12 and abs(q.w) < 1e-12, "headless：光学坐标系为相机坐标系绕 X 轴转 180°")

        # 3 秒窗口：频率、墙钟节奏
        for lst in (probe.clock, probe.joints, probe.tf, probe.imu, probe.scan):
            lst.clear()
        spin_for(executor, 3.0)
        if len(probe.clock) >= 2:
            (w0, s0), (w1, s1) = probe.clock[0], probe.clock[-1]
            factor = (s1 - s0) / (w1 - w0) if w1 > w0 else 0.0
            info("headless：墙钟节奏 仿真 / 墙钟 = %.3f（%d 条 /clock）" % (factor, len(probe.clock)))
            check(0.7 < factor < 1.3, "headless：按墙钟节奏推进（仿真 / 墙钟 = %.3f）" % factor)
        got, want = check_grid([m.header.stamp for m in probe.joints], 100.0, "headless /joint_states")
        info("headless：/joint_states %d / %d 条" % (got, want))
        check(want > 200 and got >= 0.95 * want, "headless：/joint_states 100 Hz 可靠传输，几乎不丢（%d / %d）" % (got, want))
        got, want = check_grid([m.transforms[0].header.stamp for m in probe.tf if m.transforms], 100.0, "headless /tf")
        check(want > 200 and got >= 0.95 * want, "headless：/tf 100 Hz（%d / %d）" % (got, want))
        got, want = check_grid([m.header.stamp for m in probe.imu], 500.0, "headless IMU")
        info("headless：IMU %d / %d 条（best effort）" % (got, want))
        check(want > 1000 and got >= 0.5 * want, "headless：IMU 500 Hz（%d / %d）" % (got, want))
        got, want = check_grid([m.header.stamp for m in probe.scan], 10.0, "headless 雷达")
        check(got >= 20, "headless：雷达 10 Hz，3 s 收到 %d 次" % got)

        # 数值
        if check(bool(probe.joints), "headless：收到 /joint_states"):
            j = probe.joints[-1]
            check("imu/spin" in j.name and len(j.position) == len(j.name) == len(j.velocity) == len(j.effort),
                  "headless：/joint_states 含 imu/spin，各字段等长（%s）" % list(j.name))
        if check(bool(probe.tf) and bool(probe.tf[-1].transforms), "headless：收到 /tf"):
            frames = {t.child_frame_id: t.header.frame_id for t in probe.tf[-1].transforms}
            check(frames.get("imu/level") == "world" and frames.get("room/base") == "world", "headless：/tf 中实体的父坐标系为 world")
        if check(bool(probe.imu), "headless：收到 IMU"):
            m = probe.imu[-1]
            a = m.linear_acceleration
            check(m.header.frame_id == "imu/level_imu" and abs(a.x) < 1e-6 and abs(a.y) < 1e-6 and abs(a.z - 9.81) < 1e-6,
                  "headless：水平 IMU 的比力为 (0, 0, 9.81)（实际 %.6f %.6f %.6f，坐标系 %s）" % (a.x, a.y, a.z, m.header.frame_id))
            o = m.orientation
            check(abs(o.w * o.w + o.x * o.x + o.y * o.y + o.z * o.z - 1.0) < 1e-9, "headless：IMU 姿态为单位四元数")
        if check(bool(probe.scan), "headless：收到雷达"):
            s = probe.scan[-1]
            hits = sum(1 for r in s.ranges if math.isfinite(r))
            info("headless：雷达 %d 束，命中 %d 束" % (len(s.ranges), hits))
            check(len(s.ranges) == 360 and hits == 275, "headless：雷达 360 束、命中 275 束（与 M4-03b 相同；实际 %d / %d）" % (len(s.ranges), hits))
            check(abs(s.angle_min - math.radians(-180)) < 1e-5 and abs(s.angle_increment - math.radians(1)) < 1e-6 and
                  abs(s.angle_max - math.radians(179)) < 1e-4 and s.header.frame_id == "room/scan",
                  "headless：雷达角度范围 −180°～179°、步长 1°，坐标系 room/scan")
            check(abs(s.range_min - 0.15) < 1e-6 and abs(s.range_max - 12.0) < 1e-6 and abs(s.scan_time - 0.1) < 1e-6, "headless：量程与扫描周期")
        check(not probe.rgb, "headless：没有渲染，不发布相机图像")

        check_services(executor, node, probe, "headless")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        finish(process, 60, "headless")
    stderr = read_text(stderr_path)
    check("ROS2 桥接已开启" in stderr and "ROS2 桥接：发布 " in stderr, "headless：标准错误中有开启与统计信息")


# ---- 检查 2：Headless + keyframe_arm，关节命令 ----

def run_commands(args):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState

    scene = os.path.join(args.assets, "tests/scene/keyframe_arm.terra-scene.json")
    profile = os.path.join(args.assets, "ros2/profiles/generic.terra-ros2.json")
    process, stderr_path, _ = start([args.headless, "--scene", scene, "--duration", "9", "--sample-every", "1000", "--ros2", profile],
                                    args.work, "commands")
    rclpy.init()
    node = rclpy.create_node("terra_check_commands")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    probe = Probe(node)
    publisher = node.create_publisher(JointState, "/terra/joint_commands", QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

    def joint(name):
        if not probe.joints:
            return None
        j = probe.joints[-1]
        if name not in j.name:
            return None
        i = list(j.name).index(name)
        return j.position[i], j.velocity[i], j.effort[i]

    try:
        if not check(spin_until(executor, lambda: len(probe.joints) > 0 and publisher.get_subscription_count() > 0, 20.0),
                     "commands：20 s 内收到 /joint_states，且桥接订阅了 /terra/joint_commands"):
            return
        names = list(probe.joints[-1].name)
        check(names == ["arm/shoulder", "arm/extend", "arm/spin"], "commands：/joint_states 只含转动与滑动关节（%s）" % names)
        start_pos = joint("arm/shoulder")
        info("commands：开始时 arm/shoulder = %s" % (start_pos,))
        # 一条消息、三个执行器：位置执行器取 position，速度执行器取 velocity，力矩执行器取 effort
        good = JointState(name=["arm/shoulder_pos", "arm/spin_vel", "arm/extend_motor"],
                          position=[0.5, 0.0, 0.0], velocity=[0.0, 2.0, 0.0], effort=[0.0, 0.0, 0.5])
        publisher.publish(good)
        spin_for(executor, 3.0)
        # 拒绝：未知名字、越界、缺字段、非有限值（各 1 条）；目标不受影响
        publisher.publish(JointState(name=["arm/nope"], position=[0.1]))
        publisher.publish(JointState(name=["arm/shoulder_pos"], position=[5.0]))
        publisher.publish(JointState(name=["arm/spin_vel"], position=[1.0]))
        publisher.publish(JointState(name=["arm/shoulder_pos"], position=[float("nan")]))
        spin_for(executor, 1.5)
        shoulder = joint("arm/shoulder")
        spin = joint("arm/spin")
        extend_a = joint("arm/extend")
        spin_for(executor, 0.5)
        extend_b = joint("arm/extend")
        info("commands：arm/shoulder = %s，arm/spin = %s，arm/extend %s → %s" % (shoulder, spin, extend_a, extend_b))
        check(shoulder is not None and abs(shoulder[0] - 0.5) < 0.08, "commands：位置执行器跟到 position 0.5（关键帧的目标是 0.8）")
        check(spin is not None and abs(spin[1] - 2.0) < 0.3, "commands：速度执行器跟到 velocity 2.0")
        check(extend_a is not None and extend_b is not None and extend_b[0] > extend_a[0] + 0.05 and extend_b[2] > 0.0,
              "commands：力矩执行器取 effort 0.5，滑块向 +x 运动、力为正")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        finish(process, 60, "commands")
    stderr = read_text(stderr_path)
    info("commands：" + " ".join(l for l in stderr.splitlines() if "ROS2 桥接：" in l))
    check("命令接受 3 条、拒绝 4 条" in stderr, "commands：桥接统计为接受 3 条、拒绝 4 条")
    check("超出范围或不是有限值" in stderr, "commands：给出最近一次拒绝的原因")


# ---- 检查 3：查看器 + sensor_suite（Xvfb）----

def run_viewer(args):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    import struct

    scene = os.path.join(args.assets, "tests/sensors/sensor_suite.terra-scene.json")
    profile = os.path.join(args.assets, "ros2/profiles/generic.terra-ros2.json")
    os.makedirs(args.work, exist_ok=True)
    process, _, stdout_path = start([args.viewer, "--scene", scene, "--ros2", profile, "--seconds", "40",
                                     "--settings", os.path.join(args.work, "settings.json"),
                                     "--library", os.path.join(args.work, "library.sqlite3")], args.work, "viewer")
    rclpy.init()
    node = rclpy.create_node("terra_check_viewer")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    probe = Probe(node, imu_topic="/imu/level_imu/imu", camera="/cam/eye")
    try:
        if not check(spin_until(executor, lambda: len(probe.clock) > 0, 30.0), "viewer：30 s 内收到 /clock（命令行开启桥接后直接运行）"):
            return
        check_qos(node, "/cam/eye/image_raw", False, False, "viewer")
        check_qos(node, "/cam/eye/depth/image_raw", False, False, "viewer")
        check_static_tf(executor, node, {"cam/eye_optical": "cam/eye"}, "viewer")
        check(spin_until(executor, lambda: len(probe.rgb) >= 5 and len(probe.depth) >= 5 and len(probe.info) >= 5, 15.0),
              "viewer：收到相机的彩色、深度与内参（各 %d、%d、%d 帧）" % (len(probe.rgb), len(probe.depth), len(probe.info)))
        if probe.rgb and probe.depth and probe.info:
            rgb, depth, ci = probe.rgb[-1], probe.depth[-1], probe.info[-1]
            check(rgb.encoding == "rgb8" and rgb.width == 160 and rgb.height == 120 and rgb.step == 480 and len(rgb.data) == 160 * 120 * 3,
                  "viewer：彩色图 rgb8、160 × 120")
            check(depth.encoding == "32FC1" and depth.width == 160 and depth.height == 120 and depth.step == 640 and
                  len(depth.data) == 160 * 120 * 4 and depth.is_bigendian == 0, "viewer：深度图 32FC1、160 × 120")
            check(rgb.header.frame_id == "cam/eye_optical" and depth.header.frame_id == "cam/eye_optical" and ci.header.frame_id == "cam/eye_optical",
                  "viewer：图像与内参的坐标系为 cam/eye_optical")
            k = list(ci.k)
            check(ci.width == 160 and ci.height == 120 and k[0] > 0 and abs(k[0] - k[4]) < 1e-9 and abs(k[2] - 79.5) < 1e-9 and
                  abs(k[5] - 59.5) < 1e-9 and k[8] == 1.0 and ci.distortion_model == "plumb_bob",
                  "viewer：内参 fx = fy、cx = 79.5、cy = 59.5（实际 %s）" % k)
            # 中心像素（行 59、60，列 79、80 的平均）的深度为 2 m（M4-03a 的相机夹具）
            data = bytes(depth.data)
            values = [struct.unpack_from("<f", data, (r * 160 + c) * 4)[0] for r in (59, 60) for c in (79, 80)]
            center = sum(values) / len(values)
            info("viewer：中心深度 %.4f m" % center)
            check(abs(center - 2.0) < 0.01, "viewer：中心深度 2 m（实际 %.4f）" % center)
            check(any(b != 0 for b in bytes(rgb.data)), "viewer：彩色图不是全黑")
        # 相机的时间戳在 30 Hz 的采样步上；彩色、深度、内参的时间戳一一对应
        check_grid([m.header.stamp for m in probe.rgb], 30.0, "viewer 相机")
        stamps = lambda lst: set(stamp_step(m.header.stamp) for m in lst)
        common = stamps(probe.rgb) & stamps(probe.depth) & stamps(probe.info)
        check(len(common) >= 3, "viewer：彩色、深度、内参来自同一步（共有 %d 个时间戳）" % len(common))
        if probe.imu:
            a = probe.imu[-1].linear_acceleration
            check(abs(a.z - 9.81) < 1e-6, "viewer：IMU 与 Headless 相同（比力 z = %.6f）" % a.z)
        check_services(executor, node, probe, "viewer")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        finish(process, 60, "viewer")
    stdout = read_text(stdout_path)
    check("ROS2 桥接已开启" in stdout and "ROS2 桥接：发布 " in stdout, "viewer：输出中有开启与统计信息")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mode", choices=["headless", "commands", "viewer"])
    parser.add_argument("--headless")
    parser.add_argument("--viewer")
    parser.add_argument("--assets", required=True)
    parser.add_argument("--work", required=True)
    args = parser.parse_args()
    setup_environment()
    try:
        import rclpy  # noqa: F401
    except ImportError as e:
        print("FAILED: 无法导入 rclpy（%s）；请先 source /opt/ros/humble/setup.bash" % e)
        return 1
    info("ROS_DOMAIN_ID=%s，ROS_LOCALHOST_ONLY=1，RMW=%s" % (os.environ["ROS_DOMAIN_ID"], os.environ.get("RMW_IMPLEMENTATION", "默认")))
    if args.mode == "headless":
        run_headless(args)
    elif args.mode == "commands":
        run_commands(args)
    else:
        run_viewer(args)
    if failures:
        print("%d 项检查失败" % len(failures))
        return 1
    print("all passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
