#!/usr/bin/env python3
"""宇树配置的端到端检查（M4-06b）：terra_headless --ros2 <宇树配置>，用 rclpy 与宇树官方程序核对。

用法（由 ctest 调用；需要先 source /opt/ros/humble/setup.bash 与 unitree_ros2 的 cyclonedds_ws/install/setup.bash）：
  check_unitree.py go2 --headless <terra_headless> --assets <assets> --work <目录>
      Go2（unitree_go）：话题、类型与 QoS；LowState 500 Hz、CRC、电机顺序与 /joint_states 一致、IMU；
      本脚本发 LowCmd（CRC 按官方算法）让 Go2 站起；CRC 不对与含 NaN 的命令被拒绝（保持上一条）；mode = 0 时没有力矩
  check_unitree.py hg --robot g1|as2 --headless ... --assets ... --work ...
      G1 / AS2（unitree_hg）：话题与 CRC；收到命令前力矩为 0；关节跟踪；mode_pr 回传；reserve[1..3] 不参与 CRC（与官方代码相同）
  check_unitree.py official --robot go2|go2w|g1 --unitree-examples <unitree_ros2/example> --headless ... --assets ... --work ...
      原样运行宇树官方示例（go2_stand_example / g1_low_level_example），核对机器人按示例的时间表动作，且没有命令被拒绝。
      找不到示例程序时返回 77（跳过），并提示编译命令。

全部通过时打印 “all passed” 并返回 0。
"""

import argparse
import math
import os
import signal
import struct
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_bridge as cb  # noqa: E402

check = cb.check
info = cb.info
SKIPPED = 77
LEGS = ["%s_%s_joint" % (leg, part) for leg in ("FR", "FL", "RR", "RL") for part in ("hip", "thigh", "calf")]
G1 = ["left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint", "left_ankle_pitch_joint",
      "left_ankle_roll_joint", "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint",
      "right_ankle_pitch_joint", "right_ankle_roll_joint", "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
      "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
      "left_wrist_pitch_joint", "left_wrist_yaw_joint", "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
      "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"]
ROBOTS = {
    # 名字：(场景, 配置, 实例, 电机表, 消息)
    "go2": ("scenes/robots/go2_lowlevel.terra-scene.json", "ros2/profiles/unitree_go2.terra-ros2.json", "go2", LEGS, "go"),
    "go2w": ("scenes/robots/go2w.terra-scene.json", "ros2/profiles/unitree_go2w.terra-ros2.json", "go2w",
             LEGS + ["FR_wheel_joint", "FL_wheel_joint", "RR_wheel_joint", "RL_wheel_joint"], "go"),
    "g1": ("scenes/robots/g1_lowlevel.terra-scene.json", "ros2/profiles/unitree_g1.terra-ros2.json", "g1", G1, "hg"),
    "as2": ("scenes/robots/as2.terra-scene.json", "ros2/profiles/unitree_as2.terra-ros2.json", "as2", LEGS, "hg"),
}
POS_STOP = 2.146e9
VEL_STOP = 16000.0
STAND = [0.0, 0.67, -1.3] * 4  # go2_stand_example 的 target_pos_2


# ---- 官方结构体的字节布局与 CRC（独立于 C++ 实现的 Python 版本）----

class Packer:
    def __init__(self):
        self.b = bytearray()

    def align(self, n):
        while len(self.b) % n:
            self.b.append(0)

    def put(self, fmt, value):
        size = struct.calcsize(fmt)
        self.align(size)
        self.b += struct.pack("<" + fmt, float(value) if fmt == "f" else int(value))

    def arr(self, fmt, values):
        for v in values:
            self.put(fmt, v)


def crc32_core(words):
    crc = 0xFFFFFFFF
    for data in words:
        bit = 1 << 31
        for _ in range(32):
            crc = ((crc << 1) & 0xFFFFFFFF) ^ 0x04C11DB7 if crc & 0x80000000 else (crc << 1) & 0xFFFFFFFF
            if data & bit:
                crc ^= 0x04C11DB7
            bit >>= 1
    return crc


def crc_of(p):
    p.align(4)
    words = struct.unpack("<%dI" % (len(p.b) // 4), bytes(p.b))
    return crc32_core(words[:-1])


def imu(p, s, temp_fmt):
    p.align(4)
    p.arr("f", s.quaternion)
    p.arr("f", s.gyroscope)
    p.arr("f", s.accelerometer)
    p.arr("f", s.rpy)
    p.put(temp_fmt, s.temperature)
    p.align(4)


def crc_go_lowcmd(m):
    p = Packer()
    p.arr("B", m.head); p.put("B", m.level_flag); p.put("B", m.frame_reserve)
    p.arr("I", m.sn); p.arr("I", m.version); p.put("H", m.bandwidth)
    for c in m.motor_cmd:
        p.align(4)
        p.put("B", c.mode)
        for v in (c.q, c.dq, c.tau, c.kp, c.kd):
            p.put("f", v)
        p.arr("I", c.reserve)
    p.put("B", m.bms_cmd.off); p.arr("B", m.bms_cmd.reserve)
    p.arr("B", m.wireless_remote); p.arr("B", m.led); p.arr("B", m.fan); p.put("B", m.gpio)
    p.put("I", m.reserve); p.put("I", m.crc)
    return crc_of(p)


def crc_go_lowstate(m):
    p = Packer()
    p.arr("B", m.head); p.put("B", m.level_flag); p.put("B", m.frame_reserve)
    p.arr("I", m.sn); p.arr("I", m.version); p.put("H", m.bandwidth)
    imu(p, m.imu_state, "b")
    for s in m.motor_state:
        p.align(4)
        p.put("B", s.mode)
        for v in (s.q, s.dq, s.ddq, s.tau_est, s.q_raw, s.dq_raw, s.ddq_raw):
            p.put("f", v)
        p.put("b", s.temperature); p.put("I", s.lost); p.arr("I", s.reserve)
        p.align(4)
    b = m.bms_state
    p.align(4)
    for v in (b.version_high, b.version_low, b.status, b.soc):
        p.put("B", v)
    p.put("i", b.current); p.put("H", b.cycle); p.arr("b", b.bq_ntc); p.arr("b", b.mcu_ntc); p.arr("H", b.cell_vol)
    p.align(4)
    p.arr("h", m.foot_force); p.arr("h", m.foot_force_est); p.put("I", m.tick); p.arr("B", m.wireless_remote)
    p.put("B", m.bit_flag); p.put("f", m.adc_reel); p.put("b", m.temperature_ntc1); p.put("b", m.temperature_ntc2)
    p.put("f", m.power_v); p.put("f", m.power_a); p.arr("H", m.fan_frequency); p.put("I", m.reserve); p.put("I", m.crc)
    return crc_of(p)


def crc_hg_lowcmd(m):
    p = Packer()
    p.put("B", m.mode_pr); p.put("B", m.mode_machine)
    for c in m.motor_cmd:
        p.align(4)
        p.put("B", c.mode)
        for v in (c.q, c.dq, c.tau, c.kp, c.kd):
            p.put("f", v)
        p.put("I", c.reserve)
    p.arr("I", [m.reserve[0], 0, 0, 0])  # 官方 get_crc 只拷贝 reserve[0]
    p.put("I", m.crc)
    return crc_of(p)


def crc_hg_lowstate(m):
    p = Packer()
    p.arr("I", m.version); p.put("B", m.mode_pr); p.put("B", m.mode_machine); p.put("I", m.tick)
    imu(p, m.imu_state, "h")
    for s in m.motor_state:
        p.align(4)
        p.put("B", s.mode)
        for v in (s.q, s.dq, s.ddq, s.tau_est):
            p.put("f", v)
        p.arr("h", s.temperature); p.put("f", s.vol); p.arr("I", s.sensor); p.put("I", s.motorstate); p.arr("I", s.reserve)
        p.align(4)
    p.arr("B", m.wireless_remote); p.arr("I", m.reserve); p.put("I", m.crc)
    return crc_of(p)


# ---- 公共部分 ----

class Session:
    """启动 terra_headless 与 rclpy 节点，记录 LowState、/joint_states 等。"""

    def __init__(self, args, robot, duration):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        scene, profile, self.instance, self.motors, self.messages = ROBOTS[robot]
        self.robot = robot
        if self.messages == "go":
            from unitree_go.msg import LowState, LowCmd, SportModeState
            self.SportModeState = SportModeState
        else:
            from unitree_hg.msg import LowState, LowCmd
        self.LowState, self.LowCmd = LowState, LowCmd
        self.process, self.stderr_path, _ = cb.start(
            [args.headless, "--scene", os.path.join(args.assets, scene), "--duration", str(duration), "--sample-every", "100000",
             "--ros2", os.path.join(args.assets, profile)], args.work, robot)
        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node("terra_check_unitree")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.probe = cb.Probe(self.node)
        reliable = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
        self.lowstate, self.lf, self.sport = [], [], []
        self.node.create_subscription(LowState, "/lowstate", lambda m: self.lowstate.append((time.monotonic(), m)), reliable)
        self.node.create_subscription(LowState, "/lf/lowstate", lambda m: self.lf.append((time.monotonic(), m)), reliable)
        if self.messages == "go":
            self.node.create_subscription(SportModeState, "/sportmodestate", lambda m: self.sport.append((time.monotonic(), m)), reliable)
        self.publisher = self.node.create_publisher(LowCmd, "/lowcmd", QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.crc_cmd = crc_go_lowcmd if self.messages == "go" else crc_hg_lowcmd
        self.crc_state = crc_go_lowstate if self.messages == "go" else crc_hg_lowstate

    def spin(self, seconds):
        cb.spin_for(self.executor, seconds)

    def wait_ready(self):
        ok = cb.spin_until(self.executor, lambda: bool(self.lowstate) and self.publisher.get_subscription_count() > 0, 25.0)
        return check(ok, "%s：25 s 内收到 /lowstate，且桥接订阅了 /lowcmd" % self.robot)

    def latest(self):
        return self.lowstate[-1][1] if self.lowstate else None

    def q(self, slot):
        m = self.latest()
        return None if m is None else m.motor_state[slot].q

    def body_height(self):
        return self.sport[-1][1].body_height if self.sport else None

    def command(self, targets, kp, kd, mode=1, bad_crc=False, tau=0.0, mode_pr=None, reserve=None):
        """targets：电机编号 → 目标位置；其余电机为 mode 1、q = PosStopF、kp = kd = 0（与官方示例相同）。"""
        m = self.LowCmd()
        if self.messages == "go":
            m.head = [0xFE, 0xEF]
            m.level_flag = 0xFF
        for i in range(len(m.motor_cmd)):
            c = m.motor_cmd[i]
            c.mode = mode
            c.q, c.dq, c.kp, c.kd, c.tau = POS_STOP, VEL_STOP, 0.0, 0.0, 0.0
        for slot, q in targets.items():
            c = m.motor_cmd[slot]
            c.q, c.dq = float(q), 0.0
            c.kp = float(kp[slot] if isinstance(kp, dict) else kp)
            c.kd = float(kd)
            c.tau = float(tau)
        if mode_pr is not None:
            m.mode_pr = mode_pr
        if reserve is not None:
            m.reserve = reserve
        m.crc = self.crc_cmd(m) ^ (1 if bad_crc else 0)
        self.publisher.publish(m)

    def hold(self, seconds, rate=250.0, **kw):
        """按 rate 连续发送同一条命令 seconds 秒（期间处理收到的消息）。"""
        end = time.monotonic() + seconds
        sent = 0
        while time.monotonic() < end:
            self.command(**kw)
            sent += 1
            self.spin(1.0 / rate)
        return sent

    def close(self, timeout=90):
        self.executor.shutdown()
        self.node.destroy_node()
        self.rclpy.shutdown()
        cb.finish(self.process, timeout, self.robot)
        return cb.read_text(self.stderr_path)


def stats_of(stderr):
    line = next((l for l in stderr.splitlines() if "ROS2 桥接：" in l), "")
    info("桥接统计：" + line)
    import re
    m = re.search(r"命令接受 (\d+) 条、拒绝 (\d+) 条", line)
    return (int(m.group(1)), int(m.group(2)), line) if m else (None, None, line)


def check_topics(s, what):
    names = dict(s.node.get_topic_names_and_types())
    pkg = "unitree_go" if s.messages == "go" else "unitree_hg"
    expected = {"/lowstate": pkg + "/msg/LowState", "/lf/lowstate": pkg + "/msg/LowState", "/lowcmd": pkg + "/msg/LowCmd"}
    if s.messages == "go":
        expected.update({"/sportmodestate": "unitree_go/msg/SportModeState", "/lf/sportmodestate": "unitree_go/msg/SportModeState",
                         "/wirelesscontroller": "unitree_go/msg/WirelessController"})
    for topic, kind in expected.items():
        check(names.get(topic) == [kind], "%s：话题 %s 的类型为 %s（实际 %s）" % (what, topic, kind, names.get(topic)))
    for topic in ("/lowstate", "/lf/lowstate"):
        cb.check_qos(s.node, topic, True, False, what)
    if s.messages == "hg":
        check("/sportmodestate" not in names, "%s：hg 没有 /sportmodestate" % what)
    check("/terra/joint_commands" not in names, "%s：宇树配置不订阅 /terra/joint_commands" % what)
    end = time.monotonic() + 5.0  # 节点名可能晚于端点被发现（见 check_bridge.check_qos）
    while True:
        subs = [i for i in s.node.get_subscriptions_info_by_topic("/lowcmd") if i.node_name == "terra_simulator"]
        if subs or time.monotonic() > end:
            break
        time.sleep(0.05)
    check(len(subs) == 1, "%s：terra_simulator 订阅 /lowcmd" % what)


def check_lowstate(s, what):
    """2 s 窗口：tick 每 2 ms 一条且几乎不丢；/lf/lowstate 在 20 ms 的整数倍；CRC；电机顺序与 /joint_states 一致；IMU。"""
    s.lowstate.clear(); s.lf.clear(); s.probe.joints.clear()
    s.spin(2.0)
    ticks = sorted(set(m.tick for _, m in s.lowstate))
    if check(len(ticks) > 10, "%s：收到 LowState" % what):
        gaps = set(b - a for a, b in zip(ticks, ticks[1:]))
        expected = (ticks[-1] - ticks[0]) // 2 + 1
        info("%s：LowState %d / %d 条，tick 间隔 %s ms" % (what, len(ticks), expected, sorted(gaps)[:5]))
        check(len(ticks) >= 0.95 * expected and min(gaps) == 2, "%s：/lowstate 500 Hz（tick 每 2 ms），几乎不丢" % what)
    lf_ticks = [m.tick for _, m in s.lf]
    check(len(lf_ticks) >= 50 and all(t % 20 == 0 for t in lf_ticks), "%s：/lf/lowstate 50 Hz（tick 为 20 ms 的整数倍，%d 条）" % (what, len(lf_ticks)))
    bad = [m.tick for _, m in s.lowstate[-50:] if s.crc_state(m) != m.crc]
    check(not bad, "%s：LowState 的 CRC 与官方算法一致（不一致的 tick：%s）" % (what, bad[:5]))
    # 电机顺序：同一时刻的 /joint_states
    by_ms = {}
    for js in s.probe.joints:
        by_ms[int(round(cb.stamp_seconds(js.header.stamp) * 1000))] = dict(zip(js.name, js.position))
    compared = 0
    worst = 0.0
    for _, m in s.lowstate:
        if m.tick in by_ms:
            js = by_ms[m.tick]
            for slot, joint in enumerate(s.motors):
                worst = max(worst, abs(m.motor_state[slot].q - js[s.instance + "/" + joint]))
            compared += 1
    info("%s：与 /joint_states 对照 %d 个时刻，最大差 %.2e rad" % (what, compared, worst))
    check(compared >= 10 and worst < 1e-5, "%s：motor_state[i].q 与 /joint_states 中电机表第 i 个关节一致（电机顺序正确）" % what)
    m = s.latest()
    if m is not None:
        q = list(m.imu_state.quaternion)
        norm = math.sqrt(sum(v * v for v in q))
        w, x, y, z = q
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        check(abs(norm - 1) < 1e-5 and abs(roll - m.imu_state.rpy[0]) < 1e-4 and abs(yaw - m.imu_state.rpy[2]) < 1e-4,
              "%s：IMU 四元数为单位四元数（w、x、y、z），rpy 与四元数一致" % what)
        acc = math.sqrt(sum(v * v for v in m.imu_state.accelerometer))
        check(5.0 < acc < 15.0, "%s：IMU 比力的大小 %.2f m/s²（静止时约 9.81）" % (what, acc))


# ---- go2 ----

def run_go2(args):
    s = Session(args, "go2", 20)
    bad = 0
    try:
        if not s.wait_ready():
            return
        check_topics(s, "go2")
        check_lowstate(s, "go2")
        m = s.latest()
        check(all(abs(m.motor_state[i].tau_est) < 1e-6 for i in range(12)), "go2：收到 LowCmd 之前各电机力矩为 0（场景的 PD 层让位）")
        check(list(m.head) == [0xFE, 0xEF] and m.level_flag == 0xFF, "go2：LowState 的 head 与 level_flag")
        lying = s.body_height()
        info("go2：没有命令时机身高度 %.3f m" % lying)
        check(lying is not None and lying < 0.2, "go2：没有命令时机器人趴下（%.3f m）" % (lying or -1))
        # 站起：1 s 内从当前姿态插值到 STAND，再保持 2 s
        start = [s.q(i) for i in range(12)]
        steps = 250
        for k in range(steps):
            r = (k + 1) / steps
            s.command(targets={i: (1 - r) * start[i] + r * STAND[i] for i in range(12)}, kp=60.0, kd=5.0)
            s.spin(0.004)
        s.hold(2.0, targets={i: STAND[i] for i in range(12)}, kp=60.0, kd=5.0)
        standing = s.body_height()
        m = s.latest()
        err = max(abs(m.motor_state[i].q - STAND[i]) for i in range(12))
        feet = list(m.foot_force)
        info("go2：站起后机身高度 %.3f m，关节最大误差 %.3f rad，足端力 %s N" % (standing, err, feet))
        check(standing > 0.25 and err < 0.1, "go2：LowCmd（kp 60、kd 5）让 Go2 站起，关节跟到目标")
        check(all(f > 10 for f in feet), "go2：站立时四个足端都有接触力")
        check(abs(m.motor_state[1].tau_est) > 0.5, "go2：站立时大腿电机的 tau_est 不为 0")
        # CRC 不对（kp = 0 的命令若被接受，机器人会趴下）：拒绝，保持上一条
        bad = s.hold(0.5, rate=200.0, targets={i: STAND[i] for i in range(12)}, kp=0.0, kd=0.0, bad_crc=True)
        s.command(targets={i: STAND[i] for i in range(12)}, kp=60.0, kd=5.0, tau=float("nan"))  # 含 NaN：拒绝
        s.spin(1.0)
        after = s.body_height()
        check(after > 0.25, "go2：CRC 不对的 %d 条与含 NaN 的 1 条命令被拒绝，机器人保持站立（%.3f m）" % (bad, after))
        # mode = 0：没有力矩，机器人趴下
        s.hold(1.5, targets={i: STAND[i] for i in range(12)}, kp=60.0, kd=5.0, mode=0)
        limp = s.body_height()
        m = s.latest()
        check(limp < standing - 0.08 and all(abs(m.motor_state[i].tau_est) < 1e-6 for i in range(12)),
              "go2：mode = 0 时力矩为 0，机器人趴下（%.3f m）" % limp)
    finally:
        stderr = s.close()
    accepted, rejected, line = stats_of(stderr)
    check(rejected is not None and bad - 5 <= rejected <= bad + 1 and "非有限值" in line,
          "go2：拒绝 %s 条（发送 %d 条 CRC 不对的与 1 条含 NaN 的；/lowcmd 为 best effort，允许少量丢失），最近一次的原因为非有限值" % (rejected, bad))
    check(accepted is not None and accepted > 500, "go2：接受的 LowCmd 条数 %s" % accepted)


# ---- hg：G1、AS2 ----

def run_hg(args):
    robot = args.robot
    s = Session(args, robot, 20)
    bad = 0
    tolerance = 0.1 if robot == "g1" else 0.15  # AS2 趴在地上站起，接触使误差稍大
    try:
        if not s.wait_ready():
            return
        check_topics(s, robot)
        check_lowstate(s, robot)
        m = s.latest()
        n = len(s.motors)
        check(all(abs(m.motor_state[i].tau_est) < 1e-6 for i in range(n)), "%s：收到 LowCmd 之前各电机力矩为 0" % robot)
        check(m.mode_machine == 0 and m.mode_pr == 0, "%s：mode_machine 为配置的 0，mode_pr 初值 0" % robot)
        if robot == "g1":
            target = {i: 0.0 for i in range(n)}
            target[3] = 0.6  # 左膝
            kp = {i: 100.0 if i < 13 else 50.0 for i in range(n)}
            probe_slot, probe_joint = 3, "左膝"
        else:
            target = {i: [0.0, 0.8, -1.5][i % 3] for i in range(n)}
            kp = 80.0
            probe_slot, probe_joint = 1, "右前大腿"
        s.hold(2.5, targets=target, kp=kp, kd=2.0, mode_pr=1)
        m = s.latest()
        errors = [abs(m.motor_state[i].q - target[i]) for i in range(n)]
        worst = max(range(n), key=lambda i: errors[i])
        info("%s：关节最大误差 %.3f rad（电机 %d %s）；%s %.3f（目标 %.2f）" %
             (robot, errors[worst], worst, s.motors[worst], probe_joint, m.motor_state[probe_slot].q, target[probe_slot]))
        # 力矩律：静止时 tau_est = kp (q̂ − q) − kd · dq（tau = 0；上一步的状态与本步只差一步，误差很小）
        kps = [kp[i] if isinstance(kp, dict) else kp for i in range(n)]
        law = max(abs(m.motor_state[i].tau_est - (kps[i] * (target[i] - m.motor_state[i].q) - 2.0 * m.motor_state[i].dq)) /
                  (0.05 + 0.02 * abs(m.motor_state[i].tau_est)) for i in range(n))
        check(law < 1.0, "%s：各电机的 tau_est 符合 τ = kp (q̂ − q) + kd (dq̂ − dq)（最大偏差为允许值的 %.2f 倍）" % (robot, law))
        if robot == "g1":
            # 悬挂的 G1：手臂（kp 50）承受自重，稳态误差 = 重力矩 / kp，约 0.1～0.2 rad；腿与腰（kp 100）更小
            legs = max(errors[:13])
            check(legs < tolerance and max(errors) < 0.25 and errors[probe_slot] < 0.05,
                  "g1：LowCmd 让各关节跟到目标（腿与腰最大误差 %.3f rad < 0.1，全部 %.3f < 0.25，左膝 %.3f < 0.05）" %
                  (legs, max(errors), errors[probe_slot]))
        else:
            check(errors[worst] < tolerance, "%s：LowCmd 让各关节跟到目标（最大误差 %.3f rad）" % (robot, errors[worst]))
        check(m.mode_pr == 1, "%s：LowState 回传最近一条 LowCmd 的 mode_pr" % robot)
        # reserve[1..3] 不参与 CRC：官方算法算出的 CRC 被接受（目标改变，关节跟随）
        moved = dict(target)
        moved[probe_slot] = target[probe_slot] - 0.3
        s.hold(1.5, targets=moved, kp=kp, kd=2.0, mode_pr=1, reserve=[0, 123, 456, 789])
        q1 = s.q(probe_slot)
        check(abs(q1 - moved[probe_slot]) < 0.08, "%s：reserve[1..3] 不为 0 的命令按官方 CRC 被接受（%s 到 %.3f）" % (robot, probe_joint, q1))
        # CRC 不对：拒绝，保持上一条
        wrong = dict(moved)
        wrong[probe_slot] = target[probe_slot]
        bad = s.hold(0.5, rate=200.0, targets=wrong, kp=kp, kd=2.0, bad_crc=True)
        s.spin(0.5)
        q2 = s.q(probe_slot)
        check(abs(q2 - moved[probe_slot]) < 0.08, "%s：CRC 不对的 %d 条命令被拒绝，关节保持（%.3f）" % (robot, bad, q2))
    finally:
        stderr = s.close()
    accepted, rejected, line = stats_of(stderr)
    check(rejected is not None and bad - 5 <= rejected <= bad and "CRC 不对" in line,
          "%s：拒绝 %s 条（发送 %d 条 CRC 不对的），原因为 CRC 不对" % (robot, rejected, bad))


# ---- 官方示例原样运行 ----

def find_example(root, name):
    for base, _, files in os.walk(root):
        if name in files:
            path = os.path.join(base, name)
            if os.access(path, os.X_OK) and os.path.isfile(path):
                return path
    return None


def run_official(args):
    robot = args.robot
    name = "g1_low_level_example" if robot == "g1" else "go2_stand_example"
    exe = find_example(args.unitree_examples, name) if args.unitree_examples and os.path.isdir(args.unitree_examples) else None
    if exe is None:
        print("SKIP：没有找到宇树官方示例 %s（在 %s 中查找）。请编译：\n"
              "  cd ~/unitree_ros2/example && source /opt/ros/humble/setup.bash && "
              "source ../cyclonedds_ws/install/setup.bash && colcon build" % (name, args.unitree_examples))
        return SKIPPED
    s = Session(args, robot, 25)
    example = None
    samples = []  # (相对示例启动的秒数, 机身高度, 电机位置)
    try:
        if not s.wait_ready():
            return 1
        s.spin(1.5)  # 没有命令时机器人趴下 / 悬挂
        before = s.body_height()
        info("%s：原样运行官方示例 %s" % (robot, exe))
        out = open(os.path.join(args.work, robot + ".example.txt"), "w")
        example = subprocess.Popen([exe], stdout=out, stderr=subprocess.STDOUT, env=os.environ.copy())
        t0 = time.monotonic()
        while time.monotonic() - t0 < 9.0:
            s.spin(0.02)
            m = s.latest()
            samples.append((time.monotonic() - t0, s.body_height(), [m.motor_state[i].q for i in range(len(s.motors))]))
        check(example.poll() is None, "%s：官方示例运行中（没有提前退出）" % robot)
    finally:
        if example is not None and example.poll() is None:
            example.send_signal(signal.SIGINT)
            try:
                example.wait(timeout=10)
            except subprocess.TimeoutExpired:
                example.kill()
        stderr = s.close()
    accepted, rejected, line = stats_of(stderr)
    check(accepted is not None and accepted > 1000 and rejected == 0,
          "%s：官方示例的 LowCmd 全部被接受（接受 %s、拒绝 %s 条）：CRC 与官方算法一致" % (robot, accepted, rejected))
    if robot == "g1":
        # 0～3 s：各关节插值到 0；之后脚踝以 1 Hz 摆动（俯仰 ±0.25、横滚 ±0.25 rad），手腕横滚 ±0.5 rad
        window = [q for t, _, q in samples if 4.0 <= t <= 8.0]
        if check(len(window) > 50, "g1：示例运行期间收到 LowState"):
            span = lambda slot: max(q[slot] for q in window) - min(q[slot] for q in window)
            info("g1：4～8 s 左踝俯仰幅度 %.3f、横滚 %.3f、左手腕横滚 %.3f rad；左膝 %.3f rad" %
                 (span(4), span(5), span(19), window[-1][3]))
            check(span(4) > 0.3 and span(5) > 0.3 and span(10) > 0.3, "g1：两侧脚踝按示例摆动（峰峰值 > 0.3 rad，示例为 0.5）")
            check(span(19) > 0.6, "g1：手腕横滚按示例摆动（峰峰值 > 0.6 rad，示例为 1.0）")
            check(abs(window[-1][3]) < 0.15 and abs(window[-1][0]) < 0.15, "g1：其余关节保持在 0 附近（左膝、左髋俯仰）")
        return 0
    # go2 / go2w：1 s 后开始；1～2 s 趴下（target_pos_1），2～3 s 站起（target_pos_2），3～5 s 保持，5～6.8 s 趴下（target_pos_3）
    stand = [(t, h, q) for t, h, q in samples if 3.3 <= t <= 4.8 and h is not None]
    lie = [(t, h, q) for t, h, q in samples if 7.3 <= t <= 9.0 and h is not None]
    if check(stand and lie, "%s：示例运行期间收到 sportmodestate 与 LowState" % robot):
        stand_h = max(h for _, h, _ in stand)
        lie_h = min(h for _, h, _ in lie)
        q = stand[-1][2]
        err = max(abs(q[i] - STAND[i]) for i in range(12))
        info("%s：示例前 %.3f m，站立 %.3f m（关节最大误差 %.3f rad），最后趴下 %.3f m" % (robot, before, stand_h, err, lie_h))
        check(stand_h > before + 0.08 and stand_h > lie_h + 0.08, "%s：按示例由趴下到站起，再趴下" % robot)
        check(err < 0.15, "%s：站立时 12 个腿部电机跟到示例的 target_pos_2（电机顺序与真机一致）" % robot)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mode", choices=["go2", "hg", "official"])
    parser.add_argument("--robot", choices=sorted(ROBOTS), default="go2")
    parser.add_argument("--headless", required=True)
    parser.add_argument("--assets", required=True)
    parser.add_argument("--work", required=True)
    parser.add_argument("--unitree-examples")
    args = parser.parse_args()
    cb.setup_environment()
    os.makedirs(args.work, exist_ok=True)
    try:
        import rclpy  # noqa: F401
        import unitree_go.msg  # noqa: F401
        import unitree_hg.msg  # noqa: F401
    except ImportError as e:
        print("FAILED: 无法导入 rclpy 或宇树消息（%s）；请先 source /opt/ros/humble/setup.bash 与 cyclonedds_ws/install/setup.bash" % e)
        return 1
    info("ROS_DOMAIN_ID=%s，ROS_LOCALHOST_ONLY=1，RMW=%s" % (os.environ["ROS_DOMAIN_ID"], os.environ.get("RMW_IMPLEMENTATION", "默认")))
    if args.mode == "go2":
        run_go2(args)
    elif args.mode == "hg":
        if args.robot not in ("g1", "as2"):
            print("FAILED: hg 只用于 --robot g1 或 as2")
            return 1
        run_hg(args)
    else:
        if run_official(args) == SKIPPED:
            return SKIPPED
    if cb.failures:
        print("%d 项检查失败" % len(cb.failures))
        return 1
    print("all passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
