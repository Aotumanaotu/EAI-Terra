#!/usr/bin/env python3
# 生成策略测试的 ONNX 夹具（M4-10a）。不依赖任何包：直接写 protobuf 的线格式。IR 8，opset 18。
#   gemm_elu.onnx          输入 obs [batch, 4]（float32）→ Gemm（W 2×4，transB = 1，b 2）→ Elu（alpha = 1）→ 输出 act [batch, 2]
#                          W = [[0.5, -1, 0.25, 2], [1, 1, -1, 0.5]]，b = [0.1, -0.2]；元数据 format = terra_fixture_v1
#   bench_supermimic.onnx  supermimic_moe_velocity_policy_v1 格式的元数据（与用户的 Go2 策略相同的写法，值为 JSON 编码），
#                          机器人为 policy_bench（关节 j1、j2），每帧 17 维 × 2 帧；网络只有一个 Gemm：
#                          输出 0 = 最新一帧的重力方向 y（下标 17 + 4），输出 1 = 0
# 用法：python3 -I scripts/policy/make_fixture_onnx.py <输出目录>
import hashlib
import json
import struct
import sys

def varint(v):
    out = bytearray()
    while True:
        b = v & 0x7F
        v >>= 7
        if v:
            out.append(b | 0x80)
        else:
            out.append(b)
            return bytes(out)

def key(field, wire):
    return varint((field << 3) | wire)

def vint(field, v):
    return key(field, 0) + varint(v)

def blob(field, data):
    if isinstance(data, str):
        data = data.encode("utf-8")
    return key(field, 2) + varint(len(data)) + data

def tensor(name, dims, values):
    return b"".join(vint(1, d) for d in dims) + vint(2, 1) + blob(8, name) + blob(9, struct.pack("<%df" % len(values), *values))

def value_info(name, dims):
    shape = b""
    for d in dims:
        shape += blob(1, blob(2, d) if isinstance(d, str) else vint(1, d))
    tensor_type = vint(1, 1) + blob(2, shape)
    return blob(1, name) + blob(2, blob(1, tensor_type))

def node(inputs, outputs, op, name, attributes=b""):
    return b"".join(blob(1, i) for i in inputs) + b"".join(blob(2, o) for o in outputs) + blob(3, name) + blob(4, op) + attributes

def model_bytes(graph, metadata):
    out = vint(1, 8) + blob(2, "terra-fixture") + blob(3, "1") + blob(7, graph) + blob(8, blob(1, "") + vint(2, 18))
    for k, v in metadata:
        out += blob(14, blob(1, k) + blob(2, v))
    return out


trans_b = blob(5, blob(1, "transB") + vint(3, 1) + vint(20, 2))  # AttributeProto：name、i、type = INT

W = [0.5, -1.0, 0.25, 2.0, 1.0, 1.0, -1.0, 0.5]
B = [0.1, -0.2]
graph = (blob(1, node(["obs", "W", "B"], ["hidden"], "Gemm", "gemm", trans_b)) +
         blob(1, node(["hidden"], ["act"], "Elu", "elu")) +
         blob(2, "terra_fixture") +
         blob(5, tensor("W", [2, 4], W)) + blob(5, tensor("B", [2], B)) +
         blob(11, value_info("obs", ["batch", 4])) +
         blob(12, value_info("act", ["batch", 2])))
out_dir = sys.argv[1]
with open(out_dir + "/gemm_elu.onnx", "wb") as f:
    f.write(model_bytes(graph, [("format", "terra_fixture_v1")]))

W2 = [0.0] * (2 * 34)
W2[21] = 1.0
graph2 = (blob(1, node(["obs_history", "W", "B"], ["actions"], "Gemm", "gemm", trans_b)) +
          blob(2, "bench_supermimic") +
          blob(5, tensor("W", [2, 34], W2)) + blob(5, tensor("B", [2], [0.0, 0.0])) +
          blob(11, value_info("obs_history", ["batch", 34])) +
          blob(12, value_info("actions", ["batch", 2])))
j = lambda v: json.dumps(v)
meta = [
    ("format", j("supermimic_moe_velocity_policy_v1")),
    ("spec_hash", j(hashlib.sha256(b"bench_supermimic").hexdigest())),  # 64 位十六进制，与训练检查点对应（M4-10b）
    ("iteration", "1"),
    ("input_name", j("obs_history")),
    ("output_name", j("actions")),
    ("observation_dim", "17"),
    ("history_length", "2"),
    ("history_order", j("frame_major_oldest_first")),
    ("history_reset", j("repeat_initial_frame")),
    ("observation_layout", j([{"name": "base_angular_velocity_body", "width": 3}, {"name": "projected_gravity_body", "width": 3},
                              {"name": "joint_position_offset_with_encoder_bias", "width": 2}, {"name": "joint_velocity", "width": 2},
                              {"name": "previous_unscaled_action", "width": 2}, {"name": "velocity_command_body_xy_world_yaw", "width": 3},
                              {"name": "gait_phase_sin_cos", "width": 2}])),
    ("joint_names", j(["j1", "j2"])),
    ("action_joint_names", j(["j1", "j2"])),
    ("default_joint_positions", j([0.1, -0.1])),
    ("action_scales", j([0.5, 0.5])),
    ("control_dt_seconds", "0.01"),
    ("gait_phase", j({"period_seconds": 0.04, "standing_command_threshold": 0.1})),
    ("action_semantics", j("unscaled_joint_position_offsets")),
]
with open(out_dir + "/bench_supermimic.onnx", "wb") as f:
    f.write(model_bytes(graph2, meta))
