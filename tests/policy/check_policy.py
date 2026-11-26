#!/usr/bin/env python3
"""Headless 的策略运行（M4-10a，headless.policy）。夹具 assets/tests/policy/（线性模型，不需要 ONNX Runtime）。

用法（由 ctest 调用）：
  check_policy.py --headless <terra_headless> --assets <assets 目录> --work <目录> [--library <装有 ONNX Runtime 的资源库>]
                  [--go2 <用户的 Go2 policy.onnx>]
检查：
  - --policy 说明文件（用其中的推荐场景）：两次输出逐字节相同；config 行写明 policy 与 command；stderr 写明后端与降频；
  - --command 改变运动（与默认指令的输出不同）；--no-noise 改变运动（观测取带噪声的读数）；
  - 实验记录：头部有 policy（路径、sha256、后端）；复现逐位相同（不需要推理库）；
  - .onnx 加 --scene 自动配置：组件不在时退出码 3，并给出 --install-component 与手动安装的方法；
  - --list-components：列出 onnxruntime-cpu 与状态；--install-component 不存在的组件：退出码 2；
  - --import-policy（M4-10b）：ONNX 导入到 policies/ 并登记；多出关节的警告；不认识的格式生成草稿；训练检查点按 spec_hash 与
    迭代对应；有 PyTorch 时 TorchScript 转为 ONNX；
  - 给了 --library（ONNX Runtime 已安装）：.onnx 自动配置后运行，两次逐字节相同；给了 --go2：用户的策略在
    go2_policy 场景中运行 2 s 不出错，记录与复现逐位相同。
测试不读写用户目录：资源库都在 --work 中（TERRA_LIBRARY 也指向那里）。
"""

import argparse
import json
import os
import subprocess
import sys

failures = 0


def check(ok, what):
    global failures
    print(("PASS " if ok else "FAILED: ") + what)
    if not ok:
        failures += 1


def run(cmd, env=None):
    p = subprocess.run(cmd, capture_output=True, text=True, env=env)
    return p


def show(p, cmd):
    print("命令（退出码 %d）：%s\n%s\n%s" % (p.returncode, " ".join(cmd), p.stdout[-1500:], p.stderr[-2500:]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", required=True)
    ap.add_argument("--assets", required=True)
    ap.add_argument("--work", required=True)
    ap.add_argument("--library", default="")
    ap.add_argument("--go2", default="")
    args = ap.parse_args()
    os.makedirs(args.work, exist_ok=True)
    env = dict(os.environ, TERRA_LIBRARY=os.path.join(args.work, "default_library.db"))
    empty_library = os.path.join(args.work, "empty", "library.db")
    os.makedirs(os.path.dirname(empty_library), exist_ok=True)
    policy_dir = os.path.join(args.assets, "tests", "policy")
    bench = os.path.join(policy_dir, "bench.terra-policy.json")
    hl = args.headless

    # 说明文件 + 推荐场景
    base = [hl, "--policy", bench, "--steps", "300", "--sample-every", "10", "--library", empty_library]
    a = run(base + ["--command", "0.3,0,0"], env)
    b = run(base + ["--command", "0.3,0,0"], env)
    if a.returncode != 0:
        show(a, base)
    check(a.returncode == 0 and a.stdout == b.stdout, "两次运行逐字节相同（%d 行）" % len(a.stdout.splitlines()))
    config = json.loads(a.stdout.splitlines()[0]) if a.returncode == 0 else {}
    check(config.get("policy") == bench and config.get("command") == [0.3, 0, 0], "config 行写明 policy 与 command")
    check("linear（2 × 34）" in a.stderr and "每 5 步推理一次" in a.stderr, "stderr 写明后端与降频")
    c = run(base, env)
    check(c.returncode == 0 and c.stdout.splitlines()[2:] != a.stdout.splitlines()[2:], "--command 改变运动")
    # --no-noise：夹具线性模型的输出只取重力方向（IMU 姿态没有噪声模型）与指令，噪声影响不到它。
    # 这里另写一个输出含最新一帧角速度 x（下标 17）的线性模型，噪声开关才会改变运动
    with open(os.path.join(policy_dir, "linear_policy.json"), encoding="utf-8") as fh:
        lin = json.load(fh)
    lin["weights"][1][17] = 1.0
    gyro_model = os.path.join(args.work, "linear_gyro.json")
    with open(gyro_model, "w", encoding="utf-8") as fh:
        json.dump(lin, fh)
    with open(bench, encoding="utf-8") as fh:
        pdoc = json.load(fh)
    pdoc["model"]["file"] = gyro_model
    pdoc["scene"] = os.path.join(policy_dir, "policy_bench.terra-scene.json")
    gyro_policy = os.path.join(args.work, "gyro.terra-policy.json")
    with open(gyro_policy, "w", encoding="utf-8") as fh:
        json.dump(pdoc, fh, ensure_ascii=False)
    gbase = [hl, "--policy", gyro_policy, "--steps", "300", "--sample-every", "10", "--library", empty_library, "--command", "0.3,0,0"]
    gn, gt = run(gbase, env), run(gbase + ["--no-noise"], env)
    if gn.returncode != 0:
        show(gn, gbase)
    check(gn.returncode == 0 and gt.returncode == 0 and gn.stdout.splitlines()[2:] != gt.stdout.splitlines()[2:],
          "--no-noise 改变运动（观测取带噪声的角速度）")
    d = run(base + ["--command", "0.3,0,0", "--no-noise"], env)
    check(d.returncode == 0 and d.stdout.splitlines()[2:] == a.stdout.splitlines()[2:],
          "原夹具只看重力方向与指令：--no-noise 不改变运动（噪声只加在比力与角速度上）")

    # 记录与复现
    record = os.path.join(args.work, "bench.terra-record.jsonl")
    if os.path.exists(record):
        os.remove(record)
    r = run(base + ["--command", "0.3,0,0", "--record", record], env)
    header = json.loads(open(record, encoding="utf-8").readline()) if r.returncode == 0 else {}
    pol = header.get("policy", {})
    check(pol.get("path") == bench and len(pol.get("sha256", "")) == 64 and pol.get("backend", "").startswith("linear"),
          "记录头有 policy（路径、sha256、后端）")
    rr = run([hl, "--rerun", record], env)
    check(rr.returncode == 0 and '"bitwise":true' in rr.stdout, "复现逐位相同：" + rr.stdout.strip()[:200])

    # .onnx：组件不在
    onnx = os.path.join(policy_dir, "bench_supermimic.onnx")
    scene = os.path.join(policy_dir, "policy_bench.terra-scene.json")
    m = run([hl, "--scene", scene, "--policy", onnx, "--steps", "10", "--library", empty_library], env)
    check(m.returncode == 3 and "--install-component onnxruntime-cpu" in m.stderr and "从文件安装" in m.stderr,
          "组件不在：退出码 3，给出安装方法（退出码 %d）" % m.returncode)
    check("自动配置：" in m.stderr, "自动配置的说明写到 stderr")
    m2 = run([hl, "--policy", onnx, "--steps", "10"], env)
    check(m2.returncode == 2 and "--scene" in m2.stderr, ".onnx 没有 --scene：退出码 2")

    # 组件命令
    lc = run([hl, "--list-components", "--library", empty_library], env)
    comps = [json.loads(line) for line in lc.stdout.splitlines() if line.strip()]
    check(lc.returncode == 0 and any(x.get("id") == "onnxruntime-cpu" and x.get("state") == "missing" for x in comps),
          "--list-components：onnxruntime-cpu missing")
    ic = run([hl, "--install-component", "no-such-component", "--library", empty_library], env)
    check(ic.returncode == 2 and "onnxruntime-cpu" in ic.stderr, "安装不存在的组件：退出码 2，列出可用的组件")

    # M4-10b：--import-policy（不需要 ONNX Runtime）
    import_library = os.path.join(args.work, "import", "library.db")
    import shutil
    shutil.rmtree(os.path.dirname(import_library), ignore_errors=True)
    os.makedirs(os.path.dirname(import_library))
    ip = run([hl, "--import-policy", onnx, "--scene", scene, "--name", "bench walker", "--library", import_library], env)
    res = json.loads(ip.stdout.splitlines()[-1]) if ip.stdout.strip() else {}
    imported = res.get("policy", "")
    check(ip.returncode == 0 and res.get("ok") is True and res.get("source_kind") == "onnx" and os.path.isfile(imported) and
          imported.startswith(os.path.join(os.path.dirname(import_library), "policies", "bench_walker")),
          "--import-policy ONNX：导入到资源库目录下的 policies/（退出码 %d）" % ip.returncode)
    check(any("bench/j3" in w for w in res.get("warnings", [])), "导入结果列出机器人多出关节的警告")
    la = run([hl, "--list-assets", "--kind", "policy", "--library", import_library], env)
    assets = [json.loads(line) for line in la.stdout.splitlines() if '"type":"asset"' in line]
    check(any(a.get("path") == imported and a.get("title") == "bench walker" and scene in a.get("related", []) for a in assets),
          "资源库登记为 policy，关联推荐场景")
    bad = run([hl, "--import-policy", os.path.join(policy_dir, "gemm_elu.onnx"), "--scene", scene, "--library", import_library], env)
    bres = json.loads(bad.stdout.splitlines()[-1]) if bad.stdout.strip() else {}
    check(bad.returncode == 4 and bres.get("draft", "").endswith(".terra-policy-draft.json") and os.path.isfile(bres.get("draft", "")),
          "不认识的格式：退出码 4，生成说明文件草稿")
    # 训练检查点：zip 中的 data.pkl 带 spec_hash 与迭代（与 torch.save 的写法相同的片段）
    import struct
    import zipfile
    ck_dir = os.path.join(args.work, "checkpoint_run")
    shutil.rmtree(ck_dir, ignore_errors=True)
    os.makedirs(ck_dir)
    shutil.copy(onnx, os.path.join(ck_dir, "policy.onnx"))
    shutil.copy(os.path.join(policy_dir, "task_config.json"), ck_dir)
    spec = "e117647d1c05ee855f6b388cd5e686d052c5bd61a7e9512e128db93a32c26be4"

    def u(text):
        b = text.encode()
        return b"X" + struct.pack("<I", len(b)) + b

    for it in (1, 7):
        pkl = b"\x80\x02}q\x00(" + u("spec_hash") + b"q\x01" + u(spec) + b"q\x02" + u("iteration") + b"q\x03K" + bytes([it]) + b"u."
        with zipfile.ZipFile(os.path.join(ck_dir, "model_%d.pt" % it), "w") as z:
            z.writestr("model_%d/data.pkl" % it, pkl)
    ck = run([hl, "--import-policy", os.path.join(ck_dir, "model_1.pt"), "--scene", scene, "--library", import_library], env)
    cres = json.loads(ck.stdout.splitlines()[-1]) if ck.stdout.strip() else {}
    check(ck.returncode == 0 and cres.get("source_kind") == "checkpoint", "训练检查点：使用 spec_hash 与迭代都相同的 ONNX")
    ck7 = run([hl, "--import-policy", os.path.join(ck_dir, "model_7.pt"), "--scene", scene, "--library", import_library], env)
    check(ck7.returncode == 4 and "不能代用" in ck7.stderr, "迭代不同的检查点：拒绝并说明（退出码 %d）" % ck7.returncode)
    # TorchScript：有 PyTorch 时转换
    python = os.environ.get("TERRA_PYTHON", "python3")
    has_torch = subprocess.run([python, "-c", "import torch"], capture_output=True).returncode == 0
    if has_torch:
        ts = os.path.join(args.work, "scripted_mlp.pt")
        subprocess.run([python, "-c", "import torch,sys\n"
                        "m=torch.nn.Sequential(torch.nn.Linear(17,8),torch.nn.ELU(),torch.nn.Linear(8,2))\n"
                        "torch.jit.script(m).save(sys.argv[1])", ts], check=True)
        tr = run([hl, "--import-policy", ts, "--scene", scene, "--library", import_library], env)
        tres = json.loads(tr.stdout.splitlines()[-1]) if tr.stdout.strip() else {}
        model = tres.get("model", "")
        onnx_ops = os.path.join(os.path.dirname(args.assets), "scripts", "policy", "onnx_ops.py")
        info = subprocess.run([sys.executable, "-I", onnx_ops, model, "--json"], capture_output=True, text=True) if model else None
        dims = json.loads(info.stdout) if info and info.returncode == 0 else {}
        check(tres.get("source_kind") == "torchscript" and os.path.isfile(model) and
              dims.get("inputs", [{}])[0].get("dims") == ["batch", 17] and dims.get("outputs", [{}])[0].get("dims") == ["batch", 2],
              "TorchScript：用 PyTorch 转为 ONNX（输入 17、输出 2）")
        check(tr.returncode == 4 and tres.get("draft", "").endswith(".terra-policy-draft.json"), "TorchScript 没有元数据：生成说明文件草稿")
    else:
        print("SKIP TorchScript 转换：%s 中没有 PyTorch（可用 TERRA_PYTHON 指定）" % python)

    # 装有 ONNX Runtime 时
    if args.library and os.path.exists(os.path.join(os.path.dirname(args.library), "components", "onnxruntime-cpu", "installed.json")):
        o = [hl, "--scene", scene, "--policy", onnx, "--steps", "200", "--library", args.library, "--command", "0.4,0,0"]
        x, y = run(o, env), run(o, env)
        if x.returncode != 0:
            show(x, o)
        check(x.returncode == 0 and x.stdout == y.stdout and "ONNX Runtime 1.32" in x.stderr, "ONNX Runtime：自动配置后运行，两次逐字节相同")
        ri = run([hl, "--policy", imported, "--steps", "100", "--library", args.library], env)
        check(ri.returncode == 0, "导入的策略（说明文件）可以直接运行（退出码 %d）" % ri.returncode)
        if args.go2 and os.path.exists(args.go2):
            go2_scene = os.path.join(args.assets, "scenes", "robots", "go2_policy.terra-scene.json")
            rec = os.path.join(args.work, "go2.terra-record.jsonl")
            if os.path.exists(rec):
                os.remove(rec)
            g = [hl, "--scene", go2_scene, "--policy", args.go2, "--duration", "2", "--sample-every", "50", "--library", args.library,
                 "--command", "0.5,0,0", "--record", rec]
            gr = run(g, env)
            if gr.returncode != 0:
                show(gr, g)
            check(gr.returncode == 0 and "supermimic_moe_velocity_policy_v1" in gr.stderr, "用户的 Go2 策略：Headless 运行 2 s")
            grr = run([hl, "--rerun", rec], env)
            check(grr.returncode == 0 and '"bitwise":true' in grr.stdout, "用户的 Go2 策略：记录复现逐位相同（不需要推理库）")
        else:
            print("SKIP 用户的 Go2 策略：没有给出或不存在")
    else:
        print("SKIP ONNX Runtime 的 Headless 检查：没有安装组件（验证脚本会先安装）")

    print("all passed" if failures == 0 else "%d 项失败" % failures)
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
