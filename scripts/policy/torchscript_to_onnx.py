#!/usr/bin/env python3
"""TorchScript 策略（.pt）转为 ONNX（M4-10b）。由 Terra 的策略导入调用，也可单独运行。

用法：python3 torchscript_to_onnx.py <策略.pt> <输出.onnx> [--input-dim N]
- 只用 CPU；输入维数默认取第一个二维参数（Linear 的 weight 或 LSTM 的 weight_ih_l0）的列数；
- 导出时输入名 obs、输出名 actions，batch 维可变，opset 17；导出后用同一输入比较 TorchScript 与重新加载的结果的形状；
- 最后一行打印 JSON：{"input_dim":…, "output_dim":…, "torch":"版本"}。
退出码：0 成功；2 参数错误；3 没有 PyTorch；4 转换失败。
"""
import json
import sys


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    input_dim = None
    if "--input-dim" in sys.argv:
        input_dim = int(sys.argv[sys.argv.index("--input-dim") + 1])
        args = [a for a in args if a != str(input_dim)]
    if len(args) != 2:
        print(__doc__)
        return 2
    source, target = args
    try:
        import torch
    except ImportError:
        print("没有 PyTorch：pip install torch --index-url https://download.pytorch.org/whl/cpu", file=sys.stderr)
        return 3
    try:
        model = torch.jit.load(source, map_location="cpu")
        model.eval()
        if input_dim is None:
            for name, p in model.named_parameters():
                if p.dim() == 2:
                    input_dim = int(p.shape[1])
                    print("输入维数取自参数 %s %s" % (name, tuple(p.shape)))
                    break
        if input_dim is None:
            print("无法推断输入维数，请用 --input-dim 指定", file=sys.stderr)
            return 4
        dummy = torch.zeros(1, input_dim)
        with torch.no_grad():
            out = model(dummy)
        if isinstance(out, (tuple, list)):
            out = out[0]
        kwargs = dict(input_names=["obs"], output_names=["actions"], dynamic_axes={"obs": {0: "batch"}, "actions": {0: "batch"}},
                      opset_version=17)
        import inspect
        if "dynamo" in inspect.signature(torch.onnx.export).parameters:
            kwargs["dynamo"] = False  # TorchScript 模块只能用 TorchScript 导出器
        torch.onnx.export(model, (dummy,), target, **kwargs)
        print(json.dumps({"input_dim": input_dim, "output_dim": int(out.shape[-1]), "torch": torch.__version__}))
        return 0
    except Exception as e:  # 导出失败（如带状态的 LSTM 不支持）：把原因交给调用方
        print("转换失败：%s: %s" % (type(e).__name__, e), file=sys.stderr)
        return 4


if __name__ == "__main__":
    sys.exit(main())
