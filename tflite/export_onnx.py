# -*- coding: utf-8 -*-
"""把训练权重导成 ONNX —— 参数化版，替代写死路径的 `pytorch2onnx/pytorch2onnx_simplify.py`。

那个老脚本只能导「一个」模型（默认 voc-290，输出名也写死成
`pytorch2onnx/yolo-fastestv2-opt.onnx`），而 v1.0 三级链路要导三个。
**不要**用这个脚本去覆盖 `pytorch2onnx/yolo-fastestv2-opt.onnx` ——
那是当前固件里那个模型的来源，已经备份在 `pytorch2onnx/_bak_current/`。

两类模型：
  detector  ① 整图找车牌 / ② 裁剪找字 —— FastestV2 Detector，输入 [1,3,S,S]，
              导出时 `export_onnx=True`，sigmoid/softmax 烘进图里，输出两个 NHWC [1,h,w,85]
  charhead  ③ 逐字分类 —— `train_char_head.SmallCharNet`，输入 [1,3,64,32]，
              输出 [1,70] logits（argmax 就是字）

用法：
  python tflite/export_onnx.py --kind detector \
      --data training/data/voc.data --weights training/weights/voc-299-epoch-0.270855ap-model.pth \
      --output tflite/onnx/plate.onnx

  python tflite/export_onnx.py --kind detector \
      --data training/data_crop/voc.data \
      --weights training/weights/voc_crop_ft-119-epoch-0.589619ap-model.pth \
      --output tflite/onnx/crop.onnx

  python tflite/export_onnx.py --kind charhead \
      --weights training/weights/charhead_16_32_64_32_64.pt \
      --output tflite/onnx/charhead.onnx
"""

import sys
sys.dont_write_bytecode = True

import os
import argparse

HERE = os.path.dirname(os.path.abspath(__file__))       # tflite/
ROOT = os.path.dirname(HERE)                            # 工程根目录
TRAIN = os.path.join(ROOT, "training")
if TRAIN not in sys.path:
    sys.path.insert(0, TRAIN)

import torch            # noqa: E402
import numpy as np      # noqa: E402

import model.detector as MD          # noqa: E402
import utils.utils as U              # noqa: E402
# train_char_head 在 import 时会把 cwd 切到 training/（它自己的 reload 逻辑），
# 所以下面所有路径都在此之前就解析成绝对路径了。
from train_char_head import SmallCharNet         # noqa: E402

os.chdir(ROOT)


def simplify(src, dst):
    """onnxsim 过一遍：常量折叠 + 形态推断。固件那条链一直用它，保持一致。"""
    import onnx
    from onnxsim import simplify as _simp
    m = onnx.load(src)
    onnx.checker.check_model(m)
    ms, ok = _simp(m)
    if not ok:
        raise RuntimeError("onnxsim 校验失败，保留原图 %s" % src)
    onnx.save(ms, dst)
    print("  onnxsim: %.2f MB -> %.2f MB" % (os.path.getsize(src) / 1e6,
                                             os.path.getsize(dst) / 1e6))


def export_detector(a, dev):
    cfg = U.load_datafile(a.data)
    S = int(cfg["height"])
    net = MD.Detector(int(cfg["classes"]), int(cfg["anchor_num"]), True, True).to(dev)
    net.load_state_dict(torch.load(a.weights, map_location = dev))
    net.eval()
    print("  detector  classes=%d anchor_num=%d  S=%d  权重=%s"
          % (cfg["classes"], cfg["anchor_num"], S, os.path.basename(a.weights)))

    raw = os.path.join(os.path.dirname(a.output), "_raw_" + os.path.basename(a.output))
    with torch.no_grad():
        torch.onnx.export(net, torch.rand(1, 3, S, S).to(dev), raw,
                          export_params = True, opset_version = 18,
                          do_constant_folding = True,
                          input_names = ["x"])
    print("  导出: %s (%.2f MB)" % (raw, os.path.getsize(raw) / 1e6))
    simplify(raw, a.output)
    return net, raw


def export_charhead(a, dev):
    ck = torch.load(a.weights, map_location = dev, weights_only = False)
    P_W, P_H = (int(v) for v in ck["size_wh"])
    classes = int(ck["classes"])
    net = SmallCharNet(classes, (P_H, P_W), *ck["c"]).to(dev)
    net.load_state_dict(ck["state_dict"])
    net.eval()
    # ck["val_acc"] 存的**已经是百分数**（train_char_head 写的是 100.0*(pv==yva).mean()），
    # 别再乘 100 —— 之前这里打出过 9888.73%。
    print("  charhead  classes=%d  c=%s  输入 1x3x%dx%d  val=%.2f%%"
          % (classes, list(ck["c"]), P_H, P_W, float(ck.get("val_acc", 0))))

    raw = os.path.join(os.path.dirname(a.output), "_raw_" + os.path.basename(a.output))
    with torch.no_grad():
        torch.onnx.export(net, torch.rand(1, 3, P_H, P_W).to(dev), raw,
                          export_params = True, opset_version = 18,
                          do_constant_folding = True,
                          input_names = ["x"])
    print("  导出: %s (%.2f MB)" % (raw, os.path.getsize(raw) / 1e6))
    simplify(raw, a.output)
    return net, raw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kind", required = True, choices = ["detector", "charhead"])
    ap.add_argument("--data", default = os.path.join(TRAIN, "data", "voc.data"),
                    help = "只有 detector 用得到")
    ap.add_argument("--weights", required = True)
    ap.add_argument("--output", required = True)
    ap.add_argument("--keep-raw", action = "store_true",
                    help = "保留 onnxsim 之前的中间文件，便于对比")
    args = ap.parse_args()

    args.weights = os.path.abspath(args.weights)
    args.output = os.path.abspath(args.output)
    args.data = os.path.abspath(args.data)
    os.makedirs(os.path.dirname(args.output), exist_ok = True)

    dev = torch.device("cpu")
    print("== 导出 %s -> %s" % (args.kind, os.path.relpath(args.output, ROOT)))
    net, raw = (export_detector if args.kind == "detector" else export_charhead)(args, dev)
    if not args.keep_raw:
        os.remove(raw)

    # 自检：ONNX 和 PyTorch 在同一输入上必须数值一致。
    # 不做这一步的话，导出错了要到量化、甚至到板子上才看得出来 —— 而那时候
    # 「框变差」是静默的，分不清是导出的问题还是量化的。
    import onnxruntime as ort
    cfg = U.load_datafile(args.data) if args.kind == "detector" else None
    if cfg is not None:
        S = int(cfg["height"])
        shape = (1, 3, S, S)
    else:
        ck = torch.load(args.weights, map_location = dev, weights_only = False)
        P_W, P_H = (int(v) for v in ck["size_wh"])
        shape = (1, 3, P_H, P_W)

    x = np.random.rand(*shape).astype(np.float32)
    so = ort.InferenceSession(args.output, providers = ["CPUExecutionProvider"])
    got = so.run(None, {so.get_inputs()[0].name: x})
    with torch.no_grad():
        ref = net(torch.from_numpy(x))
    ref = [ref] if torch.is_tensor(ref) else list(ref)
    print("  自检 ONNX vs PyTorch: 输入 %s -> 输出 %s" % (list(x.shape),
                                                         [list(g.shape) for g in got]))
    if args.kind == "charhead":
        rd = np.abs(got[0] - ref[0].numpy()).max()
        same = int((got[0].argmax(1) == ref[0].numpy().argmax(1)).all())
        print("    最大绝对差 %.3e   argmax 一致 %s" % (rd, "是" if same else "**否**"))
        if rd > 1e-3:
            raise SystemExit("!! ONNX 与 PyTorch 不一致，导出有问题")
    else:
        # `export_onnx=True` 的 forward 已经把输出 permute 成 NHWC 了。
        # 按**形状**配对，不按下标 —— 固件那边也是按形状找 [1,20,20,85] / [1,10,10,85] 的。
        bad = 0
        for ri in ref:
            ra = ri.numpy()
            cand = [g for g in got if g.shape == ra.shape]
            if not cand:
                raise SystemExit("!! ONNX 里找不到形状 %s 的输出" % (ra.shape,))
            rd = np.abs(cand[0] - ra).max()
            print("    %s  NHWC  最大绝对差 %.3e" % (list(ra.shape), rd))
            bad += rd > 1e-3
        if bad:
            raise SystemExit("!! ONNX 与 PyTorch 不一致，导出有问题")
    print("  ONNX %.2f MB   %s" % (os.path.getsize(args.output) / 1e6, args.output))


if __name__ == "__main__":
    raise SystemExit(main())
