# -*- coding: utf-8 -*-
"""算力/存储预算：一级检测器 vs 各档第二级识别头。

动机：报告建议「加第二级识别头」，但第二级要跑在 N32H78x 上。
参数量只回答了**存储**（flash），没回答**算力**（每帧 MACs）。
老板要判断「装不装得下」，两个都得有数。

不装 thop 之类的依赖 —— 挂 forward hook 自己数卷积/全连接的乘加，
够准确而且不用改环境。
"""

import sys
sys.dont_write_bytecode = True

import os
import numpy as np
import torch
import torch.nn as nn

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
os.chdir(HERE)

import utils.utils as U            # noqa: E402
import model.detector as MD        # noqa: E402
from train_char_head import SmallCharNet     # noqa: E402


def count(model, shape, dev = torch.device("cpu")):
    """返回 (参数量, MAC 数)。只数 Conv2d 和 Linear —— 它们是绝对大头。"""
    macs = [0]
    hs = []

    def hook(m, i, o):
        if isinstance(m, nn.Conv2d):
            # 输出元素数 x (Cin/groups * k*k)
            macs[0] += o.numel() * (m.in_channels // m.groups) * m.kernel_size[0] * m.kernel_size[1]
        elif isinstance(m, nn.Linear):
            macs[0] += o.numel() * m.in_features

    for m in model.modules():
        if isinstance(m, (nn.Conv2d, nn.Linear)):
            hs.append(m.register_forward_hook(hook))
    model.eval()
    with torch.no_grad():
        model(torch.zeros(1, *shape, device = dev))
    for h in hs:
        h.remove()
    npar = sum(p.numel() for p in model.parameters())
    return npar, macs[0]


def main():
    dev = torch.device("cpu")
    rows = []

    cfg = U.load_datafile("data_crop/voc.data")
    det = MD.Detector(int(cfg["classes"]), int(cfg["anchor_num"]), True).to(dev)
    ck = "weights/voc_crop-90-epoch-0.477014ap-model.pth"
    det.load_state_dict(torch.load(ck, map_location = dev))
    # 用 model.parameters()（可训练参数）而不是 state_dict 的求和 —— 后者把 BN 的
    # running_mean/var/num_batches_tracked 也算进去了，会多出两千多个，
    # 和「检测器 248,570 参数」这个一直用的数对不上。load_state_dict 通过了就说明
    # 现搭的结构和训练时一致，直接用它的参数量。
    dpar, dmac = count(det, (3, 320, 320), dev)
    rows.append(("一级检测器 (320x320)", dpar, dmac))

    # 第二级候选：SmallCharNet 的三档，输入就是裁剪后重采样到 32x64 的字符条
    for c in [(8, 16, 32), (16, 32, 64), (32, 64, 128)]:
        net = SmallCharNet(int(cfg["classes"]), (64, 32), *c).to(dev)
        npar, mac = count(net, (3, 64, 32), dev)
        rows.append(("第二级 SmallCharNet %d:%d:%d @32x64" % c, npar, mac))

    # 探针（今晚量到位置0=96.69% 的那个）—— 它是参数上限的参照，不是候选
    try:
        import probe_char_cls as P
        pr = P.CharCNN(int(cfg["classes"]), (64, 32)).to(dev)
        ppar, pmac = count(pr, (3, 64, 32), dev)
        rows.append(("探针 CharCNN @32x64（参照）", ppar, pmac))
    except Exception as e:
        print("  探针没数上：%s" % e)

    print("=" * 88)
    print("### 算力/存储预算  （MACs 只数 Conv2d + Linear，ReLU/BN/池化忽略）")
    print("=" * 88)
    print("%-42s %10s %12s %12s" % ("模型", "参数", "MACs", "MFLOPs/帧"))
    print("-" * 88)
    for name, npar, mac in rows:
        print("%-42s %10d %12d %12.2f" % (name, npar, mac, 2.0 * mac / 1e6))
    print("-" * 88)
    base = rows[0][2]
    print("  以一级为 1.0 的相对算力：")
    for name, npar, mac in rows[1:]:
        print("    %-40s x%.3f  （参数 x%.3f）" % (name, mac / base, npar / rows[0][1]))
    print("\n  注：MFLOPs 按「1 MAC = 2 FLOPs」算。N32H78x 的每周期 MAC 数决定实际帧率，"
          "\n      这里只给相对量级 —— 第二级相对一级有多重。")


if __name__ == "__main__":
    main()
