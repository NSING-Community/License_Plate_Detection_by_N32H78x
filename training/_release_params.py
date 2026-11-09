# -*- coding: utf-8 -*-
"""RELEASE_v1.0_2stage.md 的参数量清单 —— 可复现。

口径：一律用 model.parameters()（可训练参数），**不要**去 sum(state_dict.values())：
后者把 BN 的 running_mean/running_var/num_batches_tracked 也算进去，检测器会
多出 6,205 个（242,365 -> 248,570），字符头多出 454 个。见
memory/torchsummary-param-count-trap.md。
"""
import os, sys, torch
sys.dont_write_bytecode = True

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
os.chdir(HERE)

import utils.utils as U            # noqa: E402
import model.detector as MD        # noqa: E402
from train_char_head import SmallCharNet   # noqa: E402

DEV = torch.device("cpu")


def npar(m):
    return sum(p.numel() for p in m.parameters())


def main():
    cfg = U.load_datafile("data_crop/voc.data")
    nc, na = int(cfg["classes"]), int(cfg["anchor_num"])

    det = MD.Detector(nc, na, True).to(DEV)
    det.load_state_dict(torch.load("weights/voc-299-epoch-0.270855ap-model.pth",
                                   map_location = DEV))
    d = npar(det)

    cnet = MD.Detector(nc, na, True).to(DEV)
    cnet.load_state_dict(torch.load("weights/voc_crop_ft-119-epoch-0.589619ap-model.pth",
                                    map_location = DEV))
    c = npar(cnet)

    ck = torch.load("weights/charhead_16_32_64_32_64.pt", map_location = DEV,
                    weights_only = False)
    P_W, P_H = ck["size_wh"]
    head = SmallCharNet(int(ck["classes"]), (P_H, P_W), *ck["c"]).to(DEV)
    head.load_state_dict(ck["state_dict"])
    h = npar(head)

    print("=" * 78)
    print("### 部署参数量清单（可训练参数，model.parameters()）")
    print("=" * 78)
    print("  一级：整图找车牌      weights/voc-299-epoch-0.270855ap-model.pth   %9d" % d)
    print("  裁剪版检测器          weights/voc_crop_ft-119-*.pth               %9d" % c)
    print("  第二级字符头 %-9s weights/charhead_16_32_64_32_64.pt   %9d"
          % ("16:32:64", h))
    print("  " + "-" * 74)
    print("  合计                                                            %9d" % (d + c + h))
    print("  相对单个一级检测器  x%.3f" % ((d + c + h) / float(d)))
    print()
    print("  第二级输入尺寸 %dx%d  pad=%s  classes=%d  c=%s"
          % (P_W, P_H, ck["pad"], int(ck["classes"]), ck["c"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
