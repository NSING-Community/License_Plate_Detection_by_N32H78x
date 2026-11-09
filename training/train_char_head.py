# -*- coding: utf-8 -*-
"""第二级识别头要多大才够？—— 参数预算扫描。

前情（都是今晚实测的）：
  · 探针 `probe_charcls_32x64`（**1,715,750** 参数）在**真值框**上逐位置准确率
    位置 0 = 96.69%，其余 99.03~100%（`_probe_by_pos.py`）。信息是够的。
  · 同一个像素、同一个框，**检测头**（整个检测器才 242,365 个权重）在位置 0 只有 28.79%
    （`_diag_obj_vs_cls.py`）。所以省份字不是数据不够，是**分类容量**不够。
  · 省份类在训练集里位置 0 的中位样本量只有 111（`_count_cls.py`），
    但探针在 ≤30 个验证样本的类上仍有 96.1% —— 数据够用。

那就只剩一个现实问题：MCU 装不下 1.7M 参数的识别头（是检测器的 6.9 倍）。
这个脚本把「分类头容量 → 逐位置准确率」这条曲线量出来，
让「第二级值不值得上、要多少参数」有个数字，而不是拍脑袋。

做法：从原生分辨率裁剪图里抠真值字符框（外扩 pad，和探针同一套切法），
训练若干档参数的 CNN，只看 val 逐位置准确率。**框是给真的**，所以量的是纯分类。

用法：
  python train_char_head.py --epochs 12
  python train_char_head.py --sweep 8,16,32 --epochs 12     # 只跑指定宽度
"""

import sys
sys.dont_write_bytecode = True

import os
import time
import argparse
import numpy as np
import cv2

import torch
import torch.nn as nn

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
os.chdir(HERE)

import utils.utils as U        # noqa: E402
import model.detector as MD    # noqa: E402

CACHE_DIR = "cache"


def collect(paths):
    """返回 [(img_path, cls, cx, cy, w, h)]，只收 class>0 的字符框，按 x 排序。"""
    out = []
    for p in paths:
        lp = p.split(".")[0] + ".txt"
        if not os.path.exists(lp):
            continue
        rows = [ln.split() for ln in open(lp)]
        g = [r for r in rows if len(r) == 5 and float(r[0]) > 0]
        g.sort(key = lambda z: float(z[1]))
        for k, r in enumerate(g):
            out.append((p, int(float(r[0])), float(r[1]), float(r[2]),
                        float(r[3]), float(r[4]), k))
    return out


def cache_patches(cfg, split, size_wh, pad, refresh = False):
    os.makedirs(CACHE_DIR, exist_ok = True)
    tag = "%s_%dx%d_pad%02d" % (split, size_wh[0], size_wh[1], int(pad * 100))
    fp = os.path.join(CACHE_DIR, "charpatch_%s.npz" % tag)
    if os.path.exists(fp) and not refresh:
        z = np.load(fp)
        return z["X"], z["y"], z["pos"]
    paths = [ln.strip() for ln in open(cfg[split]) if ln.strip()]
    items = collect(paths)
    t0 = time.time()
    X, y, pos = [], [], []
    W_out, H_out = size_wh
    cur, img = None, None
    for ip, cls, cx, cy, bw, bh, k in items:
        if ip != cur:
            img = cv2.imread(ip)
            cur = ip
        if img is None:
            continue
        H, W = img.shape[:2]
        x0, y0 = (cx - bw / 2) * W, (cy - bh / 2) * H
        x1, y1 = (cx + bw / 2) * W, (cy + bh / 2) * H
        mx, my = (x1 - x0) * pad, (y1 - y0) * pad
        X0 = int(np.clip(np.floor(x0 - mx), 0, W - 1))
        Y0 = int(np.clip(np.floor(y0 - my), 0, H - 1))
        X1 = int(np.clip(np.ceil(x1 + mx), X0 + 1, W))
        Y1 = int(np.clip(np.ceil(y1 + my), Y0 + 1, H))
        pt = img[Y0:Y1, X0:X1]
        if pt.size == 0:
            continue
        X.append(cv2.resize(pt, (W_out, H_out), interpolation = cv2.INTER_LINEAR))
        y.append(cls)
        pos.append(k)
    X = np.asarray(X, np.uint8)
    y = np.asarray(y, np.int64)
    pos = np.asarray(pos, np.int16)
    np.savez_compressed(fp, X = X, y = y, pos = pos)
    print("  抠图 %s：%d 张（%.0fs）-> %s" % (split, len(y), time.time() - t0, fp))
    return X, y, pos


class SmallCharNet(nn.Module):
    """小号字符头：3 段 (conv-conv-pool)，全局平均池化，一层 FC。

    全局池化是刻意的 —— 探针相对检测头的优势就来自「对整字做空间聚合」
    （1x1 conv 没有这一步）。BN 在部署时可以折进卷积，不占参数。
    """

    def __init__(self, classes, size_wh, c1, c2, c3):
        super(SmallCharNet, self).__init__()

        def stage(i, o):
            return [nn.Conv2d(i, o, 3, padding = 1, bias = False), nn.BatchNorm2d(o),
                    nn.ReLU(inplace = True),
                    nn.Conv2d(o, o, 3, padding = 1, bias = False), nn.BatchNorm2d(o),
                    nn.ReLU(inplace = True), nn.MaxPool2d(2)]

        self.f = nn.Sequential(*stage(3, c1), *stage(c1, c2), *stage(c2, c3))
        self.fc = nn.Linear(c3, classes)

    def forward(self, x):
        f = self.f(x)
        return self.fc(f.mean(dim = (2, 3)))          # 全局平均池化


def n_params(m):
    return sum(p.numel() for p in m.parameters() if p.requires_grad)


def run_one(c, Xtr, ytr, Xva, yva, posva, classes, epochs, batch, lr, dev, log):
    net = SmallCharNet(classes, (Xtr.shape[2], Xtr.shape[1]), *c).to(dev)
    opt = torch.optim.Adam(net.parameters(), lr = lr)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max = epochs)
    lossf = nn.CrossEntropyLoss()
    # 注意别把整个训练集转成 float32 常驻：31281x3x64x32x4 ≈ 768 MB，
    # 之前那样写会和别的训练任务抢内存。改成每批从 uint8 现转。
    yt = torch.from_numpy(ytr)
    n = len(yt)

    def infer(Xnp):
        out = []
        for s in range(0, len(Xnp), 512):
            b = torch.from_numpy(Xnp[s:s + 512]).permute(0, 3, 1, 2).float() / 255.0
            with torch.no_grad():
                out.append(net(b.to(dev)).argmax(1).cpu().numpy())
        return np.concatenate(out) if out else np.zeros(0, np.int64)

    for ep in range(epochs):
        net.train()
        perm = torch.randperm(n)
        tot = cor = 0
        for s in range(0, n, batch):
            idx = perm[s:s + batch].numpy()
            xb = torch.from_numpy(Xtr[idx]).permute(0, 3, 1, 2).float() / 255.0
            yb = yt[idx].to(dev)
            opt.zero_grad()
            o = net(xb.to(dev))
            l = lossf(o, yb)
            l.backward()
            opt.step()
            tot += len(yb)
            cor += int((o.argmax(1) == yb).sum())
        sched.step()
        if ep == epochs - 1 or (ep + 1) % 4 == 0:
            pv = infer(Xva)
            p0 = posva == 0
            log("    ep%-3d train %.2f%%  val %.2f%%  val位置0 %.2f%%(%d)"
                % (ep + 1, 100.0 * cor / tot, 100.0 * (pv == yva).mean(),
                   100.0 * (pv[p0] == yva[p0]).mean(), int(p0.sum())))
    pv = infer(Xva)
    per = {}
    for k in range(0, 8):
        m = posva == k
        if m.any():
            per[k] = 100.0 * (pv[m] == yva[m]).mean()
    return n_params(net), 100.0 * (pv == yva).mean(), per, net


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default = "data_crop/voc.data")
    ap.add_argument("--size", default = "32x64")
    ap.add_argument("--pad", type = float, default = 0.25)
    ap.add_argument("--epochs", type = int, default = 12)
    ap.add_argument("--batch", type = int, default = 256)
    ap.add_argument("--lr", type = float, default = 2e-3)
    ap.add_argument("--sweep", default = "8:16:32,16:32:64,32:64:128",
                    help = "逗号分隔的 c1:c2:c3 组合")
    ap.add_argument("--refresh", action = "store_true")
    args = ap.parse_args()

    P_W, P_H = (int(v) for v in args.size.lower().split("x"))
    cfg = U.load_datafile(args.data)
    classes = int(cfg["classes"])
    dev = torch.device("cpu")

    logf = open("char_head.log", "a", encoding = "utf-8")

    def log(s):
        print(s, flush = True)
        logf.write(s + "\n")
        logf.flush()

    log("=" * 92)
    log("### 第二级识别头参数预算扫描  size=%dx%d pad=%.2f epochs=%d"
        % (P_W, P_H, args.pad, args.epochs))
    log("=" * 92)
    Xtr, ytr, _ = cache_patches(cfg, "train", (P_W, P_H), args.pad, args.refresh)
    Xva, yva, posva = cache_patches(cfg, "val", (P_W, P_H), args.pad, args.refresh)
    log("  train %s  val %s" % (Xtr.shape, Xva.shape))

    combos = [tuple(int(v) for v in c.split(":")) for c in args.sweep.split(",")]
    tag_size = args.size.replace("x", "_")           # 32x64 -> 32_64
    res = []
    for c in combos:
        log("\n  --- c1:c2:c3 = %s ---" % (c,))
        t0 = time.time()
        npar, vacc, per, net = run_one(c, Xtr, ytr, Xva, yva, posva, classes,
                                       args.epochs, args.batch, args.lr, dev, log)
        res.append((c, npar, vacc, per))
        # 存下来 —— 参数扫描本身只是回答问题，但真正要落地的是「二级串起来能到多少」，
        # 那就得有真的权重去跑 `two_stage_e2e.py`。原来只打印不存，模型全丢了。
        os.makedirs("weights", exist_ok = True)
        ck = "weights/charhead_%d_%d_%d_%s.pt" % (c[0], c[1], c[2], tag_size)
        # 全部转成 python 原生类型再存。vacc / per 都是 numpy 标量，直接存进去会让
        # 这个文件在 torch>=2.6 的默认 `weights_only=True` 下**加载不了**
        # （UnpicklingError: numpy._core.multiarray.scalar）。踩过一次。
        torch.save({"state_dict": net.state_dict(), "c": [int(v) for v in c],
                    "size_wh": [int(P_W), int(P_H)], "pad": float(args.pad),
                    "classes": int(classes), "val_acc": float(vacc),
                    "per_pos": {str(k): float(v) for k, v in per.items()}}, ck)
        log("    权重存到 %s" % ck)
        log("    -> %d 参数（%.2f MB 若 int8），val %.2f%%，用时 %.0fs"
            % (npar, npar / 1e6, vacc, time.time() - t0))

    log("\n" + "=" * 92)
    log("### 汇总：参数 vs 逐位置准确率")
    log("%-16s %10s %8s | %s" % ("c1:c2:c3", "参数", "val%",
                                 "  ".join("位置%d" % k for k in range(7))))
    for c, npar, vacc, per in res:
        log("%-16s %10d %7.2f%% | %s"
            % (":".join(str(v) for v in c), npar, vacc,
               "  ".join("%5.2f%%" % per.get(k, float("nan")) for k in range(7))))
    # 检测器参数量：从权重文件数，但**必须把 BN 缓冲剔掉**（running_mean/var/
    # num_batches_tracked 有 8,569 个，推理时折进卷积，不是权重）。
    # 训练日志里 torchsummary 报的 248,570 也不对 —— 它按「每次 forward 调用记一遍」
    # 累加，而 LightFPN 里有 3 个模块一次前向被调用两次，多算了 6,205。
    ck = "weights/voc_crop-90-epoch-0.477014ap-model.pth"
    dpar = sum(v.numel() for k, v in torch.load(ck, map_location = "cpu").items()
               if not k.endswith(("running_mean", "running_var", "num_batches_tracked")))
    log("\n对照：检测器 %d 参数（位置 0 = 28.79%%）；探针 CharCNN 1,715,750 参数（位置 0 = 96.69%%）"
        % dpar)
    logf.close()


if __name__ == "__main__":
    main()
