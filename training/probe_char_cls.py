# -*- coding: utf-8 -*-
"""字符分类的信息上限探针（会写模型到 weights/probe_*，其余只往 stdout 打）。

要回答的问题（见 memory `plate-bottleneck-2026-10-15`）：7 字车牌要整串命中需要逐字约 99%
（0.99^7=0.93），而当前模型逐字只有约 81%。在花几小时重训「车牌裁剪」版之前，必须先确认
**这批图像里到底有没有那 99% 的信息**。

做法：从**原图原生分辨率**里把每个字符框抠成小图（外扩一点上下文），训练一个独立的分类
CNN，看它能把逐字准确率做到多少。这是两级结构里识别器拿到的**同一批像素**，所以它测的是
信息上限，与检测器的架构无关。

读数方式：
  val 高（≈99%）  ⇒ 信息在，单阶段是架构/分辨率的锅，两级结构有救，值得重训。
  val 低（≈85%）  ⇒ 信息不在，两级结构同样到不了 99%，得改成像分辨率或拍摄条件，
                     别在模型上继续投入。

同时跑两个输入尺寸做对照（同一批像素，只改网络看到的网格密度）：
  --size 16x32  接近原生（字符原生约 16×29）
  --size 32x64  放大网格
若 32x64 明显好于 16x32 ⇒ 网络需要更多空间格点才分得开（两级结构放大的正是这一项）；
若两者持平 ⇒ 纯粹是信息量问题，放大没用。

判读还要看 **train acc**：train 也上不去 ⇒ 容量/信息上限；train 高但 val 低 ⇒ 过拟合，
加数据/加正则还有空间。

用法：
  python probe_char_cls.py --size 16x32
  python probe_char_cls.py --size 32x64 --epochs 15
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
import torch.nn.functional as F

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)

import utils.utils as U      # noqa: E402


def read_labels(img_path):
    lab_path = img_path.split(".")[0] + ".txt"
    if not os.path.exists(lab_path):
        return None
    rows = []
    with open(lab_path) as f:
        for ln in f:
            p = ln.split()
            if len(p) == 5:
                rows.append([float(x) for x in p])
    return np.array(rows, np.float32) if rows else None


def collect(paths):
    """返回 (列表 of (img_path, cls, cx, cy, w, h 归一化))，只收 class>0 的字符框。"""
    out = []
    for p in paths:
        lab = read_labels(p)
        if lab is None:
            continue
        for r in lab:
            if r[0] > 0:
                out.append((p, int(r[0]), r[1], r[2], r[3], r[4]))
    return out


def build_patches(items, size_wh, pad, limit = 0):
    """把字符框抠成 uint8 小图。pad=0.25 表示四周各扩框自身尺寸的 25%。
    返回 (X[N,H,W,3] uint8, y[N], n_fail)。用 uint8 存，省内存。"""
    W_out, H_out = size_wh
    X = np.zeros((len(items), H_out, W_out, 3), np.uint8)
    y = np.zeros(len(items), np.int64)
    n = 0
    n_fail = 0
    cache = {}
    for i, (ip, cls, cx, cy, bw, bh) in enumerate(items):
        if limit and n >= limit:
            break
        img = cache.get(ip)
        if img is None:
            img = cv2.imread(ip)
            if img is None:
                n_fail += 1
                continue
            if len(cache) > 64:          # 别把整个数据集缓进内存
                cache.clear()
            cache[ip] = img
        H, W = img.shape[:2]
        x0 = (cx - bw / 2) * W
        y0 = (cy - bh / 2) * H
        x1 = (cx + bw / 2) * W
        y1 = (cy + bh / 2) * H
        mx = (x1 - x0) * pad
        my = (y1 - y0) * pad
        X0 = int(np.clip(np.floor(x0 - mx), 0, W - 1))
        Y0 = int(np.clip(np.floor(y0 - my), 0, H - 1))
        X1 = int(np.clip(np.ceil(x1 + mx), X0 + 1, W))
        Y1 = int(np.clip(np.ceil(y1 + my), Y0 + 1, H))
        patch = img[Y0:Y1, X0:X1]
        if patch.size == 0:
            n_fail += 1
            continue
        X[n] = cv2.resize(patch, (W_out, H_out), interpolation = cv2.INTER_LINEAR)
        y[n] = cls
        n += 1
    return X[:n], y[:n], n_fail


class CharCNN(nn.Module):
    """够用的中小型 CNN。故意不做得太弱 —— 探针若因容量不足而误判「信息不在」就糟了。"""

    def __init__(self, classes, size_wh):
        super(CharCNN, self).__init__()
        W_out, H_out = size_wh
        # 4 次 MaxPool2d(2)  ->  特征图 H/16 × W/16
        flat = 256 * (H_out // 16) * (W_out // 16)

        def blk(i, o, n = 2):
            layers = []
            for k in range(n):
                layers += [nn.Conv2d(i if k == 0 else o, o, 3, padding = 1, bias = False),
                           nn.BatchNorm2d(o), nn.ReLU(inplace = True)]
            layers += [nn.MaxPool2d(2)]
            return layers

        self.f = nn.Sequential(*blk(3, 32), *blk(32, 64), *blk(64, 128), *blk(128, 256))
        self.head = nn.Sequential(nn.Flatten(),
                                  nn.Linear(flat, 256), nn.ReLU(inplace = True),
                                  nn.Dropout(0.2), nn.Linear(256, classes))

    def forward(self, x):
        return self.head(self.f(x))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--size", default = "16x32", help = "WxH，如 16x32 / 32x64")
    ap.add_argument("--epochs", type = int, default = 15)
    ap.add_argument("--batch", type = int, default = 256)
    ap.add_argument("--lr", type = float, default = 2e-3)
    ap.add_argument("--pad", type = float, default = 0.25, help = "字符框外扩比例（给上下文）")
    ap.add_argument("--limit", type = int, default = 0, help = "限制训练样本数，用于快速试跑")
    ap.add_argument("--tag", default = "", help = "输出文件名后缀")
    args = ap.parse_args()

    W_out, H_out = (int(v) for v in args.size.lower().split("x"))
    cfg = U.load_datafile("data/voc.data")
    names = U.load_names(cfg["names"])
    classes = cfg["classes"]

    with open(cfg["train"]) as f:
        tr_paths = [ln.strip() for ln in f if ln.strip()]
    with open(cfg["val"]) as f:
        va_paths = [ln.strip() for ln in f if ln.strip()]

    print("=" * 84)
    print("### 字符分类探针  size=%dx%d  pad=%.2f  epochs=%d" % (W_out, H_out, args.pad, args.epochs))
    print("=" * 84)
    t0 = time.time()
    tr_items = collect(tr_paths)
    va_items = collect(va_paths)
    print("字符框：train %d   val %d   （类别数 %d）" % (len(tr_items), len(va_items), classes))

    Xtr, ytr, f1 = build_patches(tr_items, (W_out, H_out), args.pad, args.limit)
    Xva, yva, f2 = build_patches(va_items, (W_out, H_out), args.pad)
    print("抠图完成 %.1fs：train %s val %s（失败 %d/%d）"
          % (time.time() - t0, Xtr.shape, Xva.shape, f1, f2))

    # 太少的类别参与不了训练，也不该算进宏平均。class 0 是 plate，字符任务里本来就不出现，排除。
    cnt = np.bincount(ytr, minlength = classes)
    rare = [c for c in np.nonzero(cnt < 2)[0] if c > 0]
    print("训练集里字符样本 <2 的类别：%d 个 %s" % (len(rare), [names[i] for i in rare[:12]]))

    dev = torch.device("cpu")
    torch.manual_seed(0)
    model = CharCNN(classes, (W_out, H_out)).to(dev)
    n_par = sum(p.numel() for p in model.parameters())
    opt = torch.optim.Adam(model.parameters(), lr = args.lr, weight_decay = 1e-4)
    sch = torch.optim.lr_scheduler.CosineAnnealingLR(opt, args.epochs)
    lossf = nn.CrossEntropyLoss()

    Xtr_t = torch.from_numpy(Xtr).permute(0, 3, 1, 2).float() / 255.0
    ytr_t = torch.from_numpy(ytr)
    Xva_t = torch.from_numpy(Xva).permute(0, 3, 1, 2).float() / 255.0
    yva_t = torch.from_numpy(yva)

    def run_eval(X, y):
        model.eval()
        pred = []
        with torch.no_grad():
            for i in range(0, len(X), 1024):
                pred.append(model(X[i:i + 1024]).argmax(1))
        pred = torch.cat(pred)
        return 100.0 * (pred == y).float().mean().item(), (pred == y).numpy()

    print("模型参数 %d" % n_par)
    n = len(Xtr_t)
    best = 0.0
    for ep in range(args.epochs):
        model.train()
        perm = torch.randperm(n)
        tot, hit, ls = 0, 0, 0.0
        for i in range(0, n, args.batch):
            b = perm[i:i + args.batch]
            xb, yb = Xtr_t[b].to(dev), ytr_t[b].to(dev)
            opt.zero_grad()
            out = model(xb)
            l = lossf(out, yb)
            l.backward()
            opt.step()
            ls += l.item() * len(b)
            hit += (out.argmax(1) == yb).sum().item()
            tot += len(b)
        sch.step()
        va_acc, _ = run_eval(Xva_t, yva_t)
        best = max(best, va_acc)
        print("  ep %2d  train_acc %6.2f%%  train_CE %.3f  val_acc %6.2f%%"
              % (ep + 1, 100.0 * hit / tot, ls / tot, va_acc))

    tr_acc, _ = run_eval(Xtr_t, ytr_t)
    va_acc, ok = run_eval(Xva_t, yva_t)
    print("\n  --- 结果 size=%dx%d ---" % (W_out, H_out))
    print("  train_acc %.2f%%   val_acc %.2f%%   （最好一轮 val %.2f%%）" % (tr_acc, va_acc, best))

    # 逐类准确率：宏平均 + 最差的类
    print("  --- 逐类（val，括号内为样本数）---")
    per = []
    for c in range(classes):
        m = yva == c
        if m.sum() == 0:
            continue
        per.append((100.0 * ok[m].mean(), c, int(m.sum())))
    per.sort()
    macro = np.mean([p[0] for p in per])
    print("  宏平均（只算 val 里出现过的类）%.2f%%   参与类数 %d" % (macro, len(per)))
    print("  最差的 15 类：")
    for acc, c, k in per[:15]:
        print("      %-4s acc %6.2f%%  n=%d" % (names[c] if c < len(names) else c, acc, k))
    print("  最好的 8 类：")
    for acc, c, k in per[-8:]:
        print("      %-4s acc %6.2f%%  n=%d" % (names[c] if c < len(names) else c, acc, k))

    # 串级换算：逐字 acc -> 7 字整串命中的期望
    print("\n  --- 串级换算（假设 7 字独立同分布）---")
    for tag, a in (("整体 top1", va_acc), ("宏平均", macro)):
        print("      %-8s %6.2f%%  ->  7 字全对 %.2f%%" % (tag, a, 100.0 * (a / 100.0) ** 7))

    out = "weights/probe_charcls_%s%s.pt" % (args.size, args.tag)
    os.makedirs("weights", exist_ok = True)
    torch.save(model.state_dict(), out)
    print("\n  权重存到 %s（仅探针用，与部署无关）" % out)


if __name__ == "__main__":
    main()
