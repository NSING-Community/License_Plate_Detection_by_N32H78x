# -*- coding: utf-8 -*-
"""第二级识别器：整条车牌 -> 字符串（CNN + CTC）。

动机（见 memory `crop-resolution-ruled-out`）：同轮对照证明，字符在输入里从 13.3px 放大到
26px，分类误差**一动不动**（2.543 -> 2.609 nat），而框回归确实变好（1-CIoU 0.475 -> 0.438）。
探针又显示：同一批像素，换成带**全局池化**的分类头，逐字能到 99.2%。
=> 瓶颈不是字符有多少像素，而是检测器的分类头是主干特征上的一层 1×1 卷积，
   每个格点用自己的 72 维向量独立决策，**没有对整字做空间池化**。

本脚本不再走「检测每个字符框再逐个分类」，而是把**整条车牌**送进一个 CNN，
沿宽度方向输出 T 个时间步，用 CTC 直接对齐到字符串。这样每个输出位置看到的是
整条特征图的一部分（经卷积的上下文），而不是一个孤立格点。

数据直接用 `make_plate_crops.py` 生成的 data_crop/（车牌框 + margin 裁出的图），
标签是裁剪图坐标系里的字符框，按 x 中心排序即得字符串（中文车牌单行，左到右即顺序）。

类别映射（data/voc.names，GBK）：
    0        = plate（裁剪数据集里不出现，CTC 里正好拿 0 当 blank）
    1..10    = 字符 '0'..'9'
    11..34   = 字母 'A'..'Z'（去掉了 I 和 O，共 24 个）
    35..69   = 35 个省份
所以 num_classes = 70，blank = 0，目标直接用 category id，不需要重新编码。

用法：
    python train_plate_ctc.py --limit 400 --epochs 2 --size 96x192   # 冒烟
    python train_plate_ctc.py --epochs 40 --size 96x192              # 正式
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
os.chdir(HERE)

BLANK = 0            # class 0 = plate，裁剪数据里不存在，正好当 CTC blank
NUM_CLASSES = 70


def read_labels(img_path):
    """返回 [(cls, cx), ...]（归一化到裁剪图），按 cx 升序 —— 从小到大就是车牌读序。"""
    lab_path = img_path.split(".")[0] + ".txt"
    if not os.path.exists(lab_path):
        return None
    rows = []
    with open(lab_path) as f:
        for ln in f:
            p = ln.split()
            if len(p) != 5:
                continue
            c = int(float(p[0]))
            if c > 0:                      # 只留字符，plate 不进字符串
                rows.append((c, float(p[1])))
    if not rows:
        return None
    rows.sort(key=lambda r: r[1])
    return rows


def load_split(list_path, size_hw, limit = 0):
    """把整份数据读进内存（uint8）。

    返回 X[N,H,W,3]、Y（list of np.int32 字符串）、paths。
    图片是已经裁好的车牌小图，直接 resize 到 size_hw —— 和 datasets.py 一样是拉伸，
    不做 letterbox，保持和现有训练管线一致。
    """
    H_out, W_out = size_hw
    with open(list_path) as f:
        paths = [ln.strip() for ln in f if ln.strip()]
    if limit:
        paths = paths[:limit]

    X = np.zeros((len(paths), H_out, W_out, 3), np.uint8)
    Y, keep = [], []
    n_fail = 0
    for p in paths:
        lab = read_labels(p)
        if lab is None:
            n_fail += 1
            continue
        img = cv2.imread(p)
        if img is None:
            n_fail += 1
            continue
        if len(keep) >= len(X):
            break
        X[len(keep)] = cv2.resize(img, (W_out, H_out), interpolation=cv2.INTER_LINEAR)
        Y.append(np.array([c for c, _ in lab], np.int32))
        keep.append(p)
    return X[:len(keep)], Y, keep, n_fail


class PlateCRNN(nn.Module):
    """CNN -> 沿高度取平均 -> 每个宽度位置一个 logits。

    高度方向一路池到 4，再取均值，等价于「对每一列做一次全局池化」——
    这正是检测器分类头缺的那一步。
    """

    def __init__(self, classes):
        super(PlateCRNN, self).__init__()

        def blk(i, o, n, pool):
            layers = []
            for k in range(n):
                layers += [nn.Conv2d(i if k == 0 else o, o, 3, padding = 1, bias = False),
                           nn.BatchNorm2d(o), nn.ReLU(inplace = True)]
            layers += [nn.MaxPool2d(pool)]
            return layers

        # 宽度 /8，高度 /16：输入 H=64,W=192 -> 特征 H=4,W=24
        self.f = nn.Sequential(*blk(3, 32, 2, (2, 2)),      # 32x96
                               *blk(32, 64, 2, (2, 2)),     # 16x48
                               *blk(64, 128, 2, (2, 1)),    # 8x48
                               *blk(128, 256, 2, (2, 1)))   # 4x48
        self.head = nn.Conv1d(256, classes, 1)

    def forward(self, x):
        f = self.f(x)                       # N,256,4,W'
        f = f.mean(2)                       # 对高度取平均 -> N,256,W'
        return self.head(f)                 # N,classes,W'  （未过 softmax，CTC 要 log_softmax）


def decode(logits):
    """贪心解码：argmax -> 去重 -> 去 blank。

    ⚠️ 入参必须是 **(T, N, C)**，也就是时间维在最前面 —— 和 nn.CTCLoss 一样。
    模型 forward 出来的是 (N, C, T)，调用前要先 `lg.permute(2, 0, 1)`。
    这里踩过坑：曾经直接把 (N,C,T) 传进来，`argmax(2)` 就变成了「在时间维上取最大」，
    得到 (N,C)，再被当成「T=C=70 个时间步」去解码 —— 解出来是一堆废字符，
    看起来像「CTC 训不起来」，其实是解码喂错了张量。见 `_check_ctc_decode.py`。
    """
    ids = logits.argmax(2)                  # T, N
    out = []
    for n in range(ids.shape[1]):
        seq = ids[:, n].tolist()
        prev, s = None, []
        for v in seq:
            if v != prev and v != BLANK:
                s.append(v)
            prev = v
        out.append(s)
    return out


def edit_ops(a, b):
    """返回 (命中位置数, 参考长度) —— 用编辑距离的近似：直接逐位置比会更悲观，
    这里用 Levenshtein 的替换/插入/删除，命中数 = len(a) - (距离里非插入的部分)。
    简化起见就报 Levenshtein 距离，再由调用方换算。"""
    n, m = len(a), len(b)
    d = np.arange(m + 1)
    for i in range(1, n + 1):
        prev, d[0] = d[0], i
        for j in range(1, m + 1):
            cur = d[j]
            d[j] = min(d[j] + 1, d[j - 1] + 1, prev + (a[i - 1] != b[j - 1]))
            prev = cur
    return int(d[m])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default = "data_crop")
    ap.add_argument("--size", default = "96x192", help = "HxW")
    ap.add_argument("--epochs", type = int, default = 40)
    ap.add_argument("--batch", type = int, default = 64)
    ap.add_argument("--lr", type = float, default = 1e-3)
    ap.add_argument("--limit", type = int, default = 0, help = "只用前 N 张，冒烟用")
    ap.add_argument("--val", default = "",
                    help = "覆盖验证集列表（诊断用：指向 train.txt 可判断是「不泛化」还是「解码/流程有 bug」）")
    ap.add_argument("--tag", default = "")
    args = ap.parse_args()

    H_out, W_out = (int(v) for v in args.size.lower().split("x"))

    print("=" * 84)
    print("### 整条车牌 CTC 识别器  size=%dx%d(HxW)  epochs=%d  batch=%d  lr=%g"
          % (H_out, W_out, args.epochs, args.batch, args.lr))
    print("=" * 84)

    t0 = time.time()
    Xtr, Ytr, Ptr, f1 = load_split(os.path.join(args.data, "train.txt"), (H_out, W_out), args.limit)
    val_list = args.val or os.path.join(args.data, "val.txt")
    if args.val:
        print("!! 验证集被覆盖为 %s —— 这是诊断模式，报的准确率不代表泛化能力" % val_list)
    Xva, Yva, Pva, f2 = load_split(val_list, (H_out, W_out), args.limit)
    print("读入 %.1fs：train %s  val %s（丢弃无字符标签 %d/%d）"
          % (time.time() - t0, Xtr.shape, Xva.shape, f1, f2))

    lt = np.array([len(y) for y in Ytr])
    hist = {}
    for k in lt:
        hist[k] = hist.get(k, 0) + 1
    print("训练集字符串长度分布（前 6）：%s"
          % "  ".join("%d字:%d张" % (k, v) for k, v in sorted(hist.items(), key = lambda kv: -kv[1])[:6]))

    dev = torch.device("cpu")
    torch.manual_seed(0)
    model = PlateCRNN(NUM_CLASSES).to(dev)
    print("模型参数 %d" % sum(p.numel() for p in model.parameters()))
    opt = torch.optim.Adam(model.parameters(), lr = args.lr, weight_decay = 1e-4)
    sch = torch.optim.lr_scheduler.CosineAnnealingLR(opt, args.epochs)
    ctc = nn.CTCLoss(blank = BLANK, zero_infinity = True)

    Xtr_t = torch.from_numpy(Xtr).permute(0, 3, 1, 2).float() / 255.0
    Xva_t = torch.from_numpy(Xva).permute(0, 3, 1, 2).float() / 255.0

    def run_val():
        model.eval()
        preds = []
        with torch.no_grad():
            for i in range(0, len(Xva_t), 128):
                lg = model(Xva_t[i:i + 128])          # N,C,T
                preds += decode(lg.permute(2, 0, 1))  # decode 要 T,N,C
        exact = sum(1 for p, y in zip(preds, Yva) if p == list(y))
        dist = sum(edit_ops(p, list(y)) for p, y in zip(preds, Yva))
        ref = sum(len(y) for y in Yva)
        return 100.0 * exact / max(len(Yva), 1), 100.0 * (1 - dist / max(ref, 1)), preds

    def pos_report(preds):
        """按项目一贯的「取前 len(gt) 位逐位比」口径，看**是哪个位置在错**。

        动机：跑起来之后 val_1-CER 爬到 72% 却一直停在 val_exact 0.19%，
        两者差得太远 —— 纯粹随机错的话 1-CER 72% 对应的整串命中应该有百分之十几。
        这个形状更像「固定某几位一直错」。CTC 是变长的，所以短的补空、长的截断，
        和 eval_plate/cascade_ceiling 的逐位口径一致，读出来的数可以直接
        和检测链路的 位置0%/位置1-6% 对照。
        """
        tot = [0] * 8
        ok = [0] * 8
        dl = []
        for p, y in zip(preds, Yva):
            y = list(y)
            dl.append(len(p) - len(y))
            for k in range(min(7, len(y))):
                tot[k] += 1
                ok[k] += int(k < len(p) and p[k] == y[k])
        head = "  ".join("位置%d %5.1f%%(n=%d)" % (k, 100.0 * ok[k] / tot[k], tot[k])
                         for k in range(4) if tot[k])
        tail = "  ".join("位置%d %5.1f%%(n=%d)" % (k, 100.0 * ok[k] / tot[k], tot[k])
                         for k in range(4, 7) if tot[k])
        return ("      %s\n      %s\n      预测长度-真值长度：中位 %+.0f  均值 %+.2f  完全相等 %.1f%%"
                % (head, tail, float(np.median(dl)), float(np.mean(dl)),
                   100.0 * float(np.mean([d == 0 for d in dl]))))

    n = len(Xtr_t)
    steps = (n + args.batch - 1) // args.batch
    print("每轮梯度步数 %d（共 %d 轮 => %d 步）" % (steps, args.epochs, steps * args.epochs))
    if steps * args.epochs < 1500:
        print("  ⚠️ 总步数偏少。CTC 从零起步通常要 1500 步以上才脱离退化态"
              "（表现为 loss 卡住、输出一大堆废字符、1-CER 为负）。")
        print("     用 --limit 冒烟时 batch 会自动大于样本数，一轮只有 1 步 —— 冒烟测不出收敛性，"
              "别据此判断架构行不行。")
    best = -1.0
    for ep in range(args.epochs):
        model.train()
        perm = torch.randperm(n)
        ls, tot = 0.0, 0
        t1 = time.time()
        for i in range(0, n, args.batch):
            b = perm[i:i + args.batch]
            xb = Xtr_t[b].to(dev)
            tg = [torch.from_numpy(Ytr[j]) for j in b]
            tgt = torch.cat(tg)
            tl = torch.tensor([len(t) for t in tg], dtype = torch.long)
            opt.zero_grad()
            lg = model(xb)                                   # N,C,T
            lp = lg.permute(2, 0, 1).log_softmax(2)          # T,N,C
            tl_ = torch.full((len(b),), lp.shape[0], dtype = torch.long)
            l = ctc(lp, tgt, tl_, tl)
            l.backward()
            opt.step()
            ls += l.item() * len(b)
            tot += len(b)
        sch.step()
        ex, cer, pv = run_val()
        # 每轮存最好的一轮：这个任务要跑好几个小时，原来只在最后存一次，
        # 中途崩了（或者我要提前看结果）就全没了。
        if ex >= best:
            os.makedirs("weights", exist_ok = True)
            torch.save(model.state_dict(),
                       "weights/plate_ctc_%s%s_best.pt" % (args.size, args.tag))
        best = max(best, ex)
        print("  ep %2d  CTC %.4f  val_exact %6.2f%%  val_1-CER %6.2f%%  (%.0fs)"
              % (ep + 1, ls / tot, ex, cer, time.time() - t1))
        if (ep + 1) % 4 == 0 or ep == args.epochs - 1:
            print(pos_report(pv))

    ex, cer, preds = run_val()
    print("\n  --- 结果 ---")
    print("  val 整串命中（exact）%.2f%%   逐字(1-CER) %.2f%%   最好一轮 exact %.2f%%"
          % (ex, cer, best))
    print(pos_report(preds))

    names = []
    try:
        with open("data/voc.names", encoding = "gbk", errors = "replace") as f:
            names = [ln.strip() for ln in f if ln.strip()]
    except Exception:
        pass
    show = lambda s: "".join(names[c] if c < len(names) else "?" for c in s)
    print("  --- 样例（真值 | 预测）---")
    bad = 0
    for p, y in zip(preds, Yva):
        if p != list(y):
            print("      %-14s | %s" % (show(list(y)), show(p)))
            bad += 1
            if bad >= 20:
                break
    if bad == 0:
        print("      （val 全部命中）")

    os.makedirs("weights", exist_ok = True)
    out = "weights/plate_ctc_%s%s.pt" % (args.size, args.tag)
    torch.save(model.state_dict(), out)
    print("\n  权重存到 %s" % out)


if __name__ == "__main__":
    main()
