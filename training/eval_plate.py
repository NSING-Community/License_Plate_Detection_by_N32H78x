# -*- coding: utf-8 -*-
"""PC 侧车牌「串级」准确率评测（只读：不写任何文件，只往 stdout 打）。

为什么需要这个脚本：`test_pth.py` 是单图可视化脚本 —— 它把框画出来、imshow 给你
看，一个数字都不输出，所以「准确率提升了没有」用它是判不出来的。本脚本按**固件
自己的装配规则**把每个样本拼成一个串，再和标签串逐字比。

固件规则（见 memory `plate-string-assembly`）：取所有 class_id > 0 的框，按 x 排序，
逐字拼接，截断 12 字节。本脚本照做（截断按字符数，标签串和预测串同样处理）。

四类数字，各有各的用处：
  det   类别无关的字符框召回 —— 每个真值字符框在 IoU>=match 下有没有被**任何类别**
        的框命中。这一项把「定位不行」和「分类不行」分开：
        det 高而 cls 低 ⇒ 框都在，只是认错字（分类问题）；det 本身就低 ⇒ 定位问题。
  cls   已命中的字符框里，类别猜对的比例。
  ch%   整串逐位准确率（预测框按 x 排序后取前 len(gt) 个与真值对齐）。
  exact 整串完全正确的样本比例。

同一个模型的 det/cls 会在一组 (conf, nms) 配置上各算一遍，**`handel_preds` 只跑一次**
（它是整个脚本里最贵的一步），所以扫配置几乎不要钱。

用法（多个权重在**同一个进程**里依次评测，否则第一个会吃到 NMS 的截断，
见 memory `nms-lazy-import-truncation`）：
  python eval_plate.py
  python eval_plate.py --weights a.pth b.pth --tag A B --split val
"""

import sys
sys.dont_write_bytecode = True

import os
import argparse
import numpy as np

import torch
import torchvision          # noqa: F401  —— 顶层导入，别让第一个模型去付这个 1.2 s
import cv2                  # noqa: F401  —— utils.utils 需要

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
os.chdir(HERE)

import utils.utils as U
import utils.datasets as D
import model.detector as MD

# 真值框与预测框算不算「同一个字符」的 IoU 门限（串级指标用 0.5）
MATCH_IOU = 0.5
# 扫的 (conf, nms_iou) 组合。1.0 表示不抑制（IoU 恒 <1，NMS 一个都不删）。
# (0.30, 0.45) 是链路默认；其余各档是为了回答两个问题：
#   conf 往下降能不能直接把召回换回来（能的话就是零重训的白捡）；
#   NMS 是不是帮凶（0.45 vs 1.00 的差）。
CONFIGS = [(0.30, 0.45), (0.20, 0.45), (0.15, 0.45), (0.10, 0.45),
           (0.05, 0.45), (0.15, 0.60), (0.15, 1.00)]
# det% 在几个匹配门限下各报一次：0.3 也能对上 = 框在、只是不准；0.3 都上不去 = 根本没框
MATCH_SWEEP = [0.30, 0.50, 0.70]
# obj 统计的分位点
PCT = [10, 25, 50, 75, 90]


def load_names(path):
    """data/voc.names 是 GBK 编码，一行一个类名。第 0 行是 plate。"""
    with open(path, encoding = "gbk") as f:
        return [ln.strip() for ln in f if ln.strip()]


def iou_matrix(a, b):
    """a:[n,4] xyxy, b:[m,4] xyxy -> [n,m] IoU（numpy，空输入返回 0 列/行）。"""
    n, m = a.shape[0], b.shape[0]
    if n == 0 or m == 0:
        return np.zeros((n, m), np.float32)
    lt = np.maximum(a[:, None, :2], b[None, :, :2])
    rb = np.minimum(a[:, None, 2:], b[None, :, 2:])
    wh = np.clip(rb - lt, 0.0, None)
    inter = wh[..., 0] * wh[..., 1]
    ar = (a[:, 2] - a[:, 0]) * (a[:, 3] - a[:, 1])
    br = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
    union = ar[:, None] + br[None, :] - inter
    return inter / np.maximum(union, 1e-9)


def conf_filter(out, conf):
    """不做 NMS，只按 conf 门限筛 —— non_max_suppression 的前半段。"""
    res = []
    for x in out:
        x = x[x[..., 4] > conf]
        if not x.shape[0]:
            res.append(np.zeros((0, 6), np.float32))
            continue
        x = x.clone()
        x[:, 5:] *= x[:, 4:5]
        box = U.xywh2xyxy(x[:, :4])
        c, j = x[:, 5:].max(1, keepdim = True)
        keep = torch.cat((box, c, j.float()), 1)[c.view(-1) > conf]
        res.append(keep.detach().numpy())
    return res


def iou_pair(a, b):
    lt = np.maximum(a[:2], b[:2])
    rb = np.minimum(a[2:], b[2:])
    wh = np.clip(rb - lt, 0.0, None)
    inter = wh[0] * wh[1]
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def gbk_bytes(ch):
    """固件那条 buf 是按**字节**截的（YOLO_PLATE_MAXLEN=12），省字占 2 个 GBK 字节。"""
    try:
        return len(ch.encode("gbk"))
    except Exception:
        return 1


def asm_variants(dxyxy, d_cls, d_conf, names, k_gt):
    """把同一批检测框用几种装配规则各拼一个串，用来判断「串错」有多少是装配的锅。

    A 就是固件的做法：按分数序取前 YOLO_PLATE_MAXBOX=16 个 class>0 的框，按 x1 排序拼接，
    按 **GBK 字节**截到 12。tools/chain_emulate.py::plate_read 是同一套。
    E 用真值字符数当 oracle 上限 —— 它回答的是「框和类别都对，只是多/少了几个」。
    """
    res = {k: "" for k in ("A", "B", "C", "E")}
    if not len(dxyxy):
        return res
    ci = np.nonzero(d_cls > 0)[0]
    if not len(ci):
        return res

    def s(sel):
        sel = np.asarray(sel, int)
        if not len(sel):
            return ""
        sel = sel[np.argsort(dxyxy[sel, 0] + dxyxy[sel, 2])]
        out = ""
        for c in d_cls[sel]:
            ch = names[c] if 0 <= c < len(names) else "?"
            if gbk_bytes(out + ch) > 12:
                break
            out += ch
        return out

    res["A"] = s(ci[:16])
    res["B"] = s(ci[np.argsort(-d_conf[ci])[:7]])
    res["E"] = s(ci[np.argsort(-d_conf[ci])[:k_gt]]) if k_gt else ""
    keep = []
    for j in ci[np.argsort(-d_conf[ci])]:
        if all(iou_pair(dxyxy[j], dxyxy[k]) <= 0.30 for k in keep):
            keep.append(j)
    res["C"] = s(keep)
    return res


def build_loader(cfg, split):
    ds = D.TensorDataset(cfg[split], cfg["width"], cfg["height"], imgaug = False)
    return torch.utils.data.DataLoader(ds,
                                       batch_size = int(cfg["batch_size"] / cfg["subdivisions"]),
                                       shuffle = False,
                                       collate_fn = D.collate_fn,
                                       num_workers = 0,
                                       pin_memory = False,
                                       drop_last = False)


def evaluate(net, loader, cfg, device, names, drop_plate = False):
    """一遍前向 + 一遍 handel_preds，然后在 CONFIGS 上各算一遍全套指标。

    drop_plate：裁掉所有 class 0 的框。给「车牌裁剪版」模型用 —— 那种数据集里根本没有
    class 0 的样本（标签只有字符），所以 class 0 头是没训过的随机噪声。它若不巧压在某个
    字符上，`Mc.argmax(1)` 会挑中它，`d_cls[bj[k]] == g_cls[k]` 就判错，把 cls% 压低。
    整图版模型不能开这个开关（class 0 = plate 是它的正经类别）。
    """
    W = cfg["width"]
    n_cfg = len(CONFIGS)
    base = dict(n_img = 0, n_gt_char = 0, n_gt_plate = 0)
    new = lambda: dict(det = [0] * len(MATCH_SWEEP), hits = 0, cls_ok = 0,
                       pairs = {}, plate = 0, exact = 0,
                       pos_ok = 0, pos_tot = 0, prov_ok = 0, prov_tot = 0,
                       named = 0, n_box = 0)
    per = [new() for _ in range(n_cfg)]
    asm = {k: dict(exact = 0, ok = 0, tot = 0, n = 0) for k in "ABCE"}
    examples = []
    # obj / cls 分解：在 IoU>=0.5 的候选里取 obj 最大的那个，记录 (obj, cls 概率, 类别对不对)。
    # 这一项不看任何 conf 门限，回答的是「一个框得很准的字符框，模型给了它多少分」。
    objs, clsp, clsok = [], [], []

    for imgs, targets in loader:
        imgs = imgs.to(device).float() / 255.0
        with torch.no_grad():
            out = U.handel_preds(net(imgs), cfg, device)
        dets = [conf_filter(out, c) if n >= 1.0 else
                U.non_max_suppression(out, conf_thres = c, iou_thres = n)
                for (c, n) in CONFIGS]

        tg = targets.numpy()          # [N,6] = img_idx, cls, cx, cy, w, h —— **归一化**
        for i in range(imgs.shape[0]):
            base["n_img"] += 1
            g = tg[tg[:, 0] == i]
            g_cls = g[:, 1].astype(int)
            if len(g):
                gxyxy = np.stack([g[:, 2] - g[:, 4] / 2, g[:, 3] - g[:, 5] / 2,
                                  g[:, 2] + g[:, 4] / 2, g[:, 3] + g[:, 5] / 2], 1) * W
            else:
                gxyxy = np.zeros((0, 4), np.float32)
            is_char = g_cls > 0
            base["n_gt_char"] += int(is_char.sum())
            base["n_gt_plate"] += int((g_cls == 0).sum())
            order = np.argsort(gxyxy[:, 0] + gxyxy[:, 2]) if len(g) else np.zeros(0, int)
            gt_str = "".join(names[c] if 0 <= c < len(names) else "?" for c in g_cls[order])
            gtc = "".join(names[c] if 0 <= c < len(names) else "?"
                          for c in g_cls[order] if c > 0)

            # ---- obj/cls 分解（无门限）
            if len(g):
                raw = out[i].cpu().numpy()
                # 类别概率里没有类别号，只有 argmax 出来才知道是谁，所以要屏蔽 class 0
                # 就把那一列清零再取 max/argmax，而不是按 `raw[:,5] != 0` 去筛（那是概率值）。
                cprob = raw[:, 5:]
                if drop_plate and cprob.shape[1]:
                    cprob = cprob.copy()
                    cprob[:, 0] = 0.0
                rxyxy = np.stack([raw[:, 0] - raw[:, 2] / 2, raw[:, 1] - raw[:, 3] / 2,
                                  raw[:, 0] + raw[:, 2] / 2, raw[:, 1] + raw[:, 3] / 2], 1)
                Mr = iou_matrix(gxyxy[is_char], rxyxy)
                for k in range(Mr.shape[0]):
                    jj = np.nonzero(Mr[k] >= MATCH_IOU)[0]
                    if not len(jj):
                        continue
                    jj = jj[raw[jj, 4].argmax()]
                    objs.append(float(raw[jj, 4]))
                    clsp.append(float(cprob[jj].max()))
                    clsok.append(int(cprob[jj].argmax() == g_cls[is_char][k]))

            for ci in range(n_cfg):
                d = dets[ci][i]
                if hasattr(d, "numpy"):
                    d = d.numpy()
                if drop_plate and len(d):
                    d = d[d[:, 5] != 0]          # d[:,5] 是类别号，这里是代码不是概率
                if len(d):
                    dxyxy, d_cls = d[:, :4], d[:, 5].astype(int)
                else:
                    dxyxy, d_cls = np.zeros((0, 4), np.float32), np.zeros(0, int)
                st = per[ci]
                st["n_box"] += len(d)
                Mc = iou_matrix(gxyxy, dxyxy)
                biou = Mc.max(1) if Mc.shape[1] else np.zeros(Mc.shape[0], np.float32)
                bj = Mc.argmax(1) if Mc.shape[1] else np.zeros(Mc.shape[0], int)
                if Mc.shape[1]:
                    for k in np.nonzero(is_char)[0]:
                        for mi, thr in enumerate(MATCH_SWEEP):
                            if biou[k] >= thr:
                                st["det"][mi] += 1
                        if biou[k] >= MATCH_IOU:
                            st["hits"] += 1
                            if d_cls[bj[k]] == g_cls[k]:
                                st["cls_ok"] += 1
                            key = (int(g_cls[k]), int(d_cls[bj[k]]))
                            st["pairs"][key] = st["pairs"].get(key, 0) + 1
                    for k in np.nonzero(g_cls == 0)[0]:
                        if biou[k] >= MATCH_IOU:
                            st["plate"] += 1

                # 串级指标：pd_all 走固件规则（分数序前 16 个 class>0 框 + 按 x 排 + GBK 12 字节），
                # pd_char 只取 class>0 按 x 排序、不截断，用来做逐位对齐。
                if len(d):
                    ci_ = np.nonzero(d_cls > 0)[0]
                    ci_ = ci_[np.argsort(dxyxy[ci_, 0] + dxyxy[ci_, 2])]
                    pd_char = "".join(names[c] if 0 <= c < len(names) else "?"
                                      for c in d_cls[ci_])
                    pd_all = asm_variants(dxyxy, d_cls, d[:, 4], names, 0)["A"]
                else:
                    pd_all, pd_char = "", ""
                if gtc:
                    st["named"] += 1
                    st["prov_tot"] += 1
                    if pd_char[:1] == gtc[:1]:
                        st["prov_ok"] += 1
                    if gtc == pd_all:
                        st["exact"] += 1
                for a, b in zip(gtc, pd_char[:len(gtc)]):
                    st["pos_tot"] += 1
                    if a == b:
                        st["pos_ok"] += 1
                if ci == 0:
                    if gtc:
                        av = asm_variants(dxyxy, d_cls, d[:, 4] if len(d) else d,
                                          names, len(gtc))
                        for kk, sv in av.items():
                            a = asm[kk]
                            a["n"] += 1
                            if sv == gtc:
                                a["exact"] += 1
                            for x, y in zip(gtc, sv[:len(gtc)]):
                                a["tot"] += 1
                                if x == y:
                                    a["ok"] += 1
                    if len(examples) < 10:
                        examples.append((gtc, pd_char, pd_char[:len(gtc)]))

    base["per"] = per
    base["asm"] = asm
    base["examples"] = examples
    base["obj"] = np.array(objs, np.float32) if objs else np.zeros(0, np.float32)
    base["clsp"] = np.array(clsp, np.float32) if clsp else np.zeros(0, np.float32)
    base["clsok"] = np.array(clsok, np.float32) if clsok else np.zeros(0, np.float32)
    return base


def report(tag, w, st, names):
    def pc(a, b):
        return 100.0 * a / b if b else 0.0
    print("\n" + "=" * 84)
    print("### %s   %s" % (tag, os.path.basename(w)))
    print("=" * 84)
    print("  样本 %d，真值字符框 %d，真值车牌框 %d"
          % (st["n_img"], st["n_gt_char"], st["n_gt_plate"]))
    print("  %-14s %8s %8s %8s %7s %7s %7s %8s %8s" % (
        "配置 conf/nms", "det@0.3", "det@0.5", "det@0.7", "框数/图",
        "cls%", "ch%", "exact%", "首字%"))
    for ci, (c, n) in enumerate(CONFIGS):
        p = st["per"][ci]
        print("  %-14s %7.2f%% %7.2f%% %7.2f%% %7.0f %7.2f %7.2f %8.4f %7.2f" % (
            "%.2f/%.2f" % (c, n),
            100.0 * p["det"][0] / max(st["n_gt_char"], 1),
            100.0 * p["det"][1] / max(st["n_gt_char"], 1),
            100.0 * p["det"][2] / max(st["n_gt_char"], 1),
            1.0 * p["n_box"] / max(st["n_img"], 1),
            100.0 * p["cls_ok"] / max(p["hits"], 1),
            100.0 * p["pos_ok"] / max(p["pos_tot"], 1),
            100.0 * p["exact"] / max(p["named"], 1),
            100.0 * p["prov_ok"] / max(p["prov_tot"], 1)))
    print("  plate 车牌框召回(conf.30/nms.45) %6.2f%%  (%d/%d)"
          % (pc(st["per"][0]["plate"], st["n_gt_plate"]),
             st["per"][0]["plate"], st["n_gt_plate"]))
    print("  --- 同一批检测框，换装配规则（conf.30/nms.45）---")
    for kk, label in (("A", "A 固件原样：分数序前16个 class>0，按 x 拼，GBK 12 字节"),
                      ("B", "B 只留最高分的 7 个框"),
                      ("C", "C 先做类无关 NMS(IoU 0.30)"),
                      ("E", "E oracle：留最高分的「真值字符数」个")):
        a = st["asm"][kk]
        print("      %-38s ch%% %6.2f%%   exact %6.2f%%  (%d/%d)"
              % (label, pc(a["ok"], a["tot"]), pc(a["exact"], a["n"]), a["exact"], a["n"]))

    o, cp_, ck = st["obj"], st["clsp"], st["clsok"]
    if len(o):
        print("  --- 框得很准(IoU>=0.5)的字符框：模型给的分（%d 个，无门限）---" % len(o))
        print("      %-8s %s" % ("分位", " ".join("%6d%%" % p for p in PCT)))
        print("      %-8s %s" % ("obj", " ".join("%6.3f" % np.percentile(o, p) for p in PCT)))
        print("      %-8s %s" % ("cls_p", " ".join("%6.3f" % np.percentile(cp_, p) for p in PCT)))
        print("      obj>0.30 的占比 %.2f%%   obj>0.10 %.2f%%   obj*cls>0.30 %.2f%%"
              % (100.0 * (o > 0.30).mean(), 100.0 * (o > 0.10).mean(),
                 100.0 * ((o * cp_) > 0.30).mean()))
        print("      这些框里 argmax 类别就是真值的占比 %.2f%%" % (100.0 * ck.mean()))

    p0 = st["per"][0]
    top = sorted(p0["pairs"].items(), key = lambda kv: -kv[1])[:24]
    print("  --- 最常混淆的 (真值 -> 预测) 前 %d ---" % len(top))
    for (a, b), n in top:
        print("      %-4s -> %-4s  %5d" % (names[a] if a < len(names) else a,
                                          names[b] if b < len(names) else b, n))
    print("  --- 前若干样本 (真值串 | 预测串 | 对齐后) ---")
    for g, pp, pa in st["examples"]:
        print("      %-12s | %-14s | %s" % (g, pp, pa))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", nargs = "+", default = [
        "weights/voc-290-epoch-0.273037ap-model.pth",
        "weights/voc-299-epoch-0.270855ap-model.pth",
    ])
    ap.add_argument("--tag", nargs = "+", default = None)
    ap.add_argument("--data", default = "data/voc.data")
    ap.add_argument("--split", default = "val")
    ap.add_argument("--drop-plate", action = "store_true",
                    help = "丢掉所有 class 0 的检测框。评测「车牌裁剪版」模型时必须开 —— "
                           "那种数据集标签里只有字符，class 0 头没训过，是随机噪声。"
                           "整图版模型不要开（class 0 就是 plate 本身）。")
    args = ap.parse_args()

    cfg = U.load_datafile(args.data)
    names = load_names(cfg["names"])
    device = torch.device("cpu")
    loader = build_loader(cfg, args.split)
    tags = args.tag or [os.path.splitext(os.path.basename(w))[0].split("-epoch")[0]
                        for w in args.weights]
    print("%s split=%s  类别数=%d  配置=%s  匹配门限=%s  drop_plate=%s"
          % (args.data, args.split, len(names) - 1, CONFIGS, MATCH_SWEEP, args.drop_plate))

    rows = []
    for tag, w in zip(tags, args.weights):
        net = MD.Detector(cfg["classes"], cfg["anchor_num"], True).to(device)
        net.load_state_dict(torch.load(w, map_location = device))
        net.eval()
        st = evaluate(net, loader, cfg, device, names, drop_plate = args.drop_plate)
        report(tag, w, st, names)
        rows.append((tag, st))

    print("\n" + "#" * 84)
    print("### 汇总（链路默认 conf0.30/nms0.45）")
    print("%-16s %9s %9s %9s %9s %9s %9s" %
          ("模型", "det@.3/.5", "cls%", "ch%", "首字%", "exact%", "plate%"))
    for tag, st in rows:
        p0 = st["per"][0]
        print("%-16s %4.1f/%4.1f %9.2f %9.2f %9.2f %9.4f %9.2f" % (
            tag,
            100.0 * p0["det"][0] / max(st["n_gt_char"], 1),
            100.0 * p0["det"][1] / max(st["n_gt_char"], 1),
            100.0 * p0["cls_ok"] / max(p0["hits"], 1),
            100.0 * p0["pos_ok"] / max(p0["pos_tot"], 1),
            100.0 * p0["prov_ok"] / max(p0["prov_tot"], 1),
            100.0 * p0["exact"] / max(p0["named"], 1),
            100.0 * p0["plate"] / max(st["n_gt_plate"], 1)))


if __name__ == "__main__":
    main()
