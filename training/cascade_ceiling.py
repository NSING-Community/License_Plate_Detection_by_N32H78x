# -*- coding: utf-8 -*-
"""两级结构的**真**上限：把「框从哪来」和「谁来分类」这两件事分开量。

前情：探针（`probe_char_cls.py`）报的「串级 94.14%」其实是 `0.9914**7` 的独立同分布外推，
**不是**真测出来的串级 —— 它假定了框完美、且逐字错误互相独立。本脚本把框换成检测器的
**真实预测框**，重新量一遍，才是两级结构落到实处的数字。

2x2 对照（同一批 val 图、同一套装配规则、同一个探针分类头）：

                      检测器分类头        探针分类头(全局池化)
    真值框(GT)        cls% 80.60(已有)    ?          <- 只差分类头
    预测框(规则 C)    43.09 / 9.53(已有)   ?          <- 两级结构真实落点

读法：
  · 若「GT 框 + 探针头」≈99% 而「预测框 + 探针头」远低于它
      ⇒ 两级结构的瓶颈全在**检测器给框的位置/数量**，换再好的识别头也救不回来。
  · 若「预测框 + 探针头」明显高于「预测框 + 检测器头」
      ⇒ 换分类头（也就是真的做第二级）是有收益的，值得上 MCU 评估。

用法：
  python cascade_ceiling.py --weights weights/voc_crop-90-epoch-0.477014ap-model.pth \
      --probe weights/probe_charcls_32x64.pt --size 32x64 --conf 0.30
"""

import sys
sys.dont_write_bytecode = True

import os
import time
import argparse
import numpy as np
import cv2

import torch
import torchvision          # noqa: F401
import torch.nn as nn

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
os.chdir(HERE)

import utils.utils as U        # noqa: E402
import model.detector as MD    # noqa: E402
import probe_char_cls as P     # noqa: E402  CharCNN 定义在上面，别抄一份


def load_names(path):
    with open(path, encoding = "gbk", errors = "replace") as f:
        return [ln.strip() for ln in f if ln.strip()]


def gbk_bytes(ch):
    try:
        return len(ch.encode("gbk"))
    except Exception:
        return 1


def iou_pair(a, b):
    lt = np.maximum(a[:2], b[:2])
    rb = np.minimum(a[2:], b[2:])
    wh = np.clip(rb - lt, 0.0, None)
    inter = wh[0] * wh[1]
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def cut_patch(img, box_px, pad, size_wh):
    """从**原生分辨率**裁剪图里按框抠小图，外扩 pad（和探针一致），再 resize 到 size_wh。"""
    W_out, H_out = size_wh
    H, W = img.shape[:2]
    x0, y0, x1, y1 = box_px
    mx, my = (x1 - x0) * pad, (y1 - y0) * pad
    X0 = int(np.clip(np.floor(x0 - mx), 0, W - 1))
    Y0 = int(np.clip(np.floor(y0 - my), 0, H - 1))
    X1 = int(np.clip(np.ceil(x1 + mx), X0 + 1, W))
    Y1 = int(np.clip(np.ceil(y1 + my), Y0 + 1, H))
    p = img[Y0:Y1, X0:X1]
    if p.size == 0:
        return None
    return cv2.resize(p, (W_out, H_out), interpolation = cv2.INTER_LINEAR)


def assemble(cx, chars, cap = 12):
    """按 x 中心排序拼接，GBK 字节截断——和固件 / eval_plate 的规则一致。"""
    out = ""
    for i in np.argsort(cx):
        ch = chars[i]
        if ch is None:
            continue
        if gbk_bytes(out + ch) > cap:
            break
        out += ch
    return out


def _geo_filter(idx, boxes_px, conf):
    """近似固件的 `plate_geo_filter()`（yolo_inference.cpp:275-319，默认关，'v' 开关打开）。

    两步：① 高度在 [0.6,1.6]×中位、宽度在 [0.5,1.6]×中位之内才留；
    ② 同一行内、x 方向重叠超过**较窄框宽度的一半**的算重复，按分数序去重。
    固件里那个 0.5 系数注释说是「载重」的（换成 0.3 会误删）。
    """
    if len(idx) < 4:
        return idx
    h = boxes_px[idx, 3] - boxes_px[idx, 1]
    w = boxes_px[idx, 2] - boxes_px[idx, 0]
    mh, mw = np.median(h), np.median(w)
    keep = idx[(h <= 1.6 * mh) & (h >= 0.6 * mh) & (w <= 1.6 * mw) & (w >= 0.5 * mw)]
    if len(keep) < 2:
        return idx
    keep = keep[np.argsort(-conf[keep])]
    out = []
    for j in keep:
        bad = False
        for k in out:
            oy = min(boxes_px[j, 3], boxes_px[k, 3]) - max(boxes_px[j, 1], boxes_px[k, 1])
            if oy <= 0.0:
                continue
            ox = min(boxes_px[j, 2], boxes_px[k, 2]) - max(boxes_px[j, 0], boxes_px[k, 0])
            bw = boxes_px[j, 2] - boxes_px[j, 0]
            cw = boxes_px[k, 2] - boxes_px[k, 0]
            if ox * 2.0 > min(bw, cw):
                bad = True
                break
        if not bad:
            out.append(j)
    return np.asarray(out, int)


def _nms(order, boxes_px, thr):
    keep = []
    for j in order:
        if all(iou_pair(boxes_px[j], boxes_px[k]) <= thr for k in keep):
            keep.append(j)
    return np.asarray(keep, int)


def select_rule(kind, boxes_px, conf, k_gt):
    """返回选中的下标。

    A/B/C/E 与 eval_plate.asm_variants 完全同定义；D/E2/F/G 是这次新加的候选规则，
    都在「同一批框、零重训成本」的范围内，用来回答「固件该怎么挑框」。
      D  = 先类无关 NMS(0.30) 再去重后按分数留 7 个   —— C 去重了但可能留多余框
      E2 = 先 NMS(0.30) 再留「真值字符数」个        —— 把「多吐框」和「框不对」分开
      F  = D 的基础上再按高度中位数滤掉尺寸离谱的框
      G  = NMS 门限收紧到 0.15（更狠地去重）
    """
    n = len(boxes_px)
    if n == 0:
        return np.zeros(0, int)
    order = np.argsort(-conf)
    if kind == "A":
        return order[:16]
    if kind == "B":
        return order[:7]
    if kind == "E":
        return order[:k_gt] if k_gt else np.zeros(0, int)
    if kind == "GT":
        return np.arange(n)
    if kind == "H":
        # 固件的几何过滤（'v' 开关）。注意顺序：固件是先把前 16 个 class>0 的挑出来
        # （yolo_plate_read:325-327 的 m < YOLO_PLATE_MAXBOX），再过滤，所以要先切 16。
        return _geo_filter(order[:16], boxes_px, conf)
    if kind in ("C", "D", "E2", "F"):
        keep = _nms(order, boxes_px, 0.30)
    elif kind == "G":
        keep = _nms(order, boxes_px, 0.15)
    else:
        return np.zeros(0, int)
    if kind in ("C", "G"):
        return keep
    if kind == "F" and len(keep):
        h = boxes_px[keep, 3] - boxes_px[keep, 1]
        med = np.median(h)
        keep = keep[(h >= 0.6 * med) & (h <= 1.5 * med)]
    k = k_gt if kind == "E2" else 7
    if k and len(keep) > k:
        keep = keep[np.argsort(-conf[keep])[:k]]
    return keep


def score(preds, gts):
    """返回 (逐位命中, 逐位总数, 整串命中, 样本数)。"""
    ok = tot = ex = 0
    for p, g in zip(preds, gts):
        if p == g:
            ex += 1
        for a, b in zip(g, p[:len(g)]):
            tot += 1
            if a == b:
                ok += 1
    return ok, tot, ex, len(gts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default = "weights/voc_crop-90-epoch-0.477014ap-model.pth")
    ap.add_argument("--probe", default = "weights/probe_charcls_32x64.pt")
    ap.add_argument("--size", default = "32x64", help = "探针输入 WxH，必须和 --probe 配套")
    ap.add_argument("--data", default = "data_crop/voc.data")
    ap.add_argument("--split", default = "val",
                    help = "在哪一份上评。规则是拿 val 选的，再跑一遍 train 只是为了看"
                           "规则是不是只在 val 上凑出来的（过拟合自查），不是泛化证据。")
    ap.add_argument("--conf", type = float, default = 0.30)
    ap.add_argument("--nms", type = float, default = 0.45)
    ap.add_argument("--batch", type = int, default = 32)
    ap.add_argument("--pad", type = float, default = 0.25)
    ap.add_argument("--limit", type = int, default = 0)
    ap.add_argument("--keep-plate", action = "store_true",
                    help = "不丢掉 class 0 的框。裁剪数据集里没有 class 0 样本，那个头是没训过的"
                           "随机权重；本开关用来验证「省份字是不是被这个随机头抢走、再被过滤掉的」"
                           "—— 开启后若检测召回从 90%% 跳到接近 100%%，假设就成立。")
    args = ap.parse_args()

    P_W, P_H = (int(v) for v in args.size.lower().split("x"))
    cfg = U.load_datafile(args.data)
    names = load_names(cfg["names"])
    classes = int(cfg["classes"])
    W = int(cfg["width"])
    MIOU = 0.5

    paths = [ln.strip() for ln in open(cfg[args.split]) if ln.strip()]
    if args.limit:
        paths = paths[:args.limit]
    paths = [p for p in paths if os.path.exists(p.split(".")[0] + ".txt")]
    print("=" * 88)
    print("### 两级结构真上限  %s %d 张  conf=%.2f nms=%.2f  探针 %dx%d"
          % (args.split, len(paths), args.conf, args.nms, P_W, P_H))
    print("=" * 88)

    dev = torch.device("cpu")
    net = MD.Detector(classes, int(cfg["anchor_num"]), True).to(dev)
    net.load_state_dict(torch.load(args.weights, map_location = dev))
    net.eval()
    probe = P.CharCNN(classes, (P_W, P_H)).to(dev)
    probe.load_state_dict(torch.load(args.probe, map_location = dev))
    probe.eval()

    rules = ["GT", "A", "B", "C", "D", "E2", "F", "G", "E"]
    acc = {r: {"prb": [0, 0, 0, 0], "det": [0, 0, 0, 0]} for r in rules}
    # 检测召回：真值字符框在 IoU>=0.5 下有没有被任何类别命中（顺带印证 eval_plate 的 number）
    n_gt_char = n_hit = n_cls0 = 0
    # 诊断：真值字符框按「最佳预测框的 IoU」分桶，看探针逐字准确率怎么随框质量退化。
    # 这是为了回答「det@0.5 有 90%，为什么串级只有一半」——IoU 0.5 对字符框是个很松的门限。
    iou_bk, acc_bk, acc_gt, pos_bk = [], [], [], []
    # 选框质量：每个规则选出来的那批框，有多少个真落在某个真值字符上（精确率），
    # 真值字符有多少被覆盖到（召回率）。用来区分「选多了杂框」和「把真字丢了」。
    selstat = {r: dict(n = 0, kept = 0, prec_hit = 0, gt = 0, gt_hit = 0, ex = 0)
               for r in rules}
    examples = []
    t0 = time.time()

    for s in range(0, len(paths), args.batch):
        chunk = paths[s:s + args.batch]
        imgs, raws, gts = [], [], []
        for p in chunk:
            img = cv2.imread(p)
            if img is None:
                continue
            lab = np.array([[float(x) for x in ln.split()]
                            for ln in open(p.split(".")[0] + ".txt") if len(ln.split()) == 5],
                           np.float32)
            g = lab[lab[:, 0] > 0] if len(lab) else np.zeros((0, 5), np.float32)
            gorder = np.argsort(g[:, 1]) if len(g) else np.zeros(0, int)
            gts.append((img, g[gorder] if len(g) else g,
                        "".join(names[int(c)] if 0 <= int(c) < len(names) else "?" for c in g[gorder, 0])))
            imgs.append(cv2.resize(img, (W, W), interpolation = cv2.INTER_LINEAR))
            raws.append(p)

        if not imgs:
            continue
        xb = torch.from_numpy(np.stack(imgs)).permute(0, 3, 1, 2).float() / 255.0
        with torch.no_grad():
            out = U.handel_preds(net(xb.to(dev)), cfg, dev)
            dets = U.non_max_suppression(out, conf_thres = args.conf, iou_thres = args.nms)

        for i in range(len(raws)):
            img, g, gstr = gts[i]
            H, Wi = img.shape[:2]
            d = dets[i]
            if hasattr(d, "numpy"):
                d = d.numpy()
            d_all = d
            if len(d) and not args.keep_plate:
                d = d[d[:, 5] != 0]           # 裁剪版模型：class 0 头是噪声，丢掉
            n_cls0 += int((d_all[:, 5] == 0).sum()) if len(d_all) else 0
            # 320 空间 -> 裁剪图原生像素
            dpx = d[:, :4] * np.array([Wi / W, H / W, Wi / W, H / W], np.float32) if len(d) else np.zeros((0, 4), np.float32)
            dconf = d[:, 4] if len(d) else np.zeros(0, np.float32)
            dname = [names[int(c)] if 0 <= int(c) < len(names) else "?" for c in d[:, 5]] if len(d) else []

            # 检测召回（不参与装配）：每个真值字符框有没有 IoU>=0.5 的预测。
            # 标签是归一化的，×320 就和 d[:, :4] 同空间（datasets.py 是纯拉伸，见其 __getitem__）。
            if len(g):
                gt_net = np.stack([(g[:, 1] - g[:, 3] / 2) * W, (g[:, 2] - g[:, 4] / 2) * W,
                                   (g[:, 1] + g[:, 3] / 2) * W, (g[:, 2] + g[:, 4] / 2) * W], 1)
                for k in range(len(gt_net)):
                    n_gt_char += 1
                    if len(d) and max(iou_pair(gt_net[k], d[j, :4]) for j in range(len(d))) >= MIOU:
                        n_hit += 1
            else:
                gt_net = np.zeros((0, 4), np.float32)

            # 真值框的裁剪图像素坐标（GT 规则用）
            gtpx = gt_net * np.array([Wi / W, H / W, Wi / W, H / W], np.float32)

            # ---- 诊断：框质量 -> 识别率
            if len(g) and len(d):
                gcls = [int(c) for c in g[:, 0]]
                pid, ious = [], []
                for k in range(len(g)):
                    js = [iou_pair(gt_net[k], d[t, :4]) for t in range(len(d))]
                    j = int(np.argmax(js))
                    pid.append(j)
                    ious.append(js[j])
                ptsA = [cut_patch(img, gtpx[k], args.pad, (P_W, P_H)) for k in range(len(g))]
                ptsB = [cut_patch(img, dpx[j], args.pad, (P_W, P_H)) for j in pid]
                both = [p for p in ptsA + ptsB if p is not None]
                pp = None
                if both:
                    bb = torch.from_numpy(np.stack(both)).permute(0, 3, 1, 2).float() / 255.0
                    with torch.no_grad():
                        pp = probe(bb.to(dev)).argmax(1).numpy()
                if pp is not None:
                    it = iter(pp)

                    def _pop(pt, k):
                        return int(next(it)) == gcls[k] if pt is not None else False
                    okA = [_pop(ptsA[k], k) for k in range(len(g))]
                    okB = [_pop(ptsB[k], k) for k in range(len(g))]
                    for k in range(len(g)):
                        acc_gt.append(okA[k])
                        iou_bk.append(ious[k])
                        acc_bk.append(okB[k])
                        pos_bk.append((k, ious[k]))   # k = 从左到右第几个字符，0 就是省份字

            for r in rules:
                if r == "GT":
                    boxes = gtpx
                    nsel = len(boxes)
                    chars_det = [None] * nsel
                    conf_d = np.ones(nsel, np.float32)
                else:
                    if not len(d):
                        for key in ("prb", "det"):
                            acc[r][key][3] += 1
                        continue
                    sel = select_rule(r, d[:, :4], dconf, len(g))
                    boxes = dpx[sel]
                    nsel = len(boxes)
                    chars_det = [dname[j] for j in sel]
                    conf_d = dconf[sel]

                if nsel == 0:
                    for key in ("prb", "det"):
                        acc[r][key][3] += 1
                    continue

                # 探针分类
                patches = [cut_patch(img, boxes[j], args.pad, (P_W, P_H)) for j in range(nsel)]
                good = [j for j, pt in enumerate(patches) if pt is not None]
                chars_prb = [None] * nsel
                if good:
                    pb = torch.from_numpy(np.stack([patches[j] for j in good])).permute(0, 3, 1, 2).float() / 255.0
                    with torch.no_grad():
                        pr = probe(pb.to(dev)).argmax(1).numpy()
                    for k, j in enumerate(good):
                        chars_prb[j] = names[int(pr[k])] if 0 <= int(pr[k]) < len(names) else "?"

                # 选框质量：选出来的框有多少真落在真值字符上 / 真值字符被覆盖多少（一一匹配）
                if r != "GT" and len(g):
                    ss = selstat[r]
                    ss["n"] += 1
                    ss["kept"] += nsel
                    used = set()
                    for j in range(nsel):
                        cand = [k for k in range(len(gtpx)) if k not in used]
                        if not cand:
                            break
                        bj = max(cand, key = lambda k: iou_pair(boxes[j], gtpx[k]))
                        if iou_pair(boxes[j], gtpx[bj]) >= MIOU:
                            used.add(bj)
                            ss["prec_hit"] += 1
                    ss["gt"] += len(gtpx)
                    ss["gt_hit"] += len(used)

                cx = (boxes[:, 0] + boxes[:, 2]) / 2.0
                for key, cs in (("prb", chars_prb), ("det", chars_det)):
                    sp = assemble(cx, cs)
                    if key == "prb" and r == "D" and sp != gstr and len(examples) < 24:
                        examples.append((gstr, sp))
                    a = acc[r][key]
                    if gstr:
                        if sp == gstr:
                            a[2] += 1
                        for x, y in zip(gstr, sp[:len(gstr)]):
                            a[1] += 1
                            if x == y:
                                a[0] += 1
                        a[3] += 1

    print("检测召回（类别无关，IoU>=0.5）%d/%d = %.2f%%   （对照：整图版旧数 61.3）"
          % (n_hit, n_gt_char, 100.0 * n_hit / max(n_gt_char, 1)))
    print("conf=%.2f 时判成 class 0(plate) 的框：%.2f 个/图   keep_plate=%s"
          % (args.conf, 1.0 * n_cls0 / max(len(paths), 1), args.keep_plate))
    print()
    print("  %-34s %10s %10s" % ("装配规则 / 分类头", "逐字%", "整串exact%"))
    lbl = {"GT": "GT 真值框（检测完美）", "A": "A 固件原样",
           "B": "B 只留最高分 7 个", "C": "C 类无关 NMS(0.30)",
           "D": "D NMS(0.30)+留分最高 7 个", "E2": "E2 NMS(0.30)+留真值字符数个",
           "F": "F D+高度中位滤除", "G": "G NMS(0.15)",
           "E": "E oracle 只留最高分 真值数个"}
    for r in rules:
        # GT 那一行只有探针头有意义：真值框上没有「检测器的分类」这回事（没有对应的预测框），
        # 那个格子会把 None 拼成空串再报 0.00%，是个假数字，别打出来。
        keys = (("prb", "探针头(全局池化)"),) if r == "GT" else \
               (("prb", "探针头(全局池化)"), ("det", "检测器头(1x1 conv)"))
        for key, nm in keys:
            a = acc[r][key]
            if a[3] == 0:
                continue
            print("  %-34s %9.2f%% %11.2f%%   (n=%d)"
                  % ("%s + %s" % (lbl[r], nm), 100.0 * a[0] / max(a[1], 1),
                     100.0 * a[2] / max(a[3], 1), a[3]))
    print("\n  --- 选出来的框质量（IoU>=0.5 一一匹配；GT 框那行恒为 100/100）---")
    print("      %-30s %8s %8s %8s %8s" % ("规则", "框数/图", "精确率", "召回率", "杂框/图"))
    for r in rules:
        ss = selstat[r]
        if not ss["n"]:
            continue
        kept = ss["kept"] / ss["n"]
        pr = 100.0 * ss["prec_hit"] / max(ss["kept"], 1)
        rc = 100.0 * ss["gt_hit"] / max(ss["gt"], 1)
        print("      %-30s %8.2f %7.2f%% %7.2f%% %8.2f"
              % (lbl[r], kept, pr, rc, kept - ss["prec_hit"] / ss["n"]))

    if examples:
        print("\n  --- 规则 D + 探针头 的错例（真值 | 预测，前 %d 个）---" % len(examples))
        for g, s in examples:
            print("      %-10s | %s" % (g, s))

    if iou_bk:
        print("\n  --- 识别器需要多准的框？真值字符框按「最佳预测框 IoU」分桶后的探针逐字率 ---")
        for lo, hi in ((0.0, 0.3), (0.3, 0.5), (0.5, 0.6), (0.6, 0.7),
                       (0.7, 0.8), (0.8, 0.9), (0.9, 1.01)):
            m = [i for i, v in enumerate(iou_bk) if lo <= v < hi]
            if not m:
                continue
            print("      IoU %.1f~%.1f  n=%4d  探针逐字 %6.2f%%"
                  % (lo, hi, len(m), 100.0 * sum(acc_bk[i] for i in m) / len(m)))
        print("      （对照）同一批图改用**真值框**切图  n=%d  探针逐字 %.2f%%"
              % (len(acc_gt), 100.0 * sum(acc_gt) / max(len(acc_gt), 1)))
        print("      注：IoU<0.5 档里还有「压根没有预测框」的真值字符，它们被算成全错。")

    if pos_bk:
        print("\n  --- 按字符位置看（位置 0 = 省份字）---")
        print("      %-6s %6s %10s %12s" % ("位置", "样本数", "中位 IoU", "IoU>=0.5 占比"))
        for k in range(8):
            v = [i for (p, i) in pos_bk if p == k]
            if len(v) < 20:
                continue
            print("      %-6d %6d %10.3f %11.2f%%"
                  % (k, len(v), float(np.median(v)), 100.0 * np.mean([x >= 0.5 for x in v])))

    print("\n  用时 %.0fs" % (time.time() - t0))


if __name__ == "__main__":
    main()
