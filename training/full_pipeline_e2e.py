# -*- coding: utf-8 -*-
"""真·全链路：整图 → 找车牌 → 裁剪 → 裁剪版模型认字 → 第二级识别头。

`two_stage_e2e.py` 量的是「**已经裁好**车牌之后」的两级，它的每一张输入图
都来自 `make_plate_crops.py` 用**真值**车牌框裁出来的 `data_crop`。
真机上第一级得**自己**找车牌，裁出来的框不可能和真值框一模一样 ——
这一段的代价一直没人量过（报告 §3.7 写的那个「唯一可能变差」的环节）。

这个脚本把它补上：**车牌框来自整图模型的预测，不是标注**。

流水线（逐段都能和前面的脚本对上）：
  1. 整图模型（`data/voc.data` + voc-299 权重）→ 固件后处理 → 取 class 0 = plate
     → 按面积取最大的那块（和 `make_plate_crops.best_plate` 同一条规则）
  2. 车牌框**外扩 margin（默认 0.10）** → 裁原图 → resize 320×320
     —— 这一步逐行复刻 `make_plate_crops.process`，所以 `--gt-plate` 模式
     （车牌框用真值）**应该复现 §3.5 的数字**，那是本脚本的自对拍。
  3. 裁剪版模型（`data_crop/voc.data` + voc_crop 权重）→ 固件后处理 → 字符框
  4. 原生分辨率抠字符 → 第二级识别头 argmax → 按 x 拼串

真值串同理走 `make_plate_crops` 的规则：最大的车牌框内部的字符（class>0、中心在框内）。

用法：
  python full_pipeline_e2e.py --gt-plate          # 自对拍：应复现 §3.5
  python full_pipeline_e2e.py                     # 真预测车牌框，这才是要看的数
  python full_pipeline_e2e.py --confs 0.05,0.10 --maxchars 0,7 --lendump
"""

import sys
sys.dont_write_bytecode = True

import os
import time
import argparse
import numpy as np
import cv2
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
os.chdir(HERE)

import utils.utils as U                                   # noqa: E402
import model.detector as MD                               # noqa: E402
from cascade_ceiling import load_names, assemble, cut_patch   # noqa: E402
from fw_emulate import fw_decode, fw_nms                  # noqa: E402
from two_stage_e2e import select_idx, load_probe          # noqa: E402
from train_char_head import SmallCharNet                  # noqa: E402


def read_labels(path):
    """labels[N,5] = [cls cx cy w h]，归一化到原图；没有文件返回 None。"""
    lp = path.split(".")[0] + ".txt"
    if not os.path.exists(lp):
        return None
    rows = []
    for ln in open(lp):
        p = ln.split()
        if len(p) == 5:
            rows.append([float(x) for x in p])
    return np.array(rows, np.float32) if rows else None


def gt_plate_and_str(labels, names, W, H):
    """复刻 make_plate_crops：取面积最大的 class 0 框；中心落在框内的字符按 x 拼串。"""
    if labels is None:
        return None, None
    pi = np.nonzero(labels[:, 0] == 0)[0]
    if not len(pi):
        return None, None
    pi = int(pi[(labels[pi, 3] * labels[pi, 4]).argmax()])
    cx, cy, bw, bh = labels[pi, 1], labels[pi, 2], labels[pi, 3], labels[pi, 4]
    px0, py0 = (cx - bw / 2) * W, (cy - bh / 2) * H
    px1, py1 = (cx + bw / 2) * W, (cy + bh / 2) * H
    tx, ty = (px1 - px0) * 0.02, (py1 - py0) * 0.02
    # 和 make_plate_crops 一样：字符中心要落在**车牌框**内（不是外扩后的裁剪框内），
    # 否则漏标的那第二块车牌的字符会混进来。
    g = []
    for r in labels:
        if r[0] <= 0:
            continue
        ecx, ecy = r[1] * W, r[2] * H
        if px0 - tx <= ecx <= px1 + tx and py0 - ty <= ecy <= py1 + ty:
            g.append((ecx, names[int(r[0])] if 0 <= int(r[0]) < len(names) else "?"))
    g.sort(key = lambda z: z[0])
    return np.array([px0, py0, px1, py1], np.float32), assemble(
        np.array([z[0] for z in g], np.float32), [z[1] for z in g], 12)


def crop_by_plate(img, box_px, margin):
    """裁车牌区域。**逐行复刻 make_plate_crops.process** 的取整方式
    （floor 左上、ceil 右下、clip 到图内），否则 --gt-plate 模式对不上 §3.5。"""
    H, W = img.shape[:2]
    px0, py0, px1, py1 = box_px
    mx, my = (px1 - px0) * margin, (py1 - py0) * margin
    x0 = int(np.clip(np.floor(px0 - mx), 0, W - 1))
    y0 = int(np.clip(np.floor(py0 - my), 0, H - 1))
    x1 = int(np.clip(np.ceil(px1 + mx), x0 + 1, W))
    y1 = int(np.clip(np.ceil(py1 + my), y0 + 1, H))
    return img[y0:y1, x0:x1], (x1 - x0), (y1 - y0)


def pick_plate(keep, rule = "area", conf_margin = 0.20):
    """在 NMS 之后的框里挑一块车牌（只看 class 0）。返回 keep 里的索引，没有就 None。

    三条规则（真值框上三条**完全等价** —— 一帧一块真车牌，所以 `--gt-plate` 自对拍不受影响）：

    `rule="area"`（默认）：**面积最大**。这是训练期 `make_plate_crops.best_plate` 的规则，
      也是 §3.5 / §3.6 / §3.7 / v1.0 那些数字用的规则。
    `rule="score"`：**分数最高**。
    `rule="area_conf"`：**先扔掉「比最高分低 `conf_margin` 以上」的框，再取面积最大**。

    ⚠️ 2026-10-16 在 `img/` 的真实手机照片上发现 `area` 会被骗：
    09.jpg 真车牌 score **0.993**/面积 7794，一块地面 score **0.378**/面积 10222；
    06.jpg 同样 1.000/9127 输给 0.145/12929。两条都把地面裁下来当车牌。
    **但在 val（= 训练相机那批）上 `area` 明显赢过 `score`**（整串 76.85% vs 71.01%，ft119 前后同向），
    所以不能简单换成 `score` —— `area_conf` 是想两头都要：val 上它和 `area` 几乎重合
    （竞争框分数差通常远小于 0.20），img/ 那两张上它把地面框先筛掉。
    """
    c0 = [i for i, z in enumerate(keep) if z[2] == 0]
    if not c0:
        return None
    if rule == "score":
        return max(c0, key = lambda i: keep[i][1])
    if rule == "area_conf":
        smax = max(keep[i][1] for i in c0)
        keep_in = [i for i in c0 if keep[i][1] >= smax - conf_margin]
        c0 = keep_in or c0
    return max(c0, key = lambda i: (keep[i][0][2] - keep[i][0][0]) *
                                   (keep[i][0][3] - keep[i][0][1]))


def decode_heads(net, cfg, xb, dev):
    """一次前向 -> (prod, cid, xyxy, n_layer, grids)。xyxy 是网络空间 cx±w/2。"""
    with torch.no_grad():
        raw = U.handel_preds(net(xb.to(dev)), cfg, dev).cpu().numpy()
    n_layer = 2
    grids = [(20, 20), (10, 10)] if raw.shape[1] == 1500 else [(40, 40), (20, 20)]
    prod = raw[:, :, 4] * raw[:, :, 5:].max(2)
    cid = raw[:, :, 5:].argmax(2)
    cx, cy, bw, bh = raw[:, :, 0], raw[:, :, 1], raw[:, :, 2], raw[:, :, 3]
    xyxy = np.stack([cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2], -1)
    return prod, cid, xyxy, n_layer, grids


def blank():
    return dict(zh = {}, zt = {}, ex = {}, pos = {}, pn = {}, n = 0, nbr = 0,
                dlen = {}, noplate = 0, exgt = {}, okgt = {})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default = "data/voc.data", help = "整图（找车牌）那份配置")
    ap.add_argument("--crop-data", default = "data_crop/voc.data", help = "裁剪版那份配置")
    ap.add_argument("--split", default = "val")
    ap.add_argument("--plate-weights",
                    default = "weights/voc-299-epoch-0.270855ap-model.pth")
    ap.add_argument("--crop-weights",
                    default = "weights/voc_crop-90-epoch-0.477014ap-model.pth")
    ap.add_argument("--charhead", default = "weights/charhead_16_32_64_32_64.pt")
    ap.add_argument("--probe", default = "none")
    ap.add_argument("--margin", type = float, default = 0.10,
                    help = "车牌框外扩比例，和 make_plate_crops 默认一致")
    ap.add_argument("--plate-conf", type = float, default = 0.25)
    ap.add_argument("--plate-nms", type = float, default = 0.45)
    ap.add_argument("--plate-maxbox", type = int, default = 64)
    ap.add_argument("--plate-pick", default = "area", choices = ["area", "score", "area_conf"],
                    help = "① 挑车牌框的规则：area=面积最大（默认，= 训练期 best_plate）；"
                           "score=分数最高；area_conf=先扔掉低分框再取面积最大（见 pick_plate 注释）")
    ap.add_argument("--plate-conf-margin", type = float, default = 0.20,
                    help = "只对 --plate-pick area_conf 有效")
    ap.add_argument("--confs", default = "0.10")
    ap.add_argument("--nmses", default = "0.30")
    ap.add_argument("--maxbox", type = int, default = 64)
    ap.add_argument("--maxchars", default = "")
    ap.add_argument("--batch", type = int, default = 16)
    ap.add_argument("--limit", type = int, default = 0)
    ap.add_argument("--gt-plate", action = "store_true",
                    help = "自对拍：车牌框用**真值**（等于重做一遍 make_plate_crops+§3.5）")
    ap.add_argument("--lendump", action = "store_true")
    args = ap.parse_args()

    dev = torch.device("cpu")
    names = load_names("data/voc.names")

    pcfg = U.load_datafile(args.data)
    ccfg = U.load_datafile(args.crop_data)
    pnet_ = MD.Detector(int(pcfg["classes"]), int(pcfg["anchor_num"]), True).to(dev)
    pnet_.load_state_dict(torch.load(args.plate_weights, map_location = dev))
    pnet_.eval()
    cnet = MD.Detector(int(ccfg["classes"]), int(ccfg["anchor_num"]), True).to(dev)
    cnet.load_state_dict(torch.load(args.crop_weights, map_location = dev))
    cnet.eval()

    heads = []
    if os.path.exists(args.charhead):
        ck = torch.load(args.charhead, map_location = dev, weights_only = False)
        P_W, P_H = ck["size_wh"]
        h = SmallCharNet(int(ck["classes"]), (P_H, P_W), *ck["c"]).to(dev)
        h.load_state_dict(ck["state_dict"])
        h.eval()
        heads.append(("小头", h, P_W, P_H, ck["pad"]))
    else:
        print("!! 没找到第二级权重 %s —— 只跑一级那一列" % args.charhead)
    if args.probe != "none" and os.path.exists(args.probe):
        pnet2, PW, PH = load_probe(args.probe, int(ccfg["classes"]), dev)
        heads.append(("探针", pnet2, PW, PH, 0.25))

    paths = [ln.strip() for ln in open(pcfg[args.split]) if ln.strip()]
    if args.limit:
        paths = paths[:args.limit]

    confs = [float(v) for v in args.confs.split(",")]
    nmses = [float(v) for v in args.nmses.split(",")]
    mcs = [int(v) for v in args.maxchars.split(",")] if args.maxchars else [0]
    cfgs = [(a, b, m) for a in confs for b in nmses for m in mcs]
    hnames = [nm for nm, _, _, _, _ in heads]

    acc = {k: blank() for k in cfgs}
    for a in acc.values():
        for hk in ["一级"] + hnames:
            a["zh"][hk] = 0; a["zt"][hk] = 0; a["ex"][hk] = 0
            a["pos"][hk] = {}; a["dlen"][hk] = {}
            # 按真值串长分桶：全体平均会被 6/8 字那几桶（真值少标字）摊平，
            # 部署场景看的是 7 字那一桶 —— 见报告 §3.8。
            a["exgt"][hk] = {}; a["okgt"][hk] = {}

    PW_CH = int(ccfg["width"])
    t0 = time.time()
    plates_found = 0
    plate_iou = []
    n_eval = 0

    for s in range(0, len(paths), args.batch):
        batch = []
        for p in paths[s:s + args.batch]:
            im = cv2.imread(p)
            if im is None:
                continue
            H, W = im.shape[:2]
            lab = read_labels(p)
            gp, gstr = gt_plate_and_str(lab, names, W, H)
            if gstr is None or gstr == "":
                continue                       # 没有车牌标注的图不参与（整图 val 里是 3 张）
            batch.append((p, im, W, H, gp, gstr))
        if not batch:
            continue

        # --- 第 1 段：整图模型找车牌 ---
        if args.gt_plate:
            prod = cid = xyxy = None            # 自对拍模式不需要整图模型，省一次前向
            nl, grids = 2, [(20, 20), (10, 10)]
        else:
            xb = np.stack([cv2.resize(im, (int(pcfg["width"]), int(pcfg["height"])),
                                      interpolation = cv2.INTER_LINEAR)
                           for _, im, _, _, _, _ in batch])
            xb = torch.from_numpy(xb).permute(0, 3, 1, 2).float() / 255.0
            prod, cid, xyxy, nl, grids = decode_heads(pnet_, pcfg, xb, dev)

        crops = []                             # 每张图的裁剪 + 尺寸 + 诊断
        for i, (p, im, W, H, gp, gstr) in enumerate(batch):
            if args.gt_plate:
                box = gp
            else:
                cands = [(xyxy[i, n], sc, cc) for n, sc, cc in
                         fw_decode(prod[i], cid[i], args.plate_conf,
                                   args.plate_maxbox, nl, grids)]
                keep = fw_nms(cands, args.plate_nms)
                j = pick_plate(keep, args.plate_pick, args.plate_conf_margin)
                box = None
                if j is not None:
                    # 网络空间是 320×320 的归一化正方形，换回原图像素
                    sx, sy = W / float(pcfg["width"]), H / float(pcfg["height"])
                    box = np.array([keep[j][0][0] * sx, keep[j][0][1] * sy,
                                    keep[j][0][2] * sx, keep[j][0][3] * sy], np.float32)
            if box is None:
                crops.append(None)             # 这一张：没找到车牌，整条链失败
                continue
            crops.append((box, im))

        # 车牌框和真值的 IoU（只有非 --gt-plate 才有意义）
        for i, (p, im, W, H, gp, gstr) in enumerate(batch):
            if crops[i] is None:
                continue
            a, b = crops[i][0], gp
            lt = np.maximum(a[:2], b[:2]); rb = np.minimum(a[2:], b[2:])
            wh = np.clip(rb - lt, 0, None)
            inter = wh[0] * wh[1]
            un = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
            plate_iou.append(inter / un if un > 0 else 0.0)

        # --- 第 2 段：裁剪 -> 320×320 ---
        # crop_native 要留着：第 4 段抠字符必须在**原生分辨率的裁剪图**上做
        # （把字符缩回 320 再放大，第二级就白搭了），而且框是裁剪图局部坐标，
        # 拿原图去抠会抠到完全错的地方 —— 我先这么写错了，小头那列掉到 0.00% 才发现。
        crop_imgs, crop_sz, crop_native = [], [], []
        for c in crops:
            if c is None:
                crop_imgs.append(np.zeros((int(ccfg["height"]), int(ccfg["width"]), 3), np.uint8))
                crop_sz.append(None)
                crop_native.append(None)
                continue
            cc, cw, ch = crop_by_plate(c[1], c[0], args.margin)
            if cc.size == 0:
                crop_imgs.append(np.zeros((int(ccfg["height"]), int(ccfg["width"]), 3), np.uint8))
                crop_sz.append(None)
                crop_native.append(None)
                continue
            crop_imgs.append(cv2.resize(cc, (PW_CH, PW_CH), interpolation = cv2.INTER_LINEAR))
            crop_sz.append((cw, ch))
            crop_native.append(cc)

        # --- 第 3 段：裁剪版模型认字 ---
        cb = torch.from_numpy(np.stack(crop_imgs)).permute(0, 3, 1, 2).float() / 255.0
        prod2, cid2, xyxy2, nl2, grids2 = decode_heads(cnet, ccfg, cb, dev)

        # 所有配置的字符框先挑出来，再一次性抠图送第二级
        jobs = []                              # (img_i, key, box_px, cid, PW, PH, pd)
        keepinfo = {}
        for i, (p, im, W, H, gp, gstr) in enumerate(batch):
            keepinfo[i] = {}
            if crops[i] is None:
                for key in cfgs:
                    keepinfo[i][key] = None
                continue
            cw, ch = crop_sz[i]
            sx, sy = cw / float(PW_CH), ch / float(PW_CH)     # 网络空间 -> 原生裁剪图像素
            for key in cfgs:
                a, b, m = key
                cands = [(xyxy2[i, n], sc, cc) for n, sc, cc in
                         fw_decode(prod2[i], cid2[i], a, args.maxbox, nl2, grids2)]
                if not cands:
                    keepinfo[i][key] = None
                    continue
                keep = fw_nms(cands, b)
                idx = select_idx(keep, False, m)
                if not idx:
                    keepinfo[i][key] = None
                    continue
                bpx = [np.array([keep[j][0][0] * sx, keep[j][0][1] * sy,
                                 keep[j][0][2] * sx, keep[j][0][3] * sy], np.float32)
                       for j in idx]
                keepinfo[i][key] = dict(keep = keep, idx = idx, bpx = bpx,
                                        cid = [keep[j][2] for j in idx])
                for q, bp in enumerate(bpx):
                    for nm, hnet, HPW, HPH, pd in heads:
                        jobs.append((i, key, q, crop_native[i], bp, nm, hnet, HPW, HPH, pd))

        preds2 = {}
        if jobs:
            by_head = {}
            for j in jobs:
                by_head.setdefault(j[5], []).append(j)
            for nm, jl in by_head.items():
                _, _, _, _, _, _, hnet, HPW, HPH, pd = jl[0]
                pt_ = []
                for i, key, q, im_, bp, _, _, _, _, _ in jl:
                    pt = cut_patch(im_, bp, pd, (HPW, HPH))
                    pt_.append(pt if pt is not None else np.zeros((HPH, HPW, 3), np.uint8))
                out = []
                for s2 in range(0, len(pt_), 512):
                    b2 = torch.from_numpy(np.asarray(pt_[s2:s2 + 512])).permute(0, 3, 1, 2).float() / 255.0
                    with torch.no_grad():
                        out.append(hnet(b2.to(dev)).argmax(1).cpu().numpy())
                out = np.concatenate(out)
                for j, lab in zip(jl, out):
                    preds2.setdefault((j[0], j[1], nm), []).append(names[int(lab)])

        # --- 计分 ---
        for i, (p, im, W, H, gp, gstr) in enumerate(batch):
            n_eval += 1
            if crops[i] is not None:
                plates_found += 1
            for key in cfgs:
                a = acc[key]
                a["n"] += 1
                d = keepinfo[i].get(key)
                for hk in ["一级"] + hnames:      # 分桶分母：每张图都要算一次
                    a["exgt"][hk][len(gstr)] = a["exgt"][hk].get(len(gstr), 0) + 1
                if d is None:
                    a["noplate"] += 1
                    for hk in ["一级"] + hnames:
                        a["dlen"][hk][-len(gstr)] = a["dlen"][hk].get(-len(gstr), 0) + 1
                    for k in range(len(gstr)):
                        a["pn"][k] = a["pn"].get(k, 0) + 1
                    continue
                keep, idx = d["keep"], d["idx"]
                a["nbr"] += len(idx)
                cx1 = np.array([keep[j][0][0] for j in idx], np.float32)
                c1 = [names[cc] if 0 <= cc < len(names) else "?" for cc in d["cid"]]
                preds = {"一级": assemble(cx1, c1, 12) if idx else ""}
                for nm in hnames:
                    p2 = preds2.get((i, key, nm))
                    preds[nm] = assemble(cx1, p2, 12) if (idx and p2 and len(p2) == len(cx1)) else ""
                for k in range(len(gstr)):
                    a["pn"][k] = a["pn"].get(k, 0) + 1
                for hk, s in preds.items():
                    for k in range(len(gstr)):
                        a["pos"][hk][k] = a["pos"][hk].get(k, 0) + int(k < len(s) and s[k] == gstr[k])
                    a["dlen"][hk][len(s) - len(gstr)] = a["dlen"][hk].get(len(s) - len(gstr), 0) + 1
                    for x, y in zip(gstr, s[:len(gstr)]):
                        a["zt"][hk] += 1
                        a["zh"][hk] += int(x == y)
                    _ok = int(s == gstr)
                    a["ex"][hk] += _ok
                    a["okgt"][hk][len(gstr)] = a["okgt"][hk].get(len(gstr), 0) + _ok

    hks = ["一级"] + hnames
    print("=" * 104)
    print("### 真·全链路（整图 → 车牌框 → 裁剪 → 裁剪模型 → 第二级）")
    print("  车牌框来源  %s   margin %.2f   plate conf %.2f / nms %.2f   挑框规则=%s"
          % ("**真值**（自对拍）" if args.gt_plate else "整图模型预测",
             args.margin, args.plate_conf, args.plate_nms, args.plate_pick))
    print("  整图模型    %s" % os.path.basename(args.plate_weights))
    print("  裁剪版模型  %s" % os.path.basename(args.crop_weights))
    for nm, _, HPW, HPH, pd in heads:
        print("  二级[%s]    %dx%d pad %.2f" % (nm, HPW, HPH, pd))
    print("  评估 %d 张；检出车牌 %d 张（%.2f%%）；车牌框 IoU 中位 %.3f  （均 %.3f）"
          % (n_eval, plates_found, 100.0 * plates_found / max(n_eval, 1),
             float(np.median(plate_iou)) if plate_iou else float("nan"),
             float(np.mean(plate_iou)) if plate_iou else float("nan")))
    print("=" * 104)

    hdr = "%-6s %-5s %-4s %7s |" % ("conf", "nms", "maxc", "框/图")
    for hk in hks:
        hdr += " %-13s |" % ("%s 逐字/整串" % hk)
    for hk in hks:
        hdr += " %-15s |" % ("%s 位0/位1-6" % hk)
    print("\n" + hdr)
    print("-" * len(hdr))
    for key in cfgs:
        a = acc[key]
        n = max(a["n"], 1)
        line = "%-6.2f %-5.2f %-4s %7.2f |" % (
            key[0], key[1], key[2] if key[2] else "-", a["nbr"] / float(n))
        for hk in hks:
            zt = max(a["zt"][hk], 1)
            line += " %5.2f%%/%5.2f%% |" % (100.0 * a["zh"][hk] / zt, 100.0 * a["ex"][hk] / n)
        p0n = max(a["pn"].get(0, 1), 1)
        rn = sum(v for k, v in a["pn"].items() if k >= 1) or 1
        for hk in hks:
            p0 = 100.0 * a["pos"][hk].get(0, 0) / p0n
            r = 100.0 * sum(v for k, v in a["pos"][hk].items() if k >= 1) / rn
            line += " %6.2f%%/%6.2f%% |" % (p0, r)
        print(line)

    print("\n注：'一级' = 裁剪版检测头自己的 class argmax；其余 = 抠出来送二级头。")
    print("    '没找到车牌' 的图在逐字/整串里按整串全错计（那是真实的部署失败模式）：")
    for key in cfgs:
        print("      conf %.2f maxchars %s -> %d 张" % (key[0], key[2] if key[2] else "-",
                                                        acc[key]["noplate"]))

    if args.lendump:
        for key in cfgs:
            a = acc[key]
            print("\n长度差（预测 - 真值）分布   conf %.2f nms %.2f maxchars %s   共 %d 张"
                  % (key[0], key[1], key[2] if key[2] else "-", a["n"]))
            for hk in hks:
                h = a["dlen"][hk]
                tot = sum(h.values()) or 1
                print("  %-6s 短 %.1f%%  正好 %.1f%%  长 %.1f%%"
                      % (hk, 100.0 * sum(v for d, v in h.items() if d < 0) / tot,
                         100.0 * h.get(0, 0) / tot,
                         100.0 * sum(v for d, v in h.items() if d > 0) / tot))

        # 按真值串长分桶 —— 这才是**部署场景该看的数**（§3.8）：
        # val 是连续切块，6 字牌占 3.5%（train 只占 0.13%），而那 18 张是真值少标了字，
        # 对任何模型都是零希望，却把全体平均往下拽。7 字牌才是 98% 的真实场景。
        for key in cfgs:
            a = acc[key]
            print("\n按**真值串长**分桶（整串率）  conf %.2f nms %.2f maxchars %s"
                  % (key[0], key[1], key[2] if key[2] else "-"))
            print("  %-6s %7s | %s" % ("串长", "张数",
                                       "  ".join("%-16s" % hk for hk in hks)))
            glens = sorted({g for hk in hks for g in a["exgt"][hk]})
            for g in glens:
                n_g = a["exgt"][hks[-1]].get(g, 0)
                cells = []
                for hk in hks:
                    ok_g = a["okgt"][hk].get(g, 0)
                    cells.append("%5.1f%% (%3d/%3d)" % (100.0 * ok_g / max(n_g, 1), ok_g, n_g))
                print("  %-6d %7d | %s" % (g, n_g, "  ".join("%-16s" % c for c in cells)))
            # 按 train 的串长分布重加权（train: 6字 0.13% / 7字 97.96% / 8字 1.88%）
            TW = {6: 0.00135, 7: 0.97962, 8: 0.01885, 11: 0.00019}
            for hk in hks:
                num = den = 0.0
                for g in glens:
                    w = TW.get(g)
                    if w is None:
                        continue
                    n_g = a["exgt"][hk].get(g, 0)
                    num += w * (a["okgt"][hk].get(g, 0) / max(n_g, 1))
                    den += w
                if den > 0:
                    print("  ⇒ %s 按 train 串长分布重加权 = %.2f%%  （val 原始 %.2f%%）"
                          % (hk, 100.0 * num / den, 100.0 * a["ex"][hk] / max(a["n"], 1)))

    print("\n用时 %.0fs" % (time.time() - t0))


if __name__ == "__main__":
    main()
