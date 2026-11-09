# -*- coding: utf-8 -*-
"""逐行复刻固件的检测后处理，量「真的烧进去之后会得到什么」。

为什么需要这个：`cascade_ceiling.py` 的规则 A 和固件 `yolo_plate_read()` 是同一个东西，
但它用 `utils.non_max_suppression()` 走的是**全量 anchor 按分数**排序的路，
而固件 `decode_head()`（yolo_inference.cpp:88-127）是**光栅顺序**扫，
且外层循环带 `cnt < max_box` 的截断：

    for (gy = 0; gy < gh && cnt < max_box; gy++)
      for (gx = 0; gx < gw && cnt < max_box; gx++)
        for (a = 0; a < YOLO_OBJ_NUM; a++) {         // 内层没有 cnt 检查
          if (obj*cls < YOLO_CONF_THRESH) continue;
          boxes[cnt++] = ...;
        }

两个 head 共用一个 cnt 和 max_box（第 3088 行同一次调用传进去），
main.c 给的 max_box 是 64。**候选数一旦到 64 就整个跳出**，
丢的是「光栅扫描靠后」的框，不是分数低的框 —— 和按分数截断完全不是一件事。

所以：降 conf 门限之前，必须先知道 64 这个预算够不够。
`_count_cand.py` 量过候选数（conf 0.25 时中位 72，已超），
这个脚本把后果算出来：逐字 / 整串 / 位置 0 召回，一行一个配置。

用法：
  python fw_emulate.py --data data_crop/voc.data --split val
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

import utils.utils as U              # noqa: E402
import model.detector as MD          # noqa: E402
from cascade_ceiling import load_names, assemble, iou_pair, _geo_filter   # noqa: E402

MIOU = 0.5
YOLO_CHAN = 85


def fw_decode(prod, cid, thr, max_box, n_layers, grids):
    """复刻 decode_head()：光栅顺序、外层截断、内层不截断。

    prod[i] = obj*max(cls)，cid[i] = argmax cls，下标 i 已经是 (layer, gy, gx, anchor)
    的光栅顺序 —— handel_preds 的 anchor_boxes 是 (h,w,anchor,C) 再 view(-1,C)，
    层序也和 TFLite 输出序一致（[20,20,85] 在前），所以下标序就是固件的扫描序。
    """
    off = 0
    cnt = 0
    for sid in range(n_layers):
        gh, gw = grids[sid]
        for gy in range(gh):
            if cnt >= max_box:
                break
            for gx in range(gw):
                if cnt >= max_box:
                    break
                base = off + (gy * gw + gx) * 3
                for a in range(3):                 # 内层：固件这里没有 cnt 检查
                    n = base + a
                    if prod[n] < thr:
                        continue
                    yield n, float(prod[n]), int(cid[n])
                    cnt += 1
        off += gh * gw * 3


def fw_nms(cands, thr):
    """复刻 nms()：按分数降序插入排序，然后类无关贪心，iou > thr 才算重叠。"""
    c = sorted(cands, key = lambda z: -z[1])
    keep = []
    for z in c:
        if all(iou_pair(z[0], k[0]) <= thr for k in keep):
            keep.append(z)
    return keep


def fw_plate_read(keep, names, cap = 12, geo = False):
    """复刻 yolo_plate_read()：分数序取前 16 个 class>0 -> 可选几何过滤 -> 按 x 拼。"""
    idx = []
    for i in range(len(keep)):
        if keep[i][2] > 0:
            idx.append(i)
        if len(idx) >= 16:
            break
    if geo and idx:
        conf = np.zeros(len(keep), np.float32)
        bpx = np.zeros((len(keep), 4), np.float32)
        for i, z in enumerate(keep):
            conf[i] = z[1]
            bpx[i] = z[0]
        idx = list(_geo_filter(np.asarray(idx, int), bpx, conf))
    idx.sort(key = lambda i: keep[i][0][0])        # 按 x1 升序
    chars = [names[keep[i][2]] if 0 <= keep[i][2] < len(names) else "?" for i in idx]
    return assemble(np.asarray([keep[i][0][0] for i in idx], np.float32), chars, cap)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default = "data_crop/voc.data")
    ap.add_argument("--split", default = "val")
    ap.add_argument("--weights", default = "weights/voc_crop-90-epoch-0.477014ap-model.pth")
    ap.add_argument("--batch", type = int, default = 32)
    ap.add_argument("--limit", type = int, default = 0)
    ap.add_argument("--confs", default = "0.30,0.25,0.20,0.15,0.10")
    ap.add_argument("--nmses", default = "0.45,0.30")
    ap.add_argument("--maxboxes", default = "64,1024")
    args = ap.parse_args()

    cfg = U.load_datafile(args.data)
    names = load_names(cfg["names"])
    classes = int(cfg["classes"])
    W = int(cfg["width"])
    paths = [ln.strip() for ln in open(cfg[args.split]) if ln.strip()]
    if args.limit:
        paths = paths[:args.limit]
    paths = [p for p in paths if os.path.exists(p.split(".")[0] + ".txt")]

    confs = [float(v) for v in args.confs.split(",")]
    nmses = [float(v) for v in args.nmses.split(",")]
    maxboxes = [int(v) for v in args.maxboxes.split(",")]

    print("=" * 104)
    print("### 固件后处理逐行复刻  %s %s %d 张  weights=%s"
          % (args.data, args.split, len(paths), os.path.basename(args.weights)))
    print("=" * 104)

    dev = torch.device("cpu")
    net = MD.Detector(classes, int(cfg["anchor_num"]), True).to(dev)
    net.load_state_dict(torch.load(args.weights, map_location = dev))
    net.eval()

    cfgs = [(c, n, m, g) for c in confs for n in nmses for m in maxboxes for g in (False, True)]
    acc = {k: dict(ok = 0, tot = 0, ex = 0, n = 0, boxes = 0, hit = 0, gt = 0,
                   pos_hit = {}, pos_n = {}) for k in cfgs}
    t0 = time.time()

    for s in range(0, len(paths), args.batch):
        imgs, metas = [], []
        for p in paths[s:s + args.batch]:
            im = cv2.imread(p)
            if im is None:
                continue
            lab = np.array([[float(x) for x in ln.split()]
                            for ln in open(p.split(".")[0] + ".txt") if len(ln.split()) == 5],
                           np.float32)
            g = lab[lab[:, 0] > 0] if len(lab) else np.zeros((0, 5), np.float32)
            gsort = g[np.argsort(g[:, 1])] if len(g) else g
            gstr = "".join(names[int(c)] if 0 <= int(c) < len(names) else "?" for c in gsort[:, 0])
            # 必须存 **gsort**：位置轴是按 x 排的，存未排序的 g 会让「位置 0」变成
            # 「标签文件里的第一个框」，逐位置那一列就全是噪声（之前就是这么错的）。
            metas.append((gsort, gstr))
            imgs.append(cv2.resize(im, (W, W), interpolation = cv2.INTER_LINEAR))
        if not imgs:
            continue
        xb = torch.from_numpy(np.stack(imgs)).permute(0, 3, 1, 2).float() / 255.0
        with torch.no_grad():
            raw = U.handel_preds(net(xb.to(dev)), cfg, dev).cpu().numpy()

        # 21 个 anchor/cell 的话层序就是 TFLite 的输出序，和固件 decode_head 的调用序一致
        n_layer = 2
        grids = [(20, 20), (10, 10)] if raw.shape[1] == 1500 else [(40, 40), (20, 20)]
        prod_all = raw[:, :, 4] * raw[:, :, 5:].max(2)          # (B, n_anchor)
        cid_all = raw[:, :, 5:].argmax(2)                       # (B, n_anchor)
        cx, cy, bw, bh = raw[:, :, 0], raw[:, :, 1], raw[:, :, 2], raw[:, :, 3]
        xyxy = np.stack([cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2], -1)

        for i, (g, gstr) in enumerate(metas):
            gtnet = np.stack([(g[:, 1] - g[:, 3] / 2) * W, (g[:, 2] - g[:, 4] / 2) * W,
                              (g[:, 1] + g[:, 3] / 2) * W, (g[:, 2] + g[:, 4] / 2) * W], 1) \
                if len(g) else np.zeros((0, 4), np.float32)
            for key in cfgs:
                c_thr, n_thr, mbox, geo = key
                cands = [(xyxy[i, n], sc, cid) for n, sc, cid in
                         fw_decode(prod_all[i], cid_all[i], c_thr, mbox, n_layer, grids)]
                keep = fw_nms(cands, n_thr)
                pred = fw_plate_read(keep, names, 12, geo)
                a = acc[key]
                a["n"] += 1
                a["boxes"] += len(keep)
                if len(g):
                    a["gt"] += len(gtnet)
                    for k in range(len(gtnet)):
                        a["pos_n"][k] = a["pos_n"].get(k, 0) + 1
                        if len(keep) and max(iou_pair(gtnet[k], z[0]) for z in keep) >= MIOU:
                            a["hit"] += 1
                            a["pos_hit"][k] = a["pos_hit"].get(k, 0) + 1
                if pred == gstr:
                    a["ex"] += 1
                for x, y in zip(gstr, pred[:len(gstr)]):
                    a["tot"] += 1
                    if x == y:
                        a["ok"] += 1

    print("\n%-6s %-5s %-7s %-4s | %8s %8s %7s %8s %8s %9s"
          % ("conf", "nms", "maxbox", "geo", "逐字%", "整串%", "框/图", "召回%", "位置0%", "位置1-6%"))
    print("-" * 104)
    for key in cfgs:
        a = acc[key]
        p0 = 100.0 * a["pos_hit"].get(0, 0) / max(a["pos_n"].get(0, 1), 1)
        rest_h = sum(v for k, v in a["pos_hit"].items() if k >= 1)
        rest_n = sum(v for k, v in a["pos_n"].items() if k >= 1)
        # conf×nms×maxbox 太多行，geo off 和 on 并排更好读，所以两两成对打印
        print("%-6.2f %-5.2f %-7d %-4s | %7.2f%% %7.2f%% %7.2f %7.2f%% %7.2f%% %8.2f%%"
              % (key[0], key[1], key[2], "on" if key[3] else "off",
                 100.0 * a["ok"] / max(a["tot"], 1),
                 100.0 * a["ex"] / max(a["n"], 1),
                 a["boxes"] / max(a["n"], 1),
                 100.0 * a["hit"] / max(a["gt"], 1),
                 p0,
                 100.0 * rest_h / max(rest_n, 1)))
    print("\n用时 %.0fs" % (time.time() - t0))
    print("注：'召回%' / '位置k%' 都是类别无关 IoU>=0.5、**NMS 之后**的存活率；")
    print("    '位置0%' 就是省份字，它是 cascade_ceiling 里那个 51.56% 的同一个量。")


if __name__ == "__main__":
    main()
