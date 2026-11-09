# -*- coding: utf-8 -*-
"""端到端：固件后处理（真的那份）→ 抠字符 → 第二级识别头 → 拼串。

前面几个脚本量的是**零件**：`fw_emulate.py` 量一级单独能到多少，
`train_char_head.py` 量识别头在**真值框**上能到多少。这个脚本把两级**串起来**，
量「真的烧进 MCU 会得到什么」—— 这才是要看的数。

刻意做成**同一批框打两种分类头**的 A/B：

  一级头（现状）  框 -> 检测头自己的 class argmax -> 拼串
  两级（方案）    框 -> 抠出来 -> 第二级识别头 argmax -> 拼串

框**完全一样**（同一份 keep 列表），所以差多少就是第二级挣的，
不会被「框选得好不好」混淆。框的筛选逐行复刻 `fw_emulate.fw_plate_read`，
所以口径和固件一致（光栅截断 max_box、类无关 NMS、分数序取前 16 个 class>0、按 x 拼）。

用法：
  python two_stage_e2e.py --confs 0.10,0.15,0.25 --nmses 0.30,0.45
  python two_stage_e2e.py --charhead weights/charhead_8_16_32_32_64.pt
"""

import sys
sys.dont_write_bytecode = True

import os
import re
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
from cascade_ceiling import (load_names, assemble, cut_patch, iou_pair,
                             _geo_filter)                 # noqa: E402
from fw_emulate import fw_decode, fw_nms                  # noqa: E402
from train_char_head import SmallCharNet                  # noqa: E402
from probe_char_cls import CharCNN                        # noqa: E402


def load_probe(path, classes, dev):
    """探针权重是裸 state_dict（`probe_char_cls.py` 只 `torch.save(model.state_dict())`），
    输入尺寸没存进去，从文件名里认（probe_charcls_32x64.pt -> W=32,H=64）。"""
    base = os.path.basename(path)
    m = re.search(r"_(\d+)x(\d+)", base)
    if not m:
        raise ValueError("从 %s 认不出输入尺寸，文件名要带 _WxH" % base)
    W, H = int(m.group(1)), int(m.group(2))
    net = CharCNN(classes, (W, H)).to(dev)
    net.load_state_dict(torch.load(path, map_location = dev))
    net.eval()
    return net, W, H


def select_idx(keep, geo, maxchars = 0):
    """复刻 yolo_plate_read() 的挑框顺序。**改了这里必须同步改 fw_emulate.fw_plate_read。**

    固件 yolo_inference.cpp:325-327 是先从分数序里取前 16 个 class>0，**再**做几何过滤，
    所以这里也是先切 16 再 _geo_filter。最后按 x1 升序排。

    `maxchars` 是**固件现在没有的一步**（默认 0 = 关，等于固件原样）：拼出来的串有多长
    完全由框数决定，所以多一个误检框就必然读错整串。真车牌最多 8 个字
    （训练集：7 字 4366 张、8 字 84 张），所以超过就按分数砍掉最低的那几个。
    ⚠️ 开着它时「一级」那一列**不再**等于 `fw_emulate` —— 那是正常的，因为多了一步。
    """
    idx = []
    for i in range(len(keep)):
        if keep[i][2] > 0:
            idx.append(i)
        if len(idx) >= 16:
            break
    if geo and idx:
        conf = np.array([z[1] for z in keep], np.float32)
        bpx = np.array([z[0] for z in keep], np.float32)
        idx = list(_geo_filter(np.asarray(idx, int), bpx, conf))
    if maxchars and len(idx) > maxchars:
        idx = sorted(idx, key = lambda i: -keep[i][1])[:maxchars]
    idx.sort(key = lambda i: keep[i][0][0])
    return idx


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default = "data_crop/voc.data")
    ap.add_argument("--split", default = "val")
    ap.add_argument("--weights", default = "weights/voc_crop-90-epoch-0.477014ap-model.pth")
    ap.add_argument("--charhead", default = "weights/charhead_8_16_32_32_64.pt")
    ap.add_argument("--probe", default = "weights/probe_charcls_32x64.pt",
                    help = "探针（1.7M 参数，不可部署）——放进来是为了把「框不够好」和"
                           "「头不够好」分开：同一批框上换更好的头还能涨多少")
    ap.add_argument("--batch", type = int, default = 32)
    ap.add_argument("--limit", type = int, default = 0)
    ap.add_argument("--confs", default = "0.25,0.15,0.10")
    ap.add_argument("--nmses", default = "0.45,0.30")
    ap.add_argument("--maxbox", type = int, default = 64)
    ap.add_argument("--geo", action = "store_true", help = "额外跑一遍开了几何过滤的")
    ap.add_argument("--maxchars", default = "",
                    help = "逗号分隔的字数上限（NMS 之后按分数砍到 N 个框）。"
                           "空/0 = 固件原样。真车牌最多 8 字，所以 7,8 值得试 —— "
                           "拼串长度完全由框数决定，多一个误检框就必然读错整串")
    ap.add_argument("--lendump", action = "store_true",
                    help = "额外打印「预测串长度 - 真值串长度」的分布："
                           "把整串错误拆成串短了 / 串长了 / 长度对但字错")
    ap.add_argument("--faildump", action = "store_true",
                    help = "列出「真值串长 = N 且整串读错」的图名，用来人工看难例长什么样")
    ap.add_argument("--failgtlen", type = int, default = 7,
                    help = "--faildump 只看哪个真值串长（默认 7，即部署场景那桶）")
    ap.add_argument("--failhead", default = "",
                    help = "--faildump 看哪个头的失败（默认最后一个 = 推荐头）")
    args = ap.parse_args()

    cfg = U.load_datafile(args.data)
    names = load_names(cfg["names"])
    classes = int(cfg["classes"])
    W = int(cfg["width"])
    paths = [ln.strip() for ln in open(cfg[args.split]) if ln.strip()]
    if args.limit:
        paths = paths[:args.limit]
    paths = [p for p in paths if os.path.exists(p.split(".")[0] + ".txt")]

    dev = torch.device("cpu")
    net = MD.Detector(classes, int(cfg["anchor_num"]), True).to(dev)
    net.load_state_dict(torch.load(args.weights, map_location = dev))
    net.eval()

    # 第二级权重是可选的：没有它也照样跑，只是只剩一级那一列。
    # 这样能拿「一级」那一列去和 fw_emulate.py 的数字对 —— 框的挑选逻辑两边必须一致，
    # 对上了才敢信「两级」那一列。
    #
    # 可以有**多个**二级头（小头 + 探针），跑在同一批框上。这不是炫技：
    # 一级头 12.06% 的整串天花板，到底是**框不够好**还是**头不够好**，
    # 只有同框换头才分得开。探针不能部署，但它能给出「框已经够好」的证据。
    heads = []
    if os.path.exists(args.charhead):
        # weights_only=False：这是**我们自己**训练脚本写出来的文件（不是下载的），
        # 而且早期版本里混了 numpy 标量，2.6 起的默认只允许张量。新的保存已经改成
        # 纯 python 类型，但旧权重还得能读。
        ck = torch.load(args.charhead, map_location = dev, weights_only = False)
        c = ck["c"]
        P_W, P_H = ck["size_wh"]
        pad = ck["pad"]
        h = SmallCharNet(int(ck["classes"]), (P_H, P_W), *c).to(dev)
        h.load_state_dict(ck["state_dict"])
        h.eval()
        heads.append(("小头", h, P_W, P_H, pad,
                      sum(p.numel() for p in h.parameters()),
                      ck.get("val_acc", float("nan")),
                      os.path.basename(args.charhead)))
    else:
        print("!! 没找到第二级权重 %s —— 只跑一级那一列（用来和 fw_emulate.py 对拍）"
              % args.charhead)
    if args.probe and os.path.exists(args.probe):
        pnet, PW, PH = load_probe(args.probe, classes, dev)
        heads.append(("探针", pnet, PW, PH, 0.25,
                      sum(p.numel() for p in pnet.parameters()), float("nan"),
                      os.path.basename(args.probe)))
    elif args.probe:
        print("!! 没找到探针权重 %s —— 跳过探针那一列" % args.probe)

    print("=" * 100)
    print("### 端到端：固件后处理 + 第二级识别头   val %d 张" % len(paths))
    print("  一级 %s" % os.path.basename(args.weights))
    for nm, _, PW, PH, pd, np_, hv, fn in heads:
        print("  二级[%s] %s（%d 参数, 输入 %dx%d, pad %.2f, 真值框上的 val %s）"
              % (nm, fn, np_, PW, PH, pd,
                 ("%.2f%%" % hv) if hv == hv else "n/a"))
    print("  配置 conf=%s nms=%s maxbox=%d geo=%s maxchars=%s"
          % (args.confs, args.nmses, args.maxbox, "on" if args.geo else "off",
             args.maxchars if args.maxchars else "off"))
    print("=" * 100)

    confs = [float(v) for v in args.confs.split(",")]
    nmses = [float(v) for v in args.nmses.split(",")]
    geos = [False, True] if args.geo else [False]
    mcs = [int(v) for v in args.maxchars.split(",")] if args.maxchars else [0]
    cfgs = [(a, b, g, m) for a in confs for b in nmses for g in geos for m in mcs]
    # 每一列一个 head key："一级" 是检测头，其余是二级各头。cfgs 之间**必须**是不同的
    # dict 对象，所以用工厂造，别用 `{k: dict(...) for k in cfgs}` 里共享子对象那种写法。
    hnames = [nm for nm, _, _, _, _, _, _, _ in heads]

    def blank():
        return dict(zh = {}, zt = {}, ex = {}, pos = {}, pn = {}, n = 0, nbr = 0,
                    dlen = {}, cross = {}, exgt = {}, okgt = {}, fail = [])

    acc = {k: blank() for k in cfgs}
    for a in acc.values():
        for hk in ["一级"] + hnames:
            a["zh"][hk] = 0
            a["zt"][hk] = 0
            a["ex"][hk] = 0
            a["pos"][hk] = {}
            a["dlen"][hk] = {}
            a["cross"][hk] = {}      # (真值串长, 预测串长) -> 张数，
                                     # 用来查「某一长度的牌是不是天生读不对」
            a["exgt"][hk] = {}       # 真值串长 -> 张数，用来按串长分桶算整串率
            a["okgt"][hk] = {}       # 真值串长 -> 整串正确张数

    t0 = time.time()
    for s in range(0, len(paths), args.batch):
        batch = []
        for p in paths[s:s + args.batch]:
            im = cv2.imread(p)
            if im is None:
                continue
            lab = np.array([[float(x) for x in ln.split()]
                            for ln in open(p.split(".")[0] + ".txt") if len(ln.split()) == 5],
                           np.float32)
            g = lab[lab[:, 0] > 0] if len(lab) else np.zeros((0, 5), np.float32)
            gsort = g[np.argsort(g[:, 1])] if len(g) else g
            gstr = "".join(names[int(v)] if 0 <= int(v) < len(names) else "?" for v in gsort[:, 0])
            batch.append((p, im, gstr))
        if not batch:
            continue

        xb = np.stack([cv2.resize(im, (W, W), interpolation = cv2.INTER_LINEAR)
                       for _, im, _ in batch])
        xb = torch.from_numpy(xb).permute(0, 3, 1, 2).float() / 255.0
        with torch.no_grad():
            raw = U.handel_preds(net(xb.to(dev)), cfg, dev).cpu().numpy()

        n_layer = 2
        grids = [(20, 20), (10, 10)] if raw.shape[1] == 1500 else [(40, 40), (20, 20)]
        prod = raw[:, :, 4] * raw[:, :, 5:].max(2)
        cid = raw[:, :, 5:].argmax(2)
        cx, cy, bw, bh = raw[:, :, 0], raw[:, :, 1], raw[:, :, 2], raw[:, :, 3]
        xyxy = np.stack([cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2], -1)

        # 先把所有配置的框都挑出来，再一次性抠图送第二级 —— 省一次前向的开销。
        per_img = []
        jobs = []                       # (key, img_i, box_px, cid)
        for i, (p, im, gstr) in enumerate(batch):
            H, Wr = im.shape[:2]
            sx, sy = Wr / float(W), H / float(W)
            item = {}
            for key in cfgs:
                a, b, g, m = key
                cands = [(xyxy[i, n], sc, cc) for n, sc, cc in
                         fw_decode(prod[i], cid[i], a, args.maxbox, n_layer, grids)]
                keep = fw_nms(cands, b)
                idx = select_idx(keep, g, m)
                # 固件坐标是 320x320 网络空间；抠图必须在**原生分辨率**上做，
                # 否则等于把字符又缩回 13px 再放大，第二级就白搭了。
                bpx = [np.array([keep[j][0][0] * sx, keep[j][0][1] * sy,
                                 keep[j][0][2] * sx, keep[j][0][3] * sy], np.float32)
                       for j in idx]
                item[key] = dict(keep = keep, idx = idx, bpx = bpx,
                                 cid = [keep[j][2] for j in idx])
                for q, bp in enumerate(bpx):
                    for nm, hnet, PW, PH, pd, _, _, _ in heads:
                        jobs.append((i, key, q, im, bp, nm, hnet, PW, PH, pd))
            per_img.append((gstr, item, p))

        # 第二级：真值框之外全部走这里。每个头自己一套输入尺寸和 pad，
        # 所以按头分组抠图、分组前向 —— 一张图上几个配置的框也要一起送，
        # 否则同一块像素会被不同配置重复抠好几遍。
        preds2 = {}
        if jobs:
            by_head = {}
            for j in jobs:
                by_head.setdefault(j[5], []).append(j)
            for nm, jl in by_head.items():
                _, _, _, _, _, _, hnet, PW, PH, pd = jl[0]
                crops = []
                for i, key, q, im, bp, _, _, _, _, _ in jl:
                    pt = cut_patch(im, bp, pd, (PW, PH))
                    crops.append(pt if pt is not None else np.zeros((PH, PW, 3), np.uint8))
                out = []
                for s2 in range(0, len(crops), 512):
                    b = torch.from_numpy(np.asarray(crops[s2:s2 + 512])).permute(0, 3, 1, 2).float() / 255.0
                    with torch.no_grad():
                        out.append(hnet(b.to(dev)).argmax(1).cpu().numpy())
                out = np.concatenate(out)
                for j, lab in zip(jl, out):
                    preds2.setdefault((j[0], j[1], nm), []).append(names[int(lab)])

        for i, (gstr, item, _p) in enumerate(per_img):
            for key in cfgs:
                d = item[key]
                keep, idx = d["keep"], d["idx"]
                a = acc[key]
                a["n"] += 1
                a["nbr"] += len(idx)
                cx1 = np.array([keep[j][0][0] for j in idx], np.float32)
                c1 = [names[cc] if 0 <= cc < len(names) else "?" for cc in d["cid"]]
                # 一级：检测头自己的 class argmax
                preds = dict()
                preds["一级"] = assemble(cx1, c1, 12) if idx else ""
                # 二级：抠出来送各自的头。assemble 里是 `chars[i]`（i 来自 cx1 的排序），
                # 两个列表**必须等长**，否则直接 IndexError；没跑第二级时 preds2 是空的，
                # 这里要挡住。
                for nm in hnames:
                    p2 = preds2.get((i, key, nm))
                    preds[nm] = assemble(cx1, p2, 12) if (idx and p2 and len(p2) == len(cx1)) else ""
                # 分母每张图只能加一次 —— 放进按头循环里会变成「头数」倍，
                # 逐位准确率就正好被腰斩（我先写成那样了，是自对拍时才发现的：
                # 位置0 报 16.54%，可它 ×2 = 33.1% 才和同一批图的逐字 73.95% 对得上）。
                for k in range(len(gstr)):
                    a["pn"][k] = a["pn"].get(k, 0) + 1
                for hk, s in preds.items():
                    for k in range(len(gstr)):
                        a["pos"][hk][k] = a["pos"][hk].get(k, 0) + int(k < len(s) and s[k] == gstr[k])
                    # 长度差（预测 - 真值）。整串率低到底是因为「字认错了」还是
                    # 「串断了/多了」—— 只看逐字/整串分不出来，这一栏把它分开。
                    dl = len(s) - len(gstr)
                    a["dlen"][hk][dl] = a["dlen"][hk].get(dl, 0) + 1
                    _ck = (len(gstr), len(s))
                    a["cross"][hk][_ck] = a["cross"][hk].get(_ck, 0) + 1
                    a["exgt"][hk][_ck[0]] = a["exgt"][hk].get(_ck[0], 0) + 1
                    a["okgt"][hk][_ck[0]] = a["okgt"][hk].get(_ck[0], 0) + int(s == gstr)
                    for x, y in zip(gstr, s[:len(gstr)]):
                        a["zt"][hk] += 1
                        a["zh"][hk] += int(x == y)
                    a["ex"][hk] += int(s == gstr)
                    if args.faildump and s != gstr:
                        a["fail"].append((hk, _p, gstr, s))

    hks = ["一级"] + hnames
    hdr = "%-6s %-5s %-4s %-4s %7s |" % ("conf", "nms", "geo", "maxc", "框/图")
    for hk in hks:
        hdr += " %-13s |" % ("%s 逐字/整串" % hk)
    for hk in hks:
        hdr += " %-15s |" % ("%s 位0/位1-6" % hk)
    print("\n" + hdr)
    print("-" * len(hdr))
    for key in cfgs:
        a = acc[key]
        n = max(a["n"], 1)
        line = "%-6.2f %-5.2f %-4s %-4s %7.2f |" % (
            key[0], key[1], "on" if key[2] else "off",
            key[3] if key[3] else "-", a["nbr"] / float(n))
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

    if args.lendump:
        # 把「整串没读对」拆成三类：串短了 / 串长了 / 长度对但字错。
        # 这是回答「79.57% 之后该往哪使劲」的那张表 —— 见报告 §3.5 第 5 条。
        for key in cfgs:
            a = acc[key]
            n = max(a["n"], 1)
            print("\n长度差（预测 - 真值）分布   conf %.2f  nms %.2f  geo %s  maxchars %s   共 %d 张"
                  % (key[0], key[1], "on" if key[2] else "off",
                     key[3] if key[3] else "-", a["n"]))
            print("  %-6s | %s | %7s %7s | %s"
                  % ("头", "   ".join("%5d" % d for d in range(-3, 4)),
                     "短(<0)", "长(>0)", "中位差  完全相等"))
            for hk in hks:
                h = a["dlen"][hk]
                tot = sum(h.values()) or 1
                srt, cur = 0, None
                for d in sorted(h):
                    srt += h[d]
                    if cur is None and srt * 2 >= tot:
                        cur = d
                short = sum(v for d, v in h.items() if d < 0)
                long_ = sum(v for d, v in h.items() if d > 0)
                print("  %-6s | %s | %6.2f%% %6.2f%% | %5d   %6.2f%%"
                      % (hk, "   ".join("%5d" % h.get(d, 0) for d in range(-3, 4)),
                         100.0 * short / tot, 100.0 * long_ / tot, cur,
                         100.0 * a["ex"][hk] / n))

            # 真值串长 × 预测串长。这一栏回答：「某个长度的车牌是不是天生读不对」——
            # 如果 6 字牌的预测长度几乎从不等于 6，那这些图就是**零希望的**，
            # 只会把整串率按占比往下拽（val 里 6 字牌占 3.5%，train 里只占 0.13%）。
            x = a["cross"][hks[-1]]
            glens = sorted({g for g, _ in x})
            plens = sorted({p for _, p in x})
            if glens:
                print("  真值串长 \\ 预测串长（%s）：" % hks[-1])
                print("    %-6s |%s | %7s" % ("", "".join("%6d" % p for p in plens), "合计"))
                for g in glens:
                    row = [x.get((g, p), 0) for p in plens]
                    print("    %-6d |%s | %7d"
                          % (g, "".join("%6d" % v for v in row), sum(row)))
                # 每个真值长度上的整串率。**这才是「代表性」数字**：val 这份连续尾块里
                # 6 字牌占 3.5%，而 train 里只占 0.13% —— 拿全体平均会把 6/8 字那几桶
                # 的硬伤摊到 7 字身上，低估真实场景。
                print("    按真值串长分桶：")
                for g in glens:
                    tot_g = a["exgt"][hks[-1]].get(g, 0)
                    ok_g = a["okgt"][hks[-1]].get(g, 0)
                    lenok = x.get((g, g), 0)
                    print("      %2d 字: %4d 张   长度对上 %5.1f%%   整串 %5.1f%% (%d/%d)"
                          % (g, tot_g, 100.0 * lenok / max(tot_g, 1),
                             100.0 * ok_g / max(tot_g, 1), ok_g, tot_g))
                # 按 train 的串长分布重加权（train: 6字 0.13% / 7字 97.96% / 8字 1.88%）
                TW = {6: 0.00135, 7: 0.97962, 8: 0.01885, 11: 0.00019}
                num = den = 0.0
                for g in glens:
                    w = TW.get(g)
                    if w is None:
                        continue                      # val 里出现的 9 字等 train 没有 -> 不给权重
                    tot_g = a["exgt"][hks[-1]].get(g, 0)
                    ok_g = a["okgt"][hks[-1]].get(g, 0)
                    num += w * (ok_g / max(tot_g, 1))
                    den += w
                if den > 0:
                    print("    ⇒ 按 train 的串长分布重加权后的整串 = %.2f%%  "
                          "(val 原始 %.2f%%)"
                          % (100.0 * num / den, 100.0 * a["ex"][hks[-1]] / n))

    if args.faildump:
        # 把某桶里「整串读错」的图名原样列出来 —— 数字看完了要去看图，
        # 难例长什么样决定了下一步该改定位、改分辨率还是改数据。
        want = args.failhead or hks[-1]
        for key in cfgs:
            a = acc[key]
            rows = [r for r in a["fail"]
                    if r[0] == want and len(r[2]) == args.failgtlen]
            if not rows:
                continue
            print("\n整串读错的图（头 = %s，真值 %d 字）  conf %.2f nms %.2f geo %s maxchars %s"
                  % (want, args.failgtlen, key[0], key[1], "on" if key[2] else "off",
                     key[3] if key[3] else "-"))
            for _, p, gstr, s in rows:
                print("  %-14s 真值 %-12s 预测 %-12s" % (os.path.basename(p), gstr, s or "(空)"))
            print("  共 %d 张" % len(rows))

    print("\n用时 %.0fs" % (time.time() - t0))
    print("注：所有列**用的是同一批框**，差别只在分类头，所以差多少就是那个头的净收益。")
    print("    '一级' = 检测头自己的 class argmax；其余 = 抠图后第二级识别头。")
    print("    位置k 那一列是**逐位字符正确率**（拼出来的串第 k 位是否等于真值第 k 位），")
    print("    不是 fw_emulate 的「框召回」—— 那个只问框在不在，这个问字对不对。")
    print("    '位0' 就是省份字。⚠️ 位置 k 超过 len(拼出来的串) 时记为错，所以")
    print("    逐位平均会略低于「逐字%」（逐字% 的 zip 只比到两个串都有的那一段）。")


if __name__ == "__main__":
    main()
