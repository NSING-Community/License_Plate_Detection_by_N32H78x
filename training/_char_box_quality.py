# -*- coding: utf-8 -*-
"""**坏框到底怎么坏的** —— 预测字符框 vs 真值字符框的形状对比。

背景链条：
  · `_char_confusion.py` 证明第二级头在**真值框**上是 99.08% 逐字、数字全对，
    错的 33 个里绝大多数是省份字 —— **识别头已经做完了**。
  · 但端到端 7 字桶只有 85.8%，而 `cascade_ceiling` 早量过：
    真值字符框按「最佳预测框 IoU」分桶，IoU 0.7 以上才有 97-99% 的逐字率。
  · ⇒ 剩下的账几乎全在**框**上。这个脚本回答「怎么坏的」：

    合并(merge)  —— 两个相邻字符被一个宽框吃进去（宽比 ≈ 2，IoU 很低）
    拆裂(split)  —— 一个字符被两个窄框分了（宽比 ≈ 0.5）
    漂移(shift)  —— 框宽对但中心偏了
    漏检(miss)   —— 附近压根没有框

  这三种的**修法完全不同**：合并/拆裂 ⇒ 需要更细的 anchor 网格（stride 8）；
  漂移 ⇒ 需要更好的回归；漏检 ⇒ 需要更高分辨率或更强特征。
  所以先量清楚，再决定要不要重训。

用法：
  python _char_box_quality.py --weights weights/voc_crop_ft-119-epoch-0.589619ap-model.pth
  python _char_box_quality.py --conf 0.10 --nms 0.30
"""
import os, sys, argparse
import numpy as np
import cv2
import torch

try:
    sys.stdout.reconfigure(encoding = "utf-8")
except Exception:
    pass

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.chdir(HERE)

import utils.utils as U                                   # noqa: E402
import model.detector as MD                               # noqa: E402
from fw_emulate import fw_decode, fw_nms                  # noqa: E402
from cascade_ceiling import load_names, iou_pair, cut_patch   # noqa: E402
from train_char_head import SmallCharNet                  # noqa: E402


def select_idx(keep, maxchars = 0, cap16 = True):
    idx = []
    for i in range(len(keep)):
        if keep[i][2] > 0:
            idx.append(i)
        if cap16 and len(idx) >= 16:
            break
    if maxchars and len(idx) > maxchars:
        idx = sorted(idx, key = lambda i: -keep[i][1])[:maxchars]
    idx.sort(key = lambda i: keep[i][0][0])
    return idx


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default = "data_crop/voc.data")
    ap.add_argument("--split", default = "val")
    ap.add_argument("--weights", default = "weights/voc_crop_ft-119-epoch-0.589619ap-model.pth")
    ap.add_argument("--conf", type = float, default = 0.10)
    ap.add_argument("--nms", type = float, default = 0.30)
    ap.add_argument("--maxbox", type = int, default = 64)
    ap.add_argument("--maxchars", default = "", help = "NMS 后按分数砍到 N 个框（空=不砍）")
    ap.add_argument("--limit", type = int, default = 0)
    ap.add_argument("--charhead", default = "weights/charhead_16_32_64_32_64.pt",
                    help = "顺手把这个头跑在**预测框**的抠图上，按 IoU 分桶 —— "
                           "这才是「真值框上 99.08% 到底能不能迁移到预测框」的答案")
    args = ap.parse_args()

    dev = torch.device("cpu")
    cfg = U.load_datafile(args.data)
    names = load_names(cfg["names"])
    classes, W = int(cfg["classes"]), int(cfg["width"])
    net = MD.Detector(classes, int(cfg["anchor_num"]), True).to(dev)
    net.load_state_dict(torch.load(args.weights, map_location = dev))
    net.eval()

    paths = [ln.strip() for ln in open(cfg[args.split]) if ln.strip()]
    if args.limit:
        paths = paths[:args.limit]
    paths = [p for p in paths if os.path.exists(p.split(".")[0] + ".txt")]
    cap = int(args.maxchars) if args.maxchars else 0

    # 第二级头（可选）：跑在**预测框**的抠图上。要和 two_stage_e2e 一个口径，
    # 所以 pad / 输入尺寸都从 checkpoint 里取，不另设。
    head = None
    if args.charhead and os.path.exists(args.charhead):
        ck = torch.load(args.charhead, map_location = dev, weights_only = False)
        P_W, P_H = ck["size_wh"]
        hpad = ck["pad"]
        head = SmallCharNet(int(ck["classes"]), (P_H, P_W), *ck["c"]).to(dev)
        head.load_state_dict(ck["state_dict"])
        head.eval()
        print("二级头 %s（%d 参数, 输入 %dx%d, pad %.2f）—— 会跑在**预测框**上"
              % (os.path.basename(args.charhead),
                 sum(p.numel() for p in head.parameters()), P_W, P_H, hpad))

    print("=" * 96)
    print("### 字符框质量诊断   %s %d 张" % (args.split, len(paths)))
    print("  权重 %s   conf %.2f  nms %.2f  maxbox %d  maxchars %s"
          % (os.path.basename(args.weights), args.conf, args.nms, args.maxbox,
             args.maxchars or "off"))
    print("=" * 96)

    # 每条匹配记录：(位置, IoU, 宽比 pred/gt, 中心偏移/GT宽, 类别是否一致)
    rec = []
    crops, gt_cid = [], []           # 与 rec 一一对应，喂二级头用
    n_gt = 0
    for s in range(0, len(paths), 32):
        batch = []
        for p in paths[s:s + 32]:
            im = cv2.imread(p)
            if im is None:
                continue
            lab = np.array([[float(x) for x in ln.split()]
                            for ln in open(p.split(".")[0] + ".txt") if len(ln.split()) == 5],
                           np.float32)
            g = lab[lab[:, 0] > 0] if len(lab) else np.zeros((0, 5), np.float32)
            g = g[np.argsort(g[:, 1])] if len(g) else g
            batch.append((im, g))
        if not batch:
            continue
        xb = np.stack([cv2.resize(im, (W, W), interpolation = cv2.INTER_LINEAR)
                       for im, _ in batch])
        xb = torch.from_numpy(xb).permute(0, 3, 1, 2).float() / 255.0
        with torch.no_grad():
            raw = U.handel_preds(net(xb.to(dev)), cfg, dev).cpu().numpy()

        n_layer = 2
        grids = [(20, 20), (10, 10)] if raw.shape[1] == 1500 else [(40, 40), (20, 20)]
        prod = raw[:, :, 4] * raw[:, :, 5:].max(2)
        cid = raw[:, :, 5:].argmax(2)
        cx, cy, bw, bh = raw[:, :, 0], raw[:, :, 1], raw[:, :, 2], raw[:, :, 3]
        xyxy = np.stack([cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2], -1)

        for i, (im, g) in enumerate(batch):
            if not len(g):
                continue
            H, Wr = im.shape[:2]
            sx, sy = Wr / float(W), H / float(W)
            # 这张裁剪车的**清晰度**（Laplacian 方差）和原始宽度。
            # 如果坏框集中在糊图/小图上，那这就是光学/数据问题，改架构没用 ——
            # 必须在动手重训之前把这条排除掉。
            sharp = float(cv2.Laplacian(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY),
                                        cv2.CV_64F).var())
            # 顺带把每个框来自哪一层带下去（fw_decode 的 n 是 1500 个 anchor 的光栅下标，
            # 层 0 = 20x20/stride16，层 1 = 10x10/stride32）。fw_nms 只碰 z[0..2]，
            # 多一个元素不影响它，但它能回答「坏框是不是粗网格那一层产出的」。
            cands = [(xyxy[i, n], sc, cc, 0 if n < 1200 else 1) for n, sc, cc in
                     fw_decode(prod[i], cid[i], args.conf, args.maxbox, n_layer, grids)]
            keep = fw_nms(cands, args.nms)
            idx = select_idx(keep, cap)
            # 预测框：原生分辨率，xyxy
            pb = np.array([[keep[j][0][0] * sx, keep[j][0][1] * sy,
                            keep[j][0][2] * sx, keep[j][0][3] * sy] for j in idx],
                          np.float32) if idx else np.zeros((0, 4), np.float32)
            pc = [keep[j][2] for j in idx]
            pl = [keep[j][3] for j in idx]        # 框来自哪一层

            # 真值框：原生分辨率
            gb = np.zeros((len(g), 4), np.float32)
            for k in range(len(g)):
                gx, gy, gw, gh = g[k, 1] * Wr, g[k, 2] * H, g[k, 3] * Wr, g[k, 4] * H
                gb[k] = [gx - gw / 2, gy - gh / 2, gx + gw / 2, gy + gh / 2]

            for k in range(len(g)):
                n_gt += 1
                gw_gt = max(gb[k, 2] - gb[k, 0], 1e-6)
                if len(pb):
                    js = [iou_pair(gb[k], pb[t]) for t in range(len(pb))]
                    j = int(np.argmax(js))
                    iou = js[j]
                    wratio = (pb[j, 2] - pb[j, 0]) / gw_gt
                    off = abs(((pb[j, 0] + pb[j, 2]) / 2) - ((gb[k, 0] + gb[k, 2]) / 2)) / gw_gt
                    same = int(pc[j]) == int(g[k, 0])
                    layer = pl[j]
                    if head is not None:
                        crops.append(cut_patch(im, pb[j], hpad, (P_W, P_H)))
                        gt_cid.append(int(g[k, 0]))
                else:
                    iou, wratio, off, same = 0.0, 0.0, 9.9, 0
                    layer = -1
                    if head is not None:
                        crops.append(None)
                        gt_cid.append(int(g[k, 0]))
                rec.append((k, iou, wratio, off, same, sharp, Wr, layer))
    rec = np.array(rec, np.float32)
    print("\n真值字符框 %d 个，预测框（conf %.2f/nms %.2f/cap %s）匹配结果："
          % (n_gt, args.conf, args.nms, args.maxchars or "off"))

    # ---- 1. IoU 分桶 ----
    print("\n--- 1) IoU 分桶（每个真值字符框取最佳预测框）---")
    edges = [0.0, 0.3, 0.5, 0.7, 0.8, 1.01]
    for a, b in zip(edges[:-1], edges[1:]):
        m = (rec[:, 1] >= a) & (rec[:, 1] < b)
        if not m.sum():
            continue
        print("  IoU %.1f~%.2f  n=%5d (%5.1f%%)   宽比中位 %.2f  中心偏移中位 %.2f×GT宽  类一致 %5.1f%%"
              % (a, b, m.sum(), 100.0 * m.mean(), np.median(rec[m, 2]),
                 np.median(rec[m, 3]), 100.0 * rec[m, 4].mean()))

    # ---- 2. 坏框（IoU<0.7）到底怎么坏的 ----
    bad = rec[rec[:, 1] < 0.7]
    print("\n--- 2) IoU<0.7 的 %d 个（占 %.1f%%）坏在哪 ---" % (len(bad), 100.0 * len(bad) / n_gt))
    if len(bad):
        m_merge = (bad[:, 2] >= 1.5) & (bad[:, 3] < 0.5)
        m_split = (bad[:, 2] <= 0.67) & (bad[:, 3] < 0.5)
        m_shift = bad[:, 3] >= 0.5
        m_other = ~(m_merge | m_split | m_shift)
        for nm, m in (("合并（宽比>=1.5 且中心基本重合）", m_merge),
                      ("拆裂（宽比<=0.67 且中心基本重合）", m_split),
                      ("漂移（中心偏 >= 0.5×GT宽）", m_shift),
                      ("其他（尺寸也怪）", m_other)):
            print("    %-28s %5d 个  占坏框 %5.1f%%  占全体 %.2f%%"
                  % (nm, int(m.sum()), 100.0 * m.sum() / len(bad), 100.0 * m.sum() / n_gt))
        print("    （宽比 = 预测框宽 / 真值框宽；1.0 = 宽度对，2.0 = 一个框吃了两个字符）")
        print("    宽比分位 p10 %.2f  p25 %.2f  p50 %.2f  p75 %.2f  p90 %.2f"
              % tuple(np.percentile(bad[:, 2], [10, 25, 50, 75, 90])))
        print("    中心偏移分位 p50 %.2f  p75 %.2f  p90 %.2f"
              % tuple(np.percentile(bad[:, 3], [50, 75, 90])))

    # ---- 3. 按位置切：坏框是不是集中在某几个位置 ----
    print("\n--- 3) 按「从左到右第几个字符」看坏框率 ---")
    print("  %-6s %8s %10s %10s   %s" % ("位置", "样本", "IoU<0.7", "类不一致", "宽比中位"))
    for k in sorted(set(rec[:, 0].astype(int).tolist())):
        m = rec[:, 0] == k
        nm = "位0(省)" if k == 0 else "位%d" % k
        print("  %-6s %8d %9.2f%% %9.2f%%   %.2f"
              % (nm, m.sum(), 100.0 * (rec[m, 1] < 0.7).mean(),
                 100.0 * (1 - rec[m, 4].mean()), np.median(rec[m, 2])))

    # ---- 4. 总数账 ----
    print("\n--- 4) 总账 ---")
    good = rec[:, 1] >= 0.7
    print("  IoU>=0.7 的字符框占 %.2f%%（这些在真值框上的逐字率是 99.08%%）" % (100.0 * good.mean()))
    print("  IoU< 0.7 的字符框占 %.2f%%（这些按 cascade_ceiling 的分桶，逐字率掉到 ~50%%）"
          % (100.0 * (1 - good.mean())))
    print("  ⇒ 预测框上的期望逐字率 ≈ %.2f%% + %.2f%% ≈ %.2f%%"
          % (100.0 * good.mean() * 0.9908, 100.0 * (1 - good.mean()) * 0.50,
             100.0 * (good.mean() * 0.9908 + (1 - good.mean()) * 0.50)))
    print("     （对照：two_stage_e2e 实测 16:32:64 在这一档是 93.37%）")

    # ---- 5. 二级头跑在预测框上：把上面那条「假设 99.08% 能迁移」验掉 ----
    if head is not None:
        probs = []
        for s in range(0, len(crops), 512):
            chunk = crops[s:s + 512]
            arr = np.asarray([c if c is not None else np.zeros((P_H, P_W, 3), np.uint8)
                              for c in chunk])
            b = torch.from_numpy(arr).permute(0, 3, 1, 2).float() / 255.0
            with torch.no_grad():
                probs.append(torch.softmax(head(b.to(dev)), 1).cpu().numpy())
        probs = np.concatenate(probs)
        gtc = np.array(gt_cid)
        hok = probs.argmax(1) == gtc
        print("\n--- 5) 同一个二级头，喂**预测框**的抠图（真值框上它是 99.08%%）---")
        print("  全体逐字 %.2f%%   （GT 框 99.08%%  ⇒ 框质量净代价 %.2f 个点）"
              % (100.0 * hok.mean(), 100.0 * (0.9908 - hok.mean())))
        print("  %-14s %7s %10s %10s" % ("IoU 档", "样本", "二级逐字", "GT框对照"))
        for a, b_ in zip(edges[:-1], edges[1:]):
            m = (rec[:, 1] >= a) & (rec[:, 1] < b_)
            if not m.sum():
                continue
            print("  %.1f~%.2f        %7d %9.2f%% %9.2f%%"
                  % (a, b_, m.sum(), 100.0 * hok[m].mean(), 99.08))
        print("  %-14s %7s %10s" % ("宽比档", "样本", "二级逐字"))
        for a, b_ in zip([0, 0.9, 1.1, 1.3, 1.6], [0.9, 1.1, 1.3, 1.6, 99]):
            m = (rec[:, 2] >= a) & (rec[:, 2] < b_)
            if not m.sum():
                continue
            print("  宽比 %.1f~%.1f    %7d %9.2f%%" % (a, b_, m.sum(), 100.0 * hok[m].mean()))
        # 闭掉总账：把「框质量」和「头」两笔账分开
        print("\n--- 6) 总账闭合 ---")
        print("  二级头在**真值框**上       逐字 %.2f%%" % (100.0 * 0.9908))
        print("  二级头在**预测框**上       逐字 %.2f%%   ⇒ 框质量吃掉 %.2f 点"
              % (100.0 * hok.mean(), 100.0 * (0.9908 - hok.mean())))
        # 如果框信息被抹掉（比如给每个字符一个完美框），上限就是 GT 框那一行
        print("  ⇒ 「把所有框修到 IoU>=0.7」能拿回的上限 = 真值框那一行 − 预测框这一行 = %.2f 点"
              % (100.0 * (0.9908 - hok.mean())))

        # ---- 7. 坏框是不是「糊图」造成的？改架构之前必须排除这一条 ----
        sh, wid = rec[:, 5], rec[:, 6]
        print("\n--- 7) 坏框 vs 图像质量（糊图/小图）---")
        qs = np.percentile(sh, [0, 20, 40, 60, 80, 100])
        print("  清晰度（Laplacian 方差）分位：%s" % "  ".join("%.0f" % v for v in qs))
        print("  %-16s %7s %10s %10s %12s" % ("清晰度档", "字符框", "IoU<0.7", "二级逐字", "宽比>=1.3"))
        for a, b in zip(qs[:-1], qs[1:]):
            m = (sh >= a) & (sh <= b)
            if not m.sum():
                continue
            print("  %7.0f~%-7.0f %7d %9.2f%% %9.2f%% %11.2f%%"
                  % (a, b, m.sum(), 100.0 * (rec[m, 1] < 0.7).mean(),
                     100.0 * hok[m].mean(), 100.0 * (rec[m, 2] >= 1.3).mean()))
        # 清晰度与坏框率的相关
        hi = sh >= np.median(sh)
        print("  清晰度 >= 中位: 坏框率 %.2f%%  二级逐字 %.2f%%"
              % (100.0 * (rec[hi, 1] < 0.7).mean(), 100.0 * hok[hi].mean()))
        print("  清晰度 <  中位: 坏框率 %.2f%%  二级逐字 %.2f%%"
              % (100.0 * (rec[~hi, 1] < 0.7).mean(), 100.0 * hok[~hi].mean()))
        print("  皮尔逊(清晰度, IoU) = %.3f   （不显著 ⇒ 坏框不是糊图造成的）"
              % float(np.corrcoef(sh, rec[:, 1])[0, 1]))

        ws = np.percentile(wid, [0, 20, 40, 60, 80, 100])
        print("\n  裁剪图宽度分位：%s" % "  ".join("%.0f" % v for v in ws))
        print("  %-16s %7s %10s %10s" % ("宽度档", "字符框", "IoU<0.7", "二级逐字"))
        for a, b in zip(ws[:-1], ws[1:]):
            m = (wid >= a) & (wid <= b)
            if not m.sum():
                continue
            print("  %7.0f~%-7.0f %7d %9.2f%% %9.2f%%"
                  % (a, b, m.sum(), 100.0 * (rec[m, 1] < 0.7).mean(), 100.0 * hok[m].mean()))

        # ---- 8. 坏框是哪个 stride 的头产出的？ ----
        # 字符在网络空间里 ~24px 宽，stride16 的格子只有 16px ⇒ 字符还不到 1.5 个格子宽；
        # stride32 的格子 32px ⇒ 字符**比一个格子还窄**。粗网格表达不了这个尺度，
        # 「太宽/合并」最容易从这里冒出来。这一栏就是判它。
        print("\n--- 8) 坏框来自哪一层（字符在 320 空间宽约 24px，stride16 格 16px、stride32 格 32px）---")
        lay = rec[:, 7]
        for L, nm in ((0, "层0 20x20 stride16"), (1, "层1 10x10 stride32"), (-1, "无框")):
            m = lay == L
            if not m.sum():
                continue
            print("  %-18s %7d 个（%.1f%%）  IoU<0.7 %5.1f%%  宽比中位 %.2f  二级逐字 %.2f%%"
                  % (nm, m.sum(), 100.0 * m.mean(), 100.0 * (rec[m, 1] < 0.7).mean(),
                     np.median(rec[m, 2]) if L >= 0 else float("nan"),
                     100.0 * hok[m].mean() if hok is not None else float("nan")))
        print("  被选中的预测框里，来自 stride32 的占 %.1f%%"
              % (100.0 * (lay == 1).mean()))


if __name__ == "__main__":
    main()
