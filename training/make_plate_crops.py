# -*- coding: utf-8 -*-
"""从现有标注生成「车牌裁剪」数据集 —— 两级结构的第一级产物。

动机见 memory `plate-bottleneck-2026-10-15`：单阶段模型吃整图时，字符在 320×320 输入里
只有 **13.3×33.7 px**（37% 窄于 12px），stride=8 的特征图上横向只盖住 1.6 个格子，分类头
几乎没有空间分辨率可用。训练集上 mean CE 仍卡在 1.82 nat（正确类概率 0.16）——欠拟合。

本脚本把每张图裁到车牌框（class 0）附近，字符在同样的 320×320 输入里就变成约 26×87 px，
特征图上横向约 3.3 个格子。**训练代码、模型、输入尺寸全不动**，只换输入区域，
这样跑出来的差异就是裁剪本身带来的，干净可归因。

坐标重映射：label 是归一化到**原图**的 [cls cx cy w h]（见 utils/datasets.py:114，
cv2.resize 是直接拉伸，非 letterbox）。裁剪后按 (cx*W - x0) / crop_w 归一化到裁剪图。
datasets.py 到时候再把它拉到 320×320，缩放倍数由裁剪框尺寸决定。

只保留落在车牌框内的字符框：那 1.8% 有两块车牌却只标了一块框的图，第二块牌的字符
中心在车牌框外，会被自然丢掉 —— 顺带把这份数据洗了一遍。

产物：
  data_crop/train/*.jpg  data_crop/train/*.txt
  data_crop/val/*.jpg    data_crop/val/*.txt
  data_crop/train.txt    data_crop/val.txt    data_crop/voc.data
（voc.data 里 epochs 默认给 100：字符变大后收敛快得多，不必再跑 300。）

用法：
  python make_plate_crops.py                      # 生成 + 打印统计
  python make_plate_crops.py --margin 0.05
  python make_plate_crops.py --dry-run            # 只统计不写盘
"""

import sys
sys.dont_write_bytecode = True

import os
import argparse
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)

TRAIN_SRC = "data/train.txt"
VAL_SRC = "data/val.txt"
NAMES_SRC = "data/voc.names"


def read_labels(img_path):
    """返回 labels[N,5] = [cls cx cy w h]（归一化到原图），没有则 None。
    只读标签文件，不解码图片 —— 图尺寸从 imread 的结果拿，避免重复解码。"""
    lab_path = img_path.split(".")[0] + ".txt"
    if not os.path.exists(lab_path):
        return None
    rows = []
    with open(lab_path) as f:
        for ln in f:
            p = ln.split()
            if len(p) != 5:
                continue
            rows.append([float(x) for x in p])
    return np.array(rows, np.float32) if rows else None


def best_plate(labels):
    """返回面积最大的车牌框索引（class 0），没有则 None。"""
    idx = np.nonzero(labels[:, 0] == 0)[0]
    if not len(idx):
        return None
    area = labels[idx, 3] * labels[idx, 4]
    return int(idx[area.argmax()])


def process(img_path, margin):
    """裁一块车牌区域，返回 (crop_img, crop_labels[N,5], 诊断信息 dict) 或 None。"""
    labels = read_labels(img_path)
    if labels is None:
        return None

    pi = best_plate(labels)
    if pi is None:
        return dict(skip = "no_plate")

    img = cv2.imread(img_path)
    if img is None:
        return dict(skip = "imread_fail")
    H, W = img.shape[:2]

    cx, cy, bw, bh = labels[pi, 1], labels[pi, 2], labels[pi, 3], labels[pi, 4]
    # 车牌框转原图像素，再按 margin 外扩（按框自身尺寸的比例扩，不按图扩）
    px0 = (cx - bw / 2) * W
    py0 = (cy - bh / 2) * H
    px1 = (cx + bw / 2) * W
    py1 = (cy + bh / 2) * H
    mx = (px1 - px0) * margin
    my = (py1 - py0) * margin
    x0 = int(np.clip(np.floor(px0 - mx), 0, W - 1))
    y0 = int(np.clip(np.floor(py0 - my), 0, H - 1))
    x1 = int(np.clip(np.ceil(px1 + mx), x0 + 1, W))
    y1 = int(np.clip(np.ceil(py1 + my), y0 + 1, H))
    cw, ch = x1 - x0, y1 - y0

    # 字符框：class>0 且中心落在**车牌框**内（不是裁剪框内！）。
    # 用裁剪框会把外扩的那圈也算进来 —— 0000.jpg 那块漏标的第二车牌，正好有一个字符
    # 落在车牌框左边 2px 的外扩区里，会被当成目标字符混进裁剪图。
    # 容差 tol 按车牌框自身尺寸取，容忍一点透视外溢。
    tol_x, tol_y = (px1 - px0) * 0.02, (py1 - py0) * 0.02
    keep = []
    n_out = 0
    for r in labels:
        if r[0] <= 0:
            continue
        ecx, ecy = r[1] * W, r[2] * H
        if not (px0 - tol_x <= ecx <= px1 + tol_x and py0 - tol_y <= ecy <= py1 + tol_y):
            n_out += 1
            continue

        nx = (r[1] * W - x0) / cw
        ny = (r[2] * H - y0) / ch
        nw = r[3] * W / cw
        nh = r[4] * H / ch
        # 夹到 [0,1]，并把被裁掉的部分截断（框可能伸出裁剪边界）
        nx0 = max(0.0, nx - nw / 2)
        nx1 = min(1.0, nx + nw / 2)
        ny0 = max(0.0, ny - nh / 2)
        ny1 = min(1.0, ny + nh / 2)
        if nx1 - nx0 <= 0 or ny1 - ny0 <= 0:
            continue
        keep.append([r[0], (nx0 + nx1) / 2, (ny0 + ny1) / 2,
                     nx1 - nx0, ny1 - ny0])

    if not keep:
        return dict(skip = "no_char_in_plate")

    crop = img[y0:y1, x0:x1]

    return dict(crop = crop, labels = np.array(keep, np.float32),
                cw = cw, ch = ch, W = W, H = H,
                n_char = len(keep), n_out = n_out,
                plate_w = px1 - px0, plate_h = py1 - py0)


def make_split(src, out_dir, margin, dry, tag):
    with open(src) as f:
        paths = [ln.strip() for ln in f if ln.strip()]
    if not dry:
        os.makedirs(out_dir, exist_ok = True)

    kept, skips = [], {}
    char_w, char_h = [], []      # 缩到 320 后的字符像素尺寸
    crop_ws, crop_hs = [], []
    n_char_tot = 0
    n_out_tot = 0
    per_crop = []                # 每张裁剪保留下来的字符数

    for p in paths:
        r = process(p, margin)
        if r is None:
            skips["unreadable"] = skips.get("unreadable", 0) + 1
            continue
        if "skip" in r:
            skips[r["skip"]] = skips.get(r["skip"], 0) + 1
            continue

        # 诊断：字符缩到 320 后的像素尺寸。datasets.py 把裁剪图直接拉到 320×320，
        # 所以 320 宽的图里，裁剪图 1 像素 -> 320/cw 像素。
        sx, sy = 320.0 / r["cw"], 320.0 / r["ch"]
        for rl in r["labels"]:
            char_w.append(rl[3] * r["cw"] * sx)
            char_h.append(rl[4] * r["ch"] * sy)
        crop_ws.append(r["cw"])
        crop_hs.append(r["ch"])
        n_char_tot += len(r["labels"])
        n_out_tot += r.get("n_out", 0)
        per_crop.append(len(r["labels"]))
        kept.append(p)

        if not dry:
            stem = os.path.splitext(os.path.basename(p))[0]
            cv2.imwrite(os.path.join(out_dir, stem + ".jpg"), r["crop"],
                        [cv2.IMWRITE_JPEG_QUALITY, 95])
            with open(os.path.join(out_dir, stem + ".txt"), "w") as f:
                for rl in r["labels"]:
                    f.write("%d %.6f %.6f %.6f %.6f\n"
                            % (int(rl[0]), rl[1], rl[2], rl[3], rl[4]))

    print("\n--- %s：%d/%d 张保留（%.1f%%），字符框 %d"
          % (tag, len(kept), len(paths), 100.0 * len(kept) / max(len(paths), 1), n_char_tot))
    for k, v in sorted(skips.items(), key = lambda kv: -kv[1]):
        print("    丢弃 %-20s %d 张" % (k, v))
    if char_w:
        cw_ = np.array(char_w)
        ch_ = np.array(char_h)
        q = lambda a, p: "%.1f" % np.percentile(a, p)
        print("    字符缩到 320 后  宽 25%%=%s 中位=%s 75%%=%s   |  高 中位=%s"
              % (q(cw_, 25), q(cw_, 50), q(cw_, 75), q(ch_, 50)))
        print("    宽度 <12px 占比 %.1f%%   <8px 占比 %.1f%%   （对照：整图时是 37.4%% / 2.5%%）"
              % (100.0 * (cw_ < 12).mean(), 100.0 * (cw_ < 8).mean()))
        print("    裁剪框原图像素  宽 中位=%.0f  高 中位=%.0f"
              % (np.median(crop_ws), np.median(crop_hs)))
    if per_crop:
        hist = {}
        for k in per_crop:
            hist[k] = hist.get(k, 0) + 1
        top = sorted(hist.items(), key = lambda kv: -kv[1])[:6]
        print("    每张裁剪的字符数分布（前 6 多）：%s"
              % "  ".join("%d字:%d张" % (k, v) for k, v in top))
        print("    落在车牌框外被丢弃的字符框 %d 个（含漏标第二车牌的 %.1f%% 那种图的字符）"
              % (n_out_tot, 100.0 * 54 / 3000))
    return kept


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--margin", type = float, default = 0.10,
                    help = "车牌框外扩比例（按框自身尺寸）。0.10 = 四周各扩 10%%。")
    ap.add_argument("--out", default = "data_crop")
    ap.add_argument("--epochs", type = int, default = 100, help = "写进新 voc.data 的轮数")
    ap.add_argument("--dry-run", action = "store_true", help = "只统计，不写任何文件")
    args = ap.parse_args()

    print("margin=%.2f  out=%s  dry_run=%s" % (args.margin, args.out, args.dry_run))
    tr = make_split(TRAIN_SRC, os.path.join(args.out, "train"), args.margin, args.dry_run, "train")
    va = make_split(VAL_SRC, os.path.join(args.out, "val"), args.margin, args.dry_run, "val")

    if args.dry_run:
        print("\n--dry-run：没有写任何文件。")
        return

    for name, paths in (("train.txt", tr), ("val.txt", va)):
        with open(os.path.join(args.out, name), "w") as f:
            for p in paths:
                stem = os.path.splitext(os.path.basename(p))[0]
                sub = "train" if name == "train.txt" else "val"
                f.write("%s/%s/%s.jpg\n" % (args.out, sub, stem))

    # 超参对齐原 voc.data，只改数据路径和 epochs。
    # steps 按 epochs 同比例缩（原 300 轮的 150/250 -> 一半处的 0.5/0.83）
    with open(os.path.join(args.out, "voc.data"), "w") as f:
        f.write("[name]\nmodel_name=voc_crop\n\n")
        f.write("[train-configure]\nepochs=%d\nsteps=%d,%d\nbatch_size=128\nsubdivisions=1\n"
                "learning_rate=0.001\n\n" % (args.epochs, int(args.epochs * 0.5),
                                             int(args.epochs * 0.833)))
        f.write("[model-configure]\npre_weights=None\nclasses=70\nwidth=320\nheight=320\n"
                "anchor_num=3\n"
                "anchors=11.03,24.54, 14.40,36.74, 20.08,52.82, 27.04,78.23, 144.02,96.56, "
                "239.25,195.42\n\n")
        f.write("[data-configure]\ntrain=%s/train.txt\nval=%s/val.txt\nnames=%s\n"
                % (args.out, args.out, NAMES_SRC))

    print("\n写好：%s/{train,val}/  %s/{train,val}.txt  %s/voc.data（epochs=%d）"
          % (args.out, args.out, args.out, args.epochs))
    print("注意：models 保存到 weights/，model_name=voc_crop，不会覆盖 voc-* 的旧权重。")


if __name__ == "__main__":
    main()
