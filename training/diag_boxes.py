# -*- coding: utf-8 -*-
"""标注框尺度统计（只读：不写文件，只往 stdout 打）。

为什么需要：`diag_loss.py` 显示模型在**训练集**上 mean CE 仍有 1.82 nat（正确类概率 0.16），
train CE 与 val CE 只差 0.22 —— 是欠拟合。但要区分「练不够」和「320px 下字符像素太少、
根本没法学」，就必须知道一个字符在 320×320 输入里占几个像素。

顺带核对标注质量：字符框数 / 车牌框数 应该≈7。若某些图出现 13 个字符框却只有 1 个车牌框，
说明存在漏标车牌的图，那会直接污染训练目标。

labels 路径规则见 utils/datasets.py:110 —— img_path.split(".")[0] + ".txt"。
label 每行 = cls cx cy w h，全部归一化到 [0,1]（cv2.resize 直接拉伸，非 letterbox）。
"""

import sys
sys.dont_write_bytecode = True

import os
import argparse
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)


def pct(a, ps = (1, 5, 25, 50, 75, 95, 99)):
    a = np.asarray(a, np.float64)
    if not len(a):
        return "  (空)"
    return "  ".join("%s:%.2f" % (p, np.percentile(a, p)) for p in ps)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--list", default = "data/train.txt")
    ap.add_argument("--limit", type = int, default = 3000, help = "只统计前 N 张，0 = 全部")
    ap.add_argument("--size", type = int, default = 320, help = "训练输入边长")
    args = ap.parse_args()

    with open(args.list) as f:
        paths = [ln.strip() for ln in f if ln.strip()]
    if args.limit:
        paths = paths[:args.limit]

    n_img = 0
    n_plate = 0
    n_char = 0
    n_lab_extra = 0        # 非 0/1 列的坏行
    char_w0, char_h0 = [], []   # 原图像素
    char_w, char_h = [], []     # 缩到 size×size 后
    plate_w, plate_h = [], []
    char_per_img = []
    plate_per_img = []
    two_plate = 0          # 字符数 > 9 但车牌框只有 1 个（疑似漏标第二块牌）
    no_plate = 0           # 有字符框却没有车牌框

    for p in paths:
        lab_path = p.split(".")[0] + ".txt"
        if not os.path.exists(lab_path):
            continue
        try:
            with open(lab_path) as f:
                rows = [ln.split() for ln in f if ln.strip()]
        except Exception as e:
            print("读不了 %s: %s" % (lab_path, e))
            continue
        if not rows:
            continue
        n_img += 1
        # 原图尺寸：只读图头，不解码像素
        try:
            import struct
            with open(p, "rb") as fh:
                data = fh.read(65536)
            W = H = None
            i = 2
            while i < len(data) - 9:
                if data[i] != 0xFF:
                    i += 1
                    continue
                m = data[i + 1]
                if m in (0xC0, 0xC1, 0xC2):
                    H, W = struct.unpack(">HH", data[i + 5:i + 9])
                    break
                if m in (0xD8, 0xD9) or 0xD0 <= m <= 0xD7:
                    i += 2
                    continue
                seg = struct.unpack(">H", data[i + 2:i + 4])[0]
                i += 2 + seg
            if W is None:
                from PIL import Image
                W, H = Image.open(p).size
        except Exception as e:
            print("读不了图尺寸 %s: %s" % (p, e))
            continue

        cs = 0
        ps_ = 0
        for r in rows:
            if len(r) != 5:
                n_lab_extra += 1
                continue
            c = int(float(r[0]))
            w_, h_ = float(r[3]), float(r[4])
            if c == 0:
                ps_ += 1
                plate_w.append(w_ * W)
                plate_h.append(h_ * H)
            else:
                cs += 1
                char_w0.append(w_ * W)
                char_h0.append(h_ * H)
                char_w.append(w_ * args.size)
                char_h.append(h_ * args.size)
        n_plate += ps_
        n_char += cs
        char_per_img.append(cs)
        plate_per_img.append(ps_)
        if cs > 9 and ps_ == 1:
            two_plate += 1
        if cs > 0 and ps_ == 0:
            no_plate += 1

    print("=" * 84)
    print("### 标注统计   %s   前 %d 张，实际读到 %d 张" % (args.list, args.limit, n_img))
    print("=" * 84)
    print("  字符框 %d   车牌框 %d   字符/车牌 = %.3f" % (n_char, n_plate, n_char / max(n_plate, 1)))
    print("  字符框/图 %.2f  （中位 %.0f）   车牌框/图 %.2f  （中位 %.0f）"
          % (n_char / max(n_img, 1), np.median(char_per_img) if char_per_img else 0,
             n_plate / max(n_img, 1), np.median(plate_per_img) if plate_per_img else 0))
    if n_lab_extra:
        print("  !! 列数 != 5 的坏行 %d" % n_lab_extra)
    if two_plate:
        print("  !! 字符框 > 9 个但只有 1 个车牌框的图：%d 张（%.1f%%）—— 疑似漏标第二块车牌"
              % (two_plate, 100.0 * two_plate / max(n_img, 1)))
    if no_plate:
        print("  !! 有字符框但无车牌框的图：%d 张（%.1f%%）" % (no_plate, 100.0 * no_plate / max(n_img, 1)))

    print("\n  --- 字符框在【原图】上的像素尺寸 ---")
    print("     宽 %s" % pct(char_w0))
    print("     高 %s" % pct(char_h0))
    print("  --- 字符框缩到 %d×%d 后的像素尺寸（模型真正看到的）---" % (args.size, args.size))
    print("     宽 %s" % pct(char_w))
    print("     高 %s" % pct(char_h))
    ar = np.array(char_h) / np.maximum(np.array(char_w), 1e-6)
    print("     高/宽 中位 %.2f  （1:1 附近=正方，越大=越窄长）" % np.median(ar))
    n_small = int((np.array(char_w) < 12).sum())
    print("     **宽度 < 12px 的字符框占比 %.1f%%**  < 8px 占比 %.1f%%"
          % (100.0 * n_small / max(len(char_w), 1),
             100.0 * int((np.array(char_w) < 8).sum()) / max(len(char_w), 1)))

    print("\n  --- 车牌框在【原图】上的像素尺寸 ---")
    print("     宽 %s" % pct(plate_w))
    print("     高 %s" % pct(plate_h))


if __name__ == "__main__":
    main()
