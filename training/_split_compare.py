# -*- coding: utf-8 -*-
"""对比 data_crop 的 train / val 两块，看那 514 张「连续尾块」能不能代表整个数据集。

背景：划分不是随机的，是把源数据按文件名排序后**连续切块**（train 0000-4458、
val 4459-4979）。如果源数据本身有序（采集批次/时间/场地），val 就会系统性地偏。
这个脚本只读 label txt，不做推理，几秒出结果。
"""
import os, sys, glob
import numpy as np

try:
    sys.stdout.reconfigure(encoding = "utf-8")
except Exception:
    pass

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)


def load_split(d):
    """返回每张图的 (字符数, 框宽px 中位, 框高px 中位, 图宽, 图高)。"""
    rows = []
    for f in sorted(glob.glob(os.path.join(d, "*.txt"))):
        img = f[:-4] + ".jpg"
        try:
            import struct
            with open(img, "rb") as fh:
                fh.read(2)
                while True:
                    b = fh.read(1)
                    if not b:
                        raise ValueError
                    if b == b"\xff":
                        m = fh.read(1)
                        if m in (b"\xc0", b"\xc1", b"\xc2"):
                            fh.read(3)
                            h, w = struct.unpack(">HH", fh.read(4))
                            break
                        fh.read(struct.unpack(">H", fh.read(2))[0] - 2)
        except Exception:
            continue
        boxes = []
        for ln in open(f, encoding = "utf-8", errors = "ignore"):
            p = ln.split()
            if len(p) != 5:
                continue
            boxes.append([float(v) for v in p[1:]])
        if not boxes:
            continue
        b = np.array(boxes)                       # cx cy w h（归一化）
        rows.append((len(b), float(np.median(b[:, 2] * w)),
                     float(np.median(b[:, 3] * h)), w, h))
    return np.array(rows, dtype = float)


def report(name, r):
    n = len(r)
    cnt = r[:, 0]
    print("\n=== %s  (%d 张) ===" % (name, n))
    print("  每图字符数：均值 %.3f  中位 %.0f" % (cnt.mean(), np.median(cnt)))
    tot = cnt.sum()
    for k in sorted(set(cnt.tolist())):
        m = int((cnt == k).sum())
        print("    %d 字: %4d 张 (%.2f%%)   占框数 %.2f%%"
              % (int(k), m, 100.0 * m / n, 100.0 * (m * k) / tot))
    print("  框宽 px：中位 %.1f  均值 %.1f   p10 %.1f  p90 %.1f"
          % (np.median(r[:, 1]), r[:, 1].mean(),
             np.percentile(r[:, 1], 10), np.percentile(r[:, 1], 90)))
    print("  框高 px：中位 %.1f  均值 %.1f   p10 %.1f  p90 %.1f"
          % (np.median(r[:, 2]), r[:, 2].mean(),
             np.percentile(r[:, 2], 10), np.percentile(r[:, 2], 90)))
    print("  图宽 px：中位 %.1f   图高 px：中位 %.1f" % (np.median(r[:, 3]), np.median(r[:, 4])))
    return cnt


if __name__ == "__main__":
    tr = load_split("data_crop/train")
    va = load_split("data_crop/val")
    ct = report("train", tr)
    cv = report("val", va)

    print("\n" + "=" * 70)
    print("=== 判读 ===")
    # 字符数分布的总变差距离（0=同分布，1=完全不重叠）
    ks = sorted(set(ct.tolist()) | set(cv.tolist()))
    pt = np.array([(ct == k).mean() for k in ks])
    pv = np.array([(cv == k).mean() for k in ks])
    print("  字符数分布 TV 距离 = %.4f  (0 = 同分布)" % (0.5 * np.abs(pt - pv).sum()))
    print("  **按框数加权**的字符数均值： train %.4f   val %.4f"
          % ((ct * (tr[:, 0])).sum() / tr[:, 0].sum(), 0))
    print("  train 每图字符数 %.3f  vs  val %.3f   差 %.3f"
          % (ct.mean(), cv.mean(), ct.mean() - cv.mean()))

    # val 内部按文件序号分 4 块，看有没有单调趋势（§3.4 说 val 是按难度排过序的）
    print("\n  val 内部按序号分 4 块（看「难度排序」是否真的存在）:")
    for i in range(4):
        a, b = i * len(cv) // 4, (i + 1) * len(cv) // 4
        print("    块%d (idx %3d-%3d): 每图字符数均值 %.3f" % (i, a, b - 1, cv[a:b].mean()))
