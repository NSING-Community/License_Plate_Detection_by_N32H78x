# -*- coding: utf-8 -*-
"""train / val 的**字符分布**对比（只读 label txt，不做推理，几秒出结果）。

背景：`_split_compare.py` 发现 val 是「连续尾块」，6 字牌被富集了 27 倍。
但串长只是表象 —— 如果 val 里还富集了**某一类车牌**（警/学/特定省份），
那 514 张上的数字就不只是「长度分布偏」，而是「换了个子population」。
这个脚本把 train / val 每个字符的出现率拉出来对，按「val 富集倍数」排序。
"""
import os, sys, glob
from collections import Counter

try:
    sys.stdout.reconfigure(encoding = "utf-8")
except Exception:
    pass

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)


def load_names(p):
    out = []
    for ln in open(p, encoding = "gbk", errors = "ignore"):
        ln = ln.strip()
        if ln:
            out.append(ln)
    return out


def count(d, names):
    """返回 (每字符框数 Counter, 图数, 含该字符的图数 Counter)。"""
    box = Counter()
    img = Counter()
    n_img = 0
    for f in sorted(glob.glob(os.path.join(d, "*.txt"))):
        seen = set()
        hit = False
        for ln in open(f, encoding = "utf-8", errors = "ignore"):
            p = ln.split()
            if len(p) != 5:
                continue
            c = int(float(p[0]))
            if c <= 0:                       # class 0 = 车牌整框，不是字符
                continue
            ch = names[c] if 0 <= c < len(names) else "?"
            box[ch] += 1
            seen.add(ch)
            hit = True
        if hit:
            n_img += 1
        for ch in seen:
            img[ch] += 1
    return box, n_img, img


if __name__ == "__main__":
    names = load_names("data/voc.names")
    print("类别表 %d 个（含 plate）" % len(names))

    bt, nt, it = count("data_crop/train", names)
    bv, nv, iv = count("data_crop/val", names)
    print("train %d 张 / %d 个字符框    val %d 张 / %d 个字符框"
          % (nt, sum(bt.values()), nv, sum(bv.values())))

    # 按「在 val 的图里出现率 / 在 train 的图里出现率」排序
    rows = []
    for ch in set(bt) | set(bv):
        rt = it[ch] / max(nt, 1)
        rv = iv[ch] / max(nv, 1)
        ratio = (rv / rt) if rt > 0 else float("inf")
        rows.append((ratio, ch, it[ch], nt, iv[ch], nv, rt, rv))
    rows.sort(key = lambda r: -r[0])

    print("\n=== val 相对 train **富集**的字符（含该字符的图占比，前 25）===")
    print("  %-4s %-10s %-22s %-22s %s"
          % ("字", "富集倍数", "train 图/总", "val 图/总", "train框 val框"))
    for ratio, ch, t, tn, v, vn, rt, rv in rows[:25]:
        if rv < 0.005 and rt < 0.005:
            continue
        print("  %-4s %-10s %5d/%d = %6.3f%%      %5d/%d = %6.3f%%      %5d %5d"
              % (ch, ("%.1fx" % ratio) if ratio != float("inf") else "∞",
                 t, tn, 100 * rt, v, vn, 100 * rv, bt[ch], bv[ch]))

    print("\n=== val 相对 train **缺失/减少**的字符（最负的 15）===")
    for ratio, ch, t, tn, v, vn, rt, rv in rows[-15:][::-1]:
        print("  %-4s %-10s %5d/%d = %6.3f%%      %5d/%d = %6.3f%%      %5d %5d"
              % (ch, ("%.2fx" % ratio) if ratio != float("inf") else "∞",
                 t, tn, 100 * rt, v, vn, 100 * rv, bt[ch], bv[ch]))
