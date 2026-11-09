"""判断主导错位掩码 XXX..XX 到底是"整行右移一位"还是"逐位独立误读"。

做法: 对每轮读到的 7 字串, 和真值 粤R888G8 做最佳对齐 (shift -2..+2),
统计有多少轮是**整体平移**, 再看不是平移的那些轮里逐位错在哪。

  python -I _log_shift.py [log.txt]
"""
import os
import re
import sys
import math
from collections import Counter

# 交付包不含 log/：把采集到的日志路径作为第一个参数传入；默认值相对仓库根。
LOG = sys.argv[1] if len(sys.argv) > 1 else os.path.join("mcu_code", "log", "log.txt")
OUT = LOG + ".shift.txt"

RE_CAS = re.compile(
    r"cas (\w+) box (-?\d+),(-?\d+) (-?\d+),(-?\d+) crop (-?\d+),(-?\d+) (-?\d+),(-?\d+) "
    r"n (\d+) cl (\d+) m (-?\d+) sn ([0-9A-Fa-f]+) vp (-?\d+),(-?\d+),(-?\d+),(-?\d+)")
RE_PLATE = re.compile(r"^#(\d+) cas plate: (.*) n (\d+) grab")
RE_FLIP = re.compile(r"^flip (\d+): (-?\d+) -> (-?\d+) m (-?\d+) box")

TRUTH = "粤R888G8"


def main():
    rows, cur = [], None
    for line in open(LOG, encoding="utf-8", errors="replace"):
        line = line.rstrip("\n")
        m = RE_PLATE.match(line)
        if m:
            cur = {"idx": int(m.group(1)), "text": m.group(2), "tn": int(m.group(3)), "flip": []}
            rows.append(cur)
            continue
        if cur is None:
            continue
        m = RE_FLIP.match(line)
        if m:
            cur["flip"].append((int(m.group(1)), int(m.group(2)), int(m.group(3)), int(m.group(4))))
            continue
        m = RE_CAS.match(line)
        if m and "bw" not in cur:
            cur.update(bw=int(m.group(4)) - int(m.group(2)),
                       cw=int(m.group(8)) - int(m.group(6)),
                       ch=int(m.group(9)) - int(m.group(7)),
                       n=int(m.group(10)), cls=int(m.group(11)), m_=int(m.group(12)),
                       sn=int(m.group(13), 16), pitch=int(m.group(14)),
                       margin=int(m.group(15)), slope=int(m.group(16)),
                       roll=int(m.group(17)))

    n7 = [r for r in rows if "bw" in r and r["n"] == 7]
    out = ["n==7 轮: %d / %d" % (len(n7), len(rows))]

    # ---- 1. 整体平移判定 ----
    def best_shift(t):
        """返回 (shift, score). shift>0 = 读出来比真值**右移**了 (前面多一个字)."""
        best = (-99, -1)
        for s in range(-3, 4):
            sc = 0
            for j in range(7):
                k = j - s
                if 0 <= k < 7 and t[j] == TRUTH[k]:
                    sc += 1
            if sc > best[1]:
                best = (s, sc)
        return best

    hist = Counter()
    for r in n7:
        t = r["text"]
        if len(t) != 7:
            hist["len!=7"] += 1
            continue
        s, sc = best_shift(t)
        # 只有 "平移后 7 位全中" 才算整行平移; 否则记下平移后还剩几位对
        hist[(s, sc)] += 1

    out.append("\n--- 对齐 (shift, 平移后命中数) 分布 ---")
    out.append("  shift>0 = 读出的串整体右移, 前面多塞了一个框")
    for (s, sc), k in sorted(hist.items(), key=lambda kv: -kv[1])[:16]:
        if isinstance(s, str):
            out.append("  %-12s %3d" % (s, k))
        else:
            out.append("  shift %+d, 命中 %d/7   %3d" % (s, sc, k))

    # ---- 2. 剔除"整行平移"后的逐位准确率 ----
    shifted = [r for r in n7 if len(r["text"]) == 7 and best_shift(r["text"]) == (1, 7)]
    noshift = [r for r in n7 if len(r["text"]) == 7 and best_shift(r["text"]) != (1, 7)]
    out.append("\n--- 整行右移一位: %d 轮 (%.1f%%) ---" % (
        len(shifted), 100.0 * len(shifted) / max(1, len(n7))))
    out.append("  例子: %s" % ", ".join(r["text"] for r in shifted[:6]))

    out.append("\n--- 去掉整行右移后, 逐位准确率 (n=%d) ---" % len(noshift))
    exact = 0
    for i in range(7):
        ok = sum(1 for r in noshift if r["text"][i] == TRUTH[i])
        wrong = Counter(r["text"][i] for r in noshift if r["text"][i] != TRUTH[i])
        out.append("  位%d %s  %3d/%3d = %5.1f%%  误读: %s" % (
            i, TRUTH[i], ok, len(noshift), 100.0 * ok / max(1, len(noshift)),
            " ".join("%s×%d" % kv for kv in wrong.most_common(4))))
    exact = sum(1 for r in noshift if r["text"] == TRUTH)
    out.append("  exact = %d/%d = %.1f%%" % (exact, len(noshift),
                                             100.0 * exact / max(1, len(noshift))))

    # ---- 3. 整行右移跟什么量相关 ----
    out.append("\n--- 整行右移 vs 其它量 (中位数) ---")
    for name, sel in (("右移", shifted), ("没右移", noshift)):
        if not sel:
            continue
        def med(k):
            v = sorted(r[k] for r in sel)
            return v[len(v) // 2]
        out.append("  %-6s n=%3d  pitch %3d  margin %3d  roll %+4d  slope %+5d  sn非零 %2d  bw %3d"
                   % (name, len(sel), med("pitch"), med("margin"), med("roll"),
                      med("slope"), sum(1 for r in sel if r["sn"]),
                      med("bw")))

    # ---- 4. sn 置位情况 (格点投影) ----
    out.append("\n--- sn (bit0..6 = 该位残差 > 0.15*pitch, bit31 = 投影过) ---")
    snh = Counter(r["sn"] & 0xFF for r in n7)
    for v, k in sorted(snh.items(), key=lambda kv: -kv[1])[:10]:
        out.append("  %08X  %3d  (位: %s)" % (v, k, "".join(str(i) for i in range(7) if v >> i & 1)))
    out.append("  bit31 投影过的轮数: %d / %d" % (sum(1 for r in n7 if r["sn"] >> 31 & 1), len(n7)))

    # ---- 5. flip 行: 每轮被改了几位 ----
    out.append("\n--- flip 行统计 (③ 每轮改判的位数) ---")
    fh = Counter(len(r["flip"]) for r in n7)
    out.append("  " + "  ".join("%d位:%d轮" % (k, fh[k]) for k in sorted(fh)))
    cnt = Counter()
    for r in n7:
        for pos, a, b, mg in r["flip"]:
            cnt[(pos, a, b)] += 1
    out.append("  最常见的改判 (位, 旧类 -> 新类, 次数):")
    for (pos, a, b), k in cnt.most_common(12):
        out.append("    位%d  %d -> %d   ×%d" % (pos, a, b, k))

    with open(OUT, "w", encoding="utf-8") as f:
        f.write("\n".join(out) + "\n")
    print("written", OUT)


if __name__ == "__main__":
    main()
