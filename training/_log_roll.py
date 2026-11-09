"""把上板日志里的 `cas` / `flip` 行按轮拆开, 看横滚闭环到底在做什么。

用法: python -I _log_roll.py [log.txt]
"""
import os
import re
import sys

# 交付包不含 log/：把采集到的日志路径作为第一个参数传入；默认值相对仓库根。
LOG = sys.argv[1] if len(sys.argv) > 1 else os.path.join("mcu_code", "log", "log.txt")
OUT = LOG + ".roll.txt"

RE_CAS = re.compile(
    r"cas (\w+) box (-?\d+),(-?\d+) (-?\d+),(-?\d+) crop (-?\d+),(-?\d+) (-?\d+),(-?\d+) "
    r"n (\d+) cl (\d+) m (-?\d+) sn ([0-9A-Fa-f]+) vp (-?\d+),(-?\d+),(-?\d+),(-?\d+)")
RE_PLATE = re.compile(r"^#(\d+) cas plate: (.*) n (\d+) grab")


def main():
    rows = []
    cur = None
    with open(LOG, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.rstrip("\n")
            m = RE_PLATE.match(line)
            if m:
                cur = {"idx": int(m.group(1)), "text": m.group(2), "tn": int(m.group(3))}
                rows.append(cur)
                continue
            m = RE_CAS.match(line)
            if m and cur is not None and "bw" not in cur:
                bw = int(m.group(4)) - int(m.group(2))
                cw = int(m.group(8)) - int(m.group(6))
                ch = int(m.group(9)) - int(m.group(7))
                cur.update(
                    fresh=m.group(1) == "fresh", bw=bw, cw=cw, ch=ch,
                    n=int(m.group(10)), cls=int(m.group(11)),
                    pitch=int(m.group(14)), margin=int(m.group(15)),
                    slope=int(m.group(16)), roll=int(m.group(17)),
                    chg=int(re.search(r"chg (-?\d+)", line).group(1)))

    import math
    from collections import Counter

    out = []
    out.append("rounds with vp: %d / %d" % (sum(1 for r in rows if "bw" in r), len(rows)))

    c = Counter(r["text"] for r in rows if "bw" in r)
    out.append("\n--- 字符串分布 (前 20) ---")
    for s, k in c.most_common(20):
        out.append("  %-14s %d" % (s, k))

    # ---- 逐位准确率 (真值按本场次唯一那块牌) ----
    TRUTH = "粤R888G8"
    n7 = [r for r in rows if "bw" in r and r["n"] == 7]
    out.append("\n--- 逐位准确率 (truth = %s, 只统计 n==7 的 %d 轮) ---" % (TRUTH, len(n7)))
    exact = 0
    pos_ok = [0] * 7
    pos_tot = [0] * 7
    by_pos = [{} for _ in range(7)]
    for r in n7:
        t = r["text"]
        if t == TRUTH:
            exact += 1
        if len(t) != 7:
            continue
        for i in range(7):
            pos_tot[i] += 1
            c = t[i]
            if c == TRUTH[i]:
                pos_ok[i] += 1
            else:
                by_pos[i][c] = by_pos[i].get(c, 0) + 1
    out.append("  exact = %d / %d = %.1f%%" % (exact, len(n7), 100.0 * exact / max(1, len(n7))))
    for i in range(7):
        wrong = sorted(by_pos[i].items(), key=lambda kv: -kv[1])[:4]
        out.append("  位%d 真值 %s 正确 %3d/%3d = %5.1f%%   误读: %s" % (
            i, TRUTH[i], pos_ok[i], pos_tot[i], 100.0 * pos_ok[i] / max(1, pos_tot[i]),
            " ".join("%s×%d" % (c, k) for c, k in wrong)))

    # 哪些位一起错 —— 判断「整行错位」还是「逐位独立误读」的唯一直接证据
    mask_hist = {}
    for r in n7:
        t = r["text"]
        if len(t) != 7:
            mask_hist["len!=7"] = mask_hist.get("len!=7", 0) + 1
            continue
        mask = "".join("." if t[i] == TRUTH[i] else "X" for i in range(7))
        mask_hist[mask] = mask_hist.get(mask, 0) + 1
    out.append("\n--- 错位掩码 (X = 该位错) 前 14 ---")
    for m, k in sorted(mask_hist.items(), key=lambda kv: -kv[1])[:14]:
        out.append("  %-10s %3d" % (m, k))

    # 有 roll / 无 roll 的对照 (|roll| < 5 视为没转)
    out.append("\n--- 按 roll 分桶 ---")
    for name, lo, hi in (("roll==0", -5, 5), ("roll 5..25", 5, 25), ("roll>=25", 25, 9999)):
        sel = [r for r in n7 if lo <= r["roll"] < hi]
        if not sel:
            continue
        e = sum(1 for r in sel if r["text"] == TRUTH)
        out.append("  %-10s n=%3d  exact %3d = %5.1f%%  pitch中位 %d  margin中位 %d" % (
            name, len(sel), e, 100.0 * e / len(sel),
            int(sorted(r["pitch"] for r in sel)[len(sel) // 2]),
            int(sorted(r["margin"] for r in sel)[len(sel) // 2])))

    out.append("\n--- 横滚闭环: 每轮 roll / slope / 帧里角度 ---")
    out.append("  #   plate        n  roll slope  ch/cw 帧角(deg)  pitch mar")
    for r in rows:
        if "bw" not in r:
            continue
        ch_cw = r["ch"] / r["cw"]
        deg = r["slope"] / 1000.0 * ch_cw
        out.append("  %-4d %-12s %d  %+4d %+5d %6.3f %+7.2f    %3d %3d" % (
            r["idx"], r["text"], r["n"], r["roll"], r["slope"], ch_cw,
            math.degrees(math.atan(deg)), r["pitch"], r["margin"]))

    txt = "\n".join(out)
    with open(OUT, "w", encoding="utf-8") as f:
        f.write(txt + "\n")
    print("written", OUT)


if __name__ == "__main__":
    main()
