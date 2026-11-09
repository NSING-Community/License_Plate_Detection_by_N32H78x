"""按轮列表 + 逐位统计 (支持一次采集里出现多块牌)。

用法: python -I _log_rounds.py [log.txt]
"""
import os
import re
import sys
from collections import Counter

# 交付包不含 log/：把采集到的日志路径作为第一个参数传入；默认值相对仓库根。
LOG = sys.argv[1] if len(sys.argv) > 1 else os.path.join("mcu_code", "log", "log.txt")
OUT = LOG + ".rounds.txt"

RE_PLATE = re.compile(r"^#(\d+) cas plate: (.*) n (\d+) grab")
RE_CAS = re.compile(
    r"cas (\w+) box (-?\d+),(-?\d+) (-?\d+),(-?\d+) crop (-?\d+),(-?\d+) (-?\d+),(-?\d+) "
    r"n (\d+) cl (\d+) m (-?\d+) sn ([0-9A-Fa-f]+) vp (-?\d+),(-?\d+),(-?\d+),(-?\d+)")
# 行尾的可选字段 (2026-10-20 起有): chg / why / n2。 老日志没有, 允许缺。
# n2 有两种写法: 老固件 `n2 8`, 新固件 `n2 8/64` (NMS 后 / NMS 前)。
RE_TAIL = re.compile(r"chg (-?\d+)(?: why (-?\d+) n2 (-?\d+)(?:/(\d+))?)?")

TRUTHS = ["粤R888G8", "琼A3M286"]


def nearest_truth(t):
    """按最长公共位判这轮读的是哪块牌。"""
    if not t or t == "(no char)":
        return None, 0
    best, bs = None, -1
    for tr in TRUTHS:
        s = sum(1 for i in range(min(len(t), len(tr))) if t[i] == tr[i])
        if s > bs:
            bs, best = s, tr
    return best, bs


def main():
    rows, cur = [], None
    for line in open(LOG, encoding="utf-8", errors="replace"):
        line = line.rstrip("\n")
        m = RE_PLATE.match(line)
        if m:
            cur = {"idx": int(m.group(1)), "text": m.group(2), "tn": int(m.group(3))}
            rows.append(cur)
            continue
        if cur is None:
            continue
        m = RE_CAS.match(line)
        if m and "pitch" not in cur:
            cur.update(fresh=(m.group(1) == "fresh"),
                       bw=int(m.group(4)) - int(m.group(2)),
                       bh=int(m.group(5)) - int(m.group(3)),
                       n=int(m.group(10)), q=int(m.group(12)),
                       sn=int(m.group(13), 16),
                       pitch=int(m.group(14)), margin=int(m.group(15)),
                       slope=int(m.group(16)), roll=int(m.group(17)),
                       chg=-99, why=-1, n2=-1, nraw=-1)
        if m:
            t = RE_TAIL.search(line)
            if t:
                cur["chg"] = int(t.group(1))
                if t.group(2) is not None:
                    cur["why"] = int(t.group(2))
                    cur["n2"] = int(t.group(3))
                    if t.group(4) is not None:
                        cur["nraw"] = int(t.group(4))

    out = []
    out.append("轮数 %d" % len(rows))
    out.append("")
    out.append("轮  n  fresh 串            判定        pitch mar roll slope  sn        chg why n2 raw")
    ok = bad = nogot = 0
    for r in rows:
        t = r["text"]
        tr, sc = nearest_truth(t)
        if r.get("tn", 0) == 0 or t == "(no char)":
            verdict = "无字"
            nogot += 1
        elif t == tr:
            verdict = "对"
            ok += 1
        else:
            verdict = "错(真值%s)" % tr
            bad += 1
        out.append("#%-3d %d  %-5s %-13s %-10s %3d %3d %+4d %+5d  %08X %3d %3d %2d %3d" % (
            r["idx"], r.get("n", -1), "fresh" if r.get("fresh") else "reuse",
            t, verdict, r.get("pitch", -1), r.get("margin", -1),
            r.get("roll", 0), r.get("slope", 0), r.get("sn", 0),
            r.get("chg", -99), r.get("why", -1), r.get("n2", -1), r.get("nraw", -1)))

    out.append("")
    out.append("整串: 对 %d / 错 %d / 无字 %d   (总 %d)" % (ok, bad, nogot, len(rows)))
    nz = ok + bad
    if nz:
        out.append("有串的轮里整串正确率 %.1f%%" % (100.0 * ok / nz))

    # (no char) 的原因: 1 = ① 一个框都没有 / 2 = 形状闸拒了 / -1 = 老日志没这字段
    noc = [r for r in rows if r.get("tn", 0) == 0 or r["text"] == "(no char)"]
    if noc:
        names = {1: "①无框", 2: "形状闸", -1: "老日志"}
        cnt = Counter(r.get("why", -1) for r in noc)
        out.append("无字 %d 轮, 原因: %s" % (
            len(noc), "  ".join("%s:%d" % (names.get(k, str(k)), v) for k, v in cnt.most_common())))

    # ② 的原始框数 (截到 7 之前)。 > n 就是这一轮框多了
    ex = [r for r in rows if r.get("n2", -1) >= 0]
    if ex:
        cnt = Counter(r["n2"] for r in ex)
        out.append("② 原始框数 n2 分布: " + "  ".join("%d:%d" % kv for kv in sorted(cnt.items())))
        over = [r for r in ex if r["n2"] > r.get("n", 0)]
        out.append("n2 > nchar 的轮: %d / %d" % (len(over), len(ex)))

    # NMS 之前的候选数 (新固件 n2 a/b 的 b)。 == 64 就是光栅序截断卡住了
    raw = [r for r in rows if r.get("nraw", -1) >= 0]
    if raw:
        capped = [r for r in raw if r["nraw"] > 64]
        out.append("② 过 conf 的候选总数: 中位 %d, 最大 %d; 超过 64 的轮: %d / %d" % (
            sorted(r["nraw"] for r in raw)[len(raw) // 2],
            max(r["nraw"] for r in raw), len(capped), len(raw)))

    # 逐位: 按每块牌各自统计
    out.append("")
    for tr in TRUTHS:
        sel = [r for r in rows if len(r["text"]) == len(tr) and nearest_truth(r["text"])[0] == tr]
        if not sel:
            continue
        out.append("--- %s : %d 轮 ---" % (tr, len(sel)))
        for i in range(len(tr)):
            good = sum(1 for r in sel if r["text"][i] == tr[i])
            wrong = Counter(r["text"][i] for r in sel if r["text"][i] != tr[i])
            out.append("  位%d %s  %3d/%3d = %5.1f%%  %s" % (
                i, tr[i], good, len(sel), 100.0 * good / len(sel),
                " ".join("%s×%d" % kv for kv in wrong.most_common(3))))
        out.append("")

    # n 分布
    out.append("n 分布: " + "  ".join("%d:%d" % kv for kv in sorted(Counter(r["text"] and r.get("n", -1) for r in rows).items())))

    with open(OUT, "w", encoding="utf-8") as f:
        f.write("\n".join(out) + "\n")
    print("written", OUT)


if __name__ == "__main__":
    main()
