"""找「哪一轮是对的、哪一轮是错的」到底跟什么量相关。

每一轮记: pitch / margin / roll / 位0..6 对不对 / 与上一轮的帧内变化量。
重点是 **框在 320 空间里到底抖多少** (flip 行给了旧框和新框)。
"""
import os
import re
import sys
from collections import Counter, defaultdict

# 交付包不含 log/：把采集到的日志路径作为第一个参数传入；默认值相对仓库根。
LOG = sys.argv[1] if len(sys.argv) > 1 else os.path.join("mcu_code", "log", "log.txt")
OUT = LOG + ".whatif.txt"
TRUTH = "粤R888G8"

RE_CAS = re.compile(
    r"cas (\w+) box (-?\d+),(-?\d+) (-?\d+),(-?\d+) crop (-?\d+),(-?\d+) (-?\d+),(-?\d+) "
    r"n (\d+) cl (\d+) m (-?\d+) sn ([0-9A-Fa-f]+) vp (-?\d+),(-?\d+),(-?\d+),(-?\d+) "
    r"\| p (\d+) c (\d+) rs (\d+) i2 (\d+) i3 (\d+) chg (-?\d+)")
RE_PLATE = re.compile(r"^#(\d+) cas plate: (.*) n (\d+) grab")
RE_FLIP = re.compile(r"^flip (\d+): (-?\d+) -> (-?\d+) m (-?\d+) box (-?\d+),(-?\d+) (-?\d+),(-?\d+) was (-?\d+),(-?\d+) (-?\d+),(-?\d+)")


def main():
    rows, cur = [], None
    for line in open(LOG, encoding="utf-8", errors="replace"):
        line = line.rstrip("\n")
        m = RE_PLATE.match(line)
        if m:
            cur = {"idx": int(m.group(1)), "text": m.group(2), "flips": []}
            rows.append(cur)
            continue
        if cur is None:
            continue
        m = RE_FLIP.match(line)
        if m:
            cur["flips"].append({
                "pos": int(m.group(1)), "old": int(m.group(2)), "new": int(m.group(3)),
                "mg": int(m.group(4)),
                "newc": (int(m.group(5)) + int(m.group(7))) / 2.0,
                "oldc": (int(m.group(9)) + int(m.group(11))) / 2.0,
                "neww": int(m.group(7)) - int(m.group(5)),
                "oldw": int(m.group(11)) - int(m.group(9)),
            })
            continue
        m = RE_CAS.match(line)
        if m and "pitch" not in cur:
            cur.update(reused=(m.group(1) == "reuse"),
                       bw=int(m.group(4)) - int(m.group(2)),
                       bh=int(m.group(5)) - int(m.group(3)),
                       cw=int(m.group(8)) - int(m.group(6)),
                       chh=int(m.group(9)) - int(m.group(7)),
                       n=int(m.group(10)), cls=int(m.group(11)), mmin=int(m.group(12)),
                       sn=int(m.group(13), 16), pitch=int(m.group(14)),
                       margin=int(m.group(15)), slope=int(m.group(16)), roll=int(m.group(17)),
                       t2=int(m.group(21)), chg=int(m.group(23)))

    n7 = [r for r in rows if "pitch" in r and r["n"] == 7]
    out = ["n==7: %d" % len(n7)]

    def exact(r):
        return r["text"] == TRUTH

    # ---- A. 框的抖动: flip 行里 |新中心 - 旧中心| / 宽度变化 ----
    d = [f["newc"] - f["oldc"] for r in n7 for f in r["flips"]]
    dw = [f["neww"] - f["oldw"] for r in n7 for f in r["flips"]]
    out.append("\n--- 翻字时框动了多少 (320 空间, 共 %d 次) ---" % len(d))
    if d:
        a = sorted(abs(x) for x in d)
        out.append("  |Δ中心| px: p50 %.1f  p90 %.1f  max %.1f" % (
            a[len(a) // 2], a[int(len(a) * .9)], a[-1]))
        b = sorted(abs(x) for x in dw)
        out.append("  |Δ宽|   px: p50 %.1f  p90 %.1f  max %.1f" % (
            b[len(b) // 2], b[int(len(b) * .9)], b[-1]))
        big = sum(1 for x in a if x > 5)
        out.append("  |Δ中心| > 5 px (=pitch 的 1/5) 的占 %.0f%%" % (100.0 * big / len(a)))

    # 翻字时的 ③ margin 分布 —— 小 = ③ 在边界上
    mgs = [f["mg"] for r in n7 for f in r["flips"]]
    out.append("\n--- 翻字那一格的 ③ margin ---")
    if mgs:
        c = Counter("0-5" if m < 6 else "6-15" if m < 16 else "16-30" if m < 31 else "31+" for m in mgs)
        out.append("  " + "  ".join("%s:%d" % kv for kv in sorted(c.items())))

    # ---- B. 按 sn / 是否 fresh / chg 分组 ----
    out.append("\n--- 分组 exact 率 ---")
    groups = defaultdict(list)
    for r in n7:
        groups["全部"].append(r)
        groups["fresh" if not r["reused"] else "reuse"].append(r)
        groups["投影过" if r["sn"] >> 31 & 1 else "没投影"].append(r)
        groups["chg>=5" if abs(r["chg"]) >= 5 else "chg<5"].append(r)
        groups["3D/4D 全对" if (r["text"][3] == "8" and r["text"][4] == "8") else "3D/4D 有错"].append(r)
        groups["位5=G" if len(r["text"]) == 7 and r["text"][5] == "G" else "位5≠G"].append(r)
    for k in ("全部", "fresh", "reuse", "投影过", "没投影", "chg>=5", "chg<5",
              "3D/4D 全对", "3D/4D 有错", "位5=G", "位5≠G"):
        s = groups[k]
        if not s:
            continue
        out.append("  %-12s n=%3d  exact %3d = %5.1f%%  pitch中位 %3d  margin中位 %3d  m中位 %3d" % (
            k, len(s), sum(1 for r in s if exact(r)), 100.0 * sum(1 for r in s if exact(r)) / len(s),
            sorted(r["pitch"] for r in s)[len(s) // 2],
            sorted(r["margin"] for r in s)[len(s) // 2],
            sorted(r["mmin"] for r in s)[len(s) // 2]))

    # ---- C. 位5/位6 的相关性: 是"对换"还是"独立误读" ----
    out.append("\n--- 位5/位6 联合分布 (真值 G,8) ---")
    j = Counter((r["text"][5], r["text"][6]) for r in n7 if len(r["text"]) == 7)
    for k, v in j.most_common(8):
        out.append("  位5=%s 位6=%s   %3d" % (k[0], k[1], v))

    # ---- D. 位0/1/2 联合: 是否整段右移 ----
    out.append("\n--- 位0/1/2 联合分布 (真值 粤,R,8) ---")
    j = Counter((r["text"][0], r["text"][1], r["text"][2]) for r in n7 if len(r["text"]) == 7)
    for k, v in j.most_common(10):
        out.append("  %s %s %s   %3d" % (k[0], k[1], k[2], v))

    # ---- E. 轮间 pitch 变化 vs exact ----
    out.append("\n--- 轮间 pitch 变化 |Δpitch| vs exact ---")
    prev = None
    buckets = defaultdict(lambda: [0, 0])
    for r in rows:
        if "pitch" not in r or r["n"] != 7:
            prev = r if "pitch" in r else prev
            continue
        if prev is not None and "pitch" in prev:
            dp = abs(r["pitch"] - prev["pitch"])
            b = "0-10" if dp <= 10 else "11-25" if dp <= 25 else "26-50" if dp <= 50 else ">50"
            buckets[b][0] += 1
            buckets[b][1] += 1 if exact(r) else 0
        prev = r
    for b in ("0-10", "11-25", "26-50", ">50"):
        if b in buckets:
            n, e = buckets[b]
            out.append("  |Δpitch| %-6s n=%3d  exact %5.1f%%" % (b, n, 100.0 * e / n))

    with open(OUT, "w", encoding="utf-8") as f:
        f.write("\n".join(out) + "\n")
    print("written", OUT)


if __name__ == "__main__":
    main()
