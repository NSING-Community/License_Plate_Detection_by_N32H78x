# -*- coding: utf-8 -*-
"""
One-off (underscore = the established practice, see tools/README conventions).

Reads the `box k: x1,y1 x2,y2 cls C sc S` blocks that key `b` dumps in
mcu_code/log/log.txt - one block per triggered frame - and scores candidate
rules for yolo_plate_read() against EACH OTHER on the SAME frames.

The point is that one frame cannot choose a rule: on a single frame every rule
that deletes the obvious junk looks right.  Ten frames of one scene can, because
the plate is the same and only the detector's jitter moves.

Rules are applied to the box list in the order the string is built (nms order =
score descending, first 16), then re-sorted by x1, exactly as
yolo_inference.cpp:206-243 does.  Output is truncated at 12 BYTES like the
firmware (YOLO_PLATE_MAXLEN), and a province glyph is 2 GBK bytes.

Usage:  C:\\Users\\1\\anaconda3\\python.exe _plate_rule.py [log.txt]
"""
import re
import sys
import os

CHARS = "0123456789ABCDEFGHJKLMNPQRSTUVWXYZ"   # class 1..34
MAXBOX = 16
MAXLEN = 12

BOX_RE = re.compile(
    r"^box\s+(\d+):\s+(\d+),(\d+)\s+(\d+),(\d+)\s+cls\s+(\d+)\s+sc\s+(\d+)\s*$")


def load_frames(path):
    """One entry per trigger: the boxes in the order the firmware printed them."""
    frames = []
    cur = None
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            line = line.rstrip("\r\n")
            if line.startswith("plate: "):
                # a new frame starts here; the boxes follow
                cur = {"plate": line[len("plate: "):], "boxes": []}
                frames.append(cur)
                continue
            m = BOX_RE.match(line)
            if m and cur is not None:
                cur["boxes"].append({
                    "x1": int(m.group(2)), "y1": int(m.group(3)),
                    "x2": int(m.group(4)), "y2": int(m.group(5)),
                    "cls": int(m.group(6)), "sc": int(m.group(7)),
                })
    return frames


def token(cid):
    """The bytes this class contributes, as a printable token."""
    if 1 <= cid <= 34:
        return CHARS[cid - 1], 1
    if cid == 66:
        return "粤", 2
    if 35 <= cid <= 69:
        return "P%d" % cid, 2        # some other province
    return None, 0                    # class 0 = the plate outline


def render(boxes_in_nms_order):
    """The firmware's assembly, verbatim."""
    idx = [b for b in boxes_in_nms_order if b["cls"] > 0][:MAXBOX]
    idx.sort(key=lambda b: b["x1"])
    out = []
    n = 0
    for b in idx:
        t, nb = token(b["cls"])
        if t is None:
            continue
        if n + nb > MAXLEN - 1:
            out.append("|CUT")       # the firmware silently stops here
            break
        out.append(t)
        n += nb
    return "".join(out)


def median(xs):
    xs = sorted(xs)
    if not xs:
        return 0
    return xs[len(xs) // 2]


def rule_size(boxes):
    """(a) A glyph is a glyph.  Drop what is grossly bigger than the plate's own."""
    g = [b for b in boxes if b["cls"] > 0]
    if len(g) < 4:
        return g
    h = median([b["y2"] - b["y1"] for b in g])
    w = median([b["x2"] - b["x1"] for b in g])
    out = []
    for b in g:
        bh, bw = b["y2"] - b["y1"], b["x2"] - b["x1"]
        if bh > 1.6 * h or bh < 0.6 * h:
            continue
        if bw > 1.6 * w or bw < 0.5 * w:
            continue
        out.append(b)
    return out


def rule_row_size(boxes):
    """(a) + (b): and it stands on the same line as the others."""
    g = rule_size(boxes)
    if len(g) < 4:
        return g
    h = median([b["y2"] - b["y1"] for b in g])
    yc = median([(b["y1"] + b["y2"]) // 2 for b in g])
    return [b for b in g if abs((b["y1"] + b["y2"]) // 2 - yc) < 0.5 * h]


def rule_row_size_dup(boxes, frac=0.5):
    """(a) + (b) + (c): and no two of them on the same glyph."""
    keep = []
    for b in rule_row_size(boxes):        # already in score-descending order
        dup = False
        for k in keep:
            ox = min(b["x2"], k["x2"]) - max(b["x1"], k["x1"])
            oy = min(b["y2"], k["y2"]) - max(b["y1"], k["y1"])
            if oy <= 0:
                continue
            if ox > frac * min(b["x2"] - b["x1"], k["x2"] - k["x1"]):
                dup = True
                break
        if not dup:
            keep.append(b)
    return keep


RULES = [
    ("R0 firmware as-is", lambda bs: [b for b in bs if b["cls"] > 0][:MAXBOX]),
    ("R1 +size", rule_size),
    ("R2 +size+row", rule_row_size),
    ("R3 +size+row+dup.5", lambda bs: rule_row_size_dup(bs, 0.5)),
    ("R4 +size+row+dup.3", lambda bs: rule_row_size_dup(bs, 0.3)),
]


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else \
        os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     "..", "..", "log", "log.txt")
    frames = load_frames(path)
    print("%d frames from %s\n" % (len(frames), os.path.normpath(path)))

    for name, fn in RULES:
        print("=== %s" % name)
        hits = {}
        for i, fr in enumerate(frames):
            s = render(fn(fr["boxes"]))
            hits[s] = hits.get(s, 0) + 1
            print("  f%-2d  %-14s   (firmware said: %s)"
                  % (i + 1, s, fr["plate"]))
        top = sorted(hits.items(), key=lambda kv: -kv[1])
        print("  -> %d distinct, modal %r x%d\n" % (len(hits), top[0][0], top[0][1]))

    # how many glyph-class boxes survive each stage, frame by frame
    print("=== survivors per stage (count of glyph-class boxes)")
    for i, fr in enumerate(frames):
        print("  f%-2d  raw %2d | size %2d | row %2d | dup.5 %2d | dup.3 %2d"
              % (i + 1, len([b for b in fr["boxes"] if b["cls"] > 0]),
                 len(rule_size(fr["boxes"])),
                 len(rule_row_size(fr["boxes"])),
                 len(rule_row_size_dup(fr["boxes"], 0.5)),
                 len(rule_row_size_dup(fr["boxes"], 0.3))))


if __name__ == "__main__":
    main()
