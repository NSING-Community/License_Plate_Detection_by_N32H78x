"""板子上那份 ② 和文件差在哪 —— 用 CRC32 反解"哪个窗口不对", 再回头查 C 数组。

上板 log (mcu_code/log/log.txt, 2026-10-16):

    crop flash 431360 B at 1504D110 -> ram C0920000
    crop headF 0000001C 334C4654 00200014 0018001C 0000001C 000000A0 000000F8 0003EA1C
    crop headF2 0003EA2C 000693C8 00000003 00000001 0003EA2C 000693C8 00000003 00000001
    crop halfF 00000014 00000020 0000001C 00000018 0000001C 00000000 000000A0 00000000
    crop crc fW FDD8557B fH FDD8557B
    crop crc fB FDD8557B rB FDD8557B want D7DC5BA5
    crop parse: SDRAM copy
    crop init FAIL: schema 199989017, pack wants 3

(注: 下面的 0x1504D110 是 `aligned(16)` 那一版的基址。 2026-10-16 之后
 g_model_crop 改成 aligned(32), ER_MODEL_RO 的基址会抬到 32 字节边界 (预计
 0x1504D120) —— 拿新日志来跑这个脚本时, BASE 要按新日志 "crop flash ... at
 XXXXXXXX" 那一行改, 否则算出来的是另一块地址的形状。)

先说三件板上已经量死的事:

1. **不是"读的宽度"的问题。** 32/16/8 位三种宽度扫同一块 flash, 加上按字节拷进
   SDRAM 再读回来, 四个 CRC 全一样 = FDD8557B。 上一轮那个"LDRH 读回 +32 处的
   半字"不是 16 位读路径特有的毛病 —— 所有宽度看到的是同一份内容。 原假设(读
   路径)被自己的对照否掉了, 这正是当初设计四个 CRC 的目的。

2. **板子眼里那份内容不等于文件。** 文件 crc32 = D7DC5BA5。 SDRAM 副本 rB 也是
   FDD8557B, 说明拷贝忠实地搬了错的字节。

3. **崩的地方对得上。** 从 SDRAM 副本解析读到的 schema version = 199989017 =
   0x0BEB9719 -> schema 检查失败 -> 干净地 return false -> "cascade init
   ERROR - single-model path"。 上一轮那个野地址 HardFault 是同一件事的另一种
   表现 (字段偏移被读成 0xEA2C)。

观察到的形状 (rel = 相对 g_model_crop), 逐字对下来:

    rel 0x00..0x0F   = 文件 rel 0x00..0x0F
    rel 0x10..0x2F   = 文件 rel 0x20..0x3F      <-- 只有这 32 字节整体 +16
    rel 0x30..0x3F   = 文件 rel 0x30..0x3F

即"文件 rel 0x10..0x1F 那 16 字节不见了, rel 0x20..0x2F 在后面又出现一次"。
所以这一版脚本做两件事:
  (1) 枚举"某个 16 字节对齐的窗口被文件后面 16 字节的内容替换"的所有形状,
      撞 FDD8557B, 确认全图是不是**只有这一个窗口**不对;
  (2) 把 inc/model_data_crop.h 里的数组按字节抽出来和 .tflite 逐字节比 ——
      如果数组本身就是这个样子, 那问题在导出, 不在烧录。

Run:  python tflite/_dbg_shift.py
"""
import os
import re
import struct
import zlib

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TFL = os.path.join(ROOT, "tflite", "onnx2tf_int8_crop", "crop-int8.tflite")
HDR = os.path.join(ROOT, "mcu_code", "Printf2", "inc", "model_data_crop.h")

BOARD_CRC = 0xFDD8557B
BOARD_HEAD = [
    0x0000001C, 0x334C4654, 0x00200014, 0x0018001C,
    0x0000001C, 0x000000A0, 0x000000F8, 0x0003EA1C,
    0x0003EA2C, 0x000693C8, 0x00000003, 0x00000001,
    0x0003EA2C, 0x000693C8, 0x00000003, 0x00000001,
]

raw = open(TFL, "rb").read()
N = len(raw)
BASE = 0x1504D110


def crc(b):
    return zlib.crc32(b) & 0xFFFFFFFF


print("file: %d bytes, crc32 %08X   board: %08X at 0x%08X\n"
      % (N, crc(raw), BOARD_CRC, BASE))

print("--- board's head: where each 4-byte value sits in the file ---")
for i in range(16):
    f = struct.unpack_from("<I", raw, 4 * i)[0]
    b = BOARD_HEAD[i]
    if f == b:
        tag = "same as file"
    else:
        p = raw.find(struct.pack("<I", b), 4 * i)
        tag = ("file rel 0x%X (delta %+d)" % (p, p - 4 * i)) if p >= 0 else "NOWHERE in file"
    print("  rel 0x%02X  file %08X  board %08X  %s" % (4 * i, f, b, tag))

finds = []

print("\n--- windows replaced by the file's own bytes from +16 (16B-aligned, len 16..128) ---")
tried = 0
for W in range(0, N - 128, 16):
    for L in (16, 32, 48, 64, 128):
        tried += 1
        if crc(raw[:W] + raw[W + 16:W + 16 + L] + raw[W + L:]) == BOARD_CRC:
            finds.append(("+16", W, L))
print("  %d (window, length) pairs tried" % tried)

print("\n--- same, with the file's bytes from -16 ---")
tried = 0
for W in range(16, N - 128, 16):
    for L in (16, 32, 48, 64, 128):
        tried += 1
        if crc(raw[:W] + raw[W - 16:W - 16 + L] + raw[W + L:]) == BOARD_CRC:
            finds.append(("-16", W, L))
print("  %d pairs tried" % tried)

print("\n=== matches: %s ===" % (finds or "NONE"))
for s, W, L in finds:
    print("  rel 0x%04X..0x%04X (0x%08X..0x%08X) holds the file's bytes from %s"
          % (W, W + L - 1, BASE + W, BASE + W + L - 1, s))

if finds:
    for s, W, L in finds:
        img = raw[:W] + raw[W + 16:W + 16 + L] + raw[W + L:]
        assert crc(img) == BOARD_CRC
        print("\n也就是说: 除了这一处, 其余 %d 字节逐字节等于文件。" % (N - L))

# ---------------------------------------------------------------- the C array
print("\n" + "=" * 72)
print("--- inc/model_data_crop.h: the array itself, byte for byte ---")
txt = open(HDR, encoding="utf-8", errors="replace").read()
body = txt[txt.index("{") + 1: txt.rindex("}")]
arr = bytes(int(m, 16) for m in re.findall(r"0x([0-9a-fA-F]{2})", body))
print("  array in header : %d bytes" % len(arr))
print("  .tflite         : %d bytes" % N)
print("  identical       : %s" % (arr == raw))
if arr != raw:
    n = min(len(arr), N)
    bad = [i for i in range(n) if arr[i] != raw[i]]
    print("  differing bytes : %d, first at rel 0x%X" % (len(bad), bad[0] if bad else -1))
    if bad:
        # group into runs so a 16-byte hole reads as one line, not sixteen
        runs = []
        s = bad[0]
        p = bad[0]
        for i in bad[1:]:
            if i != p + 1:
                runs.append((s, p))
                s = i
            p = i
        runs.append((s, p))
        print("  runs of differences (rel):")
        for a, b in runs[:12]:
            print("    0x%06X..0x%06X  (%d bytes)  header %s | file %s"
                  % (a, b, b - a + 1, arr[a:a + 8].hex(" "), raw[a:a + 8].hex(" ")))
        print("  total runs: %d" % len(runs))
print("\n  header rel 0x00..0x5F: %s" % arr[:0x60].hex(" "))

print("\n--- is the header's rel 0x10..0x2F the file's rel 0x20..0x3F? ---")
print("  header[0x10:0x30] == file[0x20:0x40] : %s" % (arr[0x10:0x30] == raw[0x20:0x40]))
print("  header[0x00:0x10] == file[0x00:0x10] : %s" % (arr[0x00:0x10] == raw[0x00:0x10]))
print("  header[0x30:0x60] == file[0x30:0x60] : %s" % (arr[0x30:0x60] == raw[0x30:0x60]))
