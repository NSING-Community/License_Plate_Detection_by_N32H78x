"""板子读到的 ② 有唯一一处 32 字节不对 (0x1504D120..0x1504D13F 装的是文件 +16 处的内容,
其余 431,328 字节逐字节等于文件)。 那个地址是绝对 32 字节对齐的 —— 一条 D-Cache 行 ——
但 main.c 里 SCB_EnableDCache() 开机先整片 invalidate, 陈旧行活不过复位, 所以不是缓存。

那就只剩两种可能, 而磁盘上就有分水岭: 被下载的那个镜像 (Printf.axf / Printf.bin)。

  镜像里那 32 字节 **也是错的** -> 是链接/fromelf/hex 生成这一环, 不是烧录
  镜像里那 32 字节 **是对的**   -> 镜像没问题, 是烧录 (擦除/编程) 没写进去

脚本在镜像里找 tflite 的魔数 "TFL3", 定位到 ② 那份拷贝, 然后和 .tflite 逐字节比。

Run:  python tflite/_dbg_image.py
"""
import glob
import os
import struct

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TFL = os.path.join(ROOT, "tflite", "onnx2tf_int8_crop", "crop-int8.tflite")
MAGIC = bytes.fromhex("1c000000") + b"TFL3"          # root uoffset 28 + "TFL3"
BOARD_LINE = bytes.fromhex(                        # board's bytes at 0x1504D120
    "1c000000a0000000f80000001cea0300"
    "2cea0300c89306000300000001000000")
# BASE 是 aligned(16) 那一版的地址。 2026-10-16 之后 g_model_crop 改成
# aligned(32), ER_MODEL_RO 基址抬到 32 字节边界 (预计 0x1504D120) —— 换镜像后
# 这两个常量要按新 Printf.map 里的 g_model_crop 地址改, 它们只描述那一次观测。
BASE = 0x1504D110
LINE_ADDR = 0x1504D120

raw = open(TFL, "rb").read()

# 板子镜像的基址 (LR_IROM1 的起始). XIP flash 从 0x15000000 起。
FLASH_BASE = 0x15000000
LINE_FILEOFF = LINE_ADDR - BASE                  # 窗口在 .tflite 里的相对偏移 = 0x10

cands = []
for pat in ("*.bin", "*.hex", "*.axf", "*.elf", "*.map"):
    cands += glob.glob(os.path.join(ROOT, "mcu_code", "**", pat), recursive=True)

print("--- image/build artefacts found on disk ---")
for p in sorted(cands):
    st = os.stat(p)
    print("  %10d  %s  %s" % (st.st_size,
                              __import__("time").strftime("%Y-%m-%d %H:%M", __import__("time").localtime(st.st_mtime)),
                              os.path.relpath(p, ROOT)))

bins = [p for p in cands if p.lower().endswith((".bin", ".elf", ".axf"))]
print("\n--- looking for the ② model inside each ---")
for p in sorted(bins):
    data = open(p, "rb").read()
    print("\n%s  (%d bytes)" % (os.path.relpath(p, ROOT), len(data)))
    off = 0
    n = 0
    while True:
        i = data.find(MAGIC, off)
        if i < 0:
            break
        n += 1
        same = data[i:i + len(raw)] == raw
        print("  TFL3 @ file offset 0x%X -> flash 0x%08X   %s"
              % (i, FLASH_BASE + i, "IDENTICAL to .tflite" if same else "differs from .tflite"))
        if not same:
            k = 0
            while k < len(raw) and data[i + k] == raw[k]:
                k += 1
            print("    first difference at rel 0x%X" % k)
            print("    image : %s" % data[i + LINE_FILEOFF:i + LINE_FILEOFF + 32].hex(" "))
            print("    file  : %s" % raw[LINE_FILEOFF:LINE_FILEOFF + 32].hex(" "))
            print("    board : %s" % BOARD_LINE.hex(" "))
            print("    image[0x10:0x30] == file[0x20:0x40] ? %s"
                  % (data[i + 0x10:i + 0x30] == raw[0x20:0x40]))
            # 差异是不是只有 0x10..0x2F 这一处?
            diffs = [x for x in range(min(len(raw), len(data) - i))
                     if data[i + x] != raw[x]]
            if diffs:
                runs = []
                s = pp = diffs[0]
                for x in diffs[1:]:
                    if x != pp + 1:
                        runs.append((s, pp))
                        s = x
                    pp = x
                runs.append((s, pp))
                print("    %d differing bytes in %d run(s): %s"
                      % (len(diffs), len(runs),
                         ", ".join("0x%X..0x%X" % r for r in runs[:8])))
        off = i + 4
    if n == 0:
        print("  no TFL3 magic found")

print("""
--- 判读 ---
  image 那 32 字节**也是错的**  -> 链接/fromelf/hex 这一环的问题 (镜像本身就是错的),
                                 那么 `Printf.bin` 和 `Printf.axf` 里应该一对一致
  image 那 32 字节**是对的**    -> 镜像没问题, 是烧录那一环: 擦除/编程没写进去
                                  -> 全片擦除后重新下载, 再看 0x1504D120
""")
