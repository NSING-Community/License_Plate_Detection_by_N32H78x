"""Convert an image into Printf2/src/yolo_detect_image.inc - the frame the board
runs detection on when there is no camera wired up yet.

    python make_detect_image.py <image> [out.inc]

Default output is ../src/yolo_detect_image.inc relative to this script, so it can
be run from anywhere.  Format is fixed: 320x320 RGB888, PIL's default (bicubic)
resize, 16 bytes per line, pinned to the detect_image_ro section so the linker
puts it in XIP flash - see the header it writes into the .inc for the details.

Regenerating is self-contained: the frame is read straight out of XIP flash, so
nothing else in the firmware needs to change - just rebuild.  The md5 / first6 /
sum it prints are only an identity check for the frame that is now on the board.
"""
import hashlib
import os
import sys

import numpy as np
from PIL import Image

W = H = 320
HERE = os.path.dirname(os.path.abspath(__file__))

if len(sys.argv) < 2:
    sys.exit(__doc__)
src = sys.argv[1]
dst = sys.argv[2] if len(sys.argv) > 2 else \
    os.path.join(HERE, os.pardir, "src", "yolo_detect_image.inc")

im = Image.open(src)
print("source:", src, im.size, im.mode)
raw = np.array(im.convert("RGB").resize((W, H)), dtype=np.uint8).tobytes()
assert len(raw) == 307200, len(raw)
md5 = hashlib.md5(raw).hexdigest().upper()

with open(dst, "w", encoding="ascii", newline="\n") as f:
    f.write(f"""/* Auto-generated from {src} by tools/make_detect_image.py - do not edit.
 *
 * 320x320 RGB888 frame ({len(raw)} bytes, md5 {md5}).
 * PIL's default bicubic resize, i.e. byte-for-byte the same input the Python
 * reference fed the model, so the board's output can be diffed against it.
 *
 * Why an .inc and not a .c: a source file added to Printf.uvprojx by hand does
 * not survive - uVision rewrites the whole file list from its in-memory copy on
 * save, so the entry disappears and the build fails with
 * "L6218E: Undefined symbol yolo_detect_image (referred from main.o)" even
 * though the file is right there on disk.  Including it needs no project change
 * at all.  Include it from exactly one .c - this is a definition, not a
 * declaration.
 *
 * __attribute__((section)) pins it to its own XIP-flash region
 * (ER_DETECT_IMG in M7_TCM.sct).  It is RO, so nothing copies it at startup and
 * yolo_plate_detect() reads it in place - no RAM is spent on it.
 *
 * To drop the frame again: delete this file, the ER_DETECT_IMG region from
 * M7_TCM.sct, and the #include plus the rgb pointer in main.c.
 */
#include <stdint.h>

#define DETECT_IMG_W  {W}
#define DETECT_IMG_H  {H}

__attribute__((section("detect_image_ro"), used))
const uint8_t yolo_detect_image[DETECT_IMG_W * DETECT_IMG_H * 3] = {{
""")
    for i in range(0, len(raw), 16):
        f.write("    " + ", ".join(f"0x{b:02X}" for b in raw[i:i + 16]) + ",\n")
    f.write("};\n")

print(f"wrote {dst}")
print(f"  md5    {md5}")
print(f"  first6 {', '.join(str(b) for b in raw[:6])}"
      f"  ({' '.join('%02X' % b for b in raw[:6])})")
print(f"  sum    {int(np.frombuffer(raw, dtype=np.uint8).astype(np.uint64).sum())}")
print("\nNow update the 'want <sum>' value and its comment in "
      "MDK-ARM/yolo_inference.cpp, then rebuild.")
