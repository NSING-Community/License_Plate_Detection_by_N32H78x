"""Decode the root table's vtable in both int8 models, and give the CRC32s the board
must reproduce.

The board's fault is now pinned to one instruction (fromelf on Printf.axf):

    ca88  LDR   r0,[r4,#0]     ; r4 = Model*, r0 = the table's soffset-to-vtable
    ca8a  SUBS  r1,r4,r0       ; r1 = vtable
    ca8c  LDRH  r0,[r1,#0]     ; vtable size
    ca94  LDRH  r1,[r1,#8]     ; r1 = VT_SUBGRAPHS field offset      <<< reads 0xEA2C
    ca9e  LDR   r2,[r4,r1]     ; r2 = *(u32*)(model + fieldoff)      = R2 = 0xD606F9F4
    caa0  ADD   r1,r1,r4       ; r1 = model + fieldoff               = R1 = 0x1505BB58
    caa4  LDR   r1,[r1,r2]     ; flatbuffer follow -> BFAR 0xEB0CB54C  <<< FAULT
    caa8  LSLS  r1,r1,#3       ; * sizeof(SubgraphAllocations)=8
    caac  BLX   r3             ; AllocatePersistentBuffer(bytes, align=4)

i.e. micro_allocator.cpp:651 `sizeof(SubgraphAllocations) * model->subgraphs()->size()`.

From the frame, fieldoff = R1 - Model* = 0x1505BB58 - (0x1504D110+28) = 0xEA2C = 59948.
The root table's own size field says 32, so no legal field offset is anywhere near that:
the halfword read from vtable+8 is not what the file holds.  Print what the file holds,
and the CRCs the board must match to prove the flash reads faithfully.

Run:  python tflite/_dbg_head.py
"""
import os
import struct
import zlib

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FILES = [
    ("(1) plate ", os.path.join(ROOT, "tflite", "yolo-plate-int8.tflite")),
    ("(2) crop  ", os.path.join(ROOT, "tflite", "onnx2tf_int8_crop", "crop-int8.tflite")),
]
FIELDS = ["version", "operator_codes", "subgraphs", "description", "buffers",
          "metadata_buffer", "metadata", "signature_defs"]

for tag, path in FILES:
    raw = open(path, "rb").read()
    print("\n=== %s %d bytes" % (tag, len(raw)))
    print("  first 64 bytes: %s" % raw[:64].hex(" "))

    root = int.from_bytes(raw[:4], "little")
    print("  root uoffset at blob 0      = %d  -> Model* = blob + %d" % (root, root))

    soff = int.from_bytes(raw[root:root + 4], "little", signed=True)
    vt = root - soff
    print("  soffset at Model+0          = %d  -> vtable at blob %d" % (soff, vt))

    vtsize, tabsize = struct.unpack_from("<HH", raw, vt)
    print("  vtable size %d, table size %d" % (vtsize, tabsize))
    for i, name in enumerate(FIELDS):
        pos = vt + 4 + 2 * i
        off = struct.unpack_from("<H", raw, pos)[0] if pos + 2 <= vt + vtsize else -1
        print("      vtable+%-2d  %-16s table offset %s" % (4 + 2 * i, name, off))

    # What the *board* read at vtable+8, for comparison.
    board = 0xEA2C
    print("  board read vtable+8 as %d (0x%X) - %s"
          % (board, board, "MATCHES THE FILE" if board == struct.unpack_from("<H", raw, vt + 8)[0]
             else "*** DIFFERS FROM THE FILE ***"))

    whole = zlib.crc32(raw) & 0xFFFFFFFF
    chunks = [zlib.crc32(raw[i:i + 32768]) & 0xFFFFFFFF for i in range(0, len(raw), 32768)]
    print("  crc32(whole) = %08X   (%d chunks of 32768)" % (whole, len(chunks)))
    print("  crc32 per 32KB chunk:")
    for i, c in enumerate(chunks):
        print("     [%2d] blob %7d..%7d  %08X" % (i, i * 32768, min(len(raw), (i + 1) * 32768) - 1, c))
