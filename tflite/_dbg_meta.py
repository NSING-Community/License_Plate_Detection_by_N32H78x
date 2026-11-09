"""Which metadata entries does each model carry?

Why this matters: TFLM 1.26.2's AllocationInfoBuilder::GetOfflinePlannedOffsets()
(micro_allocation_info.cpp:338) scans model->metadata() for an entry *named exactly*
"OfflineMemoryAllocation", then reads that entry's buffer as

    metadata_buffer[0..2]  = header, metadata_buffer[2] == nbr_tensors
    &metadata_buffer[3]    = int32[] offline-planned byte offsets per tensor

and the memory planner then computes arena addresses from those offsets.  A metadata
buffer written by a *different* converter vintage - or one whose tensor count does not
match what this pack computes - turns those offsets into wild addresses, which is the
shape of the board's precise data bus error at [R1 + R2] with R2 read out of the model.

(1) works on the board, (2) hard-faults.  Both report metadata: 2.  If the *names* (or
the buffer layout) differ, that is the whole difference.

Run:  python tflite/_dbg_meta.py
"""
import os
import struct
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FILES = [
    ("(1) plate  yolo-plate-int8.tflite", os.path.join(ROOT, "tflite", "yolo-plate-int8.tflite")),
    ("(2) crop   crop-int8.tflite", os.path.join(ROOT, "tflite", "onnx2tf_int8_crop", "crop-int8.tflite")),
]

from tensorflow.lite.python import schema_py_generated as schema_fb  # noqa: E402

OFFLINE = "OfflineMemoryAllocation"


def as_text(x):
    if isinstance(x, bytes):
        return x.decode("utf-8", "replace")
    return x


for tag, path in FILES:
    if not os.path.exists(path):
        print("missing: %s" % path)
        continue
    buf = open(path, "rb").read()
    m = schema_fb.Model.GetRootAsModel(buf, 0)
    print("\n=== %s  (%d bytes)" % (tag, len(buf)))

    n = m.MetadataLength()
    print("  metadata entries: %d" % n)
    for i in range(n):
        md = m.Metadata(i)
        name = as_text(md.Name())
        bidx = md.Buffer()
        blen = -1
        head = b""
        if 0 <= bidx < m.BuffersLength():
            b = m.Buffers(bidx)
            if b.DataLength():
                blen = b.DataLength()
                head = bytes(b.Data(j) for j in range(min(64, blen)))
        flag = "  <== OFFLINE PLAN" if name == OFFLINE else ""
        print("    [%d] name=%r buffer=%d len=%d%s" % (i, name, bidx, blen, flag))
        if blen > 0:
            u = struct.unpack("<%dI" % (len(head) // 4), head[: len(head) // 4 * 4])
            print("        head as u32: %s" % (list(u[:8]),))
            if name == OFFLINE:
                print("        parsed by the pack as: magic=%d version=%d nbr_tensors=%d"
                      % (u[0], u[1], u[2]))
                print("        first 6 planned offsets: %s" % (list(u[3:9]),))

    # What the pack will compute as info_.tensor_count: tensors + scratch buffers.
    for gi in range(m.SubgraphsLength()):
        sg = m.Subgraphs(gi)
        print("  subgraph %d: tensors=%d operators=%d" % (gi, sg.TensorsLength(),
                                                          sg.OperatorsLength()))

    # Anything else referencing metadata?
    print("  signature_defs: %d, buffers: %d"
          % (m.SignatureDefsLength() if hasattr(m, "SignatureDefsLength") else -1,
             m.BuffersLength()))
