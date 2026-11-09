"""Why does the board hard-fault in TFLM's StartModelAllocation on ②'s AllocateTensors()?

Two questions, both answerable offline:
  1. Is inc/model_data_crop.h byte-faithful to tflite/onnx2tf_int8_crop/crop-int8.tflite?
     (The map says g_model_crop is 431360 bytes and the blob's first 8 bytes are a valid
      flatbuffer root + "TFL3", so a size error is ruled out - a *content* error is not.)
  2. What ops does the model actually contain?  yolo_cascade_init() registers exactly the
     12 that ① has; if ②' s opcode list differs, the parse is reading op data that is not
     there, which is the shape of a wild-offset read.

Run:  python tflite/_dbg_crop_model.py
"""
import hashlib
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TFL = os.path.join(ROOT, "tflite", "onnx2tf_int8_crop", "crop-int8.tflite")
HDR = os.path.join(ROOT, "mcu_code", "Printf2", "inc", "model_data_crop.h")


def md5(b):
    return hashlib.md5(b).hexdigest().upper()


raw = open(TFL, "rb").read()
print("tflite : %d bytes  MD5 %s" % (len(raw), md5(raw)))

txt = open(HDR, encoding="utf-8", errors="replace").read()
body = txt[txt.index("{") + 1: txt.rindex("}")]
arr = bytes(int(m, 16) for m in re.findall(r"0x([0-9a-fA-F]{2})", body))
print("array  : %d bytes  MD5 %s" % (len(arr), md5(arr)))
print("identical: %s" % (arr == raw,))
if arr != raw:
    n = min(len(arr), len(raw))
    bad = [i for i in range(n) if arr[i] != raw[i]]
    if bad:
        print("  %d differing bytes, first at 0x%06x (array %02x, file %02x)"
              % (len(bad), bad[0], arr[bad[0]], raw[bad[0]]))
        for i in bad[:10]:
            print("    0x%06x  array %02x  file %02x" % (i, arr[i], raw[i]))

# The faulting access was [R1 + R2] with R1 = g_model_crop + 0xEA48 (0x1505BB58 - 0x1504D110)
# and R2 = 0xD606F9F4 (read as a signed soffset: -703922188).  Show what is actually there.
OFF = 0x1505BB58 - 0x1504D110
print("\noffset 0x%06x in the blob: %s" % (OFF, raw[OFF:OFF + 8].hex(" ")))
print("  as int32: %d (0x%08x)" % (int.from_bytes(raw[OFF:OFF + 4], "little", signed=True),
                                    int.from_bytes(raw[OFF:OFF + 4], "little")))
print("  base+off = 0x%08x   BFAR was 0xEB0CB54C" % ((0x1504D110 + OFF + 0xD606F9F4) & 0xFFFFFFFF))

print("\n--- structure ---")
print("first 8 bytes: %s" % raw[:8].hex(" "))
print("  root offset %d, magic %r" % (int.from_bytes(raw[:4], "little"), raw[4:8]))
print("  last 8 bytes : %s" % raw[-8:].hex(" "))

try:
    import tensorflow as tf
except Exception as e:  # noqa: BLE001
    print("tensorflow not importable: %s" % e)
    sys.exit(0)

try:
    it = tf.lite.Interpreter(model_path=TFL)
    it.allocate_tensors()
except Exception as e:  # noqa: BLE001
    print("tf.lite FAILED to load/allocate: %s" % e)
    sys.exit(0)

print("tf.lite loaded and allocated OK")
ins = it.get_input_details()
outs = it.get_output_details()
for d in ins:
    print("  in  %-12s %s %s" % (d["name"], d["shape"], d["dtype"].__name__))
for d in outs:
    print("  out %-12s %s %s" % (d["name"], d["shape"], d["dtype"].__name__))

try:
    ops = it._get_ops_details()
except Exception as e:  # noqa: BLE001
    print("_get_ops_details unavailable: %s" % e)
    ops = []

names = [o["op_name"] for o in ops]
seen = []
for n in names:
    if n not in seen:
        seen.append(n)
print("\n%d ops, %d distinct: %s" % (len(names), len(seen), ", ".join(seen)))

REGISTERED = ["CONV_2D", "DEPTHWISE_CONV_2D", "MAX_POOL_2D", "PAD", "CONCATENATION",
              "TRANSPOSE", "RESHAPE", "GATHER", "LOGISTIC", "SOFTMAX", "QUANTIZE",
              "RESIZE_NEAREST_NEIGHBOR"]
missing = [n for n in seen if n not in REGISTERED]
print("in model but NOT registered by yolo_cascade_init(): %s" % (missing or "none"))
