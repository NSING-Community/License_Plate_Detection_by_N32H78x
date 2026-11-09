"""Compare the flatbuffer structure of the two int8 plate/crop models.

Why: the board parses ①'s model (tflite/yolo-plate-int8.tflite, embedded as
inc/model_data.h) with no trouble, and hard-faults inside TFLM's
MicroAllocator::StartModelAllocation on ②'s (tflite/onnx2tf_int8_crop/crop-int8.tflite,
embedded as inc/model_data_crop.h).  Both blobs are byte-faithful to their files and
both load in tf.lite, so the difference has to be in what the pack's parser reads.

The MDK pack is tensorflow-lite-micro 1.26.2, i.e. a 2022-era TFLM with its own
schema_generated.h.  ② was converted by the Phase-0 scripts under the *current*
TensorFlow.  If ② carries op versions / builtin_options types that the pack's
generated readers do not lay out the same way, ParseOpData() misreads a table and
follows a field that is not a pointer - which is what [R1+R2] with R2 read out of
the weights looks like.

Run:  python tflite/_dbg_fb_layout.py
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FILES = [
    ("(1) plate  yolo-plate-int8.tflite", os.path.join(ROOT, "tflite", "yolo-plate-int8.tflite")),
    ("(2) crop   crop-int8.tflite", os.path.join(ROOT, "tflite", "onnx2tf_int8_crop", "crop-int8.tflite")),
]

from tensorflow.lite.python import schema_py_generated as schema_fb  # noqa: E402


def enum_name(cls, code):
    for k in dir(cls):
        if not k.startswith("_") and getattr(cls, k) == code:
            return k
    return "code=%d" % code


def dump(tag, path):
    buf = open(path, "rb").read()
    m = schema_fb.Model.GetRootAsModel(buf, 0)
    print("\n=== %s" % tag)
    print("  file %d bytes, model schema version %d" % (len(buf), m.Version()))

    print("  operator_codes (%d):" % m.OperatorCodesLength())
    for i in range(m.OperatorCodesLength()):
        oc = m.OperatorCodes(i)
        bc = oc.BuiltinCode()
        cc = oc.CustomCode()
        print("    [%2d] %-4d %-28s version %d%s"
              % (i, bc, enum_name(schema_fb.BuiltinOperator, bc), oc.Version(),
                 ("  custom=%r" % cc) if cc else ""))

    print("  subgraphs: %d" % m.SubgraphsLength())
    for gi in range(m.SubgraphsLength()):
        sg = m.Subgraphs(gi)
        print("    [%d] inputs %d outputs %d tensors %d operators %d"
              % (gi, sg.InputsLength(), sg.OutputsLength(), sg.TensorsLength(),
                 sg.OperatorsLength()))
        hist = {}
        opts = {}
        for i in range(sg.OperatorsLength()):
            op = sg.Operators(i)
            oc = m.OperatorCodes(op.OpcodeIndex())
            bc = oc.BuiltinCode()
            name = enum_name(schema_fb.BuiltinOperator, bc)
            hist[name] = hist.get(name, 0) + 1
            ot = op.BuiltinOptionsType()
            opts[enum_name(schema_fb.BuiltinOptions, ot)] = \
                opts.get(enum_name(schema_fb.BuiltinOptions, ot), 0) + 1
        print("      ops by type: %s" % ", ".join("%s x%d" % kv for kv in sorted(hist.items())))
        print("      builtin_options union tags: %s"
              % ", ".join("%s x%d" % kv for kv in sorted(opts.items())))
        # versions actually used by nodes
        vers = {}
        for i in range(sg.OperatorsLength()):
            op = sg.Operators(i)
            oc = m.OperatorCodes(op.OpcodeIndex())
            key = "%s v%d" % (enum_name(schema_fb.BuiltinOperator, oc.BuiltinCode()), oc.Version())
            vers[key] = vers.get(key, 0) + 1
        print("      node op versions: %s" % ", ".join("%s x%d" % kv for kv in sorted(vers.items())))

    if m.SignatureDefsLength() if hasattr(m, "SignatureDefsLength") else 0:
        print("  signature_defs: %d" % m.SignatureDefsLength())
    print("  buffers: %d, metadata: %d"
          % (m.BuffersLength(),
             m.MetadataLength() if hasattr(m, "MetadataLength") else 0))


for tag, path in FILES:
    if not os.path.exists(path):
        print("missing: %s" % path)
        continue
    try:
        dump(tag, path)
    except Exception as e:  # noqa: BLE001
        print("FAILED on %s: %s" % (tag, e))
        sys.exit(1)
