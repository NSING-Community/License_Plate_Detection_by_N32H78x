# -*- coding: utf-8 -*-
"""看一眼 ONNX / tflite 的输入输出名与量化参数。

用法：
  python tflite/_inspect_model.py <模型路径> [更多路径...]
"""

import os
import sys


def show_onnx(p):
    import onnx
    m = onnx.load(p)
    print("ONNX  %s" % p)
    for i in m.graph.input:
        print("   in : %-24s %s" % (i.name, [d.dim_value or d.dim_param
                                             for d in i.type.tensor_type.shape.dim]))
    for o in m.graph.output:
        print("   out: %-24s %s" % (o.name, [d.dim_value or d.dim_param
                                             for d in o.type.tensor_type.shape.dim]))


def show_tflite(p):
    import numpy as np
    import tensorflow as tf
    it = tf.lite.Interpreter(model_path = p)
    it.allocate_tensors()
    print("TFLITE %s   %d bytes" % (p, os.path.getsize(p)))

    def q(t):
        sc, zp = t["quantization"]
        sc = np.atleast_1d(np.asarray(sc, np.float64))
        zp = np.atleast_1d(np.asarray(zp, np.int64))
        return "scale %s zp %s" % (
            np.round(sc, 8).tolist() if sc.size else "[]", zp.tolist())

    for t in it.get_input_details():
        print("   in : %-11s %-18s %-8s %s" % (t["name"], t["shape"].tolist(),
                                               t["dtype"].__name__, q(t)))
    for t in it.get_output_details():
        print("   out: %-11s %-18s %-8s %s" % (t["name"], t["shape"].tolist(),
                                               t["dtype"].__name__, q(t)))


if __name__ == "__main__":
    for p in sys.argv[1:]:
        if not os.path.exists(p):
            print("!! 不存在 %s" % p)
            continue
        (show_onnx if p.lower().endswith(".onnx") else show_tflite)(p)
        print()
