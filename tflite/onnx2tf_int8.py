# -*- coding: utf-8 -*-
"""ONNX -> 全整型 int8 TFLite（板子上跑的那份）。参数化版，一份脚本导三个模型。

老版本把 `pytorch2onnx/yolo-fastestv2-opt.onnx`、输出目录、校准集全部写死，
只能导 ①。v1.0 要导三个，而且**三个模型的校准分布完全不同**：

  ① 整图找车牌 → 校准用 `training/data/val/*.jpg` 缩到 320×320（整图）
  ② 裁剪找字   → 校准用 `training/data_crop/val/*.jpg` 缩到 320×320（② 看到的就是裁剪图）
                  用整图去校准 ② 是**分布的错配**，量化 scale 会偏
  ③ 逐字分类   → 校准用 32×64 字符小块（`training/cache/charpatch_val_32x64_pad25.npz`，
                  就是 train_char_head 训练时用的那份，分布完全对得上）

用法：
  python tflite/onnx2tf_int8.py --onnx tflite/onnx/plate.onnx \\
      --calib training/data/val --size 320x320 --out-dir tflite/onnx2tf_int8_plate

  python tflite/onnx2tf_int8.py --onnx tflite/onnx/crop.onnx \\
      --calib training/data_crop/val --size 320x320 --out-dir tflite/onnx2tf_int8_crop

  python tflite/onnx2tf_int8.py --onnx tflite/onnx/charhead.onnx \\
      --calib training/cache/charpatch_val_32x64_pad25.npz \\
      --out-dir tflite/onnx2tf_int8_charhead
"""

import os
import sys
import glob
import argparse
import random
import traceback

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))       # tflite/
ROOT = os.path.dirname(HERE)                            # 工程根目录


def calib_from_dir(d, size, n, seed):
    """一个目录的 jpg -> (n, H, W, 3) float32，值域 [0,1]，NHWC。"""
    from PIL import Image
    W, H = size
    jpgs = sorted(glob.glob(os.path.join(d, "*.jpg")) +
                  glob.glob(os.path.join(d, "*.png")))
    random.Random(seed).shuffle(jpgs)
    out = []
    for p in jpgs:
        if len(out) >= n:
            break
        try:
            im = Image.open(p).convert("RGB").resize((W, H), Image.BILINEAR)
        except Exception:
            continue
        out.append(np.asarray(im, np.float32) / 255.0)
    return np.stack(out)


def calib_from_npz(p, n, seed):
    """字符小块缓存：train_char_head.cache_patches 存的 X 是 (N,H,W,3) uint8。"""
    z = np.load(p)
    X = z["X"] if "X" in z.files else z[z.files[0]]
    idx = list(range(len(X)))
    random.Random(seed).shuffle(idx)
    X = X[idx[:n]]
    return np.asarray(X, np.float32) / 255.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--onnx", required = True)
    ap.add_argument("--calib", required = True,
                    help = "图片目录，或者 .npz/.npy（字符小块缓存）")
    ap.add_argument("--size", default = "320x320",
                    help = "只有 --calib 是目录时用得到，WxH")
    ap.add_argument("--n", type = int, default = 300, help = "校准图数量")
    ap.add_argument("--seed", type = int, default = 7)
    ap.add_argument("--out-dir", required = True)
    ap.add_argument("--name", default = "", help = "tflite 输出名（默认取 onnx 的 stem）")
    ap.add_argument("--keep-npy", action = "store_true",
                    help = "把校准数组留在 out-dir 里，便于复现")
    args = ap.parse_args()

    args.onnx = os.path.abspath(args.onnx)
    args.out_dir = os.path.abspath(args.out_dir)
    args.calib = os.path.abspath(args.calib)
    stem = args.name or os.path.splitext(os.path.basename(args.onnx))[0]
    os.makedirs(args.out_dir, exist_ok = True)

    print("== int8 量化  %s -> %s" % (os.path.relpath(args.onnx, ROOT),
                                      os.path.relpath(args.out_dir, ROOT)))
    if os.path.isdir(args.calib):
        W, H = (int(v) for v in args.size.lower().split("x"))
        calib = calib_from_dir(args.calib, (W, H), args.n, args.seed)
        src = "%s (%dx%d)" % (os.path.relpath(args.calib, ROOT), W, H)
    else:
        calib = calib_from_npz(args.calib, args.n, args.seed)
        src = os.path.relpath(args.calib, ROOT)
    print("  校准集 %s  %s  min %.3f max %.3f mean %.3f"
          % (calib.shape, src, calib.min(), calib.max(), calib.mean()))
    if calib.shape[0] < args.n:
        print("  !! 校准图只有 %d 张（想要 %d）" % (calib.shape[0], args.n))

    npy = os.path.join(args.out_dir, "_calib.npy")
    np.save(npy, calib)

    # 输入 op 名必须是 ONNX 里那个名字。export_onnx.py 三个模型都写死成 "x"。
    import onnx2tf
    params = dict(
        input_onnx_file_path = args.onnx,
        output_folder_path = args.out_dir,
        output_integer_quantized_tflite = True,
        custom_input_op_name_np_data_path = [
            ["x", npy, [[[0.0, 0.0, 0.0]]], [[[1.0, 1.0, 1.0]]]]
        ],
        input_quant_dtype = "int8",
        output_quant_dtype = "int8",
        not_use_onnxsim = True,
        verbosity = "error",
        tflite_backend = "tf_converter",
    )
    try:
        onnx2tf.convert(**params)
    except Exception:
        traceback.print_exc()
        return 1

    if not args.keep_npy:
        os.remove(npy)

    got = sorted(glob.glob(os.path.join(args.out_dir, "*full_integer_quant.tflite")))
    if not got:
        print("!! 没生成 full_integer_quant.tflite，看看 %s" % args.out_dir)
        return 1
    dst = os.path.join(args.out_dir, "%s-int8.tflite" % stem)
    if os.path.exists(dst):
        os.remove(dst)
    os.replace(got[0], dst)
    print("  -> %s   %d bytes" % (os.path.relpath(dst, ROOT), os.path.getsize(dst)))
    for f in sorted(glob.glob(os.path.join(args.out_dir, "*integer_quant.tflite"))):
        print("     旁：%s  %d bytes" % (os.path.basename(f), os.path.getsize(f)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
