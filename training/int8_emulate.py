# -*- coding: utf-8 -*-
"""上板前的闸门：把 v1.0 三级链路整个换成 **int8 TFLite 模型**，在同一批 val 图上
和 float 版逐列对比。

为什么必须单独做这一步：`full_pipeline_e2e.py` 量的是 float 链路，而板子上跑的是
int8 —— 量化掉的点数和「框变差」都是**静默**的，不会崩、不会报错，只会整串率变低。
等烧进板子才发现，就分不清是量化、是后处理、还是坐标映射的问题。

这个脚本把 float 和 int8 两条链**在同一次运行里**跑同一批图，逐列并排，
所以任何一列的差都是「量化造成的」，没有别的解释。

另外量一个口径分歧：**③ 输出 class 0（"plate"）的频率**。
固件 `yolo_plate_read()` 跳过 class 0，Python 的 `assemble()` 不跳 ——
③ 只要偶尔吐 class 0，两边结果就会系统性不一致。

用法：
  python int8_emulate.py                      # 默认 val 全体 514 张
  python int8_emulate.py --limit 60           # 冒烟
"""

import sys
sys.dont_write_bytecode = True

import os
import time
import argparse
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
os.chdir(HERE)

import utils.utils as U                                             # noqa: E402
import model.detector as MD                                         # noqa: E402
from cascade_ceiling import load_names, assemble, cut_patch         # noqa: E402
from fw_emulate import fw_decode, fw_nms                            # noqa: E402
from two_stage_e2e import select_idx                                # noqa: E402
from train_char_head import SmallCharNet                            # noqa: E402
from full_pipeline_e2e import (read_labels, gt_plate_and_str,       # noqa: E402
                               crop_by_plate, pick_plate)

ROOT = os.path.dirname(HERE)
D_TFL = os.path.join(ROOT, "tflite")


# --------------------------------------------------------------------------
# 两条链的共同出口：raw [N, n_anchor, 85]，cx/cy 已是像素坐标，激活已烘进图里。
# --------------------------------------------------------------------------
def split_heads(raw):
    """raw -> (prod, cid, xyxy, n_layer, grids)，和 full_pipeline_e2e.decode_heads 一致。"""
    n_layer = 2
    grids = [(20, 20), (10, 10)] if raw.shape[1] == 1500 else [(40, 40), (20, 20)]
    prod = raw[:, :, 4] * raw[:, :, 5:].max(2)
    cid = raw[:, :, 5:].argmax(2)
    cx, cy, bw, bh = raw[:, :, 0], raw[:, :, 1], raw[:, :, 2], raw[:, :, 3]
    xyxy = np.stack([cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2], -1)
    return prod, cid, xyxy, n_layer, grids


class DetFwd:
    """一个检测器的前向。子类负责「float 输入 -> NHWC 输出」。

    输入统一是 (N,S,S,3) **uint8** —— 板子的相机就是这个形态，
    float 那条链再除 255，int8 那条链直接 -128。两条链拿到的是同一批像素。
    """

    def run(self, x_u8):        # -> list of NHWC float, 层序 [stride16, stride32]
        raise NotImplementedError

    def raw(self, cfg, x_u8):
        outs = self.run(x_u8)
        import torch
        return U.handel_preds_fused(outs, cfg, torch.device("cpu")).numpy()


class FloatDet(DetFwd):
    def __init__(self, net):
        self.net = net

    def run(self, x_u8):
        import io
        import contextlib
        import torch
        xb = torch.from_numpy(x_u8).permute(0, 3, 1, 2).float() / 255.0
        with torch.no_grad(), contextlib.redirect_stdout(io.StringIO()):
            # export_onnx=True 的 forward 每次都 print("export onnx ...")，
            # 不吞掉的话 514 张图就是 1028 行噪声
            return [t.numpy() for t in self.net(xb)]


class Int8Det(DetFwd):
    """TFLite int8：uint8 像素 -128 直接喂进去，输出反量化回 float 再走同一套后处理。

    这正是固件 `dequant_i8()` 干的事（yolo_inference.cpp:73），所以这里对上了，
    后面 `fw_decode` 的每一行就等于板上跑的每一行。
    """

    def __init__(self, path):
        import tensorflow as tf
        self.it = tf.lite.Interpreter(model_path = path)
        self.it.allocate_tensors()
        self.inp = self.it.get_input_details()[0]
        self.outs = self.it.get_output_details()
        sc, zp = self.inp["quantization"]
        # 用 float32 比 —— float64 的 1/255 转成 float32 才是 tflite 里的那个值，
        # 直接拿 float64 比会因为末位差异假报错（0.003921568627 vs 0.003921568859）
        assert np.float32(sc) == np.float32(1.0 / 255.0) and int(zp) == -128, \
            "输入量化参数和固件写死的不一样：scale %s zp %s" % (sc, zp)
        # 固件 decode_head 假定输出也是 (q+128)/255 —— 不是就得改 yolo_inference.cpp
        for o in self.outs:
            sc, zp = o["quantization"]
            assert np.float32(sc) == np.float32(1.0 / 255.0) and int(zp) == -128, \
                "输出 %s 量化参数异常：scale %s zp %s" % (o["name"], sc, zp)

    def run(self, x_u8):
        got = {}
        for i in range(x_u8.shape[0]):
            # 和固件一模一样：uint8 像素当作 (q+128)/255 的反量化值。
            # 保留 batch 维（tflite 输入是 4 维 [1,H,W,3]，少一维会 Dimension mismatch）
            self.it.set_tensor(self.inp["index"],
                               (x_u8[i:i + 1].astype(np.int16) - 128).astype(np.int8))
            self.it.invoke()
            for o in self.outs:
                got[tuple(o["shape"][1:])] = (self.it.get_tensor(o["index"]).astype(np.float32)
                                              + 128.0) / 255.0
        # 层序必须是 stride16(20x20) 在前，和 anchors.reshape(2,3,2) 一致
        return [got[(20, 20, 85)], got[(10, 10, 85)]]


class FloatCls:
    def __init__(self, net, sizes):
        self.net, self.sizes = net, sizes

    def run(self, patches_u8):     # (N,H,W,3) uint8 -> (N,) int
        import torch
        out = []
        for s in range(0, len(patches_u8), 512):
            b = torch.from_numpy(np.asarray(patches_u8[s:s + 512])).permute(0, 3, 1, 2).float() / 255.0
            with torch.no_grad():
                out.append(self.net(b).argmax(1).numpy())
        return np.concatenate(out)


class Int8Cls:
    def __init__(self, path):
        import tensorflow as tf
        self.it = tf.lite.Interpreter(model_path = path)
        self.it.allocate_tensors()
        self.inp = self.it.get_input_details()[0]
        self.out = self.it.get_output_details()[0]
        sc, zp = self.inp["quantization"]
        assert np.float32(sc) == np.float32(1.0 / 255.0) and int(zp) == -128, \
            "字符头输入量化参数异常：scale %s zp %s" % (sc, zp)

    def run(self, patches_u8):
        res = []
        for p in patches_u8:
            self.it.set_tensor(self.inp["index"],
                               (np.asarray(p, np.int16)[None] - 128).astype(np.int8))
            self.it.invoke()
            # argmax **直接在 int8 上做**：输出 scale 是正常数、zp 固定，
            # 单调变换不改变 argmax —— 固件也能这么省掉一次反量化
            res.append(int(np.argmax(self.it.get_tensor(self.out["index"])[0])))
        return np.asarray(res, np.int64)


# --------------------------------------------------------------------------
def blank():
    """一条链的累计量。②/③ 各自一份 —— 它们的差别只有「字是谁读的」，
    框是同一批（②选的），所以框质量、检出数、IoU 是共用的。"""
    return dict(n = 0, found = 0, iou = [], cls0 = 0, ncls = 0, pn = {},
                tag = {t: dict(zh = 0, zt = 0, ex = 0, pos = {}, exgt = {}, okgt = {})
                       for t in ("②", "③")})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default = "data/voc.data")
    ap.add_argument("--crop-data", default = "data_crop/voc.data")
    ap.add_argument("--split", default = "val")
    ap.add_argument("--plate-weights", default = "weights/voc-299-epoch-0.270855ap-model.pth")
    ap.add_argument("--crop-weights", default = "weights/voc_crop_ft-119-epoch-0.589619ap-model.pth")
    ap.add_argument("--charhead", default = "weights/charhead_16_32_64_32_64.pt")
    ap.add_argument("--tflite-plate",
                    default = os.path.join(D_TFL, "onnx2tf_int8_plate", "plate-int8.tflite"))
    ap.add_argument("--tflite-crop",
                    default = os.path.join(D_TFL, "onnx2tf_int8_crop", "crop-int8.tflite"))
    ap.add_argument("--tflite-charhead",
                    default = os.path.join(D_TFL, "onnx2tf_int8_charhead", "charhead-int8.tflite"))
    ap.add_argument("--margin", type = float, default = 0.10)
    ap.add_argument("--plate-conf", type = float, default = 0.25)
    ap.add_argument("--plate-nms", type = float, default = 0.45)
    ap.add_argument("--plate-maxbox", type = int, default = 64)
    ap.add_argument("--plate-pick", default = "area_conf")
    ap.add_argument("--plate-conf-margin", type = float, default = 0.20)
    ap.add_argument("--conf", type = float, default = 0.10)
    ap.add_argument("--nms", type = float, default = 0.30)
    ap.add_argument("--maxchars", type = int, default = 7)
    ap.add_argument("--maxbox", type = int, default = 64)
    ap.add_argument("--pad", type = float, default = 0.25)
    ap.add_argument("--limit", type = int, default = 0)
    args = ap.parse_args()

    import torch
    dev = torch.device("cpu")
    names = load_names("data/voc.names")
    pcfg = U.load_datafile(args.data)
    ccfg = U.load_datafile(args.crop_data)
    PW_CH = int(ccfg["width"])

    for p in (args.tflite_plate, args.tflite_crop, args.tflite_charhead):
        assert os.path.exists(p), "缺 int8 模型 %s（先跑 tflite/onnx2tf_int8.py）" % p

    pnet = MD.Detector(int(pcfg["classes"]), int(pcfg["anchor_num"]), True, True).to(dev)
    pnet.load_state_dict(torch.load(args.plate_weights, map_location = dev))
    pnet.eval()
    cnet = MD.Detector(int(ccfg["classes"]), int(ccfg["anchor_num"]), True, True).to(dev)
    cnet.load_state_dict(torch.load(args.crop_weights, map_location = dev))
    cnet.eval()
    ck = torch.load(args.charhead, map_location = dev, weights_only = False)
    P_W, P_H = ck["size_wh"]
    hnet = SmallCharNet(int(ck["classes"]), (P_H, P_W), *ck["c"]).to(dev)
    hnet.load_state_dict(ck["state_dict"])
    hnet.eval()

    print("加载 int8 模型 ...", flush = True)
    chains = {
        "float": (FloatDet(pnet), FloatDet(cnet), FloatCls(hnet, (P_W, P_H))),
        "int8": (Int8Det(args.tflite_plate), Int8Det(args.tflite_crop),
                 Int8Cls(args.tflite_charhead)),
    }

    paths = [ln.strip() for ln in open(pcfg[args.split]) if ln.strip()]
    if args.limit:
        paths = paths[:args.limit]

    acc = {k: blank() for k in chains}
    t0 = time.time()
    done = 0

    for p in paths:
        im = cv2.imread(p)
        if im is None:
            continue
        H, W = im.shape[:2]
        gstr = gt_plate_and_str(read_labels(p), names, W, H)[1]
        if not gstr:
            continue          # 没有车牌标注的图不参与（整图 val 里是 3 张）
        done += 1
        gp = gt_plate_and_str(read_labels(p), names, W, H)[0]

        im320_u8 = cv2.resize(im, (int(pcfg["width"]), int(pcfg["height"])),
                              interpolation = cv2.INTER_LINEAR)[None]

        for name, (dfwd, cfwd, hcls) in chains.items():
            a = acc[name]
            a["n"] += 1
            # --- ① ---
            prod, cid, xyxy, nl, grids = split_heads(dfwd.raw(pcfg, im320_u8))
            cands = [(xyxy[0, n], sc, cc) for n, sc, cc in
                     fw_decode(prod[0], cid[0], args.plate_conf,
                               args.plate_maxbox, nl, grids)]
            keep = fw_nms(cands, args.plate_nms)
            j = pick_plate(keep, args.plate_pick, args.plate_conf_margin)
            if j is None:
                continue
            sx, sy = W / float(pcfg["width"]), H / float(pcfg["height"])
            box = np.array([keep[j][0][0] * sx, keep[j][0][1] * sy,
                            keep[j][0][2] * sx, keep[j][0][3] * sy], np.float32)
            a["found"] += 1
            lt = np.maximum(box[:2], gp[:2]); rb = np.minimum(box[2:], gp[2:])
            wh = np.clip(rb - lt, 0, None)
            inter = wh[0] * wh[1]
            un = (box[2] - box[0]) * (box[3] - box[1]) + (gp[2] - gp[0]) * (gp[3] - gp[1]) - inter
            a["iou"].append(inter / un if un > 0 else 0.0)

            # --- ② ---
            cc, cw, ch = crop_by_plate(im, box, args.margin)
            if cc.size == 0:
                continue
            cr = cv2.resize(cc, (PW_CH, PW_CH), interpolation = cv2.INTER_LINEAR)[None]
            prod2, cid2, xyxy2, nl2, grids2 = split_heads(cfwd.raw(ccfg, cr))
            cands2 = [(xyxy2[0, n], sc, kl) for n, sc, kl in
                      fw_decode(prod2[0], cid2[0], args.conf, args.maxbox, nl2, grids2)]
            if not cands2:
                continue
            keep2 = fw_nms(cands2, args.nms)
            idx = select_idx(keep2, False, args.maxchars)
            if not idx:
                continue
            s2x, s2y = cw / float(PW_CH), ch / float(PW_CH)
            bpx = [np.array([keep2[q][0][0] * s2x, keep2[q][0][1] * s2y,
                             keep2[q][0][2] * s2x, keep2[q][0][3] * s2y], np.float32)
                   for q in idx]
            cx1 = np.array([keep2[q][0][0] for q in idx], np.float32)
            c1 = [names[keep2[q][2]] if 0 <= keep2[q][2] < len(names) else "?" for q in idx]

            # --- ③ ---
            patches = []
            for bp in bpx:
                pt = cut_patch(cc, bp, args.pad, (P_W, P_H))
                patches.append(pt if pt is not None else np.zeros((P_H, P_W, 3), np.uint8))
            lab3 = hcls.run(np.asarray(patches))
            a["ncls"] += len(lab3)
            a["cls0"] += int((lab3 == 0).sum())
            c3 = [names[int(k)] if 0 <= int(k) < len(names) else "?" for k in lab3]

            # --- 计分：② 和 ③ 各自算一遍，其他量是共用的 ---
            for k in range(len(gstr)):
                a["pn"][k] = a["pn"].get(k, 0) + 1
            for tag, chs in (("②", c1), ("③", c3)):
                t = a["tag"][tag]
                s = assemble(cx1, chs, 12)
                t["exgt"][len(gstr)] = t["exgt"].get(len(gstr), 0) + 1
                t["okgt"][len(gstr)] = t["okgt"].get(len(gstr), 0) + int(s == gstr)
                t["ex"] += int(s == gstr)
                for x, y in zip(gstr, s[:len(gstr)]):
                    t["zt"] += 1
                    t["zh"] += int(x == y)
                for k in range(len(gstr)):
                    t["pos"][k] = t["pos"].get(k, 0) + int(k < len(s) and s[k] == gstr[k])

        if done % 50 == 0:
            print("  %d 张  %.0fs" % (done, time.time() - t0), flush = True)

    # ---------------------- 打印 ----------------------
    print("=" * 96)
    print("### int8 闸门：同一批 val 图，float 链 vs int8 链（逐列并排）")
    print("  ① %s   conf %.2f/nms %.2f  挑框 %s" % (
        os.path.basename(args.tflite_plate), args.plate_conf, args.plate_nms, args.plate_pick))
    print("  ② %s   conf %.2f/nms %.2f  cap %d   margin %.2f"
          % (os.path.basename(args.tflite_crop), args.conf, args.nms, args.maxchars, args.margin))
    print("  ③ %s   pad %.2f" % (os.path.basename(args.tflite_charhead), args.pad))
    print("  评估 %d 张" % done)
    print("=" * 96)
    print("%-7s %-5s | %-12s %-7s | %-15s | %-15s | %-15s"
          % ("链路", "级", "车牌检出", "IoU中位", "逐字率", "整串率", "位0 / 位1-6"))
    print("-" * 96)


    def pct(a, tag, key, den):
        return 100.0 * a["tag"][tag][key] / max(den, 1)


    for name in chains:
        a = acc[name]
        n = max(a["n"], 1)
        p0n = max(a["pn"].get(0, 0), 1)
        rn = sum(v for k, v in a["pn"].items() if k >= 1) or 1
        for tag in ("②", "③"):
            t = a["tag"][tag]
            p0 = 100.0 * t["pos"].get(0, 0) / p0n
            rest = 100.0 * sum(v for k, v in t["pos"].items() if k >= 1) / rn
            print("%-7s %-5s | %3d/%3d %5.1f%%  %5.3f   | %5.2f%%          | %5.2f%%          | %5.2f%% / %5.2f%%"
                  % (name if tag == "②" else "", tag,
                     a["found"], a["n"], 100.0 * a["found"] / n,
                     float(np.median(a["iou"])) if a["iou"] else float("nan"),
                     100.0 * t["zh"] / max(t["zt"], 1), 100.0 * t["ex"] / n, p0, rest))
        print("-" * 96)

    for name in chains:
        a = acc[name]
        print("  [%s] ③ 共判 %d 个字，其中 class 0（固件会跳过）%d 个 = %.3f%%"
              % (name, a["ncls"], a["cls0"], 100.0 * a["cls0"] / max(a["ncls"], 1)))

    fl, i8 = acc["float"], acc["int8"]
    d_found = i8["found"] - fl["found"]
    d_ex = pct(i8, "③", "ex", i8["n"]) - pct(fl, "③", "ex", fl["n"])
    d_zh = pct(i8, "③", "zh", i8["tag"]["③"]["zt"]) - pct(fl, "③", "zh", fl["tag"]["③"]["zt"])
    d_ex2 = pct(i8, "②", "ex", i8["n"]) - pct(fl, "②", "ex", fl["n"])
    print("\n  Δ(int8 - float)：车牌检出 %+d 张  |  ② 整串 %+.2f 点  |  "
          "③ 整串 %+.2f 点，逐字 %+.2f 点" % (d_found, d_ex2, d_ex, d_zh))
    ok = d_ex >= -2.0 and d_ex2 >= -2.0 and d_found >= -6
    print("  闸门（②/③ 整串掉幅 <= 2 点、车牌检出掉 <= 6 张）：**%s**"
          % ("通过，可以开始接固件" if ok else "不过 —— 先别碰固件，把差在哪一段找出来"))
    print("\n用时 %.0fs" % (time.time() - t0))


if __name__ == "__main__":
    main()
