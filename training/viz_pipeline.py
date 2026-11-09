# -*- coding: utf-8 -*-
"""把 v1.0 两级链路跑在 img/ 的照片上，并把**车牌框 / 字符框 / 识别结果**画出来。

链路逐段对应 `RELEASE_v1.0_2stage.md` §4：

  1. 整图 resize 320×320 → ① 前向 → 固件后处理（`fw_decode` + `fw_nms`）
     → 取**面积最大**的 class 0 = 车牌框（复用 `full_pipeline_e2e.pick_plate`）
  2. 车牌框外扩 `--margin`（默认 0.10）→ **原生分辨率**裁 → resize 320×320
  3. ② 前向 → 固件后处理 → 字符框：分数序取前 16 个 class>0 → 按分数砍到 `--maxchars`
     → 按 **x1 升序**排（`two_stage_e2e.select_idx`，就是固件 `yolo_plate_read` 的规则）
  4. 每个字符框从**原生分辨率的裁剪图**上抠（外扩 `pad=0.25`）→ resize 32×64
  5. ③ 第二级 argmax → 按 x1 拼串（GBK 截断 12 字节）

输出：输入同目录、同格式的 **`<名字>result.<后缀>`**。画布上下两块 ——
上面是原图（绿框 = 车牌框 + 顶部结果条），下面是**放大后的车牌裁剪**
（红框 = 字符框，框上标出③ 读出的字和它的置信度）。小字符框只有放大才看得清。

用法（在 training/ 下）：
  python viz_pipeline.py                        # 默认跑 img/
  python viz_pipeline.py --limit 3              # 先跑几张
  python viz_pipeline.py --maxchars 0           # 不砍框，看多出来的框长什么样
"""

import sys
sys.dont_write_bytecode = True

import os
import argparse
import numpy as np
import cv2
import torch

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
os.chdir(HERE)

import utils.utils as U                                        # noqa: E402
import model.detector as MD                                    # noqa: E402
from cascade_ceiling import load_names, assemble, cut_patch     # noqa: E402
from fw_emulate import fw_decode, fw_nms                       # noqa: E402
from two_stage_e2e import select_idx                            # noqa: E402
from full_pipeline_e2e import crop_by_plate, pick_plate, decode_heads   # noqa: E402

from train_char_head import SmallCharNet                       # noqa: E402

from PIL import Image, ImageDraw, ImageFont                    # noqa: E402

# cv2.putText 的 Hershey 字体画不了汉字，标注必须走 PIL，字体按优先级找。
FONT_CANDIDATES = [
    "C:/Windows/Fonts/msyh.ttc",       # 微软雅黑
    "C:/Windows/Fonts/simhei.ttf",     # 黑体
    "C:/Windows/Fonts/simsun.ttc",     # 宋体
]

EXTS = (".jpg", ".jpeg", ".png", ".bmp", ".webp")

C_HEADER = (28, 28, 28)
C_TXT = (235, 235, 235)
C_DIM = (165, 165, 165)
C_OK = (0, 255, 120)
C_BOX = (0, 255, 0)
C_GOOD = (0, 220, 0)
C_BAD = (255, 60, 60)
C_WARN = (190, 40, 40)


def get_font(size):
    for p in FONT_CANDIDATES:
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, size)
            except Exception:
                continue
    return ImageFont.load_default()


def run_chain(im, nets, cfgs, args, dev):
    """跑完整链路。返回 dict；坐标一律是**原图像素**。

    chars 每一项：
      box       原图坐标（给上面那块画用）
      crop_box  裁剪图局部坐标（给下面那块放大画用）
      cls/score ② 认为的类别 / 分数
      pred/prob/pred_ch  ③ 读出的类别 id / 置信度 / 字符
      crop_origin, crop_wh  还原原图坐标用的裁剪原点 / 裁剪尺寸
    """
    pnet, cnet, hnet = nets
    pcfg, ccfg, names = cfgs
    H, W = im.shape[:2]

    # ---- 1. 整图找车牌 ----
    PW = int(pcfg["width"])
    xb = cv2.resize(im, (PW, PW), interpolation = cv2.INTER_LINEAR)[None]
    xb = torch.from_numpy(xb).permute(0, 3, 1, 2).float() / 255.0
    prod, cid, xyxy, nl, grids = decode_heads(pnet, pcfg, xb, dev)
    cands = [(xyxy[0, n], sc, cc) for n, sc, cc in
             fw_decode(prod[0], cid[0], args.plate_conf, args.plate_maxbox, nl, grids)]
    keep = fw_nms(cands, args.plate_nms)
    j = pick_plate(keep, args.plate_pick, args.plate_conf_margin)
    out = dict(plate_box = None, plate_score = 0.0, n_cand = len(keep),
               chars = [], string = "", crop = None)
    if j is None:
        return out

    sx, sy = W / float(PW), H / float(pcfg["height"])
    plate_box = np.array([keep[j][0][0] * sx, keep[j][0][1] * sy,
                          keep[j][0][2] * sx, keep[j][0][3] * sy], np.float32)
    out["plate_box"] = plate_box
    out["plate_score"] = float(keep[j][1])

    # ---- 2. 外扩裁剪（原生分辨率）----
    cc, cw, ch = crop_by_plate(im, plate_box, args.margin)
    if cc.size == 0:
        return out
    out["crop"] = cc
    # 裁剪原点：和 crop_by_plate 的取整方式必须一致（floor 左上 / ceil 右下 / clip）
    mx, my = (plate_box[2] - plate_box[0]) * args.margin, \
             (plate_box[3] - plate_box[1]) * args.margin
    ox = int(np.clip(np.floor(plate_box[0] - mx), 0, W - 1))
    oy = int(np.clip(np.floor(plate_box[1] - my), 0, H - 1))

    # ---- 3. 裁剪版模型找字符 ----
    CW = int(ccfg["width"])
    cb = cv2.resize(cc, (CW, CW), interpolation = cv2.INTER_LINEAR)[None]
    cb = torch.from_numpy(cb).permute(0, 3, 1, 2).float() / 255.0
    prod2, cid2, xyxy2, nl2, grids2 = decode_heads(cnet, ccfg, cb, dev)
    cands2 = [(xyxy2[0, n], sc, k) for n, sc, k in
              fw_decode(prod2[0], cid2[0], args.conf, args.maxbox, nl2, grids2)]
    if not cands2:
        return out
    keep2 = fw_nms(cands2, args.nms)
    idx = select_idx(keep2, False, args.maxchars)
    if not idx:
        return out

    csx, csy = cw / float(CW), ch / float(CW)      # 网络空间 -> 裁剪图局部像素
    cboxes = [np.array([keep2[k][0][0] * csx, keep2[k][0][1] * csy,
                        keep2[k][0][2] * csx, keep2[k][0][3] * csy], np.float32)
              for k in idx]

    # ---- 4. 原生分辨率抠字符 -> 32×64 ----
    PW_CH, PH_CH = hnet.in_w, hnet.in_h
    patches = []
    for bp in cboxes:
        pt = cut_patch(cc, bp, args.pad, (PW_CH, PH_CH))
        patches.append(pt if pt is not None else np.zeros((PH_CH, PW_CH, 3), np.uint8))
    pb = torch.from_numpy(np.asarray(patches)).permute(0, 3, 1, 2).float() / 255.0
    with torch.no_grad():
        logits = hnet(pb.to(dev))
        prob = torch.softmax(logits, 1).max(1)
    pred = logits.argmax(1).cpu().numpy()
    pconf = prob.values.cpu().numpy()

    # ---- 5. 拼串 ----
    off = np.array([ox, oy, ox, oy], np.float32)
    for q, (k, bp) in enumerate(zip(idx, cboxes)):
        pi = int(pred[q])
        out["chars"].append(dict(
            box = bp + off, crop_box = bp,
            cls = int(keep2[k][2]), score = float(keep2[k][1]),
            pred = pi, prob = float(pconf[q]),
            pred_ch = names[pi] if 0 <= pi < len(names) else "?",
        ))
    out["string"] = assemble(np.array([c["box"][0] for c in out["chars"]], np.float32),
                             [c["pred_ch"] for c in out["chars"]], 12)
    return out


def draw_result(im, res, args):
    """上：原图（车牌框 + 结果条）；下：放大车牌（字符框 + 逐字标签）。

    两块都占满画布宽度：下块的裁剪图是**按比例放大到同宽**的，
    宽度被高度上限压住时就**水平居中**（不居中会在右边留一大条黑边）。
    """
    H, W = im.shape[:2]
    W0 = min(W, args.panel_width)
    if args.max_top_height:
        W0 = min(W0, int(round(W * args.max_top_height / float(H))))
    W0 = max(320, W0)
    k = W0 / float(W)                       # 取整回代，保证框坐标和画布严丝合缝
    top_h = max(1, int(round(H * k)))
    top = cv2.resize(im, (W0, top_h), interpolation = cv2.INTER_AREA)

    # 下块：车牌裁剪**放大到占满同宽**；只有高度超上限时才缩、并水平居中。
    crop = res["crop"]
    bot, kc, bx = None, 1.0, 0
    if crop is not None and crop.size:
        ch, cw = crop.shape[:2]
        kc = min(W0 / float(cw), args.max_plate_height / float(ch))
        bw, bh = int(round(cw * kc)), int(round(ch * kc))
        bx = (W0 - bw) // 2
        bot = cv2.resize(crop, (bw, bh), interpolation = cv2.INTER_CUBIC)

    hdr_h, sub_h, lab_h = 38, 30, 46
    bot_h = bot.shape[0] if bot is not None else 0
    tail = (sub_h + bot_h + lab_h) if bot is not None else 0
    canvas = np.full((hdr_h + top_h + tail, W0, 3), 24, np.uint8)
    canvas[hdr_h:hdr_h + top_h] = top
    yb = hdr_h + top_h + sub_h
    if bot is not None:
        canvas[yb:yb + bot_h, bx:bx + bot.shape[1]] = bot

    pil = Image.fromarray(canvas[:, :, ::-1])
    d = ImageDraw.Draw(pil)
    f_head = get_font(24)
    f_small = get_font(17)
    # 标签带的字号要能塞下**所有**标签：小图（比如 378 宽的）7 个「字 0.00」在 22 号字下
    # 总宽超过画布，最后一个会被挤到边界外/盖住前一个。从大到小挑第一个装得下的。
    labs = ["%s %.2f" % (c["pred_ch"], c["prob"]) for c in res["chars"]]
    GAP = 12                                    # 标签之间至少留这么多，否则会连成一片
    f_lab = get_font(22)
    for sz in (22, 20, 18, 16, 14, 13, 12, 11, 10, 9):
        f_lab = get_font(sz)
        need = sum(d.textbbox((0, 0), t, font = f_lab)[2] for t in labs) \
               + GAP * max(0, len(labs) - 1)
        if need <= W0 - 10:
            break

    # ---- 顶栏：结果条 ----
    if res["plate_box"] is None:
        d.rectangle([0, 0, W0, hdr_h], fill = C_WARN)
        d.text((10, 7), "未检出车牌 —— 整条链失败（全链路 3.5% 的硬缺口）",
               font = f_head, fill = (255, 255, 255))
    else:
        d.rectangle([0, 0, W0, hdr_h], fill = C_HEADER)
        s = res["string"] or "（找到了车牌，但没挑出字符框）"
        d.text((10, 7), "识别结果:  %s" % s, font = f_head, fill = C_OK)
        w = d.textbbox((0, 0), "识别结果:  %s" % s, font = f_head)[2]
        d.text((10 + w + 28, 11), "车牌框 conf %.2f" % res["plate_score"],
               font = f_small, fill = C_DIM)

    # ---- 上块：原图 + 车牌框 ----
    if res["plate_box"] is not None:
        b = res["plate_box"] * k
        d.rectangle([b[0], hdr_h + b[1], b[2], hdr_h + b[3]], outline = C_BOX, width = 3)

    # ---- 下块：放大车牌（只画框）+ 下方一条逐字标签带 ----
    # 标签不压在车牌上 —— 压在车牌上会把字盖住，反而看不清「框得准不准」。
    if bot is not None:
        d.rectangle([0, hdr_h + top_h, W0, hdr_h + top_h + sub_h], fill = C_HEADER)
        d.text((10, hdr_h + top_h + 6),
               "车牌裁剪放大 %.1f×  ｜  蓝框 = ② 给出的字符框；下面是 ③ 读出的字 + 置信度" % kc,
               font = f_small, fill = C_DIM)
        d.rectangle([0, yb + bot_h, W0, yb + bot_h + lab_h], fill = C_HEADER)
        centers = []
        for c in res["chars"]:
            cb = c["crop_box"] * kc
            x0, y0 = bx + cb[0], yb + cb[1]
            x1, y1 = bx + cb[2], yb + cb[3]
            # 蓝框 = ② 的框；红框 = ③ 读出的字和 ② 的类不一致（那时候才值得看一眼）
            col = C_BOX if c["pred"] == c["cls"] else C_BAD
            d.rectangle([x0, y0, x1, y1], outline = col, width = 2)
            centers.append(((x0 + x1) / 2.0, c, col))
        # 标签带：按框中心对齐，太挤就整体左移（避免叠在一起）
        prev_right = -1e9
        for (cxx, c, col), lab in zip(centers, labs):
            lw = d.textbbox((0, 0), lab, font = f_lab)[2]
            tx = cxx - lw / 2.0
            if tx < prev_right + GAP:
                tx = prev_right + GAP
            tx = min(tx, W0 - lw - 4)
            prev_right = tx + lw
            d.text((tx, yb + bot_h + 9), lab, font = f_lab, fill = col)

    return np.array(pil)[:, :, ::-1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default = "img")
    ap.add_argument("--limit", type = int, default = 0)
    ap.add_argument("--plate-weights", default = "weights/voc-299-epoch-0.270855ap-model.pth")
    ap.add_argument("--crop-weights", default = "weights/voc_crop_ft-119-epoch-0.589619ap-model.pth")
    ap.add_argument("--charhead", default = "weights/charhead_16_32_64_32_64.pt")
    ap.add_argument("--plate-conf", type = float, default = 0.25)
    ap.add_argument("--plate-nms", type = float, default = 0.45)
    ap.add_argument("--plate-maxbox", type = int, default = 64)
    ap.add_argument("--plate-pick", default = "area_conf",
                    choices = ["area", "score", "area_conf"],
                    help = "① 挑车牌框的规则。area = v1.0 冻结的规则（= 训练期 best_plate）；"
                           "area_conf = 先扔掉比最高分低 0.2 以上的框再取面积最大 —— "
                           "val 上和 area 打平（79.18% vs 78.99%，1 张图），"
                           "但在 img/ 这批手机照片上能修掉「选了一块地面」的两张")
    ap.add_argument("--plate-conf-margin", type = float, default = 0.20)
    ap.add_argument("--conf", type = float, default = 0.10)
    ap.add_argument("--nms", type = float, default = 0.30)
    ap.add_argument("--maxbox", type = int, default = 64)
    ap.add_argument("--maxchars", type = int, default = 7,
                    help = "NMS 之后按分数砍到 N 个框（0 = 不砍，固件原样）")
    ap.add_argument("--margin", type = float, default = 0.10,
                    help = "车牌框外扩比例 —— 这是模型契约，别改（裁剪必须外扩 10%）")
    ap.add_argument("--pad", type = float, default = 0.25,
                    help = "字符框外扩比例 —— 和第二级训练时一致，别改")
    ap.add_argument("--suffix", default = "result")
    ap.add_argument("--panel-width", type = int, default = 1280)
    ap.add_argument("--max-top-height", type = int, default = 0,
                    help = "原图那块的高度上限；0 = 不限（按宽度缩，竖拍照片会长，但那才是原图）")
    ap.add_argument("--max-plate-height", type = int, default = 700,
                    help = "放大车牌那块的高度上限 —— ① 给的车牌框往往偏松（上下留白多），"
                           "裁剪块接近正方形，上限太小就只能放大到 0.8 倍，反而看不清")
    args = ap.parse_args()

    dev = torch.device("cpu")
    pcfg = U.load_datafile("data/voc.data")
    ccfg = U.load_datafile("data_crop/voc.data")
    names = load_names("data/voc.names")

    pnet = MD.Detector(int(pcfg["classes"]), int(pcfg["anchor_num"]), True).to(dev)
    pnet.load_state_dict(torch.load(args.plate_weights, map_location = dev))
    pnet.eval()
    cnet = MD.Detector(int(ccfg["classes"]), int(ccfg["anchor_num"]), True).to(dev)
    cnet.load_state_dict(torch.load(args.crop_weights, map_location = dev))
    cnet.eval()
    ck = torch.load(args.charhead, map_location = dev, weights_only = False)
    PW_CH, PH_CH = ck["size_wh"]
    hnet = SmallCharNet(int(ck["classes"]), (PH_CH, PW_CH), *ck["c"]).to(dev)
    hnet.load_state_dict(ck["state_dict"])
    hnet.eval()
    # 把输入尺寸挂在实例上，run_chain 就不用再判一次
    hnet.in_w, hnet.in_h = PW_CH, PH_CH

    # 只处理"原始"图：跳过本工具写过的东西，否则标注图会被当成输入再跑一遍，
    # 而且换个 --suffix 重跑时上一轮的产物 stem 里不含新后缀，过滤不掉，会滚雪球。
    def is_original(f):
        if not f.lower().endswith(EXTS):
            return False
        stem = os.path.splitext(f)[0]
        return "result" not in stem and args.suffix not in stem

    files = sorted(f for f in os.listdir(args.dir) if is_original(f))
    if args.limit:
        files = files[:args.limit]
    if not files:
        print("!! %s 下没有可用图片" % args.dir)
        return 1

    print("=" * 100)
    print("### v1.0 两级链路可视化   %s   共 %d 张" % (args.dir, len(files)))
    print("  ① %-44s conf %.2f / NMS %.2f / 挑框 %s" % (
        os.path.basename(args.plate_weights), args.plate_conf, args.plate_nms,
        args.plate_pick))
    print("  ② %-44s conf %.2f / NMS %.2f / maxchars %s" % (
        os.path.basename(args.crop_weights), args.conf, args.nms,
        args.maxchars if args.maxchars else "不砍"))
    print("  ③ %-44s 输入 %d×%d  pad %.2f" % (os.path.basename(args.charhead),
                                               PW_CH, PH_CH, args.pad))
    print("=" * 100)

    n_ok = 0
    for f in files:
        p = os.path.join(args.dir, f)
        im = cv2.imread(p)
        if im is None:
            print("  !! 读不了 %s" % f)
            continue
        res = run_chain(im, (pnet, cnet, hnet), (pcfg, ccfg, names), args, dev)
        base, ext = os.path.splitext(f)
        out = os.path.join(args.dir, "%s%s%s" % (base, args.suffix, ext))
        cv2.imwrite(out, draw_result(im, res, args))
        if res["string"]:
            n_ok += 1
        print("  %s  %-12s 车牌照=%s(%2d 框)  ->  %s" % (
            "OK " if res["string"] else "ERR", f,
            "有" if res["plate_box"] is not None else "无", res["n_cand"],
            res["string"] or "（空）"))
    print("-" * 100)
    print("  写出 %d 张 <名字>%s.<后缀>；有结果的 %d / %d" % (
        len(files), args.suffix, n_ok, len(files)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
