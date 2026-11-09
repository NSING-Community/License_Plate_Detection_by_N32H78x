# -*- coding: utf-8 -*-
"""loss 三分量量级诊断（只读：不写任何文件、不保存任何权重，只往 stdout 打）。

要回答的问题：`plate-chain-eval-2026-10-15` 那批评测显示「定位够用、分类很弱」
（cls% 卡 64%、argmax 仅 52%），而 obj 完全饱和。头号嫌疑是 loss.py:204 的

    lcls += BCEcls(ps[:, :], tcls[layer_index[i]]) / cfg["classes"]

—— `BCEcls` 是 nn.CrossEntropyLoss（已按 batch 平均），再除 70 是重复缩小，
配合 lcls*=32 / lobj*=64，分类项等效权重只有 32/70≈0.46，而 obj 是 64。

训练日志只记每 10 轮的 AP 表，不记各项 loss 量级，所以之前只能从代码推。本脚本
把推理变成观测：跑 N 个训练 batch，记录

  1. `compute_loss` 返回的 lbox/lobj/lcls（注意已是**加权后**的值，见 loss.py:209-211），
     直接给出每项在 total 里的占比；
  2. 反解出**未加权**的原值：mean CE（单位 nat）换算成「平均正确类概率」，
     mean BCE_obj，mean(1-CIoU)。mean CE 这一项最直观：随机猜是 ln(70)=4.248，
     模型真在学的话应该在 0.5~1.5。
  3. 三个检测头各自的梯度范数 —— 这是最硬的证据：分类头如果只在被 1/70 的折扣推，
     它的梯度范数就该比 obj 头小一到两个数量级。

解耦系数（都来自 loss.py，改那边的话这里要跟着改）：
  lcls 返回 = 32 * (CE_层0 + CE_层1) / classes     ->  mean CE = lcls / 32 * classes / 2
  lobj 返回 = 64 * (BCE_层0*1.0 + BCE_层1*0.4)     ->  两层不同权，只反解总量
  lbox 返回 = 3.2 * (CIoU_层0 + CIoU_层1)          ->  mean(1-CIoU) = lbox / 3.2 / 2
  （cls/obj/reg 各 2 层：preds 是 6 个张量，i%3 分别取 0,3 / 1,4 / 2,5）

用法：
  python diag_loss.py
  python diag_loss.py --weights weights/voc-299-epoch-0.270855ap-model.pth --batches 20
"""

import sys
sys.dont_write_bytecode = True

import os
import math
import argparse
import numpy as np

import torch
import torchvision          # noqa: F401  —— 顶层导入，别让第一个 batch 去付这个 1.2 s

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
os.chdir(HERE)

import utils.utils as U      # noqa: E402
import utils.datasets as D   # noqa: E402
import utils.loss as L       # noqa: E402
import model.detector as MD  # noqa: E402

# loss.py 里的加权系数，反解原值要用
W_CLS, W_OBJ, W_BOX = 32.0, 64.0, 3.2
# obj 的 balance = [1.0, 0.4]，按 layer_index 取；用来把 obj 的原值拆成「每层平均」
OBJ_BALANCE = (1.0, 0.4)
# 每个分支的特征图层数（preds 有 6 个张量）
N_LAYER = 2


def head_grad_norm(model, name):
    """该头自己的参数（weight/bias）上的梯度 L2 范数。共享的 backbone/FPN 不算在内，
    所以它是「这个分支在把自己往哪推」的干净代理。"""
    tot = 0.0
    for pname, p in model.named_parameters():
        if pname.startswith(name) and p.grad is not None:
            tot += float(p.grad.detach().pow(2).sum())
    return math.sqrt(tot)


def all_grad_norm(model):
    tot = 0.0
    for p in model.parameters():
        if p.grad is not None:
            tot += float(p.grad.detach().pow(2).sum())
    return math.sqrt(tot)


def obj_stats(preds):
    """obj 头原值的分布：正样本命中率、以及「无关框也被打高分」的程度。
    preds 里 i%3==1 的是 obj，形状 [B, 1, H, W]（anchor 维在 loss 里 reshape 出来）。"""
    s = {}
    for i, p in enumerate(preds):
        if i % 3 != 1:
            continue
        o = p.detach().sigmoid().flatten()
        s["layer%d" % (i // 3)] = dict(
            mean = float(o.mean()),
            gt50 = float((o > 0.5).float().mean()),
            gt90 = float((o > 0.9).float().mean()),
            mx = float(o.max()))
    return s


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default = "weights/voc-299-epoch-0.270855ap-model.pth")
    ap.add_argument("--data", default = "data/voc.data")
    ap.add_argument("--batches", type = int, default = 20)
    ap.add_argument("--split", default = "train", choices = ("train", "val"),
                    help = "train 和 val 各跑一遍比 CE：train≈val ⇒ 欠拟合（不是过拟合），"
                           "练不够/容量不够；train 远低于 val ⇒ 过拟合。")
    ap.add_argument("--imgaug", action = "store_true",
                    help = "打开训练集增强（train.py 是开的）。默认关，为了可复现。")
    args = ap.parse_args()

    cfg = U.load_datafile(args.data)
    names = U.load_names(cfg["names"])
    device = torch.device("cpu")

    ds = D.TensorDataset(cfg[args.split], cfg["width"], cfg["height"], imgaug = args.imgaug)
    loader = torch.utils.data.DataLoader(
        ds,
        batch_size = int(cfg["batch_size"] / cfg["subdivisions"]),
        shuffle = False,
        collate_fn = D.collate_fn,
        num_workers = 0,
        pin_memory = False,
        drop_last = True)

    # 与 train.py:67 完全一样的类别权重，否则量级对不上
    cls_weights, cls_counts = U.class_weights(cfg["train"], cfg["classes"])
    cls_weight = (torch.tensor(cls_weights, dtype = torch.float32, device = device)
                  if cls_weights is not None else None)

    model = MD.Detector(cfg["classes"], cfg["anchor_num"], True).to(device)
    model.load_state_dict(torch.load(args.weights, map_location = device))
    model.train()            # BN 要 train 模式才和训练时一致；只在内存里，不落盘

    print("权重 %s   split=%s" % (args.weights, args.split))
    print("batch=%d  imgaug=%s  classes=%d  anchor_num=%d"
          % (int(cfg["batch_size"] / cfg["subdivisions"]), args.imgaug,
             cfg["classes"], cfg["anchor_num"]))
    print("反解系数: cls /%.0f *%.0f    obj *%.0f    box *%.1f"
          % (cfg["classes"], W_CLS, W_OBJ, W_BOX))

    rows = []
    for bi, (imgs, targets) in enumerate(loader):
        if bi >= args.batches:
            break
        imgs = imgs.to(device).float() / 255.0
        targets = targets.to(device)

        model.zero_grad(set_to_none = True)
        preds = model(imgs)
        lbox, lobj, lcls, total = L.compute_loss(preds, targets, cfg, device, cls_weight)
        total.backward()

        r = dict(
            lbox = float(lbox), lobj = float(lobj), lcls = float(lcls),
            total = float(total),
            g_reg = head_grad_norm(model, "output_reg_layers"),
            g_obj = head_grad_norm(model, "output_obj_layers"),
            g_cls = head_grad_norm(model, "output_cls_layers"),
            g_all = all_grad_norm(model),
            n_gt = int(targets.shape[0]))
        rows.append(r)
        print("  batch %2d/%d  lbox %8.4f  lobj %8.4f  lcls %8.4f  total %8.4f   "
              "|g| cls %8.4f  obj %8.4f  reg %8.4f  (gt框 %d)"
              % (bi + 1, args.batches, r["lbox"], r["lobj"], r["lcls"], r["total"],
                 r["g_cls"], r["g_obj"], r["g_reg"], r["n_gt"]))

    if not rows:
        print("没跑到任何 batch —— 检查 data/train.txt 和 batch_size/subdivisions")
        return

    m = lambda k: float(np.mean([r[k] for r in rows]))
    lbox, lobj, lcls, total = m("lbox"), m("lobj"), m("lcls"), m("total")
    g_reg, g_obj, g_cls, g_all = m("g_reg"), m("g_obj"), m("g_cls"), m("g_all")

    print("\n" + "=" * 84)
    print("### 平均（%d 个 batch）" % len(rows))
    print("=" * 84)
    print("  %-8s %10s %10s" % ("项", "加权后", "占 total"))
    for label, v in (("lbox", lbox), ("lobj", lobj), ("lcls", lcls)):
        print("  %-8s %10.5f %9.2f%%" % (label, v, 100.0 * v / max(total, 1e-12)))
    print("  %-8s %10.5f" % ("total", total))

    print("\n  --- 反解出的原值（去掉 loss.py 里的加权）---")
    mean_ce = lcls / W_CLS * cfg["classes"] / N_LAYER
    print("  mean CE（每层平均，单位 nat）        %8.4f" % mean_ce)
    print("  随机猜的 CE = ln(%d)                %8.4f" % (cfg["classes"],
                                                          math.log(cfg["classes"])))
    print("  ⇒ 平均「正确类概率」                %8.4f" % math.exp(-mean_ce))
    print("     （CE≈ln(70) 说明分类头没在学；CE≈0.7 对应 p≈0.5）")
    mciou = lbox / W_BOX / N_LAYER
    print("  mean(1-CIoU)（每层平均）            %8.4f" % mciou)
    print("  mean BCE_obj（两层，权重 %.1f/%.1f）  %8.4f"
          % (OBJ_BALANCE[0], OBJ_BALANCE[1], lobj / W_OBJ / sum(OBJ_BALANCE)))

    print("\n  --- 若去掉 loss.py:204 那个 /classes（lcls 会乘 %d 倍）---" % cfg["classes"])
    lcls_fix = lcls * cfg["classes"]
    total_fix = lbox + lobj + lcls_fix
    print("  lcls %8.5f -> %8.5f（占 total %.2f%% -> %.2f%%），total %.4f -> %.4f"
          % (lcls, lcls_fix, 100.0 * lcls / max(total, 1e-12),
             100.0 * lcls_fix / max(total_fix, 1e-12), total, total_fix))
    print("  注：占比只是表象，真正要看的是下面 cls 头的梯度范数会不会跟着涨。")

    print("\n  --- 检测头梯度范数（共享 backbone/FPN 不计入）---")
    print("  %-6s %12s %10s" % ("头", "|grad|", "相对 obj"))
    for label, g in (("reg", g_reg), ("obj", g_obj), ("cls", g_cls)):
        print("  %-6s %12.5f %9.3fx" % (label, g, g / max(g_obj, 1e-12)))
    print("  全模型 |grad| %.5f" % g_all)
    print("  判读：cls 若比 obj 小 1~2 个数量级 ⇒ 分类头确实被加权压住了，"
          "改 loss.py:204 的归一化有意义；\n"
          "        若两者同量级 ⇒ 加权不是主因，锅在别处（分辨率/增强/类别本身难分）。")

    print("\n  --- obj 头原始输出分布（sigmoid 后，全图全 anchor）---")
    # 用最后一个 batch 的 preds 打一眼即可
    for k, v in obj_stats(preds).items():
        print("      %s  均值 %.4f  >0.5 占 %.2f%%  >0.9 占 %.2f%%  最大 %.4f"
              % (k, v["mean"], 100.0 * v["gt50"], 100.0 * v["gt90"], v["mx"]))
    print("      （>0.5 占比远大于真实正样本率 ⇒ obj 没有区分度，"
          "这解释了评测里 obj 十分位就是 1.000）")


if __name__ == "__main__":
    main()
