# -*- coding: utf-8 -*-
"""旧版导出脚本 —— 只导 ①（整图找牌），输出 `pytorch2onnx/yolo-fastestv2.onnx`。

⚠️ v1.0 的三网导出流程是 `tflite/export_onnx.py`（参数化，一条命令一张网，
产物 `tflite/onnx/plate.onnx` / `crop.onnx` / `charhead.onnx`，见
`training/RELEASE_v1.0_2stage.md` §10）。本脚本导出的图与那份**不是同一个文件**
（opset / 常量折叠后的字节不同），只用于手工对照，不要拿它当发布流程。
"""
import sys
import os
sys.dont_write_bytecode = True   # 不生成 __pycache__：model/、utils/ 保持干净

# python 执行脚本时放进 sys.path[0] 的是「脚本所在目录」而不是当前目录，所以要显式把
# 训练侧的代码包目录 training/ 补回 sys.path，否则下面的 import model / utils 会
# ModuleNotFoundError。路径一律由 __file__ 推出，在哪个目录下执行都能跑。
HERE  = os.path.dirname(os.path.abspath(__file__))   # pytorch2onnx/
ROOT  = os.path.dirname(HERE)                        # 工程根目录
TRAIN = os.path.join(ROOT, "training")               # 训练侧：model/ utils/ data/ weights/ img/ 都在这里
sys.path.insert(0, TRAIN)

import torch
from model.detector import Detector
from utils.utils import load_datafile

def export_onnx():
    cfg_path = os.path.join(TRAIN, "data", "voc.data")
    # v1.0 的 ① 权重：weights/voc-299-epoch-0.270855ap-model.pth
    # （model/ 下只剩旧的 voc-290，改回它会导出上一代模型）
    pth_path = os.path.join(TRAIN, "weights", "voc-299-epoch-0.270855ap-model.pth")  #改成你的权重路径
    save_onnx_path = os.path.join(HERE, "yolo-fastestv2.onnx")

    cfg = load_datafile(cfg_path)
    device = torch.device("cpu")
    #创建网络，和训练时保持一致
    #第 4 个参数 export_onnx 必须显式传 True：第 3 个位置参数是 load_param，
    #漏掉第 4 个参数会静默导出另一张图（六路裸张量、无 sigmoid/softmax），那张图不能量化上板
    model = Detector(cfg["classes"], cfg["anchor_num"], True, True).to(device)
    model.load_state_dict(torch.load(pth_path, map_location=device))
    model.eval()

    # dummy输入，和训练分辨率对齐 NCHW
    w, h = cfg["width"], cfg["height"]
    dummy_input = torch.randn(1, 3, h, w, dtype=torch.float32).to(device)

    #不传 input_names / output_names：输入名取 forward 的参数名 x，
    #两个输出自动命名为 permute_13 / permute_14。
    #下游 onnx2tf 量化脚本按名字 "x" 取输入，写成别的名字（例如 images）会对不上
    torch.onnx.export(
        model,
        dummy_input,
        save_onnx_path,
        verbose=False,
        opset_version=18,
        do_constant_folding=True,
    )
    print(f"✅导出完成：{save_onnx_path}")
    if os.path.exists(save_onnx_path + ".data"):
        print(f"⚠️权重被拆到附属文件 {save_onnx_path}.data（{os.path.getsize(save_onnx_path + '.data')} 字节），"
              f"搬运模型时要连它一起拷贝")
    else:
        print(f"✅权重全部内嵌：{os.path.getsize(save_onnx_path)} 字节，无 .onnx.data 附属文件")
    print("⚠️该onnx已含检测头（sigmoid/softmax 烘进图里），锚框解码、NMS 使用原版工程后处理")
    print(f"⚠️导出的是训练分辨率 {w}x{h}，须与 training/data/voc.data 的 width/height 一致")

if __name__ == "__main__":
    print("load param...")
    export_onnx()
