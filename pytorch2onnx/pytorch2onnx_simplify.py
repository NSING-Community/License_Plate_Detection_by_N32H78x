# -*- coding: utf-8 -*-
"""旧版「导出 + onnxsim」脚本 —— 只导 ①，输出同目录的 yolo-fastestv2.onnx。

⚠️ v1.0 的三网导出流程是 `tflite/export_onnx.py`（参数化，见
`training/RELEASE_v1.0_2stage.md` §10）。本脚本保留作手工对照：
它的默认权重已改为 v1.0 的 ①（weights/voc-299），但导出的图与发布用的
`tflite/onnx/plate.onnx` **不是同一个文件**，不要用它覆盖发布产物。
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

import argparse

import torch
import model.detector
import utils.utils

import onnx
from onnxsim import simplify

# 1. 导出模型（使用 opset 18 避免版本问题）
def export_onnx_model(model, dummy_input, output_path="model.onnx"):
    torch.onnx.export(
        model,
        dummy_input,
        output_path,
        opset_version=18,  # 使用 18 避免 Resize 降级问题
        do_constant_folding=True,  # 常量折叠
        input_names=['input'],
        output_names=['output'],
        dynamic_axes={'input': {0: 'batch_size'}},  # 动态 batch
        verbose=False
    )
    print(f"✓ Model exported to {output_path}")

# 2. 简化模型
def simplify_onnx_model(input_path, output_path="simplified_model.onnx"):
    model = onnx.load(input_path)
    
    # 先验证原模型
    try:
        onnx.checker.check_model(model)
        print("✓ Original model is valid")
    except Exception as e:
        print(f"✗ Model validation failed: {e}")
        return
    
    # 简化
    model_simp, check = simplify(model)
    
    if check:
        onnx.save(model_simp, output_path)
        print(f"✓ Simplified model saved to {output_path}")
        
        # 显示大小对比
        import os
        orig_size = os.path.getsize(input_path) / 1024 / 1024
        simp_size = os.path.getsize(output_path) / 1024 / 1024
        print(f"  Original size: {orig_size:.2f} MB")
        print(f"  Simplified size: {simp_size:.2f} MB")
        print(f"  Reduced by: {(1 - simp_size/orig_size)*100:.1f}%")
    else:
        print("✗ Simplification failed, original model preserved")

# 3. 使用示例
# export_onnx_model(your_model, dummy_input)
# simplify_onnx_model("model.onnx", "simplified_model.onnx")

if __name__ == '__main__':
    #指定训练配置文件
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=str, default=os.path.join(TRAIN, 'data', 'voc.data'),
                        help='Specify training profile *.data')
    parser.add_argument('--weights', type=str, default=os.path.join(TRAIN, 'weights', 'voc-299-epoch-0.270855ap-model.pth'),
                        help='The path of the .pth model to be transformed')

    parser.add_argument('--output', type=str, default=os.path.join(HERE, 'yolo-fastestv2.onnx'),
                        help='The path where the onnx model is saved')

    opt = parser.parse_args()
    cfg = utils.utils.load_datafile(opt.data)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = model.detector.Detector(cfg["classes"], cfg["anchor_num"], True, True).to(device)
    model.load_state_dict(torch.load(opt.weights, map_location=device))
    #sets the module in eval node
    model.eval()

    test_data = torch.rand(1, 3, cfg["height"], cfg["width"]).to(device)
    torch.onnx.export(model,                    #model being run
                     test_data,                 # model input (or a tuple for multiple inputs)
                     opt.output,               # where to save the model (can be a file or file-like object)
                     export_params=True,        # store the trained parameter weights inside the model file
                     opset_version=18,          # the ONNX version to export the model to
                     do_constant_folding=True)  # whether to execute constant folding for optimization

    #export_onnx_model(your_model, dummy_input)
    simplify_onnx_model(os.path.join(HERE, "yolo-fastestv2.onnx"), os.path.join(HERE, "yolo-fastestv2-opt.onnx"))

    

