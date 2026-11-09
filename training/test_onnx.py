import sys
sys.dont_write_bytecode = True   # 不生成 __pycache__：model/、utils/ 保持干净
import onnxruntime as ort
import numpy as np
import torch
import cv2
import os
from model.detector import Detector
from utils.utils import load_datafile, handel_preds_fused, non_max_suppression

# 工作目录固定到本文件所在的 training/：脚本内部（以及命令行传入的）相对路径都按它解析，
# 所以从任何目录启动结果都一样。见交付文档 2.2 节。
os.chdir(os.path.dirname(os.path.abspath(__file__)))

#====================在这里修改路径====================
ONNX_PATH = "../pytorch2onnx/yolo-fastestv2.onnx"   # 由 pytorch2onnx/pytorch2onnx.py 导出（路径相对本脚本所在的 training/）
CFG_PATH  = "data/voc.data"
PTH_PATH  = "weights/voc-299-epoch-0.270855ap-model.pth"   # v1.0 的 ①
IMG_PATH  = "img/21.jpg"
#======================================================

class Validator:
    def __init__(self, onnx_path, cfg_path, pth_path):
        if not os.path.exists(onnx_path):
            raise SystemExit(f"找不到 {onnx_path}，请先运行 python pytorch2onnx/pytorch2onnx.py 导出")

        self.cfg = load_datafile(cfg_path)
        self.onnx_session = ort.InferenceSession(onnx_path)
        self.input_name = self.onnx_session.get_inputs()[0].name
        self.output_names = [o.name for o in self.onnx_session.get_outputs()]
        self.device = torch.device("cpu")

        #先把 ONNX 是不是 export_onnx=True 那张图验一下，否则后面的误差对比没有意义
        if len(self.output_names) != 2 or self.input_name != "x":
            raise SystemExit(
                f"这个 ONNX 不是上板用的 fused 版本：输入名 {self.input_name}、"
                f"输出 {self.output_names}。\n"
                f"它应该是 export_onnx=False 导出的六路裸张量那一版（输入名 images、"
                f"输出 output0/output1 加自动命名）。\n"
                f"请用 pytorch2onnx/pytorch2onnx.py（第 4 个参数为 True）重新导出。")

        #参考模型必须与 ONNX 同形：Detector 第 4 个参数 export_onnx=True
        self.torch_model = Detector(self.cfg["classes"], self.cfg["anchor_num"], True, True).to(self.device)
        self.torch_model.load_state_dict(torch.load(pth_path, map_location=self.device))
        self.torch_model.eval()

    def preprocess(self, img_path):
        img = cv2.imread(img_path)
        h_ori, w_ori = img.shape[:2]
        w_target, h_target = self.cfg["width"], self.cfg["height"]
        img_res = cv2.resize(img, (w_target, h_target))
        img_rgb = cv2.cvtColor(img_res, cv2.COLOR_BGR2RGB)
        img_np = img_rgb.astype(np.float32) / 255.0
        img_np = np.transpose(img_np, (2, 0, 1))
        img_np = np.expand_dims(img_np, axis=0)
        img_tensor = torch.from_numpy(img_np)
        return img_np, img_tensor, h_ori, w_ori

    def run_compare(self, img_path):
        img_np, img_tensor, h_ori, w_ori = self.preprocess(img_path)

        # Pytorch推理（两个 [1,H,W,85] 的 NHWC 张量）
        with torch.no_grad():
            out_torch = self.torch_model(img_tensor)

        # ONNX推理
        out_onnx = self.onnx_session.run(self.output_names, {self.input_name: img_np})

        # 计算最大误差
        print("================误差对比================")
        print(f"输入名 {self.input_name}；输出名 {self.output_names}")
        errs = []
        for i, (t, o) in enumerate(zip(out_torch, out_onnx)):
            t_np = t.cpu().numpy()
            err = float(np.max(np.abs(t_np - o))) if t_np.shape == o.shape else float("inf")
            errs.append(err)
            print(f"分支{i}（stride {self.cfg['height'] // t_np.shape[1]}）："
                  f"torch {t_np.shape} vs onnx {o.shape}，最大绝对误差：{err:.6f}")
        if max(errs) < 0.001:
            print("✅误差很小，ONNX导出精度正常！")
        else:
            print("⚠️误差偏大，存在精度漂移！")

        # =========后处理：fused 头用 handel_preds_fused=========
        pred = handel_preds_fused(out_onnx, self.cfg, self.device)
        boxes = non_max_suppression(pred, conf_thres=0.3)
        print(f"\nONNX检测到目标数量：{len(boxes[0])}")
        print("\n=========测试结束=========")

if __name__ == "__main__":
    val = Validator(ONNX_PATH, CFG_PATH, PTH_PATH)
    val.run_compare(IMG_PATH)
    input("\n按回车键关闭窗口......")
