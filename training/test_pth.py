import sys
sys.dont_write_bytecode = True   # 不生成 __pycache__：model/、utils/ 保持干净
import os
import cv2
import time
import torch
import model.detector
import utils.utils

# 工作目录固定到本文件所在的 training/：脚本内部（以及命令行传入的）相对路径都按它解析，
# 所以从任何目录启动结果都一样。见交付文档 2.2 节。
os.chdir(os.path.dirname(os.path.abspath(__file__)))

if __name__ == '__main__':
    #=====================在这里直接填写参数=====================
    data_path    = "data/voc.data"         # 替换成你的*.data配置文件
    weights_path = "weights/voc-299-epoch-0.270855ap-model.pth" #pth权重路径 (v1.0 的 ①)
    img_path     = "img/21.jpg"       #测试图片路径
    conf_thres   = 0.3
    iou_thres    = 0.4
    #===========================================================

    cfg = utils.utils.load_datafile(data_path)
    assert os.path.exists(weights_path), "请指定正确的模型路径"
    assert os.path.exists(img_path), "请指定正确的测试图像路径"

    #模型加载
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = model.detector.Detector(cfg["classes"], cfg["anchor_num"], True).to(device)
    model.load_state_dict(torch.load(weights_path, map_location=device))
    model.eval()
    
    #数据预处理
    ori_img = cv2.imread(img_path)
    res_img = cv2.resize(ori_img, (cfg["width"], cfg["height"]), interpolation = cv2.INTER_LINEAR) 
    img = res_img.reshape(1, cfg["height"], cfg["width"], 3)
    img = torch.from_numpy(img.transpose(0,3, 1, 2))
    img = img.to(device).float() / 255.0

    #模型推理
    start = time.perf_counter()
    preds = model(img)
    end = time.perf_counter()
    infer_time = (end - start) * 1000.
    print("forward time:%fms"%infer_time)

    #特征图后处理
    output = utils.utils.handel_preds(preds, cfg, device)
    output_boxes = utils.utils.non_max_suppression(output, conf_thres = conf_thres, iou_thres = iou_thres)

    #加载label names
    LABEL_NAMES = []
    with open(cfg["names"], 'r') as f:
        for line in f.readlines():
            LABEL_NAMES.append(line.strip())
    
    h, w, _ = ori_img.shape
    scale_h, scale_w = h / cfg["height"], w / cfg["width"]

    #绘制预测框
    for box in output_boxes[0]:
        box = box.tolist()
        
        obj_score = box[4]
        category = LABEL_NAMES[int(box[5])]
        x1, y1 = int(box[0] * scale_w), int(box[1] * scale_h)
        x2, y2 = int(box[2] * scale_w), int(box[3] * scale_h)
        cv2.rectangle(ori_img, (x1, y1), (x2, y2), (255, 255, 0), 2)
        cv2.putText(ori_img, '%.2f' % obj_score, (x1, y1 - 5), 0, 0.7, (0, 255, 0), 2)  
        cv2.putText(ori_img, category, (x1, y1 - 25), 0, 0.7, (0, 255, 0), 2)
    cv2.imwrite("test_result.png", ori_img)
    cv2.imshow("test_result.png", ori_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
