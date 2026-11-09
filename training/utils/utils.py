import cv2
import time

import torch
import torch.nn.functional as F

import os, time
import numpy as np
from tqdm import tqdm

#加载data
def load_datafile(data_path):
    #需要配置的超参数
    cfg = {"model_name":None,
    
           "epochs": None,
           "steps": None,           
           "batch_size": None,
           "subdivisions":None,
           "learning_rate": None,

           "pre_weights": None,        
           "classes": None,
           "width": None,
           "height": None,           
           "anchor_num": None,
           "anchors": None,

           "val": None,           
           "train": None,
           "names":None
        }

    assert os.path.exists(data_path), "请指定正确配置.data文件路径"

    #指定配置项的类型
    list_type_key = ["anchors", "steps"]
    str_type_key = ["model_name", "val", "train", "names", "pre_weights"]
    int_type_key = ["epochs", "batch_size", "classes", "width",
                   "height", "anchor_num", "subdivisions"]
    float_type_key = ["learning_rate"]
    
    #加载配置文件
    with open(data_path, 'r') as f:
        for line in f.readlines():
            if line == '\n' or line[0] == "[":
                continue
            else:
                data = line.strip().split("=")
                #配置项类型转换
                if data[0] in cfg:
                    if data[0] in int_type_key:
                       cfg[data[0]] = int(data[1])
                    elif data[0] in str_type_key:
                        cfg[data[0]] = data[1]
                    elif data[0] in float_type_key:
                        cfg[data[0]] = float(data[1])
                    elif data[0] in list_type_key:
                        cfg[data[0]] = [float(x) for x in data[1].split(",")]
                    else:
                        print("配置文件有错误的配置项")
                else:
                    print("%s配置文件里有无效配置项:%s"%(data_path, data))
    return cfg

def load_names(names_path):
    """读 .names 类别名表，返回下标即类别号的列表。
    本仓的 data/voc.names 是 GBK（省份简称），所以先按 UTF-8 试、失败再退 GBK；
    两条都不行就当没有名字，只打类别号，不让它挡住评估。"""
    if names_path is None or not os.path.exists(names_path):
        return None
    for enc in ("utf-8", "gbk"):
        try:
            with open(names_path, 'r', encoding = enc) as f:
                return [x.strip() for x in f.readlines() if x.strip() != ""]
        except (UnicodeDecodeError, LookupError):
            continue
    return None

def class_weights(list_path, classes, power = 0.5, wmax = 4.0):
    """按训练集里每个类别的目标框个数算分类损失权重（只上抬稀有类，不压低常见类）。

    动机：出厂权重 70 类里有 18 类（16 个稀有省份 + 字母 W、T）在验证集上 AP 恰好 0.00000
    —— 模型根本不发这些字符。它们在训练集里分别只有 5~160 个框，而 8/9 两类各 2000 个，
    差了 400 倍；nn.CrossEntropyLoss() 无权重的默认行为就是直接把长尾忽略掉。

    权重 = clamp((中位数 / n_c) ** power, 1.0, wmax)，也就是**只放大稀有类**：
    常见类（含 plate 类自己，n=4461）权重恒为 1.0，分类损失的整体尺度不变。
    下界取 1.0 是刻意的 —— plate 类现在 AP 0.68，是这条链上唯一真能用的东西，
    任何"按频率均衡"的方案都会把它压到 1.0 以下，所以不做。

    中位数只按字符类（1..classes-1）算：plate 类整图一个框，混进去会把基准拉偏。

    返回 (weights, counts)，两个 list，长度都是 classes。
    清单或标注缺失时返回 (None, None)，调用方退回无权重的 CE。
    """
    if list_path is None or not os.path.exists(list_path):
        return None, None

    counts = [0] * classes
    with open(list_path, 'r') as f:
        for line in f.readlines():
            line = line.strip()
            if line == "":
                continue
            label_path = os.path.splitext(line)[0] + ".txt"
            if not os.path.exists(label_path):
                continue
            with open(label_path, 'r') as lf:
                for item in lf.read().splitlines():
                    field = item.split()
                    if len(field) >= 5:
                        c = int(float(field[0]))
                        if 0 <= c < classes:
                            counts[c] += 1

    char_counts = sorted(counts[1:])
    if not char_counts or char_counts[-1] == 0:
        return None, None
    mid = len(char_counts) // 2
    median = char_counts[mid] if len(char_counts) % 2 else (char_counts[mid - 1] + char_counts[mid]) / 2.0

    weights = []
    for n in counts:
        if n <= 0:
            weights.append(0.0)  # 训练集里根本不出现，永远不是目标；权重 0 与 1 等价
        else:
            weights.append(min(wmax, max(1.0, (median / float(n)) ** power)))
    return weights, counts

def xywh2xyxy(x):
    # Convert nx4 boxes from [x, y, w, h] to [x1, y1, x2, y2] where xy1=top-left, xy2=bottom-right
    y = torch.zeros_like(x) if isinstance(x, torch.Tensor) else np.zeros_like(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
    y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
    y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
    y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
    return y

def bbox_iou(box1, box2, x1y1x2y2=True):
    """
    Returns the IoU of two bounding boxes
    """
    if not x1y1x2y2:
        # Transform from center and width to exact coordinates
        b1_x1, b1_x2 = box1[:, 0] - box1[:, 2] / 2, box1[:, 0] + box1[:, 2] / 2
        b1_y1, b1_y2 = box1[:, 1] - box1[:, 3] / 2, box1[:, 1] + box1[:, 3] / 2
        b2_x1, b2_x2 = box2[:, 0] - box2[:, 2] / 2, box2[:, 0] + box2[:, 2] / 2
        b2_y1, b2_y2 = box2[:, 1] - box2[:, 3] / 2, box2[:, 1] + box2[:, 3] / 2
    else:
        # Get the coordinates of bounding boxes
        b1_x1, b1_y1, b1_x2, b1_y2 = \
            box1[:, 0], box1[:, 1], box1[:, 2], box1[:, 3]
        b2_x1, b2_y1, b2_x2, b2_y2 = \
            box2[:, 0], box2[:, 1], box2[:, 2], box2[:, 3]

    # get the corrdinates of the intersection rectangle
    inter_rect_x1 = torch.max(b1_x1, b2_x1)
    inter_rect_y1 = torch.max(b1_y1, b2_y1)
    inter_rect_x2 = torch.min(b1_x2, b2_x2)
    inter_rect_y2 = torch.min(b1_y2, b2_y2)
    # Intersection area
    inter_area = torch.clamp(inter_rect_x2 - inter_rect_x1 + 1, min=0) * torch.clamp(
        inter_rect_y2 - inter_rect_y1 + 1, min=0
    )
    # Union Area
    b1_area = (b1_x2 - b1_x1 + 1) * (b1_y2 - b1_y1 + 1)
    b2_area = (b2_x2 - b2_x1 + 1) * (b2_y2 - b2_y1 + 1)

    iou = inter_area / (b1_area + b2_area - inter_area + 1e-16)

    return iou

def compute_ap(recall, precision):
    """ Compute the average precision, given the recall and precision curves.
    Code originally from https://github.com/rbgirshick/py-faster-rcnn.

    # Arguments
        recall:    The recall curve (list).
        precision: The precision curve (list).
    # Returns
        The average precision as computed in py-faster-rcnn.
    """
    # correct AP calculation
    # first append sentinel values at the end
    mrec = np.concatenate(([0.0], recall, [1.0]))
    mpre = np.concatenate(([0.0], precision, [0.0]))

    # compute the precision envelope
    for i in range(mpre.size - 1, 0, -1):
        mpre[i - 1] = np.maximum(mpre[i - 1], mpre[i])

    # to calculate area under PR curve, look for points
    # where X axis (recall) changes value
    i = np.where(mrec[1:] != mrec[:-1])[0]

    # and sum (\Delta recall) * prec
    ap = np.sum((mrec[i + 1] - mrec[i]) * mpre[i + 1])
    return ap

def ap_per_class(tp, conf, pred_cls, target_cls, verbose = False, names = None):
    """ Compute the average precision, given the recall and precision curves.
    Source: https://github.com/rafaelpadilla/Object-Detection-Metrics.
    # Arguments
        tp:    True positives (list).
        conf:  Objectness value from 0-1 (list).
        pred_cls: Predicted object classes (list).
        target_cls: True object classes (list).
        verbose: 为 True 时按类打印逐类明细（AP 升序，一眼看出是长尾哪几类在拖后腿）。
        names:   类别名列表，verbose 时为 None 就只打类别号。
    # Returns
        The average precision as computed in py-faster-rcnn.
    """

    # Sort by objectness
    i = np.argsort(-conf)
    tp, conf, pred_cls = tp[i], conf[i], pred_cls[i]

    # Find unique classes
    unique_classes = np.unique(target_cls)

    # Create Precision-Recall curve and compute AP for each class
    ap, p, r = [], [], []
    for c in unique_classes:
        i = pred_cls == c
        n_gt = (target_cls == c).sum()  # Number of ground truth objects
        n_p = i.sum()  # Number of predicted objects

        if n_p == 0 and n_gt == 0:
            continue
        elif n_p == 0 or n_gt == 0:
            ap.append(0)
            r.append(0)
            p.append(0)
        else:
            # Accumulate FPs and TPs
            fpc = (1 - tp[i]).cumsum()
            tpc = (tp[i]).cumsum()

            # Recall
            recall_curve = tpc / (n_gt + 1e-16)
            r.append(recall_curve[-1])

            # Precision
            precision_curve = tpc / (tpc + fpc)
            p.append(precision_curve[-1])

            # AP from recall-precision curve
            ap.append(compute_ap(recall_curve, precision_curve))

    # Compute F1 score (harmonic mean of precision and recall)
    p, r, ap = np.array(p), np.array(r), np.array(ap)
    f1 = 2 * p * r / (p + r + 1e-16)

    if verbose:
        # 逐类明细。注意三点（和上面平均值同源，别读错）：
        #   1. 这里的 AP 是 IoU 0.5 下 PR 曲线包的面积，和返回的 AP 完全一致；
        #   2. 只统计验证集里真的出现过的类（unique_classes 来自 target_cls），
        #      所以「平均 AP」是这些类的算术平均，不是 70 类的平均；
        #   3. 这里的 P/R 是 PR 曲线**末端**那一个点，不是打印行里的
        #      Precision/Recall —— 那两个是另一次 conf_thres=0.3 的评估，别混。
        print("---- 逐类明细（AP 升序；类别数 %d，即验证集里出现过的类）----" % len(unique_classes))
        print("  %-4s %-6s %6s %9s %9s %9s %9s" % ("id", "name", "n_gt", "P", "R", "AP", "F1"))
        for j in np.argsort(ap):
            c = int(unique_classes[j])
            n_gt = int((target_cls == unique_classes[j]).sum())
            nm = names[c] if (names is not None and c < len(names)) else str(c)
            print("  %-4d %-6s %6d %9.5f %9.5f %9.5f %9.5f" % (c, nm, n_gt, p[j], r[j], ap[j], f1[j]))
        print("  平均（%d 类）            %9.5f %9.5f %9.5f %9.5f"
              % (len(unique_classes), np.mean(p), np.mean(r), np.mean(ap), np.mean(f1)))

    return np.mean(p), np.mean(r), np.mean(ap), np.mean(f1)

def get_batch_statistics(outputs, targets, iou_threshold, device):
    """ Compute true positives, predicted scores and predicted labels per sample """
    batch_metrics = []
    for sample_i in range(len(outputs)):

        if outputs[sample_i] is None:
            continue

        output = outputs[sample_i]
        pred_boxes = output[:, :4]
        pred_scores = output[:, 4]
        pred_labels = output[:, -1]

        true_positives = np.zeros(pred_boxes.shape[0])

        annotations = targets[targets[:, 0] == sample_i][:, 1:]
        target_labels = annotations[:, 0] if len(annotations) else []
        if len(annotations):
            detected_boxes = []
            target_boxes = annotations[:, 1:]

            for pred_i, (pred_box, pred_label) in enumerate(zip(pred_boxes, pred_labels)):
                
                pred_box = pred_box.to(device)
                pred_label = pred_label.to(device)

                # If targets are found break
                if len(detected_boxes) == len(annotations):
                    break

                # Ignore if label is not one of the target labels
                if pred_label.to(device) not in target_labels:
                    continue

                iou, box_index = bbox_iou(pred_box.unsqueeze(0), target_boxes).max(0)
                if iou >= iou_threshold and box_index not in detected_boxes:
                    true_positives[pred_i] = 1
                    detected_boxes += [box_index]
        batch_metrics.append([true_positives, pred_scores, pred_labels])
    return batch_metrics

def non_max_suppression(prediction, conf_thres=0.3, iou_thres=0.45, classes=None):
    """Performs Non-Maximum Suppression (NMS) on inference results
    Returns:
         detections with shape: nx6 (x1, y1, x2, y2, conf, cls)
    """

    nc = prediction.shape[2] - 5  # number of classes

    # Settings
    # (pixels) minimum and maximum box width and height
    max_wh = 4096
    max_det = 300  # maximum number of detections per image
    max_nms = 30000  # maximum number of boxes into torchvision.ops.nms()
    time_limit = 1.0  # seconds to quit after
    multi_label = nc > 1  # multiple labels per box (adds 0.5ms/img)

    # torchvision 只在这一个地方用到，所以保持函数内延迟导入：
    # 只跑导出/量化脚本（pytorch2onnx/pytorch2onnx.py、pytorch2onnx/pytorch2onnx_simplify.py）的本机
    # 不需要装 torchvision，走 NMS 的 test.py / test_onnx.py 才需要。
    #
    # ★ 必须在 t 之前（即计时开始之前）导入。原先这句在下面的循环体里，全新进程的
    #   第一次 NMS 会为这次 import 花掉约 1.2 s，直接撞上 time_limit 的 wall-clock
    #   break，把那一批 128 张里剩下的 127 张判成「零检测」——AP 系统性偏低约
    #   0.05~0.075，而且每个新进程固定丢第 0 批，所以"复测两次逐位相同"也骗人。
    import torchvision

    t = time.time()
    output = [torch.zeros((0, 6), device="cpu")] * prediction.shape[0]

    for xi, x in enumerate(prediction):  # image index, image inference
        # Apply constraints
        # x[((x[..., 2:4] < min_wh) | (x[..., 2:4] > max_wh)).any(1), 4] = 0  # width-height
        x = x[x[..., 4] > conf_thres]  # confidence

        # If none remain process next image
        if not x.shape[0]:
            continue

        # Compute conf
        x[:, 5:] *= x[:, 4:5]  # conf = obj_conf * cls_conf

        # Box (center x, center y, width, height) to (x1, y1, x2, y2)
        box = xywh2xyxy(x[:, :4])

        # Detections matrix nx6 (xyxy, conf, cls)
        conf, j = x[:, 5:].max(1, keepdim=True)
        x = torch.cat((box, conf, j.float()), 1)[conf.view(-1) > conf_thres]

        # Filter by class
        if classes is not None:
            x = x[(x[:, 5:6] == torch.tensor(classes, device=x.device)).any(1)]

        # Check shape
        n = x.shape[0]  # number of boxes
        if not n:  # no boxes
            continue
        elif n > max_nms:  # excess boxes
            # sort by confidence
            x = x[x[:, 4].argsort(descending=True)[:max_nms]]

        # Batched NMS
        c = x[:, 5:6] * max_wh  # classes
        # boxes (offset by class), scores
        boxes, scores = x[:, :4] + c, x[:, 4]
        i = torchvision.ops.nms(boxes, scores, iou_thres)  # NMS
        if i.shape[0] > max_det:  # limit detections
            i = i[:max_det]

        output[xi] = x[i].detach().cpu()

        if (time.time() - t) > time_limit:
            print(f'WARNING: NMS time limit {time_limit}s exceeded')
            break  # time limit exceeded

    return output

def make_grid(h, w, cfg, device):
    hv, wv = torch.meshgrid([torch.arange(h), torch.arange(w)])
    return torch.stack((wv, hv), 2).repeat(1,1,3).reshape(h, w, cfg["anchor_num"], -1).to(device)

#特征图后处理
def handel_preds(preds, cfg, device):
    #加载anchor配置
    anchors = np.array(cfg["anchors"])
    anchors = torch.from_numpy(anchors.reshape(len(preds) // 3, cfg["anchor_num"], 2)).to(device)

    output_bboxes = []
    layer_index = [0, 0, 0, 1, 1, 1]

    for i in range(len(preds) // 3):
        bacth_bboxes = []
        reg_preds = preds[i * 3]
        obj_preds = preds[(i * 3) + 1]
        cls_preds = preds[(i * 3) + 2]

        for r, o, c in zip(reg_preds, obj_preds, cls_preds):
            r = r.permute(1, 2, 0)
            r = r.reshape(r.shape[0], r.shape[1], cfg["anchor_num"], -1)

            o = o.permute(1, 2, 0)
            o = o.reshape(o.shape[0], o.shape[1], cfg["anchor_num"], -1)

            c = c.permute(1, 2, 0)
            c = c.reshape(c.shape[0],c.shape[1], 1, c.shape[2])
            c = c.repeat(1, 1, 3, 1)

            anchor_boxes = torch.zeros(r.shape[0], r.shape[1], r.shape[2], r.shape[3] + c.shape[3] + 1)

            #计算anchor box的cx, cy
            grid = make_grid(r.shape[0], r.shape[1], cfg, device)
            stride = cfg["height"] /  r.shape[0]
            anchor_boxes[:, :, :, :2] = ((r[:, :, :, :2].sigmoid() * 2. - 0.5) + grid) * stride

            #计算anchor box的w, h
            anchors_cfg = anchors[i]
            anchor_boxes[:, :, :, 2:4] = (r[:, :, :, 2:4].sigmoid() * 2) ** 2 * anchors_cfg # wh

            #计算obj分数
            anchor_boxes[:, :, :, 4] = o[:, :, :, 0].sigmoid()

            #计算cls分数
            anchor_boxes[:, :, :, 5:] = F.softmax(c[:, :, :, :], dim = 3)

            #torch tensor 转为 numpy array
            anchor_boxes = anchor_boxes.cpu().detach().numpy() 
            bacth_bboxes.append(anchor_boxes)     

        #n, anchor num, h, w, box => n, (anchor num*h*w), box
        bacth_bboxes = torch.from_numpy(np.array(bacth_bboxes))
        bacth_bboxes = bacth_bboxes.view(bacth_bboxes.shape[0], -1, bacth_bboxes.shape[-1]) 

        output_bboxes.append(bacth_bboxes)    
        
    #merge
    output = torch.cat(output_bboxes, 1)
            
    return output

#export_onnx=True 那张图（fused 检测头）的后处理
def handel_preds_fused(preds, cfg, device):
    """与 handel_preds 等价，区别只在输入形态。

    preds：forward 在 export_onnx=True 下返回的两个（或列表形式的）NHWC 张量，
           形状 [1, H, W, 85]，通道布局 [reg(4*anchor_num) | obj(anchor_num) | cls(classes)]。
           这一版把 sigmoid(reg)、sigmoid(obj)、softmax(cls) 烘进了图里，
           所以下面直接用，不再套一次激活 —— 其余解码公式与 handel_preds 逐行一致。
    输出：与 handel_preds 相同的 [n, N, 85] 张量，cx/cy 已换算到像素坐标。
    """
    anchors = np.array(cfg["anchors"])
    anchors = torch.from_numpy(anchors.reshape(len(preds), cfg["anchor_num"], 2)).to(device)
    anchor_num = cfg["anchor_num"]

    output_bboxes = []

    for i, pred in enumerate(preds):
        if not torch.is_tensor(pred):
            pred = torch.from_numpy(np.asarray(pred))
        pred = pred.to(device).float()

        bacth_bboxes = []
        for f in pred:
            h, w = f.shape[0], f.shape[1]

            #85 通道 = 三个头在通道维上拼接，不是每个 anchor 一组，所以先按通道切分
            reg = f[:, :, :4 * anchor_num].reshape(h, w, anchor_num, 4)
            obj = f[:, :, 4 * anchor_num:5 * anchor_num].reshape(h, w, anchor_num, 1)
            cls = f[:, :, 5 * anchor_num:].reshape(h, w, 1, -1).repeat(1, 1, anchor_num, 1)

            anchor_boxes = torch.zeros(h, w, anchor_num, 4 + 1 + cls.shape[3])

            #计算anchor box的cx, cy
            grid = make_grid(h, w, cfg, device)
            stride = cfg["height"] / h
            anchor_boxes[:, :, :, :2] = ((reg[:, :, :, :2] * 2. - 0.5) + grid) * stride

            #计算anchor box的w, h
            anchors_cfg = anchors[i]
            anchor_boxes[:, :, :, 2:4] = (reg[:, :, :, 2:4] * 2) ** 2 * anchors_cfg # wh

            #计算obj分数
            anchor_boxes[:, :, :, 4] = obj[:, :, :, 0]

            #计算cls分数（图里已是 softmax 的结果）
            anchor_boxes[:, :, :, 5:] = cls

            #torch tensor 转为 numpy array
            anchor_boxes = anchor_boxes.cpu().detach().numpy()
            bacth_bboxes.append(anchor_boxes)

        #n, anchor num, h, w, box => n, (anchor num*h*w), box
        bacth_bboxes = torch.from_numpy(np.array(bacth_bboxes))
        bacth_bboxes = bacth_bboxes.view(bacth_bboxes.shape[0], -1, bacth_bboxes.shape[-1])

        output_bboxes.append(bacth_bboxes)

    #merge
    output = torch.cat(output_bboxes, 1)

    return output

#模型评估
def evaluation(val_dataloader, cfg, model, device, conf_thres = 0.01, nms_thresh = 0.4, iou_thres = 0.5, verbose = False):
    labels = []
    sample_metrics = []  # List of tuples (TP, confs, pred)
    pbar = tqdm(val_dataloader)

    for imgs, targets in pbar:
        imgs = imgs.to(device).float() / 255.0
        targets = targets.to(device)       

        # Extract labels
        labels += targets[:, 1].tolist()
        # Rescale target
        targets[:, 2:] = xywh2xyxy(targets[:, 2:])
        targets[:, 2:] *= torch.tensor([cfg["width"], cfg["height"], cfg["width"], cfg["height"]]).to(device)

        #对预测的anchorbox进行nms处理
        with torch.no_grad():
            preds = model(imgs)

            #特征图后处理:生成anchorbox
            output = handel_preds(preds, cfg, device)
            output_boxes = non_max_suppression(output, conf_thres = conf_thres, iou_thres = nms_thresh)

        sample_metrics += get_batch_statistics(output_boxes, targets, iou_thres, device)
        pbar.set_description("Evaluation model:") 

    if len(sample_metrics) == 0:  # No detections over whole validation set.
        print("---- No detections over whole validation set ----")
        return None

    # Concatenate sample statistics
    true_positives, pred_scores, pred_labels = [np.concatenate(x, 0) for x in list(zip(*sample_metrics))]
    metrics_output = ap_per_class(true_positives, pred_scores, pred_labels, labels,
                                  verbose = verbose, names = load_names(cfg["names"]) if verbose else None)

    return metrics_output