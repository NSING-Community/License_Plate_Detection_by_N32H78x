import onnxruntime
import cv2
import numpy as np
import os

# 工作目录固定到本文件所在的 training/：脚本内部（以及命令行传入的）相对路径都按它解析，
# 所以从任何目录启动结果都一样。见交付文档 2.2 节。
os.chdir(os.path.dirname(os.path.abspath(__file__)))

# =================【改成你的训练参数】================
IN_W = 320
IN_H = 320
CLS_NUM = 20
CONF_THRESH = 0.30
NMS_THRESH = 0.45
anchors = [
    [[11.03,24.54],[14.40,36.74],[20.08,52.82]],
    [[27.04,78.23],[144.02,96.56],[239.25,195.42]]
]
strides = [16,32]
# 相对本脚本所在的 training/（脚本开头已 chdir 过去），与 test_onnx.py 一致
ONNX_PATH = "../pytorch2onnx/yolo-fastestv2-opt.onnx"
IMG_PATH = "img/21.jpg"
# ======================================================

ANCHOR_NUM = 3

def decode(feat, sid):
    """fused 头的解码：feat 为 [1,H,W,85] 的 NHWC 张量，
    通道布局 [reg(4*ANCHOR_NUM) | obj(ANCHOR_NUM) | cls(CLS_NUM)]，
    reg 与 obj 已 sigmoid、cls 已 softmax（export_onnx=True 烘进图里的），
    所以这里直接取值，不再套激活。公式与 utils/utils.py::handel_preds 一致：
        cx = (gx + v*2 - 0.5) * stride
        wh = (v*2)^2 * anchor
    """
    s = strides[sid]
    grid_w = IN_W // s
    grid_h = IN_H // s
    f = feat[0]
    boxes = []
    for gy in range(grid_h):
        for gx in range(grid_w):
            for an in range(ANCHOR_NUM):
                reg = f[gy, gx, an * 4: an * 4 + 4]
                conf = float(f[gy, gx, 4 * ANCHOR_NUM + an])
                if conf < CONF_THRESH:
                    continue

                cx = (gx + reg[0] * 2 - 0.5) * s
                cy = (gy + reg[1] * 2 - 0.5) * s
                bw = (reg[2] * 2) ** 2 * anchors[sid][an][0]
                bh = (reg[3] * 2) ** 2 * anchors[sid][an][1]
                if bw <= 0 or bh <= 0:
                    continue
                x1 = cx - bw * 0.5
                y1 = cy - bh * 0.5
                x2 = cx + bw * 0.5
                y2 = cy + bh * 0.5

                #cls 已经是 softmax 的结果，直接乘 objectness
                cls_scores = f[gy, gx, 5 * ANCHOR_NUM:] * conf
                cls_id = int(np.argmax(cls_scores))
                score = float(cls_scores[cls_id])
                if score < CONF_THRESH:
                    continue
                boxes.append([x1,y1,x2,y2,score,cls_id])
    return boxes

def nms(boxes):
    if len(boxes)==0:
        return []
    boxes = sorted(boxes,key=lambda x:x[4],reverse=True)
    res=[]
    while len(boxes)>0:
        best=boxes.pop(0)
        res.append(best)
        keep=[]
        for b in boxes:
            x1a,y1a,x2a,y2a,_,_=best
            x1b,y1b,x2b,y2b,_,_=b
            xx1=max(x1a,x1b)
            yy1=max(y1a,y1b)
            xx2=min(x2a,x2b)
            yy2=min(y2a,y2b)
            inter=max(0,xx2-xx1)*max(0,yy2-yy1)
            area_a=(x2a-x1a)*(y2a-y1a)
            area_b=(x2b-x1b)*(y2b-y1b)
            iou=inter/(area_a+area_b-inter)
            if iou<NMS_THRESH:
                keep.append(b)
        boxes=keep
    return res

if not os.path.exists(ONNX_PATH):
    raise SystemExit(f"找不到 {ONNX_PATH}")

#图像预处理
src=cv2.imread(IMG_PATH)
h0,w0=src.shape[:2]
resized=cv2.resize(src,(IN_W,IN_H))
rgb=cv2.cvtColor(resized,cv2.COLOR_BGR2RGB)
input_img=rgb.astype(np.float32)/255.0
input_img=np.expand_dims(input_img.transpose(2,0,1),axis=0)

#onnx推理
sess=onnxruntime.InferenceSession(ONNX_PATH)
inp_name=sess.get_inputs()[0].name
out_names=[o.name for o in sess.get_outputs()]
if len(out_names) != 2 or inp_name != "x":
    raise SystemExit(f"这个 ONNX 不是 fused 版本：输入名 {inp_name}、输出 {out_names}；"
                     f"请用 pytorch2onnx/pytorch2onnx.py 重新导出并 onnxsim 简化")
outs=sess.run(out_names,{inp_name:input_img})
out0,out1=outs[0],outs[1]
print("onnx input = ",inp_name)
print("out0 shape = ",out0.shape)
print("out1 shape = ",out1.shape)

#后处理
boxes0=decode(out0,0)
boxes1=decode(out1,1)
all_boxes=boxes0+boxes1
final_boxes=nms(all_boxes)

#画框保存结果
scale_x=w0/IN_W
scale_y=h0/IN_H
for b in final_boxes:
    x1,y1,x2,y2,score,cid=b
    x1*=scale_x
    y1*=scale_y
    x2*=scale_x
    y2*=scale_y
    cv2.rectangle(src,(int(x1),int(y1)),(int(x2),int(y2)),(0,255,0),2)
    print(f"class:{cid},score:{score:.2f},box:[{x1:.1f},{y1:.1f},{x2:.1f},{y2:.1f}]")
cv2.imwrite("../pytorch2onnx/onnx_result.jpg",src)
print("检测图片保存为 pytorch2onnx/onnx_result.jpg")
