https://github.com/dog-qiuqiu/Yolo-FastestV2

本项目（v1.0）是**三网串联**：① 整图找牌 → ② 裁剪找字 → ③ 逐字分类，共 561,584 参数。
- 交付权重：
    ① weights/voc-299-epoch-0.270855ap-model.pth       （整图车牌检测，AP 0.2709）
    ② weights/voc_crop_ft-119-epoch-0.589619ap-model.pth（裁剪字符检测，AP 0.5896）
    ③ weights/charhead_16_32_64_32_64.pt               （逐字分类）
- 这三个权重**怎么训出来的**（数据、配置、命令行）见 RELEASE_v1.0_2stage.md §3.1：
  ② 不是从头训，是 data_crop/voc_finetune.data 拿 ep90 续训 120 轮；
  ③ 是 train_char_head.py --epochs 12 扫出来的 16:32:64 那档。
- 完整复现命令、门限（① 0.25/0.45、② 0.10/0.30、cap 7）与实测数字见
  RELEASE_v1.0_2stage.md（PC 冻结口径）与 ..\mcu_code\RELEASE_v1.0_board.md（板端）。
- ⚠️ 脚本默认值不是 v1.0：full_pipeline_e2e.py / fw_emulate.py / two_stage_e2e.py 的
  --crop-weights（有的脚本叫 --weights）仍指向 voc_crop-90（ep90，AP 0.4770），
  two_stage_e2e.py 的 --charhead 指向小一号的 charhead_8_16_32_32_64.pt。
  不带参数跑出来是旧口径；复现 v1.0 必须像 RELEASE 里那样显式给
  --plate-weights / --crop-weights / --charhead。
- 2026-10-20 清理：中间 checkpoint、一次性调试脚本与旧日志已移到
  E:\0 AI\3 Yolo-FastestV2-main_\training\（NIGHT_REPORT 里引用的那些文件在那里，同名路径）。
  那是内部存档，不在本交付包内。
- 下面是单模型时代的最小示例，保留作训练/测试入口。

cd E:\0 AI\3 Yolo-FastestV2-main_out\training
activate Yolo-FastestV2
（训练侧脚本开头会自己切到脚本所在的 training/，所以下面几条在任何目录下执行都一样）
(Yolo-FastestV2) E:\0 AI\3 Yolo-FastestV2-main_out\training>python genanchors.py --traintxt ./data/train.txt
(Yolo-FastestV2) E:\0 AI\3 Yolo-FastestV2-main_out\training>python train.py --data data/voc.data
(Yolo-FastestV2) E:\0 AI\3 Yolo-FastestV2-main_out\training>python test.py --data data/voc.data --weights weights/voc-299-epoch-0.270855ap-model.pth --img img/21.jpg

导出模型（v1.0 的三网走 tflite/export_onnx.py + tflite/onnx2tf_int8.py，
  两个脚本都要求显式给参数，完整三条命令见 RELEASE_v1.0_2stage.md §10；
  旧的 pytorch2onnx/pytorch2onnx.py 只导 ①、默认指向 weights/voc-299，
  仅用于手工对照，不要拿它当 v1.0 的导出流程）
cd ..
python tflite/export_onnx.py --kind detector `
    --data training/data/voc.data `
    --weights training/weights/voc-299-epoch-0.270855ap-model.pth `
    --output tflite/onnx/plate.onnx
python tflite/onnx2tf_int8.py --onnx tflite/onnx/plate.onnx `
    --calib training/data/val --size 320x320 --out-dir tflite/onnx2tf_int8_plate



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

How to use
Dependent installation
PIP
pip3 install -r requirements.txt
Test
Picture test
python3 test.py --data data/coco.data --weights modelzoo/coco2017-0.241078ap-model.pth --img img/000139.jpg

How to train
Building data sets(The dataset is constructed in the same way as darknet yolo)
The format of the data set is the same as that of Darknet Yolo, Each image corresponds to a .txt label file. The label format is also based on Darknet Yolo's data set label format: "category cx cy wh", where category is the category subscript, cx, cy are the coordinates of the center point of the normalized label box, and w, h are the normalized label box The width and height, .txt label file content example as follows:

11 0.344192634561 0.611 0.416430594901 0.262
14 0.509915014164 0.51 0.974504249292 0.972
The image and its corresponding label file have the same name and are stored in the same directory. The data file structure is as follows:

.
├── train
│   ├── 000001.jpg
│   ├── 000001.txt
│   ├── 000002.jpg
│   ├── 000002.txt
│   ├── 000003.jpg
│   └── 000003.txt
└── val
    ├── 000043.jpg
    ├── 000043.txt
    ├── 000057.jpg
    ├── 000057.txt
    ├── 000070.jpg
    └── 000070.txt
Generate a dataset path .txt file, the example content is as follows：

train.txt

/home/qiuqiu/Desktop/dataset/train/000001.jpg
/home/qiuqiu/Desktop/dataset/train/000002.jpg
/home/qiuqiu/Desktop/dataset/train/000003.jpg
val.txt

/home/qiuqiu/Desktop/dataset/val/000070.jpg
/home/qiuqiu/Desktop/dataset/val/000043.jpg
/home/qiuqiu/Desktop/dataset/val/000057.jpg
Generate the .names category label file, the sample content is as follows:

category.names

person
bicycle
car
motorbike
...

The directory structure of the finally constructed training data set is as follows:

.
├── category.names        # .names category label file
├── train                 # train dataset
│   ├── 000001.jpg
│   ├── 000001.txt
│   ├── 000002.jpg
│   ├── 000002.txt
│   ├── 000003.jpg
│   └── 000003.txt
├── train.txt              # train dataset path .txt file
├── val                    # val dataset
│   ├── 000043.jpg
│   ├── 000043.txt
│   ├── 000057.jpg
│   ├── 000057.txt
│   ├── 000070.jpg
│   └── 000070.txt
└── val.txt                # val dataset path .txt file

Get anchor bias
Generate anchor based on current dataset
python3 genanchors.py --traintxt ./train.txt
The anchors6.txt file will be generated in the current directory,the sample content of the anchors6.txt is as follows:
12.64,19.39, 37.88,51.48, 55.71,138.31, 126.91,78.23, 131.57,214.55, 279.92,258.87  # anchor bias
0.636158                                                                             # iou

Build the training .data configuration file
Reference./data/coco.data
[name]
model_name=coco           # model name

[train-configure]
epochs=300                # train epichs
steps=150,250             # Declining learning rate steps
batch_size=64             # batch size
subdivisions=1            # Same as the subdivisions of the darknet cfg file
learning_rate=0.001       # learning rate

[model-configure]
pre_weights=None          # The path to load the model, if it is none, then restart the training
classes=80                # Number of detection categories
width=352                 # The width of the model input image
height=352                # The height of the model input image
anchor_num=3              # anchor num
anchors=12.64,19.39, 37.88,51.48, 55.71,138.31, 126.91,78.23, 131.57,214.55, 279.92,258.87 #anchor bias

[data-configure]
train=/media/qiuqiu/D/coco/train2017.txt   # train dataset path .txt file
val=/media/qiuqiu/D/coco/val2017.txt       # val dataset path .txt file 
names=./data/coco.names                    # .names category label file

Train
Perform training tasks
python3 train.py --data data/coco.data

Evaluation
Calculate map evaluation
python3 evaluation.py --data data/coco.data --weights modelzoo/coco2017-0.241078ap-model.pth

Reference
https://github.com/Tencent/ncnn
https://github.com/AlexeyAB/darknet
https://github.com/ultralytics/yolov5
https://github.com/eriklindernoren/PyTorch-YOLOv3