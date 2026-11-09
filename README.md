# License Plate Detection & Recognition on N32H78x

基于 **Yolo-FastestV2** 的车牌检测 + 识别工程，在 **Natsemiconductor N32H787_HMI**（Cortex-M7 @600 MHz，外扩 SDRAM，OV5640 摄像头 + LCD）上端到端跑通。

三张 int8 网络串联：**① 整图找车牌 → ② 裁剪块找字符 → ③ 逐字分类**，全部跑在 MCU 上，单帧约 2.7~3.5 秒输出一行车牌号。

| 指标（514 张 val） | 数值 |
|---|---|
| 整串识别正确率 | **78.99 %** |
| 其中 7 字标准牌 | **83.7 %** |
| 逐字符准确率 | 93.17 % |
| 车牌检出率 | 96.50 % |
| 三网总参数量 | 561,584 |

> 与单模型固件（整串约 1 %）相比高两个数量级；代价是参数量从 242,365 涨到 561,584（×2.32）。

---

## 目录结构

```
.
├── training/           PyTorch 训练链路：数据配置、训练 / 评测 / 端到端脚本、三份权重
│   ├── weights/        ① voc-299 / ② voc_crop_ft-119 / ③ charhead_16_32_64
│   ├── data/           数据配置与划分列表（voc.data、voc.names、train.txt、val.txt）
│   └── *.md            发布说明、夜间实验报告
├── tflite/             ONNX 导出 → int8 量化 → C 数组生成
│   ├── export_onnx.py     导出 ① / ② / ③ 的 ONNX
│   ├── onnx2tf_int8.py    ONNX → int8 TFLite（按真实车牌图校准）
│   ├── export_c_array.py  TFLite → model_data*.h
│   └── onnx2tf_int8_*/    三个模型的量化产物（含最终 *-int8.tflite）
├── pytorch2onnx/       旧的单模型导出脚本，仅作手工对照
├── mcu_code/Printf2/   Keil MDK 工程 + 推理代码 + 厂商 SDK
│   ├── MDK-ARM/Printf.uvprojx   uVision 工程（target: N32H78x）
│   ├── src/                     业务代码，级联入口 yolo_cascade_init() / yolo_cascade_run()
│   └── inc/model_data*.h        三个 int8 模型的 C 数组
└── N32H78x_车牌识别_Yolo-FastestV2_部署应用笔记.pdf
```

**数据集不在本仓库内**：`training/data/train`（4459 张）与 `training/data/val`（521 张）共约 460 MB，
`training/data_crop` 可由 `make_plate_crops.py` 重新生成。如需数据请开 issue。

---

## 三网规格

| 级 | 作用 | 权重 | int8 模型 | 字节 | MD5 | 链接段 |
|---|---|---|---|---|---|---|
| ① | 整图找车牌 | `voc-299-epoch-0.270855ap-model.pth` | `tflite/onnx2tf_int8_plate/plate-int8.tflite` | 431,360 | `82693BCE77C3D5A9099BF268B5D815F5` | `data_area`（启动拷到 AXI SRAM） |
| ② | 裁剪块找字符 | `voc_crop_ft-119-epoch-0.589619ap-model.pth` | `tflite/onnx2tf_int8_crop/crop-int8.tflite` | 431,360 | `C80144F384FA1642B32A780255B5C33C` | `model_ro`（拷到 `RW_SDRAM5`） |
| ③ | 逐字分类 | `charhead_16_32_64_32_64.pt` | `tflite/onnx2tf_int8_charhead/charhead-int8.tflite` | 90,144 | `17ED7E55EAEE8A5E5C5FE59BB5BEA9FA` | `model_ro`（拷到 `RW_SDRAM4`） |

① 与 ② 结构相同、参数量相同（各 242,365），但**权重与输入分布不同，不可互换**。

推理链路：

```
相机帧 ──► ① 320×320 整图检测 ──► 取面积最大的车牌框
        ──► 按 margin=0.10 在原生分辨率上外扩裁剪
        ──► ② 320×320 字符检测 ──► 按分数取前 16、cap=7、按 x1 升序
        ──► 每个字符框按 pad=0.25 外扩抠图 ──► resize 32×64
        ──► ③ 70 类分类 ──► 按 x1 拼接 ──► GBK 输出
```

---

## 快速开始

### 训练

```powershell
cd training
python make_plate_crops.py                                   # data → data_crop（margin 0.10）
python genanchors.py --traintxt ./data_crop/train.txt --input_width 320 --input_height 320
python train.py --data data/voc.data                         # ① 300 ep，从头
python train.py --data data_crop/voc_finetune.data           # ② 120 ep，从 ep90 续训
python train_char_head.py --epochs 12 --sweep 16:32:64       # ③ 字符头
```

### 导出 → 量化 → C 数组

```powershell
python tflite/export_onnx.py --kind detector `
    --data training/data/voc.data `
    --weights training/weights/voc-299-epoch-0.270855ap-model.pth `
    --output tflite/onnx/plate.onnx
python tflite/onnx2tf_int8.py --onnx tflite/onnx/plate.onnx `
    --calib training/data/val --size 320x320 --out-dir tflite/onnx2tf_int8_plate
python tflite/export_c_array.py --tflite tflite/onnx2tf_int8_plate/plate-int8.tflite `
    --array g_model_data --section data_area --out mcu_code/Printf2/inc/model_data.h
```

②③ 同法，**三者的校准集各不相同**：① 用 `training/data/val`，② 用 `training/data_crop/val`，
③ 用 `training/cache/charpatch_val_32x64_pad25.npz`。C 数组的 `--section` 对 ②③ 是 `model_ro`。

### 编译

1. uVision 打开 `mcu_code/Printf2/MDK-ARM/Printf.uvprojx`，选 target **N32H78x**，Rebuild
2. 命令行等价：

   ```powershell
   UV4.exe -b Printf.uvprojx -t N32H78x -j0 -o build.log
   ```

3. 预期 Program Size：`Code=244502 RO-data=1297442 RW-data=384 ZI-data=6682904`，0 Error / 8 Warning

> 工程里 `Printf.uvprojx` 有 6 条**绝对路径**指向厂商 SDK
> `E:\1 Nations.N32H7xx_Library.1.2.0\middlewares\gpu\...`（含 `nsgpu.lib`，10,496,954 B，
> MD5 `AAABD6A8A210CFC218A22FE6C87AECE7`）。SDK 放在别处时必须逐条改，
> **不要**换成本仓库 `Printf2/middlewares/gpu/lib/nsgpu.lib`（那个只有 1,545,090 B，不是同一份）。

### 上板

- 板卡：N32H787_HMI，串口 USART3 / PD8，**115200 8N1**，终端按 **GBK** 解码才能显示省份汉字
- 烧录后串口输出：

  ```
  yolo start ...
  yolo init OK
  粤R888G8
  ```

  之后每 2.7~3.5 秒一行车牌；**没读出串的轮次一个字都不打**（板子安静不等于板子死了）。

---

## 关键约束

改这些之前请先读 `training/RELEASE_v1.0_2stage.md`，每一条都有实测代价：

| 约束 | 值 |
|---|---|
| 车牌裁剪外扩 | `margin = 0.10`，在**原生分辨率**上裁（裁紧掉 16.5 点） |
| 字符抠图外扩 | `pad = 0.25`，同样原生分辨率 |
| 拼串顺序 | 按 **x1 升序**（NMS 输出是分数序，直接拼是乱码） |
| 两级门限 | ① conf/NMS `0.25/0.45`，② `0.10/0.30`；成对修改 |
| 字符数上限 | `YOLO_CHAR_MAX = 7`（放宽到 8 掉 1.75 点） |
| 候选框上限 | `YOLO_CROP_MAXBOX = 64`，满了按分数留 top-64 |
| 通道序 | 板子喂 **RGB**，训练全程 **BGR**（已知缺陷，翻转前须离线对拍） |

---

## 文档索引

| 文档 | 内容 |
|---|---|
| `training/RELEASE_v1.0_2stage.md` | PC 侧冻结口径：权重、链路、门限、实测数字 |
| `training/NIGHT_REPORT_2026-10-15.md` | 实验记录与取舍依据 |
| `training/readme.txt` | 交付权重说明、v1.0 与脚本默认值的差异 |
| `mcu_code/readme.txt` | MCU 移植要点、三个模型的 MD5 与段 layout |
| `mcu_code/Printf2/YOLO_INT8_INTEGRATION.txt` | ① 单独移植的细节 |
| `N32H78x_车牌识别_Yolo-FastestV2_部署应用笔记.pdf` | 应用笔记 |

---

## 致谢与许可

网络结构来自 [Yolo-FastestV2](https://github.com/dog-qiuqiu/Yolo-FastestV2)（dog-qiuqiu），
训练数据格式沿用 Darknet-YOLO 约定。本项目以 BSD 3-Clause 许可发布，见 `LICENSE`。
