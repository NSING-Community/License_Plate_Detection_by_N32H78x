## 移植 MCU 重要注意点

1. **修改宏定义**：`YOLO_IN_W、YOLO_IN_H、YOLO_CLS_NUM、anchors`，必须和训练配置完全一致；
2. MCU 没有数学库`expf/sigmoid`
   - 交付的 int8 模型已把 sigmoid/softmax 内嵌在模型里，推理侧不需要 `expf`/`sigmoid`；
     若换用未内嵌激活的模型，才需要自己实现（查表法即可）；
3. 内存：`BBox dets[]`定义为全局 / 静态数组，不要在栈上开辟大数组；
4. 坐标映射：代码输出坐标是**320×320 输入图上的像素坐标**；如果原图尺寸和输入不同，需要增加一步坐标缩放映射回原图分辨率；
5. 量化部署：
   - 本工程交付的是 **int8 全整型 TFLite**：量化（scale=1/255、zero=-128）与 sigmoid/softmax
     都已在模型内完成，输入按 `dst[i] = (int8_t)(rgb_pixel - 128)` 填，输出用
     `float = (q + 128) / 255` 反量化

6. **模型文件（重要，已更新）**：
   - 固件里有**三个** int8 模型，串成一条级联（① 整图找牌 → ② 裁剪找字 → ③ 逐字分类）。
     **身份以 `Printf2/inc/model_data*.h` 的头两行为准** —— 那才是链接进镜像的那份：

     | 级 | `.tflite` | 字节 | MD5 | C 数组 / 段 | 常驻位置 |
     |---|---|---|---|---|---|
     | ① | `tflite/onnx2tf_int8_plate/plate-int8.tflite` | 431,360 | `82693BCE77C3D5A9099BF268B5D815F5` | `model_data.h` / `data_area` | XIP Flash，启动时拷到 AXI SRAM |
     | ② | `tflite/onnx2tf_int8_crop/crop-int8.tflite` | 431,360 | `C80144F384FA1642B32A780255B5C33C` | `model_data_crop.h` / `model_ro` | `RW_SDRAM5`（init 时拷贝） |
     | ③ | `tflite/onnx2tf_int8_charhead/charhead-int8.tflite` | 90,144 | `17ED7E55EAEE8A5E5C5FE59BB5BEA9FA` | `model_data_charhead.h` / `model_ro` | `RW_SDRAM4`（init 时拷贝） |

   - ⚠️ `tflite/yolo-plate-int8.tflite`（MD5 `5193E19A…`）与 `plate-int8.tflite`
     **同为 431,360 B 但不是同一份权重**。前者是 2026-10-12 应用笔记描述的那份，
     **不在当前固件里**；别按它的 MD5 去核对镜像。
   - int8 用**真实车牌训练图校准**（早期那份用随机噪声校准的 int8 产物会把
     obj/cls 抹平，已废弃并移出仓库）
   - 布局 A：85ch = reg12 | obj3 | cls70，双头 (1,20,20,85)+(1,10,10,85)，①② 同架构
   - 级联入口：`yolo_cascade_init()` / `yolo_cascade_run()`（`Printf2/inc/yolo_inference.h`）
   - v1.0 发布规格与实测数字见 `mcu_code/RELEASE_v1.0_board.md`；
     ① 单独移植的细节见 `Printf2/YOLO_INT8_INTEGRATION.txt`

7. **交付固件里没有串口调试按键**：`Printf2/src/main.c` 中 18 个调试键分支
   （`g y f s c t w e j v b l d a n p k i`，含 GPU 排查用的 `p`/`g`/`c`/`j` 等）
   **已从源码删除**，串口收到的字节只被读掉、丢弃，不触发推理，也不改变任何开关。
   需要按键做内部排查请用保留按键源码的内部调试版（不在本交付包内）；
   本交付树里没有可翻的开关（见 `RELEASE_v1.0_board.md` §3）。