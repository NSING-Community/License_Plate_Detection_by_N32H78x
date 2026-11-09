# -*- coding: utf-8 -*-
"""tflite -> C 字节数组头文件。参数化版：一份脚本出三个模型。

产物直接写进固件头文件目录 `mcu_code/Printf2/inc/` —— 那就是 Keil 工程 #include
的那一份，不需要再手工拷贝。

**段名是三处必须一起改的第一处**（另两处在 M7_TCM.sct 和 uVision 的 Flash 算法范围）：

  `data_area`  ① 用的：落到分散加载文件的 XIP Flash 段，启动时被拷进 AXI SRAM
               （0x24000000）。AXI SRAM 只够放一个，所以 ① 保持原样。
  `model_ro`   ②③ 用的：**原地在 Flash 里读**，不拷贝 —— AXI SRAM 放不下第二个。

少写这个属性，模型会被放进默认段，链接时直接报空间不足；段名写错则可能悄悄
落到 RAM 里，那样链接过得去、跑起来却不对。

**对齐（ALIGN）是第四处必须一起看的东西，而且是量出来的，不是讲究**：
`model_ro` 那两份（②③，XIP 原地读）用 `aligned(32)`，`data_area`（①）保持
`aligned(16)`。

2026-10-16 板上的现象：② 落在 0x1504D110（Printf.map 里 `ER_MODEL_RO (Exec base:
0x1504d110, Size: 0x00069500)`，段里只有 ②），只对齐 16 字节；板子从这块 flash
读回来的 431,360 字节里**唯一不对的就是段里第一条完整的 32 字节对齐行**
（0x1504D120..0x13F），装的是**文件 +16 处的内容**，其余 431,328 字节逐字节相等。
镜像本身验证过是对的（`tflite/_dbg_image.py`：数组 = .tflite、Printf.axf /
Printf.bin 里 flash 那个地址那 431,360 字节也 = .tflite），读宽度也不是
（32/16/8 三种宽度 + 拷进 SDRAM 再读回，四个 CRC 全一样）。 32 字节对齐把"段起点
落在 cache line 中间"这个形状消掉。 详情见 `MDK-ARM/yolo_inference.cpp` 里
`yolo_cascade_init()` 顶上那段长的。

用法：
  python tflite/export_c_array.py --tflite tflite/onnx2tf_int8_crop/crop-int8.tflite \\
      --array g_model_crop --section model_ro --out mcu_code/Printf2/inc/model_data_crop.h

  不带 --tflite 时默认读 tflite/onnx2tf_int8_plate/plate-int8.tflite（① 那一份）、
  写 mcu_code/Printf2/inc/model_data.h（段 data_area）。 ②③ 必须显式给参数 ——
  默认值以前指向 tflite/yolo-plate-int8.tflite（2026-09 的旧权重，同字节数不同内容），
  照默认跑一次就会把 ① 换成那份。
"""

import os
import sys
import argparse
import hashlib

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
INC = os.path.join(ROOT, "mcu_code", "Printf2", "inc")


def tflite_to_c_array(tflite_path, header_path, array_name, section,
                      size_name=None):
    print("\n[MODEL] 导出C数组: %s" % os.path.basename(tflite_path))
    if not os.path.exists(tflite_path):
        print("[ERR] 文件不存在: %s" % tflite_path)
        return None

    os.makedirs(os.path.dirname(header_path) or ".", exist_ok = True)
    with open(tflite_path, "rb") as f:
        blob = f.read()
    md5 = hashlib.md5(blob).hexdigest().upper()
    guard = os.path.basename(header_path).upper().replace(".", "_").replace("-", "_")
    size_name = size_name or (array_name + "_size")

    # 注释一律 ASCII：这个文件要被 armclang 读，带中文就得赌工程的文件编码设置。
    #
    # 对齐分两档，写在同一个格式串里，免得以后有人"顺手统一"成 16：
    #   model_ro（②③，XIP 原地读）必须 32 字节对齐 —— 见模块头 ALIGN 那段
    #   data_area（①）保持 16：它由 __main 在 D-Cache 打开前拷进 AXI SRAM，
    #   现在落在 0x150DDDA0（32 对齐是链接顺序的巧合），不该去动它。
    align = 32 if section == "model_ro" else 16
    c = """// Auto-generated from %s
// bytes: %d   MD5: %s
// section '%s' - %s
// This file is generated; edit tflite/export_c_array.py, not this.

#ifndef %s
#define %s

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// %s
__attribute__((section("%s"), aligned(%d)))
const unsigned char %s[] = {
""" % (os.path.basename(tflite_path), len(blob), md5, section,
       "read in place out of XIP flash, aligned(32) - must not start mid cache line"
       if section == "model_ro" else "copied to AXI SRAM at startup",
       guard, guard, os.path.basename(tflite_path), section, align, array_name)

    for i in range(0, len(blob), 16):
        c += "    " + ", ".join("0x%02x" % b for b in blob[i:i + 16]) + ",\n"

    c += """};

const unsigned int %s = %d;

#ifdef __cplusplus
} // extern C
#endif

#endif // %s
""" % (size_name, len(blob), guard)

    with open(header_path, "w") as f:
        f.write(c)
    try:
        shown = os.path.relpath(header_path, ROOT)
    except ValueError:              # 输出在另一个盘符上时 relpath 会抛 ValueError
        shown = header_path
    print("[OK] %s   %d bytes   MD5 %s" % (shown, len(blob), md5))
    return header_path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tflite",
                    default = os.path.join(HERE, "onnx2tf_int8_plate", "plate-int8.tflite"),
                    help = "默认 ①（整图找牌）；换 ②③ 必须显式给 --tflite + --array + --out")
    ap.add_argument("--out", default = os.path.join(INC, "model_data.h"),
                    help = "输出头文件路径")
    ap.add_argument("--array", default = "g_model_data", help = "数组名")
    ap.add_argument("--size-name", default = "", help = "长度变量名（默认 <array>_size）")
    ap.add_argument("--section", default = "data_area",
                    help = "data_area=①拷进 AXI SRAM；model_ro=②③原地 XIP 读")
    args = ap.parse_args()

    if not os.path.exists(args.tflite):
        print("[ERR] 模型文件不存在: %s" % args.tflite)
        print("请先运行 tflite/onnx2tf_int8.py 生成 int8 模型")
        return 1
    tflite_to_c_array(os.path.abspath(args.tflite), os.path.abspath(args.out),
                      args.array, args.section, args.size_name or None)
    print("\n[OK] 下一步: Keil 重新编译（不需要手工拷贝文件）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
