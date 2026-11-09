#ifndef YOLO_INFERENCE_H
#define YOLO_INFERENCE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------- yolo-plate (Yolo-FastestV2) 70-class config ----------------
#define YOLO_IN_W       320
#define YOLO_IN_H       320
#define YOLO_CLS_NUM    70
#define YOLO_OBJ_NUM    3
#define YOLO_REG_NUM    12          /* 3 anchors * (cx,cy,bw,bh) */
#define YOLO_CHAN       85          /* 12 reg + 3 obj + 70 cls  (layout A)  */
#define YOLO_CONF_THRESH 0.25f
#define YOLO_NMS_THRESH  0.45f

// 两个输出头分辨率: 320/16=20, 320/32=10
#define YOLO_GRID0      20
#define YOLO_GRID1      10

typedef struct {
    float x1, y1, x2, y2;
    float score;
    int   class_id;
} YoloBox;

/**
 * @brief  TFLM int8 车牌模型推理 + 解码 + NMS
 * @param  rgb  : 320x320 RGB888 图像 (每像素3字节, 0~255), 内部做 (v-128) 量化
 * @param  boxes: 输出检测框缓冲
 * @param  max_box: 缓冲容量
 * @retval >=0 有效框数; <0 错误
 */
int yolo_plate_detect(const uint8_t* rgb, YoloBox* boxes, int max_box);

/** 初始化 TFLM 解释器。返回 true 成功 */
bool yolo_plate_init(void);

/* ---------------- 精确算子计时探针 (只测量, 不改结果) ----------------
   按算子类型统计"不做算术"的那些算子一次推理的调用次数与耗时。 类别顺序
   固定为 MAX_POOL_2D / PAD / CONCATENATION / TRANSPOSE / RESHAPE / GATHER /
   RESIZE_NEAREST_NEIGHBOR / LOGISTIC / SOFTMAX / QUANTIZE, 与
   yolo_inference.cpp 里 YOLO_ADD_OP_TIME 的登记顺序一致。

   用途只有一个: 量出 tools/_exact_ops.py 普查出的 133 个"不做算术"算子
   (本项目里唯一按构造不可能改输出的卸载类别) 在 CPU 上到底花掉 infer 的
   多少 —— 这是"值不值得卸载"缺的那个分母。

   关掉时与探测前逐字节相同 (包装函数只多一条可预测分支)。 复位即关, 由
   main.c 的 'e' 键切换。 见 yolo_inference.cpp 的同名注释块。 */
#define YOLO_EX_CLASSES 10
void yolo_ex_timing_set(int on);
int  yolo_ex_timing_get(void);
/** n/us 都是 YOLO_EX_CLASSES 长的数组; us 单位是微秒, 已按 600 MHz 换算,
    所以各项之和可以直接对着 main.c 打的 `infer %u us` 读。 */
void yolo_ex_stat(uint32_t* n, uint32_t* us);

/* ---------------- TRANSPOSE [0,2,3,1] 上 GPU (只测, 不改结果) ----------------
   52 个 TRANSPOSE 里有 26 个 perm 是 [0,2,3,1], 在 4 维张量上就是一次普通
   二维转置 —— 一次转置读, 不需要中间平面。 置换不含算术, 所以本项目里唯独这
   一类卸载**没有重量化可丢**: 要么逐字节对, 要么就是错的, 而 `bad 0` 就是全部
   证明。 这是它值得单独一轮的原因。

   底层与卷积不同: GPU 的 CLUT 卷积通路的被乘数只有**5 位** (0x40 读成 8,
   0xFF 读成 31, 实测), 所以卷积画不出 int8。 转置不做乘法 —— 它把 CLUT 当普通
   的 256 项恒等调色板用 (E_DHD_CLUT_INDEXED_COLOR), 进来的整个字节原样到目的
   地。 见 gpu.c 里 gpu_transpose_s8() 上面那段。

   这一轮的行为, 不许含糊: **CPU 核先跑, 它的结果就是产品用的结果**; GPU 的答案
   只进本文件自己的缓冲、逐字节比完就丢。 未验证的算子不许顶替产品结果, 而它要
   先挣到的那条读数就是 `bad 0`。 每次推理只做一层, 所以按五、六次 'x' 就是在五、
   六帧真实激活上采样同一个算子。

   'j' 键切换, 复位即关; 关着时与加这段之前逐字节相同。 */
void yolo_tr_arm(void);
void yolo_tr_set(int on);
int  yolo_tr_get(void);
void yolo_tr_stat(uint32_t* n, uint32_t* bad, uint32_t* worst, uint32_t* us);

/* ---------------- 车牌框的几何分组 (改产品答案, 所以默认关) ----------------
   yolo_plate_read() 原本把**每个** class_id>0 的框都拼进字符串, 于是画面任何
   位置的一个幻觉框都变成一个车牌字符。 2026-10-12 用 'b' 抓的同一场景 11 帧
   里, 一块牌拼出了 **8 种不同的串**, 其中**每一帧**都有一个横跨整幅画面的框
   (~600x250 像素, 被判成 '8', 分数 0.49~0.75), 它 x 最小, 于是每根串都多一个
   开头的 '8'。

   **分数门不是修法**: 那 11 帧里进串的框全在 0.374~0.917, **最低分**(0.374)
   属于最大的那个框, 而真 `粤` 只有 0.557 排在三个别的框后面 —— 没有任何阈值
   能把它们分开。 能分开的是**尺寸**。

   规则两条: (a) 按候选框自己的**中位**高宽丢掉尺寸离谱的 (高不在 [0.6,1.6]x
   中位, 宽不在 [0.5,1.6]x 中位); (b) 按**分数序**贪心去重, 与已保留框横坐标
   重叠超过较短者一半、且同一行的丢掉 —— 这就是那个重复 `粤` 的去处, 它与另
   一个 `粤` 的 IoU 是 41%, 刚好低于 YOLO_NMS_THRESH 0.45f, 而 nms 不看类。

   判据: 同一 11 帧, 8 种串 -> 4 种, 长度回到车牌自己的 7 个字形(10/11 帧),
   `粤` 11/11 帧排在第一位。 **它只删字符, 从不加** —— 所以 R 被读成 A/9、G 被
   读成 8 这些**模型**问题它一个都不修。 同工序的离线选型脚本是
   tools/_plate_rule.py (读同一份 log, 可对任何一次抓取重跑)。

   'v' 键切换, 复位即关; 关着时 yolo_plate_read() 与加这段之前逐字节相同 ——
   这同时是 tools/chain_emulate.py 的诚实性保证, 它就是从本函数解码的。 */
void yolo_plate_geo_set(int on);
int  yolo_plate_geo_get(void);

/* 车牌字符串缓冲建议大小: 省份1 + 字母1 + 序号5~6, 留余量 */
#define YOLO_PLATE_MAXLEN 12

/**
 * @brief  把检测框拼成车牌字符串 (GBK 编码)
 *         跳过 class_id==0 的整牌框, 其余框按 x1 从左到右重排后依次拼接:
 *         class 1..34  -> ASCII '0'-'9','A'-'Z' (不含 I/O, 与 voc.names 一致)
 *         class 35..69 -> 省份简称的 GBK 双字节 (如 45='吉' 0xBC 0xAA)
 * @param  boxes   : yolo_plate_detect() 的输出。注意它按 score 排序, 不是阅读
 *                   顺序, 本函数内部自己按 x1 重排, 不会修改 boxes。
 *                   yolo_plate_geo_get() 非 0 时先过一遍几何分组 (见上), 那只
 *                   会**减少**候选框。
 * @param  n       : 框数
 * @param  buf     : 输出缓冲 (以 '\0' 结尾)
 * @param  buf_size: 缓冲大小, 需 >= 2
 * @retval 写入的字节数 (不含 '\0'); 一个字符框都没有时为 0
 */
int yolo_plate_read(const YoloBox* boxes, int n, char* buf, int buf_size);

/* ========================= 两级级联 (v1.0 冻结版) =========================

   RELEASE_v1.0_2stage.md 那一套搬到板子上。三个网络串联:

     ① 整图找牌    voc-299      320x320 -> 车牌框          (已有, 见上)
     ② 裁剪找字    voc_crop_ft  320x320 裁剪图 -> 字符框
     ③ 逐字分类    charhead     32x64 小图 -> 字符类别      (已接: 见 flow 第 6 步)

   三张网都在 SDRAM 里各有自己的 tensor arena: ① 在 0xC0000000, ② 在 0xC0600000,
   ③ 在 0xC0A00000 (RW_SDRAM4)。 三个解释器互不共享下标, 都是原生算子注册 ——
   具体理由写在 .cpp 里 ② 那一段长的上面。

   为什么非要做成两级, 而不是继续调一个整图模型: ① 在一张 320 宽的图里给
   每个字只留 ~24 像素, 7 字整串因此卡在 0-1%; ② 拿到的是一块**只含车牌**
   的方图, 同样的字变成 40~50 像素, 后面接上 ③ 就是 **78.99%**(7 字桶 83.7%,
   RELEASE_v1.0_2stage.md §0/§6.3 的冻结值)。 帧间复用和裁剪外扩这些数字都在
   那份 RELEASE 和 RELEASE_v1.0_board.md 里, 不在这里重复。

   ---------------------------- 坐标空间, 三套, 别混 ----------------------------
     plate  : 相机像素 800x480。 ① 的框经 camera_map_box() 还原后的样子,
              裁剪矩形就是在这个空间里算的。
     chars  : **② 自己的输入图**像素, 320x320。 它是裁剪出来的方图的局部
              坐标, 原点在裁剪图左上角, 不是相机原点。
     text   : GBK 字符串。
   拿板子上的读数和 PC 上 full_pipeline_e2e.py 对拍时, 能直接对上的是
   yolo_cascade_crop_rect() 给出的矩形和 text; chars 要自己换算。

   -------------------------------- 一帧的流程 --------------------------------
     1. ① 出车牌框 (或复用上一轮缓存的, 见下), 经 camera_map_box() 回到相机像素。
        选哪一块: 分数在最高分 0.20 之内的 class 0 框里取**面积最大**
        (pick_plate_area_conf(); 早期版本是纯面积, 见 board RELEASE §1.1)
     2. 算命中的裁剪矩形 (复刻 full_pipeline_e2e.py 的 crop_by_plate:
        左上 floor、右下 ceil、clip 到图内)。 **首轮用训练契约的外扩 10%
        (YOLO_CROP_MARGIN), 之后每轮收窄/放宽到"让字间距回到训练 p50 25.8"**
        —— 比例夹在 [YOLO_VIEW_MINM, YOLO_VIEW_MAXM] = [0.02, 0.70], 每轮
        朝目标走半步 (YOLO_VIEW_MIX 0.50), 见下面"取景自适应"
     3. 把矩形双线性缩放到 320x320 写进 ② 的输入张量 (OpenCV 半像素约定)。
        字行倾斜超过 0.46 度时改走 resample_aff_to_i8: 沿**牌面自己的两个轴**
        重采样把字行转正 (YOLO_ROLL_MIN/MAX/MIX); 小于门限时逐像素等于只做缩放
     4. ② 出字符框, 类无关 NMS, conf 0.10 / NMS 0.30。 候选超过
        YOLO_CROP_MAXBOX 64 时**按分数**留前 64 (keep_top=1), **不是**光栅序
     5. 按分数取前 7 个 (class_id>0, YOLO_CHAR_MAX), 再按 x1 排序
     6. 7 字牌先补位 (n==6 末两位粘连时按格点补第 7 个, rebuild_row_7)、
        再把中心吸回牌面格点 i + 0.357*[i>=2] 并归一宽度 (snap_char_centers_7,
        无条件归一宽度, 只有整行位次可信时才投影中心)
     7. 每个框抠一张 32x64 (从**原生分辨率的裁剪图**上抠, 外扩 25%, 复刻
        cascade_ceiling.py 的 cut_patch), 过 ③, 取 argmax —— 类别写回框的
        class_id。 位次可信的 7 字行按**位**限定类别范围 (位 0 只认 35..69
        省份字、位 1 只认 11..34 字母、位 2..6 只认 1..34 数字字母);
        其余情况是全 70 类 argmax
     8. 交给 yolo_plate_read() 拼成 GBK 串 —— 拼串这条路径一个字节都没有重写,
        ③ 只提供类别, 不参与拼串

   第 6 步的坐标是 ② 的 320 空间, 而 patch 是从冻结帧上按裁剪矩形抠的: 框先按
   s2x = cw/320 (cw = 裁剪矩形宽) 映回原生像素, 再外扩、clip、重采样到 32x64。
   为什么不直接从 ② 的 320x320 输入上抠: Python 是从裁剪图**原始像素**抠的
   (int8_emulate.py:303 的 cc), 320x320 是它缩小之后的样子, 从那儿抠等于多缩
   一次 —— 字本来就小, 这一下是静默掉点。

   ~~~~~~~~~~~~~~~~~~~~~~~~ ① 的帧间复用, 为什么有它 ~~~~~~~~~~~~~~~~~~~~~~~~
   320x320 一次推理实测 1.7~4.6 秒 (src/main.c 的注释里有原始读数), 而一轮
   级联要跑 ① 和 ② 两次。 车牌在画面里的位置不会每 66 ms 变一次, 所以 ①
   没必要每帧跑: 缓存车牌框, 年龄到 YOLO_PLATE_REUSE_AGE 才重算。 稳态就
   只剩 ② 一次大推理, 每 5 帧才有一帧是两次, 平均 1.2 次。
   ② 一个字都没框出来时 (nchar==0) 提前作废缓存 —— 那通常正是车牌移走了
   或者 ① 上一次框歪了, 复用下去只会把这个错一直摆在那里。
   yolo_cascade_reuse_set(1) 关掉复用 (每帧都重跑 ①), 用来对照。

   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 两个必须知道的坑 ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   * 传进来的 frame888 **必须**和 rgb320 是同一帧。 相机是 ping-pong 的, 约
     66 ms 翻一次, 而 ① 要跑 1.7~4.6 秒 —— 等 ② 用的时候原帧早被覆盖了。
     冻结帧不是优化, 是正确性: 调两次 camera_frame_addr() 有可能拿到两张不同
     的图, 那样 ② 就从一个 ① 根本没看过的画面里裁字, 结果错得**很合理**,
     不打任何报错。 所以入口收的是 pin 好的指针, 不是自己去取地址。
   * 这个函数**不碰** yolo_plate_geo_set 那个开关。 默认关, 关着时拼串路径
     与加这段之前逐字节相同, 这也是 tools/chain_emulate.py 的诚实性保证。
   ========================================================================= */

#define YOLO_CHAR_MAX      7        /* 车牌字形数上限, 与 Python 的 cap7 一致 */
#define YOLO_CROP_CONF     0.10f    /* ② 的 conf: 2~3 像素的字在 0.25 就没了 */
#define YOLO_CROP_NMS      0.30f    /* ② 的 NMS: 字挨着, 0.45 会吃掉相邻字 */
#define YOLO_CROP_MAXBOX   64       /* 候选上限: 满了按分数留前 64 (keep_top=1),
                                       与 Python --maxbox 64 一致; 光栅序截断是
                                       2026-10-20 之前的旧行为, 已废 */
#define YOLO_CROP_MARGIN   0.10f    /* 首轮外扩 = 训练契约; 量到 pitch 后由取景
                                       自适应接管 (区间 0.02..0.70) */
#define YOLO_PLATE_PICK_MARGIN 0.20f /* 选牌框: 分数在中上 0.20 内再按面积取大 */
#define YOLO_PLATE_REUSE_AGE   5    /* ① 的框最多复用几轮; 1 = 不复用 */

/* ③ 的输入与输出规格。 这四个数是**模型契约**, 不是可调参数:
   输入 [1, 64, 32, 3] int8 (scale 1/255, zp -128), 输出 [1, 70] int8
   (scale 0.138210 > 0, zp 18)。 .cpp 的 init 里逐项核对着它们, 对不上就不启动。
   注意 YOLO_CHAR_IN_W 是**宽 32**、YOLO_CHAR_IN_H 是**高 64** —— 竖长条; 训练
   那边的 size_wh = [32, 64] 是 (W, H), 同一样东西的两种写法。 */
#define YOLO_CHAR_IN_W     32       /* ③ 输入宽 */
#define YOLO_CHAR_IN_H     64       /* ③ 输入高 */
#define YOLO_CHAR_CLS      70       /* ③ 类别数 = ① 的 names 数, 同一套下标空间 */
#define YOLO_CHAR_PAD      0.25f    /* 抠 patch 的外扩比例 (checkpoint 里的 pad) */

typedef struct {
    uint8_t  ok;                        /* 1 = 这一轮跑完了, 0 = 没找到车牌 */
    uint8_t  reused;                    /* 1 = 车牌框来自缓存, 这一轮没跑 ① */
    YoloBox  plate;                     /* 车牌框, **相机像素 800x480** */
    /* 字符框, **② 输入图 320x320 局部像素**, 按 x1 升序。
       跑过之后每个框的 class_id 是 **③ 认出来的类别** (0..69, 与 ① 的 names
       同一套下标), 不再是 ② 给的那个 —— ③ 没起来时才是 ② 的原值。 score 仍
       然是 ② 的分数。 */
    YoloBox  chars[YOLO_CHAR_MAX];
    int      nchar;                     /* chars[] 里的有效个数, 已按 x1 排序 */
    char     text[YOLO_PLATE_MAXLEN + 1]; /* GBK, 以 '\0' 结尾 (见 yolo_plate_read) */
    int      text_len;                  /* text 里的字节数, 不含 '\0' */
} yolo_cascade_result;

/**
 * @brief  初始化 ② 和 ③ 的解释器 (① 的 yolo_plate_init() 仍然要单独先调)。
 *         两者的算子一律**原生注册**, 不套 GPU 包装 —— 原因见 .cpp 里那段。
 *         任一环节失败即返回 false, 调用方应当退回单模型路径 (main.c 就是这么
 *         做的), 而失败原因由本函数自己的 "crop/charhead init FAIL: ..." 行说明。
 * @retval true 成功 (② 和 ③ 都可用)
 */
bool yolo_cascade_init(void);

/**
 * @brief  跑一轮级联: ① (或复用) -> 裁剪 -> ② -> 拼串。
 *
 * @param  rgb320   : 320x320 RGB888, ① 的输入 (每像素3字节)。 ① 被复用时**不读**。
 * @param  frame888 : 800x480 RGB888 冻结帧, 与 rgb320 同一帧。 裁剪从这里取。
 * @param  fw, fh   : frame888 的尺寸 (传 DVP_IMAGE_WIDTH/HEIGHT)。 裁剪矩形
 *                    要按它 clip, 而且 camera_map_box() 用的就是同一对宏 ——
 *                    传别的值, 两处就对不上了。
 * @param  out      : 结果。 总是被写满 (ok=0 时其余字段是安全的空值)。
 * @retval ② 出的字符框数 (>=0); <0 是错误 (-1 没初始化, -2 ② 的 Invoke 失败)
 */
int yolo_cascade_run(const uint8_t* rgb320, const uint8_t* frame888,
                     int fw, int fh, yolo_cascade_result* out);

/**
 * @brief  分段探针: 只跑 ②, 输入直接给一张 320x320 RGB888 图。
 *         用来把「② 本身对不对」和「裁剪通路对不对」分开 —— 板子导出一帧,
 *         PC 上切好车牌方图喂给 ②, 逐框对着 Python 看。
 * @param  rgb     : 320x320 RGB888
 * @param  boxes   : 输出缓冲
 * @param  max_box : 缓冲容量 (传 YOLO_CROP_MAXBOX)
 * @retval 框数; <0 错误
 */
int yolo_crop_detect(const uint8_t* rgb, YoloBox* boxes, int max_box);

/**
 * @brief  上一轮实际用的裁剪矩形, 相机像素, [x0, y0, x1, y1), 右下开区间。
 *         这是和 PC 对拍时最能一眼看出问题的那组数 —— 矩形错, 后面全错,
 *         而矩形错在板子上不会有任何别的症状。 未跑过时为全 0。
 */
void yolo_cascade_crop_rect(int* x0, int* y0, int* x1, int* y1);

/**
 * @brief  上一轮的逐级耗时, 单位微秒 (DWT 600 MHz 换算)。
 *         ① 被复用的那一轮, plate_us 是 0 —— 这本身就是"走了哪条路"的证据。
 *         char_us 是 ③ 整段的时间: 抠 patch + 每个字各自一次 Invoke + argmax,
 *         全部合在一起 (最多 7 次小推理)。 传 NULL 的项不写。
 */
void yolo_cascade_stat(uint32_t* plate_us, uint32_t* crop_us,
                       uint32_t* resample_us, uint32_t* crop_infer_us,
                       uint32_t* char_us);

/**
 * @brief  上一轮 ③ 真正给出类别的字符个数 (<= nchar)。
 *         ③ 没起来时恒为 0。 用来把"② 框出来的字"和"③ 认出来的字"分开 ——
 *         串短了到底是框漏了还是分类失败, 这两个数字合起来能一眼看出来。
 */
int yolo_cascade_char_cls(void);

/**
 * @brief  ① 的框最多复用几轮。 n<=1 表示不复用 (每轮都重跑 ①)。
 *         加这个 setter 是为了不改固件就能在板子上比"复用" 和 "不复用" 两条路。
 */
void yolo_cascade_reuse_set(int n);
int  yolo_cascade_reuse_get(void);

/**
 * @brief  清掉 ① 的帧间复用缓存, 下一次调用一定重跑 ①。
 *
 * main.c 的 'i' (同帧重跑) 必须调它。 不复位的话, 第二次按 'i' 会直接沿用
 * 第一次算出来的车牌框 —— 两次跑的工作量不同, 而 'i' 存在的**唯一**理由就是
 * 量"同一帧、同一条路是不是确定的", 次数不同就把这个读数毁了。
 */
void yolo_cascade_reset(void);

/**
 * @brief  复用轮里"画面变了多少": 在 ① 的框内取 64x48 个点, 与 ① 跑的那一轮
 *         同一框内同一批点逐点求平均绝对差 (三通道一起平均, 0..255)。
 *
 *         只量, 不参与任何判断 —— 复用判据现在是**时间**(龄期), 而时间不是
 *         "画面变没变"的代理。 这个数就是把它换成**内容**判据之前必须有的
 *         分布: 静止场景 / 牌移了几像素 / 换了一块牌, 三种情况的读数各是多少。
 *
 * @retval 平均绝对差; 0 表示这一轮 ① 刚跑过 (参考刚取); 0xFFFFFFFF = 没有
 *         参考 (上一轮没框出牌, 或 ① 还没跑过)。
 */
uint32_t yolo_cascade_change(void);

/**
 * @brief  ① 最近一次真正跑出来的原始候选框 (类无关 NMS 之后、pick 之前),
 *         是 **320x320 检测坐标**, 调用方要自己 camera_map_box() 换成相机像素。
 *
 *         为什么需要它: 板上日志里 ① 的牌框宽度出现两簇 (268 px / 313 px),
 *         一簇整串对、一簇整串错, 而 `plate` 那行只给了 pick 之后的那一个框 ——
 *         分不出是 ① 的回归本身在跳, 还是 pick_plate_area_conf() 在两个候选
 *         之间做选择。 这个列表是那两种原因唯一的区分手段。
 *
 *         只在 ① 真跑过的那一轮有效 (级联结果 s_cas.reused == 0); 复用轮里
 *         它留着上一次的旧值, 所以调用方必须自己按 reused 判掉。
 *
 * @param  dst : 输出缓冲, 可以为 NULL (只为取数量)
 * @param  max : 缓冲容量
 * @retval 列表长度 (可能大于 max, 表示只拷了 max 个)
 */
int yolo_cascade_plate_raw(YoloBox* dst, int max);

/**
 * @brief  ③ 对第 i 个字符的判决余量: top1 与 top2 的 int8 输出之差。
 *
 *         乘 0.138 得到 logit 差。 用它回答"同一块牌为什么每轮串不一样":
 *         若翻转的那个字余量只有 0..3, 说明模型正踩在判决边界上 (换一帧就翻),
 *         链路上并不存在随机性; 若余量很大却仍翻, 才说明别处有问题。
 *
 * @param  i : 字符下标 (0 起, 按 x1 排序, 与 cbox 行一致)
 * @retval >=0 余量; -1 = 这一位没有分类到 (③ 没起来、Invoke 失败或 i 越界)
 */
int yolo_cascade_char_margin(int i);

/**
 * @brief  牌面格点这一轮做了什么, 给串口 `sn` 字段用。
 *
 *         7 字牌才有格点 (位 0 省份字 | 位 1 字母 | 隔离点 | 位 2..6 序号)。
 *           bit 0..6 : 该位相对拟合格点的残差 > 0.15*pitch (**只是"偏了"**,
 *                      不代表一定动了)
 *           bit 31   : 本轮**投影了中心** (0x80000000)
 *         中心的判定是整行的: 行内最大残差 <= 0.20*pitch 时一个像素都不动,
 *         超过才把 7 个中心一起投到格点上。 所以 `sn 00000010` 是"第 4 位偏了
 *         但整行还准, 中心没动", `sn 80000010` 才是"偏了且中心已投影"。
 *
 *         **宽度不在这个位里**: 只要位次可信 (最大残差 <= 0.45*pitch), 7 个框
 *         的宽度每轮都会被归一到中位数 —— 实测宽度才是读错的主因 (29 px 读对,
 *         48 px 读错), 所以它不跟着中心门限走。 宽度改没改直接看 cbox 的 x1/x2。
 */
uint32_t yolo_cascade_char_snap(void);

/**
 * @brief  取景自适应: 上一轮量到的字距 (320 空间, x10) 和当前的裁剪宽/①框宽。
 *
 *         ② 只在一种取景下训练过 (pitch 25.8, 裁剪宽高比 1.78)。 板上 ① 的框
 *         比训练标签紧, 同一个"框 + 10%"会把字放大到 pitch 29..36, 比 ② 的
 *         最大 anchor (27.7x133.9) 还大。 现在每轮用 ② 自己的输出反推裁剪宽,
 *         让 pitch 落回 25.8 —— 不增加推理, 相当于自动变焦。
 *
 *         `vp 262,34` = 量到 pitch 26.2, 当前外扩 34% (裁剪宽 = 框宽 x 1.68)。
 *         第一轮 (还没量到) 打印 `vp 0,0`, 此时走的是训练契约 (框 + 10%)。
 *
 *         外扩比例夹在 **[2%, 70%]**。 下限为什么不是 10% (训练契约): 板上实测
 *         远牌 (①框 293~313 px) 需要的比例是 0.025~0.03, 夹在 10% 时 pitch 只有
 *         18.7~22.7 —— 字比训练**小** 27%, 末两位粘连、省字位每轮翻。 这里量的
 *         是 pitch (字实际多大), 不是"裁得多紧"; 字大小对齐训练才是那条契约的本意。
 *
 *         到 70% 还压不下来, 说明车牌在帧里太大了 (裁剪已经被帧边界截住),
 *         那是物理限制, 只能退相机或改输入尺度。
 *
 *         收敛节奏: 每轮朝"目标"推一半, 与 ② 这一轮有没有给满 7 个框**无关**
 *         —— 掉框本身就是尺度不对的症状, 不能让它反过来冻住尺度。
 */
int yolo_cascade_view_pitch_x10(void);
int yolo_cascade_view_margin_x100(void);

/**
 * @brief  字行的倾斜 dy/dx (x1000, **320 空间**)。 正 = 右边偏低。
 *
 *         7 个字符中心的最小二乘斜率。 要点: 这是 **320 空间**的数, 不是物理角度。
 *         ② 的输入把裁剪矩形 (cw x ch) 硬拉成 320x320, 所以帧里的真实横滚是
 *         `slope * ch/cw`; 一块 280x95 的裁剪会把 7.9 度放大成 22 度来显示。
 *
 *         本字段现在是**校正之后**的残余倾斜 (每轮已经按下面的 roll 转过一次)。
 */
int yolo_cascade_view_slope_x1000(void);

/**
 * @brief  本轮真正用上的横滚角 (x1000 rad), 正 = 右边低。
 *
 *         与 slope 的区别: slope 是残余, roll 是累计的总角度。
 *         两个都趋近 0 才是真转正; 一个 0、一个很大说明正在收敛 (1~2 轮)。
 *
 *         怎么来的: 每轮从 ② 的 7 个字框中心量残余倾斜 s320, 折回帧空间
 *         `s320*ch/cw` 后**加**到已用角度上 (拿残差当角度会来回摆), 再乘 0.65
 *         朝新目标走一步。 |角度| < 0.008 rad (0.46 度) 时不启用旋转通路,
 *         仍走 resample_rect_to_i8 —— 竖直车牌逐像素等于冻结基线。
 *
 *         旋转只作用于**取样轴**: 裁剪矩形本身、叠加框、日志里的 crop 都还是
 *         轴对齐的, 所以画面上的框不会跟着转。
 */
int yolo_cascade_roll_x1000(void);

/**
 * @brief  这一轮 (no char) 的原因: 0 = 正常, 1 = ① 一个牌框都没有,
 *         2 = ① 有框但被形状闸拒了 (高大于宽, 见 plate_shaped_cam)。
 *
 *         两者在日志上原本完全一样 (box 0,0 0,0 n 0), 但处置完全不同 ——
 *         1 是检测器/画面的事, 2 是"框错了地方", 继续往下走只会编假串。
 */
int yolo_cascade_reason(void);

/**
 * @brief  ② 的**原始**输出框数 (select_chars_7 截到 7 之前)。
 *
 *         比 nchar 大就说明这一轮 ② 框多了。 select_chars_7 是按**分数**截断
 *         的, 所以多出来的那个不一定是最差的框 —— 整行错位一位 (前导多一个
 *         像字的框) 就长这个样子。
 */
int yolo_cascade_n2(void);

/**
 * @brief  ② 里**过 conf 的候选总数** (两个头加起来, NMS 之前, 不截断)。
 *
 *         大于 YOLO_CROP_MAXBOX(64) 就说明那一刻框数超过了缓冲容量 —— 现在的
 *         解码是按**分数**留最高的 64 个 (与 PC 链路"按分数取 16"同一口径),
 *         超出部分丢的是分数最低的, 理论上无害; 但这个数持续几十上百, 就说明
 *         conf 0.10 在这张裁剪图上太松, 该考虑提高 conf 而不是加大缓冲。
 *
 *         n2 小 (6~9) **不能**证明没超: n2 是 NMS 之后的数。 两个数一起看。
 */
int yolo_cascade_nraw(void);

/**
 * @brief  n==6 补位的诊断 (本轮 ② 恰好给出 6 个框时才返回 11, 否则 0)。
 *
 *         n==6 时末两位常常粘成一个框 (宽 34..54, 正常 26), 直接按 6 字拼串就是
 *         粤R8888 / 粤R888沪 / 粤R888湘。 rebuild_row_7() 试着用牌面格点把缺的
 *         一位补回来 —— patch 是按位置从冻结帧抠的, 格点对, 抠到的字就对。
 *
 *         两道闸都过了才补 (残差都相对 pitch):
 *           A: 6 个框套 7 槽 (缺 1 槽),      <= 0.15
 *           B: 丢 1 个框, 5 点套 7 槽,       <= 0.08
 *         真·6 字牌在 A 上约 0.26、B 上约 0.13, 都过不去 —— 不会被硬掰成 7 字。
 *
 *         buf 至少 11 个 int:
 *           [0] 框数 (=6)     [1] 1 走 A / 2 走 B / 0 没补
 *           [2] B 丢掉的框下标 (-1 = 走 A)
 *           [3] 拟合出的 pitch x10
 *           [4] 最大残差/pitch x100 (999 = 退化)
 *           [5..10] 补位**之前**的 6 个框中心 (320 空间)
 *
 *         判读: `ch6` 行里 `acc 0` 表示这道保险没触发, 看 `c` 那 6 个中心就能
 *         知道是"末位缺了"还是"中间缺了"; `acc 1/2` 表示补了, 串会自动变回 7 位。
 */
int yolo_cascade_ch6_diag(int* buf, int cap);

#ifdef __cplusplus
}
#endif

#endif // YOLO_INFERENCE_H
