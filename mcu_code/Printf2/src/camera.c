/**
 * @file camera.c
 * @brief OV5640 on DVP1 -> LCD live preview, and a 320x320 RGB888 grab for the
 *        plate detector.
 *
 * Follows the vendor demo projects\n32h7xx_EVAL\examples\DVP\Camera_OV5640_LCDC
 * (bsp_dvp.c / bsp_lcd.c / bsp_sccb.c / bsp_ov5640.c, copied into inc/ and src/
 * unchanged apart from the DVP1 selection and the frame buffer address).  What
 * is added here is the glue: the frame buffer sits above the YOLO tensor arena
 * instead of at 0xC0000000, and the grab converts the sensor's RGB565 into the
 * packed RGB888 the model takes.
 */

#include "camera.h"
#include "bsp_dvp.h"
#include "bsp_lcd.h"
#include "bsp_sccb.h"
#include "bsp_ov5640.h"
#include "n32h7xx_eval.h"
#include "gpu.h"        /* the GPU grab path and the agreement probe below */
#include <stdio.h>
#include "Message.h"

#define CAM_W   DVP_IMAGE_WIDTH     /* 800 */
#define CAM_H   DVP_IMAGE_HEIGHT    /* 480 */

/* The DVP packs one RGB565 halfword per pixel.  If the picture on the LCD
   comes out with red and blue exchanged, set this to 1: the order is decided
   by DVP_PIXEL_BYTE1 and DVP_ConfigDATAByteOrder(), and which way round they
   land cannot be settled without looking at the sensor's output. */
#define CAM_RGB565_SWAP_RB  0

/* 800 -> 320 is 2.5, 480 -> 320 is 1.5.  Both are exact in 8 fractional bits,
   so the resize has no rounding drift across the frame. */
#define DET_STEP_X   ((uint32_t)((CAM_W << 8) / YOLO_IN_W))   /* 640 */
#define DET_STEP_Y   ((uint32_t)((CAM_H << 8) / YOLO_IN_H))   /* 384 */

/* Whether the GPU's version of the grab is close enough to the CPU's to be
   used for detection.  Latching it from the boot probe rather than making it a
   build switch means one flash answers both questions - the probe line says how
   far apart the two are, and the trigger loop's own `grab ? us` says what that
   cost - and it means a GPU that disagrees falls back on its own instead of
   quietly feeding the model a picture nobody checked.

   The bound is on the MEAN channel difference over the whole frame, not on
   equality, because the two cannot be equal even when both are right: the CPU
   steps the source at exactly 2.5 pixels per output pixel and the GPU's scale
   gives 2.4922, and they round in a different order.  A working pair lands in
   single digits; a wrong buffer or a swapped channel lands in the hundreds.
   16 sits between those and closer to the first.  Measured 2026-09-15, the
   working case had not yet been seen - the fault case read 109. */
#define CAM_GPU_AGREE_MAX  16u

static bool s_gpu_grab_ok = false;

/* Whether the SECOND half of the front end - the 3-tap separable convolve - is
   faithful, latched on the same kind of evidence and against the same bound.

   This is a separate latch because the shipped front end is two GPU stages, not
   one, and until this existed the second stage was in no boot verdict at all.
   The three measurements that were being read as if they covered it, and what
   each of them is actually about:

     - camera_gpu_agreement_probe() compares GPU_SCALE_DST, which is the BLIT's
       output.  The grab then filters it and unpacks GPU_SCALE_CONV instead.  So
       `gpu agree mean 5 max 199 swap 10 ok 1` is a verdict on a picture the
       product never hands the model.
     - gpu_filter_probe() runs the convolve engine on a synthetic grating with an
       IDENTITY kernel.  That answers "is the convolve's geometry right at
       320x320" - a real question, and it is why that probe exists - but not "is
       the shipped 3-tap kernel over a real frame the same picture the CPU path
       produces".
     - gpu_filter_frame()'s own return value says the driver accepted the
       configuration, not that the arithmetic came out right.

   Three green readings and no measurement of the composite.  That is the hole
   this closes: the filter leg below runs the shipped kernel over the shipped
   blit on a real frame and compares the result with the CPU image, and
   camera_grab_rgb888()/camera_grab_pair() apply the filter only when it passed.

   Default false, so a build whose probe never reaches the filter leg ships the
   unfiltered blit - the thing the first leg did validate - rather than an
   unmeasured filter.  A stage that fails its own check must cost that stage and
   nothing else; the alternative is what the reported round shows: one unmeasured
   stage in the middle of the detection path, with every printed number green. */
static bool s_gpu_filter_ok = false;

static void unpack_rgb565_to_rgb888(const uint16_t* src, uint8_t* dst);

/* The frame buffer the LCDC is scanning, i.e. the one the DVP finished filling
   most recently - DVPx_IRQHandler() sets it and only ever sets it to a buffer
   that is complete.  Written from an interrupt, read from the main loop; a
   32-bit aligned store is atomic on this core, so a plain read gives either the
   old value or the new one and never a mix of the two.                      */
static volatile uint32_t s_ready_addr = CAM_FRAME_BUFFER_ADDRESS;

/* Completed frames since boot, for the probes that need to wait for a real
   picture rather than read whatever the SDRAM powered up with.  s_ready_addr
   cannot answer that on its own: it is non-zero from the very first
   instruction, so "is there a frame yet" is not a question it can be asked. */
static volatile uint32_t s_frames = 0u;

void camera_frame_done(uint32_t addr)
{
    s_ready_addr = addr;
    s_frames++;
}

uint32_t camera_frame_addr(void)
{
    return s_ready_addr;
}

uint32_t camera_frame_count(void)
{
    return s_frames;
}

/* LCDC 像素时钟的运行时 A/B。 见 camera.h 里 camera_lcd_clk_toggle() 的说明。

   默认 1 = AXI/16。 **这是屏闪的定案修法**, 不是 A/B 的缺省位置: 2026-10-19
   上板实测, 同一块牌、同一份固件, 只按一次 'l' (AXI/8 -> AXI/16) 之后
   `lcd fu 1/1/0 fuk 1/1/0` 立刻变成 `0/0/0`, 屏不再闪; 而且推理从 2.6 s
   掉到 1.38 s (抓帧 385 -> 325 ms) —— 说明之前 LCDC 每帧从 SDRAM 取
   ~49 MB/s 的像素, 把这块"原厂确认的慢 SDRAM"占去了一半, 屏和推理一起挨饿。
   /16 之后屏只要 ~25 MB/s, 两边都够。                                          */
static uint32_t s_lcd_clk_slow = 1u;

uint32_t camera_lcd_clk_toggle(void)
{
    s_lcd_clk_slow ^= 1u;
    RCC_ConfigLCDCPixelClk(RCC_LCDCPIXELCLK_SRC_AXIDIV,
                           (s_lcd_clk_slow != 0u) ? RCC_LCDCPIXELCLK_AXIDIV16
                                                  : RCC_LCDCPIXELCLK_AXIDIV8);
    return s_lcd_clk_slow;
}

void camera_lcd_status(uint32_t* intsts, uint32_t* inten)
{
    *intsts = LCDC->INTSTS;
    *inten  = LCDC->INTEN;
}



/**
 *\*\name    rgb565_unpack.
 *\*\fun     Split a packed RGB565 pixel into 8-bit channels.
 *\*\param   v  packed pixel
 *\*\param   r,g,b  receive the channels
**/
static void rgb565_unpack(uint16_t v, uint32_t* r, uint32_t* g, uint32_t* b)
{
#if CAM_RGB565_SWAP_RB
    uint32_t rv = (uint32_t)(v & 0x1Fu);
    uint32_t bv = (uint32_t)((v >> 11) & 0x1Fu);
#else
    uint32_t rv = (uint32_t)((v >> 11) & 0x1Fu);
    uint32_t bv = (uint32_t)(v & 0x1Fu);
#endif
    uint32_t gv = (uint32_t)((v >> 5) & 0x3Fu);

    /* Replicate the top bits into the low ones so a saturated channel reaches
       255 rather than 248/252 - the model was trained on 0..255 images, and
       the int8 quantisation in yolo_plate_detect() is a plain (v - 128).    */
    *r = (rv << 3) | (rv >> 2);
    *g = (gv << 2) | (gv >> 4);
    *b = (bv << 3) | (bv >> 2);
}

/**
 *\*\name    blend.
 *\*\fun     Linear blend of two 8-bit channel values, f in 0..255.
**/
static inline uint32_t blend(uint32_t lo, uint32_t hi, uint32_t f)
{
    return (lo * (256u - f) + hi * f) >> 8;
}

/**
 *\*\name    downsample_rgb565_to_rgb888.
 *\*\fun     800x480 RGB565 -> 320x320 RGB888 by bilinear interpolation.
 *\*\param   src  the camera frame buffer
 *\*\param   dst  320*320*3 bytes
 *
 * Nearest-neighbour would be simpler, but the ratios are 2.5 and 1.5, so it
 * would drop three of every five columns: a plate character is only about 20
 * pixels wide once the frame is stretched to 320, and that much aliasing eats
 * straight into what is left.  Two extra lines per channel are the whole
 * difference.
**/
static void downsample_rgb565_to_rgb888(const uint16_t* src, uint8_t* dst)
{
    uint32_t i;
    uint32_t j;

    for (i = 0u; i < YOLO_IN_H; i++)
    {
        uint32_t sy = i * DET_STEP_Y;
        uint32_t y0 = sy >> 8;
        uint32_t fy = sy & 0xFFu;
        uint32_t y1 = (y0 + 1u < CAM_H) ? (y0 + 1u) : y0;
        const uint16_t* row0 = src + y0 * CAM_W;
        const uint16_t* row1 = src + y1 * CAM_W;
        uint8_t* out = dst + i * YOLO_IN_W * 3u;

        for (j = 0u; j < YOLO_IN_W; j++)
        {
            uint32_t sx = j * DET_STEP_X;
            uint32_t x0 = sx >> 8;
            uint32_t fx = sx & 0xFFu;
            uint32_t x1 = (x0 + 1u < CAM_W) ? (x0 + 1u) : x0;
            uint32_t p0r, p0g, p0b, p1r, p1g, p1b;
            uint32_t q0r, q0g, q0b, q1r, q1g, q1b;

            rgb565_unpack(row0[x0], &p0r, &p0g, &p0b);
            rgb565_unpack(row0[x1], &p1r, &p1g, &p1b);
            rgb565_unpack(row1[x0], &q0r, &q0g, &q0b);
            rgb565_unpack(row1[x1], &q1r, &q1g, &q1b);

            out[0] = (uint8_t)blend(blend(p0r, p1r, fx), blend(q0r, q1r, fx), fy);
            out[1] = (uint8_t)blend(blend(p0g, p1g, fx), blend(q0g, q1g, fx), fy);
            out[2] = (uint8_t)blend(blend(p0b, p1b, fx), blend(q0b, q1b, fx), fy);
            out += 3;
        }
    }
}

/**
 *\*\name    camera_init.
**/
bool camera_init(void)
{
    /* LCDC first: it is what scans the frame buffer out, and it must be ready
       before the DVP starts filling it, or the preview shows whatever the
       SDRAM powered up with. */
    LCDC_RCC_Configuration();

    /* bsp_lcd.h 的 LCDC_PIXEL_CLK_CONFIG() 给的是 AXI/8 (37.5 MHz, 800x480@64 Hz
       要 ~49 MB/s 的 SDRAM 读)。 这块片子的 SDRAM 是原厂确认的慢, 实测读
       带宽只有 24 MB/s 左右, 于是 LCDC 长期贴着上限跑, 抓帧/推理时直接把屏
       饿到下溢 (fu/fuk=1), 屏就在错位/滚。 开机直接降到 AXI/16 (~32 Hz,
       ~25 MB/s): 上板实测闪完全消失, 而且推理时间反而减半。 'l' 键仍可切回
       /8 做 A/B。 见下面 s_lcd_clk_slow 的注释。                              */
    RCC_ConfigLCDCPixelClk(RCC_LCDCPIXELCLK_SRC_AXIDIV,
                           RCC_LCDCPIXELCLK_AXIDIV16);

    LCDC_GPIO_Configuration();
    GPIO_WriteBit(LCDC_DISP_GPIO_PORT, LCDC_DISP_GPIO_PIN, Bit_SET);
    GPIO_WriteBit(LCDC_BL_GPIO_PORT,   LCDC_BL_GPIO_PIN,   Bit_SET);
    LCDC_Glober_Configuration();
    /* Start the LCDC on buffer 1, matching s_ready_addr's initial value.  The
       first M1TC re-points it at buffer 1 anyway; until then this is the only
       thing keeping it off uninitialised SDRAM.                              */
    LCDC_Layer_Configuration(LCDC_Layer1, CAM_W, CAM_H, CAM_FRAME_BUFFER_ADDRESS);
    LCDC_Enable(ENABLE);

    /* 让 FU/FUK/BE/RFE 这四位的**使能**位也置上, 但不打开 NVIC 里的 LCDC 中断。

       为什么要动它: 这四位一直是 0 —— 抓帧段、推理段、空闲段全 0 —— 而屏在闪。
       有两种可能, 从来没被分开过: (a) LCDC 真的没饿死; (b) 这位的状态只在对应
       的 INTEN 位打开时才更新, 而我们一直没开, 所以读到的 0 什么也不证明。
       本工程确实没开 LCDC 中断 (src/ 里搜不到 LCDC_IRQHandler / NVIC 使能),
       所以置 INTEN 不会产生中断, 只是把 (b) 这个可能性去掉。

       判读: 下一版日志里 fu/fuk/be/rfe 只要有一位从 0 变成 1, 就同时证明了
       (b) 和"确实出过事"; 若还是全 0, 那这四位的读数从此可信, 闪烁不在
       LCDC 取数这条路上, 该去查面板时序/换页。                            */
    LCDC->INTEN |= (LCDC_INTEN_FUIEN | LCDC_INTEN_FUKIEN |
                    LCDC_INTEN_BEIEN | LCDC_INTEN_RFEIEN);

    DVP_RCC_Configuration();
    DVP_NVIC_Configuration();
    DVP_GPIO_Configuration();
    DVP_Configuration();

    SCCB_RCC_Configuration();
    SCCB_GPIO_Configuration();

    if (OV5640_Init() != 0u)
    {
        return false;
    }
    OV5640_RGB565_Config();

    /* Only now let the sensor data flow: everything above is configuration. */
    DVP_EnablePort(DVPx, ENABLE);
    return true;
}

/**
 *\*\name    camera_dvp_pause / camera_dvp_resume.
 *
 * Why these exist: the cascade has to read one frame **twice** - once as the
 * full 800x480 RGB888 the crop is cut from, once as the 320x320 the detector
 * sees - and the two must be the same frame or the box it finds does not point
 * at the pixels it crops.  The way that was checked was camera_frame_count():
 * take it before the two reads, take it after, equal means no frame boundary
 * passed.  That check cannot come out true.  A frame period is ~66 ms (the DVP
 * ping-pongs between two buffers, so each one is untouched for exactly one of
 * them), and the two reads take ~300 ms - four and a half frame periods.  So
 * the cascade's same-frame gate failed on every attempt and the board ran the
 * single-model path in every log anyone read (2026-10-16: `grab 911017 us` =
 * three attempts x ~304 ms, all three refused; `n 9` on a path whose cap is 7).
 *
 * Stopping the port makes the check passable: no frame-end interrupts, so no
 * frame boundary can pass while we read.  What is left is the frame already in
 * flight when DISABLE lands - the port stops at a frame boundary - so pause()
 * blocks for a little over one frame period first, and after that returns the
 * count cannot move again.
 *
 * The read is no longer racing anything, so it does not matter that it is slow.
 * What it costs is a preview that stands still for ~300 ms on each scan; the
 * inference right after it already stops the preview for 1.7 s, so this is not
 * a new kind of cost - and the alternative measured out to "the cascade never
 * runs", which is not a tradeoff at all.
 *
 * resume() is not optional.  A code path that pauses and does not resume leaves
 * the preview frozen on the last frame forever, and nothing in the log says so.
**/
void camera_dvp_pause(void)
{
    /* 停在哪: DVP_EnablePort(DISABLE) 只清 CTRL 的 ENABLE 位, 端口在**帧边界**
       停下, 所以可能还有一帧在飞。 等它落地之后再读任何一页, 那一页就是静止的。

       150 ms 是量出来的, 不是拍的: 2026-10-19 的 `dvp ... idle 48 in 5000 ms`
       说明空闲帧率只有 **9.6 fps**, 一个帧周期是 **104 ms**, 不是注释里原先写的
       66 ms (那是按 15 fps 假设的)。 原来的 100 ms 等待比一个帧周期还短 —— 停
       在帧中间时, 帧尾可能还没落地就被读了。 取 1.5 个周期留余量。
       这个文件里别的地方也是用 DWT->CYCCNT 数周期 (见下面 gpu_agreement_probe),
       600 MHz 下 1 ms 是 60 万。 */
    const uint32_t t0 = DWT->CYCCNT;

    DVP_EnablePort(DVPx, DISABLE);

    while ((DWT->CYCCNT - t0) < 90000000u)
    {
        /* spin: 等正在飞的那一帧结束 */
    }
}

void camera_dvp_resume(void)
{
    /* 恢复后**不丢换页** (曾经丢 2 帧)。 2026-10-19 查清:
       1. 恢复后的坏帧来自**推理抢 SDRAM 总线**, 而推理段现在整个把 DVP 停住
          (main.c 的 dvp_held), 恢复不再产生坏帧 (`bad i` 恒 0);
       2. **丢换页本身会制造一帧撕裂**, 而且只有两个帧缓冲时是算术不是概率:
          屏显示 X、DVP 写 Y, 帧结束该切到 Y; 丢掉这次换页 ⇒ 屏还显示 X,
          下一帧 DVP 正好开始写 X ⇒ 整页在扫描它的同时被覆盖。

       被丢的那一帧只要不是显示页就安全, 而**再丢一帧就一定是**。 所以这里
       只把端口打开。 */
    DVP_EnablePort(DVPx, ENABLE);
}

/* 相机写坏一帧的证据。 若丢的是 DVP 的写而不是 LCDC 的读, LCDC 会**忠实
   地**把一张残帧显示出来 —— 看起来正是"闪/撕", 而 LCDC 的状态位一个都不置。
   DVP 自己就有这几个能证伪的证据:

     M1FO/M2FO   这一页的写 FIFO 溢出了 —— 像素没地方去, 丢了
     AHBERR1/2   那一页的 AHB 写被回了错误
     M1O/M2O     整页缓冲溢出 (写指针越界)
     SKIP        跳帧

   这个掩码刻意不含 M1TC/M2TC (那两个是中断源, 清了就丢帧)。 */
#define CAM_DVP_ERR_MASK  (DVP_FLAG_M1FO | DVP_FLAG_M2FO | \
                           DVP_FLAG_AHBERR1 | DVP_FLAG_AHBERR2 | \
                           DVP_FLAG_M1O | DVP_FLAG_M2O | DVP_FLAG_SKIP)

/* 每帧一次的坏帧计数。 粘滞位只能回答"这一段坏过", 而这里回答"坏了几帧"
   —— 两者分开的正是两种病: 每轮 1~2 帧是暂停/恢复的产物, 20+ 帧才是推理
   在抢总线。 现在推理段整个停住 DVP, 这个数应当恒为 0 ('d' 键读它)。

   ISR 里调用, 每帧把错误位读走再写 1 清 (只清错误位, 不碰 M1TC/M2TC)。 */
static volatile uint32_t s_dvp_bad = 0u;

void camera_dvp_frame_end_scan(void)
{
    const uint32_t err = (uint32_t)DVPx->INTSTS & CAM_DVP_ERR_MASK;

    if (err != 0u)
    {
        s_dvp_bad++;
        DVPx->INTSTS = err;
    }
}

uint32_t camera_dvp_bad_frames(void)
{
    return s_dvp_bad;
}

/**
 *\*\name    camera_grab_rgb888.
**/
bool camera_grab_rgb888(uint8_t* dst)
{
    /* Just read the buffer the LCDC is showing - the DVP's frame-end interrupt
       has already declared it complete, so there is nothing to wait for and no
       reason to stop the DVP.  The conversion below happens while the DVP fills
       the *other* buffer, so the preview keeps running through the inference.

       This is safe because s_ready_addr names a buffer the DVP does not touch
       again until the following frame boundary - a whole frame period away -
       and the conversion takes a few milliseconds at most.  Snapshotting the
       address once also means a flip part-way through cannot make us read two
       different frames.                                                      */
    const uint32_t addr = s_ready_addr;

    /* The GPU path, but only if it is SWITCHED ON and the boot probe found it
       faithful - two different questions, both load-bearing.

       `g_gpu_front_on` is the switch (gpu.h, flipped by the serial command in
       main.c) and it is 1 out of reset as of 2026-10-08, so this product grabs
       through the GPU unless someone presses `f`.  The block that used to hold
       it at 0 was a measurement taken while nsgpu_draw_rotblit was clipped to a
       single texel - "`pair cpu:` reads characters off a frame and `pair gpu:`
       reads none off the SAME frame" - so it is VOID, and gpu.h on that
       variable carries the correction.  What moved it to 1 is also a
       measurement: the product path costs 171753..182057 us on the CPU bilinear
       grab against 111370..116776 us with this front end on, i.e. 58..62 ms
       cheaper, 6/6 detect with a readable plate on both arms and no wedge in 12
       grabs.

       The two arms are still NOT the same picture - a filtered GPU blit against
       an unfiltered bilinear walk - and a mean-channel difference of 4..5 is not
       what a detector reads, it reads a marginal decode.  That difference stays
       part of the argument, and `p` is still the instrument that compares the
       two arms on one frame.

       `s_gpu_grab_ok` is the boot probe's verdict, and it stays a separate
       latch on purpose: the switch says what we WANT, the latch says what was
       MEASURED, and an unmeasured stage must not be able to ride in on a
       switch.  gpu_scale_frame() returning false (GPU not up, the driver refused
       RGB565, or a drain has latched the GPU dead - gpu.h) drops through to the
       CPU as well, so the GPU stays an accelerator and never a prerequisite.

       Note what is NOT here: a fallback for "the GPU produced a picture that is
       wrong but not detectably so".  That is what the probe's threshold is for,
       and it is why the threshold is checked once at boot on a real frame
       rather than assumed. */
#if GPU_ENABLE
    if ((g_gpu_front_on != 0u) &&
        s_gpu_grab_ok && gpu_scale_frame(addr, (uint32_t)CAM_W, (uint32_t)CAM_H,
                                         (uint32_t)CAM_W))
    {
        const uint16_t* px = (const uint16_t*)GPU_SCALE_DST;

#if GPU_FILTER_ENABLE
        /* Stage 3, the 3-tap separable convolve, when the boot probe's filter leg
           found it faithful - see s_gpu_filter_ok.

           Both conditions are load-bearing and they are different questions.  The
           macro says the stage is WANTED (a build decision); the latch says it is
           TRUSTED (a measurement).  Gating on the macro alone is what let one
           unmeasured stage sit in the middle of the detection path while every
           printed number stayed green.

           A false return is not a failure of the grab: the filter is an
           accelerator on an accelerator, and the blit's output - which is what
           the grab used before this existed - is still sitting in GPU_SCALE_DST,
           so the unpack just reads the unfiltered picture instead.

           A true return MOVES the pointer, because the two buffers are not
           interchangeable afterwards: the filter reads GPU_SCALE_DST and writes
           GPU_SCALE_CONV, and GPU_SCALE_CONV is then the only valid one.

           NO CONDITION IS ADDED HERE for the operator swap, deliberately.
           g_gpu_front_sep_on (gpu.h) chooses BETWEEN TWO OPERATORS, not between
           filtering and not filtering - GPU_FILTER_ENABLE is 1, so stage 3 is
           part of the front end this product ships, and "off" has to mean the
           library's filter rather than no filter at all.  The choice is made
           inside gpu_filter_frame(), once, so that this site and
           camera_grab_pair() cannot drift apart - the pair arm exists to measure
           the front end the PRODUCT has.  An earlier cut put the AND here and at
           the pair, and that made "off" silently drop stage 3, i.e. hand the
           model a picture it had never been fed and leave the pair with no
           control arm. */
        if (s_gpu_filter_ok && gpu_filter_frame())
        {
            px = (const uint16_t*)GPU_SCALE_CONV;
        }
#endif

        unpack_rgb565_to_rgb888(px, dst);
        return true;
    }
#endif  /* GPU_ENABLE - the whole GPU front end ends here */

    /* The CPU path: the ONLY path when GPU_ENABLE is 0, and the fallback when
       the arm above is switched off, not yet latched as faithful, or refused.
       It is NOT the same picture - see gpu.h's GPU_ENABLE block. */
    downsample_rgb565_to_rgb888((const uint16_t*)addr, dst);
    return true;
}

/**
 *\*\name    chan_absdiff.
 *\*\fun     Absolute difference of two channel values.
**/
static uint32_t chan_absdiff(uint32_t a, uint32_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

/* A `pair diff max` on its own cannot be read.  The same 198 comes back on six
   different frames, so the number is a property of the two operators and not of
   the scene - but two very different faults both produce a scene-independent
   maximum: a strip along the plane's outer edge (the separable pair's border
   clamp), and a whole plane that misses by a roughly constant amount.  mean and
   max are the same in both cases.  The pixel that owns the max is not.

   So agree_stats() can hand back where the worst channel sits and how much of the
   plane is far apart, and the two readings separate:
     - x or y in the outer AGREE_EDGE rows/columns, with `over` small -> an edge;
     - `over` in the thousands -> the plane itself is offset or scaled.
   The s_... arms of the product are all 320x320, which is why the coordinates are
   taken in YOLO_IN_W / YOLO_IN_H and no width argument is carried. */
#define AGREE_OVER  128u   /* "visibly different byte" - the count threshold */
#define AGREE_EDGE  2u     /* outer frame this reports as "along an edge" */

typedef struct
{
    uint32_t x;      /* the pixel owning the maximum, in 320x320 input coords */
    uint32_t y;
    uint32_t ch;     /* 0 R, 1 G, 2 B - the channel owning it */
    uint32_t av;     /* the two values at that channel: a's and b's, 0..255 */
    uint32_t bv;
    uint32_t over;   /* pixels with some channel further apart than AGREE_OVER */
    uint32_t edge;   /* of those, how many lie in the outer AGREE_EDGE frame */
} agree_worst_t;

/**
 *\*\name    agree_stats.
 *\*\fun     Sum / swapped-sum / max channel difference of two RGB888 images.
 *
 * Factored out so that every comparison this file makes - the blit against the
 * CPU, the filtered result against the CPU, and the pair's two arms against each
 * other - is the same arithmetic.  Two hand-written copies of "how far apart are
 * they" would make the numbers look comparable without being so, and here the
 * numbers are the entire verdict.
 *
 * @param  a   : n*3 bytes
 * @param  b   : n*3 bytes
 * @param  sum : receives the summed absolute channel difference
 * @param  swp : receives the same sum with a's red and blue exchanged, which is
 *              what tells a channel-order fault apart from a real difference
 * @param  max : receives the largest single-channel difference (in and out)
 * @param  loc : where the maximum sits, and how much of the plane is far apart;
 *              pass NULL to skip it.  max alone cannot tell an edge artifact
 *              from a whole-plane offset - see agree_worst_t.
**/
static void agree_stats(const uint8_t* a, const uint8_t* b, uint32_t n,
                        uint32_t* sum, uint32_t* swp, uint32_t* max,
                        agree_worst_t* loc)
{
    uint32_t s = 0u;
    uint32_t w = 0u;
    uint32_t m = 0u;
    uint32_t i;
    uint32_t wi = 0u;   /* the pixel that owns m */
    uint32_t over = 0u;
    uint32_t edge = 0u;

    for (i = 0u; i < n; i++)
    {
        const uint32_t ar = (uint32_t)a[0];
        const uint32_t ag = (uint32_t)a[1];
        const uint32_t ab = (uint32_t)a[2];
        const uint32_t br = (uint32_t)b[0];
        const uint32_t bg = (uint32_t)b[1];
        const uint32_t bb = (uint32_t)b[2];
        const uint32_t g0 = chan_absdiff(ar, br);
        const uint32_t g1 = chan_absdiff(ag, bg);
        const uint32_t g2 = chan_absdiff(ab, bb);

        s += g0 + g1 + g2;
        w += chan_absdiff(ar, bb) + g1 + chan_absdiff(ab, br);

        if (g0 > m) { m = g0; wi = i; if (loc != 0) { loc->ch = 0u; loc->av = ar; loc->bv = br; } }
        if (g1 > m) { m = g1; wi = i; if (loc != 0) { loc->ch = 1u; loc->av = ag; loc->bv = bg; } }
        if (g2 > m) { m = g2; wi = i; if (loc != 0) { loc->ch = 2u; loc->av = ab; loc->bv = bb; } }

        /* Strict > on the three tests above means a tie keeps the first pixel,
           so the coordinates are reproducible frame to frame.  This count is
           deliberately grouped by PIXEL, not by channel: how many pixels are
           wrong is the question, and a channel-count would triple it. */
        if (loc != 0)
        {
            uint32_t g = g0;
            if (g1 > g) { g = g1; }
            if (g2 > g) { g = g2; }
            if (g > AGREE_OVER)
            {
                const uint32_t fx = i % YOLO_IN_W;
                const uint32_t fy = i / YOLO_IN_W;

                over++;
                if ((fx < AGREE_EDGE) || (fx >= (YOLO_IN_W - AGREE_EDGE)) ||
                    (fy < AGREE_EDGE) || (fy >= (YOLO_IN_H - AGREE_EDGE)))
                {
                    edge++;
                }
            }
        }

        a += 3;
        b += 3;
    }

    if (loc != 0)
    {
        loc->x = wi % YOLO_IN_W;
        loc->y = wi / YOLO_IN_W;
        loc->over = over;
        loc->edge = edge;
    }

    *sum = s;
    *swp = w;
    *max = m;
}

/**
 *\*\name    unpack_rgb565_to_rgb888.
 *\*\fun     320x320 RGB565 -> 320x320 RGB888, no resampling.
 *\*\param   src  GPU_SCALE_DST, which the GPU has already filled
 *\*\param   dst  320*320*3 bytes
 *
 * The GPU half of the grab.  Everything in downsample_rgb565_to_rgb888() above
 * - the four unpacks, the six blends and the fixed-point stepping - is what the
 * 2D engine has just done, so what is left on this side is the format change
 * the engine cannot do: it writes whatever format the driver's framebuffer is
 * in, and that is RGB565.  See gpu.h.
**/
static void unpack_rgb565_to_rgb888(const uint16_t* src, uint8_t* dst)
{
    uint32_t i;
    uint32_t n = YOLO_IN_W * YOLO_IN_H;

    for (i = 0u; i < n; i++)
    {
        uint32_t r;
        uint32_t g;
        uint32_t b;

        rgb565_unpack(src[i], &r, &g, &b);
        dst[0] = (uint8_t)r;
        dst[1] = (uint8_t)g;
        dst[2] = (uint8_t)b;
        dst += 3;
    }
}

/**
 *\*\name    camera_unpack_full_rgb888.
 *\*\fun     800x480 RGB565 -> 800x480 RGB888, no resampling.
 *\*\param   src565  the frame buffer to read
 *\*\param   dst     800*480*3 bytes
 *
 * unpack_rgb565_to_rgb888() above is the same loop with YOLO_IN_W*YOLO_IN_H
 * pixels; this one exists because the cascade's crop source has to be the full
 * frame, not the 320x320 the model sees.  Two separate functions rather than a
 * parameterised one because the two counts differ (YOLO_IN_W*YOLO_IN_H is
 * 102,400, this is 384,000) and the only thing they share is one call to
 * rgb565_unpack() - the loop is the cheap part.
 *
 * The channel order is r,g,b whatever CAM_RGB565_SWAP_RB says: that macro picks
 * which of the sensor's two 5-bit fields is red, and rgb565_unpack() has
 * already applied it by the time anything is stored here.  So the frozen frame
 * matches the 320x320 buffers the model was given, and the crop path does not
 * need to know the macro exists.
 *
 * Timing, because it decides whether the caller needs to stop the DVP: this is
 * 384,000 unpacks and 1,152,000 stores, i.e. LESS arithmetic than
 * downsample_rgb565_to_rgb888() does for the 320x320 grab (it unpacks four
 * pixels and does six blends per output pixel, 102,400 times over).  That grab
 * reads a buffer the DVP will not touch again for a whole frame period and does
 * not stop it; the same argument holds here with room to spare.
**/
bool camera_unpack_full_rgb888(uint32_t src565, uint8_t* dst)
{
    const uint16_t* src = (const uint16_t*)src565;
    uint32_t        i;
    const uint32_t  n = CAM_W * CAM_H;

    if ((src565 == 0u) || (dst == (uint8_t*)0))
    {
        return false;
    }

    for (i = 0u; i < n; i++)
    {
        uint32_t r;
        uint32_t g;
        uint32_t b;

        rgb565_unpack(src[i], &r, &g, &b);
        dst[0] = (uint8_t)r;
        dst[1] = (uint8_t)g;
        dst[2] = (uint8_t)b;
        dst += 3;
    }

    return true;
}

/* Scratch for the two measurements below - the boot agreement check and the
   trigger-time path A/B (camera_grab_pair): 320*320*3 = 307,200 bytes.
   Deliberately a hard SDRAM address rather than a static array - a static would
   land in ZI-data, which is already 2.5 MB and would have to be made to fit in
   the scatter file, and this buffer only exists to answer one question once.
   No MPU region covers it, so it is Device memory: uncached, which is what a
   buffer written once and read once wants.

   0xC0380000 is just past the two camera frame buffers, which end at
   0xC0377000.  It was 0xC1000000 first - the first byte above the GPU slot -
   and that produced a mean difference of 109 with swap equal to it to the
   digit, which is the signature of a buffer holding one constant value: if
   every byte of one side is the same, exchanging red and blue cannot change
   any difference.  0xC1000000 is exactly 16 MB into the SDRAM decode, and
   "32 MB decoded" is a statement about the address decoder, not about the
   part fitted; if the part is 16 MB then that address is the first one that
   does not exist, and writes to it go nowhere.  0xC0380000 is certainly
   inside the part, because the frame buffers immediately below it work.
   The check in the probe below is what stops that class of mistake from
   presenting itself as the GPU drawing the wrong picture. */
#define CAM_ALT_BUF   ((uint8_t*)0xC0380000u)

/**
 *\*\name    camera_grab_pair.
 *\*\fun     Fill both 320x320 RGB888 buffers from ONE snapshotted frame.
 *
 * The path A/B: does the GPU front end make the model read a worse plate than
 * the CPU front end does?
 *
 * Nothing measured so far can answer that.  `gpu agree mean 4 ok 1` is a PIXEL
 * criterion - it says the two pictures are close, not that the model decodes the
 * same plate from them - and the model's own output is unstable frame to frame
 * on a stationary plate (four triggers of one build gave three different
 * strings), so comparing two paths across two triggers would be measuring the
 * scene rather than the path.
 *
 * Hence one snapshot, two conversions, both handed back for the caller to run
 * the model on.  s_ready_addr is taken once, here, for the same reason
 * camera_gpu_agreement_probe() takes it once: two calls to
 * camera_grab_rgb888() would each snapshot for themselves and could land one
 * frame apart, which is exactly the variance this is trying to remove.
 *
 * The CPU result is written into the caller's buffer and the GPU result into
 * CAM_ALT_BUF, whose only other user is the boot agreement probe - so this
 * borrows that buffer after boot rather than taking a third 300 KB.
 *
 * @param  dst_cpu : 320*320*3 bytes, receives the CPU bilinear's output
 * @retval the GPU result, or NULL if the GPU path is not available (not up, not
 *         latched as faithful at boot, or the blit refused) - in which case
 *         dst_cpu is still valid and there is simply nothing to compare with
 */
/* What camera_grab_pair()'s GPU arm actually did, so the A/B line carries the
   configuration that produced it instead of the reader having to know the build.
   This exists because a stale copy of that fact - the pair's own comment, still
   saying the filter was off - is what made the first run of this A/B measure the
   unfiltered blit while the product ran the filtered one. */
static bool s_pair_filtered = false;

#if GPU_ENABLE
bool camera_pair_filtered(void)
{
    return s_pair_filtered;
}
#endif  /* GPU_ENABLE - the pair A/B does not exist without the GPU arm */

/**
 *\*\name    pair_report_diff.
 *\*\fun     Print how far apart the pair's two images are, on a live frame.
 *
 * Both front ends are in memory at this instant, on the same frame, with
 * whatever the camera is pointing at - and that is the point.  The boot
 * agreement probe has to take its picture whenever the camera happens to be
 * ready, so its number is about a frame that may hold no plate at all; this one
 * is taken on the frame the model is about to be run on.
 *
 * It answers the question the two `pair` box counts raise and cannot settle on
 * their own.  "The CPU arm reads a plate and the GPU arm reads nothing" has two
 * explanations that call for opposite fixes:
 *
 *   - the two pictures are far apart -> the front end is what broke, and the
 *     mean/max here says so before the next probe is designed;
 *   - the two pictures agree to single digits -> the front end is faithful and
 *     the model is marginal on this plate, so the next thing to look at is not
 *     the GPU at all.
 *
 * Cheap enough to run every time: 307200 byte comparisons, ~1 ms against a
 * 4.2 s inference, and it is only on the 'p' key.
 *
 * @param  cpu : the CPU bilinear's output
 * @param  gpu : the GPU arm's output
**/
static void pair_report_diff(const uint8_t* cpu, const uint8_t* gpu)
{
    const uint32_t n = YOLO_IN_W * YOLO_IN_H;
    uint32_t sum = 0u;
    uint32_t swp = 0u;
    uint32_t max = 0u;
    agree_worst_t worst = { 0u, 0u, 0u, 0u, 0u, 0u, 0u };

    agree_stats(cpu, gpu, n, &sum, &swp, &max, &worst);

    sprintf(Message, "pair diff mean %lu max %lu swap %lu\r\n",
            (unsigned long)(sum / (n * 3u)), (unsigned long)max,
            (unsigned long)(swp / (n * 3u)));
    UART_SendString(Message);

    /* Which half of the plane owns that max, and how much of the plane is far
       apart at all.  Six frames have given the same 198, so the number is the
       operators' and not the scene's - these two readings are what say which
       kind of scene-independent fault it is.  cpu is a's side, gpu is b's. */
    sprintf(Message, "pair worst x%lu y%lu ch%lu cpu %lu gpu %lu over%lu edge%lu\r\n",
            (unsigned long)worst.x, (unsigned long)worst.y, (unsigned long)worst.ch,
            (unsigned long)worst.av, (unsigned long)worst.bv,
            (unsigned long)worst.over, (unsigned long)worst.edge);
    UART_SendString(Message);
}

/* WHICH PASS COSTS THE PICTURE ITS LEFT COLUMN.

   `pair worst` reports x0 ... gpu 0 beside a CPU value of 189, on every frame,
   and over == edge == ~323 - i.e. the whole disagreement is in the outer
   two-pixel frame and the plane is dead at the left edge.  That is the pair's
   and not the scene's, but the CPU-vs-GPU comparison cannot say WHICH of the two
   passes did it, because it never sees the intermediate at all.

   This stage's operator can be read where it matters.  Unlike pass 1 of the
   library's, it does not write its own source, so all three planes are intact
   when gpu_filter_frame() returns: GPU_SCALE_DST is the blit it read,
   GPU_SEP_TMP is pass 1's output, GPU_SCALE_CONV is pass 2's.  The mean field of
   the first three columns of each bisects the pair in one reading -

     - tmp's column 0 already flat        -> pass 1 is where the column dies;
     - tmp's level and conv's flat        -> pass 2;
     - both level and conv's level        -> neither, and the 0 is elsewhere.

   - the columns beside it are printed with each so "one column" and a
   two-column band cannot be read as the same thing, and so that a one-texel
   offset in a pass can be told from a flat column (see the note at the print
   itself).  Field units: red and blue are 5 bits, green 6, so the mean over
   the three is 0..63.

   A library arm could not be instrumented this way at all - its pass 1 runs in
   place and would have destroyed the reference on the first line. */
static void pair_report_edges(void)
{
    const volatile uint16_t* const planes[3] = {
        (const volatile uint16_t*)GPU_SCALE_DST,
        (const volatile uint16_t*)GPU_SEP_TMP,
        (const volatile uint16_t*)GPU_SCALE_CONV
    };
    uint32_t m[3][3];
    uint32_t pi;

    for (pi = 0u; pi < 3u; pi++)
    {
        uint32_t s0 = 0u;
        uint32_t s1 = 0u;
        uint32_t s2 = 0u;
        uint32_t y;

        for (y = 0u; y < GPU_SCALE_H; y++)
        {
            const uint32_t v0 = (uint32_t)planes[pi][(y * GPU_SCALE_W) + 0u];
            const uint32_t v1 = (uint32_t)planes[pi][(y * GPU_SCALE_W) + 1u];
            const uint32_t v2 = (uint32_t)planes[pi][(y * GPU_SCALE_W) + 2u];

            s0 += ((v0 >> 11) & 31u) + ((v0 >> 5) & 63u) + (v0 & 31u);
            s1 += ((v1 >> 11) & 31u) + ((v1 >> 5) & 63u) + (v1 & 31u);
            s2 += ((v2 >> 11) & 31u) + ((v2 >> 5) & 63u) + (v2 & 31u);
        }

        m[pi][0] = s0 / (GPU_SCALE_H * 3u);
        m[pi][1] = s1 / (GPU_SCALE_H * 3u);
        m[pi][2] = s2 / (GPU_SCALE_H * 3u);
    }

    /* Three columns, not two, because pass 2 is a COLUMN pass: destination
       column c is a function of tmp's column c and of nothing else in tmp.  So
       `conv c` against `tmp c` IS the per-column identity, and a UV origin that
       is one texel out shows up as conv[0] matching tmp[1] - which two columns
       could not distinguish from a column that merely happens to agree. */
    sprintf(Message, "pair edge dst %lu %lu %lu tmp %lu %lu %lu conv %lu %lu %lu\r\n",
            (unsigned long)m[0][0], (unsigned long)m[0][1], (unsigned long)m[0][2],
            (unsigned long)m[1][0], (unsigned long)m[1][1], (unsigned long)m[1][2],
            (unsigned long)m[2][0], (unsigned long)m[2][1], (unsigned long)m[2][2]);
    UART_SendString(Message);
}

/* The whole A/B exists to measure a GPU front end against the CPU one, so with
   GPU_ENABLE 0 there is nothing for it to be an A/B OF - it is not compiled, and
   neither is its caller in main.c (the `p` key). */
#if GPU_ENABLE
const uint8_t* camera_grab_pair(uint8_t* dst_cpu)
{
    const uint32_t addr = s_ready_addr;

    s_pair_filtered = false;
    downsample_rgb565_to_rgb888((const uint16_t*)addr, dst_cpu);

    if (!s_gpu_grab_ok ||
        !gpu_scale_frame(addr, (uint32_t)CAM_W, (uint32_t)CAM_H, (uint32_t)CAM_W))
    {
        return NULL;
    }

    /* The same pointer the grab itself uses, INCLUDING the filter: this A/B
       exists to measure the front end the product has, so leaving a stage out
       answers a question nobody asked.

       That is the opposite of what this said when written, and the change is not
       a reversal of the reasoning - it is the same reasoning with the filter
       flipped.  With the filter OFF the unfiltered blit WAS the shipped front
       end, so filtering one arm would have compared two changes at once; with
       GPU_FILTER_ENABLE 1 both changes ship together and the pair has to follow
       them together.  Flip the macro and this arm flips with it, which is the
       point. */
#if GPU_FILTER_ENABLE
    /* No gate of its own, and that is the point: this arm exists to measure the
       front end the PRODUCT has, so it follows the product's stage-3 decision
       rather than making its own.  The decision is inside gpu_filter_frame(),
       which is where the operator swap lives - this arm takes whichever operator
       the product is on, so the A/B is same-build, same-boot, same-scene. */
    if (s_gpu_filter_ok && gpu_filter_frame())
    {
        s_pair_filtered = true;
        unpack_rgb565_to_rgb888((const uint16_t*)GPU_SCALE_CONV, CAM_ALT_BUF);
        pair_report_diff(dst_cpu, CAM_ALT_BUF);
        /* Before the next frame can overwrite anything - see the function.
           Guarded, because the two arms do not leave the same buffers behind:
           the LIBRARY's pass 1 runs in place and destroys GPU_SCALE_DST, so the
           line would compare a live plane against a dead one and read as if
           pass 1 were the fault.  With the gate off there is nothing to look
           at, so nothing is printed. */
        if (g_gpu_front_sep_on != 0u)
        {
            pair_report_edges();
        }
        return CAM_ALT_BUF;
    }
#endif

    /* Filter not applicable, not trusted, or it refused: GPU_SCALE_DST still
       holds the blit's output, because what destroys it is pass 1 of the filter
       and pass 1 did not run.  Same fallback camera_grab_rgb888() takes, for the
       same reason - and with `f0` on the caller's line, this arm is the one the
       product is shipping too. */
    unpack_rgb565_to_rgb888((const uint16_t*)GPU_SCALE_DST, CAM_ALT_BUF);

    pair_report_diff(dst_cpu, CAM_ALT_BUF);

    return CAM_ALT_BUF;
}
#endif  /* GPU_ENABLE - end of the pair A/B (camera_grab_pair) */

/**
 *\*\name    camera_gpu_agreement_probe.
 *\*\fun     Run both grabs over the same frame and report how far apart the
 *\*\        results are, and how long each took.
 *
 * This is the check that decides whether the GPU branch can be switched on, and
 * it is a measurement rather than an assertion because the two paths cannot
 * agree bit-for-bit even when both are right:
 *
 *   - the x step differs.  The CPU walks the source at exactly 2.5 pixels per
 *     output pixel (DET_STEP_X = 640 in 8.8).  The GPU's scale is 103/256, so
 *     it walks at 2.4922 - a 0.3% difference that shifts the sample position by
 *     up to a source pixel at the right edge.
 *   - the CPU unpacks to 8 bits and then blends; the GPU filters in RGB565 and
 *     the result is unpacked afterwards.  Each channel rounds once on either
 *     path, in a different order.
 *
 * So the verdict is a mean channel difference, not equality.  Both a working
 * path and a channel-swapped one produce a large maximum - an edge is an edge -
 * and only the mean separates them, which is why both are printed, along with
 * the mean computed the other way round.
 *
 * Waits for a new frame first: at boot the DVP may not have completed one, and
 * comparing two conversions of uninitialised SDRAM would mean nothing.
 *
 * @retval true if the GPU paths ran at all (not a statement about agreement)
 */
#if GPU_ENABLE
/* The agreement probe's SUCCESS lines go through here; its failure lines do not.

   With BOOT_VERBOSE 0 (the default) the boot prints one line, `boot ok`, and the
   six lines this probe used to report on a passing boot are gone: `px ...`,
   `cpu ... blit ... unpack ...`, `mean ... ok ...`, `filt mean ...`,
   `filt vs blit ...`, `gpu sep t ...`.  The board now scans and reports on its
   own every few seconds, and a once-per-power-up burst of numbers only pushes
   those result lines apart.

   THE PROBE ITSELF STILL RUNS, unchanged.  It is the only place s_gpu_grab_ok /
   s_gpu_filter_ok are latched, i.e. the only thing that decides whether the grab
   front end runs on the GPU or on the CPU - see the note at its call site in
   main.c.  Only its reporting is gated.

   Which line takes which route is a judgement, and the rule is: is this line a
   reading taken when everything worked, or is it bad news?  `mean ... ok 0`
   carries the verdict, and ok 0 silently moves the front end off the GPU, so
   that line is NOT routed here unconditionally - see the branch at the point it
   is built.  Everything that only ever prints on a passing boot is.

   The sprintf calls stay where they are: saving a few microseconds at boot is
   worth nothing and wrapping them too would only make the diff harder to read. */
static void agree_log(const char* s)
{
#if BOOT_VERBOSE
    UART_SendString(s);
#else
    (void)s;
#endif
}

bool camera_gpu_agreement_probe(uint8_t* dst_cpu)
{
    static const char* const k_lbl = "gpu agree";
    uint8_t* dst_gpu = CAM_ALT_BUF;
    const uint32_t n = YOLO_IN_W * YOLO_IN_H;
    uint32_t start;
    uint32_t addr;
    uint32_t sum = 0u;
    uint32_t sum_swap = 0u;
    uint32_t max = 0u;
    uint32_t mean = 0u;
    uint32_t t0;
    uint32_t t_cpu;
    uint32_t t_blit;
    uint32_t t_unpack;
    uint32_t t_filt;

    /* Prove the scratch buffer is real memory before trusting anything read
       out of it.  It is a hard address, and a dead address answers with a
       constant, which in the comparison below looks exactly like the GPU
       producing the wrong picture - a mean difference in the hundreds with the
       swapped mean identical to it.  Two hundred microseconds of pattern here
       saves chasing that. */
    {
        uint32_t k;
        uint32_t bad = 0u;

        for (k = 0u; k < 256u; k++)
        {
            dst_gpu[k] = (uint8_t)(k ^ 0x5Au);
        }
        for (k = 0u; k < 256u; k++)
        {
            if (dst_gpu[k] != (uint8_t)(k ^ 0x5Au))
            {
                bad++;
            }
        }
        if (bad != 0u)
        {
            sprintf(Message, "%s buf BAD %lu\r\n", k_lbl, (unsigned long)bad);
            UART_SendString(Message);
            return false;
        }
    }

    /* Wait for a frame the DVP has actually finished, but not forever: a DVP
       that never delivers must not hang the boot, for the same reason a GPU
       failure must not stop detection.  One second is about thirty frame
       periods at this resolution, so it is generous for a live camera and still
       bounded for a dead one. */
    start = s_frames;
    t0 = DWT->CYCCNT;
    while ((s_frames == start) && ((DWT->CYCCNT - t0) < 600000000u))
    {
        /* spin */
    }
    if (s_frames == start)
    {
        UART_SendString("gpu agree NO FRAME\r\n");
        return false;
    }

    /* Both conversions read the SAME buffer, snapshotted once - not
       camera_grab_rgb888(), which would take its own snapshot and could take
       the next frame, leaving the two results one frame apart. */
    addr = s_ready_addr;

    t0 = DWT->CYCCNT;
    downsample_rgb565_to_rgb888((const uint16_t*)addr, dst_cpu);
    t_cpu = (DWT->CYCCNT - t0) / 600u;

    t0 = DWT->CYCCNT;
    if (!gpu_scale_frame(addr, (uint32_t)CAM_W, (uint32_t)CAM_H, (uint32_t)CAM_W))
    {
        UART_SendString("gpu agree NO GPU\r\n");
        return false;
    }
    t_blit = (DWT->CYCCNT - t0) / 600u;

    t0 = DWT->CYCCNT;
    unpack_rgb565_to_rgb888((const uint16_t*)GPU_SCALE_DST, dst_gpu);
    t_unpack = (DWT->CYCCNT - t0) / 600u;

    /* First pixel of each, before the comparison below.  Three values that
       separate three different faults: what the CPU produced, what the unpack
       wrote into the scratch buffer, and what the GPU actually left in
       GPU_SCALE_DST.  If the middle one is constant the buffer is bad; if the
       last one is, the blit drew nothing and the buffer was never at fault.
       Without this the two are indistinguishable - both show up as a large mean
       difference. */
    sprintf(Message, "%s px c%02X%02X%02X g%02X%02X%02X s%02X%02X%02X\r\n", k_lbl,
            (unsigned)dst_cpu[0], (unsigned)dst_cpu[1], (unsigned)dst_cpu[2],
            (unsigned)dst_gpu[0], (unsigned)dst_gpu[1], (unsigned)dst_gpu[2],
            (unsigned)(((const uint8_t*)GPU_SCALE_DST)[0]),
            (unsigned)(((const uint8_t*)GPU_SCALE_DST)[1]),
            (unsigned)(((const uint8_t*)GPU_SCALE_DST)[2]));
    agree_log(Message);

    /* agree_stats() walks its own copies of the pointers, so dst_cpu and
       dst_gpu are still at the start of their buffers for the filter leg below.
       The loop this replaces advanced both of them, which is why that leg could
       not previously have been added here without reintroducing the addresses of
       the last pixels as if they were the first. */
    agree_stats(dst_cpu, dst_gpu, n, &sum, &sum_swap, &max, 0);

    sprintf(Message, "%s cpu %lu blit %lu unpack %lu\r\n", k_lbl,
            (unsigned long)t_cpu, (unsigned long)t_blit, (unsigned long)t_unpack);
    agree_log(Message);

    /* mean over all 3n channels, so it is comparable with max.  Both a working
       path and a channel-swapped one have a large max - an edge is an edge -
       so the mean is the number the verdict rests on, and the swapped mean is
       what tells the two apart. */
    mean = sum / (n * 3u);

    sprintf(Message, "%s mean %lu max %lu swap %lu ok %u\r\n", k_lbl,
            (unsigned long)mean, (unsigned long)max,
            (unsigned long)(sum_swap / (n * 3u)),
            (unsigned)(mean <= CAM_GPU_AGREE_MAX));
    /* THE VERDICT LINE, and the one line of the blit leg that is NOT routed
       through agree_log unconditionally.  `ok 1` is a reading taken on a boot
       where everything worked, so it is silenced; `ok 0` is the news that the
       grab front end is about to move off the GPU and run the CPU bilinear
       instead - the explanation for every result line after it - so it prints
       whatever BOOT_VERBOSE says.  The latch below is computed from the same
       comparison either way. */
    if (mean <= CAM_GPU_AGREE_MAX) { agree_log(Message); }
    else                          { UART_SendString(Message); }

    /* The latch that decides which path the trigger loop takes, set here so
       that it is set on the same evidence that was printed. */
    s_gpu_grab_ok = (mean <= CAM_GPU_AGREE_MAX);
    if (s_gpu_grab_ok && (sum_swap < sum))
    {
        /* Exchanging red and blue cannot make the two agree when they already
           do; a swapped pair that scores BETTER is the channel-order fault
           wearing the right answer's clothes.  Not expected, but it costs one
           comparison to refuse it. */
        UART_SendString("gpu agree ORDER?\r\n");
        s_gpu_grab_ok = false;
    }

    /* ---- THE SHAPE OF THE DESTINATION -------------------------------------
       `s%02X%02X%02X` above is the FIRST TEXEL of GPU_SCALE_DST, and it has
       read 0x07C3 on every boot since 2026-09-15 with the CPU arm reading a
       different picture each time.  That value is already named in this
       repository: GPU_CROP_STUCK, the hard constant the library's colorkey
       PRE-FILLS its (src_w x src_h) box with, IGNORING the colour argument
       (measured - the 0xF81F passed as `colour` appears zero times in the whole
       destination, gpu.h:365-368).  So the constant is not a fault by itself:
       it means the texel read is one the blit did NOT cover.

       A 20 ms re-read of the same three bytes settled the older suspect - the
       value does not change, so nothing is actively rewriting that region and
       the camera-buffer reading of it is dead.  What that cannot answer is how
       MUCH of the plane is pre-fill and where the image starts, and that is
       what separates the two readings `mean 117` cannot: a blit that painted a
       318x320 image in the WRONG PLACE (a misplacement leaves a mean far below
       117) versus a blit that painted almost nothing, leaving the whole plane
       pre-fill (which is what a mean of 117 with max 255 looks like).

       So count the plane - pre-fill, the fill colour, everything else - and
       take the first and last texel that are neither, which bracket the image.
       all-stuck is a blit that did not draw; a first-non-stuck near the origin
       is a blit that drew nearly where it was asked to.

       Read-only, runs after everything that can decide anything, and the two
       bracket values mean nothing when `other` is 0.

       THE WHOLE BLOCK, WALK INCLUDED, IS BEHIND BOOT_VERBOSE (2026-10-12).
       That is one step further than silencing its two lines, and it is the right
       step for this block specifically: it is the only thing in the probe that
       decides nothing and costs something - a 102400-texel walk of uncached
       Device memory (see the note on gpu_scale_crop_probe() below for what a
       million of those cost) whose entire output is the two lines.  A probe that
       prints nothing and decides nothing is not a probe, it is boot time. */
#if BOOT_VERBOSE
    {
        const volatile uint16_t* p = (const volatile uint16_t*)GPU_SCALE_DST;
        const uint32_t n_t = GPU_SCALE_W * GPU_SCALE_H;
        const volatile uint16_t* srcp = (const volatile uint16_t*)addr;
        uint32_t stuck = 0u;
        uint32_t fill = 0u;
        uint32_t other = 0u;
        uint32_t fx = 0u;
        uint32_t fy = 0u;
        uint32_t lx = 0u;
        uint32_t ly = 0u;
        uint32_t i;

        for (i = 0u; i < n_t; i++)
        {
            const uint16_t v = p[i];

            if (v == (uint16_t)GPU_CROP_STUCK)
            {
                stuck++;
                continue;
            }

            if (v == (uint16_t)GPU_SCALE_BG)
            {
                fill++;
                continue;
            }

            other++;

            if (other == 1u)
            {
                fx = i % GPU_SCALE_W;
                fy = i / GPU_SCALE_W;
            }

            lx = i % GPU_SCALE_W;
            ly = i / GPU_SCALE_W;
        }

        sprintf(Message, "gpu dst3 stuck %lu fill %lu other %lu first %lu,%lu last %lu,%lu\r\n",
                (unsigned long)stuck, (unsigned long)fill, (unsigned long)other,
                (unsigned long)fx, (unsigned long)fy,
                (unsigned long)lx, (unsigned long)ly);
        UART_SendString(Message);

        /* Two source texels for reference - the corner and the centre - so the
           destination's painted region can be read against what it sampled. */
        sprintf(Message, "gpu dst4 src %04X %04X dst %04X %04X\r\n",
                (unsigned)srcp[0],
                (unsigned)srcp[(240u * (uint32_t)CAM_W) + 400u],
                (unsigned)p[0],
                (unsigned)p[(160u * GPU_SCALE_W) + 160u]);
        UART_SendString(Message);
    }
#endif  /* BOOT_VERBOSE - dst3 / dst4 */

    /* The second half of the shipped front end - and the reason everything above
       was not the gate it looked like.  Every number so far is about
       GPU_SCALE_DST, which is the BLIT's output; the grab then runs
       gpu_filter_frame() over it and unpacks GPU_SCALE_CONV instead.  So a board
       could print `gpu agree mean 5 max 199 swap 10 ok 1` and still hand the
       model a filtered picture that no boot measurement had ever looked at -
       while `ok 1` was believed, because it is about the stage that is not in
       question.

       gpu_filter_probe() does not cover this either, and it is not a defect in
       that probe: it runs the engine on a synthetic grating with an IDENTITY
       kernel, which answers "is the convolve's geometry right at 320x320", not
       "is the shipped 3-tap kernel over a real frame the picture the CPU path
       produces".  Both are worth asking; only one of them was being asked.

       This leg runs the shipped kernel over the shipped blit and measures the
       result against the CPU image with the same arithmetic and the same bound as
       the blit leg above, so the two numbers are directly comparable and a
       filtered mean close to the blit's mean is the reading that says the stage
       is transparent.

       WHERE IT SITS IS LOAD-BEARING, in both directions:

         - After every readback of GPU_SCALE_DST above, because the library call
           filters its source and the blit's own comparison has to be finished
           before its input is disturbed.
         - Before gpu_scale_crop_probe() below, because that probe paints its own
           pattern into GPU_SCALE_DST and draws over it - leave the filter leg
           until after it and the filter would be run over the crop probe's
           leftovers instead of over the blit's output.

       So this is the last thing that may read GPU_SCALE_DST as the blit left it,
       and the first of the two probes that consume it.  The blit's verdict is
       already latched by the time this runs, which is also the point: a filter
       failure must not be able to revoke a blit that passed.  It costs the GPU's
       own stage when it fails and nothing else.

       IT CALLS gpu_filter_frame_NEW(), not gpu_filter_frame(), and that is the
       one place the two entries differ.  The product's entry point dispatches on
       g_gpu_front_sep_on; this leg must not, because it is the reading the switch
       is waiting on.  With the switch at its default the gate would otherwise
       send this leg to the LIBRARY and the boot would report a verdict on the
       operator nobody is deciding about - and it would report it under the same
       label as the new one. */
    if (s_gpu_grab_ok && gpu_filter_frame_new())
    {
        uint32_t f_sum = 0u;
        uint32_t f_swap = 0u;
        uint32_t f_max = 0u;
        uint32_t f_mean;

        t0 = DWT->CYCCNT;
        unpack_rgb565_to_rgb888((const uint16_t*)GPU_SCALE_CONV, dst_gpu);
        t_filt = (DWT->CYCCNT - t0) / 600u;

        agree_stats(dst_cpu, dst_gpu, n, &f_sum, &f_swap, &f_max, 0);
        f_mean = f_sum / (n * 3u);

        sprintf(Message, "%s filt mean %lu max %lu swap %lu unpack %lu ok %u\r\n",
                k_lbl, (unsigned long)f_mean, (unsigned long)f_max,
                (unsigned long)(f_swap / (n * 3u)), (unsigned long)t_filt,
                (unsigned)(f_mean <= CAM_GPU_AGREE_MAX));
        /* Same split as the blit leg's verdict line above, for the same reason:
           ok 1 is a reading from a passing boot (silenced), ok 0 is the filter
           stage failing its own check, and that is the last thing that should
           have to be inferred from a board that went quiet. */
        if (f_mean <= CAM_GPU_AGREE_MAX) { agree_log(Message); }
        else                             { UART_SendString(Message); }

        /* ---- WHAT THE FILTER DID, with the blit as its own reference --------

           `filt mean` above is the difference from the CPU image, which answers
           "is this picture usable" but not "what did the filter DO" - and the two
           questions come apart exactly when the reading is bad, which is when the
           second one is needed.  A stage 3 whose gains are wrong by a factor can
           only be told from a stage 3 whose geometry is wrong by a line that does
           not go through the CPU at all.

           So this compares GPU_SCALE_CONV against GPU_SCALE_DST: the filtered
           picture against the UNFILTERED one it was made from.  Both are RGB565,
           so the difference is counted in FIELD units - 31 per pixel for red and
           blue and 63 for green, i.e. an average of 41.7 per pixel over the three
           channels when a plane moves the whole way.  (An earlier cut of this
           comment quoted the bands as if the counts were 8-bit, which made a
           reading of 17 look like 7% of full scale when it is 41%.  Do not band
           these numbers by eye - use the two means printed beside them.)

           `d` is printed together with the mean of each plane, and the pair is
           what makes the line self-calibrating, because at an end the numbers
           coincide exactly:

             - d == blit's own mean, conv ~ 0   the plane is BLACK: the destination
               field truncated away.  (d = SUM|0 - b| = SUM b identically.)
             - d == conv's own mean, blit ~ 0   the plane is WHITE: the row sums
               are railing at the field's top.
             - a few counts below both           near-transparent, which is what a
               unity-gain pair over an already-downsampled picture is (neighbours
               that close differ by little) - the correct reading.
             - a fraction of either, in between  a real blur, or a gain off by a
               factor.

           Read on 2026-09-29 as `filt vs blit 17`, with the shipped gains at
           gain1 = 1/S: 17 was the BLIT's own field mean, so the filtered plane was
           zero almost everywhere and the two gains were 8x apart - pass 1's
           multiplicand is a 565 field's rep(C)/8, not the 8-bit texel, so pass 1
           needed the same 8/S pass 2 already had.  That is what the extra two
           numbers on the line are for: with `conv` printed, 17 is a verdict, not
           a guess.

           The read is valid only because THIS stage's operator does not write its
           source: the library's pass 1 ran in place and destroyed GPU_SCALE_DST,
           while blit_separable_convolve_new() reads it and writes GPU_SEP_TMP and
           GPU_SCALE_CONV.  That is also why this line can sit here and nowhere
           else - everything after it in this probe may paint over the blit's
           output.  Cost is one walk of two 200 KB planes, the same walk `gpu dst3`
           already pays below. */
        {
            const volatile uint16_t* pf = (const volatile uint16_t*)GPU_SCALE_CONV;
            const volatile uint16_t* pb = (const volatile uint16_t*)GPU_SCALE_DST;
            uint32_t d = 0u;
            uint32_t pc = 0u;
            uint32_t pbl = 0u;
            uint32_t i;

            for (i = 0u; i < n; i++)
            {
                const int32_t cr = (int32_t)((pf[i] >> 11) & 31u);
                const int32_t cg = (int32_t)((pf[i] >> 5) & 63u);
                const int32_t cb = (int32_t)(pf[i] & 31u);
                const int32_t br = (int32_t)((pb[i] >> 11) & 31u);
                const int32_t bg = (int32_t)((pb[i] >> 5) & 63u);
                const int32_t bb = (int32_t)(pb[i] & 31u);
                int32_t dr = cr - br;
                int32_t dg = cg - bg;
                int32_t db = cb - bb;

                if (dr < 0) { dr = -dr; }
                if (dg < 0) { dg = -dg; }
                if (db < 0) { db = -db; }

                d += (uint32_t)(dr + dg + db);
                pc += (uint32_t)(cr + cg + cb);
                pbl += (uint32_t)(br + bg + bb);
            }

            sprintf(Message, "%s filt vs blit %lu conv %lu blit %lu\r\n", k_lbl,
                    (unsigned long)(d / (n * 3u)),
                    (unsigned long)(pc / (n * 3u)),
                    (unsigned long)(pbl / (n * 3u)));
            agree_log(Message);
        }

        /* ---- WHERE THE ~700 ms WENT ----------------------------------------

           Measured 2026-09-29: the new filter costs `grab` 763 ms under `x` and
           1041 ms under `pair`, against 112..119 ms for the library's operator
           and a known ~124 ms for two 320x320 passes.  That gap is larger than
           the work, so it is one phase refusing to be fast rather than the sum
           of the three.  This line is the split, taken INSIDE the call that was
           just made and for that call only - see g_sep_t/g_blit_t in gpu.h.

             set   filter_frame_with()'s own setup before the call
             call  blit_separable_convolve_new() end to end
             put   gpu_fb_put() after it
             bl set / drw / wai / drn
                   the same call split from inside blit_tile(), summed over the
                   two tiles a ROWCOL pair uses; wai is dhd_wait(WRITEBACK), the
                   GPU's own time, and drn is our bounded gpu_drain()
             n     how many tiles contributed (0 = the template refused)

           Read it as: drw or wai dominating = the draw really is that slow (the
           transposed read, the 565 border, the SDRAM region); drn dominating =
           our commit point is; set/call-minus-blit dominating = the driver's
           command path.  If `n` is 0 the whole set is meaningless and the line
           says so by printing zeros - the gains were refused, nothing drew.

           NOT a healthy-path print: this probe runs at boot and this line is
           inside it.  Every value is a per-call snapshot that the next call
           overwrites, so it cannot be summed across calls; that is deliberate,
           because DWT->CYCCNT is 32-bit and one call is already ~0.7 s of its
           7.158 s. */
        sprintf(Message,
                "%s gpu sep t set %lu call %lu put %lu bl set %lu drw %lu wai %lu drn %lu n %lu\r\n",
                k_lbl,
                (unsigned long)g_sep_t[0],
                (unsigned long)g_sep_t[1],
                (unsigned long)g_sep_t[2],
                (unsigned long)g_blit_t[0],
                (unsigned long)g_blit_t[1],
                (unsigned long)g_blit_t[2],
                (unsigned long)g_blit_t[3],
                (unsigned long)g_blit_t[4]);
        agree_log(Message);

        s_gpu_filter_ok = (f_mean <= CAM_GPU_AGREE_MAX);
        if (s_gpu_filter_ok && (f_swap < f_sum))
        {
            UART_SendString("gpu agree filt ORDER?\r\n");
            s_gpu_filter_ok = false;
        }
    }
    else
    {
        /* Either the blit is not trusted (so there is nothing to filter) or the
           driver refused the kernel.  Both leave the filter off, which is the
           state that ships the blit - and it is worth a line, because from here
           on `pair` printing f0 and the grab being unfiltered is a consequence
           of this and not a second, independent fault. */
        sprintf(Message, "%s filt off (blit %u, no result)\r\n", k_lbl,
                (unsigned)s_gpu_grab_ok);
        UART_SendString(Message);
        s_gpu_filter_ok = false;
    }

    /* Now the question the lines above cannot answer.  A mean of 109 says the
       two pictures are unrelated, which is what a broken texture read, a
       destination someone else is writing, and a scene that moved all look
       like.  The crop probe takes the camera and the scale out of it: a
       CPU-owned source at 1:1, compared a word at a time, so whatever it finds
       is a property of the blit itself.
       Not latched into s_gpu_grab_ok - it runs at a size the grab never uses,
       so it can inform the next change but cannot speak for the production
       path.  See gpu_scale_crop_probe() in gpu.h.

       OFF as of 2026-09-18, and the call is kept in a comment because the
       reason it went is not that it was wrong but that it was expensive and
       had nothing left to say.  It is called once per agreement probe and the
       boot makes six of those; each call runs ~20 GPU passes and then reads the
       whole 320x320 destination - 102400 words of uncached Device memory - a
       dozen times over (the prod map, crop_zone_scan, and seven crop_row_scan
       sweeps).  That is on the order of a million slow SDRAM reads per call,
       six times per boot, to answer a question that is answered: the blit's
       geometry is measured, and the fault that these sweeps were built to find
       was `dhd_set_texture_scalebias`, not the scale or the pitch.

       What it costs to keep: the boot block grows by ~270 lines and by the
       only work in it heavy enough to be mistaken for a hang.  The user's
       evidence is exactly that - a backup build with ONE agreement probe
       triggers detection from the serial port, and this build with six does
       not.  Re-enable it by uncommenting the single line below. */
    (void)addr;
    /* gpu_scale_crop_probe(addr, (uint32_t)CAM_W); */

    return s_gpu_grab_ok;
}
#endif  /* GPU_ENABLE - end of camera_gpu_agreement_probe; it is called only from
           main.c's boot block, which is inside the same macro. */

/**
 *\*\name    camera_map_box.
**/
void camera_map_box(const YoloBox* in, YoloBox* out)
{
    /* The detection frame is the camera frame stretched to a square, the same
       thing tools/make_detect_image.py does to the test frame, so undoing it is
       a plain scale per axis and there is no letterbox offset to subtract. */
    const float kx = (float)CAM_W / (float)YOLO_IN_W;
    const float ky = (float)CAM_H / (float)YOLO_IN_H;

    out->x1 = in->x1 * kx;
    out->y1 = in->y1 * ky;
    out->x2 = in->x2 * kx;
    out->y2 = in->y2 * ky;
    out->score = in->score;
    out->class_id = in->class_id;
}
