#ifndef CAMERA_H
#define CAMERA_H

#include <stdint.h>
#include <stdbool.h>
#include "yolo_inference.h"

/* BOOT_VERBOSE (the boot-log switch) used to live here.  It MOVED to
   inc/Message.h on 2026-10-12: this header is included by main.c and camera.c
   only, so gpu.c / bsp_ov5640.c / yolo_inference.cpp could not see the macro
   and kept printing their boot diagnostics straight past it.  Message.h is the
   one header every file that prints already includes.  Nothing else about the
   switch changed - read the block at the top of Message.h.                    */

/* 1: the detection frame comes from the OV5640 on DVP1, triggered by a
       character on the debug UART; the LCD shows the live preview.
   0: it comes from the frozen test frame in XIP flash (src/yolo_detect_image.inc)
      and the camera is never touched.  Keeps the model path testable on a board
      with no camera fitted, and separates "the camera is wrong" from "the
      model is wrong" when something stops working.                          */
#define CAMERA_ENABLE   1

/**
 * @brief  Bring up the LCDC preview, DVP1, the SCCB bit-bang bus and the
 *         OV5640 in RGB565 800x480, then let the DVP run free into the frame
 *         buffer so the LCD shows a live picture.
 * @retval true if the sensor answered with the expected ID
 */
bool camera_init(void);

/**
 * @brief  Copy the frame currently in the camera frame buffer into a 320x320
 *         RGB888 buffer, the layout yolo_plate_detect() wants.
 *
 * The DVP is NOT stopped here, and that is the first thing to know about this
 * function: one read takes 175..300 ms on this board (measured), while the DVP
 * ping-pongs between two buffers and each one is only stable for one frame
 * period (~66 ms).  A read that long spans several frame boundaries, so the
 * result can straddle two frames.  This doc used to claim "the DVP is stopped
 * while the frame buffer is read" long after the code stopped doing that, and a
 * caller built on the stale claim cannot work - see camera_dvp_pause() below.
 * The 800x480 -> 320x320 step is a stretch, matching what
 * tools/make_detect_image.py does to the test frame.
 *
 * @param  dst : 320*320*3 bytes
 * @retval true on success
 */
bool camera_grab_rgb888(uint8_t* dst);

/**
 * @brief  Convert a whole 800x480 RGB565 frame to RGB888, one source pixel per
 *         output pixel, into a buffer of 800*480*3 = 1,152,000 bytes.
 *
 * Not the same job as camera_grab_rgb888(): that one resamples the frame down
 * to the model's 320x320 and in doing so drops three of every five columns.
 * This keeps every pixel, which is what the cascade's second stage needs - it
 * crops a rectangle out of the frame and resamples the *crop*, so whatever it
 * crops from has to still hold the pixels the 320x320 stretch discarded.
 *
 * The address is a parameter rather than an internal snapshot, so that the
 * caller can pin one frame and read the same one for both this and the 320x320
 * grab.  Taking two independent snapshots is how a cascade ends up cropping a
 * different frame than the box it was given.  Callers that care should hold
 * camera_frame_count() still across both reads; it changes only in
 * camera_frame_done(), so an unchanged count proves the buffer did not change.
 *
 * @param  src565 : the frame buffer to read, i.e. a pinned camera_frame_addr()
 * @param  dst    : 800*480*3 bytes
 * @retval false if src565 is 0 or dst is NULL
 */
bool camera_unpack_full_rgb888(uint32_t src565, uint8_t* dst);

/**
 * @brief  Map a box found in the 320x320 detection frame back to camera
 *         (800x480) pixels, with the aspect distortion undone.
 */
void camera_map_box(const YoloBox* in, YoloBox* out);

/**
 * @brief  The 800x480 RGB565 frame buffer camera_grab_rgb888() would read,
 *         i.e. the last one the DVP finished filling.
 *
 * This is the address of the texture a GPU-side grab has to hand to the 2D
 * engine, and it is the same frame the CPU-side grab would convert - which is
 * what lets the two be compared on one frame.
 *
 * It is never 0 and never "no frame yet": it starts out at buffer 1, the one
 * the LCDC is pointed at before the DVP has completed anything, so the worst
 * case is that its contents are whatever the frame buffer powered up with.
 *
 * Reading it does not stop the DVP (neither does the grab - see the note on
 * camera_grab_rgb888(); use camera_dvp_pause() if you need one).  The buffer it
 * names is not the one being filled: the DVP ping-pongs, and camera_frame_done()
 * only records a buffer once it is complete and the LCDC has been moved to it.
 */
uint32_t camera_frame_addr(void);

/**
 * @brief  Frames the DVP has completed since boot.
 *
 * For anything that has to wait for a real picture before it measures
 * something.  camera_frame_addr() cannot answer that: it is non-zero from the
 * first instruction, so it cannot distinguish buffer 1 holding a frame from
 * buffer 1 holding whatever the SDRAM powered up with.
 */
uint32_t camera_frame_count(void);

/**
 * @brief  运行时把 LCDC 像素时钟在 AXI/8 和 AXI/16 之间切换, 返回 1 = 现在慢。
 *
 *         **默认就是慢 (AXI/16)**: 2026-10-19 上板定案, 屏闪的病因是 LCDC
 *         从这块原厂确认的慢 SDRAM 上取像素供不上。 同一份固件只按一次这个
 *         键 (→ /16) 之后, `lcd fu 1/1/0 fuk 1/1/0` 立刻变成 `0/0/0`、屏不再
 *         闪; 而且推理 2.6 s → 1.38 s、抓帧 385 → 325 ms —— LCDC 原来吃掉的
 *         那 ~49 MB/s 还给了 CPU。
 *
 *         这个键留下是为了能当场切回 /8 做 A/B。
 *
 *         AXI = 300 MHz: /8 = 37.5 MHz (1090x535 时序下约 64 Hz, 屏要 ~49 MB/s),
 *         /16 = 18.75 MHz (约 32 Hz, ~25 MB/s)。 切换瞬间屏可能抖一下, 属正常。
 */
uint32_t camera_lcd_clk_toggle(void);

/**
 * @brief  直接读 LCDC 的 INTSTS / INTEN 原始值, 给按键探针用。
 *
 *         判读时对照 camera.c 的 camera_init(): INTEN 里 FU/FUK/BE/RFE 四位
 *         现在会被置上, 所以 INTSTS 的同四位一旦为 1 就是真的出过事。
 */
void camera_lcd_status(uint32_t* intsts, uint32_t* inten);

/**
 * @brief  Hold the camera still, so that several reads of one frame really are
 *         reads of one frame.  Blocks ~100 ms, then leaves the DVP off.
 *
 * The problem it solves: a full 800x480 RGB565 read is 175..300 ms here, while
 * each frame buffer stays untouched for only one frame period (~66 ms).  So
 * "camera_frame_count() did not change across my reads" - the test the cascade
 * used - can never come out true, and a caller that insists on it never runs at
 * all.  Stopping the port turns that test into one that CAN pass: no more
 * frame-end interrupts, so nothing to change the count.
 *
 * camera_dvp_resume() MUST follow, or the preview never comes back.
 *
 * What the preview costs: it stands still for the duration of the reads (~300 ms).
 * That is not new - the inference that follows already takes 1.7 s - and it buys
 * a frame that is provably in one piece.
 */
void camera_dvp_pause(void);

/**
 * @brief  Let the DVP run again.  Always the other half of camera_dvp_pause().
 */
void camera_dvp_resume(void);

/**
 * @brief  中断侧: 每次帧结束把相机自己的错误位读走、计数、清掉。
 *
 *         相机写坏的那一页会被 LCDC 忠实地显示出来 (残帧 ≠ 屏读不到),
 *         所以这个计数是"屏上的异常来自相机"的唯一证据。 推理抢总线那阵
 *         已经靠停住 DVP 消除了, 正常应当恒为 0。
 */
void camera_dvp_frame_end_scan(void);

/**
 * @brief  开机以来被记成"坏帧"的次数 (DVP 写 FIFO 溢出/AHB 错/整页溢出/跳帧)。
 */
uint32_t camera_dvp_bad_frames(void);

/**
 * @brief  Convert one frame both ways and report the difference and the
 *         timings, so the GPU grab can be judged before it is switched on.
 *
 * Runs the CPU bilinear and the GPU scaled blit over the SAME frame - the
 * address is snapshotted once, deliberately not via camera_grab_rgb888(), which
 * would take its own snapshot.  Prints one line of timings (CPU, GPU blit, GPU
 * unpack, all in microseconds) and one line of agreement (the mean channel
 * difference, the worst single channel, and the mean with red and blue
 * exchanged).
 *
 * The two paths cannot agree bit-for-bit even when both are right: the CPU
 * steps the source at exactly 2.5 pixels per output pixel while the GPU's scale
 * gives 2.4922, and they round in a different order.  So the mean is the
 * verdict and the swapped mean is what separates a channel-order problem from
 * everything else.
 *
 * Waits for a completed frame first, bounded at about a second, so a DVP that
 * never delivers cannot hang the boot.
 *
 * Side effect, and the reason it returns anything: it latches whether the GPU
 * path may be used for detection, which camera_grab_rgb888() then acts on.  So
 * the answer to "should the GPU grab be switched on" is measured on a real
 * frame at boot rather than assumed, and a GPU that disagrees falls back to the
 * CPU on its own.  The threshold and its reasoning are at CAM_GPU_AGREE_MAX in
 * camera.c; the printed `ok 0|1` on the mean line is this verdict.
 *
 * @param  dst_cpu : 320*320*3 bytes for the CPU result
 * @retval true if the GPU path was accepted and camera_grab_rgb888() will use
 *         it; false if it was not - including when no frame arrived or the GPU
 *         was not up, in which case nothing was measured beyond the reason
 */
bool camera_gpu_agreement_probe(uint8_t* dst_cpu);

/**
 * @brief  Fill both 320x320 RGB888 buffers from ONE frame, so the model can be
 *         run over both front ends without the scene moving between them.
 *
 * The path A/B, and the only way to ask whether the GPU front end decodes as
 * well as the CPU one.  `gpu agree mean 4 ok 1` cannot answer that: it is a
 * pixel criterion, and the model's own output varies frame to frame on a
 * stationary plate, so a comparison across two triggers would measure the scene.
 * One snapshot, two conversions, both returned to the caller.
 *
 * The CPU result goes into `dst_cpu`; the GPU result into the scratch buffer the
 * boot agreement probe also uses.  Deliberately without the Stage 3 filter, so
 * the comparison is one change and not two.
 *
 * @param  dst_cpu : 320*320*3 bytes, receives the CPU bilinear's output
 * @retval the GPU result, or NULL if the GPU path is unavailable - dst_cpu is
 *         still valid then, there is just nothing to compare with
 */
const uint8_t* camera_grab_pair(uint8_t* dst_cpu);

/**
 * @brief  Did the last camera_grab_pair() GPU arm include the convolve?
 *
 * The pair's GPU arm mirrors the shipped front end, so what it included is a
 * property of the build AND of whether gpu_filter_frame() accepted the call at
 * runtime.  Printing it with the A/B keeps the reading self-describing - the
 * failure this guards against is a stale comment saying which arm ran.
 *
 * @retval true if GPU_SCALE_CONV (blit + filter) was unpacked, false if the
 *         unfiltered GPU_SCALE_DST was
 */
bool camera_pair_filtered(void);

/**
 * @brief  Called from DVPx_IRQHandler() when the DVP has finished filling a
 *         buffer and the LCDC has been pointed at it.  Records the buffer as
 *         the one camera_grab_rgb888() should read, and counts the frame.
 *
 * This runs in interrupt context and must stay free of anything that can block.
 *
 * @param  addr : the frame buffer that just completed
 */
void camera_frame_done(uint32_t addr);

#endif // CAMERA_H
