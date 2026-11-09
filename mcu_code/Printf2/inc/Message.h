#ifndef __MESSAGE_H__
#define __MESSAGE_H__

/* THE BOOT LOG SWITCH - one line, and it is the ONLY thing it affects.

   0 (the default): the boot is FIVE lines and nothing else:

         yolo start ...          <- not gated, see STAGE MARKERS below
         yolo init OK            <- not gated
         gpu init OK             <- not gated (GPU_ENABLE 1 builds only)
         firmware: CPU+GPU       <- not gated, the build identity
         boot ok                 <- not gated, always the LAST line

      Everything else the initialisation used to report on SUCCESS is gone.

   1: all of it comes back, unchanged.  Nothing else in the build moves - in
      particular the agreement probe still RUNS either way (see the note at its
      call site in main.c: it is the only place s_gpu_grab_ok /
      s_gpu_filter_ok are latched, i.e. the only thing that decides whether the
      grab front end runs on the GPU or the CPU), and the per-trigger lines
      are untouched.  Only its reporting is gated, through camera.c's
      agree_log().

   FAILURES ARE NOT GATED.  `init FAIL ...` (yolo_inference.cpp), `yolo init
   ERROR`, `camera init ERROR`, `gpu init ERROR <n>`, `gpu selftest FAIL <n>`
   and the probe's `buf BAD` / `NO FRAME` / `NO GPU` / `ORDER?` lines go out at
   0 and at 1 alike: a silent board is the one outcome this switch must not
   produce, and a boot log exists for exactly those lines.

   STAGE MARKERS ARE NOT GATED EITHER (2026-10-12, on the user's request):
   `yolo start ...`, `yolo init OK` and `gpu init OK`.  They are what makes the
   delivered boot log readable - each one says the board reached a stage - so
   they stay at 0 and at 1 alike, exactly like `boot ok` and the `firmware:`
   identity line.  Do not move them into this switch.

   It gates the same lines in the GPU_ENABLE 0 build - there it is only
   `init: arena` / `init OK`, the two `rst` lines, `camera init OK` and
   `camera ID: 0x...`, since the whole GPU block is already gone.

   IT LIVES HERE because this is the header EVERY file that prints already
   includes - main.c, camera.c, gpu.c, bsp_ov5640.c and yolo_inference.cpp all
   include "Message.h" (it is the print sink: Message[192] + UART_SendString),
   so one macro reaches all five halves of the boot log with no new include and
   no new source file.  It was in camera.h until 2026-10-12, which meant gpu.c,
   bsp_ov5640.c and yolo_inference.cpp could not see it and kept printing their
   boot diagnostics through it.

   TRAP: because it is read in `#if BOOT_VERBOSE`, a file that does NOT include
   this header does not fail to compile - an undefined macro is 0 in `#if`, so
   the lines just vanish silently.  That is why every configuration below is
   built as its own step (see the plan's acceptance list).                      */
#define BOOT_VERBOSE    0

/* THE DISPLAY SWITCH (2026-10-20, on the user's request).

   0 (the default): the serial port carries the whole round log - the `cas ...`
      line, the `ch6 acc` / cross-round probe lines, every diagnostic.  Nothing
      changes; this is the switch at rest.

   1: the port carries ONE thing per round - the plate itself, e.g.

         yolo start ...
         yolo init OK
         粤R888G8
         琼A3M286

      and nothing else.  A round that read no plate prints nothing at all (it is
      not a "(no char)" line: this mode is for looking at the result, not for
      diagnosing).  Round output keeps running at full speed either way - only
      the reporting is gated.

   IT IS ENFORCED IN ONE PLACE, UART_SendString().  Everything in the firmware
   prints through it, so this one check silences main.c, camera.c, gpu.c,
   bsp_ov5640.c and yolo_inference.cpp at once - including the DVP ISR's lines
   and the per-character cascade lines that would otherwise interleave with the
   plate.  A suppressed line is still built by sprintf into Message first - only
   the (slow) UART transmit is saved, not the formatting.  Nothing in the
   decision logic depends on whether a line was printed.

   THREE BOOT LINES ARE EXEMPT: `yolo start ...`, `yolo init OK` and `yolo init
   ERROR`.  With this switch on they are the only evidence that the board booted
   at all - a board that says nothing vs. one that printed the markers and then
   never found a plate are different problems, and a board that dies inside
   yolo_plate_init() (that path ends in while(1)) must not die silently.  They
   are matched by PREFIX inside UART_SendString, so no call site has to know
   about this switch.  `boot ok`, `firmware: ...` and `gpu init OK` are NOT
   exempt: the user asked for these lines specifically.

   EVERYTHING ELSE IS SILENCED, INCLUDING FAILURES.  That is deliberate - this
   is a delivery / demo mode, not a debug mode.  If the board misbehaves with
   this on, set it back to 0 before concluding anything from the silence; the
   three lines above are what you get for free either way. */
#define DISPLAY_PLATE_ONLY   1

#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"
#include "string.h"
//#include "tensorflow/lite/micro/debug_log.h"
extern char Message[192];   /* 64 was too small, 160 was too small again - see Message.c */

/* 1 = 这一条允许通过 UART_SendString。 只有 DISPLAY_PLATE_ONLY=1 时才有人看它:
   打印车牌那一行前把它置 1, 打完置回 0。 默认 0 = 全部拦掉 (除了两个开机标记)。 */
extern volatile uint8_t Message_allow;
void UART_SendString(const char* str);
//void DebugLog(const char* s);
#ifdef __cplusplus
}	
#endif
	
#endif