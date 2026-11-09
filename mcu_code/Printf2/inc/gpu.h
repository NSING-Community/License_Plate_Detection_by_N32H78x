#ifndef GPU_H
#define GPU_H

#include <stdint.h>
#include <stdbool.h>

/* THE MASTER GPU SWITCH - one line, two builds.

   1: the product as it ships.  The D/AVE HD 2D engine is brought up at boot
      (init + the round-trip self test below), and the camera front end runs on
      it: gpu_scale_frame() for the 800x480 -> 320x320 blit and, with
      GPU_FILTER_ENABLE, one low-pass on top.  The model's depthwise, conv and
      transpose offloads are all in the image but ship OFF
      (g_gpu_dw_on / g_gpu_conv_on / yolo_tr_get() reset to 0), so the operators
      themselves run on the CPU.

   0: PURE CPU.  Nothing in the product path reaches the GPU - no bring-up, no
      operator, no diagnostic.  The grab falls back to
      downsample_rgb565_to_rgb888(), every layer runs on CMSIS-NN, the pair A/B
      and the boot agreement probe do not exist, and the per-trigger
      `gpu dw ...` / `gpu drn ...` lines are not printed (this build is silent on
      the healthy path by construction, not by a flag).

      SERIAL KEYS: this delivery tree has none.  All 18 key branches were
      deleted from main.c (see the loop comment there); bytes arriving on the
      UART are read and discarded, so there is no GPU-key path, no CPU-side
      key (`e` `v` `b` `i`), and no generic trigger to fall through to.  In the
      internal debug tree the keys still exist, and with GPU_ENABLE = 0 each
      GPU key answers `gpu off in this build` rather than being dropped
      (an unhandled key would fall through to the generic trigger).

   THE CUT IS AT THE DECISION POINTS, NEVER INSIDE gpu.c.  The `#if GPU_ENABLE`
   sites are: camera_grab_rgb888()'s front-end arm and the three camera entry
   points that only exist to serve it, the bodies of dw_gpu_invoke() /
   conv_gpu_invoke() / ex_invoke_tr(), the conv census in yolo_plate_detect(),
   the GPU-stall watchdog in n32h7xx_it.c's DVPx_IRQHandler(), and main.c's boot
   block and the per-trigger statistics.  (The eleven GPU key handlers existed in
   the internal tree only; they are deleted from this delivery tree.)  gpu.c has
   no `#if` on purpose - with this macro at 0 nothing references it and the
   linker drops it, and a second cut inside it would be a second source of truth
   for the same decision.  The two `prepare` wrappers in yolo_inference.cpp are
   deliberately NOT cut: the map shows they reference no gpu.o symbol, so they
   are pure bookkeeping, and their invoke stub falls straight through to the
   stock kernel.

   WHAT 0 REMOVES, AND WHAT IT DOES NOT.  gpu.o goes entirely: its row in the
   map's Image component sizes table reads Code 0 / RO 0 / RW 0 / ZI 0, and no
   object refers to it.  Measured on the two 2026-10-12 builds (b and d):
   Code 213930 -> 136974, RO-data 770562 -> 761390, RW-data 352 -> 8, ZI-data
   2706472 -> 2250984 - the ZI is gpu.c's scratch planes, ~455 KB of them.

   What SURVIVES is nsgpu.lib, and that is expected, not a failure: the vector
   table hard-references GPU_IRQHandler (startup_n32h78x_cm7.lst:497), which
   lives in nsgpu.lib(davehd_kernel_irq.o), so the library's kernel side stays
   linked and is simply unreachable (Library Totals 10916 Code).

   THE JUDGE IS THE MAP'S `refers to gpu.o`, NEVER Code=.  With 0, main.o /
   camera.o / yolo_inference.o / n32h7xx_it.o must show NONE (gpu.o referring to
   itself is not a counter-example - armlink prints that for the whole input
   object whether or not its sections survive).  Read gpu.o's row in the
   component-sizes table to confirm.                                        */
#define GPU_ENABLE      1

/* Everything the GPU touches lives in one 4 MB slot of the external SDRAM.
   That slot has NO MPU region of its own, in main.c or anywhere else - and
   that is deliberate, not an oversight.

   An MPU region was tried here (0xC0C00000, 2 MB, TEX1 + NOT_CACHEABLE, the
   same attributes as the frame-buffer region beside it) and it hung the board
   at `gpu init OK`: the GPU accepted the job and never reported it done, so
   the print after it never appeared.  Not a fault - a HardFault or BusFault
   would have printed through Fault_Dump.  The likely reason is ordering: the
   driver writes a command stream into vidmem and then writes the start
   register, and the default Device mapping for 0xC0000000.. is what makes
   those writes reach SDRAM in program order.  Downgrading the slot to Normal
   memory, even non-cacheable, lets them be buffered and reordered.

   Nothing is lost by leaving it as the default: the CPU never reads or writes
   the GPU's working memory in the normal course of things, so there are no
   cached copies to go stale.  The one place the CPU does read it back is
   gpu_selftest(), and that read is of a region the GPU has already finished
   with.

   The region that DOES need its attributes set is region 0 (the tensor arena)
   and region 1 (the camera frame buffers); see the comments in main.c.  The
   bug that region 2 was meant to fix - unaligned accesses faulting - was an
   arena problem, not a GPU one.

   Slot layout - all inside 0xC0C00000..0xC0FFFFFF:

     0xC0C00000  video memory the driver carves up itself (framebuffer,
                 command lists).  nsgpu_init() takes the base but no size;
                 the driver's own accounting decides how far it runs, which
                 is why the two test buffers below sit at the very top of the
                 slot rather than immediately after it.
     0xC0E00000  the "acc" (accumulation) area.  acc_mode is 0 so it should
                 go untouched, but the vendor header points this at
                 0x30000000 - which is this board's AHB SRAM, where the C
                 library heap lives.  Pointing a second bus master at live
                 heap is not something to leave in.
     0xC0F80000  test source pattern   (128x64 max, 16 KB)
     0xC0F90000  test destination      (128x64 max, 16 KB)
     0xC0FC0000  the 320x320 scaled-blit destination, 200 KB                */

#define GPU_VIDMEM_BASE     0xC0C00000u
#define GPU_ACC_BASE        0xC0E00000u
#define GPU_SLOT_BASE       0xC0C00000u
#define GPU_SLOT_SIZE       0x00400000u     /* 4 MB, aligned to its own size */

#define GPU_TEST_SRC        0xC0F80000u
#define GPU_TEST_DST        0xC0F90000u
#define GPU_TEST_W          128u
#define GPU_TEST_H          64u

/* Stage 1 of moving the inference front end onto the GPU: the scaled blit.

   nsgpu_colorkey() is the only entry point in the library that takes a source
   rectangle AND a scale, which makes it the one candidate for doing
   800x480 -> 320x320 in a single call.  The vendor's DrawGraphic example uses
   it for exactly that - its comment reads "reduce the image from 256 to 160".

   Read off nsgpu.lib by disassembly, because the header says nothing:

     - scale_x and scale_y are 8.8 fixed point with 256 = 1.0.  The destination
       extent is (source_extent * scale) >> 8 pixels.  The demo's 256 -> 160 is
       a factor of 0.625, which is what its comment describes.
     - The destination format is the driver's own framebuffer format - the
       function reads g_framebuffers[framebuffer_acitve_index] and writes that
       format back to the texture unit.  So a scaled blit CANNOT convert
       RGB565 to RGB888; the RGB565 -> RGB888 step stays on the CPU.
     - Texture filtering (bilinear) is enabled, so the scale has interpolation.
     - dst becomes the driver's framebuffer address, so it has to be restored
       afterwards or every later GPU call uses the wrong framebuffer.

   What the probe below exists to establish is what the disassembly did NOT
   settle: nsgpu_draw_rotblit builds its destination rectangle AROUND A CENTRE
   (four corners at centre +/- half-extent), so dst_startx/dst_starty are where
   the centre of the scaled image lands, not its top-left corner - but exactly
   how that centre is converted to a raster origin is tangled enough in the
   angle maths that measuring beats reading.  The colour key is the other open
   question: one colour argument serves as both the background fill and the
   key, and on a real camera frame some pixel will match it. */
/* Where the 320x320 blit lands.  It first went to 0xC0F00000, the nominal end of
   the driver's 3 MB pool, on the theory that the driver - it gets the vidmem
   base and NO SIZE - was overrunning into it.  That theory was measured and
   disproved: 0xC0FC0000 produced a bit-identical result, pixel for pixel, so the
   driver's accounting is not the fault and either address is safe.  0xC0FC0000
   is kept because it sits above the two test buffers that gpu_selftest() proves
   writable, which is the better-evidenced of the two placements, and 200 KB
   below the top of the slot.

   What was actually wrong is on the SCALE path, not the destination; see the
   measured table in gpu_scale_crop_probe().                                  */
#define GPU_SCALE_DST       0xC0FC0000u
#define GPU_SCALE_W         320u
#define GPU_SCALE_H         320u

/* Stage 3: where the separable convolve's output lands.

   A second 320x320 buffer is not an optimisation, it is a requirement of the
   library's semantics: pass 1 of nsgpu_separable_convolve() runs IN PLACE, with
   the texture and the framebuffer being the same descriptor, so the SOURCE
   buffer is scratch by design and is destroyed.  Pass 2 writes the caller's
   `dst`.  So the filter needs a source and a destination and cannot be done in
   one buffer - measured in gpu_selftest(), where both buffers are checked
   against the pattern afterwards precisely because neither is guaranteed to
   survive.

   0xC0F00000 is the address the blit's destination was briefly moved to and
   measured bit-identical at, so it is known writable and known not to collide
   with anything the driver owns.  Its 200 KB ends at 0xC0F32000, clear of
   GPU_TEST_SRC at 0xC0F80000 and GPU_SCALE_DST at 0xC0FC0000. */
#define GPU_SCALE_CONV      0xC0F00000u

/* Stage 3's second scratch plane.  It is a real intermediate between two draws
   and can be neither the source nor the destination: the template's two-pass
   modes write pass 1 here and read it back in pass 2.  (It was called the
   ROWCOL_T mode's requirement until 2026-10-08, when that mode's reason was
   falsified and stage 3 moved to ROWCOL - the plane is needed by both, and the
   transposition ROWCOL_T stored here is no longer produced.)
   RGB565, 2 bytes per texel, so 2*320*320 = 200 KB.

   0xC0F34000 is the first 200 KB slot that is free: GPU_SCALE_CONV's own 200 KB
   ends at 0xC0F32000 (gpu.h:121), and the next named buffer above is
   GPU_TEST_SRC at 0xC0F80000 (gpu.h:60), which leaves 0xC0F32000..0xC0F80000
   empty.  Ending at 0xC0F66000, it stays clear of both. */
#define GPU_SEP_TMP         0xC0F34000u

/* Whether the grab path runs the 320x320 separable convolve between the blit
   and the unpack.  OFF, and the reason is measured rather than assumed.

   It was built as Stage 3 on the expectation that a 5-tap low-pass in front of
   the model would help it see plates, and the first round used the IDENTITY
   kernel so that the wiring and the filtering were separable questions.  Both
   halves of that came back in one flash:

     - the wiring is correct.  `gpu conv b0 f0 n0` at 320x320, all four corners
       bit-equal to the grating the CPU wrote, so no shift and no clip; and the
       identity kernel left the detection result unchanged, which is what the
       identity was there to check.

     - the filter costs 62 ms, measured as the difference in `grab` between this
       and the same build without the call (53 985 -> 115 931 us).  For scale,
       the scaled blit it sits behind is 24 ms.

   62 ms is affordable against the CPU bilinear it replaced - the whole path is
   still 58 ms faster than the pre-GPU baseline - but nothing justifies paying
   it, and the justification that was expected did not survive being checked:

     - NOT anti-aliasing.  Training resizes with a plain non-uniform bilinear
       downscale to 320x320 (utils/datasets.py: cv2.resize(img, (320, 320),
       INTER_LINEAR)), with the labels normalised against that same resized
       image.
       The GPU's bilinear blit IS that operation, so the model already learned on
       inputs from it; blurring afterwards moves the input away from the training
       distribution rather than toward it.  Training augmentation is
       contrast/brightness plus HSV jitter (utils/datasets.py: img_aug) - the
       geometric ones, motion_blur and random_resize, are commented out - so the
       model saw no resize jitter either.

     - NOT aspect ratio.  The 800x480 -> 320x320 blit squashes vertically by
       1.67x (0.4 in x, 0.667 in y), which looks like a defect and is not one:
       the training resize squashes by exactly the same factor, and the labels
       are normalised to the squashed frame.  So the firmware's geometry matches
       training and there is nothing to correct here.  Worth recording because it
       is the kind of finding that gets re-derived from scratch; see
       [[plate-decode-validation]].

   The decision (2026-09-17) is to run it anyway, and the reason is NOT a new
   measurement.  The goal for this stage is that the inference front end's
   operator work runs on the GPU at all, and the detection result is explicitly
   not the criterion for it: the recognition rate is a training and plate/camera
   geometry question, not a front-end one.  So the filter is ON as the product
   default, and the honest position is recorded here instead of argued - it costs
   62 ms per trigger (+3.6% of the 1.75 s) and nothing here claims it improves
   detection.

   Two properties keep that from being a trap:

     - One line to revert, and reverting loses nothing that was measured.  With
       the filter off the front end is 54 ms, with it on 116 ms.
     - It cannot take the board down.  gpu_filter_frame() returns false if the
       GPU is not up or the driver refused RGB565, and camera_grab_rgb888() then
       unpacks the unfiltered GPU_SCALE_DST exactly as it did before.

   The identity kernel is NOT what to ship in an ON build: it pays the 62 ms to
   return the input unchanged.  s_lp_coeff in gpu.c carries a real kernel. */
#define GPU_FILTER_ENABLE   1

/* The CPU-owned source for gpu_scale_crop_probe() below.  It sits 768 KB above
   the frame buffers, in the gap between them and the GPU slot, so nothing else
   in the firmware is anywhere near it - which is the point: the crop probe has
   to compare against memory that cannot change underneath it.  The gap is at
   least 8 MB and the part is at least 12 MB (gpu_selftest() works at 0xC0F80000,
   which is 15.5 MB in), so the address is inside the part by a wide margin. */
#define GPU_CROP_SRC        0xC0400000u
#define GPU_CROP_PITCH      800u
#define GPU_CROP_W          64u          /* the crop's width */
#define GPU_CROP_H          64u          /* rows filled with the FILL/OTHER layout */
#define GPU_CROP_PROD_H     480u         /* rows filled when reproducing the grab */
#define GPU_CROP_FILL       0x1234u     /* what the 64x64 crop holds */
#define GPU_CROP_OTHER      0x2A55u     /* what the rest of those rows hold */
#define GPU_CROP_NEIGH      0x5A5Au     /* painted into the 192 KB below and the
                                           528 KB above the source block, so that
                                           a texture read landing NEAR the source
                                           but not on it comes back as a value
                                           that names itself.  If the scaled
                                           path's constant output is a read of
                                           the wrong address rather than a
                                           constant in the library, this is what
                                           changes it - and if it does not change,
                                           the value is not coming out of this
                                           memory at all. */
#define GPU_CROP_STUCK      0x07C3u     /* NOT the scaled path's output, which is
                                           what this was named for and believed
                                           to be for several rounds.  It is the
                                           constant nsgpu_colorkey() PRE-FILLS
                                           with: the call paints an
                                           (src_w x src_h) box at the start with
                                           0x07C3, ignoring the `colour` argument
                                           entirely, and that paint is what shows
                                           through wherever the centred image
                                           does not cover.  Established by the
                                           bg pass - `whole B0` says the colour
                                           passed is never written anywhere.
                                           Kept as a named constant because a
                                           destination word equal to it means
                                           "inside the pre-fill box, outside the
                                           image" and that is worth being able to
                                           say; but a count of them is a count of
                                           where the image is NOT, not of what the
                                           sampler produced. */
#define GPU_CROP_SENT       0xC3C3u     /* what the CPU clears the destination to
                                           before every pass, so that a word
                                           which is neither FILL nor STUCK is
                                           evidence the blit did not reach that
                                           pixel rather than evidence the CPU
                                           wrote it.  Distinct from the other
                                           three by construction, and above the
                                           ramp source's 0..11820 range so a
                                           ramp pass cannot produce it by
                                           accident - though FILL and STUCK both
                                           fall inside that range, which is why
                                           a ramp pass is read by its raw words
                                           and not by its counts. */
#define GPU_SCALE_BG        0xF81Fu         /* magenta: the background fill,
                                               and what a keyed-out pixel
                                               leaves behind */

/**
 * @brief  Power up the GPU, give it a clock and its interrupt, hand it the
 *         SDRAM slot above, and initialise the driver.
 *
 * Tries RGB565 first and falls back to L_8 if the driver refuses it; see the
 * comment in gpu.c for why that ordering.
 *
 * @retval the driver's own return value, passed through unchanged: **1 on
 *         success**, 0 if it could not allocate a framebuffer, -1/-2/-3 for
 *         the three early failures.  See NSGPU_INIT_OK in gpu.c - the vendor
 *         example assigns this result and never reads it, so the library is
 *         the only place the convention exists.
 */
int gpu_init(void);

/**
 * @brief  Make the GPU move some pixels and read them back with the CPU.
 *
 * Four operations, each with its result checked against what the CPU itself
 * wrote, so a pass means the GPU really read and wrote memory:
 *
 *   1. nsgpu_data_fill() a known colour into the destination buffer, then
 *      read every pixel back and count the ones that differ.  Bit-exact.
 *   2. fill the source buffer with a position-dependent pattern, have the
 *      GPU blit it into the destination, then compare the two buffers.
 *      Bit-exact.
 *   3. nsgpu_separable_convolve() with an identity kernel - see s_conv_coeff -
 *      and compare both buffers against the pattern the CPU generated, to
 *      within GPU_CONV_GAP_MAX channel LSBs.
 *   4. the same call with the one-pixel shift kernel of s_shift_coeff, checked
 *      the same way against each of the four diagonal directions.
 *
 * Legs 3 and 4 are what prove the convolve path works on this board in this
 * configuration (RGB565; the vendor's Convolve example uses L_8), and they need
 * each other.  Leg 3 alone cannot tell a working filter from a passthrough,
 * because the identity kernel is the answer under every reading of `coeff` - a
 * library that ignored the array entirely would return the same picture.  Leg 4
 * uses a kernel no passthrough can imitate, and its four counts also pin down
 * the mapping the identity leg is blind to: tap index k is offset k - 2, the
 * same in both passes, so the image moves by (+1, +1).
 *
 * Neither leg asserts bit-exactness, and neither could.  The library normalises
 * the kernel to 8-bit weights whose largest entry is 255 and compensates with a
 * texture scale of 256/255, so every pass rounds each channel once - measured
 * at 1 LSB after the first pass and 2 after the second, never more.  See
 * GPU_CONV_GAP_MAX in gpu.c for why that leaves the verdict safe.
 *
 * The comparison is made against the generated pattern rather than against the
 * source buffer as read back, because this call filters its source in place -
 * a separable convolve is two passes and the library runs the first one into
 * the buffer it was given to read.  So the source is scratch by design, and a
 * caller has to hand this function a source it does not need afterwards, or
 * copy.  Bounding the gap rather than asserting the source is unchanged is what
 * makes that design testable; the first version of this self test asked for the
 * source back untouched and could therefore never pass.
 *
 * There is deliberately no nsgpu_convolve() leg here: that function is not
 * safely callable from C, for reasons recorded in gpu.c above s_conv_kernel,
 * and calling it directly is what faulted this board.  The separable leg goes
 * through nsgpu_separable_convolve_k() for the same reason - the library's own
 * entry point clears no register for its kernel buffer.  The wrapped form of
 * the other one exists (nsgpu_convolve_k(), same file) but its bring-up is
 * gpu_convolve_probe()'s job, kept out of the self test so that a fault in the
 * newest call cannot make the whole self test's verdict ambiguous.
 *
 * This is a readback rather than something drawn on the LCD on purpose: the
 * frame buffers belong to the DVP, the LCDC ping-pongs between two of them,
 * and anything the GPU painted would be overwritten by the next camera frame
 * before anyone could look at it.  A readback is also the only version that
 * can distinguish "the GPU drew nothing" from "the GPU drew the wrong thing".
 *
 * On failure this prints the five counts (fill, copy, destination, source, and
 * shift), the fill colour and pixel width it was testing against, and the first
 * eight bytes the GPU actually produced after the fill and after the convolve;
 * on success it prints nothing, and the caller's "gpu selftest OK" is the whole
 * story.
 *
 * @param  bad  if not NULL, receives the total number of pixels outside the
 *              gap bound, plus the bit-exact failures of the first two legs
 * @retval true if the first two operations produced exactly what the CPU wrote
 *         and both convolve legs stayed inside GPU_CONV_GAP_MAX
 */
bool gpu_selftest(uint32_t* bad);

/* Measured on hardware, 2026-09-15.  The first run of the probe below used
   scale (102, 170) - the naive "320/800 x 256" - with the start at the middle
   of the destination, and printed:

       gpu scale box 160 160 319 319      (so 160x160 painted, 25600 pixels)
       gpu scale painted 25600 hole 0
       gpu scale 102 170 us 341

   What that reading was originally taken to mean, and what it means:

     - **The image is CENTRED, not placed at the start.**  This is the correction
       to my own earlier "correction".  `box 160 160 319 319` looked like the
       rectangle starting at 160, but it is the library's PRE-FILL that starts at
       (dst_startx, dst_starty) - the pre-fill covers an (src_w x src_h) box
       there and is 0x07C3 - while the image itself is centred inside that box and
       for scale (102,170) sat mostly outside the visible 320x320.  The
       centre-based corner construction visible in nsgpu_draw_rotblit is the real
       behaviour and angle 0 takes it too.  See the geometry table below.
     - The colour key does not punch holes in real camera data: `hole 0` over a
       live frame.  Weak evidence - magenta simply may not occur in that frame -
       but it is the answer to the question as asked, and it is the answer that
       lets nsgpu_colorkey carry camera pixels at all.
     - The painted box is 160x160 because the pre-fill is CLIPPED at the 320-wide
       framebuffer.  So this reading says nothing about the extent, which is why
       the probe now runs two placements and the second one starts at the origin.

   Timing: 341 us for the call, against 176,746 us for the CPU bilinear it would
   replace.  The call blocks - the self test's bit-exact readbacks could not pass
   otherwise - so that 341 us is the whole of it, not just command submission. */

/* THE SCALED-BLIT GEOMETRY, measured 2026-09-16 and exact on 15 of 15 cases.

   `nsgpu_colorkey(dst, dst_w, dst_h, startx, starty, src, colour, alpha,
                   width, height, pitch, angle, scale_x, scale_y)` does three
   things in this order:

     1. pre-fills a `width x height` box, anchored at (startx, starty), with the
        constant 0x07C3 - NOT with the `colour` argument, which is never written
        anywhere (measured: the whole-destination scan reports `B0`, i.e. the
        0xF81F passed as the colour appears zero times);
     2. blits the scaled texture over part of it;
     3. leaves the caller's destination untouched everywhere else.

   The image is CENTRED in that box.  Its natural extent is
   `((width * scale_x) >> 8) x ((height * scale_y) >> 8)`, and its top-left
   corner lands at

       startx + ceil((width  - nat_w) / 2)
       starty + ceil((height - nat_h) / 2)

   which is 0 only when the scale is 256 (nat == src, nothing to centre).  That
   single rule reproduces every measurement taken, with no residual:

       src        scale     nat      offset   correct px in the region   measured
       800x480    256       800x480  (0,0)              102400            102400
       800x480    240       750x450  (25,15)   295*305 =  89975             89975
       800x480    200       625x375  (88,53)   232*267 =  61944             61944
       800x480    171       534x320  (133,80)  187*240 =  44880             44880
       800x480    128       400x240  (200,120) 120*120 =  14400             14400
       800x480    103       321x193  (240,144)  80*49  =   3920              3920
       800x480    64        200x120  (300,180)     0                     0
       800x480    32/16/8             further       0                     0
       800x480    103,171   321x320  (240,80)   80*240 =  19200             19200
       256x64     103,256   103x64   (77,0)    [77..102]*64 = 1664          1664
       256x64     256,171   256x42   (0,11)     256*31 =   7936              7936
       256x64     103,171   103x42   (77,11)     26*31 =    806               806
       256x64     128,128   128x32   (64,16)    128*32 =   4096              4096

   The "correct px" column is the intersection of the image with the region the
   pass actually compares, which is what makes the odd-looking 1664 and 3920
   right rather than wrong.

   Consequences for the production call: pass the offset back negated.  `startx`
   and `starty` are u32 and the library ADDS to them, so `-offset` wraps to the
   value that cancels it and the image lands at the origin.  gpu_scale_frame()
   does exactly that. */

/**
 * @brief  Scale a source image into GPU_SCALE_DST and report what landed
 *         where, so the scaled-blit semantics can be pinned down on hardware.
 *
 * Passes `src` straight through as the texture, so the caller decides what to
 * scale - a synthetic pattern, or the camera frame buffer itself.  The camera
 * is the cheaper choice and the one that gets tested: it is already 800x480
 * RGB565 and already running by the time this can be called, so there is no
 * second full-size buffer to fill and the result is a statement about the real
 * input rather than a stand-in for it.
 *
 * Clears the destination to the background colour, blits, then scans for the
 * rectangle that appeared:
 *
 *   - the bounding box of the pixels that are not the background.  Its size is
 *     the check on the (extent * scale) >> 8 reading of scale_x/scale_y, and
 *     that check only works if the rectangle is not clipped - hence the caller
 *     picking a start near the origin.
 *   - how many pixels INSIDE that box came back as the background colour, i.e.
 *     the keyed-out pixels.  That decides whether nsgpu_colorkey() can carry
 *     real camera data or needs a different entry point.
 *   - the wall time of the single call, in microseconds.
 *
 * Placement is an argument rather than a constant because that is exactly what
 * has to be measured, and because the two useful placements answer different
 * questions: one starts at the origin to read the extent off unclipped, the
 * other reproduces the first run.
 *
 * The driver's framebuffer address is saved and restored around the call,
 * because nsgpu_colorkey() repoints it at the destination.
 *
 * @param  src,src_w,src_h  : the texture to scale
 * @param  dst_startx,starty: top-left of the destination rectangle (see above)
 * @param  scale_x,scale_y  : 8.8 fixed point, 256 = 1.0
 * @param  tag              : one short string, printed with the results so the
 *                            cases can be told apart on the console
 * @retval true if anything was drawn at all, false if the destination came back
 *         entirely background - which means the call did nothing, or clipped
 *         itself away, or (in L_8) was skipped
 */
bool gpu_scale_probe(uint32_t src, uint32_t src_w, uint32_t src_h,
                     uint32_t dst_startx, uint32_t dst_starty,
                     uint32_t scale_x, uint32_t scale_y, const char* tag);

/**
 * @brief  Scale a frame into GPU_SCALE_DST, the 320x320 RGB565 the grab wants.
 *
 * This is the probe above with the arguments the measurement settled and none
 * of the instrumentation: scale rounded UP so the destination fills, the
 * centring offset cancelled so the image lands at the origin, one call, no
 * scanning.
 *
 * The scale is `ceil(dst_extent * 256 / src_extent)`, which for 800x480 is
 * (103, 171).  Rounding down instead gives 318 pixels in each axis, two short,
 * and the destination has to be exactly YOLO_IN_W x YOLO_IN_H because that is
 * the tensor's shape.  Rounding up overshoots to 321 columns, which fills the
 * 320-wide framebuffer and is clipped at the far edge.  The colour key punched
 * no holes in a live frame.
 *
 * The centring correction is the whole of the difference between "the scaled
 * blit works" and "the scaled blit writes a constant": without it the image
 * lands at (240, 80) for an 800x480 source and only the bottom-right corner of
 * the destination is right.  See the geometry table above for the measurements.
 *
 * The result is RGB565, in the driver's framebuffer format, so the caller
 * still has to unpack it; see unpack_rgb565_to_rgb888() in camera.c.
 *
 * `pitch` is in PIXELS, and for a packed frame buffer it is src_w.  The vendor's
 * DrawGraphic example is what settles the unit: it passes 220 for a 220-pixel
 * wide ARGB8888 source, so 880 bytes' worth, which a stride in bytes could not
 * be.  It is a parameter and not a constant derived from src_w only because the
 * pixel-versus-byte question was open when this was written, and the agreement
 * probe measures both rather than asserting the answer; the production call site
 * passes CAM_W.
 *
 * @param  src,src_w,src_h : the frame buffer and its dimensions
 * @param  pitch           : texture stride in pixels, src_w for a packed frame
 * @retval false if the GPU is not up, the driver refused RGB565, the dimensions
 *         are degenerate, a drain has latched the GPU dead (gpu_dead_stat()),
 *         or this call's own drain gave up - in which case the caller must fall
 *         back to the CPU path rather than use whatever is in GPU_SCALE_DST
 *
 * That last pair is the 2026-09-24 fix.  This call and gpu_filter_frame() used
 * the library's own framebuffer restore to finish, whose body is dhd_finish() -
 * an UNBOUNDED wait - so a board whose GPU had stopped hung here and never
 * returned to main(), taking the serial handler and the stall reporter with it.
 * Both now restore the address and drain with the same bounded gpu_drain() the
 * depthwise and pointwise tails use, and refuse instead of waiting forever.
 *
 * That covers the restore, not the whole call, and the rest cannot be covered
 * from here: the two draws this function makes - nsgpu_colorkey() below and the
 * library nsgpu_separable_convolve() our _k wrapper tail-branches into, which is
 * where gpu_filter_frame()'s draw goes - each call nsgpu_cmd_sync() internally
 * (Printf.map: `nsgpu.o(i.nsgpu_colorkey) refers to nsgpu.o(i.nsgpu_cmd_sync)`,
 * and 11067 for the convolve).  Those are prebuilt and unpatachable, so the two
 * front-end entries arm the depthwise progress word across the WHOLE draw
 * (stages 45/46) instead: a hang in the draw still cannot be bounded, but it now
 * prints `STALL s45 c0 p0` and reaches the eight-second reset in n32h7xx_it.c
 * rather than hanging the product silently.
 */
bool gpu_scale_frame(uint32_t src, uint32_t src_w, uint32_t src_h,
                     uint32_t pitch);

/**
 * @brief  Print the driver's ambient framebuffer state - the input nothing gates.
 *
 * One tagged line: framebuffer format, size, pitch, address, and the pixel width
 * our readback assumes.  Read after a draw, the format is the format that draw
 * WROTE, which is what unpack_rgb565_to_rgb888() has to be right about - and the
 * one thing no boot verdict has ever checked, because the library takes the
 * framebuffer AND texture format from g_framebuffers[framebuffer_acitve_index]
 * and every probe reports OK regardless.
 *
 * Call it at several points in the boot block: the first tag whose reading
 * differs from the known-good early one names the probe that moved the state.
 * That is the question the last two rounds guessed at.
 *
 * Read-only: draws nothing, sets nothing.  Costs one UART line per call, so boot
 * only.
 */
void gpu_fb_state(const char* tag);

/**
 * @brief  Print the rest of the driver's ambient draw state, plus a fingerprint
 *         of the CLUT allocator, under a short tag.
 *
 * `gpu_fb_state()` covers the framebuffer and has now answered its half: after
 * `gpu_dw_probe()` the framebuffer format, size, pitch, box and address are
 * byte-identical to what `gpu_dhd_probe()` leaves, and the blit is broken
 * anyway.  This prints everything else the driver will hand back - the texture
 * unit and its filter and CLUT base, the colour unit, its operand and its
 * enable, both blend settings, the colour key - and one field that is not a
 * register read: `c4`, the base a four-entry CLUT allocation returns (the shape
 * the vendor library itself uses), which must fail while `gpu_dw_probe()` holds
 * all 512 and must return 0 again after its free.
 *
 * The three lines of one call are one reading; compare calls, not lines.  The
 * first tag at which a field differs from the known-good early one names the
 * probe that moved it - and a boot in which nothing differs anywhere means the
 * leak is in one of the two ambient inputs with no getter,
 * `dhd_set_texture_scalebias()` or `dhd_set_texture_kernel()`.
 *
 * `gpu_dw_probe()` calls it itself, three times (dwA before its drawings, dwZ
 * after them, dwF after its CLUT free), so the two outermost tags in any boot
 * are read without a further edit.
 *
 * Boot only: three UART lines per call, and the `c4` field allocates and frees
 * four CLUT entries.  Every other line is read-only.
 */
void gpu_env_state(const char* tag);

/**
 * @brief  THE PRODUCT'S STAGE 3: a five-tap separable convolve over the blit
 *         output, into GPU_SCALE_CONV, with the OPERATOR selected by
 *         g_gpu_front_sep_on (the library's operator transcribed, or this
 *         repository's own bounded rank-1 pair - see that variable's note).
 *
 * Reads GPU_SCALE_DST and leaves the result in GPU_SCALE_CONV.  It cannot work
 * in one buffer: pass 1 of nsgpu_separable_convolve() filters its source in
 * place, so GPU_SCALE_DST no longer holds the blit's picture once this returns.
 * Callers must read GPU_SCALE_CONV afterwards, and must not read GPU_SCALE_DST.
 *
 * The taps are s_lp_coeff in gpu.c, which is the 3-tap binomial and carries the
 * reasoning for that choice.
 *
 * WHO NORMALISES THE TAPS DIFFERS BETWEEN THE TWO OPERATORS, and only there.
 * The library normalises them itself - to 8-bit weights whose largest is 255,
 * with a texture scale of 256/sum, so the net gain is 256/255 whichever kernel
 * is in the array, and the taps must be non-negative.  The new path sends
 * {0,1,2,1,0} to the CLUT as written and carries the divisions in its two scale
 * arguments, which is what lets one expression serve any kernel.
 *
 * Costs up to GPU_CONV_GAP_MAX channel LSBs per pass; do not compare bit-exactly
 * against it.  Uses gpu_filter_probe()'s numbers as the known-good reading.
 *
 * @retval false if the GPU is not up, the driver refused RGB565, or a drain has
 *         latched the GPU dead (gpu_dead_stat()) - in which case GPU_SCALE_CONV
 *         holds whatever it held before and the caller must fall back to the
 *         unfiltered GPU_SCALE_DST
 */
bool gpu_filter_frame(void);

/**
 * @brief  The new operator alone, ungated: what the BOOT PROBE calls.
 *
 * Identical to gpu_filter_frame() with g_gpu_front_sep_on forced to 1, and it
 * exists so the probe's reading cannot be switched off.  camera_gpu_agreement_probe()
 * calls this, so `gpu agree filt mean/max/swap/ok` - the acceptance criterion for
 * the operator swap - is taken on EVERY boot no matter where the switch is.
 * Do not fold it back into gpu_filter_frame(): an ungated name is what makes
 * "the probe does not follow the switch" a property of the code rather than of
 * the default value.
 *
 * @retval as gpu_filter_frame()
 */
bool gpu_filter_frame_new(void);

/**
 * @brief  Verify gpu_filter_frame()'s geometry at the size the grab uses.
 *
 * The blit's own probe exists because this library's geometry is not what the
 * header implies - it centres a scaled image in a source-extent box, which cost
 * several rounds to find.  The convolve is a different entry point with its own
 * conventions and has only ever been verified at 128x64 with a two-pixel
 * margin.  So make the same measurement at 320x320, over every pixel, before
 * letting it into the grab path.
 *
 * Runs the IDENTITY kernel, not the shipped one, and that is forced by the
 * pattern rather than chosen: the grating is a masked sawtooth, and a blur
 * cannot reproduce a sawtooth - at every wrap the 3-tap averages across a fall
 * of 24 to 0 and lands 8 away from the grating, twice GPU_CONV_GAP_MAX.  Judging
 * the shipped taps against it would report half the columns bad and print
 * `gpu conv NOT usable` over a working filter.  The identity is the kernel under
 * which a deviation from the grating means a geometry fault and nothing else;
 * see s_probe_coeff in gpu.c, and gpu_selftest() leg 4 for the separate question
 * of whether the library honours the coefficient array at all.
 *
 * Fills GPU_SCALE_DST with a grating whose every channel changes by more than
 * the rounding cost of a one-pixel shift - a shift or a clip has to be louder
 * than the 2 LSBs the filter costs, or this measures nothing.  Clears
 * GPU_SCALE_CONV to GPU_CROP_SENT first, so a word the filter never reached
 * names itself instead of looking like a wrong value.  Then reports the count
 * outside GPU_CONV_GAP_MAX, the index of the first, how many words were never
 * written, the bounding box of the failures, and the four corners.
 *
 * @param  out_bad  if not NULL, receives the count outside the gap bound
 * @retval true if the whole 320x320 came back the grating, within the bound
 */
bool gpu_filter_probe(uint32_t* out_bad);

/**
 * @brief  Bring up the SIGNED convolve, nsgpu_convolve(), at 320x320.
 *
 * The separable call above cannot express a negative tap - its weights are
 * `(u8)(coeff * scale * 255 / max)`, converted through VCVT.U32.F32, so a tap of
 * -2 becomes 0.  nsgpu_convolve() is the other half of the library and packs
 * `(int8)(coeff << 5)` instead, which is a real edge operator at 1/32
 * resolution.  Nothing had ever called it on this firmware, because reading the
 * library showed it needs the same missing-register workaround (r6, not r7) and
 * calling it unwrapped is what faulted the board.
 *
 * This is a bring-up probe and it is NOT in the trigger path - only boot calls
 * it.  It writes a single vertical step into GPU_SCALE_DST, runs the vendor's
 * own sobel coefficients through nsgpu_convolve_k() into GPU_SCALE_CONV, and
 * prints one horizontal profile across the step.  Against a step the three
 * candidate behaviours are three different shapes - passthrough keeps the bright
 * level for the whole right half, an unsigned blur smears the step, and a signed
 * gradient returns to zero on both flats with a bump only at the step - so the
 * profile alone says which one happened.  It does not assert where the response
 * lands: the position is the reading, not a precondition.
 *
 * Bring-up only.  It runs a call this firmware has never run, so it is the one
 * thing in the boot sequence that could fault, and it is kept in one function
 * with one caller so a hang is attributable to it and deleting it is one line.
 *
 * @param  out_bad  if not NULL, receives the count of words left unwritten
 * @retval true if the whole 320x320 destination was written
 */
bool gpu_convolve_probe(uint32_t* out_bad);

/**
 * @brief  Reach the D/AVE HD driver directly, and time a draw.
 *
 * Two questions, one boot, neither ever measured on this board.
 *
 * **Is `a_handle` the device?**  `gpu.c` already reads the library's global
 * `a_handle` by name and the linker resolves it, so "can we reach the driver"
 * was never the open question - "is its value a device" is.  `dhd_init()`
 * returns `dhd_handle_t` and `nsgpu_init()` stores that return in `a_handle`.
 * This checks it against the vendor's own configuration file
 * (`davehd_registerparams_custom.h`: 2 texture units, 3 colour passes, 4
 * constant colours) rather than assuming.  A handle that is not a device reads
 * a valid address and prints numbers that do not match, so it cannot fault the
 * board and cannot pass by accident.  Those three functions are real exported
 * symbols - `DHD_API_INLINE` is empty unless `DHD_INLINE_INTERFACE` is set - so
 * a local prototype reaches them and no new header is needed.
 *
 * **What does one draw cost?**  With two texture units a draw combines at most
 * two input planes, so a dense conv costs `Cout * ceil(Cin/2)` draws and a
 * depthwise one `Cin`; the real model is 109 440 draws per inference, 98.5% of
 * them dense.  The workload is draw-call bound, not pixel bound, and the only
 * timing on record is a 320x320 two-pass convolve - two draws - which cannot be
 * extrapolated.  This sweeps the same call over five sizes and prints the
 * microseconds, so the size-independent term (CLUT allocation, state
 * programming, job submit, blocking sync) can be read off.
 *
 * Bring-up only, boot only, never in the trigger path.  Every call it makes is
 * already exercised on every boot by gpu_selftest() and gpu_filter_probe().
 */
void gpu_dhd_probe(void);

/**
 * @brief  Five blits into GPU_SCALE_DST, each compared word by word, to say
 *         which part of the scaled-blit call the camera path is failing on.
 *
 * This exists because the mean-based agreement probe above cannot tell several
 * different faults apart, and all of them look the same there - a mean channel
 * difference in the hundreds:
 *
 *   - the texture path does not read the address it was given,
 *   - the destination is being written by someone else,
 *   - the scene moved between the two conversions,
 *   - the extent or the scale is out of range.
 *
 * A crop pass removes the first three at once.  Its source is GPU_CROP_SRC -
 * memory the CPU fills and nothing else writes, so it cannot change underneath
 * the comparison.  It compares raw 16-bit words, so one wrong texel is a
 * failure rather than a fraction of a mean.  And the extent is clipped to the
 * destination, so there is no edge arithmetic to get wrong.
 *
 * The first three passes each sit one variable away from `own64`, the 64x64
 * CPU-owned source at 1:1 that was measured bit-exact on 2026-09-16:
 *
 *   own64   64x64,  1:1            the baseline; a failure here means the probe
 *                                  itself is wrong, not the library
 *   own800  800x64, 1:1            changes only the SOURCE EXTENT
 *   cam64   64x64,  1:1            changes only the SOURCE ADDRESS, to the live
 *                                  frame buffer passed in
 *
 * All three are bit-exact: at 1:1 destination (x,y) must equal source (x,y) word
 * for word, and the expectation is the source itself.  All three now declare
 * pitch 800, which is the stride their rows are laid out at - the first version
 * of own64 did not, and a source walked at the wrong pitch on both sides agrees
 * with itself, so that pass could not have failed.
 *
 * **Measured 2026-09-16: all three pass, and everything scaled fails.** That is
 * the whole result this probe was built for. cam64 bit-exact kills "the texture
 * path cannot read the camera frame buffer"; own800 bit-exact kills "the extent
 * or the stride is wrong above 64 pixels"; own64 passing kills the probe itself.
 * What is left is the scale, and the production-geometry pass fails while doing
 * nothing else:
 *
 *   prod    800x480, scale 103/171  83200 of 102400 words wrong
 *
 * It is not a pure constant: prod has exactly 19200 words right, which is 60
 * whole destination rows of 320, and rows 0..7 are all in the wrong group. So
 * the scaled path emits correct pixels somewhere and GPU_CROP_STUCK elsewhere,
 * and the shape of that mixture is the question.
 *
 * **The first run of the sweep could not be read, and this is why.**  Nothing
 * cleared the destination between passes, and the sweep ran scale 256 first -
 * which writes all 102400 words correctly.  So on every later line a word equal
 * to the expectation might have been written by that pass or might have been
 * sitting there since the last one, and "the correct fraction falls as the scale
 * falls" is the shape of BOTH "the blit writes less" and "the untouched
 * leftovers are what remains".  Every pass now clears the destination to
 * GPU_CROP_SENT first, which turns each word into one of three answers: FILL
 * (this pass wrote the value that was due), GPU_CROP_STUCK (it wrote something
 * else) or SENT (it did not reach that pixel at all).  A count of FILL is
 * evidence of a correct write only because of that clear.
 *
 * So the probe runs the baseline trio, then:
 *
 *   the axis trio.  One source, 256x64, small enough that no extent is ever
 *   clipped, with the three scale combinations that separate the axes: x scaled
 *   and y not (`ax`), y scaled and x not (`ay`), and both (`axy`). If only `axy`
 *   fails, the fault is in what the two axes do together rather than in either
 *   one alone.  Measured 2026-09-16: all three fail, so it is not one axis.
 *
 *   a scale sweep, ten scales from 256 down to 8, one line each.  Scale 256 is
 *   the anchor - it is the identity, and the trio above proves it works - so a
 *   failure there means the sweep differs from the trio rather than that a scale
 *   is broken.  Measured 2026-09-16: 256 is bit-exact and every other scale
 *   fails, so the anchor holds and there is no cliff; the fault appears the
 *   moment the scale is not 1.0.
 *
 *   a prod pass, reported in full: the F/S/N counts, a per-row character map of
 *   all 320 rows (`F` all FILL, `.` all stuck, `S` all sentinel, `?` mixed), the
 *   count of all-FILL rows and the first of them, the first six words of row 0,
 *   and the source's own first and last word so that a bad fill cannot be
 *   misread as a bad blit. The map is what separates a rectangle of good rows
 *   from a periodic pattern from a scatter - three faults the counts alone read
 *   identically.
 *
 *   a ramp pass.  The same production geometry over a source that is a
 *   coordinate, `7x + 13y`, rather than a constant.  This is the only way to ask
 *   whether the scaled path reads the source AT ALL: under a uniform source
 *   "sampled and correct", "sampled and ignored" and "never sampled" all produce
 *   the same uniform output, so no count over a uniform field can separate them.
 *   If the output still comes back as the constant it comes back as under FILL,
 *   the scaled path never touched the texture.  Read from its raw words rather
 *   than its counts - FILL and STUCK both fall inside the ramp's 0..11820 range,
 *   so a handful of accidental matches is expected and means nothing.
 *
 * Everything from the axis trio onward runs against an all-FILL 800x480 source,
 * so a scaled pass needs no prediction about WHICH source pixels it reaches -
 * every one of them is FILL.  The 64-column FILL block the baseline trio uses
 * cannot serve there, because a scaled pass would have to say which columns it
 * lands in, and that is the very thing a scale fault would move.  The block's
 * `GPU_CROP_OTHER` past column 63 is therefore a 1:1-only device, which is what
 * makes a wrong stride visible in the trio.
 *
 * The bit-exact passes print the number of words that came back wrong, the index
 * of the first, and that word beside the one the GPU actually left.
 * GPU_CROP_OTHER where GPU_CROP_FILL was due means a stride or offset fault;
 * zeros or noise mean the address was not read at all.  The scaled passes print
 * `tag b<bad> f<first> F<fill> S<stuck> N<sent> o<other>` on one line, with a
 * legend printed once; the single-letter names are what fits in Message.
 *
 * `prod` and `ramp` each fill 768 KB and then read the destination back, so this
 * is a boot-time probe and not something to call in a loop.
 *
 * @param  cam_src,cam_w : the live frame buffer and its width, for cam64; pass
 *                         0 to skip that pass
 * @retval true if the probe ran at all (not a statement about the passes)
 */
bool gpu_scale_crop_probe(uint32_t cam_src, uint32_t cam_w);

/* ==========================================================================
   Stage 4: the model's own DEPTHWISE_CONV_2D on the GPU.

   The earlier verdict that "the model's convolutions cannot go on the GPU" was
   derived from nsgpu_convolve() and nsgpu_separable_convolve(), and it is true
   of those two entry points only: they quantise the kernel to
   `(u8)(coeff * scale * 255 / max)` - unsigned, normalised to the largest tap,
   one scalar gain for the whole image - which cannot carry this model's int8
   weights (symmetric, zero point 0, 45% of them negative, and a per-channel
   scale).

   The D/AVE HD driver underneath them has no such limitation.
   dhd_set_texture_kernel() writes the taps into the CLUT as raw BYTES, and the
   driver's own comment reads "(pointer is unsigned but weights can be signed)".
   Sign is selected by E_DHD_CLUT_KERNEL_SIGNED in the TEXTURE FORMAT, not by
   the kernel call.  So the model's weights go in unchanged and the operator is
   expressible after all.

   This section is the probe that establishes the two things which decide the
   design and which nothing on record answers.  See gpu_dw_probe().

   --- the buffer pair ---

   0xC0E00000 is the "acc" (accumulation) area in gpu.h's slot map: 1 MB the
   driver leaves alone because acc_mode is 0.  Two 192 KB planes are carved out
   of it, which is what the largest depthwise layer needs (82x82x24 = 161 376 B
   in, 40x40x24 = 38 400 B out; see the census in gpu_dw_probe).

   The slot has NO MPU region on purpose (see the long note at the top of this
   file), so it has the default Device mapping: the CPU's writes are not cached
   and the GPU sees them without any cache maintenance.  arch_dcache_flush_all()
   in this pack is an empty stub, so a cacheable scratch buffer would need
   SCB_CleanDCache_by_Addr() around every GPU job.  Spending the CPU a slow
   uncached byte loop is the cheaper mistake of the two, and SDRAM slowness is
   a known silicon defect that is explicitly out of scope.                   */
#define GPU_DW_SRC          0xC0E00000u     /* 192 KB, one 8-bit plane     */
#define GPU_DW_DST          0xC0E30000u     /* 192 KB, one 8-bit plane     */
#define GPU_DW_MAX_PLANE    0x00030000u

/* The probe's own sizes.  8x8 so that the source can encode its own
   coordinates in a single byte (y*8+x is 0..63), which is what makes the tap
   sweep readable: the output IS the coordinate of the texel that was read. */
#define GPU_DW_PROBE_W      8u
#define GPU_DW_PROBE_H      8u
#define GPU_DW_PROBE_N      (GPU_DW_PROBE_W * GPU_DW_PROBE_H)

/* --- the pointwise probe's scratch --------------------------------------

   gpu_conv_probe() runs at boot, before any inference, so it borrows the
   depthwise operator's two planes rather than growing the GPU slot: SRC0 and
   SRC1 are two RGB565 textures inside GPU_DW_SRC's 192 KB, and the destination
   is GPU_DW_DST's first kilobyte.  Two source planes rather than one because the
   two-unit test's whole question is whether the units can be pointed at
   DIFFERENT memory, and one buffer cannot ask that.

   The texture is one row of 64 texels duplicated over 8 rows.  The width is what
   the alignment readout needs - a 1x16 window's tap 0 lands somewhere in
   -15..0 relative to the sampled texel, so the leftmost 16 columns are where the
   window is partly outside the texture, and reading columns 24..48 keeps every
   tap inside.  Blue field = x & 31, so a single tap names the texel it read:
   out[t] = (t + off) & 31 reads `off` off the first output column directly. */
#define GPU_CV_SRC0         (GPU_DW_SRC + 0x00000u)
#define GPU_CV_SRC1         (GPU_DW_SRC + 0x10000u)
#define GPU_CV_DST          (GPU_DW_DST + 0x00000u)
#define GPU_CV_W            64u
#define GPU_CV_H            8u
/* CLUT bases for the two units, inside the 512 entries the probe allocates.  A
   1x16 ROW kernel is ceil(16/4) = 4 entries, so 64 apart is ample and each is
   far enough from the other that a shared offset could not be mistaken for an
   independent one. */
#define GPU_CV_CLUT_A       0u
#define GPU_CV_CLUT_B       64u

/* --- driver constants, derived from davehd_driver.h ---

   davehd_driver.h reaches gpu.c already, transitively through nsgpu.h, so the
   driver's own enum names are used in the source and no constant is copied by
   hand.  (gpu_dhd_probe() reaches dhd_get_*_count() through local prototypes,
   which is where the opposite was assumed - see the correction in gpu.c.)

   The one worth showing the arithmetic of is the texture format, because it is
   NOT a literal anywhere in nsgpu.lib and a grep for it therefore finds
   nothing - which cost a round.  The library computes it:

       DHD_TEXEL_ORG_FLAG       0x8000
       DHD_TEXEL_ORG_CLUT_SHIFT 5
       E_DHD_TEXEL_ORG_AL_8          = 10 | 0x8000            = 0x800A
       E_DHD_CLUT_KERNEL_SIGNED      =  5 << 5                = 0x00A0
       format                        = 0x800A | 0x00A0        = 0x80AA

   and blit_convolve in nsgpu_lib_dis.txt:28596 does exactly that:
   `LDRH r0,[r4,#0x20]` then `ORR r9,r0,#0xA0` (signed) or `#0x80`
   (E_DHD_CLUT_KERNEL, unsigned).  0x80AA is the answer to the "which constant
   selects signed mode" question that was left open.

   **None of that arithmetic is repeated in the code.**  It was written down
   here because the value is genuinely not a literal in the library and a grep
   for it finds nothing, which is what made it look unanswerable - but
   davehd_driver.h turns out to be already in gpu.c's translation unit
   (transitively, through nsgpu.h), so the source derives the same value from
   E_DHD_TEXEL_ORG_AL_8 | E_DHD_CLUT_KERNEL_SIGNED and never spells a digit of
   it.  Same for E_DHD_WAIT_WRITEBACK, which is the 0x1F8 blit_convolve passes
   literally.  The arithmetic is kept as the cross-check, not as the source of
   truth.  (blit_convolve's texture ADDRESS mode is off the kernel height:
   3 -> E_DHD_ACCESS_VIRTUAL_TILED_2, >= 4 -> _TILED_4, else linear.)        */

/**
 * @brief  Answer the two questions a depthwise convolution on this hardware
 *         turns on, neither of which is on record for this board.
 *
 * **1. Does the texture unit sign-extend the texel in signed-kernel mode?**
 *
 * The model's activations are int8 with zero point -128, so a texel byte 0x80
 * means -128.  If the sampler treats it as unsigned instead, it means +128, and
 * the two differ by 256 with no way to tell them apart after the fact.  A +128
 * pre-bias pass on the GPU would fix it - but only if the difference really is
 * a constant 256.  Unsigned reinterpretation is `in_q mod 256`, NOT `in_q+128`,
 * so for a value that is *already* positive the two readings agree and for a
 * negative one they are 256 apart: the error is data-dependent and **no
 * constant bias cancels it**.  So this is not a refinement, it decides whether
 * a whole extra pass is needed.
 *
 * The test is a constant source of 0x80, a centre-tap-only kernel, scale 1.0
 * and bias 128.  Sign-extended the sum is -128, so the output byte is
 * -128 + 128 = 0.  Unsigned it is +128, so 128 + 128 = 256, which clamps to
 * 255.  Zero against 255, with nothing in between to argue about.
 *
 * **2. Does the kernel window step by one texel, or by the destination pixel's
 *    UV delta?**
 *
 * This decides whether the model's NHWC channel planes can be gathered
 * straight off the tensor with du/dx = C, or have to be de-interleaved into
 * planar scratch by the CPU first.  (The write side is not expressible either
 * way - the framebuffer writer advances x by one pixel per pixel and cannot
 * stride by C - so the output needs a CPU pass regardless.  The input is the
 * half that can be won.)
 *
 * The test uses a source that encodes the x coordinate in the byte itself,
 * `src[y*8+x] = x`, and a single-tap kernel swept over all nine positions: the
 * output byte then IS the x of the texel that tap read.  With a 1:1 blit
 * (du/dx = 1/8, destination 8 wide) the output is `8x + step`, and `step` is
 * the answer - 1 for a one-texel window, something else for a gradient-following
 * one.  Running the same sweep with a destination 4 wide and du/dx = 2/8 - two
 * texels per pixel - separates them: a one-texel window still gives `2x + 1`,
 * a gradient-following window gives `2x + 2`.  Same buffers, same source, one
 * different framebuffer size.
 *
 * **3. And then the thing itself**: a real signed 3x3 over a pseudo-random
 * field, compared byte for byte against the same convolution the CPU computes
 * from the same bytes.  The two tests above say whether the hardware CAN do it;
 * this says whether it then agrees, which is the only claim that matters.  The
 * comparison is interior-only (x,y in 1..W-2) so that the border policy - a
 * separate question with its own answer - cannot contaminate the reading.
 *
 * Also times one plane at 64x64, because every timing on record for this GPU is
 * from the nsgpu_* wrappers, which allocate and free a CLUT per call; this path
 * allocates one for the whole layer and re-points it per channel, so their
 * numbers do not transfer.
 *
 * Boot only, never in the trigger path, additive: it touches GPU_DW_SRC/DST and
 * nothing else in the firmware.
 *
 * @param  out_bad  if not NULL, receives the count of interior bytes outside
 *                  the 1-LSB bound in test 3
 * @retval true if the GPU was up and every pass ran (NOT a claim that the
 *         answers came out the expected way - they are printed, not asserted)
 */
bool gpu_dw_probe(uint32_t* out_bad);

/* ===================================================================
 * The pointwise convolution: the three questions Step 0 could not answer.
 *
 * tools/conv_gpu_emulate.py settled the ARITHMETIC of a 1x1 convolution on this
 * hardware offline, over the model's real activations: 42 of the 52 CONV_2D
 * layers come out clean, and the two design decisions that got it there (the
 * texel carries code = a + 128, and every chunk is split into a hi and a lo
 * plane) were both found there rather than on the board.
 *
 * What it could not settle is whether the ENGINE does what the arithmetic
 * assumes, because that is a property of this chip and not of the model:
 *
 *   1. **Does a 1xN ROW kernel sum N consecutive texels?**  The driver has a
 *      dedicated separable branch for height==1 or width==1
 *      (davehd_driver_txp_inl.h:581-598) and packs ceil(n/4) CLUT entries, so
 *      the vendor code says yes.  What this board has actually run is 3x3 and
 *      5x5 QUAD kernels and a 3-tap binomial through nsgpu_convolve.  Nothing
 *      has ever read a 1x16, and 16 is the kernel width ceiling - it is the
 *      whole reason a chunk is 32 taps and not 4.
 *   2. **Do two texture units work in one draw**, each with its own CLUT offset
 *      and its own UV origin?  The registers are there (TXP_Clut_Offset_0/1,
 *      m_txa_filter_kernel_width_1, dhd_set_uv's unit argument, and
 *      dhd_active_texture_unit recomputing m_tex_unit_base as a 7-word block per
 *      unit) - but nothing in this firmware has ever called
 *      dhd_active_texture_unit() with anything but 0.  If this fails, a chunk is
 *      16 and not 32, K doubles on every layer with Cin > 16, and the model's
 *      draw count goes up by about 60%.
 *
 * Both are one measurement each, and the design is worth nothing until they are
 * made - which is why this probe is a separate flash round and why the plan says
 * 回答不了就不往下走.
 *
 * The third question - what the K-digit fold's byte error really is on real
 * data - is not asked here.  It is asked offline and with far better coverage
 * than a probe could get: that is what conv_gpu_emulate.py's per-layer table is,
 * and it runs the fold over every one of the 52 layers' real activations rather
 * than over a synthetic chunk.
 *
 * Boot only, never in the trigger path, additive.
 *
 * @param  out_bad  if not NULL, receives a bitmask of the tests that did NOT come
 *                  out as predicted (bit 0 = 1xN alignment/step, bit 1 = tap
 *                  count, bit 2 = two units, bit 3 = per-unit CLUT, bit 4 =
 *                  per-unit scale).  Zero means every reading matched the
 *                  prediction, which is the only outcome that lets step 2 be
 *                  built on 32-tap chunks.
 * @retval true if the GPU was up and every pass ran
 */
bool gpu_conv_probe(uint32_t* out_bad);

/* ===================================================================
 * The two vendor blits, rewritten in gpu.c: what the version below is FOR, and
 * the one thing it is not.
 *
 * blit_convolve_new() and blit_separable_convolve_new() (both static in gpu.c)
 * are blit_convolve() and blit_separable_convolve() with the three capability
 * gaps of the library versions closed - a gain and a bias per OUTPUT CHANNEL
 * instead of one per call, an explicit sampling STRIDE, and an explicit
 * destination FORMAT so an 8-bit plane can be written.  The library versions
 * cannot be called at all anyway: each keeps its kernel buffer in a register it
 * never assigns (r6 / r7), which is what the naked wrappers in gpu.c exist to
 * patch around and what makes an unwrapped call a hard fault on one path and
 * garbage coefficients on the other.
 *
 * All three gaps are already closed one level down, in dw_pass_bc(): per-channel
 * gain by the caller looping, stride through the UV gradient, 8-bit through
 * dhd_set_framebuffer_format.  The new pair is that solution lifted to the
 * blit-level primitive the vendor's name promises, so a future operator can call
 * a convolution blit rather than re-deriving one - it is NOT a new driver
 * sequence, and it deliberately shares dw_pass_bc()'s call order, its
 * four-way ambient save/restore and its bounded drain.
 *
 * **What they do not do is make an offload bit-exact.**  The colour unit still
 * requantises by truncating toward zero and its only egress is one byte, while
 * this model's partial sums need 11..15 bits (tools/chain_emulate.py --range);
 * and the GPU equals the CPU only when the per-channel requantisation
 * multiplier is an integer, which it is for 0 of 1032 channels.  One offloaded
 * layer of 19 is already enough to change a decoded plate (--sweep).  So
 * anything built on these is judged by a byte-for-byte comparison against the
 * CPU before it ships - closing the three gaps makes an offload expressible,
 * not correct.
 *
 * gpu_blit_probe() below is the boot-only verification of the pair.
 * =================================================================== */

/**
 * @brief  Boot-only verification of blit_convolve_new() /
 *         blit_separable_convolve_new().
 *
 * Five readings, all against numbers already on record rather than against this
 * probe's own arithmetic: the per-channel loop (five channels, four distinct
 * predicted bytes out of four different planes), the stride (a step texture and
 * a one-tap kernel, so the edge position IS the sampling grid), the four ambient
 * framebuffer items restored, the two-unit row accumulator's n-texel window gap,
 * and whether the row-then-column mode's second pass really filters the other
 * axis (a 3x3 block is nine pixels where a second row pass gives three).
 *
 * Two of the five are BANDS rather than values, because the hardware's tap-0
 * position is only pinned to a texel (dw_kstride()); a band that excludes every
 * wrong reading is worth more than a point that assumes one, and the comments in
 * gpu.c say which wrong readings each band excludes.
 *
 * Boot only, never in the trigger path, additive.
 *
 * @param  out_bad  if not NULL, receives a bitmask of the tests that did NOT come
 *                  out as predicted (bit 0 = the first call refused, bit 1 =
 *                  per-channel loop, bit 2 = stride, bit 3 = ambient restore,
 *                  bit 4 = the two-unit call refused, bit 5 = two-unit window
 *                  gap, bit 6 = column pass, bit 7 = the transposed mode's call
 *                  refused, bit 8 = the rank-1 3x3's call refused).  Zero means
 *                  every reading matched.
 * @retval true if the GPU was up and every pass ran
 */
bool gpu_blit_probe(uint32_t* out_bad);

/* ===================================================================
 * WHERE THESE TWO OPERATORS SIT RELATIVE TO CMSIS-NN - and why there is no
 * middlewares/gpu/GPU-NN tree of gpu_convolve_s8.c standing beside CMSIS-NN's
 * arm_convolve_s8.c.
 *
 * The CPU path really is CMSIS-NN.  The project selects the pack component with
 * Cvariant="CMSIS-NN" (Printf.uvprojx, Machine Learning / TensorFlow / Kernel),
 * and in that build tflite::Register_CONV_2D() is cmsis_nn/conv.cpp:484, which
 * returns RegisterOp(Init, Prepare, EvalInt8); EvalInt8:347 reaches
 * EvalQuantizedPerChannel, whose int8 arm is arm_convolve_wrapper_s8 at
 * cmsis_nn/conv.cpp:202.  Register_DEPTHWISE_CONV_2D() is
 * cmsis_nn/depthwise_conv.cpp:511 and ends in arm_depthwise_conv_wrapper_s8 at
 * :294 of the same file.
 *
 * So the correspondence already exists - it is drawn at the level of the HOOK
 * rather than the level of the function name:
 *
 *   CMSIS-NN / TFLM                          this firmware
 *   ------------------------------------     ----------------------------------
 *   arm_depthwise_conv_* family        <->   gpu_dw_conv()          (above)
 *   arm_convolve_* family              <->   gpu_conv()             (below)
 *   cmsis_nn/depthwise_conv.cpp        <->   } dw_gpu_invoke()   \
 *   cmsis_nn/conv.cpp  Eval / Prepare  <->   } conv_gpu_invoke()  |  yolo_
 *   Register_DEPTHWISE_CONV_2D()       <->   } s_dw_stock         |  inference
 *   Register_CONV_2D()                 <->   } s_conv_stock       /  .cpp
 *
 * yolo_inference.cpp:1962-1978 is the load-bearing part.  The stock registration
 * is KEPT - s_conv_stock = tflite::Register_CONV_2D(), s_dw_stock =
 * tflite::Register_DEPTHWISE_CONV_2D() - and what goes into the resolver is that
 * same POD with only `prepare` and `invoke` swapped (TFLMRegistration is a POD,
 * micro_common.h:23).  THE CPU FALLBACK IS arm_convolve_wrapper_s8, reached
 * through s_conv_stock, unchanged.  That is why nothing here re-implements
 * CMSIS-NN: the stock kernels were never replaced, only bypassed on the layers
 * that qualify.
 *
 * Three reasons a gpu_* mirror of the CMSIS-NN file names is the wrong shape,
 * recorded so it is not re-proposed:
 *
 *   1. THE LAYER ORDINAL.  Eligibility is keyed on the layer NUMBER, and that
 *      number is captured in prepare because THE MODEL REUSES FILTER TENSOR
 *      INDICES - three of them each serve two CONV_2D ops, which silently cost
 *      the three 10x10 detection-head layers before it was found
 *      (tools/conv_filter_index.py finds it offline, no board).  conv_gpu_invoke()
 *      reads the ordinal as j.q->n.  arm_convolve_s8(ctx, conv_params,
 *      quant_params, input_dims, input_data, filter_data, ...) HAS NO SUCH
 *      PARAMETER, so a same-signature gpu_convolve_s8() could not ask "is layer
 *      47 eligible?" - it would have to re-derive identity from (filter pointer,
 *      dims), which is precisely the key that already proved to be non-unique.
 *
 *   2. THE INTERFACE.  arm_convolve_* is "one call, one whole layer".  What these
 *      operators actually need is not expressible in those parameters: the
 *      five-bit replicated-field contract (the multiplicand is
 *      ((c<<3)|(c>>2))/8, not c), the hi/lo two-plane split, the PER-UNIT
 *      centring (the hardware clamps EACH texture unit before the colour add),
 *      the refusal bound, a layer ordinal for the offline verdict, and the drain
 *      discipline.  gpu_dw_conv()'s and gpu_conv()'s raw-pointer signatures are
 *      the shape those need, and neither takes a TFLM header to be callable.
 *
 *   3. THE PROJECT FILE.  Every dhd_* call lives in gpu.c, which IS a project
 *      file; Printf2/middlewares/gpu exists but is EMPTY and is not listed in
 *      Printf.uvprojx.  New .c files need new <File> entries in that file, and
 *      hand-added entries vanish if uVision is open - the link error that
 *      follows reads like a code bug rather than like a project-file edit.  The
 *      gpu.c + yolo_inference.cpp split is partly there to never touch it.
 *
 * If a CMSIS-NN-SHAPED entry point is ever genuinely wanted - so the GPU is
 * callable from anywhere CMSIS-NN is, and not only from TFLM's resolver - the
 * right artifact is a THIN ADAPTER carrying arm_convolve_wrapper_s8's exact
 * signature that dispatches to gpu_conv() and falls through to arm_* otherwise,
 * not a re-implementation per file name.  Item 1 has to be solved before that
 * adapter can be written, because the ordinal is the thing it would be missing.
 */
/* ===================================================================
 * The depthwise convolution, on the GPU.
 *
 * gpu_dw_probe() above answers what this operator can be built out of, and the
 * answers are in the long comment at gpu_dw_conv() in gpu.c.  The three that
 * shape this contract:
 *
 *   - The kernel's multiplicand is five bits, 32 levels, in every texel format
 *     this chip has.  An int8 activation therefore loses its low three bits on
 *     the way in.  There is no 8-bit entry and no alpha path around it.
 *   - The hardware's bias field is nine bits signed, so the per-channel bias
 *     the arithmetic wants has to land in [-256, 255].
 *   - The accumulated sum survives as a signed intermediate - the bias is added
 *     before the output clamp - so a layer whose sum is negative is expressible
 *     and produces the right answer.
 *
 * So this is a real convolution and not an approximation of one, with one
 * bounded deviation: the activation is quantised to five bits, halfway-rounded
 * down, and the constant term of that truncation is folded into the bias.  What
 * is left is +-3.5 of activation quantisation noise per tap, which is
 * irreducible on this silicon.  The caller is expected to check the result
 * against the CPU on real data and keep whichever it prefers - see
 * yolo_inference.cpp, which does exactly that on the first layer it meets and
 * falls back permanently if the deviation is too large.
 *
 * @param  in        input activations, NHWC int8, h*w*c
 * @param  wq        depthwise weights as TFLM stores them, [1,3,3,c] int8 with
 *                   tap i of channel ci at (i*c)+ci
 * @param  biasq     the model's int32 bias, c entries
 * @param  in_scale  input quantisation scale per channel
 * @param  w_scale   weight quantisation scale per channel
 * @param  out_scale output quantisation scale (per tensor, so a scalar)
 * @param  out_zp    the output tensor's int8 zero point
 * @param  h, w      the INPUT plane's height and width
 * @param  oh, ow    the OUTPUT plane's height and width.  Passed rather than
 *                   derived so that the caller's tensor geometry and this
 *                   function's cannot disagree; they are checked against the
 *                   stride below and a mismatch is a refusal.
 * @param  stride    1 or 2.  At 1 the geometry must be SAME-padded and oh==h,
 *                   ow==w.  At 2 it must be 3x3 VALID, i.e.
 *                   oh==(h-3)/2+1, ow==(w-3)/2+1 - the shape of this model's
 *                   six stride-2 depthwise layers, whose input a preceding PAD
 *                   op has already widened, so no tap ever falls outside.
 *                   Anything else is refused.
 * @param  split     0 to quantise the activation to five bits and accept the
 *                   residual; 1 to draw a SECOND pass over the three bits the
 *                   first one dropped and add it back, which recovers the whole
 *                   value.  The residual is zero-mean and bounded by
 *                   gpu_dw_bound(); at this model's stride-1 weight scale that
 *                   bound is small enough to ignore, and at the stride-2 scale
 *                   it is not.  Split costs one extra packing pass and one
 *                   extra draw per channel - measured, not free, and the
 *                   packing is the expensive half.
 * @param  out       output activations, NHWC int8, oh*ow*c.  UNTOUCHED on refusal
 *
 * @retval 0   the layer ran on the GPU and `out` is wholly written
 * @retval -1  refused; the caller must use the CPU path.  `out` is untouched
 *             and the reason is counted in gpu_dw_stat()'s `refused`.
 *
 * A returned 0 MAY HAVE TAKEN TWO ATTEMPTS.  A failed drain is a give-up and a
 * permanent latch, and the two states it conflates - a GPU that stopped and a
 * job that was never submitted - look identical from a frozen register, so the
 * operator re-issues the same draw ONCE A BOOT (`gpu dw redraw`, and
 * `gpu dw redraw ok sy<id>` when the queue answers).  A layer that needed it is
 * an ordinary success: the redraw writes the same bytes, and the latch is
 * withdrawn because a queue that has just answered a fresh job is not stopped.
 * The caller sees only "drew" and "refused" and needs no new state, but a log
 * with `gpu dw redraw ok` in it is not a log with no give-up in it.
 *
 * Not thread safe, not reentrant, and it changes the driver's global
 * framebuffer format to L_8 for the duration - it saves and restores it, but a
 * second caller would interleave badly.  The trigger path is single threaded.
 */
int gpu_dw_conv(const int8_t* in, const int8_t* wq, const int32_t* biasq,
                const float* in_scale, const float* w_scale, float out_scale,
                int32_t out_zp, int h, int w, int c, int stride,
                int oh, int ow, int split, int8_t* out);

/* What gpu_dw_conv() has actually done since boot.  This is the evidence for
 * "the operator runs on the GPU" that the stage is judged on, and the reason
 * the CPU-side packing cost is reported rather than hidden.
 *
 * `packed` and `unpacked` are activation counts, not bytes: multiply both by 3
 * for the bytes moved through the uncached scratch.  Any argument may be NULL.
 */
void gpu_dw_stat(uint32_t* layers, uint32_t* draws, uint32_t* refused,
                 uint32_t* packed, uint32_t* unpacked);

/* HOW MANY OF THE OPERATOR'S DRAWS WROTE NOTHING, over every draw and not the
 * two per inference `gpu dw pl2` can see.
 *
 * Three bytes with three different values are stored into the region each draw
 * covers and read back after its drain, so "the draw wrote nothing" is a
 * reading and not an inference from a second-hand plane.  See the block at the
 * definition in gpu.c for why a dropped draw is a silently WRONG activation
 * rather than an obviously missing one, and for the single redraw that repairs
 * it.  All arguments may be NULL.
 *
 * `chk`   draws the check was armed on since boot (the operator's own, only)
 * `drop`  of those, the draws that left all three sentinels untouched
 * `re`    of the drops, the ones whose ONE redraw also left all three - i.e.
 *         the failure is not a lost job
 * `first_d`/`first_c`/`first_p`  the first drop's draw ordinal (1-based, so 0
 *         means none yet), its channel, and its pass (1 or 2)
 */
void gpu_dw_drop_stat(uint32_t* chk, uint32_t* drop, uint32_t* re,
                      uint32_t* first_d, uint32_t* first_c, uint32_t* first_p);

/* THE PLANE PROBE - one differential per inference, for the first two layers.
 *
 * The chain fingerprint says the first eligible depthwise layer's INPUT is
 * byte-identical across runs while its OUTPUT is not, and `gpu dw gpudiff`
 * says a back-to-back redraw of that same layer is byte-identical.  Those two
 * cannot both hold for a draw that is a pure function of its operands, so what
 * differs between runs is the ambient state the draw reads or one of the two
 * scratch planes it sits between - and gpudiff, which redraws with no pack and
 * no unpack in between, is structurally blind to both.
 *
 * Arming it makes the NEXT dw_pass_bc() print, once, immediately before its
 * dhd_draw() and again after its wait:
 *
 *   gpu dw pl <tag> s<src> d0<pre>      and gpu_env_state(<tag>)'s three lines
 *   gpu dw pl <tag> d1<post>
 *
 * `s` is the packed source plane as the pack left it, `d0` the destination
 * before the draw, `d1` the destination the CPU is about to unpack.  Two runs
 * agreeing on `s` and `d0` but not on `d1` say the draw is not a function of
 * its operands and its state; a run whose `d0` differs says the plane held a
 * different leftover; a state line that differs names the leak directly.
 *
 * It draws nothing, allocates nothing and touches no depthwise arithmetic.  It
 * disarms itself on the first draw it sees, so arming it per layer is bounded.
 * The caller arms it for the first two eligible layers - the two gpudiff
 * tested - because that is where a disagreement with gpudiff means something.
 */
void gpu_dw_plane_arm(void);

/* THE TWO LATCHES, readable - and clearable exactly once, on purpose.
 *
 * gpu_drain() sets BOTH of them from a single drain that does not finish, and
 * nothing else in the firmware ever clears them: a board whose GPU stops is a
 * CPU-only board until it is reset.  That is the right behaviour for the
 * product - the scratch planes may still be held by a job that never returned,
 * so handing them to the next layer is how a refusal becomes a wrong answer -
 * but it is a poor instrument, because "the GPU is dead" and "the GPU stalled
 * for a while" are different findings that lead to different next steps, and a
 * latch that only a reset opens cannot tell them apart.
 *
 * Reading them is also what makes a capture self-describing.  The `gpu dw L%u
 * D%u R%u` and `gpu cv l%u d%u` lines are CUMULATIVE counters, so an inference
 * whose layers all went to the CPU is written identically to one whose layers
 * were all ineligible - the difference is whether the latch was already set,
 * and that is now its own line above those counts.  It is printed only while a
 * latch is set, so it costs nothing on a healthy board.  (It is still its own
 * line rather than a field on the stat lines, but the reason has changed:
 * `Message` is char[192] since 2026-10-20 (160 from 2026-09-23, and char[64]
 * before that), so a field would fit now.  See gpu_dw_drnmax() for the
 * counter-example - a field can be
 * the right shape when it is the same quantity.)
 *
 * Any argument may be NULL.  The values are 0 or 1.
 */
void gpu_dead_stat(uint32_t* dw, uint32_t* cv);

/* THE WORST HONEST DRAIN OF THE BOOT, in polls - the tail of the drain
 * distribution, which is the number GPU_DRAIN_LIMIT has to be sized against and
 * the one number no other reading in this file supplies.
 *
 * `gpu dw drn %u cy%u` prints the first FOUR drains of a boot and then stops, so
 * every drain reading that has ever been taken is an early-layer, small-plane
 * one - and the late layers are the big planes, 96 output channels each.  Nine
 * such readings agreeing to within a factor of two (3378..6922) is what a biased
 * sample looks like, not what a safe margin looks like.  This is read once per
 * inference and printed on the summary line, which costs one field on a line
 * that already prints.
 *
 * A give-up is never in here: the accumulator is written only by a drain that
 * saw the sync id move, so a latched boot reports its last honest drain instead
 * of the limit.  Polls are 63.5 ns each (38.06 cycles at 600 MHz, four
 * independent readings); the give-up itself measured 1.367 s on 2026-09-24.
 */
uint32_t gpu_dw_drnmax(void);

/* THE DRAIN'S OWN SPLIT, boot-wide, in microseconds.  `n` = drains counted;
 * `wait` = the `dhd_wait(WRITEBACK|CLEAR|STW)` barrier; `gate` = the fetch gate;
 * `flush` = `dhd_flush()`, i.e. the driver walking and submitting the command
 * list; `poll` = the sync-id poll loop, i.e. the GPU actually executing.
 *
 * WHY IT EXISTS.  blit_tile() has split a draw four ways since it was written
 * and the board reads `bl set 0 drw 46 wai 0 drn 59069` for a 59155 us stage-3
 * call - so the cost of a GPU draw is inside gpu_drain(), and until now the only
 * thing the firmware could say about it was which register to blame.  This says
 * which PART.  Read it as: `flush` dominating = the driver's command path, which
 * is CPU work and ours to fix; `poll` dominating = the GPU really is that slow
 * at that draw; `wait` dominating = the queueing model at the barrier's call
 * site is wrong; `gate` can be cross-checked against gpu_fetch_gate_stat()'s
 * `spent` at 38.06 cycles a poll.
 *
 * The four sum to the whole call except for the two instruction gaps that
 * bracket the stamps, so a reading that does not roughly add up is one to
 * distrust.  A give-up is counted too - a boot that gave up on its first drain
 * must not read like one that never committed anything.  Any argument may be
 * NULL.
 */
void gpu_drain_split_stat(uint32_t* n, uint32_t* wait_us, uint32_t* gate_us,
                          uint32_t* flush_us, uint32_t* poll_us);

/* THE FETCH GATE's readings, in polls.  `gated` = drains that ran it,
 * `waited` = of those, the ones that found the fetcher behind the pause address
 * (Current_Ring_Address != Ring_Pause_Address), `timeouts` = the ones that used
 * the whole GPU_FETCH_GATE_LIMIT and flushed anyway, `worst` = the most polls
 * any one gate spent, `spent` = every poll the gate has ever waited, boot-wide.
 *
 * The gate is the 2026-09-28 change, and it is the SECOND one: the STC dump
 * named the wedge (`Interrupt_Status` bits 8..10 = `m_stop_code` = 2,
 * `E_STOP_IRQ_CODE_START_WHEN_BUSY`) and the stopped stream was the drain's own
 * four-word batch.  The first gate waited for `Busy_Status == 0` before the
 * flush that can write STREAM_ADDRESS, and its own counters killed it -
 * `n94665 w94664 to94664 mx20000`, i.e. the condition is never true while the
 * stream is alive (that bit reads "running", not "busy") at a cost of ~2.8 s per
 * frame.  This gate waits for `Current_Ring_Address == Ring_Pause_Address`, the
 * one state the two dumps actually differ in: the fetcher has consumed
 * everything the driver allowed it, so the STC is parked rather than mid-stream
 * when a start signal lands.  See the block at GPU_FETCH_GATE_LIMIT in gpu.c.
 *
 * Why these are printed rather than trusted: a boot that never wedges looks
 * IDENTICAL whether the gate prevented the fault or never fired at all, and
 * that is the difference between a fix and a knob that is not being set - the
 * previous gate's counters are the proof that this is not a theoretical worry.
 * So they ride the per-frame summary line, which is written only when a key asks
 * for a frame.  `timeouts == gated` means the condition is never true at all and
 * the gate is a pure delay; any nonzero `timeouts` means the limit wants
 * raising.  `worst` is the number to raise it against (63.5 ns a poll).
 *
 * `Code=201246` read `n298 w1 to1 mx2000`: the condition is real and nearly
 * free, and the ONE drain that waited spent the whole (then 2000-poll) limit -
 * so the wedge on that same boot is either a gate that ran out of polls or a
 * gate that saw the fetcher parked and was irrelevant.  The two fields added
 * with the raised limit decide which, and neither is a per-frame count: `drain
 * dead`'s `gw` field prints the FAULTING drain's own gate wait (`gw0` = the
 * fetcher was parked and the start was refused anyway), and `spent` is the
 * budget reading - once it reaches GPU_FETCH_GATE_BUDGET the gate stops waiting
 * and every later drain counts into `timeouts`, deliberately, so a gate that has
 * stopped running cannot read like one that found nothing to do.
 *
 * Any argument may be NULL.
 */
void gpu_fetch_gate_stat(uint32_t* gated, uint32_t* waited, uint32_t* timeouts,
                         uint32_t* worst, uint32_t* spent);

/* THE START WATCH's readings: the ten-bit busy vector and the commits that wrote
 * a start.  `seen` = OR of the vector over every drain that reached its commit,
 * `starts` = commits that really wrote STREAM_ADDRESS, `starts_nz` = how many of
 * those had a busy vector, `starts_or` = OR of those vectors.
 *
 * Why this exists, and it is a falsification rather than a hunch: `Code=201246`
 * printed `gw0` on the faulting drain - its own gate saw
 * `Current_Ring_Address == Ring_Pause_Address` immediately before the commit that
 * killed the STC.  So the fetch pointer does NOT predict the fault, for exactly
 * the reason `Busy_Status` bit 0 alone did not: both read "the stream is alive
 * and waiting" and both were taken as "the engine is at rest".
 *
 * `Busy_Status` is not a flag but ten independent busy bits
 * (davehd_registertypes.h:789-804: stc, str, stw, reg, psu, txi, tex, col, blend,
 * clear), and the two readings that exist say the healthy parked stream is bit0
 * ALONE (`busy00000001`) and the post-fault state is nothing busy
 * (`busy00000000`).  The dead line's `b%08X` prints that vector as it stood one
 * instruction before the fatal commit - a reading that cannot be taken after it.
 * A lone bit0 there means a start was refused by a live parked stream with every
 * unit idle; unit bits mean the start raced work still draining.  The two want
 * opposite fixes, so it is measured before anything is changed.
 *
 * The counts are the denominator that reading needs: STREAM_ADDRESS moves only
 * when the driver is not ACTIVE, so `starts` says how rare a start really is and
 * `starts_nz`/`starts_or` say whether a busy vector at a start is ordinary or
 * exceptional.  The count can be low by ~1% when the ring wraps onto the same
 * address, which cannot change either answer.
 *
 * All four ride the per-frame summary line, which is only written when a key asks
 * for a frame.  Any argument may be NULL.
 */
void gpu_busy_vector_stat(uint32_t* seen, uint32_t* starts,
                          uint32_t* starts_nz, uint32_t* starts_or);

/* THE DRIVER'S OWN STATUS WORD - the other half of the start watch, and the half
 * that says WHY a start was written at all.
 *
 * `dhd_gpu_status_t` (davehd_kernel_driver.h:26-31) is 0 IDLE, 1 HALTED,
 * 2 ACTIVE, and it lives in the library's `s_devices[0].m_status`.  The counters
 * are a histogram over every drain that reached its commit; `last` is the word as
 * it stood at the drain that is reporting (the `drain dead` line's `ms` field).
 *
 * The mechanism it measures: davehd_kernel_ring.c:199-208 writes
 * STREAM_ADDRESS - a start - ONLY when m_status != ACTIVE, while it writes
 * RING_PAUSE_ADDRESS unconditionally.  `Code=201638` then counted the whole boot
 * at `strt1 nz1 sor001`: of 1414 gated commits exactly ONE wrote a start, and
 * that one is the commit that wedged the STC, with a busy vector of `00000001`.
 * So the fault is "a start, landing on a stream that is alive and parked", and
 * its precondition is the driver believing the engine is idle while bit 0 of the
 * busy vector says it is not.
 *
 * That leaves two readings, and they want OPPOSITE fixes:
 *   `ms2 b00000001` - the driver is out of sync with the hardware.  Repair by
 *                     keeping it ACTIVE when the STC is alive (`n`, which writes
 *                     only this word and touches no hardware).
 *   `ms0 b00000001` - the engine really did stop and the start landed in the
 *                     tail of the stop.  Keeping ACTIVE would only delay the
 *                     start; this one wants a settle/wait instead.
 * Measured 2026-09-28, and it is the SECOND row that the board printed: arm A
 * (keep off) wedged on 4 of 4 boots, every one of them on the FIRST frame, with
 * `b00000001` every time - and the one of those four that also carried the `ms`
 * field read `ms0`.  That boot's histogram is the sharpest single reading of the
 * diagnosis: 734 of 736 commits were ACTIVE, so the driver leaves ACTIVE exactly
 * at the wedge and nowhere else.  Arm B (keep on) ran 18 frames and 37594 drains
 * with the keep firing 3 times and no wedge, and every drain behind each of those
 * 3 keeps SUCCEEDED - which is the half that proves the live-parked stream
 * resumes on a pause advance, i.e. that suppressing the start is a repair and
 * not a delay.
 *
 * `keeps` counts how many times the keep overrode the word.  Default 1 since
 * 2026-09-28, so on an ordinary boot it is the count of repairs the boot made by
 * itself rather than a record of a keypress.
 *
 * `last_stop` is `m_last_stop_code` at the same instant - the stop code the ISR
 * recorded for the stop that left the driver in the state above.  It is what
 * separates the two rows for good: IDLE is reachable from a clean STREAM_END
 * (= 0, the designed resume path), so `ms0` alone cannot say whether the start
 * that followed was legitimate.  `ms0 ls0` says the driver was resuming a
 * finished stream; anything else names a stop the hardware raised for another
 * reason.
 *
 * Any argument may be NULL.
 */
void gpu_drv_status_stat(uint32_t* n_idle, uint32_t* n_halted,
                         uint32_t* n_active, uint32_t* last, uint32_t* keeps,
                         uint32_t* last_stop);

/* The hardware sync id, and the only view of the GPU that survives a drain
 * giving up.
 *
 * `gpu_drain()` polls this register, and its `drain dead` line prints it twice -
 * but the two words are EQUAL BY CONSTRUCTION, because the loop falls through
 * only while the register still equals the value it started at.  So that line
 * cannot say whether the counter moved during its twenty million polls, and that
 * is exactly the difference between a GPU that STOPPED and one that is merely
 * BEHIND a job - the distinction the two latches' safety argument rests on.
 *
 * This accessor is what answers it.  A latched refusal touches no hardware, so
 * the value can be printed for free on a line that is printed anyway, once per
 * remaining layer of every remaining frame: a column of these across the rest of
 * a boot says whether the counter ever moved again.  It is the same access
 * gpu_drain() makes in a loop, so it cannot hang.
 *
 * Meaning, measured rather than assumed (gpu.c, 2026-09-21): it is NOT a job
 * counter.  Nine draws moved it by two, three draws by one - roughly once per
 * flush.  "The counter moved" therefore never meant "our draws are done".
 */
uint32_t gpu_sync_id(void);

/* Re-arm the GPU after a latch.  This is the `k` key in main.c and it is the
 * whole reason the latch is clearable at all.
 *
 * It clears both latches and arms the cheap drain budget (the short form of
 * GPU_DRAIN_LIMIT, in gpu.c), so the next inference answers "did the GPU come
 * back" in 127 ms instead of 1.367 s - both measured on 2026-09-24, and both of
 * them what ONE drain costs, because the first failure re-latches and the layers
 * behind it then refuse without touching hardware.  A drain that SUCCEEDS
 * disarms the cheap budget again.
 *
 * THIS IS A QUESTION, NOT A FIX.  If the GPU is stopped rather than stalled,
 * the next inference re-latches on its first drain and every layer after it
 * goes to the CPU, exactly as before - the product is never worse off than the
 * latched state it was already in.  What the key buys is that the two cases can
 * be told apart without a reset, because being clearable repeatedly is the only
 * way to find out whether a stop is permanent or intermittent.
 */
void gpu_revive(void);

/* Where gpu_dw_conv() is right now, as the DVP interrupt handler reads it.
 *
 * 0 means "not inside a draw" - the operator is either refused, finished, or
 * has not started - and the reporter treats that as no information rather than
 * as a stall.  Anything else is
 *
 *     1 + stage*100000000 + channel*100000 + pixel
 *
 * and the stage codes are listed at the definition in gpu.c.  The handler
 * prints a line when this value has not moved for about a second, which is the
 * only way to name the point a silent hang is stuck at: every other reading in
 * this file is taken either before a layer or after it, and the code that stops
 * is precisely the code that cannot print.
 *
 * Written through gpu_front_prog() (static, in gpu.c) and directly by
 * gpu_dw_conv()/gpu_conv().  "Only gpu_dw_conv() writes it" was true once and is
 * not any more: the front end arms it across its whole draw (stages 38/39) and
 * so does the BOOT path - gpu_init() at 20/21 and gpu_selftest() at 6..9, both of
 * which reach unbounded library waits.  An `s20`/`s6..s9` in the log is a stuck
 * boot, not a stuck inference.  Read it from anywhere; the stage table is the one
 * at the DW_PROG definition in gpu.c, and it is the authority on which number is
 * whose.
 */
extern volatile uint32_t g_gpu_dw_prog;

/* THE DEPTHWISE OPERATOR'S ON/OFF SWITCH, and it is a runtime variable on
 * purpose.
 *
 * The operator is lossy by construction - the GPU's texture is a five-bit
 * unsigned field, so every activation it reads has its low three bits
 * truncated - and it runs on 19 of the model's 27 depthwise layers, in the
 * middle of the network, where the error feeds the next layer's input.  The
 * per-layer check that gates it measures ONE layer on ONE frame and reads
 * `worst 4`; it does not and cannot measure nineteen of them in series.
 *
 * A compile-time gate would make the answer to "did the GPU cause this
 * detection" cost a flash round, and the frames would differ between the two
 * builds anyway, so the comparison would be between two scenes as much as
 * between two paths.  This is read instead of #ifdef'd so that the switch can
 * be flipped BETWEEN TWO TRIGGERS of the same still scene, seconds apart, with
 * nothing else changed - which is the only form of that comparison that
 * measures the path.
 *
 * 0 (the reset value) means every depthwise layer runs on the CPU.  This was
 * flipped to 1 for one round on 2026-09-22, on the argument that a qualification
 * measured on the board's own frame is proof enough and that an offload nobody
 * presses a key for is not an offload the product has.  The board then read the
 * same still scene three times with only this switch and g_gpu_conv_on moved:
 * `pair gpu:` came back 黑 with 38 dw passes and 18 conv layers on the GPU, 25
 * with the 38 dw passes alone, and 8粤粤8888888 - a well-formed plate, 11 boxes -
 * with nothing offloaded at all.  So the qualification is proof of the wrong
 * property: it bounds ONE layer on ONE frame, and what the product reads is the
 * error of every offloaded layer in series.  The default is back to 0; the keys
 * still flip it either way.  gpu.c owns the storage and carries the three-arm
 * table; the serial command in main.c flips it and prints the new value.
 *
 * 2026-10-10 - THE ROUTE IS CLOSED, ON BOTH AXES.  The 2026-09-22 comparison was
 * taken between three TRIGGERS, so it measured the scene as much as the path.
 * Key `i` re-runs the inference on the RETAINED frame, which is the form this
 * switch was built for, and it was finally taken that way: one boot, one frame,
 * `grab 0 us`, nothing but `g` moved.
 *
 *     g=0   `89粤A8888888`, 3 boxes (cls 9 / 11 / 9), infer 1711221, 1710704 us
 *     g=1   `(no char)`, no boxes,                       6895334, 6932915 us
 *
 * (all four below the 7.158 s stamp rollover, so they are quotable as they read)
 *
 * The two g=0 runs are bit-identical in the boxes and in the string, and agree
 * to 0.03% in time, so the arm is deterministic on a held frame and this cannot
 * be read as scene variance.  The 1.711 s also reconciles with the `e` census's
 * all-CPU 1.696 s to 1% - two independent probes agreeing.  So the offload is
 * not a small accuracy loss that a better gate could trade against: it is 4.0x
 * SLOWER, and it is the difference between reading the plate and reading
 * nothing.
 *
 * And the chain error is not fixable - measured, not argued (tools/
 * chain_emulate.py, 14 frames).  The colour unit computes trunc(ST*asc/256) + bg
 * with the bias added AFTER the shift, so its output can only move by integers;
 * the only lossless egress is one 8-bit write while the partial sums need 11..15
 * bits; and GPU == CPU requires the requantisation ratio `m` to be an integer,
 * which holds for 0 of 1032 channels.  Every reachable correction was tried
 * (global gain, per-layer gain table, per-frame affine, DC bias) and restores
 * the box statistics while reproducing no plate string.  So there is no gate,
 * no tail value and no bias correction to look for: ONE offloaded layer of 19
 * already changes the decoded string, and the board's tail sweep confirmed it
 * from the other end - ordinal 18, the very last eligible layer, already flips
 * the plate.  Do not re-open this switch looking for detection. */
extern volatile uint32_t g_gpu_dw_on;

/* THE PER-LAYER DEPTHWISE TRACE'S SWITCH - the `L<n>` / `L<n> drew` pair, not
 * the operator, and the one switch here whose default is about COST rather than
 * about a result.  The trace changes nothing it measures.
 *
 * The pair exists because a run that dies in the third GPU layer prints the
 * same last line as a run that dies in the seventeenth: without it, "somewhere
 * in the inference" is all two different deaths have in common.  Nineteen pairs
 * buy the difference between that and "layer N", and both stalls of 2026-09-23
 * and 2026-09-24 were read out of them.
 *
 * It used to be automatic - once per boot, on the first inference after
 * g_gpu_dw_on went to 1, with the frame boundary detected by the layer ordinal
 * going backwards - so that 38 lines a frame would not land inside the interval
 * the `gpu dw L.. D.. R..` summary reports.  THAT WAS THE WRONG TRADE, for the
 * reason the automatic form was built to hide: the trace's cost is now measured
 * at somewhere between **0.2 s and 1.1 s a frame**, depending on which
 * cross-boot comparison is used, and the two readings cannot be reconciled -
 * because the only frame it ever printed on ALSO ran the two class checks,
 * worth ~1.16 s on their own (capture A: the check frame 4886 ms against 3724
 * production).  The one frame the trace ran on was never a clean one, so its
 * cost could not be measured; and re-arming it needed a power cycle, so it
 * could not be A/B'd within a boot either.  It is the same defect as a guard
 * whose sample coverage is part of its answer.
 *
 * So the frame-boundary logic is gone and this is a key like the others: `y`
 * flips it, and while it is 1 EVERY frame prints its 19 pairs.  That is what
 * makes the cost measurable - press `x` a few times, press `y`, press `x` a few
 * more, on one boot, one scene, one operator set, with the class checks already
 * latched out of the way after the first frame.  The 38 lines a frame are then
 * paid knowingly instead of hidden.
 *
 * OFF by reset, so an un-flipped board's log is exactly what it was.  gpu.c
 * owns the storage; main.c's key flips it and prints the new value. */
extern volatile uint32_t g_gpu_dw_trace;

/* THE FRONT END'S ON/OFF SWITCH, same idea and the same reason as the one
 * above: a runtime variable.  1 by reset as of 2026-10-08, so an un-flipped
 * board draws its 320x320 on the GPU; pressing `f` puts it back on the CPU
 * bilinear walk, which is what every reading before that date was taken on.
 *
 * This gates what the PRODUCT draws its 320x320 from - the blit and the
 * separable filter in camera_grab_rgb888().  It deliberately does NOT gate
 * camera_grab_pair(), because `p` is the instrument that compares the two
 * front ends on one frame, and an instrument that the switch turns off cannot
 * measure the thing the switch controls.
 *
 * THE TWO FRONT ENDS ARE NOT THE SAME PICTURE, and that stays part of the
 * argument even though it is no longer what the default stands on.  It was
 * never a brokenness verdict - see the correction below - it is that the arms
 * differ in a way this product had not validated:
 *
 *   - the CPU arm is a plain bilinear walk of the source at exactly 2.5
 *     pixels per output pixel;
 *   - the GPU arm is a scaled blit at 103/256, i.e. 2.4922 (a 0.3%
 *     difference in sampling phase), and THEN a 3-tap low-pass on the RGB565
 *     plane with the unpack afterwards, because GPU_FILTER_ENABLE is 1.
 *
 * So `pair cpu:` against `pair gpu:` is a FILTERED picture against an
 * UNFILTERED one, and on this product the model was fed through the CPU arm
 * alone until 2026-10-08, when this switch became the default.  What retired
 * that objection is a decode on the PRODUCT path, not the mean: `x` x6 with
 * the GPU arm on read 6/6 frames with a readable plate, against the CPU arm's
 * 6/6 on the same boot.  Note what that does and does not say - both arms
 * decode on this scene, neither is shown better, and a live camera cannot
 * repeat a frame, so scene variance and pipeline still cannot be separated.
 * Mean-channel readings never could capture any of it: the two look nearly
 * identical at `mean 4..5`, and mean is not what the model reads - it reads a
 * marginal, noisy decode.
 *
 * THE READING THIS USED TO REST ON IS VOID (2026-09-29).  It said the arms
 * were compared on one frame and "the CPU arm is the one that reads
 * characters", i.e. the GPU arm read none.  That was measured while
 * nsgpu_draw_rotblit was clipped to (0,0): the blit covered ONE TEXEL, so the
 * GPU arm's plane was not a picture at all and reading nothing off it was
 * forced.  `dhd_set_util_cliprect(0,0,W-1,H-1)` fixed that the same day
 * (gpu-hang-and-drain-fix carries the mechanism), and EVERY f-on detection
 * verdict taken before that fix is invalid - this one included.  After the fix
 * the reading reverses: the boot probe is `ok 1` on both legs (`mean 4` for
 * the blit, `filt mean 6` for the filter), `pair diff mean` is 6 with only `f`
 * on, inside camera.c's own single-digit criterion, and the plates in that
 * capture come off the GPU arm while the CPU arm reads none.
 *
 * SO WHAT KEPT THE DEFAULT AT 0 WAS SOMETHING ELSE, and it was the hard one:
 * with this on, the product's filter leg is the LIBRARY's
 * `nsgpu_separable_convolve`, whose two `nsgpu_cmd_sync(0)` calls are
 * `dhd_finish()` - AN UNBOUNDED WAIT.  A board whose GPU is latched dead hangs
 * the whole product inside it, which is the one place the "the GPU must never
 * take the main function down with it" rule was still violated.  Turning this
 * default to 1 was therefore NOT a one-line change: it needed stage 39 to own
 * its commit point first, exactly as stage 38 already does.  THAT IS DONE - see
 * g_gpu_front_sep_on below - so the default went to 1 on 2026-10-08, on the
 * measurement in gpu.c's block on this variable (`x` x6 per arm, one boot: the
 * GPU front end is 58..62 ms CHEAPER than the CPU bilinear path it replaces,
 * 6/6 detect with a readable plate on both arms, no wedge in 12 grabs).
 *
 * ITS PRICE WAS ONE KEYPRESS FROM THE LIBRARY'S UNBOUNDED COMMIT, AND IT IS PAID
 * (2026-10-09).  Pressing `s` alone used to land the product back on the
 * library's nsgpu_separable_convolve, whose tail is `dhd_finish()`.  `s` = 0 is
 * now filter_frame_lib(), that operator TRANSCRIBED with bounded commits, and
 * map no longer reaches nsgpu_cmd_sync from any product path - so the hole is
 * closed in both states, and what `s` = 0 draws is the picture the model has
 * always been handed.
 *
 * IT WAS REVERTED TO 0 FOR A FEW HOURS ON 2026-10-10 AND PUT BACK, and the
 * round trip is worth keeping: the revert was a hypothesis (a user's regression
 * report, timestamp-matched to the flip) and the reading that killed it is the
 * one every earlier paragraph was missing.  `p` decodes BOTH arms on ONE frame,
 * so the scene drops out, and 6/6 same-frame pairs favour the GPU arm - it
 * carries 粤A8 in 6/6 and never emits an F or an E, where the CPU arm carries 粤
 * in 2/6 and emits F/E in 3/6.  Neither arm reads the plate (both run long,
 * both read the R as an A), but the arm this switch selects is the less wrong
 * one.  See gpu.c's block on this variable for the six pairs verbatim. */
extern volatile uint32_t g_gpu_front_on;

/* WHICH OPERATOR THE PRODUCT'S STAGE 3 USES.  Reset value 1 as of 2026-10-08,
 * so a board that has had `f` pressed back on gets this repository's own bounded
 * blit_separable_convolve_new(BLIT_SEP_ROWCOL); pressing 's' moves the product
 * onto filter_frame_lib(), which since 2026-10-09 is the library's operator
 * TRANSCRIBED with bounded commits, not the library call itself.
 *
 * IT IS A STAGE OF THE GPU FRONT END, so it does nothing while
 * g_gpu_front_on is 0 - which is a state this board only reaches by pressing
 * `f`.  Reset 1 means a board that presses `f` lands on the BOUNDED operator.
 * (`pair` is unaffected either way - it runs both front ends on purpose - so
 * `p` still reads it.)
 *
 *   OFF (0) = filter_frame_lib()  = the library's operator, transcribed + bounded
 *   ON  (1) = filter_frame_with() = the rank-1 pair + bounded gpu_drain()
 *
 * IT SELECTS BETWEEN TWO OPERATORS, NOT BETWEEN FILTERING AND NOT FILTERING.
 * GPU_FILTER_ENABLE is 1 (above), so stage 3 is part of the front end this
 * product ships; a gate whose OFF state dropped the filter would hand the model
 * a picture it has never seen and would leave `pair` with no control arm to
 * compare the new operator against.  The switch is therefore also a real A/B:
 * both arms exist in one build, one boot, one scene.
 *
 * Two independent reasons were given for the move, and BOTH ARE ARGUMENTS TO TRY
 * IT, NOT MEASUREMENTS OF IT.  The first - the library's tail is the unbounded
 * nsgpu_cmd_sync ([[gpu-frontend-unbounded-wait]], [[gpu-hang-and-drain-fix]]) -
 * is RETIRED as of 2026-10-09: that wait is gone from BOTH states, so it no
 * longer favours this one.  What remains is the second: the library's filter is
 * only HALF the operator its name says - its pass 2 is a COLUMN pass that the
 * shipped wrapper hands the same n and the same kernel pointer as pass 1, so the
 * pair is real but weakly parameterised.  The replacement is a rank-1 2D pair
 * with the two passes' gains set independently and it commits through
 * blit_tile()'s bounded gpu_drain().
 *
 * (This paragraph used to say the library "hardcodes kernel height 1 in BOTH
 * passes, so what it ships is a row-then-row, i.e. horizontal smoothing with the
 * vertical half missing entirely".  THAT WAS WRONG and its own citation refutes
 * it: nsgpu_lib_dis.txt:28834-28838 is a ROW kernel and 28898-28902 is a COLUMN
 * one.  Corrected 2026-10-08, comment only.)
 *
 * So the product does not move until the boot probe has read the new operator:
 * `gpu agree filt mean/max/swap/ok` must come back `ok 1` (the same bound the
 * blit leg is held to, CAM_GPU_AGREE_MAX) and `gpu agree filt vs blit` must be in
 * the tens at most, i.e. a blur rather than an extreme.  THAT IS WHY THIS SWITCH
 * DOES NOT GATE THE PROBE: camera_gpu_agreement_probe() calls
 * gpu_filter_frame_new(), which is always the new operator, so the reading is
 * taken on every boot whether or not the product is using it.
 *
 * It gates BOTH product consumers - camera_grab_rgb888() and
 * camera_grab_pair() - and the check itself lives in gpu_filter_frame() rather
 * than at those two sites, so they cannot drift apart; the pair arm exists to
 * measure the front end the product has.
 *
 * One thing it is NOT: a latency switch - and NOT a claim that the two cost the
 * same.  That sentence used to say "the same two draws per pass; neither is the
 * cheap one", and the board refuted it on 2026-09-29: `grab` went 112..119 ms
 * (library) to 763 ms (`x`) / 1041 ms (`pair`) for the new pair, 6-10x, against
 * a known ~124 ms for two 320x320 passes.  So it is not a latency switch in the
 * sense of "pick this one to go faster".
 *
 * THOSE COST FIGURES ARE ROWCOL_T'S AND MAY NOT BE QUOTED FOR THE OPERATOR THIS
 * SWITCH NOW SELECTS.  On 2026-10-08 the reason ROWCOL_T had for existing was
 * falsified on the board (see gpu.c's block on blit_separable_convolve_new -
 * the COLUMN shape does deliver n >= 3 taps once the address mode is the one the
 * driver requires for a width-1 kernel), and stage 3 moved onto ROWCOL, the mode
 * the vendor's own blit uses.  ROWCOL was then measured on the next boot, so the
 * 6-10x above is a fact about a mode this build no longer calls: the operator's
 * own `call` line went 492710 -> 59388 us, 8.3x, almost all of it `drn` - the
 * GPU's own time - while the draw submission (`drw`) went UP, 9 -> 27, and still
 * came out 8.3x faster.  So the transposed read was the cost, not the taps.
 *
 * THAT IS THE OPERATOR, NOT THE FRONT END.  The whole grab also carries the
 * blit, the unpack and the CPU bilinear, and none of those were re-measured;
 * `pair`'s two arms in one boot are what price it, and
 * [[conv-chain-error-gate]]'s report-the-spread-first discipline is what decides
 * the default.
 *
 * TAKEN, TWICE, AND THE ANSWER IS "THE SAME COST" - so the default is now 1.
 * `Code=204066`, `p` x6 per arm, two captures: `s`=1 reads 395..428 ms and `s`=0
 * reads 387..425 ms, a difference inside the scatter, where ROWCOL_T measured
 * 1001 against 412.  The operator's own counter agrees - `gpu sep t ... call
 * 60063 ... drn 59738` is ~60 ms against the library's measured ~62 ms for its
 * two passes.  The 2.43x/6.7x belonged to a mode this build no longer calls, so
 * "keep it at 0, it is 2.43x cheaper" no longer holds.
 *
 * The other half is the picture, and it closed the same day.  The `diff max` gap
 * open since 2026-10-08 (198 for this pair against the library's 108..120) turned
 * out to be ONE fault appearing twice - a weighted texture tap taking the BORDER
 * colour 0, first on pass 2's one-wide u window (a dead column 0, `over`
 * 303..325) and then on its five-tap v window (a darkened last row, a 127-count
 * deficit).  `dhd_set_texture_border_color()` documents the fill colour as used
 * for a texel OUTSIDE the texture and its argument as ARGB8888, so a caller
 * passing 0 gets an exact 0; CLAMP is the same branch taken the other way and
 * replicates the edge texel instead.  With BOTH axes CLAMPed: `over` is 0 on
 * both arms 12/12, `max` is 99..108 against 99..111, and the argmax lands on the
 * same ordinary interior pixels on both arms.  The two operators no longer
 * separate on this instrument.
 *
 * `f` (g_gpu_front_on) IS A SEPARATE DECISION, and it went the same way on the
 * same day: reset value 1 as of 2026-10-08.  Turning the front end on WAS a cost
 * decision, and it has now been measured for the operator this switch selects -
 * `x` x6 per arm, one boot, 171753..182057 us for the CPU bilinear grab against
 * 111370..116776 ms with the front end on, i.e. 58..62 ms CHEAPER with the ~59 ms
 * filter already included, 6/6 detect with a readable plate on both arms and no
 * wedge in 12 grabs.  THAT MEASUREMENT IS WHY THIS SWITCH IS NOW 1.  (It also
 * shortened the distance to the library's unbounded commit from two keypresses
 * to one; that commit is gone from BOTH `s` states as of 2026-10-09 - the
 * library's operator is transcribed now, see the note above.)  gpu.h on
 * g_gpu_front_on carries the block. */
extern volatile uint32_t g_gpu_front_sep_on;

/* STAGE 39'S COST SPLIT, filled by filter_frame_with() and blit_tile() and read
 * once by camera_gpu_agreement_probe(); see the block comment beside their
 * definitions in gpu.c.  All microseconds at 600 MHz, per CALL and not
 * accumulated across calls (DWT->CYCCNT wraps at 7.158 s and one filter call is
 * ~0.7 s, so the single-call scope is the whole safety argument).
 *
 *   g_sep_t[0]  filter_frame_with()'s setup, before the template call
 *   g_sep_t[1]  blit_separable_convolve_new() end to end
 *   g_sep_t[2]  gpu_fb_put() after it
 *   g_sep_t[3]  1 = the template actually ran this call, 0 = it refused
 *               (written but NOT printed - g_blit_t[4] is the probe's
 *                ran-marker on the line, and two markers would be one too many)
 *   g_blit_t[0] the four dhd_set_* before each draw, summed over the tiles
 *   g_blit_t[1] dhd_draw() itself
 *   g_blit_t[2] dhd_wait(E_DHD_WAIT_WRITEBACK) - the GPU's own time
 *   g_blit_t[3] gpu_drain() - our commit point
 *   g_blit_t[4] how many tiles contributed (2 for a ROWCOL pair)
 *
 * blit_tile() is shared with the depthwise and conv paths and accumulates these
 * only while g_blit_t_on is non-zero, which filter_frame_with() is the sole
 * raiser of.  So a non-zero g_blit_t[] here is always this stage's. */
extern volatile uint32_t g_sep_t[4];
extern volatile uint32_t g_blit_t[5];
extern volatile uint32_t g_blit_t_on;

/* DIAGNOSTIC, boot only.  Substitute the UV gpu_dw_conv() uses for every
 * subsequent draw, with u0/v0/dux/dvy given in TEXELS (they are converted to
 * the driver's normalised form inside).  gpu_dw_uv_clear() puts the derived UV
 * back.
 *
 * Why this exists: the stride-2 UV is a composition of two measured facts and
 * one that neither measurement could see - which texel the kernel window
 * centres on when u0 is not 0.  `gpu dw step 1:1 2 2:1 2` reads C(t+1)-C(t-1),
 * which is 2 for every centre, and at stride 1 u0 = 0 hides the question.  The
 * first stride-2 layer came back `chk s2 in n 1296 worst 26 bad 94`, so the
 * composition is wrong and the convention is swept rather than argued.
 *
 * These two are not part of the operator's contract and the shipping path never
 * calls them. */
void gpu_dw_uv_set(float u0_texels, float v0_texels,
                   float dux_texels, float dvy_texels);
void gpu_dw_uv_clear(void);

/* The largest deviation the five-bit texel can produce for the LAST layer
 * gpu_dw_conv() drew, in output quantisation units, over its channels:
 *
 *     max_c ( m_c * 3.5 * SUM_i |w_i,c| )      m_c = in_scale*w_scale_c/out_scale
 *
 * It is a property of the layer's weights and scale and not of the UV, so it is
 * the number that says whether a layer can meet a given tolerance at all, and
 * it is what `split` in gpu_dw_conv() exists to answer: a bound above the
 * caller's threshold that shows up as a measured deviation of the same order is
 * a residual, and the second pass removes it, whereas a bound far below the
 * measured deviation means the deviation is something else and no amount of
 * precision will help.
 *
 * MEASURED, 2026-09-17, which is what settled the question: stride 1 reports 9
 * against a worst of 4, stride 2 reports 86 against a worst of 25.  The
 * threshold the caller uses (DW_GAP_MAX = 8) sits just under the stride-1
 * bound - it was calibrated against layers whose weights are an order of
 * magnitude smaller than the stride-2 ones - so it is not a criterion the
 * stride-2 class can meet without the second pass.
 *
 * Reset at the top of each gpu_dw_conv() call.
 *
 * Read by the caller only to explain a failure in the log; the shipping
 * decision is made by re-running the check with `split`, not from this number. */
float gpu_dw_bound(void);

/* ===================================================================
 * The pointwise convolution (CONV_2D), on the GPU.
 *
 * WHAT IT IS.  A 1x1 convolution is a per-pixel dot product over the input
 * channels, out[co][p] = SUM_ci w[co][ci]*a[ci][p], so this operator lays a
 * pixel's Cin activations out along the source texture's x axis - one 5-bit
 * code per texel, the same conversion the depthwise operator uses - and lets a
 * 1x16 CLUT kernel sum sixteen of them per texture unit per draw.  Two units
 * sum thirty-two channels in one draw, which is the measurement
 * gpu_conv_probe()'s `2u copy 16 add 130` was taken for.
 *
 * WHY THERE IS A FOLD.  The channel sum cannot accumulate across draws: every
 * pixel format this chip has is at most 8 bits per channel, and the one
 * accumulating blend setting (f_dst = E_DHD_ONE) appears nowhere in the
 * library, so this design does not bet on it.  Instead each chunk of 32 input
 * channels writes ITS OWN destination plane and the CPU folds the planes - the
 * depthwise operator's shipped hi/lo split (`byte1 + byte2 - 128`), written for
 * K chunks instead of 2 halves.
 *
 * THE ARITHMETIC, and where every number in it comes from.  `in` is packed as
 * code = a + 128, which cannot clip whatever the input zero point is, and code
 * splits exactly into a high digit T5 = code >> 3 and a low digit r = code & 7
 * with 8*T5 + r == code.  The kernel's multiplicand is that 5-bit field widened
 * to eight bits by replication, X = 8*T5 + (T5 >> 2), so a hi-chunk draw really
 * computes (1+g) times SUM(w*T5), where
 *
 *     g = SUM(w*(T5>>2)) / (8*SUM(w*T5))
 *
 * is a CONSTANT per (co, chunk).  The prepare step measures it on the layer's
 * own activations and divides it out in the scale field:
 *
 *     hi digit   a_scale = 256*M/(1+g)^2   target 8*M*SUM(w*T5)/(1+g)
 *     lo digit   a_scale = 32*M            target M*SUM(w*rep5(r))
 *
 * with M = in_scale*w_scale[co]/out_scale.  The lo plane needs no gain because
 * the replication is applied to r before the sum, which is exactly what the
 * hardware does.  Both land inside the scale field's [2.4e-07, 4.4e+12] for
 * every layer this model has; the emulator's table reports the uncorrected
 * 256*M as 3.9e-01 .. 3.3e+01, and the correction only divides it down.
 *
 * Each digit is CENTRED, so a chunk whose partial sums sit away from zero does
 * not spend half the byte range on the offset.  The centring is
 * cen_hi = 8*M*t5bar*wsk/(1+g) and cen_lo = M*rbar*wsk, built from the plane
 * means t5bar/rbar and the chunk's weight sum wsk.  What the fold SUBTRACTS is
 * not this number but the bias that was actually programmed, and the difference
 * matters once the field is finite.  It cannot bias a layer either way - the
 * subtraction is what cancels it - but it decides two things: whether the digit
 * stays inside [0, 255], and whether the field can hold it at all.
 *
 * THE 9-BIT BIAS FIELD IS A REAL GATE, and this is not the place the design
 * expected to find one.  The digit is `trunc(V) + bias` and the write clamps to
 * [0,255], so the centring is the only thing keeping a digit whose V sits at
 * -900 from clipping to a constant.  A field holds [-256, 255], so the centring
 * travels SPLIT between the draw's two texture units - each carries
 * b = trunc((128 - cen)/2) and the colour ADD applies it twice, reaching
 * 2*b = the ideal centring to within one.  `trunc` and not `round`, because the
 * offline table computes trunc and this must produce its integer.
 *
 * That reaches |128 - cen| <= 511, and 16 of the model's 52 layers need the
 * split while one (op182, |128-cen| = 1026) needs more than two fields can
 * carry.  It is refused, and the table that used to call it clean now calls it
 * a wall: Step 0's model did not include this field when it was first written,
 * and adding it moved op182 out of the clean set and op100 into it.  Net 42
 * either way - which is a coincidence and not a reason to trust the older list.
 *
 * The split is the SYMMETRIC configuration, both units holding the same field,
 * because that is the one this project has measured (`2u copy 16 add 130`,
 * `2u scale 73`, `2u clut 23`).  A single unit holding the whole centring would
 * reach twice as far and be exact, and it is not done: whether a texture unit's
 * output is clamped to a byte BEFORE the colour ADD is invisible offline, and
 * that one fact decides whether the asymmetric form works.  If it is ever
 * measured, the asymmetric form becomes strictly better - twice the reach and
 * no quantisation - and this paragraph is the reason to go and measure it.
 *
 * WHAT IS NOT MODELLED.  The per-digit truncation mean (`corr` in
 * tools/conv_gpu_emulate.py) is NOT measured here, because measuring it needs
 * the per-pixel partial sums, i.e. a full CPU convolution per layer.  The
 * justification for dropping it is structural rather than statistical: every
 * digit is centred, so its value straddles zero and the mean of its fractional
 * part is near zero by symmetry.  That is the strongest statement available
 * without paying for a CPU convolution, and it is why the per-layer check in
 * yolo_inference.cpp - a byte-for-byte comparison against the stock kernel, run
 * once per layer and permanent when it fails - is the arbiter, not this comment.
 *
 * AND ONE MORE THING THE OFFLINE TABLE CANNOT SEE.  Each unit truncates its own
 * partial sum, so where the emulator's model has one truncation per digit the
 * hardware has two - worth up to one LSB per unit, always downward, and it is
 * folded into the same per-layer tolerance the `corr` note above relies on.
 *
 * COST, and it is real.  The CPU writes two texels per activation (np*cin*2
 * 16-bit stores into uncached Device memory), and the GPU writes 2*K planes of
 * np bytes, i.e. 2*ceil(cin/32) times the output tensor, per layer.  Across the
 * layers this actually takes that is 0.69 M activations packed and 0.62 M
 * outputs folded by the CPU, per inference.
 *
 * THIS FUNCTION'S CHUNK IS FIXED AT 32 - two units, sixteen taps each - so what
 * it takes is not the emulator's 42-layer structural set but the intersection of
 * that set's widest rung with the structural limits below.  Measured offline for
 * exactly this model and this detection frame (tools/conv_gpu_emulate.py, whose
 * SHIPPED section applies the limits and not only the arithmetic):
 *
 *     36 layers, 11100 draws, 2.70 Mpx written -> 0.51-0.81 s at 0.19-0.30 us
 *     per written pixel, on top of a 1.70 s all-CPU inference.
 *     K (chunks per layer) is {1: 4, 2: 11, 3: 20, 6: 1}.
 *
 * That is a PREDICTION, not a measurement, and the two ways it can be wrong are
 * both in the safe direction: the per-layer byte check in yolo_inference.cpp
 * uses a 12-LSB tolerance where the emulator's own criterion is 8 or the layer's
 * truncation floor, whichever is larger - so it can pass a layer the emulator
 * rejects, and a layer it rejects stays on the CPU rather than being drawn
 * wrong.  Fewer layers than 36 is the likely outcome, never more.
 *
 * What is NOT taken, and why, so this is not read as unfinished work:
 *   * four layers (op46, op100, op136, op143) are clean only after NARROWING the
 *     chunk.  This version has one chunk size, so it refuses them; taking them
 *     needs the escalation and is Step 3's first item.
 *   * op1, the model's only 3x3, is refused by the filter-size test - a 3x3
 *     cannot be expressed as a row of taps that also walks the channels.  It is
 *     also the cheapest layer in the model (Cin 3, 96 draws at chunk 2), so this
 *     costs almost nothing and is Step 3's second item.
 *   * op186 (288 channels at 20x20) is arithmetically CLEAN and cannot be drawn:
 *     its packed source is about 115240 texels against this slot's capacity of
 *     49152.  That one is a capacity limit and not a candidate for the
 *     escalation - see GPU_CV_MAXSRC in gpu.c.
 * The seven layers that are walls offline (op7, op20, op51, op88, op112, op124,
 * op182) are refused for the same reason the emulator refuses them: their digits
 * saturate, and a saturated digit's error cannot be recovered by any fold.
 *
 * So this makes the product SLOWER while genuinely putting the operator on the
 * accelerator.  Per gpu-offload-goal that is not the gate, and it is reported
 * rather than hidden: gpu_conv_stat() counts it.
 *
 * @param  in        input activations, NHWC int8, h*w*cin
 * @param  wq        the model's weights as TFLM stores them, [cout,1,1,cin], so
 *                   w[co][ci] is at (co*cin)+ci
 * @param  biasq     the model's int32 bias, cout entries
 * @param  in_scale  the input tensor's quantisation scales; only [0] is read,
 *                   because this model's conv input is quantised per tensor
 * @param  w_scale   weight quantisation scales, cout entries
 * @param  out_scale output quantisation scale
 * @param  in_zp     the input tensor's int8 zero point.  Needed by the fold and
 *                   not by the pack: the pack is `a + 128` whatever in_zp is,
 *                   and the fold has to give back the `in_zp` that TFLM's own
 *                   accumulator subtracts.  Unlike gpu_dw_conv() this operator
 *                   does NOT require in_zp == -128, and no layer is refused for
 *                   it.
 * @param  out_zp    the output tensor's int8 zero point
 * @param  h, w      the input plane's height and width.  The output is the same
 *                   size: the only layers this accepts are 1x1 SAME, which
 *                   preserve it.
 * @param  cin       input channels, <= GPU_CV_MAXCIN (320)
 * @param  cout      output channels
 * @param  out       output activations, NHWC int8, h*w*cout
 *
 * @retval 0   the layer ran on the GPU and `out` is wholly written
 * @retval -1  refused; the caller must use the CPU path.  `out` MAY HOLD
 *             PARTIAL RESULTS in this case - the fold runs per output channel
 *             and a drain can fail between two of them - which is safe only
 *             because the caller's fallback re-runs the stock kernel and that
 *             rewrites every byte.  Do not read a refusal as "out is
 *             untouched" the way gpu_dw_conv()'s contract allows.
 * @retval -2  refused, and it is the offline table's `bwall`: the nine-bit bias
 *             field cannot hold this layer's centring.  A DIFFERENT CLAIM from
 *             -1 and not a different outcome - the caller still falls back -
 *             which is why it is worth separating: a capacity refusal is about
 *             this DRAW (too many chunks for the scratch plane, a scale outside
 *             its field, a drain that timed out) and a wall is about the LAYER,
 *             "not a numerical difference" that "no choice of chunk or a_scale
 *             recovers" in the offline table's own words.  A wall is found in
 *             pass 2, BEFORE anything is drawn, so unlike -1 `out` is untouched
 *             - which the caller must not come to rely on either.
 *
 * Not thread safe, not reentrant, and it changes the driver's global
 * framebuffer format to L_8 for the duration - it saves and restores format,
 * size, bbox and address, but a second caller would interleave badly.  The
 * trigger path is single threaded.
 */
int gpu_conv(const int8_t* in, const int8_t* wq, const int32_t* biasq,
             const float* in_scale, const float* w_scale, float out_scale,
             int32_t in_zp, int32_t out_zp, int h, int w, int cin, int cout,
             int8_t* out);

/* What gpu_conv() has actually done since boot - the evidence for "the operator
 * runs on the GPU", and the reason the CPU-side cost is reported rather than
 * hidden.
 *
 * `packed` is activations turned into texels (np*cin) and `unfolded` is output
 * activations the CPU fold produced (np*cout); both are counts, not bytes, so
 * multiply `packed` by 4 for the bytes written through the uncached scratch.
 * Any argument may be NULL.
 */
void gpu_conv_stat(uint32_t* layers, uint32_t* draws, uint32_t* refused,
                   uint32_t* packed, uint32_t* unfolded);

/* What the LAST gpu_conv() call's fold saw on the digit planes - the clamp the
 * offline table calls `rail`, and `at0` beside it.
 *
 * `rail` is the ADD's own result not fitting the byte it is written to: the
 * colour pass sums two texture-unit outputs that have EACH already been clamped,
 * and what the byte write cuts at the top is not in the byte the fold reads back.
 * `sat` - a single unit leaving the range - is a different clamp and it is NOT
 * this: only the SUM of the two halves reaches the plane, so no board quantity
 * can count a unit.  tools/conv_gpu_emulate.py says in its own words that the
 * board's `pl` dump counts exactly this one as `at255`.
 *
 * Counted over the whole layer and every output channel, on the bytes the fold
 * already reads - NOT a sample - and reset on entry to each gpu_conv() call, so
 * a caller that reads it straight after one draw is reading that draw.  This is
 * why it is separate from gpu_conv_stat(), whose numbers are since-boot.
 *
 * The two board counts are `>= 255` and `== 0`; the offline `rail` is strictly
 * `> 255`.  So `at255 > 0` is the SUPERSET - a digit landing exactly on 255
 * without clamping reads here - and a caller gating on it is slightly stricter
 * than the offline table.  Any argument may be NULL.
 */
void gpu_cv_fold_stat(uint32_t* rail, uint32_t* at0);

/**
 * @brief  The smallest |b| over every texture unit of every chunk the LAST
 *         gpu_conv() call drew.
 *
 * REPORTED ONLY.  It was built to gate a layer's verdict - the theory being that
 * a texture unit annihilated on a pixel costs the plane that unit's own bias
 * `b_u`, so `|b_u| <= CV_GAP_MAX` would make the error invisible to the byte
 * test.  That theory does not hold: `cv_digit()` centres every unit's post-bias
 * mean on 64, so a lost unit costs about that mean whatever `b_u` is.  The gate
 * was tried on the board on 2026-10-09 and reverted the same day - it cost 10 of
 * the 13 layers that passed `bad 0 rail 0`, with no measured benefit.
 *
 * It stays as a `bu` column on the `gpu cv chk` line because the mechanism
 * question is still open and this is the reading it needs.  -1 means the call
 * refused before its first chunk (nothing seen); `bu_min` may be NULL.
 */
void gpu_cv_bias_min(int32_t* bu_min);

/**
 * @brief  Read out ONE output channel's digit planes, on the next draw of a
 *         pointwise layer, and print four numbers per plane.
 *
 * The offline model of this operator (tools/conv_gpu_emulate.py, its
 * `fix_draw` column - the plane layout gpu.c actually implements) says the
 * layer op6 comes out at worst 4, bad 0.  The board reads worst 122, bad 4218
 * on it, and the model's own clipping evidence does not correlate with which
 * layers fail.  So the model cannot name the fault, and these four numbers can:
 *
 *   mn        the plane's smallest byte
 *   mx        the plane's largest
 *   at0       how many bytes are exactly 0
 *   at255     how many are exactly 255
 *
 * `at0`/`at255` non-zero on a plane the model says never clamps is a clamp that
 * should not exist; a `mn`/`mx` that misses the model's range is a shift.  The
 * sweep covers exactly the `np` bytes per plane the fold reads.
 *
 * Off by default, and it is expensive - one Device-mapped byte read per byte of
 * every plane.  The caller turns it on for a bounded number of failing layers
 * and re-runs that layer's draw, so it never costs the healthy path anything.
 *
 * THE CALLER MAY INSTEAD LEAVE IT ON FOR CHANNEL 0 OF EVERY LAYER, and that is
 * the form worth having: tools/conv_gpu_emulate.py prints, for every layer's
 * channel 0, exactly these four numbers per digit plane plus that layer
 * channel's `net` and `bsum` (its census table).  With `co == 0` on both sides
 * the board's output pairs with the offline table line for line and number for
 * number, so a disagreement is a disagreement and not a pairing argument.  Two
 * differences from the bounded form: nothing needs re-drawing (the production
 * draw already wrote channel 0's planes), and the layer index has to travel with
 * the flag or the lines cannot be paired at all - hence @p layer.
 *
 * @param  on    0 to switch it off, non-zero to census the draws that follow
 * @param  co    the output channel to census
 * @param  layer the caller's layer index, printed on every census line as `n`
 */
void gpu_cv_census(int on, int co, int layer);

/**
 * @brief  Arm the three-way dump for ONE layer, on its next draw.
 *
 * The board's own activations, which is the one input every reading of this
 * operator so far has lacked: `tools/conv_gpu_emulate.py` computes both sides'
 * arithmetic from a jpg, so it can only say what the two MODELS disagree about.
 * On the detection head the two readings are of different kinds - the oracle
 * pins all 14 head slots at 4-6/0 across 40 real images while the board reads
 * n39/n43/n47 FALLBACK on every boot - so the argument has to be closed on the
 * board's bytes.
 *
 * Armed here by layer index (there is no layer number in gpu_conv()'s
 * signature, so this rides the same variable gpu_cv_census() sets, which the
 * glue sets immediately before the draw and clears immediately after).  Prints,
 * in 32-byte hex lines: the layer's input tensor, every digit plane of output
 * channel 0 as the fold reads it, and the GPU's output.  The CPU reference is
 * dumped by the glue, which is where that buffer lives.
 *
 * It DISARMS ITSELF on first use, at the first opportunity inside gpu_conv(),
 * so a refused layer still yields its input and no layer ever dumps twice -
 * ~8 KB of serial that must not become a per-frame cost.
 *
 * @param  n  the caller's layer index, the same number its `chk` line prints
 */
void gpu_cv_dump(int n);

/**
 * @brief  Print `count` bytes from `p` as `D <tag> +<offset> <hex>` lines.
 *
 * Exported because the CPU reference the dump is compared against lives in the
 * glue (`yolo_inference.cpp`), and one implementation of the format is the
 * difference between a diffable dump and two similar-looking ones.  Chunked at
 * 32 bytes a line through a buffer of its own.  That was forced when `Message[]`
 * held 64 bytes and one line plus its tag did not fit; the buffer is 160 now but
 * the chunk size stays, because this format is parsed by tools/conv_dump_diff.py
 * and a proven dump is not worth reformatting to save a buffer.
 *
 * Print only.  Safe with `p == NULL` or `count <= 0`; does nothing then.
 */
void gpu_cv_hex(const char* tag, const int8_t* p, int count);

/* THE POINTWISE OPERATOR'S ON/OFF SWITCH, the third of the same kind as
 * g_gpu_dw_on and g_gpu_front_on, and a runtime variable for the same reason:
 * flipping it must not cost a flash round, and the two arms have to run on the
 * SAME still scene or the comparison measures the scene.
 *
 * 0 - the reset value - means the qualifying CONV_2Ds run on the CPU.  This was
 * flipped to 1 for one round on 2026-09-22, during which the operator landed 18
 * of its 52 layers on the hardware including both detection-head layers, every
 * one of them measured against the stock CPU kernel on the board's own frame
 * before use and every failure refused for the boot.  It went back to 0 when
 * that proof turned out to be about the wrong quantity: on one still scene, the
 * arm that also had these 18 layers on the GPU read 1 box where the arm with
 * only the depthwise passes read 6 and the all-CPU arm read 11 and a well-formed
 * plate.  g_gpu_dw_on above carries the three-arm table.  Expect it to be SLOWER
 * than the CPU (see the COST note on gpu_conv() below); the goal this was built
 * to is that the operator runs on the GPU and not that the product gets faster,
 * but a slower product that cannot read its plate is not that goal either.
 *
 * It is checked FIRST in conv_eligible() in yolo_inference.cpp, so "off" means
 * the layer never touches the per-layer state, never runs the one-time check,
 * and is not counted as eligible.  gpu.c owns the storage; the serial command
 * in main.c flips it and prints the new value. */
extern volatile uint32_t g_gpu_conv_on;

/* THE TAIL LIMIT - a DEPTH cut under each operator's on/off switch, one per
 * operator, and the answer to the one question the per-layer work cannot answer.
 *
 * WHAT IS WRONG THAT THIS ADDRESSES.  Every layer of both operators is verified
 * on its own before it is used, and the product still degrades.  On one still
 * scene: 11 boxes with nothing offloaded, 6 with the 38 depthwise passes alone,
 * 1 with both operators.  `lim12` bounds the error of ONE layer and the product
 * reads 38 passes and 18 layers in SERIES, each feeding the next - so a per-layer
 * gate is measuring the wrong quantity, and it has been measured to be measuring
 * the wrong quantity.  See tools/conv_gpu_emulate.py's per-layer table for the
 * other half of the same lesson: 23 of 52 conv layers fail, and every one of them
 * fails on `sat != 0`, not on the numeric gate - the failures are not marginal,
 * they are structural.
 *
 * WHAT IS NOT KNOWN, and cannot be learned off the board: HOW MUCH of that chain
 * the product actually carries.  Nothing offline models error PROPAGATION, so the
 * only instrument for this is the board itself, with a knob that changes the
 * chain and nothing else.
 *
 * WHY A TAIL AND NOT A SUBSET.  A layer near the output has no layer after it to
 * amplify its error, and the errors that matter here are amplifications.  Keeping
 * only the tail therefore keeps exactly the part of the chain that CANNOT
 * compound, and the threshold becomes a single number swept upward until the
 * plate comes back - which is a measurement of how much GPU work the product can
 * carry, in the units the stage is counted in (draws on the hardware).
 *
 * THE NUMBER IS THE OPERATOR'S OWN ORDINAL, measured from the input end, 0-based,
 * increasing with depth.  Each operator numbers its layers in model order, which
 * is why the threshold is monotone in depth and why `n >= tail` is a contiguous
 * tail of the network.  The two operators have DIFFERENT ordinal spaces (52
 * conv layers, 27 depthwise) so the two numbers are NOT comparable - do not read
 * a smaller conv threshold as less offloading, and do not diff them.
 *
 * 0 - the reset value - means NO LIMIT, i.e. exactly the behaviour this board had
 * before this variable existed: every eligible layer is offloaded.  So an
 * untouched board is unchanged, and the sweep's last state before wrapping back
 * to 0 is the interesting one rather than the default.
 *
 * It is tested AFTER each operator's own eligibility test, so a layer this
 * refuses is refused for no other reason, the per-class one-time check still runs
 * on the first layer that is actually offloaded, and a refused layer falls back
 * to the stock CPU kernel like every other refusal. */
extern volatile uint32_t g_gpu_dw_tail;
extern volatile uint32_t g_gpu_conv_tail;

/* THE `n` KEY - KEEP THE DRIVER FROM WRITING A START WHILE THE STC IS ALIVE.
 * Default 1 = on since 2026-09-28, so an untouched board carries the repair and
 * pressing `n` turns it OFF - the way to reproduce the wedge rather than the way
 * to avoid it.
 *
 * WHAT IT DOES.  gpu_drain() reads the driver's `s_devices[0].m_status` next to
 * the busy vector; when this is 1, bit 0 of `Busy_Status` is set (the stream is
 * alive and parked) and the driver says anything but ACTIVE, it writes ACTIVE
 * back.  It touches NOTHING in hardware - one word of the library's own device
 * struct - and that is the whole intervention.
 *
 * WHY THAT IS A CANDIDATE FIX, and it is a mechanism rather than a correlation.
 * davehd_kernel_ring.c:199-208 writes STREAM_ADDRESS - the start that the STC
 * refuses with `E_STOP_IRQ_CODE_START_WHEN_BUSY` - only when `m_status` is not
 * ACTIVE, and `Code=201638` counted the whole boot: of 1414 gated commits exactly
 * ONE wrote a start, and that one is the commit that wedged the STC.  Its busy
 * vector was `00000001`.  So a start is rare, and the one that happened landed on
 * a stream the hardware calls alive.  Those two facts cannot both be right unless
 * the driver's word is stale at that moment - which is what this repairs.
 *
 * WHAT SETTLED IT, and it was a two-arm measurement rather than the argument: the
 * `ms` histogram, `last`, `kp` and `ls` on the per-frame summary line, and `ms`
 * / `ls` on the `drain dead` line.  Arm A (off): 4 of 4 boots wedged, all on the
 * first frame, the boot that printed `ms` reading `ms0` with 734 of its 736
 * commits ACTIVE.  Arm B (on): 18 frames, 37594 drains, 29 starts, no wedge, and
 * the drain behind every one of the keep's 3 firings succeeded.
 *
 * WHAT IS STILL OPEN, stated so it is not read as closed: the predicate is bit 0
 * of the busy vector, and arm B contains starts that carried that same bit and
 * did NOT wedge (`strt2 nz1 sor001`), so "start + bit0" is not by itself the whole
 * condition - it is correlated with the wedge, measured across two arms, and not
 * proved sufficient.  `ls` is on the lines now to close that: a keep that fires
 * with `ls0` is a driver resuming a finished stream, anything else names a stop
 * the hardware raised for another reason.
 *
 * THE FAILURE MODE IS NOT A HANG, which is what makes it testable at all: if a
 * live STC will not consume from a pause address, the drain times out and takes
 * the same latch as before (see gpu_drain()).  The hard rule holds either way.
 *
 * THAT LAST SENTENCE IS TRUE ONLY OF OUR OWN COMMITS, and 2026-09-28 measured the
 * exception: the library's commits (nsgpu_colorkey -> nsgpu_cmd_sync ->
 * dhd_finish) have no latch behind them, and the shipped configuration wedged at
 * stage 38 - the front end's scaled blit - after eight clean frames, with eight
 * `STALL s38 c0 p0` and a `GPU WEDGE 8s -> reset`.  The repair has to be applied
 * to those commits as well, on the way in, which is what gpu_keep_active_now()
 * is for.  The failure mode there is NOT a drain timeout; it is the whole
 * product, because dhd_finish never returns. */
extern volatile uint32_t g_gpu_keep_active;

/* Apply the keep repair to a commit we do NOT own, then enter the library.
 * Reads GPU_STC_BUSY and the driver's m_status and forces ACTIVE when both halves
 * of the wedge are present - the identical predicate and the identical action
 * gpu_drain() has applied to our own commits since 2026-09-28, sharing `s_keeps`
 * so a repair can never be invisible on the `kp` field.  Call it IMMEDIATELY
 * before nsgpu_colorkey() / nsgpu_separable_convolve() / nsgpu_set_framebuffer():
 * after the call there is nothing left to run, because the wait that matters is
 * inside it.  No-op when g_gpu_keep_active is 0 or the GPU is not ready, so it is
 * safe to call unconditionally.  It prints at most twice per site per boot, tagged
 * `gpu front`, and touches none of the drain's own counters. */
void gpu_keep_active_now(void);

/* Dump the STC block, for a caller outside this file that is still running when
 * nothing else is.  The one such caller is the stall reporter in the DVP
 * interrupt: it is what prints `STALL s38 ...` and what resets the product eight
 * seconds later, and the drain give-up that normally prints this dump is never
 * reached when the wait is inside the library.  Read-only, no state written - see
 * the note above gpu_stc_dump() in gpu.c for what each word answers. */
void gpu_stc_dump_now(const char* why);

/* ===========================================================================
   THE EXACT-OPERATOR ROUTE: TRANSPOSE ON THE GPU

   133 of the model's 224 operators do no arithmetic at all (tools/_exact_ops.py;
   the board reconciles the count with key `e`).  A permutation has no
   requantisation to lose, so unlike every conv layer in this project an offload
   here cannot be a small numeric error - it is either byte-exact or it is the
   wrong answer, and that is provable on the board rather than argued.

   THE OPERATOR.  `gpu_transpose_s8()` writes the transpose of a `tw` x `th`
   8-bit plane, exactly: `out[c*th + r] == in[r*tw + c]` for every byte.  It is
   NOT a kernel pass, and that is the whole point - see the long note above it in
   gpu.c.  Every other draw in this file multiplies by CLUT kernel weights and a
   kernel's multiplicand is five bits, which is three bits short of an int8; this
   one uses the CLUT as an ordinary 256-entry identity PALETTE instead, so
   nothing truncates anything.

   THE SWITCH is NOT here.  The other offloads keep their gate in this file as a
   `g_gpu_*_on` global because the consumer lives in gpu.c; this one's consumer
   lives in yolo_inference.cpp, so the gate is yolo_tr_set()/yolo_tr_get() in
   that file and the `j` key in main.c is its only writer.  A second flag here
   would be state nothing reads - the shape that gets silently optimised away and
   then believed.

   THE PROBE.  `gpu_transpose_probe()` is boot-only and additive: it measures the
   recipe on synthetic data at the board's own shapes before anything is allowed
   to depend on it, and prints `gpu tr ...`.  It does not touch the product path. */

/* True if the operator ran on the GPU.  False is a REFUSAL, not a failure: the
 * caller must fall back to the CPU kernel, never retry.  Every refusal is either
 * an argument out of range or the CLUT pool being held by the depthwise/pointwise
 * offload - the 512-entry pool is shared, and the first refusal prints one line
 * saying so. */
bool gpu_transpose_s8(const int8_t* in, int8_t* out, uint32_t tw, uint32_t th);

/* Boot-only self-test, the shape gpu_dw_probe() has.  *out_bad is the total
 * number of bytes that disagreed across both the 16x16 full-range case and the
 * model's own 400x48 case; 0 is the whole verdict.  Prints under `gpu tr`. */
bool gpu_transpose_probe(uint32_t* out_bad);

/* 0 - the reset value - means the stock CPU kernel and nothing else; that state
 * is yolo_inference.cpp's, not this file's (see THE SWITCH above).  See the COST
 * note on gpu_transpose_s8() in gpu.c for why this is expected to be SLOWER than
 * the kernel it replaces: the goal it was built to is that the operator runs on
 * the hardware, not that the product gets faster. */

#endif // GPU_H
