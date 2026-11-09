/**
 * @file gpu.c
 * @brief D/AVE HD 2D engine bring-up and a round-trip self test.
 *
 * Follows the vendor example projects\n32h7xx_EVAL\examples\GPU\Convolve
 * (src/main.c, its RCC_Configuration() and NVIC_Configuration()) for the
 * power/clock/interrupt order, but not for the linking: that example compiles
 * all 48 GPU sources AND links nsgpu.lib, which is redundant - the library is
 * self contained and already holds nsgpu.o and every davehd_*.o.  So this file
 * needs no GPU source in the project at all, only the library and the five
 * include paths, which keeps the whole integration out of Printf.uvprojx's
 * <File> list.
 *
 * What is added here that the example does not have: the example never turns
 * the D-cache on, so it can ignore coherency.  This firmware does turn it on,
 * and the vendor's arch_dcache_flush_all() is an empty stub with no cache
 * maintenance to call, so the GPU's memory is placed in an MPU region that is
 * not cacheable instead.  See gpu.h for the slot layout.
 */

#include "gpu.h"
#include "Message.h"
#include "n32h7xx_eval.h"
/* This project has no n32h7xx_conf.h, so the standard peripheral headers are
   not pulled in by the device header and each file names the ones it uses. */
#include "n32h7xx_pwr.h"
#include "n32h7xx_rcc.h"
#include "misc.h"
#include "nsgpu.h"
/* THE DRIVER'S OWN DEVICE STRUCT.  This is the kernel's PUBLIC header (the one
   davehd_kernel_irq.c includes), it pulls in nothing but davehd_types.h, and it
   closes the `#pragma pack(push,1)` it opens - so including it here gives the
   offset of `dhd_gpu_device_data_t::m_status` by the compiler's own arithmetic
   rather than by hand, which is the whole reason it is included at all: the one
   thing this firmware must not do is poke a byte offset it guessed at.  See the
   extern block beside dhd_gpu_stream_status for what is read through it. */
#include "..\\middlewares\\gpu\\tes\\kernel\\inc\\davehd_kernel_driver.h"
/* The utility layer, and it is reached by NAME rather than by path because the
   build's include path carries `...\\tes\\user\\inc` (Printf.uvprojx:938).  It
   brings davehd_driver.h with it, which nsgpu.h already brought, so this adds
   only the four the front end's own fill needs: dhd_bound_aab(), dhd_clip_bbox(),
   dhd_set_util_cliprect() and dhd_util_blend().  They are DHD_API_INLINE, i.e.
   the vendor's own source inlined into this translation unit - which is what
   makes the fill below the library's fill and not a lookalike. */
#include "davehd_util.h"
#include <stdio.h>
#include <string.h>
#include <math.h>       /* sqrtf, for the per-unit bias window in cv_digit()   */

/* How much of the slot to zero before handing it to the driver: video memory
   and the acc area, i.e. everything below the two self test buffers at the top
   of the slot.  The vendor example memsets 16 MB of SDRAM before nsgpu_init()
   for the same reason - the driver carves its command ring out of this memory
   and a ring full of whatever was left in SDRAM from power-on is not something
   to feed the GPU.  The two test buffers are excluded because the self test
   writes them before it reads them. */
#define GPU_ZERO_BYTES  0x00300000u     /* 3 MB: 0xC0C00000..0xC0EFFFFF */

/* What nsgpu_init() returns when it worked.  Not 0 - that is what it returns
   when dhd_alloc_buffer() fails, and the three ways it can fail are -1, -2 and
   -3.  Read out of the library by disassembly (nsgpu_init + 436 is a bare
   `MOVS r0,#1` on the only path that reaches the end).  The vendor example
   assigns this to `int result` and never looks at it, so the header does not
   say, and guessing 0 is what this file did until the disassembly said
   otherwise. */
#define NSGPU_INIT_OK   1

/* The driver's device handle.  It is a global in nsgpu.lib's nsgpu.o and is
   not declared in any of the library's headers, so it is named here directly.
   The linker resolves it by name, which means this either works or fails the
   build with "undefined symbol a_handle" - it cannot silently read the wrong
   word.  Printf.map puts it at 0x30000008, in RW_AHB_SRAM.

   Why this file writes it before calling nsgpu_init(), rather than only
   reading it: the library's error handler, reportError() at 0x260a0, is

       if (flag == 0) return;
       printf("ERROR: %s\n", message);
       for (;;) { }

   and all three of nsgpu_init()'s early failures call it with
   flag = (a_handle == 0) - which is 1 precisely when the driver has NOT been
   initialised, i.e. on every one of those paths.  So the library is written
   such that any early failure prints and then locks the CPU up forever.
   Setting a_handle non-zero before the call makes that flag 0, so those paths
   return their error code instead, and gpu_init() can report it and let the
   rest of the firmware carry on.  GPU failure must not take the board down.

   dhd_init() overwrites this with the real handle on the success path, so on
   a successful init the value written here is transient and harmless.  It only
   survives on a failure - and a handle of 1 is not a real device, which is why
   gpu_selftest() must not run when gpu_init() failed.  See s_ready below. */
extern uint32_t a_handle;

/* Set by gpu_init() from whichever format the driver accepted; the self test
   needs it to know how wide one pixel is when it reads the result back. */
static uint32_t s_pixel_bytes = 2u;
static bool     s_ready       = false;

/* The fill colour, kept as two numbers because nsgpu_data_fill() takes an
   ARGB8888 colour and CONVERTS IT to the framebuffer's format on the way in -
   so the value handed to the API and the value that should be found in memory
   are not the same number.  The vendor's DataFill example is explicit about
   this: it passes 0xFFFF0000 as "0xFFFF0000 of ARGB8888 value" and documents
   the result as "actually memory filled with 0xF800 because of pixel_bytes=2
   means RGB565 format".

   The self test used to pass its expected value in as the colour argument and
   then compare memory against that same value - i.e. it expected 0xF800 to
   survive the conversion, when ARGB 0x0000F800 is nearly-black green and lands
   as RGB565 0x07C0.  Every pixel mismatched, which is the FAIL 8192 this
   replaces.

   White, not red, because it is the one colour where both conversions are
   beyond argument: ARGB8888 0xFFFFFFFF is RGB565 0xFFFF, and it is the maximum
   of any sane luminance formula, so it holds for the L_8 fallback too without
   needing to know what that formula is.  It is also what the vendor's other
   examples pass (Convolve, DrawGraphic, RLD all fill with 0xFFFFFFFF). */
static uint32_t s_fill_argb   = 0xFFFFFFFFu;    /* handed to nsgpu_data_fill()  */
static uint32_t s_fill_expect = 0xFFFFu;        /* ...expected in frame memory  */

/* Why the self test has no nsgpu_convolve() leg, only nsgpu_separable_convolve().

   BOTH entry points are defective in the same way, mirrored: nsgpu_convolve()
   keeps its kernel byte buffer in r6 and nsgpu_separable_convolve() in r7, and
   neither function assigns the register it uses.  Read off nsgpu.lib by
   disassembly, over the whole 448-byte nsgpu_convolve(): r6 occurs exactly five
   times - four `STRB r1,[r6,r0]` in the coefficient copy loop, and one
   `STRD r6,r8,[sp,#0x18]` that hands the same never-set register to
   blit_convolve() as the kernel buffer.  Neither register is in its own push
   list (`e92d4fb0` = r4,r5,r7-r11,lr for convolve, `e92d4f70` =
   r4,r5,r6,r8-r11,lr for separable), so neither is restored from the caller
   either.  Each function's missing register is the one the other one saves,
   which is what makes the two prologues read as symmetric - and is why an
   earlier version of this comment, which listed the separable push list right
   here and read only its r6 column, concluded that the separable path was
   intact.  r7 is absent from that list.  It was never safe either, and it is
   the one the firmware builds on.

   What the missing register costs, measured on this board (2026-09-15), two
   runs of nsgpu_convolve() called UNWRAPPED - the shape a caller gets:

     operator_type 0, real coeff array -> the destination came back with
       0x0040 at (2,2) where the source had 0x0202: the call reached the
       hardware but fed it coefficients read from an address it never
       computed.
     operator_type 1, NULL coeff, i.e. exactly the call the vendor's example
       makes -> *** HARD FAULT ***, CFSR 0x01000000 (UFSR.UNALIGNED),
       HFSR 0x40000000 (escalated).  The built-in path writes the same 5 bytes
       through the same garbage pointer and then the driver word-loads them.

   So this is not "the convolve path does not work on RGB565" - it is two
   functions in the library being unbuildable-to-call, and the fault is proof
   rather than theory.  Calling either one unwrapped is unbounded behaviour and
   the self test still does not.  The wrappers below are what makes them
   callable: the grab path goes through nsgpu_separable_convolve_k() on every
   trigger, and nsgpu_convolve_k() is exercised once at boot by
   gpu_convolve_probe() and by nothing else.                          */

/* The kernel handed to nsgpu_separable_convolve(), and why it is the identity.

   The library is undocumented - the tree ships nsgpu.lib with no source, the
   header carries nothing but the bare signature, and the vendor's example
   passes NULL everywhere it is actually compiled (its user-coefficient calls
   sit inside `#else`).  So the only kernel we can hold the GPU to is one we
   write ourselves.

   {0,0,1,0,0} because it is the identity under every reading of `coeff` - a
   horizontal 1-D kernel, a vertical one, or a 2-D one.  "The output matches the
   input" is then a verdict that does not rest on any guess about how the
   library interprets the array, which was the question this step existed to
   answer: does the convolve path run at all under this board's configuration,
   on RGB565.  filter_scale = 1.0 and filter_bias = 0 for the same reason -
   identity, so a wrong float ABI or a misunderstood scale cannot make a working
   path look broken.

   The disassembly has since answered the rest of it, and the answers are worth
   having here because nothing else in the tree records them:

     - operator_type 0 takes `coeff` as a 1-D array of `operator_num` ints and
       applies it twice, once as a 5x1 kernel and once as a 1x5 one, both times
       reading it in the same order.  So the filter is symmetric and diagonal,
       and tap index k is offset k - 2 in both passes - measured, not read off:
       see s_shift_coeff.
     - The function's opening validation loop, `for (i = 0; i < num*num; i++)
       if (coeff[i] == 0) return;`, is dead code.  It tests `coeff + i*4`, the
       address, which is never zero, so it never returns early.  Were it live it
       would reject the very identity kernel above, on coeff[0].
     - The length has to match operator_num; both are 5, which is what the
       vendor example uses for its arrays too (sobel_coeff[5], gaus_coeff[5]).

   "Matches the input" is not "equals the input" - up to 2 channel LSBs are lost
   on the way through; see GPU_CONV_GAP_MAX. */
static int s_conv_coeff[5] = { 0, 0, 1, 0, 0 };

/* The kernel that proves the coefficients are read at all.

   The identity kernel is the right verdict on "does the path run", and it is no
   evidence at all on "does the library use the array I gave it": a library that
   ignored `coeff` completely and passed the texture through would return the
   same picture, byte for byte.  So the self test needs one kernel a passthrough
   cannot imitate, and a one-pixel shift is the cheapest one - {0,0,0,1,0} moves
   the whole image by one pixel, which is unmistakable on a ramp where each pixel
   differs from its neighbour.

   Which way it moves is the point of measuring rather than asserting.  The
   library applies the array twice, once as a horizontal 5x1 kernel and once as
   a vertical 1x5 one, both times from the same array - so the shift is diagonal,
   and whether the two passes read the taps in the same order decides which of
   the four diagonals it lands on.  The self test counts all four.

   Measured, at 7440 interior pixels each: (+1,+1) gives 0 over the gap bound,
   (-1,-1) 7440, (+1,-1) 7440, (-1,+1) 7240.  So both passes read the array in
   the same order, tap k is offset k - 2, and the image moves by one pixel down
   and to the right.

   The confirming arithmetic is worth keeping, because it is what rules out the
   other reading of the same numbers - that the kernel was ignored and the shift
   counts are coincidence.  Taken bit-exactly, before the gap bound existed,
   (+1,+1) came back at 4136 mismatches, i.e. 3304 exact matches.  The identity
   leg puts the exact-match rate at 3276 of the same 7440 pixels, so a picture
   that really is the input shifted by one behaves exactly like this.  An
   unshifted picture would give about 7440, since pattern(x+1,y+1) differs from
   pattern(x,y) by 257 and the green field alone is 8. */
static int s_shift_coeff[5] = { 0, 0, 0, 1, 0 };

/* 1 prints the per-channel gap histogram of both buffers and the shift leg's
   four direction counts; 0 is the shipping behaviour, which is that only a
   failure prints anything.

   Back to 0 because the semantics are pinned down: the path runs, the kernel is
   read, tap k maps to offset k - 2, and the round trip costs at most two channel
   LSBs.  All of that is recorded in the comments above s_conv_coeff,
   s_shift_coeff and GPU_CONV_GAP_MAX, which is where it belongs - the numbers
   that established it were read once, and a trace is not a place to keep them.

   Turn it back on to re-establish them, or to see the shape of a failure: what
   it prints is the quantity GPU_CONV_GAP_MAX is set against.  It is placed
   immediately after the identity leg, because the shift leg below it overwrites
   both buffers. */
#define GPU_SELFTEST_TRACE  0

#define GPU_CONV_TAPS   5u
/* (taps-1)/2.  This many rows and columns at each edge are left out of the
   comparison.  What the library does with the pixels a kernel hangs off the
   edge of - clamp or zero-pad - is part of the same missing documentation, and
   it has no bearing on whether the path works. */
#define GPU_CONV_MARGIN 2u

/* The two constants of blit_separable_convolve_new()'s gain, hoisted here
   because stage 3 (filter_frame_with(), far below this point) is emitted before
   the template's own block and needs both.

   BLIT_SEP_GAIN_UNIT is the unit a scale argument is expressed in: the ARGUMENT
   is 32x the gain, so gain = arg / 32.  That is not read off the driver - the
   driver's dhd_set_texture_scalebias() converts a plain float
   (davehd_driver_txp_inl.h:644) - it is what the board's own arithmetic says,
   twice, in the one place this file works a gain out from a number it measured:

     - the impulse test at `pass 1's scale of 16 is a gain of 0.5`, from which
       the printed row results {15, 31, 47} follow (31.875 * 0.5 * {1,2,3}),
       and 16 / 32 = 0.5 is the only reading that reproduces them;
     - the same test's pass 2 at scale 32 gives 5.125 * 8.2258 = 42.2, i.e. the
       pass applies 8.2258 = (32 * MID_GAIN) / 32, the intermediate's factor and
       nothing else.

   BLIT_SEP_MID_GAIN is repeated here for the same reachability reason and is the
   SAME constant as the one defined with the template below; that one keeps its
   derivation.  Keep the two in step. */
#define BLIT_SEP_GAIN_UNIT 32.0f
#define BLIT_SEP_MID_GAIN  (255.0f / 31.0f)

/* blit_separable_convolve_new()'s mode selector, hoisted for the same
   reachability reason (stage 3 uses ROWCOL as of 2026-10-08 - it was ROWCOL_T
   until the COLUMN shape was re-measured; the template's block far below
   carries the derivation of each mode and of the transposed pair's two hard
   preconditions). */
#define BLIT_SEP_ROWS     0u /* one pass, `nu` 1xN row kernels summed in one draw */
#define BLIT_SEP_ROWCOL   1u /* two passes, 1xN across then Nx1 down (rank 1)     */
#define BLIT_SEP_ROWCOL_T 2u /* the same pair, both passes TRANSPOSED UV         */

/* ---- STAGE 39'S COST, SPLIT BY PHASE (2026-09-29) -------------------------

   Hoisted here with the two constants above and for the same reason: the
   template that fills g_blit_t[] is emitted thousands of lines below, and
   filter_frame_with() is between the two.

   The board says the new filter costs ~700 ms of `grab` where the library's
   cost ~65 (763 ms vs 112..119 ms on a 320x320 front end), while two 320x320
   passes are KNOWN to be ~124 ms of draw.  That gap has to be pointed at a
   phase before anything is changed, and these are the places it can be:

     g_sep_t[0]  filter_frame_with()'s own setup, before the template call
     g_sep_t[1]  blit_separable_convolve_new()'s whole call
     g_sep_t[2]  gpu_fb_put() after it
     g_blit_t[]  INSIDE the call, accumulated over its tiles, in blit_tile():
                   [0] the four dhd_set_* before the draw
                   [1] dhd_draw() itself
                   [2] dhd_wait(E_DHD_WAIT_WRITEBACK) - the GPU's own time
                   [3] gpu_drain() - our commit point
                   [4] how many tiles contributed (2 for a ROWCOL pair)

   So a big [2] means the draw is genuinely slow (sampler / border / memory),
   a big [3] means our drain is, and a big [0]/[1] means the driver's command
   path is.  None of the three has been measured on this board yet.

   All values are MICROSECONDS at 600 MHz, the same divisor camera.c's probe
   uses on DWT->CYCCNT, which is 32-bit and wraps at 7.158 s - one filter call
   is ~0.7 s, so a single call per reading is the whole safety argument and the
   accumulator is deliberately NOT carried across calls.

   blit_tile() is shared with the depthwise and conv paths, so the timing sits
   behind g_blit_t_on and is enabled only around this call: an untimed
   blit_tile() runs the same code it always did plus one load-and-test, and the
   numbers it accumulates are only ever read by the boot probe at the one point
   where this call is the last thing to have drawn.  g_sep_t[] is zeroed on
   entry to filter_frame_with() on every path including the early returns, so a
   stale reading cannot be printed as a current one. */
volatile uint32_t g_sep_t[4]  = { 0u, 0u, 0u, 0u };
volatile uint32_t g_blit_t[5] = { 0u, 0u, 0u, 0u, 0u };
volatile uint32_t g_blit_t_on = 0u;

/* What `nsgpu_convolve` REALLY passes, read back off the driver's own shadow
   state rather than inferred: `gpu cv2 t0 fmt 80A6 am 0 flt 11 da 0`.  So it is
   RGB565 texels with the SIGNED kernel mode - not the unsigned one an earlier
   reading of the disassembly had it as - on LINEAR addressing with the filter
   on.  Every one of those fields is what `dw_ktest` already uses except the
   texel organisation.

   Placed here and not with its siblings because stage 3
   (filter_frame_with(), far below) reads its 565 source in this format and is
   emitted before them. */
#define GPU_DW_FMT_SIGNED_565 ((dhd_texel_org_t)(E_DHD_TEXEL_ORG_RGB_565 | \
                                                 E_DHD_CLUT_KERNEL_SIGNED))


/* How far a channel may sit from the CPU's own value before the convolve is
   called wrong rather than imprecise.

   Measured, over all 8192 pixels of both buffers: 1 LSB after the first pass,
   2 after the second, and never more - the top bucket of the per-channel gap
   histogram is empty in both.  The arithmetic says the same thing.  The library
   normalises the kernel to 8-bit weights whose largest entry is 255, then hands
   the texture unit a scale of 256/255 to make up the difference, so the
   identity kernel is exact in real arithmetic and rounds every channel once per
   pass.  Two passes, at most two LSBs, and a 5-bit field losing one LSB is the
   whole of the worst case that looks alarming - red 2 falling to 0.

   4 and not 2 to leave the verdict some room, which costs nothing, because the
   failures this has to catch are nowhere near.  A wrong tap, an ignored kernel,
   a bad stride or a wrong pixel format each move a pixel by a whole texel or
   more; the cheapest of them is 8 channel LSBs, one step of a 6-bit green
   field.  The shift leg's own numbers bracket it: the correct direction comes
   back at 0 and the other three are in the thousands.  There is nothing between
   2 and 8 for a threshold to get wrong. */
#define GPU_CONV_GAP_MAX    4u

/* The kernel buffer nsgpu_separable_convolve() needs and never names.

   That function keeps the 8-bit kernel in r7 and NEVER ASSIGNS r7, nor saves
   it.  Read off nsgpu.lib over the whole 778-byte function: the push list is
   `{r4-r6,r8-r11,lr}`, which omits r7, and r7 occurs exactly seven times - the
   four `STRB r0,[r7,r4]` that write the kernel bytes, one `LDRB` that sums
   them, and two stack stores that hand r7 to blit_separable_convolve() as the
   kernel pointer.  Every one of those is a use as a base address; none is an
   assignment.  So the address the library writes the kernel to, and later
   reads it back from, is whatever the caller left in a callee-saved register.

   The same defect is in nsgpu_convolve(), which does it with r6 - and that is
   the call that faulted this board, CFSR 0x01000000 (UFSR.UNALIGNED): a word
   load through an odd r6 is a fault, and a byte store through a wild r6 is
   silent corruption of whatever it lands on.  Neither call is safe to make
   from C until the register is supplied.

   nsgpu_separable_convolve_k() below is that function with the one register
   set and nothing else changed.  It takes the same fourteen arguments plus the
   kernel buffer as a fifteenth, which AAPCS places on the stack after the nine
   the library already reads (`dst`..`pitch` = [sp+0]..[sp+32], so the new one
   is at [sp+36]); a naked function of two instructions loads it into r7 and
   tail-branches.  The kernel then lands in memory this firmware owns, and the
   same pointer is handed to the blitter, which is what the library always
   meant to do.

   The buffer has to be at least 5 bytes and word-aligned, because the driver
   reads it back as a kernel descriptor.  Note this is a workaround for a
   defect in a binary that cannot be recompiled.  If a future nsgpu.lib assigns
   r7 itself, this wrapper still sets it and the library uses its own value,
   so the wrapper degrades to harmless rather than to wrong.

   nsgpu_convolve_k() below is the same treatment for nsgpu_convolve() (r6) -
   the two entry points are the two halves of the library and differ in one
   thing that matters here.

   ---------------------------------------------------------------------------
   THE OTHER HALF OF THE DEFECT: r7 COMES BACK CLOBBERED (2026-09-16)

   Both wrappers used to load the register and tail-branch (`b`).  That is not
   enough, and the omission was measured rather than argued: `gpu_dhd_probe`'s
   size loop keeps its counter in r7, and every line printed
   `draw 805312692 px 2187476624` - the same numbers five times, whatever the
   size.  0x300018B4 is `s_conv_kernel` (Printf.map:19340), and 2187476624 is
   exactly (0x300018B4 * 0x300018B4) mod 2^32, so the counter held the kernel
   pointer by the time it was printed and the timings - which scale correctly
   with the size - were computed from the value it held before the call.

   The cause is the same omission read from the other side.  The library never
   assigns r7, so it also never RESTORES it: r7 is not in its push list, so at
   return r7 still points at the kernel buffer.  r7 is callee-saved under AAPCS,
   so any C caller is entitled to keep a live value there across the call and it
   will be silently destroyed.  Nothing in the header says so and nothing warns.

   This has been latent in every call site since the wrappers were written -
   gpu_selftest(), filter_frame_with() and gpu_convolve_probe() all make this
   call and then use values that may live in r7.  In filter_frame_with() the
   value used afterwards is `saved_fb`, which is handed to
   nsgpu_set_framebuffer(): had the compiler put it in r7, the driver's
   framebuffer address would have been repointed at &s_conv_kernel, and every
   later GPU call would write there.  It has not faulted yet, which is luck
   rather than safety - which register a variable lands in is not something the
   source controls.

   The fix cannot be a `push`.  The library reads its nine stack arguments at
   [sp+0]..[sp+32] RELATIVE TO ITS ENTRY sp, and it is entered by branch, so any
   push by the wrapper would shift all nine by the pushed amount and the library
   would read the saved registers as arguments.  And it cannot stay a `b`: a tail
   branch never returns to the wrapper, so there is no point at which r7 can be
   put back.

   So: `bl` instead of `b` (which also leaves sp untouched - `bl` writes lr, not
   the stack), park {r7, lr} in a file-scope pair, and restore both before
   returning.  The library's own epilogue pops lr back into pc, so with `bl` it
   returns to the instruction after the branch.  Not re-entrant, which is free
   here: this firmware's GPU path is synchronous and never nests.

   If a future nsgpu.lib assigns r7 itself, this wrapper still sets it and the
   library uses its own value, so the wrapper degrades to harmless rather than
   to wrong - the same property the register supply has always had. */
static uint32_t s_conv_kernel[2];

/* Where the wrapper parks the caller's r7 and lr across the call.  Two words,
   addressed through the literal pool so the wrapper needs no stack frame.

   `used` is load-bearing, not decoration.  The only reference to this array is
   inside the inline assembly above, and that reference is invisible to the
   compiler's liveness analysis - so without `used` the compiler drops the
   definition and the link fails with `L6218E: Undefined symbol s_conv_saved
   (referred from gpu.o)`.  The object disassembles perfectly either way; only
   the link tells you. */
static uint32_t s_conv_saved[2] __attribute__((used));

__attribute__((naked)) void nsgpu_separable_convolve_k(
    u32 operator_type, u32 operator_num, float filter_scale, int filter_bias,
    int* coeff, u32* dst, u32 dst_width, u32 dst_height,
    u32 dst_startx, u32 dst_starty, u32* src, u32 width, u32 height,
    u32 pitch, u32* kernel)
{
    __asm volatile("ldr r12, =s_conv_saved");
    __asm volatile("stm r12, {r7, lr}");
    __asm volatile("ldr r7, [sp, #36]");
    __asm volatile("bl nsgpu_separable_convolve");
    __asm volatile("ldr r12, =s_conv_saved");
    __asm volatile("ldm r12, {r7, lr}");
    __asm volatile("bx lr");
}

/* The signed half.  nsgpu_convolve() is the ONLY entry point that can express a
   NEGATIVE tap: it packs its kernel as (int8)(coeff[i] << 5), so a tap of -2
   becomes 0xC0, where the separable path above converts through VCVT.U32.F32 and
   any negative coefficient decays to 0.  That conversion is why the vendor's own
   separable tables are useless - g_coeff_edge_laplacian {1,-2,0,-2,1} and
   g_coeff_edge_scharr {3,0,-6,0,3} both come out as {255,0,0,0,255} once the
   negatives are gone, i.e. the same kernel, so those two built-ins were never
   validated.  The non-separable operators (sobel/prewitt/roberts, all signed)
   are the ones that work, and they only exist on this entry point.

   The prologue confirms the register with no guessing at all.  nsgpu_convolve
   saves {r4,r5,r7-r11,lr} - every register it uses EXCEPT r6 - while the
   separable function saves {r4-r6,r8-r11,lr}, every one except r7.  Each
   function's missing register is exactly the one the other one saves, which is
   why the omission is easy to miss when reading the header and why the header
   can declare both with identical argument lists.

   [sp+36] for the same reason as above: the argument lists are identical, nine
   core arguments land on the stack, and the hardfp float does not consume a core
   register.

   Same return-clobber as the separable wrapper above, with r6 instead of r7 -
   the register each function uses is exactly the one the other saves, so neither
   restores it.  Same fix, same shape. */
static uint32_t s_conv2d_kernel[2];
static uint32_t s_conv2_saved[2] __attribute__((used));   /* see s_conv_saved */

__attribute__((naked)) void nsgpu_convolve_k(
    u32 operator_type, u32 operator_num, float filter_scale, int filter_bias,
    int* coeff, u32* dst, u32 dst_width, u32 dst_height,
    u32 dst_startx, u32 dst_starty, u32* src, u32 width, u32 height,
    u32 pitch, u32* kernel)
{
    __asm volatile("ldr r12, =s_conv2_saved");
    __asm volatile("stm r12, {r6, lr}");
    __asm volatile("ldr r6, [sp, #36]");
    __asm volatile("bl nsgpu_convolve");
    __asm volatile("ldr r12, =s_conv2_saved");
    __asm volatile("ldm r12, {r6, lr}");
    __asm volatile("bx lr");
}

/* The stream controller's register block, from the driver's own map
   (nsgpu_hw.h: "GPU base address 0x5004_0000 -- 0x5004_3FFF, 16kB").  Written
   out as a literal rather than pulled in from that header because the header
   is not otherwise needed here and this file must not depend on the vendor's
   internal layout to stay compilable. */
#define GPU_STC_REG_BASE    0x50040000u

/**
 * @brief Pulse the GPU's AXI peripheral reset.
 *
 * The vendor's Convolve example never does this.  It is kept because it is the
 * one step that can move the part out of a state the power and clock enables
 * cannot, the reset is a self-clearing pulse (n32h7xx_rcc.c:
 * RCC_EnableAXIPeriphReset3 sets the bit and clears it again), and it is part
 * of the configuration that is known to work on this board.  Whether it is
 * strictly necessary has not been tested - no build has been flashed without
 * it - so this comment says that rather than claiming it is required.
 */
static void gpu_pulse_reset(void)
{
    RCC_EnableAXIPeriphReset3(RCC_AXI_PERIPHRST_GPU);
}

/**
 * @brief Power, clock and interrupt for the GPU, in the vendor example's order.
 */
static void gpu_platform_init(void)
{
    NVIC_InitType NVIC_InitStructure;

    PWR_MoudlePowerEnable(GRAPHIC_GPU_PWRCTRL, ENABLE);

    /* The low-power GPU clock as well as the main one: the example enables
       both together and the driver is not documented to work without it. */
    RCC_EnableAXIPeriphClk3(RCC_AXI_PERIPHEN_M7_GPU | RCC_AXI_PERIPHEN_M7_GPULP, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = GPU_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief Report a driver panic from the application side of the same event.
 *
 * The driver's own message is written by dhd_debug_out(), which is
 *
 *     va_start(...); vprintf(a_format, args); va_end(...);
 *
 * (davehd_os_ns.c:117).  Note what is MISSING from that: no fflush, no
 * UART_SendString, nothing that forces the standard library's stdout buffer out
 * to the port.  Every other line in this firmware's log is built with sprintf()
 * into a buffer and handed to UART_SendString(), so the buffered path has never
 * been exercised here - and `printf` in this project is not MicroLib, it is the
 * full C library, whose stdout is byte-buffered until it fills or is flushed.
 * The driver's panic path ends in `for (;;) { }` (davehd_debug.c:87), i.e. no
 * exit and therefore no flush, so it is entirely possible that a DHD_PANIC
 * prints into a buffer nobody ever drains and the board simply stops.
 *
 * That matters because DHD_PANIC is one of the two candidate explanations for
 * the inference that stops after `gpu dw L8` with no output at all, and the
 * absence of a "DHD_PANIC:" line in the log has been read as evidence against
 * it.  This callback makes that reading safe: it is called on the same path
 * (davehd_debug.c:72, only when the device handle validates) and it writes
 * through UART_SendString(), which cannot be buffered away.
 *
 * It is deliberately registered AFTER a successful init, because the callback
 * is skipped when the handle fails validation - and a panic raised while the
 * handle is the placeholder 1 is exactly the case where the driver's own print
 * is the only one there is.
 */
/* Declared in davehd_debug.h, which lives in the driver's src/ directory and is
   not on this project's include path; the definition is in davehd_os_ns.c.  The
   declaration is repeated rather than included because the two must agree
   anyway for the call below to link, and this way a missing include path shows
   up as one error instead of a build that silently uses a different prototype. */
extern void dhd_debug_out(const char* a_format, ...);

/* Same treatment, same reason.  This is the ONLY piece of the driver's own state
   that is reachable from here: it is a plain 32-bit global (`volatile
   dhd_uint32_t`, i.e. `unsigned int`), defined and exported by the library, and
   the kernel's `m_status` / `m_ringpos` are NOT - an accessor added to
   davehd_kernel_irq.c on 2026-09-28 linked as `L6218E: Undefined symbol`,
   because the target links that file's objects out of nsgpu.lib and not out of
   this tree.  See the note at the end of that file.

   What it is worth: davehd_kernel_irq.c:73 sets it to `m_stop_code` INSIDE the
   stop branch, immediately before that branch decides `m_status`.  So it answers
   two questions with one word - "did the ISR see the fault at all", and, because
   stop code 2 lands on IDLE, "is the driver willing to write STREAM_ADDRESS
   again".  0 (`E_STOP_IRQ_CODE_STREAM_END`) next to a hardware `sc2` means the
   fault happened and nobody looked at it. */
extern volatile uint32_t dhd_gpu_stream_status;

/* THE DRIVER'S DEVICE ARRAY - the thing the note above says was NOT reachable,
   and it is reachable after all, just not the way that note tried.

   What failed on 2026-09-28 was adding a NEW ACCESSOR FUNCTION to
   davehd_kernel_irq.c; the object comes out of nsgpu.lib, so the new function
   never existed.  The EXISTING GLOBAL is a different matter: `s_devices` is
   defined in davehd_kernel_driver.o (the map's symbol table puts it in that
   object's .bss), the library's own irq.c declares it `extern` - which is proof
   it is a global and not a file-static - and GPU_IRQHandler passes `&s_devices[0]`
   to the ISR, so the field the ISR writes per stop is the field the upload reads
   per commit.  Reaching it costs one extern and one header, and the header is
   included above so that `m_status`'s offset is the compiler's arithmetic and
   not a hand-counted byte.

   WHY IT IS WORTH REACHING, and this is the measurement that asked for it.  The
   `Code=201638` boot read:

       drain dead #0 ... sc2 gw0 b00000001        <- the fatal commit's busy vector
       gpu dw L17 ... bus001 strt1 nz1 sor001     <- the whole boot before it

   `strt1` is the finding: STREAM_ADDRESS MOVED ONCE in 1414 guarded commits, and
   that one commit is the one that wedged.  So the ordinary commit does NOT write
   a start - the stream stays alive and only RING_PAUSE_ADDRESS advances - and the
   fault is exactly "a commit that wrote a start", landing on a stream whose own
   busy bit says the STC is alive (b00000001).  That is `START_WHEN_BUSY` by
   definition, and it means the driver's `m_status` was not ACTIVE at that commit:
   davehd_kernel_ring.c:199 writes STREAM_ADDRESS only in that branch, and
   davehd_kernel_irq.c:97-100 is the only thing that makes it non-ACTIVE.

   So `m_status` answers the last open question - is the driver's IDLE a lie (the
   STC alive, which the busy bit says) or a truth (the STC really stopping, with
   the start landing in the tail of it) - and the two want opposite repairs.  It
   is READ on every commit (`ms%u` on the dead line, a three-way count on the
   summary line) and WRITTEN only through the `n` key, which keeps the driver
   ACTIVE while the STC is alive: the start is then never written, the pause
   address advance is what the live stream needs, and the whole boot behaves the
   way its own 1413 start-free commits already do.  That write is a repair with a
   real failure mode of its own - a live STC that does NOT resume on the pause
   advance leaves the queue unconsumed and the drain times out into the same latch
   the wedge uses - which is why it is a key and not a default. */
extern dhd_gpu_device_data_t s_devices[1];

/* ===================================================================
   THE LIBRARY'S OWN GLOBALS, AND THE ONE ENTRY POINT THAT DOES NOT COMMIT
   (2026-09-28, for the front end's own commit point - see gpu_scale_frame())

   nsgpu_colorkey() cannot be called on the shipped path, and the reason is not
   its arithmetic: its body ends in nsgpu_cmd_sync = dhd_flush + dhd_finish, and
   2026-09-28 exercised the consequence in the shipped configuration - eight
   `STALL s38 c0 p0` and a `GPU WEDGE 8s -> reset` with the main loop never
   coming out of the call ([[gpu-hang-and-drain-fix]] has the mechanism: the
   library's flush writes a start whenever the driver's belief is not ACTIVE, the
   STC refuses it with stop code 2, the hardware clears its own sync-IRQ enable,
   and dhd_finish then waits for a job that was never accepted - forever).

   So the front end has to own its commit point, and that turns on one question:
   how much of nsgpu_colorkey() can be reproduced outside the library?  The
   answer, measured rather than assumed, is all of it - but not for the reason
   this block first claimed.

   THE LIBRARY'S GLOBALS ARE NOT ALL LINKABLE, and Printf.map reads as if they
   were: its cross-reference prints `refers to nsgpu.o(.data) for NAME` for a
   file-static exactly as it does for a global.  What the linker actually did,
   and what the object file's own symbol table says (nsgpu.o extracted from
   nsgpu.lib with armar, dumped with fromelf; `Gb` = global, `Lc` = local):

       g_framebuffers            0x300006e4  Gb  80   nsgpu.o(.bss)   linkable
       g_prim                    0x30000790  Gb  16   nsgpu.o(.bss)   linkable
       a_handle                  0x30000008  Gb   4   nsgpu.o(.data)  linkable
       framebuffer_acitve_index             Lc   4   nsgpu.o(.data)  NOT
       gpu_clip_mode                        Lc   4   nsgpu.o(.data)  NOT
       clip_xmin/ymin/xmax/ymax             Lc   2   nsgpu.o(.data)  NOT

   The first version of this block extern'd all six and the build died on six
   L6218E undefined symbols, one per `Lc` row - which is the same reading from
   the other side.

   The three unreachable ones are not needed, and each is replaced by something
   PROVED out of the same object file rather than guessed:

     - gpu_clip_mode -> E_DHD_CLIP_FRAMEBUFFER.  Its initialiser is 1, and no
       writer survives in the image: nsgpu_set_clip_mode() is its only one and
       Printf.map:19092 lists it under `Removing`, and nsgpu_init() refers to
       neither.  So it is 1 for the whole boot.  This is the load-bearing one -
       it decides whether the utility cliprect below is consulted at all.  See
       gpu_vg_rect().
     - clip_xmin/ymin/xmax/ymax -> 0, 0, 0, 0.  All four initialisers are zero
       and nsgpu_draw_clip_setup(), their only writer, is `Removing` too
       (Printf.map:19052).  Inert under FRAMEBUFFER mode; reproduced anyway, so
       driver state after the call is the library's.
     - framebuffer_acitve_index / g_framebuffers -> not read at all.  Colorkey
       takes four values out of the entry - destination address, width, height
       and format - and those are GPU_SCALE_DST/W/H plus
       dhd_get_framebuffer_format(), which is the same value because
       nsgpu_init() set both from one argument.  And nsgpu_draw_rotblit() ignores
       its a_fb outright: the prologue clobbers r1 at nsgpu_lib_dis.txt:31939 and
       never reads it before that.

   So the path needs no library global but a_handle, and even g_prim is unused -
   gpu_vg_rect() carries its own scratch. */

/* nsgpu_draw_rotblit() is exported (map: 0x0002f415, Thumb Code 670) and it is
   the ONE library entry point on this path that does NOT commit.  Its refers-to
   list is exactly

       sinf, cosf, dhd_bound_quad, dhd_clip_bbox, dhd_setup_edge_aa,
       dhd_set_edge_buffer, dhd_set_uv_buffer, dhd_set_bbox, dhd_draw,
       gpu_clip_mode                                    (Printf.map:11457-11466)

   with no nsgpu_cmd_sync in it, which is precisely why the blit can be
   reproduced byte-for-byte: the drawing code is STILL the library's, and only
   the commit moves.  That is the difference between this and reimplementing the
   blit, and it is the whole reason the front end's fix is cheap.

   Declared here because no header declares it - not the in-tree nsgpu.h and not
   the one the build actually uses (the project's include path points at
   E:\1 Nations.N32H7xx_Library.1.2.0\middlewares\gpu\, whose nsgpu.h DOES carry
   nsgpu_colorkey/data_fill/vgdraw_rect but not this one).  The signature is read
   off the call site (nsgpu_lib_dis.txt:29639-29643): r0 = device, r1 = &fb,
   r2 = centre x, r3 = centre y, s0 = the angle in DEGREES as a float, then two
   stack arguments - and the body reads those two with VCVT.F32.S32 (31894-31895),
   so they are signed. */
extern void nsgpu_draw_rotblit(void* a_device, void* a_fb, int32_t a_cx,
                               int32_t a_cy, float a_angle,
                               int32_t a_width, int32_t a_height);

static void gpu_panic_report(const char* a_message, const char* a_file,
                             uint32_t a_line)
{
    char buf[200];

    sprintf(buf, "PANIC-CB %s (%s:%u)\r\n",
            (a_message != NULL) ? a_message : "?",
            (a_file != NULL) ? a_file : "?", (unsigned)a_line);
    UART_SendString(buf);
}

/* The stall reporter's stage setter, declared here rather than beside its
   definition (it needs gpu_drain() and DW_PROG, both far below) because the BOOT
   PATH arms it too: gpu_init() and gpu_selftest() both reach unbounded library
   commits, and they run before anything else in this file.  See the block over
   gpu_selftest() and the one over gpu_front_prog() itself. */
static void gpu_front_prog(uint32_t stage);

int gpu_init(void)
{
    int result;

    gpu_platform_init();
    gpu_pulse_reset();

    memset((void*)GPU_VIDMEM_BASE, 0, GPU_ZERO_BYTES);

    /* See the note on a_handle above.  This one line is what turns a driver
       failure from "the board locks up inside the library's error handler"
       into "nsgpu_init() returns an error code". */
    a_handle = 1u;

    /* RGB565 first, because the LCD frame buffers are RGB565 and a GPU that
       speaks the same format can later write a real frame buffer with no
       conversion pass.  It is also what the vendor ships: four of their five
       GPU examples (DataCopy, DataFill, DrawGraphic, RLD) init with
       E_DHD_PIXEL_ORG_RGB_565; only Convolve uses L_8, because it convolves a
       single-channel array.  The fallback is kept because the format is not
       the only input to this call and a refusal has to be survivable, not
       because RGB565 is a guess. */
    /* ARMED, because nsgpu_init() is itself an unbounded commit and it is the
       FIRST GPU call this firmware makes.  Read out of nsgpu_lib_dis.txt, not
       inferred: the library's body ends 35928 `MOVS r0,#0` / 35929 `BL
       nsgpu_cmd_sync`, and nsgpu_cmd_sync's own body (29366-29379) branches to
       `dhd_finish` for anything but r0 == 1 - so this is dhd_finish(), not the
       flush-only form, and the `MOVS r0,#0` is the argument that decides it.
       Nothing armed it before, so a wedge here was a silent dead board: no
       STALL, no dump, no reset, and `k` unreachable because the main loop has
       not started yet.  Same recipe and same reason as the self test below; see
       the block over gpu_selftest() for why arming at boot works at all (the DVP
       ISR is already running, main.c:496-497).

       Stages 20 and 21 are this function's two attempts, so the line names the
       one that never returned.  Note what an armed failure looks like from the
       outside: the wedge is now a REPORTED reset rather than a dead board, so a
       board that wedges in nsgpu_init() will reboot and STALL again - a visible
       loop, which is the intended trade. */
    gpu_front_prog(20u);
    result = nsgpu_init(GPU_VIDMEM_BASE, 1u, 800u, 480u,
                        E_DHD_PIXEL_ORG_RGB_565,
                        NSGPU_CLIP_MODE, NSGPU_ACC_MODE, GPU_ACC_BASE);
    gpu_front_prog(0u);
    if (result == NSGPU_INIT_OK)
    {
        s_pixel_bytes = 2u;
        s_fill_argb   = 0xFFFFFFFFu;    /* white...            */
        s_fill_expect = 0xFFFFu;        /* ...in RGB565 memory */
        s_ready       = true;
    }
    else
    {
        /* That path ran dhd_init() and stored its NULL return back into
           a_handle, so the guard has to be re-applied before the retry -
           otherwise the retry is the one that hangs. */
        a_handle = 1u;

        gpu_front_prog(21u);
        result = nsgpu_init(GPU_VIDMEM_BASE, 1u, 800u, 480u,
                            E_DHD_PIXEL_ORG_L_8,
                            NSGPU_CLIP_MODE, NSGPU_ACC_MODE, GPU_ACC_BASE);
        gpu_front_prog(0u);
        if (result == NSGPU_INIT_OK)
        {
            s_pixel_bytes = 1u;
            s_fill_argb   = 0xFFFFFFFFu;    /* white is white...   */
            s_fill_expect = 0xFFu;          /* ...in L_8 too       */
            s_ready       = true;
        }
    }

    /* s_ready gates the self test, and it is set only on success.  It used to
       be unconditional, on the argument that a run where init reported failure
       but the pixels still moved would be worth seeing.  That argument stopped
       being safe the moment a_handle was written above: on a failed init the
       driver is holding handle 1, which is not a device, and nsgpu_data_fill()
       would dereference it.  The diagnostic is not lost - the library's own
       "ERROR: ..." text now reaches the UART instead of faulting on a
       semihosting BKPT, which says more than a pixel count would. */

    if (s_ready)
    {
        (void)dhd_set_panic_callback((dhd_handle_t)a_handle, gpu_panic_report);
    }

    /* Does the driver's own reporting path reach this UART at all?  Two lines,
       one boot, and they answer a question the rest of this round depends on:
       if neither appears, then "no DHD_PANIC in the log" is not evidence that no
       panic fired, and every future reading of a silent stop has to say so.
       Both go through the buffered stdout the driver uses (one directly, one
       through dhd_debug_out, which is what DHD_PANIC calls).  Temporary.
       2026-10-12: gated on BOOT_VERBOSE - it is a boot diagnostic and has no
       business in the delivered log.  Note this only silences THESE two
       probes; a real DHD_PANIC still reaches the UART through
       gpu_panic_report (installed just above) and is never gated. */
#if BOOT_VERBOSE
    printf("gpu out printf %d\r\n", 7);
    dhd_debug_out("gpu out dhd_debug_out %d\r\n", 7);
#endif

    return result;
}

/**
 * @brief Put a value the fill can never produce into the destination buffer.
 *
 * The fill test used to rely on "the CPU writes nothing first, so a pixel that
 * reads back right is one the GPU wrote".  That is only half true: it also
 * reads back right if the GPU wrote nothing and the SDRAM behind the buffer
 * happened to hold the expected value already - and since the buffers are at
 * fixed addresses and this test runs once per trigger, past runs leave exactly
 * that value lying there.  Poisoning makes the claim real: every byte starts
 * out wrong, so the fill has to have touched it to score.
 */
static uint8_t poison_byte(void)
{
    return (uint8_t)(~s_fill_expect & 0xFFu);
}

static void poison_fill_dst(void)
{
    uint32_t i;
    uint32_t n = GPU_TEST_W * GPU_TEST_H * s_pixel_bytes;
    volatile uint8_t* p = (volatile uint8_t*)GPU_TEST_DST;
    uint8_t poison = poison_byte();

    for (i = 0u; i < n; i++)
    {
        p[i] = poison;
    }
}

/**
 * @brief The value make_test_pattern() writes at (x, y).
 *
 * Shared with the comparison rather than read back out of the source buffer,
 * and that is the point rather than a tidy-up.  nsgpu_separable_convolve()
 * filters its source in place, so a comparison
 * against the source buffer as read back would answer "did the GPU produce the
 * right output AND leave its input alone" - two questions, and the input one
 * has no right answer to assert.  Against the formula the comparison answers
 * only the question that has one.
 *
 * The pattern depends on both coordinates, so a blit that drops a row, shifts
 * a column or uses the wrong stride all show up as mismatches.
 */
static uint16_t pattern_pixel(uint32_t x, uint32_t y)
{
    if (s_pixel_bytes == 2u)
    {
        return (uint16_t)(((y << 8) | x) & 0xFFFFu);
    }

    return (uint16_t)((x + y) & 0xFFu);
}

/**
 * @brief Read one pixel out of a test buffer, whatever the pixel width is.
 *
 * The counters below are written against this so they have no format branch of
 * their own.  They used to - and the branch they had was the dangerous kind: a
 * count that returns 0 for a format it does not know is a test that reports
 * success without testing, and nsgpu_init() does fall back to a one-byte format
 * if the driver refuses RGB565.  Nothing was wrong on this board because the
 * fallback never happened; that is luck, not a design.
 */
static uint16_t read_pixel(uint32_t base, uint32_t index)
{
    if (s_pixel_bytes == 2u)
    {
        return ((volatile uint16_t*)base)[index];
    }

    return (uint16_t)((volatile uint8_t*)base)[index];
}

/**
 * @brief Largest per-channel difference between two pixels, in LSBs of each
 *        channel's own field.
 *
 * Comparing the words is the wrong measurement, and this is the whole reason
 * this is a function of its own.  For RGB565 red is the top five bits, so one
 * red LSB is 0x0800 in the word while one blue LSB is 1 and a green LSB is
 * 0x20 - so a filter that is exact but passes through an 8-bit intermediate,
 * losing a fraction of an LSB in each channel, shows up as a word difference of
 * up to a few thousand, which reads like a structural error.  Per channel it is
 * a difference of one, which reads like what it is.
 *
 * In one-byte mode there are no fields to split, so the gap is the difference.
 * That case only arises if nsgpu_init() fell back to a one-byte format, and it
 * is spelled out rather than refused because "the convolve arithmetic is off by
 * two counts of 255" is as meaningful on an 8-bit luminance texture as on
 * RGB565 - see GPU_CONV_GAP_MAX.
 */
static uint32_t channel_gap(uint16_t a, uint16_t b)
{
    int g;
    int t;

    if (s_pixel_bytes != 2u)
    {
        return (uint32_t)((a > b) ? ((uint32_t)a - (uint32_t)b)
                                  : ((uint32_t)b - (uint32_t)a));
    }

    g = (int)((a >> 11) & 0x1Fu) - (int)((b >> 11) & 0x1Fu);
    if (g < 0)
    {
        g = -g;
    }

    t = (int)((a >> 5) & 0x3Fu) - (int)((b >> 5) & 0x3Fu);
    if (t < 0)
    {
        t = -t;
    }
    if (t > g)
    {
        g = t;
    }

    t = (int)(a & 0x1Fu) - (int)(b & 0x1Fu);
    if (t < 0)
    {
        t = -t;
    }
    if (t > g)
    {
        g = t;
    }

    return (uint32_t)g;
}

/**
 * @brief Print how far a buffer is from the pattern, in channel LSBs.
 *
 * Four counts and a maximum: how many pixels match exactly, how many are off by
 * one LSB in their worst channel, by two, and by more than two; then the worst
 * gap seen anywhere.
 *
 * The shape of those five numbers is the diagnosis.  Almost everything in the
 * first two buckets with a small maximum means the path runs and the arithmetic
 * is right, and what is left is the precision cost of the format the library
 * filters in - worth knowing, not worth calling a failure.  A pile in the third
 * bucket means something structural, and then the shift leg's four counts say
 * whether the kernel still points where it did.
 *
 * The whole image is scanned, edges included, unlike count_gap_exceed(): this is
 * a description of what came back rather than a verdict on the interior, and the
 * edge pixels are part of what is interesting about it.
 *
 * Trace-only, and guarded as such, so that turning GPU_SELFTEST_TRACE off leaves
 * no unused function behind.
 */
#if GPU_SELFTEST_TRACE
static void dump_gap(const char* tag, uint32_t base)
{
    uint32_t x;
    uint32_t y;
    uint32_t n0 = 0u;
    uint32_t n1 = 0u;
    uint32_t n2 = 0u;
    uint32_t n3 = 0u;
    uint32_t worst = 0u;

    for (y = 0u; y < GPU_TEST_H; y++)
    {
        for (x = 0u; x < GPU_TEST_W; x++)
        {
            uint32_t g = channel_gap(read_pixel(base, (y * GPU_TEST_W) + x),
                                     pattern_pixel(x, y));

            if (g == 0u)
            {
                n0++;
            }
            else if (g == 1u)
            {
                n1++;
            }
            else if (g == 2u)
            {
                n2++;
            }
            else
            {
                n3++;
            }

            if (g > worst)
            {
                worst = g;
            }
        }
    }

    sprintf(Message, "gpu selftest gap %s %lu %lu %lu %lu %lu\r\n", tag,
            (unsigned long)n0, (unsigned long)n1, (unsigned long)n2,
            (unsigned long)n3, (unsigned long)worst);
    UART_SendString(Message);
}
#endif

/**
 * @brief Count destination pixels that are not the pattern moved by (dx, dy),
 *        to within the same rounding bound the identity leg gets.
 *
 * The shift leg's whole question is direction, so the same count is taken for
 * each candidate and the small one is the answer.  Bit-exactness would make
 * every candidate look bad - a correctly shifted image still differs from the
 * exact shifted pattern on the pixels the rounding moved, about half of them -
 * so this uses the same gap bound as the identity leg, where the correct
 * direction lands at 0 and the others in the thousands.
 *
 * Pixels whose source falls outside the image are skipped rather than clamped:
 * there is nothing to predict for them, and counting them would put a floor
 * under every candidate and hide which one is actually zero.
 */
static uint32_t count_shift_gap_exceed(int dx, int dy)
{
    uint32_t x;
    uint32_t y;
    uint32_t bad = 0u;

    for (y = GPU_CONV_MARGIN; y < (GPU_TEST_H - GPU_CONV_MARGIN); y++)
    {
        for (x = GPU_CONV_MARGIN; x < (GPU_TEST_W - GPU_CONV_MARGIN); x++)
        {
            int sx = (int)x + dx;
            int sy = (int)y + dy;

            if ((sx < 0) || (sx >= (int)GPU_TEST_W) ||
                (sy < 0) || (sy >= (int)GPU_TEST_H))
            {
                continue;
            }

            if (channel_gap(read_pixel(GPU_TEST_DST, (y * GPU_TEST_W) + x),
                            pattern_pixel((uint32_t)sx, (uint32_t)sy)) >
                GPU_CONV_GAP_MAX)
            {
                bad++;
            }
        }
    }

    return bad;
}

/**
 * @brief Count the pixels the GPU filled that do not match the colour asked for.
 */
static uint32_t count_fill_mismatch(void)
{
    uint32_t x;
    uint32_t y;
    uint32_t bad = 0u;

    if (s_pixel_bytes == 2u)
    {
        volatile uint16_t* p = (volatile uint16_t*)GPU_TEST_DST;
        for (y = 0u; y < GPU_TEST_H; y++)
        {
            for (x = 0u; x < GPU_TEST_W; x++)
            {
                if (p[y * GPU_TEST_W + x] != (uint16_t)s_fill_expect)
                {
                    bad++;
                }
            }
        }
    }
    else
    {
        volatile uint8_t* p = (volatile uint8_t*)GPU_TEST_DST;
        for (y = 0u; y < GPU_TEST_H; y++)
        {
            for (x = 0u; x < GPU_TEST_W; x++)
            {
                if (p[y * GPU_TEST_W + x] != (uint8_t)s_fill_expect)
                {
                    bad++;
                }
            }
        }
    }

    return bad;
}

static void make_test_pattern(void)
{
    uint32_t x;
    uint32_t y;

    if (s_pixel_bytes == 2u)
    {
        volatile uint16_t* p = (volatile uint16_t*)GPU_TEST_SRC;
        for (y = 0u; y < GPU_TEST_H; y++)
        {
            for (x = 0u; x < GPU_TEST_W; x++)
            {
                p[y * GPU_TEST_W + x] = pattern_pixel(x, y);
            }
        }
    }
    else
    {
        volatile uint8_t* p = (volatile uint8_t*)GPU_TEST_SRC;
        for (y = 0u; y < GPU_TEST_H; y++)
        {
            for (x = 0u; x < GPU_TEST_W; x++)
            {
                p[y * GPU_TEST_W + x] = (uint8_t)pattern_pixel(x, y);
            }
        }
    }
}

/**
 * @brief Count the bytes that differ between the pattern written by the CPU
 *        and what the GPU blitted into the destination buffer.
 */
static uint32_t count_copy_mismatch(void)
{
    uint32_t i;
    uint32_t n = GPU_TEST_W * GPU_TEST_H * s_pixel_bytes;
    uint32_t bad = 0u;
    volatile uint8_t* dst = (volatile uint8_t*)GPU_TEST_DST;
    volatile uint8_t* src = (volatile uint8_t*)GPU_TEST_SRC;

    for (i = 0u; i < n; i++)
    {
        if (dst[i] != src[i])
        {
            bad++;
        }
    }

    return bad;
}

/**
 * @brief Count the pixels of a buffer that are further from the pattern than
 *        GPU_CONV_GAP_MAX allows, in channel LSBs.
 *
 * The verdict for a buffer the convolve read or wrote, and the reason it is not
 * a plain "how many pixels changed".
 *
 * Both of those simpler tests are unpassable here, and the first run of this
 * self test failed on both counts while the GPU was working perfectly:
 *
 *   - Bit-exactness.  The scaled identity kernel comes back with a fractional
 *     LSB lost per channel per pass - up to 2 in all - so comparing words is
 *     guaranteed to differ on roughly half the pixels.  Measured: the source
 *     buffer comes back exactly equal at 3625 of 8192 pixels, off by one LSB at
 *     the other 4567, and off by more than one at none.
 *   - "The source is untouched."  It is not, and cannot be: pass 1 runs in
 *     place.  A count of changed source pixels is therefore large by design and
 *     says nothing about whether the convolve worked.
 *
 * Bounding the per-channel gap passes both and still fails hard on anything
 * structural, because the two regimes do not overlap - see GPU_CONV_GAP_MAX.
 *
 * Only the interior is compared, GPU_CONV_MARGIN rows and columns in from each
 * edge; see the note on GPU_CONV_MARGIN.  The expectation comes from
 * pattern_pixel() rather than from the source buffer, because the source is
 * scratch after pass 1.
 */
static uint32_t count_gap_exceed(uint32_t base)
{
    uint32_t x;
    uint32_t y;
    uint32_t bad = 0u;

    for (y = GPU_CONV_MARGIN; y < (GPU_TEST_H - GPU_CONV_MARGIN); y++)
    {
        for (x = GPU_CONV_MARGIN; x < (GPU_TEST_W - GPU_CONV_MARGIN); x++)
        {
            uint32_t i = (y * GPU_TEST_W) + x;

            if (channel_gap(read_pixel(base, i), pattern_pixel(x, y)) >
                GPU_CONV_GAP_MAX)
            {
                bad++;
            }
        }
    }

    return bad;
}

/**
 * @brief Read the first 8 bytes of the destination buffer into an array.
 *
 * Called once after each operation that writes the buffer, because the next
 * one overwrites it.  Eight bytes so the line reads the same whether a pixel
 * is one byte or two.
 */
static void capture_bytes(uint32_t* out)
{
    volatile uint8_t* p = (volatile uint8_t*)GPU_TEST_DST;
    uint32_t i;

    for (i = 0u; i < 8u; i++)
    {
        out[i] = (uint32_t)p[i];
    }
}

/**
 * @brief Print eight raw bytes off the destination buffer, under a short tag.
 */
static void print_bytes(const char* tag, const uint32_t* obs)
{
    sprintf(Message, "gpu selftest %s %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
            tag,
            (unsigned)obs[0], (unsigned)obs[1], (unsigned)obs[2], (unsigned)obs[3],
            (unsigned)obs[4], (unsigned)obs[5], (unsigned)obs[6], (unsigned)obs[7]);
    UART_SendString(Message);
}

/**
 * @brief Print why the self test failed: the counts, and the first few bytes of
 *        the destination buffer after the fill and after the convolve.
 *
 * Only called on failure - on success there is nothing here that "gpu selftest
 * OK" does not already say, and this runs at boot where the log is meant to be
 * short.  On failure it is worth its length: "FAIL 8192" cannot tell "the fill
 * ran and converted the colour to something else" from "the fill never ran",
 * and the two want opposite fixes.  The counts split them apart (fill 8192 /
 * copy 0 means the engine works and only the fill's colour convention is
 * wrong), and the raw bytes say which colour the GPU really produced.  Eight
 * bytes, so the line reads the same whether a pixel is one byte or two.
 *
 * `fill` and `copy` are bit-exact failures; `sepc` and `src` are pixels outside
 * the gap bound in the destination and in the source respectively, and `src`
 * being nonzero is normal in small numbers - the library filters the source in
 * place, so that buffer is scratch by design.  `shift` is printed separately by
 * the caller because it has a different failure mode: a nonzero shift count with
 * a clean sepc means the identity convolution is fine and the coefficients are
 * not being read.
 */
static void print_selftest(uint32_t bad_fill, uint32_t bad_copy,
                           uint32_t bad_sepc, uint32_t bad_src,
                           const uint32_t obs_fill[8], const uint32_t obs_sepc[8])
{
    sprintf(Message, "gpu selftest fill %lu copy %lu sepc %lu src %lu exp %04X px %lu\r\n",
            (unsigned long)bad_fill, (unsigned long)bad_copy,
            (unsigned long)bad_sepc, (unsigned long)bad_src,
            (unsigned)s_fill_expect, (unsigned long)s_pixel_bytes);
    UART_SendString(Message);

    print_bytes("dst", obs_fill);
    print_bytes("sdst", obs_sepc);
}

/* The boot path's arm of the DVP ISR's stall reporter - the plan's ⚠️ item, and
   the second of the boot path's two unarmed holes (gpu_init()'s nsgpu_init() is
   the other, armed above for the same reason).

   THE HOLE.  All four legs below are unbounded library commits:
   nsgpu_data_fill -> nsgpu_cmd_sync, nsgpu_data_copy -> nsgpu_cmd_sync, and two
   nsgpu_separable_convolve_k -> nsgpu_separable_convolve -> nsgpu_cmd_sync, all
   map-verified in Printf.map, and nsgpu_cmd_sync is dhd_flush + dhd_finish - the
   unbounded wait.  Nothing in this function wrote g_gpu_dw_prog before this
   change, and the reporter ignores 0 by design, so a wedge in ANY of the four
   legs produced no `STALL` line, no STC dump and no eight-second reset: the board
   just stopped with `gpu selftest`'s predecessor as its last line, and
   gpu_revive() (`k`) was unreachable because `k` needs a running main loop.  That
   is a power-cycle, on every power-up.

   WHY ARMING HERE IS VALID AT ALL: the reporter lives in the DVP ISR, so it can
   only fire if the DVP is already running.  It is - main.c:496-497 says so in as
   many words ("摄像头和 DVP 这时都已在跑"), which is also why this self test is run
   at this point in the boot rather than earlier.  Had the camera not been up,
   this arm would be a no-op and this comment would be a lie.

   WHAT IT DOES NOT DO: it does not bound the wait, and no edit here could.
   dhd_finish() is in davehd_driver.o inside nsgpu.lib, so a second definition is
   a link error, and transcribing nsgpu_data_fill / nsgpu_data_copy /
   nsgpu_separable_convolve would defeat the point - the self test exists to check
   the LIBRARY against the CPU, not our transcription of it.  The arm buys a
   diagnosis and an exit (`STALL sN c0 p0` + the STC dump + `GPU WEDGE 8s ->
   reset`), not a repair.  The front end's own commits were made bounded
   separately; see the contract block over gpu_sb_reset().

   STAGES 6..9, one PER LEG and not one for the function: the reporter's
   identical-word counter resets when the word returns to 0, so a `STALL s6` can
   only mean leg 1 rather than "the self test has been running a while", and the
   line names the leg that never returned.  6..9 are free (the live set is 1..5,
   6..9, 10..19, 20/21 for gpu_init(), 30..39, 40..44 - see the stage table over
   DW_PROG) and are below 42, so `1 + stage*1e8` does not wrap.

   Every exit path is straight-line: the only early return, `!s_ready`, is above
   all four arms, so no path can leave the word armed.  (Its prototype is at the
   top of the file beside gpu_init(), which arms it too.) */

bool gpu_selftest(uint32_t* bad)
{
    uint32_t bad_fill;
    uint32_t bad_copy;
    uint32_t bad_sepc;
    uint32_t bad_src;
    uint32_t bad_shift;
    uint32_t obs_fill[8];
    uint32_t obs_sepc[8];

    if (bad != NULL)
    {
        *bad = 0u;
    }

    if (!s_ready)
    {
        return false;
    }

    /* 1. Fill.  Poison first, so a matching pixel is one the GPU wrote; see
          poison_fill_dst(). */
    poison_fill_dst();

    gpu_front_prog(6u);
    nsgpu_data_fill((u32*)GPU_TEST_DST, GPU_TEST_W, GPU_TEST_H, 0u, 0u,
                    s_fill_argb, 255u, GPU_TEST_W, GPU_TEST_H);
    gpu_front_prog(0u);

    bad_fill = count_fill_mismatch();

    /* Read the result out now - the blit below overwrites it. */
    capture_bytes(obs_fill);

    /* 2. Blit.  Now both buffers start out CPU-written, so this checks the
          GPU's read path as well as its write path. */
    make_test_pattern();

    gpu_front_prog(7u);
    nsgpu_data_copy((u32*)GPU_TEST_DST, GPU_TEST_W, GPU_TEST_H, 0u, 0u,
                    (u32*)GPU_TEST_SRC, GPU_TEST_W, GPU_TEST_H,
                    GPU_TEST_W, s_pixel_bytes);
    gpu_front_prog(0u);

    bad_copy = count_copy_mismatch();

    /* 3. Convolve, identity kernel.  The expected output is the pattern itself;
          see s_conv_coeff for why the kernel has to be the identity, and the
          note above it for why this is the separable entry point and not
          nsgpu_convolve().

          Both buffers are rebuilt first: the library leaves the source filtered
          in place, so that buffer's only job is to be this call's input.  Both
          are then checked against the pattern to within GPU_CONV_GAP_MAX, the
          source included - "the source came back untouched" is not a test this
          call can pass, and asking for it is what made the first version of this
          self test fail while the GPU was working.  See count_gap_exceed(). */
    make_test_pattern();
    poison_fill_dst();
    gpu_front_prog(8u);
    nsgpu_separable_convolve_k(0u, GPU_CONV_TAPS, 1.0f, 0, s_conv_coeff,
                               (u32*)GPU_TEST_DST, GPU_TEST_W, GPU_TEST_H, 0u, 0u,
                               (u32*)GPU_TEST_SRC, GPU_TEST_W, GPU_TEST_H, GPU_TEST_W,
                               s_conv_kernel);
    gpu_front_prog(0u);
    bad_sepc = count_gap_exceed(GPU_TEST_DST);
    capture_bytes(obs_sepc);
    bad_src  = count_gap_exceed(GPU_TEST_SRC);

#if GPU_SELFTEST_TRACE
    /* Here, and not at the end of the function where the rest of the trace
       used to live.  The shift leg below overwrites both buffers, and a gap
       histogram of ITS output read against the UNSHIFTED pattern describes a
       correct result as thousands of pixels out of bounds - which is exactly
       what it did, next to a verdict of OK. */
    dump_gap("dst", GPU_TEST_DST);
    dump_gap("src", GPU_TEST_SRC);
#endif

    /* 4. Convolve, shift kernel.  The leg that proves the coefficients are read
          at all: the identity kernel gives the same picture whether the library
          used the array or ignored it, so on its own it cannot tell a working
          filter from a passthrough.  See s_shift_coeff.

          Exactly one of the four diagonals may come back clean.  Asserting the
          one that does, rather than the count being small, is the point - a
          result that matched two opposite directions, or matched one only after
          the tap order changed, would be a different library than the one this
          was characterised against, and should say so. */
    make_test_pattern();
    poison_fill_dst();
    gpu_front_prog(9u);
    nsgpu_separable_convolve_k(0u, GPU_CONV_TAPS, 1.0f, 0, s_shift_coeff,
                               (u32*)GPU_TEST_DST, GPU_TEST_W, GPU_TEST_H, 0u, 0u,
                               (u32*)GPU_TEST_SRC, GPU_TEST_W, GPU_TEST_H, GPU_TEST_W,
                               s_conv_kernel);
    gpu_front_prog(0u);

    bad_shift = count_shift_gap_exceed(1, 1);

    if ((count_shift_gap_exceed(-1, -1) == 0u) ||
        (count_shift_gap_exceed(1, -1) == 0u) ||
        (count_shift_gap_exceed(-1, 1) == 0u))
    {
        /* More than one direction clean: the four counts no longer separate,
           so the "+1,+1" reading above is not the one this result supports. */
        bad_shift = GPU_TEST_W * GPU_TEST_H;
    }

#if GPU_SELFTEST_TRACE
    /* The four directions, so that a failure here says which ones came back
       clean rather than only that the expected one did not. */
    sprintf(Message, "gpu selftest shf %lu %lu %lu %lu\r\n",
            (unsigned long)bad_shift,
            (unsigned long)count_shift_gap_exceed(-1, -1),
            (unsigned long)count_shift_gap_exceed(1, -1),
            (unsigned long)count_shift_gap_exceed(-1, 1));
    UART_SendString(Message);
#endif

    if ((bad_fill != 0u) || (bad_copy != 0u) ||
        (bad_sepc != 0u) || (bad_src != 0u) || (bad_shift != 0u))
    {
        sprintf(Message, "gpu selftest shift %lu\r\n", (unsigned long)bad_shift);
        UART_SendString(Message);

        print_selftest(bad_fill, bad_copy, bad_sepc, bad_src,
                       obs_fill, obs_sepc);
    }

    if (bad != NULL)
    {
        *bad = bad_fill + bad_copy + bad_sepc + bad_src + bad_shift;
    }

    return (bad_fill == 0u) && (bad_copy == 0u) && (bad_sepc == 0u) &&
           (bad_src == 0u) && (bad_shift == 0u);
}

/**
 * @brief Put the background colour into the scale probe's destination.
 *
 * This is the "nothing has been drawn here" state, and it is deliberately the
 * same value the blit uses as its background fill.  Two reasons:
 *
 *   - nsgpu_colorkey() fills a region with that colour before it draws, so
 *     anything it covers would read as background whether or not this ran.
 *     Making the rest read the same way means one comparison - "not the
 *     background" - is the whole of "the library painted here", with no second
 *     value to reason about.
 *   - The region it fills is NOT the destination rectangle.  The disassembly
 *     shows it filling (width-1, height-1) - the SOURCE extent - starting at
 *     (dst_startx, dst_starty), which for an 800x480 source into a 320x320
 *     destination is far larger than the framebuffer and clipped to it.  So
 *     the fill lands in the bottom-right quadrant, not over the image, and a
 *     poisoned-but-unfilled pixel and a filled one must not be told apart.
 */
static void scale_clear_dst(void)
{
    volatile uint16_t* p = (volatile uint16_t*)GPU_SCALE_DST;
    uint32_t n = GPU_SCALE_W * GPU_SCALE_H;
    uint32_t i;

    for (i = 0u; i < n; i++)
    {
        p[i] = (uint16_t)GPU_SCALE_BG;
    }
}

/* The driver's ambient framebuffer state, one line, read-only.
 *
 * Why this exists: the GPU front end was gated green through three readings and
 * the product still detected nothing, and the first two explanations both died
 * under evidence.  What survives as a suspect is that the library's draws take
 * something from the outside that nothing in this firmware gates:
 *
 *   - g_framebuffers[framebuffer_acitve_index].  Both convolve wrappers read
 *     that entry and write its m_format/m_bpp out as BOTH the framebuffer format
 *     (what the driver writes) and the texture format (what it samples).  Our
 *     CPU side assumes RGB565 when it unpacks and never checks.  And the
 *     nsgpu_get/set_framebuffer() pairs this firmware does use restore the
 *     driver's ADDRESS only - not the index, not the entry.
 *   - Whatever the previous draw left.  nsgpu_convolve() does not set the
 *     texture filter at all (the call is commented out in the vendor source),
 *     so that one is inherited outright.
 *
 * So: read the driver's live view after each boot probe.  The first tag at which
 * it differs from the known-good early one is the probe that moved it.
 *
 * Deliberately NOT reading g_framebuffers itself.  It does link (only
 * framebuffer_acitve_index and framebuffer_ready_num are static in nsgpu.o, so
 * they cannot be), but the library's entry is its own private struct: the
 * disassembly copies 0x28 bytes, while sizeof(dhd_buffer_t) here is 52.  Reading
 * it through our type would print three fields of confident garbage per entry,
 * which is worse than not reading it.
 *
 * What makes this worth a line: after a draw, the format the driver reports is
 * the format that draw WROTE - i.e. exactly what unpack_rgb565_to_rgb888() has
 * to be right about, and the one thing the whole gate chain never looked at.
 *
 * Read-only: draws nothing, sets nothing.  Boot only. */
void gpu_fb_state(const char* tag)
{
    const dhd_handle_t dev = (dhd_handle_t)a_handle;
    uint32_t w = 0u, h = 0u, p = 0u;
    uint32_t x0 = 0u, y0 = 0u, x1 = 0u, y1 = 0u, ys = 0u;

    if (!s_ready) { return; }

    dhd_get_framebuffer_size(dev, &w, &h, &p);
    dhd_get_bbox(dev, &x0, &y0, &x1, &y1, &ys);

    /* Two lines, because Message is 64 bytes and the address needs 9 of them.
       The BBOX is on the second one and it is not decoration: the geometry half
       of the leak is size AND box, the first log of this probe showed only the
       size moving, and a box left at 0..63 is the half that would survive a
       size restore and keep a draw inside a corner. Printing both is what makes
       the next log able to say which of the two the front end cares about. */
    sprintf(Message, "fb %s %04X %lux%lu p%lu b%lu\r\n", tag,
            (unsigned)dhd_get_framebuffer_format(dev),
            (unsigned long)w, (unsigned long)h, (unsigned long)p,
            (unsigned long)s_pixel_bytes);
    UART_SendString(Message);

    sprintf(Message, "fb %s bb %lu %lu %lu %lu %lu a%08lX\r\n", tag,
            (unsigned long)x0, (unsigned long)y0, (unsigned long)x1,
            (unsigned long)y1, (unsigned long)ys,
            (unsigned long)dhd_get_framebuffer_address(dev));
    UART_SendString(Message);
}

/* Put the filter scale/bias word back to the identity the library's blit
 * expects, on EVERY texture unit.
 *
 * This is the one ambient input the driver offers no getter for, and the one
 * the library never sets.  The other candidate, the texture kernel, is not:
 * the library's blit calls dhd_set_texture_filter() on every draw, and that
 * function rewrites the kernel geometry as a side effect (1x1 QUAD,
 * davehd_driver_txp_inl.h:287-289) - so the blit resets it - while the blit's
 * texture-org word carries no CLUT-kernel bit at all (measured tf8006 at every
 * healthy tag, not the tf8086 the kernel path uses).  It is inert.
 *
 * What is left is this word, and the asymmetry is sharp: the blit runs with the
 * bilinear filter ON (f11 at every tag, measured) and never sets scale/bias, so
 * it inherits whatever the last draw left - while every raw draw we own sets it
 * for its own kernel and puts nothing back.  dw_pass() passes its caller's pair
 * straight through, and gpu_dw_probe()'s last draw is
 * dw_pass(..., 1.0f, 128, ...).
 *
 * (1.0, 0) rather than a saved value, because the driver has no
 * dhd_get_texture_scalebias() and the shadow-state read that would give the
 * previous value needs davehd_internal.h, which is not in gpu.c's include path.
 * It is the right constant rather than a guess: dw_palette_run() already sets
 * exactly this at gpu.c:3355, and every blit in the boot that runs before the
 * first raw draw reads back correct, so this is the value the front end expects.
 *
 * Every unit, because dhd_set_texture_scalebias() writes only the active one and
 * which one a caller left active is exactly the kind of thing this is here to
 * stop depending on.  Ends on unit 0, the unit the blit uses. */
static void gpu_sb_reset(void)
{
    const dhd_handle_t dev = (dhd_handle_t)a_handle;
    const uint32_t n = (uint32_t)dhd_get_texture_unit_count(dev);
    uint32_t i;

    for (i = 0u; i < n; i++) {
        dhd_active_texture_unit(dev, i);
        dhd_set_texture_scalebias(dev, 1.0f, 0);
    }
    dhd_active_texture_unit(dev, 0u);
}

/* --- the front end's way out, and it used to be the library's ----------------

   Both front-end entry points below - gpu_scale_frame() and
   filter_frame_with() - end by pointing the driver's framebuffer back where
   they found it, and the library's spelling of that, nsgpu_set_framebuffer(),
   is dhd_set_framebuffer_address() followed by nsgpu_cmd_sync(0) = dhd_finish():
   the UNBOUNDED wait whose prebuilt body is why gpu_drain() exists at all (see
   the block above its definition).  The depthwise and pointwise tails were both
   converted to that bounded drain.  These two were not - and that was the ONE
   PLACE the hard rule ("GPU 挂掉绝不拖垮主功能") did not hold, measured on the
   board on 2026-09-24:

     `g` on, three triggers, the third stalls at its 18th depthwise layer and
     latches `gpu dead dw1 cv1` - the product carries on and reads a plate - and
     then the FOURTH trigger prints `rx 78 'x' drop 0` and nothing else, ever.
     `k` prints no echo either, and the echo comes before the command dispatcher
     is even reached (main.c:992), so the main loop never turned again: the CPU
     was inside this path, waiting on a sync id that a stopped GPU will never
     write.  Which of its two waits held it is not recoverable from that capture
     (the echo and the `grab ... us` line bracket both), and it does not have to
     be - both are fixed here.  Only a power cycle ends it.

   And nothing warned, because the stall reporter is keyed on the depthwise
   progress word, which only a depthwise draw sets: n32h7xx_it.c's STALL line and
   its eight-second reset both stay silent through a front-end hang, which is why
   that capture has no STALL in it at all.  gpu_front_prog() below is the other
   half of this fix.

   `false` means the drain gave up.  The caller must return the refusal and NOT
   retry - the same contract blit_tile() states - because camera_grab_rgb888()
   reads a false as "use the CPU path", which is the whole point. */
static bool gpu_fb_put(dhd_handle_t dev, uint32_t fb);

/* The front end needs the drain DIRECTLY and not only through gpu_fb_put(),
   because the library commits TWICE on that path and stage 38 has to own both
   points.  Same contract, same latches - see gpu_drain() for the give-up. */
static bool gpu_drain(dhd_handle_t dev);

/* Stage 3's draw, which lives far below this point with the rest of the blit
   templated.  Declared here for the same reason as the two above: its caller
   (filter_frame_with()) is emitted before it. */
static bool blit_separable_convolve_new(dhd_handle_t dev, uint32_t mode,
                                        uint32_t src, uint32_t dst,
                                        uint32_t tw, uint32_t th,
                                        uint32_t dw, uint32_t dh,
                                        uint32_t dst_stride,
                                        uint32_t tmp, uint32_t clut_base,
                                        const int8_t* k, uint32_t n, uint32_t nu,
                                        const float* scale,
                                        const int32_t* bias,
                                        const float* scale1,
                                        const int32_t* bias1,
                                        uint32_t co_first, uint32_t co_count,
                                        float u0, float v0,
                                        uint32_t sh, uint32_t sw,
                                        dhd_texel_org_t texfmt,
                                        dhd_pixel_org_t pixfmt,
                                        dhd_address_mode_t amode,
                                        uint32_t a_border);

/* Arm the DVP interrupt's stall reporter across the WHOLE front-end draw, and
   disarm it with 0 on every way out.

   "Whole draw" and not "just the blocking tail": since 2026-10-09 both entry
   points own their commit points (stage 38 transcribed nsgpu_colorkey, stage 39
   transcribed nsgpu_separable_convolve), so what this arm has to cover is the
   DRAW - still library code, `blit_separable_convolve` and `nsgpu_draw_rotblit`,
   each carrying its own internal `dhd_wait` - plus our own bounded gpu_drain().
   The arm therefore still has to be before the draw call, not before our own
   gpu_fb_put().  (Before the transcriptions the reason was stronger and was
   map-verified: `nsgpu.o(i.nsgpu_colorkey)` and `nsgpu.o(i.nsgpu_separable_convolve)`
   each referred to `nsgpu.o(i.nsgpu_cmd_sync)` = dhd_finish.  Neither reference
   is in the product's path now.)

   Stage 38 is gpu_scale_frame(), 39 is filter_frame_with() - see the stage table
   above DW_PROG for why they are not 40/41 (taken by the conv path) and not
   45/46 (the encoding overflows above 42); the reporter in n32h7xx_it.c decodes
   the word as `STALL s38 c0 p0`, so a front-end hang now names itself and, eight
   seconds later, resets the board instead of hanging it forever.  Without this
   the word is 0 for the whole grab - nothing else in the path writes it - and
   the reporter deliberately ignores 0.

   A stale non-zero value here would look exactly like a stall during the next
   second of unrelated CPU work, so this is a pair of calls with nothing between
   them that can return early.  See gpu_dw_conv()'s tail for the same warning. */
static void gpu_front_prog(uint32_t stage);

/* Every piece of ambient draw state the driver will hand back.  Line per group
 * because Message is 64 bytes and these fields are wide.
 *
 * The counterpart of gpu_fb_state(), and it exists because of what that one
 * measured: the framebuffer half of the state is now byte-identical after
 * gpu_dw_probe() and after gpu_dhd_probe() (format, size, pitch, box, address),
 * and the blit is STILL broken after gpu_dw_probe().  So the leak that breaks
 * the front end is not the framebuffer geometry, and this prints everything
 * else: the texture unit, its filter and its CLUT base, the colour unit, the
 * operands, the blending, the colour key.
 *
 * Read it as a differential: the first tag at which a field differs from the
 * known-good early one names the probe that moved it.  One ambient input is
 * deliberately not here because the driver gives no way to read it back - the
 * filter scale/bias word, which is why gpu_sb_reset() exists to put it back
 * instead of this function printing it.  So if NOTHING here differs anywhere in
 * the boot, the ambient candidates are cleared: the blit resets the kernel
 * geometry itself, its texture-org word never enables CLUT-kernel mode, and the
 * scale word is put back at every exit.  What would be left is the library's own
 * state, which this function deliberately does not touch.
 *
 * `c4` WAS here and is gone, on purpose.  It asked for a four-entry CLUT slice
 * and gave it straight back, printing the base, to fingerprint the allocator: a
 * full CLUT must make it fail, a freed one must make it return base 0.  It
 * answered that (4294967295 at dwA and dwZ while gpu_dw_probe held all 512, 0 at
 * dwF and at every tag before it, so the allocator round-trips cleanly and is
 * not a suspect) - and doing so cost eleven alloc calls per boot on a driver
 * whose CLUT bookkeeping was the thing in doubt.  **A probe that mutates the
 * subsystem it is measuring is not free**, and once its question is answered it
 * is strictly a risk with no remaining reading.  Removed 2026-09-18 when a boot
 * began stopping inside gpu_dw_probe, right after this function's dwA call.
 *
 * Boot only, and now every line is read-only. */
void gpu_env_state(const char* tag)
{
    const dhd_handle_t dev = (dhd_handle_t)a_handle;
    dhd_bool_t   fu = 0, fv = 0, key_en = 0, dis_a = 0;
    dhd_uint8_t  tbo = 0u;
    dhd_uint32_t key = 0u;
    dhd_uint32_t tw = 0u, th = 0u, tp = 0u;
    dhd_texel_org_t tfmt;
    dhd_address_mode_t am = E_DHD_ACCESS_LINEAR;
    dhd_channel_select_t chs = 0;
    dhd_col_modify_t     cmod = 0;
    dhd_col_input_t      o1a;
    dhd_blend_factor_t   bsrc = 0, bdst = 0;
    dhd_blend_mode_t     bfc = 0, bfa = 0;

    if (!s_ready) { return; }

    key = dhd_get_texture_color_key(dev, &key_en);
    dhd_get_texture_filter(dev, &fu, &fv);
    dhd_get_texture_size(dev, &tw, &th, &tp);
    tfmt = dhd_get_texture_format(dev, &am, &dis_a);
    o1a = dhd_get_color_op1a(dev, &chs, &cmod);
    dhd_get_color_blending(dev, &bsrc, &bdst);
    dhd_get_blend_func(dev, &bfc, &bfa);

    sprintf(Message, "env %s tu%lu n%lu f%d%d cb%lu bc%08lX\r\n", tag,
            (unsigned long)dhd_get_active_texture_unit(dev),
            (unsigned long)dhd_get_texture_unit_enable(dev),
            (int)fu, (int)fv,
            (unsigned long)dhd_get_texture_clut_base(dev),
            (unsigned long)dhd_get_texture_border_color(dev));
    UART_SendString(Message);

    sprintf(Message, "env %s ta%08lX ts%lux%lu p%lu tf%X cu%lu n%lu\r\n", tag,
            (unsigned long)dhd_get_texture_address(dev, &tbo),
            (unsigned long)tw, (unsigned long)th, (unsigned long)tp,
            (unsigned)tfmt,
            (unsigned long)dhd_get_active_color_unit(dev),
            (unsigned long)dhd_get_color_unit_enable(dev));
    UART_SendString(Message);

    sprintf(Message, "env %s o1a%u bs%u.%u bf%u.%u ck%08lX/%lu\r\n", tag,
            (unsigned)o1a, (unsigned)bsrc, (unsigned)bdst,
            (unsigned)bfc, (unsigned)bfa,
            (unsigned long)key, (unsigned long)key_en);
    UART_SendString(Message);
}

/* ================= THE PLANE PROBE =========================================

   WHY IT EXISTS.  The chain fingerprint in yolo_inference.cpp says the first
   eligible depthwise layer's INPUT is byte-identical across runs while its
   OUTPUT is not, and `gpu dw gpudiff` says a back-to-back redraw of the first
   eligible layer of each stride class is byte-identical (`worst 0 bad 0`).
   Both cannot be true of a draw that is a pure function of its operands: if
   the operands were the same and the draw were reproducible, the output would
   be the same.  So the difference between two runs is in the STATE the draw
   reads or in one of the two scratch planes it sits between - and gpudiff,
   which redraws with the CPU working in between, is structurally blind to
   both.  This is the differential that names which.

   WHAT IT PRINTS, once, for the first draw after it is armed - one line and
   gpu_env_state(<tag>)'s three, and both of them AFTER the draw, so that no
   UART time of the probe's own sits inside the window it is measuring:

     gpu dw pl <tag> s<src> d0<pre> d1<post>

   `s`  the SOURCE plane as the pack just left it - the draw's operand, read
        straight from the CPU before the draw.  Two runs whose `s` agree had
        the same operand and the same pack, so nothing upstream of the plane
        moved.
   `d0` the DESTINATION plane as it stood BEFORE the draw.  Nothing clears it
        between draws - the pre-fill that used to run was removed, see the
        comment above the channel loop in gpu_dw_conv() - so a `d0` that
        differs is a different leftover, and the unpack folds that leftover
        into the output wherever the draw does not cover the plane.
   `d1` the DESTINATION plane after dhd_wait(WRITEBACK), i.e. exactly what the
        CPU is about to unpack.  `s` and `d0` equal but `d1` different is the
        one reading gpudiff cannot produce: the draw is not a function of its
        operands and its state.
   the state lines are every readable word of the driver's texture, colour and
        framebuffer state as it applies to THIS draw.  One field differing
        between runs names the leak, and it is the whole reason to call the
        existing dumper rather than a new one.

   WHY A SPREAD SAMPLE.  FNV-1a over every 4096/n-th byte, not the head: a
   divergence confined to a later row of the plane - which is what a clip
   leaves - is not in the first bytes, and the destination is read at a row
   pitch the extent arguments may not cover.

   WHAT IT COSTS.  Three sampled hashes of the two scratch planes plus one
   ambient state read, once per inference, and only while armed: the caller arms
   it for the first two eligible layers, which are the two layers gpudiff tested
   and therefore the two whose disagreement with it means something.  It draws
   nothing, allocates nothing and touches no depthwise arithmetic, and it writes
   nothing to the UART until the draw is over.  What it still puts in the window
   before dhd_draw() is two reads of the scratch planes - SDRAM traffic of the
   class the pack and the unpack already make - so a fault that is purely a
   timing race could move rather than vanish: a clean run is inconclusive, not a
   cure. */

/* HOW MANY DRAWS ONE ARM COVERS, and it was 1 until now: the arm was a flag
   consumed by the next draw, so the probe has only ever watched the FIRST pass
   of a layer.  A layer is 2*C draws - pass 1 and the split's pass 2, per
   channel - and one reading of one of them cannot say whether the first draw of
   a layer is special or whether every draw of the live path is.  Two is the
   smallest number that separates those: the second draw is the same layer, the
   same destination and the same source plane, microseconds later.

   AND THE SECOND DRAW'S PRE-READ IS THE "HOW LATE" MEASUREMENT, for free.  If
   draw 1's output is in the plane by the time draw 2 asks for its d0, the write
   landed inside one drain-plus-hash; if draw 2's d0 is still draw 1's d0, it did
   not land within that, and the spin's own bound is then the only lag left to
   widen.  No extra spin is bought by doing this. */
#define GPU_DW_PL_PROBES 2

static int      s_pl_on   = 0;   /* draws this arm still covers; 0 = disarmed */
static int      s_pl_post = 0;   /* the draw happened, waiting for its wait */
static int      s_pl2     = 0;   /* ... and still waiting for the DRAIN */
static int      s_pl_n    = 0;   /* how many ARMS have been reported this boot */
static uint32_t s_pl_s    = 0u;  /* held between the two hooks */
static uint32_t s_pl_d0   = 0u;
static char     s_pl_tag[8] = { 0 };

/* WHAT THE LAST DRAIN ACTUALLY DID, printed beside the plane it is being asked
   about.  `i` is the poll index the loop left at - 0 means the SYNC_ID the drain
   sampled had ALREADY moved when its first read ran, i.e. the loop never waited
   for anything - `was` is the value it sampled before the commit and `sy` the
   value it stopped on.  Filled in gpu_drain()'s success path only;
   gpu_dw_plane_post2() runs immediately after that drain with nothing else in
   between, so the three belong to this draw's own drain. */
static uint32_t s_drn_last_i   = 0u;
static uint32_t s_drn_last_was = 0u;
static uint32_t s_drn_last_sy  = 0u;

void gpu_dw_plane_arm(void)
{
    s_pl_on = GPU_DW_PL_PROBES;
}

static uint32_t gpu_dw_plane_hash(uint32_t base, uint32_t n)
{
    uint32_t h = 2166136261u;
    uint32_t i, step;

    if (n == 0u) { return 0u; }

    step = (n > 4096u) ? (n / 4096u) : 1u;
    for (i = 0u; i < n; i += step) {
        h = (h ^ (uint32_t)(*(volatile uint8_t*)(base + i))) * 16777619u;
    }
    return h;
}

/* NO PRINT HERE.  Both hashes are taken at the only instant they mean anything
   and the line is printed by the post hook, because a UART write inside the
   window between dhd_set_framebuffer_address() and dhd_draw() is several
   milliseconds of the very thing this probe exists to look for - a timing race
   would see the print as the change rather than the fault.  Reading the planes
   is SDRAM traffic of the same class the pack and the unpack already make. */
static void gpu_dw_plane_pre(uint32_t src, uint32_t src_n, uint32_t dst,
                             uint32_t dst_n)
{
    if (s_pl_on <= 0) { return; }
    s_pl_on--;
    s_pl_post = 1;
    s_pl2     = 1;

    /* `D<n>_<k>`: n is the ARM (so D0_* and D1_* are the two layers the arm
       covers) and k is the DRAW inside it.  D7_1 and D7_2 are therefore one
       layer's two draws and D7_* and D8_* are two different layers. */
    sprintf(s_pl_tag, "D%d_%d", s_pl_n, GPU_DW_PL_PROBES - s_pl_on);
    s_pl_n++;

    s_pl_s  = gpu_dw_plane_hash(src, src_n);
    s_pl_d0 = gpu_dw_plane_hash(dst, dst_n);
}

/* Everything the probe prints, at the one moment the draw is over and nothing
   has been restored yet.  The ambient state is read here rather than before the
   draw for the reason above, and it is the same state either way: gpu_env_state()
   reads the driver's shadow, and nothing between dhd_draw() and this point
   writes it. */
static void gpu_dw_plane_post(uint32_t dst, uint32_t dst_n)
{
    if (s_pl_post == 0) { return; }
    s_pl_post = 0;

    sprintf(Message, "gpu dw pl %s s%08X d0%08X d1%08X\r\n", s_pl_tag,
            (unsigned)s_pl_s, (unsigned)s_pl_d0,
            (unsigned)gpu_dw_plane_hash(dst, dst_n));
    UART_SendString(Message);

    gpu_env_state(s_pl_tag);
}

/* ================= THE THIRD HASH, AND IT IS THE ONE THAT MATTERS ==========
   d1 above is taken straight after dhd_wait(E_DHD_WAIT_WRITEBACK).  The first
   capture that carried this probe (Code=212106) read d0 and d1 EQUAL for the
   two layers whose class check draws them, and UNEQUAL for the four real draws
   that followed - which read like "a redraw is idempotent and a real draw is
   not", and was wrong.

   The reading that settles it is cross-boot, and it is in the D0 line of both
   binaries:

       Code=212106  gpu dw pl D0 s15D532CF d02DEC479D d12DEC479D
       Code=212138  gpu dw pl D0 s1BCBEEAA d02DEC479D d12DEC479D

   Two different images, two different PACKED SOURCES (`s` moves), and the same
   d0 AND d1 to the byte.  A destination that matches across two builds while
   the source does not is not the draw's output - it is the content the draw
   had not yet overwritten.  So at the moment of the post hook the write is not
   visible to the CPU, and d1 == d0 says nothing about whether the draw works.

   AND THE DRAW DOES WORK: the same capture passes `gpu dw chk s2 ... worst 6`
   and `gpu dw chk s1 ... worst 4`, i.e. the CPU's own unpack of this plane is
   correct to within the gate's tolerance.  The unpack happens later than d1 -
   after the framebuffer restore and after gpu_drain() - so what separates a
   correct read from a stale one is exactly that stretch.

   SO THIS HOOK SITS AT THE OTHER END OF IT.  Same plane, same hash, taken after
   gpu_drain() returned true, which is the last statement before the caller
   unpacks it.  The two readings divide the remaining explanations:

     d2 != d0  the write lands somewhere between dhd_wait() and the drain's
               completion.  The wait is EARLY and gpu_drain() is what actually
               makes the CPU's read safe - so every unpack is one scheduling
               accident away from reading a plane that is still being written.
     d2 == d0  the draw never wrote the sampled region at all.  Then this is
               geometry or clipping and not timing, and the next candidate is
               the utility cliprect - the depthwise path still never sets it
               while the active clip mode is E_DHD_CLIP_UTILRECT.

   The one-shot latch is cleared here rather than left for the next draw, and
   `gpu_dw_plane_abort()` clears it on the refused path, so a layer that gives
   up cannot print a d2 belonging to the draw before it.

   WHAT THE FIRST pl2 CAPTURE (Code=212258) READ, and it came back BOTH ways:

       D0 d02DEC479D d12DEC479D d2213D29A3      <- inference A, changes
       D1 d0BCB08A2C d1BCB08A2C d2BD44F357
       D2 d08B0EA0E9 d18B0EA0E9 d28B0EA0E9      <- inference B, never changes
       D3 d0BCB08A2C d1BCB08A2C d2BCB08A2C
       D4 d0CF4D8D88 d1CF4D8D88 d2CF4D8D88      <- inference C, never changes
       D5 d0BCB08A2C d1BCB08A2C d2BCB08A2C

   Four equal, two not - and the two that changed are the two whose class check
   draws the same layer in the same inference.

   d2 == d0 CANNOT BE THE DRAW REPRODUCING A PLANE THAT ALREADY HELD ITS OWN
   OUTPUT, which is the only benign reading: D2 and D4 are the same layer on the
   same frozen frame with the same packed source (`sFADE7FFA` both times), so
   their outputs must be identical - and their d2 differ.  So on those four
   draws the destination really had not been written when the CPU looked.

   The two explanations left are the two the block above already names, and one
   bounded spin separates them:

     the write is merely LATE   re-hashing the same plane must eventually stop
                                returning the pre-draw value, and the ITERATION
                                it stops at is the lag - which is the number a
                                drain would have to wait out.
     the draw never wrote it    the plane reads the pre-draw value for the
                                whole spin, and this is geometry or clipping.

   `n` is dst_n (dw*dh, so a tiny or zero region cannot hide behind a -1) and
   `s` is that iteration, or -1.  One hash per iteration is the same class of
   SDRAM traffic the pack and the unpack already make, and the loop is bounded,
   so a layer that never lands costs the bound and not a hang. */
#define GPU_DW_PL2_SPIN 4000u

static void gpu_dw_plane_post2(uint32_t dst, uint32_t dst_n)
{
    int32_t  hit = -1;
    uint32_t k;

    if (s_pl2 == 0) { return; }
    s_pl2 = 0;

    for (k = 0u; k < GPU_DW_PL2_SPIN; k++)
    {
        if (gpu_dw_plane_hash(dst, dst_n) != s_pl_d0) { hit = (int32_t)k; break; }
    }

    /* `s` is the iteration the plane stopped reading its pre-draw value at, or
       -1 for the whole bound.  `i`/`w>sy` are the drain that was supposed to
       have made the write visible: the poll index it left the loop at, and the
       SYNC_ID it sampled before committing vs the one it stopped on.  `i 0` with
       `w != sy` is a drain that never waited - it saw an increment that was
       already in flight and returned while its own draws were still queued. */
    sprintf(Message, "gpu dw pl2 %s d2%08X n%u s%d i%u w%u>%u\r\n", s_pl_tag,
            (unsigned)gpu_dw_plane_hash(dst, dst_n), (unsigned)dst_n, (int)hit,
            (unsigned)s_drn_last_i, (unsigned)s_drn_last_was,
            (unsigned)s_drn_last_sy);
    UART_SendString(Message);
}

/* The refused path: nothing was unpacked and nothing will be, so the third
   reading has no meaning and must not survive into the next draw. */
static void gpu_dw_plane_abort(void)
{
    s_pl2 = 0;
}

/* nsgpu_vgdraw_rect()'s four-axis quantisation, both conversions written out.

   VCVT.S32.F32 SATURATES at the 32-bit ends and C's float-to-int cast is
   undefined there, so the ends are spelled; LSLS #4 is a plain 32-bit shift and
   therefore WRAPS.

   The wrap is load-bearing, not a curiosity, and it is the thing to get right
   here.  nsgpu_data_fill() converts dst_startx with VCVT.F32.U32, so the
   negated centring offset the front end passes (-160 for a 640-wide source)
   enters the blitter as 4.29e9, saturates to INT32_MAX and comes back as -16 on
   the shift - i.e. -1 px.  That is why the pre-fill covers the WHOLE destination
   in the shipped configuration rather than missing it.  The same arithmetic is
   what painted [160, 319] when dst_startx was still +160, which is the
   `box 160 160 319 319` line in gpu.h's probe table; reading the wrap as a
   curiosity gets the fill wrong in opposite directions depending on the sign of
   the offset, and the difference is a magenta frame around a correct image
   versus a magenta image. */
static int32_t gpu_vg_axis(float v)
{
    int32_t i;

    if (!(v < 2147483648.0f))      { i = INT32_MAX; }   /* also catches NaN */
    else if (v < -2147483648.0f)   { i = INT32_MIN; }
    else                           { i = (int32_t)v; }

    return (int32_t)((uint32_t)i << 4);                 /* LSLS: wraps */
}

/* nsgpu_vgdraw_rect()'s body, minus its nsgpu_cmd_sync - the one draw on the
   front end that is not a library entry point, because that entry point exists
   only to commit and it commits UNCONDITIONALLY: nsgpu_lib_dis.txt:41528 is
   AFTER the CBZ at 41508, so a rect that clips away entirely still flushes.

   x/y/width/height arrive as FLOATS because nsgpu_data_fill() did the
   VCVT.F32.U32 itself (30054-30058) and handed the blitter floats - the unsigned
   reading of a negative dst_startx has already happened by the time this
   function sees it, which is the subject of gpu_vg_axis() above. */
static void gpu_vg_rect(dhd_handle_t dev, float x, float y, float width,
                        float height)
{
    const float qx = (float)gpu_vg_axis(x);
    const float qy = (float)gpu_vg_axis(y);
    const float qw = (float)gpu_vg_axis(width);
    const float qh = (float)gpu_vg_axis(height);

    /* dhd_bound_aab() snaps the box to the integer grid and dhd_clip_bbox()
       merges it with the framebuffer.  Both are DHD_API_INLINE from
       davehd_util.h, so what runs is the vendor's own code; only the scratch
       struct is ours rather than the library's g_prim.

       THIS DRAW NAMES ITS CLIP MODE INSTEAD OF READING ONE, AND THAT IS THE
       POINT.  nsgpu_vgdraw_rect(), whose body this is, passes the library's own
       gpu_clip_mode; that word is UNREACHABLE at link time (it is `Lc` static in
       nsgpu.o - the first attempt to extern it failed with L6218E), it has no
       writer in this image (nsgpu_set_clip_mode() is `Removing`, map:19092, and
       nsgpu_init() does not refer to it either), and it is NOT
       E_DHD_CLIP_FRAMEBUFFER: 0x30000028 holds it (fromelf -s) and the board's
       `gpu dst3 stuck 102399 fill 0 other 1 first 0,0 last 0,0` shows the
       library's own blit being clipped to the utility rect.  Passing
       E_DHD_CLIP_FRAMEBUFFER here explicitly is what keeps this fill's coverage
       independent of a word this file cannot read or write, and its job demands
       it: the pre-fill must reach the whole destination.

       (An earlier version of this note read the initialiser out of the linked
       image at `.data+0x18` and called it 1 = E_DHD_CLIP_FRAMEBUFFER.  0x30000018
       is a_handle's slot; the mode is at 0x30000028.  See the util-cliprect note
       in gpu_scale_frame for what replaced that reading.) */
    {
        dhd_prim_data_t prim;

        dhd_bound_aab(&prim, (dhd_fx28_4_t)qx, (dhd_fx28_4_t)qy,
                      (dhd_fx28_4_t)(qx + qw), (dhd_fx28_4_t)(qy + qh));

        if (dhd_clip_bbox(dev, &prim, E_DHD_CLIP_FRAMEBUFFER))
        {
            dhd_set_bbox(dev, prim.m_xmin, prim.m_ymin, prim.m_xmax,
                         prim.m_ymax, prim.m_ystart);
            dhd_draw(dev, 0u, 0u, (dhd_drawattribute_flag_t)0u);
        }
    }
}

bool gpu_scale_frame(uint32_t src, uint32_t src_w, uint32_t src_h,
                     uint32_t pitch)
{
    uint32_t scale_x;
    uint32_t scale_y;
    uint32_t nat_w;
    uint32_t nat_h;
    uint32_t off_x = 0u;
    uint32_t off_y = 0u;
    uint32_t saved_fb;
    uint32_t dead_dw = 0u;
    uint32_t dead_cv = 0u;
    bool     put;

    if (!s_ready || (s_pixel_bytes != 2u) || (src_w == 0u) || (src_h == 0u) ||
        (pitch < src_w))
    {
        return false;
    }

    /* The latched GPU is a CPU-only GPU, and this call has to agree with that
       or the grab is the one stage still talking to a stopped device.  One GPU
       and one latch set (gpu_drain() sets both from a single drain that does not
       finish), so checking it here rather than keeping a third flag is the
       consistent reading - and it is what makes a latch cost ONE bounded wait
       instead of one per frame for the rest of the boot.  `k` re-arms it, and
       the caller's fallback is the CPU downsample, which is exactly the rule. */
    gpu_dead_stat(&dead_dw, &dead_cv);
    if ((dead_dw != 0u) || (dead_cv != 0u))
    {
        return false;
    }

    /* The one ambient input this call cannot set for itself.  The blit sets its
       own texture, size, format, filter, framebuffer and bbox, so the driver's
       leftovers in all of those are harmless - but the filter scale/bias word
       it only READS, and the driver has no getter to read it back with.  The
       last depthwise layer of the previous inference leaves its own per-unit
       gain there (that is the 2026-09-18 front-end break: gpu_dw_conv() writes
       it 19 times per inference and nothing put it back).

       Forcing it here rather than at the end of the last inference is what
       makes this call correct on its own terms - the first grab after a boot,
       with no inference behind it, is then in the same state as the thousandth,
       and the front end stops depending on which boot probe happened to run.
       See gpu_sb_reset() for why 1.0f/0 is the value. */
    gpu_sb_reset();

    /* ceil, not floor - see the note on gpu_scale_frame in gpu.h. */
    scale_x = ((GPU_SCALE_W * 256u) + src_w - 1u) / src_w;
    scale_y = ((GPU_SCALE_H * 256u) + src_h - 1u) / src_h;

    /* nsgpu_colorkey() CENTRES the scaled image inside an (src_w x src_h) box
       anchored at (dst_startx, dst_starty) - it does not put the image's
       top-left there.  The offset is ceil((src_extent - natural) / 2) per axis,
       so pass it back negated and the image lands at the origin.  See the note
       on gpu_scale_frame in gpu.h for the measurements that pin this down; at
       scale 256 the offset is 0 and none of this matters, which is why it went
       unnoticed until now. */
    nat_w = (src_w * scale_x) >> 8;
    nat_h = (src_h * scale_y) >> 8;

    if (nat_w < src_w) { off_x = (src_w - nat_w + 1u) / 2u; }
    if (nat_h < src_h) { off_y = (src_h - nat_h + 1u) / 2u; }

    saved_fb = nsgpu_get_framebuffer();

    /* ================== THE LIBRARY'S BODY, COMMIT-FREE ==================
       From here to gpu_fb_put() this is nsgpu_colorkey() transcribed call for
       call, with ONE change: the commit.  All seven nsgpu_cmd_sync()s the
       library's path contains - colorkey's own two (29446, 29645) plus the five
       nested inside nsgpu_data_fill(), nsgpu_set_draw_color(),
       nsgpu_set_draw_alpha() and nsgpu_vgdraw_rect() - are gone, and the single
       BOUNDED gpu_drain() inside gpu_fb_put() is what completes the work.  No
       byte of the result changes by that: every call in between writes a
       register or packs a command into the ring, and the only two readbacks on
       the path (dhd_get_color, dhd_get_framebuffer_format) read the driver's
       own SHADOW state, not the hardware - davehd_driver_col_inl.h:73-83 and
       davehd_driver_blu_inl.h:385-396 - so none of them needs the hardware to
       have finished.  The two draws are ordered against each other by the ring
       itself.

       WHY, in one paragraph, because the alternative is tempting every time
       someone reads this file: nsgpu_colorkey()'s last statement is
       nsgpu_cmd_sync = dhd_flush + dhd_finish, and dhd_finish is an UNBOUNDED
       wait.  On 2026-09-28 the shipped configuration exercised it: eight clean
       frames, then stage 38 held for eight `STALL s38 c0 p0` and a
       `GPU WEDGE 8s -> reset`, with the main loop never coming out of the call
       ([[gpu-frontend-unbounded-wait]], [[gpu-hang-and-drain-fix]] - the flush
       wrote a start while the driver's belief was not ACTIVE, the STC refused it
       with stop code 2, the hardware cleared its own sync-IRQ enable, and the
       finish waited for a job that was never accepted, forever).

       AND DO NOT PUT gpu_keep_active_now() BACK IN.  It was here, in front of
       the draw, and it is the wrong repair on both counts.  It is unreachable by
       a timing argument - the driver's m_status is flipped by the ISR DURING a
       long draw, so a correction before the draw is too early and one after it
       is already inside dhd_finish - and worse, forcing ACTIVE is not even the
       right correction: it suppresses the start the STC needs in order to resume
       after its own self-stop, so it converts a refused-and-wedged stream into a
       silently halted one that dhd_finish still waits on forever.  The drain in
       gpu_fb_put() applies the same repair, at the commit point we own, which is
       the only place it can be applied.  See gpu_drain().

       WHAT IS DELIBERATELY NOT HERE: the old pair of
       dhd_set_framebuffer_size()/dhd_set_bbox() calls, and the bbox-leak
       diagnosis written for them.  That diagnosis - that the blit inherited a
       64x8 box from a boot probe - is FALSIFIED: nsgpu_draw_rotblit() calls
       dhd_set_bbox() itself (nsgpu_lib_dis.txt:32088), and so does
       nsgpu_vgdraw_rect() (41520) before its own draw below, so every draw on
       this path sets its own box twice over.  It never explained
       `gpu agree mean 158 max 255 swap 158 ok 0` and it is not needed now that
       the library's own sequence is what runs.  The suspect that DOES survive is
       the entry copied below, and it is measured by gpu_fb_state().
       ==================================================================== */
    {
        const dhd_handle_t dev = (dhd_handle_t)a_handle;
        const uint32_t startx = (uint32_t)(-(int32_t)off_x);
        const uint32_t starty = (uint32_t)(-(int32_t)off_y);
        uint32_t fbw[10];                    /* the library's 40-byte entry  */

        /* ---- the destination geometry ----------------------------------
           Done ONCE here although the library does it TWICE: colorkey copies
           g_framebuffers[framebuffer_acitve_index] into a local at 29401-29408
           and sets format/size/address from it, and nsgpu_data_fill() - which
           colorkey calls FIRST - copies the same entry and sets the same four
           values from the same W/H/dst.  One block here is the library's two.

           THE ENTRY IS UNREACHABLE AND DOES NOT NEED TO BE REACHED.  The
           library reads exactly four values out of it, and all four are known
           here without it: the address, width and height are this function's own
           arguments, which colorkey OVERWRITES into its copy anyway, and the
           format is the driver's, which is the same word because nsgpu_init()
           set the entry and the driver from one argument.  Reading the driver
           back is not an approximation of the library's read; it is the same
           value from the one copy that is linkable.  `dhd_set_framebuffer_format`
           keeps only `& 31` plus the limit-cta bit and
           `dhd_get_framebuffer_format` puts `DHD_PIXEL_ORG_FLAG` back, so the
           round trip is exact for both formats this firmware can boot in
           (0x4006 RGB565, 0x400a L_8).  See the block above
           nsgpu_draw_rotblit()'s declaration for why the entry's own copy cannot
           be read at all (`Lc` - file-static - in nsgpu.o).

           The one thing this DOES close is the old suspect, and it closes it by
           removing it rather than by measuring it: the format can no longer
           disagree with the driver's, because it is no longer a separate copy.
           fbw is still built and still passed to the blit, because that is the
           library's call - but nothing reads it, since nsgpu_draw_rotblit()
           clobbers its a_fb in the prologue (nsgpu_lib_dis.txt:31939) before any
           use.  The five offsets are the ones the library's own entry touches,
           in case a later build starts reading them. */
        memset(fbw, 0, sizeof(fbw));
        fbw[0] = (uint32_t)GPU_SCALE_DST;                /* +0x00 address     */
        *(uint16_t*)((uint8_t*)fbw + 0x14) = (uint16_t)GPU_SCALE_W;
        *(uint16_t*)((uint8_t*)fbw + 0x16) = (uint16_t)GPU_SCALE_H;
        *(uint16_t*)((uint8_t*)fbw + 0x18) = (uint16_t)GPU_SCALE_W;
        *(uint16_t*)((uint8_t*)fbw + 0x20) =             /* +0x20 format      */
            (uint16_t)dhd_get_framebuffer_format(dev);

        dhd_set_framebuffer_format(dev,
                                   (dhd_pixel_org_t)dhd_get_framebuffer_format(dev));
        dhd_set_framebuffer_size(dev, GPU_SCALE_W, GPU_SCALE_H, GPU_SCALE_W);
        (void)dhd_set_framebuffer_address(dev, (dhd_gpu_ptr_t)GPU_SCALE_DST);

        /* ---- the pre-fill: nsgpu_data_fill(), also commit-free ---------
           Not decoration and not a no-op: it paints GPU_SCALE_BG over an
           (src_w-1) x (src_h-1) box anchored at (startx, starty), and because
           those are NEGATIVE in the shipped configuration the box covers the
           WHOLE destination rather than missing it - see gpu_vg_axis() for why
           the sign survives a conversion that looks like it should destroy it.
           Dropping it would leave whatever the previous frame left wherever the
           blit's half-pixel edge does not reach.

           The colour comes from nsgpu_set_draw_color()/nsgpu_set_draw_alpha(),
           which are a shadow write of index 0 and a read-modify-write of its
           alpha byte; the four colour/alpha operand calls are data_fill's own
           (30022-30041) and are NOT the same operands colorkey sets for the
           blit below. */
        /* ARMED HERE AND NOT AT THE BLIT, because this function now contains TWO
           waits and either one can be the one that stalls.  The window is the
           front end's draw, and the fill is part of that draw. */
        gpu_front_prog(38u);

        dhd_set_color(dev, 0u, GPU_SCALE_BG);                /* set_draw_color */
        {                                                    /* set_draw_alpha */
            uint32_t c = dhd_get_color(dev, 0u);
            c = (c & 0x00FFFFFFu) | ((uint32_t)255u << 24);
            dhd_set_color(dev, 0u, c);
        }
        dhd_set_color_operation(dev, (dhd_color_op_t)0u);
        dhd_set_color_op1a(dev, (dhd_col_input_t)3u,
                           (dhd_channel_select_t)0x1bu, (dhd_col_modify_t)0u);
        dhd_set_alpha_operation(dev, (dhd_color_op_t)0u);
        dhd_set_alpha_op1a(dev, (dhd_col_input_t)3u, (dhd_channel_t)0u,
                           (dhd_col_modify_t)0u);

        /* THE UTILITY CLIPRECT IS NOT INERT - IT IS WHAT CLIPPED THE BLIT, AND
           THIS LINE IS WHY THE PICTURE WAS ONE PIXEL WIDE.

           nsgpu_draw_rotblit() gets its clip mode from the library's own
           gpu_clip_mode (Printf.map: `nsgpu.o(i.nsgpu_draw_rotblit) refers to
           nsgpu.o(.data) for gpu_clip_mode`; fromelf -s puts it at 0x30000028,
           Lc, so it cannot be reached or written from here), and
           dhd_clip_bbox() intersects the primitive's bbox with the utility rect
           ONLY under E_DHD_CLIP_UTILRECT (davehd_util_inl.h:438-450 for the
           merge, 464-467 for the clamp; the out-of-line copy the call actually
           lands in is davehd_driver_inl.o `.text.dhd_clip_bbox` and reads
           device+0x2a6..0x2ac for those four fields).  Copying the library's
           four zero globals therefore made the blit's clip the single pixel
           (0,0) - measured, `gpu dst3 stuck 102399 fill 0 other 1 first 0,0
           last 0,0`, and the one pixel it did write is the source's texel 0.

           AN EARLIER VERSION OF THIS NOTE CALLED THE RECT INERT, on the strength
           of gpu_clip_mode being 1 = E_DHD_CLIP_FRAMEBUFFER "read out of the
           linked image at .data+0x18".  0x30000018 is a_handle's slot; the mode
           is at 0x30000028.  What settles the mode is the reading above rather
           than any byte read: the pre-fill in this same function hardcodes
           E_DHD_CLIP_FRAMEBUFFER, is called through this same dhd_clip_bbox(),
           and covers the whole 320x320 destination in the same state - so the
           framebuffer is 320x320, nothing between here and the blit resizes it
           (the only dhd_set_framebuffer_size() in this function is above), and
           the util-rect branch is the ONLY one that can turn a 321x320 quad
           into a one-pixel bbox.  Ergo gpu_clip_mode == E_DHD_CLIP_UTILRECT.

           So the rect is given the whole destination instead.  Cost: nothing
           that can stall.  dhd_set_util_cliprect() is four stores into the
           device struct - davehd_util_setup.c:27-28 says the region "is not
           communicated to the HW, it is a pure software feature" - so this adds
           no ring command, no flush and no commit point to stage 38.  Under
           mode 2 the intersection is now the destination; under 1 or 0 the
           value is ignored, which is why this line cannot make a correct draw
           worse. */
        dhd_set_util_cliprect(dev, 0u, 0u, (dhd_uint16_t)(GPU_SCALE_W - 1u),
                              (dhd_uint16_t)(GPU_SCALE_H - 1u));

        /* x and y go in UNSIGNED, exactly as data_fill converts them
           (VCVT.F32.U32, 30054-30058) - the wrap that makes a negative start
           cover the whole destination happens inside gpu_vg_axis(). */
        gpu_vg_rect(dev, (float)startx, (float)starty,
                    (float)(uint32_t)(src_w - 1u),
                    (float)(uint32_t)(src_h - 1u));

        /* ---- THE FILL'S COMMIT: the library's nsgpu_cmd_sync() at 29446 -----
           PUT BACK, AND MADE BOUNDED.  In the library colorkey commits here,
           before it touches the texture unit at all; stage 38 exists to own
           this path's commits, and owning a commit point means owning the WAIT
           rather than deleting it.

           WHAT THIS WAS BLAMED FOR, AND THE MEASUREMENT THAT CLEARED IT.  An
           earlier reading of this change - and the comment this block replaces
           - held that the missing syncs were what made the destination a
           constant: the STC walks the command stream far ahead of the pixels,
           so the texture/colour/blend writes below would land while the fill's
           own pixels were still draining, and the unprocessed remainder would
           be sampled with an enabled texture unit under a colour op that
           includes it.  THAT IS FALSIFIED, and the falsifying reading is
           `Code=200418` (mdk-arm/log, 2026-09-29): with this commit present and
           executed - `gpu dw drn 18 cy1075`, 18 polls, 17.9 us, no stall and
           nothing latched - the destination's first bytes are `s C307C3`, the
           SAME constant as with the syncs deleted, on a boot whose CPU arm
           reads a different picture; and `blit` went 52822 -> 58276 us, i.e.
           the wrong way.  A commit here is therefore neither necessary nor
           sufficient for the constant, and the constant's cause is NOT in this
           function: 29446..29526 was checked call for call against
           nsgpu_lib_dis.txt (arguments, registers and all) and is exact.

           WHY IT STAYS: it reproduces the library's commit structure, and it is
           bounded, so it cannot wedge - gpu_drain() gives up after
           GPU_DRAIN_LIMIT polls, latches both operators, logs its `drain dead`
           line and returns false, the same refusal contract as the tail, where
           the caller falls back to the CPU downsample.  Do NOT re-derive a
           cause for the constant from this block; it has been tested. */
        if (!gpu_drain(dev))
        {
            /* The address write above is in the driver's shadow and, on this
               path, was already flushed by the drain's own dhd_flush() - so the
               shadow has to be put back even though no further commit happens.
               Nothing else draws on the CPU path. */
            (void)dhd_set_framebuffer_address(dev, (dhd_gpu_ptr_t)saved_fb);
            gpu_front_prog(0u);
            return false;
        }

        /* ---- the texture and colour units, in the library's order -------
           Transcribed from 29450-29526.  Two things here are not obvious and
           both are load-bearing:

           * the format handed to dhd_set_texture_format() is the READBACK of
             the framebuffer format just set, not the entry's field read a
             second time.  The library writes what it read, which is what makes
             the destination and the sample agree by construction - and it is
             why the write above had to happen before this read rather than
             being folded into one call.
           * dhd_active_color_unit() comes AFTER dhd_set_texture_format(), not
             beside dhd_active_texture_unit().  That is the library's order, and
             nothing here suggests the order is inert. */
        dhd_active_texture_unit(dev, 0u);
        dhd_set_texture_address(dev, (dhd_gpu_ptr_t)src, 0u);
        {
            /* The library reads the framebuffer size here and DISCARDS all
               three answers (29463-29468).  Kept verbatim - it is the library's
               call, and dropping a read is not obviously free. */
            uint32_t w = 0u;
            uint32_t h = 0u;
            uint32_t p = 0u;
            dhd_get_framebuffer_size(dev, &w, &h, &p);
        }
        dhd_set_texture_size(dev, src_w, src_h, pitch);
        dhd_set_texture_format(dev,
                               (dhd_texel_org_t)dhd_get_framebuffer_format(dev),
                               (dhd_address_mode_t)0u, (dhd_bool_t)0);
        dhd_active_color_unit(dev, 0u);
        dhd_set_color_operation(dev, (dhd_color_op_t)0u);
        dhd_set_color_op1a(dev, (dhd_col_input_t)0x13u,
                           (dhd_channel_select_t)0x1bu, (dhd_col_modify_t)0u);
        dhd_set_alpha_operation(dev, (dhd_color_op_t)0u);
        dhd_set_alpha_op1a(dev, (dhd_col_input_t)3u, (dhd_channel_t)0u,
                           (dhd_col_modify_t)0u);
        dhd_set_color_unit_enable(dev, 1u);
        dhd_set_texture_filter(dev, (dhd_bool_t)1, (dhd_bool_t)1);
        dhd_set_texture_wrap(dev, (dhd_wrap_mode_t)0u, (dhd_wrap_mode_t)0u);
        dhd_set_texture_unit_enable(dev, 1u);
        dhd_util_blend(dev, E_DHD_UTIL_BLEND);

        /* ---- the draw: the library's own rotblit ------------------------
           nsgpu_draw_rotblit() is the library's code, so the drawing is
           identical by construction; what is ours is only the commit, and it
           is now gpu_fb_put()'s bounded drain instead of the library's
           nsgpu_cmd_sync.  The four arguments that are not this function's
           inputs are the library's own arithmetic, transcribed from
           29527-29643:

             angle  = (float)(0 % 360) = 0.0f     (the call passes 0u)
             cx     = (2*startx + cval) << 3
             cy     = (2*starty - sval) << 3
             width  = (src_w * scale_x) >> 4
             height = (src_h * scale_y) >> 4

           cval/sval fall out of a double-precision cos/sin block
           (29527-29612) which at rot == 0 collapses to
           cval = (int)(cos*src_w + sin*src_h) = +src_w and
           sval = (int)(sin*src_w - cos*src_h) = -src_h.

           THE MINUS INSIDE sval IS THE WHOLE POINT, and getting it wrong is
           not a rounding error - it is a 2*src_h displacement.  With
           sval = +src_h (the sign this file shipped until 2026-09-29) the y
           centre lands at (2*starty - src_h) << 3, which for the shipped
           800x480 -> 320x320 geometry is -320 px where +160 px is needed, so
           the rectangle is drawn a whole source-height ABOVE the destination
           and is empty.  That is what made every pixel of the destination read
           as the fill's constant 0x07C3 (GPU_CROP_STUCK) on every boot and for
           every scene: the blit painted nothing and the pre-fill was the only
           thing that ever reached the plane.

           Both are 28.4 fixed point, and the prologue says so directly
           (31888-31941): r4 = cx, r5 = cy, r10 = width*cos/2,
           [sp+0x18] = -width*sin/2, r11 = height*sin/2,
           [sp+0x14] = height*cos/2, and then the corners as
           r8 = cx - r10 - r11 and r9 = cy - [sp+0x18] - [sp+0x14] - centre
           minus half extent.  So cx = (2*startx + src_w) << 3 is 16x the centre
           in pixels and width = (src_w*scale_x) >> 4 is 16x the natural width.
           Two independent rows of the measured table in gpu.h fix the sign
           with no residual - start (0,0) scale 256 -> offsets (0,0), and start
           (0,0) scale 240 -> offsets (25,15), the ceil() rule - and they come
           out right only with sval = -src_h.  `cval`/`sval` are signed ints,
           so the casts below preserve the bit pattern rather than saturating.

           ARMED ACROSS THE WHOLE DRAW, and now it covers only bounded work.
           n32h7xx_it.c reads this word as `STALL s38 c0 p0`; disarmed on every
           way out so a stale stage can never be read as a stall during the next
           second of unrelated work.  See gpu_fb_put(). */
        nsgpu_draw_rotblit((void*)dev, fbw,
                           (int32_t)((2u * startx + src_w) << 3),
                           (int32_t)((2u * starty + src_h) << 3),
                           0.0f,
                           (int32_t)(((uint32_t)src_w * scale_x) >> 4),
                           (int32_t)(((uint32_t)src_h * scale_y) >> 4));
    }

    /* ---- THE BLIT'S COMMIT, THEN THE ADDRESS - in that order --------------
       gpu_fb_put() does `address` then `drain`, and that order is right for a
       caller whose job is to leave the framebuffer restored.  The reason to
       invert it here is FIDELITY, not a measured symptom: in the library the
       nsgpu_cmd_sync() at 29645 completed the blit BEFORE colorkey returned,
       so bak13's shipped front end always restored the address after the blit
       was finished.  Draining first restores that order exactly; the address
       write is then left in the driver's shadow, where the next commit - whose
       job is the next frame's - picks it up.

       A mechanism was offered for why the inverted order would be WRONG, and it
       is recorded as unverified rather than as fact: the address write is a
       command in the same unflushed stream as the blit, so the STC could
       execute it while the blit's pixels were still in the pipeline, and the
       framebuffer address is a register that pipeline may read late.  Nothing
       measures that, and the current readings cannot separate the two orders -
       the destination is the SAME constant `s C307C3` before and after the
       change (see the fill's commit above).  The change is kept because it
       matches the library's order, not because it has been shown to fix
       anything.

       `put` keeps its meaning: false = the GPU gave up, gpu_drain() has already
       set both latches and logged its `drain dead` line, and the caller's
       contract is to use the CPU path.  The address is put back on BOTH paths,
       because saved_fb is the value from before this call - on the failure path
       that restore is idempotent and no further commit happens. */
    put = gpu_drain((dhd_handle_t)a_handle);
    (void)dhd_set_framebuffer_address((dhd_handle_t)a_handle,
                                      (dhd_gpu_ptr_t)saved_fb);
    gpu_front_prog(0u);

    return put;
}

/* The five taps of the separable convolve.  Only the ratios matter: the library
   normalises them to 8-bit weights whose largest is 255 and then hands the
   texture unit a scale of 256/sum, so the net gain is 256/255 whichever kernel
   is here - see [[nsgpu-convolve-contract]] for that arithmetic, and
   s_conv_coeff above for how the tap order was established.

   The ACTIVE kernel is the 3-tap binomial, written into the middle three of the
   five taps.  The two ends have to stay zero rather than the array being
   shortened to operator_num 3: tap index k is offset k-2 in BOTH passes, so a
   3-long array is offsets -2,-1,0 and would shift the image by (+1,+1).  Padding
   to 5 is what keeps it centred, exactly as the identity above is written.

       identity   {0, 0, 1, 0, 0}    computes nothing, costs the same 62 ms
       ACTIVE     {0, 1, 2, 1, 0}    3-tap binomial -> {0,127,255,127,0}, sum 509
       low-pass   {1, 4, 6, 4, 1}    5-tap binomial -> {42,170,255,170,42}, sum 679

   Why the mildest of the three rather than the 5-tap: at the model's input scale
   a plate character is about 18x30 px.  The `box` lines a trigger prints are in
   800x480 camera pixels and run ~45 px wide, and the blit's x factor is 0.4, so
   the character the model sees is ~18 px across - a 5-tap kernel blurs across a
   good part of a glyph stroke where a 3-tap one does not.  The filter's
   justification is architectural rather than a measured detection gain (see
   GPU_FILTER_ENABLE in gpu.h), so the default should be the perturbation that is
   hardest to regret; the 5-tap is one line away for anyone who wants to compare.

   Non-negative only: the weights are (u8)(coeff * scale * 255 / max), so a
   negative tap cannot be represented at all and this filter can only blur.  A
   post-decimation blur is the cheap version; a true anti-alias would have to run
   on the 800x480 frame BEFORE the blit decimates it, which needs 768 KB of
   scratch and is a separate step. */
static int s_lp_coeff[5] = { 0, 1, 2, 1, 0 };

/* One implementation behind both entry points below, so the boot probe exercises
   the exact call the grab path makes and differs from it only in the coefficient
   array it hands over.

   Both buffers are 320x320 and the scales are 1:1, which is the one
   configuration this library's geometry is straightforward in - the centring
   offset the scaled blit needs is 0 when the natural extent equals the source
   extent.  Verified at this size by gpu_filter_probe() before the grab path was
   allowed to use it. */
static bool filter_frame_with(int* coeff)
{
    uint32_t saved_fb;
    uint32_t dead_dw = 0u;
    uint32_t dead_cv = 0u;
    uint32_t t_a;
    uint32_t t_b;
    uint32_t t_c;
    bool     put;
    bool     ok;

    /* Zeroed before every early return below, so the probe's line can never
       print a previous call's split as this one's.  g_blit_t_on is cleared
       too: this function is the only enabler, so an abort mid-call cannot
       leave the shared blit_tile() timing on for the depthwise path. */
    t_a = 0u;
    t_b = 0u;
    t_c = 0u;
    g_sep_t[0]  = 0u;
    g_sep_t[1]  = 0u;
    g_sep_t[2]  = 0u;
    g_sep_t[3]  = 0u;
    g_blit_t[0] = 0u;
    g_blit_t[1] = 0u;
    g_blit_t[2] = 0u;
    g_blit_t[3] = 0u;
    g_blit_t[4] = 0u;
    g_blit_t_on = 0u;

    if (!s_ready || (s_pixel_bytes != 2u))
    {
        return false;
    }

    /* The same latch check as gpu_scale_frame()'s, and for the same reason: the
       filter is the second stage of a front end that must not talk to a GPU the
       drain has already given up on.  It is also what keeps the two stages
       consistent - a scale that goes to the CPU and a filter that goes to the
       GPU would be a filtered picture assembled from two different sources. */
    gpu_dead_stat(&dead_dw, &dead_cv);
    if ((dead_dw != 0u) || (dead_cv != 0u))
    {
        return false;
    }

    /* Same argument as gpu_scale_frame(): the filter scale/bias word is the
       one ambient input this call does not set, so set it rather than inherit
       it.  1.0f/0 is not a guess - it is the state the filter leg was measured
       good in (`filt mean 5 ok 1`), because gpu_sb_reset() restores it at every
       probe exit and the probe ran with it there. */
    gpu_sb_reset();

    /* Everything from here to the template call is g_sep_t[0]: this stage's own
       setup, which is the same set of ambient writes gpu_scale_frame() makes. */
    t_a = DWT->CYCCNT;

    saved_fb = nsgpu_get_framebuffer();

    /* Its own destination geometry, same discipline as gpu_scale_frame() - and,
       as there, redundancy rather than a fix.  The bbox-leak reading of the
       `ok 0` round is falsified: the user's source shows
       blit_separable_convolve() sets its own bbox, so this call cannot be what
       makes the filter leg pass.  See the comment in gpu_scale_frame() for the
       withdrawn diagnosis and the surviving suspect. */
    dhd_set_framebuffer_size((dhd_handle_t)a_handle, GPU_SCALE_W, GPU_SCALE_H,
                             GPU_SCALE_W);
    dhd_set_bbox((dhd_handle_t)a_handle, 0u, 0u, GPU_SCALE_W - 1u,
                 GPU_SCALE_H - 1u, 0u);

    /* The arm is still placed BEFORE the draw, and from here on what it covers
       is OUR commit point rather than the library's: both of this call's passes
       end in blit_tile()'s bounded gpu_drain(), and the call restores the
       framebuffer through blit_restore(), which drains as well.

       gpu_keep_active_now() is GONE from this stage, and the stage-38 note above
       is the whole reason - it applies verbatim.  Forcing m_status ACTIVE is not
       a repair: it suppresses the start the STC needs after it stops itself at
       end-of-stream, which trades a refused-and-wedged engine for a silently
       hung one.  The keep belongs to gpu_drain(), at the moment it commits. */
    gpu_front_prog(39u);

    {
        /* The kernel as bytes, and the two gains, from the tap sum alone.

           The library normalised the caller's ints on its own -
           (u8)(coeff * scale * 255 / max), whose arithmetic this file still
           carries at the head of this section - because its path forced the
           largest tap to 255.  This path does not: the taps reach the CLUT as
           the caller wrote them, so {0,1,2,1,0} stays {0,1,2,1,0}, and the
           divisions the library's normalisation used to carry are carried by the
           two scale arguments instead.  That is why they are computed here
           rather than passed as 1.0.

           BOTH PASSES CARRY THE SAME GAIN, 8/S, and the 8 is in EACH of them for
           the same reason: every pass of this template reads a plane of RGB565
           texels under a CLUT kernel, and a 565 field read that way has a
           multiplicand of rep(field)/8 - one EIGHTH of its 8-bit value.  Pass 1
           reads GPU_SCALE_DST, pass 2 reads the intermediate; both therefore hand
           back an eighth, and both need the 8 put back.  S then normalises the
           taps.  So

               gain1 = 8/S   ->  scale1 = 32 * (8/S)
               gain2 = 8/S   ->  scale2 = 32 * (8/S) / MID_GAIN
                                              (the template multiplies the
                                               pass-2 argument by MID_GAIN
                                               itself, so the two scale NUMBERS
                                               this caller passes are the same
                                               value, one of them divided by
                                               255/31)

           With that, each pass is unity on its own: pass 1's row sum over a
           uniform field is S * (x/8) * (8/S) = x exactly, and pass 2's is
           S * (x/8) * (8/S) = x.

           WHICH IS ALSO WHY IT CANNOT RAIL, and the argument is stronger than the
           one this comment used to make.  With non-negative taps summing to S,

               R1 = (8/S) * SUM(taps[i] * m_in[i])  <=  (8/S) * S * 31.875 = 255

           because m_in <= rep(31)/8 = 31.875 for BOTH field widths (a 5-bit 31
           and a 6-bit 63 both replicate to 255).  So no pass-1 output can leave
           [0, 255] for ANY non-negative kernel and any input - the template's
           [8, 255] headroom rule is satisfied by construction, and equality is
           reached only by a kernel that is all taps at full scale.  The previous
           cut used gain1 = 1/S and justified it as "a convex combination of the
           taps"; that arithmetic was right and the INPUT was wrong - the taps are
           applied to the eighth-scale multiplicand, not to the 8-bit texel - and
           it cost an 8x dark plane, measured 2026-09-29 as `gpu agree filt mean
           111 max 207 swap 111 ok 0` with GPU_SCALE_CONV black (see the note on
           `filt vs blit` in camera.c).  Do not "simplify" the two gains back
           apart; they are equal by measurement, not by symmetry.
           Negative taps are the remaining way to leave the range (the sum can go
           below 0 and the 565 write clamps), which is why only S <= 0 is refused
           below and why the shipped kernel has none. */
        int8_t        k[GPU_CONV_TAPS];
        const int32_t b2[1] = { 0 };
        const int32_t b1[1] = { 0 };
        float         s2[1];
        float         s1[1];
        int32_t       s = 0;
        uint32_t      i;

        for (i = 0u; i < GPU_CONV_TAPS; i++)
        {
            int32_t t = coeff[i];

            if (t > 127)  { t = 127; }
            if (t < -128) { t = -128; }

            k[i] = (int8_t)t;
            s += t;
        }

        /* Taps that cancel leave no gain to normalise against, and that is not a
           filter.  Refused here rather than passed on, because a zero or
           negative divisor is the one input this function must not do
           arithmetic with; the caller reads the refusal as "use the CPU path",
           which is the same meaning every other false in here has. */
        if (s <= 0)
        {
            ok = false;
        }
        else
        {
            /* The SAME 8/S in both passes - see the block comment above.  Only
               pass 2's number is divided by MID_GAIN, because only the template's
               pass-2 line multiplies its argument by MID_GAIN. */
            s1[0] = BLIT_SEP_GAIN_UNIT * (8.0f / (float)s);
            s2[0] = (BLIT_SEP_GAIN_UNIT * (8.0f / (float)s)) / BLIT_SEP_MID_GAIN;

            /* ROWCOL and not ROWCOL_T, and the argument that chose T is now
               VOID: the T variant builds a vertical sum out of the ROW shape by
               storing pass 1's intermediate TRANSPOSED, because a COLUMN pass
               was believed to stop delivering taps past n = 2.  That belief was
               FALSIFIED ON THE BOARD (2026-10-08 - the template's own block has
               the measurement: on the address mode the driver requires for a
               width-1 kernel the same 3-tap column kernel gives THREE rows, and
               test 5's destination count went 6 -> 9).  So the two rows were a
               property of the address mode and not of the shape, and ROWCOL -
               ROW shape then COLUMN shape - is what blit_separable_convolve()
               itself does.

               WHAT IS OWED IN ITS PLACE IS A MEASUREMENT, not an argument.
               Nothing in this file now favours either mode, so the cost figures
               recorded for T (grab 1001 ms `pair` / 767 ms `x`, to the library
               arm's 412 / 115) say nothing about this one and may not be
               quoted for it.  The acceptance is the one `s` was accepted by:
               `pair`'s two arms in ONE boot, 5-6 runs each, spread reported
               before any difference is read ([[conv-chain-error-gate]]).

               Two consequences a later reader can rely on: ROWCOL_T's hard
               preconditions (dw == dh and sw == sh == 1) were its own and are
               dropped here - both hold at 320x320 anyway - and the transposed
               intermediate it needed is no longer produced, so GPU_SEP_TMP now
               holds pass 1's ordinary orientation.

               ONE THING THIS SWAP IS NOT VERIFIED ON, and it must be read out
               of the board rather than assumed: in test 5 the ROWCOL pair's two
               planes do not agree on where the impulse landed - `rowcol tmp`
               reads first at (31,5) and `rowcol dst` at (32,4), a one-row and
               one-column difference between an intermediate and the plane
               written from it.  A row pass and a column pass place the sampled
               window independently, so this is exactly the kind of offset that
               couples two passes, and a stage whose output is a picture can
               show it as a shifted image where a 9-of-9 count cannot.  `pair`'s
               diff and the visible plate are the readings that would show it. */
            /* g_sep_t[1] is this call end to end, and g_blit_t[] splits it
               from the inside: blit_tile() only accumulates while
               g_blit_t_on is non-zero, so the depthwise path's own blit_tile()
               calls (which never set it) keep reading zero here. */
            t_b = DWT->CYCCNT;
            g_blit_t_on = 1u;

            ok = blit_separable_convolve_new((dhd_handle_t)a_handle,
                                             BLIT_SEP_ROWCOL,
                                             (uint32_t)GPU_SCALE_DST,
                                             (uint32_t)GPU_SCALE_CONV,
                                             GPU_SCALE_W, GPU_SCALE_H,
                                             GPU_SCALE_W, GPU_SCALE_H,
                                             GPU_SCALE_W * 2u,
                                             (uint32_t)GPU_SEP_TMP,
                                             GPU_CV_CLUT_A,
                                             k, GPU_CONV_TAPS, 1u,
                                             s2, b2, s1, b1,
                                             0u, 1u, 0.0f, 0.0f, 1u, 1u,
                                             GPU_DW_FMT_SIGNED_565,
                                             E_DHD_PIXEL_ORG_RGB_565,
                                             E_DHD_ACCESS_LINEAR, 0u);

            g_blit_t_on = 0u;
            t_c = DWT->CYCCNT;
        }
    }

    put = gpu_fb_put((dhd_handle_t)a_handle, saved_fb);
    gpu_front_prog(0u);

    /* The split, in microseconds at 600 MHz.  t_b is the "the template was
       called" marker: it is set immediately before the call and nowhere else,
       so an `s <= 0` refusal (which returns before it) leaves the whole set at
       zero and the probe's line reads as "nothing was measured" rather than
       printing the wrap-around garbage an unsigned 0 - t_a would give. */
    if (t_b != 0u)
    {
        g_sep_t[0] = (t_b - t_a) / 600u;
        g_sep_t[1] = (t_c - t_b) / 600u;
        g_sep_t[2] = ((uint32_t)DWT->CYCCNT - t_c) / 600u;
        g_sep_t[3] = 1u;
    }

    /* `ok` and `put` are different questions and both are owed to the caller.
       The template restores the framebuffer itself (blit_restore()), so the
       gpu_fb_put() above is the same redundancy it is in gpu_scale_frame() - and
       it is what makes the drain's verdict ours: a drain that gave up must reach
       the caller as `false`, because the caller's reading of false is "take the
       CPU path".  `ok` is the template's verdict on the two draws, so the two
       are conjoined rather than chosen between. */
    return put && ok;
}

/* `blit_separable_convolve` is internal to nsgpu.lib and appears in no header.
   The 15-argument shape below is not guessed: nsgpu_separable_convolve builds
   exactly this call (nsgpu_lib_dis.txt:39745-39763) and blit_separable_convolve
   reads exactly those slots (28708-28938) - arg5/arg6 = dst_x/dst_y from
   [0x90]/[0x94], arg7/arg8 = n from [0x98]/[0x9c], arg9/arg10 = n/2 from
   [0xa0]/[0xa4], arg12/arg13 = the kernel from [0xac]/[0xb0], arg14 = the bias
   from [0xb4], which is what reaches dhd_set_texture_scalebias().  arg11
   ([0xa8]) is passed 0 and never read.  Its address is in Printf.map:24178
   (nsgpu.o, 592 B) - it is `Gb` and, unlike its caller, commits nothing. */
extern dhd_uint32_t blit_separable_convolve(dhd_handle_t a_device,
                                            const dhd_buffer_t* a_tex,
                                            const dhd_buffer_t* a_fb,
                                            dhd_uint32_t a_wrap,
                                            dhd_float32_t a_gain,
                                            dhd_uint32_t a_dstx,
                                            dhd_uint32_t a_dsty,
                                            dhd_uint32_t a_n,
                                            dhd_uint32_t a_nh,
                                            dhd_uint32_t a_half_a,
                                            dhd_uint32_t a_half_b,
                                            dhd_uint32_t a_zero,
                                            const dhd_uint8_t* a_k1,
                                            const dhd_uint8_t* a_k2,
                                            dhd_int32_t a_bias);

/* THE SHIPPED STAGE 3, NOW BOUNDED: nsgpu_separable_convolve TRANSCRIBED call
   for call, with its two nsgpu_cmd_sync(0) - dhd_flush + dhd_finish, the
   unbounded wait of [[gpu-frontend-unbounded-wait]] - replaced by our own
   bounded gpu_drain().

   WHY A TRANSCRIPTION RATHER THAN PRESSING `s`.  The gate's OFF state has to
   mean THIS operator and not "no filter": GPU_FILTER_ENABLE is 1 (gpu.h:195),
   so stage 3 IS part of the front end the product ships, and this is the picture
   the model has been handed.  Flipping `s` would ship the rank-1 pair instead -
   a picture change whose own acceptance is still owed (the protocol is in
   filter_frame_with()).  Transcribing keeps the picture and retires the wait,
   which is the only combination that changes nothing the model sees.

   WHY IT IS TRUSTWORTHY: the drawing code is not ours.  `blit_separable_convolve`
   - the exported, NON-committing half that does both draws, both CLUT allocations
   and frees, both bboxes, its own `dhd_wait` and the framebuffer restore - is
   called directly.  Only the ambient setup around it and the two commit points
   are ours.  (Same playbook stage 38 used on nsgpu_colorkey.)

   WHAT IS DELIBERATELY KEPT, because each is part of the picture:
     - the tap normalisation byte[i] = (u8)(coeff[i]*scale*255/max) and the gain
       scale*256/sum (39681-39743) - forcing the largest tap to 255 is the
       LIBRARY's behaviour, and this path passes scale 1.0f and bias 0 because of
       it.  Do not "simplify" it to the new path's 8/S form; they are different
       operators.
     - pass 1 IN PLACE on the source: the SAME struct is passed as tex and fb
       (39581-39582), which is the assembly form of the library's documented
       "pass 1 destroys the source".  That is why the struct's address is the
       SOURCE (GPU_SCALE_DST) and the wrapper sets the framebuffer to the
       DESTINATION (GPU_SCALE_CONV) so blit_separable_convolve's entry snapshot
       restores it there for pass 2.
     - the framebuffer format: the library reads it out of its own
       g_framebuffers[] entry, which is a file-static we cannot link to.  It is
       dhd_get_framebuffer_format(dev) - the same value, because nsgpu_init set
       both from one argument (see the stage-38 notes on the format round-trip).
     - gpu_sb_reset(), the same ambient discipline as the stage-38 and stage-39
       paths.

   gpu_keep_active_now() is GONE, and that is not a regression: this arm now owns
   its commit point, and the stage-38 note on the arm above covers why forcing
   m_status ACTIVE is not a repair.

   THE HONEST LIMIT: the literal library call is no longer in the product, so the
   `s` comparison can no longer be made against it inside one boot - the reference
   is now the numbers that arm already produced (f-on grab 111.7..119.0 ms,
   `pair diff mean` 6).  nsgpu_separable_convolve is still in the image and still
   exercised every boot by gpu_selftest()'s legs 3 and 4, which CALL the library
   on purpose - that self test exists to check the library against the CPU, so it
   must keep calling the library, not our transcription of it. */
static bool filter_frame_lib(int* coeff)
{
    uint32_t     saved_fb;
    uint32_t     dead_dw = 0u;
    uint32_t     dead_cv = 0u;
    uint32_t     fmt;
    uint32_t     n;
    uint32_t     i;
    uint32_t     sum;
    uint32_t     ret;
    float        maxv;
    float        gain;
    dhd_buffer_t buf;
    dhd_uint8_t* k = (dhd_uint8_t*)s_conv_kernel;
    bool         put;

    if (!s_ready || (s_pixel_bytes != 2u))
    {
        return false;
    }

    /* Same latch check as the new path, and it has to be the same: a filter that
       went to the GPU while the blit went to the CPU would build the picture out
       of two different sources. */
    gpu_dead_stat(&dead_dw, &dead_cv);
    if ((dead_dw != 0u) || (dead_cv != 0u))
    {
        return false;
    }

    /* 39488-39507 is the library's tap-count promotion (a 1..3-tap request is
       replaced by its own 5-tap low-pass with scale forced to 1.0f and bias to
       0).  GPU_CONV_TAPS is 5, so that branch is not reachable on this path and
       is not written out - but it is why scale is 1.0f and bias 0 below rather
       than the caller's values.

       THE QUANTISATION (39615-39743).  max takes the largest tap; a largest tap
       of <= 0 leaves nothing to normalise against and the library would divide
       by it, so it is refused here and read by the caller as "use the CPU path",
       which is what every other false in this file means.  (The shipped kernel's
       largest tap is 2; VCVT.U32.F32 truncates toward zero, and so does the C
       cast, so the two agree byte for byte.) */
    n    = (uint32_t)GPU_CONV_TAPS;
    maxv = 0.0f;

    if (n == 0u)
    {
        return false;
    }

    for (i = 0u; i < n; i++)
    {
        const float t = (float)coeff[i];

        if (t > maxv)
        {
            maxv = t;
        }
    }

    if (maxv <= 0.0f)
    {
        return false;
    }

    sum = 0u;

    for (i = 0u; i < n; i++)
    {
        k[i] = (dhd_uint8_t)(((float)coeff[i] * 255.0f) / maxv);
        sum += (uint32_t)k[i];
    }

    if (sum == 0u)
    {
        return false;
    }

    gain = 256.0f / (float)sum;

    gpu_sb_reset();

    saved_fb = nsgpu_get_framebuffer();

    dhd_set_framebuffer_size((dhd_handle_t)a_handle, GPU_SCALE_W, GPU_SCALE_H,
                             GPU_SCALE_W);
    dhd_set_bbox((dhd_handle_t)a_handle, 0u, 0u, GPU_SCALE_W - 1u,
                 GPU_SCALE_H - 1u, 0u);

    /* The arm covers OUR commit point now, not the library's: both commits below
       are gpu_drain()'s bounded poll.  gpu_keep_active_now() was deleted with
       the same argument the stage-38 note gives - the keep belongs to
       gpu_drain(), at the moment it commits. */
    gpu_front_prog(39u);

    /* 39528-39562: the framebuffer the entry snapshot restores to, and the
       destination pass 2 draws into.  The wrapper above already set size/bbox;
       the library's own body sets format/size/address again, and that repetition
       is reproduced rather than tidied. */
    fmt = (uint32_t)dhd_get_framebuffer_format((dhd_handle_t)a_handle);

    dhd_set_framebuffer_format((dhd_handle_t)a_handle, (dhd_pixel_org_t)fmt);
    dhd_set_framebuffer_size((dhd_handle_t)a_handle, GPU_SCALE_W, GPU_SCALE_H,
                             GPU_SCALE_W);
    (void)dhd_set_framebuffer_address((dhd_handle_t)a_handle,
                                      (dhd_gpu_ptr_t)GPU_SCALE_CONV);

    /* 39563-39582: the struct that is BOTH tex and fb.  blit_separable_convolve
       reads only +0x00, +0x14, +0x16, +0x18 and +0x20 of it (exhaustive over
       28708-28938); +0x1a (m_bpp) and +0x1e (m_usage) are inert there, and the
       library's values for them (the entry's bpp, and 2) are copied anyway so the
       state matches. */
    memset(&buf, 0, sizeof(buf));
    buf.m_memory.m_gpu_address = (dhd_gpu_ptr_t)GPU_SCALE_DST;
    buf.m_width                = (dhd_uint16_t)GPU_SCALE_W;
    buf.m_height               = (dhd_uint16_t)GPU_SCALE_H;
    buf.m_pitch                = (dhd_uint16_t)GPU_SCALE_W;
    buf.m_bpp                  = (dhd_uint16_t)(s_pixel_bytes);
    buf.m_usage                = (dhd_memory_usage_t)2;
    buf.m_format               = (dhd_enum_t)fmt;

    /* 39583-39614: the ambient texture and colour state.  The texture unit, its
       address, size, format and kernel are NOT here - they are inside
       blit_separable_convolve, along with the CLUT. */
    dhd_active_texture_unit((dhd_handle_t)a_handle, 0u);
    dhd_set_texture_unit_enable((dhd_handle_t)a_handle, 1u);
    dhd_util_color_replace((dhd_handle_t)a_handle, (dhd_col_input_t)0x13u);
    dhd_set_texture_filter((dhd_handle_t)a_handle, (dhd_bool_t)0, (dhd_bool_t)0);
    dhd_set_texture_scalebias((dhd_handle_t)a_handle, 1.0f, 0);
    dhd_set_color_blending((dhd_handle_t)a_handle, (dhd_blend_factor_t)6,
                           (dhd_blend_factor_t)2);
    dhd_set_alpha_blending((dhd_handle_t)a_handle, (dhd_blend_factor_t)6,
                           (dhd_blend_factor_t)2);

    /* COMMIT 1 of 2 - the library's nsgpu_cmd_sync(0) at 39562. */
    (void)gpu_drain((dhd_handle_t)a_handle);

    ret = blit_separable_convolve((dhd_handle_t)a_handle, &buf, &buf,
                                  (dhd_uint32_t)0u,
                                  gain,
                                  (dhd_uint32_t)0u, (dhd_uint32_t)0u,
                                  n, n, (n / 2u), (n / 2u),
                                  (dhd_uint32_t)0u,
                                  (const dhd_uint8_t*)k,
                                  (const dhd_uint8_t*)k,
                                  (dhd_int32_t)0);

    if (ret == 1u)
    {
        /* COMMIT 2 of 2 - 39768-39769, reached by the library only when the
           blit returned 1. */
        (void)gpu_drain((dhd_handle_t)a_handle);
    }

    put = gpu_fb_put((dhd_handle_t)a_handle, saved_fb);
    gpu_front_prog(0u);

    /* `ok` and `put` are different questions, as in filter_frame_with(): the blit
       has its own verdict and a draining that gave up must reach the caller as
       false.  Both, not either. */
    return put && (ret == 1u);
}

bool gpu_filter_frame(void)
{
    /* THE GATE LIVES HERE, not at the two call sites, so that the consumers
       cannot drift apart: camera_grab_pair() exists to measure the front end the
       PRODUCT has, so it and camera_grab_rgb888() must follow one decision.  Two
       separate ANDs would be two places to get it wrong, and the earlier cut did
       exactly that.

       OFF = filter_frame_lib(), the SHIPPED operator's picture - the library's
       nsgpu_separable_convolve transcribed with bounded commits, so it is
       byte-for-byte the picture the product has always had and no longer carries
       the unbounded dhd_finish.  ON = the bounded rank-1 pair, a different
       picture whose acceptance is still owed.  Neither state is "no filter". */
    return (g_gpu_front_sep_on != 0u) ? filter_frame_with(s_lp_coeff)
                                      : filter_frame_lib(s_lp_coeff);
}

bool gpu_filter_frame_new(void)
{
    /* What the BOOT PROBE calls, deliberately ungated: `gpu agree filt
       mean/max/swap/ok` is the new operator's acceptance reading and it has to be
       taken on every boot whether or not the product is using it.  Gating THIS
       entry point would make the one measurement the switch is waiting on
       disappear exactly when the switch is off - which is its default, i.e.
       always. */
    return filter_frame_with(s_lp_coeff);
}

/* What gpu_filter_probe() filters with, and it is deliberately NOT the shipped
   kernel.

   The grating that probe uses is a MASKED sawtooth - each channel steps by 8 and
   then wraps inside its own field (red every 4 px, green every 8, blue every 4
   diagonally).  The masking is what makes it loud: a one-pixel shift moves a
   channel by 8, twice GPU_CONV_GAP_MAX, so "shifted" and "filtered" cannot be
   the same measurement.

   But a sawtooth is the one thing a blur cannot reproduce.  At each wrap the
   value falls from 24 to 0, and the 3-tap averages across that fall: at x = 3 of
   every 4 columns it returns 16 where the grating holds 24, a difference of 8 -
   twice the bound.  Roughly half the columns of every channel would be counted
   bad, and the boot line would read `gpu conv NOT usable` while the filter was
   working exactly as designed.

   So the probe runs the identity.  Its question is the entry point's GEOMETRY at
   320x320 - no shift, no clip, every word written - and the identity is the one
   kernel under which any deviation from the grating is a geometry fault rather
   than a filter.  That the library honours the coefficient array AT ALL is a
   different question, and already answered on every boot by gpu_selftest() leg
   4: the one-pixel shift kernel of s_shift_coeff, which no passthrough can
   imitate.  Changing the shipped taps must not change this line. */
static int s_probe_coeff[5] = { 0, 0, 1, 0, 0 };

/* The grating gpu_filter_probe() uses, and it is chosen for loudness rather than
   for looks.  Every channel steps by 8 in its own field, so a one-pixel shift in
   either axis moves each channel by 8 - which is twice GPU_CONV_GAP_MAX, the
   most the filter's own rounding can cost.  A pattern that moved by less than the
   rounding would make "shifted" and "filtered" the same measurement, which is the
   mistake this probe exists to avoid repeating at a second entry point. */
static uint16_t conv_pixel(uint32_t x, uint32_t y)
{
    if (s_pixel_bytes == 2u)
    {
        return (uint16_t)((((x * 8u) & 0x1Fu) << 11) |
                          (((y * 8u) & 0x3Fu) << 5) |
                          (((x + y) * 8u) & 0x1Fu));
    }

    return (uint16_t)(((x + y) * 8u) & 0xFFu);
}

bool gpu_filter_probe(uint32_t* out_bad)
{
    uint32_t x;
    uint32_t y;
    uint32_t bad  = 0u;
    uint32_t first = 0u;
    uint32_t nsent = 0u;
    uint32_t n0 = 0u;
    uint32_t n12 = 0u;
    uint32_t n34 = 0u;
    uint32_t minx = GPU_SCALE_W;
    uint32_t maxx = 0u;
    uint32_t miny = GPU_SCALE_H;
    uint32_t maxy = 0u;

    if (out_bad != NULL)
    {
        *out_bad = 0u;
    }

    if (!s_ready || (s_pixel_bytes != 2u))
    {
        return false;
    }

    /* The input, then the destination cleared to a value the filter cannot
       produce - so a word it never wrote is identifiable rather than wrong. */
    for (y = 0u; y < GPU_SCALE_H; y++)
    {
        for (x = 0u; x < GPU_SCALE_W; x++)
        {
            ((volatile uint16_t*)GPU_SCALE_DST)[(y * GPU_SCALE_W) + x] =
                conv_pixel(x, y);
            ((volatile uint16_t*)GPU_SCALE_CONV)[(y * GPU_SCALE_W) + x] =
                (uint16_t)GPU_CROP_SENT;
        }
    }

    /* s_probe_coeff, NOT the shipped kernel - see its declaration for why the
       masked-sawtooth grating forces the probe onto the identity. */
    if (!filter_frame_with(s_probe_coeff))
    {
        return false;
    }

    for (y = 0u; y < GPU_SCALE_H; y++)
    {
        for (x = 0u; x < GPU_SCALE_W; x++)
        {
            const uint32_t i = (y * GPU_SCALE_W) + x;
            const uint16_t d = ((const volatile uint16_t*)GPU_SCALE_CONV)[i];
            const uint16_t e = conv_pixel(x, y);
            uint32_t g;

            if (d == (uint16_t)GPU_CROP_SENT)
            {
                nsent++;
            }

            g = channel_gap(d, e);

            if (g == 0u)       { n0++; }
            else if (g <= 2u)  { n12++; }
            else if (g <= GPU_CONV_GAP_MAX) { n34++; }

            if (g > GPU_CONV_GAP_MAX)
            {
                if (bad == 0u)
                {
                    first = i;
                }
                bad++;
                if (x < minx) { minx = x; }
                if (x > maxx) { maxx = x; }
                if (y < miny) { miny = y; }
                if (y > maxy) { maxy = y; }
            }
        }
    }

    /* b, f and n are the verdict; the box says WHERE a failure is, which is what
       localised the blit's centring - a rectangle in a corner is a placement
       fault and a border is a clip, and the counts alone cannot tell them apart. */
    sprintf(Message, "gpu conv b%lu f%lu n%lu\r\n",
            (unsigned long)bad, (unsigned long)first, (unsigned long)nsent);
    UART_SendString(Message);

    if (bad == 0u)
    {
        UART_SendString("gpu conv box none\r\n");
    }
    else
    {
        sprintf(Message, "gpu conv box r%lu-%lu c%lu-%lu\r\n",
                (unsigned long)miny, (unsigned long)maxy,
                (unsigned long)minx, (unsigned long)maxx);
        UART_SendString(Message);
    }

    /* The real precision cost of putting this in the path: how the 320x320
       actually came back, in channel LSBs. */
    sprintf(Message, "gpu conv g%lu %lu %lu\r\n",
            (unsigned long)n0, (unsigned long)n12, (unsigned long)n34);
    UART_SendString(Message);

    sprintf(Message, "gpu conv px %04X %04X %04X %04X\r\n",
            (unsigned)((const volatile uint16_t*)GPU_SCALE_CONV)[0],
            (unsigned)((const volatile uint16_t*)GPU_SCALE_CONV)[GPU_SCALE_W - 1u],
            (unsigned)((const volatile uint16_t*)GPU_SCALE_CONV)
                      [(GPU_SCALE_H - 1u) * GPU_SCALE_W],
            (unsigned)((const volatile uint16_t*)GPU_SCALE_CONV)
                      [((GPU_SCALE_H - 1u) * GPU_SCALE_W) + GPU_SCALE_W - 1u]);
    UART_SendString(Message);

    if (out_bad != NULL)
    {
        *out_bad = bad;
    }

    return bad == 0u;
}

/* The signed entry point, brought up for the first time.

   This is a bring-up probe, not a geometry test, and it is not in the trigger
   path - nothing calls it but boot.  What it answers is narrow on purpose: can
   this firmware make nsgpu_convolve() run at all, and does it produce a SIGNED
   response at the 320x320 size the front end uses.  It deliberately does not
   assert WHERE the response lands, because the scaled blit needed a centring
   correction that took several rounds to find ([[nsgpu-scaled-blit-contract]])
   and this is a third entry point again - so it reports the profile and lets
   the position be read off it.

   The pattern is one vertical step, dark left of centre and a small step up to
   the right of it.  Against a step the three candidate behaviours are three
   different SHAPES in a horizontal profile, which is what makes one line enough:

     passthrough    -> dark up to the step, then bright for the rest of the row
     unsigned blur  -> the step, smoothed
     signed gradient-> zero on BOTH flats, a bump only at the step

   Only the signed one comes back to zero on the right, so the profile settles
   it without any assumption about scaling.

   Measured on hardware 2026-09-17: `n0`, and a profile of
   0000 0000 0021 0021 | 0021 0000 0000 0000 across x = 40,150,158,160,161,
   162,165,280 - zero on both flats with the bump confined to the four columns
   around the step.  That also pins the arithmetic, which nothing had done:

       out = (sum over d of k_d * I(x+d)) * filter_scale / 256
       k   = (int8)(coeff << 5)

   The step was 0x0000 -> 0x1084, i.e. 2/4/4 per field.  A tap of 64 against
   the 4-LSB green step gives 256, and 256/256 is exactly the 1 that came back -
   so the divisor is 256, not the 32 the <<5 alone suggests, and filter_scale
   multiplies on top.  The columns nearest the step give 64*(B-A)/256 =
   (0.5, 1, 1) and the next two 96*(B-A)/256 = (0.75, 1.5, 1.5); both truncate
   to (0, 1, 1) = 0x0021, which is why the bump reads flat instead of 2-3-3-2.

   The gain being small is the format, not a fault: unlike the separable path,
   which is handed `filter_scale * 256 / sum` BY THE LIBRARY, this one is handed
   the caller's filter_scale raw.  There is no normalisation by the kernel sum
   here, so an operator that should not change the overall level has to divide
   by that sum itself when choosing filter_scale.  That is also what the
   vendor's forced 3.0 for the built-in sobel is for.

   And the geometry needed no correction.  The step was written at x = 160 and
   the response sits at 158..161, so dst_startx/dst_starty = 0 places the result
   1:1 with no offset - the centring correction the scaled blit needed
   ([[nsgpu-scaled-blit-contract]]) is a property of SCALING, since the box it
   centres in is source-extent, and there is no scaling here.

   filter_bias is 0 here, and that is a decision, not a default.  The built-in
   operators (type 1/2/3 = sobel/prewitt/roberts) force scale 3.0 AND BIAS 60,
   which the demo can afford at E_DHD_PIXEL_ORG_L_8 (8-bit luminance) but not at
   RGB565: 60 exceeds the 5-bit red and blue fields, so those two saturate before
   the kernel does anything and the operator's output lives in the green field
   alone.  On this pipeline the signed path has to be operator_type 0 with our own
   coefficients and our own bias.

   The coefficients are the vendor's g_coeff_edge_sobel, {-2,-1,0,1,2}, the same
   five values in the same order: nsgpu_convolve packs them as (int8)(coeff<<5),
   i.e. a 1/32 resolution over +-3.97, so five taps at +-2 is what the format is
   sized for. */
static int s_edge_coeff[5] = { -2, -1, 0, 1, 2 };

bool gpu_convolve_probe(uint32_t* out_bad)
{
    const volatile uint16_t* o = (const volatile uint16_t*)GPU_SCALE_CONV;
    const uint32_t row = GPU_SCALE_H / 2u;
    dhd_handle_t   dev = (dhd_handle_t)a_handle;
    uint32_t saved_fb;
    uint32_t x;
    uint32_t y;
    uint32_t u;
    uint32_t nsent = 0u;

    if (out_bad != NULL)
    {
        *out_bad = 0u;
    }

    if (!s_ready || (s_pixel_bytes != 2u))
    {
        return false;
    }

    /* The step, into the source, and the destination cleared to a value no
       convolution of this input can produce - so a word the blitter never wrote
       is identifiable rather than merely wrong. */
    for (y = 0u; y < GPU_SCALE_H; y++)
    {
        for (x = 0u; x < GPU_SCALE_W; x++)
        {
            ((volatile uint16_t*)GPU_SCALE_DST)[(y * GPU_SCALE_W) + x] =
                (uint16_t)((x < (GPU_SCALE_W / 2u)) ? 0x0000u : 0x1084u);
            ((volatile uint16_t*)GPU_SCALE_CONV)[(y * GPU_SCALE_W) + x] =
                (uint16_t)GPU_CROP_SENT;
        }
    }

    /* nsgpu_convolve, NOT nsgpu_separable_convolve: this is the only one of the
       two that can carry a negative tap.  dst becomes the driver's framebuffer
       exactly as it does on the other two paths, so it is restored. */
    saved_fb = nsgpu_get_framebuffer();
    nsgpu_convolve_k(0u, GPU_CONV_TAPS, 1.0f, 0, s_edge_coeff,
                     (u32*)GPU_SCALE_CONV, GPU_SCALE_W, GPU_SCALE_H,
                     0u, 0u,
                     (u32*)GPU_SCALE_DST, GPU_SCALE_W, GPU_SCALE_H,
                     GPU_SCALE_W,
                     s_conv2d_kernel);

    /* ---------------------------------------------------------------
       Read back the texture state the library itself just left behind.

       Every configuration the dw probe can express reads zero in kernel mode -
       modes 4 and 5, QUAD and ROW, LINEAR and TILED_2, AL_8 and RGB565 (the
       `gpu dw km` corner matrix) - and in the one test where the CLUT mode is
       the ONLY variable, mode 5 gives 0 where mode 1 gives the right answer
       (`gpu dw pk`: 1 2 11 for indexed colour, 0 0 0 for signed kernel, same
       function, same CLUT, same window).  If nsgpu_convolve() were built on
       E_DHD_CLUT_KERNEL, mode 4/5 would have to produce texture values here.
       It does not.

       So stop inferring its setup from the disassembly and read it.  These are
       the driver's own accessors over its own shadow state - what the driver
       was TOLD, with no decode in between.  Both units, because the accessors
       follow whichever one dhd_active_texture_unit() last selected. */
    for (u = 0u; u < 2u; u++)
    {
        dhd_address_mode_t am   = (dhd_address_mode_t)0;
        dhd_bool_t         disc = (dhd_bool_t)0;
        dhd_bool_t         fu   = (dhd_bool_t)0;
        dhd_bool_t         fv   = (dhd_bool_t)0;
        dhd_texel_org_t    fmt;
        uint32_t           sw = 0u, sh = 0u, sp = 0u;

        dhd_active_texture_unit(dev, u);
        fmt = dhd_get_texture_format(dev, &am, &disc);
        dhd_get_texture_filter(dev, &fu, &fv);
        dhd_get_texture_size(dev, &sw, &sh, &sp);

        sprintf(Message, "gpu cv2 t%lu fmt %04X am %lu flt %lu%lu da %lu\r\n",
                (unsigned long)u, (unsigned)fmt, (unsigned long)am,
                (unsigned long)fu, (unsigned long)fv, (unsigned long)disc);
        UART_SendString(Message);

        sprintf(Message, "gpu cv2 t%lu sz %lux%lu p%lu cl %lu en %lu\r\n",
                (unsigned long)u, (unsigned long)sw, (unsigned long)sh,
                (unsigned long)sp, (unsigned long)dhd_get_texture_clut_base(dev),
                (unsigned long)dhd_get_texture_unit_enable(dev));
        UART_SendString(Message);
    }

    nsgpu_set_framebuffer(saved_fb);

    for (y = 0u; y < GPU_SCALE_H; y++)
    {
        for (x = 0u; x < GPU_SCALE_W; x++)
        {
            if (o[(y * GPU_SCALE_W) + x] == (uint16_t)GPU_CROP_SENT)
            {
                nsent++;
            }
        }
    }

    /* n first: a call that changed nothing and a call that faulted are different
       faults, and n separates them before the profile is read. */
    sprintf(Message, "gpu cv2 n%lu\r\n", (unsigned long)nsent);
    UART_SendString(Message);

    /* Only 8 of the 320 columns, chosen either side of the centre column the
       step was written at so the bump is bracketed even if the blit places it a
       few columns off.  Two lines because 8 hex words plus a label is past the
       64-byte Message. */
    sprintf(Message, "gpu cv2 a %04X %04X %04X %04X\r\n",
            (unsigned)o[(row * GPU_SCALE_W) + 40u],
            (unsigned)o[(row * GPU_SCALE_W) + 150u],
            (unsigned)o[(row * GPU_SCALE_W) + 158u],
            (unsigned)o[(row * GPU_SCALE_W) + 160u]);
    UART_SendString(Message);

    sprintf(Message, "gpu cv2 b %04X %04X %04X %04X\r\n",
            (unsigned)o[(row * GPU_SCALE_W) + 161u],
            (unsigned)o[(row * GPU_SCALE_W) + 162u],
            (unsigned)o[(row * GPU_SCALE_W) + 165u],
            (unsigned)o[(row * GPU_SCALE_W) + 280u]);
    UART_SendString(Message);

    if (out_bad != NULL)
    {
        *out_bad = nsent;
    }

    return nsent == 0u;
}

/* The rows the 1:1 passes compare against: FILL across the first GPU_CROP_W
   columns, OTHER for the rest of the 800-wide row.  The OTHER tail is what makes
   a wrong stride visible - a sampler that walks the texture at the wrong row
   length reads into it and the count says so.  Only the first GPU_CROP_H rows
   are filled; nothing correct samples beyond them. */
static void crop_fill(void)
{
    volatile uint16_t* p = (volatile uint16_t*)GPU_CROP_SRC;
    uint32_t y;
    uint32_t x;

    for (y = 0u; y < GPU_CROP_H; y++)
    {
        for (x = 0u; x < GPU_CROP_PITCH; x++)
        {
            p[(y * GPU_CROP_PITCH) + x] = (uint16_t)GPU_CROP_OTHER;
        }
        for (x = 0u; x < GPU_CROP_W; x++)
        {
            p[(y * GPU_CROP_PITCH) + x] = (uint16_t)GPU_CROP_FILL;
        }
    }
}

/* The whole 800x480 at GPU_CROP_FILL, for the pass that reproduces the
   production geometry.  Written 32 bits at a time because it is the one fill
   big enough to notice. */
static void crop_fill_all(void)
{
    volatile uint32_t* p = (volatile uint32_t*)GPU_CROP_SRC;
    const uint32_t n = (GPU_CROP_PITCH * GPU_CROP_PROD_H) / 2u;
    const uint32_t v = ((uint32_t)GPU_CROP_FILL << 16) | (uint32_t)GPU_CROP_FILL;
    uint32_t i;

    for (i = 0u; i < n; i++)
    {
        p[i] = v;
    }
}

/* What one crop_pass() found.  A struct rather than five out-parameters because
   the 2026-09-16 round added four more answers to the same question. */
typedef struct
{
    uint32_t bad;       /* destination words that are not what was due */
    uint32_t first;     /* index of the first of them */
    uint16_t dw;        /* what the GPU left at that index */
    uint16_t sw;        /* what was due there */
    uint32_t nfill;     /* over the compared region, how many words are FILL */
    uint32_t nstuck;    /*                                  ... GPU_CROP_STUCK */
    uint32_t nsent;     /*                                  ... GPU_CROP_SENT */
    uint32_t nother;    /*                                  ... anything else */
    uint16_t oth0;      /* the first of those "anything else", 0 if none */
} crop_res;

/* Fill the whole 320x320 destination with `v`, so that a pass which does not
   write a pixel leaves a value nothing else produces.

   This is the answer to the ambiguity the 2026-09-16 sweep could not resolve.
   Nothing cleared the destination between passes, and the sweep ran scale 256
   first - which writes all 102400 words correctly.  So on every later line of
   that sweep a word that equalled FILL might have been written by this pass or
   might have been sitting there since the last one, and "the correct fraction
   falls as the scale falls" is the shape of BOTH "the blit writes less" and
   "the leftovers are what is left".  With a sentinel the two are different
   symbols and the question does not come back.

   32 bits at a time: 205 KB is the largest thing the CPU writes here. */
static void crop_clear_dst(void)
{
    volatile uint32_t* p = (volatile uint32_t*)GPU_SCALE_DST;
    const uint32_t n = (GPU_SCALE_W * GPU_SCALE_H) / 2u;
    const uint32_t v = ((uint32_t)GPU_CROP_SENT << 16) | (uint32_t)GPU_CROP_SENT;
    uint32_t i;

    for (i = 0u; i < n; i++)
    {
        p[i] = v;
    }
}

/* The whole 800x480 as a positional ramp, for the pass that asks whether the
   scaled path reads the source AT ALL.

   Every other scaled pass uses an all-FILL source, and all that can be learned
   from one is "the destination is FILL or it is not".  A ramp makes the source
   a coordinate: if any destination pixel takes its value from the source, the
   value says which source pixel, and the mapping falls out of the readback.  If
   instead the destination comes back as the same constant it comes back as
   under an all-FILL source, then the scaled path never read the source - which
   is a much stronger statement than any count of a uniform field can make,
   because a uniform field reads the same whether it was sampled or ignored.

   v = 7x + 13y is not injective over 800x480, but it does not need to be: what
   is being asked is whether the output's values lie in the ramp's range at all,
   and a value outside it cannot have come from the source. */
static void crop_fill_ramp(void)
{
    volatile uint16_t* p = (volatile uint16_t*)GPU_CROP_SRC;
    uint32_t y;
    uint32_t x;

    for (y = 0u; y < GPU_CROP_PROD_H; y++)
    {
        for (x = 0u; x < GPU_CROP_PITCH; x++)
        {
            p[(y * GPU_CROP_PITCH) + x] = (uint16_t)((7u * x) + (13u * y));
        }
    }
}

/* Paint GPU_CROP_NEIGH into the memory on both sides of the source block.

   192 KB below and 528 KB above, which is as far as the free gap reaches in each
   direction before it runs into CAM_ALT_BUF (0xC0380000, which the camera owns)
   and into nothing in particular (the gap above the source is 7 MB of unused
   SDRAM).  Not 32-bit at a time: these two runs are not aligned and not the
   point of the exercise. */
static void crop_fill_neigh(void)
{
    volatile uint16_t* lo = (volatile uint16_t*)0xC03D0000u;
    volatile uint16_t* hi = (volatile uint16_t*)(GPU_CROP_SRC +
                                                 (GPU_CROP_PITCH * GPU_CROP_PROD_H * 2u));
    uint32_t i;

    for (i = 0u; i < (96u * 1024u); i++)
    {
        lo[i] = (uint16_t)GPU_CROP_NEIGH;
    }

    for (i = 0u; i < (264u * 1024u); i++)
    {
        hi[i] = (uint16_t)GPU_CROP_NEIGH;
    }
}

/* Classify the WHOLE 320x320 destination, not just the region a pass was told to
   cover.  A scaled pass writes its rectangle and leaves everything else alone,
   so a scan of the whole thing says how many distinct values are present and
   how much of each - and the four named ones are the ones with a meaning: the
   source's FILL, the library's STUCK, the CPU's SENT, and GPU_SCALE_BG, which is
   the colour handed to nsgpu_colorkey as both background and key and which has
   not once been observed in a destination. */
static void crop_zone_scan(void)
{
    uint32_t nf = 0u;
    uint32_t ns = 0u;
    uint32_t nd = 0u;
    uint32_t nb = 0u;
    uint32_t no = 0u;
    uint16_t o0 = 0u;
    uint32_t i;

    for (i = 0u; i < (GPU_SCALE_W * GPU_SCALE_H); i++)
    {
        const uint16_t d = ((const volatile uint16_t*)GPU_SCALE_DST)[i];

        if (d == (uint16_t)GPU_CROP_FILL) { nf++; }
        else if (d == (uint16_t)GPU_CROP_STUCK) { ns++; }
        else if (d == (uint16_t)GPU_CROP_SENT) { nd++; }
        else if (d == (uint16_t)GPU_SCALE_BG) { nb++; }
        else { if (no == 0u) { o0 = d; } no++; }
    }

    sprintf(Message, "gpu whole F%lu S%lu N%lu B%lu o%lu %04X\r\n",
            (unsigned long)nf, (unsigned long)ns, (unsigned long)nd,
            (unsigned long)nb, (unsigned long)no, (unsigned)o0);
    UART_SendString(Message);
}

/* The FILL count of one destination row, and the columns of the first few of
   them.  `crop_report` says how much of the destination is right; this says
   WHERE, which is the difference between a rectangle, a period and a scatter -
   three shapes the counts read identically, and the reason the row map exists.
   Six columns is enough to read a period off. */
static void crop_row_scan(uint32_t row)
{
    char* p = Message;
    uint32_t n = 0u;
    uint32_t shown = 0u;
    uint32_t c;

    p += sprintf(p, "gpu row%lu n", (unsigned long)row);

    for (c = 0u; c < GPU_SCALE_W; c++)
    {
        if (((const volatile uint16_t*)GPU_SCALE_DST)[(row * GPU_SCALE_W) + c]
            == (uint16_t)GPU_CROP_FILL)
        {
            n++;
        }
    }

    p += sprintf(p, "%lu @", (unsigned long)n);

    for (c = 0u; (c < GPU_SCALE_W) && (shown < 6u); c++)
    {
        if (((const volatile uint16_t*)GPU_SCALE_DST)[(row * GPU_SCALE_W) + c]
            == (uint16_t)GPU_CROP_FILL)
        {
            p += sprintf(p, " %lu", (unsigned long)c);
            shown++;
        }
    }

    sprintf(p, "\r\n");
    UART_SendString(Message);
}

/* One blit and one count.  `compare_src` picks the expectation:

     true   destination (x,y) must equal source (x,y) word for word.  Only
            meaningful at scale 256, where destination and source extents are
            the same - which is what makes it a bit-exact test and not another
            mean.
     false  every word of the destination extent must be GPU_CROP_FILL.  Valid
            whenever the whole sampled part of the source is FILL, which is how
            every scaled pass is set up.

   The destination extent is the scaled source extent, clipped to the 320x320
   destination.  The destination STRIDE is GPU_SCALE_W and not the scaled
   extent: the 64x64 pass leaves the word at index 64 - the first word past a
   64-wide image - untouched, and that is only possible if the library lays the
   destination out at the width the caller declared.

   `pitch` is the SOURCE stride and is a parameter, not src_w.  It was src_w
   until the 2026-09-16 sweep, and that made own64 a test that could not fail:
   the comparison walked the source at stride src_w, exactly as the library was
   told to, so the two agreed on a source layout that was not the one in memory.
   own64 passed while reading the 64-column FILL block as if the 800-wide rows
   were 64-wide.  Every pass that uses a source narrower than its rows must now
   say so explicitly, and the GPU gets the same number the comparison uses.

   The caller clears the destination first; see crop_clear_dst().  Without that
   clear a word equal to the expectation is not evidence that this pass wrote
   it, and every count below is unreadable. */
static void crop_pass_at(uint32_t src, uint32_t src_w, uint32_t src_h,
                         uint32_t pitch,
                         uint32_t startx, uint32_t starty,
                         uint32_t scale_x, uint32_t scale_y, bool compare_src,
                         crop_res* r)
{
    const uint32_t cw = ((src_w * scale_x) >> 8);
    const uint32_t ch = ((src_h * scale_y) >> 8);
    const uint32_t lim_w = (cw < GPU_SCALE_W) ? cw : GPU_SCALE_W;
    const uint32_t lim_h = (ch < GPU_SCALE_H) ? ch : GPU_SCALE_H;
    uint32_t saved_fb;
    uint32_t x;
    uint32_t y;

    r->bad = 0u;
    r->first = 0u;
    r->dw = 0u;
    r->sw = 0u;
    r->nfill = 0u;
    r->nstuck = 0u;
    r->nsent = 0u;
    r->nother = 0u;
    r->oth0 = 0u;

    if ((lim_w == 0u) || (lim_h == 0u))
    {
        return;
    }

    saved_fb = nsgpu_get_framebuffer();

    nsgpu_colorkey((u32*)GPU_SCALE_DST, GPU_SCALE_W, GPU_SCALE_H,
                   startx, starty,
                   (u32*)src, GPU_SCALE_BG, 255u,
                   src_w, src_h, pitch,
                   0u, scale_x, scale_y);

    nsgpu_set_framebuffer(saved_fb);

    for (y = 0u; y < lim_h; y++)
    {
        for (x = 0u; x < lim_w; x++)
        {
            const uint32_t di = (y * GPU_SCALE_W) + x;
            const uint32_t si = (y * pitch) + x;
            const uint16_t d = ((const volatile uint16_t*)GPU_SCALE_DST)[di];
            const uint16_t s = compare_src
                             ? ((const volatile uint16_t*)src)[si]
                             : (uint16_t)GPU_CROP_FILL;

            if (d == (uint16_t)GPU_CROP_FILL) { r->nfill++; }
            else if (d == (uint16_t)GPU_CROP_STUCK) { r->nstuck++; }
            else if (d == (uint16_t)GPU_CROP_SENT) { r->nsent++; }
            else
            {
                if (r->nother == 0u) { r->oth0 = d; }
                r->nother++;
            }

            if (d != s)
            {
                if (r->bad == 0u)
                {
                    r->first = di;
                    r->dw = d;
                    r->sw = s;
                }
                r->bad++;
            }
        }
    }
}

/* The common case: the caller wants the rectangle at the framebuffer origin,
   which is where every pass but the compensated one starts. */
static void crop_pass(uint32_t src, uint32_t src_w, uint32_t src_h,
                      uint32_t pitch,
                      uint32_t scale_x, uint32_t scale_y, bool compare_src,
                      crop_res* r)
{
    crop_pass_at(src, src_w, src_h, pitch, 0u, 0u,
                 scale_x, scale_y, compare_src, r);
}

/* One "what is in the destination" line, shared by every scaled pass so the
   format cannot drift between them.  F/S/N are the three named words - FILL,
   STUCK, SENT - and `o` is the first word that is none of them, which is how a
   fourth value, a shifted or colour-converted one, becomes visible without a
   field to hold it.

   The letters are single because Message is 64 bytes and the full words did not
   fit: the same line spells out to 65 with them.  The legend is printed once
   before the first pass.

   F/S/N tell the story the counts alone cannot.  A word the pass never wrote is
   SENT, so `N 0` with S high means the pass covered the region and got it
   wrong, while S high with N high means it reached only part of it.  And a
   count of FILL is only evidence that this pass wrote it because the caller
   cleared the destination first. */
static void crop_report(const char* tag, const crop_res* r)
{
    sprintf(Message, "gpu %s b%lu f%lu F%lu S%lu N%lu o%04X\r\n",
            tag, (unsigned long)r->bad, (unsigned long)r->first,
            (unsigned long)r->nfill, (unsigned long)r->nstuck,
            (unsigned long)r->nsent, (unsigned)r->oth0);
    UART_SendString(Message);
}

bool gpu_scale_crop_probe(uint32_t cam_src, uint32_t cam_w)
{
    /* The production scales for an 800x480 source, computed the way
       gpu_scale_frame() computes them so the two cannot drift apart. */
    const uint32_t psx = ((GPU_SCALE_W * 256u) + GPU_CROP_PITCH - 1u) / GPU_CROP_PITCH;
    const uint32_t psy = ((GPU_SCALE_H * 256u) + GPU_CROP_PROD_H - 1u) / GPU_CROP_PROD_H;
    crop_res res;
    uint32_t nfill;
    uint32_t nstuck;
    uint32_t nother;
    uint32_t x;

    if (!s_ready)
    {
        return false;
    }

    /* Word comparisons, like gpu_scale_probe() and for the same reason: in L_8
       the pattern, the width and the pitch are all wrong. */
    if (s_pixel_bytes != 2u)
    {
        UART_SendString("gpu crop SKIP (not RGB565)\r\n");
        return false;
    }

    crop_fill();

    /* Every pass clears the destination to SENT first.  Without it a word equal
       to the expectation is not evidence that this pass wrote it, and the
       2026-09-16 sweep is what that costs: nothing cleared between passes, and
       its first line (scale 256) wrote all 102400 words correctly, so every line
       after it could have been reading the previous line's leftovers and there
       is no way to tell from the numbers.  See crop_clear_dst(). */
    UART_SendString("gpu crop tag b=bad f=first F=fill S=stuck N=sent o=other\r\n");

    /* The baseline trio, each one variable away from a call that has been
       measured to work: the 64x64 source at 1:1.  own64 is that call - it is
       here to be the baseline the others are read against.  cam64 changes only
       the SOURCE ADDRESS (the live frame buffer), own800 only the SOURCE EXTENT
       (800 wide instead of 64).  All three are expected bit-exact, and all three
       measured bit-exact on 2026-09-16, which is the finding: the address and
       the extent are good and the scale is not.

       All three now declare pitch 800, which is the stride their rows are
       actually laid out at.  own64 was the one that did not, and it is why the
       first run of this probe read as three passes: it was handed pitch 64 for a
       source whose rows are 800 apart, and compared against a source walked at
       64 - the same mistake on both sides, so it matched whatever it read.  With
       the pitch declared it is a real test. */
    crop_clear_dst();
    crop_pass(GPU_CROP_SRC, GPU_CROP_W, GPU_CROP_H, GPU_CROP_PITCH,
              256u, 256u, true, &res);
    sprintf(Message, "gpu crop own64 bad %lu f %lu d %04X s %04X\r\n",
            (unsigned long)res.bad, (unsigned long)res.first,
            (unsigned)res.dw, (unsigned)res.sw);
    UART_SendString(Message);

    crop_clear_dst();
    crop_pass(GPU_CROP_SRC, GPU_CROP_PITCH, GPU_CROP_H, GPU_CROP_PITCH,
              256u, 256u, true, &res);
    sprintf(Message, "gpu crop own800 bad %lu f %lu d %04X s %04X\r\n",
            (unsigned long)res.bad, (unsigned long)res.first,
            (unsigned)res.dw, (unsigned)res.sw);
    UART_SendString(Message);

    if ((cam_src != 0u) && (cam_w >= GPU_CROP_W))
    {
        crop_clear_dst();
        crop_pass(cam_src, cam_w, GPU_CROP_H, cam_w, 256u, 256u, true, &res);
        sprintf(Message, "gpu crop cam64 bad %lu f %lu d %04X s %04X\r\n",
                (unsigned long)res.bad, (unsigned long)res.first,
                (unsigned)res.dw, (unsigned)res.sw);
        UART_SendString(Message);
    }

    /* Everything from here on scales, so the source becomes the WHOLE 800x480
       block at FILL.  That is what makes the scale the only variable left: any
       region the sampler can reach, at any scale, is FILL, so the expectation is
       FILL everywhere and no prediction about which source pixels are sampled is
       needed.  The 64-column FILL block above cannot be used for this - a scaled
       pass would have to say which columns it reaches, and that is the very
       thing the fault would move. */
    crop_fill_all();

    /* The axis trio.  One source, 256x64, small enough that no extent is ever
       clipped, with the three scale combinations that separate the two axes:
       x scaled and y not, y scaled and x not, and both.  If only `axy` fails
       the fault is in what the two axes do together - an accumulator, an
       interaction - and not in either one alone.  Measured 2026-09-16: all three
       fail, so it is in both axes and in neither one alone. */
    crop_clear_dst();
    crop_pass(GPU_CROP_SRC, 256u, 64u, GPU_CROP_PITCH,
              103u, 256u, false, &res);
    crop_report("ax", &res);

    crop_clear_dst();
    crop_pass(GPU_CROP_SRC, 256u, 64u, GPU_CROP_PITCH,
              256u, 171u, false, &res);
    crop_report("ay", &res);

    crop_clear_dst();
    crop_pass(GPU_CROP_SRC, 256u, 64u, GPU_CROP_PITCH,
              103u, 171u, false, &res);
    crop_report("axy", &res);

    /* A sweep for a cliff.  The first sweep used the 64-column block and pitch
       64, so every line of it was measuring the pitch, not the scale - the
       numbers looked like a smooth ramp because they were counting how much of
       an 800-wide row happened to fall inside a 64-column expectation.  This one
       varies only the scale, and scale 256 is the anchor: it is the identity, it
       is the one the trio above proves works at pitch 800, so a failure there
       would mean the sweep itself differs from the trio rather than that a scale
       is broken.  Measured 2026-09-16: 256 is bit-exact and every other scale
       fails, so the anchor holds and there is no cliff - the fault is present
       the moment the scale is not 1.0. */
    {
        static const uint32_t k_scale[10] = {256u, 240u, 200u, 171u, 128u, 103u,
                                             64u, 32u, 16u, 8u};
        char tag[8];
        uint32_t k;

        for (k = 0u; k < 10u; k++)
        {
            crop_clear_dst();
            crop_pass(GPU_CROP_SRC, GPU_CROP_PITCH, GPU_CROP_PROD_H,
                      GPU_CROP_PITCH, k_scale[k], k_scale[k], false, &res);
            sprintf(tag, "s%lu", (unsigned long)k_scale[k]);
            crop_report(tag, &res);
        }
    }

    /* prod: the call the camera path makes, down to the extent and the scale,
       with only the source address changed - and all FILL, so all 102400
       destination words are due.

       The counts, the map and the source's own first and last word share one
       loop and one pass over the destination, so this is 205 KB read once rather
       than three times - which on this board's SDRAM is the difference between
       noticing and not. */
    crop_clear_dst();
    crop_pass(GPU_CROP_SRC, GPU_CROP_PITCH, GPU_CROP_PROD_H,
              GPU_CROP_PITCH, psx, psy, false, &res);
    crop_report("prod", &res);

    nfill = 0u;
    nstuck = 0u;
    nother = 0u;

    {
        char* mp = Message;
        uint32_t r;
        uint32_t nfrow = 0u;
        uint32_t first_frow = GPU_SCALE_H;

        UART_SendString("gpu prod map F=fill .=stuck S=sent(untouched) ?=mixed\r\n");

        for (r = 0u; r < GPU_SCALE_H; r++)
        {
            uint32_t nf = 0u;
            uint32_t ns = 0u;
            uint32_t nd = 0u;

            for (x = 0u; x < GPU_SCALE_W; x++)
            {
                const uint16_t d = ((const volatile uint16_t*)GPU_SCALE_DST)
                                   [(r * GPU_SCALE_W) + x];

                if (d == (uint16_t)GPU_CROP_FILL) { nf++; }
                else if (d == (uint16_t)GPU_CROP_STUCK) { ns++; }
                else if (d == (uint16_t)GPU_CROP_SENT) { nd++; }
            }

            nfill += nf;
            nstuck += ns;
            nother += (GPU_SCALE_W - nf - ns - nd);

            if (nf == GPU_SCALE_W)
            {
                *mp++ = 'F';
                nfrow++;
                if (first_frow == GPU_SCALE_H) { first_frow = r; }
            }
            else if (ns == GPU_SCALE_W) { *mp++ = '.'; }
            else if (nd == GPU_SCALE_W) { *mp++ = 'S'; }
            else { *mp++ = '?'; }

            /* 56 rows a line, so the line plus CRLF plus the terminator is 59
               bytes of the 64-byte Message. */
            if ((((r + 1u) % 56u) == 0u) || (r == (GPU_SCALE_H - 1u)))
            {
                *mp++ = '\r';
                *mp++ = '\n';
                *mp = '\0';
                UART_SendString(Message);
                mp = Message;
            }
        }

        sprintf(Message, "gpu prod fill %lu stuck %lu other %lu\r\n",
                (unsigned long)nfill, (unsigned long)nstuck, (unsigned long)nother);
        UART_SendString(Message);

        sprintf(Message, "gpu prod frows %lu first %lu\r\n",
                (unsigned long)nfrow, (unsigned long)first_frow);
        UART_SendString(Message);
    }

    sprintf(Message, "gpu prod r0 %04X %04X %04X %04X %04X %04X\r\n",
            (unsigned)((const volatile uint16_t*)GPU_SCALE_DST)[0],
            (unsigned)((const volatile uint16_t*)GPU_SCALE_DST)[1],
            (unsigned)((const volatile uint16_t*)GPU_SCALE_DST)[2],
            (unsigned)((const volatile uint16_t*)GPU_SCALE_DST)[3],
            (unsigned)((const volatile uint16_t*)GPU_SCALE_DST)[4],
            (unsigned)((const volatile uint16_t*)GPU_SCALE_DST)[5]);
    UART_SendString(Message);

    /* The source itself, so a bad fill cannot be read as a bad blit. */
    sprintf(Message, "gpu prod src %04X %04X\r\n",
            (unsigned)((const volatile uint16_t*)GPU_CROP_SRC)[0],
            (unsigned)((const volatile uint16_t*)GPU_CROP_SRC)
                      [(GPU_CROP_PITCH * (GPU_CROP_PROD_H - 1u))]);
    UART_SendString(Message);

    /* WHERE the correct pixels are, in the production pass - and it has to be
       here, while prod's output is still in the destination.  The map says how
       much of each row is right and this says which columns; rows 0..78 are pure
       stuck and are already known from the map. */
    crop_row_scan(80u);
    crop_row_scan(120u);
    crop_row_scan(160u);
    crop_row_scan(200u);
    crop_row_scan(240u);
    crop_row_scan(280u);
    crop_row_scan(310u);

    /* The same production pass over a source that is a coordinate rather than a
       constant, which is the only way to ask whether the scaled path reads the
       source AT ALL.  Under an all-FILL source a destination of 07C3 and a
       destination of "the value that was already there" are indistinguishable
       from "the value the sampler produced", because every input is the same
       value: a uniform field reads identically whether it was sampled or
       ignored.  A ramp is not uniform, so if the output still comes back as the
       constant it came back as under FILL - 07C3, per prod r0 above - then the
       scaled path never read the source and no count of a uniform field could
       have shown that.

       compare_src is false, so `bad` is meaningless here (the ramp is not FILL)
       and only F/S/N and the raw words are read.  The ramp is 7x + 13y, range
       0..11820, which excludes the 0xF81F colour key and GPU_CROP_SENT - so a
       word equal to either of those cannot have come from the source. */
    crop_fill_ramp();

    crop_clear_dst();
    crop_pass(GPU_CROP_SRC, GPU_CROP_PITCH, GPU_CROP_PROD_H,
              GPU_CROP_PITCH, psx, psy, false, &res);
    crop_report("ramp", &res);

    sprintf(Message, "gpu ramp r0 %04X %04X %04X %04X %04X %04X\r\n",
            (unsigned)((const volatile uint16_t*)GPU_SCALE_DST)[0],
            (unsigned)((const volatile uint16_t*)GPU_SCALE_DST)[1],
            (unsigned)((const volatile uint16_t*)GPU_SCALE_DST)[2],
            (unsigned)((const volatile uint16_t*)GPU_SCALE_DST)[3],
            (unsigned)((const volatile uint16_t*)GPU_SCALE_DST)[4],
            (unsigned)((const volatile uint16_t*)GPU_SCALE_DST)[5]);
    UART_SendString(Message);

    sprintf(Message, "gpu ramp r200 %04X %04X %04X %04X %04X %04X\r\n",
            (unsigned)((const volatile uint16_t*)GPU_SCALE_DST)[200u * GPU_SCALE_W],
            (unsigned)((const volatile uint16_t*)GPU_SCALE_DST)[(200u * GPU_SCALE_W) + 1u],
            (unsigned)((const volatile uint16_t*)GPU_SCALE_DST)[(200u * GPU_SCALE_W) + 2u],
            (unsigned)((const volatile uint16_t*)GPU_SCALE_DST)[(200u * GPU_SCALE_W) + 3u],
            (unsigned)((const volatile uint16_t*)GPU_SCALE_DST)[(200u * GPU_SCALE_W) + 4u],
            (unsigned)((const volatile uint16_t*)GPU_SCALE_DST)[(200u * GPU_SCALE_W) + 5u]);
    UART_SendString(Message);

    /* Now the two questions the counts cannot answer.

       First, how many values are in the destination at all.  A pass writes its
       rectangle; everything outside it keeps whatever was there.  Scanning the
       WHOLE 320x320 rather than the pass's region says how many distinct values
       are present and in what proportion.  The four named ones each have a
       source: FILL is the texture, STUCK is whatever the scaled path produces,
       SENT is the CPU's clear and therefore "untouched", and GPU_SCALE_BG is the
       colour handed to nsgpu_colorkey as both background and key - which has not
       once been observed in a destination, and if it appears here then the
       pre-fill inside the call is real and covers the ground between the blit
       rectangle and the source-extent rectangle.

       A 256x64 source at scale 128 makes those three rectangles different sizes
       on purpose: the blit rectangle is 128x32, the pre-fill rectangle is up to
       256x64, and everything past 256 columns or 64 rows was never written by
       anything but the CPU's clear.  So four probe points - inside the blit, in
       the pre-fill ring, outside both, and far outside - name the value in each
       zone in one line. */
    crop_fill_all();

    crop_clear_dst();
    crop_pass(GPU_CROP_SRC, 256u, 64u, GPU_CROP_PITCH, 128u, 128u, false, &res);
    crop_zone_scan();

    sprintf(Message, "gpu bg hit %04X pre %04X out %04X far %04X\r\n",
            (unsigned)((const volatile uint16_t*)GPU_SCALE_DST)[(28u * GPU_SCALE_W) + 120u],
            (unsigned)((const volatile uint16_t*)GPU_SCALE_DST)[(16u * GPU_SCALE_W) + 200u],
            (unsigned)((const volatile uint16_t*)GPU_SCALE_DST)[(10u * GPU_SCALE_W) + 300u],
            (unsigned)((const volatile uint16_t*)GPU_SCALE_DST)[(310u * GPU_SCALE_W) + 310u]);
    UART_SendString(Message);

    /* Second, is the constant the scaled path writes a value it read out of
       memory, or one it made up?  RAM plus 0x07C3 does not distinguish "the
       texture unit fetched from the wrong address" from "the library paints a
       constant", because a wrong address a long way off would give the same
       answer either way.

       So paint the memory on both sides of the source with something that names
       itself, and run the same two passes again.  If the output's constant
       becomes GPU_CROP_NEIGH, the texture unit is reading memory NEAR the source
       and the address it uses is wrong by a bounded amount - and the amount can
       then be measured.  If it stays 0x07C3, the value did not come out of this
       memory: it is either a constant in the library or a read of somewhere the
       fill does not reach, and those two are told apart by whether the driver's
       own area is involved. */
    crop_fill_neigh();

    crop_clear_dst();
    crop_pass(GPU_CROP_SRC, GPU_CROP_PITCH, GPU_CROP_PROD_H,
              GPU_CROP_PITCH, psx, psy, false, &res);
    crop_report("nprod", &res);

    crop_fill_ramp();

    crop_clear_dst();
    crop_pass(GPU_CROP_SRC, GPU_CROP_PITCH, GPU_CROP_PROD_H,
              GPU_CROP_PITCH, psx, psy, false, &res);
    crop_report("nramp", &res);

    /* THE COMPENSATED PASS - and the reason every pass above fails.

       The blit does not put its rectangle at (dst_startx, dst_starty).  It
       CENTRES it: the destination rectangle is the natural extent
       ((src_w * scale_x) >> 8 by (src_h * scale_y) >> 8) placed in the middle of
       an (src_w x src_h) box whose top-left corner is (dst_startx, dst_starty).
       So the image lands at an offset of ceil((src_w - nat_w) / 2) from where it
       was asked for, and for an 800x480 source at scale 103/171 that offset is
       (240, 80) - which is why prod writes only columns 240..319 over rows
       80..319 (80 x 240 = 19200, exactly prod's F count) and why everything
       outside that rectangle is still the library's 0x07C3 pre-fill.

       That one rule reproduces all ten scale-sweep counts, prod, ax/ay/axy and
       bg, with no exception - see the table in gpu.h.  At scale 256 the natural
       extent EQUALS the source extent, the offset is 0, and the image lands
       exactly where asked; that is why every 1:1 pass was bit-exact while every
       scaled one looked broken.

       So pass the offset back, negated.  dst_startx/starty are u32 and the
       library adds to them, so -off wraps to the value that cancels it, and the
       image should then cover the whole destination.  The expectation is
       F 102400 S 0 N 0 and a row scan of contiguous FILL from column 0. */
    crop_fill_all();

    {
        const uint32_t nat_w = ((GPU_CROP_PITCH * psx) >> 8);
        const uint32_t nat_h = ((GPU_CROP_PROD_H * psy) >> 8);
        const uint32_t off_x = (GPU_CROP_PITCH - nat_w + 1u) / 2u;
        const uint32_t off_y = (GPU_CROP_PROD_H - nat_h + 1u) / 2u;

        sprintf(Message, "gpu neg o%lu %lu n%lu %lu\r\n",
                (unsigned long)off_x, (unsigned long)off_y,
                (unsigned long)nat_w, (unsigned long)nat_h);
        UART_SendString(Message);

        crop_clear_dst();
        crop_pass_at(GPU_CROP_SRC, GPU_CROP_PITCH, GPU_CROP_PROD_H,
                     GPU_CROP_PITCH,
                     (uint32_t)(-(int32_t)off_x), (uint32_t)(-(int32_t)off_y),
                     psx, psy, false, &res);
        crop_report("neg", &res);

        crop_row_scan(160u);
    }

    return true;
}

bool gpu_scale_probe(uint32_t src, uint32_t src_w, uint32_t src_h,
                     uint32_t dst_startx, uint32_t dst_starty,
                     uint32_t scale_x, uint32_t scale_y, const char* tag)
{
    uint32_t x;
    uint32_t y;
    uint32_t minx = GPU_SCALE_W;
    uint32_t maxx = 0u;
    uint32_t miny = GPU_SCALE_H;
    uint32_t maxy = 0u;
    uint32_t painted = 0u;
    uint32_t holes = 0u;
    uint32_t saved_fb;
    uint32_t t0;
    uint32_t t1;

    if (!s_ready)
    {
        return false;
    }

    /* The whole probe is written in RGB565: 16-bit pixels, and a background
       value that is a colour.  gpu_init() falls back to L_8 if the driver
       refuses RGB565, and in that format every one of those assumptions is
       wrong - one byte per pixel, and 0xF81F truncated to 0x1F as the "colour".
       The scan would then walk off the end of the row and report a rectangle
       that means nothing.  The vendor's Convolve example does run in L_8, so
       this is not a theoretical branch; it just is not the branch this probe
       knows how to measure. */
    if (s_pixel_bytes != 2u)
    {
        UART_SendString("gpu scale SKIP (not RGB565)\r\n");
        return false;
    }

    scale_clear_dst();

    /* nsgpu_colorkey() repoints the driver's framebuffer at its destination
       and changes its size.  Only the address can be put back through the
       public API; the size cannot, so this runs after gpu_selftest() and
       before anything else that assumes the driver still holds 800x480 - see
       the call site in main.c. */
    saved_fb = nsgpu_get_framebuffer();

    t0 = DWT->CYCCNT;
    nsgpu_colorkey((u32*)GPU_SCALE_DST, GPU_SCALE_W, GPU_SCALE_H,
                   dst_startx, dst_starty,
                   (u32*)src, GPU_SCALE_BG, 255u,
                   src_w, src_h, src_w,
                   0u, scale_x, scale_y);
    t1 = DWT->CYCCNT;

    nsgpu_set_framebuffer(saved_fb);

    for (y = 0u; y < GPU_SCALE_H; y++)
    {
        for (x = 0u; x < GPU_SCALE_W; x++)
        {
            if (((volatile uint16_t*)GPU_SCALE_DST)[(y * GPU_SCALE_W) + x] !=
                (uint16_t)GPU_SCALE_BG)
            {
                painted++;
                if (x < minx) { minx = x; }
                if (x > maxx) { maxx = x; }
                if (y < miny) { miny = y; }
                if (y > maxy) { maxy = y; }
            }
        }
    }

    if (painted == 0u)
    {
        /* Nothing at all came back different from the background.  Either the
           call drew nothing or everything it drew was keyed out - and with the
           start at the origin that second one is not hypothetical, because
           everything it drew could equally have been clipped away.  The scale
           and the placement are the things to change, not the arithmetic. */
        sprintf(Message, "gpu scale %s NONE\r\n", tag);
        UART_SendString(Message);
        return false;
    }

    /* Background pixels strictly inside the painted box: keyed-out source
       pixels, or gaps the rectangle did not cover.  Either way they are the
       places where the destination is not the image. */
    for (y = miny; y <= maxy; y++)
    {
        for (x = minx; x <= maxx; x++)
        {
            if (((volatile uint16_t*)GPU_SCALE_DST)[(y * GPU_SCALE_W) + x] ==
                (uint16_t)GPU_SCALE_BG)
            {
                holes++;
            }
        }
    }

    sprintf(Message, "gpu scale %s box %lu %lu %lu %lu\r\n", tag,
            (unsigned long)minx, (unsigned long)miny,
            (unsigned long)maxx, (unsigned long)maxy);
    UART_SendString(Message);

    sprintf(Message, "gpu scale %s painted %lu hole %lu\r\n", tag,
            (unsigned long)painted, (unsigned long)holes);
    UART_SendString(Message);

    sprintf(Message, "gpu scale %s %lu %lu us %lu\r\n", tag,
            (unsigned long)scale_x, (unsigned long)scale_y,
            (unsigned long)((t1 - t0) / 600u));
    UART_SendString(Message);

    return true;
}

/* ==========================================================================
   DHD driver-level probe.  Boot only, additive, never in the trigger path.

   It answers the two questions the 2-texture-unit plan rests on, and neither
   has ever been measured on this board.

   --- 1. Is `a_handle` really the DAVE HD device? ---

   gpu.c already reads the library's global `a_handle` by NAME (see the long
   note above it) and the linker resolves it, so the open question was never
   "can we reach it" but "is its value a device".  dhd_init() returns
   dhd_handle_t (davehd_driver.h:834) and nsgpu_init() stores that return in
   a_handle, so it should be one - but "should be" is what the last two
   retractions in this file were made of, so it gets checked rather than
   assumed.

   The check is the vendor's OWN configuration file, not a guess:
   tes/kernel/src/davehd_registerparams_custom.h declares THIS chip as

       DHD_PARAM_MAX_TEXTURE_UNITS  2
       DHD_PARAM_MAX_COLOR_PASSES   3
       DHD_PARAM_NUM_CONST_COLORS   4

   and dhd_get_*_count() read exactly those fields out of the device
   description struct (davehd_kernel_init.c:125-127).  A handle that is not a
   device reads a valid address and prints numbers that do not match - so this
   cannot fault the board, and it cannot pass by accident either.

   These are real exported symbols: DHD_API_INLINE is defined empty unless
   DHD_INLINE_INTERFACE is set (davehd_driver.h:23-27) and the archive's linker
   symbol table has all three DEFINED.  A local prototype is therefore all this
   needs and gpu.c gains no #include - which is what keeps the whole change out
   of Printf.uvprojx's <File> list (see keil-uvprojx-clobber).

   --- 2. What does one draw cost? ---

   This is the number the decomposition turns on and it does not exist.  With
   two texture units a draw combines at most two input planes, so

       dense conv, any kernel <= 16x16 :  Cout * ceil(Cin/2)  draws
       depthwise conv                  :  Cin                  draws

   1x1 and dense 3x3 price identically - the 3x3's nine taps are free inside the
   texture unit, it is the channel sum that has to be unfolded.  The real model
   is 79 conv layers and **109 440 draws per inference**, 98.5% of them dense,
   the worst single layer a 1x1 288->72 at 20x20 = 10 368 draws for 28 800
   output pixels.  So the workload is draw-call bound, not pixel bound.  The
   only GPU timing on record is one 320x320 two-pass convolve at 62 ms - TWO
   draws - which cannot be extrapolated to 109 440.

   So: the same call, the same kernel, swept over sizes, modelled as

       T(n) = a + b*n^2

   `a` is the size-independent part: the CLUT allocation, the state
   programming, the job submit and the blocking sync that ONE call pays however
   few pixels it covers.  a/2 is therefore an upper bound on one draw's fixed
   cost - the term that decides whether 109 440 draws is a second or a minute.

   The identity kernel is used and must stay that: with any other kernel this
   would also be measuring the filter, and the filter has its own number (62 ms
   at 320x320, gpu_filter_probe).

   Both buffers are 320x320 and the sweep stays inside them at pitch = n, so no
   size reads past its own region.  The source is GPU_SCALE_DST because pass 1
   of the library runs IN PLACE on it - the buffer the convolve is documented to
   consume. */

extern uint32_t dhd_get_texture_unit_count(void* a_device);
extern uint32_t dhd_get_color_unit_count(void* a_device);
extern uint32_t dhd_get_const_col_count(void* a_device);

static const uint32_t s_dhd_sizes[5] = { 16u, 32u, 64u, 128u, 320u };

void gpu_dhd_probe(void)
{
    void*    dev = (void*)a_handle;
    uint32_t tex, col, cst;
    uint32_t i, n, t0, t1;

    if (!s_ready)
    {
        UART_SendString("gpu dhd SKIP (not ready)\r\n");
        return;
    }

    tex = dhd_get_texture_unit_count(dev);
    col = dhd_get_color_unit_count(dev);
    cst = dhd_get_const_col_count(dev);

    sprintf(Message, "gpu dhd h %08lx tex %lu col %lu cst %lu\r\n",
            (unsigned long)dev,
            (unsigned long)tex, (unsigned long)col, (unsigned long)cst);
    UART_SendString(Message);

    if ((tex == 2u) && (col == 3u) && (cst == 4u))
    {
        UART_SendString("gpu dhd handle OK (matches davehd_registerparams_custom.h)\r\n");
    }
    else
    {
        UART_SendString("gpu dhd handle SUSPECT (does not match the chip config)\r\n");
    }

    for (i = 0u; i < 5u; i++)
    {
        uint32_t saved_fb;

        n        = s_dhd_sizes[i];
        saved_fb = nsgpu_get_framebuffer();

        t0 = DWT->CYCCNT;
        nsgpu_separable_convolve_k(0u, GPU_CONV_TAPS, 1.0f, 0, s_probe_coeff,
                                   (u32*)GPU_SCALE_CONV, n, n, 0u, 0u,
                                   (u32*)GPU_SCALE_DST, n, n, n,
                                   s_conv_kernel);
        t1 = DWT->CYCCNT;

        nsgpu_set_framebuffer(saved_fb);

        sprintf(Message, "gpu dhd draw %lu px %lu us %lu\r\n",
                (unsigned long)n, (unsigned long)(n * n),
                (unsigned long)((t1 - t0) / 600u));
        UART_SendString(Message);
    }
}

/* ==========================================================================
   Depthwise convolution on the texture unit.  Bring-up probe, boot only,
   additive, never in the trigger path.

   The plan this implements is in the project's plan file; the short version is
   in gpu.h above gpu_dw_probe().  What is worth recording here is where each
   part of the call sequence comes from, because none of it is documented and
   the cost of deriving it twice is what this comment is for.

   --- the sequence is copied from the vendor's own blit_convolve ---

   nsgpu_convolve() cannot carry this model's weights (unsigned, normalised to
   the largest tap, one scalar gain for the whole image - see gpu.h), but the
   PRIVATE function it is a thin front end for does exactly the right thing, and
   it is in the disassembly at nsgpu_lib_dis.txt:28552.  So the sequence below
   is not designed, it is transcribed, argument by argument:

       dhd_alloc_clut(dev, w*h, &base)                 w=(kw+1)>>1, h=(kh+1)>>1
       dhd_set_uv(dev, 0, &uv)
       dhd_active_texture_unit(dev, 0)
       dhd_set_texture_address(dev, src, 0)
       dhd_set_texture_size(dev, tw, th, pitch)
       dhd_set_texture_format(dev, fmt, addr, 0)       fmt = texel | 0xA0 signed
                                                       addr = 14 (kh==3), 22 (kh>=4), 0
       dhd_set_texture_kernel(dev, kw, kh, k, base)
       dhd_set_texture_scalebias(dev, scale, bias)
       dhd_set_texture_wrap(dev, mode, mode)
       dhd_set_bbox(dev, xmin, ymin, xmin+tw-1, ymin+th-1, ymin)
       dhd_draw(dev, 0, 0, 0)                          no edges: the bbox fills
       dhd_wait(dev, 0x1f8)
       dhd_free_clut(dev, w*h, base)

   Two of those are worth calling out because they were open questions and the
   library settles both:

     - **`dhd_draw(dev, 0, 0, 0)`** - all three arguments zero, no edge and no
       edge count, and the primitive is the bounding box.  gpu.h listed "does
       the no-edge idiom work, or does it need four edges?" as a risk to
       measure.  It does not need measuring: this is what the vendor ships, and
       the only probe here that uses it is one whose output is checked against a
       CPU reference, so it would be caught anyway.

     - **the UV struct is normalised, and `m_dux = 1/tex_w`, `m_dvy = 1/tex_h`**
       - one destination pixel advances one texel, expressed as a fraction of
       the texture.  The cross terms are 0.  `m_u0` is one texel's worth
       (`1/tex_w` in the library's arithmetic), which is the half-texel centre
       convention; the probe's CPU comparison is what confirms it, since a
       wrong `m_u0` shifts the whole result and a random source makes any shift
       loud.

   --- what is NOT copied: the CLUT lifetime ---

   blit_convolve allocates the CLUT, uploads one kernel, draws once and frees
   it - which is why the nsgpu_* wrappers cannot pipeline and why gpu_dhd_probe
   measured 57 us per draw.  A depthwise layer has 24 to 96 channels sharing one
   kernel geometry, so this path allocates once per LAYER, packs every channel's
   taps into one array and uploads them in a single dhd_set_clut, then re-points
   dhd_set_texture_clut_base per channel.  dhd_set_clut's own DHD_STCOP_WAIT
   (davehd_driver.c:879) drains the pipeline, so the difference is one drain per
   layer instead of one per channel.  That is the whole reason the 512-entry
   CLUT matters and the reason the old 57 us/draw figure does not apply here.

   The probe below does not yet batch - it uploads per pass, because each pass
   has a different kernel - but it does hold the CLUT across all of them, which
   is the part being measured. */

/* No local prototypes are needed here after all, and the correction is worth
   recording because the opposite was assumed and written down before the
   compiler disproved it: **davehd_driver.h is already in gpu.c's translation
   unit**, pulled in transitively through nsgpu.h.  gpu_dhd_probe()'s local
   prototypes for dhd_get_*_count worked only because dhd_handle_t IS void* and
   the signatures happened to match; adding more of them produced twelve
   "conflicting types" errors against the real declarations.  So the real types
   and the real enum names are used from here down, which is strictly better:
   E_DHD_TEXEL_ORG_AL_8 | E_DHD_CLUT_KERNEL_SIGNED cannot go stale the way a
   copied hex value can, and the value written in gpu.h as 0x80AA is now
   derived by the compiler rather than by me.

   The one thing that does NOT change: dhd_gpu_ptr_t is `void*`, so every
   address below needs an explicit cast from the uint32_t constants in gpu.h. */

/* A 1-LSB bound, the same discipline as GPU_CONV_GAP_MAX: the hardware rounds
   its accumulator once per output byte and the CPU reference below rounds in
   integer arithmetic, so the two can differ by one and that is not a fault.
   Anything above it is.  (Not the same constant as GPU_CONV_GAP_MAX - that one
   bounds the nsgpu_* wrappers' 8-bit weight normalisation, which this path
   does not have: the model's weights are already int8 and go in unchanged.) */
#define GPU_DW_GAP_MAX      1u

/* The two texture formats a CLUT kernel can be selected with (davehd_driver.h:
   241-242).  Named here because the difference between them is a question the
   probe asks rather than a constant it knows: E_DHD_CLUT_KERNEL_SIGNED is the
   signed weight mode, E_DHD_CLUT_KERNEL the unsigned one.

   An earlier note here had the library using the UNSIGNED one on the strength of
   a disassembly read as `framebuffer_format | 0x80`.  Reading the driver's own
   shadow state back at runtime says otherwise - `gpu cv2 t0 fmt 80A6` - and 0xA0
   >> 5 is 5, so it is the SIGNED mode.  The constant is 0xA0, not 0x80; one bit
   of that transcription cost three flash rounds. */
#define GPU_DW_FMT_SIGNED   ((dhd_texel_org_t)(E_DHD_TEXEL_ORG_AL_8 | \
                                               E_DHD_CLUT_KERNEL_SIGNED))
#define GPU_DW_FMT_UNSIGNED ((dhd_texel_org_t)(E_DHD_TEXEL_ORG_AL_8 | \
                                               E_DHD_CLUT_KERNEL))
/* RGB565 texels with the UNSIGNED kernel mode - the pairing that the WRONG
   reading of the library's constant implied.  Kept so the unsigned mode stays
   measured against a format known to work, rather than only against AL_8, where
   it returns 0 for reasons that have nothing to do with the sign. */
#define GPU_DW_FMT_565      ((dhd_texel_org_t)(E_DHD_TEXEL_ORG_RGB_565 | \
                                               E_DHD_CLUT_KERNEL))
/* GPU_DW_FMT_SIGNED_565 is defined beside GPU_CONV_TAPS at the top of this file,
   because stage 3 (filter_frame_with()) needs it for its source texture and is
   emitted long before this point.  The comment that belonged here travelled with
   it. */
#define GPU_DW_FMT_INDEXED  ((dhd_texel_org_t)(E_DHD_TEXEL_ORG_AL_8 | \
                                               E_DHD_CLUT_INDEXED_COLOR))
/* 24-bit texels, 8 bits per channel (davehd_driver.h:271, "memory byte order").
   The one format in the whole list that gives a full 8 bits per channel, which
   is what an int8 activation needs - and it is also the one the sibling format
   AL_8 would lead you to distrust, since AL_8 is documented as "a single value
   for all channels" and is precisely the format that returns 0 under a CLUT
   kernel.  So this is a question, not an assumption: 0x80 in a channel is 128,
   which no 5-bit RGB565 channel can express, and that is what the probe reads. */
#define GPU_DW_FMT_888      ((dhd_texel_org_t)(E_DHD_TEXEL_ORG_RGB_888 | \
                                               E_DHD_CLUT_KERNEL_SIGNED))

/* The two 32-bit texels that carry an 8-bit alpha alongside 565 colour
   (davehd_driver.h:272-273).  They exist because the CLUT kernel path's
   multiplicand is only 5 bits wide - one centre tap of one over an RGB_888 blue
   byte reads 0x40 as 8, 0x80 as 16 and 0xFF as 31, i.e. blue >> 3 - and 32
   levels is 3 bits short of what an int8 activation needs.

   RGB_888 cannot fix that: it is 8 bits per channel ON THE WAY IN but the
   kernel still takes the top 5.  An alpha byte is the one part of a texel the
   blue-field rule says nothing about, so it is the one place an 8-bit
   multiplicand could be hiding.  These two names differ only in which end of
   the 32-bit word the alpha sits, and the measurement writes the same pattern
   at both ends for both of them rather than guessing the byte order. */
#define GPU_DW_FMT_565A8    ((dhd_texel_org_t)(E_DHD_TEXEL_ORG_RGB565_A8 | \
                                               E_DHD_CLUT_KERNEL_SIGNED))
#define GPU_DW_FMT_A8565    ((dhd_texel_org_t)(E_DHD_TEXEL_ORG_A8_RGB565 | \
                                               E_DHD_CLUT_KERNEL_SIGNED))

/* Test 3's geometry.  32x32 gives 900 interior pixels to compare, which is
   enough that a systematic error cannot hide in it, and small enough that the
   uncached byte loop over GPU_DW_SRC costs nothing worth timing. */
#define GPU_DW_T3_W         32u
#define GPU_DW_T3_H         32u

/* There is deliberately no "pick the address mode from the kernel height"
   helper here any more.  blit_convolve really does pass 14 (kh==3) and 22
   (kh>=4), but those are E_DHD_ACCESS_VIRTUAL_TILED_2 / _TILED_4 - "virtual
   tiled 2/4 bursts per cacheline" (davehd_driver.h:474-475), which describes
   the layout of ITS texture, not a requirement of the kernel.  The library
   convolves a GPU framebuffer, and nsgpu_* writes tiled; our plane in the GPU
   slot is plain linear, so the vendor's number is about their buffer and
   copying it would be copying the wrong fact.  The doc note that constrains the
   mode is "the texture address mode has to be E_DHD_ACCESS_VIRTUAL_TILED_4 or
   higher when the kernel WIDTH is 1" - a separable-kernel rule that a 3x3 or
   5x5 does not fall under.

   LINEAR is therefore the honest default for what we actually store, and
   gpu_dw_probe() measures it against the vendor's TILED_2 rather than assuming
   it: the constant source in test 1 cannot tell the two apart (every byte is
   0x80, so any layout reads 0x80), which is exactly why the answer has to come
   from the noise field in test 3. */

/* --- the stall probe: where this operator was when it stopped ---------------

   Written for one reading, and the reason it exists is a specific blind spot.
   From `gpu dw call` (printed once per session) and the two stride-class checks
   (once per class) onward, a run that dies inside this operator prints NOTHING
   AT ALL - not because nothing is instrumented, but because every reading that
   could say where is taken either before the layer or after it.  Two logs of
   two different deaths read as one death at one place for exactly that reason.
   The trace in yolo_inference.cpp is the other half: it prints `gpu dw L<n>`
   on entry and `gpu dw L<n> drew` on the way out, so the pair says whether the
   operator finished at all.

   This half names the STAGE.  It is a single integer, written at each stage of
   each channel, plus a pixel index inside the two fold loops, and it is read by
   the DVP interrupt handler, which reports it when it has not moved for about a
   second.  Nothing is printed from here - the writes are a store each - because
   the reporter has to be something the failure does not stop, and a printf
   inside the failing code is the one thing that cannot be.

     stage 1   past the guards, in the per-channel weight check
     stage 2   the UV setup / texture unit select
     stage 3   after dhd_set_clut()
     stage 4   after the layer's texture setup
     stage 5   after the framebuffer save and the plane-size set
     stage 10  before dw_pack_plane(), pass 1
     stage 11  after  dw_pack_plane(), pass 1
     stage 12  before dw_pass_bc(), pass 1
     stage 13  after  dw_pass_bc(), pass 1
     stage 14  the fold loop, pass 1  (per pixel)
     stage 15  before dw_pack_plane(), pass 2
     stage 16  after  dw_pack_plane(), pass 2
     stage 17  before dw_pass_bc(), pass 2
     stage 18  after  dw_pass_bc(), pass 2
     stage 19  the fold loop, pass 2  (per pixel)
     0         not inside a draw - a refusal or any other exit leaves this here,
               and the reporter ignores 0 so a refused layer cannot be mistaken
               for a stalled one.

   And 38..39, which are NOT depthwise at all and are here because the front end
   had no stage of its own: gpu_scale_frame() arms 38 and filter_frame_with()
   arms 39 across the WHOLE of their draw, so a hang in the blit now prints
   `STALL s38 c0 p0` and reaches the eight-second reset instead of taking the
   board down silently.  Measured on 2026-09-24: without it, a hang out there
   produced no STALL line and no reset, and the board needed a power cycle.
   `gpu dw call`-style claim to keep: an `s38`/`s39` can only come from the front
   end, and the front end never runs during an inference.

   And 6..9 and 20/21, the BOOT PATH's, which were the last ones left unarmed:
   gpu_init() arms 20 and 21 - its two nsgpu_init() attempts, RGB565 and then the
   L_8 retry - and gpu_selftest() arms one stage per leg (6 fill, 7 copy, 8
   identity convolve, 9 shift convolve) instead of one for the function, so the
   line names the attempt, or the leg, that never returned.  All six are unbounded
   library commits (`nsgpu_init` / `nsgpu_data_fill` / `nsgpu_data_copy` /
   `nsgpu_separable_convolve` -> `nsgpu_cmd_sync` -> `dhd_finish`, every one read
   out of nsgpu_lib_dis.txt or Printf.map) and nothing armed them before, so a boot
   wedge was silent, produced no reset, and left `k` unreachable - a power cycle on
   every power-up.  See the block over gpu_selftest() for why arming at boot is
   valid at all (the DVP must already be running or the reporter cannot see it)
   and for why those waits cannot be bounded.

   HOW TO PICK A STAGE, because this fix got it wrong twice and both ways are
   traps a reader would repeat:

     - 40..44 ARE TAKEN, by the PRODUCTION CONV PATH (`DW_PROG(40u, co, 0u)` on
       the conv layer's per-channel entry, 41/42 in its chunk loop, 43 later -
       gpu.c:9502, 9693, 9701, 9732, 11336).  `1..37` above was written before
       that path was instrumented; the live set is 1..5, 6..9 (the self test),
       10..19, 20/21 (gpu_init), 30..39 (30..37 depthwise fine grain, 38/39 the
       front end), 40..44.
       A front end on 40 prints the same `STALL s40 c0 p0` the conv path can, and
       naming WHICH code stopped is the whole point of this word.
     - 43 AND UP OVERFLOW.  `100000000u` is a 32-bit unsigned, so `stage * 1e8`
       wraps above 4294967295 - i.e. from 43 (`4.3e9`) on.  The existing
       `DW_PROG(43u, co, 0u)` and `DW_PROG(44u, co, 0u)` therefore decode as
       `s0`/`s2`, not as 43/44, and the `% 100u` in both decoders does not save
       them because the damage is in the top bits.  A front end put at 45 would
       have printed `STALL s2 c0 p0` - the depthwise UV stage.  The last stage
       that encodes correctly is 42.

   So the rule is: grep BOTH `DW_PROG(` and `gpu_front_prog(` for the live set -
   the second one is how 6..9 and 38/39 are written and the first does not match
   them, which is exactly how a "free" number gets taken twice - stay at or below
   42, and pick a number nothing else uses.  6..9, 20/21, 38 and 39 are all such
   numbers.

   And 30..38, which is the finer grain and the reason the coarse one above is
   not enough: those are inside dw_pass_bc(), one per driver call, because
   "stalled in the draw" is not an answer - every one of these calls appends to
   the driver's command stream, appending is what commits the stream, and
   committing is the only thing in the driver that can block without a panic.
   `s30` is the texture setup, `s32` the CLUT upload, `s36` dhd_draw() and
   `s37` dhd_wait() - and the last one printed is the call that never returned.

   Encoding, 1 + stage*100000000 + channel*100000 + pixel.  The reporter in
   n32h7xx_it.c decodes it with that arithmetic in reverse. */
volatile uint32_t g_gpu_dw_prog = 0u;

/* Which channel the dw_pass_bc() call now in flight belongs to.  That function
   is also the probes' and has no channel of its own, so the caller has to lend
   it one - otherwise the fine-grain stages would replace the channel as well as
   the stage, and `s36 c0` would not say whether it was the first draw of the
   layer or the ninetieth. */
static uint32_t s_dw_prog_ch = 0u;

#define DW_PROG(stage, ch, px) \
    (1u + ((uint32_t)(stage) * 100000000u) + ((uint32_t)(ch) * 100000u) + \
     (uint32_t)(px))

/* --- the drain, and why it is not dhd_finish() ------------------------------

   Every draw here has to be waited for, and not for tidiness: the fold loops
   read the plane the draw just wrote, and dw_pack_plane() overwrites the source
   plane the draw was reading.  Both need the GPU to have finished.

   The library's way to wait is dhd_finish(), and that is what
   nsgpu_set_framebuffer() calls - nsgpu_set_framebuffer -> nsgpu_cmd_sync(0) ->
   dhd_finish, read out of nsgpu.lib's own disassembly.  It is also where this
   firmware was hanging.  The stall report reads `STALL s37 c0 p0` on L7 and
   again on the next boot at the same layer, stage 37 is the last marker in
   dw_pass_bc()'s tail, and nsgpu_set_framebuffer() is the last statement of
   that tail.

   dhd_finish() ends in dhd_gpu_wait_for_job(), which polls the driver's CACHED
   copy of the hardware sync id until it passes the job's assigned id.  That
   cache is written by the GPU interrupt only (davehd_kernel_irq.c:58), and the
   loop around it is bounded by DHD_GUARD_WAIT_FOR_JOBS, which is
   DHD_GUARD_MAX_VALUE = 4294967295 - i.e. unbounded.  Printf.map has no
   _DHD_GUARD_* symbol, so those release constants were compiled in and cannot
   be changed from outside.  The driver is prebuilt inside nsgpu.lib, so the loop
   cannot be given a timeout, and dhd_isbusy_marked() - the one non-blocking
   completion query the API offers - is a stub in this release: it calls
   E_DHD_CALL_GET_SYNC, discards the answer and returns DHD_FALSE
   unconditionally.  When a job never reports done, every route through the
   library ends in a spin that only a reset leaves.

   So the wait is done here, against the hardware's own register.  Every
   scheduled job's stream ends with

       WRITE DHD_REG_STC_SYNC_ID_1 = job->m_assigned_id
                                        (davehd_kernel_schedule.c:171-178)

   emitted for EVERY job, after the job's own commands (including the writeback
   wait), and DHD_REG_STC_SYNC_ID_1 is register index 0x0C of the STC aperture
   read as a word index (arch_read_reg, platform_config_ns.c:20), i.e. byte 0x30
   of 0x50040000.  The SoC header agrees from the other side: GPU_REG_BASE is
   0x50040000, and its GPU_STC_TypeDef puts Sync_ID_0 at 0x30 while
   davehd_stcregistermap.h calls the 0x0C index "used for jobid".  So the
   register moves only when the job just committed has really finished - which
   is the same event dhd_finish() waits for, only readable without the driver.

   The loop is bounded, so a wedged GPU now costs a refusal instead of the
   product - which is the hard rule here. */
#define GPU_STC_SYNC_ID (*(volatile uint32_t*)(0x50040000u + 0x30u))

/* --- THE FETCH GATE, added 2026-09-28 (replaces the busy gate) -------------
   The STC dump (see gpu_stc_dump) named the wedge: the block is NOT waiting, it
   STOPPED, and `Interrupt_Status` says why - bits 8..10 are `m_stop_code` and
   the value is 2, `E_STOP_IRQ_CODE_START_WHEN_BUSY` (davehd_registertypes.h:369,
   "start signal received while still busy").  The stopped stream was this
   drain's own four-word batch: `str == curring == str+0` with `rpause` exactly
   16 bytes on, i.e. the fetcher never consumed one command of it.

   Read against the driver, that state has one owner (davehd_kernel_ring.c:193):
   `dhd_gpu_uploadring()` writes RING_PAUSE_ADDRESS always, and writes
   STREAM_ADDRESS - the start signal of that stop code - only
   `if (m_status != E_DHD_STATUS_ACTIVE)`.  `m_status` goes back to IDLE on the
   stop IRQ alone (davehd_kernel_irq.c:94-100).  So the driver's "I am idle" and
   the hardware's "I am busy" can disagree, and a start write landing in that
   window stops the STC for the rest of the boot.

   THE FIRST VERSION OF THIS GATE WAITED FOR `Busy_Status == 0`, AND ITS OWN
   COUNTERS FALSIFIED IT IN ONE BOOT: `gate n94665 w94664 to94664 mx20000` -
   94664 of 94665 drains found Busy_Status nonzero and 94664 of them used the
   whole limit.  The only drain that ever saw a 0 was the boot's first, before
   the engine had been started at all.  **Busy_Status bit 0 reads "the STC is
   running", not "a unit is busy"**: it is 1 for the whole life of the stream,
   including the parked-at-the-pause-address state, and 0 only before the first
   start or after a fault.  So the condition was unreachable, and the gate's only
   real effect was a fixed 1.354 ms delay before every commit, at ~2.8 s per
   frame (2083 drains/frame x 20000 polls x 67.7 ns - measured `infer` 3.7 s ->
   6.9..7.1 s).  The earlier reading of the control dump's `busy00000001` as
   "still busy with the batch's tail" was over-read for the same reason.  And the
   ONE drain that wedged with the gate in place was a drain whose gate saw
   Busy_Status 0 (`n` = `w` + 1), so the condition was not merely unreachable -
   in the one place it applied it was wrong.

   THE CONDITION THIS ONE USES IS THE ONE THE TWO DUMPS ACTUALLY DIFFER IN.
   Control (a drain that answered): `curring == rpause` (0xC0C00188 both).
   Wedged: `curring == str` (0xC0C0012C) with `rpause = str + 16` - NOT equal.
   That equality means "the fetcher has consumed every command the driver has
   allowed it", i.e. the STC is parked at the pause point rather than mid-stream,
   which is the state a start signal is safe to land in.  It is also the
   relationship the driver's own ring upload waits on before it writes
   (davehd_kernel_ring.c:150-165).

   AND IT IS THE PRINCIPLED FORM OF WHAT THE BUSY GATE ACCIDENTALLY DID: that
   boot survived ~44 frames where the two before it wedged within 1-3, and the
   only thing the gate did on every one of those frames was delay the commit by a
   constant 1.354 ms - a settle.  This gate settles by the same route, but it
   ends as soon as the fetcher catches up (the draw's own command-fetch time, not
   a constant), so it should cost a fraction of that.  n = 1 for the survival
   observation and it is NOT attributed to the gate; the four counters below are
   what will say whether this one is another knob that is not being set - the
   failure mode the last one demonstrated in its own first boot.

   WHAT THE FIRST BOOT OF THIS GATE READ, AND WHAT IT CHANGED HERE.  `Code=201246`:

       gate n298 w1 to1 mx2000

   297 of 298 drains found the fetcher already at the pause address.  So unlike
   the busy gate this condition is REACHABLE AND NEARLY FREE - and `to == w` says
   the one drain that did wait spent the whole limit, i.e. the limit is what ended
   that wait, not the condition.  That the wedge still happened on the same boot
   means one of two things, and the whole reason for the two additions below is
   that they are distinguishable and the counters above are not:

     (a) THE WAITING DRAIN WAS THE ONE THAT WEDGED, and 0.14 ms is simply far too
         short for a draw's tail - then the gate's premise is right and its number
         is wrong, and raising the limit is the fix;
     (b) THE WAITING DRAIN WAS SOMEWHERE ELSE (the front end's first drain, say)
         and the wedge's own drain saw `curring == rpause` IMMEDIATELY - then the
         start write landed with the fetcher parked at the pause point and the
         hardware still refused it, i.e. `START_WHEN_BUSY` is about something the
         fetch pointer does not show (a writeback, a unit, the pipeline) and this
         condition is the wrong quantity the way `Busy_Status` was.

   HOW THE TWO ARE SEPARATED, with no extra polls and no guessing:

     `gw%u` on the `drain dead` line   THE FAULTING DRAIN'S OWN gate wait, in
         polls, read out of s_gate_last.  `gw0` at the give-up is (b), and it is
         the whole answer: the fetcher was parked and the start was refused
         anyway.  A `gw` equal to the limit is (a).
     `gpu dw gate w%u sy%u`            printed the first two times any drain
         actually waits, carrying that drain's sync id.  The gate runs before the
         flush, so the line lands before the `dead` line it may explain, and `sy`
         places it in boot order - the front end's drains are `sy` ~6, the fatal
         draw was `sy324`.

   THE LIMIT IS RAISED on the strength of those two readings, and it is the cheap
   direction to be wrong in: the gate is satisfied on 297 of 298 drains, so 100000
   polls (~6.8 ms) is paid only on the rare drain that needs it, while the old
   2000 could not outlast even one draw's tail.  THE PER-BOOT BUDGET is what keeps
   that bounded, because the 297/298 is one boot and this gate's predecessor died
   by assuming its own frequency: once GPU_FETCH_GATE_BUDGET polls have been spent
   in total the gate stops waiting and every drain from then on is COUNTED AS A
   TIMEOUT - so a gate that has stopped running can never be invisible in the `to`
   field, which is the failure mode this codebase has now met twice.  `sp` on the
   summary line is the total actually spent, i.e. the measurement, and `gw`/`w`/`sy`
   are the diagnosis. */
#define GPU_STC_CURRING (*(volatile uint32_t*)(0x50040000u + 0x24u))
#define GPU_STC_RPAUSE  (*(volatile uint32_t*)(0x50040000u + 0x20u))
#define GPU_STC_IRQ  (*(volatile uint32_t*)(0x50040000u + 0x18u))
#define GPU_STC_BUSY (*(volatile uint32_t*)(0x50040000u + 0x10u))
#define GPU_STC_STR  (*(volatile uint32_t*)(0x50040000u + 0x1cu))
#define GPU_FETCH_GATE_LIMIT 100000u
#define GPU_FETCH_GATE_BUDGET 400000u

/* --- AND `Busy_Status` IS NOT A FLAG - IT IS A TEN-BIT BUSY VECTOR ---------
   `Code=201246` gave the gate the boot it needed and the answer is NO:

       gpu dw drain dead #0 s37 c0 p0 n20000000 cy819221601 sy10255>10255 sc2 gw0

   `gw0` is the fatal drain reading its OWN gate before the commit that killed
   it: `Current_Ring_Address == Ring_Pause_Address` at the first test, i.e. the
   fetcher was parked exactly where the driver had allowed it, with the batch
   still ahead of it.  THE FETCH POINTER DOES NOT PREDICT THE FAULT, the same way
   `Busy_Status` bit 0 alone did not, and for the same reason: both conditions
   were read as "the engine is at rest" when what the hardware means is "the
   stream is alive and waiting".  The one wait in the whole boot was the REDRAW's
   (`gpu dw gate w100000 sy10255`), and that one is meaningless by construction -
   after a START_WHEN_BUSY stop the fetcher is frozen at the start address, so
   `curring` can never again equal `rpause` and the gate can only burn its cap.

   So the next discriminator has to come from something the fetch pointers cannot
   show, and there is exactly one such thing in the block: this word, which
   davehd_registertypes.h:789-804 decomposes as ten independent bits -

       bit0 stc    bit1 str    bit2 stw    bit3 reg    bit4 psu
       bit5 txi    bit6 tex    bit7 col    bit8 blend  bit9 clear

   - and the two readings that exist say:

       `gpu stc ok busy00000001`   the HEALTHY parked stream is bit0 ALONE: the
           fetcher has consumed everything the driver allowed and is waiting, and
           every unit downstream is idle.  So "something is busy" is the NORMAL
           state of a live stream, and the whole-vector-zero condition the first
           gate waited on can only ever hold after the STC has actually stopped -
           which is why that gate's counters read `to == w == n - 1`.
       `gpu stc dead busy00000000`  after the fault, nothing is busy: the STC
           stopped itself and the units drained.

   WHAT THIS BUILD MEASURES, and why it is two numbers and not one:

     `b%08X` on the `drain dead` line   the vector AS IT WAS when the fatal start
         was written - read one instruction before the commit, printed after it.
         A `b00000001` there says the start was refused with nothing but the
         stream's own liveness on the bus, i.e. the fault is "a start landed on a
         live stream", full stop.  A vector with UNIT bits in it says the start
         lost a race with work still draining and a settle would fix it.  Neither
         has ever been observed.

     `strt%u nz%u sor%03X` on the summary line   THE DENOMINATOR, and without it
         the first number cannot be read: STREAM_ADDRESS is written only when the
         driver's `m_status` is not ACTIVE, so counting how often it MOVES (read
         before the flush, read after it) counts the commits that really wrote a
         start - and `nz`/`sor` are how many of those had a busy vector and which
         bits.  If every commit writes a start, then a busy vector at a start is
         the ordinary case and the fatal drain needs a different discriminator;
         if starts are rare, that histogram IS the signal.  The count can be low
         by ~1% when the ring wraps round to the same address, which cannot change
         either answer. */


/* See the declaration in gpu.h for why this is a variable and not an #ifdef.

   OFF BY DEFAULT, and this is a MEASURED decision rather than the "unproven"
   one that set it originally.  It was flipped ON for one round, on the
   argument that an offload which only happens when somebody presses a key on
   every boot is not an offload the product has.  A single boot then falsified
   that: the product's own reading of one static scene, taken three times with
   only these switches moved between them -

       dw ON  conv ON   38 dw passes + 18 conv layers   pair cpu: 吉CC      pair gpu: 黑          c4 g1   infer 5.03 s / 4.35 s
       dw ON  conv OFF  38 dw passes                    pair cpu: 2        pair gpu: 25         c3 g6   infer 4.12 s / 4.19 s
       dw OFF conv OFF  nothing offloaded               pair cpu: 粤8888888 pair gpu: 8粤粤8888888 c9 g11  infer 1.71 s / 1.77 s

   The box count on the GPU-grab arm climbs 1 -> 6 -> 11 as the offloaded layer
   count falls, both arms read a well-formed plate only at zero, and the two
   offloads are therefore worth a working product.  That is a dose-response in
   the layer count, not a frame artefact - and it says the per-layer gate is
   measuring the wrong thing: lim12 bounds ONE layer's error, while the product
   sees the error of 38 passes and 18 layers in series, each one feeding the
   next.  Fix that (the systematic per-unit replication-gain residual, gpu.h's
   COST note and the CV comments) and this default can go back to 1; until it
   is fixed, the keys still turn the offloads on for a demo with no flash. */
volatile uint32_t g_gpu_dw_on = 0u;

/* THE `n` KEY: keep the driver ACTIVE while the STC is alive, so that a commit
   never writes a start into a live stream.  ON by default since 2026-09-28, and
   it is the one switch here that WRITES driver state rather than hardware state -
   see the extern block beside dhd_gpu_stream_status for the reading that motivates
   it (`strt1`: one start in 1414 commits, and that one commit is the wedge) and
   for its failure mode, which is a drain timeout and the same latch, not a hang.

   THE DEFAULT WENT TO 1 ON THE MEASUREMENT, not on the argument, and it is the
   one default here that did NOT go the way the other three did.  With it off: 4
   of 4 boots wedged, every one on the first frame.  With it on: 18 frames, 37594
   drains, 29 starts, zero wedges, and all 3 times it fired the following drain
   succeeded - so the live-parked stream does resume on a pause advance, which is
   the half of the mechanism that had to be true for this to be a repair rather
   than a delay.  It costs about +0.2 s of infer (3.90 vs the 3.72 s dw-on-GPU
   figure); the key still turns it off, which is now the way to reproduce the
   wedge rather than the way to avoid it. */
volatile uint32_t g_gpu_keep_active = 1u;

/* The per-layer depthwise trace's switch - the pair of lines above, not the
   operator.  OFF, and it is the ONLY one of these switches whose default is a
   cost decision rather than a correctness one: the trace changes no result.

   It started as automatic (print once per boot, on the first inference after
   `g` goes on, detected by the layer ordinal going backwards) to stop 38 lines
   per frame from landing inside the interval the `gpu dw L.. D.. R..` summary
   reports.  THAT TRADE WAS WRONG and the reason is the measurement it was
   supposed to protect: the trace's cost came out anywhere between 0.2 s and
   1.1 s a frame depending on which cross-boot comparison is used, and the two
   readings cannot be reconciled because the trace was WELDED to the frame that
   also runs the two class checks - ~1.16 s on its own - so the one frame it
   ever printed on was never a clean one.  An instrument whose cost cannot be
   measured on the frame it runs on is not an instrument, and re-arming it cost
   a power cycle, so it could not be A/B'd on one boot either.

   So: a key, like every other switch here, and no frame-boundary logic at all.
   Press `y` and every frame prints its 19 pairs until it is pressed again -
   which makes the trace's cost measurable on one boot, on one scene, in
   interleaved frames, with the checks already latched out of the way.

   gpu.h carries the full note.  The storage is here because gpu.c already owns
   the other three switches and the drain's own prints. */
volatile uint32_t g_gpu_dw_trace = 0u;
/* The front end's default is 1 as of 2026-10-08.  Both of the blocks that held
   it at 0 were retired by measurement, not by argument.

   The first one read "the one of the three that is still BROKEN: on the same
   camera frame `pair cpu:` reads characters and `pair gpu:` reads none".  That
   reading was taken while nsgpu_draw_rotblit was clipped to one texel, so it is
   VOID - the same fix that closed the gate gap (`dhd_set_util_cliprect(0,0,W-1,H-1)`)
   reversed it, and the post-fix boot probe is `ok 1` on both legs (`mean 4`,
   `filt mean 5`) with `pair diff mean` 6.

   The second one was not a picture fault but an UNBOUNDED one: with this switch
   on and `s` off, the product's filter leg is the LIBRARY's
   `nsgpu_separable_convolve`, whose commits are `nsgpu_cmd_sync(0)` =
   `dhd_finish()`, so a latched-dead GPU would hang the whole product inside it.
   Stage 38 was given its own bounded commit point first (the transcription at
   gpu_scale_frame()); stage 39 has one now too, and as of 2026-10-09 on BOTH of
   its states - `s` = 1 is this repo's blit_separable_convolve_new(BLIT_SEP_ROWCOL)
   committed by blit_tile()'s bounded gpu_drain(), and `s` = 0 is now
   filter_frame_lib(), the library's own operator TRANSCRIBED with the same
   bounded commits.  The library's nsgpu_separable_convolve is no longer on any
   product path: map says gpu_filter_frame reaches nsgpu.o(i.blit_separable_convolve)
   and gpu_drain(), and reaches nsgpu.o(i.nsgpu_cmd_sync) nowhere.

   THE MEASUREMENT THAT FLIPPED IT (`x` x6 per arm, one boot, 2026-10-08).  The
   product path reads 171753..182057 us (mean 175.1 ms) for the CPU bilinear
   grab against 111370..116776 ms (five of six; one 135.7 ms outlier) with this
   front end on - the ranges do not overlap, and the whole GPU front end, the
   ~59 ms filter included, is 58..62 ms CHEAPER than the CPU path it replaces.
   6/6 detect with a readable plate on BOTH arms; every fault counter zero on
   both (`to0 mx0 sp0 strt0 nz0 bus001`), 12 grabs, no wedge.  `gate n` is the
   evidence the work is really there: it reads 6 on every press with this off
   and steps 12,18,..,42 with it on, i.e. the product runs ZERO GPU draws until
   this key is pressed.

   THE PRICE OF THE FLIP WAS AN UNBOUNDED COMMIT ONE KEYPRESS AWAY, AND IT IS
   PAID (2026-10-09).  It read: press `s` alone and the product is back on the
   library's nsgpu_separable_convolve, whose tail is `dhd_finish()`.  `s` = 0 is
   now filter_frame_lib() - that same operator TRANSCRIBED with bounded commits -
   so no keypress reaches the library's wait any more.  The price could be paid
   without the model noticing anything because the picture is unchanged.  gpu.h
   on g_gpu_front_on also carries the difference between the two arms (filtered
   against unfiltered), which stays part of the argument.

   REVERTED TO 0 ON 2026-10-10 ON A USER'S REGRESSION REPORT - AND PUT BACK TO
   1u THE SAME DAY, BY THE MEASUREMENT THIS BLOCK NEVER HAD.  The revert rested
   on a hypothesis: the user reports recognition got worse "in the last few
   days" and that it used to come out near the true 粤R·88888, and this default
   is the only picture-level change in that window.  The hypothesis is dead.

   Key `p` decodes BOTH front ends on ONE snapshot, so the scene is out of the
   picture - the control the 2026-10-08 note above says it could not get ("the
   live camera does not repeat frames").  Six same-frame pairs, and 6/6 favour
   the GPU arm:

     pair cpu: 98AF99988898    pair gpu: 89粤A8898888
     pair cpu: 8粤F899898      pair gpu: 8粤A89888888
     pair cpu: 98AA99898988    pair gpu: 粤8粤A888898
     pair cpu: 98粤99988898    pair gpu: 89粤A8888888
     pair cpu: 98E98998988     pair gpu: 89粤A8888988
     pair cpu: 8AA89898988     pair gpu: 8粤A8988888

   The GPU arm carries 粤A8 in 6/6 and never emits an F or an E; the CPU arm
   carries 粤 in 2/6 and emits F/E in 3/6, and it is 9-heavy where the plate is
   8-heavy.  NEITHER ARM READS THE PLATE - both run one to five characters long
   and both read the R as an A - but the arm this switch selects is the LESS
   WRONG one, on the same frame, six times out of six.

   So the flip finally has the right KIND of evidence: not cost, not "both arms
   decode", but which arm decodes BETTER.  `p` is what produced it - the same
   lesson as key `i`: a control that holds the frame still is worth more than
   more samples.  The price stays what it was, 58..62 ms a grab CHEAPER than the
   CPU path. */
volatile uint32_t g_gpu_front_on = 1u;

/* Stage 3's product switch, and it is a SEPARATE switch from g_gpu_front_on on
   purpose: the blit and the filter are independent stages of the front end, and
   the boot probe measures the filter whether or not the product runs it.

   DEFAULT 1 as of 2026-10-08, and the two reasons the old default carried were
   both retired by measurement rather than by argument.  gpu.h on this variable
   carries the full block; the short form:

     - THE COST IS THE SAME.  Two same-boot captures (`p` x6 per arm) read
       395..428 ms at `s`=1 against 387..425 ms at `s`=0, i.e. inside the
       scatter, where ROWCOL_T measured 1001 against 412.  The template's own
       counter agrees: `gpu sep t ... call 60063 ... drn 59738` is ~60 ms
       against the library's measured ~62 ms for its two passes.  Stage 3 moved
       to ROWCOL, so the 2.43x/6.7x belonged to a mode this build never calls.
     - THE PICTURE IS THE SAME.  The `diff max` gap open since 2026-10-08 (198
       for this pair against the library's 108..120) was ONE fault twice over: a
       weighted texture tap taking the BORDER colour 0, first on pass 2's
       one-wide u window (a dead column 0) and then on its five-tap v window (a
       darkened last row).  Both axes CLAMP now: `over` is 0 on BOTH arms 12/12,
       `max` is 99..108 against 99..111, and the argmax lands on the same
       ordinary interior pixels on both arms.  The two operators no longer
       separate on this instrument.

   What is left is the argument that never needed a measurement: this is the
   BOUNDED operator, and since 2026-10-09 it is no longer the only bounded one.
   The library's tail is `nsgpu_cmd_sync(0)` = `dhd_finish()`, an unbounded wait,
   and the hard rule says the GPU must never take the main function down with it.
   Defaulting to the bounded one made that hole unreachable without anyone
   having to know to press a key; transcribing the library's operator into
   filter_frame_lib() then closed it in the other state as well, so this
   argument no longer separates the two arms - what is left is only this
   switch's own reasons (the picture and the cost, both in gpu.h).

   Both keys flip: 'f' and 's' TOGETHER decide the shipped combination, and both
   reset to 1 as of 2026-10-08 - GPU front end, this repo's bounded operator.
   `s` = 0 no longer selects the library call; it selects the transcription of
   it, so the A/B arm is now "the library's picture, drawn our way" against "the
   rank-1 pair" rather than "library" against "ours".  The same-boot library
   reference goes with it - what the earlier acceptance rests on is the numbers
   that arm already produced (f-on grab 111.7..119.0 ms, `pair diff mean` 6),
   which is exactly how stage 38 was accepted. */
volatile uint32_t g_gpu_front_sep_on = 1u;

/* The pointwise operator's switch.  OFF for the reason and by the measurement
   in the depthwise block above - on that same boot, adding these layers to the
   depthwise ones took the GPU-grab arm from 6 boxes to 1.  The operator does
   work: this is the switch the stage was built for, and it runs 18 of the 52
   layers and 4308 draws on the GPU with the detection head's n39 and n47 both
   among them, on the same measured-and-refuse terms.  It is not shipped on
   because the chain error is, not because the operator is.
   Flipped with the `c` key in main.c, read first in conv_eligible() in
   yolo_inference.cpp. */
volatile uint32_t g_gpu_conv_on = 0u;

/* THE EXACT-OPERATOR ROUTE HAS NO SWITCH IN THIS FILE, and that is deliberate.
   The other three offloads keep their gate here because their consumers live
   here; the TRANSPOSE hook lives in yolo_inference.cpp, so its gate is
   yolo_tr_set()/yolo_tr_get() in that file and the `j` key in main.c is its only
   writer.  gpu_transpose_s8() below is the operator and has no gate of its own -
   it is only reachable through the hook.

   WHY IT GETS A LAYER GATE INSTEAD OF A TAIL.  g_gpu_dw_tail and g_gpu_conv_tail
   exist because a per-layer gate measures the wrong quantity for an ARITHMETIC
   operator: every layer passes its own check and the product still degrades,
   because the product reads many layers in SERIES and each feeds the next.  A
   permutation has no such chain - either every byte moved or the operator is
   simply wrong - so the tail knob would have nothing to sweep.  What it has
   instead is a comparison that is CHEAPER THAN THE GATE WOULD BE: the CPU kernel
   runs first, the GPU's bytes are compared against it, and the GPU's answer is
   adopted ONLY when all of them matched (yolo_inference.cpp's adoption rule).
   All 26 instances go to the GPU every inference; the per-instance byte compare
   replaces the per-layer margin, and the product therefore cannot come out worse
   than the CPU path even if a draw is wrong.

   It is NOT gated on a measured win.  The GPU moves pixels at ~0.88 ms per draw
   and this operator crosses the GPU slot one byte at a time in both directions,
   so it is SLOWER than the kernel it replaces - board-calibrated at ~2.4 us per
   output byte, ~+1.3 s for the class.  The stage is counted in operators running
   on the hardware.  See gpu_transpose_s8(). */

/* The depth cut under each operator's switch - gpu.h carries the whole argument
   for why a tail and not a subset, and why the two numbers are not comparable.
   BOTH RESET TO 0, which is NO LIMIT, so this pair changes nothing until a key
   moves it: an untouched board offloads exactly the layers it offloaded before
   these existed. */
volatile uint32_t g_gpu_dw_tail   = 0u;
volatile uint32_t g_gpu_conv_tail = 0u;

/* Long enough for any honest draw on this part - a whole depthwise pass is
   hundreds of microseconds, and each iteration is one APB read.

   WHAT THIS COSTS IS A MEASUREMENT NOW, and both numbers this comment used to
   carry came back wrong on 2026-09-24 (one boot, `Code=200278`):
   `gpu dw drain dead ... cy820422647` prices the give-up at 820422647 cycles =
   1.367 s, and the poll rate is 38.06 cycles a poll - 63.5 ns at 600 MHz - from
   four independent healthy drains (255393/6710, 256570/6741, 254742/6693,
   129533/3398: all 38.06, so it is a constant of the loop and not a noisy
   reading).  "About a third of a second" implied 16 ns a poll, optimistic by
   3.8x; and a give-up at 1.367 s lands AFTER the one full second n32h7xx_it.c's
   stall reporter waits for, which is the ordering the note at the give-up
   claimed the other way round.  No CYCCNT rollover reaches this number:
   820422647 is under 2^32.  The honest cost this budget has to clear is 0.43 ms.

   THE BUDGET IS NOT SHRUNK TO MATCH IT, and the reason is which way being wrong
   costs.  Too short and a working GPU is declared dead - the offload is gone for
   the rest of the boot, silently, and that is a product defect.  Too long and one
   frame is slow: the give-up latches BOTH operators, so 1.367 s is paid once per
   boot and never again.  An asymmetry that steep buys the whole margin.
   What would move it is the TAIL of the drain distribution, and that has never
   been measured - only the first four drains of a boot print, so every sample in
   hand is an early-layer, small-plane one, and a guard's sample coverage is part
   of its answer.  gpu_dw_drnmax() is that tail, carried on the summary line. */
#define GPU_DRAIN_LIMIT 20000000u

/* THE PROBE BUDGET: what a drain may spend when the only question being asked is
   "did the GPU come back", which is the one question a latch cannot answer.

   Twenty million polls is 1.367 s (measured - GPU_DRAIN_LIMIT above), and that
   is the right price for a drain with a whole layer waiting behind it.  It is
   the wrong price for gpu_revive(), whose whole question is "did the GPU come
   back", and the difference between the two budgets is what a wrong "still dead"
   costs: ONE frame's first drain gives up and re-latches, and the eighteen
   layers behind it then refuse without touching hardware at all.  So the long
   budget here is worth 1.337 s of one frame - not the nineteen spins this
   comment used to claim, which counted the layers that the latch had already
   taken out of the loop.

   Two million polls is 127 ms (measured, not the "about 33 ms" that the same 16
   ns a poll produced), which is a hundred and eighty times above the honest cost
   measured on this board: the first four drains of a healthy boot read 3378,
   6765, 6922 and 3397 polls, and 6710/6741 on the next one.  A limit that far
   above a real drain cannot mistake a working GPU for a stopped one, which is
   the only way this short budget could give a wrong answer.  It is armed by
   gpu_revive() and disarmed by the first drain that succeeds. */
#define GPU_DRAIN_LIMIT_PROBE 2000000u

/* Set when a drain gives up, and then the GPU path is off for the rest of the
   boot.  This is not a retry policy.  A job that has not finished after twenty
   million polls is still holding the source plane, and the next layer would
   hand the GPU scratch memory that this one may still be reading.

   BOTH operators' latches are set from one failure, and that is deliberate:
   there is one GPU, and it does not matter which operator's draw stopped
   answering.  The two flags exist only because the two callers each read their
   own.  Without this, a depthwise draw that wedges leaves the conv path still
   drawing, and every one of its draws then costs a full GPU_DRAIN_LIMIT spin -
   twenty million polls apiece - which is how a wedge turns into tens of seconds
   of a dead-looking product.  The conv path is guarded here as well, because
   the depthwise probe's own draws go through this function too. */
static bool     s_dw_gpu_dead = false;
static bool     s_cv_gpu_dead = false;
static uint32_t s_dw_drain_n  = 0u;
static uint32_t s_dw_dead_n   = 0u;

/* THE TAIL OF THE DRAIN DISTRIBUTION, which is the one number GPU_DRAIN_LIMIT
   has to be sized against and the one thing the `gpu dw drn` line cannot supply:
   it prints for the FIRST FOUR DRAINS OF A BOOT and then stops, so every reading
   it has ever produced is an early-layer, small-plane one.  Nine such readings
   agreed to within a factor of two (3378..6922) and that agreement is exactly
   what a biased sample looks like - the late layers are the big planes, the
   ones with 96 output channels, and no sample of them exists.

   Updated ONLY on a drain that SUCCEEDED.  A give-up would store `limit` and
   destroy the reading on the boot where a healthy tail matters most - and the
   give-up already has a line of its own.  Zero-initialised on purpose, so the
   section proof in [[static-initialiser-narrowing]] has nothing to say about it;
   and unlike the budget flag above, this one's value is PRINTED, so no compiler
   is entitled to narrow it to a boolean.  Read by gpu_dw_drnmax(). */
static uint32_t s_dw_drn_max = 0u;

/* THE FETCH GATE's four counters (see GPU_FETCH_GATE_LIMIT for what the gate is
   for).  `n` = drains that ran it, `waits` = of those, the ones that found
   `curring != rpause` at all, `to` = the ones that used the whole limit and
   flushed anyway, `worst` = the most polls any one gate spent.

   They exist because the gate's whole claim is "this is cheap and it covers the
   window", and neither half is visible without them: a boot that never wedges
   looks identical whether the gate prevented the fault or never fired.  That is
   not a hypothetical - the previous gate's counters read `w == to == n - 1` with
   `mx` pinned at the limit, which is how its premise died in a single boot.  All
   four are printed on the per-frame summary line, and `worst` is what sizes
   GPU_FETCH_GATE_LIMIT the way `s_dw_drn_max` sizes GPU_DRAIN_LIMIT.

   `s_gate_last` and `s_gate_spent` are the two added for the boot that follows
   `Code=201246`, and they answer the two questions the four above cannot.  The
   per-frame counters say a wait happened SOMEWHERE in the frame; `last` is that
   wait as seen by the drain that is about to print `drain dead`, which is the
   only way to know whether the faulting drain's own gate saw the fetcher parked
   (`gw0` - then this condition is the wrong quantity) or spent the limit waiting
   for it (`gw` = limit - then the number is the problem).  `spent` is the budget
   the gate draws on, i.e. the price of the whole boot in polls; it is also what
   makes the "budget exhausted" state possible, and that state is folded into `to`
   rather than given a counter of its own.

   Zero-initialised, every one of them PRINTED, so the narrowing in
   [[static-initialiser-narrowing]] has nothing to say here - including `spent`,
   whose only read is a print and a comparison that leads to a print. */
static uint32_t s_gate_n     = 0u;
static uint32_t s_gate_waits = 0u;
static uint32_t s_gate_to    = 0u;
static uint32_t s_gate_worst = 0u;
static uint32_t s_gate_last  = 0u;
static uint32_t s_gate_spent = 0u;

/* THE START WATCH's counters - the busy vector and the commits that wrote a
   start.  See the block above GPU_STC_BUSY for what question they answer and why
   the vector alone cannot answer it.  Every one of them is a plain `uint32_t`
   accumulator, read by a print, so the narrowing in
   [[static-initialiser-narrowing]] has nothing to say here; `last` is the only
   one written per drain and read on the dead line, which is the one place a
   reading taken BEFORE a commit survives it.

   `or` is an OR of every vector ever seen rather than a histogram: the useful
   question is which bits are EVER set while a commit is about to happen, and a
   single word answers it - a `bus00000003` says the stream reader is the only
   thing that ever runs alongside the STC, `bus00000381` would say units are. */
static uint32_t s_busy_last  = 0u;
static uint32_t s_busy_or    = 0u;
static uint32_t s_start_n    = 0u;
static uint32_t s_start_nz   = 0u;
static uint32_t s_start_or   = 0u;

/* THE DRIVER'S OWN `m_status`, as seen at each commit, and the times the `n` key
   changed it.  Three counters and not a histogram: `dhd_gpu_status_t` is three
   values (IDLE, HALTED, ACTIVE), and the question is which of them the driver is
   in when it decides whether to write a start.  `ms_last` is the one the dead
   line prints - the value at the fatal commit - and `keeps` counts the repairs
   the key actually made, which is what says the key was exercised rather than
   merely set.  Every one is printed, so [[static-initialiser-narrowing]] has
   nothing to say here; the array is indexed by the value itself and clamped,
   because a garbage value must not become an out-of-bounds store. */
static uint32_t s_ms_n[3]  = { 0u, 0u, 0u };
static uint32_t s_ms_last  = 0u;
static uint32_t s_keeps    = 0u;

/* THE STOP CODE THE ISR LAST RECORDED, one word: `m_last_stop_code` in the same
   device struct, written at davehd_kernel_irq.c:99 inside the very branch that
   sets `m_status`.  It is the missing half of `ms`: IDLE can be reached from a
   clean STREAM_END (= 0) as well as from an error, and the enum is
   STREAM_END 0 / STREAM_HALT 1 / START_WHEN_BUSY 2 / INVALID_COMMAND 3 /
   STACK_OVERFLOW 4 / INVALID_REGISTER 5 / CHECKSUM_ERROR 6 / STREAM_DATACOUNT 7
   (davehd_registertypes.h:369-377) - so `ms0 ls0` says the ISR saw a normal end
   of stream and the restart that followed was the DESIGNED resume path, while
   `ms0 ls2` says the driver went IDLE over a start-while-busy it never reported.

   It is read next to `m_status` and NOT read from `dhd_gpu_stream_status`, even
   though both carry a stop code: the global is re-written by every IRQ, so it can
   already hold THIS drain's error by the time the dead line prints, while this
   field is cleared to STREAM_END only when a start is written
   (davehd_kernel_ring.c:207) - i.e. it still holds the stop that put the driver
   into the state under test. */
static uint32_t s_ls_last  = 0u;

/* 1 = the cheap probe budget is armed, 0 = the full one.  Armed by gpu_revive()
   and disarmed by the first drain that succeeds.

   THE POLARITY IS INVERTED ON PURPOSE, and the first version of this got it
   wrong in a way worth recording, because nothing in the source showed it.

   That version held the budget itself: `static uint32_t s_drain_limit =
   GPU_DRAIN_LIMIT;`, with gpu_revive() storing GPU_DRAIN_LIMIT_PROBE.  It
   compiled clean and it was wrong.  The only read of the variable is a loop
   bound, and the compiler is entitled to notice that the ONLY property that read
   uses is "nonzero or not" - so it narrowed the storage to one byte and stored
   an arbitrary nonzero representative.  `fromelf --text -s gpu.o` shows it:
   size 1, section .bss.s_drain_limit, SHT_NOBITS.  A nonzero initialiser cannot
   live in a zeroed section, so **the variable reset to 0**, and by the read's
   own logic 0 selected the SHORT budget: GPU_DRAIN_LIMIT would have been dead
   code and every drain of every boot would have polled for 33 ms instead of a
   third of a second.  Two million polls still passes a healthy drain (3378-6922
   measured), so it would not have looked broken - it would have made a stalled
   GPU read as dead ten times sooner, silently.

   Writing the flag as a flag removes the question: the only values ever stored
   are 0 and 1, the initialiser IS zero, and the SECTION that proves it is the
   same one the other runtime switches use.  See gpu.h on why a build's reset
   value is read off SHT_NOBITS and never off `Code=`. */
static uint32_t s_drain_probe = 0u;

/* 1 = the one REDRAW of the boot has been spent.  Zero-initialised, so the reset
   value is already the safe one and [[static-initialiser-narrowing]] cannot hurt
   it: the only read is a zero test, the initialiser IS zero, and narrowing the
   storage (see the map: it lands as a 1-byte `.bss` symbol) still leaves a flag
   that resets to "not yet spent".

   ONE REDRAW PER BOOT, and it is not a policy that repeats: the redraw's success
   arm clears the latches, so without this flag a wedged GPU would pay
   GPU_DRAIN_LIMIT on every channel of every layer for as long as it is powered.
   Spent, the boot's verdict is final.

   It is DECLARED HERE because the drain's state lives here, and it is READ BY
   gpu_dw_conv() - the operator - because what a redraw needs is the DRAW
   SEQUENCE and its arguments, and gpu_drain() has never seen one.  See the
   block at that call, and the record above the give-up for the flush-shaped
   retry this replaces and why an empty commit could never answer the question. */
static uint32_t s_dw_redraw = 0u;

/* Framebuffer addresses the driver refused as misaligned, counted so the report
   can be a few lines instead of one per plane.  See the check in cv_pass(). */
static uint32_t s_cv_fb_n     = 0u;

/* ================= THE STC DUMP =========================================
   2026-09-28.  Every previous instrument has read EXACTLY ONE GPU register -
   0x30, the sync id - and the whole of the wedge investigation has had to be
   argued from that word failing to change.  It cannot say WHY, and the one
   question that separates the surviving hypotheses is what the job engine is
   doing while that word sits still: still fetching commands, still executing
   a unit, waiting on something, or idle with a completed job whose writeback
   never landed.

   The register block answers all four, and nsgpu_hw.h's GPU_STC_TypeDef names
   every word of it (its comments are BYTE offsets from 0x50040000, which is
   the spelling used here - the driver's `DHD_REG_STC_SYNC_ID_1` is a MAP INDEX
   and is NOT byte 0x0C of this block; that collision has already cost one
   wrong reading, see the note above GPU_STC_SYNC_ID):

       0x10 Busy_Status            0x24 Current_Ring_Address
       0x14 Control                0x28 Current_Stream_Address
       0x18 Interrupt_Status       0x2c Current_Checksum
       0x1c Stream_Address         0x30/34/38 Sync_ID_0/1/2
       0x20 Ring_Pause_Address     0x3c Call_Stack_Pointer + 0x40..0x5c entries

   The call stack is the one that names the failure: dhd_gpu_schedule_job()
   appends a JUMPCALL for every job, so a non-zero Call_Stack_Pointer with a
   stuck sync id says the GPU is INSIDE a job - a specific return address, not
   a mystery - while a zero one with Busy_Status idle says the engine finished
   and the writeback is what went missing.

   SAFETY: read-only, deliberately.  One of these words (Interrupt_Status) is
   write-to-clear on this class of block, so a dump that writes anything, even
   to restore, would destroy the state it is trying to observe.

   WHEN IT PRINTS, and this is the discipline the healthy path demands: once on
   the boot's first drain that ANSWERS (that is the control, and it costs five
   lines once, at boot, where every other bring-up line already prints), and
   then on at most the first two give-ups of the boot, so the wedged state can
   be read against the control AND against itself after a fresh submission (the
   redraw).  Never on a production frame. */
static uint32_t s_stc_ok_dumped = 0u;

static void gpu_stc_dump(const char* why)
{
    char msg[160];

#define GPU_STC_W(off) (*(volatile uint32_t*)(0x50040000u + (off)))

    sprintf(msg, "gpu stc %s busy%08X irq%08X ctl%08X cfg3%08X\r\n", why,
            (unsigned)GPU_STC_W(0x10u), (unsigned)GPU_STC_W(0x18u),
            (unsigned)GPU_STC_W(0x14u), (unsigned)GPU_STC_W(0x0cu));
    UART_SendString(msg);
    sprintf(msg, "gpu stc %s str%08X rpause%08X curring%08X curstr%08X\r\n", why,
            (unsigned)GPU_STC_W(0x1cu), (unsigned)GPU_STC_W(0x20u),
            (unsigned)GPU_STC_W(0x24u), (unsigned)GPU_STC_W(0x28u));
    UART_SendString(msg);
    sprintf(msg, "gpu stc %s chk%08X sy0%08X sy1%08X sy2%08X csp%08X\r\n", why,
            (unsigned)GPU_STC_W(0x2cu), (unsigned)GPU_STC_W(0x30u),
            (unsigned)GPU_STC_W(0x34u), (unsigned)GPU_STC_W(0x38u),
            (unsigned)GPU_STC_W(0x3cu));
    UART_SendString(msg);
    sprintf(msg, "gpu stc %s cs %08X %08X %08X %08X\r\n", why,
            (unsigned)GPU_STC_W(0x40u), (unsigned)GPU_STC_W(0x44u),
            (unsigned)GPU_STC_W(0x48u), (unsigned)GPU_STC_W(0x4cu));
    UART_SendString(msg);
    sprintf(msg, "gpu stc %s cs %08X %08X %08X %08X\r\n", why,
            (unsigned)GPU_STC_W(0x50u), (unsigned)GPU_STC_W(0x54u),
            (unsigned)GPU_STC_W(0x58u), (unsigned)GPU_STC_W(0x5cu));
    UART_SendString(msg);
#undef GPU_STC_W

    /* THE DRIVER'S HALF, and it is the half every earlier reading was missing.

       The block above is the HARDWARE's word: stopped, stop code 2, sync id
       still.  It cannot say what the DRIVER believes, and the two surviving
       hypotheses differ exactly there: an ISR that ran leaves `m_status` IDLE,
       so the next upload WOULD write STREAM_ADDRESS again and the stop is not
       permanent by the driver's state machine; an ISR that never saw the fault
       leaves it ACTIVE, and then nothing we can submit will ever restart the
       engine.  `dhd_gpu_stream_status` is set inside the ISR's stop branch right
       before it writes `m_status`, so one word answers both halves of that - see
       the declaration above for why it is the only reachable one.

       NVIC pending is the second, INDEPENDENT read of the same question: the
       device's interrupt line is level-asserted while Interrupt_Status is
       nonzero, so if the ISR ran, its write-to-clear leaves both the status word
       and the pending bit at 0; if the ISR never ran, the line stays asserted and
       this bit reads 1.  Two reads that can disagree are worth more than one that
       cannot: `out` = 0 with `nvic` = 1 would mean a fault nobody handled, and
       `out` = 2 with `nvic` = 0 is the ordinary handled stop.

       `str`/`rpause` above already carry the driver's ring cursor, because
       dhd_gpu_uploadring() writes RING_PAUSE_ADDRESS from it unconditionally on
       every upload.  So the redraw's dump being word-for-word identical to the
       faulting one is readable here: the flush wrote NEITHER register, i.e. it
       put no command word in front of the fetcher at all. */
    sprintf(msg, "gpu stc %s drv out%08X nvic%08X\r\n", why,
            (unsigned)dhd_gpu_stream_status,
            (unsigned)NVIC_GetPendingIRQ(GPU_IRQn));
    UART_SendString(msg);
}

/* The stall reporter's witness, exported.  n32h7xx_it.c owns the ONLY observer
   that still runs when the code under suspicion does not: g_gpu_dw_prog is held
   at a stage for 31 frame interrupts and the handler prints STALL, then resets
   the product at 8 s.  That handler is where a dump of the STOPPED block has to
   come from, because the path it is watching never comes back to us - the drain
   give-up that prints the "dead" dump is never reached when the wait is inside
   the library (2026-09-28: the front end wedged at stage 38, the boot printed
   eight STALL lines and the reset, and left NO stc dump at all).
   Read-only, no state written, same five-plus-two lines the other two callers
   print - so the same reader applies.  See the note above gpu_stc_dump for what
   each word answers. */
void gpu_stc_dump_now(const char* why)
{
    gpu_stc_dump(why);
}

/* --- THE KEEP REPAIR, AS A CALL OF ITS OWN --------------------------------

   `m_status` is the driver's BELIEF about the engine, and dhd_gpu_uploadring()
   writes a start whenever that belief is not ACTIVE.  A start landing on a live
   STC is what the hardware refuses with stop code 2 (the block above
   GPU_STC_BUSY), and the job it refused never retires - so whatever was waiting
   on it waits forever, never returns, and the product is gone while the CPU
   still runs.  Forcing the belief back to ACTIVE is what makes the commit
   advance RING_PAUSE_ADDRESS WITHOUT a start, which a live stream absorbs.  It
   is the repair of 2026-09-28 (`g_gpu_keep_active`, default 1, 37594 drains,
   zero wedges, and 3 of 3 firings followed by a drain that SUCCEEDED).

   It lives here rather than inline in gpu_drain() because gpu_drain() owns only
   ONE of the commit sites this boot makes.  There are two:

     - OURS, gpu_drain() itself, repaired since 2026-09-28;
     - THE LIBRARY'S.  nsgpu_colorkey(), nsgpu_separable_convolve() and
       nsgpu_set_framebuffer() all end in nsgpu_cmd_sync = dhd_flush +
       dhd_finish, and dhd_finish is UNBOUNDED.  Nothing in our source can reach
       into that call, so the correction has to be made on the way IN - which is
       what gpu_keep_active_now() below is for.

   One predicate, one action, one counter in both places, deliberately: this is
   one repair with one meaning, and a second copy of the condition would be a
   second thing to keep true.  `bv` and `ms` are passed in rather than re-read so
   that gpu_drain()'s own reading - the one its histogram and its dead line
   report - stays the reading this acts on. */
static uint32_t s_keep_shown_dw = 0u;   /* prints from gpu_drain()'s commits */
static uint32_t s_keep_shown_fe = 0u;   /* prints from the library's commits */

static bool gpu_keep_apply(uint32_t bv, uint32_t ms, const char* tag,
                           uint32_t* shown)
{
    if ((g_gpu_keep_active == 0u) || ((bv & 1u) == 0u) ||
        (ms == (uint32_t)E_DHD_STATUS_ACTIVE))
    {
        return false;
    }

    s_devices[0].m_status = E_DHD_STATUS_ACTIVE;
    s_keeps++;

    if (*shown < 2u)
    {
        char kmsg[72];

        /* `ls` is the stop code the ISR recorded for the stop that put the
           driver here - the one field on this line that can still separate
           "a real end of stream the driver was about to resume" from "a
           stop the hardware never raised".  It rides THIS line rather than
           the dead line because this is the moment under test: on a keep
           that works there is no dead line to read it from. */
        (*shown)++;
        sprintf(kmsg, "%s keep ms%u ls%u sy%u\r\n", tag, (unsigned)ms,
                (unsigned)s_ls_last, (unsigned)GPU_STC_SYNC_ID);
        UART_SendString(kmsg);
    }

    return true;
}

/* The same repair, read and applied in one breath, for a commit we do NOT own.
   `bv` and `ms` are read here rather than passed in because there is no drain in
   progress to have read them - the caller is about to enter the library, not our
   own commit path.  NONE of gpu_drain()'s counters are touched: `s_ms_n` and
   `s_ms_last` answer "what does the driver believe when OUR commit happens", and
   folding the library's commits into that question would change what it means.
   Only `s_keeps` is shared, on purpose - it counts repairs, and this is one, so
   `kp` on the summary line covers both sites and a repair can never be invisible.

   Call it immediately before the library draw, never after: after the call there
   is nothing left to run. */
void gpu_keep_active_now(void)
{
    if ((g_gpu_keep_active == 0u) || (s_ready == false))
    {
        return;
    }

    (void)gpu_keep_apply(GPU_STC_BUSY, (uint32_t)s_devices[0].m_status,
                         "gpu front", &s_keep_shown_fe);
}

/* --- THE DRAIN'S OWN SPLIT, the measurement that was missing -----------------

   `blit_tile()` has split a draw four ways since it was written, and the board's
   reading for the stage-3 pair is `bl set 0 drw 46 wai 0 drn 59069` - of a
   59155 us call, 59069 is gpu_drain().  So the whole cost of a GPU draw sits
   inside THIS function, and everything the file has ever said about the draw
   being slow ("the transposed read, the 565 border, the SDRAM region") is a
   guess about what happens in there.  These are the four stamps that end the
   guessing.  Four DWT->CYCCNT reads per drain - at ~4000 drains a frame that is
   sixteen thousand reads against a 1.7 s inference.

   THE FOUR PARTS, and what each one would mean:

     w  `dhd_wait(WRITEBACK|CLEAR|STW)`, the barrier `dhd_finish()` appends
        before its commit.  Its call site says it only QUEUES commands into the
        driver's list, and blit_tile()'s own `dhd_wait(WRITEBACK)` reads 0 - so
        `w ~ 0` here is the confirmation, and a large `w` would be a
        contradiction that matters.
     g  the fetch gate - the parked-fetcher spin.  Cross-checks against
        gpu_fetch_gate_stat()'s `spent` at 38.06 cycles a poll, so this is a
        free consistency check on both numbers.
     f  `dhd_flush()`, i.e. the driver WALKING AND SUBMITTING the command list.
        That is CPU work in the driver rather than GPU work, and it is the one
        part of the four that is ours to make cheaper.
     p  the poll loop - waiting for the sync id to move, i.e. the GPU actually
        executing.  `gpu dw drn`'s `cy` is this same interval, but for the first
        four drains of a boot only, which are all early-layer small-plane ones.

   Accumulated boot-wide and read on the `x` path beside the `gpu dw L..`
   summary - the same class of line, and the only one that sees a whole
   inference.  A give-up is counted too: a boot that gave up early must not read
   like a boot that never committed anything. */
static uint32_t s_drn_n        = 0u;
static uint32_t s_drn_wait_cy  = 0u;
static uint32_t s_drn_gate_cy  = 0u;
static uint32_t s_drn_flush_cy = 0u;
static uint32_t s_drn_poll_cy  = 0u;

void gpu_drain_split_stat(uint32_t* n, uint32_t* wait_us, uint32_t* gate_us,
                          uint32_t* flush_us, uint32_t* poll_us)
{
    if (n        != NULL) { *n        = s_drn_n; }
    if (wait_us  != NULL) { *wait_us  = s_drn_wait_cy  / 600u; }
    if (gate_us  != NULL) { *gate_us  = s_drn_gate_cy  / 600u; }
    if (flush_us != NULL) { *flush_us = s_drn_flush_cy / 600u; }
    if (poll_us  != NULL) { *poll_us  = s_drn_poll_cy  / 600u; }
}

static bool gpu_drain(dhd_handle_t dev)
{
    uint32_t before = GPU_STC_SYNC_ID;
    uint32_t i;
    /* The two readings the post-commit check needs: the busy vector as it stood
       when the commit was about to write its start (into s_busy_last, which the
       dead line prints), and STREAM_ADDRESS as it stood before it.  Both are
       plain reads of write-mostly registers - no side effects, no ordering. */
    uint32_t bv         = 0u;
    uint32_t str_before = 0u;
    /* Resolved ONCE, here, from the flag - so the two budgets are selected by one
       expression whose branches are the two constants themselves, and the loop
       below cannot be narrowed into a boolean by anything. */
    const uint32_t limit = (s_drain_probe != 0u) ? GPU_DRAIN_LIMIT_PROBE
                                                 : GPU_DRAIN_LIMIT;

    /* Exactly what dhd_finish() appends before flushing - the writeback
       barrier over every unit - and then exactly the commit it does.  Only the
       waiting is ours.  `w0`/`w1` are part `w` of the drain's split and they
       bracket this one call and nothing else - see the block above gpu_drain(). */
    const uint32_t w0 = DWT->CYCCNT;
    dhd_wait(dev, (dhd_wait_flags_t)(E_DHD_WAIT_WRITEBACK | E_DHD_WAIT_CLEAR |
                                     E_DHD_WAIT_STW));
    const uint32_t w1 = DWT->CYCCNT;

    /* THE FETCH GATE.  The commit below is the call that can write
       STREAM_ADDRESS - the start signal - and a start that lands while the
       fetcher is still mid-stream stops the STC with
       `E_STOP_IRQ_CODE_START_WHEN_BUSY` for the rest of the boot.  See the block
       at GPU_FETCH_GATE_LIMIT for the measurement that separates the two states
       and for the gate this one replaced.

       It sits AFTER dhd_wait() on purpose: that call only queues commands into
       the driver's list, so waiting before it would hold the queue for no
       reason.  Here it is the last thing before the commit.

       Bounded and falling through: if the fetcher never catches up, the loop
       expires and the flush happens exactly as it would without a gate, so the
       worst case is a delay and not a refusal - and the delay is bounded per
       drain by the limit and per boot by the budget (see the block above).

       `cap` is that policy in one expression: the drained-in-boot budget while
       there is any left, nothing afterwards.  A `cap` of 0 makes the loop test
       `0 >= 0`, so every drain that runs with the budget gone is counted as a
       TIMEOUT - deliberate, and the reason is in the block above: a gate that has
       quietly stopped running must not read like a gate that found nothing to
       do. */
    {
        uint32_t g;
        const uint32_t cap = (s_gate_spent < GPU_FETCH_GATE_BUDGET)
                                 ? GPU_FETCH_GATE_LIMIT : 0u;

        s_gate_n++;
        for (g = 0u; g < cap; g++)
        {
            if (GPU_STC_CURRING == GPU_STC_RPAUSE) { break; }
        }
        s_gate_spent += g;
        s_gate_last   = g;

        if (g != 0u)
        {
            s_gate_waits++;
            if (g > s_gate_worst) { s_gate_worst = g; }

            /* The first two waits of the boot, and no more: which drain had to
               wait is the whole content of reading (a) vs (b) above, and `sy`
               places it - the front end's drains run at `sy` ~6, the fatal draw
               of `Code=201246` at `sy324`.  It prints BEFORE the flush that
               follows, so it lands above any `drain dead` it explains. */
            if (s_gate_waits <= 2u)
            {
                char gmsg[48];

                sprintf(gmsg, "gpu dw gate w%u sy%u\r\n", (unsigned)g,
                        (unsigned)GPU_STC_SYNC_ID);
                UART_SendString(gmsg);
            }
        }
        if ((cap == 0u) || (g >= cap)) { s_gate_to++; }
    }

    /* The state the commit below is about to act on, taken as late as it can be:
       one read of each register, one instruction before the call that can write
       a start.  `s_busy_last` is for the give-up line (a reading taken before a
       commit cannot be reconstructed afterwards - the commit is what destroys
       it), `s_busy_or` accumulates for the report. */
    bv          = GPU_STC_BUSY;
    str_before  = GPU_STC_STR;
    s_busy_last = bv;
    s_busy_or  |= bv;

    /* THE DRIVER'S BELIEF, read in the same breath as the hardware's.  `m_status`
       is what decides whether the commit below writes a start: non-ACTIVE means
       it does, and a start on a live STC is the wedge.  The two readings side by
       side are the whole diagnosis - `ms2 b00000001` is a driver that thinks the
       engine is idle while the engine says it is alive, and that pair is what
       makes "keep it ACTIVE" the right repair.  `ms0` in the same place would mean
       the STC really was stopping and the start simply landed in the tail of it,
       which is a settle, not a state change. */
    {
        uint32_t ms = (uint32_t)s_devices[0].m_status;

        s_ms_last = (ms < 3u) ? ms : 0u;
        s_ms_n[(ms < 3u) ? ms : 0u]++;
        s_ls_last = (uint32_t)s_devices[0].m_last_stop_code;

        /* THE `n` KEY.  Only when both halves of the wedge are present: the STC
           is alive (bit0 of the busy vector) AND the driver is about to write a
           start (not ACTIVE).  Forcing ACTIVE is exactly what the other 1413
           commits of the boot do by themselves - it makes this commit advance
           RING_PAUSE_ADDRESS without a start, which a live stream can absorb.

           DEFAULT 1 SINCE 2026-09-28, on the measurement rather than on the
           argument.  Arm A (default 0): 4 of 4 boots wedged, every one of them on
           the FIRST frame, `strt1` with `b00000001 ms0` - and 1414 gated commits
           in that same boot wrote exactly one start, which IS the fatal one.
           Arm B (this key pressed): one boot, 18 frames, 37594 drains, 29 starts,
           ZERO `drain dead` and zero `gpu dead`, with the key firing 3 times -
           and all 3 times the drain after it SUCCEEDED, which is the part that
           says the live-parked stream really does resume on a pause advance and
           that the driver's word was the thing that was wrong.

           The predicate is bit0 and not the library's own "ACTIVE" definition
           (Busy_Status != 0 && Current_Ring_Address != 0, davehd_kernel_init.c:
           240-253) - deliberately, because bit0 is the bit the STC's own refusal
           names: START_WHEN_BUSY is raised by a start landing on a busy STC unit,
           so a broader predicate would suppress starts the hardware would have
           ACCEPTED.  The 37594 drains above are this predicate's evidence, and
           widening it would throw that evidence away. */
        /* The repair itself is gpu_keep_apply()'s, so that the commits we do
           not own (the library's) get the identical predicate and the identical
           action - see its block above.  Tagged "gpu dw" so the line this site
           has always printed is byte-for-byte the line it still prints. */
        (void)gpu_keep_apply(bv, ms, "gpu dw", &s_keep_shown_dw);
    }

    /* Part `g` ends and part `f` begins here: the fetch gate above is the only
       thing between `w1` and this line. */
    const uint32_t f0 = DWT->CYCCNT;

    if (dhd_flush(dev) != E_DHD_OK)
    {
        /* A refused commit is still a drain that paid for its three earlier
           parts, so they are counted here too - a boot that gave up on its
           first drain must not read like one that never committed at all. */
        s_drn_wait_cy  += (w1 - w0);
        s_drn_gate_cy  += (f0 - w1);
        s_drn_flush_cy += (DWT->CYCCNT - f0);
        s_drn_n++;
        return false;
    }

    /* Part `f` ends and part `p` begins: everything from here to the return is
       the poll loop (plus the few instructions between, which are counted into
       it on purpose - they are ours, not the driver's). */
    const uint32_t f1 = DWT->CYCCNT;

    /* DID THAT COMMIT ACTUALLY WRITE A START?  STREAM_ADDRESS is the register a
       start writes, so comparing it across the commit is a direct count of the
       commits that took the `m_status != ACTIVE` branch - the branch the whole
       wedge lives in, and one no counter has ever reached.  It is read again here
       rather than in the give-up below because the poll loop that follows can
       last a third of a second and the register is live.  See the block above
       GPU_STC_BUSY for why the count is the denominator the busy vector needs. */
    if (GPU_STC_STR != str_before)
    {
        s_start_n++;
        if (bv != 0u) { s_start_nz++; }
        s_start_or |= bv;
    }

    /* The poll loop's own cycles, so the price of a stall is a MEASUREMENT
       instead of the estimate in the comment below it.  At 600 MHz
       `ns a poll` = cy * 5 / (3 * i), and a stall costs `limit` of those - which
       is the number GPU_DRAIN_LIMIT has to be sized against and has never been
       measured (the four healthy drains print cy while `i` is in the thousands,
       so the poll rate comes out of them and the dead line only corroborates).

       Timed from HERE rather than from the top of the function on purpose:
       dhd_wait() and dhd_flush() are not in this number, so it is a LOWER bound
       on what a drain really costs.  CYCCNT is 32 bits and rolls over every
       7.158 s at this clock, so the difference is trustworthy only while the
       true elapsed time is under that - safe by construction for the healthy
       drains, and an assumption for the dead one, which is stated there. */
    const uint32_t t0 = DWT->CYCCNT;

    for (i = 0u; i < limit; i++)
    {
        if (GPU_STC_SYNC_ID != before)
        {
            /* THE SETTLE PHASE WAS HERE AND IT IS GONE - built, flashed and
               measured on 2026-09-21, and it does not fix anything.

               The hypothesis it was built for: `gpu cv dig kv n 176` twice read
               a plane with NO DRAW IN IT as written, while planes that were
               drawn into stayed at the sentinel - so a batch was landing after
               the drain that was supposed to have committed it, the drain was
               premature, and a tail of draws should be waiting.

               What it did: after the change test above, it waited for the sync
               id to stand still for GPU_SYNC_SETTLE consecutive reads, bounded
               by GPU_SETTLE_LIMIT.  It printed `gpu dw settle <polls> <sym>`.

               What came back, every drain of the boot, first line to last:
               `gpu dw settle 3999 sy 33`, `gpu dw settle 3999 sy 39`, `gpu dw
               settle 5490 sy 43`, `gpu dw settle 6665 sy 45`, `3999 sy 46`.  A
               break at poll 3999 means the counter was ALREADY still when the
               phase began - the queue was empty within one settle window
               (4000 polls, ~20 us) of the change that ended the drain.  There
               was no backlog to wait for, and the reading the hypothesis was
               built on is unchanged by the fix that would have had to move it:
               `gpu cv dig lo plane mn0 mx0 su0 n1600`, still 1600 bytes of
               exact zero.

               And the same boot names the counter's real meaning, from the four
               corners this round also added: `gpu cv dig sy 35 37 38 39`.  Nine
               draws moved it by two, three draws by one.  It is not a job
               counter and it never was - it moves about once per flush - so
               "the counter moved" has never meant "our draws are done", and the
               settle is the only honest way to have tested that.  It tested it.
               Negative.

               It costs a mandatory 4000-poll spin on every drain, and the boot
               it ran on was sicker than the one before it - `chk bail n11 gpu`,
               then five `STALL s0 c86 p32704` and no inference at all.  So the
               spin goes with it.  The `gpu cv dig sy` readings stay: they are
               four register reads, and whether a batch moved the counter at all
               is now a question about SUBMISSION rather than completion. */

            /* The first few drains print how long they took, which is the
               measurement that says whether GPU_DRAIN_LIMIT is far enough above
               the honest cost of a draw to be safe.  `cy` is the poll loop's
               cycles (see t0 above) - the field that turns the estimate in the
               comment below into arithmetic. */
            /* The GPU answered, so whatever gpu_revive() armed has been answered
               too, and the full budget goes back for the drains behind this one. */
            s_drain_probe = 0u;

            /* `i` here is exactly the number the `gpu dw drn` line prints, and it
               is free to keep: one compare, on the path that just left the loop.
               This is the only place the late layers' drains are ever recorded. */
            if (i > s_dw_drn_max) { s_dw_drn_max = i; }

            if (s_dw_drain_n < 4u)
            {
                /* 2026-10-12: only the PRINTING is gated - the counter still
                   moves, so the state this boot maintains is identical whether
                   BOOT_VERBOSE is 0 or 1. */
                s_dw_drain_n++;
#if BOOT_VERBOSE
                {
                    char msg[48];

                    sprintf(msg, "gpu dw drn %u cy%u\r\n", (unsigned)i,
                            (unsigned)(DWT->CYCCNT - t0));
                    UART_SendString(msg);
                }
#endif
            }

            /* THE CONTROL FOR THE DUMP.  Once a boot, on the first drain that
               answered - i.e. the one moment where the block is known to be in
               a state that WORKS.  It rides the same one-shot trick as the
               `gpu dw drn` line above it (so it lands while the first draw is
               still the only thing this boot has asked the GPU to do), and it
               never runs again: the healthy path stays silent after this. */
            if (s_stc_ok_dumped == 0u)
            {
                /* Same split as above: the latch is set either way, only the
                   6-line report is gated (2026-10-12).  gpu_stc_dump() keeps its
                   other two callers - gpu_stc_dump_now("stall") from
                   n32h7xx_it.c and the gpu_stc_dump("dead") further down - so
                   the watchdog path is untouched and the function cannot be
                   collected out of the image. */
                s_stc_ok_dumped = 1u;
#if BOOT_VERBOSE
                gpu_stc_dump("ok");
#endif
            }

            /* The drain's split, on the one path that succeeded.  `p` is
               measured from `t0` rather than from `f1` so that the four parts
               sum to the whole call except for the two instruction gaps that
               bracket them - a reading that does not add up is a reading to
               distrust, and this way that is checkable. */
            s_drn_wait_cy  += (w1 - w0);
            s_drn_gate_cy  += (f0 - w1);
            s_drn_flush_cy += (f1 - f0);
            s_drn_poll_cy  += (DWT->CYCCNT - t0);
            s_drn_n++;

            /* The plane probe's reading of this same drain - see the statics in
               the probe block.  On the path where the drain answered, and only
               there: a give-up has no index to report, and leaving the previous
               drain's numbers standing would let the next probe print a drain
               that is not the one it is asking about. */
            s_drn_last_i   = i;
            s_drn_last_was = before;
            s_drn_last_sy  = GPU_STC_SYNC_ID;

            return true;
        }
    }

    /* ================= THE FLUSH RETRY WAS HERE, AND IT IS GONE =============
       Built, flashed as `Code=200502` and read on 2026-09-24.  It cannot do what
       it was built to do, and the reading that shows that is the boot's own.

       WHAT IT WAS FOR: a give-up costs the offload for the REST OF THE BOOT,
       because the latch below is permanent - so the one thing worth spending a
       second wait on is whether the job was ever submitted at all.  A frozen
       counter cannot tell "the GPU stopped" from "the commit was lost", because
       a commit that never reached the hardware also leaves no job to complete:
       in the capture that motivated it, `sy574` reads the same value 183 times
       over at least 19 s.  Both are one frozen register, and only a second
       SUBMISSION separates them.  So it re-issued what the drain always issues -
       dhd_wait(WRITEBACK|CLEAR|STW) then dhd_flush() - and waited once more.

       WHAT CAME BACK: `gpu dw drn retry`, then
           `gpu dw drain dead #0 s37 c0 p0 n20000000 cy1631576834 sy574>574`.
       The cycle count IS the finding: 1631576834 is 1.984 times the 822252047 a
       single budget costs, so the second wait ran its whole length too, and the
       counter still read 574 - the same value it reads in the boot before it.

       WHY THAT SETTLES NOTHING, and this is the whole reason the block is gone:
       THE DRAIN'S OWN FLUSH HAS ALREADY COMMITTED EVERYTHING THAT WAS QUEUED, so
       a second flush of the same queue submits NO NEW JOB.  A test whose only act
       is an empty commit cannot tell "the queue is not consuming" from "there is
       nothing in the queue to consume" - and in the arm that matters, the job
       that timed out was never submitted, so there was and is nothing to consume
       and the frozen counter is exactly what this test would read either way.
       The two lines of that boot are therefore one reading, not two: `drain dead`
       over one budget, then the same thing over two.

       WHAT REPLACES IT IS A REDRAW - a real, fresh submission - and it lives at
       the draw sequence instead of here, because this function has no draw to
       re-issue.  See `s_dw_redraw` above and the block at its call in
       gpu_dw_conv(). */

    /* Why this line carries the progress word and not just the two words it
       used to: g_gpu_dw_prog still holds the last draw's stage, channel and
       chunk, three lines below it is cleared, and so this is the last moment a
       give-up can name WHERE it happened - which is the whole difference between
       a GPU that stopped during layer 3 and one that stopped during layer 17.
       Its fields are the ones the reporter decodes, `DW_PROG`'s encoding in
       reverse, so a `gpu dw drain dead s42 c40 p1` here and a later
       `STALL s42 c40 p1` there name the same stage, channel and chunk.

       THE ORDERING CLAIM THAT USED TO OPEN THIS BLOCK IS FALSIFIED.  It read
       "twenty million polls is about a third of a second, so the drain gives up
       BEFORE the stall reporter in n32h7xx_it.c has seen its one full second of
       no movement".  `cy820422647` on 2026-09-24 measures the give-up at 1.367 s,
       so the reporter's second HAS elapsed by the time this line prints.  That
       boot has a give-up and no `STALL` line at all, so the reporter clearly
       wants something other than elapsed wall time - unmeasured, and this block
       will not put an ordering on two numbers it does not have.

       `s0 c0 p0` means no draw was ever marked - the job that never landed was
       not one of ours.

       THE TWO `sy` WORDS ARE EQUAL BY CONSTRUCTION and this is stated here so
       nobody reads `sy1074>1074` as a finding a second time: the loop above only
       falls through when the register still equals `before`, so `>now` can never
       print anything but that same value.  The word that carries information is
       the FIRST one - what the counter is stuck at (1074 here, 774 on the
       2026-09-23 stall), which differs between boots and therefore says the
       counter does move over a boot.

       Whether it moved during the twenty million polls is unanswerable from this
       line, and that question is the difference between a GPU that STOPPED and
       one that is merely BEHIND a job - the distinction the latch's whole safety
       argument rests on.  It is answered instead by the `sy` field on the
       `gpu dw L<n> cpu` refusals that follow: those lines keep being printed for
       every remaining layer of every remaining frame, and each one reads this
       register, so a column of them across the rest of the boot says whether the
       counter ever moved again - for free, without a single extra poll.

       `cy` is the poll loop's cycles (t0 above), and on this line it is a
       MEASUREMENT, not the corroboration it used to be called: 820422647 cycles
       is 1.367 s, comfortably under the 7.158 s CYCCNT rollover.  Divided by the
       budget it gives 41.02 cycles a poll against the healthy 38.06 (4/4
       readings) - 7.8% higher, and no mechanism for that difference is claimed
       here.  Both numbers are the loop; only one of them is the loop on a GPU
       that is answering. */
    {
        uint32_t p = g_gpu_dw_prog;
        char     msg[96];

        /* `sc` is `m_stop_code` out of Interrupt_Status (bits 8..10), which names
           WHY the engine stopped instead of leaving it to be decoded by hand
           from the dump below: 0 = E_STOP_IRQ_CODE_STREAM_END (normal),
           1 = STREAM_HALT, 2 = START_WHEN_BUSY (the 2026-09-28 wedge), 3 =
           INVALID_COMMAND, 4 = STACK_OVERFLOW, 5 = INVALID_REGISTER,
           6 = CHECKSUM_ERROR, 7 = STREAM_DATACOUNT.  It persists after the
           fault - the ISR's clear value zeroes only this nibble on the way to
           INTERRUPT_CLEAR (davehd_kernel_irq.c:49-51) - so a `sc2` here and an
           `sc2` on the next give-up mean the same thing: the start-while-busy
           stop, still latched, and not a second fault of a different kind.

           `gw` is THIS drain's own gate wait, taken from s_gate_last, which the
           gate above has just written on its way past - the gate runs in every
           drain, this one included, so the number belongs to the drain that is
           giving up and to no other.  It is the field that decides the
           question the four per-frame counters cannot: `gw0` means the fetcher
           was parked exactly at the pause address when the start write landed and
           the STC refused it anyway, so `curring == rpause` is not the quantity
           that predicts the fault (the way `Busy_Status` was not); `gw` at the
           limit means the gate was still waiting for the fetcher when it ran out
           of polls - the premise right, the number too small.

           `b%08X` is the busy vector as it stood one instruction BEFORE the commit
           that killed this drain - the one reading that can only be taken there,
           because the commit is what overwrites it.  `Code=201246` read `gw0` on
           this line, i.e. the fetcher parked, so the vector is the next word and
           the block above GPU_STC_BUSY decodes it (bit0 stc, bit1 str, bit2 stw,
           bit3 reg, bit4 psu, bit5 txi, bit6 tex, bit7 col, bit8 blend,
           bit9 clear).  A lone `b00000001` says the start was refused by a live
           parked stream with every unit idle; unit bits say it raced work that
           was still draining.  The two want opposite fixes, which is why this is
           measured before anything is changed.

           `ms` is the driver's OWN word at the same instant - `dhd_gpu_status_t`:
           0 IDLE, 1 HALTED, 2 ACTIVE - and it is the half that says WHY a start
           was written at all, because davehd_kernel_ring.c:199 writes one only
           when this is not ACTIVE.  Read against `b`: `ms2 b00000001` is the
           driver believing the engine is idle while the engine says it is alive,
           and the repair is to keep the driver honest (`n`); `ms0 b00000001` is a
           genuine stop whose start landed in the tail of it, and the repair is to
           wait for the STC to finish stopping.  They want opposite changes and
           nothing else distinguishes them.

           `ls` is the driver's `m_last_stop_code` at the same instant - the stop
           code the ISR recorded for the stop that left it in that state.  It is
           what splits `ms0` in two: `ms0 ls0` is a clean STREAM_END and the start
           that followed was the DESIGNED resume, while `ms0 ls2` is an
           unreported start-while-busy.  See the block over `s_ls_last`. */
        sprintf(msg, "gpu dw drain dead #%u s%u c%u p%u n%u cy%u sy%u>%u sc%u gw%u b%08X ms%u ls%u\r\n",
                (unsigned)s_dw_dead_n,
                (unsigned)((p / 100000000u) % 100u),
                (unsigned)((p / 100000u) % 1000u),
                (unsigned)((p % 100000u) - ((p != 0u) ? 1u : 0u)),
                (unsigned)i,
                (unsigned)(DWT->CYCCNT - t0),
                (unsigned)before,
                (unsigned)GPU_STC_SYNC_ID,
                (unsigned)((GPU_STC_IRQ >> 8) & 7u),
                (unsigned)s_gate_last,
                (unsigned)s_busy_last,
                (unsigned)s_ms_last,
                (unsigned)s_ls_last);
        s_dw_dead_n++;
        UART_SendString(msg);

        /* The state itself, while it is still the state.  Twice a boot and no
           more: the first give-up is the wedge, and the second one - which on
           every build since Code=200638 is the REDRAW's drain, i.e. a fresh
           submission against the same stuck word - is the only pair that can
           separate "the queue stopped" from "the engine is inside a job it
           cannot finish".  If the two dumps are identical, the redraw changed
           nothing observable and the latch's verdict stands; if Stream_Address
           or the call stack moved while the sync id did not, then the engine IS
           still consuming commands and the wedge is downstream of the fetch.

           Read HERE, between the counter's increment and the latch, because the
           block is a live window: the redraw that follows this return will
           overwrite several of these words. */
        if (s_dw_dead_n <= 2u)
        {
            gpu_stc_dump("dead");
        }
    }

    g_gpu_dw_prog  = 0u;
    s_dw_gpu_dead  = true;
    s_cv_gpu_dead  = true;

    /* The split is counted on the give-up as well, and here it is the reading
       that matters most: `p` at the whole budget with `f` small says the
       submission was fine and the GPU stopped answering, while a large `f` says
       the cost was never the GPU's at all. */
    s_drn_wait_cy  += (w1 - w0);
    s_drn_gate_cy  += (f0 - w1);
    s_drn_flush_cy += (f1 - f0);
    s_drn_poll_cy  += (DWT->CYCCNT - t0);
    s_drn_n++;

    return false;
}

void gpu_dead_stat(uint32_t* dw, uint32_t* cv)
{
    if (dw != NULL) { *dw = s_dw_gpu_dead ? 1u : 0u; }
    if (cv != NULL) { *cv = s_cv_gpu_dead ? 1u : 0u; }
}

uint32_t gpu_dw_drnmax(void)
{
    /* The worst HONEST drain of the boot, in polls - the tail the `gpu dw drn`
       line cannot reach, because that line stops printing after the fourth drain
       and every reading it has ever produced is therefore an early-layer,
       small-plane one.  Decide GPU_DRAIN_LIMIT from this and not from those.

       A give-up is not in here by construction: s_dw_drn_max is written only by
       a drain that saw the register move, so a latched boot reports its last
       honest reading instead of the limit.  Multiply by 63.5 ns for seconds. */
    return s_dw_drn_max;
}

void gpu_fetch_gate_stat(uint32_t* gated, uint32_t* waited, uint32_t* timeouts,
                         uint32_t* worst, uint32_t* spent)
{
    /* The fetch gate's own four readings, in polls.  See GPU_FETCH_GATE_LIMIT for
       what the gate is for; what matters here is that the gate's claim is
       checkable from the field - `gated` counts the drains that ran it, `waited`
       the ones that found the fetcher still behind the pause address,
       `timeouts` the ones that expired and flushed anyway (today's behaviour, so
       a nonzero timeout count means the gate did nothing on those drains and the
       limit needs raising - and a timeout count equal to `gated` means the
       condition is never true at all, which is how the previous gate died), and
       `worst` the largest poll count, which is the number the limit is sized
       against - exactly as s_dw_drn_max sizes GPU_DRAIN_LIMIT.  Multiply by
       63.5 ns for seconds.  Any argument may be NULL.

       `spent` is the FIFTH field and it is the budget's, not a per-drain
       reading: total polls the gate has ever waited, across the boot.  It is what
       says how much the gate has cost, and once it reaches GPU_FETCH_GATE_BUDGET
       the gate stops waiting and every later drain counts into `timeouts` - so
       `timeouts` climbing with `waited` flat beside a `spent` at the budget is
       the budget, not the condition, and the two must be read together. */
    if (gated != NULL)    { *gated = s_gate_n; }
    if (waited != NULL)   { *waited = s_gate_waits; }
    if (timeouts != NULL) { *timeouts = s_gate_to; }
    if (worst != NULL)    { *worst = s_gate_worst; }
    if (spent != NULL)    { *spent = s_gate_spent; }
}

void gpu_busy_vector_stat(uint32_t* seen, uint32_t* starts,
                          uint32_t* starts_nz, uint32_t* starts_or)
{
    /* The START WATCH's four readings, all of them bit-fields or counts of the
       ten-bit busy vector - see the block above GPU_STC_BUSY for the bits and for
       the question they answer.

       `seen` is the OR of the vector over EVERY drain that reached its commit, so
       it says which bits are ever busy at that moment.  `starts` counts the
       commits that really wrote STREAM_ADDRESS (read before and after the flush),
       `starts_nz` how many of those had a busy vector at all, and `starts_or` the
       OR of those vectors - the pair that matters, because a busy vector at a
       start means nothing without the count of starts to compare it against.
       Multiply nothing here by 63.5 ns: these are not costs, they are states.  Any
       argument may be NULL. */
    if (seen != NULL)      { *seen = s_busy_or; }
    if (starts != NULL)    { *starts = s_start_n; }
    if (starts_nz != NULL) { *starts_nz = s_start_nz; }
    if (starts_or != NULL) { *starts_or = s_start_or; }
}

void gpu_drv_status_stat(uint32_t* n_idle, uint32_t* n_halted,
                         uint32_t* n_active, uint32_t* last, uint32_t* keeps,
                         uint32_t* last_stop)
{
    /* The DRIVER's own status word, read at the same instant as the busy vector
       above - `dhd_gpu_status_t`, so 0 IDLE / 1 HALTED / 2 ACTIVE, histogrammed
       into three counters (an out-of-range value lands in slot 0, which cannot
       happen on a boot where nothing else is wrong and is kept so the histogram
       always sums to the drain count).

       This is the reading that says WHY a start was written at all.  The start
       watch prints `starts`, and a start only appears when the driver believes it
       is not ACTIVE - davehd_kernel_ring.c:199-208 writes STREAM_ADDRESS only
       when m_status != ACTIVE, while RING_PAUSE_ADDRESS is written every time.
       So `last` next to the faulting drain's `b` field is the decisive pair:
       `ms2 b00000001` = the driver thinks the engine is idle while the engine
       says it is alive, and the repair is to keep the driver honest at the moment
       it matters (`n`), whereas `ms0 b00000001` = a real stop whose start landed
       in its tail, and that one wants a settle instead.  Nothing else on the
       board separates those two, which is why this word is read on every drain
       rather than only on the failing one.

       `keeps` counts the interventions the `n` key made - reads it and changes
       the word itself, touching NO hardware.  `g_gpu_keep_active` defaults to 1
       since 2026-09-28, so `keeps` is normally the count of repairs an ordinary
       boot made by itself; it is 0 on a boot with no GPU drains at all.

       `last_stop` is the same struct's `m_last_stop_code` at the same instant -
       the stop code the ISR recorded for the stop that left the driver in that
       state.  IDLE is reachable from a clean STREAM_END (0), so `ms` alone cannot
       say whether the driver was about to resume a finished stream (the designed
       path) or had been told about a stop that the hardware never raised; this is
       the word that splits it.  See the block over `s_ls_last`.

       Any argument may be NULL. */
    if (n_idle != NULL)    { *n_idle = s_ms_n[0]; }
    if (n_halted != NULL)  { *n_halted = s_ms_n[1]; }
    if (n_active != NULL)  { *n_active = s_ms_n[2]; }
    if (last != NULL)      { *last = s_ms_last; }
    if (keeps != NULL)     { *keeps = s_keeps; }
    if (last_stop != NULL) { *last_stop = s_ls_last; }
}

uint32_t gpu_sync_id(void)
{
    /* One read of the register gpu_drain() polls, for the TFLM side.  It is the
       only view of the GPU that exists AFTER a drain has given up: the drain's
       own line cannot report it (its two `sy` words are equal by construction,
       see the block there), and a latched refusal touches no hardware at all, so
       a caller that keeps printing this value across the rest of a boot is the
       one instrument that can tell a GPU that STOPPED from one that was merely
       BEHIND a job.

       A plain read cannot hang, and this is the same access gpu_drain() makes
       twenty million times in a row on a healthy board. */
    return GPU_STC_SYNC_ID;
}

void gpu_revive(void)
{
    /* The ordering matters and is the reason this is a function rather than a
       line in main.c: the latched callers check their flag on the way IN, so
       clearing it while a cheap probe is not yet armed would let one inference
       pay a full GPU_DRAIN_LIMIT spin per layer on a GPU that is still stopped.
       The budget goes first. */
    s_drain_probe = 1u;

    s_dw_gpu_dead = false;
    s_cv_gpu_dead = false;
}

/* The front end's bounded way out, and the reporter's stage setter.  Both are
   defined here rather than beside their callers because they need gpu_drain()
   and DW_PROG - see the forward declarations and the contract block next to
   gpu_sb_reset(). */
static bool gpu_fb_put(dhd_handle_t dev, uint32_t fb)
{
    /* Exactly what nsgpu_set_framebuffer() does with the address, and nothing
       else: the library body's second half is the nsgpu_cmd_sync(0) that
       gpu_drain() replaces.  See gpu_drain() for why that wait cannot be the
       library's, and the note at its call in dw_pass_bc()'s tail for why the
       drain rather than the sync is what the next read depends on. */
    (void)dhd_set_framebuffer_address(dev, (dhd_gpu_ptr_t)fb);
    return gpu_drain(dev);
}

static void gpu_front_prog(uint32_t stage)
{
    /* `ch` and `px` are 0: the front end has no channel, and a pixel index
       would need the fold loops it does not have.  The stage is the whole
       message, and the reporter prints it as `s38 c0 p0`. */
    g_gpu_dw_prog = (stage != 0u) ? DW_PROG(stage, 0u, 0u) : 0u;
}

/* Pack ONE kernel into the CLUT's quad layout, which is what
   dhd_set_texture_kernel() does for a single kernel (davehd_driver_txp_inl.h:
   567-632) with the loop lifted out so that a whole layer's worth of kernels
   can be packed into one array:

       cw = (kw+1)>>1, ch = (kh+1)>>1, entries = cw*ch
       entry ((x>>1) + (y>>1)*cw) gets tap (y,x) at byte ((qp^3)*8)
                                         where qp = (x&1) + ((y&1)<<1)

   so a 3x3 is 4 entries and a 5x5 is 9 - which is the fact that makes the
   512-entry CLUT enough for a whole layer: 96 channels of 3x3 is 384 entries.
   The weights go in as raw bytes, sign included; the SIGNED part is selected by
   the texture format, not here. */
static void dw_pack_quad(uint32_t* clut, const int8_t* k, uint32_t kw, uint32_t kh)
{
    uint32_t cw = (kw + 1u) >> 1u;
    uint32_t ch = (kh + 1u) >> 1u;
    uint32_t cnt = cw * ch;
    uint32_t i, x, y;

    for (i = 0u; i < cnt; i++) {
        clut[i] = 0u;
    }
    for (y = 0u; y < kh; y++) {
        for (x = 0u; x < kw; x++) {
            uint32_t qi = (x >> 1u) + ((y >> 1u) * cw);
            uint32_t qp = (x & 1u) + ((y & 1u) << 1u);
            clut[qi] |= ((uint32_t)(uint8_t)k[y * kw + x]) << ((qp ^ 3u) * 8u);
        }
    }
}

/* One convolution pass: texture `src` (tw x th, pitch tw, 8-bit) into
   framebuffer `dst` (dw x dh, pitch dw, 8-bit) with the kernel already sitting
   at `clut_base`.  The destination is a separate size from the texture so the
   UV-gradient test can make them differ; the production call passes dw==tw.

   The driver's framebuffer format is global and nsgpu_init() set it to RGB565,
   so it is read, changed to L_8 for the pass, and put back - an 8-bit
   destination is not expressible any other way, and leaving it changed would
   break every later nsgpu_* call.  nsgpu_get_framebuffer()/set_framebuffer()
   guard the address the same way, because blit_convolve's callers repoint that
   too and so does this.

   AND SO ARE THE SIZE AND THE BBOX, which the first version of this function did
   not restore.  Measured, 2026-09-18: this pass sets all four (format, size,
   address, box) to its own tile and used to put back only two, so after any
   64x64 pass the driver was left at 64x64 pitch 64 with a 0..63 box.  The boot
   log pinned the consequence exactly - `fb dw 4006 64x64 p64 ...` is the only
   ambient reading that moves in the whole boot block, and the agreement probe
   bracketing it reads `ok 1` before (`mean 3`) and `ok 0` after (`mean 106`).

   That is not a probe-only defect: the depthwise operator is ON for 19 layers,
   so every inference has been leaving the driver's framebuffer at the last
   layer's tile.  Whatever draws next inherits it.  All four are measured on the
   way in and restored on the way out now. */
static bool dw_pass_bc(dhd_handle_t dev, uint32_t src, uint32_t dst,
                       uint32_t tw, uint32_t th,
                       uint32_t dw, uint32_t dh,
                       uint32_t clut_base, const int8_t* k, uint32_t kw, uint32_t kh,
                       dhd_texel_org_t texfmt, dhd_pixel_org_t pixfmt,
                       dhd_address_mode_t amode,
                       float scale, int32_t bias,
                       float u0, float v0, float dux, float dvy,
                       uint32_t a_border)
{
    dhd_uv_data_t uv;
    uint32_t      saved_fb;
    dhd_pixel_org_t saved_fmt;
    uint32_t      sav_w, sav_h, sav_p;
    uint32_t      sav_x0, sav_y0, sav_x1, sav_y1, sav_ys;

    /* ORDER MATTERS, and this is the order blit_convolve uses:

           uv, active_texture_unit, address, size, format, kernel, scalebias,
           wrap, bbox, draw, wait

       dhd_active_texture_unit() is the one that must not be moved: it is what
       sets device->m_tex_unit_base (davehd_driver_txp_inl.h:55), and every
       setter below computes its register address as "that base + an offset".
       The offsets are named _0 (TEX_Mode_0, TXP_Clut_Offset_0,
       TXP_Filter_Scale_Bias_0, TXP_Fill_Color_0), which reads as "unit 0
       hardcoded" - it is not.  They are THAT UNIT plus an offset, where "that
       unit" is whichever one dhd_active_texture_unit() last selected.

       A previous version of this function called dhd_set_texture_kernel()
       FIRST, before selecting the unit, because "the kernel goes up and the
       CLUT base is re-pointed after".  Everything the kernel call did - the
       CLUT upload, the clut base, m_txa_filter_kernel_width/height,
       m_txa_filter_sample_shape, m_tex_filter_u/v - landed on whatever unit the
       last nsgpu_* call had left active, and unit 0 was left with a stale 1x1
       kernel.  Measured result: the probe read a weight of 0 and returned
       bias-only (0x80 -> 128), and both step sweeps returned 0.  Hence the
       order below.

       dhd_set_texture_size() must precede dhd_set_texture_format(): the format
       call recomputes m_txc_burst_pitch_0 from the pitch that size sets
       (davehd_driver_txp_inl.h:368-375), and only when the tiled address mode
       is in use - which is exactly our case. */
    uv.m_u0 = u0;  uv.m_v0 = v0;  uv.m_dux = dux;  uv.m_duy = 0.0f;
    uv.m_dvx = 0.0f;  uv.m_dvy = dvy;

    /* The stall probe, fine grain: stages 30..38 name the driver call this
       function is inside, which is the only thing a stall inside the driver can
       be waiting on - every one of these appends to the stream, and appending
       is what commits it, and committing is the only place in the driver that
       blocks without a panic.  See g_gpu_dw_prog's definition above; stage and
       channel are all the reader needs, and the channel comes from the caller
       because this function is also the probes' and has no channel of its own. */
    g_gpu_dw_prog = DW_PROG(30u, s_dw_prog_ch, 0u);
    dhd_set_uv(dev, 0u, &uv);
    dhd_active_texture_unit(dev, 0u);
    dhd_set_texture_unit_enable(dev, 1u);
    dhd_set_texture_address(dev, (dhd_gpu_ptr_t)src, 0u);
    dhd_set_texture_size(dev, tw, th, tw);
    dhd_set_texture_format(dev, texfmt, amode, 0u);
    g_gpu_dw_prog = DW_PROG(31u, s_dw_prog_ch, 0u);
    /* Also writes m_txa_filter_kernel_width_0 / _height_0 / _sample_shape_0 and
       enables the texture filter, which is half of why it has to be called even
       when the CLUT is already loaded and only the base is being re-pointed. */
    dhd_set_texture_kernel(dev, kw, kh, (const uint8_t*)k, clut_base);
    g_gpu_dw_prog = DW_PROG(32u, s_dw_prog_ch, 0u);
    dhd_set_texture_clut_base(dev, clut_base);
    dhd_set_texture_scalebias(dev, scale, bias);
    dhd_set_texture_wrap(dev, E_DHD_TEX_BORDER, E_DHD_TEX_BORDER);
    dhd_set_texture_border_color(dev, a_border);

    /* E_DHD_OP_COPY explicitly, though blit_convolve does not set it: that
       library path reads an A8,L8 texel whose alpha is 0, so it never sees the
       colour unit modulate by alpha.  An AL_8 texel's alpha is its own value,
       so leaving the colour state at whatever the last nsgpu_* call left could
       scale this result.  COPY makes the output the texture value and nothing
       else, which is what a convolution wants. */
    dhd_set_color_operation(dev, E_DHD_OP_COPY);
    /* ---- THE DRAW MUST NOT READ THE DESTINATION -------------------------
       E_DHD_OP_COPY above makes the colour unit's OUTPUT the texture value,
       and that was believed to make this draw a pure function of its texture.
       It does not, because the blend unit sits BETWEEN the colour unit and the
       framebuffer, and the blend unit's mode is AMBIENT STATE THIS PATH NEVER
       SETS.

       The vendor's own source says what the two modes do, and one of them is a
       declared read of the destination:

         davehd_util_state.c:74-80  E_DHD_UTIL_BLEND
             color_blending(SRC_ALPHA, ONE_MINUS_SRC_ALPHA)
             alpha_blending(ONE, ONE_MINUS_SRC_ALPHA)
             coverage_blend(TRUE)
             read_decision(E_DHD_USING_ONE_MINUS_ALPHA, TRUE)   <-- DST IS READ
         davehd_util_state.c:67-72  E_DHD_UTIL_REPLACE
             color_blending(ONE, ZERO)
             alpha_blending(ONE, ZERO)
             coverage_blend(FALSE)
             read_decision(E_DHD_READ_NEVER, FALSE)             <-- DST IS NOT

       Under BLEND the pixel stored is `src*alpha + dst*(1-alpha)`, so THIS
       draw's result is a function of whatever the previous draw left in
       GPU_DW_DST.  Everything downstream then reads it as if it were the
       layer's own answer.

       WHERE THE AMBIENT VALUE COMES FROM.  dhd_util_blend() has exactly one
       caller in this file - gpu_scale_frame(), line 2301, the front end - and
       it passes E_DHD_UTIL_BLEND because that is what the library's colorkey
       sequence it replaced passes.  That call runs on every grab, so from the
       first grab of the session onward every depthwise draw inherits it.  The
       depthwise path sets the framebuffer format, size, address and bbox and
       restores all four; this one word was never in that set.

       THE READING THAT NAMES IT.  The plane probe (`gpu dw pl`, key x + the
       arm in dw_gpu_invoke) shows, for position 0 across two inferences of one
       FROZEN frame:

         D2 s15D532CF d01A3087F1 d18DFB10A5
         D4 s15D532CF d0010F5284 d1D5BB39C1

       `s` - the packed source plane - is byte-identical, and every word
       gpu_env_state() can read is byte-identical too.  `d0`, the destination
       BEFORE the draw, differs, and `d1`, the destination after it, differs
       with it.  With the source and the readable state held constant, the
       destination's stale content is the only input left that can move the
       result - which is exactly what read_decision(USING_ONE_MINUS_ALPHA)
       declares and what a COPY would forbid.

       THE SAME EVIDENCE FROM THE OTHER SIDE, and this is why it went unseen:
       `gpu dw gpudiff` redraws the layer with NO pack and NO unpack in between,
       so its two draws share one destination and one source.  It reads
       `worst 0 bad 0` on both stride classes - correctly, and it still could
       not see this, because the state its two draws share is the defect.  And
       `gpu dw H`'s own columns are identical at the INPUT of positions 0 and 1
       and differ at their OUTPUT, which is what a per-draw destination
       dependency looks like from a layer boundary.

       WHY REPLACE AND NOT "the state the front end set".  The depthwise draw
       does not want a blend mode with a particular alpha; it wants no
       destination read at all.  REPLACE is the mode whose read_decision is
       READ_NEVER, so it removes the dependency by construction and does not
       require knowing what alpha the ambient operand happens to resolve to for
       an L_8 destination.  It is also the mode the vendor's own convert paths
       use as their default (davehd_util_convert.c:956, 1153, 3165, 3216).

       COST: nothing that can stall.  dhd_util_blend() is five stores into the
       device struct - no ring command, no flush, no commit point - the same
       argument the util-cliprect line in gpu_scale_frame() carries. */
    dhd_util_blend(dev, E_DHD_UTIL_REPLACE);
    g_gpu_dw_prog = DW_PROG(33u, s_dw_prog_ch, 0u);

    saved_fb  = nsgpu_get_framebuffer();
    saved_fmt = dhd_get_framebuffer_format(dev);
    dhd_get_framebuffer_size(dev, &sav_w, &sav_h, &sav_p);
    dhd_get_bbox(dev, &sav_x0, &sav_y0, &sav_x1, &sav_y1, &sav_ys);
    g_gpu_dw_prog = DW_PROG(34u, s_dw_prog_ch, 0u);

    dhd_set_framebuffer_format(dev, pixfmt);
    dhd_set_framebuffer_size(dev, dw, dh, dw);
    (void)dhd_set_framebuffer_address(dev, (dhd_gpu_ptr_t)dst);
    dhd_set_bbox(dev, 0u, 0u, dw - 1u, dh - 1u, 0u);
    g_gpu_dw_prog = DW_PROG(35u, s_dw_prog_ch, 0u);
    /* Inert unless gpu_dw_plane_arm() was called, and it disarms itself on the
       first draw it sees - see the block above it.  Placed here, after every
       setter this draw depends on and before the one call that consumes them,
       because here the ambient state IS the state that applies.  `tw`/`th` are
       the source plane's extent in 565 texels, `dw`/`dh` the destination's in
       bytes (the draw's pixel org is L_8). */
    gpu_dw_plane_pre((uint32_t)src, tw * th * 2u, (uint32_t)dst, dw * dh);
    dhd_draw(dev, 0u, 0u, (dhd_drawattribute_flag_t)0u);
    g_gpu_dw_prog = DW_PROG(36u, s_dw_prog_ch, 0u);
    /* E_DHD_WAIT_WRITEBACK - "wait until all pixels are written", 0x1F8, which
       is the value blit_convolve passes literally.  Equivalent, and it says
       what it means. */
    dhd_wait(dev, E_DHD_WAIT_WRITEBACK);
    g_gpu_dw_prog = DW_PROG(37u, s_dw_prog_ch, 0u);
    /* The same plane again, now holding what the CPU is about to unpack.  After
       the wait and before the restore, so it brackets the draw and nothing
       else. */
    gpu_dw_plane_post((uint32_t)dst, dw * dh);

    dhd_set_framebuffer_format(dev, saved_fmt);
    dhd_set_framebuffer_size(dev, sav_w, sav_h, sav_p);
    dhd_set_bbox(dev, sav_x0, sav_y0, sav_x1, sav_y1, sav_ys);
    (void)dhd_set_framebuffer_address(dev, (dhd_gpu_ptr_t)saved_fb);

    /* What nsgpu_set_framebuffer(saved_fb) does, minus its wait - see the
       comment on gpu_drain() for why the wait had to stop being the library's.
       This is the call the board was dying in: STALL s37 c0 p0 is the stage set
       on the line above, and nsgpu_set_framebuffer was the statement after it. */
    if (!gpu_drain(dev))
    {
        /* Refused, not fatal: the caller falls back to the stock kernel for
           this layer, which is the whole point of the drain being bounded. */
        gpu_dw_plane_abort();
        g_gpu_dw_prog = 0u;
        return false;
    }

    /* The plane as the caller is about to unpack it - see the block on
       gpu_dw_plane_post2().  After the drain and after every restore, so it
       reads exactly the bytes the unpack will read. */
    gpu_dw_plane_post2((uint32_t)dst, dw * dh);

    /* Cleared on the way out so a probe's call cannot leave a stale stage
       behind, where it would be reported as a stall during the next second of
       unrelated work.  gpu_dw_conv() sets its own stage immediately after this
       returns, so nothing is lost. */
    g_gpu_dw_prog = 0u;
    return true;
}

/* ================= THE ONE REDRAW =========================================
   A drain that gives up is a give-up AND a permanent latch, so it costs the
   offload for the rest of the boot - and the two states it conflates look
   IDENTICAL from the drain's side: a GPU that stopped and a job that was never
   submitted both leave one frozen register.  `gpu dw drn retry` was built to
   separate those two and could not, because its only act was an empty commit -
   see the record at the give-up in gpu_drain().

   THIS IS THE SECOND SUBMISSION: the same call, the same arguments, again.
       `gpu dw redraw ok sy<id>` = the queue ANSWERED a fresh job,
   and both latches are cleared with it, so a wedge costs ONE EXTRA DRAW
   instead of the boot.  Without that line the caller's refusal is unchanged.

   WHY RE-DRAWING IS SOUND even when the first attempt did land: the two draws
   have the same source, kernel, scale and destination, so they write the same
   bytes into the same plane in either completion order - THE REDRAW IS
   IDEMPOTENT, which is what lets this be a retry rather than a rollback.  And
   nothing here weakens the drain discipline: the CPU reads the destination only
   after a drain returned true, and if the queue is in order then the job that
   timed out completed before the redraw's own.

   IT WRAPS dw_pass_bc() INSTEAD OF LIVING IN IT, so that the measured function
   is untouched and so both of the operator's pass sites get it by construction.
   That second part is not hypothetical: the first version of this was inlined
   at pass 1 only, and the first boot to read it (`Code=200638`) wedged in PASS 2
   - `s37 c0 p0`, no `gpu dw redraw` line - where the inlined form could never
   run.  One redraw at one of two sites is a coin toss; a wrapper has no third
   site to forget.

   ONCE PER BOOT (s_dw_redraw): the success arm clears the latches, so a redraw
   that always succeeded would put a full budget's spin on every channel of
   every layer.  On a board that wedges the price is one extra GPU_DRAIN_LIMIT,
   which is what the flush-shaped retry already charged for an answer it could
   not give. */
static bool dw_pass_bc_retry(dhd_handle_t dev, uint32_t src, uint32_t dst,
                             uint32_t tw, uint32_t th,
                             uint32_t dw, uint32_t dh,
                             uint32_t clut_base, const int8_t* k,
                             uint32_t kw, uint32_t kh,
                             dhd_texel_org_t texfmt, dhd_pixel_org_t pixfmt,
                             dhd_address_mode_t amode,
                             float scale, int32_t bias,
                             float u0, float v0, float dux, float dvy,
                             uint32_t a_border)
{
    if (dw_pass_bc(dev, src, dst, tw, th, dw, dh, clut_base, k, kw, kh,
                   texfmt, pixfmt, amode, scale, bias, u0, v0, dux, dvy,
                   a_border))
    {
        return true;
    }

    if (s_dw_redraw != 0u)
    {
        return false;
    }

    s_dw_redraw = 1u;

    {
        char msg[48];

        sprintf(msg, "gpu dw redraw\r\n");
        UART_SendString(msg);

        if (!dw_pass_bc(dev, src, dst, tw, th, dw, dh, clut_base, k, kw, kh,
                        texfmt, pixfmt, amode, scale, bias, u0, v0, dux, dvy,
                        a_border))
        {
            return false;
        }

        sprintf(msg, "gpu dw redraw ok sy%u\r\n", (unsigned)gpu_sync_id());
        UART_SendString(msg);
    }

    /* The failed drain latched both operators; a queue that has just answered a
       fresh job is not the permanent stop that latch asserts, so this boot's
       verdict is withdrawn.  Both flags, for the reason gpu_revive() clears
       both: they are the same question about the same queue. */
    s_dw_gpu_dead = false;
    s_cv_gpu_dead = false;

    return true;
}

/* The probes' spelling of the above, with the border colour they have always
   used.  It is kept separate from the operator's call for one measured reason:
   dhd_set_texture_border_color()'s own documentation says the colour is
   **ARGB8888**, so 0x80808080 is four 0x80 channels and NOT the 565 texel
   0x8080 whose blue field is zero.  Read as ARGB8888 and converted to the
   five-bit blue field the SIGNED_565 kernel multiplies, it is 0x80>>3 = 16,
   where the layer needs in_zp(-128) -> 0.  gpu_dw_conv() passes 0u for that
   reason; every probe keeps 0x80808080 so its readings stay comparable with
   the ones already in the log. */
static bool dw_pass(dhd_handle_t dev, uint32_t src, uint32_t dst,
                    uint32_t tw, uint32_t th,
                    uint32_t dw, uint32_t dh,
                    uint32_t clut_base, const int8_t* k, uint32_t kw, uint32_t kh,
                    dhd_texel_org_t texfmt, dhd_pixel_org_t pixfmt,
                    dhd_address_mode_t amode,
                    float scale, int32_t bias,
                    float u0, float v0, float dux, float dvy)

{
    return dw_pass_bc(dev, src, dst, tw, th, dw, dh, clut_base, k, kw, kh,
                      texfmt, pixfmt, amode, scale, bias, u0, v0, dux, dvy,
                      0x80808080u);
}

/* GPU_DW_SRC/DST are in the GPU slot, which deliberately has no MPU region and
   therefore the default Device mapping - uncached.  So a CPU read after
   dhd_wait() is the real memory and needs no cache maintenance, which is the
   whole reason the scratch lives there instead of in the tensor arena.  (The
   arena is Normal cacheable, and this pack's arch_dcache_flush_all() is an
   empty stub - see the note in gpu.h.) */
static uint8_t  dw_rd (uint32_t base, uint32_t i)        { return *(volatile uint8_t*)(base + i); }
static uint16_t dw_rd16(uint32_t base, uint32_t i)       { return *(volatile uint16_t*)(base + (i * 2u)); }
static void     dw_wr (uint32_t base, uint32_t i, uint8_t v) { *(volatile uint8_t*)(base + i) = v; }

/* ===================================================================
   THE DROPPED DRAW, COUNTED ON EVERY DRAW.

   `gpu dw pl2` already reported a draw whose destination kept its pre-draw
   value for the whole 4000-iteration bound - and it reported one only on the
   first draw of a layer and only from the SECOND inference of a boot on.  That
   reading is two draws wide per inference, because the plane probe arms on the
   first two eligible layers.  This is the same question asked on all of them.

   WHY IT IS NOT COSMETIC.  Every draw in gpu_dw_conv()'s ci loop writes its
   channel's plane into the SAME GPU_DW_DST and the CPU unpacks that plane
   immediately afterwards.  So a draw that writes nothing does not leave a plane
   visibly unwritten - it leaves the PREVIOUS draw's plane in the scratch and
   the unpack reads it as THIS channel's activations.  In the split path the
   correction plane is then folded onto it (the `out[row] + byte2` addition), so
   the error is not even one channel's worth of stale data and it can saturate.
   Nothing downstream can see it: the byte is a plausible activation, and the
   per-layer check compares a plane the GPU wrote against a plane the CPU
   computed - never the scratch.

   HOW A DRAW IS ASKED.  Three bytes with three different values are stored into
   the region the draw is about to cover - position 0, the middle and the last
   byte - and read back after dw_pass_bc_retry() has drained.  A draw that wrote
   at all cannot leave all three as they were; a draw that wrote nothing leaves
   all three, which is the event.  Three positions and not one because a real
   draw's output can be ANY byte: one position would false-positive on 1/256 of
   draws, three on 1/(256^3) - and the three are not adjacent, so a partial
   write does not leave all three either.

   The cost is three uncached byte stores and three loads against a draw that
   cannot be cheaper than tens of microseconds; it is not visible in a profile.

   AND THE REPAIR.  A drop is redrawn once, for the same reason dw_pass_bc_retry
   already redraws a failed drain: the draw is a pure function of operands that
   were never repacked, so a second identical draw is byte-identical to the one
   that was lost - `gpu dw gpudiff s2 n 8192 worst 0 bad 0` is that statement,
   measured.  It is counted separately as `re`, because a redraw that ALSO drops
   says the failure is not a lost job but state the first draw leaves behind,
   which is a different fault and the next thing to chase.

   NOT GATED, and nothing is refused.  The repair only fires on the event it
   measures, so there is no second arm to hold a still scene for; and taking a
   layer off the hardware on the strength of a defect that has not been located
   yet would change the set being measured.  `drop`/`chk`/`re` and the first
   drop's own (draw ordinal, channel, pass) are appended to the `gpu dw L...`
   line in main.c. */
#define GPU_DW_SENT0 0xA5u
#define GPU_DW_SENT1 0x5Au
#define GPU_DW_SENT2 0xC3u

static uint32_t s_dw_dc_chk  = 0u;   /* product draws the sentinels were armed on */
static uint32_t s_dw_dc_drop = 0u;   /* ... of which the draw wrote nothing       */
static uint32_t s_dw_dc_re   = 0u;   /* ... whose one redraw ALSO wrote nothing   */
static uint32_t s_dw_dc_d    = 0u;   /* first drop: draw ordinal, 1-based (0=none)*/
static uint32_t s_dw_dc_c    = 0u;   /* first drop: channel                       */
static uint32_t s_dw_dc_p    = 0u;   /* first drop: pass (1 or 2)                 */

static void dw_sent_arm(uint32_t dst, uint32_t n)
{
    dw_wr(dst, 0u,     (uint8_t)GPU_DW_SENT0);
    dw_wr(dst, n / 2u, (uint8_t)GPU_DW_SENT1);
    dw_wr(dst, n - 1u, (uint8_t)GPU_DW_SENT2);
}

static uint32_t dw_sent_alive(uint32_t dst, uint32_t n)
{
    return ((dw_rd(dst, 0u)     == (uint8_t)GPU_DW_SENT0) &&
            (dw_rd(dst, n / 2u) == (uint8_t)GPU_DW_SENT1) &&
            (dw_rd(dst, n - 1u) == (uint8_t)GPU_DW_SENT2)) ? 1u : 0u;
}

void gpu_dw_drop_stat(uint32_t* chk, uint32_t* drop, uint32_t* re,
                      uint32_t* first_d, uint32_t* first_c, uint32_t* first_p)
{
    if (chk     != NULL) { *chk     = s_dw_dc_chk; }
    if (drop    != NULL) { *drop    = s_dw_dc_drop; }
    if (re      != NULL) { *re      = s_dw_dc_re; }
    if (first_d != NULL) { *first_d = s_dw_dc_d; }
    if (first_c != NULL) { *first_c = s_dw_dc_c; }
    if (first_p != NULL) { *first_p = s_dw_dc_p; }
}

/* dw_pass_bc_retry() with the question above wrapped around it.  The argument
   list is the operator's draw, unchanged, plus where in the layer it is (ci and
   the pass) - which is what a report of a drop has to carry, because "which
   channel" is the difference between a lost job and a lost layer. */
static bool dw_draw_checked(dhd_handle_t dev, uint32_t src, uint32_t dst,
                            uint32_t tw, uint32_t th,
                            uint32_t dw, uint32_t dh,
                            uint32_t clut_base, const int8_t* k,
                            uint32_t kw, uint32_t kh,
                            dhd_texel_org_t texfmt, dhd_pixel_org_t pixfmt,
                            dhd_address_mode_t amode,
                            float scale, int32_t bias,
                            float u0, float v0, float dux, float dvy,
                            uint32_t a_border, uint32_t ci, uint32_t pass)
{
    const uint32_t n = dw * dh;

    if (n < 4u)
    {
        return dw_pass_bc_retry(dev, src, dst, tw, th, dw, dh, clut_base, k,
                                kw, kh, texfmt, pixfmt, amode, scale, bias,
                                u0, v0, dux, dvy, a_border);
    }

    dw_sent_arm(dst, n);

    if (!dw_pass_bc_retry(dev, src, dst, tw, th, dw, dh, clut_base, k, kw, kh,
                          texfmt, pixfmt, amode, scale, bias, u0, v0, dux, dvy,
                          a_border))
    {
        return false;
    }

    s_dw_dc_chk++;

    if (dw_sent_alive(dst, n) == 0u)
    {
        return true;
    }

    s_dw_dc_drop++;
    if (s_dw_dc_d == 0u)
    {
        s_dw_dc_d = s_dw_dc_chk;
        s_dw_dc_c = ci;
        s_dw_dc_p = pass;
    }

    dw_sent_arm(dst, n);

    if (!dw_pass_bc_retry(dev, src, dst, tw, th, dw, dh, clut_base, k, kw, kh,
                          texfmt, pixfmt, amode, scale, bias, u0, v0, dux, dvy,
                          a_border))
    {
        return false;
    }

    if (dw_sent_alive(dst, n) != 0u)
    {
        s_dw_dc_re++;
    }

    return true;
}

/* Round a float to the nearest integer and clamp to a byte, matching what the
   hardware's scale/bias is expected to do.  Used only by the CPU reference in
   test 3; a disagreement of one is what GPU_DW_GAP_MAX is for. */
static uint8_t dw_ref_byte(float v)
{
    int32_t r = (int32_t)(v + (v >= 0.0f ? 0.5f : -0.5f));

    if (r < 0)   { r = 0; }
    if (r > 255) { r = 255; }
    return (uint8_t)r;
}

/* A cheap 8-bit pseudo-random field, so that test 3's source cannot be
   reproduced by a shift, a constant, or a smoothing of itself.  A linear ramp
   would be - it is the one pattern a wrong m_u0 and a wrong m_dux can both
   reproduce - which is why the reference pattern is noise and not a grating. */
static uint32_t s_dw_rand = 0x12345678u;

static uint8_t dw_rand(void)
{
    s_dw_rand = (s_dw_rand * 1103515245u) + 12345u;
    return (uint8_t)((s_dw_rand >> 16) & 0xFFu);
}

/* The 3x3 the whole probe is a rehearsal for: a signed kernel with all nine
   taps non-zero and both signs present, which is what the model's depthwise
   layers look like (int8 symmetric, zero point 0, 45% of the taps negative).
   Any reading that is not "the CPU's own answer" here is not the operator this
   project is trying to move. */
static const int8_t s_dw_k3[9] = { -1, -2, -1,
                                    0,  0,  0,
                                    1,  2,  1 };

/* A whole-CLUT filler for the two kernel-location tests below.  512 entries is
   the whole allocated slice (DHD_PARAM_TEX_CLUT_DEPTH), so a fill of the full
   slice means a kernel fetch from ANY location inside it lands on our bytes.
   File scope rather than a local because 2 KB on the stack in a boot probe is
   a needless risk. */
static uint32_t s_dw_clutfill[512];

/* Sweep the horizontal difference {-1,0,+1} across a source whose row is its
   own index (`src[i] = i`), so the reading is the TEXEL DISTANCE between the
   outer two taps and nothing else: 2 if the window steps one texel, 4 if it
   steps two.  Translation-invariant, so the unknown m_u0 cancels out - which is
   the point of choosing this pattern over a coordinate-encoding one.

   `dw` is the destination width; du/dx = 1/dw is what makes the destination
   cover the whole texture, so dw = 8 gives one texel per pixel and dw = 4 gives
   two.  Only destination column 1 is read: at two texels per pixel its taps
   reach texels 0 and 4, and the widest case (offset 2 around texel 2) still
   stays inside an 8-wide texture.

   THE SCALE MUST BE 32.0f.  It was 1.0f, and 1.0f cannot answer this probe's
   own question: the transfer function is trunc(sum(w*C) * a_scale/32), so with
   a_scale = 1 the 2 and the 4 this probe exists to tell apart both truncate to
   0.  32.0f is the value dw_kstride() already uses, and it makes
   out = clamp(sum(w*C)) - the probe then reports the tap distance it was
   written to report.

   THE TEXEL FORMAT MUST BE 565, and this was the probe's SECOND defect, still
   reading 0 after the scale was fixed.  It passed GPU_DW_FMT_SIGNED (AL_8) and
   an AL_8 texel contributes NOTHING to a CLUT kernel: four probes in one run
   each read their own bias back, which is the one term they do not vary -
   `gpu dw k9 0` (bias 0), `gpu dw k9s 3` (bias 3), `gpu dw sign 80 -> 128`
   (bias 128) and both step entries (bias 0).  The only kernel arithmetic ever
   verified pixel-by-pixel is `gpu dw kdiff bad 0 worst 1 n 900`, and it is 565
   with the multiplicand in the BLUE field (`gpu dw k888 ... B 8`), which is
   the format and the field the operator itself packs.  So the source below is
   that format, byte-stored the way dw_kstride() stores it. */
static uint32_t dw_step_sweep(dhd_handle_t dev, uint32_t clut_base, uint32_t dw)
{
    int8_t   k[9];
    uint32_t i;

    /* RGB565 texel i, blue field = i % W = the column index, red and green
       zero.  Two byte stores because the scratch is little-endian and uncached:
       the low byte carries the blue field, the high byte is a constant 0. */
    for (i = 0u; i < GPU_DW_PROBE_N; i++)
    {
        dw_wr(GPU_DW_SRC, (i * 2u),      (uint8_t)(i % GPU_DW_PROBE_W));
        dw_wr(GPU_DW_SRC, (i * 2u) + 1u, 0u);
    }
    for (i = 0u; i < GPU_DW_PROBE_N; i++) { dw_wr(GPU_DW_DST, i, 0x5Au); }

    for (i = 0u; i < 9u; i++) { k[i] = 0; }
    k[3] = -1; k[5] = 1;                    /* the middle row: dx = 0, 1, 2 */

    dw_pass(dev, GPU_DW_SRC, GPU_DW_DST, GPU_DW_PROBE_W, GPU_DW_PROBE_H,
            dw, GPU_DW_PROBE_H, clut_base, k, 3u, 3u,
            GPU_DW_FMT_SIGNED_565, E_DHD_PIXEL_ORG_L_8, E_DHD_ACCESS_LINEAR,
            32.0f, 0,
            0.0f, 0.0f,
            1.0f / (float)dw,
            1.0f / (float)GPU_DW_PROBE_H);

    return (uint32_t)dw_rd(GPU_DW_DST, 4u * dw + 1u);
}

/* Test 3, one placement.  Runs the signed 3x3 over the noise field at a given
   m_u0 and returns the interior mismatch count; *all and *worst are the
   plane-wide count and the largest gap, which say different things - see the
   call site.

   m_u0 is a parameter and not a constant because the texture unit's convention
   is not on record: u = 0 could mean the CENTRE of texel 0 or its leading
   EDGE, and those differ by half a texel.  Half a texel is not a rounding
   error, it is a blend of two neighbouring texels, so the wrong one is loud -
   which is what makes measuring it cheaper than deriving it.  blit_convolve
   computes m_u0 as a whole number of texels (nsgpu_lib_dis.txt:28618-28635),
   which reads like the edge convention, but it is the library's crop-offset
   arithmetic and it is being read for a different question.

   `amode` is a parameter for the same reason - see the note where dw_addr_mode
   used to be.  This noise field is the only test in the probe that can tell a
   linear texture from a tiled one, because it is the only one whose source is
   not constant. */
static uint32_t dw_ref_run(dhd_handle_t dev, uint32_t clut_base,
                           dhd_address_mode_t amode, float u0, float v0,
                           uint32_t* all, uint32_t* worst)
{
    uint32_t i, x, y, bad = 0u;

    *all = 0u; *worst = 0u;

    for (i = 0u; i < GPU_DW_T3_W * GPU_DW_T3_H; i++) { dw_wr(GPU_DW_DST, i, 0x5Au); }

    dw_pass(dev, GPU_DW_SRC, GPU_DW_DST, GPU_DW_T3_W, GPU_DW_T3_H,
            GPU_DW_T3_W, GPU_DW_T3_H, clut_base, s_dw_k3, 3u, 3u,
            GPU_DW_FMT_SIGNED, E_DHD_PIXEL_ORG_L_8, amode,
            1.0f, 128, u0, v0,
            1.0f / (float)GPU_DW_T3_W, 1.0f / (float)GPU_DW_T3_H);

    for (y = 0u; y < GPU_DW_T3_H; y++)
    {
        for (x = 0u; x < GPU_DW_T3_W; x++)
        {
            float    acc = 0.0f;
            uint32_t dx, dy, got, exp, d;

            for (dy = 0u; dy < 3u; dy++)
            {
                for (dx = 0u; dx < 3u; dx++)
                {
                    int32_t tv = (int32_t)(int8_t)dw_rd(GPU_DW_SRC,
                                     (y + dy - 1u) * GPU_DW_T3_W + (x + dx - 1u));

                    acc += (float)s_dw_k3[dy * 3u + dx] * (float)tv;
                }
            }

            exp = (uint32_t)dw_ref_byte(acc + 128.0f);
            got = (uint32_t)dw_rd(GPU_DW_DST, y * GPU_DW_T3_W + x);
            d   = (got > exp) ? (got - exp) : (exp - got);

            if (d > GPU_DW_GAP_MAX)
            {
                (*all)++;
                if (d > *worst) { *worst = d; }

                /* The interior only, reported separately: out here the answer
                   is the kernel and the alignment and nothing else, while at
                   the edge it also depends on what dhd_set_texture_border_color
                   put in the padded texels.  An `all` above `int` is therefore
                   a statement about the padding - and the padding has to be the
                   model's input zero point (-128, byte 0x80) for a SAME-padded
                   layer to come out right. */
                if ((x > 0u) && (y > 0u) && (x < GPU_DW_T3_W - 1u) && (y < GPU_DW_T3_H - 1u)) {
                    bad++;
                }
            }
        }
    }
    return bad;
}

/* Is the CLUT reachable AT ALL?  This is the question three failed readings
   could not answer, because "the CLUT returns 0" and "the kernel mode is
   ignored" and "the texture unit just copies the texel" all produce the same
   byte when the source is constant and the bias is the only thing that shows.

   So leave the kernel mechanism out of it: use the CLUT as an ordinary indexed
   palette (E_DHD_CLUT_INDEXED_COLOR, the mode 1 the enum describes) over an
   identity source, so a correct read is self-evident - dst[i] must equal i.
   With point sampling, so the answer is a texel and not a blend, and CLAMP
   wrap, so the border colour cannot enter.

   If this line comes back 0 1 10 200 the CLUT upload, the CLUT offset and the
   indexed lookup are all fine and the fault is specific to kernel mode.  If it
   comes back flat, nothing about the CLUT is working and the kernel was never
   going to. */
static void dw_palette_run(dhd_handle_t dev, uint32_t clut_base,
                           uint32_t src, uint32_t dst, uint32_t w, uint32_t h,
                           dhd_texel_org_t clummode, uint32_t gain)
{
    uint32_t        pal[256];
    dhd_uv_data_t   uv;
    uint32_t        saved_fb;
    dhd_pixel_org_t saved_fmt;
    uint32_t        sav_w, sav_h, sav_p;
    uint32_t        sav_x0, sav_y0, sav_x1, sav_y1, sav_ys;
    uint32_t        i;

    /* grey in all four bytes, so it does not matter which channel an L_8
       destination takes its luminance from.  `gain` is what lets the same
       function serve both questions below: with gain 0 entry i holds i (a
       palette whose output names its own input), with gain 1 it holds i+1
       (a palette that also happens to be a non-zero KERNEL - the four bytes of
       entry 0 are the four weights of a 2x2 QUAD window, and they are all 1).*/
    for (i = 0u; i < 256u; i++) { pal[i] = (i + gain) * 0x01010101u; }
    (void)dhd_set_clut(dev, pal, clut_base, 256u);

    uv.m_u0 = 0.0f;               uv.m_v0 = 0.0f;
    uv.m_dux = 1.0f / (float)w;   uv.m_duy = 0.0f;
    uv.m_dvx = 0.0f;              uv.m_dvy = 1.0f / (float)h;

    dhd_set_uv(dev, 0u, &uv);
    dhd_active_texture_unit(dev, 0u);
    dhd_set_texture_unit_enable(dev, 1u);
    dhd_set_texture_address(dev, (dhd_gpu_ptr_t)src, 0u);
    dhd_set_texture_size(dev, w, h, w);
    dhd_set_texture_format(dev, clummode, E_DHD_ACCESS_LINEAR, 0u);
    dhd_set_texture_clut_base(dev, clut_base);
    /* dhd_set_texture_filter(0,0) is the documented way back from a kernel to
       plain sampling ("after using kernel based convolution use
       dhd_set_texture_filter to switch back"), so this also clears any kernel
       state a previous pass left behind.  It also leaves a KNOWN window behind
       - 2x2, QUAD, sample_shape 0 - which is what makes the kernel-mode call
       below a measurement rather than a guess: dhd_set_texture_kernel() is
       never called for it, so the only thing that changes is the CLUT mode
       inside the format word. */
    dhd_set_texture_filter(dev, 0u, 0u);
    dhd_set_texture_scalebias(dev, 1.0f, 0);
    dhd_set_texture_wrap(dev, E_DHD_TEX_CLAMP, E_DHD_TEX_CLAMP);
    dhd_set_color_operation(dev, E_DHD_OP_COPY);

    saved_fb  = nsgpu_get_framebuffer();
    saved_fmt = dhd_get_framebuffer_format(dev);
    dhd_get_framebuffer_size(dev, &sav_w, &sav_h, &sav_p);
    dhd_get_bbox(dev, &sav_x0, &sav_y0, &sav_x1, &sav_y1, &sav_ys);

    dhd_set_framebuffer_format(dev, E_DHD_PIXEL_ORG_L_8);
    dhd_set_framebuffer_size(dev, w, h, w);
    (void)dhd_set_framebuffer_address(dev, (dhd_gpu_ptr_t)dst);
    dhd_set_bbox(dev, 0u, 0u, w - 1u, h - 1u, 0u);
    dhd_draw(dev, 0u, 0u, (dhd_drawattribute_flag_t)0u);
    dhd_wait(dev, E_DHD_WAIT_WRITEBACK);

    dhd_set_framebuffer_format(dev, saved_fmt);
    dhd_set_framebuffer_size(dev, sav_w, sav_h, sav_p);
    dhd_set_bbox(dev, sav_x0, sav_y0, sav_x1, sav_y1, sav_ys);
    nsgpu_set_framebuffer(saved_fb);
}

/* One cell of the kernel-mode matrix.  Everything except `texfmt` and the
   kernel shape is held fixed and known: the whole CLUT is 0x01010101 (a weight
   of one in every byte of every entry), the source is a constant 10, scale 1.0,
   bias 0.  So the answer is `taps * 10` if that configuration fetches weights
   at all - 90 for a 3x3, 30 for a separable 3x1 - and 0 if it does not.

   The constant source is what makes this immune to the packing question: every
   tap reads the same weight, so ANY entry/byte mapping that reads our kernel
   yields the full sum.  Returns the centre pixel. */
static uint32_t dw_ktest(dhd_handle_t dev, uint32_t clut_base,
                         dhd_texel_org_t texfmt, dhd_address_mode_t amode,
                         uint32_t kw, uint32_t kh)
{
    int8_t   k[25];
    uint32_t i, n = kw * kh;

    for (i = 0u; i < n; i++) { k[i] = 1; }
    for (i = 0u; i < GPU_DW_PROBE_N; i++) { dw_wr(GPU_DW_SRC, i, 10u); }
    for (i = 0u; i < GPU_DW_PROBE_N; i++) { dw_wr(GPU_DW_DST, i, 0x5Au); }

    dw_pass(dev, GPU_DW_SRC, GPU_DW_DST, GPU_DW_PROBE_W, GPU_DW_PROBE_H,
            GPU_DW_PROBE_W, GPU_DW_PROBE_H, clut_base, k, kw, kh,
            texfmt, E_DHD_PIXEL_ORG_L_8, amode,
            1.0f, 0,
            0.0f, 0.0f,
            1.0f / (float)GPU_DW_PROBE_W, 1.0f / (float)GPU_DW_PROBE_H);

    return (uint32_t)dw_rd(GPU_DW_DST,
                           (GPU_DW_PROBE_H / 2u) * GPU_DW_PROBE_W +
                           (GPU_DW_PROBE_W / 2u));
}

/* The library's own recipe, at probe size.

   `gpu cv2 t0 fmt 80A6 am 0 flt 11 da 0` and `sz 320x320 p320 cl 0 en 1` is
   what nsgpu_convolve() leaves in the driver's shadow state.  Read field by
   field against what dw_ktest() passes, only ONE differs:

       texel org    library RGB_565 (6)      probe AL_8 (10)
       CLUT mode    library 5 signed kernel   probe 5    same
       addressing   library LINEAR (0)        probe LINEAR same
       filter       library u,v = 1           probe 1,1  same (the kernel sets it)
       pitch        library == width          probe == width same
       clut base    library 0                 probe 0    same

   So this is dw_ktest with the texel organisation changed and the source filled
   with real 16-bit texels.  0x1084 is the step value the other probes use and
   it is chosen for the same reason here: split as RGB565 its red, green and
   blue fields are all non-zero (2, 4, 4), so a fetch that works cannot come
   back 0 no matter which channel an L_8 destination takes its luminance from.

   `pixfmt` is a parameter so the destination can be varied too - the library
   writes into an RGB565 framebuffer and this probe has only ever written L_8,
   and if the texel organisation is not the answer then that is the next
   candidate.  Returning the raw readback lets the caller print either width. */
/* What does an L_8 destination actually take out of an RGB565 texel?

   `gpu dw k565 l8 1` said a fetch happens, but 1 is not a number that can be
   predicted or compared against anything - nine taps of one over a texel of
   0x1084 has to give some function of (R,G,B) = (2,4,4) and this does not say
   which.  Until that function is known, none of the four questions the probe
   exists to answer can be asked, because every one of them is "does the GPU's
   number equal the CPU's number".

   So measure it: a CONSTANT texel with a single centre tap of one, i.e. the
   output is that one texel and nothing else.  Three primaries then name the
   channel and white names the scale - an L_8 destination that takes luminance
   answers white with 255 and each primary with a different fraction; one that
   takes a single channel from the RGB565 word answers exactly one primary with
   255 and the other two with 0.

   That prediction was WRONG and the measurement said so, which is worth keeping:
   every primary AND white read 0 at a_scale 1.  The scale is not unity - it is
   a_scale/32 - so a texel channel of 31 lands on 0.97 and truncates to 0.  At
   a_scale 32 the same 0x1084 reads 4, and nine taps of it read 1, which is
   1.125 truncated.  Those three points fix output = trunc(sum(w*C) * a_scale/32)
   with C = 4 and, with the header's own "texture blue channel is used as LUT
   index" at davehd_driver.h:238, name C as the BLUE field and a_scale = 32 as
   the value that makes the kernel a plain sum(w*C).  The primaries at a_scale 32
   are the confirmation, and the same file's E_DHD_TEXEL_ORG_RGB_888 is the
   reason to hope the 8-bit-per-channel case does not have the 5-bit ceiling.

   The kernel is packed and uploaded by dhd_set_texture_kernel() itself, so the
   only CLUT entry that matters is entry 0 and this function never touches it. */
static uint32_t dw_ktest565_px(dhd_handle_t dev, uint32_t clut_base,
                               uint16_t texel, uint32_t all_taps, float scale)
{
    int8_t   k[9];
    uint32_t i;

    for (i = 0u; i < 9u; i++) { k[i] = (int8_t)(all_taps ? 1 : 0); }
    if (!all_taps) { k[4] = 1; }                       /* centre tap alone */

    for (i = 0u; i < GPU_DW_PROBE_N; i++) {
        dw_wr(GPU_DW_SRC, i * 2u, (uint8_t)(texel & 0xFFu));
        dw_wr(GPU_DW_SRC, (i * 2u) + 1u, (uint8_t)(texel >> 8));
    }
    for (i = 0u; i < GPU_DW_PROBE_N; i++) { dw_wr(GPU_DW_DST, i, 0x5Au); }

    dw_pass(dev, GPU_DW_SRC, GPU_DW_DST, GPU_DW_PROBE_W, GPU_DW_PROBE_H,
            GPU_DW_PROBE_W, GPU_DW_PROBE_H, clut_base, k, 3u, 3u,
            GPU_DW_FMT_SIGNED_565, E_DHD_PIXEL_ORG_L_8, E_DHD_ACCESS_LINEAR,
            scale, 0,
            0.0f, 0.0f,
            1.0f / (float)GPU_DW_PROBE_W, 1.0f / (float)GPU_DW_PROBE_H);

    return (uint32_t)dw_rd(GPU_DW_DST,
                           (GPU_DW_PROBE_H / 2u) * GPU_DW_PROBE_W +
                           (GPU_DW_PROBE_W / 2u));
}

/* The same constant-texel measurement as dw_ktest565_px, but with the texel
   format and therefore the texel width parameterised.

   dw_ktest565_px is hardwired to a 16-bit RGB565 texel: it writes two bytes per
   texel and names GPU_DW_FMT_SIGNED_565.  Both of those have to change together
   with the format, so rather than a second copy of the whole setup this takes
   the format, the byte width, and the three channel bytes.

   a_tbytes is the texel size in bytes (2 for RGB565, 3 for RGB_888).  The three
   channel bytes go into memory in increasing order, which is what both formats
   document - RGB_565 as "R5, G6, B5 layout" and RGB_888 as "R8, G8, B8 memory
   byte order" - so (c0,c1,c2) is (R,G,B) in both cases and the caller does not
   have to know about the endianness of the short.

   The destination stays L_8: it is the only single-value destination, and
   `gpu dw k565 d565 0000` already ruled out RGB565 as the destination.

   Reads back one destination byte.  A constant source means the kernel window
   sees the same texel at every tap, so what comes back is the weight sum times
   one channel - which is the only way to separate "the channel is wrong" from
   "the tap is wrong". */
static uint32_t dw_kpxb(dhd_handle_t dev, uint32_t clut_base,
                        dhd_texel_org_t texfmt, const uint8_t* pat,
                        uint32_t tbytes, const int8_t* k,
                        float scale, int32_t bias)
{
    uint32_t i, t;

    for (i = 0u; i < GPU_DW_PROBE_N; i++) {
        for (t = 0u; t < tbytes; t++) {
            dw_wr(GPU_DW_SRC, (i * tbytes) + t, pat[t]);
        }
    }
    for (i = 0u; i < GPU_DW_PROBE_N; i++) { dw_wr(GPU_DW_DST, i, 0x5Au); }

    dw_pass(dev, GPU_DW_SRC, GPU_DW_DST, GPU_DW_PROBE_W, GPU_DW_PROBE_H,
            GPU_DW_PROBE_W, GPU_DW_PROBE_H, clut_base, k, 3u, 3u,
            texfmt, E_DHD_PIXEL_ORG_L_8, E_DHD_ACCESS_LINEAR,
            scale, bias,
            0.0f, 0.0f,
            1.0f / (float)GPU_DW_PROBE_W, 1.0f / (float)GPU_DW_PROBE_H);

    return (uint32_t)dw_rd(GPU_DW_DST,
                           (GPU_DW_PROBE_H / 2u) * GPU_DW_PROBE_W +
                           (GPU_DW_PROBE_W / 2u));
}

static uint32_t dw_kpx(dhd_handle_t dev, uint32_t clut_base,
                       dhd_texel_org_t texfmt, uint32_t tbytes,
                       uint8_t c0, uint8_t c1, uint8_t c2,
                       const int8_t* k, float scale, int32_t bias)
{
    uint8_t pat[3];

    /* The three-byte form is what every call site above wants, and dw_kpxb is
       the version that can also express the four-byte 32-bit texels the A8
       question needs.  Kept as a wrapper rather than folded in so the calls
       that already measure something keep measuring the same thing. */
    pat[0] = c0;
    pat[1] = c1;
    pat[2] = c2;

    return dw_kpxb(dev, clut_base, texfmt, pat, (tbytes > 3u) ? 3u : tbytes,
                   k, scale, bias);
}

/* One centre tap of one, i.e. "read this texel and nothing else". */
static const int8_t s_dw_k1[9] = { 0, 0, 0, 0, 1, 0, 0, 0, 0 };

static uint32_t dw_kpx1(dhd_handle_t dev, uint32_t clut_base,
                        dhd_texel_org_t texfmt, uint32_t tbytes,
                        uint8_t c0, uint8_t c1, uint8_t c2, float scale)
{
    return dw_kpx(dev, clut_base, texfmt, tbytes, c0, c1, c2,
                  s_dw_k1, scale, 0);
}

/* All nine taps of one - the same kernel dw_ktest565_px's all_taps builds, so
   the two can be compared tap for tap. */
static uint32_t dw_kpx9(dhd_handle_t dev, uint32_t clut_base,
                        dhd_texel_org_t texfmt, uint32_t tbytes,
                        uint8_t c0, uint8_t c1, uint8_t c2, float scale)
{
    int8_t   k[9];
    uint32_t i;

    for (i = 0u; i < 9u; i++) { k[i] = 1; }
    return dw_kpx(dev, clut_base, texfmt, tbytes, c0, c1, c2, k, scale, 0);
}

/* A centre tap of MINUS one, scale 32, bias 0, over a texel whose channel byte
   is 0xFF.  This is the sign-extension question asked in a way that has only two
   possible answers and both of them exact:

       unsigned texel    0xFF is 255, 255 * -1 = -255, clamps to 0
       signed texel      0xFF is  -1,  -1 * -1 =   +1

   and neither involves a division or a rounding step, so unlike a threshold test
   there is no margin to argue about.  A format whose high bit means "sign" would
   let an int8 activation go in raw; one that does not needs the +128 offset,
   which is a legitimate answer too and only changes the constant folded into the
   bias. */
static const int8_t s_dw_km1[9] = { 0, 0, 0, 0, -1, 0, 0, 0, 0 };

static uint32_t dw_kpx_neg(dhd_handle_t dev, uint32_t clut_base,
                           dhd_texel_org_t texfmt, uint32_t tbytes,
                           uint8_t c0, uint8_t c1, uint8_t c2)
{
    return dw_kpx(dev, clut_base, texfmt, tbytes, c0, c1, c2,
                  s_dw_km1, 32.0f, 0);
}

/* Does the row pitch mean texels or bytes?

   Every measurement above uses a CONSTANT source, and a constant is constant
   under any stride - so none of them can see a pitch error at all.  That is
   exactly the wrong blind spot to carry into the design, where the plane is
   8-bit data and a stride that is 2x or 3x too large scrambles every row after
   the first.

   dhd_set_texture_size() is handed the pitch in texels (tw, same as the width);
   whether the hardware multiplies that by the texel size for a LINEAR texture is
   the question, and it is not answerable from the driver source because the
   burst-pitch recompute in dhd_set_texture_format() (davehd_driver_txp_inl.h:
   368-375) only runs for the TILED modes - "if (a_address >> 3) != 0" - and this
   port is LINEAR.

   So: texel (x,y) carries y*W+x in its blue field, one centre tap reads it back
   unchanged, and the destination is read down column 0.  A correct texel pitch
   gives 0, 8, 16, 24; a pitch that is being treated as bytes gives a stride of
   half a row (2-byte texel) or a third of a row (3-byte texel) and the column
   reads short.  The same run also shows whether destination x lands on texel x
   at all, which is the u0 convention question, since it is the only test here
   whose source varies with position. */
static bool dw_kstride(dhd_handle_t dev, uint32_t clut_base,
                       dhd_texel_org_t texfmt, uint32_t tbytes,
                       uint32_t* out)
{
    uint32_t x, y, v, o = 0u;

    for (y = 0u; y < GPU_DW_PROBE_H; y++) {
        for (x = 0u; x < GPU_DW_PROBE_W; x++) {
            v = (y * GPU_DW_PROBE_W) + x;
            if (tbytes == 2u) {
                dw_wr(GPU_DW_SRC, o, (uint8_t)(v & 0x1Fu));   /* RGB565 blue field */
                dw_wr(GPU_DW_SRC, o + 1u, 0u);
            } else {
                dw_wr(GPU_DW_SRC, o, 0u);
                dw_wr(GPU_DW_SRC, o + 1u, 0u);
                dw_wr(GPU_DW_SRC, o + 2u, (uint8_t)v);        /* RGB888 blue byte */
            }
            o += tbytes;
        }
    }
    for (x = 0u; x < GPU_DW_PROBE_N; x++) { dw_wr(GPU_DW_DST, x, 0x5Au); }

    if (!dw_pass(dev, GPU_DW_SRC, GPU_DW_DST, GPU_DW_PROBE_W, GPU_DW_PROBE_H,
                 GPU_DW_PROBE_W, GPU_DW_PROBE_H, clut_base, s_dw_k1, 3u, 3u,
                 texfmt, E_DHD_PIXEL_ORG_L_8, E_DHD_ACCESS_LINEAR,
                 32.0f, 0,
                 0.0f, 0.0f,
                 1.0f / (float)GPU_DW_PROBE_W, 1.0f / (float)GPU_DW_PROBE_H))
    {
        return false;
    }

    for (x = 0u; x < 4u; x++) {
        out[x] = (uint32_t)dw_rd(GPU_DW_DST, x * GPU_DW_PROBE_W);
    }
    return true;
}

/* The Step-1 acceptance test: a known 8-bit plane, a known kernel, one GPU 3x3
   pass, and the identical sum done on the CPU, compared byte for byte over the
   interior.

   The plane goes in the BLUE byte of RGB_888 texels - the channel the single-tap
   measurements named, on a format where that byte is a plain byte - so the fill
   is a straight copy with a 3-byte stride and no bit packing.

   The CPU side computes the MODEL, not the ideal convolution, because the model
   is what Step 2 gets built on and it is the model that has to be right:

       out = clamp( sum(w * (a >> 3)) * scale/32 , 0, 255 )

   with the activation truncated to the 5 bits the datapath actually has
   (measured: 0x40 -> 8 and 0x80 -> 16 of a possible 255, which is >>3 and not a
   rescale to 31), the weights raw signed 8-bit (measured: a weight of -1 over a
   0xFF texel gives 0, so the weight is applied and signed and the texel is not),
   and scale 32 cancelling the /32 so the sum is exact.  Four single-tap points
   fit that; 900 interior pixels either agree or they do not.

   The kernel is a Laplacian - four taps of +1 around a centre of -4 - chosen so
   that the CPU reference is valid under EITHER answer to the output-range
   question: the largest value it can produce is 1*31 = 31 and the smallest is
   -4*31, which clamps.  So this test does not silently depend on the output
   being 8 bits wide, and the separate 9-tap-of-white line answers that
   question on its own.

   Border pixels are excluded: E_DHD_TEX_BORDER with a border colour of
   0x80808080 contributes (0x80 >> 3) = 16 per out-of-range tap, which is a
   second question and not this one. */
static const int8_t s_dw_k3lap[9] = { 0, 1, 0, 1, -4, 1, 0, 1, 0 };

static bool dw_kdiff888(dhd_handle_t dev, uint32_t clut_base,
                        uint32_t* out_bad, uint32_t* out_worst, uint32_t* out_px)
{
    static uint8_t src[GPU_DW_T3_W * GPU_DW_T3_H];
    uint32_t x, y, dx, dy, i, bad = 0u, worst = 0u, px = 0u;

    /* Deterministic, not random: a failure has to be reproducible.  (i*37)+11
       has period 256 over the 1024 pixels, so every 5-bit code appears. */
    for (i = 0u; i < (GPU_DW_T3_W * GPU_DW_T3_H); i++) {
        src[i] = (uint8_t)((i * 37u) + 11u);
    }

    for (i = 0u; i < (GPU_DW_T3_W * GPU_DW_T3_H); i++) {
        dw_wr(GPU_DW_SRC, (i * 3u),      0u);
        dw_wr(GPU_DW_SRC, (i * 3u) + 1u, 0u);
        dw_wr(GPU_DW_SRC, (i * 3u) + 2u, src[i]);
    }
    for (i = 0u; i < (GPU_DW_T3_W * GPU_DW_T3_H); i++) { dw_wr(GPU_DW_DST, i, 0x5Au); }

    if (!dw_pass(dev, GPU_DW_SRC, GPU_DW_DST, GPU_DW_T3_W, GPU_DW_T3_H,
                 GPU_DW_T3_W, GPU_DW_T3_H, clut_base,
                 s_dw_k3lap, 3u, 3u,
                 GPU_DW_FMT_888, E_DHD_PIXEL_ORG_L_8, E_DHD_ACCESS_LINEAR,
                 32.0f, 0,
                 0.0f, 0.0f,
                 1.0f / (float)GPU_DW_T3_W, 1.0f / (float)GPU_DW_T3_H))
    {
        return false;
    }

    for (y = 1u; y < (GPU_DW_T3_H - 1u); y++) {
        for (x = 1u; x < (GPU_DW_T3_W - 1u); x++) {
            int32_t  acc = 0;
            uint32_t got, want;

            for (dy = 0u; dy < 3u; dy++) {
                for (dx = 0u; dx < 3u; dx++) {
                    acc += (int32_t)s_dw_k3lap[(dy * 3u) + dx] *
                           (int32_t)(src[((y + dy - 1u) * GPU_DW_T3_W) +
                                         (x + dx - 1u)] >> 3);
                }
            }
            if (acc < 0)   { acc = 0; }
            if (acc > 255) { acc = 255; }

            want = (uint32_t)acc;
            got  = (uint32_t)dw_rd(GPU_DW_DST, (y * GPU_DW_T3_W) + x);
            px++;

            if (got != want) {
                uint32_t d = (got > want) ? (got - want) : (want - got);
                if (d > worst) { worst = d; }
                if (d > GPU_DW_GAP_MAX) { bad++; }
            }
        }
    }

    *out_bad   = bad;
    *out_worst = worst;
    *out_px    = px;
    return true;
}

/* Does the bias get added BEFORE or AFTER the 0..255 clamp?

   This is the question that decides whether a depthwise layer is implementable
   at all, and the model confirmed by dw_kdiff888() does not answer it because
   that test ran with bias 0 - where "clamp first" and "clamp last" are the same
   function.

   Why it matters: a depthwise sum is not a small number.  Nine taps of a signed
   8-bit weight against a 5-bit activation spans +-(127*31*9) = +-35433, so the
   requantisation HAS to happen inside the pass, as
   out = clamp(trunc(sum * M) + out_zp', 0, 255).  M scales the sum into the
   int8 range, which means the intermediate is negative about half the time.  If
   the texture value is clamped to 0 before the bias is added, every negative
   accumulation is silently lost and the layer produces garbage - not a rounding
   error, a different operator.

   The measurement is a centre tap of MINUS one over an RGB_888 blue byte of
   0x40, so C5 = 8 and at a_scale 32 the sum is exactly -8:

       bias added before the clamp   -8 + 128 = 120
       clamp first, bias after        0 + 128 = 128

   and a positive control, a centre tap of PLUS one, which reads 8 + 128 = 136
   under either rule and so confirms the bias is applied at all (it is: the
   all-bias symptom in this file's history was an output of exactly the bias). */
static uint32_t dw_kbias(dhd_handle_t dev, uint32_t clut_base,
                         const int8_t* k, int32_t bias)
{
    return dw_kpx(dev, clut_base, GPU_DW_FMT_888, 3u, 0x00u, 0x00u, 0x40u,
                  k, 32.0f, bias);
}

/* Is there an 8-bit way into the kernel?

   The kernel multiplies the CLUT weight by "the texture blue channel", and for
   an 8-bit blue byte that turns out to mean blue >> 3: 0x40 reads 8, 0x80 reads
   16, 0xFF reads 31.  Five bits.  RGB_888 does not help, because 8 bits per
   channel is what the byte holds, not what the kernel takes.

   That matters more than it looks.  An int8 activation has 256 levels and this
   path can carry 32 of them, so every layer's input would be quantised to 3
   fewer bits than TFLM uses - a systematic error, not the +-1 LSB that
   dw_kdiff888() bounded, and almost certainly past any per-layer tolerance
   worth setting.  Step 2 built on it would fall back on every layer.

   So look for a wider multiplicand before designing around a narrow one.  The
   one part of a texel the blue-field rule says nothing about is the alpha byte
   of the two 32-bit formats, which is why they are here.

   The probe value is 0x7F, chosen so that all four candidate behaviours read
   differently and one sweep of the four byte positions separates them:

       127  the byte is an 8-bit multiplicand        <- what we want
        31  the byte is a 565 blue field, taken whole
        15  the byte is a multiplicand after >> 3
         0  the byte is not read at all

   0x40 would not have done: its low five bits are zero, so a 565 blue field
   holding it is indistinguishable from a byte nothing reads. */
static uint32_t dw_ka8(dhd_handle_t dev, uint32_t clut_base,
                       dhd_texel_org_t fmt, uint32_t abyte)
{
    uint8_t pat[4];

    pat[0] = 0u;
    pat[1] = 0u;
    pat[2] = 0u;
    pat[3] = 0u;
    pat[abyte] = 0x7Fu;

    return dw_kpxb(dev, clut_base, fmt, pat, 4u, s_dw_k1, 32.0f, 0);
}

static uint32_t dw_ktest565(dhd_handle_t dev, uint32_t clut_base,
                            dhd_pixel_org_t pixfmt)
{
    int8_t   k[9];
    uint32_t i;

    for (i = 0u; i < 9u; i++) { k[i] = 1; }
    for (i = 0u; i < GPU_DW_PROBE_N; i++) {
        dw_wr(GPU_DW_SRC, i * 2u, 0x84u);
        dw_wr(GPU_DW_SRC, (i * 2u) + 1u, 0x10u);
    }
    for (i = 0u; i < GPU_DW_PROBE_N; i++) { dw_wr(GPU_DW_DST, i, 0x5Au); }

    dw_pass(dev, GPU_DW_SRC, GPU_DW_DST, GPU_DW_PROBE_W, GPU_DW_PROBE_H,
            GPU_DW_PROBE_W, GPU_DW_PROBE_H, clut_base, k, 3u, 3u,
            GPU_DW_FMT_SIGNED_565, pixfmt, E_DHD_ACCESS_LINEAR,
            1.0f, 0,
            0.0f, 0.0f,
            1.0f / (float)GPU_DW_PROBE_W, 1.0f / (float)GPU_DW_PROBE_H);

    if (pixfmt == E_DHD_PIXEL_ORG_L_8) {
        return (uint32_t)dw_rd(GPU_DW_DST,
                               (GPU_DW_PROBE_H / 2u) * GPU_DW_PROBE_W +
                               (GPU_DW_PROBE_W / 2u));
    }
    return (uint32_t)dw_rd16(GPU_DW_DST,
                             (GPU_DW_PROBE_H / 2u) * GPU_DW_PROBE_W +
                             (GPU_DW_PROBE_W / 2u));
}

bool gpu_dw_probe(uint32_t* out_bad)
{
    dhd_handle_t dev = (dhd_handle_t)a_handle;
    uint32_t clut_base = 0u;
    uint32_t i, bad, bad_all, worst, bad2, all2, worst2;
    uint32_t n_t2, all3, worst3;
    uint32_t t0, t1;
    int8_t   k1[9];
    int8_t   k9[9];
    uint32_t step1, step2, val;
    uint32_t pal0, pal1, pal2, pal3, k9_1, k9_2;
    uint32_t km_sn, km_t2, km_565, km_5x5;
    uint32_t km_565l8, km_565d;
    uint32_t k5w_x32, k5w_x9;
    uint32_t k5w_w32, k5w_r32, k5w_g32, k5w_b32;
    uint32_t k888_b40, k888_b80, k888_b00;
    uint32_t k888_r40, k888_g40, k888_c40;
    uint32_t k888_x9, k888_neg;
    uint32_t kst565[4], kst888[4];
    uint32_t k888_ff, k5w_wx9;
    uint32_t kd_bad, kd_worst, kd_px;
    uint32_t pk_m1_0, pk_m1_1, pk_m1_10, pk_m5_0, pk_m5_1, pk_m5_10;

    if (out_bad != NULL) { *out_bad = 0u; }

    if (!s_ready)
    {
        UART_SendString("gpu dw SKIP (not ready)\r\n");
        return false;
    }

    /* One allocation for every pass below.  The library allocates and frees a
       CLUT per call; holding it is the change this stage is built on, so the
       probe holding it too is the point and not a shortcut. */
    if (dhd_alloc_clut(dev, 512u, &clut_base) != 0u)
    {
        UART_SendString("gpu dw FAIL alloc_clut\r\n");
        return false;
    }

    sprintf(Message, "gpu dw clut base %lu\r\n", (unsigned long)clut_base);
    UART_SendString(Message);

    /* The ambient state this probe starts from, with all 512 CLUT entries held.
       c4 on the third line must FAIL here (E_DHD_CLUT_E -> 0xFFFFFFFF) for the
       same reason it must succeed at dwF: if it does not, the allocator is
       double-booking a slice we already hold. */
    gpu_env_state("dwA");

    /* ---------------------------------------------------------------
       Step 0a: the two axes that distinguish this probe from the library's
       WORKING convolve path.

       Everything the two share is already proven good on this board: the CLUT
       upload, the CLUT offset register, the indexed lookup (the `pal` line
       below reads back 0 1 10 200), the 8-bit texel read, LINEAR addressing,
       the L_8 destination and scale/bias.  So the fault has to be in the part
       that is specific to kernel mode, and there are exactly two ways our
       configuration differs from `nsgpu_convolve` - which is measured working,
       it is the 3-tap binomial in the grab path:

           the MODE      ours   E_DHD_CLUT_KERNEL_SIGNED (5)
                         theirs E_DHD_CLUT_KERNEL (4) - the library passes
                         `framebuffer_format | 0x80`, and 0x80 >> 5 == 4

           the SHAPE     ours   a non-separable 3x3  (QUAD, 4 entries)
                         theirs a separable 1xN      (ROW/COLUMN, ceil(n/4))

       Both are one-line changes, so read all four corners instead of guessing
       which axis matters.  The whole CLUT is 0x01010101 - a weight of one in
       every byte of every entry - because that is what makes the matrix immune
       to the packing question: whichever entry and whichever byte the fetch
       lands on, a fetch that works reads a 1.

           90  a 3x3 that works       (nine taps of one on a constant 10)
           30  a 3x1 that works       (three taps)
            0  that corner does not fetch weights at all

       An earlier build already ran the all-ones CLUT with the 3x3 in mode 5 and
       read 0.  That is what retired the "wrong CLUT entry" theory: no CLUT
       offset can produce a zero when every entry is one, so the fetch is not
       reading the CLUT, not reading the wrong part of it.  What is left is
       which of these two axes the fetch depends on.

       Measured: NONE of them.  All four corners read 0, so the mode and the
       kernel shape are not the variable either, and step 0b below moves on to
       the two axes that are left.                                           */
    for (i = 0u; i < 512u; i++) { s_dw_clutfill[i] = 0x01010101u; }
    (void)dhd_set_clut(dev, s_dw_clutfill, clut_base, 512u - clut_base);

    km_sn = dw_ktest(dev, clut_base, GPU_DW_FMT_SIGNED,   E_DHD_ACCESS_LINEAR, 3u, 3u);

    /* ---------------------------------------------------------------
       Step 0b: the two axes that the mode/shape matrix did NOT clear.

       Mode 4 vs 5 and QUAD vs ROW all read 0, so neither is the variable -
       which leaves the parts of `blit_convolve`'s setup that this probe still
       does differently, and the library's convolve is measured working on this
       board (both `gpu conv` and `gpu cv2` produce real output).  Its
       texture is a GPU FRAMEBUFFER: TILED addressing on an RGB565 plane.  Ours
       is a linear AL_8 plane in the GPU slot.  Both of those are one line here.

           90  that axis is the blocker
            0  it is not

       The RGB565 case reads a source that is a constant 10 byte, so every
       16-bit texel is 0x0A0A and the value does not depend on which channel or
       which alignment the format picks - which is what keeps it a test of the
       format rather than a test of the fill pattern.                        */
    km_t2  = dw_ktest(dev, clut_base, GPU_DW_FMT_SIGNED,
                      E_DHD_ACCESS_VIRTUAL_TILED_2, 3u, 3u);
    km_565 = dw_ktest(dev, clut_base, GPU_DW_FMT_565,
                      E_DHD_ACCESS_LINEAR, 3u, 3u);
    km_5x5 = dw_ktest(dev, clut_base, GPU_DW_FMT_SIGNED,
                      E_DHD_ACCESS_LINEAR, 5u, 5u);

    sprintf(Message, "gpu dw km sn %lu t2 %lu f565 %lu k5 %lu (3x3=90 5x5=250)\r\n",
            (unsigned long)km_sn, (unsigned long)km_t2,
            (unsigned long)km_565, (unsigned long)km_5x5);
    UART_SendString(Message);

    /* ---------------------------------------------------------------
       Step 0d: the library's texel organisation, which the read-back named.

       `gpu cv2 t0 fmt 80A6` decomposes to RGB_565 (6) in CLUT mode 5 - signed
       kernel, LINEAR, filter on, pitch == width, clut base 0.  Every one of
       those is what dw_ktest() already passes EXCEPT the texel org, which is
       the one field the corner matrix above left in the wrong cell: `f565` did
       try RGB565, but against mode 4, not mode 5.  Hence a zero that told us
       nothing about RGB565 and a great deal about mode 4.

       So: same everything, RGB565 texels, mode 5.  Two destinations, because
       the library writes into an RGB565 framebuffer and this probe has only
       ever written L_8 - so if the texel org is not the whole answer, the
       destination format is the next candidate and it costs one more pass to
       rule it in or out in the same flash.

       The source is 0x1084, split as R=2 G=4 B=4 - all three non-zero, so a
       fetch that works cannot come back 0 whichever channel an L_8
       destination reads.  Nine taps of one on that is roughly 25 in luminance
       and 0x1084 x 9 in RGB565; the exact figure is not the point, non-zero
       is. */
    km_565l8 = dw_ktest565(dev, clut_base, E_DHD_PIXEL_ORG_L_8);
    km_565d  = dw_ktest565(dev, clut_base, E_DHD_PIXEL_ORG_RGB_565);

    sprintf(Message, "gpu dw k565 l8 %lu d565 %04X (0=no fetch)\r\n",
            (unsigned long)km_565l8, (unsigned)km_565d);
    UART_SendString(Message);

    /* ---------------------------------------------------------------
       Step 0e: the reading that named the scale.

       A constant texel with one centre tap of one, so the output is that texel
       and nothing else.  At a_scale 1 every one of these - white and all three
       primaries included - reads 0, which is the answer the earlier version of
       this block did not anticipate: the scale is a_scale/32, so the largest
       channel in the format lands on 0.97 and truncates away.  At a_scale 32 the
       same 0x1084 texel reads 4 and nine taps of it read 1, and those two plus
       the 0 are what fix output = trunc(sum(w*C) * a_scale/32).

       The primaries at that same scale are step 0f below, not here.         */
    k5w_x32 = dw_ktest565_px(dev, clut_base, 0x1084u, 0u, 32.0f);
    k5w_x9 = dw_ktest565_px(dev, clut_base, 0x1084u, 1u,  1.0f);

    sprintf(Message, "gpu dw k5p 1084 x32 %lu x9t %lu (x9 should be 9x x)\r\n",
            (unsigned long)k5w_x32, (unsigned long)k5w_x9);
    UART_SendString(Message);

    /* ---------------------------------------------------------------
       Step 0f: the primaries at a_scale 32, where they are actually visible.

       The scale-32 reading of 0x1084 is 4, so 4 * 32/32 = 4 and the kernel is a
       plain sum(w*C) at this scale - which is why these four are read here and
       not at scale 1, where every one of them was 0.  RGB565's blue field is 31
       for white and for pure blue, 0 for pure red and pure green:

           blue field     w 31  r  0  g  0  b 31
           luminance      w 31  r  9  g 18  b  3
           red field      w 31  r 31  g  0  b  0
           green field    w 31  r  0  g 31  b  0

       so four numbers separate all four candidates, and each is a single tap of
       one so no sum is involved.                                           */
    k5w_w32 = dw_ktest565_px(dev, clut_base, 0xFFFFu, 0u, 32.0f);
    k5w_r32 = dw_ktest565_px(dev, clut_base, 0xF800u, 0u, 32.0f);
    k5w_g32 = dw_ktest565_px(dev, clut_base, 0x07E0u, 0u, 32.0f);
    k5w_b32 = dw_ktest565_px(dev, clut_base, 0x001Fu, 0u, 32.0f);

    sprintf(Message, "gpu dw k5p s32 w %lu r %lu g %lu b %lu\r\n",
            (unsigned long)k5w_w32, (unsigned long)k5w_r32,
            (unsigned long)k5w_g32, (unsigned long)k5w_b32);
    UART_SendString(Message);

    /* ---------------------------------------------------------------
       Step 0g: the 8-bit-per-channel texel, which is the whole reason an int8
       activation could go up unshifted.

       Every format that has worked so far is 16-bit RGB565, whose best channel
       is 6 bits - and the one format documented as "a single value for all
       channels" (AL_8) is exactly the one that returns 0 under a CLUT kernel, so
       "the value is 8 bits" is not a property that can be inferred from the
       others.  E_DHD_TEXEL_ORG_RGB_888 is 24-bit and is the only format in the
       list with 8 bits in every channel.

       0x80 in a channel is 128.  No RGB565 channel can express 128 - its blue
       field tops out at 31 and 31 is what the line above measures for white - so
       a 128 here is unambiguous: the format works and the channel is 8 bits.
       0x40 and 0x00 follow for the two smaller steps, and they distinguish "8
       bits but shifted" from "8 bits, value taken as-is".                   */
    k888_b40 = dw_kpx1(dev, clut_base, GPU_DW_FMT_888, 3u, 0x00u, 0x00u, 0x40u, 32.0f);
    k888_b80 = dw_kpx1(dev, clut_base, GPU_DW_FMT_888, 3u, 0x00u, 0x00u, 0x80u, 32.0f);
    k888_b00 = dw_kpx1(dev, clut_base, GPU_DW_FMT_888, 3u, 0x00u, 0x00u, 0x00u, 32.0f);

    sprintf(Message, "gpu dw k888 40 %lu 80 %lu 00 %lu (want 64 128 0)\r\n",
            (unsigned long)k888_b40, (unsigned long)k888_b80,
            (unsigned long)k888_b00);
    UART_SendString(Message);

    /* One 0x40 in each of the three bytes in turn, so that WHICH channel the
       kernel multiplies is answered on this format directly rather than
       transferred from the RGB565 reading above - the two formats need not
       agree, and 0x40 in the red byte against 0x40 in the blue byte is the
       difference between the two answers being right and being a coincidence. */
    k888_r40 = dw_kpx1(dev, clut_base, GPU_DW_FMT_888, 3u, 0x40u, 0x00u, 0x00u, 32.0f);
    k888_g40 = dw_kpx1(dev, clut_base, GPU_DW_FMT_888, 3u, 0x00u, 0x40u, 0x00u, 32.0f);
    k888_c40 = dw_kpx1(dev, clut_base, GPU_DW_FMT_888, 3u, 0x00u, 0x00u, 0x40u, 32.0f);

    sprintf(Message, "gpu dw k888 R %lu G %lu B %lu (which channel)\r\n",
            (unsigned long)k888_r40, (unsigned long)k888_g40,
            (unsigned long)k888_c40);
    UART_SendString(Message);

    /* Two checks on the same format: nine taps of a 0x10 must be exactly nine
       times one tap of it, which tests that the accumulation is a real sum over
       the window and not a single tap plus something; and 0xFF under a weight of
       minus one separates a sign-extended texel (+1) from an unsigned one
       (-255, clamped to 0) with no rounding step in between. */
    k888_x9  = dw_kpx9(dev, clut_base, GPU_DW_FMT_888, 3u, 0x00u, 0x00u, 0x10u, 32.0f);
    k888_neg = dw_kpx_neg(dev, clut_base, GPU_DW_FMT_888, 3u, 0x00u, 0x00u, 0xFFu);

    sprintf(Message, "gpu dw k888 10x9 %lu (want 144) neg %lu (1=signed texel)\r\n",
            (unsigned long)k888_x9, (unsigned long)k888_neg);
    UART_SendString(Message);

    /* ---------------------------------------------------------------
       Step 0h: the row pitch, which every test above is blind to.

       A constant source reads the same under any stride, so nothing measured so
       far would notice a pitch that is being interpreted as bytes instead of
       texels.  Column 0 of a coordinate-encoded source is the shortest thing
       that would: 0 8 16 24 means the pitch is texels, anything shorter means it
       is not.  Both formats are run because the factor differs - 2 for RGB565
       and 3 for RGB_888 - so one of them failing and the other not is itself a
       diagnosis.  A run that reads 0 8 16 24 for both also settles the u0
       convention: destination (0,y) lands on texel (0,y).                  */
    if (dw_kstride(dev, clut_base, GPU_DW_FMT_SIGNED_565, 2u, kst565) &&
        dw_kstride(dev, clut_base, GPU_DW_FMT_888,        3u, kst888))
    {
        sprintf(Message,
                "gpu dw kst 565 %lu %lu %lu %lu 888 %lu %lu %lu %lu (want 0 8 16 24)\r\n",
                (unsigned long)kst565[0], (unsigned long)kst565[1],
                (unsigned long)kst565[2], (unsigned long)kst565[3],
                (unsigned long)kst888[0], (unsigned long)kst888[1],
                (unsigned long)kst888[2], (unsigned long)kst888[3]);
        UART_SendString(Message);
    }
    else
    {
        UART_SendString("gpu dw kst pass failed\r\n");
    }

    /* ---------------------------------------------------------------
       Step 0i: the output range, which decides whether a DW layer's result can
       even be carried to the next layer.

       Nine taps of one over a white RGB565 texel sums 9 * 31 = 279.  Two things
       can happen to that:

           255   the accumulator is wide and only the destination is 8-bit
            31   the whole datapath is 5-bit and the result saturates there

       and the difference decides the design: at 31 every activation coming out
       of a GPU layer has 5 bits, so the layer after it needs the same 5-bit
       packing and the model in dw_kdiff888() is wrong by construction.  At 255
       only the INPUT is 5-bit, the sum is wide, and clamp(sum(w*(a>>3)),0,255)
       is the right model - which is what that reference computes.

       Run together with 0xFF through a single +1 tap, which is the same
       >>3 question asked where the answer is 31 rather than an ambiguous
       16: 0xFF >> 3 = 31 exactly, and no rescale to a 5-bit maximum can
       produce 31 from 255 by any route other than >>3.
       The diff is the acceptance test that has to pass before step 2.       */
    k888_ff  = dw_kpx1(dev, clut_base, GPU_DW_FMT_888, 3u, 0u, 0u, 0xFFu, 32.0f);
    k5w_wx9  = dw_kpx9(dev, clut_base, GPU_DW_FMT_SIGNED_565, 2u,
                       0xFFu, 0xFFu, 0u, 32.0f);

    sprintf(Message, "gpu dw k888 ff %lu (31=>>3) 565 wx9 %lu (255=wide out)\r\n",
            (unsigned long)k888_ff, (unsigned long)k5w_wx9);
    UART_SendString(Message);

    /* The model goes out as a literal rather than inside the format string: with
       three %lu the combined line is 60-odd characters against a 64-byte global
       Message[], and the compiler flagged the version with the prose in it as
       expanding to at least 70.  Splitting it is the fix; the numbers stay in
       one line so they are greppable. */
    UART_SendString("gpu dw kdiff model clamp(sum(w*(a>>3)),0,255) 32x32 interior\r\n");

    if (dw_kdiff888(dev, clut_base, &kd_bad, &kd_worst, &kd_px))
    {
        sprintf(Message, "gpu dw kdiff bad %lu worst %lu n %lu\r\n",
                (unsigned long)kd_bad, (unsigned long)kd_worst, (unsigned long)kd_px);
        UART_SendString(Message);
    }
    else
    {
        UART_SendString("gpu dw kdiff pass failed\r\n");
    }

    /* ---------------------------------------------------------------
       The bias-order question.  dw_kdiff888() above fixed the transfer
       function but ran with bias 0, and at bias 0 "add the bias then clamp"
       and "clamp then add the bias" are the same function - so the model it
       confirmed is silent on which one the hardware does.  See dw_kbias().

       This is the last unknown that gates step 2, because a depthwise sum
       spans +-35433 and the requantisation must therefore happen inside the
       pass, with a negative intermediate about half the time.             */
    {
        uint32_t kb_neg, kb_pos;

        kb_neg = dw_kbias(dev, clut_base, s_dw_km1, 128);
        kb_pos = dw_kbias(dev, clut_base, s_dw_k1,  128);

        sprintf(Message, "gpu dw kb %lu %lu +128 (neg120=ok neg128=lost)\r\n",
                (unsigned long)kb_neg, (unsigned long)kb_pos);
        UART_SendString(Message);
    }

    /* ---------------------------------------------------------------
       What the working format actually costs.

       The `gpu dw 64x64 us 181` line further down convolves an AL_8 texture,
       and AL_8 under a CLUT kernel reads 0 - so that reading timed a draw over
       a texture whose every texel was nothing, on a format step 2 cannot use.
       It is not a number to design against.

       This one is RGB_888, the format that works, at the shape of the model's
       largest depthwise layer (40x40, which is op9's output).  It goes through
       dw_pass, which re-uploads the kernel into the CLUT on every call - and a
       real layer uploads all of its channels once and thereafter only re-points
       the base pointer.  So this OVERSTATES the per-channel cost by the upload;
       read it as an upper bound, and take the design's number from the
       implemented path.  The source is not refilled first: timing depends on
       the geometry, not the values.                                    */
    t0 = DWT->CYCCNT;
    for (i = 0u; i < 8u; i++)
    {
        (void)dw_pass(dev, GPU_DW_SRC, GPU_DW_DST, 40u, 40u, 40u, 40u, clut_base,
                      s_dw_k3, 3u, 3u, GPU_DW_FMT_888, E_DHD_PIXEL_ORG_L_8,
                      E_DHD_ACCESS_LINEAR, 32.0f, 0,
                      0.0f, 0.0f, 1.0f / 40.0f, 1.0f / 40.0f);
    }
    t1 = DWT->CYCCNT;

    sprintf(Message, "gpu dw kcost 40x40 us %lu (x8, incl upload)\r\n",
            (unsigned long)((t1 - t0) / 600u / 8u));
    UART_SendString(Message);

    /* ---------------------------------------------------------------
       Is there an 8-bit way into the kernel?  See dw_ka8() for why the answer
       decides whether step 2 is worth building at all.

       0x7F in each of the four bytes, for each of the two 32-bit alpha
       formats, so one sweep names the layout and the width together:

         127 = an 8-bit multiplicand (build step 2 on this format)
          31 = a 565 blue field taken whole
          15 = a multiplicand after >> 3
           0 = the byte is not read                                     */
    {
        uint32_t a8[4], a8b[4];
        uint32_t b;

        for (b = 0u; b < 4u; b++)
        {
            a8[b]  = dw_ka8(dev, clut_base, GPU_DW_FMT_565A8, b);
            a8b[b] = dw_ka8(dev, clut_base, GPU_DW_FMT_A8565, b);
        }

        UART_SendString("gpu dw ka8 127=8bit 31=565 15=shift 0=unread, byte 0..3\r\n");

        sprintf(Message, "gpu dw ka8 A %lu %lu %lu %lu\r\n",
                (unsigned long)a8[0], (unsigned long)a8[1],
                (unsigned long)a8[2], (unsigned long)a8[3]);
        UART_SendString(Message);

        sprintf(Message, "gpu dw ka8 B %lu %lu %lu %lu\r\n",
                (unsigned long)a8b[0], (unsigned long)a8b[1],
                (unsigned long)a8b[2], (unsigned long)a8b[3]);
        UART_SendString(Message);

        /* And, whatever the answer, does the >> 3 truncate or round?  It sets
           how the activation is mapped onto the levels that survive, so it is
           worth one number now rather than a guess later. */
        sprintf(Message, "gpu dw krnd 0F %lu (1=trunc 2=round)\r\n",
                (unsigned long)dw_kpx1(dev, clut_base, GPU_DW_FMT_888, 3u,
                                       0u, 0u, 0x0Fu, 32.0f));
        UART_SendString(Message);
    }

    /* ---------------------------------------------------------------
       Step 0c: three lines that exist only to tell apart three failures that
       had produced byte-identical output for two builds.

       Every reading so far was constant - bias and nothing else - which is
       equally consistent with "the CLUT reads 0", "kernel mode is ignored" and
       "the texture unit just copies the texel".  A constant source and a bias
       cannot separate those.  These can.                                 */

    for (i = 0u; i < 256u; i++) { dw_wr(GPU_DW_SRC, i, (uint8_t)i); }
    for (i = 0u; i < 256u; i++) { dw_wr(GPU_DW_DST, i, 0x5Au); }
    dw_palette_run(dev, clut_base, GPU_DW_SRC, GPU_DW_DST, 16u, 16u,
                   GPU_DW_FMT_INDEXED, 0u);
    pal0 = (uint32_t)dw_rd(GPU_DW_DST,   0u);
    pal1 = (uint32_t)dw_rd(GPU_DW_DST,   1u);
    pal2 = (uint32_t)dw_rd(GPU_DW_DST,  10u);
    pal3 = (uint32_t)dw_rd(GPU_DW_DST, 200u);
    sprintf(Message, "gpu dw pal %lu %lu %lu %lu (want 0 1 10 200)\r\n",
            (unsigned long)pal0, (unsigned long)pal1,
            (unsigned long)pal2, (unsigned long)pal3);
    UART_SendString(Message);

    /* ---------------------------------------------------------------
       Step 0c: the CLUT mode is the last variable standing, so isolate it.

       Mode 4 vs 5, QUAD vs ROW, LINEAR vs TILED_2 and AL_8 vs RGB565 are all
       measured (step 0a/0b) and none of them produces a weight.  So drop the
       kernel registers out of the picture entirely and vary ONLY the CLUT mode
       inside the format word, using this same function for both halves so that
       every other register write is identical:

           mode 1  indexed colour   -> the `pal` line above, 0 1 10 200
           mode 5  signed kernel    -> this line

       The palette here is (i+1) in all four bytes, which does double duty.
       As an INDEXED palette entry i reads back i+1 (so the control line is
       1 2 11).  As a KERNEL it is a legal 2x2 QUAD window whose four weights
       are all 1 - entry 0 is 0x01010101 - and with point sampling all four
       taps land on the same texel, so a working fetch gives 4*src:

           src 0 -> 0     src 1 -> 4     src 10 -> 40

       So:

           1 2 11 / 0 4 40   the CLUT mode is fine, mode 5 reads the CLUT even
                             with the filter bit off, and the fault is in what
                             dhd_set_texture_kernel() leaves behind
           1 2 11 / 0 1 10   mode 5 with the filter bit off just passes the
                             texel through - so the kernel needs filter=1, and
                             that combination is what the four corners already
                             measured as 0
           1 2 11 / 0 0 0    CLUT_KERNEL does not produce a texture value on
                             this silicon at all - the whole approach has to be
                             re-planned around the library's own path         */
    dw_palette_run(dev, clut_base, GPU_DW_SRC, GPU_DW_DST, 16u, 16u,
                   GPU_DW_FMT_INDEXED, 1u);
    pk_m1_0  = (uint32_t)dw_rd(GPU_DW_DST,  0u);
    pk_m1_1  = (uint32_t)dw_rd(GPU_DW_DST,  1u);
    pk_m1_10 = (uint32_t)dw_rd(GPU_DW_DST, 10u);

    dw_palette_run(dev, clut_base, GPU_DW_SRC, GPU_DW_DST, 16u, 16u,
                   GPU_DW_FMT_SIGNED, 1u);
    pk_m5_0  = (uint32_t)dw_rd(GPU_DW_DST,  0u);
    pk_m5_1  = (uint32_t)dw_rd(GPU_DW_DST,  1u);
    pk_m5_10 = (uint32_t)dw_rd(GPU_DW_DST, 10u);

    sprintf(Message, "gpu dw pk m1 %lu %lu %lu m5 %lu %lu %lu (1 2 11 / 0 4 40)\r\n",
            (unsigned long)pk_m1_0, (unsigned long)pk_m1_1,
            (unsigned long)pk_m1_10,
            (unsigned long)pk_m5_0, (unsigned long)pk_m5_1,
            (unsigned long)pk_m5_10);
    UART_SendString(Message);

    /* Nine taps of one on a constant 10, scale 1.0, bias 0:

           90  the kernel is applied          - nine taps fetched and summed
           10  kernel mode is ignored         - the texel copied straight through
            0  the kernel is applied but the weights read as zero

       Three outcomes, three different faults, one line.  And it does not
       depend on which tap lands in which CLUT byte: every tap is the same
       value, so any tap ordering that reads OUR kernel gives 90.  That is the
       point - it takes the packing question out of the measurement. */
    for (i = 0u; i < GPU_DW_PROBE_N; i++) { dw_wr(GPU_DW_SRC, i, 10u); }
    for (i = 0u; i < GPU_DW_PROBE_N; i++) { dw_wr(GPU_DW_DST, i, 0x5Au); }
    for (i = 0u; i < 9u; i++) { k9[i] = 1; }

    dw_pass(dev, GPU_DW_SRC, GPU_DW_DST, GPU_DW_PROBE_W, GPU_DW_PROBE_H,
            GPU_DW_PROBE_W, GPU_DW_PROBE_H, clut_base, k9, 3u, 3u,
            GPU_DW_FMT_SIGNED, E_DHD_PIXEL_ORG_L_8, E_DHD_ACCESS_LINEAR,
            1.0f, 0,
            0.0f, 0.0f,
            1.0f / (float)GPU_DW_PROBE_W, 1.0f / (float)GPU_DW_PROBE_H);
    k9_1 = (uint32_t)dw_rd(GPU_DW_DST,
                           (GPU_DW_PROBE_H / 2u) * GPU_DW_PROBE_W +
                           (GPU_DW_PROBE_W / 2u));

    /* Same kernel and source, scale 2.0 and bias 3, to separate "the kernel
       works and scale/bias rides on top of it" from "scale/bias works and the
       kernel does not" - both of which line 2 alone would report as a number.

           183  kernel * 2 + 3     everything works
            23  texel  * 2 + 3     scale/bias only, kernel ignored
            10  neither             the texel, straight through */

    dw_pass(dev, GPU_DW_SRC, GPU_DW_DST, GPU_DW_PROBE_W, GPU_DW_PROBE_H,
            GPU_DW_PROBE_W, GPU_DW_PROBE_H, clut_base, k9, 3u, 3u,
            GPU_DW_FMT_SIGNED, E_DHD_PIXEL_ORG_L_8, E_DHD_ACCESS_LINEAR,
            2.0f, 3,
            0.0f, 0.0f,
            1.0f / (float)GPU_DW_PROBE_W, 1.0f / (float)GPU_DW_PROBE_H);
    k9_2 = (uint32_t)dw_rd(GPU_DW_DST,
                           (GPU_DW_PROBE_H / 2u) * GPU_DW_PROBE_W +
                           (GPU_DW_PROBE_W / 2u));

    sprintf(Message, "gpu dw k9 %lu (90=ok 10=ignored 0=clut0) k9s %lu (183/23/10)\r\n",
            (unsigned long)k9_1, (unsigned long)k9_2);
    UART_SendString(Message);

    /* ---------------------------------------------------------------
       Test 1: is the texel sign-extended in signed-kernel mode?

       A constant source of 0x80 - -128 to an int8, +128 to a uint8 - with a
       centre tap of 1 and nothing else, scale 1.0, bias 128.  Sign-extended
       the sum is -128 and the output byte is 0; unsigned it is +128, so
       128 + 128 = 256, which clamps to 255.  Nothing between those two is a
       plausible reading of "it worked".

       The readback is the centre pixel, three in from every edge, so no tap
       can reach the border whatever the window turns out to be.            */
    for (i = 0u; i < GPU_DW_PROBE_N; i++) { dw_wr(GPU_DW_SRC, i, 0x80u); }
    for (i = 0u; i < GPU_DW_PROBE_N; i++) { dw_wr(GPU_DW_DST, i, 0x5Au); }

    for (i = 0u; i < 9u; i++) { k1[i] = 0; }
    k1[4] = 1;                                  /* centre tap */

    dw_pass(dev, GPU_DW_SRC, GPU_DW_DST, GPU_DW_PROBE_W, GPU_DW_PROBE_H,
            GPU_DW_PROBE_W, GPU_DW_PROBE_H, clut_base, k1, 3u, 3u,
            GPU_DW_FMT_SIGNED, E_DHD_PIXEL_ORG_L_8, E_DHD_ACCESS_LINEAR,
            1.0f, 128,
            0.0f, 0.0f,
            1.0f / (float)GPU_DW_PROBE_W, 1.0f / (float)GPU_DW_PROBE_H);

    val = dw_rd(GPU_DW_DST, (GPU_DW_PROBE_H / 2u) * GPU_DW_PROBE_W + (GPU_DW_PROBE_W / 2u));
    sprintf(Message, "gpu dw sign 80 -> %lu (0=signed 255=unsigned)\r\n",
            (unsigned long)val);
    UART_SendString(Message);

    /* ---------------------------------------------------------------
       Test 2: does the kernel window step by one texel, or by the destination
       pixel's UV delta?

       See dw_step_sweep().  At 1:1 the two candidate answers coincide, so that
       first line is a control - it should read 2, and if it does not, the
       problem is not the question being asked.  At two texels per pixel the
       answers separate: 2 means one texel, 4 means the gradient.

       Every previous log of this line reads `1:1 0 2:1 0`, twice over, and 0
       was never an answer - it was the bias, which is 0 here.  dw_step_sweep()
       had two defects and both are now fixed: a_scale was 1.0f, which truncates
       either candidate to 0, and the texel format was AL_8, which contributes
       nothing to a CLUT kernel at any scale.  See that function's comment.
       0 again after this change means the draw did not happen.

       The answer is the prerequisite for Step 3's stride-2 layers, and the two
       readings mean opposite things there: 4 means the taps scale with du/dx,
       so dux = 2/width puts them on the even input texels and stride 2 is free;
       2 means the taps step by TEXEL regardless of dux, so the window would sit
       on the odd texels and the input has to be decimated on the CPU first. */
    step1 = dw_step_sweep(dev, clut_base, GPU_DW_PROBE_W);
    step2 = dw_step_sweep(dev, clut_base, GPU_DW_PROBE_W / 2u);

    sprintf(Message, "gpu dw step 1:1 %lu 2:1 %lu (2=texel 4=gradient)\r\n",
            (unsigned long)step1, (unsigned long)step2);
    UART_SendString(Message);

    /* ---------------------------------------------------------------
       Test 3: the operator itself, against the CPU's own answer.

       A pseudo-random 32x32 field - noise rather than a ramp, because a ramp
       is exactly what a wrong m_u0 and a wrong m_dux can both reproduce - run
       through a signed 3x3 with all nine taps non-zero and both signs present,
       which is what the model's depthwise layers are made of.

       Run at both candidate m_u0 values, half a texel apart, because which one
       the texture unit means is not on record and the wrong one is a blend of
       two texels rather than a rounding error.  Whichever comes back with a
       clean interior is the convention; that is a measurement, and it replaces
       a guess that would otherwise sit inside every later result.  The third
       line re-runs it at the vendor's tiled address mode for the same reason.

       `int` (interior) and `all` (the whole plane) are reported separately
       because they answer different questions.  `all` above `int` is a
       statement about the border: the padded texels have to read as the model's
       input zero point (-128, byte 0x80) for a SAME-padded layer to come out
       right, and what dhd_set_texture_border_color() actually deposits there is
       not documented per-format.                                           */
    s_dw_rand = 0x12345678u;
    for (i = 0u; i < GPU_DW_T3_W * GPU_DW_T3_H; i++) { dw_wr(GPU_DW_SRC, i, dw_rand()); }

    bad = dw_ref_run(dev, clut_base, E_DHD_ACCESS_LINEAR,
                     0.5f / (float)GPU_DW_T3_W, 0.5f / (float)GPU_DW_T3_H,
                     &bad_all, &worst);
    sprintf(Message, "gpu dw lin u0=half int %lu all %lu max %lu\r\n",
            (unsigned long)bad, (unsigned long)bad_all, (unsigned long)worst);
    UART_SendString(Message);

    bad2 = dw_ref_run(dev, clut_base, E_DHD_ACCESS_LINEAR,
                      0.0f, 0.0f, &all2, &worst2);
    sprintf(Message, "gpu dw lin u0=0    int %lu all %lu max %lu\r\n",
            (unsigned long)bad2, (unsigned long)all2, (unsigned long)worst2);
    UART_SendString(Message);

    /* And the vendor's address mode, at the u0 that the two lines above are
       expected to have settled.  Cheap - one 32x32 draw - and it is the
       difference between "LINEAR was right" and "the vendor's number was
       load-bearing", which are very different things to know before a layer
       depends on it. */
    n_t2 = dw_ref_run(dev, clut_base, E_DHD_ACCESS_VIRTUAL_TILED_2,
                      0.5f / (float)GPU_DW_T3_W, 0.5f / (float)GPU_DW_T3_H,
                      &all3, &worst3);
    sprintf(Message, "gpu dw t2  u0=half int %lu all %lu max %lu\r\n",
            (unsigned long)n_t2, (unsigned long)all3, (unsigned long)worst3);
    UART_SendString(Message);

    if (out_bad != NULL) { *out_bad = bad; }

    /* ---------------------------------------------------------------
       And what a pass costs.  Every number on record for this GPU is from the
       nsgpu_* wrappers, which allocate and free a CLUT per call; the old 57 us
       per draw is that shape.  One 64x64 pass here, with the CLUT already
       allocated, is the figure that transfers to a layer of 24..96 draws.
       The CPU comparison above is deliberately NOT included in the timing: it
       is the reference, not the operator.                                   */
    s_dw_rand = 0x2468ACE0u;
    for (i = 0u; i < 64u * 64u; i++) { dw_wr(GPU_DW_SRC, i, dw_rand()); }

    t0 = DWT->CYCCNT;
    dw_pass(dev, GPU_DW_SRC, GPU_DW_DST, 64u, 64u, 64u, 64u, clut_base,
            s_dw_k3, 3u, 3u, GPU_DW_FMT_SIGNED, E_DHD_PIXEL_ORG_L_8,
            E_DHD_ACCESS_LINEAR, 1.0f, 128,
            0.5f / 64.0f, 0.5f / 64.0f, 1.0f / 64.0f, 1.0f / 64.0f);
    t1 = DWT->CYCCNT;

    sprintf(Message, "gpu dw 64x64 us %lu\r\n", (unsigned long)((t1 - t0) / 600u));
    UART_SendString(Message);

    /* The state this probe is about to hand back, then the hand-back itself.
       Read as a pair: dwZ is everything the drawings left, and dwF isolates the
       free - which is the very last thing that touches the GPU before the boot
       block's next probe runs. */
    gpu_env_state("dwZ");
    (void)dhd_free_clut(dev, 512u, clut_base);
    gpu_env_state("dwF");

    /* Every pass above drew with dhd_set_texture_scalebias() and put nothing
       back, and the last one - dw_pass(..., 1.0f, 128, ...) - is the shape that
       matters: the whole boot's blit runs with the bilinear filter on and never
       sets this word, so it inherits whatever the last draw left.  The `gpu
       agree` reading that follows this probe is the measurement of whether
       putting it back was the whole story. */
    gpu_sb_reset();
    gpu_env_state("dwS");
    return true;
}

/* ===================================================================
   The POINTWISE convolution's probe.  See gpu.h for what it asks and why the
   two questions are worth a flash round of their own.

   The two questions have one thing in common: both are about whether a 1xN
   ROW kernel is a first-class citizen here, and neither can be answered by
   reading the driver.  davehd_driver_txp_inl.h:581-598 has a dedicated branch
   for height==1 or width==1 that packs ceil(n/4) CLUT entries, and the
   per-unit registers for CLUT offset, kernel size and UV all exist - but
   nothing in this firmware has ever run a 1x16, and nothing has ever called
   dhd_active_texture_unit() with anything but 0.  So this measures both.

   THE TEXTURE.  64 texels wide, 8 rows, every row identical, and the texel's
   BLUE field carries `(x % period) + add` with red and green zero.  Blue is
   the field the CLUT kernel multiplies - five bits of it, `gpu dw k888 ff 31`
   - so with one tap of weight one the output NAMES the texel that tap read,
   and with N taps of one it is the sum of N consecutive codes.  Every
   prediction below is then a small exact integer with no rounding to argue
   about, and no tolerance to hide a wrong answer in.

   THE ALIGNMENT TEST reads the window's own offset out of the first output
   column: `o0 = code(0 + off) = off & 31`, so the printout says where tap 0
   sits relative to the sampled texel rather than assuming it is centred.  The
   step test then checks the whole row rather than three samples - a window
   that steps two texels, or follows the UV gradient, fails on most of 35
   consecutive differences instead of on one of them.

   THE TAP-COUNT TEST does NOT predict the output, and deliberately.  Summing N
   consecutive codes of a period-16 ramp depends on where the window starts,
   and where the window starts is what the test above is measuring - so a
   prediction here would need that test's answer, and the two would no longer
   be independent measurements.  What IS offset-free is the FIRST DIFFERENCE
   along the row: N taps advance the sum by N unless the window wraps around
   the period, where a whole 16-step cycle cancels and the difference is N-16.
   So the min and max of that difference over a row are exactly (N-16, N)
   whatever `off` is - 0/0 for sixteen taps, -8/8 for eight, -12/4 for four -
   and the value at one interior column adds the absolute check for the one
   width where the sum itself is offset-free: sixteen consecutive codes of a
   period-16 ramp are one whole cycle, so that reading is always 120.        */

/* The destination is pre-filled with this before every pass, so a pixel the
   draw did not reach names itself instead of looking like a zero the kernel
   produced.  Outside every predicted value (which are all <= 120 or exactly
   255), so it cannot be confused with a reading. */
#define GPU_CV_SENT         0xEEu

/* Texel x of `addr` gets blue field `((x % period) + add) & 31`, red and green
   zero, one row duplicated over all 8.  Two byte stores because the scratch is
   little-endian and uncached, the same way dw_step_sweep() stores its own. */
static void cv_fill(uint32_t addr, uint32_t period, uint32_t add)
{
    uint32_t x, y, i = 0u;

    for (y = 0u; y < GPU_CV_H; y++) {
        for (x = 0u; x < GPU_CV_W; x++) {
            uint32_t t = (((x % period) + add) & 31u);

            dw_wr(addr, i, (uint8_t)t);
            dw_wr(addr, i + 1u, 0u);
            i += 2u;
        }
    }
}

static int cv_px(uint32_t x, uint32_t y)
{
    return (int)dw_rd(GPU_CV_DST, (y * GPU_CV_W) + x);
}

/* min and max of the first difference along row `y` over columns x0..x1, with
   the first difference seeded from the first pair so the range is over real
   data rather than over a zero. */
static void cv_diff_range(uint32_t y, uint32_t x0, uint32_t x1,
                          int* mn, int* mx)
{
    uint32_t x;
    int      d;

    *mn = 0;
    *mx = 0;
    for (x = x0 + 1u; x <= x1; x++) {
        d = cv_px(x, y) - cv_px(x - 1u, y);
        if (x == x0 + 1u) { *mn = d; *mx = d; }
        if (d < *mn) { *mn = d; }
        if (d > *mx) { *mx = d; }
    }
}

/* One texture unit's worth of state.  `clut` is an ABSOLUTE CLUT entry index -
   clut_base + an offset - because that is what dhd_set_texture_kernel() and
   dhd_set_texture_clut_base() both take, the same way gpu_dw_conv() hands
   dw_pass() `s_dw_clut_base + (ucl * ci)`. */
typedef struct
{
    uint32_t      src;
    uint32_t      clut;
    const int8_t* k;
    uint32_t      kw;
    float         scale;
} cv_unit_t;

/* The colour side of one draw.

   THIS IS WHAT THE FIRST PROBE GOT WRONG, and it is worth writing out because
   the failure looked like silicon and was not.

   The colour unit's operands are not hard-wired to a texture unit.  Each one
   has an input SELECT register (5 bits) that chooses among E_DHD_INPUT_ZERO,
   E_DHD_INPUT_COVERAGE, E_DHD_INPUT_PREVIOUS, the sixteen constant colours and
   TEXTURE_0 .. TEXTURE_7 (davehd_driver.h:392-421).  dhd_set_color_operation()
   sets only the operands the requested formula needs and leaves the rest
   alone - and it ZEROES 1b for COPY, so an operand written before it can be
   silently destroyed by it.  The ambient state (whatever nsgpu_init() left)
   has op1a = TEXTURE_0, which is why every single-unit draw in this file has
   ever worked: nothing in this file ever selected it.

   The first probe configured two texture units with two different addresses
   and two different kernels and read 16 in all five configurations, and
   concluded that unit 1 does not contribute.  It does contribute.  With
   num_tex_units = 2 the unit is configured, sampled and its result latched -
   the colour pass simply never read it, because its op1a was still TEXTURE_0
   and COPY is 1a alone.  The vendor's own
   `dhd_util_color_add(src1, src2)` (davehd_util_state.c:151) is the idiom:

       dhd_set_color_operation( dev, op );
       dhd_set_color_op1a( dev, src1, E_DHD_CHANNEL_RGB, E_DHD_INPUT_DIRECT );
       dhd_set_color_op1b( dev, src2, E_DHD_CHANNEL_RGB, E_DHD_INPUT_DIRECT );

   with op = ADD, src1 = TEXTURE_0 and src2 = TEXTURE_1 - one pass, one draw,
   both units summed in the wide accumulator.  The op is written FIRST because
   of the destructive interaction above.

   For every op other than ADD this puts 1b back to ZERO, which reproduces the
   ambient default exactly, so a COPY draw here is byte for byte the same draw
   this file has always made. */
static void cv_color_setup(dhd_handle_t dev, dhd_color_op_t op)
{
    dhd_active_color_unit(dev, 0u);
    dhd_set_color_unit_enable(dev, 1u);
    dhd_set_color_operation(dev, op);
    dhd_set_color_op1a(dev, E_DHD_INPUT_TEXTURE_0, E_DHD_CHANNEL_RGB,
                       E_DHD_INPUT_DIRECT);
    dhd_set_color_op1b(dev, (op == E_DHD_OP_ADD) ? E_DHD_INPUT_TEXTURE_1
                                                 : E_DHD_INPUT_ZERO,
                       E_DHD_CHANNEL_RGB, E_DHD_INPUT_DIRECT);
}

/* The ambient GPU state this probe must put back exactly as it found it.

   gpu_dw_conv() draws with whatever colour operands happen to be in the
   registers: it calls dhd_set_color_operation(COPY) per draw, which does not
   reset op1a.  So a probe that leaves op1a pointing at TEXTURE_1 breaks every
   depthwise layer after it, and silently - the textures are still sampled,
   just the wrong unit's.  The counts are saved for the same reason: this probe
   wants one colour pass and its own texture unit count, and neither is
   necessarily what nsgpu_init() chose.  The channel select and the modifier
   are saved with the sources so the restore is exact rather than nominal.

   The COLOUR OPERATION is deliberately not in this struct, and that is a
   finding rather than an omission.  There is no getter for it: the driver
   exposes dhd_set_color_operation() and nothing to read it back (checked
   against davehd_driver_col_inl.h, where every other operand - 1a, 1b, 2a, 2b,
   3, the scale, the clamp - has a matching dhd_get_*, and the operation itself
   has none).  So a restore could only write back a value this code assumed,
   which is worse than not writing: it would look symmetric and be a guess.

   It is also not needed, and that is checkable rather than hopeful: no draw
   this firmware makes inherits the operation.  Every draw site sets it
   immediately before drawing - dw_pass() at 2848, dw_pass_bc() at 3133, the
   production conv path at 5545, and the blit through nsgpu_colorkey(), which
   configures the colour op itself (map:9217-9246).  And dhd_set_color_operation
   repairs the other half on the way in: it zeroes every operand its formula does
   not use, so the COPY those sites select resets 1b to ZERO even though it
   leaves 1a alone - which is exactly why 1a is the one that has to be saved
   here and the operation is the one that does not. */
typedef struct
{
    uint32_t             col_unit;
    uint32_t             col_pass;
    uint32_t             tex_unit;
    uint32_t             tex_count;
    dhd_col_input_t      a_src;
    dhd_col_input_t      b_src;
    dhd_channel_select_t a_ch;
    dhd_channel_select_t b_ch;
    dhd_col_modify_t     a_mod;
    dhd_col_modify_t     b_mod;

    /* The destination geometry.  cv_run() sets the framebuffer size and the bbox
       to its own 64x8 tile because that is the shape it draws, and the first
       version of this struct restored neither - on the reasoning that both
       belong to "the framebuffer" and cv_run() already put the format and the
       handle back.

       That omission was then blamed for the round in which the GPU front end
       stopped detecting anything, and that blame is WITHDRAWN: the library sets
       its own bbox (nsgpu_colorkey() -> nsgpu_draw_rotblit(), disassembly
       29643/32088), so a stale box cannot have been what the blit drew inside.
       See the comment in gpu_scale_frame() for the falsification and for the
       suspect that survives (g_framebuffers[framebuffer_acitve_index]).

       The restore is kept regardless, and it is still right: a probe that
       mutates global draw state and does not put it back is a trap for whoever
       adds the next draw.  Both getters exist (dhd_get_framebuffer_size,
       dhd_get_bbox), so this restores measured values rather than re-asserting
       assumed ones. */
    uint32_t             fb_w;
    uint32_t             fb_h;
    uint32_t             fb_pitch;
    uint32_t             bb_xmin;
    uint32_t             bb_ymin;
    uint32_t             bb_xmax;
    uint32_t             bb_ymax;
    uint32_t             bb_ystart;
} cv_env_t;

static void cv_env_save(dhd_handle_t dev, cv_env_t* e)
{
    e->col_unit  = dhd_get_active_color_unit(dev);
    e->col_pass  = dhd_get_color_unit_enable(dev);
    e->tex_unit  = dhd_get_active_texture_unit(dev);
    e->tex_count = dhd_get_texture_unit_enable(dev);
    e->a_src     = dhd_get_color_op1a(dev, &e->a_ch, &e->a_mod);
    e->b_src     = dhd_get_color_op1b(dev, &e->b_ch, &e->b_mod);

    dhd_get_framebuffer_size(dev, &e->fb_w, &e->fb_h, &e->fb_pitch);
    dhd_get_bbox(dev, &e->bb_xmin, &e->bb_ymin, &e->bb_xmax, &e->bb_ymax,
                 &e->bb_ystart);
}

static void cv_env_restore(dhd_handle_t dev, const cv_env_t* e)
{
    dhd_active_color_unit(dev, e->col_unit);
    dhd_set_color_op1a(dev, e->a_src, e->a_ch, e->a_mod);
    dhd_set_color_op1b(dev, e->b_src, e->b_ch, e->b_mod);
    dhd_set_color_unit_enable(dev, e->col_pass);
    dhd_set_texture_unit_enable(dev, e->tex_count);
    dhd_active_texture_unit(dev, e->tex_unit);
    dhd_set_framebuffer_size(dev, e->fb_w, e->fb_h, e->fb_pitch);
    dhd_set_bbox(dev, e->bb_xmin, e->bb_ymin, e->bb_xmax, e->bb_ymax,
                 e->bb_ystart);
}

/* One draw over `nunits` texture units, destination GPU_CV_DST.

   THE ORDER IS PER UNIT and it is the same constraint dw_pass_bc() documents:
   dhd_active_texture_unit() is what sets m_tex_unit_base, and address, size,
   format, kernel, clut_base, scalebias and the border all write through that
   base.  dhd_set_uv() is the exception - it takes the unit as an argument
   (dhd_set_uv(dev, unitid, &uv), davehd_draw.c:245) - so it is called per unit
   and does not depend on which one is active.  dhd_set_texture_unit_enable()
   is the one genuinely global call here; it sets m_tex_num_units.

   The framebuffer format is read, changed to L_8 and put back, and the
   framebuffer address is saved and restored, for the reasons dw_pass_bc()
   gives: the format is global and nsgpu_init() set it to RGB565, and leaving
   either changed would break every later nsgpu_* call.

   The unit count is left at whatever the last pass set.  That is safe rather
   than tidy: every other path in this file calls
   dhd_set_texture_unit_enable(dev, 1u) before it draws.

   The colour operands are NOT left as they were found - cv_color_setup() sets
   op1a and op1b for every draw - so gpu_conv_probe() saves the ambient operand
   state up front and cv_env_restore()s it before it returns.  Leaving op1a
   pointing at TEXTURE_1 would silently break every depthwise layer after it. */
#define CV_MODE_DIRECT 0u   /* one colour pass, op1b selects unit 1 */
#define CV_MODE_CHAIN  1u   /* two colour passes, pass 1 adds PREVIOUS */

/* WHERE cv_run() DRAWS, as a variable rather than the constant it was.

   The boot probes all want GPU_CV_DST, and GPU_CV_DST is GPU_DW_DST + 0 - which
   is also THE PRODUCTION CONV'S PLANE 0 for the first channel of the first
   chunk.  At boot that costs nothing, because no layer has run yet.  The dig
   block runs it MID-INFERENCE, at co == 0 of the first conv layer, and only
   after the channel's draws - so those 512 bytes of x-control output land ON the
   production hi plane, immediately before the census reads that plane and the
   nat/rev profiles compare it against the digit model.

   That is why the hi plane has read `nat 51 54 56 65 51` and `rev 58 58 57 48
   49` in every boot - a stable mismatch of about 49 that is too steady to be a
   dropped draw and is exactly the shape of 512 substituted bytes in a 1520-byte
   comparison.  The lo plane, one plane further on and outside the x-control's
   64x8 box, reads 9..10 every boot.  Same draws, same commit, same model: one
   plane matches and the other does not, and the difference between them is that
   the x control writes into one of them.

   The dig now points this at a plane ABOVE its own twelve, so the census and the
   profiles see the production draw's output and nothing else.  It is a variable
   and not a parameter because there are twelve call sites in the boot probes and
   none of them should change. */
static uint32_t s_cv_run_dst = GPU_CV_DST;

static void cv_run(dhd_handle_t dev, const cv_unit_t* u, uint32_t nunits,
                   dhd_color_op_t op, uint32_t mode)
{
    dhd_uv_data_t   uv;
    uint32_t        saved_fb;
    dhd_pixel_org_t saved_fmt;
    uint32_t        i;

    uv.m_u0  = 0.0f;
    uv.m_v0  = 0.0f;
    uv.m_dux = 1.0f / (float)GPU_CV_W;      /* one destination column per texel */
    uv.m_duy = 0.0f;
    uv.m_dvx = 0.0f;
    uv.m_dvy = 1.0f / (float)GPU_CV_H;

    for (i = 0u; i < GPU_CV_W * GPU_CV_H; i++) { dw_wr(s_cv_run_dst, i, GPU_CV_SENT); }

    dhd_set_texture_unit_enable(dev, nunits);
    for (i = 0u; i < nunits; i++) {
        dhd_set_uv(dev, i, &uv);
        dhd_active_texture_unit(dev, i);
        dhd_set_texture_address(dev, (dhd_gpu_ptr_t)u[i].src, 0u);
        dhd_set_texture_size(dev, GPU_CV_W, GPU_CV_H, GPU_CV_W);
        dhd_set_texture_format(dev, GPU_DW_FMT_SIGNED_565,
                               E_DHD_ACCESS_LINEAR, 0u);
        /* Also writes this unit's kernel width/height/sample_shape and enables
           its texture filter, and uploads the taps to `clut`. */
        dhd_set_texture_kernel(dev, u[i].kw, 1u, (const uint8_t*)u[i].k,
                               u[i].clut);
        dhd_set_texture_clut_base(dev, u[i].clut);
        dhd_set_texture_scalebias(dev, u[i].scale, 0);
        dhd_set_texture_wrap(dev, E_DHD_TEX_BORDER, E_DHD_TEX_BORDER);
        /* 0 and not 0x80808080: the border colour is documented ARGB8888, so
           0x80808080 is 0x80 in each channel, whose five-bit blue field is 16
           and not 0.  Every tap of this probe must read 0 outside the texture,
           or the alignment reading at the left edge is a sum of a real code
           and a phantom 16. */
        dhd_set_texture_border_color(dev, 0u);
    }
    if (mode == CV_MODE_CHAIN)
    {
        /* The fallback route, and the one the extra colour passes exist for:
           pass 0 copies unit 0 to the next pass, pass 1 adds unit 1 to it.
           E_DHD_INPUT_PREVIOUS is the previous colour unit's output
           (davehd_driver.h:395).  Cost is the same as the direct route - one
           draw either way - so this is a different register sequence, not a
           different design. */
        dhd_active_color_unit(dev, 0u);
        dhd_set_color_operation(dev, E_DHD_OP_COPY);
        dhd_set_color_op1a(dev, E_DHD_INPUT_TEXTURE_0, E_DHD_CHANNEL_RGB,
                           E_DHD_INPUT_DIRECT);
        dhd_active_color_unit(dev, 1u);
        dhd_set_color_operation(dev, E_DHD_OP_ADD);
        dhd_set_color_op1a(dev, E_DHD_INPUT_TEXTURE_1, E_DHD_CHANNEL_RGB,
                           E_DHD_INPUT_DIRECT);
        dhd_set_color_op1b(dev, E_DHD_INPUT_PREVIOUS, E_DHD_CHANNEL_RGB,
                           E_DHD_INPUT_DIRECT);
        dhd_set_color_unit_enable(dev, 2u);
    }
    else
    {
        cv_color_setup(dev, op);
    }

    /* The same ambient word dw_pass_bc() clears, and this is the message
       through which it reaches the pointwise family: this draw's plane is the
       production conv's PLANE 0 (see s_cv_run_dst above) and the pointwise
       layers run on every inference.  The full argument is on the call in
       dw_pass_bc() - E_DHD_UTIL_BLEND leaves read_decision() at
       USING_ONE_MINUS_ALPHA, the front end's gpu_scale_frame() is the only
       thing that sets it, and neither path here ever cleared it.  Cleared at
       the same point: after every colour operand is declared, before the
       framebuffer is switched to the destination. */
    dhd_util_blend(dev, E_DHD_UTIL_REPLACE);

    saved_fb  = nsgpu_get_framebuffer();
    saved_fmt = dhd_get_framebuffer_format(dev);

    dhd_set_framebuffer_format(dev, E_DHD_PIXEL_ORG_L_8);
    dhd_set_framebuffer_size(dev, GPU_CV_W, GPU_CV_H, GPU_CV_W);
    (void)dhd_set_framebuffer_address(dev, (dhd_gpu_ptr_t)s_cv_run_dst);
    dhd_set_bbox(dev, 0u, 0u, GPU_CV_W - 1u, GPU_CV_H - 1u, 0u);
    /* dhd_draw() commits, so this site gets the repair on the way in too. */
    gpu_keep_active_now();
    dhd_draw(dev, 0u, 0u, (dhd_drawattribute_flag_t)0u);
    dhd_wait(dev, E_DHD_WAIT_WRITEBACK);

    dhd_set_framebuffer_format(dev, saved_fmt);
    /* BOUNDED RESTORE.  This was the last raw nsgpu_set_framebuffer() left in
       the image, and it is the tail the whole drain fix exists because of: the
       library's restore ends in nsgpu_cmd_sync = dhd_flush + dhd_finish, and
       dhd_finish waits forever on a job that never retires.  gpu_fb_put() is
       the bounded equivalent the front end has used since Code=199270 (address
       + gpu_drain(), which also carries the keep repair) - see its definition.
       Every boot probe's restore is already the same substitution; this one was
       missed because it sits on the dig path rather than on the grab path. */
    (void)gpu_fb_put(dev, saved_fb);
}

/* ===================================================================
   blit_convolve_new() / blit_separable_convolve_new()

   The vendor's blit_convolve() and blit_separable_convolve() as this file
   would write them.  Two things force a rewrite rather than a call:

     1. THE LIBRARY VERSIONS CANNOT BE CALLED AT ALL.  Both keep their kernel
        byte buffer in a register they never assign (r6 in nsgpu_convolve(),
        r7 in nsgpu_separable_convolve()), which is why the wrappers at the top
        of this file exist to patch the register in and out.  Called unwrapped
        and measured: garbage coefficients, and on the operator_type 1 path a
        HARD FAULT.  See the comment above nsgpu_separable_convolve_k().

     2. THE THREE CAPABILITY GAPS, which are what an int8 model needs and
        neither library function has:

          (a) ONE scale and ONE bias per call, where all 52 CONV_2D in this
              model are quantised PER OUTPUT CHANNEL;
          (b) NO STRIDE - the vendor samples 1:1 into its destination box, so
              a stride-2 layer is not expressible;
          (c) NO 8-BIT OUTPUT - it draws in whatever format the active
              framebuffer happens to have.

   All three are already solved one level DOWN, in dw_pass_bc(): per-channel
   gain by the caller looping, stride through the UV gradient, 8-bit through
   dhd_set_framebuffer_format.  This pair is that solution lifted to the
   blit-level primitive the vendor's name promises.  Nothing here is a new
   driver sequence: the call order, the ambient save/restore and the bounded
   drain are dw_pass_bc()'s, all measured on this board.

   WHAT IS *NOT* CLOSED, and it is the reason the depthwise operator is still
   the only convolution offloaded: the colour unit requantises by TRUNCATING
   toward zero and its only egress is ONE BYTE.  This model's partial sums need
   11..15 bits (tools/chain_emulate.py --range), so no arrangement of these
   functions is bit-exact, and one wrong LSB moves the decoded plate
   (--sweep: one offloaded layer of 19 is enough).  Closing (a)/(b)/(c) makes an
   offload EXPRESSIBLE, not CORRECT.  Anything built on these is still judged by
   a byte-for-byte comparison against the CPU before it ships.
   =================================================================== */

/* The two framebuffer items cv_env_t does NOT cover.  cv_env_save() takes the
   colour operands, the unit counts, the framebuffer SIZE and the BBOX (it is
   what gpu_conv_probe() restores with); the ADDRESS and the FORMAT are these
   two, and dw_pass_bc()'s header is the measurement that says all four have to
   go back - its first version restored two of the four and every inference
   after it left the driver at the last layer's 64x64 tile. */
typedef struct
{
    uint32_t        fb;
    dhd_pixel_org_t fmt;
} blit_fb_t;

/* Point the framebuffer at `dst` (a dw x dh tile, pitch dw), draw, wait, drain.

   The four setters are dw_pass_bc()'s order verbatim (:3565-3568) - format
   before size, which is the opposite of the TEXTURE constraint three lines
   above it and is why the two orders are worth keeping distinct.  The wait is
   blit_convolve's own 0x1F8, spelled E_DHD_WAIT_WRITEBACK.  The drain is the
   bounded replacement for nsgpu_set_framebuffer()'s unbounded one - see
   gpu_drain().  `false` means it refused: the caller must return the refusal,
   not retry. */
static bool blit_tile(dhd_handle_t dev, uint32_t dst, uint32_t dw, uint32_t dh,
                      dhd_pixel_org_t pixfmt)
{
    /* The timed and untimed bodies are the same five calls in the same order -
       they are written twice rather than sharing a helper so that the untimed
       path keeps its exact old instruction stream.  See g_blit_t in gpu.h. */
    if (g_blit_t_on == 0u)
    {
        dhd_set_framebuffer_format(dev, pixfmt);
        dhd_set_framebuffer_size(dev, dw, dh, dw);
        (void)dhd_set_framebuffer_address(dev, (dhd_gpu_ptr_t)dst);
        dhd_set_bbox(dev, 0u, 0u, dw - 1u, dh - 1u, 0u);
        dhd_draw(dev, 0u, 0u, (dhd_drawattribute_flag_t)0u);
        dhd_wait(dev, E_DHD_WAIT_WRITEBACK);
        return gpu_drain(dev);
    }
    else
    {
        uint32_t t0;
        uint32_t t1;
        uint32_t t2;
        uint32_t t3;
        bool     ok;

        t0 = DWT->CYCCNT;
        dhd_set_framebuffer_format(dev, pixfmt);
        dhd_set_framebuffer_size(dev, dw, dh, dw);
        (void)dhd_set_framebuffer_address(dev, (dhd_gpu_ptr_t)dst);
        dhd_set_bbox(dev, 0u, 0u, dw - 1u, dh - 1u, 0u);
        t1 = DWT->CYCCNT;
        dhd_draw(dev, 0u, 0u, (dhd_drawattribute_flag_t)0u);
        t2 = DWT->CYCCNT;
        dhd_wait(dev, E_DHD_WAIT_WRITEBACK);
        t3 = DWT->CYCCNT;
        ok = gpu_drain(dev);

        g_blit_t[0] += (t1 - t0) / 600u;
        g_blit_t[1] += (t2 - t1) / 600u;
        g_blit_t[2] += (t3 - t2) / 600u;
        g_blit_t[3] += ((uint32_t)DWT->CYCCNT - t3) / 600u;
        if (g_blit_t[4] < 0xFFFFFFFFu)
        {
            g_blit_t[4]++;
        }
        return ok;
    }
}

/* m_dux and m_dvy only, the other two cleared - the same six writes dw_pass_bc()
   makes at :3522.  dux is the step per DESTINATION column measured in TEXTURE
   WIDTHS - u in [0,1] spans the whole texture - so one texel is 1/tw and a
   sampling stride of s is s/tw.  dw_pass_bc()'s two stride classes are the same
   statement (:6283 1/uw, :6304 2/uw) and they are what settles this: the first
   version here divided by the DESTINATION width instead, and the board refuted
   it - `gpu blit stride got 33 9 (d 24)` where the difference had to be 16,
   because a dw-based gradient makes the step sw*tw/dw texels (4 at the probe's
   dw=32, tw=64, sw=2) instead of sw (2).  Consequence for a caller: the tile
   covers dw*sw texels and nothing clips that to the texture, so dw*sw <= tw. */
static void blit_uv(dhd_uv_data_t* uv, float u0, float v0, float dux, float dvy)
{
    uv->m_u0  = u0;   uv->m_v0  = v0;
    uv->m_dux = dux;  uv->m_duy = 0.0f;
    uv->m_dvx = 0.0f; uv->m_dvy = dvy;
}

/* The transposed twin: u advances with the destination's ROW and v with its
   COLUMN, so the destination pixel (dx,dy) reads the texel at
   ((u0 + duy*dy) * tw, (v0 + dvx*dx) * th) - i.e. the plane it samples from is
   read TRANSPOSED.  That is the one thing that makes a rank-1 VERTICAL pass
   expressible with the ROW kernel shape: the ROW shape's taps always walk u,
   and u indexes the texture's width, so they can only ever walk the source's x
   - unless the intermediate was STORED transposed, in which case the taps'
   walk along u is a walk down the source's y.  Pure addressing, no new kernel
   shape, and it is why dw == dh is required: one tap is one texel in each
   direction here, so a non-square tile would ask for texels past the edge. */
static void blit_uv_t(dhd_uv_data_t* uv, float u0, float v0, float duy,
                      float dvx)
{
    uv->m_u0  = u0;   uv->m_v0  = v0;
    uv->m_dux = 0.0f; uv->m_duy = duy;
    uv->m_dvx = dvx;  uv->m_dvy = 0.0f;
}

/* The common tail: put all four ambient framebuffer items back and drain once.
   The drain is also what commits the driver's framebuffer-address write, and a
   single one here covers the whole channel loop below - dw_pass_bc() needs one
   per call because it IS one call per channel. */
static bool blit_restore(dhd_handle_t dev, const cv_env_t* env,
                         const blit_fb_t* fb)
{
    cv_env_restore(dev, env);
    dhd_set_framebuffer_format(dev, fb->fmt);
    (void)dhd_set_framebuffer_address(dev, (dhd_gpu_ptr_t)fb->fb);
    return gpu_drain(dev);
}

/**
 * @brief  blit_convolve() with the three gaps closed: per-output-channel
 *         scale and bias, an explicit stride, and an 8-bit destination.
 *
 * Writes `co_count` output channels of one w x h tap kernel from ONE texture,
 * each channel to its own plane `dst + i*dst_stride`.  The kernel block is
 * `co_count * kw * kh` bytes, channel-major, and the CLUT offset is the only
 * CLUT decision this function makes.
 *
 * @param  dw, dh        destination tile, in texels.  With sw == sh == 1 the
 *                       caller passes dw == tw for a full 1:1 pass, and with a
 *                       stride it passes dw == tw/sw - the tile covers dw*sw
 *                       texels and the sampler does not clip that to the
 *                       texture, so dw must not exceed tw/sw.
 * @param  sh, sw        sampling stride, >= 1.  Expressed as the UV gradient
 *                       `sw/tw`, `sh/th`: the step is in texture widths, not in
 *                       destination widths (see blit_uv).
 * @param  u0, v0        window start; the stride moves the STEP, not the start.
 * @param  pixfmt        destination format - E_DHD_PIXEL_ORG_L_8 for the 8-bit
 *                       planes this file's operators use.
 * @return false if the arguments are out of range or the drain refused.  The
 *         ambient state is restored either way.
 */
static bool blit_convolve_new(dhd_handle_t dev,
                       uint32_t src, uint32_t dst,
                       uint32_t tw, uint32_t th,
                       uint32_t dw, uint32_t dh,
                       uint32_t dst_stride,
                       uint32_t clut_base,
                       const int8_t* k, uint32_t kw, uint32_t kh,
                       const float* scale, const int32_t* bias,
                       uint32_t co_first, uint32_t co_count,
                       float u0, float v0,
                       uint32_t sh, uint32_t sw,
                       dhd_texel_org_t texfmt, dhd_pixel_org_t pixfmt,
                       dhd_address_mode_t amode, uint32_t a_border)
{
    dhd_uv_data_t uv;
    cv_env_t      env;
    blit_fb_t     fb;
    uint32_t      co;
    bool          ok = true;

    /* The kernel field is 4 bits wide and stores width-1, so 16 is the hard
       ceiling (davehd_driver_txp_inl.h:576).  Zero stride would divide by zero
       in the gradient below. */
    if ((co_count == 0u) || (co_first >= co_count) ||
        (kw == 0u) || (kw > 16u) || (kh == 0u) || (kh > 16u) ||
        (sw == 0u) || (sh == 0u) || (dw == 0u) || (dh == 0u))
    {
        return false;
    }

    cv_env_save(dev, &env);
    fb.fb  = nsgpu_get_framebuffer();
    fb.fmt = dhd_get_framebuffer_format(dev);

    blit_uv(&uv, u0, v0, (float)sw / (float)tw, (float)sh / (float)th);

    /* ONE unit, and dhd_active_texture_unit() must precede everything that
       writes a per-unit register: it is what sets m_tex_unit_base and every
       setter below computes "that base + an offset".  dhd_set_texture_size()
       must precede dhd_set_texture_format() here (the opposite of blit_tile()'s
       framebuffer order): the format call recomputes m_txc_burst_pitch_0 from
       the pitch size sets, and this pass uses the tiled address mode. */
    dhd_set_texture_unit_enable(dev, 1u);
    dhd_set_uv(dev, 0u, &uv);
    dhd_active_texture_unit(dev, 0u);
    dhd_set_texture_address(dev, (dhd_gpu_ptr_t)src, 0u);
    dhd_set_texture_size(dev, tw, th, tw);
    dhd_set_texture_format(dev, texfmt, amode, 0u);
    dhd_set_texture_wrap(dev, E_DHD_TEX_BORDER, E_DHD_TEX_BORDER);
    dhd_set_texture_border_color(dev, a_border);

    for (co = 0u; co < co_count; co++)
    {
        uint32_t o = co - co_first;

        /* Per channel: kernel, gain, bias, destination - the four things the
           vendor's single scale/bias and fixed destination could not vary.

           The driver does its own packing and upload
           (dhd_set_texture_kernel() -> dhd_set_clut()), so this function has no
           CLUT arithmetic and no 512-word budget to manage: ONE slot serves
           every channel because each channel's upload lands there before that
           channel's draw.  That is also why the call is inside the loop - it is
           the only writer of m_txa_filter_kernel_width/height/sample_shape, so
           a shared offset would need one slot per channel instead. */
        dhd_set_texture_kernel(dev, kw, kh,
                               (const uint8_t*)(k + (o * kw * kh)), clut_base);
        dhd_set_texture_clut_base(dev, clut_base);
        dhd_set_texture_scalebias(dev, scale[o], bias[o]);
        dhd_set_color_operation(dev, E_DHD_OP_COPY);

        if (!blit_tile(dev, dst + (o * dst_stride), dw, dh, pixfmt))
        {
            ok = false;
            break;
        }
    }

    return blit_restore(dev, &env, &fb) && ok;
}

/* ===========================================================================
   THE EXACT-OPERATOR ROUTE: TRANSPOSE, one plain 2D transposed read.

   WHY THIS IS NOT A blit_convolve_new() CALL, AND IT IS THE WHOLE POINT.

   Every other draw in this file multiplies by CLUT KERNEL weights, and a
   kernel's multiplicand is FIVE BITS.  That is measured, not inferred: one centre
   tap of one over an RGB_888 blue byte reads 0x40 as 8, 0x80 as 16 and 0xFF as 31
   (gpu_dw_probe's `gpu dw k888` line, and the comment on GPU_DW_FMT_565A8 above
   says it again).  RGB_888 is 8 bits per channel ON THE WAY IN and the kernel
   still takes the top five, so no texel format and no scale can carry an int8
   through a kernel draw.  That is the wall the depthwise and pointwise stages
   both live behind, and it is why they are lossy by construction.

   A TRANSPOSE HAS NO ARITHMETIC.  Its output is a permutation of its input bytes.
   A requantising permutation is not a slightly-wrong version of this operator, it
   is a different and broken one - every element loses its low three bits.  So the
   kernel route is not available here at all, and the operator needs a draw that
   does not multiply.

   THE WAY OUT IS TO STOP MULTIPLYING.  E_DHD_CLUT_INDEXED_COLOR turns the CLUT
   into an ordinary 256-entry PALETTE: the texel is an 8-bit INDEX and the entry it
   selects is what reaches the framebuffer.  Nothing takes the top five bits of
   anything.  With an identity palette and an L_8 destination the draw is `dst =
   src` over the whole 0..255 range, and that is the measurement the depthwise
   stage is built on: `gpu dw pal 0 1 10 200`, where 200 is the number that
   matters, because no 5-bit field can hold it.  gpu_transpose_probe() re-measures
   it at every boot as `gpu tr`.

   THE TRANSPOSE ITSELF IS PURE ADDRESSING, and this file already has the
   primitive.  blit_uv_t() sets m_duy and m_dvx instead of m_dux and m_dvy, so
   destination (dx,dy) samples the texel at (dy,dx).  It was written for the
   separable filter's vertical pass and is board-verified there; here it is asked
   for the 1:1 transposed read, which is the same statement with a stride of one.

   WHAT IS ASSUMED AND WHAT IS MEASURED.  The 5-bit kernel rule, the indexed
   lookup, the L_8 destination and the AL_8 texel read are all measured.  What has
   never been on this board is a transposed 1:1 read at these sizes, and that is
   exactly what the probe measures before the hook is allowed to depend on it.

   COST, STATED UP FRONT.  The pack and the unpack cross the GPU slot one BYTE at
   a time through uncached accesses, and this part's SDRAM is slow in a way that is
   a silicon defect and not a software choice (see the standing note).  A 400x48
   instance moves 19200 bytes each way and the CPU kernel does the same job in
   ~4.5 us per thousand bytes.  Expect this to be SLOWER than the operator it
   replaces.  That is a reading to take and report, not a reason to skip it: the
   stage is counted in operators running on the hardware.

   WHAT IT LEAVES BEHIND.  cv_env_save()/blit_restore() put the framebuffer
   format, size, pitch, bbox, address, the colour unit and the texture unit count
   back, exactly as blit_convolve_new() does.  The per-unit texture state (size,
   format, wrap, filter, CLUT base, scale/bias) is NOT restored - blit_convolve_new()
   has the same gap, and every consumer in this file sets its own before it draws
   - but one of those writes is worth naming: dhd_set_texture_filter(0,0) is what
   turns the KERNEL off, and leaving it off is why the next path that wants a
   kernel must set it itself.  nsgpu_convolve, filter_frame_with() and
   blit_convolve_new() all do. */

/* The model's largest [0,2,3,1] TRANSPOSE is 38400 elements, and it is also the
   ceiling this operator accepts.  The three shapes are (1,48,20,20) = 19200,
   (1,24,40,40) = 38400 and (1,96,10,10) = 9600; an earlier version of this
   comment called all three "exactly 19200" and set the bound there, which would
   have REFUSED the six 38400-value instances had the caller's own bound not
   refused them first.  The arithmetic is 24*40*40, and tools/_transpose_shapes.py
   is where it is printed.  The two GPU slot planes are 192 KB each, so the two
   hard bounds are the slot size and the driver's 12-bit bbox field, and neither
   is near. */
#define GPU_TR_MAX_PX 38400u
#define GPU_TR_MAX_W  4096u
#define GPU_TR_MAX_H  4096u
#define GPU_TR_PAL_WORDS 256u

static uint32_t s_tr_no_clut = 0u;  /* the CLUT refusal is reported once, not per call */
static uint32_t s_tr_pal[GPU_TR_PAL_WORDS];
static int8_t   s_tr_in[GPU_TR_MAX_PX];
static int8_t   s_tr_out[GPU_TR_MAX_PX];

/* The 256-word slice of the 512-entry CLUT pool, and the identity palette in it.

   THE CLUT IS BORROWED FOR THE CALL, NOT HELD.  That is a correction, not a
   style choice, and it is worth writing down because the obvious version is
   wrong in a way that only shows up when a key is pressed.

   The pool is `m_num_clut_entries` = 512 entries and THE DEPTHWISE AND POINTWISE
   PATHS TAKE ALL OF IT: gpu_dw_conv() and gpu_conv() each call
   dhd_alloc_clut(dev, GPU_DW_CLUT_WORDS = 512, ...) the first time they run
   and never free it.  dhd_alloc_clut() needs a CONSECUTIVE free run, and the
   allocator is a 32-block bitmap with 16 entries per block, so a held 256 leaves
   no 512-run anywhere.  A transpose that allocated once and kept the slice would
   therefore not fail - it would SUCCEED, and then quietly kill the shipped
   depthwise offload for the rest of the boot.  Borrowing and returning costs one
   dhd_alloc_clut + one dhd_set_clut + one dhd_free_clut per transposed layer per
   inference, and it is the difference between "the two coexist" and "the last
   key wins".

   The ordering that comes out of it is the one that is actually needed:
   dhd_alloc_clut() fails only in the window AFTER a convolution offload has run,
   and the arm is refused there, once, with a line that says so.  Before the first
   `g` or `c` it always succeeds - and those two default to 0.

   The palette is re-uploaded on every call rather than once.  It is a constant,
   but the slice is not ours between calls and a convolution draw loads its kernel
   weights over whatever CLUT offset it owns; the only offset it can own is one we
   have just given back.  So the upload is what makes the identity a guarantee.
   dhd_set_clut() opens with DHD_STCOP_WAIT, so it is a pipeline drain - that is
   the price of sharing, and it is paid once per transposed layer. */

/* THE COST SPLIT, and why it is worth five stamps inside the operator.

   The 108 ms this operator costs on a real activation is the whole reason the
   family is not being extended, so "108 ms" alone is not enough to answer the
   next question.  WHICH PART of it is this operator's own and which part any
   other family would also pay is what decides what PAD, CONCATENATION or
   MAX_POOL_2D would cost, because the marshalling and the per-call setup are
   shared across all of them and only this one has ever been measured.

   Filled only while g_tr_t_on is set, which only gpu_transpose_probe() does, so
   the hook's own calls pay one `!= 0` and nothing else - the same gate idiom as
   g_blit_t_on, and for the same reason.

   Five raw DWT->CYCCNT stamps; the reader subtracts and divides by 600.  The
   four intervals are setup (CLUT alloc + palette upload + set_clut), pack, draw
   (blit_tile: texture/colour setup + the draw + writeback wait + bounded drain)
   and unpack, and the reader takes the residual to the end of the call, which is
   free_clut + blit_restore.  DWT->CYCCNT wraps at 7.158 s and one call is ~0.1 s,
   so one call's stamps are safe to subtract directly and nothing is carried
   between calls.

   WHAT THE SPLIT BOUGHT, 2026-10-09, and it closes the family (the route's
   header is in gpu.h).  The four constants above - setup 104 us per call, pack
   0.0999 us per INPUT byte, draw 2.184 us per OUTPUT byte, unpack 0.2971 us per
   output byte - reproduce the 38400-element case to 3 us (99215 against 99218
   measured), and applied to the board's own `e` census (603 ms of a 1.696 s
   inference, counts board-confirmed) they project EVERY family onto the
   hardware as a LOSS:

       TRANSPOSE  52 ops  242.3 ms CPU ->  4317 ms GPU   17.8x
       PAD         8      102.5      ->   3933          38.4x
       RESHAPE    26       85.5      ->   2877          33.7x
       CONCAT     19       64.3      ->   2193          34.1x
       GATHER     26       37.3      ->   1495          40.1x
       MAX_POOL    1       71.2      ->    444           6.2x
       TOTAL     132      603.0      ->  15259          25.3x

   The LEVEL is not firm - the draw constant is calibrated on ONE destination,
   and a family that drew at the front end's rate would shrink the whole table by
   ~4x - but the ORDERING is robust and no entry crosses 1x.  And MAX_POOL_2D,
   the best-shaped family, is the one this hardware cannot express at all:
   dhd_color_op_t is COPY / MUL / ADD / MULADD / LERP / PROD / FULL, all of them
   linear, and a kernel is a weighted sum - there is no MAX in this GPU.

   So the two objections separate cleanly.  Accuracy: ABSENT, by construction and
   on the board (`tr n 6 bad 0` on real activations) - this is the only offload
   family in the project that cannot change the output.  Cost: this table, and it
   is why the family is closed.  Offloading it would take a 1.70 s inference to
   ~15 s.  The hook stays, default off, as the demonstrated exact offload; do not
   extend the family without a new cost model. */
static uint32_t g_tr_t[5];
static uint32_t g_tr_t_on = 0u;

/**
 * @brief  The exact transpose: `out[c*th + r] == in[r*tw + c]`, every byte.
 *
 * The source is a `tw` x `th` 8-bit plane read row-major and the destination is
 * the transposed `th` x `tw` plane, also row-major - which is exactly what a
 * 4-D tensor's `[0,2,3,1]` TRANSPOSE is once its leading 1 is folded away: the
 * input (1,a1,a2,a3) is the a1 x (a2*a3) matrix with tw = a2*a3 and th = a1, and
 * the output (1,a2,a3,a1) is that matrix transposed, stored row-major.
 *
 * The draw reads the source TRANSPOSED rather than writing the destination
 * transposed: the destination tile is `th` wide and `tw` tall, which is the
 * output plane in its natural row-major order, so the unpack is a straight
 * linear copy and needs no index arithmetic anywhere in this function.  All of
 * the transposing is in the UV gradient (blit_uv_t), which is where the hardware
 * does it for free.
 *
 * @return false on a range violation or if the GPU refused.  The caller falls
 *         back to the CPU kernel; nothing is retried here.
 */
bool gpu_transpose_s8(const int8_t* in, int8_t* out, uint32_t tw, uint32_t th)
{
    dhd_handle_t  dev = (dhd_handle_t)a_handle;
    dhd_uv_data_t uv;
    cv_env_t      env;
    blit_fb_t     fb;
    uint32_t      clut_base = 0u;
    uint32_t      i, n;

    if ((!s_ready) || (in == NULL) || (out == NULL) ||
        (tw == 0u) || (th == 0u) ||
        (tw > GPU_TR_MAX_W) || (th > GPU_TR_MAX_H))
    {
        return false;
    }
    n = tw * th;
    if (n > GPU_TR_MAX_PX) { return false; }

    if (g_tr_t_on != 0u) { g_tr_t[0] = DWT->CYCCNT; }

    /* Borrow the palette slice - see the CLUT comment above the statics.  A
       refusal here is reported once and is not an error to retry: the caller
       falls back to the CPU kernel, which is the product's answer for this
       operator anyway. */
    if (dhd_alloc_clut(dev, GPU_TR_PAL_WORDS, &clut_base) != 0u)
    {
        if (s_tr_no_clut == 0u)
        {
            s_tr_no_clut = 1u;
            UART_SendString("gpu tr no clut "
                            "(the dw/cv offload holds all 512 words)\r\n");
        }
        return false;
    }
    for (i = 0u; i < GPU_TR_PAL_WORDS; i++) { s_tr_pal[i] = i * 0x01010101u; }
    (void)dhd_set_clut(dev, s_tr_pal, clut_base, GPU_TR_PAL_WORDS);

    if (g_tr_t_on != 0u) { g_tr_t[1] = DWT->CYCCNT; }

    /* 1:1 transposed read.  duy is the step per destination ROW measured in
       texture WIDTHS and dvx the step per destination COLUMN in texture
       HEIGHTS - the two fields blit_uv() leaves at zero - so destination (dx,dy)
       lands on texel (dy, dx).  Destination x therefore indexes the source's
       ROWS and destination y its COLUMNS, which is the statement
       `dst[y*th + x] = src[x*tw + y]` and is the transpose.

       THE HALF-TEXEL START IS LOAD-BEARING, and the board is what says so.  The
       first version started at 0 and the probe came back

           gpu tr A 16x16 worst 0 bad 0 of 256        (at 0 got 0 want 0)
           gpu tr B 400x48 worst 249 bad 19199 of 19200 (at 48 got 0 want 31)

       - 19199 of 19200 wrong, and all four printed samples exactly ONE TEXEL LOW
       IN BOTH AXES: destination (dy,dx) had read texel (dy-1, dx-1), clamped at
       0.  The reading is not a rounding error, it is the whole plane shifted.

       WHY 0 IS WRONG.  A start of 0 puts the sample point exactly ON the texel
       boundary, and an exact boundary is the one place a point sample is not
       defined: the unit's internal u is a fixed-point number, so u = dy/tw is
       held only as exactly as 1/tw can be held.  At tw = 16 (Case A) 1/16 is a
       dyadic fraction and is exact, so the boundary comes out on the integer and
       floors to the right texel.  At tw = 400 and th = 48 it is not, the stored
       u lands a hair BELOW the boundary, and floor() drops it a whole texel - for
       EVERY pixel, which is why the count is total rather than statistical.

       So the sample is moved to the middle of the cell, where there is half a
       texel of margin either way.  This is not a fudge: it is what the vendor's
       own 1:1 blit does - davehd_util_convert.c:3151-3152 sets
       `uv.m_u0 = uv.m_dux / 2.0f` and `uv.m_v0 = uv.m_dvy / 2.0f` - and it is
       why that code, read at 0, would have had the same defect.

       The separable filter's ROWCOL_T pass also starts at 0 and is NOT changed
       here.  Its sample feeds a KERNEL, and a kernel is a window centred on the
       sample, so a half-texel start moves the result by half a pixel instead of
       selecting the wrong texel - which is exactly why the stride-1 depthwise
       class reads `worst 4 bad 0` at u0 = 0 (gpu.c, the stride-1 UV branch).
       With no kernel there is nothing to absorb it, and one texel is the whole
       answer. */
    blit_uv_t(&uv, 0.5f / (float)tw, 0.5f / (float)th,
                    1.0f / (float)tw,  1.0f / (float)th);

    cv_env_save(dev, &env);
    fb.fb  = nsgpu_get_framebuffer();
    fb.fmt = dhd_get_framebuffer_format(dev);

    /* ONE unit, and dhd_active_texture_unit() must precede everything that writes
       a per-unit register - the same order blit_convolve_new() documents.
       dhd_set_texture_size() must precede dhd_set_texture_format() because the
       format call recomputes the burst pitch. */
    dhd_set_texture_unit_enable(dev, 1u);
    dhd_set_uv(dev, 0u, &uv);
    dhd_active_texture_unit(dev, 0u);
    dhd_set_texture_address(dev, (dhd_gpu_ptr_t)GPU_DW_SRC, 0u);
    dhd_set_texture_size(dev, tw, th, tw);
    dhd_set_texture_format(dev, GPU_DW_FMT_INDEXED, E_DHD_ACCESS_LINEAR, 0u);
    dhd_set_texture_clut_base(dev, clut_base);
    /* Point sampling and no kernel.  A filter here would BLEND neighbouring
       texels, which is the one thing an exact permutation must not do; (0,0) is
       also the call that clears any kernel state a previous pass left behind
       (dw_palette_run() makes the same call for the same two reasons). */
    dhd_set_texture_filter(dev, 0u, 0u);
    /* The indexed lookup is not multiplied by the kernel gain, but the colour
       path still has one, so it is set to unity rather than left to whichever
       convolution ran last. */
    dhd_set_texture_scalebias(dev, 1.0f, 0);
    dhd_set_texture_wrap(dev, E_DHD_TEX_CLAMP, E_DHD_TEX_CLAMP);
    /* The operation FIRST: dhd_set_color_operation() zeroes the operands the
       formula does not use, so COPY destroys 1b - an operand written before it
       can be silently thrown away.  cv_color_setup() is the vendor's own idiom
       with that order already right. */
    cv_color_setup(dev, E_DHD_OP_COPY);

    /* Pack: one 8-bit index per texel, so the source plane IS the activation
       plane.  No bit packing and a stride of one - this is the one pack in the
       file that is a straight copy. */
    for (i = 0u; i < n; i++) { dw_wr(GPU_DW_SRC, i, (uint8_t)in[i]); }

    if (g_tr_t_on != 0u) { g_tr_t[2] = DWT->CYCCNT; }

    /* The destination tile is `th` wide and `tw` tall, and blit_tile() sets the
       framebuffer format, size, pitch, address and bbox, draws, waits for the
       writeback and takes the bounded drain.  `false` is a refusal. */
    if (!blit_tile(dev, GPU_DW_DST, th, tw, E_DHD_PIXEL_ORG_L_8))
    {
        /* blit_tile() is the submit point and it has already waited for the
           writeback, so the slice is idle and can go back before the restore. */
        (void)dhd_free_clut(dev, GPU_TR_PAL_WORDS, clut_base);
        (void)blit_restore(dev, &env, &fb);
        return false;
    }

    if (g_tr_t_on != 0u) { g_tr_t[3] = DWT->CYCCNT; }

    /* Unpack: the tile is the output plane in row-major order already, so this
       is a linear read with no index arithmetic at all - `dst[y*th + x]` IS
       `out[y*th + x]`. */
    for (i = 0u; i < n; i++) { out[i] = (int8_t)dw_rd(GPU_DW_DST, i); }

    if (g_tr_t_on != 0u) { g_tr_t[4] = DWT->CYCCNT; }

    (void)dhd_free_clut(dev, GPU_TR_PAL_WORDS, clut_base);
    return blit_restore(dev, &env, &fb);
}

/* The board's own answer, taken at boot and before anything depends on it.

   FOUR CASES, AND WHICH ONE CARRIES THE PROOF IS NOT WHAT IT FIRST LOOKED
   LIKE.  The 16x16 case holds every palette index 0..255 exactly once, so `bad 0`
   there is the palette - the part of this recipe that is a silicon property -
   verified over its FULL range rather than over a sample of it.

   It is NOT the addressing proof, and the first board run is why that had to be
   written down.  16 is a power of two, so this case's 1/16 gradient is exactly
   representable in the texture unit, and it read `bad 0` while the 400x48 case
   read 19199 of 19200 wrong.  A dyadic size cannot see a START-POINT error at
   all - the sample lands on the boundary and the boundary resolves correctly -
   so the cases whose gradient is NOT a dyadic fraction are the ones that carry
   the addressing proof.  The comment on blit_uv_t() in gpu_transpose_s8() has
   the mechanism and the reading.

   So there are four, and the two variables are separated rather than crossed:
   16x16 is square and dyadic, 400x48 is rectangular and NOT dyadic, and 40x40 is
   square like the first and NOT dyadic like the second.  40x40 is the same size
   class as the depthwise layers, which is what makes it worth a case of its own:
   if the half-texel start is the whole fix it reads `bad 0` here too, and if it
   is not, the fault needs the SHAPE and not the arithmetic.

   The fourth is not a way to separate the two variables - it is the shape the
   hook actually meets.  1600x24 is op17, the first [0,2,3,1] instance in model
   order, and it is a destination tile 24 wide and 1600 tall: 67:1, where 400x48
   is 8:1 the other way.  It is also the largest instance in the model, i.e. the
   one that did not fit the bound this probe used to share.  Every other case
   here could pass and the first real activation still be the first time a tile
   that tall is ever drawn, so this one is the model's answer rather than the
   arithmetic's.

   The 400x48 case is the model's own shape at its own size, with a pattern that
   varies in BOTH indices (`r*7 + c*31`), so a UV transposition error cannot
   cancel: a `dux`/`dvy` gradient instead of `duy`/`dvx` would produce the
   identity here and not the transpose.

   `worst` is printed as well as `bad` because a reading of 0 is the verdict and a
   reading of 1 or 2 is a different finding from 255 - and because a size,
   addressing or palette fault each have a different shape in the printed
   examples, which is why the first few mismatches are printed too.

   Additive and boot-only, like gpu_dw_probe().  It draws through the same
   operator the hook uses, so it is not a model of the hook - it is the hook. */
bool gpu_transpose_probe(uint32_t* out_bad)
{
    uint32_t     tw, th, i, n, r, c;
    uint32_t     bad, worst, at, got, want;
    uint32_t     tot_bad = 0u, tot_worst = 0u;
    uint32_t     c0, us_a, us_b, us_c, us_d;
    int          ok;

    if (out_bad != NULL) { *out_bad = 0u; }

    if (!s_ready)
    {
        UART_SendString("gpu tr SKIP (not ready)\r\n");
        return false;
    }

    /* --- Case A: 16x16, the whole 0..255 range, transposed ------------------ */
    tw = 16u;
    th = 16u;
    n  = tw * th;
    for (i = 0u; i < n; i++) { s_tr_in[i] = (int8_t)(uint8_t)i; }

    bad = 0u; worst = 0u; at = 0u; got = 0u; want = 0u;
    c0   = DWT->CYCCNT;
    ok   = gpu_transpose_s8(s_tr_in, s_tr_out, tw, th);
    us_a = (DWT->CYCCNT - c0) / 600u;
    if (!ok)
    {
        UART_SendString("gpu tr A 16x16 REFUSED\r\n");
        tot_bad += 1u;
    }
    else
    {
        for (r = 0u; r < th; r++) {
            for (c = 0u; c < tw; c++) {
                const uint32_t o = (c * th) + r;
                const uint32_t e = (r * tw) + c;
                const int d = (int)s_tr_out[o] - (int)s_tr_in[e];
                const uint32_t a = (uint32_t)((d < 0) ? -d : d);
                if (a != 0u) {
                    if (bad == 0u) { at = o; got = (uint32_t)(uint8_t)s_tr_out[o];
                                     want = (uint32_t)(uint8_t)s_tr_in[e]; }
                    bad++;
                    if (a > worst) { worst = a; }
                }
            }
        }
        sprintf(Message, "gpu tr A 16x16 worst %lu bad %lu of %lu "
                         "(at %lu got %lu want %lu)\r\n",
                (unsigned long)worst, (unsigned long)bad, (unsigned long)n,
                (unsigned long)at, (unsigned long)got, (unsigned long)want);
        UART_SendString(Message);
        tot_bad += bad;
        if (worst > tot_worst) { tot_worst = worst; }
    }

    /* --- Case B: the model's own shape, 400x48 = (1,48,20,20) --------------- */
    tw = 400u;
    th = 48u;
    n  = tw * th;
    for (i = 0u; i < n; i++)
    {
        r = i / tw;
        c = i % tw;
        s_tr_in[i] = (int8_t)(uint8_t)((r * 7u) + (c * 31u));
    }

    bad = 0u; worst = 0u; at = 0u; got = 0u; want = 0u;
    c0   = DWT->CYCCNT;
    ok   = gpu_transpose_s8(s_tr_in, s_tr_out, tw, th);
    us_b = (DWT->CYCCNT - c0) / 600u;
    if (!ok)
    {
        UART_SendString("gpu tr B 400x48 REFUSED\r\n");
        tot_bad += 1u;
        if (out_bad != NULL) { *out_bad = tot_bad; }
        return false;
    }

    for (r = 0u; r < th; r++) {
        for (c = 0u; c < tw; c++) {
            const uint32_t o = (c * th) + r;
            const uint32_t e = (r * tw) + c;
            const int d = (int)s_tr_out[o] - (int)s_tr_in[e];
            const uint32_t a = (uint32_t)((d < 0) ? -d : d);
            if (a != 0u) {
                if (bad == 0u) { at = o; got = (uint32_t)(uint8_t)s_tr_out[o];
                                 want = (uint32_t)(uint8_t)s_tr_in[e]; }
                bad++;
                if (a > worst) { worst = a; }
            }
        }
    }

    /* The four examples are at spread positions rather than the first four: a
       UV or geometry fault is periodic in one axis and a stride fault in the
       other, and four adjacent bytes cannot tell those apart. */
    sprintf(Message, "gpu tr B 400x48 worst %lu bad %lu of %lu "
                     "(at %lu got %lu want %lu)\r\n",
            (unsigned long)worst, (unsigned long)bad, (unsigned long)n,
            (unsigned long)at, (unsigned long)got, (unsigned long)want);
    UART_SendString(Message);
    sprintf(Message, "gpu tr B got %lu %lu %lu %lu want %lu %lu %lu %lu "
                     "(at 0 48 9600 19199)\r\n",
            (unsigned long)(uint8_t)s_tr_out[0],
            (unsigned long)(uint8_t)s_tr_out[48],
            (unsigned long)(uint8_t)s_tr_out[9600],
            (unsigned long)(uint8_t)s_tr_out[19199],
            (unsigned long)(uint8_t)s_tr_in[0],
            (unsigned long)(uint8_t)s_tr_in[1],
            (unsigned long)(uint8_t)s_tr_in[200],
            (unsigned long)(uint8_t)s_tr_in[19199]);
    UART_SendString(Message);

    tot_bad += bad;
    if (worst > tot_worst) { tot_worst = worst; }

    /* --- Case C: 40x40, square but NOT a dyadic size ------------------------
       16x16 and 400x48 differ in two ways at once - square vs rectangular, and
       a dyadic gradient vs not - so between them they cannot say which of the
       two a fault needs.  This case is square like the first and non-dyadic
       like the second.  Same identity source, so it also covers 0..255 (1600
       elements) and the palette range is re-read here rather than assumed. */
    tw = 40u;
    th = 40u;
    n  = tw * th;
    for (i = 0u; i < n; i++) { s_tr_in[i] = (int8_t)(uint8_t)i; }

    bad = 0u; worst = 0u; at = 0u; got = 0u; want = 0u;
    c0   = DWT->CYCCNT;
    ok   = gpu_transpose_s8(s_tr_in, s_tr_out, tw, th);
    us_c = (DWT->CYCCNT - c0) / 600u;
    if (!ok)
    {
        UART_SendString("gpu tr C 40x40 REFUSED\r\n");
        tot_bad += 1u;
    }
    else
    {
        for (r = 0u; r < th; r++) {
            for (c = 0u; c < tw; c++) {
                const uint32_t o = (c * th) + r;
                const uint32_t e = (r * tw) + c;
                const int d = (int)s_tr_out[o] - (int)s_tr_in[e];
                const uint32_t a = (uint32_t)((d < 0) ? -d : d);
                if (a != 0u) {
                    if (bad == 0u) { at = o; got = (uint32_t)(uint8_t)s_tr_out[o];
                                     want = (uint32_t)(uint8_t)s_tr_in[e]; }
                    bad++;
                    if (a > worst) { worst = a; }
                }
            }
        }
        sprintf(Message, "gpu tr C 40x40 worst %lu bad %lu of %lu "
                         "(at %lu got %lu want %lu)\r\n",
                (unsigned long)worst, (unsigned long)bad, (unsigned long)n,
                (unsigned long)at, (unsigned long)got, (unsigned long)want);
        UART_SendString(Message);
        tot_bad += bad;
        if (worst > tot_worst) { tot_worst = worst; }
    }

    /* --- Case D: 1600x24, the geometry the hook meets FIRST -----------------
       None of the three above is this shape.  The first [0,2,3,1] instance in
       model order is op17, whose input is (1,24,40,40): tw = a2*a3 = 1600 and
       th = a1 = 24, i.e. a destination tile 24 wide and 1600 tall.  400x48 is
       8:1 the OTHER way; this is 67:1, and 38400 is also the largest instance
       in the model - the one whose bound had to be raised.  If the tile's
       height, its bbox field or the UV gradient runs out of range below 1600,
       the hook meets it on its very first invocation, and this case is far
       cheaper to read than that.  Identity fill like A and C, so the pattern
       wraps 0..255 a hundred and fifty times and a single texel of offset
       still shows. */
    tw = 1600u;
    th = 24u;
    n  = tw * th;               /* 38400 - the model's largest, and the bound */
    for (i = 0u; i < n; i++) { s_tr_in[i] = (int8_t)(uint8_t)i; }

    bad = 0u; worst = 0u; at = 0u; got = 0u; want = 0u;
    /* This is the case the split is taken on, so the gate is set for this call
       and only this call.  The stamps live in g_tr_t; the operator's own callers
       leave g_tr_t_on at zero and pay one `!= 0`. */
    g_tr_t_on = 1u;
    c0   = DWT->CYCCNT;
    ok   = gpu_transpose_s8(s_tr_in, s_tr_out, tw, th);
    us_d = (DWT->CYCCNT - c0) / 600u;
    g_tr_t_on = 0u;
    if (!ok)
    {
        UART_SendString("gpu tr D 1600x24 REFUSED\r\n");
        tot_bad += 1u;
    }
    else
    {
        for (r = 0u; r < th; r++) {
            for (c = 0u; c < tw; c++) {
                const uint32_t o = (c * th) + r;
                const uint32_t e = (r * tw) + c;
                const int d = (int)s_tr_out[o] - (int)s_tr_in[e];
                const uint32_t a = (uint32_t)((d < 0) ? -d : d);
                if (a != 0u) {
                    if (bad == 0u) { at = o; got = (uint32_t)(uint8_t)s_tr_out[o];
                                     want = (uint32_t)(uint8_t)s_tr_in[e]; }
                    bad++;
                    if (a > worst) { worst = a; }
                }
            }
        }
        sprintf(Message, "gpu tr D 1600x24 worst %lu bad %lu of %lu "
                         "(at %lu got %lu want %lu)\r\n",
                (unsigned long)worst, (unsigned long)bad, (unsigned long)n,
                (unsigned long)at, (unsigned long)got, (unsigned long)want);
        UART_SendString(Message);
        tot_bad += bad;
        if (worst > tot_worst) { tot_worst = worst; }
    }

    sprintf(Message, "gpu tr TOT worst %lu bad %lu (0 is the verdict)\r\n",
            (unsigned long)tot_worst, (unsigned long)tot_bad);
    UART_SendString(Message);

    /* THE COST, at boot, so the family decision needs neither a keypress nor a
       real activation.  Four points spanning 150x in n separate the PER-CALL
       constant from the PER-BYTE rate - which is the number the projection onto
       PAD / CONCATENATION / MAX_POOL_2D turns on, and the one that was only a
       guess before.  The split under them then says WHICH part of the constant is
       this operator's own (the CLUT slice and its palette, which the other
       families would not pay) and which part any family would also pay.  The five
       parts sum to us_d, which is the operator end to end. */
    sprintf(Message, "gpu tr us A 256 %lu B 19200 %lu C 1600 %lu D 38400 %lu\r\n",
            (unsigned long)us_a, (unsigned long)us_b,
            (unsigned long)us_c, (unsigned long)us_d);
    UART_SendString(Message);
    sprintf(Message, "gpu tr us D setup %lu pack %lu draw %lu unpack %lu "
                     "rest %lu of %lu\r\n",
            (unsigned long)((g_tr_t[1] - g_tr_t[0]) / 600u),
            (unsigned long)((g_tr_t[2] - g_tr_t[1]) / 600u),
            (unsigned long)((g_tr_t[3] - g_tr_t[2]) / 600u),
            (unsigned long)((g_tr_t[4] - g_tr_t[3]) / 600u),
            (unsigned long)(us_d - ((g_tr_t[4] - g_tr_t[0]) / 600u)),
            (unsigned long)us_d);
    UART_SendString(Message);

    if (out_bad != NULL) { *out_bad = tot_bad; }
    return (tot_bad == 0u);
}

/** @brief  Mode selector for blit_separable_convolve_new().
 *
 *  The three #defines are hoisted to the top of this file, beside
 *  GPU_CONV_TAPS, because stage 3 (filter_frame_with()) selects ROWCOL and is
 *  emitted long before this point. */


/* What the intermediate costs, and it is a measurement rather than a choice.

   Pass 1's destination is an RGB565 plane, and the only part of a 565 texel the
   CLUT kernel multiplies is its BLUE field - so an 8-bit colour result R written
   there comes back smaller, and by a factor two readings pin to BOTH ENDS of:

     `gpu blit rowcol tmp nz 3 first x31 y5 v3`  R = 31 reads a field of 3
                                                 (31*31/255 = 3.77 -> 3)
     `gpu blit T trans ... v1`                   R = 16 reads a field of 1
                                                 (16*31/255 = 1.95 -> 1)

   3.77 -> 3 also settles the direction: it TRUNCATES, it does not round.  And
   16 -> 1 and not 2 is what rules out R>>3, which the first reading alone could
   not (both laws agree on 31).  So the field is

       C = trunc(R * 31 / 255)

   and pass 2 must carry the reciprocal, 255/31 = 8.2258, to hand the caller its
   own gain back.  Three bits do not survive the intermediate, and that is the
   mode's cost, not a bug: an 8-bit row sum is stored in 5 bits and pass 2 puts 8
   back out.

   The headroom rule that follows is the CALLER'S, and it is a real constraint:
   pass 1's gain must leave every row sum inside [8, 255].  Below 8 the field
   truncates to 0 and the tap disappears (a row sum of 5 reads 0), above 255 the
   sum clamps.  A row sum of n taps of a 5-bit field spans up to 31n, so a kernel
   whose tap sum is over ~8 needs a scale below unity - which is why `scale1` is
   a parameter and not a constant.

   The #define itself is hoisted to the top of this file, beside GPU_CONV_TAPS,
   because stage 3 uses it and is emitted long before this point. */


/* The address mode the vendor hands to dhd_set_texture_format() for a pass
   whose kernel is ONE TEXEL WIDE, as a function of that kernel's tap count.

   This is not a tuning choice and not a preference: it is a driver
   REQUIREMENT - davehd_driver_txp_inl.h:551, "the texture adress mode has to
   be E_DHD_ACCESS_VIRTUAL_TILED_4 or higher when the kernel width is 1".  A
   width-1 kernel is exactly the E_FILTER_SAMPLE_SHAPE_COLUMN case
   (davehd_driver_txp_inl.h:581-598), so a COLUMN pass may not be run LINEAR.

   Two independent readings of the vendor's rule, from two of its functions:

     - blit_separable_convolve, nsgpu_lib_dis.txt:28755-28765.  r7 is picked
       from the column kernel's tap count and then handed to
       dhd_set_texture_format as `a_address` (28891-28895).  Decoding r7
       against the enum in davehd_driver.h:467-479: 0x16 =
       E_DHD_ACCESS_VIRTUAL_TILED_4 (n >= 4), 0xe = E_DHD_ACCESS_VIRTUAL_TILED_2
       (n == 3), 0 = E_DHD_ACCESS_LINEAR (n < 3).
     - blit_convolve, already transcribed in this file for the depthwise probe
       (the block above gpu_dw_probe): "addr = 14 (kh==3), 22 (kh>=4), 0".  The
       same three values, keyed on the kernel HEIGHT there because that
       transcription's kernel is transposed.

   Below n == 3 the vendor passes 0, so this returns 0 rather than forcing a
   tiled mode onto a kernel the vendor does not tile. */
static dhd_address_mode_t blit_col_amode(uint32_t n)
{
    if (n >= 4u)
    {
        return E_DHD_ACCESS_VIRTUAL_TILED_4;
    }
    if (n == 3u)
    {
        return E_DHD_ACCESS_VIRTUAL_TILED_2;
    }
    return E_DHD_ACCESS_LINEAR;
}


/**
 * @brief  blit_separable_convolve() with the same three gaps closed.
 *
 * Two modes, dispatched on `mode`:

 *  - BLIT_SEP_ROWS - the CHANNEL ACCUMULATOR.  A 1xN row kernel reads N
 *    consecutive texels of one line, so laying a pixel's Cin activations along
 *    x makes the kernel a row of Cin weights and the draw a dot product.  With
 *    nu == 2 the two texture units take taps [0, n) and [n, 2n) of the SAME
 *    line - each unit on its own CLUT slot, both summed in the colour unit's
 *    wide accumulator in ONE draw, i.e. 2n taps per draw.  Both units carry the
 *    same scale and bias: they are two halves of one dot product, not two
 *    channels.
 *
 *  - BLIT_SEP_ROWCOL - the vendor's rank-1 pair, done with the hardware's own
 *    two shapes.  dhd_set_texture_kernel() selects
 *    E_FILTER_SAMPLE_SHAPE_ROW for a 1xN kernel and _COLUMN for an Nx1 one
 *    (davehd_driver_txp_inl.h:594-598), so one 1-D tap array filtered twice -
 *    once across the row, once down the column - is a rank-1 2D convolution in
 *    two draws.  This IS what the vendor's blit_separable_convolve() does, and
 *    the correspondence is exact rather than approximate: its pass 1 is
 *    dhd_set_texture_kernel(dev, n, 1, k1) (nsgpu_lib_dis.txt:28834-28838) and
 *    its pass 2 is dhd_set_texture_kernel(dev, 1, n, k2) (28898-28902) - ROW
 *    then COLUMN, from two SEPARATE tap buffers, so the function is a general
 *    two-stage separable convolver and the wrapper merely hands it the same n
 *    and the same pointer twice.  **An earlier version of this comment claimed
 *    the vendor hardcoded height 1 in BOTH passes and was effectively
 *    row-then-row.  Those are the same lines it cited, and they refute it.**
 *    What ROWCOL does owe the vendor, and did not have, is the address mode of
 *    the COLUMN pass - see blit_col_amode().
 *
 *  - BLIT_SEP_ROWCOL_T - the same rank-1 pair with BOTH passes read through the
 *    transposed UV (blit_uv_t).  This mode exists because of a measurement, not
 *    a preference: the hardware's COLUMN shape was measured not to deliver an
 *    N-tap vertical sum for N > 2 - **and that measurement is FALSIFIED; the
 *    note at the end of this bullet says how, and stage 3 has moved back onto
 *    mode 1 because of it.**  A 1x3 kernel of ones over a
 *    one-row stripe wrote `gpu blit B col-shape nz 126 first x1 y4 v5`, and the
 *    per-row profile that a total cannot replace - 126 is 2 rows x 63 AND 3
 *    rows x 42 - came back `gpu blit B rows 0 0 0 0 63 63 0 0`: TWO rows, so
 *    two taps of the three.  The two rows' VALUES then differ, 5 against 11 at
 *    the same multiplicand, which is tap weights of 1 and 2 and not a window at
 *    all - the shape walks the kernel as a 2x2 quad rather than down a column.
 *    **That reading was taken on E_DHD_ACCESS_LINEAR, which the driver forbids
 *    for a width-1 kernel and which the vendor never uses there (see
 *    blit_col_amode(), which decodes the vendor's own rule), so it does NOT
 *    establish a limit of the shape.**  The ROW shape has no such limit:
 *    16 taps of one 1x16 kernel were measured exact, `tap16 v 123 d 0/0`.
 *    So a vertical sum is built out of the shape that works, by storing the
 *    intermediate TRANSPOSED - pass 1 writes f1 transposed, pass 2 walks its
 *    taps along tmp's memory columns (which is the source's y) and the
 *    transposed UV puts the result back in the caller's orientation.  Board:
 *    `gpu blit T trans nz 21` with `gpu blit T rows 0 0 0 0 7 7 7 0` - three
 *    rows across seven columns, where a pair that had degenerated to the
 *    vendor's row-then-row would have put eight in ONE row.
 *
 *    Blit mode 1 (the COLUMN shape) was therefore taken to be usable only for
 *    n <= 2, and **that consequence is now FALSIFIED ON THE BOARD (2026-10-08)**.
 *    With test (B) moved onto blit_col_amode(3u), the same 3-tap column kernel
 *    reads `rows 0 0 0 63 63 63 0 0` - THREE rows - and `col-shape nz 189`, i.e.
 *    3 x 63 with one texel's worth (5) each, against the LINEAR run's two rows
 *    and nz 126.  Test 5 moved with it: its destination count went 6 -> 9, the
 *    full ideal 3x3.  So the two rows were a property of the ADDRESS MODE and
 *    not of the shape.
 *
 *    Consequence, and it is the reason to read this before reusing the argument
 *    above: BLIT_SEP_ROWCOL is the vendor's own pair, so the 6.7x this mode
 *    costs (see the stage 3 notes) was paid to avoid a limitation that does not
 *    exist.  **This mode, not mode 1, is therefore the one with no measurement
 *    behind it any more.**  Stage 3 was moved onto mode 1 on 2026-10-08 and the
 *    modes are otherwise interchangeable in this file, so a later reader who
 *    wants mode 2 back has only to change that one argument - but then owes it
 *    the same acceptance `s` was given, not the argument above again.
 *
 *    Preconditions, and both are hard: dw == dh (one tap is one texel in each
 *    direction, so a non-square tile reads past the edge), and sw == sh == 1
 *    (a stride has no single transposed meaning).
 *
 *    Placement: the sampled window is CENTRED on the destination pixel (a tap 0
 *    that lands on the sample's own texel is what `tap16 v 123 d 0/0` settled),
 *    so a 3-tap pass reads sample-1..sample+1 and the outermost tap falls off
 *    the plane's low edge - `gpu blit T trans` shows it as the seven columns of
 *    eight.  Both axes' offsets are CONSTANT, so the caller places the result
 *    exactly by choosing u0/v0; nothing here depends on the data.
 *
 * @param  k    ROWS: `co_count * n * nu` taps, channel-major - channel o
 *              occupies [o*n*nu, (o+1)*n*nu), unit i takes the n at + i*n.
 *              ROWCOL: `n` taps, one shared kernel, applied in both passes.
 * @param  tmp  ROWCOL only: a scratch plane between the two passes, always an
 *              RGB565 plane, so at least `2 * dw * dh` bytes.  Unused by ROWS.
 * @param  scale1, bias1  ROWCOL only: pass 1's gain, needed because a row sum
 *              of n taps does not fit a byte and must be scaled down before it
 *              can be a texture for pass 2.  May be NULL to get 1.0/0.
 *              Unused by ROWS, where `scale`/`bias` are the only gain.
 *              It has a RANGE, not just a value: pass 1's result must land in
 *              [8, 255] or the 565 intermediate's blue field truncates the small
 *              sums away and clamps the large ones (see BLIT_SEP_MID_GAIN).  A
 *              row sum spans up to 31n in field units, so a kernel whose taps
 *              sum above ~8 needs scale1 below unity.
 * @param  scale, bias  ROWCOL: pass 2's gain and the bias, both in the caller's
 *              own output domain.  The intermediate's factor of 255/31 is the
 *              function's business and is applied to `scale` internally, so a
 *              caller that wants `out = k1*x then k2*` passes the same gain it
 *              would to a one-pass filter.
 */
static bool blit_separable_convolve_new(dhd_handle_t dev, uint32_t mode,
                                 uint32_t src, uint32_t dst,
                                 uint32_t tw, uint32_t th,
                                 uint32_t dw, uint32_t dh,
                                 uint32_t dst_stride,
                                 uint32_t tmp,
                                 uint32_t clut_base,
                                 const int8_t* k, uint32_t n, uint32_t nu,
                                 const float* scale, const int32_t* bias,
                                 const float* scale1, const int32_t* bias1,
                                 uint32_t co_first, uint32_t co_count,
                                 float u0, float v0,
                                 uint32_t sh, uint32_t sw,
                                 dhd_texel_org_t texfmt, dhd_pixel_org_t pixfmt,
                                 dhd_address_mode_t amode, uint32_t a_border)
{
    dhd_uv_data_t uv;
    cv_env_t      env;
    blit_fb_t     fb;
    uint32_t      co, i, ents;
    bool          ok = true;

    if ((co_count == 0u) || (co_first >= co_count) ||
        (n == 0u) || (n > 16u) ||
        (sw == 0u) || (sh == 0u) || (dw == 0u) || (dh == 0u))
    {
        return false;
    }
    if ((mode == BLIT_SEP_ROWS) && ((nu == 0u) || (nu > 2u)))
    {
        return false;
    }
    /* The transposed mode's two preconditions (see the doc block): a square
       tile, and no stride. */
    if ((mode == BLIT_SEP_ROWCOL_T) && ((dw != dh) || (sw != 1u) || (sh != 1u)))
    {
        return false;
    }

    cv_env_save(dev, &env);
    fb.fb  = nsgpu_get_framebuffer();
    fb.fmt = dhd_get_framebuffer_format(dev);

    if ((mode == BLIT_SEP_ROWCOL) || (mode == BLIT_SEP_ROWCOL_T))
    {
        for (co = 0u; co < co_count; co++)
        {
            uint32_t o = co - co_first;

            /* ---- pass 1: across the row, source -> tmp, with the stride ----

               sw/tw and not sw/dw: the source is tw wide, so that is the step
               in texture widths (see blit_uv).  tmp is written at the tile's
               own shape, so pass 2's gradient below divides by dw - there the
               texture IS dw wide, and the two are the same statement.

               Transposed mode: the same filter, but the destination's ROW
               drives u (the source's x) and its COLUMN drives v, so tmp comes
               out holding the x-sum TRANSPOSED - tmp[a][b] = f1(x=b, y=a). */
            if (mode == BLIT_SEP_ROWCOL_T)
            {
                blit_uv_t(&uv, u0, v0, 1.0f / (float)tw, 1.0f / (float)th);
            }
            else
            {
                blit_uv(&uv, u0, v0, (float)sw / (float)tw, (float)sh / (float)th);
            }
            dhd_set_texture_unit_enable(dev, 1u);
            dhd_set_uv(dev, 0u, &uv);
            dhd_active_texture_unit(dev, 0u);
            dhd_set_texture_address(dev, (dhd_gpu_ptr_t)src, 0u);
            dhd_set_texture_size(dev, tw, th, tw);
            dhd_set_texture_format(dev, texfmt, amode, 0u);
            dhd_set_texture_wrap(dev, E_DHD_TEX_BORDER, E_DHD_TEX_BORDER);
            dhd_set_texture_border_color(dev, a_border);
            cv_color_setup(dev, E_DHD_OP_COPY);
            /* width n, height 1 -> E_FILTER_SAMPLE_SHAPE_ROW. */
            dhd_set_texture_kernel(dev, n, 1u, (const uint8_t*)k, clut_base);
            dhd_set_texture_clut_base(dev, clut_base);
            dhd_set_texture_scalebias(dev,
                                      (scale1 != NULL) ? scale1[o] : 1.0f,
                                      (bias1  != NULL) ? bias1[o]  : 0);
            /* tmp is ALWAYS an RGB565 plane, at 2 bytes per texel, and the
               caller's `tmp` must be at least 2*dw*dh bytes.  It cannot be the
               8-bit L_8 plane the destination is, and that is a measurement and
               not a preference: an AL_8 texel returns 0 under a CLUT kernel in
               both sign modes - see GPU_DW_FMT_UNSIGNED's comment, where the
               unsigned constant is kept precisely so the zero is not blamed on
               the sign.  RGB565 is the format every kernel in this file is
               measured against.  The cost is that the intermediate carries the
               value in its 5-bit BLUE field, like every other activation this
               file packs: pass 2's multiplicand is pass 1's output >> 3, which
               is why pass 1's scale has to leave the sum above 8. */
            if (!blit_tile(dev, tmp, dw, dh, E_DHD_PIXEL_ORG_RGB_565))
            {
                ok = false;
                break;
            }

            /* ---- pass 2: down the column, tmp -> this channel's plane ----

               tmp is read 1:1 - it was written at the destination's own shape,
               so this pass's gradient is the plain 1/dw, 1/dh - and as 565, the
               format it was written in.  The two modes then differ in HOW the
               second axis is filtered and not in what is read: SAMPLE_SHAPE_
               COLUMN, or the ROW shape over the transposed intermediate.

               tmp may hold pass 1's bytes only because blit_tile() waited for
               the writeback and drained; that dependency is why this mode is
               two draws per channel and not one. */
            if (mode == BLIT_SEP_ROWCOL_T)
            {
                /* Transposed again, so the output lands in the caller's own
                   orientation rather than a transposed one: (dx,dy) reads
                   tmp[dx][dy+j], and tmp[dx][dy+j] = f1(x=dx, y=dy+j) - a
                   vertical sum at column dx, assigned to (dx,dy). */
                blit_uv_t(&uv, 0.0f, 0.0f, 1.0f / (float)dw, 1.0f / (float)dh);
            }
            else
            {
                /* u0 = 0 puts destination column x on texture index x, which is
                   the mapping this pass wants, so it stays 0.

                   Do NOT "fix" column 0 by moving this off zero.  Both offsets
                   were measured on the board and both are worse or useless:
                   HALF a texel is a NO-OP - the coordinate is truncated to a
                   texel index, so 0.5/tw lands in the same cell as 0 and the
                   reading is identical line for line - and a WHOLE texel lands
                   on the next cell, which moves the defect from x0 to x319 and
                   raises the plane's mean from 6 to 8 (the library arm reads
                   5..6).  The lever is the FETCH, not the coordinate: see the
                   wrap mode below. */
                blit_uv(&uv, 0.0f, 0.0f, 1.0f / (float)dw, 1.0f / (float)dh);
            }
            dhd_set_uv(dev, 0u, &uv);
            dhd_set_texture_address(dev, (dhd_gpu_ptr_t)tmp, 0u);
            dhd_set_texture_size(dev, dw, dh, dw);
            /* ROWCOL's second pass IS the COLUMN shape - its kernel is the
               1-wide one set a few lines below - so it may not be run LINEAR;
               see blit_col_amode().  ROWCOL_T's second pass is the ROW shape at
               width n, where LINEAR is legal and is what that leg was verified
               on, so it keeps exactly what it had. */
            dhd_set_texture_format(dev, GPU_DW_FMT_SIGNED_565,
                                   (mode == BLIT_SEP_ROWCOL)
                                       ? blit_col_amode(n)
                                       : E_DHD_ACCESS_LINEAR,
                                   0u);
            /* BOTH axes CLAMP, and the two axes got here by different routes.

               The u axis was a BUG, measured: this pass's u window is one texel
               wide (a point sample), and the board said its index 0 came back as
               the BORDER COLOR, exactly - `pair edge conv` read 0 where dst and
               tmp read 22..23, over 300 pixels of one column, six frames running.
               dhd_set_texture_border_color() documents the border colour as
               "used when the wrap mode is E_DHD_TEX_BORDER and a texel OUTSIDE
               the texture is addressed" and its argument as ARGB8888, so a
               caller passing 0 gets an exact 0 - which is what a fetch that fell
               off the low edge produced.  CLAMP is the same branch taken the
               other way: an outside address replicates the edge texel.  Flipping
               u alone took `over` (pixels whose worst channel is >128 apart)
               from 303..325 to 0, six frames for six, and moved `max` 189 -> 123.

               The v axis is a BOUND, not a measurement, and it is the axis this
               pass's kernel actually walks (the kernel is (1,n) - see the shape
               choice below), so its outside taps are the ones that carry weight.
               Border colour 0 on a weighted tap drags the edge row toward zero:
               with taps summing to 4 the error reaches 3/4 of the value, which
               is exactly the 127-count deficit the board still shows at y319
               (and once at y0) while the library arm - whose second pass has a
               degenerate kernel height and therefore does no vertical filtering
               at all - never leaves the plane's interior.  CLAMP replaces those
               zeros with the edge row, which bounds the same error at 1/4 of the
               LOCAL CONTRAST instead of 3/4 of the value.  That bound holds
               whatever convention the other side of the comparison uses, which
               matters here because the `pair` A/B measures this operator against
               an UNFILTERED CPU bilinear (camera.c:681) - so a residue at the
               edge is expected, and the question is only whether it is a blur
               or a hole.

               Falsifier: if y319 is still the argmax at ~127 after this, the
               outside tap is not what is being zeroed and the wrap mode is not
               the lever. */
            dhd_set_texture_wrap(dev, E_DHD_TEX_CLAMP, E_DHD_TEX_CLAMP);
            dhd_set_texture_border_color(dev, a_border);
            cv_color_setup(dev, E_DHD_OP_COPY);
            /* Safe at the same CLUT offset as pass 1: nothing reads pass 1's
               kernel again, and the upload is what re-points the slot. */
            if (mode == BLIT_SEP_ROWCOL_T)
            {
                /* ROW shape for the second pass too - (n,1) - which is the
                   whole point of transposing: the taps walk u, u now advances
                   down tmp's memory columns, and those are the source's y. */
                dhd_set_texture_kernel(dev, n, 1u, (const uint8_t*)k, clut_base);
            }
            else
            {
                /* width 1 -> E_FILTER_SAMPLE_SHAPE_COLUMN. */
                dhd_set_texture_kernel(dev, 1u, n, (const uint8_t*)k, clut_base);
            }
            dhd_set_texture_clut_base(dev, clut_base);
            /* The 255/31 is the intermediate's, not the caller's: pass 1's
               8-bit result was stored in the 565 plane's 5-bit blue field, so
               pass 2's multiplicand is smaller than what pass 1 computed and
               its gain has to say so.  The bias does NOT scale with it - it is
               added after the truncation, in the output's own domain, which is
               the domain the caller gave it in. */
            dhd_set_texture_scalebias(dev, scale[o] * BLIT_SEP_MID_GAIN,
                                      bias[o]);
            if (!blit_tile(dev, dst + (o * dst_stride), dw, dh, pixfmt))
            {
                ok = false;
                break;
            }
        }
    }
    else
    {
        /* The row branch packs (n+3)>>2 CLUT words per kernel, so two units
           need two slots - the same pair cv_run()'s GPU_CV_CLUT_A/B are. */
        ents = (n + 3u) >> 2u;

        for (co = 0u; co < co_count; co++)
        {
            uint32_t o = co - co_first;

            dhd_set_texture_unit_enable(dev, nu);
            for (i = 0u; i < nu; i++)
            {
                /* Unit i's window starts i*n SAMPLED positions along the same
                   line.  One sampled position is dux - sw/tw of u, the texture
                   being tw wide - so the shift is i*n*sw/tw, written as a pure
                   RELATIVE offset rather than as an absolute tap position:
                   where the hardware puts tap 0 is a measured property
                   (dw_kstride()), and a two-unit sum must not depend on it.
                   Both units share one kernel geometry, so the n-position gap
                   between their windows is exact whatever that position is.
                   (At sw == 1 this is i*n/tw, and that is the arithmetic the
                   board's `gpu blit sep2u got 64` was read under.) */
                blit_uv(&uv,
                        u0 + (((float)(i * n) * (float)sw) / (float)tw),
                        v0, (float)sw / (float)tw, (float)sh / (float)th);
                dhd_set_uv(dev, i, &uv);
                dhd_active_texture_unit(dev, i);
                dhd_set_texture_address(dev, (dhd_gpu_ptr_t)src, 0u);
                dhd_set_texture_size(dev, tw, th, tw);
                dhd_set_texture_format(dev, texfmt, amode, 0u);
                dhd_set_texture_wrap(dev, E_DHD_TEX_BORDER, E_DHD_TEX_BORDER);
                dhd_set_texture_border_color(dev, a_border);
                /* Separated slots: dhd_set_texture_kernel() uploads to the
                   offset it is handed, so one shared offset means the second
                   unit's upload destroys the first unit's kernel. */
                dhd_set_texture_kernel(
                    dev, n, 1u,
                    (const uint8_t*)(k + (o * n * nu) + (i * n)),
                    clut_base + (i * ents));
                dhd_set_texture_clut_base(dev, clut_base + (i * ents));
                dhd_set_texture_scalebias(dev, scale[o], bias[o]);
            }
            /* ADD with op1a = TEXTURE_0 and op1b = TEXTURE_1 is the one-pass
               two-unit sum (davehd_util_state.c:151).  COPY cannot express it:
               COPY is op1a alone, which is what made a first probe read a
               second unit as contributing nothing. */
            cv_color_setup(dev, (nu > 1u) ? E_DHD_OP_ADD : E_DHD_OP_COPY);

            if (!blit_tile(dev, dst + (o * dst_stride), dw, dh, pixfmt))
            {
                ok = false;
                break;
            }
        }
    }

    return blit_restore(dev, &env, &fb) && ok;
}

/* ===================================================================
   The pointwise (CONV_2D) probe, round 2.

   Round 1 answered its first question and got its second one wrong.  On record
   from that log:

     - `gpu cv step o0 0 o1 0 o2 1 bad 0`, `tap16 v 123 d 0/0`,
       `tap8 v 36 d -9/9`, `tap4 v 18 d -13/5`: a 1x16 row kernel really does
       sum 16 consecutive texels, the window advances one texel per destination
       pixel, and tap 0 sits on texel (sample - 1).  That is the structural
       premise of the whole design and it holds.

     - `gpu cv 2u copy 16 add 16 1u 16`, `2u clut 16`, `2u scale 16`: five
       readings, all 16, read at the time as "a second texture unit contributes
       nothing".

   That reading was wrong, and the reason is cv_color_setup()'s, above: nothing
   had selected TEXTURE_1 as a colour operand, so COPY - which is op1a alone -
   was *defined* to return 16.  Five readings that were one fact.  This round
   selects the operands and asks again, so the tests below differ from round
   1's in exactly one way: the operands are explicit, and the wanted reading of
   the two-unit test has moved from `copy` to `add`.

   ROUND 2 RAN, AND THE ANSWER IS YES ON ALL FOUR COUNTS:

     `gpu cv step o0 0 o1 0 o2 1 bad 0`, `tap16 v 123 d 0/0`, `tap8 v 36
     d -9/9`, `tap4 v 18 d -13/5`, `2u copy 16 add 130 1u 16`,
     `2u chain 130`, `2u scale 73`, `draw us 303`, and `bad` = 23.

   - `add 130` against `copy 16`: the two units really are summed, each from
     its own address.  32 taps in one draw is real, which is the premise the
     whole draw budget rests on, and it is no longer an assumption.
   - `chain 130`, the same number: pass 0 copying TEXTURE_0 and pass 1 adding
     TEXTURE_1 to E_DHD_INPUT_PREVIOUS works too, so there are two routes to
     the same draw and the fallback is not a hope.
   - `scale 73` and not 64: the gains are per unit as well.
   - `clut` never printed - the test's sprintf() was MISSING, so it re-sent the
     chain line - but `bad` came back 23 = 1+2+4+16 with bit 8 clear, and bit 8
     is that test, so the CLUT offsets are per unit too.

   AND THE 123-AGAINST-120 IS NOW SETTLED, which is the fourth thing this round
   bought.  `add 130` is what settles it: the kernel's multiplicand is not the
   5-bit code, it is that field REPLICATED to eight bits, `(c<<3)|(c>>2)`, over
   8.  Four readings fix it to the integer:

       123 = (120 + 24/8)     the period-16 ramp, and 24/8 = sum(c>>2)/8 = 3
       130 = 16*8/8 + 16*57/8 the two units, codes 1 and 7
        73 = 16 + 16*(57/8)/2 the same with unit 1's gain halved
        23 = 16 + 57/8 = 23.125, truncated - which is why it read 23 before too

   Round 1 recorded the difference as "a bounded ~2.6% gain, absorbed into M_k,
   residue under 1 LSB".  The size was right, the consequence was not: it is a
   per-texel term whose MEAN is a gain and whose variation is not, so it is
   absorbable only if the per-draw scale is set to do it.  tools/conv_gpu_
   emulate.py measures both readings (CV_REP=1): without the scale carrying
   1/(1+g) the structural verdict falls from 42 to 30 layers, and with it the
   verdict is 42 layers, 14508 draws, 6.43 Mpx, and the SAME seven walls as
   before.  g = sum(w*(c>>2)) / (8*sum(w*c)) is a constant per (co, chunk), and
   a_scale is per draw, so the prepare step can measure it and put it in.
   Calling it "absorbed into M_k" was only true because a_scale IS that M_k
   multiplied by 256 - the absorption is real, it just is not free.

   The probe's numbers are now the DERIVED ones rather than round 1's guesses,
   so this function is a regression test for the law as well: an `add` that
   reads 128 from here on means the replication is not happening and the
   emulator's model is wrong.                                             */
bool gpu_conv_probe(uint32_t* out_bad)
{
    dhd_handle_t dev = (dhd_handle_t)a_handle;
    uint32_t     clut_base = 0u;
    uint32_t     bad = 0u;
    uint32_t     i;
    int8_t       k1[16];        /* one tap, the alignment test's window */
    int8_t       k16[16];
    int8_t       k8[8];
    int8_t       k4[4];
    uint32_t     o0, o1, o2, nstep;
    int          mn16, mx16, mn8, mx8, mn4, mx4;
    uint32_t     v16, v8, v4;
    uint32_t     two, twoa, one, four, scl, chain;
    uint32_t     t0, t1;
    cv_unit_t    u[2];
    cv_env_t     env;

    if (out_bad != NULL) { *out_bad = 0u; }

    if (!s_ready)
    {
        UART_SendString("gpu cv SKIP (not ready)\r\n");
        return false;
    }

    /* Before the first draw of any kind: this probe changes the colour
       operands, and gpu_dw_conv() depends on them.  See cv_env_save(). */
    cv_env_save(dev, &env);

    if (dhd_alloc_clut(dev, 512u, &clut_base) != 0u)
    {
        UART_SendString("gpu cv FAIL alloc_clut\r\n");
        return false;
    }

    for (i = 0u; i < 16u; i++) { k16[i] = 1; k1[i] = 0; }
    for (i = 0u; i <  8u; i++) { k8[i]  = 1; }
    for (i = 0u; i <  4u; i++) { k4[i]  = 1; }
    k1[0] = 1;

    u[0].src   = GPU_CV_SRC0;
    u[0].clut  = clut_base + GPU_CV_CLUT_A;
    u[0].k     = k16;
    u[0].kw    = 16u;
    u[0].scale = 32.0f;

    u[1].src   = GPU_CV_SRC1;
    u[1].clut  = clut_base + GPU_CV_CLUT_A;
    u[1].k     = k16;
    u[1].kw    = 16u;
    u[1].scale = 32.0f;

    /* ---------------------------------------------------------------
       Test 1: does a 1x16 window step ONE TEXEL per destination pixel, and
       where does tap 0 sit?

       A single tap of one at the window's first position, over a period-32
       ramp, so the output row IS the texel index the window starts at.  Three
       things are read off it:

           o0, o1, o2   the raw codes, which name the offset directly.  This
                        is the same convention question the stride-2 depthwise
                        UV turned on (`gpu dw step 1:1 2 2:1 2`), asked where a
                        single tap answers it instead of a difference.
           nstep        how many of 28 consecutive differences are NOT +1.  The
                        row is a wrap-around ramp, so a step of 1 shows as +1
                        everywhere except across the 31->0 wrap, where it is
                        -31; both are counted as one step, and anything else is
                        a failure.  Columns 20..48, which is the widest interior
                        window that is safe for BOTH unknown conventions: a
                        window of 16 anchored at tap 0 reaches 15 texels
                        forward, and 48+15 is the last texel of the texture.  So
                        no tap reads the border colour whatever `off` turns out
                        to be - which matters, because a border tap would show
                        up here as a step fault and the reading would be wrong
                        for a reason that has nothing to do with stepping.

       A window that steps by two, or that follows the UV gradient, or that
       sums something other than consecutive texels, fails this on most of the
       35 differences.                                                       */
    cv_fill(GPU_CV_SRC0, 32u, 0u);      /* blue = x & 31 */
    cv_fill(GPU_CV_SRC1, 32u, 0u);

    u[0].k = k1;
    u[0].kw = 16u;
    cv_run(dev, u, 1u, E_DHD_OP_COPY, CV_MODE_DIRECT);

    o0 = (uint32_t)cv_px(0u, 4u);
    o1 = (uint32_t)cv_px(1u, 4u);
    o2 = (uint32_t)cv_px(2u, 4u);
    nstep = 0u;
    for (i = 21u; i <= 48u; i++) {
        int d = (cv_px((uint32_t)i, 4u) - cv_px(i - 1u, 4u)) & 31;

        if (d != 1) { nstep++; }
    }

    sprintf(Message, "gpu cv step o0 %lu o1 %lu o2 %lu bad %lu\r\n",
            (unsigned long)o0, (unsigned long)o1, (unsigned long)o2,
            (unsigned long)nstep);
    UART_SendString(Message);

    /* o1 == o0, NOT o0 + 1, and that is the `off = -1` convention showing itself
       at the texture's leading edge rather than a fault.  tap 0 sits on texel
       (sample - 1), so destination 0's window starts one texel BEFORE the
       texture and its first tap clamps to texel 0 - the same texel destination 1
       reads.  Both then read 0 and destination 2 reads 1, which is exactly what
       round 2's `o0 0 o1 0 o2 1` says.  Asserting o1 == o0 + 1 there was this
       probe's own bug: it cost bit 1 of `bad` in both rounds and made a passing
       test read as a failure.  The interior (21..48, where every tap is inside)
       is what carries the stepping answer, and `nstep` is that. */
    if ((o1 != o0) || (o2 != ((o1 + 1u) & 31u)) || (nstep != 0u))
    {
        bad |= 1u;
    }

    /* ---------------------------------------------------------------
       Test 2: does an N-tap 1xN window sum N consecutive texels?

       Three widths, over a period-16 ramp - see the block comment above for
       why the prediction is a first-difference range and not a value.  The
       expected readings:

           k = 16   d 0/0      and the value at one column is 123 = 120 + 3,
                               because 16 consecutive codes of a period-16 ramp
                               are one whole cycle whatever the offset
           k =  8   d -9/9
           k =  4   d -13/5

       THOSE ARE DERIVED NUMBERS, not the ones the first run guessed, and the
       three moves away from a plain `sum of codes` (120, -8/8, -12/4) are the
       5-to-8-bit replication the kernel applies to the texel field before it
       multiplies.  For the full period it is exactly sum(c>>2)/8 = 4*(0+1+2+3)/8
       = 3, so 120 becomes 123 with NOTHING left over - and `123 against 120` is
       the whole of what round 1 recorded as an unexplained 2.6% gain on the
       multiplicand.  The other three readings of that round confirm it: the
       two-unit add is 130 = 128 + 16*(7>>2)/8, the halved unit gives 73 = 72 + 1,
       and the CLUT reading is 23.125 which truncates to the same 23 as before.

       120 against 28 against 6 is what the design's 32-tap chunk rests on: if
       every width comes back as four taps, or as a clamp, or as the same
       number, the chunk is not 32 and the whole draw budget is wrong.      */
    cv_fill(GPU_CV_SRC0, 16u, 0u);      /* blue = x & 15 */

    u[0].k = k16; u[0].kw = 16u;
    cv_run(dev, u, 1u, E_DHD_OP_COPY, CV_MODE_DIRECT);
    v16 = (uint32_t)cv_px(32u, 4u);
    cv_diff_range(4u, 20u, 48u, &mn16, &mx16);

    u[0].k = k8; u[0].kw = 8u;
    cv_run(dev, u, 1u, E_DHD_OP_COPY, CV_MODE_DIRECT);
    v8 = (uint32_t)cv_px(32u, 4u);
    cv_diff_range(4u, 20u, 48u, &mn8, &mx8);

    u[0].k = k4; u[0].kw = 4u;
    cv_run(dev, u, 1u, E_DHD_OP_COPY, CV_MODE_DIRECT);
    v4 = (uint32_t)cv_px(32u, 4u);
    cv_diff_range(4u, 20u, 48u, &mn4, &mx4);

    UART_SendString("gpu cv taps want v16=123 d16=0/0 d8=-9/9 d4=-13/5\r\n");
    sprintf(Message, "gpu cv tap16 v %lu d %d/%d\r\n",
            (unsigned long)v16, mn16, mx16);
    UART_SendString(Message);
    sprintf(Message, "gpu cv tap8  v %lu d %d/%d\r\n",
            (unsigned long)v8, mn8, mx8);
    UART_SendString(Message);
    sprintf(Message, "gpu cv tap4  v %lu d %d/%d\r\n",
            (unsigned long)v4, mn4, mx4);
    UART_SendString(Message);

    if ((v16 != 123u) || (mn16 != 0) || (mx16 != 0) ||
        (mn8 != -9) || (mx8 != 9) || (mn4 != -13) || (mx4 != 5))
    {
        bad |= 2u;
    }

    /* ---------------------------------------------------------------
       Test 3: two texture units in one draw, each reading its OWN memory.

       Both units run a 1x16 kernel of ones at 32.0f, so each contributes
       `16 * code`.  The two codes are 1 and 7, and that pair is chosen so that
       six different mechanisms give six different numbers - the first attempt
       used 1 and 3, where 32 meant both "the units were averaged" and "both
       units read SRC0", and a reading that means two things means nothing:

           130  the units are summed, each from its own address  <- wanted;
                the +2 over 128 is the replication again, 16*(7>>2)/8, so a
                reading of exactly 128 would now mean the texel field is NOT
                being replicated and the model in conv_gpu_emulate.py is wrong
            16  only unit 0 was used
           112  only unit 1 was used
            32  both operands read SRC0, i.e. the ADDRESS is not per unit, or
                the operand select is not independent
            64  the units were averaged rather than summed
           255  the units were multiplied, or a 16-bit field wrapped

       THE EXPECTATION THAT MATTERS IS `add` AND NOT `copy`.  The first version
       of this test wanted 128 from COPY and got 16, and read that as "unit 1
       does not contribute" - but COPY is `out = op1a` and nothing else
       (davehd_driver.h:443), so COPY returning 16 was correct all along and
       the test had no way to pass.  ADD is `out = op1a + op1b`, and it is the
       operation that sums two units; with op1a and op1b explicitly selected,
       its expected value here is 16 + 112 = 128.

       The COPY pass over the same two configured units is the control that
       keeps this honest: it must still read 16, i.e. unit 0 alone.  If COPY
       ever reads 128 then something else is summing the units and the ADD
       reading means nothing.

       The unit-0-only pass at the end is the second control: also 16.        */
    cv_fill(GPU_CV_SRC0, 1u, 1u);       /* code 1 everywhere */
    cv_fill(GPU_CV_SRC1, 1u, 7u);       /* code 7 everywhere */

    u[0].k = k16; u[0].kw = 16u; u[0].clut = clut_base + GPU_CV_CLUT_A;
    u[1].k = k16; u[1].kw = 16u; u[1].clut = clut_base + GPU_CV_CLUT_A;

    cv_run(dev, u, 2u, E_DHD_OP_COPY, CV_MODE_DIRECT);
    two  = (uint32_t)cv_px(32u, 4u);
    cv_run(dev, u, 2u, E_DHD_OP_ADD, CV_MODE_DIRECT);
    twoa = (uint32_t)cv_px(32u, 4u);
    cv_run(dev, u, 1u, E_DHD_OP_COPY, CV_MODE_DIRECT);  one  = (uint32_t)cv_px(32u, 4u);

    sprintf(Message, "gpu cv 2u copy %lu add %lu 1u %lu (want add 130)\r\n",
            (unsigned long)two, (unsigned long)twoa, (unsigned long)one);
    UART_SendString(Message);

    if ((twoa != 130u) || (two != 16u) || (one != 16u)) { bad |= 4u; }

    /* ---------------------------------------------------------------
       Test 6: the same two units through CHAINED COLOUR PASSES.

       The chip has three colour passes and the operand enum has
       E_DHD_INPUT_PREVIOUS, so chaining is plainly the mechanism the extra
       passes are for - and it may well be the one that is wired where an
       independent op1b source is not.  Pass 0 copies unit 0; pass 1 adds unit
       1 to the previous pass's output; two passes enabled.  Same units, same
       kernels and same addresses as test 3, so the readings are directly
       comparable and want the same 128:

           128  the chain works and the LAST pass is what reaches the blender
           16   pass 0 reached it, or PREVIOUS carried nothing, so pass 1's
                contribution is lost
           112  pass 1 reached it and PREVIOUS read as zero - unit 1 is
                sampled and the pass is live, only the carry is missing

       This is the fallback and not the design: it is NOT part of `out_bad`.
       If test 3's `add` reads 128 the two routes cost the same and this line
       is decoration; if test 3 reads 16, this one line says whether the design
       is reachable at all without narrowing the chunk to 16 taps.           */
    cv_run(dev, u, 2u, E_DHD_OP_ADD, CV_MODE_CHAIN);
    chain = (uint32_t)cv_px(32u, 4u);

    sprintf(Message, "gpu cv 2u chain %lu (128=ok, 16, 112)\r\n",
            (unsigned long)chain);
    UART_SendString(Message);

    /* ---------------------------------------------------------------
       Test 4: are the two units' CLUT offsets independent?

       Unit 0 keeps the 1x16 of ones (16 taps -> 16); unit 1 gets a 1x16 whose
       only non-zero tap is the first (1 tap -> 7), at a DIFFERENT clut base.
       Both kernels are uploaded by the pass itself, each through its own
       unit's clut base.  The draw is ADD, so the reading is 16 + 7 or not:

           23  the offsets are per unit            <- wanted
            8  they share one register, so the LAST base programmed (unit 1's,
               CLUT_B) is the one both units read: 1*1 + 1*7
          128  they share CLUT_A and unit 1 read the sixteen taps anyway
           16  unit 1 contributed nothing at all, i.e. test 3's failure, so this
               reading is then entailed by test 3 rather than independent

       Without this, "two units" is only half an answer: a shared CLUT offset
       would silently point both halves of a 32-tap chunk at the same weights,
       and a layer would come out wrong in a way that looks like bad data.   */
    u[1].k    = k1;
    u[1].kw   = 16u;
    u[1].clut = clut_base + GPU_CV_CLUT_B;

    cv_run(dev, u, 2u, E_DHD_OP_ADD, CV_MODE_DIRECT);
    four = (uint32_t)cv_px(32u, 4u);
    sprintf(Message, "gpu cv 2u clut %lu (23, 8=shared, 128, 16)\r\n",
            (unsigned long)four);
    UART_SendString(Message);

    if (four != 23u) { bad |= 8u; }

    /* ---------------------------------------------------------------
       Test 5: is the scale per unit?

       Same shapes, unit 1's gain halved.  Unit 0: 16 * 1 * 32/32 = 16.
       Unit 1: 16 * 7 * 16/32 = 56, exactly - 112 * 0.5 has no fractional
       part, which is why a gain of 16 is a clean half and not 0.9.  ADD
       again, so the reading is 16 + 56 or not:

           73  the scales are per unit            <- wanted (72 + the
               replication's 16*(7>>2)/8/2 = 1 on the halved unit)
           64  one shared register, holding unit 1's 16: 8 + 56
          128  the scale was not applied to unit 1: 16 + 112
           16  unit 1 contributed nothing, entailed by test 3

       The design uses the same scale on both units, so a shared register would
       not break it - but it would mean the two halves of a chunk cannot be
       centred or scaled differently, which is worth knowing before the layer
       code is built on the assumption that they can.                       */
    u[0].k     = k16; u[0].kw = 16u; u[0].clut = clut_base + GPU_CV_CLUT_A;
    u[0].scale = 32.0f;
    u[1].k     = k16; u[1].kw = 16u; u[1].clut = clut_base + GPU_CV_CLUT_A;
    u[1].scale = 16.0f;

    cv_run(dev, u, 2u, E_DHD_OP_ADD, CV_MODE_DIRECT);
    scl = (uint32_t)cv_px(32u, 4u);

    sprintf(Message, "gpu cv 2u scale %lu (73, 64=shared)\r\n",
            (unsigned long)scl);
    UART_SendString(Message);

    if (scl != 73u) { bad |= 16u; }

    /* ---------------------------------------------------------------
       And what one two-unit draw costs, at the shape the model actually has.

       Every timing on record for this GPU is either the nsgpu_* wrappers (57 us
       per draw, CLUT allocated and freed per call) or gpu_dw_probe()'s 64x64
       pass with the CLUT held.  This is the number the pointwise plan needs and
       neither of those is it: TWO units configured per draw, a 64x8 destination,
       and the CLUT held across all 64 passes.  The destination is small on
       purpose - the size-independent half of the cost is the half that decides
       whether ~3000 draws is affordable, and a big destination would bury it.

       ADD, not COPY: it is the operation the conv layer will actually issue, and
       the two are not the same cost to set up.  What it measures is a whole
       cv_run() - the sentinel fill, both units' configuration, the colour
       setup, the framebuffer swap and the writeback wait - so it is an upper
       bound on the marginal draw: the layer code uploads a CLUT once per layer
       and a plane's framebuffer once per plane, not per draw.               */
    u[0].scale = 32.0f;
    u[1].scale = 32.0f;

    t0 = DWT->CYCCNT;
    for (i = 0u; i < 64u; i++) { cv_run(dev, u, 2u, E_DHD_OP_ADD, CV_MODE_DIRECT); }
    t1 = DWT->CYCCNT;

    sprintf(Message, "gpu cv draw us %lu tot %lu (64 passes)\r\n",
            (unsigned long)((t1 - t0) / 600u / 64u),
            (unsigned long)((t1 - t0) / 600u));
    UART_SendString(Message);

    cv_env_restore(dev, &env);

    (void)dhd_free_clut(dev, 512u, clut_base);

    /* The filter scale word is the one thing this probe changed that nothing
       else in the system ever sets.  Put it back before returning, or the
       library's blit - which reads that word and never writes it - inherits
       this probe's per-unit gains.  cv_run() set one per unit, so this is the
       exit where it matters most. */
    gpu_sb_reset();

    if (out_bad != NULL) { *out_bad = bad; }
    return true;
}

/* ===================================================================
   blit_convolve_new() / blit_separable_convolve_new() on hardware.

   What the offline work cannot answer, one boot, no trigger:

     - does the per-channel loop actually re-point the kernel, the gain, the
       bias and the destination, or does one of the four stay put?
     - does the stride parameter move the SAMPLING GRID (not just the UV word)?
     - does the eight-bit destination work, and do all four ambient framebuffer
       items come back?
     - does the two-unit row accumulator read the two halves of one line, i.e.
       is unit 1's window n texels past unit 0's?
     - does the Nx1 column pass run DOWN the column, which is the pass the
       vendor's separable blit does not have?

   The expected readings are DERIVED from measurements already on record, not
   guessed, and the two that are bands rather than values are bands because the
   hardware's tap-0 position is itself only pinned to a texel (dw_kstride()).
   A band that excludes every wrong reading is worth more than a point that
   assumes one.                                                     */

/* First destination column in row `y` whose byte exceeds `thr`, or dw if none.
   Used by the stride test, which is a statement about WHERE the grid samples
   and so needs a position and not a value. */
static uint32_t blit_first_over(uint32_t plane, uint32_t dw, uint32_t y,
                                uint32_t thr)
{
    uint32_t x;

    for (x = 0u; x < dw; x++)
    {
        if ((uint32_t)dw_rd(plane, (y * dw) + x) > thr) { return x; }
    }
    return dw;
}

/* Non-zero count over a (2r+1) square centred on (cx, cy), counted in 16-BIT
   pixels.  Used by the row-then-column test, whose verdict is a SHAPE: a 3x3
   block is nine pixels and a 1x3 line is three, and no value reading
   distinguishes those.  16-bit because that test's destination is a 565 plane,
   where counting bytes would double every hit. */
static uint32_t blit_count_nz16(uint32_t plane, uint32_t dw, uint32_t dh,
                                uint32_t cx, uint32_t cy, uint32_t r)
{
    uint32_t x, y, n = 0u;

    for (y = (cy > r) ? (cy - r) : 0u; (y <= (cy + r)) && (y < dh); y++)
    {
        for (x = (cx > r) ? (cx - r) : 0u; (x <= (cx + r)) && (x < dw); x++)
        {
            if (dw_rd16(plane, (y * dw) + x) != 0u) { n++; }
        }
    }
    return n;
}

/* Coordinates and value of the first non-zero 16-bit pixel in RASTER order, or
   0 for an empty plane.  Where a localized count cannot separate "nothing was
   written" from "it was written somewhere else", this does - and those two have
   different causes, so the coordinate is the reading and not a nicety.  ox/oy
   are left at 0 when the plane is empty. */
static uint32_t blit_first_nz16(uint32_t plane, uint32_t dw, uint32_t dh,
                                uint32_t* ox, uint32_t* oy)
{
    uint32_t x, y, v;

    for (y = 0u; y < dh; y++)
    {
        for (x = 0u; x < dw; x++)
        {
            v = (uint32_t)dw_rd16(plane, (y * dw) + x);
            if (v != 0u) { *ox = x; *oy = y; return v; }
        }
    }
    return 0u;
}

/* The 8-bit twin: non-zero count over the whole plane, plus the first non-zero
   byte's coordinates and value.  It exists rather than being folded into the
   16-bit scan because an L_8 destination is what a kernel pass writes when the
   question is whether the KERNEL contributed, and counting 16-bit pixels there
   would double every hit. */
static uint32_t blit_scan8(uint32_t plane, uint32_t dw, uint32_t dh,
                           uint32_t* ox, uint32_t* oy, uint32_t* ov)
{
    uint32_t x, y, v, n = 0u, got = 0u;

    for (y = 0u; y < dh; y++)
    {
        for (x = 0u; x < dw; x++)
        {
            v = (uint32_t)dw_rd(plane, (y * dw) + x);
            if (v != 0u)
            {
                if (got == 0u) { *ox = x; *oy = y; *ov = v; got = 1u; }
                n++;
            }
        }
    }
    return n;
}

/* Non-zero count PER ROW, words == 2 for a 16-bit plane (RGB565) and 1 for an
   8-bit one (L_8).  A total and a first coordinate cannot separate a band from a
   block, and in this probe that is the difference between "the column window is
   three taps" and "it is two": 126 is 2 rows x 63 and also 3 rows x 42, and only
   the profile says which. */
static void blit_row_prof(uint32_t plane, uint32_t dw, uint32_t dh,
                          uint32_t words, uint32_t* out)
{
    uint32_t x, y;

    for (y = 0u; y < dh; y++)
    {
        out[y] = 0u;
        for (x = 0u; x < dw; x++)
        {
            uint32_t v = (words == 2u) ? (uint32_t)dw_rd16(plane, (y * dw) + x)
                                       : (uint32_t)dw_rd(plane, (y * dw) + x);
            if (v != 0u) { out[y]++; }
        }
    }
}

bool gpu_blit_probe(uint32_t* out_bad)
{
    dhd_handle_t dev = (dhd_handle_t)a_handle;
    uint32_t     clut_base = 0u;
    uint32_t     bad = 0u;
    uint32_t     i, x;
    int8_t       k16x5[5 * 16];      /* five channels of 1x16 taps     */
    int8_t       k1[1];
    int8_t       k3[3];
    int8_t       k123[3];            /* the asymmetric 3-tap the outer product needs */
    int8_t       ksep[16 * 2];       /* two channels of two 1x16 units */
    uint32_t     v0, v1, v2, v3, v4;
    uint32_t     c1, c2, plus;
    uint32_t     n1, n2, nz1, nz2, fx1, fy1, fx2, fy2;
    uint32_t     n3, n4, ax, ay, av, bx, by, bv;
    uint32_t     n5, n6, n7, cx, cy, cv2, d2x, d2y, d2v;
    uint32_t     t4n, t4x, t4y, t4v;
    uint32_t     prof[8];
    const uint32_t blit_tmp = GPU_DW_DST + 0x00010000u;   /* the rowcol scratch */
    /* The B/B2/B3/T planes, all inside GPU_DW_DST's 192 KB: +0x20000 is B's
       L_8 result, +0x22200 B2's 565 one, +0x24000 B3's, +0x26000 T's. */
    const uint32_t blit_b2 = GPU_DW_DST + 0x00022000u;
    const uint32_t blit_b3 = GPU_DW_DST + 0x00024000u;
    const uint32_t blit_t2 = GPU_DW_DST + 0x00026000u;
    /* T3's 64x64 L_8 plane - 4 KB at +0x28000, still inside the 192 KB. */
    const uint32_t blit_t4 = GPU_DW_DST + 0x00028000u;
    uint32_t     fmt_a, fmt_b, w_a, h_a, p_a, w_b, h_b, p_b;
    uint32_t     x0a, y0a, x1a, y1a, ysa, x0b, y0b, x1b, y1b, ysb;
    cv_env_t     env;

    if (out_bad != NULL) { *out_bad = 0u; }

    if (!s_ready)
    {
        UART_SendString("gpu blit SKIP (not ready)\r\n");
        return false;
    }

    cv_env_save(dev, &env);

    /* The same 512-word slice gpu_dw_probe() and gpu_conv_probe() both take,
       and the same reason: the CLUT is one global resource and this probe runs
       at boot with nothing else holding it. */
    if (dhd_alloc_clut(dev, 512u, &clut_base) != 0u)
    {
        UART_SendString("gpu blit FAIL alloc_clut\r\n");
        return false;
    }

    /* ---------------------------------------------------------------
       Test 1: the per-channel loop.

       Five output channels off one texture, four planes and four readings.  A
       period-16 ramp is the pattern the `gpu cv tap16` block already reads 123
       from, so every number below is anchored to a measurement rather than to
       this probe's own arithmetic:

           ch0  16 ones, scale 32, bias  0  ->  123   (on record)
           ch1  16 ones, scale 32, bias 10  ->  133   bias is per channel
           ch2  16 ones, scale 32, bias 20  ->  143   and so is the plane
           ch3  16 ones, scale 64, bias  0  ->  246   scale is per channel too
           ch4  16 zeros, scale 32, bias  0 ->    0   the KERNEL pointer moves

       Read from four DIFFERENT planes, so a destination that did not advance
       would make all five readings equal to the last channel's.  ch4 is the
       kernel test and it is the one a shared kernel pointer cannot fake: with
       the pointer stuck on ch3's taps it reads 246, not 0.

       The bias readings are the strongest of the five because the bias is
       added AFTER the shift as an integer, so it moves the output by exactly
       its own value - there is no truncation to make 133 approximate. */
    for (i = 0u; i < 16u; i++) { k16x5[0 * 16 + i] = 1; }
    for (i = 0u; i < 16u; i++) { k16x5[1 * 16 + i] = 1; }
    for (i = 0u; i < 16u; i++) { k16x5[2 * 16 + i] = 1; }
    for (i = 0u; i < 16u; i++) { k16x5[3 * 16 + i] = 1; }
    for (i = 0u; i < 16u; i++) { k16x5[4 * 16 + i] = 0; }

    {
        const float   sc[5] = { 32.0f, 32.0f, 32.0f, 64.0f, 32.0f };
        const int32_t bi[5] = { 0, 10, 20, 0, 0 };

        cv_fill(GPU_CV_SRC0, 16u, 0u);      /* blue = x & 15 */

        if (!blit_convolve_new(dev,
                               GPU_CV_SRC0, GPU_DW_DST,
                               GPU_CV_W, GPU_CV_H, GPU_CV_W, GPU_CV_H,
                               GPU_CV_W * GPU_CV_H,
                               clut_base,
                               k16x5, 16u, 1u, sc, bi, 0u, 5u,
                               0.0f, 0.0f, 1u, 1u,
                               GPU_DW_FMT_SIGNED_565, E_DHD_PIXEL_ORG_L_8,
                               E_DHD_ACCESS_LINEAR, 0u))
        {
            UART_SendString("gpu blit c5 FAIL returned\r\n");
            bad |= 1u;
        }
    }

    v0 = (uint32_t)dw_rd(GPU_DW_DST + (0u * GPU_CV_W * GPU_CV_H),
                         (4u * GPU_CV_W) + 32u);
    v1 = (uint32_t)dw_rd(GPU_DW_DST + (1u * GPU_CV_W * GPU_CV_H),
                         (4u * GPU_CV_W) + 32u);
    v2 = (uint32_t)dw_rd(GPU_DW_DST + (2u * GPU_CV_W * GPU_CV_H),
                         (4u * GPU_CV_W) + 32u);
    v3 = (uint32_t)dw_rd(GPU_DW_DST + (3u * GPU_CV_W * GPU_CV_H),
                         (4u * GPU_CV_W) + 32u);
    v4 = (uint32_t)dw_rd(GPU_DW_DST + (4u * GPU_CV_W * GPU_CV_H),
                         (4u * GPU_CV_W) + 32u);

    UART_SendString("gpu blit c5 want 123 133 143 246 0\r\n");
    sprintf(Message, "gpu blit c5 got %lu %lu %lu %lu %lu\r\n",
            (unsigned long)v0, (unsigned long)v1, (unsigned long)v2,
            (unsigned long)v3, (unsigned long)v4);
    UART_SendString(Message);

    if ((v0 != 123u) || (v1 != 133u) || (v2 != 143u) ||
        (v3 != 246u) || (v4 != 0u))
    {
        bad |= 2u;
    }

    /* ---------------------------------------------------------------
       Test 2: the stride.

       A step texture and a ONE-tap kernel, so the output is a single edge and
       its position IS the sampling grid.  At stride 1 a 64-texel texture into a
       64-wide destination puts the edge at column 32; at stride 2 into a 32-wide
       destination it lands at column 16.  The verdict is the DIFFERENCE, 16,
       because that cancels the one thing this cannot know: where the kernel's
       single tap sits relative to the sample (dw_kstride() pins it to a texel,
       not to a position).  Both runs use the same kernel and the same u0, so a
       stride word that is written and not acted on gives a difference of 0.

       Measured on the board: 33 and 17 - those two positions, both one sampled
       step late.  The offset is the tap-0 placement and it cancels; see the
       verdict below, which is written as a difference for that reason. */
    for (i = 0u; i < GPU_CV_H; i++)
    {
        for (x = 0u; x < GPU_CV_W; x++)
        {
            uint8_t c = (x < (GPU_CV_W / 2u)) ? 0u : 31u;

            dw_wr(GPU_CV_SRC0, (i * GPU_CV_W + x) * 2u, c);
            dw_wr(GPU_CV_SRC0, ((i * GPU_CV_W + x) * 2u) + 1u, 0u);
        }
    }

    k1[0] = 1;

    {
        const float   s1[1] = { 32.0f };
        const int32_t b1[1] = { 0 };

        (void)blit_convolve_new(dev, GPU_CV_SRC0, GPU_DW_DST,
                                GPU_CV_W, GPU_CV_H, GPU_CV_W, GPU_CV_H,
                                GPU_CV_W * GPU_CV_H, clut_base,
                                k1, 1u, 1u, s1, b1, 0u, 1u,
                                0.0f, 0.0f, 1u, 1u,
                                GPU_DW_FMT_SIGNED_565, E_DHD_PIXEL_ORG_L_8,
                                E_DHD_ACCESS_LINEAR, 0u);
        c1 = blit_first_over(GPU_DW_DST, GPU_CV_W, 4u, 15u);

        /* Half the destination width, so the same 64 texels are covered: one
           destination column per TWO texels. */
        (void)blit_convolve_new(dev, GPU_CV_SRC0, GPU_DW_DST,
                                GPU_CV_W, GPU_CV_H, GPU_CV_W / 2u, GPU_CV_H,
                                GPU_CV_W * GPU_CV_H, clut_base,
                                k1, 1u, 1u, s1, b1, 0u, 1u,
                                0.0f, 0.0f, 1u, 2u,
                                GPU_DW_FMT_SIGNED_565, E_DHD_PIXEL_ORG_L_8,
                                E_DHD_ACCESS_LINEAR, 0u);
        c2 = blit_first_over(GPU_DW_DST, GPU_CV_W / 2u, 4u, 15u);
    }

    /* The board reads ONE COLUMN LATER than the pure geometry (33 and 17, where
       32 and 16 were predicted).  That offset is a constant in SAMPLED
       positions - it is where the hardware puts tap 0 - so the DIFFERENCE is
       the measurement and the two absolute positions get a band and not an
       equality.  The first version of this check demanded 32 exactly and
       therefore could not pass even with the stride right; what it actually
       caught, at 9, was the second and real defect: a gradient divided by the
       DESTINATION width instead of the texture's, which makes the step
       sw*tw/dw = 4 texels at this test's dw=32, tw=64, sw=2 instead of 2. */
    UART_SendString("gpu blit stride want 32|33 16|17 (d 16)\r\n");
    sprintf(Message, "gpu blit stride got %lu %lu (d %ld)\r\n",
            (unsigned long)c1, (unsigned long)c2,
            (long)((int32_t)c1 - (int32_t)c2));
    UART_SendString(Message);

    if ((c1 < 32u) || (c1 > 33u) || (c2 < 16u) || (c2 > 17u) ||
        (((int32_t)c1 - (int32_t)c2) != 16))
    {
        bad |= 4u;
    }

    /* ---------------------------------------------------------------
       Test 3: the four ambient framebuffer items, and the 8-bit destination.

       Read all four before and after a call and compare.  This is the check
       cv_run() does NOT have - it restores the format and the address and
       leaves the size and the box at its own 64x8 tile - so an end-to-end
       reading of it here is worth having for the new functions even though
       blit_restore() is dw_pass_bc()'s measured sequence.

       It is also the 8-bit proof: the readings above came out of GPU_DW_DST as
       one byte per pixel, which is a format the pass could not have written
       into had the destination stayed RGB565. */
    fmt_a = (uint32_t)dhd_get_framebuffer_format(dev);
    dhd_get_framebuffer_size(dev, &w_a, &h_a, &p_a);
    dhd_get_bbox(dev, &x0a, &y0a, &x1a, &y1a, &ysa);

    {
        const float   s1[1] = { 32.0f };
        const int32_t b1[1] = { 0 };

        (void)blit_convolve_new(dev, GPU_CV_SRC0, GPU_DW_DST,
                                GPU_CV_W, GPU_CV_H, GPU_CV_W, GPU_CV_H,
                                GPU_CV_W * GPU_CV_H, clut_base,
                                k1, 1u, 1u, s1, b1, 0u, 1u,
                                0.0f, 0.0f, 1u, 1u,
                                GPU_DW_FMT_SIGNED_565, E_DHD_PIXEL_ORG_L_8,
                                E_DHD_ACCESS_LINEAR, 0u);
    }

    fmt_b = (uint32_t)dhd_get_framebuffer_format(dev);
    dhd_get_framebuffer_size(dev, &w_b, &h_b, &p_b);
    dhd_get_bbox(dev, &x0b, &y0b, &x1b, &y1b, &ysb);

    sprintf(Message, "gpu blit fb ok %u (size %lu x %lu p %lu -> %lu x %lu p %lu)\r\n",
            (unsigned)((fmt_a == fmt_b) && (w_a == w_b) && (h_a == h_b) &&
                       (p_a == p_b) && (x0a == x0b) && (y0a == y0b) &&
                       (x1a == x1b) && (y1a == y1b) && (ysa == ysb)),
            (unsigned long)w_a, (unsigned long)h_a, (unsigned long)p_a,
            (unsigned long)w_b, (unsigned long)h_b, (unsigned long)p_b);
    UART_SendString(Message);

    if ((fmt_a != fmt_b) || (w_a != w_b) || (h_a != h_b) || (p_a != p_b) ||
        (x0a != x0b) || (y0a != y0b) || (x1a != x1b) || (y1a != y1b) ||
        (ysa != ysb))
    {
        bad |= 8u;
    }

    /* ---------------------------------------------------------------
       Test 4: BLIT_SEP_ROWS and the n-texel gap between the two units.

       ONE channel, TWO units, 16 taps each, so unit 0's window is texels
       [w0, w0+15] and unit 1's is [w0+16, w0+31].  The texture is code 1 over
       texels 0..31 and code 3 over 32..63, and the reading is taken at a
       destination column whose windows straddle that boundary - so unit 0 sums
       sixteen 1s and unit 1 sixteen 3s.  With the recorded multiplicand
       (code 1 -> 1, code 3 -> 3) and scale 32 the sum is 16 + 48 = 64.

       IT IS A BAND, 60..68, and the width is the honest part: the tap-0 offset
       is only pinned to a texel, so the boundary can fall one texel either way
       inside the two windows (62 at one end, 66 at the other).  What the band
       excludes is every way this can be wrong - 16 means unit 1 contributed
       nothing, 32 means both units read the SAME window (the offset is not
       applied), 48 means ONLY unit 1 contributed, and 0 means neither did. */
    for (i = 0u; i < GPU_CV_H; i++)
    {
        for (x = 0u; x < GPU_CV_W; x++)
        {
            uint8_t c = (x < (GPU_CV_W / 2u)) ? 1u : 3u;

            dw_wr(GPU_CV_SRC0, (i * GPU_CV_W + x) * 2u, c);
            dw_wr(GPU_CV_SRC0, ((i * GPU_CV_W + x) * 2u) + 1u, 0u);
        }
    }
    for (i = 0u; i < 16u; i++) { ksep[0 * 32 + i] = 1; ksep[0 * 32 + 16 + i] = 1; }

    v0 = 0u;
    {
        const float   s1[1] = { 32.0f };
        const int32_t b1[1] = { 0 };

        if (!blit_separable_convolve_new(dev, BLIT_SEP_ROWS,
                                         GPU_CV_SRC0, GPU_DW_DST,
                                         GPU_CV_W, GPU_CV_H, GPU_CV_W, GPU_CV_H,
                                         GPU_CV_W * GPU_CV_H, 0u, clut_base,
                                         ksep, 16u, 2u, s1, b1, NULL, NULL,
                                         0u, 1u, 0.0f, 0.0f, 1u, 1u,
                                         GPU_DW_FMT_SIGNED_565,
                                         E_DHD_PIXEL_ORG_L_8,
                                         E_DHD_ACCESS_LINEAR, 0u))
        {
            UART_SendString("gpu blit sep2u FAIL returned\r\n");
            bad |= 16u;
        }
        v0 = (uint32_t)dw_rd(GPU_DW_DST, (4u * GPU_CV_W) + 17u);
    }

    UART_SendString("gpu blit sep2u want 60..68 (16=1unit 32=same win 48=unit1 only)\r\n");
    sprintf(Message, "gpu blit sep2u got %lu\r\n", (unsigned long)v0);
    UART_SendString(Message);

    if ((v0 < 60u) || (v0 > 68u)) { bad |= 32u; }

    /* ---------------------------------------------------------------
       Test 5: BLIT_SEP_ROWCOL, and whether the second pass goes DOWN.

       An impulse - one texel of code 31 in a field of zeros - through a
       three-tap {1,1,1} kernel applied twice.  Rank-1 separable convolution of
       an impulse gives a 3x3 BLOCK: nine non-zero pixels.  If the second pass
       is another row pass - which is what the vendor's separable blit really
       does, since it hardcodes height 1 in both - the result is a 1x3 LINE, i.e.
       three.  Nothing about a pixel VALUE separates nine from three, which is
       why the verdict is a count.

       The destination is RGB565 and not L_8, because the intermediate has to be
       a format a CLUT kernel can read and an AL_8 texel reads 0 in the kernel.
       So this pass goes into a 565 plane and the count is over 16-BIT pixels.

       The scales are what the 5-bit intermediate demands, not round numbers.
       The impulse's code 31 has multiplicand 31.875, so pass 1 at scale 32
       writes 31 - and a 565 destination keeps the top five bits of each
       channel, i.e. 31 >> 3 = 3 in the blue field.  Pass 2 then sums three 3s
       and scale 32 returns 9 a pixel.  A pass-1 scale of 8 would write 7, whose
       blue field is 0, and the whole block would vanish for a reason that has
       nothing to do with the column pass - so the choice of 32 is load
       bearing. */
    for (i = 0u; i < GPU_CV_H; i++)
    {
        for (x = 0u; x < GPU_CV_W; x++)
        {
            dw_wr(GPU_CV_SRC0, (i * GPU_CV_W + x) * 2u, 0u);
            dw_wr(GPU_CV_SRC0, ((i * GPU_CV_W + x) * 2u) + 1u, 0u);
        }
    }
    dw_wr(GPU_CV_SRC0, ((4u * GPU_CV_W) + 32u) * 2u, 31u);

    k3[0] = 1; k3[1] = 1; k3[2] = 1;

    plus = 0u;
    {
        const float   s2[1] = { 32.0f };
        const int32_t b2[1] = { 0 };
        const float   s1[1] = { 32.0f };
        const int32_t b1[1] = { 0 };

        /* tmp 64 KB into GPU_DW_DST's 192 KB: clear of the five 512-byte
           channel planes at +0 that test 1 uses, and of the source. */
        if (!blit_separable_convolve_new(dev, BLIT_SEP_ROWCOL,
                                         GPU_CV_SRC0, GPU_DW_DST,
                                         GPU_CV_W, GPU_CV_H, GPU_CV_W, GPU_CV_H,
                                         GPU_CV_W * 2u,
                                         blit_tmp,
                                         clut_base,
                                         k3, 3u, 1u, s2, b2, s1, b1,
                                         0u, 1u, 0.0f, 0.0f, 1u, 1u,
                                         GPU_DW_FMT_SIGNED_565,
                                         E_DHD_PIXEL_ORG_RGB_565,
                                         E_DHD_ACCESS_LINEAR, 0u))
        {
            UART_SendString("gpu blit rowcol FAIL returned\r\n");
            bad |= 64u;
        }
        plus = blit_count_nz16(GPU_DW_DST, GPU_CV_W, GPU_CV_H, 32u, 4u, 3u);

        /* The splitter, and the reason `plus` alone is not a diagnosis.  A zero
           at (32,4) has two very different parents: pass 1 wrote an empty
           intermediate, or pass 2 failed to read a full one - and they point at
           opposite halves of the mode.  So both planes are also read over their
           WHOLE extent and reported by first non-zero pixel in raster order: a
           localized count cannot tell "nothing written" from "written somewhere
           else", and the coordinate is what separates those.  tmp non-empty and
           dst empty means pass 2; tmp empty means pass 1, and pass 1 is the
           part of this mode that writes an 8-bit colour result into a 565
           framebuffer - the one step in it with no precedent on this board. */
        n1  = blit_count_nz16(blit_tmp, GPU_CV_W, GPU_CV_H, 32u, 4u, 64u);
        n2  = blit_count_nz16(GPU_DW_DST, GPU_CV_W, GPU_CV_H, 32u, 4u, 64u);
        fx1 = fy1 = 0u;
        fx2 = fy2 = 0u;
        nz1 = blit_first_nz16(blit_tmp, GPU_CV_W, GPU_CV_H, &fx1, &fy1);
        nz2 = blit_first_nz16(GPU_DW_DST, GPU_CV_W, GPU_CV_H, &fx2, &fy2);

        sprintf(Message, "gpu blit rowcol tmp nz %lu first x%lu y%lu v%lu\r\n",
                (unsigned long)n1, (unsigned long)fx1, (unsigned long)fy1,
                (unsigned long)nz1);
        UART_SendString(Message);
        sprintf(Message, "gpu blit rowcol dst nz %lu first x%lu y%lu v%lu\r\n",
                (unsigned long)n2, (unsigned long)fx2, (unsigned long)fy2,
                (unsigned long)nz2);
        UART_SendString(Message);

        /* ---- (A) tmp read back through the shape that is ALREADY proven ----

           A 1xN row kernel is what every other pass in this file uses and what
           cv_run() measures on this board, so a row pass over tmp removes the
           one thing ROWCOL adds that nothing else here does: its texture is a
           plane the GPU WROTE as RGB565.  Every other 565 texture in this file
           was filled by the CPU (dw_wr), so "a GPU-written 565 plane read back
           as a texture" is a link with no precedent - and it is the link the
           destination-of-pass-1 decision rests on.  If this is zero as well, the
           column shape is not the fault; if it reads, the column shape is.

           The streak pass 1 left is three pixels of blue 3, so a working read
           gives sixes and nines here, not zeros. */
        ax = ay = av = 0u;
        (void)blit_convolve_new(dev, blit_tmp, GPU_DW_DST,
                                GPU_CV_W, GPU_CV_H, GPU_CV_W, GPU_CV_H,
                                GPU_CV_W * GPU_CV_H, clut_base,
                                k3, 3u, 1u, s2, b2, 0u, 1u,
                                0.0f, 0.0f, 1u, 1u,
                                GPU_DW_FMT_SIGNED_565, E_DHD_PIXEL_ORG_L_8,
                                E_DHD_ACCESS_LINEAR, 0u);
        n3 = blit_scan8(GPU_DW_DST, GPU_CV_W, GPU_CV_H, &ax, &ay, &av);
        sprintf(Message, "gpu blit A tmp-thru-row nz %lu first x%lu y%lu v%lu\r\n",
                (unsigned long)n3, (unsigned long)ax, (unsigned long)ay,
                (unsigned long)av);
        UART_SendString(Message);

        /* ---- (B) the COLUMN shape with nothing else in the way ----

           A 1-wide, 3-tall kernel is E_FILTER_SAMPLE_SHAPE_COLUMN by the
           driver's own rule (davehd_driver_txp_inl.h:594-598: shape = 2 when
           height != 1, regardless of width), and this is the only place that
           shape is exercised with no GPU-written texture in front of it - so a
           wrong answer here is the shape's and nothing else's.

           The source is one FULL ROW of non-zeros, and that choice is the test.
           A single impulse would not do: with one impulse the first non-zero
           output pixel of a 3-wide row window and of a 3-tall column window are
           the same pixel (about (31,5) - one sampled position late in both axes,
           as pass 1's own reading shows), so the coordinate could not tell the
           two shapes apart and the reading would be "it contributed, somehow".
           A row-constant stripe separates them twice over:

             column window -> 3 output rows x 64 columns non-zero = about 192,
                              each byte one texel's worth: code 5 -> 5
             row window    -> 1 output row  x 64 columns non-zero = 64,
                              each byte three of them summed: 3 x 5.625 = 17
             no window     -> 0

           So the COUNT says which shape ran and the VALUE says it again, which
           is what makes this a diagnosis and not just a pass/fail.  (Code 5 in
           the blue field rather than 31 so the byte cannot saturate: its
           multiplicand is 5 + 5/8 = 5.625, and scale 32 is gain ONE, not 32
           times - `c5 got 123` settles that, since a 16-tap all-ones row over
           blue codes 0..15 sums to 120 + 24/8 = 123 at scale 32 and 246 at
           scale 64, i.e. the field's unit is 1/32.)

           ** The address mode is the vendor's, NOT LINEAR. **  This test used to
           pass E_DHD_ACCESS_LINEAR, which the driver rules out for a width-1
           kernel (davehd_driver_txp_inl.h:551) and which the vendor never uses
           for one: it switches to E_DHD_ACCESS_VIRTUAL_TILED_2 at n == 3 and to
           _4 from n == 4 on (blit_col_amode()).  So the profile this test
           produced under LINEAR - `rows 0 0 0 0 63 63 0 0` - does NOT establish
           that the COLUMN shape delivers two taps and no more.  It was taken in
           a configuration the driver calls unsupported, and the count it
           produced is a property of that configuration, not of the shape.  The
           reading that decides the question is this same one, at the same n, on
           blit_col_amode(3u) - and on 2026-10-08 it came back `col-shape nz 189
           first x1 y3 v5` with `rows 0 0 0 63 63 63 0 0`: THREE rows, 3 x 63,
           one texel's worth each.  The COLUMN shape is fine at n == 3, and the
           LINEAR configuration was the whole of the defect. */
        for (i = 0u; i < GPU_CV_H * GPU_CV_W; i++)
        {
            dw_wr(GPU_CV_SRC0, i * 2u, 0u);
            dw_wr(GPU_CV_SRC0, (i * 2u) + 1u, 0u);
        }
        for (x = 0u; x < GPU_CV_W; x++)
        {
            dw_wr(GPU_CV_SRC0, ((4u * GPU_CV_W) + x) * 2u, 5u);
        }

        bx = by = bv = 0u;
        (void)blit_convolve_new(dev, GPU_CV_SRC0, GPU_DW_DST + 0x00020000u,
                                GPU_CV_W, GPU_CV_H, GPU_CV_W, GPU_CV_H,
                                GPU_CV_W * GPU_CV_H, clut_base,
                                k3, 1u, 3u, s2, b2, 0u, 1u,
                                0.0f, 0.0f, 1u, 1u,
                                GPU_DW_FMT_SIGNED_565, E_DHD_PIXEL_ORG_L_8,
                                /* NOT E_DHD_ACCESS_LINEAR - see the note at the
                                   end of this test's comment, and
                                   blit_col_amode(). */
                                blit_col_amode(3u), 0u);
        n4 = blit_scan8(GPU_DW_DST + 0x00020000u, GPU_CV_W, GPU_CV_H,
                        &bx, &by, &bv);
        sprintf(Message, "gpu blit B col-shape nz %lu first x%lu y%lu v%lu\r\n",
                (unsigned long)n4, (unsigned long)bx, (unsigned long)by,
                (unsigned long)bv);
        UART_SendString(Message);

        /* The profile, because a total cannot tell the two answers apart: 126 is
           2 rows x 63 AND 3 rows x 42, and "two taps or three" is exactly the
           difference between the COLUMN shape being usable and not. */
        blit_row_prof(GPU_DW_DST + 0x00020000u, GPU_CV_W, GPU_CV_H, 1u, prof);
        sprintf(Message, "gpu blit B rows %lu %lu %lu %lu %lu %lu %lu %lu\r\n",
                (unsigned long)prof[0], (unsigned long)prof[1],
                (unsigned long)prof[2], (unsigned long)prof[3],
                (unsigned long)prof[4], (unsigned long)prof[5],
                (unsigned long)prof[6], (unsigned long)prof[7]);
        UART_SendString(Message);

        /* ---- (B2) the same column pass into a 565 destination ----

           A and B each proved one half of the ROWCOL pair, and the pair still
           wrote an empty plane - so the halves are not independent and the two
           differences between B and pass 2 have to be split.  This one holds
           the source and the kernel exactly as B's and moves only the
           destination FORMAT to the 565 pass 2 really uses.  Zero here with 126
           in B means the format is the fault; non-zero means it is not, and
           (B3) then moves the source instead. */
        cx = cy = cv2 = 0u;
        (void)blit_convolve_new(dev, GPU_CV_SRC0, blit_b2,
                                GPU_CV_W, GPU_CV_H, GPU_CV_W, GPU_CV_H,
                                GPU_CV_W * GPU_CV_H, clut_base,
                                k3, 1u, 3u, s2, b2, 0u, 1u,
                                0.0f, 0.0f, 1u, 1u,
                                GPU_DW_FMT_SIGNED_565, E_DHD_PIXEL_ORG_RGB_565,
                                E_DHD_ACCESS_LINEAR, 0u);
        n5 = blit_count_nz16(blit_b2, GPU_CV_W, GPU_CV_H, 32u, 4u, 64u);
        n6 = blit_first_nz16(blit_b2, GPU_CV_W, GPU_CV_H, &cx, &cy);
        sprintf(Message, "gpu blit B2 col-565 nz %lu first x%lu y%lu v%lu\r\n",
                (unsigned long)n5, (unsigned long)cx, (unsigned long)cy,
                (unsigned long)n6);
        UART_SendString(Message);

        /* ---- (B3) the same column pass over a GPU-WRITTEN 565 plane ----

           Pass 2's exact pairing - the source is tmp, the plane pass 1 wrote -
           with the destination format held at the L_8 where B works.  Zero here
           means "a CLUT kernel reading a plane the GPU wrote" is a wall in the
           COLUMN shape ((A) already read the same plane through the ROW shape
           and found the streak); non-zero means the formats are B2's answer and
           there is nothing else in the pair. */
        d2x = d2y = d2v = 0u;
        (void)blit_convolve_new(dev, blit_tmp, blit_b3,
                                GPU_CV_W, GPU_CV_H, GPU_CV_W, GPU_CV_H,
                                GPU_CV_W * GPU_CV_H, clut_base,
                                k3, 1u, 3u, s2, b2, 0u, 1u,
                                0.0f, 0.0f, 1u, 1u,
                                GPU_DW_FMT_SIGNED_565, E_DHD_PIXEL_ORG_L_8,
                                E_DHD_ACCESS_LINEAR, 0u);
        n7 = blit_scan8(blit_b3, GPU_CV_W, GPU_CV_H, &d2x, &d2y, &d2v);
        sprintf(Message, "gpu blit B3 col-on-tmp nz %lu first x%lu y%lu v%lu\r\n",
                (unsigned long)n7, (unsigned long)d2x, (unsigned long)d2y,
                (unsigned long)d2v);
        UART_SendString(Message);

        /* ---- (T) the transposed mode, and the reading is a SWAP ----

           BLIT_SEP_ROWCOL_T on a square 8x8 tile over the SAME stripe.  Both
           passes' taps walk u, and the transposed UV makes u advance with the
           destination's ROW, so pass 2's taps walk tmp's memory COLUMNS - which
           are the source's y.  That is the vertical sum the COLUMN shape would
           not give, built from the ROW shape that is measured exact.

           The stripe is a whole source ROW (r=4, every column, blue 5), so the
           two outcomes differ in the shape of the result and not in a detail:
           pass 1 sums three texels of it with the taps walking the source's x,
           so tmp's whole COLUMN 4 fills; pass 2 then lands on three output ROWS
           across all eight columns - 24 non-zero in rows 2..4.  If the
           transposed UV were IGNORED the same pair degenerates to the vendor's
           row-then-row - tmp's ROW 4 fills and the output is a single row of
           eight, 8 non-zero.  Same source, same kernel, same tile: the count
           and the axis of the band are the whole reading, and a 3-tap column
           sum and a 1-row one cannot be confused for each other. */
        if (!blit_separable_convolve_new(dev, BLIT_SEP_ROWCOL_T,
                                         GPU_CV_SRC0, blit_t2,
                                         GPU_CV_W, GPU_CV_H, 8u, 8u,
                                         8u, blit_tmp, clut_base,
                                         k3, 3u, 1u, s2, b2, s1, b1,
                                         0u, 1u, 0.0f, 0.0f, 1u, 1u,
                                         GPU_DW_FMT_SIGNED_565,
                                         E_DHD_PIXEL_ORG_L_8,
                                         E_DHD_ACCESS_LINEAR, 0u))
        {
            UART_SendString("gpu blit T FAIL returned\r\n");
            bad |= 128u;
        }
        cx = cy = cv2 = 0u;
        n5 = blit_scan8(blit_t2, 8u, 8u, &cx, &cy, &cv2);
        sprintf(Message, "gpu blit T trans nz %lu first x%lu y%lu v%lu\r\n",
                (unsigned long)n5, (unsigned long)cx, (unsigned long)cy,
                (unsigned long)cv2);
        UART_SendString(Message);
        blit_row_prof(blit_t2, 8u, 8u, 1u, prof);
        sprintf(Message, "gpu blit T rows %lu %lu %lu %lu %lu %lu %lu %lu\r\n",
                (unsigned long)prof[0], (unsigned long)prof[1],
                (unsigned long)prof[2], (unsigned long)prof[3],
                (unsigned long)prof[4], (unsigned long)prof[5],
                (unsigned long)prof[6], (unsigned long)prof[7]);
        UART_SendString(Message);

        /* ---- (T3) a rank-1 3x3's impulse response, which IS the operator ----

           ONE source texel at (32,4), code 31, and an ASYMMETRIC kernel
           {1,2,3}.  A separable pair's response to an impulse is the OUTER
           PRODUCT of its two 1-D responses, so this reads nine numbers and not a
           count, and every way the pair can be wrong is a different block.
           Symmetric {1,2,1} would not have done: its outer product is unchanged
           by reversing the taps, and the tap ORDER is one of the things left to
           pin (a 1x16 all-ones kernel cannot show it).

           The numbers, with the intermediate's law in them.  A code-31 texel is
           a multiplicand of rep(31)/8 = 31.875; pass 1's scale of 16 is a gain
           of 0.5, so its row results are R1 = trunc({1,2,3}*15.9375) =
           {15, 31, 47}.  The 565 intermediate stores them as
           C = trunc(R1*31/255) = {1, 3, 5}, whose multiplicands are
           rep(C)/8 = {1.0, 3.0, 5.125}, and pass 2 multiplies each by 8.2258 -
           the intermediate's factor, which the pass now carries - giving
           {8.2, 24.7, 42.2}.  The block is that outer {1,2,3}:

                8  24  42
               16  49  84
               24  74 126

           which is {15,31,47} x {1,2,3} - k1*k2*15.9375, the ideal filter - with
           the field's 5-bit step in it, so the two are worth reading side by
           side: 15 became 8.2 because the field's quantum is 8.2258.  Read the
           nine against the pattern, not each cell: a REVERSED tap order flips
           both axes; taps that do not walk one texel per tap break the outer
           product; and the vendor's row-then-row pair puts all nine in ONE row.
           nz = 9 is itself a reading.

           The destination is 64x64 and not 8x8 because dw must equal dh, and
           the block's COLUMNS land on the source's x (31..33) - in this mode the
           destination's (row, column) is the source's (y, x). */
        for (i = 0u; i < GPU_CV_H * GPU_CV_W; i++)
        {
            dw_wr(GPU_CV_SRC0, i * 2u, 0u);
            dw_wr(GPU_CV_SRC0, (i * 2u) + 1u, 0u);
        }
        dw_wr(GPU_CV_SRC0, ((4u * GPU_CV_W) + 32u) * 2u, 31u);

        k123[0] = 1; k123[1] = 2; k123[2] = 3;

        {
            const float   s1t[1] = { 16.0f };
            const int32_t b1t[1] = { 0 };
            const float   s2t[1] = { 32.0f };
            const int32_t b2t[1] = { 0 };

            if (!blit_separable_convolve_new(dev, BLIT_SEP_ROWCOL_T,
                                             GPU_CV_SRC0, blit_t4,
                                             GPU_CV_W, GPU_CV_H, 64u, 64u,
                                             64u * 64u,
                                             blit_tmp, clut_base,
                                             k123, 3u, 1u, s2t, b2t, s1t, b1t,
                                             0u, 1u, 0.0f, 0.0f, 1u, 1u,
                                             GPU_DW_FMT_SIGNED_565,
                                             E_DHD_PIXEL_ORG_L_8,
                                             E_DHD_ACCESS_LINEAR, 0u))
            {
                UART_SendString("gpu blit T3 FAIL returned\r\n");
                bad |= 256u;
            }
        }

        t4n = blit_scan8(blit_t4, 64u, 64u, &t4x, &t4y, &t4v);
        sprintf(Message, "gpu blit T3 imp nz %lu x%lu y%lu v%lu\r\n",
                (unsigned long)t4n, (unsigned long)t4x, (unsigned long)t4y,
                (unsigned long)t4v);
        UART_SendString(Message);
        for (i = 0u; i < 3u; i++)
        {
            sprintf(Message, "gpu blit T3 row%lu %lu %lu %lu\r\n",
                    (unsigned long)i,
                    (unsigned long)dw_rd(blit_t4, ((t4y + i) * 64u) + t4x),
                    (unsigned long)dw_rd(blit_t4, ((t4y + i) * 64u) + t4x + 1u),
                    (unsigned long)dw_rd(blit_t4, ((t4y + i) * 64u) + t4x + 2u));
            UART_SendString(Message);
        }
    }

    /* The window count is a reading and the whole plane is the verdict, and the
       two are not the same question.  `want 9` was anchored on the ideal
       placement, which the board does not have: `rowcol tmp ... x31 y5` puts
       pass 1's three cells one texel left and one row down of the impulse, and
       pass 2 moves them again, so a 3x3 window at the source's own coordinate
       can never hold all nine even when the mode works.  The bit means the
       defect that was actually there - pass 2 wrote NOTHING - so it tests the
       whole destination's count, and the shape is read off n2 and the profile. */
    UART_SendString("gpu blit rowcol want nz 4..6 (0 = pass 2 wrote nothing, 3 = no column pass)\r\n");
    sprintf(Message, "gpu blit rowcol nz %lu win %lu\r\n",
            (unsigned long)n2, (unsigned long)plus);
    UART_SendString(Message);

    if (n2 == 0u) { bad |= 64u; }

    cv_env_restore(dev, &env);

    (void)dhd_free_clut(dev, 512u, clut_base);

    /* Same exit gpu_conv_probe() needs and for the same reason: the filter
       scale word is read by the library's blit and written by nothing else, so
       a probe that leaves a per-unit gain in it changes what the product's
       front end does. */
    gpu_sb_reset();

    if (out_bad != NULL) { *out_bad = bad; }
    return true;
}

/* ===================================================================
   The depthwise layer itself.

   Everything above is a rehearsal for this.  The probe settled the four things
   that decide the shape of the code below, and three of them came back
   different from what the approved design assumed:

     1. The kernel's multiplicand is `blue >> 3` - FIVE BITS, 32 levels, in
        every texel format.  RGB_888 does not widen it (8 bits per channel is
        what the BYTE holds, not what the KERNEL takes), and the alpha byte of
        the two 32-bit alpha formats is not read at all: dw_ka8() put 0x7F in
        each of the four bytes of each and read 31,31 where the 565 blue field
        is and 0 everywhere else, alpha included.  So an int8 activation loses
        its low three bits on the way in, and nothing on this silicon prevents
        that.  What is done about the systematic half of it is below.

     2. The bias is added to the sum BEFORE the output clamp.  dw_kbias(): a
        centre tap of -1 over a 5-bit code of 8 gives a sum of -8, and with
        bias 128 that reads 120, not 128.  Without this the operator would not
        be expressible at all, because a depthwise sum spans +-127*31*9.  The
        field is `m_txp_filter_bias_0 : 9` signed, so it lives in [-256, 255] -
        which is a real limit, and gpu_dw_conv() refuses a layer that needs more
        rather than letting a 9-bit field wrap a big number into a small one.

     3. The output is 8 bits wide (nine taps of 31 read 255, not 31), so L_8 is
        a sufficient destination and nothing has to rescale on the way out.

     4. A 3x3 kernel occupies four CLUT words, so a whole 96-channel layer fits
        the 512-word slice and is uploaded once.

   The transfer function, confirmed over 900 pixels against a CPU evaluation
   (gpu dw kdiff bad 0 worst 1 n 900):

       out = clamp( trunc( sum(w * C) * a_scale/32 ) + bias, 0, 255 )

   with C the 5-bit blue field and w the signed 8-bit CLUT weight.

   --- the arithmetic ---

   TFLM computes, per channel,

       acc   = sum( (in_q - in_zp) * w_q ) + bias_q32        int32
       out_q = clamp( M * acc + out_zp, -128, 127 )          int8

   with in_zp = -128 in this model, so (in_q - in_zp) = in_q + 128 is the whole
   unsigned byte.  Call that byte a and its top five bits C, so a = 8C + r with
   r = a & 7.  Then

       sum(w * a) = 8 * sum(w * C) + sum(w * r)

   and r is gone by the time the kernel sees C, so the best available estimate
   of the second term is its mean over a uniform r, which is 3.5 * sum(w).
   Folding that in,

       M * acc = M * ( 8*sum(w*C) + sum(w*r) + bias_q32 )
              ~= 8*M * sum(w*C)  +  M * (3.5*sum(w) + bias_q32)

   which is the GPU's form with

       a_scale = 32 * 8 * M = 256M
       bias    = M * (3.5*sum(w) + bias_q32) + out_zp + 128

   the +128 being the offset an unsigned byte needs to carry an int8, and the
   clamp to [0,255] then matching TFLM's clamp to [-128,127] exactly.  M is
   computed from the tensors' own quantisation parameters and not from TFLM's
   internal OpData, whose layout is a pack-version detail.

   The 3.5 term is the honest part.  It removes the DC component of the
   truncation - r is not zero-mean, it averages 3.5 - and leaves only the +-3.5
   of quantisation noise, which is the irreducible price of a 5-bit
   multiplicand.  It is a mean and not a per-pixel correction, so the layer is
   unbiased, not exact.

   --- what it costs, honestly ---

   A texture has to be a 16-bit RGB565 texel per activation, because the kernel
   reads the blue field and 5 bits of a 565 word is the cheapest way to hand it
   one.  So the CPU writes two bytes per input pixel and reads one byte per
   output pixel, all in the GPU slot's uncached memory, and repacks the source
   per channel because one channel's plane is what one kernel wants.  That is
   real work and it is counted rather than hidden - gpu_dw_stat() reports it.
   Per gpu-offload-goal it is not the gate (whether the operator runs on the GPU
   is), but it is why this is not automatically faster than the CPU.

   The buffers are the probe's own scratch, one plane each: the largest eligible
   layer is 160x160, i.e. 51200 bytes as texels and 25600 as L_8, both far
   inside the 192 KB each buffer has.  So no chunking is needed, and per
   sdram-slow-is-silicon the uncached cost of getting in and out of that memory
   is a known defect that is out of scope.
   =================================================================== */

#define GPU_DW_CLUT_WORDS 512u      /* the whole slice dhd_alloc_clut gives us */
#define GPU_DW_QUAD       4u        /* CLUT words one 3x3 kernel packs into     */
#define GPU_DW_MAXCH      128u      /* == GPU_DW_CLUT_WORDS / GPU_DW_QUAD       */

static uint32_t s_dw_clut_base = 0u;
static bool     s_dw_clut_have = false;

static uint32_t s_dw_kpack[GPU_DW_CLUT_WORDS];
static float    s_dw_asc [GPU_DW_MAXCH];
static int32_t  s_dw_bias[GPU_DW_MAXCH];
static int32_t  s_dw_bias2[GPU_DW_MAXCH];   /* the split's second-pass bias */

static uint32_t s_dw_layers  = 0u;
static uint32_t s_dw_draws   = 0u;
static uint32_t s_dw_refused = 0u;
static uint32_t s_dw_packed  = 0u;   /* input activations that became texels   */
static uint32_t s_dw_unpacked = 0u;  /* output bytes that became activations   */

/* `s_dw_cold_*` (unit 0's shadow, read once and printed once) and
   `s_dw_warm_done` (the once-per-boot gate on the warm-up re-test) used to be
   declared here.  Both belonged to instruments that are gone - see the removal
   note inside gpu_dw_conv() - and a static that nothing reads is a claim that
   something still does. */

void gpu_dw_stat(uint32_t* layers, uint32_t* draws, uint32_t* refused,
                 uint32_t* packed, uint32_t* unpacked)
{
    if (layers   != NULL) { *layers   = s_dw_layers;   }
    if (draws    != NULL) { *draws    = s_dw_draws;    }
    if (refused  != NULL) { *refused  = s_dw_refused;  }
    if (packed   != NULL) { *packed   = s_dw_packed;   }
    if (unpacked != NULL) { *unpacked = s_dw_unpacked; }
}

/* --- the stride-2 UV override: diagnostic, boot only ---------------------

   The stride-2 UV is a composition of two measured facts and one unmeasured
   one, and the composition is what turned out to be wrong: measured
   2026-09-17, the first stride-2 layer came back

       gpu dw chk s2 in n 1296 worst 26 bad 94      <- 7% of the plane
       gpu dw st  ... sz 82x82 p82 dst 107..206     <- the draw DID write

   The two measured facts are (a) the kernel's taps step by TEXEL whatever
   du/dx is - `gpu dw step 1:1 2 2:1 2`, so dux only chooses which texel the
   window CENTRES on - and (b) at stride 1, u0 = 0 with dux = 1/width lands
   destination x on texel x, which is why the whole stride-1 class is correct.
   The unmeasured one is WHICH texel the window centres on when u0 is not 0.

   Neither measurement could see it.  `gpu dw step` reads C(t+1) - C(t-1),
   which is 2 for every centre t, so it cannot report where the centre was.
   And at stride 1 u0 = 0 puts the centre on the destination index under either
   convention, so a one-texel disagreement about u0 is invisible there and
   costs a whole texel per output here.

   So it is swept instead of argued.  gpu_dw_uv_set() replaces the computed UV
   with one given in TEXELS; the caller runs a handful of candidates on the
   first stride-2 layer and prints each result beside the CPU's own answer.
   The candidate that matches is the convention.  None of them matching means
   the fault is not in the UV, and that is worth knowing too.

   It is a boot-only diagnostic and it does not belong in the shipping path:
   gpu_dw_uv_clear() puts the derived UV back. */
static int   s_dw_uv_on = 0;
static float s_dw_uv_t[4];      /* u0, v0, dux, dvy, each in TEXELS */

void gpu_dw_uv_set(float u0_texels, float v0_texels,
                   float dux_texels, float dvy_texels)
{
    s_dw_uv_t[0] = u0_texels;  s_dw_uv_t[1] = v0_texels;
    s_dw_uv_t[2] = dux_texels; s_dw_uv_t[3] = dvy_texels;
    s_dw_uv_on = 1;
}

void gpu_dw_uv_clear(void) { s_dw_uv_on = 0; }

/* --- how much error the 5-bit texel can produce for THIS layer -----------

   The texel's multiplicand is five bits, so a tap reads C = (a + 128) >> 3 and
   the 3.5 of mean truncation lost per tap is folded into the bias (see `bg`
   below).  What is left is zero-mean and layer-dependent:

       residual = m * SUM_i(w_i * (3.5 - r_i))    r_i = (a_i + 128) mod 8

   with every r_i in [0, 7], so |3.5 - r_i| <= 3.5 and the whole thing is
   bounded by

       m * 3.5 * SUM_i|w_i|

   That bound is a property of the layer's weights and scale, NOT of the UV, and
   it is the number that says whether a layer can meet DW_GAP_MAX at all.
   Measured worst for the 13 stride-1 layers is 4, so for those the bound must
   be of that order - which is what makes this worth printing for both classes:
   the check runs on one layer of each, so the two bounds and the two measured
   worsts land in the same log and can be compared.

   If the stride-2 bound comes out near 25, the window was never the problem and
   no UV will fix it - the honest conclusion being that those six layers are
   outside this method's tolerance at their weight scale, which is a finding
   rather than a bug.  If it comes out near 4 instead, the window IS the problem
   and the sweep's counts are the place to read the answer. */
static float s_dw_bound = 0.0f;

float gpu_dw_bound(void) { return s_dw_bound; }

/* Why this and not two byte stores: the scratch is uncached, so each store is
   a separate transaction to the SDRAM controller, and the high byte of a 565
   texel is always zero - the second store would write a constant and cost as
   much as the first. */
static void dw_wr16(uint32_t base, uint32_t i, uint16_t v)
{
    *(volatile uint16_t*)(base + (i * 2u)) = v;
}

/* One channel of NHWC int8 activations -> one plane of 16-bit texels whose blue
   field carries a five-bit code.  `lo` selects which five bits:

     lo 0:  C_hi = (a + 128) >> 3      the whole value, truncated - the shipping
                                       single-pass path, unchanged
     lo 1:  C_lo = (a + 128) &  7      the three bits that were thrown away

   `a + 128` is the input's unsigned code (this model's in_zp is -128).  Stride
   c is the NHWC channel stride; the row arithmetic is the layout conversion the
   CPU pays for and gpu_dw_stat() reports as packed/unpacked. */
static void dw_pack_plane(const int8_t* in, uint32_t uw, uint32_t uh,
                          uint32_t c, uint32_t ci, int lo)
{
    uint32_t x, y;

    for (y = 0u; y < uh; y++)
    {
        uint32_t row = (y * uw * c) + ci;

        for (x = 0u; x < uw; x++)
        {
            uint8_t a = (uint8_t)((int32_t)in[row] + 128);

            dw_wr16(GPU_DW_SRC, (y * uw) + x,
                    (uint16_t)(lo ? (a & 7u) : (a >> 3)));
            row += c;
        }
    }
}

/* Returns 0 if the layer ran on the GPU, -1 if it was refused and the caller
   must use the CPU.  A refusal is never a partial result: every channel is
   checked before the first draw, so `out` is either wholly written or wholly
   untouched.  See gpu_dw_conv()'s contract in gpu.h. */
int gpu_dw_conv(const int8_t* in, const int8_t* wq, const int32_t* biasq,
                const float* in_scale, const float* w_scale, float out_scale,
                int32_t out_zp, int h, int w, int c, int stride,
                int oh, int ow, int split, int8_t* out)
{
    dhd_handle_t    dev = (dhd_handle_t)a_handle;
    dhd_uv_data_t   uv;
    dhd_pixel_org_t saved_fmt;
    uint32_t        saved_fb;
    uint32_t        sav_w, sav_h, sav_p;
    uint32_t        sav_x0, sav_y0, sav_x1, sav_y1, sav_ys;
    uint32_t        ci, x, y, i;
    uint32_t        uw, uh, uow, uoh, ucl;
    int8_t          kern[9];
    int8_t          k0[9];

    /* Set when a draw's tail drain gave up, which is a REFUSAL and not a
       warning - see the two checks on dw_pass_bc() below. */
    bool            refused = false;

    if (!s_ready)                                       { return -1; }

    /* A drain gave up earlier in this boot: the GPU may still be holding the
       scratch planes, and handing them to another layer is how a refusal turns
       into a wrong answer.  Every layer after that point goes to the stock
       kernel, which is the rule that the GPU cannot take the product with it.
       Checked before anything else, because nothing here is safe to do. */
    if (s_dw_gpu_dead)                                  { s_dw_refused++; return -1; }

    if ((in == NULL) || (wq == NULL) || (out == NULL))  { return -1; }
    if ((stride != 1) && (stride != 2))                 { return -1; }
    if ((h <= 0) || (w <= 0) || (c <= 0))               { return -1; }
    if ((oh <= 0) || (ow <= 0))                         { return -1; }
    if ((uint32_t)c > GPU_DW_MAXCH)                     { return -1; }

    /* Reset with the layer, so gpu_dw_bound() reports THIS call's layer and not
       the largest one seen since boot. */
    s_dw_bound = 0.0f;

    uw = (uint32_t)w;
    uh = (uint32_t)h;

    /* The output geometry, CHECKED rather than derived.  A wrong output size
       is not a refusal the caller can absorb - it is a wrong write into its
       tensor - so the two accepted shapes are the only two, and anything else
       leaves `out` untouched.

         stride 1:  SAME padding with a 3x3 kernel preserves the size.
         stride 2:  3x3 VALID, so oh = (h-3)/2+1.  This is the shape of the
                    model's six stride-2 depthwise layers, whose input a
                    preceding PAD op has already widened (82x82 -> 40x40), so
                    every tap of every output pixel is inside the plane. */
    if (stride == 1)
    {
        if ((ow != w) || (oh != h))                     { return -1; }
    }
    else
    {
        if ((ow != (((w - 3) / 2) + 1)) ||
            (oh != (((h - 3) / 2) + 1)))                { return -1; }
    }

    uow = (uint32_t)ow;
    uoh = (uint32_t)oh;

    /* One texel per activation at two bytes, and one byte per output.  The
       destination is the SMALLER of the two at either stride - 1:1 at stride 1,
       and (ow*oh) <= (w*h), the ratio being ~1/4, at stride 2 - but both are
       tested, because "smaller" is a property of the geometries accepted above
       and not of the scratch. */
    if ((uw * uh * 2u) > GPU_DW_MAX_PLANE)              { return -1; }
    if ((uow * uoh)    > GPU_DW_MAX_PLANE)              { return -1; }

    if (biasq == NULL) { return -1; }

    if (!s_dw_clut_have)
    {
        if (dhd_alloc_clut(dev, GPU_DW_CLUT_WORDS, &s_dw_clut_base) != 0u)
        {
            return -1;
        }
        s_dw_clut_have = true;
    }

    g_gpu_dw_prog = DW_PROG(1u, 0u, 0u);

    /* --- pass 1: pack the weights and check every channel fits ---
       All C channels are checked before the first draw.  A refusal on channel
       60 of 96 would otherwise leave a tensor that is 60 channels of GPU result
       and 36 of whatever was in `out` before, which is worse than not trying. */
    for (ci = 0u; ci < (uint32_t)c; ci++)
    {
        float   m, asc, sumw;
        int32_t bg, bg2;

        /* TFLM's depthwise weights are [1, kh, kw, C], so tap i of channel ci
           is at (i * C) + ci - not a contiguous run, which is why this gathers
           rather than memcpys. */
        for (i = 0u; i < 9u; i++)
        {
            kern[i] = wq[(i * (uint32_t)c) + ci];
        }
        dw_pack_quad(&s_dw_kpack[GPU_DW_QUAD * ci], kern, 3u, 3u);

        sumw = 0.0f;
        for (i = 0u; i < 9u; i++) { sumw += (float)kern[i]; }

        m   = (in_scale[ci] * w_scale[ci]) / out_scale;
        asc = 256.0f * m;

        /* The layer's own 5-bit residual bound - see gpu_dw_bound().  Taken
           over every channel rather than channel 0 alone, because the check
           that reads it compares one channel against a threshold and a bound
           that understates a channel would be worse than no bound. */
        {
            float sa = 0.0f;
            for (i = 0u; i < 9u; i++)
            {
                float w = (float)kern[i];
                sa += (w < 0.0f) ? -w : w;
            }
            if ((3.5f * m * sa) > s_dw_bound) { s_dw_bound = 3.5f * m * sa; }
        }

        bg  = (int32_t)((m * ((3.5f * sumw) + (float)biasq[ci])) +
                        (float)out_zp + 128.0f);

        /* The hardware's bias field is nine bits signed
           (davehd_registermap_custom.h:1075), so this is the one place the
           arithmetic can run out of room.  Refuse the layer rather than let a
           9-bit field wrap a large number into a wrong small one - a silently
           wrong convolution is the worst of the available outcomes. */
        if ((bg < -256) || (bg > 255))
        {
            s_dw_refused++;
            return -1;
        }
        /* The scale field is a 10-bit significand and a 6-bit signed exponent
           (davehd_driver_txp_inl.h:658-669), so it is positive and finite or
           it is nothing. */
        if (!(asc > 0.0f) || (asc > 4.0e12f))
        {
            s_dw_refused++;
            return -1;
        }

        /* The split's second bias.  `bg` above carries `m * 3.5 * sumw` because
           the five-bit texel is `(a+128)>>3` and the 3.5 the truncation drops
           per tap is a constant that belongs in the bias.  The second pass
           re-adds exactly those three bits, so it has to give the 3.5 back -
           and it must do so as -trunc(m*3.5*sumw) rather than by leaving the
           term out, because leaving it out would put the correction's own
           centre at 0 and the byte that carries it would clip against the
           destination format's floor.

           It is computed for every channel whether or not this call splits, so
           the same arithmetic gets the same bounds check.  The 9-bit field is
           not at risk here in practice - |3.5*m*sumw| <= 3.5*m*sum|w|, which is
           the bound gpu_dw_bound() reports and is 86 for the worst stride-2
           channel - but "in practice" is what the check above refused to rely
           on, and neither will this one. */
        bg2 = 128 - (int32_t)(m * 3.5f * sumw);

        if ((bg2 < -256) || (bg2 > 255))
        {
            s_dw_refused++;
            return -1;
        }

        s_dw_asc[ci]   = asc;
        s_dw_bias[ci]  = bg;
        s_dw_bias2[ci] = bg2;

        if (ci == 0u)
        {
            for (i = 0u; i < 9u; i++) { k0[i] = kern[i]; }
        }
    }

    /* --- the texture unit, once for the whole layer ---

       ORDER MATTERS, and it is the order dw_pass() documents at length above:
       dhd_active_texture_unit() is what sets device->m_tex_unit_base, and every
       setter below computes its register address as "that base + an offset".
       A kernel call made BEFORE the unit is selected lands on whatever unit the
       last nsgpu_* call left active - which is the bug that cost three flash
       rounds to find the first time, and it fails silently as a bias-only
       result rather than as an error.  So: select the unit, then everything
       else, and note that this is why the whole packing loop above is pure CPU
       work with no dhd_* call in it. */
    uv.m_duy = 0.0f;  uv.m_dvx = 0.0f;

    if (stride == 1)
    {
        /* Destination column x lands on texel x exactly - measured, and there
           is no half-texel offset (`gpu dw kst 565 0 8 16 24`). */
        uv.m_u0 = 0.0f;  uv.m_v0 = 0.0f;
        uv.m_dux = 1.0f / (float)uw;
        uv.m_dvy = 1.0f / (float)uh;
    }
    else
    {
        /* Stride 2, and this is the whole of it - the plan's "a parameter
           question" verdict, measured.

           `gpu dw step 1:1 2 2:1 2` says the CLUT kernel's tap offsets are in
           TEXELS and do NOT scale with du/dx: at dux = 1/4 the outer two taps
           are still two texels apart.  So dux alone cannot widen the window;
           all it does is choose which texel the window CENTRES on.

           That is enough, because output j needs inputs 2j, 2j+1, 2j+2 and the
           window offers centre+-1.  Centre on 2j+1 and the taps land on 2j and
           2j+2 - exactly right.  So u0 puts the first centre on texel 1 and dux
           advances two texels per output pixel.

           Vertical is the same, and the 3x3 VALID geometry guarantees no tap is
           ever outside the plane, so the border colour cannot matter here. */
        uv.m_u0 = 1.0f / (float)uw;  uv.m_v0 = 1.0f / (float)uh;
        uv.m_dux = 2.0f / (float)uw;
        uv.m_dvy = 2.0f / (float)uh;
    }

    /* Diagnostic override, applied last so it replaces either branch above.
       See gpu_dw_uv_set() for why the composition above is in doubt and what
       the sweep does about it.  Never on in the shipping path. */
    if (s_dw_uv_on)
    {
        uv.m_u0  = s_dw_uv_t[0] / (float)uw;
        uv.m_v0  = s_dw_uv_t[1] / (float)uh;
        uv.m_dux = s_dw_uv_t[2] / (float)uw;
        uv.m_dvy = s_dw_uv_t[3] / (float)uh;
    }

    g_gpu_dw_prog = DW_PROG(2u, 0u, 0u);

    dhd_set_uv(dev, 0u, &uv);
    dhd_active_texture_unit(dev, 0u);

    /* The cold snapshot, the warm-up re-test and the 0x5A pre-fill that the
       three of them shared have all been REMOVED, and the reasons are the
       readings they produced:

         - the pre-fill separated "the draw wrote something wrong" from "the
           draw did not write at all"; the note where it used to run, further
           down this function, says what closed that question and why the answer
           is the operator's own check rather than a better reading.
         - the warm-up re-test existed to test "a preceding dw_pass() is what
           makes this layer right", a belief that `dst 90..90` had already
           withdrawn (a destination nothing wrote to is indistinguishable from
           one holding a plausible number).  Its reading is `gpu dw warm`, which
           is gone with it; gpu_dw_probe() still exercises the same form.
         - the cold snapshot was one half of a pair, and the other half was the
           post-draw `gpu dw st` line, also gone.

       What is NOT gone, because it is not scaffolding: the whole of the draw
       below, and the note on the inline copy's no-write that is still attached
       to it.  That question remains open and unanswerable now, which is exactly
       why the comment stays.

       If a future regression makes the draw's behaviour doubtful again, the
       instrument to rebuild is a pre-fill plus a two-line readback - not the
       `s_dw_cold_*` statics, which measured the DRIVER's shadow rather than the
       hardware's result and are the weaker of the two. */

    /* If the warm-up re-test is ever rebuilt, it goes HERE - before the
       dhd_set_clut() below.  It called dw_pass(), which uploads its own kernel
       to the base it is handed, and that upload must not land after the layer's
       real kernels are in place. */
    dhd_set_texture_unit_enable(dev, 1u);
    dhd_set_texture_address(dev, (dhd_gpu_ptr_t)GPU_DW_SRC, 0u);
    dhd_set_texture_size(dev, uw, uh, uw);
    dhd_set_texture_format(dev, GPU_DW_FMT_SIGNED_565, E_DHD_ACCESS_LINEAR, 0u);

    /* --- one upload for the whole layer ---
       dhd_set_clut() opens with DHD_STCOP_WAIT, draining the pipeline, so it is
       the expensive call in this function and it happens once.  Every draw
       below then only re-points m_txp_clut_offset_0, which is the entire reason
       a 512-word slice is worth holding across a layer rather than allocating
       and freeing per draw the way the nsgpu_* wrappers do. */
    (void)dhd_set_clut(dev, s_dw_kpack, s_dw_clut_base,
                       (uint32_t)c * GPU_DW_QUAD);

    g_gpu_dw_prog = DW_PROG(3u, 0u, 0u);

    /* Sits AFTER dhd_set_clut() above.  That placement is NOT what decides
       whether the layer is right, and an earlier version of this comment said
       it was.

       Measured 2026-09-17, three runs, same binary each time except for these
       lines:

         kernel before clut, no dw_pass() first  -> b0 127   (the defect)
         kernel before clut, with a dw_pass()    -> b0 157   (right)
         kernel after  clut, no dw_pass()        -> b0 127   (defect, unchanged)

       Moving the call changed nothing observable.  The preceding dw_pass()
       call is the only variable that has ever flipped 127 to 157, so the
       ordering claim is withdrawn.  It also means the "one variable at a time"
       discipline was not actually honoured here: the reorder and the removal of
       the warm-up went in together, which is precisely why the two answers
       could not be told apart from that run.

       The placement is left as-is only because it is now measured irrelevant -
       it is not a live variable.  Whether a preceding dw_pass() establishes
       anything at all was the warm-up re-test's question, and that instrument
       has been removed along with the pre-fill (see the removal note above the
       dhd_set_texture_unit_enable below).  It was removed because `b0 157` is
       not evidence for a warm-up mattering: that reading predates the pre-fill,
       so it cannot distinguish a warm-up that worked from a warm-up whose
       output was simply still in the buffer.  Do not write a second explanation
       here without a reading behind it. */
    dhd_set_texture_kernel(dev, 3u, 3u, (const uint8_t*)k0, s_dw_clut_base);
    dhd_set_texture_clut_base(dev, s_dw_clut_base);
    dhd_set_texture_scalebias(dev, s_dw_asc[0], s_dw_bias[0]);
    dhd_set_texture_wrap(dev, E_DHD_TEX_BORDER, E_DHD_TEX_BORDER);
    /* TFLM's SAME padding is padding with the quantised zero point: a tap
       outside the tensor reads in_q = in_zp, so (in_q - in_zp) = 0 and the
       texel's blue field is 0.  Border colour zero is that, exactly.  This
       call states the layer's intent and is now the same value the per-channel
       dw_pass_bc() below writes before each draw - that one is what decides,
       and this one is here so the intent is readable at the top of the layer.
       Until 2026-09-17 the per-channel call was dw_pass(), whose 0x80808080
       silently replaced this 0 with a padded tap of C = 16. */
    dhd_set_texture_border_color(dev, 0u);
    dhd_set_color_operation(dev, E_DHD_OP_COPY);

    g_gpu_dw_prog = DW_PROG(4u, 0u, 0u);

    /* The driver's framebuffer format is global and nsgpu_init() left it at
       RGB565.  L_8 is the destination this operator needs, it is not
       expressible any other way, and leaving it changed would break every
       later nsgpu_* call - so it is saved, changed once, and put back.

       The SIZE and the BBOX are saved and put back too, and that is the fix for
       the round in which the GPU front end stopped detecting anything.  This is
       the operator's OWN path, it runs on 19 layers of every inference, and it
       sets the framebuffer to the layer's plane size (uw x uh) with a matching
       box.  Until 2026-09-18 it restored neither, so the driver was left at the
       last depthwise layer's plane geometry every single time inference ran -
       and the very next thing to draw was the production blit.  The boot log
       that pinned it: the agreement probe is `ok 1` (`mean 3`) before the
       depthwise probe and `ok 0` (`mean 106`) after it, and `fb dw 4006 64x64
       p64` is the only ambient reading that moves in the whole block. */
    saved_fb  = nsgpu_get_framebuffer();
    saved_fmt = dhd_get_framebuffer_format(dev);
    dhd_get_framebuffer_size(dev, &sav_w, &sav_h, &sav_p);
    dhd_get_bbox(dev, &sav_x0, &sav_y0, &sav_x1, &sav_y1, &sav_ys);
    dhd_set_framebuffer_format(dev, E_DHD_PIXEL_ORG_L_8);
    dhd_set_framebuffer_size(dev, uw, uh, uw);

    g_gpu_dw_prog = DW_PROG(5u, 0u, 0u);

    ucl = GPU_DW_QUAD;

    /* No pre-fill.  One used to run here - 0x5A over the output plane before the
       first draw - to separate "the draw wrote something wrong" from "the draw
       did not write at all", a distinction the readings could not make for four
       flash rounds because a destination nothing wrote to looks exactly like one
       holding a plausible number.  What closed it was not a better reading of the
       pre-fill but the operator's own check: it compared a 40x40 plane of channel
       0 against the CPU and found no difference, and it now compares the whole
       tensor across every channel and still finds worst 4.  Unwritten pixels
       would have to be identical to the CPU's answer in every one of them, and
       the pre-fill itself was the cheaper half of that doubt, paid on every layer
       of every inference. */

    for (ci = 0u; ci < (uint32_t)c; ci++)
    {
        /* Pack this channel's plane: NHWC int8 in, one 16-bit 565 texel per
           activation out, the 5-bit code in the blue field. */
        g_gpu_dw_prog = DW_PROG(10u, ci, 0u);
        dw_pack_plane(in, uw, uh, (uint32_t)c, ci, 0);
        g_gpu_dw_prog = DW_PROG(11u, ci, 0u);

        /* ONE implementation of the draw sequence, and it is the measured one.

           The eight inline calls that used to be here are what dw_pass() does,
           and dw_pass() is the only form of the sequence ever OBSERVED to write
           to the destination at all.  Measured 2026-09-17, both in the same
           session, both against a 0x5A pre-fill (the prints that carried the
           two readings - `gpu dw warm`, `gpu dw op` - have since been removed
           with the rest of the scaffolding; the readings stand):

             dw_pass()    -> `gpu dw warm b0 115 b1 77`   per-pixel values, a
                             real convolution, so the buffer was written
             inline       -> `gpu dw op ... b0 90 b1 90` and `dst 90..90`
                             unchanged, so the draw wrote nothing

           Thirteen flash rounds went into the inline copy.  Two divergent
           implementations of one sequence is what produced them, so there is
           now one, and it is the one with a reading behind it.

           `kern` is re-gathered here because dw_pass() takes the taps and packs
           them into the CLUT base it is handed - this channel's slot, so its
           upload is the same four words the one big dhd_set_clut() above
           already put there.  That is a redundant upload plus its pipeline
           drain, per channel, and it is a real cost.  It is paid deliberately:
           the alternative is the second copy again.  Measure the cost, do not
           assume it is free.

           What the inline form did that stopped it writing has NOT been
           identified, and the first draft of this comment claimed there was
           nothing to identify.  Measured by reading the two builds' `gpu dw st`
           lines against each other, the one field that differs is the border
           colour - `bc 80808080` from dw_pass(), `bc 00000000` when the inline
           form ran - and the border colour is not merely cosmetic: it IS the
           value in every padded texel, so it is the whole difference between
           this operator's SAME padding and TFLM's.  dw_pass()'s 0x80808080 is
           the probes' constant and it is wrong here (`a_border` below is 0u,
           with the reason).  That explains the ring, and it cannot explain an
           interior that was never touched, so the inline form's no-write is
           still one of the differences that remain, unseparated: the
           per-channel dhd_set_texture_kernel() (the only call that sets
           m_txa_filter_kernel_width/height/sample_shape and m_tex_filter_u/v),
           the per-channel dhd_set_framebuffer_size(), the per-channel
           dhd_set_uv() and unit enable, or the interleaving of the setup with
           the pre-fill and packing loops.  Do not claim the two forms are
           equivalent again without a reading that tells those apart. */
        for (i = 0u; i < 9u; i++) { kern[i] = wq[(i * (uint32_t)c) + ci]; }

        /* Border colour 0u, not dw_pass()'s 0x80808080.  The kernel's
           multiplicand is the texel's five-bit blue field, and TFLM pads SAME
           with the input zero point, so a tap outside the tensor must read
           C = 0.  dhd_set_texture_border_color()'s own doc says its argument is
           ARGB8888, so 0x80808080 is four 0x80 channels, whose 5-bit blue after
           conversion is 16 - a padded tap contributing 16*w instead of 0.  The
           inline form used 0 and that part of it was right; the dhd_set_
           texture_border_color() call in the layer setup above set 0 for the
           same stated reason, and dw_pass() then overwrote it per draw. */
        s_dw_prog_ch  = ci;
        g_gpu_dw_prog = DW_PROG(12u, ci, 0u);
        /* CHECKED, and that is the whole of this change.  dw_pass_bc()'s own
           comment says its false return is "refused, not fatal: the caller
           falls back to the stock kernel for this layer, which is the whole
           point of the drain being bounded" - and until now BOTH callers here
           discarded it with a `(void)`.  So a layer whose tail drain gave up
           was reported as `drew`, counted in L and in D, and its output plane
           was then READ BY THE CPU at the fold below - a plane the GPU may not
           have finished writing, because that is precisely what a drain
           timeout means.

           It is the reason the board's numbers could not be reconciled with
           anything: `gpu dw L9 drew` after three `drain dead`s, an inference
           that was 0.48 s FASTER than the all-CPU control, and boxes with zero
           height and negative coordinates.  An offline in-series mirror of this
           operator's arithmetic (tools/chain_emulate.py) reproduces the head at
           all 19 layers and still decodes a plate in the right place with sane
           geometry, so the arithmetic is not what produces those boxes.

           Refusing costs this layer one stock kernel.  Not refusing costs the
           product a wrong answer, which is the trade the hard rule forbids. */
        /* dw_pass_bc_retry() and not dw_pass_bc(): a give-up here would latch
           the offload away for the rest of the boot, and the redraw wrapped
           around this call is the one thing that can still tell "the queue
           stopped" from "the job never landed".  See the wrapper's block. */
        if (!dw_draw_checked(dev, GPU_DW_SRC, GPU_DW_DST, uw, uh, uow, uoh,
                             s_dw_clut_base + (ucl * ci), kern, 3u, 3u,
                             GPU_DW_FMT_SIGNED_565, E_DHD_PIXEL_ORG_L_8,
                             E_DHD_ACCESS_LINEAR,
                             s_dw_asc[ci], s_dw_bias[ci],
                             uv.m_u0, uv.m_v0, uv.m_dux, uv.m_dvy,
                             0u, ci, 1u))
        {
            refused = true;
            break;
        }
        g_gpu_dw_prog = DW_PROG(13u, ci, 0u);

        s_dw_draws++;
        s_dw_packed += uw * uh;

        /* No prints here.  `gpu dw op` (this layer's asc and bias against the
           first two destination bytes), `gpu dw st` (the driver's shadow plus a
           readback of the whole destination plane) and `gpu dw st0` (that same
           shadow as it stood before the layer) all lived here, once per boot,
           and every one of them was built for the inline copy of the draw
           sequence that no longer exists.  `gpu dw st` existed to tell "the
           filter state never reached the hardware" from "the arithmetic ran on
           different operands" - and the answer is neither: the operand was the
           AL_8 texel, which `dhd-gpu-capability` records.

           What replaced them is the operator's own check, which is strictly
           more informative because it compares against the CPU: it reports a
           per-layer worst difference over the whole tensor and it turns the
           feature off when that difference is too large.  A shadow readback
           cannot do either. */

        /* Back to NHWC int8: the byte the GPU wrote is out_q + 128.  The output
           tensor is oh*ow, which at stride 2 is a quarter of the input plane -
           the destination plane, not the source, is what this walks. */
        for (y = 0u; y < uoh; y++)
        {
            uint32_t row = (y * uow * (uint32_t)c) + ci;

            for (x = 0u; x < uow; x++)
            {
                g_gpu_dw_prog = DW_PROG(14u, ci, (y * uow) + x);
                out[row] = (int8_t)((int32_t)dw_rd(GPU_DW_DST,
                                                   (y * uow) + x) - 128);
                row += (uint32_t)c;
            }
        }

        s_dw_unpacked += uow * uoh;

        /* --- the split's second pass: the three bits pass 1 threw away ---

           WHY.  The texel's multiplicand is five bits, C = (a+128)>>3, so every
           tap loses `0.5 + r` where r = ((a+128) mod 8) is uniform-ish on
           [0,7].  The bias absorbs the mean 3.5 exactly; what survives is
           `m * sum(w_i * (3.5 - r_i))`, zero-mean, bounded by the
           gpu_dw_bound() this layer just reported.  At stride 1 that bound is 9
           against a measured worst of 4, and DW_GAP_MAX is 8.  At stride 2 it is
           86 against a measured worst of 25 - and the eight UV candidates that
           would have to be wrong for the window to be the cause were all
           measured worse than the derived one, so the window is right and this
           residual is the whole of the error.  The threshold was calibrated on
           the 3x3 stride-1 layers, whose weights are an order of magnitude
           smaller; it does not transfer.

           HOW.  With a + 128 = 8*C_hi + C_lo, the exact answer is
           m*(sum(w*C_hi)*8 + sum(w*C_lo)) + m*bias + out_zp + 128, and the two
           passes are:

             pass 1  texel C_hi, scale 256m, bias bg   = trunc(m*(3.5*sumw +
                     biasq)) + out_zp + 128      <- UNCHANGED from the
                     single-pass path, which is why this is the safe form: pass 1
                     is byte-identical to a draw already measured to write and
                     already measured to be in range, so it cannot clamp, and it
                     remains a usable answer on its own
             pass 2  texel C_lo, scale 256m/8, bias bg2 = 128 - trunc(m*3.5*sumw)

           pass 1 + pass 2 - 128 telescopes the two 3.5 terms away and the
           remaining truncation error is < 1 LSB in each pass.  The -128 is
           because pass 2's byte is a CORRECTION CENTRED ON 128, not a value:
           centring it is what keeps it inside [0,255], and the CPU fold below
           is `out + byte2` with the final clamp in int8.  A correction centred
           on 0 would need 8 bits of signed range in a format that has none.

           WHAT IT DOES NOT FIX.  Pass 1 clamps at the format's ends, so where
           the true answer is within `bound` of 0 or 255 the correction is added
           to a saturated byte and the sum is wrong by the clamp.  That is
           strictly no worse than the single-pass result there, and it is why
           the escalation below is a MEASUREMENT (run the check again with the
           split and see) rather than a prediction from the bound.

           COST.  Two packings and two draws per channel instead of one, for the
           layer's C channels, on the classes that need it.  The packing is the
           expensive half - it is CPU writes into uncached Device memory - and
           gpu_dw_stat()'s packed counter reports it honestly rather than
           counting one pass and doing two. */
        if (split)
        {
            g_gpu_dw_prog = DW_PROG(15u, ci, 0u);
            dw_pack_plane(in, uw, uh, (uint32_t)c, ci, 1);
            g_gpu_dw_prog = DW_PROG(16u, ci, 0u);

            s_dw_prog_ch  = ci;
            g_gpu_dw_prog = DW_PROG(17u, ci, 0u);
            /* Checked for the same reason as pass 1 above: a timeout here means
               the correction plane was not finished being written, and the fold
               immediately below reads it.  Out[row] still holds pass 1's byte
               unfolded, which is exactly a plane this function is refusing, so
               the whole layer goes back to the stock kernel.

               And through the wrapper for the same reason as pass 1 - this is
               not a copy of that decision, it is the site that MADE it.  The
               boot that proved the point (Code=200638) had both full frames of
               pass 1 and then wedged HERE, in pass 2 of model #16 - `s37 c0 p0`
               - where an inline redraw at pass 1 could never run.  A retry that
               covers one of the operator's two draws is a coin toss. */
            if (!dw_draw_checked(dev, GPU_DW_SRC, GPU_DW_DST, uw, uh, uow, uoh,
                                 s_dw_clut_base + (ucl * ci), kern, 3u, 3u,
                                 GPU_DW_FMT_SIGNED_565, E_DHD_PIXEL_ORG_L_8,
                                 E_DHD_ACCESS_LINEAR,
                                 s_dw_asc[ci] / 8.0f, s_dw_bias2[ci],
                                 uv.m_u0, uv.m_v0, uv.m_dux, uv.m_dvy,
                                 0u, ci, 2u))
            {
                refused = true;
                break;
            }
            g_gpu_dw_prog = DW_PROG(18u, ci, 0u);
            s_dw_draws++;
            s_dw_packed += uw * uh;

            /* Fold, with the clamp where it belongs.  `out[row]` is pass 1's
               byte minus 128, so `out[row] + byte2` is (pass 1) + (pass 2) - 128
               in bytes: the byte arithmetic and the activation arithmetic differ
               by exactly the 128 that both passes carry, and this is that
               cancelled. */
            for (y = 0u; y < uoh; y++)
            {
                uint32_t row = (y * uow * (uint32_t)c) + ci;

                for (x = 0u; x < uow; x++)
                {
                    int32_t f;

                    g_gpu_dw_prog = DW_PROG(19u, ci, (y * uow) + x);
                    f = (int32_t)out[row] +
                        (int32_t)dw_rd(GPU_DW_DST, (y * uow) + x);

                    if (f < 0)   { f = 0; }
                    if (f > 255) { f = 255; }
                    out[row] = (int8_t)(f - 128);
                    row += (uint32_t)c;
                }
            }

            s_dw_unpacked += uow * uoh;
        }
    }

    /* Out of the draw, and the reporter has to be told so: a stale non-zero
       value left here would look exactly like a stall on the next layer's first
       second of CPU work, and that false report would be read as a real one. */
    g_gpu_dw_prog = 0u;

    /* A draw's tail drain gave up part way through this layer.  Put the global
       framebuffer state back - dw_pass_bc() already did, on its own way out,
       and doing it again is free - restore the scalebias the front end reads,
       and refuse.  The tail's gpu_drain() below is deliberately NOT run: the
       latch is already set (the timeout that caused this printed `drain dead`
       and set it), and a second bounded wait here would spend the budget again
       to learn what is already known.

       Nothing is counted.  s_dw_draws and s_dw_layers describe the set the
       caller will report as `drew`, and this layer is not in it - which keeps
       D = SUM(2*Cout) over the drawn set exactly as verified on the board. */
    if (refused)
    {
        dhd_set_framebuffer_format(dev, saved_fmt);
        dhd_set_framebuffer_size(dev, sav_w, sav_h, sav_p);
        dhd_set_bbox(dev, sav_x0, sav_y0, sav_x1, sav_y1, sav_ys);
        (void)dhd_set_framebuffer_address(dev, (dhd_gpu_ptr_t)saved_fb);
        gpu_sb_reset();
        /* Counted like every other refusal in this function, because that is
           what R is: layers handed to the stock kernel.  The board's `R` is
           read against `L` and `D`, and a refusal that did not appear in R
           would read as a layer that vanished. */
        s_dw_refused++;
        return -1;
    }

    dhd_set_framebuffer_format(dev, saved_fmt);
    dhd_set_framebuffer_size(dev, sav_w, sav_h, sav_p);
    dhd_set_bbox(dev, sav_x0, sav_y0, sav_x1, sav_y1, sav_ys);
    (void)dhd_set_framebuffer_address(dev, (dhd_gpu_ptr_t)saved_fb);

    /* The same call and the same wait as dw_pass_bc()'s tail - see gpu_drain()
       for why the library's own draining wait had to be replaced.  Every draw
       of this layer is already committed at this point, so the drain here is
       what says the GPU is done with the scratch planes. */
    if (!gpu_drain(dev))
    {
        /* The scalebias restore below still has to run: the draws already
           happened and already set it, and the front end reads it on the very
           next frame.  Then refuse the layer - a wrong answer is worse than a
           slow one. */
        gpu_sb_reset();
        return -1;
    }

    /* And the half of the state the framebuffer restore above does not cover:
       every draw in this function set dhd_set_texture_scalebias() for its own
       kernel and put nothing back.  The library's blit - the product's entire
       front end - runs with the bilinear filter on and never touches this word,
       so it inherits the last layer's gain.  This operator runs on 19 layers of
       every inference, so without this the FIRST inference leaves the front end
       reading a scaled picture for every frame after it. */
    gpu_sb_reset();

    s_dw_layers++;
    return 0;
}

/* ===================================================================
   The pointwise convolution, on the GPU

   gpu.h carries the contract and the arithmetic.  What is here is the call
   sequence, and it is deliberately the one gpu_conv_probe() MEASURED rather
   than a new one.  The probe's cv_run() establishes, per draw, two texture
   units with two kernels, two CLUT offsets and two gains, summed by
   E_DHD_OP_ADD - and `gpu cv 2u copy 16 add 130`, `2u clut 23`, `2u scale 73`
   are its readings.  So cv_pass() below is cv_run() with the geometry made a
   parameter and the tail fixed, and the fixing is the whole of the difference:

     - THE FRAMEBUFFER SIZE AND BBOX ARE SAVED AND RESTORED.  cv_run() restored
       only the format and the address, which is the defect that stopped the
       product detecting for two flash rounds (the long note in gpu_dw_conv()
       has the boot log).  This operator sets all four to its own tile on every
       layer, so all four are put back.
     - THE TAIL DRAINS WITH gpu_drain(), not with nsgpu_set_framebuffer(), whose
       library body is dhd_finish() - the unbounded wait gpu_drain() exists to
       replace.  Same statement, same wait, bounded.

   AND ONE THING THAT IS DELIBERATELY NOT RESTORED: the UV.  Every draw sets
   its own, and the depthwise operator has been leaving its own behind on 19
   layers of every inference without the front end noticing, which is evidence
   that whatever draws next programs the UV it needs.  Restoring a UV this
   function never read would be inventing a value, so it is left as found -
   stated here because it is an assumption and not an oversight.

   THE SOURCE LAYOUT, and the two texels of margin it needs.  A pixel's Cin
   activations go along the texture's x axis, G pixels per row, so one
   destination column has to step a whole PIXEL - cin texels - and that is what
   dux = cin/RW is for.  The row is RW = G*cin + 2 texels: texel 0 is a guard,
   because the kernel's tap 0 sits at (centre - 1) - measured, `gpu cv step o0
   0 o1 0 o2 1` - so a window aimed at the row's first channel would otherwise
   read one texel BEFORE the row, and the probe could not tell that from a
   clamp: its ramp reads code 0 at texel 0, so "clamped to texel 0" and "read
   the border" are the same number there.  Here they are not, so the guard
   exists.  RW WAS ROUNDED UP TO A 64-TEXEL (128-BYTE) UNIT for one build, on
   the theory that a row pitch has to be a multiple of the bus width; the next
   log falsified that (the hi draw writes through the unrounded pitch) and it is
   reverted - see gpu_conv()'s note.  RW is `G*cin + 2` and is used throughout -
   the pack's row stride, the UV divisor, the texture size's pitch and hi_bytes -
   so it is one number and every consumer follows it.  The two trailing texels
   are the only slack, and they also absorb the symmetric case: in a last chunk
   whose Cin is not a multiple of 32 the zero-padded taps read past the last
   channel with weight 0, and texel rw-1 is what they read instead of the next
   row. */

#define GPU_CV_TAPS    16u      /* one texture unit's kernel width            */
#define GPU_CV_CHUNK   (GPU_CV_TAPS * 2u)   /* both units: 32 channels        */

/* The two constants of the per-unit bias split - see s_cv_hmn[] and cv_digit().

   GPU_CV_PLANE is where the two CLAMPED unit outputs are made to ADD UP TO, and
   it is a CONVENTION rather than a requirement: the fold SUBTRACTS whatever the
   two fields sum to, so any total reconstructs the same answer.  It only decides
   how much headroom a unit has before its own floor.

   THE CEILING IS A REAL CONSTRAINT AND THIS IS AN INTERIOR OPTIMUM, but it sits
   at 128 and not at 192.  Swept offline at the firmware's OWN rung (chunk 32,
   uniform-proxy variance, k=2, and with the plane's byte rail as a gate) the
   layer count runs

       96 -> 26    112 -> 27    128 -> 28    192 -> 26    224 -> 28

   so 128 is the best of the set and the curve rises to it from both sides.

   THE PREVIOUS VALUE HERE WAS 192, AND IT WAS WRONG, because the sweep that
   chose it did not bind: `CV_PLANE` was being set in a shell whose environment
   did not reach the measurement process, so every value in that sweep ran at the
   file's DEFAULT of 128 and the result was misread as a plateau.  A PLATEAU IS
   THE SIGNATURE OF A KNOB THAT IS NOT BEING SET - a flat response is evidence
   about the harness before it is evidence about the model, and the 15:33 board
   capture on 2026-09-22 agrees with the corrected reading: it is TOP-limited
   (`at0` is 0 on every plane while `at255` is non-zero on 11 layers, and all 11
   of those read FALLBACK), which is exactly the pressure a lower plane relieves.

   GPU_CV_KSD is the window's half-width in standard deviations of the unit's own
   partial.  2.0 measured best and 1.0..4.0 all lose nothing. */
#define GPU_CV_PLANE   128.0f
#define GPU_CV_KSD     2.0f
#define GPU_CV_MAXCIN  320u     /* the model's widest input is 288            */
#define GPU_CV_MAXK    ((GPU_CV_MAXCIN + GPU_CV_CHUNK - 1u) / GPU_CV_CHUNK)
#define GPU_CV_MAXW    512u     /* texels per source row, RW                  */
#define GPU_CV_MAXCOUT 1024u
/* rw * rows, i.e. `(G*cin + 2) * rows` = np*cin + 2*rows: one plane of the
   source slot, in texels.  That is the extent the pack actually fills and the
   extent dhd_set_texture_size() walks (the pitch is rw), so it - and not the
   smaller `np*cin` the first version of the pack used - is what both planes have
   to fit in.  Two bytes per texel and two planes (hi and lo) in the 192 KB of
   GPU_DW_SRC, so the slot is 49152 texels per plane and this is a CAPACITY
   limit, not a tuning knob - 4 * 49152 = 196608 is exactly GPU_DW_SRC's size.

   It is the limit that loses op186 (288 channels at 20x20): rw*rows there is
   about 115240 texels, well past 49152, and there is no packing that fixes it.
   The pack has to cover the whole Cin because every chunk's draws read a
   different x range of the same rows, and both the hi and the lo plane have to
   exist for every draw - so there is nothing to make smaller.

   AND op186 IS NOT A WALL, which is worth saying because an earlier version of
   this comment claimed every layer this cut was one anyway.  tools/
   conv_gpu_emulate.py now checks the structural limits as well as the
   arithmetic, and op186 is CLEAN at the fixed chunk 32.  It is clean and
   undrawable, so the per-layer check in yolo_inference.cpp never gets the chance
   to pass it, and it stays on the CPU.  That costs 0.52 Mpx of the 3.22 Mpx that
   the two clean-but-undrawable layers (op186 and the 3x3 op1) would have added -
   see the COST note in gpu.h for what is actually shipped. */
/* One plane's texel bound: half of GPU_DW_SRC's 192 KB at two bytes a texel, so
   x4 is exactly the slot.  It is what the capacity check used to be written
   against; the live check is now `rw*rows*4` in the prologue, the whole
   two-half texture, because that is the exact extent that has to fit.  Kept
   because gpu.h's cost note names it and the number itself is the useful form
   of "how big is one plane allowed to be". */
#define GPU_CV_MAXSRC  49152u
/* GPU_CV_CLUT_A and GPU_CV_CLUT_B come from gpu.h: 0 and 64, the two bases the
   Step 1 probe used to prove the units read independent CLUT offsets.  Reused
   rather than redefined, because a second pair of constants with the same names
   and different values is exactly how a kernel ends up uploaded to one place and
   sampled from another. */

/* s_cv_gpu_dead lives up with gpu_drain(): one GPU, one latch set from the one
   place that can know, and this file's callers each read their own flag. */

/* Latches after the one digit comparison per boot, so the diagnostic costs two
   lines and not two per layer.  See its block in gpu_conv(). */
static int      s_cv_dig_done = 0;

/* One 1x16 kernel per unit, refilled per chunk.  Four CLUT words each, packed
   and uploaded by dhd_set_texture_kernel() itself - which is also the call that
   sets the unit's kernel width/height/sample_shape and enables its texture
   filter, so it has to happen per draw whatever the CLUT already holds. */
static int8_t   s_cv_kern[2][GPU_CV_TAPS];

/* The packing pass's per-channel plane sums: SUM_p T5, SUM_p (T5>>2), SUM_p r,
   SUM_p (r>>2).  t5bar and rbar come straight out of them, and so does each
   (co, chunk)'s gain g - exactly, and for free, because the gain's numerator
   factors over the taps: SUM_p SUM_ci w*EX is SUM_ci w*SUM_p EX.  So the
   replication's gain costs O(Cin*Cout) arithmetic and no extra memory traffic,
   where measuring it the way the emulator does would cost a CPU convolution. */
static uint32_t s_cv_t5[GPU_CV_MAXCIN];
static uint32_t s_cv_ex[GPU_CV_MAXCIN];
static uint32_t s_cv_r [GPU_CV_MAXCIN];
static uint32_t s_cv_rq[GPU_CV_MAXCIN];

/* THE PER-CHANNEL EXTREMES OF EACH PLANE, and they are the only new input the
   per-unit bias split needs beyond the weight sums it already has.

   The measured failure this answers: the hardware CLAMPS EACH TEXTURE UNIT
   BEFORE THE COLOUR ADD, so a unit whose own partial drops below its floor loses
   its taps AND its bias - a whole ~64 on a plane that is otherwise flat.  The
   board caught it on n47 chunk 1, where unit 1 was gone on all 100 pixels.

   A unit cannot be kept off its floor by centring alone, because the bias the
   pair may hand out is fixed at `PLANE - cen` and the split is the only freedom.
   Spending that split well needs to know how far each unit TRAVELS, and
   `t5bar` is a mean and cannot say.  The spread can:

       var(C_i) ~ (mx_i - mn_i)^2 / 12          (uniform between the extremes)
       var(V_u)  = scale_u^2 * SUM_{i in u} w_i^2 * var(C_i)

   and the second line drops the cross terms, which is the one approximation
   here.  The UNIFORM proxy is used rather than the exact variance because it
   measured no worse and costs two compares per activation instead of three new
   accumulator arrays per channel - `CV_VARQ` in tools/conv_gpu_emulate.py.

   Offline, at the firmware's own chunk 32: 25 layers shipped -> 34, and ZERO
   lost, on the mirror's frame.  `k` swept 1..4 loses nothing at any value, so
   this is a plateau and not a fitted constant.  The per-pixel-oracle ceiling is
   39, so this reaches most of what a window can reach at all. */
static uint8_t  s_cv_hmn[GPU_CV_MAXCIN], s_cv_hmx[GPU_CV_MAXCIN];
static uint8_t  s_cv_lmn[GPU_CV_MAXCIN], s_cv_lmx[GPU_CV_MAXCIN];

static uint32_t s_cv_layers   = 0u;
static uint32_t s_cv_draws    = 0u;
static uint32_t s_cv_refused  = 0u;
static uint32_t s_cv_packed   = 0u;
static uint32_t s_cv_unfolded = 0u;

/* THE FOLD'S OWN TWO CLAMP COUNTS, and they are about the PLANE the colour pass
   wrote and NOT about a texture unit.  That distinction is the whole reason they
   exist, because the two clamps fail differently and only get confused once:

     a TEXTURE UNIT leaving the byte range is the offline table's `sat`.  It
     happens INSIDE the colour pass - each unit is scaled, biased and clamped on
     its own and only then are the two ADDED - so the unit's own partial never
     leaves the GPU and no board quantity can count it.  What reaches the plane
     is the SUM of two already-clamped halves.

     the ADD's result not fitting the byte it is written to is the offline
     table's `rail`.  The plane is a byte and two halves that are each >= 0 can
     sum past 255, and what the write cuts at the top is not in the byte the
     fold reads back.  tools/conv_gpu_emulate.py says in its own words that the
     board's `pl` dump counts exactly this as `at255`.

   So `s_cv_rail` is `rail` and it is COUNTED HERE rather than read out of the
   planes afterwards, because the fold already has every one of these bytes in a
   register: `dw_rd()` below returns it and the two compares are free.  The
   census can also count it, but it walks the same bytes through the Device map
   a second time and only ever covers one output channel.

   `at0` rides along because it costs the same compare and because it is the
   other half of the evidence: an `at255` on a plane the model says never clamps
   is a rail, and an `at0` is what a plane of zeros (a whole dead unit) looks
   like.  Both are plane BYTES, over the whole layer and every output channel -
   NOT a sample, unlike the glue's `worst`/`n8`/`bad`.

   Zeroed on entry to every gpu_conv() call, so a caller that reads them straight
   after one draw is reading that draw and not that draw plus the frame before.

   `s_cv_bu_min` is the smallest |b| over this call's four bias fields of every
   chunk - i.e. over every texture unit the layer drew.  It is reported and NOT
   judged: a verdict clause requiring it to clear `CV_GAP_MAX` was tried in
   `conv_run_check` on 2026-10-09 and reverted the same day, because the
   mechanism it rested on ("a lost unit costs exactly its own `b_u`") does not
   hold - `cv_digit()` centres EVERY unit's post-bias mean on 64, so the cost of
   a lost unit is that mean and not `b_u`.  It survives as a column on the chk
   line because the mechanism question is still open and this is the reading it
   needs; see the note in `conv_run_check`.

   -1 means no unit was seen (the call refused before its first chunk), which a
   caller must read as "nothing drawn", not as a value to compare.

   Zeroed (to -1) on entry to every gpu_conv() call like the pair above. */
static uint32_t s_cv_rail   = 0u;
static uint32_t s_cv_at0    = 0u;
static int32_t  s_cv_bu_min = -1;

void gpu_conv_stat(uint32_t* layers, uint32_t* draws, uint32_t* refused,
                   uint32_t* packed, uint32_t* unfolded)
{
    if (layers   != NULL) { *layers   = s_cv_layers;   }
    if (draws    != NULL) { *draws    = s_cv_draws;    }
    if (refused  != NULL) { *refused  = s_cv_refused;  }
    if (packed   != NULL) { *packed   = s_cv_packed;   }
    if (unfolded != NULL) { *unfolded = s_cv_unfolded; }
}

/* What the LAST gpu_conv() call's fold saw on the digit planes - see s_cv_rail.
   Separate from gpu_conv_stat() because that one is a since-boot count and this
   one is about one call, which is what the per-layer check has to judge. */
void gpu_cv_fold_stat(uint32_t* rail, uint32_t* at0)
{
    if (rail != NULL) { *rail = s_cv_rail; }
    if (at0  != NULL) { *at0  = s_cv_at0;  }
}

/* The smallest |b_u| over every texture unit of every chunk the LAST
   gpu_conv() call drew - REPORTED, not judged (see s_cv_bu_min for why the
   verdict clause this was built for was reverted).  < 0 means no unit was seen
   (the call refused before its first chunk). */
void gpu_cv_bias_min(int32_t* bu_min)
{
    if (bu_min != NULL) { *bu_min = s_cv_bu_min; }
}

/* ===== the digit-plane census ==============================================
   WHY THIS EXISTS.  tools/conv_gpu_emulate.py models this operator's whole
   arithmetic - the 5-bit texel, the replication excess, the per-chunk gain, the
   digit's own clamp, the fold - and its `fix_draw` column is the layout gpu.c
   actually implements (write plane 2k/2k+1 per chunk, fold planes 0..2K-1, no
   `co` term on either side; see the draws at cv_pass()'s two call sites and the
   fold's `dw_rd(GPU_DW_DST + (k * pstride), y)`).  For the model's layer #2
   (op6, 24->24 40x40) that column reads worst 4, bad 0.

   The board reads worst 122, bad 4218 on the same layer - and, worse, the
   model's own clipping evidence has NO correlation with which layers fail: its
   op22 has the NARROW hi range [49..207] and fails, its op32 has the FULL range
   [0..255] and passes.  So the failures are not the saturation the design
   modelled, and no amount of re-reading the offline table will name them.

   What will: these four numbers per digit plane, which the offline model can
   print for the same layer and which say WHERE a plane differs - `mn`/`mx` for
   a shift, `at0`/`at255` for a clamp that the model says cannot happen.

   OFF BY DEFAULT, and it is expensive: one byte read per byte of every plane,
   on the Device-mapped slot.  The glue turns it on for a bounded number of
   failing layers and re-runs that layer's draw, so the cost is a few planes a
   session and never on the healthy path. */
static int s_cv_census    = 0;
static int s_cv_census_co = 0;
static int s_cv_census_n  = -1;

void gpu_cv_census(int on, int co, int layer)
{
    s_cv_census    = (on != 0) ? 1 : 0;
    s_cv_census_co = co;
    s_cv_census_n  = layer;
}

/* THE THREE-WAY DUMP - the one thing every reading so far has been missing,
   which is the BOARD'S OWN ACTIVATIONS.

   `tools/conv_gpu_emulate.py` is the Step-0 oracle, and it computes BOTH sides'
   arithmetic from a jpg's activations.  So it can only ever say what the two
   MODELS disagree about, never what the hardware did.  On the detection head
   the two readings are not merely far apart, they are of different kinds: over
   40 real images (20 train, 20 val) the oracle pins every head slot at
   fix_draw 4-6 / bad 0, while `as_written` on those very rows swings worst
   22..153 - so the oracle IS sensitive to its input - and the board reads
   n39/n43/n47 FALLBACK on nine consecutive boots, n43 worst 83 bad 2706.  A
   2706-against-0 gap is not an input choice; the argument can only be closed on
   the board's own bytes.

   What it prints, for ONE layer, ONCE per boot:

     in     the layer's input tensor, h*w*cin bytes     (n47: 7200)
     pl<k>  each digit plane as the fold reads it, np   (n47: 6 x 100 = 600)
     out    the GPU's own output, np*cout               (n47: 300)

   The CPU reference is dumped by the glue (`yolo_inference.cpp`), which is
   where that buffer lives.  Together those four are enough to attribute a
   differing byte to the PACK, the DRAW or the FOLD with no further flash: the
   pack is reproducible from `in` alone, and the fold from the planes alone, so
   planes that match a hand-computed plane while `out` does not is the fold -
   and planes that do not match are the draw, and nothing else in this function
   touches a plane.

   ARMED BY THE GLUE for one layer, and it DISARMS ITSELF at the first
   opportunity.  A dump is ~8 KB of serial on the Device-mapped slot; it must
   never become a per-frame cost. */
static int s_cv_dump   = 0;
static int s_cv_dump_n = -1;

void gpu_cv_dump(int n)
{
    s_cv_dump   = 1;
    s_cv_dump_n = n;
}

/* Hex, 32 bytes a line, through a buffer of our own.

   THAT REASON IS GONE AND THE BUFFER STAYS.  It was written when Message[] held
   64 bytes, where one 32-byte line plus its tag did not fit; Message[] is 160
   now, so a hex line would fit.  The local buffer is kept because changing it
   would mean re-driving the hex dumps to prove the byte-for-byte output is
   unchanged, to save 112 bytes of stack on a function whose whole job is
   diagnostics - not a trade worth making on a path this project has already
   paid twice to chase down a print.  The number in the old comment was true
   when written and is not true now, which is the only thing worth saying here. */
void gpu_cv_hex(const char* tag, const int8_t* p, int count)
{
    static const char H[] = "0123456789abcdef";
    char           buf[112];
    int            i, j;
    uint32_t       n = (count > 0) ? (uint32_t)count : 0u;

    if ((tag == NULL) || (p == NULL)) { return; }

    for (i = 0; (uint32_t)i < n; i += 32)
    {
        const uint32_t m = (n - (uint32_t)i < 32u) ? (n - (uint32_t)i) : 32u;
        int            k = sprintf(buf, "D %s +%d ", tag, i);

        for (j = 0; (uint32_t)j < m; j++)
        {
            const uint8_t v = (uint8_t)p[i + j];
            buf[k++] = H[v >> 4];
            buf[k++] = H[v & 15u];
        }
        buf[k++] = '\r';
        buf[k++] = '\n';
        buf[k]   = '\0';
        UART_SendString(buf);
    }
}

/* The same dump for a DIGIT PLANE.  Those live behind the uncached scratch's
   window one byte per np-strided row, so they are read with the very same
   dw_rd() the fold uses - reading them any other way would be a different
   reading than the one under test. */
static void cv_dump_plane(const char* tag, uint32_t base, uint32_t count)
{
    static const char H[] = "0123456789abcdef";
    char              buf[112];
    uint32_t          i, j;

    for (i = 0u; i < count; i += 32u)
    {
        const uint32_t m = (count - i < 32u) ? (count - i) : 32u;
        int            k = sprintf(buf, "D %s +%u ", tag, (unsigned)i);

        for (j = 0u; j < m; j++)
        {
            const uint8_t v = (uint8_t)dw_rd(base, i + j);
            buf[k++] = H[v >> 4];
            buf[k++] = H[v & 15u];
        }
        buf[k++] = '\r';
        buf[k++] = '\n';
        buf[k]   = '\0';
        UART_SendString(buf);
    }
}

/* Round half away from zero, like TFLM's own requantisation and unlike C's
   implicit truncation.  Used for the hardware's integer bias field and for the
   fold's final rounding. */
static int32_t cv_rnd(float v)
{
    return (int32_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

/* One digit of one (co, chunk): the scale and the two biases the hardware needs,
   and the range checks that are the only places this arithmetic can run out of
   room.  `inv` is 1/(1+g) for a hi digit and 1 for a lo digit - the gain is a
   pure scale on the hi plane and does not exist on the lo one, because rep5 is
   applied to r before the sum, which is exactly what the hardware does.

     hi:  k1 = 256, k2 = 8    asc = 256*M/(1+g)^2,  cen_u = 8*M*t5bar*wsk_u/(1+g)
     lo:  k1 =  32, k2 = 1    asc = 32*M,           cen_u = M*rbar*wsk_u

   THE BIAS IS PER TEXTURE UNIT, AND IT IS THE MEASURED FIX.

   The draw's two units each scale their own half of the chunk, add their OWN
   bias and are then CLAMPED TO A BYTE ON THEIR OWN, and only then does the
   colour ADD sum the two.  That was an open question in this file for as long as
   the operator has existed - the note that used to stand here said so, and said
   the symmetric split was chosen because it is no worse off *if* the clamp is
   per unit.  The board answered it on n47: it IS per unit.  Chunk 1's hi plane
   loses unit 1 ENTIRELY - its taps and its bias both - on all 100 pixels, and
   the reason is that this function used to hand BOTH units `trunc((128-cen)/2)`,
   which centres the PAIR and not either unit.  With `cen_0 ~ +102, cen_1 ~ -26`
   that puts unit 1's own mean at 0, i.e. exactly on the floor.

   What the centring is for is only ever this: the fold SUBTRACTS the bias the
   hardware was given, so the centring cancels exactly and no choice of it can
   bias a layer.  Its whole job is to keep a unit off a rail.  Unit u can absorb
   `cen_u + b_u` of downward excursion, and the digit plane is a byte too - the
   two CLAMPED outputs are added into it - so the pair's total is fixed at about
   128 by that second constraint:

       b_0 + b_1 = 128 - cen,     cen = cen_0 + cen_1

   The symmetric split spends that 128 as 64 each and is therefore right only
   when the two halves are balanced.  Centring each unit on its OWN mean,

       b_u = trunc(64 - cen_u)

   gives every unit 64 of downward headroom whatever the split is, and reduces to
   the symmetric form - to the integer - whenever `cen_0 == cen_1`.  So it cannot
   move a layer the symmetric form already passed; the offline table diffs the
   two and loses nothing while gaining five.

   WHAT THAT STILL DOES NOT GIVE is more than 64 per unit, and 64 is a bound on
   where a unit SITS rather than on how far it TRAVELS.  The board's one bad byte
   on n45 c14 is a plane of 126 reading 63 - one unit gone - and that needs
   `trunc(V_u) < -64` while the PAIR only moves by +-7.  The two units hold large
   and OPPOSITE partials that cancel, so a per-unit 64 is not the half of the
   pair's range the symmetric argument assumes.

   So the split is chosen to maximise the SMALLEST margin to the floor, which
   needs each unit's own SPREAD.  `t5bar` is a mean and cannot say; the
   per-channel extremes can, and they are four compares per activation in the
   packing pass (see s_cv_hmn[]).  The result, at the firmware's own chunk 32 and
   measured offline: 25 layers shipped -> 34, and ZERO lost.  The per-pixel
   oracle reaches 39, so this window gets most of what a window can get at all.

   `wsk[2]` are the SIGNED weight sums over each unit's own 16 channels and
   `varp[2]` the sums of `w^2 * var(C_i)` over the same halves - together the
   only inputs this needs beyond the chunk's means and gain.

   `trunc` and not `round`, because C's cast toward zero is what the offline
   table's `np.trunc` does, so the two agree by construction and not by luck.

   Returns false when either field cannot hold the value, and the caller refuses
   the LAYER - a wrong convolution being worse than a slow one. */
/* The 5-bit plane field as the KERNEL sees it: replicated to eight bits, over
   eight.  `rep5(v) = (8v + (v>>2))/8` as a float, which is what the mirror's
   rep5() computes and what the hardware's multiplicand really is (the
   measured contract, not the 5-bit value itself). */
static float cv_rep5(uint32_t v)
{
    return (float)v + ((float)(v >> 2) * 0.125f);
}

static bool cv_digit(float m, float mean, const float wsk[2],
                     const float varp[2], float k1, float k2,
                     float inv, float* asc_out, int32_t b_out[2])
{
    float asc = k1 * m * inv * inv;
    float cen[2], tl[2], th[2], cen_tot, tgt, b0f, lo0, hi0;
    int   u;

    /* The scale field is a 10-bit unsigned significand with a signed 6-bit
       exponent, so it is positive and finite or it is nothing. */
    if (!(asc >= 2.4e-7f) || (asc > 4.0e12f)) { return false; }

    cen_tot = 0.0f;
    for (u = 0; u < 2; u++)
    {
        /* `sc` is the whole of this digit's multiplier apart from the mean and
           the weights: `8*M/(1+g)` on the hi plane, `M` on the lo one.  Both
           the centring and the spread are that same scale times a per-unit
           input, which is why one `sc` serves for both. */
        const float sc = k2 * m * inv;
        const float sd = sc * sqrtf((varp[u] > 0.0f) ? varp[u] : 0.0f);

        cen[u] = sc * mean * wsk[u];
        /* np.trunc at both ends, so this function and the offline table agree
           by construction and not by luck - `trunc` is C's cast toward zero. */
        tl[u]  = (float)(int32_t)(cen[u] - (GPU_CV_KSD * sd));
        th[u]  = (float)(int32_t)(cen[u] + (GPU_CV_KSD * sd));
        cen_tot += cen[u];
    }

    tgt = (float)(int32_t)(GPU_CV_PLANE - cen_tot);

    /* THE MAX-MIN-MARGIN SPLIT.  `b_0` rises unit 0 and falls unit 1, so the
       point where the two units' distance from their own floor is equal is
       where their distances are equal:
           cen_0 - k*sd_0 + b_0  ==  cen_1 - k*sd_1 + b_1,  b_0 + b_1 = tgt
       which solves to this.  When the two units have the same spread it reduces
       to `tgt/2 - cen_0` exactly - the rule this replaces - and it is the whole
       reason the spread is worth carrying: it is what lets the split favour the
       unit that actually travels further. */
    b0f = (tgt + tl[1] - tl[0]) * 0.5f;

    /* The window.  Three constraints, and the THIRD is the one that is easy to
       leave out: the bias field is nine bits signed, so `-256 <= b_0 <= 255`
       and, because `b_1 = tgt - b_0`, also `tgt - 255 <= b_0 <= tgt + 256`.  The
       upper one arrives free inside `tgt + 256` below; the lower one is NOT
       implied by `tgt - (255 - th_1)` unless `th_1 >= 0`, and a unit whose
       weights are all one sign has `th_1 < 0`.  Leaving it out cost four layers
       offline, and putting it here rather than checking `b_1` afterwards is what
       got them back. */
    lo0 = tgt - 255.0f;
    if ((tgt - (255.0f - th[1])) > lo0) { lo0 = tgt - (255.0f - th[1]); }
    if (lo0 < -256.0f) { lo0 = -256.0f; }

    hi0 = tgt + 256.0f;
    if (hi0 > 255.0f) { hi0 = 255.0f; }
    if ((255.0f - th[0]) < hi0) { hi0 = 255.0f - th[0]; }

    /* No split fits: a wall.  The layer is refused and stays on the CPU - a
       wrong convolution being worse than a slow one. */
    if (lo0 > hi0) { return false; }

    if (b0f < lo0) { b0f = lo0; }
    if (b0f > hi0) { b0f = hi0; }

    b_out[0] = (int32_t)b0f;
    b_out[1] = (int32_t)tgt - b_out[0];

    /* With `lo0` above this cannot fail, so if it ever does the window algebra
       is wrong and the layer must not be written on a hope. */
    if ((b_out[0] < -256) || (b_out[0] > 255) ||
        (b_out[1] < -256) || (b_out[1] > 255)) { return false; }

    *asc_out = asc;
    return true;
}

/* SUM over ALL Cin of output channel co's weights.  Needed by the fold's `net`,
   which has to give back the `(128 + in_zp)*sumw` the pack left behind. */
static float cv_sumw(const int8_t* wq, uint32_t cin, uint32_t co)
{
    const int8_t* row = wq + (co * cin);
    float         s   = 0.0f;
    uint32_t      ci;

    for (ci = 0u; ci < cin; ci++) { s += (float)row[ci]; }
    return s;
}

/* The replication's gain on one (co, chunk)'s hi partial, exactly:

       g = SUM_ci w*SUM_p(EX) / (8 * SUM_ci w*SUM_p(T5))      EX = T5 >> 2

   over the chunk's channels.  Skipped - returned as exactly 0 - unless it lands
   in [0, 0.11], which is the range it has when the chunk's weighted taps share a
   sign.  Outside it the chunk's sum is near cancellation and the ratio is not a
   gain at all; the emulator's note records that dividing by a denominator near
   zero cost it 51 layers at a worst error of 255. */
static float cv_gain(const int8_t* wq, uint32_t cin, uint32_t co,
                     uint32_t c0, uint32_t c1)
{
    const int8_t* row = wq + (co * cin);
    float         num = 0.0f, den = 0.0f;
    uint32_t      ci;

    for (ci = c0; ci < c1; ci++)
    {
        float w = (float)row[ci];

        num += w * ((float)s_cv_ex[ci] * 0.125f);
        den += w * (float)s_cv_t5[ci];
    }

    if (den == 0.0f) { return 0.0f; }

    {
        float g = num / den;

        return ((g >= 0.0f) && (g <= 0.11f)) ? g : 0.0f;
    }
}

/* THE NUMBER cv_gain() THROWS AWAY, and the one the failing layers differ by.

   cv_gain() answers 0 both when the chunk is perfectly gain-free AND when the
   ratio fell outside [0, 0.11] and the correction was declined - from the
   caller the two are indistinguishable, and `inv` in pass 3 is 1.0 for both.
   When it is the second case, the hardware's multiplicand still carries its
   `(1+g)` and nothing divides it out, so the hi plane comes out that factor too
   large - which is precisely a scale error on a plane that is otherwise
   centred.  That is the shape the board's `rng` lines show for n1/n5/n7/n9/n43
   (the GPU's range is the CPU's widened several-fold, `sat g` in the thousands)
   and it is the shape the no-saturation family should NOT have.

   Same accumulation, same order, same operands as cv_gain() above - only the
   window test is left off.  Called from the diagnostic only, so it costs
   nothing on the shipped path. */
static float cv_gain_raw(const int8_t* wq, uint32_t cin, uint32_t co,
                         uint32_t c0, uint32_t c1)
{
    const int8_t* row = wq + (co * cin);
    float         num = 0.0f, den = 0.0f;
    uint32_t      ci;

    for (ci = c0; ci < c1; ci++)
    {
        float w = (float)row[ci];

        num += w * ((float)s_cv_ex[ci] * 0.125f);
        den += w * (float)s_cv_t5[ci];
    }

    return (den == 0.0f) ? 0.0f : (num / den);
}

/* The two-unit digit, for the diagnostic only: each texture unit truncates its
   OWN scaled sum, adds its OWN bias and is clamped on its own, the colour ADD
   sums the two results, and the write into the byte plane clamps that.  That is
   the hardware's structure - now MEASURED and not merely the structure the
   emulator's summed model idealises away, see cv_digit() - and the biases are
   per unit because that is what the drawing path programs. */
static int32_t cv_dig2(const float a[2], float asc, const int32_t b[2])
{
    int32_t d = 0;
    int     i;

    for (i = 0; i < 2; i++)
    {
        int32_t p = (int32_t)(a[i] * asc / 32.0f) + b[i];

        if (p < 0)   { p = 0; }
        if (p > 255) { p = 255; }
        d += p;
    }
    return d;
}

/* A ZERO BIAS PAIR, for the probe call sites whose draws carry no centring at
   all.  A named object and not a literal because cv_pass()/cv_pass_g() take the
   bias PER TEXTURE UNIT now, so a bare `0` at those call sites would be a null
   pointer and not an unbiased draw. */
static const int32_t cv_b0[2]  = { 0, 0 };
/* ...and the probe's one non-zero bias, which is symmetric on purpose: it is
   testing that a byte the pair reaches is the two units' bias summed, so both
   units must carry it. */
static const int32_t cv_b20[2] = { 20, 20 };

/* The geometry of one pointwise draw, gathered here so that the diagnostic in
   gpu_conv() can vary it a field at a time.  cv_pass() fills every field from
   (G, rows, cin, c0) - one place, so production and diagnostic cannot drift
   apart in the sequence, only in these numbers. */
typedef struct
{
    uint32_t tw, th, tp;    /* texture: width, height, pitch, in TEXELS       */
    uint32_t fw, fh, fp;    /* framebuffer: width, height, pitch, in bytes    */
    uint32_t bx, by;        /* bbox: max x and max y                          */
    float    u0, dustep;    /* unit u's window centre: u0 + u*dustep          */
    float    dux, dvy;
    float    v0;            /* the window's first ROW, in v, as a fraction of  */
                            /* th.  0.0 is the hi digit plane; 0.5 is the lo   */
                            /* one, which lives in the rows below the hi one   */
                            /* inside the SAME texture - see cv_pass().        */
} cv_geom_t;

/* One pointwise draw: two texture units over the same source plane and the same
   destination, each with its own 1x16 kernel (already in s_cv_kern[]), its own
   CLUT base, its own UV and its own gain, summed by E_DHD_OP_ADD.

   PER UNIT, in cv_run()'s order, for the reason dw_pass_bc() documents at
   length: dhd_active_texture_unit() is what sets device->m_tex_unit_base, and
   every setter below computes its register address as "that base + an offset".
   dhd_set_uv() is the exception - it takes the unit as an argument - and
   dhd_set_texture_unit_enable() is the one genuinely global call here. */
static bool cv_pass_g(dhd_handle_t dev, const cv_geom_t* g, uint32_t src,
                      uint32_t dst, float asc, const int32_t bias[2])
{
    dhd_uv_data_t   uv;
    dhd_pixel_org_t saved_fmt;
    uint32_t        saved_fb;
    uint32_t        sav_w, sav_h, sav_p;
    uint32_t        sav_x0, sav_y0, sav_x1, sav_y1, sav_ys;
    uint32_t        u;

    dhd_set_texture_unit_enable(dev, 2u);

    for (u = 0u; u < 2u; u++)
    {
        uv.m_u0  = g->u0 + ((float)u * g->dustep);
        uv.m_v0  = g->v0;
        uv.m_dux = g->dux;
        uv.m_duy = 0.0f;
        uv.m_dvx = 0.0f;
        uv.m_dvy = g->dvy;

        dhd_set_uv(dev, u, &uv);
        dhd_active_texture_unit(dev, u);
        dhd_set_texture_address(dev, (dhd_gpu_ptr_t)src, 0u);
        dhd_set_texture_size(dev, g->tw, g->th, g->tp);
        dhd_set_texture_format(dev, GPU_DW_FMT_SIGNED_565,
                               E_DHD_ACCESS_LINEAR, 0u);
        dhd_set_texture_kernel(dev, GPU_CV_TAPS, 1u,
                               (const uint8_t*)s_cv_kern[u],
                               s_dw_clut_base + ((u != 0u) ? GPU_CV_CLUT_B
                                                           : GPU_CV_CLUT_A));
        dhd_set_texture_clut_base(dev, s_dw_clut_base +
                                  ((u != 0u) ? GPU_CV_CLUT_B : GPU_CV_CLUT_A));
        dhd_set_texture_scalebias(dev, asc, bias[u]);
        dhd_set_texture_wrap(dev, E_DHD_TEX_BORDER, E_DHD_TEX_BORDER);
        /* 0 and not 0x80808080 - see dw_pass_bc()'s note: the border colour is
           documented ARGB8888, so 0x80808080 is 0x80 per channel and its 5-bit
           blue field is 16, where a tap outside the texture has to contribute
           0.  In this layout no tap with a non-zero weight ever leaves its row;
           the ones that do have weight 0 by construction. */
        dhd_set_texture_border_color(dev, 0u);
    }

    /* BACK TO UNIT 0, and it is not tidiness.  The loop above ends on unit 1
       because dhd_active_texture_unit() is what sets device->m_tex_unit_base
       and every setter that does not take a unit id computes its register
       address as that base + an offset.  dw_pass_bc() - the one form of this
       sequence OBSERVED to write - selects unit 0 and leaves it selected, so
       every call after its texture setup runs with unit 0 active.  Here the
       colour setup, the framebuffer save/set, the bbox and the draw all came
       after a loop that left unit 1 active, which is a difference from the
       proven order that nothing had ever tested and that costs one call to
       remove.  If any of those calls is in fact unit-indexed, this is the whole
       of the defect; if none is, it costs nothing. */
    dhd_active_texture_unit(dev, 0u);

    /* ADD: op1a + op1b, each operand being one unit's complete texture result,
       scale and bias included.  That is what `gpu cv 2u copy 16 add 130` reads
       and why the probe's own first version, which expected COPY to sum, had no
       way to pass. */
    cv_color_setup(dev, E_DHD_OP_ADD);

    saved_fb  = nsgpu_get_framebuffer();
    saved_fmt = dhd_get_framebuffer_format(dev);
    dhd_get_framebuffer_size(dev, &sav_w, &sav_h, &sav_p);
    dhd_get_bbox(dev, &sav_x0, &sav_y0, &sav_x1, &sav_y1, &sav_ys);

    dhd_set_framebuffer_format(dev, E_DHD_PIXEL_ORG_L_8);

    /* The destination's rows are fp bytes apart.  For the production geometry
       fp == G, so a plane is np contiguous bytes and every reader of one is a
       plain dw_rd() walk. */
    dhd_set_framebuffer_size(dev, g->fw, g->fh, g->fp);

    /* THE RETURN VALUE IS THE WHOLE POINT.  A refused address is not set
       (davehd_driver_blu_inl.h:473-492), so the draw would go to the previous
       framebuffer and the fold would read a plane nobody wrote - a wrong
       answer, silently, with every printed number green.  Reading it turns
       that into a refusal, which is the hard rule here. */
    if (dhd_set_framebuffer_address(dev, (dhd_gpu_ptr_t)dst) != E_DHD_OK)
    {
        if (s_cv_fb_n < 4u)
        {
            char msg[64];

            s_cv_fb_n++;
            sprintf(msg, "gpu cv fb reject a%08X w%u\r\n",
                    (unsigned)dst, (unsigned)dhd_get_bufferalignment_required(dev));
            UART_SendString(msg);
        }
        dhd_set_framebuffer_format(dev, saved_fmt);
        dhd_set_framebuffer_size(dev, sav_w, sav_h, sav_p);
        dhd_set_bbox(dev, sav_x0, sav_y0, sav_x1, sav_y1, sav_ys);
        (void)dhd_set_framebuffer_address(dev, (dhd_gpu_ptr_t)saved_fb);
        return false;
    }

    dhd_set_bbox(dev, 0u, 0u, g->bx, g->by, 0u);
    dhd_draw(dev, 0u, 0u, (dhd_drawattribute_flag_t)0u);
    dhd_wait(dev, E_DHD_WAIT_WRITEBACK);

    dhd_set_framebuffer_format(dev, saved_fmt);
    dhd_set_framebuffer_size(dev, sav_w, sav_h, sav_p);
    dhd_set_bbox(dev, sav_x0, sav_y0, sav_x1, sav_y1, sav_ys);
    (void)dhd_set_framebuffer_address(dev, (dhd_gpu_ptr_t)saved_fb);

    /* COMMIT THIS DRAW ON ITS OWN - the probe's own commit, and the round's
       hypothesis made into a manipulation.

       READ THE ORDER OF THE TWO CALLS ABOVE.  dhd_wait(E_DHD_WAIT_WRITEBACK)
       waits for the WRITEBACK engine; it does not submit.  Submission is
       dhd_flush(), and this function never calls it, and neither does cv_pass().
       Every draw either of them issues therefore sits in the driver's command
       buffer until the ONE gpu_drain() at the bottom of the dig block - and the
       dig block draws THIRTEEN times.

       That is the confound in every probe reading this diagnostic has taken, and
       it fits the evidence better than any geometric hypothesis ever did.  Line
       every probe reading up by its position in the commit:

         batch     draw     position   this boot     previous boot
         dig       a..e       1-6       wrote         wrote
         dig       g,h        7-8       nothing       WROTE
         dig       s1,s2,s3   9-11      nothing       nothing
         k-probe   k1,k2      1-2       wrote         wrote
         kv        control    5         untouched     written (176)

       The first six always write.  Everything after that flips between boots.
       And the K-probe - which is its OWN commit, after the dig's drain - writes
       at positions 1,2 and 4 and leaves its position-5 no-draw control alone
       this boot, where last boot that same control read 176 because the batch
       before it was still in flight.

       Under this reading `gpu cv dig nw g 0 h 0 s1 0 s2 0 s3 0` does not say the
       production draw writes nothing - it says the SEVENTH DRAW OF A COMMIT does
       not, and every conclusion about the address, the UV, the framebuffer and
       the texture's extent was drawn from probes sitting at positions 7 to 11.
       Those four falsifications are all still correct as measurements; what was
       wrong was attributing them to a field.

       So the commit becomes per draw and the positions disappear: if s1/s2/s3
       write now, position was the whole of the last five rounds.  If they still
       do not, position is dead too and only the draw itself is left. */
    (void)gpu_drain(dev);
    return true;
}

/* The production geometry, and the only caller that should use it.  Every field
   is a function of the layer's shape: `G` pixels per row of a DRAM row of
   rw = G*cin + 2 texels, one texture frame of `rows` rows, a destination plane
   of G x rows with a G-byte row pitch, a bbox over all of it, and a UV that
   walks one texel-block per destination column and one row per destination row.

   The window's centre for destination column 0, unit u, chunk starting at
   channel c0: tap 0 sits at (centre - 1), so the centre is the block's first
   channel plus the guard texel - 1 + c0 + u*16 - and one more for the -1.
   dux is cin/rw and not 1/rw: one destination column is one PIXEL, and a pixel
   is cin texels wide.

   THE TWO DIGIT PLANES ARE ROWS OF ONE TEXTURE, and `v0` is which half of it
   this draw reads.  The texture is 2*rows tall - hi in rows 0..rows-1, lo in
   rows rows..2*rows-1, which is where the pack puts them - and dvy is
   therefore 1/(2*rows), not 1/rows, so that destination row y still lands on
   source row y of its own half.  v0 = 0.0 for the hi and 0.5 for the lo, and
   0.5 is exact: the v axis resolves 1:1 onto texel rows (unlike u, which
   carries the -1 the note above describes), and 0.5*2*rows is the integer
   `rows` for every rows.

   WHY ONE TEXTURE AND NOT TWO - AND THE HYPOTHESIS THAT PROMPTED IT WAS WRONG.
   Recorded in full because it is the third answer this operator has had to
   withdraw, and the withdrawal is the useful part.

   It used to be two: the pack wrote the lo texels at GPU_DW_SRC + hi_bytes and
   the lo draw called dhd_set_texture_address(dev, GPU_DW_SRC + hi_bytes, 0).
   That draw wrote nothing in every boot ever recorded - seven of them, with the
   lo plane reading exactly mn0 mx0 su0 - and the destination, the UV and the
   gain were each measured away in turn, leaving the source address as the one
   input that varied.  A build whose two probe draws differed in exactly that
   read `e 1088` (wrote) against `g 1600` (wrote nothing) in the same commit.
   Both addresses are 128-aligned and inside the 192 KB slot, and
   dhd_set_texture_address() validates nothing at all
   (davehd_driver_txp_inl.h:173-183 stores the raw pointer and returns void), so
   there was no mechanism to read - only the measurement.

   SO THIS REMOVED THE SECOND ADDRESS, AND THEN THE MEASUREMENT SAID NO.  The
   build that introduced it drew the PRODUCTION HI DRAW through this function,
   over the whole plane, into a sentinel-filled plane after a gpu_drain(), and
   read `nw 0` / `rng 238..238` - the same as its no-draw control, twice, in two
   boots.  `g` here is a SINGLE draw; if the two-draw form were the problem it
   would have written.  So the production draw writes NOTHING for either half,
   and the address was never the cause: `e`'s one write in that earlier pair was
   the boot-to-boot instability this file already records (it read 1600 in the
   other boot of the same build, with identical gains), and a single
   non-reproducible write is not evidence.

   The interleave STAYS, because it is simpler and it does remove an address
   from the stream - but it is a simplification, not the fix, and the fix is
   still open.  Two hypotheses have been tried since and both were falsified by
   their own probes: the lo address (the interleave is what replaced it, and the
   draw still writes nothing with one source and one texture) and the
   FRAMEBUFFER (the dig's i/j/m round, which put the production texture and UV
   under the proven 64x8 pitch-64 box-63,7 framebuffer and read 238..238 - the
   sentinel, unmixed).  What the evidence points at now is the TEXTURE'S EXTENT:
   every texture under 8 KB has written, and the 77120- and 154240-byte ones have
   not.  The dig's s1/s2/s3 planes below are that bracket, striding 64 KB. */
static bool cv_pass(dhd_handle_t dev, uint32_t src, uint32_t dst,
                    uint32_t G, uint32_t rows, uint32_t cin,
                    uint32_t c0, float asc, const int32_t bias[2], float v0)
{
    cv_geom_t g;
    uint32_t  rw = (G * cin) + 2u;

    g.tw = rw;   g.th = rows * 2u;  g.tp = rw;
    g.fw = G;    g.fh = rows;       g.fp = G;
    g.bx = G - 1u;
    g.by = rows - 1u;
    g.u0 = ((float)(2u + c0)) / (float)rw;
    g.dustep = (float)GPU_CV_TAPS / (float)rw;
    g.dux = (float)cin / (float)rw;
    g.dvy = 1.0f / ((float)rows * 2.0f);

    /* THE V ANCHOR IS ONE TEXEL-ROW EARLY, AND THIS IS THE MEASURED FIX.

       A single uv governs both the sample and the destination row, so with v0
       the destination's row q carries uv v = v0 + q*dvy and the hardware reads
       texel-row (v0 + q*dvy)*th - 1 - one row EARLIER than intended.  Measured
       on a v ramp (texture row y holds y+2, asc 16), destination rows 0..7 read

           0 32 48 66 82 98 114 132

       which is exactly 16*((q-1)+2) once the 5->8-bit replication is taken into
       account: what looks like "64 80 96 112 128" is really T5(4)=4.125,
       T5(5)=5.125, ... so 66, 82, 98, 114, 132.  Row 0 reads 0 - the border -
       because texel-row -1 does not exist, and 0 is not a value the ramp can
       produce, so that reading is not a coincidence.

       Adding one dvy moves the sample to texel-row q.  1/dvy IS th (rows*2), so
       one dvy is exactly one texel-row - there is no separate constant to get
       wrong.  The hi plane then samples rows 0..rows-1 and the lo (v0 = 0.5)
       samples rows rows..2rows-1: both planes fall exactly inside the texture
       and NO destination row takes a border tap. */
    g.v0 = v0 + g.dvy;

    return cv_pass_g(dev, &g, src, dst, asc, bias);
}

int gpu_conv(const int8_t* in, const int8_t* wq, const int32_t* biasq,
             const float* in_scale, const float* w_scale, float out_scale,
             int32_t in_zp, int32_t out_zp, int h, int w, int cin, int cout,
             int8_t* out)
{
    dhd_handle_t dev = (dhd_handle_t)a_handle;
    uint32_t     np, K, G, rows, rw, hi_bytes, pstride;
    uint32_t     cin32, cout32, ci, co, k, u, t, y, x, cw;
    float        t5bar, rbar;
    int          cv_dmp;

    /* Cleared before the first thing that can return, so the pair is about THIS
       call whatever this call does - a caller that reads them after a refusal
       then reads zero rather than the previous layer's planes.  See s_cv_rail. */
    s_cv_rail   = 0u;
    s_cv_at0    = 0u;
    s_cv_bu_min = -1;

    if (!s_ready)                                       { return -1; }

    /* A drain gave up earlier in this boot: the GPU may still be holding the
       scratch, and handing the scratch to another layer is how a refusal turns
       into a wrong answer.  Checked first, because nothing here is safe to do. */
    if (s_cv_gpu_dead)                                  { s_cv_refused++; return -1; }

    if ((in == NULL) || (wq == NULL) || (biasq == NULL) || (out == NULL))
                                                        { return -1; }
    if ((in_scale == NULL) || (w_scale == NULL))         { return -1; }
    if ((h <= 0) || (w <= 0) || (cin <= 0) || (cout <= 0)) { return -1; }
    if (!(out_scale > 0.0f))                             { return -1; }

    cin32  = (uint32_t)cin;
    cout32 = (uint32_t)cout;

    if (cin32 > GPU_CV_MAXCIN)                           { s_cv_refused++; return -1; }
    if (cout32 > GPU_CV_MAXCOUT)                         { s_cv_refused++; return -1; }

    /* THE DUMP'S GATE, and it is deliberately here - before every geometry
       check, before the pack, before anything that can refuse.  The INPUT is
       the one item that is worth having even from a layer that then declines to
       run, because it is the only piece of the board's state the offline oracle
       has never had and it is what makes the oracle usable as an oracle at all.

       There is no layer number in this signature.  The glue hands it in through
       gpu_cv_census(), which it sets immediately before this call and clears
       immediately after, so keying off that same variable is exact and needs no
       new parameter.  Both must agree or this is a no-op, which is what keeps
       it off the other 51 layers. */
    cv_dmp = (s_cv_dump && (s_cv_dump_n >= 0) && (s_cv_census_n == s_cv_dump_n)) ? 1 : 0;

    if (cv_dmp)
    {
        gpu_cv_hex("in", in, (int)(cin32 * (uint32_t)h * (uint32_t)w));

        /* Taken here, not at the end: a layer that refuses below still owes us
           its input, and re-arming on the next inference would be a per-frame
           cost wearing a one-shot's clothes. */
        s_cv_dump   = 0;
        s_cv_dump_n = -1;
    }

    np   = (uint32_t)h * (uint32_t)w;
    K    = (cin32 + GPU_CV_CHUNK - 1u) / GPU_CV_CHUNK;
    cw   = 2u * K;              /* digit planes per output channel: K hi + K lo */

    if (K > GPU_CV_MAXK)                                 { s_cv_refused++; return -1; }

    /* The row width: the largest divisor of np that keeps RW = G*cin + 2 inside
       the texture unit's width.  A divisor, because np/G has to be a whole
       number of rows; G >= 1 always exists, so this cannot fail once cin is
       inside GPU_CV_MAXCIN. */
    {
        uint32_t gmax = (GPU_CV_MAXW - 2u) / cin32;

        if (gmax > np) { gmax = np; }
        for (G = gmax; (G > 1u) && ((np % G) != 0u); G--) { }
    }
    rows = np / G;

    /* THE ROUNDED PITCHES WERE FALSIFIED, AND REVERTED HERE.

       READ THE FALSIFICATION WITH ITS OWN CAVEAT, because it rests on two
       readings and one of them is now known to be unusable.  That build rounded
       the destination pitch to 64 bytes as well as the texture's - so it IS the
       fp = 64 experiment - and its sentinel read byte-identical, but the
       sentinel was BLIND (the paragraph below says why: an uncommitted draw
       leaves a dhd_wait() satisfied), and its census reading may be the
       depthwise operator's plane rather than the conv draw's output, since that
       operator writes to the same address.  A blind control plus a possibly
       misattributed reading is not a falsification.  The dig's `m` plane re-asks
       the same question with a committed draw and a sentinel read, and the
       framebuffer is now the leading candidate rather than an exonerated one.

       The build before this one rounded both row pitches up to a multiple of
       128 bytes, on the theory that a pitch which is not a multiple of the bus
       width would be refused the way a framebuffer address is
       (davehd_driver_blu_inl.h:473-492).  The next log answered it: the sentinel
       read `hi 1600 1600 lo 1600 1600 of 1600`, byte-identical to the unrounded
       build, and the digit census read `hi plane ... su200319` above
       `lo plane mn0 mx0 su0`, also unchanged.  A pitch the HI draw demonstrably
       writes through cannot be why the LO draw writes nothing, and the two
       draws share both pitches exactly.  So rw is G*cin+2 again, a plane is np
       contiguous bytes again, and the readers go back to dw_rd() - the geometry
       every reading in this file was taken under.

       THE SENTINEL THAT SETTLED IT EARLIER WAS ITSELF BLIND, and that cost two
       builds, so it is recorded here.  It read its four planes immediately after
       four cv_pass() calls and before any dhd_flush() - and a dhd_wait() cannot
       see a job that is still in the CPU-side command stream, because
       E_DHD_WAIT_WRITEBACK is a pipeline-busy wait and an UNCOMMITTED draw
       leaves the pipeline idle.  The only thing that commits a draw is
       dhd_flush().  `1600 1600 1600 1600` is what an uncommitted draw looks
       like, not what a refused one looks like.

       The reading that counts is the census below, because it runs after
       gpu_drain() - and it says the hi plane has content and the lo plane has
       none.  The lo draw is then isolated to exactly three inputs that differ
       from the hi draw's: the source address (GPU_DW_SRC + hi_bytes, the only
       non-slot-base texture address in this project), the destination address,
       and the scale/bias.  The scale/bias is not it - pass 2 calls cv_digit()
       for the same layer and refuses the layer if it fails, and these layers
       reached the draw.  The sentinel now drains before it reads, carries a
       control plane that receives no draw, and separates those two addresses. */
    rw      = (G * cin32) + 2u;                 /* texture row, in texels */
    pstride = ((np) + 127u) & ~127u;            /* one plane's bytes in the DST */

    if ((cw * pstride) > GPU_DW_MAX_PLANE)               { s_cv_refused++; return -1; }

    /* THE LO PLANE IS THE SECOND HALF OF THE SAME TEXTURE NOW, and hi_bytes is
       only an offset INSIDE it.

       It used to be a second texture address: the pack put the lo texels at
       GPU_DW_SRC + hi_bytes and the lo draw handed that pointer to
       dhd_set_texture_address().  That draw wrote nothing in every boot ever
       recorded - seven of them - so the pointer is gone.  cv_pass() draws the
       lo half with v0 = 0.5 out of ONE texture at GPU_DW_SRC + 0, the base the
       hi draw is measured to write through.  hi_bytes survives as the byte
       distance from the hi plane's first texel to the lo plane's: exactly
       rw*rows*2, because the pack writes the lo texels at texel index rows*rw
       of the same plane.  The two dw_wr16 calls below are the same address
       arithmetic they always were, one plane-length apart.

       THE 128-BYTE ALIGNMENT GOES WITH THE ADDRESS.  It existed to make a
       texture address safe (dhd_set_texture_address() validates nothing at all
       - davehd_driver_txp_inl.h:173-183 stores the raw pointer and returns
       void - and the framebuffer path's alignment rule, which IS in the driver
       at davehd_driver_blu_inl.h:473-492, is the only readable precedent).  It
       is not a texture address any more, and rounding it up would only push
       the lo plane's first texel off the row the v0 arithmetic has to land on.
       The census below still reads the two planes through
       dw_rd16(GPU_DW_SRC + kk*hi_bytes, ...), which is a raw memory read and
       not a texture lookup, so it is byte-exact either way.

       The bound is the whole texture at once - 2*rows rows of rw texels, 2
       bytes each - and it is 64 bytes TIGHTER than the two-plane check it
       replaces (op6: 154240 against 154304), still 42368 short of the slot. */
    hi_bytes = (rw * rows) * 2u;

    if (((rw * rows) * 4u) > GPU_DW_MAX_PLANE)          { s_cv_refused++; return -1; }

    /* THE CLUT ALLOCATION IS SHARED WITH THE DEPTHWISE OPERATOR, and it has to
       be.  dhd_alloc_clut() hands out from a single 512-entry pool - the
       reading that says so is gpu_env_state()'s `c4` field, which "must fail
       while gpu_dw_probe() holds all 512" - and gpu_dw_conv() asks for and
       KEEPS all 512 from its first layer on.  A request from here would
       therefore fail, and this operator would decline every layer of every
       inference with nothing in the log to say why.

       Overlapping the depthwise kernels is safe for a reason that is checkable
       rather than lucky: gpu_dw_conv() re-uploads its WHOLE slice - one
       dhd_set_clut() for the layer, then each channel's four words again inside
       dw_pass_bc() - before any of its draws.  The pointwise kernels are four
       CLUT words each and live at GPU_CV_CLUT_A and GPU_CV_CLUT_B inside the
       bottom 12 words of that slice. */
    if (!s_dw_clut_have)
    {
        if (dhd_alloc_clut(dev, GPU_DW_CLUT_WORDS, &s_dw_clut_base) != 0u)
        {
            s_cv_refused++;
            return -1;
        }
        s_dw_clut_have = true;
    }

    /* NOTHING WRITES g_gpu_dw_prog ABOVE THE DRAW LOOP, and that is a decision
       and not an omission.  The word is what the DVP ISR's stall reporter reads
       to say "the main thread is inside a GPU draw and the GPU has not moved for
       a second" - and then, after eight, to reset the GPU.  A refusal in pass 2
       runs entirely on the CPU and can leave the word set at a stage no draw
       ever reached; the reporter would then fire during the next layer's
       ordinary CPU work and reset a GPU that was never stuck.  So the stages
       below start at the first draw, and every path that refuses before then
       leaves the word exactly as it found it, which is 0. */

    /* --- pass 1: pack both planes, and gather the sums the gains need -------

       One 16-bit store per plane per activation, into uncached Device memory,
       which is the operator's real CPU cost and is counted rather than hidden.
       `code = a + 128` is the input's unsigned code and it cannot clip whatever
       in_zp is; 8*(code>>3) + (code&7) == code is the identity the two planes
       rest on.  The four accumulators are per CHANNEL, which is what makes the
       gain free later. */
    for (ci = 0u; ci < cin32; ci++)
    {
        s_cv_t5[ci] = 0u; s_cv_ex[ci] = 0u;
        s_cv_r [ci] = 0u; s_cv_rq[ci] = 0u;
        /* The extremes start INVERTED - max below min - so that a channel the
           walk never reaches keeps the WIDEST spread its field can express:
           `(mx - mn)^2` is 31^2 for the hi plane and 7^2 for the lo one, which
           is the largest either can be.  A wide spread makes a wide window, and
           a wide window refuses the layer rather than writing a plane on
           nothing.  The first real texel fixes both, because the two compares
           below are unconditional. */
        s_cv_hmn[ci] = 31u; s_cv_hmx[ci] = 0u;
        s_cv_lmn[ci] =  7u; s_cv_lmx[ci] = 0u;
    }

    /* The lo plane starts one WHOLE PLANE after the hi plane's first texel, and
       the plane is rw*rows texels because that is what dhd_set_texture_size()
       is told below - the pitch is rw, so the hi plane's extent is rw*rows*2
       bytes even though only G*cin of each row's rw texels hold data.

       `np * cin32 * 2u` is not that number: it is rw*rows*2 minus
       (rw - G*cin)*rows*2 = 4*rows bytes, so the two planes OVERLAPPED and the
       lo writes clobbered the hi plane's last few rows.  That is what the two
       guard texels buy beyond the window's alignment: 4*rows bytes of daylight
       between the planes, and it is why this number is rw*rows*2 and not
       np*cin*2.

       hi_bytes is computed in the PROLOGUE and not here, because the capacity
       check needs it.  It is now exactly rw*rows*2 - the two planes are the two
       halves of ONE texture, hi in the first `rows` rows and lo in the second,
       so the byte between them is a plane-length and nothing else.  The second
       store below is what puts the lo texels there; `hi_bytes` is the same
       number, and the draw reaches them by v0 rather than by an address. */

    for (y = 0u; y < rows; y++)
    {
        uint32_t idx = (y * rw) + 1u;      /* texel 0 of every row is a guard */

        for (x = 0u; x < G; x++)
        {
            const int8_t* src = in + (((y * G) + x) * cin32);

            for (ci = 0u; ci < cin32; ci++, idx++)
            {
                uint8_t  code = (uint8_t)((int32_t)src[ci] + 128);
                uint32_t t5   = (uint32_t)(code >> 3);
                uint32_t rr   = (uint32_t)(code & 7u);

                dw_wr16(GPU_DW_SRC, idx, (uint16_t)t5);
                /* THE LO TEXEL IS `rows` ROWS DOWN THE SAME PLANE - texel
                   rows*rw + idx, which is byte hi_bytes + idx*2.  Written as a
                   texel index so that what the draw's v0 = 0.5 lands on is
                   visible here, at the store. */
                dw_wr16(GPU_DW_SRC, (rows * rw) + idx, (uint16_t)rr);

                s_cv_t5[ci] += t5;
                s_cv_ex[ci] += (t5 >> 2);
                s_cv_r [ci] += rr;
                s_cv_rq[ci] += (rr >> 2);

                /* THE EXTREMES, and they are four compares on values already in
                   registers - no multiply, no divide, no float, and one byte per
                   side per channel.  This is the whole cost of the per-unit bias
                   split, which is why it is affordable on the one pass that also
                   writes the planes. */
                if (t5 < s_cv_hmn[ci]) { s_cv_hmn[ci] = (uint8_t)t5; }
                if (t5 > s_cv_hmx[ci]) { s_cv_hmx[ci] = (uint8_t)t5; }
                if (rr < s_cv_lmn[ci]) { s_cv_lmn[ci] = (uint8_t)rr; }
                if (rr > s_cv_lmx[ci]) { s_cv_lmx[ci] = (uint8_t)rr; }
            }
        }
    }

    s_cv_packed += np * cin32;

    /* The plane means, over every pixel of every channel, as the emulator takes
       them.  t5bar is the mean of the hi plane as the KERNEL sees it, i.e. of
       the replicated value, which is why the excess is divided by 8 and not
       added raw. */
    {
        float st5 = 0.0f, sex = 0.0f, srr = 0.0f, srq = 0.0f;
        float n   = (float)(np * cin32);

        for (ci = 0u; ci < cin32; ci++)
        {
            st5 += (float)s_cv_t5[ci];
            sex += (float)s_cv_ex[ci];
            srr += (float)s_cv_r [ci];
            srq += (float)s_cv_rq[ci];
        }
        t5bar = (st5 + (sex * 0.125f)) / n;
        rbar  = (srr + (srq * 0.125f)) / n;
    }

    /* --- pass 2: the arithmetic, checked for EVERY (co, chunk) before a draw --

       A refusal on the sixtieth output channel would otherwise leave a tensor
       that is sixty channels of GPU result, which is worse than not trying - and
       for this operator it is worse still, because the fold rewrites `out` per
       channel as it goes (see the contract in gpu.h).  Nothing is stored here:
       pass 3 recomputes exactly the same values through the same helpers, so
       there is one implementation of each number and no array to keep in step. */
    for (co = 0u; co < cout32; co++)
    {
        float m = ((float)in_scale[0] * w_scale[co]) / out_scale;

        if (!(m > 0.0f))                                { s_cv_refused++; return -1; }

        for (k = 0u; k < K; k++)
        {
            uint32_t c0 = k * GPU_CV_CHUNK;
            uint32_t ck = c0 + GPU_CV_CHUNK;
            float    g, inv, asc;
            float    wsk[2];
            float    varh[2], varl[2];
            int32_t  b[2];

            if (ck > cin32) { ck = cin32; }
            /* The two halves separately, because the biases are per texture unit
               and a unit whose own `wsk_u` is far from the chunk's is exactly the
               unit the symmetric split used to sink.  Written as one loop over
               the unit index so that the split cannot drift from the one pass 3
               fills `s_cv_kern[u][]` with - both are `c0 + u*GPU_CV_TAPS`.  The
               same loop carries the spread, and it MUST be bit-for-bit the same
               arithmetic as pass 3's: `asc` and `b` are recomputed there and a
               drift between the two would make the pre-flight check a check of
               something else. */
            wsk[0] = 0.0f; wsk[1] = 0.0f;
            varh[0] = 0.0f; varh[1] = 0.0f;
            varl[0] = 0.0f; varl[1] = 0.0f;
            for (ci = c0; ci < ck; ci++)
            {
                const float w  = (float)wq[(co * cin32) + ci];
                const uint32_t u2 = ((ci - c0) < GPU_CV_TAPS) ? 0u : 1u;
                const float dh = cv_rep5(s_cv_hmx[ci]) - cv_rep5(s_cv_hmn[ci]);
                const float dl = cv_rep5(s_cv_lmx[ci]) - cv_rep5(s_cv_lmn[ci]);
                const float w2 = w * w * (1.0f / 12.0f);

                wsk[u2]  += w;
                varh[u2] += w2 * dh * dh;
                varl[u2] += w2 * dl * dl;
            }

            g   = cv_gain(wq, cin32, co, c0, ck);
            inv = 1.0f / (1.0f + g);

            /* Nothing has been drawn yet and nothing will be if either of these
               fails, so this return needs no state put back and no progress word
               cleared - see the note above pass 1. */
            /* `-2` AND NOT `-1`, because this is the offline table's `bwall` and
               it is a different KIND of refusal from every other one in this
               function.  A capacity refusal is about this DRAW - too many
               chunks for the scratch plane, a scale outside its field, a drain
               that timed out - and a wall is about the LAYER: a centring the
               nine-bit field cannot hold is not a numerical difference, and no
               choice of chunk or a_scale recovers it, which is the offline
               table's own words for it.  The glue prints the reason on its
               `chk bail` line, and one word for both makes a layer the mirror
               REJECTS look in the log like one that merely did not fit. */
            if (!cv_digit(m, t5bar, wsk, varh, 256.0f, 8.0f, inv, &asc, b))
                                                        { s_cv_refused++; return -2; }
            if (!cv_digit(m, rbar,  wsk, varl,  32.0f, 1.0f, 1.0f, &asc, b))
                                                        { s_cv_refused++; return -2; }
        }
    }

    /* --- pass 3: the draws and the fold, one output channel at a time --------

       THE TWO HALVES OF THIS LOOP MUST AGREE ABOUT WHERE THE DIGIT PLANES ARE,
       and for one flash round they did not.  The fold reads planes 0..2K-1 of
       the destination slot, always - it has no `co` term, because the planes it
       reads are the ones the PREVIOUS few lines just drew.  The draws therefore
       have to REUSE those same 2K planes for every output channel, and the
       drain below is what makes that safe: it commits this channel's draws
       before the fold reads them and before the next channel overwrites them.

       What the draws said instead was `GPU_DW_DST + (co*cw + 2k)*np`, one
       2K-plane block per output channel - a layout in which the fold reads
       channel 0's digits for every channel of the layer.  It is not a subtle
       error either: tools/conv_gpu_emulate.py's firmware mirror runs both
       addressings on the layer's real activations, and the as-written one is
       wrong on 52 of 52 layers (worst byte error 14..255, and 12..7570 of the
       8192 sampled bytes past CV_GAP_MAX) while the reuse layout is clean on 49
       of 52 - the three it still fails are the offline arithmetic walls
       op7/op20/op182, which nothing about addressing can fix.

       It was also the wedge.  The two layouts are not the same size: the
       per-channel one needs `cout*2K*np` bytes against a 192 KB slot, which is
       2400 KB for op1 and past the slot for 5 of the first 40 layers, and this
       operator's guard only ever checked the 2K*np the reuse layout needs.  So
       the later channels of a layer wrote megabytes past GPU_DW_DST into
       whatever follows it, and the GPU stopped answering partway through op64
       on the first flipped run.  `dst KB perco/reuse` in the mirror's table is
       the column that says which layers those are.

       (cout = 96 would be 230400 bytes per layer at K = 3.  It does not fit -
       which is why the fold is per channel and the planes are reused, and not
       the other way round.) */
    for (co = 0u; co < cout32; co++)
    {
        float   m    = ((float)in_scale[0] * w_scale[co]) / out_scale;
        float   sumw = cv_sumw(wq, cin32, co);
        float   net  = (m * ((float)biasq[co] -
                             ((float)(128 + in_zp) * sumw))) +
                       (float)out_zp + 128.0f;
        float   tot0;
        int32_t bsum = 0;
        bool    ok   = true;    /* every draw of THIS channel was accepted */

        /* The FIRST write of the progress word in this function, and it is
           deliberately the first moment a draw can be outstanding. */
        g_gpu_dw_prog = DW_PROG(40u, co, 0u);

        /* A 64-BYTE SENTINEL WINDOW AT THE HEAD OF EACH OF THIS CHANNEL'S
           PLANES, for the first channel only, and the read-back that matches it.

           The census in the dig below reads plane 0 and plane 1 and the
           ambiguity in its answer is the whole reason five rounds of probes
           exist: `lo plane mn0 mx0 su0` is either a plane this layer's lo draw
           failed to write, or a plane a previous layer left zeros in.  A
           sentinel removes the ambiguity at the source instead of around it.

           IT WAS THE WHOLE PLANE LAST ROUND AND IT IS 64 BYTES NOW.  The cost is
           why: one byte write per address on this part, 2*K*pstride of them per
           layer, is 333 KB of stores over the model - and the count that read it
           back was another 272 KB of loads.  The `own` line below reads exactly
           this window, so a whole draw that never lands is still caught, which
           is the only thing the sentinel was ever for.  A draw that lands and
           covers 64 bytes of a plane and then stops is not a failure mode
           anything has ever seen.

           And the 64 bytes and not the whole plane is also what keeps `chk`
           readable: channel 0's fold output is wrong wherever a draw was
           dropped, and 64 bytes of a 1600-byte plane is a rounding error in the
           comparison rather than the 8192-pixel distortion last round's fill
           produced. */
        if (co == 0u)
        {
            uint32_t q, kk2;

            for (kk2 = 0u; kk2 < (2u * K); kk2++)
            {
                for (q = 0u; q < 64u; q++)
                {
                    dw_wr(GPU_DW_DST + (kk2 * pstride), q, 0xEEu);
                }
            }
        }

        for (k = 0u; k < K; k++)
        {
            uint32_t c0 = k * GPU_CV_CHUNK;
            uint32_t ck = c0 + GPU_CV_CHUNK;
            float    g, inv, asc_hi, asc_lo, awsk = 0.0f;
            float    wsk[2];
            float    varh[2], varl[2];
            int32_t  b_hi[2], b_lo[2];

            if (ck > cin32) { ck = cin32; }

            /* `wsk[]` is the SIGNED sum per TEXTURE UNIT and it is what each
               unit's own centring is a product of - see cv_digit(), where the
               symmetric split that read one number for the whole chunk is
               withdrawn.  `awsk` is the sum of the ABSOLUTE values over the
               whole chunk, which nothing in the operator uses and everything in
               the error model does: the multiplicand reaches the hardware as a
               5-bit texel, so a chunk's output error is `asc * (texel
               quantisation error) * SUM|w|` and the failing set has to be held
               against that product before the search moves on to anything more
               exotic.  On the shipped path and not behind a flag, because an
               accumulator that only exists when a flag is set is an accumulator
               that has never been compiled.

               `varh[]`/`varl[]` are the OTHER half of cv_digit()'s input: the
               unit's own spread, `SUM w^2 * var(C_i)`.  `var(C_i)` is the
               uniform proxy `(max - min)^2 / 12` over the per-channel extremes
               the packing pass recorded - measured as good as the exact
               variance and one multiply cheaper - and the cross terms between
               channels are dropped, which is the single approximation here.
               The extremes are of the REPLICATED field, because that is what
               the kernel really multiplies. */
            wsk[0] = 0.0f; wsk[1] = 0.0f;
            varh[0] = 0.0f; varh[1] = 0.0f;
            varl[0] = 0.0f; varl[1] = 0.0f;
            for (ci = c0; ci < ck; ci++)
            {
                const float w  = (float)wq[(co * cin32) + ci];
                const uint32_t u2 = ((ci - c0) < GPU_CV_TAPS) ? 0u : 1u;
                const float dh = cv_rep5(s_cv_hmx[ci]) - cv_rep5(s_cv_hmn[ci]);
                const float dl = cv_rep5(s_cv_lmx[ci]) - cv_rep5(s_cv_lmn[ci]);
                const float w2 = w * w * (1.0f / 12.0f);

                wsk[u2]  += w;
                varh[u2] += w2 * dh * dh;
                varl[u2] += w2 * dl * dl;
                awsk += (w < 0.0f) ? -w : w;
            }

            /* This chunk's two kernels.  The last chunk of a Cin that is not a
               multiple of 32 is ZERO-PADDED: the taps past Cin read real
               neighbouring texels, and multiplying them by 0 is what makes that
               harmless rather than something to avoid. */
            for (u = 0u; u < 2u; u++)
            {
                for (t = 0u; t < GPU_CV_TAPS; t++)
                {
                    uint32_t c = c0 + (u * GPU_CV_TAPS) + t;

                    s_cv_kern[u][t] = (c < cin32) ? wq[(co * cin32) + c] : 0;
                }
            }

            g   = cv_gain(wq, cin32, co, c0, ck);
            inv = 1.0f / (1.0f + g);

            /* Pass 2 has already proved both of these succeed; the casts are
               here so the compiler does not have to believe that. */
            (void)cv_digit(m, t5bar, wsk, varh, 256.0f, 8.0f, inv,  &asc_hi, b_hi);
            (void)cv_digit(m, rbar,  wsk, varl,  32.0f, 1.0f, 1.0f, &asc_lo, b_lo);

            /* THE ONE NUMBER THE FAILING FAMILIES DIFFER BY, and it is neither
               of the two the census already prints.

               The board's `rng` lines split the failing layers into a saturation
               family (n1 n3 n5 n7 n9 n43: the GPU range is the CPU's widened by
               some factor, `sat g` in the thousands) and a family with NO
               saturation anywhere (n39 n40 n42 n46 n49 n50, `sat c0 g0`, a
               handful of bad pixels).  A factor on a plane that is otherwise
               centred is a SCALE, and the only scale in this operator that can
               move by several-fold per layer is the replication-gain correction
               `1/(1+g)` - because cv_gain() returns exactly 0 outside [0, 0.11]
               and a chunk whose weighted sum is near cancellation has a g far
               outside it.  When that happens the hardware's multiplicand,
               `c + (c>>2)/8`, keeps its (1+g) and nothing divides it out, so the
               hi plane comes out (1+g) times too large - several-fold, in either
               direction, per layer.  The offline model has the SAME window but
               computes g from 21.jpg's activations while the board computes it
               from the first camera frame's, so one side can correct while the
               other declines to, which is a per-frame, per-layer scale error and
               is exactly the shape the board reports.

               So four numbers per (layer, chunk): `g` as cv_gain() RETURNS it,
               `raw` as it was before the window test - the two differ by the
               whole question, since `inv == 1000` with a large `raw` is the
               declined correction and `inv == 1000` with `raw` ~ 0 is a chunk
               that genuinely needs none - and then each plane's effective
               multiplier, which is asc/32 because that is literally what the
               digit multiplies by.

               `mult` is NOT the offline table's `a_scale` column; that column
               holds asc itself, so it is 32x this.  Worth writing down: 256*M
               in the table == mult 8000 here for the hi plane.  Printed as
               integers over a /1000 scale because this project has no float
               printf on the target. */
            /* cv_dmp opens this to EVERY channel, where the census is channel 0
               only.  The head's per-(co, chunk) constants are the exact
               quantity the board and the oracle must agree on, there are only
               K*cout of them (n47: 9), and a dump that showed one channel's
               would leave the obvious next question - "is it channel 0 or all
               of them?" - for another flash round. */
            if (cv_dmp || (s_cv_census && ((int)co == s_cv_census_co)))
            {
                /* TWO LINES AND NOT ONE, because `Message` is 64 bytes and the
                   per-unit fields do not fit on the end of the old one - the
                   compiler says so, and it is right: the old format was already
                   most of the buffer.  Keeping the first line byte-for-byte what
                   it always was also means every parser written before the
                   per-unit split still reads it unchanged.

                   `wsk` on it stays the CHUNK's signed sum; `b` stays each
                   plane's TOTAL, which is what the fold subtracts and what a
                   plane dump is comparable against.  The per-unit halves get
                   their own line: `wu` is the pair of weight sums the two
                   centrings are products of, and `bu` is all four bias fields,
                   hi's pair then lo's. */
                sprintf(Message,
                        "gpu cv dig n%d c%u s k%u g%d raw%d inv%d wsk%d sw%d mult %d %d b %d %d\r\n",
                        s_cv_census_n, (unsigned)co, (unsigned)k,
                        (int)(g * 1000.0f),
                        (int)(cv_gain_raw(wq, cin32, co, c0, ck) * 1000.0f),
                        (int)(inv * 1000.0f),
                        (int)((wsk[0] + wsk[1]) * 10.0f),
                        (int)(awsk * 10.0f),
                        (int)((asc_hi / 32.0f) * 1000.0f),
                        (int)((asc_lo / 32.0f) * 1000.0f),
                        (int)(b_hi[0] + b_hi[1]), (int)(b_lo[0] + b_lo[1]));
                UART_SendString(Message);
                sprintf(Message, "gpu cv dig n%d c%u s k%u u wu %d %d bu %d %d %d %d\r\n",
                        s_cv_census_n, (unsigned)co, (unsigned)k,
                        (int)(wsk[0] * 10.0f), (int)(wsk[1] * 10.0f),
                        (int)b_hi[0], (int)b_hi[1], (int)b_lo[0], (int)b_lo[1]);
                UART_SendString(Message);
            }

            /* The bias is summed over the UNITS and not doubled, because the ADD
               applies each unit's bias in that unit - and since the split is no
               longer symmetric, `2*b` is not the pair's total any more.  All four
               fields come out of the SAME helper pass 2 used, so what is
               subtracted here is literally what was programmed - not a
               re-derivation of it that could drift. */
            bsum += (b_hi[0] + b_hi[1]) + (b_lo[0] + b_lo[1]);

            /* The four bias fields of THIS chunk's two units, kept as the
               running minimum of |b| over the whole layer.  The folds' `rail`
               cannot see an annihilated unit (it is a FLOOR event, not a
               ceiling one) and the byte test cannot see one whose cost `b_u`
               is within its own tolerance - so this is the only place the
               layer's blindness to its own worst fault is measurable at all.
               See s_cv_bu_min. */
            {
                const int32_t bu[4] = { b_hi[0], b_hi[1], b_lo[0], b_lo[1] };
                int           bi;
                for (bi = 0; bi < 4; bi++)
                {
                    const int32_t a = (bu[bi] < 0) ? -bu[bi] : bu[bi];
                    if ((s_cv_bu_min < 0) || (a < s_cv_bu_min)) { s_cv_bu_min = a; }
                }
            }

            g_gpu_dw_prog = DW_PROG(41u, co, k);
            ok = cv_pass(dev, GPU_DW_SRC, GPU_DW_DST + ((2u * k) * pstride),
                         G, rows, cin32, c0, asc_hi, b_hi, 0.0f);
            s_cv_draws++;

            /* SAME SOURCE, SAME TEXTURE, THE OTHER HALF OF IT.  v0 = 0.5 is
               the lo plane's first row - see cv_pass().  The destination is
               still its own plane, so the fold below is unchanged. */
            g_gpu_dw_prog = DW_PROG(42u, co, k);
            ok = cv_pass(dev, GPU_DW_SRC,
                         GPU_DW_DST + (((2u * k) + 1u) * pstride),
                         G, rows, cin32, c0, asc_lo, b_lo, 0.5f) && ok;
            s_cv_draws++;

            if (!ok)
            {
                /* The draw was refused - cv_pass() says why on its own line.
                   Nothing here is a partial answer that can be trusted: the
                   fold below would read planes this channel never wrote.

                   The drain is not cosmetic.  Only the SECOND address can be
                   the refused one - the first plane of a layer is at
                   GPU_DW_DST + 0 - so the hi draw of this chunk is normally
                   already committed when the lo address is rejected, and it is
                   still reading GPU_DW_SRC.  The next layer's pack overwrites
                   that.  A drain that itself fails latches the dead flag, which
                   is the same refusal this return is. */
                (void)gpu_drain(dev);
                g_gpu_dw_prog = 0u;
                dhd_set_texture_unit_enable(dev, 1u);
                cv_color_setup(dev, E_DHD_OP_COPY);
                gpu_sb_reset();
                s_cv_refused++;
                return -1;
            }
        }

        /* Every draw of this channel is committed now, so this is what says the
           GPU is done with the digit planes - and the fold below reads them. */
        g_gpu_dw_prog = DW_PROG(43u, co, 0u);

        if (!gpu_drain(dev))
        {
            /* Latch and refuse: a job that has not finished may still be holding
               the source plane, and continuing would turn a refusal into a wrong
               answer.  The state the driver's ambient users read is put back
               FIRST, because it has already been changed by the draws that did
               run and the front end reads it on the very next frame. */
            s_cv_gpu_dead = true;
            s_cv_refused++;

            dhd_set_texture_unit_enable(dev, 1u);
            cv_color_setup(dev, E_DHD_OP_COPY);
            gpu_sb_reset();
            g_gpu_dw_prog = 0u;
            return -1;
        }

        /* READ THIS CHANNEL'S OWN PLANES BACK, on the production path, before
           anything else can touch them AND before the fold reads them.

           This is the only reading in the build that is about the layer's own
           draw rather than a probe's, and the sentinel above is what makes it
           decisive: `r0` and `r1` are the bytes of plane 0 and plane 1 still
           holding 238 after this channel's two draws and its drain.  n is the
           extent the fold reads, so "wrote" is n - r.

           It is printed BEFORE the dig block on purpose.  The dig then draws
           into plane 0 itself (cv_run(), which hardcodes GPU_CV_DST), which is
           why the census's own `hi plane` line has never been about the
           production hi draw alone. */
        if (co == 0u)
        {
            /* `q` belongs to THIS block and not to the one below: the cv_dmp hex
               dump further down still loops on it, so it stays out here while
               only the sentinel's own two counters move inside the guard. */
            uint32_t q;

            /* AND ONLY WHILE THE PER-LAYER CHECK IS RUNNING.  This is the gate
               that was missing, not a change to what the check prints.

               `s_cv_census` is armed by conv_run_check's gpu_cv_census(1, 0, n)
               immediately before its draw and cleared immediately after it; the
               PRODUCTION draw (conv_gpu_invoke, yolo_inference.cpp:1996) never
               arms it.  So this condition is exactly "this draw is the one the
               verdict was measured on", and a capture taken before this change
               and one taken after are comparable line for line.

               Without it this printed on every inference - 25 lines a frame, one
               per offloaded layer - which is the standing healthy-path rule
               broken by about a kilobyte of UART per frame.  The read behind it
               is not free either: 25 x 128 volatile reads of the plane window
               inside the inference, for a sentinel answer that cannot change
               between two draws of one layer with one set of weights.

               The hex dump below got a guard when it was written and this one
               did not, and the comment over there says why that guard was not
               decoration.  This is the same fault, one line higher up. */
            if (s_cv_census != 0)
            {
                uint32_t r0 = 0u, r1 = 0u;

                for (q = 0u; q < 64u; q++)
                {
                    if ((uint32_t)dw_rd(GPU_DW_DST, q) == 0xEEu) { r0++; }
                    if ((uint32_t)dw_rd(GPU_DW_DST + pstride, q) == 0xEEu) { r1++; }
                }

                sprintf(Message, "gpu cv dig own hi %u lo %u of 64 (0 = wrote)\r\n",
                        (unsigned)r0, (unsigned)r1);
                UART_SendString(Message);
            }

            /* GUARDED, and the guard is not decoration: this is 2*cw*np bytes -
               3.2 KB at n1's 40x40, and the same again for every one of the ~50
               layers that get this far.  WITHOUT the guard the first build
               emitted ~166 KB of hex per boot, which is 14 s of UART inside the
               inference and buried n47 past the end of any capture.  The
               lesson is the standing one about prints on the healthy path, and
               it was learned twice: `cv_dmp` is a local that stays set for this
               whole invocation, so the ONE layer armed above is the only one
               that reaches here. */

            /* EVERY PLANE OF CHANNEL 0, AS THE FOLD WILL READ THEM.

               Here and not later on purpose, and the reason is the whole point
               of this position: the DIG block below redraws plane 0 and plane 1
               (`cv_run()`'s dst is GPU_CV_DST + {0, pstride}) for channel 0, and
               the fold at the end of this channel reads whatever is in the
               planes when it gets there.  So a dump taken after the dig is not a
               reading of the production draw at all - it is a reading of the
               dig.  `s_cv_dig_done` is a boot-wide one-shot that fires on the
               first layer to reach here, so n47 is normally clear of it; the
               header line below prints the flag anyway, because "normally" is
               not a thing this operator has been able to rely on.

               The header also carries the geometry, because the plane layout is
               NOT derivable from the byte dump alone: the fold reads
               `dw_rd(GPU_DW_DST + (k * pstride), y)` for y < np, so where plane
               k starts and how far it runs are `pstride` and `np`, and a byte
               list without them cannot be reassembled. */
            if (cv_dmp)
            {
                sprintf(Message,
                        "gpu cv dump n%d c%u cin%u cout%u np%u cw%u pstride%u digdone%d\r\n",
                        s_cv_census_n, (unsigned)co, (unsigned)cin32,
                        (unsigned)cout32, (unsigned)np, (unsigned)cw,
                        (unsigned)pstride, s_cv_dig_done);
                UART_SendString(Message);

                for (q = 0u; q < cw; q++)
                {
                    char tag[8];

                    sprintf(tag, "pl%u", (unsigned)q);
                    cv_dump_plane(tag, GPU_DW_DST + (q * pstride), np);
                }
            }
        }

        /* The census proper - see the note at gpu_cv_census().  Walked over the
           SAME extent the fold reads (`np` bytes per plane, which is what
           `dw_rd(GPU_DW_DST + (k * pstride), y)` covers), so the four numbers are
           about exactly the bytes the answer came from and not about the padded
           tail a plane may also have.

           Gated on a CHANNEL and not on channel 0, because the planes are reused
           per output channel and a failure need not be in the first one.  When
           the caller names the channel the first bad pixel was in, the planes
           read here and the `net`/`bsum` cached below are the SAME channel's.
           When the caller names channel 0 instead - and leaves it on for every
           layer - the four numbers per plane land beside the offline model's own
           census table, which is channel 0 for all 52, so every layer has an
           oracle instead of four layers.

           `n` is the caller's layer index and travels with the flag.  Without it
           a run of 51 layers' worth of these lines can only be paired by
           counting, which is exactly the kind of guessing this whole operator
           has been losing to. */
        if (s_cv_census && ((int)co == s_cv_census_co))
        {
            uint32_t pl, q2;

            for (pl = 0u; pl < cw; pl++)
            {
                uint32_t mn = 255u, mx = 0u, at0 = 0u, at255 = 0u;

                for (q2 = 0u; q2 < np; q2++)
                {
                    const uint32_t b =
                        (uint32_t)dw_rd(GPU_DW_DST + (pl * pstride), q2) & 0xFFu;

                    if (b < mn) { mn = b; }
                    if (b > mx) { mx = b; }
                    if (b == 0u)   { at0++;   }
                    if (b == 255u) { at255++; }
                }

                sprintf(Message,
                        "gpu cv dig n%d c%u pl%u mn%u mx%u at0 %u at255 %u n%u\r\n",
                        s_cv_census_n, (unsigned)co, (unsigned)pl,
                        (unsigned)mn, (unsigned)mx,
                        (unsigned)at0, (unsigned)at255, (unsigned)np);
                UART_SendString(Message);
            }
        }

        /* --- THE DIGIT DIAGNOSTIC, once per boot, on channel 0, chunk 0 -----

           WHY IT EXISTS.  Every offline check this design has ever had - the
           emulator's table and its firmware mirror - models the digits
           ARITHMETICALLY: `clip(trunc(V) + bias)`, with V the ideal chunk sum,
           and at that level 49 of 52 layers are clean at worst 4..9.  That is a
           model of the ANSWER and not of the DRAW, so it cannot see whether the
           texture units actually produce it - and the board's first real run
           says they do not: every measured layer failed with a byte error of
           104..253 against a CPU output band only ~24 wide, and two layers came
           back a constant -128 (which is what a plane of ZEROS produces).  So
           the fault is in the only thing the mirror cannot model: the draw.

           WHAT IS UNTESTED ABOUT THE DRAW.  Three things, and each gets its own
           line below.

           1. That the taps land where the firmware thinks.  The UV window is
              calibrated on Step 1's `tap16 v 123 d 0/0`, whose kernel was ALL
              ONES over a period-16 ramp - and an all-ones kernel is blind to any
              permutation of the taps AND to a shift of the window by any
              multiple of the ramp's period, so that measurement pins down far
              less than it appears to.  The `nat` profile below sweeps the
              window over five displacements and prints the worst byte error at
              each: a 0 at displacement 0 says the window is where the firmware
              put it, and a 0 at some other displacement measures the error.

           2. That tap t carries the weight the firmware wrote for it.  The
              driver's 1xN packing is `clut[i>>2] |= a_data[i] << ((i & 3) ^ 3) * 8`
              (davehd_driver_txp_inl.h:588-593), i.e. the byte for tap i lands in
              the QUAD CHANNEL (i & 3) ^ 3 of word i >> 2 - REVERSED WITHIN EVERY
              GROUP OF FOUR.  The QUAD branch packs with the same xor (line 612)
              and the depthwise 3x3 works with it, which is why the row branch is
              assumed to be the same convention; the `rev` profile is the same
              sweep with the weights permuted by that xor, so whichever line
              comes back small is the convention those two units use.

           3. That the plane the fold reads is the plane the draw wrote.  The
              census line is that: min, max, sum and the count over the whole
              plane, before any arithmetic.  `mn 0 mx 0 su 0` is a plane nobody
              wrote - and it is worth reading FIRST, because no expectation can
              match an empty plane and it is the reading that needs no model at
              all.

           Both profiles use the two-unit digit model (cv_dig2), the structure the
           emulator does not have.  The first pixel of every row is skipped: a
           window shifted by -2 reads a texel before the row's guard, and reading
           it would be reading the previous row rather than the texel the model
           means.  `n` in the census line is np, the compare count is printed with
           the profiles so the coverage is visible.

           Print only, and s_cv_dig_done latches - six lines in a whole boot. */
        if ((co == 0u) && (s_cv_dig_done == 0))
        {
            uint32_t ck = (cin32 < GPU_CV_CHUNK) ? cin32 : GPU_CV_CHUNK;
            float    m0  = (float)in_scale[0] * w_scale[0] / out_scale;
            float    wsk[2], g0, inv0, ah, al;
            float    varh[2], varl[2];
            int32_t  bh[2], bl[2];
            float    wu[2][GPU_CV_TAPS];
            uint32_t kk;

            s_cv_dig_done = 1;

            /* The spread rides along, because this probe exists to model the
               arithmetic the draw really performs and the bias is now a function
               of it - a probe using the old bias formula would compare the board
               against a model of a configuration that is no longer on the
               board, which is the one way this line could be worse than
               absent. */
            wsk[0] = 0.0f; wsk[1] = 0.0f;
            varh[0] = 0.0f; varh[1] = 0.0f;
            varl[0] = 0.0f; varl[1] = 0.0f;
            for (ci = 0u; ci < ck; ci++)
            {
                const float w  = (float)wq[ci];
                const uint32_t u2 = (ci < GPU_CV_TAPS) ? 0u : 1u;
                const float dh = cv_rep5(s_cv_hmx[ci]) - cv_rep5(s_cv_hmn[ci]);
                const float dl = cv_rep5(s_cv_lmx[ci]) - cv_rep5(s_cv_lmn[ci]);
                const float w2 = w * w * (1.0f / 12.0f);

                wsk[u2]  += w;
                varh[u2] += w2 * dh * dh;
                varl[u2] += w2 * dl * dl;
            }
            g0   = cv_gain(wq, cin32, 0u, 0u, ck);
            inv0 = 1.0f / (1.0f + g0);

            if (cv_digit(m0, t5bar, wsk, varh, 256.0f, 8.0f, inv0, &ah, bh) &&
                cv_digit(m0, rbar,  wsk, varl,  32.0f, 1.0f, 1.0f, &al, bl))
            {
                for (u = 0u; u < 2u; u++)
                {
                    for (t = 0u; t < GPU_CV_TAPS; t++)
                    {
                        uint32_t c = (u * GPU_CV_TAPS) + t;
                        wu[u][t] = (c < ck) ? (float)wq[c] : 0.0f;
                    }
                }

                for (kk = 0u; kk < 2u; kk++)        /* 0 = the hi plane, 1 = lo */
                {
                    float    asc = (kk == 0u) ? ah : al;
                    const int32_t* b1 = (kk == 0u) ? bh : bl;
                    uint32_t src = GPU_DW_SRC + ((kk == 0u) ? 0u : hi_bytes);
                    uint32_t dst = GPU_DW_DST + ((kk == 0u) ? 0u : pstride);
                    int      wn[5], wr[5];
                    uint32_t mn = 255u, mx = 0u, su = 0u, nc = 0u;
                    int      s;

                    for (s = 0; s < 5; s++) { wn[s] = 0; wr[s] = 0; }

                    /* The census: what is in the plane, before any model.

                       THIS LINE NOW HAS A PREDICTION ATTACHED, and it is the
                       sharpest one in the block.  It reads plane 0 and plane 1
                       immediately after the CHANNEL's gpu_drain() at the top of
                       this loop, which took 58168 polls on the board - and the
                       drain returns when the counter MOVES, not when the queue is
                       empty, so those 58168 polls may describe a batch that is
                       still executing.  The reading is `hi plane mn111 mx176
                       su199842` (a real digit, mean 125) against `lo plane mn0
                       mx0 su0` - a plane of exact zeros - and the second is what
                       produced the product's error shape, because the fold is
                       `byte1 + byte2 - 128` and a byte2 of 0 shifts the whole
                       int8 output range down by 128 and pins it at -128.  That is
                       exactly what the product printed: `n1 c -121..-104 g
                       -128..-35`, `n8 c -128..-113 g -128..-128`.

                       AND THE SAME BOOT DREW THE PRODUCTION LO DRAW INTO A
                       SENTINEL PLANE AND READ IT BACK WRITTEN (`nw h 1600`, `rng
                       h 120..131`) - a real lo digit, range 11 wide - but that
                       reading was taken after the DIG's drain, which took 64860
                       polls.  Same draw, same texture, same registers: zeros
                       after the short drain, a digit after the long one.

                       So if the settle phase in gpu_drain() is the answer, this
                       line changes - `lo plane` comes back as a real digit with
                       su near 128*np - and so do the `gpu cv chk` lines above it
                       in the next boot.  If it still reads `mn0 mx0 su0`, the
                       settle is wrong, the lo draw genuinely writes zeros, and
                       the fault is in the lo half of the packed texture instead.
                       Either way the reading is decisive, which is why it is
                       printed before anything else in this block. */
                    for (y = 0u; y < np; y++)
                    {
                        uint32_t b = (uint32_t)dw_rd(dst, y);

                        if (b < mn) { mn = b; }
                        if (b > mx) { mx = b; }
                        su += b;
                    }
                    sprintf(Message, "gpu cv dig %s plane mn%u mx%u su%u n%u\r\n",
                            (kk == 0u) ? "hi" : "lo",
                            (unsigned)mn, (unsigned)mx, (unsigned)su, (unsigned)np);
                    UART_SendString(Message);

                    for (y = 0u; y < np; y++)
                    {
                        uint32_t yy = y / G;
                        uint32_t xx = y - (yy * G);
                        uint32_t tx = (yy * rw) + 1u + (xx * cin32);
                        float    v [2][GPU_CV_TAPS + 4u];   /* texels base-2..base+17 */
                        uint32_t got;
                        int      sh, uu, tt;

                        if (xx == 0u) { continue; }         /* see the note above */

                        /* Every texel any of the ten models needs, read once -
                           the window shifts then cost arithmetic, not reads. */
                        for (uu = 0u; uu < 2u; uu++)
                        {
                            for (tt = 0; tt < (int)GPU_CV_TAPS + 4; tt++)
                            {
                                uint32_t q = dw_rd16(src, tx + (uu * GPU_CV_TAPS)
                                                          + (uint32_t)tt - 2u);

                                v[uu][tt] = (float)((q << 3) | (q >> 2)) * 0.125f;
                            }
                        }

                        got = (uint32_t)dw_rd(dst, y);
                        nc++;

                        for (sh = 0; sh < 5; sh++)
                        {
                            float  a2[2], r2[2];
                            int    dd;

                            for (uu = 0u; uu < 2u; uu++)
                            {
                                float an = 0.0f, ar = 0.0f;

                                for (tt = 0; tt < (int)GPU_CV_TAPS; tt++)
                                {
                                    float rp = v[uu][sh + tt];

                                    an += wu[uu][tt]       * rp;
                                    ar += wu[uu][tt ^ 3]   * rp;
                                }
                                a2[uu] = an; r2[uu] = ar;
                            }

                            dd = (int)got - (int)cv_dig2(a2, asc, b1);
                            if (dd < 0) { dd = -dd; }
                            if (dd > wn[sh]) { wn[sh] = dd; }

                            dd = (int)got - (int)cv_dig2(r2, asc, b1);
                            if (dd < 0) { dd = -dd; }
                            if (dd > wr[sh]) { wr[sh] = dd; }
                        }
                    }

                    sprintf(Message,
                            "gpu cv dig %s nat %d %d %d %d %d n%u\r\n",
                            (kk == 0u) ? "hi" : "lo",
                            wn[0], wn[1], wn[2], wn[3], wn[4], (unsigned)nc);
                    UART_SendString(Message);
                    sprintf(Message,
                            "gpu cv dig %s rev %d %d %d %d %d n%u\r\n",
                            (kk == 0u) ? "hi" : "lo",
                            wr[0], wr[1], wr[2], wr[3], wr[4], (unsigned)nc);
                    UART_SendString(Message);
                }

                /* --- THE GEOMETRY ROUND, once per boot, same diagnostic --------

                   WHY A CENSUS CANNOT ANSWER THIS ON ITS OWN, in this file's own
                   words, from the depthwise operator's four rounds of it: "a
                   destination nothing wrote to looks exactly like one holding a
                   plausible number".  The census above is the reading WITHOUT a
                   sentinel, so it cannot say whether the digits it printed were
                   drawn or were simply already there.

                   WHAT THE FIRST SENTINEL ROUND SETTLED, and it cost two builds
                   to learn: reading the planes immediately after the draws and
                   before any dhd_flush() reads the COMMAND STREAM, not the
                   hardware - only dhd_flush() commits a draw, and
                   dhd_wait(WRITEBACK) is a pipeline-busy wait that an uncommitted
                   job leaves satisfied.  `hi 1600 1600 lo 1600 1600 of 1600` was
                   an artefact of that.

                   WHAT THE REPAIRED ONE THEN SETTLED.  With a gpu_drain() before
                   the reads, plus a fifth plane that receives no draw as the
                   control that proves the read path and the 0xEE pre-fill are
                   visible, plus the plane SUM as well as the keep-count, the 2x2
                   with the crosses read `h 1600 xh 1600 l 1600 xl 1600 c 1600`
                   and every plane's sum was exactly 1600*238 - the pre-fill.  So:

                     - nothing was written by ANY of the four, with either source
                       (hi or lo), either scale/bias, or either destination
                       - hence the lo SOURCE ADDRESS, the lo PARAMS and the
                         DESTINATION ADDRESS are all exonerated at once;
                     - the hi plane's `mn107 mx176 su199861` in the census above
                       is NOT this layer's digits.  It is the depthwise operator's
                       own plane, which lives at GPU_DW_DST + 0 - the same address
                       - and sits near 128 by design: its mean is 124.9.
                     - cv_pass() has never written anything, in any configuration.

                   NOT RESOLVED, and read the paragraph below as the state of the
                   question before the last four builds, not as the answer.  The
                   sweep afterwards established that everything with a SMALL
                   framebuffer writes and everything with the production one does
                   not, and then three hypotheses were proposed and ALL THREE were
                   falsified by their own measurements: the lo draw's texture
                   address (the build that removed it drew the production hi draw
                   through a single cv_pass() and it still wrote nothing), the
                   rounded row pitches (that build's sentinel was blind - see the
                   note in the prologue), and the framebuffer's own geometry (the
                   `i`/`j`/`m` round put the production texture and UV under the
                   proven framebuffer and read 238..238 too).  What is left is the
                   texture's EXTENT, and the `s1`/`s2`/`s3` planes below stride
                   64 KB to find its edge.  See cv_pass()'s note.

                   SO THE FAULT IS IN WHAT cv_pass() DOES THAT cv_run() DOES NOT,
                   and the two differ in exactly ONE thing: GEOMETRY.  Same
                   per-unit order, same cv_color_setup(ADD), same
                   dhd_set_texture_unit_enable position, same texel format, same
                   destination format.  cv_pass() is cv_run() with the geometry
                   made a parameter:

                       cv_run()   texture 64x8 pitch 64   dest 64x8 pitch 64
                                  bbox 63,7   UV 0, 1/64, 1/8, both units equal
                                  MEASURED: writes (`2u copy 16 add 130`)
                       cv_pass()  texture 482x80 pitch 482  dest 20x80 pitch 20
                                  bbox 19,79  UV 2/482, 24/482, 1/80, unit 1 at
                                  +16/482                                WRITES
                                  NOTHING

                   THE GEOMETRY ROUND ANSWERED IT, and this is what it said.  The
                   readings were

                       gpu cv dig run 0 of 512 (0 = wrote)
                       gpu cv dig geo a 1088 b 1088 c 0
                       gpu cv dig geo d 1600 e 1600 n 1600
                       gpu cv dig gm a 210 b 204 c 171
                       gpu cv dig gm d 238 e 238 n 238

                   and, 1088 being np-512 and 238 being the untouched 0xEE fill,
                   they read:

                     run  0      cv_run() ITSELF writes at inference time.  The
                                 CONTEXT is exonerated, and with it every ambient
                                 register - the blend factors, the colour key, the
                                 active unit, the scale/bias word that cost five
                                 boots to find.  `env cvA` is now a formality.
                     a  1088     cv_run()'s geometry rebuilt by hand in cv_pass_g()
                                 writes too, so cv_pass_g()'s register SEQUENCE is
                                 the same as cv_run()'s after all
                     b  1088     with the CONV TEXTURE (482x80 pitch 482).  The
                                 texture size and pitch are exonerated
                     c  0        with the CONV FRAMEBUFFER (20x80 pitch 20, bbox
                                 19,79) - and keep 0 means all 1600 bytes of the
                                 plane were written.  The framebuffer size, its
                                 PITCH and the bbox are exonerated, which was the
                                 leading hypothesis and is now dead
                     d  1600     with the CONV UV.  NOTHING WRITTEN
                     e  1600     the production geometry.  NOTHING WRITTEN
                     n  1600     no draw - the read path and the pre-fill are
                                 visible, so the four readings above mean what
                                 they say

                   d differs from a in NOTHING BUT THE UV, and it flips from
                   writing to writing nothing.  e - the production geometry - has
                   a UV that is entirely INSIDE its texture (u 0.004..0.95, v
                   0..0.99), so this is not about sampling outside the texture,
                   and c samples far outside it in v and works.  The UV is the
                   cause, and it is not the UV's range.

                   IT IS NOT THE DRIVER REFUSING EITHER.  dhd_set_uv()
                   (davehd_draw.c:230) copies the six floats into the shadow
                   registers as raw uint32 bit patterns through a union
                   (davehd_internal.h:147-158) - no conversion, no clamp, no
                   validation beyond the unit index.  So the hardware is given
                   these numbers and the PRIMITIVE IS DROPPED, which is the same
                   class of event as the wedge this operator ends in.

                   SO THE ROUND THAT FOLLOWED SPLIT THE UV INTO ITS FOUR FIELDS,
                   AND THE ANSWER CAME BACK: NONE OF THEM STOPS A WRITE.  Every
                   plane drew through a's texture AND a's framebuffer at a's
                   destination, so all of them could only touch the first 512 bytes
                   of the plane and each one wrote keep = np-512 or nothing at all.
                   From a's UV (u0 0, dustep 0, dux 1/64, dvy 1/8), exactly one
                   field moved at a time:

                       a  = 0,      0,       1/64,    1/8       the known good
                       f1 = 2/rw,   0,       1/64,    1/8       u0 alone
                       f2 = 0,      16/rw,   1/64,    1/8       dustep alone
                       f3 = 0,      0,       cin/rw,  1/8       dux alone
                       f4 = 0,      0,       1/64,    1/rows    dvy alone
                       e  = 2/rw,   16/rw,   cin/rw,  1/rows    all four
                       n  = no draw                                     the control

                   and they read `a 1088 f1 1088 f2 1090` and `f3 1088 f4 1088 e
                   1088 n 1600`, with means `a 210 f1 210 f2 210` and `f3 215 f4
                   215 e 217 n 238`.  Every field writes, alone and together, and
                   `f2` - the one that makes the two units' windows differ, which
                   is the plan's step-1 transfer question - writes with them.

                   AND THAT FALSIFIES THE READING ABOVE IT.  `e` is the SAME
                   configuration the previous boot's `d` measured as 1600: a's
                   texture, a's framebuffer, a's box, the production UV.  It writes
                   here.  So the UV is not what stopped that draw, "the UV is the
                   cause" was wrong, and the 1600 was not reproducible - which
                   means it was state, and the earlier round's failing plane was
                   not a property of any of these four numbers.

                   THE MEANS ARE WHAT THE COUNT CANNOT SAY.  f3 215, f4 215 and e
                   217 against a's 210 are three different samplings, so the fields
                   do reach the sampler and a plane that ignored them would have
                   read a's 210.  f1 and f2 reading exactly a's mean is NOT the
                   same evidence: f1's u0 moves the window 2/482 of a texture these
                   planes set to 64 texels wide, a quarter of a texel, and f2's
                   dustep moves only one unit's window over a constant-ish run.  So
                   the counts say `wrote`, the means say `honoured`, and NEITHER
                   says the value is right - which is the one thing no round of
                   this diagnostic has ever measured.  The K-probe below is.

                   f2's 1090 against 1088 is two bytes and is left as it reads: a
                   written output byte can BE 0xEE, and with five other planes
                   landing on exactly 1088, two bytes at the sentinel in the one
                   plane whose window differs between its units is not
                   distinguishable from that.

                   The planes are the fifteen ABOVE this layer's own cw, so the fold
                   below - which reads planes 0..cw-1 - is untouched, and nothing
                   here changes what the layer does: it is about to fall back to
                   the CPU either way. */
                if (((cw + 17u) * pstride) <= GPU_DW_MAX_PLANE)
                {
                    uint32_t base = GPU_DW_DST + (cw * pstride);
                    uint32_t k[12];
                    uint32_t su[12];
                    uint32_t mn[12];
                    uint32_t mx[12];
                    uint32_t q, p;
                    cv_geom_t ab;
                    cv_geom_t ag;
                    cv_unit_t cu[2];
                    uint32_t fb_w, fb_h, fb_p;
                    uint32_t sx0, sy0, sx1, sy1, sys;
                    uint32_t kx = 0u, ix;
                    uint32_t sy_in, sy_mid, sy_s, sy_dr;

                    /* The seven sums are printed as MEANS, and it is not cosmetic:
                       np can be 102400 on a late layer, so a raw sum reaches 8
                       digits and seven of them overflow Message[64].  A mean is at
                       most 3 digits whatever the layer, and against this sentinel
                       it reads 238 for "untouched" - the value that matters. */

                    /* --- x: cv_run() ITSELF, FIRST ---------------------------------

                       THE POINT OF THIS ONE IS THAT IT VARIES NOTHING.  `a` below is
                       cv_run()'s geometry rebuilt by hand in cv_pass_g(), so a
                       failure of `a` has TWO candidate causes - the geometry, or
                       any difference between cv_pass_g()'s register sequence and
                       cv_run()'s.  This call removes the second: it is cv_run()'s
                       own code, its own hardcoded 64x8/pitch-64/both-units-identical
                       geometry, its own 0xEE pre-fill and its own bias 0.  Nothing
                       here is mine except the source, the kernel and the gain, and
                       none of those three can stop a draw from writing -
                       dhd_set_texture_scalebias() clamps an unrepresentable gain to
                       0 or to maximum rather than refusing
                       (davehd_driver_txp_inl.h:660-670), and the bias is a 9-bit
                       field that truncates.

                       THE SAME DRAW IS IN THIS LOG, TWICE, FROM BOOT: `gpu cv 2u
                       copy 16 add 130` and `2u clut 23` and `2u scale 73` are its
                       readings then.  So this is not a new measurement, it is the
                       SAME measurement moved to inference time, and that is exactly
                       the variable in question.

                       IT READ `run 0` - all 512 bytes written.  The context is
                       fine at inference time and this line is now a standing
                       control rather than an open question: if it ever comes back
                       512, the ambient state has drifted again and nothing else on
                       this screen means anything.

                       IT DRAWS INTO GPU_CV_DST, which cv_run() hardcodes and which
                       is GPU_DW_DST + 0 - plane 0 of this layer's own destination.
                       That is safe here and not an accident: this block runs at
                       co == 0 and before this layer's first draw, and the layer's
                       own draws rewrite planes 0..2K-1 before the fold at the end
                       of the channel reads them.  cv_run() also leaves the
                       framebuffer SIZE and BBOX at its 64x8 / 63,7 - it restores
                       only the format and the nsgpu framebuffer - so both are
                       saved here and put back after the drain, which is the only
                       safe place: the driver streams registers at draw time, so
                       restoring the shadow state before the flush would change
                       what gets streamed instead of what is left behind. */
                    dhd_get_framebuffer_size(dev, &fb_w, &fb_h, &fb_p);
                    dhd_get_bbox(dev, &sx0, &sy0, &sx1, &sy1, &sys);

                    cu[0].src = GPU_DW_SRC;  cu[1].src = GPU_DW_SRC;
                    cu[0].k = s_cv_kern[0];  cu[1].k = s_cv_kern[1];
                    cu[0].kw = GPU_CV_TAPS;  cu[1].kw = GPU_CV_TAPS;
                    cu[0].scale = ah;        cu[1].scale = ah;
                    cu[0].clut = s_dw_clut_base + GPU_CV_CLUT_A;
                    cu[1].clut = s_dw_clut_base + GPU_CV_CLUT_B;

                    /* THE 13th PLANE IS THE X CONTROL'S OWN.  Without this the
                       x control draws into GPU_CV_DST, which is the production
                       hi plane at co == 0 - see s_cv_run_dst's note.  The guard
                       above admits cw + 13 planes, so this is the plane just
                       past the twelve the probes use, and it is put back
                       immediately after the draw. */
                    s_cv_run_dst = base + (12u * pstride);
                    cv_run(dev, cu, 2u, E_DHD_OP_ADD, CV_MODE_DIRECT);

                    /* THE HARDWARE'S OWN COUNTER, READ AT THE FOUR CORNERS OF
                       THIS BLOCK.  Everything the twelve readings above are
                       supposed to prove rests on one assumption - that the
                       gpu_drain() at the bottom of this block commits the draws
                       above it - and that assumption has never been measured.
                       The drain returns when this register MOVES, and a draw is
                       several jobs with several draws in flight, so a move of
                       one is not a queue that is empty.

                       It is printed as four ABSOLUTE values rather than deltas
                       so that the counter can be checked for being a counter at
                       all: `in` is read before this block's first draw, `mid`
                       after the ab-class draws and g/h (10 draws), `s` after
                       s1/s2/s3 (13 draws), `dr` after the drain that commits
                       them.  If `mid - in` is not ten-ish, or if `s - mid` is
                       zero, the counts above describe the command stream and
                       not the hardware, and the whole screen is withdrawn. */
                    sy_in = GPU_STC_SYNC_ID;

                    for (p = 0u; p < 12u; p++)
                    {
                        k[p] = 0u; su[p] = 0u; mn[p] = 0xFFu; mx[p] = 0u;
                    }

                    /* TWELVE planes, and the guard above admits exactly twelve
                       (cw + 12 planes).  The drawings below stay inside their own
                       plane - g and h write G*rows = 1600 bytes of a 1664-byte
                       plane and s1..s3 write a 64x8 box, 512 - so the fill and
                       the reading cover the same twelve planes with nothing
                       spilling out of them. */
                    for (q = 0u; q < (12u * pstride); q++) { dw_wr(base, q, 0xEEu); }

                    /* Every plane below shares a's texture, a's framebuffer and
                       a's destination box, so the six readings are directly
                       comparable: keep = np-512 means that draw wrote, and
                       keep = np means it wrote nothing. */

                    /* a: a's UV, and the known-good corner of the last round. */
                    ab.tw = GPU_CV_W;  ab.th = GPU_CV_H;  ab.tp = GPU_CV_W;
                    ab.fw = GPU_CV_W;  ab.fh = GPU_CV_H;  ab.fp = GPU_CV_W;
                    ab.bx = GPU_CV_W - 1u;
                    ab.by = GPU_CV_H - 1u;
                    ab.u0 = 0.0f;
                    ab.dustep = 0.0f;
                    ab.dux = 1.0f / (float)GPU_CV_W;
                    ab.dvy = 1.0f / (float)GPU_CV_H;
                    ab.v0  = 0.0f;      /* the hi half; a..e all read it */
                    (void)cv_pass_g(dev, &ab, GPU_DW_SRC, base, ah, bh);

                    /* f1: u0 alone moves off zero. */
                    ab.u0 = 2.0f / (float)rw;
                    (void)cv_pass_g(dev, &ab, GPU_DW_SRC, base + pstride, ah, bh);

                    /* f2: dustep alone moves off zero, which is also the only
                       one of the four that makes the two UNITS' windows start at
                       different texels. */
                    ab.u0 = 0.0f;
                    ab.dustep = (float)GPU_CV_TAPS / (float)rw;
                    (void)cv_pass_g(dev, &ab, GPU_DW_SRC, base + (2u * pstride),
                                    ah, bh);

                    /* f3: dux alone. */
                    ab.dustep = 0.0f;
                    ab.dux = (float)cin32 / (float)rw;
                    (void)cv_pass_g(dev, &ab, GPU_DW_SRC, base + (3u * pstride),
                                    ah, bh);

                    /* f4: dvy alone. */
                    ab.dux = 1.0f / (float)GPU_CV_W;
                    ab.dvy = 1.0f / (float)rows;
                    (void)cv_pass_g(dev, &ab, GPU_DW_SRC, base + (4u * pstride),
                                    ah, bh);

                    /* e: all four, which is exactly what the last round's `d`
                       measured as writing nothing.  Repeated so that this boot's
                       six one-field readings sit in one commit with the
                       all-four reading they are meant to explain. */
                    ab.u0 = 2.0f / (float)rw;
                    ab.dustep = (float)GPU_CV_TAPS / (float)rw;
                    ab.dux = (float)cin32 / (float)rw;
                    ab.dvy = 1.0f / (float)rows;
                    (void)cv_pass_g(dev, &ab, GPU_DW_SRC, base + (5u * pstride),
                                    ah, bh);

                    /* g and h: THE PRODUCTION DRAWS THEMSELVES, SIDE BY SIDE, and
                       this round's whole question.

                       g is the production HI draw - cv_pass(), the function the
                       layer's own draws call, with the layer's own G, rows,
                       cin32 and c0, over the whole plane - and h is the
                       production LO draw, differing in exactly two things: the
                       gain/bias pair (`al`/`bl` against `ah`/`bh`) and v0 (0.5
                       against 0.0).  THE SOURCE ADDRESS IS NO LONGER A VARIABLE,
                       and that is the fix: both read GPU_DW_SRC + 0, because the
                       lo plane is now the second half of the hi plane's texture
                       rather than a second texture of its own.  cv_pass()'s note
                       carries the seven boots of evidence that retired the
                       second address, including the pair of probe draws that
                       differed in that one number and read 1088 (wrote) against
                       1600 (wrote nothing) in the same commit.

                       WHAT IT SAID, AND IT IS THE THIRD OF THE THREE OUTCOMES:
                       `nw g 0 h 0`, `rng g 238..238 h 238..238`, in BOTH boots of
                       the build that introduced it.  So the production draw
                       writes NOTHING - both halves, hi as well as lo - and the
                       two-draw form is not the problem either: g is a SINGLE
                       draw and it writes nothing.  The address hypothesis is
                       falsified by its own fix, and the note that follows this
                       block records that.

                       WHICH LEAVES THE TEXTURE, and the fourth hypothesis - that
                       the FRAMEBUFFER is the fault - is what i/j/m tested and
                       killed.  Each drew a sentinel-filled plane and each read
                       back the other way: `nw i 0 j 0 m 0` and `rg i 238..238
                       j 238..238 m 238..238`, in both boots of the build that
                       carried them.  i is the sharp one - the production
                       texture AND the production UV under the PROVEN
                       64x8/p64/box-63,7 framebuffer - so the framebuffer, its
                       pitch and its box are all exonerated, and the driver and
                       register reading that produced the hypothesis (only
                       cv_pass() can program a pitch that is not a multiple of
                       8, and 20 is the first such in this project) went with
                       it.  j exonerated th = 2*rows while keeping the
                       production framebuffer, and m exonerated the pitch with
                       everything else production.  Nothing was refused and the
                       drain returned, so those draws RAN and wrote nothing.

                       WHAT IS LEFT IS THE TEXTURE'S EXTENT, and this round is a
                       bracket on it rather than a sixth field.  Line every draw
                       that has ever been read back through a sentinel fill up
                       by the texture it sampled:

                         texture                       bytes    th   result
                         ab   64x8    p64              1024       8   writes
                         kv   rw x8   p rw (constant)  7712       8   writes
                         i    rw x160 p rw            154240     160   nothing
                         j    rw x80  p rw             77120      80   nothing
                         m    rw x160 p rw            154240     160   nothing

                       Everything under 8 KB has written; the two over 75 KB
                       have not.  So the bound is somewhere in (61696, 77120]
                       and the whole extent - not the UV, not the framebuffer,
                       not the pitch, not the address - is what the two sides
                       differ in.

                       THE ONE DATUM ON THE OTHER SIDE OF THAT LINE IS `b`: the
                       earlier round's 482x80 p482 draw, which is why the texture
                       was thought exonerated and the two-draw form was blamed
                       instead.  It is re-run here as s3.  If it really wrote,
                       then what separates it from j is its UV and not its size,
                       and one of the two numbers is wrong.

                       The K-probe is the same shape of measurement from the
                       other direction and it agrees: its four draws land in the
                       SAME planes 7..11 this block uses, immediately after this
                       block's drain, and three of the four wrote (`kv g 121 124
                       176`, `kv 4 ... c 0`; the fourth is the no-draw control
                       and reads 238).  A destination plane is therefore not the
                       variable either, whatever its index - and the probe's two
                       textures are 1024 and 7712 bytes.

                         s1  rw x 67 p rw   64588 B   32294 texels   dvy 1/67
                         s2  rw x 68 p rw   65552 B   32776 texels   dvy 1/68
                         s3  rw x 80 p rw   77120 B   38560 texels   dvy 1/8
                             and A'S UV - u0 0, dustep 0, dux 1/64 - and not the
                             production one.  This is the exact `b` reproduction.

                       All three draw under the PROVEN framebuffer (64x8 p64, box
                       63,7) so a refusal cannot be blamed on it, and all three
                       into a sentinel-filled plane so that 238..238 and a
                       written range cannot be confused.

                       s1 AGAINST s2 IS THE WHOLE POINT OF THE ROUND: a
                       one-and-a-half-percent straddle of 64 KB.  s1's 64588
                       bytes and 32294 texels are both under 2^16 and 2^15; s2's
                       65552 and 32776 are both over.  Writes for s1 and nothing
                       for s2 says the bound is the texture's EXTENT at 64 KB
                       exactly, and `th = 2*rows` dies with it - the same
                       failure reading 68 rows tall.  (A byte limit cannot be
                       told apart from a 2^15-texel limit this way, because the
                       texel is two bytes: 64 KB and 32768 texels are the same
                       boundary.  It does not matter, because the fix is the
                       same.)

                       s3 IS THE UV'S OWN BRACKET HELD AT th = 80, the height the
                       earlier round recorded as writing.  If s3 writes where j
                       - same th, same pitch, same box, production UV - did not,
                       then the extent is not the bound at all and the UV is,
                       over a texture tall enough for the u walk to leave the row
                       it started in.  If s3 reads 238..238 like j, then the `b`
                       reading was the depthwise layer's leftover or a
                       misattributed plane - the census at the top of this block
                       carries the note explaining how that misreading happens -
                       and the extent stands.

                       THE FIX IS THE SAME SHAPE EITHER WAY, which is why the
                       bracket is worth a round: a strip.  Draw one band of rows
                       at a time with v0 = band_row/th, or, if the bound is on
                       th, strip so no texture is taller than 64 rows.  cv_pass()
                       gained v0 for the lo plane and a strip wants exactly that
                       field, and both forms are bounded draw-count arithmetic
                       that leave the pack alone.

                       c0 here is the literal 0 and not the variable `c0`: this
                       block runs at co == 0 and before the chunk loop, so the
                       chunk it describes is k = 0.  That is also why `ab.u0`
                       above is 2/rw and not (2 + c0)/rw.

                       The pre-fill, the commit and the reading are still the ones
                       a..e use: one sentinel fill, ONE gpu_drain() for all of
                       them, and plane 6 still the one no draw ever touches. */
                    (void)cv_pass(dev, GPU_DW_SRC, base + (7u * pstride),
                                  G, rows, cin32, 0u, ah, bh, 0.0f);
                    (void)gpu_drain(dev);      /* its own commit - see cv_pass_g() */
                    (void)cv_pass(dev, GPU_DW_SRC, base + (8u * pstride),
                                  G, rows, cin32, 0u, al, bl, 0.5f);
                    (void)gpu_drain(dev);

                    /* The production geometry's TEXTURE half, rebuilt here so
                       the height can be walked without touching anything else.
                       Every field but th is cv_pass()'s own number - and the
                       framebuffer is the PROVEN one, 64x8 p64 box 63,7, so
                       that a probe reading 238..238 can only be blamed on the
                       texture.  dvy follows th, because v should still cross
                       one texture height over the box's eight rows. */
                    ag.tw = rw;      ag.tp = rw;
                    ag.fw = GPU_CV_W; ag.fh = GPU_CV_H;  ag.fp = GPU_CV_W;
                    ag.bx = GPU_CV_W - 1u;
                    ag.by = GPU_CV_H - 1u;
                    ag.u0 = 2.0f / (float)rw;
                    ag.dustep = (float)GPU_CV_TAPS / (float)rw;
                    ag.dux = (float)cin32 / (float)rw;
                    ag.v0 = 0.0f;

                    sy_mid = GPU_STC_SYNC_ID;

                    /* s1: 64588 bytes, 32294 texels - just UNDER both 64 KB and
                       2^15 texels, and 67 rows tall, which is OVER 64 rows. */
                    ag.th = 67u;
                    ag.dvy = 1.0f / 67.0f;
                    (void)cv_pass_g(dev, &ag, GPU_DW_SRC, base + (9u * pstride),
                                    ah, bh);

                    /* s2: 65552 bytes, 32776 texels - the same 1.5% straddle
                       from the other side.  s1 writes and s2 does not is the
                       answer the round is built for. */
                    ag.th = 68u;
                    ag.dvy = 1.0f / 68.0f;
                    (void)cv_pass_g(dev, &ag, GPU_DW_SRC, base + (10u * pstride),
                                    ah, bh);

                    /* s3: th = 80 - over both bounds, the height the earlier
                       round recorded as writing - but with A'S UV, which is the
                       only thing that distinguishes it from this round's j. */
                    ag.th = 80u;
                    ag.u0 = 0.0f;    ag.dustep = 0.0f;
                    ag.dux = 1.0f / (float)GPU_CV_W;
                    ag.dvy = 1.0f / 8.0f;
                    (void)cv_pass_g(dev, &ag, GPU_DW_SRC, base + (11u * pstride),
                                    ah, bh);

                    /* COMMIT, THEN READ.  Without this the counts below
                       describe the command stream, not the hardware.  A drain
                       that fails latches the dead flag inside gpu_drain()
                       itself, which is the same refusal this layer is about to
                       take anyway. */
                    sy_s  = GPU_STC_SYNC_ID;
                    (void)gpu_drain(dev);
                    sy_dr = GPU_STC_SYNC_ID;

                    for (q = 0u; q < np; q++)
                    {
                        for (p = 0u; p < 12u; p++)
                        {
                            uint32_t b = (uint32_t)dw_rd(base + (p * pstride), q);

                            if (b == 0xEEu) { k[p]++; }
                            if (b < mn[p]) { mn[p] = b; }
                            if (b > mx[p]) { mx[p] = b; }
                            su[p] += b;
                        }
                    }

                    /* x's reading comes off the SAME commit as a..e: cv_run()
                       leaves its own draw uncommitted exactly as cv_pass_g()
                       does, so the one gpu_drain() above covers both.  It reads
                       s_cv_run_dst and not GPU_CV_DST because the x control no
                       longer draws into the production hi plane. */
                    for (ix = 0u; ix < (GPU_CV_W * GPU_CV_H); ix++)
                    {
                        if ((uint32_t)dw_rd(s_cv_run_dst, ix) == GPU_CV_SENT) { kx++; }
                    }
                    s_cv_run_dst = GPU_CV_DST;

                    /* Put the framebuffer size and bbox back AFTER the drain, so
                       the restore cannot alter what was streamed.  cv_run() left
                       them at 64x8 / 63,7. */
                    dhd_set_framebuffer_size(dev, fb_w, fb_h, fb_p);
                    dhd_set_bbox(dev, sx0, sy0, sx1, sy1, sys);

                    sprintf(Message, "gpu cv dig sy %u %u %u %u\r\n",
                            (unsigned)sy_in, (unsigned)sy_mid,
                            (unsigned)sy_s,  (unsigned)sy_dr);
                    UART_SendString(Message);
                    sprintf(Message, "gpu cv dig run %u of %u (%u = wrote)\r\n",
                            (unsigned)kx, (unsigned)(GPU_CV_W * GPU_CV_H), 0u);
                    UART_SendString(Message);

                    sprintf(Message, "gpu cv dig u0 a %u f1 %u f2 %u\r\n",
                            (unsigned)k[0], (unsigned)k[1], (unsigned)k[2]);
                    UART_SendString(Message);

                    sprintf(Message, "gpu cv dig u0 f3 %u f4 %u e %u n %u\r\n",
                            (unsigned)k[3], (unsigned)k[4], (unsigned)k[5],
                            (unsigned)k[6]);
                    UART_SendString(Message);

                    sprintf(Message, "gpu cv dig um a %u f1 %u f2 %u\r\n",
                            (unsigned)(su[0] / np), (unsigned)(su[1] / np),
                            (unsigned)(su[2] / np));
                    UART_SendString(Message);

                    sprintf(Message, "gpu cv dig um f3 %u f4 %u e %u n %u\r\n",
                            (unsigned)(su[3] / np), (unsigned)(su[4] / np),
                            (unsigned)(su[5] / np), (unsigned)(su[6] / np));
                    UART_SendString(Message);

                    /* `nw` is bytes NOT left at the sentinel, so 0 is "wrote
                       nothing" and np is "wrote the whole plane".  The old form
                       of this line counted SENTINEL bytes, which was readable
                       while the box was 512 bytes of a 1600-byte plane and is
                       not any more: g and h now draw the whole plane, so the
                       sentinel count reads np whether they wrote or not. */
                    sprintf(Message, "gpu cv dig nw g %u h %u c %u\r\n",
                            (unsigned)(np - k[7]), (unsigned)(np - k[8]),
                            (unsigned)(np - k[6]));
                    UART_SendString(Message);

                    sprintf(Message, "gpu cv dig um g %u h %u\r\n",
                            (unsigned)(su[7] / np), (unsigned)(su[8] / np));
                    UART_SendString(Message);

                    /* THE LINE THAT DECIDES THIS ROUND.  An unwritten plane is
                       238..238 - the sentinel, unmixed - and a written one is
                       whatever the convolution produced.  Nothing here can
                       confuse the two. */
                    sprintf(Message, "gpu cv dig rng g %u..%u h %u..%u\r\n",
                            (unsigned)mn[7], (unsigned)mx[7],
                            (unsigned)mn[8], (unsigned)mx[8]);
                    UART_SendString(Message);

                    /* s1/s2/s3: the extent bracket.  `nw` is bytes not left at
                       the sentinel and the range is the sharp reading.  All
                       three draw a 64x8 box, so a written plane reads 512 and an
                       unwritten one 0 and 238..238 - and 238..238 is the
                       sentinel unmixed, so there is nothing to misread. */
                    sprintf(Message, "gpu cv dig nw s1 %u s2 %u s3 %u\r\n",
                            (unsigned)(np - k[9]), (unsigned)(np - k[10]),
                            (unsigned)(np - k[11]));
                    UART_SendString(Message);

                    sprintf(Message,
                            "gpu cv dig rg s1 %u..%u s2 %u..%u s3 %u..%u\r\n",
                            (unsigned)mn[9],  (unsigned)mx[9],
                            (unsigned)mn[10], (unsigned)mx[10],
                            (unsigned)mn[11], (unsigned)mx[11]);
                    UART_SendString(Message);

                    /* The gains are printed a thousand times over and the biases
                       raw, because the question the `h` plane asks is whether
                       these four numbers survived the driver's field packing -
                       and a pair of values that cannot be told apart at the
                       printed precision would not answer it. */
                    sprintf(Message, "gpu cv dig ab h %u %d l %u %d\r\n",
                            (unsigned)(ah * 1000.0f), (int)bh,
                            (unsigned)(al * 1000.0f), (int)bl);
                    UART_SendString(Message);

                    sprintf(Message, "gpu cv dig np %u (%u = wrote)\r\n",
                            (unsigned)np, (unsigned)(np - 512u));
                    UART_SendString(Message);

                    /* --- THE K-PROBE, once per boot, same diagnostic ------------

                       WHY, after the round above.  It proved a draw happens and
                       that the UV reaches the sampler.  What the CENSUS at the top
                       of this block says, identically over two boots, is the pair
                       of readings that still do not add up:

                           gpu cv dig hi plane mn111 mx176 su199842 n1600
                           gpu cv dig lo plane mn0   mx0   su0     n1600

                       The lo plane is a plane of EXACT ZEROS, from a draw that
                       returned success.  It cannot be its source: cv_dig2() gives
                       a plane 2b - the bias, added once by each unit - for ANY
                       source, the model the census compares against predicts ~130
                       for every one of those bytes, and that model reads the same
                       source with the same scale and the same bias.  A plane of
                       zeros therefore means the SUMS the hardware formed were far
                       enough below the model's to clamp to nothing in both units -
                       and a wrong weighted sum is also exactly what leaves the hi
                       plane matching NONE of the five windows the census sweeps.
                       (The census's matching shift is 2 of 0..4: `tap 0 sits on
                       texel (sample - 1)` is measured, `tx = y*rw + 1` is the
                       pack's own origin, and v[uu][k] is read from tx+16uu+k-2,
                       so the window that should be exact is index 2 - and it
                       reads 56.)

                       SO THE NEXT READING IS A NUMBER AND NOT A PRESENCE, and it
                       is built so that the number CANNOT BE MOVED by any of the
                       things still in doubt.  The texture is a CONSTANT - every
                       texel the same code - and both kernels are ALL ONES:

                         - the window's position cannot change the sum: every tap
                           of every window reads the same texel
                         - the tap-to-byte order cannot change the sum: every
                           kernel byte is 1
                         - for k1 and k3 it does not matter which unit is which
                         - the replication contract becomes a NUMBER.  The sum is
                           16 * rep(16), which is 264 if the measured contract
                           holds (((16<<3)|(16>>2))/8 = 16.5) and 256 if the
                           multiplicand is the raw five-bit field; kv_asc is 64.5
                           per unit, so 264 gives 64 and 256 gives 62 per unit -
                           two bytes apart, not one.

                       One byte per plane then measures the WHOLE arithmetic chain
                       - the replication, the tap count, the scale, the bias, the
                       two-unit ADD, the byte clamp and the write into the plane -
                       and if it reads what the firmware computes, that chain is
                       EXACT and what is left is the tap-to-texel mapping, which is
                       the only other thing the model assumes.

                         k1  both kernels 16 ones  asc 64.5/unit  b  0   -> 128
                         k2  unit 1's kernel ZERO asc 64.5/unit  b  0   ->  64
                         k3  both kernels 16 ones  asc 64.5/unit  b 20   -> 168
                         k4  k1 through the PRODUCTION texture width and the
                             production u terms (rw x 8 pitch rw, u0 2/rw, dustep
                             16/rw, dux cin/rw, a 20 x 8 destination)    -> 128
                         n2  no draw                                     -> 238

                       k2 is the one that splits the two units: 64 against 128 says
                       the ADD and unit 1's CLUT slot are both right.  k3 measures
                       the bias's sign and magnitude.  k4 is the same constant
                       through production's TEXTURE geometry, which is the one axis
                       the round above never varied while keeping a value visible.

                       The texture goes in the TAIL of the source slot, after the
                       lo plane's own extent - the one part of GPU_DW_SRC this
                       layer does not touch.  The destination planes are the five
                       above the seven the round above used. */
                    {
                        uint32_t kv_t5  = 16u;
                        uint32_t ksrc   = (hi_bytes + ((rw * rows) * 2u) + 127u)
                                          & ~127u;
                        uint32_t kneed  = 1024u + ((((rw * 8u) * 2u) + 127u)
                                                   & ~127u);
                        uint32_t kbase  = GPU_DW_DST + ((cw + 7u) * pstride);

                        if ((ksrc + kneed) <= GPU_DW_MAX_PLANE)
                        {
                            uint32_t ktex  = GPU_DW_SRC + ksrc;
                            uint32_t ktex2 = ktex + 1024u;
                            int8_t   ksav[2][GPU_CV_TAPS];
                            uint32_t ke1, ke2, ke3, kc4 = 0u;
                            uint32_t g1, g2, g3, g4, g5;
                            uint32_t i3, t3, p3;
                            float    kv_rep = (float)((kv_t5 << 3) | (kv_t5 >> 2))
                                              / 8.0f;
                            float    kv_sum = kv_rep * (float)GPU_CV_TAPS;
                            float    kv_asc = (64.5f * 32.0f) / kv_sum;
                            float    ka2[2];
                            int32_t  kb[2];
                            cv_geom_t kg;

                            for (i3 = 0u; i3 < 2u; i3++)
                            {
                                for (t3 = 0u; t3 < GPU_CV_TAPS; t3++)
                                {
                                    ksav[i3][t3]     = s_cv_kern[i3][t3];
                                    s_cv_kern[i3][t3] = 1;
                                }
                            }

                            /* The three expectations through the SAME helper the
                               census uses, so the number printed as `e` is the
                               formula the model runs and not a re-derivation. */
                            kb[0] = 0; kb[1] = 0;
                            ka2[0] = kv_sum;  ka2[1] = kv_sum;
                            ke1 = (uint32_t)cv_dig2(ka2, kv_asc, kb);
                            ka2[1] = 0.0f;
                            ke2 = (uint32_t)cv_dig2(ka2, kv_asc, kb);
                            ka2[1] = kv_sum;
                            kb[0] = 20; kb[1] = 20;
                            ke3 = (uint32_t)cv_dig2(ka2, kv_asc, kb);

                            for (p3 = 0u; p3 < 5u; p3++)
                            {
                                for (i3 = 0u; i3 < pstride; i3++)
                                {
                                    dw_wr(kbase + (p3 * pstride), i3, 0xEEu);
                                }
                            }
                            for (i3 = 0u; i3 < (64u * 8u); i3++)
                            {
                                dw_wr16(ktex, i3, (uint16_t)kv_t5);
                            }
                            for (i3 = 0u; i3 < (rw * 8u); i3++)
                            {
                                dw_wr16(ktex2, i3, (uint16_t)kv_t5);
                            }

                            /* k1..k3: a's own geometry.  Row 0, column 32 is read
                               because it is the one column whose whole window -
                               texels 31..46 - is inside the 64-wide texture. */
                            kg.tw = 64u;  kg.th = 8u;  kg.tp = 64u;
                            kg.fw = 64u;  kg.fh = 8u;  kg.fp = 64u;
                            kg.bx = 63u;  kg.by = 7u;
                            kg.u0 = 0.0f;  kg.dustep = 0.0f;
                            kg.dux = 1.0f / 64.0f;  kg.dvy = 1.0f / 8.0f;
                            kg.v0  = 0.0f;

                            (void)cv_pass_g(dev, &kg, ktex, kbase, kv_asc, cv_b0);

                            for (t3 = 0u; t3 < GPU_CV_TAPS; t3++)
                            {
                                s_cv_kern[1][t3] = 0;
                            }
                            (void)cv_pass_g(dev, &kg, ktex, kbase + pstride,
                                            kv_asc, cv_b0);

                            for (t3 = 0u; t3 < GPU_CV_TAPS; t3++)
                            {
                                s_cv_kern[1][t3] = 1;
                            }
                            (void)cv_pass_g(dev, &kg, ktex, kbase + (2u * pstride),
                                            kv_asc, cv_b20);

                            /* k4: the same constant through production's TEXTURE
                               width, pitch and u terms.  The destination is 8 rows
                               of G, so the whole window of every destination
                               column is inside the rw-wide row. */
                            kg.tw = rw;      kg.th = 8u;  kg.tp = rw;
                            kg.fw = G;       kg.fh = 8u;  kg.fp = G;
                            kg.bx = G - 1u;  kg.by = 7u;
                            kg.u0 = 2.0f / (float)rw;
                            kg.dustep = (float)GPU_CV_TAPS / (float)rw;
                            kg.dux = (float)cin32 / (float)rw;
                            kg.dvy = 1.0f / 8.0f;
                            kg.v0  = 0.0f;
                            (void)cv_pass_g(dev, &kg, ktex2,
                                            kbase + (3u * pstride), kv_asc, cv_b0);

                            /* ================= THE THREE-FIELD BISECT =============

                               WHY IT IS NEEDED, AND IT IS THE WHOLE CONTENT OF
                               THIS ROUND.  In the two cleanest boots of the
                               position-fixed build the four readings above are

                                   gpu cv dig kv e 128 64 168      <- the model
                                   gpu cv dig kv g   0  0  40      <- the board

                               and an earlier boot of the same probe read

                                   gpu cv dig kv g 121 124 176
                                   gpu cv dig kv 4 g 176 e 128 c 0

                               Read them together, because they are the SAME
                               probe, the SAME constant texture, the SAME
                               all-ones kernel and the SAME scale:

                                 k1  asc 7.818  tw 64  dest 64x8/64  read col 32 row 0  ->  0
                                 k4  asc 7.818  tw 482 dest 20x8/20  read col  5 row 0  -> 176

                               `2 * bias` is the part that cannot be in doubt: it
                               is 0 for k1 and k2 and 40 for k3 (2 x 20), and 40
                               is EXACTLY what k3 reads.  So the two-unit ADD, the
                               bias, the byte clamp and the write into the plane
                               are all measured exact.  What is left is a weighted
                               sum that is EXACTLY ZERO in k1 and 1.37x the
                               model's 264 in k4 - and the ONLY difference between
                               those two draws is the geometry: the texture's
                               width/pitch (64 vs rw) and the destination's
                               width/pitch and readout column (64, col 32 vs G,
                               col 5).

                               So this round varies ONE field per plane, off k1's
                               own configuration, and keeps the invariant that no
                               geometric doubt can move - constant texture, all
                               ones kernel, bias 0, scale 7.818 (so a dead sum
                               reads EXACTLY 0 and a live one reads 128):

                                 p5  k1, texture width and pitch rw (u 32/rw, dux 1/rw) -> 128
                                 p6  k1, read at row 4 instead of row 0                -> 128
                                 p7  k1, destination G x 8 pitch G, read col 5         -> 128

                               All three are still inside a constant texture, so
                               every prediction is the same number and any plane
                               that reads 0 names its own field as the killer.  p6
                               is not a guess: `cv_run`'s tap16 probe - the one
                               configuration in this project OBSERVED to make a
                               1x16 row kernel sum 16 consecutive texels, `tap16 v
                               123 d 0/0` - reads ROW 4, and this block reads row
                               0 everywhere. */
                            {
                                uint32_t p5 = kbase + (5u * pstride);
                                uint32_t p6 = kbase + (6u * pstride);
                                uint32_t p7 = kbase + (7u * pstride);
                                uint32_t q3;

                                for (q3 = 0u; q3 < (3u * pstride); q3++)
                                {
                                    dw_wr(kbase + (5u * pstride), q3, 0xEEu);
                                }

                                /* p5: the production texture's own width and
                                   pitch, one destination column per texel. */
                                kg.tw = rw;  kg.th = 8u;  kg.tp = rw;
                                kg.fw = 64u; kg.fh = 8u;  kg.fp = 64u;
                                kg.bx = 63u; kg.by = 7u;
                                kg.u0 = 32.0f / (float)rw;
                                kg.dustep = 0.0f;
                                kg.dux = 1.0f / (float)rw;
                                kg.dvy = 1.0f / 8.0f;
                                kg.v0  = 0.0f;
                                (void)cv_pass_g(dev, &kg, ktex, p5, kv_asc, cv_b0);

                                /* p7: k1's texture, the production destination
                                   width and pitch. */
                                kg.tw = 64u; kg.th = 8u; kg.tp = 64u;
                                kg.fw = G;   kg.fh = 8u; kg.fp = G;
                                kg.bx = G - 1u; kg.by = 7u;
                                kg.u0 = 0.0f;
                                kg.dustep = 0.0f;
                                kg.dux = 1.0f / 64.0f;
                                kg.dvy = 1.0f / 8.0f;
                                kg.v0  = 0.0f;
                                (void)cv_pass_g(dev, &kg, ktex, p7, kv_asc, cv_b0);

                                /* p6: k1 exactly - its geometry put back field by
                                   field, because p5 and p7 both moved `kg` - and
                                   it is the only one of the three whose readout
                                   differs from this block's habit rather than its
                                   draw: the working tap16 probe read row 4. */
                                kg.tw = 64u; kg.th = 8u; kg.tp = 64u;
                                kg.fw = 64u; kg.fh = 8u; kg.fp = 64u;
                                kg.bx = 63u; kg.by = 7u;
                                kg.u0 = 0.0f;
                                kg.dustep = 0.0f;
                                kg.dux = 1.0f / 64.0f;
                                kg.dvy = 1.0f / 8.0f;
                                kg.v0  = 0.0f;
                                (void)cv_pass_g(dev, &kg, ktex, p6, kv_asc, cv_b0);

                                /* ============= THE V RAMP: WHICH TEXEL-ROW
                                   DOES DESTINATION ROW r SAMPLE? =============

                                   WHY THIS AND NOT A FIX.  x5/x6/x7 above say
                                   one destination row is dead and it is the row
                                   whose v lands on the texture's v border - but
                                   they cannot say WHICH texel-row the hardware
                                   means by v, and cv_pass()'s own v term

                                       g.v0 = v0;                    (no inset)
                                       g.u0 = (2 + c0) / rw;         (2-texel inset)

                                   carries no inset where u carries one, so
                                   there is nothing to copy the correction from -
                                   and the u side's own +2 has not been re-derived
                                   from the driver, so it is a number this project
                                   inherited rather than one it can argue.  That is
                                   exactly why the v side gets measured instead of
                                   argued: read the ladder, not the source.

                                   The texture is a v ramp - every texel of row y
                                   holds (y + 2) in the blue field - so a
                                   destination row's reading names the texel-row
                                   it actually sampled.  asc = 16 with two units
                                   and bias 0 gives

                                       plane = 2 * trunc(16*(y+2) * 16 / 32)
                                             = 16 * (y + 2)

                                   so texel-row 0..7 read 32,48,...,144 and a DEAD
                                   row reads 0.  That offset is the whole point:
                                   with a plain y ramp a dead row and texel-row 0
                                   would both read 0 and the probe could not tell
                                   "sampled row 0" from "sampled nothing".

                                   Under `tap at v*th - 1` (the u convention, one
                                   texel back) destination rows 0..7 read
                                   0,32,48,64,80,96,112,128 - row 7 sampling the
                                   sixth texel-row, leaving the last one unused.
                                   Under `tap at floor(v*th)` they read
                                   32,48,64,80,96,112,128,144 and row 0 is not
                                   dead at all, which x5 already rules out.  Under
                                   a half-texel anchor the interior values are
                                   blends and land between those two ladders.
                                   Row 0 is therefore the discriminator, and the
                                   ladder's LENGTH says how many texel-rows the
                                   eight destination rows cover. */
                                {
                                    cv_geom_t vg;
                                    uint32_t  q3;

                                    for (q3 = 0u; q3 < (32u * 8u); q3++)
                                    {
                                        dw_wr16(ktex, q3,
                                                (uint16_t)((q3 / 32u) + 2u));
                                    }

                                    vg.tw = 32u;  vg.th = 8u;  vg.tp = 32u;
                                    vg.fw = 32u;  vg.fh = 8u;  vg.fp = 32u;
                                    vg.bx = 31u;  vg.by = 7u;
                                    vg.u0 = 4.0f / 32.0f;
                                    vg.dustep = 0.0f;
                                    vg.dux = 1.0f / 32.0f;
                                    vg.dvy = 1.0f / 8.0f;
                                    /* v0 carries the fix that was just put into
                                       cv_pass() (g.v0 = v0 + g.dvy), so this draw
                                       is the fix's own confirmation: every row is
                                       now live and the ladder must come out
                                           32 48 66 82 98 114 132 148
                                       with no zero anywhere.  If a 0 survives,
                                       the fix is wrong, not merely incomplete. */
                                    vg.v0  = vg.dvy;
                                    (void)cv_pass_g(dev, &vg, ktex,
                                                    kbase + (8u * pstride),
                                                    16.0f, cv_b0);
                                }

                                /* ========== THE U WINDOW: WHERE DOES THE
                                   16-TAP ROW KERNEL ACTUALLY SIT? ==========

                                   The u anchor is the one number in cv_pass()'s
                                   geometry that has NEVER been measured:
                                   g.u0 = (2 + c0)/rw, and the 2 is inherited, not
                                   derived.  A constant texture cannot measure it -
                                   every tap reads the same texel whatever the
                                   window's position - which is why tap16 and x5/x6
                                   could not see it, and why a u RAMP would still be
                                   hard to read: sixteen weighted taps of a ramp sum
                                   to a window average, not to one texel.

                                   A DELTA measures it outright.  The texture is 0
                                   everywhere except texel column 20, which holds 16.
                                   All sixteen weights are equal, so the destination
                                   byte is non-zero EXACTLY for the columns whose
                                   16-tap window contains column 20.  The count of
                                   non-zero columns is therefore the window's WIDTH
                                   and its first and last are its POSITION - no
                                   arithmetic model, no scale, nothing to predict.

                                   WHY 16 AND NOT 2, AND WHY asc 16 AND NOT kv_asc.
                                   The first version of this used a delta of 2 and
                                   kv_asc (7.8182), which is the K-probe's own gain -
                                   and read n0, all 32 columns zero.  That gain is
                                   calibrated for SIXTEEN non-zero taps; on a single
                                   one the value is a sixteenth of it and truncates
                                   away.  The v ramp above uses a literal 16.0 and is
                                   the calibration that is known to work, so this
                                   probe uses the same 16.0 and a delta of 16: a
                                   single lit tap then reads about 16, which stays
                                   clearly non-zero and clearly unclamped even if the
                                   effective scale is wrong by a factor of four
                                   either way.  A delta of 2 had no such margin.

                                   Under `tap 0 at u*tw - 1` (what v measured) the
                                   window for destination column c is texels
                                   3+c .. 18+c, so columns 2..17 light up.
                                   Under `tap 0 at u*tw` it is 4+c .. 19+c and
                                   1..16 light up.  Under a centred window it is
                                   c-4 .. c+11 and 9..24 do.  Nothing else can. */
                                {
                                    cv_geom_t ug;
                                    uint32_t  q3;

                                    for (q3 = 0u; q3 < (32u * 8u); q3++)
                                    {
                                        dw_wr16(ktex, q3, 0u);
                                    }
                                    for (q3 = 0u; q3 < 8u; q3++)
                                    {
                                        dw_wr16(ktex, (q3 * 32u) + 20u, 16u);
                                    }

                                    ug.tw = 32u;  ug.th = 8u;  ug.tp = 32u;
                                    ug.fw = 32u;  ug.fh = 8u;  ug.fp = 32u;
                                    ug.bx = 31u;  ug.by = 7u;
                                    ug.u0 = 4.0f / 32.0f;
                                    ug.dustep = 0.0f;
                                    ug.dux = 1.0f / 32.0f;
                                    ug.dvy = 1.0f / 8.0f;
                                    ug.v0  = ug.dvy;
                                    (void)cv_pass_g(dev, &ug, ktex,
                                                    kbase + (9u * pstride),
                                                    16.0f, cv_b0);
                                }
                            }

                            /* n2: the fifth plane receives no draw at all. */
                            (void)gpu_drain(dev);

                            g1 = (uint32_t)dw_rd(kbase, 32u);
                            g2 = (uint32_t)dw_rd(kbase + pstride, 32u);
                            g3 = (uint32_t)dw_rd(kbase + (2u * pstride), 32u);
                            g4 = (uint32_t)dw_rd(kbase + (3u * pstride), 5u);
                            g5 = (uint32_t)dw_rd(kbase + (4u * pstride), 5u);

                            /* The bisect's three readings, and each carries its
                               own two controls: how many of the 512 bytes it was
                               allowed to write are still the 0xEE sentinel (`st`,
                               a draw that did not happen) and how many are exact
                               zeros (`z`).  `st` is what keeps `0` honest - a
                               plane nobody wrote reads 238 and a plane holding a
                               dead sum reads 0, and with the count printed the two
                               cannot be confused. */
                            {
                                uint32_t b5, b6, b7;
                                uint32_t st5 = 0u, st6 = 0u, st7 = 0u;
                                uint32_t z5 = 0u, z6 = 0u, z7 = 0u;

                                for (i3 = 0u; i3 < 512u; i3++)
                                {
                                    uint32_t v5 = (uint32_t)dw_rd(kbase + (5u * pstride), i3);
                                    uint32_t v6 = (uint32_t)dw_rd(kbase + (6u * pstride), i3);
                                    uint32_t v7 = (uint32_t)dw_rd(kbase + (7u * pstride), i3);

                                    if (v5 == 0xEEu) { st5++; }
                                    if (v6 == 0xEEu) { st6++; }
                                    if (v7 == 0xEEu) { st7++; }
                                    if (v5 == 0u)    { z5++; }
                                    if (v6 == 0u)    { z6++; }
                                    if (v7 == 0u)    { z7++; }
                                }

                                b5 = (uint32_t)dw_rd(kbase + (5u * pstride), 32u);
                                b6 = (uint32_t)dw_rd(kbase + (6u * pstride), 288u);
                                b7 = (uint32_t)dw_rd(kbase + (7u * pstride), 5u);

                                sprintf(Message,
                                        "gpu cv dig kv x5 g %u st %u z %u\r\n",
                                        (unsigned)b5, (unsigned)st5, (unsigned)z5);
                                UART_SendString(Message);
                                sprintf(Message,
                                        "gpu cv dig kv x6 g %u st %u z %u\r\n",
                                        (unsigned)b6, (unsigned)st6, (unsigned)z6);
                                UART_SendString(Message);
                                sprintf(Message,
                                        "gpu cv dig kv x7 g %u st %u z %u\r\n",
                                        (unsigned)b7, (unsigned)st7, (unsigned)z7);
                                UART_SendString(Message);
                            }

                            /* THE V RAMP READ OUT.  Eight destination rows, one
                               number each, read at destination column 4 - the
                               column whose u lands on the texel the window is
                               centred on, so the value is one texel-row's, not a
                               blend.  Every value must be 16*(y+2) for some y in
                               0..7: 32,48,64,80,96,112,128,144.  A 0 is a row
                               whose v tap fell outside the texture and took the
                               border colour - that row is dead, and 0 is not a
                               value the ramp can produce, so it is unambiguous.
                               A 238 means the draw did not happen at all. */
                            {
                                uint32_t vr[8];
                                uint32_t r3;

                                for (r3 = 0u; r3 < 8u; r3++)
                                {
                                    vr[r3] = (uint32_t)dw_rd(
                                        kbase + (8u * pstride),
                                        (r3 * 32u) + 4u);
                                }

                                sprintf(Message,
                                        "gpu cv dig kv v %u %u %u %u %u %u %u %u\r\n",
                                        (unsigned)vr[0], (unsigned)vr[1],
                                        (unsigned)vr[2], (unsigned)vr[3],
                                        (unsigned)vr[4], (unsigned)vr[5],
                                        (unsigned)vr[6], (unsigned)vr[7]);
                                UART_SendString(Message);
                            }

                            /* The u window's width and position, from the delta's
                               own shadow: row 4 of plane 9 is fp = 32 bytes, so
                               destination column c is byte 128 + c.  `n` must be
                               16 - the kernel's tap count - or the window is not
                               the 16 taps the design assumes. */
                            {
                                uint32_t uf = 999u, ul = 999u, un = 0u;
                                uint32_t c3, v3;

                                for (c3 = 0u; c3 < 32u; c3++)
                                {
                                    v3 = (uint32_t)dw_rd(
                                        kbase + (9u * pstride), 128u + c3);
                                    if (v3 != 0u)
                                    {
                                        if (uf == 999u) { uf = c3; }
                                        ul = c3;
                                        un++;
                                    }
                                }

                                sprintf(Message,
                                        "gpu cv dig kv u f %u l %u n %u %u\r\n",
                                        (unsigned)uf, (unsigned)ul,
                                        (unsigned)un,
                                        (unsigned)dw_rd(
                                            kbase + (9u * pstride), 128u + 4u));
                                UART_SendString(Message);
                            }

                            for (i3 = 0u; i3 < 160u; i3++)
                            {
                                if ((uint32_t)dw_rd(kbase + (3u * pstride),
                                                    i3) == 0xEEu)
                                {
                                    kc4++;
                                }
                            }

                            for (i3 = 0u; i3 < 2u; i3++)
                            {
                                for (t3 = 0u; t3 < GPU_CV_TAPS; t3++)
                                {
                                    s_cv_kern[i3][t3] = ksav[i3][t3];
                                }
                            }

                            sprintf(Message, "gpu cv dig kv q %u s %u a %u\r\n",
                                    (unsigned)kv_t5, (unsigned)kv_sum,
                                    (unsigned)(kv_asc * 1000.0f));
                            UART_SendString(Message);
                            sprintf(Message,
                                    "gpu cv dig kv d rw %u ps %u np %u\r\n",
                                    (unsigned)rw, (unsigned)pstride,
                                    (unsigned)np);
                            UART_SendString(Message);
                            sprintf(Message, "gpu cv dig kv e %u %u %u\r\n",
                                    (unsigned)ke1, (unsigned)ke2,
                                    (unsigned)ke3);
                            UART_SendString(Message);
                            sprintf(Message, "gpu cv dig kv g %u %u %u\r\n",
                                    (unsigned)g1, (unsigned)g2, (unsigned)g3);
                            UART_SendString(Message);
                            sprintf(Message,
                                    "gpu cv dig kv 4 g %u e %u c %u\r\n",
                                    (unsigned)g4, (unsigned)ke1,
                                    (unsigned)kc4);
                            UART_SendString(Message);
                            sprintf(Message, "gpu cv dig kv n %u e 238 s %u\r\n",
                                    (unsigned)g5, (unsigned)ksrc);
                            UART_SendString(Message);
                        }
                        else
                        {
                            sprintf(Message, "gpu cv dig kv no room %u %u\r\n",
                                    (unsigned)ksrc, (unsigned)kneed);
                            UART_SendString(Message);
                        }
                    }
                }
                else
                {
                    sprintf(Message, "gpu cv dig no room %u\r\n",
                            (unsigned)((cw + 12u) * pstride));
                    UART_SendString(Message);
                }

                /* The ambient state, read through the driver's own getters, on
                   either side of the geometry round.  cv_run()'s geometry is a
                   copy of a draw that writes at boot, so if A writes nothing here
                   the difference is in the CONTEXT - and this is where a context
                   difference shows: the blend factors, the colour key and its
                   enable bit, the colour op and unit, the texture unit and its
                   filter, and the texture size/pitch/address as the driver has
                   them STORED.  `gpu_env_state()` is read-only by construction
                   (its note explains what it deliberately does not touch), so
                   printing it here cannot change what it measures. */
                gpu_env_state("cvA");
            }
        }

        /* --- the fold --------------------------------------------------------
           tot = SUM_j byte_j - SUM_j (2*bias_j) + net

           and the two sums of the bias cancel the centrings EXACTLY rather than
           approximately, because 2*bias_j is the number the hardware added and
           not the cen the comment above derived it from.  The +net gives back
           the `(128 + in_zp)*sumw` the pack left behind and the M*biasq, ozp and
           128 the byte domain needs.

           Rounded and clamped once, at the end - the digits themselves were
           clamped by the hardware's write, and that clamp is the offline
           table's `rail`, NOT its `sat`.  This used to say `sat` and the two are
           different clamps: `sat` is one TEXTURE UNIT leaving the byte range
           inside the colour pass, before the add, and it never reaches this
           function at all.  What is read below is the sum of two already-clamped
           halves, so it is `rail` - and the counts taken here are the only place
           the board can see it (see s_cv_rail). */
        g_gpu_dw_prog = DW_PROG(44u, co, 0u);

        /* What every digit of this channel starts from: net minus every bias the
           hardware will have added.  Hoisted out of the pixel loop because it is
           the same for all np of them. */
        tot0 = net - (float)bsum;

        /* The two constants the fold is built from, for the channel the census
           is on, so the offline model's own `net` and `bsum` for that channel
           can be compared with these instead of assumed.  Scaled by 100 and
           printed as integers: this project has no float printf on the target,
           and the same convention is already used by the `ab h 1125 88` line. */
        if (cv_dmp || (s_cv_census && ((int)co == s_cv_census_co)))
        {
            sprintf(Message, "gpu cv dig n%d c%u fold cw%u np%u bsum%d net%d\r\n",
                    s_cv_census_n, (unsigned)co, (unsigned)cw, (unsigned)np,
                    (int)(bsum * 100.0f), (int)(net * 100.0f));
            UART_SendString(Message);
        }


        for (y = 0u; y < np; y++)
        {
            float   tot = tot0;
            int32_t q;

            /* The digit planes for THIS channel, y-th pixel of each.  Read as
               bytes and added as floats: 20 digits of at most 255 cannot lose a
               bit in a float's 24-bit mantissa, and the accumulator this mirrors
               is TFLM's int32.  A plane is np contiguous bytes, so this is a
               plain walk from the plane's base. */
            for (k = 0u; k < cw; k++)
            {
                /* Counted where the byte is already read, not in a walk of its
                   own: `rail` and `at0` are about exactly the bytes the fold
                   added - see s_cv_rail - and asking the Device map for them a
                   second time is the whole difference between one walk and two.
                   Two compares on the healthy path is the entire cost. */
                const uint32_t b = (uint32_t)dw_rd(GPU_DW_DST + (k * pstride), y);

                if (b == 255u) { s_cv_rail++; }
                if (b == 0u)   { s_cv_at0++;  }
                tot += (float)b;
            }

            q = cv_rnd(tot);
            if (q < 0)   { q = 0; }
            if (q > 255) { q = 255; }
            out[(y * cout32) + co] = (int8_t)(q - 128);
        }

        s_cv_unfolded += np;
    }

    /* THE GPU'S OWN ANSWER, the third of the dump's three items and the one the
       glue's own sample is compared against.

       The glue already holds the CPU reference and a copy of exactly these
       bytes in `s_cv_ref`/`ref`, and it dumps both - so why dump `out` here as
       well?  Because they are not the same reading.  The glue's `ref` is the
       tensor the operator was handed; this is the pointer the operator wrote
       through, at the moment it finished writing it.  If those two ever differ
       the fault is in the GLUE's buffer plumbing and not in the arithmetic, and
       that is a conclusion no amount of staring at the arithmetic can reach. */
    if (cv_dmp)
    {
        gpu_cv_hex("out", out, (int)(np * cout32));
    }

    /* Out of the draw, and the reporter has to be told: a stale non-zero value
       here would look exactly like a stall on the next layer's first second of
       CPU work. */
    g_gpu_dw_prog = 0u;

    /* The three things every draw in this function left changed, and nothing
       else did:

         - the texture UNIT COUNT, which the CHAIN-capable cv_run() leaves at
           whatever it used.  Every other path in this file calls
           dhd_set_texture_unit_enable(dev, 1u) before it draws, so 2 is not a
           hazard - but it is state this operator set and did not own.
         - the colour operands, which cv_pass() points at the two texture units
           for E_DHD_OP_ADD.  COPY puts 1a back on TEXTURE_0 and 1b back on ZERO,
           which is the ambient default exactly - leaving 1b on TEXTURE_1 is what
           would silently break every depthwise layer after this one.
         - the per-unit scale/bias, which is the fix the front end needed: the
           library's blit runs with the bilinear filter on and never touches that
           word, so it inherits the last draw's gain unless it is reset. */
    dhd_set_texture_unit_enable(dev, 1u);
    cv_color_setup(dev, E_DHD_OP_COPY);
    gpu_sb_reset();

    s_cv_layers++;
    return 0;
}
