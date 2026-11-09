/**
*     Copyright (c) 2025, Nations Technologies Inc.
* 
*     All rights reserved.
*
*     This software is the exclusive property of Nations Technologies Inc. (Hereinafter 
* referred to as NATIONS). This software, and the product of NATIONS described herein 
* (Hereinafter referred to as the Product) are owned by NATIONS under the laws and treaties
* of the People's Republic of China and other applicable jurisdictions worldwide.
*
*     NATIONS does not grant any license under its patents, copyrights, trademarks, or other 
* intellectual property rights. Names and brands of third party may be mentioned or referred 
* thereto (if any) for identification purposes only.
*
*     NATIONS reserves the right to make changes, corrections, enhancements, modifications, and 
* improvements to this software at any time without notice. Please contact NATIONS and obtain 
* the latest version of this software before placing orders.

*     Although NATIONS has attempted to provide accurate and reliable information, NATIONS assumes 
* no responsibility for the accuracy and reliability of this software.
* 
*     It is the responsibility of the user of this software to properly design, program, and test 
* the functionality and safety of any application made of this information and any resulting product. 
* In no event shall NATIONS be liable for any direct, indirect, incidental, special,exemplary, or 
* consequential damages arising in any way out of the use of this software or the Product.
*
*     NATIONS Products are neither intended nor warranted for usage in systems or equipment, any
* malfunction or failure of which may cause loss of human life, bodily injury or severe property 
* damage. Such applications are deemed, "Insecure Usage".
*
*     All Insecure Usage shall be made at user's risk. User shall indemnify NATIONS and hold NATIONS 
* harmless from and against all claims, costs, damages, and other liabilities, arising from or related 
* to any customer's Insecure Usage.

*     Any express or implied warranty with regard to this software or the Product, including,but not 
* limited to, the warranties of merchantability, fitness for a particular purpose and non-infringement
* are disclaimed to the fullest extent permitted by law.

*     Unless otherwise explicitly permitted by NATIONS, anyone may not duplicate, modify, transcribe
* or otherwise distribute this software for any purposes, in whole or in part.
*
*     NATIONS products and technologies shall not be used for or incorporated into any products or systems
* whose manufacture, use, or sale is prohibited under any applicable domestic or foreign laws or regulations. 
* User shall comply with any applicable export control laws and regulations promulgated and administered by 
* the governments of any countries asserting jurisdiction over the parties or transactions.
**/

/**
 * @file main.c
 * @author Nations
 * @version V1.0.0
 *
 * @copyright Copyright (c) 2024, Nations Technologies Inc. All rights reserved.
 */
/* core_cm7.h compiles SCB_EnableICache() and SCB_EnableDCache() down to nothing
   unless these two are defined, and nothing in this project defines them - not
   the device header, not the Keil project.  Without this the two calls in main()
   would look correct, build with 0 errors, and do precisely nothing.
   The vendor's own examples\Cortex_M\Cache puts the same pair in its Keil
   project's preprocessor defines; putting it here keeps the setting in the
   source, where a save from uVision cannot silently drop it. */
#define __ICACHE_PRESENT 1U
#define __DCACHE_PRESENT 1U

/* 1: 开 I-Cache 和 D-Cache。 0: 都不开。

   这个位曾经被关掉过一段时间, 原因是上板时有个没查清的怀疑: "调试时一次能运行
   到 PreInitialize(), 三次才能进入 main()"。缓存是那段代码里唯一没被硬件验证过
   的新东西, 关掉它就能把缓存从变量里去掉。**那个怀疑至今没有被证实, 也没有被
   证伪** —— 关掉缓存之后板子确实能起, 但当时并没有做对照实验, 所以"能起"不构
   成缓存有罪的证据。

   而它是有代价的, 代价有读数。2026-09-15 两版对照 (同一份代码, 只差这一位):

       关缓存:  grab 242868 us, infer 4587822 us, total 4830691 us
       开缓存:  grab 175055 us, infer 1711713 us, total 1886768 us

   推理快了 2.7 倍, 抓帧 1.4 倍, 总数 2.6 倍 —— 但**没有把问题解决干净**。
   yolo-fastestv2-320 是 0.3 GFLOPs 量级的模型, 600 MHz 的 M7 配 CMSIS-NN 内核算
   下来合理量级是**几百毫秒**, 不是几十。所以剩下的差距是 1.71 s 对 ~0.25 s, 大约
   6~8 倍, 而不是早先这里写的"50~90 倍"—— 那个数是拿"几十毫秒"当基准算出来的,
   基准本身站不住, 是这一版里被修掉的一处断言。

   那 6~8 倍还没有解释。抓帧那条读数尤其可疑: camera_grab_rgb888() 是纯 CPU,
   读 768 KB 非缓存帧缓冲 + 降采样到 320x320, 175 ms 在 600 MHz 上约等于一亿个
   周期, 摊到约 1 MB 的流量上是每字节近百周期, 没有哪种循环形状能解释。而 arena
   就在同一片 SDRAM 里 —— 同一件事很可能同时压着 grab 和 infer 两条读数, 所以
   下一步该量的是这片 SDRAM 的裸带宽, 不是继续改缓存或 MPU。

   注意这一位本身已经从"临时排查开关"升级成结论了: 缓存和 GPU 是能共存的, 见
   region 2 那段注释 —— 之前那版起不来是 region 2 造成的, 不是缓存。

   如果烧上去起不来, 那就是那个怀疑终于被证实了, 把这一行改回 0 即可 —— 那时
   它就不再是一个"临时排查用的开关", 而是一条已知结论, 应该连同根因一起写进
   这里的注释。

   MPU 三个区**与这一位无关**: 它们主要的作用是决定内存类型 (Normal 还是
   Device), Device 内存禁止非对齐访问, TFLM 的 int8 张量会立刻踩到 (见本文件
   上面那段注释)。开关缓存只影响缓存行为, 不改变内存类型, 所以 MPU 不该跟着
   一起关。反过来说, 缓存打开之后 region 2 (GPU 槽位) 就**必须**补上, 见下面。 */
#define CACHE_ENABLE 1

#include <stdio.h>
#include "main.h"
#include <string.h>
#include "Message.h"
#include "bsp_sdram.h"
#include "n32h7xx_rcc.h"   /* RCC_FLAG_*RST / RCC_GetFlagStatus, 复位源诊断 */
//#include "tensorflow/lite/micro/debug_log.h"
//#include "log.h"
#include "yolo_inference.h"
#include "camera.h"
#include "gpu.h"
#include "bsp_dvp.h"    /* DVP_IMAGE_WIDTH/HEIGHT, for the GPU scale probe */

/* 测试帧数据 (320x320 RGB888, 307200 字节, 由 tools/make_detect_image.py 生成)。
   故意做成被 include 的 .inc 而不是独立 .c: 手工加进 Printf.uvprojx 的 <File>
   条目会被正开着的 uVision 覆盖掉 (它保存时按内存里的旧列表重写整个工程),
   症状是 "L6218E: Undefined symbol yolo_detect_image (referred from main.o)",
   文件明明在磁盘上却没参与编译。用 include 就完全不用动工程文件 ——
   yolo_inference.cpp 用 model_data.h 装模型也是同样的做法。
   只被这一个 .c 包含; 里面是定义不是声明。 */
#include "yolo_detect_image.inc"

USART_InitType USART_InitStructure;



/* Number of 32 bit words the SDRAM self test writes and reads back.  256
   words = 1 KB, which is enough to expose any address bit that is not
   decoded.                                                              */
#define SDRAM_TEST_WORDS  256u

/**
*\*\name    sdram_selftest.
*\*\fun     Write a unique value into every word of the tensor arena and read
*\*\        it all back, to prove that the SDRAM decodes every address bit.
*\*\param   base  arena base address (0xC0000000)
*\*\return  number of words that did not read back
**/
static uint32_t sdram_selftest(volatile uint32_t* base)
{
    uint32_t i;
    uint32_t bad = 0u;

    /* Each word gets a value that is unique both in its low byte and as a
       whole 32 bit word: a stuck address bit shows up as a duplicate tag, a
       stuck data line as a wrong word.  A previous version only compared the
       low byte of a few power-of-two offsets and reported "FOLDED" without
       stopping, which let TFLM run against a broken arena.                */
    for (i = 0u; i < SDRAM_TEST_WORDS; i++)
    {
        base[i] = 0xAA000000u | i;
    }

    for (i = 0u; i < SDRAM_TEST_WORDS; i++)
    {
        if (base[i] != (0xAA000000u | i))
        {
            bad++;
        }
    }

    return bad;
}

/* A 64-BIT VIEW OF THE CYCLE COUNTER, because the 32-bit one lies in the one
   direction that matters.

   This core runs at 600 MHz, so DWT->CYCCNT rolls over every 7.158 seconds -
   and a rolled-over delta is not a large number, it is a SMALL one.  The
   reading therefore does not look broken, it looks FAST: the boot of
   2026-09-23 has both halves of that in a single line,

       grab 181435 us, infer 367542 us, total 548977 us

   on an inference where the GPU had died partway and every layer after that
   ran on the CPU.  367542 is 7525820 us minus one rollover, and grab+infer
   only equals total once you add the 7.158 s back to BOTH of them - which is
   the arithmetic that says this is a wrap and not a fast inference.

   Correct as long as no two calls are more than one rollover apart (~7.16 s):
   two rolls between consecutive reads cannot be told from none.  The gaps in
   the trigger path are the grab (~0.18 s) and the inference (1.7-7.5 s on this
   board), so the margin is a factor of two.  The first read of each trigger
   re-bases the counter, so a stale base across a long idle costs nothing:
   everything printed is a difference taken within one trigger. */
/* `b` = the BOX DUMP switch.  Off by default, so the trigger path prints the
   top 3 boxes exactly as it always has; with it on, every trigger prints every
   box that can reach the plate string, each with its score.

   Why it exists: yolo_plate_read() builds the string out of EVERY box with
   class_id > 0, in nms order, up to YOLO_PLATE_MAXBOX = 16, sorted by x1 - there
   is no per-character score gate and no grouping by plate.  This board's scene
   holds ONE plate of 7 glyphs and the string came back 11..12 characters long,
   and the top-3 print cannot say whether the extra characters are weak boxes (a
   threshold fixes that) or confident detections somewhere else in the frame
   (only grouping fixes that).  Those are different repairs, so the reading comes
   first.

   nms sorts by score descending, so boxes[0..15] ARE the set the string is built
   from and the LAST of them are the extra characters, which is why the dump
   prints them in this order.  Put the bound back to 3 once this is answered. */
static uint32_t s_box_dump = 0u;

static uint64_t cycle_now(void)
{
	static uint32_t s_last = 0u;
	static uint64_t s_base = 0ull;
	uint32_t        now    = DWT->CYCCNT;

	if (now < s_last)
	{
		s_base += 0x100000000ull;
	}
	s_last = now;

	return s_base + (uint64_t)now;
}

/* ---- SDRAM 裸带宽: 一次性读数, 开机打一行 --------------------------------

   为什么量它: 本文件顶上那段注释记着一个至今没解释的 6~8 倍 —— 0.3 GFLOPs
   的模型在这颗 600 MHz 的 M7 上要 1.7 s, 而理论量级是几十到几百毫秒。
   同一段时间里相机**每一帧**都写坏 (`bad i` == `i`), 空闲时却几乎不坏
   (`bad 0~1` / 48)。 两条读数指向同一个嫌疑人: 这片 SDRAM 的实际带宽比配置
   算出来的低得多, 于是推理把总线占满、DVP 的写 FIFO 饿死。

   配置上能查到的、可能压低带宽的三条 (firmware/CMSIS/device/system_n32h7xx.c
   的 PreInitialize(), 约 :657; src/bsp_sdram.c 那份 DeviceInit 参数相同但
   **没有被调用**):
     burst length 2  -> 32 位总线上**一次突发只有 8 字节** (M7 缓存行 32 B)
     prefetch read 关、bank interleave 关 (SDRAM_CFG1)
   133 MHz / 32 bit 的名义峰值是 532 MB/s; 上面那几条会把它砍掉一大半, 但砍
   不到零头。 所以先量, 再决定要不要动 CFG1 —— 动了要重烧、要验证数据没错。

   量法: 顺序写 1.15 MB, clean+invalidate D-Cache, 再顺序读回, 32 位访问,
   DWT->CYCCNT 计时。 buf 由调用者给 —— 主循环传 s_frozen_frame (开机时它还
   没有内容, 抓第一帧就被 camera_unpack_full_rgb888 整块覆盖, 所以这一步不改
   任何行为)。

   判读:
     读写都在 150 MB/s 以上 -> SDRAM 不慢, 6~8 倍要另找 (XIP / 算子 / 实际主频)
     某一项低于 ~50 MB/s   -> 就是它, 去查 CFG1 和引脚时序
   读之前必须 clean+invalidate, 否则读回的是缓存, 量出来是 1 GB/s 以上, 那个
   数只说明缓存好用, 不回答这里的问题。

   为什么放在 main() 外面: 它原来写在 main() 里面 (紧挨着 s_frozen_frame 的
   声明), armclang 直接报 "function definition is not allowed here" —— 嵌套
   函数不是 C。 缓冲区改成参数, 函数就能待在文件作用域。 */
static void sdram_bw_probe(uint8_t* buf)
{
	volatile uint32_t* p = (volatile uint32_t*)buf;
	const uint32_t words = (uint32_t)(DVP_IMAGE_WIDTH * DVP_IMAGE_HEIGHT * 3u) / 4u;
	const uint32_t bytes = words * 4u;
	uint32_t i, t_wr, t_rd, sum = 0u, wr_mb, rd_mb;

	t_wr = DWT->CYCCNT;
	for (i = 0u; i < words; i++) { p[i] = i; }
	t_wr = DWT->CYCCNT - t_wr;

	SCB_CleanInvalidateDCache();

	t_rd = DWT->CYCCNT;
	for (i = 0u; i < words; i++) { sum += p[i]; }
	t_rd = DWT->CYCCNT - t_rd;

	/* MB/s = 字节 / (周期 / 600e6) / 1e6 = 字节 * 600 / 周期 */
	wr_mb = (t_wr != 0u) ? (uint32_t)(((uint64_t)bytes * 600ull) / t_wr) : 0u;
	rd_mb = (t_rd != 0u) ? (uint32_t)(((uint64_t)bytes * 600ull) / t_rd) : 0u;

	/* sum 打出来有两个用处: 顺便证明这一遍读写没坏, 也让编译器没法把读循环
	   整个删掉 (p 是 volatile, 本来删不掉, 这条是第二道保险)。 */
	sprintf(Message, "sdram bw wr %u MB/s rd %u MB/s (%u KB sum %08X)\r\n",
			(unsigned)wr_mb, (unsigned)rd_mb,
			(unsigned)(bytes / 1024u), (unsigned)sum);
	UART_SendString(Message);
}

/**
*\*\name    main.
*\*\fun     Main program.
*\*\param   none
*\*\return  none
**/
int main(void)
{
    CopyVectTable(FLASH_BASE,ITCM_BASE,VECT_SIZE);
    /* NOTE: the system clock (PLL1 600MHz, sys_clk = PLL1A), the debug UART1
       TX pin (PA9/AF7) and the external SDRAM are already initialised in
       SystemInit() -> PreInitialize(), i.e. before main() is reached and even
       before the C runtime is set up.  Doing it that early guarantees that the
       SDRAM at 0xC0000000 (used as the YOLO tensor arena) is ready before any
       C code can touch it, so they must NOT be initialised again here.
       PreInitialize() only wires up TX because nothing used to be received;
       GPIO_Configuration() below adds the RX pin the trigger character needs. */
#ifdef N32H78x
    /* Enable Cortex-M4 boot*/
    RCC_EnableCM4(0x15080000);
#endif

    /* USARTx: TX is already done, RX is this call. */
    GPIO_Configuration();

    /* USARTx configuration */
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.BaudRate            = 115200;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.OverSampling        = USART_16OVER;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;

    /* Configure USARTx */
    USART_Init(USARTx, &USART_InitStructure);

    /* Enable the USARTx */
    USART_Enable(USARTx, ENABLE);

    /* Give the fault handlers back the precise exceptions.  With MemManage
       disabled every MEMFAULT escalates to a HardFault (HFSR = FORCED) and
       CFSR/MMFAR no longer tell the truth about the first fault.          */
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA;

    /* SDRAM memory attributes ------------------------------------------------
       Nothing configured the MPU, so 0xC0000000 falls into the Cortex-M7
       default region 0xA0000000..0xDFFFFFFF = External device (Device-nGnRE,
       execute-never).  Both symptoms seen on this board come from that:
         - Device memory does not permit unaligned accesses.  The model's
           first op is PAD, whose row copies start at a byte offset (output
           tensor + 966 for the top padded row + 3 for the left column = +969,
           969 % 4 = 1), so the C library memcpy takes its unaligned-source
           path: CFSR = 0x01000000 (UFSR.UNALIGNED) on the LDR inside
           __aeabi_memcpy, called from arm_pad_s8.
         - execute-never is why the earlier corrupt-arena failure surfaced as
           IACCVIOL when a bad pointer was branched to.
       Describe the SDRAM as Normal memory - write-back, non-shareable,
       non-executable, full access.  Only the memory *type* matters for the
       unaligned accesses: it is what makes them legal for the int8 tensors.
       The cache attributes are a separate question and are settled by splitting
       the SDRAM into two regions below, because the D-cache is now enabled and
       one of those regions is written by DMA rather than by the CPU.

       Write-back, not write-allocate: TEX=0 with C=1 and B=1.  Write-allocate
       would turn a partially-written cache line into a read-modify-write, and
       TFLM stores its activations in runs short enough to hit that often; the
       traffic would go up, not down, which is the opposite of the point.     */

		MPU_Region_InitType MPU_InitStructure;
		uint32_t            mpu_dregion;

		/* Region 0: the YOLO tensor arena, and nothing else.  It used to describe
		   the whole 32 MB of SDRAM, which cost nothing while the caches were off
		   but is wrong once the D-cache is on: the camera frame buffers live in
		   that same SDRAM, the DVP writes them by DMA behind the CPU's back, and
		   a cacheable mapping would let camera_grab_rgb888() read lines the CPU
		   cached frames ago instead of the frame that is actually there. */

		MPU_Disable();
		MPU_InitStructure.Enable           = MPU_REGION_ENABLE;
		MPU_InitStructure.Number           = MPU_REGION_NUMBER0;
		MPU_InitStructure.BaseAddress      = 0xC0000000u;   /* RW_SDRAM1 */
		MPU_InitStructure.Size             = MPU_REGION_SIZE_2MB;
		MPU_InitStructure.SubRegionDisable = 0x00;
		MPU_InitStructure.TypeExtField     = MPU_TEX_LEVEL0;
		MPU_InitStructure.AccessPermission = MPU_REGION_FULL_ACCESS;
		MPU_InitStructure.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
		MPU_InitStructure.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
		MPU_InitStructure.IsCacheable      = MPU_ACCESS_CACHEABLE;
		MPU_InitStructure.IsBufferable     = MPU_ACCESS_BUFFERABLE;
		MPU_ConfigRegion(&MPU_InitStructure);

		/* Region 1: the two 800x480 RGB565 frame buffers, Normal but NOT
		   cacheable.  They occupy 0xC0200000..0xC0377000 - 768000 bytes each,
		   CAM_FRAME_BUFFER_ADDRESS and CAM_FRAME_BUFFER2_ADDRESS in
		   inc/bsp_dvp.h - and a 2 MB region covers both while keeping the base
		   aligned to its own size, which the MPU requires.  Region 0 carries the
		   lower number and so outranks this one wherever the two would overlap;
		   they do not, the arena ends exactly where the first buffer starts. */

		MPU_InitStructure.Number           = MPU_REGION_NUMBER1;
		MPU_InitStructure.BaseAddress      = 0xC0200000u;
		MPU_InitStructure.Size             = MPU_REGION_SIZE_2MB;
		MPU_InitStructure.TypeExtField     = MPU_TEX_LEVEL1;
		MPU_InitStructure.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
		MPU_InitStructure.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
		MPU_ConfigRegion(&MPU_InitStructure);

		/* Regions 3..5: ②'s arena, the frozen camera frame and ③'s arena, at
		   0xC0600000 / 0xC0800000 / 0xC0A00000.  MDK-ARM/M7_TCM.sct places
		   those three sections and carries the layout map; read it before
		   moving any of the addresses below.

		   Same attributes as region 0, deliberately: Normal, write-back,
		   cacheable.  These hold the int8 tensors that TFLM walks with
		   unaligned loads, and left to the default mapping 0xC0000000.. is
		   External device, which forbids those - this is the same fault the
		   top of this block describes, and it is why these regions exist at
		   all rather than the firmware simply not touching the MPU.

		   Three regions, not one 8 MB region at 0xC0400000, and the reason is
		   not size: that one region would also cover GPU_CROP_SRC, which the
		   CPU fills and the GPU's DMA reads (crop_pass(), src/gpu.c).
		   arch_dcache_flush_all() is an empty stub on this board, so making it
		   cacheable means the GPU reads pixels the CPU has not written out
		   yet, and the crop probe's 'bad' count rises for a reason that has
		   nothing to do with the GPU.  0xC0600000 is also the first 2 MB
		   -aligned address above every GPU crop write (the highest is
		   0xC053F800, gpu.c:3587), so it could not start lower anyway.

		   The numbering skips 2 on purpose.  Region 2 is the GPU slot, and the
		   declaration is commented out below (block starting "Region 2: the
		   GPU's 4 MB slot").  Taking that number here would mean uncommenting
		   that block - the obvious first thing to try when the GPU is suspect
		   again - silently replaces ②'s arena mapping instead of adding one.
		   Nothing requires the numbers to be contiguous. */

		/* Read how many regions this MPU implements instead of assuming.  On
		   ARMv7-M MPU_RNR is masked to that count, so a number past the end
		   does not fault: the RBAR/RASR writes land on region 0 and retarget
		   the ① tensor arena, which then falls back to External device - the
		   unaligned-access fault again, and nothing about the symptom points
		   here.  ARMv7-M parts commonly implement 8 or 16 regions, so this is
		   expected to pass; it is checked because the read is one line and
		   the failure mode is not diagnosable from where it surfaces. */

		mpu_dregion = (MPU->TYPE & MPU_TYPE_DREGION_Msk) >> MPU_TYPE_DREGION_Pos;
		sprintf(Message, "mpu dregion %lu\r\n", (unsigned long)mpu_dregion);
		UART_SendString(Message);

		if (mpu_dregion >= 6u)
		{
			/* ②'s tensor arena.  RW_SDRAM2 is 2 MB because ② has ①'s exact
			   architecture, so it needs the arena that ① already measures at
			   1,303,080 bytes. */
			MPU_InitStructure.Number           = MPU_REGION_NUMBER3;
			MPU_InitStructure.BaseAddress      = 0xC0600000u;   /* RW_SDRAM2 */
			MPU_InitStructure.Size             = MPU_REGION_SIZE_2MB;
			MPU_InitStructure.TypeExtField     = MPU_TEX_LEVEL0;
			MPU_InitStructure.IsShareable      = MPU_ACCESS_NOT_SHAREABLE;
			MPU_InitStructure.IsCacheable      = MPU_ACCESS_CACHEABLE;
			MPU_InitStructure.IsBufferable     = MPU_ACCESS_BUFFERABLE;
			MPU_ConfigRegion(&MPU_InitStructure);

			/* The frozen 800x480 RGB888 frame, RW_SDRAM3: 1,152,000 bytes
			   (800*480*3) used of the 2 MB covered.  Cacheable - the
			   resampler reads it once per character patch and writes it
			   once, and nothing DMAs into it.

			   Size is spelled out even though region 3's 2 MB is still in
			   the struct: relying on that leftover was correct by accident,
			   and the next person to reorder these blocks would have given
			   the frozen frame a region too small to hold it.  The tail
			   past the end would then fall on the default memory map -
			   Device, unaligned access fault - a long way from this line.

			   The camera is still writing the ping-pong buffers down at
			   0xC0200000, which region 1 covers and deliberately keeps
			   non-cacheable; this region must not reach that far. */
			MPU_InitStructure.Number           = MPU_REGION_NUMBER4;
			MPU_InitStructure.BaseAddress      = 0xC0800000u;   /* RW_SDRAM3 */
			MPU_InitStructure.Size             = MPU_REGION_SIZE_2MB;
			MPU_ConfigRegion(&MPU_InitStructure);

			/* ③'s arena, RW_SDRAM4: 512 K of the 1 MB covered, the size
			   chosen only to keep the base aligned to its own length.  Last
			   region, ending at 0xC0B00000 - the GPU slot above it has to stay
			   uncacheable, so this one may not grow. */
			MPU_InitStructure.Number           = MPU_REGION_NUMBER5;
			MPU_InitStructure.BaseAddress      = 0xC0A00000u;   /* RW_SDRAM4 */
			MPU_InitStructure.Size             = MPU_REGION_SIZE_1MB;
			MPU_ConfigRegion(&MPU_InitStructure);
		}
		else
		{
			/* Say it out loud.  The cascade will still be built and run, and
			   it will fault the first time TFLM touches ②'s arena, a long way
			   from here - so the last thing printed before that is the only
			   thing that will explain it. */
			UART_SendString("mpu: too few regions for 2/3 arenas\r\n");
		}

		/* Region 2: the GPU's 4 MB slot, 0xC0C00000..0xC0FFFFFF (GPU_SLOT_* in
		   inc/gpu.h), Normal but NOT cacheable - the same attributes as the frame
		   buffers above and for the same reason.  The GPU is a second bus master
		   and the vendor's arch_dcache_flush_all() is an empty stub, so there is
		   no cache maintenance to call: a cacheable mapping here would let a
		   later CPU read of a GPU-written buffer be answered from a line the CPU
		   cached before the GPU ever wrote it.

		   4 MB rather than the 2 MB region 1 uses: the base 0xC0C00000 is aligned
		   to 4 MB but not to 8 MB, and the slot has to cover the driver's video
		   memory, the acc area and the two self test buffers at its top.
		   Leaving this region out entirely is worse than it looks - with no
		   region matching, the Cortex-M7 default map puts 0xA0000000..0xDFFFFFFF
		   at Device, which forbids the unaligned accesses the int8 tensors do. */

		/* region 2 (GPU 槽位, 0xC0C00000 起的 4 MB) **故意不声明**。

		   它本来是按"四区齐全"的对称性加上的, 理由写的是"Device 内存禁止非对齐
		   访问"。那个理由对 region 0 成立 —— 张量 arena 在 0xC0000000, TFLM 的
		   int8 张量确实会做非对齐访问 —— 但 GPU 槽位不是 TFLM 碰的内存, 这条理由
		   从来就没有落在它身上。

		   代价在 2026-09-15 显出来了: 声明它, 就把 0xC0C00000..0xC0FFFFFF 从默认
		   映射的 Device 改成了 Normal 非缓存。而厂商这套驱动是按 Device 写的 ——
		   厂商五个 GPU 例子一个都没开 D-Cache, 整片 SDRAM 都走默认映射, 鲜有
		   别人改过它的内存类型。不声明这一区, 槽位就退回默认映射的 Device, 和
		   厂商例子的行为完全一致, 也和本文件上一次跑通的那一版一致。

		   缓存一致性不需要这一区: Device 内存本来就**永不缓存**, 所以缓存开着
		   也不会让 CPU 读到 GPU 写之前的那一行 —— region 2 要解决的问题不存在。
		   真正需要 region 0/1 是有理由的, 那两个区里一个住着 arena, 一个住着
		   DVP 每帧都在写的帧缓冲。

		   槽位和 region 0/1 都不重叠 (region 1 到 0xC0400000 就结束了), 落回默认
		   映射不会有区域仲裁的问题。 */
		MPU_Enable(MPU_PRIVILEGED_DEFAULT);

		/* The D-cache is the point of this.  The tensor arena - 1.3 MB of it -
		   sits in the external SDRAM at 0xC0000000, and every activation the
		   model reads or writes is an access to it.  With no cache each one is
		   a full external-memory round trip, and there are enough of them to
		   keep the SDRAM bus busy for the whole inference.  The LCDC has to
		   pull 768 KB per frame out of that same SDRAM at 64 Hz or it underruns
		   its own line buffer, and the CPU leaves it nothing.  That is the
		   stutter: idle, the main loop only polls USART - a peripheral, not
		   SDRAM - which is why the picture is smooth right up until a character
		   arrives.

		   The instruction side is not the problem and the I-cache is not the
		   fix: the scatter file places .ANY (+RO) in ITCM, and TCM is never
		   cached.  It is enabled anyway because it costs nothing and would cover
		   anything that ever does execute out of the XIP flash.

		   SCB_EnableDCache() invalidates the D-cache before turning it on, so no
		   stale line from before this point can survive. */

#if CACHE_ENABLE
		SCB_EnableICache();
		SCB_EnableDCache();
#endif

		/* DWT 周期计数器。触发时打出的 grab/infer 两个数就是从这里来的 ——
		   判断卡顿到底出在抓帧还是推理，之前一直在推理，这回让它变成读数。
		   600 MHz 核心，周期数 / 600 = 微秒。 */
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT->CYCCNT = 0u;
		DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;



    /* Output a message on Hyperterminal using printf function.
       2026-10-12: the user asked for THIS line, `yolo init OK` and `gpu init OK`
       to stay, so these three are deliberately NOT behind BOOT_VERBOSE - they are
       the boot's stage markers, and a delivered board says which stage it
       reached.  Everything else the initialisation reports on SUCCESS is gated
       (see the block at the top of inc/Message.h); the failure lines never were. */
		UART_SendString("yolo start ...\n");

#if BOOT_VERBOSE
		/* ---- 复位源诊断 ------------------------------------------------
		   芯片自己知道上一次是谁把它复位了, 这是排查「推理跑到一半重新启动」
		   最快的一条读数 —— 因为几种原因在这里一眼可分, 而它们的修法完全
		   不同:

		     por 1            冷上电 (正常)
		     pin 1            复位引脚 / 按了复位键
		     bor 1            供电跌落 —— 电源问题, 不是代码
		     cm7sw 1          软件复位
		     iwdg1 / wwdg1 1  看门狗超时 —— 代码问题: 有东西太久没喂狗
		     lp 1             低功耗域复位

		   这两行现在归 BOOT_VERBOSE 管 (2026-10-12)。 板子改成上电就自己定期
		   扫描、每轮打一行结果之后, 开机这十几行成功类打印只会把结果行冲散。
		   它们没有被删, 价值也还在: 把 inc/Message.h 的 BOOT_VERBOSE 改成 1,
		   两行原样回来, 一个字都不用重写。

		   清标志不在这里面, 见下面 RCC_ClearResetFlag() 那一段。 */
		sprintf(Message, "rst pin%u por%u bor%u sw%u i1%u i2%u w1%u w2%u lp%u\r\n",
		        (unsigned)RCC_GetFlagStatus(RCC_FLAG_PINRST),
		        (unsigned)RCC_GetFlagStatus(RCC_FLAG_PORRST),
		        (unsigned)RCC_GetFlagStatus(RCC_FLAG_BORRST),
		        (unsigned)RCC_GetFlagStatus(RCC_FLAG_CM7SFTRST),
		        (unsigned)RCC_GetFlagStatus(RCC_FLAG_IWDG1RST),
		        (unsigned)RCC_GetFlagStatus(RCC_FLAG_IWDG2RST),
		        (unsigned)RCC_GetFlagStatus(RCC_FLAG_WWDG1RST),
		        (unsigned)RCC_GetFlagStatus(RCC_FLAG_WWDG2RST),
		        (unsigned)RCC_GetFlagStatus(RCC_FLAG_C1LPRST));
		UART_SendString(Message);
		/* 剩下这几个不在上面那条线上, 单独一行。 加进来有两个原因:

   一, pin 这个读法有个陷阱 —— **烧写下载自己就会拉低 NRST**。 所以「烧完
   第一次开机读到 pin 1」是正常的, 它不能当成「上一次是外部把 NRST 拉低了」
   的证据。 真正有信息量的是**异常那一次之后**那次开机的读数, 而不是紧跟着
   烧写的那一次。

   二, 这是双核芯片 (CM7 + CM4), 复位源比单核多: cm4 的软件复位、MMU 域复位、
   备份域 / 保留域的复位、CM4 低功耗域复位。 我们上面那条线一个都没读。 少读
   一个就得多烧一轮, 而多烧一轮的代价是 25 秒加上一次掉 USB 的风险。 */
sprintf(Message, "rst2 c4sw%u mmu%u bkp%u rete%u lp2%u\r\n",
        (unsigned)RCC_GetFlagStatus(RCC_FLAG_CM4SFTRST),
        (unsigned)RCC_GetFlagStatus(RCC_FLAG_MMURST),
        (unsigned)RCC_GetFlagStatus(RCC_FLAG_BKPEMCRST),
        (unsigned)RCC_GetFlagStatus(RCC_FLAG_RETEMCRST),
        (unsigned)RCC_GetFlagStatus(RCC_FLAG_C2LPRST));
UART_SendString(Message);
#endif  /* BOOT_VERBOSE - 两行复位源到此为止 */

		/* 清标志**不在**上面那道 #if 里面**, 这是刻意的: 它是状态操作, 不是打印。
		   不读也照样清, 否则第一次的读数会一直挂着、后面每次开机都报它, 就分不出
		   是哪一次了 —— 而 BOOT_VERBOSE=0 正是常态, 不能让它顺手把这个也关掉。 */
RCC_ClearResetFlag();

		if (!yolo_plate_init()) {
				UART_SendString("yolo init ERROR\n");
				while (1);
		}
		UART_SendString("yolo init OK\n");	/* NOT gated - boot stage marker, see above */

		/* 输出框缓冲。 */
		static YoloBox boxes[64];

		/* 这一轮要不要走级联 (v1.0 的两级)。 只在下面 #if CAMERA_ENABLE 的抓帧块
		   和开机初始化里被写, 但放在这里是因为下面那个调用点**两种构建都要看
		   它** —— 没有相机时它恒为 0, 于是那条路照旧走单模型。 */
		static uint32_t s_cas_req = 0u;

		/* 上一轮的级联结果。 static 而不是局部变量, 是为了让循环末尾那几条 dump
		   语句 (在 n/plen 那一块**之外**) 还够得到它。

		   它**不**放进下面 #if CAMERA_ENABLE 里, 跟着 s_cas_ok 走: 读它的地方
		   有两处 (搬串的那一块, 和循环末尾打账的两处) 都在相机宏的外面或跨宏,
		   放进去就只有 CAMERA_ENABLE=1 的构建编得过。 代价是没有相机时多 200
		   来字节的 ZI, 而那正是这类错最便宜的保险。 */
		static yolo_cascade_result s_cas;

#if CAMERA_ENABLE
		/* 摄像头那一帧 (800x480 RGB565) 的降采样结果, 307200 字节。
		   声明成 static 就落进 RW_AHB_SRAM 的 .ANY (+RW +ZI) —— 那里原本
		   剩 325 KB, 装得下; 装不下链接器会直接报错, 不会悄悄跑到别处。 */
		static uint8_t rgb[YOLO_IN_W * YOLO_IN_H * 3];

		/* rgb[] 里有没有一帧**真画面**。 'i' 键要在同一帧上重跑推理, 而 rgb 是
		   static —— 只要不抓帧, 下一次推理读到的就是上一次那一帧。 这是板上唯一
		   能把"算子把检测弄坏了"和"画面本来就在变"分开的手段: 活体相机永远不会
		   重复同一帧, 所以两次触发之间的画面差是一个分不掉的混淆项 (见下面 'g'
		   键那段注释)。 全零的 rgb 也能跑完一次推理, 但那是假画面, 读数没有意义,
		   所以这里要挡住。 */
		static uint32_t s_have_frame = 0u;

		/* ---- 级联 v1.0 用的全分辨率冻结帧 (800x480 RGB888 = 1,152,000 字节) ----

		   为什么需要它。 相机是 ping-pong 双缓冲, 约 66 ms 翻一页, 而一次
		   320x320 推理实测 1.7~4.6 秒 —— 等 ② 要用的时候, ① 看过的那个缓冲早
		   被写过了。 所以抓帧的那一刻就把整帧解出来冻在这里, 之后 ①② 全程只用
		   这一份副本。 **冻结不是优化, 是正确性**: 不冻结的话 ② 会从一个 ① 根本
		   没看过的画面里裁字, 结果是错的, 但看起来完全合理, 而且不报任何错。

		   为什么在这里而不在 yolo_inference.cpp 里: rgb[] 就在上面几行, 而相机
		   帧的尺寸/地址这一类判断本来就归这个文件 (MPU 表也是), 级联那边收的是
		   一个指针, 它不该知道 SDRAM 布局。

		   段名以 ".bss." 开头, 与 tensor_arena 同一条规则同一条理由 (见
		   yolo_inference.cpp): 名字决定 armclang 出 SHT_NOBITS 还是 SHT_PROGBITS,
		   而这里要是成了 RW, __main 会在 SDRAM 初始化之前去拷它 -> main() 之前
		   HardFault, 串口一个字都没有。 落在 M7_TCM.sct 的 RW_SDRAM3
		   (0xC0800000, 0x120000 = 1,179,648 字节, 比 1,152,000 多出的零头是给
		   2 MB 对齐让的路 —— 地址必须 2 MB 对齐, MPU 区域只能这么起)。

		   1,152,000 = 800*480*3, 不是 1,125,000: 这个算术差过一次, 差 25,600 字节,
		   那会以 L6220E 的形式报出来。 */
		static uint8_t s_frozen_frame[DVP_IMAGE_WIDTH * DVP_IMAGE_HEIGHT * 3]
				__attribute__((section(".bss.frozen_frame_area"), aligned(16)));

		/* 这一轮 pin 住的 RGB565 帧地址, 0 = 还没有可用的冻结帧 ('i' 要在
		   这个状态下挡住)。 它和 s_frozen_frame 是**同一帧的两种表示**, 只在
		   下面那个抓帧循环里一起写, 之后谁都不许单独更新其中一个 —— 两份
		   不同帧的副本, 正是这套东西唯一无法自证的那种错。 */
		static uint32_t s_cas_frame = 0u;

		/* 级联起没起来。 0 就退回单模型路径, 板子照旧能跑。 */
		static uint32_t s_cas_ok = 0u;

		/* ---- 定期扫描 (自动触发) ------------------------------------------
		   复位即开: 板子上电后不需要任何人按键, 每过一段时间自己抓一帧、跑一次
		   推理、打一行结果。 本交付树没有 `a` 键 (全套按键分支已删除),
		   s_auto_on 恒为 1, 也就是**只有定期扫描这一条路径**。

		   周期从**上一轮扫描结束**算起, 所以它读作"跑一次、歇 1 秒", 与推理本身
		   花多久无关。

		   第一轮在开机后**立刻**开始: s_auto_next 初值是 0, 而 cycle_now() >= 0
		   恒真, 所以上电第一个结果不必等一个周期。
		
		   2026-10-20: 5 秒 -> 1 秒 (用户: "有时候感觉很久都识别不了")。
		   5 秒在板子上是 6.7 秒一轮: 周期从上一轮**结束**算起, 而一轮本身就要
		   1.71 秒 (复用轮: grab 0.325 s + infer 1.38 s), ① 重跑的那一轮要 2.48
		   秒 -> 7.5 秒。 实测 161 轮里 106 轮复用、55 轮重跑 ①, 所以约每 3 轮
		   就有一次慢的。
		   这里只是**空闲等待**, 推理一步没少, 所以缩短不改变任何识别结果, 只把
		   周期降到 2.7 / 3.5 秒。 再往下调收益就有限了: 推理本身已经是下界。 */
#define AUTO_SCAN_PERIOD_US  (1u * 1000u * 1000u)

		/* 同一个 600 MHz 换算: 周期数 / 600 = 微秒 (与上面 grab/infer 那两行
		   用的是同一个除数和同一个 DWT 计数器)。 */
#define AUTO_SCAN_PERIOD     ((uint64_t)AUTO_SCAN_PERIOD_US * 600ull)

		/* 自动扫描开关。 1 = 复位即开; 交付树里没有能改它的按键。 */
		static uint32_t s_auto_on = 1u;

		/* cycle_now() 的下一个截止点。 0 = 开机立刻第一轮。 */
		static uint64_t s_auto_next = 0ull;

		/* 打在一行结果前面的 #n。 交付树只有定期扫描, 所以每轮 +1。 */
		static uint32_t s_scan_no = 0u;

		if (!camera_init()) {
				UART_SendString("camera init ERROR\r\n");
				while (1);
		}
#if BOOT_VERBOSE
		UART_SendString("camera init OK\r\n");
#endif

		/* ② 的解释器 (级联的第二级)。 起不来**不拦路**: 下面自动退回单模型
		   路径, 板子照样能用, 只是整串率回到旧模型那个水平。

		   失败原因由 yolo_cascade_init() 自己的 "crop init FAIL: ..." 行说明
		   (见 yolo_inference.cpp)。 下面这一行是给"**到底走了哪条路**"留的
		   标记: 没有它, 一块退回单模型的板子和一块级联正常的板子, 从串口上
		   只有识别率的差别, 而那要跑很多帧才看得出来 —— 而且很容易被当成
		   "级联没效果"而不是"级联根本没起来"。 与上面 "yolo init OK" 一样
		   不放进 BOOT_VERBOSE: 这是状态标记, 不是诊断打印。 */
		if (yolo_cascade_init()) {
				s_cas_ok = 1u;
				UART_SendString("cascade init OK\r\n");
		} else {
				s_cas_ok = 0u;
				UART_SendString("cascade init ERROR - single-model path\r\n");
		}
#else
		/* 没有摄像头时的老路径: rgb 直接指向 XIP flash 里那一帧 (由
		   tools/make_detect_image.py 转成的 320x320 RGB888, 见 src/yolo_detect_image.inc)。
		   yolo_plate_detect() 只读这个指针, 所以整张图不需要拷进 RAM。 */
		const uint8_t* rgb = yolo_detect_image;
#endif

#if GPU_ENABLE
		/* ===== GPU_ENABLE == 1: 这就是今天的产品 =====
		   这一块是 GPU_ENABLE 唯一的编译期切口之一 (总共四处: 这里、抓帧前端、
		   三个 invoke 的函数体、触发路径的每帧统计; 见 inc/gpu.h:7 的契约)。
		   全 1 时本块照旧开机起 GPU、跑回读自检; 全 0 时它整块不存在, 而且
		   产品代码在链接期再没有任何一处够得到 GPU 算子 —— 判据是 map 的
		   `refers to gpu.o`, **不是 Code=** (向量表硬引用 GPU_IRQHandler,
		   库的内核侧一定会留下)。

		   本交付树已删除全部串口按键 (见循环体注释), 所以 GPU_ENABLE = 0 构建
		   里也不再有 "gpu off in this build" 那条路径; 内部调试版保留按键源码,
		   按键仍可切 GPU 算子。

		   GPU 放在这里起: 它要的那 4 MB (0xC0C00000 起, 见 inc/gpu.h) 和
		   帧缓冲一样在 SDRAM 里, 而这个位置在 MPU 配好、LCDC 也点起来之后,
		   地址不再有变化。GPU 起来后立刻做一次回读自检, 结果直接打出来 ——
		   "init OK" 只说明驱动收了参数, 只有自检过了才说明它真的在读写内存。

		   起不来不拦路: 打一行 ERROR 就往下走, 检测照常。GPU 是加速手段,
		   不是主功能的前提。 */
		/* nsgpu_init() 的返回值: 反汇编 nsgpu.o 得到 —— **1 是成功**,
		   0 是 dhd_alloc_buffer 失败, -1/-2/-3 是三种早期失败。厂商例子里
		   `int result` 收了之后就再没读过, 所以除了库本身没有别的地方写着这个
		   约定; 以前这里猜的是 0, 是错的 (gpu.c 的 NSGPU_INIT_OK)。
		   失败时把原值带出来 —— 那个数比 "ERROR" 三个字贵, 而成功时它是废话。 */
		{
				int gpu_r      = gpu_init();
				uint32_t gpu_bad = 0u;

				if (gpu_r != 1) {
						sprintf(Message, "gpu init ERROR %d\r\n", gpu_r);
						UART_SendString(Message);
				}
				else {
						UART_SendString("gpu init OK\r\n");	/* NOT gated - boot stage marker */
				}

				/* 只在这里跑一次, 不进触发循环: 自检要读写 16384 个像素, 放进
				   触发路径会给每次触发都加上一笔固定开销, 把 grab/infer 那两个
				   读数一起弄脏, 而它跑过一次之后也不会再给出新信息。
				   放在这个位置其实已经覆盖了要担心的情况 —— 摄像头和 DVP 这时
				   都已在跑, 自检是在 GPU 与 DVP 争 SDRAM 的情况下做的。

				   只在 init 成功时跑。init 失败时 GPU 根本没配好, 自检必然全错,
				   再打一行 FAIL 只会把上面那行 ERROR 的原因盖住(gpu.c 的 s_ready
				   同样挡着, 所以这里即使调用也只是空转)。
				   "init 成功但算错" 是另一回事, 那种情况只有自检能说 —— 失败时
				   gpu.c 会把两个不匹配计数和实际读到的字节一起打出来。 */
				if (gpu_r == 1) {
						/* 环境基线: 任何 draw 之前的样子。 下面每一条 fb 行都跟它
						   比。 库的两个卷积入口把 g_framebuffers[active].m_format
						   同时当帧缓冲格式和纹理格式用, 而我们的 CPU 解包默认就是
						   RGB565、从不核对 —— 所以哪个探针改了它, 后面每一次出货
						   的 draw 都会写成一个模型读不出来的格式, 而探针自己照样
						   报 OK。 基线之后第一条跟它不一致的 fb 行就是凶手。 */
#if BOOT_VERBOSE
						gpu_fb_state("init");
#endif
						/* fb 那两行只管帧缓冲 (格式/尺寸/box/地址), 已经查完:
						   它在 dw 之后和 dhd 之后逐字节相同, 而 blit 照旧坏。
						   剩下能读的全部状态 (纹理单元/滤波器/CLUT 基址/颜色单元/
						   操作数/混合/色键) 加一个 CLUT 分配器的指纹, 就是 env 那三行。
						   对照读: 第一条跟基线不同的 env 行就是动了它的探针。 */
						/* gpu_env_state("init"); */  /* 探针已关, 见 gpu selftest OK 上面那段 */
						if (gpu_selftest(&gpu_bad)) {
#if BOOT_VERBOSE
								UART_SendString("gpu selftest OK\r\n");
#endif

/* ===========================================================================
   开机探针已关闭 (2026-09-18), 见下面那句 #endif。

   这一段里的每一个探针都曾经是必要的 —— 它们是 2026-09-16..18 那几轮
   「前端为什么坏」的仪器, 而那个问题已经定位到 `dhd_set_texture_scalebias`
   并修好了。 仪器要问的问题都答完了, 测量值都在笔记里, 现在它们只剩下代价:

     - `gpu_dw_probe()` 起手就用 `dhd_alloc_clut(dev, 512u, ...)` 拿走**整块**
       CLUT, 然后整块上传 512 条。 实测: 热复位后固件**必定**停在这一句之后
       (`env dwA` 三行是日志的最后三行), 冷上电则偶发 —— 你之前四次日志里有
       几次停在同一处, 就是同一个故障。 备份 `mcu_code_bak10_GPU` 里这段代码
       一模一样却能跑, 所以差别不在探针本身, 而在它之前累积的状态; 既然它要
       回答的问题已经答完, 正确的处置是**不跑**, 而不是继续找一个已经不影响
       出货的问题的根因。
     - 六个 `camera_gpu_agreement_probe()` 各带一个 crop 块 (已在 camera.c 关掉)。
     - `gpu_env_state()` 每次开机 24 行环境读数, 只在 bisect 那一次有用。

   关掉之后开机只剩: gpu_init → 自检 → 闸门那一次一致性探针。
   闸门那次必须留 —— 它是唯一锁存 `s_gpu_grab_ok` / `s_gpu_filter_ok` 的地方,
   也就是唯一决定抓帧走 GPU 还是走 CPU 的读数。

   (原来后面还有两行 `rx ready` / `rx pin`, 那是排查"发字符没反应"用的临时诊断,
   串口触发早已证明是通的, 已删。 空闲态一行都不打, 见主循环里触发那段的说明。)

   出货路径一行都没动: 前端 blit + 3 抽头滤波在 camera.c 的抓帧路径上, 深度卷积
   在 yolo 推理里 (19/27 层), 而且 `gpu_dw_conv()` 自己带 GPU-vs-CPU 逐字节
   守卫, 不匹配就回落 CPU。 探针只是「信不信 GPU」的读数来源, 不是它的一部分。
   =========================================================================== */
#if 0
#if CAMERA_ENABLE
								/* 缩放的语义在库里没有文档, 是反汇编读出来的
								   (见 gpu.h), 而其中两件事读不出来: 目标矩形到底
								   落在哪 (反汇编里是绕中心建的), 以及颜色键会不会
								   在真实画面上打洞。所以拿真实的相机帧量一次。

								   放在自检之后、且只在自检通过时跑 —— "GPU 确实
								   在读写内存" 是 "它的缩放结果是这个" 的前提,
								   自检挂了之后这行读数只会误导。

								   源的尺寸用 DVP 的, 不是抓帧路径的 320: 要量的
								   就是 800x480 进去会出来什么。
								   固定开销, 只跑这一次, 不进触发循环。 */
								{
										const uint32_t sw = (uint32_t)DVP_IMAGE_WIDTH;
										const uint32_t sh = (uint32_t)DVP_IMAGE_HEIGHT;
										uint32_t probe_src = camera_frame_addr();

										/* 两组 scale 都是 320*256/源宽, 一个向下取整一个
										   向上。 320/800 和 320/480 都不是整数
										   (102.4 / 170.7), 所以这个舍入必须在硬件上
										   定下来: 102 只有 (800*102)>>8 = 318 像素,
										   铺不满 320; 103 是 321, 从原点画出去超出的
										   那一列被帧缓冲裁掉, 结果正好是 0..319。
										   哪一组能用, 这两个读数说了算。 */
										const uint32_t sx_lo = (GPU_SCALE_W * 256u) / sw;
										const uint32_t sy_lo = (GPU_SCALE_H * 256u) / sh;
										const uint32_t sx_hi = ((GPU_SCALE_W * 256u) + sw - 1u) / sw;
										const uint32_t sy_hi = ((GPU_SCALE_H * 256u) + sh - 1u) / sh;

										sprintf(Message, "gpu scale src %08lX\r\n",
										        (unsigned long)probe_src);
										UART_SendString(Message);

										/* A: 起点放在正中 —— 复现第一次那一跑。 当时
										   由它读出的结论 ("起点是左上角") 是错的,
										   已撤回: 跟随起点的其实是预填充框, 图像
										   自己居中。 这一跑保留, 因为它是那条
										   居中规则在 800x480 上的一个实测点。 */
										gpu_scale_probe(probe_src, sw, sh,
										                GPU_SCALE_W / 2u, GPU_SCALE_H / 2u,
										                sx_lo, sy_lo, "A");

										/* B: 起点放在原点、尺寸向上取整 —— 矩形不再被
										   裁, 读数里就是真实尺寸, 也就是 Stage 2 要
										   用的那一组配置。 */
										gpu_scale_probe(probe_src, sw, sh,
										                0u, 0u, sx_hi, sy_hi, "B");

										gpu_fb_state("scale");
										gpu_env_state("scale");

										/* 把已经可信的那个探针在开机里重跑几次, 夹在
										   各个探针之间。 它的读数有过已知的好值
										   (mean 5 ok 1), 所以哪一段之后它变坏, 就
										   是那一段里的探针动了出货路径依赖的东西。
										   比读库内部状态可靠: 只用公开 API, 而且量
										   的是结果本身, 不是我们的猜测。 */
										(void)camera_gpu_agreement_probe(rgb);
										/* 上一版还跑过两例 (B2 = B 的逐字重复, C = B 的
										   scale 配 A 的落位), 用来隔离那 62 倍代价
										   的来源。 读数已记下, 结论是两者都不能单独
										   解释它, 这里就不再重复测量一个已经被证伪
										   的假设。 */

										/* 几何定死之后, 真正的判据是这两条路径在
										   同一帧上差多少、各花多久 —— 也就是
										   camera_gpu_agreement_probe()。 它自己等
										   一帧、自己快照地址, 所以放哪儿都不缺
										   依赖; CPU 那一侧写进 rgb[], 触发循环
										   随后会覆盖它, 不冲突。

										   它被挪到这一段的最末尾 (见下面 cv 探针
										   之后), 因为它现在不只是"量一下两条路
										   差多少", 而是**给产品锁存前端开关**:
										   它锁 s_gpu_grab_ok 和 s_gpu_filter_ok,
										   抓帧路径就按这两个走。 原来的位置在
										   后面四个探针之前, 等于让产品跑在一个
										   探针还没跑完的环境里去做判决 —— 探针
										   只要漏一个全局量没还原, 判决就是错的,
										   而错法看不出来 (读数全绿, 检测为零)。
										   放在最后, 它量的就是产品真正要跑的
										   那个状态。 */

										/* 第 3 步 (卷积) 的先决条件: 卷积在这个
										   尺寸上几何对不对。 库的缩放路径是绕
										   中心建矩形的, 卷积是另一个入口, 而且
										   过去只在 128x64 上验过、还留了 2 像素
										   边距。 所以先在 320x320 上逐字量一遍,
										   再让它进抓帧路径。

										   核是 identity, 所以预期 b0 —— 也就是
										   全图落在 GPU_CONV_GAP_MAX 之内。 这行
										   读数不及格时, 抓帧那边的卷积分支必须
										   关掉, 而不是去调核。

										   关掉这件事现在**由测量自动做**, 不再
										   是改宏的动作: 开关是 camera.c 的
										   s_gpu_filter_ok, 由
										   camera_gpu_agreement_probe() 末尾那条
										   滤波腿锁存 —— 它在真帧上跑真正要出货
										   的那个核, 与 CPU 比逐通道差, 判据与
										   blit 那条腿同一个。 所以这一行是**诊断**,
										   不是判决: 它不及格而滤波腿及格, 说明
										   引擎在这个合成光栅上不像真的而它在真
										   帧上是好用的 (或反过来), 两边一对照才
										   分得出"几何错"和"核错"。 判决那一条
										   严格更强 —— 它成立就意味着这条也成立。 */
										{
												uint32_t conv_bad = 0u;

												(void)gpu_filter_probe(&conv_bad);

												gpu_fb_state("filt");
												gpu_env_state("filt");
												(void)camera_gpu_agreement_probe(rgb);
												if (conv_bad != 0u) {
														UART_SendString("gpu conv NOT usable\r\n");
												}
										}

										/* 带符号卷提的首次上电。 上一步 (gpu_filter_probe)
										   走的是可分离入口, 那个入口的核是无符号的,
										   负抽头会被 VCVT.U32.F32 归零, 所以它只能
										   做低通; 真正能做边缘算子的 nsgpu_convolve
										   一直被绕开, 因为未包装地调它会把板子打挂。

										   这一跑只回答"它能不能跑、跑出来是不是带
										   符号的响应", 落位不预设 —— 320x320 上的
										   居中修正是花了好几轮才找到的, 这是第三个
										   入口, 位置本身就是读数。 判据全在
										   gpu cv2 a / b 那两行的形状上: 直通是右半行
										   一直亮, 无符号模糊是抹圆的台阶, 带符号的
										   一阶导是两侧归零、只在台阶处起一个包。 */
										{
												uint32_t cv2_bad = 0u;

												(void)gpu_convolve_probe(&cv2_bad);

												gpu_fb_state("cv2");
												gpu_env_state("cv2");
												(void)camera_gpu_agreement_probe(rgb);
												if (cv2_bad != 0u) {
														UART_SendString("gpu cv2 incomplete\r\n");
												}
										}
										/* DHD 驱动级探测, 只加不删, 只在开机跑一次。
										   两个问题: (1) a_handle 到底是不是 DAVE HD 的设备句柄 ——
										   用这颗芯片自己的配置数 (2 纹理单元 / 3 colour pass / 4 常量色)
										   去核, 对不上就说明拿到的不是设备; (2) 一次 draw 的固定开销
										   是多少 —— 2 个纹理单元下一次 draw 最多合并 2 个输入平面,
										   每层要拆成 Cout*ceil(Cin/2) 次 draw, 整个模型 109440 次,
										   而现有唯一的 GPU 计时是 320x320 两趟 = 2 次 draw, 外推不了。
										   读数里那个与尺寸无关的项就是每层的固定开销。 */
										gpu_dhd_probe();

										gpu_fb_state("dhd");
										gpu_env_state("dhd");
										(void)camera_gpu_agreement_probe(rgb);
										/* 把模型自己的算子 (深度卷积) 搬到 GPU 上的第一步,
										   只加不删, 只在开机跑一次。之前的「模型卷积搬不上去」
										   是针对 nsgpu_convolve 那条路说的 —— 它把权重按最大值
										   归一化成无符号、整图一个标量增益, 装不了这个模型的
										   int8 对称权重。驱动层没有这个限制:
										   dhd_set_texture_kernel 把权重按字节放进 CLUT, 符号由
										   纹理格式里的 E_DHD_CLUT_KERNEL_SIGNED 选。
										   这一步要实测两件没有记载的事: 纹理在带符号核模式下
										   到底有没有符号扩展 (没有的话要多加一趟 +128 预处理,
										   而且是数据相关的误差, 没有常数 bias 能补), 以及核窗口
										   是按一个 texel 步进还是按目标像素的 UV 增量步进 (决定
										   NHWC 通道平面能不能用 du/dx=C 直接取材)。最后拿真实的
										   有符号 3x3 和 CPU 自己的答案逐字节比, 那才是算子本身。 */
										{
												uint32_t dw_bad = 0u;

												(void)gpu_dw_probe(&dw_bad);

												gpu_fb_state("dw");
												gpu_env_state("dw");
												(void)camera_gpu_agreement_probe(rgb);
												/* 这一行不是 gpu_dw_conv() 的判决。dw_bad 是 Test 3
												   (dw_ref_run) 的失配计数, 而它的 CPU 参照把纹素当成完整
												   的带符号字节, 硬件的被乘数却是 5 位域 —— 已实测, 见
												   dw_kdiff888 上的注释。所以它按构造就不为零, 它正是当初
												   关掉「是否符号扩展」那个问题的测量结果。真正按硬件建模
												   的那个自检是 `gpu dw kdiff` 这一行, 它是过的。 */
												if (dw_bad != 0u) {
														sprintf(Message, "gpu dw t3 8bit-ref bad %u\r\n",
														        (unsigned)dw_bad);
														UART_SendString(Message);
												}
										}

										/* 点卷积 (CONV_2D) 上 GPU 的第一步, 只加不删, 只在开机
										   跑一次。 第 0 步 (tools/conv_gpu_emulate.py) 已经把算术
										   在 PC 上定完了: 52 层里 42 层逐字节干净, 两个设计决定
										   (纹素装 code = a+128、每个 chunk 拆 hi/lo 两个平面)
										   都是在那一跑里发现的。它回答不了的是引擎本身 ——
										   那是芯片的属性, 不是模型的:

										   (1) 1xN 的行核是不是真的按行求和 N 个连续纹素。驱动的
										       可分离分支 (height==1 或 width==1) 存在, 但这块板子
										       实测过的只有 3x3/5x5/3 抽头, 1x16 一次没跑过 ——
										       而 16 是核宽上限, 也正是 chunk 取 32 抽头而不是
										       4 的全部理由。
										   (2) 两个纹理单元能不能在一次 draw 里各用各的 CLUT 偏移
										       和 UV 起点。寄存器都在, 但本固件从来没拿除 0 以外
										       的单元号调过 dhd_active_texture_unit()。这一条不
										       成立, chunk 就是 16 而不是 32, 每层 K 翻倍, 整个
										       模型的 draw 数要涨约 60%。

											   第 (1) 条第一轮就过了。第 (2) 条第一轮读成「不成立」,
											   而那个结论是错的, 错在这一侧不在芯片那一侧: 颜色单元的
											   四个操作数不是焊在某个纹理单元上的, 每个都有自己的输入
											   选择寄存器 (TEXTURE_0..7 / PREVIOUS / 常量色), 而本
											   文件从来没调过 dhd_set_color_op1a/op1b —— 一直吃的是
											   nsgpu_init 留下的默认值 op1a = TEXTURE_0。所以两个
											   单元配好了、采样了、结果也锁存了, 只是颜色通路读的是
											   TEXTURE_0, 而 COPY 就是 `out = op1a`, 读 16 是它本该
											   有的答案 —— 五个 16 其实是同一个事实。
											   这一轮把操作数显式选出来 (op1a=TEXTURE_0、
											   op1b=TEXTURE_1、op=ADD), 即厂商自己的
											   dhd_util_color_add() 那套序 (davehd_util_state.c:151);
											   顺序有讲究 —— dhd_set_color_operation() 会清零该公式
											   用不到的操作数, COPY 会清掉 1b, 所以 op 必须先写。

										   两条都只是一次测量, 而在测出来之前整个设计一文不值 ——
										   所以它是单独一轮烧写, 回答不了就不往下走。

										   判据全在读数里: `gpu cv step` 的尾数必须是 0 (窗口按
										   一个纹素步进), `gpu cv tap16/8/4` 的 d 必须是 0/0、
										   -9/9、-13/5 (N 抽头确实求和 N 个), `gpu cv 2u copy`
										   必须是 130 (两单元相加且各读各的地址; +2 是 5->8 位复制的 16*(7>>2)/8; 同一行的 `copy` 与 `1u` 必须仍是 16,
											   拷贝只取 op1a, 那是两个对照), `gpu cv 2u
										   clut` 必须是 23 (CLUT 偏移按单元独立), `gpu cv 2u
										   scale` 必须是 73 (gain 也按单元独立; +1 是复制的那 1)。
 2026-09-18 这一轮四条全中, 而且退路 `gpu cv 2u chain 130`
 也成立 —— 32 taps/draw 的前提站住了, 设计往下走。这一轮还顺带
 把 `123 against 120` 钉死: 核的乘数是纹素字段复制成 8 位
 `(c<<3)|(c>>2)` 再除以 8, 而不是 code 本身 (所以 add 是
 130 = 128 + 16*(7>>2)/8)。conv_gpu_emulate.py 已按这个模型用
 CV_REP=1 重跑: prepare 时把 1/(1+g) 写进 a_scale, 判定仍是
 42 层结构化通过 (不写就掉到 30)。源纹素的 code 取 1 和 7 而不是
										   1 和 3: 1/3 那组里 32 既是「两单元取平均」又是
										   「两单元都读 SRC0」, 一个读数有两种含义就等于没有。 */
										{
												uint32_t cv_bad = 0u;

												(void)gpu_conv_probe(&cv_bad);

												gpu_fb_state("conv");
												gpu_env_state("conv");
												if (cv_bad != 0u) {
														sprintf(Message, "gpu cv branch NOT confirmed %u\r\n",
														        (unsigned)cv_bad);
														UART_SendString(Message);
												}
										}

										/* 开机探针的最后一条, 也是产品前端的判决
										   所在 —— 全部探针跑完之后再锁存。 它同时
										   量两件事: 缩放 blit 与 CPU 的差 (锁
										   s_gpu_grab_ok), 以及**在 blit 之上再过
										   一遍真正要出货的那个 3 抽头核、再与 CPU
										   比** (锁 s_gpu_filter_ok)。

										   第二条是这一版新加的, 因为产品的前端是
										   两级 GPU 处理, 而过去没有任何一个读数
										   覆盖第二级: 一致性探针比的是
										   GPU_SCALE_DST (blit 的输出), 抓帧却解包
										   GPU_SCALE_CONV (滤波后的); gpu_filter_probe
										   问的是"320x320 上核的几何对不对", 用的是
										   identity 核。 三个读数全绿, 中间那一级
										   一次都没被比过 —— 抓帧路径上挂着一个
										   没人量过的算子, 而打印出来的每个数字都
										   是好的。 现在滤波那一级不过就不出货,
										   自动退回被验过的 blit (也就是
										   GPU_FILTER_ENABLE 该被关掉的那个条件,
										   改成由测量自动决定, 而不是靠人去改宏)。 */
										#endif  /* CAMERA_ENABLE */
#endif  /* 开机探针关闭 —— 上面那段说明到此为止 */

								/* 闸门那一次一致性探针, 必须在这个 #if 0 的**外面**。

								   这是一个实测到的 bug, 不是格式洁癖。 上面那段说明 (494~496 行)
								   写着"闸门那次必须留 —— 它是唯一锁存 s_gpu_grab_ok /
								   s_gpu_filter_ok 的地方", 但这一句和它上面的 gpu_fb_state("gate")
								   当时都被写在了 #if 0 里面 (502 行的 #if 0 一直到 788 行才收到
								   #endif), 于是那一整段探针被关掉的同时, 连这两个也一起没了。

								   后果是静默的、而且直接落在出货路径上: camera.c 的
								   s_gpu_grab_ok 初值是 false, 唯一能把它变真的地方就是下面这个探针
								   的末尾 (camera.c:715)。 探针不跑 → 那个锁存永远是 false →
								   camera_grab_rgb888() 永远走 CPU 那条分支, camera_grab_pair()
								   永远返回 NULL。 从 2026-09-18 关探针那天起, 这条产品的前端就
								   一直是 CPU 的, 而没有任何一行打印说这件事 —— 直到 `p` 命令打出
								   `pair gpu: (no gpu)`, 那才是第一次把"GPU 那条臂根本没跑"和
								   "GPU 那条臂跑出来更差"分开。

								   （读那行要小心: `(no gpu)` 是 gpu_rgb == NULL, 也就是这条臂**没有
								   执行**, 不是"执行了但没读出字符"。 两者在日志里长得像, 含义相反。）

								   放回外面还有一个作用: 它同时锁存 s_gpu_filter_ok, 所以 `p` 和 `f`
								   才重新变得有意义 —— 不然 `f` 打开的是一个已经被 latch 关死的门。 */
#if BOOT_VERBOSE
								gpu_fb_state("gate");
#endif
								/* gpu_env_state("gate"); */  /* 探针已关 */
								(void)camera_gpu_agreement_probe(rgb);

								/* 精确算子卸载第一步: TRANSPOSE [0,2,3,1] 走 GPU。

								   这一块**只测量、不接产品**: 它拿合成图案跑的就是
								   ex_invoke_tr() 会调用的那个算子 (gpu_transpose_s8()),
								   印 worst/bad。 bad 0 是全部判据 —— 转置没有重量化可丢,
								   所以它不是"接近"而是对或错。

								   两个用例, 小的那个承担大部分证明: 16x16 装着 0..255
								   每一个调色板下标各一次, 所以 bad 0 是**整个范围**的恒等
								   (被乘数是硬件的属性, 不是我们的选择); 而方阵的转置仍是
								   转置, 寻址也一起量了。 400x48 就是模型自己的形状
								   (1,48,20,20), 图案在两个下标上都变, 所以把 duy/dvx 写成
								   dux/dvy 那种错误不会互相抵消 —— 它会在那里印成"等于输入"
								   而不是转置。

								   **它在 #if 0 的外面, 这不是格式问题**: 上面那一大片开机
								   探针是关着的 (见 549 行的 #if 0), 写进去的东西不会编译,
								   被引用的函数还会被链接器回收出镜像, 而 .axf 检查照样
								   通过 —— 这就是 boot-probe-if0-trap 那个签名。 闸门那次
								   一致性探针必须留在外面是同一个原因, 这里也一样。

								   不接产品: 'j' 关着时它一个字都不影响 (它在产品结果里
								   什么都不改, 只是把 GPU 的字节和 CPU 的字节比一遍)。
								   判据的另一半见 yolo_inference.cpp 的 ex_invoke_tr()。 */
								/* 2026-10-12: 整块归 BOOT_VERBOSE 管。 它是 boot-only、
									   仅测量的探针 (8 行 `gpu tr ...`), 没有任何东西依赖它的
									   返回值 —— 而交付版开机只该剩 `firmware:` + `boot ok`。

									   代价 (与上面 950..954 那段说它"在 #if 0 的外面"不冲突: 那道死掉的是 #if 0, 这道
									   改成 1 就原样回来。 也和上面 blit 探针同一句): 宏=0 时 gpu_transpose_probe()
									   和它独占的 helper 被链接器回收出镜像。 **但 `j` 键的 26 个
									   TRANSPOSE 卸载不受影响** —— 那条路是 yolo_inference.cpp 的
									   ex_invoke_tr() -> gpu_transpose_s8(), 和这个探针不共用函数,
									   所以算子仍然在镜像里、`j` 仍然可用。 实现后要在 map 里核过。 */
#if BOOT_VERBOSE
								{
										uint32_t tr_bad = 0u;

										(void)gpu_transpose_probe(&tr_bad);
										(void)tr_bad;
								}
#endif  /* BOOT_VERBOSE - 转置探针到此为止 */

/* ------------------------------------------------------------------
   下面这一块 (改写版 blit 的探针) 已经在板子上量完了 —— 2026-09-24。
   `gpu blit NOT confirmed` 一行没有打出来, 而本文件只在 bl_bad != 0 时
   才打印它, 所以 bl_bad == 0: 每一位读数都跟预测相符 ——
     c5 123 133 143 246 0 / stride 33 17 (d 16) / fb ok 1 / sep2u 64 /
     rowcol dst nz 6 (修之前是 0, 也就是 pass 2 什么都没写) /
     T trans nz 21 v8 / T3 imp nz 9, 3x3 = [[126,73,24],[84,49,16],[42,24,8]]
   前一次把 T3 的一格读成 74, 是 10 位 scale 字段量化 (a_scale 263 -> 增益
   8.21875) 的取整, 不是算错 —— 按实测增益算 9 格全对。

   按这一块自己那段说明的约定并回 #if 0, 开机重新安静。 代码和探针都留着,
   下次要再量只删这一对指令。 代价要如实说: 关掉之后 gpu_blit_probe 和它
   独占的 helper (含 blit_convolve_new / blit_separable_convolve_new) 会被
   链接器回收出镜像 —— 这是 boot-probe-if0-trap 那个签名的反面, 是这一次
   想要的结果, 不是那个陷阱重演。
   ------------------------------------------------------------------ */
/* 2026-10-08: 这一块曾临时打开一次 (`#if 1`), 为的是重测 COLUMN 形状的地址
   模式 —— gpu.c 的 blit_col_amode() 那条规则要靠 `gpu blit rowcol ...` 与
   `gpu blit B rows ...` 两组读数判定, 而这两组只在这里打印。 那次旧的读数
   (col-shape nz 126 / rows 0 0 0 0 63 63 0 0) 是在 E_DHD_ACCESS_LINEAR 上
   取的, 正是驱动对核宽为 1 明文禁止的配置, 所以它不成立 —— 见 gpu.c 里
   blit_col_amode() 与 BLIT_SEP_ROWCOL_T 那两段注释。

   量完了, 结论与预测一致, 所以按本块自己的约定并回 `#if 0`:
   `gpu blit B col-shape nz 189 first x1 y3 v5` 与
   `gpu blit B rows 0 0 0 63 63 63 0 0` (三行, 不再是两行),
   `gpu blit rowcol dst nz 9` (原来的 6, 现在是完整的 9). 全部对照组在同一
   次 boot 里逐字节复现, `bl_bad` 为 0。

   以后要看这两组读数, 再删这一对指令即可 —— 但要知道代价: 关掉之后
   gpu_blit_probe 与它独占的 helper 会被链接器回收出镜像。 */
#if 0
								/* 改写版 blit 的探针 —— `blit_convolve_new()` 与

								   注意: 这一块偏偏放在上面那个 `#if 0` 的**外面**, 是这一轮的一次性诊断, 不是疏忽。
								   它补的两个函数必须在板子上量过才算数, 而 `#if 0` 里那一整段开机探针是这个产品
								   刻意关掉的 (开机只剩 gpu_init → 自检 → 闸门那一次一致性探针)。 量完这一步,
								   这一块已按此约定并回 `#if 0` (2026-09-24 量完)。
								`blit_separable_convolve_new()` 是 gpu.c 里新增的两个通用原语,
								而不是厂商那两个 blit 的调用。 理由是两条各自独立: 厂商的
								`nsgpu_convolve()` / `nsgpu_separable_convolve()` **根本调不动**
								(核字节缓冲留在 r6/r7 而函数自己不赋值, 解开包装换成垃圾系数,
								operator_type 1 直接 HardFault), 所以同一层没有可用的入口;
								而它们的三个能力缺口 —— 每调用只有一个 scale+bias (本模型 52 个
								CONV_2D 全部逐输出通道量化)、没有 stride 参数、没有 8 位输出版本
								—— 在这两个新函数里补上了。 三样都不是新发明的驱动序列: 它们
								**早在驱动层就已经解决过**, `dw_pass_bc()` 用 UV 梯度表达 stride、
								用 `pixfmt` 表达 8 位输出并把 framebuffer 的 format/size/address/
								bbox 四样存回, 而每调用一个 scale+bias 的缺口本来就是
								`gpu_dw_conv()` 用「逐通道一次调用」绕过去的。 新函数就是把这套
								做法抬到 blit 这一层, **没有碰 `dw_pass_bc` / `gpu_dw_conv` /
								`gpu_conv`**。

								五条判定, 九位掩码, 每一位都对应一个能被观察到的失败:
								bit0  `gpu blit c5`      —— 调用被拒 (参数校验或 drain 拒绝);
								bit1  同一行 5 个通道    —— 逐通道 scale/bias 循环 (123/133/143 是
								      同一组核 + bias 0/10/20; 246 是 scale 64; 0 是全零核 —— 差一个
								      就说明循环串了);
								bit2  `gpu blit stride`  —— stride 参数真的改变了取样;
								bit3  `gpu blit fb ok`   —— 进出之后 fb 的 format/size/bbox 逐项相等
								      (cv_run() 就是漏了 size 和 bbox);
								bit4  `gpu blit sep2u`   —— 两单元那次调用被拒;
								bit5  同一行读数         —— 双单元窗口的 n 纹素间距, 即「一次 draw 吃
								      32 个输入通道」这条前提;
								bit6  gpu blit rowcol  —— 565 目标上 pass 2 真的写了东西 (nz 0
									      是当时量到的缺陷, 判据已改成 n2 != 0);
								bit7  同一行读数         —— 转置 UV 那次调用被拒;
								bit8  gpu blit T3      —— 秩 1 的 3x3 那次调用被拒。

								两处刻意留成**区间而不是等值**: 硬件把 tap0 放在窗口里的哪个纹素
								只有到纹素级才钉死, 所以 `stride` 判的是两次位置的**差**,
								`sep2u` 判 60..68 的带。 带外的读数各自有名字, 都写在打印行里。

								全部跑完即止, 不进触发路径。 但必须说清楚: 这两个函数补的是
								**表达能力**, 不是逐字节相等。 `--range` 已经证明部分和要 11..15
								位而 colour unit 只有一个字节的无损出口, 且 `m` 不是整数 (1032 个
								通道里 0 个), 所以新函数让一个更干净的卸载**可以写出来**,
								不会让它**端到端正确** —— 这一点在函数头注释里也写了。 */
								{
										uint32_t bl_bad = 0u;

										(void)gpu_blit_probe(&bl_bad);

										gpu_fb_state("blit");
										gpu_env_state("blit");
										if (bl_bad != 0u) {
												sprintf(Message, "gpu blit NOT confirmed %u\r\n", (unsigned)bl_bad);
												UART_SendString(Message);
										}
								}
#endif
						} else {
								sprintf(Message, "gpu selftest FAIL %u\r\n", (unsigned)gpu_bad);
								UART_SendString(Message);
						}
				}
		}
#endif

		/* 版本行: 交付版本一眼看出手上这块板子是哪一种构建。

		   判据是**编译期的 `GPU_ENABLE`**, 不是运行时那几个卸载开关 —— GPU 构建
		   即使 g/c/f/j 全关, 前端 blit/滤波照样走 GPU, 它就是这个构建。 纯 CPU
		   构建 (GPU_ENABLE 0) 里 gpu.c 整个不参与链接, 所以那一支是真话。

		   它和下面 `boot ok` 一样**不受 BOOT_VERBOSE 管**: 这是用户要的交付标识,
		   不是开机诊断, 任何配置下都打。 */
#if CAMERA_ENABLE
		/* 一次性 SDRAM 裸带宽读数 (见 sdram_bw_probe() 的注释)。 放在版本行之前,
		   好让 "boot ok 是开机最后一行" 这条约定不变。 它不改任何行为 —— 写的
		   那块缓冲第一帧抓取时就被整块覆盖。 */
		sdram_bw_probe(s_frozen_frame);
#endif

#if GPU_ENABLE
		UART_SendString("firmware: CPU+GPU\r\n");
#else
		UART_SendString("firmware: CPU\r\n");
#endif

		/* 开机只剩这一行, 而且它**不受 BOOT_VERBOSE 管** —— 上面那一整片初始化
		   打印的成功分支都被收进那个宏里了 (默认 0), 只剩失败行照打。 这一行是
		   板子的握手行: "我活着, 串口通, 下面开始出结果"。 位置固定在冷启动的
		   末尾, 抓日志时一眼看出这次上电从哪儿开始 —— 所以它是**最后**一行,
		   版本行加在它前面, 不复用这个位置。 */
		UART_SendString("boot ok\r\n");

		uint64_t t_grab0 = 0ull, t_grab1 = 0ull, t_done = 0ull;

		/* 抓帧那 ~380 ms 花在哪: 停端口 (里面含 150 ms 自旋) / 解整帧 888 /
		   缩 320。 分这三段是因为下一版要动的正是这里 —— 把 "停住等 333 ms"
		   压到只覆盖真正需要独占的一小段, 而不是整段。 */
		uint32_t g_stop = 0u, g_unp = 0u, g_320 = 0u;

		/* 交付版只有定期扫描一条路径, auto_run 恒为 1 (下面的打印段按它分流)。
		   因此所有 `auto_run == 0` 的分支 —— 内部调试版里"按键轮才打"的那些
		   诊断行 —— 在本树里都不可达, 编译器会当死代码消掉。 它们原样保留是
		   为了让本文件仍能对照内部调试版阅读; 注释里提到的"按键"一律指内部版,
		   本交付树的串口没有按键。 */
		uint32_t auto_run = 1u;

		while (1)
    {
#if CAMERA_ENABLE
				/* 交付版只有定期扫描: 一轮跑完歇 AUTO_SCAN_PERIOD_US, 自动再跑一轮。
				   串口按键 (含 GPU 排查用的 'g'/'c'/'j'/'p' 等 18 个) 已从本交付树
				   删除; 需要按键的内部调试版保留这些源码, 但不在本交付包内。
				   串口收到的字节只读掉、丢弃, 不触发推理, 也不改变任何开关。

				   s_auto_next 初值是 0 而 cycle_now() >= 0 恒真, 所以开机第一轮
				   立刻开始, 上电就有结果, 不用等一个周期。 */
				while ((s_auto_on == 0u) || (cycle_now() < s_auto_next))
				{
					/* 等待期间到达的字节照样读掉, 免得 RXDNE 溢出。 */
					if (USART_GetFlagStatus(USARTx, USART_FLAG_RXDNE) != RESET)
					{
						(void)USART_ReceiveData(USARTx);
					}
				}

				/* 交付版没有 'i' (同帧重跑): 产品路径每轮都抓新帧。
				   reuse 恒为 0, 下面的 `if (reuse == 0u)` 始终成立。 */
				uint32_t reuse = 0u;

				/* DVP 现在是不是被这次抓帧握着 (停着)。 抓帧循环里成功拿到稳定帧后
				   **不放**相机, 一路停到推理结束 —— 判据和代价见那两处注释。
				   1 = 正停着; 推理段结束时必须有人把它放回来。 */
				uint32_t dvp_held = 0u;

				t_grab0 = cycle_now();
				if (reuse == 0u) {
					if (s_cas_ok != 0u) {
						/* 一次抓取, 两个视图, 同一帧。 做法是**先把 DVP 停住再读**, 读完
						   resume —— 这一段的判据原来错在哪, 值得留在代码里, 因为它是
						   "板子上每个数都打印得出来、但级联一次都没跑"的一个标本:

						   原来: 解整帧 + 抓 320x320, 中间用 camera_frame_count()
						   前后不变来证明"这期间没翻页"。 这个判据在这块板子上**永远
						   不成立** —— 一页只稳定一个帧周期 (~66 ms), 而这两次读要
						   ~300 ms。 2026-10-16 的日志把它钉死了: grab 911017 us 正好
						   是三次尝试各 ~304 ms, 三次全否, 于是 s_cas_req 恒为 0, 级联
						   **一次都没跑过** —— 那一版打的是 `n 9`, 而级联的 cap 是 7。
						   判不出稳定帧就退单模型, 这个兜底本身没错; 错的是它变成了
						   常态, 而串口上只剩识别率的差别 —— 很容易被读成"级联没
						   效果", 而不是"级联根本没执行"。

						   现在: camera_dvp_pause() 停端口 (内部先等正在飞的那一帧
						   落地), 之后计数不可能再变, 判据变成**能达成的**。 resume
						   必须在**每个**出口上执行, 否则预览永远停住, 而且没有一行
						   打印会说这件事。 代价是预览静止 ~300 ms, 而紧接着的推理
						   本来就要 1.7 s。 */
						uint32_t att;
						s_cas_frame = 0u;
						for (att = 0u; att < 2u; att++) {
							const uint64_t tg0 = cycle_now();
							camera_dvp_pause();
							dvp_held = 1u;
							const uint64_t tg1 = cycle_now();

							const uint32_t c0 = camera_frame_count();
							const uint32_t a  = camera_frame_addr();
							if (a == 0u) {
								camera_dvp_resume();
								dvp_held = 0u;
								break;
							}
							camera_unpack_full_rgb888(a, s_frozen_frame);
							const uint64_t tg2 = cycle_now();
							camera_grab_rgb888(rgb);
							const uint64_t tg3 = cycle_now();
							g_stop = (uint32_t)((tg1 - tg0) / 600ull);
							g_unp  = (uint32_t)((tg2 - tg1) / 600ull);
							g_320  = (uint32_t)((tg3 - tg2) / 600ull);
							const uint32_t c1 = camera_frame_count();

							if (c1 == c0) {
								s_cas_frame = a;
								/* **不放相机**: 从这一句起 DVP 一直停到推理结束
								   (下面推理段末尾那个 if)。 判定依据是 2026-10-19
								   全自动 11 轮: 推理段 `bad i` 恒等于 `i` —— 每一帧
								   的写 FIFO 都溢出; 空闲段 `bad 0~1` / 48~49; 抓帧段
								   `bad g 0`。 也就是说推理期间的预览**本来就是残帧**,
								   停住只是把"闪"换成"静止的一帧"。 电池/相机都安全:
								   停的是 DVP 端口, OV5640 照常出图, 恢复时帧边界
								   重新对齐, resume 已经自带丢 2 帧换页。
								   要回到"推理期间也跑相机"的老行为: 把这一行的
								   break 前面加回 camera_dvp_resume(); dvp_held = 0u; */
								break;
							}
							/* 只有要重试才恢复: 下一圈会再停一次。 */
							camera_dvp_resume();
							dvp_held = 0u;
						}
						if (s_cas_frame == 0u) {
							/* 正常到不了这里 (端口停着, 计数变不了), 但真到了就得
							   自己抓一帧: rgb 是单模型那条路要用的, 不能留着上一
							   轮的画面。 宁可多花 300 ms。 */
							camera_grab_rgb888(rgb);
						}
						s_cas_req = (s_cas_frame != 0u) ? 1u : 0u;
					} else {
						camera_grab_rgb888(rgb);
					}
					s_have_frame = 1u;
				}
				t_grab1 = cycle_now();
#endif
				/* 级联那一路和单模型那一路从这里分岔, 之后的 plate / plen / 日志
				   都只认结果, 不认是谁给的 —— 所以分岔只有这一处, 下面不再有
				   `if (级联)` 分两套日志。 */
				int n;
				if (s_cas_req != 0u) {
#if CAMERA_ENABLE
					n = yolo_cascade_run(rgb, s_frozen_frame,
							DVP_IMAGE_WIDTH, DVP_IMAGE_HEIGHT, &s_cas);
#else
					n = -1;   /* 到不了: 没有相机时 s_cas_req 恒为 0 */
#endif
				} else {
					n = yolo_plate_detect(rgb, boxes, 64);
				}
				t_done = cycle_now();
#if CAMERA_ENABLE
				/* 推理结束, 把相机放回来 —— 推理段整个 DVP 是停着的, 这之前
				   屏上显示的一直是抓帧时那张完整帧。 */
				if (dvp_held != 0u) {
					camera_dvp_resume();
					dvp_held = 0u;
				}
#endif
				if (n < 0) {
						sprintf(Message, "yolo detect err %d\r\n", n);
						UART_SendString(Message);
						while (1);
				}
				/* 拼成车牌字符串 (GBK)。boxes[] 是分数序, yolo_plate_read() 内部按 x1
				   重排后拼接, 所以这里不用管顺序。串口按 GBK 解码才能显示省份汉字。 */
				char plate[YOLO_PLATE_MAXLEN + 1];
				int plen;
				if (s_cas_req != 0u) {
					/* 级联那一轮的串直接搬过来。 s_cas.text 与 plate 同为
					   char[YOLO_PLATE_MAXLEN+1], text_len <= 12, 所以整块拷
					   text_len+1 字节正好 (含结尾的 '\0')。

					   这里**不能**写成 yolo_plate_read(boxes, n, ...): 级联那一路
					   根本没碰过 boxes[], n 也是字符数而不是框数 —— 照抄的话会
					   把上一轮的框 (或者按键路径留下的框) 拼成一个串打出去, 而
					   那个串看起来完全正常。 级联自己的拼串是在
					   yolo_cascade_run() 里用 yolo_plate_read() 完成的, 途径一模一样
					   (只是喂进去的是 ② 选出来的那 7 个框), 所以两边的 GBK 表和
					   12 字节截断口径是同一份代码。 */
					memcpy(plate, s_cas.text, (size_t)s_cas.text_len + 1u);
					plen = s_cas.text_len;
				} else {
					plen = yolo_plate_read(boxes, n, plate, sizeof(plate));
				}
#if DISPLAY_PLATE_ONLY
						/* 只吐车牌本身, 一行一块牌 (Message.h 的 DISPLAY_PLATE_ONLY)。
						   没读出串就什么都不打 —— 这一档是"上板看效果"用的, 不是诊断用的。
						   其它所有打印都被 UART_SendString() 拦掉了, 两个开机标记除外。
						   识别本身一步没少: plate/plen 在上面已经算完, 这里只决定打不打。 */
						if (plen > 0) {
							Message_allow = 1;
							sprintf(Message, "%s\r\n", plate);
							UART_SendString(Message);
							Message_allow = 0;
						}
#else
#if CAMERA_ENABLE
				if (auto_run != 0u) {
						/* 定期扫描: 一行。 #n 只在定期扫描时 +1, 所以相邻两个 #
						   之间差几就是期间一次键都没按。 **每一行都带一个 cas/one
						   标记**, 说明这一轮走的是级联还是单模型 —— 这个标记是
						   2026-10-16 加的, 因为那天才发现: 级联的同帧判据永远不成立,
						   板子一直只跑单模型, 而串口上唯一的区别是识别率。 那正是最
						   容易读成"级联没效果"、而不是"级联没执行"的形态。

						   `n` 在两条路上含义不同: 级联那条是**字符数** (<=7), 单模型
						   那条是**框数** (<=64)。 所以 "cas" 和 n<=7 一起看才是证据;
						   光看 n 会分不清 (n=7 两条路都可能)。 没检出时车牌位置打
						   (no char)、n 打 0, **每轮必打**, 所以这一行同时是心跳: 它
						   不出现是板子或推理卡住了, 而不是"没检出"。

						   Message 是 char[192] (Message.c), 最长情况 (12 字节 GBK
						   车牌) 约 60 字符, 加上标记也装得下。 */
						s_scan_no++;
						sprintf(Message, "#%u %s plate: %s n %d grab %u us infer %u us\r\n",
										(unsigned)s_scan_no,
										(s_cas_req != 0u) ? "cas" : "one",
										plen > 0 ? plate : "(no char)", n,
										(unsigned)((t_grab1 - t_grab0) / 600ull),
										(unsigned)((t_done - t_grab1) / 600ull));
				} else
#endif
				{
						sprintf(Message, "plate: %s\r\n", plen > 0 ? plate : "(no char)");
				}
				UART_SendString(Message);
#endif

#if CAMERA_ENABLE
				/* 600 MHz 核心: 周期 / 600 = 微秒。一次烧录就能回答"卡顿是
				   抓帧造成的还是推理造成的" —— 之前全靠推理, 这回是读数。

				   按键触发才打: 定期扫描那行已经把 grab/infer 两个数带上了, 再打
				   一遍只是把结果流拉长一倍。 按键时照旧, 精度的读法一个字没变。 */
				if (auto_run == 0u) {
						sprintf(Message, "grab %u us, infer %u us, total %u us\r\n",
										(unsigned)((t_grab1 - t_grab0) / 600ull),
										(unsigned)((t_done - t_grab1) / 600ull),
										(unsigned)((t_done - t_grab0) / 600ull));
						UART_SendString(Message);
						if (reuse == 0u) {
								/* 抓帧段的三段账 (微秒)。 stop 里含 camera_dvp_pause
								   的 100 ms 自旋, 所以它至少 100,000 —— 这一行就是
								   "预览为什么停住"的定量答案, 也是把它改成"只停真正
								   需要独占的那一段"时的对照基准。 */
								sprintf(Message, "grab stop %u unpack %u g320 %u\r\n",
												(unsigned)g_stop, (unsigned)g_unp, (unsigned)g_320);
								UART_SendString(Message);
						}
				}

				/* 精确算子探针的两行, 只在 'e' 开着时打 —— 见 'e' 键那段
				   注释和 yolo_inference.cpp 的 "Exact-operator timing
				   probe"。 列序固定为 MAX_POOL_2D / PAD / CONCATENATION /
				   TRANSPOSE / RESHAPE / GATHER / RESIZE_NEAREST_NEIGHBOR /
				   LOGISTIC / SOFTMAX / QUANTIZE。

				   `ex n` 是**把离线普查搬到板上的那一次对账**: 前六列的个数
				   应当逐次等于 1/8/19/52/26/26, 第七列 (RESIZE_NEAREST_NEIGHBOR)
				   恒为 0 —— 那个算子按 stock 登记、没有包装 (见
				   yolo_inference.cpp 里它的登记处), 后三列是另外 12 个算算术
				   但便宜的算子。 两边对不上就说明模型或解析器与普查时不是同一个。

				   `ex us` 是这十类各花掉的微秒 (600 MHz, 与上面 `infer` 同一个
				   除数), `tot` 是它们的和。 tot 与 infer 的差就是 conv/dw 两个
				   算子加解释器自身的开销 —— 这个差决定那 133 个算子值不值得
				   卸载。 由于打在 t_done 之后, 它不进 infer 的读数。 */
				if (yolo_ex_timing_get() != 0) {
						uint32_t exn[YOLO_EX_CLASSES], exus[YOLO_EX_CLASSES];
						uint32_t ex_tot = 0u;
						int ex_k;
						yolo_ex_stat(exn, exus);
						for (ex_k = 0; ex_k < YOLO_EX_CLASSES; ex_k++) {
							ex_tot += exus[ex_k];
						}
						sprintf(Message, "ex n  %u %u %u %u %u %u %u %u %u %u\r\n",
										(unsigned)exn[0], (unsigned)exn[1],
										(unsigned)exn[2], (unsigned)exn[3],
										(unsigned)exn[4], (unsigned)exn[5],
										(unsigned)exn[6], (unsigned)exn[7],
										(unsigned)exn[8], (unsigned)exn[9]);
						UART_SendString(Message);
						sprintf(Message, "ex us %u %u %u %u %u %u %u %u %u %u tot %u\r\n",
										(unsigned)exus[0], (unsigned)exus[1],
										(unsigned)exus[2], (unsigned)exus[3],
										(unsigned)exus[4], (unsigned)exus[5],
										(unsigned)exus[6], (unsigned)exus[7],
										(unsigned)exus[8], (unsigned)exus[9],
										(unsigned)ex_tot);
						UART_SendString(Message);
				}

#if GPU_ENABLE
				/* 从这一行到下面 `gpu drn` 那一段, 全是 GPU 自己的账本 (tr / dw /
				   cv / dead / drain), 不带 `#else`: 宏=0 时这里一行都不打, 正好
				   符合"健康路径不打印"。 上面 `grab/infer/total` 和 `ex` 探针
				   (CPU 计时, key `e`) 都在这道 `#if` 的**外面**, 纯 CPU 构建照旧。 */
				/* 这一行是 `j` 那条 GPU 转置臂的成绩单, 只在臂开着时打 (关着时一条都不打,
				   与加这段之前逐字节相同)。

				   `bad` 是 GPU 的字节与 CPU 的字节不等的**个数**, `worst` 是那些不等里最大
				   的差值 —— 这一轮的判据就是 `bad 0`, 别的一概不算数; 而且 `bad 0` 同时
				   就是"26 个 `[0,2,3,1]` 的答案都被采纳了"的证明 (任何一个实例比不中或
				   被拒, 它都退回 CPU 的答案并让 `bad` 非零)。

				   `gpu us` 是整类 26 个实例在 GPU 上花掉的微秒 (600 MHz, 与上面对齐)。
				   它**是负收益** —— 打包和回读都是逐字节穿 GPU 槽, 而这颗芯片的 SDRAM
				   慢是硅缺陷 (见记忆 sdram-slow-is-silicon) —— 所以这一路买的不是速度,
				   是"26 个算子真的跑在 GPU 上"这件事本身, 代价如实报出来。 */
				if (yolo_tr_get() != 0) {
						uint32_t tr_n = 0u, tr_bad = 0u, tr_worst = 0u, tr_us = 0u;
						yolo_tr_stat(&tr_n, &tr_bad, &tr_worst, &tr_us);
						/* 定期扫描只出一行结果 (2026-10-12), 但 `bad` 非零是**失败** ——
						   按本仓"失败行不静音"的规矩留出来, 无人值守时也要看得见。
						   和 camera.c 里 `mean ... ok 0` 的判定行是同一套打法。 */
						if ((auto_run == 0u) || (tr_bad != 0u)) {
								sprintf(Message, "tr n %u bad %u worst %u gpu %u us\r\n",
												(unsigned)tr_n, (unsigned)tr_bad,
												(unsigned)tr_worst, (unsigned)tr_us);
								UART_SendString(Message);
						}
				}

				/* 这一行是"算子真的跑在 GPU 上"的直接证据, 也是把 CPU 侧打包
				   代价如实报出来的地方: packed/unpacked 是激活个数, 乘 3 就是
				   经由非缓存 GPU 槽搬运的字节数 —— 写侧 2 字节(一个 RGB565
				   texel), 读侧 1 字节(L_8)。refused 是算不进 9 位 bias 域而被
				   退回 CPU 的层数, 实测应为 0。 */
				{
						unsigned dw_layers = 0, dw_draws = 0, dw_refused = 0;
						unsigned dw_packed = 0, dw_unpacked = 0;
						gpu_dw_stat(&dw_layers, &dw_draws, &dw_refused,
												&dw_packed, &dw_unpacked);
						/* `tail` is a STATE, not a count, and it is APPENDED rather
						   than inserted so the shape of this line does not move
						   under anything already reading it.  0 = no limit, i.e.
						   every eligible layer.  It belongs here because a log has
						   to say which cut produced its numbers: the same `L`/`D`
						   mean different things under a different tail, and a state
						   that appears only in the key echo scrolls away from the
						   counts it qualifies. */
						/* The GPU's two latches, printed ONLY while one is set, and printed
						   ABOVE the counts they qualify.  `L`/`D`/`R` and `l`/`d` are
						   cumulative, so without this line an inference whose layers all went
						   to the CPU is written exactly like one whose layers were all
						   ineligible - the difference is whether a drain had already given up
						   earlier in the boot.  `dw` and `cv` are the two operators' flags;
						   one drain sets both, because there is one GPU.  A board reading
						   `gpu dead dw1 cv1` is a CPU-only board until it is reset - press `k`
						   to re-arm it and find out whether the GPU is dead or was only stuck.
						   See gpu_dead_stat() and gpu_revive() in gpu.h, and gpu_drain(). */
						{
							uint32_t dead_dw = 0u;
							uint32_t dead_cv = 0u;
							gpu_dead_stat(&dead_dw, &dead_cv);
							if ((dead_dw != 0u) || (dead_cv != 0u))
							{
								sprintf(Message, "gpu dead dw%u cv%u\r\n",
									(unsigned)dead_dw, (unsigned)dead_cv);
								UART_SendString(Message);
							}
						}
						/* `drnmax` 是本开机**最坏的一次诚实 drain**, 单位是轮询次数
						   (63.5 ns 一次)。 它必须在这里而不是在 `gpu dw drn` 那行上:
						   那一行只打**开机前四次** drain, 所以它给过的每一个读数都是
						   早层小平面的, 而晚层才是大平面(96 个输出通道)。 九次读数
						   相互吻合到两倍以内, 那正是有偏样本的样子, 不是安全余量的
						   样子 —— 而 GPU_DRAIN_LIMIT 正是要按这个尾巴定, 不是按那九
						   次定。 追加成字段而不是另起一行: 它和这一行是同一个量,
						   且这行本来就每帧都打。 见 gpu.h 的 gpu_dw_drnmax()。 */
						/* `gate` 是 fetch 门的读数(轮询次数): n=跑过门的
						   drain 数, w=其中取指器还没追平 pause 地址的, to=用满
						   上限后照样 flush 的, mx=单个门最多花的轮询数, sp=整个
						   开机一共花掉的轮询数(预算读数)。 必须报出来才分得清
						   "门挡住了"和"门根本没跑" —— 见 gpu.h 的
						   gpu_fetch_gate_stat()。 to==n 就说明那个条件永不为真、
						   门只是纯延迟(上一个 busy 门就是这么死的), mx 是要拿去
						   调 GPU_FETCH_GATE_LIMIT 的数。 sp 到了
						   GPU_FETCH_GATE_BUDGET 之后门就不再等, 之后每个 drain 都
						   记进 to —— 所以 to 随 sp 封顶而涨、w 不涨, 那是预算不是
						   条件, 两个数必须一起读。 门到底等在哪一次 drain 上, 由
						   门自己在那次等的时候打 `gpu dw gate w%u sy%u`, 而失败
						   那一次自己的门读数在 `drain dead` 行的 `gw` 字段上。
						   `bus/strt/nz/sor` 是 START WATCH: bus=所有走到 commit 的
						   drain 上 Busy_Status 的**按位或**(它是 10 位忙向量, 不是
						   标志位), strt=真正写了 STREAM_ADDRESS 的 commit 数,
						   nz=其中忙向量非 0 的, sor=这些向量的按位或。 `gw0` 说明
						   取指位置不预测故障, 所以要看的是那个向量 —— 见 gpu.h 的
						   gpu_busy_vector_stat()。

						   `ms%u/%u/%u` 是**驱动自己的状态字**的直方图, 顺序固定
						   为 IDLE/HALTED/ACTIVE (dhd_gpu_status_t), `last` 是最后
						   一次 drain 时的值, `kp` 是 `n` 键改写过它的次数。
						   它和 strt 是一对: 驱动**只在 `m_status != ACTIVE` 时**
						   才写 STREAM_ADDRESS, 所以 `ms` 才是"为什么会写 start"
						   的那一半。 `ls` 是同一结构里的 `m_last_stop_code`, 即
						   ISR 为"把驱动留在那个状态的那次 stop"记下的码 —— IDLE
						   既可能来自干净的 STREAM_END(0), 也可能来自一次硬件根本
						   没发过的停止, 只有这个字能把两者分开。 见 gpu.h 的
						   gpu_drv_status_stat()。 */
						{
							uint32_t gate_n = 0u, gate_w = 0u, gate_to = 0u;
							uint32_t gate_mx = 0u, gate_sp = 0u;
							uint32_t bv_seen = 0u, bv_st = 0u, bv_nz = 0u, bv_or = 0u;
							uint32_t ms_i = 0u, ms_h = 0u, ms_a = 0u;
							uint32_t ms_last = 0u, ms_keeps = 0u, ms_stop = 0u;
							/* 被打断的 draw 的计数, 见 gpu.h 的 gpu_dw_drop_stat()。
							   追加成字段而不是另起一行: 它和 `L`/`D` 是同一个量
							   (算子自己的 draw), 而这一行本来就每帧都打。
							   `drop/chk` 是"三个哨兵字节原封不动"的 draw 数 / 被
							   检查的 draw 数; `re` 是这些里**重画一次还是原封不动**
							   的 (那说明丢的不是一次作业, 而是第一次 draw 留下的
							   状态); `d/c/p` 是这个开机**第一次**丢的 draw 的序号
							   (1 起算, 0 = 还没丢过)、通道号、第几趟。 三个哨兵而
							   不是一个, 因为真实输出的字节可以是任何值 —— 一个
							   位置会在 1/256 的 draw 上误报。 */
							{
								uint32_t dp_chk = 0u, dp_n = 0u, dp_re = 0u;
								uint32_t dp_d = 0u, dp_c = 0u, dp_p = 0u;
								gpu_dw_drop_stat(&dp_chk, &dp_n, &dp_re,
								                 &dp_d, &dp_c, &dp_p);
							gpu_fetch_gate_stat(&gate_n, &gate_w, &gate_to, &gate_mx, &gate_sp);
							gpu_busy_vector_stat(&bv_seen, &bv_st, &bv_nz, &bv_or);
							gpu_drv_status_stat(&ms_i, &ms_h, &ms_a, &ms_last,
							                    &ms_keeps, &ms_stop);
							sprintf(Message, "gpu dw L%u D%u R%u pack%u/%u tail %u drnmax%u gate n%u w%u to%u mx%u sp%u bus%03X strt%u nz%u sor%03X ms%u/%u/%u last%u kp%u ls%u drop%u/%u re%u d%u c%u p%u\r\n",
											dw_layers, dw_draws, dw_refused,
											dw_packed, dw_unpacked,
											(unsigned)g_gpu_dw_tail,
											(unsigned)gpu_dw_drnmax(),
											(unsigned)gate_n, (unsigned)gate_w,
											(unsigned)gate_to, (unsigned)gate_mx,
											(unsigned)gate_sp,
											(unsigned)bv_seen,
											(unsigned)bv_st, (unsigned)bv_nz, (unsigned)bv_or,
											(unsigned)ms_i, (unsigned)ms_h, (unsigned)ms_a,
											(unsigned)ms_last, (unsigned)ms_keeps,
											(unsigned)ms_stop,
											(unsigned)dp_n, (unsigned)dp_chk,
											(unsigned)dp_re,
											(unsigned)dp_d, (unsigned)dp_c,
											(unsigned)dp_p);
							}
						}
						/* 定期扫描只出一行结果 (2026-10-12): 这条 dw 账本也只在按键触发时打。
						   sprintf 照跑 (几十微秒, 一轮一次无意义), 只把**打印**挡掉。

						   整行挡掉是安全的: 交付构建默认 `g`=0 (dw 关闭), 所以
						   `drop`/`gate`/`bus` 这些字段本来就恒为 0, 挡掉不隐藏任何
						   东西; 要看这些读数的人一定坐在串口前面按键。 上面那道
						   `gpu dead` **不在这道 if 里面** —— 它是故障上报 (板子已变成
						   纯 CPU 直到复位), 无人值守时恰恰最需要打出来。

						   这一段原来的注释写"同样只在 x 路径上打"是**错的**: 定期
						   扫描走的是同一条通用触发路径, 所以它每轮照打。 2026-10-12
						   的日志 (mcu_code\log\log.txt) 直接证伪了它。 */
						if (auto_run == 0u) { UART_SendString(Message); }
						/* drain 自身的四分账, 见 gpu.h 的 gpu_drain_split_stat()。
						   `blit_tile()` 从写下来那天就把一次 draw 拆成四段, 而板子
						   读到的三段是 `bl set 0 drw 46 wai 0 drn 59069` —— 一次
						   59155 us 的调用里有 59069 在 gpu_drain() 内部。 也就是
						   说 draw 的代价全在这个函数里, 而这四分账就是它的四段:
						   w=WRITEBACK|CLEAR|STW 屏障(调用点说它只入队, 所以应该是
						   0), g=fetch 门(可拿 gate 的 sp 按 38.06 周期/轮对账),
						   f=dhd_flush 走命令表(这是驱动里的 CPU 活, 唯一我们能改
						   便宜的一段), p=等 sync id(即 GPU 真的在画)。 四段之和
						   应当约等于整次调用 —— 加不起来就是该怀疑的读数。
						   和 `gpu dw L..` 同一类探针行, 同样只在 x 路径上打。 */
						{
							uint32_t dn = 0u, d_w = 0u, d_g = 0u, d_f = 0u, d_p = 0u;
							gpu_drain_split_stat(&dn, &d_w, &d_g, &d_f, &d_p);
							sprintf(Message, "gpu drn n%u w%u g%u f%u p%u\r\n",
							        (unsigned)dn, (unsigned)d_w, (unsigned)d_g,
							        (unsigned)d_f, (unsigned)d_p);
							/* 定期扫描只出一行结果 (2026-10-12): drain 四分账也只在按键
							   触发时打。 sprintf 照跑 (几十微秒, 一轮一次无意义), 只把
							   **打印**挡掉 —— diff 最小, 也不会让 `Message` 变成没写入。 */
							if (auto_run == 0u) { UART_SendString(Message); }
						}
				}
#endif  /* GPU_ENABLE - 每帧的 GPU 账本到此为止 (tr / dw / cv / drain) */
#endif  /* CAMERA_ENABLE */

#if CAMERA_ENABLE
				/* 框坐标映回摄像头像素 (800x480)。检测是在被拉伸成正方形的
				   320x320 上做的, 这里把横纵比例还原, 好对着画面判断框的位置。
				   最多打 3 个, 免得一屏框把车牌行冲走; 按 'b' 打开全量 dump (见 s_box_dump)。
				   分数按千分之一打 (sc 250 = 0.25), 因为 %f 在这个 printf 里没接。 */
				/* auto_run 那一项 = 定期扫描不打框。 这些框每轮重来一遍, 而它们
				   要回答的问题 ("车牌行里多出来的字符是弱框, 还是别处的强框") 是
				   按键时对着某一帧看的。 按 'x' 照旧, 最多 3 个; 按 'b' 全量。 */
				int k;

				if (s_cas_req != 0u) {
						/* ---- 级联那一轮的一行账 ----

						   逐级耗时 + 裁剪矩形, 每轮都打 (包括定期扫描)。
						   `p/c/rs/i2/i3` 依次是 ①、算矩形、重采样、②、③ 的微秒数;
						   ① 被复用的那一轮 `p` 是 0。 `box` 是车牌框在**相机像素**
						   里的位置, `crop` 是**实际用的裁剪矩形** [x0,y0 x1,y1)。

						   裁剪矩形要单独打: 它是这条链路上**唯一一个错了不会有别的
						   症状**的量 —— 矩形错, 后面的框全错, 而 ② 照样给出一个看
						   起来合理的串。 和 PC 上 full_pipeline_e2e.py 对拍时, 这组
						   数是第一个该对上的。

						   `chg` 是复用轮里"① 的框内"与 ① 那次看到的帧的平均绝对差
						   (0..255): 0 = ① 刚跑过、参考刚取; -1 = 没有参考。 */
						uint32_t tp = 0u, tcr = 0u, trs = 0u, tci = 0u, tch3 = 0u;
						int cx0 = 0, cy0 = 0, cx1 = 0, cy1 = 0;
						int mmin = -1, mv = 0;

						/* `m` = 已分类字符里**最小**的 top1-top2 (③ 的 int8 输出差,
						   1 约等于 0.138 概率)。 它回答"这一轮是不是贴着判决策
						   边界", 而这正是 `cbox` 行答不了的: cbox 只在按键轮打,
						   而串逐轮漂移是定期扫描里出现的。 判读:
						     m 大 (>=10) 而串仍逐轮变 ⇒ 变的是**抠图内容** (③ 看到
						       的像素不同), 往 ② 的框上找;
						     m 小 (0..5) ⇒ ③ 自己在边界上, 这是模型/分辨率的问题,
						       不是链路的问题。
						   -1 = 这一轮没有字符被分类 (n 0 或全被跳过)。 */
						for (k = 0; k < s_cas.nchar && k < YOLO_CHAR_MAX; k++) {
								mv = yolo_cascade_char_margin(k);
								if ((mv >= 0) && ((mmin < 0) || (mv < mmin))) mmin = mv;
						}

						/* `sn` = 牌面格点这一轮做了什么 (yolo_cascade_char_snap):
						     bit0..6 = 该位残差 > 0.15*pitch (只是"偏了")
						     bit31   = 本轮真的整行投影了 (0x80000000)
						   所以 `sn 00000010` 是"第 4 位偏了但整行还准, 没动",
						   `sn 80000010` 才是"偏了且已投影"。 固定 8 位宽好扫。 */
						yolo_cascade_stat(&tp, &tcr, &trs, &tci, &tch3);
						yolo_cascade_crop_rect(&cx0, &cy0, &cx1, &cy1);
						/* `vp` = 取景自适应 (yolo_cascade_view_*):
						     pitch  = 上一轮量到的字中心间距 (320 空间 x10), 目标是 258;
						     margin = 当前外扩比例 (%), 10 = 训练契约, 越大 = 字被放得越小;
						     slope  = 字行倾斜 dy/dx (x1000), 正 = 右边低。 这是
						              **校正之后**的残余倾斜, 在 320 空间里量;
						     roll   = 本轮真正用上的横滚角 (x1000 rad), 正 = 右边低。
						              帧里的真实角度 = slope * ch/cw, 而 roll 是累加出来的
						              总角度; 两个都趋近 0 才叫"转正了"。
						   `vp 0,0,0,0` = 还没量到, 这一轮走训练契约 (框 + 10%)。

						   `why` = 这一轮 (no char) 的原因, 只在没出串时有意义:
						     0 = 正常出串
						     1 = **① 一个牌框都没有** (画面里没有牌 / 检测器这一帧什么都没给)
						     2 = ① 有框, 但被**形状闸**拒了 (高大于宽 —— 见 plate_shaped_cam)
						   这两件事以前在日志上一模一样, 而处置完全不同: 1 是检测器的事,
						   2 是"框错了地方", 再往下走只会编假串。

						   `n2 a/b` = ② 的框数, 两个数:
						     a = NMS 之后 (select_chars_7 截到 7 之前)。 比 n 大就说明
						         这一轮 ② 框多了; 因为截断是按**分数**取的, 多出来的
						         那个不一定是最差的框 —— 整行错位一位就是这个样子。
						     b = 过 conf 的候选**总数** (两个头加起来, NMS 之前,
						         不截断)。 > 64 说明这一轮超出缓冲容量了; 现在按分数
						         留最高的 64 个, 丢的是最低分的。 a 小完全不能证明
						         没超 —— 那件事发生在 NMS 之前。 */
						sprintf(Message,
										"cas %s box %d,%d %d,%d crop %d,%d %d,%d n %d cl %d m %d sn %08X vp %d,%d,%d,%d | p %u c %u rs %u i2 %u i3 %u chg %d why %d n2 %d/%d\r\n",
										(s_cas.reused != 0u) ? "reuse" : "fresh",
										(int)s_cas.plate.x1, (int)s_cas.plate.y1,
										(int)s_cas.plate.x2, (int)s_cas.plate.y2,
										cx0, cy0, cx1, cy1, s_cas.nchar, yolo_cascade_char_cls(), mmin,
										(int)yolo_cascade_char_snap(),
										yolo_cascade_view_pitch_x10(), yolo_cascade_view_margin_x100(),
										yolo_cascade_view_slope_x1000(), yolo_cascade_roll_x1000(),
										(unsigned)tp, (unsigned)tcr, (unsigned)trs, (unsigned)tci, (unsigned)tch3,
										(int)yolo_cascade_change(),
										yolo_cascade_reason(), yolo_cascade_n2(), yolo_cascade_nraw());
						UART_SendString(Message);

						/* ---- 6 框补位诊断 ---- */
						{
								int d[11];
								if (yolo_cascade_ch6_diag(d, 11) > 0) {
										sprintf(Message, "ch6 acc %d drop %d p %d r %d c %d,%d,%d,%d,%d,%d\r\n",
														d[1], d[2], d[3], d[4], d[5], d[6], d[7], d[8], d[9], d[10]);
										UART_SendString(Message);
								}
						}

						/* ---- 跨轮翻字探针 ----
						   "每次识别结果不同"只有在**哪一位翻了**的时候才可判读。
						   把这一轮和上一轮的类别、框摆在一起, 两种病因就分开了:
						     框没动 (差 <= 2 px) 而类别翻了 ⇒ ③ 在判决策边界上;
						     框动了 (尤其宽度变了)      ⇒ ② 的框在跳, ③ 只是受害者。
						   只在真翻了时打一行, 稳态完全静默。 n 变了单独打一条,
						   因为位置对齐没意义了 (6 字/7 字不是同一组位置)。 */
						{
								static int   pv_n = -1;
								static int   pv_cls[YOLO_CHAR_MAX];
								static float pv_box[YOLO_CHAR_MAX][4];
								int i;

								if (pv_n != s_cas.nchar) {
										if (pv_n >= 0) {
												sprintf(Message, "flip n %d -> %d\r\n", pv_n, s_cas.nchar);
												UART_SendString(Message);
										}
								} else {
										for (i = 0; i < s_cas.nchar && i < YOLO_CHAR_MAX; i++) {
												if (s_cas.chars[i].class_id != pv_cls[i]) {
														sprintf(Message,
																		"flip %d: %d -> %d m %d box %d,%d %d,%d was %d,%d %d,%d\r\n",
																		i, pv_cls[i], s_cas.chars[i].class_id,
																		yolo_cascade_char_margin(i),
																		(int)s_cas.chars[i].x1, (int)s_cas.chars[i].y1,
																		(int)s_cas.chars[i].x2, (int)s_cas.chars[i].y2,
																		(int)pv_box[i][0], (int)pv_box[i][1],
																		(int)pv_box[i][2], (int)pv_box[i][3]);
														UART_SendString(Message);
												}
										}
								}
								pv_n = s_cas.nchar;
								for (i = 0; i < s_cas.nchar && i < YOLO_CHAR_MAX; i++) {
										pv_cls[i]      = s_cas.chars[i].class_id;
										pv_box[i][0]   = s_cas.chars[i].x1;
										pv_box[i][1]   = s_cas.chars[i].y1;
										pv_box[i][2]   = s_cas.chars[i].x2;
										pv_box[i][3]   = s_cas.chars[i].y2;
								}
						}

						/* ① 的原始候选框 (320x320 检测坐标, 类无关 NMS 之后、pick
						   之前)。 只在按键轮 + ① 真跑过的那一轮打: 复用轮里
						   s_plate_raw 留着上一次的旧值, 不是这一轮的读数。

						   它要回答的是日志里那个现象: 同一块牌, ① 的框宽出现两簇
						   (268 px / 313 px), 一簇整串对、一簇整串错。 是 ① 的回归
						   本身在跳, 还是 pick_plate_area_conf() 在几个候选之间做
						   选择 —— 只有原始列表能分开, `plate` 那行只给 pick 之后
						   的那一个。 */
						if ((auto_run == 0u) && (s_cas.reused == 0u)) {
								static YoloBox praw[8];
								const int np = yolo_cascade_plate_raw(praw, 8);
								for (k = 0; (k < np) && (k < 8); k++) {
										YoloBox cb;
										camera_map_box(&praw[k], &cb);
										sprintf(Message, "pbox %d: %d,%d %d,%d sc %u\r\n",
														k, (int)cb.x1, (int)cb.y1, (int)cb.x2, (int)cb.y2,
														(unsigned)(cb.score * 1000.0f));
										UART_SendString(Message);
								}
						}

						/* 第二行只在按键触发时打, 最多 4 个, 与上面那条单模型 dump
						   同一个理由 (定期扫描不打框)。 坐标是 **② 那张裁剪图的
						   320x320 局部坐标**, 所以**不**走 camera_map_box() —— 那
						   个函数假定输入是整图 320x320, 拿来映射裁剪图坐标会把它们
						   加成"看着像对的"整图坐标, 比不打还坏。 */
						/* 按键轮打**全部**字符 (最多 7 个), 不再只打 4 个: mrg 要
						   回答的是"翻的是哪一位、那一位有多不稳", 而翻的可能正好
						   是第 5..7 位。 定期扫描仍然一个都不打。 */
						for (k = 0; (auto_run == 0u) && k < s_cas.nchar && k < YOLO_CHAR_MAX; k++) {
								sprintf(Message, "cbox %d: %d,%d %d,%d cls %d sc %u mrg %d\r\n",
												k, (int)s_cas.chars[k].x1, (int)s_cas.chars[k].y1,
												(int)s_cas.chars[k].x2, (int)s_cas.chars[k].y2,
												s_cas.chars[k].class_id,
												(unsigned)(s_cas.chars[k].score * 1000.0f),
												yolo_cascade_char_margin(k));
								UART_SendString(Message);
						}
				} else {
						for (k = 0; (auto_run == 0u) && k < n && k < (s_box_dump != 0u ? 16 : 3); k++) {
								YoloBox cb;
								camera_map_box(&boxes[k], &cb);
								sprintf(Message, "box %d: %d,%d %d,%d cls %d sc %u\r\n",
												k, (int)cb.x1, (int)cb.y1, (int)cb.x2, (int)cb.y2, cb.class_id, (unsigned)(boxes[k].score * 1000.0f));
								UART_SendString(Message);
						}
				}
#endif

#if CAMERA_ENABLE
				/* 下一轮的截止点。 放在循环体**最末尾**而不是 t_done 旁边, 是为了让
				   它永远是一次新鲜读数: cycle_now() 只修正**一次** 32 位翻转
				   (DWT->CYCCNT @600 MHz = 7.158 秒), 而上面推理那一段是整轮里唯一
				   一处长间隔。 在这里取, 与它前一次调用之间只隔着这几行打印。

				   交付版只有定期扫描: 周期从**这一轮结束**起算一个整周期。 */
				if (s_auto_on != 0u) {
						s_auto_next = cycle_now() + AUTO_SCAN_PERIOD;
				}
#endif
    }

}


///**
//*\*\name    RCC_Configuration.
//*\*\fun     Configures the different system clocks.
//*\*\param   none
//*\*\return  none
//**/
void RCC_Configuration(void)
{

	    /* Enable GPIO clock */
    GPIO_AHBClkCmd(USARTx_GPIO_CLK,ENABLE); //A~H
	
 // RCC_EnableAHB5PeriphClk2(USARTx_GPIO_CLK,ENABLE); //I~J
    RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_AFIO,ENABLE);
    /* Enable USART Clock */
    USART_APBxClkCmd(USARTx_CLK,ENABLE);
}


///**
//*\*\name    GPIO_Configuration.
//*\*\fun     Configures the different GPIO ports.
//*\*\param   none
//*\*\return  none
//**/
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);

    /* Configure USARTx Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = USARTx_TxPin;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = USARTx_Tx_GPIO_AF;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;
    GPIO_InitPeripheral(USARTx_GPIO, &GPIO_InitStructure);

    /* ... and Rx.  Nothing was ever received before, so PreInitialize() leaves
       this pin alone.  It needs its own alternate function: the Tx pin above is
       on AF7 but PA10 is USART1_RX only on AF5 (bsp/src/log.c, and the pattern
       repeats on USART3/UART9 in the vendor's USART examples).  Reusing the Tx
       AF here routes the pin somewhere that never raises RXDNE. */
    GPIO_InitStructure.Pin            = USARTx_RxPin;
    GPIO_InitStructure.GPIO_Alternate = USARTx_Rx_GPIO_AF;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;
    GPIO_InitPeripheral(USARTx_GPIO, &GPIO_InitStructure);
}

/**
*\*\name    fputc.
*\*\fun     retarget the C library printf function to the USART
*\*\param   ch
*\*\param   f
*\*\return  none
*
* 这一段必须存在, 而且必须是活的 —— 它不是一个可选的便利功能。
*
* 本工程勾不了 MicroLib(带 TFLM, 是 C++), 只能用完整 C 库, 而完整 C 库的
* stdio 默认落到**半主机**上: printf -> __flsbuf -> _sys_istty/_sys_write,
* 这两个是 ARM 的半主机桩, 每个里面一条 `BKPT #0xAB`。接了调试器时 µVision
* 会应答这些 BKPT(所以调试时能打印, 只是每一条 BKPT 都要手点一次"运行",
* 一条 printf 恰好三条: _sys_istty, _sys_write, 再一次 _sys_istty);
* **一旦脱离调试器, 没人应答 BKPT, 它直接升级成 HardFault**
* (CFSR=0 + HFSR=0x80000000 FORCED 就是这个特征), 板子死在打印那一条语句上。
*
* 这不是理论: 库里的 reportError() 打印后死循环, nsgpu_init() 的任何一条
* 早期失败路径都会走它 —— 于是"GPU 初始化失败"会变成"整块板子不启动",
* 断电重上电也一样, 因为每次上电都必然走到这里。
*
* 在源码里定义 fputc, 链接器就不会再从库里取 fputc.o(归档成员只在符号
* 未定义时才被拉进来), 半主机那条路整条消失。这是标准库自带的覆盖机制,
* 不需要 MicroLib。 */
static void uart_putc(int ch)
{
    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(USARTx, USART_FLAG_TXC) == RESET)
    {
    }
    USART_SendData(USARTx, (uint8_t)ch);
}

int fputc(int ch, FILE* f)
{
    (void)f;

    uart_putc(ch);
    return (ch);
}

/* ------------------------------------------------------------------------ *
 * ARM C library low level I/O retarget.
 *
 * 上面那个 fputc 只挡住了 **运行时** 打印这一条路(printf -> fputc)。它挡不住
 * **启动期** 的那一条, 而这一版固件正是死在启动期:
 *
 *   nsgpu.lib 的 reportError 引用 __2printf
 *     -> 拉进 __2printf.o / _printf_char_file.o / stdio_streams.o
 *     -> 拉进 _initio
 *     -> 拉进 freopen
 *     -> 拉进 sys_io.o, 也就是整套半主机桩(每条 _sys_* 里一个 BKPT #0xAB)
 *
 * 然后 __rt_lib_init 在 **main() 之前** 调 _initio 去建立 stdio 流, _initio
 * 调 freopen, freopen 调 _sys_open -> BKPT。接了调试器时 µVision 会应答这些
 * BKPT(所以调试时要连点三次"运行"才能进 main, 一条 printf 恰好三次半主机
 * 请求); 不接调试器就没人应答, BKPT 直接升级成 HardFault, 板子上电后
 * **一个字都打不出来**。
 *
 * bak3_DVP_double 备份里没有 nsgpu.lib, 没人引用 __2printf, 这一整条链根本
 * 没被链进来(map 里没有 sys_io.o, 没有 _sys_open), 所以它能复位后直接跑。
 * 这不是"加 GPU 弄坏了工具链", 是"加 GPU 第一次把标准库的 stdio 拖了进来"。
 *
 * 补上这些 _sys_* 之后, sys_io.o 就成了未被引用的归档成员, 链接器不会再取它,
 * 九条 BKPT 全部从镜像里消失。这是 ARM C 库自带的重定向接口, 完整 C 库同样
 * 支持 —— 依然不需要 MicroLib。
 *
 * 行为选择: 只让 _sys_write 真的往串口写(这样 TFLM 之类走 printf 的诊断信息
 * 能看见), 其余一律做成安全空实现。本板不读写任何文件, _sys_open 直接返回
 * 一个非负假句柄即可。
 * ------------------------------------------------------------------------ */

int _sys_open(const char *name, int openmode)
{
    (void)name;
    (void)openmode;
    return 1;                       /* >= 0 即视为成功 */
}

int _sys_close(int fh)
{
    (void)fh;
    return 0;
}

int _sys_write(int fh, const unsigned char *buf, unsigned len, int mode)
{
    unsigned i;

    (void)fh;
    (void)mode;

    if (buf != NULL)
    {
        for (i = 0u; i < len; i++)
        {
            uart_putc((int)buf[i]);
        }
    }

    return 0;
}

int _sys_read(int fh, unsigned char *buf, unsigned len, int mode)
{
    (void)fh;
    (void)buf;
    (void)len;
    (void)mode;
    return 0;                       /* 不读文件, 返回"读到 0 字节" */
}

int _sys_istty(int fh)
{
    (void)fh;
    return 1;                       /* 当成终端, 库会按行刷新 */
}

int _sys_seek(int fh, long pos)
{
    (void)fh;
    (void)pos;
    return -1;                      /* 不可定位 */
}

int _sys_ensure(int fh)
{
    (void)fh;
    return 0;
}

long _sys_flen(int fh)
{
    (void)fh;
    return 0;
}

void _ttywrch(int ch)
{
    (void)ch;                       /* 调试控制台字符, 丢弃 */
}

void _sys_exit(int return_code)
{
    (void)return_code;
    for (;;)                        /* 无处可退, 停住而不是跳进半主机 */
    {
    }
}

char *_sys_command_string(char *cmd, int len)
{
    (void)cmd;
    (void)len;
    return NULL;                    /* 不支持 system() */
}

