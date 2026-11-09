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
 *\*\*\*\file n32h7xx_it.c
 *\*\author Nations
 *\*\version v1.0.0
 *\*\copyright Copyright (c) 2025, Nations Technologies Inc. All rights reserved.
 **/

#include "n32h7xx_it.h"
#include "n32h7xx.h"
#include "main.h"
#include "Message.h"
#include "bsp_dvp.h"
#include "bsp_lcd.h"
#include "camera.h"
#include "gpu.h"
#include <stdio.h>

/***  Cortex-Mx Processor Exceptions Handlers ***/

/**
 *\*\name   NMI_Handler.
 *\*\fun    This function handles NMI exception.
 *\*\param  none
 *\*\return none
 */
void NMI_Handler(void)
{
}

/**
*\*\name    HardFault_Handler.
*\*\fun     This function handles Hard Fault exception.
*\*\return  None
**/
/* ==========================================================================
 * Fault diagnostics
 *
 * The stock handlers just spin, which leaves no trace of what went wrong.
 * These print CFSR / HFSR / MMFAR / BFAR plus the stacked exception frame
 * (R0-R3, R12, LR, PC, xPSR) on the debug UART before halting.
 *
 * How to read the dump:
 *   CFSR bit9  PRECISERR   data access to a bad address, PC is the exact
 *                          faulting instruction
 *   CFSR bit10 IMPRECISERR write-buffer error, PC is NOT exact and BFAR is
 *                          not valid (typical for a store into a chip select
 *                          that never responds, e.g. 0xC0000000)
 *   CFSR bit15 BFARVALID   BFAR holds the address that could not be accessed
 *   CFSR bit7  MMARVALID   MMFAR holds the address that could not be accessed
 *   -> a PC or BFAR in the 0xC0xxxxxx range means the SDRAM (where the TFLM
 *      tensor arena lives) is not usable.
 * ========================================================================== */
static char s_fault_msg[160];

void Fault_Dump(const char* name, uint32_t* sf, uint32_t exc_return)
{
    uint32_t cfsr = SCB->CFSR;
    uint32_t i;

    UART_SendString("\r\n*** ");
    UART_SendString(name);
    UART_SendString(" ***\r\n");

    sprintf(s_fault_msg, "CFSR  %08X HFSR  %08X\r\n",
            (unsigned)cfsr, (unsigned)SCB->HFSR);
    UART_SendString(s_fault_msg);

    sprintf(s_fault_msg, "MMFAR %08X BFAR  %08X\r\n",
            (unsigned)SCB->MMFAR, (unsigned)SCB->BFAR);
    UART_SendString(s_fault_msg);

    sprintf(s_fault_msg, "EXC_RETURN %08X frame %08X\r\n",
            (unsigned)exc_return, (unsigned)sf);
    UART_SendString(s_fault_msg);

    {
        uint32_t control;

        __asm volatile("mrs %0, control" : "=r"(control));
        sprintf(s_fault_msg, "CONTROL %08X FPCCR %08X\r\n",
                (unsigned)control, (unsigned)*(volatile uint32_t*)0xE000EF34UL);
        UART_SendString(s_fault_msg);
    }

    /* Frame layout: EXC_RETURN bit4 == 1 selects the basic frame (8 words).
       With bit4 == 0 an extended (FP) frame is used, but the 18 extra words
       live ABOVE the basic frame, so the integer registers are STILL at
       sf[0..7] in both cases -- they are never shifted.  Worse, with lazy
       FP stacking (FPCCR.LSPACT == 1) the FP area has not been written at
       all yet, so sf[18..25] is stale stack, not a frame.  Summarise the
       layout instead of printing a second, misleading PC/LR pair.        */
    if ((exc_return & 0x10u) != 0u)
    {
        UART_SendString("frame: basic (8 words)\r\n");
    }
    else if ((*(volatile uint32_t*)0xE000EF34UL & 0x1u) != 0u)
    {
        UART_SendString("frame: extended, FP area not written yet (LSPACT=1)\r\n");
    }
    else
    {
        UART_SendString("frame: extended, FP area written\r\n");
    }

    UART_SendString("--- raw frame ---\r\n");
    for (i = 0U; i < 26U; i++)
    {
        if ((i & 3U) == 0U)
        {
            sprintf(s_fault_msg, "sf[%02u]", (unsigned)i);
            UART_SendString(s_fault_msg);
        }
        sprintf(s_fault_msg, " %08X", (unsigned)sf[i]);
        UART_SendString(s_fault_msg);
        if ((i & 3U) == 3U)
        {
            UART_SendString("\r\n");
        }
    }
    UART_SendString("\r\n");

    /* Basic frame: sf[0..3] R0..R3, sf[4] R12, sf[5] LR, sf[6] PC, sf[7] xPSR.
       For an IACCVIOL (CFSR bit0) PC is the exact instruction that was
       fetched from a non-executable address -- jumping to 0xC0000000, the
       SDRAM base, means an indirect call (typically BLX Rn) was handed a
       call target that came from the tensor arena.                       */
    sprintf(s_fault_msg, "R0  %08X R1  %08X R2  %08X R3  %08X\r\n",
            (unsigned)sf[0], (unsigned)sf[1], (unsigned)sf[2], (unsigned)sf[3]);
    UART_SendString(s_fault_msg);
    sprintf(s_fault_msg, "R12 %08X LR  %08X PC  %08X xPSR %08X\r\n",
            (unsigned)sf[4], (unsigned)sf[5], (unsigned)sf[6], (unsigned)sf[7]);
    UART_SendString(s_fault_msg);

    while (1)
    {
    }
}

/**
*\*\name    HardFault_Handler.
*\*\fun     This function handles Hard Fault exception.
*\*\return  None
**/
__attribute__((naked)) void HardFault_Handler(void)
{
    /* pick the stack that was in use (MSP in handler mode, PSP in thread
       mode) and hand it together with EXC_RETURN to the C reporter       */
    __asm volatile(
        "tst   lr, #4                 \n"
        "ite   eq                     \n"
        "mrseq r0, msp                \n"
        "mrsne r0, psp                \n"
        "mov   r1, lr                 \n"
        "b     HardFault_Report       \n");
}

void HardFault_Report(uint32_t* sf, uint32_t exc_return)
{
    Fault_Dump("HARD FAULT", sf, exc_return);
}

/**
 *\*\name   MemManage_Handler.
 *\*\fun    This function handles Memory Manage exception.
 *\*\param  none
 *\*\return none
 */
__attribute__((naked)) void MemManage_Handler(void)
{
    __asm volatile(
        "tst   lr, #4                 \n"
        "ite   eq                     \n"
        "mrseq r0, msp                \n"
        "mrsne r0, psp                \n"
        "mov   r1, lr                 \n"
        "b     MemManage_Report       \n");
}

void MemManage_Report(uint32_t* sf, uint32_t exc_return)
{
    Fault_Dump("MEMMANAGE FAULT", sf, exc_return);
}

/**
 *\*\name   BusFault_Handler.
 *\*\fun    This function handles Bus Fault exception.
 *\*\param  none
 *\*\return none
 */
__attribute__((naked)) void BusFault_Handler(void)
{
    __asm volatile(
        "tst   lr, #4                 \n"
        "ite   eq                     \n"
        "mrseq r0, msp                \n"
        "mrsne r0, psp                \n"
        "mov   r1, lr                 \n"
        "b     BusFault_Report        \n");
}

void BusFault_Report(uint32_t* sf, uint32_t exc_return)
{
    Fault_Dump("BUS FAULT", sf, exc_return);
}

/**
 *\*\name   UsageFault_Handler.
 *\*\fun    This function handles Usage Fault exception.
 *\*\param  none
 *\*\return none
 */
__attribute__((naked)) void UsageFault_Handler(void)
{
    __asm volatile(
        "tst   lr, #4                 \n"
        "ite   eq                     \n"
        "mrseq r0, msp                \n"
        "mrsne r0, psp                \n"
        "mov   r1, lr                 \n"
        "b     UsageFault_Report      \n");
}

void UsageFault_Report(uint32_t* sf, uint32_t exc_return)
{
    Fault_Dump("USAGE FAULT", sf, exc_return);
}

/**
 *\*\name   SVC_Handler.
 *\*\fun    This function handles SVCall exception.
 *\*\param  none
 *\*\return none
 */
void SVC_Handler(void)
{
}

/**
 *\*\name   DebugMon_Handler.
 *\*\fun    This function handles Debug Monitor exception.
 *\*\param  none
 *\*\return none
 */
void DebugMon_Handler(void)
{
}

/**
 *\*\name   SysTick_Handler.
 *\*\fun    This function handles SysTick Handler.
 *\*\param  none
 *\*\return none
 */
void SysTick_Handler(void)
{
}


/*** N32h7xx Peripherals Interrupt Handlers,available peripheral interrupt
    handler's name please refer to the startup file (startup_n32h7xx.s)***/

/**
 *\*\name   DVPx_IRQHandler.
 *\*\fun    DVP frame-end: hand the just-filled buffer to the LCDC.
 *\*\param  none
 *\*\return none
 *
 * The DVP fills buffer 1, then buffer 2, then buffer 1 again, raising M1TC and
 * M2TC as each one completes.  Pointing the LCDC at the finished buffer is what
 * makes the preview whole: the LCDC then reads a buffer the DVP has moved on
 * from, and cannot show a frame that is half old and half new.
 *
 * The address MUST change in vertical blanking or the switch tears the frame
 * this is here to avoid, so the handler waits for the Y counter to reach the
 * blanking interval before it reconfigures the layer.  The hardware VBR path
 * was measured against this one on 2026-10-19 and made no difference to the
 * tearing that was being chased (the cause turned out to be SDRAM bandwidth -
 * see camera_init()'s pixel-clock note), so the simpler software wait stayed.
 *
 * It used to say "so the loop always terminates", and that was an assumption
 * wearing the clothes of a measurement: the counter runs 0 to 534 only for as
 * long as the LCDC keeps scanning.  If the LCDC ever stops - its clock gated, a
 * layer register mis-set by the configuration call two lines below, a fetch that
 * never returns on the SDRAM this board has already reported slow - the counter
 * sits below 512 and this loop never exits.  The interrupt never returns, the
 * main thread never runs again, and the board is dead with no output at all,
 * which is one of the shapes of the hang being looked for.  So it is bounded
 * now: 200000 iterations is a few milliseconds against the ~30us a line takes,
 * so normal operation never reaches it, and reaching it costs one torn frame
 * instead of the board.
 *
 * The other thing this handler does is read the depthwise operator's progress
 * word (g_gpu_dw_prog, see gpu.c) and report it when it has stopped moving.
 * That is not a progress bar: it is what is left of the only reporter that is
 * still running when the code under suspicion is not.  Nothing here prints
 * on the healthy path - see the note at the top of the handler body.
 */
static void dvp_wait_vblank(void)
{
    uint32_t guard = 0u;

    /* 上限从 200000 提到 2000000 (约 100 ms): 一次等待的**最长**合法时间是
       一整帧 —— AXI/8 时 15.5 ms、按 'l' 切到 AXI/16 后 31 ms。 原来的
       200000 次 (读的是外设寄存器, 一轮十几个周期, 合几毫秒) 短于一整帧,
       会在正常工作时反复超时, 把 " LCDC-HANG " 刷进日志 —— 那会变成对
       仪表的误读。 100 ms 仍是有界的, 真死了也只是一帧一朵花。 */
    while (((LCDC->CPSTS & 0xFFFF) < 0x200) && (guard < 2000000u))
    {
        guard++;
    }

    if (guard >= 2000000u)
    {
        UART_SendString(" LCDC-HANG ");
    }
}

/**
 *\*\name   lcdc_show_buffer.
 *\*\fun    Point layer 1 at a completed DVP buffer.
 *
 * 软件换页: 自旋等 CPSTS 的 Y 计数器越过 0x200 (进入消隐), 再整层重配。
 *
 * 为什么不用硬件的 VBR (只写 CFBADDR + LSRCTRL.VBR): 2026-10-19 在同一块
 * 板子上 A/B 过, 两种换页方式对"屏滚"没有区别 —— 而且 `d` 键的 m1/m2 读数
 * 证明 DVP 确实在两个缓冲间乒乓、屏没有卡在旧缓冲上。 病根不在这里, 所以
 * 留机制更少的那条: 每帧都在变的只有地址, 但整层重配是这条路上已经验证过
 * 很多次的行为。
 *
 * 自旋上限 200 万次 (约 100 ms) 是一整帧的余量: AXI/8 时 15.5 ms, 按 'l'
 * 切到 AXI/16 后 31 ms。 真死了也只是一帧一朵花, 不会把中断卡住。 */
static void lcdc_show_buffer(uint32_t addr)
{
    dvp_wait_vblank();
    LCDC_Layer_Configuration(LCDC_Layer1, DVP_IMAGE_WIDTH, DVP_IMAGE_HEIGHT, addr);
}

void DVPx_IRQHandler(void)
{
#if GPU_ENABLE
    static uint32_t s_prog_last = 0u;
    static uint32_t s_prog_same = 0u;
    static uint32_t s_wedge     = 0u;
    static char     s_prog_msg[48];
#endif

    /* Nothing is printed here.  This handler has to be silent while the
       firmware is healthy - that is a requirement, not a coincidence.

       There used to be a heartbeat here, one "+" every 32nd frame (later
       every 256th), on the argument that its ABSENCE would be a finding:
       nothing else in the log is printed by an interrupt, so a main thread
       stuck inside a library call would leave the whole log silent, and a
       missing heartbeat would separate "interrupts stopped being taken"
       from "stuck in a loop".

       What it cost was the ability to read the log at all.  The DVP
       interrupt runs at the frame rate - about 33 Hz - so even one in 256
       is a new byte every eight seconds, for as long as the board is
       powered.  A terminal that scrolls on incoming data drops a text
       selection about as fast as it can be made, and selecting a whole log
       takes longer than eight seconds.  A log that cannot be read makes
       every other diagnostic in this file worth nothing.

       The detector it was protecting is not lost.  The stall reporter just
       below runs in this same interrupt and covers the failure mode that
       actually happens (the main thread stuck inside a GPU draw); the LCDC
       preview shows a frozen picture if the DVP itself dies; and the
       backup this product was cut from, mcu_code_bak10_GPU, has a DVP
       handler with no print in it at all. */

#if GPU_ENABLE
    /* The GPU-stall watchdog.  It reads the depthwise operator's progress word,
       so it is the LAST live reference to gpu.o once every other cut is made
       (n32h7xx_it.o -> gpu_stc_dump_now / g_gpu_dw_prog in the map).  With
       GPU_ENABLE 0 the engine is never brought up and there is no draw that
       could stop, so the whole block is dead code, not a lost safety net. */
    if (g_gpu_dw_prog == 0u)
    {
        /* Not inside a draw.  No information, not a stall - a refused layer
           leaves this at 0 too, and reporting that as a hang would be wrong on
           every layer the operator declines. */
        s_prog_last = 0u;
        s_prog_same = 0u;
        s_wedge     = 0u;
    }
    else if (g_gpu_dw_prog == s_prog_last)
    {
        s_prog_same++;
        if (s_prog_same >= 31u)
        {
            uint32_t v = g_gpu_dw_prog - 1u;

            sprintf(s_prog_msg, "\r\nSTALL s%u c%u p%u\r\n",
                    (unsigned)(v / 100000000u),
                    (unsigned)((v / 100000u) % 1000u),
                    (unsigned)(v % 100000u));
            UART_SendString(s_prog_msg);

            /* THE WITNESS THAT SURVIVES THIS PATH.  Every other reporter of a
               stopped engine lives in the main loop, and the main loop is
               precisely what is not running - so the drain give-up that would
               otherwise print the STC dump is never reached when the unbounded
               wait is inside a library call.  2026-09-28 is that case on
               record: eight `STALL s38 c0 p0`, then `GPU WEDGE 8s -> reset`,
               and NOT ONE `gpu stc` line in the whole capture to say why the
               engine had stopped.  So dump the block here, once per episode, on
               the first line of it - by now the engine has been stopped about
               half a second, and a stopped STC stays stopped, so a later
               reading is not a better one.

               Read-only (see the note above gpu_stc_dump() in gpu.c - one of
               those words is write-to-clear, and restoring it would destroy the
               state being observed) and it goes out through the same
               UART_SendString as the line above.  Still silent on the healthy
               path: this branch is only reached 31 frame interrupts after the
               progress word stopped moving. */
            if (s_wedge == 0u)
            {
                gpu_stc_dump_now("stall");
            }

            s_prog_same = 1u;       /* and again a second later, while it lasts */

            /* A wedged draw does not fail, it stops - and everything that is
               waiting on it stops with it.  The main loop is inside the library
               call and will never come out, so the serial command handler, the
               detection and the whole of the firmware's own logic are already
               gone: the only thing still running is this interrupt and the LCDC
               feed below it.  That is not "the GPU failed and the CPU carries
               on", which is the hard rule here; it is the GPU taking the
               product down with it.

               Eight seconds of no movement is far past any honest draw on this
               part (a whole layer is tens of milliseconds), so at that point
               the software reset is not a recovery attempt on a slow job, it is
               the refusal to stay dead.  The lines above name the place and the
               reboot put the board back in a state where the command that
               triggered it can be given again.                        */
            s_wedge++;
            if (s_wedge >= 8u)
            {
                UART_SendString("\r\nGPU WEDGE 8s -> reset\r\n");
                NVIC_SystemReset();
            }
        }
    }
    else
    {
        s_prog_last = g_gpu_dw_prog;
        s_prog_same = 0u;
        s_wedge     = 0u;
    }
#endif  /* GPU_ENABLE - the GPU-stall watchdog */

    /* 这一帧有没有写坏。 每帧读一次、清一次, 主循环那边拿到的是"坏了几帧"，
       不是"这一段出过错" —— 两种读法分开的正是两种病 (见 camera.c)。 */
    camera_dvp_frame_end_scan();

    /* 帧账先记, 换页后做 —— 顺序反过来的话 grab 那边的 c0/c1 判据会读到
       已经换过的页。 两个缓冲各一个标志, 一帧只处理一个 (if/else-if)。 */
    if (DVP_GetFlagStatus(DVPx, DVP_FLAG_M1TC) == 1)
    {
        DVP_ClrIntPendingBit(DVPx, DVP_INT_M1TC);
        camera_frame_done(DVP_IMAGE_ADDRESS1);
        lcdc_show_buffer(DVP_IMAGE_ADDRESS1);
    }
    else if (DVP_GetFlagStatus(DVPx, DVP_FLAG_M2TC) == 1)
    {
        DVP_ClrIntPendingBit(DVPx, DVP_INT_M2TC);
        camera_frame_done(DVP_IMAGE_ADDRESS2);
        lcdc_show_buffer(DVP_IMAGE_ADDRESS2);
    }
}



