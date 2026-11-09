/**
*     Copyright (c) 2023, Nations Technologies Inc.
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
*\*\file bsp_dvp.c
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2023, Nations Technologies Inc. All rights reserved.
**/

#include "bsp_dvp.h"

/* 写 FIFO 的形状。 2026-10-19 上板读数 (log.txt 第 17/21/25/... 行):
     推理段 err i 恒为 030 = M1FO|M2FO —— 两个帧缓冲的写 FIFO 都溢出;
     抓帧段 err g 恒为 000, LCDC 的 fu/be/rfe 也全是 0。
   也就是说屏上的坏帧是**相机自己写坏的**, 不是 LCDC 读错的。 每个像素
   2 字节、每帧 768 KB, 8 字节一个 AHB 突发就是每帧 96,000 次写事务;
   推理同时在 SDRAM 上跑张量, 仲裁一忙, 1 KB 的 FIFO 约 87 us 就填满。

   一次动两个量, 判据只有一个: err i 是否回到 000。
     BurstSize     8 -> 64 B   拿到总线时一次搬完, 事务数降到 1/8
     FifoThreshold 8 -> 256 B  攒够再发, 对总线延迟的容忍度按比例上升
                               (必须 >= BurstSize)
   回退就是把这两个宏改回 *_BYTE_8 —— 这是全部改动, 没有别的副作用。 */
#define DVP_CFG_BURST_SIZE     DVP_BURST_SIZE_BYTE_64
#define DVP_CFG_FIFO_THRESHOLD DVP_FIFO_THRSEHOLD_BYTE_256


/**
*\*\name    DVP_Configuration.
*\*\fun     Configures the DVP.
*\*\param   none
*\*\return  none 
**/
void DVP_Configuration(void)
{
    DVP_InitType DVP_InitStructure;
    DVP_DMAInitType DVP_DMAInitStructure;
    
    DVP_StructInit(&DVP_InitStructure);
    DVP_InitStructure.CaptureMode      = DVP_CAPTURE_MODE_CONTINUE;
    DVP_InitStructure.CropMode         = DVP_NORMAL_MODE;
    DVP_InitStructure.VBFilter         = 3;
    DVP_InitStructure.PixelClkPolarity = DVP_PIXEL_POLARITY_FALLING;
    DVP_InitStructure.HsyncPolarity    = DVP_HSYNC_POLARITY_HIGH;
    DVP_InitStructure.VsyncPolarity    = DVP_VSYNC_POLARITY_LOW;
    DVP_InitStructure.DataInvert       = DVP_DATA_NOTINVERT;
    DVP_InitStructure.DataReverse      = DVP_DATA_NOTREVERSE;
    DVP_InitStructure.DataMode         = DVP_DATA_MODE1;
    DVP_InitStructure.PixelByte        = DVP_PIXEL_BYTE1;
   
    DVP_Init(DVPx,&DVP_InitStructure);
    DVP_DMAStructInit(&DVP_DMAInitStructure);
    /* Two buffers, ping-ponged by the M1TC/M2TC frame-end interrupts (see
       DVPx_IRQHandler).  This is what the vendor demo does and it is the reason
       the LCD preview is stable: with one buffer the DVP rewrites the very rows
       the LCDC is scanning, so the tear line rolls down the panel and the
       picture reads as stutter.  The second 768 KB is why the tensor arena is
       2 MB instead of 7.25 MB - see MDK-ARM/M7_TCM.sct.                     */
    DVP_DMAInitStructure.FrameBufferNum      = 2;
    DVP_DMAInitStructure.FrameBufferAddress1 = DVP_IMAGE_ADDRESS1;
    DVP_DMAInitStructure.FrameBufferAddress2 = DVP_IMAGE_ADDRESS2;
    DVP_DMAInitStructure.DisplayStatus       = DVP_DISPLAY_ADDRESS;
    DVP_DMAInitStructure.BurstSize           = DVP_CFG_BURST_SIZE;
    DVP_DMAInitStructure.FifoThreshold       = DVP_CFG_FIFO_THRESHOLD;
    DVP_DMAInitStructure.FrameBufferSize     = DVP_IMAGE_SIZE;
    DVP_ConfigDma(DVPx,&DVP_DMAInitStructure);

    DVP_ConfigInt(DVPx,DVP_INT_M1TC,ENABLE);
    DVP_ConfigInt(DVPx,DVP_INT_M2TC,ENABLE);
    DVP_EnableBuffer1(DVPx,ENABLE);
    DVP_EnableBuffer2(DVPx,ENABLE);
}

/**
*\*\name    DVP_NVIC_Configuration.
*\*\fun     Configures the NVIC.
*\*\param   none
*\*\return  none
**/
void DVP_NVIC_Configuration(void)
{
    NVIC_InitType NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitStructure.NVIC_IRQChannel                   = DVPx_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
*\*\name    DVP_RCC_Configuration.
*\*\fun     Configures the DVP clocks.
*\*\param   none
*\*\return  none 
**/
void DVP_RCC_Configuration(void)
{
    /* DVPx, GPIOx and AFIO clocks enable */
    RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_AFIO,ENABLE);
    
#if DEMO_BOARD == N32H787_HMI
    
    DVPx_PWR_ENABLE();
    DVPx_APBCLK_ENABLE();
    DVPx_CLK_ENABLE();
    DVPx_PCLK_GPIO_CLK_ENABLE();
    DVPx_HSYNC_GPIO_CLK_ENABLE();
    DVPx_VSYNC_GPIO_CLK_ENABLE();
    DVPx_D0_GPIO_CLK_ENABLE();
    DVPx_D1_GPIO_CLK_ENABLE();
    DVPx_D2_GPIO_CLK_ENABLE();
    DVPx_D3_GPIO_CLK_ENABLE();
    DVPx_D4_GPIO_CLK_ENABLE();
    DVPx_D5_GPIO_CLK_ENABLE();
    DVPx_D6_GPIO_CLK_ENABLE();
    DVPx_D7_GPIO_CLK_ENABLE();


#elif DEMO_BOARD == N32H787_EVB

#elif DEMO_BOARD == N32H787_EVB_PRO

#endif
}

/**
*\*\name    DVP_GPIO_Configuration.
*\*\fun     Configures the DVP pins.
*\*\param   none
*\*\return  none 
**/
void DVP_GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;
    GPIO_InitStruct(&GPIO_InitStructure);
    
    /* GPIOx Configuration: Pin of DVPx */
    GPIO_InitStructure.Pin            = DVPx_PCLK_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_INPUT;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Current   = DVPx_PCLK_DC;
    GPIO_InitStructure.GPIO_Alternate = DVPx_PCLK_GPIO_AF;
    GPIO_InitPeripheral(DVPx_PCLK_GPIO_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.Pin        = DVPx_HSYNC_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = DVPx_HSYNC_DC;
    GPIO_InitStructure.GPIO_Alternate = DVPx_HSYNC_GPIO_AF;
    GPIO_InitPeripheral(DVPx_HSYNC_GPIO_PORT, &GPIO_InitStructure);
    
    /* GPIOx Configuration: Pin of DVPx */
    GPIO_InitStructure.Pin        = DVPx_VSYNC_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = DVPx_VSYNC_DC;
    GPIO_InitStructure.GPIO_Alternate = DVPx_VSYNC_GPIO_AF;
    GPIO_InitPeripheral(DVPx_VSYNC_GPIO_PORT, &GPIO_InitStructure);

    /* GPIOx Configuration: Pin of DVPx */
    GPIO_InitStructure.Pin        = DVPx_D0_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = DVPx_D0_DC;
    GPIO_InitStructure.GPIO_Alternate = DVPx_D0_GPIO_AF;
    GPIO_InitPeripheral(DVPx_D0_GPIO_PORT, &GPIO_InitStructure);

    /* GPIOx Configuration: Pin of DVPx */
    GPIO_InitStructure.Pin        = DVPx_D1_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = DVPx_D1_DC;
    GPIO_InitStructure.GPIO_Alternate = DVPx_D1_GPIO_AF;
    GPIO_InitPeripheral(DVPx_D1_GPIO_PORT, &GPIO_InitStructure);
    
    /* GPIOx Configuration: Pin of DVPx */
    GPIO_InitStructure.Pin        = DVPx_D2_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = DVPx_D2_DC;
    GPIO_InitStructure.GPIO_Alternate = DVPx_D2_GPIO_AF;
    GPIO_InitPeripheral(DVPx_D2_GPIO_PORT, &GPIO_InitStructure);

    /* GPIOx Configuration: Pin of DVPx */
    GPIO_InitStructure.Pin        = DVPx_D3_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = DVPx_D3_DC;
    GPIO_InitStructure.GPIO_Alternate = DVPx_D3_GPIO_AF;
    GPIO_InitPeripheral(DVPx_D3_GPIO_PORT, &GPIO_InitStructure);

    /* GPIOx Configuration: Pin of DVPx */
    GPIO_InitStructure.Pin        = DVPx_D4_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = DVPx_D4_DC;
    GPIO_InitStructure.GPIO_Alternate = DVPx_D4_GPIO_AF;
    GPIO_InitPeripheral(DVPx_D4_GPIO_PORT, &GPIO_InitStructure);

    /* GPIOx Configuration: Pin of DVPx */
    GPIO_InitStructure.Pin        = DVPx_D5_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = DVPx_D5_DC;
    GPIO_InitStructure.GPIO_Alternate = DVPx_D5_GPIO_AF;
    GPIO_InitPeripheral(DVPx_D5_GPIO_PORT, &GPIO_InitStructure);

    /* GPIOx Configuration: Pin of DVPx */
    GPIO_InitStructure.Pin        = DVPx_D6_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = DVPx_D6_DC;
    GPIO_InitStructure.GPIO_Alternate = DVPx_D6_GPIO_AF;
    GPIO_InitPeripheral(DVPx_D6_GPIO_PORT, &GPIO_InitStructure);

    /* GPIOx Configuration: Pin of DVPx */
    GPIO_InitStructure.Pin        = DVPx_D7_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = DVPx_D7_DC;
    GPIO_InitStructure.GPIO_Alternate = DVPx_D7_GPIO_AF;
    GPIO_InitPeripheral(DVPx_D7_GPIO_PORT, &GPIO_InitStructure);
}


