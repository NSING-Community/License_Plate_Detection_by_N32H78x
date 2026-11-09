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
*\*\file bsp_dvp.h
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2023, Nations Technologies Inc. All rights reserved.
**/
#ifndef __BSP_DVP_H__
#define __BSP_DVP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "n32h7xx.h"
#include "n32h7xx_rcc.h"
#include "n32h7xx_gpio.h"
#include "n32h7xx_dvp.h"
#include "misc.h"
#include "bsp_ov5640.h"
#include "n32h7xx_dma.h"
#include "n32h7xx_pwr.h"
#include "bsp_lcd.h"
#include "n32h7xx_eval.h"

/* Camera frame buffers: two of them, 800x480 RGB565 = 768,000 bytes each, just
   above the YOLO tensor arena, which owns 0xC0000000..0xC0200000 (see
   MDK-ARM/M7_TCM.sct).  They have to live outside the arena, and at fixed
   addresses rather than in a linker section, so that the arena's size stays a
   single number in one place.
   The DVP ping-pongs between the two and the LCDC scans out whichever one has
   just finished, which is what keeps the preview from tearing: with a single
   buffer the DVP would be rewriting the very rows the LCDC is reading.        */
#define CAM_FRAME_BUFFER_ADDRESS        ((uint32_t)0xC0200000)
#define CAM_FRAME_BUFFER2_ADDRESS       ((uint32_t)(CAM_FRAME_BUFFER_ADDRESS + (800u * 480u * 2u)))
#define CAM_FRAME_BUFFER_SIZE           (800u * 480u * 2u)

#if DEMO_BOARD == N32H787_HMI

#define BOARD_DVP1                      (1U)
#define BOARD_DVP2                      (2U)

/* OV5640 is wired to DVP1 on this board (the vendor demo defaults to DVP2). */
#define BOARD_DVPx                      (BOARD_DVP1)

#if BOARD_DVPx == BOARD_DVP1
#define DVPx_IRQn                       DVP1_IRQn
#define DVPx_IRQHandler                 DVP1_IRQHandler

#define DVP_IMAGE_FORMAT_GRAY           (1)
#define DVP_IMAGE_FORMAT_RGB565         (2)

#define DVP_IMAGE_PIXEL_SIZE            (DVP_IMAGE_FORMAT_RGB565)
#define DVP_IMAGE_WIDTH                 800
#define DVP_IMAGE_HEIGHT                480
#define IMAGE_WIDTH_H                   0x03
#define IMAGE_WIDTH_L                   0x20  
#define IMAGE_HEIGHT_H                  0x01
#define IMAGE_HEIGHT_L                  0xE0

#define DVP_IMAGE_SIZE                  (DVP_IMAGE_WIDTH * DVP_IMAGE_HEIGHT * DVP_IMAGE_PIXEL_SIZE)
#define DVP_IMAGE_BUFF_SIZE             (DVP_IMAGE_SIZE)

#define DVP_IMAGE_ADDRESS1              (CAM_FRAME_BUFFER_ADDRESS)
#define DVP_IMAGE_ADDRESS2              (CAM_FRAME_BUFFER2_ADDRESS)

#define DVPx                            DVP1
#define DVPx_PWR_ENABLE()               PWR_MoudlePowerEnable(GRAPHIC_DVP_PWRCTRL,ENABLE)
#define DVPx_APBCLK_ENABLE()            RCC_EnableAXIPeriphClk2(RCC_AXI_PERIPHEN_M7_DVP1APB,ENABLE)
#define DVPx_CLK_ENABLE()               RCC_EnableAXIPeriphClk2(RCC_AXI_PERIPHEN_M7_DVP1,ENABLE)

#define DVPx_PCLK_GPIO_PORT             GPIOA
#define DVPx_PCLK_GPIO_CLK_ENABLE()     RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOA,ENABLE)
#define DVPx_PCLK_GPIO_PIN              GPIO_PIN_6
#define DVPx_PCLK_GPIO_AF               GPIO_AF3
#define DVPx_PCLK_DC                    GPIO_DC_4mA

#define DVPx_HSYNC_GPIO_PORT            GPIOA
#define DVPx_HSYNC_GPIO_CLK_ENABLE()    RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOA,ENABLE)
#define DVPx_HSYNC_GPIO_PIN             GPIO_PIN_4
#define DVPx_HSYNC_GPIO_AF              GPIO_AF3
#define DVPx_HSYNC_DC                   GPIO_DC_4mA

#define DVPx_VSYNC_GPIO_PORT            GPIOB
#define DVPx_VSYNC_GPIO_CLK_ENABLE()    RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOB,ENABLE)
#define DVPx_VSYNC_GPIO_PIN             GPIO_PIN_7
#define DVPx_VSYNC_GPIO_AF              GPIO_AF3
#define DVPx_VSYNC_DC                   GPIO_DC_4mA

#define DVPx_D0_GPIO_PORT               GPIOC
#define DVPx_D0_GPIO_CLK_ENABLE()       RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOC,ENABLE)
#define DVPx_D0_GPIO_PIN                GPIO_PIN_6
#define DVPx_D0_GPIO_AF                 GPIO_AF5
#define DVPx_D0_DC                      GPIO_5VTOL_DC_4mA

#define DVPx_D1_GPIO_PORT               GPIOC
#define DVPx_D1_GPIO_CLK_ENABLE()       RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOC,ENABLE)
#define DVPx_D1_GPIO_PIN                GPIO_PIN_7
#define DVPx_D1_GPIO_AF                 GPIO_AF5
#define DVPx_D1_DC                      GPIO_5VTOL_DC_4mA

#define DVPx_D2_GPIO_PORT               GPIOB
#define DVPx_D2_GPIO_CLK_ENABLE()       RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOB,ENABLE)
#define DVPx_D2_GPIO_PIN                GPIO_PIN_13
#define DVPx_D2_GPIO_AF                 GPIO_AF3
#define DVPx_D2_DC                      GPIO_DC_4mA

#define DVPx_D3_GPIO_PORT               GPIOG
#define DVPx_D3_GPIO_CLK_ENABLE()       RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOG,ENABLE)
#define DVPx_D3_GPIO_PIN                GPIO_PIN_11
#define DVPx_D3_GPIO_AF                 GPIO_AF5
#define DVPx_D3_DC                      GPIO_DC_4mA

#define DVPx_D4_GPIO_PORT               GPIOE
#define DVPx_D4_GPIO_CLK_ENABLE()       RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOE,ENABLE)
#define DVPx_D4_GPIO_PIN                GPIO_PIN_4
#define DVPx_D4_GPIO_AF                 GPIO_AF3
#define DVPx_D4_DC                      GPIO_5VTOL_DC_4mA

#define DVPx_D5_GPIO_PORT               GPIOB
#define DVPx_D5_GPIO_CLK_ENABLE()       RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOB,ENABLE)
#define DVPx_D5_GPIO_PIN                GPIO_PIN_6
#define DVPx_D5_GPIO_AF                 GPIO_AF3
#define DVPx_D5_DC                      GPIO_HS_IO_DC_4mA

#define DVPx_D6_GPIO_PORT               GPIOE
#define DVPx_D6_GPIO_CLK_ENABLE()       RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOE,ENABLE)
#define DVPx_D6_GPIO_PIN                GPIO_PIN_5
#define DVPx_D6_GPIO_AF                 GPIO_AF3
#define DVPx_D6_DC                      GPIO_DC_4mA

#define DVPx_D7_GPIO_PORT               GPIOE
#define DVPx_D7_GPIO_CLK_ENABLE()       RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOE,ENABLE)
#define DVPx_D7_GPIO_PIN                GPIO_PIN_6
#define DVPx_D7_GPIO_AF                 GPIO_AF2
#define DVPx_D7_DC                      GPIO_5VTOL_DC_4mA

#elif BOARD_DVPx == BOARD_DVP2
#define DVPx_IRQn                       DVP2_IRQn
#define DVPx_IRQHandler                 DVP2_IRQHandler

#define DVP_IMAGE_FORMAT_GRAY           (1)
#define DVP_IMAGE_FORMAT_RGB565         (2)

#define DVP_IMAGE_PIXEL_SIZE            (DVP_IMAGE_FORMAT_RGB565)
#define DVP_IMAGE_WIDTH                 800
#define DVP_IMAGE_HEIGHT                480
#define IMAGE_WIDTH_H                   0x03
#define IMAGE_WIDTH_L                   0x20  
#define IMAGE_HEIGHT_H                  0x01
#define IMAGE_HEIGHT_L                  0xE0

#define DVP_IMAGE_SIZE                  (DVP_IMAGE_WIDTH * DVP_IMAGE_HEIGHT * DVP_IMAGE_PIXEL_SIZE)
#define DVP_IMAGE_BUFF_SIZE             (DVP_IMAGE_SIZE)

#define DVP_IMAGE_ADDRESS1              (CAM_FRAME_BUFFER_ADDRESS)
#define DVP_IMAGE_ADDRESS2              (CAM_FRAME_BUFFER2_ADDRESS)

#define DVPx                            DVP2
#define DVPx_PWR_ENABLE()               PWR_MoudlePowerEnable(GRAPHIC_DVP_PWRCTRL,ENABLE)
#define DVPx_APBCLK_ENABLE()            RCC_EnableAXIPeriphClk2(RCC_AXI_PERIPHEN_M7_DVP2APB,ENABLE)
#define DVPx_CLK_ENABLE()               RCC_EnableAXIPeriphClk2(RCC_AXI_PERIPHEN_M7_DVP2,ENABLE)

#define DVPx_PCLK_GPIO_PORT             GPIOH
#define DVPx_PCLK_GPIO_CLK_ENABLE()     RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOH,ENABLE)
#define DVPx_PCLK_GPIO_PIN              GPIO_PIN_5
#define DVPx_PCLK_GPIO_AF               GPIO_AF2
#define DVPx_PCLK_DC                    GPIO_HS_IO_DC_4mA

#define DVPx_HSYNC_GPIO_PORT            GPIOH
#define DVPx_HSYNC_GPIO_CLK_ENABLE()    RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOH,ENABLE)
#define DVPx_HSYNC_GPIO_PIN             GPIO_PIN_4
#define DVPx_HSYNC_GPIO_AF              GPIO_AF3
#define DVPx_HSYNC_DC                   GPIO_5VTOL_DC_4mA

#define DVPx_VSYNC_GPIO_PORT            GPIOF
#define DVPx_VSYNC_GPIO_CLK_ENABLE()    RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOF,ENABLE)
#define DVPx_VSYNC_GPIO_PIN             GPIO_PIN_9
#define DVPx_VSYNC_GPIO_AF              GPIO_AF4
#define DVPx_VSYNC_DC                   GPIO_DC_4mA

#define DVPx_D0_GPIO_PORT               GPIOE
#define DVPx_D0_GPIO_CLK_ENABLE()       RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOE,ENABLE)
#define DVPx_D0_GPIO_PIN                GPIO_PIN_2
#define DVPx_D0_GPIO_AF                 GPIO_AF3
#define DVPx_D0_DC                      GPIO_DC_4mA

#define DVPx_D1_GPIO_PORT               GPIOE
#define DVPx_D1_GPIO_CLK_ENABLE()       RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOE,ENABLE)
#define DVPx_D1_GPIO_PIN                GPIO_PIN_3
#define DVPx_D1_GPIO_AF                 GPIO_AF2
#define DVPx_D1_DC                      GPIO_DC_4mA

#define DVPx_D2_GPIO_PORT               GPIOK
#define DVPx_D2_GPIO_CLK_ENABLE()       RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOK,ENABLE)
#define DVPx_D2_GPIO_PIN                GPIO_PIN_5
#define DVPx_D2_GPIO_AF                 GPIO_AF3
#define DVPx_D2_DC                      GPIO_DC_4mA

#define DVPx_D3_GPIO_PORT               GPIOK
#define DVPx_D3_GPIO_CLK_ENABLE()       RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOK,ENABLE)
#define DVPx_D3_GPIO_PIN                GPIO_PIN_6
#define DVPx_D3_GPIO_AF                 GPIO_AF3
#define DVPx_D3_DC                      GPIO_DC_4mA

#define DVPx_D4_GPIO_PORT               GPIOI
#define DVPx_D4_GPIO_CLK_ENABLE()       RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOI,ENABLE)
#define DVPx_D4_GPIO_PIN                GPIO_PIN_14
#define DVPx_D4_GPIO_AF                 GPIO_AF3
#define DVPx_D4_DC                      GPIO_DC_4mA

#define DVPx_D5_GPIO_PORT               GPIOK
#define DVPx_D5_GPIO_CLK_ENABLE()       RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOK,ENABLE)
#define DVPx_D5_GPIO_PIN                GPIO_PIN_2
#define DVPx_D5_GPIO_AF                 GPIO_AF2
#define DVPx_D5_DC                      GPIO_5VTOL_DC_4mA

#define DVPx_D6_GPIO_PORT               GPIOK
#define DVPx_D6_GPIO_CLK_ENABLE()       RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOK,ENABLE)
#define DVPx_D6_GPIO_PIN                GPIO_PIN_1
#define DVPx_D6_GPIO_AF                 GPIO_AF2
#define DVPx_D6_DC                      GPIO_5VTOL_DC_4mA

#define DVPx_D7_GPIO_PORT               GPIOC
#define DVPx_D7_GPIO_CLK_ENABLE()       RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOC,ENABLE)
#define DVPx_D7_GPIO_PIN                GPIO_PIN_3
#define DVPx_D7_GPIO_AF                 GPIO_AF5
#define DVPx_D7_DC                      GPIO_HS_IO_DC_4mA
#else

#endif

#elif DEMO_BOARD == N32H787_EVB
#define DVPx_IRQn                       DVP2_IRQn
#define DVPx_IRQHandler                 DVP2_IRQHandler

#define DVP_IMAGE_FORMAT_GRAY           (1)
#define DVP_IMAGE_FORMAT_RGB565         (2)

#define DVP_IMAGE_PIXEL_SIZE            (DVP_IMAGE_FORMAT_RGB565)
#define DVP_IMAGE_WIDTH                 800
#define DVP_IMAGE_HEIGHT                480
#define IMAGE_WIDTH_H                   0x03
#define IMAGE_WIDTH_L                   0x20  
#define IMAGE_HEIGHT_H                  0x01
#define IMAGE_HEIGHT_L                  0xE0

#define DVP_IMAGE_SIZE                  (DVP_IMAGE_WIDTH * DVP_IMAGE_HEIGHT * DVP_IMAGE_PIXEL_SIZE)
#define DVP_IMAGE_BUFF_SIZE             (DVP_IMAGE_SIZE)

#define DVP_IMAGE_ADDRESS1              (CAM_FRAME_BUFFER_ADDRESS)
#define DVP_IMAGE_ADDRESS2              (CAM_FRAME_BUFFER2_ADDRESS)

#define DVPx                            DVP2
#define DVPx_PWR_ENABLE()               PWR_MoudlePowerEnable(GRAPHIC_DVP_PWRCTRL,ENABLE)
#define DVPx_APBCLK_ENABLE()            RCC_EnableAXIPeriphClk2(RCC_AXI_PERIPHEN_M7_DVP2APB,ENABLE)
#define DVPx_CLK_ENABLE()               RCC_EnableAXIPeriphClk2(RCC_AXI_PERIPHEN_M7_DVP2,ENABLE)

#define DVPx_PCLK_GPIO_PORT             GPIOF
#define DVPx_PCLK_GPIO_CLK_ENABLE()     RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOF,ENABLE)
#define DVPx_PCLK_GPIO_PIN              GPIO_PIN_7
#define DVPx_PCLK_GPIO_AF               GPIO_AF3
#define DVPx_PCLK_DC                    GPIO_DC_4mA

#define DVPx_HSYNC_GPIO_PORT            GPIOF
#define DVPx_HSYNC_GPIO_CLK_ENABLE()    RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOF,ENABLE)
#define DVPx_HSYNC_GPIO_PIN             GPIO_PIN_8
#define DVPx_HSYNC_GPIO_AF              GPIO_AF4
#define DVPx_HSYNC_DC                   GPIO_DC_4mA

#define DVPx_VSYNC_GPIO_PORT            GPIOF
#define DVPx_VSYNC_GPIO_CLK_ENABLE()    RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOF,ENABLE)
#define DVPx_VSYNC_GPIO_PIN             GPIO_PIN_9
#define DVPx_VSYNC_GPIO_AF              GPIO_AF4
#define DVPx_VSYNC_DC                   GPIO_DC_4mA

#define DVPx_D0_GPIO_PORT               GPIOK
#define DVPx_D0_GPIO_CLK_ENABLE()       RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOK,ENABLE)
#define DVPx_D0_GPIO_PIN                GPIO_PIN_3
#define DVPx_D0_GPIO_AF                 GPIO_AF3
#define DVPx_D0_DC                      GPIO_DC_4mA

#define DVPx_D1_GPIO_PORT               GPIOE
#define DVPx_D1_GPIO_CLK_ENABLE()       RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOE,ENABLE)
#define DVPx_D1_GPIO_PIN                GPIO_PIN_3
#define DVPx_D1_GPIO_AF                 GPIO_AF2
#define DVPx_D1_DC                      GPIO_DC_4mA

#define DVPx_D2_GPIO_PORT               GPIOI
#define DVPx_D2_GPIO_CLK_ENABLE()       RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOI,ENABLE)
#define DVPx_D2_GPIO_PIN                GPIO_PIN_12
#define DVPx_D2_GPIO_AF                 GPIO_AF3
#define DVPx_D2_DC                      GPIO_DC_4mA

#define DVPx_D3_GPIO_PORT               GPIOK
#define DVPx_D3_GPIO_CLK_ENABLE()       RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOK,ENABLE)
#define DVPx_D3_GPIO_PIN                GPIO_PIN_6
#define DVPx_D3_GPIO_AF                 GPIO_AF3
#define DVPx_D3_DC                      GPIO_DC_4mA

#define DVPx_D4_GPIO_PORT               GPIOK
#define DVPx_D4_GPIO_CLK_ENABLE()       RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOK,ENABLE)
#define DVPx_D4_GPIO_PIN                GPIO_PIN_7
#define DVPx_D4_GPIO_AF                 GPIO_AF3
#define DVPx_D4_DC                      GPIO_DC_4mA

#define DVPx_D5_GPIO_PORT               GPIOK
#define DVPx_D5_GPIO_CLK_ENABLE()       RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOK,ENABLE)
#define DVPx_D5_GPIO_PIN                GPIO_PIN_2
#define DVPx_D5_GPIO_AF                 GPIO_AF2
#define DVPx_D5_DC                      GPIO_5VTOL_DC_4mA

#define DVPx_D6_GPIO_PORT               GPIOC
#define DVPx_D6_GPIO_CLK_ENABLE()       RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOC,ENABLE)
#define DVPx_D6_GPIO_PIN                GPIO_PIN_2
#define DVPx_D6_GPIO_AF                 GPIO_AF5
#define DVPx_D6_DC                      GPIO_HS_IO_DC_4mA

#define DVPx_D7_GPIO_PORT               GPIOC
#define DVPx_D7_GPIO_CLK_ENABLE()       RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOC,ENABLE)
#define DVPx_D7_GPIO_PIN                GPIO_PIN_3
#define DVPx_D7_GPIO_AF                 GPIO_AF5
#define DVPx_D7_DC                      GPIO_HS_IO_DC_4mA
#elif DEMO_BOARD == N32H787_EVB_PRO

#endif



void DVP_RCC_Configuration(void);
void DVP_NVIC_Configuration(void);
void DVP_GPIO_Configuration(void);
void DVP_Configuration(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_DVP_H__ */

