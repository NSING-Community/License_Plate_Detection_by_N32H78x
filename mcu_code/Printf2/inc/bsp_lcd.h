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
 *\*\file bsp_lcd.h
 *\*\author Nations
 *\*\version v1.0.0
 *\*\copyright Copyright (c) 2025, Nations Technologies Inc. All rights reserved.
 **/

#ifndef __BSP_LCD_H__
#define __BSP_LCD_H__

#ifdef __cplusplus
extern "C" {
#endif


#include "n32h7xx.h"
#include "n32h7xx_rcc.h"
#include "n32h7xx_gpio.h"
#include "n32h7xx_lcdc.h"
#include "n32h7xx_pwr.h"
#include "misc.h"
#include "n32h7xx_eval.h"

typedef enum
{ 
    INCH_5  = 0x00, 
    LCD_TYPE_NUM 
}LCD_Type;

typedef struct
{
    /*LCD panel data sheet*/
    uint8_t hbp;  
    uint8_t vbp;  

    uint8_t hsw;      
    uint8_t vsw;   

    uint8_t hfp;      
    uint8_t vfp;  

    uint16_t lcd_pixel_width; 
    uint16_t lcd_pixel_height;
}LCD_Panel_Type;

extern LCD_Type cur_lcd;
#define LCD_PIXEL_WIDTH                  lcd_param[cur_lcd].lcd_pixel_width
#define LCD_PIXEL_HEIGHT                 lcd_param[cur_lcd].lcd_pixel_height
#define HBP                              lcd_param[cur_lcd].hbp        
#define VBP                              lcd_param[cur_lcd].vbp        
#define HSW                              lcd_param[cur_lcd].hsw        
#define VSW                              lcd_param[cur_lcd].vsw        
#define HFP                              lcd_param[cur_lcd].hfp        
#define VFP                              lcd_param[cur_lcd].vfp    

/* The vendor demo hard-codes its frame buffers here as 0xC0000000 / 0xC00BB900.
   On this board 0xC0000000 is the YOLO tensor arena, and the single camera
   frame buffer is CAM_FRAME_BUFFER_ADDRESS in bsp_dvp.h, so the two macros are
   gone rather than redefined into a different meaning.  LCDC_Layer_Configuration()
   takes the buffer address as a parameter anyway.                              */

#if DEMO_BOARD == N32H787_HMI

#define LCDC_PWR_ENABLE()                PWR_MoudlePowerEnable(GRAPHIC_LCDC_PWRCTRL,ENABLE)
#define LCDC_CLK_ENABLE()                RCC_EnableAXIPeriphClk2(RCC_AXI_PERIPHEN_M7_LCDC | RCC_AXI_PERIPHEN_M7_LCDCLP |\
                                                                 RCC_AXI_PERIPHEN_M7_LCDCAPB | RCC_AXI_PERIPHEN_M7_LCDCAPBLP,ENABLE);
#define LCDC_PIXEL_CLK_CONFIG()          RCC_ConfigLCDCPixelClk(RCC_LCDCPIXELCLK_SRC_AXIDIV,RCC_LCDCPIXELCLK_AXIDIV8);

/* R0 */
#define LCDC_R0_GPIO_PORT                GPIOI
#define LCDC_R0_GPIO_CLK_ENABLE()        RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOI,ENABLE)
#define LCDC_R0_GPIO_PIN                 GPIO_PIN_15
#define LCDC_R0_AF                       GPIO_AF2
#define LCDC_R0_DC                       GPIO_5VTOL_DC_4mA

/* R1 */
#define LCDC_R1_GPIO_PORT                GPIOJ
#define LCDC_R1_GPIO_CLK_ENABLE()        RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOJ,ENABLE)
#define LCDC_R1_GPIO_PIN                 GPIO_PIN_0
#define LCDC_R1_AF                       GPIO_AF1
#define LCDC_R1_DC                       GPIO_5VTOL_DC_4mA

/* R2 */
#define LCDC_R2_GPIO_PORT                GPIOJ
#define LCDC_R2_GPIO_CLK_ENABLE()        RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOJ,ENABLE)
#define LCDC_R2_GPIO_PIN                 GPIO_PIN_1
#define LCDC_R2_AF                       GPIO_AF2
#define LCDC_R2_DC                       GPIO_DC_4mA

 /* R3 */ 
#define LCDC_R3_GPIO_PORT                GPIOJ
#define LCDC_R3_GPIO_CLK_ENABLE()        RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOJ,ENABLE)
#define LCDC_R3_GPIO_PIN                 GPIO_PIN_2
#define LCDC_R3_AF                       GPIO_AF1
#define LCDC_R3_DC                       GPIO_DC_4mA

/* R4 */
#define LCDC_R4_GPIO_PORT                GPIOJ
#define LCDC_R4_GPIO_CLK_ENABLE()        RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOJ,ENABLE)
#define LCDC_R4_GPIO_PIN                 GPIO_PIN_3
#define LCDC_R4_AF                       GPIO_AF1
#define LCDC_R4_DC                       GPIO_5VTOL_DC_4mA

/* R5 */
#define LCDC_R5_GPIO_PORT                GPIOJ
#define LCDC_R5_GPIO_CLK_ENABLE()        RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOJ,ENABLE)
#define LCDC_R5_GPIO_PIN                 GPIO_PIN_4
#define LCDC_R5_AF                       GPIO_AF1
#define LCDC_R5_DC                       GPIO_5VTOL_DC_4mA

/* R6 */
#define LCDC_R6_GPIO_PORT                GPIOJ
#define LCDC_R6_GPIO_CLK_ENABLE()        RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOJ,ENABLE)
#define LCDC_R6_GPIO_PIN                 GPIO_PIN_5
#define LCDC_R6_AF                       GPIO_AF1
#define LCDC_R6_DC                       GPIO_5VTOL_DC_4mA

/* R7 */
#define LCDC_R7_GPIO_PORT                GPIOJ
#define LCDC_R7_GPIO_CLK_ENABLE()        RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOJ,ENABLE)
#define LCDC_R7_GPIO_PIN                 GPIO_PIN_6
#define LCDC_R7_AF                       GPIO_AF1
#define LCDC_R7_DC                       GPIO_5VTOL_DC_4mA

/* G0 */
#define LCDC_G0_GPIO_PORT                GPIOJ
#define LCDC_G0_GPIO_CLK_ENABLE()        RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOJ,ENABLE)
#define LCDC_G0_GPIO_PIN                 GPIO_PIN_7
#define LCDC_G0_AF                       GPIO_AF1
#define LCDC_G0_DC                       GPIO_5VTOL_DC_4mA

/* G1 */
#define LCDC_G1_GPIO_PORT                GPIOJ
#define LCDC_G1_GPIO_CLK_ENABLE()        RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOJ,ENABLE)
#define LCDC_G1_GPIO_PIN                 GPIO_PIN_8
#define LCDC_G1_AF                       GPIO_AF1
#define LCDC_G1_DC                       GPIO_5VTOL_DC_4mA

/* G2 */
#define LCDC_G2_GPIO_PORT                GPIOJ
#define LCDC_G2_GPIO_CLK_ENABLE()        RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOJ,ENABLE)
#define LCDC_G2_GPIO_PIN                 GPIO_PIN_9
#define LCDC_G2_AF                       GPIO_AF1
#define LCDC_G2_DC                       GPIO_5VTOL_DC_4mA

/* G3 */
#define LCDC_G3_GPIO_PORT                GPIOJ
#define LCDC_G3_GPIO_CLK_ENABLE()        RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOJ,ENABLE)
#define LCDC_G3_GPIO_PIN                 GPIO_PIN_10
#define LCDC_G3_AF                       GPIO_AF1
#define LCDC_G3_DC                       GPIO_5VTOL_DC_4mA

/* G4 */
#define LCDC_G4_GPIO_PORT                GPIOJ
#define LCDC_G4_GPIO_CLK_ENABLE()        RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOJ,ENABLE)
#define LCDC_G4_GPIO_PIN                 GPIO_PIN_11
#define LCDC_G4_AF                       GPIO_AF1
#define LCDC_G4_DC                       GPIO_5VTOL_DC_4mA

/* G5 */
#define LCDC_G5_GPIO_PORT                GPIOK
#define LCDC_G5_GPIO_CLK_ENABLE()        RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOK,ENABLE)
#define LCDC_G5_GPIO_PIN                 GPIO_PIN_0
#define LCDC_G5_AF                       GPIO_AF1
#define LCDC_G5_DC                       GPIO_5VTOL_DC_4mA

/* G6 */
#define LCDC_G6_GPIO_PORT                GPIOI
#define LCDC_G6_GPIO_CLK_ENABLE()        RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOK,ENABLE)
#define LCDC_G6_GPIO_PIN                 GPIO_PIN_11
#define LCDC_G6_AF                       GPIO_AF4
#define LCDC_G6_DC                       GPIO_DC_4mA

/* G7 */
#define LCDC_G7_GPIO_PORT                GPIOD
#define LCDC_G7_GPIO_CLK_ENABLE()        RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOD,ENABLE)
#define LCDC_G7_GPIO_PIN                 GPIO_PIN_3
#define LCDC_G7_AF                       GPIO_AF2
#define LCDC_G7_DC                       GPIO_5VTOL_DC_4mA

/* B0 */
#define LCDC_B0_GPIO_PORT                GPIOJ
#define LCDC_B0_GPIO_CLK_ENABLE()        RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOJ,ENABLE)
#define LCDC_B0_GPIO_PIN                 GPIO_PIN_12
#define LCDC_B0_AF                       GPIO_AF2
#define LCDC_B0_DC                       GPIO_5VTOL_DC_4mA

/* B1 */
#define LCDC_B1_GPIO_PORT                GPIOJ
#define LCDC_B1_GPIO_CLK_ENABLE()        RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOJ,ENABLE)
#define LCDC_B1_GPIO_PIN                 GPIO_PIN_13
#define LCDC_B1_AF                       GPIO_AF2
#define LCDC_B1_DC                       GPIO_5VTOL_DC_4mA

/* B2 */
#define LCDC_B2_GPIO_PORT                GPIOJ
#define LCDC_B2_GPIO_CLK_ENABLE()        RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOJ,ENABLE)
#define LCDC_B2_GPIO_PIN                 GPIO_PIN_14
#define LCDC_B2_AF                       GPIO_AF1
#define LCDC_B2_DC                       GPIO_5VTOL_DC_4mA

/* B3 */
#define LCDC_B3_GPIO_PORT                GPIOJ
#define LCDC_B3_GPIO_CLK_ENABLE()        RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOJ,ENABLE)
#define LCDC_B3_GPIO_PIN                 GPIO_PIN_15
#define LCDC_B3_AF                       GPIO_AF1
#define LCDC_B3_DC                       GPIO_5VTOL_DC_4mA

/* B4 */
#define LCDC_B4_GPIO_PORT                GPIOK
#define LCDC_B4_GPIO_CLK_ENABLE()        RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOK,ENABLE)
#define LCDC_B4_GPIO_PIN                 GPIO_PIN_3
#define LCDC_B4_AF                       GPIO_AF2
#define LCDC_B4_DC                       GPIO_DC_4mA

/* B5 */
#define LCDC_B5_GPIO_PORT                GPIOK
#define LCDC_B5_GPIO_CLK_ENABLE()        RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOK,ENABLE)
#define LCDC_B5_GPIO_PIN                 GPIO_PIN_4
#define LCDC_B5_AF                       GPIO_AF2
#define LCDC_B5_DC                       GPIO_DC_4mA

/* B6 */
#define LCDC_B6_GPIO_PORT                GPIOA
#define LCDC_B6_GPIO_CLK_ENABLE()        RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOA,ENABLE)
#define LCDC_B6_GPIO_PIN                 GPIO_PIN_15
#define LCDC_B6_AF                       GPIO_AF0
#define LCDC_B6_DC                       GPIO_5VTOL_DC_4mA

/* B7 */
#define LCDC_B7_GPIO_PORT                GPIOB
#define LCDC_B7_GPIO_CLK_ENABLE()        RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOB,ENABLE)
#define LCDC_B7_GPIO_PIN                 GPIO_PIN_9
#define LCDC_B7_AF                       GPIO_AF3
#define LCDC_B7_DC                       GPIO_5VTOL_DC_4mA

/* pixel CLK */
#define LCDC_CLK_GPIO_PORT               GPIOG
#define LCDC_CLK_GPIO_CLK_ENABLE()       RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOG,ENABLE)
#define LCDC_CLK_GPIO_PIN                GPIO_PIN_7
#define LCDC_CLK_AF                      GPIO_AF3
#define LCDC_LCK_DC                      GPIO_DC_4mA

/* HSYNC */
#define LCDC_HSYNC_GPIO_PORT             GPIOI
#define LCDC_HSYNC_GPIO_CLK_ENABLE()     RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOI,ENABLE)
#define LCDC_HSYNC_GPIO_PIN              GPIO_PIN_12
#define LCDC_HSYNC_AF                    GPIO_AF2
#define LCDC_HSYNC_DC                    GPIO_DC_4mA

/* VSYNC */
#define LCDC_VSYNC_GPIO_PORT             GPIOI
#define LCDC_VSYNC_GPIO_CLK_ENABLE()     RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOI,ENABLE)
#define LCDC_VSYNC_GPIO_PIN              GPIO_PIN_13
#define LCDC_VSYNC_AF                    GPIO_AF2
#define LCDC_VSYNC_DC                    GPIO_DC_4mA

/* DE */
#define LCDC_DE_GPIO_PORT                GPIOK
#define LCDC_DE_GPIO_CLK_ENABLE()        RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOK,ENABLE)
#define LCDC_DE_GPIO_PIN                 GPIO_PIN_7
#define LCDC_DE_AF                       GPIO_AF2
#define LCDC_DE_DC                       GPIO_DC_4mA

/* DISP */
#define LCDC_DISP_GPIO_PORT              GPIOB
#define LCDC_DISP_GPIO_CLK_ENABLE()      RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOB,ENABLE)
#define LCDC_DISP_GPIO_PIN               GPIO_PIN_10
#define LCDC_DISP_DC                     GPIO_DC_4mA

/* BL */
#define LCDC_BL_GPIO_PORT                GPIOH
#define LCDC_BL_GPIO_CLK_ENABLE()        RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOH,ENABLE)
#define LCDC_BL_GPIO_PIN                 GPIO_PIN_3
#define LCDC_BL_DC                       GPIO_HS_IO_DC_4mA

#elif DEMO_BOARD == N32H787_EVB
/* Not supportted in this board */
#elif DEMO_BOARD == N32H787_EVB_PRO
/* Not supportted in this board */
#else
/* No process */
#endif


void LCDC_RCC_Configuration(void);
void LCDC_GPIO_Configuration(void);
void LCDC_Glober_Configuration(void);
void LCDC_Layer_Configuration(LCDC_Layer_Module* LCDC_Layerx, uint16_t lcdwidth, uint16_t lcdheight, uint32_t lcdbufferaddr);



#ifdef __cplusplus
}
#endif

#endif /* __BSP_LCD_H__ */

