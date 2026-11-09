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
 *\*\file bsp_lcd.c
 *\*\author Nations
 *\*\version v1.0.0
 *\*\copyright Copyright (c) 2025, Nations Technologies Inc. All rights reserved.
 **/

#include "bsp_lcd.h"

const LCD_Panel_Type lcd_param[LCD_TYPE_NUM]={

  /* 5inch panel */
  {
    .hbp = 46,  
    .vbp = 23,  

    .hsw = 34,     
    .vsw = 10,    

    .hfp = 210,   
    .vfp = 22,   

    .lcd_pixel_width = ((uint16_t)800),
    .lcd_pixel_height = ((uint16_t)480),

  }
};
LCD_Type cur_lcd = INCH_5;

/**
*\*\name    LCDC_GPIO_Configuration.
*\*\fun     Configures the GPIO pins.
*\*\param   none
*\*\return  none 
**/
void LCDC_GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);
    
#if DEMO_BOARD == N32H787_HMI
    GPIO_InitStructure.Pin            = LCDC_R0_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Current   = LCDC_R0_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_R0_AF;
    GPIO_InitPeripheral(LCDC_R0_GPIO_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.Pin            = LCDC_R1_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_R1_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_R1_AF;
    GPIO_InitPeripheral(LCDC_R1_GPIO_PORT, &GPIO_InitStructure);    
    
    GPIO_InitStructure.Pin            = LCDC_R2_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_R2_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_R2_AF;
    GPIO_InitPeripheral(LCDC_R2_GPIO_PORT, &GPIO_InitStructure);      
    
    GPIO_InitStructure.Pin            = LCDC_R3_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_R3_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_R3_AF;
    GPIO_InitPeripheral(LCDC_R3_GPIO_PORT, &GPIO_InitStructure);    
    
    GPIO_InitStructure.Pin            = LCDC_R4_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_R4_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_R4_AF;
    GPIO_InitPeripheral(LCDC_R4_GPIO_PORT, &GPIO_InitStructure);    
    
    GPIO_InitStructure.Pin            = LCDC_R5_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_R5_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_R5_AF;
    GPIO_InitPeripheral(LCDC_R5_GPIO_PORT, &GPIO_InitStructure);    
    
    GPIO_InitStructure.Pin            = LCDC_R6_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_R6_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_R6_AF;
    GPIO_InitPeripheral(LCDC_R6_GPIO_PORT, &GPIO_InitStructure);    
    
    GPIO_InitStructure.Pin            = LCDC_R7_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_R7_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_R7_AF;
    GPIO_InitPeripheral(LCDC_R7_GPIO_PORT, &GPIO_InitStructure);    
    
    GPIO_InitStructure.Pin            = LCDC_G0_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_G0_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_G0_AF;
    GPIO_InitPeripheral(LCDC_G0_GPIO_PORT, &GPIO_InitStructure); 
    
    GPIO_InitStructure.Pin            = LCDC_G1_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_G1_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_G1_AF;
    GPIO_InitPeripheral(LCDC_G1_GPIO_PORT, &GPIO_InitStructure);    
    
    GPIO_InitStructure.Pin            = LCDC_G2_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_G2_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_G2_AF;
    GPIO_InitPeripheral(LCDC_G2_GPIO_PORT, &GPIO_InitStructure);      
    
    GPIO_InitStructure.Pin            = LCDC_G3_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_G3_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_G3_AF;
    GPIO_InitPeripheral(LCDC_G3_GPIO_PORT, &GPIO_InitStructure);    
    
    GPIO_InitStructure.Pin            = LCDC_G4_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_G4_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_G4_AF;
    GPIO_InitPeripheral(LCDC_G4_GPIO_PORT, &GPIO_InitStructure);    
    
    GPIO_InitStructure.Pin            = LCDC_G5_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_G5_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_G5_AF;
    GPIO_InitPeripheral(LCDC_G5_GPIO_PORT, &GPIO_InitStructure);    
    
    GPIO_InitStructure.Pin            = LCDC_G6_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_G6_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_G6_AF;
    GPIO_InitPeripheral(LCDC_G6_GPIO_PORT, &GPIO_InitStructure);    
    
    GPIO_InitStructure.Pin            = LCDC_G7_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_G7_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_G7_AF;
    GPIO_InitPeripheral(LCDC_G7_GPIO_PORT, &GPIO_InitStructure);      
    
    GPIO_InitStructure.Pin            = LCDC_B0_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_B0_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_B0_AF;
    GPIO_InitPeripheral(LCDC_B0_GPIO_PORT, &GPIO_InitStructure); 
    
    GPIO_InitStructure.Pin            = LCDC_B1_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_B1_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_B1_AF;
    GPIO_InitPeripheral(LCDC_B1_GPIO_PORT, &GPIO_InitStructure);    
    
    GPIO_InitStructure.Pin            = LCDC_B2_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_B2_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_B2_AF;
    GPIO_InitPeripheral(LCDC_B2_GPIO_PORT, &GPIO_InitStructure);      
    
    GPIO_InitStructure.Pin            = LCDC_B3_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_B3_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_B3_AF;
    GPIO_InitPeripheral(LCDC_B3_GPIO_PORT, &GPIO_InitStructure);    
    
    GPIO_InitStructure.Pin            = LCDC_B4_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_B4_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_B4_AF;
    GPIO_InitPeripheral(LCDC_B4_GPIO_PORT, &GPIO_InitStructure);    
    
    GPIO_InitStructure.Pin            = LCDC_B5_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_B5_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_B5_AF;
    GPIO_InitPeripheral(LCDC_B5_GPIO_PORT, &GPIO_InitStructure);    
    
    GPIO_InitStructure.Pin            = LCDC_B6_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_B6_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_B6_AF;
    GPIO_InitPeripheral(LCDC_B6_GPIO_PORT, &GPIO_InitStructure);    
    
    GPIO_InitStructure.Pin            = LCDC_B7_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_B7_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_B7_AF;
    GPIO_InitPeripheral(LCDC_B7_GPIO_PORT, &GPIO_InitStructure);    
    
    GPIO_InitStructure.Pin            = LCDC_CLK_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_LCK_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_CLK_AF;
    GPIO_InitPeripheral(LCDC_CLK_GPIO_PORT, &GPIO_InitStructure);    
    
    GPIO_InitStructure.Pin            = LCDC_HSYNC_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_HSYNC_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_HSYNC_AF;
    GPIO_InitPeripheral(LCDC_HSYNC_GPIO_PORT, &GPIO_InitStructure);  

    GPIO_InitStructure.Pin            = LCDC_VSYNC_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_VSYNC_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_VSYNC_AF;
    GPIO_InitPeripheral(LCDC_VSYNC_GPIO_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.Pin            = LCDC_DE_GPIO_PIN;
    GPIO_InitStructure.GPIO_Current   = LCDC_DE_DC;
    GPIO_InitStructure.GPIO_Alternate = LCDC_DE_AF;
    GPIO_InitPeripheral(LCDC_DE_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin            = LCDC_DISP_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Current   = LCDC_DISP_DC;
    GPIO_InitPeripheral(LCDC_DISP_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin            = LCDC_BL_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Current   = LCDC_BL_DC;
    GPIO_InitPeripheral(LCDC_BL_GPIO_PORT, &GPIO_InitStructure);
#elif DEMO_BOARD == N32H787_EVB
/* Not supportted in this board */
#elif DEMO_BOARD == N32H787_EVB_PRO
/* Not supportted in this board */
#else
/* No process */
#endif
}

/**
*\*\name    LCDC_RCC_Configuration.
*\*\fun     Configures the LCDC clocks.
*\*\param   none
*\*\return  none 
**/
void LCDC_RCC_Configuration(void)
{
    RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_AFIO,ENABLE);  

#if DEMO_BOARD == N32H787_HMI
    
    LCDC_PWR_ENABLE();
    LCDC_CLK_ENABLE();
    LCDC_PIXEL_CLK_CONFIG();  
    
    LCDC_R0_GPIO_CLK_ENABLE();
    LCDC_R1_GPIO_CLK_ENABLE();
    LCDC_R2_GPIO_CLK_ENABLE();
    LCDC_R3_GPIO_CLK_ENABLE();
    LCDC_R4_GPIO_CLK_ENABLE();
    LCDC_R5_GPIO_CLK_ENABLE();
    LCDC_R6_GPIO_CLK_ENABLE();
    LCDC_R7_GPIO_CLK_ENABLE();
    
    LCDC_G0_GPIO_CLK_ENABLE();
    LCDC_G1_GPIO_CLK_ENABLE();
    LCDC_G2_GPIO_CLK_ENABLE();
    LCDC_G3_GPIO_CLK_ENABLE();
    LCDC_G4_GPIO_CLK_ENABLE();
    LCDC_G5_GPIO_CLK_ENABLE();
    LCDC_G6_GPIO_CLK_ENABLE();
    LCDC_G7_GPIO_CLK_ENABLE();
    
    LCDC_B0_GPIO_CLK_ENABLE();
    LCDC_B1_GPIO_CLK_ENABLE();
    LCDC_B2_GPIO_CLK_ENABLE();
    LCDC_B3_GPIO_CLK_ENABLE();
    LCDC_B4_GPIO_CLK_ENABLE();
    LCDC_B5_GPIO_CLK_ENABLE();
    LCDC_B6_GPIO_CLK_ENABLE();
    LCDC_B7_GPIO_CLK_ENABLE();
    
    LCDC_HSYNC_GPIO_CLK_ENABLE();
    LCDC_VSYNC_GPIO_CLK_ENABLE();
    LCDC_DE_GPIO_CLK_ENABLE();
    LCDC_DISP_GPIO_CLK_ENABLE();
    LCDC_BL_GPIO_CLK_ENABLE();
    
#elif DEMO_BOARD == N32H787_EVB
/* Not supportted in this board */
#elif DEMO_BOARD == N32H787_EVB_PRO
/* Not supportted in this board */
#else
/* No process */
#endif
}

/**
*\*\name    LCDC_Glober_Configuration.
*\*\fun     LCDC glober configuration.
*\*\param   none
*\*\return  none 
**/
void LCDC_Glober_Configuration(void)
{
    LCDC_InitType LCDC_InitStruct;
    LCDC_StructInit(&LCDC_InitStruct);
    LCDC_InitStruct.LCDC_HSPolarity = LCDC_HSPOLARITY_LOW;
    LCDC_InitStruct.LCDC_VSPolarity = LCDC_VSPOLARITY_LOW;
    LCDC_InitStruct.LCDC_DEPolarity = LCDC_DEPOLARITY_LOW;
    LCDC_InitStruct.LCDC_PCPolarity = LCDC_PCPOLARITY_IPC;
    LCDC_InitStruct.LCDC_HorizontalSync = HSW-1;
    LCDC_InitStruct.LCDC_VerticalSync = VSW-1;    
    LCDC_InitStruct.LCDC_AccumulatedHBP = HSW+HBP-1;    
    LCDC_InitStruct.LCDC_AccumulatedVBP = VSW+VBP-1;    
    LCDC_InitStruct.LCDC_AccumulatedActiveW = HSW+HBP+LCD_PIXEL_WIDTH-1;    
    LCDC_InitStruct.LCDC_AccumulatedActiveH = VSW+VBP+LCD_PIXEL_HEIGHT-1;  
    LCDC_InitStruct.LCDC_TotalWidth = HSW+ HBP+LCD_PIXEL_WIDTH + HFP-1;    
    LCDC_InitStruct.LCDC_TotalHeight = VSW+ VBP+LCD_PIXEL_HEIGHT + VFP-1; 
    LCDC_InitStruct.LCDC_BackgroundRedValue = 0x00;
    LCDC_InitStruct.LCDC_BackgroundGreenValue = 0x00;
    LCDC_InitStruct.LCDC_BackgroundBlueValue = 0x00;
    LCDC_Init(&LCDC_InitStruct);    
}

/**
*\*\name    LCDC_Layer_Configuration.
*\*\fun     LCDC layer configuration.
*\*\param   LCDC_Layerx: refer to the definition of LCDC_Layer_Module
*\*\          - LCDC_Layer1
*\*\          - LCDC_Layer2
*\*\          - LCDC_Layer3
*\*\          - LCDC_Layer4
*\*\param   lcdwidth
*\*\          - LCDC layer width
*\*\param   lcdheight
*\*\          - LCDC layer height
*\*\param   lcdbufferaddr
*\*\          - LCDC layer frame buffer address
*\*\return  none 
**/
void LCDC_Layer_Configuration(LCDC_Layer_Module* LCDC_Layerx, uint16_t lcdwidth, uint16_t lcdheight, uint32_t lcdbufferaddr)
{
    LCDC_Layer_InitType LCDC_Layer_InitStruct;
    LCDC_LayerStructInit(&LCDC_Layer_InitStruct);
    LCDC_Layer_InitStruct.LCDC_HorizontalStart = HBP + HSW;
    LCDC_Layer_InitStruct.LCDC_HorizontalStop = HSW+HBP+lcdwidth-1;
    LCDC_Layer_InitStruct.LCDC_VerticalStart = VBP + VSW;
    LCDC_Layer_InitStruct.LCDC_VerticalStop = VSW+VBP+lcdheight-1;
    LCDC_Layer_InitStruct.LCDC_PixelFormat = LCDC_PIXEL_FORMAT_RGB565;
    LCDC_Layer_InitStruct.LCDC_ConstantAlpha = 255;
    LCDC_Layer_InitStruct.LCDC_DefaultAlpha  = 0x00;
    LCDC_Layer_InitStruct.LCDC_DefaultColorRed  = 0x00;
    LCDC_Layer_InitStruct.LCDC_DefaultColorGreen  = 0x00;
    LCDC_Layer_InitStruct.LCDC_DefaultColorBlue  = 0xFF;
    LCDC_Layer_InitStruct.LCDC_BlendingFactor1 = LCDC_BLENDING_FACTOR1_CA;
    LCDC_Layer_InitStruct.LCDC_BlendingFactor2 = LCDC_BLENDING_FACTOR2_CA;
    LCDC_Layer_InitStruct.LCDC_CFBLineLength = ((lcdwidth * 2) + 7);    
    LCDC_Layer_InitStruct.LCDC_CFBPitch = (lcdwidth * 2);
    LCDC_Layer_InitStruct.LCDC_CFBLineNumber = lcdheight;
    LCDC_Layer_InitStruct.LCDC_CFBStartAdress = lcdbufferaddr;
    LCDC_LayerInit(LCDC_Layerx,&LCDC_Layer_InitStruct);
    LCDC_LayerReloadConfig(LCDC_Layerx,LCDC_LAYER_IMRELOAD);    
    /* Enable layer */
    LCDC_LayerEnable(LCDC_Layerx, ENABLE); 
    LCDC_LayerReloadConfig(LCDC_Layerx,LCDC_LAYER_IMRELOAD); 
}
