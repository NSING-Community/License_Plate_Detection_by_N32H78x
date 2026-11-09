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
*\*\file bsp_ov5640.h
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2023, Nations Technologies Inc. All rights reserved.
**/
#ifndef __BSP_OV5640_H__
#define __BSP_OV5640_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_dvp.h"
#include "bsp_sccb.h"
#include "delay.h"
#include "n32h7xx_eval.h"

#if DEMO_BOARD == N32H787_HMI

#if BOARD_DVPx == BOARD_DVP1

#define CAMERA_RST_GPIO_PORT            GPIOC
#define CAMERA_RST_GPIO_CLK_ENABLE()    RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOC,ENABLE)
#define CAMERA_RST_GPIO_PIN             GPIO_PIN_13
#define CAMERA_RST_DC                   GPIO_5VTOL_DC_4mA

#elif BOARD_DVPx == BOARD_DVP2

#define CAMERA_RST_GPIO_PORT            GPIOG
#define CAMERA_RST_GPIO_CLK_ENABLE()    RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOG,ENABLE)
#define CAMERA_RST_GPIO_PIN             GPIO_PIN_6
#define CAMERA_RST_DC                   GPIO_DC_4mA

#else

#endif

#elif DEMO_BOARD == N32H787_EVB

#define CAMERA_RST_GPIO_PORT            GPIOI
#define CAMERA_RST_GPIO_CLK_ENABLE()    RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOI,ENABLE)
#define CAMERA_RST_GPIO_PIN             GPIO_PIN_8
#define CAMERA_RST_DC                   GPIO_5VTOL_DC_4mA

#elif DEMO_BOARD == N32H787_EVB_PRO

#endif


#define OV5640_ADDR         0x78    
#define OV5640_ID           0X5640  

#define OV5640_CHIPIDH          0X300A
#define OV5640_CHIPIDL          0X300B

#define DVP_OUTPUT_RGB565    0
#define DVP_OUTPUT_YUV422    1
#define DVP_OUTPUT_FORMAT    DVP_OUTPUT_RGB565




uint8_t OV5640_Init(void);
void OV5640_GPIO_Configuration(void);
void OV5640_RGB565_Config(void);
void OV5640_RGB565_Config(void);
uint8_t OV5640_RD_Reg(uint16_t reg);
uint8_t OV5640_WR_Reg(uint16_t reg,uint8_t data);
void OV5640_Flash_Ctrl(uint8_t sw);
uint8_t OV5640_Focus_Init(void);
void OV5640_Light_Mode(uint8_t mode);
void OV5640_Color_Saturation(uint8_t sat);
void OV5640_Brightness(uint8_t bright);
void OV5640_Contrast(uint8_t contrast);
void OV5640_Sharpness(uint8_t sharp);
uint8_t OV5640_Focus_Constant(void);



#ifdef __cplusplus
}
#endif

#endif /* __BSP_OV5640_H__ */

