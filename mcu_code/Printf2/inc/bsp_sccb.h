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
*\*\file bsp_sccb.h
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2025, Nations Technologies Inc. All rights reserved.
**/

#ifndef __BSP_SCCB_H__
#define __BSP_SCCB_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "n32h7xx.h"
#include "n32h7xx_rcc.h"
#include "n32h7xx_gpio.h"
#include "delay.h"
#include "misc.h"
#include "n32h7xx_eval.h"


#if DEMO_BOARD == N32H787_HMI

#define SCCB_SDA_IN()  {GPIOD->PMODE&=~(3<<26);GPIOD->PMODE|=0<<26;}
#define SCCB_SDA_OUT() {GPIOD->PMODE&=~(3<<26);GPIOD->PMODE|=1<<26;}

#define SCCB_SCL_H        GPIO_SetBits(GPIOD,GPIO_PIN_12)
#define SCCB_SCL_L        GPIO_ResetBits(GPIOD,GPIO_PIN_12)

#define SCCB_SDA_H        GPIO_SetBits(GPIOD,GPIO_PIN_13)
#define SCCB_SDA_L        GPIO_ResetBits(GPIOD,GPIO_PIN_13)

#define SCCB_READ_SDA     GPIO_ReadInputDataBit(GPIOD,GPIO_PIN_13)

#define SCCB_SCL_GPIO_PORT      GPIOD
#define SCCB_SCL_CLK_ENABLE()   RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOD,ENABLE)
#define SCCB_SCL_GPIO_PIN       GPIO_PIN_12
#define SCCB_SCL_DC             GPIO_DC_4mA

#define SCCB_SDA_GPIO_PORT      GPIOD
#define SCCB_SDA_CLK_ENABLE()   RCC_EnableAHB5PeriphClk1(RCC_AHB5_PERIPHEN_M7_GPIOD,ENABLE)
#define SCCB_SDA_GPIO_PIN       GPIO_PIN_13
#define SCCB_SDA_DC             GPIO_DC_4mA

#elif DEMO_BOARD == N32H787_EVB

#define SCCB_SDA_IN()  {GPIOD->PMODE&=~(3<<6);GPIOD->PMODE|=0<<6;}
#define SCCB_SDA_OUT() {GPIOD->PMODE&=~(3<<6);GPIOD->PMODE|=1<<6;}

#define SCCB_SCL_H        GPIO_SetBits(GPIOD,GPIO_PIN_4)
#define SCCB_SCL_L        GPIO_ResetBits(GPIOD,GPIO_PIN_4)

#define SCCB_SDA_H        GPIO_SetBits(GPIOD,GPIO_PIN_3)
#define SCCB_SDA_L        GPIO_ResetBits(GPIOD,GPIO_PIN_3)

#define SCCB_READ_SDA     GPIO_ReadInputDataBit(GPIOD,GPIO_PIN_3)

#define SCCB_SCL_GPIO_PORT      GPIOJ
#define SCCB_SCL_CLK_ENABLE()   RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOJ,ENABLE)
#define SCCB_SCL_GPIO_PIN       GPIO_PIN_4
#define SCCB_SCL_DC             GPIO_5VTOL_DC_4mA

#define SCCB_SDA_GPIO_PORT      GPIOJ
#define SCCB_SDA_CLK_ENABLE()   RCC_EnableAHB5PeriphClk2(RCC_AHB5_PERIPHEN_M7_GPIOJ,ENABLE)
#define SCCB_SDA_GPIO_PIN       GPIO_PIN_3
#define SCCB_SDA_DC             GPIO_5VTOL_DC_4mA

#elif DEMO_BOARD == N32H787_EVB_PRO

#endif


void SCCB_RCC_Configuration(void);
void SCCB_GPIO_Configuration(void);
uint8_t SCCB_RD_Byte(void);
uint8_t SCCB_WR_Byte(uint8_t dat);
void SCCB_No_Ack(void);
void SCCB_Stop(void);
void SCCB_Start(void);


#ifdef __cplusplus
}
#endif

#endif /* __BSP_SCCB_H__ */

