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
*\*\file bsp_sccb.c
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2025, Nations Technologies Inc. All rights reserved.
**/


#include "bsp_sccb.h"



/**
*\*\name    SCCB_RCC_Configuration.
*\*\fun     Configures the gpio clock of SCCB.
*\*\param   none
*\*\return  none 
**/
void SCCB_RCC_Configuration(void)
{
    SCCB_SCL_CLK_ENABLE();
    SCCB_SDA_CLK_ENABLE();
}

/**
*\*\name    SCCB_GPIO_Configuration.
*\*\fun     Configures the SCCB pins.
*\*\param   none
*\*\return  none 
**/
void SCCB_GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;
    GPIO_InitStruct(&GPIO_InitStructure);
    /*SCCB Pin*/
    GPIO_InitStructure.Pin = SCCB_SCL_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Current   = SCCB_SCL_DC;
    GPIO_InitPeripheral(SCCB_SCL_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = SCCB_SDA_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Current   = SCCB_SDA_DC;
    GPIO_InitPeripheral(SCCB_SDA_GPIO_PORT, &GPIO_InitStructure);
}

/**
*\*\name    SCCB_Start.
*\*\fun     SCCB start.
*\*\param   none
*\*\return  none 
**/
void SCCB_Start(void)
{
    SCCB_SDA_H;     
    SCCB_SCL_H;    
    systick_delay_us(10);
    SCCB_SDA_L;
    systick_delay_us(10);
    SCCB_SCL_L;    
}

/**
*\*\name    SCCB_Start.
*\*\fun     SCCB stop.
*\*\param   none
*\*\return  none 
**/
void SCCB_Stop(void)
{
    SCCB_SDA_L;
    systick_delay_us(10);
    SCCB_SCL_H;
    systick_delay_us(10);
    SCCB_SDA_H;
    systick_delay_us(10);
}

/**
*\*\name    SCCB_No_Ack.
*\*\fun     SCCB ACK.
*\*\param   none
*\*\return  none 
**/
void SCCB_No_Ack(void)
{
    systick_delay_us(10);
    SCCB_SDA_H;
    SCCB_SCL_H;
    systick_delay_us(10);
    SCCB_SCL_L;
    systick_delay_us(10);
    SCCB_SDA_L;
    systick_delay_us(10);
}

/**
*\*\name    SCCB_WR_Byte.
*\*\fun     SCCB write byte.
*\*\param   dat
*\*\return  none 
**/
uint8_t SCCB_WR_Byte(uint8_t dat)
{
    uint8_t j,res;	 
    for(j=0;j<8;j++) 
    {
        if(dat&0x80)
        {
            SCCB_SDA_H;
        }
        else
        {
            SCCB_SDA_L;
        }
        dat<<=1;
        systick_delay_us(10);
        SCCB_SCL_H;
        systick_delay_us(10);
        SCCB_SCL_L;
    } 
    SCCB_SDA_IN();    
    systick_delay_us(10);
    SCCB_SCL_H;    
    systick_delay_us(10);
    if(SCCB_READ_SDA)
    {
        res=1;  
    }
    else
    {
        res=0;         
    }
    SCCB_SCL_L;
    SCCB_SDA_OUT();   
    return res;  
}

/**
*\*\name    SCCB_RD_Byte.
*\*\fun     SCCB read byte.
*\*\param   none
*\*\return  none 
**/
uint8_t SCCB_RD_Byte(void)
{
    u8 temp=0,j;    
    SCCB_SDA_IN();     
    for(j=8;j>0;j--)    
    {
        systick_delay_us(10);
        SCCB_SCL_H;
        temp=temp<<1;
        if(SCCB_READ_SDA)temp++;   
        systick_delay_us(10);
        SCCB_SCL_L;
    }
    SCCB_SDA_OUT();     
    return temp;
}

