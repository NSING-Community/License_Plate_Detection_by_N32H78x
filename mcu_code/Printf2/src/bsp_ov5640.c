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
*\*\file bsp_ov5640.c
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2023, Nations Technologies Inc. All rights reserved.
**/

#include "bsp_ov5640.h"
#include "bsp_ov5640_reg.h"
#include <stdio.h>
#include "Message.h"

/**
*\*\name    OV5640_Init.
*\*\fun     OV5640 init.
*\*\param   none
*\*\return  none 
**/
uint8_t OV5640_Init(void)
{
    uint16_t i=0;
    uint16_t reg=0;
    
    OV5640_GPIO_Configuration();
    
    GPIO_SetBits(CAMERA_RST_GPIO_PORT, CAMERA_RST_GPIO_PIN );     
    GPIO_ResetBits(CAMERA_RST_GPIO_PORT, CAMERA_RST_GPIO_PIN );   
    systick_delay_ms(41);
    
    GPIO_SetBits(CAMERA_RST_GPIO_PORT, CAMERA_RST_GPIO_PIN ); 
    systick_delay_ms(41);

    reg=OV5640_RD_Reg(OV5640_CHIPIDH);    
    reg<<=8;
    reg|=OV5640_RD_Reg(OV5640_CHIPIDL);    
#if BOOT_VERBOSE
    sprintf(Message, "camera ID: 0x%x\r\n", reg);
    UART_SendString(Message);
#endif
    if(reg!=OV5640_ID)
    {
        return 1;
    } 
    
    OV5640_WR_Reg(0x3103,0X11); //system clock from pad, bit[1]
    OV5640_WR_Reg(0X3008,0X82); //software reset, 
    systick_delay_ms(10);
   
    for(i=0;i<(sizeof(ov5640_init_reg_tb)/sizeof(ov5640_init_reg_tb[0]));i++)
    {
        OV5640_WR_Reg(ov5640_init_reg_tb[i][0],ov5640_init_reg_tb[i][1]);
    }    
    systick_delay_ms(50);
    
    return 0x00;         //ok    
    
}

/**
*\*\name    OV5640_GPIO_Configuration.
*\*\fun     Configures the OV5640 pins.
*\*\param   none
*\*\return  none 
**/
void OV5640_GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;


    CAMERA_RST_GPIO_CLK_ENABLE();

    GPIO_InitStruct(&GPIO_InitStructure);

    /*RST: PB4*/
    GPIO_InitStructure.Pin = CAMERA_RST_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Current   = CAMERA_RST_DC;
    GPIO_InitPeripheral(CAMERA_RST_GPIO_PORT, &GPIO_InitStructure);  
}

/**
*\*\name    OV5640_RGB565_Mode.
*\*\fun     OV5640 RGB565 mode.
*\*\param   none
*\*\return  none 
**/
void OV5640_RGB565_Mode(void) 
{
    uint16_t i=0;

    for(i=0;i<(sizeof(ov5640_rgb565_reg_tb)/sizeof(ov5640_rgb565_reg_tb[0]));i++)
    {
        OV5640_WR_Reg(ov5640_rgb565_reg_tb[i][0],ov5640_rgb565_reg_tb[i][1]); 
    } 

} 

/**
*\*\name    OV5640_RGB565_Config.
*\*\fun     OV5640 RGB565 config.
*\*\param   none
*\*\return  none 
**/
void OV5640_RGB565_Config(void)
{
    OV5640_Flash_Ctrl(1);
    systick_delay_ms(100);
    OV5640_Flash_Ctrl(0);
    OV5640_RGB565_Mode();        
    OV5640_Focus_Init();         
    OV5640_Light_Mode(0);       
    OV5640_Color_Saturation(3); 
    OV5640_Brightness(4);       
    OV5640_Contrast(3);         
    OV5640_Sharpness(33);       
    OV5640_Focus_Constant();    
    OV5640_Flash_Ctrl(1);
    systick_delay_ms(100);
    OV5640_Flash_Ctrl(0);
}

/**
*\*\name    OV5640_RD_Reg.
*\*\fun     OV5640 read register.
*\*\param   reg
*\*\return  none 
**/
uint8_t OV5640_RD_Reg(uint16_t reg)
{
    uint8_t val=0;
    SCCB_Start();              
    SCCB_WR_Byte(OV5640_ADDR);  
    SCCB_WR_Byte(reg>>8);       
    SCCB_WR_Byte(reg);          
    SCCB_Stop();   

    SCCB_Start();
    SCCB_WR_Byte(OV5640_ADDR|0X01); 
    val=SCCB_RD_Byte();             
    SCCB_No_Ack();
    SCCB_Stop();
    return val;
}

/**
*\*\name    OV5640_WR_Reg.
*\*\fun     OV5640 write register.
*\*\param   reg
*\*\param   data
*\*\return  none 
**/
uint8_t OV5640_WR_Reg(uint16_t reg,uint8_t data)
{
    uint8_t res=0;
    SCCB_Start();                      
    if(SCCB_WR_Byte(OV5640_ADDR))res=1;   
    if(SCCB_WR_Byte(reg>>8))res=1;      
    if(SCCB_WR_Byte(reg))res=1;           
    if(SCCB_WR_Byte(data))res=1;         
    SCCB_Stop();
    return    res;
}

/**
*\*\name    OV5640_Flash_Ctrl.
*\*\fun     OV5640 flash ctrl.
*\*\param   sw
*\*\return  none 
**/
void OV5640_Flash_Ctrl(uint8_t sw)
{
    OV5640_WR_Reg(0x3016,0X02);
    OV5640_WR_Reg(0x301C,0X02); 
    if(sw)
    {
        OV5640_WR_Reg(0X3019,0X02);
    } 
    else
    {
        OV5640_WR_Reg(0X3019,0X00);
    }
} 

/**
*\*\name    OV5640_Focus_Init.
*\*\fun     OV5640 focus init.
*\*\param   none
*\*\return  none 
**/
uint8_t OV5640_Focus_Init(void)
{ 
    uint16_t i; 
    uint16_t addr=0x8000;
    uint8_t state=0x8F;
    OV5640_WR_Reg(0x3000, 0x20);             
    for(i=0;i<sizeof(OV5640_AF_Config);i++) 
    {
        OV5640_WR_Reg(addr,OV5640_AF_Config[i]);
        addr++;
    }  
    OV5640_WR_Reg(0x3022,0x00);
    OV5640_WR_Reg(0x3023,0x00);
    OV5640_WR_Reg(0x3024,0x00);
    OV5640_WR_Reg(0x3025,0x00);
    OV5640_WR_Reg(0x3026,0x00);
    OV5640_WR_Reg(0x3027,0x00);
    OV5640_WR_Reg(0x3028,0x00);
    OV5640_WR_Reg(0x3029,0x7f);
    OV5640_WR_Reg(0x3000,0x00); 
    i=0;
    do
    {
        state=OV5640_RD_Reg(0x3029);    
        systick_delay_us(10);         //about 5ms
        systick_delay_us(10);
        systick_delay_us(10);
        systick_delay_us(10);
        systick_delay_us(10);
        systick_delay_us(10);
        systick_delay_us(10);
        systick_delay_us(10);
        i++;
        if(i>1000)return 1;
    }while(state!=0x70); 
    return 0;    
}

/**
*\*\name    OV5640_Light_Mode.
*\*\fun     OV5640 light mode.
*\*\param   mode
*\*\return  none 
**/
void OV5640_Light_Mode(uint8_t mode)
{
    uint8_t i;
    OV5640_WR_Reg(0x3212,0x03);                                             
    for(i=0;i<7;i++)OV5640_WR_Reg(0x3400+i,OV5640_LIGHTMODE_TBL[mode][i]);  
    OV5640_WR_Reg(0x3212,0x13);                                             
    OV5640_WR_Reg(0x3212,0xa3);                                             
}

/**
*\*\name    OV5640_Color_Saturation.
*\*\fun     OV5640 color saturation.
*\*\param   sat
*\*\return  none 
**/
void OV5640_Color_Saturation(uint8_t sat)
{ 
    uint8_t i;
    OV5640_WR_Reg(0x3212,0x03);                                            
    OV5640_WR_Reg(0x5381,0x1c);
    OV5640_WR_Reg(0x5382,0x5a);
    OV5640_WR_Reg(0x5383,0x06);
    for(i=0;i<6;i++)OV5640_WR_Reg(0x5384+i,OV5640_SATURATION_TBL[sat][i]);  
    OV5640_WR_Reg(0x538b, 0x98);
    OV5640_WR_Reg(0x538a, 0x01);
    OV5640_WR_Reg(0x3212, 0x13);                                          
    OV5640_WR_Reg(0x3212, 0xa3);                                            
}

/**
*\*\name    OV5640_Brightness.
*\*\fun     OV5640 brightness.
*\*\param   brightness
*\*\return  none 
**/
void OV5640_Brightness(uint8_t bright)
{
    uint8_t brtval;
    if(bright<4)brtval=4-bright;
    else brtval=bright-4;
    OV5640_WR_Reg(0x3212,0x03);             //start group 3
    OV5640_WR_Reg(0x5587,brtval<<4);
    if(bright<4)OV5640_WR_Reg(0x5588,0x09);
    else OV5640_WR_Reg(0x5588,0x01);
    OV5640_WR_Reg(0x3212,0x13);             //end group 3
    OV5640_WR_Reg(0x3212,0xa3);             //launch group 3
}

/**
*\*\name    OV5640_Contrast.
*\*\fun     OV5640 contrast.
*\*\param   contrast
*\*\return  none 
**/
void OV5640_Contrast(uint8_t contrast)
{
    uint8_t reg0val=0X00;
    uint8_t reg1val=0X20;
    switch(contrast)
    {
        case 0://-3
            reg1val=reg0val=0X14;
            break;    
        case 1://-2
            reg1val=reg0val=0X18;
            break;    
        case 2://-1
            reg1val=reg0val=0X1C;
            break;
        case 4://1
            reg0val=0X10;
            reg1val=0X24;
            break;
        case 5://2
            reg0val=0X18;
            reg1val=0X28;
            break;    
        case 6://3
            reg0val=0X1C;
            reg1val=0X2C;
            break;
    } 
    OV5640_WR_Reg(0x3212,0x03); //start group 3
    OV5640_WR_Reg(0x5585,reg0val);
    OV5640_WR_Reg(0x5586,reg1val); 
    OV5640_WR_Reg(0x3212,0x13); //end group 3
    OV5640_WR_Reg(0x3212,0xa3); //launch group 3
}

/**
*\*\name    OV5640_Sharpness.
*\*\fun     OV5640 sharpness.
*\*\param   sharp
*\*\return  none 
**/
void OV5640_Sharpness(uint8_t sharp)
{
    if(sharp<33)
    {
        OV5640_WR_Reg(0x5308,0x65);
        OV5640_WR_Reg(0x5302,sharp);
    }else    
    {
        OV5640_WR_Reg(0x5308,0x25);
        OV5640_WR_Reg(0x5300,0x08);
        OV5640_WR_Reg(0x5301,0x30);
        OV5640_WR_Reg(0x5302,0x10);
        OV5640_WR_Reg(0x5303,0x00);
        OV5640_WR_Reg(0x5309,0x08);
        OV5640_WR_Reg(0x530a,0x30);
        OV5640_WR_Reg(0x530b,0x04);
        OV5640_WR_Reg(0x530c,0x06);
    }

}

/**
*\*\name    OV5640_Focus_Constant.
*\*\fun     OV5640 focus constant.
*\*\param   none
*\*\return  none 
**/
uint8_t OV5640_Focus_Constant(void)
{
    uint8_t temp=0;   
    u16 retry=0; 
    OV5640_WR_Reg(0x3023,0x01);
    OV5640_WR_Reg(0x3022,0x08);
    do 
    {
        temp=OV5640_RD_Reg(0x3023); 
        retry++;
        if(retry>1000)return 2;
        systick_delay_ms(5);
    } while(temp!=0x00);   
    OV5640_WR_Reg(0x3023,0x01);
    OV5640_WR_Reg(0x3022,0x04);
    retry=0;
    do 
    {
        temp=OV5640_RD_Reg(0x3023); 
        retry++;
        if(retry>1000)return 2;
        systick_delay_ms(5);
    }while(temp!=0x00);
    return 0;
} 



