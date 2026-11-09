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
 *\*\file nsgpu_hw.h
 *\*\author Nations
 *\*\version v1.0.0
 *\*\copyright Copyright (c) 2025, Nations Technologies Inc. All rights reserved.
 **/
#ifndef _NSGPU_HW_H_
#define _NSGPU_HW_H_


/*******************************************************************
 *                        Headers
 *******************************************************************/
#include "n32h7xx.h"
#include "nsgpu.h"
 
 
/*******************************************************************
 *                        Definitions
 ******************************************************************/
 
 #define GPU_REG_BASE                        ((uint32_t)0x50040000) /*!< GPU base address 0x5004_0000 -- 0x5004_3FFF, 16kB */
 #define GPU_VIDMEM1_BASE                    ((uint32_t)0x20010000) /*!< AHB SRAM1 base address 0x3000_0000 -- 0x3001_FFFF, 128kB */
 #define GPU_VIDMEM2_BASE                    GPU_MEMORY_BASE_ADDRESS/*!< AHB SRAM2 base address 0x3002_0000 -- 0x3003_FFFF, 128kB */
    
 #define GPU_STC_BASE                        GPU_REG_BASE
 #define GPU_PSU_BASE                        GPU_REG_BASE + 0x200
 #define GPU_TXA_BASE                        GPU_REG_BASE + 0x300
 #define GPU_ZSS_BASE                        GPU_REG_BASE + 0x400
 #define GPU_ZSA_BASE                        GPU_REG_BASE + 0x600
 #define GPU_TEX_BASE                        GPU_REG_BASE + 0x800
 #define GPU_TXP_BASE                        GPU_REG_BASE + 0xA00
 #define GPU_COL_BASE                        GPU_REG_BASE + 0xB00 
 #define GPU_BLD_BASE                        GPU_REG_BASE + 0xC00
 #define GPU_PFC_BASE                        GPU_REG_BASE + 0xE00
 #define GPU_CLR_BASE                        GPU_REG_BASE + 0xF00

#define STC                                  ((GPU_STC_TypeDef *) GPU_STC_BASE)
#define PSU                                  ((GPU_PSU_TypeDef *) GPU_PSU_BASE)
#define TXA                                  ((GPU_TXA_TypeDef *) GPU_TXA_BASE)
#define ZSS                                  ((GPU_ZSS_TypeDef *) GPU_ZSS_BASE)
#define ZSA                                  ((GPU_ZSA_TypeDef *) GPU_ZSA_BASE)
#define TEX                                  ((GPU_TEX_TypeDef *) GPU_TEX_BASE)
#define TXP                                  ((GPU_TXP_TypeDef *) GPU_TXP_BASE)
#define COL                                  ((GPU_COL_TypeDef *) GPU_COL_BASE)
#define BLD                                  ((GPU_BLD_TypeDef *) GPU_BLD_BASE)
#define PFC                                  ((GPU_PFC_TypeDef *) GPU_PFC_BASE)
#define CLR                                  ((GPU_CLR_TypeDef *) GPU_CLR_BASE)
  
 
 
   /**
      * @brief Inter Integrated Circuit Interface for register group 0
      */
    typedef struct
    {
        __IO uint32_t Version;                                //  0x00
        __IO uint32_t Config_1;    	                          //  0x04
        __IO uint32_t Config_2;	                              //  0x08
        __IO uint32_t Config_3;				                  			//  0x0c
        __IO uint32_t Busy_Status;	                          //  0x10
        __IO uint32_t Control;								  							//  0x14
        __IO uint32_t Interrupt_Status;	                      //  0x18 STC interrupt status/clear
        __IO uint32_t Stream_Address;	                      	//  0x1c
        __IO uint32_t Ring_Pause_Address;					  					//  0x20
        __IO uint32_t Current_Ring_Address;					  				//  0x24
        __IO uint32_t Current_Stream_Address;	              	//  0x28
        __IO uint32_t Current_Checksum;	                      //  0x2c
        __IO uint32_t Sync_ID_0;	                          	//  0x30
        __IO uint32_t Sync_ID_1;	                          	//  0x34
        __IO uint32_t Sync_ID_2;	                          	//  0x38
        __IO uint32_t Call_Stack_Pointer;	                  	//  0x3c
        __IO uint32_t Call_Stack_Entry[8];                    //  0x40-0x5c
        __IO uint32_t RESERVED0[8];                    	      //  0x60-0x7c		: not used
        __IO uint32_t Performance_Counter_Values[32];  				//Performance_Counter_Values[96];	  //  0x80
    } GPU_STC_TypeDef;
 

    /**
    * @brief Inter Integrated Circuit Interface for register group 2, PSU (Pixel Selection Registers), 
    */
    typedef struct
    {
        __IO uint32_t Lim_Bbox_Min;                           //  0x200, Bounding box upper left corner [27:16], [11:0],  (y, x)
        __IO uint32_t Lim_Bbox_Max;		                      	//  0x204, Bounding box lower right corner [27:16],[11:0], (y, x)
        __IO uint32_t Lim_Start;	                          	//  0x208, Start position for enumeration [27,16]
        __IO uint32_t Lim_Ctrl;				                  			//  0x20c, Pixel selection unit Limiter+Bezier control registers
        __IO uint32_t Lim_Strope;	                          	//  0x210, Enumeration strips width and offset, offset:[23:16],width:[7:0]
        __IO uint32_t Bez_Ctrl;								  							//  0x214, Bezier control, Scaling factor for Bezier limiter values: [5:0]
        __IO uint32_t Bez_Voff_0;     	                      //  0x218, Offset to Limiter value for Bezier function [31:0]
        __IO uint32_t Bez_Voff_1;     	                      //  0x21c, Offset to Limiter value for Bezier function [31:0]
        __IO uint32_t Bez_AA_Ctrl;      					  					//  0x220, Bezier anti-aliasing and width control, Aa:[25;16],Aa_Offset:[15:8],Width:[7:0]
        __IO uint32_t Lim_Vstart_0;       					  				//  0x224, Initial value of limiter #0 at enumeration start position [31:0]
        __IO uint32_t Lim_dx_0;             	              	//  0x228, Fractional part of Limiter #0 x step [19:0]
        __IO uint32_t Lim_dy_0;             	              	//  0x22c, Fractional part of Limiter #0 y step [19:0]
        __IO uint32_t Lim_Vstart_1;	                          //  0x230, Initial value of Limiter #1 of enumeration start position
        __IO uint32_t Lim_dx_1;             	              	//  0x234, Fractional part of Limiter #1 x step [19:0]
        __IO uint32_t Lim_dy_1;             	              	//  0x238, Fractional part of Limiter #1 y step [19:0]
        __IO uint32_t Lim_Vstart_2;	                          //  0x23c, Initial value of Limiter #2 at enumeration start position [31:0]
        __IO uint32_t Lim_dx_2;                               //  0x240, Fractional part of Limmiter #2 x step [19:0]
        __IO uint32_t Lim_dy_2;                               //  0x244, Fractional part of Limmiter #2 y step [19:0]
        __IO uint32_t Lim_Vstart_3;                           //  0x248, Initial value of Limiter #3 at enumeration start position [31:0]			
        __IO uint32_t Lim_dx_3;                               //  0x24c, Fractional part of Limmiter #3 x step [19:0]
        __IO uint32_t Lim_dy_3;                               //  0x250, Fractional part of Limmiter #3 y step [19:0]			
        __IO uint32_t Lim_Vstart_4;                           //  0x254, Initial value of Limiter #4 at enumeration start position [31:0]			
        __IO uint32_t Lim_dx_4;                               //  0x258, Fractional part of Limmiter #4 x step [19:0]
        __IO uint32_t Lim_dy_4;                               //  0x25c, Fractional part of Limmiter #4 y step [19:0]	
        __IO uint32_t Lim_Vstart_5;                           //  0x260, Initial value of Limiter #5 at enumeration start position [31:0]			
        __IO uint32_t Lim_dx_5;                               //  0x264, Fractional part of Limmiter #5 x step [19:0]
        __IO uint32_t Lim_dy_5;                               //  0x268, Fractional part of Limmiter #5 y step [19:0]					
        __IO uint32_t Debug_Ctrl;                             //  0x26c, Limiter debug control bits [7:4], Bezier debug control bits [3:0]
        __IO uint32_t Lim_Max_Coord;                          //  0x270, Maximum coordinates generated by PSU, clipping limits for bounding box, [27:16], [11:0]
        __IO uint32_t RESERVED1[35];			                    //  0x274 - 0x2fc

    } GPU_PSU_TypeDef;

		
		   /**
		* @brief Inter Integrated Circuit Interface for register group 3, TXI(Texel U/V Interpolation)
      */
    typedef struct
    {
        __IO uint32_t TXA_U_Offset_0;                         //  0x300, Start value for texture U coordinate of texture unit #0
        __IO uint32_t TXA_V_Offset_0;                         //  0x304, Start value for texture V coordinate of texture unit #0
        __IO uint32_t TXA_dUX_0;	                          	//  0x308, Increment of texture U coordinate for a pixel step in X direction of texture unit#0
        __IO uint32_t TXA_dUY_0;	                          	//  0x30C, Increment of texture U coordinate for a pixel step in Y direction of texture unit#0
        __IO uint32_t TXA_dVX_0;	                          	//  0x310, Increment of texture V coordinate for a pixel step in X direction of texture unit#0
        __IO uint32_t TXA_dVY_0;	                          	//  0x314, Increment of texture V coordinate for a pixel step in Y direction of texture unit#0
        __IO uint32_t TXA_U_Offset_1;                         //  0x318, Start value for texture U coordinate of texture unit #1
        __IO uint32_t TXA_V_Offset_1;                         //  0x31C, Start value for texture V coordinate of texture unit #1
        __IO uint32_t TXA_dUX_1;	                          	//  0x320, Increment of texture U coordinate for a pixel step in X direction of texture unit#1
        __IO uint32_t TXA_dUY_1;	                          	//  0x324, Increment of texture U coordinate for a pixel step in Y direction of texture unit#1
        __IO uint32_t TXA_dVX_1;	                          	//  0x328, Increment of texture V coordinate for a pixel step in X direction of texture unit#1
        __IO uint32_t TXA_dVY_1;	                          	//  0x32C, Increment of texture V coordinate for a pixel step in Y direction of texture unit#1
        __IO uint32_t RESERVED2[52];			              			//  0x330 - 0x3fc, not used

    } GPU_TXA_TypeDef;


    /**
    * @brief Inter Integrated Circuit Interface for register group 4, ZSS/ZSC (ZSA Cache + Scheduler)
    */
    typedef struct
    {
        __IO uint32_t ZSS_Pitch;                              //  0x400, ZSA cache schduler pitch [12:0]
        __IO uint32_t ZSS_Span_Config;                        //  0x404, ZSA cache scheduler span config
        __IO uint32_t TXA_Z_Offset;                           //  0x408, Start value for Z coordinate
        __IO uint32_t TXA_dZX;  	                          	//  0x40C, Increment of Z coordinate for a pixel in X direction
        __IO uint32_t TXA_dZY;	                              //  0x410, Increment of Z coordinate for a pixel step in Y direction
        __IO uint32_t TXA_Z_RHW_Offset;	                      //  0x414, Start value for RHW attribute (1/w) of Z, reset value: 0x3f800000
        __IO uint32_t TXA_Z_dRHWX;                            //  0x418, Increment of RHW of Z for a pixel step in X direction
        __IO uint32_t TXA_Z_dRHWY;                            //  0x41c, Increment of RHW of Z for a pixel step in Y direction			
        __IO uint32_t TXA_RHW_Offset_0;                       //  0x420, Start value for RHW attribute(1/w) of texture unit #0, reset value: 0x3f800000
        __IO uint32_t TXA_dRHWX_0;                            //  0x424, Increment of RHW attribute for a pixel step in X direction of texture unit #0
        __IO uint32_t TXA_dRHWY_0;                            //  0x428, Increment of RHW attribute for a pixel step in Y direction of texture unit #0
        __IO uint32_t TXA_RHW_Offset_1;                       //  0x42C, Start value for RHW attribute(1/w) of texture unit #01, reset value: 0x3f800000
        __IO uint32_t TXA_dRHWX_1;                            //  0x430, Increment of RHW attribute for a pixel step in X direction of texture unit #1
        __IO uint32_t TXA_dRHWY_1;                            //  0x434, Increment of RHW attribute for a pixel step in Y direction of texture unit #1			
        __IO uint32_t RESERVED3[50];	//RESERVED3[114];	  		//  0x438 - 0x4fc, not used

    } GPU_ZSS_TypeDef;

    /**
    * @brief Inter Integrated Circuit Interface for register group 6, ZSA Pipeline +TXA Eegisters
    */
    typedef struct
    {
        __IO uint32_t ZSA_Ctrl;                               //  0x600, ZSA conreol registers
        __IO uint32_t ZSC_Start_Address;                      //  0x604, ZSA buufer start address
        __IO uint32_t ZSU_Ctrl1;                              //  0x608, ZSA Unit Control register 1
        __IO uint32_t ZSU_Ctrl2;  	                          //  0x60C, ZSA Unit Control register 2
        __IO uint32_t TXA_Size_0;	                          	//  0x610, Texture size of texture unit #0
        __IO uint32_t TXA_Access_0;	                          //  0x614, Texture size of texture unit #0			
        __IO uint32_t TXA_Size_1;	                          	//  0x618, Texture size of texture unit #1
        __IO uint32_t TXA_Access_1;	                          //  0x61C, Texture size of texture unit #0	
        __IO uint32_t RESERVED4[56];    //RESERVED4[120];	  	//  0x620 - 0x6fc, not used

    } GPU_ZSA_TypeDef;	

    /**
    * @brief Inter Integrated Circuit Interface for register group 8, TEX/COL(Global), FBD/FBS and RLD Registers
    */
    typedef struct
    {
        __IO uint32_t TEX_Global;                             //  0x800, Global texture unit register
        __IO uint32_t COL_Global;                             //  0x804, Global color unit registers
        __IO uint32_t FBD_Config;                             //  0x808, Framebuffer read decision registers
        __IO uint32_t FBS_Pitch;  	                          //  0x80C, Framebuffer cache scheduler pitch and debug control
        __IO uint32_t FBS_Span_Config;	                      //  0x810, Framebuffer cache scheduler span config
        __IO uint32_t RLD_Start_Address;	                  	//  0x814, Start address of RLE code of the texture in memory
        __IO uint32_t COL_Const_Color_0;	                  	//  0x818, Constant color register #0
        __IO uint32_t COL_Const_Color_1;	                  	//  0x81C, Constant color register #1
        __IO uint32_t COL_Const_Color_2;	                  	//  0x820, Constant color register #2
        __IO uint32_t COL_Const_Color_3;	                  	//  0x824, Constant color register #3					
        __IO uint32_t RESERVED5[54];	//RESERVED3[118];	  		//  0x828 - 0x8fc, not used

    } GPU_TEX_TypeDef;

    /**
    * @brief Inter Integrated Circuit Interface for register group 10, TEX+TXP Registers
    */
    typedef struct
    {
        __IO uint32_t TEX_Mode_0;                             //  0xA00, Texture mode of texture unit #0
        __IO uint32_t TXC_Start_Address_0;                    //  0xA04, Start address of top-left texel of the texture in memory of texture unit #0
        __IO uint32_t TXP_Ctrl_0;                             //  0xA08, TXP control of texture unit #0
        __IO uint32_t TXP_Clut_Offset_0;                      //  0xA0C, Offset for CLUT lookups of texture unit #0
        __IO uint32_t TXP_Color_Key_0;	                      //  0xA10, Color key comparison value of texture unit #0
        __IO uint32_t TXP_Fill_Color_0; 	                  	//  0xA14, Texel fill color wrap mode FILL of tetxure unit #0
        __IO uint32_t TXP_Filter_Scale_Bias_0;                //  0xA18, Scale and bias for filter in TXP of texture unit #0
        __IO uint32_t TEX_Mode_1;	                          	//  0xA1C, Texture mode of texture unit #1
        __IO uint32_t TXC_Start_Address_1;	                  //  0xA20, Start address of top-left texel of the texture in memory of texture uitt #1
        __IO uint32_t TXP_Ctrl_1;	                          	//  0xA24, TXP control of texture unit #1
        __IO uint32_t TXP_Clut_Offset_1;	                  	//  0xA28, Offset for CLUT lookups of texture unit #1
        __IO uint32_t TXP_Color_Key_1;	                      //  0xA2C, Texel fill color for wrap mode FILL of texture unit #1
        __IO uint32_t TXP_Fill_Color_1;	                      //  0xA30, Color key comparison value of texture unit #1	
        __IO uint32_t TXP_Filter_Scale_Bias_1;	              //  0xA34, Scale and bias for filter in TXP of texture unit #1			
        __IO uint32_t RESERVED6[50];			                    //  0xA38 - 0xAfc, not used

    } GPU_TXP_TypeDef;


    /**
    * @brief Inter Integrated Circuit Interface for register group 11, COL Pass Registers
    */
    typedef struct
    {
        __IO uint32_t COL_ARGB_Op1a_0;                        //  0xB00, Control structure for Op1a of color pass 0 (Alpha and RGB path)
        __IO uint32_t COL_ARGB_Op1b_0;                        //  0xB04, Control structure for Op1b of color pass 0 (Alpha and RGB path)
        __IO uint32_t COL_ARGB_Op2a_0;                        //  0xB08, Control structure for Op2a of color pass 0
        __IO uint32_t COL_ARGB_Op2b_0;                        //  0xB0C, Control structure for Op2b of color pass 0
        __IO uint32_t COL_ARGB_Op3_0;                         //  0xB10, Control structure for Op3 of color pass 0
        __IO uint32_t COL_ARGB_Out_0;                         //  0xB14, Control structure for core output of color pass 0 
        __IO uint32_t COL_ARGB_Op1a_1;                        //  0xB18, Control structure for Op1a of color pass 1
        __IO uint32_t COL_ARGB_Op1b_1;	                      //  0xB1C, Control structure for Op1b of color pass 1
        __IO uint32_t COL_ARGB_Op2a_1;	                      //  0xB20, Control structure for Op2a of color pass 1
        __IO uint32_t COL_ARGB_Op2b_1;	                      //  0xB24, Control structure for Op2b of color pass 1
        __IO uint32_t COL_ARGB_Op3_1 ;	                      //  0xB28, Control structure for Op3 of color pass 1
        __IO uint32_t COL_ARGB_Out_1;	                      	//  0xB2C, Control structure for core output of color pass 1 
        __IO uint32_t COL_ARGB_Op1a_2;	                      //  0xB30, Control structure for Op1a of color pass 2
        __IO uint32_t COL_ARGB_Op1b_2;	                      //  0xB34, Control structure for Op1b of color pass 2	
        __IO uint32_t COL_ARGB_Op2a_2;	                      //  0xB38, Control structure for Op2a of color pass 2			
        __IO uint32_t COL_ARGB_Op2b_2;	                      //  0xB3C, Control structure for Op2b of color pass 2			
        __IO uint32_t COL_ARGB_Op3_2;	                      	//  0xB40, Control structure for Op3 of color pass 2	
        __IO uint32_t COL_ARGB_Out_2;	                      	//  0xB44, Control structure for core output of color pass 2			
        __IO uint32_t RESERVED7[46];                          //  0xB48 - 0xBFC, not used

    } GPU_COL_TypeDef;


    /**
    * @brief Inter Integrated Circuit Interface for register group 12, Blending Pipeline Registers
    */
    typedef struct
    {
        __IO uint32_t FB_Pixel_Org;                           //  0xC00, Framebuffer pixel organization/format
        __IO uint32_t FBC_Start_Address;                      //  0xC04, Framebuffer start address
        __IO uint32_t BLU_Blend;                              //  0xC08, Blend unit factor and mode
        __IO uint32_t BLU_Dither;                             //  0xC0C, Blend unit dither control
        __IO uint32_t BLU_Write;	                            //  0xC10, Blend unit write control			 		
        __IO uint32_t RESERVED8[59];			                    //  0xC - 0xAfc, not used

    } GPU_BLD_TypeDef;
    
        
    /**
    * @brief Inter Integrated Circuit Interface for register group 14, Performance Counter Registers
    */
    typedef struct
    {
        __IO uint32_t PFC_Enable;                             //  0xE00, Performance counter enable
        __IO uint32_t PFC_Clear;                              //  0xE04, Performance counter clear
        __IO uint32_t PFC_Event_Select_0;                     //  0xE08, Performance counter #0 event selection
        __IO uint32_t PFC_Event_Select_1;                     //  0xE0C, Performance counter #1 event selection
        __IO uint32_t PFC_Event_Select_2;                     //  0xE10, Performance counter #2 event selection			
        __IO uint32_t PFC_Event_Select_3;                     //  0xE14, Performance counter #3 event selection	
        __IO uint32_t RESERVED8[58];                          //  0xE18 - 0xEFC, not used
                
    } GPU_PFC_TypeDef;
        
     /**
     * @brief Inter Integrated Circuit Interface for register group 15, Clear unit Registers
     */
    typedef struct
    {
        __IO uint32_t CLR_Value;                              //  0xF00, Clear unit color value
        __IO uint32_t CLR_Line_Config;                        //  0xF04, Clear unit number of lines
        __IO uint32_t CLR_Ctrl;                               //  0xF08, Clear unit number of lines			
        __IO uint32_t CLR_Start_Address;                      //  0xF0C, Start address for clear unit
        __IO uint32_t RESERVED8[61];			              			//  0xE18 - 0xEFC, not used

    } GPU_CLR_TypeDef;		
        

#endif   // _NSGPU_HW_H



