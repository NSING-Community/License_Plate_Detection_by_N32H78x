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
*\*\file n32h7xx_i2s.h
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2025, Nations Technologies Inc. All rights reserved.
**/

#ifndef __N32H7XX_I2S_H__
#define __N32H7XX_I2S_H__

/* Includes ------------------------------------------------------------------*/
#include "n32h7xx.h"
#include "n32h7xx_rcc.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** Bit definition for I2S_RX_FIFO register **/
#define I2S_RX_FIFO            ((uint16_t)0xFFFF)

/** Bit definition for I2S_FIFO_BURST_NUM register **/
#define I2S_FIFO_BURST_NUM_TX  ((uint16_t)0x0007)   /* I2S TX FIFO Half Empty Threshold Value Configure */
#define I2S_FIFO_BURST_NUM_RX  ((uint16_t)0x0070)   /* I2S RX FIFO Half Full Threshold Value Configure */

/** Bit definition for I2S_FIFO_NUM register **/
#define I2S_FIFO_NUM_TX_CNT ((uint16_t)0x000F)      /* I2S TX FIFO Valid Data Number */
#define I2S_FIFO_NUM_RX_CNT ((uint16_t)0x00F0)      /* I2S RX FIFO Valid Data Number */

/** Bit definition for I2S_TRANS_NUM register **/
#define I2S_TRANS_NUM       ((uint16_t)0xFFFF)

/** Bit definition for I2S_I2SCFG register **/
#define I2SCFG_I2SEN        ((uint16_t)0x0001)      /* I2S Enable */
#define SPI_I2SCFG_MOD      ((uint16_t)0x0002)      /* I2S Mode */

#define I2SCFG_STD          ((uint16_t)0x000C)      /* STD[3:2] bits (I2S standard selection) */
#define I2SCFG_STDSEL1      ((uint16_t)0x0008)      /* Bit 1 */
#define I2SCFG_STDSEL0      ((uint16_t)0x0004)      /* Bit 0 */
												    
#define I2SCFG_TDATLEN      ((uint16_t)0x00C0)      /* TDATLEN[1:0] bits (Data length to be transferred) */
#define I2SCFG_TDATLEN1     ((uint16_t)0x0080)      /* Bit 1 */
#define I2SCFG_TDATLEN0     ((uint16_t)0x0040)      /* Bit 0 */
												    
#define I2SCFG_CHLEN        ((uint16_t)0x0100)      /* Channel Length */
#define I2SCFG_PCMFSYNC     ((uint16_t)0x0200)      /* PCM frame synchronization */
#define I2SCFG_CLKPOL       ((uint16_t)0x0400)      /* steady state clock polarity */
#define I2SCFG_ERR_BYPASS   ((uint16_t)0x0800)      /* PCM Long for 13bit is No bypass */

/** Bit definition for SPI_I2SPR register **/
#define I2SPR_I2SDIV        ((uint16_t)0x00FF)      /* I2S Linear prescaler */
#define I2SPR_ODD           ((uint16_t)0x0100)      /* Odd factor for the prescaler */
#define I2SPR_MCKOEN        ((uint16_t)0x0800)      /* Master Clock Output Enable */

/* -------------------------- Up is the Bit Define -------------------------------- */

/** I2s Mode select**/
#define I2S_MODE_SEL_MASK   (~SPI_I2SCFG_MOD)       /* selet spi/I2S mode bit */
#define SEL_SPI_MODE        ((uint16_t)0x0000) 
#define SEL_I2S_MODE        (SPI_I2SCFG_MOD)


#define I2SCFG_CFG      ((uint16_t)0x0030)      /* I2S Mode Configure */
#define I2SCFG_CFG1     ((uint16_t)0x0020)      /* I2S Mode Configure Bit */
#define I2SCFG_CFG0     ((uint16_t)0x0010)      /* I2S Mode Configure Bit */
/** I2sMode **/
#define I2S_MODE_SlAVE_TX         ((uint16_t)0x0000U)
#define I2S_MODE_SlAVE_RX         (I2SCFG_CFG0)
#define I2S_MODE_MASTER_TX        (I2SCFG_CFG1)
#define I2S_MODE_MASTER_RX        (I2SCFG_CFG0 | I2SCFG_CFG1)
#define I2S_MODE_MASK             (I2SCFG_CFG) /* MODCFG bit Mask */
     

																			  
/** I2S_EXT_DMA_transfer_requests **/
#define I2S_EXT_DMA_TX                ((uint16_t)0x0004U)
#define I2S_EXT_DMA_RX                ((uint16_t)0x0002U)
/** I2S_EXT_interrupts_definition **/
#define I2S_EXT_INT_TEINTEN           ((uint16_t)0x0010U)
#define I2S_EXT_INT_RNEINTEN          ((uint16_t)0x0020U)
#define I2S_EXT_INT_ERRINTEN          ((uint16_t)0x0040U)
																	    																    
/** I2S_EXT_flags_definition **/      
#define I2S_EXT_TE_FLAG     			    ((uint16_t)0x0001U) 
#define I2S_EXT_RNE_FLAG    			    ((uint16_t)0x0002U) 
#define I2S_EXT_BUSY_FLAG   			    ((uint16_t)0x0004U) 
#define I2S_EXT_OVER_FLAG   			    ((uint16_t)0x0020U) 
#define I2S_EXT_UNDER_FLAG  			    ((uint16_t)0x0040U) 
#define I2S_EXT_CHSIDE_FLAG 			    ((uint16_t)0x0080U) 
																			 
/** I2S_interrupts_definition **/
#define I2S_INT_TE                ((uint8_t)0x40)
#define I2S_INT_RNE               ((uint8_t)0x51)
#define I2S_INT_ERR               ((uint8_t)0x60)
#define I2S_INT_RXONLYC           ((uint8_t)0xEC)
#define I2S_INT_RXFIFOF           ((uint8_t)0xCB)
#define I2S_INT_RXFIFOHF          ((uint8_t)0xB9)
#define I2S_INT_TXFIFOHE          ((uint8_t)0xA8)
#define I2S_INT_MODERR            ((uint8_t)0x64)
#define I2S_INT_CRCERR            ((uint8_t)0x63)
#define I2S_INT_OVERERR           ((uint8_t)0x65)
#define I2S_INT_UNDER             ((uint8_t)0x66)
																	    

/** I2S_flags_definition **/
#define I2S_TE_FLAG           		((uint16_t)0x0001U)
#define I2S_RNE_FLAG          		((uint16_t)0x0002U)
#define I2S_BUSY_FLAG         		((uint16_t)0x0004U)
#define I2S_CRCERR_FLAG           ((uint16_t)0x0008U)
#define I2S_MODERR_FLAG           ((uint16_t)0x0010U)
#define I2S_OVER_FLAG         		((uint16_t)0x0020U)
#define I2S_UNDER_FLAG            ((uint16_t)0x0040U)
#define I2S_CHSIDE_FLAG           ((uint16_t)0x0080U)
#define I2S_TXFIFOHE_FLAG         ((uint16_t)0x0100U)
#define I2S_RXFIFOHF_FLAG         ((uint16_t)0x0200U)
#define I2S_TXFIFOE_FLAG          ((uint16_t)0x0400U)
#define I2S_RXFIFOF_FLAG          ((uint16_t)0x0800U)
#define I2S_RXONLYC_FLAG          ((uint16_t)0x1000U)

/** I2S_DMA_transfer_requests **/

#define I2S_DMA_TX                ((uint16_t)0x0004U)
#define I2S_DMA_RX                ((uint16_t)0x0002U)

/** I2S Init structure definition **/

typedef struct
{
    uint16_t I2sMode;           /*!< Specifies the I2S operating mode.
                                     This parameter can be a value of @ref I2sMode */
														    
    uint16_t Standard;          /*!< Specifies the standard used for the I2S communication.
                                     This parameter can be a value of @ref Standard */
														    
    uint16_t DataFormat;        /*!< Specifies the data format for the I2S communication.
                                     This parameter can be a value of @ref I2S_Data_Format */
														    
    uint16_t MCLKEnable;        /*!< Specifies whether the I2S MCLK output is enabled or not.
                                     This parameter can be a value of @ref I2S_MCLK_Output */
														    
    uint32_t AudioFrequency;    /*!< Specifies the frequency selected for the I2S communication.
                                     This parameter can be a value of @ref I2S_Audio_Frequency */
														    
    uint16_t CLKPOL;            /*!< Specifies the idle state of the I2S clock.
                                     This parameter can be a value of @ref I2S_Clock_Polarity */  
	  uint32_t ClkSrcFrequency;   /*!< Specifies the I2S clock source frequency in Hz.*/
} I2S_InitType;


/**  Standard **/
#define I2S_STD_PHILLIPS          ((uint16_t)0x0000U)
#define I2S_STD_MSB_ALIGN         (I2SCFG_STDSEL0)
#define I2S_STD_LSB_ALIGN         (I2SCFG_STDSEL1)
#define I2S_STD_PCM_SHORTFRAME    (I2SCFG_STDSEL0 | I2SCFG_STDSEL1)
#define I2S_STD_PCM_LONGFRAME     (I2SCFG_STDSEL0 | I2SCFG_STDSEL1 | I2SCFG_PCMFSYNC)
#define I2S_STANDARD_MASK         (I2SCFG_STD) /* STDSEL and PCMFSYNC bit Mask */


/** I2S_Data_Format **/
#define I2S_DATA_FORMAT_MASK          (~(I2SCFG_CHLEN | I2SCFG_TDATLEN)) /* CHBITS and TDATLEN bit Mask */
#define I2S_DATA_FMT_16BITS           ((uint16_t)0x0000U)
#define I2S_DATA_FMT_16BITS_EXTENDED  (I2SCFG_CHLEN)
#define I2S_DATA_FMT_24BITS           (I2SCFG_CHLEN | I2SCFG_TDATLEN0)
#define I2S_DATA_FMT_32BITS           (I2SCFG_CHLEN | I2SCFG_TDATLEN1)

/** I2S_MCLK_Output **/

#define I2S_MCLK_ENABLE              ((uint16_t)0x0800U)
#define I2S_MCLK_DISABLE             ((uint16_t)0x0000U)


/** I2S_Audio_Frequency **/

#define I2S_AUDIO_FREQ_192K           ((uint32_t)192000U)
#define I2S_AUDIO_FREQ_96K            ((uint32_t)96000U)
#define I2S_AUDIO_FREQ_48K            ((uint32_t)48000U)
#define I2S_AUDIO_FREQ_44K            ((uint32_t)44100U)
#define I2S_AUDIO_FREQ_32K            ((uint32_t)32000U)
#define I2S_AUDIO_FREQ_22K            ((uint32_t)22050U)
#define I2S_AUDIO_FREQ_16K            ((uint32_t)16000U)
#define I2S_AUDIO_FREQ_11K            ((uint32_t)11025U)
#define I2S_AUDIO_FREQ_8K             ((uint32_t)8000U)
#define I2S_AUDIO_FREQ_DEFAULT        ((uint32_t)2U)

/** I2S_Clock_Polarity **/
#define I2S_CLKPOL_LOW                ((uint16_t)0x0000U)
#define I2S_CLKPOL_HIGH               (SPI_I2SCFG_CLKPOL)
#define I2S_CLKPOL_MASK               (~SPI_I2SCFG_CLKPOL) /* MCLKOEN bit Mask */

/** I2S Converter **/                   
#define I2S_TURN_ON                   (I2SCFG_I2SEN | SPI_I2SCFG_MOD)       /* I2SEN and MODSEL bit */
#define I2S_TURN_OFF                  (~(I2SCFG_I2SEN | SPI_I2SCFG_MOD))    /* I2SEN  bit Mask */

///** I2S_EXT_DMA_transfer_requests **/

#define I2S_EXT_DMA_TX                ((uint16_t)0x0004U)
#define I2S_EXT_DMA_RX                ((uint16_t)0x0002U)

void I2S_Reset(I2S_Module* I2Sx);
void I2S_EnableInt(I2S_Module* I2Sx, uint8_t I2S_IT, FunctionalState Cmd);
FlagStatus I2S_GetStatus(I2S_Module* I2Sx, uint8_t i2s_flag);
INTStatus I2S_GetIntStatus(const I2S_Module* I2sx, uint8_t I2S_IT);
void I2S_ClrITPendingBit(I2S_Module* I2Sx, uint8_t I2S_IT);
void I2S_TransmitData(I2S_Module* I2Sx, uint16_t Data);
uint16_t I2S_ReceiveData(I2S_Module* I2Sx);
void I2S_EnableDma(I2S_Module* I2Sx, uint16_t I2S_DMAReq, FunctionalState Cmd);
void I2S_Init(I2S_Module* I2Sx,const I2S_InitType* I2S_InitStruct);
void I2S_Enable(I2S_Module* I2Sx, FunctionalState Cmd);
void I2S_InitStruct(I2S_InitType* I2S_InitStruct);
void I2S_MclkEnable(I2S_Module* I2Sx, FunctionalState Cmd);
void I2S_StandardConfig(I2S_Module* I2Sx, uint16_t Standard);
void I2S_ModeConfig(I2S_Module* I2Sx, uint16_t I2sMode);
void I2S_DataFormatConfig(I2S_Module* I2Sx, uint16_t DataFormat);
void I2S_ClkPolConfig(I2S_Module* I2Sx, uint16_t CLKPOL);
void I2S_AudioFrequencyConfig(I2S_Module* I2Sx, uint32_t AudioFrequency);
void I2S_EXTInit(I2S_EXT_Module* I2Sx_EXT,const I2S_InitType* I2S_EXT_InitStruct);
void I2S_EXTInitStruct(I2S_InitType* I2S_EXT_InitStruct);
void I2S_EXTEnable(I2S_EXT_Module* I2Sx_EXT, FunctionalState Cmd);
void I2S_EXTTransmitData(I2S_EXT_Module* I2Sx, uint16_t Data);
uint16_t  I2S_EXTReceiveData(const I2S_EXT_Module* I2Sx);
FlagStatus I2S_EXTGetStatus(const I2S_EXT_Module* I2Sx, uint16_t I2S_EXT_FLAG);
void I2S_EXTEnableDma(I2S_EXT_Module* I2Sx, uint16_t I2S_EXT_DMAReq, FunctionalState Cmd);
void I2S_EXTEnableInt(I2S_EXT_Module* I2Sx, uint8_t I2S_EXT_IT, FunctionalState Cmd);
void I2S_SPIModeSelect(I2S_Module* I2Sx, uint16_t Mode);
INTStatus I2S_EXTGetIntStatus(const I2S_EXT_Module* I2Sx, uint8_t I2S_EXT_IT);
void I2S_DataFifoTransmit(I2S_Module* I2Sx, uint16_t Data);
uint16_t I2S_DataFifoGet(I2S_Module* I2Sx);
void I2S_EXTClrITPendingBit(I2S_EXT_Module* I2Sx, uint8_t I2S_EXT_IT);

#ifdef __cplusplus
}

#endif
#endif /*__N32H7XX_I2S_H__ */


