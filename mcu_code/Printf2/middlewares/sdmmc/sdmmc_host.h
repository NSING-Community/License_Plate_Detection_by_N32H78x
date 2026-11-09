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
 *\*\file sdmmc_host.h
 *\*\author Nations
 *\*\version v1.0.0
 *\*\copyright Copyright (c) 2025, Nations Technologies Inc. All rights reserved.
 *\*\brief SD/MMC host driver header for SD card and eMMC device support
 **/

#ifndef __SDMMC_HOST_H__
#define __SDMMC_HOST_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "sdmmc_spec.h"
#include "n32h7xx_sdmmc.h"

/* ========================== Common Definitions ========================== */



/* DMA mode */
typedef enum
{
    SDMMC_NODMA = 0U,  /* no use dma */
    SDMMC_SDMA = 1U,   /* use sdma */
    SDMMC_ADMA = 2U,   /* use adma */
} sd_dma_t;

/* Card status */
typedef enum
{
    Status_Success = 1,    /* Generic status for Success */
    Status_Fail,           /* Generic status for Fail */
    Status_PollingCardIdleFailed, /* polling card idle status failed */
    Status_CardStatusIdle, /* card idle */
    Status_CardStatusBusy, /* card busy */
    Status_CardOutOfRange, /* Out of range access. */
    Status_CardNotSupportYet, /* Haven't supported. */
    Status_CardSwitchFailed, /* Switch command failed */
} Status_card;



/* Card cmd13 retry times */
#define SD_CMD13_RETRY_TIMES (10)
#define MMC_CMD13_RETRY_TIMES (10)

/* Card access idle timeout value */
#define SD_CARD_ACCESS_WAIT_IDLE_TIMEOUT (600U) /* more then 500ms timeout value */
#define MMC_CARD_ACCESS_WAIT_IDLE_TIMEOUT (10000U)

/* ========================== SD Card Specific Definitions ========================== */

/* SD card individual commands */
typedef enum
{
    SD_SendRelativeAddress    = 3U,  /* Send Relative Address */
    SD_Switch                 = 6U,  /* Switch Function */
    SD_SendInterfaceCondition = 8U,  /* Send Interface Condition */
    SD_VoltageSwitch          = 11U, /* Voltage Switch */
    SD_SpeedClassControl      = 20U, /* Speed Class control */
    SD_EraseWriteBlockStart   = 32U, /* Write Block Start */
    SD_EraseWriteBlockEnd     = 33U, /* Write Block End */
    SD_SendTuningBlock        = 19U, /* Send Tuning Block */
} sd_command_t;

/* SD card individual application commands */
typedef enum
{
    SD_ApplicationSetBusWdith             = 6U,  /* Set Bus Width */
    SD_ApplicationStatus                  = 13U, /* Send SD status */
    SD_ApplicationSendNumberWriteBlocks   = 22U, /* Send Number Of Written Blocks */
    SD_ApplicationSetWriteBlockEraseCount = 23U, /* Set Write Block Erase Count */
    SD_ApplicationSendOperationCondition  = 41U, /* Send Operation Condition */
    SD_ApplicationSetClearCardDetect      = 42U, /* Set Connnect/Disconnect pull up on detect pin */
    SD_ApplicationSendScr                 = 51U, /* Send Scr */
} sd_application_command_t;

/* SD card flags */
enum
{
    SD_SupportHighCapacityFlag     = (1U << 1U), /* Support high capacity */
    SD_Support4BitWidthFlag        = (1U << 2U), /* Support 4-bit data width */
    SD_SupportSdhcFlag             = (1U << 3U), /* Card is SDHC */
    SD_SupportSdxcFlag             = (1U << 4U), /* Card is SDXC */
    SD_SupportVoltage180v          = (1U << 5U), /* card support 1.8v voltage*/
    SD_SupportSetBlockCountCmd     = (1U << 6U), /* card support cmd23 flag*/
    SD_SupportSpeedClassControlCmd = (1U << 7U), /* card support speed class control flag */
};

/* SD card bus width */
typedef enum
{
    SDMMC_BusWdith1Bit = 0U, /* card bus 1 width */
    SDMMC_BusWdith4Bit = 1U, /* card bus 4 width */
    SDMMC_BusWdith8Bit = 2U, /* card bus 8 width */
} sd_buswidth_t;

/* SD card operation voltage */
typedef enum
{
    SD_OperationVoltageNone = 0U, /* indicate current voltage setting is not setting by suser*/
    SD_OperationVoltage330V = 1U, /* card operation voltage around 3.3v */
    SD_OperationVoltage300V = 2U, /* card operation voltage around 3.0v */
    SD_OperationVoltage180V = 3U, /* card operation voltage around 1.8v */
} sd_operation_voltage_t;

/* SD card communication method and speed */
typedef struct
{
    uint32_t busClock_Hz;   /* SD bus clock frequency united in Hz */
    sd_buswidth_t busWidth; /* set DATA bus width */
    SD_ModeConfig mode;         /* work mode */
    sd_dma_t dma;           /* dma mode */  
    sd_operation_voltage_t operationVoltageflag; /* card current operation voltage */
} sd_card_workmode;

/* SD card information structure */
typedef struct
{
    uint32_t flags;         /* sd card Flags */
    uint32_t busy;          /* busy status */
    uint32_t ocr;           /* Raw OCR content,, only 24bit avalible for SDIO card */
    sd_cid   cid;           /* CID */
    uint32_t rca;           /* Relative address of the card */
    sd_csd_t csd;           /* CSD */
    uint32_t blockCount;    /* Card total block number */
    uint32_t blockSize;     /* Card block size */
    sd_scr_t scr;           /* SCR */
    uint32_t version;       /* Card version */
} sd_card_info;

/* SD card state structure */
typedef struct
{
    SDHOST_Module* SDHOSTx; /* Host device */
    SDMMC_Module* SDMMCx;   /* SDMMC */
    SDMMC_CMD command;
    sd_card_info sd_card_information;
    sd_card_workmode card_workmode;
    SDHOST_TMODE_struct  TMODE_truct;
} sd_card_t;


/* Error bit in SPI mode R1 */
enum
{
    SDSPI_R1InIdleStateFlag        = (1U << 0U), /* In idle state */
    SDSPI_R1EraseResetFlag         = (1U << 1U), /* Erase reset */
    SDSPI_R1IllegalCommandFlag     = (1U << 2U), /* Illegal command */
    SDSPI_R1CommandCrcErrorFlag    = (1U << 3U), /* Com crc error */
    SDSPI_R1EraseSequenceErrorFlag = (1U << 4U), /* Erase sequence error */
    SDSPI_R1AddressErrorFlag       = (1U << 5U), /* Address error */
    SDSPI_R1ParameterErrorFlag     = (1U << 6U), /* Parameter error */
};

/* Error bit in SPI mode R2 */
enum
{
    SDSPI_R2CardLockedFlag            = (1U << 0U), /* Card is locked */
    SDSPI_R2WriteProtectEraseSkip     = (1U << 1U), /* Write protect erase skip */
    SDSPI_R2LockUnlockFailed          = (1U << 1U), /* Lock/unlock command failed */
    SDSPI_R2ErrorFlag                 = (1U << 2U), /* Unknown error */
    SDSPI_R2CardControllerErrorFlag   = (1U << 3U), /* Card controller error */
    SDSPI_R2CardEccFailedFlag         = (1U << 4U), /* Card ecc failed */
    SDSPI_R2WriteProtectViolationFlag = (1U << 5U), /* Write protect violation */
    SDSPI_R2EraseParameterErrorFlag   = (1U << 6U), /* Erase parameter error */
    SDSPI_R2OutOfRangeFlag            = (1U << 7U), /* Out of range */
    SDSPI_R2CsdOverwriteFlag          = (1U << 7U), /* CSD overwrite */
};

/* The bit mask for COMMAND VERSION field in R7 */
#define SDSPI_R7_VERSION_SHIFT (28U)
/* The bit mask for COMMAND VERSION field in R7 */
#define SDSPI_R7_VERSION_MASK (0xFU)
/* The bit shift for VOLTAGE ACCEPTED field in R7 */
#define SDSPI_R7_VOLTAGE_SHIFT (8U)
/* The bit mask for VOLTAGE ACCEPTED field in R7 */
#define SDSPI_R7_VOLTAGE_MASK (0xFU)
/* The bit mask for VOLTAGE 2.7V to 3.6V field in R7 */
#define SDSPI_R7_VOLTAGE_27_36_MASK (0x1U << SDSPI_R7_VOLTAGE_SHIFT)
/* The bit shift for ECHO field in R7 */
#define SDSPI_R7_ECHO_SHIFT (0U)
/* The bit mask for ECHO field in R7 */
#define SDSPI_R7_ECHO_MASK (0xFFU)

/* Data error token mask */
#define SDSPI_DATA_ERROR_TOKEN_MASK (0xFU)
/* Data Error Token mask bit */
enum
{
    SDSPI_DataErrorTokenError               = (1U << 0U), /* Data error */
    SDSPI_DataErrorTokenCardControllerError = (1U << 1U), /* Card controller error */
    SDSPI_DataErrorTokenCardEccFailed       = (1U << 2U), /* Card ecc error */
    SDSPI_DataErrorTokenOutOfRange          = (1U << 3U), /* Out of range */
};

/* Data Token */
typedef enum
{
    SDSPI_DataTokenBlockRead          = 0xFEU, /* Single block read, multiple block read */
    SDSPI_DataTokenSingleBlockWrite   = 0xFEU, /* Single block write */
    SDSPI_DataTokenMultipleBlockWrite = 0xFCU, /* Multiple block write */
    SDSPI_DataTokenStopTransfer       = 0xFDU, /* Stop transmission */
} sdspi_data_token_t;

/* Data Response Token mask */
#define SDSPI_DATA_RESPONSE_TOKEN_MASK (0x1FU) /* Mask for data response bits */
/* Data Response Token */
typedef enum
{
    SDSPI_DataResponseTokenAccepted   = 0x05U, /* Data accepted */
    SDSPI_DataResponseTokenCrcError   = 0x0BU, /* Data rejected due to CRC error */
    SDSPI_DataResponseTokenWriteError = 0x0DU, /* Data rejected due to write error */
} sdspi_data_response_token_t;

/* SDSPI individual commands */
typedef enum
{
    SDSPI_CommandCrc = 59U, /* Command crc protection on/off */
} sdspi_command_t;

/* ========================== MMC Card Specific Definitions ========================== */

/* MMC card individual commands */
typedef enum
{
    MMC_SendOperationCondition = 1U,  /* Send Operation Condition */
    MMC_SetRelativeAddress     = 3U,  /* Set Relative Address */
    MMC_SleepAwake             = 5U,  /* Sleep Awake */
    MMC_Switch                 = 6U,  /* Switch */
    MMC_SendExtendedCsd        = 8U,  /* Send EXT_CSD */
    MMC_ReadDataUntilStop      = 11U, /* Read Data Until Stop */
    MMC_BusTestRead            = 14U, /* Test Read */
    MMC_SendingBusTest         = 19U, /* test bus width cmd*/
    MMC_WriteDataUntilStop     = 20U, /* Write Data Until Stop */
    MMC_SendTuningBlock        = 21U, /* MMC sending tuning block */
    MMC_ProgramCid             = 26U, /* Program CID */
    MMC_EraseGroupStart        = 35U, /* Erase Group Start */
    MMC_EraseGroupEnd          = 36U, /* Erase Group End */
    MMC_FastInputOutput        = 39U, /* Fast IO */
    MMC_GoInterruptState       = 40U, /* Go interrupt State */
} mmc_command_t;

/* MMC card flags */
enum
{
    MMC_SupportHighSpeed26MHZFlag            = (1U << 0U),  /*mmc_card_information Support high speed 26MHZ */
    MMC_SupportHighSpeed52MHZFlag            = (1U << 1U),  /*mmc_card_information Support high speed 52MHZ */
    MMC_SupportHighSpeedDDR52MHZ180V300VFlag = (1 << 2U),   /*mmc_card_information ddr 52MHZ 1.8V or 3.0V */
    MMC_SupportHighSpeedDDR52MHZ120VFlag     = (1 << 3U),   /*mmc_card_information DDR 52MHZ 1.2V */
    MMC_SupportHS200200MHZ180VFlag           = (1 << 4U),   /*mmc_card_information HS200 ,200MHZ,1.8V */
    MMC_SupportHS200200MHZ120VFlag           = (1 << 5U),   /*mmc_card_information HS200, 200MHZ, 1.2V */
    MMC_SupportHS400DDR200MHZ180VFlag        = (1 << 6U),   /*mmc_card_information HS400, DDR, 200MHZ,1.8V */
    MMC_SupportHS400DDR200MHZ120VFlag        = (1 << 7U),   /*mmc_card_information HS400, DDR, 200MHZ,1.2V */
    MMC_SupportHighCapacityFlag              = (1U << 8U),  /*mmc_card_information Support high capacity */
    MMC_SupportAlternateBootFlag             = (1U << 9U),  /*mmc_card_information Support alternate boot */
    MMC_SupportDDRBootFlag                   = (1U << 10U), /*mmc_card_information support DDR boot flag*/
    MMC_SupportHighSpeedBootFlag             = (1U << 11U), /*mmc_card_information support high speed boot flag */
    MMC_SupportEnhanceHS400StrobeFlag        = (1U << 12U), /*mmc_card_information support enhance HS400 strobe */
};

/* MMC card access mode(Access mode in OCR) */
typedef enum
{
    MMC_AccessModeByte   = 0U, /* The card should be accessed as byte */
    MMC_AccessModeSector = 2U, /* The card should be accessed as sector */
} mmc_access_mode_t;

/* MMC card specification version(SPEC_VERS in CSD) */
typedef enum _mmc_specification_version
{
    MMC_SpecificationVersion0 = 0U, /* Allocated by MMCA */
    MMC_SpecificationVersion1 = 1U, /* Allocated by MMCA */
    MMC_SpecificationVersion2 = 2U, /* Allocated by MMCA */
    MMC_SpecificationVersion3 = 3U, /* Allocated by MMCA */
    MMC_SpecificationVersion4 = 4U, /* Version 4.1/4.2/4.3/4.41-4.5-4.51-5.0 */
} mmc_specification_version_t;

/* MMC card high-speed timing(HS_TIMING in Extended CSD) */
typedef enum
{
    MMC_HighSpeedTimingNone       = 0U, /* MMC card using none high-speed timing */
    MMC_HighSpeedTiming           = 1U, /* MMC card using high-speed timing */
    MMC_HighSpeed200Timing        = 2U, /* MMC card high speed 200 timing*/
	MMC_NormalSpeedTiming         = 3U, /* MMC card not using high-speed timing */
} mmc_high_speed_timing_t;

/* MMC card bus width */
typedef enum
{
    MMC_DataBusWidth1bit          = 0U,    /* MMC data bus width is 1 bit */
    MMC_DataBusWidth4bit          = 1U,    /* MMC data bus width is 4 bits */
    MMC_DataBusWidth8bit          = 2U,    /* MMC data bus width is 8 bits */
    MMC_DataBusWidth4bitDDR       = 5U,    /* MMC data bus width is 4 bits ddr */
    MMC_DataBusWidth8bitDDR       = 6U,    /* MMC data bus width is 8 bits ddr */
} mmc_buswidth_t;

/* MMC card operation voltage */
typedef enum
{
    MMC_OperationVoltageNone = 0U, /* indicate current voltage setting is not setting by suser*/
    MMC_OperationVoltage120V = 1U, /* mmccard operation voltage is 1.2v */
    MMC_OperationVoltage170to195V = 2U, /* mmccard operation voltage is 1.7v to 1.95v */
    MMC_OperationVoltage270to360V = 0x1FFU, /* mmccard operation voltage is 2.7v to 3.6v */
} mmc_operation_voltage_t;

/* OCR register bit definitions */
/* The bit mask for VOLTAGE WINDOW 1.70V to 1.95V field in OCR */
#define MMC_OCR_V170TO195_SHIFT (7U)
/* The bit mask for VOLTAGE WINDOW 1.70V to 1.95V field in OCR */
#define MMC_OCR_V170TO195_MASK (0x00000080U)
/* The bit shift for VOLTAGE WINDOW 2.00V to 2.60V field in OCR */
#define MMC_OCR_V200TO260_SHIFT (8U)
/* The bit mask for VOLTAGE WINDOW 2.00V to 2.60V field in OCR */
#define MMC_OCR_V200TO260_MASK (0x00007F00U)
/* The bit shift for VOLTAGE WINDOW 2.70V to 3.60V field in OCR */
#define MMC_OCR_V270TO360_SHIFT (15U)
/* The bit mask for VOLTAGE WINDOW 2.70V to 3.60V field in OCR */
#define MMC_OCR_V270TO360_MASK (0x00FF8000U)
/* The bit shift for ACCESS MODE field in OCR */
#define MMC_OCR_ACCESS_MODE_SHIFT (29U)
/* The bit mask for ACCESS MODE field in OCR */
#define MMC_OCR_ACCESS_MODE_MASK (0x60000000U)
/* The bit shift for BUSY field in OCR */
#define MMC_OCR_BUSY_SHIFT (31U)
/* The bit mask for BUSY field in OCR */
#define MMC_OCR_BUSY_MASK (1U << MMC_OCR_BUSY_SHIFT)

/* MMC card default relative address */
#define MMC_DEFAULT_RELATIVE_ADDRESS (2UL)

/* CSD structure version(CSD_STRUCTURE in CSD) */
typedef enum _mmc_csd_structure_version
{
    MMC_CsdStrucureVersion10       = 0U, /* CSD version No. 1.0 */
    MMC_CsdStrucureVersion11       = 1U, /* CSD version No. 1.1 */
    MMC_CsdStrucureVersion12       = 2U, /* CSD version No. 1.2 */
    MMC_CsdStrucureVersionInExtcsd = 3U, /* Version coded in Extended CSD */
} mmc_csd_structure_version_t;

/* MMC card information structure */
typedef struct
{
    uint32_t flags;         /* mmccard Flags */
    uint32_t busy;          /* busy status */
    uint32_t ocr;           /* Raw OCR content,, only 24bit avalible for SDIO mmccard */
    mmc_cid  cid;           /* CID */
    uint32_t rca;           /* Relative address of the mmccard */
    mmc_csd_t csd;           /* CSD */
    uint32_t userPartitionBlocks; /* Card total block number in user partition */
    uint32_t bootPartitionBlocks; /* Boot partition size united as block size */
    uint32_t eraseGroupBlocks;    /* Erase group size united as block size */
    uint32_t blockCount;     /* Card total block number */
    uint32_t blockSize;     /* mmccard block size */
    mmc_extended_csd_t extendedCsd;           /* Extended CSD */
    uint32_t version;       /* mmccard version */
} mmc_card_info;

/* MMC card communication method and speed */
typedef struct
{
    uint32_t busClock_Hz;   /* SD bus clock frequency united in Hz */
    mmc_buswidth_t busWidth; /* set DATA bus width */
    mmc_high_speed_timing_t busTiming;         /* work mode */
    sd_dma_t dma;           /* dma mode */  
    mmc_operation_voltage_t operationVoltageflag; /* mmccard current operation voltage */
} mmc_card_workmode;

/* MMC card state structure */
typedef struct
{
    SDHOST_Module* SDHOSTx; /* Host device */
    SDMMC_Module* SDMMCx;   /* SDMMC */
    SDMMC_CMD command;
    mmc_card_info mmc_card_information;
    mmc_card_workmode card_workmode;
    SDHOST_TMODE_struct  TMODE_truct;
} mmc_card_t;

/* Transfer speed calculation constants */
#define DIVIDER_IN_TRANSFER_SPEED (10U)
static const uint32_t g_transerSpeedFrequencyUnit[] = {100000U, 1000000U, 10000000U, 100000000U};
static const uint32_t g_transerSpeedMultiplierFactor[] = {0U,  10U, 12U, 13U, 15U, 20U, 26U, 30U,
                                                          35U, 40U, 45U, 52U, 55U, 60U, 70U, 80U};

/* ========================== Function Declarations ========================== */

/* SD Card Functions */
Status_card SD_NormalCMD_Send(sd_card_t* card, uint32_t index,uint32_t argument,SDMMC_CardRspType responseType);
Status_card SD_AutoCMD_Send(sd_card_t* card, uint32_t index,uint32_t argument,SDMMC_CardRspType responseType);
void SD_DecodeCid(sd_card_t* card);
void SD_DecodeCsd(sd_card_t* card);
Status_card SD_PollingCardStatusBusy(sd_card_t* card,uint32_t timeoutMs);
Status_card SD_SendSCR(sd_card_t* card);
Status_card SD_Erase_Block(sd_card_t* card, uint32_t startBlock, uint32_t blockCount);
Status_card SD_ReadBlocks(sd_card_t *card, uint32_t *buffer, uint32_t startBlock, uint32_t blockCount);
Status_card SD_WriteBlocks(sd_card_t *card, uint32_t *buffer, uint32_t startBlock, uint32_t blockCount);
Status_card SD_SwitchVoltage(sd_card_t *card);
Status_card SD_SelectBusTiming(sd_card_t *card);

/* MMC Card Functions */
Status_card MMC_NormalCMD_Send(mmc_card_t* mmccard, uint32_t index,uint32_t argument,SDMMC_CardRspType responseType);
Status_card MMC_AutoCMD_Send(mmc_card_t* mmccard, uint32_t index,uint32_t argument,SDMMC_CardRspType responseType);
void MMC_DecodeCid(mmc_card_t* mmccard);
void MMC_DecodeCsd(mmc_card_t* mmccard);
Status_card MMC_PollingCardStatusBusy(mmc_card_t* mmccard,uint32_t timeoutMs);
Status_card MMC_SendExtCsd(mmc_card_t* mmccard);
Status_card MMC_SetMaxEraseUnitSize(mmc_card_t* mmccard);
Status_card MMC_EraseGroups(mmc_card_t* mmccard, uint32_t startGroup, uint32_t endGroup);
Status_card MMC_ReadBlocks(mmc_card_t *mmccard, uint32_t *buffer, uint32_t startBlock, uint32_t blockCount);
Status_card MMC_WriteBlocks(mmc_card_t *mmccard, uint32_t *buffer, uint32_t startBlock, uint32_t blockCount);
Status_card MMC_SwitchVoltage(mmc_card_t *mmccard);
Status_card MMC_SelectBusTiming(mmc_card_t *mmccard);

/* Common utility functions */
void SDMMC_Delay(__IO uint32_t count);
uint32_t swap_uint32(uint32_t val);

/* SDSPI Functions */
Status_card SDSPI_PollingCardStatusBusy(sd_card_t* card,uint32_t timeoutMs);
Status_card SDSPI_WaitTransferDone(sd_card_t* card,uint32_t timeoutMs);
Status_card SDSPI_Erase_Block(sd_card_t* card, uint32_t startBlock, uint32_t blockCount);
Status_card SDSPI_ReadBlocks(sd_card_t *card, uint32_t *buffer, uint32_t startBlock, uint32_t blockCount);
Status_card SDSPI_WriteBlocks(sd_card_t *card, uint32_t *buffer, uint32_t startBlock, uint32_t blockCount);
Status_card SDSPI_SendSCD(sd_card_t* card);

#ifdef __cplusplus
}
#endif

#endif /* __SDMMC_HOST_H__ */
