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
*\*\file system_n32h7xx.h
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2025, Nations Technologies Inc. All rights reserved.
 */

#include "n32h7xx.h"

/* Register bit field definitions of the standard peripheral drivers.  They are
   used by PreInitialize() further down, which sets the clock tree, the GPIOs
   and the SDRAM controller by writing registers directly.  PreInitialize()
   runs before the C runtime is set up, so it must not call driver / BSP
   functions - only pure compile time macros are taken from these headers.   */
#include "n32h7xx_rcc.h"
#include "n32h7xx_gpio.h"

#if !defined  (HSE_VALUE)
#define HSE_VALUE             ((uint32_t)25000000UL) /* Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (CSI_VALUE)
  #define CSI_VALUE           ((uint32_t)4000000UL) /* Value of the Internal oscillator in Hz*/
#endif /* CSI_VALUE */

#if !defined  (HSI_VALUE)
  #define HSI_VALUE           ((uint32_t)64000000UL) /* Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

#define VECT_TAB_OFFSET       ((uint32_t)0x00000000UL) /* Vector Table base offset field.   */

#define FLASH_BANK1_BASE      (0x15000000U)
#define FLASH_BANK2_BASE      (0x15080000U)
#define AHBSRAM_ENDADDR       (0x30057FFFU)

uint32_t SystemCoreClock = 600000000;

#define TCM_SIZE_VALUE        (0x27) /*TCM_SIZE=0x20:256K ITCM;256K DTCM;512K AXI_SRAM2/3*/

/** Private_Functions  */

#define IS_RDPLEVEL_L1()      ((*(uint32_t (*)(void))0x1ff00011U)() == 1U)

/* defaule power supply is extern LDO. User can change the way of power supply*/
//#define PWR_SUPPLY_SELECTION       (PWR_LDO_SUPPLY)  /* External LDO Supply  */
#define PWR_SUPPLY_SELECTION       (PWR_DIRECT_SMPS_SUPPLY)  /* DCDC Supply  */
//#define PWR_SUPPLY_SELECTION       (PWR_EXTERNAL_SOURCE_SUPPLY)  /* VCAP Supply  */


#define READ_TCM_SIZE         (0x1ff00f01U)
#define OTP_TCM_SIZE_REG      (0x51105280U)

#define ITCM_SIZE(n) (  (n) <= 0x08U ? 8U - (n)          : \
                        (n) <= 0x10U ? 7U - ((n) - 9U)   : \
                        (n) <= 0x17U ? 6U - ((n) - 17U)  : \
                        (n) <= 0x1DU ? 5U - ((n) - 24U)  : \
                        (n) <= 0x22U ? 4U - ((n) - 30U)  : \
                        (n) <= 0x26U ? 3U - ((n) - 35U)  : \
                        (n) <= 0x29U ? 2U - ((n) - 39U)  : \
                        (n) <= 0x2BU ? 1U - ((n) - 42U)  : 0U \
)
/**
 *\*\name   ConfigTcmSize.
 *\*\fun    Config TCM_SIZE
 *\*\param  tcmSizeValue 
 *\*\    for N32H76x and N32H78x (The input parameters must be the following values):
 *\*\           0x00 :1024KB ITCM,  0   KB DTCM,  0   KB AXI_SRAM2/3,  128KB AXI SRAM1 
 *\*\           0x01 :896 KB ITCM,  128 KB DTCM,  0   KB AXI_SRAM2/3,  128KB AXI SRAM1 
 *\*\           0x02 :768 KB ITCM,  256 KB DTCM,  0   KB AXI_SRAM2/3,  128KB AXI SRAM1 
 *\*\           0x03 :640 KB ITCM,  384 KB DTCM,  0   KB AXI_SRAM2/3,  128KB AXI SRAM1 
 *\*\           0x04 :512 KB ITCM,  512 KB DTCM,  0   KB AXI_SRAM2/3,  128KB AXI SRAM1 
 *\*\           0x05 :384 KB ITCM,  640 KB DTCM,  0   KB AXI_SRAM2/3,  128KB AXI SRAM1 
 *\*\           0x06 :256 KB ITCM,  768 KB DTCM,  0   KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x07 :128 KB ITCM,  896 KB DTCM,  0   KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x08 :0   KB ITCM,  1024KB DTCM,  0   KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x09 :896 KB ITCM,  0   KB DTCM,  128 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x0A :768 KB ITCM,  128 KB DTCM,  128 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x0B :640 KB ITCM,  256 KB DTCM,  128 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x0C :512 KB ITCM,  384 KB DTCM,  128 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x0D :384 KB ITCM,  512 KB DTCM,  128 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x0E :256 KB ITCM,  640 KB DTCM,  128 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x0F :128 KB ITCM,  768 KB DTCM,  128 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x10 :0   KB ITCM,  896 KB DTCM,  128 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x11 :768 KB ITCM,  0   KB DTCM,  256 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x12 :640 KB ITCM,  128 KB DTCM,  256 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x13 :512 KB ITCM,  256 KB DTCM,  256 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x14 :384 KB ITCM,  384 KB DTCM,  256 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x15 :256 KB ITCM,  512 KB DTCM,  256 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x16 :128 KB ITCM,  640 KB DTCM,  256 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x17 :0   KB ITCM,  768 KB DTCM,  256 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x18 :640 KB ITCM,  0   KB DTCM,  384 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x19 :512 KB ITCM,  128 KB DTCM,  384 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x1A :384 KB ITCM,  256 KB DTCM,  384 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x1B :256 KB ITCM,  384 KB DTCM,  384 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x1C :128 KB ITCM,  512 KB DTCM,  384 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x1D :0   KB ITCM,  640 KB DTCM,  384 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x1E :512 KB ITCM,  0   KB DTCM,  512 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x1F :384 KB ITCM,  128 KB DTCM,  512 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x20 :256 KB ITCM,  256 KB DTCM,  512 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x21 :128 KB ITCM,  384 KB DTCM,  512 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x22 :0   KB ITCM,  512 KB DTCM,  512 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x23 :384 KB ITCM,  0   KB DTCM,  640 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x24 :256 KB ITCM,  128 KB DTCM,  640 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x25 :128 KB ITCM,  256 KB DTCM,  640 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x26 :0   KB ITCM,  384 KB DTCM,  640 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x27 :256 KB ITCM,  0   KB DTCM,  768 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x28 :128 KB ITCM,  128 KB DTCM,  768 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x29 :0   KB ITCM,  256 KB DTCM,  768 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x2A :128 KB ITCM,  0   KB DTCM,  896 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x2B :0   KB ITCM,  128 KB DTCM,  896 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x2C~2F :0KB ITCM,  0   KB DTCM,  1024KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\    for N32H73x (The input parameters must be the following values):
 *\*\           0x1E :512 KB ITCM,  0   KB DTCM,  128 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x1F :384 KB ITCM,  128 KB DTCM,  128 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x20 :256 KB ITCM,  256 KB DTCM,  128 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x21 :128 KB ITCM,  384 KB DTCM,  128 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x22 :0   KB ITCM,  512 KB DTCM,  128 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x23 :384 KB ITCM,  0   KB DTCM,  256 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x24 :256 KB ITCM,  128 KB DTCM,  256 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x25 :128 KB ITCM,  256 KB DTCM,  256 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x26 :0   KB ITCM,  384 KB DTCM,  256 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x27 :256 KB ITCM,  0   KB DTCM,  384 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x28 :128 KB ITCM,  128 KB DTCM,  384 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x29 :0   KB ITCM,  256 KB DTCM,  384 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x2A :128 KB ITCM,  0   KB DTCM,  512 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x2B :0   KB ITCM,  128 KB DTCM,  512 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\           0x2C~2F :0KB ITCM,  0   KB DTCM,  640 KB AXI_SRAM2/3,  128KB AXI SRAM1
 *\*\return none  
 */               
void ConfigTcmSize(uint32_t tcmSizeValue)
{
    uint32_t currValue = (*(uint32_t(*)(void))READ_TCM_SIZE)();
    if((currValue == 0x2c) && (currValue != tcmSizeValue))
    {
        *(__IO uint32_t*)OTP_TCM_SIZE_REG = tcmSizeValue;
        NVIC_SystemReset();
    }
    /*Configure ITCM as a protected area When RDP level is L1*/
    if(IS_RDPLEVEL_L1())
    {
        /* when it configs success for the first time, this operation will triggers a system reset. */
        (*(uint32_t (*)(uint32_t, uint32_t))(0x1ff01601U))( ITCM_BASE, 0x20000U* ITCM_SIZE(tcmSizeValue));
    }
}
#ifdef CORE_CM7
/**
*\*\name    PWR_ConfigSupply.
*\*\fun     Configure the PWR supply.
*\*\param   SupplySource (The input parameters must be the following values):
*\*\          - PWR_LDO_SUPPLY               :External LDO  SUPPLY 
*\*\          - PWR_DIRECT_SMPS_SUPPLY       :DCDC SUPPLY 
*\*\          - PWR_EXTERNAL_SOURCE_SUPPLY   :External VCAP SUPPLY 
*\*\return  none
**/
static void PWR_ConfigSupply(uint32_t SupplySource)
{
    __IO uint32_t tempreg;
    /* Get the old register value */
    tempreg = PWR->SYSCTRL4;
    /* Clear the old  value */
    tempreg &= (~PWR_SUPPLY_MODE_MASK);
    /* Set the new values */
    tempreg |= SupplySource;
    /* Set the power supply configuration */
    PWR->SYSCTRL4 = tempreg;
    
    /* Config NRST Filter Function */
    tempreg = PWR->SYSCTRL1;
    tempreg &= ~(PWR_RST_AGFBPEN_MAST | PWR_RST_DGF_CNT_MAST | PWR_RST_DGFBPEN_MAST);
    tempreg |= PWR_RST_DGF_CNT_DEFAULT;
    PWR->SYSCTRL1 = tempreg;
}
#endif

#if 0
#define DBG_CPU_DELAY_INTI() do{ DEM_CR |= DEM_CR_TRCENA; \
 DWT_CYCCNT = 0U; \
 DWT_CR |= DWT_CR_CYCCNTENA; \
 }while(0) 
#define DBG_CPU_DELAY_DISABLE() do{ \
 DEM_CR &= (uint32_t)(~(uint32_t)DEM_CR_TRCENA); \
 }while(0) 
#define __DBG_RCC_DELAY_US(usec) do{ uint32_t delay_end; \
 DBG_CPU_DELAY_INTI(); \
 delay_end=DWT_CYCCNT + (usec * (600000000/1000000)); \
 while(DWT_CYCCNT < delay_end){}; \
 DBG_CPU_DELAY_DISABLE(); \
 }while(0)

void RCC_ConfigLCDPixelClkTemp(uint32_t CLK_source, uint32_t CLK_divider)
{
		uint32_t reg_value;
		uint32_t reg_value1;
		reg_value = RCC->AXISEL1;
		reg_value &= RCC_LCDPIXELCLK_SRC_MASK;
		reg_value |= CLK_source;
		RCC->AXISEL1 = reg_value;
		if(CLK_source == RCC_LCDPIXELCLK_SRC_AXIDIV)
		{ 
		reg_value1 = RCC->AXIDIV1;
		reg_value1 &= RCC_LCDPIXELCLK_AXIDIV_MASK;
		reg_value1 |= CLK_divider;
		RCC->AXIDIV1 = reg_value1;
		}
}
//
void RCC_ConfigPLL2BDividerTemp(uint32_t CLK_divider)
{
		uint32_t reg_value = 0;
		reg_value = RCC->PLL2DIV;
		reg_value &= RCC_PLLB_DIV_MASK;
		reg_value |= CLK_divider;
		RCC->PLL2DIV = reg_value;
}
#endif
//
/* ==========================================================================
 * Register level helpers used by PreInitialize()
 *
 * PreInitialize() is called from SystemInit(), i.e. BEFORE the C runtime has
 * copied .data and cleared .bss.  Therefore it must NOT call any driver/BSP
 * function, because those may read initialised global variables.  Everything
 * below is done by writing the peripheral registers directly.  The GPIO_*,
 * AFIO_*, RCC_* and SDRAM_* symbols are pure compile time constants taken
 * from the standard peripheral driver headers.
 * ========================================================================== */

/* Micro second delay built on the DWT cycle counter (core clock = 600MHz).
   Same idea as the macro __RCC_DELAY_US() used inside n32h7xx_rcc.c.        */
#define PREINIT_DELAY_US(usec)                                                 \
    do {                                                                       \
        uint32_t _preinit_delay_end;                                           \
        CPU_DELAY_INTI();                                                      \
        _preinit_delay_end = DWT_CYCCNT + ((usec) * (600000000U / 1000000U));   \
        while (DWT_CYCCNT < _preinit_delay_end) { }                            \
        CPU_DELAY_DISABLE();                                                   \
    } while (0U)

/* Configure one FEMC (SDRAM) pin:
     alternate function push-pull output, no pull resistor, fast slew rate and
     2mA drive strength, plus the matching AFIO high speed mode / VREF /
     drive strength (N and P) remap bits.
       _port : GPIOA ... GPIOI        _pidx : GPIOA = 0 ... GPIOI = 8
       _pin  : pin number 0 ... 15    _af   : GPIO_AF0 / GPIO_AF1 ...
       _idx  : AFIO_HIGH_SPEED_IO_Pxx index of that pin                    */
/* GPIO_InitPeripheral() uses these two helper masks internally, but they are
   private to n32h7xx_gpio.c and are not exported by n32h7xx_gpio.h, so they
   are restated here with exactly the same values.
     GPIO_MODE        : masks the direction field of the GPIO mode value
     GPIO_OUTPUT_TYPE : masks the push-pull / open-drain flag                   */
#define PREINIT_GPIO_MODE_MASK          ((uint32_t)0x00000003U)
#define PREINIT_GPIO_OUTPUT_TYPE        ((uint32_t)0x00000010U)

#define PREINIT_SDRAM_PIN(_port, _pidx, _pin, _af, _idx)                       \
    do {                                                                       \
        uint32_t _preinit_rp;                                                  \
        if ((_pin) < 8U)                                                       \
        {                                                                      \
            _preinit_rp  = (_port)->AFL & ~(0xFU << ((_pin) * 4U));             \
            _preinit_rp |= ((uint32_t)(_af) << ((_pin) * 4U));                  \
            (_port)->AFL = _preinit_rp;                                        \
        }                                                                      \
        else                                                                   \
        {                                                                      \
            _preinit_rp  = (_port)->AFH & ~(0xFU << (((_pin) & 0x07U) * 4U));   \
            _preinit_rp |= ((uint32_t)(_af) << (((_pin) & 0x07U) * 4U));        \
            (_port)->AFH = _preinit_rp;                                        \
        }                                                                      \
        /* output type : push-pull (GPIO_MODE_AF_PP has no open drain flag)  */ \
        _preinit_rp  = (_port)->POTYPE & ~(GPIO_POTYPE_POT0 << (_pin));         \
        _preinit_rp |= (((GPIO_MODE_AF_PP & PREINIT_GPIO_OUTPUT_TYPE) >> 4U) << (_pin));\
        (_port)->POTYPE = _preinit_rp;                                         \
        /* direction : alternate function                                   */ \
        _preinit_rp  = (_port)->PMODE & ~(GPIO_PMODE_PMODE0 << ((_pin) * 2U));  \
        _preinit_rp |= ((GPIO_MODE_AF_PP & PREINIT_GPIO_MODE_MASK) << ((_pin) * 2U));        \
        (_port)->PMODE = _preinit_rp;                                          \
        /* no pull-up / pull-down                                           */ \
        _preinit_rp  = (_port)->PUPD & ~(GPIO_PUPD_PUPD0 << ((_pin) * 2U));     \
        _preinit_rp |= ((uint32_t)GPIO_NO_PULL << ((_pin) * 2U));               \
        (_port)->PUPD = _preinit_rp;                                           \
        /* slew rate : fast                                                 */ \
        _preinit_rp  = (_port)->SR & ~(GPIO_SR_BIT_MASK << (_pin));             \
        _preinit_rp |= ((uint32_t)GPIO_SLEW_RATE_FAST << (_pin));               \
        (_port)->SR = _preinit_rp;                                             \
        /* drive strength : 2mA                                             */ \
        _preinit_rp  = (_port)->DS & ~(GPIO_DS_BIT_MASK << ((_pin) * 2U));      \
        _preinit_rp |= ((uint32_t)GPIO_DC_2mA << ((_pin) * 2U));                \
        (_port)->DS = _preinit_rp;                                             \
        /* AFIO : high speed mode, 16 bits per port, 32 bits per register   */ \
        AFIO->HSMODE_CFG[(_pidx) / 2U] |=                                      \
            ((1U << (_pin)) << (16U * (((_pidx) + 1U) & 1U)));                  \
        /* AFIO : VREF / drive strength N / drive strength P remap          */ \
        AFIO->HSMODE_VREF_EN[(_idx) / 32U] |= (1U << ((_idx) % 32U));           \
        AFIO->HS_DSN_CFG[(_idx) / 32U]     |= (1U << ((_idx) % 32U));           \
        AFIO->HS_DSP_CFG[(_idx) / 32U]     |= (1U << ((_idx) % 32U));           \
    } while (0U)

/**
*\*\name    PreInitialize.
*\*\fun     All initialisation that has to be done before the C runtime is set
*\*\        up (before .data copy / .bss clear in __main).
*\*\        Everything is written straight into the registers, no driver or BSP
*\*\        function is called.
*\*\param   none
*\*\return  none
**/
void PreInitialize(void)
{
	uint32_t tmp;
	uint32_t wait_cnt;

	/* ==================================================================
	 * 1. System clock
	 *      PLL1 : HSI 64MHz -> 600MHz
	 *      sys_clk = PLL1A = 600MHz (M7), sys_bus/AXI = 300MHz,
	 *      APB1/2/5/6 = 150MHz
	 *    (equivalent of RCC_SetSysClkToMode0())
	 * ================================================================== */
	tmp  = RCC->SYSBUSDIV1;
	tmp &= RCC_SYSCLK_DIV_MASK;
	tmp |= RCC_SYSCLK_DIV1;
	RCC->SYSBUSDIV1 = tmp;                       /* sys_clk = PLL1A     */

	tmp  = RCC->SYSBUSDIV1;
	tmp &= RCC_BUSCLK_DIV_MASK;
	tmp |= RCC_BUSCLK_DIV2;
	RCC->SYSBUSDIV1 = tmp;                       /* sys_bus = 300MHz    */

	tmp  = RCC->SYSBUSDIV1;
	tmp &= RCC_AXICLK_DIV_MASK;
	tmp |= RCC_AXICLK_DIV2;
	RCC->SYSBUSDIV1 = tmp;                       /* AXI clk = 300MHz    */

	tmp  = RCC->SYSBUSDIV1;
	tmp &= RCC_AXICLK_HYP_DIV_MASK;
	tmp |= RCC_AXICLK_HYP_DIV2;
	RCC->SYSBUSDIV1 = tmp;                       /* AXI hyp = 300MHz    */

	PREINIT_DELAY_US(1U);

	tmp  = RCC->SYSBUSDIV2;
	tmp &= RCC_APBCLK_DIV_MASK;
	tmp |= (RCC_APB1CLK_DIV2 | RCC_APB2CLK_DIV2 |
	        RCC_APB5CLK_DIV2 | RCC_APB6CLK_DIV2);
	RCC->SYSBUSDIV2 = tmp;                       /* APB = 150MHz        */

	/* PLL1 : CLKF = 153600, CLKR = 0
	   64MHz * 153600 / 16384 / (0 + 1) = 600MHz                          */
	tmp  = RCC->PLL1CTRL1;
	tmp &= RCC_PLL_BWAJ_MASK;
	tmp |= 0x3U;                                 /* BWAJ               */
	RCC->PLL1CTRL1 = tmp;

	tmp  = RCC->PLL1CTRL2;
	tmp &= RCC_PLL_CLKR_CLKF_MASK;
	tmp |= 0x25800U;                             /* CLKR = 0, CLKF      */
	RCC->PLL1CTRL2 = tmp;

	RCC->PLL1CTRL1 |= RCC_PLL_LDO_ENABLE;        /* PLL module power on */
	PREINIT_DELAY_US(10U);

	RCC->PLL1CTRL1 &= (~RCC_PLL_POWER_DOWN);     /* analog part power on*/

	tmp  = RCC->PLL1CTRL1;
	tmp &= RCC_PLL_SRC_MASK;
	tmp |= RCC_PLL_SRC_HSI;
	RCC->PLL1CTRL1 = tmp;
	PREINIT_DELAY_US(10U);

	RCC->PLL1CTRL1 &= (~RCC_PLL_RESET_ENABLE);

	while ((RCC->PLL1CTRL1 & RCC_PLL_LOCK_FLAG) != RCC_PLL_LOCK_FLAG)
	{
	}

	RCC->PLL1CTRL1 |= RCC_PLL_ENABLE;

	PREINIT_DELAY_US(1U);

	tmp  = RCC->PLL1DIV;
	tmp &= RCC_PLLA_DIV_MASK;
	tmp |= RCC_PLLA_DIV1;
	RCC->PLL1DIV = tmp;                          /* PLL1A = 600MHz      */

	tmp  = RCC->SRCCTRL1;
	tmp &= RCC_SYSCLK_SRC_MASK;
	tmp |= RCC_SYSCLK_SRC_PLL1A;
	RCC->SRCCTRL1 = tmp;                         /* sys_clk src = PLL1A */

	while ((RCC->SRCCTRL1 & RCC_SYSCLK_STS_MASK) != RCC_SYSCLK_STS_PLL1A)
	{
	}

	/* ==================================================================
	 * 2. Debug UART (USART1)
	 *      GPIOA / AFIO / USART1 clocks, PA9 = USART1_TX (AF7)
	 *    (equivalent of RCC_Configuration() + GPIO_Configuration())
	 * ================================================================== */
	RCC->AHB5EN1 |= RCC_AHB5_PERIPHEN_M7_GPIOA;
	RCC->AHB5EN2 |= RCC_AHB5_PERIPHEN_M7_AFIO;
	RCC->APB1EN3 |= RCC_APB1_PERIPHEN_M7_USART1;

	tmp  = GPIOA->AFH;                           /* PA9 >= 8 -> AFH     */
	tmp &= ~(0xFU << (((9U) & 0x07U) * 4U));
	tmp |= ((uint32_t)GPIO_AF7 << (((9U) & 0x07U) * 4U));
	GPIOA->AFH = tmp;                            /* PA9 = AF7           */

	tmp  = GPIOA->POTYPE & ~(GPIO_POTYPE_POT0 << 9U);
	tmp |= (((GPIO_MODE_AF_PP & PREINIT_GPIO_OUTPUT_TYPE) >> 4U) << 9U);
	GPIOA->POTYPE = tmp;                         /* push-pull           */

	tmp  = GPIOA->PMODE & ~(GPIO_PMODE_PMODE0 << (9U * 2U));
	tmp |= ((GPIO_MODE_AF_PP & PREINIT_GPIO_MODE_MASK) << (9U * 2U));
	GPIOA->PMODE = tmp;                          /* alternate function  */

	tmp  = GPIOA->PUPD & ~(GPIO_PUPD_PUPD0 << (9U * 2U));
	tmp |= ((uint32_t)GPIO_PULL_UP << (9U * 2U));
	GPIOA->PUPD = tmp;                           /* pull-up             */

	tmp  = GPIOA->SR & ~(GPIO_SR_BIT_MASK << 9U);
	tmp |= ((uint32_t)GPIO_SLEW_RATE_SLOW << 9U);
	GPIOA->SR = tmp;                             /* slow slew rate      */

	tmp  = GPIOA->DS & ~(GPIO_DS_BIT_MASK << (9U * 2U));
	tmp |= ((uint32_t)GPIO_DC_2mA << (9U * 2U));
	GPIOA->DS = tmp;                             /* 2mA                 */

	/* ==================================================================
	 * 3. SDRAM clocks
	 *      GPIOA..GPIOI + AFIO clocks, HSE 25MHz,
	 *      PLL3 = 665MHz, PLL3A = PLL3/5 = 133MHz = SDRAM memory clock
	 *    (equivalent of SDRAM_RCC_Configuration())
	 * ================================================================== */
	RCC->AHB5EN1 |= (RCC_AHB5_PERIPHEN_M7_GPIOA | RCC_AHB5_PERIPHEN_M7_GPIOB |
	                 RCC_AHB5_PERIPHEN_M7_GPIOC | RCC_AHB5_PERIPHEN_M7_GPIOD |
	                 RCC_AHB5_PERIPHEN_M7_GPIOE | RCC_AHB5_PERIPHEN_M7_GPIOF |
	                 RCC_AHB5_PERIPHEN_M7_GPIOG | RCC_AHB5_PERIPHEN_M7_GPIOH);
	RCC->AHB5EN2 |= (RCC_AHB5_PERIPHEN_M7_GPIOI | RCC_AHB5_PERIPHEN_M7_AFIO);

	/* HSE on (25MHz crystal) + wait until stable
	   (equivalent of RCC_ConfigHse() + RCC_WaitHseStable())              */
	RCC->SRCCTRL1 &= (~RCC_HSE_RDCNTEN);
	RCC->SRCCTRL1 |= RCC_HSE_ENABLE;
	PREINIT_DELAY_US(50U);
	RCC->SRCCTRL1 |= RCC_HSE_RDCNTEN;

	wait_cnt = 0U;
	while (((RCC->SRCCTRL1 & RCC_HSE_STABLE_FLAG) != RCC_HSE_STABLE_FLAG) &&
	       (wait_cnt < 0x1000000U))
	{
		wait_cnt++;
	}

	/* PLL3 : fin = 25MHz, fout = 665MHz.
	   The SDK solves the dividers with RCC_CalculatePLLParam(25MHz,665MHz)
	   which returns NR = 5, NF = 34048, WB = 65, because that gives a zero
	   frequency error : 25MHz * 34048 / 5 / 256 = 665MHz.
	   Register encoding : CLKR = NR - 1 = 4, CLKF = NF * 64 = 2179072, so
	   25MHz * 2179072 / 16384 / (4 + 1) = 665MHz as well.                 */
	tmp  = RCC->PLL3CTRL1;
	tmp &= RCC_PLL_BWAJ_MASK;
	tmp |= 65U;                                  /* BWAJ (WB)           */
	RCC->PLL3CTRL1 = tmp;

	tmp  = RCC->PLL3CTRL2;
	tmp &= RCC_PLL_CLKR_CLKF_MASK;
	tmp |= ((4U << 26) | 2179072U);              /* CLKR = 4, CLKF      */
	RCC->PLL3CTRL2 = tmp;

	RCC->PLL2CTRL1 |= RCC_PLL_LDO_ENABLE;        /* as in RCC_ConfigPll3() */
	PREINIT_DELAY_US(10U);

	RCC->PLL3CTRL1 &= (~RCC_PLL_POWER_DOWN);

	tmp  = RCC->PLL3CTRL1;
	tmp &= RCC_PLL_SRC_MASK;
	tmp |= RCC_PLL_SRC_HSE;
	RCC->PLL3CTRL1 = tmp;
	PREINIT_DELAY_US(10U);

	RCC->PLL3CTRL1 &= (~RCC_PLL_RESET_ENABLE);

	while ((RCC->PLL3CTRL1 & RCC_PLL_LOCK_FLAG) != RCC_PLL_LOCK_FLAG)
	{
	}

	RCC->PLL3CTRL1 |= RCC_PLL_ENABLE;

	tmp  = RCC->PLL3DIV;
	tmp &= RCC_PLLA_DIV_MASK;
	tmp |= RCC_PLLA_DIV5;
	RCC->PLL3DIV = tmp;                          /* PLL3A = 133MHz      */

	tmp  = RCC->AXISEL2;
	tmp &= RCC_SDRAMMEMCLK_SRC_MASK;
	tmp |= RCC_SDRAMMEMCLK_SRC_PLL3A;
	RCC->AXISEL2 = tmp;                          /* SDRAM clk = PLL3A   */

	RCC->AXIEN4 |= (RCC_AXI_PERIPHEN_M7_SDRAM | RCC_AXI_PERIPHEN_M7_SDRAMLP);

	tmp  = RCC->PLLSFTLK;
	tmp &= RCC_SDRAM_DELAY_MASK;
	tmp |= RCC_SDRAM_DELAY_0_2NS;
	RCC->PLLSFTLK = tmp;
	RCC->PLLSFTLK |= RCC_SDRAM_DELAY_ENABLE;     /* SDRAM delay chain   */

	/* ==================================================================
	 * 4. SDRAM pin configuration  (SDRAM_IS42S32800J_HMI, 32bit bus)
	 *    FEMC alternate function + AFIO high speed mode
	 *    (equivalent of SDRAM_GPIO_Init())
	 *
	 *    The board is an N32H787_HMI, not the N32H787_EVB, and the two route
	 *    the SDRAM to different FEMC pins.  bsp_sdram.h selects the pin set
	 *    with SDRAM_DEVICE; the HMI set differs from the EVB set in exactly
	 *    four signals:
	 *        NCE0 (chip select) PC2   (EVB: PH3)
	 *        NWE  (write enable) PC0   (EVB: PH5)
	 *        D9                  PE12  (EVB: PA5)
	 *        D12                 PE15  (EVB: PC0, alternate function 1)
	 *    Everything else - A0..A11, BA0/BA1, RAS, CAS, CKE, CLK, DQM0..3 and
	 *    the remaining data lines - is identical on both boards.
	 *
	 *    With the EVB map on HMI hardware the SDRAM chip select and write
	 *    enable are never driven, so the device is not selected and the whole
	 *    memory appears as aliased garbage (the tensor arena probes returned
	 *    the same cell for every address below 8 MB).
	 * ================================================================== */
	/* ADD */
	PREINIT_SDRAM_PIN(GPIOF, 5U,  0U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PF0);
	PREINIT_SDRAM_PIN(GPIOF, 5U,  1U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PF1);
	PREINIT_SDRAM_PIN(GPIOF, 5U,  2U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PF2);
	PREINIT_SDRAM_PIN(GPIOF, 5U,  3U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PF3);
	PREINIT_SDRAM_PIN(GPIOF, 5U,  4U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PF4);
	PREINIT_SDRAM_PIN(GPIOF, 5U,  5U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PF5);
	PREINIT_SDRAM_PIN(GPIOF, 5U, 12U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PF12);
	PREINIT_SDRAM_PIN(GPIOF, 5U, 13U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PF13);
	PREINIT_SDRAM_PIN(GPIOF, 5U, 14U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PF14);
	PREINIT_SDRAM_PIN(GPIOF, 5U, 15U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PF15);
	PREINIT_SDRAM_PIN(GPIOG, 6U,  0U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PG0);
	PREINIT_SDRAM_PIN(GPIOG, 6U,  1U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PG1);
	/* DATA */
	PREINIT_SDRAM_PIN(GPIOD, 3U, 14U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PD14);
	PREINIT_SDRAM_PIN(GPIOD, 3U, 15U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PD15);
	PREINIT_SDRAM_PIN(GPIOD, 3U,  0U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PD0);
	PREINIT_SDRAM_PIN(GPIOD, 3U,  1U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PD1);
	PREINIT_SDRAM_PIN(GPIOE, 4U,  7U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PE7);
	PREINIT_SDRAM_PIN(GPIOE, 4U,  8U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PE8);
	PREINIT_SDRAM_PIN(GPIOE, 4U,  9U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PE9);
	PREINIT_SDRAM_PIN(GPIOE, 4U, 10U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PE10);
	PREINIT_SDRAM_PIN(GPIOE, 4U, 11U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PE11);
	PREINIT_SDRAM_PIN(GPIOE, 4U, 12U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PE12);   /* D9  : HMI */
	PREINIT_SDRAM_PIN(GPIOE, 4U, 13U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PE13);
	PREINIT_SDRAM_PIN(GPIOE, 4U, 14U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PE14);
	PREINIT_SDRAM_PIN(GPIOE, 4U, 15U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PE15);   /* D12 : HMI */
	PREINIT_SDRAM_PIN(GPIOD, 3U,  8U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PD8);
	PREINIT_SDRAM_PIN(GPIOD, 3U,  9U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PD9);
	PREINIT_SDRAM_PIN(GPIOD, 3U, 10U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PD10);
	PREINIT_SDRAM_PIN(GPIOH, 7U,  8U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PH8);
	PREINIT_SDRAM_PIN(GPIOH, 7U,  9U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PH9);
	PREINIT_SDRAM_PIN(GPIOH, 7U, 10U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PH10);
	PREINIT_SDRAM_PIN(GPIOH, 7U, 11U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PH11);
	PREINIT_SDRAM_PIN(GPIOH, 7U, 12U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PH12);
	PREINIT_SDRAM_PIN(GPIOH, 7U, 13U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PH13);
	PREINIT_SDRAM_PIN(GPIOH, 7U, 14U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PH14);
	PREINIT_SDRAM_PIN(GPIOH, 7U, 15U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PH15);
	PREINIT_SDRAM_PIN(GPIOI, 8U,  0U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PI0);
	PREINIT_SDRAM_PIN(GPIOI, 8U,  1U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PI1);
	PREINIT_SDRAM_PIN(GPIOI, 8U,  2U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PI2);
	PREINIT_SDRAM_PIN(GPIOI, 8U,  3U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PI3);
	PREINIT_SDRAM_PIN(GPIOI, 8U,  6U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PI6);
	PREINIT_SDRAM_PIN(GPIOI, 8U,  7U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PI7);
	PREINIT_SDRAM_PIN(GPIOI, 8U,  9U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PI9);
	PREINIT_SDRAM_PIN(GPIOI, 8U, 10U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PI10);
	/* BA */
	PREINIT_SDRAM_PIN(GPIOG, 6U,  4U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PG4);
	PREINIT_SDRAM_PIN(GPIOG, 6U,  5U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PG5);
	/* command pins : CS / WE / RAS / CAS */
	PREINIT_SDRAM_PIN(GPIOC, 2U,  2U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PC2);   /* CS  : HMI */
	PREINIT_SDRAM_PIN(GPIOC, 2U,  0U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PC0);   /* WE  : HMI */
	PREINIT_SDRAM_PIN(GPIOF, 5U, 11U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PF11);
	PREINIT_SDRAM_PIN(GPIOG, 6U, 15U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PG15);
	/* DQM */
	PREINIT_SDRAM_PIN(GPIOE, 4U,  0U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PE0);
	PREINIT_SDRAM_PIN(GPIOE, 4U,  1U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PE1);
	PREINIT_SDRAM_PIN(GPIOI, 8U,  4U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PI4);
	PREINIT_SDRAM_PIN(GPIOI, 8U,  5U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PI5);
	/* CKE / CLK */
	PREINIT_SDRAM_PIN(GPIOH, 7U,  2U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PH2);
	PREINIT_SDRAM_PIN(GPIOG, 6U,  8U, GPIO_AF0, AFIO_HIGH_SPEED_IO_PG8);

	/* ==================================================================
	 * 5. SDRAM device initialisation
	 *      IS42S32800J : 4 banks x 4096 rows x 512 columns x 32bit = 32MByte
	 *    (equivalent of SDRAM_DeviceInit())
	 * ================================================================== */
	SDRAM->BADD1    = 0xC0000000U;               /* SDRAM1 base address */
	SDRAM->ADDMASK1 = 0xFE000000U;               /* 0xFFFFFFFF-(32M-1)  */

	SDRAM->RI = 0x750U;                          /* 64ms/4096rows/7.5ns
	                                                = 0x7A1, round down  */

	SDRAM->RAT   = 0x7U;                         /* tRAS  RowActiveTime */
	SDRAM->RCT   = 0x9U;                         /* tRC   RowCycleTime  */
	SDRAM->RRDLY = 0x3U;                         /* tRRD  RowActToRowAct*/
	SDRAM->PT    = 0x4U;                         /* tRP   PrechargeTime */
	SDRAM->WRT   = 0x3U;                         /* tWR   WriteRecovery */
	SDRAM->RFCT  = 0x9U;                         /* tRFC  RefreshCycle  */
	SDRAM->RCDLY = 0x4U;                         /* tRCD  RAStoCASDelay */

	/* CFG : enable device + refresh, sample on rising edge, no auto
	   precharge, no prefetch, no bank interleaving, 32bit bus, burst
	   length 2, CAS latency 3, 4 banks / 4096 rows / 512 columns        */
	tmp  = SDRAM->CFG1;
	tmp |= (SDRAM_CFG_SDRAMEN | SDRAM_CFG_REFRESHEN | SDRAM_CFG_SOMEN);
	tmp &= (~(SDRAM_CFG_AUTOPCHEN | SDRAM_CFG_PREFCHRDEN |
	          SDRAM_CFG_BANKINTRLEN));
	tmp &= (~SDRAM_CFG_BUSWID);
	tmp |= SDRAM_CFG_BUSWID_1;                   /* 32bit data bus      */
	tmp &= (~SDRAM_CFG_BURSTLEN);
	tmp |= SDRAM_CFG_BURSTLEN_0;                 /* burst length 2      */
	tmp &= (~SDRAM_CFG_CASLATENCY);
	tmp |= (SDRAM_CFG_CASLATENCY_0 | SDRAM_CFG_CASLATENCY_1);   /* CL = 3 */
	tmp &= (~SDRAM_CFG_ADDCFG);
	tmp |= SDRAM_CFG_ADDCFG_0;                   /* 4096 rows x 512 col */
	SDRAM->CFG1 = tmp;

	/* Precharge all banks, 2 x auto refresh, then load mode register.
	   The operation register fields are : clock enable, opcode, chip
	   select, bank address and the mode-register address.               */
	tmp  = SDRAM->OS;
	tmp &= (~(SDRAM_OS_OPCODE | SDRAM_OS_CS | SDRAM_OS_BANKADD |
	          SDRAM_OS_ADD));                    /* clear all fields    */
	tmp |= (SDRAM_OS_CKEN | SDRAM_OS_CS_1);      /* CKE on, CS - SDRAM1 */
	/* bank address = SDRAM_BANKADD_1 (= 0), address = 0 for this command */

	SDRAM->OS = (tmp | SDRAM_OS_OPCODE_0);       /* precharge all banks */
	(void)(SDRAM->OR);                           /* dummy access        */

	SDRAM->OS = (tmp | SDRAM_OS_OPCODE_1);       /* auto refresh #1     */
	(void)(SDRAM->OR);

	SDRAM->OS = (tmp | SDRAM_OS_OPCODE_1);       /* auto refresh #2     */
	(void)(SDRAM->OR);

	/* mode register : burst length 2 (M2:0=001), sequential (M3=0),
	   CAS latency 3 (M6:4=011), standard op (M8:7=00), programmed burst
	   length (M9=0)  ->  0x00000031                                      */
	SDRAM->OS = (tmp | SDRAM_OS_OPCODE_0 | SDRAM_OS_OPCODE_1 | 0x31U);
	(void)(SDRAM->OR);                           /* load mode register  */

	PREINIT_DELAY_US(500U);                      /* tMRD >= 300us       */
}
/**
 *\*\name   SystemInit.
 *\*\fun    Setup the microcontroller system.Initialize the FPU setting, vector table location and External memory configuration.
 *\*\param  none
 *\*\return none
 */
void SystemInit (void)
{    
  /* FPU settings ------------------------------------------------------------*/
  #if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << (10*2))|(3UL << (11*2)));  /* set CP10 and CP11 Full Access */
  #endif

    /*SEVONPEND enabled so that an interrupt coming from the CPU(n) interrupt signal is
     detectable by the CPU after a WFI/WFE instruction.*/ 
 SCB->SCR |= SCB_SCR_SEVONPEND_Msk;

#ifdef CORE_CM7 
  
#endif /* CORE_CM7*/

#ifdef CORE_CM4

  /* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM
  SCB->VTOR = AHBSRAM_BASE; /* Vector Table Relocation in Internal SRAM */
#else
  SCB->VTOR = FLASH_BANK2_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif  
  /*Configure AHB-SRAM as a protected area When RDP level is L1*/
  if(IS_RDPLEVEL_L1())
  {
      /* when it configs success for the first time, this operation will triggers a system reset. */
      (*(uint32_t (*)(uint32_t, uint32_t))(0x1ff01701))( AHBSRAM_BASE, AHBSRAM_ENDADDR);
  }
#else
#ifdef CORE_CM7

  /* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM
  SCB->VTOR = SRAM_BASE;        /* Vector Table Relocation in Internal SRAM */
#else
  SCB->VTOR = FLASH_BANK1_BASE | VECT_TAB_OFFSET;       /* Vector Table Relocation in Internal FLASH */
#endif

   /*defaule TCM_SIZE=0x2F,All TCMSRAM are AXI_SRAM,if you want to use ITCM/DTCM, define TCM_SIZE_VALUE*/
#ifdef USING_TCM
    ConfigTcmSize(TCM_SIZE_VALUE);
#endif
    /*User can change the way of power supply */
    PWR_ConfigSupply(PWR_SUPPLY_SELECTION);

		PreInitialize();
#else
#error Please #define CORE_CM4 or CORE_CM7
#endif                       
#endif

}


/**
 *\*\name   N32SysTick_Handler.
 *\*\fun    This function handles N32SysTick Handler.
 *\*\param  none
 *\*\return none
 */
void N32SysTick_Handler(void)
{

}

