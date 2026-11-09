/* ptpd_dep.c */

#include "ptpd_dep.h"
#include "n32h7xx_eth.h"

/* Examples of subsecond increment and addend values using:

    HCLK = 300MHz, PTP_CLK = 150MHz,

    Fine Update Mode and ETH_MACTSCTRL.TSCTRLSSR = 0:

        Addend = (2^32 * PTP_CLK) / HCLK

        Increment = (2 * 10^9 / PTP_CLK) / 0.465ns
*/
#define ADJ_FREQ_BASE_ADDEND      0x80000000
#define ADJ_FREQ_BASE_INCREMENT   28

/**
*\*\name    ETH_PTPNanoSecond2SubSecond.
*\*\fun     Converting nanosecond values to sub-second values.
*\*\param   SubSecondValue :
*\*\          - 32-bit nanosecond value
*\*\return  32-bit sub-second value
**/
static u32_t ETH_PTPNanoSecond2SubSecond(u32_t NanoSecondValue)
{
    /* sub = (ns * 2^32) / 10^9 */
    uint64_t val = NanoSecondValue * 0x100000000ull;
    val /= 1000000000ull;
    return (u32_t)val;
}

/**
*\*\name    ETH_PTPSubSecond2NanoSecond.
*\*\fun     Converting sub-second values to nanosecond values.
*\*\param   SubSecondValue :
*\*\          - 32-bit sub-second value
*\*\return  32-bit nanosecond value
**/
static u32_t ETH_PTPSubSecond2NanoSecond(u32_t SubSecondValue)
{
    /* ns = (sub * 10^9) / 2^32 */
    uint64_t val = ((uint64_t)SubSecondValue * 1000000000ull);
    val /= 0x100000000ull;
    return (u32_t)val;
}

/**
*\*\name    ETH_PTPTimeStampStart.
*\*\fun     Initialize timestamping ability of ETH.
*\*\param   UpdateMethod (The input parameters must be the following values):
*\*\          - ETH_PTP_FINE_UPDATE   : Fine Update method
*\*\          - ETH_PTP_COARSE_UPDATE : Coarse Update method
*\*\return  None
**/
void ETH_PTPTimeStampStart(uint32_t UpdateMethod)
{
    /* MAC-enabled timestamp snapshot of all received packets. */
    ETH_MACTimeStampAllPacketsCmd(ETH, ENABLE);
    /* Ultra-high precision mode, with a nanosecond register resolution of ~0.465 ns. */
    ETH_MACTimeStampDigBinRolloverCmd(ETH, DISABLE);
    /* Program Time stamp register bit 0 to enable time stamping. */
    ETH_MACTimeStampCmd(ETH, ENABLE);

    /* Program the Subsecond increment register based on the PTP clock frequency. */
    ETH_SetSubSecondIncrementValue(ETH, ADJ_FREQ_BASE_INCREMENT);

    if (UpdateMethod == ETH_PTP_FINE_UPDATE)
    {
        /* If you are using the Fine correction method, program the Time stamp addend register
         * and set Time stamp control register bit 5 (addend register update). */
        ETH_SetTimeStampAddendValue(ETH, ADJ_FREQ_BASE_ADDEND);
        ETH_MACTimeStampAddendRegUpdate(ETH);

        /* Poll the Time stamp control register until bit 5 is cleared. */
        while (ETH_GetPTPFlagStatus(ETH_PTP_FLAG_TSARU) == SET);
    }

    /* To select the Fine correction method (if required),
     * program Time stamp control register bit 1. */
    ETH_SetTimeStampUpdateMethod(ETH, UpdateMethod);

    /* Program the Time stamp high update and Time stamp low update registers
     * with the appropriate time value. */
    ETH_SetTimeStampUpdateValue(ETH, ETH_MACTS_CALMODE_ADD, 0, 0);

    /* Set Time stamp control register bit 2 (Time stamp init). */
    ETH_MACTimeStampInit(ETH);

    /* The Time stamp counter starts operation as soon as it is initialized
     * with the value written in the Time stamp update register. */
}

/**
*\*\name    ETH_PTPTimeStampGetTime.
*\*\fun     Get the current time.
*\*\param   timestamp:
*\*\          - For storing time values
*\*\return  None
**/
void ETH_PTPTimeStampGetTime(struct ptptime_t * timestamp)
{
    uint32_t Sec = 0;
    uint32_t Nsec = 0;
    uint32_t SecAfter = 0;

    /* The problem is we are reading two 32-bit registers that form
       a 64-bit value, but it's possible the high 32-bits of the value
       rolls over before we read the low 32-bits of the value. To avoid
       this situation we read the high 32-bits twice and determine which
       high 32-bits the low 32-bit are associated with.
    */
    __disable_irq();
    Sec = ETH_GetTimeStampRegValue(ETH, ETH_MACTS_STSR_OFFSET);
    Nsec = ETH_GetTimeStampRegValue(ETH, ETH_MACTS_STNSR_OFFSET);
    SecAfter = ETH_GetTimeStampRegValue(ETH, ETH_MACTS_STSR_OFFSET);
    __enable_irq();

    /* Did a roll over occur while reading? */
    if (Sec != SecAfter)
    {
        /* We now know a roll over occurred. If the rollover occured before
           the reading of the low 32-bits we move the substitute the second
           32-bit high value for the first 32-bit high value.
        */
        if (Nsec < (INT_MAX / 2))
        {
            Sec = SecAfter;
        }
    }
    
    /* Now convert the raw registers values into timestamp values. */
    timestamp->tv_sec = Sec;
    timestamp->tv_nsec = ETH_PTPSubSecond2NanoSecond(Nsec);
}

/**
*\*\name    ETH_PTPTimeStampSetTime.
*\*\fun     Initialize time base.
*\*\param   timestamp:
*\*\          - Time with sign
*\*\return  None
**/
void ETH_PTPTimeStampSetTime(struct ptptime_t * timestamp)
{
    uint32_t Sign;
    uint32_t SecondValue;
    uint32_t NanoSecondValue;
    uint32_t SubSecondValue;

    /* determine sign and correct Second and Nanosecond values */
    if(timestamp->tv_sec < 0 || (timestamp->tv_sec == 0 && timestamp->tv_nsec < 0))
    {
        /* Negative time value */
        Sign = ETH_MACTS_CALMODE_SUB;
        SecondValue = -timestamp->tv_sec;
        NanoSecondValue = -timestamp->tv_nsec;
    }
    else
    {
        /* Positive time value */
        Sign = ETH_MACTS_CALMODE_ADD;
        SecondValue = timestamp->tv_sec;
        NanoSecondValue = timestamp->tv_nsec;
    }

    /* convert nanosecond to subseconds */
    SubSecondValue = ETH_PTPNanoSecond2SubSecond(NanoSecondValue);

    /* Write the offset (positive or negative) in the Time stamp update high and low registers. */
    ETH_SetTimeStampUpdateValue(ETH, Sign, SecondValue, SubSecondValue);
    /* Set Time stamp control register bit 2 (Time stamp init). */
    ETH_MACTimeStampInit(ETH);
    /* The Time stamp counter starts operation as soon as it is initialized
     * with the value written in the Time stamp update register. */
    while(ETH_GetPTPFlagStatus(ETH_PTP_FLAG_TSSTI) == SET);
}

/**
*\*\name    ETH_PTPTimeStampAdjFreq.
*\*\fun     Updates time stamp addend register.
*\*\param   Adj:
*\*\          - Correction value in thousandth of ppb (Adj*10^9)
*\*\return  None
**/
void ETH_PTPTimeStampAdjFreq(int32_t Adj)
{
    uint32_t addend;
    
    /* Calculate the rate by which you want to speed up or slow down the system time increments */

    /* precise */
    /*
    int64_t addend;
    addend = Adj;
    addend *= ADJ_FREQ_BASE_ADDEND;
    addend /= 1000000000;
    addend += ADJ_FREQ_BASE_ADDEND;
    */
    addend = ADJ_FREQ_BASE_ADDEND + (int32_t)((((int64_t)ADJ_FREQ_BASE_ADDEND) * Adj) / 1000000000ull);
    
    /* Reprogram the Time stamp addend register with new Rate value and set ETH_TPTSCR */
    ETH_SetTimeStampAddendValue(ETH, addend);
    ETH_MACTimeStampAddendRegUpdate(ETH);
    /* Poll the Time stamp control register until bit 5 is cleared. */
    while (ETH_GetPTPFlagStatus(ETH_PTP_FLAG_TSARU) == SET);
}

/**
*\*\name    ETH_GetPTPFlagStatus.
*\*\fun     Checks whether the specified ETHERNET PTP flag is set or not.
*\*\param   Flag (The input parameters must be the following values):
*\*\          - ETH_PTP_FLAG_TSARU : Addend Register Update
*\*\          - ETH_PTP_FLAG_TSITE : Time Stamp Interrupt Trigger Enable
*\*\          - ETH_PTP_FLAG_TSSTU : Time Stamp Update
*\*\          - ETH_PTP_FLAG_TSSTI : Time Stamp Initialize
*\*\return  The new state of ETHERNET PTP Flag (SET or RESET)
**/
FlagStatus ETH_GetPTPFlagStatus(uint32_t Flag)
{
    FlagStatus bitstatus = RESET;

    if ((ETH->MACTSCTRL & Flag) != (uint32_t)RESET)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }
    return bitstatus;
}

