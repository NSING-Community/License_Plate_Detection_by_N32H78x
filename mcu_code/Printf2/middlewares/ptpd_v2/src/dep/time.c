/* time.c */

#include "../ptpd.h"

void getTime(TimeInternal *time)
{
    struct ptptime_t timestamp;
    ETH_PTPTimeStampGetTime(&timestamp);
    time->seconds = timestamp.tv_sec;
    time->nanoseconds = timestamp.tv_nsec;
}

void setTime(const TimeInternal *time)
{
    struct ptptime_t timestamp;
    timestamp.tv_sec = time->seconds;
    timestamp.tv_nsec = time->nanoseconds;
    ETH_PTPTimeStampSetTime(&timestamp);
    DBGP("setTime: resetting system clock to %ds %dns\n", time->seconds, time->nanoseconds);
}

UInteger32 getRand(UInteger32 randMax)
{
    return rand() % randMax;
}

Boolean adjFreq(Integer32 adj)
{
    DBGV("adjFreq %d\n", adj);

    if (adj > ADJ_FREQ_MAX)
        adj = ADJ_FREQ_MAX;
    else if (adj < -ADJ_FREQ_MAX)
        adj = -ADJ_FREQ_MAX;

    /* Fine update method */
    ETH_PTPTimeStampAdjFreq(adj);
    DBGV("%s: updated\n", __FUNCTION__);
    return TRUE;
}
