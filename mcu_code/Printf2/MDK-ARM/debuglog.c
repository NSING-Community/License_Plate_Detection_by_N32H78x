// debug_log.c (?????? .c,??? Keil ????? C ??)
#include "tensorflow/lite/micro/debug_log.h"
#include <stdarg.h>  // ?? va_list ??
#include "Message.h" // ????????????,?? "n32h7xx_usart.h"


// ??????? (????? MicroLib)
#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    __asm(".global __use_no_semihosting\n\t");
    __asm(".global __ARM_use_no_argv\n\t");
#elif defined(__ARMCC_VERSION)
    #pragma import(__use_no_semihosting)
#endif


void DebugLog(const char* format, va_list args) {
    char buffer[256];  // ????????????
    
    // ?? vsnprintf ???
    vsnprintf(buffer, sizeof(buffer), format, args);
    
    // ?????
    UART_SendString(buffer);
}