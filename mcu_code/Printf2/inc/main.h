/*****************************************************************************
 * Copyright (c) 2022, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file main.h
 * @author Nations
 * @version V1.2.2
 *
 * @copyright Copyright (c) 2022, Nations Technologies Inc. All rights reserved.
 */
#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "n32h7xx.h"
#include "n32h7xx_usart.h"
#include "n32h7xx_rcc.h"
#include "n32h7xx_gpio.h"
#include "misc.h"
#include "n32h7xx_sdram.h"	


#define _USART1_COM_

#ifdef _USART3_COM_
#define USARTx                  USART3
#define USARTx_CLK              RCC_APB1_PERIPHEN_M7_USART3
#define USARTx_GPIO             GPIOD
#define USARTx_GPIO_CLK         RCC_AHB5_PERIPHEN_M7_GPIOD
#define USARTx_TxPin            GPIO_PIN_8
#define USARTx_RxPin            GPIO_PIN_9
#define USARTx_Tx_GPIO_AF       GPIO_AF6
/* Tx and Rx do NOT share an alternate function on this part: the vendor's own
   examples\USART\Polling\inc\main.h pairs USART3 Tx AF6 with Rx AF5. */
#define USARTx_Rx_GPIO_AF       GPIO_AF5
#define GPIO_AHBClkCmd          RCC_EnableAHB5PeriphClk1   //RCC_EnableAHB5PeriphClk2 I~J
#define USART_APBxClkCmd        RCC_EnableAPB1PeriphClk3
	
#elif defined(_USART1_COM_)	
#define USARTx                  USART1
#define USARTx_CLK              RCC_APB1_PERIPHEN_M7_USART1
#define USARTx_GPIO             GPIOA
#define USARTx_GPIO_CLK         RCC_AHB5_PERIPHEN_M7_GPIOA
#define USARTx_TxPin            GPIO_PIN_9
#define USARTx_RxPin            GPIO_PIN_10
#define USARTx_Tx_GPIO_AF       GPIO_AF7
/* Tx and Rx do NOT share an alternate function on this part.  bsp/src/log.c,
   the vendor's USART1 setup for this core, has PA9 on AF7 but PA10 on AF5. */
#define USARTx_Rx_GPIO_AF       GPIO_AF5
#define GPIO_AHBClkCmd          RCC_EnableAHB5PeriphClk1   //RCC_EnableAHB5PeriphClk2 I~J
#define USART_APBxClkCmd        RCC_EnableAPB1PeriphClk3	
#endif

typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;

void RCC_Configuration(void);
void GPIO_Configuration(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */
