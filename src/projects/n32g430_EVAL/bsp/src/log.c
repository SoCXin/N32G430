/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
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
*\*\file log.c
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
**/
#include "log.h"

#if LOG_ENABLE

#include "n32g430.h"

#define LOG_USARTx      USART1
#define LOG_PERIPH      RCC_APB2_PERIPH_USART1
#define LOG_GPIO        GPIOA
#define LOG_PERIPH_GPIO RCC_AHB_PERIPH_GPIOA
#define LOG_TX_PIN      GPIO_PIN_9
#define LOG_RX_PIN      GPIO_PIN_10

void log_init(void)
{
    GPIO_InitType GPIO_InitStructure;
    USART_InitType USART_InitStructure;

    RCC_AHB_Peripheral_Clock_Enable(LOG_PERIPH_GPIO);
    RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_AFIO | LOG_PERIPH);

    GPIO_Structure_Initialize(&GPIO_InitStructure);
    GPIO_InitStructure.Pin            = LOG_TX_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF5_USART1;
    GPIO_Peripheral_Initialize(LOG_GPIO, &GPIO_InitStructure);

//    GPIO_InitStructure.Pin             = LOG_RX_PIN;
//    GPIO_InitStructure.GPIO_Alternate  = GPIO_AF5_USART1;
//    GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.BaudRate            = 115200;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_TX;

    /* init uart */
    USART_Initializes(LOG_USARTx, &USART_InitStructure);

    /* enable uart */
    USART_Enable(LOG_USARTx);
}

static int is_lr_sent = 0;

int fputc(int ch, FILE* f)
{
    if (ch == '\r')
    {
        is_lr_sent = 1;
    }
    else if (ch == '\n')
    {
        if (!is_lr_sent)
        {
            USART_Data_Send(LOG_USARTx, (uint8_t)'\r');
            /* Loop until the end of transmission */
            while (USART_Flag_Status_Get(LOG_USARTx, USART_FLAG_TXC) == RESET)
            {
            }
        }
        is_lr_sent = 0;
    }
    else
    {
        is_lr_sent = 0;
    }
    USART_Data_Send(LOG_USARTx, (uint8_t)ch);
    /* Loop until the end of transmission */
    while (USART_Flag_Status_Get(LOG_USARTx, USART_FLAG_TXDE) == RESET)
    {
    }
    return ch;
}

#endif /* LOG_ENABLE */