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
*\*\file main.c
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#include "main.h"
#include "log.h"

#include <string.h>

#define BUFFER_SIZE (sizeof(SRC_Const_Buffer))

DMA_InitType DMA_InitStructure;
__IO uint32_t CurrDataCounterBegin = 0;
__IO uint32_t CurrDataCounterEnd   = 0x01; /* This variable should not be initialized to 0 */

const uint32_t SRC_Const_Buffer[] = {
    0x72aa0305, 0x839916bf, 0x913782c5, 0x7e3bd953, 0x4174430c, 0x40d59981, 0xbccff537, 0xd22bca1a,
    0xda039c07, 0xff38219a, 0x17f16c33, 0x0056b9cf, 0x4e75ab0a, 0x0751b137, 0xbb051ec9, 0xcac01e06,
    0x5de647ba, 0xf2bd2108, 0x60a32be0, 0xbf80a68f, 0x299b4a55, 0xd37b1965, 0x84a3ca5b, 0x0f08f335,
    0x21c4084b, 0x738688ca, 0xe7971a03, 0xadcd6bfc, 0xd4bb55ae, 0x0ea4ed88, 0x131711a3, 0x4e92f42f,
};

uint32_t DST_Buffer[BUFFER_SIZE];

void RCC_Configuration(void);
void NVIC_Configuration(void);

/**
*\*\name    main.
*\*\fun     Main program.
*\*\param   none
*\*\return  none
**/
int main(void)
{
    log_init();
    log_info("DMA Flash to RAM Demo\n");

    /* System Clocks Configuration */
    RCC_Configuration();

    /* NVIC configuration */
    NVIC_Configuration();

    /* DMA channel6 configuration */
    DMA_Reset(DMA_CHANNEL_USED);
    DMA_InitStructure.PeriphAddr     = (uint32_t)SRC_Const_Buffer;
    DMA_InitStructure.MemAddr        = (uint32_t)DST_Buffer;
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_SRC;
    DMA_InitStructure.BufSize        = BUFFER_SIZE;
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_MODE_ENABLE;
    DMA_InitStructure.MemoryInc      = DMA_MEM_INC_MODE_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_WIDTH_WORD;
    DMA_InitStructure.MemDataSize    = DMA_MEM_DATA_WIDTH_WORD;
    DMA_InitStructure.CircularMode   = DMA_CIRCULAR_MODE_DISABLE;
    DMA_InitStructure.Priority       = DMA_CH_PRIORITY_HIGH;
    DMA_InitStructure.Mem2Mem        = DMA_MEM2MEM_ENABLE;
    DMA_Initializes(DMA_CHANNEL_USED, &DMA_InitStructure);

    /* Enable DMA Channel6 Transfer Complete interrupt */
    DMA_Interrupts_Enable(DMA_CHANNEL_USED, DMA_INT_TXC);

    /* Get Current Data Counter value before transfer begins */
    CurrDataCounterBegin = DMA_Current_Data_Transfer_Number_Get(DMA_CHANNEL_USED);

    /* Enable DMA Channel6 transfer */
    DMA_Channel_Enable(DMA_CHANNEL_USED);

    /* Wait the end of transmission */
    while (CurrDataCounterEnd != 0)
    {
    }

    /* Check if the transmitted and received data are equal */
    if (memcmp(SRC_Const_Buffer, DST_Buffer, BUFFER_SIZE) == 0)
    {
        log_info("DMA Flash to RAM passed\n");
    }
    else
    {
        log_error("DMA Flash to RAM failed\n");
    }

    while (1)
    {
    }
}

/**
*\*\name    RCC_Configuration.
*\*\fun     Configures the different system clocks.
*\*\param   none
*\*\return  none
**/
void RCC_Configuration(void)
{
    /* Enable peripheral clocks */
    /* Enable DMA clock */
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_DMA);
}

/**
*\*\name    NVIC_Configuration.
*\*\fun     Configure the nested vectored interrupt controller.
*\*\param   none
*\*\return  none
**/
void NVIC_Configuration(void)
{
    NVIC_InitType NVIC_InitStructure;

    /* Enable DMA channel6 IRQ Channel */
    NVIC_InitStructure.NVIC_IRQChannel                   = DMA_IRQN;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);
}

/**
*\*\name    Buffercmp.
*\*\fun     Compares two buffers.
*\*\param   pBuffer buffers to be compared
*\*\param   pBuffer1 buffers to be compared
*\*\param   BufferLength buffer's length
*\*\return  PASSED or FAILED
**/
Status Buffercmp(const uint32_t* pBuffer, uint32_t* pBuffer1, uint16_t BufferLength)
{
    while (BufferLength--)
    {
        if (*pBuffer != *pBuffer1)
        {
            return FAILED;
        }

        pBuffer++;
        pBuffer1++;
    }

    return PASSED;
}



