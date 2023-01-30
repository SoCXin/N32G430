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
**/

#include <stdio.h>
#include "main.h"


#define TxBufferSize1 (countof(TxBuffer1) - 1)
#define TxBufferSize2 (countof(TxBuffer2) - 1)

#define countof(a) (sizeof(a) / sizeof(*(a)))

USART_InitType USART_InitStructure;
uint8_t TxBuffer1[] = "USART DMA Interrupt: USARTy -> USARTz using DMA Tx and Rx Flag";
uint8_t TxBuffer2[] = "USART DMA Interrupt: USARTz -> USARTy using DMA Tx and Rx Interrupt";
uint8_t RxBuffer1[TxBufferSize2];
uint8_t RxBuffer2[TxBufferSize1];
uint8_t NbrOfDataToRead             = TxBufferSize1;
uint32_t indexFlag                  = 0;
volatile TestStatus TransferStatus1 = FAILED, TransferStatus2 = FAILED;

/**
*\*\name    main.
*\*\fun     Main program.
*\*\param   none
*\*\return  none
**/
int main(void)
{
    /* System Clocks Configuration */
    RCC_Configuration();

    /* NVIC configuration */
    NVIC_Configuration();

    /* Configure the GPIO ports */
    GPIO_Configuration();

    /* Configure the DMA */
    DMA_Configuration();

    /* USARTy and USARTz configuration */
    USART_InitStructure.BaudRate            = 115200;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;

    /* Configure USARTy and USARTz */
    USART_Initializes(USARTy, &USART_InitStructure);
    USART_Initializes(USARTz, &USART_InitStructure);

    /* Enable USARTy DMA Rx and TX request */
    USART_DMA_Transfer_Enable(USARTy, USART_DMAREQ_RX | USART_DMAREQ_TX);
    /* Enable USARTz DMA Rx and TX request */
    USART_DMA_Transfer_Enable(USARTz, USART_DMAREQ_RX | USART_DMAREQ_TX);

    /* Enable the USARTz Receive Interrupt */
    USART_Interrput_Enable(USARTz, USART_INT_RXDNE);

    /* Enable USARTy TX DMA Channel */
    DMA_Channel_Enable(USARTy_Tx_DMA_Channel);
    /* Enable USARTz TX DMA Channel */
    DMA_Channel_Enable(USARTz_Tx_DMA_Channel);

    /* Enable the USARTy and USARTz */
    USART_Enable(USARTy);
    USART_Enable(USARTz);

    /* Receive the TxBuffer2 */
    while (indexFlag < TxBufferSize2)
    {
        while (USART_Flag_Status_Get(USARTy, USART_FLAG_RXDNE) == RESET)
        {
        }
        RxBuffer1[indexFlag++] = USART_Data_Receive(USARTy);
    }

    /* Wait until USARTy TX DMA Channel  Transfer Complete */
    while (DMA_Flag_Status_Get(DMA,USARTy_Tx_DMA_FLAG) == RESET)
    {
    }
    /* Wait until USARTz TX DMA Channel Transfer Complete */
    while (DMA_Flag_Status_Get(DMA,USARTz_Tx_DMA_FLAG) == RESET)
    {
    }

    /* Check the received data with the send ones */
    TransferStatus1 = Buffercmp(TxBuffer2, RxBuffer1, TxBufferSize2);
    /* TransferStatus1 = PASSED, if the data transmitted from USARTz and
       received by USARTy are the same */
    /* TransferStatus1 = FAILED, if the data transmitted from USARTz and
       received by USARTy are different */
    TransferStatus2 = Buffercmp(TxBuffer1, RxBuffer2, TxBufferSize1);
    /* TransferStatus2 = PASSED, if the data transmitted from USARTy and
       received by USARTz are the same */
    /* TransferStatus2 = FAILED, if the data transmitted from USARTy and
       received by USARTz are different */

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
    /* DMA clock enable */
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_DMA);
    /* Enable GPIO clock */
    RCC_AHB_Peripheral_Clock_Enable(USARTy_GPIO_CLK | USARTz_GPIO_CLK);
	RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_AFIO);
    /* Enable USARTy and USARTz Clock */
    USARTy_APBxClkCmd(USARTy_CLK);
    USARTz_APBxClkCmd(USARTz_CLK);
}


/**
*\*\name    GPIO_Configuration.
*\*\fun     Configures the different GPIO ports.
*\*\param   none
*\*\return  none
**/
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    /* Initialize GPIO_InitStructure */
    GPIO_Structure_Initialize(&GPIO_InitStructure);
    
    /* Configure USARTy Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = USARTy_TxPin;    
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = USARTy_Tx_GPIO_AF;
    GPIO_Peripheral_Initialize(USARTy_GPIO, &GPIO_InitStructure);

    /* Configure USARTz Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = USARTz_TxPin;
    GPIO_InitStructure.GPIO_Alternate = USARTz_Tx_GPIO_AF;
    GPIO_Peripheral_Initialize(USARTz_GPIO, &GPIO_InitStructure); 

    /* Configure USARTx Rx as alternate function push-pull */
    GPIO_InitStructure.Pin            = USARTy_RxPin;
    GPIO_InitStructure.GPIO_Alternate = USARTy_Rx_GPIO_AF;
    GPIO_Peripheral_Initialize(USARTy_GPIO, &GPIO_InitStructure);    

    /* Configure USARTz Rx as alternate function push-pull */
    GPIO_InitStructure.Pin            = USARTz_RxPin;
    GPIO_InitStructure.GPIO_Alternate = USARTz_Rx_GPIO_AF;
    GPIO_Peripheral_Initialize(USARTz_GPIO, &GPIO_InitStructure);    
}


/**
*\*\name    NVIC_Configuration.
*\*\fun     Configures the nested vectored interrupt controller.
*\*\param   none
*\*\return  none
**/
void NVIC_Configuration(void)
{
    NVIC_InitType NVIC_InitStructure;

    /* Enable the USARTz Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = USARTz_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);
}


/**
*\*\name    DMA_Configuration.
*\*\fun     Configures the DMA.
*\*\param   none
*\*\return  none
**/
void DMA_Configuration(void)
{
    DMA_InitType DMA_InitStructure;

    /* USARTy_Tx_DMA_Channel (triggered by USARTy Tx event) Config */
    DMA_Reset(USARTy_Tx_DMA_Channel);
    DMA_InitStructure.PeriphAddr     = USARTy_DAT_Base;
    DMA_InitStructure.MemAddr        = (uint32_t)TxBuffer1;
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_DST;
    DMA_InitStructure.BufSize        = TxBufferSize1;
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_MODE_DISABLE;
    DMA_InitStructure.MemoryInc      = DMA_MEM_INC_MODE_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_WIDTH_BYTE;
    DMA_InitStructure.MemDataSize    = DMA_MEM_DATA_WIDTH_BYTE;
    DMA_InitStructure.CircularMode   = DMA_CIRCULAR_MODE_DISABLE;
    DMA_InitStructure.Priority       = DMA_CH_PRIORITY_HIGHEST;
    DMA_InitStructure.Mem2Mem        = DMA_MEM2MEM_DISABLE;
    DMA_Initializes(USARTy_Tx_DMA_Channel, &DMA_InitStructure);
    DMA_Channel_Request_Remap(USARTy_Tx_DMA_Channel,DMA_REMAP_USART1_TX);

    /* USARTz_Tx_DMA_Channel (triggered by USARTz Tx event) Config */
    DMA_Reset(USARTz_Tx_DMA_Channel);
    DMA_InitStructure.PeriphAddr = USARTz_DAT_Base;
    DMA_InitStructure.MemAddr    = (uint32_t)TxBuffer2;
    DMA_InitStructure.Direction  = DMA_DIR_PERIPH_DST;
    DMA_InitStructure.BufSize    = TxBufferSize2;
    DMA_Initializes(USARTz_Tx_DMA_Channel, &DMA_InitStructure);
    DMA_Channel_Request_Remap(USARTz_Tx_DMA_Channel,DMA_REMAP_USART2_TX);
}


/**
*\*\name    Buffercmp.
*\*\fun     Compares two buffers.
*\*\param   pBuffer1
*\*\param   pBuffer2
*\*\param   BufferLength
*\*\return  PASSED or FAILED
**/
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
    while (BufferLength--)
    {
        if (*pBuffer1 != *pBuffer2)
        {
            return FAILED;
        }

        pBuffer1++;
        pBuffer2++;
    }

    return PASSED;
}

#ifdef USE_FULL_ASSERT


/**
*\*\name    assert_failed.
*\*\fun     Reports the name of the source file and the source line number
*\*\        where the assert_param error has occurred.
*\*\param   file pointer to the source file name
*\*\param   line assert_param error line source number
*\*\return  none
**/
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}

#endif

