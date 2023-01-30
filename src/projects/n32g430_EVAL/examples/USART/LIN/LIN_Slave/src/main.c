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


static uint8_t max_ms = 116;
extern uint8_t IDType[64];
extern uint8_t GotMsgFlag;
extern LIN_EX_MSG *pLINMsg;

/**
*\*\name    delay_xms.
*\*\fun     delay program.
*\*\param   nms
*\*\return  none
**/
void delay_xms(uint32_t nms)
{
    uint16_t i;
    uint16_t count_1 = nms / max_ms;
    uint16_t count_2 = nms % max_ms;
    if(0 == count_1)
    {
        systick_delay_ms(nms);
    }
    else
    {
        for(i = 0; i < count_1; i++)
        {
            systick_delay_ms(max_ms);
        }
        if(count_2 != 0)
        {
            systick_delay_ms(count_2);
        }
    }
}

/**
*\*\name    Buffercopy.
*\*\fun     Compares two buffers.
*\*\param   pBuffer1
*\*\param   pBuffer2
*\*\param   buffer's length
*\*\return  s
**/
void Buffercopy(uint8_t *dest, uint8_t *src, uint16_t BufferLength)
{
    while (BufferLength--)
    {
        *dest = *src;
        dest++;
        src++;
    }
}

void LIN_RespData(uint8_t *pData, uint8_t Len)
{
    LIN_SetResp(MSG_SEND_ID, pData, Len, CHECK_TYPE);
}

/**
*\*\name    main.
*\*\fun     Main program.
*\*\param   none
*\*\return  none
**/
int main(void)
{
		USART_InitType USART_InitStructure;
    log_init();
    printf("\r\n test LIN slave mode\r\n");
    /* System Clocks Configuration */
    RCC_Configuration();	

    /* NVIC configuration */
    NVIC_Configuration();

    /* Configure the GPIO ports */
    GPIO_Configuration();

    /* USARTx configuration ------------------------------------------------------*/
    USART_InitStructure.BaudRate            = 9600;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;
    /* Configure USARTx */
    USART_Initializes(USARTx, &USART_InitStructure);
    /* Enable the USARTx LIN mode*/
		USART_LIN_Break_Detect_Length_Set(USARTx,USART_LINBDL_10B);
		USART_LIN_Enable(USARTx);
    /* Enable USARTx Receive and Transmit interrupts */
    USART_Interrput_Enable(USARTx, USART_INT_RXDNE);
    //USART_ConfigInt(USARTx, USART_INT_TXDE, ENABLE);
    USART_Interrput_Enable(USARTx, USART_INT_LINBD);
		
    /* Enable the USARTx */
    USART_Enable(USARTx);

    //configure slave ID mode 
    IDType[MSG_RECEIVE_ID & 0x3F] = ID_TYPE_SR;
    IDType[MSG_SEND_ID & 0x3F] = ID_TYPE_SW;
    while (1)
    {
        if(GotMsgFlag)
        {
            BOOT_ExecutiveCommand(pLINMsg->Data, LIN_RespData);
            GotMsgFlag = 0;
        }			
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
    /* Enable GPIO clock */
    GPIO_AHBClkCmd(USARTx_GPIO_CLK);
	  RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_AFIO);
	  
    /* Enable USARTx Clock */
    USART_APBxClkCmd(USARTx_CLK);
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

    /* Configure USARTx Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = USARTx_TxPin;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = USARTx_Tx_GPIO_AF;
    GPIO_Peripheral_Initialize(USARTx_GPIO, &GPIO_InitStructure);   

    /* Configure USARTx Rx as alternate function push-pull */
    GPIO_InitStructure.Pin            = USARTx_RxPin;
    GPIO_InitStructure.GPIO_Alternate = USARTx_Rx_GPIO_AF;
    GPIO_Peripheral_Initialize(USARTx_GPIO, &GPIO_InitStructure); 
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

    /* Enable the USARTx Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = USARTx_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);

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


