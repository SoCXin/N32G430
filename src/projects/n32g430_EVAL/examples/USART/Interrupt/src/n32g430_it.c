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
*\*\file n32g430_it.c
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
**/
#include "n32g430_it.h"
#include "main.h"


extern uint8_t TxBuffer1[];
extern uint8_t TxBuffer2[];
extern uint8_t RxBuffer1[];
extern uint8_t RxBuffer2[];
extern __IO uint8_t TxCounter1;
extern __IO uint8_t TxCounter2;
extern __IO uint8_t RxCounter1;
extern __IO uint8_t RxCounter2;
extern uint8_t NbrOfDataToTransfer1;
extern uint8_t NbrOfDataToTransfer2;
extern uint8_t NbrOfDataToRead1;
extern uint8_t NbrOfDataToRead2;

/**  Cortex-M4 Processor Exceptions Handlers  **/


/**
*\*\name    NMI_Handler.
*\*\fun     This function handles NMI exception.
*\*\param   none
*\*\return  none 
**/
void NMI_Handler(void)
{
}

/**
*\*\name    HardFault_Handler.
*\*\fun     This function handles Hard Fault exception.
*\*\param   none
*\*\return  none 
**/
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}

/**
*\*\name    SVC_Handler.
*\*\fun     This function handles SVCall exception.
*\*\param   none
*\*\return  none 
**/
void SVC_Handler(void)
{
}

/**
*\*\name    PendSV_Handler.
*\*\fun     This function handles PendSV_Handler exception.
*\*\param   none
*\*\return  none 
**/
void PendSV_Handler(void)
{
}

/**
*\*\name    SysTick_Handler.
*\*\fun     This function handles SysTick Handler.
*\*\param   none
*\*\return  none 
**/
void SysTick_Handler(void)
{
}

/* N32G430 Peripherals Interrupt Handlers */
/* Add here the Interrupt Handler for the used peripheral(s) (PPP), for the */
/* available peripheral interrupt handler's name please refer to the startup */
/* file (startup_n32g430.s). */



/**
*\*\name    USARTy_IRQHandler.
*\*\fun     This function handles USARTy global interrupt request.
*\*\param   none
*\*\return  none 
**/
void USARTy_IRQHandler(void)
{
    if (USART_Interrupt_Status_Get(USARTy, USART_INT_RXDNE) != RESET)
    {
        /* Read one byte from the receive data register */
        RxBuffer1[RxCounter1++] = USART_Data_Receive(USARTy);

        if (RxCounter1 == NbrOfDataToRead1)
        {
            /* Disable the USARTy Receive interrupt */
            USART_Interrput_Enable(USARTy, USART_INT_RXDNE);
        }
    }

    if (USART_Interrupt_Status_Get(USARTy, USART_INT_TXDE) != RESET)
    {
        /* Write one byte to the transmit data register */
        USART_Data_Send(USARTy, TxBuffer1[TxCounter1++]);

        if (TxCounter1 == NbrOfDataToTransfer1)
        {
            /* Disable the USARTy Transmit interrupt */
            USART_Interrput_Disable(USARTy, USART_INT_TXDE);
        }
    }
}


/**
*\*\name    USARTz_IRQHandler.
*\*\fun     This function handles USARTz global interrupt request.
*\*\param   none
*\*\return  none 
**/
void USARTz_IRQHandler(void)
{
    if (USART_Interrupt_Status_Get(USARTz, USART_INT_RXDNE) != RESET)
    {
        /* Read one byte from the receive data register */
        RxBuffer2[RxCounter2++] = USART_Data_Receive(USARTz);

        if (RxCounter2 == NbrOfDataToRead1)
        {
            /* Disable the USARTz Receive interrupt */
            USART_Interrput_Disable(USARTz, USART_INT_RXDNE);
        }
    }

    if (USART_Interrupt_Status_Get(USARTz, USART_INT_TXDE) != RESET)
    {
        /* Write one byte to the transmit data register */
        USART_Data_Send(USARTz, TxBuffer2[TxCounter2++]);

        if (TxCounter2 == NbrOfDataToTransfer2)
        {
            /* Disable the USARTz Transmit interrupt */
            USART_Interrput_Disable(USARTz, USART_INT_TXDE);
        }
    }
}


