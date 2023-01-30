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
#include "bsp_key.h"



__IO uint32_t ControlFlag = 0;

extern void Delay(__IO uint32_t nCount);


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
*\*\name    EXTI1_IRQHandler.
*\*\fun     This function handles External interrupt Line 0 request.
*\*\param   none
*\*\return  none 
**/
void EXTI1_IRQHandler(void)
{
    Delay(0x2FFFFF);
    if (ControlFlag == 2)
    {
        ControlFlag = 0;
    }

    if (ControlFlag == 0)
    {
        /* Flush DAT register and clear the USARTz RXNE flag */
        USART_Data_Receive(USARTz);
        /* Enable the USARTz mute mode*/
        USART_Receiver_Wakeup_Enable(USARTz);

        ControlFlag = 1;
    }
    else if (ControlFlag == 1)
    {
        /* Send the address mark (0x102) to wakeup USARTz */
        USART_Data_Send(USARTy, 0x102);
        /* Wait while USARTy TXE = 0 */
        while (USART_Flag_Status_Get(USARTy, USART_FLAG_TXDE) == RESET)
        {
        }

        ControlFlag = 2;
    }

    /* Clear Key Button EXTI Line Pending Bit */
    EXTI_Interrupt_Status_Clear(KEY1_INT_EXTI_LINE);
}


