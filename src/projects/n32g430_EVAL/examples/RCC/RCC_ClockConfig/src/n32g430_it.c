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
#include "n32g430.h"

/*** Cortex-M4 Processor Exceptions Handlers ***/

/**
*\*\name    NMI_Handler.
*\*\fun     This function handles NMI exception.
*\*\param   none
*\*\return  none 
**/
void NMI_Handler(void)
{
    /* This interrupt is generated when HSE clock fails */

    if (RCC_Interrupt_Status_Get(RCC_INT_CLKSSIF) != RESET)
    {
        /* At this stage: HSE, PLL are disabled (but no change on PLL config) and HSI
            is selected as system clock source */

        /* Enable HSE */
        RCC_HSE_Config(RCC_HSE_ENABLE);

        /* Enable HSE Ready interrupt */
        RCC_Interrupt_Enable(RCC_INT_HSERDIEN);

#ifndef SYSCLK_HSE
        /* Enable PLL Ready interrupt */
        RCC_Interrupt_Enable(RCC_INT_PLLRDIEN);
#endif /* SYSCLK_HSE */

        /* Clear Clock Security System interrupt pending bit */
        RCC_Interrupt_Status_Clear(RCC_INT_CLKSSICLR);

        /* Once HSE clock recover, the HSERDY interrupt is generated and in the RCC INTSTS
           routine the system clock will be reconfigured to its previous state (before
           HSE clock failure) */
    }
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
*\*\name    MemManage_Handler.
*\*\fun     This function handles Memory Manage exception.
*\*\param   none
*\*\return  none 
**/
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/**
*\*\name    BusFault_Handler.
*\*\fun     This function handles Bus Fault exception.
*\*\param   none
*\*\return  none 
**/
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}

/**
*\*\name    UsageFault_Handler.
*\*\fun     This function handles Usage Fault exception.
*\*\param   none
*\*\return  none 
**/
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
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
*\*\name    DebugMon_Handler.
*\*\fun     This function handles Debug Monitor exception.
*\*\param   none
*\*\return  none 
**/
void DebugMon_Handler(void)
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

/** N32G430 Peripherals Interrupt Handlers, interrupt handler's name please refer 
     to the startup file (startup_n32g430.s) **/

/** This function handles RCC interrupt request. **/
/**
*\*\name    RCC_IRQHandler.
*\*\fun     This function handles RCC interrupt request.
*\*\param   none
*\*\return  none 
**/
void RCC_IRQHandler(void)
{
    if (RCC_Interrupt_Status_Get(RCC_INT_HSERDIF) != RESET)
    {
        /* Clear HSERDY interrupt pending bit */
        RCC_Interrupt_Status_Clear(RCC_INT_HSERDICLR);

        /* Check if the HSE clock is still available */
        if (RCC_Flag_Status_Get(RCC_FLAG_HSERD) != RESET)
        {
#ifdef SYSCLK_HSE
            /* Select HSE as system clock source */
            RCC_Sysclk_Config(RCC_SYSCLK_SRC_HSE);
#else
            /* Enable PLL: once the PLL is ready the PLLRDY interrupt is generated */
            RCC_PLL_Enable();
#endif /* SYSCLK_HSE */
        }
    }

    if (RCC_Interrupt_Status_Get(RCC_INT_PLLRDIF) != RESET)
    {
        /* Clear PLLRDY interrupt pending bit */
        RCC_Interrupt_Status_Clear(RCC_INT_PLLRDICLR);

        /* Check if the PLL is still locked */
        if (RCC_Flag_Status_Get(RCC_FLAG_PLLRD) != RESET)
        {
            /* Select PLL as system clock source */
            RCC_Sysclk_Config(RCC_SYSCLK_SRC_PLLCLK);
        }
    }
}


