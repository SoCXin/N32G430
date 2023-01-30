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


#include "n32g430.h"
#include "n32g430_it.h"
#include "main.h"
#include "log.h"


/** Cortex-M4 Processor Exceptions Handlers **/

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

uint8_t Delay_100Ms_Cnt = 0;
extern volatile uint32_t RTC_Delay_Flag;

#ifdef RTC_DELAY_USE_TIM6
/**
*\*\name    TIM6_IRQHandler.
*\*\fun     This function handles TIM6 global interrupt request.
*\*\param   none
*\*\return  none 
**/
void TIM6_IRQHandler(void)
{
    if (TIM_Interrupt_Status_Get(TIM6, TIM_INT_UPDATE) != RESET)
    {
        Delay_100Ms_Cnt++;

        if(Delay_100Ms_Cnt == 11)
        {
            RTC_Delay_Flag = 1;
            Delay_100Ms_Cnt = 0;
            /* Disable the TIM6 Counter */
            TIM6->CTRL1 &= (uint32_t)(~((uint32_t)TIM_CTRL1_CNTEN));
        }
        TIM_Interrupt_Status_Clear(TIM6, TIM_INT_UPDATE);
    }
}
#else
/**
*\*\name    SysTick_Handler.
*\*\fun     This function handles SysTick Handler.
*\*\param   none
*\*\return  none 
**/
void SysTick_Handler(void)
{
    Delay_100Ms_Cnt++;
    if(Delay_100Ms_Cnt == 11)
    {
        RTC_Delay_Flag = 1;
        Delay_100Ms_Cnt = 0;
        /* Disable the SysTick Counter */
        SysTick->CTRL &= (~SysTick_CTRL_ENABLE_Msk);
    }
}
#endif

/** N32G430 Peripherals Interrupt Handlers, interrupt handler's name please refer to the startup file (startup_n32g430.s) **/

/**
*\*\name    RTC_TAMPER_STAMP_IRQHandler.
*\*\fun     Display the current TimeStamp (time and date) and tamper on the Hyperterminal.
*\*\param   none
*\*\return  none 
**/

void RTC_TAMPER_STAMP_IRQHandler(void)
{
    if (RTC_Interrupt_Status_Get(RTC_INT_TST) != RESET)
    {
        /* Every interval delay (1s) displays calendar and clock stamp register contents */
        RTC_Date_Show();
        RTC_Time_Show();
        RTC_TimeStamp_Show();
        RTC_Interrupt_Status_Clear(RTC_INT_TST);
        EXTI_Interrupt_Status_Clear(EXTI_LINE18);
    }
    
}
