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
#include "timer_common.h"
#include "BSTIM_common.h"
#include "main.h"

uint32_t BSTIMClockFrequency = 0;

void GPIO_Config(void);

/**
*\*\brief  Main program
**/
int main(void)
{
    /* System Clocks Configuration */
    BSTIMClockFrequency = Common_BSTIM_RCC_Initialize(TIM6, RCC_HCLK_DIV4);

    /* Configure the GPIO ports */
    GPIO_Config();

    /* NVIC Configuration */
    Common_TIM_NVIC_Initialize(TIM6_IRQn, ENABLE);

    /* Time base configuration， period = 65535, prescaler = prescaler */
    Common_TIM_Base_Initialize(TIM6, 65535, 0);

    TIM_Base_Reload_Mode_Set(TIM6, TIM_PSC_RELOAD_MODE_IMMEDIATE);

    TIM_Interrupt_Enable(TIM6, TIM_INT_UPDATE);

    TIM_On(TIM6);

}

/**
*\*\brief  Configure the GPIOA Pins.
**/
void GPIO_Config(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_Structure_Initialize(&GPIO_InitStructure);

    GPIO_InitStructure.Pin        = GPIO_PIN_6;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_OUT_PP;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;

    GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);
}

void TIM6_IRQHandler(void)
{
    if (TIM_Interrupt_Status_Get(TIM6, TIM_INT_UPDATE) != RESET)
    {
        TIM_Interrupt_Status_Clear(TIM6, TIM_INT_UPDATE);

        /* Pin PC.06 toggling */
        GPIO_Pin_Toggle(GPIOB, GPIO_PIN_6);
    }
}
