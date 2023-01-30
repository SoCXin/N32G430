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
#include "ADTIM_common.h"
#include "GPTIM_common.h"
#include "timer_common.h"
#include "main.h"

uint16_t capture = 0;
__IO uint16_t CCR1_Val  = 32768;
__IO uint16_t CCR2_Val  = 16384;
__IO uint16_t CCR3_Val  = 8192;
__IO uint16_t CCR4_Val  = 4096;
uint16_t PrescalerValue = 0;
uint32_t GPTIMClockFrequency = 0;

OCInitType TIM_OCInitStructure;

void TIM3_IRQHandler(void);

int main(void)
{
    /* System Clocks Configuration */
    GPTIMClockFrequency = Common_GPTIM_RCC_Initialize(TIM3, RCC_HCLK_DIV4);

    /* Configure the GPIO ports */
    Common_GPTIM_GPIO_Initialize(TIM3);
    
    Common_TIM_NVIC_Initialize(TIM3_IRQn, ENABLE);

    PrescalerValue = (uint16_t)(GPTIMClockFrequency / 25600000) - 1;
    
    /* TIM Base init, Period = 65535, Prescaler = PrescalerValue*/
    Common_TIM_Base_Initialize(TIM3, 65535, PrescalerValue);

    TIM_Output_Channel_Struct_Initialize(&TIM_OCInitStructure);

    TIM_OCInitStructure.OcMode      = TIM_OCMODE_TOGGLE;
    TIM_OCInitStructure.OcPolarity  = TIM_OC_POLARITY_LOW;
    
    /* Output Compare Active Mode configuration: Channel1 */
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = CCR1_Val;
    TIM_Output_Channel1_Initialize(TIM3, &TIM_OCInitStructure);

    TIM_Output_Channel1_Preload_Set(TIM3, TIM_OC_PRELOAD_DISABLE);

    /* Output Compare Active Mode configuration: Channel2 */
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = CCR2_Val;
    TIM_Output_Channel2_Initialize(TIM3, &TIM_OCInitStructure);

    TIM_Output_Channel2_Preload_Set(TIM3, TIM_OC_PRELOAD_DISABLE);

    /* Output Compare Active Mode configuration: Channel3 */
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = CCR3_Val;
    TIM_Output_Channel3_Initialize(TIM3, &TIM_OCInitStructure);

    TIM_Output_Channel3_Preload_Set(TIM3, TIM_OC_PRELOAD_DISABLE);

    /* Output Compare Active Mode configuration: Channel4 */
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = CCR4_Val;
    TIM_Output_Channel4_Initialize(TIM3, &TIM_OCInitStructure);

    TIM_Output_Channel4_Preload_Set(TIM3, TIM_OC_PRELOAD_DISABLE);


    TIM_On(TIM3);
    
    TIM_Interrupt_Enable(TIM3, TIM_INT_CC1 | TIM_INT_CC2 | TIM_INT_CC3 | TIM_INT_CC4);

    while(1)
    {

    }
}

/**
*\*\brief  This function handles TIM3 global interrupt request.
**/
void TIM3_IRQHandler(void)
{
    if (TIM_Interrupt_Status_Get(TIM3, TIM_INT_CC1) != RESET)
    {
        TIM_Interrupt_Status_Clear(TIM3, TIM_INT_CC1);
        capture = TIM_Compare_Capture1_Get(TIM3);
        TIM_Compare1_Set(TIM3, capture + CCR1_Val);
    }

    if (TIM_Interrupt_Status_Get(TIM3, TIM_INT_CC2) != RESET)
    {
        TIM_Interrupt_Status_Clear(TIM3, TIM_INT_CC2);
        capture = TIM_Compare_Capture2_Get(TIM3);
        TIM_Compare2_Set(TIM3, capture + CCR2_Val);
    }

    if (TIM_Interrupt_Status_Get(TIM3, TIM_INT_CC3) != RESET)
    {
        TIM_Interrupt_Status_Clear(TIM3, TIM_INT_CC3);
        capture = TIM_Compare_Capture3_Get(TIM3);
        TIM_Compare3_Set(TIM3, capture + CCR3_Val);
    }

    if (TIM_Interrupt_Status_Get(TIM3, TIM_INT_CC4) != RESET)
    {
        TIM_Interrupt_Status_Clear(TIM3, TIM_INT_CC4);
        capture = TIM_Compare_Capture4_Get(TIM3);
        TIM_Compare4_Set(TIM3, capture + CCR4_Val);
    }
}

