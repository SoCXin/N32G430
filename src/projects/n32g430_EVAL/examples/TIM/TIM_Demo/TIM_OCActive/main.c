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
#include "GPTIM_common.h"
#include "ADTIM_common.h"
#include "timer_common.h"
#include "main.h"

uint16_t CCR1_Val       = 1000;
uint16_t CCR2_Val       = 500;
uint16_t CCR3_Val       = 250;
uint16_t CCR4_Val       = 125;
uint16_t PrescalerValue = 0;
uint32_t GPTIMClockFrequency = 0;

OCInitType TIM_OCInitStructure;

int main(void)
{


    /* System Clocks Configuration */
    GPTIMClockFrequency = Common_GPTIM_RCC_Initialize(GPTIM, RCC_HCLK_DIV4);

    /* Configure the GPIO ports */
    Common_GPTIM_GPIO_Initialize(GPTIM);

    /*
    Generate 4 signals with 4 different delays:
    TIM3_CH1 delay = CCR1_Val/TIM3 counter clock = 500 ms
    TIM3_CH2 delay = CCR2_Val/TIM3 counter clock = 250 ms
    TIM3_CH3 delay = CCR3_Val/TIM3 counter clock = 125 ms
    TIM3_CH4 delay = CCR4_Val/TIM3 counter clock = 62.5 ms
    */
    PrescalerValue = (uint16_t)(GPTIMClockFrequency / 2000) - 1;
    /* TIM Base init , Period = 65535, Prescaler = PrescalerValue */
    Common_TIM_Base_Initialize(GPTIM, 65535, PrescalerValue);

    TIM_Output_Channel_Struct_Initialize(&TIM_OCInitStructure);

    TIM_OCInitStructure.OcMode      = TIM_OCMODE_ACTIVE;
    TIM_OCInitStructure.OcPolarity  = TIM_OC_POLARITY_HIGH;
    
    /* Output Compare Active Mode configuration: Channel1 */
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = CCR1_Val;
    TIM_Output_Channel1_Initialize(GPTIM, &TIM_OCInitStructure);

    TIM_Output_Channel1_Preload_Set(GPTIM, TIM_OC_PRELOAD_DISABLE);

    /* Output Compare Active Mode configuration: Channel2 */
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = CCR2_Val;
    TIM_Output_Channel2_Initialize(GPTIM, &TIM_OCInitStructure);

    TIM_Output_Channel2_Preload_Set(GPTIM, TIM_OC_PRELOAD_DISABLE);

    /* Output Compare Active Mode configuration: Channel3 */
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = CCR3_Val;
    TIM_Output_Channel3_Initialize(GPTIM, &TIM_OCInitStructure);

    TIM_Output_Channel3_Preload_Set(GPTIM, TIM_OC_PRELOAD_DISABLE);

    /* Output Compare Active Mode configuration: Channel4 */
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = CCR4_Val;
    TIM_Output_Channel4_Initialize(GPTIM, &TIM_OCInitStructure);

    TIM_Output_Channel4_Preload_Set(GPTIM, TIM_OC_PRELOAD_DISABLE);

    TIM_Auto_Reload_Preload_Enable(GPTIM);

    TIM_On(GPTIM);

    while(1)
    {

    }
}
