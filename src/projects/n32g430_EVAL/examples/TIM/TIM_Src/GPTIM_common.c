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
*\*\file GPTIM_common.c
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
**/
#include "GPTIM_common.h"

/**
*\*\name    Common_GPTIM_RCC_Initialize.
*\*\param   TIMx :
*\*\          - TIM2
*\*\          - TIM3
*\*\          - TIM4
*\*\          - TIM5
*\*\param   hclk_division
*\*\          - RCC_HCLK_DIV1
*\*\          - RCC_HCLK_DIV2
*\*\          - RCC_HCLK_DIV4
*\*\          - RCC_HCLK_DIV8
*\*\          - RCC_HCLK_DIV16
*\*\return  uint32_t
**/
uint32_t Common_GPTIM_RCC_Initialize(TIM_Module *TIMx, uint32_t hclk_division)
{
    uint32_t GPTIM_clock;

    RCC_ClocksType RCC_Clocks;

    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOA | RCC_AHB_PERIPH_GPIOB| RCC_AHB_PERIPH_GPIOC 
                                       | RCC_AHB_PERIPH_GPIOD);
    
    RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_AFIO);
    
    RCC_Pclk1_Config(hclk_division);

    if(TIM2 == TIMx)
    {
        RCC_APB1_Peripheral_Clock_Enable(RCC_APB1_PERIPH_TIM2);
    }
    else if(TIM3 == TIMx)
    {
        RCC_APB1_Peripheral_Clock_Enable(RCC_APB1_PERIPH_TIM3);
    }
    else if(TIM4 == TIMx)
    {
        RCC_APB1_Peripheral_Clock_Enable(RCC_APB1_PERIPH_TIM4);
    }
    else if(TIM5 == TIMx)
    {
        RCC_APB1_Peripheral_Clock_Enable(RCC_APB1_PERIPH_TIM5);
    }
    else
    {

    }

    RCC_Clocks_Frequencies_Value_Get(&RCC_Clocks);

    if(RCC_Clocks.HclkFreq > RCC_Clocks.Pclk1Freq) 
    {
        GPTIM_clock = RCC_Clocks.Pclk1Freq * 2; 
    }
    else
    {
        GPTIM_clock = RCC_Clocks.Pclk1Freq;
    }
    return GPTIM_clock;
}

/**
*\*\name    Common_GPTIM_RCC_Initialize.
*\*\param   TIMx :
*\*\          - TIM2
*\*\          - TIM3
*\*\          - TIM4
*\*\          - TIM5
*\*\return  none
*\*\note The pins of remap0 of TIM2 and TIM5 conflict with each other
**/
void Common_GPTIM_GPIO_Initialize(TIM_Module *TIMx)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_Structure_Initialize(&GPIO_InitStructure);

    if(TIM2 == TIMx)
    {
        GPIO_InitStructure.Pin        = TIM2_REMAP0_CH1_PIN;
        GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStructure.GPIO_Current = GPIO_DS_4MA;
        GPIO_InitStructure.GPIO_Alternate = TIM2_REMAP0_CH1_AF;
        GPIO_Peripheral_Initialize(TIM2_REMAP0_CH1_PORT, &GPIO_InitStructure);

        GPIO_InitStructure.Pin        = TIM2_REMAP0_CH2_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM2_REMAP0_CH2_AF;
        GPIO_Peripheral_Initialize(TIM2_REMAP0_CH2_PORT, &GPIO_InitStructure);

        GPIO_InitStructure.Pin        = TIM2_REMAP0_CH3_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM2_REMAP0_CH3_AF;
        GPIO_Peripheral_Initialize(TIM2_REMAP0_CH3_PORT, &GPIO_InitStructure);

        GPIO_InitStructure.Pin        = TIM2_REMAP0_CH4_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM2_REMAP0_CH4_AF;
        GPIO_Peripheral_Initialize(TIM2_REMAP0_CH4_PORT, &GPIO_InitStructure);
    }
    else if(TIM3 == TIMx)
    {
        GPIO_InitStructure.Pin        = TIM3_REMAP0_CH1_PIN;
        GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStructure.GPIO_Current = GPIO_DS_4MA;
        GPIO_InitStructure.GPIO_Alternate = TIM3_REMAP0_CH1_AF;
        GPIO_Peripheral_Initialize(TIM3_REMAP0_CH1_PORT, &GPIO_InitStructure);

        GPIO_InitStructure.Pin        = TIM3_REMAP0_CH2_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM3_REMAP0_CH2_AF;
        GPIO_Peripheral_Initialize(TIM3_REMAP0_CH2_PORT, &GPIO_InitStructure);

        GPIO_InitStructure.Pin        = TIM3_REMAP0_CH3_PIN;

        GPIO_InitStructure.GPIO_Alternate = TIM3_REMAP0_CH3_AF;
        GPIO_Peripheral_Initialize(TIM3_REMAP0_CH3_PORT, &GPIO_InitStructure);

        GPIO_InitStructure.Pin        = TIM3_REMAP0_CH4_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM3_REMAP0_CH4_AF;
        GPIO_Peripheral_Initialize(TIM3_REMAP0_CH4_PORT, &GPIO_InitStructure);
    }
    else if(TIM4 == TIMx)
    {
        GPIO_InitStructure.Pin        = TIM4_REMAP0_CH1_PIN;
        GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStructure.GPIO_Current = GPIO_DS_4MA;
        GPIO_InitStructure.GPIO_Alternate = TIM4_REMAP0_CH1_AF;
        GPIO_Peripheral_Initialize(TIM4_REMAP0_CH1_PORT, &GPIO_InitStructure);

        GPIO_InitStructure.Pin        = TIM4_REMAP0_CH2_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM4_REMAP0_CH2_AF;
        GPIO_Peripheral_Initialize(TIM4_REMAP0_CH2_PORT, &GPIO_InitStructure);

        GPIO_InitStructure.Pin        = TIM4_REMAP0_CH3_PIN;

        GPIO_InitStructure.GPIO_Alternate = TIM4_REMAP0_CH3_AF;
        GPIO_Peripheral_Initialize(TIM4_REMAP0_CH3_PORT, &GPIO_InitStructure);

        GPIO_InitStructure.Pin        = TIM4_REMAP0_CH4_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM4_REMAP0_CH4_AF;
        GPIO_Peripheral_Initialize(TIM4_REMAP0_CH4_PORT, &GPIO_InitStructure);
    }
    else if(TIM5 == TIMx)
    {
        GPIO_InitStructure.Pin        = TIM5_REMAP0_CH1_PIN;
        GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStructure.GPIO_Current = GPIO_DS_4MA;
        GPIO_InitStructure.GPIO_Alternate = TIM5_REMAP0_CH1_AF;
        GPIO_Peripheral_Initialize(TIM5_REMAP0_CH1_PORT, &GPIO_InitStructure);

        GPIO_InitStructure.Pin        = TIM5_REMAP0_CH2_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM5_REMAP0_CH2_AF;
        GPIO_Peripheral_Initialize(TIM5_REMAP0_CH2_PORT, &GPIO_InitStructure);

        GPIO_InitStructure.Pin        = TIM5_REMAP0_CH3_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM5_REMAP0_CH3_AF;
        GPIO_Peripheral_Initialize(TIM5_REMAP0_CH3_PORT, &GPIO_InitStructure);

        GPIO_InitStructure.Pin        = TIM5_REMAP0_CH4_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM5_REMAP0_CH4_AF;
        GPIO_Peripheral_Initialize(TIM5_REMAP0_CH4_PORT, &GPIO_InitStructure);
    }
    else
    {
        /* none */
    }
}

