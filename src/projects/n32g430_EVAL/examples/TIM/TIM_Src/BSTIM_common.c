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
*\*\file BSTIM_common.c
*\*\author Nations 
*\*\version v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
**/
#include "BSTIM_common.h"

/**
*\*\name    Common_BSTIM_RCC_Initialize.
*\*\param   TIMx :
*\*\          - TIM6
*\*\param   hclk_division
*\*\          - RCC_HCLK_DIV1
*\*\          - RCC_HCLK_DIV2
*\*\          - RCC_HCLK_DIV4
*\*\          - RCC_HCLK_DIV8
*\*\          - RCC_HCLK_DIV16
*\*\return  uint32_t
**/
uint32_t Common_BSTIM_RCC_Initialize(TIM_Module *TIMx, uint32_t hclk_division)
{
    uint32_t BSTIM_clock;

    RCC_ClocksType RCC_Clocks;

    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOA | RCC_AHB_PERIPH_GPIOB| RCC_AHB_PERIPH_GPIOC 
                                       | RCC_AHB_PERIPH_GPIOD);
    
    RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_AFIO);
    
    RCC_Pclk1_Config(hclk_division);

    RCC_APB1_Peripheral_Clock_Enable(RCC_APB1_PERIPH_TIM6);

    RCC_Clocks_Frequencies_Value_Get(&RCC_Clocks);

    if(RCC_Clocks.HclkFreq > RCC_Clocks.Pclk1Freq) 
    {
        BSTIM_clock = RCC_Clocks.Pclk1Freq * 2; 
    }
    else
    {
        BSTIM_clock = RCC_Clocks.Pclk1Freq;
    }
    return BSTIM_clock;
}
