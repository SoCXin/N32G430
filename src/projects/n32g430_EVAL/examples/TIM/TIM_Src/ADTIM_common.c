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
*\*\file ADTIM_common.c
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
**/
#include "ADTIM_common.h"

/**
*\*\name    Common_ADTIM_RCC_Initialize.
*\*\param   TIMx :
*\*\          - TIM1
*\*\          - TIM8
*\*\param   hclk_division
*\*\          - RCC_HCLK_DIV1
*\*\          - RCC_HCLK_DIV2
*\*\          - RCC_HCLK_DIV4
*\*\          - RCC_HCLK_DIV8
*\*\          - RCC_HCLK_DIV16
*\*\return  uint32_t
**/
uint32_t Common_ADTIM_RCC_Initialize(TIM_Module *TIMx, uint32_t hclk_division)
{
    uint32_t ADTIM_clock;

    RCC_ClocksType RCC_Clocks;

    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOA | RCC_AHB_PERIPH_GPIOB |
                                    RCC_AHB_PERIPH_GPIOC | RCC_AHB_PERIPH_GPIOD);

    RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_AFIO);
    
    RCC_Pclk2_Config(hclk_division);

    if(TIM1 == TIMx)
    {
        RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_TIM1);
    
    }
    else if(TIM8 == TIMx)
    {
        RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_TIM8);
    }
    else
    {
        /* none */
    }

    RCC_Clocks_Frequencies_Value_Get(&RCC_Clocks);

    
    if(RCC_Clocks.HclkFreq > RCC_Clocks.Pclk2Freq) 
    {
        ADTIM_clock = RCC_Clocks.Pclk2Freq * 2; 
    }
    else
    {
        ADTIM_clock = RCC_Clocks.Pclk2Freq;
    }
    return ADTIM_clock;
}

/**
*\*\name    Common_ADTIM_Break_IO_Initialize.
*\*\param   TIMx :
*\*\          - TIM1
*\*\          - TIM8
*\*\return  none
**/
void Common_ADTIM_Break_IO_Initialize(TIM_Module* TIMx)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_Structure_Initialize(&GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_INPUT;
    GPIO_InitStructure.GPIO_Current = GPIO_DS_4MA;
    if(TIM1 == TIMx)
    {
        GPIO_InitStructure.Pin       = TIM1_REMAP0_BKIN_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM1_REMAP0_BKIN_AF;
        GPIO_Peripheral_Initialize(TIM1_REMAP0_BKIN_PORT, &GPIO_InitStructure);
    }
    else if(TIM8 == TIMx)
    {
        GPIO_InitStructure.Pin       = TIM8_REMAP0_BKIN_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM8_REMAP0_BKIN_AF;
        GPIO_Peripheral_Initialize(TIM8_REMAP0_BKIN_PORT, &GPIO_InitStructure);
    }
    else
    {
        /* none */
    }
}

/**
*\*\name    Common_ADTIM_GPIO_Initialize.
*\*\param   TIMx :
*\*\          - TIM1
*\*\          - TIM8
*\*\return  none
**/
void Common_ADTIM_GPIO_Initialize(TIM_Module* TIMx)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_Structure_Initialize(&GPIO_InitStructure);

    if(TIM1 == TIMx)
    {
        GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStructure.GPIO_Current = GPIO_DS_4MA;

        /* TIM1 CHx pins init */
        GPIO_InitStructure.Pin        = TIM1_REMAP0_CH1_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM1_REMAP0_CH1_AF;
        GPIO_Peripheral_Initialize(TIM1_REMAP0_CH1_PORT, &GPIO_InitStructure);

        GPIO_InitStructure.Pin        = TIM1_REMAP0_CH2_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM1_REMAP0_CH2_AF;
        GPIO_Peripheral_Initialize(TIM1_REMAP0_CH2_PORT, &GPIO_InitStructure);

        GPIO_InitStructure.Pin        = TIM1_REMAP0_CH3_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM1_REMAP0_CH3_AF;
        GPIO_Peripheral_Initialize(TIM1_REMAP0_CH3_PORT, &GPIO_InitStructure);

        GPIO_InitStructure.Pin        = TIM1_REMAP0_CH4_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM1_REMAP0_CH4_AF;
        GPIO_Peripheral_Initialize(TIM1_REMAP0_CH4_PORT, &GPIO_InitStructure);

        /* TIM1 CHxN pins ini. CH4N is only for TIM1 */
        GPIO_InitStructure.Pin        = TIM1_REMAP0_CH1N_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM1_REMAP0_CH1N_AF;
        GPIO_Peripheral_Initialize(TIM1_REMAP0_CH1N_PORT, &GPIO_InitStructure);

        GPIO_InitStructure.Pin        = TIM1_REMAP0_CH2N_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM1_REMAP0_CH2N_AF;
        GPIO_Peripheral_Initialize(TIM1_REMAP0_CH2N_PORT, &GPIO_InitStructure);

        GPIO_InitStructure.Pin        = TIM1_REMAP0_CH3N_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM1_REMAP0_CH3N_AF;
        GPIO_Peripheral_Initialize(TIM1_REMAP0_CH3N_PORT, &GPIO_InitStructure);

        GPIO_InitStructure.Pin        = TIM1_REMAP0_CH4N_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM1_REMAP0_CH4N_AF;
        GPIO_Peripheral_Initialize(TIM1_REMAP0_CH4N_PORT, &GPIO_InitStructure);
    }
    else if(TIM8 == TIMx)
    {
        GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStructure.GPIO_Current = GPIO_DS_4MA;

        /* TIM1 CHx pins init */
        GPIO_InitStructure.Pin        = TIM8_REMAP0_CH1_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM8_REMAP0_CH1_AF;
        GPIO_Peripheral_Initialize(TIM8_REMAP0_CH1_PORT, &GPIO_InitStructure);

        GPIO_InitStructure.Pin        = TIM8_REMAP0_CH2_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM8_REMAP0_CH2_AF;
        GPIO_Peripheral_Initialize(TIM8_REMAP0_CH2_PORT, &GPIO_InitStructure);

        GPIO_InitStructure.Pin        = TIM8_REMAP0_CH3_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM8_REMAP0_CH3_AF;
        GPIO_Peripheral_Initialize(TIM8_REMAP0_CH3_PORT, &GPIO_InitStructure);

        GPIO_InitStructure.Pin        = TIM8_REMAP0_CH4_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM8_REMAP0_CH4_AF;
        GPIO_Peripheral_Initialize(TIM8_REMAP0_CH4_PORT, &GPIO_InitStructure);

        /* TIM8 CHxN pins ini. CH4N is not for TIM8 */
        GPIO_InitStructure.Pin        = TIM8_REMAP0_CH1N_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM8_REMAP0_CH1N_AF;
        GPIO_Peripheral_Initialize(TIM8_REMAP0_CH1N_PORT, &GPIO_InitStructure);

        GPIO_InitStructure.Pin        = TIM8_REMAP0_CH2N_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM8_REMAP0_CH2N_AF;
        GPIO_Peripheral_Initialize(TIM8_REMAP0_CH2N_PORT, &GPIO_InitStructure);

        GPIO_InitStructure.Pin        = TIM8_REMAP0_CH3N_PIN;
        GPIO_InitStructure.GPIO_Alternate = TIM8_REMAP0_CH3N_AF;
        GPIO_Peripheral_Initialize(TIM8_REMAP0_CH3N_PORT, &GPIO_InitStructure);
    }
}
