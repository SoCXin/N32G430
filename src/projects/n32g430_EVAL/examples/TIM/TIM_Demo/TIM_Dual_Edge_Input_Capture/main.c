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
#include "ADTIM_common.h"
#include "GPTIM_common.h"
#include "main.h"

static TIM_ICInitType TIM_ICInitStructure;

uint16_t gOnePulsEn = 0;

__IO uint16_t IC3ReadValueL = 0, IC3ReadValueH = 0;
__IO uint16_t CaptureNumber   = 0;
__IO uint32_t Capture         = 0, CaptureH         = 0, CaptureL         = 0;
__IO uint32_t TIM3Freq        = 0;
__IO uint32_t TIM3EnterIrqCnt = 0;
__IO uint32_t GPTIMClockFrequency = 0;


void GPIO_Config(TIM_Module *TIMx);

void Two_Rising_Edge_Generate(void);

/**
*\*brief  Main program
**/
int main(void)
{
    /* System Clocks Configuration */
    GPTIMClockFrequency = Common_GPTIM_RCC_Initialize(TIM3, RCC_HCLK_DIV4);

    /* NVIC configuration */
    Common_TIM_NVIC_Initialize(TIM3_IRQn, ENABLE);

    /* Configure the GPIO ports */
    GPIO_Config(TIM3);

    /* TIM3 configuration: Input Capture mode ---------------------
     The external signal is connected to TIM3 CH2 pin (PA.07) ,Rising edge
     The external signal is connected to TIM3 CH1 pin (PA.07) ,Falling edge
     The TIM3 CCDAT1+CCDAT2 is used to compute the frequency value

    gOnePulsEn = 1,will do Two_Rising_Edge_Generate
    then
    see Capture that is dist time
    ------------------------------------------------------------ */
    
    TIM_Input_Struct_Initialize(&TIM_ICInitStructure);
    TIM_ICInitStructure.Channel     = TIM_CH_2;
    TIM_ICInitStructure.IcPolarity  = TIM_IC_POLARITY_RISING;
    TIM_ICInitStructure.IcSelection = TIM_IC_SELECTION_DIRECTTI;
    TIM_ICInitStructure.IcPrescaler = TIM_IC_PSC_DIV1;
    TIM_ICInitStructure.IcFilter    = 0x0;
    TIM_Input_Channel_Initialize(TIM3, &TIM_ICInitStructure);
    
    TIM_Input_Struct_Initialize(&TIM_ICInitStructure);
    TIM_ICInitStructure.Channel     = TIM_CH_1;
    TIM_ICInitStructure.IcPolarity  = TIM_IC_POLARITY_FALLING;
    TIM_ICInitStructure.IcSelection = TIM_IC_SELECTION_INDIRECTTI;
    TIM_ICInitStructure.IcPrescaler = TIM_IC_PSC_DIV1;
    TIM_ICInitStructure.IcFilter    = 0x0;
    TIM_Input_Channel_Initialize(TIM3, &TIM_ICInitStructure);
    /* TIM enable counter */
    TIM_On(TIM3);

    /* Enable the CC1 and CC2 Interrupt Request */
    TIM_Interrupt_Enable(TIM3, TIM_INT_CC1 | TIM_INT_CC2);

    while (1)
    {
        Two_Rising_Edge_Generate();
    }
}

// generation two rising edge
void Two_Rising_Edge_Generate(void)
{
    if (gOnePulsEn)
    {
        gOnePulsEn = 0;
        GPIO_Pins_Set(GPIOA, GPIO_PIN_3);
        {
            uint32_t i = 0;
            while (i++ < 500)
                ;
        }
        GPIO_PBC_Pins_Reset(GPIOA, GPIO_PIN_3);
        {
            uint32_t i = 0;
            while (i++ < 500)
                ;
        }

        GPIO_Pins_Set(GPIOA, GPIO_PIN_3);
        {
            uint32_t i = 0;
            while (i++ < 500)
                ;
        }
        GPIO_PBC_Pins_Reset(GPIOA, GPIO_PIN_3);
        {
            uint32_t i = 0;
            while (i++ < 500)
                ;
        }
    }
}

/**
*\*brief  Configure the GPIOA Pins.
**/
void GPIO_Config(TIM_Module *TIMx)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_Structure_Initialize(&GPIO_InitStructure);
    
    if(TIM3 == TIMx)
    {
        /* TIM3 channel 2 pin (PA.07) configuration */
        GPIO_InitStructure.Pin            = TIM3_REMAP0_CH2_PIN;
        GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_INPUT;
        GPIO_InitStructure.GPIO_Current   = GPIO_DS_4MA;
        GPIO_InitStructure.GPIO_Alternate = TIM3_REMAP0_CH2_AF;
        GPIO_Peripheral_Initialize(TIM3_REMAP0_CH2_PORT, &GPIO_InitStructure);
    }
    else
    {
        /* none */
    }

    /* PA3 */
    GPIO_InitStructure.Pin       = GPIO_PIN_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUT_PP;
    GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);
}

/**
*\*\brief  This function handles TIM3 global interrupt request.
**/
void TIM3_IRQHandler(void)
{
    uint16_t IC3ReadValueH_t = 0;
    uint32_t CaptureH_t = 0;
    /* TIM3 CH2 Rising Edge */
    if (TIM_Interrupt_Status_Get(TIM3, TIM_INT_CC2) == SET)
    {
        TIM3EnterIrqCnt++;
        /* Clear TIM3 Capture compare interrupt pending bit */
        TIM_Interrupt_Status_Clear(TIM3, TIM_INT_CC2);
        if (CaptureNumber == 0)
        {
            /* Get the Input Capture value */
            IC3ReadValueL = TIM_Compare_Capture2_Get(TIM3);
            CaptureNumber = 1;
        }
        else if (CaptureNumber == 1)
        {
            /* Get the Input Capture value */
            IC3ReadValueL = TIM_Compare_Capture2_Get(TIM3);
            /*For remove IAR Warning[Pa082]*/
            IC3ReadValueH_t = IC3ReadValueH;
            /* Capture computation */
            if (IC3ReadValueL > IC3ReadValueH_t)
            {
                CaptureL = (IC3ReadValueL - IC3ReadValueH_t);
            }
            else
            {
                CaptureL = ((0xFFFF - IC3ReadValueH_t) + IC3ReadValueL);
            }
            /*For remove IAR Warning[Pa082]*/
            CaptureH_t = CaptureH;
            /* Period computation */
            Capture = CaptureH_t + CaptureL;
            /* Frequency computation */
            TIM3Freq      = (uint32_t)GPTIMClockFrequency / Capture;
        }
    }
    else if (TIM_Interrupt_Status_Get(TIM3, TIM_INT_CC1) == SET)
    {
        TIM3EnterIrqCnt++;
        /* Clear TIM3 Capture compare interrupt pending bit */
        TIM_Interrupt_Status_Clear(TIM3, TIM_INT_CC1);
        if (CaptureNumber == 0)
        {
            /* Get the Input Capture value */
            IC3ReadValueH = TIM_Compare_Capture1_Get(TIM3);
            CaptureNumber = 1;
        }
        else if (CaptureNumber == 1)
        {
            /* Get the Input Capture value */
            IC3ReadValueH = TIM_Compare_Capture1_Get(TIM3);
            /*For remove IAR Warning[Pa082]*/
            IC3ReadValueH_t = IC3ReadValueH;
            /* Capture computation */
            if (IC3ReadValueH_t > IC3ReadValueL)
            {
                CaptureH = (IC3ReadValueH_t - IC3ReadValueL);
            }
            else
            {
                CaptureH = ((0xFFFF - IC3ReadValueL) + IC3ReadValueH_t);
            }
        }
    }
}
/**
 * @}
 */

/**
 * @}
 */
