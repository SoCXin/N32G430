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

__IO uint16_t IC2Value  = 0;
__IO uint16_t DutyCycle = 0;
__IO uint32_t Frequency = 0;

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

    /* TIM3 configuration: PWM Input mode
     The external signal is connected to TIM3 CH2 pin (PA.07),
     The Rising edge is used as active edge,
     The TIM3 CCDAT2 is used to compute the frequency value
     The TIM3 CCDAT1 is used to compute the duty cycle value
                                                                 */
    
    TIM_ICInitStructure.Channel     = TIM_CH_2;
    TIM_ICInitStructure.IcPolarity  = TIM_IC_POLARITY_RISING;
    TIM_ICInitStructure.IcSelection = TIM_IC_SELECTION_DIRECTTI;
    TIM_ICInitStructure.IcPrescaler = TIM_IC_PSC_DIV1;
    TIM_ICInitStructure.IcFilter    = 0x0;
    TIM_PWM_Input_Channel_Config(TIM3, &TIM_ICInitStructure);
    
    TIM_Trigger_Source_Select(TIM3, TIM_TRIG_SEL_TI2FP2);

    TIM_Slave_Mode_Select(TIM3, TIM_SLAVE_MODE_RESET);

    TIM_Master_Slave_Mode_Set(TIM3, TIM_MASTER_SLAVE_MODE_ENABLE);
    /* TIM enable counter */
    TIM_On(TIM3);

    /* Enable the CC1 and CC2 Interrupt Request */
    TIM_Interrupt_Enable(TIM3, TIM_INT_CC2);

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
            while (i++ < 50)
                ;
        }

        GPIO_PBC_Pins_Reset(GPIOA, GPIO_PIN_3);
        {
            uint32_t i = 0;
            while (i++ < 50)
                ;
        }

        GPIO_Pins_Set(GPIOA, GPIO_PIN_3);
        {
            uint32_t i = 0;
            while (i++ < 50)
                ;
        }

        GPIO_PBC_Pins_Reset(GPIOA, GPIO_PIN_3);
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
    /* Clear TIM3 Capture compare interrupt pending bit */
    TIM_Interrupt_Status_Clear(TIM3, TIM_INT_CC2);

    /* Get the Input Capture value */
    IC2Value = TIM_Compare_Capture2_Get(TIM3);

    if (IC2Value != 0)
    {
        /* Duty cycle computation */
        DutyCycle = (TIM_Compare_Capture1_Get(TIM3) * 100) / IC2Value;

        /* Frequency computation */
        Frequency = GPTIMClockFrequency / IC2Value;
    }
    else
    {
        DutyCycle = 0;
        Frequency = 0;
    }
}
/**
 * @}
 */

/**
 * @}
 */
