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
static OCInitType TIM_OCInitStructure;
static uint16_t PrescalerValue = 0;
uint16_t gSendTrigEn = 0;
uint32_t GPTIMClockFrequency = 0;

void GPIO_Config(void);
void One_Trig_Send(void);

/**
*\*\brief  Main program
**/
int main(void)
{
    /* System Clocks Configuration */
    GPTIMClockFrequency = Common_GPTIM_RCC_Initialize(GPTIM, RCC_HCLK_DIV4);

    /* Configure the GPIO ports */
    GPIO_Config();

    /* GPTIM configuration: One Pulse mode 
     The external signal is connected to GPTIM_CH2 pin (PA.07),
     The Rising edge is used as active edge,
     The One Pulse signal is output on GPTIM_CH1 pin (PA.06)
     The Pulse defines the delay value
     The (Period -  Pulse) defines the One Pulse value.
     TIM3CLK = SystemCoreClock, we want to get TIM3 counter clock at 12.8 MHz:
     - Prescaler = (TIM3CLK / TIM3 counter clock) - 1
     The Autoreload value is 65535 (GPTIM->AR), so the maximum frequency value
     to trigger the GPTIM input is 12800000/65535 = 1953HZ

     The Pulse defines the delay value, the delay value is fixed
     to 639.96 us:
     delay =  CCDAT1/GPTIM counter clock = 1.28ms.
     The (Period - Pulse) defines the One Pulse value,
     the pulse value is fixed to 1.92 ms:
     One Pulse value = (Period - Pulse) / GPTIM counter clock = 3.84 ms.

                                                                 */

    PrescalerValue = (uint16_t)(GPTIMClockFrequency / 12800000) - 1;

    /* Time base configuration， period = 65535, prescaler = prescaler */
    Common_TIM_Base_Initialize(GPTIM, 65535, PrescalerValue);

    /* TIM Configuration in PWM Mode */
    TIM_OCInitStructure.OcMode      = TIM_OCMODE_PWM2;
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = 16383;
    TIM_OCInitStructure.OcPolarity  = TIM_OC_POLARITY_HIGH;
    TIM_Output_Channel1_Initialize(GPTIM, &TIM_OCInitStructure);
    
    TIM_ICInitStructure.Channel     = TIM_CH_2;
    TIM_ICInitStructure.IcPolarity  = TIM_IC_POLARITY_RISING;
    TIM_ICInitStructure.IcSelection = TIM_IC_SELECTION_DIRECTTI;
    TIM_ICInitStructure.IcPrescaler = TIM_IC_PSC_DIV1;
    TIM_ICInitStructure.IcFilter    = 0;

    TIM_Input_Channel_Initialize(GPTIM, &TIM_ICInitStructure);

    /* One Pulse Mode selection */
    TIM_One_Pulse_Mode_Select(GPTIM, TIM_OPMODE_SINGLE);

    /* Input Trigger selection */
    TIM_Trigger_Source_Select(GPTIM, TIM_TRIG_SEL_TI2FP2);

    TIM_Slave_Mode_Select(GPTIM, TIM_SLAVE_MODE_TRIG);

    while(1)
    {
        One_Trig_Send();
    }
}


/**
*\*\brief  Send one trig by self.
**/
void One_Trig_Send(void)
{
    if (gSendTrigEn)
    {
        gSendTrigEn = 0;
        GPIO_Pins_Set(GPIOA, GPIO_PIN_3);
        {
            uint32_t i=0;
            while (i++ < 10)
                ;
        }
        GPIO_PBC_Pins_Reset(GPIOA, GPIO_PIN_3);
    }
}

/**
*\*\brief  Configure the GPIOA Pins.
**/
void GPIO_Config(void)
{
    GPIO_InitType GPIO_InitStructure;

    GPIO_Structure_Initialize(&GPIO_InitStructure);
    /* TIM4_CH1 pin (PA.06) configuration as out*/
    GPIO_InitStructure.Pin        = TIM3_REMAP0_CH1_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Current = GPIO_DS_4MA;
    GPIO_InitStructure.GPIO_Alternate = TIM3_REMAP0_CH1_AF;
    GPIO_Peripheral_Initialize(TIM3_REMAP0_CH1_PORT, &GPIO_InitStructure);

    /* TIM4_CH2 pin (PA.07) configuration as trig*/
    GPIO_InitStructure.Pin       = TIM3_REMAP0_CH2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.GPIO_Alternate = TIM3_REMAP0_CH2_AF;
    GPIO_Peripheral_Initialize(TIM3_REMAP0_CH2_PORT, &GPIO_InitStructure);

    /* pin (PA.03) configuration as send trig*/
    GPIO_InitStructure.Pin       = GPIO_PIN_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUT_PP;
    GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);
}
