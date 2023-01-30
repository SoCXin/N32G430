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

static OCInitType TIM_OCInitStructure;

static TIM_TimeBaseInitType TIM_TimeBaseStructure;

static TIM_BDTRInitType TIM_BDTRInitStructure;

int main(void)
{
    Common_ADTIM_RCC_Initialize(ADTIM, RCC_HCLK_DIV4);

    Common_GPTIM_RCC_Initialize(TIM3, RCC_HCLK_DIV4);

    Common_GPTIM_RCC_Initialize(TIM4, RCC_HCLK_DIV4);

    Common_ADTIM_GPIO_Initialize(TIM1);

    Common_GPTIM_GPIO_Initialize(TIM3);
    
    Common_GPTIM_GPIO_Initialize(TIM4);

    /* TIM1 and Timers(TIM3 and TIM4) synchronisation in parallel mode 
     1/TIM1 is configured as Master Timer:
     - PWM Mode is used
     - The TIM1 Update event is used as Trigger Output

     2/TIM3 and TIM4 are slaves for TIM1,
     - PWM Mode is used
     - The ITR0(TIM1) is used as input trigger for both slaves
     - Gated mode is used, so starts and stops of slaves counters
       are controlled by the Master trigger output signal(update event).

    The Master Timer TIM1 is running at:
    TIM1 frequency = TIM1 counter clock / (TIM1_Period + 1) = 250 KHz
    and the duty cycle is equal to: TIM1_CCR1/(TIM1_ARR + 1) = 50%

    The TIM3 is running at:
    (TIM1 frequency)/ ((TIM3 period +1)* (Repetition_Counter+1)) = 16.667 KHz and
    a duty cycle equal to TIM3_CCR1/(TIM3_ARR + 1) = 33.3%

    The TIM4 is running at:
    (TIM1 frequency)/ ((TIM4 period +1)* (Repetition_Counter+1)) = 25 KHz and
    a duty cycle equal to TIM4_CCR1/(TIM4_ARR + 1) = 50%
                                                                             */

    /* Time base configuration */
    TIM_TimeBaseStructure.Prescaler = 0;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;
    TIM_TimeBaseStructure.Period    = 255;
    TIM_TimeBaseStructure.ClkDiv    = 0;
    TIM_TimeBaseStructure.RepetCnt  = 4;

    TIM_Base_Initialize(TIM1, &TIM_TimeBaseStructure);

    Common_TIM_Base_Initialize(TIM3, 2, 0);

    Common_TIM_Base_Initialize(TIM4, 1, 0);

    /* Channel 1 Configuration in PWM mode */
    TIM_Output_Channel_Struct_Initialize(&TIM_OCInitStructure);
    TIM_OCInitStructure.OcMode       = TIM_OCMODE_PWM2;
    TIM_OCInitStructure.OutputState  = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.OutputNState = TIM_OUTPUT_NSTATE_ENABLE;
    TIM_OCInitStructure.Pulse        = 127;
    TIM_OCInitStructure.OcPolarity   = TIM_OC_POLARITY_LOW;
    TIM_OCInitStructure.OcNPolarity  = TIM_OCN_POLARITY_LOW;
    TIM_OCInitStructure.OcIdleState  = TIM_OC_IDLE_STATE_SET;
    TIM_OCInitStructure.OcNIdleState = TIM_OC_IDLE_STATE_RESET;
    TIM_Output_Channel1_Initialize(ADTIM, &TIM_OCInitStructure);

    TIM_Output_Channel_Struct_Initialize(&TIM_OCInitStructure);
    TIM_OCInitStructure.OcMode      = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = 1;
    TIM_Output_Channel1_Initialize(TIM3, &TIM_OCInitStructure);

    TIM_Output_Channel_Struct_Initialize(&TIM_OCInitStructure);
    TIM_OCInitStructure.OcMode      = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = 1;
    TIM_Output_Channel1_Initialize(TIM4, &TIM_OCInitStructure);


    /* Select the Master Slave Mode */
    TIM_Master_Slave_Mode_Set(ADTIM, TIM_MASTER_SLAVE_MODE_ENABLE);
    /* Master Mode selection: ADTIM */
    TIM_Output_Trigger_Select(ADTIM, TIM_TRGO_SRC_UPDATE);


    /* Slave Mode selection: TIM3 */
    TIM_Slave_Mode_Select(TIM3, TIM_SLAVE_MODE_GATED);
    TIM_Trigger_Source_Select(TIM3, TIM_TRIG_SEL_IN_TR0);

    /* Slave Mode selection: TIM4 */
    TIM_Slave_Mode_Select(TIM4, TIM_SLAVE_MODE_GATED);
    TIM_Trigger_Source_Select(TIM4, TIM_TRIG_SEL_IN_TR0);

    /* Automatic Output enable, Break, dead time and lock configuration*/
    TIM_Break_And_Dead_Time_Struct_Initialize(&TIM_BDTRInitStructure);
    TIM_BDTRInitStructure.OssrState       = TIM_OSSR_STATE_ENABLE;
    TIM_BDTRInitStructure.OssiState       = TIM_OSSI_STATE_ENABLE;
    TIM_BDTRInitStructure.LockLevel       = TIM_LOCK_LEVEL_1;
    TIM_BDTRInitStructure.DeadTime        = 5;
    TIM_BDTRInitStructure.Break           = TIM_BREAK_IN_DISABLE;
    TIM_BDTRInitStructure.BreakPolarity   = TIM_BREAK_POLARITY_HIGH;
    TIM_BDTRInitStructure.AutomaticOutput = TIM_AUTO_OUTPUT_DISABLE;

    TIM_Break_And_Dead_Time_Set(ADTIM, &TIM_BDTRInitStructure);

    /* ADTIM Main Output Enable */
    TIM_PWM_Output_Enable(ADTIM);
    /* TIM enable counter */
    TIM_On(ADTIM);
    TIM_On(TIM3);
    TIM_On(TIM4);

    while (1)
    {
    }
}
