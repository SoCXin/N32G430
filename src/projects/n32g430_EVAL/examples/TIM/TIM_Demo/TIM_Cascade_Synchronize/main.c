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

static OCInitType TIM_OCInitStructure;


int main(void)
{
    Common_GPTIM_RCC_Initialize(TIM2, RCC_HCLK_DIV4);

    Common_GPTIM_RCC_Initialize(TIM3, RCC_HCLK_DIV4);
    
    Common_GPTIM_RCC_Initialize(TIM4, RCC_HCLK_DIV4);

    Common_GPTIM_GPIO_Initialize(TIM2);

    Common_GPTIM_GPIO_Initialize(TIM3);

    Common_GPTIM_GPIO_Initialize(TIM4);

    /**  Timers synchronisation in cascade mode 
     1/TIM2 is configured as Master Timer:
     - PWM Mode is used
     - The TIM2 Update event is used as Trigger Output

     2/TIM3 is slave for TIM2 and Master for TIM4,
     - PWM Mode is used
     - The ITR1(TIM2) is used as input trigger
     - Gated mode is used, so start and stop of slave counter
       are controlled by the Master trigger output signal(TIM2 update event).
     - The TIM3 Update event is used as Trigger Output.

      3/TIM4 is slave for TIM3,
     - PWM Mode is used
     - The ITR2(TIM3) is used as input trigger
     - Gated mode is used, so start and stop of slave counter
       are controlled by the Master trigger output signal(TIM3 update event).

       The TIMxCLK is fixed to 64 MHz, the TIM2 counter clock is 64 MHz.

       The Master Timer TIM2 is running at TIM2 frequency :
       TIM2 frequency = (TIM2 counter clock)/ (TIM2 period + 1) = 250 KHz
       and the duty cycle = TIM2_CCR1/(TIM2_ARR + 1) = 25%.

       The TIM3 is running:
       - At (TIM2 frequency)/ (TIM3 period + 1) = 62.5 KHz and a duty cycle
         equal to TIM3_CCR1/(TIM3_ARR + 1) = 25%

        The TIM4 is running:
      - At (TIM3 frequency)/ (TIM4 period + 1) = 15.625 KHz and a duty cycle
        equal to TIM4_CCR1/(TIM4_ARR + 1) = 25%  **/

    /* Timer 2 base configuration, Period = 255, Prescaler = 0 */
    Common_TIM_Base_Initialize(TIM2, 255, 0);
    
    /* Timer 3 base configuration, Period = 3, Prescaler = 0 */
    Common_TIM_Base_Initialize(TIM3, 3, 0);

    /* Timer 4 base configuration, Period = 3, Prescaler = 0 */
    Common_TIM_Base_Initialize(TIM4, 3, 0);

    /* Master Configuration in PWM1 Mode */
    TIM_Output_Channel_Struct_Initialize(&TIM_OCInitStructure);
    TIM_OCInitStructure.OcMode      = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = 64;
    TIM_OCInitStructure.OcPolarity  = TIM_OC_POLARITY_HIGH;
    TIM_Output_Channel1_Initialize(TIM2, &TIM_OCInitStructure);

    /* Select the Master Slave Mode */
    TIM_Master_Slave_Mode_Set(TIM2, TIM_MASTER_SLAVE_MODE_ENABLE);

    /* Master Mode selection: TIM2 */
    TIM_Output_Trigger_Select(TIM2, TIM_TRGO_SRC_UPDATE);

    /* Slaves Configuration: Toggle Mode */
    TIM_OCInitStructure.OcMode      = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = 1;
    TIM_Output_Channel1_Initialize(TIM3, &TIM_OCInitStructure);

    TIM_Output_Channel1_Initialize(TIM4, &TIM_OCInitStructure);

    /* Slave Mode selection: TIM3 */
    TIM_Slave_Mode_Select(TIM3, TIM_SLAVE_MODE_GATED);
    TIM_Trigger_Source_Select(TIM3, TIM_TRIG_SEL_IN_TR1);

    /* Select the Master Slave Mode */
    TIM_Master_Slave_Mode_Set(TIM3, TIM_MASTER_SLAVE_MODE_ENABLE);

    /* Master Mode selection: TIM3 */
    TIM_Output_Trigger_Select(TIM3, TIM_TRGO_SRC_UPDATE);

    /* Slave Mode selection: TIM3 */
    TIM_Slave_Mode_Select(TIM4, TIM_SLAVE_MODE_GATED);
    TIM_Trigger_Source_Select(TIM4, TIM_TRIG_SEL_IN_TR2);

    /* TIM enable counter */
    TIM_On(TIM2);
    TIM_On(TIM3);
    TIM_On(TIM4);

    while (1)
    {
    }
}
