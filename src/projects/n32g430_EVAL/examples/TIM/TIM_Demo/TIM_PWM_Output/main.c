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
#include "timer_common.h"
#include "main.h"

static OCInitType TIM_OCInitStructure;

static TIM_BDTRInitType TIM_BDTRInitStructure;

uint16_t TimerPeriod   = 0;
uint16_t Channel1Pulse = 0, Channel2Pulse = 0, Channel3Pulse = 0, channel4Pulse = 0;

uint32_t ADTIMClockFrequency = 0;;

int main(void)
{
    ADTIMClockFrequency = Common_ADTIM_RCC_Initialize(ADTIM, RCC_HCLK_DIV2);

    Common_ADTIM_GPIO_Initialize(ADTIM);

    Common_ADTIM_Break_IO_Initialize(ADTIM);

    /* 
    ADTIM Configuration to:

    1/ Generate 4 complementary PWM signals with 4 different duty cycles:
    ADTIMCLK is fixed to SystemCoreClock, the ADTIM Prescaler is equal to 0 so the
    ADTIM counter clock used is SystemCoreClock.
    * SystemCoreClock is set to 48 MHz.

    The objective is to generate PWM signal at 17.57 KHz:
    - ADTIM_Period = (SystemCoreClock / 17570) - 1
    
    The Three Duty cycles are computed as the following description:

    The channel 1 duty cycle is set to 50% so channel 1N is set to 50%.
    The channel 2 duty cycle is set to 25% so channel 2N is set to 75%.
    The channel 3 duty cycle is set to 12.5% so channel 3N is set to 87.5%.
    The channel 4 duty cycle is set to 12.5% so channel 4N is set to 87.5%.
    The Timer pulse is calculated as follows:
      - ChannelxPulse = DutyCycle * (ADTIM_Period - 1) / 100

    2/ Insert a dead time equal to 11/SystemCoreClock ns
    3/ Configure the break feature, active at High level, and using the automatic
     output enable feature
    4/ Use the Locking parameters level1.
                                                                                */

    /* Compute the value to be set in AR register to generate signal frequency at 17.57 Khz */
    TimerPeriod = (ADTIMClockFrequency / 17570) - 1;
    /* Compute CCDAT1 value to generate a duty cycle at 50% for channel 1 */
    Channel1Pulse = (uint16_t)(((uint32_t)5 * (TimerPeriod - 1)) / 10);
    /* Compute CCDAT2 value to generate a duty cycle at 25%  for channel 2 */
    Channel2Pulse = (uint16_t)(((uint32_t)25 * (TimerPeriod - 1)) / 100);
    /* Compute CCDAT3 value to generate a duty cycle at 12.5%  for channel 3 */
    Channel3Pulse = (uint16_t)(((uint32_t)125 * (TimerPeriod - 1)) / 1000);
    /* Compute CCDAT4 value to generate a duty cycle at 12.5%  for channel 3 */
    channel4Pulse = (uint16_t)(((uint32_t)125 * (TimerPeriod - 1)) / 1000);

    /* TIM Base Init, Period = TimerPeriod, Prescaler = 0 */
    Common_TIM_Base_Initialize(ADTIM, TimerPeriod, 0);
    
    TIM_Output_Channel_Struct_Initialize(&TIM_OCInitStructure);

    /* Channel 1, 2 and 3 Configuration in PWM mode */
    TIM_OCInitStructure.OcMode       = TIM_OCMODE_PWM2;
    TIM_OCInitStructure.OutputState  = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.OutputNState = TIM_OUTPUT_NSTATE_ENABLE;
    TIM_OCInitStructure.Pulse        = Channel1Pulse;
    TIM_OCInitStructure.OcPolarity   = TIM_OC_POLARITY_LOW;
    TIM_OCInitStructure.OcNPolarity  = TIM_OCN_POLARITY_LOW;
    TIM_OCInitStructure.OcIdleState  = TIM_OC_IDLE_STATE_SET;
    TIM_OCInitStructure.OcNIdleState = TIM_OCN_IDLE_STATE_RESET;
    TIM_Output_Channel1_Initialize(ADTIM, &TIM_OCInitStructure);

    TIM_OCInitStructure.Pulse = Channel2Pulse;
    TIM_Output_Channel2_Initialize(ADTIM, &TIM_OCInitStructure);

    TIM_OCInitStructure.Pulse = Channel3Pulse;
    TIM_Output_Channel3_Initialize(ADTIM, &TIM_OCInitStructure);

    /* Output Compare Active Mode configuration: Channel4 */
    /* The CH4N if only for ADTIM, not for ADTIM */
    TIM_OCInitStructure.Pulse = channel4Pulse;
    TIM_Output_Channel4_Initialize(ADTIM, &TIM_OCInitStructure);

    /* Automatic Output enable, Break, dead time and lock configuration*/
    TIM_Break_And_Dead_Time_Struct_Initialize(&TIM_BDTRInitStructure);
    
    TIM_BDTRInitStructure.OssrState       = TIM_OSSR_STATE_ENABLE;
    TIM_BDTRInitStructure.OssiState       = TIM_OSSI_STATE_ENABLE;
    TIM_BDTRInitStructure.LockLevel       = TIM_LOCK_LEVEL_1;
    TIM_BDTRInitStructure.DeadTime        = 11;
    TIM_BDTRInitStructure.Break           = TIM_BREAK_IN_ENABLE;
    TIM_BDTRInitStructure.BreakPolarity   = TIM_BREAK_POLARITY_HIGH;
    TIM_BDTRInitStructure.AutomaticOutput = TIM_AUTO_OUTPUT_ENABLE;
    TIM_BDTRInitStructure.IomBreakEn      = true;

    TIM_Break_And_Dead_Time_Set(ADTIM, &TIM_BDTRInitStructure);

    TIM_On(ADTIM);
    TIM_PWM_Output_Enable(ADTIM);



    while(1)
    {

    }
}
