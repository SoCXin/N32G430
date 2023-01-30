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

#if ADTIM_NUM == 1

#define ADTIM_DMAR_ADDRESS ((uint32_t)0x40012C4C) 

#elif ADTIM_NUM == 8

#define ADTIM_DMAR_ADDRESS ((uint32_t)0x4001344C)

#endif 

DMA_InitType DMA_InitStructure;

OCInitType TIM_OCInitStructure;
uint32_t SRC_Buffer[6] = {0x0FFF, 0x0000, 0x0555};
uint16_t DmaAgain      = 0;

uint16_t prescaler = 0;

uint32_t ADTIMClockFrequency = 0;

void GPIO_Config(TIM_Module *TIMx);

void Two_Rising_Edge_Generate(void);

/**
*\*brief  Main program
**/
int main(void)
{
    /* System Clocks Configuration */
    ADTIMClockFrequency = Common_ADTIM_RCC_Initialize(ADTIM, RCC_HCLK_DIV2);

    /* DMA configuration */
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_DST;
    DMA_InitStructure.BufSize        = 3;
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_MODE_DISABLE;
    DMA_InitStructure.MemoryInc      = DMA_MEM_INC_MODE_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_WIDTH_WORD;
    DMA_InitStructure.MemDataSize    = DMA_MEM_DATA_WIDTH_WORD;
    DMA_InitStructure.CircularMode   = DMA_CIRCULAR_MODE_DISABLE;
    DMA_InitStructure.Priority       = DMA_CH_PRIORITY_HIGH;
    DMA_InitStructure.Mem2Mem        = DMA_MEM2MEM_DISABLE;
    Common_TIM_DMA_Config(DMA_CH5, &DMA_InitStructure, DMA_REMAP_TIM1_UP);
    
    /* Configure the GPIO ports */
    Common_ADTIM_GPIO_Initialize(ADTIM);

    /* 
    ADTIM Configuration: generate 1 PWM signal using the DMA burst mode:
    The ADTIMCLK frequency is set to SystemCoreClock (Hz), to get ADTIM counter
    clock at 24 MHz the Prescaler is computed as following:
     - Prescaler = (ADTIMCLK / ADTIM counter clock) - 1
    SystemCoreClock is set to 128 MHz 

    The ADTIM period is 5.8 KHz: ADTIM Frequency = ADTIM counter clock/(AR + 1)
                                               = 25.6 MHz / 4096 = 6.25KHz KHz
    ADTIM Channel1 duty cycle = (ADTIM_CCR1/ ADTIM_ARR)* 100 = 33.33%
                                                                            */
    prescaler = (uint16_t)(ADTIMClockFrequency / 25600000) - 1;

    /* Time base configuration， period = 0xFFFF, prescaler = prescaler */
    Common_TIM_Base_Initialize(ADTIM, 0xFFFF, prescaler);

    /* TIM Configuration in PWM Mode */
    TIM_OCInitStructure.OcMode      = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = 0xFFF;
    TIM_Output_Channel1_Initialize(ADTIM, &TIM_OCInitStructure);
    
    /* ADTIM DADDR Base register and DMA Burst Length Config */
    TIM_Dma_Config(ADTIM, TIM_DMABASE_AR, TIM_DMABURST_LENGTH_3TRANSFERS);

    /* ADTIM DMA Update enable */
    TIM_Dma_Enable(ADTIM, TIM_DMA_UPDATE);

    /* TIM enable counter */
    TIM_On(ADTIM);

    TIM_PWM_Output_Enable(ADTIM);

    Common_TIM_DMA_Start(DMA_CH5, (uint32_t)SRC_Buffer, ADTIM_DMAR_ADDRESS, 3);
    
    while(!DMA_Flag_Status_Get(DMA, DMA_CH5_TXCF))
    {
        /* none */
    }

    DMA_Flag_Status_Clear(DMA, DMA_CH5_GLBF | DMA_CH5_TXCF | DMA_CH5_HTXF | DMA_CH5_ERRF);

    while(1)
    {
        if (DmaAgain)
        {
            DmaAgain = 0;

            Common_TIM_DMA_Config(DMA_CH5, &DMA_InitStructure, DMA_REMAP_TIM1_UP);

            TIM_Dma_Config(ADTIM, TIM_DMABASE_AR, TIM_DMABURST_LENGTH_3TRANSFERS);
            TIM_Dma_Enable(ADTIM, TIM_DMA_UPDATE);

            /* DMA Channel5 enable */
            Common_TIM_DMA_Start(DMA_CH5, (uint32_t)SRC_Buffer, ADTIM_DMAR_ADDRESS, 3);

            /* Wait until DMA Channel5 end of Transfer */
            while (!DMA_Flag_Status_Get(DMA, DMA_CH5_TXCF))
            {
            }

            DMA_Flag_Status_Clear(DMA, DMA_CH5_GLBF | DMA_CH5_TXCF | DMA_CH5_HTXF | DMA_CH5_ERRF);
        }
    }
}


