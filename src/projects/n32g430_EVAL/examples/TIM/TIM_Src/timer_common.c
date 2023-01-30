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
*\*\file timer_common.c
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
**/
#include "timer_common.h"

static TIM_TimeBaseInitType TIM_TimeBaseStructure;

/**
*\*\name    Common_TIM_Base_Initialize.
*\*\param   TIMx :
*\*\          - TIM1
*\*\          - TIM2
*\*\          - TIM3
*\*\          - TIM4
*\*\          - TIM5
*\*\          - TIM6
*\*\param   period
*\*\          - [1, 0xffff]
*\*\param   prescaler
*\*\          - [0, 0xffff]
*\*\return  none
**/
void Common_TIM_Base_Initialize(TIM_Module *TIMx, uint16_t period, uint16_t prescaler)
{
    TIM_Base_Struct_Initialize(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.Period    = period;
    TIM_TimeBaseStructure.Prescaler = prescaler;
    TIM_TimeBaseStructure.ClkDiv    = 0;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;

    TIM_Base_Initialize(TIMx, &TIM_TimeBaseStructure);   
}

/**
*\*\name    Common_TIM_NVIC_Initialize.
*\*\param   IRQ_number :
*\*\          - TIM1_BRK_IRQn
*\*\          - TIM1_UP_IRQn
*\*\          - TIM1_TRG_COM_IRQn
*\*\          - TIM1_CC_IRQn
*\*\          - TIM2_IRQn
*\*\          - TIM3_IRQn
*\*\          - TIM4_IRQn
*\*\          - TIM8_BRK_IRQn
*\*\          - TIM8_UP_IRQn
*\*\          - TIM8_TRG_COM_IRQn
*\*\          - TIM8_CC_IRQn
*\*\          - TIM5_IRQn
*\*\          - TIM6_IRQ
*\*\param   command
*\*\          - ENABLE
*\*\          - DISABLE
*\*\return  none
**/
void Common_TIM_NVIC_Initialize(IRQn_Type IRQ_number, FunctionalState command)
{
    NVIC_InitType NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = IRQ_number;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    
    if(ENABLE == command)
    {
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    }
    else
    {
        NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    }
    
    NVIC_Initializes(&NVIC_InitStructure);
}

/**
*\*\name    Common_TIM_DMA_Config.
*\*\param   chan_handle
*\*\param   dma_param
*\*\param   req_remap
*\*\          - DMA_REMAP_TIM1_CH1
*\*\          - DMA_REMAP_TIM1_CH2
*\*\          - DMA_REMAP_TIM1_CH3
*\*\          - DMA_REMAP_TIM1_CH4
*\*\          - DMA_REMAP_TIM1_COM
*\*\          - DMA_REMAP_TIM1_UP
*\*\          - DMA_REMAP_TIM1_TRIG
*\*\          - DMA_REMAP_TIM2_CH1
*\*\          - DMA_REMAP_TIM2_CH2
*\*\          - DMA_REMAP_TIM2_CH3
*\*\          - DMA_REMAP_TIM2_CH4
*\*\          - DMA_REMAP_TIM2_UP
*\*\          - DMA_REMAP_TIM3_CH1
*\*\          - DMA_REMAP_TIM3_CH3
*\*\          - DMA_REMAP_TIM3_CH4
*\*\          - DMA_REMAP_TIM3_UP
*\*\          - DMA_REMAP_TIM3_TRIG
*\*\          - DMA_REMAP_TIM4_CH1
*\*\          - DMA_REMAP_TIM4_CH2
*\*\          - DMA_REMAP_TIM4_CH3
*\*\          - DMA_REMAP_TIM4_UP
*\*\          - DMA_REMAP_TIM5_CH1
*\*\          - DMA_REMAP_TIM5_CH2
*\*\          - DMA_REMAP_TIM5_CH3
*\*\          - DMA_REMAP_TIM5_CH4
*\*\          - DMA_REMAP_TIM5_UP
*\*\          - DMA_REMAP_TIM5_TRIG
*\*\          - DMA_REMAP_TIM6_UP
*\*\          - DMA_REMAP_TIM8_CH1
*\*\          - DMA_REMAP_TIM8_CH2
*\*\          - DMA_REMAP_TIM8_CH3
*\*\          - DMA_REMAP_TIM8_CH4
*\*\          - DMA_REMAP_TIM8_COM
*\*\          - DMA_REMAP_TIM8_UP
*\*\          - DMA_REMAP_TIM8_TRIG
*\*\return  none
**/
void Common_TIM_DMA_Config(DMA_ChannelType *chan_handle, DMA_InitType *dma_param, uint32_t req_remap)
{
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_DMA);
    
    DMA_Reset(chan_handle);

    DMA_Initializes(chan_handle, dma_param);

    DMA_Channel_Request_Remap(chan_handle, req_remap);
}

/**
*\*\name    Common_TIM_DMA_Config.
*\*\param   chan_handle
*\*\param   src_addr
*\*\param   dst_addr
*\*\param   len
*\*\return  none
**/
void Common_TIM_DMA_Start(DMA_ChannelType *chan_handle, uint32_t src_addr, uint32_t dst_addr, uint32_t len)
{
    DMA_Channel_Disable(chan_handle);

    chan_handle->TXNUM = len;
    if(chan_handle->CHCFG & DMA_DIR_PERIPH_DST)
    {
        chan_handle->PADDR = dst_addr;
        chan_handle->MADDR = src_addr;
    }
    else
    {
        chan_handle->PADDR = src_addr;
        chan_handle->MADDR = dst_addr;
    }

    DMA_Channel_Enable(chan_handle);
}
