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
*\*\file GPTIM_common.h
*\*\author Nations 
*\*\version v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
**/
#ifndef GPTIM_COMMON_H
#define GPTIM_COMMON_H

#include "n32g430.h"
#include "n32g430_rcc.h"
#include "n32g430_gpio.h"
#include "n32g430_tim.h"
#include "misc.h"
#include "TIM2_remap.h"
#include "TIM3_remap.h"
#include "TIM4_remap.h"
#include "TIM5_remap.h"


#define CC1_INTERRUPT_PIN       GPIO_PIN_8
#define CC1_INTERRUPT_PORT      GPIOA 

#define CC2_INTERRUPT_PIN       GPIO_PIN_9
#define CC2_INTERRUPT_PORT      GPIOA

#define CC3_INTERRUPT_PIN       GPIO_PIN_10
#define CC3_INTERRUPT_PORT      GPIOA

#define CC4_INTERRUPT_PIN       GPIO_PIN_11
#define CC4_INTERRUPT_PORT      GPIOA


#ifndef GPTIM_NUM 
#define GPTIM_NUM     3
#endif

#if GPTIM_NUM == 2

    #define GPTIM   TIM2

#elif GPTIM_NUM == 3

    #define GPTIM   TIM3

#elif GPTIM_NUM == 4

    #define GPTIM   TIM4

#elif GPTIM_NUM == 5

    #define GPTIM   TIM5

#endif


uint32_t Common_GPTIM_RCC_Initialize(TIM_Module *TIMx, uint32_t hclk_division);

void Common_GPTIM_GPIO_Initialize(TIM_Module *TIMx);

#endif

