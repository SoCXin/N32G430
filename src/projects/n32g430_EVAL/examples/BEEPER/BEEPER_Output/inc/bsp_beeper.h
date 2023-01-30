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
*\*\file bsp_beeper.h
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
**/

#ifndef __BSP_BEEPER_H__
#define __BSP_BEEPER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "n32g430.h"

/* BEEPER port macro definition */
#define BEEPER1_GPIO_PORT   GPIOA
#define BEEPER1_GPIO_CLK    RCC_AHB_PERIPH_GPIOA
#define BEEPER1_GPIO_PIN    GPIO_PIN_6 | GPIO_PIN_7

#define BEEPER2_GPIO_PORT   GPIOB
#define BEEPER2_GPIO_CLK    RCC_AHB_PERIPH_GPIOB
#define BEEPER2_GPIO_PIN    GPIO_PIN_6 | GPIO_PIN_7

void BEEPER_Normal_Work(uint32_t clock_selection, uint32_t prescale_factor, uint32_t div_factor);

#ifdef __cplusplus
}
#endif
#endif /* __BSP_BEEPER_H__ */

