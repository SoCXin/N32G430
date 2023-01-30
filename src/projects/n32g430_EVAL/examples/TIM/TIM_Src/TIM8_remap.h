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
*\*\file TIM8_remap.h
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
**/
#ifndef TIM8_REMAP_H
#define TIM8_REMAP_H

#include "n32g430_gpio.h"

/** TIM8 remap **/
/* ETR */
#define TIM8_REMAP0_ETR_PORT        GPIOA
#define TIM8_REMAP0_ETR_PIN         GPIO_PIN_0
#define TIM8_REMAP0_ETR_AF          GPIO_AF8_TIM8
/* CH1 */
#define TIM8_REMAP0_CH1_PORT        GPIOA
#define TIM8_REMAP0_CH1_PIN         GPIO_PIN_2
#define TIM8_REMAP0_CH1_AF          GPIO_AF9_TIM8 
/* CH2 */
#define TIM8_REMAP0_CH2_PORT        GPIOA
#define TIM8_REMAP0_CH2_PIN         GPIO_PIN_3
#define TIM8_REMAP0_CH2_AF          GPIO_AF10_TIM8
/* CH3 */
#define TIM8_REMAP0_CH3_PORT        GPIOA
#define TIM8_REMAP0_CH3_PIN         GPIO_PIN_4
#define TIM8_REMAP0_CH3_AF          GPIO_AF9_TIM8 
/* CH4 */
#define TIM8_REMAP0_CH4_PORT        GPIOA
#define TIM8_REMAP0_CH4_PIN         GPIO_PIN_5
#define TIM8_REMAP0_CH4_AF          GPIO_AF9_TIM8 

/* BKIN */
#define TIM8_REMAP0_BKIN_PORT       GPIOA
#define TIM8_REMAP0_BKIN_PIN        GPIO_PIN_6
#define TIM8_REMAP0_BKIN_AF         GPIO_AF7_TIM8 
/* CH1N */
#define TIM8_REMAP0_CH1N_PORT       GPIOA
#define TIM8_REMAP0_CH1N_PIN        GPIO_PIN_7
#define TIM8_REMAP0_CH1N_AF         GPIO_AF7_TIM8 
/* CH2N */
#define TIM8_REMAP0_CH2N_PORT       GPIOB
#define TIM8_REMAP0_CH2N_PIN        GPIO_PIN_3
#define TIM8_REMAP0_CH2N_AF         GPIO_AF8_TIM8 
/* CH3N */
#define TIM8_REMAP0_CH3N_PORT       GPIOB
#define TIM8_REMAP0_CH3N_PIN        GPIO_PIN_6
#define TIM8_REMAP0_CH3N_AF         GPIO_AF13_TIM8

#endif
