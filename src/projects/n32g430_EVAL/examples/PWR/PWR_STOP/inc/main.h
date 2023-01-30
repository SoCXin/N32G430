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
*\*\file main.h
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "n32g430.h"

/*Led1-PA1,Led2-PA7,Led3-PA9*/

#define LED1           GPIO_PIN_1
#define LED2           GPIO_PIN_7
#define KEY_INPUT_PORT GPIOA
#define KEY_INPUT_PIN  GPIO_PIN_0

typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} Status;

void LEDInit(uint16_t Pin);
void LedOn(uint16_t Pin);
void LedOff(uint16_t Pin);
void Ledlink(uint16_t Pin);
void delay(vu32 nCount);
void SYSCLKConfig_STOP(uint32_t RCC_PLLMULL);
void KeyInputExtiInit(GPIO_Module* GPIOx, uint16_t Pin);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

