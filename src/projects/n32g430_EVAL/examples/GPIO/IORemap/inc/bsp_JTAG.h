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
*\*\file bsp_JTAG.h
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
**/

#ifndef __BSP_JTAG_H__
#define __BSP_JTAG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "n32g430.h"

#define JTMS_PORT    GPIOA
#define JTMS_CLK     RCC_AHB_PERIPH_GPIOA
#define JTMS_PIN     GPIO_PIN_13

#define JTDI_PORT    GPIOA
#define JTDIS_CLK    RCC_AHB_PERIPH_GPIOA
#define JTDI_PIN     GPIO_PIN_15

#define JTRST_PORT   GPIOB
#define JTRST_CLK    RCC_AHB_PERIPH_GPIOB
#define JTRST_PIN    GPIO_PIN_4

#define JTCLK_PORT   GPIOA
#define JTCLK_CLK    RCC_AHB_PERIPH_GPIOB
#define JTCLK_PIN    GPIO_PIN_14

#define JTDO_PORT    GPIOB
#define JTDO_CLK     RCC_AHB_PERIPH_GPIOB
#define JTDO_PIN     GPIO_PIN_3

void JTAG_Function_Initialize(void);
void JTAG_As_GPIO_Initialize(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_JTAG_H__ */

