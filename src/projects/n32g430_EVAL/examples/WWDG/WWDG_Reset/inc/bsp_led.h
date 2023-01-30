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
*\*\file bsp_led.h
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
**/


#ifndef __BSP_LED_H__
#define __BSP_LED_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "n32g430.h"

/** Define the GPIO port to which the LED is connected **/
#define LED1_GPIO_PORT    	GPIOA			            /* GPIO port */
#define LED1_GPIO_CLK 	    RCC_AHB_PERIPH_GPIOA		/* GPIO port clock */
#define LED1_GPIO_PIN		GPIO_PIN_1			        /* GPIO connected to the SCL clock line */

#define LED2_GPIO_PORT    	GPIOA			            /* GPIO port */
#define LED2_GPIO_CLK 	    RCC_AHB_PERIPH_GPIOA		/* GPIO port clock */
#define LED2_GPIO_PIN		GPIO_PIN_7			        /* GPIO connected to the SCL clock line */

#define LED3_GPIO_PORT    	GPIOA			            /* GPIO port */
#define LED3_GPIO_CLK 	    RCC_AHB_PERIPH_GPIOA		/* GPIO port clock */
#define LED3_GPIO_PIN		GPIO_PIN_9			        /* GPIO connected to the SCL clock line */


/** Define macros that control IO **/
#define LED1_TOGGLE		    {LED1_GPIO_PORT->POD ^= LED1_GPIO_PIN;}
#define LED1_ON		        {LED1_GPIO_PORT->PBSC = LED1_GPIO_PIN;}
#define LED1_OFF			{LED1_GPIO_PORT->PBC = LED1_GPIO_PIN;}

#define LED2_TOGGLE		    {LED2_GPIO_PORT->POD ^= LED2_GPIO_PIN;}
#define LED2_ON		        {LED2_GPIO_PORT->PBSC = LED2_GPIO_PIN;}
#define LED2_OFF			{LED2_GPIO_PORT->PBC = LED2_GPIO_PIN;}

#define LED3_TOGGLE		    {LED3_GPIO_PORT->POD ^= LED3_GPIO_PIN;}
#define LED3_ON		        {LED3_GPIO_PORT->PBSC = LED3_GPIO_PIN;}
#define LED3_OFF			{LED3_GPIO_PORT->PBC = LED3_GPIO_PIN;}


void LED_Initialize(GPIO_Module* GPIOx, uint16_t pin);
void LED_Toggle(GPIO_Module* GPIOx, uint16_t pin);
void LED_On(GPIO_Module* GPIOx,uint16_t pin);
void LED_Off(GPIO_Module* GPIOx,uint16_t pin);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_LED_H__ */


