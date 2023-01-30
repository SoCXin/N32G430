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

#include "main.h"
#include "bsp_led.h"
#include "bsp_delay.h"


/**
 *\*\name   main.
 *\*\fun    main function.
 *\*\param  none.
 *\*\return none.
**/
int main(void)
{
	/* Initialize Led1~Led3 as output push-pull mode */
	LED_Initialize(LED1_GPIO_PORT, LED1_GPIO_PIN | LED2_GPIO_PIN | LED3_GPIO_PIN);

	/* Turn off Led1~Led3 */
	LED_Off(LED1_GPIO_PORT, LED1_GPIO_PIN | LED2_GPIO_PIN | LED3_GPIO_PIN);
	
	/* Turn on Led2~Led3 */
	LED_On(LED2_GPIO_PORT, LED2_GPIO_PIN | LED3_GPIO_PIN);
	
	/* Delay 1s */
	SysTick_Delay_Ms(1000);
	
	while(1)
	{
		/* Turn on Led1 */
		LED1_ON;
		
		/* Toggle LED2 */
		LED_Toggle(LED2_GPIO_PORT, LED2_GPIO_PIN);
		
		/* Delay 1s */
	    SysTick_Delay_Ms(1000);
		
		/* Toggle LED3 */
		LED_Toggle(LED3_GPIO_PORT, LED3_GPIO_PIN);
		
		/* Delay 1s */
	    SysTick_Delay_Ms(1000);
		
		/* Turn off LED1 */
		LED1_OFF;
		
		/* Delay 1s */
	    SysTick_Delay_Ms(1000);
	}
}


