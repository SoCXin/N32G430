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
#include "log.h"
#include "bsp_led.h"
#include "bsp_iwdg.h"
#include "bsp_delay.h"

/**
 *\*\name   main.
 *\*\fun    main function.
 *\*\param  none.
 *\*\return none.
**/
int main(void)
{
	uint8_t temp = 0;
	
	log_init();
	log_info("\r\n IWDG Demo Reset \r\n");
	
	/* Initialize the LEDs */
	LED_Initialize(LED1_GPIO_PORT, LED1_GPIO_PIN | LED2_GPIO_PIN);
	
	/* Debug mode IWDG stopped */
	DBG_Peripheral_ON(DBG_IWDG_STOP);
	
	/* Check if the system has resumed from IWDG reset */
    if(RCC_Flag_Status_Get(RCC_FLAG_IWDGRST) != RESET)
	{
		/* IWDGRST flag set */
        /* Turn On LED1 */
        LED_On(LED1_GPIO_PORT, LED1_GPIO_PIN);
        log_info("\r\n Reset By IWDG \r\n");
		
		/* Clear reset flags */
		RCC_Reset_Flag_Clear();
	}		
	else 
	{
		/* IWDG Reset flag is not set */
        /* Turn Off LED1 */
        LED_Off(LED1_GPIO_PORT, LED1_GPIO_PIN);
	}
	
	LED_Off(LED2_GPIO_PORT, LED2_GPIO_PIN);
	
	/* Timeout is equal to 1 / 40000 * 128 * 4080 = ~13s */
	IWDG_Config(IWDG_CONFIG_PRESCALER_DIV128, 0xFF0);
	
	while(temp < 3)
	{
		/* Toggle LED2 */
		LED_Toggle(LED2_GPIO_PORT, LED2_GPIO_PIN);
		
		/* Delay 10s */
		SysTick_Delay_Ms(10000);
		
		log_info("\r\n Feed the dog i = %d times.\r\n", (temp + 1));
		/* Feed the dog */
		IWDG_Feed();
		
		temp++;
	}
	
	log_info("\r\n Do not feed the dog, after 13s system reset.\r\n");
	while(1)
	{
		/* Turn Off LED2 */
        LED_Off(LED2_GPIO_PORT, LED2_GPIO_PIN);
	}
}




