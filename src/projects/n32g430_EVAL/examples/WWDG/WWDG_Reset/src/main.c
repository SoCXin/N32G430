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
#include "bsp_wwdg.h"
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
	log_info("\r\n WWDG Demo Reset \r\n");
	
	/* Initialize the LED1/LED2 */
	LED_Initialize(LED1_GPIO_PORT, LED1_GPIO_PIN | LED2_GPIO_PIN);
	
	/* Debug mode WWDG stopped */
	DBG_Peripheral_ON(DBG_WWDG_STOP);
	
	/* Check if the system has resumed from WWDG reset */
    if(RCC_Flag_Status_Get(RCC_FLAG_WWDGRST) != RESET)
	{
		/* WWDGRST flag set */
        /* Turn On LED1 */
        LED_On(LED1_GPIO_PORT, LED1_GPIO_PIN);
        log_info("\r\n Reset By WWDG \r\n");
		
		/* Clear reset flags */
		RCC_Reset_Flag_Clear();
	}		
	else 
	{
		/* WWDG Reset flag is not set */
        /* Turn Off LED1 */
        LED_Off(LED1_GPIO_PORT, LED1_GPIO_PIN);
	}
	
	/* Turn Off LED2 */
    LED_Off(LED2_GPIO_PORT, LED2_GPIO_PIN);
	
	/* Counter value to 16383, WWDG timeout = ~1024 us * 16320 = 16.7s
     In this case the refresh window is: ~1024 us * (16383 - 10000) = 6.6s < refresh window < ~1024 us * 16319 = 16.7s */
	WWDG_Config(WWDG_PRESCALER_DIV8, 0x2710, 0x3FFF);
	
	log_info("\r\n Set the upper window value to 0x2710.\r\n");
	
	while(temp < 3)
	{
		/* The program that needs to be monitored by WWDG should be written here. 
		   The running time of the monitored program determines how large the window value should be set. 
		*/
		
		/* The counter value is initialized to a maximum of 0x3FFF. 
		   When the WWDG is turned on, the counter value will continue to decrease. 
		   When the counter value is greater than the window value, if the dog is fed, the WWDG will be reset. 
		   When the counter is reduced to 0x40 and the dog is not fed, the monitored program will Very dangerous, 
		   the WWDG is reset when the counter is decremented to 0x3F again. 
		   So to feed the dog when the value of the counter is between the window value and 0x40, 
		   the lower window value of 0x40 is fixed.
		*/
		
		/* Toggle LED2 */
		LED2_TOGGLE;
		
		/* Insertion delay. delay 7s */
		SysTick_Delay_Ms(7000);
		
		/* Feed the dog, reset the counter to the maximum value of 0x3FFF */
		WWDG_Feed(0x3FFF);
		
		log_info("\r\n Feed the dog i = %d times.\r\n", (temp + 1));

		temp++;
	}
	
	log_info("\r\n Do not Feed, system reset after 16.7s.\r\n");
	while(1)
	{
	}
}



