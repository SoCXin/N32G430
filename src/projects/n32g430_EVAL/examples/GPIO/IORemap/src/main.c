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
#include "bsp_key.h"
#include "bsp_JTAG.h"


/**
 *\*\name   Delay.
 *\*\fun    Inserts a delay time.
 *\*\param  count:
 *\*\          - any number
 *\*\return none.
**/
void Delay(uint32_t count)
{
    for (; count > 0; count--)
        ;
}

/**
 *\*\name   main.
 *\*\fun    main function.
 *\*\param  none.
 *\*\return none.
**/
int main(void)
{
	/*Initialize Led1 and Led2 as output push-pull mode*/
	LED_Initialize(LED1_GPIO_PORT, LED1_GPIO_PIN | LED2_GPIO_PIN);
	
	Key_Input_Initialize(KEY3_PORT, KEY3_PIN);
	
	while(1)
	{
		if(GPIO_Input_Pin_Data_Get(KEY3_PORT, KEY3_PIN) == PIN_RESET)
		{
			/* Turn on Led1 */
			LED_On(LED1_GPIO_PORT, LED1_GPIO_PIN);
			
			/* Turn off Led2 */
			LED_Off(LED2_GPIO_PORT, LED2_GPIO_PIN);
			
			/* Disable the JTAG Debug Port SWJ-DP */
			JTAG_As_GPIO_Initialize();
			
			while(1)
			{
				/* Toggle JTMS pin */
				if(1 - GPIO_Input_Pin_Data_Get(JTMS_PORT, JTMS_PIN))
				{
					GPIO_Pins_Set(JTMS_PORT, JTMS_PIN);
					
				}
				else 
				{
					GPIO_PBC_Pins_Reset(JTMS_PORT, JTMS_PIN);
				}
				/* Insert delay */
        Delay(0x8FFFF);
				
        /* Toggle JTCK pin */
				if(1 - GPIO_Input_Pin_Data_Get(JTCLK_PORT, JTCLK_PIN))
				{
					GPIO_Pins_Set(JTCLK_PORT, JTCLK_PIN);
				}
				else 
				{
					GPIO_PBC_Pins_Reset(JTCLK_PORT, JTCLK_PIN);
				}
				/* Insert delay */
        Delay(0x8FFFF);
				
				/* Toggle JTDI pin */
				if(1 - GPIO_Input_Pin_Data_Get(JTDI_PORT, JTDI_PIN))
				{
					GPIO_Pins_Set(JTDI_PORT, JTDI_PIN);
				}
				else 
				{
					GPIO_PBC_Pins_Reset(JTDI_PORT, JTDI_PIN);
				}
				/* Insert delay */
        Delay(0x8FFFF);
				
				/* Toggle JTD0 pin */
				if(1 - GPIO_Input_Pin_Data_Get(JTDO_PORT, JTDO_PIN))
				{
					GPIO_Pins_Set(JTDO_PORT, JTDO_PIN);
				}
				else 
				{
					GPIO_PBC_Pins_Reset(JTDO_PORT, JTDO_PIN);
				}
				/* Insert delay */
        Delay(0x8FFFF);
				
				/* Toggle JTRST pin */
				if(1 - GPIO_Input_Pin_Data_Get(JTRST_PORT, JTRST_PIN))
				{
					GPIO_Pins_Set(JTRST_PORT, JTRST_PIN);
				}
				else 
				{
					GPIO_PBC_Pins_Reset(JTRST_PORT, JTRST_PIN);
				}
				/* Insert delay */
        Delay(0x8FFFF);
				
				if(GPIO_Input_Pin_Data_Get(KEY3_PORT, KEY3_PIN) == PIN_SET)
				{
					break;
				}	
			}
		}
		else
		{
			/* Enablethe JTAG Debug Port SWJ-DP */
			JTAG_Function_Initialize();
			
			/* Turn on Led2 */
			LED_On(LED2_GPIO_PORT, LED2_GPIO_PIN);
			
			/* Turn off Led1 */
			LED_Off(LED1_GPIO_PORT, LED1_GPIO_PIN);
			
			while(1)
			{
				if(GPIO_Input_Pin_Data_Get(KEY3_PORT, KEY3_PIN) == PIN_RESET)
				{
					break;
				}
			}
		}
	}
}

