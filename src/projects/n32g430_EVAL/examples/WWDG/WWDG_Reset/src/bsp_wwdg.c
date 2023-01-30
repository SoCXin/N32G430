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
*\*\file bsp_wwdg.c
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
**/

#include "bsp_wwdg.h"

/**
 *\*\name   WWDG_Config.
 *\*\fun    Configure WWDG.
 *\*\param  prescaler :
 *\*\          - WWDG_PRESCALER_DIV1  (32MHz/4096/1 = 7813Hz(~128us))
 *\*\          - WWDG_PRESCALER_DIV2  (32MHz/4096/2 = 3906Hz(~256us))
 *\*\          - WWDG_PRESCALER_DIV4  (32MHz/4096/4 = 1953Hz(~512us))
 *\*\          - WWDG_PRESCALER_DIV8  (32MHz/4096/8 = 977Hz(~1024us))
 *\*\param  window_value :
 *\*\          -0x40 ~ 0x3FFF
 *\*\param  counter_value :
 *\*\          -0x40 ~ 0x3FFF
**/
void WWDG_Config(uint32_t prescaler, uint16_t window_value, uint16_t counter_value)
{
	/* Enable WWDG clock */
	RCC_APB1_Peripheral_Clock_Enable(RCC_APB1_PERIPH_WWDG);
	
	/* WWDG clock counter = (PCLK1(32MHz)/4096)/prescaler */
	WWDG_Prescaler_Division_Set(prescaler);
	
	/* Set Window value to window_value; WWDG counter should be refreshed only when the counter
     is below window_value and greater than 0x40 otherwise a reset will be generated */
	WWDG_Window_Value_Set(window_value);
	
	/* Set the value of the down counter */
	WWDG_Counter_Value_Set(counter_value);
	
	/* Enable WWDG and set counter value to counter_value */
	WWDG_Enable(counter_value);
}

/**
 *\*\name    WWDG_Feed.
 *\*\fun     Feed the dog.
 *\*\param  counter_value :
 *\*\          -0x40 ~ 0x3FFF
 *\*\return  none
**/
void WWDG_Feed(uint16_t counter_value)
{
	WWDG_Counter_Value_Set(counter_value);
}

