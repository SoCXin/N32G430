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
*\*\file bsp_key.c
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
**/

#include "bsp_key.h"

/**
 *\*\name   Key_Input_Initialize.
 *\*\fun    Key input detection initialization.
 *\*\param  GPIOx :
 *\*\          - GPIOA
 *\*\          - GPIOB
 *\*\          - GPIOC
 *\*\          - GPIOD
 *\*\param  pin :
 *\*\          - GPIO_PIN_0
 *\*\          - GPIO_PIN_1
 *\*\          - GPIO_PIN_2
 *\*\          - GPIO_PIN_3
 *\*\          - GPIO_PIN_4
 *\*\          - GPIO_PIN_5
 *\*\          - GPIO_PIN_6
 *\*\          - GPIO_PIN_7
 *\*\          - GPIO_PIN_8
 *\*\          - GPIO_PIN_9
 *\*\          - GPIO_PIN_10
 *\*\          - GPIO_PIN_11
 *\*\          - GPIO_PIN_12
 *\*\          - GPIO_PIN_13
 *\*\          - GPIO_PIN_14
 *\*\          - GPIO_PIN_15
 *\*\return none.
**/
void Key_Input_Initialize(GPIO_Module* GPIOx, uint16_t pin)
{
	/* Define a structure of type GPIO_InitType */
	GPIO_InitType GPIO_InitStructure;
	
	/* Enable KEY related GPIO peripheral clock */
	if(GPIOx == GPIOA)
	{
		RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOA);
	}
	else if(GPIOx == GPIOB)
	{
		RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOB);
	}
	else if(GPIOx == GPIOC)
	{
		RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOC);
	}
	else
	{
		RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOD);
	}
	
	if(pin < GPIO_PIN_ALL)
	{
		/* Assign default value to GPIO_InitStructure structure */
		GPIO_Structure_Initialize(&GPIO_InitStructure);
		
		GPIO_InitStructure.Pin       = pin;
		GPIO_InitStructure.GPIO_Mode = GPIO_MODE_INPUT;
		GPIO_InitStructure.GPIO_Pull = GPIO_PULL_UP;
		/* Initialize GPIO */
		GPIO_Peripheral_Initialize(GPIOx, &GPIO_InitStructure);
	}
}

