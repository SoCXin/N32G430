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
*\*\file bsp_led.c
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
**/

#include "bsp_led.h"

/**
 *\*\name   LED_Initialize.
 *\*\fun    Initialize the specified LED.
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
 *\*\return  none.
**/
void LED_Initialize(GPIO_Module* GPIOx, uint16_t pin)
{
	/* Define a structure of type GPIO_InitType */
	GPIO_InitType GPIO_InitStructure;
	
	/* Enable LED related GPIO peripheral clock */
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
		
		/* Select the GPIO pin to control */
		GPIO_InitStructure.Pin          = pin;
		/* Set pin mode to general push-pull output */
		GPIO_InitStructure.GPIO_Mode    = GPIO_MODE_OUT_PP;
		/* Set the pin drive current to 4MA*/
		GPIO_InitStructure.GPIO_Current = GPIO_DS_4MA;
		/* Initialize GPIO */
		GPIO_Peripheral_Initialize(GPIOx, &GPIO_InitStructure);
	}
}

/**
 *\*\name   LED_Toggle.
 *\*\fun    GPIOx Specifies the led port to be toggled.
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
**/
void LED_Toggle(GPIO_Module* GPIOx, uint16_t pin)
{
    GPIO_Pin_Toggle(GPIOx, pin);;
}


/**
 *\*\name   LED_On.
 *\*\fun    GPIOx Specifies the led port to be set on.
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
**/
void LED_On(GPIO_Module* GPIOx,uint16_t pin)
{
    GPIO_Pins_Set(GPIOx, pin);
}


/**
 *\*\name   LED_Off.
 *\*\fun    GPIOx Specifies the led port to be set off.
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
**/
void LED_Off(GPIO_Module* GPIOx,uint16_t pin)
{
    GPIO_Pins_Reset(GPIOx, pin);
}

