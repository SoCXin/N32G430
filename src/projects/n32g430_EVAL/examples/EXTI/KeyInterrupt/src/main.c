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
#include <stdio.h>
#include <stdint.h>


/**
*\*\name    main.
*\*\fun     Main program.
*\*\param   none
*\*\return  none
**/
int main(void)
{
    /* SystemInit() function has been called by startup file startup_n32g430.s */

    /* Initialize Led as output pushpull mode */
    LedInit(LED1_PORT, LED1_PIN);
    LedInit(LED2_PORT, LED2_PIN);

    /*Initialize key as external line interrupt*/
    KeyInputExtiInit(KEY_INPUT_PORT, KEY_INPUT_PIN);

    /*Turn on Led1*/
    LedOn(LED1_PORT, LED1_PIN);

    while (1)
    {
    }
}


/**
*\*\name    Delay.
*\*\fun     Main program.
*\*\param   count
*\*\return  none
**/
void Delay(uint32_t count)
{
    for (; count > 0; count--)
        ;
}

/**
*\*\name    KeyInputExtiInit.
*\*\fun     Configures key port.
*\*\param  GPIOx :
*\*\          - GPIOA
*\*\          - GPIOB
*\*\          - GPIOC
*\*\          - GPIOD
*\*\param  Pin :
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
*\*\return  none
**/
void KeyInputExtiInit(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIO_InitType GPIO_InitStructure;
    EXTI_InitType EXTI_InitStructure;
    NVIC_InitType NVIC_InitStructure;



    /* Enable the GPIO Clock */
    if (GPIOx == GPIOA)
    {
        RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOA);
		RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_AFIO);
    }
    else if (GPIOx == GPIOB)
    {
		RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOB);
		RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_AFIO);
    }
    else if (GPIOx == GPIOC)
    {
		RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOC);
		RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_AFIO);
    }
    else if (GPIOx == GPIOD)
    {
		RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOD);
		RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_AFIO);		
    }
    else
    {
        return;
    }

    if (Pin <= GPIO_PIN_ALL)
    {
        GPIO_Structure_Initialize(&GPIO_InitStructure);
        GPIO_InitStructure.Pin          = Pin;
        GPIO_InitStructure.GPIO_Pull    = GPIO_PULL_UP;
        GPIO_Peripheral_Initialize(GPIOx, &GPIO_InitStructure);
    }

    /* Configure key EXTI Line to key input Pin */
    GPIO_EXTI_Line_Set(KEY_INPUT_PORT_SOURCE, KEY_INPUT_PIN_SOURCE);

    /* Configure key EXTI line */
    EXTI_InitStructure.EXTI_Line    = KEY_INPUT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Peripheral_Initializes(&EXTI_InitStructure);

    /* Set key input interrupt priority */
    NVIC_InitStructure.NVIC_IRQChannel                   = KEY_INPUT_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = NVIC_SUB_PRIORITY_1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);
}

/**
*\*\name    LedInit.
*\*\fun     Configures LED GPIO.
*\*\param  GPIOx :
*\*\          - GPIOA
*\*\          - GPIOB
*\*\          - GPIOC
*\*\          - GPIOD
*\*\param  Pin :
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
*\*\return  none
**/
void LedInit(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIO_InitType GPIO_InitStructure;

    /* Enable the GPIO Clock */
    if (GPIOx == GPIOA)
    {
        RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOA);
    }
    else if (GPIOx == GPIOB)
    {
        RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOB);
    }
    else if (GPIOx == GPIOC)
    {
        RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOB);
    }
    else if (GPIOx == GPIOD)
    {
        RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOB);     
    }
    else
    {
        return;
    }

    /* Configure the GPIO pin as output push-pull */
    if (Pin <= GPIO_PIN_ALL)
    {
        GPIO_Structure_Initialize(&GPIO_InitStructure);
        GPIO_InitStructure.Pin = Pin;
        GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUT_PP;
        GPIO_Peripheral_Initialize(GPIOx, &GPIO_InitStructure);
    }
}


/**
*\*\name    LedOn.
*\*\fun     Turns selected Led on.
*\*\param  GPIOx :
*\*\          - GPIOA
*\*\          - GPIOB
*\*\          - GPIOC
*\*\          - GPIOD
*\*\param  Pin :
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
*\*\return  none
**/
void LedOn(GPIO_Module* GPIOx, uint16_t Pin)
{    
    GPIO_Pins_Set(GPIOx, Pin);
}

/**
*\*\name    LedOff.
*\*\fun     Turns selected Led off.
*\*\param  GPIOx :
*\*\          - GPIOA
*\*\          - GPIOB
*\*\          - GPIOC
*\*\          - GPIOD
*\*\param  Pin :
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
*\*\return  none
**/
void LedOff(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIO_PBC_Pins_Reset(GPIOx, Pin);
}

/**
*\*\name    LedBlink.
*\*\fun     Toggles the selected Led.
*\*\param  GPIOx :
*\*\          - GPIOA
*\*\          - GPIOB
*\*\          - GPIOC
*\*\          - GPIOD
*\*\param  Pin :
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
*\*\return  none
**/
void LedBlink(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIO_Pin_Toggle(GPIOx, Pin);
}

/**
*\*\name    assert_failed.
*\*\fun     Reports the name of the source file and the source line number
*\*\        where the assert_param error has occurred.
*\*\param   file pointer to the source file name
*\*\param   line assert_param error line source number
*\*\return  none
**/
#ifdef USE_FULL_ASSERT
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    while (1)
    {
    }
}
#endif /* USE_FULL_ASSERT */

