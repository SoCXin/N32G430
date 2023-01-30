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
 */

#include "main.h"
#include "log.h"

#include <string.h>

/**
*\*\name    main.
*\*\fun     Main program.
*\*\param   none
*\*\return  none
**/
int main(void)
{
    /* Initialize LEDs on n32g430-EVAL board */
    LEDInit();
    LedOn(LED1_PIN);
    /* Enable PWR and BKP clock */
    RCC_APB1_Peripheral_Clock_Enable(RCC_APB1_PERIPH_PWR);

    /* Configure EXTI Line to generate an interrupt on falling edge */
    EXTI_Configuration();

    /* NVIC configuration */
    NVIC_Configuration();

    /* Configure the PVD Level to 2.28V */
    PWR_PVD_Level_Config(PWR_PVD_LEVEL_2V28);

    /* Enable the PVD Output */
    PWR_PVD_Enable();
    
    while (1)
    {
    }
}

/**
*\*\name    EXTI_Configuration.
*\*\fun     Configures EXTI Lines.
*\*\param   none
*\*\return  none
**/
void EXTI_Configuration(void)
{
    EXTI_InitType EXTI_InitStructure;

    /* Configure EXTI Line16(PVD Output) to generate an interrupt on rising and
       falling edges */
    EXTI_Flag_Status_Clear(EXTI_LINE16);
    EXTI_InitStructure.EXTI_Line    = EXTI_LINE16;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Peripheral_Initializes(&EXTI_InitStructure);
}

/**
*\*\name    NVIC_Configuration.
*\*\fun     Configure the nested vectored interrupt controller.
*\*\param   none
*\*\return  none
**/
void NVIC_Configuration(void)
{
    NVIC_InitType NVIC_InitStructure;

    /* Configure one bit for preemption priority */
    NVIC_Priority_Group_Set(NVIC_PER0_SUB4_PRIORITYGROUP);

    /* Enable the PVD Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = PVD_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);
}

/**
*\*\name    LedBlink.
*\*\fun     Toggles the selected Led.
*\*\param   GPIOx
*\*\param   Pin
*\*\return  none
**/
void LedBlink(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIOx->POD ^= Pin;
}

/**
*\*\name    LedOn.
*\*\fun     Turns selected Led on.
*\*\param   Pin
*\*\return  none
**/
void LedOn(uint16_t Pin)
{
    GPIOA->PBSC = Pin;
}

/**
*\*\name    LedOff.
*\*\fun     Turns selected Led Off.
*\*\param   Pin
*\*\return  none
**/
void LedOff(uint16_t Pin)
{
    GPIOA->PBC = Pin;
}

/**
*\*\name    LEDInit.
*\*\fun     Configures LED GPIO.
*\*\param   none
*\*\return  none
**/
void LEDInit(void)
{
    GPIO_InitType GPIO_InitStructure;

    /* Enable the GPIO_LED Clock */
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOA);

    GPIO_Structure_Initialize(&GPIO_InitStructure);
    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.Pin        = LED1_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_OUT_PP;

    GPIO_Peripheral_Initialize(LED1_PORT, &GPIO_InitStructure);
}

#ifdef USE_FULL_ASSERT
void assert_failed(const uint8_t* expr, const uint8_t* file, uint32_t line)
{
    while (1)
        ;
}
#endif

