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
    /* Initialize USART */
    log_init();
    log_info("STANDBY mode demo\n");
    /* Initialize LEDs */
    LEDInit(LED1);
    LEDInit(LED2);
    /* Turn on LED1 LED2 */
    LedOn(LED1);
    LedOn(LED2);
    
    Delay(200);
    
    /* Enable PWR clock */
    RCC_APB1_Peripheral_Clock_Enable(RCC_APB1_PERIPH_PWR);

    /* Enable WKUP pin */
    PWR_Wakeup_Pin_Enable(WAKEUP_PIN2,PWR_PIN_RISING);
//    DBG_Peripheral_ON(DBG_STANDBY);

    while (1)
    {
        /* Check if the Wake-Up flag is set */
        if (PWR_Flag_Status_Get(PWR_WKUP_FLAG_PA0) != RESET)
        {
            /* Clear Wake Up flag */
            PWR_Flag_Status_Clear(PWR_WKUP_FLAG_PA0);
        }
        /* Delay a long time */
        Delay(3000);
        log_info("Enter STANDBY mode\n");
		Ledlink(LED1);
        /* Request to enter STANDBY mode */
        PWR_STANDBY_Mode_Enter(PWR_STANDBY_ENTRY_WFI);
		Ledlink(LED2);
    }
}

/**
*\*\name    Ledlink.
*\*\fun     Toggles the selected Led.
*\*\param   Pin
*\*\return  none
**/
void Ledlink(uint16_t Pin)
{
    GPIOA->POD ^= Pin;
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
*\*\param   Pin
*\*\return  none
**/
void LEDInit(uint16_t Pin)
{
    GPIO_InitType GPIO_InitStructure;

    /* Enable the GPIO_LED Clock */
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOA);

    GPIO_Structure_Initialize(&GPIO_InitStructure);
    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.Pin        = Pin;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_OUT_PP;

    GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);
}

/**
*\*\name    Delay.
*\*\fun     Delay function.
*\*\param   nCount
*\*\return  none
**/
void Delay(vu32 nCount)
{
    vu32 index = 0;
    for (index = (34000 * nCount); index != 0; index--)
    {
    }
}

