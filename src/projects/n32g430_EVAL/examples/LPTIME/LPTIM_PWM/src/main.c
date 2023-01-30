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
*\*\file      main.c
*\*\author    Nations
*\*\version   v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved. 
*/
#include "main.h"

/* LPTIM_PWM */

void LedBlink(GPIO_Module* GPIOx, uint16_t Pin);
void LEDInit(uint16_t Pin);
void LedOn(uint16_t Pin);
void LedOff(uint16_t Pin);
void Ledlink(uint16_t Pin);
void delay(vu32 nCount);
void LPTIM_OutputIoInit(void);

/* Main program. */
int main(void)
{
    /* At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_n32g430.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_n32g430.c file
     */
    /* Init LED GPIO */
    LEDInit(LED1);
    /* Init output IO */
    LPTIM_OutputIoInit();
    /* Enable the LSI source */
    RCC_LPTIM_Enable();
    RCC_LPTIM_Clock_Config(RCC_LPTIMCLK_SRC_LSI);
    LPTIM_Prescaler_Set(LPTIM,LPTIM_PRESCALER_DIV4);
    /* Init lptim module */ 
    LPTIM_Waveform_Set(LPTIM,LPTIM_OUTPUT_WAVEFORM_PWM); 
    /* output wave */ 
    LPTIM_Polarity_Set(LPTIM,LPTIM_OUTPUT_POLARITY_REGULAR);  
    /* config the prescaler */          
    LPTIM_Interrupt_Enable(LPTIM, LPTIM_INT_CMPUPDIE);

    LPTIM_ON(LPTIM);
    /* config ARR ande compare register */ 
    LPTIM_Auto_Reload_Set(LPTIM,600);        
    LPTIM_Compare_Set(LPTIM,300);
    //while(!(LPTIM->INTSTS & LPTIM_INTSTS_CMPUPD));
    LPTIM_Counter_Start(LPTIM,LPTIM_OPERATING_MODE_CONTINUOUS);  
    while (1)
    {
    }
}

/**
*\*\name    Ledlink.
*\*\fun     Toggles the selected Led.
*\*\param   LED1
*\*\param   LED2
*\*\param   LED3
*\*\return  none
**/
void Ledlink(uint16_t Pin)
{
    GPIOA->POD ^= Pin;
}
/**
*\*\name    LedOn.
*\*\fun     Turns selected Led on.
*\*\param   LED1
*\*\param   LED2
*\*\param   LED3
*\*\return  none
**/
void LedOn(uint16_t Pin)
{
    GPIOA->PBC = Pin;
}
/**
*\*\name    LedOff.
*\*\fun     Turns selected Led Off.
*\*\param   LED1
*\*\param   LED2
*\*\param   LED3
*\*\return  none
**/
void LedOff(uint16_t Pin)
{
    GPIOA->PBSC = Pin;
}
/**
*\*\name    LEDInit.
*\*\fun     Configures LED GPIO.
*\*\param   LED1
*\*\param   LED2
*\*\param   LED3
*\*\return  none
**/
void LEDInit(uint16_t Pin)
{
    GPIO_InitType GPIO_InitStructure;
    GPIO_Structure_Initialize(&GPIO_InitStructure);

    /* Enable the GPIO_LED Clock */
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOA);

    /* Configure the GPIO_LED pin */
    GPIO_InitStructure.Pin        = Pin;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_OUT_PP;
    GPIO_Peripheral_Initialize(GPIOA, &GPIO_InitStructure);
}
/**
*\*\name    delay.
*\*\fun     Delay by counting .
*\*\param   nCount
*\*\return  none
**/
void delay(vu32 nCount)
{
    vu32 index = 0;
    for (index = (34000 * nCount); index != 0; index--)
    {
    }
}

/**
*\*\name    LPTIM_OutputIoInit.
*\*\fun     output IO Initaliza.
*\*\param   none
*\*\return  none
**/
void LPTIM_OutputIoInit(void)
{
    GPIO_InitType GPIO_InitStructure;
    GPIO_Structure_Initialize(&GPIO_InitStructure);

    /* Enable the GPIO Clock */
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOB);

    /* Configure the GPIO pin */
    GPIO_InitStructure.Pin        = GPIO_PIN_2;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF3_LPTIM;
    GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);
}
