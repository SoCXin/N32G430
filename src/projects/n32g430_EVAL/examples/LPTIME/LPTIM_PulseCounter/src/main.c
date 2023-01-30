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
*\*\version   v1.0.1
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved. 
*/
#include "main.h"

/* LPTIM_PulseCounter */

#define COUNT_NUM        10

void LedBlink(GPIO_Module* GPIOx, uint16_t Pin);
void LEDInit(uint16_t Pin);
void LedOn(uint16_t Pin);
void LedOff(uint16_t Pin);
void Ledlink(uint16_t Pin);
void delay(vu32 nCount);


void LPTIM_InputIoInit(void);
void PWM_Out(uint16_t Pin,uint8_t cnt);
uint16_t tempCNT;
/* Main program. */
int main(void)
{
    /* At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_n32g430.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_n32g430.c file
     */
    /* Init log */
    log_init();
    log_info("This is LPTIM pulse encoder mode demo\r\n");
    /* Init LED GPIO */
    LEDInit(LED1);
    LEDInit(LED2);
    /* Enable the LSI source */
    RCC_LPTIM_Enable();
    RCC_LSI_Enable();
    RCC_LPTIM_Clock_Config(RCC_LPTIMCLK_SRC_LSI);

    /* LPTIM input1 IO Init and encoder*/
    LPTIM_Prescaler_Set(LPTIM,LPTIM_PRESCALER_DIV1);
    LPTIM_InputIoInit();
  
    //PWR_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
    /* config timer    CKSEL:0  COUNTMODE:1 */  
    LPTIM_Counter_Mode_Set(LPTIM,LPTIM_COUNTER_MODE_EXTERNAL);
    /* config the rising polarity */
    LPTIM_Clock_Config(LPTIM,LPTIM_CLK_FILTER_NONE,LPTIM_CLK_POLARITY_RISING);    
    LPTIM_ON(LPTIM); 
    LPTIM_Auto_Reload_Set(LPTIM,60000);  
    LPTIM_Counter_Start(LPTIM,LPTIM_OPERATING_MODE_CONTINUOUS);  //start with the continue mode  
    /* Great 10 square waves ,and tempCNT should be equal to 10*/
    delay(10);
    PWM_Out(LED1,COUNT_NUM);
    tempCNT = LPTIM->CNT;
    if(tempCNT == COUNT_NUM)
    {
        log_info("Test PASS!\r\n");
    }
    else
    {
        log_info("Test ERR!\r\n");
    }
    while (1)
    {
    }
}

/**
*\*\name    LPTIM_InputIoInit.
*\*\fun     encode input IO Initaliza.
*\*\param   none
*\*\return  none
**/
void LPTIM_InputIoInit(void)
{
    GPIO_InitType GPIO_InitStructure;
    GPIO_Structure_Initialize(&GPIO_InitStructure);
    /* Enable the GPIO Clock */
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOB);

    /* Configure the GPIO pin */
    GPIO_InitStructure.Pin        = GPIO_PIN_5;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_INPUT;
    GPIO_InitStructure.GPIO_Alternate =  GPIO_AF3_LPTIM;
    GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);
}

/**
*\*\name    Lptim_EncInit.
*\*\fun     encode module Initaliza.
*\*\param   none
*\*\return  none
**/
void Lptim_EncInit(void)
{
    /* config LPTIM */
    LPTIM_Clock_Source_Set(LPTIM,LPTIM_CLK_SOURCE_INTERNAL);
    LPTIM->CFG &=~(LPTIM_CFG_NENC|LPTIM_CFG_ENC);
    LPTIM_Encoder_Mode_Enable(LPTIM);
    /* ENC MODE1 */
    LPTIM_Encoder_Mode_Set(LPTIM,LPTIM_ENCODER_MODE_RISING);

    LPTIM_ON(LPTIM);
    LPTIM_Auto_Reload_Set(LPTIM,15000);
    LPTIM_Compare_Set(LPTIM,10000);    
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
*\*\name    PWM_Out.
*\*\fun     Great square waves.(PA1)
*\*\param   count    the number of waves.
*\*\return  none
**/
void PWM_Out(uint16_t Pin,uint8_t cnt)
{
    uint8_t i;
    for (i = 0; i< cnt;i++)
    {
        GPIO_Pins_Set(GPIOA, Pin);
        delay(10);
        GPIO_PBSC_Pins_Reset(GPIOA, Pin);
        delay(10);
    }
}

