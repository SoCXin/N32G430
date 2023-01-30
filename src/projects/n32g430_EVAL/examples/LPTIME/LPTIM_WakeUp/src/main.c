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

/* LPTIM_WakeUP */

void LedBlink(GPIO_Module* GPIOx, uint16_t Pin);
void LEDInit(uint16_t Pin);
void LedOn(uint16_t Pin);
void LedOff(uint16_t Pin);
void Ledlink(uint16_t Pin);
void delay(vu32 nCount);
void LPTIMNVIC_Config(FunctionalState Cmd);
void SYSCLKConfig(uint32_t RCC_PLLMULL);
/* Main program. */
int main(void)
{
    /* At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_n32g430.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_n32g430.c file
     */
    /* Enable PWR Clock */
    RCC_APB1_Peripheral_Clock_Enable(RCC_APB1_PERIPH_PWR);
    /* Init LED GPIO */
    LEDInit(LED1);
    /* Enable the LSI source */
    RCC_LPTIM_Enable();
    LPTIMNVIC_Config(ENABLE);
    RCC_LPTIM_Clock_Config(RCC_LPTIMCLK_SRC_LSI); 
    RCC_LSI_Enable();
    /* Enable interrupt   */
    LPTIM_Prescaler_Set(LPTIM,LPTIM_PRESCALER_DIV1);
    LPTIM_Interrupt_Enable(LPTIM, LPTIM_INT_CMPMIE);
    /* config lptim ARR and compare register */ 
    LPTIM_ON(LPTIM);   
    LPTIM_Auto_Reload_Set(LPTIM,65000);
    LPTIM_Compare_Set(LPTIM,60000);
    LPTIM_Counter_Start(LPTIM,LPTIM_OPERATING_MODE_CONTINUOUS);
    DBG_Peripheral_ON(DBG_STOP);
    while (1)
    {
        //PWR_SLEEP_Mode_Enter(PWR_SLEEP_NOW, PWR_SLEEP_ENTRY_WFI); 
        PWR_STOP2_Mode_Enter(PWR_STOP2_ENTRY_WFI);
        Ledlink(LED1);
        delay(30);
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
*\*\name    LPTIMNVIC_Config.
*\*\fun     LPTIM Interrupt Initaliza.
*\*\param   Cmd         enable or disable
*\*\return  none
**/
void LPTIMNVIC_Config(FunctionalState Cmd)
{
    EXTI_InitType EXTI_InitStructure;
    NVIC_InitType NVIC_InitStructure;

    EXTI_Flag_Status_Clear(EXTI_LINE20);
    EXTI_InitStructure.EXTI_Line = EXTI_LINE20;
#ifdef __TEST_SEVONPEND_WFE_NVIC_DIS__
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Event;
#else
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
#endif
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Peripheral_Initializes(&EXTI_InitStructure);

    /* Enable the RTC Alarm Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = LPTIM_WKUP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PER_PRIORITY_1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = NVIC_SUB_PRIORITY_1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = Cmd;
    NVIC_Initializes(&NVIC_InitStructure);
}

/**
*\*\name    SYSCLKConfig.
*\*\fun     Configures system clock after wake-up from low power mode:
*\*\            enable HSE, PLL and select PLL as system clock source.
*\*\param   RCC_PLLMULL         
*\*\return  none
**/
void SYSCLKConfig(uint32_t RCC_PLLMULL)
{
    __IO uint32_t StartUpCounter = 0, HSEStartUpStatus = 0;

    /* reset RCC with the default values */
    RCC_Reset();

    RCC_HSI_Enable();
    HSEStartUpStatus = RCC_HSE_Stable_Wait();

    if (HSEStartUpStatus == SUCCESS)
    {
      /* The following two steps are required to operate flash */
      FLASH_Prefetch_Buffer_Enable();        
      FLASH_Latency_Set(FLASH_LATENCY_2);

      /* set different clock trees frequency division */
      RCC_Hclk_Config(RCC_SYSCLK_DIV1);        
      RCC_Pclk2_Config(RCC_HCLK_DIV2); 
      RCC_Pclk1_Config(RCC_HCLK_DIV4);

      /* the main frequence */
      /* set PLLclock resource from HSE and the PLL factor */
      /* PLLCLK = 8MHz * pllmul */
      RCC_PLL_Config(RCC_PLL_SRC_HSE_DIV2, RCC_PLLMULL);

      RCC_PLL_Enable();
      while (RCC_Flag_Status_Get(RCC_FLAG_PLLRD) == RESET)
      {
      }
      
      RCC_Sysclk_Config(RCC_SYSCLK_SRC_PLLCLK);
      while (RCC_Sysclk_Source_Get() != 0x0C)
      {
      }
    }
    else
    { 
        /* if HSE enable to fail ,the process will come here.Then the MCU clock resoure is HSI */
        while(1)
        {}
    }
}
