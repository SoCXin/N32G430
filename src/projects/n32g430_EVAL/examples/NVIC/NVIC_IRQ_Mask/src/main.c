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
#include "log.h"
#include <stdio.h>

uint8_t Key_Status = DISABLE;

/**
*\*\name   main
*\*\fun    Main program.
*\*\return none
**/
int main(void)
{    
    /* log Init */
    log_init();
    log_info("NVIC IRQ Mask \r\n");

    /* TIM configuration */
    TIM_Config();
    /* KEY configuration */
    KeyInputExtiInit(KEY_INPUT_PORT, KEY_INPUT_PIN);

    while (1)
    {
        while (Key_Status == DISABLE)
        {
        }
        log_info("Disable irq \r\n");
        /* This instruction raises the execution priority to 0. This prevents all
           exceptions with configurable priority from activating, other than through
           the HardFault fault escalation mechanism. */
        __disable_irq();

        while(GPIO_Input_Pin_Data_Get(GPIOA,GPIO_PIN_0) == RESET)
        {
        
        }
        while(GPIO_Input_Pin_Data_Get(GPIOA,GPIO_PIN_0) == SET)
        {
        
        }

        log_info("enable irq \r\n");
        /* This instruction will allow all exceptions with configurable priority to
           be activated. */

        __enable_irq();
    }
  
}

/**
*\*\name   KeyInputExtiInit
*\*\fun    External key interrupt configuration.
*\*\return none
**/
void KeyInputExtiInit(GPIO_Module* GPIOx, uint16_t Pin)
{
    GPIO_InitType GPIO_InitStructure;
    EXTI_InitType EXTI_InitStructure;
    NVIC_InitType NVIC_InitStructure;

    /* Enable the GPIO Clock */
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOA);
	RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_AFIO);


    /*Configure the GPIO pin as input floating*/
    GPIO_InitStructure.Pin        = Pin;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_INPUT;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_FAST;
    GPIO_Peripheral_Initialize(GPIOx, &GPIO_InitStructure);

    /*Configure key EXTI Line to key input Pin*/
    GPIO_EXTI_Line_Set(KEY_INPUT_PORT_SOURCE, KEY_INPUT_PIN_SOURCE);

    /*Configure key EXTI line*/
    EXTI_InitStructure.EXTI_Line    = KEY_INPUT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Peripheral_Initializes(&EXTI_InitStructure);

    /*Set key input interrupt priority*/
    NVIC_InitStructure.NVIC_IRQChannel                   = KEY_INPUT_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);
}

/**
*\*\name   KeyInputExtiInit
*\*\fun    TIM interrupt configuration.
*\*\return none
**/
void TIM_Config(void)
{
    TIM_TimeBaseInitType TIM_TimeBaseStructure;
    OCInitType TIM_OCInitStructure;
    NVIC_InitType NVIC_InitStructure;

    /* Enable TIM2 clocks */
    RCC_APB1_Peripheral_Clock_Enable(RCC_APB1_PERIPH_TIM2);

    /* TIM2 configuration */
    TIM_TimeBaseStructure.Period    = 0x4AF;
    TIM_TimeBaseStructure.Prescaler = ((SystemClockFrequency / 1200) - 1);
    TIM_TimeBaseStructure.ClkDiv    = 0x0;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;
    TIM_Base_Initialize(TIM2, &TIM_TimeBaseStructure);
    TIM_Output_Channel_Struct_Initialize(&TIM_OCInitStructure);

    /* Output Compare Timing Mode configuration: Channel1 */
    TIM_OCInitStructure.OcMode = TIM_OCMODE_TIMING;
    TIM_OCInitStructure.Pulse  = 0x0;
    TIM_Output_Channel_Struct_Initialize(&TIM_OCInitStructure);

    /* Immediate load of TIM2 Precaler values */
    TIM_Base_Prescaler_Set(TIM2, ((SystemClockFrequency / 1200) - 1));
    TIM_Base_Reload_Mode_Set(TIM2, TIM_PSC_RELOAD_MODE_IMMEDIATE);

    /* Clear TIM2 update pending flags */
    TIM_Flag_Clear(TIM2, TIM_FLAG_UPDATE);

    /* Configure two bits for preemption priority */
    NVIC_Priority_Group_Set(NVIC_PER2_SUB2_PRIORITYGROUP);

    /* Enable the TIM2 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);

    /* Enable TIM2 Update interrupts */
    TIM_Interrupt_Enable(TIM2, TIM_INT_UPDATE);

    /* TIM2 enable counters */
    TIM_On(TIM2);
}

/**
*\*\name    EXTI0_IRQHandler.
*\*\fun     This function handles EXTI0 exception.
*\*\param   none
*\*\return  none 
**/
void EXTI0_IRQHandler(void)
{
    printf("EXTI0 IRQHandler Start \r\n");
    if (EXTI_Interrupt_Status_Get(KEY_INPUT_EXTI_LINE) != RESET)
    {
        if (Key_Status == ENABLE)
        {
            Key_Status = DISABLE;
        }
        else
        {
            Key_Status = ENABLE;
        }
        /* Clears the SEL Button EXTI line pending bits. */
        EXTI_Flag_Status_Clear(KEY_INPUT_EXTI_LINE);
    }
    log_info("EXTI0 IRQHandler End \r\n");
}

/**
*\*\name    DebugMon_Handler.
*\*\fun     This function handles TIM2 exception.
*\*\param   none
*\*\return  none 
**/
void TIM2_IRQHandler(void)
{
    /* Clear TIM2 update interrupt */
    TIM_Flag_Clear(TIM2, TIM_INT_UPDATE);

    /* TIM2 IRQHandler */
    log_info("TIM2 IRQHandler \r\n");
}
