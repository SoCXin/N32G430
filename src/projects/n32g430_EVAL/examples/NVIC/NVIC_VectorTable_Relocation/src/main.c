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

/**
**  NVIC VectorTable Relocation
**/

__IO uint8_t Key_Status = DISABLE;

#define SP_PROCESS_SIZE          0x200 /* Process stack size */
#define SP_PROCESS               0x02  /* Process stack */
#define SP_MAIN                  0x00  /* Main stack */
#define THREAD_MODE_PRIVILEGED   0x00  /* Thread mode has privileged access */
#define THREAD_MODE_UNPRIVILEGED 0x01  /* Thread mode has unprivileged access */


/**
*\*\name   main
*\*\fun    Main program.
*\*\return none
**/
int main(void)
{    
    /* log Init */
    log_init();
    log_info("NVIC VectorTable Relocation \r\n");

    KeyInputExtiInit(KEY_INPUT_PORT, KEY_INPUT_PIN);

    while (1)
    {
        if (Key_Status == ENABLE)
        {
            log_info("Sets the vector table location and Offset \r\n");

            /* Vector Table Relocation in Internal SRAM. */
            NVIC_Vector_Table_Set(NVIC_VECTTAB_RAM, VECT_TAB_SRAM_OFFSET);

            while (1)
                ;
        }
    }
  
}

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
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);
}

void EXTI0_IRQHandler(void)
{
    printf("EXTI0 IRQHandler Start \r\n");
    if (EXTI_Interrupt_Status_Get(KEY_INPUT_EXTI_LINE) != RESET)
    {
#ifdef VECT_TAB_SRAM
        log_info("EXTI0 IRQHandler is executed(SRAM) \r\n");
#else
        /* Key_Status */
        Key_Status = ENABLE;
        log_info("EXTI0 IRQHandler is executed(FLASH) \r\n");
#endif
        /* Clears the SEL Button EXTI line pending bits */
        EXTI_Flag_Status_Clear(KEY_INPUT_EXTI_LINE);
    }
    log_info("EXTI0 IRQHandler End \r\n");
}


