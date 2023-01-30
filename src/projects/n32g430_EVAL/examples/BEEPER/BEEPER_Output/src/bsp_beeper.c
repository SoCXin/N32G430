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
*\*\file bsp_beeper.c
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
**/

#include "bsp_beeper.h"
#include "bsp_delay.h"
#include "log.h"
#include "n32g430_beeper.h"

/**
 *\*\name   BEEPER_GPIO_Config.
 *\*\fun    BEEPER's GPIO configuration.
 *\*\param  none.
 *\*\return none.
**/
static void BEEPER_GPIO_Config(void)
{
	GPIO_InitType GPIO_InitStructure;
	
	/* Turn on the port clock of the GPIO that controls the beeper */
	RCC_AHB_Peripheral_Clock_Enable(BEEPER1_GPIO_CLK | BEEPER2_GPIO_CLK);
	
	GPIO_Structure_Initialize(&GPIO_InitStructure);
	
	GPIO_InitStructure.Pin            = BEEPER1_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStructure.GPIO_Current   = GPIO_DS_4MA;
	GPIO_InitStructure.GPIO_Alternate = GPIO_AF12_BEEPER;
	GPIO_Peripheral_Initialize(BEEPER1_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.Pin            = BEEPER2_GPIO_PIN;
	GPIO_InitStructure.GPIO_Alternate = GPIO_AF12_BEEPER;
	GPIO_Peripheral_Initialize(BEEPER2_GPIO_PORT, &GPIO_InitStructure);
}

/**
*\*\name    BEEPER_Normal_Work.
*\*\fun     BEEPER normal operating mode and complementary mode configuration.
*\*\param   clock_selection:  
*\*\          - BEEPER_CLK_SOURCE_APB 
*\*\          - BEEPER_CLK_SOURCE_LSI 
*\*\          - BEEPER_CLK_SOURCE_LSE
*\*\param   prescale_factor:  if LSI or LSE clock is selected, this parameter is ignored
*\*\          - 0 ~ 63  if prescale_factor > 0 and prescale_factor < 64, the APB clock frequency is divided by (prescale_factor + 1)  
*\*\param   div_factor:  
*\*\          - 0 ~ 511  if div_factor is x, output frequency value divided by (x + 1) * 2
*\*\return  none. 
**/
void BEEPER_Normal_Work(uint32_t clock_selection, uint32_t prescale_factor, uint32_t div_factor)
{
	/* Enable BEEPER clock */
	RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_BEEPER);
	
	BEEPER_GPIO_Config();
	
	/* BEEPER initialization */
	BEEPER_Initialize(clock_selection, prescale_factor, div_factor);
	
	/* Enable complementary output */
	BEEPER_Inverted_Enable();
	
	/* Enable BEEPER */
	BEEPER_Enable();
	
	/* Delay 5s */
	SysTick_Delay_Ms(5000);	
	
	/* Disable complementary output */
	BEEPER_Inverted_Disable();
	
	/* Disable BEEPER */
	BEEPER_Disable();
	
	log_info("The BEEPER output frequency is %dHz.\n", (64000000 / 32 / 1024));
}

