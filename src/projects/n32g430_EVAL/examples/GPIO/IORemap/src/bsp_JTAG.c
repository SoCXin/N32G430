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
*\*\file bsp_JTAG.c
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
**/


#include "bsp_JTAG.h"

/**
 *\*\name   JTAG_Function_Initialize.
 *\*\fun    Configures as JTAG function.
 *\*\param  none.
 *\*\return none.
**/
void JTAG_Function_Initialize(void)
{
	/* Define a structure of type GPIO_InitType */
	GPIO_InitType GPIO_InitStructure;
	
	RCC_AHB_Peripheral_Clock_Enable(JTMS_CLK | JTRST_CLK);
	
	/* Assign default value to GPIO_InitStructure structure */
	GPIO_Structure_Initialize(&GPIO_InitStructure);
	
	/* Configure PA13 (JTMS) and PA15 (JTDI) as alternate output push-pull and pull-up */
	GPIO_InitStructure.Pin            = JTMS_PIN | JTDI_PIN;
	GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;
	GPIO_InitStructure.GPIO_Alternate = GPIO_AF0_JTDI;
	/* Initialize GPIO */
	GPIO_Peripheral_Initialize(JTMS_PORT, &GPIO_InitStructure);
	
	/* Configure PB4 (JTRST) as alternate output push-pull and pull-up */
	GPIO_InitStructure.Pin            = JTRST_PIN;
	GPIO_Peripheral_Initialize(JTRST_PORT, &GPIO_InitStructure);
	
	/* Configure PA14 (JTCK) as alternate output push-pull and pull-down */
	GPIO_InitStructure.Pin            = JTCLK_PIN;
	GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_DOWN;
	GPIO_Peripheral_Initialize(JTCLK_PORT, &GPIO_InitStructure);
	
	/* Configure PB3 (JTDO) as alternate output push-pull */
	GPIO_InitStructure.Pin            = JTDO_PIN;
	GPIO_InitStructure.GPIO_Pull      = GPIO_NO_PULL;
	GPIO_Peripheral_Initialize(JTDO_PORT, &GPIO_InitStructure);
}


/**
 *\*\name   JTAG_As_GPIO_Initialize.
 *\*\fun    Configures JTAG as GPIO
 *\*\param  none.
 *\*\return none.
**/
void JTAG_As_GPIO_Initialize(void)
{
	/* Define a structure of type GPIO_InitType */
	GPIO_InitType GPIO_InitStructure;
	
	RCC_AHB_Peripheral_Clock_Enable(JTMS_CLK | JTRST_CLK);
	
	/* Assign default value to GPIO_InitStructure structure */
	GPIO_Structure_Initialize(&GPIO_InitStructure);
	
	/* Configure PA13 (JTMS) and PA15 (JTDI) as gpio */
	GPIO_InitStructure.Pin            = JTMS_PIN | JTDI_PIN;
	GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_OUT_PP;
	/* Initialize GPIO */
	GPIO_Peripheral_Initialize(JTMS_PORT, &GPIO_InitStructure);
	
	/* Configure PB4 (JTRST) as gpio */
	GPIO_InitStructure.Pin            = JTRST_PIN;
	GPIO_Peripheral_Initialize(JTRST_PORT, &GPIO_InitStructure);
	
	/* Configure  PA14 (JTCK) as gpio */
	GPIO_InitStructure.Pin            = JTCLK_PIN;
	GPIO_Peripheral_Initialize(JTCLK_PORT, &GPIO_InitStructure);
	
	/* Configure PB3 (JTDO) as gpio */
	GPIO_InitStructure.Pin            = JTDO_PIN;
	GPIO_Peripheral_Initialize(JTDO_PORT, &GPIO_InitStructure);
}



