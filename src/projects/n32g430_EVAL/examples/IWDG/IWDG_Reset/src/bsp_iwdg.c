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
*\*\file bsp_iwdg.c
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
**/

#include "bsp_iwdg.h"

/**
 *\*\name   IWDG_Config.
 *\*\fun    Configure IWDG.
 *\*\param   IWDG_prescaler :
 *\*\          - IWDG_CONFIG_PRESCALER_DIV4 
 *\*\          - IWDG_CONFIG_PRESCALER_DIV8
 *\*\          - IWDG_CONFIG_PRESCALER_DIV16
 *\*\          - IWDG_CONFIG_PRESCALER_DIV32
 *\*\          - IWDG_CONFIG_PRESCALER_DIV64
 *\*\          - IWDG_CONFIG_PRESCALER_DIV128
 *\*\          - IWDG_CONFIG_PRESCALER_DIV256
 *\*\param   reload_value :
 *\*\          -0x000 ~ 0xFFF
**/
void IWDG_Config(IWDG_CONFIG_PRESCALER IWDG_prescaler, uint16_t reload_value)
{
	/* The timeout may varies due to LSI frequency dispersion */
	/* Disable write protection to IWDG_PREDIV and IWDG_RELV registers */
	IWDG_Write_Protection_Disable();
	
	/* IWDG counter clock */
	IWDG_Prescaler_Division_Set(IWDG_prescaler);
	
	/* Sets IWDG reload value */
	/* Set counter reload value to obtain x second IWDG TimeOut.
     Counter Reload Value Time = x(second)/IWDG counter clock period
                               = x(second) / (LSI/IWDG_prescaler)
    */
	IWDG_Counter_Reload(reload_value);
	
	/* Reload counter with IWDG_PREDIV value in IWDG_RELV register to prevent reset */
	IWDG_Key_Reload();
	
	/* Enable IWDG (the LSI oscillator will be enabled by hardware) */
	IWDG_Enable();
}

/**
 *\*\name    IWDG_Feed.
 *\*\fun     Feed the dog.
 *\*\param   none
 *\*\return  none
**/
void IWDG_Feed(void)
{
	/* Put the value of the reload register into the counter */
	IWDG_Key_Reload();
}

