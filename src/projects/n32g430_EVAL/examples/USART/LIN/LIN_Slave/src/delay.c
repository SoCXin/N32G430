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
*\*\file delay.c
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
**/
#include "delay.h"

/**
*\*\name    systick_delay_us.
*\*\fun     us delay  program function.
*\*\param   nus
*\*\return  none
**/
void systick_delay_us(u32 nus)
{       
    u32 temp;       
    SysTick_Clock_Source_Set(SYSTICK_CLKSOURCE_HCLK);    //select system clock   
    SysTick->LOAD=nus*(SystemClockFrequency/1000000); //time relode           
    SysTick->VAL=0x00;        //clear timer value
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;  //Start countdown  
    do
    {
        temp=SysTick->CTRL;
    }
    while(temp&0x01&&!(temp&(1<<16)));//wait for the time reach 
    SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //close the count
    SysTick->VAL =0X00;       //clear timer value    
}

/**
*\*\name    systick_delay_ms.
*\*\fun     ms delay  program function.
*\*\param   nms
*\*\return  none
**/
void systick_delay_ms(u16 nms)
{                 
    u32 temp;
    SysTick_Clock_Source_Set(SYSTICK_CLKSOURCE_HCLK);  //select system clock   
    SysTick->LOAD=(u32)nms*((SystemClockFrequency/1000000)*1000);//time relode(SysTick->LOAD is 24bit)
    SysTick->VAL =0x00;           //clear timer value
    SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //Start countdown  
    do
    {
        temp=SysTick->CTRL;
    }
    while(temp&0x01&&!(temp&(1<<16)));//wait for the time reach    
    SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //close the count
    SysTick->VAL =0X00;       //clear timer value               
} 
