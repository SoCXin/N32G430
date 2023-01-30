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
*\*\file log.c
*\*\author Nations
*\*\version v1.0.0
*\*\copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
**/

#include "log.h"
#include "n32g430.h"
#include "n32g430_gpio.h"
#include "n32g430_usart.h"
#include "n32g430_rcc.h"

#define LOG_USARTx        USART1
#define LOG_PERIPH        RCC_APB2_PERIPH_USART1
#define LOG_GPIO          GPIOA
#define LOG_PERIPH_GPIO   RCC_AHB_PERIPH_GPIOA
#define LOG_TX_PIN        GPIO_PIN_9
#define LOG_RX_PIN        GPIO_PIN_10

/**
 *\*\name   log_init.
 *\*\fun    USART initialize.
 *\*\param  none.
 *\*\return none.
**/
void log_init(void)
{
    GPIO_InitType GPIO_InitStructure;
    USART_InitType USART_InitStructure;

    RCC_AHB_Peripheral_Clock_Enable(LOG_PERIPH_GPIO);
    RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_AFIO);
  	RCC_APB2_Peripheral_Clock_Enable(LOG_PERIPH);

    GPIO_Structure_Initialize(&GPIO_InitStructure);
	
    GPIO_InitStructure.Pin            = LOG_TX_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF5_USART1;
    GPIO_Peripheral_Initialize(LOG_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.Pin            = LOG_RX_PIN;
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;
  	GPIO_InitStructure.GPIO_Alternate = GPIO_AF5_USART1;
    GPIO_Peripheral_Initialize(LOG_GPIO, &GPIO_InitStructure);

    USART_InitStructure.BaudRate            = 115200;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_TX | USART_MODE_RX;
    USART_Initializes(LOG_USARTx, &USART_InitStructure);
    
    /* Enable USART */
    USART_Enable(LOG_USARTx);
}


/**
 *\*\name   Usart_SendByte.
 *\*\fun    USART sends one byte of data.
 *\*\param  USARTx :
 *\*\          - USART1
 *\*\          - USART2
 *\*\param  byte :
 *\*\          - A byte of data to be sent
 *\*\return none.
**/
void Usart_SendByte(USART_Module* USARTx, uint8_t byte)
{
	USART_Data_Send(USARTx, byte);
	
    while (USART_Flag_Status_Get(USARTx, USART_FLAG_TXC) == RESET)
    {
    }
}


/**
 *\*\name   Usart_SendString.
 *\*\fun    USART sends a string.
 *\*\param  USARTx :
 *\*\          - USART1
 *\*\          - USART2
 *\*\param  str :
 *\*\          - The string to send
 *\*\return none.
**/
void Usart_SendString(USART_Module* USARTx, char *str)
{
	unsigned int k=0;
	
    do 
    {
       Usart_SendByte(USARTx, *(str + k) );
       k++;
    } while(*(str + k)!='\0');
  
    while(USART_Flag_Status_Get(USARTx, USART_FLAG_TXC) == RESET)
    {}
}


/**
 *\*\name   fputc.
 *\*\fun    Rewrite the C library printf function.
 *\*\param  
 *\*\return none.
**/
int fputc(int ch, FILE* f)
{
    /* Send a byte of data to the serial port */
	USART_Data_Send(LOG_USARTx, (uint8_t) ch);
		
	/* Waiting for sending */
	while (USART_Flag_Status_Get(LOG_USARTx, USART_FLAG_TXDE) == RESET);		
	
	return (ch);
}


/**
 *\*\name   fgetc.
 *\*\fun    Rewrite the C library scanf function.
 *\*\param  
 *\*\return none.
**/
int fgetc(FILE* f)
{
    /* Waiting for usart port input data */
    while (USART_Flag_Status_Get(LOG_USARTx, USART_FLAG_RXDNE) == RESET);

    return (int)USART_Data_Receive(LOG_USARTx);
}


